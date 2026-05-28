/**
 * @file  device_regs.c
 * @brief 4 通道工业电压采集模块 — 寄存器实现
 *
 * 本文件模拟真实设备的行为:
 *   - 4 路 ADC 以不同频率和幅度的正弦波变化, 模拟工业现场的缓变信号
 *   - 均值滤波: 对 avg_count 次采样取平均, 模拟硬件过采样降噪
 *   - 报警检测: 每次更新后与主机写入的阈值比较, 置位状态字对应 bit
 *   - 看门狗: 若配置了超时, 超过该时间未收到任何 MODBUS 帧则触发软件复位
 *
 * 注: 在真实硬件上, devregs_update() 中的正弦/余弦计算应替换为
 *     HAL_ADC_GetValue() 或 DMA 均值结果。本文件是协议层和驱动层的
 *     "粘合代码", 注释保留了替换点以便在真实项目中快速集成。
 */

#include "device_regs.h"
#include "stm32f1xx.h"
#include <math.h>
#include <string.h>

/* 出厂默认值 */
#define DEFAULT_HI_THRESHOLD   9000u    /* mV */
#define DEFAULT_LO_THRESHOLD    500u    /* mV */
#define DEFAULT_SAMPLE_INTV_MS  100u    /* ms */
#define DEFAULT_AVG_COUNT         8u
#define FIRMWARE_VERSION       0x0200u  /* v2.0 */

/* 内部状态: 上次更新时间 + 看门狗相关 */
static uint32_t s_last_update_ms  = 0;
static uint32_t s_last_frame_seen = 0; /* 上次看到帧时的 tick_ms */

/* ── 仿真 ADC: 4 通道正弦波参数 ────────────────────────────
 * 模拟一个典型工业场景:
 *   CH1: 主工艺量, 4500 ± 4000 mV, 慢变化 (0.1 Hz)
 *   CH2: 辅助量,   7000 ± 2000 mV, 中速   (0.23 Hz)
 *   CH3: 状态量,   2000 ± 1500 mV, 快变   (0.5 Hz, 余弦)
 *   CH4: 参考量,   5000 ± 100  mV, 微抖动 (2 Hz) — 偶尔触发低阈值报警
 */
typedef struct { float center; float amp; float freq_hz; uint8_t cos_wave; } SimCh_t;

static const SimCh_t k_sim[4] = {
    { 4500.0f, 4000.0f, 0.10f, 0 },
    { 7000.0f, 2000.0f, 0.23f, 0 },
    { 2000.0f, 1500.0f, 0.50f, 1 },
    { 5000.0f,  100.0f, 2.00f, 0 },
};

/* 将 float 电压 (mV) 限幅到 uint16_t 0~10000 */
static uint16_t clamp_mv(float v)
{
    if (v < 0.0f)     return 0;
    if (v > 10000.0f) return 10000;
    return (uint16_t)v;
}

/* 仿真单通道电压 (mV), t_sec = 系统运行秒数 (float) */
static uint16_t sim_channel_mv(uint8_t ch, float t_sec)
{
    const SimCh_t *p = &k_sim[ch];
    float omega = 2.0f * 3.14159265f * p->freq_hz;
    float val   = p->center + p->amp * (p->cos_wave ? cosf(omega * t_sec)
                                                     : sinf(omega * t_sec));
    return clamp_mv(val);
}

/* 仿真 MCU 温度: 25 °C 基准 + ±3 °C 低频漂移, 单位 °C×10 */
static uint16_t sim_mcu_temp(float t_sec)
{
    return (uint16_t)(250.0f + 30.0f * sinf(0.02f * t_sec));
}

/* ─────────────────────────────────────────────────────────────────
 * Flash 配置存储（页 63，0x0800FC00，1 KB）
 *
 * 数据布局（所有字段小端存储）：
 *   偏移  大小  含义
 *   0x00   4B   magic = 0xA5A5CDEF（有效标志）
 *   0x04   2B   version（配置格式版本，当前 = 1）
 *   0x06   2B   保留
 *   0x08  16B   阈值 ch1_hi/ch2_hi/ch3_hi/ch4_hi/ch1_lo…ch4_lo
 *   0x18   6B   采集配置 sample_intv_ms / avg_count / watchdog_s
 *   0x1E   2B   CRC16-CCITT（覆盖 0x00 ~ 0x1D 共 30 字节）
 *
 * CRC16-CCITT（poly=0x1021，init=0xFFFF）用于检测 Flash 数据损坏。
 * ──────────────────────────────────────────────────────────────── */

#define CFG_VERSION        1u
#define CFG_DATA_LEN       30u    /* magic + ver + rsv + thresholds + config */
#define CFG_TOTAL_LEN      32u    /* CFG_DATA_LEN + CRC16(2B) */

/* Flash 解锁魔数 */
#define FLASH_KEY1  0x45670123UL
#define FLASH_KEY2  0xCDEF89ABUL

/* CRC16-CCITT（与 Xmodem 块验证使用相同算法）*/
static uint16_t cfg_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8u);
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1u) ^ 0x1021u)
                                  : (uint16_t)(crc << 1u);
        }
    }
    return crc;
}

/* Flash 写一个半字（要求已解锁，目标地址已擦除）*/
static int flash_write_hw(uint32_t addr, uint16_t data)
{
    uint32_t timeout = 0x100000u;
    while ((FLASH->SR & FLASH_SR_BSY) && --timeout);
    if (!timeout) return 0;

    FLASH->CR |= FLASH_CR_PG;
    *(__IO uint16_t *)addr = data;
    timeout = 0x100000u;
    while ((FLASH->SR & FLASH_SR_BSY) && --timeout);
    FLASH->CR &= ~FLASH_CR_PG;
    if (FLASH->SR & FLASH_SR_EOP) FLASH->SR = FLASH_SR_EOP;
    return (*(volatile uint16_t *)addr == data) ? 1 : 0;
}

void devregs_save_config(ModbusSlave_t *mb)
{
    /* 构建配置字节数组 */
    uint8_t buf[CFG_TOTAL_LEN];
    memset(buf, 0, sizeof(buf));

    uint32_t magic = CFG_FLASH_MAGIC;
    memcpy(buf + 0,  &magic,        4u);
    buf[4] = (uint8_t)CFG_VERSION;
    buf[5] = 0; buf[6] = 0; buf[7] = 0;   /* version hi + reserved */

    /* 阈值 8 × 2 bytes */
    for (uint8_t i = 0; i < 4u; i++) {
        uint16_t v = Modbus_GetReg(mb, REG_CH1_HI_THR + i);
        buf[8  + i * 2u]     = (uint8_t)(v & 0xFFu);
        buf[8  + i * 2u + 1] = (uint8_t)(v >> 8u);
    }
    for (uint8_t i = 0; i < 4u; i++) {
        uint16_t v = Modbus_GetReg(mb, REG_CH1_LO_THR + i);
        buf[16 + i * 2u]     = (uint8_t)(v & 0xFFu);
        buf[16 + i * 2u + 1] = (uint8_t)(v >> 8u);
    }

    /* 采集配置 3 × 2 bytes */
    uint16_t intv = Modbus_GetReg(mb, REG_SAMPLE_INTV_MS);
    uint16_t avgn = Modbus_GetReg(mb, REG_AVG_COUNT);
    uint16_t wdog = Modbus_GetReg(mb, REG_WATCHDOG_S);
    buf[24] = (uint8_t)(intv & 0xFFu); buf[25] = (uint8_t)(intv >> 8u);
    buf[26] = (uint8_t)(avgn & 0xFFu); buf[27] = (uint8_t)(avgn >> 8u);
    buf[28] = (uint8_t)(wdog & 0xFFu); buf[29] = (uint8_t)(wdog >> 8u);

    /* 追加 CRC16 */
    uint16_t crc = cfg_crc16(buf, CFG_DATA_LEN);
    buf[CFG_DATA_LEN]     = (uint8_t)(crc & 0xFFu);
    buf[CFG_DATA_LEN + 1] = (uint8_t)(crc >> 8u);

    /* 解锁 Flash */
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
    if (FLASH->CR & FLASH_CR_LOCK) {
        /* 解锁失败：在状态字 bit[15] 设置错误标志 */
        Modbus_SetReg(mb, REG_STATUS, Modbus_GetReg(mb, REG_STATUS) | 0x8000u);
        return;
    }

    /* 擦除配置页 */
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = CFG_FLASH_PAGE_ADDR;
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_PER;
    if (FLASH->SR & FLASH_SR_EOP) FLASH->SR = FLASH_SR_EOP;

    /* 写入数据（半字对齐）*/
    int ok = 1;
    for (uint16_t i = 0; i < CFG_TOTAL_LEN; i += 2u) {
        uint16_t hw = (uint16_t)buf[i] | ((uint16_t)buf[i + 1] << 8u);
        if (!flash_write_hw(CFG_FLASH_PAGE_ADDR + i, hw)) { ok = 0; break; }
    }

    FLASH->CR |= FLASH_CR_LOCK;

    /* 保存失败时设置错误标志 */
    if (!ok) {
        Modbus_SetReg(mb, REG_STATUS, Modbus_GetReg(mb, REG_STATUS) | 0x8000u);
    }
}

int devregs_load_config(ModbusSlave_t *mb)
{
    const uint8_t *p = (const uint8_t *)CFG_FLASH_PAGE_ADDR;

    /* 验证魔数 */
    uint32_t magic;
    memcpy(&magic, p, 4u);
    if (magic != CFG_FLASH_MAGIC) return 0;

    /* 验证 CRC16 */
    uint16_t stored_crc;
    memcpy(&stored_crc, p + CFG_DATA_LEN, 2u);
    uint16_t calc_crc = cfg_crc16(p, CFG_DATA_LEN);
    if (stored_crc != calc_crc) return 0;

    /* 加载报警阈值 */
    for (uint8_t i = 0; i < 4u; i++) {
        uint16_t v;
        memcpy(&v, p + 8  + i * 2u, 2u);
        Modbus_SetReg(mb, REG_CH1_HI_THR + i, v);
        memcpy(&v, p + 16 + i * 2u, 2u);
        Modbus_SetReg(mb, REG_CH1_LO_THR + i, v);
    }

    /* 加载采集配置 */
    uint16_t intv, avgn, wdog;
    memcpy(&intv, p + 24, 2u); Modbus_SetReg(mb, REG_SAMPLE_INTV_MS, intv);
    memcpy(&avgn, p + 26, 2u); Modbus_SetReg(mb, REG_AVG_COUNT,       avgn);
    memcpy(&wdog, p + 28, 2u); Modbus_SetReg(mb, REG_WATCHDOG_S,      wdog);

    return 1;
}

void devregs_load_fault_info(ModbusSlave_t *mb)
{
    /* 使能 BKP 和 PWR 时钟访问 */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;

    if (BKP->DR1 == 0xBAD1u) {
        uint16_t pc_lo   = BKP->DR2;
        uint16_t pc_hi   = BKP->DR3;
        uint16_t cfsr_lo = BKP->DR4;
        uint16_t cfsr_hi = BKP->DR5;
        uint16_t hfsr    = BKP->DR6;
        Modbus_SetReg(mb, REG_FAULT_FLAG,    0xBAD1u);
        Modbus_SetReg(mb, REG_FAULT_PC_LO,   pc_lo);
        Modbus_SetReg(mb, REG_FAULT_PC_HI,   pc_hi);
        Modbus_SetReg(mb, REG_FAULT_CFSR_LO, cfsr_lo);
        Modbus_SetReg(mb, REG_FAULT_CFSR_HI, cfsr_hi);
        Modbus_SetReg(mb, REG_FAULT_HFSR,    hfsr);
    }
}

void devregs_init(ModbusSlave_t *mb)
{
    /* 固件版本 */
    Modbus_SetReg(mb, REG_FW_VER,        FIRMWARE_VERSION);

    /* 报警阈值出厂默认（可能被 devregs_load_config 覆盖）*/
    Modbus_SetReg(mb, REG_CH1_HI_THR,    DEFAULT_HI_THRESHOLD);
    Modbus_SetReg(mb, REG_CH2_HI_THR,    DEFAULT_HI_THRESHOLD);
    Modbus_SetReg(mb, REG_CH3_HI_THR,    DEFAULT_HI_THRESHOLD);
    Modbus_SetReg(mb, REG_CH4_HI_THR,    DEFAULT_HI_THRESHOLD);
    Modbus_SetReg(mb, REG_CH1_LO_THR,    DEFAULT_LO_THRESHOLD);
    Modbus_SetReg(mb, REG_CH2_LO_THR,    DEFAULT_LO_THRESHOLD);
    Modbus_SetReg(mb, REG_CH3_LO_THR,    DEFAULT_LO_THRESHOLD);
    Modbus_SetReg(mb, REG_CH4_LO_THR,    DEFAULT_LO_THRESHOLD);

    /* 采集配置出厂默认 */
    Modbus_SetReg(mb, REG_SAMPLE_INTV_MS, DEFAULT_SAMPLE_INTV_MS);
    Modbus_SetReg(mb, REG_AVG_COUNT,      DEFAULT_AVG_COUNT);
    Modbus_SetReg(mb, REG_WATCHDOG_S,     0u);

    /* 设置只读区: [0x0000, DEV_READONLY_END) 主机不可写 */
    Modbus_SetReadonly(mb, DEV_READONLY_END);

    /* 尝试从 Flash 恢复用户配置（覆盖上面的出厂默认值）*/
    devregs_load_config(mb);

    /* 加载上次 HardFault 诊断信息（若有）*/
    devregs_load_fault_info(mb);
}

void devregs_update(ModbusSlave_t *mb, uint32_t tick_ms)
{
    uint16_t intv_ms = Modbus_GetReg(mb, REG_SAMPLE_INTV_MS);
    if (intv_ms < 10u) {
        /* 防止主机写入非法值导致 CPU 死循环 */
        intv_ms = 10u;
        Modbus_SetReg(mb, REG_SAMPLE_INTV_MS, intv_ms);
    }

    if ((tick_ms - s_last_update_ms) < intv_ms) return;
    s_last_update_ms = tick_ms;

    float t_sec = (float)tick_ms / 1000.0f;

    /* ── 均值滤波: 对 avg_count 次正弦采样取平均 ──────────────
     * 真实项目替换点: 用 DMA 传输的 ADC 缓冲区均值代替此循环。
     */
    uint16_t avg_n = Modbus_GetReg(mb, REG_AVG_COUNT);
    if (avg_n < 1u || avg_n > 64u) avg_n = DEFAULT_AVG_COUNT;

    uint32_t acc[4] = {0};
    for (uint16_t s = 0; s < avg_n; s++) {
        float t_off = t_sec + (float)s * 0.001f; /* 模拟连续采样 */
        for (uint8_t ch = 0; ch < 4; ch++) {
            acc[ch] += sim_channel_mv(ch, t_off);
        }
    }
    uint16_t mv[4];
    for (uint8_t ch = 0; ch < 4; ch++) {
        mv[ch] = (uint16_t)(acc[ch] / avg_n);
    }

    /* ── 写入测量数据 (只读区, 只有 devregs_update 能通过内部接口写) */
    Modbus_SetReg(mb, REG_CH1_MV,    mv[0]);
    Modbus_SetReg(mb, REG_CH2_MV,    mv[1]);
    Modbus_SetReg(mb, REG_CH3_MV,    mv[2]);
    Modbus_SetReg(mb, REG_CH4_MV,    mv[3]);
    Modbus_SetReg(mb, REG_MCU_TEMP,  sim_mcu_temp(t_sec));

    uint16_t adc_cnt = Modbus_GetReg(mb, REG_ADC_COUNT);
    Modbus_SetReg(mb, REG_ADC_COUNT, (uint16_t)(adc_cnt + avg_n));

    /* ── 报警检测: 与主机配置的阈值比较 ──────────────────────── */
    uint16_t status = 0;
    uint16_t hi_base = REG_CH1_HI_THR;
    uint16_t lo_base = REG_CH1_LO_THR;
    for (uint8_t ch = 0; ch < 4; ch++) {
        uint16_t hi = Modbus_GetReg(mb, hi_base + ch);
        uint16_t lo = Modbus_GetReg(mb, lo_base + ch);
        if (mv[ch] > hi || mv[ch] < lo) {
            status |= (uint16_t)(1u << ch);
        }
    }
    Modbus_SetReg(mb, REG_STATUS, status);

    /* ── 同步 MODBUS 统计信息 ────────────────────────────────── */
    Modbus_SetReg(mb, REG_UPTIME_S,    (uint16_t)(tick_ms / 1000u));
    Modbus_SetReg(mb, REG_FRAME_COUNT, (uint16_t)(mb->frame_count & 0xFFFFu));
    Modbus_SetReg(mb, REG_LATENCY_US,  (uint16_t)(mb->last_proc_us & 0xFFFFu));
    Modbus_SetReg(mb, REG_CRC_ERR,     (uint16_t)(mb->crc_err_count & 0xFFFFu));

    /* ── 看门狗: 检查主机是否在规定时间内通信 ────────────────── */
    uint16_t wdog_s = Modbus_GetReg(mb, REG_WATCHDOG_S);
    if (wdog_s > 0u) {
        if (mb->frame_count > 0u) {
            static uint32_t s_prev_frame = 0;
            if (mb->frame_count != s_prev_frame) {
                s_prev_frame      = mb->frame_count;
                s_last_frame_seen = tick_ms;
            }
        }
        if ((tick_ms - s_last_frame_seen) > (uint32_t)wdog_s * 1000u) {
            NVIC_SystemReset();
        }
    }

    /* ── Flash 配置保存（主机写 0x5A5A 到 REG_SAVE_CONFIG 触发）── */
    if (Modbus_GetReg(mb, REG_SAVE_CONFIG) == CFG_SAVE_KEY) {
        Modbus_SetReg(mb, REG_SAVE_CONFIG, 0u);  /* 立即清零，防止重复触发 */
        devregs_save_config(mb);
    }

    /* ── HardFault 记录清除（主机写任意值到 REG_FAULT_HFSR）────── */
    if (Modbus_GetReg(mb, REG_FAULT_FLAG) == 0xBAD1u) {
        /* 检查 REG_FAULT_HFSR 是否被主机写过（由 Modbus 写操作置位）*/
        /* 简化：检测该寄存器非零时视为清除请求 */
        if (Modbus_GetReg(mb, REG_FAULT_HFSR) != 0u &&
            Modbus_GetReg(mb, REG_FAULT_FLAG)  != 0u) {
            /* 检查是否是主机写入的清除命令（非 0xBAD1 的值）*/
            /* 此处用标志寄存器 = 0 代表"已清除"请求 */
            if (Modbus_GetReg(mb, REG_FAULT_FLAG) == 0xBAD1u &&
                (tick_ms % 1000u) < intv_ms) {   /* 仅在每秒初判一次 */
                /* 清除 BKP 故障记录 */
                RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
                PWR->CR |= PWR_CR_DBP;
                BKP->DR1 = 0u;
                BKP->DR2 = 0u; BKP->DR3 = 0u;
                BKP->DR4 = 0u; BKP->DR5 = 0u; BKP->DR6 = 0u;
                /* 清除 Modbus 寄存器区的故障信息 */
                Modbus_SetReg(mb, REG_FAULT_FLAG,    0u);
                Modbus_SetReg(mb, REG_FAULT_PC_LO,   0u);
                Modbus_SetReg(mb, REG_FAULT_PC_HI,   0u);
                Modbus_SetReg(mb, REG_FAULT_CFSR_LO, 0u);
                Modbus_SetReg(mb, REG_FAULT_CFSR_HI, 0u);
                Modbus_SetReg(mb, REG_FAULT_HFSR,    0u);
            }
        }
    }
}
