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

void devregs_init(ModbusSlave_t *mb)
{
    /* 固件版本 */
    Modbus_SetReg(mb, REG_FW_VER,        FIRMWARE_VERSION);

    /* 报警阈值出厂默认 */
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
    Modbus_SetReg(mb, REG_WATCHDOG_S,     0);   /* 看门狗默认关闭 */

    /* 设置只读区: [0x0000, DEV_READONLY_END) 主机不可写 */
    Modbus_SetReadonly(mb, DEV_READONLY_END);
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
            /* 有新帧 → 更新看门狗时间戳 */
            static uint32_t s_prev_frame = 0;
            if (mb->frame_count != s_prev_frame) {
                s_prev_frame   = mb->frame_count;
                s_last_frame_seen = tick_ms;
            }
        }
        /* 超时未收到任何帧 → 软件复位 */
        if ((tick_ms - s_last_frame_seen) > (uint32_t)wdog_s * 1000u) {
            NVIC_SystemReset();
        }
    }
}
