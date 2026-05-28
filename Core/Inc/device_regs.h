/**
 * @file  device_regs.h
 * @brief 4 通道工业电压采集模块 — MODBUS 寄存器映射
 *
 * 模拟一个 4 通道 0~10 V 模拟输入模块
 * (类比 Phoenix Contact AXL F AI4 I 或 Wago 750-459)
 *
 * 寄存器地址表 (MODBUS 保持寄存器, FC03 读 / FC06、FC10 写):
 *
 *  地址      访问  描述
 *  ──────────────────────────────────────────────────────────
 *  0x0000    RO    状态字: bit[3:0]=CH1~CH4 超量程标志 (1=报警)
 *  0x0001    RO    固件版本: 0x0200 = v2.0
 *  0x0002    RO    运行时间 (秒, 16 位循环)
 *  0x0003    RO    MODBUS 帧计数 (低 16 位)
 *  0x0004    RO    上次响应处理延迟 (μs, 用于主机评估通信质量)
 *  0x0005    RO    CRC 错误累计
 *  ──────────────────────────────────────────────────────────
 *  0x0010    RO    CH1 电压 (mV, 0~10000)
 *  0x0011    RO    CH2 电压 (mV, 0~10000)
 *  0x0012    RO    CH3 电压 (mV, 0~10000)
 *  0x0013    RO    CH4 电压 (mV, 0~10000)
 *  0x0014    RO    MCU 内部温度 (°C×10, e.g. 275 = 27.5 °C)
 *  0x0015    RO    ADC 采样计数 (低 16 位)
 *  ──────────────────────────────────────────────────────────
 *  0x0020    RW    CH1 高报警阈值 (mV, 默认 9000)
 *  0x0021    RW    CH2 高报警阈值 (mV, 默认 9000)
 *  0x0022    RW    CH3 高报警阈值 (mV, 默认 9000)
 *  0x0023    RW    CH4 高报警阈值 (mV, 默认 9000)
 *  0x0024    RW    CH1 低报警阈值 (mV, 默认  500)
 *  0x0025    RW    CH2 低报警阈值 (mV, 默认  500)
 *  0x0026    RW    CH3 低报警阈值 (mV, 默认  500)
 *  0x0027    RW    CH4 低报警阈值 (mV, 默认  500)
 *  ──────────────────────────────────────────────────────────
 *  0x0030    RW    采样间隔 (ms, 默认 100, 最小 10)
 *  0x0031    RW    均值滤波次数 (默认 8, 范围 1~64)
 *  0x0032    RW    看门狗超时 (秒, 0=禁用; 超时未收到帧则复位)
 *  ──────────────────────────────────────────────────────────
 *
 * 只读边界: 地址 [0x0000, 0x001F] 为只读, 主机写操作将返回异常码 0x02。
 */
#ifndef DEVICE_REGS_H
#define DEVICE_REGS_H

#include "modbus_slave.h"
#include <stdint.h>

/* ── 只读区上界 ──────────────────────────────────────────── */
#define DEV_READONLY_END     0x0020u

/* ── 系统信息区 ──────────────────────────────────────────── */
#define REG_STATUS           0x0000u   /* 通道报警状态字 */
#define REG_FW_VER           0x0001u   /* 固件版本 */
#define REG_UPTIME_S         0x0002u   /* 运行时间 (秒) */
#define REG_FRAME_COUNT      0x0003u   /* MODBUS 帧计数 */
#define REG_LATENCY_US       0x0004u   /* 上次处理延迟 (μs) */
#define REG_CRC_ERR          0x0005u   /* CRC 错误数 */

/* ── 测量数据区 ──────────────────────────────────────────── */
#define REG_CH1_MV           0x0010u   /* CH1 电压 mV */
#define REG_CH2_MV           0x0011u   /* CH2 电压 mV */
#define REG_CH3_MV           0x0012u   /* CH3 电压 mV */
#define REG_CH4_MV           0x0013u   /* CH4 电压 mV */
#define REG_MCU_TEMP         0x0014u   /* MCU 温度 °C×10 */
#define REG_ADC_COUNT        0x0015u   /* ADC 采样计数 */

/* ── 报警阈值区 ──────────────────────────────────────────── */
#define REG_CH1_HI_THR       0x0020u
#define REG_CH2_HI_THR       0x0021u
#define REG_CH3_HI_THR       0x0022u
#define REG_CH4_HI_THR       0x0023u
#define REG_CH1_LO_THR       0x0024u
#define REG_CH2_LO_THR       0x0025u
#define REG_CH3_LO_THR       0x0026u
#define REG_CH4_LO_THR       0x0027u

/* ── 采集配置区 ──────────────────────────────────────────── */
#define REG_SAMPLE_INTV_MS   0x0030u   /* 采样间隔 ms              */
#define REG_AVG_COUNT        0x0031u   /* 均值次数                 */
#define REG_WATCHDOG_S       0x0032u   /* 看门狗秒数               */
#define REG_SAVE_CONFIG      0x0033u   /* 写 0x5A5A 触发 Flash 保存*/

/* ── HardFault 诊断区（只读）────────────────────────────────
 *  上电后若检测到 BKP 寄存器中有故障记录，则自动填充以下寄存器。
 *  SCADA/主机可通过 FC03 远程读取，无需现场物理接触即可诊断故障。
 *  ─────────────────────────────────────────────────────────
 *  0x0040  fault_flag    RO   0xBAD1 = 有故障记录；0 = 正常
 *  0x0041  fault_pc_lo   RO   故障 PC 低 16 位
 *  0x0042  fault_pc_hi   RO   故障 PC 高 16 位
 *  0x0043  fault_cfsr_lo RO   SCB->CFSR [15:0]  (内存/总线错误详情)
 *  0x0044  fault_cfsr_hi RO   SCB->CFSR [31:16] (使用错误详情)
 *  0x0045  fault_hfsr    RW   SCB->HFSR [15:0]；写任意值清除故障记录
 * ────────────────────────────────────────────────────────── */
#define REG_FAULT_FLAG       0x0040u
#define REG_FAULT_PC_LO      0x0041u
#define REG_FAULT_PC_HI      0x0042u
#define REG_FAULT_CFSR_LO    0x0043u
#define REG_FAULT_CFSR_HI    0x0044u
#define REG_FAULT_HFSR       0x0045u   /* 写此寄存器 = 清除故障记录 */

/* 总寄存器数 (需 >= 最高地址 + 1) */
#define DEVICE_REG_TOTAL     0x0046u

/* ── Flash 配置存储页 ─────────────────────────────────────
 *  使用 Flash 最后一页（页 63，0x0800FC00-0x0800FFFF，1 KB）
 *  存储掉电不丢失的用户配置（报警阈值 + 采样参数）。
 *  格式：[magic:4B][version:2B][threshold×8:16B][config×3:6B][crc16:2B]
 * ────────────────────────────────────────────────────────── */
#define CFG_FLASH_PAGE_ADDR  0x0800FC00UL   /* Flash 页 63 起始地址 */
#define CFG_FLASH_MAGIC      0xA5A5CDEFuL   /* 有效配置魔数 */
#define CFG_SAVE_KEY         0x5A5Au         /* 写入 REG_SAVE_CONFIG 触发保存 */

/**
 * @brief 初始化设备寄存器映射，写入出厂默认值，并尝试从 Flash 加载已保存配置。
 * @param mb  已完成 Modbus_Init() 的协议栈句柄
 */
void devregs_init(ModbusSlave_t *mb);

/**
 * @brief 周期更新测量数据和统计信息 (在主循环中每 ~100 ms 调用一次)
 * @param mb       协议栈句柄
 * @param tick_ms  当前毫秒计数 (SysTick)
 */
void devregs_update(ModbusSlave_t *mb, uint32_t tick_ms);

/**
 * @brief  将当前报警阈值和采集配置保存到 Flash（掉电不丢失）。
 *         先擦除页 63，然后写入 magic + 参数 + CRC16。
 *         写入失败时 REG_STATUS bit[15] 置位报错。
 * @param  mb  协议栈句柄（读取当前寄存器值）。
 */
void devregs_save_config(ModbusSlave_t *mb);

/**
 * @brief  从 Flash 读取已保存配置并写入寄存器。
 *         若 magic 无效或 CRC16 错误，保持出厂默认值不变。
 * @return 1 = 加载成功，0 = 无有效配置（使用出厂默认值）。
 */
int devregs_load_config(ModbusSlave_t *mb);

/**
 * @brief  将 HardFault 诊断数据从 BKP 寄存器同步到 Modbus 寄存器区。
 *         上电时由 devregs_init() 自动调用。
 * @param  mb  协议栈句柄。
 */
void devregs_load_fault_info(ModbusSlave_t *mb);

#endif /* DEVICE_REGS_H */
