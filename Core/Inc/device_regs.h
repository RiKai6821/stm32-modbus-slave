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
#define REG_SAMPLE_INTV_MS   0x0030u   /* 采样间隔 ms */
#define REG_AVG_COUNT        0x0031u   /* 均值次数 */
#define REG_WATCHDOG_S       0x0032u   /* 看门狗秒数 */

/* 总寄存器数 (需 >= 最高地址 + 1) */
#define DEVICE_REG_TOTAL     0x0040u

/**
 * @brief 初始化设备寄存器映射, 写入出厂默认值
 * @param mb  已完成 Modbus_Init() 的协议栈句柄
 */
void devregs_init(ModbusSlave_t *mb);

/**
 * @brief 周期更新测量数据和统计信息 (在主循环中每 ~100 ms 调用一次)
 * @param mb       协议栈句柄
 * @param tick_ms  当前毫秒计数 (SysTick)
 *
 * 完成:
 *   - 仿真 ADC 数据 (正弦波模拟 4 通道电压)
 *   - 报警阈值比较 → 更新 REG_STATUS 报警标志
 *   - 同步 MODBUS 统计 (帧计数, CRC 错误, 处理延迟) 到只读寄存器
 *   - 看门狗检查
 */
void devregs_update(ModbusSlave_t *mb, uint32_t tick_ms);

#endif /* DEVICE_REGS_H */
