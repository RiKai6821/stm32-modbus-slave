/**
 * @file  timer_drv.h
 * @brief STM32F103 TIM2 寄存器级驱动 — MODBUS 3.5 字符帧间隔检测
 *
 * MODBUS RTU 规范 §2.5.1 规定:
 *   帧内字节间隔 < 1.5 字符时间  (否则视为帧错误)
 *   帧间静默时间 >= 3.5 字符时间 (标志一帧结束)
 *   一个"字符" = 1 起始位 + 8 数据位 + 1 停止位 = 10 位
 *   T_3.5char = 3.5 × 10 / baud = 35/baud 秒
 *
 * 对于 baud >= 19200, 规范允许用固定值 1.75 ms 代替精确计算值
 * (因精确值 < 200 μs 时实现困难)。
 *
 * 实现方式: TIM2 单次模式 (OPM)
 *   - 每收到一字节 → tim2_restart() 重置计数器
 *   - 超时无新字节 → UIF 中断触发 → 调用 Modbus_OnTimeout()
 *   - 定时精度: 1 μs (PSC 将 TIM_CLK 降至 1 MHz)
 *
 * TIM2 挂载于 APB1 总线; 当 APB1 预分频器 > 1 时,
 * TIM2 时钟 = APB1 频率 × 2 (即 72 MHz)。
 */
#ifndef TIMER_DRV_H
#define TIMER_DRV_H

#include <stdint.h>

/**
 * @brief 初始化 TIM2 并按波特率计算 3.5 字符超时周期
 * @param tim_clk_hz  TIM2 输入时钟 (Hz), 通常为 APB1×2 = 72 MHz
 * @param baud        MODBUS 波特率 (如 9600, 115200)
 *
 * PSC 固定设置为 (tim_clk/1_000_000 - 1), 使计数单位 = 1 μs。
 * ARR = max(35_000_000/baud, 1750) - 1   [μs]
 */
void tim2_init_modbus(uint32_t tim_clk_hz, uint32_t baud);

/**
 * @brief 重置计数器并启动单次计时 (每收到 1 字节调用)
 *
 * 必须原子操作: 先停止 → 清零 CNT → 清 UIF → 再启动,
 * 防止在计数器恰好溢出瞬间重置时误触发中断。
 */
void tim2_restart(void);

/**
 * @brief 停止计时器 (帧处理完毕后调用, 防止误超时)
 */
void tim2_stop(void);

#endif /* TIMER_DRV_H */
