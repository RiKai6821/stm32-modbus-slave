/**
 * @file  timer_drv.c
 * @brief TIM2 寄存器级驱动实现
 *
 * TIM2 关键寄存器 (RM0008 §15.4):
 *   CR1  [0x00]: CEN[0] 计数使能, UDIS[1] 更新禁止, OPM[3] 单次模式, ARPE[7] 预装载
 *   DIER [0x0C]: UIE[0] 更新中断使能
 *   SR   [0x10]: UIF[0] 更新中断标志 (写 0 清除)
 *   EGR  [0x14]: UG[0]  软件更新事件 (立即重载 PSC/ARR/CNT)
 *   CNT  [0x24]: 当前计数值
 *   PSC  [0x28]: 预分频器 (实际分频比 = PSC + 1)
 *   ARR  [0x2C]: 自动重载值 (计数从 0 到 ARR, 溢出触发 UIF)
 *
 * 单次模式 (OPM=1): 计数到 ARR 后, UIF 置位并自动清除 CEN,
 * 定时器停止。下次 tim2_restart() 时重新设置 CEN。
 */

#include "timer_drv.h"
#include "stm32f1xx.h"

void tim2_init_modbus(uint32_t tim_clk_hz, uint32_t baud)
{
    /* ── 1. 使能 APB1 上的 TIM2 时钟 ─────────────────────────── */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* ── 2. 计算超时参数 ──────────────────────────────────────
     *
     * 目标: 计数单位 = 1 μs, 则 PSC = tim_clk_hz / 1_000_000 - 1
     *
     * 3.5 字符超时 (μs):
     *   精确值: 35_000_000 / baud
     *   最小值: 1750 μs (MODBUS spec §2.5.1, baud >= 19200 时使用)
     */
    uint32_t psc        = tim_clk_hz / 1000000u - 1u;
    uint32_t timeout_us = (baud >= 19200u) ? 1750u : (35000000u / baud);

    /* ── 3. 写入寄存器 ────────────────────────────────────────
     *
     * 顺序: 先写 PSC/ARR, 再触发 UG 更新事件使其立即生效,
     * 最后才使能中断 (DIER), 避免 UG 事件误触发中断。
     */
    TIM2->CR1  = TIM_CR1_OPM;              /* 单次模式, CEN=0 */
    TIM2->PSC  = (uint16_t)psc;
    TIM2->ARR  = (uint16_t)(timeout_us - 1u);
    TIM2->EGR  = TIM_EGR_UG;               /* 强制更新: PSC/ARR 立即装载到影子寄存器 */
    TIM2->SR   = 0;                         /* 清除 UG 产生的 UIF */
    TIM2->DIER = TIM_DIER_UIE;             /* 使能更新中断 */

    /* ── 4. 配置 NVIC ─────────────────────────────────────────
     *
     * 优先级 0 (最高): 帧结束超时检测是 MODBUS 时序的关键路径,
     * 不能被其他中断延迟超过 ~100 μs, 否则可能把下一帧误判为当前帧。
     */
    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void tim2_restart(void)
{
    /*
     * 原子重置步骤 (顺序不能错):
     * 1. 停止计数, 防止在清零过程中溢出产生 UIF
     * 2. 清零计数器
     * 3. 清除可能已设置的 UIF (避免立即触发中断)
     * 4. 重新启动
     */
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CNT  = 0;
    TIM2->SR   = 0;
    TIM2->CR1 |=  TIM_CR1_CEN;
}

void tim2_stop(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->SR   = 0;
}
