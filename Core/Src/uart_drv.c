/**
 * @file  uart_drv.c
 * @brief STM32F103 USART1 寄存器级驱动实现
 *
 * USART1 挂载于 APB2 总线, 最高支持 72 MHz。
 *
 * 关键寄存器 (RM0008 §27.6):
 *   SR  (0x00): TXE[7] 发送缓冲空, TC[6] 发送完成, RXNE[5] 收到数据, ORE[3] 溢出
 *   DR  (0x04): 写入=发送字节, 读取=接收字节 (读 DR 自动清除 RXNE)
 *   BRR (0x08): [15:4]=整数部分, [3:0]=小数部分 (F1 实际上仅用整数部分)
 *   CR1 (0x0C): UE[13] 外设使能, TE[3] 发送使能, RE[2] 接收使能, RXNEIE[5] 中断使能
 */

#include "uart_drv.h"
#include "stm32f1xx.h"

/* PA8 = RS485 DE 方向控制引脚 */
#define DE_PIN_MASK  (1u << 8)

void uart1_init(uint32_t apb2_hz, uint32_t baud)
{
    /* ── 步骤 1: 使能 APB2 外设时钟 ─────────────────────────── */
    /* IOPAEN: GPIOA 时钟, AFIOEN: 复用功能时钟, USART1EN: USART1 时钟 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN
                 |  RCC_APB2ENR_AFIOEN
                 |  RCC_APB2ENR_USART1EN;

    /* ── 步骤 2: 配置 GPIO (CRH 控制 PA8~PA15) ────────────────
     *
     * CRx 每个引脚占 4 位: [3:2]=CNF, [1:0]=MODE
     *   MODE=00: 输入       MODE=01/10/11: 输出 (2/10/50 MHz)
     *   CNF(输出): 00=通用推挽, 01=通用开漏, 10=复用推挽, 11=复用开漏
     *   CNF(输入): 00=模拟,     01=浮空,      10=上下拉
     *
     * PA8 (DE, bit[3:0]  of CRH): 通用推挽 2MHz → 0x2
     * PA9 (TX, bit[7:4]  of CRH): 复用推挽 50MHz → 0xB
     * PA10(RX, bit[11:8] of CRH): 浮空输入       → 0x4
     */
    GPIOA->CRH &= ~(0xFFFu << 0);       /* 清除 PA8/PA9/PA10 的 12 位配置 */
    GPIOA->CRH |=  (0x2u  << 0)         /* PA8:  推挽输出 2MHz */
                |  (0xBu  << 4)         /* PA9:  AF 推挽 50MHz */
                |  (0x4u  << 8);        /* PA10: 浮空输入 */
    GPIOA->BRR = DE_PIN_MASK;           /* DE 初始低电平 = 接收方向 */

    /* ── 步骤 3: 配置波特率 ──────────────────────────────────
     *
     * BRR = f_PCLK2 / BAUD
     * STM32F1 的 USART BRR 字段中 [3:0]=小数部分 (0~15/16 波特时钟)
     * 但 F1 硬件实际将小数部分 /16, 与 F4/F7 的 /8 不同。
     * 最简写法: BRR = (f_PCLK2 + BAUD/2) / BAUD (整数四舍五入)
     */
    USART1->BRR = (uint16_t)((apb2_hz + baud / 2u) / baud);

    /* ── 步骤 4: 使能 USART ───────────────────────────────────
     *
     * CR1 = UE (外设总开关) | TE (发送器) | RE (接收器)
     * 注意: 先写 CR1 再配置其他字段, 因为 UE=0 时部分字段只读
     */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    /* ── 步骤 5: 配置 NVIC ────────────────────────────────────
     *
     * 优先级 1 (0 最高): 低于 TIM2(0), 保证帧结束检测不被 UART 中断延迟
     */
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
}

void uart1_send_buf(const uint8_t *buf, uint16_t len)
{
    /* RS485: DE 高 = 发送方向 */
    GPIOA->BSRR = DE_PIN_MASK;

    while (len--) {
        /* TXE=1 表示 DR 为空, 可写入下一字节 */
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *buf++;
    }
    /* 注意: 此时最后一字节还在移位寄存器中发送, 需 wait_tc() 才能切换 DE */
}

void uart1_wait_tc(void)
{
    /* TC=1: 移位寄存器已完全发出最后一字节, 线路已回到静默状态 */
    while (!(USART1->SR & USART_SR_TC));
    /* 安全窗口: DE 拉低不会截断任何位 */
    GPIOA->BRR = DE_PIN_MASK;
}

void uart1_rxne_irq_enable(void)
{
    USART1->CR1 |= USART_CR1_RXNEIE;
}

void uart1_rxne_irq_disable(void)
{
    USART1->CR1 &= ~USART_CR1_RXNEIE;
}
