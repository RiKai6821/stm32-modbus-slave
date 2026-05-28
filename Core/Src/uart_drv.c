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

/* ─────────────────────────────────────────────────────────────────
 * DMA TX 实现（DMA1 Channel 4 → USART1_TX）
 *
 * DMA1 Channel 4 映射关系（STM32F103，RM0008 Table 78）：
 *   DMA1 CH4 = USART1_TX
 *
 * CCR 寄存器字段（使用到的位）：
 *   DIR    [4]  = 1  → 方向：Memory → Peripheral
 *   MINC   [7]  = 1  → 内存地址自增（每发送一字节自增）
 *   TCIE   [1]  = 1  → 传输完成中断使能
 *   EN     [0]  = 1  → 通道使能（写入此位触发传输开始）
 *
 * 数据宽度（默认）：MSIZE = PSIZE = 8 bit（字节）
 * 优先级：PL[9:8] = 10（High），高于 AHB 刷新，减少总线等待
 * ──────────────────────────────────────────────────────────────── */

static volatile uint8_t g_dma_tx_busy = 0u;   /* 1 = DMA 正在传输 */

/* DMA1_Channel4_IRQHandler 在 main.c 中实现（需访问 g_modbus）。
 * 这里提供一个弱定义保证链接成功。 */
__attribute__((weak)) void DMA1_Channel4_IRQHandler(void) {}

void uart1_dma_init(void)
{
    /* 使能 DMA1 总线时钟（AHB1 上）*/
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    /* 预配置 DMA1 CH4（不使能，等到实际发送时再设 CMAR/CNDTR/EN）*/
    DMA1_Channel4->CCR = 0u;                   /* 先禁用 */
    DMA1_Channel4->CPAR = (uint32_t)&USART1->DR; /* 外设地址固定 */
    DMA1_Channel4->CCR = DMA_CCR_DIR           /* M→P 方向      */
                       | DMA_CCR_MINC          /* 内存地址自增  */
                       | DMA_CCR_PL_1          /* 优先级 High   */
                       | DMA_CCR_TCIE;         /* TC 中断使能   */

    /* 使能 USART1 DMA 发送请求（DMAT 位）*/
    USART1->CR3 |= USART_CR3_DMAT;

    /* 配置 NVIC：DMA1_Channel4 优先级 = 2（低于 TIM2 和 USART1）*/
    NVIC_SetPriority(DMA1_Channel4_IRQn, 2u);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

void uart1_dma_send_buf(const uint8_t *buf, uint16_t len)
{
    /* RS485：DE 高 = 发送方向 */
    GPIOA->BSRR = DE_PIN_MASK;

    /* 确保上一次传输已完成（安全互斥）*/
    while (g_dma_tx_busy);

    g_dma_tx_busy = 1u;

    /* 配置本次传输 */
    DMA1_Channel4->CCR  &= ~DMA_CCR_EN;          /* 先禁用通道（必须先清零才能修改 CMAR/CNDTR）*/
    DMA1_Channel4->CMAR  = (uint32_t)buf;        /* 内存起始地址 */
    DMA1_Channel4->CNDTR = len;                   /* 传输字节数   */
    DMA1_Channel4->CCR  |= DMA_CCR_EN;            /* 使能 → 传输立即开始 */

    /* 函数立即返回，DMA 在后台传输 */
}

void uart1_dma_wait_tc(void)
{
    /* 等待 DMA TC（DMA1_Channel4_IRQHandler 中清除 busy 标志）*/
    while (g_dma_tx_busy);

    /* DMA TC 时最后一字节已进入 USART DR，但移位寄存器可能还在发送。
     * 等待 USART TC = 移位寄存器也空了 = 线路完全静默 */
    while (!(USART1->SR & USART_SR_TC));

    /* 现在可以安全地拉低 DE，切换 RS485 到接收方向 */
    GPIOA->BRR = DE_PIN_MASK;
}

/**
 * @brief  外部调用：在 DMA1_Channel4_IRQHandler 中调用此函数。
 *         清除 DMA TC 标志，复位 busy 状态。
 */
void uart1_dma_tc_irq_handler(void)
{
    /* 清除 DMA1 CH4 传输完成标志（IFCR.CTCIF4，bit 13）*/
    DMA1->IFCR = DMA_IFCR_CTCIF4;
    g_dma_tx_busy = 0u;
}
