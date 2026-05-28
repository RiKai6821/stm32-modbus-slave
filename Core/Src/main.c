/**
 * @file  main.c
 * @brief STM32F103 MODBUS RTU 从机 — 4 通道电压采集模块（v2.1）
 *
 * 硬件: STM32F103C8T6 (Blue Pill, 72 MHz)
 *
 * 引脚:
 *   PA8  - RS485 DE (高=发送, 低=接收)
 *   PA9  - USART1 TX
 *   PA10 - USART1 RX
 *   PC13 - 板载 LED (每收到一帧闪烁, 用于肉眼监控通信)
 *
 * v2.1 新增：
 *   - DMA TX：USART1_TX 通过 DMA1 CH4 非阻塞发送（消除 ~1.74 ms 忙等）
 *   - Flash 持久化配置：报警阈值写入 Flash 页 63，掉电不丢失
 *   - HardFault 诊断帧：故障 PC/CFSR/HFSR 保存到 BKP，主机可远程读取
 *
 * 架构要点:
 *   - 时钟:  寄存器级 PLL 配置, HSE 8MHz × 9 = 72 MHz
 *   - UART:  uart_drv.c 直接操作 USART1 寄存器, RXNE 中断接收
 *   - DMA TX: DMA1 CH4，TC 中断触发 DE 切换
 *   - 定时:  timer_drv.c 直接操作 TIM2, 单次模式检测帧结束
 *   - MODBUS: 协议栈通过函数指针调用驱动, 完全可移植
 *   - 延迟:  DWT->CYCCNT (Cortex-M3 周期计数器) 实现微秒精度统计
 */

#include "stm32f1xx.h"
#include "modbus_slave.h"
#include "uart_drv.h"
#include "timer_drv.h"
#include "device_regs.h"

/* ── 系统时钟参数 (HSE=8 MHz, PLL×9=72 MHz) ──────────────── */
#define SYSCLK_HZ    72000000u
#define APB1_HZ      36000000u  /* APB1 预分频 /2 */
#define APB2_HZ      72000000u  /* APB2 不分频 */
#define TIM2_CLK_HZ  72000000u  /* APB1 预分频>1 时 TIM 时钟 ×2 */

#define MODBUS_BAUD  115200u

/* ── 全局实例 ────────────────────────────────────────────── */
ModbusSlave_t g_modbus;

/* ── SysTick 毫秒计数器 ──────────────────────────────────── */
static volatile uint32_t g_tick_ms = 0;

/* ── DWT 微秒时间戳 (用于 MODBUS 处理延迟统计) ─────────────── */
static uint32_t get_us(void)
{
    return DWT->CYCCNT / (SYSCLK_HZ / 1000000u);
}

/* ── SysTick ─────────────────────────────────────────────── */
void SysTick_Handler(void) { g_tick_ms++; }

/* ── USART1 接收中断 ─────────────────────────────────────── */
void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t b = (uint8_t)(USART1->DR & 0xFFu);
        Modbus_OnRxByte(&g_modbus, b);
    }
    /* ORE (overrun): DR 尚未读出时又来了新字节。读一次 DR 清除。
     * 若不处理 ORE, 中断会无限重入。                              */
    if (sr & USART_SR_ORE) {
        (void)USART1->DR;
        g_modbus.crc_err_count++;  /* 溢出意味着至少丢了一个字节 */
    }
}

/* ── TIM2 超时中断 (3.5 字符帧间隔到期) ────────────────────── */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR = ~TIM_SR_UIF;
        Modbus_OnTimeout(&g_modbus);
        GPIOC->ODR ^= GPIO_ODR_ODR13;
    }
}

/* ── DMA1 CH4 传输完成中断（USART1_TX DMA）────────────────── */
void DMA1_Channel4_IRQHandler(void)
{
    uart1_dma_tc_irq_handler();   /* 清 TC 标志 + g_dma_tx_busy = 0 */
}

/* ─────────────────────────────────────────────────────────────
 * HardFault 诊断处理器
 *
 * 当发生非法内存访问、非法指令等 HardFault 时，Cortex-M3 自动将
 * 以下寄存器压栈（异常帧）：
 *   [SP+0]  R0       [SP+4]  R1      [SP+8]  R2     [SP+12] R3
 *   [SP+16] R12      [SP+20] LR      [SP+24] PC     [SP+28] xPSR
 *
 * 汇编 stub 判断使用 MSP 还是 PSP（取决于 EXC_RETURN bit[2]），
 * 将栈指针作为参数传给 C 处理函数。
 * ──────────────────────────────────────────────────────────── */
void HardFault_Handler_C(uint32_t *stack)
{
    uint32_t fault_pc   = stack[6];           /* 触发 fault 的指令地址 */
    uint32_t fault_cfsr = SCB->CFSR;          /* 详细故障原因字 */
    uint32_t fault_hfsr = SCB->HFSR;          /* HardFault 状态 */

    /* 保存到 BKP 寄存器（NVIC_SystemReset 后依然保留）*/
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    BKP->DR1 = 0xBAD1u;                              /* 故障魔数 */
    BKP->DR2 = (uint16_t)(fault_pc  & 0xFFFFu);
    BKP->DR3 = (uint16_t)(fault_pc  >> 16u);
    BKP->DR4 = (uint16_t)(fault_cfsr & 0xFFFFu);
    BKP->DR5 = (uint16_t)(fault_cfsr >> 16u);
    BKP->DR6 = (uint16_t)(fault_hfsr & 0xFFFFu);

    /* 清除 CFSR（写 1 清零）防止下次错误判读混淆 */
    SCB->CFSR = fault_cfsr;

    /* 等待 BKP 写入完成后复位，Bootloader / 启动代码会上报故障 */
    for (volatile int i = 0; i < 10000; i++);
    NVIC_SystemReset();
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "TST    LR, #4          \n"  /* EXC_RETURN bit[2]: 0=MSP,1=PSP */
        "ITE    EQ              \n"
        "MRSEQ  R0, MSP         \n"  /* 用 MSP */
        "MRSNE  R0, PSP         \n"  /* 用 PSP */
        "B      HardFault_Handler_C \n"
    );
}

/* ─────────────────────────────────────────────────────────────
 * 时钟配置 (寄存器级)
 *
 * 目标: HSE (8 MHz) → PLL × 9 → SYSCLK = 72 MHz
 *       AHB = 72 MHz, APB1 = 36 MHz (/2), APB2 = 72 MHz (不分频)
 *       USB = 48 MHz (PLL/1.5)
 *
 * 步骤来自 RM0008 §7.3:
 *   1. 使能 HSE, 等待就绪
 *   2. 配置 Flash 等待周期 (72 MHz → 2 wait states)
 *   3. 配置总线预分频
 *   4. 配置 PLL: 源 = HSE, 倍频 = ×9
 *   5. 使能 PLL, 等待就绪
 *   6. 切换系统时钟到 PLL
 * ────────────────────────────────────────────────────────────── */
static void SystemClock_Config(void)
{
    /* 1. 使能 HSE 并等待稳定 */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    /* 2. Flash 等待周期: 72 MHz 需要 2 个等待状态
     *    LATENCY[2:0] = 010b; PRFTBE (预取缓冲) = 1 提升指令吞吐 */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /* 3. 总线预分频
     *    HPRE=0000  → AHB  = SYSCLK / 1 = 72 MHz
     *    PPRE1=100  → APB1 = AHB   / 2 = 36 MHz (不超 36 MHz 限制)
     *    PPRE2=000  → APB2 = AHB   / 1 = 72 MHz
     */
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    /* 4. PLL 配置
     *    PLLSRC = HSE (而不是 HSI/2)
     *    PLLMUL = ×9 → 8 MHz × 9 = 72 MHz
     */
    RCC->CFGR |= RCC_CFGR_PLLSRC         /* PLL 时钟源 = HSE */
              |  RCC_CFGR_PLLMULL9;       /* 倍频 ×9 */

    /* 5. 使能 PLL 并等待锁定 */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* 6. 切换 SYSCLK → PLL, 等待切换完成 */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/* ── DWT 周期计数器初始化 ────────────────────────────────── */
static void dwt_init(void)
{
    /* CoreDebug DEMCR: 使能跟踪模块 (TRCENA), 否则 DWT 不工作 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;  /* 使能 32 位周期计数器 */
}

/* ── PC13 LED 初始化 ─────────────────────────────────────── */
static void led_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    /* PC13: 通用推挽输出, 2 MHz → CRH[23:20] = 0x2 */
    GPIOC->CRH &= ~(0xFu << 20);
    GPIOC->CRH |=  (0x2u << 20);
    GPIOC->BSRR = GPIO_BSRR_BS13;  /* 初始高电平 = 板载 LED 灭 (低有效) */
}

/* ───────────────────────────────────────────────────────────── */

int main(void)
{
    /* ── 1. 时钟 + 调试计数器 ──────────────────────────────── */
    SystemClock_Config();
    dwt_init();

    /* SysTick: 每 1 ms 中断一次 */
    SysTick_Config(SYSCLK_HZ / 1000u);

    /* ── 2. 外设初始化 ──────────────────────────────────────── */
    led_init();

    /* USART1: 115200 bps, 8N1 */
    uart1_init(APB2_HZ, MODBUS_BAUD);

    /* DMA1 CH4 → USART1_TX（非阻塞发送，消除约 1.74 ms 忙等）*/
    uart1_dma_init();

    /* TIM2: 精确 3.5 字符超时 = 1750 μs @115200 bps */
    tim2_init_modbus(TIM2_CLK_HZ, MODBUS_BAUD);

    /* ── 3. MODBUS 协议栈初始化（使用 DMA TX 回调）────────── */
    Modbus_Init(&g_modbus,
                uart1_dma_send_buf, /* send:      DMA 非阻塞发送       */
                uart1_dma_wait_tc,  /* send_done: 等待 DMA TC + DE 拉低 */
                tim2_restart,       /* timer_restart                    */
                tim2_stop,          /* timer_stop                       */
                get_us);            /* get_us: DWT 微秒时间戳           */

    /* ── 4. 设备寄存器初始化（含 Flash 配置加载 + 故障信息读取）*/
    devregs_init(&g_modbus);

    /* ── 5. 启动 UART 接收 ──────────────────────────────────── */
    uart1_rxne_irq_enable();

    /* ── 主循环 ─────────────────────────────────────────────── */
    while (1) {
        /* 处理已收到的 MODBUS 帧 (阻塞 ≤ 几十 μs, 由 last_proc_us 可查) */
        Modbus_Poll(&g_modbus);

        /* 定时更新 ADC 仿真数据、报警状态、统计信息 */
        devregs_update(&g_modbus, g_tick_ms);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
