/**
 * @file  uart_drv.h
 * @brief STM32F103 USART1 寄存器级驱动
 *
 * 直接操作 USART1 寄存器，不依赖 STM32 HAL/LL 库。
 * 引脚: PA9 = TX (AF push-pull 50 MHz), PA10 = RX (floating input)
 * RS485 方向控制: PA8 = DE (高=发送, 低=接收)
 *
 * 波特率寄存器计算公式 (RM0008 §27.3.4):
 *   BRR = f_PCLK2 / BAUD   (取整, +BAUD/2 四舍五入)
 *
 * 典型时序 (115200 bps):
 *   1 字符 = 10 bit = 86.8 μs
 *   TXE→下字节写入延迟 ≤ 1 字符时间
 *   TC 置位 = 最后一位已离开移位寄存器
 */
#ifndef UART_DRV_H
#define UART_DRV_H

#include <stdint.h>

/**
 * @brief 初始化 USART1
 * @param apb2_hz  APB2 时钟频率 (Hz), 用于 BRR 计算
 * @param baud     波特率 (如 9600, 115200)
 *
 * 完成: RCC 时钟使能, GPIO 复用配置, BRR 写入, CR1 使能 UE|TE|RE,
 *       NVIC 中断注册 (优先级 1)。
 */
void uart1_init(uint32_t apb2_hz, uint32_t baud);

/**
 * @brief 发送数据帧 (RS485: 自动拉高 DE)
 * @note  发送完最后一字节后 DE 仍为高; 调用 uart1_wait_tc() 等待
 *        移位寄存器完全清空后再拉低 DE, 防止截断最后一位。
 */
void uart1_send_buf(const uint8_t *buf, uint16_t len);

/**
 * @brief 等待 TC (Transmission Complete) 并拉低 DE
 *
 * MODBUS RS485 半双工切换时序:
 *   send_buf → wait_tc → [DE=低, 进入接收方向]
 *
 * TC 与 TXE 的区别:
 *   TXE=1  仅表示 DR 已经被移走, 移位寄存器仍在发送最后一字节
 *   TC=1   移位寄存器已完全发出, 线路已静默
 */
void uart1_wait_tc(void);

/** @brief 使能 RXNE 中断 (启动接收) */
void uart1_rxne_irq_enable(void);

/** @brief 关闭 RXNE 中断 */
void uart1_rxne_irq_disable(void);

#endif /* UART_DRV_H */
