/**
 * @file    main.c
 * @brief   STM32 MODBUS RTU 从机 - 主程序
 * @note    硬件: STM32F103C8T6
 *          USART1 (PA9-TX, PA10-RX) 用于 MODBUS 通信
 *          TIM2 用于 3.5 字符超时检测
 *          PC13 板载 LED，每收到一帧闪一次
 *
 *          寄存器映射示例：
 *          0x0000 - 0x0003: 系统状态（运行计数、版本号等，只读建议）
 *          0x0010 - 0x001F: 控制寄存器（主机可写）
 *          0x0020 - 0x002F: 模拟数据（如温度、电压，定时更新）
 */

#include "main.h"
#include "modbus_slave.h"

/* ===== HAL 句柄（由 CubeMX 生成） ===== */
UART_HandleTypeDef huart1;
TIM_HandleTypeDef  htim2;

/* ===== MODBUS 协议栈实例 ===== */
ModbusSlave_t g_modbus;

/* ===== UART 单字节接收用 ===== */
static uint8_t uart_rx_byte;

/* ===== 前向声明（CubeMX 生成） ===== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    
    /* 初始化 MODBUS 协议栈 */
    Modbus_Init(&g_modbus, &huart1, &htim2);

    /* 0x0000-0x000F 为系统状态寄存器，主机只读 */
    Modbus_SetReadonly(&g_modbus, 0x0010);

    /* 初始化寄存器示例数据 */
    Modbus_SetReg(&g_modbus, 0x0000, 0x0100);  /* 版本号 v1.0 */
    Modbus_SetReg(&g_modbus, 0x0001, 0);       /* 运行计数器 */
    
    /* 启动 UART 中断接收 */
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    
    uint32_t last_tick = HAL_GetTick();
    uint16_t sim_value = 0;
    
    while (1) {
        /* 处理 MODBUS 帧 */
        Modbus_Poll(&g_modbus);
        
        /* 每秒更新一次模拟数据，模拟传感器读数 */
        if (HAL_GetTick() - last_tick >= 1000) {
            last_tick = HAL_GetTick();
            
            /* 运行计数器自增 */
            uint16_t cnt = Modbus_GetReg(&g_modbus, 0x0001);
            Modbus_SetReg(&g_modbus, 0x0001, cnt + 1);
            
            /* 模拟温度数据（25.0 ~ 35.0 摄氏度，放大 10 倍存储） */
            sim_value = 250 + (cnt % 100);
            Modbus_SetReg(&g_modbus, 0x0020, sim_value);
            
            /* 模拟电压数据 */
            Modbus_SetReg(&g_modbus, 0x0021, 3300 + (cnt % 50));
        }
    }
}

/* ===== HAL 中断回调 ===== */

/* UART 接收完成中断 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Modbus_OnRxByte(&g_modbus, uart_rx_byte);
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);  /* 重新启动接收 */
    }
}

/* TIM 超时中断（3.5 字符间隔到达，帧结束） */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        Modbus_OnTimeout(&g_modbus);
        /* 收到一帧，板载 LED 翻转一下 */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

/* ===== 错误处理 ===== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
