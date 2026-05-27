/**
 * @file    modbus_slave.h
 * @brief   MODBUS RTU 从机协议栈
 * @details 支持功能码 0x03 (读保持寄存器)、0x06 (写单寄存器)、0x10 (写多寄存器)
 * @author  YourName
 */

#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ===== 用户可配置参数 ===== */
#define MODBUS_SLAVE_ADDR        0x01   /* 从机地址 (1-247) */
#define MODBUS_REG_COUNT         64     /* 保持寄存器数量 */
#define MODBUS_COIL_COUNT        64     /* 线圈数量（FC01/FC05） */
#define MODBUS_RX_BUF_SIZE       256    /* 接收缓冲区大小 */
#define MODBUS_TX_BUF_SIZE       256    /* 发送缓冲区大小 */
#define MODBUS_MAX_REGS_PER_REQ  32     /* 单次请求最大寄存器数 */
#define MODBUS_MAX_COILS_PER_REQ 64     /* 单次请求最大线圈数 */

/* MODBUS 功能码 */
#define MB_FUNC_READ_COILS      0x01
#define MB_FUNC_WRITE_COIL      0x05
#define MB_FUNC_READ_HOLDING    0x03
#define MB_FUNC_WRITE_SINGLE    0x06
#define MB_FUNC_WRITE_MULTI     0x10

/* MODBUS 异常码 */
#define MB_EXC_ILLEGAL_FUNC     0x01
#define MB_EXC_ILLEGAL_ADDR     0x02
#define MB_EXC_ILLEGAL_VALUE    0x03

/* 协议栈状态 */
typedef enum {
    MB_STATE_IDLE = 0,      /* 空闲，等待接收 */
    MB_STATE_RECEIVING,     /* 接收中 */
    MB_STATE_FRAME_READY,   /* 收到完整帧，待处理 */
    MB_STATE_PROCESSING,    /* 处理中 */
    MB_STATE_SENDING        /* 发送响应中 */
} ModbusState_t;

/* 协议栈句柄 */
typedef struct {
    UART_HandleTypeDef *huart;      /* 关联的 UART 句柄 */
    TIM_HandleTypeDef  *htim;       /* 3.5 字符超时定时器 */
    
    uint8_t  rx_buf[MODBUS_RX_BUF_SIZE];
    uint16_t rx_len;
    uint8_t  tx_buf[MODBUS_TX_BUF_SIZE];
    uint16_t tx_len;
    
    uint16_t holding_regs[MODBUS_REG_COUNT];           /* 保持寄存器 */
    uint8_t  coils[MODBUS_COIL_COUNT / 8];             /* 线圈（位打包，coil_n 在字节 n/8 的第 n%8 位） */

    uint16_t readonly_end;  /* 保持寄存器只读范围 [0, readonly_end)，0 = 不保护 */

    /* RS485 半双工方向控制引脚（DE/RE），NULL = 不使用 */
    GPIO_TypeDef *rs485_port;
    uint16_t      rs485_pin;

    volatile ModbusState_t state;
    
    uint32_t frame_count;
    uint32_t crc_err_count;
    uint32_t exception_count;
} ModbusSlave_t;

/* ===== 对外接口 ===== */

/**
 * @brief  初始化 MODBUS 从机协议栈
 * @param  mb     协议栈句柄
 * @param  huart  使用的 UART 外设
 * @param  htim   使用的定时器外设（用于 3.5 字符超时检测）
 */
void Modbus_Init(ModbusSlave_t *mb, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);

/**
 * @brief  主循环中调用，处理收到的帧并发送响应
 * @param  mb  协议栈句柄
 */
void Modbus_Poll(ModbusSlave_t *mb);

/**
 * @brief  UART 接收字节回调（在 UART RX 中断中调用）
 * @param  mb    协议栈句柄
 * @param  byte  收到的字节
 */
void Modbus_OnRxByte(ModbusSlave_t *mb, uint8_t byte);

/**
 * @brief  定时器超时回调（在 TIM 中断中调用，标志一帧结束）
 * @param  mb  协议栈句柄
 */
void Modbus_OnTimeout(ModbusSlave_t *mb);

/**
 * @brief  设置只读保持寄存器范围：地址 [0, readonly_end) 拒绝主机写操作
 */
void Modbus_SetReadonly(ModbusSlave_t *mb, uint16_t readonly_end);

/**
 * @brief  配置 RS485 半双工方向控制引脚（DE/RE 合并到同一个引脚）
 * @note   发送前 DE=高，发送完成后 DE=低（HAL_UART_Transmit 阻塞完成后立即拉低）
 *         传入 NULL 表示不使用（全双工 TTL 模式）
 */
void Modbus_SetRS485Pin(ModbusSlave_t *mb, GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief  用户层：写保持寄存器
 */
void Modbus_SetReg(ModbusSlave_t *mb, uint16_t addr, uint16_t value);

/**
 * @brief  用户层：读保持寄存器
 */
uint16_t Modbus_GetReg(ModbusSlave_t *mb, uint16_t addr);

/**
 * @brief  用户层：写线圈（0 = OFF，非 0 = ON）
 */
void Modbus_SetCoil(ModbusSlave_t *mb, uint16_t addr, uint8_t value);

/**
 * @brief  用户层：读线圈
 */
uint8_t Modbus_GetCoil(ModbusSlave_t *mb, uint16_t addr);

/* ===== 工具函数 ===== */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length);

#endif /* __MODBUS_SLAVE_H */
