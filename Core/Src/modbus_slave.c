/**
 * @file    modbus_slave.c
 * @brief   MODBUS RTU 从机协议栈实现
 */

#include "modbus_slave.h"
#include <string.h>

/* MODBUS RTU 帧格式:
 * | Slave Addr | Func Code | Data ...  | CRC Lo | CRC Hi |
 * |   1 byte   |  1 byte   | N bytes   | 1 byte | 1 byte |
 * 帧间隔 >= 3.5 字符时间（9600bps 时约 4ms，115200bps 时约 305us）
 */

/* CRC16 查表法（标准 MODBUS CRC16，初始值 0xFFFF，多项式 0xA001） */
static const uint16_t crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    while (length--) {
        uint8_t idx = (uint8_t)(crc ^ *data++);
        crc = (crc >> 8) ^ crc_table[idx];
    }
    return crc;
}

/* ===== 内部辅助函数 ===== */

static void mb_reset_timer(ModbusSlave_t *mb)
{
    __HAL_TIM_SET_COUNTER(mb->htim, 0);
    HAL_TIM_Base_Start_IT(mb->htim);
}

static void mb_stop_timer(ModbusSlave_t *mb)
{
    HAL_TIM_Base_Stop_IT(mb->htim);
}

static void mb_send_exception(ModbusSlave_t *mb, uint8_t func, uint8_t exc_code)
{
    mb->tx_buf[0] = MODBUS_SLAVE_ADDR;
    mb->tx_buf[1] = func | 0x80;  /* 异常响应：功能码最高位置 1 */
    mb->tx_buf[2] = exc_code;
    uint16_t crc = Modbus_CRC16(mb->tx_buf, 3);
    mb->tx_buf[3] = crc & 0xFF;
    mb->tx_buf[4] = (crc >> 8) & 0xFF;
    mb->tx_len = 5;
    mb->exception_count++;
}

/* 功能码 0x03：读保持寄存器 */
static void mb_handle_read_holding(ModbusSlave_t *mb)
{
    uint16_t addr = (mb->rx_buf[2] << 8) | mb->rx_buf[3];
    uint16_t count = (mb->rx_buf[4] << 8) | mb->rx_buf[5];
    
    /* 参数检查 */
    if (count == 0 || count > MODBUS_MAX_REGS_PER_REQ) {
        mb_send_exception(mb, MB_FUNC_READ_HOLDING, MB_EXC_ILLEGAL_VALUE);
        return;
    }
    if (addr + count > MODBUS_REG_COUNT) {
        mb_send_exception(mb, MB_FUNC_READ_HOLDING, MB_EXC_ILLEGAL_ADDR);
        return;
    }
    
    /* 组装响应：地址 + 功能码 + 字节数 + 数据 + CRC */
    mb->tx_buf[0] = MODBUS_SLAVE_ADDR;
    mb->tx_buf[1] = MB_FUNC_READ_HOLDING;
    mb->tx_buf[2] = count * 2;  /* 字节数 */
    
    for (uint16_t i = 0; i < count; i++) {
        uint16_t val = mb->holding_regs[addr + i];
        mb->tx_buf[3 + i * 2] = (val >> 8) & 0xFF;
        mb->tx_buf[4 + i * 2] = val & 0xFF;
    }
    
    uint16_t resp_len = 3 + count * 2;
    uint16_t crc = Modbus_CRC16(mb->tx_buf, resp_len);
    mb->tx_buf[resp_len]     = crc & 0xFF;
    mb->tx_buf[resp_len + 1] = (crc >> 8) & 0xFF;
    mb->tx_len = resp_len + 2;
}

/* 功能码 0x06：写单个寄存器 */
static void mb_handle_write_single(ModbusSlave_t *mb)
{
    uint16_t addr = (mb->rx_buf[2] << 8) | mb->rx_buf[3];
    uint16_t value = (mb->rx_buf[4] << 8) | mb->rx_buf[5];
    
    if (addr >= MODBUS_REG_COUNT) {
        mb_send_exception(mb, MB_FUNC_WRITE_SINGLE, MB_EXC_ILLEGAL_ADDR);
        return;
    }
    
    mb->holding_regs[addr] = value;
    
    /* 响应：回显请求帧（地址 + 功能码 + 寄存器地址 + 值 + CRC） */
    memcpy(mb->tx_buf, mb->rx_buf, 6);
    uint16_t crc = Modbus_CRC16(mb->tx_buf, 6);
    mb->tx_buf[6] = crc & 0xFF;
    mb->tx_buf[7] = (crc >> 8) & 0xFF;
    mb->tx_len = 8;
}

/* 功能码 0x10：写多个寄存器 */
static void mb_handle_write_multi(ModbusSlave_t *mb)
{
    uint16_t addr = (mb->rx_buf[2] << 8) | mb->rx_buf[3];
    uint16_t count = (mb->rx_buf[4] << 8) | mb->rx_buf[5];
    uint8_t  byte_count = mb->rx_buf[6];
    
    if (count == 0 || count > MODBUS_MAX_REGS_PER_REQ || byte_count != count * 2) {
        mb_send_exception(mb, MB_FUNC_WRITE_MULTI, MB_EXC_ILLEGAL_VALUE);
        return;
    }
    if (addr + count > MODBUS_REG_COUNT) {
        mb_send_exception(mb, MB_FUNC_WRITE_MULTI, MB_EXC_ILLEGAL_ADDR);
        return;
    }
    
    for (uint16_t i = 0; i < count; i++) {
        uint16_t val = (mb->rx_buf[7 + i * 2] << 8) | mb->rx_buf[8 + i * 2];
        mb->holding_regs[addr + i] = val;
    }
    
    /* 响应：地址 + 功能码 + 起始地址 + 数量 + CRC */
    mb->tx_buf[0] = MODBUS_SLAVE_ADDR;
    mb->tx_buf[1] = MB_FUNC_WRITE_MULTI;
    mb->tx_buf[2] = (addr >> 8) & 0xFF;
    mb->tx_buf[3] = addr & 0xFF;
    mb->tx_buf[4] = (count >> 8) & 0xFF;
    mb->tx_buf[5] = count & 0xFF;
    uint16_t crc = Modbus_CRC16(mb->tx_buf, 6);
    mb->tx_buf[6] = crc & 0xFF;
    mb->tx_buf[7] = (crc >> 8) & 0xFF;
    mb->tx_len = 8;
}

/* ===== 公开接口 ===== */

void Modbus_Init(ModbusSlave_t *mb, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim)
{
    memset(mb, 0, sizeof(ModbusSlave_t));
    mb->huart = huart;
    mb->htim = htim;
    mb->state = MB_STATE_IDLE;
}

void Modbus_OnRxByte(ModbusSlave_t *mb, uint8_t byte)
{
    if (mb->state == MB_STATE_FRAME_READY || mb->state == MB_STATE_PROCESSING) {
        return;  /* 上一帧还没处理完，丢弃新数据 */
    }
    
    if (mb->rx_len < MODBUS_RX_BUF_SIZE) {
        mb->rx_buf[mb->rx_len++] = byte;
        mb->state = MB_STATE_RECEIVING;
        mb_reset_timer(mb);  /* 每收到一字节，重置 3.5 字符超时定时器 */
    }
}

void Modbus_OnTimeout(ModbusSlave_t *mb)
{
    mb_stop_timer(mb);
    if (mb->state == MB_STATE_RECEIVING && mb->rx_len >= 4) {
        mb->state = MB_STATE_FRAME_READY;
    } else {
        /* 帧太短或状态异常，丢弃 */
        mb->rx_len = 0;
        mb->state = MB_STATE_IDLE;
    }
}

void Modbus_Poll(ModbusSlave_t *mb)
{
    if (mb->state != MB_STATE_FRAME_READY) return;
    
    mb->state = MB_STATE_PROCESSING;
    mb->frame_count++;
    
    /* 1. 地址过滤 */
    if (mb->rx_buf[0] != MODBUS_SLAVE_ADDR) {
        goto frame_end;  /* 不是发给本机的 */
    }
    
    /* 2. CRC 校验 */
    uint16_t recv_crc = mb->rx_buf[mb->rx_len - 2] | (mb->rx_buf[mb->rx_len - 1] << 8);
    uint16_t calc_crc = Modbus_CRC16(mb->rx_buf, mb->rx_len - 2);
    if (recv_crc != calc_crc) {
        mb->crc_err_count++;
        goto frame_end;
    }
    
    /* 3. 分发功能码 */
    uint8_t func = mb->rx_buf[1];
    mb->tx_len = 0;
    
    switch (func) {
        case MB_FUNC_READ_HOLDING:
            mb_handle_read_holding(mb);
            break;
        case MB_FUNC_WRITE_SINGLE:
            mb_handle_write_single(mb);
            break;
        case MB_FUNC_WRITE_MULTI:
            mb_handle_write_multi(mb);
            break;
        default:
            mb_send_exception(mb, func, MB_EXC_ILLEGAL_FUNC);
            break;
    }
    
    /* 4. 发送响应 */
    if (mb->tx_len > 0) {
        mb->state = MB_STATE_SENDING;
        HAL_UART_Transmit(mb->huart, mb->tx_buf, mb->tx_len, 100);
    }
    
frame_end:
    mb->rx_len = 0;
    mb->state = MB_STATE_IDLE;
}

void Modbus_SetReg(ModbusSlave_t *mb, uint16_t addr, uint16_t value)
{
    if (addr < MODBUS_REG_COUNT) {
        mb->holding_regs[addr] = value;
    }
}

uint16_t Modbus_GetReg(ModbusSlave_t *mb, uint16_t addr)
{
    if (addr < MODBUS_REG_COUNT) {
        return mb->holding_regs[addr];
    }
    return 0;
}
