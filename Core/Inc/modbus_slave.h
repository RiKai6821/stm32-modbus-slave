/**
 * @file  modbus_slave.h
 * @brief MODBUS RTU 从机协议栈 — HAL 无关可移植接口
 *
 * 协议栈通过函数指针与底层驱动解耦:
 *   - send / send_done : 发送帧数据 + 通知发送完成 (用于 RS485 DE 切换)
 *   - timer_restart    : 重置 3.5 字符帧间隔定时器
 *   - timer_stop       : 停止定时器
 *   - get_us           : 获取微秒时间戳 (用于响应延迟统计)
 *
 * 调用方式:
 *   1. 声明 ModbusSlave_t 变量
 *   2. 调用 Modbus_Init() 绑定驱动函数指针
 *   3. UART 收到字节 → 调用 Modbus_OnRxByte()
 *   4. 定时器超时     → 调用 Modbus_OnTimeout()
 *   5. 主循环         → 调用 Modbus_Poll()
 */

#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>
#include <stddef.h>

/* ===== 用户可配置参数 ===================================== */
#define MODBUS_SLAVE_ADDR        0x01u   /* 从机地址 (1-247) */
#define MODBUS_REG_COUNT         64u     /* 保持寄存器数量 */
#define MODBUS_COIL_COUNT        64u     /* 线圈数量 */
#define MODBUS_RX_BUF_SIZE       256u
#define MODBUS_TX_BUF_SIZE       256u
#define MODBUS_MAX_REGS_PER_REQ  32u
#define MODBUS_MAX_COILS_PER_REQ 64u

/* ===== MODBUS 功能码 ====================================== */
#define MB_FUNC_READ_COILS       0x01u
#define MB_FUNC_WRITE_COIL       0x05u
#define MB_FUNC_READ_HOLDING     0x03u
#define MB_FUNC_WRITE_SINGLE     0x06u
#define MB_FUNC_WRITE_MULTI      0x10u

/* ===== MODBUS 异常码 ====================================== */
#define MB_EXC_ILLEGAL_FUNC      0x01u
#define MB_EXC_ILLEGAL_ADDR      0x02u
#define MB_EXC_ILLEGAL_VALUE     0x03u

/* ===== 协议栈状态 ========================================= */
typedef enum {
    MB_STATE_IDLE = 0,
    MB_STATE_RECEIVING,
    MB_STATE_FRAME_READY,
    MB_STATE_PROCESSING,
    MB_STATE_SENDING
} ModbusState_t;

/* ===== 协议栈句柄 ========================================= */
typedef struct {
    /*
     * 驱动回调 (由 Modbus_Init() 绑定, 不得为 NULL):
     *
     * send(buf, len)  — 发送 len 字节到物理总线; RS485 实现中应在
     *                   此函数内置 DE 高, 使能发送方向。
     *
     * send_done()     — send() 完成后调用; RS485 实现中等待 TC
     *                   (Transmission Complete) 后拉低 DE。
     *                   对于全双工 TTL 接口可设为 NULL。
     *
     * timer_restart() — 每收到 1 字节调用, 重置 3.5 字符超时定时器。
     *
     * timer_stop()    — 帧处理完毕后调用, 防止误超时。
     *
     * get_us()        — 返回单调递增的微秒时间戳,
     *                   用于测量帧处理延迟 → 写入 last_proc_us。
     *                   若不需要延迟统计, 可设为 NULL (会跳过统计)。
     */
    void     (*send)          (const uint8_t *buf, uint16_t len);
    void     (*send_done)     (void);
    void     (*timer_restart) (void);
    void     (*timer_stop)    (void);
    uint32_t (*get_us)        (void);

    /* 收发缓冲区 */
    uint8_t  rx_buf[MODBUS_RX_BUF_SIZE];
    uint16_t rx_len;
    uint8_t  tx_buf[MODBUS_TX_BUF_SIZE];
    uint16_t tx_len;

    /* 数据存储 */
    uint16_t holding_regs[MODBUS_REG_COUNT];
    uint8_t  coils[MODBUS_COIL_COUNT / 8u];

    /* 只读保护: 地址 [0, readonly_end) 主机不可写, 0 = 不保护 */
    uint16_t readonly_end;

    /* 协议栈状态 (中断与主循环共享, volatile) */
    volatile ModbusState_t state;

    /* 统计信息 */
    uint32_t frame_count;       /* 成功接收并通过 CRC 的帧数 */
    uint32_t crc_err_count;     /* CRC 校验失败次数 */
    uint32_t exception_count;   /* 发送异常响应次数 */
    uint32_t last_proc_us;      /* 上次帧处理耗时 (μs), 0 = 未计时 */
} ModbusSlave_t;


/* ===== 公开接口 =========================================== */

/**
 * @brief 初始化协议栈并绑定驱动回调
 * @param mb               协议栈句柄 (调用前无需手动清零)
 * @param send_fn          发送函数, 必须非 NULL
 * @param send_done_fn     发送完成通知, RS485 必须提供, 全双工传 NULL
 * @param timer_restart_fn 重置超时定时器, 必须非 NULL
 * @param timer_stop_fn    停止定时器, 必须非 NULL
 * @param get_us_fn        微秒时间戳, 传 NULL 则禁用延迟统计
 */
void Modbus_Init(ModbusSlave_t *mb,
                 void     (*send_fn)         (const uint8_t*, uint16_t),
                 void     (*send_done_fn)     (void),
                 void     (*timer_restart_fn) (void),
                 void     (*timer_stop_fn)    (void),
                 uint32_t (*get_us_fn)        (void));

/** @brief 在 UART RXNE 中断中调用, 将收到的字节送入协议栈 */
void Modbus_OnRxByte(ModbusSlave_t *mb, uint8_t byte);

/** @brief 在定时器超时中断中调用, 标记帧结束 */
void Modbus_OnTimeout(ModbusSlave_t *mb);

/** @brief 在主循环中轮询, 处理已收到的帧并发送响应 */
void Modbus_Poll(ModbusSlave_t *mb);

/** @brief 设置保持寄存器只读保护上界 [0, readonly_end) */
void Modbus_SetReadonly(ModbusSlave_t *mb, uint16_t readonly_end);

/** @brief 内部写保持寄存器 (绕过只读保护, 供设备层使用) */
void Modbus_SetReg(ModbusSlave_t *mb, uint16_t addr, uint16_t value);

/** @brief 读保持寄存器 */
uint16_t Modbus_GetReg(ModbusSlave_t *mb, uint16_t addr);

/** @brief 写线圈 (0=OFF, 非0=ON) */
void Modbus_SetCoil(ModbusSlave_t *mb, uint16_t addr, uint8_t value);

/** @brief 读线圈 */
uint8_t Modbus_GetCoil(ModbusSlave_t *mb, uint16_t addr);

/** @brief CRC16 查表法 (初始值 0xFFFF, 多项式 0xA001) */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length);

#endif /* MODBUS_SLAVE_H */
