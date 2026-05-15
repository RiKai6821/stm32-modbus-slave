# MODBUS RTU 协议说明

## 帧格式

```
┌──────────┬───────────┬─────────────┬─────────┬─────────┐
│ Slave ID │ Func Code │    Data     │ CRC Lo  │ CRC Hi  │
│  1 byte  │  1 byte   │  0~252 byte │ 1 byte  │ 1 byte  │
└──────────┴───────────┴─────────────┴─────────┴─────────┘
```

帧之间必须有 >= 3.5 字符的间隔时间。

## 支持的功能码

### 0x03 读保持寄存器

**请求**：
```
| Slave | 0x03 | Addr Hi | Addr Lo | Count Hi | Count Lo | CRC Lo | CRC Hi |
```

**响应**：
```
| Slave | 0x03 | Byte Cnt | Data ... | CRC Lo | CRC Hi |
```

### 0x06 写单个寄存器

**请求**：
```
| Slave | 0x06 | Addr Hi | Addr Lo | Val Hi | Val Lo | CRC Lo | CRC Hi |
```

**响应**（回显请求）：
```
| Slave | 0x06 | Addr Hi | Addr Lo | Val Hi | Val Lo | CRC Lo | CRC Hi |
```

### 0x10 写多个寄存器

**请求**：
```
| Slave | 0x10 | Addr Hi | Addr Lo | Count Hi | Count Lo | Byte Cnt | Data... | CRC |
```

**响应**：
```
| Slave | 0x10 | Addr Hi | Addr Lo | Count Hi | Count Lo | CRC Lo | CRC Hi |
```

## 异常响应

当请求处理失败时，从机返回异常响应：

```
| Slave | Func | 0x80 | Exception Code | CRC Lo | CRC Hi |
```

异常码：
- `0x01` 非法功能码
- `0x02` 非法数据地址
- `0x03` 非法数据值

## CRC16 计算

- 多项式：`0xA001`（反向多项式 `0x8005`）
- 初始值：`0xFFFF`
- 不反转输入输出
- 字节序：低字节在前
