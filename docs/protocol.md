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

### 0x01 读线圈

**请求**：
```
| Slave | 0x01 | Addr Hi | Addr Lo | Count Hi | Count Lo | CRC Lo | CRC Hi |
```

**响应**：
```
| Slave | 0x01 | Byte Cnt | Data ... | CRC Lo | CRC Hi |
```

线圈数据从低位开始排列，多余位填 0。

### 0x05 写单个线圈

**请求**：
```
| Slave | 0x05 | Addr Hi | Addr Lo | Value Hi | Value Lo | CRC Lo | CRC Hi |
```

Value = `0xFF00` 表示 ON，`0x0000` 表示 OFF，其他值返回异常码 0x03。

**响应**（回显请求）：
```
| Slave | 0x05 | Addr Hi | Addr Lo | Value Hi | Value Lo | CRC Lo | CRC Hi |
```

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

当请求处理失败时，从机返回异常响应（共 5 字节）：

```
┌────────────┬──────────────────────┬────────────────┬─────────┬─────────┐
│ Slave Addr │ FuncCode | 0x80      │ Exception Code │ CRC Lo  │ CRC Hi  │
│   1 byte   │       1 byte         │    1 byte      │ 1 byte  │ 1 byte  │
└────────────┴──────────────────────┴────────────────┴─────────┴─────────┘
```

`FuncCode | 0x80`：将原请求的功能码最高位置 1，**整体是 1 个字节**。
示例：FC03 请求出错，异常响应的功能码字节为 `0x03 | 0x80 = 0x83`。

异常码：
- `0x01` 非法功能码
- `0x02` 非法数据地址
- `0x03` 非法数据值

## CRC16 计算

- 多项式：`0xA001`（反向多项式 `0x8005`）
- 初始值：`0xFFFF`
- 不反转输入输出
- 字节序：低字节在前
