# STM32 MODBUS RTU 从机协议栈

> 基于 STM32F103 的 MODBUS RTU 从机实现。寄存器级 UART/TIM 驱动 + 便携式协议栈 + 4 通道工业电压采集模拟，无 HAL 依赖，含完整的 Python 测试脚本。

## 项目简介

本项目实现一个模拟 4 通道 0~10V 工业模拟量输入模块（类比 Phoenix Contact AXL F AI4）的 MODBUS RTU 从机。

代码分三层独立实现：

| 层次 | 文件 | 关键技术点 |
|------|------|------------|
| 寄存器驱动 | `uart_drv.c`, `timer_drv.c` | 直接操作 USART1/TIM2 寄存器，不调用任何 HAL 函数 |
| 协议栈 | `modbus_slave.c` | HAL 无关，通过函数指针调用驱动，可在任意平台移植 |
| 应用层 | `device_regs.c`, `main.c` | 寄存器映射、ADC 均值滤波、报警检测、看门狗 |

## 功能特性

- 支持 FC01、FC03、FC05、FC06、FC10 功能码
- 标准 MODBUS CRC16（查表法，64 字节帧仅需 64 次 XOR，比逐位计算快 8 倍）
- 基于 TIM2 单次模式的精确 3.5 字符帧间隔检测
- RS485 半双工：等待 TC 位（非 TXE）后切换 DE，防止截断最后一位
- 响应处理延迟统计（DWT 周期计数器，精度 ±1 μs）
- 只读区保护、广播地址支持、ORE 溢出自动恢复
- 4 通道均值滤波（可配置 1~64 次）+ 独立高/低报警阈值
- 看门狗：主机超时未通信可触发软复位（REG_WATCHDOG_S）

## 软件架构

```
┌───────────────────────────────────────────────────────────┐
│                    Application Layer                       │
│  device_regs.c: ADC sim, alarm detect, watchdog           │
├───────────────────────────────────────────────────────────┤
│               MODBUS RTU Protocol Stack                    │
│  modbus_slave.c: frame parse, FC dispatch, CRC16          │
│  ↕ 函数指针 (send / send_done / timer_restart / get_us)   │
├────────────────────┬──────────────────────────────────────┤
│   uart_drv.c       │   timer_drv.c                        │
│   USART1 寄存器级   │   TIM2 单次模式                       │
│   RCC+GPIO+BRR+CR1 │   PSC=71(1μs/tick), ARR=1749        │
└────────────────────┴──────────────────────────────────────┘
         PA9/PA10/PA8               TIM2_IRQn
         USART1_IRQn                3.5 char timeout
```

## 寄存器映射

| 地址 | 访问 | 描述 |
|------|------|------|
| **系统信息区** | | |
| 0x0000 | RO | 状态字: bit[3:0] = CH1~CH4 超限报警标志 |
| 0x0001 | RO | 固件版本: 0x0200 = v2.0 |
| 0x0002 | RO | 运行时间（秒，16 位循环） |
| 0x0003 | RO | MODBUS 帧计数（低 16 位） |
| 0x0004 | RO | **上次响应处理延迟（μs）** — 主机可读取评估通信质量 |
| 0x0005 | RO | CRC 错误累计次数 |
| **测量数据区** | | |
| 0x0010 | RO | CH1 电压（mV，0~10000） |
| 0x0011 | RO | CH2 电压（mV，0~10000） |
| 0x0012 | RO | CH3 电压（mV，0~10000） |
| 0x0013 | RO | CH4 电压（mV，0~10000） |
| 0x0014 | RO | MCU 内部温度（°C×10） |
| 0x0015 | RO | ADC 采样计数（低 16 位） |
| **报警阈值区** | | |
| 0x0020~0x0023 | RW | CH1~CH4 高报警阈值（mV，默认 9000） |
| 0x0024~0x0027 | RW | CH1~CH4 低报警阈值（mV，默认 500） |
| **采集配置区** | | |
| 0x0030 | RW | 采样间隔（ms，默认 100，最小 10） |
| 0x0031 | RW | 均值滤波次数（默认 8，范围 1~64） |
| 0x0032 | RW | 看门狗超时（秒，0=禁用） |

只读边界：地址 `[0x0000, 0x001F]` 主机写入将返回 Modbus 异常码 0x02（非法地址）。

## 驱动实现细节

### USART1 驱动（uart_drv.c）

```c
/* 波特率寄存器: BRR = f_PCLK2 / BAUD (四舍五入) */
USART1->BRR = (APB2_HZ + BAUD / 2) / BAUD;

/* GPIO 复用配置: PA9 = 复用推挽 50MHz (CRH = 0xB), PA10 = 浮空输入 (0x4) */
GPIOA->CRH &= ~(0xFFFu << 0);
GPIOA->CRH |=  (0x2u << 0) | (0xBu << 4) | (0x4u << 8);
```

RS485 切换时机：
- **不能**在 `TXE=1` 时拉低 DE（移位寄存器仍在发送最后一字节）
- **必须**等待 `TC=1`（Transmission Complete）后再拉低 DE

### TIM2 驱动（timer_drv.c）

```c
/* 3.5 字符超时 (115200 bps → 1750 μs, 规范最小值) */
TIM2->PSC  = 71;            /* 72 MHz / 72 = 1 MHz → 1 μs/tick */
TIM2->ARR  = 1749;          /* 1750 μs */
TIM2->CR1  = TIM_CR1_OPM;  /* 单次模式: 溢出后自动停止 */
TIM2->EGR  = TIM_EGR_UG;   /* 强制更新, 让 PSC/ARR 立即生效 */
```

每次收到一字节后调用 `tim2_restart()` 重置计数器；超时无新字节则 `TIM2_IRQn` 触发 `Modbus_OnTimeout()`。

### 协议栈可移植性

`ModbusSlave_t` 结构体不持有任何硬件句柄，驱动通过函数指针注入：

```c
Modbus_Init(&g_modbus,
    uart1_send_buf,   /* send:         写 USART1->DR  */
    uart1_wait_tc,    /* send_done:    等待 TC + DE 低 */
    tim2_restart,     /* timer_restart */
    tim2_stop,        /* timer_stop    */
    get_us);          /* 微秒时间戳    */
```

换平台时只需替换这 5 个函数，协议栈逻辑完全不变。

### 延迟统计（DWT）

```c
/* Cortex-M3 周期计数器: 72 MHz 下 1 μs = 72 cycles */
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

uint32_t get_us(void) { return DWT->CYCCNT / 72; }
```

`last_proc_us` 记录从接收到完整帧到构建好响应帧的耗时。测试数据（115200 bps）：

| 操作 | 典型延迟 |
|------|---------|
| FC03 读 4 寄存器 | ~8 μs |
| FC06 写单寄存器 | ~6 μs |
| FC10 写 4 寄存器 | ~12 μs |
| 主机端 RTT（含串口传输） | ~3~5 ms |

## 引脚配置

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA8 | RS485 DE | 高=发送，低=接收 |
| PA9 | USART1 TX | MODBUS 发送 |
| PA10 | USART1 RX | MODBUS 接收 |
| PC13 | LED | 每收到一帧翻转 |

## 快速上手

### 编译（STM32CubeIDE）

1. 新建 STM32CubeIDE 项目，芯片 **STM32F103C8T6**
2. 在 CubeMX 中：**不需要配置 USART1 和 TIM2**（驱动自行初始化）
3. 将 `Core/Src/*.c` 和 `Core/Inc/*.h` 加入工程
4. 编译时加入 `-lm`（`device_regs.c` 使用了 `sinf/cosf`）
5. 通过 ST-Link 烧录

### 运行 Python 测试脚本

```bash
pip install pymodbus pyserial

# 修改 tools/modbus_test.py 中的 SERIAL_PORT, 然后:
python tools/modbus_test.py
```

典型输出：

```
============================================================
MODBUS RTU 从机测试 — 4 通道电压采集模块
串口: /dev/cu.usbserial-XXXX  波特率: 115200  从机地址: 1
============================================================

[Test 1] 系统信息寄存器
  固件版本:   v2.0
  运行时间:   47 s
  帧计数:     312
  处理延迟:   8 μs  (STM32 内部计时)
  CRC 错误:   0
  报警状态:   CH1=False CH2=False CH3=False CH4=False

[Test 2] 4 通道电压读取
  CH1:  4.821 V  █████████
  CH2:  8.234 V  ████████████████
  CH3:  2.109 V  ████
  CH4:  4.998 V  █████████

[Test 7] 响应延迟测量 (20 次)
  主机端 RTT (ms):  avg=3.82  min=3.51  max=4.23  p95=4.18
  设备端处理 (μs): avg=8  min=7  max=11
  (主机 RTT = 串口传输 + 设备处理; 设备处理占比约 0.2%)

[Test 8] 压力测试 (200 次读写往返)
  成功率:    200/200  (100.0%)
  平均延迟:  3.84 ms/次
  总耗时:    1.97 s
  吞吐量:    101.5 次/s

============================================================
测试结果汇总:
  [PASS] 系统信息
  [PASS] 电压读取
  [PASS] 报警阈值
  [PASS] 只读保护
  [PASS] 非法地址
  [PASS] 线圈操作
  [PASS] 延迟测量
  [PASS] 采集配置
  [PASS] 压力测试

总计: 9/9 通过
============================================================
```

## 项目结构

```
stm32-modbus-slave/
├── Core/
│   ├── Inc/
│   │   ├── uart_drv.h      ← USART1 寄存器级驱动接口
│   │   ├── timer_drv.h     ← TIM2 帧间隔定时器接口
│   │   ├── device_regs.h   ← 设备寄存器映射和地址定义
│   │   └── modbus_slave.h  ← 协议栈 (HAL 无关)
│   └── Src/
│       ├── uart_drv.c      ← USART1 实现 (直接操作寄存器)
│       ├── timer_drv.c     ← TIM2 实现 (直接操作寄存器)
│       ├── device_regs.c   ← 4 通道仿真、报警检测、看门狗
│       ├── modbus_slave.c  ← 协议帧解析、FC 分发、CRC16
│       └── main.c          ← 系统初始化 (PLL + SysTick + DWT)
├── tools/
│   └── modbus_test.py      ← 9 项自动化测试 + 延迟统计
├── docs/
│   └── protocol.md
└── README.md
```

## 技术说明

### 为什么不用 HAL？

HAL 是通用框架，屏蔽了硬件细节，代价是：
- `HAL_UART_Transmit()` 内部有额外状态检查，对时序敏感的 RS485 方向切换不可控
- `HAL_TIM_Base_Start_IT()` 每次调用会重新配置 DIER，有额外开销

直接操作寄存器，每个操作的指令数可数，行为完全可预测。对 MODBUS 3.5 字符超时这种微秒级时序，这很重要。

### TXE vs TC 的区别

```
写 DR → [TXE=1, 移位寄存器开始发送] → 最后一位发出 → [TC=1]
         ↑                                              ↑
      可以写下一字节                           此时才能切换 RS485 DE
```

如果在 TXE=1 时就切换 DE（低），移位寄存器仍在发送，RS485 线路会立即变为接收状态，导致最后 1~2 位被截断，从机响应帧损坏。

### CRC16 查表法性能

```
逐位计算: 每字节 8 次循环 → 256 字节帧 = 2048 次运算
查表法:   每字节 1 次查表 + 1 次 XOR → 256 字节帧 = 256 次运算
```

对于 115200 bps，每字节 ~86 μs 到达，协议栈处理时间必须远小于此，否则会丢帧。

### MODBUS 广播地址（0x00）

从机接受地址 0x00 的帧并执行（如写寄存器），但**不回应**。用于主机批量设置所有从机参数而不产生冲突。本实现在 `Modbus_Poll()` 中正确处理此情况。

## 许可证

MIT
