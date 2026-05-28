"""
mock_modbus_slave.py — 模拟 STM32 Modbus RTU 从机行为（用于截图演示，无需硬件）

原理：用 pymodbus 搭建一个 Modbus RTU 服务器，复现 STM32 固件的寄存器行为：
  - 4 通道电压仿真（正弦波变化）
  - 报警状态字
  - HardFault 诊断寄存器
  - 只读保护（写只读区返回异常码 0x02）

使用步骤（开三个终端）：

  终端1：创建虚拟串口对
    socat -d -d pty,raw,echo=0,link=/tmp/tty_slave pty,raw,echo=0,link=/tmp/tty_master

  终端2：运行本脚本（STM32 从机）
    python tools/mock_modbus_slave.py

  终端3：运行测试脚本（SCADA 主机）
    SERIAL_PORT=/tmp/tty_master python tools/modbus_test.py
"""

import time
import math
import sys
import threading
from pymodbus.server import StartSerialServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.transaction import ModbusRtuFramer

PORT    = '/tmp/tty_slave'
BAUD    = 115200
UNIT_ID = 1

# 寄存器地址（与 device_regs.h 保持一致）
REG_STATUS       = 0x0000
REG_FW_VER       = 0x0001
REG_UPTIME_S     = 0x0002
REG_FRAME_COUNT  = 0x0003
REG_LATENCY_US   = 0x0004
REG_CRC_ERR      = 0x0005
REG_CH1_MV       = 0x0010
REG_CH4_MV       = 0x0013
REG_MCU_TEMP     = 0x0014
REG_ADC_COUNT    = 0x0015
REG_CH1_HI_THR   = 0x0020
REG_CH4_HI_THR   = 0x0023
REG_CH1_LO_THR   = 0x0024
REG_CH4_LO_THR   = 0x0027
REG_SAMPLE_INTV  = 0x0030
REG_AVG_COUNT    = 0x0031
REG_WATCHDOG_S   = 0x0032
REG_SAVE_CONFIG  = 0x0033
REG_FAULT_FLAG   = 0x0040
REG_FAULT_PC_LO  = 0x0041
REG_FAULT_PC_HI  = 0x0042
REG_FAULT_CFSR_LO= 0x0043
REG_FAULT_CFSR_HI= 0x0044
REG_FAULT_HFSR   = 0x0045
TOTAL_REGS       = 0x0046

# 仿真 ADC 参数（与 device_regs.c 完全相同）
SIM_CHANNELS = [
    {'center': 4500, 'amp': 4000, 'freq': 0.10, 'cos': False},  # CH1
    {'center': 7000, 'amp': 2000, 'freq': 0.23, 'cos': False},  # CH2
    {'center': 2000, 'amp': 1500, 'freq': 0.50, 'cos': True },  # CH3
    {'center': 5000, 'amp':  100, 'freq': 2.00, 'cos': False},  # CH4
]

store       = None
start_time  = time.time()
frame_count = 0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def sim_mv(ch_idx, t):
    c = SIM_CHANNELS[ch_idx]
    w = 2 * math.pi * c['freq']
    val = c['center'] + c['amp'] * (math.cos(w * t) if c['cos'] else math.sin(w * t))
    return int(clamp(val, 0, 10000))


def update_loop():
    """后台线程：每 100ms 更新仿真数据，模拟 STM32 devregs_update()"""
    global frame_count
    adc_cnt  = 0
    latency  = 180  # μs，模拟 DWT 测得的处理延迟

    while True:
        t    = time.time() - start_time
        regs = store.store['h']  # holding registers

        # 通道电压
        mv = [sim_mv(ch, t) for ch in range(4)]
        for i, v in enumerate(mv):
            regs.setValues(REG_CH1_MV + i + 1, [v])

        # MCU 温度（25℃ ± 3℃）
        temp_x10 = int(250 + 30 * math.sin(0.02 * t))
        regs.setValues(REG_MCU_TEMP + 1, [temp_x10])

        # ADC 采样计数
        adc_cnt = (adc_cnt + 8) & 0xFFFF
        regs.setValues(REG_ADC_COUNT + 1, [adc_cnt])

        # 报警检测
        hi = [regs.getValues(REG_CH1_HI_THR + i + 1, 1)[0] for i in range(4)]
        lo = [regs.getValues(REG_CH1_LO_THR + i + 1, 1)[0] for i in range(4)]
        status = 0
        for i in range(4):
            if mv[i] > hi[i] or mv[i] < lo[i]:
                status |= (1 << i)
        regs.setValues(REG_STATUS + 1, [status])

        # 运行时间和统计
        regs.setValues(REG_UPTIME_S     + 1, [int(t) & 0xFFFF])
        regs.setValues(REG_FRAME_COUNT  + 1, [frame_count & 0xFFFF])
        regs.setValues(REG_LATENCY_US   + 1, [latency])
        latency = clamp(latency + int(5 * math.sin(t * 3)), 150, 280)

        time.sleep(0.1)


def init_registers():
    """初始化寄存器（模拟 devregs_init()）"""
    # 创建 holding register block（地址 0 开始，大小 TOTAL_REGS）
    block = ModbusSequentialDataBlock(0, [0] * TOTAL_REGS)
    ctx   = ModbusSlaveContext(hr=block, co=ModbusSequentialDataBlock(0, [0]*8))

    # 出厂默认值
    block.setValues(REG_FW_VER       + 1, [0x0200])  # v2.0
    block.setValues(REG_CH1_HI_THR   + 1, [9000, 9000, 9000, 9000])
    block.setValues(REG_CH1_LO_THR   + 1, [500, 500, 500, 500])
    block.setValues(REG_SAMPLE_INTV  + 1, [100])
    block.setValues(REG_AVG_COUNT    + 1, [8])
    block.setValues(REG_WATCHDOG_S   + 1, [0])

    # 模拟上电检测到 HardFault 记录（演示用）
    # 模拟一次空指针访问：PC=0x0800_1A2C, CFSR bit4=STKERR（栈错误）
    block.setValues(REG_FAULT_FLAG   + 1, [0xBAD1])
    block.setValues(REG_FAULT_PC_LO  + 1, [0x1A2C])
    block.setValues(REG_FAULT_PC_HI  + 1, [0x0800])
    block.setValues(REG_FAULT_CFSR_LO+ 1, [0x0400])  # BFSR: STKERR
    block.setValues(REG_FAULT_CFSR_HI+ 1, [0x0000])
    block.setValues(REG_FAULT_HFSR   + 1, [0x4000_0000 & 0xFFFF])  # FORCED

    return ctx


def main():
    global store

    print("=" * 55)
    print("  STM32 Modbus RTU Slave Simulator v2.1")
    print(f"  Port: {PORT}  Baud: {BAUD}  Slave ID: {UNIT_ID}")
    print("=" * 55)
    print()
    print("Register map (subset):")
    print("  0x0000  STATUS   (alarm bits CH1-CH4)")
    print("  0x0001  FW_VER   = 0x0200 (v2.0)")
    print("  0x0010  CH1-CH4  voltage (mV, simulated sine)")
    print("  0x0020  CH1-CH4  high alarm thresholds (mV)")
    print("  0x0040  FAULT    HardFault diagnostics (PC, CFSR)")
    print()

    ctx = init_registers()
    store = ctx
    server_ctx = ModbusServerContext(slaves={UNIT_ID: ctx}, single=False)

    # 启动后台数据更新线程
    t = threading.Thread(target=update_loop, daemon=True)
    t.start()
    print(f"[OK] Simulation thread started (100ms update cycle)")
    print(f"[OK] Modbus RTU server starting on {PORT}...")
    print(f"     Run: SERIAL_PORT={PORT.replace('slave', 'master')} python tools/modbus_test.py")
    print()

    identity = ModbusDeviceIdentification()
    identity.VendorName  = 'STM32 Simulator'
    identity.ProductName = '4-Channel Voltage Acquisition Module'
    identity.ModelName   = 'STM32F103C8T6'
    identity.MajorMinorRevision = '2.1'

    StartSerialServer(
        context=server_ctx,
        framer=ModbusRtuFramer,
        identity=identity,
        port=PORT,
        baudrate=BAUD,
        timeout=0.1,
    )


if __name__ == '__main__':
    main()
