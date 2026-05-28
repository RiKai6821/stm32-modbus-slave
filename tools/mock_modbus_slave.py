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
try:
    # pymodbus 2.x
    from pymodbus.server.sync import StartSerialServer
    from pymodbus.device import ModbusDeviceIdentification
    from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
    from pymodbus.datastore import ModbusSequentialDataBlock
    from pymodbus.transaction import ModbusRtuFramer
except ImportError:
    # pymodbus 3.x fallback
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

# 只读区上界：地址 [0x0000, DEV_READONLY_END) 不允许外部写入
DEV_READONLY_END = 0x0020

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

        # 固件内部写：绕过只读保护，直接调父类 setValues
        # （STM32 固件可以随时更新自己的只读诊断寄存器，只有外部 Modbus 写才受限）
        fw_set = ModbusSequentialDataBlock.setValues.__get__(regs, type(regs))

        # 通道电压
        mv = [sim_mv(ch, t) for ch in range(4)]
        for i, v in enumerate(mv):
            fw_set(REG_CH1_MV + i + 1, [v])

        # MCU 温度（25℃ ± 3℃）
        temp_x10 = int(250 + 30 * math.sin(0.02 * t))
        fw_set(REG_MCU_TEMP + 1, [temp_x10])

        # ADC 采样计数
        adc_cnt = (adc_cnt + 8) & 0xFFFF
        fw_set(REG_ADC_COUNT + 1, [adc_cnt])

        # 报警检测（读阈值用正常 getValues，阈值区可写，无需绕过）
        hi = [regs.getValues(REG_CH1_HI_THR + i + 1, 1)[0] for i in range(4)]
        lo = [regs.getValues(REG_CH1_LO_THR + i + 1, 1)[0] for i in range(4)]
        status = 0
        for i in range(4):
            if mv[i] > hi[i] or mv[i] < lo[i]:
                status |= (1 << i)
        fw_set(REG_STATUS + 1, [status])

        # 运行时间和统计（均在只读区，用 fw_set 绕过）
        fw_set(REG_UPTIME_S     + 1, [int(t) & 0xFFFF])
        fw_set(REG_FRAME_COUNT  + 1, [frame_count & 0xFFFF])
        fw_set(REG_LATENCY_US   + 1, [latency])
        latency = clamp(latency + int(5 * math.sin(t * 3)), 150, 280)

        time.sleep(0.1)


class ReadonlyProtectedBlock(ModbusSequentialDataBlock):
    """只读区写保护（模拟 STM32 固件的 Modbus_SetReadonly() 行为）。
    地址 [0x0000, DEV_READONLY_END) 写操作返回 False → pymodbus 返回 0x82 异常。
    """
    READONLY_END = DEV_READONLY_END   # = 0x0020

    def validate(self, address, count=1):
        return super().validate(address, count)

    def setValues(self, address, values):
        # address 是 1-based 内部索引，实际寄存器地址 = address - 1
        reg_addr = address - 1
        if reg_addr < self.READONLY_END:
            return False   # 触发 pymodbus 的 IllegalAddress 异常响应
        return super().setValues(address, values)


def init_registers():
    """初始化寄存器（模拟 devregs_init()）"""
    # 创建 holding register block（地址 0 开始，大小 TOTAL_REGS），带只读保护
    block = ReadonlyProtectedBlock(0, [0] * TOTAL_REGS)
    ctx   = ModbusSlaveContext(hr=block, co=ModbusSequentialDataBlock(0, [0]*8))

    # 出厂默认值（直接调父类 setValues 绕过只读保护，用于初始化）
    super_set = ModbusSequentialDataBlock.setValues.__get__(block, ReadonlyProtectedBlock)

    # 只读区 (addr < 0x0020) 用 super_set 初始化，绕过只读拦截
    super_set(REG_FW_VER       + 1, [0x0200])   # 0x0001: 固件版本 v2.0
    # 可写区 (addr >= 0x0020) 直接 setValues
    block.setValues(REG_CH1_HI_THR   + 1, [9000, 9000, 9000, 9000])
    block.setValues(REG_CH1_LO_THR   + 1, [500,  500,  500,  500 ])
    block.setValues(REG_SAMPLE_INTV  + 1, [100])
    block.setValues(REG_AVG_COUNT    + 1, [8])
    block.setValues(REG_WATCHDOG_S   + 1, [0])

    # HardFault 诊断区 (0x0040–0x0045, 只读)
    super_set(REG_FAULT_FLAG   + 1, [0xBAD1])    # 有故障记录
    super_set(REG_FAULT_PC_LO  + 1, [0x1A2C])    # 故障 PC 低16位
    super_set(REG_FAULT_PC_HI  + 1, [0x0800])    # 故障 PC 高16位 → PC=0x08001A2C
    super_set(REG_FAULT_CFSR_LO+ 1, [0x0400])    # SCB->CFSR BFSR: STKERR（栈错误）
    super_set(REG_FAULT_CFSR_HI+ 1, [0x0000])
    block.setValues(REG_FAULT_HFSR   + 1, [0x4000_0000 & 0xFFFF])  # FORCED

    return ctx


def main():
    global store

    import argparse
    parser = argparse.ArgumentParser(description='STM32 Modbus RTU 从机模拟器')
    parser.add_argument('--port', default=PORT,
                        help='从机串口路径（默认: %(default)s）')
    parser.add_argument('--master-port', default=None,
                        help='提示用户用哪个主机端口运行 modbus_test.py')
    args = parser.parse_args()
    slave_port  = args.port
    master_port = args.master_port or slave_port.replace('slave', 'master').replace('stm32', 'pc')

    print("=" * 55)
    print("  STM32 Modbus RTU Slave Simulator v2.1")
    print(f"  Port: {slave_port}  Baud: {BAUD}  Slave ID: {UNIT_ID}")
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
    th = threading.Thread(target=update_loop, daemon=True)
    th.start()
    print(f"[OK] Simulation thread started (100ms update cycle)")
    print(f"[OK] Modbus RTU server starting on {slave_port}...")
    print(f"     In another terminal run:")
    print(f"     SERIAL_PORT={master_port} python tools/modbus_test.py")
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
        port=slave_port,
        baudrate=BAUD,
        timeout=0.1,
    )


if __name__ == '__main__':
    main()
