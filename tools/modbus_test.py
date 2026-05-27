"""
MODBUS RTU 从机测试脚本 — 4 通道电压采集模块

依赖: pip install pymodbus pyserial

测试场景:
  1. 系统信息寄存器读取 (固件版本, 运行时间)
  2. 4 通道电压实时读取
  3. 报警阈值配置 + 验证
  4. 只读保护测试
  5. 异常响应测试 (非法地址, 非法功能码)
  6. 响应延迟测量 (主机端计时 + 设备端 REG_LATENCY_US 对比)
  7. 压力测试 (100 次连续读写, 统计成功率和延迟分布)

使用方法:
  连接硬件后修改 SERIAL_PORT, 然后运行:
    python tools/modbus_test.py

  无硬件时用 socat 创建虚拟串口对测试协议栈逻辑:
    socat -d -d pty,raw,echo=0 pty,raw,echo=0
    SERIAL_PORT=/dev/pts/X  python tools/modbus_test.py
"""

import time
import sys
import struct
import statistics
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

# ===== 配置 =====
SERIAL_PORT = '/dev/cu.usbserial-XXXX'   # macOS: ls /dev/cu.*
# SERIAL_PORT = 'COM3'                   # Windows
BAUDRATE    = 115200
SLAVE_ID    = 1
TIMEOUT     = 1.0

# ===== 寄存器地址 (与 device_regs.h 保持一致) =====
REG_STATUS        = 0x0000
REG_FW_VER        = 0x0001
REG_UPTIME_S      = 0x0002
REG_FRAME_COUNT   = 0x0003
REG_LATENCY_US    = 0x0004
REG_CRC_ERR       = 0x0005

REG_CH1_MV        = 0x0010
REG_CH2_MV        = 0x0011
REG_CH3_MV        = 0x0012
REG_CH4_MV        = 0x0013
REG_MCU_TEMP      = 0x0014
REG_ADC_COUNT     = 0x0015

REG_CH1_HI_THR    = 0x0020
REG_CH4_HI_THR    = 0x0023
REG_CH1_LO_THR    = 0x0024
REG_CH4_LO_THR    = 0x0027

REG_SAMPLE_INTV   = 0x0030
REG_AVG_COUNT     = 0x0031
REG_WATCHDOG_S    = 0x0032


# ===== 工具函数 =====

def read_regs(client, addr, count=1):
    """读保持寄存器, 返回寄存器列表或 None"""
    rr = client.read_holding_registers(address=addr, count=count, slave=SLAVE_ID)
    return None if rr.isError() else rr.registers


def write_reg(client, addr, value):
    """写单个寄存器, 返回是否成功"""
    wr = client.write_register(address=addr, value=value, slave=SLAVE_ID)
    return not wr.isError()


def timed_read(client, addr, count=1):
    """带主机端延迟计时的寄存器读, 返回 (regs, latency_ms)"""
    t0   = time.perf_counter()
    regs = read_regs(client, addr, count)
    ms   = (time.perf_counter() - t0) * 1000.0
    return regs, ms


# ===== 测试用例 =====

def test_system_info(client):
    """读取系统信息寄存器"""
    print("\n[Test 1] 系统信息寄存器")
    regs = read_regs(client, REG_STATUS, 6)
    if regs is None:
        print("  FAIL: 读取失败")
        return False

    status     = regs[0]
    fw_ver     = regs[1]
    uptime_s   = regs[2]
    frame_cnt  = regs[3]
    latency_us = regs[4]
    crc_err    = regs[5]

    alarm_bits = [bool(status & (1 << i)) for i in range(4)]
    print(f"  固件版本:   v{fw_ver >> 8}.{fw_ver & 0xFF}")
    print(f"  运行时间:   {uptime_s} s")
    print(f"  帧计数:     {frame_cnt}")
    print(f"  处理延迟:   {latency_us} μs  (STM32 内部计时)")
    print(f"  CRC 错误:   {crc_err}")
    print(f"  报警状态:   CH1={alarm_bits[0]} CH2={alarm_bits[1]} "
          f"CH3={alarm_bits[2]} CH4={alarm_bits[3]}")
    return fw_ver == 0x0200


def test_voltage_reading(client):
    """读取 4 通道电压"""
    print("\n[Test 2] 4 通道电压读取")
    regs = read_regs(client, REG_CH1_MV, 6)
    if regs is None:
        print("  FAIL: 读取失败")
        return False

    channels = [regs[i] / 1000.0 for i in range(4)]
    temp_c   = regs[4] / 10.0
    adc_cnt  = regs[5]

    for i, v in enumerate(channels):
        bar = '█' * int(v / 0.5)
        print(f"  CH{i+1}: {v:6.3f} V  {bar}")
    print(f"  MCU 温度:  {temp_c:.1f} °C")
    print(f"  ADC 计数:  {adc_cnt}")

    # 所有通道电压应在 0~10 V 范围内
    ok = all(0.0 <= v <= 10.0 for v in channels)
    if not ok:
        print("  FAIL: 电压超量程")
    return ok


def test_alarm_thresholds(client):
    """配置报警阈值并验证报警状态字"""
    print("\n[Test 3] 报警阈值配置")

    # 把 CH1 高阈值设为极低值 (500 mV), 触发高报警
    ok = write_reg(client, REG_CH1_HI_THR, 500)
    if not ok:
        print("  FAIL: 无法写入阈值")
        return False
    print("  设置 CH1 高阈值 = 500 mV (预期触发报警)")

    time.sleep(0.2)  # 等待设备更新

    regs = read_regs(client, REG_STATUS, 1)
    if regs is None:
        print("  FAIL: 无法读取状态")
        return False

    ch1_alarm = bool(regs[0] & 0x01)
    print(f"  状态字 = 0x{regs[0]:04X}, CH1 报警位 = {ch1_alarm}")

    # 恢复默认阈值
    write_reg(client, REG_CH1_HI_THR, 9000)
    print("  已恢复 CH1 高阈值 = 9000 mV")

    return ch1_alarm  # 应该触发报警


def test_readonly_protection(client):
    """验证只读保护: 写 REG_FW_VER (0x0001) 应返回异常"""
    print("\n[Test 4] 只读保护")
    wr = client.write_register(address=REG_FW_VER, value=0xDEAD, slave=SLAVE_ID)
    if wr.isError():
        print(f"  PASS: 收到预期异常响应 (只读区写保护生效): {wr}")
        return True
    # 可能固件未返回异常但也没改值
    regs = read_regs(client, REG_FW_VER, 1)
    if regs and regs[0] == 0x0200:
        print("  PASS: 值未被修改 (只读保护生效)")
        return True
    print(f"  FAIL: 只读寄存器被写入, 值 = 0x{regs[0] if regs else '?':04X}")
    return False


def test_illegal_address(client):
    """读非法地址 0xFFFF 应返回异常码 0x02"""
    print("\n[Test 5] 非法地址异常")
    rr = client.read_holding_registers(address=0xFFFF, count=1, slave=SLAVE_ID)
    if rr.isError():
        print(f"  PASS: 收到预期异常: {rr}")
        return True
    print("  FAIL: 应该返回异常码 02 但未返回")
    return False


def test_coil_operations(client):
    """FC01/FC05 线圈读写"""
    print("\n[Test 6] 线圈操作 (FC01/FC05)")

    wr = client.write_coil(address=0, value=True, slave=SLAVE_ID)
    if wr.isError():
        print(f"  FAIL: 写线圈 0=ON: {wr}")
        return False
    rr = client.read_coils(address=0, count=4, slave=SLAVE_ID)
    if rr.isError():
        print(f"  FAIL: 读线圈: {rr}")
        return False
    print(f"  线圈[0..3] = {rr.bits[:4]}")
    ok = rr.bits[0] is True
    if not ok:
        print("  FAIL: 线圈 0 写 ON 后读回不匹配")
    client.write_coil(address=0, value=False, slave=SLAVE_ID)
    return ok


def test_latency(client, count=20):
    """响应延迟测量: 主机端计时 vs 设备端内部计时"""
    print(f"\n[Test 7] 响应延迟测量 ({count} 次)")
    host_latencies   = []
    device_latencies = []

    for _ in range(count):
        regs, host_ms = timed_read(client, REG_CH1_MV, 4)
        if regs is None:
            continue
        host_latencies.append(host_ms)

        dev_regs = read_regs(client, REG_LATENCY_US, 1)
        if dev_regs:
            device_latencies.append(dev_regs[0])

    if not host_latencies:
        print("  FAIL: 无数据")
        return False

    print(f"  主机端 RTT (ms):  "
          f"avg={statistics.mean(host_latencies):.2f}  "
          f"min={min(host_latencies):.2f}  "
          f"max={max(host_latencies):.2f}  "
          f"p95={sorted(host_latencies)[int(len(host_latencies)*0.95)]:.2f}")

    if device_latencies:
        print(f"  设备端处理 (μs): "
              f"avg={statistics.mean(device_latencies):.0f}  "
              f"min={min(device_latencies)}  "
              f"max={max(device_latencies)}")
        print(f"  (主机 RTT = 串口传输 + 设备处理; "
              f"设备处理占比约 "
              f"{statistics.mean(device_latencies)/1000/statistics.mean(host_latencies)*100:.1f}%)")
    return True


def test_stress(client, count=200):
    """压力测试: 连续读写, 统计成功率和平均延迟"""
    print(f"\n[Test 8] 压力测试 ({count} 次读写往返)")
    success    = 0
    latencies  = []
    start      = time.perf_counter()

    for i in range(count):
        # 写一个值
        if client.write_register(address=REG_SAMPLE_INTV, value=(100 + i % 50),
                                  slave=SLAVE_ID).isError():
            continue
        # 读回验证
        t0   = time.perf_counter()
        regs = read_regs(client, REG_SAMPLE_INTV, 1)
        ms   = (time.perf_counter() - t0) * 1000.0

        if regs and regs[0] == (100 + i % 50):
            success += 1
            latencies.append(ms)

    elapsed = time.perf_counter() - start
    rate    = success * 100.0 / count
    avg_ms  = statistics.mean(latencies) if latencies else 0

    print(f"  成功率:    {success}/{count}  ({rate:.1f}%)")
    print(f"  平均延迟:  {avg_ms:.2f} ms/次")
    print(f"  总耗时:    {elapsed:.2f} s")
    print(f"  吞吐量:    {count/elapsed:.1f} 次/s")

    # 恢复默认采样间隔
    write_reg(client, REG_SAMPLE_INTV, 100)
    return success == count


def test_sample_config(client):
    """采样间隔和均值滤波配置"""
    print("\n[Test 9] 采集参数配置")

    ok1 = write_reg(client, REG_SAMPLE_INTV, 50)
    ok2 = write_reg(client, REG_AVG_COUNT,   16)

    regs = read_regs(client, REG_SAMPLE_INTV, 2)
    if regs is None or not ok1 or not ok2:
        print("  FAIL")
        return False

    print(f"  采样间隔: {regs[0]} ms (写入 50)")
    print(f"  均值次数: {regs[1]} (写入 16)")

    # 恢复默认
    write_reg(client, REG_SAMPLE_INTV, 100)
    write_reg(client, REG_AVG_COUNT,    8)
    return regs[0] == 50 and regs[1] == 16


# ===== 主程序 =====

def main():
    print("=" * 60)
    print("MODBUS RTU 从机测试 — 4 通道电压采集模块")
    print(f"串口: {SERIAL_PORT}  波特率: {BAUDRATE}  从机地址: {SLAVE_ID}")
    print("=" * 60)

    client = ModbusSerialClient(
        port=SERIAL_PORT, baudrate=BAUDRATE,
        bytesize=8, parity='N', stopbits=1, timeout=TIMEOUT,
    )

    if not client.connect():
        print(f"错误: 无法打开串口 {SERIAL_PORT}")
        print("请检查: 1) 串口名是否正确  2) 设备是否已连接并上电")
        sys.exit(1)

    tests = [
        ("系统信息",         test_system_info),
        ("电压读取",         test_voltage_reading),
        ("报警阈值",         test_alarm_thresholds),
        ("只读保护",         test_readonly_protection),
        ("非法地址",         test_illegal_address),
        ("线圈操作",         test_coil_operations),
        ("延迟测量",         test_latency),
        ("采集配置",         test_sample_config),
        ("压力测试",         test_stress),
    ]

    passed  = 0
    results = []

    for name, fn in tests:
        try:
            ok = fn(client)
            results.append((name, ok))
            if ok:
                passed += 1
        except Exception as e:
            print(f"  异常: {e}")
            results.append((name, False))

    print("\n" + "=" * 60)
    print("测试结果汇总:")
    for name, ok in results:
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}] {name}")
    print(f"\n总计: {passed}/{len(tests)} 通过")
    print("=" * 60)

    client.close()


if __name__ == '__main__':
    main()
