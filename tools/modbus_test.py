"""
MODBUS RTU 主机测试脚本
依赖: pip install pymodbus pyserial
用途: 验证 STM32 从机协议栈的正确性

测试场景:
1. 读保持寄存器 (功能码 0x03)
2. 写单个寄存器 (功能码 0x06)
3. 写多个寄存器 (功能码 0x10)
4. 异常响应（非法地址、非法功能码）
5. 压力测试（连续读写）
"""

import time
import sys
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

# ===== 配置 =====
SERIAL_PORT = '/dev/cu.usbserial-XXXX'  # Mac: ls /dev/cu.* 找到对应口
# SERIAL_PORT = 'COM3'                  # Windows: 设备管理器查看
BAUDRATE = 115200
SLAVE_ID = 1
TIMEOUT = 1


def test_read_holding(client):
    """测试读保持寄存器"""
    print("\n[Test 1] 读保持寄存器 0x0000 ~ 0x0003 (系统状态)")
    rr = client.read_holding_registers(address=0x0000, count=4, slave=SLAVE_ID)
    if rr.isError():
        print(f"  失败: {rr}")
        return False
    print(f"  版本号: 0x{rr.registers[0]:04X}")
    print(f"  运行计数: {rr.registers[1]}")
    print(f"  保留1: {rr.registers[2]}, 保留2: {rr.registers[3]}")
    return True


def test_read_sim_data(client):
    """读取模拟数据"""
    print("\n[Test 2] 读模拟传感器数据 0x0020 ~ 0x0021")
    rr = client.read_holding_registers(address=0x0020, count=2, slave=SLAVE_ID)
    if rr.isError():
        print(f"  失败: {rr}")
        return False
    print(f"  温度: {rr.registers[0] / 10.0:.1f} ℃")
    print(f"  电压: {rr.registers[1]} mV")
    return True


def test_write_single(client):
    """测试写单个寄存器"""
    print("\n[Test 3] 写单个寄存器 0x0010 = 0xABCD")
    wr = client.write_register(address=0x0010, value=0xABCD, slave=SLAVE_ID)
    if wr.isError():
        print(f"  写失败: {wr}")
        return False
    # 回读验证
    rr = client.read_holding_registers(address=0x0010, count=1, slave=SLAVE_ID)
    if rr.registers[0] == 0xABCD:
        print(f"  写入并回读成功: 0x{rr.registers[0]:04X}")
        return True
    else:
        print(f"  回读不匹配: 0x{rr.registers[0]:04X}")
        return False


def test_write_multi(client):
    """测试写多个寄存器"""
    print("\n[Test 4] 写多个寄存器 0x0010 ~ 0x0013 = [0x1111, 0x2222, 0x3333, 0x4444]")
    values = [0x1111, 0x2222, 0x3333, 0x4444]
    wr = client.write_registers(address=0x0010, values=values, slave=SLAVE_ID)
    if wr.isError():
        print(f"  写失败: {wr}")
        return False
    rr = client.read_holding_registers(address=0x0010, count=4, slave=SLAVE_ID)
    if list(rr.registers) == values:
        print(f"  写入并回读成功: {[hex(v) for v in rr.registers]}")
        return True
    else:
        print(f"  回读不匹配: {[hex(v) for v in rr.registers]}")
        return False


def test_exception_illegal_addr(client):
    """测试异常响应：非法地址"""
    print("\n[Test 5] 异常响应测试: 读非法地址 0xFFFF")
    rr = client.read_holding_registers(address=0xFFFF, count=1, slave=SLAVE_ID)
    if rr.isError():
        print(f"  收到预期的异常响应: {rr}")
        return True
    else:
        print(f"  应该返回异常但没有")
        return False


def test_stress(client, count=100):
    """压力测试：连续读写"""
    print(f"\n[Test 6] 压力测试: 连续 {count} 次读写")
    success = 0
    start = time.time()
    for i in range(count):
        wr = client.write_register(address=0x0010, value=i & 0xFFFF, slave=SLAVE_ID)
        if wr.isError():
            continue
        rr = client.read_holding_registers(address=0x0010, count=1, slave=SLAVE_ID)
        if not rr.isError() and rr.registers[0] == (i & 0xFFFF):
            success += 1
    elapsed = time.time() - start
    print(f"  成功率: {success}/{count} ({success*100/count:.1f}%)")
    print(f"  耗时: {elapsed:.2f}s, 平均: {elapsed*1000/count:.1f}ms/次")
    return success == count


def main():
    print("=" * 50)
    print("MODBUS RTU 从机测试")
    print(f"串口: {SERIAL_PORT}  波特率: {BAUDRATE}  从机地址: {SLAVE_ID}")
    print("=" * 50)
    
    client = ModbusSerialClient(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=TIMEOUT,
    )
    
    if not client.connect():
        print(f"无法打开串口 {SERIAL_PORT}")
        sys.exit(1)
    
    tests = [
        test_read_holding,
        test_read_sim_data,
        test_write_single,
        test_write_multi,
        test_exception_illegal_addr,
        test_stress,
    ]
    
    passed = 0
    for test in tests:
        try:
            if test(client):
                passed += 1
        except Exception as e:
            print(f"  异常: {e}")
    
    print("\n" + "=" * 50)
    print(f"测试结果: {passed}/{len(tests)} 通过")
    print("=" * 50)
    
    client.close()


if __name__ == '__main__':
    main()
