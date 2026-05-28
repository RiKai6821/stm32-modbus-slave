#!/bin/bash
# ============================================================
# Modbus RTU 从机演示脚本
# 用法: bash demo_modbus.sh
# 效果: 运行 9 项自动化测试，展示寄存器读写、报警、延迟测量
# ============================================================

set -e
PROJ="$(cd "$(dirname "$0")" && pwd)"
SOCAT_PID=""
SLAVE_PID=""

cleanup() {
    echo ""
    echo "[清理] 停止后台进程..."
    [ -n "$SLAVE_PID" ] && kill "$SLAVE_PID" 2>/dev/null || true
    [ -n "$SOCAT_PID" ] && kill "$SOCAT_PID" 2>/dev/null || true
    rm -f /tmp/tty_stm32 /tmp/tty_pc
}
trap cleanup EXIT INT TERM

echo "============================================================"
echo "  STM32 Modbus RTU 从机 v2.1  自动化测试演示"
echo "  展示: DMA TX / Flash 持久化 / HardFault 远程诊断"
echo "============================================================"

# ── 1. 检查依赖 ────────────────────────────────────────────
for pkg in pymodbus serial; do
    python3 -c "import $pkg" 2>/dev/null || {
        echo "[安装] pip install $pkg ..."
        pip3 install "$pkg" -q
    }
done

if ! command -v socat &>/dev/null; then
    echo "[错误] 未安装 socat。请运行: brew install socat"
    exit 1
fi

# ── 2. 创建虚拟串口对 ──────────────────────────────────────
echo ""
echo "[准备] 创建虚拟串口对..."
rm -f /tmp/tty_stm32 /tmp/tty_pc
socat -d -d \
    "pty,raw,echo=0,link=/tmp/tty_stm32" \
    "pty,raw,echo=0,link=/tmp/tty_pc" \
    2>/dev/null &
SOCAT_PID=$!
sleep 0.6
echo "       /tmp/tty_stm32 (从机)  ←→  /tmp/tty_pc (主机)"

# ── 3. 启动 Modbus 从机模拟器 ─────────────────────────────
echo "[准备] 启动 STM32 从机模拟器..."
python3 "$PROJ/tools/mock_modbus_slave.py" \
    --port /tmp/tty_stm32 \
    --master-port /tmp/tty_pc &
SLAVE_PID=$!
sleep 2.5   # 等待 pymodbus server 完全启动

echo "       从机已就绪 PID=$SLAVE_PID"
echo ""
echo "============================================================"
echo "  开始运行测试套件..."
echo "============================================================"
echo ""

# ── 4. 运行测试脚本 ────────────────────────────────────────
SERIAL_PORT=/tmp/tty_pc python3 "$PROJ/tools/modbus_test.py"

echo ""
echo "============================================================"
echo "  演示完成。截图要点："
echo "  - [Test 2] 4 通道电压折线图"
echo "  - [Test 7] 主机端 RTT 和设备端处理延迟对比"
echo "  - [Test 8] 压力测试成功率 & 吞吐量"
echo "  - 最终汇总行: 9/9 通过"
echo "============================================================"
