"""
modbus_monitor.py — Modbus RTU 实时电压监控面板

功能：
  - 4 通道电压实时波形（滚动曲线 + 报警阈值线）
  - 报警状态指示灯（CH1–CH4）
  - 系统信息面板（固件版本、运行时间、处理延迟）
  - HardFault 诊断面板（远程读取 PC 地址、CFSR）
  - 报警阈值可在 GUI 内修改并下发

依赖：
  pip install pymodbus==2.5.3 pyserial PyQt5 pyqtgraph

启动方式（需先运行 mock slave）：
  终端1: socat -d -d pty,raw,echo=0,link=/tmp/tty_stm32 pty,raw,echo=0,link=/tmp/tty_pc &
         python tools/mock_modbus_slave.py --port /tmp/tty_stm32
  终端2: python tools/modbus_monitor.py
"""

import sys
import time
import glob
import os
from collections import deque

from PyQt5 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg

try:
    from pymodbus.client.sync import ModbusSerialClient   # 2.x
except ImportError:
    from pymodbus.client import ModbusSerialClient         # 3.x

import pymodbus

# ===== 寄存器地址 =====
REG_STATUS       = 0x0000
REG_FW_VER       = 0x0001
REG_UPTIME_S     = 0x0002
REG_FRAME_COUNT  = 0x0003
REG_LATENCY_US   = 0x0004
REG_CRC_ERR      = 0x0005
REG_CH1_MV       = 0x0010
REG_MCU_TEMP     = 0x0014
REG_ADC_COUNT    = 0x0015
REG_CH1_HI_THR   = 0x0020
REG_CH1_LO_THR   = 0x0024
REG_FAULT_FLAG   = 0x0040
REG_FAULT_PC_LO  = 0x0041
REG_FAULT_PC_HI  = 0x0042
REG_FAULT_CFSR_LO= 0x0043
REG_FAULT_CFSR_HI= 0x0044
REG_FAULT_HFSR   = 0x0045

SLAVE_ID  = 1
BAUDRATE  = 115200
MAX_PTS   = 300   # 波形缓冲点数（~30s @ 100ms 轮询）

CH_COLORS = ['#FF4444', '#44CC44', '#4488FF', '#FFAA00']
CH_NAMES  = ['CH1', 'CH2', 'CH3', 'CH4']


# ──────────────────────────────────────────────────────────────
# 后台轮询线程
# ──────────────────────────────────────────────────────────────
class ModbusPoller(QtCore.QThread):
    data_ready   = QtCore.pyqtSignal(dict)   # 每次读取完整数据包
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self, port: str, interval_ms: int = 200):
        super().__init__()
        self.port        = port
        self.interval_ms = interval_ms
        self._running    = False
        self.client      = None

    def run(self):
        self._running = True
        if int(pymodbus.__version__.split('.')[0]) < 3:
            self.client = ModbusSerialClient(
                method='rtu', port=self.port, baudrate=BAUDRATE,
                bytesize=8, parity='N', stopbits=1, timeout=1.0)
        else:
            self.client = ModbusSerialClient(
                port=self.port, baudrate=BAUDRATE,
                bytesize=8, parity='N', stopbits=1, timeout=1.0)

        if not self.client.connect():
            self.error_signal.emit(f'无法连接串口 {self.port}')
            return

        while self._running:
            t0 = time.perf_counter()
            try:
                pkt = {}

                # --- 系统信息 (0x0000–0x0005) ---
                rr = self.client.read_holding_registers(
                    address=REG_STATUS, count=6, unit=SLAVE_ID)
                if not rr.isError():
                    pkt['status']    = rr.registers[0]
                    pkt['fw_ver']    = rr.registers[1]
                    pkt['uptime']    = rr.registers[2]
                    pkt['frame_cnt'] = rr.registers[3]
                    pkt['latency']   = rr.registers[4]
                    pkt['crc_err']   = rr.registers[5]

                # --- 4 通道电压 + 温度 + ADC (0x0010–0x0015) ---
                rr = self.client.read_holding_registers(
                    address=REG_CH1_MV, count=6, unit=SLAVE_ID)
                if not rr.isError():
                    pkt['channels'] = [rr.registers[i] / 1000.0 for i in range(4)]
                    pkt['temp']     = rr.registers[4] / 10.0
                    pkt['adc_cnt']  = rr.registers[5]

                # --- 高报警阈值 (0x0020–0x0023) ---
                rr = self.client.read_holding_registers(
                    address=REG_CH1_HI_THR, count=4, unit=SLAVE_ID)
                if not rr.isError():
                    pkt['hi_thr'] = [v / 1000.0 for v in rr.registers]

                # --- 低报警阈值 (0x0024–0x0027) ---
                rr = self.client.read_holding_registers(
                    address=REG_CH1_LO_THR, count=4, unit=SLAVE_ID)
                if not rr.isError():
                    pkt['lo_thr'] = [v / 1000.0 for v in rr.registers]

                # --- HardFault 诊断 (0x0040–0x0045) ---
                rr = self.client.read_holding_registers(
                    address=REG_FAULT_FLAG, count=6, unit=SLAVE_ID)
                if not rr.isError():
                    pkt['fault_flag'] = rr.registers[0]
                    pkt['fault_pc']   = (rr.registers[2] << 16) | rr.registers[1]
                    pkt['fault_cfsr'] = (rr.registers[4] << 16) | rr.registers[3]
                    pkt['fault_hfsr'] = rr.registers[5]

                pkt['ts'] = time.time()
                if pkt:
                    self.data_ready.emit(pkt)

            except Exception as e:
                self.error_signal.emit(str(e))

            # 精确控制轮询周期
            elapsed = (time.perf_counter() - t0) * 1000
            wait    = max(0, self.interval_ms - elapsed)
            self.msleep(int(wait))

        self.client.close()

    def stop(self):
        self._running = False
        self.wait(3000)

    def write_register(self, addr: int, value: int):
        """主线程调用：单寄存器写（线程安全，client 连接中才调用）"""
        if self.client and self._running:
            self.client.write_register(address=addr, value=value, unit=SLAVE_ID)


# ──────────────────────────────────────────────────────────────
# 主窗口
# ──────────────────────────────────────────────────────────────
class ModbusMonitor(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Modbus RTU 监控 — 4 通道电压采集模块')
        self.resize(1100, 780)

        self.poller  = None
        self.buffers = [deque(maxlen=MAX_PTS) for _ in range(4)]
        self.t_buf   = deque(maxlen=MAX_PTS)
        self.t_start = None

        self._build_ui()

        # 刷新定时器（20fps 绘制）
        self.draw_timer = QtCore.QTimer()
        self.draw_timer.timeout.connect(self._redraw)
        self.draw_timer.start(50)

    # ── UI 构建 ────────────────────────────────────────────────
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setSpacing(6)

        # 顶部工具栏
        root.addLayout(self._build_toolbar())

        # 主体：左侧波形 + 右侧信息面板
        body = QtWidgets.QHBoxLayout()
        body.addLayout(self._build_plots(), stretch=3)
        body.addLayout(self._build_info_panel(), stretch=1)
        root.addLayout(body)

        # 底部状态栏
        self.status_bar = QtWidgets.QLabel('未连接')
        self.status_bar.setStyleSheet('color: gray; padding: 2px 6px;')
        root.addWidget(self.status_bar)

    def _build_toolbar(self):
        bar = QtWidgets.QHBoxLayout()

        def mk_label(text):
            lbl = QtWidgets.QLabel(text)
            lbl.setStyleSheet('color: #cccccc;')
            return lbl

        bar.addWidget(mk_label('串口:'))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(220)
        self._refresh_ports()
        bar.addWidget(self.port_combo)

        refresh_btn = QtWidgets.QPushButton('刷新')
        refresh_btn.clicked.connect(self._refresh_ports)
        bar.addWidget(refresh_btn)

        self.connect_btn = QtWidgets.QPushButton('连接')
        self.connect_btn.clicked.connect(self._on_connect)
        self.connect_btn.setMinimumWidth(70)
        bar.addWidget(self.connect_btn)

        bar.addSpacing(20)
        bar.addWidget(mk_label('轮询间隔:'))
        self.interval_combo = QtWidgets.QComboBox()
        self.interval_combo.addItems(['100 ms', '200 ms', '500 ms', '1000 ms'])
        self.interval_combo.setCurrentIndex(1)
        bar.addWidget(self.interval_combo)

        bar.addStretch()

        # 报警指示灯
        bar.addWidget(mk_label('报警:'))
        self.alarm_leds = []
        for i in range(4):
            led = QtWidgets.QLabel(f' CH{i+1} ')
            led.setAlignment(QtCore.Qt.AlignCenter)
            led.setFixedWidth(42)
            led.setStyleSheet('background:#555; color:#aaa; border-radius:4px; font-size:11px;')
            self.alarm_leds.append(led)
            bar.addWidget(led)

        return bar

    def _build_plots(self):
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(4)

        pg.setConfigOptions(antialias=True, background='#1a1a2e', foreground='#cccccc')

        self.plots  = []
        self.curves = []
        self.hi_lines = []
        self.lo_lines = []

        for i in range(4):
            pw = pg.PlotWidget(title=f'<span style="color:{CH_COLORS[i]}">{CH_NAMES[i]}</span> 电压 (V)')
            pw.setLabel('left', 'V', color='#aaaaaa')
            pw.setYRange(0, 11)
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setMinimumHeight(140)
            pw.getAxis('left').setWidth(45)      # 防止 Y 轴标签被截断
            pw.getAxis('left').setTextPen('#cccccc')
            pw.getAxis('bottom').setTextPen('#cccccc')

            curve = pw.plot(pen=pg.mkPen(CH_COLORS[i], width=2))
            hi    = pg.InfiniteLine(pos=9.0, angle=0,
                        pen=pg.mkPen('#FF6666', width=1, style=QtCore.Qt.DashLine),
                        label='HI', labelOpts={'color': '#FF6666', 'position': 0.95})
            lo    = pg.InfiniteLine(pos=0.5, angle=0,
                        pen=pg.mkPen('#6699FF', width=1, style=QtCore.Qt.DashLine),
                        label='LO', labelOpts={'color': '#6699FF', 'position': 0.05})
            pw.addItem(hi)
            pw.addItem(lo)

            self.plots.append(pw)
            self.curves.append(curve)
            self.hi_lines.append(hi)
            self.lo_lines.append(lo)
            layout.addWidget(pw)

        return layout

    def _build_info_panel(self):
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(8)

        GRP_SS = """
            QGroupBox {
                color: #e0e0e0;
                font-weight: bold;
                border: 1px solid #44445a;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 4px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                color: #aaddff;
            }
            QLabel { color: #d0d0d0; }
            QDoubleSpinBox {
                color: #e0e0e0;
                background: #2a2a42;
                border: 1px solid #555570;
                border-radius: 3px;
                padding: 1px 4px;
            }
            QPushButton {
                color: #e0e0e0;
                background: #3a3a5a;
                border: 1px solid #555570;
                border-radius: 3px;
                padding: 3px 10px;
            }
            QPushButton:hover { background: #4a4a6a; }
        """

        # ── 系统信息 ──────────────────────────────────
        grp1 = QtWidgets.QGroupBox('系统信息')
        grp1.setStyleSheet(GRP_SS)
        g1 = QtWidgets.QFormLayout(grp1)
        g1.setSpacing(4)
        g1.setLabelAlignment(QtCore.Qt.AlignRight)

        self.lbl_fw    = QtWidgets.QLabel('---')
        self.lbl_up    = QtWidgets.QLabel('---')
        self.lbl_lat   = QtWidgets.QLabel('---')
        self.lbl_temp  = QtWidgets.QLabel('---')
        self.lbl_adc   = QtWidgets.QLabel('---')
        self.lbl_crc   = QtWidgets.QLabel('---')
        for text, widget in [('固件版本:', self.lbl_fw), ('运行时间:', self.lbl_up),
                              ('处理延迟:', self.lbl_lat), ('MCU 温度:', self.lbl_temp),
                              ('ADC 计数:', self.lbl_adc), ('CRC 错误:', self.lbl_crc)]:
            lbl = QtWidgets.QLabel(text)
            lbl.setStyleSheet('color: #8899bb;')
            g1.addRow(lbl, widget)
        layout.addWidget(grp1)

        # ── 当前电压 ──────────────────────────────────
        grp2 = QtWidgets.QGroupBox('当前电压')
        grp2.setStyleSheet(GRP_SS)
        g2 = QtWidgets.QFormLayout(grp2)
        g2.setSpacing(4)
        g2.setLabelAlignment(QtCore.Qt.AlignRight)
        self.lbl_ch = []
        for i in range(4):
            key = QtWidgets.QLabel(f'{CH_NAMES[i]}:')
            key.setStyleSheet('color: #8899bb;')
            lbl = QtWidgets.QLabel('--- V')
            lbl.setStyleSheet(f'color: {CH_COLORS[i]}; font-weight: bold; font-size: 13px;')
            g2.addRow(key, lbl)
            self.lbl_ch.append(lbl)
        layout.addWidget(grp2)

        # ── 报警阈值设置 ──────────────────────────────
        grp3 = QtWidgets.QGroupBox('CH1 报警阈值 (V)')
        grp3.setStyleSheet(GRP_SS)
        g3 = QtWidgets.QFormLayout(grp3)
        g3.setSpacing(4)
        g3.setLabelAlignment(QtCore.Qt.AlignRight)

        self.spin_hi = QtWidgets.QDoubleSpinBox()
        self.spin_hi.setRange(0, 10); self.spin_hi.setSingleStep(0.5); self.spin_hi.setValue(9.0)
        self.spin_lo = QtWidgets.QDoubleSpinBox()
        self.spin_lo.setRange(0, 10); self.spin_lo.setSingleStep(0.1); self.spin_lo.setValue(0.5)

        hi_lbl = QtWidgets.QLabel('高报警 (HI):')
        hi_lbl.setStyleSheet('color: #FF8888;')
        lo_lbl = QtWidgets.QLabel('低报警 (LO):')
        lo_lbl.setStyleSheet('color: #8899FF;')
        g3.addRow(hi_lbl, self.spin_hi)
        g3.addRow(lo_lbl, self.spin_lo)

        write_btn = QtWidgets.QPushButton('写入阈值')
        write_btn.clicked.connect(self._write_thresholds)
        g3.addRow(write_btn)
        layout.addWidget(grp3)

        # ── HardFault 诊断 ────────────────────────────
        grp4 = QtWidgets.QGroupBox('HardFault 诊断')
        grp4.setStyleSheet(GRP_SS)
        g4 = QtWidgets.QFormLayout(grp4)
        g4.setSpacing(4)
        g4.setLabelAlignment(QtCore.Qt.AlignRight)

        self.lbl_fault_flag = QtWidgets.QLabel('---')
        self.lbl_fault_pc   = QtWidgets.QLabel('---')
        self.lbl_fault_cfsr = QtWidgets.QLabel('---')
        self.lbl_fault_hfsr = QtWidgets.QLabel('---')
        for text, widget in [('故障标志:', self.lbl_fault_flag), ('故障 PC:', self.lbl_fault_pc),
                              ('CFSR:', self.lbl_fault_cfsr),    ('HFSR:', self.lbl_fault_hfsr)]:
            lbl = QtWidgets.QLabel(text)
            lbl.setStyleSheet('color: #8899bb;')
            g4.addRow(lbl, widget)
        layout.addWidget(grp4)

        layout.addStretch()
        return layout

    # ── 串口刷新 ───────────────────────────────────────────────
    def _refresh_ports(self):
        from serial.tools import list_ports
        cur = self.port_combo.currentText()
        self.port_combo.clear()
        for p in list_ports.comports():
            self.port_combo.addItem(p.device)
        for path in sorted(glob.glob('/tmp/tty_*')):
            if os.path.exists(path):
                self.port_combo.addItem(path)
        idx = self.port_combo.findText(cur)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)
        elif cur:
            self.port_combo.setCurrentText(cur)

    # ── 连接 / 断开 ────────────────────────────────────────────
    def _on_connect(self):
        if self.poller and self.poller.isRunning():
            self.poller.stop()
            self.poller = None
            self.connect_btn.setText('连接')
            self.status_bar.setText('已断开')
            self.status_bar.setStyleSheet('color: gray; padding: 2px 6px;')
            return

        port = self.port_combo.currentText().strip()
        if not port:
            QtWidgets.QMessageBox.warning(self, '提示', '请选择串口')
            return

        interval_text = self.interval_combo.currentText().split()[0]
        interval_ms   = int(interval_text)

        self.t_start = time.time()
        for buf in self.buffers:
            buf.clear()
        self.t_buf.clear()

        self.poller = ModbusPoller(port, interval_ms)
        self.poller.data_ready.connect(self._on_data)
        self.poller.error_signal.connect(self._on_error)
        self.poller.start()

        self.connect_btn.setText('断开')
        self.status_bar.setText(f'已连接 {port}  |  轮询间隔 {interval_ms} ms')
        self.status_bar.setStyleSheet('color: #44cc44; padding: 2px 6px;')

    # ── 数据处理 ───────────────────────────────────────────────
    def _on_data(self, pkt: dict):
        t = pkt.get('ts', time.time()) - self.t_start
        self.t_buf.append(t)

        channels = pkt.get('channels')
        if channels:
            for i, v in enumerate(channels):
                self.buffers[i].append(v)
            for i, lbl in enumerate(self.lbl_ch):
                lbl.setText(f'{channels[i]:.3f} V')

        hi_thr = pkt.get('hi_thr')
        lo_thr = pkt.get('lo_thr')
        if hi_thr:
            for i, line in enumerate(self.hi_lines):
                line.setValue(hi_thr[i])
        if lo_thr:
            for i, line in enumerate(self.lo_lines):
                line.setValue(lo_thr[i])
            # 同步 spinbox（只更新 CH1）
            self.spin_hi.blockSignals(True)
            self.spin_lo.blockSignals(True)
            if hi_thr: self.spin_hi.setValue(hi_thr[0])
            if lo_thr: self.spin_lo.setValue(lo_thr[0])
            self.spin_hi.blockSignals(False)
            self.spin_lo.blockSignals(False)

        # 报警指示灯
        status = pkt.get('status', 0)
        for i, led in enumerate(self.alarm_leds):
            if status & (1 << i):
                led.setStyleSheet(
                    f'background:{CH_COLORS[i]}; color:white; border-radius:4px;'
                    f'font-size:11px; font-weight:bold;')
            else:
                led.setStyleSheet(
                    'background:#555; color:#aaa; border-radius:4px; font-size:11px;')

        # 系统信息
        if 'fw_ver' in pkt:
            v = pkt['fw_ver']
            self.lbl_fw.setText(f'v{v>>8}.{v&0xFF}')
        if 'uptime'  in pkt: self.lbl_up.setText(f"{pkt['uptime']} s")
        if 'latency' in pkt: self.lbl_lat.setText(f"{pkt['latency']} μs")
        if 'temp'    in pkt: self.lbl_temp.setText(f"{pkt['temp']:.1f} °C")
        if 'adc_cnt' in pkt: self.lbl_adc.setText(str(pkt['adc_cnt']))
        if 'crc_err' in pkt: self.lbl_crc.setText(str(pkt['crc_err']))

        # HardFault
        if 'fault_flag' in pkt:
            ff = pkt['fault_flag']
            color = '#FF4444' if ff != 0 else '#44CC44'
            self.lbl_fault_flag.setText(f'0x{ff:04X}')
            self.lbl_fault_flag.setStyleSheet(f'color: {color};')
        if 'fault_pc'   in pkt:
            self.lbl_fault_pc.setText(f"0x{pkt['fault_pc']:08X}")
        if 'fault_cfsr' in pkt:
            cfsr = pkt['fault_cfsr']
            desc = ''
            if cfsr & 0x0400: desc = ' STKERR'
            elif cfsr & 0x0200: desc = ' UNSTKERR'
            elif cfsr & 0x0100: desc = ' IMPRECISERR'
            self.lbl_fault_cfsr.setText(f"0x{cfsr:08X}{desc}")
        if 'fault_hfsr' in pkt:
            self.lbl_fault_hfsr.setText(f"0x{pkt['fault_hfsr']:08X}")

    def _on_error(self, msg: str):
        self.status_bar.setText(f'错误: {msg}')
        self.status_bar.setStyleSheet('color: #ff6644; padding: 2px 6px;')

    # ── 写阈值 ─────────────────────────────────────────────────
    def _write_thresholds(self):
        if not (self.poller and self.poller.isRunning()):
            QtWidgets.QMessageBox.warning(self, '提示', '请先连接')
            return
        hi_mv = int(self.spin_hi.value() * 1000)
        lo_mv = int(self.spin_lo.value() * 1000)
        self.poller.write_register(REG_CH1_HI_THR, hi_mv)
        self.poller.write_register(REG_CH1_LO_THR, lo_mv)
        self.status_bar.setText(
            f'已写入 CH1 阈值: HI={hi_mv} mV  LO={lo_mv} mV')

    # ── 波形重绘 ───────────────────────────────────────────────
    def _redraw(self):
        if not self.t_buf:
            return
        t_list = list(self.t_buf)
        for i, curve in enumerate(self.curves):
            vals = list(self.buffers[i])
            n    = min(len(t_list), len(vals))
            curve.setData(t_list[-n:], vals[-n:])
        for pw in self.plots:
            pw.setXRange(t_list[-1] - 30, t_list[-1] + 1, padding=0)

    def closeEvent(self, event):
        if self.poller:
            self.poller.stop()
        event.accept()


# ──────────────────────────────────────────────────────────────
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')

    app.setStyleSheet("""
        QWidget {
            background-color: #1e1e2e;
            color: #cccccc;
            font-size: 12px;
        }
        QMainWindow, QDialog { background-color: #1e1e2e; }
        QLabel  { color: #cccccc; background: transparent; }
        QComboBox {
            background: #2a2a42; color: #e0e0e0;
            border: 1px solid #555570; border-radius: 3px;
            padding: 2px 6px;
        }
        QComboBox QAbstractItemView {
            background: #2a2a42; color: #e0e0e0;
            selection-background-color: #4466aa;
        }
        QPushButton {
            background: #3a3a5a; color: #e0e0e0;
            border: 1px solid #555570; border-radius: 4px;
            padding: 4px 12px;
        }
        QPushButton:hover   { background: #4a4a6a; }
        QPushButton:pressed { background: #2a2a4a; }
        QDoubleSpinBox, QSpinBox {
            background: #2a2a42; color: #e0e0e0;
            border: 1px solid #555570; border-radius: 3px;
            padding: 2px 4px;
        }
        QDoubleSpinBox::up-button, QDoubleSpinBox::down-button,
        QSpinBox::up-button,       QSpinBox::down-button {
            background: #3a3a5a; width: 16px;
        }
        QScrollBar:vertical {
            background: #2a2a42; width: 8px;
        }
        QScrollBar::handle:vertical { background: #555577; border-radius: 4px; }
    """)

    win = ModbusMonitor()
    win.show()
    sys.exit(app.exec_())
