import sys
import time
from collections import deque

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

try:
    import serial
except ImportError:
    serial = None


# ===================== USER SETTINGS =====================
PORT = "COM6"          # change to your port
BAUD = 115200
NUM_SENSORS = 14
TIMER_MS = 10          # UI refresh timer
READ_LATEST_ONLY = True
PID_HISTORY = 300      # number of points shown in bottom graph
# ========================================================


class SerialReader:
    def __init__(self, port, baud):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0.001)
        self.buffer = bytearray()

    def read_latest_line(self):
        """
        Returns the newest complete line if available.
        If READ_LATEST_ONLY is True, older queued lines are discarded.
        """
        if not self.ser.is_open:
            return None

        waiting = self.ser.in_waiting
        if waiting:
            self.buffer.extend(self.ser.read(waiting))

        if b"\n" not in self.buffer:
            return None

        parts = self.buffer.split(b"\n")

        if READ_LATEST_ONLY:
            latest = parts[-2] if len(parts) >= 2 else None
            self.buffer = bytearray(parts[-1])
            if latest is None:
                return None
            return latest.decode(errors="ignore").strip()
        else:
            first = parts[0]
            self.buffer = bytearray(b"\n".join(parts[1:]))
            return first.decode(errors="ignore").strip()

    def close(self):
        if hasattr(self, "ser") and self.ser and self.ser.is_open:
            self.ser.close()


class Visualizer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Line Follower Sensor + PID Visualizer")
        self.resize(1200, 800)

        self.reader = None
        self.connected = False

        self.sensor_vals = [0] * NUM_SENSORS
        self.pos = 0.0
        self.err = 0.0
        self.pid = 0.0
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.loop_us = 0

        self.pid_hist = deque([0.0] * PID_HISTORY, maxlen=PID_HISTORY)
        self.err_hist = deque([0.0] * PID_HISTORY, maxlen=PID_HISTORY)

        self._build_ui()
        self._setup_timer()

    def _build_ui(self):
        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)
        layout = QtWidgets.QVBoxLayout(cw)

        # ---------- Top controls ----------
        controls = QtWidgets.QHBoxLayout()

        self.port_edit = QtWidgets.QLineEdit(PORT)
        self.port_edit.setFixedWidth(120)

        self.baud_edit = QtWidgets.QLineEdit(str(BAUD))
        self.baud_edit.setFixedWidth(100)

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)

        self.status_lbl = QtWidgets.QLabel("Disconnected")

        self.info_lbl = QtWidgets.QLabel(
            "pos: 0.00 | err: 0.00 | pid: 0.00 | L: 0 | R: 0 | loop: 0 us"
        )

        controls.addWidget(QtWidgets.QLabel("Port:"))
        controls.addWidget(self.port_edit)
        controls.addSpacing(10)
        controls.addWidget(QtWidgets.QLabel("Baud:"))
        controls.addWidget(self.baud_edit)
        controls.addSpacing(10)
        controls.addWidget(self.connect_btn)
        controls.addSpacing(20)
        controls.addWidget(self.status_lbl)
        controls.addStretch()
        controls.addWidget(self.info_lbl)

        layout.addLayout(controls)

        # ---------- Sensor graph ----------
        self.sensor_plot = pg.PlotWidget(title="Sensor Array")
        self.sensor_plot.setBackground("w")
        self.sensor_plot.showGrid(x=True, y=True, alpha=0.25)
        self.sensor_plot.setYRange(0, 1000)
        self.sensor_plot.setLabel("left", "Normalized Value")
        self.sensor_plot.setLabel("bottom", "Sensor Index")
        self.sensor_plot.addLegend()

        x = list(range(NUM_SENSORS))
        self.sensor_curve = self.sensor_plot.plot(
            x, self.sensor_vals, pen=pg.mkPen(width=3), symbol="o", name="Sensors"
        )

        self.pos_line = pg.InfiniteLine(
            pos=(NUM_SENSORS - 1) / 2, angle=90, movable=False
        )
        self.sensor_plot.addItem(self.pos_line)

        layout.addWidget(self.sensor_plot, stretch=3)

        # ---------- PID graph ----------
        self.pid_plot = pg.PlotWidget(title="PID / Error")
        self.pid_plot.setBackground("w")
        self.pid_plot.showGrid(x=True, y=True, alpha=0.25)
        self.pid_plot.setLabel("left", "Value")
        self.pid_plot.setLabel("bottom", "Samples")
        self.pid_plot.addLegend()

        xh = list(range(PID_HISTORY))
        self.err_curve = self.pid_plot.plot(
            xh, list(self.err_hist), pen=pg.mkPen(width=2), name="Error"
        )
        self.pid_curve = self.pid_plot.plot(
            xh, list(self.pid_hist), pen=pg.mkPen(style=QtCore.Qt.DashLine, width=2), name="PID"
        )

        layout.addWidget(self.pid_plot, stretch=2)

    def _setup_timer(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_from_serial)
        self.timer.start(TIMER_MS)

    def toggle_connection(self):
        if self.connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.port_edit.text().strip()
        baud = int(self.baud_edit.text().strip())

        try:
            self.reader = SerialReader(port, baud)
            self.connected = True
            self.status_lbl.setText(f"Connected: {port}")
            self.connect_btn.setText("Disconnect")
        except Exception as e:
            self.status_lbl.setText(f"Connection failed: {e}")
            self.connected = False
            self.reader = None

    def disconnect_serial(self):
        if self.reader:
            self.reader.close()
        self.reader = None
        self.connected = False
        self.status_lbl.setText("Disconnected")
        self.connect_btn.setText("Connect")

    def parse_line(self, line):
        """
        Expected format:
        S,v0,v1,...,v13,pos,err,pid,left,right,loop_us

        Example:
        S,12,55,130,...,900,6.42,-0.58,-23.4,180,220,3100
        """
        parts = line.split(",")
        if len(parts) != 1 + NUM_SENSORS + 6:
            return None
        if parts[0] != "S":
            return None

        try:
            vals = list(map(int, parts[1:1 + NUM_SENSORS]))
            pos = float(parts[1 + NUM_SENSORS])
            err = float(parts[2 + NUM_SENSORS])
            pid = float(parts[3 + NUM_SENSORS])
            left_cmd = float(parts[4 + NUM_SENSORS])
            right_cmd = float(parts[5 + NUM_SENSORS])
            loop_us = int(parts[6 + NUM_SENSORS])

            return vals, pos, err, pid, left_cmd, right_cmd, loop_us
        except ValueError:
            return None

    def update_from_serial(self):
        if not self.connected or self.reader is None:
            return

        line = self.reader.read_latest_line()
        if not line:
            return

        parsed = self.parse_line(line)
        if parsed is None:
            return

        (
            self.sensor_vals,
            self.pos,
            self.err,
            self.pid,
            self.left_cmd,
            self.right_cmd,
            self.loop_us,
        ) = parsed

        # Update top graph
        self.sensor_curve.setData(list(range(NUM_SENSORS)), self.sensor_vals)
        self.pos_line.setPos(self.pos)

        # Update bottom graph
        self.err_hist.append(self.err)
        self.pid_hist.append(self.pid)
        self.err_curve.setData(list(range(PID_HISTORY)), list(self.err_hist))
        self.pid_curve.setData(list(range(PID_HISTORY)), list(self.pid_hist))

        self.info_lbl.setText(
            f"pos: {self.pos:.2f} | err: {self.err:.2f} | pid: {self.pid:.2f} | "
            f"L: {self.left_cmd:.0f} | R: {self.right_cmd:.0f} | loop: {self.loop_us} us"
        )

    def closeEvent(self, event):
        self.disconnect_serial()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)

    win = Visualizer()
    win.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()