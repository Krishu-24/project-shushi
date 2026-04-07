import sys, time, json, csv
from dataclasses import dataclass, asdict
import numpy as np

from PyQt5 import QtWidgets, QtCore, QtGui

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

try:
    import serial
except ImportError:
    serial = None

# ===================== USER SETTINGS =====================
PORT = "COM6"      # <-- change (or leave wrong to test UI)
BAUD = 115200
NUM_SENSORS = 14
TIMER_MS = 5
READ_LATEST_ONLY = True
# =========================================================


# --------------------- Stats helpers ---------------------
@dataclass
class Stats:
    count: int
    mean: float
    std: float
    var: float
    min: int
    max: int
    p10: float
    p50: float
    p90: float

def compute_stats(x: np.ndarray) -> Stats:
    x = np.asarray(x)
    if x.size == 0:
        return Stats(0, 0, 0, 0, 0, 0, 0, 0, 0)
    mean = float(np.mean(x))
    if x.size > 1:
        std = float(np.std(x, ddof=1))
        var = float(np.var(x, ddof=1))
    else:
        std, var = 0.0, 0.0
    mn = int(np.min(x))
    mx = int(np.max(x))
    p10, p50, p90 = np.percentile(x, [10, 50, 90]).tolist()
    return Stats(int(x.size), mean, std, var, mn, mx, float(p10), float(p50), float(p90))


# --------------------- UI components ---------------------
class SegmentedButton(QtWidgets.QWidget):
    changed = QtCore.pyqtSignal(str)

    def __init__(self, title: str, options: list, default: str):
        super().__init__()
        self._options = options
        self._buttons = {}

        title_lbl = QtWidgets.QLabel(title)
        title_lbl.setObjectName("segTitle")

        row = QtWidgets.QHBoxLayout()
        row.setSpacing(0)
        row.setContentsMargins(0, 0, 0, 0)

        group = QtWidgets.QButtonGroup(self)
        group.setExclusive(True)

        for i, opt in enumerate(options):
            b = QtWidgets.QPushButton(opt)
            b.setCheckable(True)
            b.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
            b.clicked.connect(lambda _, o=opt: self.changed.emit(o))
            self._buttons[opt] = b
            group.addButton(b)

            if i == 0:
                b.setProperty("segPos", "left")
            elif i == len(options) - 1:
                b.setProperty("segPos", "right")
            else:
                b.setProperty("segPos", "mid")

            row.addWidget(b)

        wrap = QtWidgets.QVBoxLayout()
        wrap.setContentsMargins(0, 0, 0, 0)
        wrap.setSpacing(6)
        wrap.addWidget(title_lbl)
        wrap.addLayout(row)
        self.setLayout(wrap)

        self.set_value(default)

    def set_value(self, val: str):
        if val in self._buttons:
            self._buttons[val].setChecked(True)

    def value(self) -> str:
        for k, b in self._buttons.items():
            if b.isChecked():
                return k
        return self._options[0]


class StatCard(QtWidgets.QFrame):
    def __init__(self, title: str, value: str = "-"):
        super().__init__()
        self.setObjectName("statCard")
        self.title = QtWidgets.QLabel(title)
        self.title.setObjectName("statTitle")
        self.value = QtWidgets.QLabel(value)
        self.value.setObjectName("statValue")

        lay = QtWidgets.QVBoxLayout()
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(2)
        lay.addWidget(self.title)
        lay.addWidget(self.value)
        self.setLayout(lay)

    def set_value(self, v: str):
        self.value.setText(v)


class SensorCharacterizer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sensor Characterizer (Line Follower)")
        self.setMinimumSize(1020, 580)

        # Serial connection state
        self.ser = None
        self.connected = False
        self.last_serial_error = ""

        # State
        self.current_mode = "Stationary"
        self.current_surface = "White"
        self.recording = False
        self.run_started = False
        self.t0 = None
        self.samples = []  # {t, mode, surface, vals[14]}

        # Live monitoring
        self.last_vals = None
        self.live_window = []
        self.live_window_max = 200

        # Rate tracking
        self._rate_count = 0
        self._rate_t0 = time.time()
        self._incoming_rate = 0.0

        # ===== UI =====
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        self.run_label = QtWidgets.QLineEdit()
        self.run_label.setPlaceholderText("Run label (e.g., surface_mattepaper_blacktape_height3mm)")
        self.run_label.setObjectName("runLabel")

        self.mode_seg = SegmentedButton("Mode", ["Stationary", "Moving"], "Stationary")
        self.surface_seg = SegmentedButton("Surface", ["White", "Black"], "White")

        self.record_btn = QtWidgets.QPushButton("● Record")
        self.record_btn.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.record_btn.setObjectName("recordBtn")
        self.record_btn.clicked.connect(self.toggle_record)

        self.reset_btn = QtWidgets.QPushButton("Reset (Clear Run)")
        self.reset_btn.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.reset_btn.setObjectName("secondaryBtn")
        self.reset_btn.clicked.connect(self.reset_run)

        self.export_btn = QtWidgets.QPushButton("Export… (PDF + JSON + CSV)")
        self.export_btn.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.export_btn.setObjectName("exportBtn")
        self.export_btn.clicked.connect(self.export_dialog)

        # Guide
        guide = QtWidgets.QFrame()
        guide.setObjectName("guideBox")
        guide_l = QtWidgets.QVBoxLayout()
        guide_l.setContentsMargins(12, 10, 12, 10)
        guide_l.setSpacing(6)
        gtitle = QtWidgets.QLabel("Workflow")
        gtitle.setObjectName("guideTitle")
        gtext = QtWidgets.QLabel(
            "Capture SW, SB, MW, MB:\n"
            "• Set Mode + Surface\n"
            "• Record 10–30s\n"
            "• Pause → reposition\n"
            "• Export when done"
        )
        gtext.setObjectName("guideText")
        gtext.setWordWrap(True)
        guide_l.addWidget(gtitle)
        guide_l.addWidget(gtext)
        guide.setLayout(guide_l)

        left = QtWidgets.QVBoxLayout()
        left.setContentsMargins(0, 0, 0, 0)
        left.setSpacing(14)
        left.addWidget(self.run_label)
        left.addWidget(self.mode_seg)
        left.addWidget(self.surface_seg)
        left.addSpacing(4)
        left.addWidget(self.record_btn)
        left.addWidget(self.reset_btn)
        left.addWidget(self.export_btn)
        left.addWidget(guide)
        left.addStretch(1)

        left_wrap = QtWidgets.QFrame()
        left_wrap.setObjectName("panel")
        left_wrap_l = QtWidgets.QVBoxLayout()
        left_wrap_l.setContentsMargins(16, 16, 16, 16)
        left_wrap_l.addLayout(left)
        left_wrap.setLayout(left_wrap_l)

        # Right panel top bar: recording pill (left) + device status (right)
        self.state_pill = QtWidgets.QLabel("Paused")
        self.state_pill.setObjectName("statePill")

        self.state_detail = QtWidgets.QLabel("Mode: Stationary  |  Surface: White")
        self.state_detail.setObjectName("stateDetail")

        self.device_pill = QtWidgets.QLabel("Device: Not connected")
        self.device_pill.setObjectName("devicePill")

        self.reconnect_btn = QtWidgets.QPushButton("Reconnect")
        self.reconnect_btn.setObjectName("reconnectBtn")
        self.reconnect_btn.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.reconnect_btn.clicked.connect(self.try_connect_serial)

        topbar = QtWidgets.QHBoxLayout()
        topbar.addWidget(self.state_pill)
        topbar.addSpacing(10)
        topbar.addWidget(self.state_detail)
        topbar.addStretch(1)
        topbar.addWidget(self.reconnect_btn)
        topbar.addSpacing(8)
        topbar.addWidget(self.device_pill)

        # Stat cards
        self.card_samples = StatCard("Samples stored", "0")
        self.card_rate = StatCard("Incoming rate", "0.0 Hz")
        self.card_duration = StatCard("Run duration", "0.0 s")
        self.card_last = StatCard("Last frame (S0,S1,S2…)", "-")

        cards = QtWidgets.QGridLayout()
        cards.setHorizontalSpacing(12)
        cards.setVerticalSpacing(12)
        cards.addWidget(self.card_samples, 0, 0)
        cards.addWidget(self.card_rate, 0, 1)
        cards.addWidget(self.card_duration, 1, 0)
        cards.addWidget(self.card_last, 1, 1)

        # Live table
        self.live_table = QtWidgets.QTableWidget(NUM_SENSORS, 3)
        self.live_table.setHorizontalHeaderLabels(["Sensor", "Recent mean", "Recent std"])
        self.live_table.verticalHeader().setVisible(False)
        self.live_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.live_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.live_table.setObjectName("liveTable")
        self.live_table.horizontalHeader().setStretchLastSection(True)
        self.live_table.setColumnWidth(0, 70)
        self.live_table.setColumnWidth(1, 130)

        for i in range(NUM_SENSORS):
            self.live_table.setItem(i, 0, QtWidgets.QTableWidgetItem(f"S{i}"))
            self.live_table.setItem(i, 1, QtWidgets.QTableWidgetItem("-"))
            self.live_table.setItem(i, 2, QtWidgets.QTableWidgetItem("-"))

        live_box = QtWidgets.QFrame()
        live_box.setObjectName("liveBox")
        live_l = QtWidgets.QVBoxLayout()
        live_l.setContentsMargins(12, 10, 12, 10)
        live_l.setSpacing(8)
        live_title = QtWidgets.QLabel("Live quick stats (recent window)")
        live_title.setObjectName("liveTitle")
        live_l.addWidget(live_title)
        live_l.addWidget(self.live_table)
        live_box.setLayout(live_l)

        right = QtWidgets.QVBoxLayout()
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(14)
        right.addLayout(topbar)
        right.addLayout(cards)
        right.addWidget(live_box)
        right.addStretch(1)

        right_wrap = QtWidgets.QFrame()
        right_wrap.setObjectName("panel")
        right_wrap_l = QtWidgets.QVBoxLayout()
        right_wrap_l.setContentsMargins(16, 16, 16, 16)
        right_wrap_l.addLayout(right)
        right_wrap.setLayout(right_wrap_l)

        main = QtWidgets.QHBoxLayout()
        main.setContentsMargins(16, 16, 16, 16)
        main.setSpacing(14)
        main.addWidget(left_wrap, 0)
        main.addWidget(right_wrap, 1)
        central.setLayout(main)

        # Connect segmented controls
        self.mode_seg.changed.connect(self.on_mode_changed)
        self.surface_seg.changed.connect(self.on_surface_changed)

        # Status bar
        self.statusBar().showMessage("Ready (serial optional).")
        self.apply_style()
        self.refresh_state_ui()

        # Try connecting, but don't crash if it fails
        self.try_connect_serial(silent=True)

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(TIMER_MS)

    # ---------- Style ----------
    def apply_style(self):
        self.setStyleSheet("""
        QMainWindow { background: #0f1115; }
        #panel { background: #151824; border: 1px solid #242a3a; border-radius: 14px; }
        #runLabel {
            padding: 10px 12px; border-radius: 10px;
            border: 1px solid #2a3145; background: #0f1115; color: #e7eaf0;
            font-size: 13px;
        }
        QLabel { color: #e7eaf0; }
        #segTitle { font-size: 12px; color: #aeb6cc; }

        QPushButton {
            padding: 10px 12px; border-radius: 10px;
            border: 1px solid #2a3145; background: #0f1115; color: #e7eaf0;
            font-size: 13px;
        }
        QPushButton:hover { border-color: #3b4666; }
        QPushButton:checked { background: #1b2240; border-color: #5a6cc7; }
        QPushButton[segPos="left"] { border-top-right-radius: 0px; border-bottom-right-radius: 0px; }
        QPushButton[segPos="mid"] { border-radius: 0px; }
        QPushButton[segPos="right"] { border-top-left-radius: 0px; border-bottom-left-radius: 0px; }

        #recordBtn {
            background: #2a0f12; border-color: #6b1b25;
            font-weight: 700; padding: 12px 12px;
        }
        #recordBtn:hover { border-color: #c04455; }
        #recordBtn[recording="true"] { background: #0f2a17; border-color: #1e6b3a; }

        #exportBtn { background: #111b2a; border-color: #2a3c66; font-weight: 700; }
        #exportBtn:hover { border-color: #5a6cc7; }

        #secondaryBtn { background: #12131a; }
        #reconnectBtn { background:#12131a; }

        #statePill {
            padding: 6px 10px; border-radius: 999px;
            background: #101322; border: 1px solid #2a3145;
            font-weight: 800;
        }
        #devicePill {
            padding: 6px 10px; border-radius: 999px;
            background: #2a0f12; border: 1px solid #6b1b25;
            font-weight: 800;
        }
        #stateDetail { color: #aeb6cc; }

        #statCard { background: #0f1115; border: 1px solid #242a3a; border-radius: 12px; }
        #statTitle { color: #aeb6cc; font-size: 11px; }
        #statValue { font-size: 18px; font-weight: 800; }

        #guideBox { background: #0f1115; border: 1px dashed #2a3145; border-radius: 12px; }
        #guideTitle { font-weight: 900; }
        #guideText { color: #aeb6cc; }

        #liveBox { background: #0f1115; border: 1px solid #242a3a; border-radius: 12px; }
        #liveTitle { font-weight: 900; }

        #liveTable {
            background: transparent; border: none;
            gridline-color: #242a3a;
            color: #e7eaf0;
        }
        QHeaderView::section {
            background: #151824; color: #aeb6cc;
            border: 1px solid #242a3a; padding: 6px;
        }
        """)

    # ---------- Serial connect/disconnect ----------
    def set_device_status(self, ok: bool, msg: str = ""):
        self.connected = ok
        if ok:
            self.device_pill.setText(f"Device: Connected ({PORT})")
            self.device_pill.setStyleSheet(
                "padding:6px 10px; border-radius:999px; background:#0f2a17; border:1px solid #1e6b3a; font-weight:800;"
            )
            self.statusBar().showMessage(f"Connected: {PORT} @ {BAUD}")
        else:
            self.device_pill.setText("Device: Not connected")
            self.device_pill.setStyleSheet(
                "padding:6px 10px; border-radius:999px; background:#2a0f12; border:1px solid #6b1b25; font-weight:800;"
            )
            if msg:
                self.statusBar().showMessage(msg)

    def try_connect_serial(self, silent: bool = False):
        if serial is None:
            self.ser = None
            self.set_device_status(False, "pyserial not installed.")
            return

        # Close if already open
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0)
            self.set_device_status(True)
        except Exception as e:
            self.last_serial_error = str(e)
            self.set_device_status(False, f"Not connected: {self.last_serial_error}")
            if not silent:
                QtWidgets.QMessageBox.information(
                    self, "Not connected",
                    f"Could not open {PORT} @ {BAUD}.\n\n{e}\n\n"
                    "That's okay — the app still works.\n"
                    "Plug Teensy in and press Reconnect."
                )

    # ---------- State UI ----------
    def on_mode_changed(self, mode: str):
        self.current_mode = mode
        self.refresh_state_ui()

    def on_surface_changed(self, surface: str):
        self.current_surface = surface
        self.refresh_state_ui()

    def refresh_state_ui(self):
        self.state_detail.setText(f"Mode: {self.current_mode}  |  Surface: {self.current_surface}")
        if self.recording:
            self.state_pill.setText("Recording")
            self.state_pill.setStyleSheet("background:#0f2a17; border:1px solid #1e6b3a; font-weight:800; padding:6px 10px; border-radius:999px;")
        else:
            self.state_pill.setText("Paused")
            self.state_pill.setStyleSheet("background:#101322; border:1px solid #2a3145; font-weight:800; padding:6px 10px; border-radius:999px;")

    # ---------- Record controls ----------
    def toggle_record(self):
        self.recording = not self.recording
        self.record_btn.setProperty("recording", "true" if self.recording else "false")
        self.record_btn.style().unpolish(self.record_btn)
        self.record_btn.style().polish(self.record_btn)

        if self.recording:
            self.record_btn.setText("⏸ Pause")
            if not self.run_started:
                self.run_started = True
                self.t0 = time.time()
            # Flush buffer if connected
            try:
                if self.ser is not None and self.connected:
                    self.ser.reset_input_buffer()
            except Exception:
                pass
        else:
            self.record_btn.setText("● Record")

        self.refresh_state_ui()

    def reset_run(self):
        if self.recording:
            self.toggle_record()
        self.samples.clear()
        self.run_started = False
        self.t0 = None
        self.live_window.clear()
        self.last_vals = None
        self.card_samples.set_value("0")
        self.card_duration.set_value("0.0 s")
        self.card_last.set_value("-")
        self.statusBar().showMessage("Cleared run.")
        self.update_live_table(clear=True)
        self.refresh_state_ui()

    # ---------- Tick ----------
    def tick(self):
        # If not connected, just keep UI responsive
        if not self.connected or self.ser is None:
            return

        latest_line = None
        try:
            if READ_LATEST_ONLY:
                while self.ser.in_waiting:
                    latest_line = self.ser.readline().decode(errors="ignore").strip()
            else:
                if self.ser.in_waiting:
                    latest_line = self.ser.readline().decode(errors="ignore").strip()
        except Exception as e:
            # connection dropped
            self.set_device_status(False, f"Disconnected: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return

        if not latest_line:
            return

        parts = latest_line.split(",")
        if len(parts) != NUM_SENSORS:
            return

        try:
            vals = np.array([int(p) for p in parts], dtype=np.int32)
        except Exception:
            return

        self.last_vals = vals

        # Incoming rate estimate
        self._rate_count += 1
        elapsed = time.time() - self._rate_t0
        if elapsed >= 1.0:
            self._incoming_rate = self._rate_count / elapsed
            self._rate_count = 0
            self._rate_t0 = time.time()
            self.card_rate.set_value(f"{self._incoming_rate:.1f} Hz")

        # Live window
        self.live_window.append(vals.astype(np.float32))
        if len(self.live_window) > self.live_window_max:
            self.live_window.pop(0)

        # UI: last frame preview
        s_preview = ",".join(str(int(v)) for v in vals[:5])
        self.card_last.set_value(f"{s_preview}, …")

        # Store if recording
        if self.recording:
            if self.t0 is None:
                self.t0 = time.time()
                self.run_started = True

            t = time.time() - self.t0
            self.samples.append({
                "t": float(t),
                "mode": self.current_mode.lower(),       # stationary/moving
                "surface": self.current_surface.lower(), # white/black
                "vals": vals.tolist()
            })
            self.card_samples.set_value(str(len(self.samples)))
            self.card_duration.set_value(f"{t:.1f} s")

        # Update table ~2 Hz
        if int(time.time() * 10) % 5 == 0:
            self.update_live_table()

    def update_live_table(self, clear: bool = False):
        if clear or len(self.live_window) < 5:
            for i in range(NUM_SENSORS):
                self.live_table.item(i, 1).setText("-")
                self.live_table.item(i, 2).setText("-")
            return

        block = np.stack(self.live_window, axis=0)
        means = block.mean(axis=0)
        stds = block.std(axis=0, ddof=1) if block.shape[0] > 1 else np.zeros(NUM_SENSORS)

        for i in range(NUM_SENSORS):
            self.live_table.item(i, 1).setText(f"{means[i]:.1f}")
            self.live_table.item(i, 2).setText(f"{stds[i]:.1f}")

    # ---------- Export ----------
    def export_dialog(self):
        if len(self.samples) < 50:
            QtWidgets.QMessageBox.warning(
                self, "Not enough data",
                "Record some samples before exporting.\n\nTip: capture SW, SB, MW, MB."
            )
            return

        default_base = (self.run_label.text().strip() or "run") + "_report"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export report", default_base + ".pdf", "PDF files (*.pdf)"
        )
        if not path:
            return

        base = path[:-4] if path.lower().endswith(".pdf") else path
        pdf_path = base + ".pdf"
        json_path = base + ".json"
        csv_path = base + ".csv"

        try:
            self.export_all(csv_path, json_path, pdf_path)
            QtWidgets.QMessageBox.information(
                self, "Export complete",
                f"Saved:\n{pdf_path}\n{json_path}\n{csv_path}"
            )
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Export failed", str(e))

    def export_all(self, csv_path: str, json_path: str, pdf_path: str):
        # Prepare arrays
        t = np.array([s["t"] for s in self.samples], dtype=np.float64)
        mode = np.array([s["mode"] for s in self.samples])
        surf = np.array([s["surface"] for s in self.samples])
        data = np.array([s["vals"] for s in self.samples], dtype=np.int32)

        # Save CSV
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_sec", "mode", "surface"] + [f"s{i}" for i in range(NUM_SENSORS)])
            for s in self.samples:
                w.writerow([f"{s['t']:.6f}", s["mode"], s["surface"]] + s["vals"])

        # Summary JSON
        groups = [("stationary", "white"), ("stationary", "black"),
                  ("moving", "white"), ("moving", "black")]

        summary = {
            "run_label": self.run_label.text().strip() or "run",
            "port": PORT,
            "baud": BAUD,
            "num_sensors": NUM_SENSORS,
            "samples_total": int(data.shape[0]),
            "duration_sec": float(t[-1] - t[0]) if t.size > 1 else 0.0,
            "groups": {},
            "derived": {}
        }

        for m, sfc in groups:
            idx = np.where((mode == m) & (surf == sfc))[0]
            block = data[idx, :] if idx.size else np.zeros((0, NUM_SENSORS), dtype=np.int32)
            per_sensor = [asdict(compute_stats(block[:, i])) for i in range(NUM_SENSORS)]
            summary["groups"][f"{m}_{sfc}"] = {"samples": int(block.shape[0]), "per_sensor": per_sensor}

        def margin_for(mode_name: str):
            wkey = f"{mode_name}_white"
            bkey = f"{mode_name}_black"
            pw = summary["groups"].get(wkey, {}).get("per_sensor", None)
            pb = summary["groups"].get(bkey, {}).get("per_sensor", None)
            if not pw or not pb:
                return None
            return [pb[i]["p10"] - pw[i]["p90"] for i in range(NUM_SENSORS)]

        summary["derived"]["margin_stationary_p10black_minus_p90white"] = margin_for("stationary")
        summary["derived"]["margin_moving_p10black_minus_p90white"] = margin_for("moving")

        with open(json_path, "w") as f:
            json.dump(summary, f, indent=2)

        # PDF
        self.make_pdf(pdf_path, summary, data, mode, surf)

    def make_pdf(self, pdf_path: str, summary: dict, data: np.ndarray, mode: np.ndarray, surf: np.ndarray):
        with PdfPages(pdf_path) as pdf:
            # Page 1
            fig = plt.figure(figsize=(8.27, 11.69))
            plt.axis("off")
            plt.text(0.5, 0.96, "Sensor Characterization Report", ha="center", va="top",
                     fontsize=16, fontweight="bold")
            info = (
                f"Run label: {summary['run_label']}\n"
                f"Serial: {summary['port']} @ {summary['baud']}\n"
                f"Samples: {summary['samples_total']}\n"
                f"Duration: {summary['duration_sec']:.2f} sec\n\n"
                "Groups:\n"
                "  SW = stationary_white\n"
                "  SB = stationary_black\n"
                "  MW = moving_white\n"
                "  MB = moving_black\n\n"
                "Key metric per sensor:\n"
                "  margin = p10(black) - p90(white)\n"
                "If margins stay large in MOVING → thresholding is safe.\n"
            )
            plt.text(0.06, 0.88, info, va="top", fontsize=11)
            pdf.savefig(fig)
            plt.close(fig)

            # Page 2: 14 subplots × 4 violins
            fig, axes = plt.subplots(2, 7, figsize=(16, 6), constrained_layout=True)
            axes = axes.flatten()
            labels = ["SW", "SB", "MW", "MB"]

            idx_sw = np.where((mode == "stationary") & (surf == "white"))[0]
            idx_sb = np.where((mode == "stationary") & (surf == "black"))[0]
            idx_mw = np.where((mode == "moving") & (surf == "white"))[0]
            idx_mb = np.where((mode == "moving") & (surf == "black"))[0]

            for i in range(NUM_SENSORS):
                ax = axes[i]
                datasets = [
                    data[idx_sw, i] if idx_sw.size else np.array([], dtype=np.int32),
                    data[idx_sb, i] if idx_sb.size else np.array([], dtype=np.int32),
                    data[idx_mw, i] if idx_mw.size else np.array([], dtype=np.int32),
                    data[idx_mb, i] if idx_mb.size else np.array([], dtype=np.int32),
                ]
                if all(d.size == 0 for d in datasets):
                    ax.set_title(f"S{i} (no data)")
                    ax.axis("off")
                    continue
                ax.violinplot(datasets, showmeans=True, showmedians=True, showextrema=False)
                ax.set_title(f"S{i}")
                ax.set_xticks([1, 2, 3, 4])
                ax.set_xticklabels(labels)
                ax.grid(True, alpha=0.2)

            for k in range(NUM_SENSORS, len(axes)):
                axes[k].axis("off")

            fig.suptitle("Raw ADC Distributions — SW / SB / MW / MB (per sensor)", fontsize=14)
            pdf.savefig(fig)
            plt.close(fig)


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = SensorCharacterizer()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()