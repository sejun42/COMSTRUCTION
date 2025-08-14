import sys, time, json, socket
from collections import deque

import cv2
import numpy as np
from PyQt6 import QtCore, QtGui, QtWidgets

DEFAULT_RES   = (1280, 720)
DEFAULT_ROI   = (440, 160, 400, 400)
THRESH_VAL    = 240
MIN_AREA      = 50
SANDBOX_W_CM  = 100
SANDBOX_H_CM  = 100

UDP_IP   = "127.0.0.1"
UDP_PORT = 5005
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

BACKENDS = [
    ("Auto", 0),
    ("MSMF", cv2.CAP_MSMF),
    ("DSHOW", cv2.CAP_DSHOW),
]

import serial, serial.tools.list_ports

class SerialIMUReader(QtCore.QThread):
    imuReady = QtCore.pyqtSignal(dict)   # {'yaw','pitch','roll','pot1','pot2','pot3','t_ms'}
    logMsg   = QtCore.pyqtSignal(str)

    def __init__(self, port:str="", baud:int=115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self.running = False
        self.ser = None

    def set_port(self, port, baud):
        self.port = port
        self.baud = baud

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.logMsg.emit(f"[IMU] Open {self.port} @ {self.baud}")
        except Exception as e:
            self.logMsg.emit(f"[IMU] Open failed: {e}")
            return

        self.running = True
        while self.running:
            try:
                line = self.ser.readline().decode("utf-8", "ignore").strip()
                # 사람용 로그는 '#' 로 시작할 수 있으니 필터
                if not line or line[0] != "{" or line[-1] != "}":
                    continue
                d = json.loads(line)
                ypr = {
                    "yaw":   float(d.get("yaw", 0.0)),
                    "pitch": float(d.get("pitch", 0.0)),
                    "roll":  float(d.get("roll", 0.0)),
                    "pot1":  int(d.get("pot1", 0)),
                    "pot2":  int(d.get("pot2", 0)),
                    "pot3":  int(d.get("pot3", 0)),
                    "t_ms":  float(d.get("t_ms", 0.0)),
                }
                self.imuReady.emit(ypr)
            except Exception as e:
                self.logMsg.emit(f"[IMU] read error: {e}")
                time.sleep(0.05)

        try: self.ser.close()
        except: pass
        self.logMsg.emit("[IMU] Closed")

    def stop(self):
        self.running = False
        self.wait(500)

class CaptureWorker(QtCore.QThread):
    frameReady   = QtCore.pyqtSignal(np.ndarray)
    metricsReady = QtCore.pyqtSignal(dict)
    logMsg       = QtCore.pyqtSignal(str)

    def __init__(self, cam_index=0, width=1280, height=720, backend=0, parent=None):
        super().__init__(parent)
        self.cam_index = cam_index
        self.width  = width
        self.height = height
        self.backend = backend

        self.running  = False
        self.thresh   = THRESH_VAL
        self.min_area = MIN_AREA
        self.roi      = list(DEFAULT_ROI)
        self.send_udp = False

        # yaw/pitch/roll + pot1~3 포함
        self.imu_data = {"yaw":0.0, "pitch":0.0, "roll":0.0, "t_ms":0.0, "pot1":0, "pot2":0, "pot3":0}

        self.fps_list   = deque(maxlen=60)
        self.delta_list = deque(maxlen=60)
        self.speed_list = deque(maxlen=60)
        self.prev_time  = time.time()
        self.prev_x_cm  = None
        self.prev_y_cm  = None

    def set_cam(self, index, width, height, backend=0):
        self.cam_index = index
        self.width  = width
        self.height = height
        self.backend = backend

    def set_thresh(self, v): self.thresh = int(v)
    def set_min_area(self, v): self.min_area = int(v)
    def set_roi(self, r): self.roi = list(r)
    def toggle_udp(self, b): self.send_udp = bool(b)
    def set_imu(self, ypr:dict): self.imu_data = dict(self.imu_data, **ypr)

    def open_cam(self):
        if self.backend == 0:
            self.cap = cv2.VideoCapture(self.cam_index)
        else:
            self.cap = cv2.VideoCapture(self.cam_index, self.backend)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        ok = self.cap.isOpened()
        self.logMsg.emit(
            f"[Cam] Opened: idx={self.cam_index}, be={self.backend}, res={self.width}x{self.height}" if ok
            else f"[Cam] Failed: idx={self.cam_index}, be={self.backend}"
        )
        return ok

    def close_cam(self):
        try: self.cap.release()
        except: pass

    def cm_per_px(self):
        _, _, rw, rh = self.roi
        return (SANDBOX_W_CM / max(1, rw), SANDBOX_H_CM / max(1, rh))

    def run(self):
        if not self.open_cam():
            return
        self.running = True

        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                self.logMsg.emit("[Cam] Read frame failed")
                break

            H, W = frame.shape[:2]

            x, y, rw, rh = self.roi
            x = max(0, min(x, W-1)); y = max(0, min(y, H-1))
            rw = max(1, min(rw, W-x)); rh = max(1, min(rh, H-y))
            self.roi = [x, y, rw, rh]

            roi = frame[y:y+rh, x:x+rw]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, bw = cv2.threshold(gray, self.thresh, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            now = time.time()
            dt  = now - self.prev_time
            self.prev_time = now
            if dt > 0: self.fps_list.append(1.0/dt)

            found = False
            speed = 0.0
            cx = cy = None
            if contours:
                cnt = max(contours, key=cv2.contourArea)
                if cv2.contourArea(cnt) > self.min_area:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"]/M["m00"])
                        cy = int(M["m01"]/M["m00"])
                        found = True

            if found:
                cmx, cmy = self.cm_per_px()
                x_cm = cx * cmx
                y_cm = cy * cmy

                abs_x = x + cx
                abs_y = y + cy
                cv2.circle(frame, (abs_x, abs_y), 10, (0, 255, 0), -1)
                cv2.putText(frame, f"({cx},{cy})px | {x_cm:.1f},{y_cm:.1f} cm",
                            (abs_x+10, abs_y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                if self.prev_x_cm is not None and dt > 0:
                    d = ((x_cm - self.prev_x_cm)**2 + (y_cm - self.prev_y_cm)**2) ** 0.5
                    speed = d / dt
                    self.delta_list.append(d)
                    self.speed_list.append(speed)
                self.prev_x_cm, self.prev_y_cm = x_cm, y_cm

                # 로그
                self.logMsg.emit(
                    f"[Camera] Pos=({x_cm:.2f},{y_cm:.2f}) cm, Speed={speed:.2f} cm/s | "
                    f"IMU ypr=({self.imu_data['yaw']:.1f},{self.imu_data['pitch']:.1f},{self.imu_data['roll']:.1f}) | "
                    f"pots=({self.imu_data['pot1']},{self.imu_data['pot2']},{self.imu_data['pot3']})"
                )

                # === UDP 전송 ===
                if self.send_udp:
                    payload = {
                        "x": x_cm, "y": y_cm, "speed": speed,
                        "yaw": self.imu_data.get("yaw", 0.0),
                        "pitch": self.imu_data.get("pitch", 0.0),
                        "roll": self.imu_data.get("roll", 0.0),
                        "pot1": int(self.imu_data.get("pot1", 0)),
                        "pot2": int(self.imu_data.get("pot2", 0)),
                        "pot3": int(self.imu_data.get("pot3", 0)),
                        "t_send_ms": time.time() * 1000.0
                    }
                    try:
                        udp_sock.sendto(json.dumps(payload).encode("utf-8"), (UDP_IP, UDP_PORT))
                    except Exception:
                        pass

            cv2.rectangle(frame, (x, y), (x+rw, y+rh), (255, 0, 0), 2)

            self.frameReady.emit(frame)
            metrics = {
                "fps": (sum(self.fps_list)/len(self.fps_list)) if self.fps_list else 0.0,
                "delta_avg": (sum(self.delta_list)/len(self.delta_list)) if self.delta_list else 0.0,
                "delta_std": (float(np.std(self.delta_list))) if self.delta_list else 0.0,
                "speed_avg": (sum(self.speed_list)/len(self.speed_list)) if self.speed_list else 0.0,
                "sent": bool(self.send_udp and found),
                "thresh": self.thresh, "minA": self.min_area,
                "roi": tuple(self.roi), "res": (W, H),
                "backend": self.backend, "index": self.cam_index
            }
            self.metricsReady.emit(metrics)

        self.close_cam()

    def stop(self):
        self.running = False
        self.wait(500)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Flash Tracker – UDP + IMU+POT")
        self.resize(1280, 800)

        self.worker = CaptureWorker()
        self.worker.frameReady.connect(self.updateFrame)
        self.worker.metricsReady.connect(self.updateMetrics)
        self.worker.logMsg.connect(self.log)

        self.videoLabel = QtWidgets.QLabel(alignment=QtCore.Qt.AlignmentFlag.AlignCenter)
        self.videoLabel.setStyleSheet("background:#111;")
        self.setCentralWidget(self.videoLabel)

        self.leftPanel = self.makeLeftPanel()

        # --- 여기 두 줄이 **핵심 수정** ---
        dockLeft = QtWidgets.QDockWidget("설정", self)
        dockLeft.setWidget(self.leftPanel)
        dockLeft.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.NoDockWidgetFeatures)  # ✅ PyQt6 방식
        dockLeft.setAllowedAreas(QtCore.Qt.DockWidgetArea.LeftDockWidgetArea)               # ✅ PyQt6 방식
        self.addDockWidget(QtCore.Qt.DockWidgetArea.LeftDockWidgetArea, dockLeft)

        self.console = QtWidgets.QPlainTextEdit(readOnly=True)
        self.console.setStyleSheet("background:#0b0f13;color:#d6dde6;font-family:Consolas,monospace;")

        dockBottom = QtWidgets.QDockWidget("Console", self)
        dockBottom.setWidget(self.console)
        dockBottom.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.NoDockWidgetFeatures) # ✅ PyQt6 방식
        dockBottom.setAllowedAreas(QtCore.Qt.DockWidgetArea.BottomDockWidgetArea)           # ✅ PyQt6 방식
        self.addDockWidget(QtCore.Qt.DockWidgetArea.BottomDockWidgetArea, dockBottom)

        self.btnRefresh.clicked.connect(self.refresh_camera_list)
        self.btnApply.clicked.connect(self.apply_params)
        self.btnRestart.clicked.connect(self.restart_capture)
        self.spinThresh.valueChanged.connect(lambda v: self.worker.set_thresh(v))
        self.spinMinA.valueChanged.connect(lambda v: self.worker.set_min_area(v))
        self.chkUDP.toggled.connect(self.worker.toggle_udp)

        self.imuReader = SerialIMUReader()
        self.imuReader.imuReady.connect(self.onImuData)
        self.imuReader.logMsg.connect(self.log)
        self.btnIMURefresh.clicked.connect(self.refresh_imu_ports)
        self.btnIMUConnect.clicked.connect(self.connect_imu)
        self.btnIMUDisconnect.clicked.connect(self.disconnect_imu)

        self.refresh_camera_list()
        self.refresh_imu_ports()
        self.start_capture()

    def makeLeftPanel(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w); lay.setContentsMargins(8,8,8,8)

        self.comboCam = QtWidgets.QComboBox()
        lay.addWidget(self.comboCam)

        self.btnRefresh = QtWidgets.QPushButton("카메라 새로고침")
        lay.addWidget(self.btnRefresh)

        self.comboBackend = QtWidgets.QComboBox()
        for name, code in BACKENDS:
            self.comboBackend.addItem(name, userData=code)
        self.comboBackend.setCurrentIndex(0)
        lay.addWidget(self.comboBackend)

        self.comboRes = QtWidgets.QComboBox()
        for r in ["1280x720", "1920x1080", "640x480"]:
            self.comboRes.addItem(r)
        self.comboRes.setCurrentText(f"{DEFAULT_RES[0]}x{DEFAULT_RES[1]}")
        lay.addWidget(self.comboRes)

        lab = QtWidgets.QLabel("FPS는 장치/드라이버에서 설정됨"); lab.setStyleSheet("color:#aaa;")
        lay.addWidget(lab)

        self.spinThresh = QtWidgets.QSpinBox(); self.spinThresh.setRange(0,255); self.spinThresh.setValue(THRESH_VAL)
        self.spinMinA   = QtWidgets.QSpinBox(); self.spinMinA.setRange(1,20000); self.spinMinA.setValue(MIN_AREA)
        form = QtWidgets.QFormLayout(); form.addRow("Threshold", self.spinThresh); form.addRow("Min Area", self.spinMinA)
        lay.addLayout(form)

        self.spRoiX = QtWidgets.QSpinBox(); self.spRoiX.setRange(0, 10000); self.spRoiX.setValue(DEFAULT_ROI[0])
        self.spRoiY = QtWidgets.QSpinBox(); self.spRoiY.setRange(0, 10000); self.spRoiY.setValue(DEFAULT_ROI[1])
        self.spRoiW = QtWidgets.QSpinBox(); self.spRoiW.setRange(1, 10000); self.spRoiW.setValue(DEFAULT_ROI[2])
        self.spRoiH = QtWidgets.QSpinBox(); self.spRoiH.setRange(1, 10000); self.spRoiH.setValue(DEFAULT_ROI[3])
        form2 = QtWidgets.QFormLayout()
        form2.addRow("ROI X", self.spRoiX); form2.addRow("ROI Y", self.spRoiY)
        form2.addRow("ROI W", self.spRoiW); form2.addRow("ROI H", self.spRoiH)
        lay.addLayout(form2)

        self.chkUDP = QtWidgets.QCheckBox("Unity로 UDP 전송")
        lay.addWidget(self.chkUDP)

        sep = QtWidgets.QLabel("── IMU (Serial) ──"); sep.setStyleSheet("color:#888;")
        lay.addWidget(sep)

        self.comboIMU = QtWidgets.QComboBox()
        self.btnIMURefresh = QtWidgets.QPushButton("포트 새로고침")
        row_imu = QtWidgets.QHBoxLayout()
        row_imu.addWidget(self.comboIMU, 1)
        row_imu.addWidget(self.btnIMURefresh)
        lay.addLayout(row_imu)

        self.spinBaud = QtWidgets.QSpinBox(); self.spinBaud.setRange(9600, 1000000); self.spinBaud.setValue(115200)
        form3 = QtWidgets.QFormLayout(); form3.addRow("Baud", self.spinBaud)
        lay.addLayout(form3)

        self.btnIMUConnect    = QtWidgets.QPushButton("IMU 연결")
        self.btnIMUDisconnect = QtWidgets.QPushButton("IMU 해제")
        lay.addWidget(self.btnIMUConnect)
        lay.addWidget(self.btnIMUDisconnect)

        lay.addStretch(1)
        self.btnApply   = QtWidgets.QPushButton("적용")
        self.btnRestart = QtWidgets.QPushButton("카메라 재시작")
        lay.addWidget(self.btnApply)
        lay.addWidget(self.btnRestart)

        return w

    def refresh_camera_list(self):
        self.comboCam.clear()
        found = []

        try:
            from pygrabber.dshow_graph import FilterGraph
            graph = FilterGraph()
            names = graph.get_input_devices()
            for i, name in enumerate(names):
                found.append({"label": f"{name} (DSHOW #{i})", "index": i, "backend": cv2.CAP_DSHOW, "name": name})
        except Exception:
            pass

        for i in range(0, 20):
            cap = cv2.VideoCapture(i, cv2.CAP_MSMF)
            ok = cap.isOpened()
            if ok:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                ok = cap.read()[0]
            if ok:
                found.append({"label": f"MSMF #{i}", "index": i, "backend": cv2.CAP_MSMF, "name": None})
            cap.release()

        if not found:
            self.comboCam.addItem("없음")
        else:
            for item in found:
                self.comboCam.addItem(item["label"], userData=item)
            self.comboCam.setCurrentIndex(0)

    def refresh_imu_ports(self):
        self.comboIMU.clear()
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            self.comboIMU.addItem("없음")
            return
        for p in ports:
            self.comboIMU.addItem(f"{p.device} - {p.description}", userData=p.device)

    def connect_imu(self):
        port = self.comboIMU.currentData()
        if not port:
            self.log("[IMU] 포트를 선택하세요"); return
        baud = int(self.spinBaud.value())
        if self.imuReader.isRunning():
            self.log("[IMU] 이미 연결됨"); return
        self.imuReader.set_port(port, baud)
        self.imuReader.start()

    def disconnect_imu(self):
        if self.imuReader.isRunning():
            self.imuReader.stop()

    def apply_params(self):
        roi = (self.spRoiX.value(), self.spRoiY.value(), self.spRoiW.value(), self.spRoiH.value())
        self.worker.set_roi(roi)

        try:
            w, h = map(int, self.comboRes.currentText().split("x"))
        except:
            w, h = DEFAULT_RES

        selected = self.comboCam.currentData()
        if selected is None:
            cam_index = 0; sel_backend = 0; cam_name = "Unknown"
        else:
            cam_index   = selected["index"]
            sel_backend = selected["backend"]
            cam_name    = selected.get("name") or selected["label"]

        backend_override = self.comboBackend.currentData() or 0
        backend_to_use   = sel_backend if backend_override == 0 else backend_override

        self.worker.set_cam(cam_index, w, h, backend_to_use)
        self.log(f"[UI] Apply: '{cam_name}' idx={cam_index}, be={backend_to_use}, res={w}x{h}, ROI={roi}, "
                 f"th={self.spinThresh.value()}, minA={self.spinMinA.value()}")

    def restart_capture(self):
        self.stop_capture()
        self.start_capture()

    def start_capture(self):
        self.apply_params()
        if not self.worker.isRunning():
            self.worker.start()

    def stop_capture(self):
        if self.worker.isRunning():
            self.worker.stop()

    @QtCore.pyqtSlot(np.ndarray)
    def updateFrame(self, frame_bgr):
        h, w, _ = frame_bgr.shape
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        qimg = QtGui.QImage(rgb.data, w, h, 3*w, QtGui.QImage.Format.Format_RGB888)
        self.videoLabel.setPixmap(QtGui.QPixmap.fromImage(qimg))

    @QtCore.pyqtSlot(dict)
    def updateMetrics(self, m):
        self.statusBar().showMessage(
            f"FPS {m['fps']:.1f} | ΔPos {m['delta_avg']:.2f}cm (σ={m['delta_std']:.2f}) | v_avg {m['speed_avg']:.2f}cm/s | "
            f"sent={m['sent']} | ROI={m['roi']} | Res={m['res']} | idx={m['index']} be={m['backend']}"
        )

    def onImuData(self, ypr:dict):
        # 상태바 간단 표시 + 워커에 전달
        self.statusBar().showMessage(
            f"IMU yaw={ypr['yaw']:.1f} pitch={ypr['pitch']:.1f} roll={ypr['roll']:.1f} | "
            f"pots=({ypr.get('pot1',0)},{ypr.get('pot2',0)},{ypr.get('pot3',0)})"
        )
        self.log(f"[IMU] ypr=({ypr['yaw']:.2f},{ypr['pitch']:.2f},{ypr['roll']:.2f}) pots=({ypr.get('pot1',0)},{ypr.get('pot2',0)},{ypr.get('pot3',0)})")
        self.worker.set_imu(ypr)

    def log(self, text):
        ts = time.strftime("%H:%M:%S")
        self.console.appendPlainText(f"[{ts}] {text}")

    def closeEvent(self, e: QtGui.QCloseEvent) -> None:
        self.stop_capture()
        try:
            if self.imuReader.isRunning():
                self.imuReader.stop()
        except:
            pass
        super().closeEvent(e)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
