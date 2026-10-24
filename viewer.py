"""Unitree L2 LiDAR viewer (PySide6, software-rendered 3D).

Features
--------
* Live serial connection (any COM port, default 2,000,000 baud)
* Raw capture replay (.bin recorded by the library)
* 3D orbit / pan / zoom point cloud, color by height / intensity / range / azimuth
* Live IMU, device status and packet statistics
* Save PLY, save PNG snapshot, record raw stream
* Request firmware version, switch working mode, LED mode

The 3D view is rendered with numpy into a QImage (painter's algorithm),
so it works on machines without OpenGL (remote desktop, VMs, ...).
"""

from __future__ import annotations

import math
import os
import queue
import sys
import time
from dataclasses import dataclass, field

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt, Signal

from unilidar_l2 import Calibration, UnitreeL2
from unilidar_l2.geometry import save_ply
from unilidar_l2.lidar import ImuEvent, ScanEvent, TimeSyncEvent, VersionEvent
from unilidar_l2.protocol import LED_MODES, WORKING_MODE_NORMAL, WORKING_MODE_STANDBY
from unilidar_l2.serial_io import list_ports

APP_TITLE = "Unitree L2 LiDAR Viewer"

# ---------------------------------------------------------------------------
# Colour maps
# ---------------------------------------------------------------------------

_TURBO = np.array([
    [48, 18, 59], [70, 107, 227], [27, 180, 196], [90, 233, 84],
    [199, 220, 32], [253, 139, 20], [180, 4, 38],
], dtype=np.float64)
_TURBO_X = np.linspace(0.0, 1.0, len(_TURBO))


def colormap(t: np.ndarray) -> np.ndarray:
    t = np.clip(np.asarray(t, dtype=np.float64), 0.0, 1.0)
    r = np.interp(t, _TURBO_X, _TURBO[:, 0])
    g = np.interp(t, _TURBO_X, _TURBO[:, 1])
    b = np.interp(t, _TURBO_X, _TURBO[:, 2])
    return np.stack((r, g, b), axis=1).astype(np.uint8)


def compute_colors(pts: np.ndarray, mode: str, max_range: float) -> np.ndarray:
    n = len(pts)
    if n == 0:
        return np.zeros((0, 3), np.uint8)
    x, y, z, inten = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
    if mode == "intensity":
        t = inten / 255.0
    elif mode == "range":
        rng = np.sqrt(x * x + y * y + z * z)
        t = rng / max(1e-3, max_range)
    elif mode == "azimuth":
        ang = np.arctan2(y, x)
        t = (ang + math.pi) / (2 * math.pi)
    elif mode == "uniform":
        return np.tile(np.array([[0, 210, 255]], np.uint8), (n, 1))
    else:  # height
        sample = z[:: max(1, n // 4000)]
        lo, hi = np.percentile(sample, [2, 98]) if len(sample) > 20 else (z.min(), z.max())
        t = (z - lo) / max(1e-6, hi - lo)
    return colormap(t)


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

@dataclass
class Camera:
    yaw: float = -0.95
    pitch: float = 0.42
    dist: float = 11.0
    target: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.6]))
    fov_deg: float = 55.0

    def reset(self) -> None:
        self.yaw, self.pitch, self.dist = -0.95, 0.42, 11.0
        self.target = np.array([0.0, 0.0, 1.6])

    def basis(self):
        cp = math.cos(self.pitch)
        direction = np.array([cp * math.cos(self.yaw), cp * math.sin(self.yaw), math.sin(self.pitch)])
        eye = self.target + self.dist * direction
        forward = self.target - eye
        forward /= np.linalg.norm(forward) + 1e-12
        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(forward, world_up)
        if np.linalg.norm(right) < 1e-6:
            right = np.array([1.0, 0.0, 0.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        return eye, right, up, forward


# ---------------------------------------------------------------------------
# Point cloud ring buffer
# ---------------------------------------------------------------------------

class CloudStore:
    def __init__(self, capacity: int = 250_000) -> None:
        self.capacity = int(capacity)
        self.pts = np.empty((self.capacity, 4), np.float32)
        self.count = 0
        self.head = 0

    def resize(self, capacity: int) -> None:
        capacity = int(capacity)
        if capacity == self.capacity:
            return
        old = self.pts[: self.count]
        self.capacity = capacity
        self.pts = np.empty((capacity, 4), np.float32)
        self.count = 0
        self.head = 0
        if len(old):
            self.add(old[-capacity:])

    def clear(self) -> None:
        self.count = 0
        self.head = 0

    def add(self, arr: np.ndarray) -> None:
        n = len(arr)
        if n == 0:
            return
        if n >= self.capacity:
            self.pts[:] = arr[-self.capacity:]
            self.count = self.capacity
            self.head = 0
            return
        end = self.head + n
        if end <= self.capacity:
            self.pts[self.head:end] = arr
        else:
            k = self.capacity - self.head
            self.pts[self.head:] = arr[:k]
            self.pts[: end - self.capacity] = arr[k:]
        self.head = end % self.capacity
        self.count = min(self.capacity, self.count + n)

    def array(self) -> np.ndarray:
        return self.pts[: self.count]


# ---------------------------------------------------------------------------
# Worker thread
# ---------------------------------------------------------------------------

class LidarWorker(QtCore.QThread):
    cloud = Signal(object, object)      # points (N,4) float32, info dict
    imu = Signal(object)                # ImuPacket
    version = Signal(object)            # VersionInfo
    log = Signal(str)
    error = Signal(str)
    state = Signal(str)

    def __init__(self, port: str = "COM8", baud: int = 2_000_000,
                 replay_path: str | None = None, replay_speed: float = 1.0,
                 calibration: Calibration | None = None, parent=None) -> None:
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self.replay_path = replay_path
        self.replay_speed = max(0.05, replay_speed)
        self.calibration = calibration or Calibration()
        self._stop = False
        self._commands: "queue.Queue[tuple]" = queue.Queue()
        self._record_file = None

    # -- public, thread safe -----------------------------------------
    def stop(self) -> None:
        self._stop = True

    def request_version(self) -> None:
        self._commands.put(("version",))

    def set_working_mode(self, mode: int) -> None:
        self._commands.put(("mode", mode))

    def set_led_mode(self, mode: int) -> None:
        self._commands.put(("led", mode))

    def start_record(self, path: str) -> None:
        self._commands.put(("record_start", path))

    def stop_record(self) -> None:
        self._commands.put(("record_stop",))

    # -- helpers ------------------------------------------------------
    def _drain_commands(self, lidar: UnitreeL2 | None) -> None:
        while True:
            try:
                cmd = self._commands.get_nowait()
            except queue.Empty:
                break
            try:
                if lidar is not None and cmd[0] == "version":
                    lidar.request_version()
                elif lidar is not None and cmd[0] == "mode":
                    lidar.set_working_mode(cmd[1])
                    self.log.emit(f"工作模式 -> {cmd[1]}")
                elif lidar is not None and cmd[0] == "led":
                    lidar.set_led_mode(cmd[1])
                    self.log.emit(f"LED 模式 -> {cmd[1]}")
                elif cmd[0] == "record_start":
                    self._record_file = open(cmd[1], "wb")
                    if lidar is not None:
                        lidar.raw_callback = self._write_raw
                    self.log.emit(f"开始录制 -> {cmd[1]}")
                elif cmd[0] == "record_stop":
                    self._close_record()
                    if lidar is not None:
                        lidar.raw_callback = None
                elif cmd[0] == "get_version" and lidar is not None:
                    lidar.request_version()
            except Exception as exc:  # noqa: BLE001
                self.log.emit(f"命令失败: {exc}")

    def _write_raw(self, data: bytes) -> None:
        if self._record_file is not None:
            try:
                self._record_file.write(data)
            except Exception:
                self._close_record()

    def _close_record(self) -> None:
        if self._record_file is not None:
            try:
                self._record_file.close()
            except Exception:
                pass
            self._record_file = None
            self.log.emit("录制已停止")

    # -- thread body ---------------------------------------------------
    def run(self) -> None:  # noqa: C901 - straightforward state machine
        if self.replay_path:
            self._run_replay()
        else:
            self._run_live()

    def _run_live(self) -> None:
        lidar = None
        try:
            lidar = UnitreeL2(self.port, self.baud, self.calibration)
            lidar.open()
            self.state.emit(f"已连接 {self.port} @ {self.baud}")
            self.log.emit(f"串口已打开 {self.port} @ {self.baud}")
            lidar.request_version()
            batch = []
            last_emit = time.monotonic()
            last_stats = time.monotonic()
            n_scans = n_imu = n_bytes = n_points = 0
            info = self._empty_info()
            while not self._stop:
                self._drain_commands(lidar)
                events = lidar.poll(0.02)
                for ev in events:
                    if isinstance(ev, ScanEvent):
                        batch.append(ev.points)
                        n_scans += 1
                        n_points += len(ev.points)
                        info.update(self._scan_info(ev))
                    elif isinstance(ev, ImuEvent):
                        n_imu += 1
                        if n_imu % 8 == 1:
                            self.imu.emit(ev.imu)
                    elif isinstance(ev, VersionEvent):
                        self.version.emit(ev.version)
                        self.log.emit(f"固件: {ev.version}")
                    elif isinstance(ev, TimeSyncEvent):
                        pass
                n_bytes = lidar.stats.bytes_in
                now = time.monotonic()
                if now - last_emit >= 0.05:
                    self._emit_batch(batch, info, n_scans, n_imu, n_bytes,
                                     now - last_stats, lidar.stats.crc_errors, n_points)
                    last_emit = now
                if now - last_stats >= 1.0:
                    last_stats = now
        except Exception as exc:  # noqa: BLE001
            self.error.emit(f"{type(exc).__name__}: {exc}")
            self.state.emit("连接已断开")
        finally:
            self._close_record()
            if lidar is not None:
                try:
                    lidar.close()
                except Exception:
                    pass

    def _run_replay(self) -> None:
        try:
            with open(self.replay_path, "rb") as f:
                raw = f.read()
        except Exception as exc:  # noqa: BLE001
            self.error.emit(f"无法读取文件: {exc}")
            return
        self.state.emit(f"回放 {os.path.basename(self.replay_path)}")
        self.log.emit(f"回放 {self.replay_path} ({len(raw)} 字节)")
        reader = UnitreeL2("", self.baud, self.calibration)
        chunk = 6000
        pos = 0
        batch = []
        info = self._empty_info()
        last_emit = time.monotonic()
        n_scans = n_imu = n_points = 0
        delay = chunk / (100_000.0 * self.replay_speed)
        while not self._stop:
            if pos >= len(raw):
                pos = 0
                reader._parser.reset()
                reader._pending_aux.clear()
            data = raw[pos:pos + chunk]
            pos += chunk
            for ev in reader.process_bytes(data):
                if isinstance(ev, ScanEvent):
                    batch.append(ev.points)
                    n_scans += 1
                    n_points += len(ev.points)
                    info.update(self._scan_info(ev))
                elif isinstance(ev, ImuEvent):
                    n_imu += 1
                    if n_imu % 8 == 1:
                        self.imu.emit(ev.imu)
                elif isinstance(ev, VersionEvent):
                    self.version.emit(ev.version)
            now = time.monotonic()
            if now - last_emit >= 0.05:
                self._emit_batch(batch, info, n_scans, n_imu, pos, 0.0,
                                 reader.stats.crc_errors, n_points)
                last_emit = now
            time.sleep(delay)

    def _emit_batch(self, batch, info, n_scans, n_imu, n_bytes, dt, crc=None, n_points=0) -> None:
        pts = None
        if batch:
            pts = np.concatenate(batch, axis=0) if len(batch) > 1 else batch[0]
            batch.clear()
        stats = dict(info)
        stats["scans"] = n_scans
        stats["imu_total"] = n_imu
        stats["bytes"] = n_bytes
        stats["points"] = n_points
        if crc is not None:
            stats["crc"] = crc
        self.cloud.emit(pts, stats)

    @staticmethod
    def _empty_info() -> dict:
        return {
            "work_status": -1, "apd_temp": 0.0, "imu_temp": 0.0, "dirty": 0.0,
            "scan_rate": 0.0, "com_period": 0.0, "sys_period": 0.0,
            "h_start": 0.0, "h_step": 0.0, "v_start": 0.0, "v_span": 0.0,
            "packet_id": 0, "valid_points": 0, "timestamp": 0.0,
        }

    @staticmethod
    def _scan_info(ev: ScanEvent) -> dict:
        a = ev.aux
        return {
            "work_status": a.lidar_work_status,
            "apd_temp": a.apd_temperature,
            "imu_temp": a.imu_temperature,
            "dirty": a.dirty_index,
            "scan_rate": a.scan_rate_hz,
            "com_period": a.com_rotation_period,
            "sys_period": a.sys_rotation_period,
            "h_start": a.com_horizontal_angle_start,
            "h_step": a.com_horizontal_angle_step,
            "v_start": a.sys_vertical_angle_start,
            "v_span": a.sys_vertical_angle_span,
            "packet_id": a.packet_id,
            "valid_points": int(len(ev.points)),
            "timestamp": a.timestamp,
        }


# ---------------------------------------------------------------------------
# 3D view
# ---------------------------------------------------------------------------

class PointCloudView(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(560, 420)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(False)
        self.camera = Camera()
        self.store = CloudStore(250_000)
        self.color_mode = "height"
        self.point_size = 2
        self.max_range = 50.0
        self._img = None
        self._bg = None
        self._qimg = None
        self._dirty = True
        self._last_pos = None
        self._button = None
        self._fps = 0.0
        self._fps_t = time.monotonic()
        self._frames = 0
        self._overlay_note = ""
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(33)

    # -- data ---------------------------------------------------------
    def add_points(self, pts: np.ndarray) -> None:
        self.store.add(pts)
        self._dirty = True

    def clear(self) -> None:
        self.store.clear()
        self._dirty = True

    def set_color_mode(self, mode: str) -> None:
        self.color_mode = mode
        self._dirty = True

    def set_point_size(self, size: int) -> None:
        self.point_size = int(size)
        self._dirty = True

    def set_max_range(self, value: float) -> None:
        self.max_range = float(value)
        self._dirty = True

    def set_capacity(self, capacity: int) -> None:
        self.store.resize(capacity)
        self._dirty = True

    def reset_view(self) -> None:
        self.camera.reset()
        self._dirty = True

    def save_image(self, path: str) -> bool:
        self._render()
        if self._qimg is None:
            return False
        return self._qimg.save(path)

    # -- interaction ---------------------------------------------------
    def _tick(self) -> None:
        if self._dirty:
            self.update()

    def mousePressEvent(self, e: QtGui.QMouseEvent) -> None:
        self._last_pos = e.position()
        self._button = e.button()

    def mouseReleaseEvent(self, e: QtGui.QMouseEvent) -> None:
        self._button = None
        self._last_pos = None

    def mouseMoveEvent(self, e: QtGui.QMouseEvent) -> None:
        if self._last_pos is None:
            return
        p = e.position()
        dx = p.x() - self._last_pos.x()
        dy = p.y() - self._last_pos.y()
        self._last_pos = p
        if self._button == Qt.LeftButton:
            self.camera.yaw -= dx * 0.008
            self.camera.pitch = max(-1.45, min(1.45, self.camera.pitch + dy * 0.008))
        elif self._button in (Qt.RightButton, Qt.MiddleButton):
            eye, right, up, _ = self.camera.basis()
            k = self.camera.dist * 0.0016
            self.camera.target = self.camera.target - right * dx * k + up * dy * k
        self._dirty = True

    def wheelEvent(self, e: QtGui.QWheelEvent) -> None:
        delta = e.angleDelta().y()
        self.camera.dist *= math.exp(-delta / 1200.0)
        self.camera.dist = max(0.6, min(120.0, self.camera.dist))
        self._dirty = True

    def keyPressEvent(self, e: QtGui.QKeyEvent) -> None:
        if e.key() in (Qt.Key_R, Qt.Key_Home):
            self.reset_view()
        else:
            super().keyPressEvent(e)

    def mouseDoubleClickEvent(self, e: QtGui.QMouseEvent) -> None:
        self.reset_view()

    def resizeEvent(self, e: QtGui.QResizeEvent) -> None:
        self._img = None
        self._bg = None
        self._dirty = True

    # -- rendering -----------------------------------------------------
    def _make_bg(self, w: int, h: int) -> np.ndarray:
        top = np.array([18, 22, 30], np.float64)
        bottom = np.array([5, 6, 9], np.float64)
        t = np.linspace(0.0, 1.0, h)[:, None]
        grad = top[None, :] * (1 - t) + bottom[None, :] * t  # (h,3)
        return np.repeat(grad[:, None, :], w, axis=1).astype(np.uint8)

    @staticmethod
    def _draw_line(img: np.ndarray, x0, y0, x1, y1, color) -> None:
        h, w = img.shape[:2]
        n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        if n <= 1 or n > 20000:
            return
        xs = np.linspace(x0, x1, n).astype(np.int32)
        ys = np.linspace(y0, y1, n).astype(np.int32)
        m = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
        if m.any():
            img[ys[m], xs[m]] = color

    def _project(self, pts: np.ndarray, w: int, h: int):
        eye, right, up, forward = self.camera.basis()
        rel = pts - eye
        xc = rel @ right
        yc = rel @ up
        zc = rel @ forward
        focal = (h * 0.5) / math.tan(math.radians(self.camera.fov_deg) * 0.5)
        zz = np.where(zc > 1e-4, zc, 1e-4)
        sx = w * 0.5 + (xc / zz) * focal
        sy = h * 0.5 - (yc / zz) * focal
        return sx, sy, zc

    def _render(self) -> None:
        w = max(64, self.width())
        h = max(64, self.height())
        if self._img is None or self._img.shape[0] != h or self._img.shape[1] != w:
            self._bg = self._make_bg(w, h)
            self._img = np.empty((h, w, 3), np.uint8)
        img = self._img
        img[:] = self._bg

        # ground grid + axes (drawn behind the points)
        g = 10.0
        for k in range(-10, 11):
            p0 = np.array([[k, -g, 0.0], [k, g, 0.0], [-g, k, 0.0], [g, k, 0.0]])
            sx, sy, zc = self._project(p0, w, h)
            col = np.array([40, 46, 58], np.uint8) if k != 0 else np.array([70, 78, 95], np.uint8)
            if zc[0] > 1e-3 and zc[1] > 1e-3:
                self._draw_line(img, sx[0], sy[0], sx[1], sy[1], col)
            if zc[2] > 1e-3 and zc[3] > 1e-3:
                self._draw_line(img, sx[2], sy[2], sx[3], sy[3], col)
        origin = np.array([[0, 0, 0], [1.5, 0, 0], [0, 0, 0], [0, 1.5, 0], [0, 0, 0], [0, 0, 1.5]])
        sx, sy, zc = self._project(origin, w, h)
        for a, b, col in ((0, 1, (230, 70, 70)), (2, 3, (70, 220, 90)), (4, 5, (90, 140, 255))):
            if zc[a] > 1e-3 and zc[b] > 1e-3:
                self._draw_line(img, sx[a], sy[a], sx[b], sy[b], np.array(col, np.uint8))

        pts = self.store.array()
        n = len(pts)
        if n:
            colors = compute_colors(pts, self.color_mode, self.max_range)
            sx, sy, zc = self._project(pts[:, :3], w, h)
            m = (zc > 0.05) & np.isfinite(sx) & np.isfinite(sy)
            sx = sx[m]
            sy = sy[m]
            zc = zc[m]
            colors = colors[m]
            if len(sx):
                order = np.argsort(zc)[::-1]
                sx = sx[order].astype(np.int32)
                sy = sy[order].astype(np.int32)
                colors = colors[order]
                size = max(1, min(4, int(self.point_size)))
                offs = [(dx, dy) for dy in range(size) for dx in range(size)]
                for dx, dy in offs:
                    xx = sx + dx
                    yy = sy + dy
                    ok = (xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)
                    if ok.any():
                        img[yy[ok], xx[ok]] = colors[ok]

        self._qimg = QtGui.QImage(img.data, w, h, 3 * w,
                                  QtGui.QImage.Format_RGB888).copy()
        self._dirty = False
        self._frames += 1
        now = time.monotonic()
        if now - self._fps_t >= 1.0:
            self._fps = self._frames / (now - self._fps_t)
            self._frames = 0
            self._fps_t = now

    def paintEvent(self, e: QtGui.QPaintEvent) -> None:
        if self._dirty or self._qimg is None:
            self._render()
        p = QtGui.QPainter(self)
        if self._qimg is not None:
            p.drawImage(0, 0, self._qimg)
        p.setPen(QtGui.QColor(200, 210, 225))
        f = p.font()
        f.setPointSize(9)
        p.setFont(f)
        p.drawText(10, 18, f"{self.color_mode}  |  {self.store.count} pts  |  {self._fps:4.1f} FPS")
        if self._overlay_note:
            p.drawText(10, 34, self._overlay_note)
        p.setPen(QtGui.QColor(150, 160, 180))
        p.drawText(10, self.height() - 10,
                   "左键旋转   右键/中键平移   滚轮缩放   R/双击 复位")
        p.end()

    def set_note(self, text: str) -> None:
        self._overlay_note = text
        self._dirty = True


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1360, 860)
        self.worker: LidarWorker | None = None
        self._last_stats = {"scans": 0, "imu_total": 0, "bytes": 0}
        self._rate_t = time.monotonic()
        self._rate_scans = 0
        self._rate_imu = 0
        self._rate_bytes = 0
        self._prev_scans = 0
        self._prev_imu = 0
        self._prev_bytes = 0
        self._prev_points = 0
        self._rate_points = 0
        self._rate_timer = QtCore.QTimer(self)
        self._rate_timer.timeout.connect(self._update_rates)
        self._rate_timer.start(1000)

        self.view = PointCloudView()

        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        split_h = QtWidgets.QSplitter(Qt.Horizontal)
        split_h.addWidget(self._build_left_panel())
        split_h.addWidget(self.view)
        split_h.addWidget(self._build_right_panel())
        split_h.setStretchFactor(1, 1)
        split_h.setSizes([280, 820, 260])

        split_v = QtWidgets.QSplitter(Qt.Vertical)
        split_v.addWidget(split_h)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setFixedHeight(110)
        split_v.addWidget(self.log)
        split_v.setStretchFactor(0, 1)
        root.addWidget(split_v)
        self.setCentralWidget(central)

        self.statusBar().showMessage("未连接")
        self._apply_style()
        self._refresh_ports()

    # -- UI construction ----------------------------------------------
    def _apply_style(self) -> None:
        self.setStyleSheet("""
            QWidget { background: #15181e; color: #d8dee9; }
            QGroupBox { border: 1px solid #2c323d; border-radius: 6px; margin-top: 10px;
                        padding-top: 8px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px;
                               color: #8fb7ff; }
            QPushButton { background: #232936; border: 1px solid #333c4d; border-radius: 4px;
                          padding: 6px 10px; }
            QPushButton:hover { background: #2c3444; }
            QPushButton:pressed { background: #1b2029; }
            QPushButton:disabled { color: #666; border-color: #262b34; }
            QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit, QPlainTextEdit {
                background: #1b2029; border: 1px solid #333c4d; border-radius: 4px; padding: 3px; }
            QLabel#value { color: #7ee0c0; font-family: Consolas, monospace; }
            QLabel#hint { color: #7c8698; }
            QStatusBar { background: #10131a; }
        """)

    def _group(self, title: str) -> QtWidgets.QGroupBox:
        gb = QtWidgets.QGroupBox(title)
        lay = QtWidgets.QVBoxLayout(gb)
        lay.setSpacing(6)
        return gb

    def _build_left_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        panel.setMinimumWidth(250)
        panel.setMaximumWidth(340)
        lay = QtWidgets.QVBoxLayout(panel)
        lay.setContentsMargins(8, 8, 8, 8)

        # connection
        gb = self._group("连接 / Connection")
        row = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(110)
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.setEditable(True)
        for b in (2_000_000, 1_500_000, 921_600, 460_800, 230_400, 115_200):
            self.baud_combo.addItem(str(b))
        row.addWidget(self.port_combo, 1)
        row.addWidget(self.baud_combo, 1)
        gb.layout().addLayout(row)
        row2 = QtWidgets.QHBoxLayout()
        self.refresh_btn = QtWidgets.QPushButton("刷新")
        self.connect_btn = QtWidgets.QPushButton("连接")
        self.replay_btn = QtWidgets.QPushButton("回放 .bin")
        row2.addWidget(self.refresh_btn)
        row2.addWidget(self.connect_btn)
        gb.layout().addLayout(row2)
        gb.layout().addWidget(self.replay_btn)
        self.conn_label = QtWidgets.QLabel("未连接")
        self.conn_label.setObjectName("hint")
        gb.layout().addWidget(self.conn_label)
        lay.addWidget(gb)

        # display
        gb = self._group("显示 / Display")
        gb.layout().addWidget(QtWidgets.QLabel("颜色"))
        self.color_combo = QtWidgets.QComboBox()
        for key, label in (("height", "高度 Z"), ("intensity", "反射强度"),
                           ("range", "距离"), ("azimuth", "方位角"), ("uniform", "单色")):
            self.color_combo.addItem(label, key)
        gb.layout().addWidget(self.color_combo)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("点大小"))
        self.size_spin = QtWidgets.QSpinBox()
        self.size_spin.setRange(1, 4)
        self.size_spin.setValue(2)
        row.addWidget(self.size_spin)
        gb.layout().addLayout(row)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("最大点数"))
        self.cap_spin = QtWidgets.QSpinBox()
        self.cap_spin.setRange(10_000, 2_000_000)
        self.cap_spin.setSingleStep(50_000)
        self.cap_spin.setValue(250_000)
        row.addWidget(self.cap_spin)
        gb.layout().addLayout(row)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("最大距离 m"))
        self.range_spin = QtWidgets.QDoubleSpinBox()
        self.range_spin.setRange(1.0, 200.0)
        self.range_spin.setValue(50.0)
        row.addWidget(self.range_spin)
        gb.layout().addLayout(row)

        row = QtWidgets.QHBoxLayout()
        self.clear_btn = QtWidgets.QPushButton("清空点云")
        self.reset_btn = QtWidgets.QPushButton("复位视角")
        row.addWidget(self.clear_btn)
        row.addWidget(self.reset_btn)
        gb.layout().addLayout(row)
        lay.addWidget(gb)

        # actions
        gb = self._group("操作 / Actions")
        self.save_ply_btn = QtWidgets.QPushButton("保存 PLY")
        self.save_png_btn = QtWidgets.QPushButton("保存截图 PNG")
        self.record_btn = QtWidgets.QPushButton("开始录制原始数据")
        self.record_btn.setCheckable(True)
        for b in (self.save_ply_btn, self.save_png_btn, self.record_btn):
            gb.layout().addWidget(b)
        lay.addWidget(gb)

        # lidar commands
        gb = self._group("雷达控制 / LiDAR")
        self.version_btn = QtWidgets.QPushButton("读取固件版本")
        gb.layout().addWidget(self.version_btn)
        row = QtWidgets.QHBoxLayout()
        self.normal_btn = QtWidgets.QPushButton("正常工作")
        self.standby_btn = QtWidgets.QPushButton("待机")
        row.addWidget(self.normal_btn)
        row.addWidget(self.standby_btn)
        gb.layout().addLayout(row)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("LED"))
        self.led_combo = QtWidgets.QComboBox()
        self.led_combo.addItem("(不改)", 0)
        for name, value in LED_MODES.items():
            self.led_combo.addItem(name, value)
        row.addWidget(self.led_combo, 1)
        self.led_btn = QtWidgets.QPushButton("应用")
        row.addWidget(self.led_btn)
        gb.layout().addLayout(row)
        lay.addWidget(gb)
        lay.addStretch(1)

        # wire up
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn.clicked.connect(self._toggle_connection)
        self.replay_btn.clicked.connect(self._open_replay)
        self.color_combo.currentIndexChanged.connect(
            lambda: self.view.set_color_mode(self.color_combo.currentData()))
        self.size_spin.valueChanged.connect(self.view.set_point_size)
        self.cap_spin.valueChanged.connect(self.view.set_capacity)
        self.range_spin.valueChanged.connect(self.view.set_max_range)
        self.clear_btn.clicked.connect(self.view.clear)
        self.reset_btn.clicked.connect(self.view.reset_view)
        self.save_ply_btn.clicked.connect(self._save_ply)
        self.save_png_btn.clicked.connect(self._save_png)
        self.record_btn.clicked.connect(self._toggle_record)
        self.version_btn.clicked.connect(self._request_version)
        self.normal_btn.clicked.connect(lambda: self._set_mode(WORKING_MODE_NORMAL))
        self.standby_btn.clicked.connect(lambda: self._set_mode(WORKING_MODE_STANDBY))
        self.led_btn.clicked.connect(self._set_led)
        return panel

    def _build_right_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        panel.setMinimumWidth(240)
        panel.setMaximumWidth(330)
        lay = QtWidgets.QVBoxLayout(panel)
        lay.setContentsMargins(8, 8, 8, 8)

        gb = self._group("固件 / Firmware")
        self.version_label = QtWidgets.QLabel("--")
        self.version_label.setWordWrap(True)
        self.version_label.setObjectName("value")
        gb.layout().addWidget(self.version_label)
        lay.addWidget(gb)

        gb = self._group("统计 / Statistics")
        form = QtWidgets.QFormLayout()
        self.stat_labels = {}
        for key, text in (("scan_rate", "扫描频率"), ("packet_rate", "数据包/s"),
                          ("point_rate", "点/s"), ("total_pts", "当前点数"),
                          ("imu_rate", "IMU Hz"), ("bytes_rate", "串口 kB/s"),
                          ("crc", "CRC 错误")):
            lab = QtWidgets.QLabel("--")
            lab.setObjectName("value")
            form.addRow(text, lab)
            self.stat_labels[key] = lab
        gb.layout().addLayout(form)
        lay.addWidget(gb)

        gb = self._group("IMU")
        form = QtWidgets.QFormLayout()
        self.imu_labels = {}
        for key, text in (("quat", "四元数"), ("euler", "欧拉角 °"),
                          ("gyro", "角速度"), ("accel", "加速度")):
            lab = QtWidgets.QLabel("--")
            lab.setObjectName("value")
            lab.setWordWrap(True)
            form.addRow(text, lab)
            self.imu_labels[key] = lab
        gb.layout().addLayout(form)
        lay.addWidget(gb)

        gb = self._group("扫描 / Scan")
        form = QtWidgets.QFormLayout()
        self.scan_labels = {}
        for key, text in (("work", "工作状态"), ("apd", "APD 温度"),
                          ("imu_t", "IMU 温度"), ("dirty", "脏污指数"),
                          ("com", "高速周期 us"), ("sys", "低速周期 us"),
                          ("hstep", "水平步进 °"), ("vspan", "垂直跨度 °"),
                          ("valid", "本帧有效点")):
            lab = QtWidgets.QLabel("--")
            lab.setObjectName("value")
            form.addRow(text, lab)
            self.scan_labels[key] = lab
        gb.layout().addLayout(form)
        lay.addWidget(gb)
        lay.addStretch(1)
        return panel

    # -- helpers -------------------------------------------------------
    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        ports = list_ports()
        self.port_combo.clear()
        for p in ports:
            self.port_combo.addItem(p)
        if current:
            idx = self.port_combo.findText(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
            elif self.port_combo.isEditable():
                self.port_combo.setEditText(current)
        if not ports:
            self.port_combo.setEditText(current or "COM8")

    def _log(self, text: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log.appendPlainText(f"[{ts}] {text}")

    def _baud(self) -> int:
        try:
            return int(self.baud_combo.currentText())
        except ValueError:
            return 2_000_000

    def _is_running(self) -> bool:
        return self.worker is not None and self.worker.isRunning()

    def _stop_worker(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(2000)
            self.worker = None
        self.connect_btn.setText("连接")
        self.conn_label.setText("未连接")
        self.statusBar().showMessage("未连接")

    def _start_worker(self, worker: LidarWorker) -> None:
        self._stop_worker()
        self._reset_panels()
        self.worker = worker
        worker.cloud.connect(self._on_cloud)
        worker.imu.connect(self._on_imu)
        worker.version.connect(self._on_version)
        worker.log.connect(self._log)
        worker.error.connect(self._on_error)
        worker.state.connect(self._on_state)
        worker.start()

    def _reset_panels(self) -> None:
        self.version_label.setText("--")
        for lab in self.imu_labels.values():
            lab.setText("--")
        for lab in self.scan_labels.values():
            lab.setText("--")
        for lab in self.stat_labels.values():
            lab.setText("--")
        self._last_stats = {}
        self._prev_scans = self._prev_imu = 0
        self._prev_bytes = self._prev_points = 0

    def _toggle_connection(self) -> None:
        if self._is_running() and self.worker and not self.worker.replay_path:
            self._stop_worker()
            return
        port = self.port_combo.currentText().strip()
        if not port:
            QtWidgets.QMessageBox.warning(self, "提示", "请选择串口")
            return
        self.view.clear()
        self._start_worker(LidarWorker(port, self._baud()))
        self.connect_btn.setText("断开")

    def _open_replay(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "选择原始数据文件", "", "Raw capture (*.bin);;All files (*)")
        if not path:
            return
        self.view.clear()
        self._start_worker(LidarWorker(replay_path=path))
        self.connect_btn.setText("连接")

    def _on_state(self, text: str) -> None:
        self.conn_label.setText(text)
        self.statusBar().showMessage(text)

    def _on_error(self, text: str) -> None:
        self._log("错误: " + text)
        self.statusBar().showMessage(text)
        self._stop_worker()
        self.view.set_note("连接中断")

    def _on_version(self, version) -> None:
        self.version_label.setText(str(version))

    def _on_cloud(self, pts, stats) -> None:
        if pts is not None and len(pts):
            self.view.add_points(pts)
        self._last_stats = stats
        self._update_scan_panel(stats)
        self.view.set_note(f"scan {stats.get('scan_rate', 0):.2f} Hz")

    def _on_imu(self, imu) -> None:
        r, p, y = imu.euler_deg()
        self.imu_labels["quat"].setText(
            "({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(*imu.quaternion))
        self.imu_labels["euler"].setText(f"R {r:6.1f}  P {p:6.1f}  Y {y:6.1f}")
        self.imu_labels["gyro"].setText("({:+.3f}, {:+.3f}, {:+.3f})".format(*imu.angular_velocity))
        self.imu_labels["accel"].setText("({:+.2f}, {:+.2f}, {:+.2f})".format(*imu.linear_acceleration))

    def _update_scan_panel(self, s: dict) -> None:
        self.scan_labels["work"].setText(str(s.get("work_status", "--")))
        self.scan_labels["apd"].setText(f"{s.get('apd_temp', 0):.1f} °C")
        self.scan_labels["imu_t"].setText(f"{s.get('imu_temp', 0):.1f} °C")
        self.scan_labels["dirty"].setText(f"{s.get('dirty', 0):.2f}")
        self.scan_labels["com"].setText(str(s.get("com_period", 0)))
        self.scan_labels["sys"].setText(str(s.get("sys_period", 0)))
        self.scan_labels["hstep"].setText(f"{s.get('h_step', 0):.3f}")
        self.scan_labels["vspan"].setText(f"{s.get('v_span', 0):.3f}")
        self.scan_labels["valid"].setText(str(s.get("valid_points", 0)))

    def _update_rates(self) -> None:
        s = self._last_stats
        scans = s.get("scans", 0)
        imu = s.get("imu_total", 0)
        nbytes = s.get("bytes", 0)
        points = s.get("points", 0)
        self._rate_scans = scans - self._prev_scans
        self._rate_imu = imu - self._prev_imu
        self._rate_bytes = nbytes - self._prev_bytes
        self._rate_points = points - self._prev_points
        self._prev_scans, self._prev_imu = scans, imu
        self._prev_bytes, self._prev_points = nbytes, points
        self.stat_labels["scan_rate"].setText(f"{s.get('scan_rate', 0):.2f} Hz")
        self.stat_labels["packet_rate"].setText(f"{self._rate_scans}")
        self.stat_labels["point_rate"].setText(f"{self._rate_points}")
        self.stat_labels["total_pts"].setText(f"{self.view.store.count}")
        self.stat_labels["imu_rate"].setText(f"{self._rate_imu}")
        self.stat_labels["bytes_rate"].setText(f"{self._rate_bytes / 1024:.1f}")
        self.stat_labels["crc"].setText(str(s.get("crc", 0)))

    # -- actions -------------------------------------------------------
    def _save_ply(self) -> None:
        pts = self.view.store.array()
        if not len(pts):
            QtWidgets.QMessageBox.information(self, "提示", "当前没有点云")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存点云", "scan.ply", "PLY (*.ply)")
        if not path:
            return
        colors = compute_colors(pts, self.view.color_mode, self.view.max_range)
        save_ply(path, pts, colors)
        self._log(f"已保存 {len(pts)} 点 -> {path}")

    def _save_png(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存截图", "snapshot.png", "PNG (*.png)")
        if path and self.view.save_image(path):
            self._log(f"截图已保存 -> {path}")

    def _toggle_record(self, checked: bool) -> None:
        if self.worker is None or not self.worker.isRunning():
            QtWidgets.QMessageBox.information(self, "提示", "请先连接或回放")
            self.record_btn.setChecked(False)
            return
        if self.worker.replay_path:
            QtWidgets.QMessageBox.information(self, "提示", "回放模式无需录制")
            self.record_btn.setChecked(False)
            return
        if checked:
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "保存原始数据", "capture.bin", "Raw (*.bin)")
            if not path:
                self.record_btn.setChecked(False)
                return
            self.worker.start_record(path)
            self.record_btn.setText("停止录制")
        else:
            self.worker.stop_record()
            self.record_btn.setText("开始录制原始数据")

    def _request_version(self) -> None:
        if self.worker is not None:
            self.worker.request_version()

    def _set_mode(self, mode: int) -> None:
        if self.worker is not None:
            self.worker.set_working_mode(mode)
            self._log("已发送工作模式命令")

    def _set_led(self) -> None:
        if self.worker is None:
            return
        mode = self.led_combo.currentData()
        if mode:
            self.worker.set_led_mode(int(mode))
            self._log(f"已发送 LED 模式 {mode}")

    def closeEvent(self, e: QtGui.QCloseEvent) -> None:
        self._stop_worker()
        super().closeEvent(e)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_TITLE)
    font = QtGui.QFont("Microsoft YaHei UI", 9)
    app.setFont(font)
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
