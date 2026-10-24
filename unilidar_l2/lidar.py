"""High level Unitree L2 LiDAR reader."""

from __future__ import annotations

import random
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Union

import numpy as np

from . import protocol as P
from .geometry import Calibration, scan_to_points
from .protocol import (AuxPacket, DistancePacket, ImuPacket, MsgId,
                       TimeSyncPacket, VersionInfo)
from .serial_io import SerialPort


@dataclass
class ScanEvent:
    packet_id: int
    points: np.ndarray          # (N, 4) float32: x, y, z, intensity
    aux: AuxPacket
    distance: DistancePacket

    @property
    def timestamp(self) -> float:
        return self.aux.timestamp


@dataclass
class ImuEvent:
    imu: ImuPacket

    @property
    def timestamp(self) -> float:
        return time.time()


@dataclass
class VersionEvent:
    version: VersionInfo


@dataclass
class TimeSyncEvent:
    packet: TimeSyncPacket


Event = Union[ScanEvent, ImuEvent, VersionEvent, TimeSyncEvent]


@dataclass
class ReaderStats:
    frames: int = 0
    scans: int = 0
    imu: int = 0
    crc_errors: int = 0
    dropped_pairs: int = 0
    bytes_in: int = 0
    started_at: float = field(default_factory=time.monotonic)
    last_scan_at: float = 0.0

    @property
    def elapsed(self) -> float:
        return max(1e-9, time.monotonic() - self.started_at)


class UnitreeL2:
    """Read scans / IMU and send commands to a Unitree L2 LiDAR.

    Typical use::

        lidar = UnitreeL2("COM8")
        lidar.open()
        for ev in lidar.iter_events():
            if isinstance(ev, ScanEvent):
                ...
    """

    def __init__(self, port: str = "COM8", baudrate: int = 2_000_000,
                 calibration: Optional[Calibration] = None,
                 sysid: int = 1, compid: int = 1) -> None:
        self.port_name = port
        self.baudrate = baudrate
        self.calibration = calibration or Calibration()
        self.sysid = sysid
        self.compid = compid
        self._port = SerialPort(port, baudrate)
        self._parser = P.MavlinkParser()
        self._pending_aux: "OrderedDict[int, AuxPacket]" = OrderedDict()
        self._seq = random.randrange(256)
        self._stats = ReaderStats()
        self.raw_callback: Optional[Callable[[bytes], None]] = None
        self.max_pending_aux = 16

    # -- lifecycle ----------------------------------------------------
    def open(self) -> "UnitreeL2":
        self._port.open()
        self._parser.reset()
        self._pending_aux.clear()
        self._stats = ReaderStats()
        return self

    def close(self) -> None:
        self._port.close()

    def __enter__(self) -> "UnitreeL2":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._port.is_open

    @property
    def stats(self) -> ReaderStats:
        self._stats.frames = self._parser.stats.frames
        self._stats.crc_errors = self._parser.stats.crc_errors
        self._stats.bytes_in = self._parser.stats.bytes_in
        return self._stats

    def reset_stats(self) -> None:
        self._stats = ReaderStats()

    # -- commands -----------------------------------------------------
    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return seq

    def _send(self, frame: bytes) -> None:
        self._port.write(frame)

    def request_version(self) -> None:
        self._send(P.encode_request_data(P.ENUM_REQUEST_VERSION, self._next_seq()))

    def request_time_sync(self) -> None:
        self._send(P.encode_request_data(P.ENUM_REQUEST_TIME_SYNC, self._next_seq()))

    def set_working_mode(self, mode: int) -> None:
        self._send(P.encode_working_mode(mode, self._next_seq()))

    def set_led_mode(self, led_display_mode: int) -> None:
        self._send(P.encode_led_mode(led_display_mode, self._next_seq()))

    def set_led_table(self, led_display_mode: int, led_table: bytes,
                      rotation_period_ms: int = 0, zero_point_offset_ms: int = 0,
                      packet_id: int = 0) -> None:
        self._send(P.encode_led_table(led_display_mode, led_table, rotation_period_ms,
                                      zero_point_offset_ms, packet_id, self._next_seq()))

    def send_device_command(self, cmd_type: int) -> None:
        self._send(P.encode_device_command(cmd_type, self._next_seq()))

    # -- reading ------------------------------------------------------
    def poll(self, timeout: float = 0.05) -> List[Event]:
        """Read once and return all decoded events.

        Blocking for at most `timeout` seconds.
        """
        data = self._port.read(65536, timeout)
        return self.process_bytes(data)

    def process_bytes(self, data: bytes) -> List[Event]:
        if not data:
            return []
        if self.raw_callback is not None:
            self.raw_callback(data)
        events: List[Event] = []
        for frame in self._parser.feed(data):
            ev = self._decode(frame)
            if ev is not None:
                events.append(ev)
        return events

    def _decode(self, frame: P.Frame) -> Optional[Event]:
        mid = frame.msgid
        try:
            if mid == MsgId.RET_LIDAR_AUXILIARY_DATA_PACKET:
                aux = P.decode_auxiliary(frame.payload)
                self._pending_aux[aux.packet_id] = aux
                while len(self._pending_aux) > self.max_pending_aux:
                    self._pending_aux.popitem(last=False)
                return None
            if mid == MsgId.RET_LIDAR_DISTANCE_DATA_PACKET:
                dist = P.decode_distance(frame.payload)
                aux = self._pending_aux.pop(dist.packet_id, None)
                if aux is None:
                    self._stats.dropped_pairs += 1
                    return None
                pts = scan_to_points(aux, dist, self.calibration)
                self._stats.scans += 1
                self._stats.last_scan_at = time.monotonic()
                return ScanEvent(dist.packet_id, pts, aux, dist)
            if mid == MsgId.RET_IMU_ATTITUDE_DATA_PACKET:
                self._stats.imu += 1
                return ImuEvent(P.decode_imu(frame.payload))
            if mid == MsgId.RET_LIDAR_VERSION:
                return VersionEvent(P.decode_version(frame.payload))
            if mid == MsgId.RET_LIDAR_TIME_SYNC_DATA:
                return TimeSyncEvent(P.decode_time_sync(frame.payload))
        except Exception:
            return None
        return None

    def iter_events(self, timeout: float = 0.05):
        while self.is_open:
            yield from self.poll(timeout)


# --------------------------------------------------------------------------
# Convenience helpers
# --------------------------------------------------------------------------

def capture(port: str = "COM8", seconds: float = 3.0, baudrate: int = 2_000_000,
            out: Optional[str] = None) -> bytes:
    """Capture raw bytes from the LiDAR (useful for offline replay)."""
    chunks = []
    with UnitreeL2(port, baudrate) as lidar:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            data = lidar._port.read(65536, 0.05)
            if data:
                chunks.append(data)
    raw = b"".join(chunks)
    if out:
        with open(out, "wb") as f:
            f.write(raw)
    return raw


def collect_points(port: str = "COM8", seconds: float = 3.0,
                   baudrate: int = 2_000_000,
                   calibration: Optional[Calibration] = None) -> np.ndarray:
    """Collect a point cloud for `seconds`."""
    chunks: List[np.ndarray] = []
    with UnitreeL2(port, baudrate, calibration) as lidar:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            for ev in lidar.poll(0.05):
                if isinstance(ev, ScanEvent):
                    chunks.append(ev.points)
    if not chunks:
        return np.zeros((0, 4), np.float32)
    return np.concatenate(chunks, axis=0)
