"""Unitree L2 LiDAR wire protocol (MAVLink v2).

Protocol recovered by black-box analysis of the UART stream from a
Unitree L2 LiDAR (USB CDC, 2,000,000 8N1).

Framing is MAVLink v2 (0xFD) with the SysMavlink dialect used by the
official unilidar_sdk.  Only the pieces needed to talk to the LiDAR are
implemented here, without depending on a generated MAVLink library.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Iterator, List, Optional, Tuple

STX_V1 = 0xFE
STX_V2 = 0xFD

MAVLINK_SYS_ID = 1
MAVLINK_COMP_ID = 1


class MsgId(IntEnum):
    DEVICE_REQUEST_DATA = 11
    DEVICE_COMMAND = 12
    RET_LIDAR_VERSION = 13
    CONFIG_LIDAR_WORKING_MODE = 14
    CONFIG_LED_RING_TABLE_PACKET = 15
    RET_LIDAR_DISTANCE_DATA_PACKET = 16
    RET_LIDAR_AUXILIARY_DATA_PACKET = 17
    RET_LIDAR_TIME_SYNC_DATA = 18
    RET_IMU_ATTITUDE_DATA_PACKET = 19


@dataclass(frozen=True)
class MessageDef:
    msgid: int
    name: str
    payload_len: int
    crc_extra: int


MESSAGE_DEFS: Dict[int, MessageDef] = {
    MsgId.DEVICE_REQUEST_DATA: MessageDef(11, "DEVICE_REQUEST_DATA", 1, 106),
    MsgId.DEVICE_COMMAND: MessageDef(12, "DEVICE_COMMAND", 1, 204),
    MsgId.RET_LIDAR_VERSION: MessageDef(13, "RET_LIDAR_VERSION", 38, 62),
    MsgId.CONFIG_LIDAR_WORKING_MODE: MessageDef(14, "CONFIG_LIDAR_WORKING_MODE", 1, 84),
    MsgId.CONFIG_LED_RING_TABLE_PACKET: MessageDef(15, "CONFIG_LED_RING_TABLE", 56, 236),
    MsgId.RET_LIDAR_DISTANCE_DATA_PACKET: MessageDef(16, "RET_LIDAR_DISTANCE_DATA_PACKET", 246, 74),
    MsgId.RET_LIDAR_AUXILIARY_DATA_PACKET: MessageDef(17, "RET_LIDAR_AUXILIARY_DATA_PACKET", 209, 99),
    MsgId.RET_LIDAR_TIME_SYNC_DATA: MessageDef(18, "RET_LIDAR_TIME_SYNC_DATA", 5, 12),
    MsgId.RET_IMU_ATTITUDE_DATA_PACKET: MessageDef(19, "RET_IMU_ATTITUDE_DATA_PACKET", 42, 110),
}


# --------------------------------------------------------------------------
# X25 / CRC-16 (MAVLink flavour)
# --------------------------------------------------------------------------

def _build_crc_table() -> Tuple[int, ...]:
    table = []
    for byte in range(256):
        tmp = byte
        for _ in range(8):
            tmp = (tmp >> 1) ^ 0x8408 if (tmp & 1) else (tmp >> 1)
        table.append(tmp)
    return tuple(table)


_CRC_TABLE = _build_crc_table()


def x25(data, crc: int = 0xFFFF) -> int:
    """MAVLink CRC-16/X25 over an iterable of byte values."""
    tbl = _CRC_TABLE
    for b in data:
        crc = (crc >> 8) ^ tbl[(crc ^ b) & 0xFF]
    return crc


def crc_calc(payload: bytes, msgid: int, header_from_len: bytes) -> int:
    """CRC over the frame body (starting at the LEN byte) plus CRC_EXTRA."""
    extra = MESSAGE_DEFS[msgid].crc_extra
    return x25(bytes(header_from_len) + bytes(payload) + bytes((extra,)))


# --------------------------------------------------------------------------
# Frames
# --------------------------------------------------------------------------

@dataclass
class Frame:
    msgid: int
    payload: bytes          # zero padded to the canonical length
    seq: int
    sysid: int
    compid: int
    incompat_flags: int = 0
    compat_flags: int = 0
    wire_len: int = 0       # payload length on the wire (before padding)
    offset: int = -1        # byte offset inside the fed chunk stream (debug)

    @property
    def name(self) -> str:
        d = MESSAGE_DEFS.get(self.msgid)
        return d.name if d else f"UNKNOWN({self.msgid})"


@dataclass
class ParserStats:
    frames: int = 0
    crc_errors: int = 0
    unknown_msgid: int = 0
    bytes_in: int = 0
    resync_bytes: int = 0
    synced: bool = False

    def as_dict(self) -> dict:
        return {
            "frames": self.frames,
            "crc_errors": self.crc_errors,
            "unknown_msgid": self.unknown_msgid,
            "bytes_in": self.bytes_in,
        }


class MavlinkParser:
    """Incremental MAVLink v1/v2 parser.

    Feed arbitrary chunks; yields fully validated Frame objects.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._in_error = False
        self.stats = ParserStats()

    def feed(self, data: bytes) -> List[Frame]:
        if not data:
            return []
        self.stats.bytes_in += len(data)
        self._buf += data
        return list(self._iter_frames())

    def reset(self) -> None:
        self._buf.clear()
        self._in_error = False

    def _iter_frames(self) -> Iterator[Frame]:
        buf = self._buf
        i = 0
        n = len(buf)
        frames: List[Frame] = []

        while i < n:
            stx = buf[i]
            if stx == STX_V2:
                if n - i < 12:
                    break
                plen = buf[i + 1]
                total = 12 + plen
                if n - i < total:
                    break
                msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
                mdef = MESSAGE_DEFS.get(msgid)
                if mdef is None:
                    self.stats.unknown_msgid += 1
                    self.stats.resync_bytes += 1
                    i += 1
                    continue
                crc_rx = buf[i + 10 + plen] | (buf[i + 11 + plen] << 8)
                crc = x25(buf[i + 1:i + 10 + plen] + bytes((mdef.crc_extra,)))
                if crc == crc_rx:
                    payload = bytes(buf[i + 10:i + 10 + plen])
                    if len(payload) < mdef.payload_len:
                        payload = payload.ljust(mdef.payload_len, b"\x00")
                    else:
                        payload = payload[:mdef.payload_len]
                    frames.append(Frame(
                        msgid=msgid, payload=payload, seq=buf[i + 4],
                        sysid=buf[i + 5], compid=buf[i + 6],
                        incompat_flags=buf[i + 2], compat_flags=buf[i + 3],
                        wire_len=plen, offset=i,
                    ))
                    self.stats.frames += 1
                    self.stats.synced = True
                    self._in_error = False
                    i += total
                    continue
                if self.stats.synced and not self._in_error:
                    self.stats.crc_errors += 1
                    self._in_error = True
                self.stats.resync_bytes += 1
                i += 1
                continue

            if stx == STX_V1:
                if n - i < 8:
                    break
                plen = buf[i + 1]
                total = 8 + plen
                if n - i < total:
                    break
                msgid = buf[i + 5]
                mdef = MESSAGE_DEFS.get(msgid)
                if mdef is None:
                    self.stats.unknown_msgid += 1
                    i += 1
                    continue
                crc_rx = buf[i + 6 + plen] | (buf[i + 7 + plen] << 8)
                crc = x25(buf[i + 1:i + 6 + plen] + bytes((mdef.crc_extra,)))
                if crc == crc_rx:
                    payload = bytes(buf[i + 6:i + 6 + plen])
                    if len(payload) < mdef.payload_len:
                        payload = payload.ljust(mdef.payload_len, b"\x00")
                    payload = payload[:mdef.payload_len]
                    frames.append(Frame(
                        msgid=msgid, payload=payload, seq=buf[i + 2],
                        sysid=buf[i + 3], compid=buf[i + 4], wire_len=plen,
                        offset=i,
                    ))
                    self.stats.frames += 1
                    self.stats.synced = True
                    self._in_error = False
                    i += total
                    continue
                if self.stats.synced and not self._in_error:
                    self.stats.crc_errors += 1
                    self._in_error = True
                i += 1
                continue

            self.stats.resync_bytes += 1
            i += 1

        del buf[:i]
        for f in frames:
            yield f


def encode_message(msgid: int, payload: bytes, seq: int,
                   sysid: int = MAVLINK_SYS_ID,
                   compid: int = MAVLINK_COMP_ID) -> bytes:
    """Encode a MAVLink v2 frame."""
    payload = bytes(payload)
    plen = len(payload)
    header = bytes((
        plen, 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF,
        msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF,
    ))
    crc = x25(header + payload + bytes((MESSAGE_DEFS[msgid].crc_extra,)))
    return bytes((STX_V2,)) + header + payload + struct.pack("<H", crc)


# --------------------------------------------------------------------------
# Payload decoders
# --------------------------------------------------------------------------

@dataclass
class DistancePacket:
    packet_id: int
    packet_cnt: int
    payload_size: int
    ranges: Tuple[int, ...]      # 120 values, millimetres, 0 == invalid
    raw: bytes

    def valid_ranges_m(self, scale: float = 0.001):
        return [r * scale for r in self.ranges if r]


@dataclass
class AuxPacket:
    lidar_sync_delay_time: int
    time_stamp_s: int
    time_stamp_us: int
    sys_rotation_period: int
    com_rotation_period: int
    com_horizontal_angle_start: float
    com_horizontal_angle_step: float
    sys_vertical_angle_start: float
    sys_vertical_angle_span: float
    apd_temperature: float
    dirty_index: float
    imu_temperature: float
    up_optical_q: float
    down_optical_q: float
    apd_voltage: float
    imu_angle_x_offset: float
    imu_angle_y_offset: float
    imu_angle_z_offset: float
    b_axis_dist: float
    theta_angle: float
    ksi_angle: float
    packet_id: int
    payload_size: int
    lidar_work_status: int
    reflect_data: bytes
    raw: bytes

    @property
    def timestamp(self) -> float:
        return self.time_stamp_s + self.time_stamp_us * 1e-6

    @property
    def com_rotation_period_s(self) -> float:
        return self.com_rotation_period * 1e-6

    @property
    def sys_rotation_period_s(self) -> float:
        return self.sys_rotation_period * 1e-6

    @property
    def scan_rate_hz(self) -> float:
        p = self.com_rotation_period_s
        return (1.0 / p) if p > 0 else 0.0


@dataclass
class ImuPacket:
    quaternion: Tuple[float, float, float, float]   # x, y, z, w
    angular_velocity: Tuple[float, float, float]    # rad/s
    linear_acceleration: Tuple[float, float, float]  # m/s^2
    packet_id: int

    @property
    def quaternion_norm(self) -> float:
        x, y, z, w = self.quaternion
        return (x * x + y * y + z * z + w * w) ** 0.5

    def euler_deg(self) -> Tuple[float, float, float]:
        """Roll, pitch, yaw in degrees from the quaternion."""
        import math
        x, y, z, w = self.quaternion
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


@dataclass
class VersionInfo:
    sys_soft_version: str
    com_soft_version: str
    sys_hard_version: str
    com_hard_version: str

    def __str__(self) -> str:
        return (f"SYS {self.sys_soft_version} / COM {self.com_soft_version}  "
                f"(HW SYS {self.sys_hard_version}, COM {self.com_hard_version})")


@dataclass
class TimeSyncPacket:
    lidar_sync_count: int
    lidar_sync_type: int


def _ascii(raw: bytes) -> str:
    return raw.split(b"\x00")[0].decode("ascii", "replace")


def decode_distance(payload: bytes) -> DistancePacket:
    packet_id, packet_cnt, payload_size = struct.unpack_from("<HHH", payload, 0)
    ranges = struct.unpack_from("<120H", payload, 6)
    return DistancePacket(packet_id, packet_cnt, payload_size, ranges, payload)


def decode_auxiliary(payload: bytes) -> AuxPacket:
    vals = struct.unpack_from("<5I16fHHB", payload, 0)
    reflect = bytes(payload[89:209])
    return AuxPacket(
        lidar_sync_delay_time=vals[0],
        time_stamp_s=vals[1],
        time_stamp_us=vals[2],
        sys_rotation_period=vals[3],
        com_rotation_period=vals[4],
        com_horizontal_angle_start=vals[5],
        com_horizontal_angle_step=vals[6],
        sys_vertical_angle_start=vals[7],
        sys_vertical_angle_span=vals[8],
        apd_temperature=vals[9],
        dirty_index=vals[10],
        imu_temperature=vals[11],
        up_optical_q=vals[12],
        down_optical_q=vals[13],
        apd_voltage=vals[14],
        imu_angle_x_offset=vals[15],
        imu_angle_y_offset=vals[16],
        imu_angle_z_offset=vals[17],
        b_axis_dist=vals[18],
        theta_angle=vals[19],
        ksi_angle=vals[20],
        packet_id=vals[21],
        payload_size=vals[22],
        lidar_work_status=vals[23],
        reflect_data=reflect,
        raw=payload,
    )


def decode_imu(payload: bytes) -> ImuPacket:
    vals = struct.unpack_from("<10fH", payload, 0)
    return ImuPacket(tuple(vals[0:4]), tuple(vals[4:7]), tuple(vals[7:10]), vals[10])


def decode_version(payload: bytes) -> VersionInfo:
    return VersionInfo(
        sys_soft_version=_ascii(payload[0:13]),
        com_soft_version=_ascii(payload[13:26]),
        sys_hard_version=_ascii(payload[26:32]),
        com_hard_version=_ascii(payload[32:38]),
    )


def decode_time_sync(payload: bytes) -> TimeSyncPacket:
    count, sync_type = struct.unpack_from("<IB", payload, 0)
    return TimeSyncPacket(count, sync_type)


# --------------------------------------------------------------------------
# Outgoing commands
# --------------------------------------------------------------------------

ENUM_REQUEST_VERSION = 2
ENUM_REQUEST_TIME_SYNC = 6

WORKING_MODE_NORMAL = 1
WORKING_MODE_STANDBY = 2

LED_MODES = {
    "FORWARD_SLOW": 2,
    "FORWARD_FAST": 3,
    "REVERSE_SLOW": 4,
    "REVERSE_FAST": 5,
    "TRIPLE_FLIP": 6,
    "TRIPLE_BREATHING": 7,
    "SIXSTAGE_BREATHING": 8,
}


def encode_request_data(request_type: int, seq: int) -> bytes:
    return encode_message(MsgId.DEVICE_REQUEST_DATA, bytes((request_type & 0xFF,)), seq)


def encode_device_command(cmd_type: int, seq: int) -> bytes:
    return encode_message(MsgId.DEVICE_COMMAND, bytes((cmd_type & 0xFF,)), seq)


def encode_working_mode(mode: int, seq: int) -> bytes:
    return encode_message(MsgId.CONFIG_LIDAR_WORKING_MODE, bytes((mode & 0xFF,)), seq)


def encode_led_table(led_display_mode: int, led_table: bytes,
                     rotation_period_ms: int = 0, zero_point_offset_ms: int = 0,
                     packet_id: int = 0, seq: int = 0) -> bytes:
    table = bytes(led_table)[:45].ljust(45, b"\x00")
    payload = struct.pack("<IIH B", rotation_period_ms, zero_point_offset_ms,
                          packet_id & 0xFFFF, led_display_mode & 0xFF) + table
    return encode_message(MsgId.CONFIG_LED_RING_TABLE_PACKET, payload, seq)


def encode_led_mode(led_display_mode: int, seq: int = 0) -> bytes:
    return encode_led_table(led_display_mode, b"\x00" * 45, 0, 0, 0, seq)
