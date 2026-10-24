"""Unitree L2 LiDAR protocol library.

Public API::

    from unilidar_l2 import UnitreeL2, ScanEvent, ImuEvent, Calibration
    from unilidar_l2 import protocol, geometry, serial_io
"""

from .geometry import Calibration, save_ply, scan_to_points
from .lidar import (Event, ImuEvent, ScanEvent, TimeSyncEvent, UnitreeL2,
                    VersionEvent, capture, collect_points)
from .protocol import (AuxPacket, DistancePacket, Frame, ImuPacket, MsgId,
                       MavlinkParser, TimeSyncPacket, VersionInfo,
                       encode_message, x25)
from .serial_io import SerialException, SerialPort, list_ports

__version__ = "1.0.0"

__all__ = [
    "UnitreeL2", "Calibration", "ScanEvent", "ImuEvent", "VersionEvent",
    "TimeSyncEvent", "Event", "capture", "collect_points", "save_ply",
    "scan_to_points", "MsgId", "MavlinkParser", "Frame", "DistancePacket",
    "AuxPacket", "ImuPacket", "VersionInfo", "TimeSyncPacket",
    "encode_message", "x25", "SerialPort", "SerialException", "list_ports",
]
