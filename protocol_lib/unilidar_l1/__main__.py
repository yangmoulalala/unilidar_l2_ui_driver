"""Command line tools for the Unitree L1 LiDAR.

Examples::

    python -m unilidar_l1 ports
    python -m unilidar_l1 info  --port COM8
    python -m unilidar_l1 ply   --port COM8 --seconds 5 -o scan.ply
    python -m unilidar_l1 dump  --port COM8 --seconds 5 -o capture.bin
    python -m unilidar_l1 replay capture.bin --info --ply scan.ply
"""

from __future__ import annotations

import argparse
import collections
import sys
import time
from typing import List

import numpy as np

from .geometry import save_ply
from .lidar import (ImuEvent, ScanEvent, TimeSyncEvent, UnitreeL1,
                    VersionEvent, collect_points)
from .protocol import MsgId
from .serial_io import list_ports


def cmd_ports(args) -> int:
    ports = list_ports()
    if not ports:
        print("no serial ports found")
        return 1
    for p in ports:
        print(p)
    return 0


def cmd_info(args) -> int:
    lidar = UnitreeL1(args.port, args.baud)
    lidar.open()
    print(f"opened {args.port} @ {args.baud}")
    lidar.request_version()
    lidar.request_time_sync()
    t0 = time.monotonic()
    counts = collections.Counter()
    version = None
    last_imu = None
    last_scan = None
    while time.monotonic() - t0 < args.seconds:
        for ev in lidar.poll(0.05):
            if isinstance(ev, ScanEvent):
                counts["scan"] += 1
                last_scan = ev
            elif isinstance(ev, ImuEvent):
                counts["imu"] += 1
                last_imu = ev
            elif isinstance(ev, VersionEvent):
                version = ev.version
            elif isinstance(ev, TimeSyncEvent):
                counts["timesync"] += 1
    st = lidar.stats
    el = max(1e-9, time.monotonic() - t0)
    print(f"frames            : {st.frames}")
    print(f"crc errors        : {st.crc_errors}")
    print(f"raw bytes         : {st.bytes_in} ({st.bytes_in/el/1024:.1f} KiB/s)")
    print(f"scan packets      : {counts['scan']} ({counts['scan']/el:.1f} /s)")
    print(f"imu packets       : {counts['imu']} ({counts['imu']/el:.1f} /s)")
    if version:
        print(f"firmware          : {version}")
    if last_imu:
        imu = last_imu.imu
        print(f"quaternion        : {tuple(round(v,4) for v in imu.quaternion)} "
              f"|q|={imu.quaternion_norm:.4f}")
        print(f"euler (deg)       : {tuple(round(v,2) for v in imu.euler_deg())}")
        print(f"gyro (rad/s)      : {tuple(round(v,4) for v in imu.angular_velocity)}")
        print(f"accel (m/s^2)     : {tuple(round(v,3) for v in imu.linear_acceleration)}")
    if last_scan:
        a = last_scan.aux
        print(f"scan rate         : {a.scan_rate_hz:.2f} Hz")
        print(f"point cloud       : {last_scan.points.shape[0]} valid of {a.payload_size}")
        print(f"apd/imu temp      : {a.apd_temperature:.1f} / {a.imu_temperature:.1f} C")
        print(f"dirty index       : {a.dirty_index:.2f}")
        print(f"work status       : {a.lidar_work_status}")
    lidar.close()
    return 0


def cmd_ply(args) -> int:
    pts = collect_points(args.port, args.seconds, args.baud)
    if len(pts) == 0:
        print("no points received", file=sys.stderr)
        return 1
    save_ply(args.out, pts)
    print(f"saved {len(pts)} points -> {args.out}")
    return 0


def cmd_dump(args) -> int:
    total = 0
    t0 = time.monotonic()
    with open(args.out, "wb") as f:
        with UnitreeL1(args.port, args.baud) as lidar:
            while time.monotonic() - t0 < args.seconds:
                data = lidar._port.read(65536, 0.05)
                if data:
                    f.write(data)
                    total += len(data)
    print(f"captured {total} bytes -> {args.out}")
    return 0


def _replay_events(raw: bytes):
    reader = UnitreeL1("", 2_000_000)
    return reader.process_bytes(raw)


def cmd_replay(args) -> int:
    with open(args.file, "rb") as f:
        raw = f.read()
    print(f"input: {args.file} ({len(raw)} bytes)")
    reader = UnitreeL1("", 2_000_000)
    counts = collections.Counter()
    chunks: List[np.ndarray] = []
    version = None
    first_imu = None
    first_scan = None
    for ev in reader.process_bytes(raw):
        if isinstance(ev, ScanEvent):
            counts["scan"] += 1
            chunks.append(ev.points)
            first_scan = first_scan or ev
        elif isinstance(ev, ImuEvent):
            counts["imu"] += 1
            first_imu = first_imu or ev
        elif isinstance(ev, VersionEvent):
            version = ev.version
        elif isinstance(ev, TimeSyncEvent):
            counts["timesync"] += 1
    st = reader.stats
    print(f"frames/scan/imu   : {st.frames} / {counts['scan']} / {counts['imu']}")
    print(f"crc errors        : {st.crc_errors}")
    if version:
        print(f"firmware          : {version}")
    if first_imu:
        imu = first_imu.imu
        print(f"imu quaternion    : {tuple(round(v,4) for v in imu.quaternion)}")
        print(f"imu accel         : {tuple(round(v,3) for v in imu.linear_acceleration)}")
    if first_scan:
        a = first_scan.aux
        print(f"scan rate         : {a.scan_rate_hz:.2f} Hz")
    if args.ply:
        if not chunks:
            print("no scans to save", file=sys.stderr)
            return 1
        pts = np.concatenate(chunks, axis=0)
        save_ply(args.ply, pts)
        print(f"saved {len(pts)} points -> {args.ply}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", default="COM8")
    common.add_argument("--baud", type=int, default=2_000_000)

    p = argparse.ArgumentParser(prog="unilidar_l1",
                                description="Unitree L1 LiDAR tools")
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("ports", help="list serial ports")
    sp.set_defaults(func=cmd_ports)

    sp = sub.add_parser("info", parents=[common],
                        help="read a few seconds and print device info")
    sp.add_argument("--seconds", type=float, default=2.0)
    sp.set_defaults(func=cmd_info)

    sp = sub.add_parser("ply", parents=[common],
                        help="capture a point cloud to PLY")
    sp.add_argument("--seconds", type=float, default=5.0)
    sp.add_argument("-o", "--out", default="scan.ply")
    sp.set_defaults(func=cmd_ply)

    sp = sub.add_parser("dump", parents=[common],
                        help="capture raw bytes for replay")
    sp.add_argument("--seconds", type=float, default=5.0)
    sp.add_argument("-o", "--out", default="capture.bin")
    sp.set_defaults(func=cmd_dump)

    sp = sub.add_parser("replay", help="decode a raw capture offline")
    sp.add_argument("file")
    sp.add_argument("--info", action="store_true")
    sp.add_argument("--ply", default=None)
    sp.set_defaults(func=cmd_replay)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
