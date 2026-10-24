"""Minimal dependency-free serial port (Win32 ctypes / POSIX termios).

pyserial is intentionally not required so the library runs on a bare
Python install.  Only the subset needed by the LiDAR is implemented.
"""

from __future__ import annotations

import glob
import os
import sys
import time
from typing import List

__all__ = ["SerialException", "SerialPort", "list_ports"]


class SerialException(OSError):
    pass


def list_ports() -> List[str]:
    """Return available serial port names (best effort)."""
    if sys.platform.startswith("win"):
        try:
            import winreg
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                                 r"HARDWARE\DEVICEMAP\SERIALCOMM")
            ports = []
            i = 0
            while True:
                try:
                    _, value, _ = winreg.EnumValue(key, i)
                except OSError:
                    break
                ports.append(str(value))
                i += 1
            winreg.CloseKey(key)
            return sorted(set(ports))
        except Exception:
            return []
    patterns = ["/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyS*", "/dev/cu.usbserial*",
                "/dev/cu.usbmodem*"]
    ports = []
    for pat in patterns:
        ports.extend(glob.glob(pat))
    return sorted(set(ports))


if sys.platform.startswith("win"):
    import ctypes
    from ctypes import wintypes

    _kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

    GENERIC_READ = 0x80000000
    GENERIC_WRITE = 0x40000000
    OPEN_EXISTING = 3
    INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value
    PURGE_TXABORT = 0x0001
    PURGE_RXABORT = 0x0002
    PURGE_TXCLEAR = 0x0004
    PURGE_RXCLEAR = 0x0008

    class _DCB(ctypes.Structure):
        _fields_ = [
            ("DCBlength", wintypes.DWORD),
            ("BaudRate", wintypes.DWORD),
            ("fFlags", wintypes.DWORD),
            ("wReserved", wintypes.WORD),
            ("XonLim", wintypes.WORD),
            ("XoffLim", wintypes.WORD),
            ("ByteSize", wintypes.BYTE),
            ("Parity", wintypes.BYTE),
            ("StopBits", wintypes.BYTE),
            ("XonChar", ctypes.c_char),
            ("XoffChar", ctypes.c_char),
            ("ErrorChar", ctypes.c_char),
            ("EofChar", ctypes.c_char),
            ("EvtChar", ctypes.c_char),
            ("wReserved1", wintypes.WORD),
        ]

    class _COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout", wintypes.DWORD),
            ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
            ("ReadTotalTimeoutConstant", wintypes.DWORD),
            ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
            ("WriteTotalTimeoutConstant", wintypes.DWORD),
        ]

    _kernel32.CreateFileW.restype = wintypes.HANDLE
    _kernel32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                                      ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD,
                                      wintypes.HANDLE]
    _kernel32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(_DCB)]
    _kernel32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(_DCB)]
    _kernel32.SetCommTimeouts.argtypes = [wintypes.HANDLE, ctypes.POINTER(_COMMTIMEOUTS)]
    _kernel32.SetupComm.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.DWORD]
    _kernel32.PurgeComm.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    _kernel32.ReadFile.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD,
                                   ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
    _kernel32.WriteFile.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD,
                                    ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
    _kernel32.CloseHandle.argtypes = [wintypes.HANDLE]

    def _win_error(prefix: str) -> SerialException:
        err = ctypes.get_last_error()
        try:
            msg = ctypes.FormatError(err).strip()
        except Exception:
            msg = str(err)
        return SerialException(f"{prefix}: [{err}] {msg}")


class SerialPort:
    """Blocking serial port with a short read timeout."""

    def __init__(self, port: str, baudrate: int = 2_000_000,
                 bytesize: int = 8, parity: str = "N", stopbits: int = 1) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self._handle = None
        self._fd = None
        self._is_open = False

    # -- properties ---------------------------------------------------
    @property
    def is_open(self) -> bool:
        return self._is_open

    # -- open / close -------------------------------------------------
    def open(self) -> "SerialPort":
        if self._is_open:
            return self
        if sys.platform.startswith("win"):
            self._open_win()
        else:
            self._open_posix()
        self._is_open = True
        return self

    def close(self) -> None:
        if not self._is_open:
            return
        try:
            if sys.platform.startswith("win"):
                _kernel32.CloseHandle(self._handle)
                self._handle = None
            else:
                os.close(self._fd)
                self._fd = None
        finally:
            self._is_open = False

    def __enter__(self) -> "SerialPort":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- Windows backend ---------------------------------------------
    def _open_win(self) -> None:
        path = self.port
        if not path.startswith("\\\\.\\"):
            path = "\\\\.\\" + path
        handle = _kernel32.CreateFileW(path, GENERIC_READ | GENERIC_WRITE, 0, None,
                                       OPEN_EXISTING, 0, None)
        if handle == INVALID_HANDLE_VALUE or handle is None:
            raise _win_error(f"Cannot open {self.port}")
        self._handle = handle
        try:
            _kernel32.SetupComm(handle, 1 << 20, 1 << 20)
            dcb = _DCB()
            dcb.DCBlength = ctypes.sizeof(_DCB)
            if not _kernel32.GetCommState(handle, ctypes.byref(dcb)):
                raise _win_error("GetCommState failed")
            dcb.BaudRate = self.baudrate
            dcb.ByteSize = self.bytesize
            dcb.Parity = {"N": 0, "O": 1, "E": 2, "M": 3, "S": 4}.get(self.parity.upper(), 0)
            dcb.StopBits = {1: 0, 2: 2}.get(self.stopbits, 0)  # 0=1 stop, 2=2 stops
            # fBinary=1, DTR_CONTROL_ENABLE=1 (bits 4-5), RTS_CONTROL_ENABLE=1 (bits 12-13)
            dcb.fFlags = 0x0001 | (1 << 4) | (1 << 12)
            if not _kernel32.SetCommState(handle, ctypes.byref(dcb)):
                raise _win_error("SetCommState failed")
            tmo = _COMMTIMEOUTS(
                0xFFFFFFFF, 0xFFFFFFFF, 0,   # read: return immediately with buffered data
                1000, 2000,                  # write timeouts (ms)
            )
            if not _kernel32.SetCommTimeouts(handle, ctypes.byref(tmo)):
                raise _win_error("SetCommTimeouts failed")
            self.purge()
        except Exception:
            _kernel32.CloseHandle(handle)
            self._handle = None
            raise

    # -- POSIX backend ------------------------------------------------
    def _open_posix(self) -> None:
        import termios
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0  # iflag
            attrs[1] = 0  # oflag
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
            attrs[3] = 0  # lflag (raw)
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            speed = getattr(termios, f"B{self.baudrate}", None)
            if speed is None:
                raise SerialException(f"Baudrate {self.baudrate} not supported")
            termios.cfsetispeed(attrs, speed)
            termios.cfsetospeed(attrs, speed)
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception:
            os.close(fd)
            raise
        self._fd = fd

    # -- I/O ----------------------------------------------------------
    def purge(self) -> None:
        if not self._is_open and self._handle is None and self._fd is None:
            return
        if sys.platform.startswith("win"):
            if self._handle:
                _kernel32.PurgeComm(self._handle,
                                    PURGE_RXABORT | PURGE_TXABORT
                                    | PURGE_RXCLEAR | PURGE_TXCLEAR)
        else:
            if self._fd is not None:
                import termios
                termios.tcflush(self._fd, termios.TCIOFLUSH)

    def read(self, size: int = 4096, timeout: float = 0.05) -> bytes:
        """Read up to `size` bytes, waiting up to `timeout` seconds."""
        if not self._is_open:
            raise SerialException("port is not open")
        deadline = time.monotonic() + max(0.0, timeout)
        if sys.platform.startswith("win"):
            buf = ctypes.create_string_buffer(size)
            got = wintypes.DWORD(0)
            while True:
                ok = _kernel32.ReadFile(self._handle, buf, size,
                                        ctypes.byref(got), None)
                if not ok:
                    raise _win_error("ReadFile failed")
                if got.value:
                    return buf.raw[:got.value]
                if time.monotonic() >= deadline:
                    return b""
                time.sleep(0.001)
        else:
            import select
            while True:
                remaining = max(0.0, deadline - time.monotonic())
                r, _, _ = select.select([self._fd], [], [], remaining)
                if r:
                    data = os.read(self._fd, size)
                    if data:
                        return data
                if time.monotonic() >= deadline:
                    return b""

    def write(self, data: bytes) -> int:
        if not self._is_open:
            raise SerialException("port is not open")
        data = bytes(data)
        if sys.platform.startswith("win"):
            written = wintypes.DWORD(0)
            ok = _kernel32.WriteFile(self._handle, data, len(data),
                                     ctypes.byref(written), None)
            if not ok:
                raise _win_error("WriteFile failed")
            return int(written.value)
        os.write(self._fd, data)
        return len(data)

    def flush(self) -> None:
        """No-op keep-alive for API symmetry (writes are immediate)."""
