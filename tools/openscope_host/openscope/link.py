"""Transport: find the OpenScope CDC port, open it, survive it going away.

The only layer that knows about USB (remote_protocol.md §3.1). Design rules
from §2.4: "no device" is a normal outcome, device paths are not stable,
the port may vanish (IAP flash, reboot, or the firmware's CDC self-heal from
issue #39 — which the host sees as a replug) and must be reopened.
"""
from __future__ import annotations

import glob
import time
from typing import List, Optional

AT32_VCP = (0x2E3C, 0x5740)    # Artery virtual COM port (the app's CDC shell)
FALLBACK_GLOBS = ("/dev/cu.usbmodem*", "/dev/ttyACM*")


class FlushFailed(OSError):
    """The write returned but draining failed (device vanished). Treat as
    'possibly delivered': never re-send something that can act twice."""


class NoDevice(Exception):
    """No OpenScope port found. The message says where we looked."""


def candidate_ports() -> List[str]:
    """OpenScope ports by VID:PID; falls back to the usual globs if pyserial's
    port listing is unavailable. Other USB serial devices are ignored."""
    try:
        from serial.tools import list_ports
        ports = [p for p in list_ports.comports() if (p.vid, p.pid) == AT32_VCP]
        cu = sorted(p.device for p in ports if p.device.startswith("/dev/cu."))
        return cu or sorted(p.device for p in ports)
    except Exception:
        hits: List[str] = []
        for g in FALLBACK_GLOBS:
            hits += sorted(glob.glob(g))
        return hits


def open_error_words(e: BaseException) -> str:
    """The reason an open failed, in words. On Windows pyserial wraps the OS
    error in SerialException: "could not open port 'COM7': PermissionError(13,
    'Access is denied.', ...)" when another program holds the port, and
    "FileNotFoundError(2, ...)" when the port does not exist. POSIX says
    "busy" or (exclusive=True) "could not exclusively lock port"."""
    text = str(e)
    low = text.lower()
    if (isinstance(e, PermissionError) or "busy" in low or "lock" in low
            or "access is denied" in low or "permissionerror" in low):
        return "in use by another process (only one client at a time)"
    if (isinstance(e, FileNotFoundError) or "filenotfounderror" in low
            or "cannot find the file" in low or "no such file" in low):
        return "port not found (unplugged, or a wrong port name)"
    return text


def looked_where() -> str:
    return f"USB {AT32_VCP[0]:04x}:{AT32_VCP[1]:04x} (AT32 VCP)"


class SerialLink:
    """Byte pipe over pyserial with reopen-on-loss."""

    def __init__(self, port: Optional[str] = None, reopen_wait: float = 6.0,
                 drain_timeout: float = 2.0):
        self.requested = port
        self.drain_timeout = drain_timeout
        self.port: Optional[str] = None
        self.reopen_wait = reopen_wait
        self._ser = None
        self.reopens = 0

    def open(self) -> None:
        import serial  # imported late so `openscope --help` works without pyserial
        port = self.requested
        if port is None:
            ports = candidate_ports()
            if not ports:
                raise NoDevice(f"no OpenScope found on {looked_where()}")
            port = ports[0]
        try:
            # exclusive: the protocol has no request ids, so a second process on
            # the same port would steal (and drain away) this one's replies.
            self._ser = serial.Serial(port, 115200, timeout=0.05, write_timeout=2.0,
                                      exclusive=True)
        except (OSError, serial.SerialException) as e:
            raise NoDevice(f"cannot open {port}: {open_error_words(e)}") from None
        self.port = port

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def reopen(self) -> None:
        """After the port vanished: wait for it to come back (any path).
        reopen_wait <= 0: do not wait; raise NoDevice("port lost") at once
        (the GUI worker owns reconnect, so a request never blocks)."""
        self.close()
        if self.reopen_wait <= 0:
            raise NoDevice("port lost")
        deadline = time.monotonic() + self.reopen_wait
        last: Exception = NoDevice("not found")
        while time.monotonic() < deadline:
            try:
                self.open()
                self.reopens += 1
                return
            except NoDevice as e:
                last = e
                time.sleep(0.2)
        raise NoDevice(f"device did not come back within {self.reopen_wait:.0f} s ({last})")

    def write(self, data: bytes) -> None:
        """OSError = nothing may have been sent; FlushFailed = the bytes were
        handed to the OS and may well have reached the device."""
        self._ser.write(data)
        # Not ser.flush(): on POSIX that is an unbounded tcdrain, and the device
        # only re-arms its OUT endpoint when the shell task polls, so a busy or
        # wedged device would hang the host with no timeout at all. Wait for
        # the OS queue to empty, but only up to a deadline.
        deadline = time.monotonic() + self.drain_timeout
        try:
            while self._ser.out_waiting:
                if time.monotonic() > deadline:
                    raise FlushFailed(f"device not accepting data after {self.drain_timeout:.0f} s")
                time.sleep(0.005)
        except FlushFailed:
            raise
        except Exception as e:      # port vanished while draining (termios/serial errors)
            raise FlushFailed(f"port lost after write: {e}") from e

    def discard_input(self) -> None:
        """Drop the bytes the OS holds for us (a late reply, old text). Never
        blocks; bytes still on the wire are not affected."""
        self._ser.reset_input_buffer()

    def read(self, n: int = 4096) -> bytes:
        return self._ser.read(max(1, min(n, self._ser.in_waiting or 1)))

    def drain(self, quiet: float = 0.15, max_wait: float = 1.5) -> bytes:
        """Swallow whatever the device is still saying (banner, old output)."""
        out = bytearray()
        start = last = time.monotonic()
        while time.monotonic() - start < max_wait:
            chunk = self.read()
            if chunk:
                out += chunk
                last = time.monotonic()
            elif time.monotonic() - last >= quiet:
                break
        return bytes(out)
