"""link.py without hardware: the bounded drain that replaced ser.flush()."""
from __future__ import annotations

import os
import sys
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from openscope.link import FlushFailed, NoDevice, PortClosed, SerialLink  # noqa: E402


class FakeSer:
    def __init__(self, drains_after=None, raise_on_poll=None):
        self.writes = []
        self.polls = 0
        self.drains_after = drains_after
        self.raise_on_poll = raise_on_poll
        self.resets = 0

    def reset_input_buffer(self):
        self.resets += 1

    def close(self):
        pass

    def write(self, data):
        self.writes.append(data)

    @property
    def out_waiting(self):
        self.polls += 1
        if self.raise_on_poll:
            raise self.raise_on_poll
        return 0 if (self.drains_after is not None and self.polls > self.drains_after) else 5

    def flush(self):
        raise AssertionError("ser.flush() is an unbounded tcdrain and must not be used")


def link(ser, timeout=0.05):
    l = SerialLink(drain_timeout=timeout)
    l._ser = ser
    return l


class TestBoundedDrain(unittest.TestCase):
    def test_returns_once_the_os_queue_is_empty(self):
        ser = FakeSer(drains_after=3)
        link(ser).write(b"abc")
        self.assertEqual(ser.writes, [b"abc"])

    def test_a_device_that_never_accepts_data_is_bounded(self):
        with self.assertRaises(FlushFailed):
            link(FakeSer(drains_after=None)).write(b"abc")

    def test_termios_style_error_becomes_possibly_delivered(self):
        try:
            from termios import error as term_error     # POSIX: not an OSError subclass
        except ImportError:                              # Windows has no termios: same shape
            class term_error(Exception):
                pass
        with self.assertRaises(FlushFailed) as cm:
            link(FakeSer(raise_on_poll=term_error(6, "Device not configured"))).write(b"x")
        self.assertIsInstance(cm.exception, OSError)


class FakeSerialModule:
    """`import serial` stand-in whose Serial() raises what pyserial raises."""

    class SerialException(OSError):
        pass

    def __init__(self, exc):
        self.exc = exc

    def Serial(self, *a, **kw):
        raise self.exc


class TestOpenErrors(unittest.TestCase):
    def open_error(self, exc_text=None, exc=None):
        mod = FakeSerialModule(None)
        mod.exc = exc if exc is not None else mod.SerialException(exc_text)
        with mock.patch.dict(sys.modules, {"serial": mod}):
            with self.assertRaises(NoDevice) as cm:
                SerialLink("COM7").open()
        return str(cm.exception)

    def test_windows_access_denied_is_in_use(self):
        text = self.open_error("could not open port 'COM7': PermissionError(13, 'Access is denied.', None, 5)")
        self.assertEqual(text, "cannot open COM7: in use by another process (only one client at a time)")

    def test_permission_error_itself_is_in_use(self):
        self.assertIn("in use by another process", self.open_error(exc=PermissionError(13, "denied")))

    def test_posix_busy_and_lock_are_in_use(self):
        for t in ("[Errno 16] could not open port /dev/ttyACM0: [Errno 16] Device or resource busy",
                  "Could not exclusively lock port /dev/ttyACM0: [Errno 11]"):
            self.assertIn("in use by another process", self.open_error(t))

    def test_windows_not_found(self):
        text = self.open_error("could not open port 'COM99': FileNotFoundError(2, "
                               "'The system cannot find the file specified.', None, 2)")
        self.assertEqual(text, "cannot open COM7: port not found (unplugged, or a wrong port name)")

    def test_other_errors_keep_their_text(self):
        self.assertEqual(self.open_error("OSError(22, 'odd')"), "cannot open COM7: OSError(22, 'odd')")


class TestReopen(unittest.TestCase):
    def test_reopen_wait_zero_raises_at_once(self):
        l = SerialLink(reopen_wait=0)
        l._ser = FakeSer(drains_after=0)
        l.open = lambda: self.fail("reopen_wait=0 must not try to open")
        t0 = time.monotonic()
        with self.assertRaises(NoDevice) as cm:
            l.reopen()
        self.assertLess(time.monotonic() - t0, 0.05)
        self.assertEqual(str(cm.exception), "port lost")
        self.assertIsNone(l._ser, "the lost port was not closed")

    def test_reopen_waits_then_succeeds(self):
        l = SerialLink(reopen_wait=2.0)
        tries = {"n": 0}

        def open_():
            tries["n"] += 1
            if tries["n"] < 2:
                raise NoDevice("no OpenScope found")
        l.open = open_
        with mock.patch("openscope.link.time.sleep"):
            l.reopen()
        self.assertEqual((tries["n"], l.reopens), (2, 1))

    def test_discard_input_resets_the_os_buffer(self):
        ser = FakeSer(drains_after=0)
        link(ser).discard_input()
        self.assertEqual(ser.resets, 1)


class TestClosedLink(unittest.TestCase):
    """G-410: after a failed reopen `_ser` is None. Every call must raise an
    OSError (PortClosed), never AttributeError."""

    def test_every_call_on_a_closed_link_is_port_closed(self):
        l = SerialLink(reopen_wait=0)
        self.assertFalse(l.is_open)
        for call in (l.discard_input, l.read, lambda: l.write(b"x"), l.drain):
            with self.assertRaises(PortClosed) as cm:
                call()
            self.assertIsInstance(cm.exception, OSError)
            self.assertIn("port lost", str(cm.exception))

    def test_after_a_failed_reopen(self):
        l = link(FakeSer(drains_after=0))
        l.reopen_wait = 0
        self.assertTrue(l.is_open)
        with self.assertRaises(NoDevice):
            l.reopen()
        self.assertFalse(l.is_open)
        with self.assertRaises(PortClosed):
            l.discard_input()


if __name__ == "__main__":
    unittest.main(verbosity=2)
