"""Host tool against the REAL firmware protocol code, no hardware.

firmware/src/drivers/esp_comm.c is compiled into a shared library with a
small shim (fw_shim.c) and driven through ctypes. A FakeLink stands in for
pyserial: host writes go into esp_comm_route() exactly as CDC RX chunks do
on the device, protocol replies come back from the firmware's own writer,
and non-protocol bytes reach a tiny fake shell. So these tests exercise the
real parser, dispatcher, router and STATUS encoder from the host's side —
including the acceptance criteria of remote_protocol.md §6.
"""
from __future__ import annotations

import ctypes
import io
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from contextlib import redirect_stderr, redirect_stdout
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DRIVERS = os.path.join(ROOT, "firmware", "src", "drivers")
sys.path.insert(0, os.path.join(HERE, ".."))

from openscope import cli, proto  # noqa: E402
from openscope.device import Device, DeviceError, Nak, Timeout  # noqa: E402

_LIB = None


def lib():
    global _LIB
    if _LIB is None:
        cc = shutil.which("cc") or shutil.which("gcc")
        if cc is None:
            raise unittest.SkipTest("no C compiler for the firmware shim")
        out = os.path.join(tempfile.mkdtemp(), "libfwshim.so")
        subprocess.run([cc, "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                        "-I", DRIVERS, os.path.join(HERE, "fw_shim.c"),
                        os.path.join(DRIVERS, "esp_comm.c"), "-o", out], check=True)
        L = ctypes.CDLL(out)
        L.shim_feed.argtypes = [ctypes.c_char_p, ctypes.c_uint16, ctypes.c_uint32]
        L.shim_poll.argtypes = [ctypes.c_uint32]
        L.shim_take_tx.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
        L.shim_take_tx.restype = ctypes.c_uint32
        L.shim_take_shell.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
        L.shim_take_shell.restype = ctypes.c_uint32
        L.shim_set_meter.argtypes = [ctypes.c_uint32, ctypes.c_float] + [ctypes.c_int] * 4 + [ctypes.c_char_p] * 2
        L.shim_set_meter_wrong_mode.argtypes = [ctypes.c_int]
        L.shim_set_status.argtypes = [ctypes.c_int] * 4 + [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_int]
        L.shim_set_wave_state.argtypes = [ctypes.c_int]
        L.shim_set_waveform.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                        ctypes.c_uint32, ctypes.c_int, ctypes.c_int, ctypes.c_int]
        L.shim_set_wave_channel.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_int,
                                            ctypes.c_int, ctypes.c_uint32]
        L.shim_set_ctrl_result.argtypes = [ctypes.c_int]
        L.shim_set_ctrl_bound.argtypes = [ctypes.c_int]
        L.shim_last_ctrl.argtypes = [ctypes.c_int]
        L.shim_set_scope_state.argtypes = [ctypes.c_int] * 5 + [ctypes.c_uint32] + [ctypes.c_int] * 5
        L.shim_set_scope_channel.argtypes = [ctypes.c_int] * 4 + [ctypes.c_uint32]
        L.shim_set_desc_rate.argtypes = [ctypes.c_int, ctypes.c_uint32, ctypes.c_int]
        L.shim_set_desc_range.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_uint32, ctypes.c_int]
        L.shim_set_scope_probe.argtypes = [ctypes.c_int, ctypes.c_int]
        L.shim_set_desc_identity.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p]
        L.shim_set_desc_meter.argtypes = [ctypes.c_int]
        _LIB = L
    return _LIB


class FakeLink:
    """pyserial stand-in wired to the firmware shim (64-byte 'USB packets')."""

    def __init__(self):
        self.L = lib()
        self.L.shim_init()
        self.port = "/dev/fake-openscope"
        self.now = 0
        self.rx = bytearray()          # device -> host bytes not yet read
        self.shell_line = bytearray()
        self.shell_seen = bytearray()
        self.fail_next_write = False
        self.fail_next_read_after_write = False
        self.nak_status_after_reopen = False
        self._corrupt_next_status = False
        self.reopens = 0

    # device side ------------------------------------------------------
    def _pump(self):
        buf = ctypes.create_string_buffer(65536)
        n = self.L.shim_take_tx(buf, 65536)
        self.rx += buf.raw[:n]
        n = self.L.shim_take_shell(buf, 65536)
        for b in buf.raw[:n]:
            self.shell_seen.append(b)
            if b in (0x0D, 0x0A):
                # Like firmware shell_feed(): EVERY CR and every LF ends a line,
                # echoes CRLF and prints a prompt (so CRLF yields two prompts).
                line = bytes(self.shell_line).decode()
                self.shell_line.clear()
                self.rx += line.encode() + b"\r\n"
                if line:
                    self.rx += self._shell_reply(line)
                self.rx += b"> "
            else:
                self.shell_line.append(b)

    @staticmethod
    def _shell_reply(line):
        return {"version": b"OpenScope 2C53T\r\nBuild: test\r\n"}.get(line, b"Unknown command\r\n")

    # pyserial-ish surface used by Device ------------------------------
    def write(self, data):
        if self._corrupt_next_status and data[:2] == bytes([0xAA, proto.CMD_STATUS]):
            self._corrupt_next_status = False
            data = data[:-1] + bytes([data[-1] ^ 0xFF])         # garbled by the reconnect -> NAK
        if self.fail_next_write:
            self.fail_next_write = False
            raise OSError(6, "Device not configured")        # what macOS says after a replug
        for i in range(0, len(data), 64):
            self.now += 1
            self.L.shim_feed(data[i:i + 64], len(data[i:i + 64]), self.now)
        self._pump()

    def read(self, n=4096):
        if self.fail_next_read_after_write:
            self.fail_next_read_after_write = False
            raise OSError(6, "Device not configured")         # replug after the device acted
        self.now += 5
        self.L.shim_poll(self.now)
        self._pump()
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def drain(self, quiet=0.15, max_wait=1.5):
        return self.read(1 << 20)

    def discard_input(self):
        self._pump()                    # what the device sent so far is in the OS buffer
        self.rx.clear()

    def reopen(self):
        self.reopens += 1
        if self.nak_status_after_reopen:
            self._corrupt_next_status = True

    def open(self):
        self.opened = True

    closed = False

    def close(self):
        self.closed = True


def device(timeout=0.3):
    return Device(FakeLink(), timeout=timeout)


class TestAcceptance(unittest.TestCase):
    """remote_protocol.md §6 acceptance, against the firmware's own code."""

    def test_info_reports_actual_mode_and_battery(self):
        dev = device()
        dev.link.L.shim_set_status(0, 64, proto.FLAG_CHARGING, 3900, 5000, 0, 0)
        out = io.StringIO()
        with mock.patch.object(cli.Device, "open", return_value=dev), redirect_stdout(out):
            self.assertEqual(cli.main(["info"]), 0)
        text = out.getvalue()
        self.assertIn("firmware  OpenScope 2C53T shim", text)
        self.assertIn("mode      scope", text)
        self.assertIn("battery   64% (3900 mV, charging)", text)

        # "verified by changing mode on the device and seeing the value change"
        dev.link.L.shim_set_status(1, 63, 0, 3890, 6000, 0, 0)
        out = io.StringIO()
        with mock.patch.object(cli.Device, "open", return_value=dev), redirect_stdout(out):
            cli.main(["info"])
        self.assertIn("mode      meter", out.getvalue())
        self.assertIn("battery   63% (3890 mV)", out.getvalue())

    def test_no_device_is_a_clean_exit_1(self):
        err = io.StringIO()
        with mock.patch("openscope.link.candidate_ports", return_value=[]), redirect_stderr(err):
            self.assertEqual(cli.main(["info"]), 1)
        self.assertIn("no OpenScope found", err.getvalue())
        with mock.patch.object(cli, "candidate_ports", return_value=[]), redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(["ports"]), 1)


class TestButtons(unittest.TestCase):
    def test_press_reaches_the_injector(self):
        dev = device()
        dev.press("MENU")
        self.assertEqual(dev.link.L.shim_last_button(), 9)
        self.assertEqual(dev.link.L.shim_presses(), 1)

    def test_full_queue_is_a_nak_not_success(self):
        dev = device()
        dev.link.L.shim_set_inject_ok(0)
        with self.assertRaises(Nak) as cm:
            dev.press("OK")
        self.assertEqual(proto.ERRORS[cm.exception.code], "NOT_READY")
        err = io.StringIO()
        with mock.patch.object(cli.Device, "open", return_value=dev), redirect_stderr(err):
            self.assertEqual(cli.main(["press", "OK"]), 2)
        self.assertIn("NOT_READY", err.getvalue())


class TestMeter(unittest.TestCase):
    def test_no_reading_yet_is_not_ready(self):
        dev = device()
        with self.assertRaises(Nak) as cm:
            dev.meter()
        self.assertEqual(proto.ERRORS[cm.exception.code], "NOT_READY")

    def test_reading_roundtrip_through_firmware_encoder(self):
        dev = device()
        dev.link.L.shim_set_meter(42, 1.6141, 16141, 3, 1, 0x04, b"V", b"1.6141")
        m = dev.meter()
        self.assertEqual((m.update_count, m.raw_bcd, m.decimal_pos, m.unit, m.display), (42, 16141, 3, "V", "1.6141"))
        self.assertAlmostEqual(m.value, 1.6141, places=5)
        self.assertEqual(m.result, "normal")
        self.assertTrue(m.autorange and not m.hold and not m.negative)

    def test_continuous_log_survives_not_ready(self):
        dev = device()
        calls = {"n": 0}
        real = dev.meter

        def flaky():
            calls["n"] += 1
            if calls["n"] == 1:
                raise Nak(proto.CMD_GET_METER, 0x07)          # NOT_READY during a range change
            if calls["n"] > 3:
                raise KeyboardInterrupt                         # stop the "until Ctrl-C" loop
            dev.link.L.shim_set_meter(calls["n"], 1.0, 1000, 3, 1, 0, b"V", b"1.000")
            return real()
        dev.meter = flaky
        with mock.patch.object(cli.Device, "open", return_value=dev), \
                mock.patch("time.sleep"), redirect_stdout(io.StringIO()) as out, redirect_stderr(io.StringIO()):
            self.assertEqual(cli.main(["meter", "--count", "0"]), 0)
        self.assertIn("1.000 V", out.getvalue())

    def test_finite_count_waits_through_not_ready(self):
        # stage-272 (G-220): right after a function change the meter reads
        # NOT_READY for ~1.5-4.5 s; `meter --count N` waits, up to the stale limit.
        dev = device()
        calls = {"n": 0}
        real = dev.meter

        def changing():
            calls["n"] += 1
            if calls["n"] <= 3:
                raise Nak(proto.CMD_GET_METER, 0x07)
            dev.link.L.shim_set_meter(calls["n"], 1.0, 1000, 3, 1, 0, b"V", b"1.000")
            return real()
        dev.meter = changing
        with mock.patch.object(cli.Device, "open", return_value=dev), mock.patch("time.sleep"), \
                redirect_stdout(io.StringIO()) as out, redirect_stderr(io.StringIO()):
            self.assertEqual(cli.main(["meter", "--count", "2"]), 0)
        self.assertEqual(out.getvalue().count("1.000 V"), 2)

        clock = {"t": 1000.0}

        def fake_time():
            clock["t"] += 1.0
            return clock["t"]
        dev2 = device()                                  # never a reading: gives up, exit 2
        with mock.patch.object(cli.Device, "open", return_value=dev2), mock.patch("time.sleep"), \
                mock.patch.object(cli, "_now", fake_time), redirect_stdout(io.StringIO()), \
                redirect_stderr(io.StringIO()) as err:
            self.assertEqual(cli.main(["meter", "--count", "1"]), cli.EXIT_DEVICE_ERROR)
        self.assertIn("NOT_READY", err.getvalue())

    def test_outside_meter_mode_is_refused_not_frozen(self):
        dev = device()
        dev.link.L.shim_set_meter(42, 1.6141, 16141, 3, 1, 0, b"V", b"1.6141")
        dev.link.L.shim_set_meter_wrong_mode(1)
        with self.assertRaises(Nak) as cm:
            dev.meter()
        self.assertEqual(proto.ERRORS[cm.exception.code], "UNSUPPORTED_IN_MODE")

    def test_finite_count_gives_up_on_a_frozen_meter(self):
        dev = device()
        dev.link.L.shim_set_meter(5, 1.0, 1000, 3, 1, 0, b"V", b"1.000")   # never changes
        clock = {"t": 1000.0}

        def fake_time():
            clock["t"] += 1.0
            return clock["t"]
        err = io.StringIO()
        with mock.patch.object(cli.Device, "open", return_value=dev), mock.patch("time.sleep"), \
                mock.patch.object(cli, "_now", fake_time), redirect_stdout(io.StringIO()), redirect_stderr(err):
            self.assertEqual(cli.main(["meter", "--count", "2"]), 2)
        self.assertIn("not updating", err.getvalue())

    def test_cli_meter_logs_csv(self):
        import csv
        dev = device()
        dev.link.L.shim_set_meter(7, 3.3, 3300, 3, 1, 0, b"V", b"3.300")
        log = os.path.join(tempfile.mkdtemp(), "m.csv")
        with mock.patch.object(cli.Device, "open", return_value=dev), redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(["meter", "--count", "1", "--log", log]), 0)
        rows = list(csv.DictReader(open(log)))
        self.assertEqual(rows[0]["display"], "3.300")
        self.assertEqual(rows[0]["raw_bcd"], "3300")


def sine(period, n=1024, mid=128, amp=60, head_zeros=0):
    import math
    s = [max(0, min(255, int(round(mid + amp * math.sin(2 * math.pi * i / period))))) for i in range(n)]
    for i in range(head_zeros):
        s[i] = 0                                     # the record-head defect (scope_record.h)
    return bytes(s)


def arm_waveform(dev, frame_id=1000, step=2, tb=0x10, tb_tier=2, rate=12490, ordered=1, disagrees=0,
                 ch1=None, ch2=None, vdiv1=(6, 2, 1264486), vdiv2=(8, 1, 6586022)):
    L = dev.link.L
    L.shim_set_waveform(frame_id, step, tb, tb_tier, rate, ordered, disagrees, 128)
    c1 = ch1 if ch1 is not None else sine(40.0, head_zeros=64)
    c2 = ch2 if ch2 is not None else bytes((0xAA ^ i) & 0xFF for i in range(1024))
    L.shim_set_wave_channel(0, c1, len(c1), *vdiv1)
    L.shim_set_wave_channel(1, c2, len(c2), *vdiv2)
    return c1, c2


class TestWaveform(unittest.TestCase):
    """GET_WAVEFORM through the firmware's own encoder (esp_comm.c)."""

    def test_both_channels_roundtrip(self):
        dev = device()
        c1, c2 = arm_waveform(dev)
        w1, w2 = dev.waveform(3)
        self.assertEqual((w1.channel, w2.channel), (0, 1))
        self.assertEqual(w1.frame_id, w2.frame_id, "one request = one capture")
        self.assertEqual((w1.samples, w2.samples), (c1, c2), "samples byte for byte (CH2 holds 0xAA bytes)")
        self.assertEqual((w1.timebase_idx, w1.sample_rate_hz, w1.timebase_tier), (0x10, 12490, "measured"))
        self.assertEqual((w1.vdiv_idx, w1.uv_per_div, w1.vdiv_tier), (6, 1264486, "measured"))
        self.assertEqual((w2.vdiv_idx, w2.vdiv_tier), (8, "provisional"))
        self.assertTrue(w1.time_ordered and not w1.calibrated and not w1.timebase_disagrees)
        self.assertEqual((w1.counts_per_div, w1.head_skip, w1.sample_count), (32, 128, 1024))

    def test_single_channel_and_frame_id_advances(self):
        dev = device()
        arm_waveform(dev, frame_id=1000, step=2)
        a = dev.waveform(2)
        b = dev.waveform(2)
        self.assertEqual([w.channel for w in a], [1])
        self.assertEqual(b[0].frame_id, a[0].frame_id + 2)

    def test_no_capture_data_is_a_refusal_not_a_trace(self):
        dev = device()                                   # shim starts with no capture
        with self.assertRaises(Nak) as cm:
            dev.waveform(1)
        self.assertEqual(proto.ERRORS[cm.exception.code], "NO_CAPTURE_DATA")
        err = io.StringIO()
        with mock.patch.object(cli.Device, "open", return_value=dev), redirect_stderr(err), \
                redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(["scope", "--out", os.path.join(tempfile.mkdtemp(), "x.csv")]), 2)
        self.assertIn("NO_CAPTURE_DATA", err.getvalue())
        self.assertIn("demo", err.getvalue())

    def test_lying_provider_never_reaches_the_host(self):
        dev = device()
        arm_waveform(dev)
        dev.link.L.shim_set_wave_state(4)                # provider: OK + synthetic record
        with self.assertRaises(Nak) as cm:
            dev.waveform(3)
        self.assertEqual(proto.ERRORS[cm.exception.code], "NO_CAPTURE_DATA")

    def test_wrong_mode_and_busy(self):
        dev = device()
        arm_waveform(dev)
        dev.link.L.shim_set_wave_state(2)
        with self.assertRaises(Nak) as cm:
            dev.waveform(1)
        self.assertEqual(proto.ERRORS[cm.exception.code], "UNSUPPORTED_IN_MODE")
        dev.link.L.shim_set_wave_state(3)
        with self.assertRaises(Nak) as cm:
            dev.waveform(1)
        self.assertEqual(proto.ERRORS[cm.exception.code], "NOT_READY")

    def test_bad_mask_refused_on_both_sides(self):
        dev = device()
        arm_waveform(dev)
        for bad in (0, 4, 7):
            with self.assertRaises(ValueError):
                dev.waveform(bad)
        with self.assertRaises(Nak) as cm:               # the firmware's own guard, bypassing the host's
            dev.request_many(proto.CMD_GET_WAVEFORM, b"\x04", (proto.RSP_WAVEFORM_FRAME,))
        self.assertEqual(proto.ERRORS[cm.exception.code], "BAD_ARG")

    def test_unmeasured_numbers_are_withheld(self):
        dev = device()
        arm_waveform(dev, tb=0x08, tb_tier=0, rate=1414, vdiv1=(2, 0, 9999))   # incoherent code, railed range
        w = dev.waveform(1)[0]
        self.assertEqual((w.sample_rate_hz, w.uv_per_div), (0, 0), "a number its tier disowns never travels")
        self.assertIsNone(w.volts_per_count)
        sm = w.summary()
        self.assertIsNone(sm["vpp_volts"])
        self.assertIn("no measured volts/div", sm["vpp_note"])
        self.assertIsNone(sm["frequency_hz"])
        self.assertAlmostEqual(sm["period_samples"], 40.0, delta=0.3)

    def test_display_hardware_disagreement_withholds_rate(self):
        dev = device()
        arm_waveform(dev, disagrees=1)
        w = dev.waveform(1)[0]
        self.assertTrue(w.timebase_disagrees)
        self.assertEqual(w.sample_rate_hz, 0)
        self.assertIn("disagree", w.summary()["frequency_note"])

    def test_summary_skips_the_head_defect(self):
        dev = device()
        arm_waveform(dev)                                # sine 128 +/- 60 with 64 zeros at the head
        sm = dev.waveform(1)[0].summary()
        self.assertEqual(sm["analysed_from"], 128)
        self.assertGreaterEqual(sm["min_counts"], 60, "the zeroed head leaked into the statistics")
        self.assertNotIn("clipped", sm)
        self.assertAlmostEqual(sm["frequency_hz"], 12490 / 40.0, delta=2.0)
        self.assertAlmostEqual(sm["vpp_volts"], 120 * 1264486 / 1e6 / 32, delta=0.05)

    def test_cli_scope_writes_csv(self):
        import csv
        dev = device()
        c1, c2 = arm_waveform(dev, step=2)
        out = os.path.join(tempfile.mkdtemp(), "cap.csv")
        so = io.StringIO()
        with mock.patch.object(cli.Device, "open", return_value=dev), mock.patch("time.sleep"), \
                redirect_stdout(so):
            self.assertEqual(cli.main(["scope", "--frames", "2", "--channels", "1,2", "--out", out]), 0)
        with open(out, newline="") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 2 * 2 * 1024)
        self.assertEqual(rows[0]["frame_id"], "1000")
        self.assertEqual(rows[-1]["frame_id"], "1002", "two DISTINCT captures")
        ch2 = [int(r["sample"]) for r in rows if r["frame_id"] == "1000" and r["channel"] == "2"]
        self.assertEqual(bytes(ch2), c2)
        self.assertEqual(rows[0]["in_head_defect"], "1")
        self.assertEqual(rows[128]["in_head_defect"], "0")
        self.assertAlmostEqual(float(rows[100]["t_s"]), 100 / 12490, places=9)
        self.assertEqual(rows[0]["sample_rate_hz"], "12490")
        self.assertIn("saved", so.getvalue())

    def test_cli_scope_gives_up_on_a_held_capture(self):
        dev = device()
        arm_waveform(dev, step=0)                        # frame_id never advances (STOP / SINGLE)
        clock = {"t": 0.0}

        def fake_now():
            clock["t"] += 1.0
            return clock["t"]
        out = os.path.join(tempfile.mkdtemp(), "held.csv")
        err = io.StringIO()
        with mock.patch.object(cli.Device, "open", return_value=dev), mock.patch("time.sleep"), \
                mock.patch.object(cli, "_now", fake_now), redirect_stdout(io.StringIO()), redirect_stderr(err):
            self.assertEqual(cli.main(["scope", "--frames", "3", "--out", out]), 2)
        self.assertIn("stopped or held", err.getvalue())
        self.assertTrue(os.path.exists(out), "the one real capture is still saved (marked partial)")

    def test_cli_scope_npz(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest("numpy not installed (npz is optional; CSV is always available)")
        dev = device()
        c1, c2 = arm_waveform(dev, step=2)
        out = os.path.join(tempfile.mkdtemp(), "cap.npz")
        with mock.patch.object(cli.Device, "open", return_value=dev), mock.patch("time.sleep"), \
                redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(["scope", "--frames", "3", "--channels", "1,2", "--out", out]), 0)
        z = np.load(out)
        self.assertEqual(z["samples"].shape, (3, 2, 1024))
        self.assertEqual(bytes(z["samples"][0, 0]), c1)
        self.assertEqual(list(z["frame_id"]), [1000, 1002, 1004])
        self.assertEqual(list(z["channels"]), [1, 2])
        self.assertEqual(int(z["sample_rate_hz"][0]), 12490)
        self.assertIn("not calibrated", str(z["note"]))

    def test_cli_npz_without_numpy_is_a_usage_error_before_capturing(self):
        import builtins
        real_import = builtins.__import__

        def no_numpy(name, *a, **kw):
            if name == "numpy":
                raise ImportError("no numpy")
            return real_import(name, *a, **kw)
        dev = device()
        arm_waveform(dev)
        err = io.StringIO()
        with mock.patch.object(cli.Device, "open", return_value=dev), \
                mock.patch("builtins.__import__", no_numpy), redirect_stderr(err):
            self.assertEqual(cli.main(["scope", "--out", "x.npz"]), cli.EXIT_USAGE)
        self.assertIn("numpy", err.getvalue())


class TestSharedStream(unittest.TestCase):
    def test_shell_keeps_working_next_to_the_protocol(self):
        dev = device()
        self.assertEqual(dev.ping(), "OpenScope 2C53T shim")
        self.assertEqual(dev.shell("version"), "OpenScope 2C53T\r\nBuild: test")
        self.assertEqual(dev.status().mode_name, "scope")
        self.assertNotIn(0xAA, dev.link.shell_seen, "a protocol byte leaked into the shell")

    def test_abandoned_request_does_not_poison_the_next(self):
        dev = device()
        dev.link.write(proto.encode(proto.CMD_BUTTON, b"\x09")[:-2])   # host dies mid-frame
        dev.link.read()                                                # time passes (gap)
        dev.link.now += 100
        self.assertEqual(dev.ping(), "OpenScope 2C53T shim")
        self.assertEqual(dev.link.L.shim_presses(), 0)


class TestRound3(unittest.TestCase):
    def test_second_write_failure_is_a_device_error(self):
        dev = device()
        def always_fail(data):
            raise OSError(6, "Device not configured")
        dev.link.write = always_fail
        for call in (dev.ping, dev.status, lambda: dev.press("OK"), lambda: dev.shell("version")):
            with self.assertRaises(DeviceError) as cm:
                call()
            self.assertIn("port lost twice", str(cm.exception))

    def test_unknown_major_refused_before_anything_acts(self):
        dev = device()
        with mock.patch.object(proto, "PROTO_MAJOR", 2):
            with self.assertRaises(proto.ProtocolError):
                dev._check_version()
        self.assertEqual(dev.link.L.shim_presses(), 0)

    def test_version_rechecked_after_reopen(self):
        dev = device()
        dev._check_version()
        dev.link.fail_next_write = True
        with mock.patch.object(proto, "PROTO_MAJOR", 2):
            with self.assertRaises(proto.ProtocolError):
                dev.press("OK")                 # the reopened port speaks another major
        self.assertEqual(dev.link.L.shim_presses(), 0, "pressed on a firmware we cannot speak to")

    def test_battery_unknown_is_not_printed_as_zero(self):
        dev = device()
        dev.link.L.shim_set_status(0, 0, proto.FLAG_CHARGING | proto.FLAG_BATT_UNKNOWN, 0, 1000, 0, 0)
        out = io.StringIO()
        with mock.patch.object(cli.Device, "open", return_value=dev), redirect_stdout(out):
            cli.main(["info"])
        self.assertIn("battery   unknown (no sample yet, charging)", out.getvalue())
        self.assertNotIn("0%", out.getvalue())


class TestRound4(unittest.TestCase):
    def test_open_failures_are_one_line_exits(self):
        for exc in (proto.ProtocolError("device speaks protocol v2"), Timeout("no reply to 0x08"),
                    OSError(16, "Resource busy")):
            err = io.StringIO()
            with mock.patch.object(cli.Device, "open", side_effect=exc), redirect_stderr(err):
                self.assertEqual(cli.main(["info"]), 2, repr(exc))
            self.assertEqual(len(err.getvalue().strip().splitlines()), 1, err.getvalue())

    def test_port_that_keeps_vanishing_does_not_recurse(self):
        dev = device()
        dev._check_version()                                   # proto_version set, as after open()
        def always_fail(data):
            raise OSError(6, "Device not configured")
        dev.link.write = always_fail
        with self.assertRaises(DeviceError):
            dev.ping()
        self.assertLessEqual(dev.link.reopens, 2, "reopen recursed")


class TestRound5(unittest.TestCase):
    def test_nak_during_reopen_keeps_may_have_acted(self):
        from openscope import mcp_server
        dev = device()
        dev._check_version()
        dev.link.fail_next_read_after_write = True            # BUTTON sent, then the port drops
        dev.link.nak_status_after_reopen = True               # and the version re-check is NAKed
        with self.assertRaises(DeviceError) as cm:
            dev.press("OK")
        self.assertIn("may already have acted", str(cm.exception))
        self.assertEqual(dev.link.L.shim_presses(), 1)
        dev2 = device()
        dev2._check_version()
        dev2.link.fail_next_read_after_write = True
        dev2.link.nak_status_after_reopen = True
        s = mcp_server.ScopeSession(opener=lambda port: dev2)
        with self.assertRaises(RuntimeError) as cm2:
            s.press(["OK"])
        self.assertIn("MAY OR MAY NOT", str(cm2.exception))
        self.assertNotIn("NOT pressed (", str(cm2.exception).replace("MAY OR MAY NOT", ""))

    def test_device_open_itself_refuses_an_unknown_major(self):
        from openscope import device as devmod
        made = {}

        def factory(port=None, **kw):
            made["link"] = FakeLink()
            return made["link"]
        with mock.patch.object(devmod, "SerialLink", factory), mock.patch.object(proto, "PROTO_MAJOR", 2):
            with self.assertRaises(proto.ProtocolError):
                Device.open()
        self.assertTrue(made["link"].closed, "port left open after refusing")
        self.assertEqual(made["link"].L.shim_presses(), 0)


class TestRound6(unittest.TestCase):
    def test_failed_recheck_is_a_reconnect_error_and_blocks_actions(self):
        from openscope import mcp_server
        dev = device()
        dev._check_version()
        dev.link.fail_next_read_after_write = True      # STATUS query loses its reply...
        dev.link.nak_status_after_reopen = True         # ...and the re-check is NAKed
        s = mcp_server.ScopeSession(opener=lambda port: dev)
        with self.assertRaises(RuntimeError) as cm:
            s.info()
        self.assertIn("reconnect", str(cm.exception))
        self.assertIsNone(s._dev, "session kept a device whose firmware was never checked")
        # and the Device itself refuses to act until it re-verifies
        with mock.patch.object(proto, "PROTO_MAJOR", 2):
            with self.assertRaises(proto.ProtocolError):
                dev.press("OK")
        self.assertEqual(dev.link.L.shim_presses(), 0)

    def test_write_failure_with_nak_recheck_is_not_a_bare_nak(self):
        dev = device()
        dev._check_version()
        dev.link.fail_next_write = True
        dev.link.nak_status_after_reopen = True
        with self.assertRaises(DeviceError) as cm:
            dev.shell("version")
        self.assertNotIsInstance(cm.exception, Nak)


class TestStage180(unittest.TestCase):
    """Late replies, set timeouts and screenshot cancel (review H-3, H-4)."""

    def test_late_ack_after_the_resync_window_does_not_answer_the_next_request(self):
        dev = device()
        dev.ping()
        dev.link.L.shim_set_inject_ok(0)            # the next press is refused (NOT_READY)
        # A set ACK that came after its request timed out and after _resync():
        # it sits in the OS buffer when the next request is written.
        dev.link.rx += proto.encode(proto.RSP_ACK, b"")
        with self.assertRaises(Nak) as cm:
            dev.press("OK")                         # without the discard: "success" from the stale ACK
        self.assertEqual(proto.ERRORS[cm.exception.code], "NOT_READY")

    def test_half_frame_left_in_the_decoder_is_dropped_before_a_request(self):
        dev = device()
        # a stale header that claims 1000 B: kept, it would swallow the next reply
        dev.decoder.feed(bytes([proto.SYNC, proto.RSP_WAVEFORM_FRAME, 0x03, 0xE8]))
        dev.press("OK")                             # without the decoder reset: Timeout
        self.assertEqual(dev.link.L.shim_presses(), 1)

    def test_discard_failure_is_a_write_failure_and_is_resent(self):
        dev = device()
        calls = {"n": 0}

        def discard():
            calls["n"] += 1
            if calls["n"] == 1:
                raise OSError(22, "The device does not recognize the command")
            dev.link.rx.clear()
        dev.link.discard_input = discard
        self.assertEqual(dev.ping(), "OpenScope 2C53T shim")
        self.assertEqual(dev.link.reopens, 1)

    def test_scope_sets_use_the_set_timeout(self):
        dev = ctl_device()
        seen = []
        real = dev._request_frames

        def spy(cmd, payload, expect, count):
            seen.append((cmd, dev.timeout))
            return real(cmd, payload, expect, count)
        dev._request_frames = spy
        dev.set_timebase(0x10)
        dev.set_vdiv(1, 7)
        dev.set_trigger("auto")
        dev.run_stop(True)
        dev.ping()
        self.assertEqual([t for _, t in seen[:4]], [Device.SET_TIMEOUT_S] * 4)
        self.assertEqual(seen[4][1], 0.3, "the set timeout leaked into the next request")

    def test_screenshot_cancel_stops_in_under_half_a_second(self):
        from openscope.device import Cancelled
        dev = device()
        t0 = time.monotonic()
        stop_at = t0 + 0.2
        with mock.patch.object(FakeLink, "_shell_reply", staticmethod(lambda line: b"")):
            with self.assertRaises(Cancelled):
                dev.screenshot(attempts=3, timeout=20.0, header_timeout=20.0,
                               cancel=lambda: time.monotonic() > stop_at)
        self.assertLess(time.monotonic() - t0, 0.5)

    def test_screenshot_with_no_header_gives_up_after_header_timeout(self):
        dev = device()
        t0 = time.monotonic()
        with mock.patch.object(FakeLink, "_shell_reply", staticmethod(lambda line: b"")):
            with self.assertRaises(DeviceError) as cm:
                dev.screenshot(attempts=2, timeout=20.0, header_timeout=0.1)
        self.assertIn("no SCREENBIN header", str(cm.exception))
        self.assertLess(time.monotonic() - t0, 3.0)

    def test_non_ascii_shell_line_is_refused_before_anything_is_sent(self):
        dev = device()
        with self.assertRaises(ValueError):
            dev.shell("\u00e4")
        self.assertEqual(bytes(dev.link.shell_seen), b"")

    def test_reopen_wait_zero_loss_reaches_the_caller_at_once(self):
        from openscope.link import NoDevice, SerialLink
        dev = device()
        dev._check_version()
        real = SerialLink(reopen_wait=0)
        dev.link.reopen = real.reopen                 # the real loop, with reopen_wait=0
        dev.link.fail_next_read_after_write = True
        t0 = time.monotonic()
        with self.assertRaises(NoDevice) as cm:
            dev.ping()
        self.assertEqual(str(cm.exception), "port lost")
        self.assertLess(time.monotonic() - t0, 0.5)

    def test_device_open_passes_reopen_wait(self):
        from openscope import device as devmod
        made = {}

        def factory(port=None, **kw):
            made.update(kw)
            made["link"] = FakeLink()
            return made["link"]
        with mock.patch.object(devmod, "SerialLink", factory):
            Device.open(reopen_wait=0)
            self.assertEqual(made["reopen_wait"], 0)
            Device.open()
            self.assertEqual(made["reopen_wait"], 6.0, "the CLI default must stay 6 s")


def _dumpbin(data, hdr_crc, trailer_crc):
    import zlib  # noqa: F401
    t = b"" if trailer_crc is None else (b" crc32=%08X" % trailer_crc)
    return (b"SCREENBIN x=0 y=0 w=4 h=2 format=indexed4 len=%d crc32=%08X\r\n" % (len(data), hdr_crc)
            + data + b"\r\nSCREENBIN END" + t + b"\r\n")


class TestScreenshot(unittest.TestCase):
    DATA = bytes([0x01, 0x23, 0x45, 0xAA])                       # contains 0xAA on purpose

    def capture(self, hdr_ok, trailer):
        import zlib
        crc = zlib.crc32(self.DATA) & 0xFFFFFFFF
        reply = _dumpbin(self.DATA, crc if hdr_ok else crc ^ 1, trailer if trailer != "ok" else crc)
        dev = device()
        with mock.patch.object(FakeLink, "_shell_reply", staticmethod(
                lambda line: reply if line.startswith("screen dumpbin") else b"?\r\n")):
            return dev.screenshot(attempts=2, timeout=0.5)

    def test_consistent_frame(self):
        s = self.capture(hdr_ok=True, trailer="ok")
        self.assertEqual((s.indexed4, s.torn), (self.DATA, False))

    def test_live_screen_returns_a_torn_but_intact_frame(self):
        s = self.capture(hdr_ok=False, trailer="ok")
        self.assertEqual((s.indexed4, s.torn), (self.DATA, True))

    def test_transport_corruption_is_refused(self):
        with self.assertRaises(DeviceError):
            self.capture(hdr_ok=False, trailer=0x12345678)

    def test_old_firmware_without_trailer_still_fails_closed(self):
        with self.assertRaises(DeviceError):
            self.capture(hdr_ok=False, trailer=None)


class TestTransportLoss(unittest.TestCase):
    def test_button_is_not_resent_after_its_write_went_through(self):
        dev = device()
        dev.link.fail_next_read_after_write = True
        with self.assertRaises(DeviceError):
            dev.press("OK")
        self.assertEqual(dev.link.L.shim_presses(), 1, "the press happened once and was not repeated")

    def test_query_is_resent_after_a_lost_reply(self):
        dev = device()
        dev.link.fail_next_read_after_write = True
        self.assertEqual(dev.ping(), "OpenScope 2C53T shim")
        self.assertEqual(dev.link.reopens, 1)

    def test_non_read_only_shell_is_not_resent(self):
        dev = device()
        sent = []
        real_write = dev.link.write

        def write(data):
            sent.append(data)
            real_write(data)
            dev.link.fail_next_read_after_write = True         # `fwswap b` resets the device
        dev.link.write = write
        with self.assertRaises(DeviceError):
            dev.shell("fwswap b")
        self.assertEqual(sum(1 for d in sent if d.startswith(b"fwswap")), 1, "re-sent a resetting command")

    def test_flush_failure_counts_as_possibly_sent(self):
        from openscope.link import FlushFailed
        dev = device()
        real_write = dev.link.write
        state = {"first": True}

        def write(data):
            real_write(data)                                    # the device did get it
            if state["first"]:
                state["first"] = False
                raise FlushFailed("tcdrain: device vanished")
        dev.link.write = write
        with self.assertRaises(DeviceError):
            dev.press("OK")
        self.assertEqual(dev.link.L.shim_presses(), 1, "a possibly-delivered press was re-sent")

    def test_shell_survives_a_replug(self):
        dev = device()
        dev.link.fail_next_write = True
        self.assertEqual(dev.shell("version"), "OpenScope 2C53T\r\nBuild: test")
        self.assertEqual(dev.link.reopens, 1)

    def test_replug_mid_request_is_retried_once(self):
        dev = device()
        dev.link.fail_next_write = True          # CDC self-heal (#39) looks like a replug
        self.assertEqual(dev.ping(), "OpenScope 2C53T shim")
        self.assertEqual(dev.link.reopens, 1)

    def test_silent_device_times_out_with_a_real_error(self):
        dev = device(timeout=0.05)
        dev.link.write = lambda data: None       # swallowed: nothing ever answers
        with self.assertRaises(Timeout):
            dev.ping()


def ctl_device():
    dev = device()
    L = dev.link.L
    L.shim_set_scope_state(0, 1, 0x10, 0x10, 2, 12490, 0, 0, 0, 0, 0x5A)
    L.shim_set_scope_channel(0, 1, 6, 2, 42950)
    L.shim_set_scope_channel(1, 1, 8, 1, 279050)
    return dev


def run_cli(dev, argv):
    out, err = io.StringIO(), io.StringIO()
    with mock.patch.object(cli.Device, "open", return_value=dev), redirect_stdout(out), redirect_stderr(err):
        rc = cli.main(argv)
    return rc, out.getvalue(), err.getvalue()


class TestControlSurface(unittest.TestCase):
    """M6 sets against the real esp_comm.c: validation, provider calls, replies."""

    def test_scope_state_round_trips_through_the_firmware_encoder(self):
        s = ctl_device().scope_state()
        self.assertEqual((s.mode_name, s.running, s.timebase_code, s.sample_rate_hz, s.timebase_tier_name),
                         ("scope", True, 0x10, 12490, "measured"))
        self.assertEqual((s.ch[0].vdiv_idx, s.ch[0].uv_per_div, s.ch[1].tier_name), (6, 42950, "provisional"))
        self.assertEqual(s.trig_code, 0x5A)

    def test_firmware_zeroes_a_rate_its_tier_disowns(self):
        dev = ctl_device()
        dev.link.L.shim_set_scope_state(0, 1, 0x08, 0x08, 0, 99999, 0, 0, 0, 0, 0x80)    # tier NONE
        dev.link.L.shim_set_scope_channel(0, 1, 2, 0, 1234)                            # range NONE
        s = dev.scope_state()
        self.assertEqual((s.sample_rate_hz, s.ch[0].uv_per_div), (0, 0))

    def test_each_set_reaches_the_provider_with_its_arguments(self):
        dev = ctl_device()
        L = dev.link.L
        dev.set_timebase(0x14)
        self.assertEqual([L.shim_last_ctrl(i) for i in range(2)], [2, 0x14])
        dev.set_vdiv(2, 7)
        self.assertEqual([L.shim_last_ctrl(i) for i in range(3)], [3, 2, 7])
        dev.set_trigger("single", "falling", "CH2", -55)
        self.assertEqual([L.shim_last_ctrl(i) for i in range(5)], [4, 2, 1, 1, -55])
        dev.run_stop(False)
        self.assertEqual([L.shim_last_ctrl(i) for i in range(2)], [5, 0])
        dev.set_mode("meter")
        self.assertEqual([L.shim_last_ctrl(i) for i in range(2)], [1, 1])
        s = dev.scope_state()
        self.assertEqual((s.timebase_code, s.ch[1].vdiv_idx, s.trig_mode_name, s.trig_level, s.running, s.mode_name),
                         (0x14, 7, "single", -55, False, "meter"))

    def test_bad_field_is_bad_arg_and_the_provider_is_not_called(self):
        dev = ctl_device()
        L = dev.link.L
        bad = [(proto.CMD_SET_TIMEBASE, b"\x15"), (proto.CMD_SET_VDIV, b"\x00\x01"),
               (proto.CMD_SET_VDIV, b"\x01\x0a"), (proto.CMD_SET_TRIGGER, b"\x03\x00\x00\x00\x00"),
               (proto.CMD_SET_TRIGGER, b"\x00\x02\x00\x00\x00"), (proto.CMD_SET_TRIGGER, b"\x00\x00\x02\x00\x00"),
               (proto.CMD_SET_TRIGGER, b"\x00\x00\x00\x65\x00"),      # level +101
               (proto.CMD_SET_TRIGGER, b"\x00\x00\x00\x9b\xff"),      # level -101
               (proto.CMD_RUN_STOP, b"\x02"), (proto.CMD_SET_MODE, b"\x04")]
        for cmd, payload in bad:
            with self.assertRaises(Nak, msg=f"0x{cmd:02X} {payload.hex()}") as cm:
                dev.request(cmd, payload)
            self.assertEqual(proto.ERRORS[cm.exception.code], "BAD_ARG")
        self.assertEqual(L.shim_ctrl_calls(), 0)

    def test_wrong_length_is_bad_length(self):
        dev = ctl_device()
        for cmd, payload in ((proto.CMD_GET_SCOPE_STATE, b"\x00"), (proto.CMD_SET_TIMEBASE, b""),
                             (proto.CMD_SET_VDIV, b"\x01"), (proto.CMD_SET_TRIGGER, b"\x00\x00\x00\x00"),
                             (proto.CMD_RUN_STOP, b"\x01\x01")):
            with self.assertRaises(Nak) as cm:
                dev.request(cmd, payload)
            self.assertEqual(proto.ERRORS[cm.exception.code], "BAD_LENGTH")
        self.assertEqual(dev.link.L.shim_ctrl_calls(), 0)

    def test_provider_failure_is_not_ready_and_changes_nothing(self):
        dev = ctl_device()
        dev.link.L.shim_set_ctrl_result(1)
        with self.assertRaises(Nak) as cm:
            dev.set_timebase(0x12)
        self.assertEqual(proto.ERRORS[cm.exception.code], "NOT_READY")
        self.assertEqual(dev.scope_state().timebase_code, 0x10)
        dev.link.L.shim_set_ctrl_result(2)
        with self.assertRaises(Nak) as cm:
            dev.run_stop(True)
        self.assertEqual(proto.ERRORS[cm.exception.code], "UNSUPPORTED_IN_MODE")

    def test_unbound_provider_is_unsupported(self):
        dev = ctl_device()
        dev.link.L.shim_set_ctrl_bound(0)
        for call in (dev.scope_state, lambda: dev.set_timebase(0x10)):
            with self.assertRaises(Nak) as cm:
                call()
            self.assertEqual(proto.ERRORS[cm.exception.code], "UNSUPPORTED")

    def test_cli_state_prints_every_field(self):
        rc, out, _ = run_cli(ctl_device(), ["state"])
        self.assertEqual(rc, 0)
        for text in ("mode      scope, running", "0x10 in force, 12490 S/s (measured); display agrees",
                     "CH1       on , range 6, 42.95 mV/div (measured", "CH2       on , range 8",
                     "trigger   auto, rising, CH1, level +0 (reg 0x08 = 0x5A)"):
            self.assertIn(text, out)

    def test_cli_set_prints_the_state_in_force(self):
        dev = ctl_device()
        rc, out, _ = run_cli(dev, ["set", "timebase", "0x14"])
        self.assertEqual(rc, 0)
        self.assertIn("timebase  0x14 in force", out)
        rc, out, _ = run_cli(dev, ["set", "trigger", "normal", "-30"])     # edge/source kept
        self.assertEqual(rc, 0)
        self.assertIn("trigger   normal, rising, CH1, level -30", out)
        rc, out, _ = run_cli(dev, ["set", "vdiv", "1", "5"])
        self.assertIn("CH1       on , range 5", out)
        rc, out, _ = run_cli(dev, ["set", "stop"])
        self.assertIn("STOPPED", out)

    def test_cli_refusal_exits_2_with_a_readable_line(self):
        dev = ctl_device()
        dev.link.L.shim_set_ctrl_result(1)
        rc, out, err = run_cli(dev, ["set", "timebase", "0x12"])
        self.assertEqual(rc, 2)
        self.assertIn("refused", err)
        self.assertIn("NOT_READY", err)
        self.assertEqual(out, "")

    def test_cli_bad_value_is_usage_and_sends_nothing(self):
        dev = ctl_device()
        for argv in (["set", "timebase", "0x15"], ["set", "vdiv", "3", "1"], ["set", "trigger", "sometimes"],
                     ["set", "trigger", "auto", "200"], ["set", "mode", "dmm"], ["set", "vdiv", "1"]):
            rc, _, err = run_cli(dev, argv)
            self.assertEqual(rc, cli.EXIT_USAGE, argv)
        self.assertEqual(dev.link.L.shim_ctrl_calls(), 0)

    def test_absolute_set_is_resent_after_a_lost_reply(self):
        dev = ctl_device()
        dev.link.fail_next_read_after_write = True
        dev.set_timebase(0x11)
        self.assertEqual(dev.link.L.shim_ctrl_calls(), 2)          # applied twice, same state
        self.assertEqual(dev.scope_state().timebase_code, 0x11)


def desc_device():
    dev = ctl_device()
    L = dev.link.L
    L.shim_set_desc_rate(0x10, 12490, 2)
    L.shim_set_desc_rate(0x0D, 119678, 1)
    L.shim_set_desc_rate(0x08, 7777, 0)                 # tier NONE: must arrive as no rate
    L.shim_set_desc_range(0, 6, 1342188, 2)
    L.shim_set_desc_range(1, 8, 27905, 1)
    L.shim_set_desc_range(0, 2, 5555, 0)                # tier NONE: no volts meaning
    L.shim_set_scope_probe(0, 10)
    L.shim_set_desc_identity(UNIT1_UID, b"guest-coldtrace", b"0a1af35-dirty")
    return dev


UNIT1_UID = bytes.fromhex("0E927403" "0080006E" "2094CC07")   # words 0374920E 6E008000 07CC9420


def old_proto(rev: str = "adca54a"):
    """An older library's proto.py, from git: what an old host runs
    (adca54a = stage-085, 0a1af35 = stage-086 description parser)."""
    src = subprocess.run(["git", "-C", ROOT, "show", f"{rev}:tools/openscope_host/openscope/proto.py"],
                         capture_output=True, text=True, check=True).stdout
    mod = type(sys)(f"proto_{rev}")
    sys.modules[mod.__name__] = mod             # dataclasses look the module up
    try:
        exec(compile(src, f"proto_{rev}.py", "exec"), mod.__dict__)
    finally:
        del sys.modules[mod.__name__]
    return mod


class TestDescription(unittest.TestCase):
    """stage-086 against the real esp_comm.c: DESCRIPTION, SET_PROBE, SCOPE_STATE probe."""

    def test_description_round_trips_through_the_firmware_encoder(self):
        d = desc_device().description()
        self.assertEqual((d.samples_per_div, d.counts_per_div, len(d.rates)), (32, 32, 21))
        self.assertEqual(d.sample_rate(0x10), 12490)
        self.assertEqual(d.tier("timebase", 0x0D), "provisional")
        self.assertIsNone(d.sample_rate(0x08))
        self.assertEqual(d.tier("timebase", 0x08), "none")
        self.assertEqual(d.uv_per_div_bnc(1, 6), 1342188)
        self.assertEqual(d.tier("range", 2, 8), "provisional")
        self.assertIsNone(d.volts_per_div(1, 2))
        self.assertEqual(d.probe, (10, 1))
        self.assertAlmostEqual(d.volts_per_div(1, 6), 13.42188)
        self.assertEqual((d.trigger.code(0), d.trigger.code(-100), d.trigger.record_offset), (128, 4, -28))
        self.assertEqual(d.skipped, ())
        # stage-190: section 7 through the firmware encoder
        self.assertEqual(d.identity, proto.Identity(UNIT1_UID, 2, "guest-coldtrace", "0a1af35-dirty"))
        self.assertEqual(d.api_level, 2)
        self.assertEqual(d.identity.usb_serial, "0B41262E6E00")

    def test_identity_texts_cut_and_null_through_the_firmware_encoder(self):
        dev = desc_device()
        dev.link.L.shim_set_desc_identity(UNIT1_UID, b"g" * 40, None)
        i = dev.description(refresh=True).identity
        self.assertEqual((i.target, i.git_rev, i.api_level), ("g" * 31, "", 2))

    def test_stage086_host_skips_section_7(self):
        old = old_proto("0a1af35")
        raw = desc_device().request(proto.CMD_GET_DESCRIPTION, expect=(proto.RSP_DESCRIPTION,)).payload
        d = old.parse_description(raw)
        self.assertEqual(d.skipped, (7,))
        self.assertEqual(d.sample_rate(0x10), 12490)
        self.assertEqual(d.probe, (10, 1))

    def test_cli_info_and_describe_print_the_identity(self):
        for argv in (["info"], ["describe"]):
            rc, out, _ = run_cli(desc_device(), argv)
            self.assertEqual(rc, 0, argv)
            self.assertIn("unit ID   0374920E 6E008000 07CC9420 (USB serial 0B41262E6E00)", out)
            self.assertIn("build     guest-coldtrace at git 0a1af35-dirty", out)
            self.assertIn("API level 2", out)

    def test_cli_info_on_firmware_without_description(self):
        dev = desc_device()
        dev.link.L.shim_set_ctrl_bound(0)                # GET_DESCRIPTION -> UNSUPPORTED
        rc, out, _ = run_cli(dev, ["info"])
        self.assertEqual(rc, 0)
        self.assertIn("unit ID   not reported", out)
        self.assertIn("API level 0", out)

    def test_description_is_cached_per_connection(self):
        dev = desc_device()
        calls = []
        real = dev.request
        dev.request = lambda cmd, *a, **k: (calls.append(cmd), real(cmd, *a, **k))[1]
        dev.description()
        dev.description()
        self.assertEqual(calls.count(proto.CMD_GET_DESCRIPTION), 1)
        dev.description(refresh=True)
        self.assertEqual(calls.count(proto.CMD_GET_DESCRIPTION), 2)
        dev.set_probe(1, 1)                              # probe section now stale
        self.assertEqual(dev.description().probe, (1, 1))
        self.assertEqual(calls.count(proto.CMD_GET_DESCRIPTION), 3)

    def test_set_probe_reaches_the_provider_and_shows_in_state(self):
        dev = desc_device()
        L = dev.link.L
        dev.set_probe(2, 10)
        self.assertEqual([L.shim_last_ctrl(i) for i in range(3)], [6, 2, 10])
        self.assertEqual(dev.scope_state().probe, (10, 10))
        dev.set_probe(1, 1)
        self.assertEqual(dev.scope_state().probe, (1, 10))

    def test_bad_probe_is_bad_arg_and_the_provider_is_not_called(self):
        dev = desc_device()
        for payload in (b"\x00\x01", b"\x03\x0a", b"\x01\x02", b"\x02\x00", b"\x01\x64"):
            with self.assertRaises(Nak) as cm:
                dev.request(proto.CMD_SET_PROBE, payload)
            self.assertEqual(proto.ERRORS[cm.exception.code], "BAD_ARG")
        for cmd, payload in ((proto.CMD_SET_PROBE, b"\x01"), (proto.CMD_GET_DESCRIPTION, b"\x00")):
            with self.assertRaises(Nak) as cm:
                dev.request(cmd, payload)
            self.assertEqual(proto.ERRORS[cm.exception.code], "BAD_LENGTH")
        self.assertEqual(dev.link.L.shim_ctrl_calls(), 0)

    def test_old_host_still_parses_the_new_scope_state(self):
        old = old_proto()
        dev = desc_device()
        raw = dev.request(proto.CMD_GET_SCOPE_STATE, expect=(proto.RSP_SCOPE_STATE,)).payload
        self.assertEqual(len(raw), 38)
        s = old.parse_scope_state(raw)
        self.assertEqual((s.timebase_code, s.ch[0].uv_per_div, s.trig_code), (0x10, 42950, 0x5A))
        self.assertFalse(hasattr(s, "probe"))

    def test_cli_describe_table(self):
        rc, out, _ = run_cli(desc_device(), ["describe"])
        self.assertEqual(rc, 0)
        lines = out.splitlines()
        row10 = next(x for x in lines if x.rstrip().endswith("0x10"))
        self.assertIn("2.56 ms", row10)
        self.assertIn("12490 S/s", row10)
        self.assertIn("measured", row10)
        row08 = next(x for x in lines if x.rstrip().endswith("0x08"))
        self.assertIn("not calibrated", row08)
        self.assertEqual(sum(1 for x in lines if x.rstrip()[-4:-2] == "0x"), 21)    # every code
        ch1r6 = next(x for x in lines if x.startswith("  CH1") and x.rstrip().endswith(" 6"))
        self.assertIn("1.34 V", ch1r6)                   # at the BNC
        self.assertIn("13.4 V (x10)", ch1r6)             # at the probe
        self.assertEqual(sum(1 for x in lines if x.startswith("  CH")), 20)          # every range
        self.assertIn("trunc(level*256/206)", out)
        self.assertIn("probe     CH1 x10, CH2 x1", out)

    def test_cli_set_probe_and_state_prints_it(self):
        dev = desc_device()
        rc, out, _ = run_cli(dev, ["set", "probe", "1", "1"])
        self.assertEqual(rc, 0)
        self.assertIn("probe x1", out.splitlines()[2])
        rc, out, _ = run_cli(dev, ["state"])
        self.assertIn("CH1       on , range 6, 42.95 mV/div (measured, bench unit #1), probe x1", out)
        for argv in (["set", "probe", "1", "2"], ["set", "probe", "3", "10"], ["set", "probe", "1"]):
            rc, _, _ = run_cli(dev, argv)
            self.assertEqual(rc, cli.EXIT_USAGE, argv)
        self.assertEqual(dev.link.L.shim_last_ctrl(0), 6)
        self.assertEqual(dev.link.L.shim_ctrl_calls(), 1)


def meter_device(mode=1):
    """desc_device() with section 5 (the guest-coldtrace list) in `mode`."""
    dev = desc_device()
    dev.link.L.shim_set_desc_meter(11)
    dev.link.L.shim_set_scope_state(mode, 1, 0x10, 0x10, 2, 12490, 0, 0, 0, 0, 0x5A)
    return dev


USABLE = ["DC Voltage", "AC Voltage", "Resistance", "Continuity", "Diode"]


class TestMeterFunction(unittest.TestCase):
    """stage-272 against the real esp_comm.c: SET_METER_MODE and section 5."""

    def test_section_5_through_the_firmware_encoder(self):
        d = meter_device().description()
        self.assertEqual(len(d.meter_functions), 11)
        self.assertEqual([f.id for f in d.meter_functions], list(range(11)))
        self.assertEqual([f.name for f in d.meter_functions if f.usable], USABLE)
        self.assertEqual([f.unit for f in d.meter_functions],
                         ["V", "V", "mA", "A", "mA", "A", "kOhm", "Ohm", "V", "nF", "C"])
        self.assertEqual(d.skipped, ())
        self.assertEqual(d.api_level, 2)

    def test_older_hosts_skip_section_5(self):
        raw = meter_device().request(proto.CMD_GET_DESCRIPTION, expect=(proto.RSP_DESCRIPTION,)).payload
        d086 = old_proto("0a1af35").parse_description(raw)       # stage-086 library
        self.assertEqual(d086.skipped, (7, 5))
        self.assertEqual((d086.sample_rate(0x10), d086.probe), (12490, (10, 1)))
        d270 = old_proto("c78f889").parse_description(raw)       # stage-270 library (has section 7)
        self.assertEqual(d270.skipped, (5,))
        self.assertEqual(d270.identity.api_level, 2)

    def test_set_by_name_or_id_reaches_the_provider(self):
        dev = meter_device()
        L = dev.link.L
        for arg, fid in (("dc-voltage", 0), ("Resistance", 6), ("ac_voltage", 1), (8, 8), ("7", 7)):
            f = dev.set_meter_function(arg)
            self.assertEqual(f.id, fid, arg)
            self.assertEqual([L.shim_last_ctrl(0), L.shim_last_ctrl(1)], [7, fid])
            self.assertEqual(L.shim_meter_submode(), fid)
        L.shim_set_meter(9, 0.12, 12, 2, 1, 0, b"Ohm", b"0.12")
        m = dev.meter()
        self.assertEqual(dev.description().meter_function_name(m.submode), "Continuity")

    def test_refusals_change_nothing(self):
        dev = meter_device(mode=0)                                # scope mode
        L = dev.link.L
        with self.assertRaises(Nak) as cm:
            dev.set_meter_function("resistance")
        self.assertEqual(proto.ERRORS[cm.exception.code], "UNSUPPORTED_IN_MODE")
        for name in ("capacitance", "dc-ma", "temperature"):     # not usable: refused in any mode
            for mode in (0, 1):
                L.shim_set_scope_state(mode, 1, 0x10, 0x10, 2, 12490, 0, 0, 0, 0, 0x5A)
                with self.assertRaises(Nak) as cm:
                    dev.set_meter_function(name)
                self.assertEqual(proto.ERRORS[cm.exception.code], "BAD_ARG", (name, mode))
        self.assertEqual(L.shim_meter_submode(), 0)
        calls = L.shim_ctrl_calls()
        for bad in ("bogus", 11, "voltage"):                       # not listed: nothing sent
            with self.assertRaises(ValueError):
                dev.set_meter_function(bad)
        with self.assertRaises(Nak) as cm:                         # 11 on the wire: esp_comm refuses
            dev.request(proto.CMD_SET_METER_MODE, b"\x0b")
        self.assertEqual(proto.ERRORS[cm.exception.code], "BAD_ARG")
        self.assertEqual(L.shim_ctrl_calls(), calls)               # the provider never saw them

    def test_firmware_without_section_5(self):
        dev = desc_device()                                        # no section 5
        with self.assertRaisesRegex(ValueError, "does not list meter functions"):
            dev.set_meter_function("dc-voltage")
        self.assertEqual(dev.link.L.shim_ctrl_calls(), 0)

    def test_cli_set_meter(self):
        dev = meter_device()
        rc, out, _ = run_cli(dev, ["set", "meter", "dc", "voltage"])
        self.assertEqual(rc, 0)
        self.assertIn("meter     DC Voltage (V) requested", out)
        rc, out, _ = run_cli(dev, ["set", "meter", "Resistance"])
        self.assertEqual((rc, dev.link.L.shim_meter_submode()), (0, 6))
        rc, _, err = run_cli(dev, ["set", "meter", "capacitance"])
        self.assertEqual(rc, cli.EXIT_DEVICE_ERROR)
        self.assertIn("BAD_ARG: this function is not usable on this firmware", err)
        calls = dev.link.L.shim_ctrl_calls()
        for argv in (["set", "meter", "bogus"], ["set", "meter"], ["set", "meter", "11"]):
            rc, _, err = run_cli(dev, argv)
            self.assertEqual(rc, cli.EXIT_USAGE, argv)
        self.assertEqual(dev.link.L.shim_ctrl_calls(), calls)
        self.assertIn("dc-voltage, ac-voltage", err)                # names the choices

    def test_cli_set_meter_in_scope_mode_exits_2(self):
        dev = meter_device(mode=0)
        rc, _, err = run_cli(dev, ["set", "meter", "resistance"])
        self.assertEqual(rc, cli.EXIT_DEVICE_ERROR)
        self.assertIn("UNSUPPORTED_IN_MODE: the unit is not in meter mode", err)
        self.assertEqual(dev.link.L.shim_meter_submode(), 0)

    def test_cli_describe_and_meter_print_the_functions(self):
        dev = meter_device()
        rc, out, _ = run_cli(dev, ["describe"])
        self.assertEqual(rc, 0)
        rows = [x.split() for x in out.splitlines() if x.startswith("  ") and len(x.split()) == 4
                and x.split()[2] in ("yes", "no")]
        self.assertEqual(len(rows), 11)
        self.assertEqual(rows[6], ["resistance", "kOhm", "yes", "6"])
        self.assertEqual(rows[9], ["capacitance", "nF", "no", "9"])
        rc, out, _ = run_cli(desc_device(), ["describe"])    # the shim is one global: reset
        self.assertIn("meter     functions not reported (firmware before stage-272)", out)
        dev = meter_device()
        dev.link.L.shim_set_meter(3, 1.6232, 16232, 4, 1, 0x02, b"V", b"1.6232")
        dev.set_meter_function("ac-voltage")
        rc, out, _ = run_cli(dev, ["meter"])
        self.assertEqual(rc, 0)
        self.assertIn("1.6232 V   AC Voltage   (normal", out)


if __name__ == "__main__":
    unittest.main(verbosity=2)
