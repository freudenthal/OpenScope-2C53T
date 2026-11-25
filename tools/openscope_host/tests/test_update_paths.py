"""Stage-205 update paths without hardware: an old OpenScope image with the
shell only, the first install from stock, and the return to stock. Uses the
fake unit and the fake bench of test_update.py / test_calbackup.py."""
from __future__ import annotations

import hashlib
import io
import json
import os
import struct
import sys
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from openscope import calbackup as cb  # noqa: E402
from openscope import cli, iapdrive, proto  # noqa: E402
from openscope import firmware as fw  # noqa: E402
from openscope import update as up  # noqa: E402
from openscope.device import Device, Nak, Timeout  # noqa: E402
from openscope.link import NoDevice  # noqa: E402

import test_calbackup as tcb  # noqa: E402
import test_update as tu  # noqa: E402

V041_BUILD = "Oct  5 2026 15:13:06"
STOCK_NAME = "APP_2C53T_V1.2.0_251015.bin"


def stock_image(size=8192) -> bytes:
    """Stock-like: vectors that pass the I-10 check, no OpenScope text."""
    img = bytearray(b"\x00" * size)
    img[0:8] = struct.pack("<II", 0x20036F90, 0x08007311)
    return bytes(img)


class PathUnit(tu.UpdUnit):
    """The fake unit with a switchable firmware: binary=False, fmt="v041" is
    an old image (shell only, no section 7); `build` is its shell Build text."""

    def __init__(self, binary=True, fmt="ours", build="Nov 24 2026 22:11:53", **kw):
        super().__init__(fmt=fmt, binary=binary, **kw)
        self.build = build

    def _answer_frame(self, cmd):
        if not self.binary:
            self.frames.append(cmd)             # no binary protocol: no reply
            return
        super()._answer_frame(cmd)

    def _reply(self, line):
        if line == "version":
            text = f"OpenScope 2C53T\r\nBuild: {self.build}, gcc 13.2.1 20231009\r\n"
            if self.fmt == "ours":
                text += "Git: 9fa1d98  Target: guest-coldtrace  API: 1\r\n"
            return (text + "MCU: AT32F403A @ 240MHz\r\n").encode()
        return super()._reply(line)


class PathBench(tu.Bench):
    """states: "app" (port), "iap" (drive), "stock" (stock USB Sharing drive,
    no port), "off", "restarting" (the port comes back after a few polls).
    flash: the unit attributes that the copy sets (the new firmware)."""

    def __init__(self, tmp, unit, clock):
        super().__init__(tmp, unit, clock)
        self.flash = {"fw": self.new_fw, "ident": self.new_ident}
        self.frames_at_copy = None
        self.lines_at_copy = None
        self.restart_polls = 0
        self.restarts_on_request = True

    def ports(self):
        if self.state == "restarting":
            self.restart_polls += 1
            if self.restart_polls > 3:
                self.state = "app"
            return []
        return super().ports()

    def probe(self):
        out = super().probe()
        if self.state == "stock":
            out.append(tu.drive("F:\\", "stock"))
        return out

    def open_backup(self, port):
        """Like calbackup.open_for_backup(): the protocol check, then shell only."""
        self.opened += 1
        dev = Device(self.unit, timeout=0.3, sleep=self.clock.sleep)
        try:
            dev._check_version()
        except (Timeout, Nak, proto.ProtocolError):
            dev = Device(self.unit, timeout=0.3, sleep=self.clock.sleep)
        return dev

    def opener(self, path, mode, buffering=-1):
        self.frames_at_copy = list(self.unit.frames)
        self.lines_at_copy = list(self.unit.lines)
        wrapped = super().opener(path, mode, buffering)
        bench, inner_exit = self, wrapped.__exit__

        def exit_(*a):
            inner_exit(*a)
            if bench.copy_completes:
                for k, v in bench.flash.items():
                    setattr(bench.unit, k, v)
        wrapped.__class__.__exit__ = lambda self_, *a: exit_(*a)
        return wrapped

    def report(self, e):
        self.events.append(e)
        if e.kind == "wait" and e.step == "restart":
            if self.restarts_on_request:
                self.state = "restarting"
        elif e.kind == "wait" and self.user_enters:
            self.state = "iap"

    def hooks(self):
        h = super().hooks()
        h.open_backup = self.open_backup
        return h


class Base(unittest.TestCase):
    def setUp(self):
        import shutil
        import tempfile
        self.clock = tcb.FakeClock()
        p = mock.patch("openscope.device.time", self.clock)
        p.start()
        self.addCleanup(p.stop)
        self.tmp = Path(tempfile.mkdtemp(prefix="update-paths-"))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.root = self.tmp / "backups"
        self.img = tu.make_image()
        self.m = tu.manifest_for(self.img)

    def bench(self, unit) -> PathBench:
        self.unit = unit
        self.n = getattr(self, "n", 0) + 1
        place = self.tmp / f"bench{self.n}"
        place.mkdir()
        self.b = PathBench(place, unit, self.clock)
        self.b.t += tu.timedelta(hours=self.n)            # benches of one test: new backup folders
        return self.b

    def assert_read_only_lines(self, lines):
        for line in lines:
            self.assertTrue(any(line == c or line.startswith(c + " ") for c in cb.ALLOWED), line)

    def wait_events(self):
        return [e for e in self.b.events if e.kind == "wait"]


# ── old image: shell only ─────────────────────────────────────────────

class TestOldImage(Base):
    def test_probe_firmware(self):
        b = self.bench(PathUnit())
        self.assertEqual(up.probe_firmware("FAKE7", b.open_backup), up.BINARY)
        b = self.bench(PathUnit(binary=False, fmt="v041"))
        self.assertEqual(up.probe_firmware("FAKE7", b.open_backup), up.SHELL_ONLY)
        self.assertEqual(self.unit.frames, [proto.CMD_STATUS])       # the probe's one request
        self.assertEqual(self.unit.lines, ["version"])

        def gone(port):
            raise NoDevice("not found")
        self.assertEqual(up.probe_firmware(None, gone), up.NONE)
        self.unit._reply = lambda line: b"Unknown command\r\n"         # not an OpenScope shell
        self.assertEqual(up.probe_firmware("FAKE7", b.open_backup), up.NONE)

    def test_old_image_to_latest(self):
        b = self.bench(PathUnit(binary=False, fmt="v041", build=V041_BUILD))
        b.flash.update(binary=True, fmt="ours")
        r = up.run_update(self.img, self.m, "new.bin", backup_root=self.root,
                          report=b.report, hooks=b.hooks())
        self.assertTrue(r.ok, r.problems)
        self.assertEqual((r.path, r.probe), ("A", up.SHELL_ONLY))
        self.assertEqual(b.frames_at_copy, [proto.CMD_STATUS])         # no binary request after the probe
        self.assert_read_only_lines(b.lines_at_copy)
        info = cb.read_info(r.before_backup)
        self.assertIs(info["firmware"]["binary_protocol"], False)
        self.assertEqual(info["unit"]["uid"], tcb.UID_HEX)
        self.assertIn(V041_BUILD, info["firmware"]["version_text"])
        self.assertEqual(r.before_backup.parent, r.after_backup.parent)
        log = json.loads(r.log.read_text())
        self.assertEqual((log["probe"], log["before"]["api_level"]), (up.SHELL_ONLY, 0))
        self.assertTrue(log["compare"]["block_equal"])
        self.assertTrue(any("mem read" in d for s in log["steps"] for d in s["detail"]))

    def test_old_image_uid_gate(self):
        b = self.bench(PathUnit(binary=False, fmt="v041"))
        b.ports = lambda: [("FAKE7", "FFFF00000000")]                 # not the serial of this ID
        with self.assertRaisesRegex(up.Refused, "serial made from the unit ID"):
            up.run_update(self.img, self.m, "new.bin", backup_root=self.root,
                          report=b.report, hooks=b.hooks())
        self.assertFalse(b.copied())
        self.assertEqual(self.wait_events(), [])

    def test_old_image_drive_of_another_unit(self):
        b = self.bench(PathUnit(binary=False, fmt="v041"))
        b.probe = lambda: [tu.drive(b.drive_dir, usb_serial="FFFF00000000")] if b.state == "iap" else []
        with self.assertRaisesRegex(up.Refused, "different unit"):
            up.run_update(self.img, self.m, "new.bin", backup_root=self.root,
                          report=b.report, hooks=b.hooks())
        self.assertFalse(b.copied())

    def test_install_an_old_image_verifies_on_the_shell(self):
        old = tu.make_image(build=V041_BUILD, rev=None, target=None)
        m = tu.manifest_for(old, api_level=0)
        b = self.bench(PathUnit())
        b.flash = {"binary": False, "fmt": "v041", "build": V041_BUILD}
        r = up.run_update(old, m, "release-v0.4.1-coldtrace.bin", backup_root=self.root,
                          report=b.report, hooks=b.hooks())
        self.assertTrue(r.ok, r.problems)
        self.assertIs(cb.read_info(r.after_backup)["firmware"]["binary_protocol"], False)
        b2 = self.bench(PathUnit())
        b2.flash = {"binary": False, "fmt": "v041", "build": "Oct  6 2026 01:00:00"}
        r2 = up.run_update(old, m, "release-v0.4.1-coldtrace.bin", backup_root=self.root,
                           report=b2.report, hooks=b2.hooks())
        self.assertFalse(r2.ok)
        self.assertTrue(any("shell Build time" in p for p in r2.problems), r2.problems)

    def test_check_running_shell(self):
        m = tu.manifest_for(tu.make_image(build=V041_BUILD, rev=None), api_level=0)
        text = f"OpenScope 2C53T\r\nBuild: {V041_BUILD}, gcc 14.2.1\r\n"
        self.assertEqual(fw.check_running_shell(m, text), [])
        self.assertTrue(fw.check_running_shell(m, "Unknown command"))
        m1 = fw.Manifest(**{**m.__dict__, "api_level": 1})
        self.assertTrue(any("shell only" in x for x in fw.check_running_shell(m1, text)))


# ── first install from stock ──────────────────────────────────────────

class TestFirstInstall(Base):
    def setUp(self):
        super().setUp()
        self.page = tcb.make_page()
        self.w25q = tcb.make_w25q(cb.W25Q_LEN)

    def stock_bench(self, page=None):
        b = self.bench(PathUnit(page=page or self.page, w25q=self.w25q))
        b.state = "stock"
        b.after_copy = "app"
        return b

    def first(self, **kw):
        kw.setdefault("model", "2C53T")
        kw.setdefault("install_ack", up.FIRST_INSTALL_ACK)
        return up.run_first_install(kw.pop("data", self.img), kw.pop("manifest", self.m),
                                    "guest-coldtrace-abc1234.bin", backup_root=self.root,
                                    report=self.b.report, hooks=self.b.hooks(), **kw)

    def test_order_backup_before_any_key_press(self):
        self.stock_bench()
        r = self.first()
        self.assertTrue(r.ok, r.problems)
        self.assertEqual(r.path, "F")
        ev = self.b.events
        copied = next(i for i, e in enumerate(ev) if e.kind == "progress" and e.text == "copied")
        backup_step = next(i for i, e in enumerate(ev) if e.kind == "step" and e.step == "backup")
        restart = next(i for i, e in enumerate(ev) if e.kind == "wait" and e.step == "restart")
        self.assertLess(copied, backup_step)
        self.assertLess(backup_step, restart)
        # no prompt between the copy and the end of the backup
        self.assertEqual([e for e in ev[copied:restart] if e.kind == "wait"], [])
        enter = [e for e in ev[:copied] if e.kind == "wait"]
        self.assertEqual(len(enter), 1)
        self.assertIn("Do not press a key", enter[0].text)
        files = sorted(os.listdir(r.before_backup))
        self.assertIn(cb.W25Q_FILE, files)
        self.assertTrue(cb.verify_backup(r.before_backup).ok)
        log = json.loads(r.log.read_text())
        self.assertEqual(r.log.parent, r.before_backup)
        self.assertEqual(log["first_reference"]["folder"], str(r.before_backup))
        self.assertEqual(log["drive_usb_serial"], tu.SERIAL)
        self.assertTrue(log["compare"]["block_equal"])               # second dump after the restart
        self.assertEqual([s["step"] for s in log["steps"]],
                         ["check", "enter", "drive", "copy", "wait", "backup", "restart",
                          "verify", "log"])

    def test_stock_files_are_named(self):
        self.stock_bench()
        self.first()
        infos = [e.text for e in self.b.events if e.kind == "info" and e.step == "check"]
        self.assertTrue(any("USB Sharing drive F:" in t and "Screenshot file" in t for t in infos), infos)
        self.assertTrue(any(up.STOCK_FILES_TEXT == t for t in infos))

    def test_model_confirmation_required(self):
        self.stock_bench()
        for model in (None, "", "2C23T", "1013D"):
            with self.assertRaisesRegex(up.Refused, "model is not confirmed"):
                self.first(model=model)
        for ack in (None, "install", "yes"):
            with self.assertRaisesRegex(up.Refused, "INSTALL"):
                self.first(install_ack=ack)
        self.assertEqual(self.wait_events(), [])
        self.assertFalse(self.b.copied())
        self.assertEqual(self.b.opened, 0)
        self.assertTrue(self.first(model=" 2c53t ").ok)

    def test_refused_when_openscope_runs(self):
        self.stock_bench()
        self.b.state = "app"
        with self.assertRaisesRegex(up.Refused, "runs OpenScope already"):
            self.first()
        self.assertFalse(self.b.copied())

    def test_compared_with_an_earlier_backup(self):
        self.stock_bench()
        unit_dir = self.root / tcb.UID_HEX
        ref = cb.make_backup(Device(self.unit, timeout=0.3, sleep=self.clock.sleep),
                             root=self.root, now=tu.datetime(2026, 11, 1).astimezone())
        r = self.first()
        self.assertTrue(r.ok, r.problems)
        log = json.loads(r.log.read_text())
        self.assertEqual(log["reference_backup"], str(ref))
        self.assertTrue(log["compare_reference"]["block_equal"])
        self.assertTrue(unit_dir.is_dir())
        page = bytearray(self.page)                                    # the reference was another cal
        page[0x40] ^= 0xFF
        (ref / cb.CAL_FILE).chmod(0o666)
        self.stock_bench(page=bytes(page))
        r2 = self.first()
        self.assertFalse(r2.ok)
        self.assertTrue(any("earlier backup" in p for p in r2.problems), r2.problems)

    def test_cal_change_after_restart_reported(self):
        self.stock_bench()
        real = self.b.ports

        def change_on_restart():
            if self.b.state == "restarting" and self.b.restart_polls == 3:
                self.unit.page[0x50] ^= 0xFF
            return real()
        self.b.ports = change_on_restart
        r = self.first()
        self.assertFalse(r.ok)
        self.assertTrue(any("cal block changed" in p for p in r.problems), r.problems)

    def test_no_restart_is_an_error_with_a_log(self):
        self.stock_bench()
        self.b.restarts_on_request = False
        with self.assertRaisesRegex(up.UpdateError, "did not go") as cm:
            self.first()
        self.assertTrue(cm.exception.copied)
        self.assertTrue(cm.exception.log.is_file())


# ── return to stock ───────────────────────────────────────────────────

class TestStock(Base):
    def setUp(self):
        super().setUp()
        self.stock = stock_image()
        self.sha = hashlib.sha256(self.stock).hexdigest()
        self.allowed = {self.sha: "test stock V9"}

    def run_stock(self, data=None, allowed="default", confirm=lambda t: True):
        return up.run_stock_install(data or self.stock, STOCK_NAME, backup_root=self.root,
                                    allowed=self.allowed if allowed == "default" else allowed,
                                    confirm=confirm, report=self.b.report, hooks=self.b.hooks())

    def test_allow_list(self):
        self.assertIn("a17c5c35c97bb898f15672a1747bc1041d8ed507c16999ddba0d1e4e2ec0c760", fw.STOCK_ALLOW)
        self.assertEqual(fw.check_stock_image(self.stock, self.allowed), [])
        r = fw.check_stock_image(self.stock)
        self.assertTrue(any("not on the stock allow-list" in x for x in r), r)
        other = stock_image(size=8196)
        self.assertTrue(fw.check_stock_image(other, self.allowed))
        bad = bytearray(self.stock)                                    # size and vector checks stay
        bad[0:8] = struct.pack("<II", 0x20036F90, 0x08004001)
        allowed = {hashlib.sha256(bytes(bad)).hexdigest(): "x"}
        self.assertTrue(any("below" in x for x in fw.check_stock_image(bytes(bad), allowed)))
        self.assertTrue(fw.check_image(self.stock, None, allow_unlisted=True))   # the normal path refuses

    def test_return_to_stock(self):
        b = self.bench(PathUnit())
        b.after_copy = "off"                                           # stock: no CDC port
        asked = []
        r = self.run_stock(confirm=lambda t: asked.append(t) or True)
        self.assertTrue(r.ok, r.problems)
        self.assertEqual((r.path, r.probe), ("S", up.BINARY))
        self.assertEqual((b.drive_dir / STOCK_NAME).read_bytes(), self.stock)
        self.assertEqual(asked, [up.STOCK_CONFIRM_TEXT])
        self.assertTrue(cb.verify_backup(r.before_backup).ok)
        log = json.loads(r.log.read_text())
        self.assertIn("G-150", log["note"])
        self.assertEqual(log["stock_image"], "test stock V9")
        self.assertTrue(log["user_confirmed_stock"])
        self.assertIsNone(r.after_backup)

    def test_user_says_no(self):
        b = self.bench(PathUnit())
        b.after_copy = "off"
        r = self.run_stock(confirm=lambda t: False)
        self.assertFalse(r.ok)
        self.assertTrue(b.copied())

    def test_other_stock_file_refused(self):
        b = self.bench(PathUnit())
        with self.assertRaisesRegex(up.Refused, "allow-list"):
            self.run_stock(allowed=None)
        self.assertEqual(b.opened, 0)
        self.assertFalse(b.copied())

    def test_openscope_image_refused_on_the_stock_path(self):
        b = self.bench(PathUnit())
        with self.assertRaisesRegex(up.Refused, "allow-list"):
            self.run_stock(data=self.img, allowed=None)
        self.assertFalse(b.copied())

    def test_needs_a_port_for_the_backup(self):
        b = self.bench(PathUnit())
        b.state = "iap"
        with self.assertRaises(NoDevice):
            self.run_stock()
        self.assertFalse(b.copied())


class TestCli(Base):
    def test_first_install_asks_the_model(self):
        img = self.tmp / "x.bin"
        img.write_bytes(self.img)
        self.m.save(str(img) + fw.MANIFEST_SUFFIX)
        err = io.StringIO()
        with mock.patch.object(up, "_default_ports", lambda: []), \
                mock.patch("builtins.input", side_effect=["2C23T", "INSTALL"]), \
                redirect_stdout(io.StringIO()), redirect_stderr(err):
            self.assertEqual(cli.main(["update", "--first-install", "--image", str(img)]),
                             cli.EXIT_DEVICE_ERROR)
        self.assertIn("model is not confirmed", err.getvalue())

    def test_stock_image_refused_unless_listed(self):
        img = self.tmp / STOCK_NAME
        img.write_bytes(stock_image())
        err = io.StringIO()
        with redirect_stdout(io.StringIO()), redirect_stderr(err):
            self.assertEqual(cli.main(["update", "--stock-image", str(img)]), cli.EXIT_DEVICE_ERROR)
        self.assertIn("allow-list", err.getvalue())

    def test_options_exclude_each_other(self):
        err = io.StringIO()
        with redirect_stdout(io.StringIO()), redirect_stderr(err):
            self.assertEqual(cli.main(["update", "--first-install", "--stock-image", "a.bin"]),
                             cli.EXIT_DEVICE_ERROR)


if __name__ == "__main__":
    unittest.main()
