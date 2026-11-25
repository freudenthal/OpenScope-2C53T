"""Firmware update (firmware.py, iapdrive.py, update.py) without hardware:
synthetic images, a fake drive probe over a temp folder, and the fake unit of
test_calbackup.py for the whole flow."""
from __future__ import annotations

import hashlib
import io
import json
import os
import shutil
import struct
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime, timedelta
from pathlib import Path
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from openscope import calbackup as cb  # noqa: E402
from openscope import cli, iapdrive, proto  # noqa: E402
from openscope import firmware as fw  # noqa: E402
from openscope import update as up  # noqa: E402
from openscope.device import Cancelled, Device  # noqa: E402
from openscope.link import NoDevice  # noqa: E402

import test_calbackup as tcb  # noqa: E402

SERIAL = "0B41262E6E00"
OLD_FW = "OpenScope Nov 24 2026 22:11:53 9fa1d98"
NEW_TIME, NEW_REV = "Nov 25 2026 10:00:00", "abc1234"


def make_image(size=4096, sp=0x20037FE0, reset=None, build="Nov 25 2026 10:00:00", rev="abc1234",
               target="guest-coldtrace", text=True) -> bytes:
    """A guest image: vector table, the STATUS text, the shell Build line and
    the target string, like the real coldtrace build."""
    reset = (fw.GUEST_BASE + 0x200) | 1 if reset is None else reset
    img = bytearray(b"\xFF" * size)
    img[0:8] = struct.pack("<II", sp, reset)
    if text:
        blob = (b"\x00OpenScope 2C53T\r\nBuild: " + build.encode() + b", gcc 13.2.1\r\n\x00"
                + b"OpenScope " + build.encode() + (b" " + rev.encode() if rev else b"") + b"\x00"
                + (b"\x00" + target.encode() + b"\x00" if target else b""))
        img[0x400:0x400 + len(blob)] = blob
    return bytes(img)


def manifest_for(img: bytes, **kw) -> fw.Manifest:
    kw.setdefault("api_level", 1)
    return fw.make_manifest(img, kw.pop("target", "guest-coldtrace"), **kw)


class TestImageChecks(unittest.TestCase):
    def setUp(self):
        self.img = make_image()
        self.m = manifest_for(self.img)

    def test_good_image_passes(self):
        self.assertEqual(fw.check_image(self.img, self.m), [])

    def test_wrong_hash(self):
        other = make_image(build="Nov 26 2026 10:00:00")
        r = fw.check_image(other, self.m)
        self.assertTrue(any("SHA-256" in x for x in r), r)

    def test_bad_sp(self):
        for sp in (0x10000000, 0x20038004, 0xFFFFFFFF):
            img = make_image(sp=sp)
            r = fw.check_image(img, manifest_for(img))
            self.assertTrue(any("initial SP" in x for x in r), (hex(sp), r))

    def test_bad_reset_vector(self):
        cases = {(0x08004101, "below"), (fw.GUEST_BASE + 4096 + 1, "past the end"),
                 (fw.GUEST_BASE + 0x200, "Thumb"), (0x00000001, "below")}
        for rv, word in cases:
            img = make_image(reset=rv)
            r = fw.check_image(img, manifest_for(img))
            self.assertTrue(any(word in x for x in r), (hex(rv), r))

    def test_too_large(self):
        img = make_image(size=fw.MAX_SIZE + 4)
        r = fw.check_image(img, manifest_for(img))
        self.assertTrue(any("more than 1,017,856" in x for x in r), r)
        img = make_image(size=fw.MAX_SIZE)
        self.assertEqual(fw.check_image(img, manifest_for(img)), [])

    def test_stock_image(self):
        stock = bytearray(b"\x00" * 8192)                        # stock vectors pass the I-10 check
        stock[0:8] = struct.pack("<II", 0x20036F90, 0x08007311)
        sha = hashlib.sha256(bytes(stock)).hexdigest()
        with mock.patch.dict(fw.KNOWN_STOCK, {sha: "FNIRSI stock firmware V1.2.0"}):
            r = fw.check_image(bytes(stock), None, allow_unlisted=True)
        self.assertTrue(any("stock" in x for x in r), r)
        self.assertTrue(any("no 'OpenScope' text" in x for x in r), r)
        self.assertIn("a17c5c35c97bb898f15672a1747bc1041d8ed507c16999ddba0d1e4e2ec0c760", fw.KNOWN_STOCK)

    def test_no_manifest(self):
        r = fw.check_image(self.img, None)
        self.assertTrue(any("no manifest" in x for x in r), r)
        self.assertEqual(fw.check_image(self.img, None, allow_unlisted=True), [])

    def test_no_openscope_text(self):
        img = make_image(text=False)
        r = fw.check_image(img, None, allow_unlisted=True)
        self.assertTrue(any("no 'OpenScope' text" in x for x in r), r)

    def test_target_checks(self):
        img = make_image(target="guest-coldtrace-faulttest")
        r = fw.check_image(img, manifest_for(img, target="guest-coldtrace-faulttest"))
        self.assertTrue(any("test image" in x for x in r), r)
        bad = fw.Manifest(**{**self.m.__dict__, "target": "app"})
        self.assertTrue(any("not a 2C53T guest target" in x for x in fw.check_image(self.img, bad)))
        other = fw.Manifest(**{**self.m.__dict__, "target": "guest-coldtrace-meter"})
        self.assertTrue(any("the image says" in x for x in fw.check_image(self.img, other)))

    def test_size_mismatch(self):
        bad = fw.Manifest(**{**self.m.__dict__, "size": self.m.size + 1})
        self.assertTrue(any("size 4096 B is not" in x for x in fw.check_image(self.img, bad)))


class TestManifest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="update-test-"))
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def test_reads_build_time_and_rev_from_the_image(self):
        m = manifest_for(make_image())
        self.assertEqual((m.build_time, m.git_rev, m.target), (NEW_TIME, NEW_REV, "guest-coldtrace"))
        self.assertEqual(m.fw_version, f"OpenScope {NEW_TIME} {NEW_REV}")
        self.assertEqual(m.version, "guest-coldtrace abc1234")

    def test_no_rev_build(self):
        m = manifest_for(make_image(rev=None, build="Nov 23 2026 09:19:35"), api_level=0)
        self.assertIsNone(m.git_rev)
        self.assertEqual(m.fw_version, "OpenScope Nov 23 2026 09:19:35")

    def test_api_level_from_source(self):
        h = self.tmp / "firmware" / "src" / "drivers"
        h.mkdir(parents=True)
        (h / "esp_comm.h").write_text("#define ESP_RX_GAP_MS 50\n#define ESP_API_LEVEL           7\n")
        self.assertEqual(fw.make_manifest(make_image(), "guest-coldtrace", source=self.tmp).api_level, 7)
        with self.assertRaises(fw.ManifestError):
            fw.make_manifest(make_image(), "guest-coldtrace")       # neither level nor source

    def test_refuses_non_openscope_and_wrong_target(self):
        with self.assertRaises(fw.ManifestError):
            manifest_for(make_image(text=False))
        with self.assertRaises(fw.ManifestError):
            manifest_for(make_image(), target="guest-coldtrace-meter")
        with self.assertRaises(fw.ManifestError):
            manifest_for(make_image(), target="app")

    def test_json_round_trip_and_never_overwrite(self):
        m = manifest_for(make_image(), image_name="x.bin")
        p = self.tmp / "x.bin.manifest.json"
        m.save(p)
        self.assertEqual(fw.Manifest.load(p), m)
        with self.assertRaises(FileExistsError):
            m.save(p)
        self.assertEqual(fw.image_path_for(p, m), self.tmp / "x.bin")
        for text in ("[]", "{}", '{"format": 2}', "not json",
                     json.dumps({**json.loads(m.to_json()), "sha256": "XYZ"})):
            with self.assertRaises(fw.ManifestError, msg=text):
                fw.Manifest.from_json(text)

    def test_check_running(self):
        m = manifest_for(make_image())
        ident = proto.Identity(tcb.UID, 1, "guest-coldtrace", NEW_REV)
        self.assertEqual(fw.check_running(m, m.fw_version, ident), [])
        self.assertTrue(fw.check_running(m, OLD_FW, ident))
        self.assertTrue(fw.check_running(m, m.fw_version, proto.Identity(tcb.UID, 2, "guest-coldtrace", NEW_REV)))
        self.assertTrue(fw.check_running(m, m.fw_version, None))

    def test_cli_manifest(self):
        img = self.tmp / "guest-coldtrace-abc1234.bin"
        img.write_bytes(make_image())
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            self.assertEqual(cli.main(["manifest", str(img), "--target", "guest-coldtrace",
                                       "--api-level", "1"]), cli.EXIT_OK)
            self.assertEqual(cli.main(["manifest", str(img), "--target", "guest-coldtrace",
                                       "--api-level", "1"]), cli.EXIT_DEVICE_ERROR)   # exists
        m = fw.Manifest.load(str(img) + ".manifest.json")
        self.assertEqual((m.image, m.api_level), (img.name, 1))


# ── drives ────────────────────────────────────────────────────────────

def drive(root, kind="iap", **kw) -> iapdrive.Drive:
    base = {"iap": dict(label="IAP", filesystem="FAT", total_bytes=8_347_648, sector_bytes=2048,
                        files=("Ready.TXT",), vendor="AT32", product="MSC_IAP", usb_serial=SERIAL),
            "stock": dict(label="", filesystem="FAT", total_bytes=14_643_200, sector_bytes=4096,
                          files=("LOGO", "Screenshot file"), vendor="AT32", product="Disk0",
                          usb_serial=SERIAL),
            "stick": dict(label="KINGSTON", filesystem="FAT32", total_bytes=16_000_000_000,
                          sector_bytes=512, files=("photo.jpg",), vendor="Kingston", product="DT"),
            "empty": dict()}[kind]
    base.update(kw)
    return iapdrive.Drive(root=str(root), **base)


class TestDrives(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="update-drive-"))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        (self.tmp / "Ready.TXT").write_bytes(b"")

    def find(self, *drives, name=None):
        return iapdrive.find_iap_drive(lambda: list(drives), name)

    def test_classify(self):
        self.assertEqual(iapdrive.classify(drive(self.tmp)).kind, iapdrive.IAP)
        self.assertEqual(iapdrive.classify(drive("F:\\", "stock")).kind, iapdrive.STOCK_SHARING)
        self.assertEqual(iapdrive.classify(drive("G:\\", "stick")).kind, iapdrive.OTHER)
        for change in (dict(label="IAP2"), dict(sector_bytes=512), dict(total_bytes=9_000_000),
                       dict(files=()), dict(files=("Success.TXT",)), dict(product="Disk0")):
            d = iapdrive.classify(drive(self.tmp, **change))
            self.assertEqual(d.kind, iapdrive.OTHER, change)
            self.assertTrue(d.reasons)

    def test_finds_the_one_iap_drive(self):
        self.assertEqual(self.find(drive(self.tmp)).root, str(self.tmp))
        self.assertEqual(self.find(drive(self.tmp), drive("H:\\", "empty")).root, str(self.tmp))

    def test_refuses_no_drive(self):
        with self.assertRaisesRegex(iapdrive.DriveError, "no IAP drive"):
            self.find()
        with self.assertRaisesRegex(iapdrive.DriveError, "no IAP drive"):
            self.find(drive("G:\\", "stick"))

    def test_refuses_stock_sharing(self):
        with self.assertRaisesRegex(iapdrive.DriveError, "USB Sharing"):
            self.find(drive("F:\\", "stock"))
        with self.assertRaisesRegex(iapdrive.DriveError, "not the IAP drive"):
            self.find(drive("F:\\", "stock"), name="F:")

    def test_refuses_two_iap_drives(self):
        with self.assertRaisesRegex(iapdrive.DriveError, "more than one IAP drive"):
            self.find(drive(self.tmp), drive("E:\\"))
        with self.assertRaisesRegex(iapdrive.DriveError, "more than one IAP drive"):
            self.find(drive(self.tmp), drive("E:\\"), name=str(self.tmp))

    def test_refuses_another_removable_drive_unless_named(self):
        with self.assertRaisesRegex(iapdrive.DriveError, "another removable drive"):
            self.find(drive("E:\\"), drive("G:\\", "stick"))
        self.assertEqual(self.find(drive("E:\\"), drive("G:\\", "stick"), name="e").root, "E:\\")
        with self.assertRaisesRegex(iapdrive.DriveError, "not the IAP drive"):
            self.find(drive("E:\\"), drive("G:\\", "stick"), name="G:")

    def test_near_miss_is_named(self):
        with self.assertRaisesRegex(iapdrive.DriveError, "fails a check"):
            self.find(drive(self.tmp, sector_bytes=512))

    def test_copy_is_exactly_one_write(self):
        calls = []

        class Counting:
            def __init__(self, f):
                self.f = f

            def write(self, b):
                calls.append(("write", len(b)))
                return self.f.write(b)

            def flush(self):
                calls.append(("flush",))
                self.f.flush()

            def fileno(self):
                return self.f.fileno()

            def __enter__(self):
                return self

            def __exit__(self, *a):
                calls.append(("close",))
                self.f.close()

        def opener(path, mode, buffering=-1):
            calls.append(("open", Path(path).name, mode, buffering))
            return Counting(open(path, mode, buffering=buffering))

        data = make_image(size=300_000)
        d = iapdrive.classify(drive(self.tmp))
        with mock.patch.object(iapdrive.os, "fsync", side_effect=lambda n: calls.append(("fsync",))):
            iapdrive.copy_image(d, data, "img.bin", opener)
        self.assertEqual(calls, [("open", "img.bin", "xb", 0), ("write", len(data)), ("flush",),
                                 ("fsync",), ("close",)])
        self.assertEqual((self.tmp / "img.bin").read_bytes(), data)
        self.assertEqual(sorted(os.listdir(self.tmp)), ["Ready.TXT", "img.bin"])

    def test_copy_refusals(self):
        d = iapdrive.classify(drive(self.tmp))
        for name in ("img.hex", "sub/img.bin", "..\\img.bin", "", "\u00e9.bin"):
            with self.assertRaises(iapdrive.DriveError, msg=name):
                iapdrive.copy_image(d, b"x", name)
        with self.assertRaises(iapdrive.DriveError):
            iapdrive.copy_image(iapdrive.classify(drive("G:\\", "stick")), b"x", "a.bin")

        def denied(*a, **k):
            raise PermissionError(13, "Access is denied")
        with self.assertRaisesRegex(iapdrive.DriveError, "Device Control"):
            iapdrive.copy_image(d, b"x", "a.bin", denied)
        (self.tmp / "b.bin").write_bytes(b"old")
        with self.assertRaisesRegex(iapdrive.DriveError, "exists"):
            iapdrive.copy_image(d, b"new", "b.bin")
        self.assertEqual((self.tmp / "b.bin").read_bytes(), b"old")

    def test_short_write_is_an_error(self):
        class Short(io.BytesIO):
            def write(self, b):
                return len(b) - 1

            def fileno(self):
                raise AssertionError("no fsync after a short write")
        with self.assertRaisesRegex(iapdrive.DriveError, "not complete"):
            iapdrive.copy_image(iapdrive.classify(drive(self.tmp)), b"abc", "a.bin",
                                lambda *a, **k: Short())

    def test_device_descriptor_parse(self):
        raw = bytearray(64)
        struct.pack_into("<II", raw, 0, 1, 64)
        struct.pack_into("<IIIII", raw, 12, 36, 41, 0, 49, 7)
        raw[36:41] = b"AT32\x00"
        raw[41:49] = b"MSC_IAP\x00"
        raw[49:62] = b"0B41262E6E00\x00"
        self.assertEqual(iapdrive.parse_device_descriptor(bytes(raw)),
                         ("AT32", "MSC_IAP", SERIAL, "USB"))
        self.assertEqual(iapdrive.parse_device_descriptor(b"short"), (None, None, None, None))

    def test_real_probe_is_windows_only_and_imports_everywhere(self):
        with mock.patch.object(iapdrive.sys, "platform", "linux"):
            with self.assertRaises(iapdrive.DriveError):
                iapdrive.probe_windows()

    def test_drive_present_with_probe(self):
        self.assertTrue(iapdrive.drive_present("E:", lambda: [drive("E:\\")]))
        self.assertFalse(iapdrive.drive_present("E:", lambda: [drive("F:\\")]))


# ── the whole flow ────────────────────────────────────────────────────

class UpdUnit(tcb.FakeUnit):
    """The fake unit, with a STATUS text, identity and power flags that the
    test changes (the "flash" changes fw and ident)."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.fw = OLD_FW.encode()
        self.ident = proto.Identity(tcb.UID, 1, "guest-coldtrace", "9fa1d98")
        self.flags, self.pct = proto.FLAG_CHARGING, 80

    def _answer_frame(self, cmd):
        self.frames.append(cmd)
        if cmd == proto.CMD_STATUS:
            self.rx += proto.encode(proto.RSP_STATUS, proto.STATUS_FIXED.pack(
                proto.PROTO_MAJOR, 0, self.pct, self.flags, 4000, 1234, 0, 0, len(self.fw)) + self.fw)
        elif cmd == proto.CMD_GET_DESCRIPTION:
            self.rx += proto.encode(proto.RSP_DESCRIPTION, proto.encode_description(
                proto.Description(version=1, identity=self.ident)))


class Bench:
    """Port list, drive probe and copy for the flow. state: "app" (port),
    "iap" (drive), "off" (neither)."""

    def __init__(self, tmp: Path, unit: UpdUnit, clock):
        self.unit, self.clock = unit, clock
        self.drive_dir = tmp / "drive"
        self.drive_dir.mkdir()
        (self.drive_dir / "Ready.TXT").write_bytes(b"")
        self.state = "app"
        self.copy_completes = True
        self.after_copy = "app"
        self.new_fw, self.new_ident = f"OpenScope {NEW_TIME} {NEW_REV}".encode(), \
            proto.Identity(tcb.UID, 1, "guest-coldtrace", NEW_REV)
        self.extra = []
        self.opened = 0
        self.user_enters = True
        self.events = []
        self.t = datetime(2026, 11, 25, 10, 0, 0).astimezone()

    def ports(self):
        return [("FAKE7", SERIAL)] if self.state == "app" else []

    def probe(self):
        out = list(self.extra)
        if self.state == "iap":
            out.append(drive(self.drive_dir))
        return out

    def open_device(self, port):
        self.opened += 1
        dev = Device(self.unit, timeout=0.3, sleep=self.clock.sleep)
        dev._check_version()
        return dev

    def opener(self, path, mode, buffering=-1):
        bench = self
        f = open(path, mode, buffering=buffering)

        class Wrapped:
            def write(self, b):
                return f.write(b)

            def flush(self):
                f.flush()

            def fileno(self):
                return f.fileno()

            def __enter__(self):
                return self

            def __exit__(self, *a):
                f.close()
                if bench.copy_completes:            # the unit programs the image and restarts
                    bench.state = bench.after_copy
                    bench.unit.fw, bench.unit.ident = bench.new_fw, bench.new_ident
        return Wrapped()

    def report(self, e):
        self.events.append(e)
        if e.kind == "wait" and self.user_enters:   # the user: MENU + pinhole
            self.state = "iap"

    def now(self):
        self.t += timedelta(seconds=1)
        return self.t

    def hooks(self):
        return up.Hooks(open_device=self.open_device, ports=self.ports, probe=self.probe,
                        opener=self.opener, clock=self.clock.monotonic, sleep=self.clock.sleep,
                        now=self.now)

    def copied(self):
        return sorted(os.listdir(self.drive_dir)) != ["Ready.TXT"]


class TestFlow(unittest.TestCase):
    def setUp(self):
        self.clock = tcb.FakeClock()
        p = mock.patch("openscope.device.time", self.clock)
        p.start()
        self.addCleanup(p.stop)
        self.tmp = Path(tempfile.mkdtemp(prefix="update-flow-"))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.unit = UpdUnit()
        self.bench = Bench(self.tmp, self.unit, self.clock)
        self.img = make_image()
        self.m = manifest_for(self.img)
        self.root = self.tmp / "backups"

    def run_update(self, **kw):
        kw.setdefault("backup_root", self.root)
        return up.run_update(kw.pop("data", self.img), kw.pop("manifest", self.m),
                             kw.pop("name", "guest-coldtrace-abc1234.bin"),
                             report=self.bench.report, hooks=self.bench.hooks(), **kw)

    def assert_nothing_copied(self):
        self.assertFalse(self.bench.copied())

    def test_path_a_happy(self):
        r = self.run_update()
        self.assertTrue(r.ok, r.problems)
        self.assertEqual(r.path, "A")
        self.assertEqual((self.bench.drive_dir / "guest-coldtrace-abc1234.bin").read_bytes(), self.img)
        self.assertEqual(r.before_backup.parent, r.after_backup.parent)     # same unit folder
        self.assertEqual(r.before_backup.parent.name, tcb.UID_HEX)
        log = json.loads(r.log.read_text())
        self.assertEqual(r.log.parent, r.before_backup)
        self.assertEqual(log["result"], "ok")
        self.assertEqual([s["step"] for s in log["steps"]], list(up.STEPS))
        self.assertTrue(all("seconds" in s for s in log["steps"]))
        self.assertTrue(log["compare"]["block_equal"])
        self.assertGreater(log["seconds_user_wait"], 0)
        self.assertEqual(log["manifest"]["sha256"], self.m.sha256)
        for line in self.unit.lines:                 # the unit got only read commands (I-40)
            self.assertTrue(any(line == c or line.startswith(c + " ") for c in cb.ALLOWED), line)

    def test_refused_image_opens_nothing(self):
        bad = make_image(sp=0x10000000)
        with self.assertRaises(up.Refused):
            self.run_update(data=bad, manifest=manifest_for(bad))
        self.assertEqual(self.bench.opened, 0)
        self.assert_nothing_copied()

    def test_no_backup_refused(self):
        self.unit.status_crc = 0x12345678           # backup fails its `cal status` check
        with self.assertRaisesRegex(up.Refused, "backup failed"):
            self.run_update()
        self.assertEqual([e for e in self.bench.events if e.kind == "wait"], [])
        self.assert_nothing_copied()

    def test_uid_mismatch_refused(self):
        self.unit.ident = proto.Identity(b"\x01" * 12, 1, "guest-coldtrace", "9fa1d98")
        with self.assertRaises(up.Refused):
            self.run_update()
        self.assert_nothing_copied()

    def test_backup_of_this_unit_is_checked_against_section_7(self):
        real = cb.read_info

        def other_uid(folder):
            info = real(folder)
            info["unit"]["uid"] = "FFFFFFFFFFFFFFFFFFFFFFFF"
            return info
        with mock.patch.object(cb, "read_info", other_uid):
            with self.assertRaisesRegex(up.Refused, "is not section 7 unit ID"):
                self.run_update()
        self.assert_nothing_copied()

    def test_backup_that_does_not_verify_is_refused(self):
        real, calls = cb.verify_backup, []

        def second_fails(folder):                   # make_backup's own check passes; the gate's fails
            calls.append(folder)
            return real(folder) if len(calls) == 1 else                 cb.VerifyResult(False, ("cal_0x08006000.bin: CRC32",))
        with mock.patch.object(cb, "verify_backup", second_fails):
            with self.assertRaisesRegex(up.Refused, "does not verify"):
                self.run_update()
        self.assert_nothing_copied()

    def test_no_usb_power_refused(self):
        self.unit.flags, self.unit.pct = 0, 20
        with self.assertRaisesRegex(up.Refused, "no USB power"):
            self.run_update()
        self.unit.flags, self.unit.pct = proto.FLAG_BATT_UNKNOWN, 90
        with self.assertRaisesRegex(up.Refused, "no USB power"):
            self.run_update()
        self.assert_nothing_copied()
        self.unit.flags, self.unit.pct = 0, 95                   # full battery: accepted
        self.assertTrue(self.run_update().ok)

    def test_api_level_0_uses_the_mem_read_uid(self):
        """No section 7 (an image before stage-190): the unit ID comes from
        `mem read`, tied to the unit by the port's USB serial (stage-205)."""
        self.unit.ident = None

        def no_ident(cmd, _orig=self.unit._answer_frame):
            if cmd == proto.CMD_GET_DESCRIPTION and self.unit.ident is None:   # until the flash
                self.unit.frames.append(cmd)
                self.unit.rx += proto.encode(proto.RSP_DESCRIPTION,
                                             proto.encode_description(proto.Description(version=1)))
                return
            _orig(cmd)
        self.unit._answer_frame = no_ident
        r = self.run_update()
        self.assertTrue(r.ok, r.problems)
        log = json.loads(r.log.read_text())
        self.assertEqual(log["before"]["api_level"], 0)
        self.assertTrue(any("from `mem read`" in d for s in log["steps"] for d in s["detail"]))
        self.unit.ident = None
        self.bench.ports = lambda: [("FAKE7", "FFFF00000000")] if self.bench.state == "app" else []
        with self.assertRaisesRegex(up.Refused, "serial made from the unit ID"):
            self.run_update()

    def test_drive_serial_mismatch_refused(self):
        self.bench.probe = lambda: [drive(self.bench.drive_dir, usb_serial="FFFF00000000")] \
            if self.bench.state == "iap" else []
        with self.assertRaisesRegex(up.Refused, "different unit"):
            self.run_update()
        self.assert_nothing_copied()

    def test_second_removable_drive_refused(self):
        self.bench.extra = [drive("G:\\", "stick")]
        with self.assertRaisesRegex(up.Refused, "another removable drive"):
            self.run_update()
        self.assert_nothing_copied()

    def test_cancel_before_copy(self):
        flag = {"stop": False}
        self.bench.user_enters = False

        def report(e):
            self.bench.events.append(e)
            if e.kind == "wait":
                flag["stop"] = True
        with self.assertRaises(Cancelled):
            up.run_update(self.img, self.m, "a.bin", backup_root=self.root, report=report,
                          cancel=lambda: flag["stop"], hooks=self.bench.hooks())
        self.assert_nothing_copied()

    def test_no_drive_in_time(self):
        self.bench.user_enters = False
        with self.assertRaisesRegex(up.UpdateError, "no IAP drive within"):
            self.run_update()
        self.assert_nothing_copied()

    def test_drive_stays_copy_did_not_complete(self):
        self.bench.copy_completes = False
        with self.assertRaisesRegex(up.UpdateError, "copy did not complete") as cm:
            self.run_update()
        self.assertTrue(cm.exception.copied)
        log = json.loads(cm.exception.log.read_text())
        self.assertEqual(log["result"], "failed")
        self.assertEqual(log["steps"][-1]["step"], "wait")

    def test_verify_mismatch_reported(self):
        self.bench.new_fw = b"OpenScope Nov 24 2026 22:11:53 9fa1d98"     # the old image still runs
        self.bench.new_ident = self.unit.ident
        r = self.run_update()
        self.assertFalse(r.ok)
        self.assertTrue(any("firmware text" in p for p in r.problems), r.problems)
        self.assertEqual(json.loads(r.log.read_text())["result"], "verify_failed")

    def test_cal_change_reported(self):
        def flash_and_change(path, mode, buffering=-1, _o=self.bench.opener):
            self.unit.page[0x40] ^= 0xFF
            return _o(path, mode, buffering)
        self.bench.opener = flash_and_change
        r = self.run_update()
        self.assertFalse(r.ok)
        self.assertTrue(any("cal block changed" in p for p in r.problems), r.problems)

    def test_port_does_not_come_back(self):
        self.bench.after_copy = "off"               # the image runs but has no USB link
        with self.assertRaisesRegex(up.UpdateError, "no OpenScope port within"):
            self.run_update()

    def test_path_c_with_backup_on_disk(self):
        first = self.run_update()                       # path A leaves backups for this unit
        self.assertTrue(first.ok)
        self.bench.state = "iap"
        self.bench.new_fw, self.bench.new_ident = OLD_FW.encode(), \
            proto.Identity(tcb.UID, 1, "guest-coldtrace", "9fa1d98")
        for f in os.listdir(self.bench.drive_dir):
            if f != "Ready.TXT":
                os.remove(self.bench.drive_dir / f)
        old = make_image(build="Nov 24 2026 22:11:53", rev="9fa1d98")
        r = self.run_update(data=old, manifest=manifest_for(old), name="old.bin")
        self.assertTrue(r.ok, r.problems)
        self.assertEqual(r.path, "C")
        self.assertEqual(r.before_backup, first.after_backup)      # newest backup of this serial
        self.assertEqual(r.log.parent, first.after_backup)

    def test_path_c_without_backup(self):
        self.bench.state = "iap"
        with self.assertRaisesRegex(up.Refused, "I HAVE NO BACKUP"):
            self.run_update()
        self.assert_nothing_copied()
        with self.assertRaises(up.Refused):
            self.run_update(no_backup_ack="yes")
        r = self.run_update(no_backup_ack=up.NO_BACKUP_ACK)
        self.assertTrue(r.ok, r.problems)
        self.assertIsNone(r.before_backup)
        self.assertEqual(r.log.parent, r.after_backup)
        self.assertTrue(json.loads(r.log.read_text())["no_backup_ack"])

    def test_no_port_no_drive(self):
        self.bench.state = "off"
        with self.assertRaises(NoDevice):
            self.run_update()

    def test_two_ports_refused(self):
        self.bench.ports = lambda: [("FAKE7", SERIAL), ("FAKE8", "0B41262E6E01")]
        with self.assertRaisesRegex(up.Refused, "more than one OpenScope port"):
            self.run_update()

    def test_cli_update_refused_without_manifest(self):
        img = self.tmp / "x.bin"
        img.write_bytes(self.img)
        err = io.StringIO()
        with redirect_stdout(io.StringIO()), redirect_stderr(err):
            self.assertEqual(cli.main(["update", "--image", str(img)]), cli.EXIT_DEVICE_ERROR)
        self.assertIn("no manifest", err.getvalue())

    def test_update_never_sends_a_write_command(self):
        src = Path(up.__file__).read_text() + Path(iapdrive.__file__).read_text() \
            + Path(fw.__file__).read_text()
        self.assertNotIn(".shell(", src)            # only calbackup's guarded reads touch the shell
        for word in ("cal restore", "mem write", "cal backup", "fwapply", "fwload", "reboot bootloader"):
            self.assertNotIn(word, src, word)


if __name__ == "__main__":
    unittest.main()
