"""calbackup.py without hardware: a fake unit that speaks the ASCII shell
(our reply format and the release v0.4.1 format) and records every line."""
from __future__ import annotations

import io
import json
import os
import shutil
import struct
import sys
import tempfile
import unittest
import zlib
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime
from pathlib import Path
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from openscope import calbackup as cb  # noqa: E402
from openscope import cli, proto  # noqa: E402
from openscope.device import Cancelled, Device, DeviceError  # noqa: E402
from openscope.link import NoDevice  # noqa: E402

UID_WORDS = (0x0374920E, 0x6E008000, 0x07CC9420)
UID = struct.pack("<3I", *UID_WORDS)
UID_HEX = "0374920E6E00800007CC9420"
WHEN = datetime(2026, 11, 24, 15, 2, 11).astimezone()
FORBIDDEN = ("cal backup", "cal restore", "mem write")


def make_page(seed: int = 1) -> bytes:
    """A page like the unit's: a programmed cal block, a settings tail."""
    page = bytearray(b"\xFF" * cb.CAL_LEN)
    for i in range(0x000, 0x130):
        page[i] = (i * 7 + seed) & 0xFF
    page[0x1FC:0x200] = bytes([0x20, 0x03, 0x20, 0x03])
    return bytes(page)


def make_w25q(n: int) -> bytes:
    """Raw bytes that look like protocol and shell text: a frame start (0xAA),
    a prompt, a reply header, line ends."""
    head = b"\xAA\x08\x00\x00\x08> FLASHDUMP 4096\r\nOK\r\n> \xAA\x55"
    body = bytes((i * 31 + (i >> 8)) & 0xFF for i in range(n))
    return (head + body)[:n]


class FakeClock:
    """Replaces `time` in device.py: each look at the clock moves it 20 ms,
    so the waits of the shell reader end without real delay."""

    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        self.t += 0.02
        return self.t

    def sleep(self, s):
        self.t += s


class FakeUnit:
    """Link stand-in. fmt "ours" = this branch's shell text, "v041" = release
    v0.4.1. binary=False: the binary protocol gets no reply (v0.4.1)."""

    def __init__(self, page=None, fmt="ours", binary=True, w25q=None, w25q_base=cb.W25Q_START):
        self.page = bytearray(page if page is not None else make_page())
        self.fmt, self.binary = fmt, binary
        self.w25q, self.w25q_base = w25q, w25q_base
        self.port = "FAKE7"
        self.lines = []                 # every shell line the unit received
        self.frames = []                # every binary command byte it received
        self.rx = bytearray()
        self.linebuf = bytearray()
        self.status_crc = None          # None = the true CRC of the page
        self.flip_page_reads = 0        # this many page chunks come back with one changed byte
        self.silent_dumps = 0           # this many `flash dump` commands get no reply (#39 wedge)
        self.bad_dumps = 0              # this many `flash dump` replies have one changed byte
        self.short_dumps = 0            # this many replies stop early
        self.lose_port_at = None        # `flash dump` at this address: the port is lost for good
        self.lost = False
        self.reopens = 0
        self.closed = False

    # device side ------------------------------------------------------
    def _words(self, addr, n):
        if addr == cb.UID_ADDR:
            data = UID
        else:
            ofs = addr - cb.CAL_ADDR
            data = bytes(self.page[ofs:ofs + 4 * n])
            if self.flip_page_reads:
                self.flip_page_reads -= 1
                data = bytes([data[0] ^ 0x01]) + data[1:]
        words = struct.unpack("<%dI" % n, data[:4 * n])
        out = ""
        for i, w in enumerate(words):               # usb_debug.c cmd_mem_read()
            if i % 4 == 0:
                out += "0x%08X:" % (addr + 4 * i)
            out += " %08X" % w
            if i % 4 == 3 or i == n - 1:
                out += "\r\n"
        return out.encode()

    def _reply(self, line):
        p = line.split()
        if line == "version":
            text = "OpenScope 2C53T\r\nBuild: Nov 24 2026 22:11:53, gcc 13.2.1 20231009\r\n"
            if self.fmt == "ours":
                text += ("Git: 9fa1d98  Target: guest-coldtrace  API: 1\r\n"
                         "UID: 0374920E 6E008000 07CC9420\r\n")
            return (text + "MCU: AT32F403A @ 240MHz\r\n").encode()
        if p[:2] == ["mem", "read"]:
            return self._words(int(p[2], 16), int(p[3]))
        if line == "cal status":
            crc = zlib.crc32(bytes(self.page)) if self.status_crc is None else self.status_crc
            if self.fmt == "v041":                  # v0.4.1 usb_debug.c:1798
                return ("live  @0x%08X  %-14s crc=0x%08X\r\nbackup: blank (no valid record)\r\n"
                        % (cb.CAL_ADDR, "programmed", crc)).encode()
            return ("live   @0x%08X  programmed  crc=0x%08X  block 0x025-0x12E programmed "
                    "crc=0x%08X\r\nuid    0374920E 6E008000 07CC9420\r\n"
                    "slot A @0xFFD000: valid v2  uid 0374920E 6E008000 07CC9420 (this unit)\r\n"
                    "         payload_crc=0x11111111  block_crc=0x22222222  block==live: yes"
                    "  page==live: no\r\nslot B @0xFFB000: empty\r\n"
                    % (cb.CAL_ADDR, crc, zlib.crc32(bytes(self.page[0x25:0x12F])))).encode()
        if p[:2] == ["flash", "dump"]:
            addr, n = int(p[2], 16), int(p[3])
            if self.lose_port_at is not None and addr == self.lose_port_at:
                self.lost = True
                return b""
            if self.silent_dumps:
                self.silent_dumps -= 1
                return None                         # the shell went silent: no reply, no prompt
            ofs = addr - self.w25q_base
            data = bytearray(self.w25q[ofs:ofs + n])
            if self.bad_dumps:
                self.bad_dumps -= 1
                data[5] ^= 0x80
            if self.short_dumps:
                self.short_dumps -= 1
                return b"FLASHDUMP %d\r\n" % n + bytes(data[:n - 256]) + b"\r\nERR: flash dump failed (2)\r\n"
            return b"FLASHDUMP %d\r\n" % n + bytes(data)
        return b"Unknown command\r\n"

    def _answer_frame(self, cmd):
        self.frames.append(cmd)
        if not self.binary:
            return
        if cmd == proto.CMD_STATUS:
            fw = b"OpenScope Nov 24 2026 22:11:53 9fa1d98"
            self.rx += proto.encode(proto.RSP_STATUS, proto.STATUS_FIXED.pack(
                proto.PROTO_MAJOR, 0, 80, 0, 4000, 1234, 0, 0, len(fw)) + fw)
        elif cmd == proto.CMD_GET_DESCRIPTION:
            ident = proto.Identity(UID, 1, "guest-coldtrace", "9fa1d98")
            self.rx += proto.encode(proto.RSP_DESCRIPTION, proto.encode_description(
                proto.Description(version=1, identity=ident)))

    # pyserial-ish surface used by Device ------------------------------
    def write(self, data):
        if self.lost:
            raise OSError(22, "A device which does not exist was specified")
        if data[:1] == bytes([proto.SYNC]):
            self._answer_frame(data[1])
            return
        for b in data:
            if b in (0x0D, 0x0A):
                line = bytes(self.linebuf).decode()
                self.linebuf.clear()
                self.lines.append(line)
                reply = self._reply(line) if line else b""
                if reply is None:
                    continue
                self.rx += line.encode() + b"\r\n" + reply + b"> "
            else:
                self.linebuf.append(b)

    def read(self, n=4096):
        if self.lost:
            raise OSError(22, "A device which does not exist was specified")
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def drain(self, quiet=0.15, max_wait=1.5):
        return self.read(1 << 24)

    def discard_input(self):
        if self.lost:
            raise OSError(22, "A device which does not exist was specified")
        self.rx.clear()

    def reopen(self):
        self.reopens += 1
        if self.lost:
            raise NoDevice("device did not come back within 6 s (not found)")

    def open(self):
        pass

    def close(self):
        self.closed = True


class PySerialOver:
    """pyserial stand-in over a FakeUnit, so the REAL SerialLink runs (G-410
    lives in SerialLink: FakeUnit as the link would hide it)."""

    def __init__(self, unit):
        self.unit = unit

    def _check(self):
        if self.unit.lost:
            raise OSError(22, "A device which does not exist was specified")

    def write(self, data):
        self.unit.write(data)
        return len(data)

    @property
    def out_waiting(self):
        self._check()
        return 0

    @property
    def in_waiting(self):
        self._check()
        return len(self.unit.rx)

    def read(self, n):
        return self.unit.read(n)

    def reset_input_buffer(self):
        self.unit.discard_input()

    def close(self):
        pass


class LosesPortInCalRead(FakeUnit):
    """The unit goes into upgrade mode while the cal page is read (flash 2 of
    stage-210): at one `mem read` the port goes away and does not come back."""

    def __init__(self, at_ofs=0x200, **kw):
        super().__init__(**kw)
        self.at = cb.CAL_ADDR + at_ofs

    def _words(self, addr, n):
        if addr == self.at:
            self.lost = True
            return b""
        return super()._words(addr, n)


class Base(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        p = mock.patch("openscope.device.time", self.clock)
        p.start()
        self.addCleanup(p.stop)
        self.tmp = Path(tempfile.mkdtemp(prefix="calbackup-test-"))
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def device(self, unit, check=None):
        dev = Device(unit, timeout=0.3, sleep=self.clock.sleep)
        if unit.binary if check is None else check:
            dev._check_version()
        return dev

    def assert_read_only(self, unit):
        """Every line the unit received is one of the four allowed commands."""
        self.assertTrue(unit.lines)
        for line in unit.lines:
            self.assertTrue(any(line == c or line.startswith(c + " ") for c in cb.ALLOWED), line)
            self.assertFalse(any(line.startswith(f) for f in FORBIDDEN), line)


class TestGuard(Base):
    def test_refuses_every_write_command(self):
        for line in ("cal backup", "cal backup force", "cal restore", "cal restore a CONFIRM",
                     "mem write 0x08006000 0", "mem write 0x20000000 1", "fwapply", "fwload 1 2",
                     "flash wtest", "reboot bootloader", "status", "help", "", " version",
                     "version x", "cal status\rcal backup", "mem read", "MEM READ 0x08006000 64"):
            with self.assertRaises(cb.Refused, msg=line):
                cb.guard(line)

    def test_refuses_reads_outside_the_uid_and_the_cal_page(self):
        for line in ("mem read 0x08006000 65", "mem read 0x08006F04 64", "mem read 0x08005F00 64",
                     "mem read 0x08007000 1", "mem read 0x1FFFF7E8 4", "mem read 0x40021000 1",
                     "mem read 0x08006000 1024", "mem read 0x8006000 64", "mem read 0x08006002 1",
                     "flash dump 0xD00000 4097", "flash dump 0xD00000 0",
                     "flash dump 0xFFF001 4096", "flash dump 0xD00000"):
            with self.assertRaises(cb.Refused, msg=line):
                cb.guard(line)

    def test_passes_the_lines_this_module_sends(self):
        for line in ("version", "cal status", "mem read 0x1FFFF7E8 3", "mem read 0x08006000 64",
                     "mem read 0x08006F00 64", "flash dump 0xD00000 4096", "flash dump 0xFFF000 4096"):
            self.assertEqual(cb.guard(line), line)

    def test_a_refused_line_is_not_sent(self):
        unit = FakeUnit()
        dev = self.device(unit)
        for line in FORBIDDEN:
            with self.assertRaises(cb.Refused):
                cb._shell(dev, line)
        self.assertEqual(unit.lines, [])


class TestBackup(Base):
    def test_backup_saves_the_page_and_the_record(self):
        unit = FakeUnit()
        seen = []
        folder = cb.make_backup(self.device(unit), root=self.tmp, now=WHEN,
                                progress=lambda *a: seen.append(a))
        self.assertEqual(folder, self.tmp / UID_HEX / "2026-11-24_150211")
        self.assertEqual((folder / cb.CAL_FILE).read_bytes(), bytes(unit.page))
        self.assertEqual(sorted(p.name for p in folder.iterdir()), ["backup.json", cb.CAL_FILE])
        info = cb.read_info(folder)
        crc = "%08X" % zlib.crc32(bytes(unit.page))
        self.assertEqual(info["files"][cb.CAL_FILE]["crc32"], crc)
        self.assertEqual(info["files"][cb.CAL_FILE]["size"], 4096)
        self.assertEqual(info["cal"]["cal_status_crc32"], crc)
        self.assertEqual(info["cal"]["block_crc32"], "%08X" % zlib.crc32(bytes(unit.page[0x25:0x12F])))
        self.assertFalse(info["cal"]["block_blank"])
        self.assertEqual(info["unit"]["uid"], UID_HEX)
        self.assertEqual(info["unit"]["usb_serial_from_uid"], "0B41262E6E00")
        self.assertEqual(info["unit"]["port"], "FAKE7")
        self.assertEqual(info["firmware"]["git_rev"], "9fa1d98")
        self.assertEqual(info["firmware"]["target"], "guest-coldtrace")
        self.assertEqual(info["firmware"]["api_level"], 1)
        self.assertIs(info["firmware"]["binary_protocol"], True)
        self.assertIn("Build: Nov 24 2026 22:11:53", info["firmware"]["version_text"])
        self.assertEqual(info["tool"]["name"], "openscope")
        self.assertEqual(info["time"][:19], "2026-11-24T15:02:11")
        self.assertNotIn("w25q", info)
        self.assertEqual(seen[-1], ("cal", 8192, 8192))
        self.assertTrue(cb.verify_backup(folder).ok)
        # the page is read two times: 2 x 16 commands of 64 words (never more per command)
        reads = [x for x in unit.lines if x.startswith("mem read 0x0800")]
        self.assertEqual(len(reads), 32)
        self.assertTrue(all(x.endswith(" 64") for x in reads))
        self.assert_read_only(unit)

    def test_v041_format_and_shell_only(self):
        unit = FakeUnit(fmt="v041", binary=False)
        dev = self.device(unit)
        self.assertFalse(cb.has_binary_protocol(dev))
        folder = cb.make_backup(dev, root=self.tmp, now=WHEN)
        info = cb.read_info(folder)
        self.assertIs(info["firmware"]["binary_protocol"], False)
        self.assertIsNone(info["firmware"]["git_rev"])
        self.assertEqual(info["firmware"]["api_level"], 0)
        self.assertEqual(info["cal"]["cal_status_crc32"], "%08X" % zlib.crc32(bytes(unit.page)))
        self.assertEqual(folder.parent.name, UID_HEX)
        self.assertEqual(unit.frames, [])               # no binary frame was sent
        self.assertTrue(cb.verify_backup(folder).ok)
        self.assert_read_only(unit)

    def test_image_with_no_cal_status_line(self):
        unit = FakeUnit(fmt="v041", binary=False)
        unit._reply_real = unit._reply
        unit._reply = lambda line: b"Unknown command\r\n" if line == "cal status" else unit._reply_real(line)
        folder = cb.make_backup(self.device(unit), root=self.tmp, now=WHEN)
        self.assertIsNone(cb.read_info(folder)["cal"]["cal_status_crc32"])

    def test_two_reads_that_differ_fail(self):
        unit = FakeUnit()
        real = unit._words
        count = [0]

        def unstable(addr, n):                          # byte 1 of the page changes with every command
            count[0] += 1
            unit.page[1] = count[0] & 0xFF
            return real(addr, n)
        unit._words = unstable
        with self.assertRaises(cb.BackupError) as cm:
            cb.make_backup(self.device(unit), root=self.tmp, now=WHEN)
        self.assertIn("two reads differ", str(cm.exception))
        self.assertEqual(list(self.tmp.iterdir()), [])  # nothing was saved
        self.assert_read_only(unit)

    def test_one_bad_read_is_read_again(self):
        unit = FakeUnit()
        good = bytes(unit.page)
        unit.flip_page_reads = 1                        # one chunk of the first read
        dev = self.device(unit)
        self.assertEqual(cb.read_cal_page(dev), good)
        self.assertEqual(len([x for x in unit.lines if x.startswith("mem read")]), 64)

    def serial_device(self, unit, reopen_wait):
        from openscope.link import SerialLink

        def open_():
            raise NoDevice("no OpenScope found on USB 2e3c:5740 (AT32 VCP)")
        link = SerialLink(port="FAKE7", reopen_wait=reopen_wait)
        link._ser = PySerialOver(unit)
        link.open = open_
        dev = Device(link, timeout=0.3, sleep=self.clock.sleep)
        if unit.binary:
            dev._check_version()
        return dev

    def test_port_lost_in_the_cal_read_is_a_backup_error(self):
        """G-410: was AttributeError ('NoneType' has no 'reset_input_buffer')."""
        for binary, fmt in ((True, "ours"), (False, "v041")):
            for wait in (0, 0.3):
                with self.subTest(binary=binary, reopen_wait=wait):
                    unit = LosesPortInCalRead(binary=binary, fmt=fmt)
                    dev = self.serial_device(unit, wait)
                    with mock.patch("openscope.link.time.sleep"):
                        with self.assertRaises(cb.BackupError) as cm:
                            cb.read_cal_page(dev)
                    self.assertIn("cal page read failed two times", str(cm.exception))
                    self.assertFalse(dev.link.is_open)

    def test_port_lost_in_make_backup_saves_nothing(self):
        unit = LosesPortInCalRead()
        dev = self.serial_device(unit, 0)
        with self.assertRaises(cb.BackupError):
            cb.make_backup(dev, root=self.tmp, now=WHEN)
        self.assertEqual(list(self.tmp.iterdir()), [])
        self.assert_read_only(unit)

    def test_cal_status_crc_mismatch_saves_nothing(self):
        unit = FakeUnit()
        unit.status_crc = 0x12345678
        with self.assertRaises(cb.BackupError) as cm:
            cb.make_backup(self.device(unit), root=self.tmp, now=WHEN)
        self.assertIn("cal status", str(cm.exception))
        self.assertEqual(list(self.tmp.iterdir()), [])

    def test_never_overwrites(self):
        unit = FakeUnit()
        folder = cb.make_backup(self.device(unit), root=self.tmp, now=WHEN)
        before = {p.name: p.read_bytes() for p in folder.iterdir()}
        unit.page[0x30] ^= 0xFF                         # the unit changed; same second
        with self.assertRaises(cb.BackupError) as cm:
            cb.make_backup(self.device(unit), root=self.tmp, now=WHEN)
        self.assertIn("not overwritten", str(cm.exception))
        self.assertEqual({p.name: p.read_bytes() for p in folder.iterdir()}, before)
        with self.assertRaises(cb.BackupError):
            cb._write_new(folder / cb.CAL_FILE, b"x")
        self.assertEqual((folder / cb.CAL_FILE).read_bytes(), before[cb.CAL_FILE])

    def test_cancel_before_the_read(self):
        unit = FakeUnit()
        with self.assertRaises(Cancelled):
            cb.make_backup(self.device(unit), root=self.tmp, now=WHEN, cancel=lambda: True)
        self.assertEqual(unit.lines, [])
        self.assertEqual(list(self.tmp.iterdir()), [])

    def test_uid_that_is_not_an_id(self):
        unit = FakeUnit()
        with mock.patch(__name__ + ".UID", b"\xFF" * 12):
            with self.assertRaises(cb.BackupError):
                cb.read_uid(self.device(unit))


class TestVerify(Base):
    def backup(self):
        return cb.make_backup(self.device(FakeUnit()), root=self.tmp, now=WHEN)

    def test_passes_on_a_real_backup(self):
        r = cb.verify_backup(self.backup())
        self.assertTrue(r.ok)
        self.assertEqual(r.problems, ())

    def test_fails_on_a_one_byte_change(self):
        folder = self.backup()
        for ofs in (0x000, 0x030, 0xFFF):
            copy = self.tmp / ("copy%03X" % ofs)
            shutil.copytree(folder, copy)
            data = bytearray((copy / cb.CAL_FILE).read_bytes())
            data[ofs] ^= 0x01
            (copy / cb.CAL_FILE).write_bytes(bytes(data))
            r = cb.verify_backup(copy)
            self.assertFalse(r.ok, hex(ofs))
            self.assertIn("crc32", " ".join(r.problems))
        self.assertTrue(cb.verify_backup(folder).ok)    # the source is not changed

    def test_fails_on_a_missing_or_short_file_or_record(self):
        folder = self.backup()
        a = self.tmp / "a"
        shutil.copytree(folder, a)
        (a / cb.CAL_FILE).write_bytes((a / cb.CAL_FILE).read_bytes()[:-1])
        self.assertFalse(cb.verify_backup(a).ok)
        b = self.tmp / "b"
        shutil.copytree(folder, b)
        (b / cb.CAL_FILE).unlink()
        self.assertFalse(cb.verify_backup(b).ok)
        c = self.tmp / "c"
        shutil.copytree(folder, c)
        (c / cb.JSON_FILE).unlink()
        self.assertFalse(cb.verify_backup(c).ok)
        self.assertFalse(cb.verify_backup(self.tmp / "not-there").ok)

    def test_a_record_cannot_name_a_file_outside_the_folder(self):
        folder = self.backup()
        info = cb.read_info(folder)
        info["files"]["../" + cb.CAL_FILE] = info["files"][cb.CAL_FILE]
        d = self.tmp / "d" / "e"
        d.mkdir(parents=True)
        shutil.copy(folder / cb.CAL_FILE, d / cb.CAL_FILE)
        shutil.copy(folder / cb.CAL_FILE, d.parent / cb.CAL_FILE)
        (d / cb.JSON_FILE).write_text(json.dumps(info))
        r = cb.verify_backup(d)
        self.assertFalse(r.ok)
        self.assertIn("bad file entry", " ".join(r.problems))


class TestCompare(Base):
    def test_equal_pages(self):
        c = cb.compare_cal(make_page(), make_page())
        self.assertTrue(c.block_equal and c.page_equal and c.ok)
        self.assertEqual((c.block_ranges, c.other_ranges, c.warnings), ((), (), ()))

    def test_tail_change_is_block_equal_other_ranges_changed(self):
        a = make_page()
        b = bytearray(a)
        b[0x1FC:0x200] = b"\x00\x00\x00\x00"
        b[0x800] ^= 0xFF
        c = cb.compare_cal(a, bytes(b))
        self.assertTrue(c.block_equal)
        self.assertTrue(c.ok)
        self.assertFalse(c.page_equal)
        self.assertEqual(c.other_ranges, ((0x1FC, 0x1FF), (0x800, 0x800)))
        self.assertEqual(c.warnings, ())
        self.assertIn("block equal, other ranges changed", c.lines()[0])
        self.assertIn("0x1FC-0x1FF", c.lines()[0])

    def test_block_change_is_not_equal(self):
        a = make_page()
        for ofs in (0x025, 0x0A0, 0x12E):               # first, middle and last byte of the block
            b = bytearray(a)
            b[ofs] ^= 0x01
            c = cb.compare_cal(a, bytes(b))
            self.assertFalse(c.block_equal, hex(ofs))
            self.assertFalse(c.ok)
            self.assertEqual(c.block_ranges, ((ofs, ofs),))
            self.assertIn("DIFFERENT", c.lines()[0])

    def test_bytes_next_to_the_block_are_a_warning(self):
        a = make_page()
        for ofs, warned in ((0x12F, True),              # cal_dump.c "CALIBRATION", not in the C-40 block
                            (0x024, False), (0x000, False), (0x130, False)):   # stock settings bytes
            b = bytearray(a)
            b[ofs] ^= 0x01
            c = cb.compare_cal(a, bytes(b))
            self.assertTrue(c.block_equal, hex(ofs))
            self.assertTrue(c.ok, hex(ofs))
            self.assertEqual(c.other_ranges, ((ofs, ofs),))
            self.assertEqual(len(c.warnings), 1 if warned else 0, hex(ofs))
            if warned:
                self.assertIn("next to the cal block", c.warnings[0])
                self.assertIn("warning:", c.lines()[-1])

    def test_blank_block_is_not_ok(self):
        blank = b"\xFF" * cb.CAL_LEN
        c = cb.compare_cal(blank, blank)
        self.assertTrue(c.block_equal)
        self.assertTrue(c.block_blank)
        self.assertFalse(c.ok)
        self.assertIn("blank", c.warnings[0])

    def test_sources_folder_file_bytes(self):
        unit = FakeUnit()
        folder = cb.make_backup(self.device(unit), root=self.tmp, now=WHEN)
        ref = self.tmp / "ref.bin"
        ref.write_bytes(bytes(unit.page))
        self.assertTrue(cb.compare_cal(folder, ref).page_equal)
        self.assertTrue(cb.compare_cal(str(folder), bytes(unit.page)).page_equal)
        short = self.tmp / "short.bin"
        short.write_bytes(b"\x00" * 100)
        for bad in (short, self.tmp / "none.bin", b"\x00" * 4095):
            with self.assertRaises(cb.BackupError):
                cb.compare_cal(folder, bad)


class TestFlashDump(Base):
    N = 3 * 4096

    def unit(self, **kw):
        return FakeUnit(w25q=make_w25q(self.N), **kw)

    def test_raw_bytes_with_0xAA_and_prompt_text(self):
        unit = self.unit()
        dev = self.device(unit)
        data = dev.flash_dump(cb.W25Q_START, 4096)
        self.assertEqual(data, unit.w25q[:4096])
        self.assertIn(0xAA, data)
        self.assertIn(b"> ", data)
        self.assertIn(b"FLASHDUMP 4096\r\n", data)
        self.assertEqual(dev.flash_dump(cb.W25Q_START + 4096, 100), unit.w25q[4096:4196])
        self.assertEqual(dev.shell("version").splitlines()[0], "OpenScope 2C53T")   # the link is clean
        self.assertEqual(dev.status().proto_version, 1)

    def test_length_and_address_are_checked_before_anything_is_sent(self):
        unit = self.unit()
        dev = self.device(unit)
        for addr, n in ((0, 0), (0, 4097), (0xFFF001, 4096), (-1, 16)):
            with self.assertRaises(ValueError):
                dev.flash_dump(addr, n)
        self.assertEqual(unit.lines, [])

    def test_no_reply_and_short_reply_are_errors(self):
        unit = self.unit()
        dev = self.device(unit)
        unit.silent_dumps = 1
        with self.assertRaises(DeviceError):
            dev.flash_dump(cb.W25Q_START, 4096, timeout=0.5, header_timeout=0.3)
        unit.rx += b"> "                                # the shell comes back
        unit.short_dumps = 1
        with self.assertRaises(DeviceError) as cm:
            dev.flash_dump(cb.W25Q_START, 4096, timeout=0.5)
        self.assertIn("B", str(cm.exception))
        self.assertEqual(dev.flash_dump(cb.W25Q_START, 4096), unit.w25q[:4096])

    def test_read_w25q_reads_each_chunk_two_times(self):
        unit = self.unit()
        seen = []
        data = cb.read_w25q(self.device(unit), cb.W25Q_START, self.N,
                            progress=lambda d, t: seen.append((d, t)))
        self.assertEqual(data, unit.w25q)
        self.assertEqual(unit.lines, [f"flash dump 0x{cb.W25Q_START + i * 4096:06X} 4096"
                                      for i in range(3) for _ in range(2)])
        self.assertEqual(seen, [(4096, self.N), (8192, self.N), (self.N, self.N)])
        unit.lines.clear()
        self.assertEqual(cb.read_w25q(self.device(unit), cb.W25Q_START + 10, 5000, verify=False),
                         unit.w25q[10:5010])
        self.assertEqual(unit.lines, ["flash dump 0xD0000A 4096", "flash dump 0xD0100A 904"])

    def test_read_w25q_retries_a_silent_chunk_and_a_changed_chunk(self):
        unit = self.unit()
        unit.silent_dumps = 2                           # the #39 wedge, two times
        unit.bad_dumps = 1
        unit.short_dumps = 1
        data, retried = cb._read_w25q(self.device(unit), cb.W25Q_START, self.N, True,
                                      None, None, 3, 0.5)
        self.assertEqual(data, unit.w25q)
        self.assertEqual(retried, 3)

    def test_read_w25q_gives_up_after_the_retries(self):
        unit = self.unit()
        unit.silent_dumps = 100
        with self.assertRaises(cb.W25QReadError) as cm:
            cb.read_w25q(self.device(unit), cb.W25Q_START, self.N, retries=3, chunk_timeout=0.5)
        self.assertEqual(cm.exception.next_addr, cb.W25Q_START)
        self.assertEqual(len([x for x in unit.lines if x.startswith("flash dump")]), 4)

    def test_lost_port_names_the_last_good_address(self):
        unit = self.unit()
        unit.lose_port_at = cb.W25Q_START + 8192
        with self.assertRaises(cb.W25QReadError) as cm:
            cb.read_w25q(self.device(unit), cb.W25Q_START, self.N)
        e = cm.exception
        self.assertEqual(e.next_addr, cb.W25Q_START + 8192)
        self.assertEqual(e.data, unit.w25q[:8192])
        self.assertIn("last good address 0xD01FFF", str(e))

    def test_backup_with_w25q(self):
        unit = self.unit()
        with mock.patch.object(cb, "W25Q_LEN", self.N):
            folder = cb.make_backup(self.device(unit), root=self.tmp, w25q=True, now=WHEN)
        self.assertEqual((folder / cb.W25Q_FILE).read_bytes(), unit.w25q)
        self.assertEqual(cb.W25Q_FILE, "w25q_0xD00000.bin")
        info = cb.read_info(folder)
        self.assertEqual(info["files"][cb.W25Q_FILE]["size"], self.N)
        self.assertEqual(info["files"][cb.W25Q_FILE]["crc32"], "%08X" % zlib.crc32(unit.w25q))
        self.assertTrue(info["w25q"]["complete"])
        self.assertEqual(info["w25q"]["retries"], 0)
        self.assertTrue(cb.verify_backup(folder).ok)
        copy = self.tmp / "copy"
        shutil.copytree(folder, copy)
        data = bytearray((copy / cb.W25Q_FILE).read_bytes())
        data[-1] ^= 0x01
        (copy / cb.W25Q_FILE).write_bytes(bytes(data))
        self.assertFalse(cb.verify_backup(copy).ok)
        self.assert_read_only(unit)

    def test_backup_keeps_the_cal_page_when_the_w25q_read_stops(self):
        unit = self.unit()
        unit.lose_port_at = cb.W25Q_START + 4096
        with mock.patch.object(cb, "W25Q_LEN", self.N):
            with self.assertRaises(cb.BackupError) as cm:
                cb.make_backup(self.device(unit), root=self.tmp, w25q=True, now=WHEN)
        folder = cm.exception.folder
        self.assertEqual(folder, self.tmp / UID_HEX / "2026-11-24_150211")
        self.assertEqual((folder / cb.CAL_FILE).read_bytes(), bytes(unit.page))
        self.assertEqual((folder / (cb.W25Q_FILE + ".part")).read_bytes(), unit.w25q[:4096])
        self.assertFalse((folder / cb.W25Q_FILE).exists())
        info = cb.read_info(folder)
        self.assertFalse(info["w25q"]["complete"])
        self.assertEqual(info["w25q"]["next_addr"], "0xD01000")
        r = cb.verify_backup(folder)
        self.assertTrue(r.ok)                           # the cal page is whole
        self.assertIn("not complete", " ".join(r.notes))


class TestOpen(Base):
    def test_falls_back_to_shell_only_with_no_binary_reply(self):
        units = []

        def link(port, reopen_wait=6.0):
            units.append(FakeUnit(fmt="v041", binary=False))
            return units[-1]
        with mock.patch("openscope.device.SerialLink", link):
            dev = cb.open_for_backup("FAKE7")
        self.assertEqual(len(units), 2)
        self.assertTrue(units[0].closed)                # the checked open was closed first
        self.assertEqual(units[0].frames, [proto.CMD_STATUS])
        self.assertEqual(units[1].frames, [])           # the second open sends no frame
        self.assertFalse(cb.has_binary_protocol(dev))
        folder = cb.make_backup(dev, root=self.tmp, now=WHEN)
        self.assertTrue(cb.verify_backup(folder).ok)
        self.assertEqual(units[1].frames, [])

    def test_keeps_the_binary_protocol_when_it_answers(self):
        units = []

        def link(port, reopen_wait=6.0):
            units.append(FakeUnit())
            return units[-1]
        with mock.patch("openscope.device.SerialLink", link):
            dev = cb.open_for_backup()
        self.assertEqual(len(units), 1)
        self.assertTrue(cb.has_binary_protocol(dev))

    def test_no_device_is_not_hidden(self):
        class Absent:
            def __init__(self, port, reopen_wait=6.0):
                pass

            def open(self):
                raise NoDevice("no OpenScope found on USB 2e3c:5740 (AT32 VCP)")
        with mock.patch("openscope.device.SerialLink", Absent):
            with self.assertRaises(NoDevice):
                cb.open_for_backup()


class TestCli(Base):
    def run_cli(self, *argv):
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            code = cli.main(list(argv))
        return code, out.getvalue(), err.getvalue()

    def no_port(self):
        """verify and compare DIR DIR2 must not open a port."""
        def boom(*a, **k):
            raise AssertionError("a port was opened")
        return mock.patch.multiple("openscope.device.Device", open=mock.Mock(side_effect=boom))

    def backup(self, unit=None):
        return cb.make_backup(self.device(unit or FakeUnit()), root=self.tmp, now=WHEN)

    def test_backup_command(self):
        unit = FakeUnit()
        with mock.patch.object(cb, "open_for_backup", lambda port: self.device(unit)):
            code, out, err = self.run_cli("backup", "--out", str(self.tmp))
        self.assertEqual(code, 0, err)
        self.assertIn(UID_HEX, out)
        self.assertIn("%08X" % zlib.crc32(bytes(unit.page)), out)
        self.assertTrue(unit.closed)
        folders = list((self.tmp / UID_HEX).iterdir())
        self.assertEqual(len(folders), 1)
        self.assertTrue(cb.verify_backup(folders[0]).ok)
        self.assert_read_only(unit)

    def test_backup_failure_is_exit_2_and_no_device_is_exit_1(self):
        unit = FakeUnit()
        unit.status_crc = 1
        with mock.patch.object(cb, "open_for_backup", lambda port: self.device(unit)):
            code, out, err = self.run_cli("backup", "--out", str(self.tmp))
        self.assertEqual(code, 2)
        self.assertIn("cal status", err)
        self.assertTrue(unit.closed)

        def absent(port):
            raise NoDevice("no OpenScope found on USB 2e3c:5740 (AT32 VCP)")
        with mock.patch.object(cb, "open_for_backup", absent):
            code, out, err = self.run_cli("backup", "--out", str(self.tmp))
        self.assertEqual(code, 1)
        self.assertIn("no OpenScope found", err)

    def test_verify_does_not_open_a_port(self):
        folder = self.backup()
        with self.no_port():
            code, out, err = self.run_cli("backup", "verify", str(folder))
            self.assertEqual(code, 0, err)
            self.assertIn("verified", out)
            data = bytearray((folder / cb.CAL_FILE).read_bytes())
            data[7] ^= 0x10
            copy = self.tmp / "copy"
            shutil.copytree(folder, copy)
            (copy / cb.CAL_FILE).write_bytes(bytes(data))
            code, out, err = self.run_cli("backup", "verify", str(copy))
            self.assertEqual(code, 2)
            self.assertIn("crc32", err)

    def test_compare_two_folders_does_not_open_a_port(self):
        a = self.backup()
        unit = FakeUnit()
        unit.page[0x1FC] ^= 0xFF
        b = cb.make_backup(self.device(unit), root=self.tmp / "b", now=WHEN)
        unit.page[0x40] ^= 0xFF
        c = cb.make_backup(self.device(unit), root=self.tmp / "c", now=WHEN)
        with self.no_port():
            code, out, err = self.run_cli("backup", "compare", str(a), str(b))
            self.assertEqual(code, 0, err)
            self.assertIn("block equal, other ranges changed", out)
            code, out, err = self.run_cli("backup", "compare", str(a), str(c))
            self.assertEqual(code, 2)
            self.assertIn("DIFFERENT", out)
            code, out, err = self.run_cli("backup", "compare", str(a))
            self.assertEqual(code, 64)                  # DIR2 or --live is needed
            code, out, err = self.run_cli("backup", "compare", str(a), str(b), "--live")
            self.assertEqual(code, 64)

    def test_compare_live(self):
        folder = self.backup()
        unit = FakeUnit()
        with mock.patch.object(cb, "open_for_backup", lambda port: self.device(unit)):
            code, out, err = self.run_cli("backup", "compare", str(folder), "--live")
        self.assertEqual(code, 0, err)
        self.assertIn("byte-identical", out)
        self.assertTrue(unit.closed)
        self.assert_read_only(unit)
        unit2 = FakeUnit()
        unit2.page[0x100] ^= 0x01
        with mock.patch.object(cb, "open_for_backup", lambda port: self.device(unit2)):
            code, out, err = self.run_cli("backup", "compare", str(folder), "--live")
        self.assertEqual(code, 2)

    def test_other_commands_still_open_with_the_check(self):
        calls = []

        def fake_open(port=None, check_version=True, **kw):
            calls.append(check_version)
            return self.device(FakeUnit())
        with mock.patch("openscope.cli.Device.open", fake_open):
            code, out, err = self.run_cli("shell", "version")
        self.assertEqual((code, calls), (0, [True]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
