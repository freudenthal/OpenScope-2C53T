"""Calibration backup: a checked copy of a unit's calibration. Read only.

The MCU page 0x08006000 (4096 B) holds the unit's calibration. A lost page
cannot be made again, so a backup comes before every firmware update. This
module reads the page through the ASCII shell (it works on images that have
no binary protocol, e.g. release v0.4.1), reads it two times, and saves it
with checksums. It can also save the W25Q regions that OpenScope writes.

Stable API (the firmware update and the GUI call these; keep the names and
the return values):

    open_for_backup(port=None) -> Device
        Opens with the protocol check; with no binary reply it opens shell
        only. has_binary_protocol(dev) tells which.
    make_backup(dev, root=None, w25q=False, progress=None, cancel=None) -> Path
        Returns the new folder <root>/<UID hex>/<YYYY-MM-DD_HHMMSS>/ with
        cal_0x08006000.bin, backup.json and (w25q=True) w25q_0xD00000.bin.
        progress(stage, done, total): stage is "cal" or "w25q", bytes.
        cancel() -> True stops (device.Cancelled). Never overwrites: an
        existing folder or file is a BackupError. A BackupError raised after
        the folder was made has the path in its .folder attribute.
    verify_backup(folder) -> VerifyResult(ok, problems, notes)
        Sizes, CRC32 and SHA-256 of every file against backup.json.
    compare_cal(a, b) -> CalCompare(block_equal, page_equal, block_blank,
                                    block_ranges, other_ranges, warnings)
        a, b: 4096 bytes, a .bin file, or a backup folder. block_equal is the
        cal block 0x025-0x12E byte for byte; other_ranges are the changed
        (first, last) offsets outside it (the stock settings bytes change
        with every flash: block equal + other ranges changed is normal).
        .ok = block equal and not blank. A change of byte 0x12F is a warning.

Only four shell commands are ever sent: ALLOWED. Every line goes through
guard() first. `cal backup`, `cal restore` and `mem write` cannot be sent.
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import struct
import time
import zlib
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, List, Optional, Tuple, Union

from . import __version__, proto
from .device import Cancelled, Device, DeviceError, Nak, Timeout
from .link import NoDevice

ALLOWED = ("version", "mem read", "flash dump", "cal status")

CAL_ADDR, CAL_LEN = 0x08006000, 4096
CAL_BLOCK = (0x025, 0x12E)          # first and last offset of the cal block
CAL_AREA = (0x030, 0x12F)           # firmware cal_dump.c labels this "CALIBRATION":
                                    # byte 0x12F is in it and is not in the cal block
UID_ADDR = 0x1FFFF7E8               # MCU 96-bit unique ID, three words
MEM_WORDS = 64                      # the firmware gives at most 64 words per `mem read`
FLASH_CHUNK = 4096                  # ... and at most 4096 B per `flash dump`
W25Q_SIZE = 0x1000000
W25Q_START, W25Q_LEN = 0xD00000, 0x300000   # the regions OpenScope writes, to the end

CAL_FILE = "cal_0x08006000.bin"
W25Q_FILE = "w25q_0x%06X.bin" % W25Q_START
JSON_FILE = "backup.json"
FORMAT = 1

PageSource = Union[bytes, bytearray, str, "os.PathLike[str]"]


class BackupError(DeviceError):
    """A read or a check failed. `.folder` is set when a folder was made."""
    folder: Optional[Path] = None


class Refused(BackupError):
    """guard(): the line is not a read-only backup command. Nothing was sent."""


class W25QReadError(BackupError):
    """The W25Q read stopped. `.next_addr` is the first address not read
    (the resume point); `.data` holds the good bytes before it."""

    def __init__(self, msg: str, next_addr: int, data: bytes):
        super().__init__(msg)
        self.next_addr, self.data = next_addr, data


# ── the guard ─────────────────────────────────────────────────────────
_LINE = {
    "version": re.compile(r"version\Z"),
    "cal status": re.compile(r"cal status\Z"),
    "mem read": re.compile(r"mem read 0x([0-9A-F]{8}) ([0-9]{1,2})\Z"),
    "flash dump": re.compile(r"flash dump 0x([0-9A-F]{6}) ([0-9]{1,4})\Z"),
}


def _in_range(cmd: str, m) -> bool:
    if cmd == "mem read":
        # `mem read` does not check its address and a bad one can fault the
        # unit: only the UID words and the cal page are read.
        addr, n = int(m.group(1), 16), int(m.group(2))
        if addr == UID_ADDR:
            return 1 <= n <= 3
        return (1 <= n <= MEM_WORDS and addr % 4 == 0
                and CAL_ADDR <= addr and addr + 4 * n <= CAL_ADDR + CAL_LEN)
    if cmd == "flash dump":
        addr, n = int(m.group(1), 16), int(m.group(2))
        return 1 <= n <= FLASH_CHUNK and addr + n <= W25Q_SIZE
    return True


def guard(line: str) -> str:
    """Return `line` if it is one of the ALLOWED read-only commands in the
    exact form this module sends. Any other line raises Refused."""
    for cmd in ALLOWED:
        if line == cmd or line.startswith(cmd + " "):
            m = _LINE[cmd].match(line)
            if m and _in_range(cmd, m):
                return line
            break
    raise Refused(f"refused: '{line}' is not a read-only backup command (nothing was sent)")


def _shell(dev: Device, line: str) -> str:
    return dev.shell(guard(line))


def _stop(cancel: Optional[Callable[[], bool]]) -> None:
    if cancel is not None and cancel():
        raise Cancelled("backup cancelled")


# ── open ──────────────────────────────────────────────────────────────
def open_for_backup(port: Optional[str] = None, reopen_wait: float = 6.0) -> Device:
    """Open the unit for a backup. An image with no binary protocol (release
    v0.4.1) does not answer the protocol check: then open shell only. No
    device at all raises NoDevice."""
    try:
        return Device.open(port, check_version=True, reopen_wait=reopen_wait)
    except (Timeout, Nak, proto.ProtocolError):
        return Device.open(port, check_version=False, reopen_wait=reopen_wait)


def has_binary_protocol(dev: Device) -> bool:
    return getattr(dev, "proto_version", None) is not None


# ── reads ─────────────────────────────────────────────────────────────
_MEMLINE = re.compile(r"0x([0-9A-Fa-f]{8}):((?: [0-9A-Fa-f]{8}){1,4})")
_LIVE_CRC = re.compile(r"^live\s+@0x[0-9A-Fa-f]{8}\b[^\r\n]*?crc=0x([0-9A-Fa-f]{8})", re.M)


def _read_words(dev: Device, addr: int, count: int) -> List[int]:
    text = _shell(dev, "mem read 0x%08X %d" % (addr, count))
    words = {}
    for m in _MEMLINE.finditer(text):
        base = int(m.group(1), 16)
        for i, w in enumerate(m.group(2).split()):
            words[base + 4 * i] = int(w, 16)
    want = [addr + 4 * i for i in range(count)]
    missing = [a for a in want if a not in words]
    if missing:
        raise BackupError(f"mem read 0x{addr:08X} {count}: {len(missing)} words missing "
                          f"(first 0x{missing[0]:08X}); reply starts {text[:60]!r}")
    return [words[a] for a in want]


def uid_hex(uid: bytes) -> str:
    """24 hex digits: the three ID words as `mem read 0x1FFFF7E8 3` prints
    them. The same text as proto.Identity.uid_hex; the backup folder name."""
    return "".join(f"{w:08X}" for w in struct.unpack("<3I", uid))


def read_uid(dev: Device) -> bytes:
    """The MCU unique ID, 12 bytes (each word little-endian)."""
    uid = struct.pack("<3I", *_read_words(dev, UID_ADDR, 3))
    if len(set(uid)) == 1:
        raise BackupError(f"unit ID reads as all 0x{uid[0]:02X}: not a unique ID")
    return uid


def _page_once(dev, progress, cancel, done: int) -> bytes:
    out = bytearray()
    step = 4 * MEM_WORDS
    for addr in range(CAL_ADDR, CAL_ADDR + CAL_LEN, step):
        _stop(cancel)
        out += struct.pack("<%dI" % MEM_WORDS, *_read_words(dev, addr, MEM_WORDS))
        if progress is not None:
            progress(done + len(out), 2 * CAL_LEN)
    return bytes(out)


def read_cal_page(dev: Device, progress: Optional[Callable[[int, int], None]] = None,
                  cancel: Optional[Callable[[], bool]] = None) -> bytes:
    """The cal page, 4096 bytes: two full reads that must be equal. A read
    that fails (the port dropped, a reply was cut) or two reads that differ
    start again from the first chunk, one time."""
    last = ""
    for _ in range(2):
        try:
            a = _page_once(dev, progress, cancel, 0)
            b = _page_once(dev, progress, cancel, CAL_LEN)
        except (Cancelled, Refused):
            raise
        except DeviceError as e:
            last = str(e)
            continue
        if a == b:
            return a
        first = next(i for i in range(CAL_LEN) if a[i] != b[i])
        last = f"two reads differ (first at offset 0x{first:03X})"
    raise BackupError(f"cal page read failed two times: {last}")


def cal_status_crc(dev: Device) -> Optional[int]:
    """The CRC-32 that `cal status` prints for the live page; None when the
    image has no such line."""
    m = _LIVE_CRC.search(_shell(dev, "cal status"))
    return int(m.group(1), 16) if m else None


def _read_w25q(dev, start, length, verify, progress, cancel, retries, chunk_timeout):
    if length < 1 or start < 0 or start + length > W25Q_SIZE:
        raise ValueError(f"W25Q range 0x{start:06X} + {length} is outside the 16 MiB chip")
    out = bytearray()
    retried = 0
    addr, end = start, start + length
    while addr < end:
        n = min(FLASH_CHUNK, end - addr)
        guard("flash dump 0x%06X %d" % (addr, n))
        last = ""
        for attempt in range(1 + retries):
            _stop(cancel)
            retried += 1 if attempt else 0
            try:
                data = dev.flash_dump(addr, n, timeout=chunk_timeout)
                if verify and dev.flash_dump(addr, n, timeout=chunk_timeout) != data:
                    last = "two reads of the chunk differ"
                    continue
                break
            except (NoDevice, OSError) as e:
                lost = str(e)
            except DeviceError as e:
                if "port lost" not in str(e):
                    last = str(e)           # no reply or a short reply: read it again
                    continue
                lost = str(e)
            good = f"last good address 0x{addr - 1:06X}" if addr > start else "nothing read"
            raise W25QReadError(f"W25Q read stopped at 0x{addr:06X}, {good}: {lost}",
                                addr, bytes(out))
        else:
            raise W25QReadError(f"W25Q read failed at 0x{addr:06X} after {retries} retries: {last}",
                                addr, bytes(out))
        out += data
        addr += n
        if progress is not None:
            progress(len(out), length)
    return bytes(out), retried


def read_w25q(dev: Device, start: int = W25Q_START, length: int = W25Q_LEN, verify: bool = True,
              progress: Optional[Callable[[int, int], None]] = None,
              cancel: Optional[Callable[[], bool]] = None,
              retries: int = 3, chunk_timeout: float = 5.0) -> bytes:
    """`length` bytes of the W25Q from `start`, in 4096-byte chunks. With
    verify each chunk is read two times and must be equal. A chunk with no
    reply, a short reply or two different reads is read again, `retries`
    times. A lost port stops the read: W25QReadError names the last good
    address and holds the resume point."""
    return _read_w25q(dev, start, length, verify, progress, cancel, retries, chunk_timeout)[0]


# ── files ─────────────────────────────────────────────────────────────
def default_root() -> Path:
    """%LOCALAPPDATA%\\OpenScope\\backups on Windows; the user data
    directory on other systems."""
    base = os.environ.get("LOCALAPPDATA") or os.environ.get("XDG_DATA_HOME")
    return (Path(base) if base else Path.home() / ".local" / "share") / "OpenScope" / "backups"


def _hex32(v: int) -> str:
    return f"{v & 0xFFFFFFFF:08X}"


def _record(data: bytes) -> dict:
    return {"size": len(data), "crc32": _hex32(zlib.crc32(data)),
            "sha256": hashlib.sha256(data).hexdigest()}


def _block(page: bytes) -> bytes:
    return page[CAL_BLOCK[0]:CAL_BLOCK[1] + 1]


def _write_new(path: Path, data: bytes) -> None:
    try:
        f = open(path, "xb")            # never overwrite
    except FileExistsError:
        raise BackupError(f"file exists, not overwritten: {path}") from None
    with f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())


def _identity(dev: Device) -> Optional[proto.Identity]:
    if not has_binary_protocol(dev):
        return None
    try:
        return dev.description().identity
    except (Nak, Timeout, proto.ProtocolError):
        return None                     # an image before the description or the identity section


def _port_serial(port: Optional[str]) -> Optional[str]:
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            if p.device == port:
                return p.serial_number
    except Exception:
        pass
    return None


def _field(text: str, name: str) -> Optional[str]:
    m = re.search(r"\b%s:[ \t]*(\S+)" % re.escape(name), text)
    return m.group(1) if m else None


def make_backup(dev: Device, root: Optional[PageSource] = None, w25q: bool = False,
                progress: Optional[Callable[[str, int, int], None]] = None,
                cancel: Optional[Callable[[], bool]] = None, *,
                now: Optional[datetime] = None) -> Path:
    """Read, check and save one backup. See the module text for the result."""
    _stop(cancel)
    version = _shell(dev, "version")
    uid = read_uid(dev)
    ident = _identity(dev)
    if ident is not None and ident.uid != uid:
        raise BackupError(f"unit ID from the description ({ident.uid_hex}) is not the ID "
                          f"at 0x{UID_ADDR:08X} ({uid_hex(uid)})")
    page = read_cal_page(dev, None if progress is None else (lambda d, t: progress("cal", d, t)),
                         cancel)
    crc = zlib.crc32(page) & 0xFFFFFFFF
    status_crc = cal_status_crc(dev)
    if status_crc is not None and status_crc != crc:
        raise BackupError(f"cal page CRC32 {_hex32(crc)} is not the `cal status` live CRC "
                          f"{_hex32(status_crc)}: nothing was saved")

    when = now or datetime.now().astimezone()
    folder = Path(root) if root is not None else default_root()
    folder = folder / uid_hex(uid) / when.strftime("%Y-%m-%d_%H%M%S")
    folder.parent.mkdir(parents=True, exist_ok=True)
    try:
        folder.mkdir(exist_ok=False)
    except FileExistsError:
        raise BackupError(f"backup folder exists, not overwritten: {folder}") from None

    w1, w2, w3 = struct.unpack("<3I", uid)
    info = {
        "format": FORMAT,
        "tool": {"name": "openscope", "version": __version__},
        "time": when.isoformat(timespec="seconds"),
        "unit": {"uid": uid_hex(uid),
                 "uid_source": "mem read 0x%08X 3" % UID_ADDR
                               + ("" if ident is None else " = description section 7"),
                 "usb_serial": _port_serial(dev.link.port),
                 "usb_serial_from_uid": f"{(w1 + w3) & 0xFFFFFFFF:08X}{w2 >> 16:04X}",
                 "port": dev.link.port},
        "firmware": {"version_text": version,
                     "git_rev": ident.git_rev if ident else _field(version, "Git"),
                     "target": ident.target if ident else _field(version, "Target"),
                     "api_level": ident.api_level if ident else 0,
                     "binary_protocol": has_binary_protocol(dev)},
        "cal": {"address": "0x%08X" % CAL_ADDR, "reads": 2,
                "block": "0x%03X-0x%03X" % CAL_BLOCK,
                "block_crc32": _hex32(zlib.crc32(_block(page))),
                "block_blank": set(_block(page)) == {0xFF},
                "cal_status_crc32": None if status_crc is None else _hex32(status_crc)},
        "files": {CAL_FILE: _record(page)},
    }
    failure: Optional[BaseException] = None
    try:
        _write_new(folder / CAL_FILE, page)
        if w25q:
            t0 = time.monotonic()
            region = {"start": "0x%06X" % W25Q_START, "length": W25Q_LEN, "reads": 2}
            try:
                data, retried = _read_w25q(
                    dev, W25Q_START, W25Q_LEN, True,
                    None if progress is None else (lambda d, t: progress("w25q", d, t)),
                    cancel, 3, 5.0)
            except (W25QReadError, Cancelled) as e:
                failure = e
                part = getattr(e, "data", b"")
                region.update(complete=False, error=str(e),
                              next_addr="0x%06X" % getattr(e, "next_addr", W25Q_START))
                if part:
                    _write_new(folder / (W25Q_FILE + ".part"), part)
                    region["partial_file"] = W25Q_FILE + ".part"
            else:
                _write_new(folder / W25Q_FILE, data)
                info["files"][W25Q_FILE] = _record(data)
                region.update(complete=True, retries=retried,
                              seconds=round(time.monotonic() - t0, 1))
            info["w25q"] = region
        _write_new(folder / JSON_FILE, (json.dumps(info, indent=2) + "\n").encode("utf-8"))
        check = verify_backup(folder)
        if not check.ok:
            raise BackupError("the saved files do not read back: " + "; ".join(check.problems))
    except BackupError as e:
        e.folder = folder
        raise
    if isinstance(failure, Cancelled):
        raise failure
    if failure is not None:
        e = BackupError(f"{failure}. The cal page is saved and checked in {folder}")
        e.folder = folder
        raise e from None
    return folder


def read_info(folder: PageSource) -> dict:
    """backup.json of a backup folder, as a dict."""
    try:
        with open(Path(folder) / JSON_FILE, "r", encoding="utf-8") as f:
            info = json.load(f)
    except (OSError, ValueError) as e:
        raise BackupError(f"cannot read {Path(folder) / JSON_FILE}: {e}") from None
    if not isinstance(info, dict) or not isinstance(info.get("files"), dict):
        raise BackupError(f"{Path(folder) / JSON_FILE} is not a backup record")
    return info


@dataclass(frozen=True)
class VerifyResult:
    ok: bool
    problems: Tuple[str, ...]       # one line per failed check; empty when ok
    notes: Tuple[str, ...] = ()     # what was checked, and what the backup does not hold


def verify_backup(folder: PageSource) -> VerifyResult:
    """Check every file of a backup folder against its backup.json: size,
    CRC32, SHA-256, and the CRC32 of the cal block."""
    folder = Path(folder)
    try:
        info = read_info(folder)
    except BackupError as e:
        return VerifyResult(False, (str(e),))
    problems: List[str] = []
    notes: List[str] = []
    if info.get("format") != FORMAT:
        problems.append(f"backup.json format {info.get('format')!r}: this tool reads format {FORMAT}")
    files = info["files"]
    if CAL_FILE not in files:
        problems.append(f"backup.json does not list {CAL_FILE}")
    for name, want in files.items():
        if name != os.path.basename(name) or name in ("", ".", "..") or not isinstance(want, dict):
            problems.append(f"backup.json lists a bad file entry {name!r}")
            continue
        try:
            data = (folder / name).read_bytes()
        except OSError as e:
            problems.append(f"{name}: cannot read ({e.strerror or e})")
            continue
        got = _record(data)
        bad = [k for k in ("size", "crc32", "sha256")
               if str(got[k]).lower() != str(want.get(k)).lower()]
        if bad:
            problems.append(f"{name}: " + ", ".join(
                f"{k} is {got[k]}, backup.json says {want.get(k)}" for k in bad))
        else:
            notes.append(f"{name}: {got['size']} B, CRC32 {got['crc32']}, SHA-256 equal")
        if name == CAL_FILE and len(data) == CAL_LEN:
            block = _hex32(zlib.crc32(_block(data)))
            if block.lower() != str(info.get("cal", {}).get("block_crc32")).lower():
                problems.append(f"{name}: cal block CRC32 is {block}, backup.json says "
                                f"{info.get('cal', {}).get('block_crc32')}")
        elif name == CAL_FILE:
            problems.append(f"{name}: {len(data)} B, a cal page is {CAL_LEN} B")
    region = info.get("w25q")
    if isinstance(region, dict) and not region.get("complete"):
        notes.append(f"the W25Q copy is not complete: {region.get('error')}")
    return VerifyResult(not problems, tuple(problems), tuple(notes))


# ── compare ───────────────────────────────────────────────────────────
def _runs(offsets: List[int]) -> Tuple[Tuple[int, int], ...]:
    runs: List[Tuple[int, int]] = []
    for o in offsets:
        if runs and o == runs[-1][1] + 1:
            runs[-1] = (runs[-1][0], o)
        else:
            runs.append((o, o))
    return tuple(runs)


def _ranges_text(runs) -> str:
    n = sum(b - a + 1 for a, b in runs)
    shown = ", ".join(f"0x{a:03X}-0x{b:03X}" for a, b in runs[:8])
    return f"{shown}{', ...' if len(runs) > 8 else ''} ({n} B in {len(runs)} range(s))"


def load_page(src: PageSource) -> bytes:
    """A cal page from 4096 bytes, a .bin file, or a backup folder."""
    if isinstance(src, (bytes, bytearray)):
        data, name = bytes(src), "the page"
    else:
        path = Path(src)
        if path.is_dir():
            path = path / CAL_FILE
        try:
            data = path.read_bytes()
        except OSError as e:
            raise BackupError(f"cannot read {path}: {e.strerror or e}") from None
        name = str(path)
    if len(data) != CAL_LEN:
        raise BackupError(f"{name} is {len(data)} B; a cal page is {CAL_LEN} B")
    return data


@dataclass(frozen=True)
class CalCompare:
    block_equal: bool                           # cal block 0x025-0x12E, byte for byte
    page_equal: bool                            # all 4096 bytes
    block_blank: bool                           # the block is all 0xFF in a or in b
    block_ranges: Tuple[Tuple[int, int], ...]   # changed (first, last) offsets in the block
    other_ranges: Tuple[Tuple[int, int], ...]   # changed (first, last) offsets outside it
    warnings: Tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        """The calibration is the same and is not blank."""
        return self.block_equal and not self.block_blank

    def lines(self) -> List[str]:
        name = "cal block 0x%03X-0x%03X" % CAL_BLOCK
        if self.page_equal:
            out = [f"{name}: equal; the pages are byte-identical"]
        elif self.block_equal:
            out = [f"{name}: block equal, other ranges changed: {_ranges_text(self.other_ranges)}"]
        else:
            out = [f"{name}: DIFFERENT at {_ranges_text(self.block_ranges)}"]
            if self.other_ranges:
                out.append(f"other ranges changed: {_ranges_text(self.other_ranges)}")
        return out + [f"warning: {w}" for w in self.warnings]


def compare_cal(a: PageSource, b: PageSource) -> CalCompare:
    """Compare two cal pages. The page is also the stock settings page and
    changes outside the cal block with every flash, so the block is compared
    byte for byte and the other changes are listed, not failed."""
    pa, pb = load_page(a), load_page(b)
    lo, hi = CAL_BLOCK
    diff = [i for i in range(CAL_LEN) if pa[i] != pb[i]]
    inside = _runs([i for i in diff if lo <= i <= hi])
    outside = _runs([i for i in diff if not lo <= i <= hi])
    blank = set(_block(pa)) == {0xFF} or set(_block(pb)) == {0xFF}
    warnings = []
    if blank:
        warnings.append("the cal block is blank (all 0xFF): this is not a calibration")
    near = _runs([i for i in diff if CAL_AREA[0] <= i <= CAL_AREA[1] and not lo <= i <= hi])
    if near:
        warnings.append("bytes changed next to the cal block, in the area the firmware labels "
                        f"calibration (0x{CAL_AREA[0]:03X}-0x{CAL_AREA[1]:03X}): {_ranges_text(near)}")
    return CalCompare(not inside, not diff, blank, inside, outside, tuple(warnings))
