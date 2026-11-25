"""The factory IAP drive: find it, tell it from other drives, copy one image.

In upgrade mode the 2C53T factory bootloader shows a small FAT drive
(label IAP, 2048-byte sectors, about 8 MB, one file Ready.TXT). It programs
the bytes of a .bin file in the order they arrive, so any other write to the
drive during the copy damages the image. copy_image() makes exactly one
write of the whole file, then flush and fsync; nothing in this module writes
to a drive in any other way.

The stock app's "USB Sharing" drive has the same USB VID/PID and serial but
no label and 4096-byte sectors: it is not upgrade mode.

Windows only for the real probe (stdlib ctypes). The module imports on any
system; tests inject a probe that describes a folder.

    list_drives(probe=None) -> [Drive]           each with .kind and .reasons
    find_iap_drive(probe=None, drive=None) -> Drive    or raises DriveError
    copy_image(drive, data, name, opener=open) -> Path
"""
from __future__ import annotations

import os
import string
import sys
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple

IAP = "IAP"
STOCK_SHARING = "STOCK_SHARING"
OTHER = "OTHER"

IAP_LABEL = "IAP"
IAP_SECTOR = 2048
IAP_DISK_BYTES = 8_386_560        # disk size (spec §41); the FAT volume is 8,347,648 B
IAP_SIZE_TOLERANCE = 0.01
IAP_READY_FILE = "ready.txt"
STOCK_SECTOR = 4096
# Factory IAP strings (archive/factory_iap_bootloader_2C53T.bin). ASSUMPTION:
# the status file is renamed to one of these after a copy.
IAP_STATUS_WORDS = ("ready", "success", "failed", "large", "unkown", "jump")

DEFENDER_HINT = ("Windows refused the write. Microsoft Defender Device Control (or another "
                 "removable-storage policy) can block writes to USB drives: allow the drive, "
                 "or copy on a computer without that policy")


class DriveError(Exception):
    """No usable IAP drive, more than one candidate, or the copy failed."""


@dataclass(frozen=True)
class Drive:
    root: str                          # "E:\\" (or a folder in tests)
    label: Optional[str] = None        # volume label; None = could not read the volume
    filesystem: Optional[str] = None
    total_bytes: Optional[int] = None
    sector_bytes: Optional[int] = None
    files: Tuple[str, ...] = ()        # names in the root folder
    vendor: Optional[str] = None       # SCSI vendor / product text (IOCTL_STORAGE_QUERY_PROPERTY)
    product: Optional[str] = None
    usb_serial: Optional[str] = None   # USB serial number, when Windows gives it
    bus: Optional[str] = None          # "USB" or another bus name
    kind: str = OTHER
    reasons: Tuple[str, ...] = field(default=())

    @property
    def has_media(self) -> bool:
        return self.label is not None or self.total_bytes is not None

    def describe(self) -> str:
        size = f"{self.total_bytes:,} B" if self.total_bytes else "no media"
        sec = f", {self.sector_bytes} B/sector" if self.sector_bytes else ""
        lab = f"label {self.label!r}" if self.label else "no label"
        ser = f", serial {self.usb_serial}" if self.usb_serial else ""
        prod = f", {self.vendor or ''} {self.product or ''}".rstrip() if (self.vendor or self.product) else ""
        return f"{self.root} {self.kind}: {lab}, {size}{sec}{prod}{ser}"


Probe = Callable[[], Sequence[Drive]]


def classify(d: Drive) -> Drive:
    """Set .kind and .reasons. IAP needs every IAP property; one missing
    property makes it OTHER (with the reasons), so a near miss is never used."""
    files = {f.lower() for f in d.files}
    status = [f for f in files if os.path.splitext(f)[0] in IAP_STATUS_WORDS]
    checks = [
        ((d.label or "").upper() == IAP_LABEL, f"label is {d.label!r}, not 'IAP'"),
        (d.sector_bytes == IAP_SECTOR, f"{d.sector_bytes} B/sector, not {IAP_SECTOR}"),
        (d.total_bytes is not None
         and abs(d.total_bytes - IAP_DISK_BYTES) <= IAP_DISK_BYTES * IAP_SIZE_TOLERANCE,
         f"size {d.total_bytes} B, not {IAP_DISK_BYTES:,} B +/- 1 %"),
        (IAP_READY_FILE in files, "no Ready.TXT" + (f" (has {', '.join(sorted(status))})" if status else "")),
    ]
    failed = tuple(why for ok, why in checks if not ok)
    if d.product and "IAP" not in d.product.upper() and not failed:
        failed = (f"product text {d.product!r} has no 'IAP'",)
    if not failed:
        return replace(d, kind=IAP, reasons=("label IAP, 2048 B/sector, size, Ready.TXT",))
    if not d.label and d.sector_bytes == STOCK_SECTOR:
        return replace(d, kind=STOCK_SHARING,
                       reasons=("no label, 4096 B/sector: the stock app's USB Sharing drive",))
    return replace(d, kind=OTHER, reasons=failed)


def list_drives(probe: Optional[Probe] = None) -> List[Drive]:
    """Every removable drive, classified."""
    return [classify(d) for d in (probe or probe_windows)()]


def _norm_root(text: str) -> str:
    t = text.strip()
    if len(t) == 1 and t.isalpha():
        t += ":"
    if len(t) == 2 and t[1] == ":":
        t += "\\"
    return t


def find_iap_drive(probe: Optional[Probe] = None, drive: Optional[str] = None) -> Drive:
    """The one IAP drive. Refuses: none; two or more; with no `drive` named,
    any other removable drive that has media (name the drive to go on)."""
    drives = list_drives(probe)
    iaps = [d for d in drives if d.kind == IAP]
    if drive is not None:
        want = _norm_root(drive).lower()
        hit = [d for d in drives if _norm_root(d.root).lower() == want]
        if not hit:
            raise DriveError(f"{drive} is not a removable drive"
                             + (": " + "; ".join(d.describe() for d in drives) if drives else ""))
        d = hit[0]
        if d.kind != IAP:
            raise DriveError(f"{d.describe()} is not the IAP drive: " + "; ".join(d.reasons))
        if len(iaps) > 1:
            raise DriveError("more than one IAP drive: " + "; ".join(x.describe() for x in iaps)
                             + ". Connect one unit only")
        return d
    if not iaps:
        stock = [d for d in drives if d.kind == STOCK_SHARING]
        if stock:
            raise DriveError(f"{stock[0].root} is the stock app's USB Sharing drive, not upgrade "
                             "mode. Turn the unit off, hold MENU and press Power")
        near = [d for d in drives if d.has_media and (d.label or "").upper() == IAP_LABEL]
        if near:
            raise DriveError(f"{near[0].describe()} looks like the IAP drive but fails a check: "
                             + "; ".join(near[0].reasons))
        raise DriveError("no IAP drive found (the unit is not in upgrade mode)")
    if len(iaps) > 1:
        raise DriveError("more than one IAP drive: " + "; ".join(d.describe() for d in iaps)
                         + ". Connect one unit only")
    others = [d for d in drives if d.kind != IAP and d.has_media]
    if others:
        raise DriveError("another removable drive is connected: "
                         + "; ".join(d.describe() for d in others)
                         + f". Remove it, or name the IAP drive ({iaps[0].root.rstrip(chr(92))})")
    return iaps[0]


def drive_present(root: str, probe: Optional[Probe] = None) -> bool:
    """Is the drive still there? With the real probe this looks only at the
    drive letter mask (GetLogicalDrives): it does not touch the media while
    the factory IAP programs the image."""
    want = _norm_root(root).lower()
    if probe is not None:
        return any(_norm_root(d.root).lower() == want for d in probe())
    if sys.platform != "win32":
        return os.path.exists(root)
    import ctypes
    mask = ctypes.WinDLL("kernel32").GetLogicalDrives()
    idx = ord(want[0]) - ord("a")
    return 0 <= idx < 26 and bool(mask & (1 << idx))


def check_name(name: str) -> str:
    base = os.path.basename(name)
    if base != name or not name or "/" in name or "\\" in name:
        raise DriveError(f"file name {name!r} is not a plain file name")
    if not name.lower().endswith(".bin"):
        raise DriveError(f"file name {name!r} does not end in .bin (the IAP accepts .bin only)")
    try:
        name.encode("ascii")
    except UnicodeEncodeError:
        raise DriveError(f"file name {name!r} is not ASCII") from None
    return name


def copy_image(drive: Drive, data: bytes, name: str, opener=open) -> Path:
    """Write `data` as `name` on the drive: one open, one write call of the
    whole image, flush, fsync, close. No other access writes to the drive."""
    check_name(name)
    if drive.kind != IAP:
        raise DriveError(f"{drive.describe()} is not the IAP drive")
    path = Path(drive.root) / name
    try:
        f = opener(path, "xb", buffering=0)          # unbuffered: the write goes out as one call
    except FileExistsError:
        raise DriveError(f"{path} exists on the drive; leave upgrade mode and enter it again") from None
    except PermissionError as e:
        raise DriveError(f"cannot create {path}: {e}. {DEFENDER_HINT}") from None
    except OSError as e:
        raise DriveError(f"cannot create {path}: {e}") from None
    try:
        with f:
            n = f.write(data)
            if n != len(data):
                raise DriveError(f"the drive took {n} of {len(data)} B in one write; "
                                 "the copy is not complete")
            f.flush()
            os.fsync(f.fileno())
    except PermissionError as e:
        raise DriveError(f"write to {path} failed: {e}. {DEFENDER_HINT}") from None
    except OSError as e:
        raise DriveError(f"write to {path} failed: {e}") from None
    return path


# ── Windows probe ─────────────────────────────────────────────────────

DRIVE_REMOVABLE = 2
BUS_TYPES = {7: "USB", 1: "SCSI", 2: "ATAPI", 3: "ATA", 11: "SATA", 12: "SD", 13: "MMC", 17: "NVMe"}


def probe_windows() -> List[Drive]:
    """Removable drives as Windows reports them. Reads only: volume data,
    the root folder listing, and IOCTL_STORAGE_QUERY_PROPERTY on a handle
    opened with no read or write access."""
    if sys.platform != "win32":
        raise DriveError("the drive probe works on Windows only; give --drive on other systems "
                         "is not supported yet")
    import ctypes
    from ctypes import wintypes

    k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    SEM_FAILCRITICALERRORS = 0x0001
    old = wintypes.DWORD()
    try:                                  # no "insert a disk" dialog for empty card readers
        k32.SetThreadErrorMode(SEM_FAILCRITICALERRORS, ctypes.byref(old))
    except AttributeError:
        pass
    out: List[Drive] = []
    try:
        mask = k32.GetLogicalDrives()
        for i, letter in enumerate(string.ascii_uppercase):
            if not mask & (1 << i):
                continue
            root = f"{letter}:\\"
            if k32.GetDriveTypeW(root) != DRIVE_REMOVABLE:
                continue
            out.append(_probe_one(k32, ctypes, wintypes, letter, root))
    finally:
        try:
            k32.SetThreadErrorMode(old.value, None)
        except AttributeError:
            pass
    return out


def _probe_one(k32, ctypes, wintypes, letter: str, root: str) -> Drive:
    label = fs = None
    vol = ctypes.create_unicode_buffer(261)
    fsn = ctypes.create_unicode_buffer(261)
    if k32.GetVolumeInformationW(root, vol, 261, None, None, None, fsn, 261):
        label, fs = vol.value, fsn.value
    total = sector = None
    spc, bps, nfree, nclus = (wintypes.DWORD() for _ in range(4))
    if k32.GetDiskFreeSpaceW(root, ctypes.byref(spc), ctypes.byref(bps), ctypes.byref(nfree),
                             ctypes.byref(nclus)):
        sector = bps.value
    tot = ctypes.c_ulonglong()
    if k32.GetDiskFreeSpaceExW(root, None, ctypes.byref(tot), None):
        total = tot.value
    files: Tuple[str, ...] = ()
    if label is not None:
        try:
            files = tuple(sorted(os.listdir(root)))
        except OSError:
            pass
    vendor, product, serial, bus = _storage_property(k32, ctypes, wintypes, letter)
    return Drive(root=root, label=label, filesystem=fs, total_bytes=total, sector_bytes=sector,
                 files=files, vendor=vendor, product=product, usb_serial=serial, bus=bus)


def _storage_property(k32, ctypes, wintypes, letter: str):
    """SCSI vendor, product, serial and bus of the disk under a drive letter.
    The handle has access 0 (query only); nothing is read from or written to
    the media."""
    OPEN_EXISTING = 3
    FILE_SHARE_RW = 0x1 | 0x2
    IOCTL_STORAGE_QUERY_PROPERTY = 0x002D1400
    k32.CreateFileW.restype = wintypes.HANDLE
    k32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p,
                                wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
    k32.DeviceIoControl.argtypes = [wintypes.HANDLE, wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD,
                                    ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD),
                                    ctypes.c_void_p]
    k32.CloseHandle.argtypes = [wintypes.HANDLE]
    h = k32.CreateFileW(f"\\\\.\\{letter}:", 0, FILE_SHARE_RW, None, OPEN_EXISTING, 0, None)
    if h is None or h == wintypes.HANDLE(-1).value:
        return None, None, None, None
    try:
        query = (ctypes.c_ubyte * 12)()           # PropertyId 0 (device), QueryType 0 (standard)
        buf = (ctypes.c_ubyte * 1024)()
        got = wintypes.DWORD()
        if not k32.DeviceIoControl(h, IOCTL_STORAGE_QUERY_PROPERTY, query, 12, buf, 1024,
                                   ctypes.byref(got), None):
            return None, None, None, None
        raw = bytes(buf[:got.value])
    finally:
        k32.CloseHandle(h)
    return parse_device_descriptor(raw)


def parse_device_descriptor(raw: bytes):
    """STORAGE_DEVICE_DESCRIPTOR -> (vendor, product, serial, bus)."""
    import struct
    if len(raw) < 36:
        return None, None, None, None
    (ven_off, prod_off, _rev_off, ser_off, bus) = struct.unpack_from("<IIIII", raw, 12)

    def text(off: int) -> Optional[str]:
        if not off or off >= len(raw):
            return None
        end = raw.find(b"\x00", off)
        s = raw[off: end if end >= 0 else len(raw)].decode("ascii", "replace").strip()
        return s or None
    return text(ven_off), text(prod_off), text(ser_off), BUS_TYPES.get(bus, f"bus {bus}")
