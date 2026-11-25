"""Firmware update through the factory IAP drive (stage-200, paths A and C;
stage-205: old images, first install from stock, return to stock).

Start states and paths:
  * The unit runs OpenScope with a CDC port (path A). probe_firmware() tells
    "binary" (binary protocol) or "shell_only" (an old image, e.g. release
    v0.4.1: the ASCII shell only). Both:
        check -> backup -> enter upgrade mode (the user) -> find drive -> copy
        -> wait for the port -> verify -> log
    The backup goes over the shell on both (calbackup.make_backup()). On a
    shell-only image no binary request is sent after the probe.
  * The unit is already in upgrade mode, no port (path C, recovery):
        check -> find drive -> backup on disk for this drive's USB serial (or
        the acknowledgment NO_BACKUP_ACK) -> copy -> wait for the port ->
        verify -> log
  * The unit runs the stock FNIRSI firmware (first install, path F,
    run_first_install()). Stock has no command channel, so the cal page
    cannot be read before OpenScope boots:
        check (model, INSTALL) -> enter -> drive (record its USB serial) ->
        copy -> wait for the port -> backup at once, before any key press
        (cal page and W25Q 0xD00000-0xFFFFFF) -> the user restarts the unit
        -> verify (a second cal page read = the first, in the cal block) -> log
  * Back to stock (path S, run_stock_install()): the user's own FNIRSI file,
    whose SHA-256 must be on firmware.STOCK_ALLOW:
        check -> backup -> enter -> drive -> copy -> drive gone (no port is
        expected) -> the user confirms the stock screen -> log

Gates before the copy (any failure refuses; nothing is written):
  * check_image() of firmware.py is empty (path S: check_stock_image()).
  * Paths A and S: a backup made in this run by calbackup.make_backup() that
    passes verify_backup(). Its unit ID must equal the DESCRIPTION section 7
    unit ID; an image with no section 7 (API level 0, shell only) has no
    second source of the ID, so there the USB serial that the port gives must
    equal the serial made from the `mem read` ID (the Artery rule, spec §41).
  * Paths A and S on a binary image: USB power: the STATUS charging flag, or
    a known battery level of at least MIN_BATTERY_PCT. A shell-only image
    does not report it: its CDC port shows that USB is connected.
  * The drive's USB serial equals the backup's USB serial when both are known.
  * Path F: the user confirmed the model (MODEL) and typed FIRST_INSTALL_ACK.

The unit cannot enter upgrade mode by itself: the factory bootloader reads
the MENU key at reset. The user holds MENU and taps Power (on an OpenScope
image with the input task: input_handler.c resets while MENU is held; bench
2026-11-24, 2 of 2; and on stock, spec §41), or holds MENU through a pinhole
reset (corrections C-20) and keeps MENU held 3-5 s. The run reports a "wait"
event and polls for the drive. cancel() can stop the run at any time before
the copy; after the copy starts it is not checked.

The update log (update_<time>.json) goes into the backup folder of the run.
"""
from __future__ import annotations

import hashlib
import json
import os
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple

from . import __version__, calbackup as cb, firmware as fw, iapdrive, proto
from .device import Cancelled, Device, DeviceError, Nak, Timeout
from .link import AT32_VCP, NoDevice

MIN_BATTERY_PCT = 30               # ASSUMPTION: enough for a 15 s copy on battery
NO_BACKUP_ACK = "I HAVE NO BACKUP"
FIRST_INSTALL_ACK = "INSTALL"
MODEL = "2C53T"
STEPS = ("check", "backup", "enter", "drive", "copy", "wait", "verify", "log")     # path A
PATH_STEPS = {"A": STEPS,
              "C": ("check", "drive", "backup", "copy", "wait", "verify", "log"),
              "F": ("check", "enter", "drive", "copy", "wait", "backup", "restart", "verify", "log"),
              "S": ("check", "backup", "enter", "drive", "copy", "wait", "confirm", "log")}
LOG_PREFIX = "update_"
BINARY, SHELL_ONLY, NONE = "binary", "shell_only", "none"
ENTER_TEXT = ("Put the unit in upgrade mode: keep USB connected, hold MENU and tap Power, keep MENU "
              "held until the upgrade screen shows. If it does not show: hold MENU, press the "
              "pinhole reset, keep MENU held 3-5 s. Do not open the drive in Explorer.")
NO_KEYS_TEXT = ("After the copy, OpenScope starts. Do not press a key and do not turn the unit off "
                "until the tool says that the backup is done.")
STOCK_FILES_TEXT = ("The first OpenScope save can need the W25Q area 0xD00000-0xFFFFFF. The tool "
                    "backs it up, but it does not put stock files back: before you continue, copy "
                    "any stock files that you want to keep (screenshots, saved waveforms) from the "
                    "USB Sharing drive of the stock firmware to your computer.")
STOCK_ENTER_TEXT = ("Put the unit in upgrade mode: keep USB connected, hold MENU and tap Power, keep "
                    "MENU held until the upgrade screen shows. Do not open the drive in Explorer. "
                    + NO_KEYS_TEXT)
RESTART_TEXT = ("The backup is done; you can use the keys again. Now restart the unit one time: "
                "press the pinhole reset (or turn the unit off with Power and on again). The tool "
                "waits for the port to go and to come back.")
STOCK_CONFIRM_TEXT = ("Look at the unit: the stock FNIRSI screen must show, and Settings > About "
                      "must show the stock version (1.2.0). The stock firmware has no USB port that "
                      "the tool can check.")
G150_NOTE = ("stock rewrites the cal page header (0x001-0x016) and tail (0x170-0x1FF) after it "
             "boots (G-150); the cal block 0x025-0x12E is not changed. Compare only the cal block "
             "after a later OpenScope install")


class UpdateError(Exception):
    """The update stopped. .step names the step; .log is the log file, if one
    was written; .copied tells whether the image went to the drive."""

    def __init__(self, msg: str, step: str = "", log: Optional[Path] = None, copied: bool = False):
        super().__init__(msg)
        self.step, self.log, self.copied = step, log, copied


class Refused(UpdateError):
    """A gate said no before the copy. Nothing was written to the drive."""


@dataclass(frozen=True)
class Event:
    kind: str                      # "step", "info", "progress", "wait", "done"
    step: str
    text: str
    done: Optional[int] = None
    total: Optional[int] = None


@dataclass
class UpdateResult:
    ok: bool
    path: str                                   # "A", "C", "F" (first install) or "S" (stock)
    log: Optional[Path] = None
    before_backup: Optional[Path] = None
    after_backup: Optional[Path] = None
    problems: List[str] = field(default_factory=list)   # verify differences; [] when ok
    compare_lines: List[str] = field(default_factory=list)
    seconds: float = 0.0
    user_wait_seconds: float = 0.0
    probe: Optional[str] = None                 # BINARY or SHELL_ONLY (paths A, S)


@dataclass
class Hooks:
    """What the runs touch outside themselves. Tests replace these."""
    open_device: Callable[[Optional[str]], Device] = None       # -> Device (binary protocol checked)
    open_backup: Callable[[Optional[str]], Device] = None       # -> Device, shell-only fallback
    ports: Callable[[], List[Tuple[str, Optional[str]]]] = None  # [(port, USB serial)]
    probe: Optional[iapdrive.Probe] = None                       # removable drives
    opener: Callable = open                                      # the copy's file open
    clock: Callable[[], float] = time.monotonic
    sleep: Callable[[float], None] = time.sleep
    now: Callable[[], datetime] = None
    poll: float = 0.5
    enter_timeout: float = 300.0       # the user enters upgrade mode (and restarts, path F)
    settle: float = 1.0                # the drive must stay IAP this long before the copy
    gone_timeout: float = 45.0         # after the copy the drive must go (the unit reboots)
    port_timeout: float = 60.0         # ... and the port must come back (C-160)


def _default_ports() -> List[Tuple[str, Optional[str]]]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return sorted((p.device, p.serial_number) for p in list_ports.comports()
                  if (p.vid, p.pid) == AT32_VCP)


def _default_open(port: Optional[str]) -> Device:
    return Device.open(port, check_version=True)


def _fill(h: Optional[Hooks]) -> Hooks:
    h = h or Hooks()
    if h.open_backup is None:
        # A caller that replaced open_device (a test, the GUI) gets it for the
        # backup too; the default is calbackup's open with the shell fallback.
        h.open_backup = h.open_device if h.open_device is not None else cb.open_for_backup
    if h.open_device is None:
        h.open_device = _default_open
    if h.ports is None:
        h.ports = _default_ports
    if h.now is None:
        h.now = lambda: datetime.now().astimezone()
    return h


def _same_serial(a: Optional[str], b: Optional[str]) -> Optional[bool]:
    """None = not comparable (one is unknown)."""
    if not a or not b:
        return None
    return a.strip().upper() == b.strip().upper()


def power_ok(status) -> Tuple[bool, str]:
    if status.charging:
        return True, "USB power: charging flag set"
    if status.battery_known and status.battery_pct >= MIN_BATTERY_PCT:
        return True, (f"USB power: charging flag clear, battery {status.battery_pct}% "
                      f">= {MIN_BATTERY_PCT}% (ASSUMPTION: a full battery clears the flag)")
    level = f"{status.battery_pct}%" if status.battery_known else "unknown"
    return False, (f"no USB power seen: charging flag clear and battery {level}. Connect the unit "
                   f"to USB (the cable that you use for the update) and try again")


def firmware_kind(dev: Device) -> str:
    """BINARY or SHELL_ONLY for a Device from calbackup.open_for_backup()."""
    return BINARY if cb.has_binary_protocol(dev) else SHELL_ONLY


def probe_firmware(port: Optional[str] = None,
                   open_backup: Optional[Callable[[Optional[str]], Device]] = None) -> str:
    """BINARY (OpenScope with the binary protocol), SHELL_ONLY (an old
    OpenScope image: the ASCII shell answers `version`, the binary protocol
    does not) or NONE (no port, or nothing that answers as OpenScope).
    Read only: the STATUS request of the open, then one `version` line on a
    shell-only image."""
    try:
        dev = (open_backup or cb.open_for_backup)(port)
    except NoDevice:
        return NONE
    try:
        kind = firmware_kind(dev)
        if kind == SHELL_ONLY:
            try:
                text = cb._shell(dev, "version")
            except DeviceError:
                return NONE
            if "OpenScope" not in text:
                return NONE
        return kind
    finally:
        dev.close()


def find_backups_for_serial(root: Path, serial: str) -> List[Path]:
    """Backup folders under root whose USB serial is `serial` and that pass
    verify_backup(), newest first."""
    hits = []
    if not serial or not Path(root).is_dir():
        return hits
    for unit in Path(root).iterdir():
        if not unit.is_dir():
            continue
        for folder in unit.iterdir():
            if not (folder / cb.JSON_FILE).is_file():
                continue
            try:
                u = cb.read_info(folder).get("unit", {})
            except cb.BackupError:
                continue
            if any(_same_serial(serial, u.get(k)) for k in ("usb_serial", "usb_serial_from_uid")) \
                    and cb.verify_backup(folder).ok:
                hits.append(folder)
    return sorted(hits, key=lambda p: p.name, reverse=True)


def _identity_or_none(dev: Device) -> Optional[proto.Identity]:
    try:
        return dev.description().identity
    except (Nak, Timeout, proto.ProtocolError):
        return None                     # an image before the description or section 7


def _shell_version_text(folder: Path) -> str:
    return cb.read_info(folder).get("firmware", {}).get("version_text") or ""


class _Run:
    def __init__(self, data, manifest, image_name, hooks, report, cancel):
        self.data, self.manifest, self.image_name = data, manifest, image_name
        self.h = hooks
        self.report = report or (lambda e: None)
        self.cancel = cancel
        self.t0 = hooks.clock()
        self.steps: List[dict] = []
        self.cur: Optional[dict] = None
        self.user_wait = 0.0
        self.copied = False
        self.record: dict = {}

    # bookkeeping --------------------------------------------------------
    def step(self, name: str, text: str) -> None:
        self.end_step("ok")
        self.cur = {"step": name, "start": self.h.now().isoformat(timespec="seconds"),
                    "_t": self.h.clock(), "result": None, "detail": []}
        self.report(Event("step", name, text))

    def end_step(self, result: str) -> None:
        if self.cur is not None:
            self.cur["seconds"] = round(self.h.clock() - self.cur.pop("_t"), 2)
            self.cur["result"] = result
            self.steps.append(self.cur)
            self.cur = None

    def info(self, text: str) -> None:
        if self.cur is not None:
            self.cur["detail"].append(text)
        self.report(Event("info", self.cur["step"] if self.cur else "", text))

    def stop_point(self) -> None:
        if not self.copied and self.cancel is not None and self.cancel():
            raise Cancelled("cancelled by the user before the copy; nothing was written")

    def refuse(self, msg: str) -> None:
        raise Refused(msg, self.cur["step"] if self.cur else "")

    def fail(self, msg: str) -> None:
        raise UpdateError(msg, self.cur["step"] if self.cur else "", copied=self.copied)

    # drive --------------------------------------------------------------
    def wait_for_drive(self, drive: Optional[str], timeout: float, text: str) -> iapdrive.Drive:
        """Poll until an IAP drive stays for `settle` s; then apply every
        refusal of find_iap_drive()."""
        self.report(Event("wait", self.cur["step"], text))
        t_start = self.h.clock()
        seen_at = None
        try:
            while True:
                self.stop_point()
                drives = iapdrive.list_drives(self.h.probe)
                if any(d.kind == iapdrive.IAP for d in drives):
                    if seen_at is None:
                        seen_at = self.h.clock()
                    elif self.h.clock() - seen_at >= self.h.settle:
                        break
                else:
                    seen_at = None
                if self.h.clock() - t_start > timeout:
                    others = "; ".join(d.describe() for d in drives) or "no removable drive"
                    raise UpdateError(f"no IAP drive within {timeout:.0f} s ({others}). Hold MENU, "
                                      "press the pinhole reset, keep MENU held 3-5 s until the "
                                      "upgrade screen shows", self.cur["step"])
                self.h.sleep(self.h.poll)
        finally:
            self.user_wait += self.h.clock() - t_start
        try:
            return iapdrive.find_iap_drive(self.h.probe, drive)
        except iapdrive.DriveError as e:
            self.refuse(str(e))

    def wait_drive_gone(self, drive: iapdrive.Drive) -> float:
        """After the copy: the IAP drive must go (the unit programs the image
        and restarts). Returns the clock at the copy."""
        t_copy = self.h.clock()
        while True:
            if not iapdrive.drive_present(drive.root, self.h.probe):
                break
            if self.h.clock() - t_copy > self.h.gone_timeout:
                still = [d for d in iapdrive.list_drives(self.h.probe) if d.root == drive.root]
                files = ", ".join(still[0].files) if still and still[0].files else "not read"
                raise UpdateError(
                    f"copy did not complete: the IAP drive is still there {self.h.gone_timeout:.0f} s "
                    f"after the copy (files: {files}). Do not write to it. Hold MENU, press the "
                    "pinhole reset to enter upgrade mode again, then run the update again "
                    "(the backup of this run is kept)", "wait", copied=True)
            self.h.sleep(self.h.poll)
        self.info(f"drive gone {self.h.clock() - t_copy:.1f} s after the copy")
        return t_copy

    def wait_for_port(self, serial: Optional[str], t_ref: float, timeout: float, what: str,
                      user: bool = False) -> Tuple[str, Optional[str]]:
        t_start = self.h.clock()
        try:
            while True:
                mine = [(p, s) for p, s in self.h.ports() if _same_serial(s, serial) is not False]
                if mine:
                    self.info(f"port {mine[0][0]} back {self.h.clock() - t_ref:.1f} s after {what}")
                    return mine[0]
                if self.h.clock() - t_ref > timeout:
                    return None
                self.h.sleep(self.h.poll)
        finally:
            if user:
                self.user_wait += self.h.clock() - t_start

    def wait_after_copy(self, drive: iapdrive.Drive, serial: Optional[str]) -> Tuple[str, Optional[str]]:
        t_copy = self.wait_drive_gone(drive)
        got = self.wait_for_port(serial, t_copy, self.h.port_timeout, "the copy")
        if got is None:
            raise UpdateError(
                f"no OpenScope port within {self.h.port_timeout:.0f} s of the copy. If the LCD "
                "still shows the upgrade screen: enter upgrade mode again (MENU + pinhole) and "
                "copy again (C-160). If the unit runs but has no port, the image has no USB link",
                "wait", copied=True)
        return got

    def copy(self, drive: iapdrive.Drive) -> None:
        self.stop_point()
        self.report(Event("progress", "copy", f"copying {len(self.data):,} B to {drive.root} "
                          "(do not unplug, do not open the drive)", 0, len(self.data)))
        self.copied = True                  # from here on the run is not cancellable
        t = self.h.clock()
        try:
            path = iapdrive.copy_image(drive, self.data, self.image_name, self.h.opener)
        except iapdrive.DriveError as e:
            raise UpdateError(str(e), "copy", copied=True) from None
        self.info(f"wrote {path} in {self.h.clock() - t:.1f} s (one write, flush, fsync)")
        self.report(Event("progress", "copy", "copied", len(self.data), len(self.data)))

    # backup -------------------------------------------------------------
    def backup_now(self, port_serial, root: Path, w25q: bool = False,
                   power_gate: bool = True) -> Tuple[Path, dict]:
        """Open the port, probe the firmware, back up, and apply the gates.
        Before the copy: a failure refuses. Returns (folder, unit facts)."""
        port, port_ser = port_serial
        dev = self.h.open_backup(port)
        try:
            kind = firmware_kind(dev)
            self.record["probe"] = kind
            ident, fw_version = None, None
            if kind == BINARY:
                st = dev.status()
                fw_version = st.fw_version
                if power_gate:
                    ok, why = power_ok(st)
                    if not ok:
                        self.refuse(why)
                    self.info(why)
                ident = _identity_or_none(dev)
            else:
                self.info("firmware probe: shell only (an OpenScope image with no binary protocol); "
                          "USB power: the CDC port is up, this image does not report the battery")
            try:
                folder = cb.make_backup(
                    dev, root=root, w25q=w25q, now=self.h.now(),
                    progress=lambda s, d, t: self.report(Event("progress", "backup", s, d, t)),
                    cancel=self.cancel if not self.copied else None)
            except cb.BackupError as e:
                if self.copied:
                    raise UpdateError(f"backup failed: {e}", "backup", copied=True) from None
                raise Refused(f"backup failed, nothing will be copied: {e}", "backup") from None
        finally:
            dev.close()
        self.record["before_backup"] = str(folder)
        v = cb.verify_backup(folder)
        info = cb.read_info(folder)
        unit = info["unit"]
        bad = self.fail if self.copied else self.refuse
        if not v.ok:
            bad("the backup does not verify: " + "; ".join(v.problems))
        if ident is not None:
            if unit["uid"] != ident.uid_hex:
                bad(f"backup unit ID {unit['uid']} is not section 7 unit ID {ident.uid_hex}")
            how = f"unit ID {ident.uid_hex} = section 7"
        else:
            # No section 7: the ID comes from `mem read` only. The port's USB
            # serial is made from the same ID (Artery get_serial_num()), so it
            # ties the ID to this USB device and to its IAP drive.
            same = _same_serial(port_ser, unit.get("usb_serial_from_uid"))
            if same is False:
                bad(f"the port's USB serial {port_ser} is not the serial made from the unit ID "
                    f"{unit['uid']} ({unit.get('usb_serial_from_uid')}): the ID read is not this unit's")
            how = (f"unit ID {unit['uid']} from `mem read` (no section 7); "
                   + ("USB serial matches the ID" if same else "port USB serial not known: not compared"))
        if info["cal"].get("block_blank"):
            bad("the cal block of this unit is blank (all 0xFF): stop and ask (no "
                "calibration to protect is not a normal state)")
        serial = unit.get("usb_serial") or port_ser or unit.get("usb_serial_from_uid")
        self.record["before"] = {"port": port, "probe": kind, "fw_version": fw_version,
                                 "version_text": info["firmware"].get("version_text"),
                                 "uid": unit["uid"], "usb_serial": serial,
                                 "target": ident.target if ident else info["firmware"].get("target"),
                                 "git_rev": ident.git_rev if ident else info["firmware"].get("git_rev"),
                                 "api_level": ident.api_level if ident else 0}
        self.info(f"backup {folder} verified, {how}, USB serial {serial}")
        return folder, {"uid": unit["uid"], "serial": serial, "kind": kind}

    # verify -------------------------------------------------------------
    def verify(self, port: str, uid_hex: Optional[str], before: Optional[Path],
               root: Path) -> Tuple[List[str], Optional[Path], List[str]]:
        problems: List[str] = []
        dev = self.h.open_backup(port)
        try:
            kind = firmware_kind(dev)
            if kind == BINARY:
                st = dev.status()
                ident = _identity_or_none(dev)
                if self.manifest:
                    problems += fw.check_running(self.manifest, st.fw_version, ident)
                self.info(f"firmware {st.fw_version!r}" + (f", {ident.target} at {ident.git_rev}, "
                          f"API level {ident.api_level}" if ident else ", no identity"))
                if uid_hex and ident and ident.uid_hex != uid_hex:
                    problems.append(f"unit ID {ident.uid_hex} is not the backed-up unit {uid_hex}")
            after = cb.make_backup(dev, root=root, now=self.h.now())
        finally:
            dev.close()
        info = cb.read_info(after)
        if kind == SHELL_ONLY:
            text = info["firmware"].get("version_text") or ""
            build = fw.read_build_info(text.encode()).build_time
            self.info(f"firmware answers shell only, Build {build or 'not found'}")
            if self.manifest:
                problems += fw.check_running_shell(self.manifest, text)
        if uid_hex and info["unit"]["uid"] != uid_hex:
            problems.append(f"unit ID {info['unit']['uid']} (mem read) is not the backed-up unit {uid_hex}")
        lines: List[str] = []
        v = cb.verify_backup(after)
        if not v.ok:
            problems.append("the after-update backup does not verify: " + "; ".join(v.problems))
        if before is not None:
            c = cb.compare_cal(before, after)
            lines = c.lines()
            self.record["compare"] = _compare_record(c)
            if not c.ok:
                problems.append("cal block changed or blank after the update: " + "; ".join(lines))
        return problems, after, lines

    # log ----------------------------------------------------------------
    def write_log(self, folder: Optional[Path], result: str, error: Optional[str]) -> Optional[Path]:
        self.end_step(result)
        if folder is None:
            return None
        total = self.h.clock() - self.t0
        doc = {"format": 1, "tool": {"name": "openscope", "version": __version__},
               "result": result, "error": error,
               "image": {"name": self.image_name, "size": len(self.data),
                         "sha256": hashlib.sha256(self.data).hexdigest()},
               "manifest": asdict(self.manifest) if self.manifest else None,
               "steps": self.steps,
               "seconds_total": round(total, 1),
               "seconds_user_wait": round(self.user_wait, 1),
               "seconds_without_user_wait": round(total - self.user_wait, 1)}
        doc.update(self.record)
        path = Path(folder) / f"{LOG_PREFIX}{self.h.now().strftime('%Y-%m-%d_%H%M%S')}.json"
        n = 1
        while path.exists():
            path = path.with_name(f"{path.stem}_{n}.json")
            n += 1
        with open(path, "x", encoding="utf-8", newline="\n") as f:
            f.write(json.dumps(doc, indent=2) + "\n")
            f.flush()
            os.fsync(f.fileno())
        return path

    def guarded(self, folder_of: Callable[[], Optional[Path]], body: Callable[[], object]):
        """Run body(); on any exception write the log into folder_of() and
        re-raise (the log path goes into an UpdateError)."""
        try:
            return body()
        except BaseException as e:
            result = "refused" if isinstance(e, Refused) else \
                "cancelled" if isinstance(e, Cancelled) else "failed"
            log = self.write_log(folder_of(), result, str(e))
            if isinstance(e, UpdateError):
                e.log = log
            raise


def _compare_record(c: cb.CalCompare) -> dict:
    return {"block_equal": c.block_equal, "page_equal": c.page_equal,
            "other_ranges": ["0x%03X-0x%03X" % r for r in c.other_ranges],
            "warnings": list(c.warnings)}


def _one_port(run: _Run, port: Optional[str]) -> List[Tuple[str, Optional[str]]]:
    ports = run.h.ports()
    if port is not None:
        ports = [(p, s) for p, s in ports if p.upper() == port.upper()] or [(port, None)]
    if len(ports) > 1:
        run.refuse("more than one OpenScope port (" + ", ".join(p for p, _ in ports)
                   + "): connect one unit only, or give --port")
    return ports


def run_update(data: bytes, manifest: Optional[fw.Manifest], image_name: str, *,
               port: Optional[str] = None, drive: Optional[str] = None,
               backup_root: Optional[Path] = None, allow_unlisted: bool = False,
               no_backup_ack: Optional[str] = None,
               report: Optional[Callable[[Event], None]] = None,
               cancel: Optional[Callable[[], bool]] = None,
               hooks: Optional[Hooks] = None) -> UpdateResult:
    """Run one update (paths A and C). Returns UpdateResult (ok False = the
    copy was made but verify found differences). Raises Refused (nothing
    written), Cancelled (nothing written), UpdateError (see .copied), or
    DeviceError/NoDevice before the copy."""
    h = _fill(hooks)
    root = Path(backup_root) if backup_root is not None else cb.default_root()
    run = _Run(data, manifest, image_name, h, report, cancel)

    run.step("check", "checking the image")
    refusals = fw.check_image(data, manifest, allow_unlisted)
    if refusals:
        run.refuse("the image is refused: " + "; ".join(refusals))
    iapdrive.check_name(image_name)
    run.record["image_checks"] = "passed" + (" (no manifest, --allow-unlisted)" if manifest is None else "")
    ports = _one_port(run, port)
    if ports:
        return _path_a(run, ports[0], drive, root)
    return _path_c(run, drive, root, no_backup_ack)


def _path_a(run: _Run, port_serial, drive, root: Path) -> UpdateResult:
    h = run.h
    run.record["path"] = "A"
    run.step("backup", "backing up the calibration of this unit")
    run.stop_point()
    holder = {"folder": None}

    def body():
        folder, unit = run.backup_now(port_serial, root)
        holder["folder"] = folder
        run.step("enter", "upgrade mode")
        drv = run.wait_for_drive(drive, h.enter_timeout, ENTER_TEXT)
        run.step("drive", "checking the drive")
        _drive_gate(run, drv, unit["serial"])
        run.step("copy", "copying the image")
        run.copy(drv)
        run.step("wait", "waiting for the unit to restart")
        port, _ = run.wait_after_copy(drv, unit["serial"])
        run.step("verify", "checking the firmware and the calibration")
        return folder, run.verify(port, unit["uid"], folder, root)

    folder, (problems, after, lines) = run.guarded(lambda: holder["folder"], body)
    return _finish(run, folder, after, problems, lines, "A")


def _drive_gate(run: _Run, drv: iapdrive.Drive, serial: Optional[str]) -> None:
    same = _same_serial(drv.usb_serial, serial)
    if same is False:
        run.refuse(f"the IAP drive's USB serial {drv.usb_serial} is not this unit's {serial}: "
                   "it is a different unit")
    run.info(drv.describe() + ("" if same else " (drive serial not known: not compared)"))
    run.record["drive"] = drv.describe()


def _path_c(run: _Run, drive, root: Path, ack: Optional[str]) -> UpdateResult:
    h = run.h
    run.record["path"] = "C"
    run.step("drive", "no OpenScope port: looking for a unit in upgrade mode")
    drives = iapdrive.list_drives(h.probe)
    if not any(d.kind == iapdrive.IAP for d in drives):
        raise NoDevice("no OpenScope port and no IAP drive: connect the unit (USB), or put it in "
                       "upgrade mode (hold MENU and tap Power, or hold MENU through a pinhole reset)")
    try:
        drv = iapdrive.find_iap_drive(h.probe, drive)
    except iapdrive.DriveError as e:
        run.refuse(str(e))
    run.info(drv.describe())
    run.record["drive"] = drv.describe()
    run.step("backup", "looking for a backup of this unit")
    found = find_backups_for_serial(root, drv.usb_serial) if drv.usb_serial else []
    folder = found[0] if found else None
    uid = None
    if folder is not None:
        uid = cb.read_info(folder)["unit"]["uid"]
        run.info(f"backup {folder} (unit {uid}) matches USB serial {drv.usb_serial}, verified")
        run.record["before_backup"] = str(folder)
    elif ack != NO_BACKUP_ACK:
        why = ("Windows does not give the drive's USB serial" if not drv.usb_serial
               else f"no verified backup under {root} has USB serial {drv.usb_serial}")
        run.refuse(f"{why}. Without a backup a lost calibration cannot be made again. To go on "
                   f"anyway give the acknowledgment {NO_BACKUP_ACK!r} (CLI --no-backup-ack)")
    else:
        run.info("no backup: the user gave the acknowledgment " + NO_BACKUP_ACK)
        run.record["no_backup_ack"] = True
    holder = {"after": None}

    def body():
        run.step("copy", "copying the image")
        run.copy(drv)
        run.step("wait", "waiting for the unit to restart")
        port, _ = run.wait_after_copy(drv, drv.usb_serial)
        run.step("verify", "checking the firmware and the calibration")
        res = run.verify(port, uid, folder, root)
        holder["after"] = res[1]
        return res

    problems, after, lines = run.guarded(lambda: folder or holder["after"], body)
    return _finish(run, folder or after, after, problems, lines, "C")


# ── first install from stock (path F) ─────────────────────────────────

def run_first_install(data: bytes, manifest: Optional[fw.Manifest], image_name: str, *,
                      model: Optional[str], install_ack: Optional[str],
                      drive: Optional[str] = None, backup_root: Optional[Path] = None,
                      allow_unlisted: bool = False,
                      report: Optional[Callable[[Event], None]] = None,
                      cancel: Optional[Callable[[], bool]] = None,
                      hooks: Optional[Hooks] = None) -> UpdateResult:
    """First OpenScope install on a unit that runs the stock firmware. The
    stock firmware has no command channel, so the backup comes right after
    the first OpenScope boot, before the tool asks for any key press. The
    caller must have told the user STOCK_FILES_TEXT and NO_KEYS_TEXT, and
    passes the model the user confirmed (MODEL) and FIRST_INSTALL_ACK."""
    h = _fill(hooks)
    root = Path(backup_root) if backup_root is not None else cb.default_root()
    run = _Run(data, manifest, image_name, h, report, cancel)
    run.record["path"] = "F"

    run.step("check", "checking the image and the confirmations")
    refusals = fw.check_image(data, manifest, allow_unlisted)
    if refusals:
        run.refuse("the image is refused: " + "; ".join(refusals))
    iapdrive.check_name(image_name)
    if (model or "").strip().upper() != MODEL:
        run.refuse(f"the model is not confirmed: this image is for the FNIRSI {MODEL} only. Look at "
                   f"the label of the unit and confirm {MODEL!r}")
    if install_ack != FIRST_INSTALL_ACK:
        run.refuse(f"the first install is not confirmed: type {FIRST_INSTALL_ACK!r}")
    run.record["model_confirmed"] = MODEL
    if h.ports():
        run.refuse("an OpenScope port is present: this unit runs OpenScope already. Use "
                   "`openscope update` (it backs up first)")
    drives = iapdrive.list_drives(h.probe)
    for d in drives:
        if d.kind == iapdrive.STOCK_SHARING:
            run.info(f"stock USB Sharing drive {d.root}: " + (", ".join(d.files) or "empty")
                     + ". Copy the files that you want to keep before you continue")
    run.info(STOCK_FILES_TEXT)
    holder = {"folder": None}

    def body():
        run.step("enter", "upgrade mode")
        drv = run.wait_for_drive(drive, h.enter_timeout, STOCK_ENTER_TEXT)
        run.step("drive", "checking the drive")
        serial = drv.usb_serial
        run.info(drv.describe() + ("" if serial else
                                   " (Windows gives no USB serial: the port cannot be matched)"))
        run.record["drive"] = drv.describe()
        run.record["drive_usb_serial"] = serial
        found = find_backups_for_serial(root, serial) if serial else []
        reference = found[0] if found else None
        if reference is not None:
            run.info(f"a backup of this unit exists: {reference}; the first backup is compared with it")
            run.record["reference_backup"] = str(reference)
        run.info(NO_KEYS_TEXT)
        run.step("copy", "copying the image")
        run.copy(drv)
        run.step("wait", "waiting for OpenScope to start (do not press a key)")
        port_serial = run.wait_after_copy(drv, serial)
        if _same_serial(port_serial[1], serial) is None:
            run.info("port or drive USB serial not known: not compared")

        run.step("backup", "first backup: cal page and W25Q 0xD00000-0xFFFFFF (do not press a key)")
        folder, unit = run.backup_now(port_serial, root, w25q=True, power_gate=False)
        holder["folder"] = folder
        problems: List[str] = []
        info = cb.read_info(folder)
        if _same_serial(unit["serial"], serial) is False or \
                _same_serial(info["unit"].get("usb_serial_from_uid"), serial) is False:
            problems.append(f"the unit's USB serial ({unit['serial']}, from the ID "
                            f"{info['unit'].get('usb_serial_from_uid')}) is not the IAP drive's {serial}")
        w = info.get("w25q", {})
        if not w.get("complete"):
            problems.append(f"the W25Q backup is not complete: {w.get('error')}")
        run.record["first_reference"] = {"folder": str(folder),
                                         "block_crc32": info["cal"]["block_crc32"],
                                         "cal_crc32": info["files"][cb.CAL_FILE]["crc32"]}
        lines: List[str] = []
        if reference is not None:
            c = cb.compare_cal(reference, folder)
            lines += ["reference: " + x for x in c.lines()]
            run.record["compare_reference"] = _compare_record(c)
            if not c.ok:
                problems.append("cal block differs from the earlier backup of this unit: "
                                + "; ".join(c.lines()))
        else:
            run.info("first backup of this unit: it is the reference (nothing to compare)")

        run.step("restart", "the user restarts the unit")
        run.report(Event("wait", "restart", RESTART_TEXT))
        t = h.clock()
        try:
            while any(p == port_serial[0] or _same_serial(s, serial) for p, s in h.ports()):
                if h.clock() - t > h.enter_timeout:
                    raise UpdateError(f"the port did not go within {h.enter_timeout:.0f} s: restart "
                                      "the unit, then run `openscope backup compare "
                                      f"{folder} --live`", "restart", copied=True)
                h.sleep(h.poll)
        finally:
            run.user_wait += h.clock() - t
        run.info(f"port gone {h.clock() - t:.1f} s after the request")
        back = run.wait_for_port(serial, h.clock(), h.enter_timeout, "the restart", user=True)
        if back is None:
            raise UpdateError(f"no OpenScope port within {h.enter_timeout:.0f} s of the restart",
                              "restart", copied=True)

        run.step("verify", "checking the firmware and the calibration after the restart")
        more, after, vlines = run.verify(back[0], unit["uid"], folder, root)
        return folder, problems + more, after, lines + vlines

    folder, problems, after, lines = run.guarded(lambda: holder["folder"], body)
    return _finish(run, folder, after, problems, lines, "F")


# ── back to stock (path S) ────────────────────────────────────────────

def run_stock_install(data: bytes, image_name: str, *, port: Optional[str] = None,
                      drive: Optional[str] = None, backup_root: Optional[Path] = None,
                      allowed: Optional[dict] = None,
                      confirm: Optional[Callable[[str], bool]] = None,
                      report: Optional[Callable[[Event], None]] = None,
                      cancel: Optional[Callable[[], bool]] = None,
                      hooks: Optional[Hooks] = None) -> UpdateResult:
    """Put the user's own stock FNIRSI image back (SHA-256 on `allowed`,
    default firmware.STOCK_ALLOW). The unit must run OpenScope with a port:
    the backup comes first. After the copy no port is expected; confirm(text)
    asks the user whether the stock screen shows (None = not asked: the
    result is not ok)."""
    h = _fill(hooks)
    root = Path(backup_root) if backup_root is not None else cb.default_root()
    run = _Run(data, None, image_name, h, report, cancel)
    run.record["path"] = "S"

    run.step("check", "checking the stock image")
    refusals = fw.check_stock_image(data, allowed)
    if refusals:
        run.refuse("the stock image is refused: " + "; ".join(refusals))
    iapdrive.check_name(image_name)
    sha = hashlib.sha256(data).hexdigest()
    name = (allowed if allowed is not None else fw.STOCK_ALLOW)[sha]
    run.info(f"stock image on the allow-list: {name}")
    run.record["stock_image"] = name
    run.record["note"] = G150_NOTE
    ports = _one_port(run, port)
    if not ports:
        raise NoDevice("no OpenScope port: the return to stock backs up the unit first, so the "
                       "unit must run OpenScope with USB connected")
    run.step("backup", "backing up the calibration of this unit")
    run.stop_point()
    holder = {"folder": None}

    def body():
        folder, unit = run.backup_now(ports[0], root)
        holder["folder"] = folder
        run.step("enter", "upgrade mode")
        drv = run.wait_for_drive(drive, h.enter_timeout, ENTER_TEXT)
        run.step("drive", "checking the drive")
        _drive_gate(run, drv, unit["serial"])
        run.step("copy", "copying the stock image")
        run.copy(drv)
        run.step("wait", "waiting for the unit to restart (no port is expected)")
        run.wait_drive_gone(drv)
        run.info(G150_NOTE)
        run.step("confirm", "the user checks the stock screen")
        run.report(Event("wait", "confirm", STOCK_CONFIRM_TEXT))
        t = h.clock()
        try:
            yes = confirm(STOCK_CONFIRM_TEXT) if confirm is not None else None
        finally:
            run.user_wait += h.clock() - t
        run.record["user_confirmed_stock"] = yes
        problems = [] if yes else (["the user did not confirm the stock screen"] if yes is False
                                   else ["the stock screen was not confirmed (no one was asked)"])
        run.info("the user confirmed the stock screen" if yes else problems[0])
        return folder, problems

    folder, problems = run.guarded(lambda: holder["folder"], body)
    return _finish(run, folder, None, problems, [], "S")


def _finish(run: _Run, log_folder, after, problems, lines, path) -> UpdateResult:
    run.record["after_backup"] = str(after) if after else None
    run.record["problems"] = problems
    run.step("log", "writing the update log")
    log = run.write_log(log_folder, "ok" if not problems else "verify_failed",
                        "; ".join(problems) or None)
    total = run.h.clock() - run.t0
    res = UpdateResult(ok=not problems, path=path, log=log,
                       before_backup=Path(run.record["before_backup"]) if run.record.get("before_backup") else None,
                       after_backup=after, problems=problems, compare_lines=lines,
                       seconds=round(total, 1), user_wait_seconds=round(run.user_wait, 1),
                       probe=run.record.get("probe"))
    run.report(Event("done", "log", "update complete" if res.ok else "update finished with problems"))
    return res
