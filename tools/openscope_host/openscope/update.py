"""Firmware update through the factory IAP drive (stage-200, paths A and C).

Path A (the unit runs an OpenScope image with the binary protocol and a
unit identity, API level >= 1):
    check -> backup -> enter upgrade mode (the user) -> find drive -> copy
    -> wait for the port -> verify -> log
Path C (recovery: the unit is already in upgrade mode, no port):
    check -> find drive -> backup on disk for this drive's USB serial (or the
    acknowledgment NO_BACKUP_ACK) -> copy -> wait for the port -> verify -> log

Gates before the copy (any failure refuses; nothing is written):
  * check_image() of firmware.py is empty.
  * Path A: a backup made in this run by calbackup.make_backup() that passes
    verify_backup(), whose unit ID equals the DESCRIPTION section 7 unit ID.
  * Path A: USB power: the STATUS charging flag, or a known battery level of
    at least MIN_BATTERY_PCT. ASSUMPTION: the charging flag follows USB power.
  * The drive's USB serial equals the backup's USB serial when both are known.

The unit cannot enter upgrade mode by itself: the factory bootloader reads
the MENU key at reset. The user holds MENU and taps Power (on an OpenScope
image with the input task: input_handler.c resets while MENU is held; bench
2026-11-24, 2 of 2), or holds MENU through a pinhole reset (corrections C-20)
and keeps MENU held 3-5 s. run_update() reports a "wait"
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

from . import __version__, calbackup as cb, firmware as fw, iapdrive
from .device import Cancelled, Device, DeviceError
from .link import AT32_VCP, NoDevice

MIN_BATTERY_PCT = 30               # ASSUMPTION: enough for a 15 s copy on battery
NO_BACKUP_ACK = "I HAVE NO BACKUP"
STEPS = ("check", "backup", "enter", "drive", "copy", "wait", "verify", "log")
LOG_PREFIX = "update_"
ENTER_TEXT = ("Put the unit in upgrade mode: keep USB connected, hold MENU and tap Power, keep MENU "
              "held until the upgrade screen shows. If it does not show: hold MENU, press the "
              "pinhole reset, keep MENU held 3-5 s. Do not open the drive in Explorer.")


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
    path: str                                   # "A" or "C"
    log: Optional[Path] = None
    before_backup: Optional[Path] = None
    after_backup: Optional[Path] = None
    problems: List[str] = field(default_factory=list)   # verify differences; [] when ok
    compare_lines: List[str] = field(default_factory=list)
    seconds: float = 0.0
    user_wait_seconds: float = 0.0


@dataclass
class Hooks:
    """What run_update() touches outside itself. Tests replace these."""
    open_device: Callable[[Optional[str]], Device] = None       # -> Device (binary protocol checked)
    ports: Callable[[], List[Tuple[str, Optional[str]]]] = None  # [(port, USB serial)]
    probe: Optional[iapdrive.Probe] = None                       # removable drives
    opener: Callable = open                                      # the copy's file open
    clock: Callable[[], float] = time.monotonic
    sleep: Callable[[float], None] = time.sleep
    now: Callable[[], datetime] = None
    poll: float = 0.5
    enter_timeout: float = 300.0       # the user enters upgrade mode
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

    def wait_after_copy(self, drive: iapdrive.Drive, serial: Optional[str]) -> str:
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
        while True:
            ports = self.h.ports()
            mine = [p for p, s in ports if _same_serial(s, serial) is not False]
            if mine:
                self.info(f"port {mine[0]} back {self.h.clock() - t_copy:.1f} s after the copy")
                return mine[0]
            if self.h.clock() - t_copy > self.h.port_timeout:
                raise UpdateError(
                    f"no OpenScope port within {self.h.port_timeout:.0f} s of the copy. If the LCD "
                    "still shows the upgrade screen: enter upgrade mode again (MENU + pinhole) and "
                    "copy again (C-160). If the unit runs but has no port, the image has no USB link",
                    "wait", copied=True)
            self.h.sleep(self.h.poll)

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

    # verify -------------------------------------------------------------
    def verify(self, port: str, uid_hex: Optional[str], before: Optional[Path],
               root: Path) -> Tuple[List[str], Optional[Path], List[str]]:
        problems: List[str] = []
        dev = self.h.open_device(port)
        try:
            st = dev.status()
            ident = dev.description(refresh=True).identity
            problems += fw.check_running(self.manifest, st.fw_version, ident) if self.manifest \
                else []
            self.info(f"firmware {st.fw_version!r}" + (f", {ident.target} at {ident.git_rev}, "
                      f"API level {ident.api_level}" if ident else ", no identity"))
            if uid_hex and ident and ident.uid_hex != uid_hex:
                problems.append(f"unit ID {ident.uid_hex} is not the backed-up unit {uid_hex}")
            after = cb.make_backup(dev, root=root, now=self.h.now())
        finally:
            dev.close()
        lines: List[str] = []
        v = cb.verify_backup(after)
        if not v.ok:
            problems.append("the after-update backup does not verify: " + "; ".join(v.problems))
        if before is not None:
            c = cb.compare_cal(before, after)
            lines = c.lines()
            self.record["compare"] = {"block_equal": c.block_equal, "page_equal": c.page_equal,
                                      "other_ranges": ["0x%03X-0x%03X" % r for r in c.other_ranges],
                                      "warnings": list(c.warnings)}
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


def run_update(data: bytes, manifest: Optional[fw.Manifest], image_name: str, *,
               port: Optional[str] = None, drive: Optional[str] = None,
               backup_root: Optional[Path] = None, allow_unlisted: bool = False,
               no_backup_ack: Optional[str] = None,
               report: Optional[Callable[[Event], None]] = None,
               cancel: Optional[Callable[[], bool]] = None,
               hooks: Optional[Hooks] = None) -> UpdateResult:
    """Run one update. Returns UpdateResult (ok False = the copy was made but
    verify found differences). Raises Refused (nothing written), Cancelled
    (nothing written), UpdateError (see .copied), or DeviceError/NoDevice
    before the copy."""
    h = _fill(hooks)
    root = Path(backup_root) if backup_root is not None else cb.default_root()
    run = _Run(data, manifest, image_name, h, report, cancel)

    run.step("check", "checking the image")
    refusals = fw.check_image(data, manifest, allow_unlisted)
    if refusals:
        run.refuse("the image is refused: " + "; ".join(refusals))
    iapdrive.check_name(image_name)
    run.record["image_checks"] = "passed" + (" (no manifest, --allow-unlisted)" if manifest is None else "")
    ports = h.ports()
    if port is not None:
        ports = [(p, s) for p, s in ports if p.upper() == port.upper()] or [(port, None)]
    if len(ports) > 1:
        run.refuse("more than one OpenScope port (" + ", ".join(p for p, _ in ports)
                   + "): connect one unit only, or give --port")
    if ports:
        return _path_a(run, ports[0], drive, root)
    return _path_c(run, drive, root, no_backup_ack)


def _path_a(run: _Run, port_serial, drive, root: Path) -> UpdateResult:
    h = run.h
    run.record["path"] = "A"
    run.step("backup", "backing up the calibration of this unit")
    run.stop_point()
    dev = h.open_device(port_serial[0])
    folder = None
    try:
        st = dev.status()
        ok, why = power_ok(st)
        if not ok:
            run.refuse(why)
        run.info(why)
        ident = dev.description().identity
        if ident is None:
            run.refuse("the unit reports no unit identity (API level 0, an image before stage-190): "
                       "this path cannot check the backup against the unit")
        run.record["before"] = {"port": dev.link.port, "fw_version": st.fw_version,
                                "uid": ident.uid_hex, "target": ident.target,
                                "git_rev": ident.git_rev, "api_level": ident.api_level}
        try:
            folder = cb.make_backup(dev, root=root, now=h.now(),
                                    progress=lambda s, d, t: run.report(Event("progress", "backup",
                                                                              s, d, t)),
                                    cancel=run.cancel)
        except cb.BackupError as e:
            raise Refused(f"backup failed, nothing will be copied: {e}", "backup") from None
    finally:
        dev.close()
    run.record["before_backup"] = str(folder)
    try:
        v = cb.verify_backup(folder)
        info = cb.read_info(folder)
        if not v.ok:
            run.refuse("the backup does not verify: " + "; ".join(v.problems))
        if info["unit"]["uid"] != ident.uid_hex:
            run.refuse(f"backup unit ID {info['unit']['uid']} is not section 7 unit ID {ident.uid_hex}")
        if info["cal"].get("block_blank"):
            run.refuse("the cal block of this unit is blank (all 0xFF): stop and ask (no "
                       "calibration to protect is not a normal state)")
        serial = info["unit"].get("usb_serial") or info["unit"].get("usb_serial_from_uid")
        run.info(f"backup {folder} verified, unit {ident.uid_hex}, USB serial {serial}")

        run.step("enter", "upgrade mode")
        drv = run.wait_for_drive(drive, h.enter_timeout,
                                 ENTER_TEXT)
        run.step("drive", "checking the drive")
        same = _same_serial(drv.usb_serial, serial)
        if same is False:
            run.refuse(f"the IAP drive's USB serial {drv.usb_serial} is not this unit's {serial}: "
                       "it is a different unit")
        run.info(drv.describe() + ("" if same else " (drive serial not known: not compared)"))
        run.record["drive"] = drv.describe()
        run.step("copy", "copying the image")
        run.copy(drv)
        run.step("wait", "waiting for the unit to restart")
        port = run.wait_after_copy(drv, serial)
        run.step("verify", "checking the firmware and the calibration")
        problems, after, lines = run.verify(port, ident.uid_hex, folder, root)
    except BaseException as e:
        result = "refused" if isinstance(e, Refused) else \
            "cancelled" if isinstance(e, Cancelled) else "failed"
        log = run.write_log(folder, result, str(e))
        if isinstance(e, UpdateError):
            e.log = log
        raise
    return _finish(run, folder, after, problems, lines, "A")


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
    after = None
    try:
        run.step("copy", "copying the image")
        run.copy(drv)
        run.step("wait", "waiting for the unit to restart")
        port = run.wait_after_copy(drv, drv.usb_serial)
        run.step("verify", "checking the firmware and the calibration")
        problems, after, lines = run.verify(port, uid, folder, root)
    except BaseException as e:
        result = "refused" if isinstance(e, Refused) else \
            "cancelled" if isinstance(e, Cancelled) else "failed"
        log = run.write_log(folder or after, result, str(e))
        if isinstance(e, UpdateError):
            e.log = log
        raise
    return _finish(run, folder or after, after, problems, lines, "C")


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
                       seconds=round(total, 1), user_wait_seconds=round(run.user_wait, 1))
    run.report(Event("done", "log", "update complete" if res.ok else "update finished with problems"))
    return res
