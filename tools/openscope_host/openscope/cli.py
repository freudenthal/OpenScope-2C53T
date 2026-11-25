"""`openscope` command line (remote_protocol.md §4.2).

Every subcommand exits non-zero with one readable line when the device is
absent or refuses — no tracebacks for expected failures (§4.2)."""
from __future__ import annotations

import argparse
import sys
from typing import List, Optional

from . import proto
from .device import Device, DeviceError, Nak, Timeout
from .link import NoDevice, candidate_ports, looked_where

EXIT_OK, EXIT_NO_DEVICE, EXIT_DEVICE_ERROR, EXIT_USAGE = 0, 1, 2, 64


def _now() -> float:          # the meter staleness clock (tests replace it)
    import time
    return time.monotonic()


def _info(dev: Device, _a) -> int:
    version = dev.ping()
    st = dev.status()
    if st.battery_known:
        batt = f"{st.battery_pct}% ({st.battery_mv} mV{', charging' if st.charging else ''}"
        batt += ", CRITICAL)" if st.battery_critical else ")"
    else:
        batt = "unknown (no sample yet" + (", charging)" if st.charging else ")")
    print(f"port      {dev.link.port}")
    print(f"firmware  {version}")
    print(f"protocol  v{st.proto_version}")
    print(f"mode      {st.mode_name}")
    print(f"battery   {batt}")
    print(f"capture   {'real samples' if st.capture_ready else 'no capture data yet'}")
    print(f"uptime    {st.uptime_ms / 1000:.1f} s")
    print(f"usb       {st.usb_tx_stalls} TX stalls, {st.usb_heals} self-heals")
    for line in identity_lines(_identity_of(dev)):
        print(line)
    return EXIT_OK


def _identity_of(dev: Device) -> Optional[proto.Identity]:
    """Section 7, or None: firmware before stage-190 has no section 7, and
    firmware before stage-086 has no GET_DESCRIPTION (UNSUPPORTED)."""
    try:
        return dev.description().identity
    except Nak:
        return None


def identity_lines(ident: Optional[proto.Identity]) -> List[str]:
    """`openscope info` / `describe`: which unit, which build, which API level."""
    if ident is None:
        return ["unit ID   not reported (firmware before stage-190)",
                "API level 0"]
    return [f"unit ID   {ident.uid_hex[:8]} {ident.uid_hex[8:16]} {ident.uid_hex[16:]}"
            f" (USB serial {ident.usb_serial})",
            f"build     {ident.target} at git {ident.git_rev}",
            f"API level {ident.api_level}"]


def _press(dev: Device, a) -> int:
    for b in a.buttons:
        dev.press(b)
        print(f"pressed {b.upper()}")
    return EXIT_OK


def _meter(dev: Device, a) -> int:
    import csv
    import time as _t
    f = open(a.log, "a", newline="") if a.log else None
    writer = csv.writer(f) if f else None
    if f and f.tell() == 0:
        writer.writerow(["t_unix", "update_count", "value", "unit", "display", "raw_bcd",
                         "decimal_pos", "result", "submode", "ac", "autorange", "hold"])
    continuous = a.count == 0
    last = None
    n = 0
    # A frozen update_count means the meter is not producing readings (wrong
    # mode, meter chip silent). A finite --count must not poll forever.
    stale_limit = max(3.0, 20 * a.interval)
    last_new = _now()
    try:
        while True:
            try:
                m = dev.meter()
            except (Nak, Timeout) as e:
                # A long log must survive a range change (NOT_READY) or one
                # lost reply; a one-shot read reports it.
                if not continuous:
                    raise
                print(f"# {e}", file=sys.stderr, flush=True)
                _t.sleep(max(a.interval, 0.5))
                continue
            if m.update_count == last and not continuous and _now() - last_new > stale_limit:
                print(f"error: meter reading not updating for {stale_limit:.0f} s "
                      f"(update_count stuck at {last})", file=sys.stderr)
                return EXIT_DEVICE_ERROR
            if m.update_count != last:          # only new readings, not re-reads
                last = m.update_count
                last_new = _now()
                print(f"{m.display} {m.unit}   ({m.result}, raw {m.raw_bcd}, #{m.update_count})", flush=True)
                if writer:
                    writer.writerow([f"{_t.time():.3f}", m.update_count, m.value, m.unit, m.display,
                                     m.raw_bcd, m.decimal_pos, m.result, m.submode,
                                     int(m.ac), int(m.autorange), int(m.hold)])
                    f.flush()                   # a killed logger keeps every row it printed
                n += 1
            if not continuous and n >= a.count:
                return EXIT_OK
            _t.sleep(a.interval)
    except KeyboardInterrupt:
        return EXIT_OK
    finally:
        if f:
            f.close()


CSV_FIELDS = ["t_unix", "frame_id", "channel", "index", "sample", "in_head_defect", "t_s",
              "timebase_idx", "sample_rate_hz", "vdiv_idx", "uv_per_div", "counts_per_div", "flags"]
NPZ_NOTE = ("samples: raw unsigned ADC counts [frame, channel, index]. calibrated=0: no per-unit "
            "calibration exists. volts_per_count = uv_per_div / 1e6 / counts_per_div is the bench-"
            "unit-#1 gain of the range (0 = none): amplitudes (Vpp) only, the zero point is not "
            "calibrated. sample_rate_hz 0 = no trustworthy rate. samples[..., :head_skip] are the "
            "known record-head defect (firmware scope_record.h).")


def write_capture_csv(path: str, captures) -> int:
    """Long format, one row per sample, every row self-describing. `captures`
    is [(t_unix, [Waveform, ...]), ...]. Returns the number of sample rows."""
    import csv
    rows = 0
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_FIELDS)
        for t, waves in captures:
            for wf in waves:
                dt = wf.seconds_per_sample
                for i, v in enumerate(wf.samples):
                    w.writerow([f"{t:.3f}", wf.frame_id, wf.channel + 1, i, v, int(i < wf.head_skip),
                                "" if dt is None else f"{i * dt:.9g}",
                                wf.timebase_idx, wf.sample_rate_hz, wf.vdiv_idx, wf.uv_per_div,
                                wf.counts_per_div, wf.flags])
                    rows += 1
    return rows


def write_capture_npz(path: str, captures) -> None:
    import numpy as np                   # optional: callers check before capturing
    first = captures[0][1]
    if any(len(waves) != len(first) or any(len(x.samples) != len(first[0].samples) for x in waves)
           for _, waves in captures):
        raise ValueError("frames differ in shape; use a .csv output")
    np.savez_compressed(
        path,
        samples=np.array([[np.frombuffer(x.samples, dtype=np.uint8) for x in waves]
                          for _, waves in captures], dtype=np.uint8),
        t_unix=np.array([t for t, _ in captures], dtype=np.float64),
        frame_id=np.array([waves[0].frame_id for _, waves in captures], dtype=np.uint32),
        channels=np.array([x.channel + 1 for x in first], dtype=np.uint8),
        flags=np.array([[x.flags for x in waves] for _, waves in captures], dtype=np.uint8),
        timebase_idx=np.array([waves[0].timebase_idx for _, waves in captures], dtype=np.uint8),
        sample_rate_hz=np.array([waves[0].sample_rate_hz for _, waves in captures], dtype=np.uint32),
        vdiv_idx=np.array([[x.vdiv_idx for x in waves] for _, waves in captures], dtype=np.uint8),
        uv_per_div=np.array([[x.uv_per_div for x in waves] for _, waves in captures], dtype=np.uint32),
        counts_per_div=np.array([waves[0].counts_per_div for _, waves in captures], dtype=np.uint16),
        head_skip=np.array([waves[0].head_skip for _, waves in captures], dtype=np.uint16),
        note=np.array(NPZ_NOTE),
    )


def nak_explained(e: Nak) -> str:
    name = proto.ERRORS.get(e.code, "")
    hint = proto.NAK_HINTS.get(name)
    return f"{e}: {hint}" if hint else str(e)


def _wave_line(wf: proto.Waveform) -> str:
    sm = wf.summary()
    parts = [f"#{wf.frame_id} {wf.channel_name}: {wf.sample_count} samples"]
    if "min_counts" in sm:
        parts.append(f"counts {sm['min_counts']}..{sm['max_counts']} (mean {sm['mean_counts']})")
    if sm.get("vpp_volts") is not None:
        parts.append(f"~{sm['vpp_volts']:.3g} Vpp ({wf.vdiv_tier} gain, uncalibrated)")
    if sm.get("frequency_hz"):
        parts.append(f"~{sm['frequency_hz']:.4g} Hz")
    elif sm.get("period_samples"):
        parts.append(f"period {sm['period_samples']} samples")
    rate = (f"{wf.sample_rate_hz} S/s {wf.timebase_tier}" if wf.sample_rate_hz
            else f"code 0x{wf.timebase_idx:02X}: no trustworthy rate")
    parts.append(rate)
    return ", ".join(parts)


def _scope(dev: Device, a) -> int:
    import time as _t
    mask = proto.channel_mask(a.channels)
    out = a.out
    if out and not out.lower().endswith((".csv", ".npz")):
        print("error: --out must end in .csv or .npz", file=sys.stderr)
        return EXIT_USAGE
    if out and out.lower().endswith(".npz"):
        try:
            import numpy  # noqa: F401
        except ImportError:
            print("error: .npz needs numpy, which is not installed; use a .csv output",
                  file=sys.stderr)
            return EXIT_USAGE
    if a.frames < 1:
        print("error: --frames must be >= 1", file=sys.stderr)
        return EXIT_USAGE
    captures = []
    last_id = None
    # A frame_id that does not advance means acquisition is stopped or held
    # (RUN/STOP, SINGLE done, NORMAL with no trigger). Re-reads are not new
    # captures, so a finite --frames must not poll forever.
    stale_limit = max(3.0, 20 * a.interval)
    last_new = _now()
    status = EXIT_OK
    while len(captures) < a.frames:
        try:
            waves = dev.waveform(mask)
        except Nak as e:
            if proto.ERRORS.get(e.code) == "NOT_READY" and _now() - last_new <= stale_limit:
                _t.sleep(max(a.interval, 0.05))       # no tear-free copy this time
                continue
            print(nak_explained(e), file=sys.stderr)
            status = EXIT_DEVICE_ERROR
            break
        if waves[0].frame_id != last_id:
            last_id = waves[0].frame_id
            last_new = _now()
            captures.append((_t.time(), waves))
            for wf in waves:
                print(_wave_line(wf), flush=True)
        elif _now() - last_new > stale_limit:
            print(f"error: no new capture for {stale_limit:.0f} s (frame_id stuck at {last_id}): "
                  "acquisition is stopped or held (RUN/STOP, SINGLE, NORMAL without a trigger); "
                  f"got {len(captures)} of {a.frames} frames", file=sys.stderr)
            status = EXIT_DEVICE_ERROR
            break
        if len(captures) < a.frames:
            _t.sleep(a.interval)
    if out and captures:
        if out.lower().endswith(".npz"):
            write_capture_npz(out, captures)
        else:
            write_capture_csv(out, captures)
        note = "" if status == EXIT_OK else " (partial)"
        print(f"saved {out}: {len(captures)} frame(s) x {len(captures[0][1])} channel(s){note}")
    return status


def _state_lines(s: proto.ScopeState) -> List[str]:
    rate = (f"{s.sample_rate_hz} S/s ({s.timebase_tier_name})" if s.sample_rate_hz
            else "no trustworthy rate")
    tb = f"0x{s.timebase_code:02X} in force, {rate}"
    tb += (f"; display shows 0x{s.timebase_idx:02X}: DISAGREES" if s.timebase_disagrees
           else "; display agrees")
    lines = [f"mode      {s.mode_name}, {'running' if s.running else 'STOPPED'}",
             f"timebase  {tb}"]
    for i, c in enumerate(s.ch):
        gain = (f"{c.uv_per_div / 1000:g} mV/div ({c.tier_name}, bench unit #1)" if c.uv_per_div
                else "no volts meaning on this range")
        probe = f"probe x{s.probe[i]}" if s.probe else "probe not reported (old firmware)"
        lines.append(f"CH{i + 1}       {'on ' if c.enabled else 'off'}, range {c.vdiv_idx}, {gain}, {probe}")
    lines.append(f"trigger   {s.trig_mode_name}, {s.trig_edge_name}, {s.trig_source_name}, "
                 f"level {s.trig_level:+d} (reg 0x08 = 0x{s.trig_code:02X})")
    lines.append(f"grid      {s.counts_per_div} ADC counts/div")
    return lines


def _state(dev: Device, _a) -> int:
    for line in _state_lines(dev.scope_state()):
        print(line)
    return EXIT_OK


def describe_lines(d: proto.Description) -> List[str]:
    """`openscope describe`: practical units first, the codes in the last column."""
    lines = [f"timebase  ({d.samples_per_div} samples/div)",
             f"  {'time/div':<16}{'sample rate':<14}{'tier':<13}code"]
    for code in range(len(d.rates)):
        rate = d.sample_rate(code)
        lines.append(f"  {proto.format_seconds(d.time_per_div(code)):<16}"
                     f"{(f'{rate} S/s' if rate else '-'):<14}{d.tier('timebase', code):<13}0x{code:02X}")
    lines.append(f"volts/div  ({d.counts_per_div} ADC counts/div; BNC = no probe factor)")
    lines.append(f"  {'ch':<4}{'at BNC':<18}{'at probe':<18}{'tier':<13}range")
    for ch in range(1, len(d.ranges) + 1):
        factor = d.probe_factor(ch)
        for r in range(len(d.ranges[ch - 1])):
            bnc = d.volts_per_div(ch, r, probe=1)
            tip = d.volts_per_div(ch, r)
            lines.append(f"  CH{ch:<2}{proto.format_volts(bnc):<18}"
                         f"{(proto.format_volts(tip) + f' (x{factor})') if tip else '-':<18}"
                         f"{d.tier('range', ch, r):<13}{r}")
    if d.trigger:
        t = d.trigger
        lines.append(f"trigger   level {t.level_min}..{t.level_max} -> reg 0x08 = "
                     f"clamp({t.center_code} + trunc(level*{t.num}/{t.den}), {t.code_min}, {t.code_max}); "
                     f"fires at record count code{t.record_offset:+d}; "
                     f"{t.counts_per_level:.3f} ADC counts per level step")
    if d.probe:
        lines.append("probe     " + ", ".join(f"CH{i + 1} x{f}" for i, f in enumerate(d.probe)))
    lines.extend(identity_lines(d.identity))
    if d.skipped:
        lines.append("sections this tool does not read: " + ", ".join(str(s) for s in d.skipped))
    return lines


def _describe(dev: Device, _a) -> int:
    for line in describe_lines(dev.description()):
        print(line)
    return EXIT_OK


SET_USAGE = ("set timebase <code 0x00-0x14> | set vdiv <1|2> <0-9> | "
             "set trigger <auto|normal|single> [rising|falling] [ch1|ch2] [level -100..100] | "
             "set run | set stop | set mode <scope|meter|siggen|settings> | set probe <1|2> <1|10>")


def _int(text: str) -> int:
    return int(text, 0)          # 0x10, 16, -20


def _set(dev: Device, a) -> int:
    what, v = a.what, a.values
    try:
        # Every value is checked HERE, before anything is sent (exit 64).
        if what == "timebase" and len(v) == 1:
            code = _int(v[0])
            proto.payload_set_timebase(code)
            action = lambda: dev.set_timebase(code)  # noqa: E731
        elif what == "vdiv" and len(v) == 2:
            ch, rng = _int(v[0]), _int(v[1])
            proto.payload_set_vdiv(ch, rng)
            action = lambda: dev.set_vdiv(ch, rng)  # noqa: E731
        elif what == "probe" and len(v) == 2:
            ch, factor = _int(v[0]), _int(v[1])
            proto.payload_set_probe(ch, factor)
            action = lambda: dev.set_probe(ch, factor)  # noqa: E731
        elif what in ("run", "stop") and not v:
            action = lambda: dev.run_stop(what == "run")  # noqa: E731
        elif what == "mode" and len(v) == 1:
            proto.mode_id(v[0])
            action = lambda: dev.set_mode(v[0])  # noqa: E731
        elif what == "trigger" and 1 <= len(v) <= 4:
            # Omitted fields keep the device's current values.
            cur = dev.scope_state()
            mode, edge, src, level = v[0], cur.trig_edge, cur.trig_source, cur.trig_level
            for tok in v[1:]:
                t = tok.lower()
                if t in proto.TRIG_EDGES.values():
                    edge = t
                elif t.upper() in proto.TRIG_SOURCES.values():
                    src = t.upper()
                else:
                    level = _int(tok)
            proto.payload_set_trigger(mode, edge, src, level)       # validate before sending
            action = lambda: dev.set_trigger(mode, edge, src, level)  # noqa: E731
        else:
            print(f"usage: openscope {SET_USAGE}", file=sys.stderr)
            return EXIT_USAGE
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return EXIT_USAGE
    try:
        action()
    except Nak as e:
        print(f"refused: {nak_explained(e)}", file=sys.stderr)
        return EXIT_DEVICE_ERROR
    for line in _state_lines(dev.scope_state()):          # what is in force now
        print(line)
    return EXIT_OK


def _shell(dev: Device, a) -> int:
    print(dev.shell(" ".join(a.line), timeout=a.timeout))
    return EXIT_OK


def _screenshot(dev: Device, a) -> int:
    from .screen import save_png
    s = dev.screenshot()
    save_png(a.out, s.w, s.h, s.indexed4, scale=a.scale)
    note = "transport verified; the live screen changed during capture (torn)" if s.torn else "CRC verified"
    print(f"saved {a.out} ({s.w}x{s.h}, {note})")
    return EXIT_OK


def backup_lines(folder) -> List[str]:
    """What `openscope backup` prints for the folder it made."""
    from . import calbackup as cb
    info = cb.read_info(folder)
    cal, fw, unit = info["cal"], info["firmware"], info["unit"]
    page = info["files"][cb.CAL_FILE]
    status = cal.get("cal_status_crc32")
    lines = [f"backup    {folder}",
             f"unit ID   {unit['uid']} on {unit['port']}",
             f"firmware  git {fw.get('git_rev') or 'not reported'}, binary protocol "
             f"{'yes' if fw.get('binary_protocol') else 'no (shell only)'}",
             f"cal page  {page['size']} B, CRC32 {page['crc32']}, two reads equal, "
             + ("equal to the `cal status` live CRC" if status else "no `cal status` CRC on this image"),
             f"cal block {cal['block']} CRC32 {cal['block_crc32']}"
             + ("  WARNING: blank (all 0xFF), this is not a calibration" if cal.get("block_blank") else "")]
    region = info.get("w25q")
    if region:
        w = info["files"][cb.W25Q_FILE]
        lines.append(f"W25Q      {region['start']} + {w['size']} B, CRC32 {w['crc32']}, each chunk read "
                     f"two times, {region['seconds']} s, {region['retries']} chunk(s) read again")
    lines.append("verified  the saved files read back equal to backup.json")
    return lines


def _backup_progress(stage: str, done: int, total: int) -> None:
    if stage == "w25q" and (done == total or done % (256 * 1024) == 0):
        print(f"W25Q {done // 1024} of {total // 1024} KiB", file=sys.stderr, flush=True)


def _backup(a) -> int:
    """`backup`, `backup verify DIR`, `backup compare DIR (DIR2 | --live)`.
    Only reads the unit; verify and compare DIR DIR2 open no port."""
    from . import calbackup as cb
    if a.backup_cmd == "verify":
        r = cb.verify_backup(a.dir)
        for line in r.notes:
            print(line)
        for line in r.problems:
            print(f"error: {line}", file=sys.stderr)
        if r.ok:
            print(f"verified  {a.dir}")
        return EXIT_OK if r.ok else EXIT_DEVICE_ERROR
    if a.backup_cmd == "compare":
        if (a.dir2 is None) != a.live:
            print("usage: openscope backup compare DIR (DIR2 | --live)", file=sys.stderr)
            return EXIT_USAGE
        other, name = a.dir2, a.dir2
        first = cb.load_page(a.dir)             # a bad DIR fails before a port is opened
        if a.live:
            dev = cb.open_for_backup(a.port)
            try:
                other, name = cb.read_cal_page(dev), f"the live unit on {dev.link.port}"
            finally:
                dev.close()
        c = cb.compare_cal(first, other)
        print(f"compare   {a.dir}")
        print(f"with      {name}")
        for line in c.lines():
            print(line)
        return EXIT_OK if c.ok else EXIT_DEVICE_ERROR
    dev = cb.open_for_backup(a.port)
    try:
        folder = cb.make_backup(dev, root=a.out, w25q=a.w25q, progress=_backup_progress)
    finally:
        dev.close()
    for line in backup_lines(folder):
        print(line)
    return EXIT_OK


def _manifest(a) -> int:
    """`openscope manifest IMAGE --target T [--api-level N | --source REPO]`."""
    from pathlib import Path
    from . import firmware as fw
    image = Path(a.image)
    data = image.read_bytes()
    m = fw.make_manifest(data, a.target, api_level=a.api_level, version=a.version,
                         source=a.source, product_commit=a.product_commit, image_name=image.name)
    refusals = fw.check_image(data, m)
    for r in refusals:
        print(f"error: {r}", file=sys.stderr)
    if refusals:
        return EXIT_DEVICE_ERROR
    out = Path(a.out) if a.out else fw.manifest_path_for(image)
    m.save(out)
    print(f"manifest  {out}")
    print(f"image     {image.name}, {m.size} B, SHA-256 {m.sha256}")
    print(f"build     {m.target} at git {m.git_rev or 'not in the image'}, {m.build_time}, "
          f"API level {m.api_level}")
    return EXIT_OK


def _load_update_files(a):
    """(data, manifest or None, image name) from --manifest / --image."""
    from pathlib import Path
    from . import firmware as fw
    if not a.manifest and not a.image:
        raise ValueError("give --manifest FILE or --image FILE")
    m = None
    if a.manifest:
        m = fw.Manifest.load(a.manifest)
        image = Path(a.image) if a.image else fw.image_path_for(a.manifest, m)
    else:
        image = Path(a.image)
        side = fw.manifest_path_for(image)
        if side.is_file():
            m = fw.Manifest.load(side)
            print(f"manifest  {side}")
    return image.read_bytes(), m, image.name


def _update_event(e) -> None:
    if e.kind == "step":
        print(f"[{e.step}] {e.text}", flush=True)
    elif e.kind == "info":
        print(f"          {e.text}", flush=True)
    elif e.kind == "wait":
        print(f"\n>>> {e.text}\n", flush=True)
    elif e.kind == "progress" and e.step == "copy":
        print(f"          {e.text}", flush=True)


def _update(a) -> int:
    """`openscope update ...` and `openscope update check --manifest FILE`."""
    from . import firmware as fw, iapdrive, update as up
    if a.update_cmd == "check":
        m = fw.Manifest.load(a.check_manifest)
        dev = Device.open(a.port)
        try:
            st = dev.status()
            ident = _identity_of(dev)
        finally:
            dev.close()
        diffs = fw.check_running(m, st.fw_version, ident)
        print(f"unit      {st.fw_version}")
        for line in identity_lines(ident):
            print(line)
        print(f"manifest  {m.fw_version} ({m.target}, API level {m.api_level})")
        for d in diffs:
            print(f"differs   {d}")
        print("result    " + ("the unit runs this build" if not diffs else "NOT this build"))
        return EXIT_OK if not diffs else EXIT_DEVICE_ERROR
    if a.first_install and a.stock_image:
        raise ValueError("give --first-install or --stock-image, not both")
    try:
        if a.stock_image:
            r = _update_stock(a, up)
        elif a.first_install:
            r = _update_first_install(a, up)
        else:
            data, m, name = _load_update_files(a)
            r = up.run_update(data, m, name, port=a.port, drive=a.drive, backup_root=a.backup_root,
                              allow_unlisted=a.allow_unlisted, no_backup_ack=a.no_backup_ack,
                              report=_update_event)
    except up.Refused as e:
        print(f"refused: {e} (nothing was written to the unit)", file=sys.stderr)
        if e.log:
            print(f"log       {e.log}", file=sys.stderr)
        return EXIT_DEVICE_ERROR
    except up.UpdateError as e:
        print(f"error: {e}", file=sys.stderr)
        if e.log:
            print(f"log       {e.log}", file=sys.stderr)
        return EXIT_DEVICE_ERROR
    except iapdrive.DriveError as e:
        print(f"error: {e}", file=sys.stderr)
        return EXIT_DEVICE_ERROR
    except KeyboardInterrupt:
        # The copy is one synchronous write: Ctrl-C lands before it or after it.
        print("stopped by the user (if the copy had started, it finished; run "
              "`openscope update check` when the port is back)", file=sys.stderr)
        return EXIT_DEVICE_ERROR
    for line in r.compare_lines:
        print(f"          {line}")
    for p in r.problems:
        print(f"differs   {p}", file=sys.stderr)
    print(f"backup    before {r.before_backup or 'none'}, after {r.after_backup or 'none'}")
    print(f"log       {r.log}")
    print(f"time      {r.seconds:.0f} s ({r.seconds - r.user_wait_seconds:.0f} s without user waits)")
    done = {"S": "stock image installed, the user confirmed the stock screen",
            "F": "first install done: backup made before any key press, verified after a restart"}
    print("result    " + (done.get(r.path, "updated and verified") if r.ok
                          else "the image was copied, but verify found differences"))
    return EXIT_OK if r.ok else EXIT_DEVICE_ERROR


def _ask(prompt: str) -> str:
    try:
        return input(prompt)
    except EOFError:
        return ""


def _update_first_install(a, up):
    """`openscope update --first-install`: the unit runs the stock firmware."""
    data, m, name = _load_update_files(a)
    print("First install of OpenScope on a unit that runs the stock FNIRSI firmware.")
    print(f"* {up.STOCK_FILES_TEXT}")
    print(f"* {up.NO_KEYS_TEXT}")
    print(f"* This image is for the FNIRSI {up.MODEL} only.", flush=True)
    model = a.model if a.model is not None else _ask(
        f"Type the model on the label of the unit ({up.MODEL}): ")
    ack = a.install_ack if a.install_ack is not None else _ask(
        f"Type {up.FIRST_INSTALL_ACK} to install OpenScope: ")
    return up.run_first_install(data, m, name, model=model, install_ack=ack, drive=a.drive,
                                backup_root=a.backup_root, allow_unlisted=a.allow_unlisted,
                                report=_update_event)


def _update_stock(a, up):
    """`openscope update --stock-image FILE`: back to the stock firmware."""
    from pathlib import Path
    image = Path(a.stock_image)

    def confirm(text: str) -> bool:
        answer = _ask("Does the unit show the stock screen and stock version? [y/N] ")
        return answer.strip().lower() in ("y", "yes")
    return up.run_stock_install(image.read_bytes(), image.name, port=a.port, drive=a.drive,
                                backup_root=a.backup_root, confirm=confirm, report=_update_event)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="openscope", description="Drive an OpenScope 2C53T over USB.")
    ap.add_argument("--port", help="serial port (default: auto-detect by USB VID:PID)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports", help="list candidate ports")
    sub.add_parser("info", help="firmware version, mode, battery, USB health")
    p = sub.add_parser("press", help="inject button presses, e.g. `press MENU OK`")
    p.add_argument("buttons", nargs="+", metavar="BUTTON",
                   help=" | ".join(proto.BUTTONS))
    p = sub.add_parser("meter", help="print (and optionally log to CSV) multimeter readings")
    p.add_argument("--count", type=int, default=1, help="readings to take (0 = until Ctrl-C)")
    p.add_argument("--interval", type=float, default=0.25, help="poll period, s (meter updates ~4 Hz)")
    p.add_argument("--log", metavar="CSV", help="append readings to this CSV (raw BCD kept, see #28)")
    p = sub.add_parser("scope", help="read captured waveforms (raw ADC counts) and save them")
    p.add_argument("--frames", type=int, default=1, help="distinct captures to take (new frame_id each)")
    p.add_argument("--channels", default="1", help="1, 2 or 1,2 (default 1)")
    p.add_argument("--interval", type=float, default=0.05, help="poll period, s")
    p.add_argument("--out", metavar="FILE", help="save to .csv (always) or .npz (needs numpy)")
    sub.add_parser("state", help="scope settings: mode, timebase, volts/div, probe, trigger, run/stop")
    sub.add_parser("describe", help="time/div of every timebase code, volts/div of every range, "
                                    "trigger scale, probe (practical units; codes last)")
    p = sub.add_parser("set", help="change a scope setting; prints the state in force after it",
                       description=SET_USAGE)
    p.add_argument("what", choices=["timebase", "vdiv", "trigger", "run", "stop", "mode", "probe"])
    p.add_argument("values", nargs="*", metavar="VALUE")
    p = sub.add_parser("shell", help="run one ASCII debug-shell command")
    p.add_argument("line", nargs="+")
    p.add_argument("--timeout", type=float, default=3.0)
    p = sub.add_parser("screenshot", help="save the device screen as PNG")
    p.add_argument("out")
    p.add_argument("--scale", type=int, default=2)
    p = sub.add_parser("backup", help="save a checked copy of the calibration page (read only); "
                                      "also: backup verify DIR, backup compare DIR (DIR2 | --live)")
    p.add_argument("--out", metavar="DIR", help="backup root (default: %%LOCALAPPDATA%%\\OpenScope\\backups); "
                                                "the backup goes to DIR/<unit ID>/<date_time>/")
    p.add_argument("--w25q", action="store_true",
                   help="also save the W25Q regions that OpenScope writes (3 MiB, about 3 minutes)")
    bsub = p.add_subparsers(dest="backup_cmd")
    v = bsub.add_parser("verify", help="check the files of a backup folder against its backup.json")
    v.add_argument("dir", metavar="DIR")
    c = bsub.add_parser("compare", help="compare the cal block of a backup with a second backup "
                                        "(folder or .bin file) or with the live unit")
    c.add_argument("dir", metavar="DIR")
    c.add_argument("dir2", metavar="DIR2", nargs="?")
    c.add_argument("--live", action="store_true", help="compare with the unit that is connected")
    p = sub.add_parser("update", help="install a firmware image through the factory IAP drive "
                                      "(backup first, then the user enters upgrade mode); "
                                      "--first-install from stock, --stock-image back to stock; "
                                      "also: update check --manifest FILE")
    p.add_argument("--manifest", metavar="FILE", help="image manifest (<image>.manifest.json)")
    p.add_argument("--image", metavar="FILE", help="image file (its manifest is read from "
                                                   "<image>.manifest.json when it exists)")
    p.add_argument("--drive", metavar="X:", help="the IAP drive (default: find it)")
    p.add_argument("--allow-unlisted", action="store_true",
                   help="accept an image with no manifest (the other checks still apply)")
    p.add_argument("--backup-root", metavar="DIR",
                   help="backup root (default: %%LOCALAPPDATA%%\\OpenScope\\backups)")
    p.add_argument("--no-backup-ack", metavar="TEXT",
                   help="unit already in upgrade mode with no backup on disk: give the exact text "
                        "'I HAVE NO BACKUP' to go on")
    p.add_argument("--first-install", action="store_true",
                   help="the unit runs the stock FNIRSI firmware: install OpenScope, back up at "
                        "once after the first boot (cal page and W25Q), verify after a restart")
    p.add_argument("--model", help="--first-install: the model on the unit's label (2C53T); "
                                   "asked when not given")
    p.add_argument("--install-ack", metavar="TEXT",
                   help="--first-install: the text INSTALL; asked when not given")
    p.add_argument("--stock-image", metavar="FILE",
                   help="go back to the stock firmware with your own FNIRSI file (its SHA-256 must "
                        "be a known stock version); the backup comes first")
    usub = p.add_subparsers(dest="update_cmd")
    c = usub.add_parser("check", help="compare the running firmware with a manifest")
    c.add_argument("--manifest", dest="check_manifest", metavar="FILE", required=True)
    p = sub.add_parser("manifest", help="write <image>.manifest.json for an image")
    p.add_argument("image", metavar="IMAGE")
    p.add_argument("--target", required=True, help="make target, e.g. guest-coldtrace")
    p.add_argument("--api-level", type=int, help="ESP_API_LEVEL of the source (or give --source)")
    p.add_argument("--source", metavar="REPO", help="product source tree: read ESP_API_LEVEL "
                                                     "from firmware/src/drivers/esp_comm.h")
    p.add_argument("--version", help="text for people (default: '<target> <git rev>')")
    p.add_argument("--product-commit", help="full commit of the source tree")
    p.add_argument("--out", metavar="FILE", help="manifest path (default: <image>.manifest.json)")
    return ap


def main(argv: Optional[List[str]] = None) -> int:
    a = build_parser().parse_args(argv)
    if a.cmd == "ports":
        ports = candidate_ports()
        if not ports:
            print(f"no OpenScope found on {looked_where()}")
            return EXIT_NO_DEVICE
        for i, p in enumerate(ports):
            print(p + ("   <- would use" if i == 0 else ""))
        return EXIT_OK

    handlers = {"info": _info, "press": _press, "meter": _meter, "shell": _shell,
                "screenshot": _screenshot, "scope": _scope, "state": _state, "set": _set,
                "describe": _describe}
    dev = None
    try:
        if a.cmd == "backup":               # opens the port itself, and only when it reads the unit
            return _backup(a)
        if a.cmd == "manifest":             # no port
            return _manifest(a)
        if a.cmd == "update":               # opens the port and the drive itself
            return _update(a)
        dev = Device.open(a.port)           # also checks the protocol major (§3.7)
        return handlers[a.cmd](dev, a)
    except NoDevice as e:                   # none found, busy, or did not come back
        print(str(e), file=sys.stderr)
        return EXIT_NO_DEVICE
    except Nak as e:
        print(str(e), file=sys.stderr)
        return EXIT_DEVICE_ERROR
    except (DeviceError, proto.ProtocolError, ValueError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return EXIT_DEVICE_ERROR
    finally:
        if dev is not None:
            dev.close()
