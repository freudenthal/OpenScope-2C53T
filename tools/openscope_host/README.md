# openscope — host tool for the OpenScope 2C53T remote protocol

Implements the host side of [`docs/design/remote_protocol.md`](../../docs/design/remote_protocol.md) (issue #10).
Core needs only `pyserial`; screenshots need no extra packages either.

```bash
cd tools/openscope_host
python3 -m openscope ports              # which port it would use (USB 2e3c:5740)
python3 -m openscope info               # firmware, protocol, mode, battery, USB health
python3 -m openscope press MENU OK      # inject button presses
python3 -m openscope meter --count 0 --log m.csv   # log the multimeter (~4 Hz) until Ctrl-C
python3 -m openscope scope --frames 10 --channels 1,2 --out cap.csv   # captures (raw counts); .npz needs numpy
python3 -m openscope state              # mode, timebase (display + in force), volts/div, probe, trigger, run/stop
python3 -m openscope describe           # time/div of every timebase code, volts/div of every range, trigger scale
python3 -m openscope set timebase 0x10  # also: set vdiv 1 6 | set trigger normal falling ch1 -20
                                        #       set run | set stop | set mode meter | set probe 1 10
python3 -m openscope shell usbstat      # run one ASCII debug-shell command
python3 -m openscope screenshot s.png   # device framebuffer, CRC-checked
python3 -m openscope backup             # checked copy of the calibration page (read only)
```

Exit codes: 0 ok, 1 no device (one-line message, no traceback), 2 device refused or error,
64 bad arguments (nothing sent).

`set` goes through the firmware's single entry point for each setting (the same one the
buttons use) and prints the state in force after it. `set mode` presses MENU on the device
until the mode matches. Omitted `set trigger` fields keep their current values. A host set is
not written to the settings store by itself: it is saved by the next button press, MENU, or
power-off countdown. STOP stops acquisition: `frame_id` holds and the LCD shows STOP. A frame
read while stopped reports the timebase code and range the held record was taken with, even
if they were changed after the STOP. RUN drops the record held across the STOP and starts again.

`describe` asks the unit once for its tables (`Device.description()`, cached per connection):
the sample rate and trust tier of each timebase code (time/div = 32 samples / rate; codes
with no measured rate print "not calibrated"), the volts/div of each range at the BNC and at
the probe, the trigger level formula, and the probe setting. `set probe` writes the same
setting as the unit's probe menu (x1 or x10) and is saved the same way as the other sets.
While a host sends frames, the unit shows "PC" in the status bar; it goes away 3 s after the
last frame.

Waveforms are raw unsigned ADC counts. A frame carries the bench-measured sample rate and
volts/div of its timebase code and range (bench unit #1, 0 where none was measured) and flags
saying how far to trust them; there is no per-unit calibration and no measured zero point, so
counts give amplitudes (Vpp), not absolute volts. `NO_CAPTURE_DATA` means the scope has no real
capture — it never sends its demo trace. `scope --frames N` counts only new captures (new
`frame_id`) and gives up, keeping what it got, when acquisition is stopped or held.

## Calibration backup (`openscope backup`)
The MCU flash page `0x08006000` (4096 bytes) holds the calibration of the unit. If it is lost,
it cannot be made again. Make a backup before each firmware update.

```bash
python3 -m openscope backup                    # cal page -> <root>/<unit ID>/<date_time>/
python3 -m openscope backup --out D:\scope     # a different root
python3 -m openscope backup --w25q             # also the W25Q regions OpenScope writes (3 MiB)
python3 -m openscope backup verify DIR         # the files of DIR against its backup.json
python3 -m openscope backup compare DIR DIR2   # cal block of two backups (DIR2: folder or .bin)
python3 -m openscope backup compare DIR --live # cal block of a backup and of the connected unit
```

* The default root is `%LOCALAPPDATA%\OpenScope\backups` (`~/.local/share/OpenScope/backups`
  on other systems). The folder name is the 96-bit MCU ID of the unit and the time.
* `backup` reads the page two times and the two reads must be equal. The CRC32 must be equal to
  the CRC that `cal status` prints. It writes `cal_0x08006000.bin` and `backup.json` (unit ID,
  USB serial, port, firmware version, time, size, CRC32 and SHA-256 of each file, CRC32 of the
  cal block). It never writes on top of a file or a folder that is there.
* `--w25q` saves `0xD00000`–`0xFFFFFF` as `w25q_0xD00000.bin`. Each 4096-byte chunk is read two
  times; a chunk with no reply is read again (3 times at most). If the port is lost, the cal
  page stays saved and the message gives the last good address.
* `compare` compares the cal block (offsets `0x025`–`0x12E`) byte for byte. The other bytes of
  the page are stock settings that change with each flash: "block equal, other ranges changed"
  is the usual result after an update and exits 0. A different or blank cal block exits 2.
* It only reads. The commands it sends are `version`, `mem read`, `flash dump` and
  `cal status`; a guard refuses each other line before it is sent. It uses the ASCII shell, so
  it also works on an image that has no binary protocol (release v0.4.1). It cannot read a
  unit that runs the stock firmware (stock has no serial port).
* Exit codes: 0 done and checked, 1 no device, 2 a device error or a check that failed.

From Python: `openscope.calbackup` (`open_for_backup`, `make_backup`, `verify_backup`,
`compare_cal`; the module text gives the arguments and the results). `Device.flash_dump(addr,
length)` reads up to 4096 raw bytes of the W25Q. Library version 0.4.0 adds these.

## Update the firmware (`openscope update`)
The tool installs a guest image through the factory IAP drive (upgrade mode). It makes a
calibration backup first and does not copy without a verified backup of this unit.

```bash
python3 -m openscope manifest guest-coldtrace-abc1234.bin --target guest-coldtrace --source <repo>
python3 -m openscope update --manifest guest-coldtrace-abc1234.bin.manifest.json
python3 -m openscope update --image guest-coldtrace-abc1234.bin   # reads <image>.manifest.json
python3 -m openscope update check --manifest FILE                # does the unit run this build?
```

1. **check**: the image must match its manifest (SHA-256, size, target) and be a guest image:
   initial SP in SRAM, reset vector in `0x08007000` to the image end, size at most 1,017,856 B
   (it must end before `0x080FF800`), the text "OpenScope". The stock image is refused. With no
   manifest the image is refused, unless you give `--allow-unlisted`.
2. **backup**: the unit must be on USB power (STATUS charging flag, or battery at least 30 %).
   The tool makes a backup (as `openscope backup`), verifies it, and checks that its unit ID is
   the unit ID that the firmware reports. The unit must run an image with a unit identity
   (API level 1 or higher).
3. **enter upgrade mode** (you): keep USB connected, hold MENU and tap Power, keep MENU held
   until the upgrade screen shows. If it does not show (an image with no input task), hold MENU,
   press the pinhole reset and keep MENU held 3–5 s. The tool waits for the `IAP` drive. Press
   Ctrl-C to stop; up to the copy nothing is written. Do not open the drive in Explorer.
4. **drive**: exactly one drive with label `IAP`, 2048-byte sectors, about 8 MB and
   `Ready.TXT`. The stock app's USB Sharing drive (no label, 4096-byte sectors) is refused. If an
   other removable drive (a USB stick) is connected, the tool stops; remove it or give
   `--drive X:`. When Windows gives the drive's USB serial, it must be the serial of the backed-up
   unit.
5. **copy**: one write of the whole file, then flush and fsync. The factory IAP programs the
   bytes in the order they arrive, so nothing else writes to the drive. If Windows refuses the
   write, a removable-storage policy (Microsoft Defender Device Control) can be the cause.
6. **wait**: the drive must go (the unit restarts) and the port must come back within 60 s. If
   the drive stays, the copy did not complete: enter upgrade mode again and run the update again.
7. **verify**: the firmware text, build target, git revision and API level must be the
   manifest's. A second backup goes into the same unit folder; the cal block (`0x025`–`0x12E`)
   must be equal to the first backup.
8. **log**: `update_<date_time>.json` in the backup folder (each step with its time, the
   manifest, the results).

A unit that is already in upgrade mode and has no port (recovery) also gets the image: the tool
looks for a verified backup on disk with the drive's USB serial. With no such backup it stops,
unless you give `--no-backup-ack "I HAVE NO BACKUP"`.

Exit codes: 0 updated and verified; 1 no unit found; 2 refused (nothing written), a failure, or
verify found differences. From Python: `openscope.update.run_update()`, `openscope.firmware`
(`Manifest`, `make_manifest`, `check_image`, `check_running`) and `openscope.iapdrive`
(`find_iap_drive`, `copy_image`). Library version 0.5.0 adds these. The drive search works on
Windows only.

## GUI (`openscope-gui`)
The desktop window moved to its own repository,
[OpenScope-2C53T-GUI](https://github.com/freudenthal/OpenScope-2C53T-GUI) (package `openscope_gui`).
It uses this library: `pip install -e tools/openscope_host`, then install the GUI from its checkout.
Keep the library and the firmware at the same protocol version as the GUI expects.

## Layers
| module | job |
|---|---|
| `proto.py` | framing, incremental `Decoder` (resyncs on garbage, never accepts a truncated frame), STATUS / METER / WAVEFORM codecs, waveform summary |
| `link.py` | port discovery by VID:PID, reopen after the port vanishes (waits `reopen_wait`, default 6 s; 0 = raise `NoDevice("port lost")` at once) |
| `device.py` | one method per request; stateless; one retry after a replug (e.g. the firmware's CDC self-heal, #39); input is discarded before each request, so a late reply cannot answer the next one |

`Device.open(port, reopen_wait=0)` is for a caller that reconnects by itself (the GUI): a lost
port then raises at once, so no request blocks for the reopen wait. The CLI keeps 6 s.
`screenshot(cancel=f)` stops when `f()` is true. Library version 0.2.0 adds these.
| `cli.py` | the `openscope` command |

## Tests (no hardware)
```bash
python3 tests/test_proto.py        # framing + rejection paths + a mutation
python3 tests/test_end_to_end.py   # host tool vs the REAL firmware esp_comm.c (compiled into a shim, via ctypes)
```
The end-to-end suite checks the §6 acceptance criteria against the firmware's own parser and encoder: `info` shows the live mode and battery and follows a mode change, "no device" is a clean exit 1, button presses reach the injector, a full input queue is a NAK and not a false success, the ASCII shell keeps working next to the protocol, and an abandoned frame does not poison the next request.

## MCP server (LLM agents)
`openscope/mcp_server.py` exposes `scope_info`, `scope_meter`, `scope_waveform`, `scope_press`, `scope_screenshot` and `scope_shell` over MCP (stdio). It needs the `mcp` SDK (Python >= 3.10):

```bash
claude mcp add openscope -- uv run --no-project --python 3.12 --with mcp --with pyserial \
    --directory "$PWD" python -m openscope.mcp_server
```

It is safe by default: the debug shell can erase flash (`fwapply`, `flash wtest`) or desynchronise the FPGA, so only read-only commands are reachable, and POWER is refused. `--allow-raw-shell` lifts both for supervised bench work.
