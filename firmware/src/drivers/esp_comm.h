/*
 * ESP32 Communication Protocol for OpenScope 2C53T
 *
 * UART-based protocol between the GD32F307 (scope MCU) and an ESP32
 * co-processor module. Handles module downloads, firmware updates,
 * framebuffer streaming, and remote button input.
 *
 * Packet format:
 *   [0xAA] [cmd] [len_hi] [len_lo] [payload...] [checksum]
 *   checksum = XOR of all bytes from cmd through end of payload
 *
 * All communication is half-duplex: ESP32 sends command, GD32 responds.
 */

#ifndef ESP_COMM_H
#define ESP_COMM_H

#include <stdint.h>
#include <stdbool.h>

/* Packet framing */
#define ESP_SYNC_BYTE       0xAA
#define ESP_MAX_PAYLOAD     256     /* RECEIVE cap only (host->device). Responses are not
                                     * capped: a WAVEFORM_FRAME is 1048 B. Do not "fix" the
                                     * asymmetry (remote_protocol.md §2.2, §3.5). */
#define ESP_HEADER_SIZE     4       /* sync + cmd + len_hi + len_lo */
#define ESP_CHECKSUM_SIZE   1

/* Commands: ESP32 → GD32 */
#define ESP_CMD_PING            0x01    /* Ping — scope replies with version */
#define ESP_CMD_MODULE_START    0x02    /* Begin module transfer (slot + size) */
#define ESP_CMD_MODULE_DATA     0x03    /* Module data chunk (up to 256 bytes) */
#define ESP_CMD_MODULE_END      0x04    /* Finalize module install */
#define ESP_CMD_FW_UPDATE_START 0x05    /* Begin firmware update (size) */
#define ESP_CMD_FW_UPDATE_DATA  0x06    /* Firmware data chunk */
#define ESP_CMD_FW_UPDATE_COMMIT 0x07   /* Mark staged firmware, reboot */
#define ESP_CMD_STATUS          0x08    /* Request device status */
#define ESP_CMD_FRAMEBUFFER     0x09    /* Request current framebuffer */
#define ESP_CMD_BUTTON          0x0A    /* Simulate button press */
#define ESP_CMD_SIGNAL_CONFIG   0x0B    /* Set signal injection config */
#define ESP_CMD_MODULE_LIST     0x0C    /* List installed modules */
#define ESP_CMD_MODULE_DELETE   0x0D    /* Delete a module by slot */
#define ESP_CMD_GET_METER       0x21    /* remote_protocol.md §3.4: one meter reading */
#define ESP_CMD_GET_WAVEFORM    0x22    /* §3.4: u8 channel_mask -> one WAVEFORM_FRAME per channel */
#define ESP_CMD_GET_SCOPE_STATE 0x23    /* §3.4 (M6): -> SCOPE_STATE */
#define ESP_CMD_SET_MODE        0x24    /* u8 device_mode 0..3 */
#define ESP_CMD_SET_TIMEBASE    0x25    /* u8 reg 0x01 code 0..ESP_TIMEBASE_CODES-1 */
#define ESP_CMD_SET_VDIV        0x26    /* u8 channel 1|2, u8 range 0..ESP_VDIV_RANGES-1 */
#define ESP_CMD_SET_TRIGGER     0x27    /* u8 mode, u8 edge, u8 source, i16 level */
#define ESP_CMD_RUN_STOP        0x29    /* u8 running 0|1 */
#define ESP_CMD_GET_DESCRIPTION 0x2A    /* stage-086: -> DESCRIPTION (layout below) */
#define ESP_CMD_SET_PROBE       0x2B    /* stage-086: u8 channel 1|2, u8 factor 1|10 */

/* Responses: GD32 → ESP32 */
#define ESP_RSP_ACK             0x81    /* Command accepted */
#define ESP_RSP_NAK             0x82    /* Command rejected */
#define ESP_RSP_DATA            0x83    /* Data response */
#define ESP_RSP_FRAMEBUFFER     0x84    /* Framebuffer data (multi-packet) */
#define ESP_RSP_STATUS          0x85    /* Status response */
#define ESP_RSP_MODULE_LIST     0x86    /* Module list response */
#define ESP_RSP_METER_FRAME     0x90    /* §3.5 METER_FRAME */
#define ESP_RSP_WAVEFORM_FRAME  0x91    /* §3.5 WAVEFORM_FRAME */
#define ESP_RSP_SCOPE_STATE     0x92    /* §3.4 SCOPE_STATE (layout below) */
#define ESP_RSP_DESCRIPTION     0x94    /* stage-086 DESCRIPTION (0x93 is §3.4's planned EVENT) */

/* NAK error codes */
#define ESP_ERR_UNKNOWN_CMD     0x01
#define ESP_ERR_BAD_CHECKSUM    0x02
#define ESP_ERR_BAD_LENGTH      0x03
#define ESP_ERR_FLASH_WRITE     0x04
#define ESP_ERR_FLASH_FULL      0x05
#define ESP_ERR_INVALID_SLOT    0x06
#define ESP_ERR_NOT_READY       0x07
#define ESP_ERR_TRANSFER_ACTIVE 0x08
#define ESP_ERR_UNSUPPORTED     0x09    /* command exists but is not implemented on this build */
#define ESP_ERR_RESERVED_0A     0x0A    /* reserved: never sent (a gap timeout is silent, see esp_comm_rx_poll) */
#define ESP_ERR_NO_CAPTURE_DATA 0x0B    /* remote_protocol.md §3.4 — never substitute the demo trace */
#define ESP_ERR_UNSUPPORTED_IN_MODE 0x0C
#define ESP_ERR_BAD_ARG         0x0D    /* argument out of range */

/* ─── Remote protocol (issue #10, docs/design/remote_protocol.md) ───
 *
 * Wire-format version reported in STATUS byte 0. Bump the major on any
 * incompatible change; the host refuses unknown majors (§3.7). */
#define ESP_PROTO_VERSION       1

/* A frame whose bytes stop arriving for this long is abandoned and the
 * receiver resyncs. Without it a truncated packet leaves the parser in
 * PAYLOAD and, on the shared CDC endpoint, it would swallow the operator's
 * shell text (and any later 0xAA) as payload. USB delivers a host write in
 * back-to-back 64-byte packets, so a real gap of this size means the host
 * gave up, not that it is slow. */
#define ESP_RX_GAP_MS           50

/* STATUS payload v1 — explicit little-endian layout, NOT a C struct dump
 * (a struct's padding and enum width depend on the compiler):
 *   [0]    u8   proto_version   (ESP_PROTO_VERSION)
 *   [1]    u8   current_mode    (device_mode_t: 0 scope 1 meter 2 siggen 3 settings)
 *   [2]    u8   battery_pct     (0..100)
 *   [3]    u8   flags           bit0 charging, bit1 capture data ready,
 *                               bit2 battery critical, bit3 battery UNKNOWN
 *                               (no sample yet: battery_pct/mv are 0 and
 *                               must not be read as a measurement)
 *   [4..5] u16  battery_mv
 *   [6..9] u32  uptime_ms
 *   [10..13] u32 usb_tx_stalls  (CDC IN waits that timed out, issue #39)
 *   [14..15] u16 usb_heals      (transport self-heal reconnects, issue #39)
 *   [16]   u8   fw_len
 *   [17..] char fw_version[fw_len]  (no NUL)
 */
#define ESP_STATUS_FIXED_LEN    17
#define ESP_FW_VERSION_MAX      48      /* bytes of fw_version sent; also sizes the STATUS buffer */
#define ESP_STATUS_FLAG_CHARGING      0x01
#define ESP_STATUS_FLAG_CAPTURE_READY 0x02
#define ESP_STATUS_FLAG_BATT_CRITICAL 0x04
#define ESP_STATUS_FLAG_BATT_UNKNOWN  0x08

/* Snapshot the firmware fills in for STATUS/PING. esp_comm itself knows
 * nothing about the device, so the host tests can inject any state. */
typedef struct {
    uint8_t     current_mode;
    uint8_t     battery_pct;
    uint8_t     flags;
    uint16_t    battery_mv;
    uint32_t    uptime_ms;
    uint32_t    usb_tx_stalls;
    uint16_t    usb_heals;
    const char *fw_version;     /* NUL-terminated; truncated to ESP_FW_VERSION_MAX on the wire */
} esp_status_snapshot_t;

typedef void (*esp_status_fn)(esp_status_snapshot_t *out);

/* METER_FRAME payload v1 (remote_protocol.md §3.5, plus the display text):
 *   [0..3]   u32  update_count   monotonic: a host sees drops and stale reads
 *   [4..7]   f32  value          as the firmware scaled it (IEEE-754 LE)
 *   [8..9]   i16  raw_bcd        the instrument's own digits, uncalibrated —
 *                                per-device cal (#28) means a log should keep
 *                                what the meter saw, not only what we concluded
 *   [10]     u8   decimal_pos
 *   [11]     u8   result_class   meter_result_class_t (NORMAL, OL, …)
 *   [12]     u8   flags          bit0 negative, bit1 ac, bit2 autorange, bit3 hold
 *   [13]     u8   submode
 *   [14]     u8   unit_variant
 *   [15]     u8   unit_len, then unit[unit_len] (ASCII, e.g. "V", "kOhm")
 *   [..]     u8   display_len, then display[display_len] (what the LCD shows)
 */
#define ESP_METER_FIXED_LEN     16
#define ESP_METER_FLAG_NEGATIVE 0x01
#define ESP_METER_FLAG_AC       0x02
#define ESP_METER_FLAG_AUTO     0x04
#define ESP_METER_FLAG_HOLD     0x08

typedef struct {
    uint32_t    update_count;
    float       value;
    int16_t     raw_bcd;
    uint8_t     decimal_pos;
    uint8_t     result_class;
    uint8_t     flags;
    uint8_t     submode;
    uint8_t     unit_variant;
    char        unit[16];       /* copied, NUL-terminated: the provider's own */
    char        display[16];    /* snapshot is gone by the time we encode */
} esp_meter_snapshot_t;

/* Fill a coherent reading. Anything but ESP_METER_OK is answered with a NAK:
 * a reading the instrument is not currently producing must never be sent as
 * if it were live (§2.3). */
typedef enum {
    ESP_METER_OK = 0,
    ESP_METER_NOT_READY,        /* in meter mode, no reading parsed yet */
    ESP_METER_WRONG_MODE,       /* not in meter mode: the last reading is frozen */
} esp_meter_result_t;
typedef esp_meter_result_t (*esp_meter_fn)(esp_meter_snapshot_t *out);

/* WAVEFORM_FRAME payload v1 (remote_protocol.md §3.5, header extended). One
 * frame per requested channel, CH1 first; all frames of one request are ONE
 * capture (same frame_id). Header, little-endian:
 *   [0..3]   u32  frame_id        acquisition generation of the copied record
 *                                 (fpga_acq_frame_generation(): even, +2 per
 *                                 committed capture — the `gen=` that `spi3
 *                                 frame` prints). Never 0: no record, no frame.
 *   [4]      u8   channel         0 = CH1, 1 = CH2
 *   [5]      u8   flags           ESP_WAVE_FLAG_* below — derived HERE from the
 *                                 provider's facts, never copied from it
 *   [6]      u8   timebase_idx    reg 0x01 code of THIS RECORD: the code in
 *                                 force over its whole capture window, published
 *                                 with the record (C-90, stage-240). A record held
 *                                 across a timebase change (STOP, SINGLE, NORMAL
 *                                 with no crossing) keeps the code it was taken
 *                                 at; a read the change overlapped is never
 *                                 published (src/dsp/acq_cfg.h)
 *   [7]      u8   vdiv_idx        relay range index of this channel for THIS
 *                                 record, published with it (stage-240)
 *   [8..9]   u16  sample_count    1024 today
 *   [10..11] u16  header_len      offset of samples[0] (§3.5's "reserved"):
 *                                 a host skips to it, so fields can be appended
 *                                 without a major bump
 *   [12..15] u32  sample_rate_hz  bench-measured rate of this code
 *                                 (scope_timebase.c, bench unit #1), rounded;
 *                                 0 = no trustworthy rate (never measured,
 *                                 incoherent, or display != hardware)
 *   [16..19] u32  uv_per_div      bench-measured volts/div of this channel and
 *                                 range (scope_cal.c, bench unit #1), in uV,
 *                                 TIMES the channel's probe setting (x1/x10:
 *                                 scope_cal_volts_per_div() applies it; the
 *                                 setting itself is not on the wire — stage-080
 *                                 bench, CH1 at x10 read 34 Vpp for a 3.3 V
 *                                 square); 0 = the range has
 *                                 no volts meaning. A GAIN only: the zero
 *                                 point is uncalibrated, so counts->volts
 *                                 gives Vpp, not absolute volts.
 *   [20..21] u16  counts_per_div  ADC counts one division of uv_per_div spans
 *   [22..23] u16  head_skip       samples [0, head_skip) are the known record-
 *                                 head defect (scope_record.h): analyse from here
 *   [24..]   u8   samples[sample_count]  unsigned ADC counts as committed by the
 *                                 acquisition task (a coherent copy, not the
 *                                 live buffer)
 */
#define ESP_WAVE_HDR_LEN        24
#define ESP_WAVE_MAX_SAMPLES    1024    /* one FPGA record (FPGA_ADC_BUF_SIZE) */
#define ESP_WAVE_MASK_CH1       0x01
#define ESP_WAVE_MASK_CH2       0x02
#define ESP_WAVE_MASK_ALL       (ESP_WAVE_MASK_CH1 | ESP_WAVE_MASK_CH2)

#define ESP_WAVE_FLAG_CALIBRATED   0x01  /* per-unit calibration: always 0 until
                                          * that work is bench-validated (§3.5) */
#define ESP_WAVE_FLAG_TB_MEASURED  0x02  /* sample_rate_hz is tier MEASURED   */
#define ESP_WAVE_FLAG_VDIV_MEASURED 0x04 /* uv_per_div is tier MEASURED       */
#define ESP_WAVE_FLAG_SYNTHETIC    0x08  /* never set: a synthetic record is
                                          * refused, not labelled (§2.3)      */
#define ESP_WAVE_FLAG_TB_PROVISIONAL   0x10  /* rate is right order of magnitude
                                              * only ('~' on the LCD)         */
#define ESP_WAVE_FLAG_VDIV_PROVISIONAL 0x20  /* same for uv_per_div           */
#define ESP_WAVE_FLAG_TIME_ORDERED 0x40  /* record un-rotated at its seam: the
                                          * hardware trigger sits at index 512 */
#define ESP_WAVE_FLAG_TB_DISAGREES 0x80  /* the display's timebase is not the
                                          * one in force: rate withheld (0)   */

/* Confidence of a measured number — numerically identical to scope_cal_tier_t
 * and scope_tb_tier_t (usb_debug.c asserts it). */
#define ESP_TIER_NONE           0
#define ESP_TIER_PROVISIONAL    1
#define ESP_TIER_MEASURED       2

typedef struct {
    const uint8_t *samples;     /* a coherent COPY of the record, or NULL */
    uint16_t    count;          /* 1..ESP_WAVE_MAX_SAMPLES */
    uint8_t     vdiv_idx;
    uint8_t     vdiv_tier;      /* ESP_TIER_* of uv_per_div */
    uint32_t    uv_per_div;     /* 0 = no volts meaning */
} esp_wave_channel_t;

typedef struct {
    uint32_t    frame_id;       /* generation of the copy; 0 = none (refused) */
    uint8_t     timebase_idx;   /* code in force */
    uint8_t     timebase_tier;  /* ESP_TIER_* of sample_rate_hz */
    bool        timebase_disagrees;
    bool        time_ordered;
    bool        synthetic;      /* anything but a real capture: the encoder refuses */
    uint32_t    sample_rate_hz;
    uint16_t    counts_per_div;
    uint16_t    head_skip;
    esp_wave_channel_t ch[2];   /* [0] CH1, [1] CH2; only requested ones are read */
} esp_wave_snapshot_t;

/* Fill a coherent snapshot of the channels in `channel_mask`. Anything but
 * ESP_WAVE_OK is answered with a NAK — a record the instrument is not
 * currently producing must never be sent as if it were (§2.3). */
typedef enum {
    ESP_WAVE_OK = 0,
    ESP_WAVE_NO_DATA,           /* fpga_data_ready() false / no committed record */
    ESP_WAVE_WRONG_MODE,        /* not in scope mode: the buffers are not live */
    ESP_WAVE_BUSY,              /* no tear-free copy within the retry budget */
} esp_wave_result_t;
typedef esp_wave_result_t (*esp_wave_fn)(uint8_t channel_mask, esp_wave_snapshot_t *out);
/* ─── Control surface (M6, remote_protocol.md §3.4 codes 0x23-0x29) ───
 *
 * Set payloads, little-endian. esp_comm validates the length and EVERY field
 * before the provider is called, so a refused request changes nothing:
 *   SET_MODE      u8 mode     0 scope, 1 meter, 2 siggen, 3 settings
 *   SET_TIMEBASE  u8 code     the SPI reg 0x01 code, 0..ESP_TIMEBASE_CODES-1
 *                             (scope_state.timebase_idx IS this code; §3.4's
 *                             "index into timebase_table[]" predates that)
 *   SET_VDIV      u8 channel (1|2), u8 range 0..ESP_VDIV_RANGES-1
 *   SET_TRIGGER   u8 mode (0 auto, 1 normal, 2 single), u8 edge (0 rising,
 *                 1 falling), u8 source (0 CH1, 1 CH2), i16 level -100..100
 *   RUN_STOP      u8 running (0 stop, 1 run)
 *   SET_PROBE     u8 channel (1|2), u8 factor (1|10)              (stage-086)
 * Replies: ACK once the setting is in force. NAK BAD_LENGTH / BAD_ARG for a
 * malformed request (provider not called), NOT_READY when the provider could
 * not apply it (nothing changed), UNSUPPORTED_IN_MODE when the provider needs
 * another mode, UNSUPPORTED when no provider is bound.
 *
 * SCOPE_STATE payload v1, explicit little-endian layout (not a struct dump):
 *   [0]      u8   version         ESP_SCOPE_STATE_VERSION
 *   [1]      u8   length          bytes in this payload; fields may be appended
 *   [2]      u8   mode            current device mode
 *   [3]      u8   running         scope_state.running (RUN/STOP)
 *   [4]      u8   timebase_idx    the display's code (labels the axis)
 *   [5]      u8   timebase_code   reg 0x01 code IN FORCE in the FPGA; [4] != [5]
 *                                 is the 2026-08-19 divergence, made visible
 *   [6]      u8   timebase_tier   ESP_TIER_* of the code in force
 *   [7]      u8   trig_mode
 *   [8]      u8   trig_edge
 *   [9]      u8   trig_source
 *   [10..11] i16  trig_level      -100..100 (the UI's units)
 *   [12..15] u32  sample_rate_hz  bench-measured rate of the code in force;
 *                                 0 = no trustworthy rate (tier NONE)
 *   [16]     u8   trig_code       reg 0x08 code IN FORCE
 *   [17]     u8   reserved (0)
 *   [18..19] u16  counts_per_div
 *   [20..23] CH1: u8 enabled, u8 vdiv_idx, u8 vdiv_tier, u8 reserved
 *   [24..27] CH1: u32 uv_per_div  (0 = the range has no volts meaning; includes
 *                                 the probe setting, as the waveform header does)
 *   [28..31] CH2: as [20..23]
 *   [32..35] CH2: u32 uv_per_div
 *   [36]     u8   CH1 probe factor, 1 or 10 (stage-086 append; still v1: a
 *   [37]     u8   CH2 probe factor   host reads [1] and ignores bytes it does
 *                                    not know; a 36-byte reply has no probe)
 *   uv_per_div keeps its meaning (probe included), so an old host sees no change.
 */
#define ESP_SCOPE_STATE_VERSION 1
#define ESP_SCOPE_STATE_LEN     38
#define ESP_TIMEBASE_CODES      21      /* SCOPE_TIMEBASE_CODE_COUNT (usb_debug.c asserts) */
#define ESP_VDIV_RANGES         10      /* VDIV_COUNT (usb_debug.c asserts) */
#define ESP_TRIG_LEVEL_MAX      100

typedef struct {
    bool        enabled;
    uint8_t     vdiv_idx;
    uint8_t     vdiv_tier;      /* ESP_TIER_* of uv_per_div */
    uint32_t    uv_per_div;
    uint8_t     probe;          /* factor 1 or 10 (anything else is sent as 1) */
} esp_scope_channel_t;

typedef struct {
    uint8_t     mode;
    bool        running;
    uint8_t     timebase_idx;   /* display */
    uint8_t     timebase_code;  /* in force */
    uint8_t     timebase_tier;  /* of the code in force */
    uint32_t    sample_rate_hz; /* of the code in force */
    uint8_t     trig_mode, trig_edge, trig_source;
    int16_t     trig_level;
    uint8_t     trig_code;      /* reg 0x08 in force */
    uint16_t    counts_per_div;
    esp_scope_channel_t ch[2];
} esp_scope_state_t;

/* ─── Device description (stage-086, GET_DESCRIPTION 0x2A -> 0x94) ───
 *
 * Everything a host needs to print practical values instead of codes, in one
 * request. Payload, little-endian:
 *   [0]  u8 version (ESP_DESCRIPTION_VERSION), then sections to the end:
 *        u8 id, u16 len, u8 data[len]
 * A host SKIPS a section id it does not know (len tells it how far), so a
 * later stage appends a section without a version change. Sections:
 *   1 timebase  u16 samples_per_div, u8 n, then per reg 0x01 code 0..n-1:
 *               u32 sample_rate_hz (0 = no rate), u8 tier
 *   2 ranges    u16 counts_per_div, u8 n (ranges per channel), u8 channels,
 *               then per channel (CH1 first), per range 0..n-1:
 *               u32 uv_per_div_bnc (0 = no volts meaning), u8 tier
 *               -- at the BNC, NO probe factor (scope_cal_range_label_bnc())
 *   3 trigger   i16 level_min, i16 level_max, u8 center_code, u16 num,
 *               u16 den, u8 code_min, u8 code_max, i8 record_offset
 *               reg 0x08 code = clamp(center_code + trunc(level*num/den),
 *               code_min, code_max), level clamped to [min, max] first
 *               (fpga_trigger_code_from_level()); the capture fires where the
 *               record crosses code + record_offset (EXP-56: -28)
 *   4 probe     per channel (CH1 first): u8 factor (1 or 10)
 *   7 identity  (stage-190) u8 uid[12] (MCU unique ID: the three words at
 *               0x1FFFF7E8 / EC / F0, each little-endian, as `mem read`
 *               shows them), u16 api_level (ESP_API_LEVEL), u8 target_len,
 *               char target[target_len], u8 rev_len, char rev[rev_len]
 *               (no NUL; each text cut to ESP_DESC_TEXT_MAX). Fields added
 *               later go at the end. Ids 5 and 6 are kept for the meter and
 *               the generator (stages 272, 282).
 * The encoder zeroes a value whose tier is NONE and sends tier NONE for a
 * zero value, as everywhere else on this link. */
#define ESP_DESCRIPTION_VERSION 1
#define ESP_DESC_SEC_TIMEBASE   1
#define ESP_DESC_SEC_RANGES     2
#define ESP_DESC_SEC_TRIGGER    3
#define ESP_DESC_SEC_PROBE      4
#define ESP_DESC_TB_LEN         (3u + ESP_TIMEBASE_CODES * 5u)          /* 108 */
#define ESP_DESC_RANGES_LEN     (4u + 2u * ESP_VDIV_RANGES * 5u)        /* 104 */
#define ESP_DESC_TRIGGER_LEN    12u
#define ESP_DESC_PROBE_LEN      2u
#define ESP_DESC_SEC_IDENTITY   7
#define ESP_DESC_TEXT_MAX       31u     /* target and rev bytes on the wire */
#define ESP_DESC_UID_LEN        12u
#define ESP_DESC_IDENTITY_MAX   (ESP_DESC_UID_LEN + 2u + 1u + ESP_DESC_TEXT_MAX + 1u + ESP_DESC_TEXT_MAX) /* 78 */
/* The largest reply: version + 5 section headers + the sections = 320. The
 * identity section is shorter when the texts are: the reply length is the
 * bytes written. Sections 1-4 alone are 239 B (stage-086). */
#define ESP_DESCRIPTION_LEN     (1u + 5u * 3u + ESP_DESC_TB_LEN + ESP_DESC_RANGES_LEN + \
                                 ESP_DESC_TRIGGER_LEN + ESP_DESC_PROBE_LEN + ESP_DESC_IDENTITY_MAX)

/* Link API level (stage-190, remote_protocol.md §9.2). A host reads "no
 * section 7" as level 0. Raise it by 1 when the link gets a command, a section
 * or a field that the GUI uses. Never lower it. */
#define ESP_API_LEVEL           1

typedef struct {
    uint16_t samples_per_div;
    uint32_t tb_rate_hz[ESP_TIMEBASE_CODES];
    uint8_t  tb_tier[ESP_TIMEBASE_CODES];
    uint16_t counts_per_div;
    uint32_t uv_per_div_bnc[2][ESP_VDIV_RANGES];
    uint8_t  vdiv_tier[2][ESP_VDIV_RANGES];
    int16_t  trig_level_min, trig_level_max;
    uint8_t  trig_center_code;
    uint16_t trig_num, trig_den;
    uint8_t  trig_code_min, trig_code_max;
    int8_t   trig_record_offset;
    uint8_t  probe[2];          /* 1 or 10 */
    /* stage-190: section 7. NULL texts are sent empty. */
    uint8_t     uid[ESP_DESC_UID_LEN];
    const char *build_target;   /* NUL-terminated make goal, e.g. "guest-coldtrace" */
    const char *git_rev;        /* NUL-terminated, e.g. "0a1af35" or "0a1af35-dirty" */
} esp_description_t;

typedef enum {
    ESP_CTRL_OK = 0,
    ESP_CTRL_NOT_READY,         /* could not apply (e.g. acq task would not park): nothing changed */
    ESP_CTRL_WRONG_MODE,        /* the setting needs another device mode */
} esp_ctrl_result_t;

/* All members may be NULL: the matching command then answers UNSUPPORTED.
 * Arguments are already range-checked. A provider must change nothing when
 * it returns anything but ESP_CTRL_OK. */
typedef struct {
    void              (*get_state)(esp_scope_state_t *out);
    esp_ctrl_result_t (*set_mode)(uint8_t mode);
    esp_ctrl_result_t (*set_timebase)(uint8_t code);
    esp_ctrl_result_t (*set_vdiv)(uint8_t channel, uint8_t range);
    esp_ctrl_result_t (*set_trigger)(uint8_t mode, uint8_t edge, uint8_t source, int16_t level);
    esp_ctrl_result_t (*run_stop)(bool running);
    /* stage-086. Appended: an older table that stops at run_stop stays valid. */
    void              (*get_description)(esp_description_t *out);
    esp_ctrl_result_t (*set_probe)(uint8_t channel, uint8_t factor);
} esp_control_ops_t;
/* Inject a button press (id 1..15 = button_id_t). Return false if it could
 * not be queued, so the host gets NAK instead of a false ACK. */
typedef bool (*esp_button_fn)(uint8_t button_id);
/* Block writer: the whole of `len` bytes, in order. Preferred over the
 * byte writer — the USB CDC path sends 64-byte packets, not bytes.
 * Returns false if not all bytes went out (host not reading, port closed):
 * the sender then stops the frame instead of waiting again for each part. */
typedef bool (*esp_write_block_fn)(const uint8_t *data, uint16_t len);
/* Where non-protocol bytes go when routing a shared stream (§3.2). */
typedef void (*esp_passthrough_fn)(const uint8_t *data, uint16_t len, void *ctx);

/* Module slots */
#define ESP_MODULE_SLOT_COUNT   4
#define ESP_MODULE_MAX_SIZE     (1024 * 1024)   /* 1MB per slot */

/* Button IDs (matches button_id_t in ui.h) */
#define ESP_BTN_CH1     1
#define ESP_BTN_CH2     2
#define ESP_BTN_MOVE    3
#define ESP_BTN_SELECT  4
#define ESP_BTN_TRIGGER 5
#define ESP_BTN_PRM     6
#define ESP_BTN_AUTO    7
#define ESP_BTN_SAVE    8
#define ESP_BTN_MENU    9
#define ESP_BTN_UP      10
#define ESP_BTN_DOWN    11
#define ESP_BTN_LEFT    12
#define ESP_BTN_RIGHT   13
#define ESP_BTN_OK      14
#define ESP_BTN_POWER   15

/* Firmware update state */
typedef enum {
    FW_UPDATE_IDLE = 0,
    FW_UPDATE_RECEIVING,
    FW_UPDATE_STAGED,
    FW_UPDATE_FAILED,
} fw_update_state_t;

/* Module slot info */
typedef struct {
    char     name[32];
    char     version[16];
    uint32_t size;
    bool     installed;
} module_slot_info_t;

/* Device status (sent in response to STATUS command) */
typedef struct {
    char     fw_version[16];
    uint8_t  current_mode;
    uint8_t  battery_pct;
    uint8_t  num_modules;
    fw_update_state_t fw_state;
} device_status_t;

/* Received packet (parsed) */
typedef struct {
    uint8_t  cmd;
    uint16_t payload_len;
    uint8_t  payload[ESP_MAX_PAYLOAD];
    bool     valid;
} esp_packet_t;

/* ─── Protocol API ─── */

/* Initialize the ESP32 communication handler */
void esp_comm_init(void);

/* Process one byte from UART RX. Call this from UART ISR or polling loop.
 * Returns true when a complete valid packet is ready. */
bool esp_comm_receive_byte(uint8_t byte);

/* Get the last received packet (valid after esp_comm_receive_byte returns true) */
const esp_packet_t *esp_comm_get_packet(void);

/* Process the received packet and generate response.
 * This is the main command dispatcher. */
void esp_comm_process(const esp_packet_t *pkt);

/* Send a response packet over UART.
 * write_byte: function pointer to UART TX byte function */
typedef void (*esp_write_fn)(uint8_t byte);

void esp_comm_set_writer(esp_write_fn fn);

/* Send a raw response */
void esp_comm_send_response(uint8_t cmd, const uint8_t *payload, uint16_t len);

/* Send ACK */
void esp_comm_send_ack(void);

/* Send NAK with error code */
void esp_comm_send_nak(uint8_t error_code);

/* Check if a module transfer or firmware update is in progress */
bool esp_comm_transfer_active(void);

/* ─── Remote protocol bindings ─── */

void esp_comm_set_block_writer(esp_write_block_fn fn);
void esp_comm_set_status_provider(esp_status_fn fn);
void esp_comm_set_button_injector(esp_button_fn fn);
void esp_comm_set_meter_provider(esp_meter_fn fn);
void esp_comm_set_waveform_provider(esp_wave_fn fn);
/* The ops table is used by reference: it must outlive the binding (static). */
void esp_comm_set_control_provider(const esp_control_ops_t *ops);

/* Remote badge (stage-086): true while the last VALID frame arrived less
 * than ESP_HOST_ACTIVE_MS ago, on esp_comm_route()'s clock. Shell text does
 * not count: the badge means "a program drives the unit". */
#define ESP_HOST_ACTIVE_MS      3000u
bool esp_comm_host_active(uint32_t now_ms);

/* True while a frame is being received (sync seen, checksum not yet). */
bool esp_comm_rx_in_frame(void);

/* Abandon a frame whose bytes stopped arriving ESP_RX_GAP_MS ago.
 * Returns true if a frame was abandoned. Nothing is sent: without request
 * ids an unsolicited reply would be mistaken for the next request's answer. */
bool esp_comm_rx_poll(uint32_t now_ms);

/* Refresh the gap clock of a frame still open (no-op otherwise). Call after
 * esp_comm_route() returns, with a fresh time: route() stamps a whole chunk
 * with its entry time, and work inside the chunk may have taken a while. */
void esp_comm_rx_touch(uint32_t now_ms);

/* Route a chunk from a stream shared with the ASCII shell (§3.2):
 * bytes belonging to a frame (starting at 0xAA) go to the parser and every
 * completed frame is dispatched; all other bytes are handed, in order and
 * in contiguous runs, to `passthrough`. Malformed frames are answered with
 * NAK (bad checksum / bad length) rather than dropped silently. */
void esp_comm_route(const uint8_t *data, uint16_t len, uint32_t now_ms,
                    esp_passthrough_fn passthrough, void *ctx);

/* Diagnostics for `usbstat`/tests. */
typedef struct {
    uint32_t frames_ok;
    uint32_t bad_checksum;
    uint32_t bad_length;
    uint32_t gap_timeouts;
} esp_rx_stats_t;
void esp_comm_get_rx_stats(esp_rx_stats_t *out);

/* Compute XOR checksum */
uint8_t esp_comm_checksum(const uint8_t *data, uint16_t len);

#endif /* ESP_COMM_H */
