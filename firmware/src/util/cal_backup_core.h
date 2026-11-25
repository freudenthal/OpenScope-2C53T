/*
 * cal_backup_core.h — pure record/decision logic for factory-cal self-protection.
 *
 * WHAT THIS IS
 * ---------------------------------------------------------------------------
 * The per-device factory calibration lives in MCU-internal flash at 0x08006000
 * (a 4 KB page — see cal_dump.h and factory_cal_truth_2026-08-14.md). It is
 * NOT regenerable: every unit's page differs and there is no factory source we
 * can re-download. Our own reflashing is the standing hazard to it.
 *
 * This module is the HARDWARE-FREE half of the backup mechanism: it builds and
 * validates a versioned, CRC-protected backup record wrapping the raw page, and
 * it makes the decisions that must never be wrong — "which W25Q slot may this
 * backup go to?" and "is it safe to write this backup back over the live
 * page?". Keeping it free of the AT32 HAL and the W25Q driver is what lets it
 * be unit-tested on the host (tests/test_cal_backup.c).
 *
 * The device glue (reading the UID, W25Q slot I/O, erasing and programming MCU
 * flash on restore) is in cal_backup.c. The glue feeds the slot bytes through
 * cal_slot_scan_*() and acts only on what cal_backup_plan() and
 * cal_backup_restore_plan() return.
 *
 * TWO SLOTS (stage-175, review F-2)
 *   slot A  region "calbackup"   0xFFD000, 8 KB. The first valid backup. Once it
 *           is valid, `cal backup` never erases it.
 *   slot B  region "calbackup_b" 0xFFB000, 8 KB. Later copies.
 * A capture whose cal block (page bytes 0x025-0x12E, C-40) differs from slot A
 * is refused unless forced, and force writes slot B only.
 *
 * RECORD LAYOUT (little-endian)
 *   offset  size  field
 *   0       4     magic         CAL_BACKUP_MAGIC
 *   4       2     version       1 or 2
 *   6       2     payload_len   bytes of payload following the header
 *   8       4     src_addr      provenance: 0x08006000 (informational)
 *   12      4     payload_crc   CRC32 of the payload
 *   16      4     header_crc    v1: CRC32 of bytes [0,16)
 *                               v2: CRC32 of bytes [0,16) then [20,36)
 *   v1: 20  ...   payload
 *   v2: 20  12    uid           MCU unique ID, 3 words at 0x1FFFF7E8, LE
 *       32  4     block_crc     CRC32 of payload bytes 0x025-0x12E
 *       36  ...   payload
 * A v1 record has no UID ("unknown"): it matches every unit.
 */

#ifndef CAL_BACKUP_CORE_H
#define CAL_BACKUP_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define CAL_BACKUP_MAGIC        0x4C414342u   /* 'B','C','A','L' little-endian */
#define CAL_BACKUP_VERSION_1    1u
#define CAL_BACKUP_VERSION_2    2u
#define CAL_BACKUP_VERSION      CAL_BACKUP_VERSION_2   /* the version we write */

/* The MCU-internal saved-config page this module protects. */
#define CAL_BACKUP_SRC_ADDR     0x08006000u
#define CAL_BACKUP_PAYLOAD_LEN  0x1000u        /* 4 KB — the whole page, verbatim */

#define CAL_BACKUP_HEADER_LEN_V1  20u
#define CAL_BACKUP_HEADER_LEN_V2  36u
#define CAL_BACKUP_HEADER_MAX     CAL_BACKUP_HEADER_LEN_V2
#define CAL_BACKUP_RECORD_MAX     (CAL_BACKUP_HEADER_MAX + CAL_BACKUP_PAYLOAD_LEN)

/* MCU unique ID: 3 words (UID_1..3 at 0x1FFFF7E8/EC/F0), stored LE. */
#define CAL_UID_LEN             12u
/* The cal block of the page, 0x025-0x12E inclusive (C-40). */
#define CAL_BLOCK_OFF           0x025u
#define CAL_BLOCK_LEN           0x10Au

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_len;
    uint32_t src_addr;
    uint32_t payload_crc;
    uint32_t header_crc;
    uint16_t header_len;              /* 20 (v1) or 36 (v2): payload offset  */
    bool     uid_known;               /* v2 only                             */
    uint8_t  uid[CAL_UID_LEN];        /* v2 only; zero for v1                */
    uint32_t block_crc;               /* v2 only; zero for v1                */
} cal_backup_header_t;

/* Result of parsing/validating a stored record. */
typedef enum {
    CAL_REC_OK = 0,
    CAL_REC_ERR_TRUNCATED,      /* fewer than a header's worth of bytes         */
    CAL_REC_ERR_MAGIC,          /* magic mismatch (blank/foreign/erased slot)   */
    CAL_REC_ERR_VERSION,        /* version this build does not understand       */
    CAL_REC_ERR_LEN,            /* payload_len is 0 or exceeds the buffer        */
    CAL_REC_ERR_HEADER_CRC,     /* header corrupt                                */
    CAL_REC_ERR_PAYLOAD_CRC,    /* payload corrupt                               */
    CAL_REC_ERR_BLOCK_CRC,      /* v2 block_crc does not match the payload       */
} cal_rec_status_t;

/* How a page (or the cal block of a page) reads. */
typedef enum {
    CAL_PAGE_BLANK = 0,         /* every byte 0xFF — erased, nothing there       */
    CAL_PAGE_ZEROED,            /* every byte 0x00 — wiped, nothing there        */
    CAL_PAGE_PROGRAMMED,        /* has real content — treat as precious          */
} cal_page_class_t;

/* CRC32/IEEE-802.3, reflected, init 0xFFFFFFFF, final XOR 0xFFFFFFFF — the same
 * parameters flash_regions.c and cal_dump.c use. */
uint32_t cal_backup_crc32(const void *data, uint32_t len);

/* Incremental form: seed with CAL_BACKUP_CRC32_INIT, fold each chunk with
 * _update, finalise with _final. */
#define CAL_BACKUP_CRC32_INIT  0xFFFFFFFFu
uint32_t cal_backup_crc32_update(uint32_t crc, const void *data, uint32_t len);
uint32_t cal_backup_crc32_final(uint32_t crc);

/* Header length for a version: 20 (v1), 36 (v2), 0 (unknown). */
uint16_t cal_backup_header_len(uint16_t version);

/* Serialise a record header (including header_crc) into out, which must hold
 * cal_backup_header_len(version) bytes. v2 needs uid (non-NULL). Returns the
 * header length written, or 0 on a bad argument. */
uint32_t cal_backup_write_header(uint8_t *out, uint16_t version,
                                 uint16_t payload_len, uint32_t src_addr,
                                 uint32_t payload_crc,
                                 const uint8_t *uid, uint32_t block_crc);

/* Validate the header alone: magic, version, header_crc, payload_len != 0. Does
 * NOT check the payload. Fills *hdr_out on OK. */
cal_rec_status_t cal_backup_check_header(const uint8_t *rec, uint32_t rec_len,
                                         cal_backup_header_t *hdr_out);

/* Build a record for `payload` into `out`. v2 needs uid and a payload that
 * holds the cal block. Returns the total record length, or 0 on bad argument. */
uint32_t cal_backup_build_record(const void *payload, uint16_t payload_len,
                                 uint16_t version, uint32_t src_addr,
                                 const uint8_t *uid,
                                 uint8_t *out, uint32_t out_cap);

/* Parse and fully validate `rec` (header, payload CRC, v2 block CRC). On OK,
 * *hdr_out is filled and *payload_out points at the payload inside `rec`. */
cal_rec_status_t cal_backup_parse(const uint8_t *rec, uint32_t rec_len,
                                  cal_backup_header_t *hdr_out,
                                  const uint8_t **payload_out);

/* Classify bytes already in memory. */
cal_page_class_t cal_backup_classify_page(const uint8_t *page, uint32_t len);
/* Class of two parts read one after the other. */
cal_page_class_t cal_backup_class_merge(cal_page_class_t a, cal_page_class_t b);

/* ── Live page summary ──────────────────────────────────────────────── */

typedef struct {
    cal_page_class_t page_class;      /* whole 4 KB                           */
    cal_page_class_t block_class;     /* bytes 0x025-0x12E                    */
    uint32_t         payload_crc;     /* CRC32 of the 4 KB                    */
    uint32_t         block_crc;       /* CRC32 of 0x025-0x12E                 */
} cal_live_info_t;

/* `page` holds CAL_BACKUP_PAYLOAD_LEN bytes. On the device it is the live page
 * at 0x08006000 itself (memory-mapped; no copy). */
void cal_backup_live_info(const uint8_t *page, cal_live_info_t *out);

/* ── Slot summary, built by streaming the slot bytes ────────────────── */

typedef enum {
    CAL_SLOT_EMPTY = 0,         /* header bytes all 0xFF: no record             */
    CAL_SLOT_VALID,             /* header and payload validate                  */
    CAL_SLOT_BAD,               /* bytes present but no valid record            */
} cal_slot_state_t;

typedef struct {
    cal_slot_state_t  state;
    cal_rec_status_t  rec_status;     /* why BAD; CAL_REC_OK when VALID        */
    cal_backup_header_t hdr;          /* filled when the header validated     */
    uint32_t          block_crc;      /* computed from the stored payload      */
    cal_page_class_t  block_class;    /* class of the stored cal block         */
    bool              block_equal_live; /* stored cal block == live, bytewise  */
    bool              page_equal_live;  /* stored payload == live page         */
} cal_slot_info_t;

typedef struct {
    cal_slot_info_t info;
    const uint8_t  *live;             /* live page, or NULL (equal = false)    */
    uint32_t        fed;              /* payload bytes fed so far              */
    uint32_t        crc;
    uint32_t        block_crc;
    bool            page_eq;
    bool            block_eq;
    bool            block_first;
    bool            want_payload;
} cal_slot_scan_t;

/* Start a scan with the first `n` bytes of the slot (read n =
 * CAL_BACKUP_HEADER_MAX). Returns true when the header validated and the
 * caller must feed the payload: payload_len bytes from slot offset
 * info.hdr.header_len, in order, in chunks of any size. */
bool cal_slot_scan_begin(cal_slot_scan_t *s, const uint8_t *head, uint32_t n,
                         const uint8_t *live);
void cal_slot_scan_feed(cal_slot_scan_t *s, const uint8_t *chunk, uint32_t n);
void cal_slot_scan_end(cal_slot_scan_t *s, cal_slot_info_t *out);

/* Host helper: scan a whole slot image in memory (chunk = feed size). */
void cal_slot_scan_buffer(const uint8_t *slot, uint32_t slot_len,
                          const uint8_t *live, uint32_t chunk,
                          cal_slot_info_t *out);

/* ── Store decision: where may `cal backup` write? ──────────────────── */

typedef enum {
    CAL_PLAN_REFUSE = 0,
    CAL_PLAN_WRITE_A,
    CAL_PLAN_WRITE_B,
    CAL_PLAN_NONE_EQUAL,        /* slot B already holds this page: no write     */
} cal_plan_action_t;

typedef enum {
    CAL_WHY_NONE = 0,
    CAL_WHY_LIVE_BLANK,         /* live page all 0xFF or all 0x00               */
    CAL_WHY_LIVE_BLOCK_BLANK,   /* live cal block all 0xFF or all 0x00          */
    CAL_WHY_A_DAMAGED,          /* slot A holds bytes but no valid record       */
    CAL_WHY_UID_MISMATCH,       /* the backup has another unit's UID            */
    CAL_WHY_BLOCK_DIFFERS,      /* live cal block differs from slot A           */
    CAL_WHY_NO_BACKUP,          /* restore: no valid record in the slot         */
    CAL_WHY_BACKUP_BLOCK_BLANK, /* restore: stored cal block all 0xFF/0x00      */
    CAL_WHY_LIVE_PRECIOUS,      /* restore: live page programmed, no force      */
    CAL_WHY_BAD_ARG,
} cal_plan_why_t;

typedef struct {
    cal_plan_action_t action;
    cal_plan_why_t    why;
} cal_plan_t;

/* Rules, in order:
 *   live page or live cal block blank/zeroed      -> refuse
 *   slot A empty                                  -> write A
 *   slot A bad                                    -> refuse (A is never erased)
 *   slot A UID known and != uid                   -> refuse (force too)
 *   live cal block == slot A's, or force          -> write B; no write when
 *                                                    B already holds this page
 *   live cal block != slot A's, no force          -> refuse
 * Slot A is written only when it is EMPTY. */
cal_plan_t cal_backup_plan(const cal_live_info_t *live,
                           const cal_slot_info_t *a,
                           const cal_slot_info_t *b,
                           const uint8_t *uid, bool force);

/* ── Restore decision (host tests only on this project: I-40) ───────── */

typedef enum {
    CAL_RESTORE_REFUSE = 0,
    CAL_RESTORE_WRITE,
    CAL_RESTORE_NONE_EQUAL,     /* live page already equals the backup          */
} cal_restore_action_t;

typedef struct {
    cal_restore_action_t action;
    cal_plan_why_t       why;
} cal_restore_plan_t;

/* Rules, in order: source not valid -> refuse; UID known and != uid -> refuse
 * (force too); stored cal block blank/zeroed -> refuse (force too); live page
 * programmed and no force -> refuse; live page == backup -> no write;
 * otherwise write. */
cal_restore_plan_t cal_backup_restore_plan(const cal_live_info_t *live,
                                           const cal_slot_info_t *src,
                                           const uint8_t *uid, bool force);

/* Auto-restore is permitted ONLY when a valid backup exists AND the live page
 * carries nothing worth keeping. */
bool cal_backup_should_auto_restore(cal_page_class_t live, bool backup_valid);

/* Human-readable strings for logs/shell. Never NULL. */
const char *cal_rec_status_str(cal_rec_status_t s);
const char *cal_page_class_str(cal_page_class_t c);
const char *cal_slot_state_str(cal_slot_state_t s);
const char *cal_plan_why_str(cal_plan_why_t w);

#endif /* CAL_BACKUP_CORE_H */
