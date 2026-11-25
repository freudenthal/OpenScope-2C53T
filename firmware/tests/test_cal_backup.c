/*
 * Host tests for the factory-cal backup CORE (src/util/cal_backup_core.c).
 *
 * The core is deliberately hardware-free: record build/parse/validate (v1 and
 * v2), page and cal-block classification, the streamed slot scan the device
 * uses, and the decisions that must never be wrong — "which slot may
 * `cal backup` write?" and "may a restore overwrite the live page?". What the
 * device does on top (reading 0x08006000 and the UID, W25Q slot I/O, MCU-flash
 * erase/program) lives in cal_backup.c and is validated on the bench, not here.
 * Restore is tested ONLY here (project invariant I-40: no `cal restore` on the
 * unit).
 *
 * Build: gcc -o tests/test_cal_backup tests/test_cal_backup.c \
 *            src/util/cal_backup_core.c -Isrc/util
 * Run:   ./tests/test_cal_backup
 *
 * Also the target of a mutation check in scripts/test_cal_backup.py: each guard
 * in the core is deleted in turn and these tests are required to go red.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "cal_backup_core.h"

static int tests_run = 0;
static int tests_failed = 0;
static int current_failed = 0;

#define CHECK(cond, ...) do {                       \
    if (!(cond)) {                                  \
        printf("  FAIL (line %d): ", __LINE__);     \
        printf(__VA_ARGS__);                        \
        printf("\n");                               \
        current_failed++;                           \
    }                                               \
} while (0)

#define RUN(fn) do {                                \
    current_failed = 0;                             \
    printf("%s\n", #fn);                            \
    fn();                                           \
    tests_run++;                                    \
    if (current_failed) { tests_failed++; printf("  -> FAILED\n"); }   \
    else { printf("  ok\n"); }                      \
} while (0)

#define SLOT_LEN 0x2000u   /* one W25Q slot region, 8 KB */

/* This unit's UID (stage-175 bench read) and another unit's. */
static const uint8_t UID_US[CAL_UID_LEN] = {
    0x0E, 0x92, 0x74, 0x03,  0x00, 0x80, 0x00, 0x6E,  0x20, 0x94, 0xCC, 0x07 };
static const uint8_t UID_OTHER[CAL_UID_LEN] = {
    0x0E, 0x92, 0x74, 0x03,  0x00, 0x80, 0x00, 0x6E,  0x20, 0x94, 0xCC, 0x08 };

/* A plausible 4 KB page: mostly 0xFF, a run of "cal" values in the cal block
 * (0x026 onward) and a few settings bytes in the tail (0x170 onward), which an
 * IAP flash changes on the real unit (G-150). */
static void make_page(uint8_t *page)
{
    memset(page, 0xFF, CAL_BACKUP_PAYLOAD_LEN);
    for (uint32_t i = 0; i < 240u; i++) {
        page[0x026u + i] = (uint8_t)(0x40u + (i & 0x3Fu));
    }
    for (uint32_t i = 0; i < 16u; i++) {
        page[0x170u + i] = (uint8_t)(0x10u + i);
    }
}

/* Slot image: an empty 8 KB region (0xFF) with an optional record at 0. */
static void slot_empty(uint8_t *slot)
{
    memset(slot, 0xFF, SLOT_LEN);
}

static uint32_t slot_with_record(uint8_t *slot, const uint8_t *page,
                                 uint16_t version, const uint8_t *uid)
{
    slot_empty(slot);
    return cal_backup_build_record(page, (uint16_t)CAL_BACKUP_PAYLOAD_LEN,
                                   version, CAL_BACKUP_SRC_ADDR, uid, slot, SLOT_LEN);
}

static cal_slot_info_t scan(const uint8_t *slot, const uint8_t *live)
{
    cal_slot_info_t info;
    cal_slot_scan_buffer(slot, SLOT_LEN, live, 256u, &info);   /* device chunk */
    return info;
}

static cal_live_info_t live_of(const uint8_t *page)
{
    cal_live_info_t li;
    cal_backup_live_info(page, &li);
    return li;
}

static cal_slot_info_t slot_state_only(cal_slot_state_t st)
{
    cal_slot_info_t s;
    memset(&s, 0, sizeof s);
    s.state = st;
    return s;
}

/* ── CRC32 matches the known IEEE-802.3 test vector ─────────────────── */
static void test_crc32_known_vector(void)
{
    uint32_t c = cal_backup_crc32("123456789", 9);
    CHECK(c == 0xCBF43926u, "crc32(\"123456789\")=0x%08X, expected 0xCBF43926", c);
}

/* ── v1 (the record on the unit since stage-030) still reads ────────── */
static void test_v1_roundtrip(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t rec[CAL_BACKUP_RECORD_MAX];
    make_page(page);

    uint32_t n = cal_backup_build_record(page, (uint16_t)CAL_BACKUP_PAYLOAD_LEN,
                                         (uint16_t)CAL_BACKUP_VERSION_1,
                                         CAL_BACKUP_SRC_ADDR, NULL, rec, sizeof rec);
    CHECK(n == CAL_BACKUP_HEADER_LEN_V1 + CAL_BACKUP_PAYLOAD_LEN, "v1 record len %u", n);

    cal_backup_header_t h;
    const uint8_t *payload = NULL;
    cal_rec_status_t st = cal_backup_parse(rec, n, &h, &payload);
    CHECK(st == CAL_REC_OK, "parse -> %s", cal_rec_status_str(st));
    CHECK(h.version == CAL_BACKUP_VERSION_1, "version wrong");
    CHECK(h.header_len == CAL_BACKUP_HEADER_LEN_V1, "v1 header_len %u", h.header_len);
    CHECK(!h.uid_known, "v1 record claims a UID");
    CHECK(h.src_addr == CAL_BACKUP_SRC_ADDR, "src_addr wrong");
    CHECK(payload == rec + CAL_BACKUP_HEADER_LEN_V1, "v1 payload offset wrong");
    CHECK(payload != NULL && memcmp(payload, page, CAL_BACKUP_PAYLOAD_LEN) == 0,
          "payload not byte-identical");

    /* The real header bytes of slot A on the unit (stage-175 dump). */
    static const uint8_t unit_hdr[20] = {
        0x42, 0x43, 0x41, 0x4c, 0x01, 0x00, 0x00, 0x10, 0x00, 0x60,
        0x00, 0x08, 0x78, 0xaa, 0x18, 0xf7, 0x06, 0x11, 0x67, 0x32 };
    st = cal_backup_check_header(unit_hdr, sizeof unit_hdr, &h);
    CHECK(st == CAL_REC_OK, "unit slot A header -> %s", cal_rec_status_str(st));
    CHECK(h.payload_crc == 0xF718AA78u, "unit payload crc 0x%08X", h.payload_crc);
}

/* ── v2 carries the UID and the cal-block CRC ───────────────────────── */
static void test_v2_roundtrip(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t rec[CAL_BACKUP_RECORD_MAX];
    make_page(page);

    uint32_t n = cal_backup_build_record(page, (uint16_t)CAL_BACKUP_PAYLOAD_LEN,
                                         (uint16_t)CAL_BACKUP_VERSION_2,
                                         CAL_BACKUP_SRC_ADDR, UID_US, rec, sizeof rec);
    CHECK(n == CAL_BACKUP_RECORD_MAX, "v2 record len %u", n);
    CHECK(CAL_BACKUP_RECORD_MAX <= SLOT_LEN, "v2 record does not fit a slot");
    CHECK(CAL_BACKUP_VERSION == CAL_BACKUP_VERSION_2, "the written version must be 2");

    cal_backup_header_t h;
    const uint8_t *payload = NULL;
    cal_rec_status_t st = cal_backup_parse(rec, n, &h, &payload);
    CHECK(st == CAL_REC_OK, "parse -> %s", cal_rec_status_str(st));
    CHECK(h.header_len == CAL_BACKUP_HEADER_LEN_V2, "v2 header_len %u", h.header_len);
    CHECK(h.uid_known && memcmp(h.uid, UID_US, CAL_UID_LEN) == 0, "UID not kept");
    CHECK(h.block_crc == cal_backup_crc32(page + CAL_BLOCK_OFF, CAL_BLOCK_LEN),
          "block_crc is not the CRC of 0x025-0x12E");
    CHECK(payload == rec + CAL_BACKUP_HEADER_LEN_V2, "v2 payload offset wrong");
    CHECK(payload != NULL && memcmp(payload, page, CAL_BACKUP_PAYLOAD_LEN) == 0,
          "payload not byte-identical");
    CHECK(CAL_BLOCK_OFF == 0x025u && CAL_BLOCK_OFF + CAL_BLOCK_LEN - 1u == 0x12Eu,
          "cal block is not 0x025-0x12E inclusive");
}

/* ── build refuses bad arguments ────────────────────────────────────── */
static void test_build_bad_args(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t rec[CAL_BACKUP_RECORD_MAX];
    make_page(page);

    CHECK(cal_backup_build_record(NULL, 16, 1, 0, NULL, rec, sizeof rec) == 0, "null payload accepted");
    CHECK(cal_backup_build_record(page, 0, 1, 0, NULL, rec, sizeof rec) == 0, "zero len accepted");
    CHECK(cal_backup_build_record(page, (uint16_t)CAL_BACKUP_PAYLOAD_LEN, 1, 0, NULL, rec, 8) == 0,
          "undersized out accepted");
    CHECK(cal_backup_build_record(page, (uint16_t)CAL_BACKUP_PAYLOAD_LEN, 2, 0, NULL, rec, sizeof rec) == 0,
          "v2 without a UID accepted");
    CHECK(cal_backup_build_record(page, 16, 2, 0, UID_US, rec, sizeof rec) == 0,
          "v2 payload shorter than the cal block accepted");
    CHECK(cal_backup_build_record(page, 16, 9, 0, NULL, rec, sizeof rec) == 0,
          "unknown version built");
}

/* ── parse rejects every form of corruption ─────────────────────────── */
static void test_parse_rejects_corruption(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t rec[CAL_BACKUP_RECORD_MAX];
    static uint8_t r2[CAL_BACKUP_RECORD_MAX];
    make_page(page);

    for (uint16_t v = 1; v <= 2; v++) {
        uint32_t n = cal_backup_build_record(page, (uint16_t)CAL_BACKUP_PAYLOAD_LEN, v,
                                             CAL_BACKUP_SRC_ADDR, UID_US, rec, sizeof rec);
        uint32_t hl = cal_backup_header_len(v);

        CHECK(cal_backup_parse(rec, CAL_BACKUP_HEADER_LEN_V1 - 1u, NULL, NULL) == CAL_REC_ERR_TRUNCATED,
              "v%u: short buffer not rejected", v);

        memcpy(r2, rec, n);
        r2[0] ^= 0xFFu;
        CHECK(cal_backup_parse(r2, n, NULL, NULL) == CAL_REC_ERR_MAGIC, "v%u: magic corruption passed", v);

        memcpy(r2, rec, n);
        r2[8] ^= 0x01u;   /* src_addr low byte */
        CHECK(cal_backup_parse(r2, n, NULL, NULL) == CAL_REC_ERR_HEADER_CRC,
              "v%u: header corruption passed", v);

        memcpy(r2, rec, n);
        r2[hl + 100u] ^= 0x80u;
        CHECK(cal_backup_parse(r2, n, NULL, NULL) == CAL_REC_ERR_PAYLOAD_CRC,
              "v%u: payload corruption passed", v);

        memcpy(r2, rec, n);
        r2[hl + CAL_BLOCK_OFF + 3u] ^= 0x01u;   /* inside the cal block */
        CHECK(cal_backup_parse(r2, n, NULL, NULL) == CAL_REC_ERR_PAYLOAD_CRC,
              "v%u: cal block corruption passed", v);
    }

    /* blank region (all 0xFF) — the common "no backup yet" case */
    static uint8_t blank[CAL_BACKUP_RECORD_MAX];
    memset(blank, 0xFF, sizeof blank);
    CHECK(cal_backup_parse(blank, sizeof blank, NULL, NULL) == CAL_REC_ERR_MAGIC,
          "blank region not reported as bad magic");

    /* v2: the UID and block_crc are inside header_crc. */
    uint32_t n = cal_backup_build_record(page, (uint16_t)CAL_BACKUP_PAYLOAD_LEN, 2,
                                         CAL_BACKUP_SRC_ADDR, UID_US, rec, sizeof rec);
    memcpy(r2, rec, n);
    r2[20 + 11] ^= 0x01u;
    CHECK(cal_backup_parse(r2, n, NULL, NULL) == CAL_REC_ERR_HEADER_CRC, "v2 UID corruption passed");
    memcpy(r2, rec, n);
    r2[32] ^= 0x01u;
    CHECK(cal_backup_parse(r2, n, NULL, NULL) == CAL_REC_ERR_HEADER_CRC, "v2 block_crc corruption passed");

    /* v2 header self-consistent but block_crc does not describe the payload. */
    memcpy(r2, rec, n);
    cal_backup_write_header(r2, 2, (uint16_t)CAL_BACKUP_PAYLOAD_LEN, CAL_BACKUP_SRC_ADDR,
                            cal_backup_crc32(page, CAL_BACKUP_PAYLOAD_LEN), UID_US, 0x12345678u);
    CHECK(cal_backup_parse(r2, n, NULL, NULL) == CAL_REC_ERR_BLOCK_CRC, "wrong block_crc passed");

    /* v1 record with its version byte changed to 2: not a valid v2 header. */
    uint32_t n1 = cal_backup_build_record(page, (uint16_t)CAL_BACKUP_PAYLOAD_LEN, 1,
                                          CAL_BACKUP_SRC_ADDR, NULL, rec, sizeof rec);
    memcpy(r2, rec, n1);
    r2[4] = 2u;
    CHECK(cal_backup_parse(r2, n1, NULL, NULL) == CAL_REC_ERR_HEADER_CRC, "v1 read as v2");
    /* A v2 header cut at 20 bytes is truncated, not read as v1. */
    CHECK(cal_backup_check_header(rec, CAL_BACKUP_HEADER_LEN_V1, NULL) == CAL_REC_OK, "v1 header");
    n = cal_backup_build_record(page, (uint16_t)CAL_BACKUP_PAYLOAD_LEN, 2,
                                CAL_BACKUP_SRC_ADDR, UID_US, rec, sizeof rec);
    CHECK(cal_backup_check_header(rec, CAL_BACKUP_HEADER_LEN_V1, NULL) == CAL_REC_ERR_TRUNCATED,
          "cut v2 header not truncated");
}

/* ── an unknown future version is rejected, not misread ─────────────── */
static void test_parse_rejects_future_version(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t rec[CAL_BACKUP_RECORD_MAX];
    make_page(page);
    uint32_t n = cal_backup_build_record(page, (uint16_t)CAL_BACKUP_PAYLOAD_LEN, 1,
                                         CAL_BACKUP_SRC_ADDR, NULL, rec, sizeof rec);
    /* Version 9 with a header CRC that is right for the v1 layout. */
    rec[4] = 9u;
    uint32_t c = cal_backup_crc32(rec, 16u);
    rec[16] = (uint8_t)c; rec[17] = (uint8_t)(c >> 8);
    rec[18] = (uint8_t)(c >> 16); rec[19] = (uint8_t)(c >> 24);
    CHECK(cal_backup_parse(rec, n, NULL, NULL) == CAL_REC_ERR_VERSION, "future version accepted");
    CHECK(cal_backup_header_len(9) == 0u, "header_len of an unknown version");
}

/* ── page and cal-block classification ──────────────────────────────── */
static void test_classify_page(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];

    memset(page, 0xFF, sizeof page);
    CHECK(cal_backup_classify_page(page, sizeof page) == CAL_PAGE_BLANK, "all-FF not BLANK");
    memset(page, 0x00, sizeof page);
    CHECK(cal_backup_classify_page(page, sizeof page) == CAL_PAGE_ZEROED, "all-00 not ZEROED");
    make_page(page);
    CHECK(cal_backup_classify_page(page, sizeof page) == CAL_PAGE_PROGRAMMED, "real page not PROGRAMMED");
    memset(page, 0xFF, sizeof page);
    page[2000] = 0x00;
    CHECK(cal_backup_classify_page(page, sizeof page) == CAL_PAGE_PROGRAMMED, "one byte not detected");

    CHECK(cal_backup_class_merge(CAL_PAGE_BLANK, CAL_PAGE_BLANK) == CAL_PAGE_BLANK, "merge FF+FF");
    CHECK(cal_backup_class_merge(CAL_PAGE_ZEROED, CAL_PAGE_ZEROED) == CAL_PAGE_ZEROED, "merge 00+00");
    CHECK(cal_backup_class_merge(CAL_PAGE_BLANK, CAL_PAGE_ZEROED) == CAL_PAGE_PROGRAMMED, "merge FF+00");
}

static void test_live_info(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    make_page(page);
    cal_live_info_t li = live_of(page);
    CHECK(li.page_class == CAL_PAGE_PROGRAMMED, "page class");
    CHECK(li.block_class == CAL_PAGE_PROGRAMMED, "block class");
    CHECK(li.payload_crc == cal_backup_crc32(page, CAL_BACKUP_PAYLOAD_LEN), "page crc");
    CHECK(li.block_crc == cal_backup_crc32(page + 0x025, 0x10A), "block crc");

    /* Settings in the tail but a blank cal block: a partial page (G-150). */
    memset(page + CAL_BLOCK_OFF, 0xFF, CAL_BLOCK_LEN);
    li = live_of(page);
    CHECK(li.page_class == CAL_PAGE_PROGRAMMED, "partial page not programmed");
    CHECK(li.block_class == CAL_PAGE_BLANK, "blank cal block not BLANK");
    memset(page + CAL_BLOCK_OFF, 0x00, CAL_BLOCK_LEN);
    li = live_of(page);
    CHECK(li.block_class == CAL_PAGE_ZEROED, "zeroed cal block not ZEROED");
    /* One byte at each end of the block counts. */
    memset(page + CAL_BLOCK_OFF, 0xFF, CAL_BLOCK_LEN);
    page[0x12E] = 0x00;
    CHECK(live_of(page).block_class == CAL_PAGE_PROGRAMMED, "last block byte not seen");
    page[0x12E] = 0xFF; page[0x025] = 0x01;
    CHECK(live_of(page).block_class == CAL_PAGE_PROGRAMMED, "first block byte not seen");
    page[0x025] = 0xFF; page[0x024] = 0x01; page[0x12F] = 0x01;
    CHECK(live_of(page).block_class == CAL_PAGE_BLANK, "bytes outside the block counted");
}

/* ── the streamed slot scan (the code the device runs) ──────────────── */
static void test_slot_scan(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t live[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t slot[SLOT_LEN];
    make_page(page);
    memcpy(live, page, sizeof live);

    slot_empty(slot);
    cal_slot_info_t s = scan(slot, live);
    CHECK(s.state == CAL_SLOT_EMPTY, "blank slot -> %s", cal_slot_state_str(s.state));

    /* A torn write: payload present, header still blank, is EMPTY (the header
     * is written last); one programmed header byte is BAD. */
    slot_with_record(slot, page, 2, UID_US);
    memset(slot, 0xFF, CAL_BACKUP_HEADER_LEN_V2);
    CHECK(scan(slot, live).state == CAL_SLOT_EMPTY, "headerless slot not EMPTY");
    slot[5] = 0xFE;
    s = scan(slot, live);
    CHECK(s.state == CAL_SLOT_BAD, "torn header -> %s", cal_slot_state_str(s.state));

    for (uint16_t v = 1; v <= 2; v++) {
        slot_with_record(slot, page, v, UID_US);
        s = scan(slot, live);
        CHECK(s.state == CAL_SLOT_VALID, "v%u -> %s (%s)", v, cal_slot_state_str(s.state),
              cal_rec_status_str(s.rec_status));
        CHECK(s.hdr.uid_known == (v == 2), "v%u uid_known", v);
        CHECK(s.block_equal_live && s.page_equal_live, "v%u equal flags", v);
        CHECK(s.block_crc == cal_backup_crc32(page + CAL_BLOCK_OFF, CAL_BLOCK_LEN), "v%u block crc", v);
        CHECK(s.block_class == CAL_PAGE_PROGRAMMED, "v%u block class", v);

        /* Same answer for any chunk size. */
        static const uint32_t chunks[] = { 1u, 7u, 64u, 255u, 4096u };
        for (unsigned k = 0; k < sizeof chunks / sizeof chunks[0]; k++) {
            cal_slot_info_t t;
            cal_slot_scan_buffer(slot, SLOT_LEN, live, chunks[k], &t);
            CHECK(t.state == s.state && t.rec_status == s.rec_status &&
                  t.block_crc == s.block_crc && t.block_class == s.block_class &&
                  t.block_equal_live == s.block_equal_live &&
                  t.page_equal_live == s.page_equal_live &&
                  t.hdr.payload_crc == s.hdr.payload_crc,
                  "v%u chunk %u differs", v, chunks[k]);
        }

        /* No live page given: never "equal". */
        cal_slot_info_t t;
        cal_slot_scan_buffer(slot, SLOT_LEN, NULL, 256u, &t);
        CHECK(t.state == CAL_SLOT_VALID && !t.block_equal_live && !t.page_equal_live,
              "v%u equal without a live page", v);

        /* Payload corruption -> BAD. */
        slot[cal_backup_header_len(v) + 3000u] ^= 0x04u;
        s = scan(slot, live);
        CHECK(s.state == CAL_SLOT_BAD && s.rec_status == CAL_REC_ERR_PAYLOAD_CRC,
              "v%u corrupt payload -> %s", v, cal_rec_status_str(s.rec_status));
    }

    /* Live differs outside the cal block only (an IAP flash, G-150). */
    slot_with_record(slot, page, 1, NULL);
    live[0x1A0] ^= 0x55u;
    s = scan(slot, live);
    CHECK(s.block_equal_live && !s.page_equal_live, "tail change: block %d page %d",
          s.block_equal_live, s.page_equal_live);
    /* ... and inside it, at each end. */
    memcpy(live, page, sizeof live);
    live[0x025] ^= 0x01u;
    CHECK(!scan(slot, live).block_equal_live, "first cal-block byte change not seen");
    memcpy(live, page, sizeof live);
    live[0x12E] ^= 0x01u;
    CHECK(!scan(slot, live).block_equal_live, "last cal-block byte change not seen");
    memcpy(live, page, sizeof live);

    /* v2 with a consistent header but a wrong block_crc -> BAD. */
    slot_with_record(slot, page, 2, UID_US);
    cal_backup_write_header(slot, 2, (uint16_t)CAL_BACKUP_PAYLOAD_LEN, CAL_BACKUP_SRC_ADDR,
                            cal_backup_crc32(page, CAL_BACKUP_PAYLOAD_LEN), UID_US, 0xDEADBEEFu);
    s = scan(slot, live);
    CHECK(s.state == CAL_SLOT_BAD && s.rec_status == CAL_REC_ERR_BLOCK_CRC,
          "wrong block_crc -> %s", cal_rec_status_str(s.rec_status));

    /* payload_len other than 4 KB -> BAD. */
    slot_empty(slot);
    cal_backup_build_record(page, 0x800u, 2, CAL_BACKUP_SRC_ADDR, UID_US, slot, SLOT_LEN);
    s = scan(slot, live);
    CHECK(s.state == CAL_SLOT_BAD && s.rec_status == CAL_REC_ERR_LEN,
          "2 KB record -> %s", cal_rec_status_str(s.rec_status));

    /* Payload fed short -> BAD. */
    slot_with_record(slot, page, 2, UID_US);
    cal_slot_scan_t sc;
    CHECK(cal_slot_scan_begin(&sc, slot, CAL_BACKUP_HEADER_MAX, live), "begin");
    cal_slot_scan_feed(&sc, slot + CAL_BACKUP_HEADER_LEN_V2, 1000u);
    cal_slot_scan_end(&sc, &s);
    CHECK(s.state == CAL_SLOT_BAD && s.rec_status == CAL_REC_ERR_LEN, "short feed -> %s",
          cal_rec_status_str(s.rec_status));
}

/* ── store decision ─────────────────────────────────────────────────── */
static void test_plan_refuses_empty_capture(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    cal_slot_info_t a = slot_state_only(CAL_SLOT_EMPTY);
    cal_slot_info_t b = slot_state_only(CAL_SLOT_EMPTY);
    cal_live_info_t li;
    cal_plan_t p;

    memset(page, 0xFF, sizeof page);
    li = live_of(page);
    p = cal_backup_plan(&li, &a, &b, UID_US, true);
    CHECK(p.action == CAL_PLAN_REFUSE && p.why == CAL_WHY_LIVE_BLANK, "blank page: %s",
          cal_plan_why_str(p.why));
    memset(page, 0x00, sizeof page);
    li = live_of(page);
    p = cal_backup_plan(&li, &a, &b, UID_US, true);
    CHECK(p.action == CAL_PLAN_REFUSE && p.why == CAL_WHY_LIVE_BLANK, "zeroed page: %s",
          cal_plan_why_str(p.why));

    /* Programmed page, blank or zeroed cal block (a partial page). */
    make_page(page);
    memset(page + CAL_BLOCK_OFF, 0xFF, CAL_BLOCK_LEN);
    li = live_of(page);
    p = cal_backup_plan(&li, &a, &b, UID_US, true);
    CHECK(p.action == CAL_PLAN_REFUSE && p.why == CAL_WHY_LIVE_BLOCK_BLANK, "FF block: %s",
          cal_plan_why_str(p.why));
    memset(page + CAL_BLOCK_OFF, 0x00, CAL_BLOCK_LEN);
    li = live_of(page);
    p = cal_backup_plan(&li, &a, &b, UID_US, true);
    CHECK(p.action == CAL_PLAN_REFUSE && p.why == CAL_WHY_LIVE_BLOCK_BLANK, "00 block: %s",
          cal_plan_why_str(p.why));

    p = cal_backup_plan(NULL, &a, &b, UID_US, false);
    CHECK(p.action == CAL_PLAN_REFUSE, "null live accepted");
    make_page(page);
    li = live_of(page);
    p = cal_backup_plan(&li, &a, &b, NULL, false);
    CHECK(p.action == CAL_PLAN_REFUSE, "null uid accepted");
}

static void test_plan_slot_a(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t slot[SLOT_LEN];
    make_page(page);
    cal_live_info_t li = live_of(page);
    cal_slot_info_t a, b = slot_state_only(CAL_SLOT_EMPTY);
    cal_plan_t p;

    /* Empty A: the first backup goes to A, force or not, whatever B holds. */
    a = slot_state_only(CAL_SLOT_EMPTY);
    for (int f = 0; f <= 1; f++) {
        p = cal_backup_plan(&li, &a, &b, UID_US, f);
        CHECK(p.action == CAL_PLAN_WRITE_A, "empty A, force %d: action %d", f, p.action);
    }
    slot_with_record(slot, page, 2, UID_US);
    b = scan(slot, page);
    p = cal_backup_plan(&li, &a, &b, UID_US, false);
    CHECK(p.action == CAL_PLAN_WRITE_A, "empty A with valid B: action %d", p.action);
    b = slot_state_only(CAL_SLOT_EMPTY);

    /* Damaged A is never erased, force or not. */
    slot_with_record(slot, page, 1, NULL);
    slot[CAL_BACKUP_HEADER_LEN_V1 + 50u] ^= 0x01u;
    a = scan(slot, page);
    CHECK(a.state == CAL_SLOT_BAD, "fixture: A must be BAD");
    for (int f = 0; f <= 1; f++) {
        p = cal_backup_plan(&li, &a, &b, UID_US, f);
        CHECK(p.action == CAL_PLAN_REFUSE && p.why == CAL_WHY_A_DAMAGED,
              "damaged A, force %d: %s", f, cal_plan_why_str(p.why));
    }

    /* A from another unit: refuse, force too. */
    slot_with_record(slot, page, 2, UID_OTHER);
    a = scan(slot, page);
    for (int f = 0; f <= 1; f++) {
        p = cal_backup_plan(&li, &a, &b, UID_US, f);
        CHECK(p.action == CAL_PLAN_REFUSE && p.why == CAL_WHY_UID_MISMATCH,
              "other unit's A, force %d: %s", f, cal_plan_why_str(p.why));
    }
}

static void test_plan_slot_b(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t live[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t slot_a[SLOT_LEN];
    static uint8_t slot_b[SLOT_LEN];
    make_page(page);
    memcpy(live, page, sizeof live);
    live[0x1A0] ^= 0x55u;          /* tail changed by a flash; cal block equal */
    cal_live_info_t li = live_of(live);
    cal_slot_info_t a, b;
    cal_plan_t p;

    /* A v1 (UID unknown) with an equal cal block: write B. Our unit's case. */
    slot_with_record(slot_a, page, 1, NULL);
    a = scan(slot_a, live);
    slot_empty(slot_b);
    b = scan(slot_b, live);
    p = cal_backup_plan(&li, &a, &b, UID_US, false);
    CHECK(p.action == CAL_PLAN_WRITE_B, "v1 A, equal block: action %d (%s)", p.action,
          cal_plan_why_str(p.why));

    /* A v2 with our UID: same. */
    slot_with_record(slot_a, page, 2, UID_US);
    a = scan(slot_a, live);
    p = cal_backup_plan(&li, &a, &b, UID_US, false);
    CHECK(p.action == CAL_PLAN_WRITE_B, "v2 A, equal block: action %d", p.action);

    /* B already holds this page: no write. */
    slot_with_record(slot_b, live, 2, UID_US);
    b = scan(slot_b, live);
    p = cal_backup_plan(&li, &a, &b, UID_US, false);
    CHECK(p.action == CAL_PLAN_NONE_EQUAL, "B equal: action %d", p.action);
    p = cal_backup_plan(&li, &a, &b, UID_US, true);
    CHECK(p.action == CAL_PLAN_NONE_EQUAL, "B equal, force: action %d", p.action);

    /* B holds this page under another unit's UID: rewrite B. */
    slot_with_record(slot_b, live, 2, UID_OTHER);
    b = scan(slot_b, live);
    p = cal_backup_plan(&li, &a, &b, UID_US, false);
    CHECK(p.action == CAL_PLAN_WRITE_B, "B equal, other UID: action %d", p.action);

    /* B holds an older page: write B. B damaged: write B. */
    slot_with_record(slot_b, page, 2, UID_US);
    b = scan(slot_b, live);
    p = cal_backup_plan(&li, &a, &b, UID_US, false);
    CHECK(p.action == CAL_PLAN_WRITE_B, "B older: action %d", p.action);
    slot_b[100] ^= 0x01u;
    b = scan(slot_b, live);
    CHECK(b.state == CAL_SLOT_BAD, "fixture: B must be BAD");
    p = cal_backup_plan(&li, &a, &b, UID_US, false);
    CHECK(p.action == CAL_PLAN_WRITE_B, "B damaged: action %d", p.action);

    /* Cal block differs from A: refuse; force writes B, never A. */
    live[0x080] ^= 0x01u;
    li = live_of(live);
    a = scan(slot_a, live);
    slot_empty(slot_b);
    b = scan(slot_b, live);
    p = cal_backup_plan(&li, &a, &b, UID_US, false);
    CHECK(p.action == CAL_PLAN_REFUSE && p.why == CAL_WHY_BLOCK_DIFFERS,
          "block differs: %s", cal_plan_why_str(p.why));
    p = cal_backup_plan(&li, &a, &b, UID_US, true);
    CHECK(p.action == CAL_PLAN_WRITE_B, "block differs, force: action %d", p.action);
    slot_with_record(slot_b, live, 2, UID_US);
    b = scan(slot_b, live);
    p = cal_backup_plan(&li, &a, &b, UID_US, true);
    CHECK(p.action == CAL_PLAN_NONE_EQUAL, "block differs, force, B equal: action %d", p.action);
}

/* Slot A is written only when it is EMPTY: every combination. */
static void test_plan_never_writes_valid_a(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t live[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t slot[SLOT_LEN];
    make_page(page);
    const cal_slot_state_t states[] = { CAL_SLOT_EMPTY, CAL_SLOT_VALID, CAL_SLOT_BAD };
    int writes_a = 0, bad_writes_a = 0;

    for (int diff = 0; diff <= 1; diff++) {
        memcpy(live, page, sizeof live);
        if (diff) { live[0x100] ^= 0x01u; }
        cal_live_info_t li = live_of(live);
        for (unsigned ia = 0; ia < 3; ia++) {
            for (unsigned ib = 0; ib < 3; ib++) {
                for (int f = 0; f <= 1; f++) {
                    for (int u = 0; u <= 2; u++) {   /* A: v1, v2 ours, v2 other */
                        cal_slot_info_t a = slot_state_only(states[ia]);
                        cal_slot_info_t b = slot_state_only(states[ib]);
                        if (states[ia] == CAL_SLOT_VALID) {
                            slot_with_record(slot, page, u == 0 ? 1 : 2,
                                             u == 2 ? UID_OTHER : UID_US);
                            a = scan(slot, live);
                        }
                        cal_plan_t p = cal_backup_plan(&li, &a, &b, UID_US, f);
                        if (p.action == CAL_PLAN_WRITE_A) {
                            writes_a++;
                            if (a.state != CAL_SLOT_EMPTY) { bad_writes_a++; }
                        }
                    }
                }
            }
        }
    }
    CHECK(bad_writes_a == 0, "slot A written while not empty: %d times", bad_writes_a);
    CHECK(writes_a > 0, "slot A never written (fixture)");
}

/* The bench sequence of stage-175 step 6, on the host. */
static void test_unit_sequence(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t live[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t slot_a[SLOT_LEN], slot_a0[SLOT_LEN], slot_b[SLOT_LEN];
    make_page(page);
    memcpy(live, page, sizeof live);
    for (uint32_t i = 0x001; i <= 0x016; i++) { live[i] ^= 0x5Au; }   /* stage-050 bytes */
    for (uint32_t i = 0x170; i <= 0x1FF; i++) { live[i] ^= 0x33u; }
    cal_live_info_t li = live_of(live);

    slot_with_record(slot_a, page, 1, NULL);
    memcpy(slot_a0, slot_a, SLOT_LEN);
    slot_empty(slot_b);

    /* a. A v1 valid, cal block equal, page differs; B empty. */
    cal_slot_info_t a = scan(slot_a, live), b = scan(slot_b, live);
    CHECK(a.state == CAL_SLOT_VALID && a.block_equal_live && !a.page_equal_live, "step a: A");
    CHECK(b.state == CAL_SLOT_EMPTY, "step a: B");

    /* b. cal backup -> B. */
    cal_plan_t p = cal_backup_plan(&li, &a, &b, UID_US, false);
    CHECK(p.action == CAL_PLAN_WRITE_B, "step b: action %d", p.action);
    slot_with_record(slot_b, live, CAL_BACKUP_VERSION, UID_US);
    b = scan(slot_b, live);
    CHECK(b.state == CAL_SLOT_VALID && b.hdr.version == 2 && b.page_equal_live &&
          memcmp(b.hdr.uid, UID_US, CAL_UID_LEN) == 0 &&
          b.hdr.payload_crc == li.payload_crc, "step b: B after write");

    /* c. cal backup again -> no write. */
    p = cal_backup_plan(&li, &a, &b, UID_US, false);
    CHECK(p.action == CAL_PLAN_NONE_EQUAL, "step c: action %d", p.action);

    /* d. slot A untouched. */
    CHECK(memcmp(slot_a, slot_a0, SLOT_LEN) == 0, "step d: slot A changed");
}

/* ── restore decision (host only, I-40) ─────────────────────────────── */
static void test_restore_plan(void)
{
    static uint8_t page[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t live[CAL_BACKUP_PAYLOAD_LEN];
    static uint8_t slot[SLOT_LEN];
    make_page(page);
    cal_restore_plan_t r;
    cal_slot_info_t s;
    cal_live_info_t li;

    /* No valid source. */
    memset(live, 0xFF, sizeof live);
    li = live_of(live);
    s = slot_state_only(CAL_SLOT_EMPTY);
    r = cal_backup_restore_plan(&li, &s, UID_US, true);
    CHECK(r.action == CAL_RESTORE_REFUSE && r.why == CAL_WHY_NO_BACKUP, "empty src: %s",
          cal_plan_why_str(r.why));
    s = slot_state_only(CAL_SLOT_BAD);
    r = cal_backup_restore_plan(&li, &s, UID_US, true);
    CHECK(r.action == CAL_RESTORE_REFUSE && r.why == CAL_WHY_NO_BACKUP, "bad src: %s",
          cal_plan_why_str(r.why));

    /* Blank live page, valid source: write (v1 and v2 ours). */
    slot_with_record(slot, page, 1, NULL);
    s = scan(slot, live);
    r = cal_backup_restore_plan(&li, &s, UID_US, false);
    CHECK(r.action == CAL_RESTORE_WRITE, "v1 over blank: action %d (%s)", r.action,
          cal_plan_why_str(r.why));
    slot_with_record(slot, page, 2, UID_US);
    s = scan(slot, live);
    r = cal_backup_restore_plan(&li, &s, UID_US, false);
    CHECK(r.action == CAL_RESTORE_WRITE, "v2 over blank: action %d", r.action);

    /* Another unit's backup: refuse, force too. */
    slot_with_record(slot, page, 2, UID_OTHER);
    s = scan(slot, live);
    for (int f = 0; f <= 1; f++) {
        r = cal_backup_restore_plan(&li, &s, UID_US, f);
        CHECK(r.action == CAL_RESTORE_REFUSE && r.why == CAL_WHY_UID_MISMATCH,
              "other unit, force %d: %s", f, cal_plan_why_str(r.why));
    }

    /* A backup whose cal block is blank or zeroed: refuse, force too. */
    static uint8_t bad_page[CAL_BACKUP_PAYLOAD_LEN];
    for (int z = 0; z <= 1; z++) {
        make_page(bad_page);
        memset(bad_page + CAL_BLOCK_OFF, z ? 0x00 : 0xFF, CAL_BLOCK_LEN);
        slot_with_record(slot, bad_page, 2, UID_US);
        s = scan(slot, live);
        CHECK(s.state == CAL_SLOT_VALID, "fixture: record valid");
        for (int f = 0; f <= 1; f++) {
            r = cal_backup_restore_plan(&li, &s, UID_US, f);
            CHECK(r.action == CAL_RESTORE_REFUSE && r.why == CAL_WHY_BACKUP_BLOCK_BLANK,
                  "%s block, force %d: %s", z ? "00" : "FF", f, cal_plan_why_str(r.why));
        }
    }

    /* Programmed live page: refuse without force (also a partial page). */
    memcpy(live, page, sizeof live);
    live[0x080] ^= 0x01u;
    li = live_of(live);
    slot_with_record(slot, page, 2, UID_US);
    s = scan(slot, live);
    r = cal_backup_restore_plan(&li, &s, UID_US, false);
    CHECK(r.action == CAL_RESTORE_REFUSE && r.why == CAL_WHY_LIVE_PRECIOUS,
          "programmed live: %s", cal_plan_why_str(r.why));
    r = cal_backup_restore_plan(&li, &s, UID_US, true);
    CHECK(r.action == CAL_RESTORE_WRITE, "programmed live, force: action %d", r.action);
    memset(live, 0xFF, sizeof live);
    live[0x1F0] = 0x12u;                 /* a partial page after a torn restore */
    li = live_of(live);
    s = scan(slot, live);
    r = cal_backup_restore_plan(&li, &s, UID_US, false);
    CHECK(r.action == CAL_RESTORE_REFUSE && r.why == CAL_WHY_LIVE_PRECIOUS,
          "partial live: %s", cal_plan_why_str(r.why));

    /* Live already equal: no write. */
    memcpy(live, page, sizeof live);
    li = live_of(live);
    s = scan(slot, live);
    r = cal_backup_restore_plan(&li, &s, UID_US, true);
    CHECK(r.action == CAL_RESTORE_NONE_EQUAL, "equal: action %d", r.action);

    r = cal_backup_restore_plan(&li, NULL, UID_US, true);
    CHECK(r.action == CAL_RESTORE_REFUSE, "null src accepted");
    r = cal_backup_restore_plan(&li, &s, NULL, true);
    CHECK(r.action == CAL_RESTORE_REFUSE, "null uid accepted");
}

/* ── auto-restore: never clobber a programmed page ──────────────────── */
static void test_auto_restore_decision(void)
{
    CHECK(cal_backup_should_auto_restore(CAL_PAGE_BLANK, false) == false, "auto-restore with no backup");
    CHECK(cal_backup_should_auto_restore(CAL_PAGE_ZEROED, false) == false, "auto-restore with no backup");
    CHECK(cal_backup_should_auto_restore(CAL_PAGE_PROGRAMMED, false) == false, "auto-restore with no backup");
    CHECK(cal_backup_should_auto_restore(CAL_PAGE_BLANK, true) == true, "should restore over blank");
    CHECK(cal_backup_should_auto_restore(CAL_PAGE_ZEROED, true) == true, "should restore over zeroed");
    CHECK(cal_backup_should_auto_restore(CAL_PAGE_PROGRAMMED, true) == false,
          "auto-restore clobbered a programmed page");
}

int main(void)
{
    RUN(test_crc32_known_vector);
    RUN(test_v1_roundtrip);
    RUN(test_v2_roundtrip);
    RUN(test_build_bad_args);
    RUN(test_parse_rejects_corruption);
    RUN(test_parse_rejects_future_version);
    RUN(test_classify_page);
    RUN(test_live_info);
    RUN(test_slot_scan);
    RUN(test_plan_refuses_empty_capture);
    RUN(test_plan_slot_a);
    RUN(test_plan_slot_b);
    RUN(test_plan_never_writes_valid_a);
    RUN(test_unit_sequence);
    RUN(test_restore_plan);
    RUN(test_auto_restore_decision);

    printf("\n%d/%d test groups passed\n", tests_run - tests_failed, tests_run);
    return tests_failed ? 1 : 0;
}
