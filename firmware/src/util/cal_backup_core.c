/*
 * cal_backup_core.c — see cal_backup_core.h. Hardware-free; host-tested.
 */

#include "cal_backup_core.h"

#include <string.h>

/* ── CRC32 (IEEE-802.3, reflected). Bitwise: no 1 KB table, imperceptible on
 * 4 KB at 240 MHz, and identical to flash_regions.c / cal_dump.c. ── */
uint32_t cal_backup_crc32_update(uint32_t crc, const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc;
}

uint32_t cal_backup_crc32_final(uint32_t crc)
{
    return ~crc;
}

uint32_t cal_backup_crc32(const void *data, uint32_t len)
{
    return cal_backup_crc32_final(
        cal_backup_crc32_update(CAL_BACKUP_CRC32_INIT, data, len));
}

/* Little-endian pack/unpack helpers keep the on-flash layout host-independent. */
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint16_t cal_backup_header_len(uint16_t version)
{
    switch (version) {
    case CAL_BACKUP_VERSION_1: return (uint16_t)CAL_BACKUP_HEADER_LEN_V1;
    case CAL_BACKUP_VERSION_2: return (uint16_t)CAL_BACKUP_HEADER_LEN_V2;
    }
    return 0u;
}

/* header_crc: v1 covers [0,16); v2 covers [0,16) then [20,36) (UID, block CRC). */
static uint32_t header_crc_of(const uint8_t *h, uint16_t version)
{
    uint32_t c = cal_backup_crc32_update(CAL_BACKUP_CRC32_INIT, h, 16u);
    if (version == CAL_BACKUP_VERSION_2) {
        c = cal_backup_crc32_update(c, h + 20, CAL_BACKUP_HEADER_LEN_V2 - 20u);
    }
    return cal_backup_crc32_final(c);
}

uint32_t cal_backup_write_header(uint8_t *out, uint16_t version,
                                 uint16_t payload_len, uint32_t src_addr,
                                 uint32_t payload_crc,
                                 const uint8_t *uid, uint32_t block_crc)
{
    uint16_t hl = cal_backup_header_len(version);
    if (out == NULL || hl == 0u) {
        return 0u;
    }
    if (version == CAL_BACKUP_VERSION_2 && uid == NULL) {
        return 0u;
    }
    put_u32(out + 0, CAL_BACKUP_MAGIC);
    put_u16(out + 4, version);
    put_u16(out + 6, payload_len);
    put_u32(out + 8, src_addr);
    put_u32(out + 12, payload_crc);
    if (version == CAL_BACKUP_VERSION_2) {
        memcpy(out + 20, uid, CAL_UID_LEN);
        put_u32(out + 32, block_crc);
    }
    put_u32(out + 16, header_crc_of(out, version));
    return hl;
}

cal_rec_status_t cal_backup_check_header(const uint8_t *rec, uint32_t rec_len,
                                         cal_backup_header_t *hdr_out)
{
    if (rec == NULL || rec_len < CAL_BACKUP_HEADER_LEN_V1) {
        return CAL_REC_ERR_TRUNCATED;
    }

    cal_backup_header_t h;
    memset(&h, 0, sizeof h);
    h.magic       = get_u32(rec + 0);
    h.version     = get_u16(rec + 4);
    h.payload_len = get_u16(rec + 6);
    h.src_addr    = get_u32(rec + 8);
    h.payload_crc = get_u32(rec + 12);
    h.header_crc  = get_u32(rec + 16);
    h.header_len  = cal_backup_header_len(h.version);

    if (h.magic != CAL_BACKUP_MAGIC) {
        return CAL_REC_ERR_MAGIC;
    }
    if (h.header_len == 0u) {
        return CAL_REC_ERR_VERSION;
    }
    if (rec_len < h.header_len) {
        return CAL_REC_ERR_TRUNCATED;
    }
    if (header_crc_of(rec, h.version) != h.header_crc) {
        return CAL_REC_ERR_HEADER_CRC;
    }
    if (h.payload_len == 0u) {
        return CAL_REC_ERR_LEN;
    }
    if (h.version == CAL_BACKUP_VERSION_2) {
        h.uid_known = true;
        memcpy(h.uid, rec + 20, CAL_UID_LEN);
        h.block_crc = get_u32(rec + 32);
    }

    if (hdr_out != NULL) {
        *hdr_out = h;
    }
    return CAL_REC_OK;
}

uint32_t cal_backup_build_record(const void *payload, uint16_t payload_len,
                                 uint16_t version, uint32_t src_addr,
                                 const uint8_t *uid,
                                 uint8_t *out, uint32_t out_cap)
{
    uint16_t hl = cal_backup_header_len(version);
    if (payload == NULL || out == NULL || payload_len == 0u || hl == 0u) {
        return 0u;
    }
    if (version == CAL_BACKUP_VERSION_2 &&
        (uid == NULL || payload_len < CAL_BLOCK_OFF + CAL_BLOCK_LEN)) {
        return 0u;
    }
    uint32_t total = (uint32_t)hl + payload_len;
    if (out_cap < total) {
        return 0u;
    }
    const uint8_t *p = (const uint8_t *)payload;
    uint32_t block_crc = 0u;
    if (version == CAL_BACKUP_VERSION_2) {
        block_crc = cal_backup_crc32(p + CAL_BLOCK_OFF, CAL_BLOCK_LEN);
    }
    cal_backup_write_header(out, version, payload_len, src_addr,
                            cal_backup_crc32(p, payload_len), uid, block_crc);
    memcpy(out + hl, p, payload_len);
    return total;
}

cal_rec_status_t cal_backup_parse(const uint8_t *rec, uint32_t rec_len,
                                  cal_backup_header_t *hdr_out,
                                  const uint8_t **payload_out)
{
    cal_backup_header_t h;
    cal_rec_status_t st = cal_backup_check_header(rec, rec_len, &h);
    if (st != CAL_REC_OK) {
        return st;
    }
    if ((uint32_t)h.header_len + h.payload_len > rec_len) {
        return CAL_REC_ERR_LEN;
    }
    const uint8_t *payload = rec + h.header_len;
    if (cal_backup_crc32(payload, h.payload_len) != h.payload_crc) {
        return CAL_REC_ERR_PAYLOAD_CRC;
    }
    if (h.version == CAL_BACKUP_VERSION_2) {
        if (h.payload_len < CAL_BLOCK_OFF + CAL_BLOCK_LEN ||
            cal_backup_crc32(payload + CAL_BLOCK_OFF, CAL_BLOCK_LEN) != h.block_crc) {
            return CAL_REC_ERR_BLOCK_CRC;
        }
    }

    if (hdr_out != NULL) {
        *hdr_out = h;
    }
    if (payload_out != NULL) {
        *payload_out = payload;
    }
    return CAL_REC_OK;
}

cal_page_class_t cal_backup_classify_page(const uint8_t *page, uint32_t len)
{
    if (page == NULL || len == 0u) {
        return CAL_PAGE_BLANK;
    }
    bool all_ff = true;
    bool all_00 = true;
    for (uint32_t i = 0; i < len; i++) {
        if (page[i] != 0xFFu) { all_ff = false; }
        if (page[i] != 0x00u) { all_00 = false; }
        if (!all_ff && !all_00) { return CAL_PAGE_PROGRAMMED; }
    }
    if (all_ff) { return CAL_PAGE_BLANK; }
    if (all_00) { return CAL_PAGE_ZEROED; }
    return CAL_PAGE_PROGRAMMED;
}

cal_page_class_t cal_backup_class_merge(cal_page_class_t a, cal_page_class_t b)
{
    return (a == b) ? a : CAL_PAGE_PROGRAMMED;
}

/* ── live page ──────────────────────────────────────────────────────── */

void cal_backup_live_info(const uint8_t *page, cal_live_info_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof *out);
    if (page == NULL) {
        out->page_class  = CAL_PAGE_BLANK;
        out->block_class = CAL_PAGE_BLANK;
        return;
    }
    out->page_class  = cal_backup_classify_page(page, CAL_BACKUP_PAYLOAD_LEN);
    out->block_class = cal_backup_classify_page(page + CAL_BLOCK_OFF, CAL_BLOCK_LEN);
    out->payload_crc = cal_backup_crc32(page, CAL_BACKUP_PAYLOAD_LEN);
    out->block_crc   = cal_backup_crc32(page + CAL_BLOCK_OFF, CAL_BLOCK_LEN);
}

/* ── slot scan ──────────────────────────────────────────────────────── */

static bool all_bytes(const uint8_t *p, uint32_t n, uint8_t v)
{
    for (uint32_t i = 0; i < n; i++) {
        if (p[i] != v) {
            return false;
        }
    }
    return true;
}

bool cal_slot_scan_begin(cal_slot_scan_t *s, const uint8_t *head, uint32_t n,
                         const uint8_t *live)
{
    if (s == NULL) {
        return false;
    }
    memset(s, 0, sizeof *s);
    s->live        = live;
    s->crc         = CAL_BACKUP_CRC32_INIT;
    s->block_crc   = CAL_BACKUP_CRC32_INIT;
    s->page_eq     = (live != NULL);
    s->block_eq    = (live != NULL);
    s->block_first = true;
    s->info.state       = CAL_SLOT_BAD;
    s->info.block_class = CAL_PAGE_BLANK;

    if (head == NULL || n == 0u) {
        s->info.rec_status = CAL_REC_ERR_TRUNCATED;
        return false;
    }
    if (n >= CAL_BACKUP_HEADER_LEN_V1 && all_bytes(head, n, 0xFFu)) {
        s->info.state      = CAL_SLOT_EMPTY;
        s->info.rec_status = CAL_REC_ERR_MAGIC;
        return false;
    }
    cal_rec_status_t st = cal_backup_check_header(head, n, &s->info.hdr);
    s->info.rec_status = st;
    if (st != CAL_REC_OK) {
        return false;
    }
    /* The device keeps only whole 4 KB pages. */
    if (s->info.hdr.payload_len != CAL_BACKUP_PAYLOAD_LEN) {
        s->info.rec_status = CAL_REC_ERR_LEN;
        return false;
    }
    s->want_payload = true;
    return true;
}

void cal_slot_scan_feed(cal_slot_scan_t *s, const uint8_t *chunk, uint32_t n)
{
    if (s == NULL || chunk == NULL || !s->want_payload) {
        return;
    }
    uint32_t total = s->info.hdr.payload_len;
    if (s->fed >= total) {
        return;
    }
    if (n > total - s->fed) {
        n = total - s->fed;
    }
    uint32_t p = s->fed;

    s->crc = cal_backup_crc32_update(s->crc, chunk, n);
    if (s->live != NULL && memcmp(chunk, s->live + p, n) != 0) {
        s->page_eq = false;
    }

    /* Part of this chunk that lies in the cal block. */
    uint32_t lo = p > CAL_BLOCK_OFF ? p : CAL_BLOCK_OFF;
    uint32_t hi = (p + n) < (CAL_BLOCK_OFF + CAL_BLOCK_LEN) ? (p + n)
                                                           : (CAL_BLOCK_OFF + CAL_BLOCK_LEN);
    if (lo < hi) {
        const uint8_t *b = chunk + (lo - p);
        uint32_t bn = hi - lo;
        s->block_crc = cal_backup_crc32_update(s->block_crc, b, bn);
        cal_page_class_t c = cal_backup_classify_page(b, bn);
        s->info.block_class = s->block_first ? c
                                             : cal_backup_class_merge(s->info.block_class, c);
        s->block_first = false;
        if (s->live != NULL && memcmp(b, s->live + lo, bn) != 0) {
            s->block_eq = false;
        }
    }
    s->fed += n;
}

void cal_slot_scan_end(cal_slot_scan_t *s, cal_slot_info_t *out)
{
    if (s == NULL) {
        return;
    }
    if (s->want_payload) {
        s->want_payload = false;
        uint32_t crc = cal_backup_crc32_final(s->crc);
        uint32_t bcrc = cal_backup_crc32_final(s->block_crc);
        if (s->fed != s->info.hdr.payload_len) {
            s->info.rec_status = CAL_REC_ERR_LEN;
        } else if (crc != s->info.hdr.payload_crc) {
            s->info.rec_status = CAL_REC_ERR_PAYLOAD_CRC;
        } else if (s->info.hdr.version == CAL_BACKUP_VERSION_2 &&
                   bcrc != s->info.hdr.block_crc) {
            s->info.rec_status = CAL_REC_ERR_BLOCK_CRC;
        } else {
            s->info.state            = CAL_SLOT_VALID;
            s->info.rec_status       = CAL_REC_OK;
            s->info.block_crc        = bcrc;
            s->info.block_equal_live = s->block_eq;
            s->info.page_equal_live  = s->page_eq;
        }
    }
    if (out != NULL) {
        *out = s->info;
    }
}

void cal_slot_scan_buffer(const uint8_t *slot, uint32_t slot_len,
                          const uint8_t *live, uint32_t chunk,
                          cal_slot_info_t *out)
{
    cal_slot_scan_t s;
    uint32_t hn = slot_len < CAL_BACKUP_HEADER_MAX ? slot_len : CAL_BACKUP_HEADER_MAX;
    if (chunk == 0u) {
        chunk = 1u;
    }
    if (cal_slot_scan_begin(&s, slot, hn, live)) {
        uint32_t off = s.info.hdr.header_len;
        uint32_t end = off + s.info.hdr.payload_len;
        if (end > slot_len) {
            end = slot_len;
        }
        while (off < end) {
            uint32_t n = (end - off) < chunk ? (end - off) : chunk;
            cal_slot_scan_feed(&s, slot + off, n);
            off += n;
        }
    }
    cal_slot_scan_end(&s, out);
}

/* ── decisions ──────────────────────────────────────────────────────── */

static bool uid_mismatch(const cal_backup_header_t *h, const uint8_t *uid)
{
    return h->uid_known && memcmp(h->uid, uid, CAL_UID_LEN) != 0;
}

static cal_plan_t plan(cal_plan_action_t a, cal_plan_why_t w)
{
    cal_plan_t p = { a, w };
    return p;
}

cal_plan_t cal_backup_plan(const cal_live_info_t *live,
                           const cal_slot_info_t *a,
                           const cal_slot_info_t *b,
                           const uint8_t *uid, bool force)
{
    if (live == NULL || a == NULL || b == NULL || uid == NULL) {
        return plan(CAL_PLAN_REFUSE, CAL_WHY_BAD_ARG);
    }
    /* Never back up an empty capture. */
    if (live->page_class != CAL_PAGE_PROGRAMMED) {
        return plan(CAL_PLAN_REFUSE, CAL_WHY_LIVE_BLANK);
    }
    if (live->block_class != CAL_PAGE_PROGRAMMED) {
        return plan(CAL_PLAN_REFUSE, CAL_WHY_LIVE_BLOCK_BLANK);
    }
    /* The first valid backup goes to slot A. */
    if (a->state == CAL_SLOT_EMPTY) {
        return plan(CAL_PLAN_WRITE_A, CAL_WHY_NONE);
    }
    /* A damaged slot A can still be the only copy: never erase it here. */
    if (a->state != CAL_SLOT_VALID) {
        return plan(CAL_PLAN_REFUSE, CAL_WHY_A_DAMAGED);
    }
    if (uid_mismatch(&a->hdr, uid)) {
        return plan(CAL_PLAN_REFUSE, CAL_WHY_UID_MISMATCH);
    }
    if (!a->block_equal_live && !force) {
        return plan(CAL_PLAN_REFUSE, CAL_WHY_BLOCK_DIFFERS);
    }
    if (b->state == CAL_SLOT_VALID && b->page_equal_live && !uid_mismatch(&b->hdr, uid)) {
        return plan(CAL_PLAN_NONE_EQUAL, CAL_WHY_NONE);
    }
    return plan(CAL_PLAN_WRITE_B, CAL_WHY_NONE);
}

static cal_restore_plan_t rplan(cal_restore_action_t a, cal_plan_why_t w)
{
    cal_restore_plan_t p = { a, w };
    return p;
}

cal_restore_plan_t cal_backup_restore_plan(const cal_live_info_t *live,
                                           const cal_slot_info_t *src,
                                           const uint8_t *uid, bool force)
{
    if (live == NULL || src == NULL || uid == NULL) {
        return rplan(CAL_RESTORE_REFUSE, CAL_WHY_BAD_ARG);
    }
    if (src->state != CAL_SLOT_VALID) {
        return rplan(CAL_RESTORE_REFUSE, CAL_WHY_NO_BACKUP);
    }
    if (uid_mismatch(&src->hdr, uid)) {
        return rplan(CAL_RESTORE_REFUSE, CAL_WHY_UID_MISMATCH);
    }
    if (src->block_class != CAL_PAGE_PROGRAMMED) {
        return rplan(CAL_RESTORE_REFUSE, CAL_WHY_BACKUP_BLOCK_BLANK);
    }
    /* A programmed live page is precious: only a human with force overwrites it. */
    if (live->page_class == CAL_PAGE_PROGRAMMED && !force) {
        return rplan(CAL_RESTORE_REFUSE, CAL_WHY_LIVE_PRECIOUS);
    }
    if (src->page_equal_live) {
        return rplan(CAL_RESTORE_NONE_EQUAL, CAL_WHY_NONE);
    }
    return rplan(CAL_RESTORE_WRITE, CAL_WHY_NONE);
}

bool cal_backup_should_auto_restore(cal_page_class_t live, bool backup_valid)
{
    if (!backup_valid) {
        return false;
    }
    /* Only when the live page holds nothing. A programmed page is precious and
     * is never touched without a human. */
    return live == CAL_PAGE_BLANK || live == CAL_PAGE_ZEROED;
}

const char *cal_rec_status_str(cal_rec_status_t s)
{
    switch (s) {
    case CAL_REC_OK:              return "ok";
    case CAL_REC_ERR_TRUNCATED:   return "truncated";
    case CAL_REC_ERR_MAGIC:       return "no backup (bad magic)";
    case CAL_REC_ERR_VERSION:     return "unknown version";
    case CAL_REC_ERR_LEN:         return "bad length";
    case CAL_REC_ERR_HEADER_CRC:  return "header crc fail";
    case CAL_REC_ERR_PAYLOAD_CRC: return "payload crc fail";
    case CAL_REC_ERR_BLOCK_CRC:   return "cal block crc fail";
    }
    return "unknown";
}

const char *cal_page_class_str(cal_page_class_t c)
{
    switch (c) {
    case CAL_PAGE_BLANK:      return "blank (0xFF)";
    case CAL_PAGE_ZEROED:     return "zeroed (0x00)";
    case CAL_PAGE_PROGRAMMED: return "programmed";
    }
    return "unknown";
}

const char *cal_slot_state_str(cal_slot_state_t s)
{
    switch (s) {
    case CAL_SLOT_EMPTY: return "empty";
    case CAL_SLOT_VALID: return "valid";
    case CAL_SLOT_BAD:   return "damaged";
    }
    return "unknown";
}

const char *cal_plan_why_str(cal_plan_why_t w)
{
    switch (w) {
    case CAL_WHY_NONE:               return "ok";
    case CAL_WHY_LIVE_BLANK:         return "live page is blank or zeroed";
    case CAL_WHY_LIVE_BLOCK_BLANK:   return "live cal block 0x025-0x12E is blank or zeroed";
    case CAL_WHY_A_DAMAGED:          return "slot A holds data but no valid record (not erased)";
    case CAL_WHY_UID_MISMATCH:       return "backup is from another unit (UID differs)";
    case CAL_WHY_BLOCK_DIFFERS:      return "live cal block differs from slot A (use 'cal backup force' to write slot B)";
    case CAL_WHY_NO_BACKUP:          return "no valid backup in the slot";
    case CAL_WHY_BACKUP_BLOCK_BLANK: return "backup cal block is blank or zeroed";
    case CAL_WHY_LIVE_PRECIOUS:      return "live page is programmed (use force)";
    case CAL_WHY_BAD_ARG:            return "bad argument";
    }
    return "unknown";
}
