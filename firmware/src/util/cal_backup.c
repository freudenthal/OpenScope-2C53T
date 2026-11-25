/*
 * cal_backup.c — device glue for factory-cal self-protection. See cal_backup.h.
 *
 * All record/decision logic is in cal_backup_core.c (host-tested): the slot
 * scan, the store plan (which slot, or refuse) and the restore plan. This file
 * only does what the core cannot: read the live MCU page and the UID, move
 * bytes to/from the two W25Q slot regions, and — on restore only — erase and
 * program MCU-internal flash. It never erases a slot that the plan did not
 * pick, and it erases slot A only when the plan says WRITE_A (slot A empty).
 *
 * MEMORY: RAM is nearly full and the shell task stack is 3 KB, so this holds
 * NO 4 KB buffer. The live page is read in place (memory-mapped flash); the
 * W25Q slots are streamed in CAL_BK_CHUNK-byte pieces through one small stack
 * buffer. Every entry point runs on the single shell/boot thread.
 */

#include "cal_backup.h"

#include <string.h>

#include "flash_regions.h"

#ifndef EMULATOR_BUILD
#include "at32f403a_407.h"
#endif

#define CAL_BK_CHUNK    64u

static flash_region_id_t slot_region(cal_slot_id_t s)
{
    return (s == CAL_SLOT_B) ? FLASH_REGION_FACTORY_CAL_BACKUP_B
                             : FLASH_REGION_FACTORY_CAL_BACKUP;
}

const char *cal_slot_name(cal_slot_id_t s)
{
    return (s == CAL_SLOT_B) ? "B" : "A";
}

uint32_t cal_slot_addr(cal_slot_id_t s)
{
    const flash_region_t *r = flash_region_get(slot_region(s));
    return r != NULL ? r->start : 0u;
}

/* ── live MCU page and UID (read-only) ──────────────────────────────── */

/* The live page, read in place. The emulator has no cal page: NULL reads as a
 * blank page in the core, so the store plan refuses and nothing is written. */
static const uint8_t *live_page(void)
{
#ifndef EMULATOR_BUILD
    return (const uint8_t *)CAL_BACKUP_SRC_ADDR;
#else
    return NULL;
#endif
}

void cal_backup_read_uid(uint8_t uid[CAL_UID_LEN])
{
    for (uint32_t i = 0; i < 3u; i++) {
#ifndef EMULATOR_BUILD
        uint32_t w = *(const volatile uint32_t *)(CAL_UID_ADDR + 4u * i);
#else
        uint32_t w = 0u;
#endif
        uid[4u * i + 0u] = (uint8_t)(w);
        uid[4u * i + 1u] = (uint8_t)(w >> 8);
        uid[4u * i + 2u] = (uint8_t)(w >> 16);
        uid[4u * i + 3u] = (uint8_t)(w >> 24);
    }
}

/* ── W25Q slots (streamed) ──────────────────────────────────────────── */

/* Scan one slot through the core. Returns false on a region read error. */
static bool slot_scan(cal_slot_id_t id, const uint8_t *live, cal_slot_info_t *out)
{
    flash_region_id_t r = slot_region(id);
    uint8_t head[CAL_BACKUP_HEADER_MAX];
    if (flash_region_read(r, 0, head, sizeof head) != FLASH_REGION_OK) {
        return false;
    }
    cal_slot_scan_t s;
    if (cal_slot_scan_begin(&s, head, sizeof head, live)) {
        uint8_t buf[CAL_BK_CHUNK];
        uint32_t base = s.info.hdr.header_len;
        uint32_t len  = s.info.hdr.payload_len;
        for (uint32_t off = 0; off < len; off += CAL_BK_CHUNK) {
            uint32_t n = (len - off) < CAL_BK_CHUNK ? (len - off) : CAL_BK_CHUNK;
            if (flash_region_read(r, base + off, buf, n) != FLASH_REGION_OK) {
                return false;
            }
            cal_slot_scan_feed(&s, buf, n);
        }
    }
    cal_slot_scan_end(&s, out);
    return true;
}

/* Write a v2 record of the live page into one slot: erase that slot only,
 * payload first, header last (a torn write leaves the header blank: the slot
 * then scans EMPTY or BAD, never valid), then verify by a full scan. */
static cal_bk_status_t slot_write(cal_slot_id_t id, const uint8_t *live,
                                  const cal_live_info_t *li,
                                  const uint8_t uid[CAL_UID_LEN])
{
    flash_region_id_t r = slot_region(id);
    uint8_t hbuf[CAL_BACKUP_HEADER_LEN_V2];
    if (live == NULL ||
        cal_backup_write_header(hbuf, (uint16_t)CAL_BACKUP_VERSION,
                                (uint16_t)CAL_BACKUP_PAYLOAD_LEN, CAL_BACKUP_SRC_ADDR,
                                li->payload_crc, uid, li->block_crc) != sizeof hbuf) {
        return CAL_BK_ERR_ARG;
    }
    if (flash_region_reset(r) != FLASH_REGION_OK) {
        return CAL_BK_ERR_REGION_IO;
    }
    uint8_t buf[CAL_BK_CHUNK];
    for (uint32_t off = 0; off < CAL_BACKUP_PAYLOAD_LEN; off += CAL_BK_CHUNK) {
        memcpy(buf, live + off, CAL_BK_CHUNK);
        if (flash_region_write(r, CAL_BACKUP_HEADER_LEN_V2 + off,
                               buf, CAL_BK_CHUNK) != FLASH_REGION_OK) {
            return CAL_BK_ERR_REGION_IO;
        }
    }
    if (flash_region_write(r, 0, hbuf, sizeof hbuf) != FLASH_REGION_OK) {
        return CAL_BK_ERR_REGION_IO;
    }

    cal_slot_info_t v;
    if (!slot_scan(id, live, &v) || v.state != CAL_SLOT_VALID ||
        !v.page_equal_live || v.hdr.payload_crc != li->payload_crc ||
        !v.hdr.uid_known || memcmp(v.hdr.uid, uid, CAL_UID_LEN) != 0) {
        return CAL_BK_ERR_VERIFY;
    }
    return CAL_BK_OK;
}

cal_bk_status_t cal_backup_store(bool force, cal_plan_t *plan_out)
{
    const uint8_t *live = live_page();
    cal_live_info_t li;
    uint8_t uid[CAL_UID_LEN];
    cal_slot_info_t a, b;
    cal_plan_t p = { CAL_PLAN_REFUSE, CAL_WHY_BAD_ARG };

    cal_backup_live_info(live, &li);
    cal_backup_read_uid(uid);
    if (!slot_scan(CAL_SLOT_A, live, &a) || !slot_scan(CAL_SLOT_B, live, &b)) {
        if (plan_out != NULL) { *plan_out = p; }
        return CAL_BK_ERR_REGION_IO;
    }
    p = cal_backup_plan(&li, &a, &b, uid, force);
    if (plan_out != NULL) { *plan_out = p; }

    switch (p.action) {
    case CAL_PLAN_WRITE_A:
        /* Second guard on the one rule that must hold: A is written only empty. */
        if (a.state != CAL_SLOT_EMPTY) {
            return CAL_BK_ERR_ARG;
        }
        return slot_write(CAL_SLOT_A, live, &li, uid);
    case CAL_PLAN_WRITE_B:
        return slot_write(CAL_SLOT_B, live, &li, uid);
    case CAL_PLAN_NONE_EQUAL:
        return CAL_BK_EQUAL;
    case CAL_PLAN_REFUSE:
        break;
    }
    return CAL_BK_REFUSED;
}

cal_bk_status_t cal_backup_status(cal_backup_report_t *r)
{
    if (r == NULL) {
        return CAL_BK_ERR_ARG;
    }
    memset(r, 0, sizeof(*r));
    const uint8_t *live = live_page();
    cal_backup_live_info(live, &r->live);
    cal_backup_read_uid(r->uid);
    for (uint32_t i = 0; i < CAL_SLOT_COUNT; i++) {
        r->slot_read_ok[i] = slot_scan((cal_slot_id_t)i, live, &r->slot[i]);
    }
    return CAL_BK_OK;
}

/* ── restore: W25Q -> MCU flash. The dangerous direction. ───────────── */

#ifndef EMULATOR_BUILD
/* Erase the sectors covering the cal page, then program it from the slot
 * payload, streamed. Erases in 2 KB steps so the 4 KB page is fully cleared
 * whether the part's sector is 2 KB or 4 KB (a repeat erase of one 4 KB sector
 * is a harmless no-op). Returns false on any erase/program/region-read failure. */
static bool mcu_flash_restore_from_region(flash_region_id_t r, uint32_t payload_off)
{
    bool ok = true;
    flash_unlock();

    for (uint32_t off = 0; off < CAL_BACKUP_PAYLOAD_LEN; off += 2048u) {
        if (flash_sector_erase(CAL_BACKUP_SRC_ADDR + off) != FLASH_OPERATE_DONE) {
            ok = false;
            break;
        }
    }
    if (ok) {
        uint8_t buf[CAL_BK_CHUNK];
        for (uint32_t off = 0; off < CAL_BACKUP_PAYLOAD_LEN && ok; off += CAL_BK_CHUNK) {
            if (flash_region_read(r, payload_off + off, buf, CAL_BK_CHUNK) != FLASH_REGION_OK) {
                ok = false;
                break;
            }
            for (uint32_t i = 0; i < CAL_BK_CHUNK; i += 4u) {
                uint32_t w = (uint32_t)buf[i] |
                             ((uint32_t)buf[i + 1] << 8) |
                             ((uint32_t)buf[i + 2] << 16) |
                             ((uint32_t)buf[i + 3] << 24);
                if (flash_word_program(CAL_BACKUP_SRC_ADDR + off + i, w) != FLASH_OPERATE_DONE) {
                    ok = false;
                    break;
                }
            }
        }
    }

    flash_lock();
    return ok;
}
#endif /* !EMULATOR_BUILD */

cal_bk_status_t cal_backup_restore(cal_slot_id_t src, bool force,
                                   cal_restore_plan_t *plan_out)
{
    const uint8_t *live = live_page();
    cal_live_info_t li;
    uint8_t uid[CAL_UID_LEN];
    cal_slot_info_t s;
    cal_restore_plan_t p = { CAL_RESTORE_REFUSE, CAL_WHY_BAD_ARG };

    if (src != CAL_SLOT_A && src != CAL_SLOT_B) {
        if (plan_out != NULL) { *plan_out = p; }
        return CAL_BK_ERR_ARG;
    }
    cal_backup_live_info(live, &li);
    cal_backup_read_uid(uid);
    if (!slot_scan(src, live, &s)) {
        if (plan_out != NULL) { *plan_out = p; }
        return CAL_BK_ERR_REGION_IO;
    }
    p = cal_backup_restore_plan(&li, &s, uid, force);
    if (plan_out != NULL) { *plan_out = p; }
    if (p.action == CAL_RESTORE_REFUSE) {
        return CAL_BK_REFUSED;
    }
    if (p.action == CAL_RESTORE_NONE_EQUAL) {
        return CAL_BK_EQUAL;
    }

#ifndef EMULATOR_BUILD
    if (!mcu_flash_restore_from_region(slot_region(src), s.hdr.header_len)) {
        return CAL_BK_ERR_MCU_FLASH;
    }
    /* Verify the page now matches the slot, byte for byte. */
    cal_slot_info_t v;
    if (!slot_scan(src, live, &v) || v.state != CAL_SLOT_VALID || !v.page_equal_live) {
        return CAL_BK_ERR_MCU_FLASH;
    }
    return CAL_BK_OK;
#else
    return CAL_BK_ERR_MCU_FLASH;
#endif
}

cal_bk_status_t cal_backup_maybe_auto_restore(void)
{
    const uint8_t *live = live_page();
    cal_live_info_t li;
    cal_slot_info_t a;
    cal_backup_live_info(live, &li);
    if (!slot_scan(CAL_SLOT_A, live, &a)) {
        return CAL_BK_ERR_REGION_IO;
    }
    if (!cal_backup_should_auto_restore(li.page_class, a.state == CAL_SLOT_VALID)) {
        return CAL_BK_OK;   /* deliberate no-op */
    }
    return cal_backup_restore(CAL_SLOT_A, false, NULL);
}

const char *cal_bk_status_str(cal_bk_status_t s)
{
    switch (s) {
    case CAL_BK_OK:               return "ok";
    case CAL_BK_EQUAL:            return "equal, no write";
    case CAL_BK_REFUSED:          return "refused";
    case CAL_BK_ERR_REGION_IO:    return "W25Q region io error";
    case CAL_BK_ERR_VERIFY:       return "backup readback/verify failed";
    case CAL_BK_ERR_MCU_FLASH:    return "MCU flash erase/program/verify failed";
    case CAL_BK_ERR_ARG:          return "bad argument";
    }
    return "unknown";
}
