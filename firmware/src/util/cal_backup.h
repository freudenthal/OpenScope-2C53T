/*
 * cal_backup.h — factory-cal self-protection: back the MCU cal page up to the
 * external W25Q, and (deliberately, manually) restore it.
 *
 * WHY (2026-08-25)
 * ---------------------------------------------------------------------------
 * The per-device factory calibration is a 4 KB page in MCU-internal flash at
 * 0x08006000. It is NOT regenerable — see cal_backup_core.h and
 * factory_cal_truth_2026-08-14.md. Reflashing this device is the standing
 * threat to it, and we are about to reflash a great deal (the FPGA config-
 * transport bisect). The safe ordering is: capture the page to a place a
 * reflash cannot reach — the external W25Q — BEFORE the reflash spree.
 *
 * cal_dump.c already proved the CPU can READ its own protected flash (an
 * external debugger cannot, and merely attaching one kills this part). This
 * module adds the write side, split so the dangerous direction is opt-in:
 *
 *   cal_backup_store()   MCU page  -> W25Q   SAFE. Only writes the W25Q, which
 *                                            has a fixed, verified driver, and
 *                                            only the slot the core picks.
 *   cal_backup_status()  read-only inspection of both sides.
 *   cal_backup_restore() W25Q -> MCU page    DANGEROUS. Erases + programs MCU
 *                                            flash. Refuses to overwrite a
 *                                            programmed (precious) page unless
 *                                            explicitly forced.
 *
 * STORAGE — flagged for review. The backup lives in a DEDICATED W25Q region
 * (FLASH_REGION_FACTORY_CAL_BACKUP), not a FAT file and not MCU flash:
 *   - Not a FAT file: those live in stock-owned read-only volumes, and the
 *     project forbids inventing cal filenames (scripts/validate_dmm_goal.py).
 *   - Not MCU flash: the entire point is to survive an MCU-flash erase.
 *   - A dedicated region, not scratch/modules: the region layer's default-deny
 *     means a screenshot stream or a settings write can never reach it. This
 *     is data we cannot regenerate.
 * The region is carved from the tail of the general scratch region, inside the
 * 0xF00000+ area proven blank on bench unit #1 — the same basis as usercal,
 * settings, modules and scratch. See flash_regions.c.
 *
 * AUTO-RESTORE at boot is compiled OUT by default (CAL_BACKUP_AUTO_RESTORE=0).
 * Manual `cal restore` is the primary path. Even when enabled it only acts on a
 * BLANK/ZEROED page — cal_backup_should_auto_restore() never overwrites content.
 */

#ifndef CAL_BACKUP_H
#define CAL_BACKUP_H

#include <stdint.h>
#include <stdbool.h>

#include "cal_backup_core.h"

/* Two W25Q slots (stage-175, review F-2). Slot A ("calbackup", 0xFFD000) keeps
 * the first valid backup and `cal backup` never erases it once it is valid.
 * Slot B ("calbackup_b", 0xFFB000) takes the later copies. The slot rules are
 * in cal_backup_plan() (core, host-tested). */
typedef enum {
    CAL_SLOT_A = 0,
    CAL_SLOT_B = 1,
    CAL_SLOT_COUNT
} cal_slot_id_t;

/* MCU unique ID, 3 words (AT32F403A UID_1..3; the vendor CDC descriptor code
 * reads the same words for the USB serial number). */
#define CAL_UID_ADDR            0x1FFFF7E8u

typedef enum {
    CAL_BK_OK = 0,              /* written (store) or restored                   */
    CAL_BK_EQUAL,               /* no write needed: the target already holds it  */
    CAL_BK_REFUSED,             /* the core decision refused; see the plan `why` */
    CAL_BK_ERR_REGION_IO,       /* W25Q region read/write/erase failed           */
    CAL_BK_ERR_VERIFY,          /* stored record did not read back / re-validate */
    CAL_BK_ERR_MCU_FLASH,       /* MCU-flash erase/program/verify failed         */
    CAL_BK_ERR_ARG,             /* bad argument / not initialised                */
} cal_bk_status_t;

typedef struct {
    cal_live_info_t live;                       /* MCU page 0x08006000          */
    uint8_t         uid[CAL_UID_LEN];           /* this unit                    */
    bool            slot_read_ok[CAL_SLOT_COUNT];
    cal_slot_info_t slot[CAL_SLOT_COUNT];
} cal_backup_report_t;

/* Read the MCU unique ID (12 bytes, 3 words little-endian). */
void cal_backup_read_uid(uint8_t uid[CAL_UID_LEN]);

/* Capture the live MCU page into the slot that cal_backup_plan() picks. Erases
 * only the slot it writes; writes the payload first and the v2 header last,
 * then verifies the slot by a full read-back scan. *plan_out (may be NULL)
 * gets the decision (action = slot written, or why it refused). */
cal_bk_status_t cal_backup_store(bool force, cal_plan_t *plan_out);

/* Fill *r from the live page and both slots. Read-only. */
cal_bk_status_t cal_backup_status(cal_backup_report_t *r);

/* Restore slot `src` to MCU flash 0x08006000, as cal_backup_restore_plan()
 * decides. Erases + programs MCU flash and verifies the readback.
 * NEVER sent to the project's unit (I-40); the decision is host-tested. */
cal_bk_status_t cal_backup_restore(cal_slot_id_t src, bool force,
                                   cal_restore_plan_t *plan_out);

/* Boot hook: restore slot A ONLY if the live page is blank/zeroed AND slot A is
 * valid (cal_backup_should_auto_restore). A no-op otherwise. Not called unless
 * CAL_BACKUP_AUTO_RESTORE is defined non-zero. */
cal_bk_status_t cal_backup_maybe_auto_restore(void);

const char *cal_bk_status_str(cal_bk_status_t s);
const char *cal_slot_name(cal_slot_id_t s);
uint32_t    cal_slot_addr(cal_slot_id_t s);

#endif /* CAL_BACKUP_H */
