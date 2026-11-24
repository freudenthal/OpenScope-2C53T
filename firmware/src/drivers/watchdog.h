/*
 * OpenScope 2C53T - Watchdog & System Health Monitor
 *
 * Uses the GD32F307 FWDGT (Free Watchdog Timer) with a cooperative
 * health-check pattern: the watchdog is only fed when ALL registered
 * tasks have checked in within the deadline. If any task hangs, the
 * system resets cleanly instead of freezing forever.
 *
 * This directly addresses community issues #1 (auto-measurement hang),
 * #2 (GUI lockup), and #7 (unexpected restarts/freezes).
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

/* Maximum number of tasks that can register with the health monitor */
#define HEALTH_MAX_TASKS    6

/* Deadline: if a task hasn't checked in for this many ticks, it's stalled */
#define HEALTH_DEADLINE_MS  2000

/* ═══════════════════════════════════════════════════════════════════
 * Watchdog (FWDGT) — low-level
 * ═══════════════════════════════════════════════════════════════════ */

/* FWDGT timeout. 8 s nominal from the LICK (~40 kHz, wide tolerance) since
 * stage-160 (3 s before): margin for fpga_acq_pause() and settings writes on
 * the monitored tasks. A stall resets HEALTH_DEADLINE_MS + up to this later. */
#define WATCHDOG_TIMEOUT_MS 8000

/* Initialize FWDGT with WATCHDOG_TIMEOUT_MS (must be > HEALTH_DEADLINE_MS).
 * Called only in builds that arm the watchdog (main.c: not GUEST_BUILD, or
 * OPENSCOPE_WATCHDOG=1). It cannot be stopped by software once armed. */
void watchdog_init(void);

/* Feed the watchdog — only called by the health monitor, not by tasks */
void watchdog_feed(void);

/* ═══════════════════════════════════════════════════════════════════
 * Task Health Monitor — cooperative check-in pattern
 * ═══════════════════════════════════════════════════════════════════ */

typedef struct {
    const char    *name;               /* Task name (for diagnostics) */
    TaskHandle_t   handle;             /* FreeRTOS task handle */
    TickType_t     last_checkin;        /* Tick count of last check-in */
    UBaseType_t    stack_hwm;           /* Stack high water mark (words) */
    bool           registered;         /* Slot in use */
} task_health_t;

/* Stall record (stage-160). When health_check() first decides not to feed the
 * watchdog, it writes this to .noinit: the reset that follows keeps it, and the
 * shell `status` reports it on the next boot. The live tools of C-110
 * (draw_watch.py, task_pc.py) have no time once the watchdog is on. */
#define STALL_MAGIC 0x57A11ED1u
typedef struct {
    uint32_t magic;        /* STALL_MAGIC once fully written */
    uint32_t boot_count;   /* fault_boot_count() of the boot that wrote it */
    char     task[16];     /* health_register() name of the stalled task */
    uint32_t since_ms;     /* ms since its last check-in, at the decision */
    uint32_t uptime_ms;    /* tick count at the decision */
    uint32_t top;          /* its pxTopOfStack (0 = TCB not readable) */
    uint32_t exc_return;   /* from the saved frame: bit4 0 = FP context saved */
    uint32_t pc, lr;       /* its saved context: the frame task_pc.py reads */
    uint32_t recovered_ms; /* 0 = never checked in again (a watchdog reset
                            * followed if the watchdog is armed); else ms from
                            * the decision to the next all-healthy check */
} stall_record_t;

extern volatile stall_record_t g_stall;

/* Copy the stall record of an earlier boot. Call once at boot, after
 * fault_init() (it uses the boot number) and before the scheduler starts. */
void health_records_init(void);

/* The stall record found at boot. False if none. *read (may be NULL) is true
 * once health_stall_mark_read() ran. */
bool health_stall_previous(stall_record_t *out, bool *read);

/* Invalidate the .noinit stall record (if it is still that one), so the next
 * boot does not report it again. The boot copy stays readable. */
void health_stall_mark_read(void);

/* Saved context of a task that is not running (ARM_CM4F frame at its
 * pxTopOfStack). Zeros for what cannot be read: every pointer is range-checked
 * against SRAM first, so it is safe in a hook with a damaged stack. */
void health_saved_context(TaskHandle_t h, uint32_t *top_out,
                          uint32_t *exc_out, uint32_t *pc_out, uint32_t *lr_out);

/* Register a task for health monitoring. Returns slot index or -1 on failure. */
int health_register(const char *name, TaskHandle_t handle);

/* Task check-in: call this periodically from each task's main loop */
void health_checkin(int slot);

/* Run one health check cycle: update HWMs, check deadlines, feed or starve WDT.
 * Called from a timer callback or dedicated low-priority task. */
void health_check(void);

/* Get read-only pointer to task health array (for diagnostics display) */
const task_health_t *health_get_tasks(void);

/* Get count of registered tasks */
int health_get_count(void);

/* Check if a specific task is stalled */
bool health_is_stalled(int slot);

/* ═══════════════════════════════════════════════════════════════════
 * Fault display — show error on LCD before reset
 * ═══════════════════════════════════════════════════════════════════ */

/* Display a fatal error on the LCD (red screen of death).
 * This bypasses FreeRTOS and writes directly to the LCD. It returns at once;
 * the caller then calls fault_restart() (fault.h), which waits and resets. */
void fault_display(const char *title, const char *detail);
void fault_assert(const char *file, int line);

#endif /* WATCHDOG_H */
