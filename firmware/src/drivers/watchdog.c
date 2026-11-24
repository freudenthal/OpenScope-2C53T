/*
 * OpenScope 2C53T - Watchdog & System Health Monitor
 *
 * Cooperative watchdog pattern:
 *   1. Each task registers with health_register() at startup
 *   2. Each task calls health_checkin() in its main loop
 *   3. A timer periodically calls health_check(), which:
 *      - Updates stack high water marks
 *      - Checks if any task has missed its deadline
 *      - Feeds the FWDGT only if all tasks are healthy
 *   4. If any task is stalled, the FWDGT starves and resets the MCU
 *
 * The FWDGT timeout (WATCHDOG_TIMEOUT_MS, 8 s) is much longer than the health
 * check interval, so the monitor has time to detect and record the problem
 * (stall record, stage-160) before the reset.
 */

#include "watchdog.h"
#include "fault.h"
#include "lcd.h"
#include <string.h>
#include <stdio.h>

#ifndef EMULATOR_BUILD
#include "at32f403a_407_wdt.h"
#endif

/* ═══════════════════════════════════════════════════════════════════
 * Health monitor state
 * ═══════════════════════════════════════════════════════════════════ */

static task_health_t health_tasks[HEALTH_MAX_TASKS];
static int health_count = 0;

/* .noinit: survives the watchdog reset that follows a stall. */
volatile stall_record_t g_stall __attribute__((section(".noinit")));

static stall_record_t stall_prev;
static bool stall_prev_valid;
static bool stall_prev_read;
static bool stall_open;           /* a record of this boot waits for recovery */

/* ═══════════════════════════════════════════════════════════════════
 * Watchdog (FWDGT) — low-level
 * ═══════════════════════════════════════════════════════════════════ */

void watchdog_init(void)
{
#ifndef EMULATOR_BUILD
    /*
     * FWDGT clock = LICK (~40 kHz nominal; the datasheet tolerance is wide,
     * so the real timeout is measured on the bench, not assumed).
     * Prescaler DIV128: 40000 / 128 = 312.5 Hz per tick.
     * Reload 2500: 2500 / 312.5 = 8.0 seconds (WATCHDOG_TIMEOUT_MS).
     *
     * The health monitor runs every 500 ms and feeds the WDT if all tasks
     * are healthy. A stalled task stops the feed HEALTH_DEADLINE_MS after its
     * last check-in, so the reset comes 2 s + up to 8 s after the stall.
     */
    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_128);
    wdt_reload_value_set(2500);
    wdt_counter_reload();
    wdt_enable();
#endif
}

void watchdog_feed(void)
{
#ifndef EMULATOR_BUILD
    wdt_counter_reload();
#endif
}

/* ═══════════════════════════════════════════════════════════════════
 * Saved-context readers (memory reads only; no FreeRTOS state change)
 * ═══════════════════════════════════════════════════════════════════ */

/* AT32F403A SRAM with EOPB0 = 0xFE: 224 KB from 0x20000000. */
static bool ram_ok(uint32_t addr, uint32_t len)
{
    return (addr & 3u) == 0u && addr >= 0x20000000u &&
           len <= 0x38000u && addr <= 0x20038000u - len;
}

const char *health_running_task_name(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return NULL;
    }
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    /* pcTaskGetName() asserts only on NULL; the range check excludes it. */
    if (!ram_ok((uint32_t)h, 64u)) {
        return NULL;
    }
    return pcTaskGetName(h);
}

/*
 * The frame the ARM_CM4F port saves for a task that is not running: at
 * pxTopOfStack (first member of the TCB) r4-r11, EXC_RETURN, then s16-s31 if
 * EXC_RETURN bit 4 is 0, then the hardware frame r0 r1 r2 r3 r12 lr pc xpsr.
 * Same walk as .llm openscope-bench/task_pc.py.
 */
void health_saved_context(TaskHandle_t h, uint32_t *top_out,
                          uint32_t *exc_out, uint32_t *pc_out, uint32_t *lr_out)
{
    *top_out = *exc_out = *pc_out = *lr_out = 0;
    if (!ram_ok((uint32_t)h, 4u)) {
        return;
    }
    uint32_t top = *(const volatile uint32_t *)h;
    if (!ram_ok(top, 9u * 4u)) {
        return;
    }
    const volatile uint32_t *sw = (const volatile uint32_t *)top;
    uint32_t exc = sw[8];
    uint32_t fp  = ((exc & 0x10u) == 0u) ? 16u : 0u;
    uint32_t hw  = top + 4u * (9u + fp);
    *top_out = top;
    *exc_out = exc;
    if (!ram_ok(hw, 8u * 4u)) {
        return;
    }
    *lr_out = ((const volatile uint32_t *)hw)[5];
    *pc_out = ((const volatile uint32_t *)hw)[6];
}

/* ═══════════════════════════════════════════════════════════════════
 * Stall record
 * ═══════════════════════════════════════════════════════════════════ */

void health_records_init(void)
{
    stall_prev_valid = (g_stall.magic == STALL_MAGIC);
    stall_prev_read  = false;
    stall_open       = false;
    if (stall_prev_valid) {
        memcpy(&stall_prev, (const void *)&g_stall, sizeof(stall_prev));
        stall_prev.task[sizeof(stall_prev.task) - 1] = '\0';
    }
}

bool health_stall_previous(stall_record_t *out, bool *read)
{
    if (stall_prev_valid && out != NULL) {
        memcpy(out, &stall_prev, sizeof(*out));
    }
    if (read != NULL) {
        *read = stall_prev_read;
    }
    return stall_prev_valid;
}

void health_stall_mark_read(void)
{
    if (!stall_prev_valid || stall_prev_read) {
        return;
    }
    if (g_stall.magic == STALL_MAGIC && g_stall.boot_count == stall_prev.boot_count) {
        g_stall.magic = 0;
    }
    stall_prev_read = true;
}

static void stall_record_write(int slot, TickType_t now)
{
    const task_health_t *t = &health_tasks[slot];
    uint32_t top, exc, pc, lr;

    health_saved_context(t->handle, &top, &exc, &pc, &lr);

    g_stall.magic = 0;                     /* invalid while being written */
    g_stall.boot_count = fault_boot_count();
    memset((void *)g_stall.task, 0, sizeof(g_stall.task));
    if (t->name != NULL) {
        strncpy((char *)g_stall.task, t->name, sizeof(g_stall.task) - 1u);
    }
    g_stall.since_ms   = (uint32_t)((now - t->last_checkin) * portTICK_PERIOD_MS);
    g_stall.uptime_ms  = (uint32_t)(now * portTICK_PERIOD_MS);
    g_stall.top        = top;
    g_stall.exc_return = exc;
    g_stall.pc         = pc;
    g_stall.lr         = lr;
    g_stall.recovered_ms = 0;
    g_stall.magic = STALL_MAGIC;
}

/* ═══════════════════════════════════════════════════════════════════
 * Task Health Monitor
 * ═══════════════════════════════════════════════════════════════════ */

int health_register(const char *name, TaskHandle_t handle)
{
    if (health_count >= HEALTH_MAX_TASKS)
        return -1;

    int slot = health_count;
    health_tasks[slot].name         = name;
    health_tasks[slot].handle       = handle;
    health_tasks[slot].last_checkin = xTaskGetTickCount();
    health_tasks[slot].stack_hwm    = 0;
    health_tasks[slot].registered   = true;
    health_count++;
    return slot;
}

void health_checkin(int slot)
{
    if (slot >= 0 && slot < HEALTH_MAX_TASKS && health_tasks[slot].registered) {
        health_tasks[slot].last_checkin = xTaskGetTickCount();
    }
}

bool health_is_stalled(int slot)
{
    if (slot < 0 || slot >= HEALTH_MAX_TASKS || !health_tasks[slot].registered)
        return false;

    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = now - health_tasks[slot].last_checkin;
    return elapsed > pdMS_TO_TICKS(HEALTH_DEADLINE_MS);
}

void health_check(void)
{
    bool all_healthy = true;
    int first_stalled = -1;

    for (int i = 0; i < health_count; i++) {
        if (!health_tasks[i].registered)
            continue;

        /* Update stack high water mark */
        health_tasks[i].stack_hwm = uxTaskGetStackHighWaterMark(
            health_tasks[i].handle);

        /* Check deadline */
        if (health_is_stalled(i)) {
            all_healthy = false;
            if (first_stalled < 0) {
                first_stalled = i;
            }
        }
    }

    TickType_t now = xTaskGetTickCount();
    if (all_healthy) {
        if (stall_open) {
            /* The task came back before the watchdog fired: keep the record,
             * mark how long the stall lasted. */
            g_stall.recovered_ms = (uint32_t)(now * portTICK_PERIOD_MS) - g_stall.uptime_ms;
            if (g_stall.recovered_ms == 0u) {
                g_stall.recovered_ms = 1u;
            }
            stall_open = false;
        }
        watchdog_feed();
    } else if (!stall_open) {
        /* First decision not to feed: record who and where, once. */
        stall_record_write(first_stalled, now);
        stall_open = true;
    }
    /* If not all healthy, we deliberately don't feed the watchdog.
     * The FWDGT will reset the MCU after its timeout expires. */
}

const task_health_t *health_get_tasks(void)
{
    return health_tasks;
}

int health_get_count(void)
{
    return health_count;
}

/* ═══════════════════════════════════════════════════════════════════
 * Fault display — red screen of death
 * ═══════════════════════════════════════════════════════════════════ */

void fault_display(const char *title, const char *detail)
{
    /*
     * This runs outside FreeRTOS (interrupts disabled, from a hook or a fault
     * handler). Write directly to LCD — no queues, no RTOS calls, no wait:
     * the caller calls fault_restart(), which waits FAULT_RESTART_DELAY_MS
     * on the DWT counter and resets (stage-160; the old 2 s nop loop here
     * was not timed and never reset by itself in guest builds).
     */
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, COLOR_RED);

    /* Title bar */
    lcd_fill_rect(0, 0, LCD_WIDTH, 30, 0x8000); /* Dark red */
    lcd_draw_string(10, 8, "SYSTEM FAULT", COLOR_WHITE, 0x8000);

    /* Error details */
    lcd_draw_string(10, 50, title, COLOR_WHITE, COLOR_RED);

    if (detail != NULL) {
        lcd_draw_string(10, 80, detail, COLOR_YELLOW, COLOR_RED);
    }

    lcd_draw_string(10, 130, "Restart in 2 s (on battery:", COLOR_WHITE, COLOR_RED);
    lcd_draw_string(10, 155, "the unit switches off).", COLOR_WHITE, COLOR_RED);

    lcd_draw_string(10, 200, "Shell 'status' shows the record.",
                    COLOR_GRAY, COLOR_RED);
#ifdef GUEST_BUILD
    /* Factory IAP bootloader: MENU held through a pinhole reset (C-20). */
    lcd_draw_string(10, 218, "Loop: hold MENU + pinhole reset.",
                    COLOR_GRAY, COLOR_RED);
#else
    /* Our HID bootloader (0x08000000): recovery gesture POWER+PRM. */
    lcd_draw_string(10, 218, "Loop: hold POWER+PRM at reset.",
                    COLOR_GRAY, COLOR_RED);
#endif
}

void fault_assert(const char *file, int line)
{
    char detail[64];
    const char *base = file;
    const char *p = file;

    while (p != NULL && *p != '\0') {
        if (*p == '/' || *p == '\\') {
            base = p + 1;
        }
        p++;
    }

    snprintf(detail, sizeof(detail), "%s:%d", base, line);
    fault_display("ASSERT FAIL", detail);
    /* Reset rather than hang forever on the red screen (PR #13). The emulator
     * build halts in fault_restart(), so the assert text stays on screen. */
    fault_restart();
}
