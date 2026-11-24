/*
 * OpenScope 2C53T — fault capture
 *
 * Background (analysis_v120/hardfault_idle_task_2026-08-11.md): this unit
 * HardFaults ~55 s into every boot and has done for weeks. Diagnosing it was
 * needlessly hard because:
 *
 *   * MemManage / BusFault / UsageFault handlers were all DISABLED (SHCSR = 0),
 *     so every fault escalated to HardFault and the specific cause was lost.
 *   * CFSR is write-1-to-clear and nothing ever cleared it, so its bits
 *     ACCUMULATE across every fault since reset — the value describes a smear of
 *     events, not one event.
 *   * the default handlers are a bare `b .`, so nothing was recorded at all and
 *     the evidence had to be reconstructed by hand over SWD.
 *
 * This fixes all three. The record lives in .noinit so it SURVIVES a reset,
 * which matters for an intermittent fault: the device can die, reset, and still
 * tell you what happened on the previous boot.
 *
 * Stage-160 (2026-11-23): every fault path now ends in a reset (fault_restart)
 * instead of a spin with the power hold on, and the shell `status` command
 * reports the record of the previous boot.
 */

#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>
#include <stdbool.h>

/* 0xFA17ED00 until 2026-11-23. Bumped when the record gained the STACK kind,
 * the boot number of the boot that wrote it, and the task name (stage-160), so
 * a record left in .noinit by an older image is not read as a new one. */
#define FAULT_MAGIC 0xFA17ED01u

/* Which handler ran. Recorded because with the configurable handlers enabled a
 * BusFault no longer masquerades as a HardFault. */
typedef enum {
    FAULT_KIND_NONE = 0,
    FAULT_KIND_HARD,
    FAULT_KIND_MEMMANAGE,
    FAULT_KIND_BUS,
    FAULT_KIND_USAGE,
    FAULT_KIND_STACK,      /* vApplicationStackOverflowHook (not a CPU fault):
                            * pc/lr are the task's saved context, sp is its
                            * pxTopOfStack, task is the FreeRTOS task name. */
} fault_kind_t;

typedef struct {
    uint32_t magic;        /* FAULT_MAGIC once fully written */
    uint32_t kind;         /* fault_kind_t */
    uint32_t boot_count;   /* fault_boot_count() of the boot that wrote the
                            * record; compare with the current one for
                            * "boots ago" */

    /* SCB fault status, sampled before anything else can disturb it. */
    uint32_t cfsr, hfsr, dfsr, afsr, mmfar, bfar;

    /* The exception frame the core stacked. */
    uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr;

    uint32_t exc_return;   /* bit2: 0 = frame on MSP, 1 = on PSP */
    uint32_t sp;           /* the stack the frame was found on */
    uint32_t cyccnt;       /* DWT cycle counter at fault. NOT a FreeRTOS tick:
                            * the handler must not call FreeRTOS (see fault.c). */
    char     task[16];     /* CPU fault in a task (frame on PSP): the FreeRTOS
                            * task name, copied AFTER the record is marked
                            * valid, so a corrupt TCB cannot cost the record.
                            * "ISR" for a fault in handler mode. Empty if the
                            * name could not be read: then identify the task
                            * from `sp`, which lands in its stack. */
} fault_record_t;

/* Non-static so the address comes out of the ELF with nm and the record can be
 * read over SWD on a dead target — which is how this one has to be read. */
extern volatile fault_record_t g_fault;

/*
 * Enable the configurable fault handlers, clear stale CFSR/HFSR bits, count
 * this boot and copy the record of an earlier boot. Call early in main(),
 * before anything that could fault. Safe before the scheduler starts.
 */
void fault_init(void);

/* Boot number, kept in .noinit: 1 after a power-on (or the first boot of an
 * image with a new layout), +1 at each reset that keeps SRAM. */
uint32_t fault_boot_count(void);

/* The record found at boot (written by an earlier boot). False if none.
 * *read (may be NULL) is true once fault_previous_mark_read() ran. */
bool fault_previous(fault_record_t *out, bool *read);

/* Invalidate the .noinit record, so the next boot does not report it again.
 * The copy taken at boot stays readable for the rest of this boot. */
void fault_previous_mark_read(void);

const char *fault_kind_name(uint32_t kind);

/* Record a fault that is not a CPU exception (FreeRTOS hooks). Call with
 * interrupts off. Does not reset: call fault_restart() after the red screen. */
void fault_record_event(uint32_t kind, const char *task,
                        uint32_t pc, uint32_t lr, uint32_t sp);

/* Busy wait on the DWT cycle counter. Interrupt-free, RTOS-free. */
void fault_delay_ms(uint32_t ms);

/* End of every fault path (stage-160): wait FAULT_RESTART_DELAY_MS so the red
 * screen can be read, then NVIC_SystemReset(). On USB power the unit boots
 * again; on battery the reset releases PC9 (power hold) and the unit switches
 * off. With FAULT_HALT_FOR_SWD=1 it spins instead (bench builds only: SWD
 * kills the CPU on a unit with RDP set). The emulator build also spins. */
#define FAULT_RESTART_DELAY_MS 1500u
void fault_restart(void) __attribute__((noreturn));

/* Supplied by watchdog.c (it has the FreeRTOS headers; fault.c must not):
 * name of the running task, or NULL if the scheduler is not running or the
 * current TCB pointer is not in SRAM. Reads memory only. */
const char *health_running_task_name(void);

#endif /* FAULT_H */
