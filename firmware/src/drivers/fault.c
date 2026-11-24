/*
 * OpenScope 2C53T — fault capture. See fault.h for why this exists.
 */

#include "fault.h"
#include "at32f403a_407.h"
/* Deliberately NOT including FreeRTOS headers: nothing in the fault path
 * may call a FreeRTOS API. See the note in fault_capture(). */

#include <string.h>

#ifndef FAULT_HALT_FOR_SWD
#define FAULT_HALT_FOR_SWD 0
#endif

/* watchdog.c. Plain LCD writes over EXMC: no FreeRTOS call, no allocation.
 * Declared here so this file does not pull in the FreeRTOS headers. */
extern void fault_display(const char *title, const char *detail);
extern unsigned int system_core_clock;

/* .noinit is NOLOAD in the linker script, so startup neither zeroes nor loads
 * it: the record survives a reset and can be reported on the next boot. */
volatile fault_record_t g_fault __attribute__((section(".noinit")));

/* Boot counter, also .noinit. Its own magic: SRAM content after a power-on is
 * random, so a mismatch means "first boot since power-on". */
#define BOOT_MAGIC 0xB0075EEDu
static volatile uint32_t s_boot_magic __attribute__((section(".noinit")));
static volatile uint32_t s_boot_count __attribute__((section(".noinit")));

/* Captured before g_fault can be overwritten, so the CURRENT boot can still
 * report what an EARLIER one did. */
static fault_record_t prev;
static bool prev_valid;
static bool prev_read;

/* Set on the first entry into a fault path. A second fault while the first is
 * being reported (for example the red screen faults on the LCD bus) must not
 * overwrite the record: it resets at once. .bss, so it is clear at each boot. */
static volatile uint32_t s_in_fault;

const char *fault_kind_name(uint32_t k)
{
    switch (k) {
        case FAULT_KIND_HARD:       return "HARD";
        case FAULT_KIND_MEMMANAGE:  return "MEM";
        case FAULT_KIND_BUS:        return "BUS";
        case FAULT_KIND_USAGE:      return "USAGE";
        case FAULT_KIND_STACK:      return "STACK";
        default:                    return "?";
    }
}

static void cyccnt_on(void)
{
    if ((DWT->CTRL & 1u) == 0u) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= 1u;            /* CYCCNTENA */
    }
}

void fault_init(void)
{
    if (s_boot_magic == BOOT_MAGIC) {
        s_boot_count = s_boot_count + 1u;
    } else {
        s_boot_magic = BOOT_MAGIC;
        s_boot_count = 1u;
    }

    prev_valid = (g_fault.magic == FAULT_MAGIC);
    prev_read  = false;
    if (prev_valid) {
        memcpy(&prev, (const void *)&g_fault, sizeof(prev));
        prev.task[sizeof(prev.task) - 1] = '\0';
    }

    /* Enable MemManage / BusFault / UsageFault. Without these every fault
     * escalates to HardFault and the specific cause is lost — which is exactly
     * what happened to the ~55 s fault. */
    SCB->SHCSR |= (SCB_SHCSR_MEMFAULTENA_Msk |
                   SCB_SHCSR_BUSFAULTENA_Msk |
                   SCB_SHCSR_USGFAULTENA_Msk);

    /* CFSR is write-1-to-clear and nothing ever cleared it, so its bits used to
     * accumulate across every fault since reset. Clear it so the next value
     * describes ONE event. */
    SCB->CFSR  = SCB->CFSR;
    SCB->HFSR  = SCB->HFSR;

    /* Precise bus faults are much easier to attribute than imprecise ones.
     * ACTLR.DISDEFWBUF forces the write buffer off so a faulting store reports
     * at the instruction that caused it rather than several instructions later.
     * Costs a little store throughput; worth it while this bug is open.
     *
     * ACTLR is at 0xE000E008 and is not a member of SCB_Type in this CMSIS
     * version, hence the direct access. */
    *(volatile uint32_t *)0xE000E008u |= (1u << 1);

    /* DWT cycle counter — the fault handler's only timestamp source, since it
     * must not call into FreeRTOS, and the clock of fault_delay_ms(). At
     * 240 MHz CYCCNT wraps every ~17.9 s, so it gives sub-wrap precision rather
     * than absolute uptime. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= 1u;            /* CYCCNTENA */
}

uint32_t fault_boot_count(void) { return s_boot_count; }

bool fault_previous(fault_record_t *out, bool *read)
{
    if (prev_valid && out != NULL) {
        memcpy(out, &prev, sizeof(*out));
    }
    if (read != NULL) {
        *read = prev_read;
    }
    return prev_valid;
}

void fault_previous_mark_read(void)
{
    if (!prev_valid || prev_read) {
        return;
    }
    /* Clear only if .noinit still holds that same record: a fault of THIS boot
     * would have its own boot_count and must stay. */
    if (g_fault.magic == FAULT_MAGIC && g_fault.boot_count == prev.boot_count) {
        g_fault.magic = 0;
    }
    prev_read = true;
}

void fault_delay_ms(uint32_t ms)
{
    uint32_t per_ms = system_core_clock / 1000u;
    if (per_ms == 0u) {
        per_ms = 240000u;
    }
    cyccnt_on();
    /* Wait in steps of 1 s or less: 1 s at 240 MHz is far below the ~17.9 s
     * CYCCNT wrap, and the unsigned subtraction handles one wrap. */
    while (ms > 0u) {
        uint32_t step = (ms > 1000u) ? 1000u : ms;
        uint32_t start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < step * per_ms) { }
        ms -= step;
    }
}

void fault_restart(void)
{
#if FAULT_HALT_FOR_SWD || defined(EMULATOR_BUILD)
    /* Bench option: keep the frozen state for an SWD read. The emulator build
     * halts too, so the red screen stays for inspection. */
    for (;;) { }
#else
    fault_delay_ms(FAULT_RESTART_DELAY_MS);
    __DSB();
    NVIC_SystemReset();
    for (;;) { }
#endif
}

static void copy_name(volatile char *dst, const char *src)
{
    size_t i = 0;
    if (src != NULL) {
        for (; i < 15u && src[i] != '\0'; i++) {
            char c = src[i];
            dst[i] = (c >= 0x20 && c < 0x7F) ? c : '?';
        }
    }
    dst[i] = '\0';
}

void fault_record_event(uint32_t kind, const char *task,
                        uint32_t pc, uint32_t lr, uint32_t sp)
{
    if (s_in_fault) {
        return;                       /* keep the first record */
    }
    s_in_fault = 1u;

    g_fault.magic = 0;
    g_fault.kind  = kind;
    g_fault.boot_count = s_boot_count;
    g_fault.cfsr  = SCB->CFSR;
    g_fault.hfsr  = SCB->HFSR;
    g_fault.dfsr  = SCB->DFSR;
    g_fault.afsr  = SCB->AFSR;
    g_fault.mmfar = SCB->MMFAR;
    g_fault.bfar  = SCB->BFAR;
    g_fault.r0 = g_fault.r1 = g_fault.r2 = g_fault.r3 = g_fault.r12 = 0;
    g_fault.lr   = lr;
    g_fault.pc   = pc;
    g_fault.xpsr = 0;
    g_fault.exc_return = 0;
    g_fault.sp     = sp;
    g_fault.cyccnt = DWT->CYCCNT;
    copy_name(g_fault.task, task);
    __DMB();
    g_fault.magic = FAULT_MAGIC;
    __DMB();
}

/* "pc=XXXXXXXX " + up to 15 name characters, without printf (no libc in the
 * fault path). */
static void fault_detail(char *out, uint32_t pc, const volatile char *task)
{
    static const char hex[] = "0123456789ABCDEF";
    size_t n = 0;
    out[n++] = 'p'; out[n++] = 'c'; out[n++] = '=';
    for (int sh = 28; sh >= 0; sh -= 4) {
        out[n++] = hex[(pc >> sh) & 0xFu];
    }
    out[n++] = ' ';
    for (size_t i = 0; i < 15u && task[i] != '\0'; i++) {
        out[n++] = task[i];
    }
    out[n] = '\0';
}

/*
 * Record, show, reset. Deliberately minimal and allocation-free: the record is
 * written FIRST, so if anything later in this function faults again (which is
 * how the core ends up in LOCKUP with pc=0xFFFFFFFE) the evidence is already
 * safe in .noinit.
 */
static void fault_capture(uint32_t *frame, uint32_t exc_return, uint32_t kind)
{
    if (s_in_fault) {
        /* A fault while a fault is being reported (red screen, task name):
         * the first record is already valid. Do not touch it; reset. */
#if FAULT_HALT_FOR_SWD
        for (;;) { }
#else
        NVIC_SystemReset();
        for (;;) { }
#endif
    }
    s_in_fault = 1u;

    g_fault.magic = 0;                    /* invalid while being written */

    g_fault.kind  = kind;
    g_fault.boot_count = s_boot_count;
    g_fault.cfsr  = SCB->CFSR;
    g_fault.hfsr  = SCB->HFSR;
    g_fault.dfsr  = SCB->DFSR;
    g_fault.afsr  = SCB->AFSR;
    g_fault.mmfar = SCB->MMFAR;
    g_fault.bfar  = SCB->BFAR;

    g_fault.r0   = frame[0];
    g_fault.r1   = frame[1];
    g_fault.r2   = frame[2];
    g_fault.r3   = frame[3];
    g_fault.r12  = frame[4];
    g_fault.lr   = frame[5];
    g_fault.pc   = frame[6];
    g_fault.xpsr = frame[7];

    g_fault.exc_return = exc_return;
    g_fault.sp         = (uint32_t)frame;

    /* Timestamp from the DWT cycle counter, NOT from FreeRTOS.
     *
     * The first version of this called xTaskGetTickCountFromISR() here and it
     * destroyed the whole handler. That function runs
     * portASSERT_IF_INTERRUPT_PRIORITY_INVALID(), which ALWAYS fails in a fault
     * handler (priority -1/-2, above configMAX_SYSCALL_INTERRUPT_PRIORITY), and
     * this project's configASSERT calls fault_display() — i.e. it drives the
     * LCD over EXMC from inside a fault handler. That faulted again, escalated,
     * and locked the core up at 0xFFFFFFFE with the record already invalidated
     * by the magic=0 above. Measured: SHCSR showed USGFAULTACT set with
     * CFSR = UNDEFINSTR and HFSR = FORCED, and g_fault was never written.
     *
     * Before the magic is set, touch NOTHING but architectural state. No
     * FreeRTOS API, no LCD, no printf. DWT is a core peripheral: a plain
     * register read that cannot assert, block or fault. */
    g_fault.cyccnt = DWT->CYCCNT;
    g_fault.task[0] = '\0';

    __DMB();
    g_fault.magic = FAULT_MAGIC;
    __DMB();

    /* The record is safe from here on. Everything below is best effort; a
     * second fault re-enters above and resets without touching the record.
     *
     * Task name: only for a fault in thread mode (frame on PSP). The helper
     * reads pxCurrentTCB only after a range check, and pcTaskGetName() asserts
     * only on NULL, which the range check excludes. */
    if (exc_return & 4u) {
        copy_name(g_fault.task, health_running_task_name());
    } else {
        copy_name(g_fault.task, "ISR");
    }

#if FAULT_HALT_FOR_SWD
    /* Stop here rather than resetting: the frozen state is what SWD reads.
     * Bench builds only (RDP on unit #1 makes SWD unusable anyway). */
    for (;;) { }
#else
    {
        char detail[32];
        fault_detail(detail, g_fault.pc, g_fault.task);
        switch (kind) {
            case FAULT_KIND_HARD:      fault_display("HARD FAULT", detail);  break;
            case FAULT_KIND_MEMMANAGE: fault_display("MEMORY FAULT", detail); break;
            case FAULT_KIND_BUS:       fault_display("BUS FAULT", detail);   break;
            default:                   fault_display("USAGE FAULT", detail); break;
        }
    }
    fault_restart();
#endif
}

/*
 * EXC_RETURN bit 2 selects which stack the core pushed the frame onto: 0 = MSP
 * (fault in a handler), 1 = PSP (fault in a task). Getting this wrong means
 * reading eight words of unrelated stack and inventing a plausible, wrong
 * answer — so it is resolved here rather than guessed at afterwards.
 */
#define FAULT_ENTRY(name, kind)                     \
    __attribute__((naked)) void name(void)          \
    {                                               \
        __asm volatile (                            \
            "tst   lr, #4              \n"          \
            "ite   eq                  \n"          \
            "mrseq r0, msp             \n"          \
            "mrsne r0, psp             \n"          \
            "mov   r1, lr              \n"          \
            "mov   r2, %0              \n"          \
            "b     fault_capture_c     \n"          \
            :: "i" (kind) : "r0", "r1", "r2"        \
        );                                          \
    }

void fault_capture_c(uint32_t *frame, uint32_t exc_return, uint32_t kind);
void fault_capture_c(uint32_t *frame, uint32_t exc_return, uint32_t kind)
{
    fault_capture(frame, exc_return, kind);
}

FAULT_ENTRY(HardFault_Handler,  FAULT_KIND_HARD)
FAULT_ENTRY(MemManage_Handler,  FAULT_KIND_MEMMANAGE)
FAULT_ENTRY(BusFault_Handler,   FAULT_KIND_BUS)
FAULT_ENTRY(UsageFault_Handler, FAULT_KIND_USAGE)
