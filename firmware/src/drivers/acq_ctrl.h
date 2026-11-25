/*
 * acq_ctrl.h — pure decisions of the acquisition park handshake and the host
 * scope sets (stage-230, G-300). No FreeRTOS, no vendor headers: the host
 * tests (firmware/tests/test_remote_proto.c) compile it as is.
 *
 * Park owner. fpga_acq_pause() has two requesters, the key task (priority 4)
 * and the USB task (priority 2). Before stage-230 they shared one request
 * flag and one ack flag with no owner: one task's resume ended the other's
 * park, and one ack satisfied both. Now a mutex gives one owner. The FreeRTOS
 * kernel asserts when a task gives a mutex that it does not hold
 * (tasks.c xTaskPriorityDisinherit), so fpga.c asks these helpers first:
 *   pause:  acq_owner_nested() true  -> acq_owner_enter_nested(), no take;
 *           else take the mutex, park, then acq_owner_took().
 *   resume: acq_owner_release() says GIVE (last level: clear the request,
 *           give the mutex), KEEP (a nested level ended) or REFUSE (the
 *           caller is not the holder: do nothing).
 * The holder is an opaque task handle; NULL is never a holder.
 */
#ifndef ACQ_CTRL_H
#define ACQ_CTRL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct {
    const void *holder;     /* task that holds the park, NULL = none */
    uint8_t     depth;      /* nested pause count of that task */
} acq_owner_t;

typedef enum {
    ACQ_RELEASE_REFUSE = 0, /* caller does not hold the park: change nothing */
    ACQ_RELEASE_KEEP,       /* a nested level ended: the park stays */
    ACQ_RELEASE_GIVE,       /* last level: end the park, give the mutex */
} acq_release_t;

/* The caller already holds the park (a pause inside a pause of its own). */
static inline bool acq_owner_nested(const acq_owner_t *o, const void *me)
{
    return me != NULL && o->depth > 0u && o->holder == me;
}

/* A nested pause by the holder. False (no change) if the count is full. */
static inline bool acq_owner_enter_nested(acq_owner_t *o, const void *me)
{
    if (!acq_owner_nested(o, me) || o->depth == UINT8_MAX)
        return false;
    o->depth++;
    return true;
}

/* The caller took the mutex and the task parked: it is the holder now. */
static inline void acq_owner_took(acq_owner_t *o, const void *me)
{
    o->holder = me;
    o->depth = 1u;
}

static inline acq_release_t acq_owner_release(acq_owner_t *o, const void *me)
{
    if (me == NULL || o->depth == 0u || o->holder != me)
        return ACQ_RELEASE_REFUSE;
    if (o->depth > 1u) {
        o->depth--;
        return ACQ_RELEASE_KEEP;
    }
    o->depth = 0u;
    o->holder = NULL;
    return ACQ_RELEASE_GIVE;
}

/* U-4. Host scope sets (SET_TIMEBASE, SET_VDIV, SET_TRIGGER) only in scope
 * mode: SET_VDIV drives the front-end relays that the meter also uses.
 * `mode` is current_mode; ACQ_CTRL_SCOPE_MODE is MODE_OSCILLOSCOPE
 * (usb_debug.c checks that with a _Static_assert). */
#define ACQ_CTRL_SCOPE_MODE 0u
static inline bool acq_scope_set_allowed(uint8_t mode)
{
    return mode == ACQ_CTRL_SCOPE_MODE;
}

/* A-8. A SET_TRIGGER needs the register writes (and so a park and a
 * re-prime) only when the trigger code (reg 0x08) or the source (reg 0x06)
 * changes. A mode or edge change alone is state only. */
static inline bool acq_trigger_needs_write(uint8_t code_now, uint8_t src_now,
                                           uint8_t code_new, uint8_t src_new)
{
    return code_new != code_now || src_new != src_now;
}

#endif /* ACQ_CTRL_H */
