/*
 * OpenScope 2C53T - USB CDC link decisions (stage-155)
 *
 * The choices usb_debug.c makes about the CDC link — classify a TX stall,
 * decide a self-heal, gate a write, confirm a lost OUT completion, track the
 * host's DTR — as pure functions. No vendor, USB-stack or FreeRTOS header:
 * the caller reads the endpoint register and the stack's flags, turns them
 * into the booleans below, and applies the result. That keeps every decision
 * testable on the host (tests/test_usb_link_policy.c, mutation-checked by
 * scripts/test_remote_proto.py).
 *
 * Behavior is the one stage-150 shipped (usb_debug.c 1e4ba9c); see the
 * comments in usb_debug.c for why each rule exists.
 */
#ifndef USB_LINK_POLICY_H
#define USB_LINK_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/* Consecutive lost-completion stalls, with the host's port open, before the
 * soft disconnect/reconnect (issue #39). */
#define USB_LINK_HEAL_AFTER_STALLS  2

/* ── TX stall (a 1 s wait for the IN completion timed out) ── */

typedef enum {
    USB_LINK_STALL_HOST_SLOW = 0,   /* packet still VALID: host is not reading */
    USB_LINK_STALL_LATE,            /* completion landed between timeout and capture */
    USB_LINK_STALL_LOST,            /* packet gone, completion never seen (#39) */
} usb_link_stall_t;

/* ept_tx_valid: the IN endpoint's TX status is VALID at the capture.
 * tx_completed: the class's g_tx_completed at the capture. */
usb_link_stall_t usb_link_classify_stall(bool ept_tx_valid, bool tx_completed);

/* Counter of lost-completion stalls after one more (saturates at 0xFF). */
uint8_t usb_link_count_lost(uint8_t consecutive);

/* True when the self-heal (disconnect, wait, reconnect) must run now. Only a
 * host that raised DTR and still holds it gets a heal; never before the
 * scheduler runs (the heal sleeps). */
bool usb_link_heal_due(bool heal_enabled, bool dtr_seen, bool dtr,
                       uint8_t consecutive, bool scheduler_running);

/* ── TX gate (before a write and before each 64-byte chunk) ── */

typedef enum {
    USB_LINK_TX_SEND = 0,           /* go on: wait for the endpoint and send */
    USB_LINK_TX_DROP_CLOSED,        /* host closed the port (DTR 1 -> 0) */
    USB_LINK_TX_DROP_STALLED,       /* host-slow latch, packet still VALID */
    USB_LINK_TX_CLEAR_LATCH,        /* clear the latch, then send as usual */
} usb_link_tx_gate_t;

/* Once per write: SEND or DROP_CLOSED. A host that never raised DTR (CDC does
 * not require it) is still answered. */
usb_link_tx_gate_t usb_link_tx_open_gate(bool dtr_opened, bool dtr);

/* Before each chunk: SEND (not latched), DROP_STALLED, or CLEAR_LATCH (the
 * armed packet completed, or it left with no completion — then the wait runs
 * and a lost completion is counted, so the heal stays reachable). */
usb_link_tx_gate_t usb_link_tx_latch_gate(bool latched, bool tx_completed,
                                          bool ept_tx_valid);

/* ── RX watchdog (lost CDC OUT completion) ── */

/* rx_nak: OUT endpoint RX status NAK. rxtc_pending: the RXTC flag is set.
 * rx_completed: the class's g_rx_completed (the task has not consumed yet). */
bool usb_link_rx_completion_lost(bool rx_nak, bool rxtc_pending, bool rx_completed);

typedef struct {
    bool     suspect;
    uint32_t suspect_tick;
} usb_link_rx_wd_t;

/* One poll. Returns true when the lost state has held on two polls at least
 * confirm_ticks apart: the caller re-checks with the USB IRQ off and finishes
 * the ISR's work. The suspicion is cleared either way. */
bool usb_link_rx_watchdog_step(usb_link_rx_wd_t *wd, bool lost_now,
                               uint32_t now_tick, uint32_t confirm_ticks);

/* ── DTR (the host's port open/close, from SET_CONTROL_LINE_STATE) ── */

typedef struct {
    uint8_t dtr;            /* last DTR from the host */
    uint8_t dtr_seen;       /* a SET_CONTROL_LINE_STATE arrived since enumeration */
    uint8_t dtr_opened;     /* DTR was 1 at some point since enumeration */
    uint8_t banner_pending; /* a DTR rise asked for the shell banner */
} usb_link_dtr_t;

/* A SET_CONTROL_LINE_STATE with this wValue arrived (bit 0 = DTR). */
usb_link_dtr_t usb_link_dtr_on_setup(usb_link_dtr_t s, uint16_t w_value);

/* True when input arrived with DTR low after an earlier host closed the
 * port: a new host opened it without DTR, so clear dtr_opened and answer it
 * again. (A predicate, not a new state: the caller writes only dtr_opened,
 * so a DTR change the setup ISR makes meanwhile is not overwritten.) */
bool usb_link_dtr_reopen_on_input(usb_link_dtr_t s, uint16_t rx_len);

/* The DTR state after the port went away (re-enumeration or a heal).
 * banner_pending is left as it was. */
usb_link_dtr_t usb_link_dtr_forget(usb_link_dtr_t s);

/* Not configured any more: forget DTR only on a real re-enumeration. A
 * suspend keeps the host's port open and the host will not resend DTR. */
bool usb_link_forget_on_unconfigured(bool suspended);

/* DTR fell since the last poll: cancel the armed IN packet. */
bool usb_link_dtr_fell(uint8_t dtr_prev, uint8_t dtr_now);

/* First input since enumeration: send the banner? A binary host (first byte
 * is the frame sync) and a firmware transfer or its drain get none. */
bool usb_link_banner_on_first_input(bool banner_sent, uint8_t first_byte,
                                    uint8_t sync_byte, bool loader_busy);

#endif /* USB_LINK_POLICY_H */
