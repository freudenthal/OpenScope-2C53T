/*
 * USB CDC link decisions — host tests (stage-155).
 *
 * One block per decision in src/drivers/usb_link_policy.c: TX stall
 * classification, heal, the two TX gates, the RX watchdog, and the DTR
 * rules. Each block pins the stage-150 behavior (usb_debug.c 1e4ba9c) and
 * scripts/test_remote_proto.py removes each guard in turn and requires this
 * suite to go red.
 *
 * Build:  make test-usb-link-policy
 */

#include <stdio.h>
#include <string.h>
#include "usb_link_policy.h"

static int failures = 0, checks = 0;
#define CHECK(cond, msg) do { checks++; if (!(cond)) { failures++; \
    printf("  FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); } } while (0)

static void test_classify_stall(void)
{
    /* All four input pairs. VALID wins over the flag: a packet still armed
     * means the host is not reading, whatever the flag says. */
    CHECK(usb_link_classify_stall(true, false) == USB_LINK_STALL_HOST_SLOW,
          "packet still VALID, no completion: host slow");
    CHECK(usb_link_classify_stall(true, true) == USB_LINK_STALL_HOST_SLOW,
          "packet VALID and flag set: still host slow (endpoint state first)");
    CHECK(usb_link_classify_stall(false, true) == USB_LINK_STALL_LATE,
          "endpoint free, flag set: the completion landed late");
    CHECK(usb_link_classify_stall(false, false) == USB_LINK_STALL_LOST,
          "endpoint NAK, flag 0: lost completion (#39)");
}

static void test_count_lost(void)
{
    CHECK(usb_link_count_lost(0) == 1, "first lost stall counts 1");
    CHECK(usb_link_count_lost(1) == 2, "second counts 2");
    CHECK(usb_link_count_lost(0xFE) == 0xFF, "counts up to 0xFF");
    CHECK(usb_link_count_lost(0xFF) == 0xFF, "saturates at 0xFF, never wraps to 0");
}

static void test_heal_due(void)
{
    const uint8_t n = USB_LINK_HEAL_AFTER_STALLS;
    CHECK(USB_LINK_HEAL_AFTER_STALLS == 2, "heal after 2 consecutive lost stalls (stage-150 value)");
    CHECK(usb_link_heal_due(true, true, true, n, true), "all conditions: heal");
    CHECK(usb_link_heal_due(true, true, true, 0xFF, true), "more stalls than needed: heal");
    CHECK(!usb_link_heal_due(true, true, true, (uint8_t)(n - 1), true), "one stall short: no heal");
    CHECK(!usb_link_heal_due(false, true, true, n, true), "heal disabled: no heal");
    CHECK(!usb_link_heal_due(true, false, true, n, true), "no DTR seen since enumeration: no heal");
    CHECK(!usb_link_heal_due(true, true, false, n, true), "port closed (DTR low): no heal");
    CHECK(!usb_link_heal_due(true, true, true, n, false), "scheduler not running: no heal (it sleeps)");
}

static void test_tx_open_gate(void)
{
    CHECK(usb_link_tx_open_gate(false, false) == USB_LINK_TX_SEND,
          "a host that never raised DTR is answered");
    CHECK(usb_link_tx_open_gate(true, true) == USB_LINK_TX_SEND, "port open: send");
    CHECK(usb_link_tx_open_gate(true, false) == USB_LINK_TX_DROP_CLOSED,
          "DTR raised then dropped: port closed, drop");
    CHECK(usb_link_tx_open_gate(false, true) == USB_LINK_TX_SEND,
          "DTR high (opened flag not yet set): send");
}

static void test_tx_latch_gate(void)
{
    /* Not latched: send whatever the endpoint says (the wait handles it). */
    CHECK(usb_link_tx_latch_gate(false, false, true) == USB_LINK_TX_SEND,
          "not latched, packet armed: send (wait as usual)");
    CHECK(usb_link_tx_latch_gate(false, false, false) == USB_LINK_TX_SEND,
          "not latched, endpoint free: send");
    CHECK(usb_link_tx_latch_gate(false, true, false) == USB_LINK_TX_SEND,
          "not latched, flag set: send");
    /* Latched. */
    CHECK(usb_link_tx_latch_gate(true, false, true) == USB_LINK_TX_DROP_STALLED,
          "latched, packet still VALID: drop at once (no 1 s wait per chunk)");
    CHECK(usb_link_tx_latch_gate(true, true, false) == USB_LINK_TX_CLEAR_LATCH,
          "latched, packet completed: clear the latch");
    CHECK(usb_link_tx_latch_gate(true, true, true) == USB_LINK_TX_CLEAR_LATCH,
          "latched, flag set (endpoint re-armed): clear, the flag wins");
    CHECK(usb_link_tx_latch_gate(true, false, false) == USB_LINK_TX_CLEAR_LATCH,
          "latched, packet left with no completion: clear so the lost stall is counted");
}

static void test_rx_completion_lost(void)
{
    for (int i = 0; i < 8; i++) {
        bool nak = i & 1, tc = (i >> 1) & 1, done = (i >> 2) & 1;
        bool want = nak && !tc && !done;
        char msg[96];
        snprintf(msg, sizeof msg, "lost(nak=%d, rxtc=%d, rx_completed=%d) = %d", nak, tc, done, want);
        CHECK(usb_link_rx_completion_lost(nak, tc, done) == want, msg);
    }
}

static void test_rx_watchdog(void)
{
    usb_link_rx_wd_t wd;
    const uint32_t c = 10;

    memset(&wd, 0, sizeof wd);
    CHECK(!usb_link_rx_watchdog_step(&wd, false, 0, c), "healthy: no action");
    CHECK(!wd.suspect, "healthy: no suspicion");

    CHECK(!usb_link_rx_watchdog_step(&wd, true, 100, c), "first sight of the lost state: only suspect");
    CHECK(wd.suspect && wd.suspect_tick == 100, "suspicion records its tick");
    CHECK(!usb_link_rx_watchdog_step(&wd, true, 100 + c - 1, c), "still inside the confirm window: wait");
    CHECK(wd.suspect && wd.suspect_tick == 100, "a second poll does not restart the window");
    CHECK(usb_link_rx_watchdog_step(&wd, true, 100 + c, c), "held for confirm_ticks: recover");
    CHECK(!wd.suspect, "recovery clears the suspicion");

    /* A healthy poll in between resets the window. */
    memset(&wd, 0, sizeof wd);
    (void)usb_link_rx_watchdog_step(&wd, true, 200, c);
    (void)usb_link_rx_watchdog_step(&wd, false, 205, c);
    CHECK(!wd.suspect, "the state cleared: suspicion dropped");
    CHECK(!usb_link_rx_watchdog_step(&wd, true, 212, c), "seen again: a new first sight, no recovery");
    CHECK(wd.suspect_tick == 212, "new window starts at the new sight");

    /* 32-bit tick wrap inside the window. */
    memset(&wd, 0, sizeof wd);
    (void)usb_link_rx_watchdog_step(&wd, true, 0xFFFFFFFCu, c);
    CHECK(!usb_link_rx_watchdog_step(&wd, true, 0xFFFFFFFEu, c), "2 ticks, before the wrap: still waiting");
    CHECK(!usb_link_rx_watchdog_step(&wd, true, 0x00000004u, c), "8 ticks across the wrap: still waiting");
    CHECK(usb_link_rx_watchdog_step(&wd, true, 0x00000006u, c), "10 ticks across the wrap: recover");
}

static usb_link_dtr_t dtr_state(uint8_t dtr, uint8_t seen, uint8_t opened, uint8_t banner)
{
    usb_link_dtr_t s = { dtr, seen, opened, banner };
    return s;
}

static void test_dtr_on_setup(void)
{
    usb_link_dtr_t s;

    s = usb_link_dtr_on_setup(dtr_state(0, 0, 0, 0), 0x0001);
    CHECK(s.dtr == 1 && s.dtr_seen == 1 && s.dtr_opened == 1, "DTR rise: dtr, seen, opened set");
    CHECK(s.banner_pending == 1, "DTR rise asks for the banner");

    s = usb_link_dtr_on_setup(dtr_state(1, 1, 1, 0), 0x0003);
    CHECK(s.dtr == 1 && s.banner_pending == 0, "DTR stays high (RTS too): no second banner");

    s = usb_link_dtr_on_setup(dtr_state(1, 1, 1, 0), 0x0000);
    CHECK(s.dtr == 0 && s.dtr_seen == 1, "DTR fall: dtr 0, seen stays");
    CHECK(s.dtr_opened == 1, "DTR fall keeps opened (that is what marks the port closed)");
    CHECK(s.banner_pending == 0, "DTR fall: no banner");

    s = usb_link_dtr_on_setup(dtr_state(0, 0, 0, 0), 0x0002);
    CHECK(s.dtr == 0 && s.dtr_seen == 1 && s.dtr_opened == 0, "RTS only: DTR seen low, not opened");
    CHECK(s.banner_pending == 0, "RTS only: no banner");

    s = usb_link_dtr_on_setup(dtr_state(0, 1, 1, 1), 0x0000);
    CHECK(s.banner_pending == 1, "a pending banner is not cancelled by a later setup");
}

static void test_dtr_reopen_on_input(void)
{
    CHECK(usb_link_dtr_reopen_on_input(dtr_state(0, 1, 1, 0), 5),
          "input with DTR low after a close: a new host, answer again");
    CHECK(!usb_link_dtr_reopen_on_input(dtr_state(0, 1, 1, 0), 0), "no input: nothing changes");
    CHECK(!usb_link_dtr_reopen_on_input(dtr_state(1, 1, 1, 0), 5),
          "input with DTR high: the port is open, keep opened");
    CHECK(!usb_link_dtr_reopen_on_input(dtr_state(0, 1, 0, 0), 5),
          "input from a host that never raised DTR: already answered, nothing to clear");
}

static void test_dtr_forget(void)
{
    usb_link_dtr_t s = usb_link_dtr_forget(dtr_state(1, 1, 1, 1));
    CHECK(s.dtr == 0 && s.dtr_seen == 0 && s.dtr_opened == 0, "forget clears dtr, seen, opened");
    CHECK(s.banner_pending == 1, "forget leaves a pending banner alone (stage-150 did not touch it)");
    s = usb_link_dtr_forget(dtr_state(1, 1, 1, 0));
    CHECK(usb_link_dtr_on_setup(s, 1).banner_pending == 1, "after a forget the next open is a rise again");

    CHECK(usb_link_forget_on_unconfigured(false), "not configured and not suspended: forget");
    CHECK(!usb_link_forget_on_unconfigured(true), "suspended: keep DTR (host port stays open)");
}

static void test_dtr_fell(void)
{
    CHECK(usb_link_dtr_fell(1, 0), "1 -> 0: fell, cancel the armed packet");
    CHECK(!usb_link_dtr_fell(0, 0), "0 -> 0: no");
    CHECK(!usb_link_dtr_fell(0, 1), "0 -> 1: no");
    CHECK(!usb_link_dtr_fell(1, 1), "1 -> 1: no");
}

static void test_banner_on_first_input(void)
{
    CHECK(usb_link_banner_on_first_input(false, 'h', 0xAA, false), "terminal's first key: banner");
    CHECK(!usb_link_banner_on_first_input(true, 'h', 0xAA, false), "banner already sent: no second");
    CHECK(!usb_link_banner_on_first_input(false, 0xAA, 0xAA, false), "binary host (sync byte first): no banner");
    CHECK(!usb_link_banner_on_first_input(false, 'h', 0xAA, true), "loader busy or draining: no banner");
}

int main(void)
{
    printf("test_usb_link_policy\n");
    test_classify_stall();
    test_count_lost();
    test_heal_due();
    test_tx_open_gate();
    test_tx_latch_gate();
    test_rx_completion_lost();
    test_rx_watchdog();
    test_dtr_on_setup();
    test_dtr_reopen_on_input();
    test_dtr_forget();
    test_dtr_fell();
    test_banner_on_first_input();
    printf("%d/%d checks passed\n", checks - failures, checks);
    return failures ? 1 : 0;
}
