/*
 * OpenScope 2C53T - USB CDC link decisions (stage-155). See usb_link_policy.h.
 * Pure: no vendor, USB-stack or FreeRTOS header.
 */
#include "usb_link_policy.h"

usb_link_stall_t usb_link_classify_stall(bool ept_tx_valid, bool tx_completed)
{
    if (ept_tx_valid)
        return USB_LINK_STALL_HOST_SLOW;
    if (tx_completed)
        return USB_LINK_STALL_LATE;
    return USB_LINK_STALL_LOST;
}

uint8_t usb_link_count_lost(uint8_t consecutive)
{
    return (consecutive < 0xFFu) ? (uint8_t)(consecutive + 1u) : consecutive;
}

bool usb_link_heal_due(bool heal_enabled, bool dtr_seen, bool dtr,
                       uint8_t consecutive, bool scheduler_running)
{
    return heal_enabled && dtr_seen && dtr &&
           consecutive >= USB_LINK_HEAL_AFTER_STALLS &&
           scheduler_running;
}

usb_link_tx_gate_t usb_link_tx_open_gate(bool dtr_opened, bool dtr)
{
    if (dtr_opened && !dtr)
        return USB_LINK_TX_DROP_CLOSED;
    return USB_LINK_TX_SEND;
}

usb_link_tx_gate_t usb_link_tx_latch_gate(bool latched, bool tx_completed,
                                          bool ept_tx_valid)
{
    if (!latched)
        return USB_LINK_TX_SEND;
    if (tx_completed)
        return USB_LINK_TX_CLEAR_LATCH;
    if (ept_tx_valid)
        return USB_LINK_TX_DROP_STALLED;
    return USB_LINK_TX_CLEAR_LATCH;
}

bool usb_link_rx_completion_lost(bool rx_nak, bool rxtc_pending, bool rx_completed)
{
    return rx_nak && !rxtc_pending && !rx_completed;
}

bool usb_link_rx_watchdog_step(usb_link_rx_wd_t *wd, bool lost_now,
                               uint32_t now_tick, uint32_t confirm_ticks)
{
    if (!lost_now) {
        wd->suspect = false;
        return false;
    }
    if (!wd->suspect) {
        wd->suspect = true;
        wd->suspect_tick = now_tick;
        return false;
    }
    if ((uint32_t)(now_tick - wd->suspect_tick) < confirm_ticks)
        return false;
    wd->suspect = false;
    return true;
}

usb_link_dtr_t usb_link_dtr_on_setup(usb_link_dtr_t s, uint16_t w_value)
{
    uint8_t dtr = (uint8_t)(w_value & 0x0001u);
    if (dtr && !s.dtr)
        s.banner_pending = 1;       /* DTR 0 -> 1: a port open */
    s.dtr = dtr;
    s.dtr_seen = 1;
    if (dtr)
        s.dtr_opened = 1;
    return s;
}

bool usb_link_dtr_reopen_on_input(usb_link_dtr_t s, uint16_t rx_len)
{
    return rx_len > 0 && !s.dtr && s.dtr_opened;
}

usb_link_dtr_t usb_link_dtr_forget(usb_link_dtr_t s)
{
    s.dtr = 0;                      /* the next open is a DTR rise again */
    s.dtr_seen = 0;
    s.dtr_opened = 0;
    return s;
}

bool usb_link_forget_on_unconfigured(bool suspended)
{
    return !suspended;
}

bool usb_link_dtr_fell(uint8_t dtr_prev, uint8_t dtr_now)
{
    return dtr_prev && !dtr_now;
}

bool usb_link_banner_on_first_input(bool banner_sent, uint8_t first_byte,
                                    uint8_t sync_byte, bool loader_busy)
{
    return !banner_sent && first_byte != sync_byte && !loader_busy;
}
