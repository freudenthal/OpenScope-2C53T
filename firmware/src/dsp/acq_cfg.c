/*
 * OpenScope 2C53T - the capture configuration of a record (acq_cfg.h)
 */
#include "acq_cfg.h"

/* Is t inside [s, s + l)? Modular ticks; a t before s counts as inside (a
 * read that ended before the change it is judged against is not trusted). */
static bool acq_in_window(uint32_t t, uint32_t s, uint32_t l)
{
    const int32_t d = (int32_t)(t - s);
    return d < 0 || (uint32_t)d < l;
}

uint32_t acq_cfg_fill_ms(float fs_hz)
{
    if (!(fs_hz > 0.0f)) return ACQ_CFG_UNKNOWN_MS;
    const float ms = (float)ACQ_CFG_SAMPLES * 1000.0f / fs_hz;
    if (ms >= 60000.0f) return 60000u;
    uint32_t m = (uint32_t)ms;
    if ((float)m < ms) m++;
    return m ? m : 1u;
}

/* A field changed if it differs; UNKNOWN on one side only is a change (the
 * hardware state became known or was lost); UNKNOWN on both is not. */
static bool acq_field_changed(uint8_t a, uint8_t b)
{
    return a != b;
}

uint8_t acq_cfg_diff(const acq_cfg_t *was, const acq_cfg_t *now)
{
    uint8_t k = 0;
    if (acq_field_changed(was->rate_idx, now->rate_idx))  k |= ACQ_CHG_TIMEBASE;
    for (int c = 0; c < 2; c++) {
        if (acq_field_changed(was->vdiv[c], now->vdiv[c]) ||
            acq_field_changed(was->coupling[c], now->coupling[c]))
            k |= ACQ_CHG_FRONTEND;
    }
    if (acq_field_changed(was->trig_src, now->trig_src))   k |= ACQ_CHG_SOURCE;
    if (acq_field_changed(was->trig_code, now->trig_code)) k |= ACQ_CHG_LEVEL;
    return k;
}

uint32_t acq_cfg_window_ms(uint8_t kinds, uint32_t fill_ms, uint32_t poll_ms)
{
    if (kinds == 0u) return 0u;
    if ((kinds & ACQ_CHG_FULL) == 0u)
        return (fill_ms + 1u) / 2u + poll_ms;          /* level only: 512 samples + one poll */
    uint32_t len = fill_ms + ACQ_CFG_HOLD_MS + poll_ms;
    if (kinds & ACQ_CHG_FRONTEND) len += ACQ_CFG_SETTLE_MS;
    return len;
}

uint8_t acq_win_change(acq_win_t *w, const acq_cfg_t *now, uint8_t forced,
                       uint32_t now_ms, uint32_t extra_fill_ms, uint32_t poll_ms)
{
    uint8_t k = forced;
    uint32_t fill_old = 0u;
    if (w->primed) {
        k |= acq_cfg_diff(&w->cfg, now);
        fill_old = w->cfg.fill_ms;
    }
    w->cfg = *now;
    w->primed = true;

    if (k & ACQ_CHG_FULL) {
        uint32_t fill = now->fill_ms;
        if (fill_old > fill)      fill = fill_old;       /* the slower of old and new rate */
        if (extra_fill_ms > fill) fill = extra_fill_ms;
        const uint32_t len = acq_cfg_window_ms((uint8_t)(k & ACQ_CHG_FULL), fill, poll_ms);
        if (w->open && acq_in_window(now_ms, w->start, w->len)) {
            const int32_t left = (int32_t)(w->start + w->len - now_ms);
            if (left < 0 || (uint32_t)left < len) {      /* extend to the later end */
                w->start = now_ms;
                w->len = len;
            }
            w->kinds |= (uint8_t)(k & ACQ_CHG_FULL);
        } else {
            w->open = true;
            w->start = now_ms;
            w->len = len;
            w->kinds = (uint8_t)(k & ACQ_CHG_FULL);
        }
        w->gen++;
    } else if (k & ACQ_CHG_LEVEL) {
        const uint32_t lvl_len = acq_cfg_window_ms(ACQ_CHG_LEVEL, now->fill_ms, poll_ms);
        const bool burst = w->lvl_seen && (uint32_t)(now_ms - w->lvl_last) < lvl_len;
        const bool any_open = (w->open && acq_in_window(now_ms, w->start, w->len)) ||
                              (w->lvl_open && acq_in_window(now_ms, w->lvl_start, w->lvl_len));
        if (!burst && !any_open) {
            w->lvl_open = true;
            w->lvl_start = now_ms;
            w->lvl_len = lvl_len;
        }
    }
    if (k & ACQ_CHG_LEVEL) {
        w->lvl_seen = true;
        w->lvl_last = now_ms;
    }
    return k;
}

acq_pub_t acq_win_publish(acq_win_t *w, uint32_t gen_at_read, uint32_t read_ms, bool strobed)
{
    if (gen_at_read != w->gen)
        return ACQ_DROP_DURING;
    if (w->open) {
        if (acq_in_window(read_ms, w->start, w->len))
            return ACQ_DROP_WINDOW;
        w->open = false;
        w->kinds = 0u;
    }
    if (w->lvl_open) {
        if (acq_in_window(read_ms, w->lvl_start, w->lvl_len)) {
            if (strobed)
                return ACQ_DROP_LEVEL;
        } else {
            w->lvl_open = false;
        }
    }
    return ACQ_PUB_OK;
}

bool acq_win_full_open(const acq_win_t *w, uint32_t now_ms)
{
    return w->open && acq_in_window(now_ms, w->start, w->len);
}
