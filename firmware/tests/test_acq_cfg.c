/*
 * acq_cfg: the record-validity window after a configuration change (stage-240).
 *
 * [1] fill time: measured rates, unmeasured (rate 0) = 600 ms, limits
 * [2] diff: each field gives its kind; no change gives none; UNKNOWN
 * [3] window lengths per kind
 * [4] first report primes; forced kinds open a window on it
 * [5] timebase: unstrobed and strobed reads inside dropped (A-1); slower rate
 * [6] range / coupling: window has the relay settle (A-4)
 * [7] source: the stage-091 stale records (562-593 ms, case D ~640 ms) dropped
 * [8] a write with no change opens no window
 * [9] level only: strobed dropped, roll kept; no extension of an open window
 * [10] a held level key (150 ms) and a host slider still publish in NORMAL
 * [11] STOP -> RUN and re-arm (forced), RUN uses the published record's fill
 * [12] a change during the read drops it
 * [13] a second change extends to the later end, never shortens
 * [14] tick wrap
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "acq_cfg.h"

static int fails = 0;
static int checks = 0;
#define CHECK(cond, ...) do { checks++; if (cond) { printf("  PASS: "); } else { printf("  FAIL: "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

#define FS_0F   24979.1f        /* scope_timebase.c, measured */
#define FS_12   2494.9f
#define POLL    30u

static acq_cfg_t cfg(uint8_t rate, float fs)
{
    acq_cfg_t c;
    memset(&c, 0, sizeof c);
    c.rate_idx = rate;
    c.vdiv[0] = 7; c.vdiv[1] = 7;
    c.coupling[0] = 0; c.coupling[1] = 0;
    c.trig_code = 0x80;
    c.trig_src = 0;
    c.fill_ms = acq_cfg_fill_ms(fs);
    return c;
}

static acq_win_t fresh(const acq_cfg_t *c, uint32_t t)
{
    acq_win_t w;
    memset(&w, 0, sizeof w);
    acq_win_change(&w, c, 0u, t, 0u, POLL);
    return w;
}

static const char *pub_name(acq_pub_t p)
{
    return p == ACQ_PUB_OK ? "OK" : p == ACQ_DROP_DURING ? "DURING" :
           p == ACQ_DROP_WINDOW ? "WINDOW" : "LEVEL";
}

int main(void)
{
    printf("[1] fill time\n");
    CHECK(acq_cfg_fill_ms(FS_12) == 411u, "0x12 (2494.9 S/s): 1024 samples = 411 ms (got %u)", acq_cfg_fill_ms(FS_12));
    CHECK(acq_cfg_fill_ms(FS_0F) == 41u, "0x0F (24979.1 S/s): 41 ms (got %u)", acq_cfg_fill_ms(FS_0F));
    CHECK(acq_cfg_fill_ms(1024000.0f) == 1u, "exactly 1 ms stays 1 (got %u)", acq_cfg_fill_ms(1024000.0f));
    CHECK(acq_cfg_fill_ms(1e9f) == 1u, "very fast rate: at least 1 ms (got %u)", acq_cfg_fill_ms(1e9f));
    CHECK(acq_cfg_fill_ms(0.0f) == ACQ_CFG_UNKNOWN_MS && ACQ_CFG_UNKNOWN_MS == 600u,
          "unmeasured rate 0: 600 ms (got %u)", acq_cfg_fill_ms(0.0f));
    CHECK(acq_cfg_fill_ms(-5.0f) == 600u && acq_cfg_fill_ms(NAN) == 600u, "negative and NaN rate: 600 ms");
    CHECK(acq_cfg_fill_ms(1.0f) == 60000u, "1 S/s: capped at 60 s (got %u)", acq_cfg_fill_ms(1.0f));

    printf("[2] diff\n");
    {
        const acq_cfg_t a = cfg(0x12, FS_12);
        acq_cfg_t b = a;
        CHECK(acq_cfg_diff(&a, &b) == 0u, "same configuration: no change");
        b.fill_ms = 5u;
        CHECK(acq_cfg_diff(&a, &b) == 0u, "fill_ms is carried, not compared");
        b = a; b.rate_idx = 0x0F;     CHECK(acq_cfg_diff(&a, &b) == ACQ_CHG_TIMEBASE, "rate -> TIMEBASE");
        b = a; b.vdiv[0] = 6;         CHECK(acq_cfg_diff(&a, &b) == ACQ_CHG_FRONTEND, "CH1 range -> FRONTEND");
        b = a; b.vdiv[1] = 5;         CHECK(acq_cfg_diff(&a, &b) == ACQ_CHG_FRONTEND, "CH2 range -> FRONTEND");
        b = a; b.coupling[0] = 1;     CHECK(acq_cfg_diff(&a, &b) == ACQ_CHG_FRONTEND, "CH1 coupling -> FRONTEND");
        b = a; b.coupling[1] = 1;     CHECK(acq_cfg_diff(&a, &b) == ACQ_CHG_FRONTEND, "CH2 coupling -> FRONTEND");
        b = a; b.trig_src = 1;        CHECK(acq_cfg_diff(&a, &b) == ACQ_CHG_SOURCE, "source -> SOURCE");
        b = a; b.trig_code = 0x90;    CHECK(acq_cfg_diff(&a, &b) == ACQ_CHG_LEVEL, "code -> LEVEL");
        b = a; b.trig_code = 0x90; b.trig_src = 1;
        CHECK(acq_cfg_diff(&a, &b) == (ACQ_CHG_LEVEL | ACQ_CHG_SOURCE), "level and source in one write");
        b = a; b.vdiv[0] = ACQ_CFG_UNKNOWN;
        CHECK(acq_cfg_diff(&a, &b) == ACQ_CHG_FRONTEND, "range known -> unknown is a change");
        acq_cfg_t u = a; u.vdiv[0] = ACQ_CFG_UNKNOWN;
        CHECK(acq_cfg_diff(&u, &b) == 0u, "unknown -> unknown is not");
    }

    printf("[3] window lengths\n");
    CHECK(acq_cfg_window_ms(0u, 411u, POLL) == 0u, "no kinds: 0");
    CHECK(acq_cfg_window_ms(ACQ_CHG_LEVEL, 411u, POLL) == 236u, "level at 0x12: 512 samples + poll = 236 (got %u)",
          acq_cfg_window_ms(ACQ_CHG_LEVEL, 411u, POLL));
    CHECK(acq_cfg_window_ms(ACQ_CHG_TIMEBASE, 411u, POLL) == 691u, "timebase: fill + 250 + poll = 691");
    CHECK(acq_cfg_window_ms(ACQ_CHG_SOURCE, 411u, POLL) == 691u, "source: fill + 250 + poll = 691");
    CHECK(acq_cfg_window_ms(ACQ_CHG_FRONTEND, 411u, POLL) == 701u, "range: fill + 250 + poll + settle 10 = 701");
    CHECK(acq_cfg_window_ms(ACQ_CHG_RUN, 411u, POLL) == 691u, "RUN: as timebase");
    CHECK(acq_cfg_window_ms(ACQ_CHG_REARM, 411u, POLL) == 691u, "re-arm: as timebase");
    CHECK(acq_cfg_window_ms(ACQ_CHG_LEVEL | ACQ_CHG_SOURCE, 411u, POLL) == 691u, "level + source: the full rule");
    CHECK(acq_cfg_window_ms(ACQ_CHG_TIMEBASE, 600u, POLL) == 880u, "unmeasured rate: 600 + 250 + 30 = 880");

    printf("[4] first report\n");
    {
        acq_cfg_t c = cfg(0x12, FS_12);
        acq_win_t w;
        memset(&w, 0, sizeof w);
        CHECK(acq_win_change(&w, &c, 0u, 1000u, 0u, POLL) == 0u && !w.open && w.primed && w.gen == 0u,
              "first report primes, opens no window");
        CHECK(acq_win_publish(&w, 0u, 1001u, false) == ACQ_PUB_OK, "a read right after the prime is published");
        memset(&w, 0, sizeof w);
        CHECK(acq_win_change(&w, &c, ACQ_CHG_REARM, 1000u, 0u, POLL) == ACQ_CHG_REARM && w.open,
              "a forced kind on the first report opens a window");
    }

    printf("[5] timebase\n");
    {
        acq_cfg_t c12 = cfg(0x12, FS_12), c0f = cfg(0x0F, FS_0F);
        acq_win_t w = fresh(&c12, 0u);
        uint8_t k = acq_win_change(&w, &c0f, 0u, 10000u, 0u, POLL);
        uint32_t g = w.gen;
        CHECK(k == ACQ_CHG_TIMEBASE && w.open && g == 1u, "0x12 -> 0x0F opens a full window, gen 1");
        CHECK(w.len == 691u, "window at the slower (old) rate: 691 ms, not 321 (got %u)", w.len);
        CHECK(acq_win_publish(&w, g, 10001u, false) == ACQ_DROP_WINDOW, "A-1: the first read (not strobed) is dropped");
        CHECK(acq_win_publish(&w, g, 10440u, true) == ACQ_DROP_WINDOW, "the old-rate record (strobed, +440 ms) is dropped");
        CHECK(acq_win_publish(&w, g, 10690u, false) == ACQ_DROP_WINDOW, "+690 ms: still inside");
        CHECK(acq_win_publish(&w, g, 10691u, false) == ACQ_PUB_OK && !w.open, "+691 ms: published, window closed");
        CHECK(acq_win_publish(&w, g, 10700u, true) == ACQ_PUB_OK, "strobed after the window: published");

        w = fresh(&c0f, 0u);
        acq_win_change(&w, &c12, 0u, 500u, 0u, POLL);
        CHECK(w.len == 691u, "0x0F -> 0x12: window at the new, slower rate (691)");
    }

    printf("[6] range and coupling\n");
    {
        acq_cfg_t a = cfg(0x12, FS_12), b = a;
        acq_win_t w = fresh(&a, 0u);
        b.vdiv[0] = 5;
        CHECK(acq_win_change(&w, &b, 0u, 2000u, 0u, POLL) == ACQ_CHG_FRONTEND && w.len == 701u,
              "range change: full window 701 ms (relay settle)");
        CHECK(acq_win_publish(&w, w.gen, 2700u, true) == ACQ_DROP_WINDOW, "+700 ms strobed: dropped");
        CHECK(acq_win_publish(&w, w.gen, 2701u, true) == ACQ_PUB_OK, "+701 ms: published");
        acq_cfg_t c = b; c.coupling[1] = 1;
        CHECK(acq_win_change(&w, &c, 0u, 5000u, 0u, POLL) == ACQ_CHG_FRONTEND && w.open, "coupling change: full window");
        CHECK(acq_win_publish(&w, w.gen, 5001u, false) == ACQ_DROP_WINDOW, "roll read after a coupling change: dropped");
    }

    printf("[7] source (stage-091)\n");
    {
        acq_cfg_t a = cfg(0x12, FS_12), b = a;
        a.trig_src = 1;
        acq_win_t w = fresh(&a, 0u);
        b.trig_src = 0;
        CHECK(acq_win_change(&w, &b, 0u, 3000u, 0u, POLL) == ACQ_CHG_SOURCE && w.len == 691u,
              "CH2 -> CH1: full window 691 ms");
        int dropped = 0;
        const uint32_t stale[] = { 562u, 575u, 593u, 640u };      /* 4/4 run 1 + case D */
        for (unsigned i = 0; i < 4; i++)
            dropped += acq_win_publish(&w, w.gen, 3000u + stale[i], true) == ACQ_DROP_WINDOW;
        CHECK(dropped == 4, "old-source records at +562..593 ms and a second at +640 ms: all dropped (%d/4)", dropped);
    }

    printf("[8] a write with no change\n");
    {
        acq_cfg_t a = cfg(0x12, FS_12);
        acq_win_t w = fresh(&a, 0u);
        CHECK(acq_win_change(&w, &a, 0u, 4000u, 0u, POLL) == 0u && !w.open && !w.lvl_open && w.gen == 0u,
              "same code, source and range: no window, gen unchanged");
        CHECK(acq_win_publish(&w, 0u, 4001u, true) == ACQ_PUB_OK, "the next strobed read is published");
        CHECK(acq_win_publish(&w, 0u, 4002u, false) == ACQ_PUB_OK, "and a roll read");
    }

    printf("[9] level only\n");
    {
        acq_cfg_t a = cfg(0x12, FS_12), b = a;
        acq_win_t w = fresh(&a, 0u);
        b.trig_code = 0x90;
        CHECK(acq_win_change(&w, &b, 0u, 6000u, 0u, POLL) == ACQ_CHG_LEVEL && w.lvl_open && !w.open && w.gen == 0u,
              "level change: level window, no full window, gen unchanged");
        CHECK(w.lvl_len == 236u, "level window 236 ms at 0x12 (got %u)", w.lvl_len);
        CHECK(acq_win_publish(&w, 0u, 6100u, false) == ACQ_PUB_OK, "roll read inside: published (same samples)");
        CHECK(acq_win_publish(&w, 0u, 6100u, true) == ACQ_DROP_LEVEL, "strobed inside: dropped");
        CHECK(acq_win_publish(&w, 0u, 6236u, true) == ACQ_PUB_OK && !w.lvl_open, "strobed at +236: published");

        /* no extension of an open full window */
        acq_cfg_t c = b; c.rate_idx = 0x11;
        acq_win_change(&w, &c, 0u, 10000u, 0u, POLL);
        const uint32_t end = w.start + w.len;
        acq_cfg_t d = c; d.trig_code = 0xA0;
        acq_win_change(&w, &d, 0u, 10600u, 0u, POLL);
        CHECK(w.start + w.len == end && !w.lvl_open, "level change inside a full window: no extension, no level window");
        CHECK(acq_win_publish(&w, w.gen, end, true) == ACQ_PUB_OK, "strobed at the full window's end: published");

        /* no extension of an open level window */
        acq_win_t v = fresh(&a, 0u);
        acq_cfg_t e1 = a, e2 = a;
        e1.trig_code = 0x90; e2.trig_code = 0x70;
        acq_win_change(&v, &e1, 0u, 20000u, 0u, POLL);
        acq_win_change(&v, &e2, 0u, 20200u, 0u, POLL);
        CHECK(v.lvl_start == 20000u && v.lvl_len == 236u, "second level change at +200: window not moved");
        CHECK(acq_win_publish(&v, 0u, 20236u, true) == ACQ_PUB_OK, "strobed at +236 of the first: published");
        /* a lone level change long after the burst opens a new window */
        acq_win_change(&v, &e1, 0u, 30000u, 0u, POLL);
        CHECK(v.lvl_open && v.lvl_start == 30000u, "a level change 10 s later opens a new level window");
    }

    printf("[10] held level key / host slider in NORMAL\n");
    {
        /* 0x12, writes every 150 ms for 10 s; the FPGA hands over a record
         * every ~600 ms (fill 411 + hold ~190). Count records per 2 s. */
        const char *names[3] = { "two levels alternating", "a held key stepping", "the same level re-sent" };
        for (int run = 0; run < 3; run++) {
            acq_cfg_t a = cfg(0x12, FS_12);
            acq_win_t w = fresh(&a, 0u);
            int bins[5] = { 0 }, total = 0, records = 0;
            uint32_t next_rec = 100000u + 300u;
            for (uint32_t t = 100000u; t < 110000u; t++) {
                if ((t - 100000u) % 150u == 0u) {
                    acq_cfg_t n = a;
                    uint32_t i = (t - 100000u) / 150u;
                    if (run == 0) n.trig_code = (i & 1u) ? 0x70 : 0x90;
                    if (run == 1) n.trig_code = (uint8_t)(0x60 + (i % 64u));
                    acq_win_change(&w, &n, 0u, t, 0u, POLL);
                }
                if (t == next_rec) {
                    records++;
                    if (acq_win_publish(&w, w.gen, t, true) == ACQ_PUB_OK) {
                        bins[(t - 100000u) / 2000u]++;
                        total++;
                    }
                    next_rec += 600u;
                }
            }
            int min = bins[0];
            for (int i = 1; i < 5; i++) if (bins[i] < min) min = bins[i];
            CHECK(min >= 1, "%s: at least 1 record in every 2 s (min %d, %d/%d published)",
                  names[run], min, total, records);
            if (run == 2)
                CHECK(total == records, "%s: every record published (%d/%d)", names[run], total, records);
            else
                CHECK(total * 2 >= records, "%s: at least half the records published (%d/%d)",
                      names[run], total, records);
        }
    }

    printf("[11] STOP -> RUN and re-arm\n");
    {
        acq_cfg_t c0f = cfg(0x0F, FS_0F);
        acq_win_t w = fresh(&c0f, 0u);
        CHECK(acq_win_change(&w, &c0f, ACQ_CHG_RUN, 1000u, acq_cfg_fill_ms(FS_12), POLL) == ACQ_CHG_RUN &&
              w.open && w.len == 691u, "RUN at 0x0F after a 0x12 record: window at the record's fill (691)");
        CHECK(acq_win_publish(&w, w.gen, 1001u, true) == ACQ_DROP_WINDOW, "the record held across STOP: dropped");
        CHECK(acq_win_publish(&w, w.gen, 1001u, false) == ACQ_DROP_WINDOW, "a roll read after RUN: dropped");
        CHECK((w.kinds & ACQ_CHG_RUN) != 0u, "the window records that RUN opened it");
        w = fresh(&c0f, 0u);
        acq_win_change(&w, &c0f, ACQ_CHG_RUN, 1000u, 0u, POLL);
        CHECK(w.len == 321u, "RUN with no extra fill at 0x0F: 41 + 250 + 30 = 321 (got %u)", w.len);
        acq_win_change(&w, &c0f, ACQ_CHG_REARM, 5000u, 0u, POLL);
        CHECK(w.open && w.kinds == ACQ_CHG_REARM && w.len == 321u, "re-arm: full window");
    }

    printf("[12] a change during the read\n");
    {
        acq_cfg_t a = cfg(0x12, FS_12), b = a;
        acq_win_t w = fresh(&a, 0u);
        uint32_t g = w.gen;
        b.vdiv[1] = 3;
        acq_win_change(&w, &b, 0u, 50u, 0u, POLL);
        acq_pub_t p = acq_win_publish(&w, g, 5000u, true);
        CHECK(p == ACQ_DROP_DURING, "gen moved during the read: dropped even after the window (%s)", pub_name(p));
        acq_cfg_t c = b; c.trig_code = 0x91;
        g = w.gen;
        acq_win_change(&w, &c, 0u, 6000u, 0u, POLL);
        CHECK(w.gen == g, "a level change does not move gen");
        CHECK(acq_win_publish(&w, g, 6001u, false) == ACQ_PUB_OK, "so a roll read across it is kept");
    }

    printf("[13] a second change extends, never shortens\n");
    {
        acq_cfg_t c0f = cfg(0x0F, FS_0F), c12 = cfg(0x12, FS_12);
        acq_win_t w = fresh(&c0f, 0u);
        acq_win_change(&w, &c12, 0u, 1000u, 0u, POLL);          /* 691, end 1691 */
        acq_cfg_t r = c12; r.vdiv[0] = 4;
        acq_win_change(&w, &r, 0u, 1100u, 0u, POLL);            /* 701 from 1100: end 1801 */
        CHECK(w.start + w.len == 1801u && (w.kinds & (ACQ_CHG_TIMEBASE | ACQ_CHG_FRONTEND)) ==
              (ACQ_CHG_TIMEBASE | ACQ_CHG_FRONTEND), "longer second window: end moves to 1801, kinds kept");
        acq_win_t v = fresh(&c0f, 0u);
        acq_win_change(&v, &c12, 0u, 1000u, 0u, POLL);          /* end 1691 */
        acq_cfg_t f = c12; f.rate_idx = 0x0F; f.fill_ms = c0f.fill_ms;
        acq_win_change(&v, &f, 0u, 1050u, 0u, POLL);            /* slower of 411 and 41: 691 from 1050 */
        CHECK(v.start + v.len == 1741u, "back to 0x0F 50 ms later: the old 0x12 fill still counts (end 1741)");
        acq_win_t x = fresh(&c12, 0u);
        acq_win_change(&x, &c0f, 0u, 0u, 0u, POLL);             /* end 691 */
        acq_cfg_t y = c0f; y.vdiv[0] = 2;
        acq_win_change(&x, &y, 0u, 100u, 0u, POLL);             /* 331 from 100 = 431 < 691 */
        CHECK(x.start + x.len == 691u, "shorter second window: end stays 691");
        CHECK(acq_win_publish(&x, x.gen, 600u, true) == ACQ_DROP_WINDOW, "read at 600: still dropped");
    }

    printf("[14] tick wrap\n");
    {
        acq_cfg_t a = cfg(0x12, FS_12), b = a;
        const uint32_t t0 = 0xFFFFFF00u;
        acq_win_t w = fresh(&a, t0 - 10u);
        b.rate_idx = 0x11;
        b.fill_ms = acq_cfg_fill_ms(FS_12);
        acq_win_change(&w, &b, 0u, t0, 0u, POLL);
        CHECK(acq_win_publish(&w, w.gen, t0 + 100u, true) == ACQ_DROP_WINDOW, "wrap: +100 ms dropped");
        CHECK(acq_win_publish(&w, w.gen, t0 + 691u, true) == ACQ_PUB_OK, "wrap: +691 ms published");
        CHECK(acq_win_publish(&w, w.gen, t0 - 5u, true) == ACQ_PUB_OK, "window closed: an earlier tick is not re-judged");
        acq_win_change(&w, &a, 0u, t0 + 1000u, 0u, POLL);
        CHECK(acq_win_publish(&w, w.gen, t0 + 999u, true) == ACQ_DROP_WINDOW,
              "a read that ended before the change it is judged against: dropped");
    }

    printf("\n%d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
