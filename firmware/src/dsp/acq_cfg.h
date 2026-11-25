/*
 * OpenScope 2C53T - the capture configuration of a record (pure, no RTOS)
 *
 * stage-240 (A-1, A-4, A-6, P-15). A record must carry only the settings that
 * were in force over its whole capture window. After a register write or a
 * relay change the records in flight can come from the old configuration:
 *   - the FPGA holds a completed record until the next read, so the first
 *     strobed read after a change can be a record from before it;
 *   - the FPGA applies a reg 0x06 write from its NEXT cycle (stage-091: the
 *     old-source record came 562-593 ms after the write at 0x12), and the
 *     cycle armed before the write can still complete;
 *   - the rolling buffer (a read that was not strobed) holds samples from
 *     before the change;
 *   - relays take a few ms to settle.
 *
 * The acquisition task keeps one acq_win_t. Every writer that changes the
 * configuration reports the configuration now in force (acq_win_change); the
 * change opens a WINDOW. The task asks acq_win_publish() for each read it is
 * about to commit. P-15 (fork 9923550a) drops every record inside one fill +
 * poll + settle of ANY change; here a level-only change has its own, smaller
 * rule, so a held level key does not stop NORMAL (A-6 b).
 *
 * Rules:
 *   - A change of timebase, range, coupling or source, a STOP -> RUN and an
 *     FPGA re-arm open a FULL window: every read inside it is dropped, strobed
 *     or not (A-1). Length = 1024 samples at the slowest rate concerned +
 *     ACQ_CFG_HOLD_MS + one poll (+ ACQ_CFG_SETTLE_MS for relays). A new full
 *     change while a window is open extends it to the later end.
 *   - A level-only change (reg 0x08) opens a LEVEL window: only strobed reads
 *     inside 512 samples + one poll are dropped. It never extends an open
 *     window, and a level change less than one level window after the last
 *     one (a held key: 150 ms repeat) opens none: a burst has one window.
 *   - A write with no change opens no window.
 *   - A read during which a full change landed (gen moved) is dropped.
 *
 * Host-tested: make test-acq-cfg.
 */
#ifndef ACQ_CFG_H
#define ACQ_CFG_H

#include <stdbool.h>
#include <stdint.h>

#define ACQ_CFG_SAMPLES      1024u  /* samples in one record */
/* After a handover the FPGA holds ~189-202 ms (EXP-54, EXP-72); the task's
 * first poll after a re-prime read comes fill + 100 ms later. 250 ms is the
 * stage-091 margin (80565ab) that covers both. */
#define ACQ_CFG_HOLD_MS      250u
#define ACQ_CFG_SETTLE_MS    10u    /* relay settle; stage-240 default, checked by the range sweep */
#define ACQ_CFG_UNKNOWN_MS   600u   /* fill time for a code with no measured rate */
#define ACQ_CFG_UNKNOWN      0xFFu  /* a field whose hardware state is not known */

/* What is in force in the hardware. fill_ms is carried, not compared. */
typedef struct {
    uint8_t  rate_idx;              /* reg 0x01 */
    uint8_t  vdiv[2];               /* relay range per channel (ACQ_CFG_UNKNOWN: not known) */
    uint8_t  coupling[2];           /* coupling_t per channel */
    uint8_t  trig_code;             /* reg 0x08 */
    uint8_t  trig_src;              /* reg 0x06: 0 CH1, 1 CH2 */
    uint32_t fill_ms;               /* 1024 samples at rate_idx (acq_cfg_fill_ms) */
} acq_cfg_t;

/* Kinds of change, as a bit mask. */
enum {
    ACQ_CHG_LEVEL    = 0x01u,       /* reg 0x08 only */
    ACQ_CHG_SOURCE   = 0x02u,       /* reg 0x06 */
    ACQ_CHG_TIMEBASE = 0x04u,       /* reg 0x01 */
    ACQ_CHG_FRONTEND = 0x08u,       /* range or coupling relays */
    ACQ_CHG_RUN      = 0x10u,       /* STOP -> RUN: the FPGA may hold a record from before */
    ACQ_CHG_REARM    = 0x20u,       /* an FPGA config sequence re-armed the engine */
};
#define ACQ_CHG_FULL (ACQ_CHG_SOURCE | ACQ_CHG_TIMEBASE | ACQ_CHG_FRONTEND | ACQ_CHG_RUN | ACQ_CHG_REARM)

typedef struct {
    acq_cfg_t cfg;                  /* the configuration last reported */
    bool      primed;               /* cfg holds a report */
    uint32_t  gen;                  /* +1 per full change */
    uint8_t   kinds;                /* kinds that opened or extended the open full window */
    bool      open;                 /* full window */
    uint32_t  start, len;
    bool      lvl_open;             /* level window */
    uint32_t  lvl_start, lvl_len;
    bool      lvl_seen;             /* a level change was reported */
    uint32_t  lvl_last;             /* tick of the last level change */
} acq_win_t;

typedef enum {
    ACQ_PUB_OK = 0,
    ACQ_DROP_DURING,                /* a full change landed during the read */
    ACQ_DROP_WINDOW,                /* inside a full window */
    ACQ_DROP_LEVEL,                 /* strobed, inside a level window */
} acq_pub_t;

/* 1024 samples at fs, in whole ms rounded up (>= 1); ACQ_CFG_UNKNOWN_MS when
 * fs is not a positive number (an unmeasured code). */
uint32_t  acq_cfg_fill_ms(float fs_hz);
/* Which kinds differ between two configurations (0 = none). A field that is
 * ACQ_CFG_UNKNOWN on either side counts as changed. */
uint8_t   acq_cfg_diff(const acq_cfg_t *was, const acq_cfg_t *now);
/* Window length for a set of kinds. fill_ms is the slowest fill concerned.
 * Level-only: half a record + one poll. Any full kind: fill + hold + poll,
 * + settle if FRONTEND is in the set. 0 for no kinds. */
uint32_t  acq_cfg_window_ms(uint8_t kinds, uint32_t fill_ms, uint32_t poll_ms);

/* Report the configuration now in force at now_ms. forced adds kinds that a
 * comparison cannot see (ACQ_CHG_RUN, ACQ_CHG_REARM, or a relay write after
 * another path moved the relays). extra_fill_ms widens the full window (the
 * fill of the last published record, for RUN; 0 for none). The first report
 * primes and opens nothing unless forced. Returns the kinds that changed. */
uint8_t   acq_win_change(acq_win_t *w, const acq_cfg_t *now, uint8_t forced,
                         uint32_t now_ms, uint32_t extra_fill_ms, uint32_t poll_ms);
/* May a read be published? gen_at_read = w->gen sampled before the read;
 * read_ms = tick when the read ended; strobed = the read strobed PC0. Closes
 * windows that have ended. */
acq_pub_t acq_win_publish(acq_win_t *w, uint32_t gen_at_read, uint32_t read_ms, bool strobed);
/* Is a full window open at now_ms (for status). */
bool      acq_win_full_open(const acq_win_t *w, uint32_t now_ms);

#endif /* ACQ_CFG_H */
