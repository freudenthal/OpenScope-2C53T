/*
 * Which edge fired a committed record? (trigger-modes S2 (c), 2026-09-22)
 *
 * The FPGA triggers on EITHER edge of (reg 0x08 - 28) and no register selects
 * one (EXP-53 50j, EXP-55). So "Rising" / "Falling" is a filter on the MCU:
 * classify each strobed record and let the acquisition loop drop the wrong
 * ones. The record is one continuous 1024-sample segment rotated so the seam
 * sits at the FPGA's write pointer, with the trigger 512 samples after it
 * (EXP-53/54).
 *
 * The classifier REFUSES rather than guesses. It answers UNKNOWN when the seam
 * does not stand out from the signal's own sample-to-sample steps (fast or
 * integer-period signals, where the pointer is not recoverable), when the
 * record does not cross the level near the trigger point, or when no slope
 * clears the noise. The caller commits UNKNOWN records unfiltered: on a fast
 * periodic signal both edges are in every record and the display's soft
 * trigger already aligns to the chosen one.
 *
 * Host-tested on 58 real records (EXP-55/56) and a synthetic sweep, with a
 * negative control: tests/test_trig_edge.c.
 */
#ifndef TRIG_EDGE_H
#define TRIG_EDGE_H

#include <stdint.h>

typedef enum {
    TRIG_EDGE_CLASS_UNKNOWN = 0,
    TRIG_EDGE_CLASS_RISING,
    TRIG_EDGE_CLASS_FALLING
} trig_edge_class_t;

#define TRIG_EDGE_REC_N 1024u

/* rec: the committed CH1 record (1024 samples, record units = ADC - 28).
 * crossing: the level in record units (reg 0x08 code - 28). */
trig_edge_class_t trig_edge_classify(const volatile uint8_t *rec, int crossing);

/* The seam of a committed record: the largest circular step, accepted only
 * when it is >= 2x every other step and >= 6 counts. On success *k is the
 * index of the OLDEST sample (the newer side of the step), so rotating the
 * record left by k puts it in time order with the trigger at index 512.
 * Returns 0 (and leaves *k alone) when the seam does not stand out -- for an
 * integer number of periods the wrap is continuous and there is nothing to
 * undo; for a fast signal the pointer is not recoverable. */
int trig_edge_find_seam(const volatile uint8_t *rec, uint32_t *k);

/* The seam by linear prediction, for records where the value step does not
 * stand out (fast periodic signals: the two segments can meet at similar
 * values but different phase). An order-8 predictor is fitted to the
 * circular record (autocorrelation + Levinson); a continuous signal,
 * harmonics included, is predicted well everywhere except where the
 * predictor reaches back across the seam. Accepted only when the peak
 * prediction error is >= 3x the largest error more than 8 samples away; the
 * seam is the earliest index within 8 before the peak whose error is >= 35%
 * of it. Bias measured on synthetic records: exact or 1-2 samples LATE,
 * never early. Integer periods (no seam in the circular record) refused.
 * No working buffer: the error is recomputed per pass. */
int trig_edge_find_seam_lpc(const volatile uint8_t *rec, uint32_t *k);

/* Value step first (never off in any test), else linear prediction. */
int trig_edge_find_seam_any(const volatile uint8_t *rec, uint32_t *k);

/* The seam by the crossings of the trigger level (stage-250, G-240), for a
 * STROBED record where both finders above refuse: a square with many equal
 * edges, where the seam step (if any) is no larger than an edge and every
 * edge defeats the predictor.
 *
 * The crossings of `crossing` (record units, = reg 0x08 code - 28) are found
 * around the circular record (Schmitt, 3 counts, as trig_edge_anchor). For
 * each direction the intervals between successive crossings are one period
 * each, except where the seam joins the newest and the oldest sample: there
 * one interval differs from the median (the seam lies in it), or two that
 * share a crossing: then the seam step itself crosses the level and that
 * crossing is the oldest sample. Both directions must agree. An outlier differs from the median by more than 2 samples and
 * 3 %; if those are not one contiguous run, the test is repeated at the 10 %
 * of the stage plan. Then the trigger fixes the rotation: k = c - 512 for a
 * crossing c that puts k in that arc (a k up to 12 outside is moved to the
 * arc's end: EXP-53 placed the trigger at 501..520). `prefer` = 1 rising,
 * 0 falling, -1 either: the direction tried first.
 *
 * Refused (returns 0, *k untouched): no outlier in either direction (an
 * integer number of periods: nothing to undo), outliers that are not one
 * contiguous run, fewer than 4 crossings per direction, more than 128 (fast
 * signal: the predictor's domain), or no crossing that fits.
 *
 * Any k in the arc puts the seam between the record's last and first
 * crossings, so no period inside the rotated record is cut. Where the arc is
 * a flat level on both sides, the true pointer inside it is not in the data;
 * for an either-edge trigger on a symmetric square the edge at 512 can be the
 * other direction. Uses static buffers: call from one task only. */
int trig_edge_find_seam_xing(const volatile uint8_t *rec, int crossing, int prefer,
                             uint32_t *k);

/* The hardware trigger point of a TIME-ORDERED record: the crossing of
 * `crossing` (record units, = reg 0x08 code - 28, EXP-55/56) in the chosen
 * direction nearest index 512, within +/-48 (EXP-53: 501..520). Schmitt-armed
 * with 3 counts of hysteresis, searched over 448..560. Returns the index or
 * -1. Only meaningful on a record un-rotated at its seam -- on a raw rotated
 * record index 512 is not the trigger.
 *
 * One copy, two callers: the display's hardware anchor (scope_ui.c) and the
 * mask pass/fail test (scope_mask.c). If they ever disagreed, the mask would
 * test a different alignment from the one on the glass. */
int trig_edge_anchor(const volatile uint8_t *rec, int crossing, int rising);

#endif
