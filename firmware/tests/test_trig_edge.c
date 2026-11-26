/*
 * trig_edge_classify(): the MCU-side trigger edge filter (EXP-55 follow-up).
 *
 * [1] real records (EXP-55/56, unit #1): never wrong, and classifies most.
 * [2] negative control: the same records with the level moved off the signal
 *     must come back UNKNOWN -- a classifier that ignores the level would pass
 *     [1] and still be wrong about what triggered.
 * [3] synthetic sweep, sines and squares from 0.2 to 31 periods per record,
 *     both edges, random rotation: never wrong; integer periods (seam
 *     invisible by construction) must be refused, not guessed.
 * [7] level-crossing seam finder (stage-250, G-240): squares with 1-sample
 *     edges (about 16 and 40 edges per record), rotated by random k, with
 *     and without 1 LSB noise: no period cut inside the rotated record and a
 *     crossing within 12 of index 512; integer periods and a level off the
 *     signal are refused.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "trig_edge.h"
#include "fixtures/trig_edge_records.h"

static int fails = 0;
#define CHECK(cond, ...) do { if (cond) { printf("  PASS: "); } else { printf("  FAIL: "); fails++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

static int label_of(trig_edge_class_t c)
{
    return c == TRIG_EDGE_CLASS_RISING ? 1 : c == TRIG_EDGE_CLASS_FALLING ? -1 : 0;
}

static uint32_t lcg = 12345u;
static double urand(void) { lcg = lcg * 1664525u + 1013904223u; return (lcg >> 8) / 16777216.0; }

static uint32_t last_rot;
static void synth(uint8_t *rec, double periods, int want, int square, double amp)
{
    const double pi = 3.14159265358979;
    double per = 1024.0 / periods;
    double ph = -2.0 * pi * 512.0 / per + (want > 0 ? 0.0 : pi) + (urand() - 0.5) * 0.1;
    uint32_t rot = (uint32_t)(urand() * 1024.0) & 1023u;
    last_rot = rot;
    for (int n = 0; n < 1024; n++) {
        double b = sin(2.0 * pi * n / per + ph);
        if (square) b = tanh(8.0 * b);
        double v = 128.0 + amp * b + (urand() - 0.5) * 4.0;
        if (v < 0) v = 0;
        if (v > 227) v = 227;
        rec[(n + rot) & 1023u] = (uint8_t)(v + 0.5);
    }
}

/* [7] A sharp square, time-ordered, trigger crossing at index 512 + dj
 * (EXP-53: 501..520), then rotated by a random k. 1-sample edges: every edge
 * is a full step, so the value step and the predictor have nothing to find. */
static void synth_sq(uint8_t *rec, double per, int rising, int noise, int amp, int dj)
{
    uint32_t rot = (uint32_t)(urand() * 1024.0) & 1023u;
    last_rot = rot;
    for (int n = 0; n < 1024; n++) {
        /* phase so the crossing of 128 sits at sample 512 + dj */
        double ph = (n - 512 - dj) / per;
        ph -= floor(ph);
        int hi = rising ? (ph < 0.5) : (ph >= 0.5);
        int v = 128 + (hi ? amp : -amp);
        if (noise) v += (int)(urand() * 3.0) - 1;          /* -1, 0, +1 */
        rec[(n + rot) & 1023u] = (uint8_t)v;
    }
}

/* The rotated record r[t] = rec[k + t]: same-direction crossing intervals
 * inside it (no wrap), and the crossing nearest 512. cut = intervals below
 * 90 % of the median (the stage-250 cut-period rule). */
static void judge(const uint8_t *rec, uint32_t k, int *cut, int *trig_ok, int *periods)
{
    int xs[2][256], nx[2] = { 0, 0 }, best = 9999;
    int prev = rec[k & 1023u];
    for (int t = 1; t < 1024; t++) {
        int s = rec[(k + (uint32_t)t) & 1023u];
        int dir = (prev < 128 && s >= 128) ? 0 : (prev > 128 && s <= 128) ? 1 : -1;
        if (dir >= 0 && nx[dir] < 256) xs[dir][nx[dir]++] = t;
        if (dir >= 0 && abs(t - 512) < abs(best - 512)) best = t;
        prev = s;
    }
    *cut = 0; *periods = 0;
    for (int d = 0; d < 2; d++) {
        int iv[256], n = 0;
        for (int i = 1; i < nx[d]; i++) iv[n++] = xs[d][i] - xs[d][i - 1];
        if (n == 0) continue;
        int srt[256];
        for (int i = 0; i < n; i++) {
            int j = i; while (j > 0 && srt[j - 1] > iv[i]) { srt[j] = srt[j - 1]; j--; } srt[j] = iv[i];
        }
        int med = srt[n / 2];
        for (int i = 0; i < n; i++) if (iv[i] * 10 < med * 9) (*cut)++;
        *periods += n;
    }
    *trig_ok = abs(best - 512) <= 12;
}

int main(void)
{
    printf("[1] real records, unit #1 (EXP-55/56)\n");
    int ok = 0, wrong = 0, unk = 0;
    for (unsigned i = 0; i < EDGE_FIXTURE_N; i++) {
        int g = label_of(trig_edge_classify(EDGE_FIXTURES[i].rec, EDGE_FIXTURES[i].crossing));
        if (g == 0) unk++;
        else if (g == EDGE_FIXTURES[i].label) ok++;
        else { wrong++; printf("    wrong: %s\n", EDGE_FIXTURES[i].name); }
    }
    CHECK(wrong == 0, "%u records: %d correct, %d wrong, %d unknown", (unsigned)EDGE_FIXTURE_N, ok, wrong, unk);
    CHECK(ok * 10 >= (int)EDGE_FIXTURE_N * 8, "classifies >= 80%% of real records (%d/%u)", ok, (unsigned)EDGE_FIXTURE_N);

    printf("[2] negative control: level moved off the signal\n");
    int classified = 0;
    for (unsigned i = 0; i < EDGE_FIXTURE_N; i++)
        if (trig_edge_classify(EDGE_FIXTURES[i].rec, EDGE_FIXTURES[i].crossing + 150) != TRIG_EDGE_CLASS_UNKNOWN)
            classified++;
    CHECK(classified == 0, "no record classified at a level it never crosses (%d)", classified);

    printf("[3] synthetic sweep\n");
    static const double frac[] = { 0.2, 0.3, 0.4, 0.7, 1.3, 1.8, 2.7, 4.4, 6.6, 9.1, 15.6, 31.3 };
    int s_ok = 0, s_wrong = 0, s_unk = 0;
    uint8_t rec[1024];
    for (int sq = 0; sq < 2; sq++)
        for (unsigned f = 0; f < sizeof frac / sizeof frac[0]; f++)
            for (int i = 0; i < 100; i++) {
                int want = (i & 1) ? -1 : 1;
                synth(rec, frac[f], want, sq, 25.0 + urand() * 35.0);
                int g = label_of(trig_edge_classify(rec, 128));
                if (g == 0) s_unk++; else if (g == want) s_ok++; else s_wrong++;
            }
    CHECK(s_wrong == 0, "2400 fractional-period records: %d correct, %d wrong, %d unknown", s_ok, s_wrong, s_unk);
    CHECK(s_ok >= 1800, "classifies >= 75%% of them (%d)", s_ok);
    int int_classified = 0;
    for (int i = 0; i < 200; i++) {
        synth(rec, (double)(1 + (i % 8)), (i & 1) ? -1 : 1, i & 2, 40.0);
        if (trig_edge_classify(rec, 128) != TRIG_EDGE_CLASS_UNKNOWN) int_classified++;
    }
    CHECK(int_classified == 0, "integer periods (seam invisible) refused, not guessed (%d classified)", int_classified);

    printf("[4] seam finder recovers the rotation (un-rotation, dev plan 2.3)\n");
    int found = 0, exact = 0, off = 0;
    for (int sq = 0; sq < 2; sq++)
        for (unsigned f = 0; f < sizeof frac / sizeof frac[0]; f++)
            for (int i = 0; i < 50; i++) {
                synth(rec, frac[f], (i & 1) ? -1 : 1, sq, 25.0 + urand() * 35.0);
                uint32_t k = 9999u;
                if (!trig_edge_find_seam(rec, &k)) continue;
                found++;
                /* synth() writes sample n at (n + rot): the oldest sample
                 * (n = 0) sits at index rot. */
                if (k == last_rot) exact++; else { off++; if (off <= 3) printf("    k %u vs rot %u (periods %.1f)\n", (unsigned)k, (unsigned)last_rot, frac[f]); }
            }
    CHECK(off == 0, "every confident seam is the true rotation (%d found, %d exact, %d off)", found, exact, off);
    CHECK(found >= 900, "finds the seam on >= 75%% of 1200 fractional-period records (%d)", found);

    printf("[5] seam by linear prediction (fast periodic records)\n");
    static const double fast[] = { 0.3, 1.3, 4.4, 9.1, 16.5, 31.3, 41.3, 60.6, 82.6 };
    int l_exact = 0, l_late = 0, l_early = 0, l_off = 0, l_ref = 0, v_found = 0, any_found = 0;
    int v_refused = 0, rescued = 0, rescued_wrong = 0;
    for (int sq = 0; sq < 2; sq++)
        for (unsigned f = 0; f < sizeof fast / sizeof fast[0]; f++)
            for (int i = 0; i < 100; i++) {
                synth(rec, fast[f], (i & 1) ? -1 : 1, sq, 25.0 + urand() * 35.0);
                uint32_t k = 9999u;
                uint32_t kv;
                int vf = trig_edge_find_seam(rec, &kv);
                if (vf) v_found++;
                if (trig_edge_find_seam_any(rec, &k)) any_found++;
                if (!vf) {
                    uint32_t kl;
                    v_refused++;
                    if (trig_edge_find_seam_lpc(rec, &kl)) {
                        int dd = (int)((kl - last_rot + 1536u) & 1023u) - 512;
                        if (dd >= 0 && dd <= 2) rescued++; else rescued_wrong++;
                    }
                }
                if (!trig_edge_find_seam_lpc(rec, &k)) { l_ref++; continue; }
                int d = (int)((k - last_rot + 1536u) & 1023u) - 512;
                if (d == 0) l_exact++;
                else if (d > 0 && d <= 2) l_late++;
                else if (d < 0 && d >= -2) l_early++;
                else { l_off++; if (l_off <= 3) printf("    off by %d (periods %.1f, %s)\n", d, fast[f], sq ? "square" : "sine"); }
            }
    CHECK(l_off == 0 && l_early == 0, "1800 records: %d exact, %d late by 1-2, %d early, %d off, %d refused",
          l_exact, l_late, l_early, l_off, l_ref);
    CHECK(l_exact >= 9 * (l_exact + l_late) / 10, "at least 90%% of answers exact (%d of %d)", l_exact, l_exact + l_late);
    CHECK(rescued_wrong == 0 && rescued > 0,
          "where the value step refuses (%d records), prediction recovers %d and is never wrong (%d); combined finds %d vs %d",
          v_refused, rescued, rescued_wrong, any_found, v_found);
    int lpc_int = 0;
    for (int i = 0; i < 200; i++) {
        synth(rec, (double)(3 + (i % 20)), (i & 1) ? -1 : 1, i & 2, 40.0);
        uint32_t k;
        if (trig_edge_find_seam_lpc(rec, &k)) lpc_int++;
    }
    CHECK(lpc_int == 0, "integer periods (no seam in the circle) refused by prediction too (%d answered)", lpc_int);
    int agree = 0, both = 0;
    for (unsigned i = 0; i < EDGE_FIXTURE_N; i++) {
        uint32_t kv, kl;
        if (trig_edge_find_seam(EDGE_FIXTURES[i].rec, &kv) && trig_edge_find_seam_lpc(EDGE_FIXTURES[i].rec, &kl)) {
            both++;
            int d = (int)((kl - kv + 1536u) & 1023u) - 512;
            if (d >= 0 && d <= 2) agree++;
        }
    }
    CHECK(both == 0 || agree == both, "real records: prediction agrees with the value step where both answer (%d/%d)", agree, both);

    printf("[6] stray sample 0 (unit #1: 13/20 raw records at 201 Hz / 0x10)\n");
    int s_exact = 0, s_bad = 0, s_found = 0;
    for (int i = 0; i < 600; i++) {
        synth(rec, fast[1 + (i % 8)], (i & 1) ? -1 : 1, (i / 8) & 1, 25.0 + urand() * 35.0);
        if (last_rot < 4u || last_rot > 1020u) continue;          /* seam must not BE sample 0 */
        rec[0] = (uint8_t)(rec[1] + ((i & 2) ? 30 : -30));         /* a stray first sample */
        uint32_t k;
        if (!trig_edge_find_seam_any(rec, &k)) continue;
        s_found++;
        int d = (int)((k - last_rot + 1536u) & 1023u) - 512;
        if (d == 0) s_exact++; else s_bad++;
    }
    CHECK(s_bad == 0 && s_found >= 300, "with a stray sample 0: %d found, %d exact, %d wrong", s_found, s_exact, s_bad);

    printf("[7] level-crossing seam finder (stage-250, G-240)\n");
    {
        /* 0x14: 10 Hz at 500 S/s = 50 samples per period (about 40 edges);
         * 0x13: 125 samples per period (about 16 edges). */
        const double pers[] = { 50.0, 49.3, 125.0, 37.7 };
        for (int noise = 0; noise <= 1; noise++) {
            int tot = 0, other = 0, found = 0, wrong = 0, cut_sum = 0, per_sum = 0;
            for (int i = 0; i < 2000; i++) {
                int rising = i & 1;
                synth_sq(rec, pers[i % 4], rising, noise, 20 + (int)(urand() * 40.0),
                         (int)(urand() * 20.0) - 10);
                if (last_rot < 2u) continue;              /* keep sample 0 inside the record */
                tot++;
                uint32_t k;
                if (trig_edge_find_seam_any(rec, &k)) { other++; continue; }
                if (!trig_edge_find_seam_xing(rec, 128, rising, &k)) continue;
                found++;
                int cut, trig_ok, np;
                judge(rec, k, &cut, &trig_ok, &np);
                cut_sum += cut; per_sum += np;
                if (cut || !trig_ok) wrong++;
            }
            CHECK(other * 2 < tot, "noise %d: value step and predictor answer few (%d of %d)", noise, other, tot);
            CHECK(wrong == 0 && cut_sum == 0, "noise %d: %d found, %d wrong, %d cut periods in %d",
                  noise, found, wrong, cut_sum, per_sum);
            CHECK(found * 10 >= (tot - other) * 9, "noise %d: finds >= 90%% of the rest (%d of %d)",
                  noise, found, tot - other);
        }
        int refused = 1;
        for (int i = 0; i < 200; i++) {                   /* 32 or 16 periods: no seam */
            synth_sq(rec, (i & 1) ? 32.0 : 64.0, i & 2, (i >> 2) & 1, 40, 0);
            uint32_t k;
            if (trig_edge_find_seam_xing(rec, 128, -1, &k)) refused = 0;
        }
        CHECK(refused, "integer periods (no outlier interval) refused");
        int off = 1;
        for (int i = 0; i < 200; i++) {                   /* level above the signal */
            synth_sq(rec, 50.0, i & 1, 1, 30, 0);
            uint32_t k;
            if (trig_edge_find_seam_xing(rec, 200, -1, &k)) off = 0;
        }
        CHECK(off, "level off the signal refused");
        uint32_t k = 777u;
        for (int i = 0; i < 1024; i++) rec[i] = (uint8_t)(128 + ((i * 37) % 11) - 5);
        CHECK(!trig_edge_find_seam_xing(rec, 128, -1, &k) && k == 777u, "noise around the level refused, k untouched");
    }

    printf("\n%s\n", fails ? "FAILED" : "all passed");
    return fails ? 1 : 0;
}
