/* See trig_edge.h. Integer-only; ~3 passes over 1024 bytes. */
#include "trig_edge.h"

#define N          TRIG_EDGE_REC_N
#define MASK       (N - 1u)
#define TRIG_POS   512
#define SEARCH     24      /* crossing searched within +/-24 of the trigger
                              index: EXP-53 placed it 501..520 */
#define SEAM_RATIO 2       /* seam jump must be >= 2x every other step   */
#define SEAM_MIN   6       /* and at least 6 counts                      */
#define DMIN       4       /* slope must clear 4 counts at some scale    */

static inline int at(const volatile uint8_t *rec, uint32_t k, int i)
{
    return (int)rec[(k + (uint32_t)i) & MASK];
}

int trig_edge_find_seam(const volatile uint8_t *rec, uint32_t *k_out)
{
    if (rec == 0)
        return 0;
    /* The largest circular step. k = its newer side = oldest sample. */
    uint32_t k = 0;
    int j1 = -1;
    /* Steps touching sample 0 are skipped: on unit #1 sample 0 of a read
     * is often a stray value that fits neither neighbour (2026-09-22: 13/20
     * records at 201 Hz / 0x10; samples 1023 -> 1 are continuous), and its
     * two false steps defeated the ratio test. */
    for (uint32_t i = 2; i < N; i++) {
        int d = (int)rec[i] - (int)rec[(i + MASK) & MASK];
        if (d < 0) d = -d;
        if (d > j1) { j1 = d; k = i; }
    }
    int j2 = 0;
    for (uint32_t i = 2; i < N; i++) {           /* sample 0 skipped, as above */
        uint32_t off = (i - k) & MASK;           /* skip k-2 .. k+2 */
        if (off <= 2u || off >= N - 2u) continue;
        int d = (int)rec[i] - (int)rec[(i + MASK) & MASK];
        if (d < 0) d = -d;
        if (d > j2) j2 = d;
    }
    if (j1 < SEAM_MIN || j1 < SEAM_RATIO * j2)
        return 0;
    *k_out = k;
    return 1;
}

trig_edge_class_t trig_edge_classify(const volatile uint8_t *rec, int crossing)
{
    /* 1. The seam (see trig_edge_find_seam). */
    uint32_t k = 0;
    if (!trig_edge_find_seam(rec, &k))
        return TRIG_EDGE_CLASS_UNKNOWN;

    /* 2. The crossing nearest the trigger index, on a 4-sample running sum
     *    (r = the un-rotated record, r[0] oldest). */
    int c4 = 4 * crossing;
    int best = -1;
    for (int j = TRIG_POS - SEARCH; j < TRIG_POS + SEARCH; j++) {
        int a = at(rec, k, j - 1) + at(rec, k, j) + at(rec, k, j + 1) + at(rec, k, j + 2);
        int b = at(rec, k, j)     + at(rec, k, j + 1) + at(rec, k, j + 2) + at(rec, k, j + 3);
        if ((a < c4 && c4 <= b) || (a >= c4 && c4 > b)) {
            int dj = j - TRIG_POS; if (dj < 0) dj = -dj;
            int db = best - TRIG_POS; if (db < 0) db = -db;
            if (best < 0 || dj < db) best = j;
        }
    }
    if (best < 0)
        return TRIG_EDGE_CLASS_UNKNOWN;          /* no trigger at this level */

    /* 3. The slope at the smallest scale that clears the noise: a fast edge
     *    is read locally, a slow ramp over a wider window. */
    static const int scales[] = { 4, 8, 16, 32, 64 };
    for (unsigned s = 0; s < sizeof scales / sizeof scales[0]; s++) {
        int w = scales[s];
        int after = 0, before = 0;
        for (int i = 1; i <= w; i++) {
            after  += at(rec, k, best + i);
            before += at(rec, k, best + 1 - i);
        }
        int d = after - before;                  /* = w x (mean difference) */
        if (d >= DMIN * w)  return TRIG_EDGE_CLASS_RISING;
        if (d <= -DMIN * w) return TRIG_EDGE_CLASS_FALLING;
    }
    return TRIG_EDGE_CLASS_UNKNOWN;
}

/* ── Linear-prediction seam finder (see trig_edge.h) ─────────────────── */
#define LPC_P 8

static float lpc_err(const volatile uint8_t *rec, const float *a, float mean, uint32_t n)
{
    float e = 0.0f;
    for (int j = 0; j <= LPC_P; j++)
        e += a[j] * ((float)rec[(n - (uint32_t)j) & MASK] - mean);
    return e < 0.0f ? -e : e;
}

int trig_edge_find_seam_lpc(const volatile uint8_t *rec, uint32_t *k_out)
{
    if (rec == 0)
        return 0;
    /* Sample 0 is excluded throughout (stray on unit #1, see
     * trig_edge_find_seam): from the mean, the autocorrelation, and every
     * prediction that would read it (n = 0..P). */
    float mean = 0.0f;
    for (uint32_t i = 1; i < N; i++) mean += (float)rec[i];
    mean /= (float)(N - 1u);

    /* Circular autocorrelation r[0..P]. */
    float r[LPC_P + 1];
    for (int l = 0; l <= LPC_P; l++) {
        float acc = 0.0f;
        for (uint32_t i = 0; i < N; i++) {
            uint32_t j = (i + MASK + 1u - (uint32_t)l) & MASK;
            if (i == 0u || j == 0u) continue;
            acc += ((float)rec[i] - mean) * ((float)rec[j] - mean);
        }
        r[l] = acc;
    }
    if (r[0] <= 0.0f)
        return 0;

    /* Levinson-Durbin: a[0] = 1, e = prediction error power. */
    float a[LPC_P + 1] = { 1.0f };
    float tmp[LPC_P + 1];
    float e = r[0];
    for (int i = 1; i <= LPC_P; i++) {
        float acc = r[i];
        for (int j = 1; j < i; j++) acc += a[j] * r[i - j];
        float kk = -acc / e;
        for (int j = 0; j <= i; j++) tmp[j] = a[j];
        for (int j = 1; j < i; j++) a[j] = tmp[j] + kk * tmp[i - j];
        a[i] = kk;
        e *= (1.0f - kk * kk);
        if (e <= 0.0f)
            return 0;
    }

    /* Pass 1: the peak prediction error. */
    uint32_t m = 0;
    float peak = -1.0f;
    for (uint32_t n = (uint32_t)LPC_P + 1u; n < N; n++) {
        float v = lpc_err(rec, a, mean, n);
        if (v > peak) { peak = v; m = n; }
    }
    /* Pass 2: the largest error more than P samples from the peak. */
    float second = 0.0f;
    for (uint32_t n = (uint32_t)LPC_P + 1u; n < N; n++) {
        uint32_t d = (n - m) & MASK;
        if (d <= (uint32_t)LPC_P || d >= N - (uint32_t)LPC_P) continue;
        float v = lpc_err(rec, a, mean, n);
        if (v > second) second = v;
    }
    if (peak < 3.0f * second)
        return 0;
    /* Pass 3: the first prediction that reaches across the seam is AT the
     * seam; walk back to the earliest index within P whose error is >= 35%
     * of the peak. */
    uint32_t k = m;
    /* A peak within 2P+1 of index 0 cannot be walked back without reading
     * sample 0: refuse rather than answer short (the value step, which only
     * skips sample 0 itself, covers seams there). */
    if (m <= 2u * (uint32_t)LPC_P + 1u)
        return 0;
    for (uint32_t o = 1; o <= (uint32_t)LPC_P; o++) {
        uint32_t n = (m - o) & MASK;
        if (lpc_err(rec, a, mean, n) >= 0.35f * peak)
            k = n;
    }
    *k_out = k;
    return 1;
}

int trig_edge_find_seam_any(const volatile uint8_t *rec, uint32_t *k)
{
    return trig_edge_find_seam(rec, k) || trig_edge_find_seam_lpc(rec, k);
}

/* ── Level-crossing seam finder (see trig_edge.h) ────────────────────── */
#define XING_HYST   3
#define XING_MAX    128     /* crossings per direction */
#define XING_MIN    4
#define XING_SLACK  12      /* EXP-53: trigger at 501..520 */

static uint16_t xing_r[XING_MAX], xing_f[XING_MAX], xing_tmp[XING_MAX];

typedef struct { uint32_t s; uint32_t len; int info; } xing_arc_t;   /* k in s+1 .. s+len */

static uint32_t xing_iv(const uint16_t *x, uint32_t n, uint32_t j)
{
    return ((uint32_t)x[(j + 1u) % n] - (uint32_t)x[j]) & MASK;
}

/* The arc of the outlier intervals of one direction. Returns 0 when the
 * outliers are not one contiguous run (or more than 2); arc->info = 0 when
 * there is none. strict: > max(2 samples, 3 %); else > 10 %. */
static int xing_arc(const uint16_t *x, uint32_t n, int strict, xing_arc_t *arc)
{
    for (uint32_t j = 0; j < n; j++) {               /* insertion sort */
        uint16_t v = (uint16_t)xing_iv(x, n, j);
        uint32_t i = j;
        while (i > 0u && xing_tmp[i - 1u] > v) { xing_tmp[i] = xing_tmp[i - 1u]; i--; }
        xing_tmp[i] = v;
    }
    uint32_t med = xing_tmp[n / 2u];
    uint32_t tol = strict ? (med * 3u + 99u) / 100u : (med * 10u + 99u) / 100u;
    if (strict && tol < 2u) tol = 2u;
    uint32_t first = n, cnt = 0;
    for (uint32_t j = 0; j < n; j++) {
        uint32_t d = xing_iv(x, n, j);
        uint32_t dev = d > med ? d - med : med - d;
        int out = dev > tol;
        uint32_t jp = (j + n - 1u) % n;
        uint32_t dp = xing_iv(x, n, jp);
        uint32_t devp = dp > med ? dp - med : med - dp;
        if (out) {
            cnt++;
            if (!(devp > tol)) {                     /* a run starts at j */
                if (first != n) return 0;            /* a second run */
                first = j;
            }
        }
    }
    arc->info = 0;
    if (cnt == 0u) return 1;
    if (cnt > 2u || first == n) return 0;            /* first == n: every one */
    arc->info = 1;
    if (cnt == 2u) {
        /* Two outliers that share a crossing: the seam step itself crosses
         * the level, so that crossing IS the oldest sample. Any other k would
         * leave the step inside the record as a false crossing. */
        arc->s = ((uint32_t)x[(first + 1u) % n] + MASK) & MASK;
        arc->len = 1;
    } else {
        arc->s = x[first];
        arc->len = xing_iv(x, n, first);
    }
    return 1;
}

static int xing_intersect(const xing_arc_t *a, const xing_arc_t *b, xing_arc_t *r)
{
    if (!a->info) { *r = *b; return b->info; }
    if (!b->info) { *r = *a; return 1; }
    uint32_t o = (b->s - a->s) & MASK;
    if (o < a->len) {
        r->s = b->s;
        r->len = (b->len < a->len - o) ? b->len : a->len - o;
    } else {
        o = (a->s - b->s) & MASK;
        if (o >= b->len) return 0;
        r->s = a->s;
        r->len = (a->len < b->len - o) ? a->len : b->len - o;
    }
    r->info = 1;
    return r->len > 0u;
}

/* k for crossing list x inside arc r: 2 = inside, 1 = moved in, 0 = none. */
static int xing_pick(const uint16_t *x, uint32_t n, const xing_arc_t *r, uint32_t *k)
{
    int best = 0;
    uint32_t bk = 0, bd = MASK;
    for (uint32_t j = 0; j < n; j++) {
        uint32_t k0 = ((uint32_t)x[j] + N - (uint32_t)TRIG_POS) & MASK;
        uint32_t o = (k0 - r->s) & MASK;
        if (o >= 1u && o <= r->len) {
            if (best == 2) return 0;                 /* two fit: ambiguous */
            best = 2; bk = k0; bd = 0;
            continue;
        }
        if (best == 2) continue;
        uint32_t d_lo = (r->s + 1u - k0) & MASK;     /* k0 before the arc */
        uint32_t d_hi = (k0 - r->s - r->len) & MASK; /* k0 after it */
        uint32_t d = d_lo < d_hi ? d_lo : d_hi;
        if (d <= XING_SLACK && d < bd) {
            best = 1; bd = d;
            bk = (d == d_lo) ? ((r->s + 1u) & MASK) : ((r->s + r->len) & MASK);
        }
    }
    if (best) *k = bk;
    return best;
}

int trig_edge_find_seam_xing(const volatile uint8_t *rec, int c, int prefer, uint32_t *k_out)
{
    if (rec == 0 || c < XING_HYST + 1 || c > 255 - XING_HYST - 1)
        return 0;
    /* Two passes around the circle: the first only arms the Schmitt state.
     * Sample 0 is skipped (stray on unit #1, see trig_edge_find_seam). */
    uint32_t nr = 0, nf = 0;
    int arm_r = 0, arm_f = 0;
    for (uint32_t p = 0; p < 2u * N; p++) {
        uint32_t i = p & MASK;
        if (i == 0u) continue;
        int s = (int)rec[i];
        if (s <= c - XING_HYST) arm_r = 1;
        if (s >= c + XING_HYST) arm_f = 1;
        if (arm_r && s >= c) {
            arm_r = 0; arm_f = (s >= c + XING_HYST);
            if (p >= N) { if (nr >= XING_MAX) return 0; xing_r[nr++] = (uint16_t)i; }
        } else if (arm_f && s <= c) {
            arm_f = 0; arm_r = (s <= c - XING_HYST);
            if (p >= N) { if (nf >= XING_MAX) return 0; xing_f[nf++] = (uint16_t)i; }
        }
    }
    if (nr < XING_MIN || nf < XING_MIN)
        return 0;
    xing_arc_t ar, af, r;
    int ok = 0;
    for (int strict = 1; strict >= 0 && !ok; strict--) {
        if (!xing_arc(xing_r, nr, strict, &ar) || !xing_arc(xing_f, nf, strict, &af))
            continue;
        ok = xing_intersect(&ar, &af, &r);
        if (!ar.info && !af.info) return 0;          /* no seam to undo */
    }
    if (!ok)
        return 0;
    uint32_t kr = 0, kf = 0;
    int gr = xing_pick(xing_r, nr, &r, &kr);
    int gf = xing_pick(xing_f, nf, &r, &kf);
    int use_r;
    if (gr == 0 && gf == 0) return 0;
    if (gr != gf)          use_r = gr > gf;          /* inside beats moved in */
    else if (prefer >= 0)  use_r = prefer == 1;
    else                   use_r = 1;
    *k_out = use_r ? kr : kf;
    return 1;
}

int trig_edge_anchor(const volatile uint8_t *rec, int c, int rising)
{
    if (rec == 0 || c < 4 || c > 251)
        return -1;
    const int hyst = 3;
    int best = -1;
    int armed = 0;
    for (int i = TRIG_POS - 64; i <= TRIG_POS + 48; i++) {
        int s = (int)rec[i];
        if (!armed) {
            if (rising ? (s <= c - hyst) : (s >= c + hyst)) armed = 1;
        } else if (rising ? (s >= c) : (s <= c)) {
            if (i >= TRIG_POS - 48) {
                int d = i - TRIG_POS; if (d < 0) d = -d;
                int db = best - TRIG_POS; if (db < 0) db = -db;
                if (best < 0 || d < db) best = i;
            }
            armed = 0;                            /* re-arm for the next one */
        }
    }
    return best;
}
