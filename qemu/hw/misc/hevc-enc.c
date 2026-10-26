/*
 * A minimal HEVC intra encoder for the HEVC engine models (xm-hevc).
 *
 * Every picture is an IDR access unit with its own VPS, SPS and PPS, one I
 * slice, 16x16 CTBs each split into four 8x8 coding units.  Every CU is
 * 2Nx2N intra DC (chroma: DM, so DC too) with one transform unit -- an 8x8
 * luma and two 4x4 chroma blocks -- that carries only its DC coefficient.
 * So the picture comes out at 8x8 (chroma 4x4) block resolution, which is
 * plenty for a test pattern.  PCM, sign hiding and transform skip are off,
 * QP deltas cap the QP (HEVC_QP_MAX), and the encoder reconstructs exactly as a
 * decoder does (ITU-T H.265 8.4.4.2: reference sample availability and
 * substitution, DC prediction with its edge filter; 8.6.2-8.6.4: scaling
 * and the inverse transform of a DC-only block), so intra prediction never
 * drifts and the decoded picture equals the reconstruction.
 *
 * The CABAC engine is H.264's (the arithmetic coder is the same), copied
 * from hantro-h264.c with its x264-derived tables; the contexts and their
 * I-slice init values are HEVC's (Tables 9-5 to 9-37).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/thread.h"
#include "qemu/atomic.h"
#include <math.h>
#include "hevc-enc.h"
#include "hantro-h264-tables.h"

/* ---- bit writer ---------------------------------------------------------- */

typedef struct BitWriter {
    uint8_t *p;
    size_t n, cap;
    uint32_t acc;
    int bits;
} BitWriter;

static void bw_bits(BitWriter *w, uint32_t v, int n)
{
    while (n--) {
        w->acc = (w->acc << 1) | ((v >> n) & 1);
        if (++w->bits == 8) {
            if (w->n < w->cap) {
                w->p[w->n] = w->acc;
            }
            w->n++;
            w->acc = w->bits = 0;
        }
    }
}

static void bw_ue(BitWriter *w, uint32_t v)
{
    int len = 0;
    uint32_t t = v + 1;

    while (t >> (len + 1)) {
        len++;
    }
    bw_bits(w, 0, len);
    bw_bits(w, v + 1, len + 1);
}

static void bw_se(BitWriter *w, int32_t v)
{
    bw_ue(w, v > 0 ? 2 * v - 1 : -2 * v);
}

static void bw_trailing(BitWriter *w)
{
    bw_bits(w, 1, 1);
    while (w->bits) {
        bw_bits(w, 0, 1);
    }
}

/* ---- CABAC engine (as hantro-h264.c) ------------------------------------- */

enum {
    CTX_SPLIT_CU = 0,           /* 3 */
    CTX_PART_MODE = 3,
    CTX_PREV_INTRA = 4,
    CTX_CHROMA_MODE = 5,
    CTX_CBF_LUMA = 6,           /* 2 */
    CTX_CBF_CHROMA = 8,         /* 4 */
    CTX_LAST_X = 12,            /* 18 */
    CTX_LAST_Y = 30,            /* 18 */
    CTX_GT1 = 48,               /* 24 */
    CTX_GT2 = 72,               /* 6 */
    CTX_CU_QP_DELTA = 78,       /* 2 */
    CTX_SKIP = 80,              /* 3, P slices */
    CTX_PRED_MODE = 83,         /* P slices */
    CTX_SAO_MERGE = 84,
    CTX_SAO_TYPE = 85,
    CTX_MERGE_IDX = 86,         /* P slices */
    CTX_NUM = 87
};

/* initValue, in the order above: initType 0 (I slices) and 1 (P slices
 * with cabac_init_flag 0, which is what the library's P slice headers say) */
static const uint8_t hevc_init[2][CTX_NUM] = { {
    139, 141, 157,                                      /* split_cu_flag */
    184,                                                /* part_mode */
    184,                                                /* prev_intra_luma */
    63,                                                 /* intra_chroma */
    111, 141,                                           /* cbf_luma */
    94, 138, 182, 154,                                  /* cbf_cb/cr */
    110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,
    79, 108, 123, 63,                                   /* last_x_prefix */
    110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,
    79, 108, 123, 63,                                   /* last_y_prefix */
    140, 92, 137, 138, 140, 152, 138, 139, 153, 74, 149, 92, 139, 107, 122,
    152, 140, 179, 166, 182, 140, 227, 122, 197,        /* greater1 */
    138, 153, 136, 167, 152, 152,                       /* greater2 */
    154, 154,                                           /* cu_qp_delta_abs */
    154, 154, 154,                                      /* (cu_skip_flag) */
    154,                                                /* (pred_mode) */
    153,                                                /* sao_merge */
    200,                                                /* sao_type_idx */
    154,                                                /* (merge_idx) */
}, {
    107, 139, 126,
    154,
    154,
    152,
    153, 111,
    149, 107, 167, 154,
    125, 110, 94, 110, 95, 79, 125, 111, 110, 78, 110, 111, 111, 95,
    94, 108, 123, 108,
    125, 110, 94, 110, 95, 79, 125, 111, 110, 78, 110, 111, 111, 95,
    94, 108, 123, 108,
    154, 196, 196, 167, 154, 152, 167, 182, 182, 134, 149, 136, 153, 121, 136,
    137, 169, 194, 166, 167, 154, 167, 137, 182,
    107, 167, 91, 122, 107, 167,
    154, 154,
    197, 185, 201,
    149,
    153,
    185,
    122,
} };

typedef struct Cabac {
    uint8_t state[CTX_NUM];
    int low, range, queue, outstanding;
    uint8_t *p, *start, *end;
} Cabac;

static void cabac_init(Cabac *c, int qp, int init_type, uint8_t *p,
                       uint8_t *end)
{
    int i;

    for (i = 0; i < CTX_NUM; i++) {
        const int m = (hevc_init[init_type][i] >> 4) * 5 - 45;
        const int n = ((hevc_init[init_type][i] & 15) << 3) - 16;
        const int st = MIN(MAX(((m * MIN(MAX(qp, 0), 51)) >> 4) + n, 1), 126);
        c->state[i] = (MIN(st, 127 - st) << 1) | (st >> 6);
    }
    c->low = 0;
    c->range = 0x1fe;
    c->queue = -9;
    c->outstanding = 0;
    c->p = c->start = p;
    c->end = end;
}

static void cabac_put(Cabac *c, uint8_t b)
{
    if (c->p < c->end) {
        *c->p = b;
    }
    c->p++;
}

static void cabac_putbyte(Cabac *c)
{
    if (c->queue >= 0) {
        const int out = c->low >> (c->queue + 10);
        c->low &= (0x400 << c->queue) - 1;
        c->queue -= 8;
        if ((out & 0xff) == 0xff) {
            c->outstanding++;
        } else {
            const int carry = out >> 8;
            if (c->p > c->start && c->p - 1 < c->end) {
                c->p[-1] += carry;
            }
            while (c->outstanding > 0) {
                cabac_put(c, carry - 1);
                c->outstanding--;
            }
            cabac_put(c, out);
        }
    }
}

static void cabac_renorm(Cabac *c)
{
    const int shift = h264_cabac_renorm_shift[c->range >> 3];

    c->range <<= shift;
    c->low <<= shift;
    c->queue += shift;
    cabac_putbyte(c);
}

static void cabac_decision(Cabac *c, int ctx, int b)
{
    const int st = c->state[ctx];
    const int lps = h264_cabac_range_lps[st >> 1][(c->range >> 6) - 4];

    c->range -= lps;
    if (b != (st & 1)) {
        c->low += c->range;
        c->range = lps;
    }
    c->state[ctx] = h264_cabac_transition[st][b];
    cabac_renorm(c);
}

static void cabac_bypass(Cabac *c, int b)
{
    c->low <<= 1;
    c->low += b ? c->range : 0;
    c->queue += 1;
    cabac_putbyte(c);
}

static void cabac_terminate0(Cabac *c)
{
    c->range -= 2;
    cabac_renorm(c);
}

/* end_of_slice_segment_flag = 1, then flush (with the stop bit) */
static void cabac_flush(Cabac *c)
{
    c->low += c->range - 2;
    c->low |= 1;
    c->low <<= 9;
    c->queue += 9;
    cabac_putbyte(c);
    cabac_putbyte(c);
    c->low <<= -c->queue;
    c->low |= 1 << 10;
    c->queue = 0;
    cabac_putbyte(c);
    while (c->outstanding > 0) {
        cabac_put(c, 0xff);
        c->outstanding--;
    }
}

/* ---- residual: one DC coefficient (7.3.8.11, 9.3.3.11) -------------------- */

static void cabac_egk(Cabac *c, uint32_t v, int k)
{
    for (;;) {
        if (v >= (1u << k)) {
            cabac_bypass(c, 1);
            v -= 1u << k;
            k++;
        } else {
            cabac_bypass(c, 0);
            while (k--) {
                cabac_bypass(c, (v >> k) & 1);
            }
            return;
        }
    }
}

/* residual_coding() of a TB whose only non-zero coefficient is DC: last
 * position (0, 0), one level */
static void residual_dc(Cabac *c, int level, int log2, bool chroma)
{
    const int a = abs(level);
    int last_ctx;

    if (chroma) {
        last_ctx = 15;
    } else {
        last_ctx = 3 * (log2 - 2) + ((log2 - 1) >> 2);
    }
    cabac_decision(c, CTX_LAST_X + last_ctx, 0);    /* last_x_prefix = 0 */
    cabac_decision(c, CTX_LAST_Y + last_ctx, 0);    /* last_y_prefix = 0 */
    /* sub-block 0 only, its position 0 the last: no sig flags */
    cabac_decision(c, CTX_GT1 + 1 + (chroma ? 16 : 0), a > 1);
    if (a > 1) {
        cabac_decision(c, CTX_GT2 + (chroma ? 4 : 0), a > 2);
    }
    cabac_bypass(c, level < 0);
    if (a > 2) {
        const uint32_t v = a - 3;                   /* cRiceParam 0 */
        if (v < 4) {
            for (uint32_t i = 0; i < v; i++) {
                cabac_bypass(c, 1);
            }
            cabac_bypass(c, 0);
        } else {
            for (int i = 0; i < 4; i++) {
                cabac_bypass(c, 1);
            }
            cabac_egk(c, v - 4, 1);
        }
    }
}

/* ---- prediction, scaling, reconstruction --------------------------------- */

typedef struct Pic {
    int w, h;                       /* luma, multiples of 8 */
    uint8_t *y, *cb, *cr;           /* reconstruction */
    uint8_t *done;                  /* decoded 8x8 blocks, (w/8) x (h/8) */
    const uint8_t *sy, *suv;
    int sw, sh, sys, scs;
    int *xmap, *ymap;               /* luma x, y -> source column, row */
    int *cxmap, *cymap;             /* chroma likewise (source pairs) */
    bool qp_delta_coded;            /* in the current quantization group */
    int qp;                         /* the QP the CUs are quantized with */
    int qp_prev;                    /* qPY_PREV: the last group's QpY */
    const HevcEncRef *ref;          /* a P slice's reference, or NULL */
    uint8_t *cy, *ccb, *ccr;        /* the source scaled to w x h */
    uint8_t *skip;                  /* per 8x8 CU: skipped (else intra) */
    uint8_t *cuqp;                  /* per 8x8 CU: its QpY */
    int skip_thr;                   /* skip_threshold() at qp */
} Pic;

static void hevc_enc_ref_clear(HevcEncRef *r);

/* a decoded picture (deblocked): what the next P picture of the stream
 * refers to */
struct HevcEncRef {
    int w, h;
    uint8_t *y, *cb, *cr;
};

static uint8_t clip1(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* is the luma sample (x, y) available for intra prediction (6.4.1) */
static bool avail(Pic *p, int x, int y)
{
    return x >= 0 && y >= 0 && x < p->w && y < p->h &&
           p->done[(y >> 3) * (p->w >> 3) + (x >> 3)];
}

/*
 * 8.4.4.2.2 reference samples of an n x n block at (x0, y0) of a plane whose
 * samples are scale x luma ones, then 8.4.4.2.6 DC: pred[] (n x n).
 */
static void pred_dc(Pic *p, const uint8_t *plane, int stride, int scale,
                    int x0, int y0, int n, bool luma, int *pred)
{
    int ref[4 * 32 + 1];            /* p[-1][2n-1] .. p[-1][-1] .. p[2n-1][-1] */
    bool av[4 * 32 + 1];
    const int cnt = 4 * n + 1;
    int i, k, any = 0;

    for (i = 0; i < cnt; i++) {
        int x, y;
        if (i < 2 * n) {            /* left column, bottom up */
            x = x0 - 1;
            y = y0 + 2 * n - 1 - i;
        } else if (i == 2 * n) {    /* corner */
            x = x0 - 1;
            y = y0 - 1;
        } else {                    /* top row, left to right */
            x = x0 + i - 2 * n - 1;
            y = y0 - 1;
        }
        av[i] = avail(p, x * scale, y * scale);
        ref[i] = av[i] ? plane[y * stride + x] : 0;
        any |= av[i];
    }
    if (!any) {
        for (i = 0; i < cnt; i++) {
            ref[i] = 128;
        }
    } else {
        if (!av[0]) {
            for (k = 1; k < cnt && !av[k]; k++) {
            }
            ref[0] = ref[k];
        }
        for (i = 1; i < cnt; i++) {
            if (!av[i]) {
                ref[i] = ref[i - 1];
            }
        }
    }
    /* left p[-1][y] = ref[2n-1-y], top p[x][-1] = ref[2n+1+x] */
    int sum = n, log2n = 31 - __builtin_clz(n);
    for (i = 0; i < n; i++) {
        sum += ref[2 * n - 1 - i] + ref[2 * n + 1 + i];
    }
    const int dc = sum >> (log2n + 1);
    for (i = 0; i < n * n; i++) {
        pred[i] = dc;
    }
    if (luma && n < 32) {
        pred[0] = (ref[2 * n - 1] + 2 * dc + ref[2 * n + 1] + 2) >> 2;
        for (i = 1; i < n; i++) {
            pred[i] = (ref[2 * n + 1 + i] + 3 * dc + 2) >> 2;          /* row 0 */
            pred[i * n] = (ref[2 * n - 1 - i] + 3 * dc + 2) >> 2;      /* col 0 */
        }
    }
}

static const int level_scale[6] = { 40, 45, 51, 57, 64, 72 };

/* 8.6.2-8.6.4.2: the residual every sample of an n x n block gets from a
 * DC-only level (flat scaling, 8-bit) */
static int dc_residual(int level, int qp, int log2n)
{
    const int bd_shift = 8 + log2n - 5;
    int64_t d = ((int64_t)level * 16 * level_scale[qp % 6]) << (qp / 6);
    d = (d + (1 << (bd_shift - 1))) >> bd_shift;
    d = MIN(MAX(d, -32768), 32767);
    int g = (int)((64 * d + 64) >> 7);
    g = MIN(MAX(g, -32768), 32767);
    return (64 * g + 2048) >> 12;
}

/* the level whose residual comes closest to target */
static int dc_level_search(int target, int qp, int log2n)
{
    const int bd_shift = 8 + log2n - 5;
    /* the two transform stages take d to ((64 d + 64) >> 7) * 64 >> 12,
     * about d / 128; d is about level * 16 s 2^(qp/6) >> bd_shift */
    const double per = 16.0 * level_scale[qp % 6] * (1 << (qp / 6)) /
                       (double)(1 << bd_shift) / 128.0;
    const int est = (int)lround(target / per);
    int best = 0, best_err = INT_MAX;

    for (int l = est - 3; l <= est + 3; l++) {
        const int e = abs(dc_residual(l, qp, log2n) - target);
        if (e < best_err || (e == best_err && abs(l) < abs(best))) {
            best_err = e;
            best = l;
        }
    }
    return MIN(MAX(best, -32767), 32767);
}

/* dc_level_search, tabulated: targets are mean differences, -255..255 */
static int dc_level(int target, int qp, int log2n)
{
    static int16_t table[52][4][511];
    static bool filled[52][4];
    static QemuMutex lock;
    static bool lock_init;

    if (target < -255 || target > 255 || log2n < 2 || log2n > 5) {
        return dc_level_search(target, qp, log2n);
    }
    if (!qatomic_load_acquire(&filled[qp][log2n - 2])) {
        if (!lock_init) {
            qemu_mutex_init(&lock);
            lock_init = true;
        }
        qemu_mutex_lock(&lock);
        if (!filled[qp][log2n - 2]) {
            for (int t = -255; t <= 255; t++) {
                table[qp][log2n - 2][t + 255] = dc_level_search(t, qp, log2n);
            }
            qatomic_store_release(&filled[qp][log2n - 2], true);
        }
        qemu_mutex_unlock(&lock);
    }
    return table[qp][log2n - 2][target + 255];
}

static int src_y(Pic *p, int x, int y)
{
    return p->sy[p->ymap[y] + p->xmap[x]];
}

static int src_c(Pic *p, int comp, int x, int y)
{
    return p->suv[p->cymap[y] + p->cxmap[x] + comp];
}

/* the source sample offsets of the nearest-neighbour scaling */
static void pic_maps(Pic *p)
{
    int i;

    p->xmap = g_new(int, p->w);
    p->ymap = g_new(int, p->h);
    p->cxmap = g_new(int, p->w / 2);
    p->cymap = g_new(int, p->h / 2);
    for (i = 0; i < p->w; i++) {
        p->xmap[i] = i * p->sw / p->w;
    }
    for (i = 0; i < p->h; i++) {
        p->ymap[i] = (i * p->sh / p->h) * p->sys;
    }
    for (i = 0; i < p->w / 2; i++) {
        p->cxmap[i] = 2 * (i * (p->sw / 2) / (p->w / 2));
    }
    for (i = 0; i < p->h / 2; i++) {
        p->cymap[i] = (i * (p->sh / 2) / (p->h / 2)) * p->scs;
    }
}

static void pic_maps_free(Pic *p)
{
    g_free(p->xmap);
    g_free(p->ymap);
    g_free(p->cxmap);
    g_free(p->cymap);
}

/* chroma QP for ChromaArrayType 1 (Table 8-10) */
static int chroma_qp(int qpi)
{
    static const uint8_t t[] = { 29, 30, 31, 32, 33, 33, 34, 34, 35, 35,
                                 36, 36, 37, 37 };
    if (qpi < 30) {
        return qpi;
    }
    if (qpi > 43) {
        return qpi - 6;
    }
    return t[qpi - 30];
}

/* cu_qp_delta_abs (prefix TU of 5, first bin context 0, the rest 1; then
 * EG0) and cu_qp_delta_sign_flag (9.3.3.10) */
static void cu_qp_delta(Cabac *c, int d)
{
    const int a = abs(d);

    for (int i = 0; i < MIN(a, 5); i++) {
        cabac_decision(c, CTX_CU_QP_DELTA + !!i, 1);
    }
    if (a < 5) {
        cabac_decision(c, CTX_CU_QP_DELTA + !!a, 0);
    } else {
        cabac_egk(c, a - 5, 0);
    }
    if (a) {
        cabac_bypass(c, d < 0);
    }
}

/*
 * A P picture's CU is skipped (its reference block copied) where that costs
 * no more than coding it intra: the block's squared error against the
 * source, with the bits the intra CU would take at lambda (HM's for P
 * pictures).  An intra block next to a bright neighbour has its DC
 * prediction's filtered edge pulled up and its interior pulled down to keep
 * the mean; skipped as long as its own source does not change, it would keep
 * that after the neighbour has gone.  Measured, it is coded again.
 */
#define INTRA_BITS      12

static int skip_threshold(int qp)
{
    return (int)(0.57 * pow(2.0, (qp - 12) / 3.0) * INTRA_BITS);
}

/* one 8x8 CU at luma (x0, y0) */
static void encode_cu(Pic *p, Cabac *c, int x0, int y0, int qp, bool p_slice)
{
    int pred[64], x, y, comp;
    int lev_c[2], pred_c[2][16], r_c[2];
    const int cw = p->w >> 3, cu = (y0 >> 3) * cw + (x0 >> 3);
    const int cs = p->w / 2;

    /* the intra candidate: DC prediction, DC residual, luma 8x8 */
    pred_dc(p, p->y, p->w, 1, x0, y0, 8, true, pred);
    int sum = 0;
    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
            sum += p->cy[(y0 + y) * p->w + x0 + x] - pred[y * 8 + x];
        }
    }
    qp = p->qp;
    const int lev_y = dc_level((sum + (sum < 0 ? -32 : 32)) / 64, qp, 3);
    const int r_y = dc_residual(lev_y, qp, 3);

    /* chroma, 4x4 at (x0/2, y0/2) */
    const int qpc = chroma_qp(qp);
    for (comp = 0; comp < 2; comp++) {
        const uint8_t *plane = comp ? p->cr : p->cb;
        const uint8_t *src = comp ? p->ccr : p->ccb;
        pred_dc(p, (uint8_t *)plane, cs, 2, x0 / 2, y0 / 2, 4, false,
                pred_c[comp]);
        int t = 0;
        for (y = 0; y < 4; y++) {
            for (x = 0; x < 4; x++) {
                t += src[(y0 / 2 + y) * cs + x0 / 2 + x] -
                     pred_c[comp][y * 4 + x];
            }
        }
        lev_c[comp] = dc_level((t + (t < 0 ? -8 : 8)) / 16, qpc, 2);
        r_c[comp] = dc_residual(lev_c[comp], qpc, 2);
    }

    if (p_slice) {
        /* cu_skip_flag, its context from the left and upper CUs' */
        const int ctx = (x0 > 0 && p->skip[cu - 1]) +
                        (y0 > 0 && p->skip[cu - cw]);
        bool skip = false;

        if (p->ref) {
            int sse_i = 0, sse_s = 0;
            for (y = 0; y < 8; y++) {
                const int o = (y0 + y) * p->w + x0;
                for (x = 0; x < 8; x++) {
                    const int src = p->cy[o + x];
                    const int di = clip1(pred[y * 8 + x] + r_y) - src;
                    const int ds = p->ref->y[o + x] - src;
                    sse_i += di * di;
                    sse_s += ds * ds;
                }
            }
            for (comp = 0; comp < 2; comp++) {
                const uint8_t *src = comp ? p->ccr : p->ccb;
                const uint8_t *ref = comp ? p->ref->cr : p->ref->cb;
                for (y = 0; y < 4; y++) {
                    const int o = (y0 / 2 + y) * cs + x0 / 2;
                    for (x = 0; x < 4; x++) {
                        const int di = clip1(pred_c[comp][y * 4 + x] +
                                             r_c[comp]) - src[o + x];
                        const int ds = ref[o + x] - src[o + x];
                        sse_i += di * di;
                        sse_s += ds * ds;
                    }
                }
            }
            skip = sse_s <= sse_i + p->skip_thr;
        }
        if (skip) {
            /* merge candidate 0 (every candidate is the zero vector into
             * the one reference), no residual */
            cabac_decision(c, CTX_SKIP + ctx, 1);
            cabac_decision(c, CTX_MERGE_IDX, 0);
            for (y = 0; y < 8; y++) {
                memcpy(&p->y[(y0 + y) * p->w + x0],
                       &p->ref->y[(y0 + y) * p->w + x0], 8);
            }
            for (y = 0; y < 4; y++) {
                const int o = (y0 / 2 + y) * cs + x0 / 2;
                memcpy(&p->cb[o], &p->ref->cb[o], 4);
                memcpy(&p->cr[o], &p->ref->cr[o], 4);
            }
            p->skip[cu] = 1;
            p->cuqp[cu] = p->qp_delta_coded ? p->qp : p->qp_prev;
            p->done[cu] = 1;
            return;
        }
        /* not skipped, coded intra */
        cabac_decision(c, CTX_SKIP + ctx, 0);
        cabac_decision(c, CTX_PRED_MODE, 1);
    }

    /* part_mode 2Nx2N, prev_intra_luma_pred_flag 1, mpm_idx 1 (DC: the
     * neighbours are all DC, so the list is planar, DC, vertical) */
    cabac_decision(c, CTX_PART_MODE, 1);
    cabac_decision(c, CTX_PREV_INTRA, 1);
    cabac_bypass(c, 1);
    cabac_bypass(c, 0);
    cabac_decision(c, CTX_CHROMA_MODE, 0);              /* DM */

    /* transform_tree: no split; cbf_cb, cbf_cr, cbf_luma */
    cabac_decision(c, CTX_CBF_CHROMA + 0, lev_c[0] != 0);
    cabac_decision(c, CTX_CBF_CHROMA + 0, lev_c[1] != 0);
    cabac_decision(c, CTX_CBF_LUMA + 1, lev_y != 0);
    /* cu_qp_delta, once per quantization group (the CTB): from the last
     * group's QP (the slice's at first) to ours */
    if ((lev_y || lev_c[0] || lev_c[1]) && !p->qp_delta_coded) {
        cu_qp_delta(c, p->qp - p->qp_prev);
        p->qp_delta_coded = true;
    }
    if (lev_y) {
        residual_dc(c, lev_y, 3, false);
    }
    for (comp = 0; comp < 2; comp++) {
        if (lev_c[comp]) {
            residual_dc(c, lev_c[comp], 2, true);
        }
    }

    /* reconstruct */
    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
            p->y[(y0 + y) * p->w + x0 + x] = clip1(pred[y * 8 + x] + r_y);
        }
    }
    for (comp = 0; comp < 2; comp++) {
        uint8_t *plane = comp ? p->cr : p->cb;
        for (y = 0; y < 4; y++) {
            for (x = 0; x < 4; x++) {
                plane[(y0 / 2 + y) * cs + x0 / 2 + x] =
                    clip1(pred_c[comp][y * 4 + x] + r_c[comp]);
            }
        }
    }
    p->cuqp[cu] = p->qp_delta_coded ? p->qp : p->qp_prev;
    p->done[cu] = 1;
}

/* ---- parameter sets and NAL units ---------------------------------------- */

static void profile_tier_level(BitWriter *w, int level_idc)
{
    bw_bits(w, 0, 2);                       /* general_profile_space */
    bw_bits(w, 0, 1);                       /* general_tier_flag */
    bw_bits(w, 1, 5);                       /* Main */
    bw_bits(w, 0x60000000, 32);             /* compatible with 1, 2 */
    bw_bits(w, 0, 4);                       /* progressive, interlaced,
                                             * non_packed, frame_only: 0 */
    bw_bits(w, 0, 32);                      /* 43 reserved + 1 bits */
    bw_bits(w, 0, 12);
    bw_bits(w, level_idc, 8);
}

static size_t nal_wrap(uint8_t *out, size_t cap, int type, const uint8_t *rbsp,
                       size_t n)
{
    size_t o = 0, i;
    int zeros = 0;
#define PUT(b) do { if (o < cap) { out[o] = (b); } o++; } while (0)
    PUT(0); PUT(0); PUT(0); PUT(1);
    PUT(type << 1);                         /* forbidden 0, layer 0 */
    PUT(1);                                 /* temporal_id_plus1 */
    for (i = 0; i < n; i++) {
        if (zeros >= 2 && rbsp[i] <= 3) {
            PUT(3);
            zeros = 0;
        }
        PUT(rbsp[i]);
        zeros = rbsp[i] ? 0 : zeros + 1;
    }
#undef PUT
    return o;
}

enum { NAL_TRAIL_R = 1, NAL_IDR_W_RADL = 19, NAL_VPS = 32, NAL_SPS = 33,
       NAL_PPS = 34 };

/* emulation prevention over the slice data alone, as the engine writes it
 * after the slice header the library puts in front */
static size_t escape(uint8_t *out, size_t cap, const uint8_t *rbsp, size_t n)
{
    size_t o = 0, i;
    int zeros = 0;

    for (i = 0; i < n; i++) {
        if (zeros >= 2 && rbsp[i] <= 3) {
            if (o < cap) {
                out[o] = 3;
            }
            o++;
            zeros = 0;
        }
        if (o < cap) {
            out[o] = rbsp[i];
        }
        o++;
        zeros = rbsp[i] ? 0 : zeros + 1;
    }
    return o;
}

/* ---- deblocking (8.7.2), as the decoder does it ---------------------------
 * Every CU is 8x8 with one TU, so the edges are the 8x8 grid's.  bS is 2
 * where either side is intra, else 0: two skipped CUs have the same (zero)
 * vector into the same picture and no coefficients.  Vertical edges of the
 * whole picture first, then the horizontal ones. */

static const uint8_t beta_table[52] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 7, 8, 9, 10, 11, 12,
    13, 14, 15, 16, 17, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42,
    44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64
};

static const uint8_t tc_table[54] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 7, 8, 9, 10, 11,
    13, 14, 16, 18, 20, 22, 24
};

/* one luma edge segment of 4 lines: pix the q0 sample of line 0, xs the
 * step across the edge, ls the step along it (8.7.2.5.3, .6, .7) */
static void deblock_luma(uint8_t *pix, int xs, int ls, int beta, int tc)
{
#define P(k, i) pix[(k) * ls - ((i) + 1) * xs]
#define Q(k, i) pix[(k) * ls + (i) * xs]
    const int dp0 = abs(P(0, 2) - 2 * P(0, 1) + P(0, 0));
    const int dp3 = abs(P(3, 2) - 2 * P(3, 1) + P(3, 0));
    const int dq0 = abs(Q(0, 2) - 2 * Q(0, 1) + Q(0, 0));
    const int dq3 = abs(Q(3, 2) - 2 * Q(3, 1) + Q(3, 0));
    const int dpq0 = dp0 + dq0, dpq3 = dp3 + dq3;
    const int dp = dp0 + dp3, dq = dq0 + dq3;

    if (dpq0 + dpq3 >= beta) {
        return;
    }
    bool strong = true;
    for (int k = 0; k < 4; k += 3) {
        const int dpq = 2 * (k ? dpq3 : dpq0);
        if (!(dpq < (beta >> 2) &&
              abs(P(k, 3) - P(k, 0)) + abs(Q(k, 0) - Q(k, 3)) < (beta >> 3) &&
              abs(P(k, 0) - Q(k, 0)) < ((5 * tc + 1) >> 1))) {
            strong = false;
        }
    }
    const bool dep = dp < ((beta + (beta >> 1)) >> 3);
    const bool deq = dq < ((beta + (beta >> 1)) >> 3);

    for (int k = 0; k < 4; k++) {
        const int p0 = P(k, 0), p1 = P(k, 1), p2 = P(k, 2), p3 = P(k, 3);
        const int q0 = Q(k, 0), q1 = Q(k, 1), q2 = Q(k, 2), q3 = Q(k, 3);

        if (strong) {
            const int t2 = 2 * tc;
            P(k, 0) = MIN(MAX((p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3,
                              p0 - t2), p0 + t2);
            P(k, 1) = MIN(MAX((p2 + p1 + p0 + q0 + 2) >> 2, p1 - t2), p1 + t2);
            P(k, 2) = MIN(MAX((2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3,
                              p2 - t2), p2 + t2);
            Q(k, 0) = MIN(MAX((p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) >> 3,
                              q0 - t2), q0 + t2);
            Q(k, 1) = MIN(MAX((p0 + q0 + q1 + q2 + 2) >> 2, q1 - t2), q1 + t2);
            Q(k, 2) = MIN(MAX((p0 + q0 + q1 + 3 * q2 + 2 * q3 + 4) >> 3,
                              q2 - t2), q2 + t2);
            continue;
        }
        int d = (9 * (q0 - p0) - 3 * (q1 - p1) + 8) >> 4;
        if (abs(d) >= tc * 10) {
            continue;
        }
        d = MIN(MAX(d, -tc), tc);
        P(k, 0) = clip1(p0 + d);
        Q(k, 0) = clip1(q0 - d);
        if (dep) {
            const int dd = (((p2 + p0 + 1) >> 1) - p1 + d) >> 1;
            P(k, 1) = clip1(p1 + MIN(MAX(dd, -(tc >> 1)), tc >> 1));
        }
        if (deq) {
            const int dd = (((q2 + q0 + 1) >> 1) - q1 - d) >> 1;
            Q(k, 1) = clip1(q1 + MIN(MAX(dd, -(tc >> 1)), tc >> 1));
        }
    }
#undef P
#undef Q
}

/* one chroma edge segment of n lines (8.7.2.5.5) */
static void deblock_chroma(uint8_t *pix, int xs, int ls, int n, int tc)
{
    for (int k = 0; k < n; k++) {
        uint8_t *q = pix + k * ls;
        const int p0 = q[-xs], p1 = q[-2 * xs], q0 = q[0], q1 = q[xs];
        int d = (((q0 - p0) * 4) + p1 - q1 + 4) >> 3;

        d = MIN(MAX(d, -tc), tc);
        q[-xs] = clip1(p0 + d);
        q[0] = clip1(q0 - d);
    }
}

static void deblock(Pic *p, const HevcEncParams *prm)
{
    const int cw = p->w >> 3, ch = p->h >> 3, cs = p->w / 2;

    for (int dir = 0; dir < 2; dir++) {         /* 0 vertical edges */
        for (int by = dir; by < ch; by++) {
            for (int bx = !dir; bx < cw; bx++) {
                const int q = by * cw + bx, pp = dir ? q - cw : q - 1;
                if (p->skip[q] && p->skip[pp]) {
                    continue;                   /* bS 0 */
                }
                const int qpl = (p->cuqp[q] + p->cuqp[pp] + 1) >> 1;
                const int beta = beta_table[MIN(MAX(qpl + 2 *
                                         prm->beta_offset_div2, 0), 51)];
                const int tc = tc_table[MIN(MAX(qpl + 2 +
                                         2 * prm->tc_offset_div2, 0), 53)];
                const int xs = dir ? p->w : 1, ls = dir ? 1 : p->w;
                uint8_t *pix = &p->y[by * 8 * p->w + bx * 8];

                deblock_luma(pix, xs, ls, beta, tc);
                deblock_luma(pix + 4 * ls, xs, ls, beta, tc);
                /* chroma edges on the 8x8 chroma grid: luma 16 */
                if ((dir ? by : bx) & 1) {
                    continue;
                }
                const int qpc = chroma_qp(qpl);
                const int tcc = tc_table[MIN(MAX(qpc + 2 +
                                          2 * prm->tc_offset_div2, 0), 53)];
                const int o = by * 4 * cs + bx * 4;
                deblock_chroma(&p->cb[o], dir ? cs : 1, dir ? 1 : cs, 4, tcc);
                deblock_chroma(&p->cr[o], dir ? cs : 1, dir ? 1 : cs, 4, tcc);
            }
        }
    }
}

/* coding quadtree (7.3.8.4) from a CTB down to 8x8 CUs: split_cu_flag
 * coded inside the picture, inferred at its edges; z-scan order */
static void quadtree(Pic *p, Cabac *c, int x0, int y0, int log2, int qp,
                     bool p_slice)
{
    if (log2 == 3) {
        encode_cu(p, c, x0, y0, qp, p_slice);
        return;
    }
    if (x0 + (1 << log2) <= p->w && y0 + (1 << log2) <= p->h) {
        /* every neighbouring CU is 8x8, deeper than this node */
        cabac_decision(c, CTX_SPLIT_CU + (x0 > 0) + (y0 > 0), 1);
    }
    for (int k = 0; k < 4; k++) {
        const int x = x0 + (k & 1) * (1 << (log2 - 1));
        const int y = y0 + (k >> 1) * (1 << (log2 - 1));
        if (x < p->w && y < p->h) {
            quadtree(p, c, x, y, log2 - 1, qp, p_slice);
        }
    }
}

/*
 * The QP the CUs are coded with at most, whatever the slice's.  The vendor
 * libraries' rate control raises the slice QP as far as 44 (XM533's main
 * stream, within a GOP), where DC-only blocks go blotchy; and on a flat
 * picture even QP 30's DC steps (2.5 luma, 4.5 chroma levels) show as bands
 * where blocks were coded again in a P picture next to ones from the I
 * picture.  cu_qp_delta brings it down to where neither shows; skipped P
 * blocks keep the cost down.
 */
#define HEVC_QP_MAX     22

/* slice_segment_data() of a picture: CABAC from its first byte */
static size_t slice_data(const HevcEncParams *prm, int w, int h, int qp,
                         bool p_slice, const HevcEncRef *ref,
                         HevcEncRef *recon, const uint8_t *y, int sw, int sh,
                         int ys, const uint8_t *uv, int cs, uint8_t *rbsp,
                         size_t cap)
{
    const int ctb = 1 << prm->log2_ctb;
    int x, yy;
    Pic p = { .w = w, .h = h, .sy = y, .suv = uv, .sw = sw, .sh = sh,
              .sys = ys, .scs = cs, .qp = MIN(qp, HEVC_QP_MAX),
              .qp_prev = qp };
    g_autofree uint8_t *ry = g_malloc0((size_t)w * h);
    g_autofree uint8_t *rcb = g_malloc0((size_t)w * h / 4);
    g_autofree uint8_t *rcr = g_malloc0((size_t)w * h / 4);
    g_autofree uint8_t *done = g_malloc0((size_t)(w / 8) * (h / 8));
    g_autofree uint8_t *skip = g_malloc0((size_t)(w / 8) * (h / 8));
    g_autofree uint8_t *cuqp = g_malloc0((size_t)(w / 8) * (h / 8));
    g_autofree uint8_t *cy = g_malloc((size_t)w * h);
    g_autofree uint8_t *ccb = g_malloc((size_t)w * h / 4);
    g_autofree uint8_t *ccr = g_malloc((size_t)w * h / 4);
    p.y = ry;
    p.cb = rcb;
    p.cr = rcr;
    p.done = done;
    p.skip = skip;
    p.cuqp = cuqp;
    p.cy = cy;
    p.ccb = ccb;
    p.ccr = ccr;
    pic_maps(&p);
    for (yy = 0; yy < h; yy++) {
        for (x = 0; x < w; x++) {
            cy[yy * w + x] = src_y(&p, x, yy);
        }
    }
    for (yy = 0; yy < h / 2; yy++) {
        for (x = 0; x < w / 2; x++) {
            ccb[yy * (w / 2) + x] = src_c(&p, 0, x, yy);
            ccr[yy * (w / 2) + x] = src_c(&p, 1, x, yy);
        }
    }
    if (p_slice && ref && ref->w == w && ref->h == h) {
        p.ref = ref;
    }
    p.skip_thr = skip_threshold(p.qp);

    Cabac c;
    cabac_init(&c, qp, p_slice ? 1 : 0, rbsp, rbsp + cap);
    const int ctbw = (w + ctb - 1) / ctb, ctbh = (h + ctb - 1) / ctb;
    for (yy = 0; yy < ctbh; yy++) {
        for (x = 0; x < ctbw; x++) {
            if (prm->sao) {
                /* sao(): not merged, type 0 (off) for luma and chroma */
                if (x > 0) {
                    cabac_decision(&c, CTX_SAO_MERGE, 0);
                }
                if (yy > 0) {
                    cabac_decision(&c, CTX_SAO_MERGE, 0);
                }
                cabac_decision(&c, CTX_SAO_TYPE, 0);
                cabac_decision(&c, CTX_SAO_TYPE, 0);
            }
            p.qp_delta_coded = false;
            quadtree(&p, &c, x * ctb, yy * ctb, prm->log2_ctb, qp, p_slice);
            if (p.qp_delta_coded) {
                p.qp_prev = p.qp;
            }
            if (x == ctbw - 1 && yy == ctbh - 1) {
                cabac_flush(&c);
            } else {
                cabac_terminate0(&c);
            }
        }
    }
    pic_maps_free(&p);
    if (recon) {
        /* the in-loop filter, then keep the picture as the next one's
         * reference */
        deblock(&p, prm);
        if (recon->w != w || recon->h != h) {
            hevc_enc_ref_clear(recon);
            recon->w = w;
            recon->h = h;
            recon->y = g_malloc((size_t)w * h);
            recon->cb = g_malloc((size_t)w * h / 4);
            recon->cr = g_malloc((size_t)w * h / 4);
        }
        memcpy(recon->y, ry, (size_t)w * h);
        memcpy(recon->cb, rcb, (size_t)w * h / 4);
        memcpy(recon->cr, rcr, (size_t)w * h / 4);
    }
    return MIN((size_t)(c.p - rbsp), cap);
}

static void hevc_enc_ref_clear(HevcEncRef *r)
{
    g_free(r->y);
    g_free(r->cb);
    g_free(r->cr);
    *r = (HevcEncRef){ 0 };
}

HevcEncRef *hevc_enc_ref_new(void)
{
    return g_new0(HevcEncRef, 1);
}

void hevc_enc_ref_free(HevcEncRef *r)
{
    if (r) {
        hevc_enc_ref_clear(r);
        g_free(r);
    }
}

static bool size_ok(int w, int h)
{
    return w > 0 && h > 0 && !(w & 7) && !(h & 7) && w <= 8192 && h <= 8192;
}

const HevcEncParams hevc_params_ctb16 = {
    .log2_ctb = 4, .sao = false, .beta_offset_div2 = 5, .tc_offset_div2 = -2,
    .vui = false,
};

const HevcEncParams hevc_params_ctb32_sao = {
    .log2_ctb = 5, .sao = true, .beta_offset_div2 = 5, .tc_offset_div2 = -5,
    .vui = true,
};

size_t hevc_enc_slice_data(const HevcEncParams *prm,
                           int w, int h, int qp, bool p_slice,
                           const HevcEncRef *ref, HevcEncRef *recon,
                           const uint8_t *y, int sw, int sh, int ys,
                           const uint8_t *uv, int cs,
                           uint8_t *out, size_t cap)
{
    if (!size_ok(w, h)) {
        return 0;
    }
    const size_t rbsp_cap = (size_t)(w / 8) * (h / 8) * 48 + 256;
    g_autofree uint8_t *rbsp = g_malloc0(rbsp_cap);
    const size_t n = slice_data(prm, w, h, MIN(MAX(qp, 0), 51), p_slice,
                                ref, recon, y, sw, sh, ys, uv, cs, rbsp,
                                rbsp_cap);
    return escape(out, cap, rbsp, n);
}

size_t hevc_enc_frame(const HevcEncParams *prm,
                      int w, int h, int qp, bool p_slice, int poc,
                      const HevcEncRef *ref, HevcEncRef *recon,
                      const uint8_t *y, int sw, int sh, int ys,
                      const uint8_t *uv, int cs,
                      uint8_t *out, size_t cap)
{
    uint8_t hdr[128];
    size_t o = 0;
    BitWriter b;

    if (!size_ok(w, h)) {
        return 0;
    }
    qp = MIN(MAX(qp, 0), 51);
    /* the parameter sets Sofia's jc_v200 library itself sends (decoded
     * from its RTSP stream): it codes skip P slices against them, so ours
     * must carry the same fields -- one short-term RPS, 10-bit POC LSBs,
     * cabac_init_present, cu_qp_delta, deblocking with slice override */
    const int level_idc = 63;

    if (p_slice) {
        goto slice;
    }
    /* VPS */
    b = (BitWriter){ .p = hdr, .cap = sizeof(hdr) };
    bw_bits(&b, 0, 4);                      /* vps id */
    bw_bits(&b, 3, 2);                      /* base layer internal, available */
    bw_bits(&b, 0, 6);                      /* max_layers_minus1 */
    bw_bits(&b, 0, 3);                      /* max_sub_layers_minus1 */
    bw_bits(&b, 1, 1);                      /* temporal_id_nesting */
    bw_bits(&b, 0xffff, 16);
    profile_tier_level(&b, level_idc);
    bw_bits(&b, 1, 1);                      /* sub_layer_ordering_info */
    bw_ue(&b, 1);                           /* max_dec_pic_buffering - 1 */
    bw_ue(&b, 0);
    bw_ue(&b, 0);
    bw_bits(&b, 0, 6);                      /* max_layer_id */
    bw_ue(&b, 0);                           /* num_layer_sets_minus1 */
    bw_bits(&b, 1, 1);                      /* timing_info */
    bw_bits(&b, 1000, 32);                  /* num_units_in_tick */
    bw_bits(&b, 12000, 32);                 /* time_scale: 12 fps */
    bw_bits(&b, 0, 1);                      /* poc_proportional_to_timing */
    bw_ue(&b, 0);                           /* num_hrd_parameters */
    bw_bits(&b, 0, 1);                      /* extension */
    bw_trailing(&b);
    o += nal_wrap(out + MIN(o, cap), cap > o ? cap - o : 0, NAL_VPS, hdr, b.n);

    /* SPS */
    b = (BitWriter){ .p = hdr, .cap = sizeof(hdr) };
    bw_bits(&b, 0, 4);                      /* vps id */
    bw_bits(&b, 0, 3);                      /* max_sub_layers_minus1 */
    bw_bits(&b, 1, 1);                      /* temporal_id_nesting */
    profile_tier_level(&b, level_idc);
    bw_ue(&b, 0);                           /* sps id */
    bw_ue(&b, 1);                           /* 4:2:0 */
    bw_ue(&b, w);
    bw_ue(&b, h);
    bw_bits(&b, 1, 1);                      /* conformance_window, */
    bw_ue(&b, 0);                           /*   all offsets 0 */
    bw_ue(&b, 0);
    bw_ue(&b, 0);
    bw_ue(&b, 0);
    bw_ue(&b, 0);                           /* bit depths 8 */
    bw_ue(&b, 0);
    bw_ue(&b, 6);                           /* log2_max_poc_lsb 10 */
    bw_bits(&b, 1, 1);                      /* sub_layer_ordering_info */
    bw_ue(&b, 1);
    bw_ue(&b, 0);
    bw_ue(&b, 0);
    bw_ue(&b, 0);                           /* min CB 8 */
    bw_ue(&b, prm->log2_ctb - 3);           /* CTB 16 or 32 */
    bw_ue(&b, 0);                           /* min TB 4 */
    bw_ue(&b, prm->log2_ctb - 2);           /* max TB: the CTB */
    bw_ue(&b, 0);                           /* hierarchy depth inter */
    bw_ue(&b, 0);                           /* hierarchy depth intra */
    bw_bits(&b, 0, 1);                      /* scaling lists */
    bw_bits(&b, 0, 1);                      /* amp */
    bw_bits(&b, prm->sao, 1);               /* sao */
    bw_bits(&b, 0, 1);                      /* pcm */
    bw_ue(&b, 1);                           /* one short-term RPS: */
    bw_ue(&b, 1);                           /*   one negative picture, */
    bw_ue(&b, 0);                           /*   no positive ones, */
    bw_ue(&b, 0);                           /*   delta POC -1, */
    bw_bits(&b, 1, 1);                      /*   used by the current one */
    bw_bits(&b, 0, 1);                      /* long-term refs */
    bw_bits(&b, 0, 1);                      /* temporal mvp */
    bw_bits(&b, 0, 1);                      /* strong intra smoothing */
    bw_bits(&b, prm->vui, 1);               /* vui: */
    if (prm->vui) {
        bw_bits(&b, 0, 8);                  /*   aspect ratio .. display window */
        bw_bits(&b, 1, 1);                  /*   timing info */
        bw_bits(&b, 1000, 32);
        bw_bits(&b, 12000, 32);
        bw_bits(&b, 0, 1);                  /*   poc proportional */
        bw_bits(&b, 0, 1);                  /*   hrd */
        bw_bits(&b, 0, 1);                  /*   bitstream restriction */
    }
    bw_bits(&b, 0, 1);                      /* extension */
    bw_trailing(&b);
    o += nal_wrap(out + MIN(o, cap), cap > o ? cap - o : 0, NAL_SPS, hdr, b.n);

    /* PPS */
    b = (BitWriter){ .p = hdr, .cap = sizeof(hdr) };
    bw_ue(&b, 0);                           /* pps id */
    bw_ue(&b, 0);                           /* sps id */
    bw_bits(&b, 0, 1);                      /* dependent slices */
    bw_bits(&b, 0, 1);                      /* output_flag_present */
    bw_bits(&b, 0, 3);                      /* extra slice header bits */
    bw_bits(&b, 0, 1);                      /* sign data hiding */
    bw_bits(&b, 1, 1);                      /* cabac_init_present */
    bw_ue(&b, 0);
    bw_ue(&b, 0);
    bw_se(&b, 0);                           /* init_qp 26 */
    bw_bits(&b, 0, 1);                      /* constrained intra */
    bw_bits(&b, 0, 1);                      /* transform skip */
    bw_bits(&b, 1, 1);                      /* cu_qp_delta, */
    bw_ue(&b, 0);                           /*   per CTB */
    bw_se(&b, 0);
    bw_se(&b, 0);
    bw_bits(&b, 0, 1);                      /* slice chroma qp offsets */
    bw_bits(&b, 0, 1);                      /* weighted pred */
    bw_bits(&b, 0, 1);                      /* weighted bipred */
    bw_bits(&b, 0, 1);                      /* transquant bypass */
    bw_bits(&b, 0, 1);                      /* tiles */
    bw_bits(&b, 0, 1);                      /* entropy sync */
    bw_bits(&b, 1, 1);                      /* loop filter across slices */
    bw_bits(&b, 1, 1);                      /* deblocking control */
    bw_bits(&b, 1, 1);                      /*   override enabled */
    bw_bits(&b, 0, 1);                      /*   disabled */
    bw_se(&b, prm->beta_offset_div2);       /*   beta_offset_div2 */
    bw_se(&b, prm->tc_offset_div2);         /*   tc_offset_div2 */
    bw_bits(&b, 0, 1);                      /* scaling list data */
    bw_bits(&b, 0, 1);                      /* lists modification */
    bw_ue(&b, 0);                           /* parallel merge level */
    bw_bits(&b, 0, 1);                      /* slice header extension */
    bw_bits(&b, 0, 1);                      /* extension */
    bw_trailing(&b);
    o += nal_wrap(out + MIN(o, cap), cap > o ? cap - o : 0, NAL_PPS, hdr, b.n);

slice:
    /* the slice header the library writes (decoded from its stream), then
     * the slice data */
    b = (BitWriter){ .p = hdr, .cap = sizeof(hdr) };
    bw_bits(&b, 1, 1);                      /* first_slice_segment_in_pic */
    if (!p_slice) {
        bw_bits(&b, 0, 1);                  /* no_output_of_prior_pics */
    }
    bw_ue(&b, 0);                           /* pps id */
    bw_ue(&b, p_slice ? 1 : 2);             /* P / I */
    if (p_slice) {
        bw_bits(&b, poc & 1023, 10);        /* pic_order_cnt_lsb */
        bw_bits(&b, 1, 1);                  /* the SPS's RPS */
    }
    if (prm->sao) {
        bw_bits(&b, 3, 2);                  /* slice_sao_luma / chroma */
    }
    if (p_slice) {
        bw_bits(&b, 0, 1);                  /* num_ref_idx_active_override */
        bw_bits(&b, 0, 1);                  /* cabac_init_flag */
        bw_ue(&b, 1);                       /* five_minus_max_num_merge_cand */
    }
    bw_se(&b, qp - 26);                     /* slice_qp_delta */
    bw_bits(&b, 1, 1);                      /* deblocking override: */
    bw_bits(&b, 0, 1);                      /*   on, */
    bw_se(&b, prm->beta_offset_div2);       /*   beta_offset_div2 */
    bw_se(&b, prm->tc_offset_div2);         /*   tc_offset_div2 */
    bw_bits(&b, 1, 1);                      /* loop filter across slices */
    bw_bits(&b, 1, 1);                      /* byte_alignment() */
    while (b.bits) {
        bw_bits(&b, 0, 1);
    }
    /* NAL: header bytes, then the escaped slice data */
    {
        const int type = p_slice ? NAL_TRAIL_R : NAL_IDR_W_RADL;
        const size_t hl = nal_wrap(out + MIN(o, cap), cap > o ? cap - o : 0,
                                   type, hdr, b.n);
        o += hl;
        o += hevc_enc_slice_data(prm, w, h, qp, p_slice, ref, recon, y, sw,
                                 sh, ys, uv, cs,
                                 out + MIN(o, cap), cap > o ? cap - o : 0);
    }
    return o;
}
