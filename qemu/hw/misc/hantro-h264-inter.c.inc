/*
 * P slices with real inter coding, for the engines whose library writes
 * the slice header (included by hantro-h264.c, whose CABAC engine,
 * residual coding and intra DC macroblocks it uses).
 *
 * Each macroblock is P_Skip, P_L0_16x16 or the I_16x16 DC macroblock of
 * the I slices:
 *
 *  - motion: quarter-pel luma vectors, found by testing the predicted,
 *    skip, neighbours' and zero vectors at whole samples, then (if none is
 *    close) a 4-pixel grid over +-16 pixels, a diamond search, and a
 *    refinement to half and quarter samples; luma predicted by the 6-tap
 *    and averaging interpolation of 8.4.2.2.1; one reference (the previous
 *    picture: num_ref_idx_l0_active 1, so ref_idx is not sent); the
 *    16x16 vector prediction and the P_Skip vector of 8.4.1.1 / 8.4.1.3;
 *    chroma at eighth-pel by the bilinear filter of 8.4.2.2.2;
 *  - residual: the 4x4 integer transform of every luma block, quantised
 *    with inter rounding, chroma DC (2x2) and AC; coded_block_pattern,
 *    transform_size_8x8_flag (always 0, when the PPS allows the flag),
 *    mb_qp_delta (non-zero only on the first coded macroblock of a picture
 *    re-encoded at a higher QP to fit the bitstream buffer);
 *  - P_Skip where the skip vector's prediction leaves no coefficient,
 *    I_16x16 DC where the motion search finds nothing close (a scene
 *    change).
 *
 * The reconstruction is the decoder's (8.4.2.2, 8.5.12, 8.5.14): intra
 * prediction reads the picture before deblocking, and the picture kept as
 * the next reference is deblocked as the slice header asks (8.7; frame
 * macroblocks, 4x4 transforms, one slice per picture), so prediction never
 * drifts.  CABAC only: the contexts are initialised for the slice's
 * cabac_init_idc (h264_cabac_init_P).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* ---- tables ------------------------------------------------------------- */

/* quantiser multipliers and dequantiser scales by QP % 6, for the three
 * classes of 4x4 position: (even, even), (odd, odd), the rest */
static const int q_mf[6][3] = {
    { 13107, 5243, 8066 }, { 11916, 4660, 7490 }, { 10082, 4194, 6554 },
    { 9362, 3647, 5825 }, { 8192, 3355, 5243 }, { 7282, 2893, 4559 },
};
static const int q_v[6][3] = {
    { 10, 16, 13 }, { 11, 18, 14 }, { 13, 20, 16 },
    { 14, 23, 18 }, { 16, 25, 20 }, { 18, 29, 23 },
};

static const uint8_t db_alpha[52] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 5, 6, 7, 8, 9, 10,
    12, 13, 15, 17, 20, 22, 25, 28, 32, 36, 40, 45, 50, 56, 63, 71, 80, 90,
    101, 113, 127, 144, 162, 182, 203, 226, 255, 255,
};
static const uint8_t db_beta[52] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 3, 3, 3, 3, 4,
    4, 4, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14,
    15, 15, 16, 16, 17, 17, 18, 18,
};
static const uint8_t db_tc0[52][3] = {
    { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 },
    { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 },
    { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 },
    { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 1 }, { 0, 0, 1 }, { 0, 0, 1 },
    { 0, 0, 1 }, { 0, 1, 1 }, { 0, 1, 1 }, { 1, 1, 1 }, { 1, 1, 1 },
    { 1, 1, 1 }, { 1, 1, 1 }, { 1, 1, 2 }, { 1, 1, 2 }, { 1, 1, 2 },
    { 1, 1, 2 }, { 1, 2, 3 }, { 1, 2, 3 }, { 2, 2, 3 }, { 2, 2, 4 },
    { 2, 3, 4 }, { 2, 3, 4 }, { 3, 3, 5 }, { 3, 4, 6 }, { 3, 4, 6 },
    { 4, 5, 7 }, { 4, 5, 8 }, { 4, 6, 9 }, { 5, 7, 10 }, { 6, 8, 11 },
    { 6, 8, 13 }, { 7, 10, 14 }, { 8, 11, 16 }, { 9, 12, 18 },
    { 10, 13, 20 }, { 11, 15, 23 }, { 13, 17, 25 },
};

static int q_class(int pos)
{
    const int i = pos >> 2, j = pos & 3;

    return !(i & 1) && !(j & 1) ? 0 : (i & 1) && (j & 1) ? 1 : 2;
}

/* ---- the picture and its macroblocks ------------------------------------ */

enum { PMB_SKIP, PMB_INTER, PMB_INTRA };

typedef struct PMb {
    uint8_t type;
    uint8_t qp;                 /* QP_Y */
    int16_t mv[2];              /* quarter-pel (skip: the skip vector) */
    int16_t mvd[2];
    uint8_t cbp_luma, cbp_chroma;
    uint16_t nz;                /* luma 4x4 blocks with coefficients */
    uint8_t cdc;                /* chroma DC coded_block_flag: Cb 1, Cr 2 */
    uint8_t cac;                /* chroma AC: Cb blocks bits 0-3, Cr 4-7 */
    bool luma_dc_cbf;           /* I_16x16's Intra16x16DCLevel */
    bool qp_delta_nz;
} PMb;

typedef struct PPic {
    Frame f;                    /* reconstruction (y, cb, cr) and source */
    const uint8_t *ry, *rcb, *rcr;  /* reference, deblocked */
    int ry0, rh;                /* the reference rows of f's row 0, and the
                                 * reference's height (a slice of a picture
                                 * is a window: f and mb from its first row) */
    bool force_intra;           /* no reference: every macroblock intra */
    bool force_skip;            /* no room left: every macroblock P_Skip */
    PMb *mb;
    int qp_slice, qp_target, qp_cur, cqp;
    bool t8x8;                  /* transform_8x8_mode_flag */
    int last_qp_delta_nz;       /* the previous macroblock's, for ctx 60 */
} PPic;

static inline int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static inline uint8_t ref_y(const PPic *p, int x, int y)
{
    return p->ry[clampi(y + p->ry0, 0, p->rh - 1) * p->f.w +
                 clampi(x, 0, p->f.w - 1)];
}

/*
 * Luma sample interpolation (8.4.2.2.1): the half samples by the 6-tap
 * filter (1, -5, 20, 20, -5, 1) -- b, h rounded >> 5 from their sums, j
 * from the sums of the sums >> 10 -- and the quarter samples as the
 * rounded-up average of the two nearest integer / half samples.
 */
static inline int tap6(int e, int f, int g, int h, int i, int j)
{
    return e - 5 * f + 20 * g + 20 * h - 5 * i + j;
}

/* b1: the sum for the half sample right of integer (x, y) */
static inline int b1_at(const PPic *p, int x, int y)
{
    return tap6(ref_y(p, x - 2, y), ref_y(p, x - 1, y), ref_y(p, x, y),
                ref_y(p, x + 1, y), ref_y(p, x + 2, y), ref_y(p, x + 3, y));
}

static inline int clip255(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* b: the half sample right of (x, y); h: below it; j: right and below */
static inline int hb(const PPic *p, int x, int y)
{
    return clip255((b1_at(p, x, y) + 16) >> 5);
}

static inline int hh(const PPic *p, int x, int y)
{
    return clip255((tap6(ref_y(p, x, y - 2), ref_y(p, x, y - 1),
                         ref_y(p, x, y), ref_y(p, x, y + 1),
                         ref_y(p, x, y + 2), ref_y(p, x, y + 3)) + 16) >> 5);
}

static inline int hj(const PPic *p, int x, int y)
{
    return clip255((tap6(b1_at(p, x, y - 2), b1_at(p, x, y - 1),
                         b1_at(p, x, y), b1_at(p, x, y + 1),
                         b1_at(p, x, y + 2), b1_at(p, x, y + 3)) + 512) >> 10);
}

/* the luma prediction sample at integer (x, y) plus (fx, fy) quarters */
static int luma_q(const PPic *p, int x, int y, int fx, int fy)
{
    switch (fy * 4 + fx) {
    case 0:  return ref_y(p, x, y);                                 /* G */
    case 1:  return (ref_y(p, x, y) + hb(p, x, y) + 1) >> 1;        /* a */
    case 2:  return hb(p, x, y);                                    /* b */
    case 3:  return (ref_y(p, x + 1, y) + hb(p, x, y) + 1) >> 1;    /* c */
    case 4:  return (ref_y(p, x, y) + hh(p, x, y) + 1) >> 1;        /* d */
    case 5:  return (hb(p, x, y) + hh(p, x, y) + 1) >> 1;           /* e */
    case 6:  return (hb(p, x, y) + hj(p, x, y) + 1) >> 1;           /* f */
    case 7:  return (hb(p, x, y) + hh(p, x + 1, y) + 1) >> 1;       /* g */
    case 8:  return hh(p, x, y);                                    /* h */
    case 9:  return (hh(p, x, y) + hj(p, x, y) + 1) >> 1;           /* i */
    case 10: return hj(p, x, y);                                    /* j */
    case 11: return (hj(p, x, y) + hh(p, x + 1, y) + 1) >> 1;       /* k */
    case 12: return (ref_y(p, x, y + 1) + hh(p, x, y) + 1) >> 1;    /* n */
    case 13: return (hh(p, x, y) + hb(p, x, y + 1) + 1) >> 1;       /* p */
    case 14: return (hj(p, x, y) + hb(p, x, y + 1) + 1) >> 1;       /* q */
    default: return (hh(p, x + 1, y) + hb(p, x, y + 1) + 1) >> 1;   /* r */
    }
}

/* the 16x16 luma prediction by quarter-pel vector mv */
static void mc_luma(const PPic *p, int mx, int my, const int16_t mv[2],
                    uint8_t py[256])
{
    const int dx = mv[0] >> 2, dy = mv[1] >> 2, fx = mv[0] & 3,
              fy = mv[1] & 3;
    int x, y;

    for (y = 0; y < 16; y++) {
        for (x = 0; x < 16; x++) {
            const int X = mx * 16 + x + dx, Y = my * 16 + y + dy;
            py[y * 16 + x] = fx | fy ? luma_q(p, X, Y, fx, fy)
                                     : ref_y(p, X, Y);
        }
    }
}

/* motion-compensated luma (quarter-pel) and chroma (eighth-pel) prediction */
static void mc_mb(const PPic *p, int mx, int my, const int16_t mv[2],
                  uint8_t py[256], uint8_t pc[2][64])
{
    const int cw = p->f.w / 2, ch = p->rh / 2;
    const int fx = mv[0] & 7, fy = mv[1] & 7;
    const int ix = mv[0] >> 3, iy = mv[1] >> 3;
    int x, y, comp;

    mc_luma(p, mx, my, mv, py);
    for (comp = 0; comp < 2; comp++) {
        const uint8_t *r = comp ? p->rcr : p->rcb;

        for (y = 0; y < 8; y++) {
            for (x = 0; x < 8; x++) {
                const int x0 = mx * 8 + x + ix;
                const int y0 = my * 8 + y + iy + p->ry0 / 2;
                const int xa = clampi(x0, 0, cw - 1);
                const int xb = clampi(x0 + 1, 0, cw - 1);
                const int ya = clampi(y0, 0, ch - 1);
                const int yb = clampi(y0 + 1, 0, ch - 1);

                pc[comp][y * 8 + x] =
                    ((8 - fx) * (8 - fy) * r[ya * cw + xa] +
                     fx * (8 - fy) * r[ya * cw + xb] +
                     (8 - fx) * fy * r[yb * cw + xa] +
                     fx * fy * r[yb * cw + xb] + 32) >> 6;
            }
        }
    }
}

/* full-pel luma SAD (the integer search) */
static int sad_mb(const PPic *p, int mx, int my, int dx, int dy, int best)
{
    int x, y, s = 0;

    for (y = 0; y < 16; y++) {
        const uint8_t *src = p->f.sy + (my * 16 + y) * p->f.sys + mx * 16;

        for (x = 0; x < 16; x++) {
            s += abs(src[x] - ref_y(p, mx * 16 + x + dx, my * 16 + y + dy));
        }
        if (s >= best) {
            return s;
        }
    }
    return s;
}

/* the luma SAD of a quarter-pel vector's prediction (the refinement) */
static int sad_mb_q(const PPic *p, int mx, int my, const int16_t mv[2])
{
    uint8_t py[256];
    int x, y, s = 0;

    mc_luma(p, mx, my, mv, py);
    for (y = 0; y < 16; y++) {
        const uint8_t *src = p->f.sy + (my * 16 + y) * p->f.sys + mx * 16;

        for (x = 0; x < 16; x++) {
            s += abs(src[x] - py[y * 16 + x]);
        }
    }
    return s;
}

/* SAD left after each 4x4 block's mean difference from the prediction py
 * is removed: what an inter macroblock with only DC residuals would still
 * miss */
static int mrsad_mb(const PPic *p, int mx, int my, const uint8_t py[256])
{
    int bx, by, x, y, e = 0;

    for (by = 0; by < 4; by++) {
        for (bx = 0; bx < 4; bx++) {
            int d[16], m = 0;

            for (y = 0; y < 4; y++) {
                for (x = 0; x < 4; x++) {
                    const int px = mx * 16 + bx * 4 + x;
                    const int pyy = my * 16 + by * 4 + y;

                    d[y * 4 + x] = p->f.sy[pyy * p->f.sys + px] -
                                   py[(by * 4 + y) * 16 + bx * 4 + x];
                    m += d[y * 4 + x];
                }
            }
            m = m >= 0 ? (m + 8) >> 4 : -((-m + 8) >> 4);
            for (x = 0; x < 16; x++) {
                e += abs(d[x] - m);
            }
        }
    }
    return e;
}

/* ---- neighbours and vector prediction ------------------------------------ */

static PMb *pmb_at(PPic *p, int mx, int my)
{
    if (mx < 0 || my < 0 || mx >= p->f.mbw || my >= p->f.mbh) {
        return NULL;
    }
    return &p->mb[my * p->f.mbw + mx];
}

/* refIdx (0 or -1) and vector of a neighbouring 16x16 partition */
static int nb_ref(const PMb *n, int16_t mv[2])
{
    if (!n || n->type == PMB_INTRA) {
        mv[0] = mv[1] = 0;
        return -1;
    }
    mv[0] = n->mv[0];
    mv[1] = n->mv[1];
    return 0;
}

static int median3(int a, int b, int c)
{
    return MAX(MIN(a, b), MIN(MAX(a, b), c));
}

/* 8.4.1.3: the predicted vector of a 16x16 partition with refIdx 0 */
static void mv_pred(PPic *p, int mx, int my, int16_t mvp[2])
{
    PMb *a = pmb_at(p, mx - 1, my), *b = pmb_at(p, mx, my - 1);
    PMb *c = pmb_at(p, mx + 1, my - 1);
    int16_t va[2], vb[2], vc[2];
    int ra, rb, rc;

    if (!c) {
        c = pmb_at(p, mx - 1, my - 1);          /* D for C */
    }
    ra = nb_ref(a, va);
    rb = nb_ref(b, vb);
    rc = nb_ref(c, vc);
    if (!b && !c && a) {
        vb[0] = vc[0] = va[0];
        vb[1] = vc[1] = va[1];
        rb = rc = ra;
    }
    if ((ra == 0) + (rb == 0) + (rc == 0) == 1) {
        const int16_t *v = ra == 0 ? va : rb == 0 ? vb : vc;

        mvp[0] = v[0];
        mvp[1] = v[1];
        return;
    }
    mvp[0] = median3(va[0], vb[0], vc[0]);
    mvp[1] = median3(va[1], vb[1], vc[1]);
}

/* 8.4.1.1: P_Skip's vector */
static void mv_skip(PPic *p, int mx, int my, int16_t mv[2])
{
    PMb *a = pmb_at(p, mx - 1, my), *b = pmb_at(p, mx, my - 1);
    int16_t va[2], vb[2];

    if (!a || !b) {
        mv[0] = mv[1] = 0;
        return;
    }
    if ((nb_ref(a, va) == 0 && !va[0] && !va[1]) ||
        (nb_ref(b, vb) == 0 && !vb[0] && !vb[1])) {
        mv[0] = mv[1] = 0;
        return;
    }
    mv_pred(p, mx, my, mv);
}

/* ---- transform and quantisation -------------------------------------------- */

static void fdct4(const int *r, int stride, int out[16])
{
    int t[16], i;

    for (i = 0; i < 4; i++) {
        const int *s = r + i * stride;
        const int a = s[0] + s[3], b = s[1] + s[2];
        const int c = s[1] - s[2], d = s[0] - s[3];

        t[i * 4 + 0] = a + b;
        t[i * 4 + 1] = 2 * d + c;
        t[i * 4 + 2] = a - b;
        t[i * 4 + 3] = d - 2 * c;
    }
    for (i = 0; i < 4; i++) {
        const int a = t[i] + t[12 + i], b = t[4 + i] + t[8 + i];
        const int c = t[4 + i] - t[8 + i], d = t[i] - t[12 + i];

        out[i] = a + b;
        out[4 + i] = 2 * d + c;
        out[8 + i] = a - b;
        out[12 + i] = d - 2 * c;
    }
}

static int quant_inter(int v, int qp, int cls, int extra)
{
    const int qbits = 15 + qp / 6 + extra;
    const int a = (abs(v) * q_mf[qp % 6][cls] + ((1 << qbits) / 6)) >> qbits;

    return v < 0 ? -a : a;
}

/* 8.5.12.1, flat scaling matrices: d for a coefficient other than a DC
 * that went through its own transform */
static int dequant4(int c, int qp, int pos)
{
    const int ls = 16 * q_v[qp % 6][q_class(pos)];

    if (qp >= 24) {
        return (c * ls) << (qp / 6 - 4);
    }
    return (c * ls + (1 << (3 - qp / 6))) >> (4 - qp / 6);
}

/* 8.5.12.2: the inverse transform of d (raster), added to pred, clipped */
static void idct4_add(const int d[16], uint8_t *dst, int stride,
                      const uint8_t *pred, int pstride)
{
    int t[16], i, j;

    for (i = 0; i < 4; i++) {
        const int *s = d + 4 * i;
        const int e0 = s[0] + s[2], e1 = s[0] - s[2];
        const int e2 = (s[1] >> 1) - s[3], e3 = s[1] + (s[3] >> 1);

        t[4 * i + 0] = e0 + e3;
        t[4 * i + 1] = e1 + e2;
        t[4 * i + 2] = e1 - e2;
        t[4 * i + 3] = e0 - e3;
    }
    for (j = 0; j < 4; j++) {
        const int e0 = t[j] + t[8 + j], e1 = t[j] - t[8 + j];
        const int e2 = (t[4 + j] >> 1) - t[12 + j];
        const int e3 = t[4 + j] + (t[12 + j] >> 1);
        const int h[4] = { e0 + e3, e1 + e2, e1 - e2, e0 - e3 };

        for (i = 0; i < 4; i++) {
            dst[i * stride + j] = clip1(pred[i * pstride + j] +
                                        ((h[i] + 32) >> 6));
        }
    }
}

/* the residual of a P macroblock against pred, quantised */
typedef struct PRes {
    int luma[16][16];           /* per block (raster), raster coefficients */
    int cdc[2][4];
    int cac[2][4][16];          /* raster, [0] unused */
    uint8_t cbp_luma, cbp_chroma;
    uint16_t nz;
} PRes;

static void residual_mb(PPic *p, int mx, int my, const uint8_t py[256],
                        const uint8_t pc[2][64], int qp, PRes *r)
{
    const int qpc = chroma_qp(qp, p->cqp);
    int diff[256], cd[64], coef[16];
    int b, i, x, y, comp;

    memset(r, 0, sizeof(*r));
    for (y = 0; y < 16; y++) {
        for (x = 0; x < 16; x++) {
            diff[y * 16 + x] = src_y(&p->f, mx * 16 + x, my * 16 + y) -
                               py[y * 16 + x];
        }
    }
    for (b = 0; b < 16; b++) {
        const int bx = b & 3, by = b >> 2;
        int sum = 0;

        fdct4(diff + by * 4 * 16 + bx * 4, 16, coef);
        for (i = 0; i < 16; i++) {
            r->luma[b][i] = quant_inter(coef[i], qp, q_class(i), 0);
            sum |= r->luma[b][i];
        }
        if (sum) {
            r->nz |= 1 << b;
            r->cbp_luma |= 1 << ((by >> 1) * 2 + (bx >> 1));
        }
    }
    /* an 8x8 with no coefficient but a lone +-1 here and there costs more
     * than it brings: drop 8x8s whose coefficients are all +-1 and few */
    for (b = 0; b < 4; b++) {
        int ones = 0, big = 0, k;

        if (!(r->cbp_luma & (1 << b))) {
            continue;
        }
        for (k = 0; k < 4; k++) {
            const int blk = ((b >> 1) * 2 + (k >> 1)) * 4 + (b & 1) * 2 +
                            (k & 1);

            for (i = 0; i < 16; i++) {
                ones += abs(r->luma[blk][i]) == 1;
                big += abs(r->luma[blk][i]) > 1;
            }
        }
        if (!big && ones <= 2) {
            for (k = 0; k < 4; k++) {
                const int blk = ((b >> 1) * 2 + (k >> 1)) * 4 + (b & 1) * 2 +
                                (k & 1);

                memset(r->luma[blk], 0, sizeof(r->luma[blk]));
                r->nz &= ~(1 << blk);
            }
            r->cbp_luma &= ~(1 << b);
        }
    }
    for (comp = 0; comp < 2; comp++) {
        int dc[4];

        for (y = 0; y < 8; y++) {
            for (x = 0; x < 8; x++) {
                cd[y * 8 + x] = src_c(&p->f, comp, mx * 8 + x, my * 8 + y) -
                                pc[comp][y * 8 + x];
            }
        }
        for (b = 0; b < 4; b++) {
            fdct4(cd + (b >> 1) * 4 * 8 + (b & 1) * 4, 8, coef);
            dc[b] = coef[0];
            for (i = 1; i < 16; i++) {
                r->cac[comp][b][i] = quant_inter(coef[i], qpc, q_class(i), 0);
                if (r->cac[comp][b][i]) {
                    r->cbp_chroma = 2;
                }
            }
        }
        const int f[4] = { dc[0] + dc[1] + dc[2] + dc[3],
                           dc[0] - dc[1] + dc[2] - dc[3],
                           dc[0] + dc[1] - dc[2] - dc[3],
                           dc[0] - dc[1] - dc[2] + dc[3] };
        for (i = 0; i < 4; i++) {
            r->cdc[comp][i] = quant_inter(f[i], qpc, 0, 1);
            if (r->cdc[comp][i] && !r->cbp_chroma) {
                r->cbp_chroma = 1;
            }
        }
    }
    if (r->cbp_chroma < 2) {
        memset(r->cac, 0, sizeof(r->cac));
    }
    if (!r->cbp_chroma) {
        memset(r->cdc, 0, sizeof(r->cdc));
    }
}

/* reconstruct pred + residual into the picture (8.5.12, 8.5.11) */
static void recon_inter(PPic *p, int mx, int my, const uint8_t py[256],
                        const uint8_t pc[2][64], int qp, const PRes *r)
{
    const int qpc = chroma_qp(qp, p->cqp);
    const int lsc = 16 * q_v[qpc % 6][0];
    int d[16], b, i, comp;

    for (b = 0; b < 16; b++) {
        const int bx = b & 3, by = b >> 2;

        for (i = 0; i < 16; i++) {
            d[i] = dequant4(r->luma[b][i], qp, i);
        }
        idct4_add(d, p->f.y + (my * 16 + by * 4) * p->f.w + mx * 16 + bx * 4,
                  p->f.w, py + by * 4 * 16 + bx * 4, 16);
    }
    for (comp = 0; comp < 2; comp++) {
        uint8_t *dst = comp ? p->f.cr : p->f.cb;
        const int *l = r->cdc[comp];
        const int f[4] = { l[0] + l[1] + l[2] + l[3], l[0] - l[1] + l[2] - l[3],
                           l[0] + l[1] - l[2] - l[3], l[0] - l[1] - l[2] + l[3] };

        for (b = 0; b < 4; b++) {
            d[0] = ((f[b] * lsc) << (qpc / 6)) >> 5;
            for (i = 1; i < 16; i++) {
                d[i] = dequant4(r->cac[comp][b][i], qpc, i);
            }
            idct4_add(d, dst + (my * 8 + (b >> 1) * 4) * (p->f.w / 2) +
                      mx * 8 + (b & 1) * 4, p->f.w / 2,
                      pc[comp] + (b >> 1) * 4 * 8 + (b & 1) * 4, 8);
        }
    }
}

/* ---- CABAC syntax ------------------------------------------------------------ */

static void cabac_mvd(Cabac *c, int base, int sum, int v)
{
    const int a = abs(v);
    const int inc0 = sum < 3 ? 0 : sum > 32 ? 2 : 1;
    static const uint8_t inc[9] = { 0, 3, 4, 5, 6, 6, 6, 6, 6 };
    int i;

    if (!a) {
        cabac_decision(c, base + inc0, 0);
        return;
    }
    cabac_decision(c, base + inc0, 1);
    for (i = 1; i < MIN(a, 9); i++) {
        cabac_decision(c, base + inc[i], 1);
    }
    if (a < 9) {
        cabac_decision(c, base + inc[MIN(a, 8)], 0);
    } else {
        int k = 3, s = a - 9;

        while (s >= (1 << k)) {                 /* UEG3 suffix */
            cabac_bypass(c, 1);
            s -= 1 << k;
            k++;
        }
        cabac_bypass(c, 0);
        while (k--) {
            cabac_bypass(c, (s >> k) & 1);
        }
    }
    cabac_bypass(c, v < 0);
}

static void cabac_qp_delta(PPic *p, Cabac *c, int delta)
{
    const int u = delta > 0 ? 2 * delta - 1 : -2 * delta;
    int i;

    cabac_decision(c, 60 + p->last_qp_delta_nz, u != 0);
    for (i = 1; i <= u; i++) {
        cabac_decision(c, i == 1 ? 62 : 63, i < u);
    }
}

/* condTermFlag of a neighbouring macroblock for coded_block_flag when the
 * block in it does not exist or carries no flag (9.3.3.1.1.9) */
static int cbf_absent(const PMb *n, bool intra)
{
    return n ? 0 : intra;
}

/* luma 4x4 block (bx, by)'s coded_block_flag condTerm from the left (dx -1)
 * or upper (dy -1) neighbour */
static int cbf_luma_nb(PPic *p, int mx, int my, const PMb *cur, int bx, int by,
                       int dx, int dy)
{
    int nx = bx + dx, ny = by + dy;
    const PMb *n = cur;

    if (nx < 0 || ny < 0) {
        n = pmb_at(p, mx + dx, my + dy);
        nx &= 3;
        ny &= 3;
        if (!n || n->type != PMB_INTER) {
            return cbf_absent(n, false);
        }
    }
    return (n->nz >> (ny * 4 + nx)) & 1;
}

static int cbf_cac_nb(PPic *p, int mx, int my, const PMb *cur, int comp,
                      int b, int dx, int dy, bool intra)
{
    int nx = (b & 1) + dx, ny = (b >> 1) + dy;
    const PMb *n = cur;

    if (nx < 0 || ny < 0) {
        n = pmb_at(p, mx + dx, my + dy);
        nx &= 1;
        ny &= 1;
        if (!n || n->type == PMB_SKIP || n->cbp_chroma != 2) {
            return cbf_absent(n, intra);
        }
    }
    return (n->cac >> (comp * 4 + ny * 2 + nx)) & 1;
}

static int cbf_cdc_nb(const PMb *n, int comp, bool intra)
{
    if (!n || n->type == PMB_SKIP || !n->cbp_chroma) {
        return cbf_absent(n, intra);
    }
    return (n->cdc >> comp) & 1;
}

static void put_chroma(PPic *p, Cabac *c, int mx, int my, PMb *me,
                       const PRes *r, bool intra)
{
    const PMb *a = pmb_at(p, mx - 1, my), *b = pmb_at(p, mx, my - 1);
    int comp, blk, i;

    me->cdc = me->cac = 0;
    for (comp = 0; comp < 2 && r->cbp_chroma; comp++) {
        const int nz = r->cdc[comp][0] || r->cdc[comp][1] ||
                       r->cdc[comp][2] || r->cdc[comp][3];

        cabac_decision(c, 97 + cbf_cdc_nb(a, comp, intra) +
                       2 * cbf_cdc_nb(b, comp, intra), nz);
        if (nz) {
            cabac_residual(c, 105 + 44, 166 + 44, 227 + 30, r->cdc[comp], 4);
            me->cdc |= 1 << comp;
        }
    }
    for (comp = 0; comp < 2 && r->cbp_chroma == 2; comp++) {
        for (blk = 0; blk < 4; blk++) {
            int scan[15], nz = 0;

            for (i = 0; i < 15; i++) {
                scan[i] = r->cac[comp][blk][zigzag4x4[i + 1]];
                nz |= scan[i];
            }
            cabac_decision(c, 101 +
                           cbf_cac_nb(p, mx, my, me, comp, blk, -1, 0, intra) +
                           2 * cbf_cac_nb(p, mx, my, me, comp, blk, 0, -1,
                                          intra), nz != 0);
            if (nz) {
                cabac_residual(c, 105 + 47, 166 + 47, 227 + 39, scan, 15);
                me->cac |= 1 << (comp * 4 + blk);
            }
        }
    }
}

/* coded_block_pattern of an inter macroblock (9.3.3.1.1.4) */
static void put_cbp(PPic *p, Cabac *c, int mx, int my, int luma, int chroma)
{
    const PMb *a = pmb_at(p, mx - 1, my), *b = pmb_at(p, mx, my - 1);
    int b8;

    for (b8 = 0; b8 < 4; b8++) {
        int ca, cb;

        if (b8 & 1) {
            ca = !((luma >> (b8 - 1)) & 1);
        } else {
            ca = a && !(a->type != PMB_SKIP && ((a->cbp_luma >> (b8 + 1)) & 1));
        }
        if (b8 & 2) {
            cb = !((luma >> (b8 - 2)) & 1);
        } else {
            cb = b && !(b->type != PMB_SKIP && ((b->cbp_luma >> (b8 + 2)) & 1));
        }
        cabac_decision(c, 73 + ca + 2 * cb, (luma >> b8) & 1);
    }
    const int ca = a && a->type != PMB_SKIP && a->cbp_chroma != 0;
    const int cb = b && b->type != PMB_SKIP && b->cbp_chroma != 0;

    cabac_decision(c, 77 + ca + 2 * cb, chroma != 0);
    if (chroma) {
        const int ca2 = a && a->type != PMB_SKIP && a->cbp_chroma == 2;
        const int cb2 = b && b->type != PMB_SKIP && b->cbp_chroma == 2;

        cabac_decision(c, 81 + ca2 + 2 * cb2, chroma == 2);
    }
}

static void put_inter(PPic *p, Cabac *c, int mx, int my, PMb *me,
                      const PRes *r)
{
    const PMb *a = pmb_at(p, mx - 1, my), *b = pmb_at(p, mx, my - 1);
    int comp, b8, k, i;

    cabac_decision(c, 14, 0);                   /* P_L0_16x16: 0 0 0 */
    cabac_decision(c, 15, 0);
    cabac_decision(c, 16, 0);
    for (comp = 0; comp < 2; comp++) {
        const int sum = (a && a->type == PMB_INTER ? abs(a->mvd[comp]) : 0) +
                        (b && b->type == PMB_INTER ? abs(b->mvd[comp]) : 0);

        cabac_mvd(c, comp ? 47 : 40, sum, me->mvd[comp]);
    }
    put_cbp(p, c, mx, my, r->cbp_luma, r->cbp_chroma);
    if (r->cbp_luma && p->t8x8) {
        cabac_decision(c, 399, 0);              /* transform_size_8x8 */
    }
    if (r->cbp_luma || r->cbp_chroma) {
        const int delta = p->qp_target - p->qp_cur;

        cabac_qp_delta(p, c, delta);
        p->qp_cur = p->qp_target;
        me->qp_delta_nz = delta != 0;
    }
    me->qp = p->qp_cur;
    me->nz = 0;
    for (b8 = 0; b8 < 4; b8++) {
        if (!((r->cbp_luma >> b8) & 1)) {
            continue;
        }
        for (k = 0; k < 4; k++) {
            const int bx = (b8 & 1) * 2 + (k & 1);
            const int by = (b8 >> 1) * 2 + (k >> 1);
            const int blk = by * 4 + bx;
            int scan[16], nz = 0;

            for (i = 0; i < 16; i++) {
                scan[i] = r->luma[blk][zigzag4x4[i]];
                nz |= scan[i];
            }
            cabac_decision(c, 85 + 8 +
                           cbf_luma_nb(p, mx, my, me, bx, by, -1, 0) +
                           2 * cbf_luma_nb(p, mx, my, me, bx, by, 0, -1),
                           nz != 0);
            if (nz) {
                cabac_residual(c, 105 + 29, 166 + 29, 227 + 20, scan, 16);
                me->nz |= 1 << blk;
            }
        }
    }
    put_chroma(p, c, mx, my, me, r, false);
}

/* ---- intra DC macroblocks in a P slice ------------------------------------- */

static int intra_dc_error(PPic *p, int mx, int my)
{
    int bx, by, x, y, e = 0;

    for (by = 0; by < 4; by++) {
        for (bx = 0; bx < 4; bx++) {
            int s = 0;

            for (y = 0; y < 4; y++) {
                for (x = 0; x < 4; x++) {
                    s += src_y(&p->f, mx * 16 + bx * 4 + x,
                               my * 16 + by * 4 + y);
                }
            }
            s = (s + 8) >> 4;
            for (y = 0; y < 4; y++) {
                for (x = 0; x < 4; x++) {
                    e += abs(src_y(&p->f, mx * 16 + bx * 4 + x,
                                   my * 16 + by * 4 + y) - s);
                }
            }
        }
    }
    return e;
}

/* I_16x16 DC, as encode_mb() but with the P slice's mb_type contexts and
 * neighbours that may be inter */
static void put_intra(PPic *p, Cabac *c, int mx, int my, PMb *me)
{
    const PMb *na = pmb_at(p, mx - 1, my), *nb = pmb_at(p, mx, my - 1);
    Frame *f = &p->f;
    const int qp = p->qp_target, qpc = chroma_qp(qp, p->cqp);
    int dc[16], lev[16], i, bx, by, x, y, comp;
    int cdc[2][4], clev[2][4], cpred[2][4];
    bool chroma_nz = false;

    const int pred = pred16_dc(f, mx, my);
    for (by = 0; by < 4; by++) {
        for (bx = 0; bx < 4; bx++) {
            int s = 0;
            for (y = 0; y < 4; y++) {
                for (x = 0; x < 4; x++) {
                    s += src_y(f, mx * 16 + bx * 4 + x, my * 16 + by * 4 + y) -
                         pred;
                }
            }
            dc[by * 4 + bx] = s;
        }
    }
    hadamard4(dc);
    for (i = 0; i < 16; i++) {
        lev[i] = quant(dc[i] / 2, qp, 1);
    }
    for (comp = 0; comp < 2; comp++) {
        uint8_t *rp = comp ? f->cr : f->cb;
        int w[4];
        for (i = 0; i < 4; i++) {
            const int xo = (i & 1) * 4, yo = (i >> 1) * 4;
            int s = 0;
            cpred[comp][i] = predc_dc(rp, f->w / 2, mx, my, xo, yo);
            for (y = 0; y < 4; y++) {
                for (x = 0; x < 4; x++) {
                    s += src_c(f, comp, mx * 8 + xo + x, my * 8 + yo + y) -
                         cpred[comp][i];
                }
            }
            w[i] = s;
        }
        cdc[comp][0] = w[0] + w[1] + w[2] + w[3];
        cdc[comp][1] = w[0] - w[1] + w[2] - w[3];
        cdc[comp][2] = w[0] + w[1] - w[2] - w[3];
        cdc[comp][3] = w[0] - w[1] - w[2] + w[3];
        for (i = 0; i < 4; i++) {
            clev[comp][i] = quant(cdc[comp][i], qpc, 1);
            chroma_nz |= clev[comp][i] != 0;
        }
    }

    cabac_decision(c, 14, 1);                   /* prefix: intra */
    cabac_decision(c, 17, 1);                   /* not I_NxN */
    cabac_terminate0(c);                        /* not I_PCM */
    cabac_decision(c, 17 + 1, 0);               /* cbp luma 0 */
    cabac_decision(c, 17 + 2, chroma_nz);
    if (chroma_nz) {
        cabac_decision(c, 17 + 2, 0);           /* cbp chroma 1 */
    }
    cabac_decision(c, 17 + 3, 1);               /* pred mode 2: 1 0 */
    cabac_decision(c, 17 + 3, 0);
    cabac_decision(c, 64, 0);                   /* chroma pred: DC */
    {
        const int delta = qp - p->qp_cur;

        cabac_qp_delta(p, c, delta);
        p->qp_cur = qp;
        me->qp_delta_nz = delta != 0;
    }
    me->qp = qp;

    int scan[16], nz = 0;
    for (i = 0; i < 16; i++) {
        scan[i] = lev[zigzag4x4[i]];
        nz |= scan[i];
    }
    /* Intra16x16DCLevel's flag: an absent neighbour counts 1, an inter or
     * skipped one 0 */
    const int ca = !na ? 1 : na->type == PMB_INTRA ? na->luma_dc_cbf : 0;
    const int cb = !nb ? 1 : nb->type == PMB_INTRA ? nb->luma_dc_cbf : 0;
    cabac_decision(c, 85 + ca + 2 * cb, nz != 0);
    if (nz) {
        cabac_residual(c, 105, 166, 227, scan, 16);
    }
    me->luma_dc_cbf = nz != 0;
    me->cbp_luma = 0;
    me->nz = 0;
    me->cbp_chroma = chroma_nz;
    me->cdc = me->cac = 0;
    for (comp = 0; comp < 2 && chroma_nz; comp++) {
        const int cnz = clev[comp][0] || clev[comp][1] || clev[comp][2] ||
                        clev[comp][3];

        cabac_decision(c, 97 + cbf_cdc_nb(na, comp, true) +
                       2 * cbf_cdc_nb(nb, comp, true), cnz);
        if (cnz) {
            cabac_residual(c, 105 + 44, 166 + 44, 227 + 30, clev[comp], 4);
            me->cdc |= 1 << comp;
        }
    }
    if (!chroma_nz) {
        memset(clev, 0, sizeof(clev));
    }
    recon_mb(f, mx, my, pred, lev, qp, cpred, clev, qpc);
}

/* ---- the deblocking filter (8.7) ------------------------------------------- */

typedef struct DbMb {
    uint8_t intra, qp;
    int16_t mv[2];
    uint16_t nz;
} DbMb;

static int db_bs(const DbMb *pm, int pb, const DbMb *qm, int qb, bool mb_edge)
{
    if (pm->intra || qm->intra) {
        return mb_edge ? 4 : 3;
    }
    if (((pm->nz >> pb) & 1) || ((qm->nz >> qb) & 1)) {
        return 2;
    }
    if (abs(pm->mv[0] - qm->mv[0]) >= 4 || abs(pm->mv[1] - qm->mv[1]) >= 4) {
        return 1;
    }
    return 0;
}

/* one line of samples across an edge: p[-k * step] and q[k * step] */
static void db_line(uint8_t *q, int step, int bs, int alpha, int beta,
                    int tc0, bool chroma)
{
    const int p0 = q[-step], p1 = q[-2 * step], q0 = q[0], q1 = q[step];

    if (abs(p0 - q0) >= alpha || abs(p1 - p0) >= beta ||
        abs(q1 - q0) >= beta) {
        return;
    }
    if (chroma) {
        if (bs < 4) {
            const int tc = tc0 + 1;
            const int d = clampi((((q0 - p0) << 2) + (p1 - q1) + 4) >> 3,
                                 -tc, tc);

            q[-step] = clip1(p0 + d);
            q[0] = clip1(q0 - d);
        } else {
            q[-step] = (2 * p1 + p0 + q1 + 2) >> 2;
            q[0] = (2 * q1 + q0 + p1 + 2) >> 2;
        }
        return;
    }
    const int p2 = q[-3 * step], q2 = q[2 * step];
    const int ap = abs(p2 - p0), aq = abs(q2 - q0);

    if (bs < 4) {
        const int tc = tc0 + (ap < beta) + (aq < beta);
        const int d = clampi((((q0 - p0) << 2) + (p1 - q1) + 4) >> 3, -tc, tc);

        q[-step] = clip1(p0 + d);
        q[0] = clip1(q0 - d);
        if (ap < beta) {
            q[-2 * step] = p1 + clampi((p2 + ((p0 + q0 + 1) >> 1) -
                                        (p1 << 1)) >> 1, -tc0, tc0);
        }
        if (aq < beta) {
            q[step] = q1 + clampi((q2 + ((p0 + q0 + 1) >> 1) -
                                   (q1 << 1)) >> 1, -tc0, tc0);
        }
        return;
    }
    const int p3 = q[-4 * step], q3 = q[3 * step];
    const bool strong = abs(p0 - q0) < ((alpha >> 2) + 2);

    if (ap < beta && strong) {
        q[-step] = (p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3;
        q[-2 * step] = (p2 + p1 + p0 + q0 + 2) >> 2;
        q[-3 * step] = (2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3;
    } else {
        q[-step] = (2 * p1 + p0 + q1 + 2) >> 2;
    }
    if (aq < beta && strong) {
        q[0] = (p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) >> 3;
        q[step] = (p0 + q0 + q1 + q2 + 2) >> 2;
        q[2 * step] = (2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) >> 3;
    } else {
        q[0] = (2 * q1 + q0 + p1 + 2) >> 2;
    }
}

/*
 * Deblock the picture in place, macroblocks in raster order: luma vertical
 * edges left to right, then horizontal ones top to bottom, then chroma the
 * same way, the chroma edges taking the bS of the luma ones they align with.
 */
static void deblock(Frame *f, const DbMb *mbs, int cqp, int off_a, int off_b,
                    const uint16_t *slice)
{
    const int cw = f->w / 2;
    int mx, my, e, k, dir, comp;

    for (my = 0; my < f->mbh; my++) {
        for (mx = 0; mx < f->mbw; mx++) {
            const DbMb *q = &mbs[my * f->mbw + mx];
            int bs[2][4][4];                    /* [dir][edge][quarter] */

            for (dir = 0; dir < 2; dir++) {
                for (e = 0; e < 4; e++) {
                    const DbMb *pm = q;

                    if (e == 0) {
                        const int pi = dir == 0 ? my * f->mbw + mx - 1
                                                : (my - 1) * f->mbw + mx;

                        /* the picture's edge; with slice (idc 2) a
                         * neighbour in another slice too */
                        if ((dir == 0 ? mx == 0 : my == 0) ||
                            (slice && slice[pi] != slice[my * f->mbw + mx])) {
                            memset(bs[dir][e], 0, sizeof(bs[dir][e]));
                            continue;
                        }
                        pm = dir == 0 ? q - 1 : q - f->mbw;
                    }
                    for (k = 0; k < 4; k++) {
                        const int qb = dir == 0 ? k * 4 + e : e * 4 + k;
                        const int pb = dir == 0 ? k * 4 + (e + 3) % 4
                                                : ((e + 3) % 4) * 4 + k;

                        bs[dir][e][k] = db_bs(pm, pb, q, qb, e == 0);
                    }
                }
            }
            /* luma */
            for (dir = 0; dir < 2; dir++) {
                for (e = 0; e < 4; e++) {
                    const DbMb *pm = e ? q : dir == 0 ? q - 1 : q - f->mbw;
                    const int qpav = (pm->qp + q->qp + 1) >> 1;
                    const int ia = clampi(qpav + off_a, 0, 51);
                    const int ib = clampi(qpav + off_b, 0, 51);

                    for (k = 0; k < 16; k++) {
                        const int b = bs[dir][e][k >> 2];
                        uint8_t *s;

                        if (!b) {
                            continue;
                        }
                        s = dir == 0 ? f->y + (my * 16 + k) * f->w +
                                       mx * 16 + e * 4
                                     : f->y + (my * 16 + e * 4) * f->w +
                                       mx * 16 + k;
                        db_line(s, dir == 0 ? 1 : f->w, b, db_alpha[ia],
                                db_beta[ib], b < 4 ? db_tc0[ia][b - 1] : 0,
                                false);
                    }
                }
            }
            /* chroma: edges 0 and 2 of the luma's */
            for (comp = 0; comp < 2; comp++) {
                uint8_t *pl = comp ? f->cr : f->cb;

                for (dir = 0; dir < 2; dir++) {
                    for (e = 0; e < 4; e += 2) {
                        const DbMb *pm = e ? q : dir == 0 ? q - 1 : q - f->mbw;
                        const int qpav = (chroma_qp(pm->qp, cqp) +
                                          chroma_qp(q->qp, cqp) + 1) >> 1;
                        const int ia = clampi(qpav + off_a, 0, 51);
                        const int ib = clampi(qpav + off_b, 0, 51);

                        for (k = 0; k < 8; k++) {
                            const int b = bs[dir][e][k >> 1];
                            uint8_t *s;

                            if (!b) {
                                continue;
                            }
                            s = dir == 0 ? pl + (my * 8 + k) * cw + mx * 8 +
                                           e * 2
                                         : pl + (my * 8 + e * 2) * cw +
                                           mx * 8 + k;
                            db_line(s, dir == 0 ? 1 : cw, b, db_alpha[ia],
                                    db_beta[ib],
                                    b < 4 ? db_tc0[ia][b - 1] : 0, true);
                        }
                    }
                }
            }
        }
    }
}

/* the reconstruction, deblocked, into the reference (NV12 at stride w) */
static void frame_to_ref(Frame *f, const DbMb *db, int cqp, int deblock_idc,
                         int off_a, int off_b, uint8_t *ry, uint8_t *ruv)
{
    int i, n = f->w * f->h / 4;

    if (deblock_idc != 1) {
        deblock(f, db, cqp, off_a, off_b, NULL);
    }
    memcpy(ry, f->y, (size_t)f->w * f->h);
    for (i = 0; i < n; i++) {
        ruv[2 * i] = f->cb[i];
        ruv[2 * i + 1] = f->cr[i];
    }
}

/* ---- the slices ------------------------------------------------------------ */

size_t h264_slice_i16dc_cabac_ref(int qp, int chroma_qp_offset,
                                  const H264Deblock *db, int mbw, int mbh,
                                  const uint8_t *y, int ys, const uint8_t *uv,
                                  int cs, uint8_t *ry, uint8_t *ruv,
                                  uint8_t *out, size_t cap)
{
    g_autofree MbInfo *info = g_new0(MbInfo, mbw * mbh);
    g_autofree DbMb *dbm = g_new0(DbMb, mbw * mbh);
    Frame f;
    Cabac c;
    int mx, my, i;

    qp = MIN(MAX(qp, 0), 51);
    frame_setup(&f, mbw, mbh, y, ys, uv, cs);
    cabac_init(&c, qp, out, out + cap);
    const int qpc = chroma_qp(qp, chroma_qp_offset);
    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            encode_mb(&f, &c, NULL, info, mx, my, qp, qpc);
            if (mx == mbw - 1 && my == mbh - 1) {
                cabac_flush(&c);
            } else {
                cabac_terminate0(&c);
            }
        }
    }
    for (i = 0; i < mbw * mbh; i++) {
        dbm[i] = (DbMb) { .intra = 1, .qp = qp };
    }
    frame_to_ref(&f, dbm, chroma_qp_offset, db->idc, db->alpha_off,
                 db->beta_off, ry, ruv);
    frame_free(&f);
    return c.p - out;
}

/* a P_Skip macroblock: its prediction is the picture */
static void put_skip(PPic *p, int mx, int my, PMb *me, const int16_t mvs[2],
                     const uint8_t py[256], const uint8_t pc[2][64])
{
    int i;

    me->type = PMB_SKIP;
    me->mv[0] = mvs[0];
    me->mv[1] = mvs[1];
    me->qp = p->qp_cur;
    for (i = 0; i < 16; i++) {
        memcpy(p->f.y + (my * 16 + i) * p->f.w + mx * 16, py + i * 16, 16);
    }
    for (i = 0; i < 8; i++) {
        memcpy(p->f.cb + (my * 8 + i) * (p->f.w / 2) + mx * 8, pc[0] + i * 8,
               8);
        memcpy(p->f.cr + (my * 8 + i) * (p->f.w / 2) + mx * 8, pc[1] + i * 8,
               8);
    }
    p->last_qp_delta_nz = 0;
}

static size_t p_slice_try(PPic *p, int cabac_init_idc, uint8_t *out,
                          size_t cap, H264PStats *st)
{
    const int mbw = p->f.mbw, mbh = p->f.mbh;
    const int8_t (*mn)[2] = h264_cabac_init_P[MIN(MAX(cabac_init_idc, 0), 2)];
    const int lambda = 1 << MAX(0, (p->qp_target - 12) / 6);
    Cabac c;
    int mx, my, i;

    cabac_init(&c, p->qp_slice, out, out + cap);
    for (i = 0; i < 460; i++) {
        const int s = MIN(MAX(((mn[i][0] * p->qp_slice) >> 4) + mn[i][1], 1),
                          126);
        c.state[i] = (MIN(s, 127 - s) << 1) | (s >> 6);
    }
    p->qp_cur = p->qp_slice;
    p->last_qp_delta_nz = 0;
    memset(st, 0, sizeof(*st));
    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            PMb *me = pmb_at(p, mx, my);
            const PMb *a = pmb_at(p, mx - 1, my), *b = pmb_at(p, mx, my - 1);
            uint8_t py[256], pc[2][64];
            int16_t mvs[2], mvp[2], mv[2];
            PRes r;

            memset(me, 0, sizeof(*me));
            if (p->force_intra) {
                /* the decoder's reference unknown: nothing may refer to it */
                cabac_decision(&c, 11 + (a && a->type != PMB_SKIP) +
                               (b && b->type != PMB_SKIP), 0);
                me->type = PMB_INTRA;
                put_intra(p, &c, mx, my, me);
                p->last_qp_delta_nz = me->qp_delta_nz;
                st->intra++;
                goto next;
            }
            mv_skip(p, mx, my, mvs);
            mv_pred(p, mx, my, mvp);

            /* P_Skip if its prediction leaves nothing to code */
            mc_mb(p, mx, my, mvs, py, pc);
            if (p->force_skip) {
                cabac_decision(&c, 11 + (a && a->type != PMB_SKIP) +
                               (b && b->type != PMB_SKIP), 1);
                put_skip(p, mx, my, me, mvs, py, pc);
                st->skip++;
                goto next;
            }
            residual_mb(p, mx, my, py, pc, p->qp_target, &r);
            const int skip_inc = (a && a->type != PMB_SKIP) +
                                 (b && b->type != PMB_SKIP);
            if (!r.cbp_luma && !r.cbp_chroma) {
                cabac_decision(&c, 11 + skip_inc, 1);
                put_skip(p, mx, my, me, mvs, py, pc);
                st->skip++;
                goto next;
            }

            /* motion search: candidates, then a diamond around the best */
            {
                const int16_t cand[5][2] = {
                    { 0, 0 }, { mvp[0], mvp[1] }, { mvs[0], mvs[1] },
                    { a && a->type != PMB_INTRA ? a->mv[0] : 0,
                      a && a->type != PMB_INTRA ? a->mv[1] : 0 },
                    { b && b->type != PMB_INTRA ? b->mv[0] : 0,
                      b && b->type != PMB_INTRA ? b->mv[1] : 0 },
                };
                int bx = 0, by = 0, best = INT_MAX, it;

                for (i = 0; i < 5; i++) {
                    const int cx = clampi((cand[i][0] + 2) >> 2, -64, 64);
                    const int cy = clampi((cand[i][1] + 2) >> 2, -64, 64);
                    const int cost = sad_mb(p, mx, my, cx, cy, best) +
                        lambda * (abs(cx * 4 - mvp[0]) + abs(cy * 4 - mvp[1]))
                        / 4;

                    if (cost < best) {
                        best = cost;
                        bx = cx;
                        by = cy;
                    }
                }
                /* still far off: a coarse grid over +-16 pixels first */
                if (best > 512) {
                    int gx, gy, cbx = bx, cby = by;

                    for (gy = -16; gy <= 16; gy += 4) {
                        for (gx = -16; gx <= 16; gx += 4) {
                            const int cost = sad_mb(p, mx, my, gx, gy, best) +
                                lambda * (abs(gx * 4 - mvp[0]) +
                                          abs(gy * 4 - mvp[1])) / 4;

                            if (cost < best) {
                                best = cost;
                                cbx = gx;
                                cby = gy;
                            }
                        }
                    }
                    bx = cbx;
                    by = cby;
                }
                for (it = 0; it < 16 && best; it++) {
                    static const int8_t dia[4][2] = {
                        { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },
                    };
                    int nbx = bx, nby = by;

                    for (i = 0; i < 4; i++) {
                        const int cx = bx + dia[i][0], cy = by + dia[i][1];
                        int cost;

                        if (abs(cx) > 64 || abs(cy) > 64) {
                            continue;
                        }
                        cost = sad_mb(p, mx, my, cx, cy, best) +
                            lambda * (abs(cx * 4 - mvp[0]) +
                                      abs(cy * 4 - mvp[1])) / 4;
                        if (cost < best) {
                            best = cost;
                            nbx = cx;
                            nby = cy;
                        }
                    }
                    if (nbx == bx && nby == by) {
                        break;
                    }
                    bx = nbx;
                    by = nby;
                }
                mv[0] = bx * 4;
                mv[1] = by * 4;
                /* then half and quarter samples around it */
                {
                    int step, dx, dy;

                    best = sad_mb_q(p, mx, my, mv) +
                        lambda * (abs(mv[0] - mvp[0]) + abs(mv[1] - mvp[1]))
                        / 4;
                    for (step = 2; step >= 1; step >>= 1) {
                        int16_t nb[2] = { mv[0], mv[1] };

                        for (dy = -step; dy <= step; dy += step) {
                            for (dx = -step; dx <= step; dx += step) {
                                const int16_t m[2] = {
                                    (int16_t)(mv[0] + dx),
                                    (int16_t)(mv[1] + dy),
                                };
                                int cost;

                                if ((!dx && !dy) || abs(m[0]) > 256 ||
                                    abs(m[1]) > 256) {
                                    continue;
                                }
                                cost = sad_mb_q(p, mx, my, m) +
                                    lambda * (abs(m[0] - mvp[0]) +
                                              abs(m[1] - mvp[1])) / 4;
                                if (cost < best) {
                                    best = cost;
                                    nb[0] = m[0];
                                    nb[1] = m[1];
                                }
                            }
                        }
                        mv[0] = nb[0];
                        mv[1] = nb[1];
                    }
                }

                /* nothing near: the scene changed, code it intra */
                mc_mb(p, mx, my, mv, py, pc);
                if (mrsad_mb(p, mx, my, py) >
                    2 * intra_dc_error(p, mx, my) + 1024) {
                    cabac_decision(&c, 11 + skip_inc, 0);
                    me->type = PMB_INTRA;
                    put_intra(p, &c, mx, my, me);
                    p->last_qp_delta_nz = me->qp_delta_nz;
                    st->intra++;
                    goto next;
                }
            }
            mc_mb(p, mx, my, mv, py, pc);
            residual_mb(p, mx, my, py, pc, p->qp_target, &r);
            if (mv[0] == mvs[0] && mv[1] == mvs[1] && !r.cbp_luma &&
                !r.cbp_chroma) {
                cabac_decision(&c, 11 + skip_inc, 1);
                put_skip(p, mx, my, me, mvs, py, pc);
                st->skip++;
                goto next;
            }
            cabac_decision(&c, 11 + skip_inc, 0);
            me->type = PMB_INTER;
            me->mv[0] = mv[0];
            me->mv[1] = mv[1];
            me->mvd[0] = mv[0] - mvp[0];
            me->mvd[1] = mv[1] - mvp[1];
            me->cbp_luma = r.cbp_luma;
            me->cbp_chroma = r.cbp_chroma;
            put_inter(p, &c, mx, my, me, &r);
            recon_inter(p, mx, my, py, pc, me->qp, &r);
            p->last_qp_delta_nz = (r.cbp_luma || r.cbp_chroma) &&
                                  me->qp_delta_nz;
            st->inter++;
        next:
            if (mx == mbw - 1 && my == mbh - 1) {
                cabac_flush(&c);
            } else {
                cabac_terminate0(&c);
            }
        }
    }
    return c.p - out;
}

size_t h264_slice_p_cabac(int qp, int chroma_qp_offset, int cabac_init_idc,
                          bool transform_8x8_mode, const H264Deblock *db,
                          int mbw, int mbh, const uint8_t *y, int ys,
                          const uint8_t *uv, int cs, uint8_t *ry, uint8_t *ruv,
                          uint8_t *out, size_t cap, H264PStats *st)
{
    const int w = mbw * 16, h = mbh * 16, n = w * h / 4;
    g_autofree PMb *mbs = g_new0(PMb, mbw * mbh);
    g_autofree DbMb *dbm = g_new0(DbMb, mbw * mbh);
    g_autofree uint8_t *rcb = g_malloc(n), *rcr = g_malloc(n);
    H264PStats dummy;
    PPic p;
    size_t len;
    int i;

    if (!st) {
        st = &dummy;
    }
    qp = MIN(MAX(qp, 0), 51);
    for (i = 0; i < n; i++) {
        rcb[i] = ruv[2 * i];
        rcr[i] = ruv[2 * i + 1];
    }
    p = (PPic) { .ry = ry, .rcb = rcb, .rcr = rcr, .ry0 = 0, .rh = h,
                 .mb = mbs, .qp_slice = qp, .qp_target = qp,
                 .cqp = chroma_qp_offset, .t8x8 = transform_8x8_mode };
    frame_setup(&p.f, mbw, mbh, y, ys, uv, cs);
    /* the buffer must hold the picture: if it does not, again at a higher
     * QP (mb_qp_delta on the first coded macroblock) */
    for (;;) {
        len = p_slice_try(&p, cabac_init_idc, out, cap, st);
        if (len + 64 <= cap || p.qp_target >= 51) {
            break;
        }
        p.qp_target = MIN(p.qp_target + 6, 51);
    }
    st->qp = p.qp_target;
    for (i = 0; i < mbw * mbh; i++) {
        dbm[i] = (DbMb) { .intra = mbs[i].type == PMB_INTRA,
                          .qp = mbs[i].qp, .nz = mbs[i].nz,
                          .mv = { mbs[i].mv[0], mbs[i].mv[1] } };
        if (mbs[i].type == PMB_INTRA) {
            dbm[i].mv[0] = dbm[i].mv[1] = 0;
        }
    }
    frame_to_ref(&p.f, dbm, chroma_qp_offset, db->idc, db->alpha_off,
                 db->beta_off, ry, ruv);
    frame_free(&p.f);
    return len;
}

/* ---- a picture of several slices ------------------------------------------ */

/*
 * For engines that split a picture into slices of whole macroblock rows
 * (the AVPU).  Each slice is coded on its own window of the picture's
 * reconstruction and macroblock array: its first row has no neighbour
 * above, as a slice's does not (intra prediction, vector prediction, the
 * CABAC contexts), while motion compensation reads the whole reference.
 * The in-loop filter runs once the picture is complete, across the slices'
 * edges or not as disable_deblocking_filter_idc says (0 / 2).
 */
struct H264Pic {
    int mbw, mbh;
    uint8_t *y, *cb, *cr;           /* the reconstruction, not deblocked */
    const uint8_t *sy, *suv;
    int sys, scs;
    uint8_t *ry, *rcb, *rcr;        /* the reference, NULL: none */
    PMb *mb;
    MbInfo *info;
    DbMb *db;
    uint16_t *slice;                /* each macroblock's slice */
    int nslices;
    int cqp;
    bool t8x8;
};

H264Pic *h264_pic_new(int mbw, int mbh, const uint8_t *y, int ys,
                      const uint8_t *uv, int cs, const uint8_t *ry,
                      const uint8_t *ruv, int chroma_qp_offset,
                      bool transform_8x8_mode)
{
    H264Pic *pic = g_new0(H264Pic, 1);
    const size_t n = (size_t)mbw * mbh * 64;        /* chroma samples */
    size_t i;

    pic->mbw = mbw;
    pic->mbh = mbh;
    pic->y = g_malloc0((size_t)mbw * mbh * 256);
    pic->cb = g_malloc0(n);
    pic->cr = g_malloc0(n);
    pic->sy = y;
    pic->suv = uv;
    pic->sys = ys;
    pic->scs = cs;
    pic->mb = g_new0(PMb, mbw * mbh);
    pic->info = g_new0(MbInfo, mbw * mbh);
    pic->db = g_new0(DbMb, mbw * mbh);
    pic->slice = g_new0(uint16_t, mbw * mbh);
    pic->cqp = chroma_qp_offset;
    pic->t8x8 = transform_8x8_mode;
    if (ry && ruv) {
        pic->ry = g_memdup2(ry, (size_t)mbw * mbh * 256);
        pic->rcb = g_malloc(n);
        pic->rcr = g_malloc(n);
        for (i = 0; i < n; i++) {
            pic->rcb[i] = ruv[2 * i];
            pic->rcr[i] = ruv[2 * i + 1];
        }
    }
    return pic;
}

/* the window of rows [r0, r0 + rows) as a Frame of its own */
static void pic_window(const H264Pic *pic, Frame *f, int r0, int rows)
{
    const int w = pic->mbw * 16;

    *f = (Frame) { .mbw = pic->mbw, .mbh = rows, .w = w, .h = rows * 16,
                   .y = pic->y + (size_t)r0 * 16 * w,
                   .cb = pic->cb + (size_t)r0 * 8 * (w / 2),
                   .cr = pic->cr + (size_t)r0 * 8 * (w / 2),
                   .sy = pic->sy + (size_t)r0 * 16 * pic->sys,
                   .suv = pic->suv + (size_t)r0 * 8 * pic->scs,
                   .sys = pic->sys, .scs = pic->scs };
}

size_t h264_pic_slice_cabac(H264Pic *pic, bool p_slice, int r0, int rows,
                            int qp, int cabac_init_idc, uint8_t *out,
                            size_t cap, H264PStats *st)
{
    const int mbw = pic->mbw;
    H264PStats dummy;
    size_t len;
    int i;

    if (!st) {
        st = &dummy;
    }
    memset(st, 0, sizeof(*st));
    qp = MIN(MAX(qp, 0), 51);
    r0 = MIN(MAX(r0, 0), pic->mbh - 1);
    rows = MIN(MAX(rows, 1), pic->mbh - r0);
    for (i = r0 * mbw; i < (r0 + rows) * mbw; i++) {
        pic->slice[i] = pic->nslices;
    }
    pic->nslices++;

    if (!p_slice) {
        Frame f;
        Cabac c;
        MbInfo *info = pic->info + (size_t)r0 * mbw;
        const int qpc = chroma_qp(qp, pic->cqp);
        int mx, my;

        pic_window(pic, &f, r0, rows);
        memset(info, 0, sizeof(*info) * mbw * rows);
        cabac_init(&c, qp, out, out + cap);
        for (my = 0; my < rows; my++) {
            for (mx = 0; mx < mbw; mx++) {
                encode_mb(&f, &c, NULL, info, mx, my, qp, qpc);
                if (mx == mbw - 1 && my == rows - 1) {
                    cabac_flush(&c);
                } else {
                    cabac_terminate0(&c);
                }
            }
        }
        for (i = r0 * mbw; i < (r0 + rows) * mbw; i++) {
            pic->db[i] = (DbMb) { .intra = 1, .qp = qp };
        }
        st->intra = mbw * rows;
        st->qp = qp;
        return c.p - out;
    }

    PPic p = { .ry = pic->ry, .rcb = pic->rcb, .rcr = pic->rcr,
               .ry0 = r0 * 16, .rh = pic->mbh * 16,
               .force_intra = !pic->ry,
               .mb = pic->mb + (size_t)r0 * mbw, .qp_slice = qp,
               .qp_target = qp, .cqp = pic->cqp, .t8x8 = pic->t8x8 };

    pic_window(pic, &p.f, r0, rows);
    for (;;) {
        len = p_slice_try(&p, cabac_init_idc, out, cap, st);
        if (len + 64 <= cap || p.force_skip || p.force_intra) {
            break;
        }
        if (p.qp_target >= 51) {
            /* not even at QP 51: every macroblock skipped */
            p.force_skip = true;
            continue;
        }
        p.qp_target = MIN(p.qp_target + 6, 51);
    }
    st->qp = p.qp_target;
    for (i = r0 * mbw; i < (r0 + rows) * mbw; i++) {
        const PMb *m = &pic->mb[i];

        pic->db[i] = (DbMb) { .intra = m->type == PMB_INTRA, .qp = m->qp,
                              .nz = m->nz };
        if (m->type != PMB_INTRA) {
            pic->db[i].mv[0] = m->mv[0];
            pic->db[i].mv[1] = m->mv[1];
        }
    }
    return len;
}

void h264_pic_finish(H264Pic *pic, const H264Deblock *db, uint8_t *ry,
                     uint8_t *ruv)
{
    const int w = pic->mbw * 16;
    Frame f = { .mbw = pic->mbw, .mbh = pic->mbh, .w = w, .h = pic->mbh * 16,
                .y = pic->y, .cb = pic->cb, .cr = pic->cr };
    const int n = w * f.h / 4;
    int i;

    if (db->idc != 1) {
        deblock(&f, pic->db, pic->cqp, db->alpha_off, db->beta_off,
                db->idc == 2 ? pic->slice : NULL);
    }
    if (ry && ruv) {
        memcpy(ry, pic->y, (size_t)w * f.h);
        for (i = 0; i < n; i++) {
            ruv[2 * i] = pic->cb[i];
            ruv[2 * i + 1] = pic->cr[i];
        }
    }
}

void h264_pic_free(H264Pic *pic)
{
    if (!pic) {
        return;
    }
    g_free(pic->y);
    g_free(pic->cb);
    g_free(pic->cr);
    g_free(pic->ry);
    g_free(pic->rcb);
    g_free(pic->rcr);
    g_free(pic->mb);
    g_free(pic->info);
    g_free(pic->db);
    g_free(pic->slice);
    g_free(pic);
}
