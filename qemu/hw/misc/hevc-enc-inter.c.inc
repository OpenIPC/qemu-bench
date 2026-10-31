/*
 * P slices with real inter coding for the HEVC encoder (included by
 * hevc-enc.c, whose CABAC engine, intra DC CUs and picture it uses), for
 * engines whose library asks for it (HevcEncParams.inter).
 *
 * Each 8x8 CU of a P slice is
 *  - skipped: a merge candidate's motion, no residual;
 *  - merged with a residual, or coded with a motion vector (AMVP: a
 *    predictor of the list and the difference), one 2Nx2N PU with one
 *    transform unit (an 8x8 luma TB, two 4x4 chroma TBs; split_transform_flag
 *    0 where the SPS allows a split);
 *  - or intra DC, as the I slices' CUs, where nothing in the reference
 *    comes close.
 * Motion is full-pel at even luma positions (vectors are multiples of 8
 * quarter-samples), so that chroma is full-pel too and prediction is a copy
 * of the reference (8.5.3.3.3: integer positions, clipped to the picture);
 * one reference, refIdx 0.  The merge list (8.5.3.2.2-5: A1, B1, B0, A0, B2
 * with their comparisons, the temporal candidate, zero candidates up to
 * MaxNumMergeCand) and the AMVP list (8.5.3.2.6-8) are built as the decoder
 * builds them, the temporal candidates from the reference picture's motion
 * (its decoded vectors, kept with it: the 16x16-aligned block of the
 * bottom-right or the centre position); every picture refers to the one
 * before it, whose own reference is the one before that, so no vector is
 * scaled.
 *
 * The residual goes through the HEVC integer transforms (8.6.4.2, with the
 * intermediate clipping) and the default scaling lists when
 * scaling_list_enabled_flag is set (Table 7-6: 8x8 TBs, the inter list for
 * inter CUs; 4x4 TBs flat), and is coded as residual_coding() says
 * (7.3.8.11, 9.3.4.2.4-7: last position, coded_sub_block_flag,
 * sig_coeff_flag, greater1 / greater2 with their context sets,
 * coeff_abs_level_remaining with the Rice parameter; sign hiding when the
 * PPS has it).  The reconstruction is the decoder's, and the picture kept
 * as the reference is deblocked with the inter boundary strengths
 * (8.7.2.4: 2 at intra CUs, 1 where a side has luma coefficients or the
 * vectors differ by a sample or more; chroma filtered at 2 only).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* ---- tables -------------------------------------------------------------- */

/* Table 7-6: the default 8x8 scaling lists (symmetric, raster) */
static const uint8_t sl_intra8[64] = {
    16, 16, 16, 16, 17, 18, 21, 24, 16, 16, 16, 16, 17, 19, 22, 25,
    16, 16, 17, 18, 20, 22, 25, 29, 16, 16, 18, 21, 24, 27, 31, 36,
    17, 17, 20, 24, 30, 35, 41, 47, 18, 19, 22, 27, 35, 44, 54, 65,
    21, 22, 25, 31, 41, 54, 70, 88, 24, 25, 29, 36, 47, 65, 88, 115,
};
static const uint8_t sl_inter8[64] = {
    16, 16, 16, 16, 17, 18, 20, 24, 16, 16, 16, 17, 18, 20, 24, 25,
    16, 16, 17, 18, 20, 24, 25, 28, 16, 17, 18, 20, 24, 25, 28, 33,
    17, 18, 20, 24, 25, 28, 33, 41, 18, 20, 24, 25, 28, 33, 41, 54,
    20, 24, 25, 28, 33, 41, 54, 71, 24, 25, 28, 33, 41, 54, 71, 91,
};

/* the 8-point transform matrix (8.6.4.2); the 4-point one is its even rows'
 * first four columns */
static const int8_t tm8[8][8] = {
    { 64,  64,  64,  64,  64,  64,  64,  64 },
    { 89,  75,  50,  18, -18, -50, -75, -89 },
    { 83,  36, -36, -83, -83, -36,  36,  83 },
    { 75, -18, -89, -50,  50,  89,  18, -75 },
    { 64, -64, -64,  64,  64, -64, -64,  64 },
    { 50, -89,  18,  75, -75, -18,  89, -50 },
    { 36, -83,  83, -36, -36,  83, -83,  36 },
    { 18, -50,  75, -89,  89, -75,  50, -18 },
};

static inline int tmat(int n, int k, int j)
{
    return n == 8 ? tm8[k][j] : tm8[2 * k][j];
}

static const int quant_scale[6] = { 26214, 23302, 20560, 18396, 16384, 14564 };

/* up-right diagonal scans (6.5.3): 4x4 positions (x, y), and the 2x2
 * sub-blocks of an 8x8 TB */
static const uint8_t diag4[16][2] = {
    { 0, 0 }, { 0, 1 }, { 1, 0 }, { 0, 2 }, { 1, 1 }, { 2, 0 }, { 0, 3 },
    { 1, 2 }, { 2, 1 }, { 3, 0 }, { 1, 3 }, { 2, 2 }, { 3, 1 }, { 2, 3 },
    { 3, 2 }, { 3, 3 },
};
static const uint8_t diag2[4][2] = { { 0, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 } };

/* 9.3.4.2.5: ctxIdxMap of 4x4 TBs, by (y << 2) + x */
static const uint8_t sig_ctx_4x4[16] = {
    0, 1, 4, 5, 2, 3, 4, 5, 6, 6, 8, 8, 7, 7, 8, 8,
};

/* ---- transforms ---------------------------------------------------------- */

/* the scaling factor m of an n x n TB at (x, y) (8.6.3) */
static int scale_m(const Pic *p, int log2, int x, int y, bool inter)
{
    if (!p->prm->scaling_list || log2 == 2) {
        return 16;
    }
    return inter ? sl_inter8[y * 8 + x] : sl_intra8[y * 8 + x];
}

/* the forward transform (T R T^T) and quantisation of an n x n residual,
 * both raster; any rounding will do here, the decoder's side is exact */
static void fwd_quant(const Pic *p, const int *res, int log2, int qp,
                      bool inter, int *lev)
{
    const int n = 1 << log2, s1 = log2 - 1, s2 = log2 + 6;
    const int qbits = 14 + qp / 6 + (15 - 8 - log2);
    int tmp[64], c[64], i, j, k;

    for (i = 0; i < n; i++) {                   /* each row, horizontally */
        for (k = 0; k < n; k++) {
            int64_t s = 0;
            for (j = 0; j < n; j++) {
                s += (int64_t)tmat(n, k, j) * res[i * n + j];
            }
            tmp[i * n + k] = (int)((s + (1 << (s1 - 1))) >> s1);
        }
    }
    for (j = 0; j < n; j++) {                   /* each column, vertically */
        for (k = 0; k < n; k++) {
            int64_t s = 0;
            for (i = 0; i < n; i++) {
                s += (int64_t)tmat(n, k, i) * tmp[i * n + j];
            }
            c[k * n + j] = (int)((s + (1 << (s2 - 1))) >> s2);
        }
    }
    for (i = 0; i < n * n; i++) {
        const int m = scale_m(p, log2, i % n, i / n, inter);
        const int64_t q = ((int64_t)quant_scale[qp % 6] << 4) / m;
        const int64_t off = (int64_t)(inter ? 85 : 171) << (qbits - 9);
        const int64_t a = ((int64_t)abs(c[i]) * q + off) >> qbits;

        lev[i] = (int)MIN(a, 32767) * (c[i] < 0 ? -1 : 1);
    }
}

/* 8.6.2-8.6.4: scaling and the inverse transform, as the decoder does
 * them: res[] (raster) from lev[] */
static void dequant_inv(const Pic *p, const int *lev, int log2, int qp,
                        bool inter, int *res)
{
    static const int ls[6] = { 40, 45, 51, 57, 64, 72 };
    const int n = 1 << log2, bd = 8 + log2 - 5;
    int d[64], e[64], x, y, k;

    for (k = 0; k < n * n; k++) {
        const int m = scale_m(p, log2, k % n, k / n, inter);
        int64_t v = ((int64_t)lev[k] * m * ls[qp % 6]) << (qp / 6);

        v = (v + (1 << (bd - 1))) >> bd;
        d[k] = (int)MIN(MAX(v, -32768), 32767);
    }
    for (x = 0; x < n; x++) {                   /* columns */
        for (y = 0; y < n; y++) {
            int64_t s = 0;
            for (k = 0; k < n; k++) {
                s += (int64_t)tmat(n, k, y) * d[k * n + x];
            }
            e[y * n + x] = (int)MIN(MAX((s + 64) >> 7, -32768), 32767);
        }
    }
    for (y = 0; y < n; y++) {                   /* rows */
        for (x = 0; x < n; x++) {
            int64_t s = 0;
            for (k = 0; k < n; k++) {
                s += (int64_t)tmat(n, k, x) * e[y * n + k];
            }
            res[y * n + x] = (int)((s + 2048) >> 12);
        }
    }
}

/* ---- residual_coding() --------------------------------------------------- */

/* coeff_abs_level_remaining (9.3.3.11): TR prefix of cMax 4 << r, then
 * EG(r + 1) */
static void put_rem(Cabac *c, int v, int r)
{
    int i;

    if (v < (4 << r)) {
        for (i = 0; i < (v >> r); i++) {
            cabac_bypass(c, 1);
        }
        cabac_bypass(c, 0);
        for (i = r - 1; i >= 0; i--) {
            cabac_bypass(c, (v >> i) & 1);
        }
    } else {
        for (i = 0; i < 4; i++) {
            cabac_bypass(c, 1);
        }
        cabac_egk(c, v - (4 << r), r + 1);
    }
}

static const uint8_t last_group[32] = {
    0, 1, 2, 3, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7,
    8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9,
};
static const uint8_t last_min[10] = { 0, 1, 2, 3, 4, 6, 8, 12, 16, 24 };

/* last_sig_coeff_x/y_prefix (9.3.4.2.3) */
static void put_last_prefix(Cabac *c, int pos, int log2, int cidx, int base)
{
    const int off = cidx ? 15 : 3 * (log2 - 2) + ((log2 - 1) >> 2);
    const int shift = cidx ? log2 - 2 : (log2 + 1) >> 2;
    const int pre = last_group[pos], cmax = (log2 << 1) - 1;
    int i;

    for (i = 0; i < pre; i++) {
        cabac_decision(c, base + off + (i >> shift), 1);
    }
    if (pre < cmax) {
        cabac_decision(c, base + off + (pre >> shift), 0);
    }
}

static void put_last_suffix(Cabac *c, int pos)
{
    const int pre = last_group[pos];
    int i;

    if (pre > 3) {
        const int len = (pre >> 1) - 1, v = pos - last_min[pre];
        for (i = len - 1; i >= 0; i--) {
            cabac_bypass(c, (v >> i) & 1);
        }
    }
}

/*
 * residual_coding() of an n x n TB (n 4 or 8) of lev[] (raster), the
 * diagonal scan.  With sign hiding a level may be made one larger to set a
 * sub-block's parity, so lev[] is updated.  Some level is not zero.
 */
static void residual_full(Pic *p, Cabac *c, int *lev, int log2, int cidx)
{
    const int n = 1 << log2, nsb = n == 8 ? 4 : 1;
    int s, k, last_sb = 0, last_k = 0;
    bool csbf[2][2] = { { false } };
    int greater1_ctx = 1;               /* the previous sub-block's, at end */
    bool first_sb = true;

    for (s = 0; s < nsb; s++) {
        for (k = 0; k < 16; k++) {
            const int x = diag2[s][0] * 4 + diag4[k][0];
            const int y = diag2[s][1] * 4 + diag4[k][1];
            if (lev[y * n + x]) {
                last_sb = s;
                last_k = k;
                csbf[diag2[s][0]][diag2[s][1]] = true;
            }
        }
    }
    if (p->prm->transform_skip && log2 == 2) {
        cabac_decision(c, CTX_TRANSFORM_SKIP + !!cidx, 0);
    }
    {
        const int lx = diag2[last_sb][0] * 4 + diag4[last_k][0];
        const int ly = diag2[last_sb][1] * 4 + diag4[last_k][1];

        put_last_prefix(c, lx, log2, cidx, CTX_LAST_X);
        put_last_prefix(c, ly, log2, cidx, CTX_LAST_Y);
        put_last_suffix(c, lx);
        put_last_suffix(c, ly);
    }

    for (s = last_sb; s >= 0; s--) {
        const int xs = diag2[s][0], ys = diag2[s][1];
        const bool right = n == 8 && xs == 0 && csbf[1][ys];
        const bool below = n == 8 && ys == 0 && csbf[xs][1];
        int nz_k[16], a[16], nnz = 0;
        bool infer_dc = false;

        if (s < last_sb && s > 0) {
            cabac_decision(c, CTX_CSBF + MIN(right + below, 1) +
                           (cidx ? 2 : 0), csbf[xs][ys]);
            if (!csbf[xs][ys]) {
                continue;
            }
            infer_dc = true;
        }
        if (s == last_sb) {
            nz_k[nnz++] = last_k;           /* the last: not signalled */
        }
        for (k = s == last_sb ? last_k - 1 : 15; k >= 0; k--) {
            const int xp = diag4[k][0], yp = diag4[k][1];
            const int x = xs * 4 + xp, y = ys * 4 + yp;
            const bool sig = lev[y * n + x] != 0;
            int ctx;

            if (k == 0 && infer_dc) {
                /* every other one 0: inferred to be 1 */
                nz_k[nnz++] = 0;
                continue;
            }
            if (log2 == 2) {
                ctx = sig_ctx_4x4[(yp << 2) + xp];
            } else if (x + y == 0) {
                ctx = 0;
            } else {
                const int prev = right + 2 * below;

                if (prev == 0) {
                    ctx = xp + yp == 0 ? 2 : xp + yp < 3 ? 1 : 0;
                } else if (prev == 1) {
                    ctx = yp == 0 ? 2 : yp == 1 ? 1 : 0;
                } else if (prev == 2) {
                    ctx = xp == 0 ? 2 : xp == 1 ? 1 : 0;
                } else {
                    ctx = 2;
                }
                if (cidx == 0 && (xs > 0 || ys > 0)) {
                    ctx += 3;
                }
                ctx += 9;                   /* 8x8, diagonal scan */
            }
            cabac_decision(c, CTX_SIG + (cidx ? 27 : 0) + ctx, sig);
            if (sig) {
                nz_k[nnz++] = k;
                infer_dc = false;
            }
        }

        /* sign hiding: the lowest scan position's sign is the parity of
         * the sub-block's sum, made so by one more on its magnitude */
        const bool hide = p->prm->sign_hiding && nz_k[0] - nz_k[nnz - 1] > 3;
        if (hide) {
            const int xl = xs * 4 + diag4[nz_k[nnz - 1]][0];
            const int yl = ys * 4 + diag4[nz_k[nnz - 1]][1];
            int sum = 0;

            for (k = 0; k < nnz; k++) {
                sum += abs(lev[(ys * 4 + diag4[nz_k[k]][1]) * n +
                               xs * 4 + diag4[nz_k[k]][0]]);
            }
            if ((sum & 1) != (lev[yl * n + xl] < 0)) {
                lev[yl * n + xl] += lev[yl * n + xl] < 0 ? -1 : 1;
            }
        }
        for (k = 0; k < nnz; k++) {
            a[k] = abs(lev[(ys * 4 + diag4[nz_k[k]][1]) * n +
                           xs * 4 + diag4[nz_k[k]][0]]);
        }

        /* greater1 for the first 8, greater2 for the first of those > 1 */
        int ctx_set = (s == 0 || cidx > 0) ? 0 : 2;
        int g1 = 1, first_g2 = -1;

        if (!first_sb && greater1_ctx == 0) {
            ctx_set++;
        }
        first_sb = false;
        for (k = 0; k < MIN(nnz, 8); k++) {
            const bool gt1 = a[k] > 1;

            cabac_decision(c, CTX_GT1 + ctx_set * 4 + g1 + (cidx ? 16 : 0),
                           gt1);
            if (gt1) {
                g1 = 0;
                if (first_g2 < 0) {
                    first_g2 = k;
                }
            } else if (g1 > 0 && g1 < 3) {
                g1++;
            }
        }
        greater1_ctx = g1;
        if (first_g2 >= 0) {
            cabac_decision(c, CTX_GT2 + ctx_set + (cidx ? 4 : 0),
                           a[first_g2] > 2);
        }
        for (k = 0; k < nnz; k++) {
            if (hide && k == nnz - 1) {
                break;
            }
            cabac_bypass(c, lev[(ys * 4 + diag4[nz_k[k]][1]) * n +
                                xs * 4 + diag4[nz_k[k]][0]] < 0);
        }
        /* coeff_abs_level_remaining where baseLevel (1 + greater1 +
         * greater2) is ((k < 8) ? (k is the greater2 one ? 3 : 2) : 1) */
        {
            int rice = 0;

            for (k = 0; k < nnz; k++) {
                const int g1f = k < 8 && a[k] > 1;
                const int g2f = k == first_g2 && a[k] > 2;
                const int base = 1 + g1f + g2f;
                const int want = k < 8 ? (k == first_g2 ? 3 : 2) : 1;

                if (base == want) {
                    put_rem(c, a[k] - base, rice);
                    if (a[k] > 3 * (1 << rice)) {
                        rice = MIN(rice + 1, 4);
                    }
                }
            }
        }
    }
}

/* ---- motion -------------------------------------------------------------- */

typedef struct Mv {
    int16_t x, y;                   /* quarter luma samples */
} Mv;

static bool mv_eq(Mv a, Mv b)
{
    return a.x == b.x && a.y == b.y;
}

/* the CU (8x8) index of luma (x, y), or -1 outside the picture */
static int cu_at(const Pic *p, int x, int y)
{
    if (x < 0 || y < 0 || x >= p->w || y >= p->h) {
        return -1;
    }
    return (y >> 3) * (p->w >> 3) + (x >> 3);
}

/* 6.4.2: the PU at (x, y) available for motion prediction: in the picture,
 * decoded before (z-scan), not intra; its vector */
static bool pu_avail(const Pic *p, int x, int y, Mv *mv)
{
    const int cu = cu_at(p, x, y);

    if (cu < 0 || !p->done[cu] || p->intra[cu]) {
        return false;
    }
    mv->x = p->mvx[cu];
    mv->y = p->mvy[cu];
    return true;
}

/* 8.5.3.2.8: the collocated vector, from the reference picture's motion */
static bool col_mv(const Pic *p, int x0, int y0, Mv *mv)
{
    const HevcEncRef *r = p->ref;
    const int cw = p->w >> 3;
    int xc = x0 + 8, yc = y0 + 8, k;

    if (!p->prm->tmvp || !r || !r->intra) {
        return false;
    }
    for (k = 0; k < 2; k++) {
        if (k == 0) {
            /* bottom right: in the same CTB row, inside the picture */
            if ((y0 >> p->log2_ctb) != (yc >> p->log2_ctb) || yc >= p->h ||
                xc >= p->w) {
                continue;
            }
        } else {
            xc = x0 + 4;
            yc = y0 + 4;
        }
        xc = (xc >> 4) << 4;
        yc = (yc >> 4) << 4;
        const int cu = (yc >> 3) * cw + (xc >> 3);
        if (!r->intra[cu]) {
            mv->x = r->mvx[cu];
            mv->y = r->mvy[cu];
            return true;
        }
    }
    return false;
}

/*
 * 8.5.3.2.2-5: the merge candidates of the 8x8 PU at (x0, y0).  The
 * comparisons are with the neighbours as available (6.4.2), whatever their
 * own comparisons decided; B2 is not looked at once four are in.
 */
static int merge_list(const Pic *p, int x0, int y0, Mv *list)
{
    const int maxc = p->prm->max_merge_cand ? p->prm->max_merge_cand : 5;
    Mv a1, b1, b0, a0, b2, col;
    const bool va1 = pu_avail(p, x0 - 1, y0 + 7, &a1);
    const bool vb1 = pu_avail(p, x0 + 7, y0 - 1, &b1);
    const bool vb0 = pu_avail(p, x0 + 8, y0 - 1, &b0);
    const bool va0 = pu_avail(p, x0 - 1, y0 + 8, &a0);
    const bool vb2 = pu_avail(p, x0 - 1, y0 - 1, &b2);
    bool fa1 = va1, fb1, fb0, fa0, fb2;
    int n = 0;

    fb1 = vb1 && !(va1 && mv_eq(a1, b1));
    fb0 = vb0 && !(vb1 && mv_eq(b1, b0));
    fa0 = va0 && !(va1 && mv_eq(a1, a0));
    fb2 = vb2 && !(va1 && mv_eq(a1, b2)) && !(vb1 && mv_eq(b1, b2)) &&
          fa0 + fa1 + fb0 + fb1 != 4;
    if (fa1) {
        list[n++] = a1;
    }
    if (fb1) {
        list[n++] = b1;
    }
    if (fb0) {
        list[n++] = b0;
    }
    if (fa0) {
        list[n++] = a0;
    }
    if (fb2) {
        list[n++] = b2;
    }
    if (n < maxc && col_mv(p, x0, y0, &col)) {
        list[n++] = col;
    }
    while (n < maxc) {
        list[n++] = (Mv) { 0, 0 };          /* refIdx 0: one reference */
    }
    return maxc;
}

/*
 * 8.5.3.2.6-8: the two AMVP predictors of the PU at (x0, y0).  Every
 * candidate refers to the one reference, so none is scaled: A is the first
 * of A0, A1; B the first of B0, B1, B2; without an A (isScaledFlag 0, an
 * intra neighbour counting as absent) A takes B's vector, and the list
 * keeps one of the two; then the temporal one, then zeros.
 */
static void amvp_list(const Pic *p, int x0, int y0, Mv *list)
{
    Mv mva = { 0, 0 }, mvb = { 0, 0 }, col;
    bool fa, fb;
    int n = 0;

    fa = pu_avail(p, x0 - 1, y0 + 8, &mva) ||
         pu_avail(p, x0 - 1, y0 + 7, &mva);
    fb = pu_avail(p, x0 + 8, y0 - 1, &mvb) ||
         pu_avail(p, x0 + 7, y0 - 1, &mvb) ||
         pu_avail(p, x0 - 1, y0 - 1, &mvb);
    if (!fa && fb) {
        mva = mvb;
        fa = true;
    }
    if (fa) {
        list[n++] = mva;
    }
    if (fb && !mv_eq(mva, mvb)) {
        list[n++] = mvb;
    }
    if (n < 2 && col_mv(p, x0, y0, &col)) {
        list[n++] = col;
    }
    while (n < 2) {
        list[n++] = (Mv) { 0, 0 };
    }
}

/* the prediction of the CU at (x0, y0) by mv: a copy of the reference,
 * clipped to it (vectors are even luma samples) */
static void mc_cu(const Pic *p, int x0, int y0, Mv mv, uint8_t py[64],
                  uint8_t pc[2][16])
{
    const HevcEncRef *r = p->ref;
    const int dx = mv.x >> 2, dy = mv.y >> 2, cw = p->w / 2, ch = p->h / 2;
    int x, y, comp;

    for (y = 0; y < 8; y++) {
        const int ry = MIN(MAX(y0 + y + dy, 0), p->h - 1);
        for (x = 0; x < 8; x++) {
            py[y * 8 + x] = r->y[ry * p->w + MIN(MAX(x0 + x + dx, 0),
                                                p->w - 1)];
        }
    }
    for (comp = 0; comp < 2; comp++) {
        const uint8_t *pl = comp ? r->cr : r->cb;
        for (y = 0; y < 4; y++) {
            const int ry = MIN(MAX(y0 / 2 + y + dy / 2, 0), ch - 1);
            for (x = 0; x < 4; x++) {
                pc[comp][y * 4 + x] =
                    pl[ry * cw + MIN(MAX(x0 / 2 + x + dx / 2, 0), cw - 1)];
            }
        }
    }
}

static int sad_cu(const Pic *p, int x0, int y0, Mv mv, int best)
{
    const int dx = mv.x >> 2, dy = mv.y >> 2;
    int x, y, s = 0;

    for (y = 0; y < 8; y++) {
        const int ry = MIN(MAX(y0 + y + dy, 0), p->h - 1);
        const uint8_t *src = p->cy + (y0 + y) * p->w + x0;
        for (x = 0; x < 8; x++) {
            s += abs(src[x] - p->ref->y[ry * p->w + MIN(MAX(x0 + x + dx, 0),
                                                        p->w - 1)]);
        }
        if (s >= best) {
            return s;
        }
    }
    return s;
}

/* bits of one mvd component, roughly: for the search's cost */
static int mvd_bits(int d)
{
    int a = abs(d) >> 3, b = 1;

    while (a) {
        b += 2;
        a >>= 1;
    }
    return b;
}

/* ---- the CU -------------------------------------------------------------- */

typedef struct InterRes {
    int ly[64], lcb[16], lcr[16];       /* levels */
    bool cy, ccb, ccr;                  /* coded */
} InterRes;

/* the residual of prediction py / pc against the source, transformed and
 * quantised */
static void inter_residual(Pic *p, int x0, int y0, const uint8_t py[64],
                           const uint8_t pc[2][16], InterRes *r)
{
    const int cs = p->w / 2;
    const int qpc = chroma_qp(MIN(MAX(p->qp + p->cqp_offset, 0), 57));
    int res[64], i, x, y, comp;

    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
            res[y * 8 + x] = p->cy[(y0 + y) * p->w + x0 + x] - py[y * 8 + x];
        }
    }
    fwd_quant(p, res, 3, p->qp, true, r->ly);
    r->cy = false;
    for (i = 0; i < 64; i++) {
        r->cy |= r->ly[i] != 0;
    }
    for (comp = 0; comp < 2; comp++) {
        const uint8_t *src = comp ? p->ccr : p->ccb;
        int *lev = comp ? r->lcr : r->lcb;
        bool nz = false;

        for (y = 0; y < 4; y++) {
            for (x = 0; x < 4; x++) {
                res[y * 4 + x] = src[(y0 / 2 + y) * cs + x0 / 2 + x] -
                                 pc[comp][y * 4 + x];
            }
        }
        fwd_quant(p, res, 2, qpc, true, lev);
        for (i = 0; i < 16; i++) {
            nz |= lev[i] != 0;
        }
        if (comp) {
            r->ccr = nz;
        } else {
            r->ccb = nz;
        }
    }
}

/* the reconstruction: prediction plus the decoder's residual */
static void inter_recon(Pic *p, int x0, int y0, const uint8_t py[64],
                        const uint8_t pc[2][16], const InterRes *r)
{
    const int cs = p->w / 2;
    const int qpc = chroma_qp(MIN(MAX(p->qp + p->cqp_offset, 0), 57));
    int res[64] = { 0 }, x, y, comp;

    if (r->cy) {
        dequant_inv(p, r->ly, 3, p->qp, true, res);
    }
    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
            p->y[(y0 + y) * p->w + x0 + x] = clip1(py[y * 8 + x] +
                                                   res[y * 8 + x]);
        }
    }
    for (comp = 0; comp < 2; comp++) {
        uint8_t *pl = comp ? p->cr : p->cb;
        const bool coded = comp ? r->ccr : r->ccb;

        memset(res, 0, sizeof(res));
        if (coded) {
            dequant_inv(p, comp ? r->lcr : r->lcb, 2, qpc, true, res);
        }
        for (y = 0; y < 4; y++) {
            for (x = 0; x < 4; x++) {
                pl[(y0 / 2 + y) * cs + x0 / 2 + x] =
                    clip1(pc[comp][y * 4 + x] + res[y * 4 + x]);
            }
        }
    }
}

/* the luma SAD the reconstruction from py and r would leave */
static int inter_recon_sad(const Pic *p, int x0, int y0, const uint8_t py[64],
                           const InterRes *r)
{
    int res[64] = { 0 }, x, y, e = 0;

    if (r->cy) {
        dequant_inv(p, r->ly, 3, p->qp, true, res);
    }
    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
            e += abs(p->cy[(y0 + y) * p->w + x0 + x] -
                     clip1(py[y * 8 + x] + res[y * 8 + x]));
        }
    }
    return e;
}

/* the SSE of a prediction against the source, luma and chroma */
static int pred_sse(const Pic *p, int x0, int y0, const uint8_t py[64],
                    const uint8_t pc[2][16])
{
    const int cs = p->w / 2;
    int x, y, comp, e = 0;

    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
            const int d = p->cy[(y0 + y) * p->w + x0 + x] - py[y * 8 + x];
            e += d * d;
        }
    }
    for (comp = 0; comp < 2; comp++) {
        const uint8_t *src = comp ? p->ccr : p->ccb;
        for (y = 0; y < 4; y++) {
            for (x = 0; x < 4; x++) {
                const int d = src[(y0 / 2 + y) * cs + x0 / 2 + x] -
                              pc[comp][y * 4 + x];
                e += d * d;
            }
        }
    }
    return e;
}

/* the SAD an intra DC CU would leave: each 4x4's mean removed */
static int intra_dc_err(const Pic *p, int x0, int y0)
{
    int bx, by, x, y, e = 0;

    for (by = 0; by < 2; by++) {
        for (bx = 0; bx < 2; bx++) {
            int s = 0;
            for (y = 0; y < 4; y++) {
                for (x = 0; x < 4; x++) {
                    s += p->cy[(y0 + by * 4 + y) * p->w + x0 + bx * 4 + x];
                }
            }
            s = (s + 8) >> 4;
            for (y = 0; y < 4; y++) {
                for (x = 0; x < 4; x++) {
                    e += abs(p->cy[(y0 + by * 4 + y) * p->w + x0 + bx * 4 +
                                   x] - s);
                }
            }
        }
    }
    return e;
}

static void put_merge_idx(Pic *p, Cabac *c, int idx)
{
    const int maxc = p->prm->max_merge_cand ? p->prm->max_merge_cand : 5;
    int i;

    if (maxc <= 1) {
        return;
    }
    for (i = 0; i < MIN(idx, maxc - 1); i++) {
        if (i == 0) {
            cabac_decision(c, CTX_MERGE_IDX, 1);
        } else {
            cabac_bypass(c, 1);
        }
    }
    if (idx < maxc - 1) {
        if (idx == 0) {
            cabac_decision(c, CTX_MERGE_IDX, 0);
        } else {
            cabac_bypass(c, 0);
        }
    }
}

/* mvd_coding() (7.3.8.9, 9.3.4.2) */
static void put_mvd(Cabac *c, int dx, int dy)
{
    const int ax = abs(dx), ay = abs(dy);

    cabac_decision(c, CTX_MVD_G0, ax > 0);
    cabac_decision(c, CTX_MVD_G0, ay > 0);
    if (ax > 0) {
        cabac_decision(c, CTX_MVD_G1, ax > 1);
    }
    if (ay > 0) {
        cabac_decision(c, CTX_MVD_G1, ay > 1);
    }
    if (ax > 0) {
        if (ax > 1) {
            cabac_egk(c, ax - 2, 1);
        }
        cabac_bypass(c, dx < 0);
    }
    if (ay > 0) {
        if (ay > 1) {
            cabac_egk(c, ay - 2, 1);
        }
        cabac_bypass(c, dy < 0);
    }
}

/* the CU's motion, for the neighbours' prediction and the picture after */
static void cu_motion(Pic *p, int cu, bool intra, Mv mv, bool skip)
{
    p->intra[cu] = intra;
    p->mvx[cu] = intra ? 0 : mv.x;
    p->mvy[cu] = intra ? 0 : mv.y;
    p->skip[cu] = skip;
}

/* one 8x8 CU of a P slice, inter coding */
static void encode_cu_inter(Pic *p, Cabac *c, int x0, int y0)
{
    const int cw = p->w >> 3, cu = (y0 >> 3) * cw + (x0 >> 3);
    const int ctx = (x0 > 0 && p->skip[cu - 1]) + (y0 > 0 && p->skip[cu - cw]);
    const int lambda = 1 << MAX(0, (p->qp - 12) / 6);
    Mv merge[5], amvp[2], best = { 0, 0 };
    uint8_t py[64], pc[2][16];
    InterRes r;
    int nm, i, best_cost = INT_MAX, skip_idx = -1, skip_sse = INT_MAX;

    if (!p->ref) {
        /* nothing to refer to: intra (the decoder's picture unknown) */
        p->force_intra = true;
        encode_cu(p, c, x0, y0, p->qp, true);
        p->force_intra = false;
        cu_motion(p, cu, true, best, false);
        p->cbfy[cu] = 1;
        return;
    }
    nm = merge_list(p, x0, y0, merge);
    amvp_list(p, x0, y0, amvp);

    if (p->prm->all_skip) {
        /* the stream buffer's room is all but gone: skip, candidate 0 */
        cabac_decision(c, CTX_SKIP + ctx, 1);
        put_merge_idx(p, c, 0);
        mc_cu(p, x0, y0, merge[0], py, pc);
        memset(&r, 0, sizeof(r));
        inter_recon(p, x0, y0, py, pc, &r);
        cu_motion(p, cu, false, merge[0], true);
        p->cbfy[cu] = 0;
        p->cuqp[cu] = p->qp_delta_coded ? p->qp : p->qg_pred;
        p->qp_prev = p->cuqp[cu];
        p->done[cu] = 1;
        return;
    }

    /* skip: a merge candidate whose prediction leaves nothing to code */
    for (i = 0; i < nm; i++) {
        int e;

        if ((merge[i].x & 7) || (merge[i].y & 7)) {
            continue;                       /* not full-pel chroma */
        }
        mc_cu(p, x0, y0, merge[i], py, pc);
        inter_residual(p, x0, y0, py, pc, &r);
        if (r.cy || r.ccb || r.ccr) {
            continue;
        }
        e = pred_sse(p, x0, y0, py, pc);
        if (e < skip_sse) {
            skip_sse = e;
            skip_idx = i;
        }
    }
    if (skip_idx >= 0) {
        cabac_decision(c, CTX_SKIP + ctx, 1);
        put_merge_idx(p, c, skip_idx);
        mc_cu(p, x0, y0, merge[skip_idx], py, pc);
        memset(&r, 0, sizeof(r));
        inter_recon(p, x0, y0, py, pc, &r);
        cu_motion(p, cu, false, merge[skip_idx], true);
        p->cbfy[cu] = 0;
        p->cuqp[cu] = p->qp_delta_coded ? p->qp : p->qg_pred;
        p->qp_prev = p->cuqp[cu];
        p->done[cu] = 1;
        return;
    }

    /* motion search: the candidates, a grid, then a diamond (2 samples) */
    {
        Mv cand[8];
        int nc = 0, it;

        cand[nc++] = (Mv) { 0, 0 };
        cand[nc++] = amvp[0];
        cand[nc++] = amvp[1];
        for (i = 0; i < nm && nc < 8; i++) {
            cand[nc++] = merge[i];
        }
        for (i = 0; i < nc; i++) {
            const Mv m = { (int16_t)(((cand[i].x + 4) >> 3) << 3),
                           (int16_t)(((cand[i].y + 4) >> 3) << 3) };
            const int cost = sad_cu(p, x0, y0, m, best_cost) + lambda *
                (mvd_bits(m.x - amvp[0].x) + mvd_bits(m.y - amvp[0].y));
            if (cost < best_cost) {
                best_cost = cost;
                best = m;
            }
        }
        if (best_cost > 256) {
            Mv cb = best;
            for (int gy = -16; gy <= 16; gy += 4) {
                for (int gx = -16; gx <= 16; gx += 4) {
                    const Mv m = { (int16_t)(gx * 4), (int16_t)(gy * 4) };
                    const int cost = sad_cu(p, x0, y0, m, best_cost) +
                        lambda * (mvd_bits(m.x - amvp[0].x) +
                                  mvd_bits(m.y - amvp[0].y));
                    if (cost < best_cost) {
                        best_cost = cost;
                        cb = m;
                    }
                }
            }
            best = cb;
        }
        for (it = 0; it < 16 && best_cost; it++) {
            static const int8_t dia[4][2] = {
                { -8, 0 }, { 8, 0 }, { 0, -8 }, { 0, 8 },
            };
            Mv nb = best;

            for (i = 0; i < 4; i++) {
                const Mv m = { (int16_t)(best.x + dia[i][0]),
                               (int16_t)(best.y + dia[i][1]) };
                int cost;

                if (abs(m.x) > 256 || abs(m.y) > 256) {
                    continue;
                }
                cost = sad_cu(p, x0, y0, m, best_cost) + lambda *
                    (mvd_bits(m.x - amvp[0].x) + mvd_bits(m.y - amvp[0].y));
                if (cost < best_cost) {
                    best_cost = cost;
                    nb = m;
                }
            }
            if (mv_eq(nb, best)) {
                break;
            }
            best = nb;
        }
    }

    mc_cu(p, x0, y0, best, py, pc);
    inter_residual(p, x0, y0, py, pc, &r);

    /*
     * Intra DC only where even the residual leaves the block worse than a
     * DC block would (the residual carries the detail of whatever the
     * prediction missed, a scene change too)
     */
    if (inter_recon_sad(p, x0, y0, py, &r) > 2 * intra_dc_err(p, x0, y0) +
        512) {
        p->force_intra = true;
        encode_cu(p, c, x0, y0, p->qp, true);
        p->force_intra = false;
        cu_motion(p, cu, true, best, false);
        p->cbfy[cu] = 1;
        return;
    }
    {
        int midx = -1, aidx = 0;
        const bool any = r.cy || r.ccb || r.ccr;

        for (i = 0; i < nm; i++) {
            if (mv_eq(merge[i], best)) {
                midx = i;
                break;
            }
        }
        if (midx >= 0 && !any) {
            /* the search found a merge candidate and nothing to code */
            cabac_decision(c, CTX_SKIP + ctx, 1);
            put_merge_idx(p, c, midx);
            inter_recon(p, x0, y0, py, pc, &r);
            cu_motion(p, cu, false, best, true);
            p->cbfy[cu] = 0;
            p->cuqp[cu] = p->qp_delta_coded ? p->qp : p->qg_pred;
            p->qp_prev = p->cuqp[cu];
            p->done[cu] = 1;
            return;
        }
        cabac_decision(c, CTX_SKIP + ctx, 0);
        cabac_decision(c, CTX_PRED_MODE, 0);            /* inter */
        cabac_decision(c, CTX_PART_MODE, 1);            /* 2Nx2N */
        /* merged when the vector is a candidate's and there is a residual
         * (a merged 2Nx2N CU infers rqt_root_cbf 1) */
        if (midx >= 0 && any) {
            cabac_decision(c, CTX_MERGE_FLAG, 1);
            put_merge_idx(p, c, midx);
        } else {
            cabac_decision(c, CTX_MERGE_FLAG, 0);
            if (!mv_eq(amvp[0], best) && mv_eq(amvp[1], best)) {
                aidx = 1;
            } else if (!mv_eq(amvp[0], best) &&
                       mvd_bits(best.x - amvp[1].x) +
                       mvd_bits(best.y - amvp[1].y) <
                       mvd_bits(best.x - amvp[0].x) +
                       mvd_bits(best.y - amvp[0].y)) {
                aidx = 1;
            }
            put_mvd(c, best.x - amvp[aidx].x, best.y - amvp[aidx].y);
            cabac_decision(c, CTX_MVP, aidx);
            cabac_decision(c, CTX_ROOT_CBF, any);
        }
        if (any) {
            /* transform_tree: no split, cbf_cb, cbf_cr, cbf_luma (inferred
             * 1 when neither chroma has coefficients) */
            if (p->prm->trafo_depth_inter > 0) {
                cabac_decision(c, CTX_SPLIT_TRAFO + 5 - 3, 0);
            }
            cabac_decision(c, CTX_CBF_CHROMA + 0, r.ccb);
            cabac_decision(c, CTX_CBF_CHROMA + 0, r.ccr);
            if (r.ccb || r.ccr) {
                cabac_decision(c, CTX_CBF_LUMA + 1, r.cy);
            } else {
                r.cy = true;        /* inferred: and it has coefficients */
            }
            if (!p->qp_delta_coded) {
                cu_qp_delta(c, p->qp - p->qg_pred);
                p->qp_delta_coded = true;
            }
            if (r.cy) {
                residual_full(p, c, r.ly, 3, 0);
            }
            if (r.ccb) {
                residual_full(p, c, r.lcb, 2, 1);
            }
            if (r.ccr) {
                residual_full(p, c, r.lcr, 2, 2);
            }
        }
        inter_recon(p, x0, y0, py, pc, &r);
        cu_motion(p, cu, false, best, false);
        p->cbfy[cu] = r.cy;
        p->cuqp[cu] = p->qp_delta_coded ? p->qp : p->qg_pred;
        p->qp_prev = p->cuqp[cu];
        p->done[cu] = 1;
    }
}

/* ---- deblocking with inter CUs (8.7.2) ----------------------------------- */

static int bs_inter(const Pic *p, int q, int pp)
{
    if (p->intra[q] || p->intra[pp]) {
        return 2;
    }
    if (p->cbfy[q] || p->cbfy[pp]) {
        return 1;
    }
    if (abs(p->mvx[q] - p->mvx[pp]) >= 4 || abs(p->mvy[q] - p->mvy[pp]) >= 4) {
        return 1;
    }
    return 0;
}

static void deblock_inter(Pic *p, const HevcEncParams *prm)
{
    const int cw = p->w >> 3, ch = p->h >> 3, cs = p->w / 2;

    for (int dir = 0; dir < 2; dir++) {         /* 0: vertical edges */
        for (int by = dir; by < ch; by++) {
            for (int bx = !dir; bx < cw; bx++) {
                const int q = by * cw + bx, pp = dir ? q - cw : q - 1;
                const int bs = bs_inter(p, q, pp);

                if (!bs) {
                    continue;
                }
                const int qpl = (p->cuqp[q] + p->cuqp[pp] + 1) >> 1;
                const int beta = beta_table[MIN(MAX(qpl + 2 *
                                         prm->beta_offset_div2, 0), 51)];
                const int tc = tc_table[MIN(MAX(qpl + 2 * (bs - 1) +
                                         2 * prm->tc_offset_div2, 0), 53)];
                const int xs = dir ? p->w : 1, ls = dir ? 1 : p->w;
                uint8_t *pix = &p->y[by * 8 * p->w + bx * 8];

                deblock_luma(pix, xs, ls, beta, tc);
                deblock_luma(pix + 4 * ls, xs, ls, beta, tc);
                if (bs < 2 || ((dir ? by : bx) & 1)) {
                    continue;
                }
                const int qpc = chroma_qp(qpl + prm->chroma_qp_offset);
                const int tcc = tc_table[MIN(MAX(qpc + 2 +
                                          2 * prm->tc_offset_div2, 0), 53)];
                const int o = by * 4 * cs + bx * 4;
                deblock_chroma(&p->cb[o], dir ? cs : 1, dir ? 1 : cs, 4, tcc);
                deblock_chroma(&p->cr[o], dir ? cs : 1, dir ? 1 : cs, 4, tcc);
            }
        }
    }
}
