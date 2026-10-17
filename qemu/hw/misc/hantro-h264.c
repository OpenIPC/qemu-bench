/*
 * A minimal H.264 intra encoder for the Hantro H1 model (xm-h1).
 *
 * The H1 is driven by Hantro's own encoder library, which writes the SPS and
 * PPS in software and has the hardware produce the slices.  So the slices
 * here are made to fit whatever SPS/PPS the library chose: they are parsed
 * from the bytes it left in the stream buffer, and the slice header follows
 * them (frame_num and POC widths, POC type, PPS id, deblocking control).
 *
 * Every picture is coded as an I slice (an IDR when the H1 was asked for an
 * intra picture, otherwise a non-IDR reference picture), every macroblock
 * as I_16x16 with DC prediction, and only DC coefficients are sent: the
 * luma DC Hadamard block, and chroma DC when it is non-zero.  The image is
 * therefore reproduced at 4x4-block resolution, which is plenty for a test
 * pattern and keeps the encoder small; it reconstructs exactly as a decoder
 * does (ITU-T H.264 8.3.3, 8.3.4, 8.5.10, 8.5.11), so intra prediction never
 * drifts.
 *
 * Entropy coding is CABAC when the PPS asks for it, which majestic's
 * Main-profile setting does.  The arithmetic coder and the residual block
 * coding follow x264 (encoder/cabac.c, common/cabac.c; x264 project,
 * GPL-2.0-or-later); the tables are in hantro-h264-tables.h.  CAVLC PPSs
 * are not supported yet.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hantro-h264.h"
#include "hantro-h264-tables.h"

/* ---- bit reader (RBSP) ----------------------------------------------- */

typedef struct BitReader {
    const uint8_t *p;
    size_t n, pos;              /* pos in bits */
} BitReader;

static int br_bit(BitReader *r)
{
    if (r->pos >= r->n * 8) {
        r->pos++;
        return 0;
    }
    const int b = (r->p[r->pos >> 3] >> (7 - (r->pos & 7))) & 1;
    r->pos++;
    return b;
}

static uint32_t br_bits(BitReader *r, int n)
{
    uint32_t v = 0;

    while (n--) {
        v = (v << 1) | br_bit(r);
    }
    return v;
}

static uint32_t br_ue(BitReader *r)
{
    int zeros = 0;

    while (!br_bit(r) && zeros < 32) {
        zeros++;
    }
    return ((1u << zeros) - 1) + br_bits(r, zeros);
}

static int32_t br_se(BitReader *r)
{
    const uint32_t k = br_ue(r);

    return k & 1 ? (int32_t)((k + 1) / 2) : -(int32_t)(k / 2);
}

/* NAL payload without emulation-prevention bytes */
static size_t unescape(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t i, o = 0;
    int zeros = 0;

    for (i = 0; i < n; i++) {
        if (zeros >= 2 && in[i] == 3) {
            zeros = 0;
            continue;
        }
        zeros = in[i] ? 0 : zeros + 1;
        out[o++] = in[i];
    }
    return o;
}

static void parse_sps(H264Enc *e, const uint8_t *rbsp, size_t n)
{
    BitReader r = { rbsp, n, 0 };
    const int profile = br_bits(&r, 8);

    br_bits(&r, 16);                            /* constraints, level */
    br_ue(&r);                                  /* seq_parameter_set_id */
    if (profile == 100 || profile == 110 || profile == 122 ||
        profile == 244 || profile == 44 || profile == 83 || profile == 86 ||
        profile == 118 || profile == 128) {
        if (br_ue(&r) == 3) {                   /* chroma_format_idc */
            br_bit(&r);
        }
        br_ue(&r);
        br_ue(&r);
        br_bit(&r);
        if (br_bit(&r)) {                       /* scaling matrices */
            e->unsupported = true;
        }
    }
    e->log2_max_frame_num = br_ue(&r) + 4;
    e->poc_type = br_ue(&r);
    if (e->poc_type == 0) {
        e->log2_max_poc_lsb = br_ue(&r) + 4;
    } else if (e->poc_type == 1) {
        e->unsupported = true;
    }
    br_ue(&r);                                  /* max_num_ref_frames */
    br_bit(&r);
    br_ue(&r);                                  /* width */
    br_ue(&r);                                  /* height */
    if (!br_bit(&r)) {                          /* frame_mbs_only_flag */
        e->unsupported = true;
    }
    e->have_sps = true;
}

static void parse_pps(H264Enc *e, const uint8_t *rbsp, size_t n)
{
    BitReader r = { rbsp, n, 0 };

    e->pps_id = br_ue(&r);
    br_ue(&r);                                  /* sps id */
    e->cabac = br_bit(&r);
    e->bottom_field_poc_present = br_bit(&r);
    if (br_ue(&r)) {                            /* slice groups */
        e->unsupported = true;
    }
    br_ue(&r);
    br_ue(&r);
    br_bit(&r);
    br_bits(&r, 2);
    e->pic_init_qp = 26 + br_se(&r);
    br_se(&r);
    e->chroma_qp_offset = br_se(&r);
    e->deblocking_control = br_bit(&r);
    br_bit(&r);                                 /* constrained_intra_pred */
    e->redundant_pic_cnt_present = br_bit(&r);
    e->have_pps = true;
}

void h264_enc_parse_headers(H264Enc *e, const uint8_t *buf, size_t len)
{
    size_t i = 0;

    while (i + 4 < len) {
        if (buf[i] || buf[i + 1] || buf[i + 2] != 1) {
            i++;
            continue;
        }
        size_t s = i + 3, j = s;
        while (j + 2 < len && !(buf[j] == 0 && buf[j + 1] == 0 &&
                                (buf[j + 2] == 1 || buf[j + 2] == 0))) {
            j++;
        }
        if (j + 2 >= len) {
            j = len;
        }
        if (j > s + 1) {
            g_autofree uint8_t *rbsp = g_malloc(j - s);
            const size_t n = unescape(buf + s + 1, j - s - 1, rbsp);
            switch (buf[s] & 0x1f) {
            case 7:
                parse_sps(e, rbsp, n);
                break;
            case 8:
                parse_pps(e, rbsp, n);
                break;
            }
        }
        i = j;
    }
}

/* ---- bit writer (header) and NAL framing ------------------------------ */

typedef struct BitWriter {
    uint8_t *p;
    size_t cap, n;
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
    const uint32_t x = v + 1;
    const int len = 32 - __builtin_clz(x);

    bw_bits(w, 0, len - 1);
    bw_bits(w, x, len);
}

static void bw_se(BitWriter *w, int32_t v)
{
    bw_ue(w, v <= 0 ? -2 * v : 2 * v - 1);
}

/* ---- CABAC (after x264) ----------------------------------------------- */

typedef struct Cabac {
    uint8_t state[460];
    int low, range, queue, outstanding;
    uint8_t *p, *start, *end;
} Cabac;

static void cabac_init(Cabac *c, int qp, uint8_t *p, uint8_t *end)
{
    int i;

    for (i = 0; i < 460; i++) {
        const int st = MIN(MAX(((h264_cabac_init_I[i][0] * qp) >> 4) +
                               h264_cabac_init_I[i][1], 1), 126);
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

/* end_of_slice_flag = 1, then flush: x264_cabac_encode_flush() */
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

/* Exp-Golomb (k = 0) in bypass bins: the coeff_abs_level suffix. */
static void cabac_ue_bypass(Cabac *c, int v)
{
    int k = 0;

    while (v >= (1 << k)) {
        cabac_bypass(c, 1);
        v -= 1 << k;
        k++;
    }
    cabac_bypass(c, 0);
    while (k--) {
        cabac_bypass(c, (v >> k) & 1);
    }
}

static const uint8_t level1_ctx[8] = { 1, 2, 3, 4, 0, 0, 0, 0 };
static const uint8_t levelgt1_ctx[8] = { 5, 5, 5, 5, 6, 7, 8, 9 };
static const uint8_t level_transition[2][8] = {
    { 1, 2, 3, 3, 4, 5, 6, 7 },
    { 4, 4, 4, 4, 5, 6, 7, 7 },
};

/* residual_block_cabac() for a block known to have a non-zero coefficient:
 * significance map, then levels from last to first. */
static void cabac_residual(Cabac *c, int sig_ctx, int last_ctx, int lvl_ctx,
                           const int *l, int count)
{
    int coeffs[16], n = 0, last = count - 1, i, node = 0;

    while (last > 0 && !l[last]) {
        last--;
    }
    for (i = 0; i < count; i++) {
        if (i == count - 1) {               /* implied by its position */
            coeffs[n++] = l[i];
            break;
        }
        if (l[i]) {
            coeffs[n++] = l[i];
            cabac_decision(c, sig_ctx + i, 1);
            cabac_decision(c, last_ctx + i, i == last);
            if (i == last) {
                break;
            }
        } else {
            cabac_decision(c, sig_ctx + i, 0);
        }
    }
    while (n--) {
        const int v = coeffs[n], a = abs(v);
        int ctx = lvl_ctx + level1_ctx[node];

        if (a > 1) {
            cabac_decision(c, ctx, 1);
            ctx = lvl_ctx + levelgt1_ctx[node];
            for (i = MIN(a, 15) - 2; i > 0; i--) {
                cabac_decision(c, ctx, 1);
            }
            if (a < 15) {
                cabac_decision(c, ctx, 0);
            } else {
                cabac_ue_bypass(c, a - 15);
            }
            node = level_transition[1][node];
        } else {
            cabac_decision(c, ctx, 0);
            node = level_transition[0][node];
        }
        cabac_bypass(c, v < 0);
    }
}

/* ---- transforms and quantisation -------------------------------------- */

static const int mf00[6] = { 13107, 11916, 10082, 9362, 8192, 7282 };
static const int v00[6] = { 10, 11, 13, 14, 16, 18 };
static const uint8_t zigzag4x4[16] = {
    0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15,
};
static const uint8_t qpc_table[22] = {
    29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38,
    39, 39, 39, 39,
};

static int chroma_qp(int qp, int offset)
{
    const int i = MIN(MAX(qp + offset, 0), 51);

    return i < 30 ? i : qpc_table[i - 30];
}

static int quant(int v, int qp, int shift_extra)
{
    const int qbits = 15 + qp / 6 + shift_extra;
    const int f = (1 << qbits) / 3;                     /* intra rounding */
    const int a = (abs(v) * mf00[qp % 6] + f) >> qbits;

    return v < 0 ? -a : a;
}

/* 4x4 Hadamard, in place, rows then columns */
static void hadamard4(int m[16])
{
    int i, t[16];

    for (i = 0; i < 4; i++) {
        const int *r = m + 4 * i;
        const int s01 = r[0] + r[1], d01 = r[0] - r[1];
        const int s23 = r[2] + r[3], d23 = r[2] - r[3];
        t[4 * i + 0] = s01 + s23;
        t[4 * i + 1] = s01 - s23;
        t[4 * i + 2] = d01 - d23;
        t[4 * i + 3] = d01 + d23;
    }
    for (i = 0; i < 4; i++) {
        const int s01 = t[i] + t[4 + i], d01 = t[i] - t[4 + i];
        const int s23 = t[8 + i] + t[12 + i], d23 = t[8 + i] - t[12 + i];
        m[i] = s01 + s23;
        m[4 + i] = s01 - s23;
        m[8 + i] = d01 - d23;
        m[12 + i] = d01 + d23;
    }
}

static uint8_t clip1(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* ---- the encoder -------------------------------------------------------- */

typedef struct MbInfo {
    bool luma_dc_cbf, cb_dc_cbf, cr_dc_cbf, chroma_coded;
} MbInfo;

typedef struct Frame {
    int mbw, mbh;
    int w, h;
    uint8_t *y, *cb, *cr;           /* reconstruction */
    const uint8_t *sy, *suv;        /* source: NV12 */
    int sys, scs;
} Frame;

static int pred16_dc(Frame *f, int mx, int my)
{
    const int x0 = mx * 16, y0 = my * 16;
    int s = 0, i;

    if (mx && my) {
        for (i = 0; i < 16; i++) {
            s += f->y[(y0 - 1) * f->w + x0 + i] + f->y[(y0 + i) * f->w + x0 - 1];
        }
        return (s + 16) >> 5;
    }
    if (mx) {
        for (i = 0; i < 16; i++) {
            s += f->y[(y0 + i) * f->w + x0 - 1];
        }
        return (s + 8) >> 4;
    }
    if (my) {
        for (i = 0; i < 16; i++) {
            s += f->y[(y0 - 1) * f->w + x0 + i];
        }
        return (s + 8) >> 4;
    }
    return 128;
}

/* 8.3.4.1-3, one 4x4 chroma block at (xo, yo) of the MB */
static int predc_dc(const uint8_t *p, int stride, int mx, int my, int xo,
                    int yo)
{
    const int x0 = mx * 8 + xo, y0 = my * 8 + yo;
    const bool top = my > 0, left = mx > 0;
    int st = 0, sl = 0, i;

    /* neighbours are the MB's, not the block's: row my * 8 - 1 and
     * column mx * 8 - 1, at the block's x / y */
    for (i = 0; i < 4 && top; i++) {
        st += p[(my * 8 - 1) * stride + x0 + i];
    }
    for (i = 0; i < 4 && left; i++) {
        sl += p[(y0 + i) * stride + mx * 8 - 1];
    }
    if ((xo == 0 && yo == 0) || (xo > 0 && yo > 0)) {
        if (top && left) {
            return (st + sl + 4) >> 3;
        }
        if (left) {
            return (sl + 2) >> 2;
        }
        if (top) {
            return (st + 2) >> 2;
        }
        return 128;
    }
    if (xo > 0) {                                   /* yo == 0 */
        if (top) {
            return (st + 2) >> 2;
        }
        if (left) {
            return (sl + 2) >> 2;
        }
        return 128;
    }
    if (left) {                                     /* xo == 0, yo > 0 */
        return (sl + 2) >> 2;
    }
    if (top) {
        return (st + 2) >> 2;
    }
    return 128;
}

static uint8_t src_y(Frame *f, int x, int y)
{
    return f->sy[y * f->sys + x];
}

static uint8_t src_c(Frame *f, int comp, int x, int y)
{
    return f->suv[y * f->scs + 2 * x + comp];
}

static void encode_mb(Frame *f, Cabac *c, MbInfo *info, int mx, int my,
                      int qp, int qpc)
{
    MbInfo *me = &info[my * f->mbw + mx];
    const MbInfo *ma = mx ? &info[my * f->mbw + mx - 1] : NULL;
    const MbInfo *mb = my ? &info[(my - 1) * f->mbw + mx] : NULL;
    int dc[16], lev[16], i, bx, by, x, y, comp;
    int cdc[2][4], clev[2][4], cpred[2][4];

    /* luma: DC prediction, per-4x4 sums, Hadamard, quantise */
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
        dc[i] /= 2;                                     /* (H W H) / 2 */
        lev[i] = quant(dc[i], qp, 1);
    }

    /* chroma: per-4x4 DC prediction, sums, 2x2 transform, quantise */
    bool chroma_nz = false;
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
    const int cbp_chroma = chroma_nz ? 1 : 0;

    /* mb_type: I_16x16, pred DC (2), cbp luma 0, cbp chroma 0/1 */
    const int ctx0 = 3 + (ma != NULL) + (mb != NULL);
    cabac_decision(c, ctx0, 1);                         /* not I_NxN */
    cabac_terminate0(c);                                /* not I_PCM */
    cabac_decision(c, 3 + 3, 0);                        /* cbp luma != 0 */
    if (!cbp_chroma) {
        cabac_decision(c, 3 + 4, 0);
    } else {
        cabac_decision(c, 3 + 4, 1);
        cabac_decision(c, 3 + 5, 0);                    /* chroma == 2 */
    }
    cabac_decision(c, 3 + 6, 1);                        /* pred 2 = 0b10 */
    cabac_decision(c, 3 + 7, 0);
    cabac_decision(c, 64, 0);                           /* chroma pred: DC */
    cabac_decision(c, 60, 0);                           /* mb_qp_delta = 0 */

    /* Intra16x16DCLevel, ctxBlockCat 0 */
    int scan[16], nz = 0;
    for (i = 0; i < 16; i++) {
        scan[i] = lev[zigzag4x4[i]];
        nz |= scan[i] != 0;
    }
    const int cbf_a = ma ? ma->luma_dc_cbf : 1, cbf_b = mb ? mb->luma_dc_cbf : 1;
    cabac_decision(c, 85 + cbf_a + 2 * cbf_b, nz != 0);
    if (nz) {
        cabac_residual(c, 105 + 0, 166 + 0, 227 + 0, scan, 16);
    }
    me->luma_dc_cbf = nz != 0;

    /* chroma DC, ctxBlockCat 3, Cb then Cr */
    me->chroma_coded = cbp_chroma;
    for (comp = 0; comp < 2 && cbp_chroma; comp++) {
        const int cnz = clev[comp][0] || clev[comp][1] || clev[comp][2] ||
                        clev[comp][3];
        const int a = ma ? (ma->chroma_coded ? (comp ? ma->cr_dc_cbf
                                                     : ma->cb_dc_cbf) : 0) : 1;
        const int b = mb ? (mb->chroma_coded ? (comp ? mb->cr_dc_cbf
                                                     : mb->cb_dc_cbf) : 0) : 1;
        cabac_decision(c, 97 + a + 2 * b, cnz);
        if (cnz) {
            cabac_residual(c, 105 + 44, 166 + 44, 227 + 30, clev[comp], 4);
        }
        if (comp) {
            me->cr_dc_cbf = cnz;
        } else {
            me->cb_dc_cbf = cnz;
        }
    }
    if (!cbp_chroma) {
        me->cb_dc_cbf = me->cr_dc_cbf = false;
        memset(clev, 0, sizeof(clev));
    }

    /* reconstruct, exactly as a decoder (8.5.10, 8.5.11) */
    int f16[16];
    for (i = 0; i < 16; i++) {
        f16[i] = lev[i];
    }
    hadamard4(f16);
    const int ls = 16 * v00[qp % 6];
    for (by = 0; by < 4; by++) {
        for (bx = 0; bx < 4; bx++) {
            const int fv = f16[by * 4 + bx];
            const int d = qp >= 36 ? (fv * ls) << (qp / 6 - 6)
                                   : (fv * ls + (1 << (5 - qp / 6))) >>
                                     (6 - qp / 6);
            const int r = (d + 32) >> 6;
            for (y = 0; y < 4; y++) {
                for (x = 0; x < 4; x++) {
                    f->y[(my * 16 + by * 4 + y) * f->w + mx * 16 + bx * 4 + x] =
                        clip1(pred + r);
                }
            }
        }
    }
    const int lsc = 16 * v00[qpc % 6];
    for (comp = 0; comp < 2; comp++) {
        uint8_t *rp = comp ? f->cr : f->cb;
        const int *l = clev[comp];
        const int fc[4] = { l[0] + l[1] + l[2] + l[3], l[0] - l[1] + l[2] - l[3],
                            l[0] + l[1] - l[2] - l[3], l[0] - l[1] - l[2] + l[3] };
        for (i = 0; i < 4; i++) {
            const int d = ((fc[i] * lsc) << (qpc / 6)) >> 5;
            const int r = (d + 32) >> 6;
            const int xo = (i & 1) * 4, yo = (i >> 1) * 4;
            for (y = 0; y < 4; y++) {
                for (x = 0; x < 4; x++) {
                    rp[(my * 8 + yo + y) * (f->w / 2) + mx * 8 + xo + x] =
                        clip1(cpred[comp][i] + r);
                }
            }
        }
    }
}

static size_t nal_wrap(uint8_t *out, size_t cap, int ref_idc, int type,
                       const uint8_t *rbsp, size_t n)
{
    size_t o = 0, i;
    int zeros = 0;
#define PUT(b) do { if (o < cap) { out[o] = (b); } o++; } while (0)
    PUT(0); PUT(0); PUT(0); PUT(1);
    PUT((ref_idc << 5) | type);
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

size_t h264_enc_frame(H264Enc *e, bool idr, int qp, int mbw, int mbh,
                      const uint8_t *y, int ys, const uint8_t *uv, int cs,
                      uint8_t *out, size_t cap)
{
    Frame f = { .mbw = mbw, .mbh = mbh, .w = mbw * 16, .h = mbh * 16,
                .sy = y, .suv = uv, .sys = ys, .scs = cs };
    const size_t rbsp_cap = (size_t)mbw * mbh * 512 + 256;
    g_autofree uint8_t *rbsp = g_malloc0(rbsp_cap);
    g_autofree MbInfo *info = g_new0(MbInfo, mbw * mbh);
    BitWriter w = { .p = rbsp, .cap = rbsp_cap };
    Cabac c;
    int mx, my;

    if (!e->have_sps || !e->have_pps || !e->cabac || e->unsupported) {
        return 0;
    }
    qp = MIN(MAX(qp, 0), 51);
    if (idr) {
        e->frame_num = 0;
        e->poc = 0;
        e->idr_pic_id = (e->idr_pic_id + 1) & 0xffff;
    } else {
        e->frame_num = (e->frame_num + 1) & ((1 << e->log2_max_frame_num) - 1);
        e->poc += 2;
    }

    /* slice_header() */
    bw_ue(&w, 0);                                       /* first_mb */
    bw_ue(&w, 7);                                       /* I, all slices */
    bw_ue(&w, e->pps_id);
    bw_bits(&w, e->frame_num, e->log2_max_frame_num);
    if (idr) {
        bw_ue(&w, e->idr_pic_id);
    }
    if (e->poc_type == 0) {
        bw_bits(&w, e->poc & ((1 << e->log2_max_poc_lsb) - 1),
                e->log2_max_poc_lsb);
        if (e->bottom_field_poc_present) {
            bw_se(&w, 0);
        }
    }
    if (e->redundant_pic_cnt_present) {
        bw_ue(&w, 0);
    }
    if (idr) {                                          /* dec_ref_pic_marking */
        bw_bits(&w, 0, 1);
        bw_bits(&w, 0, 1);
    } else {
        bw_bits(&w, 0, 1);
    }
    bw_se(&w, qp - e->pic_init_qp);
    if (e->deblocking_control) {
        bw_ue(&w, 0);
        bw_se(&w, 0);
        bw_se(&w, 0);
    }
    while (w.bits) {                                    /* cabac_alignment */
        bw_bits(&w, 1, 1);
    }

    f.y = g_malloc(f.w * f.h);
    f.cb = g_malloc(f.w * f.h / 4);
    f.cr = g_malloc(f.w * f.h / 4);
    cabac_init(&c, qp, rbsp + w.n, rbsp + rbsp_cap);
    const int qpc = chroma_qp(qp, e->chroma_qp_offset);
    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            encode_mb(&f, &c, info, mx, my, qp, qpc);
            if (mx == mbw - 1 && my == mbh - 1) {
                cabac_flush(&c);
            } else {
                cabac_terminate0(&c);
            }
        }
    }
    g_free(f.y);
    g_free(f.cb);
    g_free(f.cr);

    return nal_wrap(out, cap, idr ? 3 : 2, idr ? 5 : 1, rbsp,
                    c.p - rbsp);
}
