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
 * GPL-2.0-or-later); the tables are in hantro-h264-tables.h.
 *
 * Otherwise CAVLC (ITU-T H.264 9.2), which a Baseline PPS needs (Sofia's,
 * on the second-generation XM53x).  Only two residual blocks ever occur:
 * Intra16x16DCLevel, whose nC is always 0 -- its neighbours' 4x4 blocks
 * carry no AC coefficients, cbp luma being 0 -- and ChromaDCLevel (nC -1).
 * The VLC tables are the standard's (Tables 9-5, 9-7, 9-8, 9-9, 9-10).
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

/* ---- CAVLC (9.2) ------------------------------------------------------- */

/* coeff_token, 0 <= nC < 2 (Table 9-5), [TotalCoeff][TrailingOnes] */
static const uint8_t ct_len0[17][4] = {
    { 1 }, { 6, 2 }, { 8, 6, 3 }, { 9, 8, 7, 5 }, { 10, 9, 8, 6 },
    { 11, 10, 9, 7 }, { 13, 11, 10, 8 }, { 13, 13, 11, 9 },
    { 13, 13, 13, 10 }, { 14, 14, 13, 11 }, { 14, 14, 14, 13 },
    { 15, 15, 14, 14 }, { 15, 15, 15, 14 }, { 16, 15, 15, 15 },
    { 16, 16, 16, 15 }, { 16, 16, 16, 16 }, { 16, 16, 16, 16 },
};
static const uint8_t ct_bits0[17][4] = {
    { 1 }, { 5, 1 }, { 7, 4, 1 }, { 7, 6, 5, 3 }, { 7, 6, 5, 3 },
    { 7, 6, 5, 4 }, { 15, 6, 5, 4 }, { 11, 14, 5, 4 }, { 8, 10, 13, 4 },
    { 15, 14, 9, 4 }, { 11, 10, 13, 12 }, { 15, 14, 9, 12 },
    { 11, 10, 13, 8 }, { 15, 1, 9, 12 }, { 11, 14, 13, 8 },
    { 7, 10, 9, 12 }, { 4, 6, 5, 8 },
};
/* coeff_token, 2 <= nC < 4 and 4 <= nC < 8 (Table 9-5) */
static const uint8_t ct_len1[17][4] = {
    { 2 }, { 6, 2 }, { 6, 5, 3 }, { 7, 6, 6, 4 }, { 8, 6, 6, 4 },
    { 8, 7, 7, 5 }, { 9, 8, 8, 6 }, { 11, 9, 9, 6 }, { 11, 11, 11, 7 },
    { 12, 11, 11, 9 }, { 12, 12, 12, 11 }, { 12, 12, 12, 11 },
    { 13, 13, 13, 12 }, { 13, 13, 13, 13 }, { 13, 14, 13, 13 },
    { 14, 14, 14, 13 }, { 14, 14, 14, 14 },
};
static const uint8_t ct_bits1[17][4] = {
    { 3 }, { 11, 2 }, { 7, 7, 3 }, { 7, 10, 9, 5 }, { 7, 6, 5, 4 },
    { 4, 6, 5, 6 }, { 7, 6, 5, 8 }, { 15, 6, 5, 4 }, { 11, 14, 13, 4 },
    { 15, 10, 9, 4 }, { 11, 14, 13, 12 }, { 8, 10, 9, 8 },
    { 15, 14, 13, 12 }, { 11, 10, 9, 12 }, { 7, 11, 6, 8 },
    { 9, 8, 10, 1 }, { 7, 6, 5, 4 },
};
static const uint8_t ct_len2[17][4] = {
    { 4 }, { 6, 4 }, { 6, 5, 4 }, { 6, 5, 5, 4 }, { 7, 5, 5, 4 },
    { 7, 5, 5, 4 }, { 7, 6, 6, 4 }, { 7, 6, 6, 4 }, { 8, 7, 7, 5 },
    { 8, 8, 7, 6 }, { 9, 8, 8, 7 }, { 9, 9, 8, 8 }, { 9, 9, 9, 8 },
    { 10, 9, 9, 9 }, { 10, 10, 10, 10 }, { 10, 10, 10, 10 },
    { 10, 10, 10, 10 },
};
static const uint8_t ct_bits2[17][4] = {
    { 15 }, { 15, 14 }, { 11, 15, 13 }, { 8, 12, 14, 12 },
    { 15, 10, 11, 11 }, { 11, 8, 9, 10 }, { 9, 14, 13, 9 },
    { 8, 10, 9, 8 }, { 15, 14, 13, 13 }, { 11, 14, 10, 12 },
    { 15, 10, 13, 12 }, { 11, 14, 9, 12 }, { 8, 10, 13, 8 },
    { 13, 7, 9, 12 }, { 9, 12, 11, 10 }, { 5, 8, 7, 6 }, { 1, 4, 3, 2 },
};
/* coeff_token, nC == -1: chroma DC 4:2:0 */
static const uint8_t ct_len_cdc[5][4] = {
    { 2 }, { 6, 1 }, { 6, 6, 3 }, { 6, 7, 7, 6 }, { 6, 8, 8, 7 },
};
static const uint8_t ct_bits_cdc[5][4] = {
    { 1 }, { 7, 1 }, { 4, 6, 1 }, { 3, 3, 2, 5 }, { 2, 3, 2, 0 },
};
/* total_zeros, 4x4 blocks (Tables 9-7, 9-8): [TotalCoeff - 1][total_zeros] */
static const uint8_t tz_len[15][16] = {
    { 1, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9 },
    { 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6, 6, 6 },
    { 4, 3, 3, 3, 4, 4, 3, 3, 4, 5, 5, 6, 5, 6 },
    { 5, 3, 4, 4, 3, 3, 3, 4, 3, 4, 5, 5, 5 },
    { 4, 4, 4, 3, 3, 3, 3, 3, 4, 5, 4, 5 },
    { 6, 5, 3, 3, 3, 3, 3, 3, 4, 3, 6 },
    { 6, 5, 3, 3, 3, 2, 3, 4, 3, 6 },
    { 6, 4, 5, 3, 2, 2, 3, 3, 6 },
    { 6, 6, 4, 2, 2, 3, 2, 5 },
    { 5, 5, 3, 2, 2, 2, 4 },
    { 4, 4, 3, 3, 1, 3 },
    { 4, 4, 2, 1, 3 },
    { 3, 3, 1, 2 },
    { 2, 2, 1 },
    { 1, 1 },
};
static const uint8_t tz_bits[15][16] = {
    { 1, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 1 },
    { 7, 6, 5, 4, 3, 5, 4, 3, 2, 3, 2, 3, 2, 1, 0 },
    { 5, 7, 6, 5, 4, 3, 4, 3, 2, 3, 2, 1, 1, 0 },
    { 3, 7, 5, 4, 6, 5, 4, 3, 3, 2, 2, 1, 0 },
    { 5, 4, 3, 7, 6, 5, 4, 3, 2, 1, 1, 0 },
    { 1, 1, 7, 6, 5, 4, 3, 2, 1, 1, 0 },
    { 1, 1, 5, 4, 3, 3, 2, 1, 1, 0 },
    { 1, 1, 1, 3, 3, 2, 2, 1, 0 },
    { 1, 0, 1, 3, 2, 1, 1, 1 },
    { 1, 0, 1, 3, 2, 1, 1 },
    { 0, 1, 1, 2, 1, 3 },
    { 0, 1, 1, 1, 1 },
    { 0, 1, 1, 1 },
    { 0, 1, 1 },
    { 0, 1 },
};
/* total_zeros, chroma DC 4:2:0 (Table 9-9a) */
static const uint8_t tz_len_cdc[3][4] = {
    { 1, 2, 3, 3 }, { 1, 2, 2 }, { 1, 1 },
};
static const uint8_t tz_bits_cdc[3][4] = {
    { 1, 1, 1, 0 }, { 1, 1, 0 }, { 1, 0 },
};
/* run_before (Table 9-10): [min(zerosLeft, 7) - 1][run_before] */
static const uint8_t rb_len[7][15] = {
    { 1, 1 }, { 1, 2, 2 }, { 2, 2, 2, 2 }, { 2, 2, 2, 3, 3 },
    { 2, 2, 3, 3, 3, 3 }, { 2, 3, 3, 3, 3, 3, 3 },
    { 3, 3, 3, 3, 3, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11 },
};
static const uint8_t rb_bits[7][15] = {
    { 1, 0 }, { 1, 1, 0 }, { 3, 2, 1, 0 }, { 3, 2, 1, 1, 0 },
    { 3, 2, 3, 2, 1, 0 }, { 3, 0, 1, 3, 2, 5, 4 },
    { 7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
};

/* residual_block_cavlc() for coefficients in scan order, coeff_token by
 * nC (9.2.1): -1 the chroma DC table, 0-1, 2-3, 4-7, 8 and more the
 * fixed-length code */
static void cavlc_residual_nc(BitWriter *w, const int *c, int max, int nc);

/* the same with nC 0 (see the file comment), or -1 for chroma_dc */
static void cavlc_residual(BitWriter *w, const int *c, int max, bool chroma_dc)
{
    cavlc_residual_nc(w, c, max, chroma_dc ? -1 : 0);
}

static void cavlc_residual_nc(BitWriter *w, const int *c, int max, int nc)
{
    const bool chroma_dc = nc < 0;
    int total = 0, t1 = 0, last = -1, i, n;
    int lev[16], run[16];               /* nonzero coefficients, high first */

    for (i = max - 1; i >= 0; i--) {
        if (c[i]) {
            if (last < 0) {
                last = i;
            }
            lev[total] = c[i];
            run[total] = 0;
            total++;
        } else if (total) {
            run[total - 1]++;           /* zeros below that coefficient */
        }
    }
    for (i = 0; i < total && i < 3 && abs(lev[i]) == 1; i++) {
        t1++;
    }
    if (chroma_dc) {
        bw_bits(w, ct_bits_cdc[total][t1], ct_len_cdc[total][t1]);
    } else if (nc < 2) {
        bw_bits(w, ct_bits0[total][t1], ct_len0[total][t1]);
    } else if (nc < 4) {
        bw_bits(w, ct_bits1[total][t1], ct_len1[total][t1]);
    } else if (nc < 8) {
        bw_bits(w, ct_bits2[total][t1], ct_len2[total][t1]);
    } else {
        bw_bits(w, total ? ((total - 1) << 2) | t1 : 3, 6);
    }
    if (!total) {
        return;
    }
    for (i = 0; i < t1; i++) {
        bw_bits(w, lev[i] < 0, 1);
    }
    int suffix_len = total > 10 && t1 < 3;
    for (i = t1; i < total; i++) {
        int code = lev[i] > 0 ? 2 * lev[i] - 2 : -2 * lev[i] - 1;

        if (i == t1 && t1 < 3) {
            code -= 2;
        }
        if (suffix_len == 0) {
            if (code < 14) {
                bw_bits(w, 1, code + 1);
            } else if (code < 30) {
                bw_bits(w, 1, 15);
                bw_bits(w, code - 14, 4);
            } else {
                bw_bits(w, 1, 16);
                bw_bits(w, MIN(code - 30, 4095), 12);
            }
        } else if (code < (15 << suffix_len)) {
            bw_bits(w, 1, (code >> suffix_len) + 1);
            bw_bits(w, code & ((1 << suffix_len) - 1), suffix_len);
        } else {
            bw_bits(w, 1, 16);
            bw_bits(w, MIN(code - (15 << suffix_len), 4095), 12);
        }
        if (suffix_len == 0) {
            suffix_len = 1;
        }
        if (abs(lev[i]) > (3 << (suffix_len - 1)) && suffix_len < 6) {
            suffix_len++;
        }
    }
    if (total < max) {
        int tz = 0;
        for (i = 0; i < last; i++) {
            tz += c[i] == 0;
        }
        if (chroma_dc) {
            bw_bits(w, tz_bits_cdc[total - 1][tz], tz_len_cdc[total - 1][tz]);
        } else {
            bw_bits(w, tz_bits[total - 1][tz], tz_len[total - 1][tz]);
        }
        int left = tz;
        for (n = 0; n < total - 1 && left > 0; n++) {
            const int t = MIN(left, 7) - 1;
            bw_bits(w, rb_bits[t][run[n]], rb_len[t][run[n]]);
            left -= run[n];
        }
    }
}

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

/* reconstruct an I_16x16 DC macroblock exactly as a decoder (8.5.10-11) */
static void recon_mb(Frame *f, int mx, int my, int pred, const int lev[16],
                     int qp, int cpred[2][4], int clev[2][4], int qpc)
{
    int i, bx, by, x, y, comp;
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

/* cw: write the macroblock with CAVLC there instead of CABAC to c */
static void encode_mb(Frame *f, Cabac *c, BitWriter *cw, MbInfo *info,
                      int mx, int my, int qp, int qpc)
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

    int scan[16], nz = 0;
    for (i = 0; i < 16; i++) {
        scan[i] = lev[zigzag4x4[i]];
        nz |= scan[i] != 0;
    }
    if (cw) {
        /* mb_type I_16x16_2_<cbp chroma>_0 (Table 7-11): 1 + 2 + 4 * cbpc */
        bw_ue(cw, 3 + 4 * cbp_chroma);
        bw_ue(cw, 0);                                   /* chroma pred: DC */
        bw_se(cw, 0);                                   /* mb_qp_delta */
        cavlc_residual(cw, scan, 16, false);
        for (comp = 0; comp < 2 && cbp_chroma; comp++) {
            cavlc_residual(cw, clev[comp], 4, true);
        }
        if (!cbp_chroma) {
            memset(clev, 0, sizeof(clev));
        }
        goto reconstruct;
    }

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

reconstruct:
    /* reconstruct, exactly as a decoder (8.5.10, 8.5.11) */
    recon_mb(f, mx, my, pred, lev, qp, cpred, clev, qpc);
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

    if (!e->have_sps || !e->have_pps || e->unsupported) {
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
    f.y = g_malloc(f.w * f.h);
    f.cb = g_malloc(f.w * f.h / 4);
    f.cr = g_malloc(f.w * f.h / 4);
    const int qpc = chroma_qp(qp, e->chroma_qp_offset);
    size_t len;
    if (e->cabac) {
        while (w.bits) {                                /* cabac_alignment */
            bw_bits(&w, 1, 1);
        }
        cabac_init(&c, qp, rbsp + w.n, rbsp + rbsp_cap);
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
        len = c.p - rbsp;
    } else {
        for (my = 0; my < mbh; my++) {
            for (mx = 0; mx < mbw; mx++) {
                encode_mb(&f, NULL, &w, info, mx, my, qp, qpc);
            }
        }
        bw_bits(&w, 1, 1);                              /* rbsp_stop_one_bit */
        while (w.bits) {
            bw_bits(&w, 0, 1);
        }
        len = MIN(w.n, rbsp_cap);
    }
    g_free(f.y);
    g_free(f.cb);
    g_free(f.cr);

    return nal_wrap(out, cap, idr ? 3 : 2, idr ? 5 : 1, rbsp, len);
}

/* ---- slice data for engines whose drivers write the headers ------------ */

/*
 * Some engines (Grain Media's MCP280) have their driver pack the SPS, PPS
 * and slice header through the hardware's bit writer and leave the slice
 * data to the hardware.  These produce just that: the macroblock layer of
 * one slice covering mbw x mbh macroblocks, ending with the slice's
 * trailing bits, as the RBSP (no emulation prevention).  The slice is a
 * picture of its own for intra prediction, as slices are.
 */

static void frame_setup(Frame *f, int mbw, int mbh, const uint8_t *y, int ys,
                        const uint8_t *uv, int cs)
{
    *f = (Frame) { .mbw = mbw, .mbh = mbh, .w = mbw * 16, .h = mbh * 16,
                   .sy = y, .suv = uv, .sys = ys, .scs = cs };
    f->y = g_malloc(f->w * f->h);
    f->cb = g_malloc(f->w * f->h / 4);
    f->cr = g_malloc(f->w * f->h / 4);
}

static void frame_free(Frame *f)
{
    g_free(f->y);
    g_free(f->cb);
    g_free(f->cr);
}

size_t h264_slice_i16dc_cabac(int qp, int chroma_qp_offset, int mbw, int mbh,
                              const uint8_t *y, int ys, const uint8_t *uv,
                              int cs, uint8_t *out, size_t cap)
{
    g_autofree MbInfo *info = g_new0(MbInfo, mbw * mbh);
    Frame f;
    Cabac c;
    int mx, my;

    qp = MIN(MAX(qp, 0), 51);
    frame_setup(&f, mbw, mbh, y, ys, uv, cs);
    cabac_init(&c, qp, out, out + cap);
    const int qpc = chroma_qp(qp, chroma_qp_offset);
    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            encode_mb(&f, &c, NULL, info, mx, my, qp, qpc);
            if (mx == mbw - 1 && my == mbh - 1) {
                cabac_flush(&c);            /* with the stop bit */
            } else {
                cabac_terminate0(&c);
            }
        }
    }
    frame_free(&f);
    return c.p - out;
}

/*
 * A P slice of nmb P_Skip macroblocks.  Only mb_skip_flag's first context
 * (11; both neighbours skipped or absent) is used, initialised for the
 * slice's cabac_init_idc (Table 9-13).
 */
size_t h264_slice_pskip_cabac(int qp, int cabac_init_idc, int nmb,
                              uint8_t *out, size_t cap)
{
    static const int8_t skip_init[3][2] = { { 23, 33 }, { 22, 25 },
                                            { 29, 16 } };
    const int8_t *mn = skip_init[MIN(MAX(cabac_init_idc, 0), 2)];
    Cabac c;
    int i, st;

    qp = MIN(MAX(qp, 0), 51);
    cabac_init(&c, qp, out, out + cap);
    st = MIN(MAX(((mn[0] * qp) >> 4) + mn[1], 1), 126);
    c.state[11] = (MIN(st, 127 - st) << 1) | (st >> 6);
    for (i = 0; i < nmb; i++) {
        cabac_decision(&c, 11, 1);
        if (i == nmb - 1) {
            cabac_flush(&c);
        } else {
            cabac_terminate0(&c);
        }
    }
    return c.p - out;
}

/*
 * CAVLC residual of a block with at most one coefficient, at scan
 * position 0: what the DC-of-DC coding below sends.  nc is 0 (luma DC of
 * an I_16x16 neighbourhood with no AC) or -1 (chroma DC).
 */
static void cavlc_dc1(BitWriter *w, int level, bool chroma_dc)
{
    const int c[16] = { level };

    cavlc_residual(w, c, chroma_dc ? 4 : 16, chroma_dc);
}

/*
 * An I slice in CAVLC: every macroblock I_16x16 with DC prediction, its
 * luma and chroma DC sent as their first (DC-of-DC) coefficient only, so
 * each macroblock comes out as its mean.  The slice data starts
 * bit_offset bits into a byte of the stream (byte alignment -- PCM's,
 * the trailing bits' -- is the stream's): out's first byte holds that
 * many unused bits, then the data, rbsp_slice_trailing_bits included, and
 * the return is the bits out holds, the unused ones counted.
 */
size_t h264_slice_i16dc_cavlc(int qp, int chroma_qp_offset, int mbw, int mbh,
                              const uint8_t *y, int ys, const uint8_t *uv,
                              int cs, int bit_offset, uint8_t *out, size_t cap)
{
    BitWriter w = { .p = out, .cap = cap, .bits = bit_offset & 7 };
    Frame f;
    int mx, my, i, bx, by, x, yy, comp;

    qp = MIN(MAX(qp, 0), 51);
    frame_setup(&f, mbw, mbh, y, ys, uv, cs);
    const int qpc = chroma_qp(qp, chroma_qp_offset);
    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            int dc[16], lev[16] = { 0 }, cpred[2][4], clev[2][4] = { { 0 } };
            const int pred = pred16_dc(&f, mx, my);

            for (by = 0; by < 4; by++) {
                for (bx = 0; bx < 4; bx++) {
                    int s = 0;
                    for (yy = 0; yy < 4; yy++) {
                        for (x = 0; x < 4; x++) {
                            s += src_y(&f, mx * 16 + bx * 4 + x,
                                       my * 16 + by * 4 + yy) - pred;
                        }
                    }
                    dc[by * 4 + bx] = s;
                }
            }
            hadamard4(dc);
            lev[0] = quant(dc[0] / 2, qp, 1);
            lev[0] = MIN(MAX(lev[0], -2047), 2047);

            for (comp = 0; comp < 2; comp++) {
                uint8_t *rp = comp ? f.cr : f.cb;
                int s = 0;
                for (i = 0; i < 4; i++) {
                    const int xo = (i & 1) * 4, yo = (i >> 1) * 4;
                    cpred[comp][i] = predc_dc(rp, f.w / 2, mx, my, xo, yo);
                    for (yy = 0; yy < 4; yy++) {
                        for (x = 0; x < 4; x++) {
                            s += src_c(&f, comp, mx * 8 + xo + x,
                                       my * 8 + yo + yy) - cpred[comp][i];
                        }
                    }
                }
                clev[comp][0] = quant(s, qpc, 1);
                clev[comp][0] = MIN(MAX(clev[comp][0], -2047), 2047);
            }
            const int cbpc = clev[0][0] || clev[1][0];

            bw_ue(&w, 3 + 4 * cbpc);            /* I_16x16_2_cbpc_0 */
            bw_ue(&w, 0);                       /* chroma pred: DC */
            bw_se(&w, 0);                       /* mb_qp_delta */
            cavlc_dc1(&w, lev[0], false);
            if (cbpc) {
                cavlc_dc1(&w, clev[0][0], true);
                cavlc_dc1(&w, clev[1][0], true);
            }
            recon_mb(&f, mx, my, pred, lev, qp, cpred, clev, qpc);
        }
    }
    frame_free(&f);
    bw_bits(&w, 1, 1);                          /* rbsp_stop_one_bit */
    while (w.bits) {
        bw_bits(&w, 0, 1);
    }
    return w.n * 8;
}

/* A P slice of nmb P_Skip macroblocks in CAVLC; returns bits, as above. */
size_t h264_slice_pskip_cavlc(int nmb, int bit_offset, uint8_t *out,
                              size_t cap)
{
    BitWriter w = { .p = out, .cap = cap, .bits = bit_offset & 7 };

    bw_ue(&w, nmb);                             /* mb_skip_run */
    bw_bits(&w, 1, 1);
    while (w.bits) {
        bw_bits(&w, 0, 1);
    }
    return w.n * 8;
}

/*
 * P slices that follow the picture: every macroblock that differs from
 * the reference is sent as I_PCM, the rest as P_Skip.  PCM needs no
 * prediction, so the decoder's picture is exactly the source wherever
 * something changed -- and with nothing coded, edges between PCM and
 * skipped macroblocks are not deblocked while the slice QP stays below
 * 31 (their average QP picks a zero alpha).  ref is the picture the
 * decoder holds for this slice's rows, as NV12 at stride w, and is
 * updated to what it holds afterwards.
 */
static bool mb_differs(int mx, int my, int w, const uint8_t *y, int ys,
                       const uint8_t *uv, int cs, const uint8_t *ry,
                       const uint8_t *ruv)
{
    int r;

    for (r = 0; r < 16; r++) {
        if (memcmp(y + (my * 16 + r) * ys + mx * 16,
                   ry + (my * 16 + r) * w + mx * 16, 16)) {
            return true;
        }
    }
    for (r = 0; r < 8; r++) {
        if (memcmp(uv + (my * 8 + r) * cs + mx * 16,
                   ruv + (my * 8 + r) * w + mx * 16, 16)) {
            return true;
        }
    }
    return false;
}

/* the macroblock's PCM samples (Y, Cb, Cr), copied into ref as well */
static void mb_pcm(int mx, int my, int w, const uint8_t *y, int ys,
                   const uint8_t *uv, int cs, uint8_t *ry, uint8_t *ruv,
                   uint8_t pcm[384])
{
    int r, x;

    for (r = 0; r < 16; r++) {
        memcpy(pcm + r * 16, y + (my * 16 + r) * ys + mx * 16, 16);
        memcpy(ry + (my * 16 + r) * w + mx * 16, pcm + r * 16, 16);
    }
    for (r = 0; r < 8; r++) {
        const uint8_t *s = uv + (my * 8 + r) * cs + mx * 16;

        for (x = 0; x < 8; x++) {
            pcm[256 + r * 8 + x] = s[2 * x];
            pcm[320 + r * 8 + x] = s[2 * x + 1];
        }
        memcpy(ruv + (my * 8 + r) * w + mx * 16, s, 16);
    }
}

size_t h264_slice_ppcm_cavlc(int mbw, int mbh, const uint8_t *y, int ys,
                             const uint8_t *uv, int cs, uint8_t *ry,
                             uint8_t *ruv, int bit_offset, uint8_t *out,
                             size_t cap)
{
    BitWriter w = { .p = out, .cap = cap, .bits = bit_offset & 7 };
    uint8_t pcm[384];
    int mx, my, i, run = 0;

    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            if (!mb_differs(mx, my, mbw * 16, y, ys, uv, cs, ry, ruv)) {
                run++;
                continue;
            }
            bw_ue(&w, run);                     /* mb_skip_run */
            run = 0;
            bw_ue(&w, 5 + 25);                  /* I_PCM in a P slice */
            while (w.bits) {
                bw_bits(&w, 0, 1);              /* pcm_alignment_zero_bit */
            }
            mb_pcm(mx, my, mbw * 16, y, ys, uv, cs, ry, ruv, pcm);
            for (i = 0; i < 384; i++) {
                bw_bits(&w, pcm[i], 8);
            }
        }
    }
    if (run) {
        bw_ue(&w, run);
    }
    bw_bits(&w, 1, 1);
    while (w.bits) {
        bw_bits(&w, 0, 1);
    }
    return w.n * 8;
}

/* (m, n) of contexts 11-14 and 17 for P slices, by cabac_init_idc */
static const int8_t p_ctx_init[3][5][2] = {
    { { 23, 33 }, { 23, 2 }, { 21, 0 }, { 1, 9 }, { 5, 57 } },
    { { 22, 25 }, { 34, 0 }, { 16, 0 }, { -2, 9 }, { 2, 65 } },
    { { 29, 16 }, { 25, 0 }, { 14, 0 }, { -10, 51 }, { 26, 16 } },
};

size_t h264_slice_ppcm_cabac(int qp, int cabac_init_idc, int mbw, int mbh,
                             const uint8_t *y, int ys, const uint8_t *uv,
                             int cs, uint8_t *ry, uint8_t *ruv, uint8_t *out,
                             size_t cap)
{
    static const int ctx_of[5] = { 11, 12, 13, 14, 17 };
    const int8_t (*mn)[2] = p_ctx_init[MIN(MAX(cabac_init_idc, 0), 2)];
    g_autofree bool *skipped = g_new0(bool, mbw * mbh);
    uint8_t pcm[384];
    Cabac c;
    int mx, my, i;

    qp = MIN(MAX(qp, 0), 51);
    cabac_init(&c, qp, out, out + cap);
    for (i = 0; i < 5; i++) {
        const int st = MIN(MAX(((mn[i][0] * qp) >> 4) + mn[i][1], 1), 126);

        c.state[ctx_of[i]] = (MIN(st, 127 - st) << 1) | (st >> 6);
    }
    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            const bool last = mx == mbw - 1 && my == mbh - 1;
            /* mb_skip_flag: one per neighbour present and not skipped */
            const int inc = (mx && !skipped[my * mbw + mx - 1]) +
                            (my && !skipped[(my - 1) * mbw + mx]);

            if (!mb_differs(mx, my, mbw * 16, y, ys, uv, cs, ry, ruv)) {
                skipped[my * mbw + mx] = true;
                cabac_decision(&c, 11 + inc, 1);
            } else {
                cabac_decision(&c, 11 + inc, 0);
                cabac_decision(&c, 14, 1);      /* prefix: intra */
                cabac_decision(&c, 17, 1);      /* not I_NxN */
                cabac_flush(&c);                /* terminate 1: I_PCM */
                mb_pcm(mx, my, mbw * 16, y, ys, uv, cs, ry, ruv, pcm);
                for (i = 0; i < 384; i++) {
                    cabac_put(&c, pcm[i]);
                }
                c.low = 0;                      /* the engine restarts */
                c.range = 0x1fe;
                c.queue = -9;
                c.outstanding = 0;
                c.start = c.p;
            }
            if (last) {
                cabac_flush(&c);
            } else {
                cabac_terminate0(&c);
            }
        }
    }
    return c.p - out;
}

#include "hantro-h264-inter.c.inc"
