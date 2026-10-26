/*
 * A baseline JPEG encoder for the encoder models (xm-h1, xm-jpeg): 4:2:0,
 * the ITU-T T.81 Annex K Huffman tables, a float DCT.  Moved out of xm-h1.c
 * unchanged.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "jpeg-enc.h"

/* zigzag position -> natural index */
static const uint8_t zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/* ITU-T T.81 Annex K.3, the tables hantro_jpeg writes into its header */
static const uint8_t dc_luma_bits[16] = { 0, 1, 5, 1, 1, 1, 1, 1, 1 };
static const uint8_t dc_chroma_bits[16] = { 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
static const uint8_t dc_vals[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
static const uint8_t ac_luma_bits[16] = {
    0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d,
};
static const uint8_t ac_luma_vals[162] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
    0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
    0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72,
    0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75,
    0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9,
    0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};
static const uint8_t ac_chroma_bits[16] = {
    0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77,
};
static const uint8_t ac_chroma_vals[162] = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41,
    0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1,
    0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44,
    0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74,
    0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a,
    0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
    0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
    0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};

typedef struct HuffTable {
    uint16_t code[256];
    uint8_t len[256];
} HuffTable;

static HuffTable huff_dc[2], huff_ac[2];     /* [0] luma, [1] chroma */

static void huff_build(HuffTable *t, const uint8_t *bits, const uint8_t *vals)
{
    unsigned code = 0, k = 0, l, i;

    for (l = 1; l <= 16; l++) {
        for (i = 0; i < bits[l - 1]; i++, k++) {
            t->code[vals[k]] = code++;
            t->len[vals[k]] = l;
        }
        code <<= 1;
    }
}

typedef struct BitWriter {
    uint8_t *buf;
    size_t cap, n;
    uint32_t acc;
    int nbits;
    bool overflow;
} BitWriter;

static void bw_byte(BitWriter *w, uint8_t b)
{
    if (w->n >= w->cap) {
        w->overflow = true;
        return;
    }
    w->buf[w->n++] = b;
}

static void bw_put(BitWriter *w, uint32_t bits, int len)
{
    w->acc = (w->acc << len) | (bits & ((1u << len) - 1));
    w->nbits += len;
    while (w->nbits >= 8) {
        const uint8_t b = w->acc >> (w->nbits - 8);
        bw_byte(w, b);
        if (b == 0xff) {
            bw_byte(w, 0x00);           /* byte stuffing */
        }
        w->nbits -= 8;
    }
}

static void bw_flush(BitWriter *w)
{
    if (w->nbits) {
        bw_put(w, 0x7f, 8 - w->nbits);  /* pad with 1s */
    }
}

static int bit_size(int v)
{
    int n = 0;

    v = v < 0 ? -v : v;
    while (v) {
        n++;
        v >>= 1;
    }
    return n;
}

static void put_value(BitWriter *w, int v, int size)
{
    if (size) {
        bw_put(w, v < 0 ? v - 1 : v, size);
    }
}

/* Forward 8x8 DCT, then quantise; out[] in zigzag order. */
static void fdct_quant(const float in[64], const uint8_t q[64], int out[64])
{
    static float c[8][8];
    static bool init;
    float rows[64], dct[64];
    int u, v, x, y;

    if (!init) {
        for (u = 0; u < 8; u++) {
            for (x = 0; x < 8; x++) {
                c[u][x] = (u ? 0.5f : 0.5f / sqrtf(2.0f)) *
                          cosf((2 * x + 1) * u * (float)M_PI / 16);
            }
        }
        init = true;
    }
    for (y = 0; y < 8; y++) {           /* rows: rows[y][u] */
        for (u = 0; u < 8; u++) {
            float acc = 0;
            for (x = 0; x < 8; x++) {
                acc += c[u][x] * in[y * 8 + x];
            }
            rows[y * 8 + u] = acc;
        }
    }
    for (u = 0; u < 8; u++) {           /* columns: dct[v][u] */
        for (v = 0; v < 8; v++) {
            float acc = 0;
            for (y = 0; y < 8; y++) {
                acc += c[v][y] * rows[y * 8 + u];
            }
            dct[v * 8 + u] = acc;
        }
    }
    for (x = 0; x < 64; x++) {
        const int nat = zigzag[x];
        out[x] = (int)lrintf(dct[nat] / q[nat]);
    }
}

static void encode_block(BitWriter *w, const float px[64], const uint8_t q[64],
                         int comp, int *dc_pred)
{
    const HuffTable *dc = &huff_dc[comp ? 1 : 0];
    const HuffTable *ac = &huff_ac[comp ? 1 : 0];
    int coef[64], k, run = 0, size;

    fdct_quant(px, q, coef);
    const int diff = coef[0] - *dc_pred;
    *dc_pred = coef[0];
    size = bit_size(diff);
    bw_put(w, dc->code[size], dc->len[size]);
    put_value(w, diff, size);
    for (k = 1; k < 64; k++) {
        if (!coef[k]) {
            run++;
            continue;
        }
        while (run > 15) {
            bw_put(w, ac->code[0xf0], ac->len[0xf0]);   /* ZRL */
            run -= 16;
        }
        size = bit_size(coef[k]);
        const int sym = (run << 4) | size;
        bw_put(w, ac->code[sym], ac->len[sym]);
        put_value(w, coef[k], size);
        run = 0;
    }
    if (run) {
        bw_put(w, ac->code[0], ac->len[0]);              /* EOB */
    }
}

static uint8_t px_at(const uint8_t *p, int stride, int w, int h, int x, int y)
{
    x = MIN(x, w - 1);
    y = MIN(y, h - 1);
    return p[y * stride + x];
}

void jpeg_enc_header(GByteArray *b, int w, int h, const uint8_t ql[64],
                     const uint8_t qc[64])
{
    static const uint8_t soi[] = { 0xff, 0xd8 };
    uint8_t seg[4 + 1 + 64 + 1 + 64];
    int i, t, n;

    g_byte_array_append(b, soi, 2);
    seg[0] = 0xff; seg[1] = 0xdb; seg[2] = 0; seg[3] = 2 + 2 * 65;
    for (t = 0; t < 2; t++) {
        seg[4 + t * 65] = t;
        for (i = 0; i < 64; i++) {
            seg[5 + t * 65 + i] = (t ? qc : ql)[zigzag[i]];
        }
    }
    g_byte_array_append(b, seg, 4 + 130);
    const uint8_t sof[] = { 0xff, 0xc0, 0, 17, 8, h >> 8, h, w >> 8, w, 3,
                            1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1 };
    g_byte_array_append(b, sof, sizeof(sof));
    for (t = 0; t < 4; t++) {
        const uint8_t *bits = (const uint8_t *[]){ dc_luma_bits, ac_luma_bits,
                                                   dc_chroma_bits,
                                                   ac_chroma_bits }[t];
        const uint8_t *vals = (const uint8_t *[]){ dc_vals, ac_luma_vals,
                                                   dc_vals,
                                                   ac_chroma_vals }[t];
        const uint8_t cls = ((t & 1) << 4) | (t >> 1);
        for (n = 0, i = 0; i < 16; i++) {
            n += bits[i];
        }
        const uint8_t hdr[] = { 0xff, 0xc4, (3 + 16 + n) >> 8, 3 + 16 + n, cls };
        g_byte_array_append(b, hdr, sizeof(hdr));
        g_byte_array_append(b, bits, 16);
        g_byte_array_append(b, vals, n);
    }
    const uint8_t sos[] = { 0xff, 0xda, 0, 12, 3, 1, 0x00, 2, 0x11, 3, 0x11,
                            0, 63, 0 };
    g_byte_array_append(b, sos, sizeof(sos));
}


/* the Huffman codes, built once */
static void huff_init(void)
{
    static bool done;

    if (done) {
        return;
    }
    huff_build(&huff_dc[0], dc_luma_bits, dc_vals);
    huff_build(&huff_dc[1], dc_chroma_bits, dc_vals);
    huff_build(&huff_ac[0], ac_luma_bits, ac_luma_vals);
    huff_build(&huff_ac[1], ac_chroma_bits, ac_chroma_vals);
    done = true;
}

size_t jpeg_enc_scan(const uint8_t *y, int ys, const uint8_t *cb,
                     const uint8_t *cr, int cs, int w, int h,
                     const uint8_t ql[64], const uint8_t qc[64], bool eoi,
                     uint8_t *out, size_t cap, bool *overflow)
{
    BitWriter bw = { .buf = out, .cap = cap };
    int dc[3] = { 0, 0, 0 }, mx, my;
    const int mbw = (w + 15) / 16, mbh = (h + 15) / 16;

    huff_init();
    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            float blk[64];
            int b, bx, by;

            for (b = 0; b < 4; b++) {               /* Y: 2x2 blocks */
                const int ox = mx * 16 + (b & 1) * 8, oy = my * 16 + (b >> 1) * 8;
                for (by = 0; by < 8; by++) {
                    for (bx = 0; bx < 8; bx++) {
                        blk[by * 8 + bx] = px_at(y, ys, w, h, ox + bx,
                                                 oy + by) - 128.0f;
                    }
                }
                encode_block(&bw, blk, ql, 0, &dc[0]);
            }
            for (b = 0; b < 2; b++) {               /* Cb, Cr */
                const uint8_t *p = b ? cr : cb;
                for (by = 0; by < 8; by++) {
                    for (bx = 0; bx < 8; bx++) {
                        blk[by * 8 + bx] = px_at(p, cs, (w + 1) / 2,
                                                 (h + 1) / 2, mx * 8 + bx,
                                                 my * 8 + by) - 128.0f;
                    }
                }
                encode_block(&bw, blk, qc, 1, &dc[1 + b]);
            }
        }
    }
    bw_flush(&bw);
    if (eoi) {
        bw_byte(&bw, 0xff);
        bw_byte(&bw, 0xd9);
    }
    *overflow = bw.overflow;
    return bw.n;
}
