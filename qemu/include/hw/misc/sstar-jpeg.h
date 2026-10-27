/*
 * A baseline JPEG scan coder shared by the SigmaStar/MStar JPEG encoder
 * models (sstar-jpe.c for Ifado, sstar-i3-jpe.c for infinity3).
 *
 * The hardware writes only the entropy-coded segment: 8x8 blocks, a
 * floating point forward DCT, quantization by the tables in zigzag order,
 * DC prediction per component and Huffman codes from the tables given
 * (ITU-T T.81 F.1.2), with 0xFF stuffed by 0x00.  No restart markers.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SSTAR_JPEG_H
#define HW_MISC_SSTAR_JPEG_H

#include <math.h>

/* What a scan is coded with: SOF0 sampling, DQT and DHT tables. */
typedef struct SstarJpegTables {
    int w, h, ncomp;
    int hs[3], vs[3], tq[3], td[3], ta[3];
    uint8_t q[4][64];           /* zigzag order, as in DQT */
    uint16_t code[8][256];      /* DC 0-3, AC 4-7 */
    uint8_t len[8][256];
} SstarJpegTables;

typedef struct SstarJpegBits {
    uint8_t *buf;
    size_t len, cap;
    uint32_t acc;
    int n;
    bool full;
} SstarJpegBits;

/* zigzag index -> natural (raster) index, T.81 Figure A.6 */
static const uint8_t sstar_jpeg_zigzag[64] = {
    0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/*
 * The T.81 Annex K.3 tables (K.3.3.1 DC, K.3.3.2 AC), as BITS (code
 * counts per length 1-16) followed by HUFFVAL, the layout of a DHT
 * segment.  Luma DC/AC, then chroma DC/AC.
 */
static const uint8_t sstar_jpeg_k3_dc_l[16 + 12] = {
    0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};
static const uint8_t sstar_jpeg_k3_dc_c[16 + 12] = {
    0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};
static const uint8_t sstar_jpeg_k3_ac_l[16 + 162] = {
    0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7D,
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
    0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08,
    0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52, 0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72,
    0x82, 0x09, 0x0A, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75,
    0x76, 0x77, 0x78, 0x79, 0x7A, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3,
    0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6,
    0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9,
    0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2,
    0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4,
    0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA,
};
static const uint8_t sstar_jpeg_k3_ac_c[16 + 162] = {
    0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77,
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41,
    0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0, 0x15, 0x62, 0x72, 0xD1,
    0x0A, 0x16, 0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26,
    0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44,
    0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74,
    0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A,
    0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4,
    0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
    0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA,
    0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4,
    0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA,
};

/*
 * Build table tab (DC 0-3, AC 4-7) from BITS + HUFFVAL (T.81 C.1/C.2).
 * Returns the bytes used, or 0 when they run past n.
 */
static inline size_t sstar_jpeg_huff_build(SstarJpegTables *t, int tab,
                                           const uint8_t *d, size_t n)
{
    int code = 0, v = 0, bits, k;

    if (n < 16) {
        return 0;
    }
    for (bits = 1; bits <= 16; bits++) {
        for (k = 0; k < d[bits - 1]; k++) {
            if (16 + (size_t)v >= n) {
                return 0;
            }
            t->code[tab][d[16 + v]] = code++;
            t->len[tab][d[16 + v]] = bits;
            v++;
        }
        code <<= 1;
    }
    return 16 + v;
}

/* Tables 0 (luma) and 1 (chroma) of Annex K.3. */
static inline void sstar_jpeg_annex_k(SstarJpegTables *t)
{
    sstar_jpeg_huff_build(t, 0, sstar_jpeg_k3_dc_l,
                          sizeof(sstar_jpeg_k3_dc_l));
    sstar_jpeg_huff_build(t, 1, sstar_jpeg_k3_dc_c,
                          sizeof(sstar_jpeg_k3_dc_c));
    sstar_jpeg_huff_build(t, 4, sstar_jpeg_k3_ac_l,
                          sizeof(sstar_jpeg_k3_ac_l));
    sstar_jpeg_huff_build(t, 5, sstar_jpeg_k3_ac_c,
                          sizeof(sstar_jpeg_k3_ac_c));
}

static inline void sstar_jpeg_put(SstarJpegBits *o, uint32_t v, int n)
{
    o->acc = (o->acc << n) | (v & ((1u << n) - 1));
    o->n += n;
    while (o->n >= 8) {
        uint8_t byte = o->acc >> (o->n - 8);

        o->n -= 8;
        if (o->len + 2 > o->cap) {
            o->full = true;
            return;
        }
        o->buf[o->len++] = byte;
        if (byte == 0xFF) {
            o->buf[o->len++] = 0;       /* byte stuffing */
        }
    }
}

static inline void sstar_jpeg_huff(SstarJpegBits *o, const SstarJpegTables *t,
                                   int tab, int sym)
{
    sstar_jpeg_put(o, t->code[tab][sym], t->len[tab][sym]);
}

/* One 8x8 block of component c (raster samples), DC predictor in *pred. */
static inline void sstar_jpeg_block(SstarJpegBits *o, const SstarJpegTables *t,
                                    int c, const uint8_t px[64], int *pred)
{
    static double cs[8][8];
    static bool init;
    double tmp[64], f[64];
    int i, j, k, run = 0, last = 63;
    int q[64];

    if (!init) {
        for (i = 0; i < 8; i++) {
            for (j = 0; j < 8; j++) {
                cs[i][j] = (i ? 0.5 : sqrt(0.125)) *
                           cos((2 * j + 1) * i * M_PI / 16);
            }
        }
        init = true;
    }
    for (i = 0; i < 8; i++) {           /* rows, then columns */
        for (k = 0; k < 8; k++) {
            double a = 0;

            for (j = 0; j < 8; j++) {
                a += cs[k][j] * (px[i * 8 + j] - 128);
            }
            tmp[i * 8 + k] = a;
        }
    }
    for (j = 0; j < 8; j++) {
        for (k = 0; k < 8; k++) {
            double a = 0;

            for (i = 0; i < 8; i++) {
                a += cs[k][i] * tmp[i * 8 + j];
            }
            f[k * 8 + j] = a;
        }
    }
    for (i = 0; i < 64; i++) {
        int z = sstar_jpeg_zigzag[i];
        int qv = MAX(t->q[t->tq[c]][i], 1);

        q[i] = (int)lround(f[z] / qv);
    }
    /* DC */
    {
        int d = q[0] - *pred, a = abs(d), n = 0;

        *pred = q[0];
        while (a >> n) {
            n++;
        }
        sstar_jpeg_huff(o, t, t->td[c], n);
        if (n) {
            sstar_jpeg_put(o, d < 0 ? d - 1 : d, n);
        }
    }
    while (last > 0 && !q[last]) {
        last--;
    }
    for (i = 1; i <= last; i++) {
        int a, n = 0;

        if (!q[i]) {
            run++;
            continue;
        }
        while (run >= 16) {
            sstar_jpeg_huff(o, t, 4 + t->ta[c], 0xF0);
            run -= 16;
        }
        a = abs(q[i]);
        while (a >> n) {
            n++;
        }
        sstar_jpeg_huff(o, t, 4 + t->ta[c], run << 4 | n);
        sstar_jpeg_put(o, q[i] < 0 ? q[i] - 1 : q[i], n);
        run = 0;
    }
    if (last < 63) {
        sstar_jpeg_huff(o, t, 4 + t->ta[c], 0x00);    /* EOB */
    }
}

/* Pad the last byte with ones (T.81 F.1.2.3). */
static inline void sstar_jpeg_flush(SstarJpegBits *o)
{
    if (o->n) {
        sstar_jpeg_put(o, 0x7F, 8 - o->n);
    }
}

#endif /* HW_MISC_SSTAR_JPEG_H */
