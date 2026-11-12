/*
 * HiSilicon IVE image ops on host buffers, shared by hisi-ive (V4) and
 * hisi-ive-npu (V5); see hisi-ive-ops.h.
 *
 * Copyright (c) 2026 OpenIPC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/hisi-ive-ops.h"

static inline int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* the pixel at (x, y), edges replicated */
static inline uint8_t at(const uint8_t *src, uint32_t w, uint32_t h,
                         int x, int y)
{
    return src[clampi(y, 0, h - 1) * w + clampi(x, 0, w - 1)];
}

void ive_bitwise(int op, const uint8_t *a, const uint8_t *b, uint8_t *out,
                 size_t n)
{
    for (size_t i = 0; i < n; i++) {
        out[i] = op == IVE_AND ? a[i] & b[i] :
                 op == IVE_OR ? a[i] | b[i] : a[i] ^ b[i];
    }
}

void ive_sub(const uint8_t *a, const uint8_t *b, uint8_t *out, size_t n,
             bool shift)
{
    for (size_t i = 0; i < n; i++) {
        int d = (int)a[i] - (int)b[i];

        out[i] = shift ? (uint8_t)(int8_t)(d >> 1) : abs(d);
    }
}

void ive_add(const uint8_t *a, const uint8_t *b, uint8_t *out, size_t n,
             uint16_t x, uint16_t y)
{
    for (size_t i = 0; i < n; i++) {
        out[i] = MIN(((uint32_t)a[i] * x + (uint32_t)b[i] * y) >> 16, 255);
    }
}

void ive_hist(const uint8_t *a, size_t n, uint32_t bins[256])
{
    memset(bins, 0, 256 * sizeof(uint32_t));
    for (size_t i = 0; i < n; i++) {
        bins[a[i]]++;
    }
}

void ive_sad(const uint8_t *a, const uint8_t *b, uint32_t w, uint32_t h,
             int blk, uint32_t *sad)
{
    uint32_t bw = w / blk, bh = h / blk;

    for (uint32_t by = 0; by < bh; by++) {
        for (uint32_t bx = 0; bx < bw; bx++) {
            uint32_t s = 0;

            for (int dy = 0; dy < blk; dy++) {
                const uint8_t *pa = a + (by * blk + dy) * w + bx * blk;
                const uint8_t *pb = b + (by * blk + dy) * w + bx * blk;

                for (int dx = 0; dx < blk; dx++) {
                    s += abs((int)pa[dx] - (int)pb[dx]);
                }
            }
            sad[by * bw + bx] = s;
        }
    }
}

/*
 * The three bands -- at or below lo, above lo up to hi, above hi -- each
 * give a fixed value or the original
 */
#define ORIG    INT_MIN

static inline int band(int v, int lo, int hi, int a, int b, int c)
{
    int r = v <= lo ? a : v <= hi ? b : c;

    return r == ORIG ? v : r;
}

void ive_thresh(const uint8_t *src, uint8_t *out, size_t n, int mode,
                uint8_t lo, uint8_t hi, uint8_t min, uint8_t mid,
                uint8_t max)
{
    /* low band, middle, high per mode; 0..2 have no middle band */
    static const int lut[8][3] = {
        { 0, 2, 2 },  /* binary:        min, max */
        { 3, 2, 2 },  /* trunc:         orig, max */
        { 0, 3, 3 },  /* to min val:    min, orig */
        { 0, 1, 2 },  /* min mid max */
        { 3, 1, 2 },  /* orig mid max */
        { 0, 1, 3 },  /* min mid orig */
        { 0, 3, 2 },  /* min orig max */
        { 3, 1, 3 },  /* orig mid orig */
    };
    int v[4] = { min, mid, max, ORIG };
    const int *m = lut[mode & 7];
    int top = mode <= 2 ? lo : hi;

    for (size_t i = 0; i < n; i++) {
        out[i] = band(src[i], lo, top, v[m[0]], v[m[1]], v[m[2]]);
    }
}

void ive_thresh_s16(const int16_t *src, uint8_t *out, size_t n, int mode,
                    int16_t lo, int16_t hi, int min, int mid, int max)
{
    bool orig = mode & 1, s8 = mode < 2;

    for (size_t i = 0; i < n; i++) {
        int r = band(src[i], lo, hi, min, orig ? ORIG : mid, max);

        out[i] = s8 ? (uint8_t)clampi(r, -128, 127) : clampi(r, 0, 255);
    }
}

void ive_thresh_u16(const uint16_t *src, uint8_t *out, size_t n, int mode,
                    uint16_t lo, uint16_t hi, uint8_t min, uint8_t mid,
                    uint8_t max)
{
    for (size_t i = 0; i < n; i++) {
        out[i] = clampi(band(src[i], lo, hi, min, mode == 1 ? ORIG : mid,
                             max), 0, 255);
    }
}

void ive_16bit_to_8bit(const uint16_t *src, uint8_t *out, size_t n, int mode,
                       uint32_t ratio_q16, int8_t bias)
{
    for (size_t i = 0; i < n; i++) {
        int64_t v = mode == 3 ? (int64_t)src[i] : (int64_t)(int16_t)src[i];

        if (mode == 1) {
            v = llabs(v);
        }
        /* rounded to nearest, halves away from zero */
        v *= ratio_q16;
        v = (v + (v < 0 ? -0x8000 : 0x8000)) / 0x10000;
        switch (mode) {
        case 0:
            out[i] = (uint8_t)clampi(v, -128, 127);
            break;
        case 2:
            out[i] = clampi(v + bias, 0, 255);
            break;
        default:
            out[i] = clampi(v, 0, 255);
            break;
        }
    }
}

void ive_ncc(const uint8_t *a, const uint8_t *b, size_t n, uint64_t out[3])
{
    out[0] = out[1] = out[2] = 0;
    for (size_t i = 0; i < n; i++) {
        out[0] += (uint64_t)a[i] * b[i];
        out[1] += (uint64_t)a[i] * a[i];
        out[2] += (uint64_t)b[i] * b[i];
    }
}

void ive_integ(const uint8_t *src, uint32_t w, uint32_t h, uint32_t *sum,
               uint64_t *sqsum)
{
    for (uint32_t y = 0; y < h; y++) {
        uint32_t row = 0;
        uint64_t row2 = 0;

        for (uint32_t x = 0; x < w; x++) {
            uint32_t i = y * w + x, p = src[i];

            row += p;
            row2 += p * p;
            if (sum) {
                sum[i] = row + (y ? sum[i - w] : 0);
            }
            if (sqsum) {
                sqsum[i] = row2 + (y ? sqsum[i - w] : 0);
            }
        }
    }
}

void ive_morph(const uint8_t *src, uint32_t w, uint32_t h,
               const uint8_t mask[25], bool dilate, uint8_t *out)
{
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            uint8_t acc = dilate ? 0 : 0xff;

            for (int k = 0; k < 25; k++) {
                uint8_t p = at(src, w, h, x + k % 5 - 2, y + k / 5 - 2);

                acc = dilate ? acc | (p & mask[k])
                             : acc & (p | (uint8_t)~mask[k]);
            }
            out[y * w + x] = acc;
        }
    }
}

void ive_correlate(const uint8_t *src, uint32_t w, uint32_t h,
                   const int8_t *mask, int kw, int kh, int32_t *out)
{
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            int32_t s = 0;

            for (int j = 0; j < kh; j++) {
                for (int i = 0; i < kw; i++) {
                    s += mask[j * kw + i] *
                         at(src, w, h, x + i - kw / 2, y + j - kh / 2);
                }
            }
            out[y * w + x] = s;
        }
    }
}

static int cmp_u8(const void *a, const void *b)
{
    return *(const uint8_t *)a - *(const uint8_t *)b;
}

void ive_ord_stat(const uint8_t *src, uint32_t w, uint32_t h, int mode,
                  uint8_t *out)
{
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            uint8_t n[9];

            for (int k = 0; k < 9; k++) {
                n[k] = at(src, w, h, x + k % 3 - 1, y + k / 3 - 1);
            }
            qsort(n, 9, 1, cmp_u8);
            out[y * w + x] = mode == 1 ? n[8] : mode == 2 ? n[0] : n[4];
        }
    }
}
