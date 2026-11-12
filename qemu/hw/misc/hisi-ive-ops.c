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

/*
 * The 16-bit pixel at (x, y), zero past the picture: the suppression
 * reads a neighbour outside it as 0, so a border pixel is kept rather
 * than compared with itself
 */
static inline uint16_t at16(const uint16_t *src, uint32_t w, uint32_t h,
                            int x, int y)
{
    if (x < 0 || y < 0 || x >= (int)w || y >= (int)h) {
        return 0;
    }
    return src[(uint32_t)y * w + (uint32_t)x];
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

void ive_lbp(const uint8_t *src, uint32_t w, uint32_t h, int mode, int thr,
             uint8_t *out)
{
    /* bit 7 top-left, then clockwise */
    static const int dx[8] = { -1, 0, 1, 1, 1, 0, -1, -1 };
    static const int dy[8] = { -1, -1, -1, 0, 1, 1, 1, 0 };

    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            int c = src[y * w + x];
            uint8_t v = 0;

            for (int k = 0; k < 8; k++) {
                int d = (int)at(src, w, h, x + dx[k], y + dy[k]) - c;

                if ((mode ? abs(d) : d) >= thr) {
                    v |= 1 << (7 - k);
                }
            }
            out[y * w + x] = v;
        }
    }
}

/* The gradient's octant: 0 for +x, one per 45 degrees anticlockwise */
static uint8_t grad_octant(int gh, int gv)
{
    /* tan(22.5) and tan(67.5) in Q30: the boundaries are irrational, so
     * no integer gradient can tie and this decides every case exactly */
    const int64_t t1 = 444797225, t2 = 2592341274LL, one = 1 << 30;
    int64_t ah = abs(gh), av = abs(gv);

    if (!ah && !av) {
        return 0;
    }
    if (av * one < ah * t1) {
        return gh >= 0 ? 0 : 4;
    }
    if (av * one < ah * t2) {
        return gv >= 0 ? (gh >= 0 ? 1 : 3) : (gh >= 0 ? 7 : 5);
    }
    return gv >= 0 ? 2 : 6;
}

static void ive_gradients(const uint8_t *src, uint32_t w, uint32_t h,
                          const int8_t mask[25], int32_t *gh, int32_t *gv)
{
    int8_t t[25];

    for (int k = 0; k < 25; k++) {
        t[k] = mask[(k % 5) * 5 + k / 5];
    }
    ive_correlate(src, w, h, mask, 5, 5, gh);
    ive_correlate(src, w, h, t, 5, 5, gv);
}

void ive_mag_and_ang(const uint8_t *src, uint32_t w, uint32_t h,
                     const int8_t mask[25], uint16_t thr, uint16_t *mag,
                     uint8_t *ang)
{
    size_t n = (size_t)w * h;
    g_autofree int32_t *gh = g_new(int32_t, n);
    g_autofree int32_t *gv = g_new(int32_t, n);

    ive_gradients(src, w, h, mask, gh, gv);
    for (size_t i = 0; i < n; i++) {
        if (mag) {
            int64_t m = (int64_t)abs(gh[i]) + abs(gv[i]);

            mag[i] = m >= thr ? MIN(m, UINT16_MAX) : 0;
        }
        if (ang) {
            ang[i] = grad_octant(gh[i], gv[i]);
        }
    }
}

void ive_norm_grad(const uint8_t *src, uint32_t w, uint32_t h,
                   const int8_t mask[25], int norm, int8_t *gh, int8_t *gv)
{
    size_t n = (size_t)w * h;
    g_autofree int32_t *h32 = g_new(int32_t, n);
    g_autofree int32_t *v32 = g_new(int32_t, n);

    norm = MIN(MAX(norm, 0), 30);
    ive_gradients(src, w, h, mask, h32, v32);
    for (size_t i = 0; i < n; i++) {
        /* rounded half up, as the engine does, then saturated */
        if (gh) {
            gh[i] = clampi((h32[i] + (1 << norm >> 1)) >> norm, -128, 127);
        }
        if (gv) {
            gv[i] = clampi((v32[i] + (1 << norm >> 1)) >> norm, -128, 127);
        }
    }
}

void ive_resize(const uint8_t *src, uint32_t sw, uint32_t sh, uint8_t *dst,
                uint32_t dw, uint32_t dh, uint32_t xratio, int32_t xoff,
                uint32_t yratio, int32_t yoff)
{
    for (uint32_t y = 0; y < dh; y++) {
        int64_t py = (int64_t)yoff + (int64_t)y * yratio;
        int y0 = clampi(py >> 27, 0, sh - 1);
        int y1 = clampi((py >> 27) + 1, 0, sh - 1);
        uint32_t ay = (py & 0x7ffffff) >> 15, cy = (1 << 12) - ay;

        for (uint32_t x = 0; x < dw; x++) {
            int64_t px = (int64_t)xoff + (int64_t)x * xratio;
            int x0 = clampi(px >> 27, 0, sw - 1);
            int x1 = clampi((px >> 27) + 1, 0, sw - 1);
            uint32_t ax = (px & 0x7ffffff) >> 15, cx = (1 << 12) - ax;
            /* each row to 7 fraction bits, then the column weights */
            uint32_t top = (src[y0 * sw + x0] * cx + src[y0 * sw + x1] * ax
                            + 16) >> 5;
            uint32_t bot = (src[y1 * sw + x0] * cx + src[y1 * sw + x1] * ax
                            + 16) >> 5;

            dst[y * dw + x] = (((top * cy) >> 17) + ((bot * ay) >> 17) + 2) >> 2;
        }
    }
}

void ive_acc(const uint8_t *a, const uint8_t *b, uint8_t *out, size_t n,
             int mode)
{
    for (size_t i = 0; i < n; i++) {
        int v = mode == 1 ? a[i] - b[i] :
                mode == 2 ? a[i] + (b[i] & 1) : a[i] + b[i];

        out[i] = clampi(v, 0, 255);
    }
}

void ive_image_dot(const int32_t *a, const int32_t *b, int32_t *out, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        int64_t v = ((int64_t)a[i] * b[i] + (1 << 29)) >> 30;

        out[i] = v < INT32_MIN ? INT32_MIN : v > INT32_MAX ? INT32_MAX : v;
    }
}

void ive_alpha_blend(const uint8_t *a, const uint8_t *b, const uint8_t *alpha,
                     uint8_t *out, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        out[i] = (a[i] * alpha[i] + b[i] * (255 - alpha[i]) + 128) >> 8;
    }
}

/*
 * The CSC matrices, Q10 with a per-channel constant, fitted from a V5
 * board over every chroma value (qemu-boot/ive-v5-capture.c): the
 * coefficients are the BT.601 / BT.709 ones divided by 1.164 for the
 * video-range output modes and plain for the full-range ones.  Both
 * directions treat the packed and planar RGB as B, G, R.
 */
typedef struct {
    int cy, cu, cv, k;
} IveCscRow;

static const struct {
    IveCscRow ch[3];            /* B, G, R */
    int lo, hi;
} ive_csc_to_rgb[4] = {
    {   /* 0: video BT.601 */
        { { 1024, 1774, 0, -226504 }, { 1024, -344, -715, 136040 },
          { 1024, 0, 1404, -179187 } }, 16, 235,
    },
    {   /* 1: video BT.709 */
        { { 1024, 1860, 0, -237515 }, { 1024, -187, -470, 84660 },
          { 1024, 0, 1577, -201334 } }, 16, 235,
    },
    {   /* 2: full-range BT.601 */
        { { 1192, 2066, 0, -283062 }, { 1192, -400, -833, 139252 },
          { 1192, 0, 1634, -227750 } }, 0, 255,
    },
    {   /* 3: full-range BT.709 */
        { { 1192, 2166, 0, -295776 }, { 1192, -218, -547, 79352 },
          { 1192, 0, 1836, -253571 } }, 0, 255,
    },
};

/* Y, then V and U, each as (b, g, r, k) */
static const int ive_csc_to_yuv[4][3][4] = {
    {   /* 8: RGB to full-range BT.601 */
        { 117, 601, 306, 512 }, { -85, -438, 523, 131584 },
        { 523, -347, -176, 131584 },
    },
    {   /* 9: RGB to full-range BT.709 */
        { 74, 732, 218, 512 }, { -48, -475, 523, 131584 },
        { 523, -403, -120, 131584 },
    },
    {   /* 10: RGB to video-range BT.601 */
        { 100, 516, 263, 16896 }, { -73, -377, 450, 131584 },
        { 450, -298, -152, 131584 },
    },
    {   /* 11: RGB to video-range BT.709 */
        { 63, 629, 187, 16896 }, { -41, -409, 450, 131584 },
        { 450, -346, -103, 131584 },
    },
};

bool ive_csc_mode_valid(int mode)
{
    return (mode >= 0 && mode < 4) || (mode >= 8 && mode < 12);
}

void ive_csc_yuv_to_rgb(const uint8_t *y, const uint8_t *u, const uint8_t *v,
                        uint8_t *out[3], int step, size_t n, int mode)
{
    const IveCscRow *row = ive_csc_to_rgb[mode & 3].ch;
    int lo = ive_csc_to_rgb[mode & 3].lo, hi = ive_csc_to_rgb[mode & 3].hi;

    for (size_t i = 0; i < n; i++) {
        for (int c = 0; c < 3; c++) {
            int32_t t = row[c].cy * y[i] + row[c].cu * u[i] +
                        row[c].cv * v[i] + row[c].k;

            out[c][i * step] = clampi(t >> 10, lo, hi);
        }
    }
}

void ive_csc_rgb_to_yuv(const uint8_t *b, const uint8_t *g, const uint8_t *r,
                        int rgb_stride, uint32_t w, uint32_t h, int mode,
                        uint8_t *yout, uint8_t *cout)
{
    const int (*m)[4] = ive_csc_to_yuv[(mode - 8) & 3];

    for (uint32_t i = 0; i < w * h; i++) {
        int32_t t = m[0][0] * b[i * rgb_stride] + m[0][1] * g[i * rgb_stride] +
                    m[0][2] * r[i * rgb_stride] + m[0][3];

        yout[i] = clampi(t >> 10, 0, 255);
    }
    /* the chroma of each 2x2 block comes from its top-left pixel */
    for (uint32_t y = 0; y < h / 2; y++) {
        for (uint32_t x = 0; x < w / 2; x++) {
            size_t i = (size_t)(2 * y) * w + 2 * x;

            for (int c = 0; c < 2; c++) {
                int32_t t = m[1 + c][0] * b[i * rgb_stride] +
                            m[1 + c][1] * g[i * rgb_stride] +
                            m[1 + c][2] * r[i * rgb_stride] + m[1 + c][3];

                cout[(size_t)y * w + 2 * x + c] = clampi(t >> 10, 0, 255);
            }
        }
    }
}

/* the two neighbours along octant o, and whether each is strictly beaten */
static const struct {
    int dx, dy;
} ive_octant_step[8] = {
    { 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 },
    { -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 },
};

void ive_canny_hys_edge(const uint16_t *mag, const uint8_t *ang, uint32_t w,
                        uint32_t h, uint16_t lo, uint16_t hi, uint8_t *edge,
                        uint32_t *stack, uint32_t max_pts,
                        uint32_t *stack_size)
{
    /* 0 none, 1 weak by magnitude, 2 a seed, 3 an edge thinned away */
    g_autofree uint8_t *state = g_malloc0((size_t)w * h);
    uint32_t n = 0;

    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            size_t i = (size_t)y * w + x;
            int o = ang[i] & 7, dx = ive_octant_step[o].dx;
            int dy = ive_octant_step[o].dy, m = mag[i];
            /* the neighbour the raster scan has already passed */
            bool first_earlier = dy < 0 || (dy == 0 && dx < 0);
            int n1 = at16(mag, w, h, (int)x + dx, (int)y + dy);
            int n2 = at16(mag, w, h, (int)x - dx, (int)y - dy);
            bool keep;

            if (o & 1) {                        /* the diagonals: both strict */
                keep = m > n1 && m > n2;
            } else {
                keep = first_earlier ? (m > n1 && m >= n2)
                                     : (m >= n1 && m > n2);
            }
            edge[i] = 1;
            if (!keep || m <= lo) {
                continue;
            }
            if (m <= hi) {
                state[i] = 1;
                edge[i] = 0;
                continue;
            }
            /* an edge: a seed unless one is already next to it */
            if ((x && (state[i - 1] == 2 || state[i - 1] == 3)) ||
                (y && state[i - w] == 2)) {
                state[i] = 3;
                edge[i] = 0;
                continue;
            }
            state[i] = 2;
            edge[i] = 2;
            if (stack && n < max_pts) {
                stack[n] = x | y << 16;         /* ot_svp_point_u16 */
            }
            n++;
        }
    }
    if (stack_size) {
        *stack_size = n;
    }
}

uint8_t ive_gmm2_pixel(IveGmm2Model *g, uint8_t v, const IveGmm2Ctrl *c,
                       uint8_t *bg, uint8_t *info)
{
    int n = MIN(MAX(c->model_num, 1), 5), hit = -1, lead = 0;
    int sample = v << 7;                /* the pixel in the model's U9Q7 */
    bool matched;

    /* the frequency leader before the update: the background, and the
     * match info's high bits */
    for (int i = 1; i < n; i++) {
        if (g[i].freq > g[lead].freq) {
            lead = i;
        }
    }
    for (int i = 0; i < n; i++) {
        int d = sample - g[i].mean;

        if (g[i].freq &&
            (int64_t)d * d <= ((int64_t)g[i].var * c->sns_factor << 7)) {
            hit = i;
            break;
        }
    }
    matched = hit >= 0;
    if (hit >= 0) {
        /*
         * The matched Gaussian walks towards the sample: both updates are
         * x -= (x - target) >> shift, which rounds towards +infinity, and
         * the variance's target is the squared distance in whole grey
         * levels, also rounded up
         */
        int d = sample - g[hit].mean, shift = 6 + MAX(c->var_rate, 1);
        int dg = -((-d) >> 7), sq = (dg * dg) << 7;
        int var = g[hit].var - ((g[hit].var - sq) >> shift);
        uint32_t freq = ((uint32_t)g[hit].freq * c->freq_reduce >> 16) +
                        c->freq_add;

        g[hit].mean = g[hit].mean - ((g[hit].mean - sample) >> shift);
        g[hit].var = MIN(MAX(var, c->min_var), c->max_var);
        g[hit].freq = MIN(freq, UINT16_MAX);
        g[hit].life = MIN(g[hit].life + c->life_factor, UINT16_MAX);
    }
    for (int i = 0; i < n; i++) {       /* the others only decay */
        if (i != hit && g[i].freq) {
            g[i].freq = (uint32_t)g[i].freq * c->freq_reduce >> 16;
        }
    }
    if (hit < 0) {
        int slot = -1;

        for (int i = n - 1; i >= 0; i--) {
            if (!g[i].freq) {
                slot = i;
                break;
            }
        }
        if (slot < 0) {
            slot = 0;
            for (int i = 1; i < n; i++) {
                if (g[i].freq < g[slot].freq) {
                    slot = i;
                }
            }
        }
        g[slot].mean = sample;
        g[slot].var = c->max_var;
        g[slot].freq = c->freq_init;
        g[slot].life = 1;
        hit = slot;
    }
    for (int i = 0; i < n; i++) {
        if (!g[i].freq) {              /* an unused slot reads life 0xffff */
            g[i].life = UINT16_MAX;
        }
    }
    if (bg) {
        *bg = g[lead].mean >> 7;
    }
    if (info) {
        *info = lead << 1 | matched;
    }
    return g[hit].life < c->life_threshold ? 255 : 0;
}
