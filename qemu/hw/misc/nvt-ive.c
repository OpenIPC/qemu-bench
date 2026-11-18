/*
 * Novatek NA51089 IVE, the intelligent video engine (kdrv_ive: ive_reg.h,
 * ive_lib.h; the NT9856x IVE user guide), the work of one job.
 *
 * An 8-bit image (IN_ADDR, IN_LOFS; SIZE, width and height not minus one)
 * goes down a pipeline whose stages FUNC_EN turns on, a stage that is off
 * passing its input on:
 *
 *   general filter 7x7 | IRV -> median 3x3 -> edge 3x3 -> non-maximum
 *   suppression -> threshold LUT -> morphology 5x5 -> OUT_ADDR, OUT_LOFS
 *   (and the integral image, from the median's output)
 *
 * OUT_DATA_SEL picks the tap and its packing: 0 / 1 the LUT's (or the
 * morphology's) 1- or 4-bit result, LSB first; 2 the median's output;
 * 3 the suppression's; 4 the edge magnitude and a 4-bit direction, a
 * 12-bit stream; 5 the edge's x and y, a byte each; 6 the integral image,
 * u32.  Each output row writes only its ceil(width x bits / 8) bytes.
 * The morphology can instead take a 1- or 4-bit image from DRAM
 * (MORPH_IN_SEL 2 / 1).
 *
 * The guide documents the windows' shapes and the general filter's
 * weights and normalisation, the median's modes, the LUT (a 4-bit level
 * counts the thresholds a pixel is over; a 1-bit output is "over Th0") and
 * the morphology's neighbourhood.  The rest is a guess: borders replicate
 * the edge pixel; the edge's 5-bit coefficients are two's complement, its
 * magnitude (|gx| + |gy|) >> shift, its direction one of 16 sectors of
 * the half-turn; suppression keeps a magnitude at or over EDGE_MAG_TH
 * that is no smaller than its two neighbours across the edge; IRV (a
 * stereo-disparity filter, whose algorithm nothing describes) replaces a
 * pixel with its window's most frequent valid 6-bit value when the
 * window's valid share and that value's share pass THR_S and THR_H, out
 * of 64.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/misc/nvt-ive.h"

#define R(off)              regs[(off) / 4]

#define IVE_FUNC            0x010
#define IVE_IN_ADDR         0x014
#define IVE_IN_LOFS         0x018
#define IVE_OUT_ADDR        0x01C
#define IVE_OUT_LOFS        0x020
#define IVE_SIZE            0x024
#define IVE_GEN0            0x028
#define IVE_GEN1            0x02C
#define IVE_EDGE0           0x030
#define IVE_LUT0            0x03C
#define IVE_LUT3            0x048
#define IVE_MORPH           0x04C
#define IVE_IRV             0x050
#define IVE_IRV_MOD         0x054
#define IVE_IRV_NMOD        0x058
#define IVE_INTEGRAL        0x05C
#define IVE_DMA             0x068

#define IVE_F_GEN           (1u << 0)
#define IVE_F_MEDIAN        (1u << 1)
#define IVE_F_EDGE          (1u << 4)
#define IVE_F_EDGE_NODIR    (1u << 5)
#define IVE_F_NMS           (1u << 10)
#define IVE_F_LUT           (1u << 13)
#define IVE_F_MORPH         (1u << 14)
#define IVE_F_ERODE         (1u << 15)
#define IVE_F_INTEGRAL      (1u << 16)
#define IVE_F_IRV           (1u << 20)
#define IVE_F_IRV_5X5       (1u << 21)
#define IVE_F_IRV_INVAL63   (1u << 22)
#define IVE_DMA_DISABLE     (1u << 30)

#define IVE_MAX_W           4096
#define IVE_MAX_H           4096

typedef struct {
    int w, h;
} IveSize;

static bool ive_rd(hwaddr a, void *buf, size_t n)
{
    if (dma_memory_read(&address_space_memory, a, buf, n,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ive: cannot read 0x%"
                      HWADDR_PRIx "\n", a);
        return false;
    }
    return true;
}

static void ive_wr(hwaddr a, const void *buf, size_t n)
{
    if (dma_memory_write(&address_space_memory, a, buf, n,
                         MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ive: cannot write 0x%"
                      HWADDR_PRIx "\n", a);
    }
}

/* p[x, y] with the coordinates clamped to the image */
static inline int px(const int32_t *p, IveSize s, int x, int y)
{
    x = x < 0 ? 0 : x >= s.w ? s.w - 1 : x;
    y = y < 0 ? 0 : y >= s.h ? s.h - 1 : y;
    return p[y * s.w + x];
}

/*
 * The general filter: 7x7, symmetric, ten 4-bit weights (the guide's
 * index map), normalised by their sum; a sum of 0 passes the pixel.
 */
static void ive_gen(const uint32_t *regs, const int32_t *in, int32_t *out,
                    IveSize s)
{
    static const uint8_t map[7][7] = {
        { 10, 9, 8, 7, 8, 9, 10 }, { 9, 6, 5, 4, 5, 6, 9 },
        { 8, 5, 3, 2, 3, 5, 8 }, { 7, 4, 2, 1, 2, 4, 7 },
        { 8, 5, 3, 2, 3, 5, 8 }, { 9, 6, 5, 4, 5, 6, 9 },
        { 10, 9, 8, 7, 8, 9, 10 },
    };
    uint64_t coef = R(IVE_GEN0) | (uint64_t)R(IVE_GEN1) << 32;
    int c[10], sum = 0, i, x, y, dx, dy;

    for (i = 0; i < 10; i++) {
        c[i] = (coef >> (4 * i)) & 0xF;
    }
    for (dy = 0; dy < 7; dy++) {
        for (dx = 0; dx < 7; dx++) {
            sum += c[map[dy][dx] - 1];
        }
    }
    for (y = 0; y < s.h; y++) {
        for (x = 0; x < s.w; x++) {
            int acc = 0;

            if (!sum) {
                out[y * s.w + x] = in[y * s.w + x];
                continue;
            }
            for (dy = 0; dy < 7; dy++) {
                for (dx = 0; dx < 7; dx++) {
                    acc += c[map[dy][dx] - 1] *
                           px(in, s, x + dx - 3, y + dy - 3);
                }
            }
            out[y * s.w + x] = (acc + sum / 2) / sum;
        }
    }
}

/*
 * IRV: a 7x7 (or 5x5) window's 6-bit values, the invalid one (0 or 63)
 * left out; the most frequent valid value (the smallest of a tie)
 * replaces the pixel when valid > THR_S / 64 of the window and its count
 * > THR_H / 64 of the valid ones.
 */
static void ive_irv(uint32_t *regs, const int32_t *in, int32_t *out,
                    IveSize s)
{
    uint32_t f = R(IVE_FUNC), irv = R(IVE_IRV);
    int r = f & IVE_F_IRV_5X5 ? 2 : 3, area = (2 * r + 1) * (2 * r + 1);
    int inval = f & IVE_F_IRV_INVAL63 ? 63 : 0;
    int ths = extract32(irv, 4, 6), thh = extract32(irv, 12, 6);
    uint32_t mod = 0, nmod = 0;
    int x, y, dx, dy, v;

    for (y = 0; y < s.h; y++) {
        for (x = 0; x < s.w; x++) {
            int hist[64] = { 0 }, n = 0, best = 0;

            for (dy = -r; dy <= r; dy++) {
                for (dx = -r; dx <= r; dx++) {
                    v = px(in, s, x + dx, y + dy) & 63;
                    if (v != inval) {
                        hist[v]++;
                        n++;
                    }
                }
            }
            for (v = 1; v < 64; v++) {
                if (hist[v] > hist[best]) {
                    best = v;
                }
            }
            if (n * 64 > ths * area && hist[best] * 64 > thh * n) {
                out[y * s.w + x] = best;
                mod++;
            } else {
                out[y * s.w + x] = in[y * s.w + x];
                nmod++;
            }
        }
    }
    R(IVE_IRV_MOD) = mod & 0x0FFFFFFF;
    R(IVE_IRV_NMOD) = nmod & 0x0FFFFFFF;
}

static int ive_cmp(const void *a, const void *b)
{
    return *(const int *)a - *(const int *)b;
}

/*
 * The 3x3 median: mode 0 the median, 1 the largest, 2 the smallest, 3 the
 * median of the pixels over MEDN_INVAL_TH (the centre if there are none).
 */
static void ive_median(const uint32_t *regs, const int32_t *in, int32_t *out,
                       IveSize s)
{
    int mode = extract32(R(IVE_FUNC), 2, 2);
    int th = extract32(R(IVE_IRV), 20, 4);
    int x, y, dx, dy;

    for (y = 0; y < s.h; y++) {
        for (x = 0; x < s.w; x++) {
            int v[9], n = 0;

            for (dy = -1; dy <= 1; dy++) {
                for (dx = -1; dx <= 1; dx++) {
                    int p = px(in, s, x + dx, y + dy);

                    if (mode != 3 || p > th) {
                        v[n++] = p;
                    }
                }
            }
            if (!n) {
                out[y * s.w + x] = in[y * s.w + x];
                continue;
            }
            qsort(v, n, sizeof(int), ive_cmp);
            out[y * s.w + x] = mode == 1 ? v[n - 1] : mode == 2 ? v[0] :
                               v[n / 2];
        }
    }
}

/*
 * The 3x3 edge filter: two kernels (or one without direction) of 5-bit
 * two's complement taps, row-major, nine to a kernel from EDGE0.
 */
static void ive_edge(const uint32_t *regs, const int32_t *in, int32_t *mag,
                     int32_t *gxo, int32_t *gyo, int32_t *theta, IveSize s)
{
    uint32_t f = R(IVE_FUNC);
    unsigned shift = extract32(f, 6, 4);
    bool nodir = f & IVE_F_EDGE_NODIR;
    int e1[9], e2[9], i, x, y;

    /* six 5-bit taps a register, from bit 0: e1[0..8], then e2[0..8] */
    for (i = 0; i < 18; i++) {
        uint32_t reg = R(IVE_EDGE0 + 4 * (i / 6));
        int t = sextract32(reg, 5 * (i % 6), 5);

        if (i < 9) {
            e1[i] = t;
        } else {
            e2[i - 9] = t;
        }
    }
    for (y = 0; y < s.h; y++) {
        for (x = 0; x < s.w; x++) {
            int gx = 0, gy = 0, p = y * s.w + x;

            for (i = 0; i < 9; i++) {
                int v = px(in, s, x + i % 3 - 1, y + i / 3 - 1);

                gx += e1[i] * v;
                gy += e2[i] * v;
            }
            if (nodir) {
                mag[p] = MIN(MAX(gx >> shift, 0), 255);
                gxo[p] = gyo[p] = theta[p] = 0;
                continue;
            }
            mag[p] = MIN((abs(gx) + abs(gy)) >> shift, 255);
            gxo[p] = MIN(MAX(gx >> shift, -128), 127);
            gyo[p] = MIN(MAX(gy >> shift, -128), 127);
            /* the direction, one of 16 sectors of [0, 180) degrees */
            if (gx || gy) {
                double a = atan2(gy, gx) * 180.0 / M_PI;

                if (a < 0) {
                    a += 180.0;
                }
                theta[p] = MIN((int)(a * 16.0 / 180.0), 15);
            } else {
                theta[p] = 0;
            }
        }
    }
}

/* suppression: a magnitude over the threshold and its two neighbours */
static void ive_nms(const uint32_t *regs, const int32_t *mag,
                    const int32_t *theta, int32_t *out, IveSize s)
{
    /* the neighbours across the edge for 4 sectors: 0, 45, 90, 135 deg */
    static const int8_t nb[4][2] = { { 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 } };
    int th = extract32(R(IVE_LUT3), 24, 8);
    int x, y;

    for (y = 0; y < s.h; y++) {
        for (x = 0; x < s.w; x++) {
            int p = y * s.w + x, m = mag[p];
            int sec = ((theta[p] + 2) / 4) & 3;
            int a = px(mag, s, x + nb[sec][0], y + nb[sec][1]);
            int b = px(mag, s, x - nb[sec][0], y - nb[sec][1]);

            out[p] = m >= th && m >= a && m >= b ? m : 0;
        }
    }
}

/* the morphology: centre plus the enabled 5x5 neighbours, max or min */
static void ive_morph(const uint32_t *regs, const int32_t *in, int32_t *out,
                      IveSize s)
{
    uint32_t nbm = R(IVE_MORPH);
    bool erode = R(IVE_FUNC) & IVE_F_ERODE;
    int x, y, dx, dy;

    for (y = 0; y < s.h; y++) {
        for (x = 0; x < s.w; x++) {
            int v = in[y * s.w + x], bit = 0;

            for (dy = -2; dy <= 2; dy++) {
                for (dx = -2; dx <= 2; dx++) {
                    int nx = x + dx, ny = y + dy, q;

                    if (!dx && !dy) {
                        continue;
                    }
                    if ((nbm >> bit++) & 1 && nx >= 0 && nx < s.w &&
                        ny >= 0 && ny < s.h) {
                        q = in[ny * s.w + nx];
                        v = erode ? MIN(v, q) : MAX(v, q);
                    }
                }
            }
            out[y * s.w + x] = v;
        }
    }
}

/* the integral image: per channel of 1, 2 or 3 interleaved, inclusive */
static void ive_integral(const uint32_t *regs, const int32_t *in,
                         uint32_t *out, IveSize s)
{
    int ch = extract32(R(IVE_INTEGRAL), 0, 2) + 1, x, y;

    ch = MIN(ch, 3);
    for (y = 0; y < s.h; y++) {
        for (x = 0; x < s.w; x++) {
            uint32_t v = in[y * s.w + x];

            if (x >= ch) {
                v += out[y * s.w + x - ch];
            }
            if (y) {
                v += out[(y - 1) * s.w + x];
                if (x >= ch) {
                    v -= out[(y - 1) * s.w + x - ch];
                }
            }
            out[y * s.w + x] = v;
        }
    }
}

/* the bits an output pixel takes, by OUT_DATA_SEL */
static int ive_out_bits(int sel)
{
    static const uint8_t bits[8] = { 1, 4, 8, 8, 12, 16, 32, 8 };

    return bits[sel & 7];
}

/* put value v of the given width at pixel x of a row, LSB first */
static void ive_put(uint8_t *row, int x, int bits, uint32_t v)
{
    size_t bit = (size_t)x * bits;
    int i;

    if (bits >= 8 && !(bits & 7)) {
        for (i = 0; i < bits / 8; i++) {
            row[bit / 8 + i] = v >> (8 * i);
        }
        return;
    }
    for (i = 0; i < bits; i++, bit++) {
        if ((v >> i) & 1) {
            row[bit / 8] |= 1u << (bit & 7);
        }
    }
}

void nvt_ive_run(uint32_t *regs)
{
    uint32_t f = R(IVE_FUNC);
    int sel = extract32(f, 17, 3), in_sel = extract32(f, 11, 2);
    IveSize s = { extract32(R(IVE_SIZE), 0, 14),
                  extract32(R(IVE_SIZE), 16, 13) };
    uint32_t in_lofs = R(IVE_IN_LOFS) & 0xFFFFC;
    uint32_t out_lofs = R(IVE_OUT_LOFS) & 0xFFFFC;
    int bits = ive_out_bits(sel);
    size_t n = (size_t)s.w * s.h, rowb = ((size_t)s.w * bits + 7) / 8;
    g_autofree int32_t *a = NULL, *b = NULL, *mag = NULL, *gx = NULL;
    g_autofree int32_t *gy = NULL, *theta = NULL, *nms = NULL;
    g_autofree uint32_t *integ = NULL;
    g_autofree uint8_t *row = NULL;
    int32_t *med = NULL, *res;
    const int32_t *magv = NULL, *nmsv = NULL;   /* edge / NMS taps */
    int x, y;

    if (R(IVE_DMA) & IVE_DMA_DISABLE) {
        return;
    }
    if (!s.w || !s.h || s.w > IVE_MAX_W || s.h > IVE_MAX_H ||
        out_lofs < rowb || sel == 7) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ive: %dx%d, output %d, line "
                      "offsets %u / %u\n", s.w, s.h, sel, in_lofs, out_lofs);
        return;
    }
    a = g_try_new(int32_t, n);
    b = g_try_new(int32_t, n);
    row = g_try_malloc(MAX(rowb, (size_t)s.w));
    if (!a || !b || !row) {
        goto nomem;
    }

    /* the input: 8-bit, or the morphology's packed 4- / 1-bit image */
    {
        int ib = (f & IVE_F_MORPH) && in_sel ? (in_sel == 1 ? 4 : 1) : 8;
        size_t inb = ((size_t)s.w * ib + 7) / 8;

        if (in_lofs < inb) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-ive: input line offset %u\n",
                          in_lofs);
            return;
        }
        for (y = 0; y < s.h; y++) {
            if (!ive_rd((R(IVE_IN_ADDR) & ~3u) + (hwaddr)y * in_lofs, row,
                        inb)) {
                return;
            }
            for (x = 0; x < s.w; x++) {
                size_t bit = (size_t)x * ib;

                a[y * s.w + x] = ib == 8 ? row[x] :
                    (row[bit / 8] >> (bit & 7)) & ((1u << ib) - 1);
            }
        }
        if (ib != 8) {
            /* morphology alone, from DRAM */
            ive_morph(regs, a, b, s);
            res = b;
            goto pack;
        }
    }

    /* general filter or IRV, then the median */
    if (f & IVE_F_GEN) {
        ive_gen(regs, a, b, s);
        memcpy(a, b, n * sizeof(*a));
    } else if (f & IVE_F_IRV) {
        ive_irv(regs, a, b, s);
        memcpy(a, b, n * sizeof(*a));
    }
    if (f & IVE_F_MEDIAN) {
        ive_median(regs, a, b, s);
        memcpy(a, b, n * sizeof(*a));
    }
    med = a;

    if (f & IVE_F_INTEGRAL) {
        integ = g_try_new(uint32_t, n);
        if (!integ) {
            goto nomem;
        }
        ive_integral(regs, med, integ, s);
    }

    /* edge, suppression: their planes only for a job that uses them */
    magv = med;
    if ((f & (IVE_F_EDGE | IVE_F_NMS)) || sel == 4 || sel == 5) {
        mag = g_try_new0(int32_t, n);
        gx = g_try_new0(int32_t, n);
        gy = g_try_new0(int32_t, n);
        theta = g_try_new0(int32_t, n);
        if (!mag || !gx || !gy || !theta) {
            goto nomem;
        }
        if (f & IVE_F_EDGE) {
            ive_edge(regs, med, mag, gx, gy, theta, s);
        } else {
            memcpy(mag, med, n * sizeof(*mag));
        }
        magv = mag;
    }
    nmsv = magv;
    if (f & IVE_F_NMS) {
        nms = g_try_new(int32_t, n);
        if (!nms) {
            goto nomem;
        }
        ive_nms(regs, magv, theta, nms, s);
        nmsv = nms;
    }

    /* the LUT's 1- or 4-bit result, then the morphology */
    res = b;
    for (x = 0; x < (int)n; x++) {
        int v = nmsv[x];

        if (f & IVE_F_LUT) {
            if (sel == 0) {
                res[x] = v > (int)extract32(R(IVE_LUT0), 0, 8);
            } else {
                int lvl = 0, i;

                for (i = 0; i < 15; i++) {
                    if (v > (int)extract32(R(IVE_LUT0 + 4 * (i / 4)),
                                           8 * (i % 4), 8)) {
                        lvl++;
                    }
                }
                res[x] = lvl;
            }
        } else {
            res[x] = sel == 0 ? v != 0 : v >> 4;
        }
    }
    if (f & IVE_F_MORPH) {
        ive_morph(regs, res, a, s);
        res = a;
    }

pack:
    for (y = 0; y < s.h; y++) {
        memset(row, 0, rowb);
        for (x = 0; x < s.w; x++) {
            size_t p = (size_t)y * s.w + x;
            uint32_t v;

            switch (sel) {
            case 0:
            case 1:
                v = res[p];
                break;
            case 2:
                v = med ? med[p] : 0;
                break;
            case 3:
                v = nmsv ? nmsv[p] : 0;
                break;
            case 4:
                v = mag ? mag[p] | theta[p] << 8 : 0;
                break;
            case 5:
                v = gx ? (gx[p] & 0xFF) | (gy[p] & 0xFF) << 8 : 0;
                break;
            default:
                v = integ ? integ[p] : 0;
                break;
            }
            ive_put(row, x, bits, v);
        }
        ive_wr((R(IVE_OUT_ADDR) & ~3u) + (hwaddr)y * out_lofs, row, rowb);
    }
    return;

nomem:
    qemu_log_mask(LOG_GUEST_ERROR, "nvt-ive: no memory for a %dx%d job\n",
                  s.w, s.h);
}
