/*
 * SigmaStar / MStar IVE, the image processing engine of infinity3 on.
 *
 * Two banks, 0x1F2A4000 and 0x1F2A4200 (infinity3.dtsi); the vendor's
 * drivers/sstar/ive, hal_ive_reg.h for the layout and hal_ive.c for how it
 * is driven.  Bank 0: reg 0 fire, 2 reset, 0x10 interrupt mask, 0x12 raw,
 * 0x13 final, 0x15 write-one-to-clear; bit 0 is frame done.  Bank 1: the
 * operation (reg 4), formats (5), size less 1 (6, 7), three sources and
 * destinations with strides less 1, a 5x5 mask (0x1A-0x26), thresholds and
 * CSC coefficients.  Addresses are MIU offsets; strides count pixels, as
 * ive_drv_sync_image's sizes show.
 *
 * What the operations compute comes from mi_ive_datatype.h, which defines
 * the thresholds, the LBP compare and "xA + yB", and from how the HAL fills
 * the registers (LBP's threshold is written less one, so the compare is
 * ">").  The sources do not say the following, which are assumptions:
 * the frame's edge repeats its last pixels, the CSC coefficients are
 * Q8 with 12-bit signed offsets, magnitude is |Gx| + |Gy| with the angle in
 * 256 steps of a turn, integral "both" is the 28-bit sum under a 36-bit
 * square sum, an 8-bit SAD is the 16-bit one >> 8 clipped, and YUV input to
 * a filter touches luma only.  MAP's HAL overwrites source 1 with the table,
 * so the image is source 2.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_SSTAR_IVE "sstar-ive"
OBJECT_DECLARE_SIMPLE_TYPE(SstarIveState, SSTAR_IVE)

#define R0_FIRE         0x00
#define R0_RESET        0x02
#define R0_IRQ_MASK     0x10
#define R0_IRQ_FORCE    0x11
#define R0_IRQ_RAW      0x12
#define R0_IRQ_FINAL    0x13
#define R0_IRQ_CLEAR    0x15
#define R0_CYCLES_LO    0x40
#define R0_CYCLES_HI    0x41

#define R1_OP           0x04
#define R1_FMT          0x05
#define IVE_MAX_DIM         8192
#define R1_WIDTH        0x06
#define R1_HEIGHT       0x07
#define R1_SRC(n)       (0x08 + 4 * (n))
#define R1_DST(n)       (0x0A + 4 * (n))
#define R1_SRC_STRIDE(n) (0x14 + 2 * (n))
#define R1_DST_STRIDE(n) (0x15 + 2 * (n))
#define R1_MASK         0x1A
#define R1_THRESH1      0x28
#define R1_THRESH2      0x29
#define R1_FRACTION     0x2A
#define R1_WEIGHT_X     0x2B
#define R1_WEIGHT_Y     0x2C
#define R1_CSC          0x30
#define R1_CSC_OFFSET   0x39
#define R1_CSC_CLAMP    0x3C

#define IRQ_FRAME_DONE  1

enum {
    OP_FILTER, OP_CSC, OP_FILTER_CSC, OP_SOBEL, OP_MAG_ANG, OP_ORD_STAT,
    OP_BERNSEN, OP_DILATE, OP_ERODE, OP_THRESH, OP_THRESH_S16, OP_THRESH_U16,
    OP_AND, OP_OR, OP_XOR, OP_ADD, OP_SUB, OP_16TO8, OP_MAP, OP_HIST,
    OP_INTEGRAL, OP_SAD, OP_NCC, OP_LBP = 0x18,
};

enum {
    FMT_B8C1 = 0, FMT_B8C3_PLAN = 2, FMT_B16C1 = 3, FMT_B8C3_PACK = 4,
    FMT_420SP = 5, FMT_422SP = 6,
};

struct SstarIveState {
    SysBusDevice parent_obj;
    MemoryRegion io0, io1;
    qemu_irq irq;
    uint16_t bank0[0x80];
    uint16_t bank1[0x80];
    uint8_t irq_raw;
};

typedef struct IveJob {
    SstarIveState *s;
    int w, h, op, mode;
} IveJob;

static uint32_t bank1_addr(SstarIveState *s, int reg)
{
    return s->bank1[reg] | (uint32_t)s->bank1[reg + 1] << 16;
}

static void *ive_read(uint32_t miu, uint32_t len)
{
    void *p = g_malloc(len);

    dma_memory_read(&address_space_memory, 0x20000000ull + miu, p, len,
                    MEMTXATTRS_UNSPECIFIED);
    return p;
}

static void ive_write(uint32_t miu, const void *p, uint32_t len)
{
    dma_memory_write(&address_space_memory, 0x20000000ull + miu, p, len,
                     MEMTXATTRS_UNSPECIFIED);
}

/* A plane in w * bpp bytes per line, packed; the strides go here. */
static void *plane_in(IveJob *j, int n, int w, int h, int bpp)
{
    SstarIveState *s = j->s;
    uint32_t a = bank1_addr(s, R1_SRC(n));
    uint32_t stride = (s->bank1[R1_SRC_STRIDE(n)] + 1) * bpp;
    uint8_t *out = g_malloc(w * h * bpp);
    int y;

    for (y = 0; y < h; y++) {
        dma_memory_read(&address_space_memory,
                        0x20000000ull + a + y * stride, out + y * w * bpp,
                        w * bpp, MEMTXATTRS_UNSPECIFIED);
    }
    return out;
}

static void plane_out(IveJob *j, int n, const void *p, int w, int h, int bpp)
{
    SstarIveState *s = j->s;
    uint32_t a = bank1_addr(s, R1_DST(n));
    uint32_t stride = (s->bank1[R1_DST_STRIDE(n)] + 1) * bpp;
    int y;

    for (y = 0; y < h; y++) {
        ive_write(a + y * stride, (const uint8_t *)p + y * w * bpp, w * bpp);
    }
}

static inline int px(const uint8_t *p, int w, int h, int x, int y)
{
    return p[CLAMP(y, 0, h - 1) * w + CLAMP(x, 0, w - 1)];
}

static uint8_t clip8(int v)
{
    return CLAMP(v, 0, 255);
}

static int mask_at(SstarIveState *s, int i)
{
    uint16_t r = s->bank1[R1_MASK + i / 2];

    return (int8_t)(i & 1 ? r >> 8 : r);
}

static int ive_shift(SstarIveState *s)
{
    return s->bank1[R1_MASK + 12] >> 8;
}

/* The 5x5 mask over (x, y), transposed for the vertical gradient. */
static int mask_sum(SstarIveState *s, const uint8_t *p, int w, int h, int x,
                    int y, bool transpose)
{
    int i, j, sum = 0;

    for (j = 0; j < 5; j++) {
        for (i = 0; i < 5; i++) {
            int c = transpose ? mask_at(s, i * 5 + j) : mask_at(s, j * 5 + i);

            sum += c * px(p, w, h, x + i - 2, y + j - 2);
        }
    }
    return sum;
}

static void do_filter(IveJob *j, uint8_t *dst, const uint8_t *src)
{
    int x, y, sh = ive_shift(j->s);

    for (y = 0; y < j->h; y++) {
        for (x = 0; x < j->w; x++) {
            dst[y * j->w + x] = clip8(mask_sum(j->s, src, j->w, j->h, x, y,
                                               false) >> sh);
        }
    }
}

/* Plane 0 of a job's input; luma for the YUV formats. */
static uint8_t *luma_in(IveJob *j)
{
    return plane_in(j, 0, j->w, j->h, 1);
}

static int csc_one(SstarIveState *s, int row, const int *in)
{
    int16_t off = sextract32(s->bank1[R1_CSC_OFFSET + row], 0, 12);
    uint16_t cl = s->bank1[R1_CSC_CLAMP + row];
    int v = 0, i;

    for (i = 0; i < 3; i++) {
        v += sextract32(s->bank1[R1_CSC + row * 3 + i], 0, 12) * in[i];
    }
    v = ((v + 128) >> 8) + off;
    return CLAMP(v, cl & 0xFF, cl >> 8);
}

/* Three channels per pixel from any input format; y overrides the luma. */
static uint8_t *csc_input(IveJob *j, int fmt, const uint8_t *y)
{
    uint8_t *rgb = g_malloc(j->w * j->h * 3);
    uint8_t *a, *b = NULL, *c = NULL;
    int px_, x, l;

    switch (fmt) {
    case FMT_B8C3_PACK:
        a = plane_in(j, 0, j->w * 3, j->h, 1);
        memcpy(rgb, a, j->w * j->h * 3);
        g_free(a);
        return rgb;
    case FMT_B8C3_PLAN:
        a = luma_in(j);
        b = plane_in(j, 1, j->w, j->h, 1);
        c = plane_in(j, 2, j->w, j->h, 1);
        for (px_ = 0; px_ < j->w * j->h; px_++) {
            rgb[px_ * 3] = y ? y[px_] : a[px_];
            rgb[px_ * 3 + 1] = b[px_];
            rgb[px_ * 3 + 2] = c[px_];
        }
        g_free(a);
        g_free(b);
        g_free(c);
        return rgb;
    default:
        a = luma_in(j);
        l = fmt == FMT_420SP ? (j->h + 1) / 2 : j->h;
        b = plane_in(j, 1, j->w & ~1, l, 1);
        for (px_ = 0; px_ < j->w * j->h; px_++) {
            int ly = px_ / j->w, row = fmt == FMT_420SP ? ly / 2 : ly;

            /* an odd width's last pixel shares the last pair */
            x = MIN((px_ % j->w) & ~1, (j->w & ~1) - 2);
            rgb[px_ * 3] = y ? y[px_] : a[px_];
            rgb[px_ * 3 + 1] = b[row * (j->w & ~1) + x];
            rgb[px_ * 3 + 2] = b[row * (j->w & ~1) + x + 1];
        }
        g_free(a);
        g_free(b);
        return rgb;
    }
}

static void csc_output(IveJob *j, int fmt, const uint8_t *in)
{
    int n = j->w * j->h, i, c;
    g_autofree uint8_t *out = g_malloc(n * 3);

    for (i = 0; i < n; i++) {
        int ch[3] = { in[i * 3], in[i * 3 + 1], in[i * 3 + 2] };

        for (c = 0; c < 3; c++) {
            out[i * 3 + c] = csc_one(j->s, c, ch);
        }
    }
    if (fmt == FMT_B8C3_PACK) {
        plane_out(j, 0, out, j->w * 3, j->h, 1);
    } else {
        g_autofree uint8_t *pl = g_malloc(n);

        for (c = 0; c < 3; c++) {
            for (i = 0; i < n; i++) {
                pl[i] = out[i * 3 + c];
            }
            plane_out(j, c, pl, j->w, j->h, 1);
        }
    }
}

static void do_sobel(IveJob *j)
{
    g_autofree uint8_t *src = luma_in(j);
    int n = j->w * j->h, x, y;
    g_autofree int16_t *h = g_new(int16_t, n), *v = g_new(int16_t, n);
    int mode = j->s->bank1[R1_FMT] >> 8;

    for (y = 0; y < j->h; y++) {
        for (x = 0; x < j->w; x++) {
            h[y * j->w + x] = mask_sum(j->s, src, j->w, j->h, x, y, false);
            v[y * j->w + x] = mask_sum(j->s, src, j->w, j->h, x, y, true);
        }
    }
    /* A single direction goes to destination 1 */
    if (mode == 2) {
        plane_out(j, 0, v, j->w, j->h, 2);
        return;
    }
    plane_out(j, 0, h, j->w, j->h, 2);
    if (mode == 0) {
        plane_out(j, 1, v, j->w, j->h, 2);
    }
}

static void do_mag_ang(IveJob *j)
{
    g_autofree uint8_t *src = luma_in(j);
    int n = j->w * j->h, x, y;
    g_autofree uint16_t *mag = g_new(uint16_t, n);
    g_autofree uint8_t *ang = g_new(uint8_t, n);
    int thresh = j->s->bank1[R1_THRESH1];

    for (y = 0; y < j->h; y++) {
        for (x = 0; x < j->w; x++) {
            int gx = mask_sum(j->s, src, j->w, j->h, x, y, false);
            int gy = mask_sum(j->s, src, j->w, j->h, x, y, true);
            int m = abs(gx) + abs(gy);

            mag[y * j->w + x] = m < thresh ? 0 : m;
            ang[y * j->w + x] = (int)(atan2(gy, gx) * 128.0 / M_PI) & 0xFF;
        }
    }
    plane_out(j, 0, mag, j->w, j->h, 2);
    if (j->s->bank1[R1_FMT] >> 8) {
        plane_out(j, 1, ang, j->w, j->h, 1);
    }
}

static int cmp_u8(const void *a, const void *b)
{
    return *(const uint8_t *)a - *(const uint8_t *)b;
}

static void do_ord_stat(IveJob *j)
{
    g_autofree uint8_t *src = luma_in(j);
    g_autofree uint8_t *dst = g_malloc(j->w * j->h);
    int x, y, a, b;

    for (y = 0; y < j->h; y++) {
        for (x = 0; x < j->w; x++) {
            uint8_t v[9];
            int k = 0;

            for (b = -1; b <= 1; b++) {
                for (a = -1; a <= 1; a++) {
                    v[k++] = px(src, j->w, j->h, x + a, y + b);
                }
            }
            qsort(v, 9, 1, cmp_u8);
            dst[y * j->w + x] = v[j->mode == 0 ? 4 : j->mode == 1 ? 8 : 0];
        }
    }
    plane_out(j, 0, dst, j->w, j->h, 1);
}

static void do_bernsen(IveJob *j)
{
    g_autofree uint8_t *src = luma_in(j);
    g_autofree uint8_t *dst = g_malloc(j->w * j->h);
    int r = j->mode & 1 ? 2 : 1, contrast = j->mode >> 1;
    int thresh = j->s->bank1[R1_THRESH1], x, y, a, b;

    for (y = 0; y < j->h; y++) {
        for (x = 0; x < j->w; x++) {
            int lo = 255, hi = 0, t;

            for (b = -r; b <= r; b++) {
                for (a = -r; a <= r; a++) {
                    int v = px(src, j->w, j->h, x + a, y + b);

                    lo = MIN(lo, v);
                    hi = MAX(hi, v);
                }
            }
            t = (lo + hi) >> 1;
            if (contrast && hi - lo < thresh) {
                t = thresh;
            }
            dst[y * j->w + x] = src[y * j->w + x] >= t ? 255 : 0;
        }
    }
    plane_out(j, 0, dst, j->w, j->h, 1);
}

static void do_morph(IveJob *j, bool dilate)
{
    g_autofree uint8_t *src = luma_in(j);
    g_autofree uint8_t *dst = g_malloc(j->w * j->h);
    int x, y, a, b;

    for (y = 0; y < j->h; y++) {
        for (x = 0; x < j->w; x++) {
            int v = dilate ? 0 : 255;

            for (b = 0; b < 5; b++) {
                for (a = 0; a < 5; a++) {
                    int p = px(src, j->w, j->h, x + a - 2, y + b - 2);

                    if (mask_at(j->s, b * 5 + a)) {
                        v = dilate ? MAX(v, p) : MIN(v, p);
                    }
                }
            }
            dst[y * j->w + x] = v;
        }
    }
    plane_out(j, 0, dst, j->w, j->h, 1);
}

static int thresh_tri(int v, int lo, int hi, int cmin, int cmid, int cmax,
                      int mode)
{
    static const uint8_t lut[8][3] = {
        /* 0 min, 1 mid, 2 max, 3 src: at or under low, between, over high */
        { 0, 2, 2 }, { 3, 2, 2 }, { 0, 3, 3 }, { 0, 1, 2 },
        { 3, 1, 2 }, { 0, 1, 3 }, { 0, 3, 2 }, { 3, 1, 3 },
    };
    int sel = lut[mode & 7][v <= lo ? 0 : v <= hi ? 1 : 2];

    return sel == 0 ? cmin : sel == 1 ? cmid : sel == 2 ? cmax : v;
}

static void do_thresh(IveJob *j)
{
    SstarIveState *s = j->s;
    g_autofree uint8_t *src = luma_in(j);
    g_autofree uint8_t *dst = g_malloc(j->w * j->h);
    int lo = s->bank1[R1_THRESH1], hi = s->bank1[R1_THRESH2];
    int m0 = s->bank1[R1_MASK], m1 = s->bank1[R1_MASK + 1];
    int i;

    /* The two-level modes compare with the low threshold alone */
    if (j->mode < 3) {
        hi = lo;
    }
    for (i = 0; i < j->w * j->h; i++) {
        dst[i] = thresh_tri(src[i], lo, hi, m0 & 0xFF, m0 >> 8, m1 & 0xFF,
                            j->mode);
    }
    plane_out(j, 0, dst, j->w, j->h, 1);
}

static void do_thresh16(IveJob *j, bool is_signed)
{
    SstarIveState *s = j->s;
    g_autofree uint8_t *raw = plane_in(j, 0, j->w, j->h, 2);
    g_autofree uint8_t *dst = g_malloc(j->w * j->h);
    int lo = is_signed ? (int16_t)s->bank1[R1_THRESH1] : s->bank1[R1_THRESH1];
    int hi = is_signed ? (int16_t)s->bank1[R1_THRESH2] : s->bank1[R1_THRESH2];
    int m0 = s->bank1[R1_MASK], m1 = s->bank1[R1_MASK + 1];
    /* bit 0: original between the thresholds, else mid */
    bool ori = j->mode & 1;
    int i;

    for (i = 0; i < j->w * j->h; i++) {
        uint16_t u = lduw_le_p(raw + i * 2);
        int v = is_signed ? (int16_t)u : u;

        dst[i] = v <= lo ? m0 & 0xFF : v > hi ? m1 & 0xFF :
                 ori ? v : m0 >> 8;
    }
    plane_out(j, 0, dst, j->w, j->h, 1);
}

static void do_binary(IveJob *j)
{
    SstarIveState *s = j->s;
    g_autofree uint8_t *a = luma_in(j);
    g_autofree uint8_t *b = plane_in(j, 1, j->w, j->h, 1);
    g_autofree uint8_t *d = g_malloc(j->w * j->h);
    uint32_t wx = s->bank1[R1_WEIGHT_X], wy = s->bank1[R1_WEIGHT_Y];
    int i;

    for (i = 0; i < j->w * j->h; i++) {
        switch (j->op) {
        case OP_AND:
            d[i] = a[i] & b[i];
            break;
        case OP_OR:
            d[i] = a[i] | b[i];
            break;
        case OP_XOR:
            d[i] = a[i] ^ b[i];
            break;
        case OP_ADD:
            d[i] = clip8((a[i] * wx + b[i] * wy + 0x8000) >> 16);
            break;
        default:
            d[i] = j->mode ? (uint8_t)((a[i] - b[i]) >> 1) :
                             abs(a[i] - b[i]);
            break;
        }
    }
    plane_out(j, 0, d, j->w, j->h, 1);
}

static void do_16to8(IveJob *j)
{
    SstarIveState *s = j->s;
    g_autofree uint8_t *raw = plane_in(j, 0, j->w, j->h, 2);
    g_autofree uint8_t *dst = g_malloc(j->w * j->h);
    int64_t f = s->bank1[R1_FRACTION];
    int bias = (int8_t)s->bank1[R1_MASK], i;

    for (i = 0; i < j->w * j->h; i++) {
        uint16_t u = lduw_le_p(raw + i * 2);
        int64_t v;

        switch (j->mode) {
        case 0:
            v = ((int16_t)u * f) >> 16;
            dst[i] = (uint8_t)(int8_t)CLAMP(v, -128, 127);
            break;
        case 1:
            v = (abs((int16_t)u) * f) >> 16;
            dst[i] = clip8(v);
            break;
        case 2:
            v = (((int16_t)u * f) >> 16) + bias;
            dst[i] = clip8(v);
            break;
        default:
            dst[i] = clip8((u * f) >> 16);
            break;
        }
    }
    plane_out(j, 0, dst, j->w, j->h, 1);
}

static void do_map(IveJob *j)
{
    g_autofree uint8_t *map = ive_read(bank1_addr(j->s, R1_SRC(0)), 256);
    g_autofree uint8_t *src = plane_in(j, 1, j->w, j->h, 1);
    int i;

    for (i = 0; i < j->w * j->h; i++) {
        src[i] = map[src[i]];
    }
    plane_out(j, 0, src, j->w, j->h, 1);
}

static void do_hist(IveJob *j)
{
    g_autofree uint8_t *src = luma_in(j);
    uint32_t hist[256] = {};
    int i;

    for (i = 0; i < j->w * j->h; i++) {
        hist[src[i]]++;
    }
    for (i = 0; i < 256; i++) {
        hist[i] = cpu_to_le32(hist[i]);
    }
    ive_write(bank1_addr(j->s, R1_DST(0)), hist, sizeof(hist));
}

static void do_integral(IveJob *j)
{
    g_autofree uint8_t *src = luma_in(j);
    int n = j->w * j->h, x, y;
    g_autofree uint64_t *sum = g_new(uint64_t, n);
    g_autofree uint64_t *sq = g_new(uint64_t, n);

    for (y = 0; y < j->h; y++) {
        uint64_t rs = 0, rq = 0;

        for (x = 0; x < j->w; x++) {
            int v = src[y * j->w + x];

            rs += v;
            rq += v * v;
            sum[y * j->w + x] = rs + (y ? sum[(y - 1) * j->w + x] : 0);
            sq[y * j->w + x] = rq + (y ? sq[(y - 1) * j->w + x] : 0);
        }
    }
    if (j->mode == 1) {
        g_autofree uint32_t *o = g_new(uint32_t, n);

        for (x = 0; x < n; x++) {
            o[x] = cpu_to_le32(sum[x]);
        }
        plane_out(j, 0, o, j->w, j->h, 4);
    } else {
        for (x = 0; x < n; x++) {
            sq[x] = cpu_to_le64(j->mode == 2 ? sq[x] :
                                (sum[x] & 0xFFFFFFF) | sq[x] << 28);
        }
        plane_out(j, 0, sq, j->w, j->h, 8);
    }
}

static void do_sad(IveJob *j)
{
    SstarIveState *s = j->s;
    g_autofree uint8_t *a = luma_in(j);
    g_autofree uint8_t *b = plane_in(j, 1, j->w, j->h, 1);
    int bs = 4 << MIN(j->mode, 2), cols = j->w / bs, rows = j->h / bs;
    int out = s->bank1[R1_FMT] >> 8, thresh = s->bank1[R1_THRESH1];
    int m = s->bank1[R1_MASK], x, y, c, r;
    g_autofree uint16_t *sad = g_new0(uint16_t, cols * rows);
    g_autofree uint8_t *sad8 = g_new0(uint8_t, cols * rows);
    g_autofree uint8_t *thr = g_new0(uint8_t, cols * rows);

    for (r = 0; r < rows; r++) {
        for (c = 0; c < cols; c++) {
            uint32_t v = 0;

            for (y = 0; y < bs; y++) {
                for (x = 0; x < bs; x++) {
                    int o = (r * bs + y) * j->w + c * bs + x;

                    v += abs(a[o] - b[o]);
                }
            }
            sad[r * cols + c] = cpu_to_le16(MIN(v, 0xFFFF));
            sad8[r * cols + c] = MIN(v >> 8, 255);
            thr[r * cols + c] = (int)v < thresh ? m & 0xFF : m >> 8;
        }
    }
    if (out == 0 || out == 2) {
        plane_out(j, 0, sad, cols, rows, 2);
    } else if (out == 1 || out == 3) {
        plane_out(j, 0, sad8, cols, rows, 1);
    }
    if (out == 0 || out == 1) {
        plane_out(j, 1, thr, cols, rows, 1);
    } else if (out == 4) {
        plane_out(j, 0, thr, cols, rows, 1);
    }
}

static void do_ncc(IveJob *j)
{
    g_autofree uint8_t *a = luma_in(j);
    g_autofree uint8_t *b = plane_in(j, 1, j->w, j->h, 1);
    uint64_t r[3] = {};
    int i;

    for (i = 0; i < j->w * j->h; i++) {
        r[0] += a[i] * b[i];
        r[1] += a[i] * a[i];
        r[2] += b[i] * b[i];
    }
    for (i = 0; i < 3; i++) {
        r[i] = cpu_to_le64(r[i]);
    }
    ive_write(bank1_addr(j->s, R1_DST(0)), r, sizeof(r));
}

static void do_lbp(IveJob *j)
{
    static const int8_t nx[8] = { -1, 0, 1, 1, 1, 0, -1, -1 };
    static const int8_t ny[8] = { -1, -1, -1, 0, 1, 1, 1, 0 };
    g_autofree uint8_t *src = luma_in(j);
    g_autofree uint8_t *dst = g_malloc(j->w * j->h);
    int t = j->mode ? (uint16_t)j->s->bank1[R1_THRESH1] :
                      (int16_t)j->s->bank1[R1_THRESH1];
    int x, y, n;

    for (y = 0; y < j->h; y++) {
        for (x = 0; x < j->w; x++) {
            int c = src[y * j->w + x], v = 0;

            for (n = 0; n < 8; n++) {
                int d = px(src, j->w, j->h, x + nx[n], y + ny[n]) - c;

                if ((j->mode ? abs(d) : d) > t) {
                    v |= 1 << n;
                }
            }
            dst[y * j->w + x] = v;
        }
    }
    plane_out(j, 0, dst, j->w, j->h, 1);
}

static void ive_run(SstarIveState *s)
{
    IveJob job = {
        .s = s,
        .w = s->bank1[R1_WIDTH] + 1,
        .h = s->bank1[R1_HEIGHT] + 1,
        .op = s->bank1[R1_OP] & 0xFF,
        .mode = s->bank1[R1_OP] >> 8,
    };
    IveJob *j = &job;
    int infmt = s->bank1[R1_FMT] & 0xFF, outfmt = s->bank1[R1_FMT] >> 8;
    uint32_t cycles = j->w * j->h;

    /* the size registers reach 65536: past int's products below */
    if (j->w < 2 || j->w > IVE_MAX_DIM || j->h > IVE_MAX_DIM) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-ive: picture %dx%d\n", j->w, j->h);
        return;
    }
    switch (j->op) {
    case OP_FILTER: {
        g_autofree uint8_t *src = luma_in(j);
        g_autofree uint8_t *dst = g_malloc(j->w * j->h);

        do_filter(j, dst, src);
        plane_out(j, 0, dst, j->w, j->h, 1);
        if (infmt == FMT_420SP || infmt == FMT_422SP) {
            int l = infmt == FMT_420SP ? (j->h + 1) / 2 : j->h;
            g_autofree uint8_t *uv = plane_in(j, 1, j->w & ~1, l, 1);

            plane_out(j, 1, uv, j->w & ~1, l, 1);
        }
        break;
    }
    case OP_CSC: {
        g_autofree uint8_t *in = csc_input(j, infmt, NULL);

        csc_output(j, outfmt, in);
        break;
    }
    case OP_FILTER_CSC: {
        g_autofree uint8_t *src = luma_in(j);
        g_autofree uint8_t *y = g_malloc(j->w * j->h);
        g_autofree uint8_t *in = NULL;

        do_filter(j, y, src);
        in = csc_input(j, infmt, y);
        csc_output(j, outfmt, in);
        break;
    }
    case OP_SOBEL:
        do_sobel(j);
        break;
    case OP_MAG_ANG:
        do_mag_ang(j);
        break;
    case OP_ORD_STAT:
        do_ord_stat(j);
        break;
    case OP_BERNSEN:
        do_bernsen(j);
        break;
    case OP_DILATE:
    case OP_ERODE:
        do_morph(j, j->op == OP_DILATE);
        break;
    case OP_THRESH:
        do_thresh(j);
        break;
    case OP_THRESH_S16:
    case OP_THRESH_U16:
        do_thresh16(j, j->op == OP_THRESH_S16);
        break;
    case OP_AND: case OP_OR: case OP_XOR: case OP_ADD: case OP_SUB:
        do_binary(j);
        break;
    case OP_16TO8:
        do_16to8(j);
        break;
    case OP_MAP:
        do_map(j);
        break;
    case OP_HIST:
        do_hist(j);
        break;
    case OP_INTEGRAL:
        do_integral(j);
        break;
    case OP_SAD:
        do_sad(j);
        break;
    case OP_NCC:
        do_ncc(j);
        break;
    case OP_LBP:
        do_lbp(j);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "sstar-ive: operation 0x%x\n", j->op);
        break;
    }
    s->bank0[R0_CYCLES_LO] = cycles;
    s->bank0[R0_CYCLES_HI] = cycles >> 16;
}

static uint8_t ive_final(SstarIveState *s)
{
    return (s->irq_raw | (s->bank0[R0_IRQ_FORCE] & 0xFF)) &
           ~s->bank0[R0_IRQ_MASK];
}

static void ive_update(SstarIveState *s)
{
    qemu_set_irq(s->irq, ive_final(s) != 0);
}

static uint64_t ive0_read(void *opaque, hwaddr off, unsigned size)
{
    SstarIveState *s = opaque;
    int reg = (off >> 2) & 0x7F;
    uint16_t v;

    switch (reg) {
    case R0_IRQ_RAW:
        v = s->irq_raw;
        break;
    case R0_IRQ_FINAL:
        v = ive_final(s);
        break;
    default:
        v = s->bank0[reg];
        break;
    }
    return off & 1 ? v >> 8 : v;
}

static void ive0_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarIveState *s = opaque;
    int reg = (off >> 2) & 0x7F;

    switch (reg) {
    case R0_FIRE:
        if (val & 1) {
            ive_run(s);
            s->irq_raw |= IRQ_FRAME_DONE;
        }
        break;
    case R0_RESET:
        if (val & 1) {
            s->irq_raw = 0;
        }
        break;
    case R0_IRQ_CLEAR:
        s->irq_raw &= ~val;
        break;
    default:
        s->bank0[reg] = val;
        break;
    }
    ive_update(s);
}

static uint64_t ive1_read(void *opaque, hwaddr off, unsigned size)
{
    SstarIveState *s = opaque;
    uint16_t v = s->bank1[(off >> 2) & 0x7F];

    return off & 1 ? v >> 8 : v;
}

static void ive1_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarIveState *s = opaque;

    s->bank1[(off >> 2) & 0x7F] = val;
}

static const MemoryRegionOps ive0_ops = {
    .read = ive0_read,
    .write = ive0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps ive1_ops = {
    .read = ive1_read,
    .write = ive1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_ive_reset(DeviceState *dev)
{
    SstarIveState *s = SSTAR_IVE(dev);

    memset(s->bank0, 0, sizeof(s->bank0));
    memset(s->bank1, 0, sizeof(s->bank1));
    s->bank0[R0_IRQ_MASK] = 0xFF;
    s->irq_raw = 0;
    ive_update(s);
}

static void sstar_ive_init(Object *obj)
{
    SstarIveState *s = SSTAR_IVE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->io0, obj, &ive0_ops, s, "sstar-ive.0", 0x200);
    memory_region_init_io(&s->io1, obj, &ive1_ops, s, "sstar-ive.1", 0x200);
    sysbus_init_mmio(sbd, &s->io0);
    sysbus_init_mmio(sbd, &s->io1);
    sysbus_init_irq(sbd, &s->irq);
}

static const VMStateDescription vmstate_sstar_ive = {
    .name = TYPE_SSTAR_IVE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(bank0, SstarIveState, 0x80),
        VMSTATE_UINT16_ARRAY(bank1, SstarIveState, 0x80),
        VMSTATE_UINT8(irq_raw, SstarIveState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_ive_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_ive_reset);
    dc->vmsd = &vmstate_sstar_ive;
}

static const TypeInfo sstar_ive_info = {
    .name           = TYPE_SSTAR_IVE,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarIveState),
    .instance_init  = sstar_ive_init,
    .class_init     = sstar_ive_class_init,
};

static void sstar_ive_register_types(void)
{
    type_register_static(&sstar_ive_info);
}

type_init(sstar_ive_register_types)
