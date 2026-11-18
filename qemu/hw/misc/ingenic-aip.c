/*
 * Ingenic AIP ("ingenic,t41-aip", 0x12B00000, core INTC sources 40, 39
 * and 38): T41's image pre-processor for the NNA, three units sharing a
 * 4 KiB window.  The kernel driver (drivers/video/ingenic_aip) only maps
 * them, hands out a chain buffer per unit (/dev/jzaip_t, _f, _p) and
 * waits for the interrupt; the programming is Ingenic's libaip
 * (magik-toolkit InferenceKit/nna2, behind libvenus's warp_* / crop_* /
 * img_format_convert), which maps the same units at 0x13090000 through
 * /dev/mem, and whose chain descriptors address them there.  The
 * contract below is read from that library (its jz_aip.c, bscaler_api).
 *
 * Every unit: 0x00 CTRL (bit 0 start one job from the registers, bit 1
 * reset -- clears at once, bit 2 start the chain, T: bit 4 start a step),
 * 0x04 status (bit 0 job done, bit 1 chain done, bit 2 timeout; write 1
 * to clear), 0x08 CFG, 0x0C interrupt enable (the library writes ~0).
 *
 *  T (+0x000, "convert"): NV12 to 4-byte colour.  0x10 step source Y,
 *    0x14 Y, 0x18 CbCr, 0x1C destination, 0x20 dst stride << 16 | src
 *    stride, 0x24 height << 16 | width, 0x28..0x48 the 3x3 YUV->RGB
 *    matrix (rows R, G, B; signed Q20, the library negating G's U and V
 *    terms), 0x4C order << 24 | C offset << 8 | Y offset.
 *  F (+0x200, "resize"): bilinear scaling of a box.  0x10 / 0x14 x / y
 *    step (src per dst pixel, Q16), 0x18 / 0x1C x / y start (half-pixel
 *    phase, Q16, +16), 0x20 / 0x24 source Y / CbCr (box origin), 0x28 /
 *    0x2C destination Y / CbCr, 0x30 box h << 16 | w, 0x34 dst h << 16
 *    | w, 0x38 dst stride << 16 | src stride, 0x3C / 0x40 chain address /
 *    length; CFG [9:7] the format: 0 / 1 one byte, 2 four, 3 eight,
 *    4 sixteen, 5 thirty-two, 6 sixteen bytes a pixel (8-bit channels,
 *    NNA feature maps), 7 NV12 (both planes).  The output has the
 *    input's format.
 *  P (+0x300, "perspective"): affine and perspective warps.  0x10 input
 *    0 one byte, 1 two bytes, 2 NV12, 3 four bytes a pixel; 0x14 / 0x18
 *    source Y / CbCr (box origin), 0x1C source stride, 0x20 destination,
 *    0x24 its stride, 0x28 dst h << 16 | w, 0x2C box h << 16 | w, 0x30
 *    the value outside the box, 0x34..0x6C the dst->src 3x3 matrix (per
 *    row two 48-bit and one 59-bit signed fixed-point terms, split over
 *    the words), 0x70..0x90 the YUV->RGB matrix, 0x94 as T's 0x4C, 0x98 /
 *    0x9C chain address / length.  NV12 comes out converted to 4-byte
 *    colour, the others as they are.
 *
 * The 4-byte orders: [2:0] the order of B, G, R in the three colour
 * bytes, lexicographic (0 BGR .. 5 RGB), bit 3 alpha first instead of
 * last -- libvenus's BGRA is 0, RGBA 5, ARGB 13.  Alpha is written 0.
 *
 * A chain is (value, register) word pairs, registers by their 0x1309xxxx
 * address; a pair with bit 31 of the address set writes and then runs
 * the job, and the chain ends with its length.  Jobs complete at once;
 * the status bit and interrupt follow DONE_NS later.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_INGENIC_AIP "ingenic-aip"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicAipState, INGENIC_AIP)

enum { AIP_T, AIP_F, AIP_P, AIP_UNITS };

#define UNIT_REGS       0x100
#define R_CTRL          0x00
#define R_STATUS        0x04
#define R_CFG           0x08
#define R_IRQ_EN        0x0C

#define CTRL_START      (1u << 0)
#define CTRL_RESET      (1u << 1)
#define CTRL_CHAIN      (1u << 2)
#define CTRL_STEP       (1u << 4)

#define ST_DONE         (1u << 0)
#define ST_CHAIN_DONE   (1u << 1)

#define DONE_NS         (50 * SCALE_US)
#define MAX_DIM         4096
#define MAX_CHAIN       (1024 * 216)

struct IngenicAipState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[AIP_UNITS];
    QEMUTimer *timer[AIP_UNITS];
    uint32_t regs[AIP_UNITS][UNIT_REGS / 4];
    uint32_t pending[AIP_UNITS];    /* status bits the timer will set */
};

static const hwaddr unit_base[AIP_UNITS] = { 0x000, 0x200, 0x300 };

#define REG(s, u, off)  ((s)->regs[u][(off) >> 2])

static void aip_update(IngenicAipState *s, int u)
{
    qemu_set_irq(s->irq[u], (REG(s, u, R_STATUS) & REG(s, u, R_IRQ_EN)) != 0);
}

static uint8_t clamp8(int64_t v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* YUV -> the 4 output bytes, by a Q20 matrix, offsets and order (0x4C) */
static void aip_yuv_bgra(const uint32_t *coef, uint32_t ofs, int y, int cb,
                         int cr, uint8_t *out)
{
    static const uint8_t perm[6][3] = {   /* B G R = 0 1 2, lexicographic */
        { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 },
        { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 },
    };
    int yo = ofs & 0xff, co = (ofs >> 8) & 0xff, order = (ofs >> 24) & 0xf;
    int64_t yy = y - yo, u = cb - co, v = cr - co;
    uint8_t rgb[3], bgr[3];
    int i, c0 = order & 8 ? 1 : 0;

    for (i = 0; i < 3; i++) {
        int64_t a = (int64_t)(int32_t)coef[3 * i] * yy +
                    (int64_t)(int32_t)coef[3 * i + 1] * u +
                    (int64_t)(int32_t)coef[3 * i + 2] * v;
        rgb[i] = clamp8((a + (1 << 19)) >> 20);
    }
    bgr[0] = rgb[2];
    bgr[1] = rgb[1];
    bgr[2] = rgb[0];
    out[c0 ? 0 : 3] = 0;
    for (i = 0; i < 3; i++) {
        out[c0 + i] = bgr[perm[(order & 7) < 6 ? order & 7 : 0][i]];
    }
}

/* Read @h rows of @row bytes, @stride apart; NULL when too big */
static uint8_t *aip_read_rect(hwaddr base, uint32_t stride, uint32_t row,
                              uint32_t h)
{
    uint8_t *buf;
    uint32_t r;

    if (!row || !h || row > MAX_DIM * 32 || h > MAX_DIM) {
        return NULL;
    }
    buf = g_malloc(row * h);
    for (r = 0; r < h; r++) {
        dma_memory_read(&address_space_memory, base + (hwaddr)r * stride,
                        buf + r * row, row, MEMTXATTRS_UNSPECIFIED);
    }
    return buf;
}

/* A bilinear sample of channel @c of a @bpp-byte image, Q16 coordinates */
static uint8_t aip_bilinear(const uint8_t *img, int w, int h, int bpp, int c,
                            int64_t xq, int64_t yq)
{
    int64_t x0 = xq >> 16, y0 = yq >> 16;
    int fx = xq & 0xffff, fy = yq & 0xffff;
    int64_t x1 = x0 + 1, y1 = y0 + 1;
    int64_t p00, p01, p10, p11, top, bot;

    x0 = MAX(0, MIN(x0, w - 1));
    x1 = MAX(0, MIN(x1, w - 1));
    y0 = MAX(0, MIN(y0, h - 1));
    y1 = MAX(0, MIN(y1, h - 1));
    if (xq < 0) {
        fx = 0;
    }
    if (yq < 0) {
        fy = 0;
    }
    p00 = img[(y0 * w + x0) * bpp + c];
    p01 = img[(y0 * w + x1) * bpp + c];
    p10 = img[(y1 * w + x0) * bpp + c];
    p11 = img[(y1 * w + x1) * bpp + c];
    top = p00 * 65536 + (p01 - p00) * fx;
    bot = p10 * 65536 + (p11 - p10) * fx;
    return clamp8(((top * 65536 + (bot - top) * fy) + (1ll << 31)) >> 32);
}

/* --- T: NV12 to 4-byte colour ------------------------------------------- */

static void aip_run_t(IngenicAipState *s, bool step)
{
    uint32_t w = REG(s, AIP_T, 0x24) & 0xffff, h = REG(s, AIP_T, 0x24) >> 16;
    uint32_t sstride = REG(s, AIP_T, 0x20) & 0xffff;
    uint32_t dstride = REG(s, AIP_T, 0x20) >> 16;
    hwaddr ya = REG(s, AIP_T, 0x14), uva = REG(s, AIP_T, 0x18);
    hwaddr dst = REG(s, AIP_T, 0x1C);
    uint8_t *yp, *uvp, *out;
    uint32_t x, y;
    bool swap = REG(s, AIP_T, R_CFG) & 1;

    if (step && REG(s, AIP_T, 0x10)) {
        /* a step: the same picture layout from another source */
        uva = REG(s, AIP_T, 0x10) + (uva - ya);
        ya = REG(s, AIP_T, 0x10);
    }
    if (!w || !h || w > MAX_DIM || h > MAX_DIM || !dstride) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-aip: T: bad size %ux%u\n",
                      w, h);
        return;
    }
    yp = aip_read_rect(ya, sstride, w, h);
    uvp = aip_read_rect(uva, sstride, w & ~1u, h / 2);
    out = g_malloc(w * 4);
    for (y = 0; y < h && yp && uvp; y++) {
        for (x = 0; x < w; x++) {
            const uint8_t *c = &uvp[(y / 2) * (w & ~1u) + (x & ~1u)];

            aip_yuv_bgra(&REG(s, AIP_T, 0x28), REG(s, AIP_T, 0x4C),
                         yp[y * w + x], c[swap], c[!swap], &out[x * 4]);
        }
        dma_memory_write(&address_space_memory, dst + (hwaddr)y * dstride,
                         out, w * 4, MEMTXATTRS_UNSPECIFIED);
    }
    g_free(yp);
    g_free(uvp);
    g_free(out);
}

/* --- F: resize ------------------------------------------------------------ */

static const uint8_t f_bpp[8] = { 1, 1, 4, 8, 16, 32, 16, 1 };

static void aip_resize_plane(hwaddr src, uint32_t sstride, uint32_t bw,
                             uint32_t bh, hwaddr dst, uint32_t dstride,
                             uint32_t dw, uint32_t dh, int bpp,
                             int64_t sx, int64_t sy, int64_t x0, int64_t y0)
{
    uint8_t *in = aip_read_rect(src, sstride, bw * bpp, bh);
    uint8_t *out = g_malloc(dw * bpp);
    uint32_t x, y;
    int c;

    for (y = 0; y < dh && in; y++) {
        int64_t yq = y0 + (int64_t)y * sy;

        for (x = 0; x < dw; x++) {
            int64_t xq = x0 + (int64_t)x * sx;

            for (c = 0; c < bpp; c++) {
                out[x * bpp + c] = aip_bilinear(in, bw, bh, bpp, c, xq, yq);
            }
        }
        dma_memory_write(&address_space_memory, dst + (hwaddr)y * dstride,
                         out, dw * bpp, MEMTXATTRS_UNSPECIFIED);
    }
    g_free(in);
    g_free(out);
}

static void aip_run_f(IngenicAipState *s)
{
    int fmt = (REG(s, AIP_F, R_CFG) >> 7) & 7;
    uint32_t bw = REG(s, AIP_F, 0x30) & 0xffff, bh = REG(s, AIP_F, 0x30) >> 16;
    uint32_t dw = REG(s, AIP_F, 0x34) & 0xffff, dh = REG(s, AIP_F, 0x34) >> 16;
    uint32_t sstride = REG(s, AIP_F, 0x38) & 0xffff;
    uint32_t dstride = REG(s, AIP_F, 0x38) >> 16;
    int64_t sx = REG(s, AIP_F, 0x10), sy = REG(s, AIP_F, 0x14);
    int64_t x0 = (int64_t)REG(s, AIP_F, 0x18) - 16;
    int64_t y0 = (int64_t)REG(s, AIP_F, 0x1C) - 16;

    if (!bw || !bh || !dw || !dh || MAX(MAX(bw, bh), MAX(dw, dh)) > MAX_DIM) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-aip: F: bad size %ux%u -> "
                      "%ux%u\n", bw, bh, dw, dh);
        return;
    }
    aip_resize_plane(REG(s, AIP_F, 0x20), sstride, bw, bh,
                     REG(s, AIP_F, 0x28), dstride, dw, dh, f_bpp[fmt],
                     sx, sy, x0, y0);
    if (fmt == 7) {
        /* the CbCr plane: two-byte pixels at half size, the same steps */
        aip_resize_plane(REG(s, AIP_F, 0x24), sstride, bw / 2, bh / 2,
                         REG(s, AIP_F, 0x2C), dstride, dw / 2, dh / 2, 2,
                         sx, sy, x0, y0);
    }
}

/* --- P: affine / perspective ---------------------------------------------- */

static int64_t sext(uint64_t v, int bits)
{
    return (int64_t)(v << (64 - bits)) >> (64 - bits);
}

static void aip_p_matrix(IngenicAipState *s, double m[9])
{
    int r;

    for (r = 0; r < 3; r++) {
        hwaddr b = 0x34 + r * 0x14;
        uint32_t w0 = REG(s, AIP_P, b), w1 = REG(s, AIP_P, b + 4);
        uint32_t w2 = REG(s, AIP_P, b + 8), w3 = REG(s, AIP_P, b + 12);
        uint32_t w4 = REG(s, AIP_P, b + 16);

        m[3 * r] = sext(w0 | ((uint64_t)(w1 & 0xffff) << 32), 48);
        m[3 * r + 1] = sext((w1 >> 16) | ((uint64_t)w2 << 16), 48);
        m[3 * r + 2] = sext(w3 | ((uint64_t)(w4 & 0x7ffffff) << 32), 59);
    }
}

static void aip_run_p(IngenicAipState *s)
{
    static const uint8_t in_bpp[4] = { 1, 2, 1, 4 };
    int fmt = REG(s, AIP_P, 0x10) & 3;
    uint32_t dw = REG(s, AIP_P, 0x28) & 0xffff, dh = REG(s, AIP_P, 0x28) >> 16;
    uint32_t bw = REG(s, AIP_P, 0x2C) & 0xffff, bh = REG(s, AIP_P, 0x2C) >> 16;
    uint32_t sstride = REG(s, AIP_P, 0x1C), dstride = REG(s, AIP_P, 0x24);
    uint8_t pad = REG(s, AIP_P, 0x30);
    int ibpp = in_bpp[fmt], obpp = fmt == 2 ? 4 : ibpp;
    uint8_t *yp, *uvp = NULL, *out;
    double m[9];
    uint32_t x, y;
    int c;

    if (!bw || !bh || !dw || !dh || MAX(MAX(bw, bh), MAX(dw, dh)) > MAX_DIM) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-aip: P: bad size %ux%u -> "
                      "%ux%u\n", bw, bh, dw, dh);
        return;
    }
    aip_p_matrix(s, m);
    yp = aip_read_rect(REG(s, AIP_P, 0x14), sstride, bw * ibpp, bh);
    if (fmt == 2) {
        uvp = aip_read_rect(REG(s, AIP_P, 0x18), sstride, bw & ~1u, bh / 2);
    }
    out = g_malloc(dw * obpp);
    for (y = 0; y < dh && yp && (fmt != 2 || uvp); y++) {
        for (x = 0; x < dw; x++) {
            double wq = m[6] * x + m[7] * y + m[8];
            double sxf = wq ? (m[0] * x + m[1] * y + m[2]) / wq : -1;
            double syf = wq ? (m[3] * x + m[4] * y + m[5]) / wq : -1;
            int64_t xq = llround(sxf * 65536), yq = llround(syf * 65536);
            uint8_t *o = &out[x * obpp];

            if (sxf < 0 || syf < 0 || sxf > bw - 1 || syf > bh - 1) {
                memset(o, pad, obpp);
                continue;
            }
            if (fmt != 2) {
                for (c = 0; c < ibpp; c++) {
                    o[c] = aip_bilinear(yp, bw, bh, ibpp, c, xq, yq);
                }
            } else {
                uint8_t yy = aip_bilinear(yp, bw, bh, 1, 0, xq, yq);
                /* chroma sites at (2i + 0.5, 2j + 0.5) in luma pixels */
                int64_t cx = (xq - 32768) / 2, cy = (yq - 32768) / 2;
                uint8_t cb = aip_bilinear(uvp, bw / 2, bh / 2, 2, 0, cx, cy);
                uint8_t cr = aip_bilinear(uvp, bw / 2, bh / 2, 2, 1, cx, cy);

                aip_yuv_bgra(&REG(s, AIP_P, 0x70), REG(s, AIP_P, 0x94),
                             yy, cb, cr, o);
            }
        }
        dma_memory_write(&address_space_memory,
                         REG(s, AIP_P, 0x20) + (hwaddr)y * dstride,
                         out, dw * obpp, MEMTXATTRS_UNSPECIFIED);
    }
    g_free(yp);
    g_free(uvp);
    g_free(out);
}

/* --- jobs, chains ----------------------------------------------------------- */

static void aip_run(IngenicAipState *s, int u, bool step)
{
    switch (u) {
    case AIP_T:
        aip_run_t(s, step);
        break;
    case AIP_F:
        aip_run_f(s);
        break;
    case AIP_P:
        aip_run_p(s);
        break;
    }
}

static void aip_finish(IngenicAipState *s, int u, uint32_t bits)
{
    s->pending[u] |= bits;
    timer_mod(s->timer[u], qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DONE_NS);
}

static void aip_reg_write(IngenicAipState *s, int u, hwaddr off, uint32_t val);

/* Walk unit @u's chain: register writes, a job at each marked pair */
static void aip_chain(IngenicAipState *s, int u)
{
    hwaddr base, at;
    uint32_t len, pair[2];

    if (u == AIP_F) {
        base = REG(s, u, 0x3C);
        len = REG(s, u, 0x40);
    } else if (u == AIP_P) {
        base = REG(s, u, 0x98);
        len = REG(s, u, 0x9C);
    } else {
        qemu_log_mask(LOG_UNIMP, "ingenic-aip: T has no chain\n");
        return;
    }
    len = MIN(len, MAX_CHAIN) & ~7u;
    for (at = base; at < base + len; at += 8) {
        hwaddr off;
        int tu;

        dma_memory_read(&address_space_memory, at, pair, sizeof(pair),
                        MEMTXATTRS_UNSPECIFIED);
        pair[0] = le32_to_cpu(pair[0]);
        pair[1] = le32_to_cpu(pair[1]);
        off = pair[1] & 0xfff;
        tu = off >= 0x300 ? AIP_P : off >= 0x200 ? AIP_F : AIP_T;
        off -= unit_base[tu];
        if (off == R_CTRL && (pair[1] & 0x80000000u)) {
            /* the end of a node: its job */
            aip_run(s, tu, false);
            continue;
        }
        if (off != R_CTRL && off < UNIT_REGS) {
            aip_reg_write(s, tu, off, pair[0]);
        }
    }
    aip_finish(s, u, ST_CHAIN_DONE);
}

static void aip_done(void *opaque)
{
    IngenicAipState *s = opaque;
    int u;

    for (u = 0; u < AIP_UNITS; u++) {
        if (s->pending[u] && !timer_pending(s->timer[u])) {
            REG(s, u, R_STATUS) |= s->pending[u];
            REG(s, u, R_CTRL) &= ~(CTRL_START | CTRL_CHAIN | CTRL_STEP);
            s->pending[u] = 0;
            aip_update(s, u);
        }
    }
}

static void aip_reg_write(IngenicAipState *s, int u, hwaddr off, uint32_t val)
{
    switch (off) {
    case R_CTRL:
        if (val & CTRL_RESET) {
            timer_del(s->timer[u]);
            s->pending[u] = 0;
            REG(s, u, R_CTRL) = 0;
            REG(s, u, R_STATUS) = 0;
            aip_update(s, u);
            return;
        }
        REG(s, u, R_CTRL) = val;
        if (val & CTRL_CHAIN) {
            aip_chain(s, u);
        } else if (val & (CTRL_START | CTRL_STEP)) {
            aip_run(s, u, !!(val & CTRL_STEP));
            aip_finish(s, u, ST_DONE);
        }
        break;
    case R_STATUS:
        REG(s, u, R_STATUS) &= ~val;
        aip_update(s, u);
        break;
    default:
        REG(s, u, off) = val;
        if (off == R_IRQ_EN) {
            aip_update(s, u);
        }
        break;
    }
}

static int aip_unit(hwaddr addr, hwaddr *off)
{
    int u = addr >= 0x300 ? AIP_P : addr >= 0x200 ? AIP_F : AIP_T;

    *off = addr - unit_base[u];
    return *off < UNIT_REGS ? u : -1;
}

static uint64_t ingenic_aip_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicAipState *s = opaque;
    hwaddr off;
    int u = aip_unit(addr, &off);

    return u < 0 ? 0 : REG(s, u, off);
}

static void ingenic_aip_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    IngenicAipState *s = opaque;
    hwaddr off;
    int u = aip_unit(addr, &off);

    if (u >= 0) {
        aip_reg_write(s, u, off, val);
    }
}

static const MemoryRegionOps ingenic_aip_ops = {
    .read = ingenic_aip_read,
    .write = ingenic_aip_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_aip_reset(DeviceState *dev)
{
    IngenicAipState *s = INGENIC_AIP(dev);
    int u;

    for (u = 0; u < AIP_UNITS; u++) {
        timer_del(s->timer[u]);
        s->pending[u] = 0;
        memset(s->regs[u], 0, sizeof(s->regs[u]));
        aip_update(s, u);
    }
}

static void ingenic_aip_init(Object *obj)
{
    IngenicAipState *s = INGENIC_AIP(obj);
    int u;

    memory_region_init_io(&s->iomem, obj, &ingenic_aip_ops, s,
                          TYPE_INGENIC_AIP, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (u = 0; u < AIP_UNITS; u++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[u]);
        s->timer[u] = timer_new_ns(QEMU_CLOCK_VIRTUAL, aip_done, s);
    }
}

static const VMStateDescription vmstate_ingenic_aip = {
    .name = TYPE_INGENIC_AIP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(regs, IngenicAipState, AIP_UNITS,
                               UNIT_REGS / 4),
        VMSTATE_UINT32_ARRAY(pending, IngenicAipState, AIP_UNITS),
        VMSTATE_TIMER_PTR_ARRAY(timer, IngenicAipState, AIP_UNITS),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_aip_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_aip_reset);
    dc->vmsd = &vmstate_ingenic_aip;
}

static const TypeInfo ingenic_aip_info = {
    .name = TYPE_INGENIC_AIP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicAipState),
    .instance_init = ingenic_aip_init,
    .class_init = ingenic_aip_class_init,
};

static void ingenic_aip_register_types(void)
{
    type_register_static(&ingenic_aip_info);
}

type_init(ingenic_aip_register_types)
