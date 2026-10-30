/*
 * Ingenic BSCALER ("ingenic,t40-bscaler", 0x13090000, core INTC sources
 * 40 and 39): T40's image engine for the NNA.  The kernel driver
 * (drivers/video/ingenic_bscaler) only enables the clocks, waits for the
 * two interrupts and clears their status bits; the programming is in
 * Ingenic's libvenus (magik-toolkit InferenceKit/nna1: bscaler_api,
 * bscaler_hal, bscaler_segmentation), which maps the registers through
 * /dev/mem.  The contract below is read from that library: its register
 * writes and chains logged while it ran on the host, and its code.
 *
 * Two units in one 256-byte window:
 *
 *  C (0x00..0x7F, "frmc", IRQ 39): warps -- resize, affine, perspective
 *    -- of NV12 or 4-byte pixels, with NV12 to 4-byte colour on the way.
 *    The library splits each box into tiles of at most 64x64 output
 *    pixels and loads one tile's registers per job:
 *      0x04  bit 0 source 4-byte (else NV12), bit 3 destination 4-byte,
 *            [15:8] the value outside the source, other bits storage
 *      0x08 / 0x0C  source Y / CbCr of the tile
 *      0x10  source tile h << 16 | w        0x14  source stride
 *      0x18  destination planes: a FIFO, Y then (NV12) CbCr
 *      0x1C  destination tile h << 16 | w   0x20  destination stride
 *      0x40 / 0x44  source / destination tile origin, y << 16 | x, in
 *            whole-picture coordinates
 *      0x48..0x68  the destination -> source 3x3 matrix, signed Q16, row
 *            by row (a resize: scale and the half-pixel phase; affine and
 *            perspective: the inverse of the caller's forward matrix)
 *      0x28 / 0x2C  a byte table per output column (storage)
 *      0x00  0x201 runs the job; bit 8 is set when the chain is done
 *    and the jobs come as a chain: 0x34 its physical address, 0x38 its
 *    length in bytes, 0x3C interrupt enable, 0x30 bit 0 start (bit 1:
 *    the chain is in ORAM, bit 16 soft reset).  A chain is (value,
 *    register) word pairs, bit 31 of the register marking a job's last
 *    one.  When it ends, 0x30 bit 4 and 0x00 bit 8 are set and the
 *    interrupt raised; the driver clears them (0x30 bit 6 would be a
 *    timeout).
 *  T (0x80..0xFF, "frmt", IRQ 40): NV12 to 4-byte colour by lines, for
 *    libvenus's warp_covert_nv2bgr.  0x88 / 0x8C source Y / CbCr, 0x90
 *    h << 16 | w, 0x94 source stride, 0x98 destination, 0xA4 its
 *    stride, 0xAC bit 0 source 4-byte, bit 2 destination 4-byte, 0x80
 *    (0x201) arms it, 0x84 lines << 16 | 1 converts the next lines
 *    (0xA0 / 0xA8 / 0xF0 / 0xF4 storage: a ring size and packed
 *    parameters); 0x84 bit 2 is a step's done flag the driver clears.
 *    0x80 bit 0 stays set while the frame is in progress -- the library
 *    holds its lock until it reads it clear -- and drops when a step
 *    reaches the last line, with bit 8 set.  0xBC is the interrupt
 *    enable, 0xB0 bit 16 a soft reset.
 *  Common: 0xC0..0xE0 the YUV->colour matrix, signed Q20, rows B, G, R
 *    (G's U and V terms subtracted: libvenus's defaults are all
 *    positive), 0xE4 [31:24] CbCr offset, [23:16] Y offset, [7:4] C's
 *    and [3:0] T's output order: [2:0] the order of B, G, R in the colour
 *    bytes, lexicographic (0 BGR .. 5 RGB), bit 3 alpha first -- BGRA 0,
 *    RGBA 5, ARGB 13.  Alpha is written 0.
 *
 * Sampling is bilinear, the edge pixels repeated; a source position more
 * than a pixel outside the tile takes the 0x04 value (128 for CbCr).
 * Jobs complete at once; status and interrupt follow DONE_NS later.
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

#define TYPE_INGENIC_BSCALER "ingenic-bscaler"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicBscalerState, INGENIC_BSCALER)

enum { BS_T, BS_C, BS_UNITS };          /* sysbus IRQ 0 / 1 */

#define BS_REGS         0x100

#define C_STAT          0x00
#define C_FMT           0x04
#define C_SRC_Y         0x08
#define C_SRC_UV        0x0C
#define C_SRC_SIZE      0x10
#define C_SRC_STRIDE    0x14
#define C_DST_PLANE     0x18
#define C_DST_SIZE      0x1C
#define C_DST_STRIDE    0x20
#define C_CTRL          0x30
#define C_CHAIN         0x34
#define C_CHAIN_LEN     0x38
#define C_IRQ_EN        0x3C
#define C_SRC_ORG       0x40
#define C_DST_ORG       0x44
#define C_MATRIX        0x48

#define T_MODE          0x80
#define T_STEP          0x84
#define T_SRC_Y         0x88
#define T_SRC_UV        0x8C
#define T_SIZE          0x90
#define T_SRC_STRIDE    0x94
#define T_DST           0x98
#define T_DST_STRIDE    0xA4
#define T_FMT           0xAC
#define T_CTRL          0xB0
#define T_IRQ_EN        0xBC

#define COEF            0xC0
#define OFS_ORDER       0xE4

#define CTRL_START      (1u << 0)
#define CTRL_DONE       (1u << 4)
#define CTRL_TIMEOUT    (1u << 6)
#define CTRL_RESET      (1u << 16)
#define STAT_DONE       (1u << 8)
#define STEP_DONE       (1u << 2)

#define DONE_NS         (50 * SCALE_US)
#define MAX_DIM         4096
#define MAX_CHAIN       (1 << 20)

struct IngenicBscalerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[BS_UNITS];
    QEMUTimer *timer[BS_UNITS];
    uint32_t regs[BS_REGS / 4];
    uint32_t planes[2];             /* the 0x18 FIFO */
    uint32_t nplanes;
    uint32_t t_row;                 /* T: the next line to convert */
    uint32_t t_frame_done;          /* T: the last step finished the frame */
    uint32_t pending[BS_UNITS];     /* the timer is to set them */
};

#define REG(s, off)     ((s)->regs[(off) >> 2])

static void bs_update(IngenicBscalerState *s)
{
    bool t = (REG(s, T_STEP) & STEP_DONE) || (REG(s, T_MODE) & STAT_DONE) ||
             (REG(s, T_CTRL) & (CTRL_DONE | CTRL_TIMEOUT));
    bool c = (REG(s, C_STAT) & STAT_DONE) ||
             (REG(s, C_CTRL) & (CTRL_DONE | CTRL_TIMEOUT));

    qemu_set_irq(s->irq[BS_T], t && REG(s, T_IRQ_EN));
    qemu_set_irq(s->irq[BS_C], c && REG(s, C_IRQ_EN));
}

static uint8_t clamp8(double v)
{
    return v < 0 ? 0 : v > 255 ? 255 : (uint8_t)lrint(v);
}

/* YUV -> the 4 colour bytes, by the Q20 matrix, offsets and @order */
static void bs_yuv_color(IngenicBscalerState *s, int order, double y,
                         double cb, double cr, uint8_t *out)
{
    static const uint8_t perm[6][3] = {   /* B G R = 0 1 2, lexicographic */
        { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 },
        { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 },
    };
    static const int sign[3][3] = { { 1, 1, 1 }, { 1, -1, -1 }, { 1, 1, 1 } };
    uint32_t ofs = REG(s, OFS_ORDER);
    double yy = y - ((ofs >> 16) & 0xff);
    double u = cb - (ofs >> 24), v = cr - (ofs >> 24);
    uint8_t bgr[3];
    int i, c0 = order & 8 ? 1 : 0;

    for (i = 0; i < 3; i++) {
        const uint32_t *k = &REG(s, COEF + 12 * i);

        bgr[i] = clamp8(((int32_t)k[0] * sign[i][0] * yy +
                         (int32_t)k[1] * sign[i][1] * u +
                         (int32_t)k[2] * sign[i][2] * v) / (1 << 20));
    }
    out[c0 ? 0 : 3] = 0;
    for (i = 0; i < 3; i++) {
        out[c0 + i] = bgr[perm[(order & 7) < 6 ? order & 7 : 0][i]];
    }
}

/* Read @h rows of @row bytes, @stride apart; NULL when too big */
static uint8_t *bs_read_rect(hwaddr base, uint32_t stride, uint32_t row,
                             uint32_t h)
{
    uint8_t *buf;
    uint32_t r;

    if (!row || !h || row > MAX_DIM * 4 || h > MAX_DIM) {
        return NULL;
    }
    buf = g_malloc(row * h);
    for (r = 0; r < h; r++) {
        dma_memory_read(&address_space_memory, base + (hwaddr)r * stride,
                        buf + r * row, row, MEMTXATTRS_UNSPECIFIED);
    }
    return buf;
}

/*
 * Channel @c of a @bpp-byte image sampled at (@x, @y) bilinearly, edges
 * repeated; false (use the pad value) a pixel or more outside
 */
static bool bs_sample(const uint8_t *img, int w, int h, int bpp, int c,
                      double x, double y, double *out)
{
    int x0 = floor(x), y0 = floor(y);
    double fx = x - x0, fy = y - y0;
    int xa, xb, ya, yb;

    if (x0 < -1 || y0 < -1 || x0 >= w || y0 >= h) {
        return false;
    }
    xa = MAX(0, MIN(x0, w - 1));
    xb = MAX(0, MIN(x0 + 1, w - 1));
    ya = MAX(0, MIN(y0, h - 1));
    yb = MAX(0, MIN(y0 + 1, h - 1));
    *out = (img[(ya * w + xa) * bpp + c] * (1 - fx) +
            img[(ya * w + xb) * bpp + c] * fx) * (1 - fy) +
           (img[(yb * w + xa) * bpp + c] * (1 - fx) +
            img[(yb * w + xb) * bpp + c] * fx) * fy;
    return true;
}

/* The source position of whole-picture destination point (@X, @Y) */
static void bs_map(const double m[9], double X, double Y, double *sx,
                   double *sy)
{
    double d = m[6] * X + m[7] * Y + m[8];

    if (fabs(d) < 1e-9) {
        d = 1e-9;
    }
    *sx = (m[0] * X + m[1] * Y + m[2]) / d;
    *sy = (m[3] * X + m[4] * Y + m[5]) / d;
}

/* --- C: one tile ------------------------------------------------------- */

static void bs_run_c_job(IngenicBscalerState *s)
{
    uint32_t fmt = REG(s, C_FMT);
    bool src4 = fmt & 1, dst4 = fmt & 8;
    uint8_t pad = fmt >> 8;
    int order = (REG(s, OFS_ORDER) >> 4) & 0xf;
    uint32_t sw = REG(s, C_SRC_SIZE) & 0xffff, sh = REG(s, C_SRC_SIZE) >> 16;
    uint32_t dw = REG(s, C_DST_SIZE) & 0xffff, dh = REG(s, C_DST_SIZE) >> 16;
    uint32_t sstride = REG(s, C_SRC_STRIDE), dstride = REG(s, C_DST_STRIDE);
    double sox = REG(s, C_SRC_ORG) & 0xffff, soy = REG(s, C_SRC_ORG) >> 16;
    double dox = REG(s, C_DST_ORG) & 0xffff, doy = REG(s, C_DST_ORG) >> 16;
    hwaddr dy_a = s->nplanes ? s->planes[0] : REG(s, C_DST_PLANE);
    hwaddr duv_a = s->planes[1];
    uint8_t *yp = NULL, *uvp = NULL, *out;
    double m[9], sx, sy, v;
    uint32_t x, y;
    int c;

    for (c = 0; c < 9; c++) {
        m[c] = (int32_t)REG(s, C_MATRIX + 4 * c) / 65536.0;
    }
    if (!sw || !sh || !dw || !dh || MAX(MAX(sw, sh), MAX(dw, dh)) > MAX_DIM ||
        (src4 && !dst4)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-bscaler: C: bad job %ux%u -> "
                      "%ux%u, format 0x%x\n", sw, sh, dw, dh, fmt);
        return;
    }
    yp = bs_read_rect(REG(s, C_SRC_Y), sstride, sw * (src4 ? 4 : 1), sh);
    if (!src4) {
        uvp = bs_read_rect(REG(s, C_SRC_UV), sstride, sw & ~1u,
                           (sh + 1) / 2);
    }
    out = g_malloc(dw * 4);
    for (y = 0; y < dh && yp; y++) {
        for (x = 0; x < dw; x++) {
            bs_map(m, x + dox, y + doy, &sx, &sy);
            sx -= sox;
            sy -= soy;
            if (src4) {
                for (c = 0; c < 4; c++) {
                    out[x * 4 + c] = bs_sample(yp, sw, sh, 4, c, sx, sy, &v)
                                     ? clamp8(v) : pad;
                }
            } else if (dst4) {
                double yv, cb = 128, cr = 128;
                double cx = (sx - 0.5) / 2, cy = (sy - 0.5) / 2;

                if (!bs_sample(yp, sw, sh, 1, 0, sx, sy, &yv)) {
                    memset(&out[x * 4], pad, 4);
                    continue;
                }
                if (uvp) {
                    bs_sample(uvp, sw / 2, (sh + 1) / 2, 2, 0, cx, cy, &cb);
                    bs_sample(uvp, sw / 2, (sh + 1) / 2, 2, 1, cx, cy, &cr);
                }
                bs_yuv_color(s, order, yv, cb, cr, &out[x * 4]);
            } else {
                out[x] = bs_sample(yp, sw, sh, 1, 0, sx, sy, &v) ? clamp8(v)
                                                                 : pad;
            }
        }
        dma_memory_write(&address_space_memory, dy_a + (hwaddr)y * dstride,
                         out, dw * (dst4 ? 4 : 1), MEMTXATTRS_UNSPECIFIED);
    }
    if (!src4 && !dst4 && uvp && s->nplanes > 1) {
        /* the CbCr plane: each chroma sample where its luma centre maps */
        for (y = 0; y < dh / 2; y++) {
            for (x = 0; x < dw / 2; x++) {
                bs_map(m, 2 * x + 0.5 + dox, 2 * y + 0.5 + doy, &sx, &sy);
                sx = (sx - sox - 0.5) / 2;
                sy = (sy - soy - 0.5) / 2;
                for (c = 0; c < 2; c++) {
                    out[x * 2 + c] = bs_sample(uvp, sw / 2, (sh + 1) / 2, 2,
                                               c, sx, sy, &v) ? clamp8(v)
                                                              : 128;
                }
            }
            dma_memory_write(&address_space_memory,
                             duv_a + (hwaddr)y * dstride, out, dw & ~1u,
                             MEMTXATTRS_UNSPECIFIED);
        }
    }
    g_free(yp);
    g_free(uvp);
    g_free(out);
}

static void bs_c_write(IngenicBscalerState *s, hwaddr off, uint32_t val);

/* Run the chain at 0x34: (value, register) pairs, 0x38 bytes of them */
static void bs_run_chain(IngenicBscalerState *s)
{
    uint32_t len = MIN(REG(s, C_CHAIN_LEN), MAX_CHAIN) & ~7u;
    hwaddr pa = REG(s, C_CHAIN);
    uint32_t i, pair[2];

    for (i = 0; i < len; i += 8) {
        dma_memory_read(&address_space_memory, pa + i, pair, 8,
                        MEMTXATTRS_UNSPECIFIED);
        bs_c_write(s, le32_to_cpu(pair[1]) & 0xff, le32_to_cpu(pair[0]));
    }
}

static void bs_c_write(IngenicBscalerState *s, hwaddr off, uint32_t val)
{
    switch (off) {
    case C_DST_PLANE:
        if (s->nplanes < 2) {
            s->planes[s->nplanes++] = val;
        }
        REG(s, off) = val;
        break;
    case C_STAT:
        REG(s, off) = val & ~CTRL_START;
        if (val & CTRL_START) {
            bs_run_c_job(s);
            s->nplanes = 0;
        }
        break;
    default:
        REG(s, off) = val;
        break;
    }
}

/* --- T: lines of NV12 to colour ---------------------------------------- */

static void bs_run_t(IngenicBscalerState *s, uint32_t lines)
{
    uint32_t w = REG(s, T_SIZE) & 0xffff, h = REG(s, T_SIZE) >> 16;
    uint32_t sstride = REG(s, T_SRC_STRIDE), dstride = REG(s, T_DST_STRIDE);
    bool src4 = REG(s, T_FMT) & 1, dst4 = REG(s, T_FMT) & 4;
    int order = REG(s, OFS_ORDER) & 0xf;
    uint32_t r0 = s->t_row, r1 = MIN(h, r0 + lines), r, x;
    uint8_t *row, *uv, *out;
    hwaddr dst = REG(s, T_DST);

    if (!w || !h || w > MAX_DIM || h > MAX_DIM) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-bscaler: T: bad size %ux%u\n",
                      w, h);
        return;
    }
    row = g_malloc(w * 4);
    uv = g_malloc(w + 2);
    out = g_malloc(w * 4);
    for (r = r0; r < r1; r++) {
        hwaddr d = dst + (hwaddr)(r - r0) * dstride;

        if (src4 || !dst4) {
            /* nothing to convert: the line as it is (NV12: both planes) */
            uint32_t n = src4 ? w * 4 : w;

            dma_memory_read(&address_space_memory,
                            REG(s, T_SRC_Y) + (hwaddr)r * sstride, row, n,
                            MEMTXATTRS_UNSPECIFIED);
            dma_memory_write(&address_space_memory, d, row, n,
                             MEMTXATTRS_UNSPECIFIED);
            continue;
        }
        dma_memory_read(&address_space_memory,
                        REG(s, T_SRC_Y) + (hwaddr)r * sstride, row, w,
                        MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory,
                        REG(s, T_SRC_UV) + (hwaddr)(r / 2) * sstride, uv,
                        w & ~1u, MEMTXATTRS_UNSPECIFIED);
        for (x = 0; x < w; x++) {
            const uint8_t *c = &uv[x & ~1u];

            bs_yuv_color(s, order, row[x], c[0], c[1], &out[x * 4]);
        }
        dma_memory_write(&address_space_memory, d, out, w * 4,
                         MEMTXATTRS_UNSPECIFIED);
    }
    if (!src4 && !dst4) {
        /* NV12 out: the CbCr lines of these rows after the Y */
        for (r = r0 / 2; r < (r1 + 1) / 2; r++) {
            dma_memory_read(&address_space_memory,
                            REG(s, T_SRC_UV) + (hwaddr)r * sstride, row, w,
                            MEMTXATTRS_UNSPECIFIED);
            dma_memory_write(&address_space_memory,
                             dst + (hwaddr)(r1 - r0 + r - r0 / 2) * dstride,
                             row, w, MEMTXATTRS_UNSPECIFIED);
        }
    }
    if (r1 >= h) {
        /* the frame is through: 0x80 bit 0 drops, bit 8 says so */
        s->t_row = 0;
        s->t_frame_done = 1;
    } else {
        s->t_row = r1;
    }
    g_free(row);
    g_free(uv);
    g_free(out);
}

/* --- registers --------------------------------------------------------- */

static void bs_done(void *opaque, int u)
{
    IngenicBscalerState *s = opaque;

    if (u == BS_C) {
        REG(s, C_CTRL) |= CTRL_DONE;
        REG(s, C_STAT) |= STAT_DONE;
    } else {
        REG(s, T_STEP) |= STEP_DONE;
        if (s->t_frame_done) {
            REG(s, T_MODE) = (REG(s, T_MODE) & ~CTRL_START) | STAT_DONE;
            s->t_frame_done = 0;
        }
    }
    s->pending[u] = 0;
    bs_update(s);
}

static void bs_done_t(void *opaque)
{
    bs_done(opaque, BS_T);
}

static void bs_done_c(void *opaque)
{
    bs_done(opaque, BS_C);
}

static void bs_kick(IngenicBscalerState *s, int u)
{
    s->pending[u] = 1;
    timer_mod(s->timer[u], qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DONE_NS);
}

static uint64_t ingenic_bscaler_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicBscalerState *s = opaque;

    return off < BS_REGS ? REG(s, off) : 0;
}

static void ingenic_bscaler_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    IngenicBscalerState *s = opaque;

    if (off >= BS_REGS) {
        return;
    }
    switch (off) {
    case C_CTRL:
        REG(s, off) = val & ~(CTRL_START | CTRL_RESET);
        if (val & CTRL_RESET) {
            s->nplanes = 0;
        }
        if (val & CTRL_START) {
            s->nplanes = 0;
            bs_run_chain(s);
            bs_kick(s, BS_C);
        }
        break;
    case C_STAT:
        bs_c_write(s, off, val);
        if (val & CTRL_START) {
            bs_kick(s, BS_C);
        }
        break;
    case T_CTRL:
        REG(s, off) = val & ~CTRL_RESET;
        if (val & CTRL_RESET) {
            s->t_row = 0;
        }
        break;
    case T_MODE:
        REG(s, off) = val;
        s->t_row = 0;
        s->t_frame_done = 0;
        break;
    case T_STEP:
        REG(s, off) = val & ~CTRL_START;
        if (val & CTRL_START) {
            bs_run_t(s, val >> 16);
            bs_kick(s, BS_T);
        }
        break;
    default:
        if (off < 0x80) {
            bs_c_write(s, off, val);
        } else {
            REG(s, off) = val;
        }
        break;
    }
    bs_update(s);
}

static const MemoryRegionOps ingenic_bscaler_ops = {
    .read = ingenic_bscaler_read,
    .write = ingenic_bscaler_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_bscaler_reset(DeviceState *dev)
{
    IngenicBscalerState *s = INGENIC_BSCALER(dev);
    int u;

    for (u = 0; u < BS_UNITS; u++) {
        timer_del(s->timer[u]);
        s->pending[u] = 0;
    }
    memset(s->regs, 0, sizeof(s->regs));
    s->nplanes = 0;
    s->t_row = 0;
    s->t_frame_done = 0;
    bs_update(s);
}

static void ingenic_bscaler_init(Object *obj)
{
    IngenicBscalerState *s = INGENIC_BSCALER(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_bscaler_ops, s,
                          TYPE_INGENIC_BSCALER, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[BS_T]);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[BS_C]);
    s->timer[BS_T] = timer_new_ns(QEMU_CLOCK_VIRTUAL, bs_done_t, s);
    s->timer[BS_C] = timer_new_ns(QEMU_CLOCK_VIRTUAL, bs_done_c, s);
}

static const VMStateDescription vmstate_ingenic_bscaler = {
    .name = TYPE_INGENIC_BSCALER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicBscalerState, BS_REGS / 4),
        VMSTATE_UINT32_ARRAY(planes, IngenicBscalerState, 2),
        VMSTATE_UINT32(nplanes, IngenicBscalerState),
        VMSTATE_UINT32(t_row, IngenicBscalerState),
        VMSTATE_UINT32(t_frame_done, IngenicBscalerState),
        VMSTATE_UINT32_ARRAY(pending, IngenicBscalerState, BS_UNITS),
        VMSTATE_TIMER_PTR_ARRAY(timer, IngenicBscalerState, BS_UNITS),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_bscaler_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_bscaler_reset);
    dc->vmsd = &vmstate_ingenic_bscaler;
}

static const TypeInfo ingenic_bscaler_info = {
    .name = TYPE_INGENIC_BSCALER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicBscalerState),
    .instance_init = ingenic_bscaler_init,
    .class_init = ingenic_bscaler_class_init,
};

static void ingenic_bscaler_register_types(void)
{
    type_register_static(&ingenic_bscaler_info);
}

type_init(ingenic_bscaler_register_types)
