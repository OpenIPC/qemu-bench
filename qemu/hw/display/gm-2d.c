/*
 * Grain Media think2d (GM2D) 2D engine.
 *
 * The 2D engine at 0x92200000 on the GM813x (module/gm2d/gm2d_driver.c,
 * module/include/gm2d/gm2d_if.h; DirectFB's gfxdrivers/think2d), which
 * draws the OSD bitmaps and privacy masks into the captured frames and
 * blits frames to the LCD.  Its driver checks the magic 0x84465203 at
 * 0xEC, then hands the engine a command list in memory: 0xF8 = 2
 * (interrupt control), 0xF0 = the list's address, 0xF4 = its length in
 * words, which starts it.  A list is pairs of words, a register offset
 * (bits 11:0; bit 31 "wait for the engine", which the model has no need
 * of) and the value to write; writing DRAW_CMD or BLIT_CMD runs that
 * operation.  The driver waits for STATUS (0xFC) to read 0 and for the
 * interrupt (4), which it acknowledges by writing 0xF8 again.
 *
 * The target:
 *   0x00 TARGET_MODE   [3:0] pixel format
 *   0x04 TARGET_BLEND  [3:0] source / [19:16] destination factor
 *                      (DirectFB's DSBF_*: 1 zero, 2 one, 5 source alpha,
 *                      6 inverse source alpha ...), bit 31 destination
 *                      colour key
 *   0x08 TARGET_BAR, 0x0C STRIDE (bytes), 0x10 RESOLXY (y << 16 | x)
 *   0x14 DST_COLORKEY, 0x18 / 0x1C CLIPMIN / CLIPMAX
 * drawing (DRAW_CMD 0x20: 1 line, 2 filled rectangle):
 *   0x24 / 0x28 start / end, inclusive; 0x2C colour, R << 24 | G << 16 |
 *   B << 8 | A
 * blitting (BLIT_CMD 0x30: [3:0] source format, [22:20] rotation -- 0, 90,
 * 180, 270, mirror x, mirror y -- [26:23] scaling, 27 source colour key,
 * 28 the foreground colour's alpha as a constant alpha):
 *   0x34 SRCADDR, 0x38 SRCRESOL, 0x3C SRCSTRIDE, 0x40 SRCCOLORKEY,
 *   0x44 FG_COLOR (A8 / BW1 sources), 0x48 DSTADDR (the destination x, y),
 *   0x4C DSTYXSIZE (the scaled size)
 *
 * Formats: 0 RGBX8888, 1 RGBA8888, 2 XRGB8888, 3 ARGB8888, 4 RGB565,
 * 5 RGBA5551, 6 RGBA4444, 7 / 8 A8, 9 L8, 0xA RGB332, 0xB LUT8 (as L8),
 * 0xC BW1, 0xD UYVY.  The GM8136's OSD blits RGB565 bitmaps, keyed on
 * their transparent colour, into the UYVY capture frames; its privacy
 * masks fill rectangles of the frames as RGB565, the colour already
 * packed as a U Y / V Y pair; the LCD path blits UYVY frames to RGB565.
 * UYVY is converted with BT.601 limited-range coefficients, each target
 * pixel writing its own luma and every other chroma sample.  Scaling is
 * nearest-neighbour.  A list completes at once.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_GM_2D "gm-2d"
OBJECT_DECLARE_SIMPLE_TYPE(Gm2dState, GM_2D)

#define GM_2D_SIZE          0x1000
#define GM_2D_NREGS         (0x200 / 4)
#define GM_2D_MAGIC         0x84465203
#define GM_2D_MAX_DIM       4096
#define GM_2D_MAX_WORDS     (64 * 1024)

#define R_TARGET_MODE       0x00
#define R_TARGET_BLEND      0x04
#define R_TARGET_BAR        0x08
#define R_TARGET_STRIDE     0x0C
#define R_TARGET_RESOLXY    0x10
#define R_DST_COLORKEY      0x14
#define R_CLIPMIN           0x18
#define R_CLIPMAX           0x1C
#define R_DRAW_CMD          0x20
#define R_DRAW_STARTXY      0x24
#define R_DRAW_ENDXY        0x28
#define R_DRAW_COLOR        0x2C
#define R_BLIT_CMD          0x30
#define R_BLIT_SRCADDR      0x34
#define R_BLIT_SRCRESOL     0x38
#define R_BLIT_SRCSTRIDE    0x3C
#define R_BLIT_SRCCOLORKEY  0x40
#define R_BLIT_FG_COLOR     0x44
#define R_BLIT_DSTADDR      0x48
#define R_BLIT_DSTYXSIZE    0x4C
#define R_MAGIC             0xEC
#define R_CMDADDR           0xF0
#define R_CMDSIZE           0xF4
#define R_INTERRUPT         0xF8
#define R_STATUS            0xFC

#define CMDLIST_REG         0xFFF

enum {
    F_RGBX8888, F_RGBA8888, F_XRGB8888, F_ARGB8888, F_RGB565, F_RGBA5551,
    F_RGBA4444, F_RGBA0800, F_A8, F_L8, F_RGB332, F_LUT8, F_BW1, F_UYVY,
};

/* DirectFB's blend factors */
enum {
    BF_ZERO = 1, BF_ONE, BF_SRCCOLOR, BF_INVSRCCOLOR, BF_SRCALPHA,
    BF_INVSRCALPHA, BF_DESTALPHA, BF_INVDESTALPHA, BF_DESTCOLOR,
    BF_INVDESTCOLOR, BF_SRCALPHASAT,
};

struct Gm2dState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[GM_2D_NREGS];
    bool irq_pending;
};

/* ---- pixels ---------------------------------------------------------- */

typedef struct Rgba {
    uint8_t r, g, b, a;
} Rgba;

static unsigned gm_2d_bpp(unsigned fmt)
{
    switch (fmt) {
    case F_RGBX8888: case F_RGBA8888: case F_XRGB8888: case F_ARGB8888:
        return 4;
    case F_RGB565: case F_RGBA5551: case F_RGBA4444: case F_UYVY:
        return 2;
    default:
        return 1;                       /* BW1 is handled bitwise */
    }
}

static size_t gm_2d_row_bytes(unsigned fmt, uint32_t w)
{
    return fmt == F_BW1 ? (w + 7) / 8 : (size_t)w * gm_2d_bpp(fmt);
}

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

static Rgba yuv_to_rgb(int y, int u, int v)
{
    const int c = 298 * (y - 16), d = u - 128, e = v - 128;

    return (Rgba) {
        clamp8((c + 409 * e + 128) >> 8),
        clamp8((c - 100 * d - 208 * e + 128) >> 8),
        clamp8((c + 516 * d + 128) >> 8),
        255,
    };
}

static uint8_t rgb_y(Rgba p)
{
    return ((66 * p.r + 129 * p.g + 25 * p.b + 128) >> 8) + 16;
}

static uint8_t rgb_u(Rgba p)
{
    return ((-38 * p.r - 74 * p.g + 112 * p.b + 128) >> 8) + 128;
}

static uint8_t rgb_v(Rgba p)
{
    return ((112 * p.r - 94 * p.g - 18 * p.b + 128) >> 8) + 128;
}

/* the raw value of pixel x of a row, for colour keys */
static uint32_t gm_2d_raw(unsigned fmt, const uint8_t *row, uint32_t x)
{
    switch (fmt) {
    case F_BW1:
        return (row[x / 8] >> (7 - x % 8)) & 1;
    case F_UYVY:
        return lduw_le_p(row + x * 2);
    }
    switch (gm_2d_bpp(fmt)) {
    case 4:
        return ldl_le_p(row + x * 4);
    case 2:
        return lduw_le_p(row + x * 2);
    }
    return row[x];
}

static Rgba gm_2d_get(unsigned fmt, const uint8_t *row, uint32_t x,
                      uint32_t fg)
{
    const uint32_t v = gm_2d_raw(fmt, row, x);
    const Rgba fgc = { fg >> 24, fg >> 16, fg >> 8, fg };

    switch (fmt) {
    case F_RGBX8888:
        return (Rgba) { v >> 24, v >> 16, v >> 8, 255 };
    case F_RGBA8888:
        return (Rgba) { v >> 24, v >> 16, v >> 8, v };
    case F_XRGB8888:
        return (Rgba) { v >> 16, v >> 8, v, 255 };
    case F_ARGB8888:
        return (Rgba) { v >> 16, v >> 8, v, v >> 24 };
    case F_RGB565:
        return (Rgba) { (v >> 8 & 0xF8) | v >> 13,
                        (v >> 3 & 0xFC) | (v >> 9 & 3),
                        (v << 3 & 0xF8) | (v >> 2 & 7), 255 };
    case F_RGBA5551:
        return (Rgba) { (v >> 8 & 0xF8) | v >> 13,
                        (v >> 3 & 0xF8) | (v >> 8 & 7),
                        (v << 2 & 0xF8) | (v >> 3 & 7), (v & 1) ? 255 : 0 };
    case F_RGBA4444:
        return (Rgba) { (v >> 12) * 17, (v >> 8 & 0xF) * 17,
                        (v >> 4 & 0xF) * 17, (v & 0xF) * 17 };
    case F_RGBA0800:
    case F_A8:
        return (Rgba) { fgc.r, fgc.g, fgc.b, v };
    case F_L8:
    case F_LUT8:
        return (Rgba) { v, v, v, 255 };
    case F_RGB332:
        return (Rgba) { (v >> 5) * 255 / 7, (v >> 2 & 7) * 255 / 7,
                        (v & 3) * 85, 255 };
    case F_BW1:
        return (Rgba) { fgc.r, fgc.g, fgc.b, v ? 255 : 0 };
    case F_UYVY: {
        const uint8_t *pair = row + (x & ~1u) * 2;

        return yuv_to_rgb(pair[(x & 1) ? 3 : 1], pair[0], pair[2]);
    }
    }
    return (Rgba) { 0, 0, 0, 255 };
}

static void gm_2d_put(unsigned fmt, uint8_t *row, uint32_t x, Rgba p)
{
    switch (fmt) {
    case F_RGBX8888:
    case F_RGBA8888:
        stl_le_p(row + x * 4, p.r << 24 | p.g << 16 | p.b << 8 |
                              (fmt == F_RGBX8888 ? 0xFF : p.a));
        break;
    case F_XRGB8888:
    case F_ARGB8888:
        stl_le_p(row + x * 4, (uint32_t)(fmt == F_XRGB8888 ? 0xFF : p.a) << 24 |
                              p.r << 16 | p.g << 8 | p.b);
        break;
    case F_RGB565:
        stw_le_p(row + x * 2, (p.r >> 3) << 11 | (p.g >> 2) << 5 | p.b >> 3);
        break;
    case F_RGBA5551:
        stw_le_p(row + x * 2, (p.r >> 3) << 11 | (p.g >> 3) << 6 |
                              (p.b >> 3) << 1 | (p.a >= 128));
        break;
    case F_RGBA4444:
        stw_le_p(row + x * 2, (p.r >> 4) << 12 | (p.g >> 4) << 8 |
                              (p.b >> 4) << 4 | p.a >> 4);
        break;
    case F_RGBA0800:
    case F_A8:
        row[x] = p.a;
        break;
    case F_L8:
    case F_LUT8:
        row[x] = rgb_y(p);
        break;
    case F_RGB332:
        row[x] = (p.r >> 5) << 5 | (p.g >> 5) << 2 | p.b >> 6;
        break;
    case F_BW1:
        row[x / 8] = deposit32(row[x / 8], 7 - x % 8, 1, rgb_y(p) >= 128);
        break;
    case F_UYVY:
        row[x * 2] = (x & 1) ? rgb_v(p) : rgb_u(p);
        row[x * 2 + 1] = rgb_y(p);
        break;
    }
}

static unsigned gm_2d_factor(unsigned f, Rgba s, Rgba d, int ch)
{
    const uint8_t sc[3] = { s.r, s.g, s.b }, dc[3] = { d.r, d.g, d.b };

    switch (f) {
    case BF_ZERO:
        return 0;
    case BF_ONE:
        return 255;
    case BF_SRCCOLOR:
        return ch < 3 ? sc[ch] : s.a;
    case BF_INVSRCCOLOR:
        return 255 - (ch < 3 ? sc[ch] : s.a);
    case BF_SRCALPHA:
        return s.a;
    case BF_INVSRCALPHA:
        return 255 - s.a;
    case BF_DESTALPHA:
        return d.a;
    case BF_INVDESTALPHA:
        return 255 - d.a;
    case BF_DESTCOLOR:
        return ch < 3 ? dc[ch] : d.a;
    case BF_INVDESTCOLOR:
        return 255 - (ch < 3 ? dc[ch] : d.a);
    case BF_SRCALPHASAT:
        return ch < 3 ? MIN(s.a, 255 - d.a) : 255;
    }
    return 0;
}

/* ---- the target ------------------------------------------------------ */

typedef struct Gm2dTarget {
    unsigned fmt;
    hwaddr bar;
    uint32_t stride;
    int x0, y0, x1, y1;     /* the drawable box, inclusive */
    unsigned src_bf, dst_bf;
    bool dst_key;
    uint32_t dst_keyval;
} Gm2dTarget;

static bool gm_2d_target(Gm2dState *s, Gm2dTarget *t)
{
    const uint32_t res = s->regs[R_TARGET_RESOLXY / 4];
    const uint32_t cmin = s->regs[R_CLIPMIN / 4];
    const uint32_t cmax = s->regs[R_CLIPMAX / 4];
    const uint32_t blend = s->regs[R_TARGET_BLEND / 4];
    const int w = MIN(extract32(res, 0, 16), GM_2D_MAX_DIM);
    const int h = MIN(extract32(res, 16, 16), GM_2D_MAX_DIM);

    t->fmt = s->regs[R_TARGET_MODE / 4] & 0xF;
    t->bar = s->regs[R_TARGET_BAR / 4];
    t->stride = s->regs[R_TARGET_STRIDE / 4] & 0xFFFF;
    t->x0 = extract32(cmin, 0, 16);
    t->y0 = extract32(cmin, 16, 16);
    t->x1 = MIN((int)extract32(cmax, 0, 16), w - 1);
    t->y1 = MIN((int)extract32(cmax, 16, 16), h - 1);
    t->src_bf = extract32(blend, 0, 4) ?: BF_ONE;     /* unset: a copy */
    t->dst_bf = extract32(blend, 16, 4) ?: BF_ZERO;
    t->dst_key = blend & (1u << 31);
    t->dst_keyval = s->regs[R_DST_COLORKEY / 4];
    if (t->fmt > F_UYVY || !t->stride || t->x0 > t->x1 || t->y0 > t->y1) {
        return false;
    }
    if (gm_2d_row_bytes(t->fmt, t->x1 + 1) > t->stride) {
        t->x1 = (t->fmt == F_BW1 ? t->stride * 8
                                 : t->stride / gm_2d_bpp(t->fmt)) - 1;
    }
    return t->x0 <= t->x1;
}

static bool gm_2d_copy_blend(const Gm2dTarget *t)
{
    return t->src_bf == BF_ONE && t->dst_bf == BF_ZERO;
}

/*
 * Write src[0..n) (alpha 0 = skip, for keyed / transparent pixels with
 * mask[] false) at x..x+n of target row y.
 */
static void gm_2d_span(const Gm2dTarget *t, int y, int x, int n,
                       const Rgba *src, const bool *mask)
{
    const size_t row_len = t->stride;
    g_autofree uint8_t *row = NULL;
    const hwaddr addr = t->bar + (hwaddr)y * t->stride;
    const bool copy = gm_2d_copy_blend(t);
    int i;

    if (y < t->y0 || y > t->y1) {
        return;
    }
    row = g_malloc(row_len);
    dma_memory_read(&address_space_memory, addr, row, row_len,
                    MEMTXATTRS_UNSPECIFIED);
    for (i = 0; i < n; i++) {
        const int xx = x + i;
        Rgba out = src[i];

        if (xx < t->x0 || xx > t->x1 || (mask && !mask[i])) {
            continue;
        }
        if (t->dst_key && gm_2d_raw(t->fmt, row, xx) != t->dst_keyval) {
            continue;
        }
        if (!copy) {
            const Rgba d = gm_2d_get(t->fmt, row, xx, 0);
            const uint8_t sv[4] = { src[i].r, src[i].g, src[i].b, src[i].a };
            const uint8_t dv[4] = { d.r, d.g, d.b, d.a };
            uint8_t ov[4];
            int ch;

            for (ch = 0; ch < 4; ch++) {
                ov[ch] = MIN((sv[ch] * gm_2d_factor(t->src_bf, src[i], d, ch) +
                              dv[ch] * gm_2d_factor(t->dst_bf, src[i], d, ch) +
                              127) / 255, 255);
            }
            out = (Rgba) { ov[0], ov[1], ov[2], ov[3] };
        }
        gm_2d_put(t->fmt, row, xx, out);
    }
    dma_memory_write(&address_space_memory, addr, row, row_len,
                     MEMTXATTRS_UNSPECIFIED);
}

/* ---- operations ------------------------------------------------------ */

static void gm_2d_draw(Gm2dState *s, uint32_t cmd)
{
    const uint32_t c = s->regs[R_DRAW_COLOR / 4];
    const uint32_t st = s->regs[R_DRAW_STARTXY / 4];
    const uint32_t en = s->regs[R_DRAW_ENDXY / 4];
    int x0 = (int16_t)extract32(st, 0, 16), y0 = (int16_t)extract32(st, 16, 16);
    int x1 = (int16_t)extract32(en, 0, 16), y1 = (int16_t)extract32(en, 16, 16);
    const Rgba col = { c >> 24, c >> 16, c >> 8, c };
    Gm2dTarget t;

    if (!gm_2d_target(s, &t)) {
        return;
    }
    if (cmd == 2) {                         /* filled rectangle */
        g_autofree Rgba *span = NULL;
        int y, n, i;

        if (x0 > x1) {
            int tmp = x0;
            x0 = x1;
            x1 = tmp;
        }
        if (y0 > y1) {
            int tmp = y0;
            y0 = y1;
            y1 = tmp;
        }
        x0 = MAX(x0, t.x0);
        x1 = MIN(x1, t.x1);
        y0 = MAX(y0, t.y0);
        y1 = MIN(y1, t.y1);
        if (x0 > x1 || y0 > y1) {
            return;
        }
        n = x1 - x0 + 1;
        span = g_new(Rgba, n);
        for (i = 0; i < n; i++) {
            span[i] = col;
        }
        for (y = y0; y <= y1; y++) {
            gm_2d_span(&t, y, x0, n, span, NULL);
        }
    } else if (cmd == 1) {                  /* line, Bresenham */
        const int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        const int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, steps = 0;

        while (steps++ < 2 * GM_2D_MAX_DIM) {
            int e2;

            gm_2d_span(&t, y0, x0, 1, &col, NULL);
            if (x0 == x1 && y0 == y1) {
                break;
            }
            e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "gm-2d: draw command %u\n", cmd);
    }
}

static void gm_2d_blit(Gm2dState *s, uint32_t cmd)
{
    const unsigned sfmt = cmd & 0xF;
    const unsigned rot = extract32(cmd, 20, 3);
    const bool scale = extract32(cmd, 23, 4) != 0;
    const bool src_key = cmd & (1u << 27);
    const bool const_alpha = cmd & (1u << 28);
    const uint32_t sres = s->regs[R_BLIT_SRCRESOL / 4];
    const uint32_t dxy = s->regs[R_BLIT_DSTADDR / 4];
    const uint32_t dsz = s->regs[R_BLIT_DSTYXSIZE / 4];
    const uint32_t sstride = s->regs[R_BLIT_SRCSTRIDE / 4] & 0xFFFF;
    const uint32_t keyval = s->regs[R_BLIT_SRCCOLORKEY / 4];
    const uint32_t fg = s->regs[R_BLIT_FG_COLOR / 4];
    const hwaddr saddr = s->regs[R_BLIT_SRCADDR / 4];
    const uint32_t sw = MIN(extract32(sres, 0, 16), GM_2D_MAX_DIM);
    const uint32_t sh = MIN(extract32(sres, 16, 16), GM_2D_MAX_DIM);
    const int dx = (int16_t)extract32(dxy, 0, 16);
    const int dy = (int16_t)extract32(dxy, 16, 16);
    const bool swap = rot == 1 || rot == 3;
    uint32_t ow, oh, x, y;
    g_autofree uint8_t *src = NULL;
    g_autofree Rgba *span = NULL;
    g_autofree bool *mask = NULL;
    Gm2dTarget t;

    if (!gm_2d_target(s, &t) || sfmt > F_UYVY || !sw || !sh || !sstride ||
        gm_2d_row_bytes(sfmt, sw) > sstride) {
        return;
    }
    /* the output size: the scaled size, else the (rotated) source's */
    if (scale && extract32(dsz, 0, 16) && extract32(dsz, 16, 16)) {
        ow = MIN(extract32(dsz, 0, 16), GM_2D_MAX_DIM);
        oh = MIN(extract32(dsz, 16, 16), GM_2D_MAX_DIM);
    } else {
        ow = swap ? sh : sw;
        oh = swap ? sw : sh;
    }
    src = g_malloc0((size_t)sstride * sh + 4);  /* an odd UYVY width */
    dma_memory_read(&address_space_memory, saddr, src, (size_t)sstride * sh,
                    MEMTXATTRS_UNSPECIFIED);
    span = g_new(Rgba, ow);
    mask = g_new(bool, ow);
    for (y = 0; y < oh; y++) {
        bool any = false;

        if (dy + (int)y < t.y0 || dy + (int)y > t.y1) {
            continue;
        }
        for (x = 0; x < ow; x++) {
            /* the output pixel in unrotated, unscaled source space */
            const uint32_t rw = swap ? sh : sw, rh = swap ? sw : sh;
            const uint32_t ux = (uint64_t)x * rw / ow;
            const uint32_t uy = (uint64_t)y * rh / oh;
            uint32_t sx, sy;
            const uint8_t *row;

            switch (rot) {
            case 1:                             /* 90 */
                sx = uy;
                sy = sh - 1 - ux;
                break;
            case 2:                             /* 180 */
                sx = sw - 1 - ux;
                sy = sh - 1 - uy;
                break;
            case 3:                             /* 270 */
                sx = sw - 1 - uy;
                sy = ux;
                break;
            case 4:                             /* mirror x */
                sx = sw - 1 - ux;
                sy = uy;
                break;
            case 5:                             /* mirror y */
                sx = ux;
                sy = sh - 1 - uy;
                break;
            default:
                sx = ux;
                sy = uy;
                break;
            }
            row = src + (size_t)sy * sstride;
            span[x] = gm_2d_get(sfmt, row, sx, fg);
            if (const_alpha) {
                span[x].a = span[x].a * (fg & 0xFF) / 255;
            }
            /* a UYVY source has no key: the LCD path's frames */
            mask[x] = !(src_key && sfmt != F_UYVY &&
                        gm_2d_raw(sfmt, row, sx) == keyval) &&
                      !((sfmt == F_A8 || sfmt == F_BW1 || sfmt == F_RGBA0800) &&
                        gm_2d_copy_blend(&t) && !span[x].a);
            any |= mask[x];
        }
        if (any) {
            gm_2d_span(&t, dy + y, dx, ow, span, mask);
        }
    }
}

/* ---- registers and lists --------------------------------------------- */

static void gm_2d_reg(Gm2dState *s, uint32_t off, uint32_t val)
{
    off &= CMDLIST_REG & ~3u;
    if (off >= GM_2D_NREGS * 4) {
        return;
    }
    s->regs[off / 4] = val;
    switch (off) {
    case R_DRAW_CMD:
        gm_2d_draw(s, val);
        break;
    case R_BLIT_CMD:
        gm_2d_blit(s, val);
        break;
    }
}

static void gm_2d_run_list(Gm2dState *s)
{
    const hwaddr addr = s->regs[R_CMDADDR / 4];
    const uint32_t words = MIN(s->regs[R_CMDSIZE / 4], GM_2D_MAX_WORDS) & ~1u;
    g_autofree uint32_t *list = g_new(uint32_t, words + 2);
    uint32_t i;

    dma_memory_read(&address_space_memory, addr, list, words * 4,
                    MEMTXATTRS_UNSPECIFIED);
    for (i = 0; i < words; i += 2) {
        const uint32_t off = le32_to_cpu(list[i]) & CMDLIST_REG;

        if (off == R_CMDADDR || off == R_CMDSIZE || off == R_INTERRUPT ||
            off == R_STATUS) {
            continue;
        }
        gm_2d_reg(s, off, le32_to_cpu(list[i + 1]));
    }
    s->irq_pending = true;
}

static uint64_t gm_2d_read(void *opaque, hwaddr off, unsigned size)
{
    Gm2dState *s = opaque;

    switch (off) {
    case R_MAGIC:
        return GM_2D_MAGIC;
    case R_STATUS:
        return 0;
    }
    return off < GM_2D_NREGS * 4 ? s->regs[off / 4] : 0;
}

static void gm_2d_write(void *opaque, hwaddr off, uint64_t val,
                        unsigned size)
{
    Gm2dState *s = opaque;

    if (off >= GM_2D_NREGS * 4) {
        return;
    }
    switch (off) {
    case R_CMDSIZE:
        s->regs[off / 4] = val;
        if (val) {
            gm_2d_run_list(s);
        }
        break;
    case R_INTERRUPT:
        s->regs[off / 4] = val;
        s->irq_pending = false;         /* acknowledge */
        break;
    case R_CMDADDR:
    case R_STATUS:
        s->regs[off / 4] = val;
        break;
    default:
        gm_2d_reg(s, off, val);
        break;
    }
    qemu_set_irq(s->irq, s->irq_pending);
}

static const MemoryRegionOps gm_2d_ops = {
    .read = gm_2d_read,
    .write = gm_2d_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_2d_reset(DeviceState *dev)
{
    Gm2dState *s = GM_2D(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->irq_pending = false;
    qemu_set_irq(s->irq, 0);
}

static void gm_2d_realize(DeviceState *dev, Error **errp)
{
    Gm2dState *s = GM_2D(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_2d_ops, s, TYPE_GM_2D,
                          GM_2D_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static const VMStateDescription vmstate_gm_2d = {
    .name = TYPE_GM_2D,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Gm2dState, GM_2D_NREGS),
        VMSTATE_BOOL(irq_pending, Gm2dState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_2d_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_2d_realize;
    device_class_set_legacy_reset(dc, gm_2d_reset);
    dc->vmsd = &vmstate_gm_2d;
}

static const TypeInfo gm_2d_info = {
    .name           = TYPE_GM_2D,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Gm2dState),
    .class_init     = gm_2d_class_init,
};

static void gm_2d_register_types(void)
{
    type_register_static(&gm_2d_info);
}

type_init(gm_2d_register_types)
