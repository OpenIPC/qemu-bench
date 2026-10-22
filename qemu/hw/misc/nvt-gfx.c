/*
 * Novatek NA51089 2D graphics engine and ISE scaler: the work of a job.
 *
 * nvt-media runs the engines' linked lists and handshakes; when a job is
 * set up it calls here with the engine's registers, and the job's pixels
 * are read from and written to guest memory a line at a time.
 *
 * Graphics engine (kdrv_gfx2d/kdrv_grph/include/graphic_reg.h, with the
 * encodings kdrv_grph.h and the closed grph.o use):
 *
 *   0x04  CONFIG  b0 AOP (1) or GOP (0); [6:4] precision (1 8-bit,
 *                 2 16-bit, 3 32-bit); [10:8] 16-bit format (1 UV pairs,
 *                 2 RGB565, 3 ARGB1555, 4 ARGB4444); b11 ARGB8888;
 *                 [13:12] U only / V only; [19:16] GOP; [28:24] AOP
 *   0x20 / 0x24 / 0x28   IN1 (A) / IN2 (B) / OUT (C) operators
 *   0x60 / 0x64 / 0x68   constants
 *   0x70 / 0x74, 0x80 / 0x84, 0x90 / 0x94   images A, B, C: address and
 *                 line offset
 *   0x78 / 0x7C   lines and bytes a line, for all three
 *
 * ISE (kdrv_gfx2d/kdrv_ise/eng/include/ise_eng_int_reg.h):
 *
 *   0x04  [3:0] format (0 Y8, 3 U/V pairs, 5 ARGB8888); [9:8] method
 *         (1 bilinear, 2 nearest, 3 integration)
 *   0x10 / 0x18 / 0x20   input size (w | h << 16), line offset, address
 *   0x38 / 0x90 / 0x98   output size, line offset, address
 *
 * The register interface is the vendor's; the arithmetic where the
 * driver does not pin it down (blending, rounding, the integration
 * filter) is ours.  docs/novatek.md lists those choices.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/misc/nvt-gfx.h"

#define R(off)      (regs[(off) / 4])

/* Read guest memory; a failed read ends the job, as a bus error would. */
static bool rd(hwaddr a, void *buf, size_t n)
{
    if (dma_memory_read(&address_space_memory, a, buf, n,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-gfx: cannot read 0x%" HWADDR_PRIx
                      "\n", a);
        return false;
    }
    return true;
}

static void wr(hwaddr a, const void *buf, size_t n)
{
    dma_memory_write(&address_space_memory, a, buf, n,
                     MEMTXATTRS_UNSPECIFIED);
}

/* ---- graphics engine ---- */

#define GRPH_CONFIG     0x04
#define GRPH_IN2_OP     0x24
#define GRPH_OUT_OP     0x28
#define GRPH_CONST1     0x60
#define GRPH_CONST2     0x64
#define GRPH_A_ADDR     0x70
#define GRPH_A_LOFS     0x74
#define GRPH_HEIGHT     0x78
#define GRPH_WIDTH      0x7C
#define GRPH_B_ADDR     0x80
#define GRPH_B_LOFS     0x84
#define GRPH_C_ADDR     0x90
#define GRPH_C_LOFS     0x94

enum {
    AOP_A_COPY = 0x00, AOP_PLUS_SHF = 0x01, AOP_MINUS_SHF = 0x02,
    AOP_TEXT_COPY = 0x08, AOP_MULTIPLY_DIV = 0x0F, AOP_PACKING = 0x10,
    AOP_RGBYUV_BLEND = 0x18,
};

enum {
    GOP_ROT_180 = 2, GOP_HRZ_FLIP = 3, GOP_VTC_FLIP = 4,
};

/* jobs are done at once, in the device's timer: bound them */
#define GRPH_MAX_JOB    (64 * MiB)

typedef struct {
    hwaddr a, b, c;
    uint32_t alofs, blofs, clofs;
    uint32_t w, h;              /* bytes a line, lines */
} GrphJob;

static uint32_t clamp_u(int64_t v, uint32_t max)
{
    return v < 0 ? 0 : v > max ? max : v;
}

/*
 * PLUS_SHF, MINUS_SHF and MULTIPLY_DIV.  8-bit: B's shift is its IN2
 * operator 2, (x >> s) + c; results saturate, and an OUT operator 4 takes
 * MINUS's magnitude.  16-bit: the shift (MULTIPLY's divisor) is CONST1
 * [3:0] and its bit 31 asks for the magnitude; sums wrap.
 */
static void grph_arith(const uint32_t *regs, const GrphJob *j, unsigned aop,
                       bool wide)
{
    g_autofree uint8_t *a = g_malloc(j->w), *b = g_malloc(j->w);
    uint32_t c1 = R(GRPH_CONST1), in2 = R(GRPH_IN2_OP);
    unsigned shf = wide || aop == AOP_MULTIPLY_DIV ? c1 & 0xF :
                   (in2 & 7) == 2 ? (in2 >> 8) & 0xF : 0;
    bool mag = wide ? c1 >> 31 : (R(GRPH_OUT_OP) & 7) == 4;
    uint32_t max = wide ? 0xFFFF : 0xFF, n = wide ? j->w / 2 : j->w, y, i;

    for (y = 0; y < j->h; y++) {
        if (!rd(j->a + (hwaddr)y * j->alofs, a, j->w)) {
            return;
        }
        if (!rd(j->b + (hwaddr)y * j->blofs, b, j->w)) {
            return;
        }
        for (i = 0; i < n; i++) {
            int64_t x = wide ? lduw_le_p(a + 2 * i) : a[i];
            int64_t z = wide ? lduw_le_p(b + 2 * i) : b[i];
            int64_t v;

            if (aop == AOP_MULTIPLY_DIV) {
                v = (x * z + (shf ? 1 << (shf - 1) : 0)) >> shf;
                v = clamp_u(v, max);
            } else {
                v = aop == AOP_PLUS_SHF ? x + (z >> shf) : x - (z >> shf);
                if (mag && v < 0) {
                    v = -v;
                }
                v = wide ? v & max : clamp_u(v, max);
            }
            if (wide) {
                stw_le_p(a + 2 * i, v);
            } else {
                a[i] = v;
            }
        }
        wr(j->c + (hwaddr)y * j->clofs, a, j->w);
    }
}

/*
 * TEXT_COPY: fill C with CONST1.  8-bit: byte k of it goes where the
 * address is k mod 4, which is how the driver lays a 2-pixel pattern
 * (an NV12 U / V pair) over a 4-byte word.  16-bit: its low half; U / V
 * pairs: U from CONST1, V from CONST2, or only one of them.
 */
static void grph_fill(const uint32_t *regs, const GrphJob *j, uint32_t cfg,
                      unsigned prec)
{
    g_autofree uint8_t *row = g_malloc(j->w);
    uint32_t c1 = R(GRPH_CONST1), c2 = R(GRPH_CONST2), y, i;
    unsigned fmt = (cfg >> 8) & 7, only = (cfg >> 12) & 3;

    for (y = 0; y < j->h; y++) {
        hwaddr at = j->c + (hwaddr)y * j->clofs;

        if (prec == 1) {
            for (i = 0; i < j->w; i++) {
                row[i] = c1 >> (8 * ((at + i) & 3));
            }
        } else if (fmt == 1) {
            /* an odd last byte, and the half a U / V-only fill skips, stay */
            if (!rd(at, row, j->w)) {
                return;
            }
            for (i = 0; i + 1 < j->w; i += 2) {
                if (only != 2) {
                    row[i] = c1;
                }
                if (only != 1) {
                    row[i + 1] = c2;
                }
            }
        } else {
            if ((j->w & 1) && !rd(at + j->w - 1, row + j->w - 1, 1)) {
                return;
            }
            for (i = 0; i + 1 < j->w; i += 2) {
                stw_le_p(row + i, c1);
            }
        }
        wr(at, row, j->w);
    }
}

/* PACKING: U from A and V from B interleave into C, W bytes of it. */
static void grph_pack(const GrphJob *j)
{
    uint32_t half = j->w / 2, y, i;
    g_autofree uint8_t *u = g_malloc(half + 1), *v = g_malloc(half + 1);
    g_autofree uint8_t *c = g_malloc(j->w);

    for (y = 0; y < j->h; y++) {
        if (!rd(j->a + (hwaddr)y * j->alofs, u, half)) {
            return;
        }
        if (!rd(j->b + (hwaddr)y * j->blofs, v, half)) {
            return;
        }
        for (i = 0; i < half; i++) {
            c[2 * i] = u[i];
            c[2 * i + 1] = v[i];
        }
        wr(j->c + (hwaddr)y * j->clofs, c, half * 2);
    }
}

/*
 * RGB to YCbCr as the vendor's own software does it (gximage's
 * argb_to_ycbcr), so a colour the driver converts itself for a fill and
 * the same colour blended by the engine come out alike.
 */
static void rgb_to_yuv(int r, int g, int b, int *y, int *u, int *v)
{
    *y = clamp_u(g + ((79 * (r - g)) >> 8) + ((29 * (b - g)) >> 8), 255);
    *u = clamp_u(128 + ((b - g) >> 1) + ((43 * (g - r)) >> 8), 255);
    *v = clamp_u(128 + ((r - g) >> 1) + ((21 * (g - b)) >> 8), 255);
}

typedef struct {
    int y, u, v, a;
} Yuva;

/* One ARGB pixel of the source format, alpha scaled to 0..255. */
static Yuva argb_pixel(const uint8_t *p, uint32_t cfg, uint32_t c1)
{
    unsigned fmt = (cfg >> 8) & 7;
    int r, g, b, a;
    Yuva o;

    if (cfg & (1u << 11)) {                 /* ARGB8888 */
        uint32_t x = ldl_le_p(p);

        a = x >> 24;
        r = (x >> 16) & 0xFF;
        g = (x >> 8) & 0xFF;
        b = x & 0xFF;
    } else {
        uint16_t x = lduw_le_p(p);

        if (fmt == 4) {                     /* ARGB4444 */
            a = (x >> 12) * 17;
            r = ((x >> 8) & 0xF) * 17;
            g = ((x >> 4) & 0xF) * 17;
            b = (x & 0xF) * 17;
        } else if (fmt == 3) {              /* ARGB1555: alpha by index */
            a = ((c1 >> (x & 0x8000 ? 12 : 8)) & 0xF) * 17;
            r = ((x >> 10) & 0x1F) * 255 / 31;
            g = ((x >> 5) & 0x1F) * 255 / 31;
            b = (x & 0x1F) * 255 / 31;
        } else {                            /* RGB565: constant alpha */
            a = (c1 >> 8) & 0xFF;
            r = ((x >> 11) & 0x1F) * 255 / 31;
            g = ((x >> 5) & 0x3F) * 255 / 63;
            b = (x & 0x1F) * 255 / 31;
        }
    }
    rgb_to_yuv(r, g, b, &o.y, &o.u, &o.v);
    o.a = a;
    return o;
}

static uint8_t blend(int s, int d, int a)
{
    return (s * a + d * (255 - a) + 127) / 255;
}

/*
 * RGBYUV_BLEND: ARGB image A over the YUV picture whose Y plane is B and
 * whose U / V pairs are C.  CONST1 bit 0: the pairs are 4:2:0, one for
 * each 2x2 pixels (otherwise one for each two of a line); bit 5: a pair
 * takes the pixels' average (otherwise the top left one's), their colours
 * weighted by alpha, so a transparent pixel adds none of its colour.
 */
static void grph_rgbyuv_blend(const uint32_t *regs, const GrphJob *j,
                              uint32_t cfg)
{
    uint32_t c1 = R(GRPH_CONST1);
    unsigned bpp = cfg & (1u << 11) ? 4 : 2;
    uint32_t n = j->w / bpp, rows = c1 & 1 ? 2 : 1, y, x, k;
    bool avg = c1 & (1u << 5);
    g_autofree uint8_t *src = g_malloc(j->w * 2);
    g_autofree uint8_t *yl = g_malloc(n + 1), *uv = g_malloc(n + 1);
    g_autofree Yuva *px = g_new(Yuva, 2 * (n + 1));

    for (y = 0; y < j->h; y += rows) {
        uint32_t got = MIN(rows, j->h - y);

        for (k = 0; k < got; k++) {
            hwaddr yat = j->b + (hwaddr)(y + k) * j->blofs;

            if (!rd(j->a + (hwaddr)(y + k) * j->alofs, src, j->w)) {
                return;
            }
            if (!rd(yat, yl, n)) {
                return;
            }
            for (x = 0; x < n; x++) {
                px[k * n + x] = argb_pixel(src + x * bpp, cfg, c1);
                yl[x] = blend(px[k * n + x].y, yl[x], px[k * n + x].a);
            }
            wr(yat, yl, n);
        }
        /* the U / V pairs under these lines */
        if (!rd(j->c + (hwaddr)(y / rows) * j->clofs, uv, n & ~1u)) {
            return;
        }
        for (x = 0; x + 1 < n; x += 2) {
            int su = 0, sv = 0, sa = 0, cnt = 0, a;

            for (k = 0; k < (avg ? got : 1); k++) {
                uint32_t i;

                for (i = 0; i < (avg ? 2 : 1); i++) {
                    Yuva *p = &px[k * n + x + i];

                    su += p->u * p->a;
                    sv += p->v * p->a;
                    sa += p->a;
                    cnt++;
                }
            }
            if (sa) {
                a = sa / cnt;
                uv[x] = blend(su / sa, uv[x], a);
                uv[x + 1] = blend(sv / sa, uv[x + 1], a);
            }
        }
        wr(j->c + (hwaddr)(y / rows) * j->clofs, uv, n & ~1u);
    }
}

/*
 * GOP flips, a line at a time, units of the precision: for a vertical
 * flip or a half turn the driver points C at the last line, so line y of
 * A lands y lines above it.
 */
static void grph_flip(const GrphJob *j, unsigned gop, unsigned unit)
{
    g_autofree uint8_t *a = g_malloc(j->w), *c = g_malloc(j->w);
    bool rev = gop != GOP_VTC_FLIP, up = gop != GOP_HRZ_FLIP;
    uint32_t y, i, n = j->w / unit;

    for (y = 0; y < j->h; y++) {
        if (!rd(j->a + (hwaddr)y * j->alofs, a, j->w)) {
            return;
        }
        if (rev) {
            for (i = 0; i < n; i++) {
                memcpy(c + i * unit, a + (n - 1 - i) * unit, unit);
            }
        } else {
            memcpy(c, a, j->w);
        }
        wr(up ? j->c - (hwaddr)y * j->clofs : j->c + (hwaddr)y * j->clofs,
           c, n * unit);
    }
}

void nvt_gfx_grph_run(const uint32_t *regs)
{
    static const unsigned unit[8] = { 0, 1, 2, 4 };
    uint32_t cfg = R(GRPH_CONFIG);
    unsigned prec = (cfg >> 4) & 7;
    GrphJob j = {
        .a = R(GRPH_A_ADDR) & 0x7FFFFFFF, .alofs = R(GRPH_A_LOFS) & 0x1FFFF,
        .b = R(GRPH_B_ADDR) & 0x7FFFFFFF, .blofs = R(GRPH_B_LOFS) & 0x1FFFF,
        .c = R(GRPH_C_ADDR) & 0x7FFFFFFF, .clofs = R(GRPH_C_LOFS) & 0x1FFFF,
        .w = R(GRPH_WIDTH) & 0x3FFF, .h = R(GRPH_HEIGHT) & 0x3FFF,
    };

    if (!j.w || !j.h) {
        return;
    }
    if ((uint64_t)j.w * j.h > GRPH_MAX_JOB) {
        qemu_log_mask(LOG_UNIMP, "nvt-gfx: graphics job of %u x %u bytes\n",
                      j.w, j.h);
        return;
    }
    if (!(cfg & 1)) {
        unsigned gop = (cfg >> 16) & 0xF;

        if ((gop == GOP_HRZ_FLIP || gop == GOP_VTC_FLIP ||
             gop == GOP_ROT_180) && unit[prec]) {
            grph_flip(&j, gop, unit[prec]);
        } else {
            qemu_log_mask(LOG_UNIMP, "nvt-gfx: graphics GOP %u at "
                          "precision %u\n", gop, prec);
        }
        return;
    }
    switch ((cfg >> 24) & 0x1F) {
    case AOP_A_COPY:
        {
            g_autofree uint8_t *row = g_malloc(j.w);
            uint32_t y;

            for (y = 0; y < j.h; y++) {
                if (!rd(j.a + (hwaddr)y * j.alofs, row, j.w)) {
                    return;
                }
                wr(j.c + (hwaddr)y * j.clofs, row, j.w);
            }
        }
        break;
    case AOP_PLUS_SHF:
    case AOP_MINUS_SHF:
    case AOP_MULTIPLY_DIV:
        if (prec == 1 || prec == 2) {
            grph_arith(regs, &j, (cfg >> 24) & 0x1F, prec == 2);
            break;
        }
        goto unimp;
    case AOP_TEXT_COPY:
        if (prec == 1 || prec == 2) {
            grph_fill(regs, &j, cfg, prec);
            break;
        }
        goto unimp;
    case AOP_PACKING:
        grph_pack(&j);
        break;
    case AOP_RGBYUV_BLEND:
        grph_rgbyuv_blend(regs, &j, cfg);
        break;
    default:
    unimp:
        qemu_log_mask(LOG_UNIMP, "nvt-gfx: graphics AOP 0x%x at precision "
                      "%u\n", (cfg >> 24) & 0x1F, prec);
        break;
    }
}

/* ---- ISE ---- */

#define ISE_IN_CTRL     0x04
#define ISE_IN_SIZE     0x10
#define ISE_IN_LOFS     0x18
#define ISE_IN_ADDR     0x20
#define ISE_OUT_SIZE    0x38
#define ISE_OUT_LOFS    0x90
#define ISE_OUT_ADDR    0x98

#define ISE_MAX_INPUT   (64 * MiB)
#define ISE_MAX_OUTPUT  (64 * MiB)

/*
 * Scale the input to the output size, each byte of a pixel a channel
 * (Y8: one; U / V pairs: two; ARGB8888: four).  Bilinear maps the corner
 * pixels onto each other (x * (in - 1) / (out - 1), the scale factor
 * ise_eng_cal_scale_factor programs); nearest takes x * in / out;
 * integration averages the input pixels each output pixel covers.
 */
void nvt_gfx_ise_run(const uint32_t *regs)
{
    uint32_t ctrl = R(ISE_IN_CTRL), fmt = ctrl & 0xF, method = (ctrl >> 8) & 3;
    uint32_t iw = R(ISE_IN_SIZE) & 0xFFFF, ih = R(ISE_IN_SIZE) >> 16;
    uint32_t ow = R(ISE_OUT_SIZE) & 0xFFFF, oh = R(ISE_OUT_SIZE) >> 16;
    uint32_t ilofs = R(ISE_IN_LOFS) & 0xFFFFC, olofs = R(ISE_OUT_LOFS) &
                     0xFFFFC;
    hwaddr in = R(ISE_IN_ADDR), out = R(ISE_OUT_ADDR);
    unsigned nch = fmt == 0 ? 1 : fmt == 3 ? 2 : fmt == 5 ? 4 : 0;
    g_autofree uint8_t *src = NULL, *row = NULL;
    uint64_t xstep, ystep;
    uint32_t ox, oy, y, c;

    if (!iw || !ih || !ow || !oh) {
        return;
    }
    if (!nch) {
        qemu_log_mask(LOG_UNIMP, "nvt-gfx: ISE format %u\n", fmt);
        return;
    }
    if ((uint64_t)iw * nch * ih > ISE_MAX_INPUT ||
        (uint64_t)ow * nch * oh > ISE_MAX_OUTPUT ||
        ilofs < iw * nch || olofs < ow * nch) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-gfx: ISE %ux%u -> %ux%u, line "
                      "offsets %u / %u\n", iw, ih, ow, oh, ilofs, olofs);
        return;
    }
    src = g_malloc((size_t)iw * nch * ih);
    for (y = 0; y < ih; y++) {
        if (!rd(in + (hwaddr)y * ilofs, src + (size_t)y * iw * nch, iw * nch)) {
            return;
        }
    }
    row = g_malloc((size_t)ow * nch);
    xstep = ow > 1 ? ((uint64_t)(iw - 1) << 16) / (ow - 1) : 0;
    ystep = oh > 1 ? ((uint64_t)(ih - 1) << 16) / (oh - 1) : 0;
    for (oy = 0; oy < oh; oy++) {
        for (ox = 0; ox < ow; ox++) {
            for (c = 0; c < nch; c++) {
                const uint8_t *p = src + c;
                uint32_t v;

                if (method == 2) {
                    v = p[((size_t)oy * ih / oh * iw + ox * iw / ow) * nch];
                } else if (method == 3) {
                    uint32_t x0 = (uint64_t)ox * iw / ow;
                    uint32_t x1 = MAX((uint64_t)(ox + 1) * iw / ow, x0 + 1);
                    uint32_t y0 = (uint64_t)oy * ih / oh;
                    uint32_t y1 = MAX((uint64_t)(oy + 1) * ih / oh, y0 + 1);
                    uint64_t sum = 0;
                    uint32_t xx, yy;

                    for (yy = y0; yy < y1; yy++) {
                        for (xx = x0; xx < x1; xx++) {
                            sum += p[((size_t)yy * iw + xx) * nch];
                        }
                    }
                    v = (sum + (x1 - x0) * (y1 - y0) / 2) /
                        ((x1 - x0) * (y1 - y0));
                } else {
                    uint64_t fx = ox * xstep, fy = oy * ystep;
                    uint32_t x0 = fx >> 16, y0 = fy >> 16;
                    uint32_t x1 = MIN(x0 + 1, iw - 1), y1 = MIN(y0 + 1, ih - 1);
                    uint64_t ax = fx & 0xFFFF, ay = fy & 0xFFFF;
                    uint64_t top = p[((size_t)y0 * iw + x0) * nch] *
                                   (0x10000 - ax) +
                                   p[((size_t)y0 * iw + x1) * nch] * ax;
                    uint64_t bot = p[((size_t)y1 * iw + x0) * nch] *
                                   (0x10000 - ax) +
                                   p[((size_t)y1 * iw + x1) * nch] * ax;

                    v = (top * (0x10000 - ay) + bot * ay +
                         (1ull << 31)) >> 32;
                }
                row[ox * nch + c] = v;
            }
        }
        wr(out + (hwaddr)oy * olofs, row, (size_t)ow * nch);
    }
}
