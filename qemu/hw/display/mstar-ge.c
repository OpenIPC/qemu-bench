/*
 * MStar/SigmaStar GE (2D graphics engine)
 *
 * The GE is a bitblt/fill/line accelerator at 0x1f281200. The MI_GFX
 * middleware and MainUI composite the on-screen UI through it, so nothing
 * appears on the panel until its blits run: MI_GFX programs a source and
 * destination surface, kicks the engine and waits for it to go idle.
 *
 * This models the completion status (the engine always reads back idle, with
 * its command FIFO reported empty and fully free so the driver's wait loops
 * never block) and executes bitblt copies so the composited UI actually lands
 * in the framebuffer the GOP scans out. Register map: linux-chenxing ip/ge.md.
 *
 * Ported from github.com/fifteenhex/qemu (branch mstar-clean,
 * hw/display/mstar_ge.c) for the SigmaStar machines in hw/arm/sigmastar.c.
 *
 * Copyright (c) 2026 Daniel Palmer <daniel@thingy.jp>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "system/address-spaces.h"

#define TYPE_MSTAR_GE "mstar-ge"
OBJECT_DECLARE_SIMPLE_TYPE(MStarGeState, MSTAR_GE)

#define MSTAR_GE_SIZE     0x200
#define MSTAR_GE_NUM_REGS (MSTAR_GE_SIZE / 4)

struct MStarGeState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;               /* job-done interrupt */
    /* Physical base of DRAM; the GE's surface addresses are MIU offsets */
    uint32_t dram_base;

    uint16_t regs[MSTAR_GE_NUM_REGS];
};

/*
 * Status register. GE_WaitIdle polls bits[7:3] for 0x10; GE_WaitCmdQAvail
 * waits until (status >> 11) is at least the number of commands it wants to
 * push. Report bit7 (CMDQ FIFO empty), bit0 (not busy) and a full set of free
 * FIFO slots [15:11] so neither wait ever blocks.
 */
#define GE_STATUS       0x1c
#define GE_STATUS_IDLE  0xf880

/* Command register: bit6 kicks a bitblt */
#define GE_CMD          0x180
/*
 * Blit direction bits: the driver flips the destination (and optionally the
 * source) rather than using the rotation register for the cheap 90/180 cases.
 * A 180 degree blit comes through as H+V destination flip.
 */
#define GE_CMD_DST_HFLIP (1 << 9)   /* pri_x_dir */
#define GE_CMD_DST_VFLIP (1 << 10)  /* pri_y_dir */

/* Source/destination surface base addresses (MIU offsets), low/high halves */
#define GE_SRC_ADDR_L   0x80
#define GE_SRC_ADDR_H   0x84
#define GE_DST_ADDR_L   0x98
#define GE_DST_ADDR_H   0x9c
/* Source/destination pitch, in bytes */
#define GE_SRC_PITCH    0xc0
#define GE_DST_PITCH    0xcc
/* Pixel formats: source in bits[3:0], destination in bits[11:8] */
#define GE_FMT          0xd0
/* Rotation in bits[1:0]: 0/1/2/3 = 0/90/180/270 degrees */
#define GE_ROTATE       0x164
/* Destination rectangle bottom-right corner (v0) */
#define GE_DST_X0       0x1a0
#define GE_DST_Y0       0x1a4
/* Blit width/height (the source rectangle size) */
#define GE_WIDTH        0x1b8
#define GE_HEIGHT       0x1bc

/* Pixel-format codes (ip/ge.md b_fm field) MainUI uses */
#define GE_FMT_RGB565   0x8
#define GE_FMT_ARGB1555 0x9
#define GE_FMT_ARGB4444 0xa
#define GE_FMT_ARGB8888 0xf

#define GE_ROTATE_180   2
#define GE_MAX_DIM      4096

/*
 * The kernel GE driver (drivers/gpu/drm/mstar/mstar_ge.c) selects the
 * primitive with prim_type in CMD bits[6:4] (LINE=1, RECTFILL=3, BITBLT=4),
 * rather than the single bit MI_GFX pokes for a blit. The first vertex is in
 * GE_DST_X0/Y0, the second in GE_X1/Y1; the fill/line colour is the "start
 * colour" in BG_ST/RA_ST; and drawing is clipped to the clip window.
 *
 * NOTE: the line/rectfill executors below are a best-effort model to be aligned
 * against real silicon later - the fractional line walk and the exact rectfill
 * edge/clip semantics are guesses.
 */
#define GE_CMD_PRIMTYPE(v)  (((v) >> 4) & 0x7)
#define GE_PRIM_LINE        1
#define GE_PRIM_RECTFILL    3
#define GE_PRIM_BITBLT      4

/* Second vertex (line end / rectfill bottom-right); first is GE_DST_X0/Y0. */
#define GE_X1           0x1a8
#define GE_Y1           0x1ac
/* Start colour: BG_ST = b | (g << 8), RA_ST = r | (a << 8). */
#define GE_BG_ST        0x1c0
#define GE_RA_ST        0x1c4
/* Clip window, inclusive. */
#define GE_CLIP_LEFT    0x154
#define GE_CLIP_RIGHT   0x158
#define GE_CLIP_TOP     0x15c
#define GE_CLIP_BOTTOM  0x160

/*
 * Job-done interrupt (REG_IRQ, 0x78): the kernel driver waits for a completion
 * IRQ per job (wait_event on ge->inflight). status is bits[13:12], and the
 * handler clears it by setting the clear bits[11:10]. Fire one IRQ per kicked
 * primitive; hold the (level) line until the driver clears it.
 */
#define GE_IRQ_REG      0x78
#define GE_IRQ_STATUS   (0x3 << 12)
#define GE_IRQ_CLR      (0x3 << 10)

static unsigned ge_bpp(unsigned fmt)
{
    return fmt == GE_FMT_ARGB8888 ? 4 : 2;
}

/* Decode one source pixel to 0xAARRGGBB */
static uint32_t ge_to_argb(unsigned fmt, uint32_t px)
{
    unsigned a, r, g, b;

    switch (fmt) {
    case GE_FMT_ARGB8888:
        return px;                            /* stored B,G,R,A LE == ARGB32 */
    case GE_FMT_RGB565:
        r = (px >> 11) & 0x1f;
        g = (px >> 5) & 0x3f;
        b = px & 0x1f;
        r = (r << 3) | (r >> 2);
        g = (g << 2) | (g >> 4);
        b = (b << 3) | (b >> 2);
        return 0xff000000 | (r << 16) | (g << 8) | b;
    case GE_FMT_ARGB1555:
        a = (px & 0x8000) ? 0xff : 0;
        r = (px >> 10) & 0x1f;
        g = (px >> 5) & 0x1f;
        b = px & 0x1f;
        r = (r << 3) | (r >> 2);
        g = (g << 3) | (g >> 2);
        b = (b << 3) | (b >> 2);
        return (a << 24) | (r << 16) | (g << 8) | b;
    case GE_FMT_ARGB4444:
        a = (px >> 12) & 0xf;
        r = (px >> 8) & 0xf;
        g = (px >> 4) & 0xf;
        b = px & 0xf;
        return ((a * 0x11) << 24) | ((r * 0x11) << 16) | ((g * 0x11) << 8) |
               (b * 0x11);
    default:
        return px;
    }
}

/* Encode 0xAARRGGBB into a destination pixel */
static uint32_t ge_from_argb(unsigned fmt, uint32_t argb)
{
    unsigned a = (argb >> 24) & 0xff, r = (argb >> 16) & 0xff;
    unsigned g = (argb >> 8) & 0xff, b = argb & 0xff;

    switch (fmt) {
    case GE_FMT_ARGB8888:
        return argb;
    case GE_FMT_RGB565:
        return ((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3);
    case GE_FMT_ARGB1555:
        return (a ? 0x8000 : 0) | ((r & 0xf8) << 7) | ((g & 0xf8) << 2) |
               (b >> 3);
    case GE_FMT_ARGB4444:
        return ((a & 0xf0) << 8) | ((r & 0xf0) << 4) | (g & 0xf0) | (b >> 4);
    default:
        return argb;
    }
}

/*
 * Execute a bitblt. MI_GFX programs a source and destination surface (MIU
 * addresses, so physical = DRAM base + reg), pitches, pixel formats and a
 * destination rectangle whose size is width x height. Copy the pixels,
 * converting the format, so the composited UI lands in the framebuffer.
 * The x0/y0 registers hold the destination rectangle's BOTTOM-RIGHT corner, so
 * the top-left is (x0-(w-1), y0-(h-1)); a full-screen 640x480 blit programs
 * (639, 479) = position (0, 0). MainUI pre-rotates 180 degrees for the
 * upside-down panel; honour that. Alpha blending, ROP, stretch and 90/270
 * rotation are not modelled.
 */
static void mstar_ge_bitblt(MStarGeState *s)
{
    const uint16_t *r = s->regs;
    uint32_t src = s->dram_base +
                   (((uint32_t)r[GE_SRC_ADDR_H / 4] << 16) |
                     r[GE_SRC_ADDR_L / 4]);
    uint32_t dst = s->dram_base +
                   (((uint32_t)r[GE_DST_ADDR_H / 4] << 16) |
                     r[GE_DST_ADDR_L / 4]);
    unsigned spit = r[GE_SRC_PITCH / 4], dpit = r[GE_DST_PITCH / 4];
    unsigned sfmt = r[GE_FMT / 4] & 0xf, dfmt = (r[GE_FMT / 4] >> 8) & 0xf;
    unsigned w = r[GE_WIDTH / 4], h = r[GE_HEIGHT / 4];
    unsigned rot = r[GE_ROTATE / 4] & 3;
    unsigned cmd = r[GE_CMD / 4];
    /*
     * A 180 degree blit reaches us either via the rotation register or, more
     * commonly, as an H+V destination flip. Treat both the same. (The source
     * V-flip bit isn't needed for the rotation path and isn't modelled.)
     */
    bool hflip = rot == GE_ROTATE_180 || (cmd & GE_CMD_DST_HFLIP);
    bool vflip = rot == GE_ROTATE_180 || (cmd & GE_CMD_DST_VFLIP);
    unsigned sbpp = ge_bpp(sfmt), dbpp = ge_bpp(dfmt);
    int dx = (int)r[GE_DST_X0 / 4] - (int)w + 1;
    int dy = (int)r[GE_DST_Y0 / 4] - (int)h + 1;
    unsigned row, col;

    if (w == 0 || h == 0 || w > GE_MAX_DIM || h > GE_MAX_DIM ||
        dx < 0 || dy < 0) {
        return;
    }

    for (row = 0; row < h; row++) {
        uint8_t sbuf[GE_MAX_DIM * 4], dbuf[GE_MAX_DIM * 4];
        unsigned orow = vflip ? h - 1 - row : row;
        uint32_t srow = src + row * spit;
        uint32_t drow = dst + (dy + orow) * dpit + dx * dbpp;

        if (address_space_read(&address_space_memory, srow,
                               MEMTXATTRS_UNSPECIFIED, sbuf,
                               w * sbpp) != MEMTX_OK) {
            return;
        }
        for (col = 0; col < w; col++) {
            uint32_t px = sbpp == 4 ? ldl_le_p(sbuf + col * 4)
                                    : lduw_le_p(sbuf + col * 2);
            uint32_t out = ge_from_argb(dfmt, ge_to_argb(sfmt, px));
            unsigned ocol = hflip ? w - 1 - col : col;

            if (dbpp == 4) {
                stl_le_p(dbuf + ocol * 4, out);
            } else {
                stw_le_p(dbuf + ocol * 2, out);
            }
        }
        address_space_write(&address_space_memory, drow,
                            MEMTXATTRS_UNSPECIFIED, dbuf, w * dbpp);
    }
    /* The blit lands in DRAM; the display redraws from it every frame. */
}

/* The fill/line start colour, as 0xAARRGGBB. */
static uint32_t ge_start_argb(MStarGeState *s)
{
    const uint16_t *r = s->regs;
    unsigned b = r[GE_BG_ST / 4] & 0xff, g = (r[GE_BG_ST / 4] >> 8) & 0xff;
    unsigned rr = r[GE_RA_ST / 4] & 0xff, a = (r[GE_RA_ST / 4] >> 8) & 0xff;

    return (a << 24) | (rr << 16) | (g << 8) | b;
}

/*
 * Write one destination pixel, encoded to the destination format and clipped
 * to the clip window (only when the driver has actually programmed one, i.e.
 * right > left / bottom > top).
 */
static void ge_put_pixel(MStarGeState *s, int x, int y, uint32_t argb)
{
    const uint16_t *r = s->regs;
    uint32_t dst = s->dram_base +
                   (((uint32_t)r[GE_DST_ADDR_H / 4] << 16) |
                     r[GE_DST_ADDR_L / 4]);
    unsigned dpit = r[GE_DST_PITCH / 4];
    unsigned dfmt = (r[GE_FMT / 4] >> 8) & 0xf;
    unsigned dbpp = ge_bpp(dfmt);
    int cl = r[GE_CLIP_LEFT / 4], cr = r[GE_CLIP_RIGHT / 4];
    int ct = r[GE_CLIP_TOP / 4], cb = r[GE_CLIP_BOTTOM / 4];
    uint32_t px = ge_from_argb(dfmt, argb);
    uint8_t buf[4];

    if (x < 0 || y < 0) {
        return;
    }
    if (cr > cl && (x < cl || x > cr)) {
        return;
    }
    if (cb > ct && (y < ct || y > cb)) {
        return;
    }
    if (dbpp == 4) {
        stl_le_p(buf, px);
    } else {
        stw_le_p(buf, px);
    }
    address_space_write(&address_space_memory,
                        dst + (unsigned)y * dpit + (unsigned)x * dbpp,
                        MEMTXATTRS_UNSPECIFIED, buf, dbpp);
}

/*
 * RECTFILL: the driver puts the top-left corner in (X0,Y0) and the bottom-right
 * in (X1,Y1) and fills the rectangle with the start colour. Treated as
 * inclusive of both corners - to confirm on hardware.
 */
static void mstar_ge_rectfill(MStarGeState *s)
{
    const uint16_t *r = s->regs;
    int x0 = r[GE_DST_X0 / 4] & 0xfff, y0 = r[GE_DST_Y0 / 4] & 0xfff;
    int x1 = r[GE_X1 / 4] & 0xfff, y1 = r[GE_Y1 / 4] & 0xfff;
    uint32_t argb = ge_start_argb(s);
    int x, y;

    for (y = y0; y <= y1; y++) {
        for (x = x0; x <= x1; x++) {
            ge_put_pixel(s, x, y, argb);
        }
    }
}

/*
 * LINE: draw from (X0,Y0) to (X1,Y1) in the start colour. The hardware walks
 * the line with the fractional line_delta/line_major registers; a plain
 * Bresenham raster is close enough to eyeball and will be aligned with silicon.
 */
static void mstar_ge_line(MStarGeState *s)
{
    const uint16_t *r = s->regs;
    int x0 = r[GE_DST_X0 / 4] & 0xfff, y0 = r[GE_DST_Y0 / 4] & 0xfff;
    int x1 = r[GE_X1 / 4] & 0xfff, y1 = r[GE_Y1 / 4] & 0xfff;
    uint32_t argb = ge_start_argb(s);
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        int e2;

        ge_put_pixel(s, x0, y0, argb);
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
}

static uint64_t mstar_ge_read(void *opaque, hwaddr addr, unsigned size)
{
    MStarGeState *s = MSTAR_GE(opaque);

    /* RIU: 16-bit registers, nothing at +2 of each 4-byte slot. */
    if (addr & 2) {
        return 0;
    }
    if (addr == GE_STATUS) {
        return GE_STATUS_IDLE;
    }
    return s->regs[addr / 4];
}

static void mstar_ge_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    MStarGeState *s = MSTAR_GE(opaque);

    if (addr & 2) {
        return;
    }
    s->regs[addr / 4] = val;

    /* The driver clears the job-done IRQ by setting the clear bits. */
    if (addr == GE_IRQ_REG && (val & GE_IRQ_CLR)) {
        s->regs[GE_IRQ_REG / 4] &= ~GE_IRQ_STATUS;
        qemu_set_irq(s->irq, 0);
    }

    if (addr == GE_CMD) {
        bool done = true;

        switch (GE_CMD_PRIMTYPE(val)) {
        case GE_PRIM_LINE:
            mstar_ge_line(s);
            break;
        case GE_PRIM_RECTFILL:
            mstar_ge_rectfill(s);
            break;
        case GE_PRIM_BITBLT:
            mstar_ge_bitblt(s);
            break;
        default:
            done = false;
            break;
        }
        if (done) {
            /* Raise the job-done interrupt (level; held until cleared). */
            s->regs[GE_IRQ_REG / 4] |= GE_IRQ_STATUS;
            qemu_set_irq(s->irq, 1);
        }
    }
}

static const MemoryRegionOps mstar_ge_ops = {
    .read = mstar_ge_read,
    .write = mstar_ge_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
};

static void mstar_ge_reset(DeviceState *dev)
{
    MStarGeState *s = MSTAR_GE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_set_irq(s->irq, 0);
}

static void mstar_ge_init(Object *obj)
{
    MStarGeState *s = MSTAR_GE(obj);

    memory_region_init_io(&s->iomem, obj, &mstar_ge_ops, s,
                          "mstar-ge", MSTAR_GE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property mstar_ge_properties[] = {
    DEFINE_PROP_UINT32("dram-base", MStarGeState, dram_base, 0),
};

static void mstar_ge_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, mstar_ge_reset);
    device_class_set_props(dc, mstar_ge_properties);
}

static const TypeInfo mstar_ge_types[] = {
    {
        .name           = TYPE_MSTAR_GE,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MStarGeState),
        .instance_init  = mstar_ge_init,
        .class_init     = mstar_ge_class_init,
    },
};

DEFINE_TYPES(mstar_ge_types)
