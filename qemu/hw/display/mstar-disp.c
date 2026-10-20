/*
 * MStar/SigmaStar display controller (GOP + display top)
 *
 * The front of the display pipe that feeds the DSI output path. The
 * GOP (Graphic Output Processor) scans a framebuffer plane out of
 * DRAM; the display top generates the frame timing and the vsync
 * interrupt the framebuffer driver waits on.
 *
 * The primary GOP window (WIN0, the RGB plane the vendor fbdev and DRM
 * fbcon use) and the MOP video windows (semi-planar YUV420 surfaces: the
 * boot logo, the NVR's live-view channels) are composited - the GOP over
 * the MOP, opaque but for its transparent colour when keyed - and
 * scanned out to a QEMU console. The
 * display-top vsync interrupt is generated each frame. The UI is
 * composited into the GOP plane by the GE 2D engine (mstar-ge.c); the
 * whole frame is redrawn on every update.
 *
 * GOP registers (16-bit, RIU 4 byte stride), from the previous branch
 * and the mainline mstar DRM driver (``prev``, ``linux``):
 *   0xc0  STRETCH_W   [11:0] crtc width >> 1
 *   0xc4  STRETCH_H   [11:0] crtc height
 *   0x200 WIN0        bit0 enable, bits[7:4] pixel format
 *   0x204 WIN0_ADDRL  [15:0] framebuffer address, low
 *   0x208 WIN0_ADDRH  [11:0] framebuffer address, high
 *   0x224 WIN0_PITCH  [10:0] stride, in 16 byte units
 * The address and pitch are in 16 byte units (<<4). Pixel format
 * 0x1 = RGB565, 0x5 = ARGB8888, 0x7 = ABGR8888.
 *
 * Display-top registers:
 *   0x08  VSYNC_FLAG  bit3 vsync pending, write-1-to-clear
 *   0x0c  VSYNC_MASK  bit3, 0 = vsync interrupt enabled
 *
 * Ported from github.com/fifteenhex/qemu (branch mstar-clean,
 * hw/display/mstar_disp.c) for the SigmaStar machines in hw/arm/sigmastar.c.
 * The SSR621Q NVR firmware's fbdev drives this GOP (its "GOP 1", banks
 * 0x1234/0x1235) at 1280x1024 in ARGB1555, so the 16-bit formats
 * ARGB1555 (6) and ARGB4444 (2) of MStar's GOP format field are decoded
 * as well.
 *
 * Copyright (c) 2026 Daniel Palmer <daniel@thingy.jp>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "qom/object.h"
#include "system/address-spaces.h"

/* DRAM, the MIU0 window: plane addresses are offsets into it. */
#define MSTARV7_MIU0_BASE       0x20000000
#define MSTARV7_MIU0_WINDOW     (512 * 1024 * 1024)

#define TYPE_MSTAR_DISP "mstar-disp"
OBJECT_DECLARE_SIMPLE_TYPE(MStarDispState, MSTAR_DISP)

#define MSTAR_DISP_GOP_SIZE 0x400
#define MSTAR_DISP_TOP_SIZE 0x200
#define MSTAR_DISP_MOP_SIZE 0x600
#define MSTAR_DISP_GOP_NUM_REGS (MSTAR_DISP_GOP_SIZE / 4)
#define MSTAR_DISP_TOP_NUM_REGS (MSTAR_DISP_TOP_SIZE / 4)
#define MSTAR_DISP_MOP_NUM_REGS (MSTAR_DISP_MOP_SIZE / 4)

struct MStarDispState {
    SysBusDevice parent_obj;

    MemoryRegion gop;
    MemoryRegion top;
    MemoryRegion mop;
    QemuConsole *con;
    QEMUTimer *vblank;
    QEMUTimer *gop_pulse;   /* one-shot: end of the GOP vsync pulse */
    qemu_irq top_irq;       /* display-top vsync */
    qemu_irq gop_irq;       /* GOP/fbdev vsync */

    uint32_t width;
    uint32_t height;
    bool flip;              /* panel mounted 180 degrees ("flip" property) */

    uint16_t gopregs[MSTAR_DISP_GOP_NUM_REGS];
    uint16_t topregs[MSTAR_DISP_TOP_NUM_REGS];
    uint16_t mopregs[MSTAR_DISP_MOP_NUM_REGS];
};

/* GOP registers */
#define GOP_CTRL0           0x00
#define GOP_CTRL0_TRS_EN    (1 << 11)   /* transparent colour keying */
#define GOP_TRS_CLR_L       0x90        /* the key, RGB888 */
#define GOP_TRS_CLR_H       0x94
#define GOP_STRETCH_W       0xc0
#define GOP_STRETCH_H       0xc4
#define GOP_WIN0            0x200
#define GOP_WIN0_EN         (1 << 0)
#define GOP_WIN0_FMT(v)     (((v) >> 4) & 0xf)
#define GOP_WIN0_ADDRL      0x204
#define GOP_WIN0_ADDRH      0x208
#define GOP_WIN0_PITCH      0x224
#define GOP_ADDR_SHIFT      4
#define GOP_FMT_RGB565      0x1
#define GOP_FMT_ARGB4444    0x2
#define GOP_FMT_ARGB1555    0x6
#define GOP_FMT_ARGB8888    0x5
#define GOP_FMT_ABGR8888    0x7

/* Display-top registers */
#define TOP_VSYNC_FLAG      0x08
#define TOP_VSYNC_MASK      0x0c
#define TOP_VSYNC_BIT       (1 << 3)

/*
 * MOP (video/overlay) windows: 16 of them, 0x40 bytes apart from MOP +
 * 0x200 (banks 0x1406/0x1407). Each scans a semi-planar YUV420 (NV12)
 * surface into a rectangle of the screen, scaled by source/destination
 * ratios in 1/4096ths. The vendor u-boot points window 0 at the JPEG-
 * decoded boot logo; the SSR621Q's mi_disp gives each live-view channel
 * a window (the 9-split: 426x340 cells). Read off the running NVR.
 */
#define MOP_WIN0            0x200
#define MOP_WIN_STRIDE      0x40
#define MOP_NUM_WIN         16
#define MOP_WIN_EN          0x00        /* bit0 enable */
#define MOP_WIN_YADDRL      0x08
#define MOP_WIN_YADDRH      0x0c
#define MOP_WIN_CADDRL      0x10
#define MOP_WIN_CADDRH      0x14
#define MOP_WIN_HST         0x18        /* on the screen, inclusive */
#define MOP_WIN_HEND        0x1c
#define MOP_WIN_VST         0x20
#define MOP_WIN_VEND        0x24
#define MOP_WIN_PITCH       0x28        /* luma stride, 16 byte units */
#define MOP_WIN_SRCW        0x2c        /* source width - 1 */
#define MOP_WIN_SRCH        0x30        /* source height - 1 */
#define MOP_WIN_HRATIO      0x34        /* source / destination, 4.12 */
#define MOP_WIN_VRATIO      0x38
#define MOP_RATIO_ONE       0x1000
#define MOP_ADDR_SHIFT      4

#define MSTAR_DISP_DEFAULT_W    640
#define MSTAR_DISP_DEFAULT_H    480
#define MSTAR_DISP_REFRESH_NS   (NANOSECONDS_PER_SECOND / 60)
/*
 * How long the GOP vsync line is held per frame: long enough for the
 * level-based interrupt path to latch it, far shorter than a frame.
 */
#define MSTAR_DISP_VSYNC_PULSE_NS   200000

/*
 * RIU registers: 16 bits on a 4-byte stride, the high byte at +1; the
 * vendor drivers write bytes as well as halfwords.
 */
static uint64_t riu_reg_read(const uint16_t *regs, hwaddr addr, unsigned size)
{
    uint16_t v = regs[addr / 4];

    if (addr & 2) {
        return 0;
    }
    if (addr & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xff) : v;
}

static uint16_t riu_reg_merge(const uint16_t *regs, hwaddr addr,
                              uint64_t val, unsigned size)
{
    uint16_t v = regs[addr / 4];

    if (size != 1) {
        return val;
    }
    return (addr & 1) ? (v & 0x00ff) | ((val & 0xff) << 8)
                      : (v & 0xff00) | (val & 0xff);
}

/* A plane address plus a row offset, kept inside the MIU0 window. */
static hwaddr mstar_disp_dram(hwaddr plane, uint64_t off)
{
    return MSTARV7_MIU0_BASE +
           ((plane - MSTARV7_MIU0_BASE + off) & (MSTARV7_MIU0_WINDOW - 1));
}

static void mstar_disp_update_top_irq(MStarDispState *s)
{
    bool flag = s->topregs[TOP_VSYNC_FLAG / 4] & TOP_VSYNC_BIT;
    bool masked = s->topregs[TOP_VSYNC_MASK / 4] & TOP_VSYNC_BIT;

    qemu_set_irq(s->top_irq, flag && !masked);
}

static inline uint8_t mstar_disp_clamp(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/*
 * Draw the MOP (video) windows, semi-planar YUV420 (NV12) surfaces,
 * as the background: each is scaled into its rectangle on the screen.
 * Returns false (leaving the caller to clear to black) if none is
 * enabled.
 */
static bool mstar_disp_draw_mop_win(MStarDispState *s, int n, uint32_t *dst,
                                    uint32_t sw, uint32_t sh)
{
    const uint16_t *r = &s->mopregs[(MOP_WIN0 + n * MOP_WIN_STRIDE) / 4];
    uint32_t sx0 = r[MOP_WIN_HST / 4] & 0xfff;
    uint32_t sx1 = r[MOP_WIN_HEND / 4] & 0xfff;
    uint32_t sy0 = r[MOP_WIN_VST / 4] & 0xfff;
    uint32_t sy1 = r[MOP_WIN_VEND / 4] & 0xfff;
    uint32_t w = (r[MOP_WIN_SRCW / 4] & 0xfff) + 1;
    uint32_t h = (r[MOP_WIN_SRCH / 4] & 0xfff) + 1;
    uint32_t hratio = r[MOP_WIN_HRATIO / 4];
    uint32_t vratio = r[MOP_WIN_VRATIO / 4];
    uint32_t pitch = (r[MOP_WIN_PITCH / 4] & 0x1fff) << MOP_ADDR_SHIFT;
    /* Masked to the MIU0 window, as for the GOP (mstar_disp_draw_gop). */
    hwaddr yaddr = MSTARV7_MIU0_BASE +
        ((((hwaddr)(((r[MOP_WIN_YADDRH / 4] & 0xfff) << 16) |
                    r[MOP_WIN_YADDRL / 4])) << MOP_ADDR_SHIFT)
         & (MSTARV7_MIU0_WINDOW - 1));
    hwaddr caddr = MSTARV7_MIU0_BASE +
        ((((hwaddr)(((r[MOP_WIN_CADDRH / 4] & 0xfff) << 16) |
                    r[MOP_WIN_CADDRL / 4])) << MOP_ADDR_SHIFT)
         & (MSTARV7_MIU0_WINDOW - 1));
    g_autofree uint8_t *yrow = NULL;
    g_autofree uint8_t *crow = NULL;
    uint32_t dw, dh, x, y, last = UINT32_MAX;

    if (!(r[MOP_WIN_EN / 4] & 1)) {
        return false;
    }
    if (pitch == 0) {
        pitch = ROUND_UP(w, 2);
    }
    /* Whole chroma pairs within the pitch, an odd last row included. */
    w = MIN(w, pitch & ~1u);
    if (sx1 <= sx0 || sy1 <= sy0) {
        /* No rectangle (the U-Boot logo): unscaled at the origin. */
        sx0 = sy0 = 0;
        sx1 = w - 1;
        sy1 = h - 1;
        hratio = vratio = MOP_RATIO_ONE;
    }
    if (!hratio) {
        hratio = MOP_RATIO_ONE;
    }
    if (!vratio) {
        vratio = MOP_RATIO_ONE;
    }
    if (w == 0 || sx0 >= sw || sy0 >= sh) {
        return false;
    }
    dw = MIN(sx1, sw - 1) - sx0 + 1;
    dh = MIN(sy1, sh - 1) - sy0 + 1;

    /* One source row at a time: the guest picks pitch and height. */
    yrow = g_malloc(w);
    crow = g_malloc(w);
    for (y = 0; y < dh; y++) {
        uint32_t srcy = MIN((uint64_t)y * vratio / MOP_RATIO_ONE, h - 1);
        uint32_t *d = dst + (sy0 + y) * sw + sx0;

        if (srcy != last) {
            address_space_read(&address_space_memory,
                               mstar_disp_dram(yaddr, (uint64_t)srcy * pitch),
                               MEMTXATTRS_UNSPECIFIED, yrow, w);
            address_space_read(&address_space_memory,
                               mstar_disp_dram(caddr,
                                               (uint64_t)(srcy / 2) * pitch),
                               MEMTXATTRS_UNSPECIFIED, crow, w);
            last = srcy;
        }
        for (x = 0; x < dw; x++) {
            uint32_t srcx = MIN((uint64_t)x * hratio / MOP_RATIO_ONE, w - 1);
            int yy = yrow[srcx];
            int uu = crow[srcx & ~1u] - 128;         /* Cb */
            int vv = crow[(srcx & ~1u) + 1] - 128;   /* Cr */
            /* Full-range (JFIF) BT.601 YCbCr -> RGB */
            uint8_t rr = mstar_disp_clamp(yy + ((91881 * vv) >> 16));
            uint8_t g = mstar_disp_clamp(yy -
                            ((22554 * uu + 46802 * vv) >> 16));
            uint8_t b = mstar_disp_clamp(yy + ((116130 * uu) >> 16));

            d[x] = 0xff000000 | (rr << 16) | (g << 8) | b;
        }
    }
    return true;
}

static bool mstar_disp_draw_mop(MStarDispState *s, uint32_t *dst,
                                uint32_t sw, uint32_t sh)
{
    bool any = false;
    int n;

    for (n = 0; n < MOP_NUM_WIN; n++) {
        const uint16_t *r = &s->mopregs[(MOP_WIN0 + n * MOP_WIN_STRIDE) / 4];

        if (!(r[MOP_WIN_EN / 4] & 1)) {
            continue;
        }
        if (!any) {
            memset(dst, 0, (size_t)sw * sh * 4);
            any = true;
        }
        mstar_disp_draw_mop_win(s, n, dst, sw, sh);
    }
    return any;
}

/* Composite the GOP (RGB overlay) plane over the surface using its alpha */
static void mstar_disp_draw_gop(MStarDispState *s, uint32_t *dst,
                                uint32_t sw, uint32_t sh)
{
    uint16_t win0 = s->gopregs[GOP_WIN0 / 4];
    uint32_t fmt = GOP_WIN0_FMT(win0);
    uint32_t addrl = s->gopregs[GOP_WIN0_ADDRL / 4];
    uint32_t addrh = s->gopregs[GOP_WIN0_ADDRH / 4] & 0xfff;
    /*
     * The address register holds the driver's DMA address >> GOP_ADDR_SHIFT.
     * With no dma-ranges the mainline DRM driver's dma_addr is the full CPU
     * physical address (MIU0 base included); the MIU0 window only decodes
     * MSTARV7_MIU0_WINDOW, so those high base bits are not wired and wrap.
     * Mask to the window before adding the base back, so a full-physical
     * address (mainline) and a MIU-relative one (vendor) resolve alike.
     */
    hwaddr fbaddr = MSTARV7_MIU0_BASE +
                    ((((hwaddr)((addrh << 16) | addrl)) << GOP_ADDR_SHIFT)
                     & (MSTARV7_MIU0_WINDOW - 1));
    bool is32 = (fmt == GOP_FMT_ARGB8888 || fmt == GOP_FMT_ABGR8888);
    bool bgr = (fmt == GOP_FMT_ABGR8888);
    uint32_t bpp = is32 ? 4 : 2;
    uint32_t pitch = (s->gopregs[GOP_WIN0_PITCH / 4] & 0x7ff)
                     << GOP_ADDR_SHIFT;
    bool key_en = s->gopregs[GOP_CTRL0 / 4] & GOP_CTRL0_TRS_EN;
    uint32_t key = ((s->gopregs[GOP_TRS_CLR_H / 4] & 0xff) << 16) |
                   s->gopregs[GOP_TRS_CLR_L / 4];
    g_autofree uint8_t *buf = NULL;
    uint32_t x, y, w;

    if (!(win0 & GOP_WIN0_EN) || fbaddr == MSTARV7_MIU0_BASE) {
        return;
    }
    if (pitch == 0) {
        pitch = sw * bpp;
    }
    /* What of each line fits in the pitch. */
    w = MIN(sw, pitch / bpp);
    if (w == 0) {
        return;
    }

    /* One visible row at a time: the guest picks the pitch. */
    buf = g_malloc(w * bpp);
    for (y = 0; y < sh; y++) {
        const uint8_t *src = buf;
        uint32_t *d = dst + y * sw;

        address_space_read(&address_space_memory,
                           mstar_disp_dram(fbaddr, (uint64_t)y * pitch),
                           MEMTXATTRS_UNSPECIFIED, buf, w * bpp);

        for (x = 0; x < w; x++) {
            uint8_t r, g, b;

            if (is32) {
                uint32_t px = ldl_le_p(src + x * 4);
                uint8_t c0 = px & 0xff, c1 = (px >> 8) & 0xff;
                uint8_t c2 = (px >> 16) & 0xff;

                /* ARGB8888 is B,G,R,A in memory; ABGR8888 is R,G,B,A */
                r = bgr ? c0 : c2;
                g = c1;
                b = bgr ? c2 : c0;
            } else {
                uint16_t px = lduw_le_p(src + x * 2);

                if (fmt == GOP_FMT_ARGB1555) {
                    r = ((px >> 10) & 0x1f) << 3;
                    g = ((px >> 5) & 0x1f) << 3;
                    b = (px & 0x1f) << 3;
                } else if (fmt == GOP_FMT_ARGB4444) {
                    r = ((px >> 8) & 0xf) * 0x11;
                    g = ((px >> 4) & 0xf) * 0x11;
                    b = (px & 0xf) * 0x11;
                } else {
                    r = ((px >> 11) & 0x1f) << 3;
                    g = ((px >> 5) & 0x3f) << 2;
                    b = (px & 0x1f) << 3;
                }
            }
            /*
             * The firmware blends the GOP over the video plane with a
             * constant alpha of 255 (sstar_FB_SetBlending aType=1,
             * constAlpha=255), so the plane is opaque: the per-pixel
             * alpha is ignored (the UI leaves it at 0 on most pixels).
             * Only the transparent colour, when keyed, lets the video
             * through (the NVR's live view: black).
             */
            if (key_en && ((r << 16) | (g << 8) | b) == key) {
                continue;
            }
            d[x] = 0xff000000 | (r << 16) | (g << 8) | b;
        }
    }
}

/* Composite the display planes into the console surface */
static void mstar_disp_scanout(MStarDispState *s)
{
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint32_t *dst = (uint32_t *)surface_data(surface);
    uint32_t w = s->width, h = s->height;

    if (!mstar_disp_draw_mop(s, dst, w, h)) {
        memset(dst, 0, surface_stride(surface) * h);
    }
    mstar_disp_draw_gop(s, dst, w, h);

    /*
     * The Miyoo Mini's panel is mounted upside down, so the firmware
     * draws the framebuffer already rotated 180 degrees. Reverse the
     * pixels so a screendump shows what is physically on the panel.
     */
    if (s->flip) {
        uint32_t n = w * h, i;

        for (i = 0; i < n / 2; i++) {
            uint32_t t = dst[i];

            dst[i] = dst[n - 1 - i];
            dst[n - 1 - i] = t;
        }
    }
}

static void mstar_disp_gop_pulse_end(void *opaque)
{
    MStarDispState *s = MSTAR_DISP(opaque);

    qemu_set_irq(s->gop_irq, 0);
}

static void mstar_disp_vblank(void *opaque)
{
    MStarDispState *s = MSTAR_DISP(opaque);

    /* display-top vsync: latch the flag (the driver acks it W1C) */
    s->topregs[TOP_VSYNC_FLAG / 4] |= TOP_VSYNC_BIT;
    mstar_disp_update_top_irq(s);

    /*
     * The GOP/fbdev vsync is a separate interrupt with no status
     * register; the driver just counts it and wakes its vsync wait
     * (sstar_FB_WaitForVsync). The interrupt path is level based, so
     * an instantaneous pulse would be lost before the GIC samples it,
     * and holding the line would storm the never-acked handler: hold
     * the line for a short vsync-pulse interval instead.
     */
    qemu_set_irq(s->gop_irq, 1);
    timer_mod(s->gop_pulse,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              MSTAR_DISP_VSYNC_PULSE_NS);

    timer_mod(s->vblank,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + MSTAR_DISP_REFRESH_NS);
}

static uint64_t mstar_disp_gop_read(void *opaque, hwaddr addr, unsigned size)
{
    MStarDispState *s = MSTAR_DISP(opaque);

    return riu_reg_read(s->gopregs, addr, size);
}

static void mstar_disp_gop_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    MStarDispState *s = MSTAR_DISP(opaque);

    if (addr & 2) {
        return;
    }
    s->gopregs[addr / 4] = riu_reg_merge(s->gopregs, addr, val, size);
}

static uint64_t mstar_disp_top_read(void *opaque, hwaddr addr, unsigned size)
{
    MStarDispState *s = MSTAR_DISP(opaque);

    return riu_reg_read(s->topregs, addr, size);
}

static void mstar_disp_top_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    MStarDispState *s = MSTAR_DISP(opaque);

    if (addr & 2) {
        return;
    }
    switch (addr & ~3) {
    case TOP_VSYNC_FLAG:
        /* write-1-clear */
        s->topregs[TOP_VSYNC_FLAG / 4] &=
            ~(uint16_t)((size == 1 && (addr & 1)) ? val << 8 : val);
        break;
    default:
        s->topregs[addr / 4] = riu_reg_merge(s->topregs, addr, val, size);
        break;
    }
    mstar_disp_update_top_irq(s);
}

static const MemoryRegionOps mstar_disp_gop_ops = {
    .read = mstar_disp_gop_read,
    .write = mstar_disp_gop_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps mstar_disp_top_ops = {
    .read = mstar_disp_top_read,
    .write = mstar_disp_top_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static uint64_t mstar_disp_mop_read(void *opaque, hwaddr addr, unsigned size)
{
    MStarDispState *s = MSTAR_DISP(opaque);

    return riu_reg_read(s->mopregs, addr, size);
}

static void mstar_disp_mop_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    MStarDispState *s = MSTAR_DISP(opaque);

    if (addr & 2) {
        return;
    }
    s->mopregs[addr / 4] = riu_reg_merge(s->mopregs, addr, val, size);
}

static const MemoryRegionOps mstar_disp_mop_ops = {
    .read = mstar_disp_mop_read,
    .write = mstar_disp_mop_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void mstar_disp_gfx_update(void *opaque)
{
    MStarDispState *s = MSTAR_DISP(opaque);
    uint32_t w = MSTAR_DISP_DEFAULT_W, h = MSTAR_DISP_DEFAULT_H;
    uint32_t sw = (s->gopregs[GOP_STRETCH_W / 4] & 0xfff) << 1;
    uint32_t sh = s->gopregs[GOP_STRETCH_H / 4] & 0xfff;

    if (sw && sh) {
        w = sw;
        h = sh;
    }
    if (w != s->width || h != s->height) {
        s->width = w;
        s->height = h;
        qemu_console_resize(s->con, w, h);
    }

    mstar_disp_scanout(s);
    dpy_gfx_update_full(s->con);
}

static const GraphicHwOps mstar_disp_gfx_ops = {
    .gfx_update = mstar_disp_gfx_update,
};

static void mstar_disp_reset(DeviceState *dev)
{
    MStarDispState *s = MSTAR_DISP(dev);

    memset(s->gopregs, 0, sizeof(s->gopregs));
    memset(s->topregs, 0, sizeof(s->topregs));
    memset(s->mopregs, 0, sizeof(s->mopregs));
    /*
     * The vsync interrupt starts masked (VSYNC_MASK bit3 = 1). The driver
     * unmasks it once it has set up its regmap fields; latching it from
     * reset makes mstar_top_probe's request_irq fire the handler before the
     * fields exist, which NULL-derefs in regmap_field_force_write.
     */
    s->topregs[TOP_VSYNC_MASK / 4] = TOP_VSYNC_BIT;
    mstar_disp_update_top_irq(s);
    qemu_set_irq(s->gop_irq, 0);
}

static void mstar_disp_realize(DeviceState *dev, Error **errp)
{
    MStarDispState *s = MSTAR_DISP(dev);

    s->width = MSTAR_DISP_DEFAULT_W;
    s->height = MSTAR_DISP_DEFAULT_H;
    s->con = graphic_console_init(dev, 0, &mstar_disp_gfx_ops, s);
    qemu_console_resize(s->con, s->width, s->height);

    s->vblank = timer_new_ns(QEMU_CLOCK_VIRTUAL, mstar_disp_vblank, s);
    s->gop_pulse = timer_new_ns(QEMU_CLOCK_VIRTUAL, mstar_disp_gop_pulse_end,
                                s);
    timer_mod(s->vblank,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + MSTAR_DISP_REFRESH_NS);
}

static void mstar_disp_init(Object *obj)
{
    MStarDispState *s = MSTAR_DISP(obj);

    memory_region_init_io(&s->gop, obj, &mstar_disp_gop_ops, s,
                          "mstar-disp.gop", MSTAR_DISP_GOP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->gop);
    memory_region_init_io(&s->top, obj, &mstar_disp_top_ops, s,
                          "mstar-disp.top", MSTAR_DISP_TOP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->top);
    memory_region_init_io(&s->mop, obj, &mstar_disp_mop_ops, s,
                          "mstar-disp.mop", MSTAR_DISP_MOP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mop);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->top_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->gop_irq);
}

static const Property mstar_disp_properties[] = {
    DEFINE_PROP_BOOL("flip", MStarDispState, flip, false),
};

static const VMStateDescription vmstate_mstar_disp = {
    .name = TYPE_MSTAR_DISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(gopregs, MStarDispState,
                             MSTAR_DISP_GOP_NUM_REGS),
        VMSTATE_UINT16_ARRAY(topregs, MStarDispState,
                             MSTAR_DISP_TOP_NUM_REGS),
        VMSTATE_UINT16_ARRAY(mopregs, MStarDispState,
                             MSTAR_DISP_MOP_NUM_REGS),
        VMSTATE_TIMER_PTR(vblank, MStarDispState),
        VMSTATE_TIMER_PTR(gop_pulse, MStarDispState),
        VMSTATE_END_OF_LIST()
    }
};

static void mstar_disp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mstar_disp_realize;
    device_class_set_legacy_reset(dc, mstar_disp_reset);
    device_class_set_props(dc, mstar_disp_properties);
    dc->vmsd = &vmstate_mstar_disp;
}

static const TypeInfo mstar_disp_types[] = {
    {
        .name           = TYPE_MSTAR_DISP,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MStarDispState),
        .instance_init  = mstar_disp_init,
        .class_init     = mstar_disp_class_init,
    },
};

DEFINE_TYPES(mstar_disp_types)
