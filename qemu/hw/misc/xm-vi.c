/*
 * Xiongmai video input (VI) with the ISP's frame interrupts, as the vendor
 * VI drivers program it: XM530 (xm530_vi.ko, 0x30000000) and XM510
 * (xm510_vi.ko, 0x40000000).  The `soc` property picks the layout.
 *
 * What the model does was learnt from the register writes the drivers make
 * at run time (logged with -d unimp), not from their code.
 *
 * XM530.  Per channel c, in the window W = 0x84000 + c * 0x1000:
 *
 *   W+0x20/+0x28   luma / chroma stride, in 8-byte units
 *   W+0x24         width in pixels
 *   W+0x34         channel enable
 *   W+0x40/+0x44   buffer 0/1 luma,   (address - 0x80000000) >> 3
 *   W+0xC0/+0xC4   buffer 0/1 chroma, likewise (NV12: CbCr interleaved)
 *   W+0x08         buffer-done flags, one bit per buffer
 *   0xF0080+c*0x20 interrupt enable; +4 the height plus one
 *   0xF008C+c*0x20 interrupt status: set with a frame, cleared by writing 0
 *
 * The ISP's two frame interrupts sit in the same grid, below the channels:
 * 0xF0040 (isp_irq_bayer, SPI 25) and 0xF0060 (isp_irq_lum, SPI 26), enable
 * at +0, status at +0xC.  They are what keeps VI running: when majestic
 * calls XM_MPI_ISP_MemInit the driver stops every channel, and it is the
 * ISP's per-frame handler that starts them again, so without these lines
 * the pipeline stalls after a handful of frames.
 *
 * XM510.  The same blocks at other offsets, with up to three buffers and one
 * back-end interrupt for every channel.  Per channel c, W = 0x81000 + c *
 * 0x1000:
 *
 *   W+0x00/04/08   buffer 0/1/2 luma, (address - 0x80000000) >> 3
 *   W+0x10/14/18   buffer 0/1/2 chroma
 *   W+0x20/+0x28   luma / chroma stride, W+0x24 width, W+0x34 enable
 *   W+0x38         the last buffer: 2 for three, 1 for two (load_xiongmai
 *                  gives an 18M kernel three, anything else two)
 *   W+0x40/+0x44   the control words XM530 has at W+0x00/04
 *   W+0x3C         the buffer the hardware writes next; the driver takes
 *                  the one before it as the finished frame
 *   W+0x48         frame done (1)
 *   W+0x4C         overflow: non-zero makes the handler drop the frame
 *
 * There is no height register: the driver lays chroma directly after luma,
 * so the height is (chroma - luma) / stride.  Interrupt blocks, enable at
 * +0 and status at +0xC as on XM530: 0xF0010 is vi_front_v and isp_irq
 * (IRQ 16), 0xF0030 vi_back_v (IRQ 18), its status a bit per channel.
 *
 * Each frame period a timer raises the enabled ISP lines, then for every
 * enabled channel fills the next of its buffers with a test pattern
 * (moving colour bars over a frame counter), sets its done bit and raises
 * the channel's line (XM530: SPI 33 + c; the driver's vi_back_int takes
 * the channel as dev_id).
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "qemu/timer.h"
#include "hw/qdev-properties.h"
#include "system/dma.h"
#include "qapi/error.h"

#define TYPE_XM_VI "xm-vi"
OBJECT_DECLARE_SIMPLE_TYPE(XmViState, XM_VI)

#define XM_VI_SIZE      0x100000
#define XM_VI_CHANNELS  4
#define XM_VI_ISP_LINES 2
#define XM_VI_IRQ_AREA  0xF0000
#define XM_VI_DDR       0x80000000u

typedef struct XmViLayout {
    const char *soc;
    uint32_t win;               /* channel 0's window; channels 0x1000 apart */
    int nbuf;                   /* buffers, or the most last_off allows */
    uint32_t last_off;          /* 0, or where the driver puts nbuf - 1 */
    uint32_t y_off, c_off;      /* buffer 0's luma / chroma address */
    uint32_t done_off;          /* one done bit per buffer, or */
    bool done_flag;             /* just 1, and next_off says where from */
    uint32_t next_off;          /* the buffer the hardware writes next */
    uint32_t chan_int;          /* channel 0's interrupt block */
    uint32_t chan_int_step;     /* 0: one block, a status bit per channel */
    int isp_lines;
    uint32_t isp_int[XM_VI_ISP_LINES];
} XmViLayout;

static const XmViLayout xm_vi_layouts[] = {
    {
        .soc = "xm530", .win = 0x84000, .nbuf = 2,
        .y_off = 0x40, .c_off = 0xC0, .done_off = 0x08,
        .chan_int = 0xF0080, .chan_int_step = 0x20,
        .isp_lines = 2, .isp_int = { 0xF0040, 0xF0060 },
    }, {
        .soc = "xm510", .win = 0x81000, .nbuf = 3, .last_off = 0x38,
        .y_off = 0x00, .c_off = 0x10, .done_off = 0x48, .done_flag = true,
        .next_off = 0x3C,
        .chan_int = 0xF0030, .chan_int_step = 0,
        .isp_lines = 1, .isp_int = { 0xF0010 },
    },
};

#define XM_VI_WIN(s, c)  ((s)->l->win + (c) * 0x1000)

struct XmViState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[XM_VI_CHANNELS];
    qemu_irq isp_irq[XM_VI_ISP_LINES];
    uint32_t regs[XM_VI_SIZE / 4];
    QEMUTimer *timer;
    uint32_t fps;
    char *soc;
    const XmViLayout *l;
    uint32_t frame[XM_VI_CHANNELS];
};

#define R(s, off) ((s)->regs[(off) / 4])

static bool xm_vi_enabled(XmViState *s, int c)
{
    return R(s, XM_VI_WIN(s, c) + 0x34) & 1;
}

/* Channel c's interrupt block, and the status bit that is its frame. */
static uint32_t xm_vi_int(XmViState *s, int c)
{
    return s->l->chan_int + c * s->l->chan_int_step;
}

static uint32_t xm_vi_int_bit(XmViState *s, int c)
{
    return s->l->chan_int_step ? 1 : 1u << c;
}

/* An interrupt block's line: status pending and the enable bit set, so a
 * masked channel still captures and only the interrupt is held back. */
static void xm_vi_update_irq(XmViState *s)
{
    const int lines = s->l->chan_int_step ? XM_VI_CHANNELS : 1;
    int n;

    for (n = 0; n < lines; n++) {
        const uint32_t blk = xm_vi_int(s, n);
        qemu_set_irq(s->irq[n], R(s, blk + 0x0C) && (R(s, blk) & 1));
    }
    for (n = 0; n < s->l->isp_lines; n++) {
        const uint32_t blk = s->l->isp_int[n];
        qemu_set_irq(s->isp_irq[n], R(s, blk + 0x0C) && (R(s, blk) & 1));
    }
}

/* XM510's load_xiongmai picks two or three buffers by the kernel's mem= */
static int xm_vi_nbuf(XmViState *s, int c)
{
    if (!s->l->last_off) {
        return s->l->nbuf;
    }
    return MIN(R(s, XM_VI_WIN(s, c) + s->l->last_off), s->l->nbuf - 1) + 1;
}

static int xm_vi_height(XmViState *s, int c)
{
    const uint32_t win = XM_VI_WIN(s, c);
    const uint32_t ys = R(s, win + 0x20);

    if (s->l->chan_int_step) {
        return (int)R(s, xm_vi_int(s, c) + 4) - 1;
    }
    /* XM510: chroma follows luma, both in 8-byte units */
    if (!ys || R(s, win + s->l->c_off) <= R(s, win + s->l->y_off)) {
        return 0;
    }
    return MIN((R(s, win + s->l->c_off) - R(s, win + s->l->y_off)) / ys, 4097);
}

/* Moving SMPTE-ish bars, a frame counter in the top-left as binary
 * blocks, so a stuck or repeated frame shows. */
static bool xm_vi_pattern(XmViState *s, int c)
{
    static const uint8_t bars[8][3] = {           /* Y, Cb, Cr */
        { 235, 128, 128 }, { 210, 16, 146 }, { 170, 166, 16 },
        { 145, 54, 34 }, { 106, 202, 222 }, { 81, 90, 240 },
        { 41, 240, 110 }, { 16, 128, 128 },
    };
    const uint32_t win = XM_VI_WIN(s, c);
    const int w = R(s, win + 0x24);
    const int h = xm_vi_height(s, c);
    const int ys = R(s, win + 0x20) * 8, cs = R(s, win + 0x28) * 8;
    const int k = s->frame[c] % xm_vi_nbuf(s, c);
    const uint32_t yr = R(s, win + s->l->y_off + 4 * k);
    const uint32_t cr = R(s, win + s->l->c_off + 4 * k);
    const hwaddr ya = XM_VI_DDR + (hwaddr)yr * 8;
    const hwaddr ca = XM_VI_DDR + (hwaddr)cr * 8;
    const int shift = (s->frame[c] * 4) % MAX(w, 1);
    uint8_t *line;
    int x, y;

    /* an unset buffer is not the bottom of RAM */
    if (w <= 0 || h <= 0 || w > 4096 || h > 4096 || ys < w || cs < w ||
        !yr || !cr) {
        return false;
    }
    line = g_malloc(w);
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            const int bar = ((x + shift) % w) * 8 / w;
            uint8_t v = bars[bar][0];
            /* 16 binary blocks of the frame number, 16 px square */
            if (y < 16 && x < 256) {
                v = (s->frame[c] >> (15 - x / 16)) & 1 ? 235 : 16;
            }
            line[x] = v;
        }
        dma_memory_write(&address_space_memory, ya + (hwaddr)y * ys, line, w,
                         MEMTXATTRS_UNSPECIFIED);
    }
    for (y = 0; y < h / 2; y++) {
        for (x = 0; x < w / 2; x++) {
            const int bar = ((2 * x + shift) % w) * 8 / w;
            const bool blk = y < 8 && 2 * x < 256;
            line[2 * x] = blk ? 128 : bars[bar][1];
            line[2 * x + 1] = blk ? 128 : bars[bar][2];
        }
        dma_memory_write(&address_space_memory, ca + (hwaddr)y * cs, line,
                         w & ~1, MEMTXATTRS_UNSPECIFIED);
    }
    g_free(line);
    return true;
}

static void xm_vi_tick(void *opaque)
{
    XmViState *s = opaque;
    int c;

    for (c = 0; c < s->l->isp_lines; c++) {
        if (R(s, s->l->isp_int[c]) & 1) {
            R(s, s->l->isp_int[c] + 0x0C) = 1;
        }
    }
    for (c = 0; c < XM_VI_CHANNELS; c++) {
        /* a buffer the geometry does not allow is not a frame */
        if (!xm_vi_enabled(s, c) || !xm_vi_pattern(s, c)) {
            continue;
        }
        R(s, XM_VI_WIN(s, c) + s->l->done_off) = s->l->done_flag ? 1 :
            1u << (s->frame[c] % xm_vi_nbuf(s, c));
        if (s->l->next_off) {
            R(s, XM_VI_WIN(s, c) + s->l->next_off) =
                (s->frame[c] + 1) % xm_vi_nbuf(s, c);
        }
        R(s, xm_vi_int(s, c) + 0x0C) |= xm_vi_int_bit(s, c);
        s->frame[c]++;
    }
    xm_vi_update_irq(s);
    timer_mod_ns(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 NANOSECONDS_PER_SECOND / s->fps);
}

static uint64_t xm_vi_read(void *opaque, hwaddr off, unsigned size)
{
    XmViState *s = XM_VI(opaque);

    return s->regs[off / 4];
}

static void xm_vi_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    XmViState *s = XM_VI(opaque);

    qemu_log_mask(LOG_UNIMP, "xm-vi: [0x%05" HWADDR_PRIx "] = 0x%08" PRIx64
                  "\n", off, val);
    s->regs[off / 4] = val;
    if (off >= XM_VI_IRQ_AREA) {
        xm_vi_update_irq(s);
    }
}

static const MemoryRegionOps xm_vi_ops = {
    .read = xm_vi_read,
    .write = xm_vi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_vi_reset(DeviceState *dev)
{
    XmViState *s = XM_VI(dev);
    int n;

    memset(s->regs, 0, sizeof(s->regs));
    for (n = 0; n < XM_VI_CHANNELS; n++) {
        qemu_irq_lower(s->irq[n]);
        s->frame[n] = 0;
    }
    for (n = 0; n < XM_VI_ISP_LINES; n++) {
        qemu_irq_lower(s->isp_irq[n]);
    }
    timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1000);
}

static void xm_vi_init(Object *obj)
{
    XmViState *s = XM_VI(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &xm_vi_ops, s, TYPE_XM_VI,
                          XM_VI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (n = 0; n < XM_VI_CHANNELS; n++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[n]);
    }
    for (n = 0; n < XM_VI_ISP_LINES; n++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->isp_irq[n]);
    }
    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, xm_vi_tick, s);
}

static const Property xm_vi_props[] = {
    DEFINE_PROP_UINT32("fps", XmViState, fps, 10),
    DEFINE_PROP_STRING("soc", XmViState, soc),
};

static void xm_vi_realize(DeviceState *dev, Error **errp)
{
    XmViState *s = XM_VI(dev);
    int i;

    for (i = 0; i < ARRAY_SIZE(xm_vi_layouts); i++) {
        if (!g_strcmp0(s->soc ? s->soc : "xm530", xm_vi_layouts[i].soc)) {
            s->l = &xm_vi_layouts[i];
        }
    }
    if (!s->l) {
        error_setg(errp, "xm-vi: unknown soc '%s'", s->soc);
        return;
    }

    if (s->fps < 1 || s->fps > 1000) {
        error_setg(errp, "xm-vi: fps must be 1..1000, not %u", s->fps);
    }
}

static void xm_vi_class_init(ObjectClass *klass, const void *data)
{
    DEVICE_CLASS(klass)->realize = xm_vi_realize;
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_vi_reset);
    device_class_set_props(DEVICE_CLASS(klass), xm_vi_props);
}

static const TypeInfo xm_vi_info = {
    .name          = TYPE_XM_VI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmViState),
    .instance_init = xm_vi_init,
    .class_init    = xm_vi_class_init,
};

static void xm_vi_register_types(void)
{
    type_register_static(&xm_vi_info);
}

type_init(xm_vi_register_types)
