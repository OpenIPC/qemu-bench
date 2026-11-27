/*
 * Xiongmai XM232 VI + ISP, as isp.ko ("XM210V200_ISP") and vi_mu.ko drive
 * them: one capture path time-multiplexed between the sensors ("vi_mu",
 * loadmod's mode 3: two SC2336 halves spliced into one picture), the BPU
 * and the VPSS online behind it, and a store that writes the VPSS's three
 * outputs (1920x1080, 640x360, 512x288 per sensor here) to memory.
 *
 * One RAM-backed window, 0x30000000 + 68 MiB (isp.ko maps it all), so the
 * drivers' configuration reads back.  Each frame period the VICU takes the
 * sensor isp.ko selected at +0x10008 (it switches every frame):
 *   frame start   that sensor's mux timer: +0x2100 / +0x3100 (sensor 1),
 *                 +0x2108 reading either; isp.ko's mux_timer_irq writes 0
 *   frame end     +0xF006C (vicu_bayer_irq, if +0xF0064 enabled it),
 *                 +0x42044 (bpu_hvgen_irq), the BPU's statistics, the store
 *                 (each channel set up at the last +0x10014 "go": drawn,
 *                 +0x90 done, which vi_mu.ko acknowledges at +0x94), the
 *                 VICU's +0x7104C (polled by vi_mu.ko, written 0), then
 *                 vi_sel: +0x10020 the raw events, +0x10028 them through the
 *                 +0x10024 mask; vi_mu.ko's handler writes +0x10020 0
 * Store channel c at +0x73000 + 0x4000 * c: +0x00 3 on, +0x08 / +0x24 luma
 * / chroma and +0x18 / +0x34 their strides and +0x20 the luma's size, each
 * >> 3, the addresses (address + 0x80000000) of vb.ko's media view of the
 * DDR, 0x90000000 up (the machine's ram_media_alias).
 * The picture is the other Xiongmai VI models' (xm-pattern.c.inc), each
 * sensor's dot at its own phase.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/xm-regtrace.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qapi/error.h"
#include "xm-pattern.c.inc"

#define TYPE_XM_VIMU "xm-vimu"
OBJECT_DECLARE_SIMPLE_TYPE(XmVimuState, XM_VIMU)

#define WIN_SIZE    0x4400000

/* registers, offsets in the window */
#define MIPI(n)         (0x1000 * (n))          /* MIPI receiver n */
#define FRONT(n)        (0x2000 + 0x1000 * (n)) /* sensor n's input */
#define FRONT_LINE      0x11C
#define MUX_ST0         0x100       /* frame start: the driver writes 0 */
#define MUX_ST1         0x104
#define MUX_PEND        0x108       /* read-only: either is set */
#define SEL_GO          0x10014     /* 1: take the store setup */
#define SEL_RAW         0x10020     /* vi_sel events; the driver writes 0 */
#define SEL_EN          0x10024
#define SEL_ST          0x10028     /* read-only: RAW & EN */
#define SEL_SNS         0x10008     /* the sensor the VICU takes next */
#define BPU_ST          0x42044
#define VICU_EN         0xF0064
#define VICU_ST         0xF006C
#define VIC_END         0x7104C     /* polled by vi_mu.ko; written 0 */
#define VIC_LINE        0x71080
#define STORE(c)        (0x73000 + 0x4000 * (c))
#define ST_EN           0x00        /* 3: store this frame */
#define ST_Y            0x08        /* (address - 0x80000000) >> 3 */
#define ST_YS           0x18        /* stride >> 3 */
#define ST_YSIZE        0x20        /* luma bytes >> 3 */
#define ST_C            0x24
#define ST_CS           0x34
#define ST_DONE         0x90        /* frame stored */
#define ST_ACK          0x94        /* 1: clear ST_DONE */
#define CHANNELS        3
/*
 * The BPU's zone statistics, which the isp daemon's AE / AWB / day-night
 * logic runs on: per block, the zone grid (+0x00..+0x80 the 33 column
 * edges, +0xC0..+0x100 the 17 row edges) and a buffer pair (+0x118 start,
 * +0x11C end, (address - 0x80000000) >> 3) the frame's 512 zones go to,
 * 16 bytes each: the R, G (both greens) and B sums, a word unused.  isp.ko
 * divides by a quarter of the zone's pixels.  AWB: one block for whichever
 * sensor is captured, on at +0x110; MD: one per sensor.
 */
#define AWB_STATS       0x4B000
#define MD_STATS(n)     (0x82000 + 0x10000 * (n))
#define ZS_X(i)         (0x00 + 4 * (i))
#define ZS_Y(i)         (0xC0 + 4 * (i))
#define ZS_ON           0x110
#define ZS_BUF          0x118
#define ZONES_X         32
#define ZONES_Y         16
/* the scene the statistics describe: grey, a quarter of full scale */
#define STATS_LEVEL     256     /* of 1023 */

enum { IRQ_VICU, IRQ_BPU, IRQ_SEL, IRQ_MUX, NUM_IRQ };

typedef struct XmVimuStore {
    uint32_t en, y, c, ys, cs, ysize;
} XmVimuStore;

struct XmVimuState {
    SysBusDevice parent_obj;
    MemoryRegion io;
    uint32_t *r;
    unsigned long *seen;
    qemu_irq irq[NUM_IRQ];
    QEMUTimer *timer;
    bool end_phase;
    uint32_t sns;               /* the sensor being captured */
    XmVimuStore shadow[CHANNELS];   /* taken at SEL_GO */
    uint32_t stored[CHANNELS];
    uint32_t fps, sensors, sns_w, sns_h;
    bool trace, trace_reads, trace_writes;
};

#define R(off) s->r[(off) / 4]

static void xm_vimu_update(XmVimuState *s)
{
    bool mux = false;

    for (uint32_t n = 0; n < s->sensors; n++) {
        mux |= R(FRONT(n) + MUX_ST0) || R(FRONT(n) + MUX_ST1);
    }
    qemu_set_irq(s->irq[IRQ_VICU], R(VICU_ST) != 0);
    qemu_set_irq(s->irq[IRQ_BPU], R(BPU_ST) != 0);
    qemu_set_irq(s->irq[IRQ_SEL], (R(SEL_RAW) & R(SEL_EN)) != 0);
    qemu_set_irq(s->irq[IRQ_MUX], mux);
}

static void xm_vimu_store(XmVimuState *s, int c)
{
    const XmVimuStore *t = &s->shadow[c];
    const int ys = t->ys * 8, cs = t->cs * 8;
    const int h = ys ? (int)((uint64_t)t->ysize * 8 / ys) : 0;

    if (s->trace) {
        fprintf(stderr, "xm-vimu: store %d sns %u en %u y %08llx c %08llx "
                "ys %d cs %d h %d\n", c, s->sns, t->en,
                0x80000000ULL + (hwaddr)t->y * 8,
                0x80000000ULL + (hwaddr)t->c * 8, ys, cs, h);
    }
    if (t->en != 3 || !t->y || !t->c) {
        return;
    }
    xm_pattern_draw(0x80000000ULL + (hwaddr)t->y * 8,
                    0x80000000ULL + (hwaddr)t->c * 8, ys, h, ys, cs,
                    s->stored[c]++, s->sns);
    R(STORE(c) + ST_DONE) = 1;
}

static void xm_vimu_stats(XmVimuState *s, uint32_t blk)
{
    static uint32_t z[ZONES_X * ZONES_Y * 4];
    const uint32_t buf = R(blk + ZS_BUF);

    if (!buf) {
        return;
    }
    for (int y = 0; y < ZONES_Y; y++) {
        const uint32_t h = R(blk + ZS_Y(y + 1)) - R(blk + ZS_Y(y));

        for (int x = 0; x < ZONES_X; x++) {
            const uint32_t w = R(blk + ZS_X(x + 1)) - R(blk + ZS_X(x));
            const uint32_t n = w * h / 4, i = (y * ZONES_X + x) * 4;

            z[i] = cpu_to_le32(n * STATS_LEVEL);
            z[i + 1] = cpu_to_le32(2 * n * STATS_LEVEL);
            z[i + 2] = cpu_to_le32(n * STATS_LEVEL);
            z[i + 3] = 0;
        }
    }
    dma_memory_write(&address_space_memory, 0x80000000ULL + (hwaddr)buf * 8,
                     z, sizeof(z), MEMTXATTRS_UNSPECIFIED);
}

/*
 * Each period the VICU takes one sensor's frame (SEL_SNS, which isp.ko
 * switches every frame), the BPU and VPSS process it online and the store
 * writes the VPSS outputs to the buffers set up at the last SEL_GO.
 * Frame start: the sensor's mux timer; half a period later the end: VICU,
 * BPU, store done, then vi_sel.
 */
static void xm_vimu_frame(void *opaque)
{
    XmVimuState *s = XM_VIMU(opaque);

    for (uint32_t n = 0; n < s->sensors; n++) {
        const uint32_t w = s->sns_w, h = s->sns_h;

        R(MIPI(n) + 0x40) = w;
        R(MIPI(n) + 0x44) = h;
        R(MIPI(n) + 0x48) = w;
        R(MIPI(n) + 0x4C) = h;
        R(MIPI(n) + 0x10C) = w;
        R(MIPI(n) + 0x110) = h;
        R(MIPI(n) + 0x114) = w;
        R(MIPI(n) + 0x118) = h;
        R(FRONT(n) + FRONT_LINE) = h;
    }
    if (!s->end_phase) {
        s->sns = R(SEL_SNS) < s->sensors ? R(SEL_SNS) : 0;
        if (R(FRONT(s->sns) + 0x08)) {
            R(FRONT(s->sns) + MUX_ST0) = 1;
        }
    } else {
        if (R(VICU_EN)) {
            R(VICU_ST) = 1;
        }
        R(BPU_ST) = 1;
        if (R(AWB_STATS + ZS_ON) & 1) {
            xm_vimu_stats(s, AWB_STATS);
        }
        xm_vimu_stats(s, MD_STATS(s->sns));
        for (int c = 0; c < CHANNELS; c++) {
            xm_vimu_store(s, c);
        }
        R(VIC_END) = 1;
        R(VIC_LINE) = s->sns_h;
        R(SEL_RAW) |= 1;
    }
    s->end_phase = !s->end_phase;
    xm_vimu_update(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / (s->fps * s->sensors) / 2);
}

static uint64_t xm_vimu_read(void *opaque, hwaddr off, unsigned size)
{
    XmVimuState *s = XM_VIMU(opaque);
    uint32_t v = s->r[off / 4];

    if (off == SEL_ST) {
        v = R(SEL_RAW) & R(SEL_EN);
    } else if (off >= FRONT(0) && off < FRONT(s->sensors) &&
               (off & 0xFFF) == MUX_PEND) {
        v = R((off & ~0xFFF) + MUX_ST0) || R((off & ~0xFFF) + MUX_ST1);
    }

    if (s->trace_reads && !test_and_set_bit(off / 4, s->seen)) {
        fprintf(stderr, "xm-vimu: R %07" HWADDR_PRIx " = %08x\n", off, v);
    }
    return v;
}

static void xm_vimu_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    XmVimuState *s = XM_VIMU(opaque);

    xm_regtrace("vimu", off, val);
    if (s->trace_writes) {
        fprintf(stderr, "xm-vimu: W %07" HWADDR_PRIx " %08" PRIx64 "\n",
                off, val);
    }
    s->r[off / 4] = val;
    if (off == SEL_GO && val) {
        for (int c = 0; c < CHANNELS; c++) {
            XmVimuStore *t = &s->shadow[c];

            t->en = R(STORE(c) + ST_EN);
            t->y = R(STORE(c) + ST_Y);
            t->c = R(STORE(c) + ST_C);
            t->ys = R(STORE(c) + ST_YS);
            t->cs = R(STORE(c) + ST_CS);
            t->ysize = R(STORE(c) + ST_YSIZE);
        }
    }
    for (int c = 0; c < CHANNELS; c++) {
        if (off == STORE(c) + ST_ACK && val) {
            R(STORE(c) + ST_DONE) = 0;
        }
    }
    xm_vimu_update(s);
}

static const MemoryRegionOps xm_vimu_ops = {
    .read = xm_vimu_read,
    .write = xm_vimu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_vimu_reset(DeviceState *dev)
{
    XmVimuState *s = XM_VIMU(dev);

    memset(s->r, 0, WIN_SIZE);
    memset(s->shadow, 0, sizeof(s->shadow));
    s->end_phase = false;
    s->sns = 0;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / s->fps);
    xm_vimu_update(s);
}

static void xm_vimu_realize(DeviceState *dev, Error **errp)
{
    XmVimuState *s = XM_VIMU(dev);

    if (!s->fps || !s->sensors || s->sensors > 2) {
        error_setg(errp, "xm-vimu: fps must not be 0, sensors 1 or 2");
    }
}

static void xm_vimu_init(Object *obj)
{
    XmVimuState *s = XM_VIMU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->r = g_malloc0(WIN_SIZE);
    s->seen = bitmap_new(WIN_SIZE / 4);
    memory_region_init_io(&s->io, obj, &xm_vimu_ops, s, "xm-vimu", WIN_SIZE);
    sysbus_init_mmio(sbd, &s->io);
    for (int n = 0; n < NUM_IRQ; n++) {
        sysbus_init_irq(sbd, &s->irq[n]);
    }
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_vimu_frame, s);
}

static const Property xm_vimu_props[] = {
    DEFINE_PROP_UINT32("fps", XmVimuState, fps, 12),
    DEFINE_PROP_UINT32("sensors", XmVimuState, sensors, 2),
    DEFINE_PROP_UINT32("sensor-width", XmVimuState, sns_w, 1920),
    DEFINE_PROP_UINT32("sensor-lines", XmVimuState, sns_h, 1080),
    DEFINE_PROP_BOOL("trace", XmVimuState, trace, false),
    DEFINE_PROP_BOOL("trace-reads", XmVimuState, trace_reads, false),
    DEFINE_PROP_BOOL("trace-writes", XmVimuState, trace_writes, false),
};

static void xm_vimu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, xm_vimu_reset);
    dc->realize = xm_vimu_realize;
    device_class_set_props(dc, xm_vimu_props);
}

static const TypeInfo xm_vimu_info = {
    .name          = TYPE_XM_VIMU,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmVimuState),
    .instance_init = xm_vimu_init,
    .class_init    = xm_vimu_class_init,
};

static void xm_vimu_register_types(void)
{
    type_register_static(&xm_vimu_info);
}

type_init(xm_vimu_register_types)
