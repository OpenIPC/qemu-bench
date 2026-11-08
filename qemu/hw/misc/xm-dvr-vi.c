/*
 * Xiongmai DVR video input (xm_vi.ko, 0x30000000, 2 MiB): the capture side
 * of the AHD receivers' BT.656 ports, one store engine per channel.
 *
 * XM8521DV200 / XM8536E (layout 0).  Channel c, 0..8:
 *   store block S = 0x20000 + (c & 3) * 0x4000 + (c >> 2) * 0x1000
 *     +0x00 / +0x10   Y / C start, (phys - 0x80000000) >> 4 (NV12)
 *     +0x20           stride >> 4
 *     +0x34           1: store armed for the next frame
 *     +0x58           luma size >> 4
 *     +0x200 / +0x208 read as different values: the frame is valid
 *   control block P = 0x100000 + c * 0x10000
 *     +0x6008         reads non-zero: signal present
 *     +0x604C         A, "frame done": set at the end of a frame
 *     +0x605C         B, "arm next": set 3/4 into a frame; the handler
 *                     programs the store block and sets +0x34
 *     +0x6050         interrupt enable; A and B clear by writing 0
 * The driver rotates its buffers in software (next / now / last) and
 * clears +0x34 after A, so the engine latches the armed set at the frame
 * boundary: a frame is stored into the set latched one boundary earlier.
 * Channels 0..3 raise the first interrupt (IRQ 69), 4..8 the second (70).
 *
 * XM8520D (layout 1, its older xm_vi.ko), 20 channels: store block at
 * 0x80000 + (c & 3) * 0x8000 + (c >> 2) * 0x1000 (same fields), status at
 * 0x113000 + (c & 3) * 0x40000 + (c >> 2) * 0x4000 (+0x4C A, +0x5C B,
 * +0x50 enable); no signal-present word.  Channels 0..9 / 10..19 per line.
 *
 * BT.656 front end, lane L (A, B) at 0x12000 + L * 0x200, set up by Sofia
 * through /dev/mem: it sweeps each channel's sampling delay (DLL) 0..127 in
 * +0x20 + 4 * chn and
 * takes the middle of the first run of delays, more than 3 long and ended
 * by one that fails, at which +0x44 + 0x40 * chn reads 15 (all four sync
 * codes seen) and +0x150 + 0x20 * chn [31:16] a plausible line length (720,
 * 1080, 1950, 1440 or 288 clocks).  None found, it reboots the DVR ("can not find dll").  Here
 * delays 32..63 lock, at 1950 (two multiplexed 960-wide channels).
 *
 * XM8520D: engine e = c & 3 takes lane e >> 1, time slot e & 1 of that
 * lane's 2-mux; c >> 2 picks one of the engine's five outputs (main, sub,
 * ...) of the same camera.  Which camera that is, the AHD receiver's port
 * mux says ("ahd" link, "vports": its port on each lane, board wiring);
 * the pictures carry that camera's number.
 *
 * Every other word is RAM-backed.  Read from the vendor modules (no public
 * source).  The pictures are generated: colour bars, the channel's number
 * and a bar that moves a step a frame, a camera on every channel.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "system/address-spaces.h"
#include "qom/object.h"
#include "hw/arm/xiongmai.h"

#define TYPE_XM_DVR_VI "xm-dvr-vi"
OBJECT_DECLARE_SIMPLE_TYPE(XmDvrViState, XM_DVR_VI)

#define VI_SIZE         (2 * MiB)
#define VI_MAX_CHN      20
#define VI_FRAME_NS     (NANOSECONDS_PER_SECOND / 25)
#define VI_MAX_W        4096
#define VI_MAX_H        2304
#define DDR_BASE        0x80000000ULL

typedef struct XmViLatch {
    bool armed;
    hwaddr y, c;
    uint32_t stride, h;
} XmViLatch;

struct XmDvrViState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[2];
    QEMUTimer *timer;
    bool arm_phase;             /* next tick: the 3/4 point (B), else the end */
    uint32_t *regs;
    uint32_t layout, chans;
    Object *ahd;                /* layout 1: the AHD receiver feeding it */
    uint32_t vports;            /* its port on lane A [3:0], B [7:4] */
    XmViLatch latch[VI_MAX_CHN];
    uint64_t frame;
    uint8_t *pic;               /* one generated picture, re-used per channel */
    uint32_t pic_w, pic_h;
    int pic_chn;
    uint32_t dll[2][4];         /* each lane's channels' delays */
};

#define R(o) (s->regs[(o) / 4])

static hwaddr vi_store(XmDvrViState *s, int c)
{
    return s->layout ? 0x80000 + (c & 3) * 0x8000 + (c >> 2) * 0x1000
                     : 0x20000 + (c & 3) * 0x4000 + (c >> 2) * 0x1000;
}

/* the block holding A (+0x4C), enable (+0x50), B (+0x5C) */
static hwaddr vi_status(XmDvrViState *s, int c)
{
    return s->layout ? 0x113000 + (c & 3) * 0x40000 + (c >> 2) * 0x4000
                     : 0x106000 + c * 0x10000;
}

static void xm_dvr_vi_update(XmDvrViState *s)
{
    int c, half = s->layout ? 10 : 4;
    bool lv[2] = { false, false };

    for (c = 0; c < s->chans; c++) {
        hwaddr p = vi_status(s, c);

        if (R(p + 0x50) && (R(p + 0x4C) || R(p + 0x5C))) {
            lv[c >= half] = true;
        }
    }
    qemu_set_irq(s->irq[0], lv[0]);
    qemu_set_irq(s->irq[1], lv[1]);
}

/* 7-row bitmaps, 5 columns, of the digits */
static const uint8_t vi_font[10][7] = {
    { 14, 17, 19, 21, 25, 17, 14 }, { 4, 12, 4, 4, 4, 4, 14 },
    { 14, 17, 1, 2, 4, 8, 31 },     { 31, 2, 4, 2, 1, 17, 14 },
    { 2, 6, 10, 18, 31, 2, 2 },     { 31, 16, 30, 1, 1, 17, 14 },
    { 6, 8, 16, 30, 17, 17, 14 },   { 31, 1, 2, 4, 8, 8, 8 },
    { 14, 17, 17, 14, 17, 17, 14 }, { 14, 17, 17, 15, 1, 2, 12 },
};

static void vi_box(uint8_t *y, uint8_t *uv, uint32_t w, uint32_t h,
                   uint32_t x0, uint32_t y0, uint32_t bw, uint32_t bh,
                   uint8_t Y, uint8_t U, uint8_t V)
{
    uint32_t i, j;

    for (j = y0; j < y0 + bh && j < h; j++) {
        for (i = x0; i < x0 + bw && i < w; i++) {
            y[j * w + i] = Y;
            if (!(j & 1) && !(i & 1)) {
                uv[(j / 2) * w + i] = U;
                uv[(j / 2) * w + i + 1] = V;
            }
        }
    }
}

/* Colour bars with the channel number (1-based) large in the middle */
static void xm_dvr_vi_make(XmDvrViState *s, int chn, uint32_t w, uint32_t h)
{
    static const uint8_t bars[8][3] = {
        { 235, 128, 128 }, { 210, 16, 146 }, { 170, 166, 16 }, { 145, 54, 34 },
        { 106, 202, 222 }, { 81, 90, 240 }, { 41, 240, 110 }, { 16, 128, 128 },
    };
    uint8_t *y, *uv;
    uint32_t i, sc, n = chn + 1, dx, nd = n >= 10 ? 2 : 1, d;

    if (s->pic && s->pic_w == w && s->pic_h == h && s->pic_chn == chn) {
        return;
    }
    g_free(s->pic);
    s->pic = g_malloc(w * h * 3 / 2);
    s->pic_w = w;
    s->pic_h = h;
    s->pic_chn = chn;
    y = s->pic;
    uv = s->pic + w * h;
    for (i = 0; i < 8; i++) {
        vi_box(y, uv, w, h, i * w / 8, 0, w / 8 + 1, h, bars[i][0],
               bars[i][1], bars[i][2]);
    }
    sc = h / 12;
    dx = (w - nd * 6 * sc) / 2;
    vi_box(y, uv, w, h, dx - sc, (h - 9 * sc) / 2, (nd * 6 + 1) * sc, 9 * sc,
           16, 128, 128);
    for (d = 0; d < nd; d++) {
        const uint8_t *g = vi_font[nd == 2 && d == 0 ? n / 10 : n % 10];
        uint32_t r, col;

        for (r = 0; r < 7; r++) {
            for (col = 0; col < 5; col++) {
                if (g[r] & (16 >> col)) {
                    vi_box(y, uv, w, h, dx + (d * 6 + col) * sc,
                           (h - 7 * sc) / 2 + r * sc, sc, sc, 235, 128, 128);
                }
            }
        }
    }
}

/* the camera (0-based) a channel's pictures are numbered with */
static int xm_dvr_vi_camera(XmDvrViState *s, int c)
{
    int e = c & 3, vport, cam;

    if (!s->layout || !s->ahd) {
        return c;
    }
    vport = s->vports >> (4 * (e >> 1)) & 0xF;
    cam = vport < 4 ? XM_AHD_GET_CLASS(s->ahd)->camera(s->ahd, vport, e & 1)
                    : -1;
    return cam >= 0 ? cam : e;
}

static void xm_dvr_vi_store(XmDvrViState *s, int c, XmViLatch *l)
{
    uint32_t w = l->stride, h = l->h, bar, j;

    if (!w || !h || w > VI_MAX_W || h > VI_MAX_H) {
        return;
    }
    xm_dvr_vi_make(s, xm_dvr_vi_camera(s, c), w, h);
    address_space_write(&address_space_memory, l->y, MEMTXATTRS_UNSPECIFIED,
                        s->pic, w * h);
    address_space_write(&address_space_memory, l->c, MEMTXATTRS_UNSPECIFIED,
                        s->pic + w * h, w * h / 2);
    /* a white bar sweeping down, so motion is visible */
    bar = (s->frame * 8) % (h - h / 32);
    for (j = bar; j < bar + h / 32; j++) {
        static uint8_t white[VI_MAX_W];

        if (!white[0]) {
            memset(white, 235, sizeof(white));
        }
        address_space_write(&address_space_memory, l->y + (hwaddr)j * w,
                            MEMTXATTRS_UNSPECIFIED, white, w);
    }
}

static void xm_dvr_vi_tick(void *opaque)
{
    XmDvrViState *s = opaque;
    int c;

    for (c = 0; c < s->chans; c++) {
        hwaddr p = vi_status(s, c), st = vi_store(s, c);

        if (!R(p + 0x50)) {
            continue;
        }
        if (s->arm_phase) {
            R(p + 0x5C) = 1;
            continue;
        }
        /* the end of a frame: store it, latch what B armed, report */
        if (s->latch[c].armed) {
            xm_dvr_vi_store(s, c, &s->latch[c]);
        }
        s->latch[c].armed = R(st + 0x34) != 0;
        s->latch[c].y = DDR_BASE + ((hwaddr)R(st + 0x00) << 4);
        s->latch[c].c = DDR_BASE + ((hwaddr)R(st + 0x10) << 4);
        s->latch[c].stride = R(st + 0x20) << 4;
        /* v100 never writes +0x58: the height is (C - Y) / stride */
        if (s->layout) {
            s->latch[c].h = R(st + 0x20) && R(st + 0x10) > R(st) ?
                            (R(st + 0x10) - R(st)) / R(st + 0x20) : 0;
        } else {
            s->latch[c].h = s->latch[c].stride ?
                            (R(st + 0x58) << 4) / s->latch[c].stride : 0;
        }
        R(p + 0x4C) = 1;
    }
    if (!s->arm_phase) {
        s->frame++;
    }
    xm_dvr_vi_update(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              (s->arm_phase ? VI_FRAME_NS / 4 : VI_FRAME_NS * 3 / 4));
    s->arm_phase = !s->arm_phase;
}

static uint64_t xm_dvr_vi_read(void *opaque, hwaddr off, unsigned size)
{
    XmDvrViState *s = opaque;
    int c;

    /* front end, lane L, channel n: status +0x44 + 0x40 n, line length
     * +0x150 + 0x20 n, both following that channel's delay */
    if (off >= 0x12000 && off < 0x12400) {
        hwaddr lo = (off - 0x12000) & 0x1ff;
        int lane = (off - 0x12000) >> 9, n = -1;
        bool len = false;

        if (lo >= 0x44 && lo < 0x144 && ((lo - 0x44) & 0x3f) == 0) {
            n = (lo - 0x44) >> 6;
        } else if (lo >= 0x150 && lo < 0x1d0 && !((lo - 0x150) & 0x1f)) {
            n = (lo - 0x150) >> 5;
            len = true;
        }
        if (n >= 0) {
            uint32_t dll = s->dll[lane][n];
            bool lock = dll >= 32 && dll < 64;

            return !lock ? 0 : len ? 1950u << 16 : 15;
        }
    }
    for (c = 0; c < s->chans; c++) {
        hwaddr st = vi_store(s, c);

        if (off == st + 0x200) {
            return 0;
        }
        if (off == st + 0x208) {
            return 1;
        }
        if (!s->layout && off == 0x106008 + c * 0x10000) {
            return 1;
        }
    }
    return R(off);
}

static void xm_dvr_vi_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    XmDvrViState *s = opaque;

    R(off) = val;
    /* a channel's delay: +0x20 + 4 * chn, in either lane */
    if ((off >= 0x12020 && off < 0x12030) || (off >= 0x12220 && off < 0x12230)) {
        s->dll[off >= 0x12200][(off >> 2) & 3] = val;
    }
    xm_dvr_vi_update(s);
}

static const MemoryRegionOps xm_dvr_vi_ops = {
    .read = xm_dvr_vi_read,
    .write = xm_dvr_vi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void xm_dvr_vi_reset(DeviceState *dev)
{
    XmDvrViState *s = XM_DVR_VI(dev);

    memset(s->regs, 0, VI_SIZE);
    memset(s->latch, 0, sizeof(s->latch));
    s->arm_phase = true;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              VI_FRAME_NS * 3 / 4);
}

static void xm_dvr_vi_realize(DeviceState *dev, Error **errp)
{
    XmDvrViState *s = XM_DVR_VI(dev);

    s->chans = s->layout ? 20 : 9;
    s->regs = g_malloc0(VI_SIZE);
    s->pic_chn = -1;
    memory_region_init_io(&s->iomem, OBJECT(dev), &xm_dvr_vi_ops, s,
                          TYPE_XM_DVR_VI, VI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_dvr_vi_tick, s);
}

static void xm_dvr_vi_init(Object *obj)
{
    XmDvrViState *s = XM_DVR_VI(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[0]);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[1]);
}

static const Property xm_dvr_vi_props[] = {
    DEFINE_PROP_UINT32("layout", XmDvrViState, layout, 0),
    DEFINE_PROP_UINT32("vports", XmDvrViState, vports, 0xFF),
    DEFINE_PROP_LINK("ahd", XmDvrViState, ahd, TYPE_XM_AHD, Object *),
};

static void xm_dvr_vi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = xm_dvr_vi_realize;
    device_class_set_legacy_reset(dc, xm_dvr_vi_reset);
    device_class_set_props(dc, xm_dvr_vi_props);
}

static const TypeInfo xm_dvr_vi_info = {
    .name = TYPE_XM_DVR_VI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmDvrViState),
    .instance_init = xm_dvr_vi_init,
    .class_init = xm_dvr_vi_class_init,
};

static const TypeInfo xm_ahd_info = {
    .name = TYPE_XM_AHD,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(XmAhdClass),
};

static void xm_dvr_vi_register(void)
{
    type_register_static(&xm_ahd_info);
    type_register_static(&xm_dvr_vi_info);
}

type_init(xm_dvr_vi_register)
