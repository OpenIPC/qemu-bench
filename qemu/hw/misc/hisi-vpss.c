/*
 * HiSilicon V4 VPSS (Hi3516EV200 family), VPSS-online behind VI online
 * or offline: the frame source of the video pipeline.
 *
 * In online mode the sensor's frames stream from VI through the ISP into
 * VPSS, which scales them into up to three output channels in DDR.  The
 * driver (hi_vpss.o, VPSS_COMM_OnLineIrqProc) works per frame start:
 * it hands the frame VPSS has just written to the bound modules (VENC)
 * and programs the next frame's buffers, latched by REGUP.  This model
 * produces the frames: on every frame of VI's (the "vi-frame" GPIO, from
 * hisi-vi) it paints the scene in front of the camera (hisi-scene, as the
 * ISP renders it with the frame's exposure and white balance) into every
 * enabled channel's buffers and raises frame start and end.
 *
 *   0x0000  CTRL      bit 31 online video enable (VPSS_HAL_OnLineVideoEn)
 *   0x0010  REGUP     writing bit 0 has VPSS's next frame take the
 *                     configuration as it then stands (the model's
 *                     "active" copy); without it, frames keep the last
 *   0x00f0  INTMASK   0x7400 online (bit 10 frame start), 0x7401 + EOF
 *   0x0310  RAWINT    write 1s to clear: 0 end of frame, 10 frame start
 *   0x0314  INTSTAT   RAWINT & INTMASK
 *   channel 0 / 1 / 2:
 *     size   0x44c / 0x4a0 / 0x4e8   (width - 1) | (height - 1) << 16
 *     ctrl   0x450 / 0x4a4 / 0x4ec   bit 31 enable: whether this frame
 *                                     is written (the driver's frame-rate
 *                                     control toggles it per frame)
 *     addr   0x468 / 0x4b0 / 0x4f4   four words, 8 bytes apart: headers
 *            (compressed output), then luma and chroma
 *     stride 0x488 / 0x4d0 / 0x514   luma | chroma << 16
 *   0x0700  channel 0's compression (VPSS_HAL_SetChnCmpAttr), bit 0 on
 *
 * Behind VI offline (a sensor wider than VPSS's 2304-pixel line buffer)
 * VPSS is node-driven instead: per frame the driver writes 0x304 (bit 0
 * starts the node list) and PNEXT (0x2f8, the first node), then starts
 * VI's pipe, which is when VPSS loads the list ("vi-frame").  The nodes
 * are register images in DDR, in the layout above, chained through their
 * own PNEXT, one per vertical strip of a wide picture; no VPSS interrupt
 * is used.
 *
 * A compressed channel's buffers are smaller than a linear picture and
 * the real format (headers at the first two addresses) is not modelled.
 * Instead VPSS and hisi-vedu share a stand-in: every other row, luma at
 * the luma address and chroma at the chroma address, the width as the
 * stride; hisi-vedu doubles the rows back when it encodes a picture whose
 * register image carries compression headers.
 *
 * (VPSS_HAL_SetChnImgSize / SetChnEn / SetChnPicAddr / SetMask /
 * SetRegup / GetIntStatus / ClrStartInt in the SDK's hi_vpss.o.)
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/misc/hisi-scene.h"

#define TYPE_HISI_VPSS "hisi-vpss"
OBJECT_DECLARE_SIMPLE_TYPE(HisiVpssState, HISI_VPSS)

#define VPSS_REG_SIZE       0x10000
#define VPSS_CTRL           0x0000  /* bit 31 online video enable */
#define VPSS_REGUP          0x0010
#define VPSS_INTMASK        0x00f0
#define VPSS_RAWINT         0x0310
#define VPSS_INTSTAT        0x0314
#define VPSS_CHN0_CMP       0x0700  /* bit 0: channel 0 writes compressed */
#define VPSS_PNEXT          0x02f8  /* node mode: the first node */
#define VPSS_NODE_START     0x0304  /* node mode: bit 0 starts the list */
#define V5_VPSS_HEAD        0x0368  /* V5 node mode: the first node (V4:
                                     * 0x2f8); nodes still chain at 0x2f8 */
#define VPSS_NODE_MAX       8       /* strips of one picture */
#define VPSS_NODE_SIZE      0x800   /* what of a node the model reads */

#define VPSS_INT_EOF        (1u << 0)
#define VPSS_INT_START      (1u << 10)
/*
 * The early ("delay") interrupt, at the line in 0x4[31:16]: a group whose
 * frame interrupt is EARLY / EARLY_END (the SDK's sample_venc) unmasks it
 * (0xf0 = 0x7fd9, START alone 0x7ed9), and its driver then prepares the
 * next frame there -- buffers, channel enables, REGUP -- not at START.
 * That driver also hands the frame's channels on only at an EOF that
 * comes on its own, without START (mod_vpss.o vpss_online_irq_proc,
 * `(raw & 0x401) == 1`), so in that mode they are raised apart, as
 * silicon does (vpss_phase).
 */
#define VPSS_INT_DELAY      (1u << 8)
/*
 * V5 low delay (`ot_low_delay_info`, the SDK sample's channel 1): 0x400 bit
 * 4 + chn turns it on, 0x4b4 holds the line count (chn 0 low half), and
 * 0x4a8 + 4 * chn the frame's tunnel word in DDR, whose u16 at +0 (luma)
 * and +32 (chroma) count the lines written so far -- VENC follows them, and
 * the driver checks them against the height at EOF.  The channel's tunnel
 * interrupt (0x1000 << chn) fires at the line count; the driver counts
 * START up and the tunnel interrupt down per channel and hands the frame
 * on only while the count is 1 (vpss_get_tunl_int, vpss_online_irq_proc).
 * Channel 2 has no low delay: the HAL's enable and line-count setters
 * (vpss_hal_enable_chn_low_delay, vpss_hal_set_chn_low_delay_attr) take
 * channels 0 and 1 only.
 */
#define V5_VPSS_LOWDELAY    0x0400
#define V5_VPSS_TUNL_ADDR   0x04a8
#define VPSS_INT_TUNL(c)    (0x1000u << (c))
#define VPSS_LD_CHN         2

#define VPSS_CHN            3
#define PIC_MAX             4096

/*
 * A channel's output registers.  Its address block holds the header Y,
 * header C, Y and C addresses: V4 spaces them 8 bytes apart (Y is word
 * 4, C word 6), V5 (Hi3516CV610, vpss_drv_update_chn_addr) packs them
 * (Y word 2, C word 3).
 */
typedef struct VpssChnRegs {
    hwaddr size, ctrl, addr, stride;
} VpssChnRegs;

static const VpssChnRegs vpss_chn_v4[VPSS_CHN] = {
    { 0x44c, 0x450, 0x468, 0x488 },
    { 0x4a0, 0x4a4, 0x4b0, 0x4d0 },
    { 0x4e8, 0x4ec, 0x4f4, 0x514 },
};

static const VpssChnRegs vpss_chn_v5[VPSS_CHN] = {
    { 0x428, 0x42c, 0x444, 0x454 },
    { 0x458, 0x45c, 0x468, 0x478 },
    { 0x47c, 0x480, 0x488, 0x498 },
};

/* one channel's output of a frame */
typedef struct VpssOut {
    uint32_t ya, ca;
    int stride, w, h;
    bool cmp;
} VpssOut;

struct HisiVpssState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq running;           /* to hisi-vi: the online pipeline runs */
    QEMUTimer *timer;
    QEMUTimer *phase_timer;     /* the frame's later interrupts */
    unsigned phase;             /* which of them is next (VPSS_PH_*) */
    uint32_t tunl_addr[VPSS_LD_CHN];     /* this frame's low-delay channels */
    uint32_t tunl_h[VPSS_LD_CHN];        /* ... and their heights */
    uint32_t regs[VPSS_REG_SIZE / 4];
    uint32_t active[VPSS_REG_SIZE / 4];  /* what frames are written with */
    bool regup;                          /* REGUP written since the last */
    bool node_armed;                     /* node mode: VPSS waits for VI */
    VpssOut node_out[VPSS_CHN];          /* ... and what it then loaded */
    uint32_t raw;
    uint32_t frame;
    uint32_t fps;               /* VI's frame rate, until it is measured */
    int64_t vi_ns;              /* VI's last frame */
    int64_t period_ns;          /* ... and the interval before it */
    bool trace;
    uint8_t gen;                /* 4: Hi3516EV200 family, 5: Hi3516CV610 */
    HisiSceneState *scene;      /* what the pictures show */
};

static void vpss_update_irq(HisiVpssState *s)
{
    qemu_set_irq(s->irq, (s->raw & s->regs[VPSS_INTMASK / 4]) != 0);
}

/* the online video is enabled and its frame interrupt unmasked */
static bool vpss_running(HisiVpssState *s)
{
    return (s->regs[VPSS_CTRL / 4] & (1u << 31)) &&
           (s->regs[VPSS_INTMASK / 4] & VPSS_INT_START);
}

/*
 * The picture, NV21: the scene (hisi-scene) as the ISP renders it with
 * the frame's exposure and white balance, and the frame number as 16
 * binary squares along the top twelfth (the stream checks read it).  Rows
 * of luma are written while they fit before the chroma plane (a
 * compressed channel's buffer is smaller than a linear picture).
 */
static void vpss_paint(HisiVpssState *s, uint32_t ya, uint32_t ca,
                       int stride, int w, int h, uint32_t n, bool half)
{
    const int sq = MAX(w / 16, 1), step = half ? 2 : 1;
    const float *grid = s->scene ? hisi_scene_grid(s->scene, w, h) : NULL;
    g_autofree uint8_t *line = g_malloc(MAX(stride, w) + 1);
    g_autofree uint8_t *vu = g_malloc(w + 2);
    int r, x;

    for (r = 0; r < h; r += step) {
        if (r < h / 12) {
            for (x = 0; x < w; x++) {
                line[x] = (n >> (15 - MIN(x / sq, 15))) & 1 ? 235 : 16;
            }
        } else if (grid) {
            hisi_scene_display_row(s->scene, grid + (size_t)r * w * 3, w,
                                   line, NULL);
        } else {
            memset(line, 126, w);
        }
        dma_memory_write(&address_space_memory,
                         ya + (uint64_t)(r / step) * stride, line, w,
                         MEMTXATTRS_UNSPECIFIED);
    }
    for (r = 0; r < h / 2; r += step) {
        if (2 * r < h / 12 || !grid) {
            memset(vu, 128, w & ~1);
        } else {
            hisi_scene_display_row(s->scene, grid + (size_t)2 * r * w * 3,
                                   w & ~1, line, vu);
        }
        dma_memory_write(&address_space_memory,
                         ca + (uint64_t)(r / step) * stride, vu, w & ~1,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

/*
 * A channel's output as a register file (the registers, or a node in
 * DDR) sets it; w is 0 when the channel writes nothing this frame (off,
 * or skipped by the driver's frame-rate control).
 */
static void vpss_chn_out(HisiVpssState *s, const uint32_t *r, int c,
                         VpssOut *o)
{
    const VpssChnRegs *ch = s->gen == 5 ? &vpss_chn_v5[c] : &vpss_chn_v4[c];
    const int yw = s->gen == 5 ? 2 : 4, cw = s->gen == 5 ? 3 : 6;
    const uint32_t size = r[ch->size / 4];

    o->w = (size & 0xffff) + 1;
    o->h = (size >> 16) + 1;
    o->ya = r[ch->addr / 4 + yw];
    o->ca = r[ch->addr / 4 + cw];
    o->cmp = c == 0 && (r[VPSS_CHN0_CMP / 4] & 1);
    o->stride = o->cmp ? o->w : r[ch->stride / 4] & 0xffff;
    if (!(r[ch->ctrl / 4] & (1u << 31)) || !o->ya || !o->ca ||
        o->w > PIC_MAX || o->h > PIC_MAX) {
        o->w = 0;
    }
}

/*
 * Node mode: the driver's node list for the frame.  Each node is a
 * register image in DDR (the MMIO layout), chained through its own PNEXT;
 * a picture wider than VPSS's line buffer comes as vertical strips, one
 * node each, that write the same buffers side by side.  The picture each
 * channel gets is the strips' widths together, at the first node's
 * addresses.
 */
static void vpss_load_nodes(HisiVpssState *s)
{
    uint32_t node[VPSS_NODE_SIZE / 4];
    uint32_t a = s->regs[(s->gen == 5 ? V5_VPSS_HEAD : VPSS_PNEXT) / 4];
    int c, k;

    memset(s->node_out, 0, sizeof(s->node_out));
    for (k = 0; k < VPSS_NODE_MAX && a; k++) {
        dma_memory_read(&address_space_memory, a, node, sizeof(node),
                        MEMTXATTRS_UNSPECIFIED);
        for (c = 0; c < VPSS_CHN; c++) {
            VpssOut o;

            vpss_chn_out(s, node, c, &o);
            if (k == 0) {
                s->node_out[c] = o;
            } else if (o.w && s->node_out[c].w) {
                s->node_out[c].w += o.w;
            }
        }
        a = node[VPSS_PNEXT / 4];
    }
    for (c = 0; c < VPSS_CHN; c++) {
        if (s->node_out[c].cmp) {
            s->node_out[c].stride = s->node_out[c].w;   /* the stand-in's */
        }
    }
}

/*
 * A frame with low-delay channels or the early interrupt has its
 * interrupts apart, an eighth of a frame each: START, the tunnel
 * interrupts, the early line, EOF.
 */
enum { VPSS_PH_TUNL = 1, VPSS_PH_DELAY, VPSS_PH_EOF };

/*
 * VI's frame interval as measured, so that a frame's interrupts all fall
 * before the next one whatever the sensor's timing; "fps" until then.
 */
static int64_t vpss_period(HisiVpssState *s)
{
    return s->period_ns ? s->period_ns
                        : NANOSECONDS_PER_SECOND / MAX(s->fps, 1);
}

static void vpss_next_phase(HisiVpssState *s, unsigned phase)
{
    s->phase = phase;
    timer_mod(s->phase_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + vpss_period(s) / 8);
}

static void vpss_frame(void *opaque)
{
    HisiVpssState *s = opaque;
    VpssOut out[VPSS_CHN];
    bool online = vpss_running(s);
    bool apart = false;
    int c;

    if (online) {
        /*
         * VPSS's frame starts after VI's early interrupt, in whose handler
         * the driver programs this frame's buffers and writes REGUP: once
         * REGUP was written, the configuration as it stands now is the one
         * this frame is written with.  (The SDK sample's driver path writes
         * REGUP before the addresses, majestic's sets and clears it after
         * them.)
         */
        if (s->regup) {
            memcpy(s->active, s->regs, sizeof(s->active));
            s->regup = false;
        }
        for (c = 0; c < VPSS_CHN; c++) {
            vpss_chn_out(s, s->active, c, &out[c]);
        }
    } else if (s->node_armed) {
        s->node_armed = false;
        memcpy(out, s->node_out, sizeof(out));
    } else {
        return;             /* the group stopped since VI's frame */
    }
    for (c = 0; c < VPSS_CHN; c++) {
        if (out[c].w && out[c].stride >= out[c].w) {
            vpss_paint(s, out[c].ya, out[c].ca, out[c].stride, out[c].w,
                       out[c].h, s->frame, out[c].cmp);
        }
    }
    s->frame++;
    if (!online) {
        return;
    }
    for (c = 0; c < VPSS_LD_CHN; c++) {
        s->tunl_addr[c] = 0;
        if (s->gen == 5 && out[c].w &&
            (s->active[V5_VPSS_LOWDELAY / 4] & (1u << (4 + c)))) {
            s->tunl_addr[c] = s->active[V5_VPSS_TUNL_ADDR / 4 + c];
            s->tunl_h[c] = out[c].h;
            apart |= s->tunl_addr[c] != 0;
        }
    }
    if (apart || (s->regs[VPSS_INTMASK / 4] & VPSS_INT_DELAY)) {
        s->raw |= VPSS_INT_START;
        vpss_next_phase(s, VPSS_PH_TUNL);
    } else {
        s->raw |= VPSS_INT_START | VPSS_INT_EOF;
    }
    vpss_update_irq(s);
}

static void vpss_phase(void *opaque)
{
    HisiVpssState *s = opaque;
    int c;

    if (!vpss_running(s)) {
        return;
    }
    switch (s->phase) {
    case VPSS_PH_TUNL:
        for (c = 0; c < VPSS_LD_CHN; c++) {
            if (s->tunl_addr[c]) {
                uint16_t lines[2] = { cpu_to_le16(s->tunl_h[c]),
                                      cpu_to_le16(s->tunl_h[c] / 2) };

                /* the whole picture is written: every line is done */
                dma_memory_write(&address_space_memory, s->tunl_addr[c],
                                 &lines[0], 2, MEMTXATTRS_UNSPECIFIED);
                dma_memory_write(&address_space_memory, s->tunl_addr[c] + 32,
                                 &lines[1], 2, MEMTXATTRS_UNSPECIFIED);
                s->raw |= VPSS_INT_TUNL(c);
            }
        }
        vpss_next_phase(s, VPSS_PH_DELAY);
        break;
    case VPSS_PH_DELAY:
        if (s->regs[VPSS_INTMASK / 4] & VPSS_INT_DELAY) {
            s->raw |= VPSS_INT_DELAY;
        }
        vpss_next_phase(s, VPSS_PH_EOF);
        break;
    default:
        s->raw |= VPSS_INT_EOF;
        break;
    }
    vpss_update_irq(s);
}

/*
 * VI's frame: the online VPSS writes it out an eighth of a frame later,
 * after VI's early interrupt had the driver program the buffers.
 */
static void vpss_vi_frame(void *opaque, int n, int level)
{
    HisiVpssState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!level) {
        return;
    }
    /* a gap of over a second is a pause, not the frame rate */
    if (s->vi_ns && now - s->vi_ns < NANOSECONDS_PER_SECOND) {
        s->period_ns = now - s->vi_ns;
    }
    s->vi_ns = now;
    if (s->node_armed) {
        vpss_load_nodes(s);     /* node mode: the list is loaded at start */
    }
    if ((s->regs[VPSS_INTMASK / 4] & VPSS_INT_START) || s->node_armed) {
        timer_mod(s->timer, now + vpss_period(s) / 8);
    }
}

static uint64_t hisi_vpss_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiVpssState *s = HISI_VPSS(opaque);
    uint32_t val;

    switch (offset) {
    case VPSS_RAWINT:
        val = s->raw;
        break;
    case VPSS_INTSTAT:
        val = s->raw & s->regs[VPSS_INTMASK / 4];
        break;
    default:
        val = s->regs[offset / 4];
        break;
    }
    if (s->trace) {
        fprintf(stderr, "vpss R %04x = %08x\n", (int)offset, val);
    }
    return val;
}

static void hisi_vpss_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    HisiVpssState *s = HISI_VPSS(opaque);

    if (s->trace) {
        fprintf(stderr, "vpss W %04x : %08x\n", (int)offset, (uint32_t)val);
    }
    switch (offset) {
    case VPSS_RAWINT:
        s->raw &= ~(uint32_t)val;
        vpss_update_irq(s);
        return;
    }
    s->regs[offset / 4] = val;
    if (offset == VPSS_REGUP && (val & 1)) {
        s->regup = true;
    }
    if (offset == VPSS_NODE_START && (val & 1)) {
        s->node_armed = true;   /* runs on VI's output, from PNEXT */
    }
    if (offset == VPSS_INTMASK || offset == VPSS_CTRL) {
        /* VI runs while the online video is enabled and its frame
         * interrupt unmasked: from VPSS_OnlineEn to the group's stop.
         * A frame due after the stop is not written. */
        qemu_set_irq(s->running, vpss_running(s));
        if (!vpss_running(s)) {
            timer_del(s->timer);
            timer_del(s->phase_timer);
        }
        vpss_update_irq(s);
    }
}

static const MemoryRegionOps hisi_vpss_ops = {
    .read = hisi_vpss_read,
    .write = hisi_vpss_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_vpss_init(Object *obj)
{
    HisiVpssState *s = HISI_VPSS(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_vpss_ops, s, "hisi-vpss",
                          VPSS_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vpss_frame, s);
    s->phase_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vpss_phase, s);
    qdev_init_gpio_in_named(DEVICE(obj), vpss_vi_frame, "vi-frame", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->running, "running", 1);
    object_property_add_link(obj, "scene", TYPE_HISI_SCENE,
                             (Object **)&s->scene,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
}

static void hisi_vpss_reset(DeviceState *dev)
{
    HisiVpssState *s = HISI_VPSS(dev);

    timer_del(s->timer);
    timer_del(s->phase_timer);
    s->phase = 0;
    s->vi_ns = 0;
    s->period_ns = 0;
    memset(s->tunl_addr, 0, sizeof(s->tunl_addr));
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->active, 0, sizeof(s->active));
    s->regup = false;
    s->node_armed = false;
    s->raw = 0;
    s->frame = 0;
    qemu_set_irq(s->irq, 0);
    qemu_set_irq(s->running, 0);
}

static const Property hisi_vpss_properties[] = {
    DEFINE_PROP_UINT32("fps", HisiVpssState, fps, 25),
    DEFINE_PROP_BOOL("trace", HisiVpssState, trace, false),
    /* register layout: 4 (Hi3516EV200 family) or 5 (Hi3516CV610) */
    DEFINE_PROP_UINT8("gen", HisiVpssState, gen, 4),
};

static void hisi_vpss_realize(DeviceState *dev, Error **errp)
{
    HisiVpssState *s = HISI_VPSS(dev);

    if (s->gen != 4 && s->gen != 5) {
        error_setg(errp, "hisi-vpss: gen must be 4 or 5");
    }
}

static void hisi_vpss_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, hisi_vpss_properties);
    device_class_set_legacy_reset(dc, hisi_vpss_reset);
    dc->realize = hisi_vpss_realize;
}

static const TypeInfo hisi_vpss_info = {
    .name          = TYPE_HISI_VPSS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiVpssState),
    .instance_init = hisi_vpss_init,
    .class_init    = hisi_vpss_class_init,
};

static void hisi_vpss_register_types(void)
{
    type_register_static(&hisi_vpss_info);
}

type_init(hisi_vpss_register_types)
