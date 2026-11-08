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
#define VPSS_NODE_MAX       8       /* strips of one picture */
#define VPSS_NODE_SIZE      0x800   /* what of a node the model reads */

#define VPSS_INT_EOF        (1u << 0)
#define VPSS_INT_START      (1u << 10)

#define VPSS_CHN            3
#define PIC_MAX             4096

static const struct {
    hwaddr size, ctrl, addr, stride;
} vpss_chn[VPSS_CHN] = {
    { 0x44c, 0x450, 0x468, 0x488 },
    { 0x4a0, 0x4a4, 0x4b0, 0x4d0 },
    { 0x4e8, 0x4ec, 0x4f4, 0x514 },
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
    uint32_t regs[VPSS_REG_SIZE / 4];
    uint32_t active[VPSS_REG_SIZE / 4];  /* what frames are written with */
    bool regup;                          /* REGUP written since the last */
    bool node_armed;                     /* node mode: VPSS waits for VI */
    VpssOut node_out[VPSS_CHN];          /* ... and what it then loaded */
    uint32_t raw;
    uint32_t frame;
    uint32_t fps;               /* VI's frame rate */
    bool trace;
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
static void vpss_chn_out(const uint32_t *r, int c, VpssOut *o)
{
    const uint32_t size = r[vpss_chn[c].size / 4];

    o->w = (size & 0xffff) + 1;
    o->h = (size >> 16) + 1;
    o->ya = r[vpss_chn[c].addr / 4 + 4];
    o->ca = r[vpss_chn[c].addr / 4 + 6];
    o->cmp = c == 0 && (r[VPSS_CHN0_CMP / 4] & 1);
    o->stride = o->cmp ? o->w : r[vpss_chn[c].stride / 4] & 0xffff;
    if (!(r[vpss_chn[c].ctrl / 4] & (1u << 31)) || !o->ya || !o->ca ||
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
    uint32_t a = s->regs[VPSS_PNEXT / 4];
    int c, k;

    memset(s->node_out, 0, sizeof(s->node_out));
    for (k = 0; k < VPSS_NODE_MAX && a; k++) {
        dma_memory_read(&address_space_memory, a, node, sizeof(node),
                        MEMTXATTRS_UNSPECIFIED);
        for (c = 0; c < VPSS_CHN; c++) {
            VpssOut o;

            vpss_chn_out(node, c, &o);
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

static void vpss_frame(void *opaque)
{
    HisiVpssState *s = opaque;
    VpssOut out[VPSS_CHN];
    bool online = vpss_running(s);
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
            vpss_chn_out(s->active, c, &out[c]);
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
    if (online) {
        s->raw |= VPSS_INT_START | VPSS_INT_EOF;
        vpss_update_irq(s);
    }
}

/*
 * VI's frame: the online VPSS writes it out an eighth of a frame later,
 * after VI's early interrupt had the driver program the buffers.
 */
static void vpss_vi_frame(void *opaque, int n, int level)
{
    HisiVpssState *s = opaque;

    if (!level) {
        return;
    }
    if (s->node_armed) {
        vpss_load_nodes(s);     /* node mode: the list is loaded at start */
    }
    if ((s->regs[VPSS_INTMASK / 4] & VPSS_INT_START) || s->node_armed) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / 8 / MAX(s->fps, 1));
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
};

static void hisi_vpss_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, hisi_vpss_properties);
    device_class_set_legacy_reset(dc, hisi_vpss_reset);
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
