/*
 * HiSilicon V4 VPSS (Hi3516EV200 family) in VI-online / VPSS-online mode:
 * the frame source of the video pipeline.
 *
 * In online mode the sensor's frames stream from VI through the ISP into
 * VPSS, which scales them into up to three output channels in DDR.  The
 * driver (hi_vpss.o, VPSS_COMM_OnLineIrqProc) works per frame start:
 * it hands the frame VPSS has just written to the bound modules (VENC)
 * and programs the next frame's buffers, latched by REGUP.  This model
 * produces the frames: on every frame of VI's (the "vi-frame" GPIO, from
 * hisi-vi) it paints a test picture into every enabled channel's buffers
 * and raises frame start and end.
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

#define TYPE_HISI_VPSS "hisi-vpss"
OBJECT_DECLARE_SIMPLE_TYPE(HisiVpssState, HISI_VPSS)

#define VPSS_REG_SIZE       0x10000
#define VPSS_CTRL           0x0000  /* bit 31 online video enable */
#define VPSS_REGUP          0x0010
#define VPSS_INTMASK        0x00f0
#define VPSS_RAWINT         0x0310
#define VPSS_INTSTAT        0x0314
#define VPSS_CHN0_CMP       0x0700  /* bit 0: channel 0 writes compressed */

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

struct HisiVpssState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq running;           /* to hisi-vi: the online pipeline runs */
    QEMUTimer *timer;
    uint32_t regs[VPSS_REG_SIZE / 4];
    uint32_t active[VPSS_REG_SIZE / 4];  /* what frames are written with */
    bool regup;                          /* REGUP written since the last */
    uint32_t raw;
    uint32_t frame;
    uint32_t fps;               /* VI's frame rate */
    bool trace;
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
 * The test picture, NV21: vertical bars that move 8 pixels a frame, the
 * frame number as 16 binary squares along the top.  Rows of luma are
 * written while they fit before the chroma plane (a compressed channel's
 * buffer is smaller than a linear picture).
 */
static void vpss_paint(uint32_t ya, uint32_t ca, int stride, int w, int h,
                       uint32_t n, bool half)
{
    static const uint8_t bar_y[8] = { 235, 210, 170, 145, 106, 81, 41, 16 };
    const int bw = MAX(w / 8, 1), sq = MAX(w / 16, 1), step = half ? 2 : 1;
    g_autofree uint8_t *line = g_malloc(MAX(stride, w));
    int r, x;

    for (r = 0; r < h; r += step) {
        for (x = 0; x < w; x++) {
            line[x] = r < h / 12 ? ((n >> (15 - MIN(x / sq, 15))) & 1 ? 235 : 16)
                                 : bar_y[((x + n * 8) / bw) % 8];
        }
        dma_memory_write(&address_space_memory,
                         ya + (uint64_t)(r / step) * stride, line, w,
                         MEMTXATTRS_UNSPECIFIED);
    }
    for (r = 0; r < h / 2; r += step) {
        for (x = 0; x < w / 2; x++) {
            const int b = ((x * 2 + n * 8) / bw) % 8;

            line[x * 2] = b & 1 ? 188 : 68;         /* V */
            line[x * 2 + 1] = b & 2 ? 188 : 68;     /* U */
        }
        dma_memory_write(&address_space_memory,
                         ca + (uint64_t)(r / step) * stride, line, w & ~1,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

static void vpss_frame(void *opaque)
{
    HisiVpssState *s = opaque;
    const uint32_t *r = s->active;
    int c;

    if (!vpss_running(s)) {
        return;             /* the group stopped since VI's frame */
    }

    /*
     * VPSS's frame starts after VI's early interrupt, in whose handler the
     * driver programs this frame's buffers and writes REGUP: once REGUP
     * was written, the configuration as it stands now is the one this
     * frame is written with.  (The SDK sample's driver path writes REGUP
     * before the addresses, majestic's sets and clears it after them.)
     */
    if (s->regup) {
        memcpy(s->active, s->regs, sizeof(s->active));
        s->regup = false;
    }
    for (c = 0; c < VPSS_CHN; c++) {
        const uint32_t size = r[vpss_chn[c].size / 4];
        const int w = (size & 0xffff) + 1, h = (size >> 16) + 1;
        const uint32_t ya = r[vpss_chn[c].addr / 4 + 4];
        const uint32_t ca = r[vpss_chn[c].addr / 4 + 6];
        const bool cmp = c == 0 && (r[VPSS_CHN0_CMP / 4] & 1);
        const int stride = cmp ? w : r[vpss_chn[c].stride / 4] & 0xffff;

        if (!(r[vpss_chn[c].ctrl / 4] & (1u << 31)) || !ya || !ca ||
            w > PIC_MAX || h > PIC_MAX || stride < w) {
            continue;       /* not written this frame (frame-rate control) */
        }
        vpss_paint(ya, ca, stride, w, h, s->frame, cmp);
    }
    s->frame++;
    s->raw |= VPSS_INT_START | VPSS_INT_EOF;
    vpss_update_irq(s);
}

/*
 * VI's frame: the online VPSS writes it out an eighth of a frame later,
 * after VI's early interrupt had the driver program the buffers.
 */
static void vpss_vi_frame(void *opaque, int n, int level)
{
    HisiVpssState *s = opaque;

    if (level && (s->regs[VPSS_INTMASK / 4] & VPSS_INT_START)) {
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
}

static void hisi_vpss_reset(DeviceState *dev)
{
    HisiVpssState *s = HISI_VPSS(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->active, 0, sizeof(s->active));
    s->regup = false;
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
