/*
 * HiSilicon VIU of the Hi3516CV200 / Hi3518EV200 family: capture and the
 * ISP in one block at 0x20580000 (the ISP's registers from 0x205A0000),
 * one interrupt (VIC 22) shared by hi3518e_viu.ko and the ISP driver.
 *
 * Offline (vi_vpss_online=0, what OpenIPC loads) capture writes each
 * frame to DDR and the VIU driver hands the buffer to VPSS.  What the
 * drivers wait for (hi3518e_viu.o's VIU_HAL_* accessors and
 * ViuDrvInterruptRoute; the ISP's openhisilicon isp.c, ISP_ISR):
 *
 *   0x00f0  global indicator, derived: bit 1 channel 0 (the VIU driver
 *           masks it with 0xfffe), bit 8 the ISP, bit 16 port 0
 *   0x00f8  its mask (the ISP driver sets 0x10100, the VIU driver its
 *           channel's bit)
 *   0x01f0 / 0x01f8  port 0 status / mask: bit 0 frame start (ISP_ISR
 *           counts frames on it), bits 1-2 a size change (the ISP resets)
 *   0x01e0  bit 3: the bottom field (VIU_HAL_IsInCapTop); 0, progressive
 *   0x01ec  the port's measured size, width | height << 16
 *   0x1004  bit 0: the channel takes the registers below at the next frame
 *   0x1080 / 0x10a0  Y / C write: bit 31 on
 *   0x1084 / 0x10a4  Y / C size: width - 1 | (height - 1) << 16
 *   0x1090 / 0x10b0  Y / C address
 *   0x1098 / 0x10b8  Y / C stride in bits 31:16
 *   0x10f0 / 0x10f8  channel 0 status / mask, write 1s to clear: bit 0 a
 *           frame ended (ViuDrvChnIntProcess queues the next buffer and
 *           hands on the last; it rises whether or not the channel wrote
 *           or is unmasked)
 *   0x200f0 / 0x200f8  the ISP's status / mask, write 1s to clear: bit 0
 *           frame start (the 3A wake-up and register sync), bit 1 the
 *           input formatter, bit 6 frame end
 *
 * The ISP's statistics (isp.c ISP_DRV_StatisticsRead; the accessors are
 * openhisilicon's vreg isp_config*.h, ISP-relative, here + 0x20000), with
 * the scales a running camera's registers show:
 *   0x22010 / 0x22110  AE / AWB zones, columns | rows << 8 (17 x 15)
 *   0x22018 / 0x2201c  AE pixels counted / their weight: 2x2 Bayer quads
 *   0x22020..0x2202c   AE global R / Gr / Gb / B means, 8-bit
 *   0x22088 / 0x2208c  AE histogram port: write the index, then each read
 *                      returns a bin and moves on (256 bins of 8-bit luma)
 *   0x22098 / 0x2209c  AE zone port: R << 24 | Gr << 16 | Gb << 8 | B
 *   0x22140..0x22154   AWB global R / G / B means (12-bit, before the
 *                      white balance), count, count min / max
 *   0x22188 / 0x2218c  AWB zone port, three words a zone: G << 16 | R,
 *                      count << 16 | B, then the counts above / below the
 *                      thresholds (0 here)
 *   0x21110..0x21116   white balance gains, 8.8, 16-bit each: Gr, R, Gb, B
 * The ISP code reads and writes bytes and halfwords, so the register file
 * is byte-addressed.
 *
 * Statistics and the picture come from the scene in front of the camera
 * (hisi-scene) through the sensor's exposure one frame late, as on the V4
 * machines: AE converges on an exposure, AWB on gains that neutralise the
 * light.  The picture is NV21, with the frame number as 16 binary squares
 * along its top twelfth (the stream checks read them).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/i2c/image-sensor.h"
#include "hw/misc/hisi-scene.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_HISI_VIU "hisi-viu"
OBJECT_DECLARE_SIMPLE_TYPE(HisiViuState, HISI_VIU)

#define VIU_SIZE            0x40000
#define VIU_GLB_INT         0x00f0
#define VIU_GLB_MASK        0x00f8
#define VIU_PT_INT          0x01f0
#define VIU_PT_MASK         0x01f8
#define VIU_PT_SIZE         0x01ec
#define VIU_CH_NEWER        0x1004
#define VIU_CH_Y_CTRL       0x1080
#define VIU_CH_Y_SIZE       0x1084
#define VIU_CH_Y_ADDR       0x1090
#define VIU_CH_Y_STRIDE     0x1098
#define VIU_CH_C_ADDR       0x10b0
#define VIU_CH_C_STRIDE     0x10b8
#define VIU_CH_INT          0x10f0
#define VIU_CH_MASK         0x10f8
#define VIU_ISP_INT         0x200f0
#define VIU_ISP_MASK        0x200f8

#define ISP_WB_GAIN         0x21110     /* Gr, R, Gb, B halfwords */
#define ISP_AE_ZONES        0x22010
#define ISP_AE_COUNT        0x22018
#define ISP_AE_WEIGHT       0x2201c
#define ISP_AE_MEAN         0x22020
#define ISP_AE_HIST_ADDR    0x22088
#define ISP_AE_HIST_DATA    0x2208c
#define ISP_AE_ZONE_ADDR    0x22098
#define ISP_AE_ZONE_DATA    0x2209c
#define ISP_AWB_ZONES       0x22110
#define ISP_AWB_MEAN        0x22140
#define ISP_AWB_COUNT       0x2214c
#define ISP_AWB_ZONE_ADDR   0x22188
#define ISP_AWB_ZONE_DATA   0x2218c

#define ISP_INT_FSTART      0x01
#define ISP_INT_IF          0x02
#define ISP_INT_FEND        0x40

#define VIU_FRAME_NS        (NANOSECONDS_PER_SECOND / 30)
#define VIU_FRAME_NS_MIN    (NANOSECONDS_PER_SECOND / 1000)
#define VIU_FRAME_NS_MAX    NANOSECONDS_PER_SECOND
#define ZONE_COLS           17
#define ZONE_ROWS           15
#define ZONES               (ZONE_COLS * ZONE_ROWS)
#define GRID_W              (ZONE_COLS * 8)     /* the statistics' grid */
#define GRID_H              (ZONE_ROWS * 8)

struct HisiViuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *start_timer;
    QEMUTimer *end_timer;

    uint8_t regs[VIU_SIZE];
    uint32_t act[0x40 / 4];         /* 0x1080..0x10bf as the frame took them */
    bool newer;                     /* 0x1004 bit 0 written */
    uint32_t frames;
    int64_t frame_ns;

    /* this frame's statistics, behind the read ports */
    uint32_t hist[256];
    uint32_t ae_zone[ZONES];
    uint32_t awb_zone[ZONES * 3];
    uint32_t hist_i, ae_i, awb_i;

    Object *sensor;
    HisiSceneState *scene;
    ImageSensorExposure expo, frame_expo;
    double level;
};

static uint32_t viu_rd(HisiViuState *s, hwaddr a)
{
    return ldl_le_p(s->regs + a);
}

static void viu_wr(HisiViuState *s, hwaddr a, uint32_t v)
{
    stl_le_p(s->regs + a, v);
}

static void viu_update_irq(HisiViuState *s)
{
    uint32_t g = 0;

    if (viu_rd(s, VIU_CH_INT) & viu_rd(s, VIU_CH_MASK)) {
        g |= 1u << 1;
    }
    if (viu_rd(s, VIU_ISP_INT) & viu_rd(s, VIU_ISP_MASK)) {
        g |= 1u << 8;
    }
    if (viu_rd(s, VIU_PT_INT) & viu_rd(s, VIU_PT_MASK)) {
        g |= 1u << 16;
    }
    viu_wr(s, VIU_GLB_INT, g);
    qemu_set_irq(s->irq, (g & viu_rd(s, VIU_GLB_MASK)) != 0);
}

/* Anything to capture: the ISP waits for frames before the channel exists */
static bool viu_running(HisiViuState *s)
{
    return viu_rd(s, VIU_PT_MASK) || viu_rd(s, VIU_ISP_MASK) ||
           viu_rd(s, VIU_CH_MASK);
}

/* The channel's registers as the frame in progress took them */
static uint32_t viu_act(HisiViuState *s, hwaddr a)
{
    return s->act[(a - VIU_CH_Y_CTRL) / 4];
}

static void viu_schedule(HisiViuState *s)
{
    int64_t ns = 0;

    if (s->sensor) {
        ns = image_sensor_frame_ns(s->sensor);
    }
    /* a sensor's timing is the guest's to program: keep the frame clock
     * between 1000 and 1 frames a second */
    s->frame_ns = ns > 0 ? MIN(MAX(ns, VIU_FRAME_NS_MIN), VIU_FRAME_NS_MAX)
                         : VIU_FRAME_NS;
    timer_mod(s->start_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->frame_ns);
}

/* the white balance gains the ISP programmed, 1.0 = none */
static double viu_wb(HisiViuState *s, int k)
{
    uint16_t g = lduw_le_p(s->regs + ISP_WB_GAIN + 2 * k);

    return g ? g / 256.0 : 1.0;
}

static void viu_paint(HisiViuState *s)
{
    uint32_t sz = viu_act(s, VIU_CH_Y_SIZE);
    int w = (sz & 0x1fff) + 1, h = ((sz >> 16) & 0x1fff) + 1;
    uint32_t ya = viu_act(s, VIU_CH_Y_ADDR), ca = viu_act(s, VIU_CH_C_ADDR);
    int ys = viu_act(s, VIU_CH_Y_STRIDE) >> 16;
    int cs = viu_act(s, VIU_CH_C_STRIDE) >> 16;
    const int sq = MAX(w / 16, 1);
    const uint32_t n = s->frames;
    const float *grid;
    g_autofree uint8_t *line = NULL, *vu = NULL;

    if (!ya || w < 2 || h < 2) {
        return;
    }
    ys = ys ? ys : w;
    cs = cs ? cs : ys;
    grid = s->scene ? hisi_scene_grid(s->scene, w, h) : NULL;
    line = g_malloc(w + 2);
    vu = g_malloc(w + 2);
    for (int r = 0; r < h; r++) {
        if (r < h / 12) {
            for (int x = 0; x < w; x++) {
                line[x] = (n >> (15 - MIN(x / sq, 15))) & 1 ? 235 : 16;
            }
            memset(vu, 128, w);
        } else if (grid) {
            hisi_scene_display_row(s->scene, grid + (size_t)r * w * 3, w,
                                   line, r & 1 ? NULL : vu);
        } else {
            memset(line, 126, w);
            memset(vu, 128, w);
        }
        dma_memory_write(&address_space_memory, ya + (uint64_t)r * ys, line,
                         w, MEMTXATTRS_UNSPECIFIED);
        if (!(r & 1) && ca) {
            dma_memory_write(&address_space_memory,
                             ca + (uint64_t)(r / 2) * cs, vu, w & ~1,
                             MEMTXATTRS_UNSPECIFIED);
        }
    }
}

/*
 * The statistics of the frame starting: the scene's raw Bayer levels on a
 * grid of 8 x 8 cells a zone, white at the sensor's exposure x gain.
 */
static void viu_stats(HisiViuState *s, const double tint[3])
{
    const float *refl = s->scene ? hisi_scene_grid(s->scene, GRID_W, GRID_H)
                                 : NULL;
    int w = s->frame_expo.width ? s->frame_expo.width : 1920;
    int h = s->frame_expo.height ? s->frame_expo.height : 1080;
    double quads = (double)w * h / 4, per_cell = quads / (GRID_W * GRID_H);
    double sum[3] = { 0 }, zs[ZONES][3] = { { 0 } };
    double hist[256] = { 0 };

    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            int z = (y / 8) * ZONE_COLS + x / 8;
            double v[3], luma;

            for (int c = 0; c < 3; c++) {
                double p = refl ? refl[((size_t)y * GRID_W + x) * 3 + c]
                                : 0.5;

                v[c] = MIN(p * tint[c] * s->level, 65535.0);
                sum[c] += v[c];
                zs[z][c] += v[c];
            }
            luma = (v[0] + 2 * v[1] + v[2]) / 4;
            hist[MIN((int)luma >> 8, 255)] += per_cell;
        }
    }
    for (int i = 0; i < 256; i++) {
        s->hist[i] = (uint32_t)hist[i];
    }
    for (int z = 0; z < ZONES; z++) {
        uint32_t r = zs[z][0] / 64, g = zs[z][1] / 64, b = zs[z][2] / 64;
        uint32_t cnt = MIN(quads / ZONES, 65535);

        s->ae_zone[z] = (r >> 8) << 24 | (g >> 8) << 16 | (g >> 8) << 8 |
                        (b >> 8);
        s->awb_zone[z * 3] = (g >> 4) << 16 | (r >> 4);
        s->awb_zone[z * 3 + 1] = cnt << 16 | (b >> 4);
        s->awb_zone[z * 3 + 2] = 0;    /* counts beyond the thresholds:
                                         * a camera's global ones read 0 */
    }
    for (int c = 0; c < 3; c++) {
        sum[c] /= GRID_W * GRID_H;
    }
    viu_wr(s, ISP_AE_ZONES, ZONE_COLS | ZONE_ROWS << 8);
    viu_wr(s, ISP_AWB_ZONES, ZONE_COLS | ZONE_ROWS << 8);
    viu_wr(s, ISP_AE_COUNT, quads);
    viu_wr(s, ISP_AE_WEIGHT, quads);
    viu_wr(s, ISP_AE_MEAN, (uint32_t)sum[0] >> 8);
    viu_wr(s, ISP_AE_MEAN + 4, (uint32_t)sum[1] >> 8);
    viu_wr(s, ISP_AE_MEAN + 8, (uint32_t)sum[1] >> 8);
    viu_wr(s, ISP_AE_MEAN + 12, (uint32_t)sum[2] >> 8);
    viu_wr(s, ISP_AWB_MEAN, (uint32_t)sum[0] >> 4);
    viu_wr(s, ISP_AWB_MEAN + 4, (uint32_t)sum[1] >> 4);
    viu_wr(s, ISP_AWB_MEAN + 8, (uint32_t)sum[2] >> 4);
    viu_wr(s, ISP_AWB_COUNT, MIN(quads, 65535));
}

/* Frame start: the ISP's and the port's; the sensor applies its writes */
static void viu_frame_start(void *opaque)
{
    HisiViuState *s = opaque;
    double tint[3] = { 1, 1, 1 }, light = 1, gg;
    ImageSensorExposure now;

    if (!viu_running(s)) {
        return;
    }
    /* VIU_HAL_SetChnRegNewer: the queued buffer is this frame's */
    if (s->newer) {
        for (int i = 0; i < ARRAY_SIZE(s->act); i++) {
            s->act[i] = viu_rd(s, VIU_CH_Y_CTRL + 4 * i);
        }
        s->newer = false;
    }
    if (s->scene) {
        hisi_scene_frame_begin(s->scene);
        light = hisi_scene_light(s->scene, tint);
    }
    if (s->sensor && image_sensor_exposure(s->sensor, &now)) {
        if (image_sensor_frame_start(s->sensor)) {
            s->frame_expo = now;
        } else {
            s->frame_expo = s->expo;
            s->expo = now;
        }
        s->level = (double)s->frame_expo.lines * s->frame_expo.gain / 1024 *
                   light;
    } else {
        s->level = 65535.0 / 2;
    }
    viu_stats(s, tint);
    if (s->scene) {
        /* the picture through the gains: green's as overall level, R and
         * B relative to it */
        gg = (viu_wb(s, 0) + viu_wb(s, 2)) / 2;
        hisi_scene_set_frame(s->scene, s->level * gg, tint,
                             viu_wb(s, 1) / gg, viu_wb(s, 3) / gg);
    }
    if (s->frame_expo.width && s->frame_expo.height) {
        viu_wr(s, VIU_PT_SIZE, s->frame_expo.width |
                               (uint32_t)s->frame_expo.height << 16);
    } else {
        uint32_t sz = viu_rd(s, VIU_CH_Y_SIZE);
        viu_wr(s, VIU_PT_SIZE, ((sz & 0xffff) + 1) |
                               ((sz + 0x10000) & 0xffff0000));
    }
    viu_wr(s, VIU_PT_INT, viu_rd(s, VIU_PT_INT) | 1);
    viu_wr(s, VIU_ISP_INT, viu_rd(s, VIU_ISP_INT) | ISP_INT_FSTART |
                           ISP_INT_IF);
    viu_update_irq(s);
    /* the frame is in DDR a little before the next one starts */
    timer_mod(s->end_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              s->frame_ns * 3 / 4);
    viu_schedule(s);
}

static void viu_frame_end(void *opaque)
{
    HisiViuState *s = opaque;

    if ((viu_act(s, VIU_CH_Y_CTRL) & (1u << 31)) &&
        (viu_rd(s, VIU_CH_Y_CTRL) & (1u << 31))) {
        viu_paint(s);
        s->frames++;
    }
    /* the status rises whether or not it is unmasked; the mask only
     * gates the interrupt (viu_update_irq) */
    viu_wr(s, VIU_CH_INT, viu_rd(s, VIU_CH_INT) | 1);
    viu_wr(s, VIU_ISP_INT, viu_rd(s, VIU_ISP_INT) | ISP_INT_FEND);
    viu_update_irq(s);
}

static uint64_t hisi_viu_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiViuState *s = HISI_VIU(opaque);

    if (offset + size > VIU_SIZE) {
        return 0;
    }
    switch (offset) {
    case ISP_AE_HIST_DATA:
        return s->hist[s->hist_i++ % 256];
    case ISP_AE_ZONE_DATA:
        return s->ae_zone[s->ae_i++ % ZONES];
    case ISP_AWB_ZONE_DATA:
        return s->awb_zone[s->awb_i++ % (ZONES * 3)];
    }
    return ldn_le_p(s->regs + offset, size);
}

static void hisi_viu_write(void *opaque, hwaddr offset, uint64_t val64,
                           unsigned size)
{
    HisiViuState *s = HISI_VIU(opaque);
    uint32_t val = val64;
    bool was = viu_running(s);

    if (offset + size > VIU_SIZE) {
        return;
    }
    switch (offset) {
    case VIU_GLB_INT:
        return;
    case VIU_PT_INT:
    case VIU_CH_INT:
    case VIU_ISP_INT:
        viu_wr(s, offset, viu_rd(s, offset) & ~val);
        viu_update_irq(s);
        return;
    case VIU_CH_NEWER:
        viu_wr(s, offset, val & ~1u);       /* taken at the next frame */
        s->newer |= val & 1;
        return;
    case ISP_AE_HIST_ADDR:
        s->hist_i = val;
        break;
    case ISP_AE_ZONE_ADDR:
        s->ae_i = val;
        break;
    case ISP_AWB_ZONE_ADDR:
        s->awb_i = (val & 0x3ff) * 3;
        break;
    }
    stn_le_p(s->regs + offset, size, val);
    if (offset == VIU_GLB_MASK || offset == VIU_PT_MASK ||
        offset == VIU_CH_MASK || offset == VIU_ISP_MASK) {
        viu_update_irq(s);
    }
    if (!was && viu_running(s) && !timer_pending(s->start_timer)) {
        viu_schedule(s);
    }
}

static const MemoryRegionOps hisi_viu_ops = {
    .read = hisi_viu_read,
    .write = hisi_viu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void hisi_viu_reset(DeviceState *dev)
{
    HisiViuState *s = HISI_VIU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->act, 0, sizeof(s->act));
    s->newer = false;
    memset(&s->expo, 0, sizeof(s->expo));
    memset(&s->frame_expo, 0, sizeof(s->frame_expo));
    s->frames = 0;
    s->level = 0;
    s->hist_i = s->ae_i = s->awb_i = 0;
    timer_del(s->start_timer);
    timer_del(s->end_timer);
    qemu_set_irq(s->irq, 0);
}

static void hisi_viu_realize(DeviceState *dev, Error **errp)
{
    HisiViuState *s = HISI_VIU(dev);

    s->start_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, viu_frame_start, s);
    s->end_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, viu_frame_end, s);
}

static void hisi_viu_init(Object *obj)
{
    HisiViuState *s = HISI_VIU(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_viu_ops, s, TYPE_HISI_VIU,
                          VIU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    object_property_add_link(obj, "sensor", TYPE_OBJECT,
                             (Object **)&s->sensor,
                             qdev_prop_allow_set_link_before_realize,
                             OBJ_PROP_LINK_STRONG);
    object_property_add_link(obj, "scene", TYPE_HISI_SCENE,
                             (Object **)&s->scene,
                             qdev_prop_allow_set_link_before_realize,
                             OBJ_PROP_LINK_STRONG);
    object_property_add_uint32_ptr(obj, "frames", &s->frames,
                                   OBJ_PROP_FLAG_READ);
}

static const VMStateDescription vmstate_hisi_viu = {
    .name = TYPE_HISI_VIU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, HisiViuState, VIU_SIZE),
        VMSTATE_UINT32_ARRAY(act, HisiViuState, 0x40 / 4),
        VMSTATE_BOOL(newer, HisiViuState),
        VMSTATE_UINT32(frames, HisiViuState),
        VMSTATE_INT64(frame_ns, HisiViuState),
        VMSTATE_UINT32_ARRAY(hist, HisiViuState, 256),
        VMSTATE_UINT32_ARRAY(ae_zone, HisiViuState, ZONES),
        VMSTATE_UINT32_ARRAY(awb_zone, HisiViuState, ZONES * 3),
        VMSTATE_UINT32(hist_i, HisiViuState),
        VMSTATE_UINT32(ae_i, HisiViuState),
        VMSTATE_UINT32(awb_i, HisiViuState),
        VMSTATE_TIMER_PTR(start_timer, HisiViuState),
        VMSTATE_TIMER_PTR(end_timer, HisiViuState),
        VMSTATE_END_OF_LIST()
    },
};

static void hisi_viu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_viu_reset);
    dc->realize = hisi_viu_realize;
    dc->vmsd = &vmstate_hisi_viu;
}

static const TypeInfo hisi_viu_info = {
    .name          = TYPE_HISI_VIU,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiViuState),
    .instance_init = hisi_viu_init,
    .class_init    = hisi_viu_class_init,
};

static void hisi_viu_register_types(void)
{
    type_register_static(&hisi_viu_info);
}

type_init(hisi_viu_register_types)
