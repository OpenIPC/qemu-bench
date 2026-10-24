/*
 * Allwinner G2D (2D engine, "mixer" with a register configuration queue)
 * on the V831/V833, at 0x01480000.
 *
 * The vendor driver (sunxi g2d_rcq) does not program the mixer's registers
 * directly: it lays their values out in memory as blocks, writes a queue
 * of block headers (the RCQ), and has the top block load and run it:
 *
 *   0x000 SCLK_GATE  0x004 HCLK_GATE  0x008 AHB_RESET
 *   0x020 RCQ_IRQ_CTL   bit 4 task-end interrupt enable
 *   0x024 RCQ_STATUS    write 1 to clear
 *   0x028 RCQ_CTRL      bit 0 start
 *   0x02C RCQ_HEADER_LOW_ADDR  0x030 high  0x034 RCQ_HEADER_LEN (bytes)
 *
 * A header is 16 bytes: the block's address, its length in bytes, a dirty
 * flag, and the register offset it loads at.  Addresses are device
 * addresses, behind the IOMMU as its G2D master.  Running the queue loads
 * the blocks into the register file and runs the task they describe; then
 * RCQ_STATUS raises the task end.
 *
 * The task modelled is what the media stack asks for: the video layer V0
 * scaled into the write-back buffer, with format conversion -- the YI
 * application turns each 640x360 NV21 frame into a 320x240 grey (Y8) one
 * for its motion and person detection.  Formats are the driver's
 * g2d_fmt_enh numbers; scaling is nearest-sample.  Blending with the UI
 * layers, rotation and the other ops are not modelled.
 *
 *   V0  0x0800  ATTCTL bit 0 enable, bits 13:8 format; 0x04 MBSIZE,
 *               0x08 COOR, 0x0C..0x14 PITCH0-2, 0x18..0x20 LADD0-2
 *   WB  0x3000  ATT bits 5:0 format; 0x04 SIZE; 0x08..0x10 PITCH0-2;
 *               0x14, 0x1C, 0x24 LADD0-2
 *   sizes are (height - 1) << 16 | (width - 1)
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/aw-iommu.h"
#include "qom/object.h"

#define TYPE_AW_G2D "aw-g2d"
OBJECT_DECLARE_SIMPLE_TYPE(AwG2dState, AW_G2D)

#define AW_G2D_SIZE         0xC0000

#define G2D_RCQ_IRQ_CTL     0x020
#define G2D_RCQ_STATUS      0x024
#define G2D_RCQ_CTRL        0x028
#define G2D_RCQ_HEADER_LOW  0x02C
#define G2D_RCQ_HEADER_HIGH 0x030
#define G2D_RCQ_HEADER_LEN  0x034

#define G2D_V0              0x0800
#define G2D_WB              0x3000

/* g2d_fmt_enh */
#define FMT_ARGB8888        0x00
#define FMT_ABGR8888        0x01
#define FMT_RGBA8888        0x02
#define FMT_BGRA8888        0x03
#define FMT_XRGB8888        0x04
#define FMT_XBGR8888        0x05
#define FMT_RGBX8888        0x06
#define FMT_BGRX8888        0x07
#define FMT_RGB888          0x08
#define FMT_BGR888          0x09
#define FMT_YUV422SP_VU     0x24
#define FMT_YUV422SP_UV     0x25
#define FMT_YUV422P         0x26
#define FMT_YUV420SP_VU     0x28
#define FMT_YUV420SP_UV     0x29
#define FMT_YUV420P         0x2A
#define FMT_Y8              0x30

struct AwG2dState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    bool trace;
    DeviceState *iommu;
    uint32_t *regs;
};

static void aw_g2d_update_irq(AwG2dState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[G2D_RCQ_STATUS >> 2] &
                            s->regs[G2D_RCQ_IRQ_CTL >> 2] & 0x11));
}

/* Load the queue's register blocks into the register file. */
static void aw_g2d_load_rcq(AwG2dState *s)
{
    uint32_t addr = s->regs[G2D_RCQ_HEADER_LOW >> 2];
    uint32_t len = s->regs[G2D_RCQ_HEADER_LEN >> 2];
    uint32_t i;

    for (i = 0; i + 16 <= len; i += 16) {
        uint32_t hd[4], n, k;
        g_autofree uint32_t *blk = NULL;

        aw_iommu_dma_read(s->iommu, AW_IOMMU_MASTER_G2D, addr + i, hd, 16);
        n = hd[1] & 0xFFFFFF;
        if (!n || hd[3] >= AW_G2D_SIZE || n > AW_G2D_SIZE - hd[3]) {
            continue;
        }
        blk = g_malloc(n);
        aw_iommu_dma_read(s->iommu, AW_IOMMU_MASTER_G2D, hd[0], blk, n);
        for (k = 0; k < n / 4; k++) {
            uint32_t r = hd[3] + k * 4;

            s->regs[r >> 2] = blk[k];
        }
    }
}

/* A picture in memory, in one of the formats modelled. */
typedef struct AwG2dPic {
    uint32_t fmt, w, h;
    uint32_t pitch[3], addr[3];
} AwG2dPic;

static uint32_t aw_g2d_reg(AwG2dState *s, uint32_t off)
{
    return s->regs[off >> 2];
}

/* bytes a pixel, for the packed RGB formats; 0 for the others */
static unsigned aw_g2d_fmt_bpp(uint32_t fmt)
{
    return fmt <= FMT_BGRX8888 ? 4 : fmt <= FMT_BGR888 ? 3 : 0;
}

static bool aw_g2d_fmt_ok(uint32_t fmt)
{
    return aw_g2d_fmt_bpp(fmt) || fmt == FMT_Y8 ||
           (fmt >= FMT_YUV422SP_VU && fmt <= FMT_YUV422P) ||
           (fmt >= FMT_YUV420SP_VU && fmt <= FMT_YUV420P);
}

/* Sample (x, y) of @p, from the picture's planes in @pl, as Y, U, V. */
static void aw_g2d_get(const AwG2dPic *p, uint8_t *const pl[3], uint32_t x,
                       uint32_t y, uint8_t *yuv)
{
    uint32_t cy = (p->fmt >= FMT_YUV420SP_VU && p->fmt <= FMT_YUV420P) ?
                  y / 2 : y;
    unsigned bpp = aw_g2d_fmt_bpp(p->fmt);

    if (bpp) {
        const uint8_t *px = pl[0] + y * p->pitch[0] + x * bpp;
        int r, g, b;

        switch (p->fmt) {
        case FMT_ARGB8888: case FMT_XRGB8888:
            b = px[0]; g = px[1]; r = px[2];
            break;
        case FMT_ABGR8888: case FMT_XBGR8888:
            r = px[0]; g = px[1]; b = px[2];
            break;
        case FMT_RGBA8888: case FMT_RGBX8888:
            b = px[1]; g = px[2]; r = px[3];
            break;
        case FMT_BGRA8888: case FMT_BGRX8888:
            r = px[1]; g = px[2]; b = px[3];
            break;
        case FMT_RGB888:
            b = px[0]; g = px[1]; r = px[2];
            break;
        default:
            r = px[0]; g = px[1]; b = px[2];
            break;
        }
        /* BT.601, studio range */
        yuv[0] = (66 * r + 129 * g + 25 * b + 128) / 256 + 16;
        yuv[1] = (-38 * r - 74 * g + 112 * b + 128) / 256 + 128;
        yuv[2] = (112 * r - 94 * g - 18 * b + 128) / 256 + 128;
        return;
    }
    yuv[0] = pl[0][y * p->pitch[0] + x];
    switch (p->fmt) {
    case FMT_Y8:
        yuv[1] = yuv[2] = 128;
        break;
    case FMT_YUV422SP_VU: case FMT_YUV420SP_VU:
        yuv[2] = pl[1][cy * p->pitch[1] + (x & ~1u)];
        yuv[1] = pl[1][cy * p->pitch[1] + (x | 1)];
        break;
    case FMT_YUV422SP_UV: case FMT_YUV420SP_UV:
        yuv[1] = pl[1][cy * p->pitch[1] + (x & ~1u)];
        yuv[2] = pl[1][cy * p->pitch[1] + (x | 1)];
        break;
    default:                        /* planar */
        yuv[1] = pl[1][cy * p->pitch[1] + x / 2];
        yuv[2] = pl[2][cy * p->pitch[2] + x / 2];
        break;
    }
}

static void aw_g2d_put(const AwG2dPic *p, uint8_t *const pl[3], uint32_t x,
                       uint32_t y, const uint8_t *yuv)
{
    uint32_t cy = (p->fmt >= FMT_YUV420SP_VU && p->fmt <= FMT_YUV420P) ?
                  y / 2 : y;
    unsigned bpp = aw_g2d_fmt_bpp(p->fmt);

    if (bpp) {
        uint8_t *px = pl[0] + y * p->pitch[0] + x * bpp;
        int c = 298 * (yuv[0] - 16), d = yuv[1] - 128, e = yuv[2] - 128;
        uint8_t r = MIN(MAX((c + 409 * e + 128) >> 8, 0), 255);
        uint8_t g = MIN(MAX((c - 100 * d - 208 * e + 128) >> 8, 0), 255);
        uint8_t b = MIN(MAX((c + 516 * d + 128) >> 8, 0), 255);
        uint8_t q[4];

        switch (p->fmt) {
        case FMT_ARGB8888: case FMT_XRGB8888:
            q[0] = b; q[1] = g; q[2] = r; q[3] = 0xFF;
            break;
        case FMT_ABGR8888: case FMT_XBGR8888:
            q[0] = r; q[1] = g; q[2] = b; q[3] = 0xFF;
            break;
        case FMT_RGBA8888: case FMT_RGBX8888:
            q[0] = 0xFF; q[1] = b; q[2] = g; q[3] = r;
            break;
        case FMT_BGRA8888: case FMT_BGRX8888:
            q[0] = 0xFF; q[1] = r; q[2] = g; q[3] = b;
            break;
        case FMT_RGB888:
            q[0] = b; q[1] = g; q[2] = r;
            break;
        default:
            q[0] = r; q[1] = g; q[2] = b;
            break;
        }
        memcpy(px, q, bpp);
        return;
    }
    pl[0][y * p->pitch[0] + x] = yuv[0];
    if (p->fmt == FMT_Y8 || (x & 1) ||
        (cy != y && (y & 1))) {
        return;
    }
    switch (p->fmt) {
    case FMT_YUV422SP_VU: case FMT_YUV420SP_VU:
        pl[1][cy * p->pitch[1] + x] = yuv[2];
        pl[1][cy * p->pitch[1] + x + 1] = yuv[1];
        break;
    case FMT_YUV422SP_UV: case FMT_YUV420SP_UV:
        pl[1][cy * p->pitch[1] + x] = yuv[1];
        pl[1][cy * p->pitch[1] + x + 1] = yuv[2];
        break;
    default:
        pl[1][cy * p->pitch[1] + x / 2] = yuv[1];
        pl[2][cy * p->pitch[2] + x / 2] = yuv[2];
        break;
    }
}

/* The size of plane @n of @p, in bytes. */
static uint32_t aw_g2d_plane_size(const AwG2dPic *p, int n)
{
    bool c420 = p->fmt >= FMT_YUV420SP_VU && p->fmt <= FMT_YUV420P;

    if (n == 0) {
        return p->pitch[0] * p->h;
    }
    if (p->fmt == FMT_Y8 || aw_g2d_fmt_bpp(p->fmt) ||
        (n == 2 && p->fmt != FMT_YUV422P && p->fmt != FMT_YUV420P)) {
        return 0;
    }
    return p->pitch[n] * (c420 ? (p->h + 1) / 2 : p->h);
}

static void aw_g2d_pic(AwG2dState *s, AwG2dPic *p, uint32_t fmt,
                       uint32_t size, uint32_t pitch_reg, uint32_t addr_reg,
                       uint32_t addr_step)
{
    int n;

    p->fmt = fmt;
    p->w = (size & 0x1FFF) + 1;
    p->h = ((size >> 16) & 0x1FFF) + 1;
    for (n = 0; n < 3; n++) {
        p->pitch[n] = aw_g2d_reg(s, pitch_reg + n * 4);
        p->addr[n] = aw_g2d_reg(s, addr_reg + n * addr_step);
    }
    if (!p->pitch[0]) {
        p->pitch[0] = p->w * MAX(aw_g2d_fmt_bpp(fmt), 1);
    }
    if (!p->pitch[1]) {
        p->pitch[1] = (fmt == FMT_YUV422P || fmt == FMT_YUV420P) ?
                      (p->pitch[0] + 1) / 2 : p->pitch[0];
    }
    if (!p->pitch[2]) {
        p->pitch[2] = p->pitch[1];
    }
}

/* The video layer, scaled and converted into the write-back buffer. */
static void aw_g2d_task(AwG2dState *s)
{
    uint32_t v0 = aw_g2d_reg(s, G2D_V0);
    AwG2dPic in, out;
    uint8_t *ip[3] = { NULL }, *op[3] = { NULL };
    uint32_t x, y;
    int n;

    if (!(v0 & 1)) {
        return;
    }
    aw_g2d_pic(s, &in, (v0 >> 8) & 0x3F, aw_g2d_reg(s, G2D_V0 + 0x04),
               G2D_V0 + 0x0C, G2D_V0 + 0x18, 4);
    aw_g2d_pic(s, &out, aw_g2d_reg(s, G2D_WB) & 0x3F,
               aw_g2d_reg(s, G2D_WB + 0x04), G2D_WB + 0x08, G2D_WB + 0x14, 8);
    if (!aw_g2d_fmt_ok(in.fmt) || !aw_g2d_fmt_ok(out.fmt)) {
        qemu_log_mask(LOG_UNIMP, "aw-g2d: format 0x%x -> 0x%x\n",
                      in.fmt, out.fmt);
        return;
    }
    if (s->trace) {
        qemu_log("aw-g2d: V0 %ux%u fmt 0x%x @0x%x -> WB %ux%u fmt 0x%x "
                 "@0x%x\n", in.w, in.h, in.fmt, in.addr[0], out.w, out.h,
                 out.fmt, out.addr[0]);
    }
    for (n = 0; n < 3; n++) {
        uint32_t isz = aw_g2d_plane_size(&in, n);
        uint32_t osz = aw_g2d_plane_size(&out, n);

        if (isz) {
            ip[n] = g_malloc(isz);
            aw_iommu_dma_read(s->iommu, AW_IOMMU_MASTER_G2D, in.addr[n],
                              ip[n], isz);
        }
        if (osz) {
            op[n] = g_malloc0(osz);
        }
    }
    for (y = 0; y < out.h; y++) {
        for (x = 0; x < out.w; x++) {
            uint8_t yuv[3];

            aw_g2d_get(&in, ip, (uint64_t)x * in.w / out.w,
                       (uint64_t)y * in.h / out.h, yuv);
            aw_g2d_put(&out, op, x, y, yuv);
        }
    }
    for (n = 0; n < 3; n++) {
        if (op[n]) {
            aw_iommu_dma_write(s->iommu, AW_IOMMU_MASTER_G2D, out.addr[n],
                               op[n], aw_g2d_plane_size(&out, n));
        }
        g_free(ip[n]);
        g_free(op[n]);
    }
}

static void aw_g2d_run(AwG2dState *s)
{
    aw_g2d_load_rcq(s);
    aw_g2d_task(s);
    s->regs[G2D_RCQ_STATUS >> 2] |= 0x11;
    aw_g2d_update_irq(s);
}

static uint64_t aw_g2d_read(void *opaque, hwaddr addr, unsigned size)
{
    AwG2dState *s = opaque;

    return s->regs[addr >> 2];
}

static void aw_g2d_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    AwG2dState *s = opaque;

    if (addr == G2D_RCQ_STATUS) {
        s->regs[addr >> 2] &= ~val;
        aw_g2d_update_irq(s);
        return;
    }
    s->regs[addr >> 2] = val;
    if (addr == G2D_RCQ_CTRL && (val & 1)) {
        s->regs[addr >> 2] &= ~1u;
        aw_g2d_run(s);
    } else if (addr == G2D_RCQ_IRQ_CTL) {
        aw_g2d_update_irq(s);
    }
}

static const MemoryRegionOps aw_g2d_ops = {
    .read = aw_g2d_read,
    .write = aw_g2d_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aw_g2d_reset_hold(Object *obj, ResetType type)
{
    AwG2dState *s = AW_G2D(obj);

    memset(s->regs, 0, AW_G2D_SIZE);
}

static void aw_g2d_init(Object *obj)
{
    AwG2dState *s = AW_G2D(obj);

    memory_region_init_io(&s->iomem, obj, &aw_g2d_ops, s, TYPE_AW_G2D,
                          AW_G2D_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->regs = g_malloc0(AW_G2D_SIZE);
}

static void aw_g2d_finalize(Object *obj)
{
    AwG2dState *s = AW_G2D(obj);

    g_free(s->regs);
}

static const Property aw_g2d_properties[] = {
    DEFINE_PROP_BOOL("trace", AwG2dState, trace, false),
    DEFINE_PROP_LINK("iommu", AwG2dState, iommu, TYPE_DEVICE, DeviceState *),
};

static void aw_g2d_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_g2d_reset_hold;
    device_class_set_props(dc, aw_g2d_properties);
}

static const TypeInfo aw_g2d_info = {
    .name              = TYPE_AW_G2D,
    .parent            = TYPE_SYS_BUS_DEVICE,
    .instance_size     = sizeof(AwG2dState),
    .instance_init     = aw_g2d_init,
    .instance_finalize = aw_g2d_finalize,
    .class_init        = aw_g2d_class_init,
};

static void aw_g2d_register_types(void)
{
    type_register_static(&aw_g2d_info);
}

type_init(aw_g2d_register_types)
