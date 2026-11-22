/*
 * Rockchip VOP lite ("rockchip,rv1126-vop", 0xFFB00000; the PX30 /
 * RK3366-lite register layout), the RV1126's display controller.  From
 * the vendor kernel's drivers/gpu/drm/rockchip/rockchip_vop_reg.c
 * (develop-4.19, rv1126_vop):
 *
 *   0x000 REG_CFG_DONE   0x018 SYS_CTRL2 (bit 1 standby)
 *   0x030 LINE_FLAG      0x034 INTR_EN / 0x038 INTR_CLEAR (hiword masked)
 *   0x03c INTR_STATUS    bit 0 frame start, 1 new frame start, 3 / 4
 *                        line flag 0 / 1, ... (enabled ones only)
 *   0x100..0x10c DSP_HTOTAL_HS_END, HACT_ST_END, VTOTAL_VS_END,
 *                VACT_ST_END
 *   window 2 (the primary plane) at base 0xe0 + RK3368 WIN2 offsets:
 *   0x190 CTRL0  bit 4 enable, bits 6:5 format (0 ARGB8888, 1 RGB888,
 *                2 RGB565), bit 20 R/B swap
 *   0x198 VIR    stride in 32-bit words   0x1a0 MST  frame buffer
 *   0x1a4 DSP_INFO  (width - 1) | (height - 1) << 16
 *
 * Unless the controller is in standby, a frame clock ("refresh-hz", 60
 * by default; the dot clock is the CRU's, not modelled) raises frame
 * start and both line flags each frame: the DRM driver's vblank waits
 * on them, and with none it times out ("vblank wait timed out").  The
 * enabled window is scanned out to a QEMU graphic console, through the
 * "iommu" link's translation when the driver turns its paging on.  Other
 * registers are storage.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "ui/console.h"
#include "hw/misc/rk-iommu.h"
#include "qom/object.h"

#define TYPE_RK_VOP "rk-vop"
OBJECT_DECLARE_SIMPLE_TYPE(RkVopState, RK_VOP)

#define REGS_SIZE       0xe00
#define R_SYS_CTRL2     0x018
#define R_INTR_EN       0x034
#define R_INTR_CLEAR    0x038
#define R_INTR_STATUS   0x03c
#define R_WIN2_CTRL0    0x190
#define R_WIN2_VIR      0x198
#define R_WIN2_MST      0x1a0
#define R_WIN2_DSP_INFO 0x1a4

#define SYS2_STANDBY    (1u << 1)
#define INT_FS          (1u << 0)
#define INT_FS_NEW      (1u << 1)
#define INT_LINE_FLAG0  (1u << 3)
#define INT_LINE_FLAG1  (1u << 4)

struct RkVopState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *frame;
    QemuConsole *con;

    uint32_t regs[REGS_SIZE / 4];
    uint32_t intr_en, intr_status;
    uint32_t refresh_hz;
    Object *iommu;          /* link: the MMU the scan-out reads through */
};

static void rk_vop_update_irq(RkVopState *s)
{
    qemu_set_irq(s->irq, !!(s->intr_en & s->intr_status));
}

static void rk_vop_frame(void *opaque)
{
    RkVopState *s = opaque;

    if (!(s->regs[R_SYS_CTRL2 / 4] & SYS2_STANDBY)) {
        /* only what is enabled latches: the driver takes a pending
         * frame start with the interrupt off as "vblank IRQ stuck" */
        s->intr_status |= (INT_FS | INT_FS_NEW | INT_LINE_FLAG0 |
                           INT_LINE_FLAG1) & s->intr_en;
        rk_vop_update_irq(s);
    }
    timer_mod(s->frame, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / MAX(s->refresh_hz, 1));
}

static void rk_vop_gfx_update(void *opaque)
{
    RkVopState *s = opaque;
    uint32_t ctrl = s->regs[R_WIN2_CTRL0 / 4];
    uint32_t info = s->regs[R_WIN2_DSP_INFO / 4];
    uint32_t w = (info & 0xfff) + 1, h = ((info >> 16) & 0xfff) + 1;
    uint32_t fmt = (ctrl >> 5) & 3, swap = (ctrl >> 20) & 1;
    uint32_t bpp = fmt == 0 ? 4 : fmt == 1 ? 3 : 2;
    uint32_t stride = (s->regs[R_WIN2_VIR / 4] & 0x1fff) * 4;
    hwaddr base = s->regs[R_WIN2_MST / 4];
    DisplaySurface *ds;
    g_autofree uint8_t *line = NULL;
    uint32_t x, y;

    /* MST 0 is a buffer too: the DRM driver's IOVAs start at 0 */
    if (!(ctrl & (1u << 4)) || w > 4096 || h > 4096) {
        return;
    }
    if (!stride) {
        stride = w * bpp;
    }
    ds = qemu_console_surface(s->con);
    if (!ds || surface_width(ds) != w || surface_height(ds) != h) {
        qemu_console_resize(s->con, w, h);
        ds = qemu_console_surface(s->con);
    }
    line = g_malloc(w * bpp);
    for (y = 0; y < h; y++) {
        uint32_t *out = (uint32_t *)(surface_data(ds) +
                                     y * surface_stride(ds));

        if (s->iommu) {
            if (!rk_iommu_dma_read(s->iommu, base + y * stride, line, w * bpp)) {
                memset(line, 0, w * bpp);       /* unmapped: black */
            }
        } else {
            dma_memory_read(&address_space_memory, base + y * stride, line,
                            w * bpp, MEMTXATTRS_UNSPECIFIED);
        }
        for (x = 0; x < w; x++) {
            const uint8_t *p = line + x * bpp;
            uint32_t r, g, b;

            if (bpp == 2) {
                uint16_t v = lduw_le_p(p);

                r = (v >> 11) << 3;
                g = ((v >> 5) & 0x3f) << 2;
                b = (v & 0x1f) << 3;
            } else {
                b = p[0];
                g = p[1];
                r = p[2];
            }
            out[x] = swap ? (b << 16 | g << 8 | r) : (r << 16 | g << 8 | b);
        }
    }
    dpy_gfx_update(s->con, 0, 0, w, h);
}

static const GraphicHwOps rk_vop_gfx_ops = {
    .gfx_update = rk_vop_gfx_update,
};

static uint64_t rk_vop_read(void *opaque, hwaddr off, unsigned size)
{
    RkVopState *s = opaque;

    switch (off) {
    case R_INTR_EN:
        return s->intr_en;
    case R_INTR_CLEAR:
        return 0;
    case R_INTR_STATUS:
        return s->intr_status;
    }
    return s->regs[off / 4];
}

static void rk_vop_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    RkVopState *s = opaque;
    uint32_t mask = val >> 16;

    switch (off) {
    case R_INTR_EN:
        s->intr_en = (s->intr_en & ~mask) | (val & mask);
        break;
    case R_INTR_CLEAR:
        s->intr_status &= ~(val & mask);
        break;
    case R_INTR_STATUS:
        return;
    default:
        s->regs[off / 4] = val;
        return;
    }
    rk_vop_update_irq(s);
}

static const MemoryRegionOps rk_vop_ops = {
    .read = rk_vop_read,
    .write = rk_vop_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_vop_reset(DeviceState *dev)
{
    RkVopState *s = RK_VOP(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_SYS_CTRL2 / 4] = SYS2_STANDBY;
    s->intr_en = s->intr_status = 0;
    rk_vop_update_irq(s);
    timer_mod(s->frame, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / MAX(s->refresh_hz, 1));
}

static void rk_vop_realize(DeviceState *dev, Error **errp)
{
    RkVopState *s = RK_VOP(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &rk_vop_ops, s, TYPE_RK_VOP,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->frame = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_vop_frame, s);
    s->con = graphic_console_init(dev, 0, &rk_vop_gfx_ops, s);
}

static const Property rk_vop_properties[] = {
    DEFINE_PROP_UINT32("refresh-hz", RkVopState, refresh_hz, 60),
    DEFINE_PROP_LINK("iommu", RkVopState, iommu, "rk-iommu", Object *),
};

static const VMStateDescription rk_vop_vmstate = {
    .name = TYPE_RK_VOP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkVopState, REGS_SIZE / 4),
        VMSTATE_UINT32(intr_en, RkVopState),
        VMSTATE_UINT32(intr_status, RkVopState),
        VMSTATE_TIMER_PTR(frame, RkVopState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_vop_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rk_vop_realize;
    device_class_set_legacy_reset(dc, rk_vop_reset);
    device_class_set_props(dc, rk_vop_properties);
    dc->vmsd = &rk_vop_vmstate;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo rk_vop_info = {
    .name = TYPE_RK_VOP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkVopState),
    .class_init = rk_vop_class_init,
};

static void rk_vop_register_types(void)
{
    type_register_static(&rk_vop_info);
}

type_init(rk_vop_register_types)
