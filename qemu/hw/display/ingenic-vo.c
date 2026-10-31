/*
 * Ingenic VO ("ingenic,t41-vo", 0x130C0000, core INTC source 43): T41's
 * video output, sending NV12 frames out over BT.656 / BT.1120 (drivers/
 * video/ingenic_vo, /dev/jz-vo; disabled in t41.dtsi, no shipped kernel
 * probes it).  The register contract is the driver's:
 *
 *   0x004 TOP_CON    bit 0 DMA reset, bit 1 reset
 *   0x008 TOP_STATUS bit 0 the DMA reset's acknowledge, a moment after it
 *                    is asserted (the driver waits for it); write 1 clears
 *   0x00C INT_EN  0x010 INT_VAL  0x014 INT_CLC (a set bit clears; the
 *                    driver sets and clears it): bit 0 frame done (8 and
 *                    12, the driver's other two, are its error sources)
 *   0x100 SIZE h << 16 | w   0x104 STRIDE uv << 16 | y
 *   0x108 / 0x10C Y / UV address: a write of UV queues the frame, and
 *                    ADDR_INFO (0x128) bit 3 says one is queued
 *   0x114 CON_PAR line threshold << 16 | transmit (12) | BT mode
 *   0x118 VBLK_PAR  0x11C HBLK_PAR  0x110 BLK_INFO  0x124 TIME_OUT
 *   0x130 YUV_CLIP: storage
 *   0x120 TRAN_EN bit 0 transmit
 *
 * While TRAN_EN is set a queued frame goes out over the next frame time
 * (1 / "fps", 30): the picture shows on the machine's display (a QEMU
 * graphic console, so screendump captures it), ADDR_INFO bit 3 clears and
 * INT_VAL bit 0 raises the interrupt.
 *
 * Copyright (c) 2026 OpenIPC.
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
#include "qom/object.h"

#define TYPE_INGENIC_VO "ingenic-vo"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicVoState, INGENIC_VO)

#define VO_REGS         0x200

#define R_TOP_CON       0x004
#define R_TOP_STATUS    0x008
#define R_INT_EN        0x00C
#define R_INT_VAL       0x010
#define R_INT_CLC       0x014
#define R_SIZE          0x100
#define R_STRIDE        0x104
#define R_Y_ADDR        0x108
#define R_UV_ADDR       0x10C
#define R_TRAN_EN       0x120
#define R_ADDR_INFO     0x128

#define CON_DMA_RESET   (1u << 0)
#define ADDR_QUEUED     (1u << 3)
#define INT_FRAME       (1u << 0)

#define ACK_NS          (10 * SCALE_US)
#define MAX_DIM         4096

struct IngenicVoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *frame_timer;
    QEMUTimer *ack_timer;
    QemuConsole *con;
    uint32_t regs[VO_REGS / 4];
    uint32_t fps;
    uint32_t frames;
};

#define VREG(s, off)    ((s)->regs[(off) >> 2])

static void vo_update(IngenicVoState *s)
{
    qemu_set_irq(s->irq, (VREG(s, R_INT_VAL) & VREG(s, R_INT_EN)) != 0);
}

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* The queued NV12 frame onto the console, BT.601 */
static void vo_show(IngenicVoState *s)
{
    uint32_t w = VREG(s, R_SIZE) & 0xffff, h = VREG(s, R_SIZE) >> 16;
    uint32_t ys = VREG(s, R_STRIDE) & 0xffff, uvs = VREG(s, R_STRIDE) >> 16;
    DisplaySurface *surf;
    uint8_t *yl, *uvl;
    uint32_t x, y;

    if (!w || !h || w > MAX_DIM || h > MAX_DIM) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vo: bad size %ux%u\n", w, h);
        return;
    }
    surf = qemu_console_surface(s->con);
    if (!surf || surface_width(surf) != w || surface_height(surf) != h) {
        qemu_console_resize(s->con, w, h);
        surf = qemu_console_surface(s->con);
    }
    yl = g_malloc(w);
    uvl = g_malloc(w);
    for (y = 0; y < h; y++) {
        uint32_t *row = (uint32_t *)(surface_data(surf) +
                                     y * surface_stride(surf));

        dma_memory_read(&address_space_memory,
                        VREG(s, R_Y_ADDR) + (hwaddr)y * ys, yl, w,
                        MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory,
                        VREG(s, R_UV_ADDR) + (hwaddr)(y / 2) * uvs, uvl,
                        w & ~1u, MEMTXATTRS_UNSPECIFIED);
        for (x = 0; x < w; x++) {
            int c = 298 * (yl[x] - 16), d = uvl[x & ~1u] - 128;
            int e = uvl[(x & ~1u) + 1] - 128;
            uint8_t r = clamp8((c + 409 * e + 128) >> 8);
            uint8_t g = clamp8((c - 100 * d - 208 * e + 128) >> 8);
            uint8_t b = clamp8((c + 516 * d + 128) >> 8);

            row[x] = (r << 16) | (g << 8) | b;
        }
    }
    g_free(yl);
    g_free(uvl);
    dpy_gfx_update_full(s->con);
}

static void vo_arm(IngenicVoState *s)
{
    if ((VREG(s, R_TRAN_EN) & 1) && (VREG(s, R_ADDR_INFO) & ADDR_QUEUED) &&
        !timer_pending(s->frame_timer)) {
        timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
    }
}

static void vo_frame(void *opaque)
{
    IngenicVoState *s = opaque;

    if (!(VREG(s, R_TRAN_EN) & 1) || !(VREG(s, R_ADDR_INFO) & ADDR_QUEUED)) {
        return;
    }
    vo_show(s);
    s->frames++;
    VREG(s, R_ADDR_INFO) &= ~ADDR_QUEUED;
    VREG(s, R_INT_VAL) |= INT_FRAME;
    vo_update(s);
}

static void vo_ack(void *opaque)
{
    IngenicVoState *s = opaque;

    if (VREG(s, R_TOP_CON) & CON_DMA_RESET) {
        VREG(s, R_TOP_STATUS) |= 1;
    }
}

static uint64_t ingenic_vo_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicVoState *s = opaque;

    return off < VO_REGS ? VREG(s, off) : 0;
}

static void ingenic_vo_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    IngenicVoState *s = opaque;
    uint32_t old;

    if (off >= VO_REGS) {
        return;
    }
    switch (off) {
    case R_TOP_CON:
        old = VREG(s, R_TOP_CON);
        VREG(s, R_TOP_CON) = val;
        if ((val & CON_DMA_RESET) && !(old & CON_DMA_RESET)) {
            timer_del(s->frame_timer);
            VREG(s, R_ADDR_INFO) &= ~ADDR_QUEUED;
            timer_mod(s->ack_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ACK_NS);
        }
        if (val & 2) {
            VREG(s, R_TRAN_EN) = 0;
            VREG(s, R_INT_VAL) = 0;
        }
        break;
    case R_TOP_STATUS:
        VREG(s, R_TOP_STATUS) &= ~val;
        break;
    case R_INT_VAL:
    case R_ADDR_INFO:
        break;                              /* read only */
    case R_INT_CLC:
        VREG(s, R_INT_VAL) &= ~val;
        VREG(s, R_INT_CLC) = val;
        break;
    case R_UV_ADDR:
        VREG(s, R_UV_ADDR) = val;
        VREG(s, R_ADDR_INFO) |= ADDR_QUEUED;
        break;
    default:
        VREG(s, off) = val;
        break;
    }
    vo_arm(s);
    vo_update(s);
}

static const MemoryRegionOps ingenic_vo_ops = {
    .read = ingenic_vo_read,
    .write = ingenic_vo_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const GraphicHwOps ingenic_vo_gfx_ops = {
};

static void ingenic_vo_reset(DeviceState *dev)
{
    IngenicVoState *s = INGENIC_VO(dev);

    timer_del(s->frame_timer);
    timer_del(s->ack_timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->frames = 0;
    vo_update(s);
}

static void ingenic_vo_realize(DeviceState *dev, Error **errp)
{
    IngenicVoState *s = INGENIC_VO(dev);

    s->con = graphic_console_init(dev, 0, &ingenic_vo_gfx_ops, s);
    qemu_console_resize(s->con, 640, 360);
}

static void ingenic_vo_init(Object *obj)
{
    IngenicVoState *s = INGENIC_VO(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_vo_ops, s,
                          TYPE_INGENIC_VO, 0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vo_frame, s);
    s->ack_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vo_ack, s);
    object_property_add_uint32_ptr(obj, "frames", &s->frames,
                                   OBJ_PROP_FLAG_READ);
}

static const Property ingenic_vo_props[] = {
    DEFINE_PROP_UINT32("fps", IngenicVoState, fps, 30),
};

static const VMStateDescription vmstate_ingenic_vo = {
    .name = TYPE_INGENIC_VO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicVoState, VO_REGS / 4),
        VMSTATE_UINT32(frames, IngenicVoState),
        VMSTATE_TIMER_PTR(frame_timer, IngenicVoState),
        VMSTATE_TIMER_PTR(ack_timer, IngenicVoState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_vo_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ingenic_vo_realize;
    device_class_set_legacy_reset(dc, ingenic_vo_reset);
    device_class_set_props(dc, ingenic_vo_props);
    dc->vmsd = &vmstate_ingenic_vo;
}

static const TypeInfo ingenic_vo_info = {
    .name = TYPE_INGENIC_VO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicVoState),
    .instance_init = ingenic_vo_init,
    .class_init = ingenic_vo_class_init,
};

static void ingenic_vo_register_types(void)
{
    type_register_static(&ingenic_vo_info);
}

type_init(ingenic_vo_register_types)
