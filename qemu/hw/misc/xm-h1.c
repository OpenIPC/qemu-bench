/*
 * Hantro H1 video encoder, as XM530 carries it (0x20070000, SPI 45).
 *
 * The vendor driver is xm530_h264.ko, Hantro's own hx280enc ("Hantro
 * 6280/7280/8270/8290 Encoder driver"): at load it reads swreg0 and wants a
 * product ID of 0x6280, 0x7280, 0x8270 or 0x8290 in the top half, else
 * "HW not found" and no VENC device for the SDK to open.
 *
 * Stage one: the register file and the ID, so the driver loads.  Encoding
 * comes later.
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

#define TYPE_XM_H1 "xm-h1"
OBJECT_DECLARE_SIMPLE_TYPE(XmH1State, XM_H1)

#define H1_REGS_SIZE    0x1000
#define H1_SWREG0_ID    0x000
/* 8290 is the H1 in the driver's list; low half is the build. */
#define H1_ID           0x82901000u
/* swreg63, the synthesis configuration Hantro's EWL reads before any
 * encoder will initialise: max width (bits 11:0, pixels), and the codecs
 * the silicon has -- VS 24, JPEG 26, H.264 27, RGB 28 -- with an AXI bus
 * (type 1, bits 23:20) 64 bits wide (bits 13:12 = 1). */
#define H1_SWREG63_CFG  0x0FC
#define H1_CFG          ((1u << 28) | (1u << 27) | (1u << 26) | (1u << 24) | \
                         (1u << 20) | (1u << 12) | 1920)

struct XmH1State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[H1_REGS_SIZE / 4];
};

static uint64_t xm_h1_read(void *opaque, hwaddr off, unsigned size)
{
    XmH1State *s = XM_H1(opaque);

    if (off == H1_SWREG0_ID) {
        return H1_ID;
    }
    if (off == H1_SWREG63_CFG) {
        return H1_CFG;
    }
    return s->regs[off / 4];
}

static void xm_h1_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    XmH1State *s = XM_H1(opaque);

    if (off == H1_SWREG0_ID) {
        return;
    }
    s->regs[off / 4] = val;
}

static const MemoryRegionOps xm_h1_ops = {
    .read = xm_h1_read,
    .write = xm_h1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_h1_reset(DeviceState *dev)
{
    XmH1State *s = XM_H1(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void xm_h1_init(Object *obj)
{
    XmH1State *s = XM_H1(obj);

    memory_region_init_io(&s->iomem, obj, &xm_h1_ops, s, TYPE_XM_H1,
                          H1_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void xm_h1_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_h1_reset);
}

static const TypeInfo xm_h1_info = {
    .name          = TYPE_XM_H1,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmH1State),
    .instance_init = xm_h1_init,
    .class_init    = xm_h1_class_init,
};

static void xm_h1_register_types(void)
{
    type_register_static(&xm_h1_info);
}

type_init(xm_h1_register_types)
