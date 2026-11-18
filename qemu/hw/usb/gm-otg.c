/*
 * Grain Media FOTG210 USB 2.0 OTG controller, host role.
 *
 * Faraday's FOTG210 at 0x93000000 on the GM813x (the vendor EHCI glue
 * drivers/usb/host/ehci-fotg2xx.c, fotg2xx_opt.c and their -macro.h
 * headers in the GM 3.3 kernel).  Its host block is the FUSBH200 one --
 * EHCI capabilities at 0, operational registers at 0x10, the one PORTSC
 * at 0x30 and the bus control register at 0x40 -- which is QEMU's
 * fusbh200-ehci-usb.  Above it sit the OTG registers:
 *
 *   0x80 OTG control/status: bits 23:22 the speed of the device on the
 *        host port (0 full, 1 low, 2 high -- what the vendor
 *        ehci_port_speed() reads instead of PORTSC), bit 21 the ID pin
 *        (0 = A device), bit 20 the current role (0 = host)
 *   0x84 OTG interrupt status (write 1 to clear)   0x88 its enables
 *   0xC0 global interrupt status: bit 2 the host controller's
 *   0xC4 global interrupt mask (1 masks)
 *   0x100... the device controller, unused in host mode
 *
 * Reset leaves an A device in the host role, as the cameras' ID pin
 * reads, so the driver drives VBUS and starts the EHCI.  The port speed
 * reads high: QEMU's EHCI only takes high-speed devices on its ports.
 * The interrupt line is the host interrupt unless masked in 0xC4.
 * -device usb-storage and friends plug into its root port.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/usb/hcd-ehci.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_OTG "gm-otg"
OBJECT_DECLARE_SIMPLE_TYPE(GmOtgState, GM_OTG)

#define GM_OTG_SIZE         0x1000
#define GM_OTG_GLUE_BASE    0x80
#define GM_OTG_GLUE_SIZE    (GM_OTG_SIZE - GM_OTG_GLUE_BASE)
#define GM_OTG_NREGS        (GM_OTG_GLUE_SIZE / 4)

#define R_OTG_CSR           0x80
#define R_OTG_ISR           0x84
#define R_GLB_ISR           0xC0
#define R_GLB_IMR           0xC4

#define GLB_HOST            (1u << 2)
#define CSR_SPEED_HIGH      (2u << 22)
#define CSR_INPUTS          ((3u << 22) | (1u << 21) | (1u << 20))

struct GmOtgState {
    SysBusDevice parent_obj;

    MemoryRegion container;
    MemoryRegion glue;
    FUSBH200EHCIState ehci;
    qemu_irq irq;
    bool host_irq;
    uint32_t regs[GM_OTG_NREGS];
};

static uint32_t *gm_otg_reg(GmOtgState *s, hwaddr off)
{
    return &s->regs[(off - GM_OTG_GLUE_BASE) / 4];
}

static void gm_otg_update(GmOtgState *s)
{
    qemu_set_irq(s->irq, s->host_irq &&
                 !(*gm_otg_reg(s, R_GLB_IMR) & GLB_HOST));
}

static void gm_otg_host_irq(void *opaque, int n, int level)
{
    GmOtgState *s = opaque;

    s->host_irq = level;
    gm_otg_update(s);
}

static uint64_t gm_otg_read(void *opaque, hwaddr addr, unsigned size)
{
    GmOtgState *s = opaque;
    hwaddr off = addr + GM_OTG_GLUE_BASE;

    if (off == R_OTG_CSR) {
        return *gm_otg_reg(s, off) | CSR_SPEED_HIGH;
    }
    if (off == R_GLB_ISR) {
        return (*gm_otg_reg(s, off) & ~GLB_HOST) |
               (s->host_irq ? GLB_HOST : 0);
    }
    return *gm_otg_reg(s, off);
}

static void gm_otg_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    GmOtgState *s = opaque;
    hwaddr off = addr + GM_OTG_GLUE_BASE;

    switch (off) {
    case R_OTG_CSR:
        /* the speed, the ID pin and the role are inputs */
        *gm_otg_reg(s, off) = val & ~CSR_INPUTS;
        break;
    case R_OTG_ISR:
    case R_GLB_ISR:
        /* write 1 to clear; the host bit follows the EHCI */
        *gm_otg_reg(s, off) &= ~val;
        break;
    default:
        *gm_otg_reg(s, off) = val;
        break;
    }
    gm_otg_update(s);
}

static const MemoryRegionOps gm_otg_ops = {
    .read = gm_otg_read,
    .write = gm_otg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_otg_reset(DeviceState *dev)
{
    GmOtgState *s = GM_OTG(dev);

    memset(s->regs, 0, sizeof(s->regs));
    gm_otg_update(s);
}

static void gm_otg_init(Object *obj)
{
    GmOtgState *s = GM_OTG(obj);

    object_initialize_child(obj, "ehci", &s->ehci, TYPE_FUSBH200_EHCI);
}

static void gm_otg_realize(DeviceState *dev, Error **errp)
{
    GmOtgState *s = GM_OTG(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    SysBusDevice *ehci = SYS_BUS_DEVICE(&s->ehci);

    if (!sysbus_realize(ehci, errp)) {
        return;
    }
    memory_region_init(&s->container, OBJECT(s), TYPE_GM_OTG, GM_OTG_SIZE);
    memory_region_add_subregion(&s->container, 0,
                                sysbus_mmio_get_region(ehci, 0));
    memory_region_init_io(&s->glue, OBJECT(s), &gm_otg_ops, s, "gm-otg.glue",
                          GM_OTG_GLUE_SIZE);
    memory_region_add_subregion_overlap(&s->container, GM_OTG_GLUE_BASE,
                                        &s->glue, 1);
    sysbus_init_mmio(sbd, &s->container);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(dev, gm_otg_host_irq, "host-irq", 1);
    sysbus_connect_irq(ehci, 0, qdev_get_gpio_in_named(dev, "host-irq", 0));
}

static const VMStateDescription vmstate_gm_otg = {
    .name = TYPE_GM_OTG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(host_irq, GmOtgState),
        VMSTATE_UINT32_ARRAY(regs, GmOtgState, GM_OTG_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_otg_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_otg_realize;
    device_class_set_legacy_reset(dc, gm_otg_reset);
    dc->vmsd = &vmstate_gm_otg;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static const TypeInfo gm_otg_info = {
    .name           = TYPE_GM_OTG,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmOtgState),
    .instance_init  = gm_otg_init,
    .class_init     = gm_otg_class_init,
};

static void gm_otg_register_types(void)
{
    type_register_static(&gm_otg_info);
}

type_init(gm_otg_register_types)
