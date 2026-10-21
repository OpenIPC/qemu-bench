/*
 * Novatek NA51055 / NA51089 USB 2.0 controller, host side.
 *
 * A Faraday FOTG200 dual-role core at 0xF0600000 (SPI 27): the vendor
 * kernel's host driver is drivers/usb/host/ehci-na51089.c plus
 * CONFIG_USB_NVTIVOT_HCD hooks in the EHCI core; the device side
 * (fotg200-udc) is not modelled.  Its EHCI is laid out like Faraday's
 * FUSBH200, which QEMU has: capabilities at 0x00, operational registers
 * at 0x10, no CONFIGFLAG, PORTSC of the one port at 0x30 (ehci_def.h's
 * CONFIG_USB_NVTIVOT_HCD layout), and Faraday's own registers from 0x34.
 * This device is that EHCI with the rest of the 64 KiB block behind it
 * ("glue", sysbus region 1), storage for what the driver programs:
 *
 *   0x80   OTG control/status: A_BUS_REQ [4], A_BUS_DROP [5], and the
 *          speed of the device on the port in [23:22] (0 full, 1 low,
 *          2 high), which ehci_hub_control() reads in place of PORTSC's
 *          (has_tt, ehci_port_speed())
 *   0x100  device-side main control, 0x1C8 PHY suspend, 0x30C PHY trim,
 *   0x400  USB ID / VBUS, 0x1000.. the UTMI PHY (U2PHY_SETREG): storage
 *
 * The driver finds the block at 0xFF600000 rather than 0xF0600000 unless
 * TOP's version word is 0x48210000 (nvtim_usb_ehci_probe() adds
 * 0x0F000000 to the resource), so on NA51089 the machine maps it at
 * both.  The EHCI's MMIO region is a pure container: offsets it leaves
 * unclaimed fall through to the glue region, mapped beneath it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "hw/usb/hcd-ehci.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_USB "nvt-usb"
OBJECT_DECLARE_TYPE(NvtUsbState, NvtUsbClass, NVT_USB)

#define NVT_USB_SIZE        0x10000
#define NVT_USB_OTG_CSR     0x80
#define OTG_CSR_SPEED_SHIFT 22
#define OTG_CSR_SPEED_MASK  (3u << OTG_CSR_SPEED_SHIFT)

struct NvtUsbState {
    FUSBH200EHCIState parent_obj;

    MemoryRegion glue;
    uint32_t regs[NVT_USB_SIZE / 4];
};

struct NvtUsbClass {
    SysBusEHCIClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

/* FOTG200's speed code for what is on the port: 0 full, 1 low, 2 high. */
static uint32_t nvt_usb_port_speed(NvtUsbState *s)
{
    USBDevice *dev = SYS_BUS_EHCI(s)->ehci.ports[0].dev;

    if (!dev) {
        return 0;
    }
    switch (dev->speed) {
    case USB_SPEED_LOW:
        return 1;
    case USB_SPEED_FULL:
        return 0;
    default:
        return 2;
    }
}

static uint64_t nvt_usb_glue_read(void *opaque, hwaddr off, unsigned size)
{
    NvtUsbState *s = opaque;
    uint32_t v = s->regs[off / 4];

    if (off == NVT_USB_OTG_CSR) {
        v = (v & ~OTG_CSR_SPEED_MASK) |
            (nvt_usb_port_speed(s) << OTG_CSR_SPEED_SHIFT);
    }
    return v;
}

static void nvt_usb_glue_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    NvtUsbState *s = opaque;

    s->regs[off / 4] = val;
}

static const MemoryRegionOps nvt_usb_glue_ops = {
    .read = nvt_usb_glue_read,
    .write = nvt_usb_glue_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_usb_reset_hold(Object *obj, ResetType type)
{
    NvtUsbClass *c = NVT_USB_GET_CLASS(obj);

    if (c->parent_phases.hold) {
        c->parent_phases.hold(obj, type);
    }
    memset(NVT_USB(obj)->regs, 0, sizeof(NVT_USB(obj)->regs));
}

static void nvt_usb_init(Object *obj)
{
    NvtUsbState *s = NVT_USB(obj);

    memory_region_init_io(&s->glue, obj, &nvt_usb_glue_ops, s,
                          "nvt-usb-glue", NVT_USB_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->glue);
}

static const VMStateDescription vmstate_nvt_usb_glue = {
    .name = "nvt-usb-glue",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtUsbState, NVT_USB_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

/* The EHCI keeps its own vmsd (vmstate_ehci_sysbus); the glue's rides
 * alongside it. */
static void nvt_usb_realize(DeviceState *dev, Error **errp)
{
    NvtUsbClass *c = NVT_USB_GET_CLASS(dev);
    Error *err = NULL;

    c->parent_realize(dev, &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    vmstate_register(VMSTATE_IF(dev), VMSTATE_INSTANCE_ID_ANY,
                     &vmstate_nvt_usb_glue, dev);
}

static void nvt_usb_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    NvtUsbClass *c = NVT_USB_CLASS(klass);

    device_class_set_parent_realize(dc, nvt_usb_realize, &c->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, nvt_usb_reset_hold, NULL,
                                       &c->parent_phases);
}

static const TypeInfo nvt_usb_info = {
    .name = TYPE_NVT_USB,
    .parent = TYPE_FUSBH200_EHCI,
    .instance_size = sizeof(NvtUsbState),
    .instance_init = nvt_usb_init,
    .class_size = sizeof(NvtUsbClass),
    .class_init = nvt_usb_class_init,
};

static void nvt_usb_register_types(void)
{
    type_register_static(&nvt_usb_info);
}

type_init(nvt_usb_register_types)
