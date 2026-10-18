/*
 * Anyka AK39xx USB 2.0 OTG controller, with nothing plugged in.
 *
 * A Mentor MUSB-style core at 0x20200000 (kernel include/plat-anyka/
 * usb-reg-define.h): FADDR / POWER at 0, the TX / RX / common interrupt
 * status at 0x02 / 0x04 / 0x0A (read to clear), DEVCTL at 0x60, the DMA
 * interrupt status at 0x200.  The cameras' vendor scripts load the host
 * driver to look for a USB Wi-Fi dongle; as plain storage this block
 * handed the driver back its own bits as connect events and line state,
 * and the hub layer retried "enumerating" a device forever.  Here the
 * registers keep what is written, the interrupt status never reports
 * anything and DEVCTL never sees a device on the line, which is what the
 * driver finds on a board with the port empty.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_AK_USB "ak-usb"
OBJECT_DECLARE_SIMPLE_TYPE(AkUsbState, AK_USB)

#define AK_USB_SIZE         0x1000

#define R_INTRTX            0x02
#define R_INTRRX            0x04
#define R_INTRUSB           0x0A
#define R_DEVCTL            0x60
#define R_DMA_INTR          0x200

/* DEVCTL: VBUS level (4:3), low / full speed device on the line (5 / 6). */
#define DEVCTL_LINE_STATE   0x78

struct AkUsbState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint8_t regs[AK_USB_SIZE];
};

static bool ak_usb_is_status(hwaddr offset)
{
    return (offset >= R_INTRTX && offset < R_INTRTX + 4) ||
           offset == R_INTRUSB ||
           (offset >= R_DMA_INTR && offset < R_DMA_INTR + 4);
}

static uint64_t ak_usb_read(void *opaque, hwaddr offset, unsigned size)
{
    AkUsbState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        hwaddr a = offset + i;
        uint8_t b = s->regs[a];

        if (ak_usb_is_status(a)) {
            b = 0;
        } else if (a == R_DEVCTL) {
            b &= ~DEVCTL_LINE_STATE;
        }
        val |= (uint64_t)b << (8 * i);
    }
    return val;
}

static void ak_usb_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    AkUsbState *s = opaque;
    unsigned i;

    for (i = 0; i < size; i++) {
        s->regs[offset + i] = val >> (8 * i);
    }
}

static const MemoryRegionOps ak_usb_ops = {
    .read = ak_usb_read,
    .write = ak_usb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void ak_usb_reset(DeviceState *dev)
{
    AkUsbState *s = AK_USB(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void ak_usb_init(Object *obj)
{
    AkUsbState *s = AK_USB(obj);

    memory_region_init_io(&s->iomem, obj, &ak_usb_ops, s, TYPE_AK_USB,
                          AK_USB_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_ak_usb = {
    .name = TYPE_AK_USB,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, AkUsbState, AK_USB_SIZE),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_usb_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_ak_usb;
    device_class_set_legacy_reset(dc, ak_usb_reset);
}

static const TypeInfo ak_usb_info = {
    .name          = TYPE_AK_USB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkUsbState),
    .instance_init = ak_usb_init,
    .class_init    = ak_usb_class_init,
};

static void ak_usb_register_types(void)
{
    type_register_static(&ak_usb_info);
}

type_init(ak_usb_register_types)
