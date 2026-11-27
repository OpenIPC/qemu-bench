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
 *
 * With "host" set (the AV130, whose two interrupts the machine wires) the
 * core is a Mentor MUSB host instead (hw/usb/hcd-musb.c) with one port on
 * its own USB bus, so `-device usb-kbd` and the like plug in and the
 * vendor's prebuilt ak_hcd enumerates them over EP0 through the FIFO, as
 * on the cameras' Wi-Fi dongles.  The registers at 0x200 are the MUSB
 * HSDMA block: kept as storage, no transfer is moved by it yet.  Interrupt
 * 0 is the core's (ak_hcd's usb_add_hcd()), 1 the DMA's.  The MUSB state
 * is not in the migration stream: with the port empty there is none to
 * keep, and a device on the port blocks migration (hcd-musb.c).
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/usb/hcd-musb.h"

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

    bool host;
    MUSBState *musb;
    uint32_t lines;             /* level of each musb_irq_* source */
    qemu_irq irq;
    qemu_irq dma_irq;
};

/* The musb_irq_* sources that are interrupts (not VBUS / session). */
#define AK_USB_CORE_LINES   (((1u << musb_irq_max) - 1) & \
                             ~(1u << musb_set_vbus | 1u << musb_set_session))

static void ak_usb_musb_line(void *opaque, int line, int level)
{
    AkUsbState *s = opaque;

    if (level) {
        s->lines |= 1u << line;
    } else {
        s->lines &= ~(1u << line);
    }
    qemu_set_irq(s->irq, !!(s->lines & AK_USB_CORE_LINES));
}

static unsigned ak_usb_size_index(unsigned size)
{
    return size == 1 ? 0 : size == 2 ? 1 : 2;
}

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

    if (s->host && offset < R_DMA_INTR) {
        return musb_read[ak_usb_size_index(size)](s->musb, offset);
    }

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

    if (s->host && offset < R_DMA_INTR) {
        musb_write[ak_usb_size_index(size)](s->musb, offset, val);
        return;
    }
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
    if (s->musb) {
        musb_reset(s->musb);
        s->lines = 0;
        qemu_irq_lower(s->irq);
    }
}

static void ak_usb_realize(DeviceState *dev, Error **errp)
{
    AkUsbState *s = AK_USB(dev);

    if (!s->host) {
        return;
    }
    qdev_init_gpio_in(dev, ak_usb_musb_line, musb_irq_max);
    s->musb = musb_init(dev, 0);
    /* ak_hcd addresses the device through FADDR, not TXFUNCADDR. */
    musb_set_single_point(s->musb, true);
}

static void ak_usb_init(Object *obj)
{
    AkUsbState *s = AK_USB(obj);

    memory_region_init_io(&s->iomem, obj, &ak_usb_ops, s, TYPE_AK_USB,
                          AK_USB_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->dma_irq);
}

static const Property ak_usb_properties[] = {
    DEFINE_PROP_BOOL("host", AkUsbState, host, false),
};

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
    dc->realize = ak_usb_realize;
    device_class_set_legacy_reset(dc, ak_usb_reset);
    device_class_set_props(dc, ak_usb_properties);
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
