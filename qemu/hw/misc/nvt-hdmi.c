/*
 * Novatek NA51103 HDMI 2.0 transmitter (0x2_F0880000, SPI 52): the status
 * its drivers wait on, with no link behind it.
 *
 * The HDAL's kdrv_videoout/hdmi/nvt_hdmi20.ko (not stripped) and the
 * vendor U-Boot's bootlogo code drive it.  What they poll:
 *
 *   0x00   bit 12  video clock stable: hdmitx_check_video_stable() spins
 *                  on it 0x1800000 times ("HDMI Clk No Stable!Timeout!")
 *          bit 13  hotplug (hdmitx_check_hotplug())
 *   0x20/0x24      interrupt status (write 1 to clear) / enable; bit 2 a
 *                  hotplug change
 *   0x88   bit 10  the mute state as applied, polled after a write to
 *                  0x1B0 (bit 4 mute) by hdmitx_set_config() ("HDMI MUTE
 *                  STS!Timeout!"): it reads back as the inverse of bit 4
 *   0x200 + 4n     the PHY's registers (hdmitx_read/write_phy): storage
 *
 * The clock reads stable and no cable is in (hotplug 0), so the drivers
 * skip EDID over DDC and the GUI stays on its default output; what the
 * LCD300 scans out is shown regardless.  Everything else is storage.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_HDMI "nvt-hdmi"
OBJECT_DECLARE_SIMPLE_TYPE(NvtHdmiState, NVT_HDMI)

#define HDMI_SIZE           0x1000
#define HDMI_STS            0x00
#define HDMI_INT_STS        0x20
#define HDMI_INT_EN         0x24
#define HDMI_MUTE_STS       0x88
#define HDMI_MUTE           0x1B0

#define STS_CLK_STABLE      (1u << 12)
#define STS_HOTPLUG         (1u << 13)
#define MUTE_STS_BIT        (1u << 10)
#define MUTE_BIT            (1u << 4)

struct NvtHdmiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[HDMI_SIZE / 4];
};

static void nvt_hdmi_update_irq(NvtHdmiState *s)
{
    qemu_set_irq(s->irq, (s->regs[HDMI_INT_STS / 4] &
                          s->regs[HDMI_INT_EN / 4]) != 0);
}

static uint64_t nvt_hdmi_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtHdmiState *s = opaque;
    uint32_t val = s->regs[offset / 4];

    switch (offset) {
    case HDMI_STS:
        return (val & ~STS_HOTPLUG) | STS_CLK_STABLE;
    case HDMI_MUTE_STS:
        return (val & ~MUTE_STS_BIT) |
               (s->regs[HDMI_MUTE / 4] & MUTE_BIT ? 0 : MUTE_STS_BIT);
    }
    return val;
}

static void nvt_hdmi_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    NvtHdmiState *s = opaque;

    if (offset == HDMI_INT_STS) {
        s->regs[offset / 4] &= ~value;
    } else {
        s->regs[offset / 4] = value;
    }
    nvt_hdmi_update_irq(s);
}

static const MemoryRegionOps nvt_hdmi_ops = {
    .read = nvt_hdmi_read,
    .write = nvt_hdmi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void nvt_hdmi_reset(DeviceState *dev)
{
    NvtHdmiState *s = NVT_HDMI(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void nvt_hdmi_init(Object *obj)
{
    NvtHdmiState *s = NVT_HDMI(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_hdmi_ops, s, TYPE_NVT_HDMI,
                          HDMI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_nvt_hdmi = {
    .name = TYPE_NVT_HDMI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtHdmiState, HDMI_SIZE / 4),
        VMSTATE_END_OF_LIST()
    },
};

static void nvt_hdmi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_hdmi_reset);
    dc->vmsd = &vmstate_nvt_hdmi;
}

static const TypeInfo nvt_hdmi_info = {
    .name           = TYPE_NVT_HDMI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(NvtHdmiState),
    .instance_init  = nvt_hdmi_init,
    .class_init     = nvt_hdmi_class_init,
};

static void nvt_hdmi_register_types(void)
{
    type_register_static(&nvt_hdmi_info);
}

type_init(nvt_hdmi_register_types)
