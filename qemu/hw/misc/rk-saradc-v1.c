/*
 * Rockchip SARADC v1 ("rockchip,rk3399-saradc": RV1126 / RV1109
 * 0xFF5E0000), as U-Boot's and Linux's rockchip_saradc drivers use it:
 *
 *   0x00 DATA        the last conversion (10 bits)
 *   0x04 STAS        bit 0 busy
 *   0x08 CTRL        bits 2:0 channel, 3 power up (starts a conversion),
 *                    5 interrupt enable, 6 conversion done (the drivers
 *                    poll it, or take the interrupt; writing CTRL with
 *                    it clear clears it)
 *   0x0c DLY_PU_SOC  storage
 *
 * A conversion is done at once.  Every channel reads "channel-level"
 * (default full scale, 0x3ff): the recovery / volume keys on the ADC
 * ladder read released.  U-Boot's "Key Error: ... failed to read saradc"
 * on the RV1109 dump came from the polled bit that never rose.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_RK_SARADC_V1 "rk-saradc-v1"
OBJECT_DECLARE_SIMPLE_TYPE(RkSaradcV1State, RK_SARADC_V1)

#define R_DATA          0x00
#define R_STAS          0x04
#define R_CTRL          0x08
#define R_DLY_PU_SOC    0x0c
#define CTRL_POWER_UP   (1u << 3)
#define CTRL_IRQ_EN     (1u << 5)
#define CTRL_IRQ_STATUS (1u << 6)

struct RkSaradcV1State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t data, ctrl, dly;
    uint32_t level;             /* property: every channel's reading */
};

static void rk_saradc_update(RkSaradcV1State *s)
{
    qemu_set_irq(s->irq, (s->ctrl & CTRL_IRQ_STATUS) &&
                         (s->ctrl & CTRL_IRQ_EN));
}

static uint64_t rk_saradc_read(void *opaque, hwaddr off, unsigned size)
{
    RkSaradcV1State *s = opaque;

    switch (off) {
    case R_DATA:
        return s->data;
    case R_STAS:
        return 0;                   /* never busy */
    case R_CTRL:
        return s->ctrl;
    case R_DLY_PU_SOC:
        return s->dly;
    }
    return 0;
}

static void rk_saradc_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    RkSaradcV1State *s = opaque;

    switch (off) {
    case R_CTRL:
        s->ctrl = val & ~CTRL_IRQ_STATUS;
        if (val & CTRL_POWER_UP) {
            /* the conversion of channel CTRL[2:0], done at once */
            s->data = s->level & 0x3ff;
            s->ctrl |= CTRL_IRQ_STATUS;
        }
        rk_saradc_update(s);
        break;
    case R_DLY_PU_SOC:
        s->dly = val;
        break;
    }
}

static const MemoryRegionOps rk_saradc_ops = {
    .read = rk_saradc_read,
    .write = rk_saradc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_saradc_reset(DeviceState *dev)
{
    RkSaradcV1State *s = RK_SARADC_V1(dev);

    s->data = s->ctrl = s->dly = 0;
    rk_saradc_update(s);
}

static void rk_saradc_init(Object *obj)
{
    RkSaradcV1State *s = RK_SARADC_V1(obj);

    memory_region_init_io(&s->iomem, obj, &rk_saradc_ops, s,
                          TYPE_RK_SARADC_V1, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property rk_saradc_properties[] = {
    DEFINE_PROP_UINT32("channel-level", RkSaradcV1State, level, 0x3ff),
};

static const VMStateDescription rk_saradc_vmstate = {
    .name = TYPE_RK_SARADC_V1,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(data, RkSaradcV1State),
        VMSTATE_UINT32(ctrl, RkSaradcV1State),
        VMSTATE_UINT32(dly, RkSaradcV1State),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_saradc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_saradc_reset);
    device_class_set_props(dc, rk_saradc_properties);
    dc->vmsd = &rk_saradc_vmstate;
}

static const TypeInfo rk_saradc_info = {
    .name = TYPE_RK_SARADC_V1,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkSaradcV1State),
    .instance_init = rk_saradc_init,
    .class_init = rk_saradc_class_init,
};

static void rk_saradc_register_types(void)
{
    type_register_static(&rk_saradc_info);
}

type_init(rk_saradc_register_types)
