/*
 * Grain Media SAR ADC (the "ADC wrapper" at 0x90A00000).
 *
 * Three 8-bit channels, XGAIN0-2, under the vendor sar_adc_drv.ko
 * (module/SAR_ADC: sar_adc_drv.h, gm8136.c):
 *
 *   0x00 CR0   [2:0] channel enables, bit 8 power, bit 19 software mode,
 *              [21:20] software channel, the high byte the sample clock
 *   0x04 INT   high / low threshold flags, two per channel; write 1 clears
 *   0x08 / 0x0C / 0x10  DAT0-2, the channels' last conversions
 *   0x14 / 0x18 / 0x1C  THD0-2 (max << 16 | min), 0x20-0x28 auto requests
 *
 * The driver polls every 200 ms, waiting a millisecond after selecting a
 * channel instead of checking for a conversion to finish, so the data
 * registers simply hold each input's level: the "ch0".."ch2" properties,
 * changeable at run time (qom-set /machine/saradc ch1 0x40).  XGAIN2 is
 * the CVBS cable detector -- above 0x78 the driver powers the video DAC
 * and TV encoder down, below 0x2F up -- and defaults to "no cable".
 * Thresholds are not evaluated: interrupt 35 stays low.
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
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_SARADC "gm-saradc"
OBJECT_DECLARE_SIMPLE_TYPE(GmSaradcState, GM_SARADC)

#define GM_SARADC_SIZE      0x1000
#define GM_SARADC_NREGS     (0x40 / 4)
#define R_INT               0x04
#define R_DAT0              0x08

struct GmSaradcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[GM_SARADC_NREGS];
    uint32_t ch[3];
};

static uint64_t gm_saradc_read(void *opaque, hwaddr off, unsigned size)
{
    GmSaradcState *s = opaque;

    if (off >= R_DAT0 && off < R_DAT0 + 12) {
        return s->ch[(off - R_DAT0) / 4] & 0xFF;
    }
    return off < GM_SARADC_NREGS * 4 ? s->regs[off / 4] : 0;
}

static void gm_saradc_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    GmSaradcState *s = opaque;

    if (off >= GM_SARADC_NREGS * 4 || (off >= R_DAT0 && off < R_DAT0 + 12)) {
        return;
    }
    if (off == R_INT) {
        s->regs[off / 4] &= ~val;
        return;
    }
    s->regs[off / 4] = val;
}

static const MemoryRegionOps gm_saradc_ops = {
    .read = gm_saradc_read,
    .write = gm_saradc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_saradc_reset(DeviceState *dev)
{
    GmSaradcState *s = GM_SARADC(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void gm_saradc_realize(DeviceState *dev, Error **errp)
{
    GmSaradcState *s = GM_SARADC(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_saradc_ops, s,
                          TYPE_GM_SARADC, GM_SARADC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static const Property gm_saradc_props[] = {
    DEFINE_PROP_UINT32("ch0", GmSaradcState, ch[0], 0x80),
    DEFINE_PROP_UINT32("ch1", GmSaradcState, ch[1], 0x80),
    DEFINE_PROP_UINT32("ch2", GmSaradcState, ch[2], 0x80),
};

static const VMStateDescription vmstate_gm_saradc = {
    .name = TYPE_GM_SARADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmSaradcState, GM_SARADC_NREGS),
        VMSTATE_UINT32_ARRAY(ch, GmSaradcState, 3),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_saradc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_saradc_realize;
    device_class_set_legacy_reset(dc, gm_saradc_reset);
    device_class_set_props(dc, gm_saradc_props);
    dc->vmsd = &vmstate_gm_saradc;
}

static const TypeInfo gm_saradc_info = {
    .name           = TYPE_GM_SARADC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmSaradcState),
    .class_init     = gm_saradc_class_init,
};

static void gm_saradc_register_types(void)
{
    type_register_static(&gm_saradc_info);
}

type_init(gm_saradc_register_types)
