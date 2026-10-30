/*
 * Ingenic SADC: the T-series auxiliary ADC at 0x10070000.
 *
 * Eight 12-bit AUX channels (four on the 3.10 kernels' jz_adc_v13.c,
 * eight on 4.4's ingenic_adc_v13.c), one INTC line (18 on every part):
 *
 *   0x00 ENA     bit 15 power down (set out of reset); bit n starts a
 *                conversion of AUX n, cleared again when it is done
 *   0x04 CFG     storage
 *   0x08 CTRL    bit n masks AUX n's interrupt (byte access)
 *   0x0C STATUS  bit n: AUX n converted; write 1 to clear (byte access)
 *   0x10 + 2n    AUX n's result, 12 bits (16-bit or 32-bit reads: the
 *                driver reads odd channels as the top half of the word)
 *   0x20 CLKDIV  storage (div, us, ms dividers); 0x24 CMD storage
 *
 * The driver enables a channel, unmasks its interrupt and waits for it;
 * the conversion ends CONV_NS later.  The levels on the pins are the
 * "auxN" properties (raw codes, 0..4095; VREF 1.8 V full scale), which
 * a test can change at run time with qom-set.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_SADC "ingenic-sadc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicSadcState, INGENIC_SADC)

#define SADC_REGS       0x40
#define NCHAN           8

#define R_ENA           0x00
#define R_CTRL          0x08
#define R_STATUS        0x0C
#define R_DATA          0x10
#define R_CLKDIV        0x20

#define ENA_POWERDOWN   (1u << 15)
#define CONV_NS         (20 * SCALE_US)

struct IngenicSadcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[SADC_REGS / 4];
    uint32_t aux[NCHAN];
};

static uint32_t *sadc_reg(IngenicSadcState *s, hwaddr off)
{
    return &s->regs[off >> 2];
}

static void sadc_update(IngenicSadcState *s)
{
    qemu_set_irq(s->irq, (*sadc_reg(s, R_STATUS) &
                          ~*sadc_reg(s, R_CTRL) & 0xff) != 0);
}

static void sadc_start(IngenicSadcState *s)
{
    uint32_t ena = *sadc_reg(s, R_ENA);

    if (!(ena & ENA_POWERDOWN) && (ena & 0xff)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CONV_NS);
    }
}

static void sadc_convert(void *opaque)
{
    IngenicSadcState *s = opaque;
    uint32_t *ena = sadc_reg(s, R_ENA);
    int n;

    if (*ena & ENA_POWERDOWN) {
        return;
    }
    for (n = 0; n < NCHAN; n++) {
        if (*ena & (1u << n)) {
            uint32_t *d = sadc_reg(s, R_DATA + 2 * n);

            *d = deposit32(*d, (n & 1) * 16, 16, MIN(s->aux[n], 0xfff));
            *sadc_reg(s, R_STATUS) |= 1u << n;
        }
    }
    *ena &= ~0xffu;
    sadc_update(s);
}

static uint64_t ingenic_sadc_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicSadcState *s = opaque;

    if (off + size > SADC_REGS) {
        return 0;
    }
    return extract64(*sadc_reg(s, off), (off & 3) * 8, size * 8);
}

static void ingenic_sadc_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    IngenicSadcState *s = opaque;
    hwaddr word = off & ~3;
    int shift = (off & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t *reg;

    if (off + size > SADC_REGS) {
        return;
    }
    reg = sadc_reg(s, word);
    val = (val << shift) & mask;
    switch (word) {
    case R_STATUS:
        *reg &= ~val;
        break;
    case R_DATA ... R_DATA + 0xf:
        return;                         /* the results are read only */
    default:
        *reg = (*reg & ~mask) | val;
        break;
    }
    if (word == R_ENA) {
        sadc_start(s);
    }
    sadc_update(s);
}

static const MemoryRegionOps ingenic_sadc_ops = {
    .read = ingenic_sadc_read,
    .write = ingenic_sadc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .impl.unaligned = false,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void ingenic_sadc_reset(DeviceState *dev)
{
    IngenicSadcState *s = INGENIC_SADC(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    *sadc_reg(s, R_ENA) = ENA_POWERDOWN;
    *sadc_reg(s, R_CTRL) = 0xff;
    sadc_update(s);
}

static void ingenic_sadc_init(Object *obj)
{
    IngenicSadcState *s = INGENIC_SADC(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &ingenic_sadc_ops, s,
                          TYPE_INGENIC_SADC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sadc_convert, s);

    /* mid-scale on every pin, a little apart so that channels can be told */
    for (n = 0; n < NCHAN; n++) {
        g_autofree char *name = g_strdup_printf("aux%d", n);

        s->aux[n] = 0x800 + 0x10 * n;
        object_property_add_uint32_ptr(obj, name, &s->aux[n],
                                       OBJ_PROP_FLAG_READWRITE);
    }
}

static const VMStateDescription vmstate_ingenic_sadc = {
    .name = TYPE_INGENIC_SADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicSadcState, SADC_REGS / 4),
        VMSTATE_UINT32_ARRAY(aux, IngenicSadcState, NCHAN),
        VMSTATE_TIMER_PTR(timer, IngenicSadcState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_sadc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_sadc_reset);
    dc->vmsd = &vmstate_ingenic_sadc;
}

static const TypeInfo ingenic_sadc_info = {
    .name = TYPE_INGENIC_SADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicSadcState),
    .instance_init = ingenic_sadc_init,
    .class_init = ingenic_sadc_class_init,
};

static void ingenic_sadc_register_types(void)
{
    type_register_static(&ingenic_sadc_info);
}

type_init(ingenic_sadc_register_types)
