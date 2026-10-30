/*
 * Ingenic DTRNG: the true random number generator at 0x10072000 (T21,
 * T23, T31, T40, T41; INTC source 34).
 *
 * The SDKs' sample_dtrng (jz-dtrng.c) and the 4.4 kernel's hw_random
 * driver (drivers/char/hw_random/ingenic-rng.c) drive it the same way:
 *
 *   0x00 CFG        bit 0 enable, bits 10:1 clock divider, bit 11 masks
 *                   the interrupt, bit 12 clears "ready" while set,
 *                   bits 31:16 a seed (storage here)
 *   0x04 RANDOMNUM  the number
 *   0x08 STAT       bit 0: a number is ready
 *
 * Enabled, a number is ready READY_NS later, and stays ready (each read
 * of RANDOMNUM gives a fresh one, from the host's entropy) until CFG
 * bit 12 is set; when it is released, the next one is ready READY_NS on.
 * The interrupt is ready & enabled & not masked: the sample's interrupt
 * mode reads the number, then clears ready and disables the block.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/guest-random.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_DTRNG "ingenic-dtrng"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDtrngState, INGENIC_DTRNG)

#define R_CFG           0x00
#define R_RANDOMNUM     0x04
#define R_STAT          0x08

#define CFG_EN          (1u << 0)
#define CFG_INT_MASK    (1u << 11)
#define CFG_RDY_CLR     (1u << 12)

#define READY_NS        (10 * SCALE_US)

struct IngenicDtrngState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t cfg;
    uint32_t ready;
};

static void dtrng_update(IngenicDtrngState *s)
{
    qemu_set_irq(s->irq, s->ready && (s->cfg & CFG_EN) &&
                         !(s->cfg & CFG_INT_MASK));
}

static void dtrng_ready(void *opaque)
{
    IngenicDtrngState *s = opaque;

    if ((s->cfg & CFG_EN) && !(s->cfg & CFG_RDY_CLR)) {
        s->ready = 1;
        dtrng_update(s);
    }
}

static uint64_t ingenic_dtrng_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicDtrngState *s = opaque;
    uint32_t v = 0;

    switch (off) {
    case R_CFG:
        return s->cfg;
    case R_RANDOMNUM:
        if (s->ready) {
            qemu_guest_getrandom_nofail(&v, sizeof(v));
        }
        return v;
    case R_STAT:
        return s->ready;
    }
    return 0;
}

static void ingenic_dtrng_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    IngenicDtrngState *s = opaque;

    if (off != R_CFG) {
        return;
    }
    s->cfg = val;
    if (!(s->cfg & CFG_EN) || (s->cfg & CFG_RDY_CLR)) {
        s->ready = 0;
        timer_del(s->timer);
    } else if (!s->ready && !timer_pending(s->timer)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + READY_NS);
    }
    dtrng_update(s);
}

static const MemoryRegionOps ingenic_dtrng_ops = {
    .read = ingenic_dtrng_read,
    .write = ingenic_dtrng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_dtrng_reset(DeviceState *dev)
{
    IngenicDtrngState *s = INGENIC_DTRNG(dev);

    timer_del(s->timer);
    s->cfg = 0;
    s->ready = 0;
    dtrng_update(s);
}

static void ingenic_dtrng_init(Object *obj)
{
    IngenicDtrngState *s = INGENIC_DTRNG(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_dtrng_ops, s,
                          TYPE_INGENIC_DTRNG, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dtrng_ready, s);
}

static const VMStateDescription vmstate_ingenic_dtrng = {
    .name = TYPE_INGENIC_DTRNG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg, IngenicDtrngState),
        VMSTATE_UINT32(ready, IngenicDtrngState),
        VMSTATE_TIMER_PTR(timer, IngenicDtrngState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_dtrng_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_dtrng_reset);
    dc->vmsd = &vmstate_ingenic_dtrng;
}

static const TypeInfo ingenic_dtrng_info = {
    .name = TYPE_INGENIC_DTRNG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDtrngState),
    .instance_init = ingenic_dtrng_init,
    .class_init = ingenic_dtrng_class_init,
};

static void ingenic_dtrng_register_types(void)
{
    type_register_static(&ingenic_dtrng_info);
}

type_init(ingenic_dtrng_register_types)
