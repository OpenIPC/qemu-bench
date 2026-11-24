/*
 * Faraday FTINTC010 interrupt controller, the 64-input one of the Grain
 * Media GM8126 (0x99500000).
 *
 * Inputs 0-31 have their registers at 0x00, 32-63 (the "EX" bank) at 0x60
 * (arch/arm/mach-GM/ftintc010.c):
 *
 *   +0x00 source   +0x04 IRQ mask (1 = enabled)   +0x08 clear an edge latch
 *   +0x0C mode (1 = edge)   +0x10 level (1 = active low / falling)
 *   +0x14 status; the same for FIQ from +0x20
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_INTC010 "gm-intc010"
OBJECT_DECLARE_SIMPLE_TYPE(GmIntc010State, GM_INTC010)

#define GM_INTC010_SIZE 0x1000
#define BANKS           2

#define R_SOURCE        0x00
#define R_MASK          0x04
#define R_CLEAR         0x08
#define R_MODE          0x0C
#define R_LEVEL         0x10
#define R_STATUS        0x14

/* the registers of one kind of interrupt: IRQ 0, FIQ 1 */
typedef struct GmIntc010Bank {
    uint32_t mask, mode, level, latch;
} GmIntc010Bank;

struct GmIntc010State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq out[2];
    uint32_t input[BANKS];
    GmIntc010Bank b[2][BANKS];
};

static uint32_t intc010_status(GmIntc010State *s, int kind, int n)
{
    const GmIntc010Bank *b = &s->b[kind][n];
    const uint32_t lvl = (s->input[n] ^ b->level) & ~b->mode;

    return (lvl | (b->latch & b->mode)) & b->mask;
}

static void intc010_update(GmIntc010State *s)
{
    int kind;

    for (kind = 0; kind < 2; kind++) {
        qemu_set_irq(s->out[kind], (intc010_status(s, kind, 0) |
                                    intc010_status(s, kind, 1)) != 0);
    }
}

static void intc010_set(void *opaque, int n, int level)
{
    GmIntc010State *s = opaque;
    const uint32_t bit = 1u << (n % 32);
    const uint32_t old = s->input[n / 32];
    int kind;

    s->input[n / 32] = level ? old | bit : old & ~bit;
    for (kind = 0; kind < 2; kind++) {
        GmIntc010Bank *b = &s->b[kind][n / 32];

        if (((old ^ s->input[n / 32]) & bit) &&
            !!level != !!(b->level & bit)) {
            b->latch |= bit;
        }
    }
    intc010_update(s);
}

static GmIntc010Bank *intc010_bank(GmIntc010State *s, hwaddr *off, int *kind,
                                   int *n)
{
    *n = *off >= 0x60;
    if (*n) {
        *off -= 0x60;
    }
    *kind = *off >= 0x20;
    if (*kind) {
        *off -= 0x20;
    }
    return *off < 0x20 ? &s->b[*kind][*n] : NULL;
}

static uint64_t intc010_read(void *opaque, hwaddr off, unsigned size)
{
    GmIntc010State *s = opaque;
    int kind, n;
    GmIntc010Bank *b = off < 0xA0 ? intc010_bank(s, &off, &kind, &n) : NULL;

    if (!b) {
        return 0;
    }
    switch (off) {
    case R_SOURCE:
        return s->input[n];
    case R_MASK:
        return b->mask;
    case R_MODE:
        return b->mode;
    case R_LEVEL:
        return b->level;
    case R_STATUS:
        return intc010_status(s, kind, n);
    }
    return 0;
}

static void intc010_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    GmIntc010State *s = opaque;
    int kind, n;
    GmIntc010Bank *b = off < 0xA0 ? intc010_bank(s, &off, &kind, &n) : NULL;

    if (!b) {
        qemu_log_mask(LOG_UNIMP, "gm-intc010: write 0x%" HWADDR_PRIx "\n",
                      off);
        return;
    }
    switch (off) {
    case R_MASK:
        b->mask = val;
        break;
    case R_CLEAR:
        b->latch &= ~val;
        break;
    case R_MODE:
        b->mode = val;
        break;
    case R_LEVEL:
        b->level = val;
        break;
    }
    intc010_update(s);
}

static const MemoryRegionOps intc010_ops = {
    .read = intc010_read,
    .write = intc010_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void intc010_reset(DeviceState *dev)
{
    GmIntc010State *s = GM_INTC010(dev);

    memset(s->b, 0, sizeof(s->b));
    intc010_update(s);
}

static void intc010_realize(DeviceState *dev, Error **errp)
{
    GmIntc010State *s = GM_INTC010(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &intc010_ops, s,
                          TYPE_GM_INTC010, GM_INTC010_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->out[0]);
    sysbus_init_irq(sbd, &s->out[1]);
    qdev_init_gpio_in(dev, intc010_set, 32 * BANKS);
}

static const VMStateDescription vmstate_intc010_bank = {
    .name = TYPE_GM_INTC010 "/bank",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mask, GmIntc010Bank),
        VMSTATE_UINT32(mode, GmIntc010Bank),
        VMSTATE_UINT32(level, GmIntc010Bank),
        VMSTATE_UINT32(latch, GmIntc010Bank),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_intc010 = {
    .name = TYPE_GM_INTC010,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(input, GmIntc010State, BANKS),
        VMSTATE_STRUCT_2DARRAY(b, GmIntc010State, 2, BANKS, 1,
                               vmstate_intc010_bank, GmIntc010Bank),
        VMSTATE_END_OF_LIST()
    }
};

static void intc010_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = intc010_realize;
    device_class_set_legacy_reset(dc, intc010_reset);
    dc->vmsd = &vmstate_intc010;
}

static const TypeInfo intc010_info = {
    .name = TYPE_GM_INTC010,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GmIntc010State),
    .class_init = intc010_class_init,
};

static void intc010_register_types(void)
{
    type_register_static(&intc010_info);
}

type_init(intc010_register_types)
