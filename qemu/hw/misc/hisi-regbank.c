/*
 * HiSilicon generic RAM-backed register bank.
 *
 * Provides a simple read/write MMIO region that stores values in RAM.
 * Used for register blocks that firmware writes via read-modify-write
 * (pin mux, DDR PHY cal, PWM, etc.) where returning 0 on read would
 * break the RMW pattern.
 *
 * Optional auto-clear: if autoclear_offset is set, writes to that
 * offset have autoclear_mask bits cleared immediately (simulates
 * hardware "start" bits that self-clear when a command completes).
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"

#define TYPE_HISI_REGBANK "hisi-regbank"
OBJECT_DECLARE_SIMPLE_TYPE(HisiRegbankState, HISI_REGBANK)

struct HisiRegbankState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t *regs;
    uint32_t size;
    char *name;
    uint32_t autoclear_offset;  /* register offset with auto-clear bits */
    uint32_t autoclear_mask;    /* bits to clear on write (e.g. START bit) */
    bool sub_word;              /* accept 8/16-bit accesses too */
};

static uint64_t hisi_regbank_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiRegbankState *s = HISI_REGBANK(opaque);
    return s->regs[offset / 4];
}

static void hisi_regbank_write(void *opaque, hwaddr offset,
                                uint64_t val, unsigned size)
{
    HisiRegbankState *s = HISI_REGBANK(opaque);
    s->regs[offset / 4] = val;

    /* Auto-clear "start" bits so poll loops see command-done immediately */
    if (s->autoclear_mask && offset == s->autoclear_offset) {
        s->regs[offset / 4] &= ~s->autoclear_mask;
    }
}

static const MemoryRegionOps hisi_regbank_ops = {
    .read = hisi_regbank_read,
    .write = hisi_regbank_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * The same bank for blocks whose drivers use byte and halfword accesses
 * (Goke's HAL has readb/readw for every block): narrower accesses become
 * read-modify-writes of the 32-bit word.
 */
static const MemoryRegionOps hisi_regbank_subword_ops = {
    .read = hisi_regbank_read,
    .write = hisi_regbank_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void hisi_regbank_realize(DeviceState *dev, Error **errp)
{
    HisiRegbankState *s = HISI_REGBANK(dev);

    if (s->size == 0 || (s->size & 3)) {
        error_setg(errp, "hisi-regbank: size must be non-zero and 4-aligned");
        return;
    }

    s->regs = g_new0(uint32_t, s->size / 4);

    memory_region_init_io(&s->iomem, OBJECT(dev),
                          s->sub_word ? &hisi_regbank_subword_ops
                                      : &hisi_regbank_ops, s,
                          s->name ? s->name : TYPE_HISI_REGBANK, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static const Property hisi_regbank_properties[] = {
    DEFINE_PROP_UINT32("size", HisiRegbankState, size, 0),
    DEFINE_PROP_STRING("name", HisiRegbankState, name),
    DEFINE_PROP_UINT32("autoclear-offset", HisiRegbankState,
                       autoclear_offset, 0),
    DEFINE_PROP_UINT32("autoclear-mask", HisiRegbankState,
                       autoclear_mask, 0),
    DEFINE_PROP_BOOL("sub-word", HisiRegbankState, sub_word, false),
};

static void hisi_regbank_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, hisi_regbank_properties);
    dc->realize = hisi_regbank_realize;
}

static const TypeInfo hisi_regbank_info = {
    .name          = TYPE_HISI_REGBANK,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiRegbankState),
    .class_init    = hisi_regbank_class_init,
};

static void hisi_regbank_register_types(void)
{
    type_register_static(&hisi_regbank_info);
}

type_init(hisi_regbank_register_types)
