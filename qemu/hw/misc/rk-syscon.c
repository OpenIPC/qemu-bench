/*
 * Rockchip register file: CRU, GRF, PMU, IOC and friends, as storage.
 *
 * Most Rockchip control registers take "hiword mask" writes: bits 31:16
 * of the written value select which of bits 15:0 change, and the upper
 * half reads back as zero.  hiword=off stores whole words instead.
 *
 * lock-regs lists offsets whose bit 10 always reads set: the CON1 of
 * every PLL (Rockchip's pll_rk3328 type, LOCK), which boot code and the
 * clock drivers poll after changing a rate.  pll-mode-reg, when set, is
 * the MODE register the PLLs are switched to "normal" through; it is
 * plain storage like the rest.
 *
 * mirror lists (dst, src, shift) triples: a read of dst returns, ORed
 * over its triples, the low half of the register at src shifted left by
 * shift -- for a handshake that completes at once: the PMU's power
 * status following its power requests, its idle ack / idle status
 * following its idle requests.
 *
 * plain-regs lists offsets that take whole-word writes inside a hiword
 * file: the CRU's fractional dividers (COMPOSITE_FRACMUX in the clock
 * drivers), numerator << 16 | denominator.  As hiword writes they would
 * keep a denominator of 0, and U-Boot's clock driver divides by it.
 *
 * const lists offset / value pairs that read back fixed whatever is
 * written: status registers whose "done" bits the drivers poll and then
 * clear by writing them (OTP's INT_STATUS).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_RK_SYSCON "rk-syscon"
OBJECT_DECLARE_SIMPLE_TYPE(RkSysconState, RK_SYSCON)

#define RK_PLL_LOCK (1u << 10)
#define RK_MAX_INIT 32

struct RkSysconState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t size;
    bool hiword;
    uint32_t *regs;

    uint32_t num_lock;
    uint32_t *lock_regs;
    /* reset values: pairs of offset, value */
    uint32_t num_init;
    uint32_t *init;
    uint32_t num_const;
    uint32_t *konst;
    uint32_t num_plain;
    uint32_t *plain;
    uint32_t num_mirror;
    uint32_t *mirror;
};

static bool rk_syscon_is_lock(RkSysconState *s, hwaddr off)
{
    uint32_t i;

    for (i = 0; i < s->num_lock; i++) {
        if (s->lock_regs[i] == off) {
            return true;
        }
    }
    return false;
}

static uint64_t rk_syscon_read(void *opaque, hwaddr off, unsigned size)
{
    RkSysconState *s = opaque;
    uint32_t v = s->regs[off / 4];
    uint32_t i;

    for (i = 0; i + 1 < s->num_const; i += 2) {
        if (s->konst[i] == (off & ~3)) {
            return s->konst[i + 1];
        }
    }

    for (i = 0; i + 2 < s->num_mirror; i += 3) {
        if (s->mirror[i] == (off & ~3)) {
            uint32_t m = 0, j;

            for (j = 0; j + 2 < s->num_mirror; j += 3) {
                if (s->mirror[j] == (off & ~3) &&
                    s->mirror[j + 1] < s->size) {
                    m |= (s->regs[s->mirror[j + 1] / 4] & 0xffff)
                         << s->mirror[j + 2];
                }
            }
            return m;
        }
    }

    if (rk_syscon_is_lock(s, off & ~3)) {
        v |= RK_PLL_LOCK;
    }
    return v;
}

static void rk_syscon_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    RkSysconState *s = opaque;
    uint32_t *r = &s->regs[off / 4];

    uint32_t i;

    for (i = 0; i < s->num_plain; i++) {
        if (s->plain[i] == (off & ~3)) {
            *r = val;
            return;
        }
    }
    if (s->hiword) {
        uint32_t mask = val >> 16;

        *r = (*r & ~mask) | (val & mask);
    } else {
        *r = val;
    }
}

static const MemoryRegionOps rk_syscon_ops = {
    .read = rk_syscon_read,
    .write = rk_syscon_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_syscon_reset(DeviceState *dev)
{
    RkSysconState *s = RK_SYSCON(dev);
    uint32_t i;

    memset(s->regs, 0, s->size);
    for (i = 0; i + 1 < s->num_init; i += 2) {
        if (s->init[i] < s->size) {
            s->regs[s->init[i] / 4] = s->init[i + 1];
        }
    }
}

static void rk_syscon_realize(DeviceState *dev, Error **errp)
{
    RkSysconState *s = RK_SYSCON(dev);

    s->size = ROUND_UP(s->size, 4);
    s->regs = g_malloc0(s->size);
    memory_region_init_io(&s->iomem, OBJECT(s), &rk_syscon_ops, s,
                          TYPE_RK_SYSCON, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static const Property rk_syscon_props[] = {
    DEFINE_PROP_UINT32("size", RkSysconState, size, 0x10000),
    DEFINE_PROP_BOOL("hiword", RkSysconState, hiword, true),
    DEFINE_PROP_ARRAY("lock-regs", RkSysconState, num_lock, lock_regs,
                      qdev_prop_uint32, uint32_t),
    DEFINE_PROP_ARRAY("init", RkSysconState, num_init, init,
                      qdev_prop_uint32, uint32_t),
    DEFINE_PROP_ARRAY("const", RkSysconState, num_const, konst,
                      qdev_prop_uint32, uint32_t),
    DEFINE_PROP_ARRAY("plain-regs", RkSysconState, num_plain, plain,
                      qdev_prop_uint32, uint32_t),
    DEFINE_PROP_ARRAY("mirror", RkSysconState, num_mirror, mirror,
                      qdev_prop_uint32, uint32_t),
};

static void rk_syscon_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rk_syscon_realize;
    device_class_set_legacy_reset(dc, rk_syscon_reset);
    device_class_set_props(dc, rk_syscon_props);
    dc->desc = "Rockchip CRU / GRF register file";
}

static const TypeInfo rk_syscon_info = {
    .name = TYPE_RK_SYSCON,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkSysconState),
    .class_init = rk_syscon_class_init,
};

static void rk_syscon_register_types(void)
{
    type_register_static(&rk_syscon_info);
}

type_init(rk_syscon_register_types)
