/*
 * Allwinner "ncat" clock control unit (H6 layout; V831/V833 here).
 *
 * Storage for every register, plus the two kinds of status bit the boot
 * chain waits on:
 *   - PLL control registers (0x000..0x0FC): LOCK (bit 28) reads set while
 *     the PLL is enabled (bit 31), as soon as it is.
 *   - the MBUS (0x540) and DRAM (0x800) clock registers: the update bit
 *     (bit 27) clears itself once the new divider is taken.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_AW_CCU "aw-ccu"
OBJECT_DECLARE_SIMPLE_TYPE(AwCcuState, AW_CCU)

#define AW_CCU_SIZE         0x1000
#define AW_CCU_PLL_END      0x100
#define AW_CCU_PLL_EN       (1u << 31)
#define AW_CCU_PLL_LOCK     (1u << 28)
#define AW_CCU_UPD          (1u << 27)

struct AwCcuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[AW_CCU_SIZE / 4];
};

/*
 * Values the drivers read before writing.  The mask ROM leaves the
 * peripheral PLL running (N = 100, 1200 MHz VCO), which boot0 and
 * U-Boot take as the SPI clock source without programming it.
 */
static const struct {
    hwaddr off;
    uint32_t val;
} aw_ccu_reset[] = {
    { 0x020, 0xA0006300 },      /* PLL_PERI0 */
};

static uint64_t aw_ccu_read(void *opaque, hwaddr addr, unsigned size)
{
    AwCcuState *s = opaque;
    uint32_t val = s->regs[addr >> 2];

    if (addr < AW_CCU_PLL_END && (val & AW_CCU_PLL_EN)) {
        val |= AW_CCU_PLL_LOCK;
    }
    return val;
}

static void aw_ccu_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    AwCcuState *s = opaque;

    if (addr < AW_CCU_PLL_END) {
        val &= ~AW_CCU_PLL_LOCK;
    }
    if (addr == 0x540 || addr == 0x800) {
        val &= ~AW_CCU_UPD;
    }
    s->regs[addr >> 2] = val;
}

static const MemoryRegionOps aw_ccu_ops = {
    .read = aw_ccu_read,
    .write = aw_ccu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aw_ccu_reset_hold(Object *obj, ResetType type)
{
    AwCcuState *s = AW_CCU(obj);
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    for (i = 0; i < ARRAY_SIZE(aw_ccu_reset); i++) {
        s->regs[aw_ccu_reset[i].off >> 2] = aw_ccu_reset[i].val;
    }
}

static void aw_ccu_init(Object *obj)
{
    AwCcuState *s = AW_CCU(obj);

    memory_region_init_io(&s->iomem, obj, &aw_ccu_ops, s, TYPE_AW_CCU,
                          AW_CCU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void aw_ccu_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_ccu_reset_hold;
}

static const TypeInfo aw_ccu_info = {
    .name           = TYPE_AW_CCU,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(AwCcuState),
    .instance_init  = aw_ccu_init,
    .class_init     = aw_ccu_class_init,
};

static void aw_ccu_register_types(void)
{
    type_register_static(&aw_ccu_info);
}

type_init(aw_ccu_register_types)
