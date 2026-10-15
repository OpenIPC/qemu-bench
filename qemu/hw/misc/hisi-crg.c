/*
 * HiSilicon Clock/Reset Generator (CRG).
 *
 * Provides a simple read/write register bank so the kernel's clock
 * driver can probe without hanging. PLL status registers always
 * report "locked" (bit 28 set).
 *
 * When cpu_srst_offset is set (e.g. 0x78 for CV500/DV300), the CPU1
 * soft-reset request bit of that register drives the "cpu1-reset" GPIO
 * output (1 = held in reset).  The machine wires it to CPU1's power
 * state, so the kernel releasing the bit starts CPU1 like silicon does.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"

#define TYPE_HISI_CRG "hisi-crg"
OBJECT_DECLARE_SIMPLE_TYPE(HisiCrgState, HISI_CRG)

#define HISI_CRG_REG_SIZE  0x10000
#define HISI_CRG_REG_COUNT (HISI_CRG_REG_SIZE / 4)

/*
 * CPU soft-reset register bits (from SDK: arch/arm/mach-hibvt/).
 * Bit 2 = CPU1 reset request, bit 4 = CPU1 debug reset request.
 * Clearing CPU1_SRST_REQ releases CPU1 from reset.
 */
#define CPU1_SRST_REQ  (1 << 2)
#define DBG1_SRST_REQ  (1 << 4)

/*
 * PLL lock bit — HiSilicon CRG drivers check bit 28 of PLL status
 * registers to confirm PLL has locked before using the clock.
 */
#define PLL_LOCK_BIT  (1 << 28)

/* Known PLL status register offsets (from SDK clock drivers) */
#define HISI_PLL_APLL_STATUS  0x0004  /* APLL */
#define HISI_PLL_DPLL_STATUS  0x0014  /* DPLL */
#define HISI_PLL_VPLL_STATUS  0x0024  /* VPLL */
#define HISI_PLL_EPLL_STATUS  0x0034  /* EPLL */

struct HisiCrgState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[HISI_CRG_REG_COUNT];
    uint32_t cpu_srst_offset;   /* 0 = disabled, e.g. 0x78 for CV500 */
    uint32_t pll_lock_offset;   /* per-PLL lock status, 0 = none */
    uint32_t pll_lock_bits;     /* bits that read as locked there */
    qemu_irq cpu1_reset;        /* CPU1_SRST_REQ level, 1 = in reset */
};

static bool is_pll_status_reg(hwaddr offset)
{
    switch (offset) {
    case HISI_PLL_APLL_STATUS:
    case HISI_PLL_DPLL_STATUS:
    case HISI_PLL_VPLL_STATUS:
    case HISI_PLL_EPLL_STATUS:
        return true;
    default:
        return false;
    }
}

static uint64_t hisi_crg_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiCrgState *s = HISI_CRG(opaque);

    if (offset >= HISI_CRG_REG_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi_crg_read: offset 0x%x out of range\n",
                      (int)offset);
        return 0;
    }

    uint32_t val = s->regs[offset / 4];

    /* PLL status registers always report locked */
    if (is_pll_status_reg(offset)) {
        val |= PLL_LOCK_BIT;
    }

    /*
     * V1 keeps one lock bit per PLL in a single status register
     * (CRG+0xe8), and its U-Boot reg_info table waits there for all four
     * after reprogramming them.  There is no PLL to lose lock.
     */
    if (s->pll_lock_offset && offset == s->pll_lock_offset) {
        val |= s->pll_lock_bits;
    }

    return val;
}

static void hisi_crg_write(void *opaque, hwaddr offset,
                            uint64_t val, unsigned size)
{
    HisiCrgState *s = HISI_CRG(opaque);

    if (offset >= HISI_CRG_REG_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi_crg_write: offset 0x%x out of range\n",
                      (int)offset);
        return;
    }

    uint32_t old = s->regs[offset / 4];
    s->regs[offset / 4] = (uint32_t)val;

    /*
     * CPU soft-reset register: the vendor kernel's SMP bringup
     * (mach-hibvt/platsmp.c hi35xx_boot_secondary) writes an
     * "ldr pc, [pc, #-4]; .word secondary_startup" trampoline at
     * physical 0x0, then clears CPU1_SRST_REQ (bit 2) to release CPU1,
     * which leaves reset at PC 0.  cpu_die sets the bit again.  Only
     * edges are forwarded; the machine owns what "reset" means for the
     * CPU.
     */
    if (s->cpu_srst_offset && offset == s->cpu_srst_offset &&
        ((old ^ (uint32_t)val) & CPU1_SRST_REQ)) {
        qemu_set_irq(s->cpu1_reset, !!(val & CPU1_SRST_REQ));
    }
}

static const MemoryRegionOps hisi_crg_ops = {
    .read = hisi_crg_read,
    .write = hisi_crg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_crg_init(Object *obj)
{
    HisiCrgState *s = HISI_CRG(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_crg_ops, s,
                          TYPE_HISI_CRG, HISI_CRG_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), &s->cpu1_reset, "cpu1-reset", 1);
}

static void hisi_crg_reset(DeviceState *dev)
{
    HisiCrgState *s = HISI_CRG(dev);

    /*
     * Don't zero regs — CRG defaults (clock enables for UART, FMC, ETH)
     * are written during machine init and must survive machine reset.
     * Only reinitialize the CPU reset register for SMP bringup detection.
     */
    if (s->cpu_srst_offset) {
        s->regs[s->cpu_srst_offset / 4] = CPU1_SRST_REQ | DBG1_SRST_REQ;
    }
}

static const Property hisi_crg_properties[] = {
    DEFINE_PROP_UINT32("cpu-srst-offset", HisiCrgState, cpu_srst_offset, 0),
    DEFINE_PROP_UINT32("pll-lock-offset", HisiCrgState, pll_lock_offset, 0),
    DEFINE_PROP_UINT32("pll-lock-bits", HisiCrgState, pll_lock_bits, 0),
};

static void hisi_crg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, hisi_crg_properties);
    device_class_set_legacy_reset(dc, hisi_crg_reset);
}

static const TypeInfo hisi_crg_info = {
    .name          = TYPE_HISI_CRG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiCrgState),
    .instance_init = hisi_crg_init,
    .class_init    = hisi_crg_class_init,
};

static void hisi_crg_register_types(void)
{
    type_register_static(&hisi_crg_info);
}

type_init(hisi_crg_register_types)
