/*
 * Xiongmai XM510 / XM530 / XM550 system controller (SYS_CTRL, 0x20000000).
 *
 * RAM-backed like hisi-regbank -- the vendor kernels and U-Boot
 * read-modify-write clock and pin bits here -- plus the one register with
 * a side effect that matters to the OS: soft reset.  Both kernels restart
 * the same way, only the offsets differ (mach-xm510/include/mach/system.h
 * arch_reset(), mach-xm530/core.c xm530_restart()):
 *
 *     writel(1,          SYS + REG_SYS_SOFT_RSTEN);   0x30 / 0x80
 *     writel(0xca110000, SYS + REG_SYS_SOFT_RST);     0x34 / 0x84
 *
 * Without it a guest "reboot" spins in the restart handler forever.
 *
 * U-Boot's lowlevel init also programs each PLL and spins until its status
 * word reads 1 (XM510: control at +0x04/+0x18/+0x20, lock at +0x08/+0x1C/
 * +0x28).  The words named in pll-lock-mask (bit n = offset 4n) are those
 * status words: they always read locked.  XM530's U-Boot instead polls the
 * PLL control words themselves (+0x08/+0x10/+0x18/+0x20) for bit 31, LOCK:
 * the words in pll-lock31-mask read back with bit 31 set.
 *
 * XM530/XM550 add what a dual-core part needs (mach-xm530/hotplug.c):
 *  - REG_SC_REMAP (+0x204): 1 puts RAM at address 0 in place of the boot
 *    flash alias; the kernel waits for it to read back non-zero, then
 *    writes its secondary trampoline at physical 0.  Drives the "remap"
 *    output, which the machine turns into that memory switch.
 *  - REG_CPU1_SOFT_RST (+0x8C), honoured while RSTEN is 1: 0xCA11C100
 *    holds CPU1 in reset, 1 lets it go.  Drives "cpu1-reset" (1 = held).
 *  - The kernel derives the TWD rate from the PLL words (+0x08, +0x28)
 *    with no U-Boot in front of it under -kernel, so init-offsets /
 *    init-values give them values at reset.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "system/runstate.h"

#define TYPE_XM_SYSCTL "xm-sysctl"
OBJECT_DECLARE_SIMPLE_TYPE(XmSysctlState, XM_SYSCTL)

#define XM_SYSCTL_SIZE      0x10000
#define XM_SOFT_RST_MAGIC   0xca110000
#define XM_CPU1_HOLD        0xca11c100
#define XM_MAX_INIT         8

struct XmSysctlState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[XM_SYSCTL_SIZE / 4];
    uint32_t rsten_offset;
    uint32_t rst_offset;
    uint32_t pll_lock_mask;
    uint32_t pll_lock31_mask;
    uint32_t remap_offset;      /* 0: none */
    uint32_t cpu1_offset;       /* 0: none */
    uint32_t num_init_offsets, num_init_values;
    uint32_t *init_offsets, *init_values;
    qemu_irq remap;
    qemu_irq cpu1_reset;
};

static uint64_t xm_sysctl_read(void *opaque, hwaddr offset, unsigned size)
{
    XmSysctlState *s = XM_SYSCTL(opaque);

    if (offset < 32 * 4 && (s->pll_lock_mask & (1u << (offset / 4)))) {
        return 1;
    }
    if (offset < 32 * 4 && (s->pll_lock31_mask & (1u << (offset / 4)))) {
        return s->regs[offset / 4] | 0x80000000u;
    }
    return s->regs[offset / 4];
}

static void xm_sysctl_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    XmSysctlState *s = XM_SYSCTL(opaque);

    const bool unlocked = s->regs[s->rsten_offset / 4] & 1;

    if (offset == s->rst_offset && val == XM_SOFT_RST_MAGIC && unlocked) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return;
    }
    if (s->cpu1_offset && offset == s->cpu1_offset && unlocked) {
        if (val == XM_CPU1_HOLD) {
            qemu_irq_raise(s->cpu1_reset);
        } else if (val == 1) {
            qemu_irq_lower(s->cpu1_reset);
        }
    }
    if (s->remap_offset && offset == s->remap_offset) {
        qemu_set_irq(s->remap, val & 1);
    }
    s->regs[offset / 4] = val;
}

static const MemoryRegionOps xm_sysctl_ops = {
    .read = xm_sysctl_read,
    .write = xm_sysctl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_sysctl_reset(DeviceState *dev)
{
    XmSysctlState *s = XM_SYSCTL(dev);

    uint32_t i;

    memset(s->regs, 0, sizeof(s->regs));
    for (i = 0; i < s->num_init_offsets && i < s->num_init_values; i++) {
        s->regs[s->init_offsets[i] / 4] = s->init_values[i];
    }
    /* After a reset CPU1 reads as held, and address 0 is the flash again. */
    if (s->cpu1_offset) {
        s->regs[s->cpu1_offset / 4] = 1;
    }
    qemu_irq_lower(s->remap);
}

static void xm_sysctl_realize(DeviceState *dev, Error **errp)
{
    XmSysctlState *s = XM_SYSCTL(dev);

    if (s->rsten_offset >= XM_SYSCTL_SIZE || s->rst_offset >= XM_SYSCTL_SIZE ||
        (s->rsten_offset & 3) || (s->rst_offset & 3)) {
        error_setg(errp, "xm-sysctl: reset offsets must be 4-aligned "
                   "and inside the 64 KiB block");
        return;
    }
    uint32_t i;

    if (s->num_init_offsets != s->num_init_values) {
        error_setg(errp, "xm-sysctl: init-offsets and init-values differ "
                   "in length");
        return;
    }
    for (i = 0; i < s->num_init_offsets; i++) {
        if (s->init_offsets[i] >= XM_SYSCTL_SIZE || (s->init_offsets[i] & 3)) {
            error_setg(errp, "xm-sysctl: bad init offset 0x%x",
                       s->init_offsets[i]);
            return;
        }
    }
    if (s->remap_offset >= XM_SYSCTL_SIZE || s->cpu1_offset >= XM_SYSCTL_SIZE) {
        error_setg(errp, "xm-sysctl: remap/cpu1 offset outside the block");
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(dev), &xm_sysctl_ops, s,
                          TYPE_XM_SYSCTL, XM_SYSCTL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    qdev_init_gpio_out_named(dev, &s->remap, "remap", 1);
    qdev_init_gpio_out_named(dev, &s->cpu1_reset, "cpu1-reset", 1);
}

static const Property xm_sysctl_properties[] = {
    DEFINE_PROP_UINT32("rsten-offset", XmSysctlState, rsten_offset, 0x30),
    DEFINE_PROP_UINT32("rst-offset", XmSysctlState, rst_offset, 0x34),
    DEFINE_PROP_UINT32("pll-lock-mask", XmSysctlState, pll_lock_mask, 0),
    DEFINE_PROP_UINT32("pll-lock31-mask", XmSysctlState, pll_lock31_mask, 0),
    DEFINE_PROP_UINT32("remap-offset", XmSysctlState, remap_offset, 0),
    DEFINE_PROP_UINT32("cpu1-offset", XmSysctlState, cpu1_offset, 0),
    DEFINE_PROP_ARRAY("init-offsets", XmSysctlState, num_init_offsets,
                      init_offsets, qdev_prop_uint32, uint32_t),
    DEFINE_PROP_ARRAY("init-values", XmSysctlState, num_init_values,
                      init_values, qdev_prop_uint32, uint32_t),
};

static void xm_sysctl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, xm_sysctl_properties);
    device_class_set_legacy_reset(dc, xm_sysctl_reset);
    dc->realize = xm_sysctl_realize;
}

static const TypeInfo xm_sysctl_info = {
    .name          = TYPE_XM_SYSCTL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmSysctlState),
    .class_init    = xm_sysctl_class_init,
};

static void xm_sysctl_register_types(void)
{
    type_register_static(&xm_sysctl_info);
}

type_init(xm_sysctl_register_types)
