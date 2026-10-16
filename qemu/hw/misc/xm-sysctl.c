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
#include "system/runstate.h"

#define TYPE_XM_SYSCTL "xm-sysctl"
OBJECT_DECLARE_SIMPLE_TYPE(XmSysctlState, XM_SYSCTL)

#define XM_SYSCTL_SIZE      0x10000
#define XM_SOFT_RST_MAGIC   0xca110000

struct XmSysctlState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[XM_SYSCTL_SIZE / 4];
    uint32_t rsten_offset;
    uint32_t rst_offset;
};

static uint64_t xm_sysctl_read(void *opaque, hwaddr offset, unsigned size)
{
    XmSysctlState *s = XM_SYSCTL(opaque);

    return s->regs[offset / 4];
}

static void xm_sysctl_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    XmSysctlState *s = XM_SYSCTL(opaque);

    if (offset == s->rst_offset && val == XM_SOFT_RST_MAGIC &&
        (s->regs[s->rsten_offset / 4] & 1)) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return;
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

    memset(s->regs, 0, sizeof(s->regs));
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
    memory_region_init_io(&s->iomem, OBJECT(dev), &xm_sysctl_ops, s,
                          TYPE_XM_SYSCTL, XM_SYSCTL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static const Property xm_sysctl_properties[] = {
    DEFINE_PROP_UINT32("rsten-offset", XmSysctlState, rsten_offset, 0x30),
    DEFINE_PROP_UINT32("rst-offset", XmSysctlState, rst_offset, 0x34),
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
