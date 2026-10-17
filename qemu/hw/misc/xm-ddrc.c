/*
 * Xiongmai XM510 DDR controller (0x100B0000, three 4 KiB blocks).
 *
 * Only U-Boot touches it: 2014.04's lowlevel init trains DDR before
 * relocating out of SRAM.  The first block is a Synopsys uPCTL, going by
 * the registers that init drives (read from the board's own U-Boot, there
 * being no public source):
 *
 *   +0x0004  SCTL     state request: 0 INIT, 1 CFG, 2 GO, 3 SLEEP, 4 WAKEUP
 *   +0x0008  STAT     state reached:  0 Init_mem, 1 Config, 3 Access,
 *                     5 Low_power -- polled after each request
 *   +0x0040  MCMD     bit 31 start_cmd: set to issue, poll until clear
 *   +0x0048  POWSTAT  bit 0 power_up_done
 *   +0x1038           (PHY side) reads 1 once training is done
 *
 * Those last two are XM510's; XM530's U-Boot spins on +0x00F4 and +0x00F8
 * instead.  The words that read as done are a property, done-offsets.
 * XM530's also programs PHY +0xA8 and waits for +0xB8 to read the same
 * value back: a status word that echoes a request once it is applied.
 * Such pairs are mirror-reads (status) / mirror-of (request), offsets from
 * the block base.
 *
 * Everything else is RAM-backed so the init's read-modify-writes survive.
 * DDR itself is plain RAM in QEMU, so there is nothing to train.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qom/object.h"

#define TYPE_XM_DDRC "xm-ddrc"
OBJECT_DECLARE_SIMPLE_TYPE(XmDdrcState, XM_DDRC)

#define XM_DDRC_SIZE        0x3000
#define DDRC_SCTL           0x0004
#define DDRC_STAT           0x0008
#define DDRC_CMD            0x0040
#define DDRC_CMD_BUSY       (1u << 31)

struct XmDdrcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[XM_DDRC_SIZE / 4];
    uint32_t num_done;
    uint32_t *done;
    uint32_t num_mirror, num_mirror_of;
    uint32_t *mirror, *mirror_of;
};

static uint64_t xm_ddrc_read(void *opaque, hwaddr offset, unsigned size)
{
    XmDdrcState *s = XM_DDRC(opaque);

    uint32_t i;

    for (i = 0; i < s->num_done; i++) {
        if (offset == s->done[i]) {
            return 1;
        }
    }
    for (i = 0; i < s->num_mirror; i++) {
        if (offset == s->mirror[i]) {
            return s->regs[s->mirror_of[i] / 4];
        }
    }
    switch (offset) {
    case DDRC_CMD:
        return s->regs[offset / 4] & ~DDRC_CMD_BUSY;
    default:
        return s->regs[offset / 4];
    }
}

static void xm_ddrc_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    XmDdrcState *s = XM_DDRC(opaque);

    if (offset == DDRC_SCTL) {
        /* States are reached at once: DDR is plain RAM here. */
        static const uint32_t reached[] = { 0, 1, 3, 5, 3 };
        if (val < ARRAY_SIZE(reached)) {
            s->regs[DDRC_STAT / 4] = reached[val];
        }
    }
    s->regs[offset / 4] = val;
}

static const MemoryRegionOps xm_ddrc_ops = {
    .read = xm_ddrc_read,
    .write = xm_ddrc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_ddrc_reset(DeviceState *dev)
{
    XmDdrcState *s = XM_DDRC(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void xm_ddrc_init(Object *obj)
{
    XmDdrcState *s = XM_DDRC(obj);

    memory_region_init_io(&s->iomem, obj, &xm_ddrc_ops, s, TYPE_XM_DDRC,
                          XM_DDRC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void xm_ddrc_realize(DeviceState *dev, Error **errp)
{
    XmDdrcState *s = XM_DDRC(dev);
    uint32_t i;

    for (i = 0; i < s->num_done; i++) {
        if (s->done[i] >= XM_DDRC_SIZE || (s->done[i] & 3)) {
            error_setg(errp, "xm-ddrc: bad done offset 0x%x", s->done[i]);
            return;
        }
    }
    if (s->num_mirror != s->num_mirror_of) {
        error_setg(errp, "xm-ddrc: mirror-reads and mirror-of differ in length");
        return;
    }
    for (i = 0; i < s->num_mirror; i++) {
        if (s->mirror[i] >= XM_DDRC_SIZE || (s->mirror[i] & 3) ||
            s->mirror_of[i] >= XM_DDRC_SIZE || (s->mirror_of[i] & 3)) {
            error_setg(errp, "xm-ddrc: bad mirror pair 0x%x/0x%x",
                       s->mirror[i], s->mirror_of[i]);
            return;
        }
    }
}

static const Property xm_ddrc_props[] = {
    DEFINE_PROP_ARRAY("done-offsets", XmDdrcState, num_done, done,
                      qdev_prop_uint32, uint32_t),
    DEFINE_PROP_ARRAY("mirror-reads", XmDdrcState, num_mirror, mirror,
                      qdev_prop_uint32, uint32_t),
    DEFINE_PROP_ARRAY("mirror-of", XmDdrcState, num_mirror_of, mirror_of,
                      qdev_prop_uint32, uint32_t),
};

static void xm_ddrc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, xm_ddrc_reset);
    device_class_set_props(dc, xm_ddrc_props);
    dc->realize = xm_ddrc_realize;
}

static const TypeInfo xm_ddrc_info = {
    .name          = TYPE_XM_DDRC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmDdrcState),
    .instance_init = xm_ddrc_init,
    .class_init    = xm_ddrc_class_init,
};

static void xm_ddrc_register_types(void)
{
    type_register_static(&xm_ddrc_info);
}

type_init(xm_ddrc_register_types)
