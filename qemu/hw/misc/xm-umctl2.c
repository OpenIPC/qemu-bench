/*
 * Xiongmai XM530V200 / XM533 / XM535 DDR controller (0x20050000, 64 KiB).
 *
 * The later XM53x parts moved DDR off XM510's uPCTL at 0x100B0000 onto a
 * Synopsys DesignWare uMCTL2, with the PHY in the same window.  Only
 * U-Boot 2014.04's lowlevel init touches it, before relocating out of
 * SRAM.  What it waits on, read from those U-Boots (no public source):
 *
 *   +0x0004  STAT     bits [2:0] operating_mode, 1 = normal
 *   +0x01BC  DFISTAT  bit 0 dfi_init_complete
 *   +0x0320  SWCTL    bit 0 sw_done: 0 opens a quasi-dynamic update...
 *   +0x0324  SWSTAT   bit 0 sw_done_ack ...which is acked once it is 1
 *   +0x8004           PHY command: bit 0 starts training, bit 4 (with
 *                     bit 0 clear) a step the 2023+ U-Boots run first
 *   +0x8174           PHY status, polled for an exact value: 0x403 once
 *                     training is done, 0x30000 after the bit-4 step
 *   +0x80A4, +0x80B0  two more PHY training requests, bit 0 = start
 *                     (only the XM530V200 / XM535D2 U-Boots use them)
 *   +0x8158           their status, polled for an exact value: 0xC0 while
 *                     +0xB0 is started, else 1 while +0xA4 is
 *   +0xC004           PHY command word, read back until it holds what
 *                     was written (RAM-backed covers it)
 *
 * Everything else is RAM-backed so the init's read-modify-writes survive.
 * DDR itself is plain RAM in QEMU, so there is nothing to train.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_XM_UMCTL2 "xm-umctl2"
OBJECT_DECLARE_SIMPLE_TYPE(XmUmctl2State, XM_UMCTL2)

#define XM_UMCTL2_SIZE      0x10000
#define UMCTL2_STAT         0x0004
#define UMCTL2_DFISTAT      0x01BC
#define UMCTL2_SWCTL        0x0320
#define UMCTL2_SWSTAT       0x0324
#define PHY_CMD             0x8004
#define PHY_CMD_TRAIN       (1u << 0)
#define PHY_CMD_STEP        (1u << 4)
#define PHY_TRAIN_STAT      0x8174
#define PHY_TRAIN_DONE      0x403
#define PHY_STEP_DONE       0x30000
#define PHY_REQ_A           0x80A4
#define PHY_REQ_B           0x80B0
#define PHY_REQ_STAT        0x8158

struct XmUmctl2State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[XM_UMCTL2_SIZE / 4];
};

static uint64_t xm_umctl2_read(void *opaque, hwaddr offset, unsigned size)
{
    XmUmctl2State *s = XM_UMCTL2(opaque);

    switch (offset) {
    case UMCTL2_STAT:
        return (s->regs[offset / 4] & ~7u) | 1;
    case UMCTL2_DFISTAT:
        return s->regs[offset / 4] | 1;
    case UMCTL2_SWSTAT:
        return s->regs[UMCTL2_SWCTL / 4] & 1;
    case PHY_TRAIN_STAT: {
        uint32_t cmd = s->regs[PHY_CMD / 4];

        if (!(cmd & PHY_CMD_TRAIN) && (cmd & PHY_CMD_STEP)) {
            return PHY_STEP_DONE;
        }
        return PHY_TRAIN_DONE;
    }
    case PHY_REQ_STAT:
        if (s->regs[PHY_REQ_B / 4] & 1) {
            return 0xC0;
        }
        return s->regs[PHY_REQ_A / 4] & 1;
    default:
        return s->regs[offset / 4];
    }
}

static void xm_umctl2_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    XmUmctl2State *s = XM_UMCTL2(opaque);

    s->regs[offset / 4] = val;
}

static const MemoryRegionOps xm_umctl2_ops = {
    .read = xm_umctl2_read,
    .write = xm_umctl2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_umctl2_reset(DeviceState *dev)
{
    XmUmctl2State *s = XM_UMCTL2(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void xm_umctl2_init(Object *obj)
{
    XmUmctl2State *s = XM_UMCTL2(obj);

    memory_region_init_io(&s->iomem, obj, &xm_umctl2_ops, s, TYPE_XM_UMCTL2,
                          XM_UMCTL2_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void xm_umctl2_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, xm_umctl2_reset);
}

static const TypeInfo xm_umctl2_info = {
    .name          = TYPE_XM_UMCTL2,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmUmctl2State),
    .instance_init = xm_umctl2_init,
    .class_init    = xm_umctl2_class_init,
};

static void xm_umctl2_register_types(void)
{
    type_register_static(&xm_umctl2_info);
}

type_init(xm_umctl2_register_types)
