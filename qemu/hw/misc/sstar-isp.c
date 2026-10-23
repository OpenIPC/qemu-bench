/*
 * SigmaStar Ifado ISP top: interrupts and the frame clock.
 *
 * RIU bank 0x1302 (0x1F260400), 16-bit registers on a 4-byte stride, is
 * the ISP0 top (sstar_mi.ko HalIsp*).  Seven interrupt groups of four
 * registers each from byte offset 0xA0 (HalIspGetIntStatus1..7,
 * HalIspEnableIrq, HalIspMaskClear1..7):
 *
 *   +0x0  mask (1 = masked)
 *   +0x4  force
 *   +0x8  clear: bits written as 1 clear the status
 *   +0xC  status; the driver takes status & ~mask
 *
 * There is no sensor here, so the model is the frame clock: while any
 * bit of "events0".."events6" is unmasked, a timer at "fps" raises them
 * each frame (group 0 bit 12 is the sensor frame the ISR counts,
 * HalIspSetSensorFrameCount).  The line is the OR of status & ~mask over
 * the groups.  Everything else in the bank is storage.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_SSTAR_ISP "sstar-isp"
OBJECT_DECLARE_SIMPLE_TYPE(SstarIspState, SSTAR_ISP)

#define ISP_REGS            0x80
#define INT_BASE            0x28        /* reg index of group 0's mask */
#define INT_GROUPS          7
#define INT_MASK(g)         (INT_BASE + 4 * (g))
#define INT_FORCE(g)        (INT_BASE + 4 * (g) + 1)
#define INT_CLEAR(g)        (INT_BASE + 4 * (g) + 2)
#define INT_STATUS(g)       (INT_BASE + 4 * (g) + 3)

struct SstarIspState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *frame;
    uint16_t regs[ISP_REGS];
    uint32_t frames;

    uint32_t fps;
    uint32_t events[INT_GROUPS];    /* bits raised each frame, per group */
    bool trace;
};

static int isp_group(int reg)
{
    if (reg < INT_BASE || reg >= INT_BASE + 4 * INT_GROUPS) {
        return -1;
    }
    return (reg - INT_BASE) / 4;
}

static void sstar_isp_update_irq(SstarIspState *s)
{
    int g;
    bool level = false;

    for (g = 0; g < INT_GROUPS; g++) {
        level |= !!(s->regs[INT_STATUS(g)] & ~s->regs[INT_MASK(g)]);
    }
    qemu_set_irq(s->irq, level);
}

static bool sstar_isp_streaming(SstarIspState *s)
{
    int g;

    for (g = 0; g < INT_GROUPS; g++) {
        if (s->events[g] & ~s->regs[INT_MASK(g)]) {
            return true;
        }
    }
    return false;
}

static void sstar_isp_schedule(SstarIspState *s)
{
    if (sstar_isp_streaming(s) && !timer_pending(s->frame)) {
        timer_mod(s->frame, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
    }
}

static void sstar_isp_frame(void *opaque)
{
    SstarIspState *s = opaque;

    int g;

    s->frames++;
    for (g = 0; g < INT_GROUPS; g++) {
        s->regs[INT_STATUS(g)] |= s->events[g];
    }
    if (s->trace) {
        qemu_log("sstar-isp: frame %u, status 0x%04x mask 0x%04x\n",
                 s->frames, s->regs[INT_STATUS(0)], s->regs[INT_MASK(0)]);
    }
    sstar_isp_update_irq(s);
    sstar_isp_schedule(s);
}

static uint64_t sstar_isp_read(void *opaque, hwaddr off, unsigned size)
{
    SstarIspState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->regs[(off >> 2) & (ISP_REGS - 1)];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_isp_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    SstarIspState *s = opaque;
    int reg = (off >> 2) & (ISP_REGS - 1);
    int g = isp_group(reg);
    uint16_t mask, v;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        mask = off & 1 ? 0xFF00 : 0x00FF;
        v = off & 1 ? (val & 0xFF) << 8 : val & 0xFF;
    } else {
        mask = 0xFFFF;
        v = val;
    }
    if (s->trace && g >= 0) {
        qemu_log("sstar-isp: W reg 0x%02x (group %d +%d) = 0x%04x/0x%04x\n",
                 reg, g, (reg - INT_BASE) % 4, v, mask);
    }
    if (g >= 0 && reg == INT_STATUS(g)) {
        return;
    }
    s->regs[reg] = (s->regs[reg] & ~mask) | (v & mask);
    if (g >= 0) {
        if (reg == INT_CLEAR(g)) {
            s->regs[INT_STATUS(g)] &= ~(v & mask);
        } else if (reg == INT_FORCE(g)) {
            s->regs[INT_STATUS(g)] |= v & mask;
        }
        sstar_isp_update_irq(s);
        sstar_isp_schedule(s);
    }
}

static const MemoryRegionOps sstar_isp_ops = {
    .read = sstar_isp_read,
    .write = sstar_isp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_isp_reset(DeviceState *dev)
{
    SstarIspState *s = SSTAR_ISP(dev);
    int g;

    memset(s->regs, 0, sizeof(s->regs));
    for (g = 0; g < INT_GROUPS; g++) {
        s->regs[INT_MASK(g)] = 0xFFFF;
    }
    s->frames = 0;
    timer_del(s->frame);
    qemu_set_irq(s->irq, 0);
}

static void sstar_isp_realize(DeviceState *dev, Error **errp)
{
    SstarIspState *s = SSTAR_ISP(dev);

    s->frame = timer_new_ns(QEMU_CLOCK_VIRTUAL, sstar_isp_frame, s);
}

static void sstar_isp_init(Object *obj)
{
    SstarIspState *s = SSTAR_ISP(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_isp_ops, s,
                          TYPE_SSTAR_ISP, ISP_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property sstar_isp_props[] = {
    DEFINE_PROP_UINT32("fps", SstarIspState, fps, 25),
    /*
     * Per frame, as HalIspIsr maps them: group 0 bit 12 (frame start,
     * event 0), group 1 bit 14 (event 1), group 4 bits 0 and 3 (events 2
     * and 3).  Group 0 bit 6 is the overflow/IP-reset path.
     */
    DEFINE_PROP_UINT32("events0", SstarIspState, events[0], 1u << 12),
    DEFINE_PROP_UINT32("events1", SstarIspState, events[1], 1u << 14),
    DEFINE_PROP_UINT32("events2", SstarIspState, events[2], 0),
    DEFINE_PROP_UINT32("events3", SstarIspState, events[3], 0),
    DEFINE_PROP_UINT32("events4", SstarIspState, events[4], 0x9),
    DEFINE_PROP_UINT32("events5", SstarIspState, events[5], 0),
    DEFINE_PROP_UINT32("events6", SstarIspState, events[6], 0),
    DEFINE_PROP_BOOL("trace", SstarIspState, trace, false),
};

static const VMStateDescription vmstate_sstar_isp = {
    .name = TYPE_SSTAR_ISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarIspState, ISP_REGS),
        VMSTATE_UINT32(frames, SstarIspState),
        VMSTATE_TIMER_PTR(frame, SstarIspState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_isp_realize;
    device_class_set_legacy_reset(dc, sstar_isp_reset);
    device_class_set_props(dc, sstar_isp_props);
    dc->vmsd = &vmstate_sstar_isp;
}

static const TypeInfo sstar_isp_info = {
    .name           = TYPE_SSTAR_ISP,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarIspState),
    .instance_init  = sstar_isp_init,
    .class_init     = sstar_isp_class_init,
};

static void sstar_isp_register_types(void)
{
    type_register_static(&sstar_isp_info);
}

type_init(sstar_isp_register_types)
