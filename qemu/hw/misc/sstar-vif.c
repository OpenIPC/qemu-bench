/*
 * SigmaStar Ifado VIF: its channels' frame interrupts and frame clock.
 *
 * RIU bank 0x1308 (0x1F261000) holds VIF channels 0-3 (sstar_mi.ko
 * HAL_VIF_GetChnBaseAddr, HAL_VIF_IspVif*): four interrupt bits per
 * channel, channel n in bits [4n+3:4n], with the mask at reg 0x60
 * (+0x180), the clear at reg 0x6A (+0x1A8; bits written as 1 clear) and
 * the status at reg 0x70 (+0x1C0).  DRV_VIF_EventIspVifIsr runs one
 * event callback per bit.
 *
 * There is no sensor, so this is the frame clock: while any bit of
 * "events" is unmasked, a timer at "fps" raises those bits each frame.
 * Everything else in the bank is storage.
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

#define TYPE_SSTAR_VIF "sstar-vif"
OBJECT_DECLARE_SIMPLE_TYPE(SstarVifState, SSTAR_VIF)

#define ISP_REGS            0x80
#define INT_GROUPS          1
#define INT_MASK(g)         0x60
#define INT_CLEAR(g)        0x6A
#define INT_STATUS(g)       0x70

struct SstarVifState {
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
    return reg == INT_MASK(0) || reg == INT_CLEAR(0) ||
           reg == INT_STATUS(0) ? 0 : -1;
}

static void sstar_vif_update_irq(SstarVifState *s)
{
    int g;
    bool level = false;

    for (g = 0; g < INT_GROUPS; g++) {
        level |= !!(s->regs[INT_STATUS(g)] & ~s->regs[INT_MASK(g)]);
    }
    qemu_set_irq(s->irq, level);
}

static bool sstar_vif_streaming(SstarVifState *s)
{
    int g;

    for (g = 0; g < INT_GROUPS; g++) {
        if (s->events[g] & ~s->regs[INT_MASK(g)]) {
            return true;
        }
    }
    return false;
}

static void sstar_vif_schedule(SstarVifState *s)
{
    if (sstar_vif_streaming(s) && !timer_pending(s->frame)) {
        timer_mod(s->frame, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
    }
}

static void sstar_vif_frame(void *opaque)
{
    SstarVifState *s = opaque;

    int g;

    s->frames++;
    for (g = 0; g < INT_GROUPS; g++) {
        s->regs[INT_STATUS(g)] |= s->events[g];
    }
    if (s->trace) {
        qemu_log("sstar-vif: %" PRId64 " ms frame %u, status 0x%04x "
                 "mask 0x%04x\n",
                 qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL),
                 s->frames, s->regs[INT_STATUS(0)], s->regs[INT_MASK(0)]);
    }
    sstar_vif_update_irq(s);
    sstar_vif_schedule(s);
}

static uint64_t sstar_vif_read(void *opaque, hwaddr off, unsigned size)
{
    SstarVifState *s = opaque;
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

static void sstar_vif_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    SstarVifState *s = opaque;
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
        qemu_log("sstar-vif: W reg 0x%02x = 0x%04x/0x%04x\n", reg, v, mask);
    }
    if (g >= 0 && reg == INT_STATUS(g)) {
        return;
    }
    s->regs[reg] = (s->regs[reg] & ~mask) | (v & mask);
    if (g >= 0) {
        if (reg == INT_CLEAR(g)) {
            s->regs[INT_STATUS(g)] &= ~(v & mask);
        }
        sstar_vif_update_irq(s);
        sstar_vif_schedule(s);
    }
}

static const MemoryRegionOps sstar_vif_ops = {
    .read = sstar_vif_read,
    .write = sstar_vif_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_vif_reset(DeviceState *dev)
{
    SstarVifState *s = SSTAR_VIF(dev);
    int g;

    memset(s->regs, 0, sizeof(s->regs));
    for (g = 0; g < INT_GROUPS; g++) {
        s->regs[INT_MASK(g)] = 0xFFFF;
    }
    s->frames = 0;
    timer_del(s->frame);
    qemu_set_irq(s->irq, 0);
}

static void sstar_vif_realize(DeviceState *dev, Error **errp)
{
    SstarVifState *s = SSTAR_VIF(dev);

    s->frame = timer_new_ns(QEMU_CLOCK_VIRTUAL, sstar_vif_frame, s);
}

static void sstar_vif_init(Object *obj)
{
    SstarVifState *s = SSTAR_VIF(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_vif_ops, s,
                          TYPE_SSTAR_VIF, ISP_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property sstar_vif_props[] = {
    DEFINE_PROP_UINT32("fps", SstarVifState, fps, 25),
    /* channel 0's bits 0, 1 and 3: the ones the driver unmasks */
    DEFINE_PROP_UINT32("events", SstarVifState, events[0], 0xB),
    DEFINE_PROP_BOOL("trace", SstarVifState, trace, false),
};

static const VMStateDescription vmstate_sstar_isp = {
    .name = TYPE_SSTAR_VIF,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarVifState, ISP_REGS),
        VMSTATE_UINT32(frames, SstarVifState),
        VMSTATE_TIMER_PTR(frame, SstarVifState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_vif_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_vif_realize;
    device_class_set_legacy_reset(dc, sstar_vif_reset);
    device_class_set_props(dc, sstar_vif_props);
    dc->vmsd = &vmstate_sstar_isp;
}

static const TypeInfo sstar_vif_info = {
    .name           = TYPE_SSTAR_VIF,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarVifState),
    .instance_init  = sstar_vif_init,
    .class_init     = sstar_vif_class_init,
};

static void sstar_vif_register_types(void)
{
    type_register_static(&sstar_vif_info);
}

type_init(sstar_vif_register_types)
