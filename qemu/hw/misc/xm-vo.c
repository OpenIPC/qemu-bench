/*
 * Xiongmai XM530V200 video output at 0x31000000 (vo.ko), a register file
 * with what vo.ko waits on: word 0 (the update strobe) and 0x1800 (the
 * image layer's) read back 0 once written; with the frame interrupt on
 * (0x1010 = 1) and the timing running (0x1006 = 1) word 0x1011 reads 1
 * every 40 ms and raises the interrupt (SPI 27, IRQ 59) until it is
 * written 0.  Nothing is displayed.  Writes go to the XM_REGTRACE log as
 * "vo".
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/xm-regtrace.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_XM_VO "xm-vo"
OBJECT_DECLARE_SIMPLE_TYPE(XmVoState, XM_VO)

#define XM_VO_SIZE      0x40000
#define VO_STROBE       0x0000
#define VO_RUN          (0x1006 * 4)
#define VO_LINE_IE      (0x1010 * 4)
#define VO_LINE_ST      (0x1011 * 4)
#define VO_IMG_STROBE   (0x1800 * 4)
#define VO_FRAME_NS     (40 * SCALE_MS)

struct XmVoState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[XM_VO_SIZE / 4];
};

static void xm_vo_update(XmVoState *s)
{
    qemu_set_irq(s->irq, s->regs[VO_LINE_ST / 4] & 1);
}

static void xm_vo_frame(void *opaque)
{
    XmVoState *s = XM_VO(opaque);

    if (s->regs[VO_RUN / 4] && s->regs[VO_LINE_IE / 4]) {
        s->regs[VO_LINE_ST / 4] = 1;
        xm_vo_update(s);
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VO_FRAME_NS);
}

static uint64_t xm_vo_read(void *opaque, hwaddr off, unsigned size)
{
    return XM_VO(opaque)->regs[off / 4];
}

static void xm_vo_write(void *opaque, hwaddr off, uint64_t val,
                        unsigned size)
{
    XmVoState *s = XM_VO(opaque);

    xm_regtrace("vo", off, val);
    switch (off) {
    case VO_STROBE:
    case VO_IMG_STROBE:
        s->regs[off / 4] = 0;   /* taken at once */
        break;
    default:
        s->regs[off / 4] = val;
        break;
    }
    xm_vo_update(s);
}

static const MemoryRegionOps xm_vo_ops = {
    .read = xm_vo_read,
    .write = xm_vo_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_vo_reset(DeviceState *dev)
{
    XmVoState *s = XM_VO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VO_FRAME_NS);
    xm_vo_update(s);
}

static void xm_vo_init(Object *obj)
{
    XmVoState *s = XM_VO(obj);

    memory_region_init_io(&s->iomem, obj, &xm_vo_ops, s, TYPE_XM_VO,
                          XM_VO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_vo_frame, s);
}

static void xm_vo_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_vo_reset);
}

static const TypeInfo xm_vo_info = {
    .name          = TYPE_XM_VO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmVoState),
    .instance_init = xm_vo_init,
    .class_init    = xm_vo_class_init,
};

static void xm_vo_register_types(void)
{
    type_register_static(&xm_vo_info);
}

type_init(xm_vo_register_types)
