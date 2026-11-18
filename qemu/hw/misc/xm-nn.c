/*
 * Xiongmai second-generation NN accelerator ("xmnn"; 0x21200000 on
 * XM530V200, 0x21500000 on XM535), as nn.ko drives it for the jc_v200
 * library in Sofia (human detection).  nn_start writes the job (+0x00,
 * +0x10, +0x14, +0x18), 4 and then the go value to +0x04; nn_irq (SPI 43)
 * checks bit 0 of +0x08 and acknowledges with 4 to +0x04.
 *
 * Jobs complete a millisecond after they start and compute nothing: the
 * network's outputs stay as the library left them, which it reads as no
 * detections.  Without the completion every job waits out nn.ko's timeout
 * ("cnn wait for timeout") and the library's error path takes Sofia down.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_XM_NN "xm-nn"
OBJECT_DECLARE_SIMPLE_TYPE(XmNnState, XM_NN)

#define XM_NN_SIZE      0x200
#define NN_CTRL         0x04    /* 4: acknowledge / clear; else: start */
#define NN_STATUS       0x08    /* bit 0: done */
#define NN_CTRL_ACK     4

struct XmNnState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[XM_NN_SIZE / 4];
};

static void xm_nn_update(XmNnState *s)
{
    qemu_set_irq(s->irq, s->regs[NN_STATUS / 4] & 1);
}

static void xm_nn_done(void *opaque)
{
    XmNnState *s = XM_NN(opaque);

    s->regs[NN_STATUS / 4] |= 1;
    xm_nn_update(s);
}

static uint64_t xm_nn_read(void *opaque, hwaddr off, unsigned size)
{
    return XM_NN(opaque)->regs[off / 4];
}

static void xm_nn_write(void *opaque, hwaddr off, uint64_t val,
                        unsigned size)
{
    XmNnState *s = XM_NN(opaque);

    switch (off) {
    case NN_CTRL:
        s->regs[off / 4] = val;
        if (val == NN_CTRL_ACK) {
            s->regs[NN_STATUS / 4] &= ~1u;
        } else if (val) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      SCALE_MS);
        }
        break;
    case NN_STATUS:
        break;
    default:
        s->regs[off / 4] = val;
        break;
    }
    xm_nn_update(s);
}

static const MemoryRegionOps xm_nn_ops = {
    .read = xm_nn_read,
    .write = xm_nn_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_nn_reset(DeviceState *dev)
{
    XmNnState *s = XM_NN(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    xm_nn_update(s);
}

static void xm_nn_init(Object *obj)
{
    XmNnState *s = XM_NN(obj);

    memory_region_init_io(&s->iomem, obj, &xm_nn_ops, s, TYPE_XM_NN,
                          XM_NN_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_nn_done, s);
}

static void xm_nn_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_nn_reset);
}

static const TypeInfo xm_nn_info = {
    .name          = TYPE_XM_NN,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmNnState),
    .instance_init = xm_nn_init,
    .class_init    = xm_nn_class_init,
};

static void xm_nn_register_types(void)
{
    type_register_static(&xm_nn_info);
}

type_init(xm_nn_register_types)
