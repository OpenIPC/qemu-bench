/*
 * Xiongmai XM533 FFT engine at 0x21300000, as ive.ko drives it (IRQ 95,
 * SPI 63).  fft_start writes the job's words to +0x00..+0x10 and +0x18,
 * then +0x14 last, which starts it; fft_irq checks that +0x1C's low byte
 * reads 9 (done) and acknowledges with 0 to +0x18.
 *
 * Jobs complete a millisecond after they start and compute nothing.
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

#define TYPE_XM_FFT "xm-fft"
OBJECT_DECLARE_SIMPLE_TYPE(XmFftState, XM_FFT)

#define XM_FFT_SIZE     0x40
#define FFT_GO          0x14    /* the job's last word: starts it */
#define FFT_ACK         0x18    /* also a job word; 0 clears done */
#define FFT_STATUS      0x1c    /* low byte 9: done */
#define FFT_DONE        9

struct XmFftState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[XM_FFT_SIZE / 4];
};

static void xm_fft_update(XmFftState *s)
{
    qemu_set_irq(s->irq, (s->regs[FFT_STATUS / 4] & 0xff) == FFT_DONE);
}

static void xm_fft_done(void *opaque)
{
    XmFftState *s = XM_FFT(opaque);

    s->regs[FFT_STATUS / 4] = FFT_DONE;
    xm_fft_update(s);
}

static uint64_t xm_fft_read(void *opaque, hwaddr off, unsigned size)
{
    return XM_FFT(opaque)->regs[off / 4];
}

static void xm_fft_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    XmFftState *s = XM_FFT(opaque);

    switch (off) {
    case FFT_GO:
        s->regs[off / 4] = val;
        s->regs[FFT_STATUS / 4] = 0;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SCALE_MS);
        break;
    case FFT_ACK:
        s->regs[off / 4] = val;
        if (!val) {
            s->regs[FFT_STATUS / 4] = 0;
        }
        break;
    case FFT_STATUS:
        break;
    default:
        s->regs[off / 4] = val;
        break;
    }
    xm_fft_update(s);
}

static const MemoryRegionOps xm_fft_ops = {
    .read = xm_fft_read,
    .write = xm_fft_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_fft_reset(DeviceState *dev)
{
    XmFftState *s = XM_FFT(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    xm_fft_update(s);
}

static void xm_fft_init(Object *obj)
{
    XmFftState *s = XM_FFT(obj);

    memory_region_init_io(&s->iomem, obj, &xm_fft_ops, s, TYPE_XM_FFT,
                          XM_FFT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_fft_done, s);
}

static void xm_fft_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_fft_reset);
}

static const TypeInfo xm_fft_info = {
    .name          = TYPE_XM_FFT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmFftState),
    .instance_init = xm_fft_init,
    .class_init    = xm_fft_class_init,
};

static void xm_fft_register_types(void)
{
    type_register_static(&xm_fft_info);
}

type_init(xm_fft_register_types)
