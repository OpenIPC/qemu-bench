/*
 * HiSilicon V4 VI (Hi3516EV200 family): capture (VI_CAP) and the ISP
 * pipe (VI_PROC) as the frame clock of the online pipeline.
 *
 * The sensor's frames never exist here: VPSS (hisi-vpss) paints the
 * pictures.  What VI must do is what hi_vi.o waits for every frame:
 *
 *   VI_CAP (0x11000000):
 *     0x00f0  status: bit 8 pipe 0 (while its write channel has anything
 *             pending), bit 9 pipe 1 (VI_DRV_IsCapValidInt wants one of 0xf00)
 *     0x00f8  mask (the driver sets all of it when it loads)
 *     0x10000 + pipe * 0x1000 + 0xf0: the pipe's write-channel status,
 *             write 1s to clear: bit 0 frame start, bit 15 "early" (the
 *             online VPSS prepares its next frame, VPSS_COMM_VI_EarlyIrqProc),
 *             bit 1 frame end (the frame is sent on)
 *             (VI_COMM_CapOnLineFrameStartIrqRoute / ...EarlyEndIrqRoute)
 *   VI_PROC (0x11200000):
 *     0x0310  status, write 1s to clear: bit 10 frame start, bit 1 frame
 *             end; VI_COMM_ProcIrqRoute panics on a zero status
 *     0x0318  cleared with 0x310 (VI_HAL_ClearAnyProcInt)
 *
 * Every other register, the ISP's included, is storage.  Capture
 * interrupts go out on the first line, the pipe's on the second; the
 * "frame" GPIO pulses once a frame, for VPSS to write it.
 *
 * Frames run while the "run" GPIO is high: hisi-vpss raises it while its
 * online frame interrupt is unmasked, which the driver does only once the
 * whole pipeline (VI pipe, VPSS group) exists.  Nothing in VI's own
 * registers says so early enough or late enough: its interrupt mask is
 * set in full when the module loads, and an interrupt for a pipe that is
 * not there (any more) dereferences a NULL context in the driver.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"

#define TYPE_HISI_VI "hisi-vi"
OBJECT_DECLARE_SIMPLE_TYPE(HisiViState, HISI_VI)

#define VI_CAP_SIZE         0x200000
#define VI_PROC_SIZE        0x40000

#define CAP_INT             0x00f0
#define CAP_INT_MASK        0x00f8
#define CAP_INT_PIPE0       (1u << 8)
#define CAP_WCH_INT(p)      (0x10000 + (p) * 0x1000 + 0xf0)
#define WCH_INT_FSTART      (1u << 0)
#define WCH_INT_FEND        (1u << 1)
#define WCH_INT_EARLY       (1u << 15)  /* lines in: VPSS prepares the next */

#define PROC_INT            0x0310
#define PROC_INT2           0x0318
#define PROC_INT_FEND       (1u << 1)
#define PROC_INT_FSTART     (1u << 10)

struct HisiViState {
    SysBusDevice parent_obj;
    MemoryRegion cap_iomem;
    MemoryRegion proc_iomem;
    qemu_irq cap_irq;
    qemu_irq proc_irq;
    qemu_irq frame_out;
    QEMUTimer *timer;
    uint32_t *cap;
    uint32_t *proc;
    uint32_t fps;
    bool run;
    bool trace;
};

/* the capture status: pipe 0's bit stands while its write channel has
 * anything pending, the driver clears only the latter */
static uint32_t vi_cap_status(HisiViState *s)
{
    return s->cap[CAP_INT / 4] |
           (s->cap[CAP_WCH_INT(0) / 4] ? CAP_INT_PIPE0 : 0);
}

static void vi_update(HisiViState *s)
{
    qemu_set_irq(s->cap_irq, (vi_cap_status(s) & s->cap[CAP_INT_MASK / 4] &
                              0xf00) != 0);
    qemu_set_irq(s->proc_irq, s->proc[PROC_INT / 4] != 0);
}

static void vi_arm(HisiViState *s)
{
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
}

static void vi_frame(void *opaque)
{
    HisiViState *s = opaque;

    if (!s->run) {
        return;                     /* the pipeline was stopped */
    }
    s->cap[CAP_WCH_INT(0) / 4] |= WCH_INT_FSTART | WCH_INT_EARLY | WCH_INT_FEND;
    s->proc[PROC_INT / 4] |= PROC_INT_FSTART | PROC_INT_FEND;
    vi_update(s);
    qemu_irq_pulse(s->frame_out);
    vi_arm(s);
}

static void vi_run(void *opaque, int n, int level)
{
    HisiViState *s = opaque;

    s->run = level;
    if (level && !timer_pending(s->timer)) {
        vi_arm(s);
    }
}

static uint64_t vi_cap_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiViState *s = HISI_VI(opaque);
    const uint32_t val = offset == CAP_INT ? vi_cap_status(s)
                                           : s->cap[offset / 4];

    if (s->trace) {
        fprintf(stderr, "vi-cap R %06x = %08x\n", (int)offset, val);
    }
    return val;
}

static void vi_cap_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    HisiViState *s = HISI_VI(opaque);

    if (s->trace) {
        fprintf(stderr, "vi-cap W %06x : %08x\n", (int)offset, (uint32_t)val);
    }
    if (offset == CAP_INT || offset == CAP_WCH_INT(0) ||
        offset == CAP_WCH_INT(1)) {
        s->cap[offset / 4] &= ~(uint32_t)val;
    } else {
        s->cap[offset / 4] = val;
    }
    vi_update(s);
}

static uint64_t vi_proc_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiViState *s = HISI_VI(opaque);

    if (s->trace && offset < 0x1000) {
        fprintf(stderr, "vi-proc R %05x = %08x\n", (int)offset,
                s->proc[offset / 4]);
    }
    return s->proc[offset / 4];
}

static void vi_proc_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    HisiViState *s = HISI_VI(opaque);

    if (s->trace && offset < 0x1000) {
        fprintf(stderr, "vi-proc W %05x : %08x\n", (int)offset,
                (uint32_t)val);
    }
    if (offset == PROC_INT || offset == PROC_INT2) {
        s->proc[offset / 4] &= ~(uint32_t)val;
    } else {
        s->proc[offset / 4] = val;
    }
    vi_update(s);
}

static const MemoryRegionOps vi_cap_ops = {
    .read = vi_cap_read,
    .write = vi_cap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps vi_proc_ops = {
    .read = vi_proc_read,
    .write = vi_proc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_vi_init(Object *obj)
{
    HisiViState *s = HISI_VI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->cap = g_new0(uint32_t, VI_CAP_SIZE / 4);
    s->proc = g_new0(uint32_t, VI_PROC_SIZE / 4);
    memory_region_init_io(&s->cap_iomem, obj, &vi_cap_ops, s, "hisi-vi-cap",
                          VI_CAP_SIZE);
    memory_region_init_io(&s->proc_iomem, obj, &vi_proc_ops, s,
                          "hisi-vi-proc", VI_PROC_SIZE);
    sysbus_init_mmio(sbd, &s->cap_iomem);
    sysbus_init_mmio(sbd, &s->proc_iomem);
    sysbus_init_irq(sbd, &s->cap_irq);
    sysbus_init_irq(sbd, &s->proc_irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->frame_out, "frame", 1);
    qdev_init_gpio_in_named(DEVICE(obj), vi_run, "run", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vi_frame, s);
}

static void hisi_vi_reset(DeviceState *dev)
{
    HisiViState *s = HISI_VI(dev);

    timer_del(s->timer);
    s->run = false;
    memset(s->cap, 0, VI_CAP_SIZE);
    memset(s->proc, 0, VI_PROC_SIZE);
    qemu_set_irq(s->cap_irq, 0);
    qemu_set_irq(s->proc_irq, 0);
}

static const Property hisi_vi_properties[] = {
    DEFINE_PROP_UINT32("fps", HisiViState, fps, 25),
    DEFINE_PROP_BOOL("trace", HisiViState, trace, false),
};

static void hisi_vi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, hisi_vi_properties);
    device_class_set_legacy_reset(dc, hisi_vi_reset);
}

static const TypeInfo hisi_vi_info = {
    .name          = TYPE_HISI_VI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiViState),
    .instance_init = hisi_vi_init,
    .class_init    = hisi_vi_class_init,
};

static void hisi_vi_register_types(void)
{
    type_register_static(&hisi_vi_info);
}

type_init(hisi_vi_register_types)
