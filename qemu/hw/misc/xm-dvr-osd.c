/*
 * Xiongmai DVR OSD engine (XM8521DV200 / XM8536E xm_rgn.ko: 0x34000000,
 * 4 KiB, IRQ 89): blends region bitmaps into frames for the encoders.
 *
 * xm_rgn.ko's osd_process: +0x408 0 then 1 resets, +0xF8 1 enables the
 * interrupt, 0 to +0xF4 clears the done flags; region n at +0x100 * n
 * (+0x00 enable, colours at +0x10.., size +0x2C/+0x30, bitmap and frame
 * addresses +0x40..+0x54); +0x064 / +0x068 / +0x060 1 start the job.  It
 * then sleeps until +0xF0 reads non-zero (done), osd_int seeing +0xFC.  Without completions every OSD update times out ("osd:
 * timeout"), which stalls Sofia's start-up long enough that its main
 * thread applies the motion-detect config before the capture thread has
 * created the detectors, and Sofia crashes on the NULL handle.
 *
 * A job completes 100 us after its start, and nothing is drawn yet.  Every word is
 * RAM-backed.  Read from the vendor module (no public source).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qom/object.h"

#define TYPE_XM_DVR_OSD "xm-dvr-osd"
OBJECT_DECLARE_SIMPLE_TYPE(XmDvrOsdState, XM_DVR_OSD)

#define OSD_SIZE        0x1000
#define OSD_ACK         0xF4
#define OSD_STATUS      0xFC
#define OSD_DONE        0xF0
#define OSD_START       0x060
#define OSD_JOB_NS      (100 * SCALE_US)

struct XmDvrOsdState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[OSD_SIZE / 4];
};

static void xm_dvr_osd_done(void *opaque)
{
    XmDvrOsdState *s = opaque;

    s->regs[OSD_DONE / 4] = 1;
    s->regs[OSD_STATUS / 4] = 1;
    if (s->regs[0xF8 / 4]) {
        qemu_irq_raise(s->irq);
    }
}

static uint64_t xm_dvr_osd_read(void *opaque, hwaddr off, unsigned size)
{
    XmDvrOsdState *s = opaque;

    return s->regs[off / 4];
}

static void xm_dvr_osd_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    XmDvrOsdState *s = opaque;

    if (off == OSD_ACK) {
        /* the interrupt only: the waiter still has to see +0xF0 */
        s->regs[OSD_STATUS / 4] = 0;
        qemu_irq_lower(s->irq);
        return;
    }
    s->regs[off / 4] = val;
    if (off == OSD_START && (val & 1)) {
        s->regs[OSD_DONE / 4] = 0;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + OSD_JOB_NS);
    }
}

static const MemoryRegionOps xm_dvr_osd_ops = {
    .read = xm_dvr_osd_read,
    .write = xm_dvr_osd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void xm_dvr_osd_reset(DeviceState *dev)
{
    XmDvrOsdState *s = XM_DVR_OSD(dev);

    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->timer);
    qemu_irq_lower(s->irq);
}

static void xm_dvr_osd_init(Object *obj)
{
    XmDvrOsdState *s = XM_DVR_OSD(obj);

    memory_region_init_io(&s->iomem, obj, &xm_dvr_osd_ops, s,
                          TYPE_XM_DVR_OSD, OSD_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_dvr_osd_done, s);
}

static void xm_dvr_osd_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_dvr_osd_reset);
}

static const TypeInfo xm_dvr_osd_info = {
    .name = TYPE_XM_DVR_OSD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmDvrOsdState),
    .instance_init = xm_dvr_osd_init,
    .class_init = xm_dvr_osd_class_init,
};

static void xm_dvr_osd_register(void)
{
    type_register_static(&xm_dvr_osd_info);
}

type_init(xm_dvr_osd_register)
