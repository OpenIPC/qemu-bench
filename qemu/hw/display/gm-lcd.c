/*
 * Grain Media LCD200 display controller, as the video graph's clock.
 *
 * Faraday's LCD200 at 0x9BF00000 on the GM813x (module/LCD200_v3: dev.h,
 * dev.c, pip.c and lcd_vg.c of the SDK), here driving the TVE100's D1
 * output.  The registers the driver gives meaning to:
 *
 *   0x00 FEATURE    bit 0 enables the LCD
 *   0x04 PANEL      bits 10:9 when the frame interrupt fires
 *   0x08 INT_MASK   0x0C INT_CLEAR (write 1 to clear)
 *   0x10 INT_STATUS bit 0 underrun, 1 next base, 2 vertical, 3 bus error
 *   0x18 / 0x24 / 0x30 frame buffer bases, 0x100... timing, 0x200...
 *        CCIR656, 0x300... the PIP windows
 *
 * The vendor driver queues display jobs and moves them on only from its
 * interrupt handler on a vertical interrupt, which also advances the
 * video graph's reference clock (video_process_tick): without it the
 * graph's first display job times out and its scheduler stops.  So while
 * the LCD is enabled a frame timer raises the vertical status at the
 * output's rate (NTSC, 30000 / 1001 Hz), on the interrupt line while it is
 * unmasked.  Nothing is scanned out.  Everything else is storage.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_LCD "gm-lcd"
OBJECT_DECLARE_SIMPLE_TYPE(GmLcdState, GM_LCD)

#define GM_LCD_SIZE         0xD000
#define GM_LCD_NREGS        (0x1000 / 4)

#define R_FEATURE           0x00
#define R_INT_MASK          0x08
#define R_INT_CLEAR         0x0C
#define R_INT_STATUS        0x10

#define FEATURE_EN          (1u << 0)
#define INT_VSTATUS         (1u << 2)

struct GmLcdState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[GM_LCD_NREGS];
    uint32_t frame_ns;
};

static void gm_lcd_update(GmLcdState *s)
{
    /* INT_MASK set bits enable: the driver writes 0x4 for the frame */
    qemu_set_irq(s->irq, (s->regs[R_INT_STATUS / 4] &
                          s->regs[R_INT_MASK / 4] & 0xF) != 0);
}

static void gm_lcd_arm(GmLcdState *s)
{
    if (s->regs[R_FEATURE / 4] & FEATURE_EN) {
        if (!timer_pending(s->timer)) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                s->frame_ns);
        }
    } else {
        timer_del(s->timer);
    }
}

static void gm_lcd_frame(void *opaque)
{
    GmLcdState *s = opaque;

    s->regs[R_INT_STATUS / 4] |= INT_VSTATUS;
    gm_lcd_update(s);
    gm_lcd_arm(s);
}

static uint64_t gm_lcd_read(void *opaque, hwaddr off, unsigned size)
{
    GmLcdState *s = opaque;

    return off < GM_LCD_NREGS * 4 ? s->regs[off / 4] : 0;
}

static void gm_lcd_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    GmLcdState *s = opaque;

    if (off >= GM_LCD_NREGS * 4) {
        return;
    }
    switch (off) {
    case R_INT_CLEAR:
        s->regs[R_INT_STATUS / 4] &= ~val;
        break;
    case R_INT_STATUS:
        break;
    default:
        s->regs[off / 4] = val;
        break;
    }
    gm_lcd_arm(s);
    gm_lcd_update(s);
}

static const MemoryRegionOps gm_lcd_ops = {
    .read = gm_lcd_read,
    .write = gm_lcd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_lcd_reset(DeviceState *dev)
{
    GmLcdState *s = GM_LCD(dev);

    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->timer);
    gm_lcd_update(s);
}

static void gm_lcd_realize(DeviceState *dev, Error **errp)
{
    GmLcdState *s = GM_LCD(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_lcd_ops, s,
                          TYPE_GM_LCD, GM_LCD_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_lcd_frame, s);
}

static const Property gm_lcd_props[] = {
    /* NTSC: 1001 / 30000 s */
    DEFINE_PROP_UINT32("frame-ns", GmLcdState, frame_ns, 33366667),
};

static const VMStateDescription vmstate_gm_lcd = {
    .name = TYPE_GM_LCD,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, GmLcdState),
        VMSTATE_UINT32_ARRAY(regs, GmLcdState, GM_LCD_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_lcd_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_lcd_realize;
    device_class_set_legacy_reset(dc, gm_lcd_reset);
    device_class_set_props(dc, gm_lcd_props);
    dc->vmsd = &vmstate_gm_lcd;
}

static const TypeInfo gm_lcd_info = {
    .name           = TYPE_GM_LCD,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmLcdState),
    .class_init     = gm_lcd_class_init,
};

static void gm_lcd_register_types(void)
{
    type_register_static(&gm_lcd_info);
}

type_init(gm_lcd_register_types)
