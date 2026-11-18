/*
 * Grain Media FTPWM010 PWM timers.
 *
 * Eight PWM outputs at 0x91B00000 on the GM813x (module/pwm/ftpwmtmr010.c),
 * which drive IR-cut filters, LEDs and the like:
 *
 *   0x00 INT_CSTAT   per-timer interrupt status, write 1 clears
 *   0x04 START_CTRL  bit n runs timer n
 *   0x10 + 0x10n     CTRL (0 external clock, 1 update: load CNTB / CMPB,
 *                    2 inverter, 4:3 mode, 8 PWM enable), CNTB, CMPB, CNTO
 *   0x90 TMR_REV, 0xA0 + 4n REP, 0xC0 + 0x10n PAT, 0x140 + 4n PATLEN
 *   0x160 INT_PRIO   the low nibble says which timer interrupted, plus 1
 *
 * The outputs go nowhere, so the timers need not count: CNTO reads the
 * loaded period, the update bit clears at once, and interrupt 49 stays
 * low -- the driver's handler indexes its table by INT_PRIO - 1, and
 * INT_PRIO stays 0.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_PWM "gm-pwm"
OBJECT_DECLARE_SIMPLE_TYPE(GmPwmState, GM_PWM)

#define GM_PWM_SIZE         0x1000
#define GM_PWM_NREGS        (0x180 / 4)
#define GM_PWM_NUM          8

#define R_INT_CSTAT         0x00
#define R_TMR_REV           0x90
#define R_INT_PRIO          0x160
#define R_CTRL(n)           (0x10 + 0x10 * (n))
#define R_CNTB(n)           (0x14 + 0x10 * (n))
#define R_CNTO(n)           (0x1C + 0x10 * (n))

#define CTRL_UPDATE         (1u << 1)

struct GmPwmState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[GM_PWM_NREGS];
};

static uint64_t gm_pwm_read(void *opaque, hwaddr off, unsigned size)
{
    GmPwmState *s = opaque;

    if (off == R_TMR_REV) {
        return 0x00010000;
    }
    return off < GM_PWM_NREGS * 4 ? s->regs[off / 4] : 0;
}

static void gm_pwm_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    GmPwmState *s = opaque;
    unsigned n;

    if (off >= GM_PWM_NREGS * 4 || off == R_TMR_REV || off == R_INT_PRIO) {
        return;
    }
    if (off == R_INT_CSTAT) {
        s->regs[off / 4] &= ~val;
        return;
    }
    s->regs[off / 4] = val;
    for (n = 0; n < GM_PWM_NUM; n++) {
        if (off == R_CTRL(n) && (val & CTRL_UPDATE)) {
            s->regs[off / 4] &= ~CTRL_UPDATE;
            s->regs[R_CNTO(n) / 4] = s->regs[R_CNTB(n) / 4];
        }
    }
}

static const MemoryRegionOps gm_pwm_ops = {
    .read = gm_pwm_read,
    .write = gm_pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_pwm_reset(DeviceState *dev)
{
    GmPwmState *s = GM_PWM(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void gm_pwm_init(Object *obj)
{
    GmPwmState *s = GM_PWM(obj);

    memory_region_init_io(&s->iomem, obj, &gm_pwm_ops, s, TYPE_GM_PWM,
                          GM_PWM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_gm_pwm = {
    .name = TYPE_GM_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmPwmState, GM_PWM_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_pwm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, gm_pwm_reset);
    dc->vmsd = &vmstate_gm_pwm;
}

static const TypeInfo gm_pwm_info = {
    .name           = TYPE_GM_PWM,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmPwmState),
    .instance_init  = gm_pwm_init,
    .class_init     = gm_pwm_class_init,
};

static void gm_pwm_register_types(void)
{
    type_register_static(&gm_pwm_info);
}

type_init(gm_pwm_register_types)
