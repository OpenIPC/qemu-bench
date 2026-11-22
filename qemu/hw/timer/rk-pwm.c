/*
 * Rockchip PWM v2 ("rockchip,rk3328-pwm" and the RV1106 / RV1126 ones
 * after it): four channels, as the kernel's drivers/pwm/pwm-rockchip.c
 * (pwm_data_v2) drives them.
 *
 *   0x10 n + 0x0 CNT          the channel's counter (read only)
 *   0x10 n + 0x4 PERIOD_HPR   period, in PWM clocks
 *   0x10 n + 0x8 DUTY_LPR     duty (active) time
 *   0x10 n + 0xc CTRL         bit 0 enable, 2:1 mode (0 one-shot, 1
 *                             continuous, 2 capture), 3 / 4 duty / inactive
 *                             polarity, 5 centre aligned, 6 lock, 9 clock
 *                             select (scaled), 14:12 prescale (2^n), 23:16
 *                             scale (2 x, 0 = 512), 31:24 one-shot repeats
 *   0x40 INTSTS  bit n channel n's one-shot done (write 1 to clear)
 *   0x44 INT_EN
 *
 * With registers that read back, the driver's state matches what it set:
 * read as zeros, every channel looks disabled, and the vendor SDK
 * re-applies its PWMs over and over.  The counter runs from the PWM clock
 * ("clock-hz", 24 MHz on these SoCs: xin24m), divided as CTRL says, from
 * 0 up to the period and over again; a one-shot channel stops after
 * repeats + 1 periods, clears its enable and raises its interrupt.
 * Capture mode counts nothing.
 *
 * The output is not a waveform; its level is the QOM property "outputN"
 * (read only): how much of the time channel N's pin is high, in permille.
 * A running channel is high for duty / period with a positive duty
 * polarity (CTRL bit 3), for the rest of the period with a negative one.
 * A stopped one sits at its inactive level (bit 4): 1000 high, 0 low.
 * An IR LED's or a backlight's brightness reads off it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "qapi/visitor.h"

#define TYPE_RK_PWM "rk-pwm"
OBJECT_DECLARE_SIMPLE_TYPE(RkPwmState, RK_PWM)

#define REGS_SIZE       0x100
#define NCH             4
#define R_CNT           0x0
#define R_PERIOD        0x4
#define R_DUTY          0x8
#define R_CTRL          0xc
#define R_INTSTS        0x40
#define R_INT_EN        0x44

#define CTRL_EN         (1u << 0)
#define CTRL_MODE(c)    (((c) >> 1) & 3)
#define MODE_ONESHOT    0
#define MODE_CONT       1

struct RkPwmState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer[NCH];

    uint32_t regs[REGS_SIZE / 4];
    int64_t start_ns[NCH];      /* when each counter started from 0 */
    uint32_t clock_hz;
};

#define REG(s, off) ((s)->regs[(off) / 4])
#define CH(n, r)    (0x10 * (n) + (r))

static void rk_pwm_update(RkPwmState *s)
{
    qemu_set_irq(s->irq, !!(REG(s, R_INTSTS) & REG(s, R_INT_EN) & 0xf));
}

/* Nanoseconds a PWM clock lasts on channel @n */
static double rk_pwm_tick_ns(RkPwmState *s, int n)
{
    uint32_t c = REG(s, CH(n, R_CTRL));
    double div = 1u << ((c >> 12) & 7);

    if (c & (1u << 9)) {
        uint32_t scale = (c >> 16) & 0xff;

        div *= 2 * (scale ? scale : 256);
    }
    return 1e9 * div / MAX(s->clock_hz, 1);
}

static uint64_t rk_pwm_period_ticks(RkPwmState *s, int n)
{
    return (uint64_t)REG(s, CH(n, R_PERIOD)) + 1;
}

static void rk_pwm_arm(RkPwmState *s, int n)
{
    uint32_t c = REG(s, CH(n, R_CTRL));

    timer_del(s->timer[n]);
    if ((c & CTRL_EN) && CTRL_MODE(c) == MODE_ONESHOT) {
        uint64_t reps = ((c >> 24) & 0xff) + 1;

        timer_mod(s->timer[n], s->start_ns[n] +
                  (int64_t)(rk_pwm_tick_ns(s, n) *
                            rk_pwm_period_ticks(s, n) * reps));
    }
}

static void rk_pwm_oneshot_done(RkPwmState *s, int n)
{
    REG(s, CH(n, R_CTRL)) &= ~CTRL_EN;
    REG(s, R_INTSTS) |= 1u << n;
    rk_pwm_update(s);
}

static void rk_pwm_timer0(void *o) { rk_pwm_oneshot_done(o, 0); }
static void rk_pwm_timer1(void *o) { rk_pwm_oneshot_done(o, 1); }
static void rk_pwm_timer2(void *o) { rk_pwm_oneshot_done(o, 2); }
static void rk_pwm_timer3(void *o) { rk_pwm_oneshot_done(o, 3); }

static uint32_t rk_pwm_cnt(RkPwmState *s, int n)
{
    uint32_t c = REG(s, CH(n, R_CTRL));
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t ticks;

    if (!(c & CTRL_EN) || CTRL_MODE(c) > MODE_CONT) {
        return 0;
    }
    ticks = (now - s->start_ns[n]) / rk_pwm_tick_ns(s, n);
    return ticks % rk_pwm_period_ticks(s, n);
}

/* permille of the time channel @n's output is high */
static uint32_t rk_pwm_output(RkPwmState *s, int n)
{
    uint32_t c = REG(s, CH(n, R_CTRL));
    uint64_t period = rk_pwm_period_ticks(s, n);
    uint64_t duty = MIN((uint64_t)REG(s, CH(n, R_DUTY)), period);
    uint32_t active;

    if (!(c & CTRL_EN) || CTRL_MODE(c) > MODE_CONT) {
        return (c & (1u << 4)) ? 1000 : 0;
    }
    active = duty * 1000 / period;
    return (c & (1u << 3)) ? active : 1000 - active;
}

static void rk_pwm_get_output(Object *obj, Visitor *v, const char *name,
                              void *opaque, Error **errp)
{
    uint32_t val = rk_pwm_output(RK_PWM(obj), (uintptr_t)opaque);

    visit_type_uint32(v, name, &val, errp);
}

static uint64_t rk_pwm_read(void *opaque, hwaddr off, unsigned size)
{
    RkPwmState *s = opaque;

    if (off < 0x10 * NCH && (off & 0xf) == R_CNT) {
        return rk_pwm_cnt(s, off / 0x10);
    }
    return s->regs[off / 4];
}

static void rk_pwm_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    RkPwmState *s = opaque;

    if (off < 0x10 * NCH) {
        int n = off / 0x10;
        uint32_t was = REG(s, CH(n, R_CTRL));

        if ((off & 0xf) == R_CNT) {
            return;
        }
        s->regs[off / 4] = val;
        if ((off & 0xf) == R_CTRL && (val & CTRL_EN) && !(was & CTRL_EN)) {
            s->start_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        rk_pwm_arm(s, n);
        return;
    }
    switch (off) {
    case R_INTSTS:
        REG(s, R_INTSTS) &= ~val;
        break;
    default:
        s->regs[off / 4] = val;
    }
    rk_pwm_update(s);
}

static const MemoryRegionOps rk_pwm_ops = {
    .read = rk_pwm_read,
    .write = rk_pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_pwm_reset(DeviceState *dev)
{
    RkPwmState *s = RK_PWM(dev);
    int n;

    memset(s->regs, 0, sizeof(s->regs));
    for (n = 0; n < NCH; n++) {
        timer_del(s->timer[n]);
        s->start_ns[n] = 0;
    }
    rk_pwm_update(s);
}

static void rk_pwm_init(Object *obj)
{
    static QEMUTimerCB *const cbs[NCH] = {
        rk_pwm_timer0, rk_pwm_timer1, rk_pwm_timer2, rk_pwm_timer3,
    };
    RkPwmState *s = RK_PWM(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &rk_pwm_ops, s, TYPE_RK_PWM,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    for (n = 0; n < NCH; n++) {
        g_autofree char *name = g_strdup_printf("output%d", n);

        s->timer[n] = timer_new_ns(QEMU_CLOCK_VIRTUAL, cbs[n], s);
        object_property_add(obj, name, "uint32", rk_pwm_get_output, NULL,
                            NULL, (void *)(uintptr_t)n);
    }
}

static const Property rk_pwm_properties[] = {
    DEFINE_PROP_UINT32("clock-hz", RkPwmState, clock_hz, 24000000),
};

static const VMStateDescription rk_pwm_vmstate = {
    .name = TYPE_RK_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkPwmState, REGS_SIZE / 4),
        VMSTATE_INT64_ARRAY(start_ns, RkPwmState, NCH),
        VMSTATE_TIMER_PTR_ARRAY(timer, RkPwmState, NCH),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_pwm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_pwm_reset);
    device_class_set_props(dc, rk_pwm_properties);
    dc->vmsd = &rk_pwm_vmstate;
}

static const TypeInfo rk_pwm_info = {
    .name = TYPE_RK_PWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkPwmState),
    .instance_init = rk_pwm_init,
    .class_init = rk_pwm_class_init,
};

static void rk_pwm_register_types(void)
{
    type_register_static(&rk_pwm_info);
}

type_init(rk_pwm_register_types)
