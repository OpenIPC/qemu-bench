/*
 * Novatek NA51055 / NA51089 PWM controller at 0xF0210000 (SPI 26).
 *
 * The kernel's PWM-framework driver (drivers/pwm/pwm-na51055.c, sysfs)
 * binds "nvt,nvt_pwm", the vendor trees' node.  The HDAL kdrv_comm.ko the
 * OpenIPC release loads has a PWM driver too, but it binds only
 * "nvt,nvt_kdrv_pwm" and refuses "nvt,nvt_pwm".  Registers, from the
 * former and the HDAL's pwm_reg.h:
 *
 *   0x00 + 8n  CTRL n     [15:0] cycle count: 0 runs free, otherwise the
 *                         channel stops after that many periods
 *   0x04 + 8n  PERIOD n   [7:0] rise, [15:8] fall, [23:16] base period,
 *                         bit 28 invert
 *   0x230 + 4n            the high bytes of the same, channels 0..7
 *   0xE0/0xE4/0xE8/0xEC   interrupt enables of the four status words
 *   0xF0  INT_STS         bit n: channel n stopped (disabled, or its
 *                         cycles done); write 1 to clear
 *   0xF4 / 0xF8           micro-step and cycle-counter status (storage)
 *   0xFC  CLKDIV_LOAD_STS bit n: a clock-divider load done
 *   0x100 ENABLE          write 1 to start channel n; reads the running set
 *   0x104 DISABLE         write 1 to stop channel n (INT_STS n follows)
 *   0x108 LOAD            reload a running channel's PERIOD
 *   0x10C CLKDIV_LOAD     load a divider (CLKDIV_LOAD_STS follows)
 *
 * The drivers wait on the stop interrupt (nvt_pwm_disable(), and
 * nvt_pwm_enable() with a cycle count) and poll ENABLE after setting it.
 * A cycle-count run ends on a timer: base period ticks per cycle at a
 * nominal 30 MHz (the 120 MHz PWM source over the drivers' default
 * divider of 4); the clock generator's real divider is not consulted.
 * There is no output pin: each channel's state is visible as the "out"
 * GPIO line n, high while it runs with a nonzero duty.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-core.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_PWM "nvt-pwm"
OBJECT_DECLARE_SIMPLE_TYPE(NvtPwmState, NVT_PWM)

#define PWM_REGS_SIZE       0x400
#define PWM_CHANNELS        16
#define PWM_CTRL(n)         ((n) * 8)
#define PWM_PERIOD(n)       ((n) * 8 + 4)
#define PWM_EXT_PERIOD(n)   (0x230 + (n) * 4)
#define PWM_INT_EN0         0xE0
#define PWM_INT_STS         0xF0
#define PWM_CLKDIV_STS      0xFC
#define PWM_ENABLE          0x100
#define PWM_DISABLE         0x104
#define PWM_LOAD            0x108
#define PWM_CLKDIV_LOAD     0x10C

#define PWM_TICK_HZ         30000000

struct NvtPwmState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq out[PWM_CHANNELS];
    QEMUTimer *timer[PWM_CHANNELS];
    uint32_t regs[PWM_REGS_SIZE / 4];
    uint32_t running;
};

static uint32_t *reg(NvtPwmState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_pwm_update(NvtPwmState *s)
{
    int i, level = 0;

    for (i = 0; i < 4; i++) {
        if (*reg(s, PWM_INT_STS + 4 * i) & *reg(s, PWM_INT_EN0 + 4 * i)) {
            level = 1;
        }
    }
    qemu_set_irq(s->irq, level);
}

/* Base period and fall of channel n, with the high bytes of 0..7. */
static uint32_t nvt_pwm_field(NvtPwmState *s, int n, int shift)
{
    uint32_t v = (*reg(s, PWM_PERIOD(n)) >> shift) & 0xFF;

    if (n < 8) {
        v |= ((*reg(s, PWM_EXT_PERIOD(n)) >> shift) & 0xFF) << 8;
    }
    return v;
}

static void nvt_pwm_set_out(NvtPwmState *s, int n)
{
    bool on = (s->running & (1u << n)) && nvt_pwm_field(s, n, 8) != 0;

    qemu_set_irq(s->out[n], on);
}

static void nvt_pwm_stop(NvtPwmState *s, int n)
{
    timer_del(s->timer[n]);
    s->running &= ~(1u << n);
    *reg(s, PWM_INT_STS) |= 1u << n;
    nvt_pwm_set_out(s, n);
}

static void nvt_pwm_start(NvtPwmState *s, int n)
{
    uint32_t cycles = *reg(s, PWM_CTRL(n)) & 0xFFFF;
    uint64_t ticks;

    s->running |= 1u << n;
    nvt_pwm_set_out(s, n);
    if (cycles) {
        ticks = (uint64_t)cycles * MAX(nvt_pwm_field(s, n, 16), 1);
        timer_mod(s->timer[n], qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  muldiv64(ticks, NANOSECONDS_PER_SECOND, PWM_TICK_HZ));
    }
}

typedef struct {
    NvtPwmState *s;
    int n;
} NvtPwmChannel;

static void nvt_pwm_cycles_done(void *opaque)
{
    NvtPwmChannel *c = opaque;

    nvt_pwm_stop(c->s, c->n);
    nvt_pwm_update(c->s);
}

static uint64_t nvt_pwm_read(void *opaque, hwaddr off, unsigned size)
{
    NvtPwmState *s = opaque;

    switch (off) {
    case PWM_ENABLE:
        return s->running;
    case PWM_DISABLE:
    case PWM_LOAD:
    case PWM_CLKDIV_LOAD:
        return 0;
    default:
        return off < PWM_REGS_SIZE ? *reg(s, off) : 0;
    }
}

static void nvt_pwm_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    NvtPwmState *s = opaque;
    int n;

    switch (off) {
    case PWM_ENABLE:
        for (n = 0; n < PWM_CHANNELS; n++) {
            if ((val & (1u << n)) && !(s->running & (1u << n))) {
                nvt_pwm_start(s, n);
            }
        }
        break;
    case PWM_DISABLE:
        for (n = 0; n < PWM_CHANNELS; n++) {
            if ((val & (1u << n)) && (s->running & (1u << n))) {
                nvt_pwm_stop(s, n);
            }
        }
        break;
    case PWM_LOAD:
        for (n = 0; n < PWM_CHANNELS; n++) {
            if (val & (1u << n)) {
                nvt_pwm_set_out(s, n);
            }
        }
        break;
    case PWM_CLKDIV_LOAD:
        *reg(s, PWM_CLKDIV_STS) |= val;
        break;
    case PWM_INT_STS:
    case PWM_CLKDIV_STS:
        *reg(s, off) &= ~val;
        break;
    default:
        if (off < PWM_REGS_SIZE) {
            *reg(s, off) = val;
        }
        break;
    }
    nvt_pwm_update(s);
}

static const MemoryRegionOps nvt_pwm_ops = {
    .read = nvt_pwm_read,
    .write = nvt_pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_pwm_reset(DeviceState *dev)
{
    NvtPwmState *s = NVT_PWM(dev);
    int n;

    for (n = 0; n < PWM_CHANNELS; n++) {
        timer_del(s->timer[n]);
    }
    memset(s->regs, 0, sizeof(s->regs));
    s->running = 0;
    for (n = 0; n < PWM_CHANNELS; n++) {
        qemu_irq_lower(s->out[n]);
    }
    qemu_irq_lower(s->irq);
}

static void nvt_pwm_init(Object *obj)
{
    NvtPwmState *s = NVT_PWM(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &nvt_pwm_ops, s, TYPE_NVT_PWM,
                          0x2000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), s->out, "out", PWM_CHANNELS);
    for (n = 0; n < PWM_CHANNELS; n++) {
        NvtPwmChannel *c = g_new(NvtPwmChannel, 1);

        c->s = s;
        c->n = n;
        s->timer[n] = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_pwm_cycles_done, c);
    }
}

/* The out lines are derived state: drive them again from what arrived. */
static int nvt_pwm_post_load(void *opaque, int version_id)
{
    NvtPwmState *s = opaque;
    int n;

    for (n = 0; n < PWM_CHANNELS; n++) {
        nvt_pwm_set_out(s, n);
    }
    nvt_pwm_update(s);
    return 0;
}

static const VMStateDescription vmstate_nvt_pwm = {
    .name = TYPE_NVT_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = nvt_pwm_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtPwmState, PWM_REGS_SIZE / 4),
        VMSTATE_UINT32(running, NvtPwmState),
        VMSTATE_TIMER_PTR_ARRAY(timer, NvtPwmState, PWM_CHANNELS),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_pwm_reset);
    dc->vmsd = &vmstate_nvt_pwm;
}

static const TypeInfo nvt_pwm_info = {
    .name = TYPE_NVT_PWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtPwmState),
    .instance_init = nvt_pwm_init,
    .class_init = nvt_pwm_class_init,
};

static void nvt_pwm_register_types(void)
{
    type_register_static(&nvt_pwm_info);
}

type_init(nvt_pwm_register_types)
