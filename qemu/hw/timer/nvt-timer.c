/*
 * Novatek NA51055 / NA51089 general-purpose timers at 0xF0040000 (SPI 0).
 *
 * The HDAL's kdrv_comm.ko drives them ("nvt,nvt_timer"; drv_sys/
 * timer_reg.h, and its closed timer core, as disassembled), and U-Boot and
 * the loader read timer 0 as a microsecond clock (get_nvt_timer0_cnt(),
 * timer2_delay(): COUNTER 0 at 0x108).  Twenty timers:
 *
 *   0x00..0x0C DST n   which core each timer interrupts (storage)
 *   0x10 STS           bit n: timer n reached its target; write 1 to clear
 *   0x14..0x1C         the other cores' status words (storage)
 *   0x20 INT           bit n: timer n interrupts this core (SPI 0)
 *   0x30 CLKDIV        [7:0] / [15:8] dividers 0 / 1, bits 16 / 17 their
 *                      enables (storage); divider 0 is fixed at 1 MHz, and
 *                      divider 1 runs at 3 MHz / ([15:8] + 1) (timer.h)
 *   0x100 + 0x10n  CTRL     bit 0 enable, bit 1 free run (else one shot),
 *                           bit 2 clock from divider 1 (else divider 0)
 *   0x104 + 0x10n  TARGET   the count that ends a period
 *   0x108 + 0x10n  COUNTER  counts up from 0 while enabled
 *   0x10C + 0x10n  RELOAD   bit 0 starts a new period with TARGET
 *
 * A timer counts 0..TARGET and then flags STS: a one-shot timer stops
 * (CTRL enable reads 0), a free-running one wraps to 0 and goes on.  The
 * drivers poll CTRL after enabling or disabling (timer_cfg()), so both take
 * effect at once.  Timer 0 comes out of reset running free at 1 MHz with
 * the full 32-bit target, as the boot loader leaves it: the vendor's
 * "boot time" prints and timer_open_sys_timer() rely on it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_TIMER "nvt-timer"
OBJECT_DECLARE_SIMPLE_TYPE(NvtTimerState, NVT_TIMER)

#define TMR_REGS_SIZE   0x300
#define TMR_NUM         20
#define TMR_STS         0x10
#define TMR_INT         0x20
#define TMR_CLKDIV      0x30
#define TMR_BASE(n)     (0x100 + (n) * 0x10)
#define TMR_CTRL        0x0
#define TMR_TARGET      0x4
#define TMR_COUNTER     0x8
#define TMR_RELOAD      0xC

#define CTRL_EN         (1u << 0)
#define CTRL_FREE_RUN   (1u << 1)
#define CTRL_DIV1       (1u << 2)

typedef struct NvtTimerState NvtTimerState;

typedef struct {
    NvtTimerState *s;
    int n;
    QEMUTimer *qt;
    int64_t start_ns;       /* when the current period's count was 0 */
    uint32_t target;        /* the period's target, from TARGET */
} NvtTimerChannel;

struct NvtTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[TMR_REGS_SIZE / 4];
    NvtTimerChannel ch[TMR_NUM];
};

static uint32_t *reg(NvtTimerState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static uint32_t *treg(NvtTimerState *s, int n, hwaddr off)
{
    return reg(s, TMR_BASE(n) + off);
}

static void nvt_timer_update_irq(NvtTimerState *s)
{
    qemu_set_irq(s->irq, !!(*reg(s, TMR_STS) & *reg(s, TMR_INT)));
}

static uint64_t nvt_timer_hz(NvtTimerState *s, int n)
{
    if (*treg(s, n, TMR_CTRL) & CTRL_DIV1) {
        return 3000000 / (((*reg(s, TMR_CLKDIV) >> 8) & 0xFF) + 1);
    }
    return 1000000;
}

/* Ticks counted in the current period, uncapped. */
static uint64_t nvt_timer_elapsed(NvtTimerChannel *c)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - c->start_ns;

    return muldiv64(MAX(ns, 0), nvt_timer_hz(c->s, c->n),
                    NANOSECONDS_PER_SECOND);
}

/* Arm the QEMU timer for the end of the current period. */
static void nvt_timer_schedule(NvtTimerChannel *c)
{
    uint64_t ticks = (uint64_t)c->target + 1;

    timer_mod(c->qt, c->start_ns + muldiv64(ticks, NANOSECONDS_PER_SECOND,
                                            nvt_timer_hz(c->s, c->n)));
}

static void nvt_timer_start(NvtTimerChannel *c)
{
    c->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    c->target = *treg(c->s, c->n, TMR_TARGET);
    nvt_timer_schedule(c);
}

/*
 * The clock of a running timer changed (CLKDIV, or CTRL's divider select):
 * it goes on from the count it had reached, at the new rate.
 */
static void nvt_timer_rebase(NvtTimerChannel *c, uint64_t count)
{
    c->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                  muldiv64(count, NANOSECONDS_PER_SECOND,
                           nvt_timer_hz(c->s, c->n));
    nvt_timer_schedule(c);
}

static void nvt_timer_expired(void *opaque)
{
    NvtTimerChannel *c = opaque;
    NvtTimerState *s = c->s;
    uint32_t *ctrl = treg(s, c->n, TMR_CTRL);

    *reg(s, TMR_STS) |= 1u << c->n;
    if (*ctrl & CTRL_FREE_RUN) {
        c->start_ns += muldiv64((uint64_t)c->target + 1,
                                NANOSECONDS_PER_SECOND,
                                nvt_timer_hz(s, c->n));
        nvt_timer_schedule(c);
    } else {
        *ctrl &= ~CTRL_EN;
        *treg(s, c->n, TMR_COUNTER) = c->target;
    }
    nvt_timer_update_irq(s);
}

static uint32_t nvt_timer_counter(NvtTimerState *s, int n)
{
    NvtTimerChannel *c = &s->ch[n];

    if (!(*treg(s, n, TMR_CTRL) & CTRL_EN)) {
        return *treg(s, n, TMR_COUNTER);
    }
    return MIN(nvt_timer_elapsed(c), (uint64_t)c->target);
}

static uint64_t nvt_timer_read(void *opaque, hwaddr off, unsigned size)
{
    NvtTimerState *s = opaque;
    int n;

    if (off >= TMR_BASE(0) && off < TMR_BASE(TMR_NUM)) {
        n = (off - TMR_BASE(0)) / 0x10;
        switch (off & 0xF) {
        case TMR_COUNTER:
            return nvt_timer_counter(s, n);
        case TMR_RELOAD:
            return 0;
        }
    }
    return off < TMR_REGS_SIZE ? *reg(s, off) : 0;
}

static void nvt_timer_write_ctrl(NvtTimerState *s, int n, uint32_t val)
{
    NvtTimerChannel *c = &s->ch[n];
    uint32_t old = *treg(s, n, TMR_CTRL);

    uint32_t count = nvt_timer_counter(s, n);

    if ((old & CTRL_EN) && !(val & CTRL_EN)) {
        /* Stopped: the count holds where it was. */
        *treg(s, n, TMR_COUNTER) = count;
        timer_del(c->qt);
    }
    *treg(s, n, TMR_CTRL) = val & 0x7;
    if (!(old & CTRL_EN) && (val & CTRL_EN)) {
        nvt_timer_start(c);
    } else if ((old & val & CTRL_EN) && ((old ^ val) & CTRL_DIV1)) {
        nvt_timer_rebase(c, count);
    }
}

static void nvt_timer_write_clkdiv(NvtTimerState *s, uint32_t val)
{
    uint32_t count[TMR_NUM];
    int n;

    for (n = 0; n < TMR_NUM; n++) {
        count[n] = nvt_timer_counter(s, n);
    }
    *reg(s, TMR_CLKDIV) = val;
    for (n = 0; n < TMR_NUM; n++) {
        if ((*treg(s, n, TMR_CTRL) & (CTRL_EN | CTRL_DIV1)) ==
            (CTRL_EN | CTRL_DIV1)) {
            nvt_timer_rebase(&s->ch[n], count[n]);
        }
    }
}

static void nvt_timer_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    NvtTimerState *s = opaque;
    int n;

    if (off >= TMR_BASE(0) && off < TMR_BASE(TMR_NUM)) {
        n = (off - TMR_BASE(0)) / 0x10;
        switch (off & 0xF) {
        case TMR_CTRL:
            nvt_timer_write_ctrl(s, n, val);
            return;
        case TMR_COUNTER:
            return;                     /* read only */
        case TMR_RELOAD:
            if ((val & 1) && (*treg(s, n, TMR_CTRL) & CTRL_EN)) {
                nvt_timer_start(&s->ch[n]);
            }
            return;
        }
    }
    switch (off) {
    case TMR_STS:
        *reg(s, off) &= ~val;
        nvt_timer_update_irq(s);
        break;
    case TMR_INT:
        *reg(s, off) = val;
        nvt_timer_update_irq(s);
        break;
    case TMR_CLKDIV:
        nvt_timer_write_clkdiv(s, val);
        break;
    default:
        if (off < TMR_REGS_SIZE) {
            *reg(s, off) = val;
        }
        break;
    }
}

static const MemoryRegionOps nvt_timer_ops = {
    .read = nvt_timer_read,
    .write = nvt_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_timer_reset(DeviceState *dev)
{
    NvtTimerState *s = NVT_TIMER(dev);
    int n;

    for (n = 0; n < TMR_NUM; n++) {
        timer_del(s->ch[n].qt);
    }
    memset(s->regs, 0, sizeof(s->regs));
    /* Divider 1 at 3 MHz / 3 = 1 MHz, its documented default. */
    *reg(s, TMR_CLKDIV) = 2 << 8;
    /* The system timer the loader leaves running. */
    *treg(s, 0, TMR_TARGET) = UINT32_MAX;
    *treg(s, 0, TMR_CTRL) = CTRL_EN | CTRL_FREE_RUN;
    nvt_timer_start(&s->ch[0]);
    qemu_irq_lower(s->irq);
}

static void nvt_timer_init(Object *obj)
{
    NvtTimerState *s = NVT_TIMER(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &nvt_timer_ops, s, TYPE_NVT_TIMER,
                          TMR_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    for (n = 0; n < TMR_NUM; n++) {
        s->ch[n].s = s;
        s->ch[n].n = n;
        s->ch[n].qt = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_timer_expired,
                                   &s->ch[n]);
    }
}

static const VMStateDescription vmstate_nvt_timer_channel = {
    .name = "nvt-timer-channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(qt, NvtTimerChannel),
        VMSTATE_INT64(start_ns, NvtTimerChannel),
        VMSTATE_UINT32(target, NvtTimerChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_nvt_timer = {
    .name = TYPE_NVT_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtTimerState, TMR_REGS_SIZE / 4),
        VMSTATE_STRUCT_ARRAY(ch, NvtTimerState, TMR_NUM, 1,
                             vmstate_nvt_timer_channel, NvtTimerChannel),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_timer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_timer_reset);
    dc->vmsd = &vmstate_nvt_timer;
}

static const TypeInfo nvt_timer_info = {
    .name = TYPE_NVT_TIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtTimerState),
    .instance_init = nvt_timer_init,
    .class_init = nvt_timer_class_init,
};

static void nvt_timer_register_types(void)
{
    type_register_static(&nvt_timer_info);
}

type_init(nvt_timer_register_types)
