/*
 * HiSilicon PWM controller (the "PWM" / "PWM1" block of every IPC SoC).
 *
 * Every generation shares one programming model, in one of three layouts:
 *
 *   classic  channel n at n * 0x20:
 *              +0x00 CFG0  period, in clock ticks
 *              +0x04 CFG1  high time, in the same ticks
 *              +0x08 CFG2  periods to emit (a bounded run)
 *              +0x0c CTRL  bit0 enable, bit1 invert, bit2 keep (run forever)
 *              +0x10 ST0   period the output is running   (read-only)
 *              +0x14 ST1   high time it is running         (read-only)
 *              +0x18 ST2   periods still to emit, in [31:12] (read-only)
 *   split    hi3516av100: classic, but channels 4-7 sit in a second bank at
 *            +0x1000 (openhisilicon kernel/pwm/hi3516av100/pwm.c).
 *   v5       hi3516cv6xx: channel n at n * 0x100, and period and duty are
 *            written MINUS ONE in a 20-bit field (the kernel's pwm-bsp.c
 *            writes period - 1; a reporter's 400 Hz at 1 MHz read 2499):
 *              +0x00 period cfg, +0x04 duty cfg, +0x10 periods to emit,
 *              +0x14 CTRL (same bits), +0x40 / +0x44 / +0x50 / +0x54 the
 *              running period, duty, periods left and control.
 *
 * What makes this worth modelling rather than leaving as RAM is the part a
 * driver can get wrong:
 *
 *  - The block has a clock word in the CRG, and while that word gates it (or
 *    holds it in reset) the block takes every write and reads back zero.
 *    That is how a camera silently fails to dim (measured on an hi3516av300:
 *    0x120101BC bit 7 clear, every register 0).  The word is read through the
 *    system address space, so hisi-crg needs no link: "crg-addr" is its
 *    physical address, "gate-mask" the bits that must be set, "reset-mask"
 *    the bits that must be clear.
 *  - Which clock it counts depends on that word: "mux-shift"/"mux-width"
 *    pick an entry of "rate0".."rate3" (Hz).  Without a mux, rate0 is it.
 *  - The configuration reaches the output on a write to CTRL, not on the
 *    write to CFG1.  On V1 ("edge-latch") only the 0->1 edge of the enable
 *    bit loads it; everywhere else any CTRL write with enable set does.  On
 *    v5 a CFG write to a running channel lands too (the kernel's driver
 *    rewrites period/duty without touching CTRL).
 *  - A bounded run counts down on the clock it is given, so the bench
 *    method of measuring a family's clock -- sample the remaining-periods
 *    count twice, a known time apart -- gives this model's rate back.
 *  - A high time of 0 is not dark: the output keeps a floor of 2 ticks
 *    (classic; ST1 reads 2) or 1 tick (v5, duty 0 is "1 cycle").
 *  - The CRG's soft reset for the block (the "reset" GPIO input, which
 *    hisi-crg drives from the same bit "reset-mask" names) puts every
 *    channel back to its defaults, so a driver that pulses it starts clean.
 *
 * A run counts at the rate it was loaded with.  A source change takes
 * effect at the next load, which is when every driver here makes one (they
 * reprogram a channel disabled); a source switched under a running
 * channel is not modelled.
 *
 * The output is published as QOM properties per channel, for a test to
 * read over QMP: chN-enabled, chN-period-ns, chN-high-ns (time the pad is
 * HIGH, polarity applied), and clock-hz (0 while gated).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"

#define TYPE_HISI_PWM "hisi-pwm"
OBJECT_DECLARE_SIMPLE_TYPE(HisiPwmState, HISI_PWM)

#define HISI_PWM_MAX_CH 16

enum { LAYOUT_CLASSIC = 0, LAYOUT_SPLIT = 1, LAYOUT_V5 = 2 };

#define CTRL_ENABLE (1u << 0)
#define CTRL_INVERT (1u << 1)
#define CTRL_KEEP   (1u << 2)

typedef struct HisiPwmChannel {
    uint32_t cfg0, cfg1, cfg2, ctrl;
    uint32_t run_period, run_high, run_ctrl; /* what the output is doing */
    uint32_t run_count;                      /* periods the run emits */
    uint32_t run_hz;                         /* the clock it was loaded at */
    int64_t start_ns;                        /* when the run was loaded */
    bool running;
    bool loaded;                             /* ever, since reset */
} HisiPwmChannel;

struct HisiPwmState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t size;
    uint32_t layout;
    uint32_t channels;
    bool edge_latch;
    uint64_t crg_addr;
    uint32_t gate_mask;
    uint32_t reset_mask;
    uint32_t mux_shift;
    uint32_t mux_width;
    uint32_t rate[4];
    uint32_t cfg0_reset, cfg1_reset;

    HisiPwmChannel ch[HISI_PWM_MAX_CH];
};

/* The CRG word as the bus sees it, or "always clocked" when there is none. */
static bool hisi_pwm_clock(HisiPwmState *s, uint32_t *hz)
{
    uint32_t word = 0;
    unsigned idx = 0;

    if (s->crg_addr) {
        word = address_space_ldl_le(&address_space_memory, s->crg_addr,
                                    MEMTXATTRS_UNSPECIFIED, NULL);
        if ((word & s->gate_mask) != s->gate_mask ||
            (word & s->reset_mask)) {
            *hz = 0;
            return false;
        }
    }
    if (s->mux_width) {
        idx = (word >> s->mux_shift) & ((1u << s->mux_width) - 1);
    }
    *hz = s->rate[idx & 3];
    return *hz != 0;
}

static uint32_t hisi_pwm_mask(HisiPwmState *s)
{
    return s->layout == LAYOUT_V5 ? 0xfffff : 0xffffffff;
}

/* Ticks a configured value stands for: v5 stores value - 1. */
static uint64_t hisi_pwm_ticks(HisiPwmState *s, uint32_t v)
{
    return s->layout == LAYOUT_V5 ? (uint64_t)v + 1 : v;
}

static void hisi_pwm_load(HisiPwmState *s, HisiPwmChannel *c)
{
    hisi_pwm_clock(s, &c->run_hz);
    c->loaded = true;
    c->run_period = c->cfg0;
    c->run_high = c->cfg1;
    c->run_ctrl = c->ctrl;
    c->run_count = c->cfg2;
    c->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    c->running = c->ctrl & CTRL_ENABLE;
}

/* Periods of a bounded run not yet emitted. */
static uint32_t hisi_pwm_remaining(HisiPwmState *s, HisiPwmChannel *c)
{
    uint32_t hz;

    if (!c->running || (c->run_ctrl & CTRL_KEEP) || !hisi_pwm_clock(s, &hz)) {
        return 0;
    }
    hz = c->run_hz;
    uint64_t period = hisi_pwm_ticks(s, c->run_period);
    if (!period) {
        return 0;
    }
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - c->start_ns;
    uint64_t ticks = muldiv64(ns < 0 ? 0 : ns, hz, NANOSECONDS_PER_SECOND);
    uint64_t done = ticks / period;
    return done >= c->run_count ? 0 : c->run_count - done;
}

static int hisi_pwm_decode(HisiPwmState *s, hwaddr off, unsigned *reg)
{
    unsigned n;

    switch (s->layout) {
    case LAYOUT_V5:
        n = off / 0x100;
        *reg = off % 0x100;
        break;
    case LAYOUT_SPLIT:
        if (off >= 0x1000) {
            n = 4 + (off - 0x1000) / 0x20;
            *reg = (off - 0x1000) % 0x20;
        } else {
            n = off / 0x20;
            *reg = off % 0x20;
            if (n >= 4) {
                return -1;
            }
        }
        break;
    default:
        n = off / 0x20;
        *reg = off % 0x20;
        break;
    }
    return n < s->channels ? (int)n : -1;
}

static uint64_t hisi_pwm_read(void *opaque, hwaddr off, unsigned size)
{
    HisiPwmState *s = HISI_PWM(opaque);
    unsigned reg;
    uint32_t hz;
    int n = hisi_pwm_decode(s, off, &reg);

    if (n < 0 || !hisi_pwm_clock(s, &hz)) {
        return 0;
    }
    HisiPwmChannel *c = &s->ch[n];

    if (s->layout == LAYOUT_V5) {
        switch (reg) {
        case 0x00: return c->cfg0;
        case 0x04: return c->cfg1;
        case 0x10: return c->cfg2;
        case 0x14: return c->ctrl;
        case 0x40: return c->running ? c->run_period : 0;
        case 0x44: return c->running ? c->run_high : 0;
        case 0x50: return hisi_pwm_remaining(s, c);
        case 0x54: return c->running ? c->run_ctrl : 0;
        default:   return 0;
        }
    }
    switch (reg) {
    case 0x00: return c->cfg0;
    case 0x04: return c->cfg1;
    case 0x08: return c->cfg2;
    case 0x0c: return c->ctrl;
    case 0x10: return c->loaded ? c->run_period : 0;
    case 0x14: return !c->loaded ? 0 : c->run_high < 2 ? 2 : c->run_high;
    case 0x18: return (hisi_pwm_remaining(s, c) << 12) | (c->cfg2 & 0x3ff);
    default:   return 0;
    }
}

static void hisi_pwm_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    HisiPwmState *s = HISI_PWM(opaque);
    unsigned reg;
    uint32_t hz;
    int n = hisi_pwm_decode(s, off, &reg);

    if (n < 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-pwm: write to undecoded offset 0x%" HWADDR_PRIx
                      "\n", off);
        return;
    }
    if (!hisi_pwm_clock(s, &hz)) {
        return; /* a gated block takes the write and keeps nothing */
    }
    HisiPwmChannel *c = &s->ch[n];
    bool was_on = c->ctrl & CTRL_ENABLE;
    bool v5 = s->layout == LAYOUT_V5;

    /* One switch over the v5 offsets: classic CFG2/CTRL are moved onto them,
     * and classic's read-only mirrors (+0x10..) onto nothing. */
    unsigned r = reg;
    if (!v5) {
        r = reg == 0x08 ? 0x10 : reg == 0x0c ? 0x14 : reg >= 0x10 ? 0xff : reg;
    }
    switch (r) {
    case 0x00:
        c->cfg0 = val & hisi_pwm_mask(s);
        if (v5 && c->running) {
            c->run_period = c->cfg0;
        }
        return;
    case 0x04:
        c->cfg1 = val & hisi_pwm_mask(s);
        if (v5 && c->running) {
            c->run_high = c->cfg1;
        }
        return;
    case 0x10:
        c->cfg2 = val;
        return;
    case 0x14:
        c->ctrl = val & 0x3f;
        if (!(c->ctrl & CTRL_ENABLE)) {
            c->running = false;
        } else if (!s->edge_latch || !was_on) {
            hisi_pwm_load(s, c);
        }
        return;
    default:
        return; /* the status mirrors are read-only */
    }
}

static const MemoryRegionOps hisi_pwm_ops = {
    .read = hisi_pwm_read,
    .write = hisi_pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* What the pad is doing, for a test: enabled, period and high time in ns. */
static void hisi_pwm_output(HisiPwmState *s, int n, bool *on,
                            uint64_t *period_ns, uint64_t *high_ns)
{
    HisiPwmChannel *c = &s->ch[n];
    uint32_t hz;

    *on = false;
    *period_ns = *high_ns = 0;
    if (!c->running || !hisi_pwm_clock(s, &hz)) {
        return;
    }
    if (!(c->run_ctrl & CTRL_KEEP) && hisi_pwm_remaining(s, c) == 0) {
        return; /* a bounded run that has finished */
    }
    hz = c->run_hz;
    uint64_t period = hisi_pwm_ticks(s, c->run_period);
    uint64_t high = hisi_pwm_ticks(s, c->run_high);
    if (s->layout != LAYOUT_V5 && high < 2) {
        high = 2;
    }
    if (high > period) {
        high = period;
    }
    if (c->run_ctrl & CTRL_INVERT) {
        high = period - high;
    }
    *on = true;
    *period_ns = muldiv64(period, NANOSECONDS_PER_SECOND, hz);
    *high_ns = muldiv64(high, NANOSECONDS_PER_SECOND, hz);
}

enum { PROP_ENABLED, PROP_PERIOD, PROP_HIGH };

static void hisi_pwm_get_prop(Object *obj, Visitor *v, const char *name,
                              void *opaque, Error **errp)
{
    HisiPwmState *s = HISI_PWM(obj);
    uintptr_t key = (uintptr_t)opaque;
    int n = key >> 4;
    bool on;
    uint64_t period, high;

    hisi_pwm_output(s, n, &on, &period, &high);
    switch (key & 0xf) {
    case PROP_ENABLED:
        visit_type_bool(v, name, &on, errp);
        break;
    case PROP_PERIOD:
        visit_type_uint64(v, name, &period, errp);
        break;
    default:
        visit_type_uint64(v, name, &high, errp);
        break;
    }
}

static void hisi_pwm_get_clock(Object *obj, Visitor *v, const char *name,
                               void *opaque, Error **errp)
{
    uint32_t hz;

    hisi_pwm_clock(HISI_PWM(obj), &hz);
    visit_type_uint32(v, name, &hz, errp);
}

static void hisi_pwm_defaults(HisiPwmState *s)
{
    for (unsigned i = 0; i < HISI_PWM_MAX_CH; i++) {
        memset(&s->ch[i], 0, sizeof(s->ch[i]));
        s->ch[i].cfg0 = s->cfg0_reset;
        s->ch[i].cfg1 = s->cfg1_reset;
    }
}

static void hisi_pwm_reset(DeviceState *dev)
{
    hisi_pwm_defaults(HISI_PWM(dev));
}

/* The CRG's soft reset for this block: held, it holds the defaults. */
static void hisi_pwm_reset_in(void *opaque, int n, int level)
{
    if (level) {
        hisi_pwm_defaults(HISI_PWM(opaque));
    }
}

static void hisi_pwm_realize(DeviceState *dev, Error **errp)
{
    HisiPwmState *s = HISI_PWM(dev);
    static const char *const kind[] = { "enabled", "period-ns", "high-ns" };

    if (!s->channels || s->channels > HISI_PWM_MAX_CH) {
        error_setg(errp, "hisi-pwm: channels must be 1..%d", HISI_PWM_MAX_CH);
        return;
    }
    if (s->layout > LAYOUT_V5 ||
        (s->layout == LAYOUT_SPLIT && s->channels > 8)) {
        error_setg(errp, "hisi-pwm: bad layout");
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(dev), &hisi_pwm_ops, s,
                          TYPE_HISI_PWM, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    qdev_init_gpio_in_named(dev, hisi_pwm_reset_in, "reset", 1);

    for (unsigned n = 0; n < s->channels; n++) {
        for (unsigned k = 0; k < 3; k++) {
            g_autofree char *pname = g_strdup_printf("ch%u-%s", n, kind[k]);
            object_property_add(OBJECT(dev), pname,
                                k == PROP_ENABLED ? "bool" : "uint64",
                                hisi_pwm_get_prop, NULL, NULL,
                                (void *)(uintptr_t)((n << 4) | k));
        }
    }
    object_property_add(OBJECT(dev), "clock-hz", "uint32", hisi_pwm_get_clock,
                        NULL, NULL, NULL);
}

static const Property hisi_pwm_properties[] = {
    DEFINE_PROP_UINT32("size", HisiPwmState, size, 0x10000),
    DEFINE_PROP_UINT32("layout", HisiPwmState, layout, LAYOUT_CLASSIC),
    DEFINE_PROP_UINT32("channels", HisiPwmState, channels, 4),
    DEFINE_PROP_BOOL("edge-latch", HisiPwmState, edge_latch, false),
    DEFINE_PROP_UINT64("crg-addr", HisiPwmState, crg_addr, 0),
    DEFINE_PROP_UINT32("gate-mask", HisiPwmState, gate_mask, 0),
    DEFINE_PROP_UINT32("reset-mask", HisiPwmState, reset_mask, 0),
    DEFINE_PROP_UINT32("mux-shift", HisiPwmState, mux_shift, 0),
    DEFINE_PROP_UINT32("mux-width", HisiPwmState, mux_width, 0),
    DEFINE_PROP_UINT32("rate0", HisiPwmState, rate[0], 3000000),
    DEFINE_PROP_UINT32("rate1", HisiPwmState, rate[1], 0),
    DEFINE_PROP_UINT32("rate2", HisiPwmState, rate[2], 0),
    DEFINE_PROP_UINT32("rate3", HisiPwmState, rate[3], 0),
    DEFINE_PROP_UINT32("cfg0-reset", HisiPwmState, cfg0_reset, 0),
    DEFINE_PROP_UINT32("cfg1-reset", HisiPwmState, cfg1_reset, 0),
};

static void hisi_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, hisi_pwm_properties);
    device_class_set_legacy_reset(dc, hisi_pwm_reset);
    dc->realize = hisi_pwm_realize;
}

static const TypeInfo hisi_pwm_info = {
    .name          = TYPE_HISI_PWM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiPwmState),
    .class_init    = hisi_pwm_class_init,
};

static void hisi_pwm_register_types(void)
{
    type_register_static(&hisi_pwm_info);
}

type_init(hisi_pwm_register_types)
