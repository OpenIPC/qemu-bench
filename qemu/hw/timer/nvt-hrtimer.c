/*
 * Novatek NS02501 (NT98532 / NT98533) high-resolution timers: one 64-bit
 * up-counter per 4 KiB block, nine blocks (0xF00C0000, 0xF0150000 ..
 * 0xF01A0000, 0xF01C0000, 0xF01E0000).  The SDK kernel's
 * drivers/soc/nvt/basic/nvt-timer ("nvt,nvt_hwtimer") and
 * drivers/clocksource/timer-novatek.c ("nvt,nvt_clk_src") drive them,
 * with plat-ns02501_a32/hrtimer-reg.h:
 *
 *   0x00 STS        bit 0 the count reached TVAL, bit 1 a latch is done;
 *                   write 1 to clear
 *   0x04 INTE       bit 0: STS bit 0 raises the interrupt
 *   0x08 CTRL       bit 0 enable, bit 1 free run (else one shot), bit 2
 *                   start from INIT_TVAL (else from 0)
 *   0x0C / 0x10     TVAL, the 64-bit target
 *   0x1C RLD        (storage; the drivers write 0)
 *   0x20 CLK_DIV    [7:0]: the count runs at 120 MHz / (CLK_DIV + 1)
 *   0x24 / 0x28     INIT_TVAL, the count an enable with bit 2 starts from
 *   0x2C .. 0x7C    the external-count capture (EXTCNT_EN, the buffer
 *                   index, the captured counts): storage, nothing captures
 *   0x100 LATCH     a write latches the count and sets STS bit 1
 *   0x104 / 0x108   CNT, the count
 *
 * NA51103 (nt98332) has the same block at 0x2_F0140000 (SPI 83: HR timer 0
 * and the clocksource) and 0x2_F0170000 (SPI 84: HR timer 1 and ptp_nvt.c's
 * PTP clock), with plat-na51103_a32/hrtimer-reg.h's older layout: CNT at
 * 0x14 / 0x18 and no latch -- property "count-at" 0x14.
 *
 * Enabling a stopped timer starts it from 0 (or INIT_TVAL): the
 * clockevent's set_next_event() stops it, writes the delta to TVAL and
 * starts it again, which works only that way.  At TVAL STS bit 0 is set;
 * a free-running timer wraps to 0 and goes on, a one-shot one stops at
 * TVAL (CTRL enable reads 0).  Stopping holds the count.  The latch is
 * instant, so the polling-sync read sees it done at once.
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

#define TYPE_NVT_HRTIMER "nvt-hrtimer"
OBJECT_DECLARE_SIMPLE_TYPE(NvtHrtimerState, NVT_HRTIMER)

#define HRT_SIZE        0x1000
#define HRT_REGS        0x110
#define HRT_STS         0x00
#define HRT_INTE        0x04
#define HRT_CTRL        0x08
#define HRT_TVAL_L      0x0C
#define HRT_TVAL_H      0x10
#define HRT_CLK_DIV     0x20
#define HRT_INIT_L      0x24
#define HRT_INIT_H      0x28
#define HRT_LATCH       0x100
#define HRT_CNT_L       0x104
#define HRT_CNT_H       0x108

#define STS_TARGET      (1u << 0)
#define STS_LATCH       (1u << 1)
#define CTRL_EN         (1u << 0)
#define CTRL_FREE_RUN   (1u << 1)
#define CTRL_INIT       (1u << 2)

#define HRT_HZ          120000000ULL
/* further than this (hours at the slowest divider) is never scheduled */
#define HRT_MAX_DELTA   (1ULL << 52)

struct NvtHrtimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *qt;
    uint32_t regs[HRT_REGS / 4];
    /* while enabled, the count is base_cnt at base_ns plus the ticks since */
    int64_t base_ns;
    uint64_t base_cnt;
    uint64_t held;              /* the count while stopped */
    uint32_t count_at;          /* CNT_L's offset: 0x104, or NA51103's 0x14 */
};

static uint32_t *reg(NvtHrtimerState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static uint64_t reg64(NvtHrtimerState *s, hwaddr lo)
{
    return *reg(s, lo) | (uint64_t)*reg(s, lo + 4) << 32;
}

static uint64_t hrt_hz(NvtHrtimerState *s)
{
    return HRT_HZ / ((*reg(s, HRT_CLK_DIV) & 0xFF) + 1);
}

static void hrt_update_irq(NvtHrtimerState *s)
{
    qemu_set_irq(s->irq, !!(*reg(s, HRT_STS) & *reg(s, HRT_INTE) &
                            STS_TARGET));
}

/* The count without wrapping: base_cnt plus the ticks since base_ns. */
static uint64_t hrt_raw(NvtHrtimerState *s)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->base_ns;

    return s->base_cnt + muldiv64(MAX(ns, 0), hrt_hz(s),
                                  NANOSECONDS_PER_SECOND);
}

/* What CNT reads: a free-running timer wraps after TVAL. */
static uint64_t hrt_count(NvtHrtimerState *s)
{
    uint64_t raw, tval = reg64(s, HRT_TVAL_L);

    if (!(*reg(s, HRT_CTRL) & CTRL_EN)) {
        return s->held;
    }
    raw = hrt_raw(s);
    if (*reg(s, HRT_CTRL) & CTRL_FREE_RUN) {
        return tval == UINT64_MAX ? raw : raw % (tval + 1);
    }
    return MIN(raw, tval);
}

/*
 * Arm the QEMU timer for the next time the count reaches TVAL: in free
 * run the next raw count that is TVAL modulo TVAL + 1 (strictly later
 * than now when a period has just ended).
 */
static void hrt_schedule(NvtHrtimerState *s, bool after)
{
    uint64_t tval = reg64(s, HRT_TVAL_L), raw = hrt_raw(s), next, d, ns;
    uint64_t hz = hrt_hz(s);

    timer_del(s->qt);
    if (*reg(s, HRT_CTRL) & CTRL_FREE_RUN) {
        if (tval == UINT64_MAX) {
            return;                     /* the clocksource: never reached */
        }
        next = raw - raw % (tval + 1) + tval;
        if (next < raw || (after && next == raw)) {
            next += tval + 1;
        }
    } else if (after) {
        return;
    } else {
        next = MAX(tval, raw);
    }
    d = next - s->base_cnt;
    if (d > HRT_MAX_DELTA) {
        return;
    }
    ns = muldiv64(d, NANOSECONDS_PER_SECOND, hz);
    if (muldiv64(ns, hz, NANOSECONDS_PER_SECOND) < d) {
        ns++;
    }
    timer_mod(s->qt, s->base_ns + ns);
}

static void hrt_rebase(NvtHrtimerState *s, uint64_t cnt)
{
    s->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->base_cnt = cnt;
    hrt_schedule(s, false);
}

static void hrt_expired(void *opaque)
{
    NvtHrtimerState *s = opaque;

    *reg(s, HRT_STS) |= STS_TARGET;
    if (*reg(s, HRT_CTRL) & CTRL_FREE_RUN) {
        hrt_schedule(s, true);
    } else {
        s->held = reg64(s, HRT_TVAL_L);
        *reg(s, HRT_CTRL) &= ~CTRL_EN;
    }
    hrt_update_irq(s);
}

static uint64_t nvt_hrtimer_read(void *opaque, hwaddr off, unsigned size)
{
    NvtHrtimerState *s = opaque;

    if (off == s->count_at) {
        return (uint32_t)hrt_count(s);
    }
    if (off == s->count_at + 4) {
        return hrt_count(s) >> 32;
    }
    if (off == HRT_LATCH) {
        return 0;
    }
    return off < HRT_REGS ? *reg(s, off) : 0;
}

static void hrt_write_ctrl(NvtHrtimerState *s, uint32_t val)
{
    uint32_t old = *reg(s, HRT_CTRL);
    uint64_t cnt = hrt_count(s);

    val &= CTRL_EN | CTRL_FREE_RUN | CTRL_INIT;
    *reg(s, HRT_CTRL) = val;
    if ((old & CTRL_EN) && !(val & CTRL_EN)) {
        s->held = cnt;
        timer_del(s->qt);
    } else if (!(old & CTRL_EN) && (val & CTRL_EN)) {
        hrt_rebase(s, val & CTRL_INIT ? reg64(s, HRT_INIT_L) : 0);
    } else if ((val & CTRL_EN) && ((old ^ val) & CTRL_FREE_RUN)) {
        hrt_rebase(s, cnt);
    }
}

static void nvt_hrtimer_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    NvtHrtimerState *s = opaque;
    bool running = *reg(s, HRT_CTRL) & CTRL_EN;
    uint64_t cnt;

    if (off == s->count_at || off == s->count_at + 4) {
        return;                         /* read only */
    }
    switch (off) {
    case HRT_STS:
        *reg(s, off) &= ~val;
        hrt_update_irq(s);
        return;
    case HRT_INTE:
        *reg(s, off) = val & 1;
        hrt_update_irq(s);
        return;
    case HRT_CTRL:
        hrt_write_ctrl(s, val);
        return;
    case HRT_LATCH:
        if (s->count_at == HRT_CNT_L) {
            *reg(s, HRT_STS) |= STS_LATCH;
        }
        return;
    case HRT_TVAL_L:
    case HRT_TVAL_H:
    case HRT_CLK_DIV:
        cnt = hrt_count(s);
        *reg(s, off) = val;
        if (running) {
            hrt_rebase(s, cnt);
        }
        return;
    }
    if (off < HRT_REGS) {
        *reg(s, off) = val;
    }
}

static const MemoryRegionOps nvt_hrtimer_ops = {
    .read = nvt_hrtimer_read,
    .write = nvt_hrtimer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_hrtimer_reset(DeviceState *dev)
{
    NvtHrtimerState *s = NVT_HRTIMER(dev);

    timer_del(s->qt);
    memset(s->regs, 0, sizeof(s->regs));
    s->held = 0;
    s->base_cnt = 0;
    s->base_ns = 0;
    qemu_irq_lower(s->irq);
}

static void nvt_hrtimer_init(Object *obj)
{
    NvtHrtimerState *s = NVT_HRTIMER(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_hrtimer_ops, s,
                          TYPE_NVT_HRTIMER, HRT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->qt = timer_new_ns(QEMU_CLOCK_VIRTUAL, hrt_expired, s);
}

static const VMStateDescription vmstate_nvt_hrtimer = {
    .name = TYPE_NVT_HRTIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtHrtimerState, HRT_REGS / 4),
        VMSTATE_TIMER_PTR(qt, NvtHrtimerState),
        VMSTATE_INT64(base_ns, NvtHrtimerState),
        VMSTATE_UINT64(base_cnt, NvtHrtimerState),
        VMSTATE_UINT64(held, NvtHrtimerState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property nvt_hrtimer_properties[] = {
    DEFINE_PROP_UINT32("count-at", NvtHrtimerState, count_at, HRT_CNT_L),
};

static void nvt_hrtimer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, nvt_hrtimer_properties);
    device_class_set_legacy_reset(dc, nvt_hrtimer_reset);
    dc->vmsd = &vmstate_nvt_hrtimer;
}

static const TypeInfo nvt_hrtimer_info = {
    .name = TYPE_NVT_HRTIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtHrtimerState),
    .instance_init = nvt_hrtimer_init,
    .class_init = nvt_hrtimer_class_init,
};

static void nvt_hrtimer_register_types(void)
{
    type_register_static(&nvt_hrtimer_info);
}

type_init(nvt_hrtimer_register_types)
