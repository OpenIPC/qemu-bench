/*
 * Ingenic system OST: the T-series "operating system timer" at 0x12000000.
 *
 * Two counters behind one enable register (arch/mips/xburst/soc-t31/
 * include/soc/ost.h, common/timer_sys_ost.c -- the same file and header
 * on T10, T20, T21, T23 and T30):
 *
 *   TCCR   0x00  prescaler, 2 bits per counter: EXTAL / 4^n
 *   TER    0x04  enables (bit 0 counter 1, bit 1 counter 2)
 *   TCR    0x08  write 1 to zero a counter
 *   TFR    0x0C  bit 0: counter 1 reached T1DFR (write to set/clear)
 *   TMR    0x10  bit 0: counter 1 interrupt mask, 1 = masked
 *   T1DFR  0x14  counter 1 compare value
 *   T1CNT  0x18  counter 1, 32-bit up counter
 *   T2CNTH 0x1C  counter 2 bits 63:32
 *   T2CNTL 0x20  counter 2 bits 31:0; reading it latches the high half
 *   T2HBUF 0x24  ... into here
 *   TESR   0x34  write 1 to set TER bits
 *   TECR   0x38  write 1 to clear TER bits
 *
 * Counter 1 is the clock event: it counts up to T1DFR, sets TFR bit 0,
 * starts again from zero, and interrupts while TFR & ~TMR.  The line goes
 * to the CPU's IP4 directly, not through the INTC.  Counter 2 is the
 * 64-bit free-running clock source.  Both run from EXTAL.
 *
 * XBurst2 (T40, T41; drivers/clocksource/ingenic_core_ost.c) splits the
 * two counters into blocks of their own, which this model also is
 * (layout property):
 *
 *   global OST 0x12000000 (layout 1): CCR 0x00 prescaler, ER 0x04, CR 0x08
 *     clear, CNTH 0x0C, CNTL 0x10 (latches CNTB), CNTB 0x14 -- counter 2
 *   core OST 0x12100000 + 0x100 n (layout 2): CCR 0x00, ER 0x04, CR 0x08,
 *     FR 0x0C, MR 0x10, DFR 0x14, CNT 0x18 -- counter 1, one per core
 *
 * ER is a plain enable there (1 = run).  U-Boot runs the global one at
 * EXTAL/4, the kernel at EXTAL.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_OST "ingenic-ost"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicOstState, INGENIC_OST)

#define R_TCCR      0x00
#define R_TER       0x04
#define R_TCR       0x08
#define R_TFR       0x0C
#define R_TMR       0x10
#define R_T1DFR     0x14
#define R_T1CNT     0x18
#define R_T2CNTH    0x1C
#define R_T2CNTL    0x20
#define R_T2HBUF    0x24
#define R_TESR      0x34
#define R_TECR      0x38

#define CH1         (1u << 0)
#define CH2         (1u << 1)

struct IngenicOstState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;
    uint32_t extal_hz;
    uint32_t layout;

    uint32_t tccr, ter, tfr, tmr, t1dfr;
    uint32_t t2hbuf;
    /* counter value at base_ns, while enabled; the value itself if not */
    uint64_t cnt[2];
    int64_t base_ns[2];
};

static uint64_t ost_rate(IngenicOstState *s, int ch)
{
    return s->extal_hz >> (2 * ((s->tccr >> (2 * ch)) & 3));
}

static int64_t ost_now(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static uint64_t ost_count(IngenicOstState *s, int ch)
{
    if (!(s->ter & (1u << ch))) {
        return s->cnt[ch];
    }
    return s->cnt[ch] + muldiv64(ost_now() - s->base_ns[ch], ost_rate(s, ch),
                                 NANOSECONDS_PER_SECOND);
}

/* Freeze or re-base a counter so that later rate/enable changes apply. */
static void ost_latch(IngenicOstState *s, int ch)
{
    s->cnt[ch] = ost_count(s, ch);
    s->base_ns[ch] = ost_now();
}

static void ost_update_irq(IngenicOstState *s)
{
    qemu_set_irq(s->irq, (s->tfr & ~s->tmr & CH1) != 0);
}

/* Fold elapsed matches of counter 1 into TFR and arm the next one. */
static void ost_ch1_sync(IngenicOstState *s)
{
    uint64_t cnt, rate, period;
    int64_t when;

    if (!(s->ter & CH1)) {
        /* stopped: no match to come, but a TFR write still moves the line
         * (the T41 kernel clears TFR with the counter off; left high, core
         * 1 storms with "ost couldn't find ostfr") */
        timer_del(s->timer);
        ost_update_irq(s);
        return;
    }
    rate = ost_rate(s, 0);
    period = (uint64_t)s->t1dfr + 1;
    cnt = ost_count(s, 0);
    if (cnt > s->t1dfr) {
        /* passed the compare value: flag it and restart from zero */
        s->tfr |= CH1;
        cnt %= period;
        s->cnt[0] = cnt;
        s->base_ns[0] = ost_now();
    }
    /* the match after cnt; ost_count() measures from base_ns/cnt[0] */
    when = s->base_ns[0] +
           muldiv64(period - s->cnt[0], NANOSECONDS_PER_SECOND, rate);
    timer_mod(s->timer, MAX(when, ost_now() + 1));
    ost_update_irq(s);
}

static void ost_timer_cb(void *opaque)
{
    IngenicOstState *s = opaque;

    ost_ch1_sync(s);
}

static uint64_t ost_v1_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicOstState *s = opaque;
    uint64_t c2;

    switch (addr) {
    case R_TCCR:
        return s->tccr;
    case R_TER:
        return s->ter;
    case R_TFR:
        ost_ch1_sync(s);
        return s->tfr;
    case R_TMR:
        return s->tmr;
    case R_T1DFR:
        return s->t1dfr;
    case R_T1CNT:
        ost_ch1_sync(s);
        return ost_count(s, 0);
    case R_T2CNTH:
        return ost_count(s, 1) >> 32;
    case R_T2CNTL:
        c2 = ost_count(s, 1);
        s->t2hbuf = c2 >> 32;
        return (uint32_t)c2;
    case R_T2HBUF:
        return s->t2hbuf;
    default:
        return 0;
    }
}

static void ost_set_enable(IngenicOstState *s, uint32_t ter)
{
    int ch;

    for (ch = 0; ch < 2; ch++) {
        ost_latch(s, ch);
    }
    s->ter = ter & (CH1 | CH2);
}

static void ost_v1_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    IngenicOstState *s = opaque;
    int ch;

    switch (addr) {
    case R_TCCR:
        for (ch = 0; ch < 2; ch++) {
            ost_latch(s, ch);
        }
        s->tccr = val & 0xf;
        break;
    case R_TER:
        ost_set_enable(s, val);
        break;
    case R_TESR:
        ost_set_enable(s, s->ter | val);
        break;
    case R_TECR:
        ost_set_enable(s, s->ter & ~val);
        break;
    case R_TCR:
        for (ch = 0; ch < 2; ch++) {
            if (val & (1u << ch)) {
                s->cnt[ch] = 0;
                s->base_ns[ch] = ost_now();
            }
        }
        break;
    case R_TFR:
        s->tfr = val & CH1;
        break;
    case R_TMR:
        s->tmr = val & CH1;
        break;
    case R_T1DFR:
        s->t1dfr = val;
        break;
    case R_T1CNT:
        s->cnt[0] = (uint32_t)val;
        s->base_ns[0] = ost_now();
        break;
    case R_T2CNTH:
        ost_latch(s, 1);
        s->cnt[1] = deposit64(s->cnt[1], 32, 32, val);
        break;
    case R_T2CNTL:
        ost_latch(s, 1);
        s->cnt[1] = deposit64(s->cnt[1], 0, 32, val);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ingenic-ost: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", addr, val);
        return;
    }
    ost_ch1_sync(s);
}


enum { OST_SYS, OST_GLOBAL, OST_CORE };

/* XBurst2 global OST: counter 2 behind its own register names */
static const hwaddr ost_global_map[] = {
    [0x00 / 4] = R_TCCR, [0x04 / 4] = R_TER, [0x08 / 4] = R_TCR,
    [0x0C / 4] = R_T2CNTH, [0x10 / 4] = R_T2CNTL, [0x14 / 4] = R_T2HBUF,
};
/* XBurst2 core OST: counter 1 */
static const hwaddr ost_core_map[] = {
    [0x00 / 4] = R_TCCR, [0x04 / 4] = R_TER, [0x08 / 4] = R_TCR,
    [0x0C / 4] = R_TFR, [0x10 / 4] = R_TMR, [0x14 / 4] = R_T1DFR,
    [0x18 / 4] = R_T1CNT,
};

static bool ost_map(IngenicOstState *s, hwaddr addr, hwaddr *reg)
{
    const hwaddr *map = s->layout == OST_GLOBAL ? ost_global_map :
                        ost_core_map;
    size_t n = s->layout == OST_GLOBAL ? ARRAY_SIZE(ost_global_map) :
               ARRAY_SIZE(ost_core_map);

    if (addr / 4 >= n || (addr & 3)) {
        return false;
    }
    *reg = map[addr / 4];
    return true;
}

static uint64_t ingenic_ost_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicOstState *s = opaque;
    hwaddr reg;
    uint32_t v;

    if (s->layout == OST_SYS) {
        return ost_v1_read(opaque, addr, size);
    }
    if (!ost_map(s, addr, &reg)) {
        return 0;
    }
    v = ost_v1_read(opaque, reg, size);
    if (s->layout == OST_GLOBAL) {
        if (reg == R_TCCR) {
            v = (v >> 2) & 3;
        } else if (reg == R_TER) {
            v = (v >> 1) & 1;
        }
    }
    return v;
}

static void ingenic_ost_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    IngenicOstState *s = opaque;
    int ch = s->layout == OST_GLOBAL ? 1 : 0;
    hwaddr reg;

    if (s->layout == OST_SYS) {
        ost_v1_write(opaque, addr, val, size);
        return;
    }
    if (!ost_map(s, addr, &reg)) {
        return;
    }
    switch (reg) {
    case R_TCCR:
        val = deposit32(s->tccr, 2 * ch, 2, val);
        break;
    case R_TER:
        reg = (val & 1) ? R_TESR : R_TECR;
        val = 1u << ch;
        break;
    case R_TCR:
        val = (val & 1) << ch;
        break;
    }
    ost_v1_write(opaque, reg, val, size);
}

static const MemoryRegionOps ingenic_ost_ops = {
    .read = ingenic_ost_read,
    .write = ingenic_ost_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ingenic_ost_reset(DeviceState *dev)
{
    IngenicOstState *s = INGENIC_OST(dev);

    s->tccr = s->ter = s->tfr = s->t1dfr = s->t2hbuf = 0;
    s->tmr = CH1;
    s->cnt[0] = s->cnt[1] = 0;
    s->base_ns[0] = s->base_ns[1] = ost_now();
    timer_del(s->timer);
    ost_update_irq(s);
}

static void ingenic_ost_init(Object *obj)
{
    IngenicOstState *s = INGENIC_OST(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_ost_ops, s,
                          TYPE_INGENIC_OST, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ost_timer_cb, s);
}

static const Property ingenic_ost_props[] = {
    DEFINE_PROP_UINT32("extal-hz", IngenicOstState, extal_hz, 24000000),
    DEFINE_PROP_UINT32("layout", IngenicOstState, layout, 0),
};

static const VMStateDescription vmstate_ingenic_ost = {
    .name = TYPE_INGENIC_OST,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(tccr, IngenicOstState),
        VMSTATE_UINT32(ter, IngenicOstState),
        VMSTATE_UINT32(tfr, IngenicOstState),
        VMSTATE_UINT32(tmr, IngenicOstState),
        VMSTATE_UINT32(t1dfr, IngenicOstState),
        VMSTATE_UINT32(t2hbuf, IngenicOstState),
        VMSTATE_UINT64_ARRAY(cnt, IngenicOstState, 2),
        VMSTATE_INT64_ARRAY(base_ns, IngenicOstState, 2),
        VMSTATE_TIMER_PTR(timer, IngenicOstState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ost_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_ost_reset);
    device_class_set_props(dc, ingenic_ost_props);
    dc->vmsd = &vmstate_ingenic_ost;
}

static const TypeInfo ingenic_ost_info = {
    .name = TYPE_INGENIC_OST,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicOstState),
    .instance_init = ingenic_ost_init,
    .class_init = ingenic_ost_class_init,
};

static void ingenic_ost_register_types(void)
{
    type_register_static(&ingenic_ost_info);
}

type_init(ingenic_ost_register_types)
