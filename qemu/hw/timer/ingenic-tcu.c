/*
 * Ingenic TCU: timer/counter unit at 0x10002000, with the watchdog at its
 * base and, on T10..T31, the boot loader's 64-bit OST inside it.
 *
 *   0x00 TDR  0x04 TCER  0x08 TCNT  0x0C TCSR   watchdog (8/16/32-bit)
 *   0x10 TER  0x14 TESR  0x18 TECR             counter enables: bits 0-7
 *                                              channels, 15 the OST
 *   0x1C TSR  0x2C TSSR  0x3C TSCR             stop bits; 16 = watchdog
 *   0x20 TFR  0x24 TFSR  0x28 TFCR             flags: 0-7 full, 16-23 half
 *   0x30 TMR  0x34 TMSR  0x38 TMCR             flag masks
 *   0x40 + 0x10 n  TDFR TDHR TCNT TCSR          channel n (0..7)
 *   0xE0 OSTDR 0xE4 OSTCNTL 0xE8 OSTCNTH 0xEC OSTCSR 0xFC OSTCNTHBUF
 *   0xF0 TSTR 0xF4 TSTSR 0xF8 TSTCR
 *
 * Each TCSR picks a clock (bit 0 PCLK, 1 RTC, 2 EXTAL) and a prescaler
 * 4^(bits 5:3).  U-Boot's udelay is the OST: TESR bit 15, OSTCSR =
 * EXTAL/4 free running, read low then the high half latched in HBUF
 * (U-Boot arch/mips/cpu/xburst/timer.c).  The watchdog counts while TCER
 * bit 0 is set and TSR bit 16 is clear; at TCNT >= TDR it resets the
 * system, which is how both U-Boot and the kernels reboot (soc-t31/
 * common/reset.c: TDR = 2 at RTC/64, then a spin).  Each channel counts
 * at its TCSR clock and, on reaching TDHR (half) and TDFR (full, then
 * from zero), sets its TFR flags on time -- a timer per channel -- and
 * interrupts unless TMR masks it: TCU0 for channel 0, TCU1 for 1, TCU2
 * for the rest (the 3.10 jz_tcu demuxes every channel off TCU2; the
 * motor drivers step on channel 2's full interrupt).
 *
 * A channel's PWM pin (TCSR bit 7 enables it, bit 8 is the initial level,
 * bit 9 shuts it down; OpenIPC's jz_tcu.c and the SDKs' sample_pwm) starts
 * each period at the initial level and flips when the counter passes
 * TDHR, until TDFR.  The read-only "pwmN" properties report the pin as
 * "high/period" in counter ticks while the channel counts, "0" / "1" for
 * a level that does not change, or "off" while the TCU does not drive it.
 *
 * XBurst2 (T40, T41; property xburst2): TSR bit 16 does not stop the
 * watchdog, and TCSR bit 10 zeroes its counter (it reads back 0).  The
 * 4.4 kernels' watchdog driver stops "channel 16" through TSSR when it
 * probes, nothing clears it again, and their restart (soc-t4x/reset.c
 * wdt_start_count: TCSR = 1 << 10, TDR, TCSR = RTC/64, TCER = 1) and the
 * driver's ping (ingenic_watchdog_set_count: TCSR |= 1 << 10) rely on
 * both; the 3.10 kernels clear TSR bit 16 through TSCR first.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/runstate.h"

#define TYPE_INGENIC_TCU "ingenic-tcu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicTcuState, INGENIC_TCU)

#define R_TDR       0x00
#define R_TCER      0x04
#define R_TCNT      0x08
#define R_TCSR      0x0C
#define R_TER       0x10
#define R_TESR      0x14
#define R_TECR      0x18
#define R_TSR       0x1C
#define R_TFR       0x20
#define R_TFSR      0x24
#define R_TFCR      0x28
#define R_TSSR      0x2C
#define R_TMR       0x30
#define R_TMSR      0x34
#define R_TMCR      0x38
#define R_TSCR      0x3C
#define R_CH0       0x40
#define R_CH_END    0xC0
#define R_OSTDR     0xE0
#define R_OSTCNTL   0xE4
#define R_OSTCNTH   0xE8
#define R_OSTCSR    0xEC
#define R_TSTR      0xF0
#define R_TSTSR     0xF4
#define R_TSTCR     0xF8
#define R_OSTHBUF   0xFC

#define NCH         8
#define TER_OST     (1u << 15)
#define TSR_WDT     (1u << 16)
#define RTC_HZ      32768

typedef struct TcuChannel {
    uint32_t tdfr, tdhr, tcsr;
    uint32_t cnt;           /* at base_ns while enabled */
    int64_t base_ns;
    QEMUTimer *timer;       /* the next half or full compare */
    struct IngenicTcuState *tcu;
} TcuChannel;

struct IngenicTcuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer *wdt_timer;
    qemu_irq irq[3];
    uint32_t extal_hz, pclk_hz;
    bool xburst2;

    /* watchdog */
    uint32_t tdr, tcer, tcsr;
    uint32_t wcnt;
    int64_t wbase_ns;

    uint32_t ter, tsr, tfr, tmr, tstr;
    TcuChannel ch[NCH];

    /* OST */
    uint32_t ostdr, ostcsr, osthbuf;
    uint64_t ostcnt;
    int64_t ostbase_ns;
};

static int64_t tcu_now(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static uint64_t tcu_rate(IngenicTcuState *s, uint32_t tcsr)
{
    uint64_t hz;

    if (tcsr & 4) {
        hz = s->extal_hz;
    } else if (tcsr & 2) {
        hz = RTC_HZ;
    } else if (tcsr & 1) {
        hz = s->pclk_hz;
    } else {
        return 0;
    }
    return hz >> (2 * MIN(extract32(tcsr, 3, 3), 5));
}

static uint64_t tcu_ticks(uint64_t rate, int64_t since)
{
    return rate ? muldiv64(tcu_now() - since, rate, NANOSECONDS_PER_SECOND) : 0;
}

/* --- watchdog ----------------------------------------------------------- */

static bool wdt_running(IngenicTcuState *s)
{
    return (s->tcer & 1) && (s->xburst2 || !(s->tsr & TSR_WDT)) &&
           tcu_rate(s, s->tcsr);
}

static uint32_t wdt_count(IngenicTcuState *s)
{
    if (!wdt_running(s)) {
        return s->wcnt;
    }
    return MIN(s->wcnt + tcu_ticks(tcu_rate(s, s->tcsr), s->wbase_ns),
               0xffffu);
}

static void wdt_latch(IngenicTcuState *s)
{
    s->wcnt = wdt_count(s);
    s->wbase_ns = tcu_now();
}

static void wdt_arm(IngenicTcuState *s)
{
    uint64_t rate = tcu_rate(s, s->tcsr);

    if (!wdt_running(s)) {
        timer_del(s->wdt_timer);
        return;
    }
    if (s->wcnt >= s->tdr) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-tcu: watchdog reset\n");
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        timer_del(s->wdt_timer);
        return;
    }
    timer_mod(s->wdt_timer, s->wbase_ns +
              muldiv64(s->tdr - s->wcnt, NANOSECONDS_PER_SECOND, rate));
}

static void wdt_timer_cb(void *opaque)
{
    IngenicTcuState *s = opaque;

    wdt_latch(s);
    wdt_arm(s);
}

/* --- channels ----------------------------------------------------------- */

static uint32_t ch_count(IngenicTcuState *s, int n)
{
    TcuChannel *c = &s->ch[n];
    uint64_t cnt, period;

    if (!(s->ter & (1u << n)) || (s->tsr & (1u << n))) {
        return c->cnt;
    }
    cnt = c->cnt + tcu_ticks(tcu_rate(s, c->tcsr), c->base_ns);
    if (c->tdfr && cnt >= c->tdfr) {
        /* reached full: flag it (and half, passed on the way), from zero */
        s->tfr |= 1u << n;
        if (c->tdhr < c->tdfr) {
            s->tfr |= 1u << (n + 16);
        }
        period = c->tdfr;
        c->cnt = cnt % period;
        c->base_ns = tcu_now();
        cnt = c->cnt;
    }
    if (cnt >= c->tdhr) {
        s->tfr |= 1u << (n + 16);
    }
    return cnt;
}

/*
 * Arm channel @n's timer for its next compare: half while below TDHR,
 * else full.  A TDFR of 0 would compare on every tick; nothing programs
 * that, and it is left to the lazy reads.
 */
static void ch_arm(IngenicTcuState *s, int n)
{
    TcuChannel *c = &s->ch[n];
    uint64_t rate = tcu_rate(s, c->tcsr);
    uint32_t cnt, target;

    if (!(s->ter & (1u << n)) || (s->tsr & (1u << n)) || !rate || !c->tdfr) {
        timer_del(c->timer);
        return;
    }
    cnt = ch_count(s, n);
    target = (cnt < c->tdhr && c->tdhr < c->tdfr) ? c->tdhr : c->tdfr;
    /* c->cnt is the count at c->base_ns; round up to the tick that hits */
    timer_mod(c->timer, c->base_ns +
              muldiv64(target - c->cnt, NANOSECONDS_PER_SECOND, rate) + 1);
}

static void tcu_update_irq(IngenicTcuState *s);

static void ch_timer_cb(void *opaque)
{
    TcuChannel *c = opaque;
    IngenicTcuState *s = c->tcu;
    int n = c - s->ch;

    ch_count(s, n);
    tcu_update_irq(s);
    ch_arm(s, n);
}

static void ch_arm_all(IngenicTcuState *s)
{
    int n;

    for (n = 0; n < NCH; n++) {
        ch_arm(s, n);
    }
}

static void ch_latch_all(IngenicTcuState *s)
{
    int n;

    for (n = 0; n < NCH; n++) {
        s->ch[n].cnt = ch_count(s, n);
        s->ch[n].base_ns = tcu_now();
    }
}

static void tcu_update_irq(IngenicTcuState *s)
{
    uint32_t p = s->tfr & ~s->tmr;

    /* TCU0: channel 0; TCU1: channel 1; TCU2: the rest */
    qemu_set_irq(s->irq[0], (p & 0x00010001) != 0);
    qemu_set_irq(s->irq[1], (p & 0x00020002) != 0);
    qemu_set_irq(s->irq[2], (p & 0x00fc00fc) != 0);
}

static void tcu_get_pwm(Object *obj, Visitor *v, const char *name,
                        void *opaque, Error **errp)
{
    IngenicTcuState *s = INGENIC_TCU(obj);
    int n = (uintptr_t)opaque;
    TcuChannel *c = &s->ch[n];
    bool initl = c->tcsr & (1u << 8);
    bool counting = (s->ter & (1u << n)) && !(s->tsr & (1u << n));
    uint32_t high;
    g_autofree char *str = NULL;

    if (!(c->tcsr & (1u << 7))) {
        str = g_strdup("off");
    } else if (!counting || (c->tcsr & (1u << 9)) || c->tdhr >= c->tdfr) {
        /* stopped, shut down, or never reaching half: the initial level */
        str = g_strdup(initl ? "1" : "0");
    } else if (!c->tdhr) {
        str = g_strdup(initl ? "0" : "1");  /* flipped from the start */
    } else {
        high = initl ? c->tdhr : c->tdfr - c->tdhr;
        str = g_strdup_printf("%u/%u", high, c->tdfr);
    }
    visit_type_str(v, name, &str, errp);
}

/* --- OST ---------------------------------------------------------------- */

static uint64_t ost_count(IngenicTcuState *s)
{
    if (!(s->ter & TER_OST)) {
        return s->ostcnt;
    }
    return s->ostcnt + tcu_ticks(tcu_rate(s, s->ostcsr), s->ostbase_ns);
}

static void ost_latch(IngenicTcuState *s)
{
    s->ostcnt = ost_count(s);
    s->ostbase_ns = tcu_now();
}

/* --- registers ---------------------------------------------------------- */

static uint64_t ingenic_tcu_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicTcuState *s = opaque;
    uint64_t c;
    int n;

    if (off >= R_CH0 && off < R_CH_END) {
        n = (off - R_CH0) / 0x10;
        switch (off & 0xc) {
        case 0x0:
            return s->ch[n].tdfr;
        case 0x4:
            return s->ch[n].tdhr;
        case 0x8:
            return ch_count(s, n);
        default:
            return s->ch[n].tcsr;
        }
    }
    switch (off & ~3) {
    case R_TDR:
        return s->tdr;
    case R_TCER:
        return s->tcer;
    case R_TCNT:
        return wdt_count(s);
    case R_TCSR:
        return s->tcsr;
    case R_TER:
        return s->ter;
    case R_TSR:
        return s->tsr;
    case R_TFR:
        for (n = 0; n < NCH; n++) {
            ch_count(s, n);
        }
        tcu_update_irq(s);
        return s->tfr;
    case R_TMR:
        return s->tmr;
    case R_TSTR:
        return s->tstr;
    case R_OSTDR:
        return s->ostdr;
    case R_OSTCNTL:
        c = ost_count(s);
        s->osthbuf = c >> 32;
        return (uint32_t)c;
    case R_OSTCNTH:
        return ost_count(s) >> 32;
    case R_OSTCSR:
        return s->ostcsr;
    case R_OSTHBUF:
        return s->osthbuf;
    default:
        return 0;
    }
}

static void ingenic_tcu_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicTcuState *s = opaque;
    int n;

    if (off >= R_CH0 && off < R_CH_END) {
        n = (off - R_CH0) / 0x10;
        ch_latch_all(s);
        switch (off & 0xc) {
        case 0x0:
            s->ch[n].tdfr = val & 0xffff;
            break;
        case 0x4:
            s->ch[n].tdhr = val & 0xffff;
            break;
        case 0x8:
            s->ch[n].cnt = val & 0xffff;
            break;
        default:
            s->ch[n].tcsr = val;
            break;
        }
        ch_arm_all(s);
        tcu_update_irq(s);
        return;
    }

    switch (off & ~3) {
    case R_TDR:
        wdt_latch(s);
        s->tdr = val & 0xffff;
        break;
    case R_TCER:
        wdt_latch(s);
        s->tcer = val & 1;
        break;
    case R_TCNT:
        s->wcnt = val & 0xffff;
        s->wbase_ns = tcu_now();
        break;
    case R_TCSR:
        wdt_latch(s);
        if (s->xburst2 && (val & (1u << 10))) {
            s->wcnt = 0;                /* counter clear */
            val &= ~(1u << 10);
        }
        s->tcsr = val;
        break;
    case R_TESR:
    case R_TECR:
    case R_TER:
        ch_latch_all(s);
        ost_latch(s);
        if ((off & ~3) == R_TESR) {
            s->ter |= val;
        } else if ((off & ~3) == R_TECR) {
            s->ter &= ~val;
        } else {
            s->ter = val;
        }
        break;
    case R_TSSR:
        wdt_latch(s);
        ch_latch_all(s);
        s->tsr |= val;
        break;
    case R_TSCR:
        wdt_latch(s);
        ch_latch_all(s);
        s->tsr &= ~val;
        break;
    case R_TFR:
        s->tfr = val;
        break;
    case R_TFSR:
        s->tfr |= val;
        break;
    case R_TFCR:
        s->tfr &= ~val;
        break;
    case R_TMR:
        s->tmr = val;
        break;
    case R_TMSR:
        s->tmr |= val;
        break;
    case R_TMCR:
        s->tmr &= ~val;
        break;
    case R_TSTR:
        s->tstr = val;
        break;
    case R_TSTSR:
        s->tstr |= val;
        break;
    case R_TSTCR:
        s->tstr &= ~val;
        break;
    case R_OSTDR:
        s->ostdr = val;
        break;
    case R_OSTCNTL:
        ost_latch(s);
        s->ostcnt = deposit64(s->ostcnt, 0, 32, val);
        break;
    case R_OSTCNTH:
        ost_latch(s);
        s->ostcnt = deposit64(s->ostcnt, 32, 32, val);
        break;
    case R_OSTCSR:
        ost_latch(s);
        s->ostcsr = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ingenic-tcu: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", off, val);
        return;
    }
    wdt_arm(s);
    ch_arm_all(s);
    tcu_update_irq(s);
}

static const MemoryRegionOps ingenic_tcu_ops = {
    .read = ingenic_tcu_read,
    .write = ingenic_tcu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void ingenic_tcu_reset(DeviceState *dev)
{
    IngenicTcuState *s = INGENIC_TCU(dev);
    int64_t now = tcu_now();
    int n;

    s->tdr = s->tcer = s->tcsr = s->wcnt = 0;
    s->wbase_ns = now;
    s->ter = s->tfr = s->tstr = 0;
    s->tsr = 0;
    s->tmr = 0x00ff80ff;
    for (n = 0; n < NCH; n++) {
        TcuChannel *c = &s->ch[n];

        timer_del(c->timer);
        c->tdfr = c->tdhr = c->tcsr = c->cnt = 0;
        c->base_ns = now;
    }
    s->ostdr = s->ostcsr = s->osthbuf = 0;
    s->ostcnt = 0;
    s->ostbase_ns = now;
    timer_del(s->wdt_timer);
    tcu_update_irq(s);
}

static void ingenic_tcu_init(Object *obj)
{
    IngenicTcuState *s = INGENIC_TCU(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &ingenic_tcu_ops, s,
                          TYPE_INGENIC_TCU, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (i = 0; i < 3; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    s->wdt_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wdt_timer_cb, s);
    for (i = 0; i < NCH; i++) {
        s->ch[i].tcu = s;
        s->ch[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ch_timer_cb,
                                      &s->ch[i]);
    }
    for (i = 0; i < NCH; i++) {
        g_autofree char *name = g_strdup_printf("pwm%d", i);

        object_property_add(obj, name, "str", tcu_get_pwm, NULL, NULL,
                            (void *)(uintptr_t)i);
    }
}

static const Property ingenic_tcu_props[] = {
    DEFINE_PROP_UINT32("extal-hz", IngenicTcuState, extal_hz, 24000000),
    DEFINE_PROP_UINT32("pclk-hz", IngenicTcuState, pclk_hz, 125000000),
    DEFINE_PROP_BOOL("xburst2", IngenicTcuState, xburst2, false),
};

static const VMStateDescription vmstate_tcu_channel = {
    .name = "ingenic-tcu-channel",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(tdfr, TcuChannel),
        VMSTATE_UINT32(tdhr, TcuChannel),
        VMSTATE_UINT32(tcsr, TcuChannel),
        VMSTATE_UINT32(cnt, TcuChannel),
        VMSTATE_INT64(base_ns, TcuChannel),
        VMSTATE_TIMER_PTR(timer, TcuChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_tcu = {
    .name = TYPE_INGENIC_TCU,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(tdr, IngenicTcuState),
        VMSTATE_UINT32(tcer, IngenicTcuState),
        VMSTATE_UINT32(tcsr, IngenicTcuState),
        VMSTATE_UINT32(wcnt, IngenicTcuState),
        VMSTATE_INT64(wbase_ns, IngenicTcuState),
        VMSTATE_UINT32(ter, IngenicTcuState),
        VMSTATE_UINT32(tsr, IngenicTcuState),
        VMSTATE_UINT32(tfr, IngenicTcuState),
        VMSTATE_UINT32(tmr, IngenicTcuState),
        VMSTATE_UINT32(tstr, IngenicTcuState),
        VMSTATE_STRUCT_ARRAY(ch, IngenicTcuState, NCH, 2,
                             vmstate_tcu_channel, TcuChannel),
        VMSTATE_UINT32(ostdr, IngenicTcuState),
        VMSTATE_UINT32(ostcsr, IngenicTcuState),
        VMSTATE_UINT32(osthbuf, IngenicTcuState),
        VMSTATE_UINT64(ostcnt, IngenicTcuState),
        VMSTATE_INT64(ostbase_ns, IngenicTcuState),
        VMSTATE_TIMER_PTR(wdt_timer, IngenicTcuState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_tcu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_tcu_reset);
    device_class_set_props(dc, ingenic_tcu_props);
    dc->vmsd = &vmstate_ingenic_tcu;
}

static const TypeInfo ingenic_tcu_info = {
    .name = TYPE_INGENIC_TCU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicTcuState),
    .instance_init = ingenic_tcu_init,
    .class_init = ingenic_tcu_class_init,
};

static void ingenic_tcu_register_types(void)
{
    type_register_static(&ingenic_tcu_info);
}

type_init(ingenic_tcu_register_types)
