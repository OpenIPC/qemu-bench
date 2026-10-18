/*
 * Anyka AK39xx system control module.
 *
 * One 32 KiB block at 0x08000000 holds most of what is "the SoC" on these
 * parts, and the vendor code (U-Boot 2013.10 arch-ak39, the 3.4.35 kernel's
 * mach-ak39, the 4.4 kernel's mach-anycloud) reaches all of it directly:
 *
 *   0x00        chip ID               0x04..0x18  CPU / ASIC / peri PLLs
 *   0x1C        clock gates (1 = off) 0x20        soft resets
 *   0x24 / 0x28 IRQ / FIQ enable of the 32 module interrupts
 *   0x2C / 0x30 enable / status of the "system" sub-interrupts, which the
 *               module controller sees as its input 3
 *   0x4C        module interrupt status (level, before the enables)
 *   0x50 / 0x54 RTC command / data (the RTC sits behind a serial link)
 *   0xB4..0xD8  timers 1..5, a CTRL1 / CTRL2 pair each
 *   0xE4 / 0xE8 watchdog           0xF4..0x118  timers 6..10 (EV330 only)
 *   0x8C        EV330 eFuse read: bit 31 starts and is busy, 30 also busy;
 *               the fuse words come back in 0x90 / 0x94 (all blank here)
 *
 * Interrupt numbering (mach/irqs.h, identical on EV200 / EV300 / EV330):
 * the module bit n of 0x24 / 0x4C is source n; sub-interrupt bit 0 is the
 * SAR ADC, bits 5..1 timers 1..5, bit 7 RTC ready, bits 15..11 timers
 * 6..10.
 *
 * Timer CTRL2 (drivers/clocksource/ak_timer.c): bit 30 clears the expired
 * flag, 29 reloads the count from CTRL1, 28 enables, 27 is the expired
 * flag, 26 makes CTRL1 read the live count, 25:24 the mode (0 auto-reload,
 * 1 one-shot, 2 PWM).  The count runs down at the 12 MHz crystal.
 *
 * Everything else is plain storage, reset to values that make the vendor
 * clock code compute sane rates (it panics on a PLL it cannot decode).
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/runstate.h"

#define TYPE_AK_SYSCTRL "ak-sysctrl"
OBJECT_DECLARE_SIMPLE_TYPE(AkSysctrlState, AK_SYSCTRL)

#define AK_SYSCTRL_SIZE     0x8000
#define AK_SYSCTRL_NREGS    (0x200 / 4)
#define AK_TIMER_MAX        10
#define AK_TIMER_FREQ       12000000
/* Watchdog count unit: 21.333 us (ak_top_wdt.c UNIT_NS). */
#define AK_WDT_FREQ         46875

#define R_CHIP_ID           (0x00 / 4)
#define R_CPU_PLL           (0x04 / 4)
#define R_ASIC_PLL          (0x08 / 4)
#define R_PERI_PLL1         (0x14 / 4)
#define R_PERI_PLL2         (0x18 / 4)
#define R_IRQ_MASK          (0x24 / 4)
#define R_FIQ_MASK          (0x28 / 4)
#define R_SUB_MASK          (0x2C / 4)
#define R_SUB_STATUS        (0x30 / 4)
#define R_INT_STATUS        (0x4C / 4)
#define R_RTC_CONF          (0x50 / 4)
#define R_RTC_DATA          (0x54 / 4)
#define R_EFUSE_CTRL        (0x8C / 4)
#define R_WDT_CFG1          (0xE4 / 4)
#define R_WDT_CFG2          (0xE8 / 4)

#define CPU_PLL_APPLY       (1u << 28)
#define EFUSE_BUSY          (3u << 30)
#define PERI_PLL2_APPLY     (1u << 29)

#define IRQ_SYSCTRL         3

#define SUB_RTC_READY       (1u << 7)

#define TMR_CLEAR           (1u << 30)
#define TMR_FEED            (1u << 29)
#define TMR_EN              (1u << 28)
#define TMR_STATUS          (1u << 27)
#define TMR_READ_SEL        (1u << 26)
#define TMR_MODE(v)         (((v) >> 24) & 3)
#define TMR_MODE_ONESHOT    1

/*
 * RTC_CONF: bit 21 starts a transfer, bit 17 says it is a read, 16:14 the
 * RTC register, 13:0 the data to write.
 */
#define RTC_CONF_GO         (1u << 21)
#define RTC_CONF_READ       (1u << 17)
#define RTC_NREGS           8
#define RTC_SETTING         7
/*
 * RTC_SETTING bits the RTC clears once it has acted on them (the real-time
 * read / write latches, the WAKEUP pin pulses).
 */
#define RTC_SETTING_SELFCLR ((1u << 3) | (1u << 4) | (1u << 7) | (1u << 8))

#define WDT_CFG1_KEY        0x55
#define WDT_CFG2_KEY        0xAA
#define WDT_EN              (1u << 0)
#define WDT_FEED            (1u << 1)

typedef struct AkTimer {
    struct AkSysctrlState *parent;
    ptimer_state *ptimer;
    uint32_t load;
    uint32_t ctrl2;
} AkTimer;

struct AkSysctrlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq fiq;

    uint32_t regs[AK_SYSCTRL_NREGS];
    uint32_t level;             /* module interrupt inputs */
    AkTimer timer[AK_TIMER_MAX];
    uint16_t rtc[RTC_NREGS];
    ptimer_state *wdt;
    uint32_t wdt_load;

    uint32_t chip_id;
    uint32_t num_timers;
    bool has_efuse;
};

/* CTRL1 / CTRL2 offsets of timer n (1-based in the vendor's naming). */
static hwaddr ak_timer_offset(int n)
{
    return n < 5 ? 0xB4 + 8 * n : 0xF4 + 8 * (n - 5);
}

/* Sub-interrupt status bit of timer n (0-based). */
static uint32_t ak_timer_sub_bit(int n)
{
    return 1u << (n < 5 ? 5 - n : 20 - n);
}

static uint32_t ak_sub_status(AkSysctrlState *s)
{
    uint32_t st = s->regs[R_SUB_STATUS] | SUB_RTC_READY;
    int n;

    for (n = 0; n < s->num_timers; n++) {
        if (s->timer[n].ctrl2 & TMR_STATUS) {
            st |= ak_timer_sub_bit(n);
        }
    }
    return st;
}

static uint32_t ak_int_status(AkSysctrlState *s)
{
    uint32_t st = s->level & ~(1u << IRQ_SYSCTRL);

    if (ak_sub_status(s) & s->regs[R_SUB_MASK]) {
        st |= 1u << IRQ_SYSCTRL;
    }
    return st;
}

static void ak_sysctrl_update(AkSysctrlState *s)
{
    uint32_t st = ak_int_status(s);

    qemu_set_irq(s->irq, !!(st & s->regs[R_IRQ_MASK]));
    qemu_set_irq(s->fiq, !!(st & s->regs[R_FIQ_MASK]));
}

static void ak_sysctrl_set_irq(void *opaque, int n, int level)
{
    AkSysctrlState *s = opaque;

    if (level) {
        s->level |= 1u << n;
    } else {
        s->level &= ~(1u << n);
    }
    ak_sysctrl_update(s);
}

static void ak_timer_tick(void *opaque)
{
    AkTimer *t = opaque;

    t->ctrl2 |= TMR_STATUS;
    if (TMR_MODE(t->ctrl2) == TMR_MODE_ONESHOT) {
        ptimer_stop(t->ptimer);
    }
    ak_sysctrl_update(t->parent);
}

/* (Re)start the count from CTRL1, as a write of FEED does. */
static void ak_timer_start(AkTimer *t)
{
    ptimer_transaction_begin(t->ptimer);
    ptimer_stop(t->ptimer);
    if (t->ctrl2 & TMR_EN) {
        ptimer_set_limit(t->ptimer, t->load, 1);
        ptimer_run(t->ptimer, TMR_MODE(t->ctrl2) == TMR_MODE_ONESHOT);
    }
    ptimer_transaction_commit(t->ptimer);
}

static void ak_timer_write_ctrl2(AkTimer *t, uint32_t val)
{
    uint32_t old = t->ctrl2;

    /* The expired flag is the timer's; CLEAR and FEED are commands. */
    t->ctrl2 = (val & ~(TMR_CLEAR | TMR_FEED | TMR_STATUS)) |
               (old & TMR_STATUS);
    if (val & TMR_CLEAR) {
        t->ctrl2 &= ~TMR_STATUS;
    }
    if ((val & TMR_FEED) || ((val ^ old) & TMR_EN)) {
        ak_timer_start(t);
    }
}

static void ak_wdt_expire(void *opaque)
{
    qemu_log_mask(LOG_GUEST_ERROR, "ak-sysctrl: watchdog reset\n");
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void ak_wdt_write_cfg2(AkSysctrlState *s, uint32_t val)
{
    ptimer_transaction_begin(s->wdt);
    if (!(val & WDT_EN)) {
        ptimer_stop(s->wdt);
    } else if (val & WDT_FEED) {
        ptimer_set_limit(s->wdt, MAX(s->wdt_load, 1), 1);
        ptimer_run(s->wdt, 1);
    }
    ptimer_transaction_commit(s->wdt);
}

static void ak_rtc_command(AkSysctrlState *s, uint32_t val)
{
    unsigned reg = (val >> 14) & 7;

    if (!(val & RTC_CONF_GO)) {
        return;
    }
    if (val & RTC_CONF_READ) {
        s->regs[R_RTC_DATA] = s->rtc[reg];
        if (reg == RTC_SETTING) {
            s->rtc[reg] &= ~RTC_SETTING_SELFCLR;
        }
    } else {
        s->rtc[reg] = val & 0x3FFF;
    }
}

static uint64_t ak_sysctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    AkSysctrlState *s = opaque;
    int n;

    for (n = 0; n < s->num_timers; n++) {
        AkTimer *t = &s->timer[n];
        hwaddr base = ak_timer_offset(n);

        if (offset == base) {
            if (t->ctrl2 & TMR_READ_SEL) {
                return ptimer_get_count(t->ptimer);
            }
            return t->load;
        }
        if (offset == base + 4) {
            return t->ctrl2;
        }
    }
    if (offset >= AK_SYSCTRL_NREGS * 4) {
        qemu_log_mask(LOG_UNIMP, "ak-sysctrl: read of 0x%04" HWADDR_PRIx
                      "\n", offset);
        return 0;
    }
    switch (offset / 4) {
    case R_CHIP_ID:
        return s->chip_id;
    case R_SUB_STATUS:
        return ak_sub_status(s);
    case R_INT_STATUS:
        return ak_int_status(s);
    default:
        return s->regs[offset / 4];
    }
}

static void ak_sysctrl_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    AkSysctrlState *s = opaque;
    int n;

    for (n = 0; n < s->num_timers; n++) {
        AkTimer *t = &s->timer[n];
        hwaddr base = ak_timer_offset(n);

        if (offset == base) {
            t->load = val;
            return;
        }
        if (offset == base + 4) {
            ak_timer_write_ctrl2(t, val);
            ak_sysctrl_update(s);
            return;
        }
    }
    if (offset >= AK_SYSCTRL_NREGS * 4) {
        qemu_log_mask(LOG_UNIMP, "ak-sysctrl: write 0x%08" PRIx64
                      " to 0x%04" HWADDR_PRIx "\n", val, offset);
        return;
    }
    switch (offset / 4) {
    case R_CHIP_ID:
    case R_INT_STATUS:
        return;
    case R_CPU_PLL:
        /* The new PLL setting takes at once: the apply bit reads back 0. */
        s->regs[R_CPU_PLL] = val & ~CPU_PLL_APPLY;
        return;
    case R_PERI_PLL2:
        s->regs[R_PERI_PLL2] = val & ~PERI_PLL2_APPLY;
        return;
    case R_EFUSE_CTRL:
        /* A read of the (blank) fuses is over before the next access. */
        s->regs[R_EFUSE_CTRL] = s->has_efuse ? val & ~EFUSE_BUSY : val;
        return;
    case R_SUB_STATUS:
        /* Sources the model does not drive: write 1 to clear. */
        s->regs[R_SUB_STATUS] &= ~val;
        break;
    case R_RTC_CONF:
        s->regs[R_RTC_CONF] = val & ~RTC_CONF_GO;
        ak_rtc_command(s, val);
        return;
    case R_WDT_CFG1:
        if ((val >> 24) == WDT_CFG1_KEY) {
            s->wdt_load = val & 0xFFFFFF;
            s->regs[R_WDT_CFG1] = val;
        }
        return;
    case R_WDT_CFG2:
        if ((val >> 24) == WDT_CFG2_KEY) {
            s->regs[R_WDT_CFG2] = val & ~WDT_FEED;
            ak_wdt_write_cfg2(s, val);
        }
        return;
    default:
        s->regs[offset / 4] = val;
        break;
    }
    ak_sysctrl_update(s);
}

static const MemoryRegionOps ak_sysctrl_ops = {
    .read = ak_sysctrl_read,
    .write = ak_sysctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ak_sysctrl_reset(DeviceState *dev)
{
    AkSysctrlState *s = AK_SYSCTRL(dev);
    time_t now = time(NULL);
    int n;

    memset(s->regs, 0, sizeof(s->regs));
    /*
     * 400 MHz CPU PLL, 400 MHz ASIC PLL (ASIC = /4 = 100 MHz), 600 MHz peri
     * PLL: what the boot header scripts and U-Boot program.
     */
    s->regs[R_CPU_PLL] = 0x011253C8;
    s->regs[R_ASIC_PLL] = 0x018213C8;
    s->regs[R_PERI_PLL1] = 0x000012C8;
    for (n = 0; n < s->num_timers; n++) {
        AkTimer *t = &s->timer[n];

        ptimer_transaction_begin(t->ptimer);
        ptimer_stop(t->ptimer);
        ptimer_transaction_commit(t->ptimer);
        t->load = 0;
        t->ctrl2 = 0;
    }
    ptimer_transaction_begin(s->wdt);
    ptimer_stop(s->wdt);
    ptimer_transaction_commit(s->wdt);
    s->wdt_load = 0;

    /*
     * Real time, 14 bits a register: seconds+minutes / hours+day /
     * month+year since 1980 (plat-anyka/rtc.c), so the clock is not
     * reset to 1980 on every boot.
     */
    {
        struct tm tm;

        gmtime_r(&now, &tm);
        memset(s->rtc, 0, sizeof(s->rtc));
        s->rtc[0] = (tm.tm_min << 6) | tm.tm_sec;
        s->rtc[1] = ((tm.tm_wday & 7) << 10) | (tm.tm_mday << 5) | tm.tm_hour;
        s->rtc[2] = (MAX(tm.tm_year - 80, 0) << 4) | (tm.tm_mon + 1);
    }
}

static void ak_sysctrl_realize(DeviceState *dev, Error **errp)
{
    AkSysctrlState *s = AK_SYSCTRL(dev);
    int n;

    if (s->num_timers < 5 || s->num_timers > AK_TIMER_MAX) {
        error_setg(errp, "ak-sysctrl: num-timers must be 5..%d",
                   AK_TIMER_MAX);
        return;
    }
    for (n = 0; n < s->num_timers; n++) {
        AkTimer *t = &s->timer[n];

        t->parent = s;
        t->ptimer = ptimer_init(ak_timer_tick, t,
                                PTIMER_POLICY_WRAP_AFTER_ONE_PERIOD |
                                PTIMER_POLICY_NO_IMMEDIATE_TRIGGER |
                                PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                                PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);
        ptimer_transaction_begin(t->ptimer);
        ptimer_set_freq(t->ptimer, AK_TIMER_FREQ);
        ptimer_transaction_commit(t->ptimer);
    }
    s->wdt = ptimer_init(ak_wdt_expire, s, PTIMER_POLICY_NO_IMMEDIATE_TRIGGER);
    ptimer_transaction_begin(s->wdt);
    ptimer_set_freq(s->wdt, AK_WDT_FREQ);
    ptimer_transaction_commit(s->wdt);
}

static void ak_sysctrl_init(Object *obj)
{
    AkSysctrlState *s = AK_SYSCTRL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ak_sysctrl_ops, s,
                          TYPE_AK_SYSCTRL, AK_SYSCTRL_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(DEVICE(obj), ak_sysctrl_set_irq, 32);
}

static const Property ak_sysctrl_properties[] = {
    DEFINE_PROP_UINT32("chip-id", AkSysctrlState, chip_id, 0x20160100),
    DEFINE_PROP_UINT32("num-timers", AkSysctrlState, num_timers, 5),
    DEFINE_PROP_BOOL("has-efuse", AkSysctrlState, has_efuse, false),
};

static const VMStateDescription vmstate_ak_timer = {
    .name = "ak-sysctrl-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PTIMER(ptimer, AkTimer),
        VMSTATE_UINT32(load, AkTimer),
        VMSTATE_UINT32(ctrl2, AkTimer),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ak_sysctrl = {
    .name = TYPE_AK_SYSCTRL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AkSysctrlState, AK_SYSCTRL_NREGS),
        VMSTATE_UINT32(level, AkSysctrlState),
        VMSTATE_STRUCT_ARRAY(timer, AkSysctrlState, AK_TIMER_MAX, 1,
                             vmstate_ak_timer, AkTimer),
        VMSTATE_UINT16_ARRAY(rtc, AkSysctrlState, RTC_NREGS),
        VMSTATE_PTIMER(wdt, AkSysctrlState),
        VMSTATE_UINT32(wdt_load, AkSysctrlState),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_sysctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ak_sysctrl_realize;
    dc->vmsd = &vmstate_ak_sysctrl;
    device_class_set_legacy_reset(dc, ak_sysctrl_reset);
    device_class_set_props(dc, ak_sysctrl_properties);
}

static const TypeInfo ak_sysctrl_info = {
    .name          = TYPE_AK_SYSCTRL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkSysctrlState),
    .instance_init = ak_sysctrl_init,
    .class_init    = ak_sysctrl_class_init,
};

static void ak_sysctrl_register_types(void)
{
    type_register_static(&ak_sysctrl_info);
}

type_init(ak_sysctrl_register_types)
