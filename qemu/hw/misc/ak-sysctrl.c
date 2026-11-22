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
 *   0x68        SAR-ADC1 samples, from the "ain0", "ain1" and "bat"
 *               properties: millivolts at the pin, which qom-set changes
 *               while the guest runs, so a test can cover and uncover a
 *               photocell. The default is a lit room on both inputs
 *               (3000 mV, what the PE3020-W's photocell gives in light).
 *               How a sample gets there is the chip's:
 *    - EV330 (and the default): always there, AIN0 in bits 9:0, AIN1 in
 *               19:10, BAT in 29:20, 10 bits of the 3.3 V AVCC each
 *               (mach-anycloud ak_adc1.c's EV330 reader);
 *    - EV300 ("sar-ev300"): one conversion at a time, 12 bits in 11:0
 *               against 3.0 V. 0x5C bit 0 starts it and 0x6C bit 0 says it
 *               is ready (cleared by writing 0). It reads its input only
 *               when the converter is clocked (0x0C bit 3), out of reset
 *               (0x20 bit 30), and powered (0xA0 bit 0 clear), and the
 *               input is selected (0x5C bit 5) and enabled (0xA0 bit 5):
 *               otherwise the sample is 0, which is what an EV330-style
 *               read of this silicon gets. AIN0 only, the one input whose
 *               analog enable is known (OpenIPC/builder#234).
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
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/watchdog.h"
#include "qom/object.h"
#include "system/runstate.h"

#define TYPE_AK_SYSCTRL "ak-sysctrl"
OBJECT_DECLARE_SIMPLE_TYPE(AkSysctrlState, AK_SYSCTRL)

#define AK_SYSCTRL_SIZE     0x8000
#define AK_SYSCTRL_NREGS    (0x400 / 4)
#define AK_SYSCTRL_NREGS_V1 (0x200 / 4)  /* migration version 1 */
#define AK_TIMER_MAX        10
#define AK_TIMER_FREQ       12000000
/* Watchdog count unit: 21.333 us (ak_top_wdt.c UNIT_NS). */
#define AK_WDT_FREQ         46875

#define R_CHIP_ID           (0x00 / 4)
#define R_CPU_PLL           (0x04 / 4)
#define R_ASIC_PLL          (0x08 / 4)
#define R_SPI0_CLK          (0x0C / 4)
#define R_HS_CLK            (0x10 / 4)
#define R_PERI_PLL1         (0x14 / 4)
#define R_PERI_PLL2         (0x18 / 4)
#define R_IRQ_MASK          (0x24 / 4)
#define R_FIQ_MASK          (0x28 / 4)
#define R_SUB_MASK          (0x2C / 4)
#define R_SUB_STATUS        (0x30 / 4)
#define R_INT_STATUS        (0x4C / 4)
#define R_RTC_CONF          (0x50 / 4)
#define R_RTC_DATA          (0x54 / 4)
#define R_SAR_IF_CFG        (0x5C / 4)
#define R_SAR_SMP_DAT       (0x68 / 4)
#define R_SAR_STATUS        (0x6C / 4)
#define R_SAR_ACR2          (0xA0 / 4)
#define R_EFUSE_CTRL        (0x8C / 4)
#define R_WDT_CFG1          (0xE4 / 4)
#define R_WDT_CFG2          (0xE8 / 4)

#define CPU_PLL_APPLY       (1u << 28)
#define EFUSE_BUSY          (3u << 30)
/*
 * EV200 / EV330 CSI clocks (kernel ak39ev330_clk.c): the PCLK divider is
 * bits 25:20, enabled by bit 28, the SCLK (sensor MCLK) divider bits 15:10,
 * enabled by bit 18.  Bits 29 and 19 are their "divider valid" bits, which
 * the hardware clears once the new divider runs.  AliOS Things on an
 * AK3918EV201 sets bit 19 and waits for it to clear.
 */
#define PERI_PLL2_APPLY     (1u << 29)
#define PERI_PLL2_SCLK_VLD  (1u << 19)
/*
 * EV300L / AV100 clock block (U-Boot clk-ak3918ev300l.h): writes that
 * take effect when the clock settles, self-clearing.
 */
#define CLK_V2_PLL_ADJ      (7u << 28)
#define CLK_V2_GCLK_VLD     (1u << 25)
#define CLK_V2_DIV_VLD      ((1u << 29) | (1u << 19) | (1u << 9))

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
    /* What the running count started from and when: the first decides
     * whether a run-out is the kernel's restart or an expiry, so it is
     * migrated (version 3); the second only times the warning. */
    uint32_t wdt_run_load;
    int64_t wdt_fed_ms;

    uint32_t chip_id;
    uint32_t chip_id_av;        /* AV130: the ID at +0x3FC, 0 = none */
    /*
     * AV130: its interrupts are another controller's (ak-av130-intc), whose
     * bank 1 this block's +0x2C mask and +0x30 status are; the IRQ output
     * is that bank's level, and +0x24 / +0x28 are unused.
     */
    bool cascade;
    uint32_t num_timers;
    bool has_efuse;
    bool clk_v2;
    uint32_t ain[3];            /* SAR-ADC1 AIN0, AIN1, BAT; mV at the pin */
    bool sar_ev300;
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
    uint32_t st;

    if (s->cascade) {
        qemu_set_irq(s->irq, !!(ak_sub_status(s) & s->regs[R_SUB_MASK]));
        return;
    }
    st = ak_int_status(s);
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

/*
 * A count this short is the kernel restarting the board, not a watchdog
 * that went unfed: mach-ak39ev330.c's restart hook is wdt_enable() plus
 * wdt_keepalive(2), two counts (43 us). Below 1 ms the reset is a plain
 * reboot, as it always was; anything longer is reported, because on this
 * machine the watchdog is the only thing that resets the board and an
 * expiry would otherwise look exactly like a `reboot`.
 */
#define AK_WDT_RESTART_COUNTS   (AK_WDT_FREQ / 1000)

/* SAR-ADC1 (see the register list at the top). */
#define SAR_EV330_AVCC_MV   3300
#define SAR_EV300_VREF_MV   3000
#define SAR_CLK_EN          (1u << 3)       /* 0x0C */
#define SAR_RESET_RELEASE   (1u << 30)      /* 0x20 */
#define SAR_CFG_START       (1u << 0)       /* 0x5C */
#define SAR_CFG_AIN0        (1u << 5)
#define SAR_STATUS_READY    (1u << 0)       /* 0x6C */
#define SAR_ACR2_POWER_DOWN (1u << 0)       /* 0xA0 */
#define SAR_ACR2_AIN0       (1u << 5)
#define R_RESET_CTRL        (0x20 / 4)

static uint32_t ak_sar_scale(uint32_t mv, uint32_t ref_mv, unsigned bits)
{
    uint32_t full = (1u << bits) - 1;
    uint64_t raw = (uint64_t)mv << bits;

    raw /= ref_mv;
    return raw > full ? full : raw;
}

static uint32_t ak_sar_ev330(uint32_t mv)
{
    return ak_sar_scale(mv, SAR_EV330_AVCC_MV, 10);
}

static void ak_sar_ev300_convert(AkSysctrlState *s)
{
    uint32_t acr2 = s->regs[R_SAR_ACR2];
    bool live = (s->regs[R_SPI0_CLK] & SAR_CLK_EN) &&
                (s->regs[R_RESET_CTRL] & SAR_RESET_RELEASE) &&
                !(acr2 & SAR_ACR2_POWER_DOWN);
    uint32_t sample = 0;

    if (live && (s->regs[R_SAR_IF_CFG] & SAR_CFG_AIN0) &&
        (acr2 & SAR_ACR2_AIN0)) {
        sample = ak_sar_scale(s->ain[0], SAR_EV300_VREF_MV, 12);
    }
    /* The conversion is quick next to the driver's 250 us poll: done by the
     * time anyone looks. */
    s->regs[R_SAR_SMP_DAT] = sample;
    s->regs[R_SAR_STATUS] |= SAR_STATUS_READY;
}

static void ak_wdt_expire(void *opaque)
{
    AkSysctrlState *s = opaque;

    if (s->wdt_run_load < AK_WDT_RESTART_COUNTS) {
        qemu_log_mask(LOG_GUEST_ERROR, "ak-sysctrl: watchdog reset "
                      "(restart)\n");
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return;
    }
    warn_report("ak-sysctrl: watchdog expired: last fed %.3f s ago with "
                "a %.3f s timeout",
                (qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) - s->wdt_fed_ms) /
                    1000.0,
                (double)s->wdt_run_load / AK_WDT_FREQ);
    /* Reset by default; -action watchdog=pause|none|... applies. */
    watchdog_perform_action();
}

static void ak_wdt_write_cfg2(AkSysctrlState *s, uint32_t val)
{
    ptimer_transaction_begin(s->wdt);
    if (!(val & WDT_EN)) {
        ptimer_stop(s->wdt);
    } else if (val & WDT_FEED) {
        s->wdt_run_load = MAX(s->wdt_load, 1);
        s->wdt_fed_ms = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
        ptimer_set_limit(s->wdt, s->wdt_run_load, 1);
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
    case 0x3FC / 4:
        if (s->chip_id_av) {
            return s->chip_id_av;
        }
        return s->regs[offset / 4];
    case R_SUB_STATUS:
        return ak_sub_status(s);
    case R_INT_STATUS:
        return ak_int_status(s);
    case R_SAR_SMP_DAT:
        if (s->sar_ev300) {
            return s->regs[R_SAR_SMP_DAT];
        }
        return ak_sar_ev330(s->ain[0]) | ak_sar_ev330(s->ain[1]) << 10 |
               ak_sar_ev330(s->ain[2]) << 20;
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
        /*
         * The new PLL setting takes at once: the apply bit reads back 0.
         * EV300L / AV100: bits 30 / 29 / 28 apply PLL1 / PLL3 / PLL2.
         */
        s->regs[R_CPU_PLL] = val & ~(s->clk_v2 ? CLK_V2_PLL_ADJ
                                                : CPU_PLL_APPLY);
        return;
    case R_ASIC_PLL:
        s->regs[R_ASIC_PLL] = s->clk_v2 ? val & ~CLK_V2_GCLK_VLD : val;
        return;
    case R_SPI0_CLK:
    case R_HS_CLK:
        /* EV300L: SD ADC / SPI0 / SD DAC divider "valid" bits */
        s->regs[offset / 4] = s->clk_v2 ? val & ~CLK_V2_DIV_VLD : val;
        return;
    case R_PERI_PLL2:
        if (s->clk_v2) {
            /* EV300L: CSI / MAC_OPCLK dividers; bit 9 is MAC's "valid" */
            s->regs[R_PERI_PLL2] = val & ~(1u << 9);
            return;
        }
        s->regs[R_PERI_PLL2] = val & ~(PERI_PLL2_APPLY | PERI_PLL2_SCLK_VLD);
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
    case R_SAR_IF_CFG:
        s->regs[R_SAR_IF_CFG] = val;
        if (s->sar_ev300 && (val & SAR_CFG_START)) {
            ak_sar_ev300_convert(s);
        }
        return;
    case R_SAR_SMP_DAT:
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
    if (s->clk_v2) {
        /*
         * EV300L: 24 MHz * (M + 1) / (N + 1) / (OD + 1), M 12:0, N 18:13,
         * OD 22:19.  PLL1 (CPU) 888 MHz is what the boot header script
         * writes; PLL2 240 MHz with GCLK = PLL2 / 2(23:24 + 1) = 60 MHz
         * is the reset default the clock driver's comment gives (U-Boot
         * then sets 400 / 200 MHz).
         */
        s->regs[R_CPU_PLL] = 0x00000024;
        s->regs[R_ASIC_PLL] = 0x00980027;
    } else {
        s->regs[R_CPU_PLL] = 0x011253C8;
        s->regs[R_ASIC_PLL] = 0x018213C8;
        s->regs[R_PERI_PLL1] = 0x000012C8;
    }
    for (n = 0; n < AK_TIMER_MAX; n++) {
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
    s->wdt_run_load = 0;
    s->wdt_fed_ms = 0;

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
    ak_sysctrl_update(s);
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
    /*
     * All AK_TIMER_MAX, though only num_timers decode: the VMState saves
     * the whole array, and a NULL ptimer there trips its save path.
     */
    for (n = 0; n < AK_TIMER_MAX; n++) {
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

    /* Not qdev properties: those cannot be set once the device is realized,
     * and these are the outside world, which changes while the guest runs. */
    s->ain[0] = s->ain[1] = 3000;
    object_property_add_uint32_ptr(obj, "ain0", &s->ain[0],
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "ain1", &s->ain[1],
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "bat", &s->ain[2],
                                   OBJ_PROP_FLAG_READWRITE);
}

static const Property ak_sysctrl_properties[] = {
    DEFINE_PROP_UINT32("chip-id", AkSysctrlState, chip_id, 0x20160100),
    DEFINE_PROP_UINT32("chip-id-av", AkSysctrlState, chip_id_av, 0),
    DEFINE_PROP_BOOL("cascade", AkSysctrlState, cascade, false),
    DEFINE_PROP_UINT32("num-timers", AkSysctrlState, num_timers, 5),
    DEFINE_PROP_BOOL("has-efuse", AkSysctrlState, has_efuse, false),
    DEFINE_PROP_BOOL("clk-v2", AkSysctrlState, clk_v2, false),
    DEFINE_PROP_BOOL("sar-ev300", AkSysctrlState, sar_ev300, false),
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

static int ak_sysctrl_post_load(void *opaque, int version_id)
{
    AkSysctrlState *s = opaque;

    /*
     * An older stream does not say what the running count started from.
     * The last load written is the best guess, and keeps a long timeout an
     * expiry (with the configured watchdog action) rather than a reboot.
     */
    if (version_id < 3) {
        s->wdt_run_load = s->wdt_load;
    }
    ak_sysctrl_update(opaque);
    return 0;
}

static const VMStateDescription vmstate_ak_sysctrl = {
    .name = TYPE_AK_SYSCTRL,
    .version_id = 3,
    .minimum_version_id = 1,
    .post_load = ak_sysctrl_post_load,
    .fields = (const VMStateField[]) {
        /*
         * Version 1 had 0x200 bytes of registers; the EV300L's pad
         * registers run past them. A version-1 stream leaves the upper
         * half at its reset value.
         */
        VMSTATE_SUB_ARRAY(regs, AkSysctrlState, 0, AK_SYSCTRL_NREGS_V1, 0,
                          vmstate_info_uint32, uint32_t),
        VMSTATE_UINT32(level, AkSysctrlState),
        VMSTATE_STRUCT_ARRAY(timer, AkSysctrlState, AK_TIMER_MAX, 1,
                             vmstate_ak_timer, AkTimer),
        VMSTATE_UINT16_ARRAY(rtc, AkSysctrlState, RTC_NREGS),
        VMSTATE_PTIMER(wdt, AkSysctrlState),
        VMSTATE_UINT32(wdt_load, AkSysctrlState),
        VMSTATE_SUB_ARRAY(regs, AkSysctrlState, AK_SYSCTRL_NREGS_V1,
                          AK_SYSCTRL_NREGS - AK_SYSCTRL_NREGS_V1, 2,
                          vmstate_info_uint32, uint32_t),
        VMSTATE_UINT32_V(wdt_run_load, AkSysctrlState, 3),
        VMSTATE_INT64_V(wdt_fed_ms, AkSysctrlState, 3),
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
