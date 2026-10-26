/*
 * Grain Media FTWDT010 watchdog.
 *
 * Faraday's watchdog at 0x90E00000 on the GM813x parts
 * (drivers/watchdog/ftwdt010_wdt.c, the platform's arch_reset() and
 * U-Boot's reset_cpu in board/gm/gm8136/lowlevel_init.S):
 *
 *   0x00 counter (read only)   0x04 load   0x08 restart (0x5AB9 reloads)
 *   0x0C control: 0 enable, 1 system reset, 2 interrupt, 3 external
 *        signal, 4 count the 30 MHz oscillator instead of PCLK
 *   0x10 status (1 = timed out)   0x14 clear status   0x18 pulse length
 *
 * Both reboot paths load 0 or 1, restart and enable with the reset bit,
 * so the machine resets on the next tick; the kernel's /dev/watchdog
 * counts reset_secs * 30 MHz on the oscillator.  PCLK comes from the
 * PMU's "pclk" clock.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/clock.h"
#include "hw/ptimer.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/watchdog.h"
#include "qom/object.h"

#define TYPE_GM_WDT "gm-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(GmWdtState, GM_WDT)

#define GM_WDT_SIZE         0x1000
#define GM_WDT_MAGIC        0x5AB9

#define R_COUNTER           0x00
#define R_LOAD              0x04
#define R_RESTART           0x08
#define R_CR                0x0C
#define R_STATUS            0x10
#define R_CLEAR             0x14
#define R_INTRLEN           0x18
#define R_REVISION          0x1C

#define CR_EN               (1u << 0)
#define CR_RST              (1u << 1)
#define CR_INT              (1u << 2)
#define CR_EXTCLK           (1u << 4)

struct GmWdtState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    ptimer_state *pt;

    uint32_t load;
    uint32_t cr;
    uint32_t status;
    uint32_t intrlen;
    uint32_t ext_freq;
};

/* Called inside a ptimer transaction. */
static void gm_wdt_set_rate(GmWdtState *s)
{
    if ((s->cr & CR_EXTCLK) || !clock_has_source(s->clk) ||
        !clock_get_hz(s->clk)) {
        ptimer_set_freq(s->pt, s->ext_freq);
    } else {
        ptimer_set_period_from_clock(s->pt, s->clk, 1);
    }
}

static void gm_wdt_clk_update(void *opaque, ClockEvent event)
{
    GmWdtState *s = opaque;

    ptimer_transaction_begin(s->pt);
    gm_wdt_set_rate(s);
    ptimer_transaction_commit(s->pt);
}

static void gm_wdt_update(GmWdtState *s)
{
    qemu_set_irq(s->irq, s->status && (s->cr & CR_INT));
}

static void gm_wdt_expired(void *opaque)
{
    GmWdtState *s = opaque;

    s->status = 1;
    gm_wdt_update(s);
    if (s->cr & CR_RST) {
        qemu_log_mask(CPU_LOG_RESET, "gm-wdt: watchdog reset\n");
        watchdog_perform_action();
    }
}

/* Called inside a ptimer transaction. */
static void gm_wdt_reload(GmWdtState *s)
{
    ptimer_stop(s->pt);
    ptimer_set_count(s->pt, s->load);
    if (s->cr & CR_EN) {
        ptimer_run(s->pt, 1);
    }
}

static uint64_t gm_wdt_read(void *opaque, hwaddr off, unsigned size)
{
    GmWdtState *s = opaque;

    switch (off) {
    case R_COUNTER:
        return ptimer_get_count(s->pt);
    case R_LOAD:
        return s->load;
    case R_CR:
        return s->cr;
    case R_STATUS:
        return s->status;
    case R_INTRLEN:
        return s->intrlen;
    case R_REVISION:
        return 0x00010502;
    }
    return 0;
}

static void gm_wdt_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    GmWdtState *s = opaque;
    uint32_t old;

    switch (off) {
    case R_LOAD:
        s->load = val;
        break;
    case R_RESTART:
        if (val == GM_WDT_MAGIC) {
            ptimer_transaction_begin(s->pt);
            gm_wdt_reload(s);
            ptimer_transaction_commit(s->pt);
        }
        break;
    case R_CR:
        old = s->cr;
        s->cr = val & 0x1F;
        ptimer_transaction_begin(s->pt);
        gm_wdt_set_rate(s);
        if (!(s->cr & CR_EN)) {
            ptimer_stop(s->pt);
        } else if (!(old & CR_EN)) {
            if (!ptimer_get_count(s->pt)) {
                /* a zero count expires on the first tick */
                ptimer_set_count(s->pt, 1);
            }
            ptimer_run(s->pt, 1);
        }
        ptimer_transaction_commit(s->pt);
        gm_wdt_update(s);
        break;
    case R_CLEAR:
        s->status = 0;
        gm_wdt_update(s);
        break;
    case R_INTRLEN:
        s->intrlen = val & 0xFF;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, off);
        break;
    }
}

static const MemoryRegionOps gm_wdt_ops = {
    .read = gm_wdt_read,
    .write = gm_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void gm_wdt_reset(DeviceState *dev)
{
    GmWdtState *s = GM_WDT(dev);

    s->load = 0x3EF1480;        /* the documented reset value */
    s->cr = 0;
    s->status = 0;
    s->intrlen = 0xFF;
    ptimer_transaction_begin(s->pt);
    ptimer_stop(s->pt);
    ptimer_set_count(s->pt, s->load);
    gm_wdt_set_rate(s);
    ptimer_transaction_commit(s->pt);
    gm_wdt_update(s);
}

static void gm_wdt_realize(DeviceState *dev, Error **errp)
{
    GmWdtState *s = GM_WDT(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_wdt_ops, s,
                          TYPE_GM_WDT, GM_WDT_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->pt = ptimer_init(gm_wdt_expired, s,
                        PTIMER_POLICY_NO_IMMEDIATE_TRIGGER |
                        PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);
    ptimer_transaction_begin(s->pt);
    gm_wdt_set_rate(s);
    ptimer_transaction_commit(s->pt);
}

static void gm_wdt_init(Object *obj)
{
    GmWdtState *s = GM_WDT(obj);

    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", gm_wdt_clk_update, s,
                                ClockUpdate);
}

static const Property gm_wdt_props[] = {
    DEFINE_PROP_UINT32("ext-freq", GmWdtState, ext_freq, 30000000),
};

static const VMStateDescription vmstate_gm_wdt = {
    .name = TYPE_GM_WDT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, GmWdtState),
        VMSTATE_PTIMER(pt, GmWdtState),
        VMSTATE_UINT32(load, GmWdtState),
        VMSTATE_UINT32(cr, GmWdtState),
        VMSTATE_UINT32(status, GmWdtState),
        VMSTATE_UINT32(intrlen, GmWdtState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_wdt_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_wdt_realize;
    device_class_set_legacy_reset(dc, gm_wdt_reset);
    device_class_set_props(dc, gm_wdt_props);
    dc->vmsd = &vmstate_gm_wdt;
}

static const TypeInfo gm_wdt_info = {
    .name           = TYPE_GM_WDT,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmWdtState),
    .instance_init  = gm_wdt_init,
    .class_init     = gm_wdt_class_init,
};

static void gm_wdt_register_types(void)
{
    type_register_static(&gm_wdt_info);
}

type_init(gm_wdt_register_types)
