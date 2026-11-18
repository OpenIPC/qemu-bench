/*
 * Lotus (Xiongmai / Goke GK7206 "xmorca") watchdog block at 0x12030000.
 *
 * One 4 KiB APB window holding several ARM SP805-style watchdog cores at a
 * 0x20 stride and ONE lock register for all of them at 0xC00.  The vendor
 * DT describes two of them as separate "arm,sp805" nodes (watchdog@12030000,
 * SPI 2, and watchdog@12030020, SPI 3, both clocked from clk_24m); the SDK
 * sp805_wdt.c is patched ("struct lotus_wdt") to unlock/lock through the
 * shared 0x12030000+0xC00 instead of base+0xC00.  Stock XM DTs disable the
 * second node, OpenIPC's enables both.
 *
 * Per core n (base = n * 0x20):
 *   +0x00 LOAD    +0x04 VALUE (RO)  +0x08 CONTROL [0]=INTEN [1]=RESEN
 *   +0x0C INTCLR (WO)  +0x10 RIS (RO)  +0x14 MIS (RO)
 * Shared:  0xC00 LOCK  write 0x1ACCE551 unlocks, anything else locks;
 *          reads 1 while locked.  Core registers ignore writes when locked.
 *
 * What a running board shows (devmem, read-only): reset values LOAD/VALUE
 * 0xFFFFFFFF at 0x00, 0x20 and 0x40 (three cores; the third has no DT node
 * and no known interrupt), CONTROL 0; on stock XM firmware core 0 runs with
 * LOAD 0x2AEA53FF (the driver's 24 MHz/2 * 60 s - 1), CONTROL 3, LOCK 1,
 * VALUE dropping ~24e6 per second.  Primecell ID registers read 0 (the DT
 * supplies arm,primecell-periphid), so they read 0 here too.
 *
 * SP805 behaviour: the counter runs while INTEN is set (setting INTEN
 * reloads it from LOAD).  On reaching zero the first time it raises the
 * interrupt (RIS) and reloads; reaching zero with RIS still set asserts
 * reset if RESEN.  Writing LOAD or INTCLR reloads the counter, INTCLR also
 * clears RIS — that is the Linux driver's ping.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "system/watchdog.h"
#include "migration/vmstate.h"

#define TYPE_LOTUS_WDT "lotus-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(LotusWdtState, LOTUS_WDT)

#define LOTUS_WDT_CORES     3
#define CORE_STRIDE         0x20

#define WDT_LOAD            0x00
#define WDT_VALUE           0x04
#define WDT_CONTROL         0x08
#define WDT_INTCLR          0x0c
#define WDT_RIS             0x10
#define WDT_MIS             0x14
#define WDT_LOCK            0xc00

#define CTRL_INTEN          (1 << 0)
#define CTRL_RESEN          (1 << 1)
#define UNLOCK_KEY          0x1acce551

typedef struct LotusWdtCore {
    struct LotusWdtState *s;
    ptimer_state *timer;
    qemu_irq irq;
    uint32_t load;
    uint32_t control;
    uint32_t ris;
} LotusWdtCore;

struct LotusWdtState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    LotusWdtCore core[LOTUS_WDT_CORES];
    uint32_t locked;
    uint32_t freq;
};

static void lotus_wdt_update_irq(LotusWdtCore *c)
{
    qemu_set_irq(c->irq, c->ris && (c->control & CTRL_INTEN));
}

/* Restart the count from LOAD (a zero LOAD expires on the next tick). */
static void lotus_wdt_reload(LotusWdtCore *c)
{
    ptimer_transaction_begin(c->timer);
    ptimer_set_limit(c->timer, c->load ? c->load : 1, 1);
    if (c->control & CTRL_INTEN) {
        ptimer_run(c->timer, 0);
    } else {
        ptimer_stop(c->timer);
    }
    ptimer_transaction_commit(c->timer);
}

static void lotus_wdt_tick(void *opaque)
{
    LotusWdtCore *c = opaque;

    if (!c->ris) {
        c->ris = 1;
        lotus_wdt_update_irq(c);
        return;                 /* ptimer reloads from the limit */
    }
    if (c->control & CTRL_RESEN) {
        qemu_log_mask(CPU_LOG_RESET, "lotus-wdt: core %d timed out\n",
                      (int)(c - c->s->core));
        /*
         * WDOGRES is asserted once; don't keep re-firing until the reset
         * lands (the kernel's restart handler loads 0, a 1-tick period).
         * Called from the ptimer callback, so already inside a transaction.
         */
        ptimer_stop(c->timer);
        watchdog_perform_action();
    }
}

static uint64_t lotus_wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    LotusWdtState *s = LOTUS_WDT(opaque);
    LotusWdtCore *c;

    if (offset == WDT_LOCK) {
        return s->locked;
    }
    if (offset >= LOTUS_WDT_CORES * CORE_STRIDE) {
        return 0;               /* ITCR/ITOP/IDs: zero on the real chip */
    }
    c = &s->core[offset / CORE_STRIDE];
    switch (offset % CORE_STRIDE) {
    case WDT_LOAD:
        return c->load;
    case WDT_VALUE:
        return ptimer_get_count(c->timer);
    case WDT_CONTROL:
        return c->control;
    case WDT_RIS:
        return c->ris;
    case WDT_MIS:
        return c->ris && (c->control & CTRL_INTEN);
    default:
        return 0;
    }
}

static void lotus_wdt_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    LotusWdtState *s = LOTUS_WDT(opaque);
    LotusWdtCore *c;

    if (offset == WDT_LOCK) {
        s->locked = (val != UNLOCK_KEY);
        return;
    }
    if (offset >= LOTUS_WDT_CORES * CORE_STRIDE) {
        qemu_log_mask(LOG_GUEST_ERROR, "lotus-wdt: write to 0x%" HWADDR_PRIx
                      "\n", offset);
        return;
    }
    if (s->locked) {
        return;
    }
    c = &s->core[offset / CORE_STRIDE];
    switch (offset % CORE_STRIDE) {
    case WDT_LOAD:
        c->load = val;
        lotus_wdt_reload(c);
        break;
    case WDT_CONTROL: {
        bool was_on = c->control & CTRL_INTEN;

        c->control = val & (CTRL_INTEN | CTRL_RESEN);
        if (!was_on && (c->control & CTRL_INTEN)) {
            lotus_wdt_reload(c);
        } else if (!(c->control & CTRL_INTEN)) {
            ptimer_transaction_begin(c->timer);
            ptimer_stop(c->timer);
            ptimer_transaction_commit(c->timer);
        }
        lotus_wdt_update_irq(c);
        break;
    }
    case WDT_INTCLR:
        c->ris = 0;
        lotus_wdt_update_irq(c);
        lotus_wdt_reload(c);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "lotus-wdt: write to RO 0x%"
                      HWADDR_PRIx "\n", offset);
        break;
    }
}

static const MemoryRegionOps lotus_wdt_ops = {
    .read = lotus_wdt_read,
    .write = lotus_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void lotus_wdt_init(Object *obj)
{
    LotusWdtState *s = LOTUS_WDT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &lotus_wdt_ops, s, "lotus-wdt",
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int i = 0; i < LOTUS_WDT_CORES; i++) {
        sysbus_init_irq(sbd, &s->core[i].irq);
    }
}

static void lotus_wdt_realize(DeviceState *dev, Error **errp)
{
    LotusWdtState *s = LOTUS_WDT(dev);

    for (int i = 0; i < LOTUS_WDT_CORES; i++) {
        LotusWdtCore *c = &s->core[i];

        c->s = s;
        c->timer = ptimer_init(lotus_wdt_tick, c,
                               PTIMER_POLICY_WRAP_AFTER_ONE_PERIOD |
                               PTIMER_POLICY_TRIGGER_ONLY_ON_DECREMENT |
                               PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                               PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);
        ptimer_transaction_begin(c->timer);
        ptimer_set_freq(c->timer, s->freq);
        ptimer_transaction_commit(c->timer);
    }
}

static void lotus_wdt_reset(DeviceState *dev)
{
    LotusWdtState *s = LOTUS_WDT(dev);

    s->locked = 0;
    for (int i = 0; i < LOTUS_WDT_CORES; i++) {
        LotusWdtCore *c = &s->core[i];

        c->load = 0xffffffff;
        c->control = 0;
        c->ris = 0;
        lotus_wdt_reload(c);
        qemu_irq_lower(c->irq);
    }
}

static const VMStateDescription vmstate_lotus_wdt_core = {
    .name = "lotus-wdt-core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PTIMER(timer, LotusWdtCore),
        VMSTATE_UINT32(load, LotusWdtCore),
        VMSTATE_UINT32(control, LotusWdtCore),
        VMSTATE_UINT32(ris, LotusWdtCore),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_lotus_wdt = {
    .name = TYPE_LOTUS_WDT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(core, LotusWdtState, LOTUS_WDT_CORES, 1,
                             vmstate_lotus_wdt_core, LotusWdtCore),
        VMSTATE_UINT32(locked, LotusWdtState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property lotus_wdt_props[] = {
    DEFINE_PROP_UINT32("freq", LotusWdtState, freq, 24000000),
};

static void lotus_wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = lotus_wdt_realize;
    dc->vmsd = &vmstate_lotus_wdt;
    device_class_set_legacy_reset(dc, lotus_wdt_reset);
    device_class_set_props(dc, lotus_wdt_props);
}

static const TypeInfo lotus_wdt_info = {
    .name          = TYPE_LOTUS_WDT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LotusWdtState),
    .instance_init = lotus_wdt_init,
    .class_init    = lotus_wdt_class_init,
};

static void lotus_wdt_register_types(void)
{
    type_register_static(&lotus_wdt_info);
}

type_init(lotus_wdt_register_types)
