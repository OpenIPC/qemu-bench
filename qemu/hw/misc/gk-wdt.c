/*
 * Goke GK710x watchdog.
 *
 * The Ambarella watchdog, at the offsets the silicon decodes (the vendor
 * HAL presents them shuffled; include/hw/arm/goke.h):
 *
 *   0x00 STATUS   current count (read only)
 *   0x04 RELOAD
 *   0x08 RESTART  writing the magic 0x4755 reloads STATUS from RELOAD
 *   0x0C CONTROL  bit 0 enable, bit 1 reset on timeout, bit 2 interrupt
 *   0x10 TIMEOUT  bit 0 set once the count has run out
 *   0x14 CLR_TMO  write 1 to clear TIMEOUT
 *   0x18 RST_WD   reset pulse width
 *
 * The count runs at the APB clock.  The vendor kernel reboots through it
 * (gk_restart: RELOAD 0xff, RESTART, CONTROL = reset | enable), so the
 * reset-on-timeout path is what makes "reboot" work.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/watchdog.h"

#define TYPE_GK_WDT "gk-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(GkWdtState, GK_WDT)

#define WDT_STATUS      0x00
#define WDT_RELOAD      0x04
#define WDT_RESTART     0x08
#define WDT_CONTROL     0x0C
#define WDT_TIMEOUT     0x10
#define WDT_CLR_TMO     0x14
#define WDT_RST_WD      0x18

#define WDT_MAGIC       0x4755
#define CTRL_EN         (1u << 0)
#define CTRL_RST        (1u << 1)
#define CTRL_INT        (1u << 2)

struct GkWdtState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    ptimer_state *ptimer;
    qemu_irq irq;
    uint32_t reload;
    uint32_t control;
    uint32_t timeout;
    uint32_t rst_wd;
    uint32_t freq;
};

static void gk_wdt_expired(void *opaque)
{
    GkWdtState *s = opaque;

    /* Runs one-shot: the count stops at zero until the next RESTART. */
    s->timeout = 1;
    if (s->control & CTRL_INT) {
        qemu_irq_pulse(s->irq);
    }
    if (s->control & CTRL_RST) {
        watchdog_perform_action();
    }
}

static void gk_wdt_restart(GkWdtState *s)
{
    ptimer_transaction_begin(s->ptimer);
    ptimer_set_limit(s->ptimer, s->reload ? s->reload : 1, 1);
    if (s->control & CTRL_EN) {
        ptimer_run(s->ptimer, 1);
    } else {
        ptimer_stop(s->ptimer);
    }
    ptimer_transaction_commit(s->ptimer);
}

static uint64_t gk_wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    GkWdtState *s = opaque;

    switch (offset) {
    case WDT_STATUS:
        return ptimer_get_count(s->ptimer);
    case WDT_RELOAD:
        return s->reload;
    case WDT_CONTROL:
        return s->control;
    case WDT_TIMEOUT:
        return s->timeout;
    case WDT_RST_WD:
        return s->rst_wd;
    default:
        return 0;
    }
}

static void gk_wdt_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    GkWdtState *s = opaque;

    switch (offset) {
    case WDT_RELOAD:
        s->reload = val;
        break;
    case WDT_RESTART:
        if (val == WDT_MAGIC) {
            gk_wdt_restart(s);
        }
        break;
    case WDT_CONTROL:
        s->control = val & (CTRL_EN | CTRL_RST | CTRL_INT);
        gk_wdt_restart(s);
        break;
    case WDT_CLR_TMO:
        if (val & 1) {
            s->timeout = 0;
        }
        break;
    case WDT_RST_WD:
        s->rst_wd = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "gk-wdt: write 0x%08" PRIx64
                      " to 0x%02" HWADDR_PRIx "\n", val, offset);
        break;
    }
}

static const MemoryRegionOps gk_wdt_ops = {
    .read = gk_wdt_read,
    .write = gk_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gk_wdt_reset(DeviceState *dev)
{
    GkWdtState *s = GK_WDT(dev);

    s->reload = 0;
    s->control = 0;
    s->timeout = 0;
    s->rst_wd = 0;
    ptimer_transaction_begin(s->ptimer);
    ptimer_stop(s->ptimer);
    ptimer_set_limit(s->ptimer, 0xFFFFFFFF, 1);
    ptimer_transaction_commit(s->ptimer);
}

static void gk_wdt_init(Object *obj)
{
    GkWdtState *s = GK_WDT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &gk_wdt_ops, s, TYPE_GK_WDT,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void gk_wdt_realize(DeviceState *dev, Error **errp)
{
    GkWdtState *s = GK_WDT(dev);

    s->ptimer = ptimer_init(gk_wdt_expired, s, PTIMER_POLICY_LEGACY);
    ptimer_transaction_begin(s->ptimer);
    ptimer_set_freq(s->ptimer, s->freq);
    ptimer_transaction_commit(s->ptimer);
}

static const Property gk_wdt_properties[] = {
    DEFINE_PROP_UINT32("freq", GkWdtState, freq, 69000000),
};

static const VMStateDescription vmstate_gk_wdt = {
    .name = TYPE_GK_WDT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PTIMER(ptimer, GkWdtState),
        VMSTATE_UINT32(reload, GkWdtState),
        VMSTATE_UINT32(control, GkWdtState),
        VMSTATE_UINT32(timeout, GkWdtState),
        VMSTATE_UINT32(rst_wd, GkWdtState),
        VMSTATE_END_OF_LIST()
    }
};

static void gk_wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = gk_wdt_realize;
    device_class_set_legacy_reset(dc, gk_wdt_reset);
    device_class_set_props(dc, gk_wdt_properties);
    dc->vmsd = &vmstate_gk_wdt;
    set_bit(DEVICE_CATEGORY_WATCHDOG, dc->categories);
}

static const TypeInfo gk_wdt_info = {
    .name          = TYPE_GK_WDT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GkWdtState),
    .instance_init = gk_wdt_init,
    .class_init    = gk_wdt_class_init,
};

static void gk_wdt_register_types(void)
{
    type_register_static(&gk_wdt_info);
}

type_init(gk_wdt_register_types)
