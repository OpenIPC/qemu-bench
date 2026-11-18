/*
 * Axera AX_WDT: the real reset mechanism on this SoC -- the real
 * devicetree has NO "arm,psci*" node and NO "syscon-reboot"/
 * "syscon-poweroff" node anywhere (and a live trace of
 * TYPE_AXERA_A64's own PSCI-over-SMC conduit across an entire
 * boot+reboot cycle showed zero calls), but it does have
 * watchdog@4840000 { compatible = "axera,ax-wdt"; interrupts = <0
 * 0x95 4>; }, exactly this device's own base address.
 *
 * Confirmed live: without this device, reboot(RB_AUTOBOOT) prints
 * "reboot: Restarting system" and then the kernel falls all the way
 * through its own generic machine_restart() to a bare "while (1);"
 * fallback forever (confirmed via gdb: PC parked on a tight branch-
 * to-self, not even WFI) -- real silicon never reaches that fallback
 * because the watchdog genuinely resets it first.
 *
 * The vendor's closed-source "axera,ax-wdt" driver isn't in any
 * available source tree. It was initially modelled on the public
 * "snps,dw-wdt" driver's own real restart handler, dw_wdt_restart()
 * (set minimum timeout, switch to reset mode, kick with the
 * documented magic byte 0x76, wait ~500ms) -- but live register
 * tracing of the real sequence this dump's kernel actually performs
 * does not match that at all:
 *
 *   t=X     CR=0x1                  (watchdog driver probe enables it)
 *   t=X+5us CRR=0x80be              (a kick -- not the 0x76 magic)
 *   t=X+9us CR=0x1                  (redundant re-enable)
 *   ... nothing else touches this device for the entire rest of a
 *       normal boot (~40+ seconds: userspace init, flash provisioning,
 *       reboot(RB_AUTOBOOT) decision) ...
 *   t=Y     "reboot: Restarting system" printed
 *   t=Y+?   CRR=0x0                 (another kick -- also not 0x76)
 *
 * TORR is never written at all, at either point -- so this variant's
 * real restart handler apparently doesn't rely on dw_wdt_restart()'s
 * "minimize the timeout first" step, and CRR's own "kick" doesn't
 * require the documented DW_APB_WDT magic byte either. The only
 * register-level signal that actually distinguishes "harmless probe
 * self-test" from "the guest genuinely wants to reboot now" is
 * ordering: exactly one kick happens early, during normal probe, and
 * is never followed by anything else until a second, much later kick
 * arrives right when a real reboot is wanted. Treating any CRR write
 * while enabled as fatal reliably fired a reset ~tens-of-milliseconds
 * after the probe-time kick on every single boot (confirmed: 9
 * reboots within under 2 minutes, every one cut off right after "CPU1:
 * failed to come online" with no further kernel output at all) --
 * ignoring exactly the first such kick and firing (quickly) on any
 * kick after that matches both observed events correctly.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/misc/axera-wdt.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "system/runstate.h"

OBJECT_DECLARE_SIMPLE_TYPE(AxeraWdtState, AXERA_WDT)

#define AXERA_WDT_SIZE          0x400

#define WDT_CR                  0x00
#define WDT_TORR                0x04
#define WDT_CCVR                0x08
#define WDT_CRR                 0x0c
#define WDT_STAT                0x10
#define WDT_EOI                 0x14

#define WDT_CR_EN               BIT(0)
#define WDT_CR_RMOD             BIT(1)
#define WDT_CRR_KICK            0x76

/* Short enough to be clearly observable, long enough that the write
 * itself (and any immediately-following redundant register pokes)
 * cleanly completes first. */
#define AXERA_WDT_RESET_DELAY_NS (50 * SCALE_MS)

struct AxeraWdtState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer reset_timer;
    uint32_t cr;
    uint32_t torr;
    bool seen_first_kick;

    /*
     * dw-apb=on: a plain Synopsys DW_APB_WDT as driven by upstream
     * drivers/watchdog/dw_wdt.c ("snps,dw-wdt" -- AX620U's
     * watchdog@49aa000). Unlike AX620E's vendor driver above, that
     * driver's restart handler is the documented one: TORR=0, then
     * kick (0x76) if already enabled or set CR.WDT_EN if not, then
     * mdelay(500) waiting for the reset. So this mode counts down for
     * real: 2^(16 + TORR.TOP) cycles of clock-hz from every enable or
     * kick, then a system reset (or, with CR.RMOD, an interrupt first
     * and the reset on the following timeout). WDT_EN is sticky until
     * reset, as on the real IP. clock-hz only needs to be no faster
     * than what the guest believes the clock is, so a guest pinging
     * on schedule never times out; the restart handler's TOP=0 period
     * is 65536 cycles (~66 ms at the 1 MHz default).
     */
    bool dw_apb;
    uint32_t clock_hz;
    bool stat;
};

static void axera_wdt_dw_restart(AxeraWdtState *s)
{
    uint32_t top = s->torr & 0xf;
    uint64_t cycles = 1ULL << (16 + top);

    timer_mod(&s->reset_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              muldiv64(cycles, NANOSECONDS_PER_SECOND, s->clock_hz));
}

static void axera_wdt_reset_fire(void *opaque)
{
    AxeraWdtState *s = opaque;

    if (s->dw_apb && (s->cr & WDT_CR_RMOD) && !s->stat) {
        s->stat = true;
        qemu_irq_raise(s->irq);
        axera_wdt_dw_restart(s);
        return;
    }
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static uint64_t axera_wdt_dw_read(AxeraWdtState *s, hwaddr offset)
{
    switch (offset) {
    case WDT_CR:
        return s->cr;
    case WDT_TORR:
        return s->torr;
    case WDT_CCVR:
        if (timer_pending(&s->reset_timer)) {
            int64_t ns = timer_expire_time_ns(&s->reset_timer) -
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

            return ns > 0 ? muldiv64(ns, s->clock_hz,
                                     NANOSECONDS_PER_SECOND) : 0;
        }
        return 0;
    case WDT_STAT:
        return s->stat;
    case WDT_EOI:
        s->stat = false;
        qemu_irq_lower(s->irq);
        return 0;
    default:
        return 0;
    }
}

static void axera_wdt_dw_write(AxeraWdtState *s, hwaddr offset,
                               uint32_t value)
{
    switch (offset) {
    case WDT_CR:
        if ((value & WDT_CR_EN) && !(s->cr & WDT_CR_EN)) {
            s->cr = value;
            axera_wdt_dw_restart(s);
        }
        s->cr = value | (s->cr & WDT_CR_EN);
        break;
    case WDT_TORR:
        s->torr = value;
        break;
    case WDT_CRR:
        if ((s->cr & WDT_CR_EN) && (value & 0xff) == WDT_CRR_KICK) {
            s->stat = false;
            qemu_irq_lower(s->irq);
            axera_wdt_dw_restart(s);
        }
        break;
    default:
        break;
    }
}

static uint64_t axera_wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraWdtState *s = opaque;

    if (s->dw_apb) {
        return axera_wdt_dw_read(s, offset);
    }
    switch (offset) {
    case WDT_CR:
        return s->cr;
    case WDT_TORR:
        return s->torr;
    case WDT_CCVR:
        return 0;
    case WDT_STAT:
        return 0;
    case WDT_EOI:
        return 0;
    default:
        return 0;
    }
}

static void axera_wdt_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    AxeraWdtState *s = opaque;

    if (s->dw_apb) {
        axera_wdt_dw_write(s, offset, value);
        return;
    }
    switch (offset) {
    case WDT_CR:
        s->cr = value;
        if (!(s->cr & WDT_CR_EN)) {
            /* Disabled: no pending reboot request, and the next
             * enable's own first kick should again be treated as a
             * harmless probe self-test. */
            timer_del(&s->reset_timer);
            s->seen_first_kick = false;
        }
        break;
    case WDT_TORR:
        s->torr = value;
        break;
    case WDT_CRR:
        if (!(s->cr & WDT_CR_EN)) {
            break;
        }
        if (!s->seen_first_kick) {
            /* The probe's own self-test kick -- see this file's top
             * comment for why this one specifically is ignored. */
            s->seen_first_kick = true;
            break;
        }
        timer_mod(&s->reset_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 AXERA_WDT_RESET_DELAY_NS);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps axera_wdt_ops = {
    .read = axera_wdt_read,
    .write = axera_wdt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_wdt_reset(DeviceState *dev)
{
    AxeraWdtState *s = AXERA_WDT(dev);

    timer_del(&s->reset_timer);
    s->cr = 0;
    s->torr = 0;
    s->seen_first_kick = false;
    s->stat = false;
    qemu_set_irq(s->irq, 0);
}

static void axera_wdt_realize(DeviceState *dev, Error **errp)
{
    AxeraWdtState *s = AXERA_WDT(dev);

    timer_init_ns(&s->reset_timer, QEMU_CLOCK_VIRTUAL,
                 axera_wdt_reset_fire, s);
    memory_region_init_io(&s->iomem, OBJECT(s), &axera_wdt_ops, s,
                          TYPE_AXERA_WDT, AXERA_WDT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static const Property axera_wdt_properties[] = {
    DEFINE_PROP_BOOL("dw-apb", AxeraWdtState, dw_apb, false),
    DEFINE_PROP_UINT32("clock-hz", AxeraWdtState, clock_hz, 1000000),
};

static void axera_wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_wdt_realize;
    device_class_set_props(dc, axera_wdt_properties);
    device_class_set_legacy_reset(dc, axera_wdt_reset);
}

static const TypeInfo axera_wdt_info = {
    .name = TYPE_AXERA_WDT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraWdtState),
    .class_init = axera_wdt_class_init,
};

static void axera_wdt_register_types(void)
{
    type_register_static(&axera_wdt_info);
}

type_init(axera_wdt_register_types)
