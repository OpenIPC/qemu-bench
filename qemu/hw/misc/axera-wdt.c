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
};

static void axera_wdt_reset_fire(void *opaque)
{
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static uint64_t axera_wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraWdtState *s = opaque;

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

static void axera_wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_wdt_realize;
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
