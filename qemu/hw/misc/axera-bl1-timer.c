/*
 * Axera BL1's generic delay timer (TIMER_BASE @ 0x04830000): a standard
 * Synopsys DesignWare APB free-running down-counter.
 *
 * Grounded directly in the real vendor BL1 source
 * (maix_ax620e_sdk/boot/bl1/driver/include/timer.h): TIMER_LOADCOUNT
 * (+0x0) sets the reload value, TIMER_CONTROL_REG (+0x8) bit 0 starts
 * it, and TIMER_CURRENT_VALUE (+0x4) free-runs downward from there at
 * 24 MHz (TICK2SEC=24000000) entirely in hardware, independent of
 * software. BL1's delay()/mdelay()/udelay() -- and, in turn, a "wait
 * until N ticks have elapsed" loop buried under a timer-ratio helper
 * called from a tight retry loop -- work by reading
 * TIMER_CURRENT_VALUE twice and checking the difference. A plain
 * storage regbank can never satisfy that: two reads of inert storage
 * always return the same value, so the difference is always 0 and the
 * loop never sees enough elapsed time, spinning forever with no
 * -d unimp trace (it's reading storage that exists, just never
 * changes, same shape as the PLL ready-bit problem axera-pllc.c
 * solves). This model actually counts down against QEMU's virtual
 * clock so successive reads differ, exactly as real silicon would.
 *
 * The same block is Linux's ax_hrtimer@4830000 (drivers/soc/axera/
 * axera_hrtimer/axera_hrtimer.c, GIC SPI 124 for timer 0): the
 * AX_HRTIMER_USLEEP ioctl that the camera stack's VENC/IVPS worker threads
 * sleep in loads TIMER_LOADCOUNT, enables timer 0 with its interrupt
 * unmasked (TIMER_CONTROL_REG bit 2 clear) and waits for the expiry
 * interrupt, acked by reading TIMER_EOI. Without that interrupt every
 * hrtimer sleep lasts forever: the encoder threads never wake to take the
 * frames IVPS links to them, so nothing is ever encoded.
 *
 * (Earlier mapped at this same address as a generic "axera-bl1-wdt"
 * regbank, named from an ambiguous -d unimp access pattern seen before
 * the real vendor header was read; TIMER_BASE is the correct identity.)
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AXERA_BL1_TIMER "axera-bl1-timer"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraBl1TimerState, AXERA_BL1_TIMER)

#define TIMER_SIZE              0x100
#define TIMER_LOADCOUNT         0x0
#define TIMER_CURRENT_VALUE     0x4
#define TIMER_CONTROL_REG       0x8
#define TIMER_EOI               0xc
#define TIMER_INT_STATUS        0x10
#define TIMERS_INT_STATUS       0xa0
#define TIMERS_EOI              0xa4
#define TIMERS_RAW_INT_STATUS   0xa8
#define TIMERS_COMP_VERSION     0xac
#define TIMER_ENABLE            BIT(0)
#define TIMER_INT_MASK          BIT(2)

#define TIMER_TICK_HZ           24000000ULL

struct AxeraBl1TimerState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t loadcount;
    uint32_t control;
    int64_t enable_time_ns;
    bool raw_int;
    qemu_irq irq;
    QEMUTimer *expiry;
};

static void axera_bl1_timer_update_irq(AxeraBl1TimerState *s)
{
    qemu_set_irq(s->irq, s->raw_int && !(s->control & TIMER_INT_MASK));
}

/* (Re)arm the expiry callback for the current count-down period. */
static void axera_bl1_timer_arm(AxeraBl1TimerState *s)
{
    uint64_t ticks = (uint64_t)s->loadcount + 1;

    if (!(s->control & TIMER_ENABLE)) {
        timer_del(s->expiry);
        return;
    }
    timer_mod(s->expiry, s->enable_time_ns +
              muldiv64(ticks, NANOSECONDS_PER_SECOND, TIMER_TICK_HZ));
}

static void axera_bl1_timer_expire(void *opaque)
{
    AxeraBl1TimerState *s = opaque;

    s->raw_int = true;
    axera_bl1_timer_update_irq(s);
    /* Count again from the reload value (user-defined and free-running
     * modes differ only in that value; the driver re-arms explicitly). */
    s->enable_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    axera_bl1_timer_arm(s);
}

static uint32_t axera_bl1_timer_current(AxeraBl1TimerState *s)
{
    uint64_t elapsed_ns, elapsed_ticks, period;

    if (!(s->control & TIMER_ENABLE)) {
        return s->loadcount;
    }

    elapsed_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->enable_time_ns;
    elapsed_ticks = muldiv64(elapsed_ns, TIMER_TICK_HZ, NANOSECONDS_PER_SECOND);
    period = (uint64_t)s->loadcount + 1;
    return s->loadcount - (elapsed_ticks % period);
}

static uint64_t axera_bl1_timer_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    AxeraBl1TimerState *s = opaque;

    switch (offset) {
    case TIMER_LOADCOUNT:
        return s->loadcount;
    case TIMER_CURRENT_VALUE:
        return axera_bl1_timer_current(s);
    case TIMER_CONTROL_REG:
        return s->control;
    case TIMER_EOI:
    case TIMERS_EOI:
        s->raw_int = false;
        axera_bl1_timer_update_irq(s);
        return 0;
    case TIMER_INT_STATUS:
    case TIMERS_INT_STATUS:
        return s->raw_int && !(s->control & TIMER_INT_MASK);
    case TIMERS_RAW_INT_STATUS:
        return s->raw_int;
    case TIMERS_COMP_VERSION:
        return 0x3230392a; /* DW_apb_timers "209*" */
    default:
        return 0;
    }
}

static void axera_bl1_timer_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    AxeraBl1TimerState *s = opaque;

    switch (offset) {
    case TIMER_LOADCOUNT:
        s->loadcount = value;
        if (s->control & TIMER_ENABLE) {
            s->enable_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        axera_bl1_timer_arm(s);
        break;
    case TIMER_CONTROL_REG:
        if ((value & TIMER_ENABLE) && !(s->control & TIMER_ENABLE)) {
            s->enable_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        s->control = value;
        axera_bl1_timer_arm(s);
        axera_bl1_timer_update_irq(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps axera_bl1_timer_ops = {
    .read = axera_bl1_timer_read,
    .write = axera_bl1_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_bl1_timer_realize(DeviceState *dev, Error **errp)
{
    AxeraBl1TimerState *s = AXERA_BL1_TIMER(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_bl1_timer_ops, s,
                          TYPE_AXERA_BL1_TIMER, TIMER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->expiry = timer_new_ns(QEMU_CLOCK_VIRTUAL, axera_bl1_timer_expire, s);
}

/*
 * System reset (watchdog or reboot): stop the counter, drop any pending
 * expiry and interrupt, and return the registers to their power-on
 * zero state so the next boot does not inherit the previous one's
 * hrtimer sleep.
 */
static void axera_bl1_timer_reset(DeviceState *dev)
{
    AxeraBl1TimerState *s = AXERA_BL1_TIMER(dev);

    timer_del(s->expiry);
    s->loadcount = 0;
    s->control = 0;
    s->enable_time_ns = 0;
    s->raw_int = false;
    qemu_irq_lower(s->irq);
}

static void axera_bl1_timer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_bl1_timer_realize;
    device_class_set_legacy_reset(dc, axera_bl1_timer_reset);
}

static const TypeInfo axera_bl1_timer_info = {
    .name = TYPE_AXERA_BL1_TIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraBl1TimerState),
    .class_init = axera_bl1_timer_class_init,
};

static void axera_bl1_timer_register_types(void)
{
    type_register_static(&axera_bl1_timer_info);
}

type_init(axera_bl1_timer_register_types)
