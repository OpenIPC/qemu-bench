/*
 * Axera AX630C/AX620E on-chip RTC ("axera,axi-top-rtc", a DesignWare
 * RTC core -- its own driver header literally says "DesignWare Real
 * Time Clock Driver").
 *
 * drivers/rtc/rtc-axera.c + rtc-axera.h (vendor kernel source, both
 * real open .c/.h files, not a closed-source blob) give the exact
 * register map and the driver's own access pattern:
 *
 *   0x28/0x34/0x40/0x4C  SET_SEC/MIN/HOUR/DAY: staging registers for a
 *                        time commit (SEC is seconds * RTC_DEFAULT_CLOCK,
 *                        a fixed 4 Hz tick, not seconds * 1).
 *   0x68                 SET_SW: write 0xF to commit the four staging
 *                        registers above into the live clock.
 *   0xF8                 CFG_VALID_RSTAT: read-only, bits [3:0] go to
 *                        1 once the commit above has latched; the
 *                        driver polls this (up to 500 ms) after every
 *                        SET_SW write.
 *   0x10C                CFG_VALID_CLR_INTR: write 0xF to clear the
 *                        bits above once the driver has seen them set.
 *   0x58/0x5C/0x60/0x64  SEC/MIN/HOUR/DAY_CCVR: the live, free-running
 *                        current-value registers the driver actually
 *                        reads time back from (same SEC*4 scaling).
 *   0x90/0x9C/0xA8/0xB4  SEC/MIN/HOUR/DAY_MATCH: alarm compare value.
 *   0xC0                 MATCH_START: arm alarm compare.
 *   0xD4/0xE0/0xEC       ALARM_INTR_EN/CLR_INTR/INTR_MASK.
 *
 * "days" in this format is plain days-since-1970-01-01 (RTC_START_YEAR
 * 1970) -- no 1900 epoch or BCD encoding to undo, unlike many RTC
 * designs. The live clock is modelled as a single `base` (whole
 * seconds since 1970-01-01, QEMU's own `rtc_clock` added on top) the
 * same way hw/rtc/nvt-rtc.c already does for an unrelated Novatek RTC
 * in this same tree -- SET_SW recomputes `base` from the four staging
 * registers; SEC/MIN/HOUR/DAY_CCVR derive their answer from `base` plus
 * elapsed `rtc_clock` time, so the clock keeps advancing between reads
 * exactly like real silicon, without needing a periodic timer.
 *
 * The real app stack's camera services never set an alarm in any boot
 * observed with this model (the only RTC calls exercised are read/set
 * time) -- the alarm compare/match/enable registers are modelled as
 * plain read-back storage with no actual compare-and-fire logic, since
 * there is no evidence anything here needs it; a future user of this
 * file that does would need to add that, not this file's current scope.
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/rtc.h"
#include "system/system.h"

#define TYPE_AXERA_RTC "axera-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraRtcState, AXERA_RTC)

#define RTC_SET_SEC             0x28
#define RTC_SET_MIN             0x34
#define RTC_SET_HOUR            0x40
#define RTC_SET_DAY             0x4C
#define RTC_SEC_CCVR            0x58
#define RTC_MIN_CCVR            0x5C
#define RTC_HOUR_CCVR           0x60
#define RTC_DAY_CCVR            0x64
#define RTC_SET_SW              0x68
#define RTC_SEC_MATCH           0x90
#define RTC_MIN_MATCH           0x9C
#define RTC_HOUR_MATCH          0xA8
#define RTC_DAY_MATCH           0xB4
#define RTC_MATCH_START         0xC0
#define RTC_ALARM_CLR_INTR      0xE0
#define RTC_ALARM_INTR_EN       0xD4
#define RTC_ALARM_INTR_MASK     0xEC
#define RTC_CFG_VALID_RSTAT     0xF8
#define RTC_CFG_VALID_CLR_INTR  0x10C

#define RTC_DEFAULT_CLOCK       4   /* SEC_CCVR/SET_SEC tick at 4 Hz */
#define CFG_VALID_ALL           0xF

struct AxeraRtcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    int64_t base;              /* whole seconds since 1970-01-01 */
    uint32_t set_sec, set_min, set_hour, set_day;  /* SET_SW staging */
    uint32_t cfg_valid;

    uint32_t alarm_sec, alarm_min, alarm_hour, alarm_day;
    uint32_t alarm_intr_en, alarm_intr_mask, match_start;
};

static int64_t axera_rtc_now(AxeraRtcState *s)
{
    return s->base + qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND;
}

static uint64_t axera_rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraRtcState *s = opaque;
    int64_t now, day_secs;

    switch (offset) {
    case RTC_SEC_CCVR:
    case RTC_MIN_CCVR:
    case RTC_HOUR_CCVR:
    case RTC_DAY_CCVR:
        now = axera_rtc_now(s);
        day_secs = now % 86400;
        switch (offset) {
        case RTC_SEC_CCVR:
            return (day_secs % 60) * RTC_DEFAULT_CLOCK;
        case RTC_MIN_CCVR:
            return (day_secs / 60) % 60;
        case RTC_HOUR_CCVR:
            return day_secs / 3600;
        default:
            return now / 86400;
        }
    case RTC_CFG_VALID_RSTAT:
        return s->cfg_valid;
    case RTC_SEC_MATCH:
        return s->alarm_sec;
    case RTC_MIN_MATCH:
        return s->alarm_min;
    case RTC_HOUR_MATCH:
        return s->alarm_hour;
    case RTC_DAY_MATCH:
        return s->alarm_day;
    case RTC_ALARM_INTR_EN:
        return s->alarm_intr_en;
    case RTC_ALARM_INTR_MASK:
        return s->alarm_intr_mask;
    case RTC_MATCH_START:
        return s->match_start;
    default:
        return 0;
    }
}

static void axera_rtc_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    AxeraRtcState *s = opaque;

    switch (offset) {
    case RTC_SET_SEC:
        s->set_sec = val;
        break;
    case RTC_SET_MIN:
        s->set_min = val;
        break;
    case RTC_SET_HOUR:
        s->set_hour = val;
        break;
    case RTC_SET_DAY:
        s->set_day = val;
        break;
    case RTC_SET_SW:
        if (val & CFG_VALID_ALL) {
            int64_t total = (int64_t)s->set_day * 86400 +
                             s->set_hour * 3600 + s->set_min * 60 +
                             s->set_sec / RTC_DEFAULT_CLOCK;

            s->base = total - qemu_clock_get_ns(rtc_clock) /
                      NANOSECONDS_PER_SECOND;
            s->cfg_valid = CFG_VALID_ALL;
        }
        break;
    case RTC_CFG_VALID_CLR_INTR:
        s->cfg_valid &= ~val;
        break;
    case RTC_SEC_MATCH:
        s->alarm_sec = val;
        break;
    case RTC_MIN_MATCH:
        s->alarm_min = val;
        break;
    case RTC_HOUR_MATCH:
        s->alarm_hour = val;
        break;
    case RTC_DAY_MATCH:
        s->alarm_day = val;
        break;
    case RTC_MATCH_START:
        s->match_start = val;
        break;
    case RTC_ALARM_INTR_EN:
        s->alarm_intr_en = val;
        break;
    case RTC_ALARM_INTR_MASK:
        s->alarm_intr_mask = val;
        break;
    case RTC_ALARM_CLR_INTR:
        break;
    default:
        break;
    }
}

static const MemoryRegionOps axera_rtc_ops = {
    .read = axera_rtc_read,
    .write = axera_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void axera_rtc_reset(DeviceState *dev)
{
    AxeraRtcState *s = AXERA_RTC(dev);
    struct tm tm;

    qemu_get_timedate(&tm, 0);
    s->base = mktimegm(&tm) - qemu_clock_get_ns(rtc_clock) /
              NANOSECONDS_PER_SECOND;
    s->cfg_valid = 0;
    qemu_irq_lower(s->irq);
}

static void axera_rtc_init(Object *obj)
{
    AxeraRtcState *s = AXERA_RTC(obj);

    memory_region_init_io(&s->iomem, obj, &axera_rtc_ops, s, TYPE_AXERA_RTC,
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static int axera_rtc_pre_save(void *opaque)
{
    AxeraRtcState *s = opaque;

    /* Save wall-clock time, not an offset from this host's rtc_clock. */
    s->base = axera_rtc_now(s);
    return 0;
}

static int axera_rtc_post_load(void *opaque, int version_id)
{
    AxeraRtcState *s = opaque;

    s->base -= qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND;
    return 0;
}

static const VMStateDescription vmstate_axera_rtc = {
    .name = TYPE_AXERA_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = axera_rtc_pre_save,
    .post_load = axera_rtc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(base, AxeraRtcState),
        VMSTATE_UINT32(set_sec, AxeraRtcState),
        VMSTATE_UINT32(set_min, AxeraRtcState),
        VMSTATE_UINT32(set_hour, AxeraRtcState),
        VMSTATE_UINT32(set_day, AxeraRtcState),
        VMSTATE_UINT32(cfg_valid, AxeraRtcState),
        VMSTATE_UINT32(alarm_sec, AxeraRtcState),
        VMSTATE_UINT32(alarm_min, AxeraRtcState),
        VMSTATE_UINT32(alarm_hour, AxeraRtcState),
        VMSTATE_UINT32(alarm_day, AxeraRtcState),
        VMSTATE_UINT32(alarm_intr_en, AxeraRtcState),
        VMSTATE_UINT32(alarm_intr_mask, AxeraRtcState),
        VMSTATE_UINT32(match_start, AxeraRtcState),
        VMSTATE_END_OF_LIST()
    }
};

static void axera_rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, axera_rtc_reset);
    dc->vmsd = &vmstate_axera_rtc;
}

static const TypeInfo axera_rtc_info = {
    .name = TYPE_AXERA_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraRtcState),
    .instance_init = axera_rtc_init,
    .class_init = axera_rtc_class_init,
};

static void axera_rtc_register_types(void)
{
    type_register_static(&axera_rtc_info);
}

type_init(axera_rtc_register_types)
