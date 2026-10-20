/*
 * Novatek NA51055 / NA51089 RTC and power-button controller.
 *
 * plat-na51089/rtc_reg.h, rtc_int.h and drivers/rtc/rtc-na51055.c:
 *
 *   0x00 TIMER    [5:0] sec [11:6] min [16:12] hour
 *   0x04 DAYKEY   [15:0] days since 1900-01-01, [19:16] key
 *   0x08 ALARM    0x20 PWRALM (power-on alarm), 0x14 DATA, 0x24 OSCAN
 *   0x0C STATUS   bit 0 alarm, bit 1 SRST busy, bit 2 ready, bit 8 CSET
 *                 done; alarm and CSET are write-1-to-clear
 *   0x10 CTRL     bit 0 alarm int, bit 1 SRST, bit 2 CSET, bits 3..7 and
 *                 12 select what a CSET commits, bit 8 CSET int
 *   0x18 PWBC     bit 0 reset shutdown timer, bit 1 power off, bit 3 / 4
 *                 power alarm enable / disable (self-clearing)
 *   0x1C PWBCSTS  bit 0 power alarm armed, bit 1 core power valid
 *
 * Writes to TIMER, DAYKEY and the alarms land in shadow registers that a
 * CSET copies across, raising CSET done; the driver sleeps on that
 * interrupt (rtc_cset_work) before every access.  A key other than 0xA,
 * or an oscillator setting other than 0x9, makes it treat the RTC as
 * having lost power and reset it to 1970: both read as a kept-alive RTC
 * here, and the time is the host's.
 *
 * Reboot goes through here (plat-na51089/reset.c): the kernel arms the
 * power-on alarm a few seconds ahead and powers the chip off, and the
 * alarm powers it back on.  Power off with the alarm armed is a reset;
 * without it (poweroff), a shutdown.
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/rtc.h"
#include "system/runstate.h"
#include "system/system.h"

#define TYPE_NVT_RTC "nvt-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(NvtRtcState, NVT_RTC)

#define RTC_TIMER       0x00
#define RTC_DAYKEY      0x04
#define RTC_ALARM       0x08
#define RTC_STATUS      0x0C
#define RTC_CTRL        0x10
#define RTC_DATA        0x14
#define RTC_PWBC        0x18
#define RTC_PWBCSTS     0x1C
#define RTC_PWRALM      0x20
#define RTC_OSCAN       0x24

#define STS_ALARM       (1u << 0)
#define STS_READY       (1u << 2)
#define STS_CSET        (1u << 8)

#define CTRL_ALARM_INT  (1u << 0)
#define CTRL_SRST       (1u << 1)
#define CTRL_CSET       (1u << 2)
#define CTRL_TIME_SEL   (1u << 3)
#define CTRL_DAY_SEL    (1u << 4)
#define CTRL_KEY_SEL    (1u << 5)
#define CTRL_PWRALM_SEL (3u << 6)
#define CTRL_CSET_INT   (1u << 8)
#define CTRL_DATA0_SEL  (1u << 12)
#define CTRL_SELS       (CTRL_TIME_SEL | CTRL_DAY_SEL | CTRL_KEY_SEL | \
                         CTRL_PWRALM_SEL | CTRL_DATA0_SEL)

#define PWBC_PWR_OFF    (1u << 1)
#define PWBC_ALM_EN     (1u << 3)
#define PWBC_ALM_DIS    (1u << 4)
#define PWBC_INTEN      (3u << 24)

#define PWBCSTS_ALM     (1u << 0)
#define PWBCSTS_COREPWR (1u << 1)

#define RTC_KEY         0xA         /* RTC_INT_KEY */
#define RTC_OSC_CFG     0x9         /* RTC_INT_OSC_ANALOG_CFG */

/* 1900-01-01 to 1970-01-01 */
#define DAYS_1900_TO_1970   25567

struct NvtRtcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    /* RTC time = this + rtc_clock seconds, in seconds since 1900. */
    int64_t base;
    uint32_t key;
    uint32_t alarm;
    uint32_t pwralm;
    uint32_t data;
    uint32_t oscan;
    uint32_t status;
    uint32_t ctrl;
    uint32_t pwbc;
    uint32_t pwbcsts;
    /* what a CSET commits */
    uint32_t set_timer;
    uint32_t set_daykey;
};

static int64_t nvt_rtc_clock_secs(void)
{
    return qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND;
}

static int64_t nvt_rtc_now(NvtRtcState *s)
{
    return s->base + nvt_rtc_clock_secs();
}

static void nvt_rtc_set_now(NvtRtcState *s, int64_t secs)
{
    s->base = secs - nvt_rtc_clock_secs();
}

static void nvt_rtc_update_irq(NvtRtcState *s)
{
    bool level = ((s->status & STS_ALARM) && (s->ctrl & CTRL_ALARM_INT)) ||
                 ((s->status & STS_CSET) && (s->ctrl & CTRL_CSET_INT));

    qemu_set_irq(s->irq, level);
}

static void nvt_rtc_cset(NvtRtcState *s)
{
    int64_t now = nvt_rtc_now(s);
    int64_t days = now / 86400, secs = now % 86400;

    if (s->ctrl & CTRL_TIME_SEL) {
        secs = (s->set_timer & 0x3F) + ((s->set_timer >> 6) & 0x3F) * 60 +
               ((s->set_timer >> 12) & 0x1F) * 3600;
    }
    if (s->ctrl & CTRL_DAY_SEL) {
        days = s->set_daykey & 0xFFFF;
    }
    if (s->ctrl & (CTRL_TIME_SEL | CTRL_DAY_SEL)) {
        nvt_rtc_set_now(s, days * 86400 + secs);
    }
    if (s->ctrl & CTRL_KEY_SEL) {
        s->key = (s->set_daykey >> 16) & 0xF;
    }
    s->ctrl &= ~(CTRL_CSET | CTRL_SELS);
    s->status |= STS_CSET;
    nvt_rtc_update_irq(s);
}

static uint64_t nvt_rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtRtcState *s = opaque;
    int64_t now = nvt_rtc_now(s);
    int64_t secs = now % 86400;

    switch (offset) {
    case RTC_TIMER:
        return (secs % 60) | (((secs / 60) % 60) << 6) |
               ((secs / 3600) << 12);
    case RTC_DAYKEY:
        return ((now / 86400) & 0xFFFF) | (s->key << 16);
    case RTC_ALARM:
        return s->alarm;
    case RTC_STATUS:
        return s->status | STS_READY;
    case RTC_CTRL:
        return s->ctrl;
    case RTC_DATA:
        return s->data;
    case RTC_PWBC:
        return s->pwbc;
    case RTC_PWBCSTS:
        return s->pwbcsts | PWBCSTS_COREPWR;
    case RTC_PWRALM:
        return s->pwralm;
    case RTC_OSCAN:
        return s->oscan;
    default:
        return 0;
    }
}

static void nvt_rtc_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    NvtRtcState *s = opaque;

    switch (offset) {
    case RTC_TIMER:
        s->set_timer = val;
        break;
    case RTC_DAYKEY:
        s->set_daykey = val;
        break;
    case RTC_ALARM:
        s->alarm = val;
        break;
    case RTC_STATUS:
        s->status &= ~(val & (STS_ALARM | STS_CSET));
        nvt_rtc_update_irq(s);
        break;
    case RTC_CTRL:
        if (val & CTRL_SRST) {
            /* A software reset forgets the time and the key. */
            nvt_rtc_set_now(s, 0);
            s->key = 0;
            s->ctrl = 0;
            s->status = 0;
            nvt_rtc_update_irq(s);
            break;
        }
        s->ctrl = val;
        if (val & CTRL_CSET) {
            nvt_rtc_cset(s);
        } else {
            nvt_rtc_update_irq(s);
        }
        break;
    case RTC_DATA:
        s->data = val & 0xFF;
        break;
    case RTC_PWBC:
        if (val & PWBC_ALM_EN) {
            s->pwbcsts |= PWBCSTS_ALM;
        }
        if (val & PWBC_ALM_DIS) {
            s->pwbcsts &= ~PWBCSTS_ALM;
        }
        s->pwbc = val & PWBC_INTEN;
        if (val & PWBC_PWR_OFF) {
            if (s->pwbcsts & PWBCSTS_ALM) {
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            } else {
                qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
            }
        }
        break;
    case RTC_PWBCSTS:
        s->pwbcsts &= ~(val & ~(PWBCSTS_ALM | PWBCSTS_COREPWR));
        break;
    case RTC_PWRALM:
        s->pwralm = val;
        break;
    case RTC_OSCAN:
        s->oscan = val & 0xFF;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-rtc: write 0x%08" PRIx64
                      " to 0x%02" HWADDR_PRIx "\n", val, offset);
        break;
    }
}

static const MemoryRegionOps nvt_rtc_ops = {
    .read = nvt_rtc_read,
    .write = nvt_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* The RTC domain is battery-backed: a chip reset keeps time and key. */
static void nvt_rtc_reset(DeviceState *dev)
{
    NvtRtcState *s = NVT_RTC(dev);

    s->status = 0;
    s->ctrl = 0;
    s->pwbc = 0;
    s->pwbcsts = 0;
    qemu_irq_lower(s->irq);
}

static void nvt_rtc_realize(DeviceState *dev, Error **errp)
{
    NvtRtcState *s = NVT_RTC(dev);
    struct tm tm;

    qemu_get_timedate(&tm, 0);
    nvt_rtc_set_now(s, mktimegm(&tm) + (int64_t)DAYS_1900_TO_1970 * 86400);
    s->key = RTC_KEY;
    s->oscan = RTC_OSC_CFG;
}

static void nvt_rtc_init(Object *obj)
{
    NvtRtcState *s = NVT_RTC(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_rtc_ops, s, TYPE_NVT_RTC,
                          0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static int nvt_rtc_pre_save(void *opaque)
{
    NvtRtcState *s = opaque;

    /* Save wall-clock time, not an offset from this host's rtc_clock. */
    s->base = nvt_rtc_now(s);
    return 0;
}

static int nvt_rtc_post_save(void *opaque)
{
    NvtRtcState *s = opaque;

    nvt_rtc_set_now(s, s->base);
    return 0;
}

static int nvt_rtc_post_load(void *opaque, int version_id)
{
    return nvt_rtc_post_save(opaque);
}

static const VMStateDescription vmstate_nvt_rtc = {
    .name = TYPE_NVT_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = nvt_rtc_pre_save,
    .post_save = nvt_rtc_post_save,
    .post_load = nvt_rtc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(base, NvtRtcState),
        VMSTATE_UINT32(key, NvtRtcState),
        VMSTATE_UINT32(alarm, NvtRtcState),
        VMSTATE_UINT32(pwralm, NvtRtcState),
        VMSTATE_UINT32(data, NvtRtcState),
        VMSTATE_UINT32(oscan, NvtRtcState),
        VMSTATE_UINT32(status, NvtRtcState),
        VMSTATE_UINT32(ctrl, NvtRtcState),
        VMSTATE_UINT32(pwbc, NvtRtcState),
        VMSTATE_UINT32(pwbcsts, NvtRtcState),
        VMSTATE_UINT32(set_timer, NvtRtcState),
        VMSTATE_UINT32(set_daykey, NvtRtcState),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_rtc_realize;
    device_class_set_legacy_reset(dc, nvt_rtc_reset);
    dc->vmsd = &vmstate_nvt_rtc;
}

static const TypeInfo nvt_rtc_info = {
    .name = TYPE_NVT_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtRtcState),
    .instance_init = nvt_rtc_init,
    .class_init = nvt_rtc_class_init,
};

static void nvt_rtc_register_types(void)
{
    type_register_static(&nvt_rtc_info);
}

type_init(nvt_rtc_register_types)
