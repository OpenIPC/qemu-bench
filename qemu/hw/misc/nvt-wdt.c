/*
 * Novatek NA51055 / NA51089 watchdog.
 *
 * plat-na51089/wdt_reg.h and drivers/watchdog/na51055_wdt.c:
 *
 *   0x00 CTRL      bit 0 enable, bit 1 mode (0 interrupt, 1 reset),
 *                  bit 4 external reset, [15:8] MSB of the 20-bit count,
 *                  [31:16] must hold the key 0x5A96 or the write is lost
 *   0x04 STS       [19:0] count, bit 30 enabled, bit 31 expired;
 *                  writing it back acknowledges the interrupt
 *   0x08 TRIG      bit 0 restarts the count
 *   0x0C MANUAL    bit 0 resets the chip at once (while enabled)
 *   0x80 REC       reset counters, 0x84 UDATA user word
 *
 * The count runs at 12 MHz / 1024 up to (MSB << 12) | 0xFFF, so the
 * driver's (t * 11718) >> 12 gives a t-second timeout.  The kernel's
 * restart hook (plat-na51089/reset.c) writes 0x5a960112, 0x5a960113 and
 * then MANUAL = 1: that manual reset is how "reboot" leaves the guest.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/runstate.h"
#include "system/watchdog.h"

#define TYPE_NVT_WDT "nvt-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(NvtWdtState, NVT_WDT)

#define WDT_CTRL        0x00
#define WDT_STS         0x04
#define WDT_TRIG        0x08
#define WDT_MANUAL      0x0C
#define WDT_REC         0x80
#define WDT_UDATA       0x84

#define CTRL_EN         (1u << 0)
#define CTRL_MODE_RST   (1u << 1)
#define CTRL_KEY        0x5A96u
#define STS_EN          (1u << 30)
#define STS_EXPIRED     (1u << 31)

#define WDT_FREQ        (12000000 / 1024)

struct NvtWdtState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    ptimer_state *ptimer;
    qemu_irq irq;
    uint32_t ctrl;
    uint32_t sts;
    uint32_t rec;
    uint32_t udata;
};

static uint32_t nvt_wdt_limit(NvtWdtState *s)
{
    return (((s->ctrl >> 8) & 0xFF) << 12) | 0xFFF;
}

/* Called inside a ptimer transaction. */
static void nvt_wdt_restart(NvtWdtState *s)
{
    ptimer_set_limit(s->ptimer, nvt_wdt_limit(s), 1);
    if (s->ctrl & CTRL_EN) {
        ptimer_run(s->ptimer, 1);
    } else {
        ptimer_stop(s->ptimer);
    }
}

static void nvt_wdt_expired(void *opaque)
{
    NvtWdtState *s = opaque;

    s->sts |= STS_EXPIRED;
    if (s->ctrl & CTRL_MODE_RST) {
        s->rec = (s->rec & ~0xFFu) | ((s->rec + 1) & 0xFF);
        watchdog_perform_action();
    } else {
        qemu_irq_raise(s->irq);
    }
}

static uint64_t nvt_wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtWdtState *s = opaque;
    uint32_t cnt;

    switch (offset) {
    case WDT_CTRL:
        return s->ctrl;
    case WDT_STS:
        /* The hardware counts up; the ptimer counts down to expiry. */
        cnt = nvt_wdt_limit(s) - ptimer_get_count(s->ptimer);
        return (s->sts & STS_EXPIRED) | ((s->ctrl & CTRL_EN) ? STS_EN : 0) |
               (cnt & 0xFFFFF);
    case WDT_REC:
        return s->rec;
    case WDT_UDATA:
        return s->udata;
    default:
        return 0;
    }
}

static void nvt_wdt_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    NvtWdtState *s = opaque;

    switch (offset) {
    case WDT_CTRL:
        if ((val >> 16) != CTRL_KEY) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "nvt-wdt: CTRL write 0x%08" PRIx64 " without key\n",
                          val);
            return;
        }
        s->ctrl = val & 0xFFFF;
        ptimer_transaction_begin(s->ptimer);
        nvt_wdt_restart(s);
        ptimer_transaction_commit(s->ptimer);
        break;
    case WDT_STS:
        if (val & STS_EXPIRED) {
            s->sts &= ~STS_EXPIRED;
            qemu_irq_lower(s->irq);
        }
        break;
    case WDT_TRIG:
        if (val & 1) {
            ptimer_transaction_begin(s->ptimer);
            nvt_wdt_restart(s);
            ptimer_transaction_commit(s->ptimer);
        }
        break;
    case WDT_MANUAL:
        if ((val & 1) && (s->ctrl & CTRL_EN)) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        break;
    case WDT_REC:
        s->rec = val;
        break;
    case WDT_UDATA:
        s->udata = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-wdt: write 0x%08" PRIx64
                      " to 0x%02" HWADDR_PRIx "\n", val, offset);
        break;
    }
}

static const MemoryRegionOps nvt_wdt_ops = {
    .read = nvt_wdt_read,
    .write = nvt_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_wdt_reset(DeviceState *dev)
{
    NvtWdtState *s = NVT_WDT(dev);

    /* REC and UDATA survive the reset they record, as on silicon. */
    s->ctrl = 0;
    s->sts = 0;
    qemu_irq_lower(s->irq);
    ptimer_transaction_begin(s->ptimer);
    nvt_wdt_restart(s);
    ptimer_transaction_commit(s->ptimer);
}

static void nvt_wdt_init(Object *obj)
{
    NvtWdtState *s = NVT_WDT(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_wdt_ops, s, TYPE_NVT_WDT,
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void nvt_wdt_realize(DeviceState *dev, Error **errp)
{
    NvtWdtState *s = NVT_WDT(dev);

    s->ptimer = ptimer_init(nvt_wdt_expired, s, PTIMER_POLICY_LEGACY);
    ptimer_transaction_begin(s->ptimer);
    ptimer_set_freq(s->ptimer, WDT_FREQ);
    ptimer_transaction_commit(s->ptimer);
}

static const VMStateDescription vmstate_nvt_wdt = {
    .name = TYPE_NVT_WDT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PTIMER(ptimer, NvtWdtState),
        VMSTATE_UINT32(ctrl, NvtWdtState),
        VMSTATE_UINT32(sts, NvtWdtState),
        VMSTATE_UINT32(rec, NvtWdtState),
        VMSTATE_UINT32(udata, NvtWdtState),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_wdt_realize;
    device_class_set_legacy_reset(dc, nvt_wdt_reset);
    dc->vmsd = &vmstate_nvt_wdt;
}

static const TypeInfo nvt_wdt_info = {
    .name = TYPE_NVT_WDT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtWdtState),
    .instance_init = nvt_wdt_init,
    .class_init = nvt_wdt_class_init,
};

static void nvt_wdt_register_types(void)
{
    type_register_static(&nvt_wdt_info);
}

type_init(nvt_wdt_register_types)
