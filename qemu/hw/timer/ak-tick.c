/*
 * Anyka AK3918AV130 tick timer, at 0x21110000.
 *
 * What the AV130's U-Boot 2019.10 (V4.0.x) keeps time with, as its
 * timer_init() and get_timer() drive it (first seen in owner report
 * r-5uwdebux's flash; no datasheet):
 *
 *   0x00  reload: 0x2EE0 at init, 12000 ticks of 12 MHz, a millisecond
 *   0x04  count, falling from the reload to 0 and reloading; read after
 *         latching it
 *   0x08  control: bit 0 latches the count into 0x04 (self-clearing);
 *         bits 3:1 cleared at init
 *   0x10  enable (bit 0)
 *   0x14  mode: 7:0 set to 1 at init
 *
 * Without it every U-Boot delay spins for ever: get_timer() reads the
 * count and waits for it to move.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_AK_TICK "ak-tick"
OBJECT_DECLARE_SIMPLE_TYPE(AkTickState, AK_TICK)

#define AK_TICK_HZ  12000000

struct AkTickState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t reload;
    uint32_t ctrl;
    uint32_t enable;
    uint32_t mode;
    uint32_t latched;
    int64_t start_ns;
};

static uint32_t ak_tick_count(AkTickState *s)
{
    uint64_t ticks;

    if (!s->enable || !s->reload) {
        return s->reload;
    }
    ticks = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->start_ns,
                     AK_TICK_HZ, NANOSECONDS_PER_SECOND);
    return s->reload - ticks % ((uint64_t)s->reload + 1);
}

static uint64_t ak_tick_read(void *opaque, hwaddr offset, unsigned size)
{
    AkTickState *s = opaque;

    switch (offset) {
    case 0x00:
        return s->reload;
    case 0x04:
        return s->latched;
    case 0x08:
        return s->ctrl;
    case 0x10:
        return s->enable;
    case 0x14:
        return s->mode;
    }
    return 0;
}

static void ak_tick_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    AkTickState *s = opaque;

    switch (offset) {
    case 0x00:
        s->reload = val;
        break;
    case 0x08:
        if (val & 1) {
            s->latched = ak_tick_count(s);
        }
        s->ctrl = val & ~1u;
        break;
    case 0x10:
        if ((val & 1) && !s->enable) {
            s->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        s->enable = val & 1;
        break;
    case 0x14:
        s->mode = val;
        break;
    }
}

static const MemoryRegionOps ak_tick_ops = {
    .read = ak_tick_read,
    .write = ak_tick_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ak_tick_reset(DeviceState *dev)
{
    AkTickState *s = AK_TICK(dev);

    s->reload = s->ctrl = s->enable = s->mode = s->latched = 0;
    s->start_ns = 0;
}

static void ak_tick_init(Object *obj)
{
    AkTickState *s = AK_TICK(obj);

    memory_region_init_io(&s->iomem, obj, &ak_tick_ops, s, TYPE_AK_TICK,
                          0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_ak_tick = {
    .name = TYPE_AK_TICK,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(reload, AkTickState),
        VMSTATE_UINT32(ctrl, AkTickState),
        VMSTATE_UINT32(enable, AkTickState),
        VMSTATE_UINT32(mode, AkTickState),
        VMSTATE_UINT32(latched, AkTickState),
        VMSTATE_INT64(start_ns, AkTickState),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_tick_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_ak_tick;
    device_class_set_legacy_reset(dc, ak_tick_reset);
}

static const TypeInfo ak_tick_info = {
    .name          = TYPE_AK_TICK,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkTickState),
    .instance_init = ak_tick_init,
    .class_init    = ak_tick_class_init,
};

static void ak_tick_register_types(void)
{
    type_register_static(&ak_tick_info);
}

type_init(ak_tick_register_types)
