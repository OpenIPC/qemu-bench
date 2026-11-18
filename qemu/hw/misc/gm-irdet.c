/*
 * Grain Media FTIRDET infrared remote receiver.
 *
 * The IR decoder at 0x91600000 on the GM813x (module/irda: irda_dev.h,
 * irda_drv.c):
 *
 *   0x00 Control   bit 0 enable, 1-4 protocol options (NEC 0x05)
 *   0x04-0x0C Param0-2   pulse thresholds, clock divider
 *   0x10 Status    bit 0 a code came in, bit 1 a repeat; write 1 clears
 *   0x14 Data      the code: NEC address | ~address << 8 | command << 16
 *                  | ~command << 24
 *
 * With no IR LED, codes are pressed from the monitor: writing the "key"
 * property (qom-set /machine/irdet key 0x45) delivers that NEC command
 * from address "address", repeat-flagged if it is the last one again
 * within "repeat-ms".  Interrupt 48 follows Status while enabled.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_IRDET "gm-irdet"
OBJECT_DECLARE_SIMPLE_TYPE(GmIrdetState, GM_IRDET)

#define GM_IRDET_SIZE       0x1000

#define R_CONTROL           0x00
#define R_PARAM0            0x04
#define R_PARAM1            0x08
#define R_PARAM2            0x0C
#define R_STATUS            0x10
#define R_DATA              0x14
#define GM_IRDET_NREGS      (0x18 / 4)

#define CTRL_ENABLE         (1u << 0)
#define STS_INT             (1u << 0)
#define STS_REPEAT          (1u << 1)

struct GmIrdetState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[GM_IRDET_NREGS];
    uint32_t address;
    uint32_t repeat_ms;
    uint32_t last_key;
    int64_t last_ns;
};

static void gm_irdet_update(GmIrdetState *s)
{
    qemu_set_irq(s->irq, (s->regs[R_CONTROL / 4] & CTRL_ENABLE) &&
                         (s->regs[R_STATUS / 4] & (STS_INT | STS_REPEAT)));
}

static void gm_irdet_key(GmIrdetState *s, uint32_t key)
{
    const int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    const uint8_t a = s->address, c = key;

    if (!(s->regs[R_CONTROL / 4] & CTRL_ENABLE)) {
        return;
    }
    s->regs[R_DATA / 4] = a | (uint8_t)~a << 8 | c << 16 |
                          (uint32_t)(uint8_t)~c << 24;
    s->regs[R_STATUS / 4] |= STS_INT;
    if (key == s->last_key &&
        now - s->last_ns < (int64_t)s->repeat_ms * SCALE_MS) {
        s->regs[R_STATUS / 4] |= STS_REPEAT;
    }
    s->last_key = key;
    s->last_ns = now;
    gm_irdet_update(s);
}

static uint64_t gm_irdet_read(void *opaque, hwaddr off, unsigned size)
{
    GmIrdetState *s = opaque;

    return off < GM_IRDET_NREGS * 4 ? s->regs[off / 4] : 0;
}

static void gm_irdet_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    GmIrdetState *s = opaque;

    switch (off) {
    case R_STATUS:
        s->regs[off / 4] &= ~val;
        break;
    case R_DATA:
        return;
    default:
        if (off >= GM_IRDET_NREGS * 4) {
            return;
        }
        s->regs[off / 4] = val;
        break;
    }
    gm_irdet_update(s);
}

static const MemoryRegionOps gm_irdet_ops = {
    .read = gm_irdet_read,
    .write = gm_irdet_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_irdet_get_key(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    GmIrdetState *s = GM_IRDET(obj);
    uint32_t key = s->last_key;

    visit_type_uint32(v, name, &key, errp);
}

static void gm_irdet_set_key(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    GmIrdetState *s = GM_IRDET(obj);
    uint32_t key;

    if (visit_type_uint32(v, name, &key, errp)) {
        gm_irdet_key(s, key & 0xFF);
    }
}

static void gm_irdet_reset(DeviceState *dev)
{
    GmIrdetState *s = GM_IRDET(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->last_key = 0;
    s->last_ns = 0;
    gm_irdet_update(s);
}

static void gm_irdet_init(Object *obj)
{
    GmIrdetState *s = GM_IRDET(obj);

    memory_region_init_io(&s->iomem, obj, &gm_irdet_ops, s, TYPE_GM_IRDET,
                          GM_IRDET_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    object_property_add(obj, "key", "uint32", gm_irdet_get_key,
                        gm_irdet_set_key, NULL, NULL);
}

static const Property gm_irdet_props[] = {
    DEFINE_PROP_UINT32("address", GmIrdetState, address, 0x00),
    DEFINE_PROP_UINT32("repeat-ms", GmIrdetState, repeat_ms, 150),
};

static const VMStateDescription vmstate_gm_irdet = {
    .name = TYPE_GM_IRDET,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmIrdetState, GM_IRDET_NREGS),
        VMSTATE_UINT32(last_key, GmIrdetState),
        VMSTATE_INT64(last_ns, GmIrdetState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_irdet_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, gm_irdet_reset);
    device_class_set_props(dc, gm_irdet_props);
    dc->vmsd = &vmstate_gm_irdet;
}

static const TypeInfo gm_irdet_info = {
    .name           = TYPE_GM_IRDET,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmIrdetState),
    .instance_init  = gm_irdet_init,
    .class_init     = gm_irdet_class_init,
};

static void gm_irdet_register_types(void)
{
    type_register_static(&gm_irdet_info);
}

type_init(gm_irdet_register_types)
