/*
 * GalaxyCore image sensor I2C stub, for the parts whose ID sits in a pair
 * of registers: 0xF0 / 0xF1 behind 8-bit register addresses (GC2053,
 * GC2063, GC1054) or 0x03F0 / 0x03F1 behind 16-bit ones (GC4653, GC2093).
 *
 *   "reg16"     register addresses are two bytes, big-endian
 *   "id-reg"    address of the ID's high byte (the low byte follows)
 *   "id"        the 16-bit chip ID, e.g. 0x4653
 *
 * Writes land in a register file the reads return, so a driver's init
 * table runs; a read continues from the register the last write set.  A
 * 16-bit part answers 0 when it was given a single address byte -- the
 * 8-bit probe ipctool tries first must fall through to its 16-bit one.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define TYPE_GALAXYCORE_SENSOR "galaxycore-sensor"
OBJECT_DECLARE_SIMPLE_TYPE(GcSensorState, GALAXYCORE_SENSOR)

#define GC_REGS             0x10000

struct GcSensorState {
    I2CSlave parent_obj;

    bool reg16;
    uint16_t id_reg;
    uint16_t id;

    uint16_t ptr;
    uint8_t naddr;          /* address bytes since the last START_SEND */
    uint8_t regs[GC_REGS];
};

static uint16_t gc_mask(GcSensorState *s)
{
    return s->reg16 ? 0xFFFF : 0xFF;
}

static int gc_event(I2CSlave *i2c, enum i2c_event event)
{
    GcSensorState *s = GALAXYCORE_SENSOR(i2c);

    if (event == I2C_START_SEND) {
        s->naddr = 0;
        s->ptr = 0;
    }
    return 0;
}

static int gc_send(I2CSlave *i2c, uint8_t data)
{
    GcSensorState *s = GALAXYCORE_SENSOR(i2c);

    if (s->naddr < (s->reg16 ? 2 : 1)) {
        s->ptr = (s->ptr << 8 | data) & gc_mask(s);
        s->naddr++;
        return 0;
    }
    s->regs[s->ptr] = data;
    s->ptr = (s->ptr + 1) & gc_mask(s);
    return 0;
}

static uint8_t gc_recv(I2CSlave *i2c)
{
    GcSensorState *s = GALAXYCORE_SENSOR(i2c);
    uint8_t val;

    if (s->reg16 && s->naddr == 1) {
        return 0;
    }
    val = s->regs[s->ptr];
    s->ptr = (s->ptr + 1) & gc_mask(s);
    return val;
}

static void gc_reset(DeviceState *dev)
{
    GcSensorState *s = GALAXYCORE_SENSOR(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[s->id_reg & gc_mask(s)] = s->id >> 8;
    s->regs[(s->id_reg + 1) & gc_mask(s)] = s->id & 0xFF;
    s->ptr = 0;
    s->naddr = 0;
}

static const VMStateDescription vmstate_gc_sensor = {
    .name = TYPE_GALAXYCORE_SENSOR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, GcSensorState),
        VMSTATE_UINT16(ptr, GcSensorState),
        VMSTATE_UINT8(naddr, GcSensorState),
        VMSTATE_UINT8_ARRAY(regs, GcSensorState, GC_REGS),
        VMSTATE_END_OF_LIST()
    },
};

static const Property gc_properties[] = {
    DEFINE_PROP_BOOL("reg16", GcSensorState, reg16, false),
    DEFINE_PROP_UINT16("id-reg", GcSensorState, id_reg, 0xF0),
    DEFINE_PROP_UINT16("id", GcSensorState, id, 0x2053),
};

static void gc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, gc_reset);
    device_class_set_props(dc, gc_properties);
    dc->vmsd = &vmstate_gc_sensor;
    k->event = gc_event;
    k->send = gc_send;
    k->recv = gc_recv;
}

static const TypeInfo gc_sensor_info = {
    .name          = TYPE_GALAXYCORE_SENSOR,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(GcSensorState),
    .class_init    = gc_class_init,
};

static void gc_sensor_register_types(void)
{
    type_register_static(&gc_sensor_info);
}

type_init(gc_sensor_register_types)
