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
#include "hw/i2c/image-sensor.h"
#include "hw/i2c/ingenic-sensor.h"
#include "hw/i2c/sensor-sync.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/log.h"

#define TYPE_GALAXYCORE_SENSOR "galaxycore-sensor"
OBJECT_DECLARE_SIMPLE_TYPE(GcSensorState, GALAXYCORE_SENSOR)

#define GC_REGS             0x10000

struct GcSensorState {
    I2CSlave parent_obj;

    bool reg16;
    uint16_t id_reg;
    uint16_t id;
    bool trace;

    uint16_t ptr;
    uint8_t naddr;          /* address bytes since the last START_SEND */
    uint8_t regs[GC_REGS];
    /*
     * The 8-bit parts page their registers by 0xFE[2:0]; regs[] holds
     * the last write to each address on any page, p0[] page 0's -- where
     * the exposure lives.
     */
    uint8_t page;
    uint8_t p0[0x100];
    /*
     * The 16-bit parts: what the frame in progress uses of the registers
     * a frame takes at its start (gc_latched[]); the rest as written.
     */
    uint8_t applied[GC_REGS];
    char name[24];
    SensorSync sync;
};

/*
 * What the HiSilicon GC4653 driver writes every frame (gc4653_2l_cmos.c's
 * sync list): exposure (and the DOL short frame's), digital gain, the
 * analog gain's stages and the frame length.
 */
static const uint16_t gc_latched[] = {
    0x0200, 0x0201, 0x0202, 0x0203, 0x020E, 0x020F, 0x02B3, 0x02B4,
    0x02B8, 0x02B9, 0x02D9, 0x0515, 0x0519, 0x0340, 0x0341,
};

static bool gc_is_latched(uint16_t a)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(gc_latched); i++) {
        if (gc_latched[i] == a) {
            return true;
        }
    }
    return false;
}

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
    if (s->trace) {
        qemu_log("galaxycore: [0x%04x] = 0x%02x\n", s->ptr, data);
    }
    sensor_sync_write(&s->sync, s->ptr, data);
    if (s->reg16 && (!s->sync.latching || !gc_is_latched(s->ptr))) {
        s->applied[s->ptr] = data;
    }
    if (!s->reg16) {
        if (s->ptr == 0xFE) {
            s->page = data & 7;
        } else if (!s->page) {
            s->p0[s->ptr & 0xFF] = data;
        }
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

/*
 * The exposure as the T31 SDK's sensor drivers program it (gc*_set_expo):
 * rows in page 0's 0x03[5:0]:0x04 (GC2053), 0x0D03:0x0D04 (GC2083),
 * 0x0202:0x0203 (GC4653; T41's GC4663 the same); the gain the analog
 * gain table's last stage, 0xB8:0xB9 (0x02B8:0x02B9 on the GC4653), in
 * 1/64 -- the tables' coarse steps sit in other registers, but these
 * track the total.  In the two-exposure (DOL) WDR mode of the GC4653 /
 * GC4663 drivers (gc*_set_integration_time_short) the short frame's rows
 * are 0x0200[5:0]:0x0201, the gain shared by both frames; the model
 * reports them only once a driver has written them (0 otherwise).
 */
bool galaxycore_exposure(Object *sensor, IngenicSensorExposure *e)
{
    GcSensorState *s = (GcSensorState *)
        object_dynamic_cast(sensor, TYPE_GALAXYCORE_SENSOR);
    uint16_t g = 0xB8;

    if (!s) {
        return false;
    }
    if (!s->reg16) {
        e->lines = (s->p0[0x03] & 0x3F) << 8 | s->p0[0x04];
        e->gain = (s->p0[0xB8] << 6 | (s->p0[0xB9] & 0x3F)) * 16;
        e->gain = e->gain ? e->gain : 1024;
        return true;
    } else if (s->id == 0x2083) {
        e->lines = s->applied[0x0D03] << 8 | s->applied[0x0D04];
    } else {
        e->lines = s->applied[0x0202] << 8 | s->applied[0x0203];
        e->short_lines = (s->applied[0x0200] & 0x3F) << 8 |
                         s->applied[0x0201];
        g = 0x02B8;
    }
    e->gain = (s->applied[g] << 6 | (s->applied[g + 1] & 0x3F)) * 16;
    if (!e->gain) {
        e->gain = 1024;
    }
    return true;
}

static void galaxycore_exposure_sensor(Object *sensor, ImageSensorExposure *e)
{
    GcSensorState *s = GALAXYCORE_SENSOR(sensor);
    IngenicSensorExposure x = { 0 };

    galaxycore_exposure(sensor, &x);
    e->lines = x.lines;
    e->gain = x.gain;
    if (s->reg16) {
        e->vts = (s->applied[0x0340] & 0x3F) << 8 | s->applied[0x0341];
    }
}

/*
 * A frame starts: on the 16-bit parts the registers written for it take
 * effect, and the exposure must leave the frame 8 rows
 * (gc4653_2l_cmos.c: u32MaxIntTime = full lines - 8).
 */
static bool galaxycore_frame_start(Object *sensor)
{
    GcSensorState *s = GALAXYCORE_SENSOR(sensor);
    ImageSensorExposure e = { 0 };
    int i;

    if (!s->reg16) {
        return false;           /* the 8-bit parts apply writes at once */
    }
    if (sensor_sync_frame_start(&s->sync, 0, 0)) {
        for (i = 0; i < ARRAY_SIZE(gc_latched); i++) {
            s->applied[gc_latched[i]] = s->regs[gc_latched[i]];
        }
    }
    galaxycore_exposure_sensor(sensor, &e);
    s->sync.vts = e.vts;
    if (e.vts && e.lines + 8 > e.vts) {
        sensor_sync_violation(&s->sync, "exposure %u rows beyond the frame "
                              "(VTS %u - 8)", e.lines, e.vts);
    }
    return true;
}

static void gc_init(Object *obj)
{
    GcSensorState *s = GALAXYCORE_SENSOR(obj);

    sensor_sync_add_properties(obj, &s->sync);
}

static void gc_realize(DeviceState *dev, Error **errp)
{
    GcSensorState *s = GALAXYCORE_SENSOR(dev);

    snprintf(s->name, sizeof(s->name), "galaxycore-%04x", s->id);
    s->sync.name = s->name;
}

static void gc_reset(DeviceState *dev)
{
    GcSensorState *s = GALAXYCORE_SENSOR(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[s->id_reg & gc_mask(s)] = s->id >> 8;
    s->regs[(s->id_reg + 1) & gc_mask(s)] = s->id & 0xFF;
    s->ptr = 0;
    s->naddr = 0;
    s->page = 0;
    memset(s->p0, 0, sizeof(s->p0));
    memcpy(s->applied, s->regs, sizeof(s->applied));
    sensor_sync_reset(&s->sync);
}

/* a stream from before applied[] (version 2): the frame uses what was
 * written */
static int gc_post_load(void *opaque, int version_id)
{
    GcSensorState *s = opaque;

    if (version_id < 3) {
        memcpy(s->applied, s->regs, sizeof(s->applied));
    }
    return 0;
}

static const VMStateDescription vmstate_gc_sensor = {
    .name = TYPE_GALAXYCORE_SENSOR,
    .version_id = 3,
    .minimum_version_id = 1,
    .post_load = gc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, GcSensorState),
        VMSTATE_UINT16(ptr, GcSensorState),
        VMSTATE_UINT8(naddr, GcSensorState),
        VMSTATE_UINT8_ARRAY(regs, GcSensorState, GC_REGS),
        VMSTATE_UINT8_V(page, GcSensorState, 2),
        VMSTATE_UINT8_ARRAY_V(p0, GcSensorState, 0x100, 2),
        VMSTATE_UINT8_ARRAY_V(applied, GcSensorState, GC_REGS, 3),
        VMSTATE_BOOL_V(sync.latching, GcSensorState, 3),
        VMSTATE_UINT32_V(sync.frame, GcSensorState, 3),
        VMSTATE_UINT32_V(sync.violations, GcSensorState, 3),
        VMSTATE_END_OF_LIST()
    },
};

static const Property gc_properties[] = {
    DEFINE_PROP_BOOL("reg16", GcSensorState, reg16, false),
    DEFINE_PROP_UINT16("id-reg", GcSensorState, id_reg, 0xF0),
    DEFINE_PROP_UINT16("id", GcSensorState, id, 0x2053),
    DEFINE_PROP_BOOL("trace", GcSensorState, trace, false),
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
    dc->realize = gc_realize;
    IMAGE_SENSOR_CLASS(klass)->exposure = galaxycore_exposure_sensor;
    IMAGE_SENSOR_CLASS(klass)->frame_start = galaxycore_frame_start;
}

static const TypeInfo gc_sensor_info = {
    .name          = TYPE_GALAXYCORE_SENSOR,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(GcSensorState),
    .instance_init = gc_init,
    .class_init    = gc_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { TYPE_IMAGE_SENSOR },
        { }
    },
};

static void gc_sensor_register_types(void)
{
    type_register_static(&gc_sensor_info);
}

type_init(gc_sensor_register_types)
