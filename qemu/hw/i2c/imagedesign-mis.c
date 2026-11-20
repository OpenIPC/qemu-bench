/*
 * ImageDesign MIS2008 / MIS2009 image sensor (one 1936x1096 die; the 2009
 * adds DVP).  16-bit register addresses, 8-bit data, I2C 0x30.
 *
 * The register map is the drivers': the GK7205V510 vendor App's
 * mis2009_cmos / mis2009_linear_2M30_init (Zenointel SD-2N-4G "hunter"),
 * OpenIPC's libsns_mis2009.so and openhisilicon's imagedesign_mis2009
 * (hi3516ev200), which agree:
 *
 *   0x3000:0x3001  chip ID, 0x2008 for both parts -- what ipctool reads on
 *                  the SD-2N-4G ("mis2008_i2c"); every MIS2009 driver
 *                  checks for it
 *   0x3006         bit 1: standby (the init table opens with 0x02 and
 *                  ends with 0x00, streaming)
 *   0x3100:0x3101  integration time, rows
 *   0x3102         analog gain, an index into the drivers' table:
 *                  (0x400 << i / 32) + (0x20 << i / 32) * (i % 32), 1/1024,
 *                  128 entries, 1x to 15.75x
 *   0x3200:0x3201  frame length (VMAX), rows; the vendor's inttime_update
 *                  reads it back every frame, so writes must stick
 *   0x3202:0x3203  line length
 *   0x3204..0x320B row start / end, column start / end, 11 bits each
 *
 * The line rate is fixed by the PLL setup: 33780 rows/s, VMAX 1126 at
 * 30 fps (MIS2009_LINE_RATE).  Exposure, gain and VMAX are written every
 * frame (the drivers' sync list, with 0x3A02 / 0x3A07 / 0x3A1C) and take
 * effect at the next frame start once an ISP model drives the frames
 * (sensor-sync.h).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/image-sensor.h"
#include "hw/i2c/sensor-sync.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/visitor.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define TYPE_IMAGEDESIGN_MIS "imagedesign-mis"
OBJECT_DECLARE_SIMPLE_TYPE(MisSensorState, IMAGEDESIGN_MIS)

#define MIS_REGS            0x10000
#define MIS_LINE_RATE       33780       /* rows per second */
#define MIS_EXP_MARGIN      2           /* exposure <= VMAX - 2 (drivers) */

struct MisSensorState {
    I2CSlave parent_obj;

    uint16_t id;
    bool trace;

    uint16_t ptr;
    uint8_t naddr;
    uint8_t regs[MIS_REGS];     /* as written */
    uint8_t applied[MIS_REGS];  /* what the frame in progress uses */
    char name[24];
    SensorSync sync;
};

/* What the drivers write every frame and the sensor takes at frame start */
static const uint16_t mis_latched[] = {
    0x3100, 0x3101, 0x3102, 0x3200, 0x3201, 0x3A02, 0x3A07, 0x3A1C,
};

static bool mis_is_latched(uint16_t a)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(mis_latched); i++) {
        if (mis_latched[i] == a) {
            return true;
        }
    }
    return false;
}

static int mis_event(I2CSlave *i2c, enum i2c_event event)
{
    MisSensorState *s = IMAGEDESIGN_MIS(i2c);

    if (event == I2C_START_SEND) {
        s->naddr = 0;
        s->ptr = 0;
    }
    return 0;
}

static int mis_send(I2CSlave *i2c, uint8_t data)
{
    MisSensorState *s = IMAGEDESIGN_MIS(i2c);

    if (s->naddr < 2) {
        s->ptr = s->ptr << 8 | data;
        s->naddr++;
        return 0;
    }
    if (s->trace) {
        qemu_log("%s: [0x%04x] = 0x%02x\n", s->name, s->ptr, data);
    }
    if (s->ptr == 0x3000 || s->ptr == 0x3001) {
        s->ptr++;               /* the chip ID is read-only */
        return 0;
    }
    sensor_sync_write(&s->sync, s->ptr, data);
    if (!s->sync.latching || !mis_is_latched(s->ptr)) {
        s->applied[s->ptr] = data;
    }
    s->regs[s->ptr] = data;
    s->ptr++;
    return 0;
}

static uint8_t mis_recv(I2CSlave *i2c)
{
    MisSensorState *s = IMAGEDESIGN_MIS(i2c);
    uint8_t val;

    if (s->naddr == 1) {
        return 0;               /* an 8-bit probe: nothing here */
    }
    val = s->regs[s->ptr];
    s->ptr++;
    return val;
}

static uint32_t mis_again(uint8_t idx)
{
    unsigned i = MIN(idx, 127);

    return (0x400u << (i / 32)) + (0x20u << (i / 32)) * (i % 32);
}

static uint16_t mis_reg16(const uint8_t *r, uint16_t a, uint16_t mask)
{
    return (r[a] << 8 | r[a + 1]) & mask;
}

static void mis_exposure(Object *sensor, ImageSensorExposure *e)
{
    MisSensorState *s = IMAGEDESIGN_MIS(sensor);
    const uint8_t *r = s->applied;
    uint16_t r0 = mis_reg16(r, 0x3204, 0x7FF), r1 = mis_reg16(r, 0x3206, 0x7FF);
    uint16_t c0 = mis_reg16(r, 0x3208, 0x7FF), c1 = mis_reg16(r, 0x320A, 0x7FF);

    e->lines = mis_reg16(r, 0x3100, 0xFFFF);
    e->vts = mis_reg16(r, 0x3200, 0xFFFF);
    e->gain = mis_again(r[0x3102]);
    e->width = c1 > c0 ? c1 - c0 + 1 : 0;
    e->height = r1 > r0 ? r1 - r0 + 1 : 0;
}

/* VMAX rows at the fixed line rate; 0 in standby or before the init table */
static int64_t mis_frame_ns(Object *sensor)
{
    MisSensorState *s = IMAGEDESIGN_MIS(sensor);
    ImageSensorExposure e = { 0 };

    if (s->regs[0x3006] & 0x02) {
        return 0;
    }
    mis_exposure(sensor, &e);
    if (!e.vts) {
        return 0;
    }
    return muldiv64(e.vts, NANOSECONDS_PER_SECOND, MIS_LINE_RATE);
}

static bool mis_frame_start(Object *sensor)
{
    MisSensorState *s = IMAGEDESIGN_MIS(sensor);
    ImageSensorExposure e = { 0 };
    bool pending = false;
    int i;

    for (i = 0; i < ARRAY_SIZE(mis_latched); i++) {
        if (s->applied[mis_latched[i]] != s->regs[mis_latched[i]]) {
            pending = true;
        }
    }
    if (sensor_sync_frame_start(&s->sync, mis_frame_ns(sensor), 0, pending)) {
        for (i = 0; i < ARRAY_SIZE(mis_latched); i++) {
            s->applied[mis_latched[i]] = s->regs[mis_latched[i]];
        }
    }
    mis_exposure(sensor, &e);
    s->sync.vts = e.vts;
    if (e.vts && e.lines + MIS_EXP_MARGIN > e.vts) {
        sensor_sync_violation(&s->sync, "exposure %u rows beyond the frame "
                              "(VMAX %u - %u)", e.lines, e.vts,
                              MIS_EXP_MARGIN);
    }
    if (s->regs[0x3102] > 127) {
        sensor_sync_violation(&s->sync, "analog gain index %u beyond the "
                              "table (127)", s->regs[0x3102]);
    }
    return true;
}

/* "lines" / "vts" / "gain": what the frame in progress uses (qom-get) */
static void mis_get_applied(Object *obj, Visitor *v, const char *name,
                            void *opaque, Error **errp)
{
    ImageSensorExposure e = { 0 };
    uint32_t value;

    mis_exposure(obj, &e);
    value = !strcmp(name, "lines") ? e.lines :
            !strcmp(name, "vts") ? e.vts : e.gain;
    visit_type_uint32(v, name, &value, errp);
}

static void mis_init(Object *obj)
{
    MisSensorState *s = IMAGEDESIGN_MIS(obj);

    sensor_sync_add_properties(obj, &s->sync);
    object_property_add(obj, "lines", "uint32", mis_get_applied, NULL, NULL,
                        NULL);
    object_property_add(obj, "vts", "uint32", mis_get_applied, NULL, NULL,
                        NULL);
    object_property_add(obj, "gain", "uint32", mis_get_applied, NULL, NULL,
                        NULL);
}

static void mis_realize(DeviceState *dev, Error **errp)
{
    MisSensorState *s = IMAGEDESIGN_MIS(dev);

    snprintf(s->name, sizeof(s->name), "mis-%04x", s->id);
    s->sync.name = s->name;
}

static void mis_reset(DeviceState *dev)
{
    MisSensorState *s = IMAGEDESIGN_MIS(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x3000] = s->id >> 8;
    s->regs[0x3001] = s->id & 0xFF;
    s->regs[0x3004] = 0x00;     /* BROADCAST_ID, reset 0x0004 (datasheet) */
    s->regs[0x3005] = 0x04;
    s->regs[0x3006] = 0x02;     /* standby until the driver starts it */
    memcpy(s->applied, s->regs, sizeof(s->applied));
    s->ptr = 0;
    s->naddr = 0;
    sensor_sync_reset(&s->sync);
}

static const VMStateDescription vmstate_mis_sensor = {
    .name = TYPE_IMAGEDESIGN_MIS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MisSensorState),
        VMSTATE_UINT16(ptr, MisSensorState),
        VMSTATE_UINT8(naddr, MisSensorState),
        VMSTATE_UINT8_ARRAY(regs, MisSensorState, MIS_REGS),
        VMSTATE_UINT8_ARRAY(applied, MisSensorState, MIS_REGS),
        VMSTATE_BOOL(sync.latching, MisSensorState),
        VMSTATE_UINT32(sync.frame, MisSensorState),
        VMSTATE_UINT32(sync.violations, MisSensorState),
        VMSTATE_UINT32(sync.latches, MisSensorState),
        VMSTATE_UINT32(sync.late_latch, MisSensorState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property mis_properties[] = {
    DEFINE_PROP_UINT16("id", MisSensorState, id, 0x2008),
    DEFINE_PROP_BOOL("trace", MisSensorState, trace, false),
};

static void mis_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, mis_reset);
    device_class_set_props(dc, mis_properties);
    dc->vmsd = &vmstate_mis_sensor;
    dc->realize = mis_realize;
    k->event = mis_event;
    k->send = mis_send;
    k->recv = mis_recv;
    IMAGE_SENSOR_CLASS(klass)->exposure = mis_exposure;
    IMAGE_SENSOR_CLASS(klass)->frame_start = mis_frame_start;
    IMAGE_SENSOR_CLASS(klass)->frame_ns = mis_frame_ns;
}

static const TypeInfo mis_sensor_info = {
    .name          = TYPE_IMAGEDESIGN_MIS,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MisSensorState),
    .instance_init = mis_init,
    .class_init    = mis_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { TYPE_IMAGE_SENSOR },
        { }
    },
};

static void mis_sensor_register_types(void)
{
    type_register_static(&mis_sensor_info);
}

type_init(mis_sensor_register_types)
