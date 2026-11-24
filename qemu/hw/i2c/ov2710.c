/*
 * OmniVision OV2710 / OV2715 image sensor (1080p, 16-bit register
 * addresses, chip ID 0x27 0x10 / 0x15 at 0x300A / 0x300B).
 *
 * Modelled from what the Grain Media driver does with it (OpenIPC's
 * gm8136 release, fisp_ov2715.ko, which keeps its symbols):
 *
 *   exposure  0x3500[3:0]:0x3501:0x3502, rows x 16 (set_exp writes
 *             rows << 4, the low nibble 0)
 *   extra VTS 0x350C:0x350D, rows added to the frame when the exposure
 *             needs more than VTS - 6 (set_exp)
 *   gain      0x350A[0] and 0x350B[7:4] each double, 0x350B[3:0] adds
 *             sixteenths: (1 + 0x350A[0]) x 2^popcount(0x350B[7:4]) x
 *             (1 + 0x350B[3:0] / 16), the driver's own read-back
 *             (set_gain, 64 = 1x)
 *   group     0x3212: 0x00 opens group 0, 0x10 ends it, 0xA0 launches
 *             it; set_gain writes 0x350A / 0x350B inside it
 *   HTS       0x380C[3:0]:0x380D, VTS 0x380E:0x380F (reset 2420 x 1104)
 *   window    0x3808:0x3809 x 0x380A:0x380B
 *   control   0x3008 bit 7 software reset, bit 6 sleep (no stream)
 *
 * The driver keeps VTS (1104) and sets the frame rate with HTS =
 * pclk / fps / VTS, where its pixel clock is 90 MHz from a 27 MHz XCLK
 * (0x3010 / 0x3011 = 0x00 / 0x24 at 20 frames/s and up): "pclk".  The
 * frame is HTS x (VTS + extra) pixel clocks.
 *
 * Exposure, gain and frame length take effect at a frame start
 * (sensor-sync.h) once an ISP model clocks the frames, and the group
 * hold keeps them back until it is launched.
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
#include "qemu/module.h"
#include "qemu/timer.h"

#define TYPE_OV2710_SENSOR "ov2710-sensor"
OBJECT_DECLARE_SIMPLE_TYPE(Ov2710State, OV2710_SENSOR)

#define OV_REGS             0x10000

#define OV_SYS_CTRL         0x3008
#define OV_SYS_RESET        0x80
#define OV_SYS_SLEEP        0x40
#define OV_GROUP            0x3212
#define OV_GROUP_END        0x10
#define OV_GROUP_LAUNCH     0x20

/* the driver's margin: set_exp keeps the exposure VTS - 6 rows */
#define OV_EXP_MARGIN       6

struct Ov2710State {
    I2CSlave parent_obj;

    uint16_t id_reg;
    uint16_t id;
    uint32_t pclk;

    uint16_t ptr;
    uint8_t naddr;
    uint8_t regs[OV_REGS];      /* as written */
    uint8_t applied[OV_REGS];   /* what the frame in progress uses */
    char name[24];
    SensorSync sync;
};

/* registers a frame takes at its start */
static const uint16_t ov_latched[] = {
    0x3500, 0x3501, 0x3502, 0x350A, 0x350B, 0x350C, 0x350D,
    0x380C, 0x380D, 0x380E, 0x380F,
};

/* reset values of the registers the model reads (OV2710 defaults) */
static const struct {
    uint16_t reg;
    uint8_t val;
} ov_defaults[] = {
    { 0x3008, 0x02 },
    { 0x3808, 0x07 }, { 0x3809, 0x80 },     /* 1920 */
    { 0x380A, 0x04 }, { 0x380B, 0x38 },     /* x 1080 */
    { 0x380C, 0x09 }, { 0x380D, 0x74 },     /* HTS 2420 */
    { 0x380E, 0x04 }, { 0x380F, 0x50 },     /* VTS 1104 */
};

static bool ov_is_latched(uint16_t a)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(ov_latched); i++) {
        if (ov_latched[i] == a) {
            return true;
        }
    }
    return false;
}

static void ov_registers_reset(Ov2710State *s)
{
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    for (i = 0; i < ARRAY_SIZE(ov_defaults); i++) {
        s->regs[ov_defaults[i].reg] = ov_defaults[i].val;
    }
    s->regs[s->id_reg] = s->id >> 8;
    s->regs[(s->id_reg + 1) & 0xFFFF] = s->id & 0xFF;
    memcpy(s->applied, s->regs, sizeof(s->applied));
}

static int ov_event(I2CSlave *i2c, enum i2c_event event)
{
    Ov2710State *s = OV2710_SENSOR(i2c);

    if (event == I2C_START_SEND) {
        s->naddr = 0;
    }
    return 0;
}

static void ov_write(Ov2710State *s, uint16_t a, uint8_t v)
{
    sensor_sync_write(&s->sync, a, v);
    if (a == OV_SYS_CTRL && (v & OV_SYS_RESET)) {
        ov_registers_reset(s);
        v &= ~OV_SYS_RESET;
    }
    if (a == OV_GROUP) {
        if (v & OV_GROUP_LAUNCH) {
            sensor_sync_hold(&s->sync, false);
        } else if (!(v & OV_GROUP_END)) {
            sensor_sync_hold(&s->sync, true);
        }
    }
    s->regs[a] = v;
    if (!s->sync.latching || !ov_is_latched(a)) {
        s->applied[a] = v;
    }
}

static int ov_send(I2CSlave *i2c, uint8_t data)
{
    Ov2710State *s = OV2710_SENSOR(i2c);

    if (s->naddr < 2) {
        s->ptr = s->ptr << 8 | data;
        s->naddr++;
        return 0;
    }
    ov_write(s, s->ptr, data);
    s->ptr++;
    return 0;
}

static uint8_t ov_recv(I2CSlave *i2c)
{
    Ov2710State *s = OV2710_SENSOR(i2c);

    if (s->naddr == 1) {
        return 0;       /* an 8-bit probe (ipctool tries one first) */
    }
    return s->regs[s->ptr++];
}

static uint32_t ov_gain(const uint8_t *r)
{
    uint32_t g = 1024 << (r[0x350A] & 1);

    g <<= ctpop8(r[0x350B] >> 4);
    return g * (16 + (r[0x350B] & 0x0F)) / 16;
}

static void ov_exposure(Object *obj, ImageSensorExposure *e)
{
    Ov2710State *s = OV2710_SENSOR(obj);
    const uint8_t *a = s->applied;

    e->lines = ((a[0x3500] & 0x0F) << 16 | a[0x3501] << 8 | a[0x3502]) >> 4;
    e->gain = ov_gain(a);
    e->vts = (a[0x380E] << 8 | a[0x380F]) + (a[0x350C] << 8 | a[0x350D]);
    e->width = (s->regs[0x3808] & 0x0F) << 8 | s->regs[0x3809];
    e->height = (s->regs[0x380A] & 0x0F) << 8 | s->regs[0x380B];
}

static bool ov_streaming(Object *obj)
{
    Ov2710State *s = OV2710_SENSOR(obj);

    return !(s->regs[OV_SYS_CTRL] & OV_SYS_SLEEP);
}

static int64_t ov_frame_ns(Object *obj)
{
    Ov2710State *s = OV2710_SENSOR(obj);
    ImageSensorExposure e = { 0 };
    uint32_t hts = (s->applied[0x380C] & 0x1F) << 8 | s->applied[0x380D];

    ov_exposure(obj, &e);
    if (!ov_streaming(obj) || !s->pclk || !hts || !e.vts) {
        return 0;
    }
    return muldiv64((uint64_t)hts * e.vts, NANOSECONDS_PER_SECOND, s->pclk);
}

static bool ov_frame_start(Object *obj)
{
    Ov2710State *s = OV2710_SENSOR(obj);
    ImageSensorExposure e = { 0 };
    bool pending = false;
    int i;

    for (i = 0; i < ARRAY_SIZE(ov_latched); i++) {
        if (s->applied[ov_latched[i]] != s->regs[ov_latched[i]]) {
            pending = true;
        }
    }
    if (sensor_sync_frame_start(&s->sync, ov_frame_ns(obj), 0, pending)) {
        for (i = 0; i < ARRAY_SIZE(ov_latched); i++) {
            s->applied[ov_latched[i]] = s->regs[ov_latched[i]];
        }
    }
    ov_exposure(obj, &e);
    s->sync.vts = e.vts;
    if (e.vts && e.lines + OV_EXP_MARGIN > e.vts) {
        sensor_sync_violation(&s->sync, "exposure %u rows beyond the frame "
                              "(VTS %u - %u)", e.lines, e.vts,
                              OV_EXP_MARGIN);
    }
    return true;
}

/* "lines" / "vts" / "gain": what the frame in progress uses (qom-get) */
static void ov_get_applied(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    ImageSensorExposure e = { 0 };
    uint32_t value;

    ov_exposure(obj, &e);
    value = !strcmp(name, "lines") ? e.lines :
            !strcmp(name, "vts") ? e.vts : e.gain;
    visit_type_uint32(v, name, &value, errp);
}

static void ov_init(Object *obj)
{
    Ov2710State *s = OV2710_SENSOR(obj);

    sensor_sync_add_properties(obj, &s->sync);
    object_property_add(obj, "lines", "uint32", ov_get_applied, NULL, NULL,
                        NULL);
    object_property_add(obj, "vts", "uint32", ov_get_applied, NULL, NULL,
                        NULL);
    object_property_add(obj, "gain", "uint32", ov_get_applied, NULL, NULL,
                        NULL);
}

static void ov_realize(DeviceState *dev, Error **errp)
{
    Ov2710State *s = OV2710_SENSOR(dev);

    snprintf(s->name, sizeof(s->name), "ov%04x", s->id);
    s->sync.name = s->name;
}

static void ov_reset(DeviceState *dev)
{
    Ov2710State *s = OV2710_SENSOR(dev);

    ov_registers_reset(s);
    s->ptr = 0;
    s->naddr = 0;
    sensor_sync_reset(&s->sync);
}

static const VMStateDescription vmstate_ov2710 = {
    .name = TYPE_OV2710_SENSOR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, Ov2710State),
        VMSTATE_UINT16(ptr, Ov2710State),
        VMSTATE_UINT8(naddr, Ov2710State),
        VMSTATE_UINT8_ARRAY(regs, Ov2710State, OV_REGS),
        VMSTATE_UINT8_ARRAY(applied, Ov2710State, OV_REGS),
        VMSTATE_BOOL(sync.latching, Ov2710State),
        VMSTATE_BOOL(sync.hold, Ov2710State),
        VMSTATE_UINT32(sync.frame, Ov2710State),
        VMSTATE_UINT32(sync.violations, Ov2710State),
        VMSTATE_UINT32(sync.latches, Ov2710State),
        VMSTATE_UINT32(sync.late_latch, Ov2710State),
        VMSTATE_END_OF_LIST()
    },
};

static const Property ov_properties[] = {
    DEFINE_PROP_UINT16("id-reg", Ov2710State, id_reg, 0x300A),
    DEFINE_PROP_UINT16("id", Ov2710State, id, 0x2715),
    DEFINE_PROP_UINT32("pclk", Ov2710State, pclk, 90000000),
};

static void ov_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    ImageSensorClass *ik = IMAGE_SENSOR_CLASS(klass);

    device_class_set_legacy_reset(dc, ov_reset);
    device_class_set_props(dc, ov_properties);
    dc->vmsd = &vmstate_ov2710;
    dc->realize = ov_realize;
    k->event = ov_event;
    k->send = ov_send;
    k->recv = ov_recv;
    ik->exposure = ov_exposure;
    ik->frame_start = ov_frame_start;
    ik->frame_ns = ov_frame_ns;
    ik->streaming = ov_streaming;
}

static const TypeInfo ov2710_info = {
    .name          = TYPE_OV2710_SENSOR,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Ov2710State),
    .instance_init = ov_init,
    .class_init    = ov_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { TYPE_IMAGE_SENSOR },
        { }
    },
};

static void ov2710_register_types(void)
{
    type_register_static(&ov2710_info);
}

type_init(ov2710_register_types)
