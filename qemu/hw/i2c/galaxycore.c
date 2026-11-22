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
 * GC2083, from its datasheet (GC2083 CSP Datasheet V0.0, sections 9.5 and
 * 9.8 and the register list): shutter 0x0D03[5:0]:0x0D04 and frame
 * length 0x0D41[5:0]:0x0D42 in rows, line length 0x0D05[3:0]:0x0D06 in
 * units of two pixel clocks, output window 0x0195..0x0198, column gain
 * 0x00B8[5:0]:0x00B9[5:0] and pregain 0x00B1[3:0]:0x00B2[7:2], both Q6;
 * the registers hold its reset values until a driver writes them.  It
 * streams once the MIPI output enable (0x023E bit 7) is set -- the last
 * write of the init tables (OpenIPC/linux's 4.19 gc2083.c, the HiSilicon
 * gc2083_ctrl.c: 0x99); its frame period is then frame length x line
 * length over the pixel clock ("pclk": 100 MHz in the 24 MHz reference
 * setting those tables use, 1125 rows of 2962 clocks at 30 frames/s).
 * Not modelled: the analog PGA (0x00D0, 0x0DC1), whose curve the
 * datasheet does not give -- below 53x the column gain, which the
 * drivers' gain tables step with it, carries the total; above, their
 * last steps raise only the PGA, so the model stops at 53x times the
 * pregain.
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
#include "qapi/visitor.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/timer.h"

#define TYPE_GALAXYCORE_SENSOR "galaxycore-sensor"
OBJECT_DECLARE_SIMPLE_TYPE(GcSensorState, GALAXYCORE_SENSOR)

#define GC_REGS             0x10000

struct GcSensorState {
    I2CSlave parent_obj;

    bool reg16;
    uint16_t id_reg;
    uint16_t id;
    uint32_t pclk;
    uint16_t hts;
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
    /* GC1054: page 1's analog gain, page 3's MIPI control (gc1054_*) */
    uint8_t p1[0x100];
    uint8_t p3[0x100];
    /*
     * The 16-bit parts: what the frame in progress uses of the registers
     * a frame takes at its start (gc_latched[]); the rest as written.
     */
    uint8_t applied[GC_REGS];
    /*
     * GC2083 "vts-delay": the frame length (0x0D41, 0x0D42) latched up to
     * GC2083_VTS_DELAY_MAX frames after the rest; vts_q[0] is what the
     * last frame start took in, vts_q[n - 1] what the next one applies.
     */
    uint32_t vts_delay;
    uint8_t vts_q[2][2];
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
    /* GC2083: its exposure, gain and frame length, where
     * galaxycore_exposure() reads them */
    0x0D03, 0x0D04, 0x00B8, 0x00B9, 0x00B1, 0x00B2, 0x0D41, 0x0D42,
};

#define GC2083_ID           0x2083
#define GC2083_VTS_DELAY_MAX 2

static bool gc_vts_delayed(GcSensorState *s, uint16_t a)
{
    return s->vts_delay && s->id == GC2083_ID &&
           (a == 0x0D41 || a == 0x0D42);
}

/* GC2083's reset values of the registers the model reads (datasheet) */
static const struct {
    uint16_t reg;
    uint8_t val;
} gc2083_defaults[] = {
    { 0x0D03, 0x00 }, { 0x0D04, 0x10 },     /* shutter 16 */
    { 0x0D05, 0x04 }, { 0x0D06, 0x48 },     /* line length 2 x 1096 */
    { 0x0D41, 0x04 }, { 0x0D42, 0x65 },     /* frame length 1125 */
    { 0x0195, 0x04 }, { 0x0196, 0x38 },     /* window 1080 rows */
    { 0x0197, 0x07 }, { 0x0198, 0x38 },     /* by 1848 columns */
    { 0x00B1, 0x01 }, { 0x00B2, 0x00 },     /* pregain 1x */
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
        if (s->ptr == 0x0D41 || s->ptr == 0x0D42) {
            /* before streaming: at once, the whole pipeline */
            s->vts_q[0][s->ptr - 0x0D41] = data;
            s->vts_q[1][s->ptr - 0x0D41] = data;
        }
    }
    if (!s->reg16) {
        if (s->ptr == 0xFE) {
            s->page = data & 7;
        } else if (!s->page) {
            s->p0[s->ptr & 0xFF] = data;
        } else if (s->page == 1) {
            s->p1[s->ptr & 0xFF] = data;
        } else if (s->page == 3) {
            s->p3[s->ptr & 0xFF] = data;
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
    val = (!s->reg16 && !s->page && s->ptr != 0xFE) ?
          s->p0[s->ptr & 0xFF] : s->regs[s->ptr];
    s->ptr = (s->ptr + 1) & gc_mask(s);
    return val;
}

/*
 * The exposure as the T31 SDK's sensor drivers program it (gc*_set_expo):
 * rows in page 0's 0x03[5:0]:0x04 (GC2053), 0x0D03:0x0D04 (GC2083),
 * 0x0202:0x0203 (GC4653; T41's GC4663 the same); the gain the analog
 * gain table's last stage, 0xB8:0xB9 (0x02B8:0x02B9 on the GC4653), in
 * 1/64 -- the tables' coarse steps sit in other registers, but these
 * track the total; on the GC2083 times its pregain (see the top).  In
 * the two-exposure (DOL) WDR mode of the GC4653 / GC4663 drivers
 * (gc*_set_integration_time_short) the short frame's rows
 * are 0x0200[5:0]:0x0201, the gain shared by both frames; the model
 * reports them only once a driver has written them (0 otherwise).
 */
/*
 * GC1054, as OpenIPC's 4.19 gc1054.c drives it: the gain is a bracket,
 * page 1's 0xB6, times a Q6 fine step in 0xB1[5:0]:0xB2[7:2], each
 * bracket's base (gain at 0xB6 = n, step 64) in its gc1054_gain_table.
 */
static const uint16_t gc1054_gain_base[] = {
    0x040, 0x05B, 0x07F, 0x0B6, 0x101, 0x171, 0x203, 0x2E1, 0x407, 0x5D3,
};

static uint32_t gc1054_gain(GcSensorState *s)
{
    unsigned b = s->p1[0xB6] < ARRAY_SIZE(gc1054_gain_base) ?
                 s->p1[0xB6] : ARRAY_SIZE(gc1054_gain_base) - 1;
    uint32_t q = (s->p1[0xB1] & 0x3F) << 6 | s->p1[0xB2] >> 2;

    /* Q6 total, then 1.0 = 1024; before any write, 1x */
    return q ? q * gc1054_gain_base[b] / 64 * 16 : 1024;
}

bool galaxycore_exposure(Object *sensor, IngenicSensorExposure *e)
{
    GcSensorState *s = (GcSensorState *)
        object_dynamic_cast(sensor, TYPE_GALAXYCORE_SENSOR);
    uint16_t g = 0xB8;

    if (!s) {
        return false;
    }
    if (!s->reg16 && s->id == 0x1054) {
        e->lines = (s->p0[0x03] & 0x1F) << 8 | s->p0[0x04];
        e->gain = gc1054_gain(s);
        return true;
    }
    if (!s->reg16) {
        e->lines = (s->p0[0x03] & 0x3F) << 8 | s->p0[0x04];
        e->gain = (s->p0[0xB8] << 6 | (s->p0[0xB9] & 0x3F)) * 16;
        e->gain = e->gain ? e->gain : 1024;
        return true;
    } else if (s->id == GC2083_ID) {
        uint32_t col = (s->applied[0x00B8] & 0x3F) << 6 |
                       (s->applied[0x00B9] & 0x3F);
        uint32_t pre = (s->applied[0x00B1] & 0x0F) << 6 |
                       s->applied[0x00B2] >> 2;

        e->lines = (s->applied[0x0D03] & 0x3F) << 8 | s->applied[0x0D04];
        /* Q6 x Q6, then 1.0 = 1024; a gain left at 0 is 1x */
        e->gain = (col ? col : 64) * (pre ? pre : 64) / 4;
        return true;
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
    if (s->id == GC2083_ID && s->reg16) {
        e->vts = (s->applied[0x0D41] & 0x3F) << 8 | s->applied[0x0D42];
        e->width = (s->regs[0x0197] & 0x0F) << 8 | s->regs[0x0198];
        e->height = (s->regs[0x0195] & 0x07) << 8 | s->regs[0x0196];
    } else if (s->reg16) {
        e->vts = (s->applied[0x0340] & 0x3F) << 8 | s->applied[0x0341];
    } else if (s->id == 0x1054) {
        /* 720 rows, VB in 0x07[3:0]:0x08, and 16 more (gc1054.c) */
        e->vts = 720 + ((s->p0[0x07] & 0x0F) << 8 | s->p0[0x08]) + 16;
        e->width = 1280;
        e->height = 720;
    } else if (s->id == 0x2053 || s->id == 0x2063) {
        e->vts = (s->p0[0x41] & 0x3F) << 8 | s->p0[0x42];
    }
}

/*
 * GC1054: a row is 1732 periods of its 38.8 MHz timing clock (gc1054.c),
 * so VB 160 -- the driver's default -- is 25 frames/s.  0 until the
 * driver has started the stream (page 3's 0x10 = 0x90, MIPI on).
 * GC2083: 0 until its MIPI output is on (0x023E bit 7), then its frame
 * length times its line length (0x0D05:0x0D06, x2) over "pclk".
 * The other parts: HTS x VTS over the pixel clock, when a machine gives
 * both ("pclk", "hts"); 0 otherwise.
 */
static int64_t galaxycore_frame_ns(Object *sensor)
{
    GcSensorState *s = GALAXYCORE_SENSOR(sensor);
    ImageSensorExposure e = { 0 };

    if (!s->reg16 && s->id == 0x1054) {
        if (!(s->p3[0x10] & 0x80)) {
            return 0;
        }
        galaxycore_exposure_sensor(sensor, &e);
        return muldiv64((uint64_t)e.vts * 1732, NANOSECONDS_PER_SECOND,
                        38800000);
    }
    galaxycore_exposure_sensor(sensor, &e);
    if (s->reg16 && s->id == GC2083_ID) {
        uint32_t hts = ((s->regs[0x0D05] & 0x0F) << 8 | s->regs[0x0D06]) * 2;

        if (!(s->regs[0x023E] & 0x80) || !s->pclk || !hts || !e.vts) {
            return 0;
        }
        return muldiv64((uint64_t)hts * e.vts, NANOSECONDS_PER_SECOND,
                        s->pclk);
    }
    if (!s->pclk || !s->hts || !e.vts) {
        return 0;
    }
    return (int64_t)s->hts * e.vts * NANOSECONDS_PER_SECOND / s->pclk;
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
    bool pending = false;
    int i;

    if (!s->reg16) {
        return false;           /* the 8-bit parts apply writes at once */
    }
    for (i = 0; i < ARRAY_SIZE(gc_latched); i++) {
        if (!gc_vts_delayed(s, gc_latched[i]) &&
            s->applied[gc_latched[i]] != s->regs[gc_latched[i]]) {
            pending = true;
        }
    }
    if (sensor_sync_frame_start(&s->sync, 0, 0, pending)) {
        for (i = 0; i < ARRAY_SIZE(gc_latched); i++) {
            if (!gc_vts_delayed(s, gc_latched[i])) {
                s->applied[gc_latched[i]] = s->regs[gc_latched[i]];
            }
        }
        if (gc_vts_delayed(s, 0x0D41)) {
            /* the frame length a frame (or two) behind the rest */
            unsigned n = MIN(s->vts_delay, GC2083_VTS_DELAY_MAX);

            s->applied[0x0D41] = s->vts_q[n - 1][0];
            s->applied[0x0D42] = s->vts_q[n - 1][1];
            memcpy(s->vts_q[1], s->vts_q[0], 2);
            s->vts_q[0][0] = s->regs[0x0D41];
            s->vts_q[0][1] = s->regs[0x0D42];
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

/* "lines" / "vts": the exposure and frame length the frame in progress
 * uses (qom-get) */
static void gc_get_applied(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    ImageSensorExposure e = { 0 };
    uint32_t value;

    galaxycore_exposure_sensor(obj, &e);
    value = !strcmp(name, "lines") ? e.lines : e.vts;
    visit_type_uint32(v, name, &value, errp);
}

static void gc_init(Object *obj)
{
    GcSensorState *s = GALAXYCORE_SENSOR(obj);

    sensor_sync_add_properties(obj, &s->sync);
    object_property_add(obj, "lines", "uint32", gc_get_applied, NULL, NULL,
                        NULL);
    object_property_add(obj, "vts", "uint32", gc_get_applied, NULL, NULL,
                        NULL);
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
    memset(s->p1, 0, sizeof(s->p1));
    memset(s->p3, 0, sizeof(s->p3));
    if (!s->reg16) {
        s->p0[s->id_reg & 0xFF] = s->id >> 8;
        s->p0[(s->id_reg + 1) & 0xFF] = s->id & 0xFF;
    }
    if (s->reg16 && s->id == GC2083_ID) {
        int i;

        for (i = 0; i < ARRAY_SIZE(gc2083_defaults); i++) {
            s->regs[gc2083_defaults[i].reg] = gc2083_defaults[i].val;
        }
    }
    memcpy(s->applied, s->regs, sizeof(s->applied));
    s->vts_q[0][0] = s->vts_q[1][0] = s->regs[0x0D41];
    s->vts_q[0][1] = s->vts_q[1][1] = s->regs[0x0D42];
    sensor_sync_reset(&s->sync);
}

/* a stream from before applied[] (version 2): the frame uses what was
 * written */
static int gc_post_load(void *opaque, int version_id)
{
    GcSensorState *s = opaque;

    if (version_id < 2 && !s->reg16) {
        memcpy(s->p0, s->regs, sizeof(s->p0));
    }
    if (version_id < 3) {
        memcpy(s->applied, s->regs, sizeof(s->applied));
    }
    if (version_id < 6) {
        s->vts_q[0][0] = s->vts_q[1][0] = s->applied[0x0D41];
        s->vts_q[0][1] = s->vts_q[1][1] = s->applied[0x0D42];
    }
    return 0;
}

static const VMStateDescription vmstate_gc_sensor = {
    .name = TYPE_GALAXYCORE_SENSOR,
    .version_id = 6,
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
        VMSTATE_UINT32_V(sync.latches, GcSensorState, 4),
        VMSTATE_UINT32_V(sync.late_latch, GcSensorState, 4),
        VMSTATE_UINT8_ARRAY_V(p1, GcSensorState, 0x100, 5),
        VMSTATE_UINT8_ARRAY_V(p3, GcSensorState, 0x100, 5),
        VMSTATE_UINT8_2DARRAY_V(vts_q, GcSensorState, 2, 2, 6),
        VMSTATE_END_OF_LIST()
    },
};

static const Property gc_properties[] = {
    DEFINE_PROP_BOOL("reg16", GcSensorState, reg16, false),
    DEFINE_PROP_UINT16("id-reg", GcSensorState, id_reg, 0xF0),
    DEFINE_PROP_UINT16("id", GcSensorState, id, 0x2053),
    DEFINE_PROP_UINT32("pclk", GcSensorState, pclk, 0),
    DEFINE_PROP_UINT16("hts", GcSensorState, hts, 0),
    DEFINE_PROP_BOOL("trace", GcSensorState, trace, false),
    /*
     * GC2083 only, opt in: its frame length takes effect 1 or 2 frames
     * after the exposure and gain written with it.  Not what the chip is
     * known to do -- the datasheet gives no latch timing, and the public
     * drivers disagree (SigmaStar: shutter and gain 2; the xmedia
     * gk7206v100 driver: exposure and gain 2, VMAX 3) -- but a what-if
     * for an AE loop's write order: does its exposure stay in the frame?
     */
    DEFINE_PROP_UINT32("vts-delay", GcSensorState, vts_delay, 0),
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
    IMAGE_SENSOR_CLASS(klass)->frame_ns = galaxycore_frame_ns;
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
