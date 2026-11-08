/*
 * SmartSens (SC*) image sensor I2C stub.
 *
 * Generic model for SmartSens sensors that share the same probe protocol:
 * 16-bit register addresses, 8-bit data, ID at 0x3107/0x3108 with optional
 * discriminator at 0x3109.
 *
 * Sofia (XiongMai) detects these sensors at I2C addr 0x60 (8-bit) =
 * 0x30 (7-bit slave addr).  All variants are identified by a unique
 * 16-bit ID composed of regs 0x3107 (high) and 0x3108 (low).  Several
 * variants share the same ID (e.g. SC2235P and SC2235E both report
 * 0x2232) and are disambiguated by the value of reg 0x3109.
 *
 * The instance properties select the variant:
 *   id_high       — value returned for reg 0x3107
 *   id_low        — value returned for reg 0x3108
 *   disc          — value returned for reg 0x3109 (discriminator)
 *
 * Sofia variants currently supported (instance type names):
 *   hisi-sc2235p   ID 0x2232  disc 0x01
 *   hisi-sc2235e   ID 0x2232  disc 0x20
 *   hisi-sc2315    ID 0x2311  disc 0x00
 *   hisi-sc2315e   ID 0x2238  disc 0x00
 *   hisi-sc2335    ID 0xCB14  disc 0x00
 *   hisi-sc2239    ID 0xCB10  disc 0x00
 *   hisi-sc307h    ID 0xCB1C  disc 0x00
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/image-sensor.h"
#include "hw/i2c/ingenic-sensor.h"
#include "hw/i2c/sensor-sync.h"
#include "hw/qdev-properties.h"
#include "qapi/visitor.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define TYPE_HISI_SMARTSENS "hisi-smartsens"
OBJECT_DECLARE_SIMPLE_TYPE(HisiSmartSensState, HISI_SMARTSENS)

#define SS_REG_BASE  0x3000
#define SS_REG_COUNT 0x1000

struct HisiSmartSensState {
    I2CSlave parent_obj;

    /* Variant selection (set via qdev properties) */
    uint8_t  id_high;
    uint8_t  id_low;
    uint8_t  disc;
    uint8_t  gain_fine;    /* 0x3E09 at 1x; 0: the SC2336's gain registers */

    uint32_t pclk;         /* pixel clock: HTS x VTS / pclk is the frame */

    /* I2C state */
    uint8_t  reg_buf[2];   /* register address accumulator (big-endian) */
    uint8_t  reg_ptr;      /* bytes received for register address */
    uint16_t cur_reg;      /* current register address */
    uint8_t  regs[SS_REG_COUNT];        /* what the driver wrote */
    uint8_t  applied[SS_REG_COUNT];     /* what the frame in progress uses */
    bool     streaming;    /* 0x0100 bit 0 */
    char     name[16];
    SensorSync sync;
};

/*
 * Registers a frame takes at its start, all together, the ones the
 * HiSilicon drivers write every frame (sc2315e_cmos.c's sync list):
 * exposure, analog / digital gain, frame length and the two analog
 * settings that follow the gain.  0x3812 opens (0x00) and closes (0x30) a
 * group hold around them.
 */
static const uint16_t ss_latched[] = {
    0x3E00, 0x3E01, 0x3E02, 0x3E06, 0x3E07, 0x3E08, 0x3E09,
    0x320E, 0x320F, 0x3301, 0x3632,
};
#define SS_GROUP_HOLD       0x3812

/* Registers that set the mode: changing them while streaming corrupts
 * frames on silicon (window, HTS, MIPI, PLL, bit depth) */
static bool ss_mode_reg(uint16_t a)
{
    return (a >= 0x3200 && a <= 0x320D) || a == 0x3018 || a == 0x3019 ||
           a == 0x3031 || a == 0x36E9 || a == 0x36F9;
}

static bool ss_is_latched(uint16_t a)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(ss_latched); i++) {
        if (ss_latched[i] == a) {
            return true;
        }
    }
    return false;
}

static void hisi_smartsens_reset_regs(HisiSmartSensState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    /* Sofia checks reg 0x3107/0x3108 (ID) and optionally reg 0x3109 (discriminator) */
    s->regs[0x3107 - SS_REG_BASE] = s->id_high;
    s->regs[0x3108 - SS_REG_BASE] = s->id_low;
    s->regs[0x3109 - SS_REG_BASE] = s->disc;
    memcpy(s->applied, s->regs, sizeof(s->applied));
}

static int hisi_smartsens_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiSmartSensState *s = HISI_SMARTSENS(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->reg_ptr = 0;
        break;
    case I2C_START_RECV:
        s->cur_reg = ((uint16_t)s->reg_buf[0] << 8) | s->reg_buf[1];
        break;
    default:
        break;
    }
    return 0;
}

static int hisi_smartsens_send(I2CSlave *i2c, uint8_t data)
{
    HisiSmartSensState *s = HISI_SMARTSENS(i2c);

    if (s->reg_ptr < 2) {
        s->reg_buf[s->reg_ptr++] = data;
    } else {
        uint16_t addr = ((uint16_t)s->reg_buf[0] << 8) | s->reg_buf[1];

        sensor_sync_write(&s->sync, addr, data);
        if (addr == 0x0100) {
            s->streaming = data & 1;
        } else if (addr >= SS_REG_BASE &&
                   addr < SS_REG_BASE + SS_REG_COUNT) {
            if (s->streaming && ss_mode_reg(addr) &&
                s->regs[addr - SS_REG_BASE] != data) {
                sensor_sync_violation(&s->sync, "mode register %04x changed "
                                      "while streaming (%02x -> %02x)", addr,
                                      s->regs[addr - SS_REG_BASE], data);
            }
            s->regs[addr - SS_REG_BASE] = data;
            /* a register the frame latches waits for the next frame */
            if (!s->sync.latching || !ss_is_latched(addr)) {
                s->applied[addr - SS_REG_BASE] = data;
            }
            if (addr == SS_GROUP_HOLD) {
                sensor_sync_hold(&s->sync, data == 0x00);
            }
        }
        s->reg_buf[1]++;
        if (s->reg_buf[1] == 0) {
            s->reg_buf[0]++;
        }
    }
    return 0;
}

static uint8_t hisi_smartsens_recv(I2CSlave *i2c)
{
    HisiSmartSensState *s = HISI_SMARTSENS(i2c);
    uint8_t val = 0xFF;

    if (s->cur_reg >= SS_REG_BASE &&
        s->cur_reg < SS_REG_BASE + SS_REG_COUNT) {
        val = s->regs[s->cur_reg - SS_REG_BASE];
    }
    s->cur_reg++;
    return val;
}

/*
 * The exposure as the T31 SDK's sensor_sc2336 programs it (sc2336_set_expo):
 * rows in 0x3E00[3:0]:0x3E01:0x3E02[7:4], the analog gain's coarse step in
 * 0x3E09 (0x00 1x, 0x08 2x, 0x09 4x, 0x0B 8x, 0x0F 16x, 0x1F 32x) and its
 * fine part in 0x3E07, 0x80 = 1.0.
 */
bool smartsens_exposure(Object *sensor, IngenicSensorExposure *e)
{
    HisiSmartSensState *s = (HisiSmartSensState *)
        object_dynamic_cast(sensor, TYPE_HISI_SMARTSENS);
    uint32_t coarse;

    if (!s) {
        return false;
    }
#define R(a) s->applied[(a) - SS_REG_BASE]
    e->lines = (R(0x3E00) & 0xF) << 12 | R(0x3E01) << 4 | R(0x3E02) >> 4;
    switch (R(0x3E09)) {
    case 0x08: coarse = 2; break;
    case 0x09: coarse = 4; break;
    case 0x0B: coarse = 8; break;
    case 0x0F: coarse = 16; break;
    case 0x1F: coarse = 32; break;
    default:   coarse = 1; break;
    }
    e->gain = coarse * (R(0x3E07) ? R(0x3E07) : 0x80) * 8;
#undef R
    return true;
}

/*
 * The frame around that exposure: 0x320E/F the frame length, 0x3208-B the
 * output window, as every SmartSens driver's mode tables set them.
 */
static void smartsens_sensor_exposure(Object *sensor, ImageSensorExposure *e)
{
    HisiSmartSensState *s = HISI_SMARTSENS(sensor);
    IngenicSensorExposure x = { 0 };

#define R(a) s->applied[(a) - SS_REG_BASE]
#define R16(a) (R(a) << 8 | R((a) + 1))
    smartsens_exposure(sensor, &x);
    e->lines = x.lines;
    e->gain = x.gain;
    if (s->gain_fine) {
        /*
         * The SC2310's pCus_GetAEGain, with the half rows its
         * pCus_SetAEUSecs writes (capped at twice the frame less 6):
         * coarse 0x3E08[4:2] + 1 (bit 5: HCG, x2.72), fine 0x3E09, digital
         * 0x3E07 / 0x80 and 0x3E06[3:0] + 1.
         */
        uint8_t fine = R(0x3E09) ? R(0x3E09) : s->gain_fine;
        uint8_t dig = R(0x3E07) ? R(0x3E07) : 0x80;
        double gain = (((R(0x3E08) >> 2) & 7) + 1) *
                      ((double)fine / s->gain_fine) * (dig / 128.0) *
                      ((R(0x3E06) & 0xF) + 1) *
                      (R(0x3E08) & 0x20 ? 2.72 : 1.0);

        e->lines = x.lines / 2;
        e->gain = gain * 1024;
    }
    e->vts = R16(0x320E);
    e->width = R16(0x3208);
    e->height = R16(0x320A);
#undef R16
#undef R
}

static int64_t smartsens_frame_ns(Object *sensor)
{
    HisiSmartSensState *s = HISI_SMARTSENS(sensor);
#define R16(a) (s->applied[(a) - SS_REG_BASE] << 8 | \
                s->applied[(a) + 1 - SS_REG_BASE])
    const uint64_t hts = R16(0x320C), vts = R16(0x320E);
#undef R16

    if (!s->pclk || !hts || !vts) {
        return 0;
    }
    return hts * vts * NANOSECONDS_PER_SECOND / s->pclk;
}

/*
 * A frame starts: the registers written for it take effect, unless a
 * group hold is still open; then the datasheet's rules on the frame
 * (sc2315e_cmos.c: the exposure is in half rows, at most 2 x VTS - 4).
 */
static bool smartsens_frame_start(Object *sensor)
{
    HisiSmartSensState *s = HISI_SMARTSENS(sensor);
    ImageSensorExposure e = { 0 };
    IngenicSensorExposure raw = { 0 };
    int i;

    if (sensor_sync_frame_start(&s->sync, smartsens_frame_ns(sensor),
                                s->applied[0x320E - SS_REG_BASE] << 8 |
                                s->applied[0x320F - SS_REG_BASE])) {
        for (i = 0; i < ARRAY_SIZE(ss_latched); i++) {
            s->applied[ss_latched[i] - SS_REG_BASE] =
                s->regs[ss_latched[i] - SS_REG_BASE];
        }
    }
    s->sync.vts = s->applied[0x320E - SS_REG_BASE] << 8 |
                  s->applied[0x320F - SS_REG_BASE];
    s->sync.frame_ns = smartsens_frame_ns(sensor);
    if (!s->streaming) {
        return true;
    }
    IMAGE_SENSOR_GET_CLASS(sensor)->exposure(sensor, &e);
    /* the registers count half rows (decoded before any conversion) */
    smartsens_exposure(sensor, &raw);
    if (e.vts && raw.lines + 4 > 2 * e.vts) {
        sensor_sync_violation(&s->sync, "exposure %u half rows beyond the "
                              "frame (2 x VTS %u - 4)", raw.lines, e.vts);
    }
    if (e.vts && e.height && e.vts <= e.height) {
        sensor_sync_violation(&s->sync, "frame of %u rows shorter than its "
                              "window of %u", e.vts, e.height);
    }
    return true;
}

/* "lines" / "vts": the exposure and frame length the frame in progress
 * uses, for a twin's tooling (qom-get) */
static void smartsens_get_applied(Object *obj, Visitor *v, const char *name,
                                  void *opaque, Error **errp)
{
    ImageSensorExposure e = { 0 };
    uint32_t value;

    IMAGE_SENSOR_GET_CLASS(obj)->exposure(obj, &e);
    value = !strcmp(name, "lines") ? e.lines : e.vts;
    visit_type_uint32(v, name, &value, errp);
}

static void hisi_smartsens_init(Object *obj)
{
    HisiSmartSensState *s = HISI_SMARTSENS(obj);

    sensor_sync_add_properties(obj, &s->sync);
    object_property_add(obj, "lines", "uint32", smartsens_get_applied, NULL,
                        NULL, NULL);
    object_property_add(obj, "vts", "uint32", smartsens_get_applied, NULL,
                        NULL, NULL);
}

static void hisi_smartsens_realize(DeviceState *dev, Error **errp)
{
    HisiSmartSensState *s = HISI_SMARTSENS(dev);

    snprintf(s->name, sizeof(s->name), "smartsens-%02x%02x", s->id_high,
             s->id_low);
    s->sync.name = s->name;
}

static void hisi_smartsens_reset(DeviceState *dev)
{
    HisiSmartSensState *s = HISI_SMARTSENS(dev);

    s->reg_ptr = 0;
    s->cur_reg = 0;
    memset(s->reg_buf, 0, sizeof(s->reg_buf));
    s->streaming = false;
    sensor_sync_reset(&s->sync);
    hisi_smartsens_reset_regs(s);
}

/* a stream from before applied[] (version 1): the frame uses what was
 * written */
static int hisi_smartsens_post_load(void *opaque, int version_id)
{
    HisiSmartSensState *s = opaque;

    if (version_id < 2) {
        memcpy(s->applied, s->regs, sizeof(s->applied));
    }
    return 0;
}

static const Property hisi_smartsens_props[] = {
    DEFINE_PROP_UINT8("id_high", HisiSmartSensState, id_high, 0x00),
    DEFINE_PROP_UINT8("id_low",  HisiSmartSensState, id_low,  0x00),
    DEFINE_PROP_UINT8("gain-fine", HisiSmartSensState, gain_fine, 0),
    DEFINE_PROP_UINT8("disc",    HisiSmartSensState, disc,    0x00),
    /* the pixel clock, when the part's is known (the catalogue's): 0 for
     * none, then the ISP keeps its own frame rate */
    DEFINE_PROP_UINT32("pclk",   HisiSmartSensState, pclk,    0),
};

/* What the guest programmed: the register file and the address pointer. */
static const VMStateDescription vmstate_hisi_smartsens = {
    .name = TYPE_HISI_SMARTSENS,
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = hisi_smartsens_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, HisiSmartSensState),
        VMSTATE_UINT8_ARRAY(reg_buf, HisiSmartSensState, 2),
        VMSTATE_UINT8(reg_ptr, HisiSmartSensState),
        VMSTATE_UINT16(cur_reg, HisiSmartSensState),
        VMSTATE_UINT8_ARRAY(regs, HisiSmartSensState, SS_REG_COUNT),
        VMSTATE_UINT8_ARRAY_V(applied, HisiSmartSensState, SS_REG_COUNT, 2),
        VMSTATE_BOOL_V(streaming, HisiSmartSensState, 2),
        VMSTATE_BOOL_V(sync.latching, HisiSmartSensState, 2),
        VMSTATE_BOOL_V(sync.hold, HisiSmartSensState, 2),
        VMSTATE_UINT32_V(sync.frame, HisiSmartSensState, 2),
        VMSTATE_UINT32_V(sync.violations, HisiSmartSensState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void hisi_smartsens_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_smartsens_reset);
    dc->realize = hisi_smartsens_realize;
    dc->vmsd = &vmstate_hisi_smartsens;
    device_class_set_props(dc, hisi_smartsens_props);
    k->event = hisi_smartsens_event;
    k->recv  = hisi_smartsens_recv;
    k->send  = hisi_smartsens_send;
    IMAGE_SENSOR_CLASS(klass)->exposure = smartsens_sensor_exposure;
    IMAGE_SENSOR_CLASS(klass)->frame_start = smartsens_frame_start;
    IMAGE_SENSOR_CLASS(klass)->frame_ns = smartsens_frame_ns;
}

static const TypeInfo hisi_smartsens_info = {
    .name          = TYPE_HISI_SMARTSENS,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HisiSmartSensState),
    .instance_init = hisi_smartsens_init,
    .class_init    = hisi_smartsens_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { TYPE_IMAGE_SENSOR },
        { }
    },
};

static void hisi_smartsens_register_types(void)
{
    type_register_static(&hisi_smartsens_info);
}

type_init(hisi_smartsens_register_types)
