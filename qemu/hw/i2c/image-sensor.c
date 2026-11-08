/*
 * The image sensor catalogue: every sensor= name any machine takes, with
 * the model that answers for it, its usual 7-bit I2C address and the chip
 * ID its driver probes for.  A sensor is the same part on any SoC, so a
 * machine names one here instead of keeping a table of its own.
 *
 * The IDs come from the vendor sensor drivers (Sensor_id_table in the
 * MStar libdrv_ms_cus_*.so; the Ingenic and FullHan SDKs; Sofia).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/i2c/image-sensor.h"
#include "hw/qdev-properties.h"

#define GALAXYCORE      "galaxycore-sensor"
#define SMARTSENS       "hisi-smartsens"
#define OV2735          "ingenic-ov2735"

typedef struct ImageSensorSpec {
    const char *name;
    const char *type;
    uint8_t addr;
    uint16_t id;        /* GALAXYCORE, SMARTSENS; OV2735: its revision */
    uint16_t id_reg;    /* GALAXYCORE: the ID's high byte */
    bool reg16;         /* GALAXYCORE: 16-bit register addresses */
    uint8_t disc;       /* SMARTSENS: 0x3109 where parts share an ID */
    uint8_t gain_fine;  /* SMARTSENS: 0x3E09 at 1x, on the SC2310's layout */
    uint32_t pclk;      /* SMARTSENS: pixel clock, when known: frame timing */
} ImageSensorSpec;

/* GalaxyCore's register pairs; OmniVision's OV27x5/OV97x5 have the same */
#define GC8(n, a, i)        { n, GALAXYCORE, a, i, 0xF0, false }
#define GC16(n, a, i)       { n, GALAXYCORE, a, i, 0x03F0, true }
#define OV16(n, a, i)       { n, GALAXYCORE, a, i, 0x300A, true }
#define SC(n, i)            { n, SMARTSENS, 0x30, i }
#define SC_DISC(n, i, d)    { n, SMARTSENS, 0x30, i, .disc = d }

static const ImageSensorSpec image_sensors[] = {
    /* Sony */
    { "imx291", "hisi-imx291", 0x1A },
    { "imx307", "hisi-imx307", 0x1A },
    { "imx323", "imx323", 0x1A },
    { "imx335", "hisi-imx335", 0x1A },
    { "imx385", "hisi-imx385", 0x1A },
    { "imx415", "hisi-imx415", 0x1A },

    /* SOI / JX */
    { "f37", "hisi-f37", 0x40 },
    { "jxf37", "hisi-f37", 0x40 },
    { "jxf22", "hisi-jxf22", 0x40 },
    { "jxh42", "hisi-jxh42", 0x30 },
    { "jxh62", "hisi-jxh62", 0x30 },

    /* GalaxyCore */
    GC8("gc1054", 0x21, 0x1054),
    GC8("gc2053", 0x37, 0x2053),
    GC8("gc2053s1", 0x3F, 0x2053),      /* T23 SDK's second sensor */
    GC8("gc2063", 0x37, 0x2063),
    GC16("gc2083", 0x37, 0x2083),
    GC16("gc2093", 0x37, 0x2093),
    GC16("gc4653", 0x29, 0x4653),
    GC16("gc4663", 0x29, 0x4653),

    /* SmartSens */
    SC("sc2235", 0x2235),
    SC_DISC("sc2235e", 0x2232, 0x20),
    SC_DISC("sc2235p", 0x2232, 0x01),
    SC("sc2239", 0xCB10),
    /* 74.25 MHz: the SC2315E's init table runs HTS 2200 x VTS 1125 at
     * 1080p30 (sc2315e_sensor_ctl.c), and its driver's VTS is 33750 / fps */
    { "sc2310", SMARTSENS, 0x30, 0x2311, .gain_fine = 0x20,
      .pclk = 74250000 },
    { "sc2315", SMARTSENS, 0x30, 0x2311, .gain_fine = 0x20,
      .pclk = 74250000 },
    { "sc2315e", SMARTSENS, 0x30, 0x2238, .disc = 0x20, .gain_fine = 0x10,
      .pclk = 74250000 },
    SC("sc2335", 0xCB14),
    SC("sc2336", 0xCB3A),
    SC("sc307h", 0xCB1C),
    SC("sc401ai", 0xCD2E),
    SC("sc431hai", 0xCD6B),
    SC("sc501ai", 0xCE1F),
    SC("sc5035", 0x5035),
    SC("sc5238", 0x5235),

    /* OmniVision */
    { "os04d10", "os04d10", 0x3C },
    OV16("ov2715", 0x36, 0x2715),
    OV16("ov4689", 0x36, 0x4688),
    OV16("ov9715", 0x30, 0x9711),
    { "ov2735", OV2735, 0x3C, 0x05 },
    { "ov2735b", OV2735, 0x3C, 0x07 },

    /* others */
    { "mis2006", "hisi-mis2006", 0x30 },
    { "sp2305", "hisi-sp2305", 0x3C },
    { "nvp6124b", "hisi-nvp6124b", 0x60 },  /* a decoder in the sensor's place */
};

static const ImageSensorSpec *image_sensor_find(const char *name)
{
    int n;

    for (n = 0; n < ARRAY_SIZE(image_sensors); n++) {
        if (!strcmp(name, image_sensors[n].name)) {
            return &image_sensors[n];
        }
    }
    return NULL;
}

bool image_sensor_known(const char *name)
{
    return name && image_sensor_find(name);
}

DeviceState *image_sensor_create_at(I2CBus *bus, const char *name,
                                    uint8_t addr)
{
    const ImageSensorSpec *spec;
    DeviceState *dev;
    int n;

    if (!name || !strcmp(name, "none")) {
        return NULL;
    }
    spec = image_sensor_find(name);
    if (!spec) {
        error_report("unknown sensor '%s'; one of: none", name);
        for (n = 0; n < ARRAY_SIZE(image_sensors); n++) {
            error_printf("  %s\n", image_sensors[n].name);
        }
        exit(1);
    }
    dev = DEVICE(i2c_slave_new(spec->type, addr ? addr : spec->addr));
    if (!strcmp(spec->type, GALAXYCORE)) {
        qdev_prop_set_bit(dev, "reg16", spec->reg16);
        qdev_prop_set_uint16(dev, "id-reg", spec->id_reg);
        qdev_prop_set_uint16(dev, "id", spec->id);
    } else if (!strcmp(spec->type, SMARTSENS)) {
        qdev_prop_set_uint8(dev, "id_high", spec->id >> 8);
        qdev_prop_set_uint8(dev, "id_low", spec->id & 0xFF);
        qdev_prop_set_uint8(dev, "disc", spec->disc);
        qdev_prop_set_uint8(dev, "gain-fine", spec->gain_fine);
        qdev_prop_set_uint32(dev, "pclk", spec->pclk);
    } else if (!strcmp(spec->type, OV2735)) {
        qdev_prop_set_uint8(dev, "rev", spec->id);
    }
    i2c_slave_realize_and_unref(I2C_SLAVE(dev), bus, &error_fatal);
    return dev;
}

DeviceState *image_sensor_create(I2CBus *bus, const char *name)
{
    return image_sensor_create_at(bus, name, 0);
}

bool image_sensor_exposure(Object *sensor, ImageSensorExposure *e)
{
    if (!sensor || !object_dynamic_cast(sensor, TYPE_IMAGE_SENSOR)) {
        return false;
    }
    memset(e, 0, sizeof(*e));
    IMAGE_SENSOR_GET_CLASS(sensor)->exposure(sensor, e);
    return true;
}

bool image_sensor_frame_start(Object *sensor)
{
    ImageSensorClass *k;

    if (!sensor || !object_dynamic_cast(sensor, TYPE_IMAGE_SENSOR)) {
        return false;
    }
    k = IMAGE_SENSOR_GET_CLASS(sensor);
    return k->frame_start && k->frame_start(sensor);
}

int64_t image_sensor_frame_ns(Object *sensor)
{
    ImageSensorClass *k;

    if (!sensor || !object_dynamic_cast(sensor, TYPE_IMAGE_SENSOR)) {
        return 0;
    }
    k = IMAGE_SENSOR_GET_CLASS(sensor);
    return k->frame_ns ? k->frame_ns(sensor) : 0;
}

static const TypeInfo image_sensor_info = {
    .name       = TYPE_IMAGE_SENSOR,
    .parent     = TYPE_INTERFACE,
    .class_size = sizeof(ImageSensorClass),
};

static void image_sensor_register_types(void)
{
    type_register_static(&image_sensor_info);
}

type_init(image_sensor_register_types)
