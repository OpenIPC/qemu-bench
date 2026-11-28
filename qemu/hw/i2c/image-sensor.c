/*
 * The image sensor catalogue: every sensor= name any machine takes, with
 * the model that answers for it, its usual 7-bit I2C address and the chip
 * ID its driver probes for.  A sensor is the same part on any SoC, so a
 * machine names one here instead of keeping a table of its own.
 *
 * The IDs come from the vendor sensor drivers (Sensor_id_table in the
 * MStar libdrv_ms_cus_*.so; the Ingenic, FullHan and Axera SDKs; Sofia).
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
#define OMNIVISION      "axera-ov-sensor"
#define IMAGEDESIGN     "imagedesign-mis"
#define OV2710          "ov2710-sensor"

typedef struct ImageSensorSpec {
    const char *name;
    const char *type;
    uint8_t addr;
    uint32_t id;        /* GALAXYCORE, SMARTSENS, OMNIVISION (3 bytes);
                         * OV2735: its revision */
    uint16_t id_reg;    /* GALAXYCORE, OMNIVISION, OV2710: the ID's high
                         * byte */
    bool reg16;         /* GALAXYCORE: 16-bit register addresses */
    uint8_t disc;       /* SMARTSENS: 0x3109 where parts share an ID */
    uint8_t gain_fine;  /* SMARTSENS: 0x3E09 at 1x, on the SC2310's layout */
    uint32_t pclk;      /* SMARTSENS, GALAXYCORE, OV2710: pixel clock,
                         * when known:
                         * frame timing */
    uint16_t hts;       /* SMARTSENS: HTS at reset, where a mode leaves it */
    bool exp_rows;      /* SMARTSENS: exposure in rows, not half rows */
    uint16_t gain_hcg;  /* SMARTSENS: the SC465SL's gain layout, its HCG
                         * factor in 1/1024 */
    uint16_t alt_id_reg; /* OMNIVISION: a second, four-byte ID where the */
    uint32_t alt_id;     /* part has one */
} ImageSensorSpec;

/* GalaxyCore's register pairs; OmniVision's OV4689/OV97x5 have the same */
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
    { "jxf23", "hisi-jxf23", 0x40 },
    { "jxq03", "hisi-jxq03", 0x40 },
    { "jxh42", "hisi-jxh42", 0x30 },
    { "jxh62", "hisi-jxh62", 0x30 },

    /* GalaxyCore */
    GC8("gc1054", 0x21, 0x1054),
    { "gc2053", GALAXYCORE, 0x37, 0x2053, 0xF0, false,
      .pclk = 74250000, .hts = 2200 },
    { "gc2053s1", GALAXYCORE, 0x3F, 0x2053, 0xF0, false,
      .pclk = 74250000, .hts = 2200 },  /* T23 SDK's second sensor */
    GC8("gc2063", 0x37, 0x2063),
    /* 100 MHz: the 24 MHz reference setting's pixel clock (galaxycore.c) */
    { "gc2083", GALAXYCORE, 0x37, 0x2083, 0x03F0, true,
      .pclk = 100000000 },
    GC16("gc2093", 0x37, 0x2093),
    GC16("gc4653", 0x29, 0x4653),
    GC16("gc4663", 0x29, 0x4653),

    /* SmartSens */
    SC("sc1345", 0xDA23),       /* OpenIPC's Anyka EV200 driver */
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
    { "sc2336", SMARTSENS, 0x30, 0xCB3A, .pclk = 79200000, .hts = 2200,
      .exp_rows = true },
    { "sc2336p", SMARTSENS, 0x30, 0x9B3A, .pclk = 79200000, .hts = 2200,
      .exp_rows = true },
    { "sc2336ps1", SMARTSENS, 0x32, 0x9B3A, .pclk = 79200000, .hts = 2200,
      .exp_rows = true },
    { "sc2337p", SMARTSENS, 0x30, 0x9B3A, .pclk = 79200000, .hts = 2200,
      .exp_rows = true },
    { "sc2337ps1", SMARTSENS, 0x32, 0x9B3A, .pclk = 79200000, .hts = 2200,
      .exp_rows = true },
    /* the SC1346 reads the SC1A4T's ID: AliOS's camera_sc1346 takes it */
    { "sc1346", SMARTSENS, 0x30, 0xDA4D, .pclk = 72000000, .hts = 1600,
      .exp_rows = true },
    { "sc1a4t", SMARTSENS, 0x30, 0xDA4D, .pclk = 72000000, .hts = 1600,
      .exp_rows = true },
    { "sc1a4ts1", SMARTSENS, 0x32, 0xDA4D, .pclk = 72000000, .hts = 1600,
      .exp_rows = true },
    /* RV1106 camera boards' (Rockchip kernel drivers/media/i2c: CHIP_ID
     * at 0x3107:0x3108) */
    SC("sc231hai", 0xCB6A),
    SC("sc3336", 0xCC41),
    SC("sc530ai", 0x8E39),
    SC("sc307h", 0xCB1C),
    SC("sc401ai", 0xCD2E),
    SC("sc431hai", 0xCD6B),
    /* the AX620E camera boards' (sc450ai_reg.h: 0x3107:0x3108 = 0xbd2f) */
    SC("sc450ai", 0xBD2F),
    /*
     * hi3516cv6xx: load_hisilicon's default.  Its 4M mode (sc4336p_cfg.h,
     * "27M input, 2 lanes, 10 bit, 630 Mbps, 2560x1440 30 fps") writes VTS
     * 1500 but leaves HTS at reset: 630 Mbps x 2 / 10 = 126 Mpixel/s, so
     * HTS is 2800 (0x0af0, which the 2M mode writes) for 30 fps.
     */
    { "sc4336p", SMARTSENS, 0x30, 0x9C42, .pclk = 126000000, .hts = 2800,
      .exp_rows = true },
    /*
     * GK7206 SDK smartsens_sc465sl: 27 MHz in, 2560x1440 12-bit; its 30 fps
     * modes write HTS 0x546 x VTS 0xA6A, so 1350 x 2666 x 30 = 108 Mpixel/s.
     * The ID (0x3107:0x3108 = 0xBD82) is what XM's App compares against.
     * Its exposure counts rows (sc465sl.c: max VTS - 8), its gains are
     * again_table / dgain_table's layout, HCG x3.41 (3492 / 1024).
     */
    { "sc465sl", SMARTSENS, 0x30, 0xBD82, .pclk = 108000000, .hts = 1350,
      .exp_rows = true, .gain_hcg = 3492 },
    SC("sc501ai", 0xCE1F),
    SC("sc5035", 0x5035),
    SC("sc5238", 0x5235),

    /* OmniVision */
    { "os04d10", "os04d10", 0x3C },
    /* exposure, gain and timing from the Grain Media fisp_ov2715.ko
     * (ov2710.c): 90 MHz from a 27 MHz XCLK */
    { "ov2710", OV2710, 0x36, 0x2710, 0x300A, .pclk = 90000000 },
    { "ov2715", OV2710, 0x36, 0x2715, 0x300A, .pclk = 90000000 },
    OV16("ov4689", 0x36, 0x4688),
    OV16("ov9715", 0x30, 0x9711),
    /* PID / VER at 0x0A / 0x0B (fcap200_ov7720.ko of the GM8126) */
    { "ov7725", GALAXYCORE, 0x21, 0x7721, 0x0A, false },
    /* the AX620U boards' (Axera SDK ov_os04a10/os04a10.c: 0x36, chip ID
     * 0x300A:0x300B:0x300C = 0x530441) */
    { "os04a10", OMNIVISION, 0x36, 0x530441, 0x300A },
    /* the Reolink E340's, as its sensor reads: 0x530541 at 0x300A, and
     * 0x1c050a01 at 0x6000, the ID Novatek's nvt_sen_os05a10.ko probes */
    { "os05a10", OMNIVISION, 0x36, 0x530541, 0x300A,
      .alt_id_reg = 0x6000, .alt_id = 0x1c050a01 },
    { "ov2735", OV2735, 0x3C, 0x05 },
    { "ov2735b", OV2735, 0x3C, 0x07 },

    /* ImageDesign: MIS2008 and MIS2009 are one die and both read 0x2008
     * at 0x3000 (ipctool's "mis2008_i2c" on a GK7205V510 MIS2009 camera) */
    { "mis2008", IMAGEDESIGN, 0x30, 0x2008 },
    { "mis2009", IMAGEDESIGN, 0x30, 0x2008 },

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
        qdev_prop_set_uint32(dev, "pclk", spec->pclk);
        qdev_prop_set_uint16(dev, "hts", spec->hts);
    } else if (!strcmp(spec->type, SMARTSENS)) {
        qdev_prop_set_uint8(dev, "id_high", spec->id >> 8);
        qdev_prop_set_uint8(dev, "id_low", spec->id & 0xFF);
        qdev_prop_set_uint8(dev, "disc", spec->disc);
        qdev_prop_set_uint8(dev, "gain-fine", spec->gain_fine);
        qdev_prop_set_uint32(dev, "pclk", spec->pclk);
        qdev_prop_set_uint16(dev, "hts", spec->hts);
        qdev_prop_set_bit(dev, "exp-rows", spec->exp_rows);
        qdev_prop_set_uint16(dev, "gain-hcg", spec->gain_hcg);
    } else if (!strcmp(spec->type, OMNIVISION)) {
        qdev_prop_set_uint16(dev, "id_reg_base", spec->id_reg);
        qdev_prop_set_uint8(dev, "id0", spec->id >> 16);
        qdev_prop_set_uint8(dev, "id1", (spec->id >> 8) & 0xFF);
        qdev_prop_set_uint8(dev, "id2", spec->id & 0xFF);
        qdev_prop_set_uint16(dev, "id_alt_reg", spec->alt_id_reg);
        qdev_prop_set_uint32(dev, "id_alt", spec->alt_id);
    } else if (!strcmp(spec->type, OV2710)) {
        qdev_prop_set_uint16(dev, "id-reg", spec->id_reg);
        qdev_prop_set_uint16(dev, "id", spec->id);
        qdev_prop_set_uint32(dev, "pclk", spec->pclk);
    } else if (!strcmp(spec->type, IMAGEDESIGN)) {
        qdev_prop_set_uint16(dev, "id", spec->id);
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

bool image_sensor_streaming(Object *sensor)
{
    ImageSensorClass *k;

    if (!object_dynamic_cast(sensor, TYPE_IMAGE_SENSOR)) {
        return true;
    }
    k = IMAGE_SENSOR_GET_CLASS(sensor);
    return !k->streaming || k->streaming(sensor);
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
