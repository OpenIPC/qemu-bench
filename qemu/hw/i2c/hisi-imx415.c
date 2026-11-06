/*
 * Sony IMX415 image sensor I2C stub.
 *
 * Sony IMX415 is the standard 4K Starvis sensor on Hi3516AV300-class
 * boards.  ipctool / vendor SDK detect IMX415 by:
 *   1. Setting I2C addr 0x34 (8-bit) = 0x1A (7-bit slave addr)
 *   2. Reading reg 0x3B00: must be 0x2E (programmed) or 0x28 (POR default)
 *
 * Returning 0x28 at 0x3B00 satisfies the IMX415 branch.  All other
 * registers read back as 0xFF (uninitialized) so earlier branches in
 * the probe (IMX347, IMX334, IMX335, IMX322, IMX323) all fall through.
 *
 * The exposure registers (datasheet, and the vendor drivers' sensor_imx415
 * tables): VMAX 0x3024-0x3026, SHR0 0x3050-0x3052, GAIN_PCG 0x3090-0x3091 in
 * 0.3 dB steps.  What integrates is VMAX - SHR0 rows.  The registers read 0xFF
 * until the driver programs them, so nothing is exposed before then.
 *
 * Reference: ipctool/src/sensors.c detect_sony_sensor() lines 202-209
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/image-sensor.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include <math.h>

#define TYPE_HISI_IMX415 "hisi-imx415"
OBJECT_DECLARE_SIMPLE_TYPE(HisiIMX415State, HISI_IMX415)

#define IMX415_REG_BASE  0x3000
#define IMX415_REG_COUNT 0x1000

struct HisiIMX415State {
    I2CSlave parent_obj;
    uint8_t  reg_buf[2];
    uint8_t  reg_ptr;
    uint16_t cur_reg;
    uint8_t  regs[IMX415_REG_COUNT];
};

static void hisi_imx415_reset_regs(HisiIMX415State *s)
{
    memset(s->regs, 0xFF, sizeof(s->regs));
    /* IMX415 datasheet p.46: 0x3B00 default after reset is 0x28 */
    s->regs[0x3B00 - IMX415_REG_BASE] = 0x28;
}

static int hisi_imx415_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiIMX415State *s = HISI_IMX415(i2c);

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

static int hisi_imx415_send(I2CSlave *i2c, uint8_t data)
{
    HisiIMX415State *s = HISI_IMX415(i2c);

    if (s->reg_ptr < 2) {
        s->reg_buf[s->reg_ptr++] = data;
    } else {
        uint16_t addr = ((uint16_t)s->reg_buf[0] << 8) | s->reg_buf[1];
        if (addr >= IMX415_REG_BASE &&
            addr < IMX415_REG_BASE + IMX415_REG_COUNT) {
            s->regs[addr - IMX415_REG_BASE] = data;
        }
        s->reg_buf[1]++;
        if (s->reg_buf[1] == 0) {
            s->reg_buf[0]++;
        }
    }
    return 0;
}

static uint8_t hisi_imx415_recv(I2CSlave *i2c)
{
    HisiIMX415State *s = HISI_IMX415(i2c);
    uint8_t val = 0xFF;

    if (s->cur_reg >= IMX415_REG_BASE &&
        s->cur_reg < IMX415_REG_BASE + IMX415_REG_COUNT) {
        val = s->regs[s->cur_reg - IMX415_REG_BASE];
    }
    s->cur_reg++;
    return val;
}

static uint32_t imx415_reg24(HisiIMX415State *s, uint16_t reg)
{
    const uint8_t *r = &s->regs[reg - IMX415_REG_BASE];

    return r[0] | r[1] << 8 | (r[2] & 0xF) << 16;
}

static void hisi_imx415_exposure(Object *sensor, ImageSensorExposure *e)
{
    HisiIMX415State *s = HISI_IMX415(sensor);
    uint32_t vmax = imx415_reg24(s, 0x3024);
    uint32_t shr = imx415_reg24(s, 0x3050);
    uint32_t g = s->regs[0x3090 - IMX415_REG_BASE] |
                 (s->regs[0x3091 - IMX415_REG_BASE] & 1) << 8;

    e->lines = vmax > shr ? vmax - shr : 1;
    e->vts = vmax;
    e->gain = lrint(1024 * pow(10, g * 0.3 / 20));
    e->width = 3840;
    e->height = 2160;
}

static void hisi_imx415_reset(DeviceState *dev)
{
    HisiIMX415State *s = HISI_IMX415(dev);

    s->reg_ptr = 0;
    s->cur_reg = 0;
    memset(s->reg_buf, 0, sizeof(s->reg_buf));
    hisi_imx415_reset_regs(s);
}

static void hisi_imx415_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_imx415_reset);
    k->event = hisi_imx415_event;
    k->recv  = hisi_imx415_recv;
    k->send  = hisi_imx415_send;
    IMAGE_SENSOR_CLASS(klass)->exposure = hisi_imx415_exposure;
}

static const TypeInfo hisi_imx415_info = {
    .name          = TYPE_HISI_IMX415,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HisiIMX415State),
    .class_init    = hisi_imx415_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { TYPE_IMAGE_SENSOR },
        { }
    },
};

static void hisi_imx415_register_types(void)
{
    type_register_static(&hisi_imx415_info);
}

type_init(hisi_imx415_register_types)
