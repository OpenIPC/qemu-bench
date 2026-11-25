/*
 * Sony IMX335 image sensor I2C stub.
 *
 * Emulates the I2C register interface of the IMX335 sensor so that
 * ipctool (and similar tools) can detect it.  Only the identification
 * registers are populated; everything else reads as zero.
 *
 * The exposure the image pipeline renders (image_sensor_exposure()) is
 * read from the registers the vendor drivers' AE writes (SigmaStar's
 * sensor_imx335_mipi.c expo_reg, vts_reg and gain_reg): VMAX 0x3030-0x3032
 * and SHR0 0x3058-0x305A, 20 bits each, low byte first, the exposure being
 * VMAX - SHR0 rows; GAIN 0x30E8-0x30E9, 11 bits in 0.3 dB steps.  Out of
 * reset they are zero, which is one row at 0 dB.  The window is the one
 * the mode registers set: WINMODE 0x3018 = 4 (window cropping) gives
 * HNUM 0x302E-0x302F by Y_OUT_SIZE 0x3056-0x3057, 13 bits each, low byte
 * first; any other WINMODE the full 2592x1944.  OpenIPC's ssc377d driver
 * sets 2592x1920@60, 2584x1248@90 and 1944x1080@120 this way.  It is given
 * only when the machine sets "window" (infinity6c, whose ISP takes the
 * sensor's window): elsewhere hisi-viu and hisi-vi keep the size their own
 * registers set.
 *
 * Register space: 16-bit addresses (0x3000-0x3FFF), 8-bit data.
 * Protocol: master sends 2-byte register address, then reads/writes data.
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

#define TYPE_HISI_IMX335 "hisi-imx335"
OBJECT_DECLARE_SIMPLE_TYPE(HisiIMX335State, HISI_IMX335)

#define IMX335_REG_BASE  0x3000
#define IMX335_REG_COUNT 0x1000

struct HisiIMX335State {
    I2CSlave parent_obj;
    uint8_t  reg_buf[2];    /* register address accumulator (big-endian) */
    uint8_t  reg_ptr;       /* bytes received for register address */
    uint16_t cur_reg;       /* current register address */
    uint8_t  regs[IMX335_REG_COUNT];
    bool     window;        /* report the window the mode registers set */
};

static void hisi_imx335_reset_regs(HisiIMX335State *s)
{
    memset(s->regs, 0, sizeof(s->regs));

    /* IMX335 identification registers */
    s->regs[0x3057 - IMX335_REG_BASE] = 0x07;  /* chip_id (not 0x06=IMX347) */
    s->regs[0x316A - IMX335_REG_BASE] = 0x7C;  /* IMX335 detect: (v&0xFC)==0x7C */
    s->regs[0x3078 - IMX335_REG_BASE] = 0x01;  /* hint check */
    s->regs[0x341C - IMX335_REG_BASE] = 0x00;  /* not 0x47 (IMX334) */
    /* BLKLEVEL, power-on 0x032.  Xiongmai's libsns_X50 sensor_get_chip()
     * tells an IMX335 at 0x1A by reading 0x3302 == 0x32. */
    s->regs[0x3302 - IMX335_REG_BASE] = 0x32;
    s->regs[0x3303 - IMX335_REG_BASE] = 0x00;
}

static int hisi_imx335_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiIMX335State *s = HISI_IMX335(i2c);

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

static int hisi_imx335_send(I2CSlave *i2c, uint8_t data)
{
    HisiIMX335State *s = HISI_IMX335(i2c);

    if (s->reg_ptr < 2) {
        /* Accumulate 16-bit register address */
        s->reg_buf[s->reg_ptr++] = data;
    } else {
        /* Data write */
        uint16_t addr = ((uint16_t)s->reg_buf[0] << 8) | s->reg_buf[1];
        if (addr >= IMX335_REG_BASE &&
            addr < IMX335_REG_BASE + IMX335_REG_COUNT) {
            s->regs[addr - IMX335_REG_BASE] = data;
        }
        /* Auto-increment address for multi-byte writes */
        s->reg_buf[1]++;
        if (s->reg_buf[1] == 0) {
            s->reg_buf[0]++;
        }
    }
    return 0;
}

static uint8_t hisi_imx335_recv(I2CSlave *i2c)
{
    HisiIMX335State *s = HISI_IMX335(i2c);
    uint8_t val = 0xFF;

    if (s->cur_reg >= IMX335_REG_BASE &&
        s->cur_reg < IMX335_REG_BASE + IMX335_REG_COUNT) {
        val = s->regs[s->cur_reg - IMX335_REG_BASE];
    }
    s->cur_reg++;
    return val;
}

static uint32_t imx335_reg20(HisiIMX335State *s, uint16_t reg)
{
    const uint8_t *r = &s->regs[reg - IMX335_REG_BASE];

    return r[0] | r[1] << 8 | (r[2] & 0xF) << 16;
}

static uint32_t imx335_reg13(HisiIMX335State *s, uint16_t reg)
{
    const uint8_t *r = &s->regs[reg - IMX335_REG_BASE];

    return r[0] | (r[1] & 0x1F) << 8;
}

static void hisi_imx335_exposure(Object *sensor, ImageSensorExposure *e)
{
    HisiIMX335State *s = HISI_IMX335(sensor);
    uint32_t vmax = imx335_reg20(s, 0x3030);
    uint32_t shr = imx335_reg20(s, 0x3058);
    uint32_t g = s->regs[0x30E8 - IMX335_REG_BASE] |
                 (s->regs[0x30E9 - IMX335_REG_BASE] & 7) << 8;

    e->lines = vmax > shr ? vmax - shr : 1;
    e->vts = vmax;
    e->gain = lrint(1024 * pow(10, MIN(g, 240) * 0.3 / 20));   /* 72 dB */
    if (s->window) {
        uint32_t hnum = imx335_reg13(s, 0x302E);
        uint32_t vout = imx335_reg13(s, 0x3056);

        if ((s->regs[0x3018 - IMX335_REG_BASE] & 0xF) == 4 && hnum && vout) {
            e->width = MIN(hnum, 2592);
            e->height = MIN(vout, 1944);
        } else {
            e->width = 2592;
            e->height = 1944;
        }
    }
}

static bool hisi_imx335_get_window(Object *obj, Error **errp)
{
    return HISI_IMX335(obj)->window;
}

static void hisi_imx335_set_window(Object *obj, bool value, Error **errp)
{
    HISI_IMX335(obj)->window = value;
}

static void hisi_imx335_reset(DeviceState *dev)
{
    HisiIMX335State *s = HISI_IMX335(dev);

    s->reg_ptr = 0;
    s->cur_reg = 0;
    memset(s->reg_buf, 0, sizeof(s->reg_buf));
    hisi_imx335_reset_regs(s);
}

static void hisi_imx335_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_imx335_reset);
    k->event = hisi_imx335_event;
    k->recv = hisi_imx335_recv;
    k->send = hisi_imx335_send;
    IMAGE_SENSOR_CLASS(klass)->exposure = hisi_imx335_exposure;
    object_class_property_add_bool(klass, "window", hisi_imx335_get_window,
                                   hisi_imx335_set_window);
    object_class_property_set_description(klass, "window",
        "Report the window the mode registers set to the image pipeline");
}

static const TypeInfo hisi_imx335_info = {
    .name          = TYPE_HISI_IMX335,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HisiIMX335State),
    .class_init    = hisi_imx335_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { TYPE_IMAGE_SENSOR },
        { }
    },
};

static void hisi_imx335_register_types(void)
{
    type_register_static(&hisi_imx335_info);
}

type_init(hisi_imx335_register_types)
