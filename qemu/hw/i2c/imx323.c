/*
 * Sony IMX323: 1080p parallel sensor at 0x34 (8-bit) = 0x1A, with 16-bit
 * register addresses and 8-bit data.  Unlike the IMX307 family its timing
 * registers sit in the SMIA space below 0x1000 as well as at 0x3000: the
 * shutter at 0x0202 and the frame length at 0x0340, big-endian.  All from
 * libdrv_ms_cus_imx323.so (expo_reg, vts_reg, Sensor_init_table), and its
 * Sensor_id_table: 0x301C reads 0x50.  It reports no output window: 0x034C
 * says 1984 wide, while the MStar ISP crops 1920 at x = 106 out of the
 * parallel data, so that register does not bound what reaches the ISP.
 *
 * The driver's gain is 20 log10(g) / 0.3.  That it lands in 0x301E is not
 * from the driver: the register is in its init table, as the IMX family's.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/image-sensor.h"
#include "qemu/module.h"
#include <math.h>

#define TYPE_IMX323 "imx323"
OBJECT_DECLARE_SIMPLE_TYPE(IMX323State, IMX323)

#define IMX323_REGS     0x4000

struct IMX323State {
    I2CSlave parent_obj;

    uint8_t addr[2];
    uint8_t addr_len;
    uint16_t reg;
    uint8_t regs[IMX323_REGS];
};

static int imx323_event(I2CSlave *i2c, enum i2c_event event)
{
    IMX323State *s = IMX323(i2c);

    if (event == I2C_START_SEND) {
        s->addr_len = 0;
    }
    return 0;
}

static int imx323_send(I2CSlave *i2c, uint8_t data)
{
    IMX323State *s = IMX323(i2c);

    if (s->addr_len < 2) {
        s->addr[s->addr_len++] = data;
        s->reg = s->addr[0] << 8 | s->addr[1];
    } else {
        if (s->reg < IMX323_REGS) {
            s->regs[s->reg] = data;
        }
        s->reg++;
    }
    return 0;
}

static uint8_t imx323_recv(I2CSlave *i2c)
{
    IMX323State *s = IMX323(i2c);
    uint8_t val = s->reg < IMX323_REGS ? s->regs[s->reg] : 0xFF;

    s->reg++;
    return val;
}

static void imx323_exposure(Object *sensor, ImageSensorExposure *e)
{
    IMX323State *s = IMX323(sensor);

#define R16(a) (s->regs[a] << 8 | s->regs[(a) + 1])
    e->vts = R16(0x0340);
    /* SHS: the rows from the frame's end, 0 the longest exposure */
    e->lines = e->vts > R16(0x0202) ? e->vts - R16(0x0202) : 1;
#undef R16
    e->gain = 1024 * pow(10, s->regs[0x301E] * 0.3 / 20);
}

static void imx323_reset(DeviceState *dev)
{
    IMX323State *s = IMX323(dev);

    s->addr_len = 0;
    s->reg = 0;
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x301C] = 0x50;
}

static void imx323_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, imx323_reset);
    k->event = imx323_event;
    k->send = imx323_send;
    k->recv = imx323_recv;
    IMAGE_SENSOR_CLASS(klass)->exposure = imx323_exposure;
}

static const TypeInfo imx323_info = {
    .name          = TYPE_IMX323,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(IMX323State),
    .class_init    = imx323_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { TYPE_IMAGE_SENSOR },
        { }
    },
};

static void imx323_register_types(void)
{
    type_register_static(&imx323_info);
}

type_init(imx323_register_types)
