/*
 * OmniVision OS04D10, 2560x1440 RAW10 over two MIPI lanes: the sensor of
 * the SigmaStar Iford demo boards.
 *
 * From the vendor's drv_ss_os04d10_mipi.c.  One-byte registers in pages,
 * 0xFD selecting the page:
 *
 *   page 0  0x02-0x05 the chip ID, 53 04 44 10; 0x20 and 0x36 the stream
 *   page 1  0x03/0x04 exposure in rows; 0x0C/0x0D frame length in rows;
 *           0x24 analog gain, 16 = 1x, to 0xF8; 0x45 2 = high conversion
 *           gain, 0 = low
 *   page 5  0x37 [2:0]/0x39 digital gain, 64 = 1x
 *
 * GetAEGain multiplies the high conversion gain in as otp_data / 1024:
 * its default, 3120, when the OTP says nothing.  This OTP says nothing.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/image-sensor.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define TYPE_OS04D10 "os04d10"
OBJECT_DECLARE_SIMPLE_TYPE(OS04D10State, OS04D10)

#define OS04D10_PAGES       8
#define OS04D10_PAGE        0xFD
#define OS04D10_HCG_GAIN    3120        /* otp_data's default */

struct OS04D10State {
    I2CSlave parent_obj;

    bool have_reg;
    uint8_t reg;
    uint8_t page;
    uint8_t regs[OS04D10_PAGES][256];
};

static uint8_t *os04d10_slot(OS04D10State *s)
{
    return &s->regs[s->page % OS04D10_PAGES][s->reg];
}

static int os04d10_event(I2CSlave *i2c, enum i2c_event event)
{
    OS04D10State *s = OS04D10(i2c);

    if (event == I2C_START_SEND) {
        s->have_reg = false;
    }
    return 0;
}

static int os04d10_send(I2CSlave *i2c, uint8_t data)
{
    OS04D10State *s = OS04D10(i2c);

    if (!s->have_reg) {
        s->reg = data;
        s->have_reg = true;
    } else if (s->reg == OS04D10_PAGE) {
        s->page = data;
    } else {
        *os04d10_slot(s) = data;
        s->reg++;
    }
    return 0;
}

static uint8_t os04d10_recv(I2CSlave *i2c)
{
    OS04D10State *s = OS04D10(i2c);
    uint8_t v = s->reg == OS04D10_PAGE ? s->page : *os04d10_slot(s);

    s->reg++;
    return v;
}

static void os04d10_exposure(Object *sensor, ImageSensorExposure *e)
{
    OS04D10State *s = OS04D10(sensor);
    uint32_t again = s->regs[1][0x24];
    uint32_t dgain = (s->regs[5][0x37] & 7) << 8 | s->regs[5][0x39];
    uint32_t cg = s->regs[1][0x45] == 2 ? OS04D10_HCG_GAIN : 1024;

    e->lines = s->regs[1][0x03] << 8 | s->regs[1][0x04];
    e->vts = s->regs[1][0x0C] << 8 | s->regs[1][0x0D];
    e->gain = (uint64_t)again * dgain * cg / (16 * 64);
    e->width = 2560;
    e->height = 1440;
}

static void os04d10_reset(DeviceState *dev)
{
    OS04D10State *s = OS04D10(dev);

    s->have_reg = false;
    s->reg = 0;
    s->page = 0;
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0][0x02] = 0x53;
    s->regs[0][0x03] = 0x04;
    s->regs[0][0x04] = 0x44;
    s->regs[0][0x05] = 0x10;
    s->regs[1][0x24] = 0x10;
    s->regs[5][0x39] = 0x40;
}

static const VMStateDescription vmstate_os04d10 = {
    .name = TYPE_OS04D10,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, OS04D10State),
        VMSTATE_BOOL(have_reg, OS04D10State),
        VMSTATE_UINT8(reg, OS04D10State),
        VMSTATE_UINT8(page, OS04D10State),
        VMSTATE_UINT8_2DARRAY(regs, OS04D10State, OS04D10_PAGES, 256),
        VMSTATE_END_OF_LIST()
    }
};

static void os04d10_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, os04d10_reset);
    dc->vmsd = &vmstate_os04d10;
    k->event = os04d10_event;
    k->send = os04d10_send;
    k->recv = os04d10_recv;
    IMAGE_SENSOR_CLASS(klass)->exposure = os04d10_exposure;
}

static const TypeInfo os04d10_info = {
    .name          = TYPE_OS04D10,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(OS04D10State),
    .class_init    = os04d10_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { TYPE_IMAGE_SENSOR },
        { }
    },
};

static void os04d10_register_types(void)
{
    type_register_static(&os04d10_info);
}

type_init(os04d10_register_types)
