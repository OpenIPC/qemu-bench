/*
 * OmniVision OV2735 / OV2735B image sensor I2C stub, for the Ingenic T30
 * machine's sensor= (the only sensor drivers the OpenIPC T30 rootfs has).
 *
 * 8-bit register addresses and data, in pages selected by register 0xFD
 * (which every page has).  The ID is in page 0: 0x02 / 0x03 = 0x27 / 0x35
 * and 0x04 the revision, which tells the two apart: sinfo.ko's table
 * takes 0x05 as an OV2735 and 0x06 or 0x07 as an OV2735B (the "rev"
 * property, default 0x07).  Everything else is RAM, so the drivers' init
 * tables run; the ID registers keep their values whatever is written.
 * ov2735_exposure() decodes the exposure sensor_ov2735b_t30.ko writes:
 * page 1, rows in 0x03:0x04, analogue gain x16 in 0x24 (its again LUT:
 * 0x10 = 1x .. 0xF8 = 15.5x), each latched by 0x01 = 1.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/ingenic-sensor.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define TYPE_INGENIC_OV2735 "ingenic-ov2735"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicOv2735State, INGENIC_OV2735)

#define OV_PAGES        4
#define OV_PAGE_SEL     0xFD

struct IngenicOv2735State {
    I2CSlave parent_obj;

    uint8_t rev;

    uint8_t page;
    uint8_t ptr;
    bool have_ptr;
    uint8_t regs[OV_PAGES][256];
};

static uint8_t ov_read(IngenicOv2735State *s, uint8_t r)
{
    if (r == OV_PAGE_SEL) {
        return s->page;
    }
    if (s->page == 0) {
        switch (r) {
        case 0x02:
            return 0x27;
        case 0x03:
            return 0x35;
        case 0x04:
            return s->rev;
        }
    }
    return s->regs[s->page % OV_PAGES][r];
}

bool ov2735_exposure(Object *sensor, IngenicSensorExposure *e)
{
    IngenicOv2735State *s = (IngenicOv2735State *)
        object_dynamic_cast(sensor, TYPE_INGENIC_OV2735);

    if (!s) {
        return false;
    }
    e->lines = s->regs[1][0x03] << 8 | s->regs[1][0x04];
    e->gain = s->regs[1][0x24] * 64;
    e->gain = e->gain ? e->gain : 1024;
    return true;
}

static int ov_event(I2CSlave *i2c, enum i2c_event event)
{
    IngenicOv2735State *s = INGENIC_OV2735(i2c);

    if (event == I2C_START_SEND) {
        s->have_ptr = false;
    }
    return 0;
}

static int ov_send(I2CSlave *i2c, uint8_t data)
{
    IngenicOv2735State *s = INGENIC_OV2735(i2c);

    if (!s->have_ptr) {
        s->ptr = data;
        s->have_ptr = true;
        return 0;
    }
    if (s->ptr == OV_PAGE_SEL) {
        s->page = data;
    } else {
        s->regs[s->page % OV_PAGES][s->ptr] = data;
    }
    s->ptr++;
    return 0;
}

static uint8_t ov_recv(I2CSlave *i2c)
{
    IngenicOv2735State *s = INGENIC_OV2735(i2c);

    return ov_read(s, s->ptr++);
}

static void ov_reset(DeviceState *dev)
{
    IngenicOv2735State *s = INGENIC_OV2735(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->page = 0;
    s->ptr = 0;
    s->have_ptr = false;
}

static const VMStateDescription vmstate_ov2735 = {
    .name = TYPE_INGENIC_OV2735,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, IngenicOv2735State),
        VMSTATE_UINT8(page, IngenicOv2735State),
        VMSTATE_UINT8(ptr, IngenicOv2735State),
        VMSTATE_BOOL(have_ptr, IngenicOv2735State),
        VMSTATE_UINT8_2DARRAY(regs, IngenicOv2735State, OV_PAGES, 256),
        VMSTATE_END_OF_LIST()
    },
};

static const Property ov_properties[] = {
    DEFINE_PROP_UINT8("rev", IngenicOv2735State, rev, 0x07),
};

static void ov_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);

    device_class_set_legacy_reset(dc, ov_reset);
    device_class_set_props(dc, ov_properties);
    dc->vmsd = &vmstate_ov2735;
    k->event = ov_event;
    k->send = ov_send;
    k->recv = ov_recv;
}

static const TypeInfo ov_info = {
    .name          = TYPE_INGENIC_OV2735,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(IngenicOv2735State),
    .class_init    = ov_class_init,
};

static void ov_register_types(void)
{
    type_register_static(&ov_info);
}

type_init(ov_register_types)
