/*
 * SOI (JX-*) image sensor I2C stubs: 8-bit register, 8-bit data, product ID
 * in registers 0x0A (high) and 0x0B (low).
 *
 *   hisi-f37    JX-F37, ID 0x0F37.  Sofia (XiongMai) probes it at 0x80
 *               (8-bit) = 0x40 (7-bit) and checks (0x0A << 8) | 0x0B.
 *   hisi-jxh62  JX-H62, ID 0xA062, at 0x60 (8-bit) = 0x30 (7-bit): the
 *               sensor on XM510 boards.  ipcinfo identifies it the same way.
 *
 * Everything else is RAM: the sensor tables the ISP writes read back.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_HISI_F37 "hisi-f37"
#define TYPE_HISI_JXH62 "hisi-jxh62"
OBJECT_DECLARE_TYPE(HisiF37State, HisiSoiClass, HISI_F37)

struct HisiSoiClass {
    I2CSlaveClass parent_class;
    uint16_t id;
};

#define F37_REG_COUNT 0x100

struct HisiF37State {
    I2CSlave parent_obj;
    uint8_t  cur_reg;
    bool     have_reg;
    uint8_t  regs[F37_REG_COUNT];
};

static void hisi_f37_reset_regs(HisiF37State *s)
{
    const uint16_t id = HISI_F37_GET_CLASS(s)->id;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x0A] = id >> 8;
    s->regs[0x0B] = id & 0xff;
}

static int hisi_f37_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiF37State *s = HISI_F37(i2c);

    if (event == I2C_START_SEND) {
        s->have_reg = false;
    }
    return 0;
}

static int hisi_f37_send(I2CSlave *i2c, uint8_t data)
{
    HisiF37State *s = HISI_F37(i2c);

    if (!s->have_reg) {
        s->cur_reg = data;
        s->have_reg = true;
    } else {
        s->regs[s->cur_reg] = data;
        s->cur_reg++;
    }
    return 0;
}

static uint8_t hisi_f37_recv(I2CSlave *i2c)
{
    HisiF37State *s = HISI_F37(i2c);
    uint8_t val = s->regs[s->cur_reg];
    s->cur_reg++;
    return val;
}

static void hisi_f37_reset(DeviceState *dev)
{
    HisiF37State *s = HISI_F37(dev);

    s->cur_reg = 0;
    s->have_reg = false;
    hisi_f37_reset_regs(s);
}

static void hisi_f37_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_f37_reset);
    k->event = hisi_f37_event;
    k->recv  = hisi_f37_recv;
    k->send  = hisi_f37_send;
    HISI_F37_CLASS(klass)->id = 0x0F37;
}

static void hisi_jxh62_class_init(ObjectClass *klass, const void *data)
{
    HISI_F37_CLASS(klass)->id = 0xA062;
}

static void hisi_f37_register_types(void)
{
    static const TypeInfo types[] = {
        {
            .name          = TYPE_HISI_F37,
            .parent        = TYPE_I2C_SLAVE,
            .instance_size = sizeof(HisiF37State),
            .class_size    = sizeof(HisiSoiClass),
            .class_init    = hisi_f37_class_init,
        }, {
            .name          = TYPE_HISI_JXH62,
            .parent        = TYPE_HISI_F37,
            .class_init    = hisi_jxh62_class_init,
        },
    };
    int i;

    for (i = 0; i < ARRAY_SIZE(types); i++) {
        type_register_static(&types[i]);
    }
}

type_init(hisi_f37_register_types)
