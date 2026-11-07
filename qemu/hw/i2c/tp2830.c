/*
 * Techpoint TP2830 four-channel HD-TVI / AHD receiver, as the Xiongmai DVR
 * Sofia (tp2830_* in its statically linked AD driver) finds and polls it.
 *
 * Register 0x40 selects a page: 0..3 a channel, 4 all four (writes go to
 * every channel, reads come from channel 0), 0x40 audio, 0x10 and 8 rarely
 * used.  Every register reads back what was written, except:
 *
 *   0xFF / 0xFD / 0xFE  chip ID: 0x30 0x00 (TP2830), revision 0x28
 *   per channel page:
 *   0x01  status: 0x78 an AHD signal locked (bit 7 loss, bits 6/5 lock,
 *         bit 2 clear: AHD, not TVI); 0x80 no camera
 *   0x03  the detected format, [2:0] 3 = 1080p25
 *   0x94  non-zero: AHD (0x00 / 0xFF read as CVI)
 *   0x04  the indirect port 0x2F selects: 0 the EQ value (0x10, stable),
 *         6 at least 0x0C (EQ settled), else 0x10
 *
 * Sofia probes 0x44.. first (0x40 = 0, then 0xFF and 0xFD); the chips that
 * answer give four channels each.  "cameras" is the channels with a camera,
 * a bit each.  From Sofia's code (no public datasheet).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"

#define TYPE_TP2830 "tp2830"
OBJECT_DECLARE_SIMPLE_TYPE(Tp2830State, TP2830)

/* stores: channel pages 0..3, then 8, 0x10, 0x40 and any other */
#define TP_STORES   8

struct Tp2830State {
    I2CSlave parent_obj;
    uint8_t regs[TP_STORES][256];
    uint8_t page, ptr;
    bool have_ptr;
    uint8_t cameras;
};

static int tp2830_store(uint8_t page)
{
    switch (page) {
    case 0: case 1: case 2: case 3:
        return page;
    case 4:
        return 0;
    case 8:
        return 4;
    case 0x10:
        return 5;
    case 0x40:
        return 6;
    default:
        return 7;
    }
}

static int tp2830_event(I2CSlave *i2c, enum i2c_event event)
{
    Tp2830State *s = TP2830(i2c);

    if (event == I2C_START_SEND) {
        s->have_ptr = false;
    }
    return 0;
}

static int tp2830_send(I2CSlave *i2c, uint8_t data)
{
    Tp2830State *s = TP2830(i2c);
    int n;

    if (!s->have_ptr) {
        s->ptr = data;
        s->have_ptr = true;
        return 0;
    }
    if (s->ptr == 0x40) {
        s->page = data;
    } else if (s->page == 4) {
        for (n = 0; n < 4; n++) {
            s->regs[n][s->ptr] = data;
        }
    } else {
        s->regs[tp2830_store(s->page)][s->ptr] = data;
    }
    s->ptr++;
    return 0;
}

static uint8_t tp2830_recv(I2CSlave *i2c)
{
    Tp2830State *s = TP2830(i2c);
    int st = tp2830_store(s->page);
    uint8_t r = s->ptr++;
    bool cam = st < 4 && (s->cameras & (1 << st));

    switch (r) {
    case 0x40:
        return s->page;
    case 0xFF:
        return 0x30;
    case 0xFD:
        return 0x00;
    case 0xFE:
        return 0x28;
    }
    if (st < 4) {
        switch (r) {
        case 0x01:
            return cam ? 0x78 : 0x80;
        case 0x03:
            return 0x03;
        case 0x94:
            return 0x01;
        case 0x04:
            return s->regs[st][0x2F] == 6 ? 0x20 : 0x10;
        }
    }
    return s->regs[st][r];
}

static void tp2830_reset(DeviceState *dev)
{
    Tp2830State *s = TP2830(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->page = 0;
    s->ptr = 0;
    s->have_ptr = false;
}

static const Property tp2830_props[] = {
    DEFINE_PROP_UINT8("cameras", Tp2830State, cameras, 0x0F),
};

static void tp2830_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = tp2830_event;
    k->send = tp2830_send;
    k->recv = tp2830_recv;
    device_class_set_legacy_reset(dc, tp2830_reset);
    device_class_set_props(dc, tp2830_props);
}

static const TypeInfo tp2830_info = {
    .name = TYPE_TP2830,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Tp2830State),
    .class_init = tp2830_class_init,
};

static void tp2830_register(void)
{
    type_register_static(&tp2830_info);
}

type_init(tp2830_register)
