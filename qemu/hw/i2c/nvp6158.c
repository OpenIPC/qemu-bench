/*
 * Nextchip NVP6158C four-channel AHD receiver, as the Xiongmai DVR Sofia
 * (nvp6158_* in its statically linked AD driver) finds and polls it.
 *
 * A banked register file: register 0xFF selects the bank, every bank holds
 * 256 bytes that read back what was written, except the status registers
 * below.  One write sets the register pointer; further bytes are data with
 * the pointer advancing.  What Sofia reads, from its code (no public
 * datasheet):
 *
 *   bank 0    0xF4  chip ID: 0xA0 NVP6158C (0xA1 6158, 0xC0 6168C ...)
 *             0xF5  revision
 *             0xA8  no-video bitmask, a bit per channel (the "cameras"
 *                   property: a 1 bit there is a camera, so a 0 here)
 *             0xA9  motion bitmask (0)
 *             0xE2  a bit per channel the EQ path reads (0)
 *   bank 0x13 0xF0+ch  the format code ("VFC") detected on channel ch
 *             0xC8/0xC9  cable distance, must not be 0
 *             0xD0..0xD7 two little-endian words the EQ subtracts; their
 *                   difference >> 20 must be 1..0xFFE
 *   bank 5+ch 0xF0  the format code again, sampled for stability
 *
 * Format codes (the "format" property): 0x31 1080p25 (the default), 0x30
 * 1080p30, 0x21 720p25, 0x20 720p30.  A channel without a camera reads
 * 0xFF there and its 0xA8 bit set.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "hw/arm/xiongmai.h"

#define TYPE_NVP6158 "nvp6158"
OBJECT_DECLARE_SIMPLE_TYPE(Nvp6158State, NVP6158)

#define NVP_BANKS   0x20

struct Nvp6158State {
    I2CSlave parent_obj;
    uint8_t regs[NVP_BANKS][256];
    uint8_t bank, ptr;
    bool have_ptr;
    uint8_t id, format, cameras;
    uint32_t logged;            /* accesses logged under -d unimp so far */
};

#define NVP_LOG_MAX 400

static void nvp6158_log(Nvp6158State *s, const char *op, uint8_t reg,
                        uint8_t v)
{
    if (s->logged < NVP_LOG_MAX) {
        s->logged++;
        qemu_log_mask(LOG_UNIMP, "nvp6158@0x%02x: %s bank 0x%02x reg 0x%02x "
                      "= 0x%02x\n", I2C_SLAVE(s)->address, op, s->bank, reg,
                      v);
    }
}

static uint8_t nvp6158_status(Nvp6158State *s, uint8_t bank, uint8_t reg,
                              bool *fixed)
{
    unsigned ch;

    *fixed = true;
    if (bank == 0) {
        switch (reg) {
        case 0xF4:
            return s->id;
        case 0xF5:
            return 0x01;
        case 0xA8:
            return ~s->cameras & 0x0F;
        case 0xA9:
        case 0xE2:
            return 0;
        }
    } else if (bank == 0x13) {
        if (reg >= 0xF0 && reg <= 0xF3) {
            ch = reg - 0xF0;
            return s->cameras & (1 << ch) ? s->format : 0xFF;
        }
        switch (reg) {
        case 0xC8:
            return 0;
        case 0xC9:
            return 0x10;
        case 0xD2:
            return 0x40;
        case 0xD0: case 0xD1: case 0xD3:
        case 0xD4: case 0xD5: case 0xD6: case 0xD7:
            return 0;
        }
    } else if (bank >= 5 && bank <= 8 && reg == 0xF0) {
        ch = bank - 5;
        return s->cameras & (1 << ch) ? s->format : 0xFF;
    }
    *fixed = false;
    return 0;
}

static int nvp6158_event(I2CSlave *i2c, enum i2c_event event)
{
    Nvp6158State *s = NVP6158(i2c);

    if (event == I2C_START_SEND) {
        s->have_ptr = false;
    }
    return 0;
}

static int nvp6158_send(I2CSlave *i2c, uint8_t data)
{
    Nvp6158State *s = NVP6158(i2c);

    if (!s->have_ptr) {
        s->ptr = data;
        s->have_ptr = true;
        return 0;
    }
    nvp6158_log(s, "write", s->ptr, data);
    if (s->ptr == 0xFF) {
        s->bank = data;
    } else if (s->bank < NVP_BANKS) {
        s->regs[s->bank][s->ptr] = data;
    }
    s->ptr++;
    return 0;
}

static uint8_t nvp6158_recv(I2CSlave *i2c)
{
    Nvp6158State *s = NVP6158(i2c);
    bool fixed;
    uint8_t v;

    if (s->ptr == 0xFF) {
        v = s->bank;
    } else {
        v = nvp6158_status(s, s->bank, s->ptr, &fixed);
        if (!fixed) {
            v = s->bank < NVP_BANKS ? s->regs[s->bank][s->ptr] : 0;
        }
    }
    nvp6158_log(s, "read", s->ptr, v);
    s->ptr++;
    return v;
}

static void nvp6158_reset(DeviceState *dev)
{
    Nvp6158State *s = NVP6158(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->bank = 0;
    s->ptr = 0;
    s->have_ptr = false;
}

/*
 * Bank 1 0xC0 + 2 * port (+1): the port's time slots, a nibble each, low
 * first; 0..3 / 8..0xB a channel (the 8s for the 960-wide 2-mux modes).
 */
static int nvp6158_camera(Object *obj, int vport, int slot)
{
    Nvp6158State *s = NVP6158(obj);
    uint8_t v;
    int ch;

    if (vport < 0 || vport > 3 || slot < 0 || slot > 3) {
        return -1;
    }
    v = s->regs[1][0xC0 + 2 * vport + slot / 2] >> (4 * (slot & 1)) & 0xF;
    ch = v & 3;
    return (v & 4) || !(s->cameras & (1 << ch)) ? -1 : ch;
}

static const Property nvp6158_props[] = {
    DEFINE_PROP_UINT8("id", Nvp6158State, id, 0xA0),
    DEFINE_PROP_UINT8("format", Nvp6158State, format, 0x31),
    DEFINE_PROP_UINT8("cameras", Nvp6158State, cameras, 0x0F),
};

static void nvp6158_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = nvp6158_event;
    k->send = nvp6158_send;
    k->recv = nvp6158_recv;
    device_class_set_legacy_reset(dc, nvp6158_reset);
    device_class_set_props(dc, nvp6158_props);
    XM_AHD_CLASS(klass)->camera = nvp6158_camera;
}

static const TypeInfo nvp6158_info = {
    .name = TYPE_NVP6158,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Nvp6158State),
    .class_init = nvp6158_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { TYPE_XM_AHD },
        { }
    },
};

static void nvp6158_register(void)
{
    type_register_static(&nvp6158_info);
}

type_init(nvp6158_register)
