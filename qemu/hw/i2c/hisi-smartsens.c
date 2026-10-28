/*
 * SmartSens (SC*) image sensor I2C stub.
 *
 * Generic model for SmartSens sensors that share the same probe protocol:
 * 16-bit register addresses, 8-bit data, ID at 0x3107/0x3108 with optional
 * discriminator at 0x3109.
 *
 * Sofia (XiongMai) detects these sensors at I2C addr 0x60 (8-bit) =
 * 0x30 (7-bit slave addr).  All variants are identified by a unique
 * 16-bit ID composed of regs 0x3107 (high) and 0x3108 (low).  Several
 * variants share the same ID (e.g. SC2235P and SC2235E both report
 * 0x2232) and are disambiguated by the value of reg 0x3109.
 *
 * The instance properties select the variant:
 *   id_high       — value returned for reg 0x3107
 *   id_low        — value returned for reg 0x3108
 *   disc          — value returned for reg 0x3109 (discriminator)
 *
 * Sofia variants currently supported (instance type names):
 *   hisi-sc2235p   ID 0x2232  disc 0x01
 *   hisi-sc2235e   ID 0x2232  disc 0x20
 *   hisi-sc2315    ID 0x2311  disc 0x00
 *   hisi-sc2315e   ID 0x2238  disc 0x00
 *   hisi-sc2335    ID 0xCB14  disc 0x00
 *   hisi-sc2239    ID 0xCB10  disc 0x00
 *   hisi-sc307h    ID 0xCB1C  disc 0x00
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/xm-regtrace.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_HISI_SMARTSENS "hisi-smartsens"
OBJECT_DECLARE_SIMPLE_TYPE(HisiSmartSensState, HISI_SMARTSENS)

#define SS_REG_BASE  0x3000
#define SS_REG_COUNT 0x1000

struct HisiSmartSensState {
    I2CSlave parent_obj;

    /* Variant selection (set via qdev properties) */
    uint8_t  id_high;
    uint8_t  id_low;
    uint8_t  disc;
    uint8_t reg_801e;  /* SC2336P/SC2337P variant register */
    uint8_t bus;       /* board bus number (XM_REGTRACE) */

    /* I2C state */
    uint8_t  reg_buf[2];   /* register address accumulator (big-endian) */
    uint8_t  reg_ptr;      /* bytes received for register address */
    uint16_t cur_reg;      /* current register address */
    uint8_t  regs[SS_REG_COUNT];
};

static void hisi_smartsens_reset_regs(HisiSmartSensState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    /* Sofia checks reg 0x3107/0x3108 (ID) and optionally reg 0x3109 (discriminator) */
    s->regs[0x3107 - SS_REG_BASE] = s->id_high;
    s->regs[0x3108 - SS_REG_BASE] = s->id_low;
    s->regs[0x3109 - SS_REG_BASE] = s->disc;
}

static int hisi_smartsens_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiSmartSensState *s = HISI_SMARTSENS(i2c);

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

static int hisi_smartsens_send(I2CSlave *i2c, uint8_t data)
{
    HisiSmartSensState *s = HISI_SMARTSENS(i2c);

    if (s->reg_ptr < 2) {
        s->reg_buf[s->reg_ptr++] = data;
    } else {
        uint16_t addr = ((uint16_t)s->reg_buf[0] << 8) | s->reg_buf[1];
        xm_regtrace("sensor", ((uint32_t)s->bus << 24) |
                    ((uint32_t)i2c->address << 16) | addr, data);
        if (addr >= SS_REG_BASE &&
            addr < SS_REG_BASE + SS_REG_COUNT) {
            s->regs[addr - SS_REG_BASE] = data;
        }
        s->reg_buf[1]++;
        if (s->reg_buf[1] == 0) {
            s->reg_buf[0]++;
        }
    }
    return 0;
}

static uint8_t hisi_smartsens_recv(I2CSlave *i2c)
{
    HisiSmartSensState *s = HISI_SMARTSENS(i2c);
    uint8_t val = 0xFF;

    if (s->cur_reg == 0x801e) {
        val = s->reg_801e;
    } else if (s->cur_reg >= SS_REG_BASE &&
        s->cur_reg < SS_REG_BASE + SS_REG_COUNT) {
        val = s->regs[s->cur_reg - SS_REG_BASE];
    }
    s->cur_reg++;
    return val;
}

static void hisi_smartsens_reset(DeviceState *dev)
{
    HisiSmartSensState *s = HISI_SMARTSENS(dev);

    s->reg_ptr = 0;
    s->cur_reg = 0;
    memset(s->reg_buf, 0, sizeof(s->reg_buf));
    hisi_smartsens_reset_regs(s);
}

static const Property hisi_smartsens_props[] = {
    DEFINE_PROP_UINT8("id_high", HisiSmartSensState, id_high, 0x00),
    DEFINE_PROP_UINT8("id_low",  HisiSmartSensState, id_low,  0x00),
    DEFINE_PROP_UINT8("disc",    HisiSmartSensState, disc,    0x00),
    DEFINE_PROP_UINT8("reg_801e", HisiSmartSensState, reg_801e, 0xff),
    /* the board's bus number, for the XM_REGTRACE log only */
    DEFINE_PROP_UINT8("bus",     HisiSmartSensState, bus,     0x00),
};

/* What the guest programmed: the register file and the address pointer. */
static const VMStateDescription vmstate_hisi_smartsens = {
    .name = TYPE_HISI_SMARTSENS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, HisiSmartSensState),
        VMSTATE_UINT8_ARRAY(reg_buf, HisiSmartSensState, 2),
        VMSTATE_UINT8(reg_ptr, HisiSmartSensState),
        VMSTATE_UINT16(cur_reg, HisiSmartSensState),
        VMSTATE_UINT8_ARRAY(regs, HisiSmartSensState, SS_REG_COUNT),
        VMSTATE_END_OF_LIST()
    }
};

static void hisi_smartsens_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_smartsens_reset);
    dc->vmsd = &vmstate_hisi_smartsens;
    device_class_set_props(dc, hisi_smartsens_props);
    k->event = hisi_smartsens_event;
    k->recv  = hisi_smartsens_recv;
    k->send  = hisi_smartsens_send;
}

static const TypeInfo hisi_smartsens_info = {
    .name          = TYPE_HISI_SMARTSENS,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HisiSmartSensState),
    .class_init    = hisi_smartsens_class_init,
};

static void hisi_smartsens_register_types(void)
{
    type_register_static(&hisi_smartsens_info);
}

type_init(hisi_smartsens_register_types)
