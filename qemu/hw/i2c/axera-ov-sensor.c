/*
 * OmniVision/OS-series image sensor I2C stub.
 *
 * Generic model for OmniVision-family sensors that share the same probe
 * protocol: 16-bit register addresses, 8-bit data, a three-byte chip ID
 * at a configurable base register (0x300A/0x300B/0x300C for the parts
 * confirmed so far).
 *
 * AX620U's real deployed camera board (confirmed from the vendor SDK's
 * own riscv/drivers/sensors/ov_os04a10/os04a10.c: gOs04A10SlaveAddr[] =
 * {0x36, 0x36}, and os04a10_get_chipid() reading exactly
 * reg(0x300A)<<16 | reg(0x300B)<<8 | reg(0x300C) and comparing against
 * OS04A10_SENSOR_CHIP_ID = 0x530441) uses an OmniVision OS04A10, not a
 * SmartSens part -- see docs/axera.md's own "confirmed sensors" note.
 * The instance properties select the variant:
 *   id_reg_base   — register address of the first (most significant) ID byte
 *   id0/id1/id2   — the three ID bytes, most-significant first
 *
 * Confirmed variants (instance property values):
 *   OS04A10   id_reg_base=0x300A  id0=0x53  id1=0x04  id2=0x41
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_AXERA_OV_SENSOR "axera-ov-sensor"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraOvSensorState, AXERA_OV_SENSOR)

#define OV_REG_COUNT 0x1000
#define OV_REG_BASE  0x3000

struct AxeraOvSensorState {
    I2CSlave parent_obj;

    /* Variant selection (set via qdev properties) */
    uint16_t id_reg_base;
    uint8_t  id0;
    uint8_t  id1;
    uint8_t  id2;

    /* I2C state */
    uint8_t  reg_buf[2];   /* register address accumulator (big-endian) */
    uint8_t  reg_ptr;      /* bytes received for register address */
    uint16_t cur_reg;      /* current register address */
    uint8_t  regs[OV_REG_COUNT];

    /* MODE_SELECT (0x0100) bit 0, the OmniVision stream on/off control:
     * raised while the sensor is streaming, so the capture model behind
     * it can stay quiet until frames could really exist. */
    qemu_irq streaming;
};

#define OV_REG_MODE_SELECT 0x0100

static void axera_ov_sensor_reset_regs(AxeraOvSensorState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    if (s->id_reg_base >= OV_REG_BASE &&
        s->id_reg_base + 2 < OV_REG_BASE + OV_REG_COUNT) {
        s->regs[s->id_reg_base - OV_REG_BASE] = s->id0;
        s->regs[s->id_reg_base + 1 - OV_REG_BASE] = s->id1;
        s->regs[s->id_reg_base + 2 - OV_REG_BASE] = s->id2;
    }
}

static int axera_ov_sensor_event(I2CSlave *i2c, enum i2c_event event)
{
    AxeraOvSensorState *s = AXERA_OV_SENSOR(i2c);

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

static int axera_ov_sensor_send(I2CSlave *i2c, uint8_t data)
{
    AxeraOvSensorState *s = AXERA_OV_SENSOR(i2c);

    if (s->reg_ptr < 2) {
        s->reg_buf[s->reg_ptr++] = data;
    } else {
        uint16_t addr = ((uint16_t)s->reg_buf[0] << 8) | s->reg_buf[1];

        if (addr >= OV_REG_BASE && addr < OV_REG_BASE + OV_REG_COUNT) {
            s->regs[addr - OV_REG_BASE] = data;
        }
        if (getenv("AXERA_VIN_DEBUG") && addr >= 0x0100 && addr < 0x0110) {
            fprintf(stderr, "axera-ov-sensor: [0x%04x] = 0x%02x\n", addr, data);
        }
        if (addr == OV_REG_MODE_SELECT) {
            qemu_set_irq(s->streaming, data & 1);
        }
        s->reg_buf[1]++;
        if (s->reg_buf[1] == 0) {
            s->reg_buf[0]++;
        }
    }
    return 0;
}

static uint8_t axera_ov_sensor_recv(I2CSlave *i2c)
{
    AxeraOvSensorState *s = AXERA_OV_SENSOR(i2c);
    uint8_t val = 0xFF;

    if (s->cur_reg >= OV_REG_BASE && s->cur_reg < OV_REG_BASE + OV_REG_COUNT) {
        val = s->regs[s->cur_reg - OV_REG_BASE];
    }
    s->cur_reg++;
    return val;
}

static void axera_ov_sensor_reset(DeviceState *dev)
{
    AxeraOvSensorState *s = AXERA_OV_SENSOR(dev);

    s->reg_ptr = 0;
    s->cur_reg = 0;
    memset(s->reg_buf, 0, sizeof(s->reg_buf));
    axera_ov_sensor_reset_regs(s);
    qemu_set_irq(s->streaming, 0);
}

static void axera_ov_sensor_init(Object *obj)
{
    AxeraOvSensorState *s = AXERA_OV_SENSOR(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->streaming, "streaming", 1);
}

static const Property axera_ov_sensor_props[] = {
    DEFINE_PROP_UINT16("id_reg_base", AxeraOvSensorState, id_reg_base,
                       0x300A),
    DEFINE_PROP_UINT8("id0", AxeraOvSensorState, id0, 0x00),
    DEFINE_PROP_UINT8("id1", AxeraOvSensorState, id1, 0x00),
    DEFINE_PROP_UINT8("id2", AxeraOvSensorState, id2, 0x00),
};

static const VMStateDescription vmstate_axera_ov_sensor = {
    .name = TYPE_AXERA_OV_SENSOR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, AxeraOvSensorState),
        VMSTATE_UINT8_ARRAY(reg_buf, AxeraOvSensorState, 2),
        VMSTATE_UINT8(reg_ptr, AxeraOvSensorState),
        VMSTATE_UINT16(cur_reg, AxeraOvSensorState),
        VMSTATE_UINT8_ARRAY(regs, AxeraOvSensorState, OV_REG_COUNT),
        VMSTATE_END_OF_LIST()
    }
};

static void axera_ov_sensor_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, axera_ov_sensor_reset);
    dc->vmsd = &vmstate_axera_ov_sensor;
    device_class_set_props(dc, axera_ov_sensor_props);
    k->event = axera_ov_sensor_event;
    k->recv  = axera_ov_sensor_recv;
    k->send  = axera_ov_sensor_send;
}

static const TypeInfo axera_ov_sensor_info = {
    .name          = TYPE_AXERA_OV_SENSOR,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(AxeraOvSensorState),
    .instance_init = axera_ov_sensor_init,
    .class_init    = axera_ov_sensor_class_init,
};

static void axera_ov_sensor_register_types(void)
{
    type_register_static(&axera_ov_sensor_info);
}

type_init(axera_ov_sensor_register_types)
