/*
 * NXP PCAL6416A I2C GPIO-expander stub.
 *
 * AX620U's real devicetree (openipc-linux-ax620u's ax620u.dts) declares
 * expander_io@0x20 on i2c@49b4000 (= /dev/i2c-8, the SAME bus the real
 * OS04A10 sensor sits on at 0x36 -- see hw/i2c/axera-ov-sensor.c's own
 * comment). This stub exists because a real AX620U firmware dump's App
 * process logs, around the same ~120ms window as its camera pipe's own
 * "HAL_Reset_init pipe 0" step, a burst of
 *   "msg i2c_read_byte i2c read error: -121"
 *   "msg i2c_write_byte i2c write error: -121"
 * immediately followed by AX_VIN_SetChnAttr[0] failing with
 * "Error [80010180]: Close all chn is not allowed" -- -121 is EREMOTEIO
 * (no ACK), which is exactly what a real dw-i2c bus correctly reports
 * for a transaction addressed to 0x20 when nothing is attached there,
 * as was the case before this file existed (only the sensor at 0x36 was
 * modelled on this bus).
 *
 * Confirmed NOT a guess: the real deployed ax_pcal6416a.ko (extracted
 * from the dump's own usr partition, unstripped) contains the literal
 * printf format strings "msg %s i2c read error: %d" / "msg %s i2c
 * write error: %d" (the %s is the caller's own helper function name,
 * "i2c_read_byte"/"i2c_write_byte", which is exactly what renders as
 * the console text above) -- this is the one real kernel module whose
 * strings match that exact message, not a loosely-matched guess.
 * Its own disassembly (pcal6416a_gpio_get/_set/_direction_output/
 * _direction_input/_get_port_direction_by_mask, all via the module's
 * generic i2c_read_byte()/i2c_write_byte() SMBus-style helpers) uses
 * only registers 0/1 (Input Port 0/1), 2/3 (Output Port 0/1) and 6/7
 * (Configuration Port 0/1, 1=input/0=output, same convention as every
 * other TCA6408/PCA9539/PCAL6416A-family part) -- never touches the
 * polarity-inversion, latch, pull-up/down or interrupt-mask registers
 * a real PCAL6416A also has, so this stub only needs to model those
 * six registers faithfully; nothing in the real driver's own code path
 * reads/writes the rest.
 *
 * Reset defaults (NXP PCAL6416A datasheet): Output Port 0/1 = 0xFF,
 * Configuration Port 0/1 = 0xFF (every pin starts as an input). Input
 * Port reads mirror whatever was last written to the Output Port for
 * pins actually driven as outputs -- a reasonable loopback model for a
 * board-internal control line (sensor reset/power-enable) with no
 * external pull fighting the driver, and harmless for the handful of
 * pins the real driver never reads back at all.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_AXERA_PCAL6416A "axera-pcal6416a"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraPcal6416aState, AXERA_PCAL6416A)

#define PCAL_REG_INPUT0   0x00
#define PCAL_REG_INPUT1   0x01
#define PCAL_REG_OUTPUT0  0x02
#define PCAL_REG_OUTPUT1  0x03
#define PCAL_REG_CONFIG0  0x06
#define PCAL_REG_CONFIG1  0x07

struct AxeraPcal6416aState {
    I2CSlave parent_obj;

    uint8_t output0, output1;
    uint8_t config0, config1;

    /* I2C transaction state */
    uint8_t cur_reg;
    bool    have_reg;
};

static void axera_pcal6416a_reset_regs(AxeraPcal6416aState *s)
{
    s->output0 = 0xFF;
    s->output1 = 0xFF;
    s->config0 = 0xFF;
    s->config1 = 0xFF;
    s->have_reg = false;
    s->cur_reg = 0;
}

static int axera_pcal6416a_event(I2CSlave *i2c, enum i2c_event event)
{
    AxeraPcal6416aState *s = AXERA_PCAL6416A(i2c);

    if (event == I2C_START_SEND) {
        s->have_reg = false;
    }
    return 0;
}

static int axera_pcal6416a_send(I2CSlave *i2c, uint8_t data)
{
    AxeraPcal6416aState *s = AXERA_PCAL6416A(i2c);

    if (!s->have_reg) {
        s->cur_reg = data;
        s->have_reg = true;
        return 0;
    }

    switch (s->cur_reg) {
    case PCAL_REG_OUTPUT0:
        s->output0 = data;
        break;
    case PCAL_REG_OUTPUT1:
        s->output1 = data;
        break;
    case PCAL_REG_CONFIG0:
        s->config0 = data;
        break;
    case PCAL_REG_CONFIG1:
        s->config1 = data;
        break;
    case PCAL_REG_INPUT0:
    case PCAL_REG_INPUT1:
        /* Read-only on real silicon; ACK and ignore, like real parts do. */
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "axera-pcal6416a: write to unmodelled reg 0x%x "
                      "(driver only ever touches 0/1/2/3/6/7)\n",
                      s->cur_reg);
        break;
    }
    /* Real chip auto-increments the register pointer on multi-byte writes. */
    s->cur_reg++;
    return 0;
}

static uint8_t axera_pcal6416a_recv(I2CSlave *i2c)
{
    AxeraPcal6416aState *s = AXERA_PCAL6416A(i2c);
    uint8_t val;

    switch (s->cur_reg) {
    case PCAL_REG_INPUT0:
        /* Loopback: pins driven as output (config bit clear) read back
         * what was last written there; pins left as input float high. */
        val = (uint8_t)(s->output0 | s->config0);
        break;
    case PCAL_REG_INPUT1:
        val = (uint8_t)(s->output1 | s->config1);
        break;
    case PCAL_REG_OUTPUT0:
        val = s->output0;
        break;
    case PCAL_REG_OUTPUT1:
        val = s->output1;
        break;
    case PCAL_REG_CONFIG0:
        val = s->config0;
        break;
    case PCAL_REG_CONFIG1:
        val = s->config1;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "axera-pcal6416a: read from unmodelled reg 0x%x\n",
                      s->cur_reg);
        val = 0x00;
        break;
    }
    s->cur_reg++;
    return val;
}

static void axera_pcal6416a_reset(DeviceState *dev)
{
    AxeraPcal6416aState *s = AXERA_PCAL6416A(dev);

    axera_pcal6416a_reset_regs(s);
}

static const VMStateDescription vmstate_axera_pcal6416a = {
    .name = TYPE_AXERA_PCAL6416A,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, AxeraPcal6416aState),
        VMSTATE_UINT8(output0, AxeraPcal6416aState),
        VMSTATE_UINT8(output1, AxeraPcal6416aState),
        VMSTATE_UINT8(config0, AxeraPcal6416aState),
        VMSTATE_UINT8(config1, AxeraPcal6416aState),
        VMSTATE_UINT8(cur_reg, AxeraPcal6416aState),
        VMSTATE_BOOL(have_reg, AxeraPcal6416aState),
        VMSTATE_END_OF_LIST()
    }
};

static void axera_pcal6416a_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, axera_pcal6416a_reset);
    dc->vmsd = &vmstate_axera_pcal6416a;
    k->event = axera_pcal6416a_event;
    k->recv  = axera_pcal6416a_recv;
    k->send  = axera_pcal6416a_send;
}

static const TypeInfo axera_pcal6416a_info = {
    .name          = TYPE_AXERA_PCAL6416A,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(AxeraPcal6416aState),
    .class_init    = axera_pcal6416a_class_init,
};

static void axera_pcal6416a_register_types(void)
{
    type_register_static(&axera_pcal6416a_info);
}

type_init(axera_pcal6416a_register_types)
