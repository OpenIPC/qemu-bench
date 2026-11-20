/*
 * Everest ES8311 mono audio codec, I2C control interface (7-bit address
 * 0x18 on the RV1109 camera board's I2C4).  QOM type "es8311".
 *
 * 256 8-bit registers behind an 8-bit register address that
 * auto-increments.  The kernel's sound/soc/codecs/es8311.c reads the chip
 * ID at 0xFD / 0xFE (0x83, 0x11) and otherwise programs the clocking,
 * the serial port and the analogue path; here those are storage.  No
 * audio passes through: the I2S model supplies silence.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_ES8311 "es8311"
OBJECT_DECLARE_SIMPLE_TYPE(Es8311State, ES8311)

struct Es8311State {
    I2CSlave parent_obj;

    uint8_t regs[256];
    uint8_t ptr;
    bool addr_phase;
};

static int es8311_event(I2CSlave *i2c, enum i2c_event event)
{
    Es8311State *s = ES8311(i2c);

    if (event == I2C_START_SEND) {
        s->addr_phase = true;
    }
    return 0;
}

static uint8_t es8311_recv(I2CSlave *i2c)
{
    Es8311State *s = ES8311(i2c);

    return s->regs[s->ptr++];
}

static int es8311_send(I2CSlave *i2c, uint8_t data)
{
    Es8311State *s = ES8311(i2c);

    if (s->addr_phase) {
        s->ptr = data;
        s->addr_phase = false;
    } else if (s->ptr < 0xfd) {         /* the ID registers are read-only */
        s->regs[s->ptr++] = data;
    } else {
        s->ptr++;
    }
    return 0;
}

static void es8311_reset(DeviceState *dev)
{
    Es8311State *s = ES8311(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0xfd] = 0x83;               /* CHIP ID1 */
    s->regs[0xfe] = 0x11;               /* CHIP ID2 */
    s->ptr = 0;
    s->addr_phase = false;
}

static const VMStateDescription es8311_vmstate = {
    .name = TYPE_ES8311,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, Es8311State),
        VMSTATE_UINT8_ARRAY(regs, Es8311State, 256),
        VMSTATE_UINT8(ptr, Es8311State),
        VMSTATE_BOOL(addr_phase, Es8311State),
        VMSTATE_END_OF_LIST()
    }
};

static void es8311_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);

    k->event = es8311_event;
    k->recv = es8311_recv;
    k->send = es8311_send;
    device_class_set_legacy_reset(dc, es8311_reset);
    dc->vmsd = &es8311_vmstate;
}

static const TypeInfo es8311_info = {
    .name = TYPE_ES8311,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Es8311State),
    .class_init = es8311_class_init,
};

static void es8311_register_types(void)
{
    type_register_static(&es8311_info);
}

type_init(es8311_register_types)
