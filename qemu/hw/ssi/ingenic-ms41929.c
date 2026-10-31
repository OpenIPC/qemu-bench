/*
 * MS41929-type lens / pan-tilt stepper driver on an SSI bus, as the
 * Ingenic SDKs' sample_motor3 (ms419xx_spi_dev.c, motor.c) drives it.
 *
 * The SPI frames: an address byte (bit 6 set for a read), then a 16-bit
 * value, low byte first; LSB-first bit order on the wire, the chip
 * select active high -- the SSI model passes bytes in their logical
 * order and selects active low, so neither shows here.  Registers 0x20
 * to 0x2F hold 16 bits each and read back.  The sample sets 0x20 (clock
 * division), 0x22 / 0x27 (phase), 0x23 / 0x28 (PWM duty), 0x25 / 0x2A
 * (step period), and drives the two channels through 0x24 (AB) and 0x29
 * (CD):
 *
 *   bits 7:0 PSUM, the micro-steps per VD pulse; bit 8 the direction
 *   (0x3408 forward, 0x3508 reverse in the sample); bit 10 the channel
 *   enabled (0x3000 stops it); bits 15:12 the micro-step resolution
 *
 * Each falling edge on "vd" (the sample's VDFZ, GPIO PB25) moves every
 * enabled channel PSUM micro-steps; "reset" low (PB26) clears the
 * registers.  The field meanings are inferred from the values the sample
 * writes; the read-only properties "pos-ab" / "pos-cd" give each
 * channel's position in micro-steps.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qapi/visitor.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_MS41929 "ingenic-ms41929"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicMs41929State, INGENIC_MS41929)

#define REG_BASE        0x20
#define NREGS           16
#define REG_AB          0x24
#define REG_CD          0x29

#define CH_PSUM         0xffu
#define CH_REVERSE      (1u << 8)
#define CH_ENABLE       (1u << 10)

struct IngenicMs41929State {
    SSIPeripheral parent_obj;

    uint16_t regs[NREGS];
    uint8_t addr;               /* the frame's address byte */
    uint8_t pos;                /* bytes of this frame so far */
    uint16_t value;
    int32_t ch_pos[2];          /* AB, CD in micro-steps */
    uint32_t vd, nreset;
};

static uint16_t *ms_reg(IngenicMs41929State *s, uint8_t a)
{
    a &= 0x3f;
    return a >= REG_BASE && a < REG_BASE + NREGS ? &s->regs[a - REG_BASE]
                                                 : NULL;
}

static uint32_t ingenic_ms41929_transfer(SSIPeripheral *dev, uint32_t tx)
{
    IngenicMs41929State *s = INGENIC_MS41929(dev);
    uint16_t *r;
    uint32_t rx = 0;

    tx &= 0xff;
    if (s->pos == 0) {
        s->addr = tx;
        s->value = 0;
    } else if (s->pos <= 2) {
        r = ms_reg(s, s->addr);
        if (s->addr & 0x40) {
            rx = r ? (*r >> (8 * (s->pos - 1))) & 0xff : 0;
        } else {
            s->value |= tx << (8 * (s->pos - 1));
            if (s->pos == 2 && r && s->nreset) {
                *r = s->value;
            }
        }
    }
    if (s->pos < 3) {
        s->pos++;
    }
    return rx;
}

static int ingenic_ms41929_set_cs(SSIPeripheral *dev, bool select)
{
    IngenicMs41929State *s = INGENIC_MS41929(dev);

    s->pos = 0;                 /* each select starts a frame */
    return 0;
}

static void ms_step(IngenicMs41929State *s, int ch, uint16_t v)
{
    int32_t n = v & CH_PSUM;

    if (v & CH_ENABLE) {
        s->ch_pos[ch] += (v & CH_REVERSE) ? -n : n;
    }
}

static void ingenic_ms41929_vd(void *opaque, int n, int level)
{
    IngenicMs41929State *s = opaque;

    if (s->vd && !level && s->nreset) {
        ms_step(s, 0, *ms_reg(s, REG_AB));
        ms_step(s, 1, *ms_reg(s, REG_CD));
    }
    s->vd = level;
}

static void ingenic_ms41929_reset_pin(void *opaque, int n, int level)
{
    IngenicMs41929State *s = opaque;

    if (!level) {
        memset(s->regs, 0, sizeof(s->regs));
    }
    s->nreset = level;
}

static void ingenic_ms41929_get_pos(Object *obj, Visitor *v, const char *name,
                                    void *opaque, Error **errp)
{
    IngenicMs41929State *s = INGENIC_MS41929(obj);
    int32_t val = s->ch_pos[(uintptr_t)opaque];

    visit_type_int32(v, name, &val, errp);
}

static void ingenic_ms41929_realize(SSIPeripheral *dev, Error **errp)
{
    IngenicMs41929State *s = INGENIC_MS41929(dev);

    qdev_init_gpio_in_named(DEVICE(dev), ingenic_ms41929_vd, "vd", 1);
    qdev_init_gpio_in_named(DEVICE(dev), ingenic_ms41929_reset_pin,
                            "reset", 1);
    s->vd = 1;
    s->nreset = 1;
}

static void ingenic_ms41929_init(Object *obj)
{
    object_property_add(obj, "pos-ab", "int32", ingenic_ms41929_get_pos,
                        NULL, NULL, (void *)0);
    object_property_add(obj, "pos-cd", "int32", ingenic_ms41929_get_pos,
                        NULL, NULL, (void *)1);
}

static const VMStateDescription vmstate_ingenic_ms41929 = {
    .name = TYPE_INGENIC_MS41929,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, IngenicMs41929State),
        VMSTATE_UINT16_ARRAY(regs, IngenicMs41929State, NREGS),
        VMSTATE_UINT8(addr, IngenicMs41929State),
        VMSTATE_UINT8(pos, IngenicMs41929State),
        VMSTATE_UINT16(value, IngenicMs41929State),
        VMSTATE_INT32_ARRAY(ch_pos, IngenicMs41929State, 2),
        VMSTATE_UINT32(vd, IngenicMs41929State),
        VMSTATE_UINT32(nreset, IngenicMs41929State),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ms41929_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = ingenic_ms41929_realize;
    k->transfer = ingenic_ms41929_transfer;
    k->set_cs = ingenic_ms41929_set_cs;
    k->cs_polarity = SSI_CS_LOW;
    dc->vmsd = &vmstate_ingenic_ms41929;
}

static const TypeInfo ingenic_ms41929_info = {
    .name = TYPE_INGENIC_MS41929,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(IngenicMs41929State),
    .instance_init = ingenic_ms41929_init,
    .class_init = ingenic_ms41929_class_init,
};

static void ingenic_ms41929_register_types(void)
{
    type_register_static(&ingenic_ms41929_info);
}

type_init(ingenic_ms41929_register_types)
