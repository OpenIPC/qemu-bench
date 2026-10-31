/*
 * MS32006-type dual stepper driver on I2C (address 0x10), as the Ingenic
 * SDKs' sample_motor_i2c (motor.c) drives it.
 *
 * Byte registers, read and written one at a time: a write sends the
 * register number in one message and the value in the next (with or
 * without a repeated start between them -- a write that follows a
 * pointer-only write without a stop carries data), a read sets the
 * pointer and reads a byte.  The sample's use, from which the fields
 * here are inferred:
 *
 *   0x00  bit 0 the chip on (0: reset), bit 2 / bit 1 pause motor A / B,
 *         bit 7 four-phase five-wire motors
 *   0x01 / 0x02  motor A's speed: 14 bits, bit 15 half-stepping; a step
 *         takes speed * 128 cycles of the 24 MHz clock (fclk / pps / 128)
 *   0x03 / 0x04  motor A's step count: 12 bits; bit 15 the output on,
 *         bit 14 counter-clockwise
 *   0x05..0x08   the same for motor B
 *   0x09  bit 7 / bit 6 load and run motor A / B (cleared when the run
 *         ends), bits 3:2 the IR-cut output (1 one way, 2 the other)
 *
 * A run counts its steps down in virtual time; the read-only properties
 * "pos-a" / "pos-b" give each motor's position in steps (clockwise
 * positive) and "ircut" the IR-cut output.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/visitor.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_MS32006 "ingenic-ms32006"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicMs32006State, INGENIC_MS32006)

#define NREGS           16
#define FCLK_HZ         24000000ULL

typedef struct Ms32006Motor {
    int32_t pos;                /* steps, clockwise positive */
    int32_t left;               /* steps still to go */
    int32_t dir;                /* +1 / -1 */
    int64_t last_ns;            /* when the last step was counted */
} Ms32006Motor;

struct IngenicMs32006State {
    I2CSlave parent_obj;

    uint8_t regs[NREGS];
    uint8_t ptr;
    uint8_t nbytes;             /* bytes of this write transfer */
    uint8_t ptr_only;           /* the last write set only the pointer */
    uint8_t stopped;            /* a stop since the last transfer */
    Ms32006Motor m[2];
    QEMUTimer *timer;
};

static int64_t ms_step_ns(IngenicMs32006State *s, int i)
{
    uint32_t speed = (s->regs[1 + 4 * i] | (s->regs[2 + 4 * i] << 8)) &
                     0x3fff;

    return MAX(speed, 32) * 128 * NANOSECONDS_PER_SECOND / FCLK_HZ;
}

/* bring each running motor's count up to now */
static void ms_advance(IngenicMs32006State *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int i;

    for (i = 0; i < 2; i++) {
        Ms32006Motor *m = &s->m[i];
        int64_t step = ms_step_ns(s, i);
        int64_t n;

        if (!m->left || !(s->regs[0] & 1) ||
            (s->regs[0] & (i ? 0x02 : 0x04))) {
            m->last_ns = now;
            continue;
        }
        n = MIN((now - m->last_ns) / step, m->left);
        m->pos += n * m->dir;
        m->left -= n;
        m->last_ns += n * step;
        if (!m->left) {
            s->regs[9] &= ~(i ? 0x40 : 0x80);
        }
    }
}

static void ms_schedule(IngenicMs32006State *s)
{
    int64_t next = INT64_MAX;
    int i;

    for (i = 0; i < 2; i++) {
        if (s->m[i].left) {
            next = MIN(next, s->m[i].last_ns +
                       ms_step_ns(s, i) * s->m[i].left);
        }
    }
    if (next == INT64_MAX) {
        timer_del(s->timer);
    } else {
        timer_mod(s->timer, next);
    }
}

static void ms_tick(void *opaque)
{
    IngenicMs32006State *s = opaque;

    ms_advance(s);
    ms_schedule(s);
}

/* register 9 written: load the motors whose bits are set */
static void ms_load(IngenicMs32006State *s, uint8_t v)
{
    int i;

    for (i = 0; i < 2; i++) {
        Ms32006Motor *m = &s->m[i];
        uint8_t hi = s->regs[4 + 4 * i];

        if (!(v & (i ? 0x40 : 0x80))) {
            continue;
        }
        m->left = (hi & 0x80) ? (s->regs[3 + 4 * i] | ((hi & 0x0f) << 8))
                              : 0;
        m->dir = (hi & 0x40) ? -1 : 1;
        m->last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if (!m->left) {
            s->regs[9] &= ~(i ? 0x40 : 0x80);
        }
    }
}

static void ms_write(IngenicMs32006State *s, uint8_t r, uint8_t v)
{
    if (r >= NREGS) {
        return;
    }
    ms_advance(s);
    s->regs[r] = v;
    if (r == 0 && !(v & 1)) {
        s->m[0].left = s->m[1].left = 0;        /* the chip held in reset */
        s->regs[9] &= ~0xc0;
    }
    if (r == 9) {
        ms_load(s, v);
    }
    ms_schedule(s);
}

static int ingenic_ms32006_event(I2CSlave *i2c, enum i2c_event event)
{
    IngenicMs32006State *s = INGENIC_MS32006(i2c);

    switch (event) {
    case I2C_START_SEND:
        /* a write right after a pointer-only one, no stop between: data */
        s->nbytes = (!s->stopped && s->ptr_only) ? 1 : 0;
        s->ptr_only = 0;
        s->stopped = 0;
        break;
    case I2C_START_RECV:
        s->ptr_only = 0;
        s->stopped = 0;
        break;
    case I2C_FINISH:
        s->stopped = 1;
        break;
    default:
        break;
    }
    return 0;
}

static int ingenic_ms32006_send(I2CSlave *i2c, uint8_t data)
{
    IngenicMs32006State *s = INGENIC_MS32006(i2c);

    if (s->nbytes == 0) {
        s->ptr = data;
        s->ptr_only = 1;
    } else {
        ms_write(s, s->ptr++, data);
        s->ptr_only = 0;
    }
    s->nbytes++;
    return 0;
}

static uint8_t ingenic_ms32006_recv(I2CSlave *i2c)
{
    IngenicMs32006State *s = INGENIC_MS32006(i2c);

    ms_advance(s);
    return s->ptr < NREGS ? s->regs[s->ptr++] : 0;
}

static void ingenic_ms32006_get_pos(Object *obj, Visitor *v, const char *name,
                                    void *opaque, Error **errp)
{
    IngenicMs32006State *s = INGENIC_MS32006(obj);
    int32_t val;

    ms_advance(s);
    val = s->m[(uintptr_t)opaque].pos;
    visit_type_int32(v, name, &val, errp);
}

static char *ingenic_ms32006_get_ircut(Object *obj, Error **errp)
{
    IngenicMs32006State *s = INGENIC_MS32006(obj);

    switch ((s->regs[9] >> 2) & 3) {
    case 1:
        return g_strdup("cw");
    case 2:
        return g_strdup("ccw");
    default:
        return g_strdup("off");
    }
}

static void ingenic_ms32006_reset(DeviceState *dev)
{
    IngenicMs32006State *s = INGENIC_MS32006(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->m, 0, sizeof(s->m));
    s->ptr = s->nbytes = s->ptr_only = 0;
    s->stopped = 1;
}

static void ingenic_ms32006_init(Object *obj)
{
    IngenicMs32006State *s = INGENIC_MS32006(obj);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ms_tick, s);
    object_property_add(obj, "pos-a", "int32", ingenic_ms32006_get_pos,
                        NULL, NULL, (void *)0);
    object_property_add(obj, "pos-b", "int32", ingenic_ms32006_get_pos,
                        NULL, NULL, (void *)1);
    object_property_add_str(obj, "ircut", ingenic_ms32006_get_ircut, NULL);
}

static const VMStateDescription vmstate_ms32006_motor = {
    .name = "ingenic-ms32006-motor",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT32(pos, Ms32006Motor),
        VMSTATE_INT32(left, Ms32006Motor),
        VMSTATE_INT32(dir, Ms32006Motor),
        VMSTATE_INT64(last_ns, Ms32006Motor),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_ms32006 = {
    .name = TYPE_INGENIC_MS32006,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, IngenicMs32006State),
        VMSTATE_UINT8_ARRAY(regs, IngenicMs32006State, NREGS),
        VMSTATE_UINT8(ptr, IngenicMs32006State),
        VMSTATE_UINT8(nbytes, IngenicMs32006State),
        VMSTATE_UINT8(ptr_only, IngenicMs32006State),
        VMSTATE_UINT8(stopped, IngenicMs32006State),
        VMSTATE_STRUCT_ARRAY(m, IngenicMs32006State, 2, 1,
                             vmstate_ms32006_motor, Ms32006Motor),
        VMSTATE_TIMER_PTR(timer, IngenicMs32006State),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ms32006_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = ingenic_ms32006_event;
    k->send = ingenic_ms32006_send;
    k->recv = ingenic_ms32006_recv;
    device_class_set_legacy_reset(dc, ingenic_ms32006_reset);
    dc->vmsd = &vmstate_ingenic_ms32006;
}

static const TypeInfo ingenic_ms32006_info = {
    .name = TYPE_INGENIC_MS32006,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(IngenicMs32006State),
    .instance_init = ingenic_ms32006_init,
    .class_init = ingenic_ms32006_class_init,
};

static void ingenic_ms32006_register_types(void)
{
    type_register_static(&ingenic_ms32006_info);
}

type_init(ingenic_ms32006_register_types)
