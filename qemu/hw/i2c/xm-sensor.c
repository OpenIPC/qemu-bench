/*
 * A generic image sensor on I2C: a register file with 8- or 16-bit
 * register addresses and 8-bit data, whose chip ID and other power-on
 * values come from a property.  It stands in for sensors with no model of
 * their own, so a vendor ISP that probes a list of sensors can be shown
 * any one of them by its ID, and every access can be logged.
 *
 *   -M xm510,sensor=xm-sensor,sensor-addr=0x30
 *   -global xm-sensor.addr-bytes=2
 *   -global xm-sensor.regs=0x3107:0x22,0x3108:0x35
 *   -global xm-sensor.log=/path/sensor.log
 *
 * The log has one line per register access, "W reg val" or "R reg val",
 * in hex.  Registers not preset read 0.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qemu/module.h"

#define TYPE_XM_SENSOR "xm-sensor"
OBJECT_DECLARE_SIMPLE_TYPE(XmSensorState, XM_SENSOR)

#define XM_SENSOR_REGS 0x10000

struct XmSensorState {
    I2CSlave parent_obj;

    uint8_t addr_bytes; /* 1 or 2 */
    char *regs_preset;  /* "reg:val,reg:val", hex or decimal */
    char *log_path;
    FILE *log;

    uint8_t ptr;        /* register address bytes received */
    uint16_t cur;
    uint8_t regs[XM_SENSOR_REGS];
};

/* A log that can't be written is reported once and closed. */
static void xm_sensor_log(XmSensorState *s, char op, uint16_t reg,
                          uint8_t val)
{
    if (!s->log) {
        return;
    }
    if (fprintf(s->log, "%c %04x %02x\n", op, reg, val) < 0 ||
        fflush(s->log) != 0) {
        warn_report("xm-sensor: writing %s: %s; no more log", s->log_path,
                    strerror(errno));
        fclose(s->log);
        s->log = NULL;
    }
}

static int xm_sensor_event(I2CSlave *i2c, enum i2c_event event)
{
    XmSensorState *s = XM_SENSOR(i2c);

    if (event == I2C_START_SEND) {
        s->ptr = 0;
    }
    return 0;
}

static int xm_sensor_send(I2CSlave *i2c, uint8_t data)
{
    XmSensorState *s = XM_SENSOR(i2c);

    if (s->ptr < s->addr_bytes) {
        s->cur = s->ptr ? (s->cur << 8) | data : data;
        s->ptr++;
        return 0;
    }
    s->regs[s->cur] = data;
    xm_sensor_log(s, 'W', s->cur, data);
    s->cur++;
    if (s->addr_bytes == 1) {
        s->cur &= 0xff;
    }
    return 0;
}

static uint8_t xm_sensor_recv(I2CSlave *i2c)
{
    XmSensorState *s = XM_SENSOR(i2c);
    uint8_t val = s->regs[s->cur];

    xm_sensor_log(s, 'R', s->cur, val);
    s->cur++;
    if (s->addr_bytes == 1) {
        s->cur &= 0xff;
    }
    return val;
}

static void xm_sensor_preset(XmSensorState *s, Error **errp)
{
    g_auto(GStrv) pairs = NULL;

    memset(s->regs, 0, sizeof(s->regs));
    if (!s->regs_preset || !*s->regs_preset) {
        return;
    }
    const uint64_t regs = s->addr_bytes == 1 ? 0x100 : XM_SENSOR_REGS;

    pairs = g_strsplit(s->regs_preset, ",", -1);
    for (int i = 0; pairs[i]; i++) {
        uint64_t reg, val;
        const char *colon = strchr(pairs[i], ':');
        g_autofree char *r = colon ? g_strndup(pairs[i], colon - pairs[i])
                                   : NULL;

        if (!r || qemu_strtou64(r, NULL, 0, &reg) < 0 ||
            qemu_strtou64(colon + 1, NULL, 0, &val) < 0 ||
            reg >= regs || val > 0xff) {
            error_setg(errp, "xm-sensor: bad regs entry '%s' (registers "
                       "0-0x%" PRIx64 " with addr-bytes=%u, values 0-0xff)",
                       pairs[i], regs - 1, s->addr_bytes);
            return;
        }
        s->regs[reg] = val;
    }
}

static void xm_sensor_realize(DeviceState *dev, Error **errp)
{
    XmSensorState *s = XM_SENSOR(dev);

    if (s->addr_bytes != 1 && s->addr_bytes != 2) {
        error_setg(errp, "xm-sensor: addr-bytes must be 1 or 2");
        return;
    }
    if (s->log_path && *s->log_path) {
        s->log = fopen(s->log_path, "a");
        if (!s->log) {
            error_setg_errno(errp, errno, "xm-sensor: %s", s->log_path);
            return;
        }
    }
    xm_sensor_preset(s, errp);
}

static void xm_sensor_reset(DeviceState *dev)
{
    XmSensorState *s = XM_SENSOR(dev);

    s->ptr = 0;
    s->cur = 0;
    xm_sensor_preset(s, &error_abort);
}

static const Property xm_sensor_props[] = {
    DEFINE_PROP_UINT8("addr-bytes", XmSensorState, addr_bytes, 1),
    DEFINE_PROP_STRING("regs", XmSensorState, regs_preset),
    DEFINE_PROP_STRING("log", XmSensorState, log_path),
};

static const VMStateDescription vmstate_xm_sensor = {
    .name = TYPE_XM_SENSOR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, XmSensorState),
        VMSTATE_UINT8(ptr, XmSensorState),
        VMSTATE_UINT16(cur, XmSensorState),
        VMSTATE_UINT8_ARRAY(regs, XmSensorState, XM_SENSOR_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void xm_sensor_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = xm_sensor_realize;
    device_class_set_legacy_reset(dc, xm_sensor_reset);
    dc->vmsd = &vmstate_xm_sensor;
    device_class_set_props(dc, xm_sensor_props);
    k->event = xm_sensor_event;
    k->recv = xm_sensor_recv;
    k->send = xm_sensor_send;
}

static const TypeInfo xm_sensor_info = {
    .name          = TYPE_XM_SENSOR,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(XmSensorState),
    .class_init    = xm_sensor_class_init,
};

static void xm_sensor_register_types(void)
{
    type_register_static(&xm_sensor_info);
}

type_init(xm_sensor_register_types)
