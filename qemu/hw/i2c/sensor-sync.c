/*
 * When an image sensor's register writes take effect, and what a driver
 * did wrong getting there.  See include/hw/i2c/sensor-sync.h.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/visitor.h"
#include "qom/object.h"
#include "hw/i2c/sensor-sync.h"

bool sensor_sync_frame_start(SensorSync *s, int64_t frame_ns, uint32_t vts)
{
    s->latching = true;
    s->frame++;
    s->t0 = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->frame_ns = frame_ns;
    s->vts = vts;
    return !s->hold;            /* an open group is still being written */
}

void sensor_sync_reset(SensorSync *s)
{
    s->latching = false;
    s->hold = false;
    s->frame = 0;
    s->t0 = 0;
    s->frame_ns = 0;
    s->vts = 0;
    s->violations = 0;
}

/* where in the frame the sensor is: its line, or microseconds when the
 * frame's timing is not modelled */
static void sensor_sync_where(SensorSync *s, char *buf, size_t len)
{
    const int64_t t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->t0;

    if (!s->latching) {
        snprintf(buf, len, "before streaming");
    } else if (s->frame_ns > 0 && s->vts) {
        snprintf(buf, len, "frame %u line %" PRId64, s->frame,
                 t * s->vts / s->frame_ns);
    } else {
        snprintf(buf, len, "frame %u +%" PRId64 "us", s->frame, t / 1000);
    }
}

void sensor_sync_write(SensorSync *s, uint16_t reg, uint8_t val)
{
    char where[48];

    if (s->trace) {
        sensor_sync_where(s, where, sizeof(where));
        fprintf(stderr, "%s: %s: %04x = %02x%s\n", s->name, where, reg, val,
                s->hold ? " (held)" : "");
    }
}

void sensor_sync_hold(SensorSync *s, bool on)
{
    s->hold = on;
}

void sensor_sync_violation(SensorSync *s, const char *fmt, ...)
{
    g_autofree char *msg = NULL;
    char where[48];
    va_list ap;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);
    s->violations++;
    sensor_sync_where(s, where, sizeof(where));
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s: %s\n", s->name, where, msg);
    if (s->trace) {
        fprintf(stderr, "%s: %s: VIOLATION %s\n", s->name, where, msg);
    }
}

static void sync_get_u32(Object *obj, Visitor *v, const char *name,
                         void *opaque, Error **errp)
{
    visit_type_uint32(v, name, opaque, errp);
}

static void sync_get_bool(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    visit_type_bool(v, name, opaque, errp);
}

static void sync_set_bool(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    bool value;

    if (visit_type_bool(v, name, &value, errp)) {
        *(bool *)opaque = value;
    }
}

void sensor_sync_add_properties(Object *obj, SensorSync *s)
{
    object_property_add(obj, "frame", "uint32", sync_get_u32, NULL, NULL,
                        &s->frame);
    object_property_add(obj, "violations", "uint32", sync_get_u32, NULL,
                        NULL, &s->violations);
    object_property_add(obj, "trace-i2c", "bool", sync_get_bool,
                        sync_set_bool, NULL, &s->trace);
}
