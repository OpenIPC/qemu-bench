/*
 * When an image sensor's register writes take effect, and what a driver
 * did wrong getting there (docs/hisilicon-digital-twin.md, layer 2).
 *
 * A sensor applies its exposure, gain and frame length at the start of a
 * frame, all together, and a group hold keeps writes back until it is
 * released.  A model embeds a SensorSync and keeps two copies of those
 * registers: what the driver wrote and what the frame in progress uses.
 * Latching starts only when an ISP model drives the sensor's frames
 * (image_sensor_frame_start()); until then writes apply at once, as
 * before, for every machine whose ISP has no frame clock.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_I2C_SENSOR_SYNC_H
#define HW_I2C_SENSOR_SYNC_H

typedef struct SensorSync {
    const char *name;           /* the model, for the log */
    bool latching;              /* a frame clock drives the sensor */
    bool hold;                  /* a group hold is open: nothing latches */
    uint32_t frame;             /* frames started */
    int64_t t0;                 /* the frame's start, virtual ns */
    int64_t frame_ns;           /* its length, 0 when not known */
    uint32_t vts;               /* its rows, for the log's line */
    uint32_t violations;        /* datasheet rules broken */
    bool trace;                 /* log every write with frame and line */
} SensorSync;

/*
 * A frame starts: true when the latched registers should be copied from
 * what the driver wrote to what the frame uses (no group hold open; a
 * write made before the first frame applied at once anyway).
 */
bool sensor_sync_frame_start(SensorSync *s, int64_t frame_ns, uint32_t vts);

/* The driver wrote @val to @reg (the log, with frame and line) */
void sensor_sync_write(SensorSync *s, uint16_t reg, uint8_t val);

/* Group hold: opened / closed by the driver */
void sensor_sync_hold(SensorSync *s, bool on);

/* The sensor's reset: no frames, no hold, no violations; the log stays */
void sensor_sync_reset(SensorSync *s);

/*
 * A datasheet rule broken: counted (the "violations" property) and
 * reported with -d guest_errors, with the frame it happened in.
 */
void G_GNUC_PRINTF(2, 3) sensor_sync_violation(SensorSync *s,
                                               const char *fmt, ...);

/* "frame", "violations" (read-only) and "trace-i2c" on the sensor */
void sensor_sync_add_properties(Object *obj, SensorSync *s);

#endif
