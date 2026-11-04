/*
 * Image sensors, whatever the SoC: the catalogue machines attach them from
 * by name, and the interface an ISP model reads the exposure through.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_I2C_IMAGE_SENSOR_H
#define HW_I2C_IMAGE_SENSOR_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_IMAGE_SENSOR "image-sensor"

typedef struct ImageSensorExposure {
    uint32_t lines;         /* integration time, rows */
    uint32_t vts;           /* frame length, rows; 0 if unset */
    uint32_t gain;          /* total gain, 1.0 = 1024 */
    uint32_t width;         /* output window; 0 if unset */
    uint32_t height;
} ImageSensorExposure;

typedef struct ImageSensorClass {
    InterfaceClass parent_class;

    /* Fill @e from the sensor's registers */
    void (*exposure)(Object *sensor, ImageSensorExposure *e);
} ImageSensorClass;

#define IMAGE_SENSOR_CLASS(klass) \
    OBJECT_CLASS_CHECK(ImageSensorClass, (klass), TYPE_IMAGE_SENSOR)
#define IMAGE_SENSOR_GET_CLASS(obj) \
    OBJECT_GET_CLASS(ImageSensorClass, (obj), TYPE_IMAGE_SENSOR)

/* Fill @e from @sensor; false if @sensor implements no image-sensor. */
bool image_sensor_exposure(Object *sensor, ImageSensorExposure *e);

/* Whether the catalogue has a sensor called @name. */
bool image_sensor_known(const char *name);

/*
 * The sensor @name from the catalogue on @bus at its usual address, or at
 * @addr if that is not 0, realized.  An unknown name lists the catalogue
 * and exits.  NULL for "none" or no name: the board has no sensor.
 */
DeviceState *image_sensor_create_at(I2CBus *bus, const char *name,
                                    uint8_t addr);
DeviceState *image_sensor_create(I2CBus *bus, const char *name);

#endif /* HW_I2C_IMAGE_SENSOR_H */
