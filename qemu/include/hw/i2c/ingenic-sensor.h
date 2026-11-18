/*
 * What the Ingenic ISP models (hw/misc/ingenic-isp*.c) read of the image
 * sensor models on its I2C bus: the exposure the kernel's AE last
 * programmed, decoded the way each part's tx-isp sensor driver
 * (sensor_<name>_<soc>.ko, SDK opensource/drivers/sensors-*) writes it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_I2C_INGENIC_SENSOR_H
#define HW_I2C_INGENIC_SENSOR_H

#include "qom/object.h"

typedef struct IngenicSensorExposure {
    uint32_t lines;         /* integration time, rows */
    uint32_t gain;          /* total gain, 1.0 = 1024 */
    uint32_t short_lines;   /* a DOL WDR mode's short exposure, rows; 0: none */
} IngenicSensorExposure;

/* hw/i2c/galaxycore.c: GC2053, GC2083, GC4653 / GC4663 */
bool galaxycore_exposure(Object *sensor, IngenicSensorExposure *e);
/* hw/i2c/hisi-smartsens.c: SC2336, SC2335 */
bool smartsens_exposure(Object *sensor, IngenicSensorExposure *e);
/* hw/i2c/ingenic-ov2735.c: OV2735, OV2735B */
bool ov2735_exposure(Object *sensor, IngenicSensorExposure *e);

#endif /* HW_I2C_INGENIC_SENSOR_H */
