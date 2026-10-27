/*
 * SOI (JX-*) image sensors (hw/i2c/hisi-f37.c): what an ISP model needs
 * to know of the picture the sensor would put out.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_I2C_HISI_SOI_H
#define HW_I2C_HISI_SOI_H

#include "qom/object.h"

#define TYPE_HISI_F37 "hisi-f37"
#define TYPE_HISI_JXF22 "hisi-jxf22"

typedef struct HisiSoiExposure {
    uint32_t lines;         /* exposure, rows: 0x02 high, 0x01 low */
    uint32_t vts;           /* frame length, rows: 0x23 high, 0x22 low */
    uint32_t gain;          /* analog gain, 1.0 = 1024: 0x00 */
    uint32_t width;         /* output window: 0x24-0x26; 0 if unset */
    uint32_t height;
} HisiSoiExposure;

/* Fill @e from @sensor's registers; false if it is no SOI sensor. */
bool hisi_soi_exposure(Object *sensor, HisiSoiExposure *e);

#endif /* HW_I2C_HISI_SOI_H */
