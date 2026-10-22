/*
 * Novatek NA51089 clock generator (hw/misc/nvt-cg.c): rates other blocks
 * take from it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_NVT_CG_H
#define HW_MISC_NVT_CG_H

/*
 * The audio sample rate the clock tree gives the DAI, which runs at 256
 * times it, snapped to the nearest standard rate.
 */
unsigned nvt_cg_audio_rate(DeviceState *cg);

#endif
