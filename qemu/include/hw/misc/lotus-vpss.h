/*
 * XMedia "lotus" SPC030 VPSS (Goke GK7206): what other media models may
 * ask it.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_LOTUS_VPSS_H
#define HW_MISC_LOTUS_VPSS_H

/*
 * The picture VPSS last wrote to its output channel in wrap (loop buffer,
 * low delay) mode, if y_addr (and c_addr, when not 0) are that channel's
 * buffers.  Real hardware streams such a picture to the encoder a few
 * lines at a time through the loop buffer, so DDR never holds all of it;
 * a reader that takes the frame whole (the encoder model) gets it here:
 * w x h luma, w x h / 2 interleaved chroma, NV12 order if *nv12, else
 * NV21.  Owned by VPSS, valid until its next frame.
 */
bool lotus_vpss_picture(uint32_t y_addr, uint32_t c_addr, int *w, int *h,
                        const uint8_t **y, const uint8_t **c, bool *nv12);

#endif
