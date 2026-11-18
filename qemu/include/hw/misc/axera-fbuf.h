/*
 * Axera camera pipeline: pictures passed between the device models
 *
 * The ISP (axera-vin.c), the IVPS engines (axera-ivps-engine.c) and the
 * encoder (axera-vcmd.c) hand NV12 pictures to each other through guest
 * memory, as the hardware does. Some buffers on AX620E are compressed
 * (the ISP's main output and VPP's main encoder port, written in a
 * proprietary lossy frame-buffer format at half the size of NV12), and
 * no model can write plain NV12 into those without overrunning the
 * buffer. For those, the writer keeps the picture here, keyed by the
 * buffer's guest luma address, and the reader looks it up.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_AXERA_FBUF_H
#define HW_MISC_AXERA_FBUF_H

#include "exec/hwaddr.h"

/* An NV12 picture in host memory: w x h luma, then w x h/2 Cb/Cr pairs */
typedef struct AxeraPicture {
    uint32_t w, h;
    uint8_t *y;
    uint8_t *uv;
} AxeraPicture;

/* An NV12 picture in guest memory */
typedef struct AxeraNv12Buf {
    hwaddr y, uv;
    uint32_t w, h;
    uint32_t y_stride, uv_stride;
    bool compressed;    /* AX620E frame-buffer compression: keep it here */
} AxeraNv12Buf;

void axera_picture_alloc(AxeraPicture *p, uint32_t w, uint32_t h);
void axera_picture_free(AxeraPicture *p);

/* Keep a copy of @pic for the compressed buffer at @y (replacing any) */
void axera_fbuf_put(hwaddr y, const AxeraPicture *pic);
/* Whether a picture is kept for the buffer at @y */
bool axera_fbuf_has(hwaddr y);
/*
 * Copy the picture kept for the buffer at @y into @pic, scaled to
 * @pic's size if @pic is allocated, else at its own size. False if none
 * is kept. Safe from any thread (the encoder works outside the BQL).
 */
bool axera_fbuf_copy(hwaddr y, AxeraPicture *pic);
/*
 * Run @fn on the picture kept for the buffer at @y, in place and under
 * the lock (an overlay drawn into a compressed frame). False if none.
 */
bool axera_fbuf_edit(hwaddr y, void (*fn)(AxeraPicture *, void *),
                     void *opaque);
/* Forget the buffer at @y: something wrote plain pixels there */
void axera_fbuf_drop(hwaddr y);

/*
 * Read the picture in @buf into @pic (allocated to @buf's size): from
 * the kept copy for a compressed buffer, else from guest memory.
 * Returns false if a compressed buffer has no kept picture.
 */
bool axera_nv12_load(const AxeraNv12Buf *buf, AxeraPicture *pic);
/* Write @pic, which must be @buf's size, to @buf */
void axera_nv12_store(const AxeraNv12Buf *buf, const AxeraPicture *pic);
/* Write rows [r0, r1) of @pic's luma (and their chroma) to @buf */
void axera_nv12_store_rows(const AxeraNv12Buf *buf, const AxeraPicture *pic,
                           uint32_t r0, uint32_t r1);

/* Scale @src to @dst's size: bilinear, as the hardware scalers do */
void axera_picture_scale(const AxeraPicture *src, AxeraPicture *dst);

#endif /* HW_MISC_AXERA_FBUF_H */
