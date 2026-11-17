/*
 * Axera camera pipeline: pictures passed between the device models,
 * see include/hw/misc/axera-fbuf.h
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/axera-fbuf.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qemu/thread.h"

/* Compressed buffers in flight: two ISP outputs and a few encoder inputs */
#define FBUF_SLOTS 16

static struct {
    hwaddr y;
    AxeraPicture pic;
    uint64_t used;
} fbuf[FBUF_SLOTS];
static uint64_t fbuf_clock;
static QemuMutex fbuf_lock;

static void __attribute__((constructor)) fbuf_init(void)
{
    qemu_mutex_init(&fbuf_lock);
}

static int fbuf_find(hwaddr y)
{
    int i;

    for (i = 0; i < FBUF_SLOTS; i++) {
        if (fbuf[i].y == y && fbuf[i].pic.y) {
            return i;
        }
    }
    return -1;
}

void axera_picture_alloc(AxeraPicture *p, uint32_t w, uint32_t h)
{
    w = ROUND_UP(w, 2);
    h = ROUND_UP(h, 2);
    if (p->y && p->w == w && p->h == h) {
        return;
    }
    axera_picture_free(p);
    p->w = w;
    p->h = h;
    p->y = g_malloc((size_t)w * h);
    p->uv = g_malloc((size_t)w * h / 2);
}

void axera_picture_free(AxeraPicture *p)
{
    g_free(p->y);
    g_free(p->uv);
    p->y = p->uv = NULL;
    p->w = p->h = 0;
}

void axera_fbuf_put(hwaddr y, const AxeraPicture *pic)
{
    int i, slot = 0;

    QEMU_LOCK_GUARD(&fbuf_lock);
    for (i = 0; i < FBUF_SLOTS; i++) {
        if (fbuf[i].y == y && fbuf[i].pic.y) {
            slot = i;
            break;
        }
        if (fbuf[i].used < fbuf[slot].used) {
            slot = i;
        }
    }
    fbuf[slot].y = y;
    fbuf[slot].used = ++fbuf_clock;
    axera_picture_alloc(&fbuf[slot].pic, pic->w, pic->h);
    memcpy(fbuf[slot].pic.y, pic->y, (size_t)pic->w * pic->h);
    memcpy(fbuf[slot].pic.uv, pic->uv, (size_t)pic->w * pic->h / 2);
}

bool axera_fbuf_has(hwaddr y)
{
    QEMU_LOCK_GUARD(&fbuf_lock);
    return fbuf_find(y) >= 0;
}

bool axera_fbuf_copy(hwaddr y, AxeraPicture *pic)
{
    const AxeraPicture *kept;
    int i;

    QEMU_LOCK_GUARD(&fbuf_lock);
    i = fbuf_find(y);
    if (i < 0) {
        return false;
    }
    fbuf[i].used = ++fbuf_clock;
    kept = &fbuf[i].pic;
    if (!pic->y) {
        axera_picture_alloc(pic, kept->w, kept->h);
    }
    axera_picture_scale(kept, pic);
    return true;
}

bool axera_fbuf_edit(hwaddr y, void (*fn)(AxeraPicture *, void *),
                     void *opaque)
{
    int i;

    QEMU_LOCK_GUARD(&fbuf_lock);
    i = fbuf_find(y);
    if (i < 0) {
        return false;
    }
    fbuf[i].used = ++fbuf_clock;
    fn(&fbuf[i].pic, opaque);
    return true;
}

void axera_fbuf_drop(hwaddr y)
{
    int i;

    QEMU_LOCK_GUARD(&fbuf_lock);
    for (i = 0; i < FBUF_SLOTS; i++) {
        if (fbuf[i].y == y) {
            axera_picture_free(&fbuf[i].pic);
            fbuf[i].y = 0;
            fbuf[i].used = 0;
        }
    }
}

static void rows_rw(hwaddr base, uint32_t stride, uint8_t *p, uint32_t w,
                    uint32_t r0, uint32_t r1, bool write)
{
    DMADirection dir = write ? DMA_DIRECTION_FROM_DEVICE
                             : DMA_DIRECTION_TO_DEVICE;

    if (stride == w) {
        dma_memory_rw(&address_space_memory, base + (hwaddr)r0 * stride,
                      p + (size_t)r0 * w, (size_t)(r1 - r0) * w, dir,
                      MEMTXATTRS_UNSPECIFIED);
        return;
    }
    for (uint32_t r = r0; r < r1; r++) {
        dma_memory_rw(&address_space_memory, base + (hwaddr)r * stride,
                      p + (size_t)r * w, w, dir, MEMTXATTRS_UNSPECIFIED);
    }
}

bool axera_nv12_load(const AxeraNv12Buf *buf, AxeraPicture *pic)
{
    if (buf->compressed) {
        axera_picture_alloc(pic, buf->w, buf->h);
        return axera_fbuf_copy(buf->y, pic);
    }
    axera_picture_alloc(pic, buf->w, buf->h);
    rows_rw(buf->y, buf->y_stride ? buf->y_stride : pic->w, pic->y, pic->w,
            0, pic->h, false);
    rows_rw(buf->uv, buf->uv_stride ? buf->uv_stride : pic->w, pic->uv,
            pic->w, 0, pic->h / 2, false);
    return true;
}

void axera_nv12_store_rows(const AxeraNv12Buf *buf, const AxeraPicture *pic,
                           uint32_t r0, uint32_t r1)
{
    uint32_t w = MIN(pic->w, buf->w);

    if (buf->compressed) {
        axera_fbuf_put(buf->y, pic);
        return;
    }
    r1 = MIN(r1, MIN(pic->h, buf->h));
    r0 &= ~1u;
    if (r0 >= r1) {
        return;
    }
    if (w == pic->w) {
        rows_rw(buf->y, buf->y_stride ? buf->y_stride : w, pic->y, w,
                r0, r1, true);
        rows_rw(buf->uv, buf->uv_stride ? buf->uv_stride : w, pic->uv, w,
                r0 / 2, DIV_ROUND_UP(r1, 2), true);
        return;
    }
    for (uint32_t r = r0; r < r1; r++) {
        dma_memory_write(&address_space_memory,
                         buf->y + (hwaddr)r * buf->y_stride,
                         pic->y + (size_t)r * pic->w, w,
                         MEMTXATTRS_UNSPECIFIED);
        if (!(r & 1)) {
            dma_memory_write(&address_space_memory,
                             buf->uv + (hwaddr)(r / 2) * buf->uv_stride,
                             pic->uv + (size_t)(r / 2) * pic->w, w,
                             MEMTXATTRS_UNSPECIFIED);
        }
    }
}

void axera_nv12_store(const AxeraNv12Buf *buf, const AxeraPicture *pic)
{
    axera_nv12_store_rows(buf, pic, 0, pic->h);
}

/*
 * One plane, bilinear in 16.16 fixed point: @sw x @sh samples of @bpp
 * interleaved components (1 luma, 2 for Cb/Cr) to @dw x @dh.
 */
static void scale_plane(const uint8_t *s, uint32_t sw, uint32_t sh,
                        uint8_t *d, uint32_t dw, uint32_t dh, int bpp)
{
    g_autofree uint32_t *xi = g_new(uint32_t, dw);
    g_autofree uint16_t *xf = g_new(uint16_t, dw);
    uint64_t sx = ((uint64_t)sw << 16) / dw, sy = ((uint64_t)sh << 16) / dh;
    uint32_t x, y;
    int c;

    for (x = 0; x < dw; x++) {
        int64_t fx = (int64_t)x * sx + sx / 2 - 0x8000;

        fx = MAX(fx, 0);
        xi[x] = MIN((uint32_t)(fx >> 16), sw - 1);
        xf[x] = xi[x] + 1 < sw ? (fx & 0xffff) >> 8 : 0;
    }
    for (y = 0; y < dh; y++) {
        int64_t fy = (int64_t)y * sy + sy / 2 - 0x8000;
        uint32_t y0, y1, wy;
        const uint8_t *r0, *r1;
        uint8_t *o = d + (size_t)y * dw * bpp;

        fy = MAX(fy, 0);
        y0 = MIN((uint32_t)(fy >> 16), sh - 1);
        y1 = MIN(y0 + 1, sh - 1);
        wy = (fy & 0xffff) >> 8;
        r0 = s + (size_t)y0 * sw * bpp;
        r1 = s + (size_t)y1 * sw * bpp;
        for (x = 0; x < dw; x++) {
            uint32_t i = xi[x] * bpp, j = xf[x] ? i + bpp : i, wx = xf[x];

            for (c = 0; c < bpp; c++) {
                uint32_t a = r0[i + c] * (256 - wx) + r0[j + c] * wx;
                uint32_t b = r1[i + c] * (256 - wx) + r1[j + c] * wx;

                o[x * bpp + c] = (a * (256 - wy) + b * wy + 32768) >> 16;
            }
        }
    }
}

void axera_picture_scale(const AxeraPicture *src, AxeraPicture *dst)
{
    if (src->w == dst->w && src->h == dst->h) {
        memcpy(dst->y, src->y, (size_t)src->w * src->h);
        memcpy(dst->uv, src->uv, (size_t)src->w * src->h / 2);
        return;
    }
    scale_plane(src->y, src->w, src->h, dst->y, dst->w, dst->h, 1);
    scale_plane(src->uv, src->w / 2, src->h / 2, dst->uv, dst->w / 2,
                dst->h / 2, 2);
}
