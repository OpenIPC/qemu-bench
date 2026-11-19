/*
 * Fullhan FH8626V100 JPEG encoder at 0xE8300000, as OpenIPC's 4.19
 * drivers/media/platform/fh8626/fh8626-jpeg.c drives it (registers in its
 * fh8626-jpeg.h; there is no SDK or manual for the part).
 *
 *   0x00 HW_ID          0x00038401, which the driver checks at probe and
 *                       after every frame
 *   0x04 INT_STATUS     bit 0 done; write one to clear (IRQ 8)
 *   0x08 MODE           3, read back after every frame
 *   0x0C STREAM_SIZE    the bytes written for the last picture
 *   0x10 CTRL           0x80000001 starts a picture
 *   0x14 / 0x1C         [31:16] the coded height / width, MCU-aligned
 *   0x34 / 0x38         the bitstream's start / write pointer
 *   0x3C / 0x40         the NV12 input's luma / interleaved chroma
 *   0x50 BS_LIMIT       the room for it
 *   0x58 ENC_CTRL       [31:16] the input row width (its stride), [9:8]
 *                       the rotation, in 90 degree steps clockwise
 *   0x80 QTABLE         luma then chroma quantisers, zigzag order
 *   0x100 / 0x400       the Huffman code and DHT tables (Annex K's)
 *
 * A picture is a whole JFIF file, the way the driver expects it: SOI, the
 * quantisers from 0x80, SOF0 at the coded size -- the driver patches the
 * displayed size in -- the Annex K Huffman tables, the scan and EOI.  The
 * input is read rotated as ENC_CTRL says, so the coded size is the input's
 * turned.  The scan is the shared baseline encoder's (jpeg-enc.c); the
 * engine's burst, slice and chip configuration registers are storage.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "jpeg-enc.h"

#define TYPE_FH_JPEG "fh-jpeg"
OBJECT_DECLARE_SIMPLE_TYPE(FhJpegState, FH_JPEG)

#define FH_JPEG_SIZE        0x600
#define FH_JPEG_HW_ID       0x00038401

#define R_HW_ID             0x00
#define R_INT_STATUS        0x04
#define R_STREAM_SIZE       0x0C
#define R_CTRL              0x10
#define R_HEIGHT_CFG        0x14
#define R_WIDTH_CFG         0x1C
#define R_BS_START          0x34
#define R_BS_CURR           0x38
#define R_Y_ADDR            0x3C
#define R_UV_ADDR           0x40
#define R_BS_LIMIT          0x50
#define R_ENC_CTRL          0x58
#define R_QTABLE            0x80

#define CTRL_START          0x80000001
#define INT_DONE            1
#define MAX_DIM             4096
/* a picture takes this long to come out */
#define ENCODE_NS           (2 * SCALE_MS)

static const uint8_t fh_jpeg_zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

struct FhJpegState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[FH_JPEG_SIZE / 4];
    uint32_t status;
};

static uint32_t fh_jpeg_r(FhJpegState *s, hwaddr off)
{
    return s->regs[off >> 2];
}

static void fh_jpeg_update_irq(FhJpegState *s)
{
    qemu_set_irq(s->irq, !!s->status);
}

/*
 * The input, an iw x ih NV12 picture at stride st, turned rot x 90 degrees
 * clockwise into planar w x h luma and w/2 x h/2 Cb and Cr.
 */
static void fh_jpeg_rotate(const uint8_t *y, const uint8_t *uv, int st,
                           int iw, int ih, int rot, uint8_t *oy,
                           uint8_t *ocb, uint8_t *ocr, int w, int h)
{
    int x, r;

    for (r = 0; r < h; r++) {
        for (x = 0; x < w; x++) {
            int sx = x, sy = r;

            switch (rot) {
            case 1:
                sx = r;
                sy = ih - 1 - x;
                break;
            case 2:
                sx = iw - 1 - x;
                sy = ih - 1 - r;
                break;
            case 3:
                sx = iw - 1 - r;
                sy = x;
                break;
            }
            oy[r * w + x] = y[sy * st + sx];
        }
    }
    for (r = 0; r < h / 2; r++) {
        for (x = 0; x < w / 2; x++) {
            int sx = x, sy = r;

            switch (rot) {
            case 1:
                sx = r;
                sy = ih / 2 - 1 - x;
                break;
            case 2:
                sx = iw / 2 - 1 - x;
                sy = ih / 2 - 1 - r;
                break;
            case 3:
                sx = iw / 2 - 1 - r;
                sy = x;
                break;
            }
            ocb[r * (w / 2) + x] = uv[sy * st + 2 * sx];
            ocr[r * (w / 2) + x] = uv[sy * st + 2 * sx + 1];
        }
    }
}

static void fh_jpeg_encode(FhJpegState *s)
{
    int w = fh_jpeg_r(s, R_WIDTH_CFG) >> 16;
    int h = fh_jpeg_r(s, R_HEIGHT_CFG) >> 16;
    int st = fh_jpeg_r(s, R_ENC_CTRL) >> 16;      /* the input's stride */
    int rot = (fh_jpeg_r(s, R_ENC_CTRL) >> 8) & 3;
    int iw = (rot & 1) ? h : w, ih = (rot & 1) ? w : h;
    /*
     * The room the guest gives, but no more than a baseline 4:2:0 picture
     * can take (well under 3 bytes a pixel) whatever BS_LIMIT says.
     */
    uint32_t cap = MIN(fh_jpeg_r(s, R_BS_LIMIT), (uint32_t)w * h * 3 + 4096);
    uint32_t dst = fh_jpeg_r(s, R_BS_START);
    g_autofree uint8_t *in = NULL, *py = NULL, *pc = NULL, *out = NULL;
    g_autoptr(GByteArray) hdr = g_byte_array_new();
    uint8_t ql[64], qc[64], q[128];
    size_t n, len;
    bool overflow = false;
    int i;

    s->regs[R_STREAM_SIZE >> 2] = 0;
    if (w < 16 || h < 16 || w > MAX_DIM || h > MAX_DIM || (w & 15) ||
        (h & 15) || st < iw || st > MAX_DIM) {
        qemu_log_mask(LOG_GUEST_ERROR, "fh-jpeg: %dx%d from rows of %d, "
                      "rotation %d\n", w, h, st, rot);
        return;
    }
    for (i = 0; i < 32; i++) {
        stl_le_p(q + 4 * i, s->regs[(R_QTABLE >> 2) + i]);
    }
    for (i = 0; i < 64; i++) {
        ql[fh_jpeg_zigzag[i]] = q[i] ? q[i] : 1;
        qc[fh_jpeg_zigzag[i]] = q[64 + i] ? q[64 + i] : 1;
    }
    in = g_malloc((size_t)st * ih * 3 / 2);
    dma_memory_read(&address_space_memory, fh_jpeg_r(s, R_Y_ADDR), in,
                    (size_t)st * ih, MEMTXATTRS_UNSPECIFIED);
    dma_memory_read(&address_space_memory, fh_jpeg_r(s, R_UV_ADDR),
                    in + (size_t)st * ih, (size_t)st * ih / 2,
                    MEMTXATTRS_UNSPECIFIED);
    py = g_malloc((size_t)w * h);
    pc = g_malloc((size_t)w * h / 2);
    fh_jpeg_rotate(in, in + (size_t)st * ih, st, iw, ih, rot, py, pc,
                   pc + (size_t)w * h / 4, w, h);

    jpeg_enc_header(hdr, w, h, ql, qc);
    if (hdr->len > cap) {
        qemu_log_mask(LOG_GUEST_ERROR, "fh-jpeg: no room for the header\n");
        return;
    }
    out = g_malloc(cap);
    memcpy(out, hdr->data, hdr->len);
    n = jpeg_enc_scan(py, w, pc, pc + (size_t)w * h / 4, w / 2, w, h, ql, qc,
                      true, out + hdr->len, cap - hdr->len, &overflow);
    len = hdr->len + n;
    if (overflow) {
        /*
         * As far as it fits, with no EOI: the driver looks for one at the
         * end and fails the picture without it ("does not fit").  The
         * scan is byte-stuffed, so FF D9 can only be the real marker.
         */
        qemu_log_mask(LOG_GUEST_ERROR, "fh-jpeg: %u bytes are not enough\n",
                      cap);
        len = cap;
    }
    dma_memory_write(&address_space_memory, dst, out, len,
                     MEMTXATTRS_UNSPECIFIED);
    s->regs[R_STREAM_SIZE >> 2] = len;
    s->regs[R_BS_CURR >> 2] = dst + len;
}

static void fh_jpeg_done(void *opaque)
{
    FhJpegState *s = opaque;

    s->status |= INT_DONE;
    fh_jpeg_update_irq(s);
}

static uint64_t fh_jpeg_read(void *opaque, hwaddr off, unsigned size)
{
    FhJpegState *s = opaque;

    switch (off) {
    case R_HW_ID:
        return FH_JPEG_HW_ID;
    case R_INT_STATUS:
        return s->status;
    }
    return s->regs[off >> 2];
}

static void fh_jpeg_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    FhJpegState *s = opaque;

    switch (off) {
    case R_HW_ID:
        return;
    case R_INT_STATUS:
        s->status &= ~val;
        fh_jpeg_update_irq(s);
        return;
    }
    s->regs[off >> 2] = val;
    if (off == R_CTRL && val == CTRL_START) {
        fh_jpeg_encode(s);
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  ENCODE_NS);
    }
}

static const MemoryRegionOps fh_jpeg_ops = {
    .read = fh_jpeg_read,
    .write = fh_jpeg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_jpeg_reset(DeviceState *dev)
{
    FhJpegState *s = FH_JPEG(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->status = 0;
    timer_del(s->timer);
    fh_jpeg_update_irq(s);
}

static void fh_jpeg_init(Object *obj)
{
    FhJpegState *s = FH_JPEG(obj);

    memory_region_init_io(&s->iomem, obj, &fh_jpeg_ops, s, TYPE_FH_JPEG,
                          FH_JPEG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fh_jpeg_done, s);
}

static int fh_jpeg_post_load(void *opaque, int version_id)
{
    fh_jpeg_update_irq(opaque);
    return 0;
}

static const VMStateDescription vmstate_fh_jpeg = {
    .name = TYPE_FH_JPEG,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = fh_jpeg_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, FhJpegState, FH_JPEG_SIZE / 4),
        VMSTATE_UINT32(status, FhJpegState),
        VMSTATE_TIMER_PTR(timer, FhJpegState),
        VMSTATE_END_OF_LIST()
    },
};

static void fh_jpeg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, fh_jpeg_reset);
    dc->vmsd = &vmstate_fh_jpeg;
}

static const TypeInfo fh_jpeg_info = {
    .name          = TYPE_FH_JPEG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FhJpegState),
    .instance_init = fh_jpeg_init,
    .class_init    = fh_jpeg_class_init,
};

static void fh_jpeg_register_types(void)
{
    type_register_static(&fh_jpeg_info);
}

type_init(fh_jpeg_register_types)
