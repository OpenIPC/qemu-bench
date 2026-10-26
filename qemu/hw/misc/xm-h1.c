/*
 * Hantro H1 video encoder, as XM530 carries it (0x20070000, SPI 45).
 *
 * The vendor driver is xm530_h264.ko, Hantro's own hx280enc: at load it reads
 * swreg0 and wants a product ID of 0x6280/0x7280/0x8270/0x8290, and Hantro's
 * EWL in the SDK reads the synthesis configuration (swreg63) before any
 * encoder initialises.
 *
 * Encoding follows the register contract of the open-source H1 driver in
 * mainline Linux (drivers/media/platform/verisilicon: hantro_h1_regs.h,
 * hantro_h1_jpeg_enc.c, hantro_jpeg.c), not any vendor code:
 *
 *   ADDR_IN_PLANE_0/1/2   input planes;  IN_IMG_CTRL  row length, format
 *                         (0 YUV420 planar, 1 YUV420 semi-planar)
 *   ADDR_OUTPUT_STREAM    where the entropy-coded scan goes (the driver
 *                         writes the JFIF header in front of it itself)
 *   STR_BUF_LIMIT         bytes available; on completion, bits written
 *   JPEG_LUMA/CHROMA_QUAT the quantisers, packed big-endian four to a word
 *                         in the driver's hw_reorder order
 *   ENC_CTRL              MB width/height, mode, EN: writing EN starts it
 *   INTERRUPT             FRAME_RDY | IRQ on completion, the line raised
 *                         unless DIS
 *
 * JPEG: baseline, 4:2:0, the Annex K Huffman tables hantro_jpeg puts in the
 * header, ending with EOI.  The frame completes before the start write
 * returns.
 *
 * H.264 (mode 3): Hantro's library writes the SPS and PPS itself, up to an
 * 8-byte aligned ADDR_OUTPUT_STREAM, and hands the H1 the bits that did not
 * fit: STR_HDR_REM_MSB/LSB, RLC_CTRL[28:23] of them.  The model emits those
 * first, then a slice from hantro-h264.c fitted to the SPS/PPS it reads
 * back out of the stream buffer.  ENC_CTRL's intra bit makes it an IDR;
 * ENC_CTRL0[31:26] is the QP.
 *
 * dump-dir (-global xm-h1.dump-dir=DIR) keeps every job on the host, to see
 * what a guest's encoder actually produced: a JPEG job as a complete
 * frame-NNNNN.jpg (the model writes the JFIF header the driver would, from
 * the programmed quantisers), an H.264 job as frame-NNNNN.h264, the bytes
 * the H1 wrote; and each job's registers as frame-NNNNN.regs.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "system/dma.h"
#include "hw/qdev-properties.h"
#include "hantro-h264.h"
#include "jpeg-enc.h"

#define TYPE_XM_H1 "xm-h1"
OBJECT_DECLARE_SIMPLE_TYPE(XmH1State, XM_H1)

#define H1_REGS_SIZE    0x1000
#define H1_SWREG0_ID    0x000
/* 8290 is the H1 in the driver's list; low half is the build. */
#define H1_ID           0x82901000u
/* swreg63, the synthesis configuration Hantro's EWL reads before any
 * encoder will initialise (EWLReadAsicConfig): max width (bits 10:0,
 * pixels), and the codecs the silicon has -- VS 24, JPEG 25, VP8 26,
 * H.264 27, RGB 28 -- with an AXI bus (type 1, bits 23:20) 64 bits wide
 * (bits 13:12 = 1). */
#define H1_SWREG63_CFG  0x0FC
#define H1_CFG          ((1u << 28) | (1u << 27) | (1u << 26) | (1u << 25) | \
                         (1u << 24) | (1u << 20) | (1u << 12))
#define H1_CFG_MAXW     0xFFF   /* XM530's reads 1920; max-width sets it */

/* hantro_h1_regs.h */
#define H1_REG_INTERRUPT            0x004
/* 0x08 and 0x20 as Hantro's own encoder names them (encasiccontroller.h):
 * ASIC_STATUS_ERROR and ASIC_STATUS_BUFF_FULL */
#define   H1_INT_BUF_FULL           (1u << 5)
#define   H1_INT_ERROR              (1u << 3)
#define   H1_INT_FRAME_RDY          (1u << 2)
#define   H1_INT_DIS                (1u << 1)
#define   H1_INT_BIT                (1u << 0)
#define H1_REG_ADDR_OUTPUT_STREAM   0x014
#define H1_REG_ADDR_IN_PLANE_0      0x02c
#define H1_REG_ADDR_IN_PLANE_1      0x030
#define H1_REG_ADDR_IN_PLANE_2      0x034
#define H1_REG_ENC_CTRL             0x038
#define   H1_ENC_EN                 (1u << 0)
#define   H1_ENC_MODE(v)            (((v) >> 1) & 3)
#define   H1_ENC_MODE_JPEG          2
#define   H1_ENC_WIDTH_MB(v)        (((v) >> 19) & 0x1ff)
#define   H1_ENC_HEIGHT_MB(v)       (((v) >> 10) & 0x1ff)
#define H1_REG_IN_IMG_CTRL          0x03c
#define   H1_IN_ROW_LEN(v)          (((v) >> 12) & 0x3fff)
#define   H1_IN_FMT(v)              (((v) >> 2) & 0xf)
#define H1_REG_ENC_CTRL0            0x040
#define   H1_CTRL0_INIT_QP(v)       (((v) >> 26) & 0x3f)
#define   H1_ENC_PIC_INTRA          (1u << 3)
#define   H1_ENC_MODE_H264          3
#define H1_REG_STR_HDR_REM_MSB      0x058
#define H1_REG_STR_HDR_REM_LSB      0x05c
#define H1_REG_STR_BUF_LIMIT        0x060
#define H1_REG_RLC_CTRL             0x094
#define   H1_RLC_STR_OFFS(v)        (((v) >> 23) & 0x3f)
#define H1_REG_JPEG_LUMA_QUAT       0x100
#define H1_REG_JPEG_CHROMA_QUAT     0x140

struct XmH1State {
    SysBusDevice parent_obj;
    uint32_t max_width;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[H1_REGS_SIZE / 4];
    char *dump_dir;
    uint32_t jobs;
    H264Enc h264;
};

/* ---- JPEG (jpeg-enc.c) ---------------------------------------------- */

/* hantro_jpeg.c: hw[i] = Q[hw_reorder[i]], Q in natural (row-major) order */
static const uint8_t hw_reorder[64] = {
     0,  8, 16, 24,  1,  9, 17, 25, 32, 40, 48, 56, 33, 41, 49, 57,
     2, 10, 18, 26,  3, 11, 19, 27, 34, 42, 50, 58, 35, 43, 51, 59,
     4, 12, 20, 28,  5, 13, 21, 29, 36, 44, 52, 60, 37, 45, 53, 61,
     6, 14, 22, 30,  7, 15, 23, 31, 38, 46, 54, 62, 39, 47, 55, 63,
};

static void xm_h1_dump(XmH1State *s, const char *ext, const uint8_t *data,
                       size_t len)
{
    g_autofree char *path = g_strdup_printf("%s/frame-%05u.%s", s->dump_dir,
                                            s->jobs, ext);
    g_autoptr(GError) err = NULL;

    if (!g_file_set_contents(path, (const char *)data, len, &err)) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-h1: cannot dump %s: %s\n", path,
                      err->message);
    }
}

static void xm_h1_dump_regs(XmH1State *s)
{
    g_autoptr(GString) out = g_string_new(NULL);
    int i;

    for (i = 0; i < 0x200 / 4; i++) {
        g_string_append_printf(out, "%03x %08x\n", i * 4, s->regs[i]);
    }
    xm_h1_dump(s, "regs", (const uint8_t *)out->str, out->len);
}

/* swreg63 advertises 1920 pixels; the stride must cover the picture so
 * every read stays inside the input copy, and stay a row's worth */
#define H1_MAX_HEIGHT_MB  (4096 / 16)

static bool xm_h1_geometry_ok(XmH1State *s, const char *what, int mbw,
                              int mbh, int stride)
{
    /* as wide as swreg63 says the synthesis is (max-width) */
    if (!mbw || !mbh || mbw > (int)(s->max_width / 16) ||
        mbh > H1_MAX_HEIGHT_MB ||
        stride < mbw * 16 || stride > 4096) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-h1: %s %dx%d MB, stride %d\n",
                      what, mbw, mbh, stride);
        return false;
    }
    return true;
}

/* Host memory for the output: the programmed limit, but never more than a
 * frame of this size can take (6 bytes a pixel covers JPEG at quality 1). */
static uint32_t xm_h1_out_cap(XmH1State *s, int mbw, int mbh)
{
    return MIN(s->regs[H1_REG_STR_BUF_LIMIT / 4],
               (uint32_t)mbw * mbh * 256 * 6 + 4096);
}

static uint32_t xm_h1_jpeg(XmH1State *s)
{
    const uint32_t ctrl = s->regs[H1_REG_ENC_CTRL / 4];
    const uint32_t img = s->regs[H1_REG_IN_IMG_CTRL / 4];
    const int mbw = H1_ENC_WIDTH_MB(ctrl), mbh = H1_ENC_HEIGHT_MB(ctrl);
    const int w = mbw * 16, h = mbh * 16;
    const int stride = H1_IN_ROW_LEN(img) ? H1_IN_ROW_LEN(img) : w;
    const int fmt = H1_IN_FMT(img);
    uint8_t ql[64], qc[64];
    uint8_t *y, *cb, *cr, *out;
    bool overflow;
    int i;

    if (fmt != 0 && fmt != 1) {
        qemu_log_mask(LOG_UNIMP, "xm-h1: JPEG input format %d\n", fmt);
        return H1_INT_ERROR;
    }
    if (!xm_h1_geometry_ok(s, "JPEG", mbw, mbh, stride)) {
        return H1_INT_ERROR;
    }
    for (i = 0; i < 64; i++) {
        const uint32_t lw = s->regs[H1_REG_JPEG_LUMA_QUAT / 4 + i / 4];
        const uint32_t cw = s->regs[H1_REG_JPEG_CHROMA_QUAT / 4 + i / 4];
        ql[hw_reorder[i]] = MAX(1, (lw >> (24 - 8 * (i % 4))) & 0xff);
        qc[hw_reorder[i]] = MAX(1, (cw >> (24 - 8 * (i % 4))) & 0xff);
    }

    y = g_malloc(stride * h);
    cb = g_malloc((stride / 2) * (h / 2));
    cr = g_malloc((stride / 2) * (h / 2));
    dma_memory_read(&address_space_memory, s->regs[H1_REG_ADDR_IN_PLANE_0 / 4],
                    y, stride * h, MEMTXATTRS_UNSPECIFIED);
    if (fmt == 1) {                     /* semi-planar: CbCr interleaved */
        uint8_t *uv = g_malloc(stride * (h / 2));
        dma_memory_read(&address_space_memory,
                        s->regs[H1_REG_ADDR_IN_PLANE_1 / 4], uv,
                        stride * (h / 2), MEMTXATTRS_UNSPECIFIED);
        for (i = 0; i < (stride / 2) * (h / 2); i++) {
            cb[i] = uv[2 * i];
            cr[i] = uv[2 * i + 1];
        }
        g_free(uv);
    } else {
        dma_memory_read(&address_space_memory,
                        s->regs[H1_REG_ADDR_IN_PLANE_1 / 4], cb,
                        (stride / 2) * (h / 2), MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory,
                        s->regs[H1_REG_ADDR_IN_PLANE_2 / 4], cr,
                        (stride / 2) * (h / 2), MEMTXATTRS_UNSPECIFIED);
    }

    const uint32_t cap = xm_h1_out_cap(s, mbw, mbh);
    out = g_malloc(MAX(cap, 1));
    const size_t n = jpeg_enc_scan(y, stride, cb, cr, stride / 2, w, h, ql, qc,
                                   true, out, cap, &overflow);
    if (overflow) {
        /* nothing is written: the driver sees BUFF_FULL, not a short scan */
        qemu_log_mask(LOG_GUEST_ERROR, "xm-h1: JPEG output over the %u-byte "
                      "buffer\n", (unsigned)cap);
    } else {
        dma_memory_write(&address_space_memory,
                         s->regs[H1_REG_ADDR_OUTPUT_STREAM / 4], out, n,
                         MEMTXATTRS_UNSPECIFIED);
        /* hantro_h1_jpeg_enc_done(): bytesused = STR_BUF_LIMIT / 8 */
        s->regs[H1_REG_STR_BUF_LIMIT / 4] = n * 8;
    }
    if (s->dump_dir && !overflow) {
        g_autoptr(GByteArray) f = g_byte_array_new();
        jpeg_enc_header(f, w, h, ql, qc);
        g_byte_array_append(f, out, n);
        xm_h1_dump(s, "jpg", f->data, f->len);
    }
    g_free(out);
    g_free(y);
    g_free(cb);
    g_free(cr);
    return overflow ? H1_INT_BUF_FULL : H1_INT_FRAME_RDY;
}

static uint32_t xm_h1_h264(XmH1State *s)
{
    const uint32_t ctrl = s->regs[H1_REG_ENC_CTRL / 4];
    const uint32_t img = s->regs[H1_REG_IN_IMG_CTRL / 4];
    const int mbw = H1_ENC_WIDTH_MB(ctrl), mbh = H1_ENC_HEIGHT_MB(ctrl);
    const int stride = H1_IN_ROW_LEN(img) ? H1_IN_ROW_LEN(img) : mbw * 16;
    const bool idr = ctrl & H1_ENC_PIC_INTRA;
    const hwaddr out = s->regs[H1_REG_ADDR_OUTPUT_STREAM / 4];
    const uint32_t cap = xm_h1_out_cap(s, mbw, mbh);
    const int rem_bits = H1_RLC_STR_OFFS(s->regs[H1_REG_RLC_CTRL / 4]);
    const uint64_t rem = ((uint64_t)s->regs[H1_REG_STR_HDR_REM_MSB / 4] << 32) |
                         s->regs[H1_REG_STR_HDR_REM_LSB / 4];
    uint8_t rem_bytes[8];
    uint8_t hdr[512 + 8];
    size_t hdr_at = 512;                /* the headers' start in hdr, if any */
    int i, nrem = rem_bits / 8;

    if (H1_IN_FMT(img) != 1) {
        qemu_log_mask(LOG_UNIMP, "xm-h1: H.264 input format %d\n",
                      H1_IN_FMT(img));
        return H1_INT_ERROR;
    }
    if (!xm_h1_geometry_ok(s, "H.264", mbw, mbh, stride)) {
        return H1_INT_ERROR;
    }
    if (rem_bits % 8) {
        qemu_log_mask(LOG_UNIMP, "xm-h1: %d header bits, not whole bytes\n",
                      rem_bits);
    }
    for (i = 0; i < nrem; i++) {
        rem_bytes[i] = rem >> (56 - 8 * i);
    }
    if (idr) {
        /* the SPS/PPS the library wrote, completed by the remainder */
        dma_memory_read(&address_space_memory, out - 512, hdr, 512,
                        MEMTXATTRS_UNSPECIFIED);
        memcpy(hdr + 512, rem_bytes, nrem);
        h264_enc_parse_headers(&s->h264, hdr, 512 + nrem);
        /* the last SPS's start code: the dump of an IDR job carries the
         * headers it was coded against, so the files concatenate into a
         * stream any decoder plays even when rate control rewrites the PPS */
        for (i = 512 - 5; i >= 0; i--) {
            if (!hdr[i] && !hdr[i + 1] && hdr[i + 2] == 1 &&
                (hdr[i + 3] & 0x1f) == 7) {
                hdr_at = i > 0 && !hdr[i - 1] ? i - 1 : i;
                break;
            }
        }
    }

    g_autofree uint8_t *y = g_malloc((size_t)stride * mbh * 16);
    g_autofree uint8_t *uv = g_malloc((size_t)stride * mbh * 8);
    dma_memory_read(&address_space_memory, s->regs[H1_REG_ADDR_IN_PLANE_0 / 4],
                    y, (size_t)stride * mbh * 16, MEMTXATTRS_UNSPECIFIED);
    dma_memory_read(&address_space_memory, s->regs[H1_REG_ADDR_IN_PLANE_1 / 4],
                    uv, (size_t)stride * mbh * 8, MEMTXATTRS_UNSPECIFIED);

    g_autofree uint8_t *buf = g_malloc(MAX(cap, 16));
    memcpy(buf, rem_bytes, MIN(nrem, (int)cap));
    size_t n = h264_enc_frame(&s->h264, idr,
                              H1_CTRL0_INIT_QP(s->regs[H1_REG_ENC_CTRL0 / 4]),
                              mbw, mbh, y, stride, uv, stride,
                              buf + nrem, cap > (uint32_t)nrem ? cap - nrem : 0);
    if (!n) {
        qemu_log_mask(LOG_UNIMP, "xm-h1: no SPS/PPS this encoder handles "
                      "(POC type 0/2, progressive) before the stream\n");
        return H1_INT_ERROR;
    }
    n += nrem;
    if (n > cap) {
        /* Hantro's library answers BUFF_FULL with a bigger buffer or a
         * skipped frame; nothing is written */
        qemu_log_mask(LOG_GUEST_ERROR, "xm-h1: H.264 output over the %u-byte "
                      "buffer\n", cap);
        return H1_INT_BUF_FULL;
    }
    dma_memory_write(&address_space_memory, out, buf, n,
                     MEMTXATTRS_UNSPECIFIED);
    if (s->dump_dir) {
        g_autofree uint8_t *file = g_malloc(512 - hdr_at + n);
        memcpy(file, hdr + hdr_at, 512 - hdr_at);
        memcpy(file + 512 - hdr_at, buf, n);
        xm_h1_dump(s, "h264", file, 512 - hdr_at + n);
    }
    s->regs[H1_REG_STR_BUF_LIMIT / 4] = n * 8;
    return H1_INT_FRAME_RDY;
}

static void xm_h1_update_irq(XmH1State *s)
{
    const uint32_t v = s->regs[H1_REG_INTERRUPT / 4];

    qemu_set_irq(s->irq, (v & H1_INT_BIT) && !(v & H1_INT_DIS));
}

static void xm_h1_start(XmH1State *s)
{
    const uint32_t ctrl = s->regs[H1_REG_ENC_CTRL / 4];
    uint32_t status = H1_INT_ERROR;

    if (s->dump_dir) {
        xm_h1_dump_regs(s);
    }
    if (H1_ENC_MODE(ctrl) == H1_ENC_MODE_JPEG) {
        status = xm_h1_jpeg(s);
    } else if (H1_ENC_MODE(ctrl) == H1_ENC_MODE_H264) {
        status = xm_h1_h264(s);
    } else {
        qemu_log_mask(LOG_UNIMP, "xm-h1: encode mode %u\n", H1_ENC_MODE(ctrl));
    }
    s->jobs++;
    s->regs[H1_REG_ENC_CTRL / 4] &= ~H1_ENC_EN;
    s->regs[H1_REG_INTERRUPT / 4] |= status | H1_INT_BIT;
    xm_h1_update_irq(s);
}

static uint64_t xm_h1_read(void *opaque, hwaddr off, unsigned size)
{
    XmH1State *s = XM_H1(opaque);

    if (off == H1_SWREG0_ID) {
        return H1_ID;
    }
    if (off == H1_SWREG63_CFG) {
        return H1_CFG | (s->max_width & H1_CFG_MAXW);
    }
    return s->regs[off / 4];
}

static void xm_h1_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    XmH1State *s = XM_H1(opaque);

    if (off == H1_SWREG0_ID || off == H1_SWREG63_CFG) {
        return;
    }
    s->regs[off / 4] = val;
    if (off == H1_REG_INTERRUPT) {
        xm_h1_update_irq(s);
    } else if (off == H1_REG_ENC_CTRL && (val & H1_ENC_EN)) {
        xm_h1_start(s);
    }
}

static const MemoryRegionOps xm_h1_ops = {
    .read = xm_h1_read,
    .write = xm_h1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_h1_reset(DeviceState *dev)
{
    XmH1State *s = XM_H1(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void xm_h1_init(Object *obj)
{
    XmH1State *s = XM_H1(obj);

    memory_region_init_io(&s->iomem, obj, &xm_h1_ops, s, TYPE_XM_H1,
                          H1_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property xm_h1_props[] = {
    DEFINE_PROP_STRING("dump-dir", XmH1State, dump_dir),
    DEFINE_PROP_UINT32("max-width", XmH1State, max_width, 1920),
};

static void xm_h1_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_h1_reset);
    device_class_set_props(DEVICE_CLASS(klass), xm_h1_props);
}

static const TypeInfo xm_h1_info = {
    .name          = TYPE_XM_H1,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmH1State),
    .instance_init = xm_h1_init,
    .class_init    = xm_h1_class_init,
};

static void xm_h1_register_types(void)
{
    type_register_static(&xm_h1_info);
}

type_init(xm_h1_register_types)
