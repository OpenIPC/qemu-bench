/*
 * Xiongmai XM530V200 JPEG encoder engine (0x21100000, SPI 46), as jpeg.ko
 * and the library in Sofia drive it for snapshots (VENC channel 4).
 *
 * jpeg.ko is a shim like hevc.ko: an ioctl writes the library's register
 * list (offset, value, dirty; the start word last), another sleeps until
 * jpeg_interrupt sees +0x3C == 3 (done: it writes 0 to +0x00 and +0x24) or
 * an error in +0x40 (JPEG_MIS_ERR), or times out after a second -- status
 * 0x80, which Sofia reports as "wl_jpeg_wait_ready error status=0x80" and
 * which, a second per snapshot, starves the live streams of a camera that
 * records to a card.  What a job is, from the registers the library writes:
 *
 *   +0x000     1: start
 *   +0x008     5 (NV12 here)
 *   +0x00C     height << 16 | width          +0x014  the source stride
 *   +0x060/64  luma / chroma quantisers, a write each:
 *              1 << 24 | (0x40 + i) << 16 | 65536 / q, natural order
 *   +0x088     where the scan goes           +0x08C  the buffer's end
 *   +0x090/94  source luma / chroma (NV12)
 *   +0x03C     3: done                       +0x044  bytes of scan written
 *
 * The library writes SOI, DQT, SOF0, DHT (the Annex K tables) and SOS up to
 * +0x088 itself, and EOI after the scan; the engine writes only the
 * entropy-coded scan (jpeg-enc.c).  A job completes a millisecond after
 * its start.
 *
 * dump-dir (-global xm-jpeg.dump-dir=DIR) keeps every job on the host as a
 * whole frame-NNNNN.jpg (a header like the library's, the scan, EOI).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/dma.h"
#include "jpeg-enc.h"

#define TYPE_XM_JPEG "xm-jpeg"
OBJECT_DECLARE_SIMPLE_TYPE(XmJpegState, XM_JPEG)

#define XM_JPEG_SIZE    0x1000
#define J_START         0x000
#define J_SIZE          0x00C
#define J_STRIDE        0x014
#define J_STATUS        0x03C
#define J_ERR           0x040
#define J_LEN           0x044
#define J_QUANT_LUMA    0x060
#define J_QUANT_CHROMA  0x064
#define J_OUT           0x088
#define J_OUT_END       0x08C
#define J_SRC_Y         0x090
#define J_SRC_C         0x094

#define ST_DONE         3
#define ERR_BUF_FULL    1

struct XmJpegState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[XM_JPEG_SIZE / 4];
    uint8_t q[2][64];           /* luma, chroma; natural order */
    char *dump_dir;
    uint32_t jobs;
    bool trace;
};

#define R(s, off) ((s)->regs[(off) / 4])

static void xm_jpeg_update(XmJpegState *s)
{
    qemu_set_irq(s->irq, R(s, J_STATUS) == ST_DONE || R(s, J_ERR));
}

/* a quantiser write: the engine multiplies by 65536 / q */
static void xm_jpeg_quant(XmJpegState *s, int t, uint32_t val)
{
    const int i = ((val >> 16) & 0xff) - 0x40;
    const uint32_t r = val & 0xffff;

    if (!(val & (1u << 24)) || i < 0 || i >= 64 || !r) {
        return;
    }
    s->q[t][i] = MIN(MAX((65536 + r / 2) / r, 1), 255);
}

static void xm_jpeg_job(XmJpegState *s)
{
    const int w = R(s, J_SIZE) & 0xffff, h = R(s, J_SIZE) >> 16;
    const int ys = R(s, J_STRIDE) ? R(s, J_STRIDE) : w;
    const uint32_t out = R(s, J_OUT);
    const uint32_t cap = R(s, J_OUT_END) > out ? R(s, J_OUT_END) - out : 0;
    bool overflow;

    R(s, J_LEN) = 0;
    if (w < 8 || h < 8 || w > 4096 || h > 4096 || ys < w || !out || !cap ||
        !R(s, J_SRC_Y) || !R(s, J_SRC_C)) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-jpeg: job %dx%d stride %d to "
                      "0x%x+%u not done\n", w, h, ys, out, cap);
        R(s, J_ERR) = ERR_BUF_FULL;
        return;
    }
    const int cw = (w + 1) / 2, ch = (h + 1) / 2;
    g_autofree uint8_t *y = g_malloc((size_t)ys * h);
    g_autofree uint8_t *uv = g_malloc((size_t)ys * ch);
    g_autofree uint8_t *cb = g_malloc((size_t)cw * ch);
    g_autofree uint8_t *cr = g_malloc((size_t)cw * ch);
    g_autofree uint8_t *buf = g_malloc(cap);

    dma_memory_read(&address_space_memory, R(s, J_SRC_Y), y, (size_t)ys * h,
                    MEMTXATTRS_UNSPECIFIED);
    dma_memory_read(&address_space_memory, R(s, J_SRC_C), uv, (size_t)ys * ch,
                    MEMTXATTRS_UNSPECIFIED);
    for (int r = 0; r < ch; r++) {
        for (int c = 0; c < cw; c++) {
            cb[r * cw + c] = uv[(size_t)r * ys + 2 * c];
            cr[r * cw + c] = uv[(size_t)r * ys + 2 * c + 1];
        }
    }
    const size_t n = jpeg_enc_scan(y, ys, cb, cr, cw, w, h, s->q[0], s->q[1],
                                   false, buf, cap, &overflow);
    if (overflow) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-jpeg: scan over the %u-byte "
                      "buffer\n", cap);
        R(s, J_ERR) = ERR_BUF_FULL;
        return;
    }
    dma_memory_write(&address_space_memory, out, buf, n,
                     MEMTXATTRS_UNSPECIFIED);
    R(s, J_LEN) = n;
    if (s->dump_dir) {
        g_autoptr(GByteArray) f = g_byte_array_new();
        g_autofree char *path = g_strdup_printf("%s/frame-%05u.jpg",
                                                s->dump_dir, s->jobs);
        static const uint8_t eoi[] = { 0xff, 0xd9 };

        jpeg_enc_header(f, w, h, s->q[0], s->q[1]);
        g_byte_array_append(f, buf, n);
        g_byte_array_append(f, eoi, 2);
        g_file_set_contents(path, (const char *)f->data, f->len, NULL);
    }
}

static void xm_jpeg_done(void *opaque)
{
    XmJpegState *s = XM_JPEG(opaque);

    if (!R(s, J_ERR)) {
        R(s, J_STATUS) = ST_DONE;
    }
    xm_jpeg_update(s);
}

static uint64_t xm_jpeg_read(void *opaque, hwaddr off, unsigned size)
{
    return XM_JPEG(opaque)->regs[off / 4];
}

static void xm_jpeg_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    XmJpegState *s = XM_JPEG(opaque);

    if (s->trace) {
        qemu_log("xm-jpeg: +0x%03x = 0x%08x\n", (unsigned)off, (unsigned)val);
    }
    switch (off) {
    case J_START:
        R(s, off) = val;
        if (val & 1) {
            R(s, J_STATUS) = 0;
            R(s, J_ERR) = 0;
            xm_jpeg_job(s);
            s->jobs++;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      SCALE_MS);
        } else {
            /* jpeg_interrupt: the job is taken */
            R(s, J_STATUS) = 0;
            R(s, J_ERR) = 0;
        }
        break;
    case J_QUANT_LUMA:
    case J_QUANT_CHROMA:
        R(s, off) = val;
        xm_jpeg_quant(s, off == J_QUANT_CHROMA, val);
        break;
    case J_STATUS:
    case J_ERR:
    case J_LEN:
        break;
    default:
        R(s, off) = val;
        break;
    }
    xm_jpeg_update(s);
}

static const MemoryRegionOps xm_jpeg_ops = {
    .read = xm_jpeg_read,
    .write = xm_jpeg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_jpeg_reset(DeviceState *dev)
{
    XmJpegState *s = XM_JPEG(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->q, 1, sizeof(s->q));
    xm_jpeg_update(s);
}

static void xm_jpeg_init(Object *obj)
{
    XmJpegState *s = XM_JPEG(obj);

    memory_region_init_io(&s->iomem, obj, &xm_jpeg_ops, s, TYPE_XM_JPEG,
                          XM_JPEG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_jpeg_done, s);
}

static const Property xm_jpeg_props[] = {
    DEFINE_PROP_STRING("dump-dir", XmJpegState, dump_dir),
    DEFINE_PROP_BOOL("trace", XmJpegState, trace, false),
};

static void xm_jpeg_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_jpeg_reset);
    device_class_set_props(DEVICE_CLASS(klass), xm_jpeg_props);
}

static const TypeInfo xm_jpeg_info = {
    .name          = TYPE_XM_JPEG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmJpegState),
    .instance_init = xm_jpeg_init,
    .class_init    = xm_jpeg_class_init,
};

static void xm_jpeg_register_types(void)
{
    type_register_static(&xm_jpeg_info);
}

type_init(xm_jpeg_register_types)
