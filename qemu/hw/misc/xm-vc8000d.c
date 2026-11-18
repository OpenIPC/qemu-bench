/*
 * Xiongmai XM8536E Hantro VC8000D video decoder (0x21200000, 8 KiB, IRQ 84).
 * xm_h265d.ko (hantrodec) maps 3 KiB of swregs from +0x800 and accepts ID
 * 0x9001 (VC8000D), 0xB16D (BigOcean) or 0x6731/2 (G2) in the top half of
 * swreg0; Sofia's DWL (libvdec/libh265d, linked into Sofia) takes 0x9001.
 *
 * The synthesis registers say what the core decodes; all zero, the kernel
 * refuses every client and libvdec's channels are never created:
 *   swreg50  [25:24] H.264 (2 = high), [28] JPEG, [10:0] max width low
 *   swreg54  [15:14] max width / 2048, [11] stride support
 *   swreg56  [28:26] HEVC (1 = main)
 *   swreg260 [31] post-processor
 *   swreg309 hardware build ID, which libvdec looks up in its feature
 *            table (codec support and maximum size come from there):
 *            0x1F9F = 8192x4352, H.264 high, HEVC main/main10, PP (0x1FA8,
 *            the same at 4096x2160, makes libvdec refuse a 2304x2592 camera
 *            with DEC_STREAM_NOT_SUPPORTED).  Older libvdec builds have
 *            other tables: XM8520D's Sofia (2021) knows 0x1F41..0x1F66
 *            only; with an ID it does not know, every H.264 instance --
 *            which libvdec creates for each channel first -- fails
 *            (H264DecInit -1000) and playback shows nothing.  Its G2
 *            takes 0xFFFF there (every feature, four PPUs): 0x1F48, the
 *            8192-wide H.264 / HEVC one, has a single PPU and fails
 *            HevcDecSetInfo's post-processor check.
 * They are read only: the driver's reset writes 0 over the window.
 *
 * A decode run is started by writing swreg1 bit 0 and ends with swreg1
 * DEC_IRQ | DEC_RDY_INT.  Registers as libvdec programs them for HEVC
 * (mode 12) and H.264 (mode 0, 15 for high10):
 *   swreg3   [31:27] decoding mode (3 = JPEG, 12 = HEVC)
 *   swreg4   [31:19] / [18:6] picture width / height in 8-pixel units
 *   swreg5   [31:25] start bit of the stream data within the base's 16 bytes
 *   swreg6   stream length in bytes, counted from the 16-byte aligned base
 *   swreg169 stream base (swreg168 the high half)
 *   swreg167/402  per-instance table buffers: they identify the instance
 *   swreg386 / swreg388  post-processor luma / chroma output (NV12)
 *   swreg391 [31:16] / [15:0] PP luma / chroma stride
 *   swreg395 [31:16] / [15:0] PP output width / height
 * The stream buffer is plain Annex B with the parameter sets in front of
 * the first slice, as the camera sent it.
 *
 * PP output is written whatever swreg260 says: its bit 31 is a read-only
 * synthesis bit here.
 *
 * Decoding is done with libavcodec: the slice data, with the VPS/SPS/PPS
 * NAL units that come right before it in the buffer, goes to one decoder
 * per instance, and the picture is scaled (nearest) into the PP output,
 * which is what VPSS/VO read.  The tiled reference pictures and motion
 * vectors the core also writes are not produced.  A JPEG run (mode 3)
 * decodes the whole file found in front of the scan data it is given.
 *
 * The XM8520D's Hantro G2 (xm_h265d.ko: ID 0x8001xxxx, its swreg0 at the
 * block's base) is this device with id=0x80010000, mapped 0x800 lower so
 * swreg0 lines up.  Its post-processor words sit 60 registers lower
 * (swreg326 / 328 luma / chroma, 329 strides, 332 size); with none set,
 * pictures go to its own output registers (swreg65 / 99) as raster NV12.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "system/address-spaces.h"
#include "qemu/units.h"
#include "qom/object.h"

#ifdef XM_VC8000D_LIBAVCODEC
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>
#include <libavutil/pixdesc.h>
#endif

#define TYPE_XM_VC8000D "xm-vc8000d"
OBJECT_DECLARE_SIMPLE_TYPE(XmVc8000dState, XM_VC8000D)

#define VC8000D_SIZE    0x2000
#define VC8000D_SWREG0  0x800
#define VC8000D_ID      0x90010000u
#define SWREG(n)        (VC8000D_SWREG0 + 4 * (n))
#define SWREG_NUM       512
/* swreg1: decoder control and interrupt status */
#define DEC_E           (1u << 0)
#define DEC_IRQ_DIS     (1u << 4)
#define DEC_IRQ         (1u << 8)
#define DEC_RDY_INT     (1u << 12)
#define DEC_IRQ_BITS    0x03fff800u     /* [25:11]: the interrupt causes */

#define MODE_H264       0
#define MODE_JPEG       3
#define MODE_HEVC       12
#define MODE_H264_HI10  15

#define PS_WINDOW       2048    /* how far back parameter sets are looked for */
#define JPEG_BACK       (256 * KiB) /* how far back a JPEG's SOI is looked for */
#define STREAM_MAX      (16 * MiB)
#define VC_INSTANCES    16
#define VC_PAD          64      /* AV_INPUT_PADDING_SIZE */

typedef struct XmVcInst {
    uint32_t key;
    int mode;
    uint64_t used;
#ifdef XM_VC8000D_LIBAVCODEC
    AVCodecContext *ctx;
#endif
} XmVcInst;

struct XmVc8000dState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    unsigned runs;
    uint64_t clock;
    uint32_t regs[VC8000D_SIZE / 4];
    XmVcInst inst[VC_INSTANCES];
#ifdef XM_VC8000D_LIBAVCODEC
    AVPacket *pkt;
    AVFrame *frame;
#endif
    uint8_t *buf;
    uint8_t *line;
    uint32_t id;                /* swreg0: 0x9001xxxx VC8000D, 0x8001xxxx G2 */
    uint32_t trace_reads;       /* -global ...trace-reads=N: log N reads */
    uint32_t build_id;          /* swreg309 */
};

static uint64_t xm_vc8000d_read(void *opaque, hwaddr offset, unsigned size)
{
    XmVc8000dState *s = opaque;
    uint32_t v = offset == VC8000D_SWREG0 ? s->id : s->regs[offset / 4];

    if (s->trace_reads) {
        s->trace_reads--;
        qemu_log("xm-vc8000d %08x: read swreg%u (+0x%03x) = 0x%08x\n", s->id,
                 (unsigned)(offset - VC8000D_SWREG0) / 4, (unsigned)offset, v);
    }
    return v;
}

#ifdef XM_VC8000D_LIBAVCODEC
static XmVcInst *xm_vc8000d_inst(XmVc8000dState *s, uint32_t key, int mode)
{
    XmVcInst *in = NULL, *lru = &s->inst[0];
    const AVCodec *codec;
    int n;

    for (n = 0; n < VC_INSTANCES; n++) {
        if (s->inst[n].ctx && s->inst[n].key == key &&
            s->inst[n].mode == mode) {
            in = &s->inst[n];
            break;
        }
        if (s->inst[n].used < lru->used) {
            lru = &s->inst[n];
        }
    }
    if (!in) {
        in = lru;
        avcodec_free_context(&in->ctx);
        codec = avcodec_find_decoder(mode == MODE_HEVC ? AV_CODEC_ID_HEVC :
                                     mode == MODE_JPEG ? AV_CODEC_ID_MJPEG :
                                                         AV_CODEC_ID_H264);
        in->ctx = codec ? avcodec_alloc_context3(codec) : NULL;
        if (!in->ctx) {
            return NULL;
        }
        /* a picture comes out of the run that decoded it */
        in->ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
        in->ctx->thread_type = FF_THREAD_SLICE;
        in->ctx->thread_count = 4;
        if (avcodec_open2(in->ctx, codec, NULL) < 0) {
            avcodec_free_context(&in->ctx);
            return NULL;
        }
        in->key = key;
        in->mode = mode;
    }
    in->used = ++s->clock;
    return in;
}

static bool xm_vc8000d_is_ps(const uint8_t *nal, int mode)
{
    if (mode == MODE_HEVC) {
        int t = (nal[0] >> 1) & 0x3f;
        return t >= 32 && t <= 34;              /* VPS, SPS, PPS */
    }
    return (nal[0] & 0x1f) == 7 || (nal[0] & 0x1f) == 8;  /* SPS, PPS */
}

/*
 * The parameter sets right before the slice data: the trailing run of
 * VPS/SPS/PPS NAL units in buf[0, len), as an offset into buf (len if none).
 */
static size_t xm_vc8000d_ps_start(const uint8_t *buf, size_t len, int mode)
{
    size_t sc[64], n = 0, i, end = len, first = len;

    for (i = 0; i + 3 < len; i++) {
        if (buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 1) {
            sc[n++ % 64] = i;
            i += 2;
        }
    }
    while (n > 0) {
        size_t p = sc[--n % 64];

        if (p + 3 >= end || !xm_vc8000d_is_ps(&buf[p + 3], mode)) {
            break;
        }
        first = (p > 0 && buf[p - 1] == 0) ? p - 1 : p;
        end = first;
    }
    return first;
}

/*
 * Nearest-neighbour scale of the decoded picture into the PP's NV12: any
 * planar YUV the decoders give (a JPEG's 4:2:2 / 4:4:4 / grey too) or NV12.
 */
static void xm_vc8000d_pp_out(XmVc8000dState *s, AVFrame *f)
{
    const uint32_t *r = &s->regs[VC8000D_SWREG0 / 4];
    const AVPixFmtDescriptor *d = av_pix_fmt_desc_get(f->format);
    hwaddr lu = r[386], ch = r[388];
    uint32_t ys = r[391] >> 16, cs = r[391] & 0xffff;
    uint32_t ow = r[395] >> 16, oh = r[395] & 0xffff;
    bool nv12 = f->format == AV_PIX_FMT_NV12;
    bool grey;

    /* the G2's post-processor: the same words 60 registers lower */
    if (!lu && r[326]) {
        lu = r[326] | (hwaddr)r[325] << 32;
        ch = r[328] | (hwaddr)r[327] << 32;
        ys = r[329] >> 16;
        cs = r[329] & 0xffff;
        ow = r[332] >> 16;
        oh = r[332] & 0xffff;
    }
    /* no post-processing: the decoder's own output, raster NV12 at
     * swreg65 / swreg99, the picture's size in 8x8 units in swreg4 */
    if (!lu) {
        lu = r[65] | (hwaddr)r[64] << 32;
        ch = r[99] | (hwaddr)r[98] << 32;
        ow = (r[4] >> 19) * 8;
        oh = ((r[4] >> 6) & 0x1fff) * 8;
        ys = cs = ow;
    }
    uint32_t x, y, cw, chh;

    if (!lu || !ow || !oh || ow > 8192 || oh > 8192 || ys < ow || cs < ow ||
        !d || (d->flags & (AV_PIX_FMT_FLAG_RGB | AV_PIX_FMT_FLAG_PAL)) ||
        d->comp[0].depth != 8 || (!nv12 && !(d->flags & AV_PIX_FMT_FLAG_PLANAR) &&
                                  d->nb_components > 1)) {
        return;
    }
    grey = d->nb_components < 3;
    cw = AV_CEIL_RSHIFT(f->width, d->log2_chroma_w);
    chh = AV_CEIL_RSHIFT(f->height, d->log2_chroma_h);
    for (y = 0; y < oh; y++) {
        const uint8_t *src = f->data[0] +
            (size_t)(y * (uint64_t)f->height / oh) * f->linesize[0];

        for (x = 0; x < ow; x++) {
            s->line[x] = src[x * (uint64_t)f->width / ow];
        }
        address_space_write(&address_space_memory, lu + (hwaddr)y * ys,
                            MEMTXATTRS_UNSPECIFIED, s->line, ow);
    }
    for (y = 0; y < oh / 2; y++) {
        size_t sy = y * (uint64_t)chh / (oh / 2);

        for (x = 0; x < ow / 2; x++) {
            size_t sx = x * (uint64_t)cw / (ow / 2);

            if (grey) {
                s->line[2 * x] = s->line[2 * x + 1] = 128;
            } else if (nv12) {
                const uint8_t *uv = f->data[1] + sy * f->linesize[1];
                s->line[2 * x] = uv[2 * sx];
                s->line[2 * x + 1] = uv[2 * sx + 1];
            } else {
                s->line[2 * x] = f->data[1][sy * f->linesize[1] + sx];
                s->line[2 * x + 1] = f->data[2][sy * f->linesize[2] + sx];
            }
        }
        address_space_write(&address_space_memory, ch + (hwaddr)y * cs,
                            MEMTXATTRS_UNSPECIFIED, s->line, ow & ~1u);
    }
}

/*
 * JPEG: the driver points the engine at the entropy-coded data after SOS
 * (swreg169) and parses the headers itself, but both U-Boot's logo and
 * Sofia keep the whole file in front of it: decode it from its SOI.
 */
static void xm_vc8000d_jpeg(XmVc8000dState *s)
{
    const uint32_t *r = &s->regs[VC8000D_SWREG0 / 4];
    hwaddr base = r[169] | (hwaddr)r[168] << 32;
    uint32_t len = r[6], back = MIN(base, (hwaddr)JPEG_BACK);
    XmVcInst *in;
    int64_t i;

    if (!base || !len || len > STREAM_MAX - JPEG_BACK - VC_PAD) {
        return;
    }
    address_space_read(&address_space_memory, base - back,
                       MEMTXATTRS_UNSPECIFIED, s->buf, back + len);
    for (i = back - 3; i >= 0; i--) {
        if (s->buf[i] == 0xff && s->buf[i + 1] == 0xd8 && s->buf[i + 2] == 0xff) {
            break;
        }
    }
    if (i < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-vc8000d: no JPEG SOI before 0x%"
                      HWADDR_PRIx "\n", base);
        return;
    }
    in = xm_vc8000d_inst(s, 0, MODE_JPEG);
    if (!in) {
        return;
    }
    memset(s->buf + back + len, 0, VC_PAD);
    s->pkt->data = s->buf + i;
    s->pkt->size = back + len - i;
    if (avcodec_send_packet(in->ctx, s->pkt) < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-vc8000d: JPEG decode error\n");
        return;
    }
    while (avcodec_receive_frame(in->ctx, s->frame) >= 0) {
        xm_vc8000d_pp_out(s, s->frame);
        av_frame_unref(s->frame);
    }
}

static void xm_vc8000d_decode(XmVc8000dState *s, int mode)
{
    const uint32_t *r = &s->regs[VC8000D_SWREG0 / 4];
    hwaddr base = r[169] | (hwaddr)r[168] << 32;
    uint32_t skip = (r[5] >> 25) / 8, len = r[6];
    uint32_t key = r[167] ? r[167] : r[402];
    size_t ps, data;
    XmVcInst *in;
    int ret;

    if (!base || len <= skip || len > STREAM_MAX - PS_WINDOW) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-vc8000d: stream 0x%" HWADDR_PRIx
                      " length %u start %u\n", base, len, skip);
        return;
    }
    in = xm_vc8000d_inst(s, key, mode);
    if (!in) {
        return;
    }
    /* PS_WINDOW bytes before the slice data, then the data itself */
    if (base + skip < PS_WINDOW) {
        return;
    }
    address_space_read(&address_space_memory, base + skip - PS_WINDOW,
                       MEMTXATTRS_UNSPECIFIED, s->buf, PS_WINDOW + len - skip);
    ps = xm_vc8000d_ps_start(s->buf, PS_WINDOW, mode);
    data = PS_WINDOW + len - skip;
    memset(s->buf + data, 0, VC_PAD);

    s->pkt->data = s->buf + ps;
    s->pkt->size = data - ps;
    ret = avcodec_send_packet(in->ctx, s->pkt);
    if (ret < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-vc8000d: decode error %d\n", ret);
        return;
    }
    while (avcodec_receive_frame(in->ctx, s->frame) >= 0) {
        xm_vc8000d_pp_out(s, s->frame);
        av_frame_unref(s->frame);
    }
}
#endif

static void xm_vc8000d_run(XmVc8000dState *s)
{
    uint32_t *r = &s->regs[VC8000D_SWREG0 / 4];
    int mode = r[3] >> 27, n;

    if (s->runs < 4) {
        qemu_log_mask(LOG_UNIMP, "xm-vc8000d: run %u:", s->runs);
        for (n = 0; n < SWREG_NUM; n++) {
            if (r[n]) {
                qemu_log_mask(LOG_UNIMP, " %d=%08x", n, r[n]);
            }
        }
        qemu_log_mask(LOG_UNIMP, "\n");
    }
    s->runs++;
    (void)mode;
#ifdef XM_VC8000D_LIBAVCODEC
    if (mode == MODE_HEVC || mode == MODE_H264 || mode == MODE_H264_HI10) {
        xm_vc8000d_decode(s, mode);
    } else if (mode == MODE_JPEG) {
        xm_vc8000d_jpeg(s);
    }
#endif
    /*
     * The stream pointer moves on to where decoding stopped: the whole
     * buffer given (swreg6 counts from the 16-byte aligned base).  Older
     * libvdec builds (XM8520D's) take what it consumed from it, and with
     * the base left as written count the whole buffer as consumed, then
     * drop the picture as broken (HevcRunAsic: DEC_HW_IRQ_ERROR).
     */
    if (mode != MODE_JPEG) {
        r[169] += r[6];
    }
    r[1] = (r[1] & ~(DEC_E | DEC_IRQ_BITS)) | DEC_IRQ | DEC_RDY_INT;
}

static void xm_vc8000d_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    XmVc8000dState *s = opaque;

    /* read only; the driver's reset writes 0 over the whole window */
    if (offset == SWREG(50) || offset == SWREG(54) || offset == SWREG(56) ||
        offset == SWREG(260) || offset == SWREG(309)) {
        return;
    }
    s->regs[offset / 4] = value;
    if (offset == SWREG(1)) {
        if (value & DEC_E) {
            xm_vc8000d_run(s);
        }
        qemu_set_irq(s->irq, (s->regs[offset / 4] & DEC_IRQ) &&
                     !(s->regs[offset / 4] & DEC_IRQ_DIS));
    }
}

static void xm_vc8000d_reset(DeviceState *dev)
{
    XmVc8000dState *s = XM_VC8000D(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[SWREG(50) / 4] = 2u << 24 | 1u << 28;
    s->regs[SWREG(54) / 4] = 2u << 14 | 1u << 11;
    s->regs[SWREG(56) / 4] = 1u << 26;
    s->regs[SWREG(260) / 4] = 1u << 31;
    s->regs[SWREG(309) / 4] = s->build_id;
    qemu_irq_lower(s->irq);
}

static const MemoryRegionOps xm_vc8000d_ops = {
    .read = xm_vc8000d_read,
    .write = xm_vc8000d_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void xm_vc8000d_init(Object *obj)
{
    XmVc8000dState *s = XM_VC8000D(obj);

    memory_region_init_io(&s->iomem, obj, &xm_vc8000d_ops, s, TYPE_XM_VC8000D,
                          VC8000D_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void xm_vc8000d_realize(DeviceState *dev, Error **errp)
{
    XmVc8000dState *s = XM_VC8000D(dev);

    s->buf = g_malloc(STREAM_MAX + VC_PAD);
    s->line = g_malloc(16384);
#ifdef XM_VC8000D_LIBAVCODEC
    s->pkt = av_packet_alloc();
    s->frame = av_frame_alloc();
#endif
}

static const Property xm_vc8000d_props[] = {
    DEFINE_PROP_UINT32("id", XmVc8000dState, id, VC8000D_ID),
    DEFINE_PROP_UINT32("trace-reads", XmVc8000dState, trace_reads, 0),
    DEFINE_PROP_UINT32("build-id", XmVc8000dState, build_id, 0x1f9f),
};

static void xm_vc8000d_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, xm_vc8000d_props);
    dc->realize = xm_vc8000d_realize;
    device_class_set_legacy_reset(dc, xm_vc8000d_reset);
}

static const TypeInfo xm_vc8000d_info = {
    .name = TYPE_XM_VC8000D,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmVc8000dState),
    .instance_init = xm_vc8000d_init,
    .class_init = xm_vc8000d_class_init,
};

static void xm_vc8000d_register(void)
{
    type_register_static(&xm_vc8000d_info);
}

type_init(xm_vc8000d_register)
