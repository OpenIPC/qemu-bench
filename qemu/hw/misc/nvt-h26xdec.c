/*
 * Novatek NA51103 H.264 / H.265 decoder (0x2_F0600000, SPI 9).
 *
 * Driven by the HDAL's kdrv_videodec/h26xdec.ko (not stripped).  The
 * driver parses the parameter sets and slice headers itself, keeps the
 * hardware's registers in a software map and hands each job over as a
 * link list of register writes.  The 40 KiB window:
 *
 *   0x0000  the decoder core: 0x34 interrupt status, 0x3C its mask
 *           (& 0x707), 0x58 busy, 0x2000.. the register map the list fills
 *   0x1800  the auxiliary block: 0x4C the job interrupt's status (bit 0),
 *           0x48 its acknowledge (VPU_ClearInterrupt() writes 1; the line
 *           stays up until it does, and a handler run with no list in
 *           flight indexes garbage)
 *   0x3000  the link-list wrapper: 0x00 control (2 fires, 1 resets), 0x04
 *           the list's address, 0x08 bit 31 reset done, 0x18 / 0x1C the
 *           last command and value run (the job id in 0x18 [27:20];
 *           IMVQ_VDEC_LLCheckFinalJob() wants 0x1C = -16)
 *
 * List commands are two words, the first (op << 28) | (job << 20) |
 * offset, the second a value: op 0 writes the value to the register, op 8
 * waits until the register has the value's bits set, op 0xC links to the
 * list at the value, or ends the job when the value is -16.
 *
 *   0x1000  the H.264 core: 0x1000 control (bit 0 reset, which clears
 *           itself once done -- VPU_ResetHW() polls for that; bit 1 decode;
 *           bit 16 done, which the list's waits check), 0x1038 interrupt enable, 0x103C flags (bit 0 done,
 *           0x16 errors; write 1 to clear), 0x1040 / 0x1044 the picture's
 *           Y / UV, 0x1080 checksum, 0x10C4 concealed macroblocks
 *   0x1814  the BIN (CABAC) stage's control, the same three bits
 *
 * A decode reads the job's bitstream -- a whole Annex-B access unit, from
 * the {count, address, length} block 0x102C points at -- and, built with
 * libavcodec, decodes it and writes the picture to 0x1040 (Y) and 0x1044
 * (UV interleaved), at the strides in 0x1008 ([15:0] Y, [31:16] UV).  With
 * 0x1894 bit 0 set it also writes the reduced copy the preview windows
 * scale from: 1 / 2^[11:8] of the width and 1 / 2^[15:12] of the height
 * (half each way as the NVR sets it), to 0x1898, its width the stride and
 * its UV right after its Y.  That layout is this model's: nvt-engine's
 * scaler (the VPE) reads it back the same way.  One
 * decoder per picture size keeps the streams apart.  Without libavcodec
 * jobs complete with nothing written.  trace=on logs each list, every
 * access and the bitstream's head.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#ifdef NVT_H26XDEC_LIBAVCODEC
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>
#endif

#define TYPE_NVT_H26XDEC "nvt-h26xdec"
OBJECT_DECLARE_SIMPLE_TYPE(NvtH26xDecState, NVT_H26XDEC)

#define DEC_SIZE            0xA000
#define CORE_INT_STS        0x0034
#define CORE_INT_MASK       0x003C
#define AUX_BASE            0x1800
#define AUX_INT_ACK         (AUX_BASE + 0x48)
#define AUX_INT_STS         (AUX_BASE + 0x4C)
#define LL_BASE             0x3000
#define LL_CTRL             (LL_BASE + 0x00)
#define LL_ADDR             (LL_BASE + 0x04)
#define LL_RESET_STS        (LL_BASE + 0x08)
#define LL_LAST_CMD         (LL_BASE + 0x18)
#define LL_LAST_VAL         (LL_BASE + 0x1C)

#define H264_CTRL           0x1000
#define H264_INT_FLG        0x103C
#define BIN_CTRL            0x1814
#define CTRL_RESET          1
#define CTRL_START          2
#define CTRL_DONE           (1u << 16)
#define H264_INT_DONE       1

#define H264_SIZE           0x1004
#define H264_STRIDE         0x1008
#define H264_BS_DESC        0x102C
#define H264_OUT_Y          0x1040
#define H264_OUT_UV         0x1044
#define SUB_CTRL            0x1894
#define SUB_Y               0x1898
#define BS_MAX              (4 * MiB)
#define DEC_CTXS            4

#define LL_OP_WRITE         0x0
#define LL_OP_WAIT          0x8
#define LL_OP_LINK          0xC

#define LL_CTRL_RESET       1
#define LL_CTRL_FIRE        2
#define LL_RESET_DONE       (1u << 31)
#define LL_FINAL            0xFFFFFFF0u
#define LL_MAX_CMDS         65536

#ifdef NVT_H26XDEC_LIBAVCODEC
typedef struct NvtDecCtx {
    AVCodecContext *ctx;
    uint32_t key;                       /* the job's 0x1004 */
    uint32_t used;
} NvtDecCtx;
#endif

struct NvtH26xDecState {
    SysBusDevice parent_obj;

#ifdef NVT_H26XDEC_LIBAVCODEC
    NvtDecCtx dec[DEC_CTXS];
    AVPacket *pkt;
    AVFrame *frame;
    uint32_t tick;
#endif

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;
    uint32_t regs[DEC_SIZE / 4];
    uint32_t jobs;
    bool trace;
};

static uint32_t *reg(NvtH26xDecState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_h26xdec_update_irq(NvtH26xDecState *s)
{
    bool aux = (*reg(s, AUX_INT_STS) & 1) != 0;
    bool core = (*reg(s, CORE_INT_STS) & *reg(s, CORE_INT_MASK) & 0x707) != 0;

    qemu_set_irq(s->irq, aux || core);
}

static void nvt_h26xdec_dump(NvtH26xDecState *s, const char *what, hwaddr a)
{
    uint8_t buf[32];
    char hex[sizeof(buf) * 2 + 1];
    int i;

    address_space_read(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                       buf, sizeof(buf));
    for (i = 0; i < sizeof(buf); i++) {
        sprintf(hex + 2 * i, "%02x", buf[i]);
    }
    qemu_log("nvt-h26xdec: job %u %s @%08" HWADDR_PRIx ": %s\n", s->jobs,
             what, a, hex);
}

#ifdef NVT_H26XDEC_LIBAVCODEC
/* The decoder for this stream: one per 0x1004, the least used recycled. */
static AVCodecContext *nvt_h26xdec_ctx(NvtH26xDecState *s, uint32_t key)
{
    NvtDecCtx *d = NULL;
    const AVCodec *codec;
    int i;

    for (i = 0; i < DEC_CTXS; i++) {
        if (s->dec[i].ctx && s->dec[i].key == key) {
            d = &s->dec[i];
            break;
        }
        if (!d || !s->dec[i].ctx || s->dec[i].used < d->used) {
            d = &s->dec[i];
        }
    }
    d->used = ++s->tick;
    if (d->ctx && d->key == key) {
        return d->ctx;
    }
    avcodec_free_context(&d->ctx);
    codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
        return NULL;
    }
    d->ctx = avcodec_alloc_context3(codec);
    d->ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
    d->ctx->thread_count = 1;
    if (avcodec_open2(d->ctx, codec, NULL) < 0) {
        avcodec_free_context(&d->ctx);
        return NULL;
    }
    d->key = key;
    return d->ctx;
}

/* The picture to the job's buffers: Y, then U and V interleaved. */
static void nvt_h26xdec_put_picture(NvtH26xDecState *s, AVFrame *f)
{
    uint32_t stride = *reg(s, H264_STRIDE);
    uint32_t ys = stride & 0xFFFF, cs = stride >> 16;
    hwaddr y = *reg(s, H264_OUT_Y), uv = *reg(s, H264_OUT_UV);
    g_autofree uint8_t *line = NULL;
    int r, x, w = f->width, h = f->height;

    if (f->format != AV_PIX_FMT_YUV420P && f->format != AV_PIX_FMT_YUVJ420P) {
        return;
    }
    ys = ys ? ys : w;
    cs = cs ? cs : ys;
    if (s->trace) {
        qemu_log("nvt-h26xdec: job %u picture %dx%d -> y %08" HWADDR_PRIx
                 " uv %08" HWADDR_PRIx " stride %u/%u\n", s->jobs, w, h, y,
                 uv, ys, cs);
    }
    w = MIN(w, (int)ys);
    for (r = 0; r < h; r++) {
        dma_memory_write(&address_space_memory, y + (hwaddr)r * ys,
                         f->data[0] + r * f->linesize[0], w,
                         MEMTXATTRS_UNSPECIFIED);
    }
    line = g_malloc(w);
    for (r = 0; r < h / 2; r++) {
        for (x = 0; x < w / 2; x++) {
            line[2 * x] = f->data[1][r * f->linesize[1] + x];
            line[2 * x + 1] = f->data[2][r * f->linesize[2] + x];
        }
        dma_memory_write(&address_space_memory, uv + (hwaddr)r * cs, line,
                         w & ~1, MEMTXATTRS_UNSPECIFIED);
    }
}

/* The reduced copy: nearest neighbour, Y then UV interleaved. */
static void nvt_h26xdec_put_sub(NvtH26xDecState *s, AVFrame *f)
{
    uint32_t ctrl = *reg(s, SUB_CTRL);
    int sw = f->width >> ((ctrl >> 8) & 0xF);
    int sh = f->height >> ((ctrl >> 12) & 0xF);
    hwaddr y = *reg(s, SUB_Y), uv = y + (hwaddr)sw * sh;
    g_autofree uint8_t *line = NULL;
    int r, x;

    if (!(ctrl & 1) || sw < 2 || sh < 2 || !y) {
        return;
    }
    line = g_malloc(sw);
    for (r = 0; r < sh; r++) {
        const uint8_t *src = f->data[0] + (r * f->height / sh) * f->linesize[0];

        for (x = 0; x < sw; x++) {
            line[x] = src[x * f->width / sw];
        }
        dma_memory_write(&address_space_memory, y + (hwaddr)r * sw, line, sw,
                         MEMTXATTRS_UNSPECIFIED);
    }
    for (r = 0; r < sh / 2; r++) {
        int sr = r * (f->height / 2) / (sh / 2);

        for (x = 0; x < sw / 2; x++) {
            int sx = x * (f->width / 2) / (sw / 2);

            line[2 * x] = f->data[1][sr * f->linesize[1] + sx];
            line[2 * x + 1] = f->data[2][sr * f->linesize[2] + sx];
        }
        dma_memory_write(&address_space_memory, uv + (hwaddr)r * sw, line,
                         sw & ~1, MEMTXATTRS_UNSPECIFIED);
    }
}

static void nvt_h26xdec_decode(NvtH26xDecState *s)
{
    uint32_t bs[3];
    AVCodecContext *ctx;

    address_space_read(&address_space_memory, *reg(s, H264_BS_DESC),
                       MEMTXATTRS_UNSPECIFIED, bs, sizeof(bs));
    bs[1] = le32_to_cpu(bs[1]);
    bs[2] = le32_to_cpu(bs[2]);
    if (!bs[2] || bs[2] > BS_MAX) {
        return;
    }
    ctx = nvt_h26xdec_ctx(s, *reg(s, H264_SIZE));
    if (!ctx) {
        return;
    }
    if (av_new_packet(s->pkt, bs[2]) < 0) {
        return;
    }
    address_space_read(&address_space_memory, bs[1], MEMTXATTRS_UNSPECIFIED,
                       s->pkt->data, bs[2]);
    if (avcodec_send_packet(ctx, s->pkt) >= 0) {
        while (avcodec_receive_frame(ctx, s->frame) >= 0) {
            nvt_h26xdec_put_picture(s, s->frame);
            nvt_h26xdec_put_sub(s, s->frame);
            av_frame_unref(s->frame);
        }
    }
    av_packet_unref(s->pkt);
}
#endif

/* A register write from the list, with the effect it has on the core. */
static void nvt_h26xdec_ll_write(NvtH26xDecState *s, uint32_t off,
                                 uint32_t val)
{
    if (off >= LL_BASE) {
        return;
    }
    if (off == H264_INT_FLG) {
        *reg(s, off) &= ~val;               /* write 1 to clear, here too */
        return;
    }
    if (off == H264_CTRL || off == BIN_CTRL) {
        /*
         * A reset finishes at once and leaves the block idle; it wins over
         * the start bit VPU_ResetHW()'s read-modify-write carries along.
         */
        if (val & CTRL_RESET) {
            *reg(s, off) = 0;
            return;
        }
    }
    *reg(s, off) = val;
    if (off == H264_CTRL && (val & CTRL_START)) {
        if (s->trace) {
            static const uint32_t regs[] = { 0x1018, 0x1024, 0x1028, 0x102C };
            int i;

            for (i = 0; i < ARRAY_SIZE(regs); i++) {
                char what[16];

                snprintf(what, sizeof(what), "[%04x]", regs[i]);
                nvt_h26xdec_dump(s, what, *reg(s, regs[i]));
            }
        }
        if (s->trace) {
            uint32_t bs[3];

            address_space_read(&address_space_memory, *reg(s, 0x102C),
                               MEMTXATTRS_UNSPECIFIED, bs, sizeof(bs));
            qemu_log("nvt-h26xdec: job %u bs %u x %08x+%u\n", s->jobs,
                     bs[0], bs[1], bs[2]);
            nvt_h26xdec_dump(s, "bs", bs[1]);
        }
#ifdef NVT_H26XDEC_LIBAVCODEC
        nvt_h26xdec_decode(s);
#endif
        *reg(s, H264_INT_FLG) |= H264_INT_DONE;
    }
}

static void nvt_h26xdec_run_ll(NvtH26xDecState *s)
{
    hwaddr addr = *reg(s, LL_ADDR);
    uint32_t cmd[2];
    int n;

    for (n = 0; n < LL_MAX_CMDS; n++, addr += 8) {
        if (address_space_read(&address_space_memory, addr,
                               MEMTXATTRS_UNSPECIFIED, cmd, sizeof(cmd))) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-h26xdec: list at 0x%"
                          HWADDR_PRIx " unreadable\n", addr);
            return;
        }
        cmd[0] = le32_to_cpu(cmd[0]);
        cmd[1] = le32_to_cpu(cmd[1]);
        if (s->trace) {
            qemu_log("nvt-h26xdec: job %u ll[%d] %08x %08x\n", s->jobs, n,
                     cmd[0], cmd[1]);
        }
        *reg(s, LL_LAST_CMD) = cmd[0];
        *reg(s, LL_LAST_VAL) = cmd[1];
        switch (cmd[0] >> 28) {
        case LL_OP_WRITE:
            nvt_h26xdec_ll_write(s, cmd[0] & 0x7FFFC, cmd[1]);
            break;
        case LL_OP_WAIT:
            break;                          /* everything is done at once */
        case LL_OP_LINK:
            if (cmd[1] == LL_FINAL) {
                return;
            }
            addr = (hwaddr)cmd[1] - 8;
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "nvt-h26xdec: list command %08x\n",
                          cmd[0]);
            break;
        }
    }
}

static void nvt_h26xdec_done(void *opaque)
{
    NvtH26xDecState *s = opaque;

    *reg(s, LL_LAST_VAL) = LL_FINAL;
    *reg(s, AUX_INT_STS) |= 1;
    nvt_h26xdec_update_irq(s);
}

static uint64_t nvt_h26xdec_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtH26xDecState *s = opaque;
    uint32_t val = *reg(s, offset & ~3);

    /* Resets and decodes finish at once: the control words read done. */
    if ((offset & ~3) == H264_CTRL || (offset & ~3) == BIN_CTRL) {
        val |= CTRL_DONE;
    }

    if (s->trace) {
        qemu_log("nvt-h26xdec: r %04x = %08x\n", (unsigned)offset, val);
    }
    return val;
}

static void nvt_h26xdec_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    NvtH26xDecState *s = opaque;
    uint32_t val = value;

    offset &= ~3;
    if (s->trace) {
        qemu_log("nvt-h26xdec: w %04x = %08x\n", (unsigned)offset, val);
    }
    switch (offset) {
    case AUX_INT_STS:
    case CORE_INT_STS:
    case H264_INT_FLG:
        *reg(s, offset) &= ~val;
        nvt_h26xdec_update_irq(s);
        return;
    case H264_CTRL:
    case BIN_CTRL:
        nvt_h26xdec_ll_write(s, offset, val);
        return;
    case LL_CTRL:
        *reg(s, offset) = val;
        if (val & LL_CTRL_RESET) {
            *reg(s, LL_RESET_STS) |= LL_RESET_DONE;
        } else {
            *reg(s, LL_RESET_STS) &= ~LL_RESET_DONE;
        }
        if (val & LL_CTRL_FIRE) {
            nvt_h26xdec_run_ll(s);
            s->jobs++;
            timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      2 * SCALE_MS);
        }
        return;
    }
    if (offset == AUX_INT_ACK) {
        if (val & 1) {
            *reg(s, AUX_INT_STS) &= ~1;
        }
        nvt_h26xdec_update_irq(s);
        return;
    }
    *reg(s, offset) = val;
    if (offset == CORE_INT_MASK) {
        nvt_h26xdec_update_irq(s);
    }
}

static const MemoryRegionOps nvt_h26xdec_ops = {
    .read = nvt_h26xdec_read,
    .write = nvt_h26xdec_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void nvt_h26xdec_reset(DeviceState *dev)
{
    NvtH26xDecState *s = NVT_H26XDEC(dev);

    timer_del(s->done);
    memset(s->regs, 0, sizeof(s->regs));
    s->jobs = 0;
    qemu_irq_lower(s->irq);
}

static void nvt_h26xdec_init(Object *obj)
{
    NvtH26xDecState *s = NVT_H26XDEC(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_h26xdec_ops, s,
                          TYPE_NVT_H26XDEC, DEC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_h26xdec_done, s);
#ifdef NVT_H26XDEC_LIBAVCODEC
    s->pkt = av_packet_alloc();
    s->frame = av_frame_alloc();
#endif
}

static const Property nvt_h26xdec_properties[] = {
    DEFINE_PROP_BOOL("trace", NvtH26xDecState, trace, false),
};

static const VMStateDescription vmstate_nvt_h26xdec = {
    .name = TYPE_NVT_H26XDEC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtH26xDecState, DEC_SIZE / 4),
        VMSTATE_UINT32(jobs, NvtH26xDecState),
        VMSTATE_TIMER_PTR(done, NvtH26xDecState),
        VMSTATE_END_OF_LIST()
    },
};

static void nvt_h26xdec_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_h26xdec_reset);
    device_class_set_props(dc, nvt_h26xdec_properties);
    dc->vmsd = &vmstate_nvt_h26xdec;
}

static const TypeInfo nvt_h26xdec_info = {
    .name           = TYPE_NVT_H26XDEC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(NvtH26xDecState),
    .instance_init  = nvt_h26xdec_init,
    .class_init     = nvt_h26xdec_class_init,
};

static void nvt_h26xdec_register_types(void)
{
    type_register_static(&nvt_h26xdec_info);
}

type_init(nvt_h26xdec_register_types)
