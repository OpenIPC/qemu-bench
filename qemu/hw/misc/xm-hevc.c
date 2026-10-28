/*
 * Xiongmai second-generation HEVC encoder engine (0x21000000, 64 KiB, and
 * 0x21010000, 256 bytes), as the vendor hevc.ko and the jc_v200 library
 * statically linked into Sofia drive it.
 *
 * hevc.ko is a shim: the library hands it register lists, it writes them,
 * starts the engine and sleeps until hevc_interrupt (SPI 45) wakes it.
 * What a job is, read from the registers the library programs (logged at
 * run time; there is no public source):
 *
 *   +0x000     bit 0: start (the kernel writes 0 when it is done)
 *   +0x018     interrupt acknowledge: write 1s to clear +0x074 bits
 *   +0x044/48  encoded width - 1 / height - 1
 *   +0x054     slice type: 0 I (an IDR), 1 P
 *   +0x058     table select, the table's words at +0x800..  (rate control)
 *   +0x05C     slice QP
 *   +0x088     bytes free at the output pointer
 *   +0x0A0/A4  source luma / chroma (NV12), physical
 *   +0x0A8/AC  the reference picture, luma / chroma: what the last job of
 *              the stream wrote to +0x0B0/B4, its reconstruction
 *   +0x0D0     output pointer (the stream buffer + 256)
 *   +0x0D8/DC  source luma / chroma strides
 *   +0x0F8     bytes the job wrote (hevc_interrupt's stream length)
 *   +0x0FC     1 main stream, 0 sub stream
 *   +0x100/104 source width - 1 / height - 1: the engine scales, which is
 *              how a 720p sensor gets a "1080P" stream
 *   +0x074     status: bit 0 frame done (3 takes the plain success path)
 *
 *   +0x11C     regions to overlay, a bit each (Sofia's main-stream clock and
 *              title; the sub stream's go through the OSD engine).  Region n
 *              at +0x120 + 0x20 * n: +0x00 x0, y0, x1, y1 (bytes, from the
 *              top, in 16-pixel units, the end exclusive), +0x04 enable,
 *              +0x08 bit 0 / bit 2: the bitmap doubled across / down,
 *              +0x0C..+0x18 the four colours (A Y U V, 0 clear), +0x1C the
 *              bitmap: 2 bits a pixel in 16x16 tiles of 64 bytes, row by
 *              row, a tile's rows 4 bytes each, most significant pixel
 *              first.  Blended into the source before it is encoded.
 *
 * +0x068 / +0x06C / +0x070 (errors, line mode) read 0.  Every other word is
 * RAM-backed.  A job starts a millisecond after the start bit and is
 * encoded on a worker thread; it completes when the encoder is done, so a
 * big picture takes (virtual) time as on the hardware.  The library
 * writes the parameter sets and the slice header itself, in the 256 bytes
 * before the output pointer, and the engine only the slice data after them:
 * so the job writes slice_segment_data() (hevc-enc.c), coded against the
 * library's slice header.  A P job skips the CUs whose source is the
 * reference's: the engine keeps each decoded picture (on the host, not in
 * the guest's reconstruction buffer, whose format is the hardware's) under
 * the reconstruction address, and the next P job of the stream names it as
 * its reference.  Without one (a reset, a picture of another size) the P
 * picture is coded all intra.
 *
 * layout "xm533" (XM530V300) is a different engine behind the same
 * library: hevc.ko writes registers 2..188 from the list and 1 to +0x004;
 *
 *   +0x034/38  encoded width - 1 / height - 1 (the source is that size too)
 *   +0x040     slice type: 0 I, 1 P          +0x044  slice QP
 *   +0x1D4     bytes free at the output pointer
 *   +0x1D8     bytes the job wrote
 *   +0x1E0/E4  source luma / chroma (NV12)   +0x224/228  their strides
 *   +0x1E8/EC  the reference picture          +0x1F0/F4  the reconstruction
 *   +0x210     output pointer (the stream buffer + 256)
 *   (layout "xm530v200", its earlier revision: the output pointer at
 *   +0x200, the source strides at +0x214 / +0x218; its library codes
 *   32x32 CTBs with SAO, so the slice data does too)
 *   +0x058     status, 3 frame done; 3 to +0x030 acknowledges.  +0x054
 *              (acknowledged at +0x02C) and +0x050 (errors, at +0x028)
 *              also raise the interrupt; neither happens here.
 *
 * Its parameter sets and slice headers are field for field those of the
 * XM535 library, so the slice data is the same.
 *
 * dump-dir (-global xm-hevc.dump-dir=DIR) keeps every job on the host as
 * frame-NNNNN.hevc, a whole access unit with headers equal to the
 * library's, main and sub stream jobs alike.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/xm-regtrace.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "qom/object.h"
#include "system/dma.h"
#include "hevc-enc.h"

#define TYPE_XM_HEVC "xm-hevc"
OBJECT_DECLARE_SIMPLE_TYPE(XmHevcState, XM_HEVC)

#define XM_HEVC_SIZE    0x10000
#define XM_HEVC2_SIZE   0x100

#define R_START         0x000
#define R_ACK           0x018
#define R_ENC_W         0x044
#define R_ENC_H         0x048
#define R_SLICE_TYPE    0x054
#define R_QP            0x05C
#define R_STATUS        0x074
#define R_OUT_FREE      0x088
#define R_SRC_Y         0x0A0
#define R_SRC_C         0x0A4
#define R_OUT           0x0D0
#define R_SRC_YS        0x0D8
#define R_SRC_CS        0x0DC
#define R_OUT_LEN       0x0F8
#define R_MAIN          0x0FC
#define R_SRC_W         0x100
#define R_SRC_H         0x104
#define R_OSD_MASK      0x11C
#define R_OSD(n)        (0x120 + 0x20 * (n))
#define OSD_REGIONS     4

#define ST_DONE         3

/* the widest source line a job may name: twice the widest picture */
#define XM_HEVC_MAX_STRIDE  16384

/* decoded pictures kept for P jobs: one a stream is enough (D2: six) */
#define XM_HEVC_REFS    8

/* XM530V300 (XM533): another engine.  hevc.ko's handle_bgn writes the
 * library's register list (registers 2..188) and then 1 to +0x004; its
 * hevc_interrupt reads +0x058 (3: frame done; the stream length at
 * +0x1D8), acknowledges with 3 to +0x030, and checks +0x054 (acknowledged
 * at +0x02C) and +0x050 (errors). */
#define V3_START        0x004
#define V3_ERR          0x050
#define V3_ST2          0x054
#define V3_STATUS       0x058
#define V3_ACK2         0x02C
#define V3_ACK          0x030
#define V3_ENC_W        0x034
#define V3_ENC_H        0x038
#define V3_SLICE_TYPE   0x040   /* 0 I (an IDR), 1 P */
#define V3_QP           0x044
#define V3_OUT_FREE     0x1D4
#define V3_OUT_LEN      0x1D8
#define V3_SRC_Y        0x1E0   /* NV12, at the encoded size */
#define V3_SRC_C        0x1E4

struct XmHevcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem, iomem2;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[XM_HEVC_SIZE / 4];
    uint32_t regs2[XM_HEVC2_SIZE / 4];
    char *dump_dir;
    char *layout;
    bool v3;                    /* layout "xm533" or "xm530v200" */
    uint32_t v3_out, v3_ys, v3_cs;  /* where they differ */
    const HevcEncParams *prm;   /* what the library's headers say */
    uint32_t ref_off, recon_off;    /* the reference / reconstruction luma */
    uint32_t jobs;
    uint32_t poc[2];            /* of the dumps, per stream */
    /* the worker's: decoded pictures by the reconstruction buffer the
     * engine wrote them to, which a P job names as its reference */
    struct {
        uint32_t pa;
        HevcEncRef *pic;
        uint64_t used;
    } refs[XM_HEVC_REFS];
    HevcEncRef *spare;
    uint64_t ref_clock;
    uint32_t ref_gen;
    /* encoding runs on a worker thread, the guest carries on meanwhile */
    GThreadPool *pool;
    GAsyncQueue *done;
    QEMUBH *done_bh;
    uint32_t gen;               /* bumped by reset: stale jobs are dropped */
};

typedef struct XmHevcJob {
    XmHevcState *s;
    uint32_t gen;
    int w, h, sw, sh, ys, cs, qp, poc;
    bool p_slice;
    uint8_t *y, *uv, *out;
    uint32_t out_pa, cap;
    uint32_t ref_pa, recon_pa;
    size_t n;
    uint32_t len_reg;
    char *dump_path;
} XmHevcJob;

#define R(s, off) ((s)->regs[(off) / 4])

static void xm_hevc_update(XmHevcState *s)
{
    if (s->v3) {
        qemu_set_irq(s->irq, ((R(s, V3_STATUS) | R(s, V3_ST2) |
                               R(s, V3_ERR)) & 3) != 0);
        return;
    }
    qemu_set_irq(s->irq, R(s, R_STATUS) != 0);
}

static uint8_t osd_blend(uint8_t bg, uint8_t fg, unsigned a)
{
    return bg + ((int)fg - bg) * (int)a / 255;
}

/* the engine's overlay regions, into the source (w x h encoded from
 * sw x sh: output pixels mapped back to the source's) */
static void xm_hevc_osd(XmHevcState *s, uint8_t *y, uint8_t *uv, int w,
                        int h, int sw, int sh, int ys, int cs)
{
    for (int n = 0; n < OSD_REGIONS; n++) {
        const uint32_t *r = &s->regs[R_OSD(n) / 4];
        const uint32_t pos = r[0], fmt = r[2];
        const int x0 = (pos >> 24) * 16, y0 = ((pos >> 16) & 0xff) * 16;
        const int x1 = MIN(((pos >> 8) & 0xff) * 16, w);
        const int y1 = MIN((pos & 0xff) * 16, h);
        const int sx2 = fmt & 1, sy2 = (fmt >> 2) & 1;
        const int tpr = (((x1 - x0) >> sx2) + 15) / 16;
        const int bh = (((y1 - y0) >> sy2) + 15) / 16 * 16;

        if (!(R(s, R_OSD_MASK) & (1u << n)) || !r[1] || !r[7] ||
            x1 <= x0 || y1 <= y0) {
            continue;
        }
        const size_t len = (size_t)tpr * (bh / 16) * 64;
        g_autofree uint8_t *bmp = g_malloc(len);

        dma_memory_read(&address_space_memory, r[7], bmp, len,
                        MEMTXATTRS_UNSPECIFIED);
        for (int oy = y0; oy < y1; oy++) {
            const int by = (oy - y0) >> sy2;
            const int sy = oy * sh / h;

            for (int ox = x0; ox < x1; ox++) {
                const int bx = (ox - x0) >> sx2;
                const uint8_t b = bmp[((by / 16) * tpr + bx / 16) * 64 +
                                      (by % 16) * 4 + (bx % 16) / 4];
                const uint32_t c = r[3 + ((b >> (6 - 2 * (bx % 4))) & 3)];
                const int sx = ox * sw / w;

                if (!(c >> 24)) {
                    continue;
                }
                y[(size_t)sy * ys + sx] = osd_blend(y[(size_t)sy * ys + sx],
                                                    c >> 16, c >> 24);
                if (!(sy & 1)) {
                    uint8_t *p = &uv[(size_t)(sy / 2) * cs + (sx & ~1)];

                    p[0] = osd_blend(p[0], c >> 8, c >> 24);
                    p[1] = osd_blend(p[1], c, c >> 24);
                }
            }
        }
    }
}

/* encode w x h from NV12 at y / uv (strides ys / cs) into out_pa, at most
 * cap bytes: the slice data under the library's own headers */
/* the worker: the slice data, and the dump if asked for */
static void xm_hevc_work(gpointer data, gpointer user_data)
{
    XmHevcJob *j = data;
    XmHevcState *s = j->s;
    const HevcEncRef *ref = NULL;
    int i, slot = 0;

    if (j->gen != s->ref_gen) {             /* reset: forget them all */
        for (i = 0; i < XM_HEVC_REFS; i++) {
            hevc_enc_ref_free(s->refs[i].pic);
            s->refs[i].pic = NULL;
            s->refs[i].pa = 0;
        }
        s->ref_gen = j->gen;
    }
    for (i = 0; i < XM_HEVC_REFS; i++) {
        if (s->refs[i].pic && s->refs[i].pa == j->ref_pa && j->p_slice) {
            ref = s->refs[i].pic;
        }
    }
    if (!s->spare) {
        s->spare = hevc_enc_ref_new();
    }
    j->n = hevc_enc_slice_data(s->prm, j->w, j->h, j->qp, j->p_slice, ref,
                               j->recon_pa ? s->spare : NULL, j->y, j->sw,
                               j->sh, j->ys, j->uv, j->cs, j->out, j->cap);
    if (j->dump_path) {
        const size_t dcap = MIN(j->n, j->cap) + 256;
        g_autofree uint8_t *au = g_malloc(dcap);
        const size_t dn = hevc_enc_frame(s->prm, j->w, j->h, j->qp,
                                         j->p_slice, j->poc, ref, NULL,
                                         j->y, j->sw,
                                         j->sh, j->ys, j->uv, j->cs, au, dcap);
        g_file_set_contents(j->dump_path, (const char *)au, MIN(dn, dcap),
                            NULL);
    }
    if (j->recon_pa) {
        /* keep the picture under its buffer, in the slot that held that
         * buffer or the least recently used one */
        for (i = 0; i < XM_HEVC_REFS; i++) {
            if (s->refs[i].pic && s->refs[i].pa == j->recon_pa) {
                slot = i;
                break;
            }
            if (!s->refs[i].pic ||
                (s->refs[slot].pic && s->refs[i].used < s->refs[slot].used)) {
                slot = i;
            }
        }
        HevcEncRef *old = s->refs[slot].pic;
        s->refs[slot].pic = s->spare;
        s->refs[slot].pa = j->recon_pa;
        s->refs[slot].used = ++s->ref_clock;
        s->spare = old;
    }
    g_async_queue_push(s->done, j);
    qemu_bh_schedule(s->done_bh);
}

static void xm_hevc_job_free(XmHevcJob *j)
{
    g_free(j->y);
    g_free(j->uv);
    g_free(j->out);
    g_free(j->dump_path);
    g_free(j);
}

/* back in the main loop: the stream into guest memory, then done */
static void xm_hevc_done(void *opaque)
{
    XmHevcState *s = XM_HEVC(opaque);
    XmHevcJob *j;

    while ((j = g_async_queue_try_pop(s->done))) {
        if (j->gen == s->gen) {
            size_t n = j->n;

            if (n > j->cap) {
                qemu_log_mask(LOG_GUEST_ERROR, "xm-hevc: %zu bytes over the "
                              "%u-byte buffer\n", n, j->cap);
                n = j->cap;
            }
            dma_memory_write(&address_space_memory, j->out_pa, j->out, n,
                             MEMTXATTRS_UNSPECIFIED);
            s->regs[j->len_reg / 4] = n;
            if (s->v3) {
                R(s, V3_STATUS) = ST_DONE;
            } else {
                R(s, R_STATUS) = ST_DONE;
            }
            xm_hevc_update(s);
        }
        xm_hevc_job_free(j);
    }
}

/* a job: its source out of guest memory, then to the worker; a job the
 * engine cannot do completes at once with nothing written */
static void xm_hevc_encode(XmHevcState *s, int w, int h, int sw, int sh,
                           int ys, int cs, uint32_t y_pa, uint32_t uv_pa,
                           uint32_t out_pa, uint32_t cap, bool p_slice,
                           int qp, int stream, uint32_t len_reg)
{
    if (w < 8 || h < 8 || w > 8192 || h > 8192 || (w & 7) || (h & 7) ||
        sw < 2 || sh < 2 || sw > 8192 || sh > 8192 || ys < sw || cs < sw ||
        ys > XM_HEVC_MAX_STRIDE || cs > XM_HEVC_MAX_STRIDE ||
        !y_pa || !uv_pa || !out_pa || !cap) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-hevc: job %dx%d from %dx%d "
                      "(strides %d/%d) to 0x%x+%u not done\n", w, h, sw, sh,
                      ys, cs, out_pa, cap);
        s->regs[len_reg / 4] = 0;
        if (s->v3) {
            R(s, V3_STATUS) = ST_DONE;
        } else {
            R(s, R_STATUS) = ST_DONE;
        }
        xm_hevc_update(s);
        return;
    }
    XmHevcJob *j = g_new0(XmHevcJob, 1);
    uint32_t *poc = &s->poc[stream & 1];

    *poc = p_slice ? *poc + 1 : 0;
    *j = (XmHevcJob){ .s = s, .gen = s->gen, .w = w, .h = h, .sw = sw,
                      .sh = sh, .ys = ys, .cs = cs, .qp = qp, .poc = *poc,
                      .p_slice = p_slice, .out_pa = out_pa, .cap = cap,
                      .ref_pa = s->regs[s->ref_off / 4],
                      .recon_pa = s->regs[s->recon_off / 4],
                      .len_reg = len_reg };
    j->y = g_malloc((size_t)ys * sh);
    j->uv = g_malloc((size_t)cs * (sh / 2));
    /* no more than the slice data can take: the guest's free count can be
     * anything */
    j->cap = cap = MIN(cap, (uint32_t)((w / 8) * (h / 8) * 80 + 4096));
    j->out = g_malloc(cap);
    dma_memory_read(&address_space_memory, y_pa, j->y, (size_t)ys * sh,
                    MEMTXATTRS_UNSPECIFIED);
    dma_memory_read(&address_space_memory, uv_pa, j->uv,
                    (size_t)cs * (sh / 2), MEMTXATTRS_UNSPECIFIED);
    if (!s->v3) {
        xm_hevc_osd(s, j->y, j->uv, w, h, sw, sh, ys, cs);
    }
    if (s->dump_dir) {
        j->dump_path = g_strdup_printf("%s/frame-%05u.hevc", s->dump_dir,
                                       s->jobs);
    }
    s->jobs++;
    g_thread_pool_push(s->pool, j, NULL);
}

static void xm_hevc_v3_job(XmHevcState *s)
{
    const int w = R(s, V3_ENC_W) + 1, h = R(s, V3_ENC_H) + 1;

    xm_hevc_encode(s, w, h, w, h, s->regs[s->v3_ys / 4],
                   s->regs[s->v3_cs / 4], R(s, V3_SRC_Y), R(s, V3_SRC_C),
                   s->regs[s->v3_out / 4], R(s, V3_OUT_FREE),
                   R(s, V3_SLICE_TYPE) == 1, MIN(R(s, V3_QP), 51), w >= 1280,
                   V3_OUT_LEN);
}

static void xm_hevc_job(void *opaque)
{
    XmHevcState *s = XM_HEVC(opaque);

    if (s->v3) {
        xm_hevc_v3_job(s);
        return;
    }
    const int w = R(s, R_ENC_W) + 1, h = R(s, R_ENC_H) + 1;
    const int sw = R(s, R_SRC_W) + 1, sh = R(s, R_SRC_H) + 1;

    xm_hevc_encode(s, w, h, sw, sh, R(s, R_SRC_YS) ? R(s, R_SRC_YS) : sw,
                   R(s, R_SRC_CS) ? R(s, R_SRC_CS) : sw, R(s, R_SRC_Y),
                   R(s, R_SRC_C), R(s, R_OUT), R(s, R_OUT_FREE),
                   R(s, R_SLICE_TYPE) == 1, MIN(R(s, R_QP), 51),
                   R(s, R_MAIN), R_OUT_LEN);
}

static uint64_t xm_hevc_read(void *opaque, hwaddr off, unsigned size)
{
    XmHevcState *s = XM_HEVC(opaque);

    if (s->v3) {
        return s->regs[off / 4];
    }
    switch (off) {
    case 0x068:
    case 0x06C:
    case 0x070:
        return 0;
    default:
        return s->regs[off / 4];
    }
}

static void xm_hevc_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    XmHevcState *s = XM_HEVC(opaque);

    xm_regtrace("hevc", off, val);
    if (s->v3) {
        switch (off) {
        case V3_START:
            s->regs[off / 4] = val;
            if (val & 1) {
                timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          SCALE_MS);
            }
            break;
        case V3_ACK:
            R(s, V3_STATUS) &= ~(uint32_t)val;
            break;
        case V3_ACK2:
            R(s, V3_ST2) &= ~(uint32_t)val;
            break;
        case 0x028:
            R(s, V3_ERR) &= ~(uint32_t)val;
            break;
        case V3_STATUS:
        case V3_ST2:
        case V3_ERR:
        case V3_OUT_LEN:
            break;
        default:
            s->regs[off / 4] = val;
            break;
        }
        xm_hevc_update(s);
        return;
    }
    switch (off) {
    case R_START:
        s->regs[off / 4] = val;
        if (val & 1) {
            R(s, R_STATUS) = 0;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      SCALE_MS);
        }
        break;
    case R_ACK:
        s->regs[off / 4] = val;
        R(s, R_STATUS) &= ~(uint32_t)val;
        break;
    default:
        s->regs[off / 4] = val;
        break;
    }
    xm_hevc_update(s);
}

static const MemoryRegionOps xm_hevc_ops = {
    .read = xm_hevc_read,
    .write = xm_hevc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t xm_hevc2_read(void *opaque, hwaddr off, unsigned size)
{
    return XM_HEVC(opaque)->regs2[off / 4];
}

static void xm_hevc2_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    xm_regtrace("hevc2", off, val);
    XM_HEVC(opaque)->regs2[off / 4] = val;
}

static const MemoryRegionOps xm_hevc2_ops = {
    .read = xm_hevc2_read,
    .write = xm_hevc2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_hevc_reset(DeviceState *dev)
{
    XmHevcState *s = XM_HEVC(dev);

    timer_del(s->timer);
    s->gen++;
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->regs2, 0, sizeof(s->regs2));
    xm_hevc_update(s);
}

static void xm_hevc_init(Object *obj)
{
    XmHevcState *s = XM_HEVC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &xm_hevc_ops, s, TYPE_XM_HEVC,
                          XM_HEVC_SIZE);
    memory_region_init_io(&s->iomem2, obj, &xm_hevc2_ops, s,
                          TYPE_XM_HEVC ".2", XM_HEVC2_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_mmio(sbd, &s->iomem2);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_hevc_job, s);
    s->done = g_async_queue_new();
    s->done_bh = qemu_bh_new(xm_hevc_done, s);
    /* one worker: jobs complete in order, as the engine's would */
    s->pool = g_thread_pool_new(xm_hevc_work, NULL, 1, FALSE, NULL);
}

static void xm_hevc_realize(DeviceState *dev, Error **errp)
{
    XmHevcState *s = XM_HEVC(dev);

    s->prm = &hevc_params_ctb16;
    s->ref_off = 0x1E8;
    s->recon_off = 0x1F0;
    if (!s->layout || !strcmp(s->layout, "xm535")) {
        s->v3 = false;
        s->ref_off = 0x0A8;
        s->recon_off = 0x0B0;
    } else if (!strcmp(s->layout, "xm533")) {
        s->v3 = true;
        s->v3_out = 0x210;
        s->v3_ys = 0x224;
        s->v3_cs = 0x228;
    } else if (!strcmp(s->layout, "xm530v200")) {
        s->v3 = true;
        s->v3_out = 0x200;
        s->v3_ys = 0x214;
        s->v3_cs = 0x218;
        s->prm = &hevc_params_ctb32_sao;
    } else {
        error_setg(errp, "xm-hevc: unknown layout '%s'", s->layout);
    }
}

static const Property xm_hevc_props[] = {
    DEFINE_PROP_STRING("dump-dir", XmHevcState, dump_dir),
    DEFINE_PROP_STRING("layout", XmHevcState, layout),
};

static void xm_hevc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, xm_hevc_reset);
    dc->realize = xm_hevc_realize;
    device_class_set_props(dc, xm_hevc_props);
}

static const TypeInfo xm_hevc_info = {
    .name          = TYPE_XM_HEVC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmHevcState),
    .instance_init = xm_hevc_init,
    .class_init    = xm_hevc_class_init,
};

static void xm_hevc_register_types(void)
{
    type_register_static(&xm_hevc_info);
}

type_init(xm_hevc_register_types)
