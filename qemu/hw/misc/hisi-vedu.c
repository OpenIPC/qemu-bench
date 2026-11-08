/*
 * HiSilicon V4 video encoder: VEDU (H.264 / H.265) and JPGE.
 *
 * Hi3516EV200 / EV300 / Hi3518EV300 / Hi3516DV200 and the Goke
 * GK7205V200 family carry VEDU at 0x11410000 (SPI 47) and JPGE at
 * 0x11420000 (SPI 48).  The register contract below comes from the
 * HiMPP V4 driver objects (hi_vedu.o, hi_h264e.o; the VEDU_HAL_* and
 * h264e_drv_* functions) and from tracing that driver under this model
 * (-global hisi-vedu.trace=on); docs/hisilicon-venc.md has the details.
 *
 * VEDU MMIO:
 *   0x0000  INTMASK   bit 0 end of picture, ...
 *   0x0004  INTCLR    write 1s to clear
 *   0x0008  START     1 starts a picture
 *   0x0014  PIC_SIZE  (width - 1) | (height - 1) << 16
 *   0x004C  RESET     reset request
 *   0x0058  REG_IMAGE physical address of the register image
 *   0x3000  INTSTAT   masked status: 0 end of picture, 2 buffer full,
 *                     3 bits over, 6 timeout
 *   0x3004  RAWINT    raw status (11 ring full, 12 ring back)
 *   0x3010  BUS_IDLE  bit 0: idle (the driver polls it around a reset)
 *
 * The driver programs only a few MMIO registers per picture.  The rest
 * of the configuration is a register image in DDR, laid out like the
 * MMIO space, which the engine fetches at START from REG_IMAGE.  Of it:
 *
 *   0x104 / 0x108  source luma / chroma (NV21, PIXEL_FORMAT_YVU_SEMIPLANAR_420)
 *   0x1f4          source strides: luma | chroma << 16 (0x160 / 0x170
 *                  are the reconstruction / reference pictures', aligned
 *                  to 64: the same only while the width is)
 *   0x12c          address of the stream ring's write-pointer cell
 *   0x130          address of its read-pointer cell
 *   0x16c          stream ring base;  0x4a0  its size
 *   0x61c          slice QP (low 6 bits)
 *   0x4a4 / 0x4a8  PTS
 *   0x1ec          compression header address of a compressed source
 *                  (hisi-vpss's half-height stand-in, see there)
 *
 * The stream ring holds packets, each a 64-byte header and its payload
 * padded to 64 bytes:
 *
 *   +0   u32  packet length, header included
 *   +4   u32  padding bytes at the payload's end
 *   +8   u8   NAL type (H264E_NALU_*: 1 P, 5 IDR, 6 SEI, 7 SPS, 8 PPS)
 *   +11  u8   bit 0 last slice of the picture, bits 7:6 slice type
 *             (0 I, 1 P, 2 B)
 *   +12  u32  channel
 *   +16  u64  PTS
 *
 * The driver writes the parameter-set packets itself, advancing the
 * write-pointer cell; the engine appends the picture's slice there and
 * leaves the cell at its end.  The model reads the parameter sets back
 * out of those packets and codes a slice that fits them: H.264 from
 * hantro-h264.c under a slice header the model writes, H.265 from
 * hevc-enc.c under the driver's own header bits (image 0x404..0x42c,
 * first_slice_segment_in_pic_flag and byte_alignment() added here).
 *
 * JPGE takes its job over MMIO (JPGE_* below); the driver puts the JFIF
 * header in the same kind of ring and the engine appends the scan and EOI
 * (jpeg-enc.c) as a JPEGE_PACK_ECS packet.
 *
 * Both complete a millisecond of virtual time after START, never inside
 * the write: the drivers arm their wait after it.
 *
 * dump-dir (-global hisi-vedu.dump-dir=DIR) appends every channel's
 * stream, the driver's packets and the model's, to DIR/vedu-<n>.h264 /
 * .h265 / .mjpeg, as the guest's GetStream returns it.  trace=on logs
 * every register access and the register image at START.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hantro-h264.h"
#include "hevc-enc.h"
#include "jpeg-enc.h"

#define TYPE_HISI_VEDU "hisi-vedu"
OBJECT_DECLARE_SIMPLE_TYPE(HisiVeduState, HISI_VEDU)

#define HISI_VEDU_REG_SIZE  0x10000
#define HISI_VEDU_REG_COUNT (HISI_VEDU_REG_SIZE / 4)

#define VEDU_INTMASK        0x0000
#define VEDU_INTCLR         0x0004
#define VEDU_START          0x0008
#define VEDU_PIC_SIZE       0x0014
#define VEDU_RESET          0x004C
#define VEDU_REG_IMAGE      0x0058
#define VEDU_INTSTAT        0x3000
#define VEDU_RAWINT         0x3004
#define VEDU_BUS_IDLE       0x3010

#define VEDU_INT_EOP        (1u << 0)
#define VEDU_INT_BUF_FULL   (1u << 2)

/* register image offsets */
#define IMG_SRC_Y           0x104
#define IMG_SRC_C           0x108
#define IMG_STRM_WP         0x12c
#define IMG_STRM_RP         0x130
#define IMG_SRC_STRIDE      0x1f4
#define IMG_SRC_CMP_HDR     0x1ec   /* compressed source: its header address */
#define IMG_STRM_BASE       0x16c
#define IMG_STRM_SIZE       0x4a0
#define IMG_SLICE_QP        0x61c
#define IMG_SLC_HDR_LEN     0x404   /* H.265: part 1 bits | part 2 << 16 */
#define IMG_SLC_HDR_PART1   0x408   /* left-aligned in the low 16 bits */
#define IMG_SLC_HDR_PART2   0x40c   /* big-endian words, eight at most */
#define IMG_PTS_LO          0x4a4
#define IMG_PTS_HI          0x4a8
#define IMG_WORDS           (0x1000 / 4)

/* JPGE MMIO */
#define JPGE_INTSTAT        0x0000  /* 0 done, 1 error, 5 ring full, 7 back */
#define JPGE_INTEN          0x0004
#define JPGE_RAWINT         0x0008
#define JPGE_INTCLR         0x000c
#define JPGE_START          0x002c
#define JPGE_PIC_SIZE       0x0034  /* (width - 1) | (height - 1) << 16 */
#define JPGE_STRM_SIZE      0x0060
#define JPGE_RESET          0x0068
#define JPGE_SRC_STRIDE     0x0164  /* luma | chroma << 16 */
#define JPGE_SRC_Y          0x0200
#define JPGE_SRC_C          0x0204
#define JPGE_STRM_BASE      0x0210
#define JPGE_STRM_RP        0x0214  /* address of the read-pointer cell */
#define JPGE_STRM_WP        0x0218  /* address of the write-pointer cell */
#define JPGE_QUANT_Y        0x0800  /* 64 reciprocals, 2^23 / q, by column */
#define JPGE_QUANT_CB       0x0900
#define JPGE_QUANT_CR       0x0a00
#define JPGE_BUS_IDLE       0x0c0c

#define JPGE_INT_DONE       (1u << 0)
#define JPGE_INT_ERROR      (1u << 1)
#define JPGE_INT_RING_FULL  (1u << 5)
#define JPGE_PACK_ECS       5

#define PKT_HDR             64
#define PKT_MAX_READ        (64 * KiB)  /* parameter sets, JFIF headers */
#define RING_MAX            (64 * MiB)
#define PIC_MAX             4096        /* V4 encodes 2304 x 1296 at most */
#define VEDU_DONE_NS        (1 * SCALE_MS)
#define VEDU_MAX_CHN        8

/* what the H.265 slice data has to follow, from the driver's SPS / PPS */
typedef struct HevcHdrs {
    bool have_sps, have_pps, unsupported;
    int w, h, log2_max_poc_lsb, log2_ctb, trafo_inter, trafo_intra;
    bool chroma, scaling_list, sao, lt_present, sps_tmvp;
    int num_lt_sps, num_st_rps, st_num_delta[65];
    bool output_flag, cabac_init_present, sign_hiding, transform_skip;
    bool cu_qp_delta, slice_chroma_qp_offsets, weighted_pred, lists_mod;
    bool dbk_override, dbk_disabled;
    int num_extra_bits, init_qp, diff_cu_qp_depth, cb_qp_offset;
    int beta_div2, tc_div2, num_ref_idx_l0;
} HevcHdrs;

enum { CODEC_NONE, CODEC_H264, CODEC_H265, CODEC_JPEG };

typedef struct VeduChannel {
    uint32_t ring;              /* stream ring base: the channel's key */
    uint32_t scanned;           /* ring offset the model has read up to */
    uint32_t chn;               /* channel id the driver's packets carry */
    uint64_t pts;               /* of the driver's last packet */
    int codec;
    H264Enc h264;
    HevcHdrs hevc;
    HevcEncRef *hevc_ref, *hevc_spare;
    int frame_num, poc_lsb, idr_pic_id;
    int mbw, mbh;
    uint8_t *ref_y, *ref_uv;    /* decoder's picture, NV12 at mbw * 16 */
    bool have_ref;
    unsigned index;             /* for dump file names */
} VeduChannel;

struct HisiVeduState {
    SysBusDevice parent_obj;
    MemoryRegion vedu_iomem;
    MemoryRegion jpge_iomem;
    qemu_irq vedu_irq;
    qemu_irq jpge_irq;
    QEMUTimer *vedu_timer;
    QEMUTimer *jpge_timer;
    uint32_t vedu_regs[HISI_VEDU_REG_COUNT];
    uint32_t jpge_regs[HISI_VEDU_REG_COUNT];
    uint32_t vedu_raw;
    uint32_t jpge_raw;
    uint32_t img[IMG_WORDS];
    VeduChannel chn[VEDU_MAX_CHN];
    unsigned nchn;
    bool trace;
    char *dump_dir;
};

/* ------------------------------------------------------------------ */
/* guest memory and the stream ring */

static void vedu_dma_read(hwaddr a, void *buf, size_t n)
{
    dma_memory_read(&address_space_memory, a, buf, n, MEMTXATTRS_UNSPECIFIED);
}

static void vedu_dma_write(hwaddr a, const void *buf, size_t n)
{
    dma_memory_write(&address_space_memory, a, buf, n,
                     MEMTXATTRS_UNSPECIFIED);
}

static uint32_t vedu_ldl(hwaddr a)
{
    uint32_t v;

    vedu_dma_read(a, &v, 4);
    return le32_to_cpu(v);
}

static void vedu_stl(hwaddr a, uint32_t v)
{
    v = cpu_to_le32(v);
    vedu_dma_write(a, &v, 4);
}

/* n bytes at ring offset off, wrapping at size */
static void ring_read(uint32_t base, uint32_t size, uint32_t off,
                      void *buf, uint32_t n)
{
    uint32_t first;

    off %= size;
    first = MIN(n, size - off);
    vedu_dma_read(base + off, buf, first);
    if (n > first) {
        vedu_dma_read(base, (uint8_t *)buf + first, n - first);
    }
}

static void ring_write(uint32_t base, uint32_t size, uint32_t off,
                       const void *buf, uint32_t n)
{
    uint32_t first;

    off %= size;
    first = MIN(n, size - off);
    vedu_dma_write(base + off, buf, first);
    if (n > first) {
        vedu_dma_write(base, (const uint8_t *)buf + first, n - first);
    }
}

static void vedu_dump(HisiVeduState *s, VeduChannel *c, const uint8_t *buf,
                      size_t n)
{
    g_autofree char *path = NULL;
    FILE *f;

    if (!s->dump_dir) {
        return;
    }
    path = g_strdup_printf("%s/vedu-%u.%s", s->dump_dir, c->index,
                           c->codec == CODEC_H265 ? "h265" :
                           c->codec == CODEC_H264 ? "h264" : "mjpeg");
    if (c->codec == CODEC_NONE) {
        return;
    }
    f = fopen(path, "ab");
    if (!f) {
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-vedu: cannot write %s\n", path);
        return;
    }
    fwrite(buf, 1, n, f);
    fclose(f);
}

static void hevc_parse_nal(HevcHdrs *h, const uint8_t *p, size_t n);

static void vedu_channel_free(VeduChannel *c)
{
    g_free(c->ref_y);
    g_free(c->ref_uv);
    hevc_enc_ref_free(c->hevc_ref);
    hevc_enc_ref_free(c->hevc_spare);
}

/* a channel the driver destroyed and created again on the same ring */
static void vedu_channel_reset(VeduChannel *c, int codec)
{
    const uint32_t ring = c->ring;
    const unsigned index = c->index;

    vedu_channel_free(c);
    memset(c, 0, sizeof(*c));
    c->ring = ring;
    c->index = index;
    c->codec = codec;
}

/*
 * Walk the driver's packets from off to the engine's start: parameter
 * sets to fit the slice to, the channel id and PTS.  False when they do
 * not lead there (a ring the driver started over).
 */
static bool vedu_walk(HisiVeduState *s, VeduChannel *c, uint32_t base,
                      uint32_t size, uint32_t off, uint32_t start, bool *sps)
{
    uint32_t walked = 0;

    while (off != start) {
        uint32_t hdr[PKT_HDR / 4], len, pad, payload;
        g_autofree uint8_t *p = NULL;
        uint8_t type;

        ring_read(base, size, off, hdr, sizeof(hdr));
        len = le32_to_cpu(hdr[0]);
        pad = le32_to_cpu(hdr[1]);
        type = le32_to_cpu(hdr[2]) & 0xff;
        if (len < PKT_HDR || len % 64 || len > size - walked ||
            pad > len - PKT_HDR) {
            return false;
        }
        payload = len - PKT_HDR - pad;
        c->chn = le32_to_cpu(hdr[3]);
        c->pts = le32_to_cpu(hdr[4]) | (uint64_t)le32_to_cpu(hdr[5]) << 32;
        if (payload <= PKT_MAX_READ) {
            p = g_malloc(payload);
            ring_read(base, size, off + PKT_HDR, p, payload);
            if (c->codec == CODEC_JPEG) {
                /* JPEGE_PACK_*: the header the scan goes under */
            } else if (type == 7 || type == 8) {
                c->codec = CODEC_H264;
                h264_enc_parse_headers(&c->h264, p, payload);
                *sps |= type == 7;
            } else if (type >= 32 && type <= 34) {
                c->codec = CODEC_H265;
                hevc_parse_nal(&c->hevc, p, payload);
                *sps |= type == 32;
            }
            vedu_dump(s, c, p, payload);
        } else {
            qemu_log_mask(LOG_UNIMP, "hisi-vedu: %u-byte packet of type %u "
                          "skipped\n", payload, type);
        }
        off = (off + len) % size;
        walked += len;
    }
    return true;
}

/*
 * The driver's packets between what the model has read and the engine's
 * start.  Returns true when the picture is an IDR (an SPS came with it).
 */
static bool vedu_scan_packets(HisiVeduState *s, VeduChannel *c,
                              uint32_t base, uint32_t size, uint32_t start)
{
    bool sps = false;

    if (!vedu_walk(s, c, base, size, c->scanned, start, &sps)) {
        vedu_channel_reset(c, c->codec == CODEC_JPEG ? CODEC_JPEG
                                                     : CODEC_NONE);
        sps = false;
        if (!vedu_walk(s, c, base, size, 0, start, &sps)) {
            qemu_log_mask(LOG_GUEST_ERROR, "hisi-vedu: no packets lead to "
                          "ring offset 0x%x\n", start);
        }
    }
    c->scanned = start;
    return sps;
}

static VeduChannel *vedu_channel(HisiVeduState *s, uint32_t ring, bool jpeg)
{
    VeduChannel *c;
    unsigned i;

    for (i = 0; i < s->nchn; i++) {
        c = &s->chn[i];
        if (c->ring == ring) {
            if (jpeg != (c->codec == CODEC_JPEG)) {
                vedu_channel_reset(c, jpeg ? CODEC_JPEG : CODEC_NONE);
            }
            return c;
        }
    }
    if (s->nchn == VEDU_MAX_CHN) {
        /* a channel was destroyed and its ring reused elsewhere */
        vedu_channel_free(&s->chn[0]);
        memmove(&s->chn[0], &s->chn[1], sizeof(s->chn[0]) * --s->nchn);
    }
    c = &s->chn[s->nchn];
    memset(c, 0, sizeof(*c));
    c->ring = ring;
    c->index = s->nchn++;
    c->codec = jpeg ? CODEC_JPEG : CODEC_NONE;
    return c;
}

/*
 * Room for need bytes at the write pointer wp before the read pointer rp
 * (offsets in a ring of size; rp == wp: empty).  No read-pointer cell:
 * the whole ring.
 */
static uint32_t ring_room(uint32_t size, uint32_t wp, hwaddr rp_cell)
{
    const uint32_t rp = rp_cell ? vedu_ldl(rp_cell) % size : wp;

    return size - (wp + size - rp) % size - 1;
}

/* ------------------------------------------------------------------ */
/* H.264 slice: header written here, data from hantro-h264.c */

typedef struct BitWriter {
    uint8_t *buf;
    size_t cap, bits;
} BitWriter;

static void bw_bits(BitWriter *w, uint32_t v, int n)
{
    while (n--) {
        size_t byte = w->bits / 8;

        if (byte < w->cap) {
            if ((v >> n) & 1) {
                w->buf[byte] |= 0x80 >> (w->bits % 8);
            } else {
                w->buf[byte] &= ~(0x80 >> (w->bits % 8));
            }
        }
        w->bits++;
    }
}

static void bw_ue(BitWriter *w, uint32_t v)
{
    int len = 32 - clz32(v + 1);

    bw_bits(w, 0, len - 1);
    bw_bits(w, v + 1, len);
}

static void bw_se(BitWriter *w, int32_t v)
{
    bw_ue(w, v > 0 ? 2 * v - 1 : -2 * v);
}

/* RBSP to a byte-stream NAL: start code, header, emulation prevention */
static size_t nal_wrap(uint8_t hdr, const uint8_t *rbsp, size_t n,
                       uint8_t *out)
{
    size_t i, o = 0;
    int zeros = 0;

    out[o++] = 0;
    out[o++] = 0;
    out[o++] = 0;
    out[o++] = 1;
    out[o++] = hdr;
    for (i = 0; i < n; i++) {
        if (zeros >= 2 && rbsp[i] <= 3) {
            out[o++] = 3;
            zeros = 0;
        }
        out[o++] = rbsp[i];
        zeros = rbsp[i] ? 0 : zeros + 1;
    }
    return o;
}

/*
 * NV21 / NV12 source into an NV12 picture padded to whole macroblocks.
 * half: a compressed source in hisi-vpss's stand-in, every other row
 * stored (see hisi-vpss.c), which is doubled back here.
 */
static void vedu_load_nv12(uint32_t ya, uint32_t ca, int ys, int cs,
                           int w, int h, bool vu, int mbw, int mbh,
                           uint8_t *y, uint8_t *uv, bool half)
{
    const int pw = mbw * 16, ph = mbh * 16, step = half ? 2 : 1;
    g_autofree uint8_t *line = g_malloc(MAX(w, 1));
    int r, x;

    for (r = 0; r < ph; r++) {
        vedu_dma_read(ya + (uint64_t)(MIN(r, h - 1) / step) * ys, line, w);
        memcpy(y + r * pw, line, w);
        memset(y + r * pw + w, line[w - 1], pw - w);
    }
    for (r = 0; r < ph / 2; r++) {
        uint8_t *d = uv + r * pw;

        vedu_dma_read(ca + (uint64_t)(MIN(r, h / 2 - 1) / step) * cs, line, w);
        for (x = 0; x < pw / 2; x++) {
            int sx = MIN(x, w / 2 - 1) * 2;

            d[x * 2] = line[sx + (vu ? 1 : 0)];
            d[x * 2 + 1] = line[sx + (vu ? 0 : 1)];
        }
    }
}

/*
 * One picture as one slice NAL into out (byte stream, start code
 * included).  Returns its length, 0 if the headers seen do not allow it.
 */
static size_t vedu_h264_picture(VeduChannel *c, bool *idr_io, int qp, int w,
                                int h, const uint8_t *y, const uint8_t *uv,
                                uint8_t *out, size_t cap)
{
    H264Enc *e = &c->h264;
    const int mbw = DIV_ROUND_UP(w, 16), mbh = DIV_ROUND_UP(h, 16);
    bool idr = *idr_io;
    const bool p = !idr && c->have_ref && c->mbw == mbw && c->mbh == mbh;
    H264Deblock db = { 0, 0, 0 };
    g_autofree uint8_t *rbsp = g_malloc0(cap);
    BitWriter bw = { rbsp, cap, 0 };
    H264Pic *pic;
    size_t n;

    if (!e->have_sps || !e->have_pps || e->unsupported) {
        return 0;
    }
    if (!p) {
        idr = true;     /* nothing to refer to: the picture is an IDR */
    }
    *idr_io = idr;
    if (idr) {
        c->frame_num = 0;
        c->poc_lsb = 0;
    }

    /* slice_header() */
    bw_ue(&bw, 0);                              /* first_mb_in_slice */
    bw_ue(&bw, p ? 5 : 7);                      /* P / I, all slices */
    bw_ue(&bw, e->pps_id);
    bw_bits(&bw, c->frame_num, e->log2_max_frame_num);
    if (idr) {
        bw_ue(&bw, c->idr_pic_id);
    }
    if (e->poc_type == 0) {
        bw_bits(&bw, c->poc_lsb, e->log2_max_poc_lsb);
        if (e->bottom_field_poc_present) {
            bw_se(&bw, 0);
        }
    }
    if (e->redundant_pic_cnt_present) {
        bw_ue(&bw, 0);
    }
    if (p) {
        bw_bits(&bw, 0, 1);         /* num_ref_idx_active_override_flag */
        bw_bits(&bw, 0, 1);         /* ref_pic_list_modification_flag_l0 */
    }
    if (idr) {
        bw_bits(&bw, 0, 1);         /* no_output_of_prior_pics_flag */
        bw_bits(&bw, 0, 1);         /* long_term_reference_flag */
    } else {
        bw_bits(&bw, 0, 1);         /* adaptive_ref_pic_marking_mode_flag */
    }
    if (e->cabac && p) {
        bw_ue(&bw, 0);              /* cabac_init_idc */
    }
    bw_se(&bw, qp - e->pic_init_qp);
    if (e->deblocking_control) {
        bw_ue(&bw, 0);              /* disable_deblocking_filter_idc */
        bw_se(&bw, 0);
        bw_se(&bw, 0);
    }

    if (bw.bits / 8 + 256 > cap) {
        return 0;                       /* no room for any slice data */
    }
    if (c->mbw != mbw || c->mbh != mbh) {
        g_free(c->ref_y);
        g_free(c->ref_uv);
        c->ref_y = g_malloc0(mbw * mbh * 256);
        c->ref_uv = g_malloc0(mbw * mbh * 128);
        c->mbw = mbw;
        c->mbh = mbh;
    }
    pic = h264_pic_new(mbw, mbh, y, mbw * 16, uv, mbw * 16,
                       p ? c->ref_y : NULL, p ? c->ref_uv : NULL,
                       e->chroma_qp_offset, false);
    if (e->cabac) {
        while (bw.bits % 8) {
            bw_bits(&bw, 1, 1);     /* cabac_alignment_one_bit */
        }
        n = h264_pic_slice_cabac(pic, p, 0, mbh, qp, 0, rbsp + bw.bits / 8,
                                 cap - bw.bits / 8, NULL);
        n += bw.bits / 8;
    } else {
        /* the coder starts mid-byte but does not keep the bits before */
        const uint8_t keep = 0xff00 >> (bw.bits % 8);
        const uint8_t hdr_bits = rbsp[bw.bits / 8] & keep;

        n = h264_pic_slice_cavlc(pic, p, 0, mbh, qp, bw.bits % 8,
                                 rbsp + bw.bits / 8, cap - bw.bits / 8, NULL);
        rbsp[bw.bits / 8] = hdr_bits | (rbsp[bw.bits / 8] & ~keep);
        n = (bw.bits / 8 * 8 + n + 7) / 8;
    }
    h264_pic_finish(pic, &db, c->ref_y, c->ref_uv);
    h264_pic_free(pic);
    c->have_ref = true;

    if (n * 4 / 3 + 8 > cap) {
        return 0;
    }
    n = nal_wrap(idr ? 0x65 : 0x41, rbsp, n, out);

    if (idr) {
        c->idr_pic_id = (c->idr_pic_id + 1) & 0xffff;
    }
    c->frame_num = (c->frame_num + 1) & ((1 << e->log2_max_frame_num) - 1);
    if (e->poc_type == 0) {
        c->poc_lsb = (c->poc_lsb + 2) & ((1 << e->log2_max_poc_lsb) - 1);
    }
    return n;
}


/* ------------------------------------------------------------------ */
/* H.265: the driver's parameter sets and slice header bits */

typedef struct BitReader {
    const uint8_t *p;
    size_t n, pos;              /* in bits */
    bool err;
} BitReader;

static uint32_t br_bits(BitReader *r, int n)
{
    uint32_t v = 0;

    while (n--) {
        if (r->pos >= r->n) {
            r->err = true;
            return v;
        }
        v = (v << 1) | ((r->p[r->pos / 8] >> (7 - r->pos % 8)) & 1);
        r->pos++;
    }
    return v;
}

static uint32_t br_ue(BitReader *r)
{
    int zeros = 0;

    while (!br_bits(r, 1) && !r->err && zeros < 32) {
        zeros++;
    }
    return ((1u << zeros) - 1) + br_bits(r, zeros);
}

static int32_t br_se(BitReader *r)
{
    const uint32_t k = br_ue(r);

    return k & 1 ? (int32_t)((k + 1) / 2) : -(int32_t)(k / 2);
}

static int ceil_log2(int v)
{
    int n = 0;

    while ((1 << n) < v) {
        n++;
    }
    return n;
}

/* NAL payload (after the 2-byte header) without emulation prevention */
static size_t rbsp_unescape(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t i, o = 0;
    int zeros = 0;

    for (i = 0; i < n; i++) {
        if (zeros >= 2 && in[i] == 3) {
            zeros = 0;
            continue;
        }
        zeros = in[i] ? 0 : zeros + 1;
        out[o++] = in[i];
    }
    return o;
}

static void hevc_ptl(BitReader *r, int max_sub)
{
    bool prof[8], level[8];
    int i;

    br_bits(r, 32);
    br_bits(r, 32);
    br_bits(r, 24);             /* general profile, 88 bits */
    br_bits(r, 8);              /* general_level_idc */
    for (i = 0; i < max_sub; i++) {
        prof[i] = br_bits(r, 1);
        level[i] = br_bits(r, 1);
    }
    if (max_sub > 0) {
        for (i = max_sub; i < 8; i++) {
            br_bits(r, 2);
        }
    }
    for (i = 0; i < max_sub; i++) {
        if (prof[i]) {
            br_bits(r, 32);
            br_bits(r, 32);
            br_bits(r, 24);
        }
        if (level[i]) {
            br_bits(r, 8);
        }
    }
}

/* st_ref_pic_set(idx); returns its NumDeltaPocs */
static int hevc_st_rps(BitReader *r, HevcHdrs *h, int idx)
{
    int i, n = 0;

    if (idx && br_bits(r, 1)) {         /* inter_ref_pic_set_prediction */
        int ref = idx - 1;

        if (idx == h->num_st_rps) {
            ref = idx - 1 - (int)br_ue(r);
        }
        br_bits(r, 1);
        br_ue(r);
        if (ref < 0 || ref > 64) {
            h->unsupported = true;
            return 0;
        }
        for (i = 0; i <= h->st_num_delta[ref]; i++) {
            if (br_bits(r, 1) || br_bits(r, 1)) {   /* used / use_delta */
                n++;
            }
        }
        return n;
    }
    {
        const int neg = br_ue(r), pos = br_ue(r);

        for (i = 0; i < neg + pos && !r->err; i++) {
            br_ue(r);
            br_bits(r, 1);
        }
        return neg + pos;
    }
}

static void hevc_parse_sps(HevcHdrs *h, const uint8_t *b, size_t n)
{
    BitReader r = { b, n * 8 };
    int max_sub, i, log2_min_cb;

    br_bits(&r, 4);
    max_sub = br_bits(&r, 3);
    br_bits(&r, 1);
    hevc_ptl(&r, max_sub);
    br_ue(&r);                                  /* sps_seq_parameter_set_id */
    i = br_ue(&r);                              /* chroma_format_idc */
    if (i == 3) {
        br_bits(&r, 1);
    }
    h->chroma = i != 0;
    if (i != 1) {
        h->unsupported = true;
    }
    h->w = br_ue(&r);
    h->h = br_ue(&r);
    if (br_bits(&r, 1)) {                       /* conformance_window */
        br_ue(&r); br_ue(&r); br_ue(&r); br_ue(&r);
    }
    if (br_ue(&r) || br_ue(&r)) {               /* bit depths beyond 8 */
        h->unsupported = true;
    }
    h->log2_max_poc_lsb = br_ue(&r) + 4;
    for (i = br_bits(&r, 1) ? 0 : max_sub; i <= max_sub; i++) {
        br_ue(&r); br_ue(&r); br_ue(&r);
    }
    log2_min_cb = br_ue(&r) + 3;
    h->log2_ctb = log2_min_cb + br_ue(&r);
    br_ue(&r);
    br_ue(&r);
    h->trafo_inter = br_ue(&r);
    h->trafo_intra = br_ue(&r);
    h->scaling_list = br_bits(&r, 1);
    if (h->scaling_list && br_bits(&r, 1)) {    /* lists other than default */
        h->unsupported = true;
    }
    br_bits(&r, 1);                             /* amp */
    h->sao = br_bits(&r, 1);
    if (br_bits(&r, 1)) {                       /* pcm */
        br_bits(&r, 8);
        br_ue(&r);
        br_ue(&r);
        br_bits(&r, 1);
    }
    h->num_st_rps = br_ue(&r);
    if (h->num_st_rps > 64) {
        h->unsupported = true;
        return;
    }
    for (i = 0; i < h->num_st_rps; i++) {
        h->st_num_delta[i] = hevc_st_rps(&r, h, i);
    }
    h->lt_present = br_bits(&r, 1);
    if (h->lt_present) {
        h->num_lt_sps = br_ue(&r);
        for (i = 0; i < h->num_lt_sps; i++) {
            br_bits(&r, h->log2_max_poc_lsb);
            br_bits(&r, 1);
        }
    }
    h->sps_tmvp = br_bits(&r, 1);
    if (h->log2_ctb < 4 || h->log2_ctb > 6 || log2_min_cb != 3) {
        h->unsupported = true;
    }
    h->have_sps = !r.err;
}

static void hevc_parse_pps(HevcHdrs *h, const uint8_t *b, size_t n)
{
    BitReader r = { b, n * 8 };

    br_ue(&r);
    br_ue(&r);
    if (br_bits(&r, 1)) {                       /* dependent slices */
        h->unsupported = true;
    }
    h->output_flag = br_bits(&r, 1);
    h->num_extra_bits = br_bits(&r, 3);
    if (h->num_extra_bits) {
        /* slice_reserved_flags would sit before slice_type, which is
         * where the driver's part 2 starts */
        h->unsupported = true;
    }
    h->sign_hiding = br_bits(&r, 1);
    h->cabac_init_present = br_bits(&r, 1);
    h->num_ref_idx_l0 = br_ue(&r) + 1;
    br_ue(&r);
    h->init_qp = 26 + br_se(&r);
    br_bits(&r, 1);                             /* constrained intra */
    h->transform_skip = br_bits(&r, 1);
    h->cu_qp_delta = br_bits(&r, 1);
    h->diff_cu_qp_depth = h->cu_qp_delta ? br_ue(&r) : 0;
    h->cb_qp_offset = br_se(&r);
    br_se(&r);
    h->slice_chroma_qp_offsets = br_bits(&r, 1);
    h->weighted_pred = br_bits(&r, 1);
    br_bits(&r, 1);
    if (br_bits(&r, 1)) {                       /* transquant bypass */
        h->unsupported = true;
    }
    if (br_bits(&r, 1) | br_bits(&r, 1)) {      /* tiles, WPP */
        h->unsupported = true;
    }
    br_bits(&r, 1);                             /* loop filter across */
    h->dbk_override = h->dbk_disabled = false;
    h->beta_div2 = h->tc_div2 = 0;
    if (br_bits(&r, 1)) {                       /* deblocking control */
        h->dbk_override = br_bits(&r, 1);
        h->dbk_disabled = br_bits(&r, 1);
        if (!h->dbk_disabled) {
            h->beta_div2 = br_se(&r);
            h->tc_div2 = br_se(&r);
        }
    }
    if (br_bits(&r, 1)) {                       /* PPS scaling lists */
        h->unsupported = true;
    }
    h->lists_mod = br_bits(&r, 1);
    h->have_pps = !r.err;
}

static void hevc_parse_nal(HevcHdrs *h, const uint8_t *p, size_t n)
{
    size_t i = 0;

    while (i + 5 < n) {
        size_t j, len;
        int type;
        g_autofree uint8_t *rbsp = NULL;

        if (p[i] || p[i + 1] || p[i + 2] != 1) {
            i++;
            continue;
        }
        i += 3;
        for (j = i; j + 2 < n && !(!p[j] && !p[j + 1] && p[j + 2] <= 1); j++) {
        }
        if (j + 2 >= n) {
            j = n;
        }
        if (j < i + 2) {
            continue;                   /* no room for a NAL header */
        }
        type = (p[i] >> 1) & 0x3f;
        rbsp = g_malloc(j - i);
        len = rbsp_unescape(p + i + 2, j - i - 2, rbsp);
        if (type == 33) {
            hevc_parse_sps(h, rbsp, len);
        } else if (type == 34) {
            hevc_parse_pps(h, rbsp, len);
        }
        i = j;
    }
}

/* the slice-level choices in the driver's slice header (part 2) */
typedef struct HevcSlice {
    int type, qp, max_merge, beta_div2, tc_div2;
    bool sao_luma, sao_chroma, tmvp, cabac_init, dbk_disabled;
} HevcSlice;

static bool hevc_parse_slice(const HevcHdrs *h, bool irap, const uint8_t *b,
                             int nbits, HevcSlice *sl)
{
    BitReader r = { b, nbits };
    HevcHdrs tmp;
    int i;

    memset(sl, 0, sizeof(*sl));
    sl->type = br_ue(&r);
    if (h->output_flag) {
        br_bits(&r, 1);
    }
    if (!irap) {
        br_bits(&r, h->log2_max_poc_lsb);
        if (!br_bits(&r, 1)) {                  /* short_term_ref_pic_set_sps */
            tmp = *h;
            hevc_st_rps(&r, &tmp, h->num_st_rps);
        } else if (h->num_st_rps > 1) {
            br_bits(&r, ceil_log2(h->num_st_rps));
        }
        if (h->lt_present) {
            const int n1 = h->num_lt_sps ? br_ue(&r) : 0;
            const int n2 = br_ue(&r);

            for (i = 0; i < n1 + n2 && !r.err; i++) {
                if (i < n1) {
                    if (h->num_lt_sps > 1) {
                        br_bits(&r, ceil_log2(h->num_lt_sps));
                    }
                } else {
                    br_bits(&r, h->log2_max_poc_lsb);
                    br_bits(&r, 1);
                }
                if (br_bits(&r, 1)) {
                    br_ue(&r);
                }
            }
        }
        if (h->sps_tmvp) {
            sl->tmvp = br_bits(&r, 1);
        }
    }
    if (h->sao) {
        sl->sao_luma = br_bits(&r, 1);
        if (h->chroma) {
            sl->sao_chroma = br_bits(&r, 1);
        }
    }
    sl->max_merge = 5;
    if (sl->type != 2) {
        int nref = h->num_ref_idx_l0;

        if (sl->type == 0 || h->lists_mod || h->weighted_pred) {
            return false;                       /* B slices, lists, weights */
        }
        if (br_bits(&r, 1)) {
            nref = br_ue(&r) + 1;
        }
        if (h->cabac_init_present) {
            sl->cabac_init = br_bits(&r, 1);
        }
        if (sl->tmvp && nref > 1) {
            br_ue(&r);
        }
        sl->max_merge = 5 - br_ue(&r);
    }
    sl->qp = h->init_qp + br_se(&r);
    if (h->slice_chroma_qp_offsets) {
        br_se(&r);
        br_se(&r);
    }
    sl->dbk_disabled = h->dbk_disabled;
    sl->beta_div2 = h->beta_div2;
    sl->tc_div2 = h->tc_div2;
    if (h->dbk_override && br_bits(&r, 1)) {
        sl->dbk_disabled = br_bits(&r, 1);
        if (!sl->dbk_disabled) {
            sl->beta_div2 = br_se(&r);
            sl->tc_div2 = br_se(&r);
        }
    }
    return !r.err && sl->qp >= 0 && sl->qp <= 51;
}

/*
 * One H.265 picture: the driver's slice header bits from the register
 * image, first_slice_segment_in_pic_flag in front (the engine's own) and
 * byte_alignment() behind, then slice data from hevc-enc.c.
 */
static size_t vedu_h265_picture(VeduChannel *c, const uint32_t *img,
                                bool *irap, const uint8_t *y,
                                const uint8_t *uv, int sw, int sh,
                                uint8_t *out, size_t cap)
{
    HevcHdrs *h = &c->hevc;
    const uint32_t len = img[IMG_SLC_HDR_LEN / 4];
    const int n1 = len & 0xffff, n2 = len >> 16;
    uint8_t hdr[64] = { 0 }, part2[32];
    BitWriter bw = { hdr, sizeof(hdr), 0 };
    HevcEncParams prm = { 0 };
    HevcSlice sl;
    size_t n, o;
    int i;

    if (!h->have_sps || !h->have_pps || h->unsupported || n1 < 1 ||
        n1 > 16 || n2 > 256) {
        qemu_log_mask(LOG_UNIMP, "hisi-vedu: H.265 SPS %d PPS %d "
                      "unsupported %d, slice header %d + %d bits\n",
                      h->have_sps, h->have_pps, h->unsupported, n1, n2);
        return 0;
    }
    *irap = n1 == 2;            /* no_output_of_prior_pics_flag before the id */
    for (i = 0; i < 8; i++) {
        stl_be_p(part2 + i * 4, img[IMG_SLC_HDR_PART2 / 4 + i]);
    }
    if (!hevc_parse_slice(h, *irap, part2, n2, &sl)) {
        qemu_log_mask(LOG_UNIMP, "hisi-vedu: H.265 slice header not "
                      "understood\n");
        return 0;
    }

    bw_bits(&bw, 1, 1);                         /* first_slice_segment_in_pic */
    bw_bits(&bw, (img[IMG_SLC_HDR_PART1 / 4] & 0xffff) >> (16 - n1), n1);
    for (i = 0; i < n2; i++) {
        bw_bits(&bw, (part2[i / 8] >> (7 - i % 8)) & 1, 1);
    }
    bw_bits(&bw, 1, 1);                         /* byte_alignment() */
    while (bw.bits % 8) {
        bw_bits(&bw, 0, 1);
    }

    prm.log2_ctb = h->log2_ctb;
    prm.log2_qg = h->cu_qp_delta ? h->log2_ctb - h->diff_cu_qp_depth : 0;
    prm.sao = sl.sao_luma || sl.sao_chroma;
    prm.sao_luma_only = sl.sao_luma && !sl.sao_chroma;
    prm.chroma_qp_offset = h->cb_qp_offset;
    prm.beta_offset_div2 = sl.beta_div2;
    prm.tc_offset_div2 = sl.tc_div2;
    prm.trafo_depth_intra = h->trafo_intra;
    prm.transform_skip = h->transform_skip;
    prm.cabac_init_flag = sl.cabac_init;
    prm.inter = true;
    prm.scaling_list = h->scaling_list;
    prm.tmvp = sl.tmvp;
    prm.trafo_depth_inter = h->trafo_inter;
    prm.max_merge_cand = sl.max_merge;
    prm.sign_hiding = h->sign_hiding;

    if (cap < 256) {
        return 0;
    }
    if (!c->hevc_spare) {
        c->hevc_spare = hevc_enc_ref_new();
    }
    o = 0;
    out[o++] = 0;
    out[o++] = 0;
    out[o++] = 0;
    out[o++] = 1;
    out[o++] = (*irap ? 19 : 1) << 1;
    out[o++] = 1;
    {
        int zeros = 0;

        for (i = 0; i < (int)(bw.bits / 8); i++) {
            if (zeros >= 2 && hdr[i] <= 3) {
                out[o++] = 3;
                zeros = 0;
            }
            out[o++] = hdr[i];
            zeros = hdr[i] ? 0 : zeros + 1;
        }
    }
    n = hevc_enc_slice_data(&prm, h->w, h->h, sl.qp, sl.type == 1,
                            sl.type == 1 ? c->hevc_ref : NULL, c->hevc_spare,
                            y, sw, sh, sw, uv, sw, out + o, cap - o);
    if (!n || n > cap - o) {
        return 0;
    }
    {
        HevcEncRef *t = c->hevc_ref;

        c->hevc_ref = c->hevc_spare;
        c->hevc_spare = t;
    }
    return o + n;
}
/* ------------------------------------------------------------------ */

static void vedu_update_irq(HisiVeduState *s)
{
    s->vedu_regs[VEDU_INTSTAT / 4] = s->vedu_raw & s->vedu_regs[VEDU_INTMASK / 4];
    qemu_set_irq(s->vedu_irq, s->vedu_regs[VEDU_INTSTAT / 4] != 0);
}

/* one picture; returns the raw interrupt bits it ends with */
static uint32_t vedu_encode(HisiVeduState *s)
{
    const uint32_t *img = s->img;
    const uint32_t size_reg = s->vedu_regs[VEDU_PIC_SIZE / 4];
    const int w = (size_reg & 0xffff) + 1, h = (size_reg >> 16) + 1;
    const uint32_t base = img[IMG_STRM_BASE / 4];
    const uint32_t size = img[IMG_STRM_SIZE / 4];
    const uint32_t wp_cell = img[IMG_STRM_WP / 4];
    const size_t mbw = DIV_ROUND_UP(w, 16), mbh = DIV_ROUND_UP(h, 16);
    VeduChannel *c;
    uint32_t start, room, hdr[PKT_HDR / 4] = { 0 }, total;
    uint64_t pts;
    g_autofree uint8_t *y = NULL, *uv = NULL, *nal = NULL;
    size_t want, cap, n;
    bool idr;

    if (!base || size < 4 * PKT_HDR || size > RING_MAX || !wp_cell ||
        w < 16 || h < 16 || w > PIC_MAX || h > PIC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-vedu: picture not set up "
                      "(ring 0x%x+0x%x, %dx%d)\n", base, size, w, h);
        return VEDU_INT_EOP;
    }
    c = vedu_channel(s, base, false);
    start = vedu_ldl(wp_cell) % size;
    idr = vedu_scan_packets(s, c, base, size, start);
    room = ring_room(size, start, img[IMG_STRM_RP / 4]);

    want = mbw * mbh * 512 + 4096;
    cap = room > 2 * PKT_HDR ? MIN(want, (size_t)room - 2 * PKT_HDR) : 0;
    if (cap < 1024) {
        return VEDU_INT_EOP | VEDU_INT_BUF_FULL;
    }

    y = g_malloc(mbw * mbh * 256);
    uv = g_malloc(mbw * mbh * 128);
    vedu_load_nv12(img[IMG_SRC_Y / 4], img[IMG_SRC_C / 4],
                   img[IMG_SRC_STRIDE / 4] & 0xffff,
                   img[IMG_SRC_STRIDE / 4] >> 16, w, h, true, mbw, mbh, y, uv,
                   img[IMG_SRC_CMP_HDR / 4] != 0);

    nal = g_malloc(cap + 64);
    if (c->codec == CODEC_H265) {
        n = vedu_h265_picture(c, img, &idr, y, uv, mbw * 16, mbh * 16,
                              nal, cap);
    } else {
        n = vedu_h264_picture(c, &idr, img[IMG_SLICE_QP / 4] & 0x3f, w, h,
                              y, uv, nal, cap);
    }
    if (!n) {
        if (cap < want) {
            return VEDU_INT_EOP | VEDU_INT_BUF_FULL;
        }
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-vedu: no usable parameter sets "
                      "for channel %u, picture dropped\n", c->chn);
        return VEDU_INT_EOP;
    }

    total = PKT_HDR + ROUND_UP(n, 64);
    pts = img[IMG_PTS_LO / 4] | (uint64_t)img[IMG_PTS_HI / 4] << 32;
    hdr[0] = cpu_to_le32(total);
    hdr[1] = cpu_to_le32(total - PKT_HDR - n);
    hdr[2] = cpu_to_le32((c->codec == CODEC_H265 ? (idr ? 19 : 1)
                                                 : (idr ? 5 : 1)) |
                         (1u << 24) |                  /* last slice */
                         ((idr ? 0u : 1u) << 30));     /* I / P */
    hdr[3] = cpu_to_le32(c->chn);
    hdr[4] = cpu_to_le32(pts);
    hdr[5] = cpu_to_le32(pts >> 32);
    ring_write(base, size, start, hdr, PKT_HDR);
    memset(nal + n, 0, total - PKT_HDR - n);
    ring_write(base, size, start + PKT_HDR, nal, total - PKT_HDR);
    vedu_dump(s, c, nal, n);

    c->scanned = (start + total) % size;
    vedu_stl(wp_cell, c->scanned);
    return VEDU_INT_EOP;
}

static void vedu_done(void *opaque)
{
    HisiVeduState *s = opaque;

    s->vedu_raw |= vedu_encode(s);
    vedu_update_irq(s);
}

static void vedu_start(HisiVeduState *s)
{
    vedu_dma_read(s->vedu_regs[VEDU_REG_IMAGE / 4], s->img, sizeof(s->img));
    if (s->trace) {
        int i;

        fprintf(stderr, "vedu START image @%08x\n",
                s->vedu_regs[VEDU_REG_IMAGE / 4]);
        for (i = 0; i < IMG_WORDS; i += 4) {
            if (s->img[i] | s->img[i + 1] | s->img[i + 2] | s->img[i + 3]) {
                fprintf(stderr, "img %04x: %08x %08x %08x %08x\n", i * 4,
                        s->img[i], s->img[i + 1], s->img[i + 2],
                        s->img[i + 3]);
            }
        }
    }
    timer_mod(s->vedu_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VEDU_DONE_NS);
}

static uint64_t hisi_vedu_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiVeduState *s = HISI_VEDU(opaque);
    uint32_t val;

    switch (offset) {
    case VEDU_INTSTAT:
        val = s->vedu_raw & s->vedu_regs[VEDU_INTMASK / 4];
        break;
    case VEDU_RAWINT:
        val = s->vedu_raw;
        break;
    case VEDU_BUS_IDLE:
        val = timer_pending(s->vedu_timer) ? 0 : 1;
        break;
    default:
        val = s->vedu_regs[offset / 4];
        break;
    }
    if (s->trace) {
        fprintf(stderr, "vedu R %04x = %08x\n", (int)offset, val);
    }
    return val;
}

static void hisi_vedu_write(void *opaque, hwaddr offset,
                            uint64_t val, unsigned size)
{
    HisiVeduState *s = HISI_VEDU(opaque);

    if (s->trace) {
        fprintf(stderr, "vedu W %04x : %08x\n", (int)offset, (uint32_t)val);
    }
    switch (offset) {
    case VEDU_INTCLR:
        s->vedu_raw &= ~(uint32_t)val;
        vedu_update_irq(s);
        return;
    case VEDU_INTMASK:
        s->vedu_regs[offset / 4] = val;
        vedu_update_irq(s);
        return;
    case VEDU_START:
        s->vedu_regs[offset / 4] = val;
        if (val & 1) {
            vedu_start(s);
        }
        return;
    case VEDU_RESET:
        timer_del(s->vedu_timer);
        s->vedu_raw = 0;
        vedu_update_irq(s);
        break;
    }
    s->vedu_regs[offset / 4] = val;
}


/* ------------------------------------------------------------------ */
/* JPGE: the driver writes the JFIF header (SOI .. SOS) as a packet; the
 * engine appends the entropy-coded scan and EOI as a JPEGE_PACK_ECS one */

static void jpge_update_irq(HisiVeduState *s)
{
    s->jpge_regs[JPGE_INTSTAT / 4] = s->jpge_raw & s->jpge_regs[JPGE_INTEN / 4];
    qemu_set_irq(s->jpge_irq, s->jpge_regs[JPGE_INTSTAT / 4] != 0);
}

/* one picture; returns the raw interrupt bits it ends with */
static uint32_t jpge_encode(HisiVeduState *s)
{
    const uint32_t *r = s->jpge_regs;
    const int w = (r[JPGE_PIC_SIZE / 4] & 0xffff) + 1;
    const int h = (r[JPGE_PIC_SIZE / 4] >> 16) + 1;
    const uint32_t base = r[JPGE_STRM_BASE / 4], size = r[JPGE_STRM_SIZE / 4];
    const uint32_t wp_cell = r[JPGE_STRM_WP / 4];
    const size_t mbw = DIV_ROUND_UP(w, 16), mbh = DIV_ROUND_UP(h, 16);
    const size_t pw = mbw * 16, ph = mbh * 16;
    g_autofree uint8_t *y = NULL, *uv = NULL, *cb = NULL, *cr = NULL;
    g_autofree uint8_t *out = NULL;
    uint8_t ql[64], qc[64];
    uint32_t start, room, hdr[PKT_HDR / 4] = { 0 }, total;
    VeduChannel *c;
    bool overflow = false;
    size_t cap, n, i;

    if (!base || size < 4 * PKT_HDR || size > RING_MAX || !wp_cell ||
        w < 8 || h < 8 || w > PIC_MAX || h > PIC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-jpge: picture not set up "
                      "(ring 0x%x+0x%x, %dx%d)\n", base, size, w, h);
        return JPGE_INT_ERROR;
    }
    c = vedu_channel(s, base, true);
    start = vedu_ldl(wp_cell) % size;
    vedu_scan_packets(s, c, base, size, start);
    room = ring_room(size, start, r[JPGE_STRM_RP / 4]);

    for (i = 0; i < 64; i++) {
        /* register i holds row i % 8, column i / 8 */
        const uint32_t ry = r[JPGE_QUANT_Y / 4 + i] ?: 1;
        const uint32_t rc = r[JPGE_QUANT_CB / 4 + i] ?: 1;
        const size_t k = (i % 8) * 8 + i / 8;

        ql[k] = MIN(255, MAX(1, ((1u << 23) + ry / 2) / ry));
        qc[k] = MIN(255, MAX(1, ((1u << 23) + rc / 2) / rc));
    }
    if (memcmp(&r[JPGE_QUANT_CB / 4], &r[JPGE_QUANT_CR / 4], 64 * 4)) {
        /* jpeg-enc.c codes Cb and Cr with one table, as the driver's
         * two-table DQT does */
        qemu_log_mask(LOG_UNIMP, "hisi-jpge: Cr quantisers differ from Cb's; "
                      "Cr coded with Cb's\n");
    }

    cap = room > 2 * PKT_HDR ? MIN(pw * ph * 2 + 1024,
                                   (size_t)room - 2 * PKT_HDR) : 0;
    if (cap < 1024) {
        return JPGE_INT_ERROR | JPGE_INT_RING_FULL;
    }

    y = g_malloc(pw * ph);
    uv = g_malloc(pw * ph / 2);
    cb = g_malloc(pw * ph / 4);
    cr = g_malloc(pw * ph / 4);
    vedu_load_nv12(r[JPGE_SRC_Y / 4], r[JPGE_SRC_C / 4],
                   r[JPGE_SRC_STRIDE / 4] & 0xffff,
                   r[JPGE_SRC_STRIDE / 4] >> 16, w, h, true, mbw, mbh, y, uv,
                   false);
    for (i = 0; i < pw * ph / 4; i++) {
        cb[i] = uv[i * 2];
        cr[i] = uv[i * 2 + 1];
    }

    out = g_malloc(cap + 64);
    n = jpeg_enc_scan(y, pw, cb, cr, pw / 2, w, h, ql, qc, true, out, cap,
                      &overflow);
    if (overflow) {
        return JPGE_INT_ERROR | JPGE_INT_RING_FULL;
    }

    total = PKT_HDR + ROUND_UP(n, 64);
    hdr[0] = cpu_to_le32(total);
    hdr[1] = cpu_to_le32(total - PKT_HDR - n);
    hdr[2] = cpu_to_le32(JPGE_PACK_ECS | (1u << 24));     /* last */
    hdr[3] = cpu_to_le32(c->chn);
    hdr[4] = cpu_to_le32(c->pts);
    hdr[5] = cpu_to_le32(c->pts >> 32);
    ring_write(base, size, start, hdr, PKT_HDR);
    memset(out + n, 0, total - PKT_HDR - n);
    ring_write(base, size, start + PKT_HDR, out, total - PKT_HDR);
    vedu_dump(s, c, out, n);

    c->scanned = (start + total) % size;
    vedu_stl(wp_cell, c->scanned);
    return JPGE_INT_DONE;
}

static void jpge_done(void *opaque)
{
    HisiVeduState *s = opaque;

    s->jpge_raw |= jpge_encode(s);
    jpge_update_irq(s);
}

static uint64_t hisi_jpge_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiVeduState *s = HISI_VEDU(opaque);
    uint32_t val;

    switch (offset) {
    case JPGE_INTSTAT:
        val = s->jpge_raw & s->jpge_regs[JPGE_INTEN / 4];
        break;
    case JPGE_RAWINT:
        val = s->jpge_raw;
        break;
    case JPGE_BUS_IDLE:
        val = timer_pending(s->jpge_timer) ? 0 : 1;
        break;
    default:
        val = s->jpge_regs[offset / 4];
        break;
    }
    if (s->trace) {
        fprintf(stderr, "jpge R %04x = %08x\n", (int)offset, val);
    }
    return val;
}

static void hisi_jpge_write(void *opaque, hwaddr offset,
                            uint64_t val, unsigned size)
{
    HisiVeduState *s = HISI_VEDU(opaque);

    if (s->trace) {
        fprintf(stderr, "jpge W %04x : %08x\n", (int)offset, (uint32_t)val);
    }
    switch (offset) {
    case JPGE_INTCLR:
        s->jpge_raw &= ~(uint32_t)val;
        jpge_update_irq(s);
        return;
    case JPGE_INTEN:
        s->jpge_regs[offset / 4] = val;
        jpge_update_irq(s);
        return;
    case JPGE_START:
        s->jpge_regs[offset / 4] = val;
        if (val & 1) {
            timer_mod(s->jpge_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VEDU_DONE_NS);
        }
        return;
    case JPGE_RESET:
        if (val & 1) {
            timer_del(s->jpge_timer);
            s->jpge_raw = 0;
            jpge_update_irq(s);
        }
        break;
    }
    s->jpge_regs[offset / 4] = val;
}

static const MemoryRegionOps hisi_vedu_ops = {
    .read = hisi_vedu_read,
    .write = hisi_vedu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps hisi_jpge_ops = {
    .read = hisi_jpge_read,
    .write = hisi_jpge_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_vedu_init(Object *obj)
{
    HisiVeduState *s = HISI_VEDU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->vedu_iomem, obj, &hisi_vedu_ops, s,
                          "hisi-vedu", HISI_VEDU_REG_SIZE);
    sysbus_init_mmio(sbd, &s->vedu_iomem);

    memory_region_init_io(&s->jpge_iomem, obj, &hisi_jpge_ops, s,
                          "hisi-jpge", HISI_VEDU_REG_SIZE);
    sysbus_init_mmio(sbd, &s->jpge_iomem);

    sysbus_init_irq(sbd, &s->vedu_irq);
    sysbus_init_irq(sbd, &s->jpge_irq);
    s->vedu_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vedu_done, s);
    s->jpge_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, jpge_done, s);
}

static void hisi_vedu_reset(DeviceState *dev)
{
    HisiVeduState *s = HISI_VEDU(dev);
    unsigned i;

    timer_del(s->vedu_timer);
    timer_del(s->jpge_timer);
    memset(s->vedu_regs, 0, sizeof(s->vedu_regs));
    memset(s->jpge_regs, 0, sizeof(s->jpge_regs));
    s->vedu_raw = 0;
    s->jpge_raw = 0;
    for (i = 0; i < s->nchn; i++) {
        vedu_channel_free(&s->chn[i]);
    }
    memset(s->chn, 0, sizeof(s->chn));
    s->nchn = 0;
}

static const Property hisi_vedu_properties[] = {
    DEFINE_PROP_BOOL("trace", HisiVeduState, trace, false),
    DEFINE_PROP_STRING("dump-dir", HisiVeduState, dump_dir),
};

static void hisi_vedu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, hisi_vedu_properties);
    device_class_set_legacy_reset(dc, hisi_vedu_reset);
}

static const TypeInfo hisi_vedu_info = {
    .name          = TYPE_HISI_VEDU,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiVeduState),
    .instance_init = hisi_vedu_init,
    .class_init    = hisi_vedu_class_init,
};

static void hisi_vedu_register_types(void)
{
    type_register_static(&hisi_vedu_info);
}

type_init(hisi_vedu_register_types)
