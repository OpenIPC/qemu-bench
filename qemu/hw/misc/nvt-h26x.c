/*
 * Novatek NA51089 H.26x encoder: the pictures.
 *
 * The vendor driver (kdrv_h26x, nvt_vdocdc: closed objects) writes every
 * parameter set and every slice header itself.  It hands the header bytes
 * to the engine through the "BSDMA" list at 0x06C -- a count, then an
 * address and length for each slice, the length's top nibble the bits
 * used in its last byte; H.265 adds one 1-byte entry past the last -- and
 * the engine streams them out ahead of the slice data it codes.  On an
 * IDR the first entry starts with the sequence header (H.264 SPS / PPS,
 * H.265 VPS / SPS / PPS), which HDAL then skips in the bitstream buffer
 * and takes from its own copy.  So whatever the picture's slice data,
 * the stream a decoder sees has the driver's parameter sets.
 *
 * With x264 / x265 (pkg-config, at build time) the picture is coded for
 * real: an encoder per stream is set up to match the driver's parameter
 * sets in everything slice data depends on, which its own headers are
 * checked against, and its slice data goes out under a slice header that
 * fits the driver's.  For H.264 that is the driver's own header, the
 * encoder made to use its QP and deblocking; for H.265 a header built
 * from the driver's (NAL type, POC, reference set) and the encoder's
 * (slice type, QP, SAO, deblocking, merge candidates).
 *
 * Otherwise -- no encoder built in, a stream the encoders cannot match
 * (tiles, several slices, SVC / long-term references), or a buffer too
 * small -- the slice is a filler NAL unit, which decoders skip.
 *
 * Registers (h26x_reg.h; h264Enc_wrapPicCfg / h265Enc_wrapSeqCfg):
 *   0x024 / 0x028  source Y / CbCr (NV12; FUNC_CFG bit 17 swaps CbCr)
 *   0x02C / 0x034  reconstruction / reference Y
 *   0x068          NAL length list
 *   0x06C          BSDMA list
 *   0x070 / 0x07C  bitstream buffer and its size
 *   0x084          source line offsets, words: Y [15:0], C [31:16]
 *   0x0A0          FUNC_CFG: bit 0 H.264; bits 2 / 3 compressed source,
 *                  bits 20 / 21 direct from IME
 *   0x0A8          the picture's width [15:0] and height [31:16]
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/misc/nvt-h26x.h"

#ifdef NVT_H26X_X264
#include <x264.h>
#endif
#ifdef NVT_H26X_X265
#include <x265.h>
#endif

#define R(off)              (regs[(off) / 4])

#define H26X_SRC_Y          0x024
#define H26X_SRC_C          0x028
#define H26X_REC_Y          0x02C
#define H26X_REF_Y          0x034
#define H26X_NAL_LEN_ADDR   0x068
#define H26X_BSDMA          0x06C
#define H26X_BSOUT_ADDR     0x070
#define H26X_BSOUT_SIZE     0x07C
#define H26X_SRC_LOFS       0x084
#define H26X_FUNC_CFG       0x0A0
#define H26X_SEQ_CFG        0x0A8

#define FUNC_H264           (1u << 0)
#define FUNC_SRC_COMPRESS   (3u << 2)
#define FUNC_SRC_CBCR_SWAP  (1u << 17)
#define FUNC_SRC_DIRECT     (3u << 20)

#define H26X_STREAMS        4
#define H26X_MAX_HDR        4096
#define H26X_FILLER_LEN     64

/* ---- bits ---- */

typedef struct {
    const uint8_t *p;
    size_t bits;
    size_t pos;
    bool err;
} BitReader;

static void br_init(BitReader *b, const uint8_t *p, size_t len)
{
    *b = (BitReader) { .p = p, .bits = len * 8 };
}

static uint32_t br_u(BitReader *b, unsigned n)
{
    uint32_t v = 0;

    while (n--) {
        if (b->pos >= b->bits) {
            b->err = true;
            return 0;
        }
        v = v << 1 | ((b->p[b->pos / 8] >> (7 - b->pos % 8)) & 1);
        b->pos++;
    }
    return v;
}

static void br_skip(BitReader *b, size_t n)
{
    b->pos += n;
    if (b->pos > b->bits) {
        b->err = true;
    }
}

static uint32_t br_ue(BitReader *b)
{
    unsigned lz = 0;

    while (!br_u(b, 1)) {
        if (b->err || ++lz > 31) {
            b->err = true;
            return 0;
        }
    }
    return ((1u << lz) - 1) + br_u(b, lz);
}

static int32_t br_se(BitReader *b)
{
    uint32_t k = br_ue(b);

    return k & 1 ? (int32_t)((k + 1) / 2) : -(int32_t)(k / 2);
}

/* the position of an RBSP's stop bit: its last 1 */
static size_t rbsp_stop_bit(const uint8_t *p, size_t len)
{
    while (len && !p[len - 1]) {
        len--;
    }
    if (!len) {
        return 0;
    }
    return (len - 1) * 8 + 7 - ctz32(p[len - 1]);
}

static bool br_more_rbsp(BitReader *b)
{
    return b->pos < rbsp_stop_bit(b->p, b->bits / 8);
}

typedef struct {
    GByteArray *a;
    uint32_t acc;
    unsigned have;
} BitWriter;

static void bw_u(BitWriter *w, uint32_t v, unsigned n)
{
    while (n--) {
        w->acc = w->acc << 1 | ((v >> n) & 1);
        if (++w->have == 8) {
            uint8_t c = w->acc;

            g_byte_array_append(w->a, &c, 1);
            w->acc = 0;
            w->have = 0;
        }
    }
}

static void bw_ue(BitWriter *w, uint32_t v)
{
    unsigned n = 32 - clz32(v + 1);

    bw_u(w, 0, n - 1);
    bw_u(w, v + 1, n);
}

static void bw_se(BitWriter *w, int32_t v)
{
    bw_ue(w, v > 0 ? 2 * v - 1 : -2 * v);
}

static void bw_copy(BitWriter *w, const uint8_t *p, size_t from, size_t to)
{
    for (; from < to && from % 8; from++) {
        bw_u(w, (p[from / 8] >> (7 - from % 8)) & 1, 1);
    }
    for (; from + 8 <= to; from += 8) {
        bw_u(w, p[from / 8], 8);
    }
    for (; from < to; from++) {
        bw_u(w, (p[from / 8] >> (7 - from % 8)) & 1, 1);
    }
}

/* rbsp_trailing_bits(): the stop bit, then zeros to a byte */
static void bw_trailing(BitWriter *w)
{
    bw_u(w, 1, 1);
    while (w->have) {
        bw_u(w, 0, 1);
    }
}

/* ---- NAL units ---- */

/* the payload after start code and NAL header, emulation prevention out */
static GByteArray *nal_rbsp(const uint8_t *p, size_t len)
{
    GByteArray *a = g_byte_array_sized_new(len);
    unsigned zeros = 0;
    size_t i;

    for (i = 0; i < len; i++) {
        if (zeros >= 2 && p[i] == 3) {
            zeros = 0;
            continue;
        }
        zeros = p[i] ? 0 : zeros + 1;
        g_byte_array_append(a, &p[i], 1);
    }
    return a;
}

/* append a NAL unit: start code, header, the RBSP with prevention bytes */
static void nal_put(GByteArray *out, const uint8_t *hdr, size_t hlen,
                    const uint8_t *rbsp, size_t len)
{
    static const uint8_t sc[4] = { 0, 0, 0, 1 }, epb = 3;
    unsigned zeros = 0;
    size_t i;

    g_byte_array_append(out, sc, 4);
    g_byte_array_append(out, hdr, hlen);
    for (i = 0; i < len; i++) {
        if (zeros >= 2 && rbsp[i] <= 3) {
            g_byte_array_append(out, &epb, 1);
            zeros = 0;
        }
        zeros = rbsp[i] ? 0 : zeros + 1;
        g_byte_array_append(out, &rbsp[i], 1);
    }
}

/*
 * The next NAL unit at or after *pos: its header's offset, and its end
 * (the next start code).  False when there is none.
 */
static bool nal_next(const uint8_t *p, size_t len, size_t *pos,
                     size_t *start, size_t *end)
{
    size_t i;

    for (i = *pos; i + 3 <= len; i++) {
        if (!p[i] && !p[i + 1] && p[i + 2] == 1) {
            break;
        }
    }
    if (i + 3 > len) {
        return false;
    }
    *start = i + 3;
    for (i = *start; i + 3 <= len; i++) {
        if (!p[i] && !p[i + 1] && (p[i + 2] == 1 ||
            (p[i + 2] == 0 && i + 3 < len && p[i + 3] == 1))) {
            break;
        }
    }
    *end = i + 3 <= len ? i : len;
    *pos = *end;
    return true;
}

/* ---- H.264 ---- */

typedef struct {
    bool valid, separate_planes, frame_mbs_only, poc_always_zero, scaling;
    unsigned profile, chroma_format, depth_luma, depth_chroma;
    unsigned log2_max_frame_num, poc_type, log2_max_poc_lsb;
    unsigned mb_w, mb_h;
} AvcSps;

typedef struct {
    bool valid, cabac, bottom_field_poc, weighted_pred, deblocking_ctrl;
    bool constrained_intra, redundant_pic_cnt, transform_8x8, scaling;
    unsigned num_ref_l0, weighted_bipred, slice_groups;
    int init_qp, chroma_qp_offset, chroma_qp_offset2;
} AvcPps;

typedef struct {
    unsigned nal_ref_idc, nal_type, first_mb, slice_type, num_ref;
    unsigned cabac_init_idc, deblock_idc;
    int qp, alpha, beta;
    size_t data_bit;            /* where slice_data() starts */
} AvcSlice;

static bool avc_sps_parse(AvcSps *s, const uint8_t *p, size_t len)
{
    BitReader b;
    unsigned i, n;

    br_init(&b, p, len);
    memset(s, 0, sizeof(*s));
    s->profile = br_u(&b, 8);
    br_skip(&b, 16);
    br_ue(&b);
    s->chroma_format = 1;
    switch (s->profile) {
    case 100: case 110: case 122: case 244: case 44: case 83: case 86:
    case 118: case 128: case 138: case 139: case 134: case 135:
        s->chroma_format = br_ue(&b);
        if (s->chroma_format == 3) {
            s->separate_planes = br_u(&b, 1);
        }
        s->depth_luma = br_ue(&b);
        s->depth_chroma = br_ue(&b);
        br_skip(&b, 1);
        s->scaling = br_u(&b, 1);
        if (s->scaling) {
            return false;
        }
        break;
    }
    s->log2_max_frame_num = br_ue(&b) + 4;
    s->poc_type = br_ue(&b);
    if (s->poc_type == 0) {
        s->log2_max_poc_lsb = br_ue(&b) + 4;
    } else if (s->poc_type == 1) {
        s->poc_always_zero = br_u(&b, 1);
        br_se(&b);
        br_se(&b);
        n = br_ue(&b);
        for (i = 0; i < n && !b.err; i++) {
            br_se(&b);
        }
    }
    br_ue(&b);
    br_skip(&b, 1);
    s->mb_w = br_ue(&b) + 1;
    s->mb_h = br_ue(&b) + 1;
    s->frame_mbs_only = br_u(&b, 1);
    if (!s->frame_mbs_only) {
        s->mb_h *= 2;
    }
    s->valid = !b.err && s->log2_max_frame_num <= 16 &&
               s->log2_max_poc_lsb <= 16;
    return s->valid;
}

static bool avc_pps_parse(AvcPps *s, const uint8_t *p, size_t len)
{
    BitReader b;

    br_init(&b, p, len);
    memset(s, 0, sizeof(*s));
    br_ue(&b);
    br_ue(&b);
    s->cabac = br_u(&b, 1);
    s->bottom_field_poc = br_u(&b, 1);
    s->slice_groups = br_ue(&b) + 1;
    s->num_ref_l0 = br_ue(&b) + 1;
    br_ue(&b);
    s->weighted_pred = br_u(&b, 1);
    s->weighted_bipred = br_u(&b, 2);
    s->init_qp = 26 + br_se(&b);
    br_se(&b);
    s->chroma_qp_offset = br_se(&b);
    s->deblocking_ctrl = br_u(&b, 1);
    s->constrained_intra = br_u(&b, 1);
    s->redundant_pic_cnt = br_u(&b, 1);
    s->chroma_qp_offset2 = s->chroma_qp_offset;
    if (br_more_rbsp(&b)) {
        s->transform_8x8 = br_u(&b, 1);
        s->scaling = br_u(&b, 1);
        if (!s->scaling) {
            s->chroma_qp_offset2 = br_se(&b);
        }
    }
    s->valid = !b.err && s->slice_groups == 1 && !s->scaling;
    return s->valid;
}

enum { AVC_P = 0, AVC_B = 1, AVC_I = 2, AVC_SP = 3, AVC_SI = 4 };

/* a slice header, from the RBSP after the NAL header */
static bool avc_slice_parse(AvcSlice *sl, const AvcSps *sps,
                            const AvcPps *pps, unsigned nal_ref_idc,
                            unsigned nal_type, const uint8_t *p, size_t len)
{
    BitReader b;
    unsigned op;

    br_init(&b, p, len);
    memset(sl, 0, sizeof(*sl));
    sl->nal_ref_idc = nal_ref_idc;
    sl->nal_type = nal_type;
    sl->first_mb = br_ue(&b);
    sl->slice_type = br_ue(&b) % 5;
    br_ue(&b);
    if (sps->separate_planes) {
        br_skip(&b, 2);
    }
    br_skip(&b, sps->log2_max_frame_num);
    if (!sps->frame_mbs_only && br_u(&b, 1)) {
        return false;                   /* field pictures */
    }
    if (nal_type == 5) {
        br_ue(&b);
    }
    if (sps->poc_type == 0) {
        br_skip(&b, sps->log2_max_poc_lsb);
        if (pps->bottom_field_poc) {
            br_se(&b);
        }
    } else if (sps->poc_type == 1 && !sps->poc_always_zero) {
        br_se(&b);
        if (pps->bottom_field_poc) {
            br_se(&b);
        }
    }
    if (pps->redundant_pic_cnt) {
        br_ue(&b);
    }
    if (sl->slice_type != AVC_P && sl->slice_type != AVC_I) {
        return false;                   /* B, SP, SI */
    }
    sl->num_ref = sl->slice_type == AVC_P ? pps->num_ref_l0 : 0;
    if (sl->slice_type == AVC_P) {
        if (br_u(&b, 1)) {
            sl->num_ref = br_ue(&b) + 1;
        }
        if (br_u(&b, 1)) {              /* ref_pic_list_modification */
            do {
                op = br_ue(&b);
                if (op < 3) {
                    br_ue(&b);
                }
            } while (op != 3 && !b.err);
        }
        if (pps->weighted_pred) {
            return false;
        }
    }
    if (nal_ref_idc) {
        if (nal_type == 5) {
            br_skip(&b, 2);
        } else if (br_u(&b, 1)) {       /* adaptive_ref_pic_marking */
            while ((op = br_ue(&b)) && !b.err) {
                if (op == 1 || op == 3) {
                    br_ue(&b);
                }
                if (op == 2 || op == 3 || op == 6 || op == 4) {
                    br_ue(&b);
                }
            }
        }
    }
    if (pps->cabac && sl->slice_type != AVC_I) {
        sl->cabac_init_idc = br_ue(&b);
    }
    sl->qp = pps->init_qp + br_se(&b);
    if (pps->deblocking_ctrl) {
        sl->deblock_idc = br_ue(&b);
        if (sl->deblock_idc != 1) {
            sl->alpha = br_se(&b);
            sl->beta = br_se(&b);
        }
    }
    sl->data_bit = pps->cabac ? ROUND_UP(b.pos, 8) : b.pos;
    return !b.err && sl->data_bit <= len * 8;
}

/* ---- H.265 ---- */

#define HEVC_MAX_RPS        65

typedef struct {
    bool valid, separate_planes, scaling, amp, sao, pcm, long_term, tmvp;
    bool strong_intra;
    unsigned chroma_format, width, height, depth_luma, depth_chroma;
    unsigned log2_max_poc_lsb, log2_min_cb, log2_ctb, log2_min_tb;
    unsigned log2_max_tb, depth_inter, depth_intra, num_rps;
    uint8_t rps_count[HEVC_MAX_RPS];    /* NumDeltaPocs */
} HevcSps;

typedef struct {
    bool valid, dependent_slices, output_flag, sign_hiding, cabac_init;
    bool constrained_intra, transform_skip, cu_qp_delta, slice_chroma_qp;
    bool weighted_pred, weighted_bipred, transquant_bypass, tiles;
    bool entropy_sync, lf_across_slices, deblocking_override;
    bool deblocking_disabled, scaling, lists_mod, slice_ext;
    unsigned id, extra_bits, num_ref_l0, cu_qp_delta_depth, log2_merge_level;
    int init_qp, cb_qp_offset, cr_qp_offset, beta, tc;
} HevcPps;

typedef struct {
    unsigned nal_type, tid, pps_id, slice_type, poc_lsb, rps_idx, num_ref;
    unsigned max_merge;
    bool no_output_prior, rps_from_sps, tmvp, sao_luma, sao_chroma;
    bool cabac_init, deblocking_disabled;
    int qp, beta, tc, cb_qp_offset, cr_qp_offset;
    size_t data_bit;
} HevcSlice;

enum { HEVC_B = 0, HEVC_P = 1, HEVC_I = 2 };

static void hevc_ptl_skip(BitReader *b, unsigned max_sub_layers_minus1)
{
    unsigned i, prof = 0, lev = 0;

    br_skip(b, 96);                     /* general profile, tier, level */
    for (i = 0; i < max_sub_layers_minus1; i++) {
        prof |= br_u(b, 1) << i;
        lev |= br_u(b, 1) << i;
    }
    if (max_sub_layers_minus1) {
        br_skip(b, 2 * (8 - max_sub_layers_minus1));
    }
    for (i = 0; i < max_sub_layers_minus1; i++) {
        br_skip(b, (prof >> i & 1) * 88 + (lev >> i & 1) * 8);
    }
}

static unsigned ceil_log2(unsigned v)
{
    return v > 1 ? 32 - clz32(v - 1) : 0;
}

/* st_ref_pic_set(idx): the number of pictures it holds, or -1 */
static int hevc_rps_parse(BitReader *b, unsigned idx, unsigned num_sets,
                          const uint8_t *counts)
{
    unsigned i, neg, pos, ref, n = 0;

    if (idx && br_u(b, 1)) {            /* inter_ref_pic_set_prediction */
        unsigned delta_idx = idx == num_sets ? br_ue(b) + 1 : 1;

        if (delta_idx > idx) {
            return -1;
        }
        ref = idx - delta_idx;
        br_skip(b, 1);
        br_ue(b);
        for (i = 0; i <= counts[ref]; i++) {
            if (br_u(b, 1) || br_u(b, 1)) {
                n++;
            }
        }
        return b->err || n > 16 ? -1 : (int)n;
    }
    neg = br_ue(b);
    pos = br_ue(b);
    if (neg > 16 || pos > 16 || neg + pos > 16) {
        return -1;
    }
    for (i = 0; i < neg + pos; i++) {
        br_ue(b);
        br_skip(b, 1);
    }
    return b->err ? -1 : (int)(neg + pos);
}

static bool hevc_sps_parse(HevcSps *s, const uint8_t *p, size_t len)
{
    BitReader b;
    unsigned i, sub, n;
    int c;

    br_init(&b, p, len);
    memset(s, 0, sizeof(*s));
    br_skip(&b, 4);
    sub = br_u(&b, 3);
    br_skip(&b, 1);
    hevc_ptl_skip(&b, sub);
    br_ue(&b);
    s->chroma_format = br_ue(&b);
    if (s->chroma_format == 3) {
        s->separate_planes = br_u(&b, 1);
    }
    s->width = br_ue(&b);
    s->height = br_ue(&b);
    if (br_u(&b, 1)) {                  /* conformance window */
        for (i = 0; i < 4; i++) {
            br_ue(&b);
        }
    }
    s->depth_luma = br_ue(&b);
    s->depth_chroma = br_ue(&b);
    s->log2_max_poc_lsb = br_ue(&b) + 4;
    for (i = br_u(&b, 1) ? 0 : sub; i <= sub; i++) {
        br_ue(&b);
        br_ue(&b);
        br_ue(&b);
    }
    s->log2_min_cb = br_ue(&b) + 3;
    s->log2_ctb = s->log2_min_cb + br_ue(&b);
    s->log2_min_tb = br_ue(&b) + 2;
    s->log2_max_tb = s->log2_min_tb + br_ue(&b);
    s->depth_inter = br_ue(&b);
    s->depth_intra = br_ue(&b);
    s->scaling = br_u(&b, 1);
    if (s->scaling) {
        return false;
    }
    s->amp = br_u(&b, 1);
    s->sao = br_u(&b, 1);
    s->pcm = br_u(&b, 1);
    if (s->pcm) {
        return false;
    }
    s->num_rps = br_ue(&b);
    if (s->num_rps >= HEVC_MAX_RPS) {
        return false;
    }
    for (i = 0; i < s->num_rps; i++) {
        c = hevc_rps_parse(&b, i, s->num_rps, s->rps_count);
        if (c < 0) {
            return false;
        }
        s->rps_count[i] = c;
    }
    s->long_term = br_u(&b, 1);
    if (s->long_term) {
        n = br_ue(&b);
        for (i = 0; i < n && !b.err; i++) {
            br_skip(&b, s->log2_max_poc_lsb + 1);
        }
    }
    s->tmvp = br_u(&b, 1);
    s->strong_intra = br_u(&b, 1);
    s->valid = !b.err && s->log2_max_poc_lsb <= 16 && s->log2_ctb <= 6 &&
               s->width && s->height;
    return s->valid;
}

static bool hevc_pps_parse(HevcPps *s, const uint8_t *p, size_t len)
{
    BitReader b;
    unsigned i, cols, rows;

    br_init(&b, p, len);
    memset(s, 0, sizeof(*s));
    s->id = br_ue(&b);
    br_ue(&b);
    s->dependent_slices = br_u(&b, 1);
    s->output_flag = br_u(&b, 1);
    s->extra_bits = br_u(&b, 3);
    s->sign_hiding = br_u(&b, 1);
    s->cabac_init = br_u(&b, 1);
    s->num_ref_l0 = br_ue(&b) + 1;
    br_ue(&b);
    s->init_qp = 26 + br_se(&b);
    s->constrained_intra = br_u(&b, 1);
    s->transform_skip = br_u(&b, 1);
    s->cu_qp_delta = br_u(&b, 1);
    if (s->cu_qp_delta) {
        s->cu_qp_delta_depth = br_ue(&b);
    }
    s->cb_qp_offset = br_se(&b);
    s->cr_qp_offset = br_se(&b);
    s->slice_chroma_qp = br_u(&b, 1);
    s->weighted_pred = br_u(&b, 1);
    s->weighted_bipred = br_u(&b, 1);
    s->transquant_bypass = br_u(&b, 1);
    s->tiles = br_u(&b, 1);
    s->entropy_sync = br_u(&b, 1);
    if (s->tiles) {
        cols = br_ue(&b) + 1;
        rows = br_ue(&b) + 1;
        if (!br_u(&b, 1)) {             /* not uniform */
            for (i = 0; i < cols + rows - 2 && !b.err; i++) {
                br_ue(&b);
            }
        }
        br_skip(&b, 1);
    }
    s->lf_across_slices = br_u(&b, 1);
    if (br_u(&b, 1)) {                  /* deblocking control */
        s->deblocking_override = br_u(&b, 1);
        s->deblocking_disabled = br_u(&b, 1);
        if (!s->deblocking_disabled) {
            s->beta = br_se(&b);
            s->tc = br_se(&b);
        }
    }
    s->scaling = br_u(&b, 1);
    s->lists_mod = br_u(&b, 1);
    s->log2_merge_level = br_ue(&b) + 2;
    s->slice_ext = br_u(&b, 1);
    s->valid = !b.err && !s->scaling;
    return s->valid;
}

/* a slice segment header, from the RBSP after the NAL header */
static bool hevc_slice_parse(HevcSlice *sl, const HevcSps *sps,
                             const HevcPps *pps, unsigned nal_type,
                             const uint8_t *p, size_t len)
{
    unsigned ctbs, i, n;
    BitReader b;

    br_init(&b, p, len);
    memset(sl, 0, sizeof(*sl));
    sl->nal_type = nal_type;
    if (!br_u(&b, 1)) {
        return false;                   /* not a picture's first slice */
    }
    if (nal_type >= 16 && nal_type <= 23) {
        sl->no_output_prior = br_u(&b, 1);
    }
    sl->pps_id = br_ue(&b);
    if (sl->pps_id != pps->id) {
        return false;                   /* not the PPS we hold */
    }
    br_skip(&b, pps->extra_bits);
    sl->slice_type = br_ue(&b);
    if (sl->slice_type != HEVC_P && sl->slice_type != HEVC_I) {
        return false;
    }
    if (pps->output_flag) {
        br_skip(&b, 1);
    }
    if (sps->separate_planes) {
        br_skip(&b, 2);
    }
    if (nal_type != 19 && nal_type != 20) {
        sl->poc_lsb = br_u(&b, sps->log2_max_poc_lsb);
        sl->rps_from_sps = br_u(&b, 1);
        if (!sl->rps_from_sps) {
            if (hevc_rps_parse(&b, sps->num_rps, sps->num_rps,
                               sps->rps_count) < 0) {
                return false;
            }
        } else if (sps->num_rps > 1) {
            sl->rps_idx = br_u(&b, ceil_log2(sps->num_rps));
        }
        if (sps->long_term) {
            return false;
        }
        if (sps->tmvp) {
            sl->tmvp = br_u(&b, 1);
        }
    }
    if (sps->sao) {
        sl->sao_luma = br_u(&b, 1);
        if (sps->chroma_format) {
            sl->sao_chroma = br_u(&b, 1);
        }
    }
    if (sl->slice_type == HEVC_P) {
        sl->num_ref = pps->num_ref_l0;
        if (br_u(&b, 1)) {
            sl->num_ref = br_ue(&b) + 1;
        }
        if (pps->lists_mod || pps->weighted_pred) {
            return false;
        }
        if (pps->cabac_init) {
            sl->cabac_init = br_u(&b, 1);
        }
        if (sl->tmvp && sl->num_ref > 1) {
            br_ue(&b);
        }
        sl->max_merge = 5 - br_ue(&b);
    }
    sl->qp = pps->init_qp + br_se(&b);
    if (pps->slice_chroma_qp) {
        sl->cb_qp_offset = br_se(&b);
        sl->cr_qp_offset = br_se(&b);
    }
    sl->deblocking_disabled = pps->deblocking_disabled;
    sl->beta = pps->beta;
    sl->tc = pps->tc;
    if (pps->deblocking_override && br_u(&b, 1)) {
        sl->deblocking_disabled = br_u(&b, 1);
        if (!sl->deblocking_disabled) {
            sl->beta = br_se(&b);
            sl->tc = br_se(&b);
        }
    }
    if (pps->lf_across_slices &&
        (sl->sao_luma || sl->sao_chroma || !sl->deblocking_disabled)) {
        br_skip(&b, 1);
    }
    if (pps->tiles || pps->entropy_sync) {
        n = br_ue(&b);
        if (n) {
            i = br_ue(&b) + 1;
            br_skip(&b, (size_t)n * i);
        }
    }
    if (pps->slice_ext) {
        br_skip(&b, (size_t)br_ue(&b) * 8);
    }
    if (!br_u(&b, 1)) {                 /* byte_alignment() */
        return false;
    }
    sl->data_bit = ROUND_UP(b.pos, 8);
    ctbs = DIV_ROUND_UP(sps->width, 1u << sps->log2_ctb) *
           DIV_ROUND_UP(sps->height, 1u << sps->log2_ctb);
    return !b.err && ctbs && sl->data_bit <= len * 8;
}

/* ---- the streams ---- */

typedef struct {
    uint32_t key;               /* BSDMA list address: one per encoder */
    bool used;
    bool h264;
    bool broken;                /* not encoding until the next IDR */
    uint32_t last_rec;          /* the previous picture's reconstruction */
    GByteArray *seq;            /* the driver's parameter sets, as written */
    AvcSps asps;                /* the driver's parameter sets */
    AvcPps apps;
    HevcSps hsps;
    HevcPps hpps;
    AvcSps easps;               /* the encoder's */
    AvcPps eapps;
    HevcSps ehsps;
    HevcPps ehpps;
    void *enc;
    void *param;                /* x265's, for its pictures */
    uint64_t pts;
    unsigned w, h;              /* the encoder's coded size */
} H26xStream;

struct NvtH26x {
    H26xStream st[H26X_STREAMS];
    unsigned victim;
};

static void stream_close(H26xStream *st)
{
    if (st->enc) {
#ifdef NVT_H26X_X264
        if (st->h264) {
            x264_encoder_close(st->enc);
        }
#endif
#ifdef NVT_H26X_X265
        if (!st->h264) {
            x265_encoder_close(st->enc);
        }
#endif
        st->enc = NULL;
    }
#ifdef NVT_H26X_X265
    if (st->param) {
        x265_param_free(st->param);
        st->param = NULL;
    }
#endif
}

static void stream_drop(H26xStream *st)
{
    stream_close(st);
    if (st->seq) {
        g_byte_array_free(st->seq, true);
    }
    memset(st, 0, sizeof(*st));
}

NvtH26x *nvt_h26x_new(void)
{
    return g_new0(NvtH26x, 1);
}

void nvt_h26x_reset(NvtH26x *h)
{
    unsigned i;

    for (i = 0; i < H26X_STREAMS; i++) {
        stream_drop(&h->st[i]);
    }
}

void nvt_h26x_free(NvtH26x *h)
{
    if (h) {
        nvt_h26x_reset(h);
        g_free(h);
    }
}

static H26xStream *stream_get(NvtH26x *h, uint32_t key, bool h264)
{
    H26xStream *st;
    unsigned i;

    for (i = 0; i < H26X_STREAMS; i++) {
        if (h->st[i].used && h->st[i].key == key) {
            st = &h->st[i];
            if (st->h264 != h264) {
                stream_drop(st);
                break;
            }
            return st;
        }
    }
    for (i = 0; i < H26X_STREAMS; i++) {
        if (!h->st[i].used) {
            break;
        }
    }
    if (i == H26X_STREAMS) {
        i = h->victim++ % H26X_STREAMS;
    }
    st = &h->st[i];
    stream_drop(st);
    st->used = true;
    st->key = key;
    st->h264 = h264;
    st->broken = true;
    return st;
}

static void unimp_once(const char *why)
{
    static GHashTable *seen;

    if (!seen) {
        seen = g_hash_table_new(g_str_hash, g_str_equal);
    }
    if (!g_hash_table_contains(seen, why)) {
        g_hash_table_add(seen, (gpointer)why);
        qemu_log_mask(LOG_UNIMP, "nvt-h26x: %s; filler slices instead\n", why);
    }
}

/* ---- the encoders ---- */

typedef struct {
    const uint8_t *y, *uv;      /* NV12, the coded size, Cb first */
    unsigned w, h;
    bool idr;
    int qp;
} H26xPicture;

/* the encoder's slice NAL for a picture (with start code), or NULL */
static GByteArray *encoder_picture(H26xStream *st, const H26xPicture *pic);

#ifdef NVT_H26X_X264
static bool enc264_open(H26xStream *st, const AvcSlice *sl)
{
    x264_param_t p;

    if (x264_param_default_preset(&p, "ultrafast", "zerolatency") < 0) {
        return false;
    }
    p.i_log_level = X264_LOG_NONE;
    p.i_width = st->w;
    p.i_height = st->h;
    p.i_csp = X264_CSP_NV12;
    p.i_threads = 1;
    p.b_sliced_threads = 0;
    p.i_lookahead_threads = 1;
    p.i_sync_lookahead = 0;
    p.rc.i_lookahead = 0;
    p.i_bframe = 0;
    p.i_frame_reference = 1;
    p.i_keyint_max = X264_KEYINT_MAX_INFINITE;
    p.i_scenecut_threshold = 0;
    p.b_intra_refresh = 0;
    p.b_cabac = st->apps.cabac;
    p.analyse.b_transform_8x8 = st->apps.transform_8x8;
    p.analyse.i_weighted_pred = X264_WEIGHTP_NONE;
    p.analyse.b_psy = 0;
    p.analyse.i_chroma_qp_offset = st->apps.chroma_qp_offset;
    p.b_deblocking_filter = sl->deblock_idc != 1;
    p.i_deblocking_filter_alphac0 = sl->alpha;
    p.i_deblocking_filter_beta = sl->beta;
    /*
     * The QP is forced each picture.  Not with constant QP: x264 clamps
     * that to the range its I / P / B offsets span around the constant.
     */
    p.rc.i_rc_method = X264_RC_CRF;
    p.rc.f_rf_constant = 26;
    p.rc.i_qp_min = 0;
    p.rc.i_qp_max = 51;
    p.rc.i_aq_mode = X264_AQ_NONE;
    p.rc.b_mb_tree = 0;
    p.b_repeat_headers = 0;
    p.b_annexb = 1;
    p.b_vfr_input = 0;
    p.i_fps_num = 30;
    p.i_fps_den = 1;
    if (x264_param_apply_profile(&p, st->asps.profile >= 100 ? "high" :
                                 st->asps.profile == 77 ? "main" :
                                 "baseline") < 0) {
        return false;
    }
    st->enc = x264_encoder_open(&p);
    return st->enc;
}

/* the encoder's parameter sets against the driver's */
static bool enc264_matches(H26xStream *st)
{
    x264_nal_t *nal;
    int n, i;
    AvcSps *sps = &st->easps;
    AvcPps *pps = &st->eapps;
    g_autoptr(GByteArray) rbsp = NULL;

    memset(sps, 0, sizeof(*sps));
    memset(pps, 0, sizeof(*pps));

    if (x264_encoder_headers(st->enc, &nal, &n) < 0) {
        return false;
    }
    for (i = 0; i < n; i++) {
        const uint8_t *q = nal[i].p_payload;
        size_t len = nal[i].i_payload, s, e, pos = 0;

        if (!nal_next(q, len, &pos, &s, &e) || e - s < 2) {
            continue;
        }
        rbsp = nal_rbsp(q + s + 1, e - s - 1);
        if (nal[i].i_type == NAL_SPS) {
            avc_sps_parse(sps, rbsp->data, rbsp->len);
        } else if (nal[i].i_type == NAL_PPS) {
            avc_pps_parse(pps, rbsp->data, rbsp->len);
        }
        g_byte_array_free(g_steal_pointer(&rbsp), true);
    }
    return sps->valid && pps->valid &&
           sps->chroma_format == st->asps.chroma_format &&
           sps->depth_luma == st->asps.depth_luma &&
           sps->depth_chroma == st->asps.depth_chroma &&
           sps->frame_mbs_only && st->asps.frame_mbs_only &&
           sps->mb_w == st->asps.mb_w && sps->mb_h == st->asps.mb_h &&
           pps->cabac == st->apps.cabac &&
           pps->transform_8x8 == st->apps.transform_8x8 &&
           pps->constrained_intra == st->apps.constrained_intra &&
           pps->chroma_qp_offset == st->apps.chroma_qp_offset &&
           pps->chroma_qp_offset2 == st->apps.chroma_qp_offset2 &&
           !pps->weighted_pred && !st->apps.weighted_pred;
}

static GByteArray *enc264_picture(H26xStream *st, const H26xPicture *pic)
{
    x264_picture_t in, out;
    x264_nal_t *nal;
    int n, i;

    x264_picture_init(&in);
    in.img.i_csp = X264_CSP_NV12;
    in.img.i_plane = 2;
    in.img.plane[0] = (uint8_t *)pic->y;
    in.img.plane[1] = (uint8_t *)pic->uv;
    in.img.i_stride[0] = pic->w;
    in.img.i_stride[1] = pic->w;
    in.i_type = pic->idr ? X264_TYPE_IDR : X264_TYPE_P;
    in.i_qpplus1 = pic->qp + 1;
    in.i_pts = st->pts++;
    if (x264_encoder_encode(st->enc, &nal, &n, &in, &out) <= 0) {
        return NULL;
    }
    for (i = 0; i < n; i++) {
        if (nal[i].i_type == NAL_SLICE || nal[i].i_type == NAL_SLICE_IDR) {
            GByteArray *a = g_byte_array_new();

            g_byte_array_append(a, nal[i].p_payload, nal[i].i_payload);
            return a;
        }
    }
    return NULL;
}
#endif

#ifdef NVT_H26X_X265
static bool enc265_open(H26xStream *st)
{
    x265_param *p = x265_param_alloc();
    bool ok = false;

    if (!p || x265_param_default_preset(p, "ultrafast", "zerolatency") < 0) {
        goto out;
    }
    p->logLevel = X265_LOG_NONE;
    p->sourceWidth = st->w;
    p->sourceHeight = st->h;
    p->internalCsp = X265_CSP_I420;
    p->fpsNum = 30;
    p->fpsDenom = 1;
    p->frameNumThreads = 1;
    p->bEnableWavefront = 0;
    p->lookaheadDepth = 0;
    p->bframes = 0;
    p->maxNumReferences = 1;
    p->keyframeMax = INT_MAX;
    p->keyframeMin = 1;
    p->scenecutThreshold = 0;
    p->bOpenGOP = 0;
    p->bRepeatHeaders = 0;
    p->bAnnexB = 1;
    p->bEmitInfoSEI = 0;
    p->bEmitHRDSEI = 0;
    p->decodedPictureHashSEI = 0;
    p->bEnableAccessUnitDelimiters = 0;
    p->maxSlices = 1;
    p->maxCUSize = 1u << st->hsps.log2_ctb;
    p->minCUSize = 1u << st->hsps.log2_min_cb;
    p->maxTUSize = 1u << st->hsps.log2_max_tb;
    p->tuQTMaxIntraDepth = st->hsps.depth_intra + 1;
    p->tuQTMaxInterDepth = st->hsps.depth_inter + 1;
    p->bEnableAMP = st->hsps.amp;
    p->bEnableRectInter = st->hsps.amp;
    p->bEnableSignHiding = st->hpps.sign_hiding;
    p->bEnableTransformSkip = st->hpps.transform_skip;
    p->bEnableConstrainedIntra = st->hpps.constrained_intra;
    p->bEnableStrongIntraSmoothing = st->hsps.strong_intra;
    p->bEnableSAO = st->hsps.sao;
    p->bEnableTemporalMvp = st->hsps.tmvp;
    p->bEnableWeightedPred = 0;
    p->bEnableLoopFilter = !st->hpps.deblocking_disabled;
    p->deblockingFilterBetaOffset = st->hpps.beta;
    p->deblockingFilterTCOffset = st->hpps.tc;
    p->cbQpOffset = st->hpps.cb_qp_offset;
    p->crQpOffset = st->hpps.cr_qp_offset;
    p->bLossless = 0;
    p->bCULossless = 0;
    /*
     * cu_qp_delta: x265 signals it only with adaptive quantisation on,
     * which constant QP turns off; CRF with a forced QP each picture
     * keeps it, the picture still at the forced QP.
     */
    p->rc.rateControlMode = X265_RC_CRF;
    p->rc.rfConstant = 26;
    p->rc.cuTree = 0;
    if (st->hpps.cu_qp_delta) {
        p->rc.aqMode = X265_AQ_VARIANCE;
        p->rc.aqStrength = 0.1;
        p->rc.qgSize = p->maxCUSize >> st->hpps.cu_qp_delta_depth;
    } else {
        p->rc.aqMode = X265_AQ_NONE;
    }
    st->enc = x265_encoder_open(p);
    ok = st->enc;
out:
    if (ok) {
        st->param = p;
    } else if (p) {
        x265_param_free(p);
    }
    return ok;
}

static bool enc265_matches(H26xStream *st)
{
    const HevcSps *d = &st->hsps, *sps = &st->ehsps;
    const HevcPps *q = &st->hpps, *pps = &st->ehpps;
    x265_nal *nal;
    uint32_t n, i;

    memset(&st->ehsps, 0, sizeof(st->ehsps));
    memset(&st->ehpps, 0, sizeof(st->ehpps));
    if (x265_encoder_headers(st->enc, &nal, &n) < 0) {
        return false;
    }
    for (i = 0; i < n; i++) {
        size_t s, e, pos = 0;
        g_autoptr(GByteArray) rbsp = NULL;

        if (!nal_next(nal[i].payload, nal[i].sizeBytes, &pos, &s, &e) ||
            e - s < 3) {
            continue;
        }
        rbsp = nal_rbsp(nal[i].payload + s + 2, e - s - 2);
        if (nal[i].type == NAL_UNIT_SPS) {
            hevc_sps_parse(&st->ehsps, rbsp->data, rbsp->len);
        } else if (nal[i].type == NAL_UNIT_PPS) {
            hevc_pps_parse(&st->ehpps, rbsp->data, rbsp->len);
        }
    }
    return sps->valid && pps->valid &&
           sps->chroma_format == d->chroma_format &&
           sps->width == d->width && sps->height == d->height &&
           sps->depth_luma == d->depth_luma &&
           sps->depth_chroma == d->depth_chroma &&
           sps->log2_min_cb == d->log2_min_cb && sps->log2_ctb == d->log2_ctb &&
           sps->log2_min_tb == d->log2_min_tb &&
           sps->log2_max_tb == d->log2_max_tb &&
           sps->depth_inter == d->depth_inter &&
           sps->depth_intra == d->depth_intra &&
           sps->amp == d->amp && sps->strong_intra == d->strong_intra &&
           (!sps->sao || d->sao) && (!sps->tmvp || d->tmvp) &&
           pps->sign_hiding == q->sign_hiding &&
           pps->constrained_intra == q->constrained_intra &&
           pps->transform_skip == q->transform_skip &&
           pps->cu_qp_delta == q->cu_qp_delta &&
           (!q->cu_qp_delta ||
            pps->cu_qp_delta_depth == q->cu_qp_delta_depth) &&
           pps->cb_qp_offset == q->cb_qp_offset &&
           pps->cr_qp_offset == q->cr_qp_offset &&
           pps->transquant_bypass == q->transquant_bypass &&
           !pps->tiles && !q->tiles && pps->entropy_sync == q->entropy_sync &&
           !pps->weighted_pred && !q->weighted_pred &&
           pps->log2_merge_level == q->log2_merge_level &&
           !q->lists_mod;
}

static GByteArray *enc265_picture(H26xStream *st, const H26xPicture *pic)
{
    x265_picture *in = x265_picture_alloc();
    x265_nal *nal;
    uint32_t n = 0, i;
    unsigned cw = pic->w / 2, ch = pic->h / 2, x, y;
    g_autofree uint8_t *u = g_malloc(cw * ch), *v = g_malloc(cw * ch);
    GByteArray *a = NULL;

    x265_picture_init(st->param, in);
    for (y = 0; y < ch; y++) {
        for (x = 0; x < cw; x++) {
            u[y * cw + x] = pic->uv[y * pic->w + 2 * x];
            v[y * cw + x] = pic->uv[y * pic->w + 2 * x + 1];
        }
    }
    in->colorSpace = X265_CSP_I420;
    in->bitDepth = 8;
    in->planes[0] = (void *)pic->y;
    in->planes[1] = u;
    in->planes[2] = v;
    in->stride[0] = pic->w;
    in->stride[1] = cw;
    in->stride[2] = cw;
    in->sliceType = pic->idr ? X265_TYPE_IDR : X265_TYPE_P;
    in->forceqp = pic->qp + 1;
    in->pts = st->pts++;
    if (x265_encoder_encode(st->enc, &nal, &n, in, NULL) > 0) {
        for (i = 0; i < n; i++) {
            if (nal[i].type < 32) {
                a = g_byte_array_new();
                g_byte_array_append(a, nal[i].payload, nal[i].sizeBytes);
                break;
            }
        }
    }
    x265_picture_free(in);
    return a;
}
#endif

static GByteArray *encoder_picture(H26xStream *st, const H26xPicture *pic)
{
#ifdef NVT_H26X_X264
    if (st->h264) {
        return enc264_picture(st, pic);
    }
#endif
#ifdef NVT_H26X_X265
    if (!st->h264) {
        return enc265_picture(st, pic);
    }
#endif
    return NULL;
}

/* ---- a picture ---- */

typedef struct {
    GByteArray *seq;            /* the sequence header, IDR only */
    GByteArray *slice;          /* the slice NAL's header, no start code */
    unsigned nal_hdr_len;
    bool idr;
} H26xJob;

/*
 * The driver's headers for a picture: one slice (H.264: one entry; H.265:
 * one, then the 1-byte extra), the sequence header in front on an IDR.
 */
static bool job_read(const uint32_t *regs, bool h264, H26xJob *job)
{
    uint32_t list = R(H26X_BSDMA), addr, len;
    g_autofree uint8_t *hdr = NULL;
    size_t pos = 0, s, e, last = 0;
    unsigned type;

    memset(job, 0, sizeof(*job));
    if (!list || ldl_le_phys(&address_space_memory, list) !=
        (h264 ? 1 : 2)) {
        return false;
    }
    addr = ldl_le_phys(&address_space_memory, list + 4);
    len = ldl_le_phys(&address_space_memory, list + 8);
    len &= 0x0FFFFFFF;
    if (!addr || len < 6 || len > H26X_MAX_HDR) {
        return false;
    }
    hdr = g_malloc(len);
    if (dma_memory_read(&address_space_memory, addr, hdr, len,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    /* parameter sets up to the slice NAL, the last */
    job->nal_hdr_len = h264 ? 1 : 2;
    while (nal_next(hdr, len, &pos, &s, &e)) {
        type = h264 ? hdr[s] & 0x1F : (hdr[s] >> 1) & 0x3F;
        if (h264 ? type == 1 || type == 5 : type < 32) {
            last = s;
            job->idr = h264 ? type == 5 : type == 19 || type == 20;
            break;
        }
    }
    if (!last || last + job->nal_hdr_len >= len) {
        return false;
    }
    /* the start code's first byte: three or four bytes before */
    s = last - 3 - (last >= 4 && !hdr[last - 4]);
    if (s) {
        job->seq = g_byte_array_new();
        g_byte_array_append(job->seq, hdr, s);
    }
    job->slice = g_byte_array_new();
    g_byte_array_append(job->slice, hdr + last, len - last);
    return true;
}

static void job_free(H26xJob *job)
{
    if (job->seq) {
        g_byte_array_free(job->seq, true);
    }
    if (job->slice) {
        g_byte_array_free(job->slice, true);
    }
}

/* the driver's parameter sets for a stream, from an IDR's header */
static bool stream_learn(H26xStream *st, const GByteArray *seq)
{
    size_t pos = 0, s, e;
    unsigned type;
    bool sps = false, pps = false;

    while (nal_next(seq->data, seq->len, &pos, &s, &e)) {
        g_autoptr(GByteArray) rbsp = NULL;

        if (st->h264) {
            type = seq->data[s] & 0x1F;
            rbsp = nal_rbsp(seq->data + s + 1, e - s - 1);
            if (type == 7) {
                sps = avc_sps_parse(&st->asps, rbsp->data, rbsp->len);
            } else if (type == 8) {
                pps = avc_pps_parse(&st->apps, rbsp->data, rbsp->len);
            }
        } else {
            type = (seq->data[s] >> 1) & 0x3F;
            if (e - s < 3) {
                continue;
            }
            rbsp = nal_rbsp(seq->data + s + 2, e - s - 2);
            if (type == 33) {
                sps = hevc_sps_parse(&st->hsps, rbsp->data, rbsp->len);
            } else if (type == 34) {
                pps = hevc_pps_parse(&st->hpps, rbsp->data, rbsp->len);
            }
        }
    }
    return sps && pps;
}

/* the source picture, padded to the coded size */
static bool read_source(const uint32_t *regs, unsigned w, unsigned h,
                        uint8_t *y, uint8_t *uv)
{
    uint32_t size = R(H26X_SEQ_CFG), lofs = R(H26X_SRC_LOFS);
    unsigned dw = MIN(size & 0xFFFF, w), dh = MIN(size >> 16, h);
    unsigned ylofs = (lofs & 0xFFFF) * 4, clofs = (lofs >> 16) * 4, r, x;
    bool swap = R(H26X_FUNC_CFG) & FUNC_SRC_CBCR_SWAP;

    if (dw < 2 || dh < 2 || ylofs < dw || clofs < dw) {
        return false;
    }
    for (r = 0; r < h; r++) {
        uint8_t *row = y + (size_t)r * w;

        if (r < dh) {
            if (dma_memory_read(&address_space_memory,
                                R(H26X_SRC_Y) + (hwaddr)r * ylofs, row, dw,
                                MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
                return false;
            }
        } else {
            memcpy(row, row - w, w);
        }
        memset(row + dw, row[dw - 1], w - dw);
    }
    for (r = 0; r < h / 2; r++) {
        uint8_t *row = uv + (size_t)r * w;

        if (r < (dh + 1) / 2) {
            if (dma_memory_read(&address_space_memory,
                                R(H26X_SRC_C) + (hwaddr)r * clofs, row,
                                dw & ~1u, MEMTXATTRS_UNSPECIFIED) !=
                MEMTX_OK) {
                return false;
            }
        } else {
            memcpy(row, row - w, w);
        }
        for (x = dw & ~1u; x + 1 < w; x += 2) {
            row[x] = row[(dw & ~1u) - 2];
            row[x + 1] = row[(dw & ~1u) - 1];
        }
        if (swap) {
            for (x = 0; x + 1 < w; x += 2) {
                uint8_t t = row[x];

                row[x] = row[x + 1];
                row[x + 1] = t;
            }
        }
    }
    return true;
}

/*
 * The encoder's slice data under the slice header that goes out: for
 * H.264 the driver's, for H.265 one built from the driver's and the
 * encoder's.  Appends the NAL to out; false if they do not fit together.
 */
static bool splice_avc(H26xStream *st, const H26xJob *job,
                       const uint8_t *enc, size_t enc_len, GByteArray *out)
{
    g_autoptr(GByteArray) drv = NULL, er = NULL;
    AvcSlice ds, es;
    BitWriter w = { .a = g_byte_array_new() };
    size_t pos = 0, s, e, stop;
    uint8_t nh = job->slice->data[0];

    drv = nal_rbsp(job->slice->data + 1, job->slice->len - 1);
    if (!avc_slice_parse(&ds, &st->asps, &st->apps, (nh >> 5) & 3, nh & 0x1F,
                         drv->data, drv->len) ||
        !nal_next(enc, enc_len, &pos, &s, &e)) {
        goto fail;
    }
    er = nal_rbsp(enc + s + 1, e - s - 1);
    if (!avc_slice_parse(&es, &st->easps, &st->eapps, (enc[s] >> 5) & 3,
                         enc[s] & 0x1F, er->data, er->len)) {
        goto fail;
    }
    /*
     * x264 turns deblocking off where it would change nothing (QP plus
     * twice the smaller offset at most 15: indexA / indexB give zero
     * thresholds; chroma's QP is no higher without a positive chroma
     * offset), so the driver's "on" is the same there.
     */
    if (es.deblock_idc == 1 && ds.deblock_idc == 0 &&
        ds.qp + 2 * MIN(ds.alpha, ds.beta) <= 15 &&
        st->apps.chroma_qp_offset <= 0 && st->apps.chroma_qp_offset2 <= 0) {
        es.deblock_idc = 0;
        es.alpha = ds.alpha;
        es.beta = ds.beta;
    }
    if (ds.slice_type != es.slice_type || ds.qp != es.qp ||
        ds.num_ref != es.num_ref || ds.cabac_init_idc != es.cabac_init_idc ||
        ds.deblock_idc != es.deblock_idc ||
        (ds.deblock_idc != 1 && (ds.alpha != es.alpha ||
                                 ds.beta != es.beta)) ||
        ds.first_mb || es.first_mb) {
        goto fail;
    }
    stop = rbsp_stop_bit(er->data, er->len);
    if (stop < es.data_bit) {
        goto fail;
    }
    /* the driver's header up to where it ends, alignment ones included */
    bw_copy(&w, drv->data, 0, ds.data_bit);
    bw_copy(&w, er->data, es.data_bit, stop);
    bw_trailing(&w);
    nal_put(out, &nh, 1, w.a->data, w.a->len);
    g_byte_array_free(w.a, true);
    return true;
fail:
    g_byte_array_free(w.a, true);
    return false;
}

static bool splice_hevc(H26xStream *st, const H26xJob *job,
                        const uint8_t *enc, size_t enc_len, GByteArray *out)
{
    const HevcSps *sps = &st->hsps;
    const HevcPps *pps = &st->hpps;
    g_autoptr(GByteArray) drv = NULL, er = NULL;
    HevcSlice ds, es;
    BitWriter w = { .a = g_byte_array_new() };
    size_t pos = 0, s, e, stop;
    const uint8_t *nh = job->slice->data;

    drv = nal_rbsp(nh + 2, job->slice->len - 2);
    if (!hevc_slice_parse(&ds, sps, pps, (nh[0] >> 1) & 0x3F, drv->data,
                          drv->len) || (!ds.rps_from_sps && !job->idr) ||
        !nal_next(enc, enc_len, &pos, &s, &e) || e - s < 3) {
        goto fail;
    }
    er = nal_rbsp(enc + s + 2, e - s - 2);
    if (!hevc_slice_parse(&es, &st->ehsps, &st->ehpps, (enc[s] >> 1) & 0x3F,
                          er->data, er->len) ||
        es.slice_type != ds.slice_type ||
        (es.slice_type == HEVC_P && es.num_ref != 1) ||
        (es.cabac_init && !pps->cabac_init) ||
        ((es.cb_qp_offset || es.cr_qp_offset) && !pps->slice_chroma_qp) ||
        (!pps->deblocking_override &&
         (es.deblocking_disabled != pps->deblocking_disabled ||
          (!es.deblocking_disabled && (es.beta != pps->beta ||
                                       es.tc != pps->tc))))) {
        goto fail;
    }
    /* slice_segment_header(), the first and only segment */
    bw_u(&w, 1, 1);
    if (ds.nal_type >= 16 && ds.nal_type <= 23) {
        bw_u(&w, ds.no_output_prior, 1);
    }
    bw_ue(&w, ds.pps_id);
    bw_u(&w, 0, pps->extra_bits);
    bw_ue(&w, es.slice_type);
    if (pps->output_flag) {
        bw_u(&w, 1, 1);
    }
    if (ds.nal_type != 19 && ds.nal_type != 20) {
        bw_u(&w, ds.poc_lsb, sps->log2_max_poc_lsb);
        bw_u(&w, 1, 1);
        if (sps->num_rps > 1) {
            bw_u(&w, ds.rps_idx, ceil_log2(sps->num_rps));
        }
        if (sps->tmvp) {
            bw_u(&w, es.tmvp, 1);
        }
    }
    if (sps->sao) {
        bw_u(&w, es.sao_luma, 1);
        if (sps->chroma_format) {
            bw_u(&w, es.sao_chroma, 1);
        }
    }
    if (es.slice_type == HEVC_P) {
        bw_u(&w, pps->num_ref_l0 != 1, 1);
        if (pps->num_ref_l0 != 1) {
            bw_ue(&w, 0);
        }
        if (pps->cabac_init) {
            bw_u(&w, es.cabac_init, 1);
        }
        bw_ue(&w, 5 - es.max_merge);
    }
    bw_se(&w, es.qp - pps->init_qp);
    if (pps->slice_chroma_qp) {
        bw_se(&w, es.cb_qp_offset);
        bw_se(&w, es.cr_qp_offset);
    }
    if (pps->deblocking_override) {
        bw_u(&w, 1, 1);
        bw_u(&w, es.deblocking_disabled, 1);
        if (!es.deblocking_disabled) {
            bw_se(&w, es.beta);
            bw_se(&w, es.tc);
        }
    }
    if (pps->lf_across_slices &&
        (es.sao_luma || es.sao_chroma || !es.deblocking_disabled)) {
        bw_u(&w, 0, 1);
    }
    if (pps->tiles || pps->entropy_sync) {
        bw_ue(&w, 0);
    }
    if (pps->slice_ext) {
        bw_ue(&w, 0);
    }
    bw_trailing(&w);                    /* byte_alignment() */
    stop = rbsp_stop_bit(er->data, er->len);
    if (stop < es.data_bit) {
        goto fail;
    }
    bw_copy(&w, er->data, es.data_bit, stop);
    bw_trailing(&w);
    nal_put(out, nh, 2, w.a->data, w.a->len);
    g_byte_array_free(w.a, true);
    return true;
fail:
    g_byte_array_free(w.a, true);
    return false;
}

/* the slice QP the driver's header gives the picture */
static bool job_qp(H26xStream *st, const H26xJob *job, int *qp)
{
    const uint8_t *nh = job->slice->data;
    g_autoptr(GByteArray) drv = NULL;

    if (st->h264) {
        AvcSlice sl;

        drv = nal_rbsp(nh + 1, job->slice->len - 1);
        if (!avc_slice_parse(&sl, &st->asps, &st->apps, (nh[0] >> 5) & 3,
                             nh[0] & 0x1F, drv->data, drv->len)) {
            return false;
        }
        *qp = sl.qp;
    } else {
        HevcSlice sl;

        drv = nal_rbsp(nh + 2, job->slice->len - 2);
        if (!hevc_slice_parse(&sl, &st->hsps, &st->hpps, (nh[0] >> 1) & 0x3F,
                              drv->data, drv->len)) {
            return false;
        }
        *qp = sl.qp;
    }
    return *qp >= 0 && *qp <= 51;
}

/* set the stream's encoder up for an IDR; false if it cannot match */
static bool stream_open(H26xStream *st, const H26xJob *job)
{
    unsigned w, h;

    stream_close(st);
    if (st->h264) {
        w = st->asps.mb_w * 16;
        h = st->asps.mb_h * 16;
    } else {
        w = st->hsps.width;
        h = st->hsps.height;
    }
    if (!w || !h || w > 8192 || h > 8192 || (w | h) & 15) {
        return false;
    }
    st->w = w;
    st->h = h;
    st->pts = 0;
#ifdef NVT_H26X_X264
    if (st->h264) {
        g_autoptr(GByteArray) drv = nal_rbsp(job->slice->data + 1,
                                             job->slice->len - 1);
        AvcSlice sl;
        uint8_t nh = job->slice->data[0];

        if (!avc_slice_parse(&sl, &st->asps, &st->apps, (nh >> 5) & 3,
                             nh & 0x1F, drv->data, drv->len) ||
            !enc264_open(st, &sl)) {
            return false;
        }
        if (!enc264_matches(st)) {
            unimp_once("x264 cannot match the driver's H.264 parameter sets");
            stream_close(st);
            return false;
        }
        return true;
    }
#endif
#ifdef NVT_H26X_X265
    if (!st->h264) {
        if (st->hpps.tiles) {
            unimp_once("tiled H.265");
            return false;
        }
        if (!enc265_open(st)) {
            return false;
        }
        if (!enc265_matches(st)) {
            unimp_once("x265 cannot match the driver's H.265 parameter sets");
            stream_close(st);
            return false;
        }
        return true;
    }
#endif
    unimp_once(st->h264 ? "built without x264" : "built without x265");
    return false;
}

/* a picture coded for real into out; false to fall back */
static bool encode_real(NvtH26x *h, const uint32_t *regs, bool h264,
                        const H26xJob *job, GByteArray *out, int *qp,
                        H26xStream **stp)
{
    H26xStream *st = stream_get(h, R(H26X_BSDMA), h264);

    *stp = st;
    g_autofree uint8_t *y = NULL, *uv = NULL;
    g_autoptr(GByteArray) enc = NULL;
    H26xPicture pic;
    bool ok;

    if (R(H26X_FUNC_CFG) & (FUNC_SRC_COMPRESS | FUNC_SRC_DIRECT)) {
        unimp_once("a compressed or direct source");
        return false;
    }
    if (job->idr) {
        if (!job->seq) {
            st->broken = true;
            return false;
        }
        if (!st->seq || st->seq->len != job->seq->len ||
            memcmp(st->seq->data, job->seq->data, job->seq->len)) {
            if (st->seq) {
                g_byte_array_free(st->seq, true);
            }
            st->seq = g_byte_array_new();
            g_byte_array_append(st->seq, job->seq->data, job->seq->len);
            stream_close(st);
            if (!stream_learn(st, st->seq)) {
                st->broken = true;
                return false;
            }
        }
        if (!stream_open(st, job)) {
            st->broken = true;
            return false;
        }
        st->broken = false;
    } else if (st->broken || !st->enc ||
               R(H26X_REF_Y) != st->last_rec) {
        /* the decoder's reference is not the encoder's */
        st->broken = true;
        return false;
    }
    st->last_rec = R(H26X_REC_Y);

    y = g_malloc((size_t)st->w * st->h);
    uv = g_malloc((size_t)st->w * st->h / 2);
    if (!read_source(regs, st->w, st->h, y, uv)) {
        st->broken = true;
        return false;
    }
    pic = (H26xPicture) {
        .y = y, .uv = uv, .w = st->w, .h = st->h, .idr = job->idr,
    };
    if (!job_qp(st, job, &pic.qp)) {
        st->broken = true;
        return false;
    }
    enc = encoder_picture(st, &pic);
    if (!enc) {
        st->broken = true;
        return false;
    }
    if (job->seq) {
        g_byte_array_append(out, job->seq->data, job->seq->len);
    }
    if (h264) {
        ok = splice_avc(st, job, enc->data, enc->len, out);
    } else {
        ok = splice_hevc(st, job, enc->data, enc->len, out);
    }
    if (!ok) {
        unimp_once("an encoder slice that does not fit the driver's header");
        st->broken = true;
        return false;
    }
    *qp = pic.qp;
    return true;
}

/* the placeholder: the sequence header, if any, then a filler NAL unit */
static void encode_filler(bool h264, const H26xJob *job, GByteArray *out)
{
    uint8_t nal[H26X_FILLER_LEN];

    if (job->seq) {
        g_byte_array_append(out, job->seq->data, job->seq->len);
    }
    memset(nal, 0xFF, sizeof(nal));
    nal[0] = 0;
    nal[1] = 0;
    nal[2] = 0;
    nal[3] = 1;
    if (h264) {
        nal[4] = 12;                    /* filler data */
    } else {
        nal[4] = 38 << 1;               /* FD_NUT */
        nal[5] = 1;
    }
    nal[H26X_FILLER_LEN - 1] = 0x80;    /* rbsp trailing bits */
    g_byte_array_append(out, nal, sizeof(nal));
}

void nvt_h26x_encode(NvtH26x *h, const uint32_t *regs, NvtH26xResult *r)
{
    bool h264 = R(H26X_FUNC_CFG) & FUNC_H264;
    uint32_t room = R(H26X_BSOUT_SIZE), size = R(H26X_SEQ_CFG);
    unsigned units;
    g_autoptr(GByteArray) out = g_byte_array_new();
    H26xStream *st = NULL;
    H26xJob job;
    int qp = R(0x0B8) & 0x3F;
    uint32_t le;

    memset(r, 0, sizeof(*r));
    if (!job_read(regs, h264, &job)) {
        unimp_once("a picture of several slices, or no header list");
        encode_filler(h264, &job, out);
    } else if (!encode_real(h, regs, h264, &job, out, &qp, &st)) {
        g_byte_array_set_size(out, 0);
        encode_filler(h264, &job, out);
    } else if (out->len > room) {
        /* the decoder will not have this picture to refer to */
        st->broken = true;
    }
    job_free(&job);
    if (out->len > room) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-h26x: a %u-byte picture for a "
                      "%u-byte bitstream buffer\n", out->len, room);
        r->full = true;
        return;
    }
    dma_memory_write(&address_space_memory, R(H26X_BSOUT_ADDR), out->data,
                     out->len, MEMTXATTRS_UNSPECIFIED);
    if (R(H26X_NAL_LEN_ADDR)) {
        le = cpu_to_le32(out->len);
        dma_memory_write(&address_space_memory, R(H26X_NAL_LEN_ADDR), &le, 4,
                         MEMTXATTRS_UNSPECIFIED);
    }
    units = h264 ? DIV_ROUND_UP(size & 0xFFFF, 16) *
                   DIV_ROUND_UP(size >> 16, 16)
                 : DIV_ROUND_UP(size & 0xFFFF, 64) *
                   DIV_ROUND_UP(size >> 16, 64);
    r->len = out->len;
    r->qp_sum = qp * units;
}
