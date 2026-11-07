/*
 * Hantro H2 HEVC encoder (VeriSilicon, the VC8000E's predecessor) as the
 * XM8520D's xm_h265e.ko ("hantroenc") and the H2 library in Sofia drive
 * it: 0x21000000, 0x7D0 bytes of registers, IRQ 77.
 *
 * Register words by byte offset (the VC8000E register table in
 * VeriSilicon's VPE SDK, sdk_inc/VC8000E/software/inc/registertable.h):
 *   +0x000  product ID: 0x4832 ("H2") in [31:16]; hantroenc accepts it
 *   +0x004  interrupt status: bit 0 IRQ, bit 2 frame ready, bit 1 IRQ off
 *   +0x014  bit 0 ENC_E: start; [31:22] width / 8, [21:11] height / 8,
 *           [2:1] frame coding type
 *   +0x140  hardware configuration (read only): HEVC, AXI, max width
 *
 * A job completes 2 ms after its start bit.  HEVC jobs (+0x010 [31:29] == 1), from the registers Sofia's library
 * programs (logged at run time) and the VC8000E table:
 *   +0x014  [31:22] width / 8, [21:11] height / 8, [2:1] 1 I, 0 P
 *   +0x018  [4:2] the SPS short-term RPS used (== their number: coded
 *           in the slice header from +0x3AC / +0x3B0)
 *   +0x01C  [13:8] slice QP
 *   +0x020  output pointer: the library has written the VPS, SPS and PPS
 *           in front of it (a whole access unit's parameter sets, at the
 *           start of the buffer), the engine writes the slice NAL unit --
 *           start code, NAL header, slice header and data
 *   +0x024  the buffer's room in bytes; on completion the bytes written
 *   +0x028  the size table: the NAL unit's size, a 32-bit word
 *   +0x02C  POC        +0x030/034  source luma / CbCr (NV12)
 *   +0x03C  the reconstruction (luma), +0x048 the reference's
 *   +0x2FC  [31:26] NAL unit type
 *   +0x348/34C [31:12] source luma / chroma stride
 * The slice header follows the parameter sets the library wrote (parsed
 * from the buffer; kept per picture size for jobs that come without
 * them); the slice data is hevc-enc.c's, with SAO off in the slice
 * header, its CUs 8x8 and the CTB the quantisation group.  A P picture's
 * reference is the decoded picture kept under the address of the
 * reconstruction that named it.  H.264 jobs (mode 2) are written the same
 * way with hantro-h264.c's slices (h2_h264), JPEG jobs (mode 4) as the
 * scan after the library's headers (h2_jpeg).  The registers of the first jobs are logged under
 * -d unimp.  dump=FILE (-global xm-h2.dump=FILE) appends every HEVC job,
 * the parameter sets in front of it included, to FILE-WxH-N.hevc (N: the
 * stream, by its reference chain; an IDR starts a new one).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "hevc-enc.h"
#include "hantro-h264.h"
#include "jpeg-enc.h"

#define TYPE_XM_H2 "xm-h2"
OBJECT_DECLARE_SIMPLE_TYPE(XmH2State, XM_H2)

#define H2_SIZE         0x1000
#define H2_ID           0x48321201u
#define H2_STATUS       0x004
#define H2_CTRL         0x014
#define H2_CONFIG       0x140
#define ST_IRQ          (1u << 0)
#define ST_IRQ_DIS      (1u << 1)
#define ST_FRAME_RDY    (1u << 2)

#define H2_MAX_W        4096
#define H2_MAX_H        2304
#define H2_REFS         32  /* 8 streams, up to 3 reconstructions each */
#define H2_PS_WINDOW    2048
#define H2_STREAMS      4

/* What the slice header and data have to follow, from the SPS / PPS */
typedef struct H2Ps {
    bool valid;
    int w, h;
    int log2_poc, log2_ctb, num_st_rps;
    bool sao, cabac_init_present, lf_across;
    int init_qp, cb_qp_offset, beta, tc, pps_id;
} H2Ps;

typedef struct H2Ref {
    uint32_t addr;
    HevcEncRef *pic;
    uint64_t used;
    uint32_t poc;
} H2Ref;

/* an H.264 decoded picture (NV12 at stride mbw * 16), by reconstruction */
typedef struct H2Ref264 {
    uint32_t addr;
    int mbw, mbh;
    uint8_t *y, *uv;
    uint64_t used;
} H2Ref264;

typedef struct H2Ps264 {
    int w, h;
    H264Enc e;
} H2Ps264;

struct XmH2State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[H2_SIZE / 4];
    unsigned jobs;
    H2Ps ps[H2_STREAMS];
    H2Ref refs[H2_REFS];
    H2Ref264 refs264[H2_REFS];
    H2Ps264 ps264[H2_STREAMS];
    uint64_t clock;
    uint8_t *src, *out;
    QEMUTimer *timer;
    uint32_t modes_seen;        /* modes whose first job was logged */           /* a job completes a while after its start */
    char *dump;                 /* append every HEVC job's stream here */
    struct { uint32_t rec; unsigned id; } chain[64];  /* dump: rec -> stream */
    unsigned chain_n, streams;
};

/* dump: the stream a job belongs to, following its reference chain */
static unsigned h2_stream_id(XmH2State *s, bool idr, uint32_t ref,
                             uint32_t rec)
{
    unsigned id = 0, i;
    bool found = false;

    if (!idr) {
        /* the newest entry for that picture: buffers are reused */
        for (i = 1; i <= MIN(s->chain_n, ARRAY_SIZE(s->chain)); i++) {
            unsigned k = (s->chain_n - i) % ARRAY_SIZE(s->chain);

            if (s->chain[k].rec == ref) {
                id = s->chain[k].id;
                found = true;
                break;
            }
        }
    }
    if (!found) {
        id = s->streams++;
    }
    s->chain[s->chain_n++ % ARRAY_SIZE(s->chain)].rec = rec;
    s->chain[(s->chain_n - 1) % ARRAY_SIZE(s->chain)].id = id;
    return id;
}

#define R(o) (s->regs[(o) / 4])

/* --- bits ---------------------------------------------------------- */

typedef struct BitR {
    const uint8_t *b;
    size_t n, p;
} BitR;

static unsigned br_u(BitR *r, int n)
{
    unsigned v = 0;

    while (n--) {
        v <<= 1;
        if (r->p < r->n * 8) {
            v |= (r->b[r->p >> 3] >> (7 - (r->p & 7))) & 1;
        }
        r->p++;
    }
    return v;
}

static unsigned br_ue(BitR *r)
{
    int z = 0;

    while (!br_u(r, 1) && z < 32) {
        z++;
    }
    return (1u << z) - 1 + br_u(r, z);
}

static int br_se(BitR *r)
{
    unsigned v = br_ue(r);

    return v & 1 ? (int)(v + 1) / 2 : -(int)(v / 2);
}

typedef struct BitW {
    uint8_t *b;
    size_t cap, p;
} BitW;

static void bw_u(BitW *w, unsigned v, int n)
{
    while (n--) {
        size_t i = w->p >> 3;

        if (i < w->cap) {
            if (!(w->p & 7)) {
                w->b[i] = 0;
            }
            w->b[i] |= ((v >> n) & 1) << (7 - (w->p & 7));
        }
        w->p++;
    }
}

static void bw_ue(BitW *w, unsigned v)
{
    int z = 0;

    while ((v + 1) >> (z + 1)) {
        z++;
    }
    bw_u(w, 0, z);
    bw_u(w, v + 1, z + 1);
}

static void bw_se(BitW *w, int v)
{
    bw_ue(w, v > 0 ? 2 * v - 1 : -2 * v);
}

/* RBSP of a NAL unit's payload: emulation prevention bytes removed */
static size_t h2_rbsp(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t i, o = 0;
    int z = 0;

    for (i = 0; i < n; i++) {
        if (z >= 2 && in[i] == 3) {
            z = 0;
            continue;
        }
        out[o++] = in[i];
        z = in[i] ? 0 : z + 1;
    }
    return o;
}

static void h2_skip_ptl(BitR *r, int max_sub)
{
    int i;

    br_u(r, 2 + 1 + 5 + 32 + 4 + 43 + 1 + 8);
    for (i = 0; i < max_sub; i++) {
        br_u(r, 2);
    }
    if (max_sub > 0) {
        for (i = max_sub; i < 8; i++) {
            br_u(r, 2);
        }
    }
}

static void h2_parse_sps(H2Ps *ps, BitR *r)
{
    int max_sub, i, min_cb;

    br_u(r, 4);
    max_sub = br_u(r, 3);
    br_u(r, 1);
    h2_skip_ptl(r, max_sub);
    br_ue(r);
    if (br_ue(r) == 3) {
        br_u(r, 1);
    }
    ps->w = br_ue(r);
    ps->h = br_ue(r);
    if (br_u(r, 1)) {
        br_ue(r); br_ue(r); br_ue(r); br_ue(r);
    }
    br_ue(r);
    br_ue(r);
    ps->log2_poc = br_ue(r) + 4;
    for (i = br_u(r, 1) ? 0 : max_sub; i <= max_sub; i++) {
        br_ue(r); br_ue(r); br_ue(r);
    }
    min_cb = br_ue(r) + 3;
    ps->log2_ctb = min_cb + br_ue(r);
    br_ue(r); br_ue(r); br_ue(r); br_ue(r);
    if (br_u(r, 1) && br_u(r, 1)) {
        ps->valid = false;          /* explicit scaling lists: not handled */
        return;
    }
    br_u(r, 1);                     /* amp */
    ps->sao = br_u(r, 1);
    if (br_u(r, 1)) {               /* pcm */
        br_u(r, 8); br_ue(r); br_ue(r); br_u(r, 1);
    }
    ps->num_st_rps = br_ue(r);
}

static void h2_parse_pps(H2Ps *ps, BitR *r)
{
    ps->pps_id = br_ue(r);
    br_ue(r);
    br_u(r, 1 + 1 + 3 + 1);         /* dependent, output flag, extra, SDH */
    ps->cabac_init_present = br_u(r, 1);
    br_ue(r); br_ue(r);
    ps->init_qp = 26 + br_se(r);
    br_u(r, 2);
    if (br_u(r, 1)) {
        br_ue(r);
    }
    ps->cb_qp_offset = br_se(r);
    br_se(r);
    br_u(r, 1 + 1 + 1 + 1 + 1 + 1);  /* slice qp offsets .. entropy sync */
    ps->lf_across = br_u(r, 1);
    ps->beta = ps->tc = 0;
    if (br_u(r, 1)) {
        br_u(r, 1);
        if (!br_u(r, 1)) {
            ps->beta = br_se(r);
            ps->tc = br_se(r);
        }
    }
}

/* The SPS and PPS in front of the output pointer, if any */
static bool h2_find_ps(XmH2State *s, hwaddr out, H2Ps *ps)
{
    uint8_t win[H2_PS_WINDOW], rb[H2_PS_WINDOW];
    hwaddr from = out > H2_PS_WINDOW ? out - H2_PS_WINDOW : 0;
    size_t n = out - from, i;
    bool sps = false, pps = false;

    address_space_read(&address_space_memory, from, MEMTXATTRS_UNSPECIFIED,
                       win, n);
    for (i = 0; i + 4 < n; i++) {
        size_t j, len;
        int type;
        BitR r;

        if (win[i] || win[i + 1] || win[i + 2] != 1) {
            continue;
        }
        for (j = i + 3; j + 2 < n; j++) {
            if (!win[j] && !win[j + 1] && win[j + 2] <= 1) {
                break;
            }
        }
        if (j + 2 >= n) {
            j = n;
        }
        if (j < i + 5) {
            break;
        }
        type = (win[i + 3] >> 1) & 0x3f;
        len = h2_rbsp(win + i + 5, j - (i + 5), rb);
        r = (BitR){ .b = rb, .n = len };
        if (type == 33) {
            ps->valid = true;
            h2_parse_sps(ps, &r);
            sps = ps->valid;
        } else if (type == 34 && sps) {
            h2_parse_pps(ps, &r);
            pps = true;
        }
        i = j - 1;
    }
    return sps && pps;
}

static HevcEncRef **h2_ref_slot(XmH2State *s, uint32_t addr, bool create)
{
    H2Ref *lru = &s->refs[0];
    int n;

    for (n = 0; n < H2_REFS; n++) {
        if (s->refs[n].pic && s->refs[n].addr == addr) {
            s->refs[n].used = ++s->clock;
            return &s->refs[n].pic;
        }
        if (s->refs[n].used < lru->used) {
            lru = &s->refs[n];
        }
    }
    if (!create) {
        return NULL;
    }
    if (!lru->pic) {
        lru->pic = hevc_enc_ref_new();
    }
    lru->addr = addr;
    lru->used = ++s->clock;
    return &lru->pic;
}

/* One HEVC picture: start code, NAL header, slice header, slice data */
static size_t h2_hevc(XmH2State *s, uint8_t *out, size_t cap)
{
    int w = (R(0x014) >> 22) * 8, h = ((R(0x014) >> 11) & 0x7ff) * 8;
    bool p_slice = ((R(0x014) >> 1) & 3) == 0;
    int qp = (R(0x01C) >> 8) & 0x3f, nal = R(0x2FC) >> 26;
    int rps = (R(0x018) >> 2) & 7, ys = R(0x348) >> 12, cs = R(0x34C) >> 12;
    hwaddr optr = R(0x020);
    H2Ps found = { 0 }, *ps = NULL;
    HevcEncParams prm = { 0 };
    HevcEncRef **ref = NULL, **rec;
    BitW bw = { .b = out + 6, .cap = cap - 6 };
    size_t hdr, n;
    int i;

    if (w < 8 || h < 8 || w > H2_MAX_W || h > H2_MAX_H || ys < w || cs < w ||
        ys > 8192 || cs > 8192 || cap < 64) {
        return 0;
    }
    /* the parameter sets: in front of the output, else this size's last */
    for (i = 0; i < H2_STREAMS; i++) {
        if (s->ps[i].valid && s->ps[i].w == w && s->ps[i].h == h) {
            ps = &s->ps[i];
        }
    }
    if (h2_find_ps(s, optr, &found) && found.w == w && found.h == h) {
        if (!ps) {
            ps = &s->ps[0];
            for (i = 0; i < H2_STREAMS; i++) {
                if (!s->ps[i].valid) {
                    ps = &s->ps[i];
                    break;
                }
            }
        }
        *ps = found;
    }
    if (!ps) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-h2: %dx%d job without an SPS\n",
                      w, h);
        return 0;
    }

    /* NAL unit header and slice_segment_header() */
    out[0] = out[1] = out[2] = 0;
    out[3] = 1;
    out[4] = nal << 1;
    out[5] = 1;
    bw_u(&bw, 1, 1);                    /* first_slice_segment_in_pic */
    if (nal >= 16 && nal <= 23) {
        bw_u(&bw, 0, 1);                /* no_output_of_prior_pics */
    }
    bw_ue(&bw, ps->pps_id);
    bw_ue(&bw, p_slice ? 1 : 2);
    if (nal != 19 && nal != 20) {
        bw_u(&bw, R(0x02C) & ((1u << ps->log2_poc) - 1), ps->log2_poc);
        if (rps < ps->num_st_rps) {
            bw_u(&bw, 1, 1);
            if (ps->num_st_rps > 1) {
                int bits = 0;

                while ((1 << bits) < ps->num_st_rps) {
                    bits++;
                }
                bw_u(&bw, rps, bits);
            }
        } else {
            /* st_ref_pic_set(num): the one reference before */
            bw_u(&bw, 0, 1);
            if (ps->num_st_rps) {
                bw_u(&bw, 0, 1);        /* inter_ref_pic_set_prediction */
            }
            bw_ue(&bw, p_slice ? 1 : 0);
            bw_ue(&bw, 0);
            if (p_slice) {
                bw_ue(&bw, 0);          /* delta_poc_s0_minus1 */
                bw_u(&bw, 1, 1);        /* used_by_curr_pic_s0 */
            }
        }
    }
    if (ps->sao) {
        bw_u(&bw, 0, 1);                /* slice_sao_luma_flag */
        bw_u(&bw, 0, 1);                /* slice_sao_chroma_flag */
    }
    if (p_slice) {
        bw_u(&bw, 0, 1);                /* num_ref_idx_active_override */
        if (ps->cabac_init_present) {
            bw_u(&bw, 0, 1);
        }
        bw_ue(&bw, 0);                  /* five_minus_max_num_merge_cand */
    }
    bw_se(&bw, qp - ps->init_qp);
    if (ps->lf_across) {
        bw_u(&bw, 1, 1);                /* loop filter across slices */
    }
    bw_u(&bw, 1, 1);                    /* byte_alignment() */
    while (bw.p & 7) {
        bw_u(&bw, 0, 1);
    }
    /* emulation prevention over the header */
    {
        uint8_t tmp[64];
        size_t k, m = 0, len = bw.p >> 3;
        int z = 0;

        memcpy(tmp, out + 6, MIN(len, sizeof(tmp)));
        for (k = 0; k < len && 6 + m + 2 < cap; k++) {
            if (z >= 2 && tmp[k] <= 3) {
                out[6 + m++] = 3;
                z = 0;
            }
            out[6 + m++] = tmp[k];
            z = tmp[k] ? 0 : z + 1;
        }
        hdr = 6 + m;
    }

    /* the source picture */
    address_space_read(&address_space_memory, R(0x030),
                       MEMTXATTRS_UNSPECIFIED, s->src, (size_t)ys * h);
    address_space_read(&address_space_memory, R(0x034),
                       MEMTXATTRS_UNSPECIFIED, s->src + (size_t)ys * h,
                       (size_t)cs * (h / 2));

    prm.log2_ctb = ps->log2_ctb;
    prm.chroma_qp_offset = ps->cb_qp_offset;
    prm.beta_offset_div2 = ps->beta;
    prm.tc_offset_div2 = ps->tc;
    prm.trafo_depth_intra = 2;
    if (p_slice) {
        ref = h2_ref_slot(s, R(0x048), false);
    }
    rec = h2_ref_slot(s, R(0x03C), true);
    if (p_slice && s->jobs < 400) {
        uint32_t rpoc = UINT32_MAX;

        for (i = 0; i < H2_REFS; i++) {
            if (s->refs[i].pic && s->refs[i].addr == R(0x048)) {
                rpoc = s->refs[i].poc;
            }
        }
        if (rpoc != R(0x02C) - 1) {
            qemu_log_mask(LOG_UNIMP, "xm-h2: P poc %u ref poc %d rps %u "
                          "3ac=%08x 3b0=%08x 44=%08x 16c=%08x\n", R(0x02C),
                          (int)rpoc, rps, R(0x3AC), R(0x3B0), R(0x044),
                          R(0x16C));
        }
    }
    for (i = 0; i < H2_REFS; i++) {
        if (s->refs[i].pic && &s->refs[i].pic == rec) {
            s->refs[i].poc = (nal >= 16 && nal <= 23) ? 0 : R(0x02C);
        }
    }
    n = hevc_enc_slice_data(&prm, w, h, qp, p_slice,
                            ref ? *ref : NULL, *rec, s->src, w, h, ys,
                            s->src + (size_t)ys * h, cs, out + hdr,
                            cap - hdr);
    return MIN(hdr + n, cap);
}


static H2Ref264 *h2_ref264(XmH2State *s, uint32_t addr, int mbw, int mbh,
                           bool create)
{
    H2Ref264 *lru = &s->refs264[0];
    int n;

    for (n = 0; n < H2_REFS; n++) {
        H2Ref264 *r = &s->refs264[n];

        if (r->y && r->addr == addr && r->mbw == mbw && r->mbh == mbh) {
            r->used = ++s->clock;
            return r;
        }
        if (r->used < lru->used) {
            lru = r;
        }
    }
    if (!create) {
        return NULL;
    }
    if (!lru->y || lru->mbw != mbw || lru->mbh != mbh) {
        g_free(lru->y);
        g_free(lru->uv);
        lru->y = g_malloc0((size_t)mbw * mbh * 256);
        lru->uv = g_malloc0((size_t)mbw * mbh * 128);
        lru->mbw = mbw;
        lru->mbh = mbh;
    }
    lru->addr = addr;
    lru->used = ++s->clock;
    return lru;
}

/* the SPS / PPS in front of the output: the bytes from the window's
 * first start code to the output pointer */
static size_t h2_front(XmH2State *s, hwaddr out, uint8_t *win)
{
    hwaddr from = out - MIN(out, H2_PS_WINDOW);

    address_space_read(&address_space_memory, from, MEMTXATTRS_UNSPECIFIED,
                       win, out - from);
    return out - from;
}

/*
 * One H.264 picture: start code, NAL header, slice header, slice data
 * (hantro-h264.c's), following the SPS / PPS the library wrote in front
 * of the output (kept per picture size).  frame_num is +0x02C, the POC
 * twice that; +0x304: [0] CABAC, [1] 8x8 transform, [2] idr_pic_id,
 * [3] nal_ref_idc.
 */
static size_t h2_h264(XmH2State *s, uint8_t *out, size_t cap)
{
    int w = (R(0x014) >> 22) * 8, h = ((R(0x014) >> 11) & 0x7ff) * 8;
    int mbw = w / 16, mbh = h / 16, i;
    bool p_slice = ((R(0x014) >> 1) & 3) == 0;
    int qp = (R(0x01C) >> 8) & 0x3f, nal = R(0x2FC) >> 26;
    int ys = R(0x348) >> 12, cs = R(0x34C) >> 12;
    uint32_t ctl = R(0x304), frame = R(0x02C);
    bool idr = nal == 5, ref_idc = ctl & 8;
    uint8_t win[H2_PS_WINDOW];
    H2Ps264 *ps = NULL;
    H264Enc *e;
    H2Ref264 *ref = NULL, *rec;
    H264Pic *pic;
    H264Deblock db = { 0, 0, 0 };
    H264PStats st;
    BitW bw = { .b = out + 5, .cap = cap - 5 };
    size_t n, k, hdr;

    if (w < 16 || h < 16 || (w & 15) || (h & 15) || w > H2_MAX_W ||
        h > H2_MAX_H || ys < w || cs < w || ys > 8192 || cs > 8192 ||
        cap < 64) {
        return 0;
    }
    for (i = 0; i < H2_STREAMS; i++) {
        if (s->ps264[i].w == w && s->ps264[i].h == h) {
            ps = &s->ps264[i];
        }
    }
    if (!ps) {
        for (i = 0; i < H2_STREAMS && s->ps264[i].w; i++) {
        }
        ps = &s->ps264[i < H2_STREAMS ? i : 0];
        memset(ps, 0, sizeof(*ps));
        ps->w = w;
        ps->h = h;
    }
    e = &ps->e;
    /*
     * Fresh headers only when the window ends in SPS + PPS: the last start
     * code in front of the output is a PPS, the one before it an SPS.  A
     * P picture has none there (older stream bytes), so the cached set of
     * this picture size stays.
     */
    n = h2_front(s, R(0x020), win);
    {
        size_t sc[2] = { SIZE_MAX, SIZE_MAX };

        for (k = n - MIN(n, 4); k-- > 0;) {
            if (!win[k] && !win[k + 1] && win[k + 2] == 1) {
                if (sc[0] == SIZE_MAX) {
                    sc[0] = k;
                } else {
                    sc[1] = k;
                    break;
                }
            }
        }
        if (sc[1] != SIZE_MAX && (win[sc[0] + 3] & 0x1f) == 8 &&
            (win[sc[1] + 3] & 0x1f) == 7) {
            H264Enc t;

            memset(&t, 0, sizeof(t));
            h264_enc_parse_headers(&t, win + sc[1], n - sc[1]);
            if (t.have_sps && t.have_pps) {
                *e = t;
            }
        }
    }
    if (!e->have_sps || !e->have_pps || e->unsupported) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-h2: %dx%d H.264 job without "
                      "usable SPS / PPS\n", w, h);
        return 0;
    }

    out[0] = out[1] = out[2] = 0;
    out[3] = 1;
    out[4] = (ref_idc ? 3 : 0) << 5 | nal;
    bw_ue(&bw, 0);                          /* first_mb_in_slice */
    bw_ue(&bw, p_slice ? 5 : 7);            /* slice_type: all P / all I */
    bw_ue(&bw, e->pps_id);
    bw_u(&bw, idr ? 0 : frame & ((1u << e->log2_max_frame_num) - 1),
         e->log2_max_frame_num);
    if (idr) {
        bw_ue(&bw, (ctl >> 2) & 1);         /* idr_pic_id */
    }
    if (e->poc_type == 0) {
        bw_u(&bw, idr ? 0 : (2 * frame) & ((1u << e->log2_max_poc_lsb) - 1),
             e->log2_max_poc_lsb);
        if (e->bottom_field_poc_present) {
            bw_se(&bw, 0);
        }
    }
    if (e->redundant_pic_cnt_present) {
        bw_ue(&bw, 0);
    }
    if (p_slice) {
        bw_u(&bw, 0, 1);                    /* num_ref_idx_active_override */
        bw_u(&bw, 0, 1);                    /* ref_pic_list_modification */
    }
    if (ref_idc) {
        bw_u(&bw, 0, 1);                    /* no_output_of_prior_pics /
                                             * adaptive_ref_pic_marking */
        if (idr) {
            bw_u(&bw, 0, 1);                /* long_term_reference_flag */
        }
    }
    if (e->cabac && p_slice) {
        bw_ue(&bw, 0);                      /* cabac_init_idc */
    }
    bw_se(&bw, qp - e->pic_init_qp);
    if (e->deblocking_control) {
        bw_ue(&bw, 0);                      /* deblocking on, offsets 0 */
        bw_se(&bw, 0);
        bw_se(&bw, 0);
    }

    address_space_read(&address_space_memory, R(0x030),
                       MEMTXATTRS_UNSPECIFIED, s->src, (size_t)ys * h);
    address_space_read(&address_space_memory, R(0x034),
                       MEMTXATTRS_UNSPECIFIED, s->src + (size_t)ys * h,
                       (size_t)cs * (h / 2));
    if (p_slice) {
        ref = h2_ref264(s, R(0x048), mbw, mbh, false);
    }
    rec = h2_ref264(s, R(0x03C), mbw, mbh, true);
    pic = h264_pic_new(mbw, mbh, s->src, ys, s->src + (size_t)ys * h, cs,
                       ref ? ref->y : NULL, ref ? ref->uv : NULL,
                       e->chroma_qp_offset, ctl & 2);
    if (e->cabac) {
        bw_u(&bw, 1, 1);                    /* cabac_alignment_one_bits */
        while (bw.p & 7) {
            bw_u(&bw, 1, 1);
        }
        hdr = 5 + (bw.p >> 3);
        n = h264_pic_slice_cabac(pic, p_slice, 0, mbh, qp, 0, out + hdr,
                                 cap - hdr, &st);
        n = hdr + n;
    } else {
        /* the slice data continues the header's last, partial byte */
        hdr = 5 + (bw.p >> 3);
        n = h264_pic_slice_cavlc(pic, p_slice, 0, mbh, qp, bw.p & 7,
                                 out + hdr, cap - hdr, &st);
        n = hdr + (n + 7) / 8;
    }
    h264_pic_finish(pic, &db, rec->y, rec->uv);
    h264_pic_free(pic);
    /* emulation prevention over the whole NAL unit after its header */
    {
        g_autofree uint8_t *tmp = g_memdup2(out + 5, MIN(n, cap) - 5);
        size_t m = 5, len = MIN(n, cap) - 5;
        int z = 0;

        for (k = 0; k < len && m + 2 < cap; k++) {
            if (z >= 2 && tmp[k] <= 3) {
                out[m++] = 3;
                z = 0;
            }
            out[m++] = tmp[k];
            z = tmp[k] ? 0 : z + 1;
        }
        n = m;
    }
    return MIN(n, cap);
}

static const uint8_t h2_zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/*
 * A JPEG picture: the library wrote SOI .. SOS in front of the output
 * (+0x020); the engine writes the entropy-coded scan (Annex K tables)
 * and EOI.  +0x014 [31:20] / [19:8] width / height in 8s, the source
 * NV12 at +0x030 / +0x034 with the luma stride in +0x050 [31:17] (else
 * +0x348 [31:12]).  The
 * quantisers come from the DQT in front.
 */
static size_t h2_jpeg(XmH2State *s, uint8_t *out, size_t cap)
{
    int w = (R(0x014) >> 20) * 8, h = ((R(0x014) >> 8) & 0xfff) * 8;
    int ys = R(0x348) >> 12, cs = R(0x34C) >> 12, x, y;
    uint8_t win[H2_PS_WINDOW], q[2][64];
    bool have[2] = { false, false }, ovf;
    uint8_t *cb, *cr;
    size_t n, k, len, rem;

    if (R(0x050) >> 17) {                   /* JPEG row length (pixels) */
        ys = R(0x050) >> 17;
    }
    if (!cs) {
        cs = ys;
    }
    if (w < 8 || h < 8 || w > H2_MAX_W || h > H2_MAX_H || ys < w ||
        ys > 8192 || cs < w) {
        return 0;
    }
    n = h2_front(s, R(0x020), win);
    for (k = 0; k + 4 < n; k++) {
        if (win[k] != 0xff || win[k + 1] != 0xdb) {
            continue;
        }
        len = win[k + 2] << 8 | win[k + 3];
        for (size_t t = k + 4; t + 65 <= k + 2 + len && t + 65 <= n; t += 65) {
            int id = win[t] & 3, i;

            if ((win[t] >> 4) || id > 1) {
                break;
            }
            for (i = 0; i < 64; i++) {
                q[id][h2_zigzag[i]] = win[t + 1 + i];
            }
            have[id] = true;
        }
    }
    if (!have[0]) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-h2: JPEG job without a DQT\n");
        return 0;
    }
    if (!have[1]) {
        memcpy(q[1], q[0], 64);
    }
    address_space_read(&address_space_memory, R(0x030),
                       MEMTXATTRS_UNSPECIFIED, s->src, (size_t)ys * h);
    cb = s->src + (size_t)ys * h;
    cr = cb + (size_t)(w / 2) * (h / 2);
    {
        g_autofree uint8_t *uv = g_malloc((size_t)cs * (h / 2));

        address_space_read(&address_space_memory, R(0x034),
                           MEMTXATTRS_UNSPECIFIED, uv, (size_t)cs * (h / 2));
        for (y = 0; y < h / 2; y++) {
            for (x = 0; x < w / 2; x++) {
                cb[y * (w / 2) + x] = uv[y * cs + 2 * x];
                cr[y * (w / 2) + x] = uv[y * cs + 2 * x + 1];
            }
        }
    }
    /*
     * +0x048 [31:26]: the header's last bits the library could not write
     * byte-aligned (the SOS tail), MSB first in +0x04C then +0x054; the
     * engine writes them ahead of the scan.
     */
    rem = MIN(R(0x048) >> 26, 64) / 8;
    if (rem >= cap) {
        return 0;
    }
    for (k = 0; k < rem; k++) {
        out[k] = (k < 4 ? R(0x04C) : R(0x054)) >> (24 - 8 * (k & 3));
    }
    n = jpeg_enc_scan(s->src, ys, cb, cr, w / 2, w, h, q[0], q[1], true,
                      out + rem, cap - rem, &ovf);
    return MIN(rem + n, cap);
}

static void h2_job_done(XmH2State *s)
{
    uint32_t mode = R(0x010) >> 29;
    size_t cap = R(0x024), n = 0;

    if ((mode == 1 || mode == 2 || mode == 4) && R(0x020)) {
        cap = MIN(cap, (size_t)H2_MAX_W * H2_MAX_H * 2);
        n = mode == 1 ? h2_hevc(s, s->out, cap) :
            mode == 2 ? h2_h264(s, s->out, cap) : h2_jpeg(s, s->out, cap);
        address_space_write(&address_space_memory, R(0x020),
                            MEMTXATTRS_UNSPECIFIED, s->out, n);
        if (s->dump && n) {
            unsigned id = h2_stream_id(s, (R(0x2FC) >> 26) >= 16, R(0x048),
                                       R(0x03C));
            g_autofree char *path = g_strdup_printf("%s-%ux%u-%u.%s",
                                                    s->dump,
                                                    (R(0x014) >> 22) * 8,
                                                    ((R(0x014) >> 11) & 0x7ff) * 8,
                                                    id, mode == 1 ? "hevc" :
                                                    mode == 2 ? "h264" : "jpg");
            FILE *f = fopen(path, "ab");

            if (f) {
                uint8_t win[H2_PS_WINDOW];
                hwaddr from = R(0x020) - MIN(R(0x020), H2_PS_WINDOW);
                size_t k, wn = R(0x020) - from;

                /* the library's parameter sets in front, from the first
                 * start code */
                address_space_read(&address_space_memory, from,
                                   MEMTXATTRS_UNSPECIFIED, win, wn);
                for (k = 0; k + 4 <= wn; k++) {
                    if ((mode != 4 && !win[k] && !win[k + 1] && !win[k + 2] &&
                         win[k + 3] == 1) ||
                        (mode == 4 && win[k] == 0xff && win[k + 1] == 0xd8)) {
                        fwrite(win + k, 1, wn - k, f);
                        break;
                    }
                }
                fwrite(s->out, 1, n, f);
                fclose(f);
            }
        }
    }
    if (s->jobs < 60 || mode != 1) {
        qemu_log_mask(LOG_UNIMP, "xm-h2: job %u mode %u %ux%u type %u nal %u "
                      "poc %u rps %u qp %u ref %08x rec %08x out %08x room %u "
                      "-> %zu\n", s->jobs, mode, (R(0x014) >> 22) * 8,
                      ((R(0x014) >> 11) & 0x7ff) * 8, (R(0x014) >> 1) & 3,
                      R(0x2FC) >> 26, R(0x02C), (R(0x018) >> 2) & 7,
                      (R(0x01C) >> 8) & 0x3f, R(0x048), R(0x03C), R(0x020),
                      R(0x024), n);
    }
    R(0x024) = n;
    if (R(0x028)) {
        uint32_t le = cpu_to_le32(n);

        address_space_write(&address_space_memory, R(0x028),
                            MEMTXATTRS_UNSPECIFIED, &le, 4);
    }
}

static void xm_h2_update(XmH2State *s)
{
    uint32_t st = s->regs[H2_STATUS / 4];

    qemu_set_irq(s->irq, (st & ST_IRQ) && !(st & ST_IRQ_DIS));
}

static void xm_h2_run(XmH2State *s)
{
    int n;

    if (s->jobs < 2 || !(s->modes_seen & (1u << (R(0x010) >> 29)))) {
        s->modes_seen |= 1u << (R(0x010) >> 29);
        qemu_log_mask(LOG_UNIMP, "xm-h2: job %u:", s->jobs);
        for (n = 0; n < H2_SIZE / 4; n++) {
            if (s->regs[n]) {
                qemu_log_mask(LOG_UNIMP, " %x=%08x", n * 4, s->regs[n]);
            }
        }
        qemu_log_mask(LOG_UNIMP, "\n");
    }
    s->jobs++;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2 * SCALE_MS);
}

static void xm_h2_complete(void *opaque)
{
    XmH2State *s = opaque;

    h2_job_done(s);
    s->regs[H2_CTRL / 4] &= ~1u;
    s->regs[H2_STATUS / 4] |= ST_IRQ | ST_FRAME_RDY;
    xm_h2_update(s);
}

static uint64_t xm_h2_read(void *opaque, hwaddr off, unsigned size)
{
    XmH2State *s = opaque;

    switch (off) {
    case 0:
        return H2_ID;
    case H2_CONFIG:
        /* H.264, HEVC, JPEG, AXI (3), 64-bit bus, 4096 wide: Sofia creates
         * its H.264 and snapshot channels on this core too */
        return 1u << 31 | 1u << 27 | 1u << 15 | 3u << 21 | 1u << 13 | 4096;
    }
    return s->regs[off / 4];
}

static void xm_h2_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    XmH2State *s = opaque;

    if (off == 0 || off == H2_CONFIG) {
        return;
    }
    s->regs[off / 4] = val;
    if (off == H2_CTRL && (val & 1)) {
        xm_h2_run(s);
    }
    xm_h2_update(s);
}

static const MemoryRegionOps xm_h2_ops = {
    .read = xm_h2_read,
    .write = xm_h2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void xm_h2_reset(DeviceState *dev)
{
    XmH2State *s = XM_H2(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void xm_h2_init(Object *obj)
{
    XmH2State *s = XM_H2(obj);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_h2_complete, s);
    s->src = g_malloc((size_t)H2_MAX_W * H2_MAX_H * 2);
    s->out = g_malloc((size_t)H2_MAX_W * H2_MAX_H * 2);
    memory_region_init_io(&s->iomem, obj, &xm_h2_ops, s, TYPE_XM_H2, H2_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property xm_h2_props[] = {
    DEFINE_PROP_STRING("dump", XmH2State, dump),
};

static void xm_h2_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_h2_reset);
    device_class_set_props(DEVICE_CLASS(klass), xm_h2_props);
}

static const TypeInfo xm_h2_info = {
    .name = TYPE_XM_H2,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmH2State),
    .instance_init = xm_h2_init,
    .class_init = xm_h2_class_init,
};

static void xm_h2_register(void)
{
    type_register_static(&xm_h2_info);
}

type_init(xm_h2_register)
