/*
 * MStar infinity3 MFE: the H.264 encoder (vendor driver drivers/mstar/
 * mfev5 with mfe6, "mstar,mfe", 0x1F264800, GIC SPI 60).
 *
 * Register-driven, no firmware.  16-bit registers on the RIU's 4-byte
 * stride; 0x00-0x7F are bank 0x1324 and 0x80-0xFF bank 0x1325, so the
 * window is 0x400 bytes.  What the driver (mfe6_ios.c, mfe6_ops.c,
 * h264_enc.c in the I3 SDK MSC3XX_SDK_V_1711291828) does per frame:
 *
 *   - 0x00 bit 1 soft_rstz low then high, the configuration (0x01/0x02
 *     picture width and height, 16-aligned; 0x00 [5:4] frame type, 0 = I,
 *     1 = P; 0x03 bit 13 CABAC; 0x06-0x09 the source Y/C, 0x0A/0x0B and
 *     0x0E/0x0F the reference, 0x12-0x15 the reconstruction, MIU address
 *     >> 8 in a low and a high register; 0x90 the source pitch in bytes;
 *     0xF3 bit 0 the source format, 0 = NV12, 1 = YUYV, bit 15 chroma
 *     swap), and the output buffer: an address in 8-byte units through
 *     0x3C (low) and 0x3D [12:0] (high) with [15:14] naming it (0 = start,
 *     1 = end), latched by a write of bit 0 to 0x3E;
 *   - 0x00 bit 0, frame start (it reads back 0);
 *   - then the headers: the driver writes SPS, PPS and the whole slice
 *     header itself and feeds them to the FDC bit packer, MSB first, 16
 *     bits per 0x46 word: 0x47 [14:5] holds the number of words - 1 and
 *     [4:0] the length of the last, 0x48 bit 11 pushes a word.  The
 *     hardware's output is those bits followed by the slice data.
 *
 * When the last header word is in, the model encodes the slice data,
 * CAVLC or CABAC as 0x03 bit 13 says (the PPS's entropy_coding_mode_flag;
 * with CABAC the driver's slice header ends in its cabac_alignment_one
 * bits), and writes header + data + the trailing bits to the output
 * buffer, with emulation prevention carried across the join (the
 * header's whole bytes come escaped already).  What the slice data needs
 * from the header comes from the registers the driver fills from the
 * same values (mfe6_ops.c _enc_conf()): the slice QP is 0x00 [13:8]
 * (qscale, = pic_init_qp + slice_qp_delta), the deblocking control 0x39
 * bit 5 (deblocking_filter_control_present_flag) and 0x3A [4:3]
 * (disable_deblocking_filter_idc), [8:5] and [12:9] (the alpha and beta
 * offsets / 2), chroma_qp_index_offset 0x29 [12:8]; cabac_init_idc is
 * always 0 (h264_enc.c writes sh->i_cabac_init_idc, which nothing sets).
 * 0x42 and 0x43 [7:0] report the bits written, 0x44/0x45 the write
 * pointer, and 0x1E bit 0 is frame done (bit 1 too when the buffer was
 * too small).  0x1C [7:0] masks (1 = masked) and [15:8] forces, 0x1D
 * [7:0] clears; 0x1E [7:0] is the status past the mask, [15:8] the raw
 * one.  The line is high while 0x1E [7:0] is non-zero.
 *
 * The coding needs no library and keeps the picture: in I slices a
 * macroblock is Intra16x16 with no residual when one of the V, H or DC
 * predictions (chroma the same) reproduces it exactly, which the test
 * pattern's navy does (its gradient runs down the picture, so H is exact),
 * and I_PCM otherwise; in P slices a
 * macroblock within "skip-tolerance" (2) of the reference's in every
 * sample is P_Skip and any other is I_PCM, up to an eighth of the
 * picture or 64 (and what fits in the output buffer); the rest stay
 * P_Skip and catch up in the next frames, which start looking where this
 * one stopped.  A whole-picture change (an exposure or white balance step
 * of the ISP's 3A) would otherwise make a P frame of over 1 MiB, which
 * overflows the driver's buffer and breaks the camera's RTSP stream.  A
 * P_Skip macroblock decodes to the reference's samples, and so does the
 * model's picture.  In CABAC the same choices are coded with the standard's contexts
 * (9.3; I_PCM flushes the arithmetic coder and restarts it after the
 * samples), sstar-cabac.h's coder, which sstar-vhe shares.  The model
 * runs the deblocking filter (8.7) over the picture as the decoder does,
 * writes the result to the reconstruction buffer as NV12 and reads the
 * previous one back through the reference address, so each channel (main
 * stream, sub stream) keeps its own and the stream decodes to what the
 * model holds, without drift.  "dump=DIR" appends each frame's output to
 * DIR/mfe-<width>x<height>.h264, a file per picture size (channels of
 * the same size share one).  When the driver inserts user data (SEI) it
 * writes the parameter sets, the SEI and the slice header into the buffer
 * itself and points the output address past their whole 256-byte blocks,
 * feeding only the rest through the FDC (mfe6_ops.c _put_data()); the
 * stream in memory is whole, the dump of such a frame lacks that start.
 * The source is read from DRAM; when its address is outside DRAM (the scaler streaming
 * straight into the encoder through IMI, the 64 MiB parts'
 * mi.sys.shrink_mem) it is the test pattern itself.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/sstar-cabac.h"
#include "aw-vinc-logo.h"
#include "hw/misc/sstar-pattern.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_SSTAR_MFE "sstar-mfe"
OBJECT_DECLARE_SIMPLE_TYPE(SstarMfeState, SSTAR_MFE)

#define MFE_REGS            0x100

#define R_CTRL              0x00
#define R_WIDTH             0x01
#define R_HEIGHT            0x02
#define R_MODE              0x03
#define R_CUR_Y             0x06
#define R_CUR_C             0x08
#define R_REF_Y             0x0A
#define R_REF_C             0x0E
#define R_REC_Y             0x12
#define R_REC_C             0x14
#define R_IRQ_MASK          0x1C
#define R_IRQ_CLR           0x1D
#define R_IRQ_STATUS        0x1E
#define R_CQP_OFFSET        0x29
#define R_DBF_CTRL          0x39
#define R_DBF_IDC           0x3A
#define R_OBUF_LO           0x3C
#define R_OBUF_HI           0x3D
#define R_OBUF_STROBE       0x3E
#define R_BITS_LO           0x42
#define R_BITS_HI           0x43
#define R_WPTR_LO           0x44
#define R_WPTR_HI           0x45
#define R_FDC_BS            0x46
#define R_FDC_LEN           0x47
#define R_FDC_CTRL          0x48
#define R_CYCLES_LO         0x76
#define R_CYCLES_HI         0x77
#define R_PITCH_Y           0x90
#define R_SRC_FMT           0xF3

#define CTRL_START          (1u << 0)
#define CTRL_RSTZ           (1u << 1)
#define MODE_CABAC          (1u << 13)
#define DBF_CONTROL_PRESENT (1u << 5)
#define FDC_VLD             (1u << 11)
#define IRQ_DONE            (1u << 0)
#define IRQ_BUF_FULL        (1u << 1)

#define DRAM_BASE           0x20000000
#define MAX_MBS             (2688 / 16 * 2048 / 16)
#define HDR_MAX_BITS        (64 * 1024)
#define ENCODE_NS           (5 * SCALE_MS)

struct SstarMfeState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint16_t regs[MFE_REGS];
    uint8_t raw;                    /* interrupt status before the mask */
    uint32_t obuf[2];               /* start and end, MIU byte addresses */
    bool started;
    uint32_t hdr_words;             /* FDC words pushed this frame */
    uint32_t hdr_bits;
    uint8_t hdr[HDR_MAX_BITS / 8];
    uint32_t frames;
    uint64_t dram_size;
    bool trace;
    char *dump;
    uint8_t skip_tol;
    uint32_t pcm_next;                  /* where P frames look for changes */
};

static void mfe_update(SstarMfeState *s)
{
    qemu_set_irq(s->irq, !!(s->raw & ~s->regs[R_IRQ_MASK] & 0xFF));
}

static uint32_t mfe_addr(SstarMfeState *s, int reg, int shift)
{
    return (s->regs[reg] | (uint32_t)(s->regs[reg + 1] & 0xFF) << 16)
           << shift;
}

/* A picture: Y then interleaved CbCr (NV12), the luma width as pitch. */
typedef struct MfePic {
    int w, h;
    uint8_t *y, *c;
} MfePic;

static void mfe_pic_alloc(MfePic *p, int w, int h)
{
    p->w = w;
    p->h = h;
    p->y = g_malloc0((size_t)w * h);
    p->c = g_malloc0((size_t)w * h / 2);
}

static void mfe_pic_free(MfePic *p)
{
    g_free(p->y);
    g_free(p->c);
}

static bool mfe_in_dram(SstarMfeState *s, uint32_t miu, size_t len)
{
    return miu + len <= s->dram_size;
}

static void mfe_load_source(SstarMfeState *s, MfePic *p)
{
    uint32_t ya = mfe_addr(s, R_CUR_Y, 8), ca = mfe_addr(s, R_CUR_C, 8);
    uint32_t pitch = s->regs[R_PITCH_Y];
    bool yuyv = s->regs[R_SRC_FMT] & 1;
    bool swap = s->regs[R_SRC_FMT] & 0x8000;
    g_autofree uint8_t *line = NULL;
    int x, y;

    if (!pitch) {
        pitch = yuyv ? 2 * p->w : p->w;
    }
    if (!mfe_in_dram(s, ya, (size_t)pitch * p->h)) {
        /* fed through IMI by the scaler: the picture it would carry */
        uint8_t px[3];

        for (y = 0; y < p->h; y++) {
            for (x = 0; x < p->w; x++) {
                sstar_pattern_pixel(s->frames, p->w, p->h, x, y, px);
                p->y[y * p->w + x] = px[0];
                if (!(x & 1) && !(y & 1)) {
                    p->c[(y / 2) * p->w + x] = px[1];
                    p->c[(y / 2) * p->w + x + 1] = px[2];
                }
            }
        }
        return;
    }
    line = g_malloc(pitch);
    for (y = 0; y < p->h; y++) {
        address_space_read(&address_space_memory,
                           DRAM_BASE + ya + (hwaddr)pitch * y,
                           MEMTXATTRS_UNSPECIFIED, line, pitch);
        if (yuyv) {
            for (x = 0; x < p->w; x++) {
                p->y[y * p->w + x] = line[2 * x];
                if (!(y & 1) && !(x & 1)) {
                    p->c[(y / 2) * p->w + x] = line[2 * x + (swap ? 3 : 1)];
                    p->c[(y / 2) * p->w + x + 1] =
                        line[2 * x + (swap ? 1 : 3)];
                }
            }
        } else {
            memcpy(p->y + y * p->w, line, MIN(pitch, (uint32_t)p->w));
        }
    }
    if (yuyv) {
        return;
    }
    for (y = 0; y < p->h / 2; y++) {
        address_space_read(&address_space_memory,
                           DRAM_BASE + ca + (hwaddr)pitch * y,
                           MEMTXATTRS_UNSPECIFIED, line, pitch);
        memcpy(p->c + y * p->w, line, MIN(pitch, (uint32_t)p->w));
        if (swap) {
            for (x = 0; x < p->w; x += 2) {
                uint8_t t = p->c[y * p->w + x];

                p->c[y * p->w + x] = p->c[y * p->w + x + 1];
                p->c[y * p->w + x + 1] = t;
            }
        }
    }
}

/*
 * The first edition of H.264 has no PCM sample 0; the source goes from 0
 * to 1 so that the macroblocks coded I_PCM decode to it as well.
 */
static void mfe_pic_no_zero(MfePic *p)
{
    size_t i, n = (size_t)p->w * p->h;

    for (i = 0; i < n; i++) {
        p->y[i] = MAX(p->y[i], 1);
    }
    for (i = 0; i < n / 2; i++) {
        p->c[i] = MAX(p->c[i], 1);
    }
}

/* A picture to or from DRAM, NV12 at the address in @yreg and @creg. */
static void mfe_pic_rw(SstarMfeState *s, MfePic *p, int yreg, int creg,
                       bool write)
{
    uint32_t ya = mfe_addr(s, yreg, 8), ca = mfe_addr(s, creg, 8);
    size_t ylen = (size_t)p->w * p->h;

    if (!mfe_in_dram(s, ya, ylen) || !mfe_in_dram(s, ca, ylen / 2)) {
        return;
    }
    if (write) {
        address_space_write(&address_space_memory, DRAM_BASE + ya,
                            MEMTXATTRS_UNSPECIFIED, p->y, ylen);
        address_space_write(&address_space_memory, DRAM_BASE + ca,
                            MEMTXATTRS_UNSPECIFIED, p->c, ylen / 2);
    } else {
        address_space_read(&address_space_memory, DRAM_BASE + ya,
                           MEMTXATTRS_UNSPECIFIED, p->y, ylen);
        address_space_read(&address_space_memory, DRAM_BASE + ca,
                           MEMTXATTRS_UNSPECIFIED, p->c, ylen / 2);
    }
}

static uint8_t mfe_cb(const MfePic *p, int x, int y)
{
    return p->c[y * p->w + 2 * x];
}

static uint8_t mfe_cr(const MfePic *p, int x, int y)
{
    return p->c[y * p->w + 2 * x + 1];
}

/* Does a 16x16 Intra prediction mode (0 V, 1 H, 2 DC) give the MB? */
static bool mfe_luma_pred_ok(const MfePic *p, int mx, int my, int mode)
{
    int x0 = mx * 16, y0 = my * 16, x, y, sum = 0, n = 0, dc = 128;

    if ((mode == 0 && !my) || (mode == 1 && !mx)) {
        return false;
    }
    if (mode == 2) {
        if (my) {
            for (x = 0; x < 16; x++) {
                sum += p->y[(y0 - 1) * p->w + x0 + x];
            }
            n += 16;
        }
        if (mx) {
            for (y = 0; y < 16; y++) {
                sum += p->y[(y0 + y) * p->w + x0 - 1];
            }
            n += 16;
        }
        if (n) {
            dc = (sum + n / 2) / n;
        }
    }
    for (y = 0; y < 16; y++) {
        for (x = 0; x < 16; x++) {
            int pred = mode == 0 ? p->y[(y0 - 1) * p->w + x0 + x] :
                       mode == 1 ? p->y[(y0 + y) * p->w + x0 - 1] : dc;

            if (p->y[(y0 + y) * p->w + x0 + x] != pred) {
                return false;
            }
        }
    }
    return true;
}

/* The same for chroma (intra_chroma_pred_mode 0 DC, 1 H, 2 V). */
static bool mfe_chroma_pred_ok(const MfePic *p, int mx, int my, int mode)
{
    uint8_t (*get[2])(const MfePic *, int, int) = { mfe_cb, mfe_cr };
    int x0 = mx * 8, y0 = my * 8, c, x, y;

    if ((mode == 2 && !my) || (mode == 1 && !mx)) {
        return false;
    }
    for (c = 0; c < 2; c++) {
        for (y = 0; y < 8; y++) {
            for (x = 0; x < 8; x++) {
                int pred;

                if (mode == 2) {
                    pred = get[c](p, x0 + x, y0 - 1);
                } else if (mode == 1) {
                    pred = get[c](p, x0 - 1, y0 + y);
                } else {
                    /* per 4x4 block, 8.3.4.1-3 */
                    int bx = x / 4, by = y / 4, st = 0, sl = 0, i;
                    bool top = my > 0, left = mx > 0;

                    for (i = 0; i < 4; i++) {
                        if (top) {
                            st += get[c](p, x0 + bx * 4 + i, y0 - 1);
                        }
                        if (left) {
                            sl += get[c](p, x0 - 1, y0 + by * 4 + i);
                        }
                    }
                    if (bx == by) {
                        pred = top && left ? (st + sl + 4) >> 3 :
                               top ? (st + 2) >> 2 :
                               left ? (sl + 2) >> 2 : 128;
                    } else if (bx) {    /* top right: top first */
                        pred = top ? (st + 2) >> 2 :
                               left ? (sl + 2) >> 2 : 128;
                    } else {            /* bottom left: left first */
                        pred = left ? (sl + 2) >> 2 :
                               top ? (st + 2) >> 2 : 128;
                    }
                }
                if (get[c](p, x0 + x, y0 + y) != pred) {
                    return false;
                }
            }
        }
    }
    return true;
}

/* What a macroblock is coded as */
enum { MB_SKIP, MB_I16, MB_PCM };

typedef struct MfeMb {
    uint8_t type;
    uint8_t lm;                         /* Intra16x16PredMode */
    uint8_t cm;                         /* intra_chroma_pred_mode */
} MfeMb;

/*
 * Whether macroblock (mx, my) of @a is within @tol of the reference's in
 * every sample; if so and not equal, it takes the reference's samples,
 * as a decoder of its P_Skip would.
 */
static bool mfe_mb_close(MfePic *a, const MfePic *r, int mx, int my,
                         int tol)
{
    int y, x;

    for (y = 0; y < 24; y++) {
        size_t o = (size_t)(y < 16 ? my * 16 + y : my * 8 + y - 16) * a->w;
        const uint8_t *pa = (y < 16 ? a->y : a->c) + o;
        const uint8_t *pr = (y < 16 ? r->y : r->c) + o;

        for (x = mx * 16; x < mx * 16 + 16; x++) {
            if (abs(pa[x] - pr[x]) > tol) {
                return false;
            }
        }
    }
    for (y = 0; y < 16; y++) {
        size_t o = (size_t)(my * 16 + y) * a->w + mx * 16;

        memcpy(a->y + o, r->y + o, 16);
    }
    for (y = 0; y < 8; y++) {
        size_t o = (size_t)(my * 8 + y) * a->w + mx * 16;

        memcpy(a->c + o, r->c + o, 16);
    }
    return true;
}

/*
 * Choose each macroblock's coding; returns the number of I_PCM ones.  In
 * a P slice at most @budget are I_PCM, looked for from macroblock *@next
 * on, round the picture; the rest of those that changed stay P_Skip (they
 * take the reference's samples), and *@next moves past the last I_PCM
 * one so that the next frame carries on from there.
 */
static int mfe_decide(MfePic *p, const MfePic *ref, MfeMb *mbs, int tol,
                      int budget, uint32_t *next)
{
    int mbw = p->w / 16, mbh = p->h / 16, mx, my, m, pcm = 0;

    if (ref) {
        int n = mbw * mbh, start = *next % n, k, i;

        for (k = 0; k < n; k++) {
            i = (start + k) % n;
            mbs[i].type = MB_SKIP;
            if (!mfe_mb_close(p, ref, i % mbw, i / mbw,
                              pcm < budget ? tol : 255)) {
                mbs[i].type = MB_PCM;
                if (++pcm == budget) {
                    *next = (i + 1) % n;
                }
            }
        }
        return pcm;
    }
    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            MfeMb *mb = &mbs[my * mbw + mx];
            int lm = -1, cm = -1;

            for (m = 0; m < 3 && lm < 0; m++) {
                if (mfe_luma_pred_ok(p, mx, my, m)) {
                    lm = m;
                }
            }
            for (m = 0; m < 3 && cm < 0; m++) {
                if (mfe_chroma_pred_ok(p, mx, my, m)) {
                    cm = m;
                }
            }
            mb->type = lm < 0 || cm < 0 ? MB_PCM : MB_I16;
            mb->lm = MAX(lm, 0);
            mb->cm = MAX(cm, 0);
            pcm += mb->type == MB_PCM;
        }
    }
    return pcm;
}

/* pcm_sample_luma, pcm_sample_chroma (the source has no zero samples) */
static void mfe_put_pcm_samples(SstarBits *b, const MfePic *p, int mx, int my)
{
    int x, y;

    for (y = 0; y < 16; y++) {
        for (x = 0; x < 16; x++) {
            sstar_bits_put(b, p->y[(my * 16 + y) * p->w + mx * 16 + x], 8);
        }
    }
    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
            sstar_bits_put(b, mfe_cb(p, mx * 8 + x, my * 8 + y), 8);
        }
    }
    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
            sstar_bits_put(b, mfe_cr(p, mx * 8 + x, my * 8 + y), 8);
        }
    }
}

/* coeff_token for TotalCoeff 0 (Table 9-5) */
static void mfe_put_no_coeffs(SstarBits *b, int nc)
{
    if (nc < 2) {
        sstar_bits_put(b, 1, 1);
    } else if (nc < 4) {
        sstar_bits_put(b, 3, 2);
    } else if (nc < 8) {
        sstar_bits_put(b, 0xF, 4);
    } else {
        sstar_bits_put(b, 3, 6);
    }
}

/* slice_data(), CAVLC */
static void mfe_slice_cavlc(SstarBits *b, const MfePic *p, const MfeMb *mbs,
                            bool pslice)
{
    int mbw = p->w / 16, mbh = p->h / 16, mx, my;
    g_autofree uint8_t *nz = g_malloc0(mbw * mbh);  /* total_coeff a block */
    uint32_t skip = 0;

    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            int i = my * mbw + mx;
            const MfeMb *mb = &mbs[i];

            if (mb->type == MB_SKIP) {
                skip++;
                continue;
            }
            if (pslice) {
                sstar_bits_ue(b, skip);         /* mb_skip_run */
                skip = 0;
            }
            if (mb->type == MB_PCM) {
                sstar_bits_ue(b, pslice ? 30 : 25);     /* I_PCM */
                sstar_bits_zero_align(b);       /* pcm_alignment_zero_bit */
                mfe_put_pcm_samples(b, p, mx, my);
                nz[i] = 16;
            } else {
                bool a = mx > 0, t = my > 0;
                int na = a ? nz[i - 1] : 0, nb = t ? nz[i - mbw] : 0;
                int nc = a && t ? (na + nb + 1) >> 1 : a ? na : nb;

                sstar_bits_ue(b, 1 + mb->lm);   /* I_16x16_<lm>_0_0 */
                sstar_bits_ue(b, mb->cm);       /* intra_chroma_pred_mode */
                sstar_bits_put(b, 1, 1);        /* mb_qp_delta = 0 */
                mfe_put_no_coeffs(b, nc);       /* Intra16x16DCLevel */
            }
        }
    }
    if (skip) {
        sstar_bits_ue(b, skip);
    }
}

/*
 * CABAC (H.264 9.3).  The (m, n) of contexts 0..104 (Tables 9-12 to
 * 9-18), the ones the coding below can use: mb_type (I 3-10, prefix and
 * suffix in P 14-20), mb_skip_flag (11-13), mb_qp_delta (60-63),
 * intra_chroma_pred_mode (64-67) and coded_block_flag (85-104); for I
 * slices, then P slices by cabac_init_idc.
 */
#define MFE_CTX_NUM         105

static const int8_t mfe_cabac_mn[4][MFE_CTX_NUM][2] = {
    {   /* I slices */
        { 20, -15 }, { 2, 54 }, { 3, 74 }, { 20, -15 }, { 2, 54 }, { 3, 74 },
        { -28, 127 }, { -23, 104 }, { -6, 53 }, { -1, 54 }, { 7, 51 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
        { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
        { 0, 41 }, { 0, 63 }, { 0, 63 }, { 0, 63 }, { -9, 83 }, { 4, 86 },
        { 0, 97 }, { -7, 72 }, { 13, 41 }, { 3, 62 }, { 0, 11 }, { 1, 55 },
        { 0, 69 }, { -17, 127 }, { -13, 102 }, { 0, 82 }, { -7, 74 },
        { -21, 107 }, { -27, 127 }, { -31, 127 }, { -24, 127 }, { -18, 95 },
        { -27, 127 }, { -21, 114 }, { -30, 127 }, { -17, 123 }, { -12, 115 },
        { -16, 122 }, { -11, 115 }, { -12, 63 }, { -2, 68 }, { -15, 84 },
        { -13, 104 }, { -3, 70 }, { -8, 93 }, { -10, 90 }, { -30, 127 },
        { -1, 74 }, { -6, 97 }, { -7, 91 }, { -20, 127 }, { -4, 56 },
        { -5, 82 }, { -7, 76 }, { -22, 125 },
    },
    {   /* P slices, cabac_init_idc 0 */
        { 20, -15 }, { 2, 54 }, { 3, 74 }, { 20, -15 }, { 2, 54 }, { 3, 74 },
        { -28, 127 }, { -23, 104 }, { -6, 53 }, { -1, 54 }, { 7, 51 },
        { 23, 33 }, { 23, 2 }, { 21, 0 }, { 1, 9 }, { 0, 49 }, { -37, 118 },
        { 5, 57 }, { -13, 78 }, { -11, 65 }, { 1, 62 }, { 12, 49 },
        { -4, 73 }, { 17, 50 }, { 18, 64 }, { 9, 43 }, { 29, 0 }, { 26, 67 },
        { 16, 90 }, { 9, 104 }, { -46, 127 }, { -20, 104 }, { 1, 67 },
        { -13, 78 }, { -11, 65 }, { 1, 62 }, { -6, 86 }, { -17, 95 },
        { -6, 61 }, { 9, 45 }, { -3, 69 }, { -6, 81 }, { -11, 96 }, { 6, 55 },
        { 7, 67 }, { -5, 86 }, { 2, 88 }, { 0, 58 }, { -3, 76 }, { -10, 94 },
        { 5, 54 }, { 4, 69 }, { -3, 81 }, { 0, 88 }, { -7, 67 }, { -5, 74 },
        { -4, 74 }, { -5, 80 }, { -7, 72 }, { 1, 58 }, { 0, 41 }, { 0, 63 },
        { 0, 63 }, { 0, 63 }, { -9, 83 }, { 4, 86 }, { 0, 97 }, { -7, 72 },
        { 13, 41 }, { 3, 62 }, { 0, 45 }, { -4, 78 }, { -3, 96 },
        { -27, 126 }, { -28, 98 }, { -25, 101 }, { -23, 67 }, { -28, 82 },
        { -20, 94 }, { -16, 83 }, { -22, 110 }, { -21, 91 }, { -18, 102 },
        { -13, 93 }, { -29, 127 }, { -7, 92 }, { -5, 89 }, { -7, 96 },
        { -13, 108 }, { -3, 46 }, { -1, 65 }, { -1, 57 }, { -9, 93 },
        { -3, 74 }, { -9, 92 }, { -8, 87 }, { -23, 126 }, { 5, 54 },
        { 6, 60 }, { 6, 59 }, { 6, 69 }, { -1, 48 }, { 0, 68 }, { -4, 69 },
        { -8, 88 },
    },
    {   /* P slices, cabac_init_idc 1 */
        { 20, -15 }, { 2, 54 }, { 3, 74 }, { 20, -15 }, { 2, 54 }, { 3, 74 },
        { -28, 127 }, { -23, 104 }, { -6, 53 }, { -1, 54 }, { 7, 51 },
        { 22, 25 }, { 34, 0 }, { 16, 0 }, { -2, 9 }, { 4, 41 }, { -29, 118 },
        { 2, 65 }, { -6, 71 }, { -13, 79 }, { 5, 52 }, { 9, 50 }, { -3, 70 },
        { 10, 54 }, { 26, 34 }, { 19, 22 }, { 40, 0 }, { 57, 2 }, { 41, 36 },
        { 26, 69 }, { -45, 127 }, { -15, 101 }, { -4, 76 }, { -6, 71 },
        { -13, 79 }, { 5, 52 }, { 6, 69 }, { -13, 90 }, { 0, 52 }, { 8, 43 },
        { -2, 69 }, { -5, 82 }, { -10, 96 }, { 2, 59 }, { 2, 75 }, { -3, 87 },
        { -3, 100 }, { 1, 56 }, { -3, 74 }, { -6, 85 }, { 0, 59 }, { -3, 81 },
        { -7, 86 }, { -5, 95 }, { -1, 66 }, { -1, 77 }, { 1, 70 }, { -2, 86 },
        { -5, 72 }, { 0, 61 }, { 0, 41 }, { 0, 63 }, { 0, 63 }, { 0, 63 },
        { -9, 83 }, { 4, 86 }, { 0, 97 }, { -7, 72 }, { 13, 41 }, { 3, 62 },
        { 13, 15 }, { 7, 51 }, { 2, 80 }, { -39, 127 }, { -18, 91 },
        { -17, 96 }, { -26, 81 }, { -35, 98 }, { -24, 102 }, { -23, 97 },
        { -27, 119 }, { -24, 99 }, { -21, 110 }, { -18, 102 }, { -36, 127 },
        { 0, 80 }, { -5, 89 }, { -7, 94 }, { -4, 92 }, { 0, 39 }, { 0, 65 },
        { -15, 84 }, { -35, 127 }, { -2, 73 }, { -12, 104 }, { -9, 91 },
        { -31, 127 }, { 3, 55 }, { 7, 56 }, { 7, 55 }, { 8, 61 }, { -3, 53 },
        { 0, 68 }, { -7, 74 }, { -9, 88 },
    },
    {   /* P slices, cabac_init_idc 2 */
        { 20, -15 }, { 2, 54 }, { 3, 74 }, { 20, -15 }, { 2, 54 }, { 3, 74 },
        { -28, 127 }, { -23, 104 }, { -6, 53 }, { -1, 54 }, { 7, 51 },
        { 29, 16 }, { 25, 0 }, { 14, 0 }, { -10, 51 }, { -3, 62 },
        { -27, 99 }, { 26, 16 }, { -4, 85 }, { -24, 102 }, { 5, 57 },
        { 6, 57 }, { -17, 73 }, { 14, 57 }, { 20, 40 }, { 20, 10 }, { 29, 0 },
        { 54, 0 }, { 37, 42 }, { 12, 97 }, { -32, 127 }, { -22, 117 },
        { -2, 74 }, { -4, 85 }, { -24, 102 }, { 5, 57 }, { -6, 93 },
        { -14, 88 }, { -6, 44 }, { 4, 55 }, { -11, 89 }, { -15, 103 },
        { -21, 116 }, { 19, 57 }, { 20, 58 }, { 4, 84 }, { 6, 96 }, { 1, 63 },
        { -5, 85 }, { -13, 106 }, { 5, 63 }, { 6, 75 }, { -3, 90 },
        { -1, 101 }, { 3, 55 }, { -4, 79 }, { -2, 75 }, { -12, 97 },
        { -7, 50 }, { 1, 60 }, { 0, 41 }, { 0, 63 }, { 0, 63 }, { 0, 63 },
        { -9, 83 }, { 4, 86 }, { 0, 97 }, { -7, 72 }, { 13, 41 }, { 3, 62 },
        { 7, 34 }, { -9, 88 }, { -20, 127 }, { -36, 127 }, { -17, 91 },
        { -14, 95 }, { -25, 84 }, { -25, 86 }, { -12, 89 }, { -17, 91 },
        { -31, 127 }, { -14, 76 }, { -18, 103 }, { -13, 90 }, { -37, 127 },
        { 11, 80 }, { 5, 76 }, { 2, 84 }, { 5, 78 }, { -6, 55 }, { 4, 61 },
        { -14, 83 }, { -37, 127 }, { -5, 79 }, { -11, 104 }, { -11, 91 },
        { -30, 127 }, { 0, 65 }, { -2, 79 }, { 0, 72 }, { -4, 92 },
        { -6, 56 }, { 3, 68 }, { -8, 71 }, { -13, 98 },
    },
};

#define CTX_MB_TYPE_I       3
#define CTX_MB_SKIP         11
#define CTX_MB_TYPE_P       14          /* prefix */
#define CTX_MB_TYPE_P_INTRA 17          /* suffix: an I mb_type in a P slice */
#define CTX_MB_QP_DELTA     60
#define CTX_CHROMA_PRED     64
#define CTX_CBF             85

QEMU_BUILD_BUG_ON(MFE_CTX_NUM > SSTAR_CABAC_CTX_MAX);

/*
 * I_PCM after its mb_type: the terminate bin of 1 flushes the coder
 * (9.3.4.5), pcm_alignment_zero_bits, the samples, and the coder starts
 * again (9.3.1.2).
 */
static void mfe_cabac_pcm(SstarCabac *c, SstarBits *b, const MfePic *p,
                          int mx, int my)
{
    sstar_cabac_terminate(c, 1);
    sstar_cabac_finish(c);
    sstar_bits_one_align(b);
    mfe_put_pcm_samples(b, p, mx, my);
    sstar_cabac_start(c, b);
}

/* slice_data(), CABAC; it starts byte aligned (cabac_alignment_one_bit) */
static void mfe_slice_cabac(SstarBits *b, const MfePic *p, const MfeMb *mbs,
                            bool pslice, int idc, int qp)
{
    const int8_t (*mn)[2] = mfe_cabac_mn[pslice ? 1 + idc : 0];
    int mbw = p->w / 16, n = mbw * (p->h / 16), i, k;
    SstarCabac c;

    for (i = 0; i < MFE_CTX_NUM; i++) {
        sstar_cabac_init_ctx(&c, i, mn[i][0], mn[i][1], qp);
    }
    sstar_cabac_start(&c, b);
    for (i = 0; i < n; i++) {
        const MfeMb *mb = &mbs[i];
        const MfeMb *a = i % mbw ? mb - 1 : NULL;
        const MfeMb *t = i >= mbw ? mb - mbw : NULL;

        if (pslice) {
            /* ctxIdxInc: the neighbours available and not skipped */
            sstar_cabac_bin(&c, mb->type == MB_SKIP, CTX_MB_SKIP +
                            (a && a->type != MB_SKIP) +
                            (t && t->type != MB_SKIP));
            if (mb->type == MB_PCM) {
                /* prefix 1: intra; suffix bin 0 1: not I_NxN; then PCM */
                sstar_cabac_bin(&c, 1, CTX_MB_TYPE_P);
                sstar_cabac_bin(&c, 1, CTX_MB_TYPE_P_INTRA);
                mfe_cabac_pcm(&c, b, p, i % mbw, i / mbw);
            }
        } else {
            /*
             * mb_type bin 0, 1 = not I_NxN; ctxIdxInc counts the
             * neighbours available and not I_NxN: all of them here.
             */
            sstar_cabac_bin(&c, 1, CTX_MB_TYPE_I + !!a + !!t);
            if (mb->type == MB_PCM) {
                mfe_cabac_pcm(&c, b, p, i % mbw, i / mbw);
            } else {
                int inc;

                /* I_16x16_<lm>_0_0: not PCM, no AC, no chroma, lm */
                sstar_cabac_terminate(&c, 0);
                sstar_cabac_bin(&c, 0, CTX_MB_TYPE_I + 3);
                sstar_cabac_bin(&c, 0, CTX_MB_TYPE_I + 4);
                sstar_cabac_bin(&c, mb->lm >> 1, CTX_MB_TYPE_I + 6);
                sstar_cabac_bin(&c, mb->lm & 1, CTX_MB_TYPE_I + 7);
                /*
                 * intra_chroma_pred_mode, TU cMax 3; ctxIdxInc counts the
                 * neighbours coded Intra (not I_PCM) with a mode other
                 * than DC
                 */
                inc = (a && a->type == MB_I16 && a->cm) +
                      (t && t->type == MB_I16 && t->cm);
                for (k = 0; k < MIN(mb->cm + 1, 3); k++) {
                    sstar_cabac_bin(&c, k < mb->cm,
                                    CTX_CHROMA_PRED + (k ? 3 : inc));
                }
                /* mb_qp_delta 0, the previous macroblock's was 0 too */
                sstar_cabac_bin(&c, 0, CTX_MB_QP_DELTA);
                /*
                 * coded_block_flag 0 of the Intra16x16DCLevel block;
                 * condTermFlagN is 1 for an unavailable neighbour (this
                 * one is intra) and an I_PCM one, the flag of an I16 one
                 * (0)
                 */
                sstar_cabac_bin(&c, 0, CTX_CBF +
                                (!a || a->type == MB_PCM) +
                                2 * (!t || t->type == MB_PCM));
            }
        }
        sstar_cabac_terminate(&c, i == n - 1);      /* end_of_slice_flag */
    }
    sstar_cabac_finish(&c);
    sstar_bits_one_align(b);            /* rbsp_slice_trailing_bits() */
}

/*
 * The deblocking filter (8.7) over the decoded picture, which holds the
 * source here.  With these macroblock types bS is 4 on a macroblock edge
 * next to an intra (I16 or I_PCM) one, 3 inside an intra one and 0 for
 * every edge between and inside P_Skip ones (no coefficients, the same
 * reference, all vectors zero: each P_Skip prediction comes from zero
 * vectors or intra neighbours).
 */
typedef struct MfeDbf {
    int idc;                            /* disable_deblocking_filter_idc */
    int offset_a, offset_b;             /* FilterOffsetA/B */
    int qp, cqp_offset;
} MfeDbf;

/* alpha', beta' (Table 8-16) and tC0 for bS 3 (Table 8-17), indexA/B >= 16 */
static const uint8_t mfe_alpha[36] = {
    4, 4, 5, 6, 7, 8, 9, 10, 12, 13, 15, 17, 20, 22, 25, 28, 32, 36, 40, 45,
    50, 56, 63, 71, 80, 90, 101, 113, 127, 144, 162, 182, 203, 226, 255, 255,
};
static const uint8_t mfe_beta[36] = {
    2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10,
    11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18,
};
static const uint8_t mfe_tc0_bs3[36] = {
    0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4,
    4, 5, 6, 6, 7, 8, 9, 10, 11, 13, 14, 16, 18, 20, 23, 25,
};
/* QPc for qPI 30..51 (Table 8-15) */
static const uint8_t mfe_chroma_qp[22] = {
    29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38,
    39, 39, 39, 39,
};

/* qPp of a macroblock (8.7.2.2): I_PCM counts as QPY 0 */
static int mfe_dbf_qp(const MfeDbf *d, const MfeMb *mb, bool chroma)
{
    int qp = mb->type == MB_PCM ? 0 : d->qp;

    if (chroma) {
        qp = MIN(MAX(qp + d->cqp_offset, 0), 51);
        qp = qp < 30 ? qp : mfe_chroma_qp[qp - 30];
    }
    return qp;
}

/*
 * Filter @len sample sets across one edge (8.7.2.3, 8.7.2.4): @px is q0 of
 * the first, @step goes across the edge (p0 = px[-step]), @along to the
 * next set.
 */
static void mfe_dbf_edge(uint8_t *px, int step, int along, int len, int bs,
                         bool chroma, int qpav, const MfeDbf *d)
{
    int ia = MIN(MAX(qpav + d->offset_a, 0), 51);
    int ib = MIN(MAX(qpav + d->offset_b, 0), 51);
    int alpha = ia < 16 ? 0 : mfe_alpha[ia - 16];
    int beta = ib < 16 ? 0 : mfe_beta[ib - 16];
    int tc0 = ia < 16 ? 0 : mfe_tc0_bs3[ia - 16];
    int k;

    for (k = 0; k < len; k++, px += along) {
        int p0 = px[-step], p1 = px[-2 * step], q0 = px[0], q1 = px[step];
        int p2, p3, q2, q3, ap, aq, tc, delta;

        if (!(abs(p0 - q0) < alpha && abs(p1 - p0) < beta &&
              abs(q1 - q0) < beta)) {
            continue;
        }
        if (chroma) {
            if (bs == 4) {
                px[-step] = (2 * p1 + p0 + q1 + 2) >> 2;
                px[0] = (2 * q1 + q0 + p1 + 2) >> 2;
            } else {
                tc = tc0 + 1;
                delta = MIN(MAX((((q0 - p0) << 2) + (p1 - q1) + 4) >> 3,
                                -tc), tc);
                px[-step] = MIN(MAX(p0 + delta, 0), 255);
                px[0] = MIN(MAX(q0 - delta, 0), 255);
            }
            continue;
        }
        p2 = px[-3 * step];
        p3 = px[-4 * step];
        q2 = px[2 * step];
        q3 = px[3 * step];
        ap = abs(p2 - p0);
        aq = abs(q2 - q0);
        if (bs == 4) {
            bool small = abs(p0 - q0) < ((alpha >> 2) + 2);

            if (ap < beta && small) {
                px[-step] = (p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3;
                px[-2 * step] = (p2 + p1 + p0 + q0 + 2) >> 2;
                px[-3 * step] = (2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3;
            } else {
                px[-step] = (2 * p1 + p0 + q1 + 2) >> 2;
            }
            if (aq < beta && small) {
                px[0] = (p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) >> 3;
                px[step] = (p0 + q0 + q1 + q2 + 2) >> 2;
                px[2 * step] = (2 * q3 + 3 * q2 + q1 + q0 + p0 + 4) >> 3;
            } else {
                px[0] = (2 * q1 + q0 + p1 + 2) >> 2;
            }
        } else {
            tc = tc0 + (ap < beta) + (aq < beta);
            delta = MIN(MAX((((q0 - p0) << 2) + (p1 - q1) + 4) >> 3, -tc),
                        tc);
            px[-step] = MIN(MAX(p0 + delta, 0), 255);
            px[0] = MIN(MAX(q0 - delta, 0), 255);
            if (ap < beta) {
                px[-2 * step] = p1 + MIN(MAX((p2 + ((p0 + q0 + 1) >> 1) -
                                              (p1 << 1)) >> 1, -tc0), tc0);
            }
            if (aq < beta) {
                px[step] = q1 + MIN(MAX((q2 + ((p0 + q0 + 1) >> 1) -
                                         (q1 << 1)) >> 1, -tc0), tc0);
            }
        }
    }
}

static void mfe_deblock(MfePic *p, const MfeMb *mbs, const MfeDbf *d)
{
    int mbw = p->w / 16, mbh = p->h / 16, mx, my, e, dir, comp, w = p->w;

    if (d->idc == 1) {
        return;
    }
    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            const MfeMb *q = &mbs[my * mbw + mx];
            bool intra = q->type != MB_SKIP;

            /* luma: vertical edges left to right, then horizontal ones */
            for (dir = 0; dir < 2; dir++) {
                for (e = 0; e < 4; e++) {
                    const MfeMb *nb = q;
                    int bs = intra ? 3 : 0;
                    uint8_t *px = p->y + (my * 16) * w + mx * 16;

                    if (!e) {
                        if (dir ? !my : !mx) {
                            continue;
                        }
                        nb = dir ? q - mbw : q - 1;
                        bs = intra || nb->type != MB_SKIP ? 4 : 0;
                    }
                    if (!bs) {
                        continue;
                    }
                    px += dir ? 4 * e * w : 4 * e;
                    mfe_dbf_edge(px, dir ? w : 1, dir ? 1 : w, 16, bs, false,
                                 (mfe_dbf_qp(d, nb, false) +
                                  mfe_dbf_qp(d, q, false) + 1) >> 1, d);
                }
            }
            /* chroma (4:2:0): edges 0 and 4, as luma edges 0 and 8 */
            for (comp = 0; comp < 2; comp++) {
                for (dir = 0; dir < 2; dir++) {
                    for (e = 0; e < 2; e++) {
                        const MfeMb *nb = q;
                        int bs = intra ? 3 : 0;
                        uint8_t *px = p->c + (my * 8) * w + mx * 16 + comp;

                        if (!e) {
                            if (dir ? !my : !mx) {
                                continue;
                            }
                            nb = dir ? q - mbw : q - 1;
                            bs = intra || nb->type != MB_SKIP ? 4 : 0;
                        }
                        if (!bs) {
                            continue;
                        }
                        px += dir ? 4 * e * w : 8 * e;
                        mfe_dbf_edge(px, dir ? w : 2, dir ? 2 : w, 8, bs,
                                     true, (mfe_dbf_qp(d, nb, true) +
                                            mfe_dbf_qp(d, q, true) + 1) >> 1,
                                     d);
                    }
                }
            }
        }
    }
}

static int sext(uint32_t v, int bits)
{
    return (int32_t)(v << (32 - bits)) >> (32 - bits);
}

/* Append the frame's output, as written, to dump/mfe-<w>x<h>.h264 */
static void mfe_dump(SstarMfeState *s, int w, int h, const uint8_t *data,
                     size_t len)
{
    g_autofree char *path = g_strdup_printf("%s/mfe-%dx%d.h264", s->dump,
                                            w, h);
    FILE *f = fopen(path, "ab");

    if (!f) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-mfe: cannot open %s\n", path);
        return;
    }
    if (fwrite(data, 1, len, f) != len) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-mfe: short write to %s\n",
                      path);
    }
    fclose(f);
}

static void mfe_encode(void *opaque)
{
    SstarMfeState *s = opaque;
    int w = s->regs[R_WIDTH] & ~15, h = s->regs[R_HEIGHT] & ~15;
    bool pframe = ((s->regs[R_CTRL] >> 4) & 3) == 1;
    bool cabac = s->regs[R_MODE] & MODE_CABAC;
    int qp = (s->regs[R_CTRL] >> 8) & 0x3F;
    SstarBits b = { .buf = g_byte_array_new() };
    g_autoptr(GByteArray) out = g_byte_array_new();
    g_autofree MfeMb *mbs = NULL;
    uint32_t i, whole = s->hdr_bits / 8, zeros = 0, cap, len;
    MfeDbf dbf = { .qp = qp };
    MfePic pic, ref;
    int pcm;

    s->started = false;
    if (!w || !h || w * h / 256 > MAX_MBS) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-mfe: picture %dx%d\n", w, h);
        g_byte_array_unref(b.buf);
        return;
    }
    /* the header bits, then the slice data and its trailing bits */
    for (i = 0; i < s->hdr_bits; i++) {
        sstar_bits_put(&b, (s->hdr[i >> 3] >> (7 - (i & 7))) & 1, 1);
    }
    mfe_pic_alloc(&pic, w, h);
    mfe_load_source(s, &pic);
    mfe_pic_no_zero(&pic);
    mbs = g_new0(MfeMb, w * h / 256);
    if (pframe) {
        mfe_pic_alloc(&ref, w, h);
        mfe_pic_rw(s, &ref, R_REF_Y, R_REF_C, false);
        /*
         * an eighth of the picture (64 macroblocks at least) as I_PCM at
         * most, 384 bytes and a few each, and no more than fits in the
         * output buffer
         */
        cap = s->obuf[1] >= s->obuf[0] ? s->obuf[1] - s->obuf[0] + 1 : 0;
        cap = cap > whole + 4096 ? (cap - whole - 4096) / 400 : 0;
        pcm = mfe_decide(&pic, &ref, mbs, s->skip_tol,
                         MIN(cap, MAX(w * h / 256 / 8, 64)), &s->pcm_next);
        mfe_pic_free(&ref);
    } else {
        pcm = mfe_decide(&pic, NULL, mbs, 0, 0, NULL);
    }
    if (cabac) {
        /* the driver's header ends aligned; if not, align as it would */
        while (b.n) {
            sstar_bits_put(&b, 1, 1);   /* cabac_alignment_one_bit */
        }
        /* h264_enc.c leaves cabac_init_idc 0 (never set, kzalloc'ed) */
        mfe_slice_cabac(&b, &pic, mbs, pframe, 0, qp);
    } else {
        mfe_slice_cavlc(&b, &pic, mbs, pframe);
        sstar_bits_one_align(&b);       /* rbsp_slice_trailing_bits() */
    }

    /* the reconstruction: the decoder's picture, deblocked */
    if (s->regs[R_DBF_CTRL] & DBF_CONTROL_PRESENT) {
        dbf.idc = (s->regs[R_DBF_IDC] >> 3) & 3;
        dbf.offset_a = 2 * sext(s->regs[R_DBF_IDC] >> 5, 4);
        dbf.offset_b = 2 * sext(s->regs[R_DBF_IDC] >> 9, 4);
    }
    dbf.cqp_offset = sext(s->regs[R_CQP_OFFSET] >> 8, 5);
    mfe_deblock(&pic, mbs, &dbf);
    mfe_pic_rw(s, &pic, R_REC_Y, R_REC_C, true);
    mfe_pic_free(&pic);

    /* the header's whole bytes come escaped; escape from there on */
    for (i = 0; i < b.buf->len; i++) {
        uint8_t v = b.buf->data[i];

        if (i >= whole && zeros >= 2 && v <= 3) {
            uint8_t e = 3;

            g_byte_array_append(out, &e, 1);
            zeros = 0;
        }
        g_byte_array_append(out, &v, 1);
        zeros = v ? 0 : zeros + 1;
    }
    g_byte_array_unref(b.buf);

    cap = s->obuf[1] >= s->obuf[0] ? s->obuf[1] - s->obuf[0] + 1 : 0;
    len = MIN(out->len, cap);
    if (mfe_in_dram(s, s->obuf[0], len)) {
        address_space_write(&address_space_memory, DRAM_BASE + s->obuf[0],
                            MEMTXATTRS_UNSPECIFIED, out->data, len);
    }
    if (s->dump) {
        mfe_dump(s, w, h, out->data, len);
    }
    s->regs[R_BITS_LO] = (len * 8) & 0xFFFF;
    s->regs[R_BITS_HI] = ((len * 8) >> 16) & 0xFF;
    s->regs[R_WPTR_LO] = ((s->obuf[0] + len) >> 3) & 0xFFFF;
    s->regs[R_WPTR_HI] = ((s->obuf[0] + len) >> 19) & 0x1FFF;
    s->regs[R_CYCLES_LO] = (w * h / 256 * 200) & 0xFFFF;
    s->regs[R_CYCLES_HI] = ((w * h / 256 * 200) >> 16) & 0xFF;
    s->raw |= IRQ_DONE | (len < out->len ? IRQ_BUF_FULL : 0);
    if (s->trace) {
        qemu_log("sstar-mfe: %c %dx%d %s qp %d src 0x%x hdr %u bits, "
                 "%u bytes (%d PCM MBs) at 0x%x%s\n", pframe ? 'P' : 'I',
                 w, h, cabac ? "CABAC" : "CAVLC", qp,
                 mfe_addr(s, R_CUR_Y, 8), s->hdr_bits, len, pcm, s->obuf[0],
                 len < out->len ? " (buffer full)" : "");
    }
    s->frames++;
    mfe_update(s);
}

static uint64_t mfe_read(void *opaque, hwaddr off, unsigned size)
{
    SstarMfeState *s = opaque;
    int reg = (off >> 2) & (MFE_REGS - 1);
    uint16_t v = s->regs[reg];

    if ((off & 3) >= 2) {
        return 0;
    }
    if (reg == R_IRQ_STATUS) {
        v = (s->raw & ~s->regs[R_IRQ_MASK] & 0xFF) | s->raw << 8;
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void mfe_fdc_push(SstarMfeState *s)
{
    uint32_t count = (s->regs[R_FDC_LEN] >> 5) & 0x3FF;
    uint32_t n = s->hdr_words < count ? 16 : s->regs[R_FDC_LEN] & 0x1F;
    uint16_t word = s->regs[R_FDC_BS];
    uint32_t i;

    for (i = 0; i < MIN(n, 16u) && s->hdr_bits < HDR_MAX_BITS; i++) {
        if ((word >> (15 - i)) & 1) {
            s->hdr[s->hdr_bits >> 3] |= 0x80 >> (s->hdr_bits & 7);
        }
        s->hdr_bits++;
    }
    if (++s->hdr_words == count + 1 && s->started) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  ENCODE_NS);
    }
}

static void mfe_new_frame(SstarMfeState *s)
{
    s->hdr_words = 0;
    s->hdr_bits = 0;
    memset(s->hdr, 0, sizeof(s->hdr));
}

static void mfe_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarMfeState *s = opaque;
    int reg = (off >> 2) & (MFE_REGS - 1);
    uint16_t v = val;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (s->regs[reg] & 0x00FF) | ((val & 0xFF) << 8)
                    : (s->regs[reg] & 0xFF00) | (val & 0xFF);
    }
    switch (reg) {
    case R_CTRL:
        if (!(v & CTRL_RSTZ)) {
            timer_del(s->timer);
            s->started = false;
            mfe_new_frame(s);
        }
        if (v & CTRL_START) {
            s->started = true;
            mfe_new_frame(s);
        }
        v &= ~CTRL_START;
        break;
    case R_IRQ_MASK:
        s->raw |= v >> 8;
        v &= 0xFF;
        break;
    case R_IRQ_CLR:
        s->raw &= ~v;
        v = 0;
        break;
    case R_IRQ_STATUS:
    case R_BITS_LO:
    case R_BITS_HI:
        return;
    case R_OBUF_STROBE:
        if (v & 1) {
            int id = (s->regs[R_OBUF_HI] >> 14) & 1;

            s->obuf[id] = (s->regs[R_OBUF_LO] |
                           (uint32_t)(s->regs[R_OBUF_HI] & 0x1FFF) << 16)
                          << 3;
            if (id) {
                s->obuf[1] |= 7;        /* the end address is inclusive */
            }
        }
        break;
    case R_FDC_CTRL:
        s->regs[reg] = v & ~FDC_VLD;
        if (v & FDC_VLD) {
            mfe_fdc_push(s);
        }
        return;
    }
    s->regs[reg] = v;
    mfe_update(s);
}

static const MemoryRegionOps mfe_ops = {
    .read = mfe_read,
    .write = mfe_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_mfe_reset(DeviceState *dev)
{
    SstarMfeState *s = SSTAR_MFE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_IRQ_MASK] = 0xFF;
    s->raw = 0;
    s->obuf[0] = s->obuf[1] = 0;
    s->started = false;
    mfe_new_frame(s);
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void sstar_mfe_init(Object *obj)
{
    SstarMfeState *s = SSTAR_MFE(obj);

    memory_region_init_io(&s->iomem, obj, &mfe_ops, s, TYPE_SSTAR_MFE,
                          MFE_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mfe_encode, s);
}

static const Property sstar_mfe_props[] = {
    DEFINE_PROP_UINT64("dram-size", SstarMfeState, dram_size, 64 * MiB),
    DEFINE_PROP_BOOL("trace", SstarMfeState, trace, false),
    DEFINE_PROP_UINT8("skip-tolerance", SstarMfeState, skip_tol, 2),
    DEFINE_PROP_STRING("dump", SstarMfeState, dump),
};

static const VMStateDescription vmstate_sstar_mfe = {
    .name = TYPE_SSTAR_MFE,
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarMfeState, MFE_REGS),
        VMSTATE_UINT8(raw, SstarMfeState),
        VMSTATE_UINT32_ARRAY(obuf, SstarMfeState, 2),
        VMSTATE_BOOL(started, SstarMfeState),
        VMSTATE_UINT32(hdr_words, SstarMfeState),
        VMSTATE_UINT32(hdr_bits, SstarMfeState),
        VMSTATE_UINT8_ARRAY(hdr, SstarMfeState, HDR_MAX_BITS / 8),
        VMSTATE_UINT32(frames, SstarMfeState),
        VMSTATE_TIMER_PTR(timer, SstarMfeState),
        VMSTATE_UINT32_V(pcm_next, SstarMfeState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_mfe_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_mfe_reset);
    device_class_set_props(dc, sstar_mfe_props);
    dc->vmsd = &vmstate_sstar_mfe;
}

static const TypeInfo sstar_mfe_info = {
    .name           = TYPE_SSTAR_MFE,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarMfeState),
    .instance_init  = sstar_mfe_init,
    .class_init     = sstar_mfe_class_init,
};

static void sstar_mfe_register_types(void)
{
    type_register_static(&sstar_mfe_info);
}

type_init(sstar_mfe_register_types)
