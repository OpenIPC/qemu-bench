/*
 * Grain Media FTMCP100 MPEG-4 / JPEG codec: control and the MJPEG encoder.
 *
 * The codec's registers sit at 0x92020000 on the GM813x, above its local
 * memory at 0x92000000 (the SRAM nsboot also runs in).  fmcp_drv.ko
 * (module/ftmcp100/fmcp.c fmcp100_resetrun()) resets the block through
 * 0x2040C bit 27 and polls it clear, stops the codec's embedded CPU
 * (INCTL 0x2007C = 1 << 17), copies the CPU's firmware into the local
 * memory, starts it (3 << 18) and polls INCTL bit 23 for the CPU running.
 * The model has no embedded CPU: INCTL reads it running, the reset
 * completes at once, and it does the firmware's part of the jobs itself.
 *
 * MJPEG (fmjpeg_drv.ko, the "embedded" build: jpeg_enc.c, jccoefct.c,
 * jcmarker.c) goes through the firmware's mailbox, the Share_Node_JE at
 * local 0x8000, which the model maps over the SRAM:
 *
 *   +0x00 command (byte): the host writes 8 to encode, the firmware 0x80
 *         when done (and 0 when it starts)
 *   +0x04 bs_phy, +0x08 image_width, +0x0C restart_interval (MCUs)
 *   +0x20 u82D: 4 packed UYVY through the DMA wrapper, 1 planar I420
 *   +0x24-0x30 the ROI's left / top / right / bottom, in MCUs
 *   +0x38-0x40 the I420 planes, +0x44 max_bs_thd, +0x48 / 0x4C stop x / y
 *   +0xD4 bitslen, +0xD8 jpgenc_done
 *
 * The host writes the headers itself (SOI, APP0, DQT, SOF0, DHT, DRI,
 * SOS) through the bit packer: BADR 0x2004C the value, BALR 0x20050 the
 * bit count, appended MSB-first at ABADR 0x20078 (= bs_phy | 1); it polls
 * VLDSTS 0x20074 bit 10 between segments.  On the command the model
 * encodes the picture -- 4:2:0, 16x16 MCUs, the quantisation tables of
 * the DQTs it was given, the standard Huffman tables, RSTn every
 * restart_interval MCUs -- right after the headers, ends it with EOI,
 * fills in the node and raises INT_FLAG 0x20098 bit 31 on interrupt 31
 * (the host writes the value back to clear it).  A scan that would pass
 * max_bs_thd is cut short with EOI and stop_x / stop_y, as the firmware
 * does, so the host retries at a lower quality.
 *
 * Packed input comes through the DMA wrapper at 0x92400000 (the second
 * region): +0x08 / +0x0C the ROI's width / height in MCUs, +0x18 the word
 * address of its first pixel, rows image_width * 2 bytes apart; chroma
 * line pairs are averaged to 4:2:0.  MPEG-4 is not built into the
 * cameras' kernels and its commands are not executed.
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
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/nvt-pixel.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"
#include <math.h>
#include "mpeg4-enc.c.inc"

#define TYPE_GM_MCP100 "gm-mcp100"
OBJECT_DECLARE_SIMPLE_TYPE(GmMcp100State, GM_MCP100)

#define GM_MCP100_SIZE      0x1000
#define GM_MCP100_NREGS     (GM_MCP100_SIZE / 4)
#define GM_MCP100_NODE_SIZE 0x1000
#define GM_MCP100_WRP_SIZE  0x1000
#define GM_MCP100_WRP_NREGS (0x40 / 4)
#define GM_MCP100_HDR_MAX   2048
#define GM_MCP100_JOB_NS    5000000
#define GM_MCP100_MAX_MCUS  256         /* per side: 4096 pixels */

#define R_VADR              0x044
#define R_BADR              0x04C
#define R_BALR              0x050
#define R_VLDSTS            0x074
#define R_ABADR             0x078
#define R_INCTL             0x07C
#define R_INT_FLAG          0x098
#define R_RESET             0x40C

#define INCTL_CPU_STOP      (1u << 17)
#define INCTL_CPU_START     (3u << 18)
#define INCTL_CPU_IDLE      (1u << 23)
#define VLDSTS_READY        (1u << 10)
#define INT_JPEG_DONE       (1u << 31)
#define RESET_GO            (1u << 27)

/* the Share_Node_JE */
#define N_COMMAND           0x00
#define N_BS_PHY            0x04
#define N_IMAGE_WIDTH       0x08
#define N_RESTART           0x0C
#define N_U82D              0x20
#define N_ROI_LEFT          0x24
#define N_ROI_TOP           0x28
#define N_ROI_RIGHT         0x2C
#define N_ROI_BOTTOM        0x30
#define N_YUV_ADDR          0x38
#define N_MAX_BS_THD        0x44
#define N_STOP_X            0x48
#define N_STOP_Y            0x4C
#define N_BITSLEN           0xD4
#define N_DONE              0xD8

#define CMD_ENCODE_I        4
#define CMD_ENCODE_P        5
#define CMD_ENCODE_N        6
#define CMD_ENCODE_JPEG     8

/*
 * The Share_Node_Enc of the MPEG-4 encoder: the driver's Encoder_x from
 * +0x04, then its current and reference FRAMEINFOs (fmpeg4_drv.ko of the
 * GM8126, "new" 2012 build).
 */
#define N4_VOL              0x024
#define N4_PRED             0x2BC
#define N4_MB_W             0x300
#define N4_MB_H             0x304
#define N4_WIDTH            0x2D8
#define N4_ROI_W            0x2F0
#define N4_ROI_H            0x2F4
#define N4_FINCR            0x308
#define N4_FBASE            0x30C
#define N4_ROUNDING         0x310
#define N4_MODULE_BASE      0x384
#define N4_BITSTREAM        0x38C
#define N4_IMG_FMT          0x390
#define N4_CUR              0x3A4
#define N4_REF              0x3FC
#define F_QUANT             0x00
#define F_TYPE              0x04
#define F_ROUNDING          0x08
#define F_SECONDS           0x0C
#define F_TICKS             0x10
#define F_IMAGE             0x20        /* Y, Cb, Cr physical */
#define F_RECON             0x38

/* the DMA wrapper */
#define W_WIDTH             0x08
#define W_HEIGHT            0x0C
#define W_VSMSA             0x18

struct GmMcp100State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion node_mem;
    MemoryRegion wrp_mem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[GM_MCP100_NREGS];
    uint8_t node[GM_MCP100_NODE_SIZE];
    uint32_t wrp[GM_MCP100_WRP_NREGS];

    /* the header bit packer */
    uint32_t bs_phy;
    uint32_t hdr_len;
    bool packer_off;
    uint8_t job;
    char *dump;
    uint32_t acc;
    int acc_bits;
    uint8_t hdr[GM_MCP100_HDR_MAX];
};

static void gm_mcp100_update(GmMcp100State *s)
{
    qemu_set_irq(s->irq, (s->regs[R_INT_FLAG / 4] & INT_JPEG_DONE) != 0);
}

/* ---- the header bit packer ------------------------------------------- */

static void gm_mcp100_put_bits(GmMcp100State *s, uint32_t val, int n)
{
    int i;

    for (i = n - 1; i >= 0; i--) {
        s->acc = s->acc << 1 | ((val >> i) & 1);
        if (++s->acc_bits == 8) {
            uint8_t b = s->acc;

            if (s->hdr_len < GM_MCP100_HDR_MAX) {
                s->hdr[s->hdr_len] = b;
            }
            dma_memory_write(&address_space_memory, s->bs_phy + s->hdr_len,
                             &b, 1, MEMTXATTRS_UNSPECIFIED);
            s->hdr_len++;
            s->acc = 0;
            s->acc_bits = 0;
        }
    }
}

/* the quantisation tables of the DQT segments the host wrote (zig-zag) */
static void gm_mcp100_parse_dqt(GmMcp100State *s, uint8_t q[4][64])
{
    const uint32_t n = MIN(s->hdr_len, GM_MCP100_HDR_MAX);
    uint32_t i = 2;

    while (i + 4 <= n) {
        uint32_t seg, end;

        if (s->hdr[i] != 0xFF) {
            i++;
            continue;
        }
        if (s->hdr[i + 1] == 0xDA) {    /* SOS: the headers end */
            break;
        }
        seg = s->hdr[i + 2] << 8 | s->hdr[i + 3];
        end = MIN(i + 2 + seg, n);
        if (s->hdr[i + 1] == 0xDB) {
            uint32_t p = i + 4;

            while (p + 65 <= end) {
                const unsigned id = s->hdr[p] & 3;

                if (s->hdr[p] >> 4) {   /* 16-bit tables: not baseline */
                    break;
                }
                memcpy(q[id], &s->hdr[p + 1], 64);
                p += 65;
            }
        }
        i += 2 + seg;
    }
}

/* ---- the encoder ----------------------------------------------------- */

static uint32_t gm_mcp100_node32(GmMcp100State *s, uint32_t off)
{
    return ldl_le_p(&s->node[off]);
}

/* fetch the picture into Y / Cb / Cr planes of w x h and w/2 x h/2 */
static bool gm_mcp100_fetch(GmMcp100State *s, uint32_t w, uint32_t h,
                            uint8_t *y, uint8_t *cb, uint8_t *cr)
{
    const uint32_t width = gm_mcp100_node32(s, N_IMAGE_WIDTH);
    const unsigned u82d = s->node[N_U82D];
    const uint32_t left = gm_mcp100_node32(s, N_ROI_LEFT) * 16;
    const uint32_t top = gm_mcp100_node32(s, N_ROI_TOP) * 16;
    uint32_t r, x;

    if (u82d == 4) {
        const uint32_t stride = width * 2;
        const hwaddr base = (hwaddr)s->wrp[W_VSMSA / 4] << 2;
        g_autofree uint8_t *row = g_malloc(stride ? stride : 1);

        if (!stride || w * 2 > stride) {
            return false;
        }
        for (r = 0; r < h; r++) {
            dma_memory_read(&address_space_memory, base + (hwaddr)r * stride,
                            row, w * 2, MEMTXATTRS_UNSPECIFIED);
            for (x = 0; x < w; x++) {
                y[r * w + x] = row[x * 2 + 1];
            }
            for (x = 0; x < w / 2; x++) {
                const size_t c = (size_t)(r / 2) * (w / 2) + x;

                if (r & 1) {        /* average the line pair */
                    cb[c] = (cb[c] + row[x * 4] + 1) / 2;
                    cr[c] = (cr[c] + row[x * 4 + 2] + 1) / 2;
                } else {
                    cb[c] = row[x * 4];
                    cr[c] = row[x * 4 + 2];
                }
            }
        }
        return true;
    }
    if (u82d == 1) {
        const hwaddr py = gm_mcp100_node32(s, N_YUV_ADDR);
        const hwaddr pu = gm_mcp100_node32(s, N_YUV_ADDR + 4);
        const hwaddr pv = gm_mcp100_node32(s, N_YUV_ADDR + 8);

        if (!width || left + w > width) {
            return false;
        }
        for (r = 0; r < h; r++) {
            dma_memory_read(&address_space_memory,
                            py + (hwaddr)(top + r) * width + left,
                            y + (size_t)r * w, w, MEMTXATTRS_UNSPECIFIED);
        }
        for (r = 0; r < h / 2; r++) {
            const hwaddr off = (hwaddr)(top / 2 + r) * (width / 2) + left / 2;

            dma_memory_read(&address_space_memory, pu + off,
                            cb + (size_t)r * (w / 2), w / 2,
                            MEMTXATTRS_UNSPECIFIED);
            dma_memory_read(&address_space_memory, pv + off,
                            cr + (size_t)r * (w / 2), w / 2,
                            MEMTXATTRS_UNSPECIFIED);
        }
        return true;
    }
    qemu_log_mask(LOG_UNIMP, "gm-mcp100: JPEG input format %u\n", u82d);
    return false;
}

static void gm_mcp100_encode(GmMcp100State *s)
{
    uint8_t q[4][64];
    uint32_t mw, mh, w, h, room, thd, restart, len = 0;
    g_autofree uint8_t *y = NULL, *cb = NULL, *cr = NULL, *out = NULL;
    bool ok;

    if (s->node[N_U82D] == 4) {
        mw = s->wrp[W_WIDTH / 4];
        mh = s->wrp[W_HEIGHT / 4];
    } else {
        mw = gm_mcp100_node32(s, N_ROI_RIGHT) - gm_mcp100_node32(s, N_ROI_LEFT);
        mh = gm_mcp100_node32(s, N_ROI_BOTTOM) - gm_mcp100_node32(s, N_ROI_TOP);
    }
    memset(q, 1, sizeof(q));
    gm_mcp100_parse_dqt(s, q);
    thd = gm_mcp100_node32(s, N_MAX_BS_THD);
    room = thd > s->hdr_len && thd < 64 * MiB ? thd - s->hdr_len : 16 * MiB;
    restart = gm_mcp100_node32(s, N_RESTART);

    if (mw && mh && mw <= GM_MCP100_MAX_MCUS && mh <= GM_MCP100_MAX_MCUS) {
        w = mw * 16;
        h = mh * 16;
        y = g_malloc0((size_t)w * h);
        cb = g_malloc0((size_t)w * h / 4);
        cr = g_malloc0((size_t)w * h / 4);
        out = g_malloc(room);
        ok = gm_mcp100_fetch(s, w, h, y, cb, cr);
        if (ok) {
            len = nvt_pixel_jpeg_scan_rst(out, room, y, w, cb, cr, w / 2, 1,
                                          mw, mh, q[0], q[1], restart);
        }
    }
    if (len) {
        dma_memory_write(&address_space_memory, s->bs_phy + s->hdr_len, out,
                         len, MEMTXATTRS_UNSPECIFIED);
    } else {
        /* would not fit (or no picture): stop at the start, as the firmware */
        static const uint8_t eoi[2] = { 0xFF, 0xD9 };

        dma_memory_write(&address_space_memory, s->bs_phy + s->hdr_len, eoi,
                         2, MEMTXATTRS_UNSPECIFIED);
        len = 2;
        stl_le_p(&s->node[N_STOP_X], 0);
        stl_le_p(&s->node[N_STOP_Y], 0);
        stl_le_p(&s->node[N_MAX_BS_THD], 0x0FFFFFFF);
    }
    stl_le_p(&s->node[N_BITSLEN], s->hdr_len + len);
    stl_le_p(&s->node[N_DONE], 1);
    s->node[N_COMMAND] = CMD_ENCODE_JPEG << 4;
    s->regs[R_INT_FLAG / 4] |= INT_JPEG_DONE;
    gm_mcp100_update(s);
}

static int m4_log2bin(uint32_t v)
{
    return v ? 32 - clz32(v) : 0;
}

/* VOS / VO / VOL headers, as the driver's BitstreamWriteVolHeader() */
static void gm_mcp100_vol(GmMcp100State *s, Mpeg4Enc *e)
{
    const uint32_t base = gm_mcp100_node32(s, N4_FBASE);

    m4_put(e, 0x00000100, 32);          /* video object */
    m4_put(e, 0x00000120, 32);          /* video object layer */
    m4_put(e, 2, 10);                   /* simple object type */
    m4_put(e, 216, 11);                 /* 1:1, 4:2:0, low delay, rectangle */
    m4_put(e, 1, 1);
    m4_put(e, base, 16);
    m4_put(e, 1, 1);
    m4_put(e, 1, 1);                    /* fixed VOP rate */
    m4_put(e, gm_mcp100_node32(s, N4_FINCR),
           MAX(m4_log2bin(base - 1), 1));
    m4_put(e, 1, 1);
    m4_put(e, gm_mcp100_node32(s, N4_ROI_W), 13);
    m4_put(e, 1, 1);
    m4_put(e, gm_mcp100_node32(s, N4_ROI_H), 13);
    m4_put(e, 1, 1);
    m4_put(e, 4, 4);                    /* progressive, no OBMC / sprite */
    m4_put(e, 0, 1);                    /* H.263 quantisation */
    m4_put(e, 12, 4);                   /* no resync markers */
    m4_stuff(e);
}

static void gm_mcp100_vop_header(GmMcp100State *s, Mpeg4Enc *e, bool coded,
                                 int rounding)
{
    const uint32_t base = gm_mcp100_node32(s, N4_FBASE);
    int32_t sec = gm_mcp100_node32(s, N4_MODULE_BASE);

    if (sec < 0) {
        sec = gm_mcp100_node32(s, N4_CUR + F_SECONDS);
    }
    m4_put(e, 0x000001B6, 32);
    m4_put(e, e->intra_vop ? 0 : 1, 2);
    for (sec = MIN(sec, 60); sec > 0; sec--) {
        m4_put(e, 1, 1);
    }
    m4_put(e, 0, 1);
    m4_put(e, 1, 1);
    m4_put(e, gm_mcp100_node32(s, N4_CUR + F_TICKS),
           MAX(m4_log2bin(base - 1), 1));
    m4_put(e, 1, 1);
    m4_put(e, coded, 1);
    if (!coded) {
        m4_stuff(e);
        return;
    }
    if (!e->intra_vop) {
        m4_put(e, rounding, 1);
    }
    m4_put(e, e->q, 8);                 /* DC VLC always, the quantiser */
    if (!e->intra_vop) {
        m4_put(e, 1, 3);
    }
}

static void gm_mcp100_planes(GmMcp100State *s, uint32_t frame, uint8_t **p,
                             size_t luma, bool write)
{
    int c;

    for (c = 0; c < 3; c++) {
        const hwaddr a = gm_mcp100_node32(s, frame + F_RECON + 4 * c);
        const size_t n = c ? luma / 4 : luma;

        if (write) {
            dma_memory_write(&address_space_memory, a, p[c], n,
                             MEMTXATTRS_UNSPECIFIED);
        } else {
            dma_memory_read(&address_space_memory, a, p[c], n,
                            MEMTXATTRS_UNSPECIFIED);
        }
    }
}

/*
 * The source through the DMA wrapper: packed 4:2:2, rows of the picture's
 * width.  Both UYVY orders come from the capture block as it is set up.
 */
static void gm_mcp100_fetch422(GmMcp100State *s, uint8_t **p, int w, int h)
{
    const uint32_t stride = gm_mcp100_node32(s, N4_WIDTH) * 2;
    const hwaddr base = gm_mcp100_node32(s, N4_CUR + F_IMAGE);
    g_autofree uint8_t *row = g_malloc0(w * 2);
    int x, y;

    for (y = 0; y < h; y++) {
        dma_memory_read(&address_space_memory, base + (hwaddr)y * stride,
                        row, w * 2, MEMTXATTRS_UNSPECIFIED);
        for (x = 0; x < w; x++) {
            p[0][y * w + x] = row[x * 2 + 1];
        }
        for (x = 0; x < w / 2; x++) {
            const int o = (y / 2) * (w / 2) + x;

            p[1][o] = y & 1 ? (p[1][o] + row[x * 4] + 1) / 2 : row[x * 4];
            p[2][o] = y & 1 ? (p[2][o] + row[x * 4 + 2] + 1) / 2
                            : row[x * 4 + 2];
        }
    }
}

static void gm_mcp100_mpeg4(GmMcp100State *s, unsigned cmd)
{
    const uint32_t mw = gm_mcp100_node32(s, N4_MB_W);
    const uint32_t mh = gm_mcp100_node32(s, N4_MB_H);
    const hwaddr bs = gm_mcp100_node32(s, N4_BITSTREAM);
    const size_t luma = (size_t)mw * 16 * mh * 16;
    const uint32_t fmt = gm_mcp100_node32(s, N4_IMG_FMT);
    Mpeg4Enc e = { 0 };
    int rounding = gm_mcp100_node32(s, N4_ROUNDING) & 1;

    if (cmd != CMD_ENCODE_N && fmt != 3) {
        /* no picture: the job ends with no bytes */
        qemu_log_mask(LOG_UNIMP, "gm-mcp100: MPEG-4 input format %u\n", fmt);
    } else if (mw && mh && mw <= 128 && mh <= 128) {
        g_autofree uint8_t *src = g_malloc0(luma * 3 / 2);
        g_autofree uint8_t *ref = g_malloc0(luma * 3 / 2);
        g_autofree uint8_t *rec = g_malloc0(luma * 3 / 2);
        g_autofree uint8_t *out = g_malloc(luma * 2 + 4096);
        uint8_t *ps[3] = { src, src + luma, src + luma * 5 / 4 };
        uint8_t *pr[3] = { ref, ref + luma, ref + luma * 5 / 4 };
        uint8_t *pc[3] = { rec, rec + luma, rec + luma * 5 / 4 };
        int c;

        e.out = out;
        e.cap = luma * 2 + 4096;
        e.w = mw * 16;
        e.h = mh * 16;
        e.q = MIN(MAX(gm_mcp100_node32(s, N4_CUR + F_QUANT), 1), 31);
        e.intra_vop = cmd == CMD_ENCODE_I;
        for (c = 0; c < 3; c++) {
            e.src[c] = ps[c];
            e.ref[c] = pr[c];
            e.rec[c] = pc[c];
        }
        rounding = e.intra_vop ? 0 : rounding ^ 1;
        if (e.intra_vop || (gm_mcp100_node32(s, N4_VOL) & 0xFF)) {
            gm_mcp100_vol(s, &e);
        }
        gm_mcp100_vop_header(s, &e, cmd != CMD_ENCODE_N, rounding);
        if (cmd != CMD_ENCODE_N) {
            gm_mcp100_fetch422(s, ps, e.w, e.h);
            if (!e.intra_vop) {
                gm_mcp100_planes(s, N4_REF, pr, luma, false);
            }
            m4_vop_data(&e);
            gm_mcp100_planes(s, N4_CUR, pc, luma, true);
        }
        dma_memory_write(&address_space_memory, bs, out, MIN(e.len, e.cap),
                         MEMTXATTRS_UNSPECIFIED);
        if (s->dump) {                  /* one file per encoder instance */
            g_autofree char *name =
                g_strdup_printf("%s/mpeg4-%ux%u-%08x.m4v", s->dump, e.w, e.h,
                                gm_mcp100_node32(s, N4_PRED));
            FILE *f = fopen(name, "ab");

            if (f) {
                fwrite(out, 1, MIN(e.len, e.cap), f);
                fclose(f);
            }
        }
        stl_le_p(&s->node[N4_ROUNDING], rounding);
        stl_le_p(&s->node[N4_CUR + F_ROUNDING], rounding);
        stl_le_p(&s->node[N4_CUR + F_TYPE], e.intra_vop ? 0 : 1);
    }
    /* what the host's FlushBitstream() adds up to the length */
    s->regs[R_ABADR / 4] = bs + (e.len & ~63);
    s->regs[R_BALR / 4] = e.len & 0x3C;
    s->regs[R_VADR / 4] = (e.len & 3) * 8;
    s->packer_off = true;               /* the flush's padding */
    s->node[N_COMMAND] = cmd << 4;
    s->regs[R_INT_FLAG / 4] |= INT_JPEG_DONE;
    gm_mcp100_update(s);
}

static void gm_mcp100_job(void *opaque)
{
    GmMcp100State *s = opaque;
    const unsigned cmd = s->job;

    s->job = 0;

    if (cmd >= CMD_ENCODE_I && cmd <= CMD_ENCODE_N) {
        gm_mcp100_mpeg4(s, cmd);
        return;
    }

    if (cmd == CMD_ENCODE_JPEG) {
        gm_mcp100_encode(s);
    }
}

/* ---- registers ------------------------------------------------------- */

static uint64_t gm_mcp100_read(void *opaque, hwaddr off, unsigned size)
{
    GmMcp100State *s = opaque;

    switch (off) {
    case R_INCTL:
        return s->regs[off / 4] | INCTL_CPU_IDLE;
    case R_VLDSTS:
        return s->regs[off / 4] | VLDSTS_READY;
    }
    return s->regs[off / 4];
}

static void gm_mcp100_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    GmMcp100State *s = opaque;

    switch (off) {
    case R_RESET:
        val &= ~RESET_GO;
        break;
    case R_BALR:
        if (!s->packer_off) {
            gm_mcp100_put_bits(s, s->regs[R_BADR / 4], MIN(val & 0x3F, 32));
        }
        break;
    case R_ABADR:
        s->packer_off = false;
        s->bs_phy = val & ~3u;
        s->hdr_len = 0;
        s->acc = 0;
        s->acc_bits = 0;
        break;
    case R_INCTL:
        if ((val & INCTL_CPU_START) == INCTL_CPU_START) {
            s->node[N_COMMAND] = 0;     /* the firmware's first act */
        }
        break;
    case R_INT_FLAG:
        s->regs[off / 4] &= ~val;
        gm_mcp100_update(s);
        return;
    }
    s->regs[off / 4] = val;
}

static const MemoryRegionOps gm_mcp100_ops = {
    .read = gm_mcp100_read,
    .write = gm_mcp100_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* the mailbox: plain memory, but for the command byte */
static uint64_t gm_mcp100_node_read(void *opaque, hwaddr off, unsigned size)
{
    GmMcp100State *s = opaque;

    return ldn_le_p(&s->node[off], size);
}

static void gm_mcp100_node_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    GmMcp100State *s = opaque;

    stn_le_p(&s->node[off], size, val);
    if (off == N_COMMAND && ((val & 0xF) == CMD_ENCODE_JPEG ||
                             ((val & 0xF) >= CMD_ENCODE_I &&
                              (val & 0xF) <= CMD_ENCODE_N))) {
        /* latched, as the firmware does: the host may touch the node again */
        s->job = val & 0xF;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            GM_MCP100_JOB_NS);
    } else if (off == N_COMMAND && (val & 0xF)) {
        qemu_log_mask(LOG_UNIMP, "gm-mcp100: firmware command %u\n",
                      (unsigned)(val & 0xF));
    }
}

static const MemoryRegionOps gm_mcp100_node_ops = {
    .read = gm_mcp100_node_read,
    .write = gm_mcp100_node_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static uint64_t gm_mcp100_wrp_read(void *opaque, hwaddr off, unsigned size)
{
    GmMcp100State *s = opaque;

    return off < GM_MCP100_WRP_NREGS * 4 ? s->wrp[off / 4] : 0;
}

static void gm_mcp100_wrp_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    GmMcp100State *s = opaque;

    if (off < GM_MCP100_WRP_NREGS * 4) {
        s->wrp[off / 4] = val;
    }
}

static const MemoryRegionOps gm_mcp100_wrp_ops = {
    .read = gm_mcp100_wrp_read,
    .write = gm_mcp100_wrp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_mcp100_reset(DeviceState *dev)
{
    GmMcp100State *s = GM_MCP100(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->node, 0, sizeof(s->node));
    memset(s->wrp, 0, sizeof(s->wrp));
    s->bs_phy = s->hdr_len = s->acc = 0;
    s->acc_bits = 0;
    s->packer_off = false;
    s->job = 0;
    gm_mcp100_update(s);
}

static void gm_mcp100_realize(DeviceState *dev, Error **errp)
{
    GmMcp100State *s = GM_MCP100(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_mcp100_ops, s,
                          TYPE_GM_MCP100, GM_MCP100_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    memory_region_init_io(&s->node_mem, OBJECT(s), &gm_mcp100_node_ops, s,
                          TYPE_GM_MCP100 ".node", GM_MCP100_NODE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->node_mem);
    memory_region_init_io(&s->wrp_mem, OBJECT(s), &gm_mcp100_wrp_ops, s,
                          TYPE_GM_MCP100 ".dmawrp", GM_MCP100_WRP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->wrp_mem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_mcp100_job, s);
}

static const Property gm_mcp100_props[] = {
    DEFINE_PROP_STRING("dump", GmMcp100State, dump),
};

static const VMStateDescription vmstate_gm_mcp100 = {
    .name = TYPE_GM_MCP100,
    .version_id = 3,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, GmMcp100State),
        VMSTATE_UINT32_ARRAY(regs, GmMcp100State, GM_MCP100_NREGS),
        VMSTATE_UINT8_ARRAY(node, GmMcp100State, GM_MCP100_NODE_SIZE),
        VMSTATE_UINT32_ARRAY(wrp, GmMcp100State, GM_MCP100_WRP_NREGS),
        VMSTATE_UINT32(bs_phy, GmMcp100State),
        VMSTATE_UINT32(hdr_len, GmMcp100State),
        VMSTATE_UINT32(acc, GmMcp100State),
        VMSTATE_INT32(acc_bits, GmMcp100State),
        VMSTATE_UINT8_ARRAY(hdr, GmMcp100State, GM_MCP100_HDR_MAX),
        VMSTATE_BOOL_V(packer_off, GmMcp100State, 3),
        VMSTATE_UINT8_V(job, GmMcp100State, 3),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_mcp100_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_mcp100_realize;
    device_class_set_legacy_reset(dc, gm_mcp100_reset);
    device_class_set_props(dc, gm_mcp100_props);
    dc->vmsd = &vmstate_gm_mcp100;
}

static const TypeInfo gm_mcp100_info = {
    .name           = TYPE_GM_MCP100,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmMcp100State),
    .class_init     = gm_mcp100_class_init,
};

static void gm_mcp100_register_types(void)
{
    type_register_static(&gm_mcp100_info);
}

type_init(gm_mcp100_register_types)
