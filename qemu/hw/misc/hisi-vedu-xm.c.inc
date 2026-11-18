/*
 * hisi-vedu gen 7: the encoders of Goke GK7206 / Xiongmai "xmorca"
 * (lotus,vedu: VEDU at 0x11800000, SPI 45; JPGE at 0x11420000, SPI 40),
 * as XM's xm_vedu.ko / xm_h264e.ko / xm_h265e.ko / xm_jpege.ko /
 * xm_chnl.ko drive them.  Included by hisi-vedu.c.
 *
 * The drivers descend from HiMPP V4: the same stream ring of 64-byte
 * packet headers, the parameter-set packets the driver writes there
 * itself, an H.265 slice header the driver codes and the engine puts
 * behind its own first bits.  The engines do not: they take every
 * parameter over MMIO (no register image in DDR), and the ring packets
 * are padded to 8 bytes, not 64.
 *
 * VEDU (from xm_vedu.ko vedu_hal_*, h26xe_drv_*, rc drv_read_regs and the
 * registers of a GK7206V11A encoding 3840x2160 H.265, read with devmem):
 *
 *   0x010  START       0 then 1 starts a picture
 *   0x020  MODE
 *   0x024  PIC_SIZE    width | height << 16 (not minus one)
 *   0x084  SOC_INTCLR  1 (nothing modelled behind it)
 *   0x090  INTEN       the driver enables 0x10a1 (+ bit 1 per slice)
 *   0x094  INTCLR      write 1s to clear RAWINT bits
 *   0x098  INTSTAT     RAWINT & INTEN
 *   0x09c  RAWINT      0 end of picture, 1 slice end, 2 buffer full,
 *                      4 bits over, 5 timeout, 7 AXI read error,
 *                      9 missed start, 10 ring full, 12 AXI write error,
 *                      14..16 FBD errors (current, ref 0, ref 1)
 *   0x0a4  WORK_TIME   cycles of the last picture
 *   0x0ac  FRM_BYTES   rate control reads bits = this * 8
 *   0x1b8 / 0x1d0      source luma / chroma stride
 *   0x240 / 0x244      source luma / chroma (NV21, yvu420)
 *   0x2d0 / 0x2d4      PTS, 0x2d8 / 0x2dc zero, 0x2e0 channel: the
 *                      engine's packet header +16, +24, +12
 *   0x300  NAL_HDR     the two NAL header bytes, first << 8: 0x0201 a
 *                      TRAIL_R, 0x2601 an IDR_W_RADL
 *   0x304 / 0x308      stream ring size / physical base
 *   0x310              the write pointer's value at configuration
 *   0x324              H.265 slice header bit counts: low 16 the P
 *                      pictures' header at 0x330 (8 words), high 16 the
 *                      IDR pictures' at 0x350 (h265e_drv_update_slc_cfg
 *                      writes the second only for an IDR, or both)
 *   0x330 ..           slice_type onwards, MSB first in each word: the
 *                      engine codes first_slice_segment_in_pic_flag,
 *                      [no_output_of_prior_pics_flag,] PPS id 0 before it
 *                      (checked against a real camera's ring)
 *   0x370 / 0x374      physical address of the ring's write / read
 *                      pointer cell (offsets; the driver's stream buffer
 *                      struct sits behind the ring: +0x40 read, +0x80
 *                      write)
 *   0xd6c .. 0xdc4     statistics xm_rc reads after each picture (see
 *                      xm_vedu_stats)
 *   0xe00  bit 0       FBC enable;  0xe40 bit 0: compressed source
 *
 * The engine appends one packet per picture at the write pointer: header
 * { length (header included), padding, NAL type | last << 24, channel,
 * PTS, 0, 0 } and the slice (00 00 01 start code, NAL header, slice
 * header, data) padded to 8 bytes, then leaves the write pointer cell
 * behind it.
 *
 * JPGE (xm_jpege.ko jpeg_hal_*): parameters over MMIO too.
 *
 *   0x004  bits 2:0 the source: 1 YVU420 (NV21), 2 YUV420 (NV12), 0
 *                      YUV400, 3 / 4 the first two coded 4:2:2
 *                      (420_to_422); bit 3 chroma self-generated, bit 4
 *                      slice mode, 6:5 tile mode (1 linear), 22:7 slice
 *                      height.  A real camera's snapshot channel (yvu420
 *                      800x448): 0xe021.
 *   0x024 / 0x028      width / height, not minus one (real: 0x320, 0x1c0)
 *   0x030 / 0x034      source chroma / luma, 0x038 / 0x040 their strides
 *   0x048 / 0x050      luma / chroma quantiser write port: bit 27 write,
 *                      bits 26:21 index (natural, row-major order: the
 *                      table the driver also writes into its DQT), 20:0
 *                      2^20 / q
 *   0x060 / 0x064      stream buffer start / end (physical)
 *   0x068              where the scan goes (physical), 0x070 read pointer
 *   0x074              write pointer the engine leaves (offset from 0x060)
 *   0x078              invalid bytes at its end
 *   0x080  bit 0       frame start
 *   0x200 / 0x204 / 0x208   SoC interrupt enable / clear / status (bit 0)
 *   0x214 / 0x218 / 0x220 / 0x224  interrupt enable (0x323) / clear /
 *                      status / raw: 0 slice end, 1 frame end, 2 buffer
 *                      full, 3 FBD error, 4 timeout, 7 loop overflow,
 *                      8 / 9 AXI write / read error
 *
 * The driver writes the JFIF header up to DRI (its DHT: Annex K's luma DC
 * and AC tables, as tables 0 and 1 alike) and the scan's packet header;
 * the engine writes SOS, the entropy-coded scan and EOI.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#define XM_VEDU_START           0x0010
#define XM_VEDU_PIC_SIZE        0x0024
#define XM_VEDU_SOC_INTCLR      0x0084
#define XM_VEDU_INTEN           0x0090
#define XM_VEDU_INTCLR          0x0094
#define XM_VEDU_INTSTAT         0x0098
#define XM_VEDU_RAWINT          0x009c
#define XM_VEDU_WORK_TIME       0x00a4
#define XM_VEDU_FRM_BYTES       0x00ac
#define XM_VEDU_SRC_Y_STRIDE    0x01b8
#define XM_VEDU_SRC_C_STRIDE    0x01d0
#define XM_VEDU_SRC_Y           0x0240
#define XM_VEDU_SRC_C           0x0244
#define XM_VEDU_PTS_LO          0x02d0
#define XM_VEDU_PTS_HI          0x02d4
#define XM_VEDU_PKT_W6          0x02d8
#define XM_VEDU_PKT_W7          0x02dc
#define XM_VEDU_CHN             0x02e0
#define XM_VEDU_NAL_HDR         0x0300
#define XM_VEDU_STRM_SIZE       0x0304
#define XM_VEDU_STRM_BASE       0x0308
#define XM_VEDU_SLC_HDR_LEN     0x0324
#define XM_VEDU_SLC_HDR         0x0330
#define XM_VEDU_SLC_HDR2        0x0350
#define XM_VEDU_STRM_WP         0x0370
#define XM_VEDU_STRM_RP         0x0374
#define XM_VEDU_SRC_CMP         0x0e40

/* what xm_rc's drv_read_regs reads back (xm_vedu_stats) */
#define XM_VEDU_ST_INTRA        0x0d6c
#define XM_VEDU_ST_INTER        0x0d70
#define XM_VEDU_ST_D74          0x0d74
#define XM_VEDU_ST_SSE_Y_LO     0x0d84
#define XM_VEDU_ST_SSE_Y_HI     0x0d88  /* bits 9:0 */
#define XM_VEDU_ST_QP_SUM       0x0d8c
#define XM_VEDU_ST_CODED8       0x0d90
#define XM_VEDU_ST_SSE_C_LO     0x0d94
#define XM_VEDU_ST_SSE_C_HI     0x0d98
#define XM_VEDU_ST_COST_I       0x0da4
#define XM_VEDU_ST_GMV_X        0x0dbc
#define XM_VEDU_ST_GMV_Y        0x0dc0
#define XM_VEDU_ST_GMV_N        0x0dc4

#define XM_VEDU_INT_EOP         (1u << 0)
#define XM_VEDU_INT_BUF_FULL    (1u << 2)
#define XM_PKT_ALIGN            8

#define XM_JPGE_CTRL            0x0004
#define XM_JPGE_CHR_SELFGEN     (1u << 3)
#define XM_JPGE_FMT_YUV400      0       /* jpege_get_input_cfg's formats */
#define XM_JPGE_FMT_NV21        1
#define XM_JPGE_FMT_NV12        2
#define XM_JPGE_FMT_NV21_422    3       /* 420_to_422 */
#define XM_JPGE_FMT_NV12_422    4
#define XM_JPGE_HOR             0x0024
#define XM_JPGE_VER             0x0028
#define XM_JPGE_SRC_C           0x0030
#define XM_JPGE_SRC_Y           0x0034
#define XM_JPGE_C_STRIDE        0x0038
#define XM_JPGE_Y_STRIDE        0x0040
#define XM_JPGE_QUANT_Y         0x0048
#define XM_JPGE_QUANT_C         0x0050
#define XM_JPGE_BS_START        0x0060
#define XM_JPGE_BS_END          0x0064
#define XM_JPGE_BS_WR           0x0068
#define XM_JPGE_BS_RD           0x0070
#define XM_JPGE_RPT_WPTR        0x0074
#define XM_JPGE_RPT_INVLD       0x0078
#define XM_JPGE_FRAME_START     0x0080
#define XM_JPGE_SOC_EN          0x0200
#define XM_JPGE_SOC_CLR         0x0204
#define XM_JPGE_SOC_STAT        0x0208
#define XM_JPGE_SOC_RAW         0x0210
#define XM_JPGE_INTEN           0x0214
#define XM_JPGE_INTCLR          0x0218
#define XM_JPGE_INTSTAT         0x0220
#define XM_JPGE_RAWINT          0x0224
#define XM_JPGE_WORK_TIME       0x0228

#define XM_JPGE_INT_FRM_EOF     (1u << 1)
#define XM_JPGE_INT_BUF_FULL    (1u << 2)

static void xm_vedu_update_irq(HisiVeduState *s)
{
    qemu_set_irq(s->vedu_irq,
                 (s->vedu_raw & s->vedu_regs[XM_VEDU_INTEN / 4]) != 0);
}

/*
 * Measure a coded picture for the rate control: source (pw-wide luma,
 * interleaved CbCr) against the picture decoded from it (recon, NULL if
 * the encoder keeps none) and against the reference it was predicted from
 * (prev, NULL for an I picture), both rw wide, over w x h.
 */
static void xm_vedu_measure(VeduChannel *c, const uint8_t *y,
                            const uint8_t *uv, int pw, int w, int h,
                            const HevcEncRef *prev, const HevcEncRef *recon,
                            int rw)
{
    const uint8_t *py = NULL, *pcb = NULL, *pcr = NULL;
    const uint8_t *ry = NULL, *rcb = NULL, *rcr = NULL;
    uint64_t sse_y = 0, sse_c = 0, madi16 = 0, sad = 0;
    uint32_t intra8 = 0;
    int bx, by, i, j;

    if (prev) {
        hevc_enc_ref_planes(prev, &py, &pcb, &pcr);
    }
    if (recon) {
        hevc_enc_ref_planes(recon, &ry, &rcb, &rcr);
    }
    w &= ~7;
    h &= ~7;
    for (by = 0; by < h; by += 8) {
        for (bx = 0; bx < w; bx += 8) {
            bool coded = !py || !ry;
            int q;

            for (q = 0; q < 4; q++) {           /* the 4x4 blocks */
                const int x0 = bx + (q & 1) * 4, y0 = by + (q >> 1) * 4;
                int sum = 0;

                for (j = 0; j < 4; j++) {
                    const uint8_t *s = y + (size_t)(y0 + j) * pw + x0;

                    sum += s[0] + s[1] + s[2] + s[3];
                }
                for (j = 0; j < 4; j++) {
                    const uint8_t *s = y + (size_t)(y0 + j) * pw + x0;

                    for (i = 0; i < 4; i++) {
                        madi16 += abs(s[i] * 16 - sum);
                    }
                }
            }
            for (j = 0; j < 8; j++) {
                const size_t so = (size_t)(by + j) * pw + bx;
                const size_t ro = (size_t)(by + j) * rw + bx;

                if (ry) {
                    for (i = 0; i < 8; i++) {
                        const int d = y[so + i] - ry[ro + i];

                        sse_y += d * d;
                    }
                }
                if (py) {
                    for (i = 0; i < 8; i++) {
                        sad += abs(y[so + i] - py[ro + i]);
                    }
                }
                if (py && ry && !coded) {
                    coded = memcmp(py + ro, ry + ro, 8) != 0;
                }
            }
            intra8 += coded;
        }
    }
    if (rcb) {
        for (j = 0; j < h / 2; j++) {
            const uint8_t *s = uv + (size_t)j * pw;
            const size_t ro = (size_t)j * (rw / 2);

            for (i = 0; i < w / 2; i++) {
                const int db = s[i * 2] - rcb[ro + i];
                const int dr = s[i * 2 + 1] - rcr[ro + i];

                sse_c += db * db + dr * dr;
            }
        }
    }
    c->xm_st.sse_y = sse_y;
    c->xm_st.sse_c = sse_c;
    c->xm_st.madi = madi16 / 16;
    c->xm_st.sad = sad;
    c->xm_st.intra8 = intra8;
    c->xm_st.valid = true;
}

/*
 * What xm_rc's drv_read_regs reads after a picture and what its
 * rc_read_one_frame makes of it (pixels = 0x24's w * h):
 *   0x0ac        bytes: pic_bits = this * 8
 *   0x d8c       bits 26:0, mean_qp = this / (pixels / 16): the QPs of the
 *                picture's 4x4 blocks, summed
 *   0xd84/0xd88  bits 41:0, + 0xd94/0xd98: mse = their sum / pixels (its
 *                log: "pic_bits ... mean_qp ... mse"; below 0.001 it takes
 *                the picture for a solid one); 0xd84 alone feeds its scene
 *                conditions as luma: luma and chroma squared error here
 *   0xd6c, 0xd70 bits 28:0: per 4x4 block (pixels / 16), and the second as
 *                a percentage of the first against a scene change
 *                threshold: intra and inter activity, as sums of absolute
 *                differences / 16
 *   0xda4        intra cost (per pixel for the I picture's bits): the sum
 *                of |sample - its 4x4 block's mean|
 *   0xd90        bits 18:0, a percentage of the 8x8 blocks: those coded
 *   0xd74        bits 24:0, not used
 *   0xdbc, 0xdc0 signed 24 bits: motion vector sums, global motion =
 *                these / (0xdc4 + 1) when 0xdc4, a block count, reaches
 *                40 % of the 16x16 blocks: no global motion here
 */
static void xm_vedu_stats(HisiVeduState *s, VeduChannel *c, uint32_t bytes,
                          int w, int h, int qp, bool intra)
{
    const uint64_t px = (uint64_t)w * h;
    uint64_t sse_y = c->xm_st.sse_y, sse_c = c->xm_st.sse_c;
    uint64_t madi = c->xm_st.madi, sad = intra ? 0 : c->xm_st.sad;
    uint32_t *r = s->vedu_regs;

    if (!c->xm_st.valid || (!sse_y && !sse_c)) {
        /* not measured (H.264): the error of a uniform quantiser */
        const double step = pow(2.0, (qp - 4) / 6.0);

        sse_y = px * step * step / 12;
        sse_c = sse_y / 4;
    }
    r[XM_VEDU_FRM_BYTES / 4] = bytes;
    r[XM_VEDU_WORK_TIME / 4] = px / 2;
    r[XM_VEDU_ST_INTRA / 4] = MIN(madi / 16, 0x1fffffff);
    r[XM_VEDU_ST_INTER / 4] = MIN(sad / 16, 0x1fffffff);
    r[XM_VEDU_ST_D74 / 4] = 0;
    r[XM_VEDU_ST_SSE_Y_LO / 4] = sse_y;
    r[XM_VEDU_ST_SSE_Y_HI / 4] = (sse_y >> 32) & 0x3ff;
    r[XM_VEDU_ST_QP_SUM / 4] = MIN(px / 16 * qp, 0x7ffffff);
    r[XM_VEDU_ST_CODED8 / 4] = MIN(intra ? px / 64 : c->xm_st.intra8,
                                   0x7ffff);
    r[XM_VEDU_ST_SSE_C_LO / 4] = sse_c;
    r[XM_VEDU_ST_SSE_C_HI / 4] = (sse_c >> 32) & 0x3ff;
    r[XM_VEDU_ST_COST_I / 4] = MIN(madi, 0xffffffffu);
    r[XM_VEDU_ST_GMV_X / 4] = 0;
    r[XM_VEDU_ST_GMV_Y / 4] = 0;
    r[XM_VEDU_ST_GMV_N / 4] = 0;
    if (s->trace) {
        fprintf(stderr, "vedu stats: %ux%u qp %d %u bytes mse %.2f madi "
                "%.2f sad %.2f coded8 %u\n", w, h, qp, bytes,
                (double)(sse_y + sse_c) / px, (double)madi / px,
                (double)sad / px, r[XM_VEDU_ST_CODED8 / 4]);
    }
}

/*
 * The source, NV21.  A compressed one (0xe40 bit 0, VPSS's own frame
 * compression) is read as linear at the frame's strides: lotus-vpss
 * writes every frame uncompressed.
 */
static void xm_vedu_load(const uint32_t *r, uint32_t ys, uint32_t cs, int w,
                         int h, size_t mbw, size_t mbh, uint8_t *y,
                         uint8_t *uv)
{
    const uint8_t *py, *pc;
    int pw, ph;
    bool nv12;

    /* a wrap-buffer source: DDR holds only a few lines of it */
    if (lotus_vpss_picture(r[XM_VEDU_SRC_Y / 4], r[XM_VEDU_SRC_C / 4], &pw,
                           &ph, &py, &pc, &nv12) && pw == w && ph == h) {
        const int stride = mbw * 16;

        for (int row = 0; row < (int)mbh * 16; row++) {
            const uint8_t *l = py + (size_t)MIN(row, h - 1) * w;

            memcpy(y + (size_t)row * stride, l, w);
            memset(y + (size_t)row * stride + w, l[w - 1], stride - w);
        }
        for (int row = 0; row < (int)mbh * 8; row++) {
            const uint8_t *l = pc + (size_t)MIN(row, h / 2 - 1) * w;
            uint8_t *d = uv + (size_t)row * stride;

            for (int x = 0; x < stride / 2; x++) {
                const int sx = MIN(x, w / 2 - 1) * 2;

                d[x * 2] = l[sx + (nv12 ? 0 : 1)];
                d[x * 2 + 1] = l[sx + (nv12 ? 1 : 0)];
            }
        }
        return;
    }
    vedu_load_nv12(r[XM_VEDU_SRC_Y / 4], r[XM_VEDU_SRC_C / 4], ys, cs, w, h,
                   true, mbw, mbh, y, uv, false);
}

/* one picture from the registers START latched; the raw bits it ends with */
static uint32_t xm_vedu_encode(HisiVeduState *s)
{
    const uint32_t *r = s->img;
    const int w = r[XM_VEDU_PIC_SIZE / 4] & 0xffff;
    const int h = r[XM_VEDU_PIC_SIZE / 4] >> 16;
    const uint32_t base = r[XM_VEDU_STRM_BASE / 4];
    const uint32_t size = r[XM_VEDU_STRM_SIZE / 4];
    const uint32_t wp_cell = r[XM_VEDU_STRM_WP / 4];
    const uint32_t nal_hdr = r[XM_VEDU_NAL_HDR / 4] & 0xffff;
    const int nal_type = (nal_hdr >> 9) & 0x3f;
    const size_t mbw = DIV_ROUND_UP(w, 16), mbh = DIV_ROUND_UP(h, 16);
    uint32_t ys = r[XM_VEDU_SRC_Y_STRIDE / 4], cs = r[XM_VEDU_SRC_C_STRIDE / 4];
    uint32_t start, room, hdr[PKT_HDR / 4] = { 0 }, total;
    g_autofree uint8_t *y = NULL, *uv = NULL, *nal = NULL;
    VeduChannel *c;
    size_t want, cap, n;
    bool idr;
    int qp = 30;

    if (!base || size < 4 * PKT_HDR || size > RING_MAX || !wp_cell ||
        w < 16 || h < 16 || w > PIC_MAX || h > PIC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-vedu: picture not set up "
                      "(ring 0x%x+0x%x, %dx%d)\n", base, size, w, h);
        return XM_VEDU_INT_EOP;
    }
    c = vedu_channel(s, base, false);
    start = vedu_ldl(wp_cell) % size;
    idr = vedu_scan_packets(s, c, base, size, start);
    room = ring_room(size, start, r[XM_VEDU_STRM_RP / 4]);

    /*
     * The bound for a picture, but no more than half the ring: a 4K one's
     * would exceed any ring, and then every failure (headers the model
     * cannot follow) would read as a full buffer, which the driver answers
     * by restarting the engine at once.
     */
    want = MIN(mbw * mbh * 512 + 4096, size / 2);
    cap = room > 2 * PKT_HDR ? MIN(want, (size_t)room - 2 * PKT_HDR) : 0;
    if (cap < 1024) {
        return XM_VEDU_INT_EOP | XM_VEDU_INT_BUF_FULL;
    }

    if (ys < (uint32_t)w) {
        ys = ROUND_UP(w, 16);
    }
    if (cs < (uint32_t)w) {
        cs = ys;
    }
    y = g_malloc(mbw * mbh * 256);
    uv = g_malloc(mbw * mbh * 128);

    nal = g_malloc(cap + 64);
    if (c->codec == CODEC_H265) {
        /*
         * The V4 picture path takes the slice header as part 1 (bits up to
         * the PPS id) and part 2 (slice_type onwards): XM's registers hold
         * part 2; part 1 is what the engine codes from the NAL type.
         */
        uint32_t *v4 = g_new0(uint32_t, IMG_WORDS);
        const bool irap = nal_type >= 16 && nal_type <= 23;
        /* an IRAP picture's header is the second one (0x350) */
        const unsigned slc = irap ? XM_VEDU_SLC_HDR2 : XM_VEDU_SLC_HDR;
        const uint32_t n2 = irap ? r[XM_VEDU_SLC_HDR_LEN / 4] >> 16
                                 : r[XM_VEDU_SLC_HDR_LEN / 4] & 0xffff;
        bool irap_out;
        int i;

        v4[IMG_SLC_HDR_LEN / 4] = (irap ? 2 : 1) | MIN(n2, 256u) << 16;
        v4[IMG_SLC_HDR_PART1 / 4] = irap ? 0x4000 : 0x8000;
        for (i = 0; i < 8; i++) {
            v4[IMG_SLC_HDR_PART2 / 4 + i] = r[slc / 4 + i];
        }
        c->no_search = (uint64_t)w * h > s->inter_max_pixels;
        c->skip_p = (uint64_t)w * h > s->skip_min_pixels;
        /* an all-skipped P picture coded before needs no source (a 4K
         * one takes milliseconds to read) */
        n = c->skip_p && !irap
            ? vedu_h265_picture(c, v4, &irap_out, &qp, NULL, NULL, mbw * 16,
                                mbh * 16, nal, cap)
            : 0;
        if (n) {
            c->xm_st.intra8 = 0;        /* every CU skipped */
        } else {
            xm_vedu_load(r, ys, cs, w, h, mbw, mbh, y, uv);
            n = vedu_h265_picture(c, v4, &irap_out, &qp, y, uv, mbw * 16,
                                  mbh * 16, nal, cap);
            if (n) {
                /* coded afresh: hevc_ref now holds the decoded picture,
                 * hevc_spare the one it was predicted from */
                const HevcHdrs *hh = &c->hevc;
                const bool same = c->ref_w == hh->w && c->ref_h == hh->h;

                xm_vedu_measure(c, y, uv, mbw * 16, MIN(w, hh->w),
                                MIN(h, hh->h),
                                irap_out || !same ? NULL : c->hevc_spare,
                                c->hevc_ref, hh->w);
                c->ref_w = hh->w;
                c->ref_h = hh->h;
            }
        }
        g_free(v4);
        idr = irap_out;
    } else {
        xm_vedu_load(r, ys, cs, w, h, mbw, mbh, y, uv);
        n = vedu_h264_picture(c, &idr, qp, w, h, y, uv, nal, cap);
        if (n) {
            xm_vedu_measure(c, y, uv, mbw * 16, w, h, NULL, NULL, w);
        }
    }
    if (!n) {
        if (cap < want) {
            return XM_VEDU_INT_EOP | XM_VEDU_INT_BUF_FULL;
        }
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-vedu: no usable parameter sets "
                      "for ring 0x%x, picture dropped\n", base);
        return XM_VEDU_INT_EOP;
    }

    /*
     * The engine's slices start with a three-byte start code, and the
     * App's RTP packetizer takes the NAL header from the fourth byte: a
     * four-byte one turns every P slice into NAL type 0 on the wire.
     */
    if (n > 4 && !nal[0] && !nal[1] && !nal[2] && nal[3] == 1) {
        memmove(nal, nal + 1, --n);
    }
    total = PKT_HDR + ROUND_UP(n, XM_PKT_ALIGN);
    hdr[0] = cpu_to_le32(total);
    hdr[1] = cpu_to_le32(total - PKT_HDR - n);
    hdr[2] = cpu_to_le32((c->codec == CODEC_H265 ? (idr ? 19 : 1)
                                                 : (idr ? 5 : 1)) |
                         (1u << 24));                   /* last slice */
    hdr[3] = cpu_to_le32(r[XM_VEDU_CHN / 4]);
    hdr[4] = cpu_to_le32(r[XM_VEDU_PTS_LO / 4]);
    hdr[5] = cpu_to_le32(r[XM_VEDU_PTS_HI / 4]);
    hdr[6] = cpu_to_le32(r[XM_VEDU_PKT_W6 / 4]);
    hdr[7] = cpu_to_le32(r[XM_VEDU_PKT_W7 / 4]);
    ring_write(base, size, start, hdr, PKT_HDR);
    memset(nal + n, 0, total - PKT_HDR - n);
    ring_write(base, size, start + PKT_HDR, nal, total - PKT_HDR);
    vedu_dump(s, c, nal, n);

    c->scanned = (start + total) % size;
    vedu_stl(wp_cell, c->scanned);
    xm_vedu_stats(s, c, total, w, h, qp, idr);
    return XM_VEDU_INT_EOP;
}

static uint32_t xm_vedu_read(HisiVeduState *s, hwaddr offset)
{
    switch (offset) {
    case XM_VEDU_INTSTAT:
        return s->vedu_raw & s->vedu_regs[XM_VEDU_INTEN / 4];
    case XM_VEDU_RAWINT:
        return s->vedu_raw;
    case XM_VEDU_INTCLR:
    case XM_VEDU_SOC_INTCLR:
        return 0;
    }
    return s->vedu_regs[offset / 4];
}

static void xm_vedu_write(HisiVeduState *s, hwaddr offset, uint32_t val)
{
    switch (offset) {
    case XM_VEDU_INTCLR:
        s->vedu_raw &= ~val;
        xm_vedu_update_irq(s);
        return;
    case XM_VEDU_SOC_INTCLR:
        return;
    case XM_VEDU_INTEN:
        s->vedu_regs[offset / 4] = val;
        xm_vedu_update_irq(s);
        return;
    case XM_VEDU_START:
        s->vedu_regs[offset / 4] = val;
        if ((val & 1) && !timer_pending(s->vedu_timer)) {
            /* the engine works from what was programmed at START */
            memcpy(s->img, s->vedu_regs, sizeof(s->img));
            if (s->trace) {
                int i;

                fprintf(stderr, "vedu START (xm)\n");
                for (i = 0; i < IMG_WORDS; i += 4) {
                    if (s->img[i] | s->img[i + 1] | s->img[i + 2] |
                        s->img[i + 3]) {
                        fprintf(stderr, "reg %04x: %08x %08x %08x %08x\n",
                                i * 4, s->img[i], s->img[i + 1],
                                s->img[i + 2], s->img[i + 3]);
                    }
                }
            }
            timer_mod(s->vedu_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VEDU_DONE_NS);
        }
        return;
    }
    s->vedu_regs[offset / 4] = val;
}

/* ------------------------------------------------------------------ */

static void xm_jpge_update_irq(HisiVeduState *s)
{
    qemu_set_irq(s->jpge_irq,
                 s->xm_jsoc && (s->jpge_regs[XM_JPGE_SOC_EN / 4] & 1));
}

/* SOS: Y on DC/AC tables 0, Cb and Cr on 1, the whole spectrum */
static const uint8_t xm_jpge_sos[14] = {
    0xff, 0xda, 0x00, 0x0c, 0x03, 0x01, 0x00, 0x02, 0x11, 0x03, 0x11,
    0x00, 0x3f, 0x00,
};

static uint32_t xm_jpge_encode(HisiVeduState *s)
{
    const uint32_t *r = s->jimg;
    const int w = r[XM_JPGE_HOR / 4] & 0xffff;
    const int h = r[XM_JPGE_VER / 4] & 0xffff;
    const uint32_t base = r[XM_JPGE_BS_START / 4];
    const uint32_t end = r[XM_JPGE_BS_END / 4];
    const uint32_t wr = r[XM_JPGE_BS_WR / 4], rd = r[XM_JPGE_BS_RD / 4];
    const uint32_t size = end - base;
    const size_t mbw = DIV_ROUND_UP(w, 16), mbh = DIV_ROUND_UP(h, 16);
    const size_t pw = mbw * 16, ph = mbh * 16;
    g_autofree uint8_t *y = NULL, *uv = NULL, *cb = NULL, *cr = NULL;
    g_autofree uint8_t *out = NULL;
    uint32_t ys = r[XM_JPGE_Y_STRIDE / 4], cs = r[XM_JPGE_C_STRIDE / 4];
    const uint32_t fmt = r[XM_JPGE_CTRL / 4] & 7;
    const bool selfgen = r[XM_JPGE_CTRL / 4] & XM_JPGE_CHR_SELFGEN;
    uint32_t start, rp, room;
    uint8_t ql[64], qc[64];
    bool overflow = false;
    VeduChannel *c;
    size_t cap, n, i;

    if (!base || end <= base || size > RING_MAX || wr < base || wr >= end ||
        w < 8 || h < 8 || w > PIC_MAX || h > PIC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-jpge: picture not set up "
                      "(stream 0x%x..0x%x at 0x%x, %dx%d)\n", base, end, wr,
                      w, h);
        return XM_JPGE_INT_FRM_EOF;
    }
    c = vedu_channel(s, base, true);
    start = wr - base;
    rp = rd >= base && rd < end ? rd - base : start;
    room = size - (start + size - rp) % size - 1;

    /*
     * Register i is natural (row-major) coefficient i: jpeg_hal_cfg_dqt
     * writes 2^20 / q of the very table jpege_update_dqt_mark puts into
     * the header in zigzag order (dering mode transposes both alike).
     */
    for (i = 0; i < 64; i++) {
        const uint32_t ry = s->xm_jq[0][i] ?: 1, rc = s->xm_jq[1][i] ?: 1;

        ql[i] = MIN(255, MAX(1, ((1u << 20) + ry / 2) / ry));
        qc[i] = MIN(255, MAX(1, ((1u << 20) + rc / 2) / rc));
    }

    cap = room > 2 * PKT_HDR ? MIN(pw * ph * 2 + 1024,
                                   (size_t)room - 2 * PKT_HDR) : 0;
    if (cap < 1024) {
        return XM_JPGE_INT_BUF_FULL;
    }
    if (ys < (uint32_t)w) {
        ys = ROUND_UP(w, 16);
    }
    if (cs < (uint32_t)w) {
        cs = ys;
    }
    y = g_malloc(pw * ph);
    uv = g_malloc(pw * ph / 2);
    cb = g_malloc(pw * ph / 4);
    cr = g_malloc(pw * ph / 4);
    if (fmt != XM_JPGE_FMT_NV21 && fmt != XM_JPGE_FMT_NV12) {
        qemu_log_mask(LOG_UNIMP, "hisi-jpge: format %u coded as 4:2:0\n",
                      fmt);
    }
    vedu_load_nv12(r[XM_JPGE_SRC_Y / 4], r[XM_JPGE_SRC_C / 4], ys, cs, w, h,
                   fmt != XM_JPGE_FMT_NV12 && fmt != XM_JPGE_FMT_NV12_422,
                   mbw, mbh, y, uv, false);
    for (i = 0; i < pw * ph / 4; i++) {
        cb[i] = selfgen ? 128 : uv[i * 2];
        cr[i] = selfgen ? 128 : uv[i * 2 + 1];
    }
    /*
     * The driver's header stops after DRI: its 593-byte template ends in
     * an SOS (rodata 0x767) but the packet it writes does not carry it,
     * and the stock App's snapshots came out with the scan right behind
     * DRI.  A real camera's snapshot has the SOS: the engine writes it.
     */
    out = g_malloc(cap + 64);
    memcpy(out, xm_jpge_sos, sizeof(xm_jpge_sos));
    n = sizeof(xm_jpge_sos);
    /* its DHT has Annex K's luma tables as tables 1 too */
    n += jpeg_enc_scan_huff(y, pw, cb, cr, pw / 2, w, h, ql, qc, true, true,
                            out + n, cap - n, &overflow);
    if (overflow) {
        return XM_JPGE_INT_BUF_FULL;
    }
    ring_write(base, size, start, out, n);
    vedu_dump(s, c, out, n);
    s->jpge_regs[XM_JPGE_RPT_WPTR / 4] = (start + n) % size;
    s->jpge_regs[XM_JPGE_RPT_INVLD / 4] = 0;
    s->jpge_regs[XM_JPGE_WORK_TIME / 4] = pw * ph / 4;
    return XM_JPGE_INT_FRM_EOF;
}

static uint32_t xm_jpge_read(HisiVeduState *s, hwaddr offset)
{
    switch (offset) {
    case XM_JPGE_SOC_STAT:
    case XM_JPGE_SOC_RAW:
        return s->xm_jsoc;
    case XM_JPGE_INTSTAT:
        return s->jpge_raw & s->jpge_regs[XM_JPGE_INTEN / 4];
    case XM_JPGE_RAWINT:
        return s->jpge_raw;
    case XM_JPGE_INTCLR:
    case XM_JPGE_SOC_CLR:
        return 0;
    }
    return s->jpge_regs[offset / 4];
}

static void xm_jpge_write(HisiVeduState *s, hwaddr offset, uint32_t val)
{
    switch (offset) {
    case XM_JPGE_QUANT_Y:
    case XM_JPGE_QUANT_C:
        if (val & (1u << 27)) {
            s->xm_jq[offset == XM_JPGE_QUANT_C][(val >> 21) & 63] =
                val & 0x1fffff;
        }
        break;
    case XM_JPGE_INTCLR:
        s->jpge_raw &= ~val;
        return;
    case XM_JPGE_SOC_CLR:
        if (val & 1) {
            s->xm_jsoc = false;
            xm_jpge_update_irq(s);
        }
        return;
    case XM_JPGE_SOC_EN:
        s->jpge_regs[offset / 4] = val;
        xm_jpge_update_irq(s);
        return;
    case XM_JPGE_FRAME_START:
        s->jpge_regs[offset / 4] = val;
        if ((val & 1) && !timer_pending(s->jpge_timer)) {
            memcpy(s->jimg, s->jpge_regs, sizeof(s->jimg));
            timer_mod(s->jpge_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VEDU_DONE_NS);
        }
        return;
    }
    s->jpge_regs[offset / 4] = val;
}
