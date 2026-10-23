/*
 * Novatek NA51089 MDBC, motion detection by background modelling
 * (kdrv_md: mdbc_reg.h, mdbc_lib.h), the work of one trigger.
 *
 * A frame is 8-bit Y (DRAM_SAI0, line offset DRAM_OFSI0) and NV12 CbCr
 * (SAI1, OFSI1).  The engine keeps a model of the background a pixel in
 * three DRAM buffers it alone reads -- the samples (SAI3 in, SAO1 out,
 * 5 bytes a sample), var1 (SAI4 / SAO2, 6 bytes a pixel) and var2 (SAI5 /
 * SAO3) -- and writes the foreground as one bit a pixel, row-major with no
 * padding, LSB first (SAO0).  Software only ever passes the model buffers
 * back, in place, so their layout is the emulation's own.
 *
 * Nothing in the SDK describes the algorithm; its parameters are named
 * after SuBSENSE (LBSP descriptors, a sample model, minimum matches, an
 * update rate, neighbour diffusion, ghost removal), and this is a
 * deterministic, reduced SuBSENSE / ViBe:
 * - INIT (MODE0 bit 0 clear) fills each pixel's samples with its own
 *   value in the frame and reports no motion (seeding them from
 *   neighbours, as ViBe does, leaves a still scene's edges in the
 *   foreground for frames);
 * - otherwise a sample matches when |Y - Yk| <= D_COLOUR, the Hamming
 *   distance of the 16-bit LBSP descriptors <= D_LBSP and, without Y-only
 *   mode, |U - Uk| + |V - Vk| <= UV_THRES (a region's own values inside an
 *   enabled ROI); a pixel with fewer than BC_MIN_MATCH matches is
 *   foreground;
 * - a background pixel replaces a random sample with itself, at a rate of
 *   1 / BC_MIN_T, and with neighbour updates on, one of a random
 *   neighbour's too; with ghost removal on, a pixel foreground for more
 *   than BC_MAX_FG_FRM frames updates as if background;
 * - the foreground then goes through up to four 3x3 stages: a stage with
 *   a threshold t counts the 8 neighbours in the foreground and, if its
 *   select bit is set, adds pixels with t or more, else keeps only
 *   those; a threshold of 0 skips the stage; the dilation threshold adds
 *   pixels last.
 * The random numbers are a 15-bit LFSR kept in UPDATE4's RND field (seeded
 * by DBG_RND when enabled), so a sequence of frames always gives the same
 * result.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/misc/nvt-md.h"

#define R(off)              regs[(off) / 4]

#define MD_MODE0            0x004
#define MD_SAI_Y            0x010
#define MD_SAI_UV           0x014
#define MD_OFSI_Y           0x018
#define MD_OFSI_UV          0x01C
#define MD_SAI_MODEL        0x024
#define MD_SAI_VAR1         0x028
#define MD_SAO_S1           0x034
#define MD_SAO_MODEL        0x038
#define MD_SAO_VAR1         0x03C
#define MD_SIZE             0x048
#define MD_MATCH0           0x04C
#define MD_MATCH1           0x050
#define MD_MATCH2           0x054
#define MD_MATCH3           0x058
#define MD_MATCH4           0x05C
#define MD_MATCH5           0x060
#define MD_MORPH            0x064
#define MD_UPDATE0          0x068
#define MD_UPDATE3          0x074
#define MD_UPDATE4          0x078
#define MD_ROI(i)           (0x07C + 0x10 * (i))

#define MD_MODE_NORM        (1u << 0)
#define MD_MODE_NEI         (1u << 4)
#define MD_MODE_DEGHOST     (1u << 8)
#define MD_MODE_Y_ONLY      (1u << 28)

#define MD_ROIS             8
#define MD_SAMPLE           5           /* Y, U, V, LBSP (16 bits) */
#define MD_VAR1             6           /* a pixel: foreground frames */
#define MD_MAX_W            960
#define MD_MAX_H            1022
#define MD_MAX_MODELS       8

typedef struct {
    unsigned colour, lbsp, uv, min_t;   /* a region's thresholds */
    bool morph;
} MdThr;

static bool md_rd(hwaddr a, void *buf, size_t n)
{
    if (dma_memory_read(&address_space_memory, a, buf, n,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-md: cannot read 0x%" HWADDR_PRIx
                      "\n", a);
        return false;
    }
    return true;
}

static void md_wr(hwaddr a, const void *buf, size_t n)
{
    if (dma_memory_write(&address_space_memory, a, buf, n,
                         MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-md: cannot write 0x%"
                      HWADDR_PRIx "\n", a);
    }
}

/*
 * The 15-bit LFSR, x^15 + x^14 + 1, a step a draw, its state hashed: the
 * state itself only shifts along, so a draw that passed "% rate" would
 * leave the next one 0 or 1 modulo 8.
 */
static unsigned md_rand(uint32_t *lfsr)
{
    uint32_t bit = ((*lfsr >> 14) ^ (*lfsr >> 13)) & 1;

    *lfsr = ((*lfsr << 1) | bit) & 0x7FFF;
    return (*lfsr * 0x9E3779B1u) >> 17;
}

/*
 * The 16-point LBSP descriptor of (x, y): the 8 neighbours at distance 1
 * and the 8 at distance 2 on the 5x5 ring's corners and midpoints, a bit
 * set where one differs from the centre by more than t (LBSP_TH, or a
 * third of the centre's value when that is 0).  Clamped at the borders.
 */
static uint16_t md_lbsp(const uint8_t *y, int w, int h, int px, int py,
                        unsigned th)
{
    static const int8_t pts[16][2] = {
        { -1, -1 }, { 0, -1 }, { 1, -1 }, { -1, 0 }, { 1, 0 }, { -1, 1 },
        { 0, 1 }, { 1, 1 }, { -2, -2 }, { 0, -2 }, { 2, -2 }, { -2, 0 },
        { 2, 0 }, { -2, 2 }, { 0, 2 }, { 2, 2 },
    };
    int c = y[py * w + px], t = th ? th : c / 3;
    uint16_t d = 0;
    int i;

    for (i = 0; i < 16; i++) {
        int x = MIN(MAX(px + pts[i][0], 0), w - 1);
        int yy = MIN(MAX(py + pts[i][1], 0), h - 1);

        if (abs(y[yy * w + x] - c) > t) {
            d |= 1u << i;
        }
    }
    return d;
}

/*
 * The thresholds: thr[0] the global ones, thr[1 + i] ROI i's; and each
 * pixel's, the first enabled ROI holding it, else the global ones.
 */
static void md_regions(const uint32_t *regs, int w, int h, MdThr *thr,
                       uint8_t *map)
{
    uint32_t m0 = R(MD_MATCH0);
    int i, x, y;

    thr[0] = (MdThr) {
        .colour = extract32(m0, 8, 8), .lbsp = extract32(m0, 24, 4),
        .uv = extract32(R(MD_MATCH3), 0, 8),
        .min_t = extract32(R(MD_UPDATE0), 0, 8), .morph = true,
    };
    memset(map, 0, (size_t)w * h);
    for (i = MD_ROIS - 1; i >= 0; i--) {
        uint32_t r0 = R(MD_ROI(i)), r1 = R(MD_ROI(i) + 4);
        uint32_t r2 = R(MD_ROI(i) + 8), r3 = R(MD_ROI(i) + 12);
        int rx = extract32(r0, 0, 10), ry = extract32(r0, 12, 10);
        int rw = extract32(r1, 0, 10), rh = extract32(r1, 12, 10);

        thr[1 + i] = (MdThr) {
            .colour = extract32(r2, 8, 8), .lbsp = extract32(r2, 24, 4),
            .uv = extract32(r1, 24, 8), .min_t = extract32(r3, 4, 8),
            .morph = r3 & 1,
        };
        if (!(R(MD_MODE0) & (1u << (12 + i)))) {
            continue;
        }
        /* the lowest-numbered ROI is painted last, so it wins */
        for (y = ry; y < MIN(ry + rh, h); y++) {
            for (x = rx; x < MIN(rx + rw, w); x++) {
                map[y * w + x] = 1 + i;
            }
        }
    }
}

/* one 3x3 stage: add (or keep only) pixels with th or more neighbours */
static void md_morph(const MdThr *thr, const uint8_t *map, uint8_t *fg,
                     int w, int h, unsigned th, bool add)
{
    g_autofree uint8_t *in = g_memdup2(fg, (size_t)w * h);
    int x, y, dx, dy;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            unsigned n = 0;

            if (!thr[map[y * w + x]].morph) {
                continue;
            }
            for (dy = -1; dy <= 1; dy++) {
                for (dx = -1; dx <= 1; dx++) {
                    if ((dx || dy) && x + dx >= 0 && x + dx < w &&
                        y + dy >= 0 && y + dy < h) {
                        n += in[(y + dy) * w + x + dx];
                    }
                }
            }
            fg[y * w + x] = add ? in[y * w + x] | (n >= th)
                                : in[y * w + x] & (n >= th);
        }
    }
}

void nvt_md_run(uint32_t *regs)
{
    uint32_t mode = R(MD_MODE0), morph = R(MD_MORPH);
    uint32_t u3 = R(MD_UPDATE3), u4 = R(MD_UPDATE4);
    int w = R(MD_SIZE) & 0x3FE, h = (R(MD_SIZE) >> 12) & 0x3FE;
    unsigned models = extract32(R(MD_MATCH1), 0, 4);
    unsigned min_match = extract32(R(MD_MATCH2), 0, 4);
    unsigned lbsp_th = extract32(R(MD_MATCH0), 0, 8);
    unsigned max_fg = extract32(R(MD_UPDATE0), 16, 8);
    uint32_t lofs_y = R(MD_OFSI_Y) & 0xFFFFC, lofs_uv = R(MD_OFSI_UV) & 0xFFFFC;
    size_t n = (size_t)w * h, msz = n * models * MD_SAMPLE;
    g_autofree uint8_t *y = NULL, *uv = NULL, *model = NULL, *var1 = NULL;
    g_autofree uint8_t *fg = NULL, *bits = NULL, *map = NULL;
    g_autofree uint16_t *lbsp = NULL;
    MdThr thr[1 + MD_ROIS];
    uint32_t lfsr = u4 & (1u << 15) ? extract32(u4, 0, 15) :
                                      extract32(u4, 16, 15);
    uint64_t lum = 0;
    int x, yy, i;
    unsigned k;

    if (w < 2 || h < 2 || w > MD_MAX_W || h > MD_MAX_H || !models ||
        models > MD_MAX_MODELS || lofs_y < (uint32_t)w ||
        lofs_uv < (uint32_t)w) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-md: %dx%d, %u models, line "
                      "offsets %u / %u\n", w, h, models, lofs_y, lofs_uv);
        return;
    }
    if (!lfsr) {
        lfsr = 0x56AE;
    }
    y = g_malloc(n);
    uv = g_malloc(n / 2);
    for (yy = 0; yy < h; yy++) {
        if (!md_rd((R(MD_SAI_Y) & ~3u) + (hwaddr)yy * lofs_y, y + yy * w, w) ||
            (!(yy & 1) && !md_rd((R(MD_SAI_UV) & ~3u) +
                                 (hwaddr)(yy / 2) * lofs_uv,
                                 uv + (yy / 2) * w, w))) {
            return;
        }
    }
    lbsp = g_new(uint16_t, n);
    for (yy = 0; yy < h; yy++) {
        for (x = 0; x < w; x++) {
            lbsp[yy * w + x] = md_lbsp(y, w, h, x, yy, lbsp_th);
        }
    }
    map = g_malloc(n);
    md_regions(regs, w, h, thr, map);
    model = g_malloc(msz);
    var1 = g_malloc0(n * MD_VAR1);
    fg = g_malloc0(n);

    if (!(mode & MD_MODE_NORM)) {
        /* INIT: the samples from the frame, no motion */
        for (yy = 0; yy < h; yy++) {
            for (x = 0; x < w; x++) {
                for (k = 0; k < models; k++) {
                    uint8_t *s = model + ((size_t)(yy * w + x) * models + k) *
                                         MD_SAMPLE;

                    s[0] = y[yy * w + x];
                    s[1] = uv[(yy / 2) * w + (x & ~1)];
                    s[2] = uv[(yy / 2) * w + (x & ~1) + 1];
                    stw_le_p(s + 3, lbsp[yy * w + x]);
                }
            }
        }
    } else {
        g_autofree uint8_t *upd = g_malloc0(n);

        if (!md_rd(R(MD_SAI_MODEL) & ~3u, model, msz) ||
            !md_rd(R(MD_SAI_VAR1) & ~3u, var1, n * MD_VAR1)) {
            return;
        }
        /* match against the model as it came in */
        for (yy = 0; yy < h; yy++) {
            for (x = 0; x < w; x++) {
                size_t p = (size_t)yy * w + x;
                const MdThr t = thr[map[p]];
                int u = uv[(yy / 2) * w + (x & ~1)];
                int v = uv[(yy / 2) * w + (x & ~1) + 1];
                unsigned match = 0;
                uint8_t *fgc = var1 + p * MD_VAR1;

                for (k = 0; k < models; k++) {
                    const uint8_t *s = model + (p * models + k) * MD_SAMPLE;

                    if (!k) {
                        lum += abs(y[p] - s[0]);
                    }
                    if (abs(y[p] - s[0]) <= (int)t.colour &&
                        ctpop16(lbsp[p] ^ lduw_le_p(s + 3)) <= (int)t.lbsp &&
                        ((mode & MD_MODE_Y_ONLY) ||
                         abs(u - s[1]) + abs(v - s[2]) <= (int)t.uv)) {
                        match++;
                    }
                }
                fg[p] = match < min_match;
                if (mode & MD_MODE_DEGHOST) {
                    *fgc = fg[p] ? MIN(*fgc + 1, 255) : 0;
                }
                upd[p] = !fg[p] || ((mode & MD_MODE_DEGHOST) && max_fg &&
                                    *fgc > max_fg);
            }
        }
        /* then the updates, in raster order */
        for (yy = 0; yy < h; yy++) {
            for (x = 0; x < w; x++) {
                size_t p = (size_t)yy * w + x;
                unsigned rate = MAX(thr[map[p]].min_t, 1);
                uint8_t cur[MD_SAMPLE];

                if (!upd[p]) {
                    continue;
                }
                cur[0] = y[p];
                cur[1] = uv[(yy / 2) * w + (x & ~1)];
                cur[2] = uv[(yy / 2) * w + (x & ~1) + 1];
                stw_le_p(cur + 3, lbsp[p]);
                if (md_rand(&lfsr) % rate == 0) {
                    k = md_rand(&lfsr) % models;
                    memcpy(model + (p * models + k) * MD_SAMPLE, cur,
                           MD_SAMPLE);
                }
                if ((mode & MD_MODE_NEI) && md_rand(&lfsr) % rate == 0) {
                    static const int8_t nb[8][2] = {
                        { -1, -1 }, { 0, -1 }, { 1, -1 }, { -1, 0 },
                        { 1, 0 }, { -1, 1 }, { 0, 1 }, { 1, 1 },
                    };
                    unsigned d = md_rand(&lfsr) & 7;
                    int nx = MIN(MAX(x + nb[d][0], 0), w - 1);
                    int ny = MIN(MAX(yy + nb[d][1], 0), h - 1);

                    k = md_rand(&lfsr) % models;
                    memcpy(model + (((size_t)ny * w + nx) * models + k) *
                           MD_SAMPLE, cur, MD_SAMPLE);
                }
            }
        }
        for (i = 0; i < 4; i++) {
            unsigned th = extract32(morph, 4 * i, 4);

            if (th) {
                md_morph(thr, map, fg, w, h, th, morph & (1u << (20 + i)));
            }
        }
        if (extract32(morph, 16, 4)) {
            md_morph(thr, map, fg, w, h, extract32(morph, 16, 4), true);
        }
    }

    bits = g_malloc0((n + 7) / 8);
    for (k = 0; k < n; k++) {
        bits[k >> 3] |= fg[k] << (k & 7);
    }
    md_wr(R(MD_SAO_MODEL) & ~3u, model, msz);
    md_wr(R(MD_SAO_VAR1) & ~3u, var1, n * MD_VAR1);
    md_wr(R(MD_SAO_S1) & ~3u, bits, (n + 7) / 8);

    R(MD_MATCH5) = R(MD_MATCH4) & (1u << 28) ? extract32(R(MD_MATCH4), 0, 28)
                                              : MIN(lum, (1u << 28) - 1);
    R(MD_UPDATE3) = deposit32(u3, 12, 8, u3 & (1u << 8) ?
                              extract32(u3, 0, 8) : extract32(u3, 12, 8) + 1);
    R(MD_UPDATE4) = deposit32(u4, 16, 15, lfsr);
}
