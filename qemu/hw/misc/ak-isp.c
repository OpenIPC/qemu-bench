/*
 * Anyka ISP2 (0x20000000, interrupt 1) of the AK3918EV200 and of the
 * AnyCloud39EV300 parts (AK3916/AK3918/AK3919EV300): the frames, not the
 * image processing.
 *
 * The register contract is the ISP2 core the OpenIPC kernel builds from the
 * SDK sources (drivers/media/platform/soc_camera/plat-anyka/ak39_isp2.c and
 * ak39_isp2_reg.h), driven on EV200 by the ak-cam capture bridge
 * (drivers/media/platform/anyka/ak-cam.c) and on EV300 by the soc_camera
 * host ak_camera.c.  The two parts are one core with three differences
 * that matter here, which the driver keeps under CONFIG_CPU_AK3918EV200
 * (ev300=on selects the other side):
 *
 *                  EV200                       EV300
 *   buffer         GEN_CTRL 15..18 main,       interrupt-enable 31..28 main,
 *   enables        21..24 sub, counting up     27..24 sub, 23..20 channel 3,
 *                                              counting DOWN (bit 31 is
 *                                              buffer 1: low_4bits_revert())
 *   register       3424 bytes: block 4 at      3540 bytes: block 4 at 3188,
 *   table          3152, block 5 at 3292       block 5 at 3396 (EV300 has a
 *                                              third output channel, so
 *                                              blocks 1, 4, 5 are larger)
 *   channel        block 4 words 31 / 33,      block 4 words 48 / 50,
 *   sizes          12-bit width and height     13-bit width and height
 *
 * MMIO:
 *
 *   +0x00 GEN_CTRL   bit 20 ISP_ENABLE starts and stops capture; bits 10 / 11
 *                    MAIN_UV_MODE / SUB_UV_MODE (below); EV200: the buffer
 *                    enables
 *   +0x04 CSI_ERR_CHECK_1  EV200's sensor window, width 0..13, height 14..27
 *   +0x10 STATUS     bit 0 FRAME_ALL_DONE, 1 MAIN_FRAME_DONE, 2
 *                    SUB_FRAME_DONE (write 1 to clear); bits 22..23 the
 *                    buffer the main channel writes next, 20..21 the sub's
 *                    (0 = buffer 1 on both parts: the driver compares them
 *                    with its own buffer index); bit 14 ISP_UNDER_WORKING
 *   +0x14 interrupt enables, the STATUS bit numbers; EV300: the buffer
 *                    enables above them
 *   +0x18 DMA_REG_ADDR  the register table in DRAM (below)
 *   +0x1C REG_UPDATE    bits 0..2 (re)load the table; EV300 keeps MIPI
 *                    line-end settings in 24..31
 *
 * Most of the ISP's configuration is not in MMIO at all but in that register
 * table, which the core fetches by DMA when REG_UPDATE is written.  The model
 * fetches the whole table at every REG_UPDATE write, whatever the mode bit
 * (ALL / MOST / LEAST): what the core leaves out in the smaller modes is not
 * known, and the driver programs every word this model reads before a
 * MODE_ALL load.  A table written but never loaded is not seen.  The words
 * it reads are the ones that say where a frame comes from and goes to:
 *
 *   block 1 word 0 / 1    the crop (CUT) window on the sensor frame: x 0..12,
 *                         y 16..28; width 0..12, height 16..28
 *   block 5 words 25..30  buffer 1: main Y, U, V; sub Y, U, V
 *   block 1 words 31..48  buffers 2..4: main Y (31..33), U (34..36), V
 *                         (37..39); sub Y (40..42), U (43..45), V (46..48)
 *   block 4 (above)       main / sub output size
 *   block 4 word 34 / 51  frame control, a byte per channel (main, sub,
 *                         channel 3)
 *
 * Every frame period, while ISP_ENABLE is set, the core takes the crop window
 * out of the sensor's frame, scales it to each channel's size, writes the
 * main picture into the buffer it is on (if that buffer is enabled) and the
 * sub picture beside it (if its enable is set too), raises FRAME_ALL_DONE,
 * and moves on to the next enabled buffer, which it reports in STATUS.  That
 * is the order ak_isp_irq_work() expects: it takes the buffer it predicted
 * as the finished one and reads STATUS for the next.
 *
 * Chroma follows the channel's UV_MODE bit in GEN_CTRL.  Clear: I420, the
 * three planes at the Y, U and V addresses (what the EV200 driver programs).
 * Set: NV12, one plane of interleaved U, V at the U address, and the V
 * address is not used (what the EV300 driver programs, 0x7 << MAIN_UV_MODE,
 * "uv planar" in its comment).  The EV200 driver's own comment is the
 * evidence for which way round the bit goes: with it set, the encoder took
 * the frame's interleaved chroma as two planes.  The EV300 driver puts the U
 * address at Y + w * h, so a main frame is exactly w * h * 3 / 2 bytes of
 * NV12, as a PE3020-W captured it (OpenIPC/builder#234: 1920 x 1080 NV12,
 * 3,110,400 bytes).  Whether the core can also write V first (NV21) is not
 * known; CSI_ERR_CHECK_1's UV_SWITCH is not modelled.
 *
 * What the model does not forgive, because the driver gives silicon no
 * choice either:
 *  - the crop window.  The release EV300 host (ak_camera.c) sets it only
 *    from VIDIOC_S_CROP; nothing at open, S_FMT or STREAMON writes it, so
 *    unless the application crops, block 1 words 0 / 1 are the zeroes
 *    isp2_module_init() cleared the table to.  The PE3020-W gave a usable
 *    picture only after "explicit full-frame input crop" was added to its
 *    kernel.  What the core writes from an empty window is not known; the
 *    model writes nothing (the buffers keep whatever was in them) but the
 *    frame still completes, and logs a guest error.  A window that reaches
 *    past the sensor's frame reads black there; also a guess.
 *  - the sub channel.  It is written whenever its enable is set, at the
 *    address and size the table holds.  The EV300 host sets every sub
 *    enable with its main one, puts the sub picture right after the main
 *    one (+ w * h * 3 / 2, outside the V4L2 buffer it sized), and takes the
 *    sub size only from VIDIOC_S_PARM (raw_data {1, width, height}).
 *    Without that S_PARM the size is 0 and nothing is written.
 *  - the sensor.  On EV300 the frame is the sensor's output window, read
 *    from its registers (the tuning's register list writes it).  A sensor
 *    that has not been set up, or is held in reset, sends no frames, and the
 *    ISP raises no interrupts.  EV200 keeps its older fallbacks (the
 *    CSI window, then the crop window), because its SC1345 model reports
 *    none.
 *
 * Not modelled: the image processing itself, the statistics the 3A reads
 * (they stay zero), the 3D-NR reference reads and writes, OSD, channel 3
 * (EV300's driver turns it off with a frame-control byte of 0; anything else
 * is logged as unimplemented), the MIPI receiver (frames flow whatever the
 * D-PHY at 0x20400000 is set to), the scaler's limits, and frame-control
 * bytes other than 0 (channel off) and 0xFF (every frame), whose meaning is
 * not known: they are logged and treated as 0xFF.  ISP_UNDER_WORKING reads
 * 0, as frames are written at once.
 *
 * The picture is a test pattern on the sensor's frame -- colour bars, a
 * moving block, a frame counter strip -- so that an encoder downstream has
 * something to code, a decoded stream can be checked frame by frame, and a
 * wrong crop or scale shows.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/i2c/image-sensor.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/dma.h"
#include "system/address-spaces.h"

#define TYPE_AK_ISP "ak-isp"
OBJECT_DECLARE_SIMPLE_TYPE(AkIspState, AK_ISP)

#define AK_ISP_SIZE         0x20000
#define AK_ISP_NREGS        0x40

#define REG_GEN_CTRL        0x00
#define   GEN_MAIN_UV_MODE  (1u << 10)
#define   GEN_SUB_UV_MODE   (1u << 11)
#define   GEN_ISP_ENABLE    (1u << 20)
#define   GEN_MAIN_EN_SHIFT 15            /* EV200 */
#define   GEN_SUB_EN_SHIFT  21            /* EV200 */
#define REG_CSI_CHECK1      0x04
#define REG_STATUS          0x10
#define   ST_FRAME_ALL_DONE (1u << 0)
#define   ST_MAIN_DONE      (1u << 1)
#define   ST_SUB_DONE       (1u << 2)
#define   ST_SUB_ID_SHIFT   20
#define   ST_MAIN_ID_SHIFT  22
#define REG_INTEN           0x14
#define   INTEN_MAIN_SHIFT  28            /* EV300 */
#define   INTEN_SUB_SHIFT   24            /* EV300 */
#define REG_DMA_TABLE       0x18
#define REG_UPDATE          0x1C
#define   UPDATE_LOAD       0x7u
#define   UPDATE_MIPI       0xFF000000u   /* EV300 */

#define TBL_MAX             3540
#define MAX_W               4096
#define MAX_H               4096

/* isp2_module_init()'s layout of the register table, per part */
typedef struct AkIspLayout {
    uint32_t tbl_size;
    uint32_t blk4, blk5;        /* block 1 is at 0 on both */
    uint32_t main_dst, sub_dst; /* block 4 words */
    uint32_t dst_bits;
    uint32_t frame_ctrl;        /* block 4 word */
    uint32_t r5_enable;         /* block 5 word: [25] mirror, [26] flip */
    uint32_t st_w1c;            /* STATUS bits a 1 clears */
    uint32_t irq_mask;          /* interrupt-enable bits that are enables */
} AkIspLayout;

static const AkIspLayout ak_isp_ev200 = {
    .tbl_size = 3424, .blk4 = 3152, .blk5 = 3292,
    .main_dst = 31, .sub_dst = 33, .dst_bits = 12, .frame_ctrl = 34,
    .r5_enable = 32,
    .st_w1c = 0xFFFF, .irq_mask = 0xFFFF,
};

/* ISP_UNDER_WORKING (14) is a state, not an event */
static const AkIspLayout ak_isp_ev300 = {
    .tbl_size = 3540, .blk4 = 3188, .blk5 = 3396,
    .main_dst = 48, .sub_dst = 50, .dst_bits = 13, .frame_ctrl = 51,
    .r5_enable = 35,
    .st_w1c = 0x3FFFF & ~(1u << 14), .irq_mask = 0xFFFFF,
};

struct AkIspState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[AK_ISP_NREGS];
    uint8_t table[TBL_MAX];     /* as last loaded by REG_UPDATE */
    uint32_t cur;               /* the buffer the core writes next, 0..3 */
    uint32_t frames;

    const AkIspLayout *l;
    uint32_t fps;
    bool ev300;
    DeviceState *sensor;
    bool warned_crop, warned_sensor, warned_ctrl, warned_ch3;

    /* source=: a Y4M recording played in place of the test pattern */
    char *source;
    FILE *src_f;
    long src_first;             /* offset of the first FRAME */
    uint32_t src_w, src_h;
    uint8_t *src_pic;           /* the current frame, I420 */
    uint32_t src_index;         /* its number in the recording */
    bool src_ok;

    /* the frame being written: per sensor column, then one row */
    uint8_t bar_of[MAX_W];
    uint8_t ramp_of[MAX_W];
    uint8_t cell_on[MAX_W];
    uint16_t xmap[MAX_W];
    uint8_t src[3][MAX_W];
    uint8_t row[MAX_W];
};

static void ak_isp_update_irq(AkIspState *s)
{
    uint32_t pending = s->regs[REG_STATUS / 4] & s->regs[REG_INTEN / 4] &
                       s->l->irq_mask & s->l->st_w1c;

    qemu_set_irq(s->irq, pending != 0);
}

static uint32_t tbl_word(AkIspState *s, uint32_t byte_off)
{
    return ldl_le_p(s->table + byte_off);
}

static void ak_isp_load_table(AkIspState *s)
{
    hwaddr base = s->regs[REG_DMA_TABLE / 4];

    if (!base) {
        qemu_log_mask(LOG_GUEST_ERROR, "ak-isp: REG_UPDATE with no table "
                      "address\n");
        return;
    }
    dma_memory_read(&address_space_memory, base, s->table, s->l->tbl_size,
                    MEMTXATTRS_UNSPECIFIED);
}

/* EV300 numbers its buffer enables from the top: bit 3 is buffer 1 */
static uint32_t rev4(uint32_t v)
{
    return ((v & 1) << 3) | ((v & 2) << 1) | ((v & 4) >> 1) | ((v & 8) >> 3);
}

static uint32_t main_enables(AkIspState *s)
{
    if (s->ev300) {
        return rev4((s->regs[REG_INTEN / 4] >> INTEN_MAIN_SHIFT) & 0xF);
    }
    return (s->regs[REG_GEN_CTRL / 4] >> GEN_MAIN_EN_SHIFT) & 0xF;
}

static uint32_t sub_enables(AkIspState *s)
{
    if (s->ev300) {
        return rev4((s->regs[REG_INTEN / 4] >> INTEN_SUB_SHIFT) & 0xF);
    }
    return (s->regs[REG_GEN_CTRL / 4] >> GEN_SUB_EN_SHIFT) & 0xF;
}

typedef struct {
    uint32_t y, u, v;
} AkIspPlanes;

static void buffer_planes(AkIspState *s, unsigned buf, bool sub,
                          AkIspPlanes *p)
{
    if (buf == 0) {
        uint32_t w = s->l->blk5 + (sub ? 28 : 25) * 4;

        p->y = tbl_word(s, w);
        p->u = tbl_word(s, w + 4);
        p->v = tbl_word(s, w + 8);
    } else {
        unsigned k = buf - 1;

        p->y = tbl_word(s, ((sub ? 40 : 31) + k) * 4);
        p->u = tbl_word(s, ((sub ? 43 : 34) + k) * 4);
        p->v = tbl_word(s, ((sub ? 46 : 37) + k) * 4);
    }
}

static void channel_size(AkIspState *s, bool sub, uint32_t *w, uint32_t *h)
{
    uint32_t bits = s->l->dst_bits;
    uint32_t v = tbl_word(s, s->l->blk4 +
                             (sub ? s->l->sub_dst : s->l->main_dst) * 4);

    *w = v & ((1u << bits) - 1);
    *h = (v >> bits) & ((1u << bits) - 1);
    if (!s->ev300 && !sub && (!*w || !*h)) {
        /* EV200, no scaler set up: the sensor's window */
        v = s->regs[REG_CSI_CHECK1 / 4];
        *w = v & 0x3FFF;
        *h = (v >> 14) & 0x3FFF;
    }
}

/* The sensor's frame: false if no frame comes */
static bool sensor_frame(AkIspState *s, uint32_t *fw, uint32_t *fh)
{
    ImageSensorExposure e;

    if (image_sensor_exposure(OBJECT(s->sensor), &e) && e.width &&
        e.height) {
        *fw = e.width;
        *fh = e.height;
    } else if (s->ev300) {
        if (!s->warned_sensor) {
            qemu_log_mask(LOG_GUEST_ERROR, "ak-isp: capture is on and the "
                          "sensor sends no frames (not set up, or held in "
                          "reset)\n");
            s->warned_sensor = true;
        }
        return false;
    } else {
        uint32_t v = s->regs[REG_CSI_CHECK1 / 4];
        uint32_t cut0 = tbl_word(s, 0), cut1 = tbl_word(s, 4);

        *fw = v & 0x3FFF;
        *fh = (v >> 14) & 0x3FFF;
        if (!*fw || !*fh) {
            /* a frame that ends where the crop window does */
            *fw = (cut0 & 0x1FFF) + (cut1 & 0x1FFF);
            *fh = ((cut0 >> 16) & 0x1FFF) + ((cut1 >> 16) & 0x1FFF);
        }
    }
    *fw = MIN(*fw, MAX_W);
    *fh = MIN(*fh, MAX_H);
    return *fw && *fh;
}

/* BT.601 limited range, the order of SMPTE bars */
static const uint8_t bars[8][3] = {
    { 180, 128, 128 }, { 168, 44, 136 }, { 145, 147, 44 }, { 133, 63, 52 },
    { 63, 193, 204 },  { 51, 109, 212 }, { 28, 212, 120 }, { 16, 128, 128 },
};

/*
 * Bars over the top two thirds of the sensor frame, a ramp below; a block
 * that crosses it once every 64 frames; and the frame counter as sixteen
 * black or white cells along the bottom, so a decoded frame names itself.
 */
/*
 * source=FILE: a YUV4MPEG2 recording, 4:2:0 (the C420* tags, or none), read
 * one frame per ISP frame and looped, in place of the test pattern -- so
 * the encoder downstream codes a real camera's scene, its noise and its
 * motion, and the stream's sizes and rate control can be compared with the
 * camera it was recorded on. `ffmpeg -i rec.mp4 -pix_fmt yuv420p rec.y4m`
 * makes one.
 */
static bool source_open(AkIspState *s, Error **errp)
{
    char hdr[256], *tok;

    s->src_f = fopen(s->source, "rb");
    if (!s->src_f) {
        error_setg_errno(errp, errno, "ak-isp: source %s", s->source);
        return false;
    }
    if (!fgets(hdr, sizeof(hdr), s->src_f) ||
        strncmp(hdr, "YUV4MPEG2 ", 10)) {
        error_setg(errp, "ak-isp: source %s is not YUV4MPEG2", s->source);
        return false;
    }
    for (tok = strtok(hdr + 10, " \n"); tok; tok = strtok(NULL, " \n")) {
        if (tok[0] == 'W') {
            s->src_w = atoi(tok + 1);
        } else if (tok[0] == 'H') {
            s->src_h = atoi(tok + 1);
        } else if (tok[0] == 'C' && strncmp(tok, "C420", 4)) {
            error_setg(errp, "ak-isp: source %s is %s, not 4:2:0",
                       s->source, tok);
            return false;
        }
    }
    if (!s->src_w || !s->src_h || s->src_w > 4096 || s->src_h > 4096 ||
        (s->src_w | s->src_h) & 1) {
        error_setg(errp, "ak-isp: source %s is %ux%u", s->source,
                   s->src_w, s->src_h);
        return false;
    }
    s->src_first = ftell(s->src_f);
    s->src_pic = g_malloc((size_t)s->src_w * s->src_h * 3 / 2);
    return true;
}

/* The next frame of the recording, from the start again after the last */
static void source_next(AkIspState *s)
{
    size_t n = (size_t)s->src_w * s->src_h * 3 / 2;
    char line[128];

    for (int tries = 0; tries < 2; tries++) {
        if (fgets(line, sizeof(line), s->src_f) &&
            !strncmp(line, "FRAME", 5) &&
            fread(s->src_pic, 1, n, s->src_f) == n) {
            s->src_ok = true;
            s->src_index++;
            return;
        }
        fseek(s->src_f, s->src_first, SEEK_SET);
        s->src_index = 0;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "ak-isp: source %s has no frames\n",
                  s->source);
    s->src_ok = false;
}

static void pattern_prepare(AkIspState *s, uint32_t fw)
{
    if (s->src_f) {
        source_next(s);
    }
    for (uint32_t x = 0; x < fw; x++) {
        s->bar_of[x] = x * 8 / fw;
        s->ramp_of[x] = 16 + x * 219 / fw;
        s->cell_on[x] = (s->frames >> (15 - x * 16 / fw)) & 1;
    }
}

/* Sensor row @sy, columns xmap[0..n), into src[] */
static void pattern_row(AkIspState *s, uint32_t sy, uint32_t fw, uint32_t fh,
                        uint32_t n, unsigned step)
{
    uint32_t bx = (s->frames % 64) * (fw - fw / 8) / 63;
    uint32_t by = fh / 3;
    bool block_row = sy >= by && sy < by + fh / 6;

    for (uint32_t i = 0; i < n; i++) {
        uint32_t x = s->xmap[i * step];
        uint8_t yuv[3];

        if (x >= fw || sy >= fh) {
            /* the crop window reaches past the frame: black (a guess) */
            yuv[0] = 16;
            yuv[1] = yuv[2] = 128;
        } else if (s->src_ok) {
            /* the recording, stretched over the sensor frame */
            uint32_t px = (uint64_t)x * s->src_w / fw;
            uint32_t py = (uint64_t)sy * s->src_h / fh;
            const uint8_t *u = s->src_pic + s->src_w * s->src_h;
            const uint8_t *v = u + (s->src_w / 2) * (s->src_h / 2);
            uint32_t c = (py / 2) * (s->src_w / 2) + px / 2;

            yuv[0] = s->src_pic[py * s->src_w + px];
            yuv[1] = u[c];
            yuv[2] = v[c];
        } else if (sy >= fh - fh / 16) {
            yuv[0] = s->cell_on[x] ? 235 : 16;
            yuv[1] = yuv[2] = 128;
        } else if (block_row && x >= bx && x < bx + fw / 8) {
            yuv[0] = 235;
            yuv[1] = 90;
            yuv[2] = 240;
        } else if (sy < fh * 2 / 3) {
            memcpy(yuv, bars[s->bar_of[x]], 3);
        } else {
            yuv[0] = s->ramp_of[x];
            yuv[1] = yuv[2] = 128;
        }
        s->src[0][i] = yuv[0];
        s->src[1][i] = yuv[1];
        s->src[2][i] = yuv[2];
    }
}

static void put_row(hwaddr a, const uint8_t *data, uint32_t n)
{
    dma_memory_write(&address_space_memory, a, data, n,
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * One channel's picture: the crop window of the sensor frame, scaled to
 * w x h (nearest neighbour), into the planes @p.
 */
static void write_picture(AkIspState *s, const AkIspPlanes *p, bool nv12,
                          uint32_t w, uint32_t h, uint32_t fw, uint32_t fh)
{
    uint32_t cut0 = tbl_word(s, 0), cut1 = tbl_word(s, 4);
    uint32_t cx = cut0 & 0x1FFF, cy = (cut0 >> 16) & 0x1FFF;
    uint32_t cw = cut1 & 0x1FFF, ch = (cut1 >> 16) & 0x1FFF;
    /*
     * HFILP_EN / VFILP_EN (ak_isp_set_flip_mirror): the picture is
     * mirrored and / or flipped. Applied here to the crop window as it is
     * written out, which for the full-frame window every driver programs
     * is the same as mirroring the sensor frame; whether silicon mirrors
     * before or after a narrower crop is not known.
     */
    uint32_t r5 = tbl_word(s, s->l->blk5 + s->l->r5_enable * 4);
    bool hflip = r5 & (1u << 25), vflip = r5 & (1u << 26);

    if (!p->y) {
        return;
    }
    for (uint32_t x = 0; x < w; x++) {
        s->xmap[x] = cx + (uint64_t)(hflip ? w - 1 - x : x) * cw / w;
    }
    for (uint32_t y = 0; y < h; y++) {
        pattern_row(s, cy + (uint64_t)(vflip ? h - 1 - y : y) * ch / h,
                    fw, fh, w, 1);
        put_row(p->y + (hwaddr)y * w, s->src[0], w);
    }
    if (!p->u || (!nv12 && !p->v)) {
        return;
    }
    for (uint32_t y = 0; y < h / 2; y++) {
        uint32_t ry = vflip ? h / 2 - 1 - y : y;

        pattern_row(s, cy + (uint64_t)ry * 2 * ch / h, fw, fh, w / 2, 2);
        if (nv12) {
            for (uint32_t x = 0; x < w / 2; x++) {
                s->row[2 * x] = s->src[1][x];
                s->row[2 * x + 1] = s->src[2][x];
            }
            put_row(p->u + (hwaddr)y * w, s->row, w / 2 * 2);
        } else {
            put_row(p->u + (hwaddr)y * (w / 2), s->src[1], w / 2);
            put_row(p->v + (hwaddr)y * (w / 2), s->src[2], w / 2);
        }
    }
}

/* Frame control byte @ch: whether that channel writes this frame */
static bool channel_on(AkIspState *s, unsigned ch)
{
    uint32_t b = (tbl_word(s, s->l->blk4 + s->l->frame_ctrl * 4) >> (8 * ch))
                 & 0xFF;

    if (b != 0 && b != 0xFF && !s->warned_ctrl) {
        qemu_log_mask(LOG_UNIMP, "ak-isp: frame control 0x%02x for channel "
                      "%u (only 0 and 0xFF are modelled)\n", b, ch + 1);
        s->warned_ctrl = true;
    }
    return b != 0;
}

/* Write channel @sub of buffer @buf; false if nothing was written */
static bool write_channel(AkIspState *s, unsigned buf, bool sub,
                          uint32_t fw, uint32_t fh)
{
    uint32_t cut1 = tbl_word(s, 4);
    uint32_t w, h;
    AkIspPlanes p;

    if (!channel_on(s, sub)) {
        return false;
    }
    channel_size(s, sub, &w, &h);
    if (!w || !h) {
        return false;
    }
    if (w > MAX_W || h > MAX_H) {
        qemu_log_mask(LOG_GUEST_ERROR, "ak-isp: %s channel %ux%u is "
                      "larger than modelled\n", sub ? "sub" : "main", w, h);
        return false;
    }
    if (!(cut1 & 0x1FFF) || !((cut1 >> 16) & 0x1FFF)) {
        if (!s->warned_crop) {
            qemu_log_mask(LOG_GUEST_ERROR, "ak-isp: the crop window is "
                          "empty (block 1 word 1 = 0x%08x): nothing is "
                          "written\n", cut1);
            s->warned_crop = true;
        }
        return false;
    }
    buffer_planes(s, buf, sub, &p);
    write_picture(s, &p, s->regs[REG_GEN_CTRL / 4] &
                         (sub ? GEN_SUB_UV_MODE : GEN_MAIN_UV_MODE),
                  w, h, fw, fh);
    return true;
}

static int next_enabled(uint32_t en, uint32_t from)
{
    for (int i = 1; i <= 4; i++) {
        uint32_t b = (from + i) % 4;

        if (en & (1u << b)) {
            return b;
        }
    }
    return -1;
}

static int64_t frame_ns(AkIspState *s)
{
    return NANOSECONDS_PER_SECOND / (s->fps ? s->fps : 15);
}

static void ak_isp_frame(void *opaque)
{
    AkIspState *s = opaque;
    uint32_t main_en = main_enables(s);
    uint32_t status = 0;
    uint32_t fw, fh;

    if (!(s->regs[REG_GEN_CTRL / 4] & GEN_ISP_ENABLE)) {
        return;
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                        frame_ns(s));
    if (!sensor_frame(s, &fw, &fh)) {
        return;
    }

    if (!(main_en & (1u << s->cur))) {
        /* the buffer it is on was taken away: move on, write nothing */
        int n = next_enabled(main_en, s->cur);

        if (n < 0) {
            return;
        }
        s->cur = n;
    }

    pattern_prepare(s, fw);
    if (write_channel(s, s->cur, false, fw, fh)) {
        status |= ST_MAIN_DONE;
    }
    if ((sub_enables(s) & (1u << s->cur)) &&
        write_channel(s, s->cur, true, fw, fh)) {
        status |= ST_SUB_DONE;
    }
    if (s->ev300 && !s->warned_ch3 && channel_on(s, 2)) {
        qemu_log_mask(LOG_UNIMP, "ak-isp: channel 3 is not modelled\n");
        s->warned_ch3 = true;
    }
    s->frames++;

    int n = next_enabled(main_en, s->cur);
    if (n >= 0) {
        s->cur = n;
    }
    s->regs[REG_STATUS / 4] &= ~((3u << ST_MAIN_ID_SHIFT) |
                                 (3u << ST_SUB_ID_SHIFT));
    s->regs[REG_STATUS / 4] |= status | ST_FRAME_ALL_DONE |
                               (s->cur << ST_MAIN_ID_SHIFT) |
                               (s->cur << ST_SUB_ID_SHIFT);
    ak_isp_update_irq(s);
}

static uint64_t ak_isp_read(void *opaque, hwaddr off, unsigned size)
{
    AkIspState *s = opaque;

    if (off / 4 < AK_ISP_NREGS) {
        return s->regs[off / 4];
    }
    return 0;
}

static void ak_isp_start(AkIspState *s)
{
    /* ak_isp_vi_start_capturing(): the first enabled buffer */
    int n = next_enabled(main_enables(s), 3);

    s->cur = n < 0 ? 0 : n;
    s->warned_crop = s->warned_sensor = false;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                        frame_ns(s));
}

static void ak_isp_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    AkIspState *s = opaque;
    uint32_t old;

    if (off / 4 >= AK_ISP_NREGS) {
        return;
    }
    old = s->regs[off / 4];

    switch (off) {
    case REG_STATUS:
        s->regs[REG_STATUS / 4] &= ~(val & s->l->st_w1c);
        ak_isp_update_irq(s);
        return;
    case REG_INTEN:
        s->regs[off / 4] = val;
        ak_isp_update_irq(s);
        return;
    case REG_GEN_CTRL:
        s->regs[off / 4] = val;
        if ((val & GEN_ISP_ENABLE) && !(old & GEN_ISP_ENABLE)) {
            ak_isp_start(s);
        } else if (!(val & GEN_ISP_ENABLE)) {
            timer_del(s->timer);
        }
        return;
    case REG_UPDATE:
        if (val & UPDATE_LOAD) {
            ak_isp_load_table(s);
        }
        /* the load bits clear themselves; EV300's MIPI fields stay */
        s->regs[off / 4] = s->ev300 ? val & UPDATE_MIPI : 0;
        return;
    default:
        s->regs[off / 4] = val;
        return;
    }
}

static const MemoryRegionOps ak_isp_ops = {
    .read = ak_isp_read,
    .write = ak_isp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void ak_isp_reset(DeviceState *dev)
{
    AkIspState *s = AK_ISP(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->table, 0, sizeof(s->table));
    s->cur = 0;
    s->frames = 0;
    qemu_set_irq(s->irq, 0);
}

static void ak_isp_realize(DeviceState *dev, Error **errp)
{
    AkIspState *s = AK_ISP(dev);

    s->l = s->ev300 ? &ak_isp_ev300 : &ak_isp_ev200;
    memory_region_init_io(&s->iomem, OBJECT(s), &ak_isp_ops, s, TYPE_AK_ISP,
                          AK_ISP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ak_isp_frame, s);
    if (s->source && *s->source && !source_open(s, errp)) {
        if (s->src_f) {
            fclose(s->src_f);
            s->src_f = NULL;
        }
        return;
    }
}

static void ak_isp_unrealize(DeviceState *dev)
{
    AkIspState *s = AK_ISP(dev);

    timer_free(s->timer);
    if (s->src_f) {
        fclose(s->src_f);
    }
    g_free(s->src_pic);
}

/* A migrated source= picks the recording up where the source VM was */
static int ak_isp_post_load(void *opaque, int version_id)
{
    AkIspState *s = opaque;
    uint32_t want = s->src_index;

    if (!s->src_f) {
        return 0;
    }
    fseek(s->src_f, s->src_first, SEEK_SET);
    s->src_index = 0;
    s->src_ok = false;
    while (s->src_index < want) {
        uint32_t before = s->src_index;

        source_next(s);
        if (!s->src_ok || s->src_index <= before) {
            break;      /* a shorter recording here: it went round */
        }
    }
    return 0;
}

static const VMStateDescription ak_isp_vmstate = {
    .name = TYPE_AK_ISP,
    .version_id = 3,
    .minimum_version_id = 2,
    .post_load = ak_isp_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AkIspState, AK_ISP_NREGS),
        VMSTATE_UINT8_ARRAY(table, AkIspState, TBL_MAX),
        VMSTATE_UINT32(cur, AkIspState),
        VMSTATE_UINT32(frames, AkIspState),
        VMSTATE_TIMER_PTR(timer, AkIspState),
        VMSTATE_UINT32_V(src_index, AkIspState, 3),
        VMSTATE_END_OF_LIST()
    },
};

static const Property ak_isp_props[] = {
    DEFINE_PROP_UINT32("fps", AkIspState, fps, 15),
    DEFINE_PROP_BOOL("ev300", AkIspState, ev300, false),
    DEFINE_PROP_STRING("source", AkIspState, source),
    DEFINE_PROP_LINK("sensor", AkIspState, sensor, TYPE_DEVICE,
                     DeviceState *),
};

static void ak_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = ak_isp_realize;
    dc->unrealize = ak_isp_unrealize;
    dc->vmsd = &ak_isp_vmstate;
    device_class_set_legacy_reset(dc, ak_isp_reset);
    device_class_set_props(dc, ak_isp_props);
}

static const TypeInfo ak_isp_info = {
    .name          = TYPE_AK_ISP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkIspState),
    .class_init    = ak_isp_class_init,
};

static void ak_isp_register_types(void)
{
    type_register_static(&ak_isp_info);
}

type_init(ak_isp_register_types)
