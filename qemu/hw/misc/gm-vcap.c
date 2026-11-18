/*
 * Grain Media VCAP300 video capture.
 *
 * The capture engine at 0x96100000 on the GM813x (module/vcap300:
 * vcap_reg.h, vcap_lib.c, vcap_lli.c, vcap_lli.h, vcap_dev.h), which takes
 * the ISP's output -- the cascade input, VI 1, hardware channel 32 -- and
 * writes packed 4:2:2 frames for the video graph.  The window holds the
 * per-channel parameter memory (channel n at n * 0x1D8, the cascade
 * channel at 0x3E00), VI and source crop registers at 0x5000 / 0x5100,
 * globals at 0x5400, status at 0x5500 and the link-list SRAM at 0x40000.
 *
 * The driver runs the engine in link-list mode.  Each path p of a channel
 * has a "next" table pointer in the channel block (+0x1C8 + 4 * (p / 2),
 * 16 bits per path, an offset into the SRAM, bit 0 stop).  At every frame
 * the engine runs the table there: 64-bit commands, low word data, high
 * word desc[31:29] | channel[28:23] | path[22:21] | ...
 *
 *   0 NULL      write the path's control byte to 0: the path idles and
 *               the pointer stays, so a job queued later is found there
 *   1 STATUS    the slot the result goes to
 *   2 UPDATE    byte enables 22:19, register 18:0
 *   4 NEXT_LLI  end of the frame's table; the pointer moves to bits 18:0
 *   5 NEXT_CMD  continue at bits 18:0
 *
 * A table that reaches NEXT_LLI captures a frame: the target crop of the
 * path (+0x160 + 8p: x start | x end << 16, +4: y) at its DMA address
 * (+0x198 + 8p) with the line gap (+0x19C + 8p, bits 12:0, 4-pixel units)
 * -- unless the path control byte's bit 7 blocks the DMA -- then writes
 * the status slot (ready, frame mode, the frame count; DMA write done) and
 * sets the channel's LL done bit (0x55B0, the cascade's 0x55B4 bit 0).
 * The picture is the Allwinner capture model's: the OpenIPC logo on navy
 * with a sliding dot, drawn to the scaler output size (+0x10 + 4p) so
 * every path shows the same scene.
 *
 * The driver links queued jobs into the lists only from the sync timer
 * interrupt (0x55AC bit 0), so while capture is enabled (0x5404 bit 31)
 * that ticks at 60 Hz and frames come every second tick.  Status
 * registers clear by writing them back, their masks sit 0x80 below, and
 * the line is interrupt 32.  0x55E8 / 0x55EC identify the GM8136's
 * VCAP300: version 0x20140311, and the capability word with four scalers
 * that can scale -- gmlib refuses a scaled sub-stream otherwise.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"
#include "aw-vinc-logo.h"

#define TYPE_GM_VCAP "gm-vcap"
OBJECT_DECLARE_SIMPLE_TYPE(GmVcapState, GM_VCAP)

#define GM_VCAP_SIZE        0x48000
#define GM_VCAP_NREGS       (GM_VCAP_SIZE / 4)
#define GM_VCAP_LLI         0x40000
#define GM_VCAP_CH_SIZE     0x1D8
#define GM_VCAP_CAS_BASE    0x3E00
#define GM_VCAP_CHANNELS    8
#define GM_VCAP_PATHS       4
#define GM_VCAP_TICK_NS     (NANOSECONDS_PER_SECOND / 60)
#define GM_VCAP_MAX_CMDS    512

#define R_CTRL0             0x5400
#define R_CTRL1             0x5404
#define R_CH_EN             0x5408
#define R_CAS_EN            0x540C
#define R_STS_ERR           0x55A8
#define R_STS_TICK          0x55AC
#define R_STS_LL            0x55B0
#define R_STS_LL_CAS        0x55B4
#define R_VERSION           0x55E8
#define R_CAPS              0x55EC

#define CTRL0_SELF_CLEAR    ((1u << 0) | (1u << 4))
#define CTRL0_DMA_OFF       (1u << 31)
#define CTRL1_RESET         (1u << 0)
#define CTRL1_FIRE          (1u << 28)
#define CTRL1_CAP_EN        (1u << 31)

#define STS_DMA_DISABLED    ((1u << 2) | (1u << 10))

/* status registers that clear by writing back, with masks 0x80 below */
static const uint32_t gm_vcap_status_regs[] = {
    0x5568, R_STS_ERR, R_STS_TICK, R_STS_LL, R_STS_LL_CAS, 0x55DC,
};

struct GmVcapState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[GM_VCAP_NREGS];
    uint32_t ticks;
    uint32_t frames;
    uint32_t version;
    uint32_t caps;
};

static void gm_vcap_update(GmVcapState *s)
{
    bool level = false;
    int i;

    for (i = 0; i < ARRAY_SIZE(gm_vcap_status_regs); i++) {
        const uint32_t r = gm_vcap_status_regs[i];

        if (r == 0x55DC) {
            continue;                   /* an ID, not an event */
        }
        level |= (s->regs[r / 4] & ~s->regs[(r - 0x80) / 4]) != 0;
    }
    qemu_set_irq(s->irq, level);
}

/* ---- the test picture ---------------------------------------------------- */

/*
 * The same scene as the Allwinner capture model (hw/misc/aw-vinc.c): the
 * OpenIPC logo, white on a navy gradient, with a round dot that slides to
 * and fro beneath it.  It is drawn in the sensor's 1920x1080 and scaled to
 * each path's scaler output, and the dot's place comes from the frame
 * count, so the main and sub streams show the same scene.  The logo is the
 * run-length bitmap of aw-vinc-logo.h, averaged into its coverage once.
 */
#define PAT_W               1920
#define PAT_H               1080
#define PAT_LOGO_X          ((PAT_W - AW_VINC_LOGO_W) / 2)
#define PAT_LOGO_Y          (PAT_H * 2 / 5 - AW_VINC_LOGO_H / 2)
#define PAT_DOT_R           64
#define PAT_DOT_Y           (PAT_H * 4 / 5)
#define PAT_DOT_X0          (PAT_LOGO_X + PAT_DOT_R)
#define PAT_DOT_RANGE       (AW_VINC_LOGO_W - 2 * PAT_DOT_R)

static uint8_t *gm_vcap_logo;           /* coverage 0..255 */

static void gm_vcap_logo_init(void)
{
    const unsigned sc = AW_VINC_LOGO_SCALE;
    const unsigned bw = AW_VINC_LOGO_W * sc;
    gsize n, i = 0;
    g_autofree guchar *runs = g_base64_decode(aw_vinc_logo_runs_b64, &n);
    g_autofree uint8_t *count = g_malloc0(AW_VINC_LOGO_W * AW_VINC_LOGO_H);
    uint64_t pos = 0, end = (uint64_t)bw * AW_VINC_LOGO_H * sc;
    bool set = false;

    while (i < n && pos < end) {
        uint64_t run = 0;
        unsigned shift = 0;

        do {
            run |= (uint64_t)(runs[i] & 0x7F) << shift;
            shift += 7;
        } while ((runs[i++] & 0x80) && i < n && shift < 35);
        for (; set && run && pos < end; run--, pos++) {
            count[(pos / bw / sc) * AW_VINC_LOGO_W + (pos % bw) / sc]++;
        }
        pos += run;
        set = !set;
    }
    gm_vcap_logo = g_malloc(AW_VINC_LOGO_W * AW_VINC_LOGO_H);
    for (i = 0; i < AW_VINC_LOGO_W * AW_VINC_LOGO_H; i++) {
        gm_vcap_logo[i] = count[i] * 255 / (sc * sc);
    }
}

static uint8_t gm_vcap_blend(uint8_t bg, uint8_t fg, unsigned a)
{
    return bg + ((int)fg - bg) * (int)a / 255;
}

/* Y, Cb, Cr at sensor pixel (px, py) with the dot centred at dot_x */
static void gm_vcap_pixel(int px, int py, int dot_x, uint8_t *yuv)
{
    const int dx = px - dot_x, dy = py - PAT_DOT_Y;
    unsigned a = 0;

    yuv[0] = 50 - 14 * py / PAT_H;      /* navy, lighter at the top */
    yuv[1] = 146;
    yuv[2] = 122;
    if (px >= PAT_LOGO_X && px < PAT_LOGO_X + AW_VINC_LOGO_W &&
        py >= PAT_LOGO_Y && py < PAT_LOGO_Y + AW_VINC_LOGO_H) {
        a = gm_vcap_logo[(py - PAT_LOGO_Y) * AW_VINC_LOGO_W +
                         px - PAT_LOGO_X];
    } else if (abs(dx) <= PAT_DOT_R + 1 && abs(dy) <= PAT_DOT_R + 1) {
        /* the dot, its edge anti-aliased over a pixel */
        const int d = (int)(sqrt(dx * dx + dy * dy) * 256);

        a = d <= PAT_DOT_R * 256 - 128 ? 255 :
            d >= PAT_DOT_R * 256 + 128 ? 0 :
            (PAT_DOT_R * 256 + 128 - d) * 255 / 256;
    }
    if (a) {
        yuv[0] = gm_vcap_blend(yuv[0], 235, a);
        yuv[1] = gm_vcap_blend(yuv[1], 128, a);
        yuv[2] = gm_vcap_blend(yuv[2], 128, a);
    }
}

static void gm_vcap_paint(GmVcapState *s, uint32_t base, int p)
{
    const uint32_t ctl = extract32(s->regs[(base + 0x00) / 4], p * 8, 8);
    const uint32_t tx = s->regs[(base + 0x160 + 8 * p) / 4];
    const uint32_t ty = s->regs[(base + 0x164 + 8 * p) / 4];
    const uint32_t sd = s->regs[(base + 0x10 + 4 * p) / 4];
    const hwaddr dst = s->regs[(base + 0x198 + 8 * p) / 4] & ~7u;
    const uint32_t gap = extract32(s->regs[(base + 0x19C + 8 * p) / 4], 0, 13);
    const int x0 = extract32(tx, 0, 13), x1 = extract32(tx, 16, 13);
    const int y0 = extract32(ty, 0, 13), y1 = extract32(ty, 16, 13);
    const int w = (x1 - x0 + 1) & ~1, h = y1 - y0 + 1;
    const uint32_t t = (s->frames * 8) % (2 * PAT_DOT_RANGE);
    const int dot_x = PAT_DOT_X0 + (t < PAT_DOT_RANGE ? t
                                                      : 2 * PAT_DOT_RANGE - t);
    int sw = extract32(sd, 0, 13), sh = extract32(sd, 16, 13);
    g_autofree uint8_t *line = NULL;
    int x, y;

    if ((ctl & 0x80) || !dst || w <= 0 || h <= 0 || w > 8192 || h > 8192) {
        return;
    }
    if (!gm_vcap_logo) {
        gm_vcap_logo_init();
    }
    if (sw < x1 + 1 || sh < y1 + 1) {
        sw = x1 + 1;
        sh = y1 + 1;
    }
    line = g_malloc(w * 2);
    for (y = 0; y < h; y++) {
        const int py = (int64_t)(y0 + y) * PAT_H / sh;

        for (x = 0; x < w; x += 2) {
            uint8_t a[3], b[3];

            gm_vcap_pixel((int64_t)(x0 + x) * PAT_W / sw, py, dot_x, a);
            gm_vcap_pixel((int64_t)(x0 + x + 1) * PAT_W / sw, py, dot_x, b);
            line[x * 2 + 0] = (a[1] + b[1]) / 2;       /* UYVY */
            line[x * 2 + 1] = a[0];
            line[x * 2 + 2] = (a[2] + b[2]) / 2;
            line[x * 2 + 3] = b[0];
        }
        dma_memory_write(&address_space_memory,
                         dst + (hwaddr)y * (w * 2 + gap * 8), line, w * 2,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

/* ---- the link lists ------------------------------------------------------ */

static void gm_vcap_update_reg(GmVcapState *s, uint32_t lo, uint32_t hi)
{
    const uint32_t reg = hi & 0x7FFFC;
    const uint32_t be = extract32(hi, 19, 4);
    uint32_t v;
    int b;

    if (reg >= GM_VCAP_SIZE) {
        return;
    }
    v = s->regs[reg / 4];
    for (b = 0; b < 4; b++) {
        if (be & (1u << b)) {
            v = deposit32(v, b * 8, 8, extract32(lo, b * 8, 8));
        }
    }
    s->regs[reg / 4] = v;
}

/* Run path p's table; returns true when a frame's table completed. */
static bool gm_vcap_path(GmVcapState *s, uint32_t base, int p)
{
    const uint32_t nreg = base + 0x1C8 + 4 * (p / 2);
    const int half = (p % 2) * 16;
    const uint32_t next = extract32(s->regs[nreg / 4], half, 16);
    uint32_t pc = GM_VCAP_LLI + (next & 0xFFFE), slot = 0;
    int n;

    if (next & 1) {                             /* stopped */
        return false;
    }
    for (n = 0; n < GM_VCAP_MAX_CMDS && pc + 8 <= GM_VCAP_SIZE; n++) {
        const uint32_t lo = s->regs[pc / 4], hi = s->regs[pc / 4 + 1];

        switch (hi >> 29) {
        case 0:                                 /* NULL: idle */
            gm_vcap_update_reg(s, lo, hi);
            return false;
        case 1:
            slot = pc;
            pc += 8;
            break;
        case 2:
            gm_vcap_update_reg(s, lo, hi);
            pc += 8;
            break;
        case 5:
            pc = hi & 0x7FFF8;
            break;
        case 4: {
            const uint32_t to = (hi & 0x7FFFF) - GM_VCAP_LLI;
            const uint32_t creg = base + 0x1C0 + 4 * (p / 2);

            s->regs[creg / 4] = deposit32(s->regs[creg / 4], half, 16, next);
            s->regs[nreg / 4] = deposit32(s->regs[nreg / 4], half, 16,
                                          to & 0xFFFE);
            gm_vcap_paint(s, base, p);
            if (slot) {
                uint32_t w1 = s->regs[slot / 4 + 1];

                s->regs[slot / 4] = 1 | 2 | ((s->frames & 0xFFFF) << 16);
                w1 &= ~((1u << 3) | (1u << 7) | (0x3Fu << 12));
                s->regs[slot / 4 + 1] = w1 | (1u << 1);
            }
            return true;
        }
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "gm-vcap: bad command 0x%08x at 0x%x\n", hi, pc);
            return false;
        }
    }
    return false;
}

static void gm_vcap_channel(GmVcapState *s, uint32_t base, uint32_t done_reg,
                            uint32_t done_bit)
{
    bool done = false;
    int p;

    for (p = 0; p < GM_VCAP_PATHS; p++) {
        done |= gm_vcap_path(s, base, p);
    }
    if (done) {
        s->regs[done_reg / 4] |= done_bit;
    }
}

static void gm_vcap_tick(void *opaque)
{
    GmVcapState *s = opaque;
    int ch;

    if (!(s->regs[R_CTRL1 / 4] & CTRL1_CAP_EN)) {
        return;
    }
    s->ticks++;
    if (!(s->ticks & 1)) {                      /* 30 frames a second */
        s->frames++;
        if (s->regs[R_CAS_EN / 4] & 1) {
            gm_vcap_channel(s, GM_VCAP_CAS_BASE, R_STS_LL_CAS, 1);
        }
        for (ch = 0; ch < GM_VCAP_CHANNELS; ch++) {
            if (s->regs[R_CH_EN / 4] & (1u << ch)) {
                gm_vcap_channel(s, ch * GM_VCAP_CH_SIZE, R_STS_LL, 1u << ch);
            }
        }
    }
    s->regs[R_STS_TICK / 4] |= 1;
    gm_vcap_update(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                        GM_VCAP_TICK_NS);
}

static void gm_vcap_arm(GmVcapState *s)
{
    if (s->regs[R_CTRL1 / 4] & CTRL1_CAP_EN) {
        if (!timer_pending(s->timer)) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                GM_VCAP_TICK_NS);
        }
    } else {
        timer_del(s->timer);
    }
}

/* ---- registers ----------------------------------------------------------- */

static uint64_t gm_vcap_read(void *opaque, hwaddr off, unsigned size)
{
    GmVcapState *s = opaque;

    switch (off) {
    case R_VERSION:
        return s->version;
    case R_CAPS:
        return s->caps;
    }
    return s->regs[off / 4];
}

static void gm_vcap_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    GmVcapState *s = opaque;
    int i;

    for (i = 0; i < ARRAY_SIZE(gm_vcap_status_regs); i++) {
        if (off == gm_vcap_status_regs[i]) {
            s->regs[off / 4] &= ~val;
            gm_vcap_update(s);
            return;
        }
    }
    switch (off) {
    case R_VERSION:
    case R_CAPS:
        return;
    case R_CTRL0:
        s->regs[off / 4] = val & ~CTRL0_SELF_CLEAR;
        if (val & CTRL0_DMA_OFF) {
            s->regs[R_STS_ERR / 4] |= STS_DMA_DISABLED;
            gm_vcap_update(s);
        }
        return;
    case R_CTRL1:
        s->regs[off / 4] = val & ~(CTRL1_FIRE | CTRL1_RESET);
        gm_vcap_arm(s);
        return;
    }
    s->regs[off / 4] = val;
    if (off >= 0x5500 && off < 0x5600) {
        gm_vcap_update(s);
    }
}

static const MemoryRegionOps gm_vcap_ops = {
    .read = gm_vcap_read,
    .write = gm_vcap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_vcap_reset(DeviceState *dev)
{
    GmVcapState *s = GM_VCAP(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->ticks = s->frames = 0;
    gm_vcap_update(s);
}

static void gm_vcap_realize(DeviceState *dev, Error **errp)
{
    GmVcapState *s = GM_VCAP(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_vcap_ops, s,
                          TYPE_GM_VCAP, GM_VCAP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_vcap_tick, s);
}

static const Property gm_vcap_props[] = {
    DEFINE_PROP_UINT32("version", GmVcapState, version, 0x20140311),
    /* cascade, 1 VI, OSD 8, mark 4, mask 8, scaling 0x7, 4 scalers */
    DEFINE_PROP_UINT32("caps", GmVcapState, caps, 0x0F184874),
};

static const VMStateDescription vmstate_gm_vcap = {
    .name = TYPE_GM_VCAP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, GmVcapState),
        VMSTATE_UINT32_ARRAY(regs, GmVcapState, GM_VCAP_NREGS),
        VMSTATE_UINT32(ticks, GmVcapState),
        VMSTATE_UINT32(frames, GmVcapState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_vcap_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_vcap_realize;
    device_class_set_legacy_reset(dc, gm_vcap_reset);
    device_class_set_props(dc, gm_vcap_props);
    dc->vmsd = &vmstate_gm_vcap;
}

static const TypeInfo gm_vcap_info = {
    .name           = TYPE_GM_VCAP,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmVcapState),
    .class_init     = gm_vcap_class_init,
};

static void gm_vcap_register_types(void)
{
    type_register_static(&gm_vcap_info);
}

type_init(gm_vcap_register_types)
