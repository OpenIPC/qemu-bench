/*
 * Allwinner CSIC DMA ("vinc", the video input's write-back) on the
 * V831/V833: four channels at 0x06609000 + 0x200 * n, the end of the
 * sensor -> CSI -> ISP -> scaler chain.  There is no sensor data: each
 * frame, at the sensor's rate, the channel paints a test pattern into the
 * buffer the driver queued, in the geometry the driver programmed, and
 * raises the interrupts the vendor vin driver (vin_core.c vin_isr) waits
 * for.  Buffer addresses are device addresses, behind the IOMMU ("iommu"
 * link) as its CSI master.
 *
 *   0x000 EN       bit 0 top enable, 3 FBC, 4 DMA enable, 7 buffer-address
 *                  FIFO, 8 LBC
 *   0x004 CFG      bits 19:16 output format (frame NV12 6, NV21 10, ...)
 *   0x010 HSIZE / 0x014 VSIZE   bits 29:16 length
 *   0x020 / 0x028  F0 / F1 buffer address (>> 2), single-address mode
 *   0x038 BUF_LEN  bits 13:0 Y line length, 29:16 C line length
 *   0x050 INT_EN / 0x054 INT_STA (write 1 to clear): 1 frame done,
 *                  7 vsync, 14 stored frames, 15 frame lost
 *   0x05C FRM_CNT
 *   0x080 / 0x084  F0 / F1 address FIFO entries (>> 2), FIFO mode
 *   0x08C BUF_THRESHOLD  bits 21:16 frames stored per interrupt
 *
 * FIFO mode (the firmware's CONFIG_BUF_AUTO_UPDATE): every frame takes the
 * next queued address pair, and every `threshold` frames the channel
 * raises STORED_FRM_CNT, for which the driver completes that many
 * buffers; with nothing queued the frame is lost.  Single-address mode:
 * the address written before a vsync is the next frame's, and the frame
 * before it is complete at the vsync.
 *
 * In LBC mode (Allwinner's compressed format, which the encoder takes
 * directly, and whose bitstream is not public) the frame is the private
 * stand-in of aw-qlbc.h, which the aw-ve encoder reads.  FBC frames are
 * signalled but not painted.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/misc/aw-iommu.h"
#include "hw/misc/aw-qlbc.h"
#include "qom/object.h"

#define TYPE_AW_VINC "aw-vinc"
OBJECT_DECLARE_SIMPLE_TYPE(AwVincState, AW_VINC)

#define AW_VINC_SIZE        0x200
#define AW_VINC_FIFO        32

#define REG_EN              0x000
#define REG_CFG             0x004
#define REG_HSIZE           0x010
#define REG_VSIZE           0x014
#define REG_F0_BUFA         0x020
#define REG_F1_BUFA         0x028
#define REG_BUF_LEN         0x038
#define REG_INT_EN          0x050
#define REG_INT_STA         0x054
#define REG_FRM_CNT         0x05C
#define REG_F0_ENTRY        0x080
#define REG_F1_ENTRY        0x084
#define REG_F2_ENTRY        0x088
#define REG_BUF_THRESHOLD   0x08C
#define REG_STORED_FRM_CNT  0x094

#define EN_TOP              (1u << 0)
#define EN_DMA              (1u << 4)
#define EN_ADDR_FIFO        (1u << 7)
#define EN_FBC              (1u << 3)
#define EN_LBC              (1u << 8)

#define INT_FD              (1u << 1)
#define INT_VS              (1u << 7)
#define INT_STORED          (1u << 14)
#define INT_LOST            (1u << 15)

typedef struct AwVincFifo {
    uint32_t addr[AW_VINC_FIFO];
    unsigned head, count;
} AwVincFifo;

struct AwVincState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t fps;
    bool trace;
    DeviceState *iommu;

    uint32_t regs[AW_VINC_SIZE / 4];
    AwVincFifo fifo[2];         /* Y, C */
    uint32_t latched[2];        /* single-address mode: frame in progress */
    bool have_latched;
    uint32_t stored;
    uint32_t frames;
    uint8_t *line;
};

static void aw_vinc_update_irq(AwVincState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[REG_INT_STA >> 2] &
                            s->regs[REG_INT_EN >> 2]));
}

static void aw_vinc_fifo_push(AwVincFifo *f, uint32_t v)
{
    if (f->count == AW_VINC_FIFO) {
        qemu_log_mask(LOG_GUEST_ERROR, "aw-vinc: address FIFO overflow\n");
        return;
    }
    f->addr[(f->head + f->count) % AW_VINC_FIFO] = v;
    f->count++;
}

static uint32_t aw_vinc_fifo_pop(AwVincFifo *f)
{
    uint32_t v = f->addr[f->head];

    f->head = (f->head + 1) % AW_VINC_FIFO;
    f->count--;
    return v;
}

/*
 * The test pattern: eight vertical colour bars, with a bright square that
 * moves a step a frame, so motion is visible downstream.  It is drawn in
 * the sensor's 1920x1080 and scaled to each channel's size, and its frame
 * number comes from the clock, so the channels show the same scene.
 */
#define PAT_W               1920
#define PAT_H               1080
#define PAT_SQ              (PAT_H / 8)

static const uint8_t aw_vinc_bars[8][3] = {     /* Y, U, V */
    { 235, 128, 128 }, { 210, 16, 146 }, { 170, 166, 16 },
    { 145, 54, 34 }, { 106, 202, 222 }, { 81, 90, 240 },
    { 41, 240, 110 }, { 16, 128, 128 },
};

typedef struct AwVincPattern {
    uint32_t w, h;
    uint32_t sx, sy;            /* the square, sensor pixels */
} AwVincPattern;

static void aw_vinc_pattern(AwVincState *s, AwVincPattern *p,
                            uint32_t w, uint32_t h)
{
    uint64_t frame = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                              MAX(s->fps, 1), NANOSECONDS_PER_SECOND);

    p->w = w;
    p->h = h;
    p->sx = (frame * 8) % (PAT_W - PAT_SQ);
    p->sy = (frame * 4) % (PAT_H - PAT_SQ);
}

/* Y, U, V at (x, y) of the channel's picture */
static void aw_vinc_pixel(const AwVincPattern *p, uint32_t x, uint32_t y,
                          uint8_t *yuv)
{
    uint32_t px = (uint64_t)x * PAT_W / p->w;
    uint32_t py = (uint64_t)y * PAT_H / p->h;

    memcpy(yuv, aw_vinc_bars[(px * 8 / PAT_W) & 7], 3);
    if (px >= p->sx && px < p->sx + PAT_SQ &&
        py >= p->sy && py < p->sy + PAT_SQ) {
        yuv[0] = 250;
    }
}

/* A frame in a raw format: Y then the interleaved C plane (4:2:0/4:2:2). */
static void aw_vinc_paint(AwVincState *s, const AwVincPattern *p,
                          uint32_t ya, uint32_t ca)
{
    uint32_t ylen = s->regs[REG_BUF_LEN >> 2] & 0x3FFF;
    uint32_t clen = (s->regs[REG_BUF_LEN >> 2] >> 16) & 0x3FFF;
    uint32_t fmt = (s->regs[REG_CFG >> 2] >> 16) & 0xF;
    bool vu = fmt == 9 || fmt == 10 || fmt == 8 || fmt == 11;
    bool c422 = fmt == 7 || fmt == 11 || fmt == 4 || fmt == 8;
    uint32_t x, y, ch;
    uint8_t yuv[3];

    ylen = ylen ? ylen : p->w;
    clen = clen ? clen : p->w;
    s->line = g_realloc(s->line, MAX(ylen, clen));
    for (y = 0; y < p->h; y++) {
        for (x = 0; x < ylen; x++) {
            aw_vinc_pixel(p, MIN(x, p->w - 1), y, yuv);
            s->line[x] = yuv[0];
        }
        aw_iommu_dma_write(s->iommu, AW_IOMMU_MASTER_CSI, ya + y * ylen,
                           s->line, ylen);
    }
    ch = c422 ? p->h : (p->h + 1) / 2;
    for (y = 0; y < ch; y++) {
        for (x = 0; x + 1 < clen; x += 2) {
            aw_vinc_pixel(p, MIN(x, p->w - 1), c422 ? y : y * 2, yuv);
            s->line[x] = vu ? yuv[2] : yuv[1];
            s->line[x + 1] = vu ? yuv[1] : yuv[2];
        }
        if (clen & 1) {
            s->line[clen - 1] = 128;    /* half a pair: no colour */
        }
        aw_iommu_dma_write(s->iommu, AW_IOMMU_MASTER_CSI, ca + y * clen,
                           s->line, clen);
    }
}

/*
 * A frame in LBC mode: the stand-in of aw-qlbc.h, block averages, at the
 * luma address.  (Averages of the pattern's samples at the blocks'
 * pixels, so the square's edges are blended as a camera's would be.)
 */
static void aw_vinc_paint_qlbc(AwVincState *s, const AwVincPattern *p,
                               uint32_t ya)
{
    AwQlbcHeader hd = {
        .magic = cpu_to_le32(AW_QLBC_MAGIC),
        .w = cpu_to_le16(p->w), .h = cpu_to_le16(p->h),
        .frame = cpu_to_le32(s->frames),
        .bw = cpu_to_le16((p->w + 3) / 4), .bh = cpu_to_le16((p->h + 3) / 4),
        .cw = cpu_to_le16((p->w + 7) / 8), .ch = cpu_to_le16((p->h + 7) / 8),
    };
    uint32_t bw = (p->w + 3) / 4, bh = (p->h + 3) / 4;
    uint32_t cw = (p->w + 7) / 8, ch = (p->h + 7) / 8;
    uint32_t size = aw_qlbc_size(p->w, p->h), bx, by, i, j;
    uint8_t *buf = g_malloc(size), *y = buf + sizeof(hd);
    uint8_t *u = y + bw * bh, *v = u + cw * ch;
    uint8_t yuv[3];

    memcpy(buf, &hd, sizeof(hd));
    for (by = 0; by < bh; by++) {
        for (bx = 0; bx < bw; bx++) {
            uint32_t sum = 0;

            for (i = 0; i < 4; i++) {
                for (j = 0; j < 4; j++) {
                    aw_vinc_pixel(p, MIN(bx * 4 + j, p->w - 1),
                                  MIN(by * 4 + i, p->h - 1), yuv);
                    sum += yuv[0];
                }
            }
            y[by * bw + bx] = (sum + 8) / 16;
        }
    }
    for (by = 0; by < ch; by++) {
        for (bx = 0; bx < cw; bx++) {
            uint32_t su = 0, sv = 0;

            for (i = 0; i < 4; i++) {
                for (j = 0; j < 4; j++) {
                    aw_vinc_pixel(p, MIN(bx * 8 + j * 2, p->w - 1),
                                  MIN(by * 8 + i * 2, p->h - 1), yuv);
                    su += yuv[1];
                    sv += yuv[2];
                }
            }
            u[by * cw + bx] = (su + 8) / 16;
            v[by * cw + bx] = (sv + 8) / 16;
        }
    }
    aw_iommu_dma_write(s->iommu, AW_IOMMU_MASTER_CSI, ya, buf, size);
    g_free(buf);
}

/* The frame for the buffer at @ya / @ca, in the channel's format. */
static void aw_vinc_store(AwVincState *s, uint32_t ya, uint32_t ca)
{
    uint32_t w = (s->regs[REG_HSIZE >> 2] >> 16) & 0x3FFF;
    uint32_t h = (s->regs[REG_VSIZE >> 2] >> 16) & 0x3FFF;
    uint32_t en = s->regs[REG_EN >> 2];
    AwVincPattern p;

    if (!w || !h || (en & EN_FBC)) {
        return;
    }
    aw_vinc_pattern(s, &p, w, h);
    if (en & EN_LBC) {
        aw_vinc_paint_qlbc(s, &p, ya);
    } else {
        aw_vinc_paint(s, &p, ya, ca);
    }
}

static void aw_vinc_frame(void *opaque)
{
    AwVincState *s = opaque;
    uint32_t en = s->regs[REG_EN >> 2];
    uint32_t *sta = &s->regs[REG_INT_STA >> 2];

    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
    if ((en & (EN_TOP | EN_DMA)) != (EN_TOP | EN_DMA)) {
        return;
    }
    s->frames++;
    s->regs[REG_FRM_CNT >> 2] = (s->regs[REG_FRM_CNT >> 2] & ~0xFFFFu) |
                                (s->frames & 0xFFFF);
    *sta |= INT_VS;
    if (en & EN_ADDR_FIFO) {
        uint32_t thr = (s->regs[REG_BUF_THRESHOLD >> 2] >> 16) & 0x3F;

        if (!s->fifo[0].count) {
            *sta |= INT_LOST;
        } else {
            uint32_t ya = aw_vinc_fifo_pop(&s->fifo[0]) << 2;
            uint32_t ca = s->fifo[1].count ?
                          aw_vinc_fifo_pop(&s->fifo[1]) << 2 : 0;

            aw_vinc_store(s, ya, ca);
            if (++s->stored >= MAX(thr, 1)) {
                s->stored = 0;
                *sta |= INT_STORED;
            }
            *sta |= INT_FD;
        }
        s->regs[REG_STORED_FRM_CNT >> 2] = s->stored;
    } else {
        if (s->have_latched) {
            aw_vinc_store(s, s->latched[0] << 2, s->latched[1] << 2);
            *sta |= INT_FD;
        }
        s->latched[0] = s->regs[REG_F0_BUFA >> 2];
        s->latched[1] = s->regs[REG_F1_BUFA >> 2];
        s->have_latched = s->latched[0] != 0;
    }
    aw_vinc_update_irq(s);
}

static uint64_t aw_vinc_read(void *opaque, hwaddr addr, unsigned size)
{
    AwVincState *s = opaque;

    return s->regs[addr >> 2];
}

static void aw_vinc_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    AwVincState *s = opaque;
    uint32_t old;

    if (s->trace) {
        qemu_log("aw-vinc@%p: write 0x%03x <- 0x%08x\n", (void *)s,
                 (unsigned)addr, (uint32_t)val);
    }
    switch (addr) {
    case REG_INT_STA:
        s->regs[addr >> 2] &= ~val;
        aw_vinc_update_irq(s);
        return;
    case REG_F0_ENTRY:
        aw_vinc_fifo_push(&s->fifo[0], val);
        return;
    case REG_F1_ENTRY:
        aw_vinc_fifo_push(&s->fifo[1], val);
        return;
    case REG_F2_ENTRY:
        return;
    case REG_EN:
        old = s->regs[addr >> 2];
        s->regs[addr >> 2] = val;
        if ((val & EN_DMA) && !(old & EN_DMA)) {
            s->have_latched = false;
            s->stored = 0;
        }
        if (!(val & EN_DMA) && (old & EN_DMA)) {
            memset(s->fifo, 0, sizeof(s->fifo));
        }
        return;
    }
    s->regs[addr >> 2] = val;
    if (addr == REG_INT_EN) {
        aw_vinc_update_irq(s);
    }
}

static const MemoryRegionOps aw_vinc_ops = {
    .read = aw_vinc_read,
    .write = aw_vinc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aw_vinc_reset_hold(Object *obj, ResetType type)
{
    AwVincState *s = AW_VINC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->fifo, 0, sizeof(s->fifo));
    s->have_latched = false;
    s->stored = 0;
    s->frames = 0;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
}

static void aw_vinc_init(Object *obj)
{
    AwVincState *s = AW_VINC(obj);

    memory_region_init_io(&s->iomem, obj, &aw_vinc_ops, s, TYPE_AW_VINC,
                          AW_VINC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, aw_vinc_frame, s);
}

static void aw_vinc_finalize(Object *obj)
{
    AwVincState *s = AW_VINC(obj);

    timer_free(s->timer);
    g_free(s->line);
}

static const Property aw_vinc_properties[] = {
    DEFINE_PROP_UINT32("fps", AwVincState, fps, 20),
    DEFINE_PROP_BOOL("trace", AwVincState, trace, false),
    DEFINE_PROP_LINK("iommu", AwVincState, iommu, TYPE_DEVICE, DeviceState *),
};

static void aw_vinc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_vinc_reset_hold;
    device_class_set_props(dc, aw_vinc_properties);
}

static const TypeInfo aw_vinc_info = {
    .name              = TYPE_AW_VINC,
    .parent            = TYPE_SYS_BUS_DEVICE,
    .instance_size     = sizeof(AwVincState),
    .instance_init     = aw_vinc_init,
    .instance_finalize = aw_vinc_finalize,
    .class_init        = aw_vinc_class_init,
};

static void aw_vinc_register_types(void)
{
    type_register_static(&aw_vinc_info);
}

type_init(aw_vinc_register_types)
