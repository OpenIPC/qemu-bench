/*
 * Grain Media ISP328 image signal processor.
 *
 * The ISP at 0x93930000 on the GM813x (fisp328.ko, fisp_algorithm.ko;
 * their headers isp_common.h, isp_dev.h) sits between the sensor and the
 * capture engine.  Its window is register file and lookup tables
 * (0x0000-0x2FFF), which the model keeps as memory, apart from:
 *
 *   0x08 func0   bit 31 ISP on; 11-14 histogram / AE / AWB / AF
 *   0x48 input DMA   [31:8] a command table's address, bit 0 runs it
 *   0x4C output DMA  [31:8] the statistics copy table's address
 *   0x50 / 0x54  interrupt status 0 / 1, write 1 to clear
 *   0x58 / 0x5C  their masks, 1 = masked
 *   0x68 DMA control: reads 3 (DMA idle and disabled)
 *
 * Status 0: 0-3 AE / AWB / AF / histogram ready, 16 frame start, 17 frame
 * end, 18 input DMA done, 19 output DMA done; interrupt 43.  Frames run
 * at "fps" whether or not the ISP is on -- the driver waits for frame
 * start and end around every stop and resize.  At each frame start the
 * driver queues its register updates as an input DMA table:
 *
 *   header [31] last, [30] burst, [27:16] words incl. the header,
 *          [15:2] register offset; a burst has a dummy word, then the
 *          values for consecutive registers; otherwise the value follows
 *          and further offset / value pairs, the last offset flagged
 *
 * At each frame end the model fills the statistics registers for the
 * scene, runs the output DMA table -- pairs of [31] last | [27:16] words
 * | [15:0] register offset and a destination address, which the AE / AWB
 * / AF threads then read -- and raises frame end with the ready bits of
 * the enabled units (the driver handles those at the next frame start).
 * The scene is a neutral grey of level "luma" (the algorithms' default
 * target is 100), so AE settles at once and AWB keeps unity gains: per AE
 * window i (0x28C + 8i / 0x290 + 8i, start and end x | y << 16) sums of
 * R, G, B of luma * area / 4 at 0x2800 + 0x18i; the histogram (0x2000,
 * 16-bit bins; 0x2400, 32-bit) around the level; AWB (0x2A00) ratios of
 * 1.0 (128) over the 2x2 blocks of its window; AF (0x2B00) a flat
 * texture figure per window.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
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

#define TYPE_GM_ISP "gm-isp"
OBJECT_DECLARE_SIMPLE_TYPE(GmIspState, GM_ISP)

#define GM_ISP_SIZE         0x3000
#define GM_ISP_NREGS        (GM_ISP_SIZE / 4)
#define GM_ISP_MAX_WORDS    16384

#define R_FUNC0             0x008
#define R_IDMA              0x048
#define R_ODMA              0x04C
#define R_STATUS0           0x050
#define R_STATUS1           0x054
#define R_MASK0             0x058
#define R_MASK1             0x05C
#define R_DMACTL            0x068
#define R_HIST              0x2000
#define R_CE_HIST           0x2400
#define R_AE_SUM            0x2800
#define R_AE_CNT            0x2900
#define R_AWB               0x2A00
#define R_AF                0x2B00
#define R_HIST_WIN          0x264
#define R_AE_WIN            0x28C
#define R_AWB_WIN           0x2E0
#define R_AF_WIN            0x314

#define FUNC0_ON            (1u << 31)
#define FUNC0_HIST          (1u << 11)
#define FUNC0_AE            (1u << 12)
#define FUNC0_AWB           (1u << 13)
#define FUNC0_AF            (1u << 14)

#define ST_AE               (1u << 0)
#define ST_AWB              (1u << 1)
#define ST_AF               (1u << 2)
#define ST_HIST             (1u << 3)
#define ST_VSTART           (1u << 16)
#define ST_VEND             (1u << 17)
#define ST_IDMA_DONE        (1u << 18)
#define ST_ODMA_DONE        (1u << 19)

#define DMA_GO              (1u << 0)

struct GmIspState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[GM_ISP_NREGS];
    bool in_frame;              /* between frame start and end */
    uint32_t fps;
    uint32_t luma;
};

static void gm_isp_update(GmIspState *s)
{
    qemu_set_irq(s->irq,
                 (s->regs[R_STATUS0 / 4] & ~s->regs[R_MASK0 / 4]) ||
                 (s->regs[R_STATUS1 / 4] & ~s->regs[R_MASK1 / 4]));
}

/* a window register pair: start x | y << 16, end (inclusive) */
static uint64_t gm_isp_win_area(GmIspState *s, uint32_t off,
                                uint32_t *w, uint32_t *h)
{
    const uint32_t a = s->regs[off / 4], b = s->regs[off / 4 + 1];
    const uint32_t x0 = extract32(a, 0, 13), y0 = extract32(a, 16, 13);
    const uint32_t x1 = extract32(b, 0, 13), y1 = extract32(b, 16, 13);

    *w = x1 >= x0 ? x1 - x0 + 1 : 0;
    *h = y1 >= y0 ? y1 - y0 + 1 : 0;
    return (uint64_t)*w * *h;
}

static void gm_isp_put64(GmIspState *s, uint32_t off, uint64_t v)
{
    s->regs[off / 4] = v;
    s->regs[off / 4 + 1] = v >> 32;
}

static void gm_isp_statistics(GmIspState *s)
{
    const uint32_t y = MIN(s->luma, 255);
    uint32_t w, h, i;
    uint64_t area, n;

    /* histograms: a narrow bell around the level */
    area = gm_isp_win_area(s, R_HIST_WIN, &w, &h);
    memset(&s->regs[R_HIST / 4], 0, 0x200);
    memset(&s->regs[R_CE_HIST / 4], 0, 0x400);
    for (i = 0; i < 256; i++) {
        const int d = abs((int)i - (int)y);
        const uint64_t v = d < 8 ? area / 64 * (8 - d) / 64 : 0;

        s->regs[R_HIST / 4 + i / 2] |= MIN(v, 0xFFFF) << (16 * (i & 1));
        s->regs[R_CE_HIST / 4 + i] = MIN(v, 0xFFFFFFFF);
    }
    /* AE: R, G, B sums giving (R + 2G + B) / area = the level */
    for (i = 0; i < 9; i++) {
        const uint64_t sum = gm_isp_win_area(s, R_AE_WIN + 8 * i, &w, &h) *
                             y / 4;

        gm_isp_put64(s, R_AE_SUM + 0x18 * i, sum);
        gm_isp_put64(s, R_AE_SUM + 0x18 * i + 8, sum);
        gm_isp_put64(s, R_AE_SUM + 0x18 * i + 16, sum);
        memset(&s->regs[(R_AE_CNT + 0x10 * i) / 4], 0, 16);
    }
    /* AWB: neutral, 2x2 blocks */
    gm_isp_win_area(s, R_AWB_WIN, &w, &h);
    n = (uint64_t)(w / 2) * (h / 2);
    s->regs[R_AWB / 4 + 0] = n * 128;           /* rg_lvl */
    s->regs[R_AWB / 4 + 1] = n * 128;           /* bg_lvl */
    s->regs[R_AWB / 4 + 2] = n;                 /* pixel_num_lvl */
    s->regs[R_AWB / 4 + 3] = n * 128;           /* rg */
    s->regs[R_AWB / 4 + 4] = n * 128;           /* bg */
    s->regs[R_AWB / 4 + 5] = n * y;             /* r, g, b */
    s->regs[R_AWB / 4 + 6] = n * y;
    s->regs[R_AWB / 4 + 7] = n * y;
    s->regs[R_AWB / 4 + 8] = n;                 /* pixel_num */
    s->regs[R_AWB / 4 + 9] = 0;
    /* AF: a flat texture figure */
    for (i = 0; i < 9; i++) {
        s->regs[R_AF / 4 + i] = gm_isp_win_area(s, R_AF_WIN + 8 * i,
                                                &w, &h) * 8;
    }
    s->regs[R_AF / 4 + 9] = 0;
}

/* the statistics copy: register blocks to the algorithms' buffers */
static void gm_isp_output_dma(GmIspState *s)
{
    hwaddr tbl = s->regs[R_ODMA / 4] & ~0xFFu;
    int n;

    if (!tbl) {
        return;
    }
    for (n = 0; n < 64; n++, tbl += 8) {
        uint32_t e[2], off, words;

        dma_memory_read(&address_space_memory, tbl, e, 8,
                        MEMTXATTRS_UNSPECIFIED);
        e[0] = le32_to_cpu(e[0]);
        e[1] = le32_to_cpu(e[1]);
        off = e[0] & 0xFFFC;
        words = extract32(e[0], 16, 12);
        if (off + words * 4 <= GM_ISP_SIZE) {
            uint32_t buf[0x1000];       /* words is 12 bits */
            uint32_t i;

            for (i = 0; i < words; i++) {
                buf[i] = cpu_to_le32(s->regs[off / 4 + i]);
            }
            dma_memory_write(&address_space_memory, e[1], buf, words * 4,
                             MEMTXATTRS_UNSPECIFIED);
        }
        if (e[0] & (1u << 31)) {
            break;
        }
    }
    s->regs[R_STATUS0 / 4] |= ST_ODMA_DONE;
}

static void gm_isp_reg_write(GmIspState *s, uint32_t off, uint32_t val);

static uint32_t ldl_le_dma_or_zero(hwaddr a)
{
    uint32_t v = 0;

    dma_memory_read(&address_space_memory, a, &v, 4, MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

/* the driver's register jobs */
static void gm_isp_input_dma(GmIspState *s)
{
    hwaddr p = s->regs[R_IDMA / 4] & ~0xFFu;
    int budget = GM_ISP_MAX_WORDS;

    while (budget > 0) {
        uint32_t hdr, off, words, v, i;

        hdr = ldl_le_dma_or_zero(p);
        off = hdr & 0xFFFC;
        words = extract32(hdr, 16, 12);
        if (hdr & (1u << 30)) {                 /* burst */
            for (i = 0; i + 2 < words && budget > 0; i++, budget--) {
                v = ldl_le_dma_or_zero(p + 8 + 4 * i);
                gm_isp_reg_write(s, off + 4 * i, v);
            }
            p += 4 * MAX(words, 2);
            budget -= 2;
        } else {                                /* offset / value pairs */
            v = ldl_le_dma_or_zero(p + 4);
            gm_isp_reg_write(s, off, v);
            p += 8;
            budget -= 2;
        }
        if (hdr & (1u << 31)) {
            break;
        }
    }
    s->regs[R_IDMA / 4] &= ~DMA_GO;
    s->regs[R_STATUS0 / 4] |= ST_IDMA_DONE;
}

static void gm_isp_frame(void *opaque)
{
    GmIspState *s = opaque;
    const int64_t period = NANOSECONDS_PER_SECOND / MAX(s->fps, 1);
    const uint32_t f0 = s->regs[R_FUNC0 / 4];

    if (!s->in_frame) {
        s->regs[R_STATUS0 / 4] |= ST_VSTART;
        s->in_frame = true;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            period * 3 / 4);
    } else {
        gm_isp_statistics(s);
        gm_isp_output_dma(s);
        s->regs[R_STATUS0 / 4] |= ST_VEND;
        if (f0 & FUNC0_ON) {
            s->regs[R_STATUS0 / 4] |= (f0 & FUNC0_AE ? ST_AE : 0) |
                                      (f0 & FUNC0_AWB ? ST_AWB : 0) |
                                      (f0 & FUNC0_AF ? ST_AF : 0) |
                                      (f0 & FUNC0_HIST ? ST_HIST : 0);
        }
        s->in_frame = false;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            period / 4);
    }
    gm_isp_update(s);
}

static void gm_isp_reg_write(GmIspState *s, uint32_t off, uint32_t val)
{
    if (off >= GM_ISP_SIZE) {
        return;
    }
    switch (off) {
    case R_STATUS0:
    case R_STATUS1:
        s->regs[off / 4] &= ~val;
        break;
    case R_DMACTL:
        s->regs[off / 4] = val & ~0x10u;        /* the reset is done */
        break;
    case R_IDMA:
        s->regs[off / 4] = val;
        if (val & DMA_GO) {
            gm_isp_input_dma(s);
        }
        break;
    default:
        s->regs[off / 4] = val;
        break;
    }
}

static uint64_t gm_isp_read(void *opaque, hwaddr off, unsigned size)
{
    GmIspState *s = opaque;

    if (off == R_DMACTL) {
        return s->regs[off / 4] | 3;
    }
    return s->regs[off / 4];
}

static void gm_isp_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    GmIspState *s = opaque;

    gm_isp_reg_write(s, off, val);
    gm_isp_update(s);
}

static const MemoryRegionOps gm_isp_ops = {
    .read = gm_isp_read,
    .write = gm_isp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_isp_reset(DeviceState *dev)
{
    GmIspState *s = GM_ISP(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_MASK0 / 4] = s->regs[R_MASK1 / 4] = 0xFFFFFFFF;
    s->in_frame = false;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                        NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
    gm_isp_update(s);
}

static void gm_isp_realize(DeviceState *dev, Error **errp)
{
    GmIspState *s = GM_ISP(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_isp_ops, s, TYPE_GM_ISP,
                          GM_ISP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_isp_frame, s);
}

static const Property gm_isp_props[] = {
    DEFINE_PROP_UINT32("fps", GmIspState, fps, 30),
    DEFINE_PROP_UINT32("luma", GmIspState, luma, 100),
};

static const VMStateDescription vmstate_gm_isp = {
    .name = TYPE_GM_ISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, GmIspState),
        VMSTATE_UINT32_ARRAY(regs, GmIspState, GM_ISP_NREGS),
        VMSTATE_BOOL(in_frame, GmIspState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_isp_realize;
    device_class_set_legacy_reset(dc, gm_isp_reset);
    device_class_set_props(dc, gm_isp_props);
    dc->vmsd = &vmstate_gm_isp;
}

static const TypeInfo gm_isp_info = {
    .name           = TYPE_GM_ISP,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmIspState),
    .class_init     = gm_isp_class_init,
};

static void gm_isp_register_types(void)
{
    type_register_static(&gm_isp_info);
}

type_init(gm_isp_register_types)
