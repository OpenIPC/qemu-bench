/*
 * Anyka AK3918EV200 ISP2 (0x20000000, interrupt 1): the frames, not the
 * image processing.
 *
 * The register contract is the ISP2 core the OpenIPC kernel builds from the
 * SDK sources (drivers/media/platform/soc_camera/plat-anyka/ak39_isp2.c and
 * ak39_isp2_reg.h, CONFIG_CPU_AK3918EV200), driven by the ak-cam capture
 * bridge (drivers/media/platform/anyka/ak-cam.c):
 *
 *   +0x00 GEN_CTRL   bit 20 ISP_ENABLE starts and stops capture; bits 15..18
 *                    enable the main output of buffers 1..4, 21..24 their
 *                    sub output (EV200 keeps them here, counting up)
 *   +0x04 CSI_ERR_CHECK_1  the sensor's window, width bits 0..13, height
 *                    14..27
 *   +0x10 STATUS     bit 0 FRAME_ALL_DONE, 1 MAIN_FRAME_DONE, 2
 *                    SUB_FRAME_DONE (write 1 to clear); bits 22..23 the
 *                    buffer the main channel writes next, 20..21 the sub's
 *   +0x14 interrupt enables, the same bit numbers
 *   +0x18 DMA_REG_ADDR  the register table in DRAM (below)
 *   +0x1C REG_UPDATE    the table is (re)loaded
 *
 * Most of the ISP's configuration is not in MMIO at all but in a register
 * table in DRAM that the core fetches by DMA: five register blocks and six
 * memory blocks, laid out by isp2_module_init(). The model reads the parts
 * that say where a frame goes:
 *
 *   block 5 (table +3292) words 25..30   buffer 1: main Y, U, V; sub Y, U, V
 *   block 1 (table +0)    words 31..48   buffers 2..4: main Y (31..33),
 *                         U (34..36), V (37..39); sub Y (40..42), U
 *                         (43..45), V (46..48)
 *   block 4 (table +3152) word 31 / 33   main / sub size: width bits 0..11,
 *                         height 12..23
 *
 * Every frame period, while ISP_ENABLE is set, the core writes an I420
 * picture into the buffer it is on (if that buffer is enabled), raises
 * FRAME_ALL_DONE, and moves on to the next enabled buffer, which it reports
 * in STATUS. That is the order ak39_isp2.c's ak_isp_irq_work() expects: it
 * takes the buffer it predicted as the finished one and reads STATUS for the
 * next. With one buffer enabled the core stays on it.
 *
 * The picture is a test pattern -- colour bars, a moving block, a frame
 * counter strip -- so that an encoder downstream has something to code and
 * a decoded stream can be checked frame by frame.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/dma.h"
#include "system/address-spaces.h"

#define TYPE_AK_ISP "ak-isp"
OBJECT_DECLARE_SIMPLE_TYPE(AkIspState, AK_ISP)

#define AK_ISP_SIZE         0x20000
#define AK_ISP_NREGS        0x40

#define REG_GEN_CTRL        0x00
#define   GEN_ISP_ENABLE    (1u << 20)
#define   GEN_MAIN_EN_SHIFT 15
#define   GEN_SUB_EN_SHIFT  21
#define REG_CSI_CHECK1      0x04
#define REG_STATUS          0x10
#define   ST_FRAME_ALL_DONE (1u << 0)
#define   ST_MAIN_DONE      (1u << 1)
#define   ST_SUB_DONE       (1u << 2)
#define   ST_W1C            0xFFFFu
#define   ST_SUB_ID_SHIFT   20
#define   ST_MAIN_ID_SHIFT  22
#define REG_INTEN           0x14
#define REG_DMA_TABLE       0x18
#define REG_UPDATE          0x1C

/* isp2_module_init()'s EV200 layout of the register table, in bytes */
#define TBL_BLK1            0
#define TBL_BLK4            3152
#define TBL_BLK5            3292
#define TBL_SIZE            3424

#define MAX_W               4096
#define MAX_H               4096

struct AkIspState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[AK_ISP_NREGS];
    uint32_t cur;           /* the buffer the core writes next, 0..3 */
    uint32_t frames;

    uint32_t fps;
    uint8_t *row;           /* one row of the picture being written */
};

static void ak_isp_update_irq(AkIspState *s)
{
    uint32_t pending = s->regs[REG_STATUS / 4] & s->regs[REG_INTEN / 4] &
                       ST_W1C;

    qemu_set_irq(s->irq, pending != 0);
}

static uint32_t tbl_word(AkIspState *s, uint32_t byte_off)
{
    hwaddr base = s->regs[REG_DMA_TABLE / 4];
    uint32_t v = 0;

    if (!base) {
        return 0;
    }
    dma_memory_read(&address_space_memory, base + byte_off, &v, 4,
                    MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

typedef struct {
    uint32_t y, u, v;
} AkIspPlanes;

static void buffer_planes(AkIspState *s, unsigned buf, bool sub,
                          AkIspPlanes *p)
{
    if (buf == 0) {
        uint32_t w = TBL_BLK5 + (sub ? 28 : 25) * 4;

        p->y = tbl_word(s, w);
        p->u = tbl_word(s, w + 4);
        p->v = tbl_word(s, w + 8);
    } else {
        unsigned k = buf - 1;

        p->y = tbl_word(s, TBL_BLK1 + ((sub ? 40 : 31) + k) * 4);
        p->u = tbl_word(s, TBL_BLK1 + ((sub ? 43 : 34) + k) * 4);
        p->v = tbl_word(s, TBL_BLK1 + ((sub ? 46 : 37) + k) * 4);
    }
}

static bool channel_size(AkIspState *s, bool sub, uint32_t *w, uint32_t *h)
{
    uint32_t v = tbl_word(s, TBL_BLK4 + (sub ? 33 : 31) * 4);

    *w = v & 0xFFF;
    *h = (v >> 12) & 0xFFF;
    if (!sub && (!*w || !*h)) {
        /* no scaler set up: the sensor's window */
        v = s->regs[REG_CSI_CHECK1 / 4];
        *w = v & 0x3FFF;
        *h = (v >> 14) & 0x3FFF;
    }
    return *w && *h && *w <= MAX_W && *h <= MAX_H;
}

/* BT.601 limited range, the order of SMPTE bars */
static const uint8_t bars[8][3] = {
    { 180, 128, 128 }, { 168, 44, 136 }, { 145, 147, 44 }, { 133, 63, 52 },
    { 63, 193, 204 },  { 51, 109, 212 }, { 28, 212, 120 }, { 16, 128, 128 },
};

/*
 * Bars over the top two thirds, a ramp below; a block that crosses the
 * picture once every 64 frames; and the frame counter as sixteen black or
 * white cells along the bottom, so a decoded frame names itself.
 */
static void pixel(AkIspState *s, uint32_t x, uint32_t y, uint32_t w,
                  uint32_t h, uint8_t yuv[3])
{
    uint32_t bx = (s->frames % 64) * (w - w / 8) / 63;
    uint32_t by = h / 3;

    if (y >= h - h / 16) {
        uint32_t cell = x * 16 / w;
        bool on = (s->frames >> (15 - cell)) & 1;

        yuv[0] = on ? 235 : 16;
        yuv[1] = yuv[2] = 128;
    } else if (x >= bx && x < bx + w / 8 && y >= by && y < by + h / 6) {
        yuv[0] = 235;
        yuv[1] = 90;
        yuv[2] = 240;
    } else if (y < h * 2 / 3) {
        const uint8_t *b = bars[x * 8 / w];

        yuv[0] = b[0];
        yuv[1] = b[1];
        yuv[2] = b[2];
    } else {
        yuv[0] = 16 + x * 219 / w;
        yuv[1] = yuv[2] = 128;
    }
}

static void write_picture(AkIspState *s, const AkIspPlanes *p, uint32_t w,
                          uint32_t h)
{
    uint8_t yuv[3];

    if (!p->y) {
        return;
    }
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            pixel(s, x, y, w, h, yuv);
            s->row[x] = yuv[0];
        }
        dma_memory_write(&address_space_memory, p->y + (hwaddr)y * w, s->row,
                         w, MEMTXATTRS_UNSPECIFIED);
    }
    if (!p->u || !p->v) {
        return;
    }
    for (int plane = 1; plane <= 2; plane++) {
        hwaddr base = plane == 1 ? p->u : p->v;

        for (uint32_t y = 0; y < h / 2; y++) {
            for (uint32_t x = 0; x < w / 2; x++) {
                pixel(s, x * 2, y * 2, w, h, yuv);
                s->row[x] = yuv[plane];
            }
            dma_memory_write(&address_space_memory,
                             base + (hwaddr)y * (w / 2), s->row, w / 2,
                             MEMTXATTRS_UNSPECIFIED);
        }
    }
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
    uint32_t gen = s->regs[REG_GEN_CTRL / 4];
    uint32_t main_en = (gen >> GEN_MAIN_EN_SHIFT) & 0xF;
    uint32_t sub_en = (gen >> GEN_SUB_EN_SHIFT) & 0xF;
    uint32_t status = 0;
    uint32_t w, h;
    AkIspPlanes p;

    if (!(gen & GEN_ISP_ENABLE)) {
        return;
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                        frame_ns(s));

    if (!(main_en & (1u << s->cur))) {
        /* the buffer it is on was taken away: move on, write nothing */
        int n = next_enabled(main_en, s->cur);

        if (n < 0) {
            return;
        }
        s->cur = n;
    }

    if (channel_size(s, false, &w, &h)) {
        buffer_planes(s, s->cur, false, &p);
        write_picture(s, &p, w, h);
        status |= ST_MAIN_DONE;
    }
    if ((sub_en & (1u << s->cur)) && channel_size(s, true, &w, &h)) {
        buffer_planes(s, s->cur, true, &p);
        write_picture(s, &p, w, h);
        status |= ST_SUB_DONE;
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
        s->regs[REG_STATUS / 4] &= ~(val & ST_W1C);
        ak_isp_update_irq(s);
        return;
    case REG_INTEN:
        s->regs[off / 4] = val;
        ak_isp_update_irq(s);
        return;
    case REG_GEN_CTRL:
        s->regs[off / 4] = val;
        if ((val & GEN_ISP_ENABLE) && !(old & GEN_ISP_ENABLE)) {
            /* ak_isp_vi_start_capturing(): the first enabled buffer */
            int n = next_enabled((val >> GEN_MAIN_EN_SHIFT) & 0xF, 3);

            s->cur = n < 0 ? 0 : n;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                frame_ns(s));
        } else if (!(val & GEN_ISP_ENABLE)) {
            timer_del(s->timer);
        }
        return;
    case REG_UPDATE:
        /* the table is read when a frame is written; nothing to latch */
        s->regs[off / 4] = 0;
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
    s->cur = 0;
    s->frames = 0;
    qemu_set_irq(s->irq, 0);
}

static void ak_isp_realize(DeviceState *dev, Error **errp)
{
    AkIspState *s = AK_ISP(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ak_isp_ops, s, TYPE_AK_ISP,
                          AK_ISP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ak_isp_frame, s);
    s->row = g_malloc(MAX_W);
}

static void ak_isp_unrealize(DeviceState *dev)
{
    AkIspState *s = AK_ISP(dev);

    timer_free(s->timer);
    g_free(s->row);
}

static const VMStateDescription ak_isp_vmstate = {
    .name = TYPE_AK_ISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AkIspState, AK_ISP_NREGS),
        VMSTATE_UINT32(cur, AkIspState),
        VMSTATE_UINT32(frames, AkIspState),
        VMSTATE_TIMER_PTR(timer, AkIspState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property ak_isp_props[] = {
    DEFINE_PROP_UINT32("fps", AkIspState, fps, 15),
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
