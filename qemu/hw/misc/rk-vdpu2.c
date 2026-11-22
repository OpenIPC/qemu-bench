/*
 * Rockchip VDPU2 ("rockchip,vpu-decoder-v2", RV1126 / RV1109 0xFFB90400,
 * SPI 73, the IOMMU at 0xFFB90800 it shares with the VEPU2): the
 * Hantro-derived decoder.  On this SoC MPP gives it JPEG only
 * (osal/mpp_soc.c: vdpu2_jpeg_fix; H.264 / H.265 go to the rkvdec), as
 * mpp/hal/vpu/jpegd/hal_jpegd_vdpu2.c programs it:
 *
 *   reg51  (0x0cc) bits 23:0 the stream's length from reg64
 *   reg53  (0x0d4) the decode mode: 3 JPEG
 *   reg55  (0x0dc) interrupt: bit 0 raised, 1 disabled, 4 ready, 12
 *                  stream error; the driver writes 0 to clear it
 *   reg57  (0x0e4) bit 0 start ("dec_e"), back to 0 when done
 *   reg63  (0x0fc) the output's luma
 *   reg64  (0x100) the stream, 8-byte aligned, just before the scan
 *   reg120 (0x1e0) the picture in macroblocks: width bits 31:23 (+ 5:3
 *                  << 9), height 18:11 (+ 2:0 << 8)
 *   reg131 (0x20c) the output's chroma
 *
 * The HAL parses the headers itself and gives the hardware the Huffman
 * and quantisation tables in a table buffer and the scan's position in
 * reg64 / reg122.  The packet buffer reg64 points into holds the whole
 * file from SOI at its start (a page boundary): the model goes back to
 * it and decodes the file with jpeg-dec.c.  The output is the picture
 * at the macroblock size, luma then interleaved CbCr at the JPEG's own
 * chroma sampling (4:2:0 NV12, 4:2:2 NV16, 4:4:4 NV24; grey: luma only),
 * the planes MPP sets up for those (hor_stride = 16 x the macroblocks).
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
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"
#include "hw/misc/rk-iommu.h"
#include "jpeg-dec.h"

#define TYPE_RK_VDPU2 "rk-vdpu2"
OBJECT_DECLARE_SIMPLE_TYPE(RkVdpu2State, RK_VDPU2)

#define REGS_SIZE       0x400
#define R_STRM_LEN      51
#define R_MODE          53
#define R_INT           55
#define R_EN            57
#define R_OUT_Y         63
#define R_STRM          64
#define R_MB_SIZE       120
#define R_OUT_C         131

#define MODE_JPEG       3
#define INT_IRQ         (1u << 0)
#define INT_DIS         (1u << 1)
#define INT_RDY         (1u << 4)
#define INT_ERR         (1u << 12)
#define EN_DEC          (1u << 0)

#define JOB_NS          1000000         /* a frame's decode, guest time */
#define SOI_BACK        (1u << 20)      /* how far back SOI may be */

struct RkVdpu2State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;
    uint32_t regs[REGS_SIZE / 4];
    uint32_t result;                    /* reg55 when the job is done */
    Object *iommu;
    bool trace;
};

static void rk_vdpu2_update(RkVdpu2State *s)
{
    qemu_set_irq(s->irq, (s->regs[R_INT] & INT_IRQ) &&
                         !(s->regs[R_INT] & INT_DIS));
}

static bool rk_vdpu2_rd(RkVdpu2State *s, uint32_t a, void *b, uint32_t n)
{
    if (s->iommu) {
        return rk_iommu_dma_read(s->iommu, a, b, n);
    }
    return dma_memory_read(&address_space_memory, a, b, n,
                           MEMTXATTRS_UNSPECIFIED) == MEMTX_OK;
}

static void rk_vdpu2_wr(RkVdpu2State *s, uint32_t a, const void *b,
                        uint32_t n)
{
    if (s->iommu) {
        rk_iommu_dma_write(s->iommu, a, b, n);
    } else {
        dma_memory_write(&address_space_memory, a, b, n,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

/* The file's start: SOI at the page boundary at or before the scan */
static bool rk_vdpu2_soi(RkVdpu2State *s, uint32_t strm, uint32_t *start)
{
    uint32_t a = strm & ~0xfffu, back;
    uint8_t m[3];

    for (back = 0; back <= SOI_BACK && back <= a; back += 0x1000) {
        if (rk_vdpu2_rd(s, a - back, m, 3) &&
            m[0] == 0xff && m[1] == 0xd8 && m[2] == 0xff) {
            *start = a - back;
            return true;
        }
    }
    return false;
}

/* @img's plane @c, @w x @h of it, rows repeated / cut to fit */
static const uint8_t *rk_vdpu2_row(const JpegDecImage *img, int c, int y)
{
    return img->plane[c] + (size_t)MIN(y, img->ph[c] - 1) * img->pw[c];
}

static uint32_t rk_vdpu2_jpeg(RkVdpu2State *s)
{
    uint32_t strm = s->regs[R_STRM], len = s->regs[R_STRM_LEN] & 0xffffff;
    uint32_t mb = s->regs[R_MB_SIZE];
    int mbw = ((mb >> 23) & 0x1ff) | ((mb >> 3) & 7) << 9;
    int mbh = ((mb >> 11) & 0xff) | (mb & 7) << 8;
    int w = mbw * 16, h = mbh * 16, y, x, cw, ch;
    uint32_t start, total;
    g_autofree uint8_t *file = NULL, *row = NULL;
    JpegDecImage img;
    const char *err = "";

    /* the VDPU2's limit is 8192 x 8192; beyond it the job is refused */
    if (!mbw || !mbh || mbw > 512 || mbh > 512 ||
        !rk_vdpu2_soi(s, strm, &start)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rk-vdpu2: no JPEG at 0x%x\n", strm);
        return INT_IRQ | INT_ERR;
    }
    total = strm - start + len;
    if (total > 64 << 20) {
        return INT_IRQ | INT_ERR;
    }
    file = g_malloc(total);
    if (!rk_vdpu2_rd(s, start, file, total) ||
        !jpeg_dec_decode(file, total, &img, &err)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rk-vdpu2: JPEG at 0x%x: %s\n",
                      start, err);
        return INT_IRQ | INT_ERR;
    }
    if (s->trace) {
        qemu_log("rk-vdpu2: JPEG %dx%d, %d components %dx%d, at 0x%x "
                 "(%u bytes) -> 0x%x / 0x%x, %dx%d\n", img.w, img.h,
                 img.ncomp, img.hs[0], img.vs[0], start, total,
                 s->regs[R_OUT_Y], s->regs[R_OUT_C], w, h);
    }
    row = g_malloc(2 * w);
    for (y = 0; y < h; y++) {
        const uint8_t *p = rk_vdpu2_row(&img, 0, y);

        for (x = 0; x < w; x++) {
            row[x] = p[MIN(x, img.pw[0] - 1)];
        }
        rk_vdpu2_wr(s, s->regs[R_OUT_Y] + y * w, row, w);
    }
    if (img.ncomp == 3) {
        /* interleaved Cb Cr at the JPEG's sampling */
        cw = w * img.hs[1] / img.hmax;
        ch = h * img.vs[1] / img.vmax;
        for (y = 0; y < ch; y++) {
            const uint8_t *cb = rk_vdpu2_row(&img, 1, y);
            const uint8_t *cr = rk_vdpu2_row(&img, 2, y);

            for (x = 0; x < cw; x++) {
                row[2 * x] = cb[MIN(x, img.pw[1] - 1)];
                row[2 * x + 1] = cr[MIN(x, img.pw[2] - 1)];
            }
            rk_vdpu2_wr(s, s->regs[R_OUT_C] + y * 2 * cw, row, 2 * cw);
        }
    }
    jpeg_dec_free(&img);
    return INT_IRQ | INT_RDY;
}

static void rk_vdpu2_done(void *opaque)
{
    RkVdpu2State *s = opaque;

    s->regs[R_EN] &= ~EN_DEC;
    s->regs[R_INT] = (s->regs[R_INT] & INT_DIS) | s->result;
    rk_vdpu2_update(s);
}

static void rk_vdpu2_start(RkVdpu2State *s)
{
    if ((s->regs[R_MODE] & 0xf) == MODE_JPEG) {
        s->result = rk_vdpu2_jpeg(s);
    } else {
        qemu_log_mask(LOG_UNIMP, "rk-vdpu2: decode mode %u\n",
                      s->regs[R_MODE] & 0xf);
        s->result = INT_IRQ | INT_ERR;
    }
    timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + JOB_NS);
}

static uint64_t rk_vdpu2_read(void *opaque, hwaddr off, unsigned size)
{
    RkVdpu2State *s = opaque;

    return s->regs[off / 4];
}

static void rk_vdpu2_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    RkVdpu2State *s = opaque;

    s->regs[off / 4] = val;
    if (off / 4 == R_EN && (val & EN_DEC)) {
        rk_vdpu2_start(s);
    }
    if (off / 4 == R_INT) {
        rk_vdpu2_update(s);
    }
}

static const MemoryRegionOps rk_vdpu2_ops = {
    .read = rk_vdpu2_read,
    .write = rk_vdpu2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_vdpu2_reset(DeviceState *dev)
{
    RkVdpu2State *s = RK_VDPU2(dev);

    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->done);
    rk_vdpu2_update(s);
}

static void rk_vdpu2_init(Object *obj)
{
    RkVdpu2State *s = RK_VDPU2(obj);

    memory_region_init_io(&s->iomem, obj, &rk_vdpu2_ops, s, TYPE_RK_VDPU2,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_vdpu2_done, s);
}

static const Property rk_vdpu2_properties[] = {
    DEFINE_PROP_LINK("iommu", RkVdpu2State, iommu, "rk-iommu", Object *),
    DEFINE_PROP_BOOL("trace", RkVdpu2State, trace, false),
};

static const VMStateDescription rk_vdpu2_vmstate = {
    .name = TYPE_RK_VDPU2,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkVdpu2State, REGS_SIZE / 4),
        VMSTATE_UINT32(result, RkVdpu2State),
        VMSTATE_TIMER_PTR(done, RkVdpu2State),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_vdpu2_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_vdpu2_reset);
    device_class_set_props(dc, rk_vdpu2_properties);
    dc->vmsd = &rk_vdpu2_vmstate;
}

static const TypeInfo rk_vdpu2_info = {
    .name          = TYPE_RK_VDPU2,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkVdpu2State),
    .instance_init = rk_vdpu2_init,
    .class_init    = rk_vdpu2_class_init,
};

static void rk_vdpu2_register_types(void)
{
    type_register_static(&rk_vdpu2_info);
}

type_init(rk_vdpu2_register_types)
