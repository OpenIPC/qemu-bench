/*
 * Rockchip VEPU2 ("rockchip,vpu-encoder-v2"; RV1126 / RV1109 0xFFB90000,
 * SPI 74, its IOMMU at +0x800), the Hantro-derived encoder the RV1126's
 * MPP uses for JPEG (hal_jpege_vepu2_v2.c; the kernel's mpp_vepu2.c), as
 * far as JPEG goes.  Registers by word index (offset = 4 x index):
 *
 *   48 / 49 / 50  source Y / U (or interleaved UV) / V
 *   53            in: room in the stream buffer; out: bits written,
 *                 counted from the 8-byte word the stream starts in
 *   60            bits 21:16 the start's bit offset in that word, 7:4
 *                 x fill (4-pixel units), 3:0 y fill (lines)
 *   61            source stride, pixels
 *   74            bits 7:4 input format (0 I420, 1 NV12, 2 YUYV, 3 UYVY)
 *   77            stream output: just past the JPEG header the HAL wrote
 *   103 ENC_EN    bit 0 start, 5:4 mode (2 JPEG), 15:8 MCU columns,
 *                 29:20 MCU rows (before the fills)
 *   109 INT       bit 0 raw interrupt, 1 frame ready; the kernel writes 0
 *
 * The HAL writes SOI..SOS before the output address; the model takes the
 * quantisers from that header's DQT (scanning back up to HDR_SCAN bytes)
 * and writes the scan and EOI with the shared JPEG encoder (jpeg-enc.c).
 * It runs JOB_NS after the start, at once (the SDK encodes JPEG only for
 * snapshots).  H.264 and the other modes are not done.  Everything else
 * is storage.
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
#include "jpeg-enc.h"

#define TYPE_RK_VEPU2 "rk-vepu2"
OBJECT_DECLARE_SIMPLE_TYPE(RkVepu2State, RK_VEPU2)

#define REGS_SIZE       0x400
#define W_IN_Y          48
#define W_IN_U          49
#define W_IN_V          50
#define W_BITS          53
#define W_FILL          60
#define W_STRIDE        61
#define W_FMT           74
#define W_OUT           77
#define W_ENC_EN        103
#define W_INT           109

#define INT_RAW         (1u << 0)
#define INT_RDY         (1u << 1)
#define JOB_NS          2000000
#define HDR_SCAN        1024

struct RkVepu2State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;
    uint32_t regs[REGS_SIZE / 4];
    Object *iommu;
};

static const uint8_t rk_zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

static void rk_vepu2_update(RkVepu2State *s)
{
    qemu_set_irq(s->irq, s->regs[W_INT] & INT_RAW);
}

static void rk_vepu2_rd(RkVepu2State *s, uint32_t a, void *b, uint32_t n)
{
    if (s->iommu) {
        rk_iommu_dma_read(s->iommu, a, b, n);
    } else {
        dma_memory_read(&address_space_memory, a, b, n,
                        MEMTXATTRS_UNSPECIFIED);
    }
}

static void rk_vepu2_wr(RkVepu2State *s, uint32_t a, const void *b,
                        uint32_t n)
{
    if (s->iommu) {
        rk_iommu_dma_write(s->iommu, a, b, n);
    } else {
        dma_memory_write(&address_space_memory, a, b, n,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

/* The DQT of the header the HAL wrote before the output address */
static bool rk_vepu2_dqt(RkVepu2State *s, uint8_t ql[64], uint8_t qc[64])
{
    uint32_t out = s->regs[W_OUT], back = MIN(HDR_SCAN, out);
    g_autofree uint8_t *b = NULL;
    uint32_t i, k, got = 0;

    if (back < 4) {
        return false;
    }
    b = g_malloc(back);
    rk_vepu2_rd(s, out - back, b, back);
    for (i = back - 2; i > 0 && !(b[i] == 0xff && b[i + 1] == 0xd8); i--) {
        continue;
    }
    if (b[i] != 0xff || b[i + 1] != 0xd8) {
        return false;
    }
    for (i += 2; i + 4 <= back && b[i] == 0xff; ) {
        uint32_t len = b[i + 2] << 8 | b[i + 3], end = i + 2 + len;

        if (end > back) {
            break;
        }
        if (b[i + 1] == 0xdb) {
            for (k = i + 4; k + 65 <= end; k += 65) {
                uint8_t *q = (b[k] & 0xf) ? qc : ql;
                int j;

                if (b[k] >> 4) {
                    return false;               /* 16-bit tables */
                }
                for (j = 0; j < 64; j++) {
                    q[rk_zigzag[j]] = b[k + 1 + j];
                }
                got |= (b[k] & 0xf) ? 2 : 1;
            }
        }
        if (b[i + 1] == 0xda) {
            break;
        }
        i = end;
    }
    if (got == 1) {
        memcpy(qc, ql, 64);
    }
    return got != 0;
}

static void rk_vepu2_run(RkVepu2State *s)
{
    uint32_t en = s->regs[W_ENC_EN], fill = s->regs[W_FILL];
    int w = ((en >> 8) & 0xff) * 16 - ((fill >> 4) & 0xf) * 4;
    int h = ((en >> 20) & 0x3ff) * 16 - (fill & 0xf);
    int ys = s->regs[W_STRIDE] & 0x7fff, fmt = (s->regs[W_FMT] >> 4) & 0xf;
    int bitoff = (fill >> 16) & 0x3f, x, y;
    uint8_t ql[64], qc[64];
    g_autofree uint8_t *py = NULL, *pu = NULL, *pv = NULL, *raw = NULL,
                       *out = NULL;
    size_t cap = s->regs[W_BITS], n;
    bool overflow = false;

    if (((en >> 4) & 3) != 2 || w <= 0 || h <= 0 || w > 8192 ||
        h > 8192 || !rk_vepu2_dqt(s, ql, qc)) {
        qemu_log_mask(LOG_UNIMP, "rk-vepu2: not a JPEG job we do (0x%x)\n",
                      en);
        s->regs[W_BITS] = bitoff;
        return;
    }
    ys = MAX(ys, w);
    py = g_malloc((size_t)w * h);
    pu = g_malloc((size_t)(w / 2) * (h / 2) + 1);
    pv = g_malloc((size_t)(w / 2) * (h / 2) + 1);
    if (fmt == 2 || fmt == 3) {                 /* packed 4:2:2 */
        raw = g_malloc((size_t)ys * 2);
        for (y = 0; y < h; y++) {
            rk_vepu2_rd(s, s->regs[W_IN_Y] + y * ys * 2, raw, ys * 2);
            for (x = 0; x < w; x++) {
                const uint8_t *q = raw + (x & ~1) * 2;

                py[y * w + x] = fmt == 2 ? q[(x & 1) * 2] : q[(x & 1) * 2 + 1];
                if (!(y & 1) && !(x & 1)) {
                    pu[(y / 2) * (w / 2) + x / 2] = fmt == 2 ? q[1] : q[0];
                    pv[(y / 2) * (w / 2) + x / 2] = fmt == 2 ? q[3] : q[2];
                }
            }
        }
    } else {
        raw = g_malloc(ys);
        for (y = 0; y < h; y++) {
            rk_vepu2_rd(s, s->regs[W_IN_Y] + y * ys, py + y * w, w);
        }
        for (y = 0; y < h / 2; y++) {
            if (fmt == 1) {                     /* NV12 */
                rk_vepu2_rd(s, s->regs[W_IN_U] + y * ys, raw, w);
                for (x = 0; x < w / 2; x++) {
                    pu[y * (w / 2) + x] = raw[2 * x];
                    pv[y * (w / 2) + x] = raw[2 * x + 1];
                }
            } else {                            /* I420 */
                rk_vepu2_rd(s, s->regs[W_IN_U] + y * (ys / 2),
                            pu + y * (w / 2), w / 2);
                rk_vepu2_rd(s, s->regs[W_IN_V] + y * (ys / 2),
                            pv + y * (w / 2), w / 2);
            }
        }
    }
    cap = MIN(cap, 64u << 20);
    out = g_malloc(cap + 1);
    n = jpeg_enc_scan(py, w, pu, pv, w / 2, w, h, ql, qc, true, out, cap,
                      &overflow);
    rk_vepu2_wr(s, s->regs[W_OUT], out, n);
    s->regs[W_BITS] = bitoff + n * 8;
}

static void rk_vepu2_done(void *opaque)
{
    RkVepu2State *s = opaque;

    rk_vepu2_run(s);
    s->regs[W_ENC_EN] &= ~1u;
    s->regs[W_INT] |= INT_RAW | INT_RDY;
    rk_vepu2_update(s);
}

static uint64_t rk_vepu2_read(void *opaque, hwaddr off, unsigned size)
{
    return RK_VEPU2(opaque)->regs[off / 4];
}

static void rk_vepu2_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    RkVepu2State *s = opaque;

    s->regs[off / 4] = val;
    if (off / 4 == W_ENC_EN && (val & 1)) {
        timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + JOB_NS);
    }
    if (off / 4 == W_INT) {
        rk_vepu2_update(s);
    }
}

static const MemoryRegionOps rk_vepu2_ops = {
    .read = rk_vepu2_read,
    .write = rk_vepu2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_vepu2_reset(DeviceState *dev)
{
    RkVepu2State *s = RK_VEPU2(dev);

    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->done);
    rk_vepu2_update(s);
}

static void rk_vepu2_init(Object *obj)
{
    RkVepu2State *s = RK_VEPU2(obj);

    memory_region_init_io(&s->iomem, obj, &rk_vepu2_ops, s, TYPE_RK_VEPU2,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_vepu2_done, s);
}

static const Property rk_vepu2_properties[] = {
    DEFINE_PROP_LINK("iommu", RkVepu2State, iommu, "rk-iommu", Object *),
};

static const VMStateDescription rk_vepu2_vmstate = {
    .name = TYPE_RK_VEPU2,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkVepu2State, REGS_SIZE / 4),
        VMSTATE_TIMER_PTR(done, RkVepu2State),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_vepu2_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_vepu2_reset);
    device_class_set_props(dc, rk_vepu2_properties);
    dc->vmsd = &rk_vepu2_vmstate;
}

static const TypeInfo rk_vepu2_info = {
    .name = TYPE_RK_VEPU2,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkVepu2State),
    .instance_init = rk_vepu2_init,
    .class_init = rk_vepu2_class_init,
};

static void rk_vepu2_register_types(void)
{
    type_register_static(&rk_vepu2_info);
}

type_init(rk_vepu2_register_types)
