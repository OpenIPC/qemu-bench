/*
 * Rockchip RV1126 / RV1109 ISPP ("rockchip,rv1126-rkispp", 0xFFB60000),
 * the image post-processor after the ISP20: temporal noise reduction
 * (TNR), noise reduction and sharpening (NR / SHP), the three scalers and
 * the fisheye corrector (FEC), as the vendor driver (develop-4.19
 * drivers/media/platform/rockchip/ispp: hw.c, stream.c, regs.h) runs them.
 *
 * The driver starts each stage per frame by writing CTRL_STRT (0x000):
 * bit 0 TNR, bit 1 NR + SHP (and the scalers behind them), bit 2 FEC.  A
 * stage ends STAGE_NS later with its interrupt, in CTRL_INT_RAW_STA
 * (0x024; 0x028 INT_STA is RAW & INT_MSK 0x020, 0x02c sets, 0x030
 * clears): TNR_INT (bit 2); NR_INT | SHP_INT (bits 3, 4) and SCLn_INT
 * (bit 8 + n) for each scaler enabled (SCLn_CTRL bit 0); FEC_INT
 * (bit 7).  CTRL_SYS_STATUS (0x03c) shows the stages still working.
 *
 * What the stages write is drawn, not computed: the frames from the ISP
 * are only the bridge's buffers here (rk-isp32.c, "readback"), so each
 * output is the shared scene (rk-scene.c) at the sensor's exposure
 * ("sensor" link), drawn at the output's own size:
 *   NR / SHP  the main bypass picture at SHARP_WR_Y / UV_BASE (0x608 /
 *             0x60c), CTRL_SIZE (0x014: height << 16 | width), unless the
 *             write DMA is off (SHARP_CORE_CTRL 0x680 bit 2) or the
 *             format FBC (an internal buffer); format SHARP_CTRL bits 3:0
 *   SCLn      at 0x800 + 0x100 n: CTRL (bit 0 enable, 1 bypass, 3 no UV,
 *             format bits 6:4), CUR_Y / UV_BASE (+0x08 / +0x0c), the size
 *             from the input's by FACTOR (+0x18: v << 16 | h, out - 1 =
 *             (factor - 1) (in - 1) / 8192)
 *   FEC       FEC_WR_Y / UV_BASE (0xc24 / 0xc28), FEC_PIC_SIZE (0xc88),
 *             unless FEC_CORE_CTRL (0xc80) bit 1 (no DDR write); format
 *             FEC_CTRL bits 7:4
 * Formats: FBC (bit 0; nothing is written), 4:2:2 (bit 1, else 4:2:0),
 * packed YUYV (bit 2), Y/C swap (bit 3: Y first in packed, CrCb in
 * semi-planar).  Strides are the VIR_STRIDE registers, in words; limited
 * or full range from each path's YUV_LIMIT bit.  The writes go through
 * the ISPP's IOMMU ("iommu" link, rk-iommu.c).  CTRL_UPDATE (0x004) and a
 * stage's start latch the address shadows.
 *
 * Everything else is storage.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
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
#include "hw/misc/rk-scene.h"

#define TYPE_RK_ISPP "rk-ispp"
OBJECT_DECLARE_SIMPLE_TYPE(RkIsppState, RK_ISPP)

#define REGS_SIZE       0x1000
#define R_STRT          0x000
#define R_UPDATE        0x004
#define R_SIZE          0x014
#define R_INT_MSK       0x020
#define R_INT_RAW       0x024
#define R_INT_STA       0x028
#define R_INT_SET       0x02c
#define R_INT_CLR       0x030
#define R_SYS_STATUS    0x03c
#define R_TNR_WR_Y      0x244
#define R_TNR_WR_UV     0x248
#define R_TNR_WR_Y_SHD  0x274
#define R_TNR_WR_UV_SHD 0x278
#define R_NR_Y          0x408
#define R_NR_UV         0x40c
#define R_NR_Y_SHD      0x424
#define R_NR_UV_SHD     0x428
#define R_SHP_CTRL      0x600
#define R_SHP_STRIDE    0x604
#define R_SHP_WR_Y      0x608
#define R_SHP_WR_UV     0x60c
#define R_SHP_WR_Y_SHD  0x624
#define R_SHP_WR_UV_SHD 0x628
#define R_SHP_CORE      0x680
#define R_SCL(n)        (0x800 + 0x100 * (n))
#define SCL_STRIDE      0x04
#define SCL_Y           0x08
#define SCL_UV          0x0c
#define SCL_Y_SHD       0x10
#define SCL_UV_SHD      0x14
#define SCL_FACTOR      0x18
#define R_FEC_CTRL      0xc00
#define R_FEC_STRIDE    0xc20
#define R_FEC_WR_Y      0xc24
#define R_FEC_WR_UV     0xc28
#define R_FEC_WR_Y_SHD  0xc44
#define R_FEC_WR_UV_SHD 0xc48
#define R_FEC_CORE      0xc80
#define R_FEC_PIC_SIZE  0xc88

#define ST_TNR          (1u << 0)
#define ST_NR_SHP       (1u << 1)
#define ST_FEC          (1u << 2)
#define TNR_INT         (1u << 2)
#define NR_INT          (1u << 3)
#define SHP_INT         (1u << 4)
#define FEC_INT         (1u << 7)
#define SCL_INT(n)      (1u << (8 + (n)))
#define TNR_WORKING     (1u << 0)
#define NR_WORKING      (1u << 1)
#define SHP_WORKING     (1u << 2)
#define SCL_WORKING(n)  (1u << (4 + (n)))
#define FEC_WORKING     (1u << 7)

#define FMT_FBC         (1u << 0)
#define FMT_422         (1u << 1)
#define FMT_YUYV        (1u << 2)
#define FMT_SWAP        (1u << 3)
#define SHP_LIMIT       (1u << 4)
#define SHP_DMA_DIS     (1u << 2)
#define SCL_ENABLE      (1u << 0)
#define SCL_BYPASS      (1u << 1)
#define SCL_UV_DIS      (1u << 3)
#define SCL_LIMIT       (1u << 7)
#define SCL_EN_SHD      (1u << 30)
#define SCL_BYPASS_SHD  (1u << 31)
#define FEC_LIMIT       (1u << 8)
#define FEC2DDR_DIS     (1u << 1)

#define STAGE_NS        3000000     /* a stage over a frame */
#define BLK             8           /* the scene is flat at this scale */

struct RkIsppState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq, irq_fec;
    QEMUTimer *timer;

    uint32_t regs[REGS_SIZE / 4];
    uint32_t pending;           /* stages started and not yet done */
    uint32_t frame;             /* NR stages run: moves the picture */
    Object *sensor;
    Object *iommu;
};

#define REG(s, off) ((s)->regs[(off) / 4])

static void rk_ispp_update(RkIsppState *s)
{
    REG(s, R_INT_STA) = REG(s, R_INT_RAW) & REG(s, R_INT_MSK);
    qemu_set_irq(s->irq, !!REG(s, R_INT_STA));
}

static void rk_ispp_wr(RkIsppState *s, uint32_t a, const void *buf,
                       uint32_t len)
{
    if (s->iommu) {
        rk_iommu_dma_write(s->iommu, a, buf, len);
    } else {
        dma_memory_write(&address_space_memory, a, buf, len,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

/*
 * The scene, w x h, into a picture at y_base / c_base, lines @stride
 * bytes apart, in format @fmt (FMT_*); no chroma with @no_uv.
 */
static void rk_ispp_draw(RkIsppState *s, uint32_t y_base, uint32_t c_base,
                         uint32_t w, uint32_t h, uint32_t stride,
                         uint32_t fmt, bool limited, bool no_uv)
{
    bool packed = fmt & FMT_YUYV, swap = fmt & FMT_SWAP;
    uint32_t bw = DIV_ROUND_UP(w, BLK), line = packed ? 2 * w : w;
    double k = rk_scene_scale(s->sensor);
    g_autofree uint8_t *yuv = NULL, *yl = NULL, *cl = NULL;
    uint32_t x, y, by, bx;
    int c;

    /* behind the IOMMU, 0 is an IOVA like any other */
    if ((fmt & FMT_FBC) || (!y_base && !s->iommu) || !w || !h ||
        w > 8192 || h > 8192) {
        return;
    }
    stride = MAX(stride, line);
    yuv = g_new(uint8_t, bw * 3);
    yl = g_new0(uint8_t, stride + 4);
    cl = g_new0(uint8_t, stride + 4);
    for (by = 0; by * BLK < h; by++) {
        for (bx = 0; bx < bw; bx++) {
            double rgb[3], lvl[3];

            rk_scene_rgb(s->frame, (bx * BLK + BLK / 2.0) / w,
                         (by * BLK + BLK / 2.0) / h, rgb);
            for (c = 0; c < 3; c++) {
                lvl[c] = MIN(rgb[c] * k, RK_SCENE_LEVEL_MAX);
            }
            rk_scene_yuv(lvl, limited, &yuv[bx * 3]);
        }
        for (x = 0; x < w; x += 2) {
            const uint8_t *p = &yuv[(x / BLK) * 3];
            uint8_t cb = no_uv ? 128 : p[1], cr = no_uv ? 128 : p[2];

            if (packed) {               /* YUYV with swap, else UYVY */
                uint8_t *q = &yl[2 * x];

                q[swap ? 0 : 1] = p[0];
                q[swap ? 2 : 3] = p[0];
                q[swap ? 1 : 0] = cb;
                q[swap ? 3 : 2] = cr;
            } else {
                yl[x] = p[0];
                if (x + 1 < w) {
                    yl[x + 1] = p[0];
                }
                cl[x] = swap ? cr : cb;
                cl[x + 1] = swap ? cb : cr;
            }
        }
        for (y = by * BLK; y < MIN(h, (by + 1) * BLK); y++) {
            rk_ispp_wr(s, y_base + y * stride, yl, line);
            if (packed || no_uv || !c_base) {
                continue;
            }
            if (fmt & FMT_422) {
                rk_ispp_wr(s, c_base + y * stride, cl, w);
            } else if (!(y & 1)) {
                rk_ispp_wr(s, c_base + (y / 2) * stride, cl, w);
            }
        }
    }
}

/* out - 1 = (factor - 1)(in - 1) / 8192, rounded up as the driver's inverse */
static uint32_t rk_ispp_scaled(uint32_t in, uint32_t fac)
{
    if (!in || fac <= 1) {
        return in;
    }
    return ((uint64_t)(fac - 1) * (in - 1) + 8191) / 8192 + 1;
}

static void rk_ispp_latch(RkIsppState *s)
{
    int n;

    REG(s, R_TNR_WR_Y_SHD) = REG(s, R_TNR_WR_Y);
    REG(s, R_TNR_WR_UV_SHD) = REG(s, R_TNR_WR_UV);
    REG(s, R_NR_Y_SHD) = REG(s, R_NR_Y);
    REG(s, R_NR_UV_SHD) = REG(s, R_NR_UV);
    REG(s, R_SHP_WR_Y_SHD) = REG(s, R_SHP_WR_Y);
    REG(s, R_SHP_WR_UV_SHD) = REG(s, R_SHP_WR_UV);
    REG(s, R_FEC_WR_Y_SHD) = REG(s, R_FEC_WR_Y);
    REG(s, R_FEC_WR_UV_SHD) = REG(s, R_FEC_WR_UV);
    for (n = 0; n < 3; n++) {
        uint32_t c = REG(s, R_SCL(n));

        REG(s, R_SCL(n) + SCL_Y_SHD) = REG(s, R_SCL(n) + SCL_Y);
        REG(s, R_SCL(n) + SCL_UV_SHD) = REG(s, R_SCL(n) + SCL_UV);
        REG(s, R_SCL(n)) = (c & ~(SCL_EN_SHD | SCL_BYPASS_SHD)) |
            ((c & SCL_ENABLE) ? SCL_EN_SHD : 0) |
            ((c & SCL_BYPASS) ? SCL_BYPASS_SHD : 0);
    }
}

/* NR + SHP: the main picture, and each scaler's from it */
static uint32_t rk_ispp_nr(RkIsppState *s)
{
    uint32_t in_w = REG(s, R_SIZE) & 0xffff, in_h = REG(s, R_SIZE) >> 16;
    uint32_t ctrl = REG(s, R_SHP_CTRL), irq = NR_INT | SHP_INT;
    int n;

    if (!(REG(s, R_SHP_CORE) & SHP_DMA_DIS)) {
        rk_ispp_draw(s, REG(s, R_SHP_WR_Y), REG(s, R_SHP_WR_UV), in_w, in_h,
                     REG(s, R_SHP_STRIDE) * 4, ctrl & 0xf,
                     ctrl & SHP_LIMIT, false);
    }
    for (n = 0; n < 3; n++) {
        hwaddr b = R_SCL(n);
        uint32_t c = REG(s, b), f = REG(s, b + SCL_FACTOR);
        bool byp = c & SCL_BYPASS;

        if (!(c & SCL_ENABLE)) {
            continue;
        }
        rk_ispp_draw(s, REG(s, b + SCL_Y), REG(s, b + SCL_UV),
                     byp ? in_w : rk_ispp_scaled(in_w, f & 0xffff),
                     byp ? in_h : rk_ispp_scaled(in_h, f >> 16),
                     REG(s, b + SCL_STRIDE) * 4, (c >> 3) & 0xe,
                     c & SCL_LIMIT, c & SCL_UV_DIS);
        irq |= SCL_INT(n);
    }
    s->frame++;
    return irq;
}

static uint32_t rk_ispp_fec(RkIsppState *s)
{
    uint32_t ctrl = REG(s, R_FEC_CTRL), size = REG(s, R_FEC_PIC_SIZE);

    if (!(REG(s, R_FEC_CORE) & FEC2DDR_DIS)) {
        rk_ispp_draw(s, REG(s, R_FEC_WR_Y), REG(s, R_FEC_WR_UV),
                     size & 0xffff, size >> 16, REG(s, R_FEC_STRIDE) * 4,
                     (ctrl >> 4) & 0xf, ctrl & FEC_LIMIT, false);
    }
    return FEC_INT;
}

static void rk_ispp_tick(void *opaque)
{
    RkIsppState *s = opaque;
    uint32_t irq = 0;

    if (s->pending & ST_TNR) {
        irq |= TNR_INT;
    }
    if (s->pending & ST_NR_SHP) {
        irq |= rk_ispp_nr(s);
    }
    if (s->pending & ST_FEC) {
        irq |= rk_ispp_fec(s);
    }
    s->pending = 0;
    REG(s, R_SYS_STATUS) = 0;
    REG(s, R_INT_RAW) |= irq;
    rk_ispp_update(s);
}

static void rk_ispp_start(RkIsppState *s, uint32_t st)
{
    int n;

    st &= ST_TNR | ST_NR_SHP | ST_FEC;
    if (!st) {
        return;
    }
    rk_ispp_latch(s);
    s->pending |= st;
    if (st & ST_TNR) {
        REG(s, R_SYS_STATUS) |= TNR_WORKING;
    }
    if (st & ST_NR_SHP) {
        REG(s, R_SYS_STATUS) |= NR_WORKING | SHP_WORKING;
        for (n = 0; n < 3; n++) {
            if (REG(s, R_SCL(n)) & SCL_ENABLE) {
                REG(s, R_SYS_STATUS) |= SCL_WORKING(n);
            }
        }
    }
    if (st & ST_FEC) {
        REG(s, R_SYS_STATUS) |= FEC_WORKING;
    }
    if (!timer_pending(s->timer)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + STAGE_NS);
    }
}

static uint64_t rk_ispp_read(void *opaque, hwaddr off, unsigned size)
{
    RkIsppState *s = opaque;

    switch (off) {
    case R_STRT:
    case R_UPDATE:
    case R_INT_SET:
    case R_INT_CLR:
        return 0;
    }
    return s->regs[off / 4];
}

static void rk_ispp_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    RkIsppState *s = opaque;

    switch (off) {
    case R_STRT:
        rk_ispp_start(s, val);
        return;
    case R_UPDATE:
        rk_ispp_latch(s);
        return;
    case R_INT_RAW:
    case R_INT_STA:
    case R_SYS_STATUS:
        return;                         /* read-only */
    case R_INT_SET:
        REG(s, R_INT_RAW) |= val;
        break;
    case R_INT_CLR:
        REG(s, R_INT_RAW) &= ~val;
        break;
    default:
        s->regs[off / 4] = val;
        if (off != R_INT_MSK) {
            return;
        }
    }
    rk_ispp_update(s);
}

static const MemoryRegionOps rk_ispp_ops = {
    .read = rk_ispp_read,
    .write = rk_ispp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_ispp_reset(DeviceState *dev)
{
    RkIsppState *s = RK_ISPP(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pending = 0;
    s->frame = 0;
    timer_del(s->timer);
    rk_ispp_update(s);
}

static void rk_ispp_init(Object *obj)
{
    RkIsppState *s = RK_ISPP(obj);

    memory_region_init_io(&s->iomem, obj, &rk_ispp_ops, s, TYPE_RK_ISPP,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq_fec);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_ispp_tick, s);
}

static const Property rk_ispp_properties[] = {
    DEFINE_PROP_LINK("sensor", RkIsppState, sensor, TYPE_DEVICE, Object *),
    DEFINE_PROP_LINK("iommu", RkIsppState, iommu, "rk-iommu", Object *),
};

static const VMStateDescription rk_ispp_vmstate = {
    .name = TYPE_RK_ISPP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkIsppState, REGS_SIZE / 4),
        VMSTATE_UINT32(pending, RkIsppState),
        VMSTATE_UINT32(frame, RkIsppState),
        VMSTATE_TIMER_PTR(timer, RkIsppState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_ispp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_ispp_reset);
    device_class_set_props(dc, rk_ispp_properties);
    dc->vmsd = &rk_ispp_vmstate;
}

static const TypeInfo rk_ispp_info = {
    .name = TYPE_RK_ISPP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkIsppState),
    .instance_init = rk_ispp_init,
    .class_init = rk_ispp_class_init,
};

static void rk_ispp_register_types(void)
{
    type_register_static(&rk_ispp_info);
}

type_init(rk_ispp_register_types)
