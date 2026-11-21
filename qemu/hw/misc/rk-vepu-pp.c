/*
 * Rockchip RKVENC-PP ("rockchip,rkvenc-pp-rv1106", 0xFFA60000), the
 * encoder's pre-processor: smear, weighted prediction, motion and
 * occlusion detection for the smart features (rockit's IVS).  As the
 * kernel driver drives it (kmpp mpp/mpp_rkvenc_pp.c, public in
 * ticklab/rockchip-kmpp):
 *
 *   0x010 ENC_STRT   1 starts the job configured below
 *   0x020 INT_EN  0x024 INT_MSK  0x028 INT_CLR (write 1 to clear)
 *   0x02c INT_STA    bit 0 the job done (the driver's
 *                    PP_REG_RO_VALID_INT_STS), 4:2 bus errors
 *   0x034 ENC_PIC_FMT  0x038 ENC_PIC_RSL
 *   0x03c..0x054 reference / statistics buffers, 0x100.. the source
 *   0x200 smear, 0x400 motion detection, 0x500 occlusion detection,
 *   0x800.. results (read back by the driver)
 *
 * A job is done "job-ns" after it starts: INT_STA bit 0, the interrupt
 * while enabled and not masked.
 *
 * Motion detection is computed.  The job's picture is ENC_PIC_RSL
 * ((width / 8 - 1) | (height / 8 - 1) << 16), its luma at 0x10c with
 * the stride at 0x118.  The hardware works on it downscaled 4 x 4; each
 * 8 x 8 block there (32 x 32 of the picture) is compared with the same
 * block of the previous picture, and its flag set when the SAD exceeds
 * MD_CON_BASE (0x404) bits 15:4 (thres_sad; bit 0 this picture, bit 1
 * against a reference).  The flags go to 0x400 (adr_md_base), a bit a
 * block, MD_STO_STRD (0x40c, 64) bytes a block row: block x of row y is
 * bit x % 8 of byte y * stride + x / 8 -- what the RV1103B kernel's
 * vepu_pp_get_blk8_move_flag() reads, and rockit's IVS on the RV1106.
 * The scene's moving box shows as motion: XM's motion alarm and its
 * event recording see it.  Smear, weighted prediction and occlusion
 * results stay as they were (nothing covered).  Everything else is
 * storage.
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
#include "qom/object.h"
#include "system/address-spaces.h"

#define TYPE_RK_VEPU_PP "rk-vepu-pp"
OBJECT_DECLARE_SIMPLE_TYPE(RkVepuPpState, RK_VEPU_PP)

#define REGS_SIZE       0x1000
#define R_ENC_STRT      0x010
#define R_INT_EN        0x020
#define R_INT_MSK       0x024
#define R_INT_CLR       0x028
#define R_INT_STA       0x02c
#define INT_DONE        (1u << 0)
#define R_PIC_RSL       0x038
#define R_SRC0          0x10c
#define R_SRC_STRD0     0x118
#define R_MD_BASE       0x400
#define R_MD_CON        0x404
#define R_MD_STO_STRD   0x40c
#define MD_CUR_EN       (1u << 0)
#define MD_REF_EN       (1u << 1)
#define MD_MAX_W        4096        /* pixels */
#define MD_MAX_H        4096

struct RkVepuPpState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;

    uint32_t regs[REGS_SIZE / 4];
    uint32_t job_ns;            /* property: a job's time */
    uint8_t *md_ref;            /* the last picture, downscaled 4 x 4 */
    uint32_t md_ref_w, md_ref_h;
};

#define REG(s, off) ((s)->regs[(off) / 4])

static void rk_vepu_pp_update(RkVepuPpState *s)
{
    qemu_set_irq(s->irq, !!(REG(s, R_INT_STA) & REG(s, R_INT_EN) &
                            ~REG(s, R_INT_MSK)));
}

static void rk_vepu_pp_done(void *opaque)
{
    RkVepuPpState *s = opaque;

    REG(s, R_INT_STA) |= INT_DONE;
    rk_vepu_pp_update(s);
}

/* The job's motion flags into adr_md_base, against the last picture */
static void rk_vepu_pp_motion(RkVepuPpState *s)
{
    uint32_t rsl = REG(s, R_PIC_RSL), con = REG(s, R_MD_CON);
    uint32_t w = ((rsl & 0x7ff) + 1) * 8, h = (((rsl >> 16) & 0x7ff) + 1) * 8;
    uint32_t dw = w / 4, dh = h / 4, stride = REG(s, R_SRC_STRD0) & 0x1ffff;
    uint32_t mstride = REG(s, R_MD_STO_STRD), thres = (con >> 4) & 0xfff;
    uint32_t bw = DIV_ROUND_UP(w, 32), bh = DIV_ROUND_UP(h, 32);
    g_autofree uint8_t *cur = NULL, *line = NULL, *flags = NULL;
    bool ref;
    uint32_t x, y, i, j;

    if (!(con & MD_CUR_EN) || w > MD_MAX_W || h > MD_MAX_H || !stride ||
        !mstride || bw > mstride * 8) {
        return;
    }
    cur = g_malloc(dw * dh);
    line = g_malloc(4 * stride);
    for (y = 0; y < dh; y++) {
        address_space_read(&address_space_memory,
                           REG(s, R_SRC0) + (hwaddr)y * 4 * stride,
                           MEMTXATTRS_UNSPECIFIED, line, 4 * stride);
        for (x = 0; x < dw; x++) {
            uint32_t sum = 0;

            for (j = 0; j < 4; j++) {
                for (i = 0; i < 4; i++) {
                    sum += line[j * stride + x * 4 + i];
                }
            }
            cur[y * dw + x] = sum / 16;
        }
    }
    ref = (con & MD_REF_EN) && s->md_ref && s->md_ref_w == dw &&
          s->md_ref_h == dh;
    flags = g_malloc0(bh * mstride);
    for (y = 0; ref && y < bh; y++) {
        for (x = 0; x < bw; x++) {
            uint32_t sad = 0;

            for (j = y * 8; j < MIN(y * 8 + 8, dh); j++) {
                for (i = x * 8; i < MIN(x * 8 + 8, dw); i++) {
                    sad += abs(cur[j * dw + i] - s->md_ref[j * dw + i]);
                }
            }
            if (sad > thres) {
                flags[y * mstride + x / 8] |= 1u << (x % 8);
            }
        }
    }
    address_space_write(&address_space_memory, REG(s, R_MD_BASE),
                        MEMTXATTRS_UNSPECIFIED, flags, bh * mstride);
    g_free(s->md_ref);
    s->md_ref = g_steal_pointer(&cur);
    s->md_ref_w = dw;
    s->md_ref_h = dh;
}

static uint64_t rk_vepu_pp_read(void *opaque, hwaddr off, unsigned size)
{
    RkVepuPpState *s = opaque;

    return s->regs[off / 4];
}

static void rk_vepu_pp_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    RkVepuPpState *s = opaque;

    switch (off) {
    case R_INT_CLR:
        REG(s, R_INT_STA) &= ~val;
        rk_vepu_pp_update(s);
        return;
    case R_INT_STA:
        return;                     /* read-only */
    case R_ENC_STRT:
        if (val & 1) {
            rk_vepu_pp_motion(s);
            timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      s->job_ns);
        }
        return;
    }
    s->regs[off / 4] = val;
    rk_vepu_pp_update(s);
}

static const MemoryRegionOps rk_vepu_pp_ops = {
    .read = rk_vepu_pp_read,
    .write = rk_vepu_pp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_vepu_pp_reset(DeviceState *dev)
{
    RkVepuPpState *s = RK_VEPU_PP(dev);

    timer_del(s->done);
    memset(s->regs, 0, sizeof(s->regs));
    g_free(s->md_ref);
    s->md_ref = NULL;
    s->md_ref_w = s->md_ref_h = 0;
    rk_vepu_pp_update(s);
}

static void rk_vepu_pp_init(Object *obj)
{
    RkVepuPpState *s = RK_VEPU_PP(obj);

    memory_region_init_io(&s->iomem, obj, &rk_vepu_pp_ops, s,
                          TYPE_RK_VEPU_PP, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_vepu_pp_done, s);
}

static const Property rk_vepu_pp_properties[] = {
    DEFINE_PROP_UINT32("job-ns", RkVepuPpState, job_ns, 500000),
};

static const VMStateDescription rk_vepu_pp_vmstate = {
    .name = TYPE_RK_VEPU_PP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkVepuPpState, REGS_SIZE / 4),
        VMSTATE_TIMER_PTR(done, RkVepuPpState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_vepu_pp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_vepu_pp_reset);
    device_class_set_props(dc, rk_vepu_pp_properties);
    dc->vmsd = &rk_vepu_pp_vmstate;
}

static const TypeInfo rk_vepu_pp_info = {
    .name = TYPE_RK_VEPU_PP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkVepuPpState),
    .instance_init = rk_vepu_pp_init,
    .class_init = rk_vepu_pp_class_init,
};

static void rk_vepu_pp_register_types(void)
{
    type_register_static(&rk_vepu_pp_info);
}

type_init(rk_vepu_pp_register_types)
