/*
 * Novatek HDAL media engine stand-in: a register file that completes jobs.
 *
 * NA51103's media blocks (the VPE scaler, and the HDAL engines like it)
 * are run by the HDAL's kdrv_* modules the same way: they program a job
 * -- registers, or a link list of register writes in DDR -- set a fire bit
 * in a control register, and sleep until the engine's interrupt reports
 * the job done in a write-1-to-clear status register.  A job that never
 * completes stalls the HDAL's job manager (em_putjob times out after 3 s)
 * and it panics the whole media stack.
 *
 * This model does no pixel work.  It keeps every register as written and,
 * when a write to fire-reg sets a bit of fire-mask, clears those bits after
 * delay-us and sets done-bits in sts-reg, raising the line while
 * sts & en is non-zero (en-reg; all ones when en-reg is 0).  Output
 * buffers keep whatever was in them.  The engine's own registers, offsets
 * and bits come from the machine, per engine, from its kdrv module.
 *
 * With ll-reg set, a fire with a bit of ll-fire-mask first runs the link
 * list whose 40-bit address is at ll-reg (low word) and ll-reg + 4 (bits
 * 39:32).  The list is what the engine would otherwise be programmed with
 * -- the interrupt enable included -- as 64-bit commands, low word first
 * (vpe_drv_ll_write_reg() and friends):
 *
 *   hi[31:28] = 8   register write: hi[12:0] offset, lo the value
 *   hi[31:28] = 2   continue at hi[15:0]:lo[31:8], lo[3:0] the job id
 *   hi[31:28] = 0   end of the list, lo[3:0] the job id
 *
 * trace=on logs every list command (hi, lo), for working out an engine's
 * registers from a running firmware.
 *
 * scaler=on makes a job also do the VPE's pixel work, crop-free scaling of
 * one NV12 picture into a window of another, as the NVR's preview uses it
 * (kdrv_vpe's lists):
 *
 *   0x010  source size, [27:16] height, [11:0] width
 *   0x308  source Y; 0x30C its UV, or right after Y when 0
 *   0x800  source stride, [15:0]
 *   0x360  destination Y, where the window starts; 0x364 its UV, or one
 *          frame below the window when 0 (exact for a window on the top row)
 *   0x50C  destination frame, [27:16] height, [11:0] width (the stride)
 *   0x550  window width, 0x508 window height
 *   0x008  bits 11:10 set: scale the source; clear: fill the window with
 *          the background colour 0x260 ([7:0] Y, [15:8] U, [23:16] V), the
 *          NVR's "clear window" jobs, which keep empty tiles black
 *
 * The NVR's windows read the decoder's reduced copies (nvt-h26xdec), whose
 * UV follows Y; the destinations are the LCD300's video frames.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"

#define TYPE_NVT_ENGINE "nvt-engine"
OBJECT_DECLARE_SIMPLE_TYPE(NvtEngineState, NVT_ENGINE)

struct NvtEngineState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint8_t *regs;              /* size bytes, 32-bit registers */

    /* properties */
    char *name;
    uint32_t size;
    uint32_t fire_reg;
    uint32_t fire_mask;
    uint32_t sts_reg;
    uint32_t en_reg;
    uint32_t done_bits;
    uint32_t delay_us;
    uint32_t ll_reg;
    uint32_t ll_fire_mask;
    bool trace;
    bool scaler;
    uint32_t jobs;
};

#define LL_OP_END           0x0
#define LL_OP_NEXT          0x2
#define LL_OP_WRITE         0x8
#define LL_MAX_CMDS         65536

static uint32_t *reg(NvtEngineState *s, hwaddr off)
{
    return (uint32_t *)(s->regs + off);
}

static void nvt_engine_update_irq(NvtEngineState *s)
{
    uint32_t en = s->en_reg ? *reg(s, s->en_reg) : UINT32_MAX;

    qemu_set_irq(s->irq, (*reg(s, s->sts_reg) & en) != 0);
}

static void nvt_engine_done(void *opaque)
{
    NvtEngineState *s = opaque;

    *reg(s, s->fire_reg) &= ~s->fire_mask;
    *reg(s, s->sts_reg) |= s->done_bits;
    nvt_engine_update_irq(s);
}

/* Apply a list's register writes; a list that loops or faults just ends. */
static void nvt_engine_run_ll(NvtEngineState *s)
{
    hwaddr addr = ((hwaddr)(*reg(s, s->ll_reg + 4) & 0xFF) << 32) |
                  *reg(s, s->ll_reg);
    uint32_t cmd[2], off;
    int n;

    for (n = 0; n < LL_MAX_CMDS; n++, addr += 8) {
        if (address_space_read(&address_space_memory, addr,
                               MEMTXATTRS_UNSPECIFIED, cmd, sizeof(cmd))) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: link list at 0x%" HWADDR_PRIx
                          " unreadable\n", s->name, addr);
            return;
        }
        cmd[0] = le32_to_cpu(cmd[0]);
        cmd[1] = le32_to_cpu(cmd[1]);
        if (s->trace) {
            qemu_log("%s: job %u ll %08x %08x\n", s->name, s->jobs, cmd[1],
                     cmd[0]);
        }
        switch (cmd[1] >> 28) {
        case LL_OP_WRITE:
            off = cmd[1] & 0x1FFC;
            if (off < s->size && off != s->fire_reg && off != s->sts_reg) {
                *reg(s, off) = cmd[0];
            }
            break;
        case LL_OP_NEXT:
            addr = (((hwaddr)(cmd[1] & 0xFFFF) << 24) | (cmd[0] >> 8)) - 8;
            break;
        case LL_OP_END:
            return;
        default:
            qemu_log_mask(LOG_UNIMP, "%s: link list command 0x%08x%08x\n",
                          s->name, cmd[1], cmd[0]);
            break;
        }
    }
}

static void nvt_engine_scale(NvtEngineState *s)
{
    uint32_t sw = *reg(s, 0x010) & 0xFFF, sh = (*reg(s, 0x010) >> 16) & 0xFFF;
    uint32_t ss = *reg(s, 0x800) & 0xFFFF;
    uint32_t fw = *reg(s, 0x50C) & 0xFFF, fh = (*reg(s, 0x50C) >> 16) & 0xFFF;
    uint32_t dw = *reg(s, 0x550) & 0xFFF, dh = *reg(s, 0x508) & 0xFFF;
    hwaddr sy = *reg(s, 0x308), suv = *reg(s, 0x30C);
    hwaddr dy = *reg(s, 0x360), duv = *reg(s, 0x364);
    g_autofree uint8_t *src = NULL, *line = NULL;
    uint32_t r, x;

    ss = ss ? ss : sw;
    if (!dw || !dh || !fw || !fh || !dy || dw > fw || dh > fh) {
        return;
    }
    duv = duv ? duv : dy + (hwaddr)fw * fh;
    if (!(*reg(s, 0x008) & 0xC00)) {
        uint32_t bg = *reg(s, 0x260);

        line = g_malloc(dw + 1);
        memset(line, bg & 0xFF, dw);
        for (r = 0; r < dh; r++) {
            address_space_write(&address_space_memory, dy + (hwaddr)r * fw,
                                MEMTXATTRS_UNSPECIFIED, line, dw);
        }
        for (x = 0; x + 1 < dw; x += 2) {
            line[x] = (bg >> 8) & 0xFF;
            line[x + 1] = (bg >> 16) & 0xFF;
        }
        for (r = 0; r < dh / 2; r++) {
            address_space_write(&address_space_memory, duv + (hwaddr)r * fw,
                                MEMTXATTRS_UNSPECIFIED, line, dw & ~1);
        }
        return;
    }
    if (!sw || !sh || !sy || ss < sw) {
        return;
    }
    suv = suv ? suv : sy + (hwaddr)ss * sh;
    src = g_malloc((size_t)ss * sh);
    line = g_malloc(dw + 1);

    address_space_read(&address_space_memory, sy, MEMTXATTRS_UNSPECIFIED,
                       src, (size_t)ss * sh);
    for (r = 0; r < dh; r++) {
        const uint8_t *row = src + (size_t)(r * sh / dh) * ss;

        for (x = 0; x < dw; x++) {
            line[x] = row[x * sw / dw];
        }
        address_space_write(&address_space_memory, dy + (hwaddr)r * fw,
                            MEMTXATTRS_UNSPECIFIED, line, dw);
    }
    address_space_read(&address_space_memory, suv, MEMTXATTRS_UNSPECIFIED,
                       src, (size_t)ss * (sh / 2));
    for (r = 0; r < dh / 2; r++) {
        const uint8_t *row = src + (size_t)(r * (sh / 2) / (dh / 2)) * ss;

        for (x = 0; x < dw / 2; x++) {
            uint32_t sx = (x * (sw / 2) / (dw / 2)) * 2;

            line[2 * x] = row[sx];
            line[2 * x + 1] = row[sx + 1];
        }
        address_space_write(&address_space_memory, duv + (hwaddr)r * fw,
                            MEMTXATTRS_UNSPECIFIED, line, dw & ~1);
    }
}

static uint64_t nvt_engine_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtEngineState *s = opaque;

    return *reg(s, offset & ~3);
}

static void nvt_engine_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    NvtEngineState *s = opaque;
    uint32_t val = value;

    offset &= ~3;
    if (offset == s->sts_reg) {
        *reg(s, offset) &= ~val;
        nvt_engine_update_irq(s);
        return;
    }
    *reg(s, offset) = val;
    if (offset == s->fire_reg && (val & s->fire_mask)) {
        if (s->ll_reg && (val & s->ll_fire_mask)) {
            nvt_engine_run_ll(s);
        }
        if (s->scaler) {
            nvt_engine_scale(s);
        }
        s->jobs++;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  (int64_t)s->delay_us * SCALE_US);
    } else if (offset == s->en_reg) {
        nvt_engine_update_irq(s);
    }
}

static const MemoryRegionOps nvt_engine_ops = {
    .read = nvt_engine_read,
    .write = nvt_engine_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void nvt_engine_reset(DeviceState *dev)
{
    NvtEngineState *s = NVT_ENGINE(dev);

    timer_del(s->timer);
    memset(s->regs, 0, s->size);
    qemu_irq_lower(s->irq);
}

static void nvt_engine_realize(DeviceState *dev, Error **errp)
{
    NvtEngineState *s = NVT_ENGINE(dev);

    if (!s->size || s->size & 3 || s->fire_reg >= s->size ||
        s->sts_reg >= s->size || s->en_reg >= s->size ||
        s->ll_reg + 8 > s->size ||
        !s->fire_mask || !s->done_bits) {
        error_setg(errp, "nvt-engine: bad register layout");
        return;
    }
    s->regs = g_malloc0(s->size);
    memory_region_init_io(&s->iomem, OBJECT(dev), &nvt_engine_ops, s,
                          s->name ? s->name : TYPE_NVT_ENGINE, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_engine_done, s);
}

static void nvt_engine_init(Object *obj)
{
    NvtEngineState *s = NVT_ENGINE(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property nvt_engine_properties[] = {
    DEFINE_PROP_STRING("name", NvtEngineState, name),
    DEFINE_PROP_UINT32("size", NvtEngineState, size, 0x1000),
    DEFINE_PROP_UINT32("fire-reg", NvtEngineState, fire_reg, 0),
    DEFINE_PROP_UINT32("fire-mask", NvtEngineState, fire_mask, 0),
    DEFINE_PROP_UINT32("sts-reg", NvtEngineState, sts_reg, 0),
    DEFINE_PROP_UINT32("en-reg", NvtEngineState, en_reg, 0),
    DEFINE_PROP_UINT32("done-bits", NvtEngineState, done_bits, 0),
    DEFINE_PROP_UINT32("delay-us", NvtEngineState, delay_us, 1000),
    DEFINE_PROP_UINT32("ll-reg", NvtEngineState, ll_reg, 0),
    DEFINE_PROP_UINT32("ll-fire-mask", NvtEngineState, ll_fire_mask, 0),
    DEFINE_PROP_BOOL("trace", NvtEngineState, trace, false),
    DEFINE_PROP_BOOL("scaler", NvtEngineState, scaler, false),
};

static const VMStateDescription vmstate_nvt_engine = {
    .name = TYPE_NVT_ENGINE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_VBUFFER_UINT32(regs, NvtEngineState, 1, NULL, size),
        VMSTATE_TIMER_PTR(timer, NvtEngineState),
        VMSTATE_END_OF_LIST()
    },
};

static void nvt_engine_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_engine_realize;
    device_class_set_legacy_reset(dc, nvt_engine_reset);
    device_class_set_props(dc, nvt_engine_properties);
    dc->vmsd = &vmstate_nvt_engine;
}

static const TypeInfo nvt_engine_info = {
    .name           = TYPE_NVT_ENGINE,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(NvtEngineState),
    .instance_init  = nvt_engine_init,
    .class_init     = nvt_engine_class_init,
};

static void nvt_engine_register_types(void)
{
    type_register_static(&nvt_engine_info);
}

type_init(nvt_engine_register_types)
