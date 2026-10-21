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
