/*
 * Novatek IR remote receiver ("nvt,nvt_remote"; NA51103 / NA51090 at
 * 0x2_F0130000, SPI 25): drivers/media/rc/novatek_ir/nvt_ir_fr.{c,h} in
 * the NS02501 SDK kernel, the driver the NVRs' kernels carry.
 *
 *   0x00 CTRL      bit 0 enable, bit 4 threshold select, bit 5 invert,
 *                  bit 6 MSB first, bit 7 repeat detect, [9:8] header,
 *                  bit 12 bi-phase, [22:16] data length, [30:24] match
 *                  length
 *   0x08 INT_EN    the status bits that interrupt
 *   0x0C STS       bit 0 data ready, 1 error (timeout), 2 match, 3 repeat,
 *                  4 overrun; write 1 to clear
 *   0x10 / 0x14    the frame received, bits 31:0 / 63:32
 *   0x18 / 0x1C    the match data
 *   0x20..0x28     header / repeat / logic / glitch / error thresholds
 *   0x2C / 0x30    the raw (bi-phase) data
 *
 * Writing the QOM property "frame" (a 64-bit word) delivers a received
 * frame to an enabled receiver: DATA, ready -- match too when it equals
 * the match data -- and overrun when the last ready was not yet cleared.
 * "repeat" delivers a repeat code (STS bit 3) to one with repeat detect.
 * The driver disables the receiver in its interrupt and enables it again
 * once the frame is decoded, so a frame sent meanwhile is dropped, as on
 * silicon.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_IR "nvt-ir"
OBJECT_DECLARE_SIMPLE_TYPE(NvtIrState, NVT_IR)

#define IR_SIZE         0x1000
#define IR_REGS         0x40
#define IR_CTRL         0x00
#define IR_INT_EN       0x08
#define IR_STS          0x0C
#define IR_DATA_LO      0x10
#define IR_DATA_HI      0x14
#define IR_MATCH_LO     0x18
#define IR_MATCH_HI     0x1C

#define CTRL_EN         (1u << 0)
#define CTRL_REP_EN     (1u << 7)
#define STS_RD          (1u << 0)
#define STS_MATCH       (1u << 2)
#define STS_REP         (1u << 3)
#define STS_OVERRUN     (1u << 4)
#define STS_ALL         0x1F

struct NvtIrState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[IR_REGS / 4];
    uint64_t frames;            /* frames delivered, for the record */
};

static uint32_t *reg(NvtIrState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_ir_update_irq(NvtIrState *s)
{
    qemu_set_irq(s->irq, !!(*reg(s, IR_STS) & *reg(s, IR_INT_EN) & STS_ALL));
}

static uint64_t nvt_ir_read(void *opaque, hwaddr off, unsigned size)
{
    NvtIrState *s = opaque;

    return off < IR_REGS ? *reg(s, off) : 0;
}

static void nvt_ir_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    NvtIrState *s = opaque;

    if (off >= IR_REGS) {
        return;
    }
    switch (off) {
    case IR_STS:
        *reg(s, off) &= ~val;
        break;
    case IR_DATA_LO:
    case IR_DATA_HI:
        return;                         /* read only */
    default:
        *reg(s, off) = val;
        break;
    }
    nvt_ir_update_irq(s);
}

static const MemoryRegionOps nvt_ir_ops = {
    .read = nvt_ir_read,
    .write = nvt_ir_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_ir_get_u64(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    NvtIrState *s = NVT_IR(obj);
    uint64_t val = (uint64_t)*reg(s, IR_DATA_HI) << 32 | *reg(s, IR_DATA_LO);

    visit_type_uint64(v, name, &val, errp);
}

static void nvt_ir_set_frame(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    NvtIrState *s = NVT_IR(obj);
    uint64_t val;

    if (!visit_type_uint64(v, name, &val, errp)) {
        return;
    }
    if (!(*reg(s, IR_CTRL) & CTRL_EN)) {
        return;                         /* not listening */
    }
    if (*reg(s, IR_STS) & STS_RD) {
        *reg(s, IR_STS) |= STS_OVERRUN;
    }
    *reg(s, IR_DATA_LO) = val;
    *reg(s, IR_DATA_HI) = val >> 32;
    *reg(s, IR_STS) |= STS_RD;
    if (*reg(s, IR_DATA_LO) == *reg(s, IR_MATCH_LO) &&
        *reg(s, IR_DATA_HI) == *reg(s, IR_MATCH_HI)) {
        *reg(s, IR_STS) |= STS_MATCH;
    }
    s->frames++;
    nvt_ir_update_irq(s);
}

static void nvt_ir_set_repeat(Object *obj, Visitor *v, const char *name,
                              void *opaque, Error **errp)
{
    NvtIrState *s = NVT_IR(obj);
    bool val;

    if (!visit_type_bool(v, name, &val, errp)) {
        return;
    }
    if (val && (*reg(s, IR_CTRL) & (CTRL_EN | CTRL_REP_EN)) ==
               (CTRL_EN | CTRL_REP_EN)) {
        *reg(s, IR_STS) |= STS_REP;
        nvt_ir_update_irq(s);
    }
}

static void nvt_ir_reset(DeviceState *dev)
{
    NvtIrState *s = NVT_IR(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void nvt_ir_init(Object *obj)
{
    NvtIrState *s = NVT_IR(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_ir_ops, s, TYPE_NVT_IR,
                          IR_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    object_property_add(obj, "frame", "uint64", nvt_ir_get_u64,
                        nvt_ir_set_frame, NULL, NULL);
    object_property_add(obj, "repeat", "bool", NULL, nvt_ir_set_repeat,
                        NULL, NULL);
}

static const VMStateDescription vmstate_nvt_ir = {
    .name = TYPE_NVT_IR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtIrState, IR_REGS / 4),
        VMSTATE_UINT64(frames, NvtIrState),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_ir_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_ir_reset);
    dc->vmsd = &vmstate_nvt_ir;
}

static const TypeInfo nvt_ir_info = {
    .name = TYPE_NVT_IR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtIrState),
    .instance_init = nvt_ir_init,
    .class_init = nvt_ir_class_init,
};

static void nvt_ir_register_types(void)
{
    type_register_static(&nvt_ir_info);
}

type_init(nvt_ir_register_types)
