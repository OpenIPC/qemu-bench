/*
 * Ingenic IVDC: T41's and T23's direct ISP-to-encoder buffer at
 * 0x13200000.
 *
 * In direct_mode the ISP's channel 0 writes into the IVDC's store and
 * the encoder (T41's AVPU, T23's HELIX) reads it from there
 * (ingenic-isp.c, ingenic-avpu.c, ingenic-vpu.c); this is the block's own
 * register file, as the IVDC driver inside tx-isp-t41.ko and
 * tx-isp-t23.ko (tx-isp-ivdc.c, the same on both) uses it:
 *
 *   0x00         bit 0 resets the block: the driver writes 1 and spins
 *                (rewriting 0x70 / 0x88) until it reads back 0, which it
 *                does at once here
 *   0x04         2 starts it
 *   0x10 / 0x14  Y / UV picture size, w << 16 | h
 *   0x24         misc ioctl settings; 0x30 1 at init; 0x5C the mode
 *   0x34 / 0x38  the DRAM spill buffers' Y / UV addresses
 *   0x40         interrupt enables, a bit per status bit (0x244FC0FF from
 *                ivdc_core_ops_init: the frame, VPU and error bits)
 *   0x44         status (ivdc_core_interrupt_service_routine): bit 16 ISP
 *                frame done, 17 ISP frame start, 18 / 14 VPU0 end / half
 *                end, 19 / 15 VPU1's, 22 snapshot DMA done; 0..3, 6, 7,
 *                26 the error counts (FIFO and DDR overflows, ISP reset
 *                and frame errors, VPU errors) of /proc/jz/isp/isp-ivdc
 *   0x54         write the bits read from 0x44 to clear them
 *   0xA4 / 0xA8  memory lines; 0xEC / 0xF0 Y / UV maximum DRAM line
 *                (cleared by writing 0)
 *   others       storage
 *
 * The interrupt (T41's core INTC source 13, IRQ_IVDC: tx-isp requests it
 * as "isp-ivdc"; the device tree's node names 0, jpeg0's line; T23's INTC
 * 38) is high while an enabled status bit is set.  The ISP raises frame start and
 * frame done for each picture it writes into the store (the "frame" GPIO
 * input); nothing here overflows, so the error bits stay clear.  The
 * driver counts the bits; at frame done it moves its state to 2 (running)
 * and tells the ISP core so once.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_IVDC "ingenic-ivdc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicIvdcState, INGENIC_IVDC)

#define IVDC_REGS       0x400
#define R_CTRL          0x00
#define R_IRQ_EN        0x40
#define R_STATUS        0x44
#define R_CLEAR         0x54

#define ST_ISP_FRAME_DONE   (1u << 16)
#define ST_ISP_FRAME_START  (1u << 17)

struct IngenicIvdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    bool trace;
    uint32_t regs[IVDC_REGS / 4];
};

static void ivdc_update(IngenicIvdcState *s)
{
    qemu_set_irq(s->irq, (s->regs[R_IRQ_EN >> 2] &
                          s->regs[R_STATUS >> 2]) != 0);
}

/* a picture written into the store: frame start, then frame done */
static void ivdc_frame(void *opaque, int n, int level)
{
    IngenicIvdcState *s = opaque;

    if (level) {
        s->regs[R_STATUS >> 2] |= ST_ISP_FRAME_START | ST_ISP_FRAME_DONE;
        ivdc_update(s);
    }
}

static uint64_t ingenic_ivdc_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIvdcState *s = opaque;
    uint32_t v = off < IVDC_REGS ? s->regs[off >> 2] : 0;

    if (s->trace) {
        qemu_log("ingenic-ivdc: read [0x%03x] = 0x%08x\n", (int)off, v);
    }
    return v;
}

static void ingenic_ivdc_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    IngenicIvdcState *s = opaque;

    if (s->trace) {
        qemu_log("ingenic-ivdc: [0x%03x] = 0x%08x\n", (int)off,
                 (uint32_t)val);
    }
    if (off >= IVDC_REGS) {
        return;
    }
    switch (off) {
    case R_STATUS:
        return;                         /* read only */
    case R_CLEAR:
        s->regs[R_STATUS >> 2] &= ~val;
        break;
    case R_CTRL:
        s->regs[R_CTRL >> 2] = val & ~1u;   /* the reset is done */
        break;
    default:
        s->regs[off >> 2] = val;
        break;
    }
    ivdc_update(s);
}

static const MemoryRegionOps ingenic_ivdc_ops = {
    .read = ingenic_ivdc_read,
    .write = ingenic_ivdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_ivdc_reset(DeviceState *dev)
{
    IngenicIvdcState *s = INGENIC_IVDC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    ivdc_update(s);
}

static void ingenic_ivdc_init(Object *obj)
{
    IngenicIvdcState *s = INGENIC_IVDC(obj);

    /* the driver maps 0x80000; JPEG follows at 0x13280000 */
    memory_region_init_io(&s->iomem, obj, &ingenic_ivdc_ops, s,
                          TYPE_INGENIC_IVDC, 0x80000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), ivdc_frame, "frame", 1);
}

static const Property ingenic_ivdc_props[] = {
    DEFINE_PROP_BOOL("trace", IngenicIvdcState, trace, false),
};

static const VMStateDescription vmstate_ingenic_ivdc = {
    .name = TYPE_INGENIC_IVDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicIvdcState, IVDC_REGS / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ivdc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_ivdc_reset);
    dc->vmsd = &vmstate_ingenic_ivdc;
    device_class_set_props(dc, ingenic_ivdc_props);
}

static const TypeInfo ingenic_ivdc_info = {
    .name = TYPE_INGENIC_IVDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicIvdcState),
    .instance_init = ingenic_ivdc_init,
    .class_init = ingenic_ivdc_class_init,
};

static void ingenic_ivdc_register_types(void)
{
    type_register_static(&ingenic_ivdc_info);
}

type_init(ingenic_ivdc_register_types)
