/*
 * Fullhan SADC: the eight-input successive-approximation ADC at 0xF1200000
 * (drivers/misc/fh_sadc*.c), which cameras read their light sensor and
 * buttons through.
 *
 * "version" 1, the 3.0.8 kernels' block (FH885x V100, FH8632):
 *   0x00 CMD (1 starts), 0x04 CONTROL (7:0 the input, 12 power), 0x08 IER,
 *   0x0C INT_STATUS (bit 0 done, write 1 to clear), 0x10-0x1C DOUT0-3: two
 *   inputs a word, the even one in the high half.
 * "version" 2, FH8626V100's (CONFIG_FH_SADC_V21): the same with a MODEL
 *   register in front and scan / glitch registers after CONTROL -- CMD
 *   0x04, CONTROL 0x08, IER 0x24, INT_STATUS 0x28, DOUT0-3 0x2C-0x38.
 * "version" 3, the 4.9 kernels' (FH885xV200, FH865x):
 *   0x00 CTRL0 bit 0 enable, 0x04 CTRL1 bit 0 take the configuration,
 *   0x40 INT_EN / 0x44 INT_STA (7:0 a conversion done per input; the
 *   driver clears by writing the complement), 0x50-0x5C DOUT0-3: the even
 *   input in the low half.
 *
 * A conversion takes 100 us and returns the input's level, the "chN"
 * properties (12 bits; qom-set them while the guest runs to move a light
 * sensor).  The button-hit and glitch logic of v3 is not modelled.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_FH_SADC "fh-sadc"
OBJECT_DECLARE_SIMPLE_TYPE(FhSadcState, FH_SADC)

#define FH_SADC_SIZE        0x1000
#define FH_SADC_NREGS       (0x130 / 4)
#define FH_SADC_INPUTS      8
#define FH_SADC_CONV_NS     100000

/* v1, v2: CMD, CONTROL, IER, INT_STATUS, DOUT0 */
static const uint8_t fh_sadc_v12[2][5] = {
    { 0x00 / 4, 0x04 / 4, 0x08 / 4, 0x0C / 4, 0x10 / 4 },
    { 0x04 / 4, 0x08 / 4, 0x24 / 4, 0x28 / 4, 0x2C / 4 },
};
#define V1_CMD              (fh_sadc_v12[s->version - 1][0])
#define V1_CONTROL          (fh_sadc_v12[s->version - 1][1])
#define V1_IER              (fh_sadc_v12[s->version - 1][2])
#define V1_INT_STATUS       (fh_sadc_v12[s->version - 1][3])
#define V1_DOUT0            (fh_sadc_v12[s->version - 1][4])
/* v3 */
#define V3_CTRL0            (0x00 / 4)
#define V3_CTRL1            (0x04 / 4)
#define V3_INT_EN           (0x40 / 4)
#define V3_INT_STA          (0x44 / 4)
#define V3_DOUT0            (0x50 / 4)

struct FhSadcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t version;
    uint16_t level[FH_SADC_INPUTS];
    uint32_t regs[FH_SADC_NREGS];
    uint32_t pending;           /* inputs being converted */
};

static void fh_sadc_update_irq(FhSadcState *s)
{
    bool level;

    if (s->version < 3) {
        level = s->regs[V1_INT_STATUS] & s->regs[V1_IER] & 1;
    } else {
        level = s->regs[V3_INT_STA] & s->regs[V3_INT_EN] & 0xFF;
    }
    qemu_set_irq(s->irq, level);
}

static void fh_sadc_store(FhSadcState *s, int dout0, int ch, bool even_high)
{
    uint32_t *w = &s->regs[dout0 + ch / 2];
    int shift = ((ch & 1) ^ even_high) ? 16 : 0;

    *w = deposit32(*w, shift, 16, s->level[ch] & 0xFFF);
}

static void fh_sadc_done(void *opaque)
{
    FhSadcState *s = FH_SADC(opaque);
    int ch;

    for (ch = 0; ch < FH_SADC_INPUTS; ch++) {
        if (!(s->pending & (1 << ch))) {
            continue;
        }
        if (s->version < 3) {
            fh_sadc_store(s, V1_DOUT0, ch, true);
            s->regs[V1_INT_STATUS] |= 1;
        } else {
            fh_sadc_store(s, V3_DOUT0, ch, false);
            s->regs[V3_INT_STA] |= 1 << ch;
        }
    }
    s->pending = 0;
    fh_sadc_update_irq(s);
}

static void fh_sadc_start(FhSadcState *s, uint32_t inputs)
{
    s->pending |= inputs & 0xFF;
    if (s->pending) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  FH_SADC_CONV_NS);
    }
}

static uint64_t fh_sadc_read(void *opaque, hwaddr off, unsigned size)
{
    FhSadcState *s = FH_SADC(opaque);

    return off < FH_SADC_NREGS * 4 ? s->regs[off >> 2] : 0;
}

static void fh_sadc_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    FhSadcState *s = FH_SADC(opaque);
    int r = off >> 2;

    if (off >= FH_SADC_NREGS * 4) {
        return;
    }
    if (s->version < 3) {
        if (r == V1_CMD) {
            if (val & 1) {
                fh_sadc_start(s, s->regs[V1_CONTROL]);
            }
            return;
        } else if (r == V1_INT_STATUS) {
            s->regs[r] &= ~val;
        } else if (r >= V1_DOUT0 && r < V1_DOUT0 + 4) {
            return;
        } else {
            s->regs[r] = val;
        }
    } else {
        switch (r) {
        case V3_CTRL1:
            if ((val & 1) && (s->regs[V3_CTRL0] & 1)) {
                fh_sadc_start(s, s->regs[V3_INT_EN]);
            }
            s->regs[r] = val & ~1;
            return;
        case V3_INT_STA:
            s->regs[r] &= val;
            break;
        case V3_DOUT0 ... V3_DOUT0 + 7:
            return;
        default:
            s->regs[r] = val;
            break;
        }
    }
    fh_sadc_update_irq(s);
}

static const MemoryRegionOps fh_sadc_ops = {
    .read = fh_sadc_read,
    .write = fh_sadc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_sadc_reset(DeviceState *dev)
{
    FhSadcState *s = FH_SADC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pending = 0;
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void fh_sadc_realize(DeviceState *dev, Error **errp)
{
    FhSadcState *s = FH_SADC(dev);

    if (s->version < 1 || s->version > 3) {
        error_setg(errp, "fh-sadc: version must be 1, 2 or 3");
    }
}

static void fh_sadc_init(Object *obj)
{
    FhSadcState *s = FH_SADC(obj);
    char name[8];
    int ch;

    memory_region_init_io(&s->iomem, obj, &fh_sadc_ops, s, TYPE_FH_SADC,
                          FH_SADC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fh_sadc_done, s);
    for (ch = 0; ch < FH_SADC_INPUTS; ch++) {
        snprintf(name, sizeof(name), "ch%d", ch);
        object_property_add_uint16_ptr(obj, name, &s->level[ch],
                                       OBJ_PROP_FLAG_READWRITE);
    }
}

static const VMStateDescription vmstate_fh_sadc = {
    .name = TYPE_FH_SADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, FhSadcState, FH_SADC_NREGS),
        VMSTATE_UINT16_ARRAY(level, FhSadcState, FH_SADC_INPUTS),
        VMSTATE_UINT32(pending, FhSadcState),
        VMSTATE_TIMER_PTR(timer, FhSadcState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property fh_sadc_properties[] = {
    DEFINE_PROP_UINT32("version", FhSadcState, version, 3),
};

static void fh_sadc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = fh_sadc_realize;
    device_class_set_legacy_reset(dc, fh_sadc_reset);
    device_class_set_props(dc, fh_sadc_properties);
    dc->vmsd = &vmstate_fh_sadc;
}

static const TypeInfo fh_sadc_info = {
    .name           = TYPE_FH_SADC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhSadcState),
    .instance_init  = fh_sadc_init,
    .class_init     = fh_sadc_class_init,
};

static void fh_sadc_register_types(void)
{
    type_register_static(&fh_sadc_info);
}

type_init(fh_sadc_register_types)
