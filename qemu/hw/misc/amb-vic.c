/*
 * Ambarella S3L vectored interrupt controller: three 32-line banks and the
 * AHB scratchpad register that names the next interrupt.
 *
 * A bank is PL190's first nine registers, then per-line trigger setup
 * (SENSE / BOTHEDGE / EVENT), then one-at-a-time versions of the same
 * registers that take a line number instead of a mask.  This kernel uses
 * those: VIC_SUPPORT_CPU_OFFLOAD is 2 on S3L, so irq-ambarella.c reads
 * AHBSP_PRI_IRQ_C0 instead of walking the banks, and all ones there means
 * nothing is pending.
 *
 * SENSE 1 = level (EVENT 1 = high), SENSE 0 = edge (BOTHEDGE 1 = either
 * edge, else EVENT 1 = rising).  An edge stays latched until INT_EDGE_CLR.
 * Lines are numbered as the device tree numbers them, bank * 32 + bit.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_AMB_VIC "amb-vic"
OBJECT_DECLARE_SIMPLE_TYPE(AmbVicState, AMB_VIC)

#define AMB_VIC_BANKS   3
#define AMB_VIC_LINES   (AMB_VIC_BANKS * 32)
#define AMB_VIC_NONE    0xffffffff /* VIC_NULL_PRI_IRQ_VAL on S3L */

#define R_IRQ_STA       0x00
#define R_FIQ_STA       0x04
#define R_RAW_STA       0x08
#define R_INT_SEL       0x0c
#define R_INTEN         0x10
#define R_INTEN_CLR     0x14
#define R_SOFTEN        0x18
#define R_SOFTEN_CLR    0x1c
#define R_PROTEN        0x20
#define R_SENSE         0x24
#define R_BOTHEDGE      0x28
#define R_EVENT         0x2c
#define R_INT_PTR0      0x30
#define R_INT_PTR1      0x34
#define R_EDGE_CLR      0x38
#define R_I_SEL         0x3c
#define R_I_SEL_CLR     0x40
#define R_I_EN          0x44
#define R_I_EN_CLR      0x48
#define R_I_SOFT        0x4c
#define R_I_SOFT_CLR    0x50
#define R_I_SENSE       0x54
#define R_I_SENSE_CLR   0x58
#define R_I_BOTH        0x5c
#define R_I_BOTH_CLR    0x60
#define R_I_EVT         0x64
#define R_I_EVT_CLR     0x68
#define R_I_EDGE_CLR    0x98

#define AHBSP_PRI_IRQ_C0 0x38

typedef struct AmbVicBank {
    struct AmbVicState *owner;
    uint32_t level;     /* input lines as they are */
    uint32_t edge;      /* latched edges */
    uint32_t int_sel;
    uint32_t inten;
    uint32_t soften;
    uint32_t proten;
    uint32_t sense;
    uint32_t bothedge;
    uint32_t event;
    uint32_t ptr0;
    uint32_t ptr1;
} AmbVicBank;

struct AmbVicState {
    SysBusDevice parent_obj;

    MemoryRegion bank_io[AMB_VIC_BANKS];
    MemoryRegion scratch_io;
    qemu_irq irq;
    qemu_irq fiq;
    AmbVicBank bank[AMB_VIC_BANKS];
    uint32_t scratch[0x1000 / 4];
};

static uint32_t amb_vic_raw(const AmbVicBank *b)
{
    uint32_t level = (b->level & b->event) | (~b->level & ~b->event);

    return (b->sense & level) | (~b->sense & b->edge) | b->soften;
}

static uint32_t amb_vic_sta(const AmbVicBank *b, bool fiq)
{
    uint32_t sel = fiq ? b->int_sel : ~b->int_sel;

    return amb_vic_raw(b) & b->inten & sel;
}

static void amb_vic_update(AmbVicState *s)
{
    bool irq = false, fiq = false;

    for (int i = 0; i < AMB_VIC_BANKS; i++) {
        irq |= amb_vic_sta(&s->bank[i], false) != 0;
        fiq |= amb_vic_sta(&s->bank[i], true) != 0;
    }
    qemu_set_irq(s->irq, irq);
    qemu_set_irq(s->fiq, fiq);
}

static void amb_vic_set_irq(void *opaque, int line, int level)
{
    AmbVicState *s = opaque;
    AmbVicBank *b = &s->bank[line / 32];
    uint32_t bit = 1u << (line % 32);
    uint32_t was = b->level & bit;

    if (level) {
        b->level |= bit;
    } else {
        b->level &= ~bit;
    }
    if (was != (b->level & bit) && !(b->sense & bit)) {
        if ((b->bothedge & bit) || !!level == !!(b->event & bit)) {
            b->edge |= bit;
        }
    }
    amb_vic_update(s);
}

static uint64_t amb_vic_read(void *opaque, hwaddr off, unsigned size)
{
    AmbVicBank *b = opaque;

    switch (off) {
    case R_IRQ_STA:
        return amb_vic_sta(b, false);
    case R_FIQ_STA:
        return amb_vic_sta(b, true);
    case R_RAW_STA:
        return amb_vic_raw(b);
    case R_INT_SEL:
        return b->int_sel;
    case R_INTEN:
        return b->inten;
    case R_SOFTEN:
        return b->soften;
    case R_PROTEN:
        return b->proten;
    case R_SENSE:
        return b->sense;
    case R_BOTHEDGE:
        return b->bothedge;
    case R_EVENT:
        return b->event;
    case R_INT_PTR0:
        return b->ptr0;
    case R_INT_PTR1:
        return b->ptr1;
    default:
        qemu_log_mask(LOG_UNIMP, "amb-vic: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
}

static void amb_vic_write_bank(AmbVicBank *b, hwaddr off, uint32_t val)
{
    uint32_t bit = 1u << (val & 31);

    switch (off) {
    case R_INT_SEL:
        b->int_sel = val;
        break;
    case R_INTEN:
        b->inten |= val;
        break;
    case R_INTEN_CLR:
        b->inten &= ~val;
        break;
    case R_SOFTEN:
        b->soften |= val;
        break;
    case R_SOFTEN_CLR:
        b->soften &= ~val;
        break;
    case R_PROTEN:
        b->proten = val;
        break;
    case R_SENSE:
        b->sense = val;
        break;
    case R_BOTHEDGE:
        b->bothedge = val;
        break;
    case R_EVENT:
        b->event = val;
        break;
    case R_INT_PTR0:
        b->ptr0 = val;
        break;
    case R_INT_PTR1:
        b->ptr1 = val;
        break;
    case R_EDGE_CLR:
        b->edge &= ~val;
        break;
    case R_I_SEL:
        b->int_sel |= bit;
        break;
    case R_I_SEL_CLR:
        b->int_sel &= ~bit;
        break;
    case R_I_EN:
        b->inten |= bit;
        break;
    case R_I_EN_CLR:
        b->inten &= ~bit;
        break;
    case R_I_SOFT:
        b->soften |= bit;
        break;
    case R_I_SOFT_CLR:
        b->soften &= ~bit;
        break;
    case R_I_SENSE:
        b->sense |= bit;
        break;
    case R_I_SENSE_CLR:
        b->sense &= ~bit;
        break;
    case R_I_BOTH:
        b->bothedge |= bit;
        break;
    case R_I_BOTH_CLR:
        b->bothedge &= ~bit;
        break;
    case R_I_EVT:
        b->event |= bit;
        break;
    case R_I_EVT_CLR:
        b->event &= ~bit;
        break;
    case R_I_EDGE_CLR:
        b->edge &= ~bit;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "amb-vic: write 0x%" HWADDR_PRIx "\n", off);
        break;
    }
}

static void amb_vic_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    AmbVicBank *b = opaque;

    amb_vic_write_bank(b, off, val);
    amb_vic_update(b->owner);
}

static const MemoryRegionOps amb_vic_ops = {
    .read = amb_vic_read,
    .write = amb_vic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* The lowest pending line.  Priorities are left at their reset order. */
static uint64_t amb_scratch_read(void *opaque, hwaddr off, unsigned size)
{
    AmbVicState *s = opaque;

    if (off == AHBSP_PRI_IRQ_C0) {
        for (int i = 0; i < AMB_VIC_BANKS; i++) {
            uint32_t sta = amb_vic_sta(&s->bank[i], false);

            if (sta) {
                return i * 32 + ctz32(sta);
            }
        }
        return AMB_VIC_NONE;
    }
    return s->scratch[off / 4];
}

static void amb_scratch_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    AmbVicState *s = opaque;

    s->scratch[off / 4] = val;
}

static const MemoryRegionOps amb_scratch_ops = {
    .read = amb_scratch_read,
    .write = amb_scratch_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void amb_vic_reset(DeviceState *dev)
{
    AmbVicState *s = AMB_VIC(dev);

    for (int i = 0; i < AMB_VIC_BANKS; i++) {
        uint32_t level = s->bank[i].level;

        memset(&s->bank[i], 0, sizeof(s->bank[i]));
        s->bank[i].owner = s;
        s->bank[i].level = level;
    }
    memset(s->scratch, 0, sizeof(s->scratch));
    amb_vic_update(s);
}

static void amb_vic_init(Object *obj)
{
    AmbVicState *s = AMB_VIC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    for (int i = 0; i < AMB_VIC_BANKS; i++) {
        s->bank[i].owner = s;
        memory_region_init_io(&s->bank_io[i], obj, &amb_vic_ops, &s->bank[i],
                              "amb-vic", 0x1000);
        sysbus_init_mmio(sbd, &s->bank_io[i]);
    }
    memory_region_init_io(&s->scratch_io, obj, &amb_scratch_ops, s,
                          "amb-scratchpad", 0x1000);
    sysbus_init_mmio(sbd, &s->scratch_io);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(DEVICE(obj), amb_vic_set_irq, AMB_VIC_LINES);
}

static int amb_vic_post_load(void *opaque, int version_id)
{
    amb_vic_update(opaque);
    return 0;
}

static const VMStateDescription amb_vic_bank_vmstate = {
    .name = "amb-vic-bank",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(level, AmbVicBank),
        VMSTATE_UINT32(edge, AmbVicBank),
        VMSTATE_UINT32(int_sel, AmbVicBank),
        VMSTATE_UINT32(inten, AmbVicBank),
        VMSTATE_UINT32(soften, AmbVicBank),
        VMSTATE_UINT32(proten, AmbVicBank),
        VMSTATE_UINT32(sense, AmbVicBank),
        VMSTATE_UINT32(bothedge, AmbVicBank),
        VMSTATE_UINT32(event, AmbVicBank),
        VMSTATE_UINT32(ptr0, AmbVicBank),
        VMSTATE_UINT32(ptr1, AmbVicBank),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription amb_vic_vmstate = {
    .name = TYPE_AMB_VIC,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = amb_vic_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(bank, AmbVicState, AMB_VIC_BANKS, 1,
                             amb_vic_bank_vmstate, AmbVicBank),
        VMSTATE_UINT32_ARRAY(scratch, AmbVicState, 0x1000 / 4),
        VMSTATE_END_OF_LIST()
    },
};

static void amb_vic_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, amb_vic_reset);
    dc->vmsd = &amb_vic_vmstate;
}

static const TypeInfo amb_vic_info = {
    .name = TYPE_AMB_VIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AmbVicState),
    .instance_init = amb_vic_init,
    .class_init = amb_vic_class_init,
};

static void amb_vic_register(void)
{
    type_register_static(&amb_vic_info);
}

type_init(amb_vic_register)
