/*
 * Goke GK710x vectored interrupt controller.
 *
 * The silicon block is the Ambarella-style VIC: PL190's first nine
 * registers at their PL190 offsets, then per-line trigger configuration
 * (SENSE / BOTHEDGE / EVENT) and an edge-latch clear where PL190 has its
 * vector registers.  Every register is a 32-bit mask with bit n = input n.
 *
 * Goke's drivers never see this layout: they go through the vendor HAL
 * blob, which moves the registers to other offsets and permutes the bits
 * of every mask (see include/hw/arm/goke.h).  The offsets below are the
 * ones the HAL actually touches, recovered by running it:
 *
 *   0x00 IRQSTATUS   0x04 FIQSTATUS   0x08 RAWINTR    0x0C INTSELECT
 *   0x10 INTENABLE   0x14 INTENCLEAR  0x18 SOFTINT    0x1C SOFTINTCLEAR
 *   0x20 PROTECTION  0x24 SENSE       0x28 BOTHEDGE   0x2C EVENT
 *   0x38 EDGECLR
 *
 * SENSE 1 = level (EVENT 1 = active high, 0 = active low); SENSE 0 = edge
 * (BOTHEDGE 1 = either edge, else EVENT 1 = rising, 0 = falling).  An edge
 * stays latched until software writes its bit to EDGECLR, which the vendor
 * kernel does from irq_ack (mach-gk710x/irq.c).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GK_VIC "gk-vic"
OBJECT_DECLARE_SIMPLE_TYPE(GkVicState, GK_VIC)

#define GK_VIC_LINES    32

#define VIC_IRQSTATUS   0x00
#define VIC_FIQSTATUS   0x04
#define VIC_RAWINTR     0x08
#define VIC_INTSELECT   0x0C
#define VIC_INTENABLE   0x10
#define VIC_INTENCLEAR  0x14
#define VIC_SOFTINT     0x18
#define VIC_SOFTINTCLR  0x1C
#define VIC_PROTECTION  0x20
#define VIC_SENSE       0x24
#define VIC_BOTHEDGE    0x28
#define VIC_EVENT       0x2C
#define VIC_EDGECLR     0x38

struct GkVicState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq fiq;

    uint32_t level;     /* input lines as driven */
    uint32_t latched;   /* edges seen, until EDGECLR */
    uint32_t select;
    uint32_t enable;
    uint32_t soft;
    uint32_t protect;
    uint32_t sense;
    uint32_t bothedge;
    uint32_t event;
    uint32_t spare[2];  /* 0x30, 0x34: no function known */

    /* Inputs whose source is active low and so rests high (the IR
     * receiver: gk_ir_v1_00.c asks for IRQF_TRIGGER_LOW).  With nothing
     * modelled behind them they must still read idle. */
    uint32_t idle_high;
};

static uint32_t gk_vic_raw(GkVicState *s)
{
    /* Level lines read their (polarity-corrected) input; edge lines their
     * latch. */
    uint32_t lvl = (s->level & s->event) | (~s->level & ~s->event);

    return ((lvl & s->sense) | (s->latched & ~s->sense)) | s->soft;
}

static void gk_vic_update(GkVicState *s)
{
    uint32_t pend = gk_vic_raw(s) & s->enable;

    qemu_set_irq(s->irq, !!(pend & ~s->select));
    qemu_set_irq(s->fiq, !!(pend & s->select));
}

static void gk_vic_set_irq(void *opaque, int n, int level)
{
    GkVicState *s = opaque;
    const uint32_t bit = 1u << n;
    const bool was = s->level & bit;

    if (was == !!level) {
        return;
    }
    if (level) {
        s->level |= bit;
    } else {
        s->level &= ~bit;
    }
    if (!(s->sense & bit)) {
        bool rising = level && !was;

        if ((s->bothedge & bit) || (rising == !!(s->event & bit))) {
            s->latched |= bit;
        }
    }
    gk_vic_update(s);
}

static uint64_t gk_vic_read(void *opaque, hwaddr offset, unsigned size)
{
    GkVicState *s = opaque;
    uint32_t raw = gk_vic_raw(s);

    switch (offset) {
    case VIC_IRQSTATUS:
        return raw & s->enable & ~s->select;
    case VIC_FIQSTATUS:
        return raw & s->enable & s->select;
    case VIC_RAWINTR:
        return raw;
    case VIC_INTSELECT:
        return s->select;
    case VIC_INTENABLE:
        return s->enable;
    case VIC_SOFTINT:
        return s->soft;
    case VIC_PROTECTION:
        return s->protect;
    case VIC_SENSE:
        return s->sense;
    case VIC_BOTHEDGE:
        return s->bothedge;
    case VIC_EVENT:
        return s->event;
    case 0x30:
    case 0x34:
        return s->spare[(offset - 0x30) / 4];
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "gk-vic: read of 0x%02" HWADDR_PRIx
                      "\n", offset);
        return 0;
    }
}

static void gk_vic_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    GkVicState *s = opaque;

    switch (offset) {
    case VIC_INTSELECT:
        s->select = val;
        break;
    case VIC_INTENABLE:
        s->enable |= val;
        break;
    case VIC_INTENCLEAR:
        s->enable &= ~val;
        break;
    case VIC_SOFTINT:
        s->soft |= val;
        break;
    case VIC_SOFTINTCLR:
        s->soft &= ~val;
        break;
    case VIC_PROTECTION:
        s->protect = val & 1;
        break;
    case VIC_SENSE:
        s->sense = val;
        break;
    case VIC_BOTHEDGE:
        s->bothedge = val;
        break;
    case VIC_EVENT:
        s->event = val;
        break;
    case VIC_EDGECLR:
        s->latched &= ~val;
        break;
    case 0x30:
    case 0x34:
        s->spare[(offset - 0x30) / 4] = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "gk-vic: write 0x%08" PRIx64
                      " to 0x%02" HWADDR_PRIx "\n", val, offset);
        return;
    }
    gk_vic_update(s);
}

static const MemoryRegionOps gk_vic_ops = {
    .read = gk_vic_read,
    .write = gk_vic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gk_vic_reset(DeviceState *dev)
{
    GkVicState *s = GK_VIC(dev);

    s->level = s->idle_high;
    s->latched = 0;
    s->select = 0;
    s->enable = 0;
    s->soft = 0;
    s->protect = 0;
    s->sense = 0;
    s->bothedge = 0;
    s->event = 0;
    s->spare[0] = s->spare[1] = 0;
    gk_vic_update(s);
}

static void gk_vic_init(Object *obj)
{
    GkVicState *s = GK_VIC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &gk_vic_ops, s, TYPE_GK_VIC,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(DEVICE(obj), gk_vic_set_irq, GK_VIC_LINES);
}

static const VMStateDescription vmstate_gk_vic = {
    .name = TYPE_GK_VIC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(level, GkVicState),
        VMSTATE_UINT32(latched, GkVicState),
        VMSTATE_UINT32(select, GkVicState),
        VMSTATE_UINT32(enable, GkVicState),
        VMSTATE_UINT32(soft, GkVicState),
        VMSTATE_UINT32(protect, GkVicState),
        VMSTATE_UINT32(sense, GkVicState),
        VMSTATE_UINT32(bothedge, GkVicState),
        VMSTATE_UINT32(event, GkVicState),
        VMSTATE_UINT32_ARRAY(spare, GkVicState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static const Property gk_vic_properties[] = {
    DEFINE_PROP_UINT32("idle-high", GkVicState, idle_high, 0),
};

static void gk_vic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, gk_vic_properties);
    device_class_set_legacy_reset(dc, gk_vic_reset);
    dc->vmsd = &vmstate_gk_vic;
}

static const TypeInfo gk_vic_info = {
    .name          = TYPE_GK_VIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GkVicState),
    .instance_init = gk_vic_init,
    .class_init    = gk_vic_class_init,
};

static void gk_vic_register_types(void)
{
    type_register_static(&gk_vic_info);
}

type_init(gk_vic_register_types)
