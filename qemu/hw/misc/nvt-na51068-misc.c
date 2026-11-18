/*
 * Small Novatek NA51068 (NT98321 / NT98323) blocks:
 *
 * nvt-na51068-pwm, 0xFE6C0000 (SPI 43), drivers/pwm/pwm-na51068.c: three
 * channels clocked at 12 MHz.
 *   0x00 INT_CSTAT  0x04 START_CTRL  bit n runs channel n
 *   0x10 + 0x10 n   CTRL: bit 1 UPDATE (load CNTB / CMPB, self-clearing),
 *                   2 invert, [4:3] mode (1 continuous), 5 int, 8 enable
 *   0x14 + 0x10 n   CNTB period in ticks    0x18 + 0x10 n  CMPB, the low
 *                   part of the period (period - duty)
 *   0x1C + 0x10 n   CNTO, the running count: CNTB down to 0
 *   0x90 TMR_REV    0xA0.. repeat, 0xC0.. pattern, 0x140.. length: storage
 * The output is not a pin: each channel's period and duty (ns) read back
 * as QOM properties "period-ns<n>" / "duty-ns<n>" while it runs, else 0.
 *
 * nvt-na51068-ir, 0xFE540000 (SPI 65), drivers/media/rc/novatek_ir: an
 * IR remote decoder.
 *   0x00 CTRL  bit 0 enable    0x04..0x0C thresholds, divider, FIFO size
 *   0x10 STS   bit 0 RD (a frame), 1 REP (repeat code), 2 TOUT; [8:3]
 *              the bit count; write 1 to clear
 *   0x14 DATA_LO  0x18 DATA_HI  the frame     0x1C INT  the STS enables
 * Writing the QOM property "frame" (a 64-bit word; "repeat" for a repeat
 * code) delivers a received frame with an enabled decoder: DATA, RD,
 * a bit count of 32 (64 for frames above 32 bits), and clears CTRL's
 * enable, as the decoder does after each frame (nvt_ir_enable() turns it
 * back on).
 *
 * nvt-thermal, Novatek's stand-alone thermal sensor (NA51068 0xFE2C0000,
 * NS02501 0xF0260000; drivers/thermal/nvtivot_thermal.c):
 *   0x04 CTRL (0xB enables)  0x08 the reading, which the kernel takes as
 *   (reading - trim) * step / 1000 + 30 degrees, trim the efuse's or the
 *   tree's default_trim (231 on NA51068, 191 on NS02501), step 1319 on
 *   NA51068 and 500 on NS02501 (TEMP_MULTI).  "temperature" (degrees C,
 *   default 45), "trim" and "step" set what 0x08 reads once enabled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

/* ---- PWM ---------------------------------------------------------------- */

#define TYPE_NVT_NA51068_PWM "nvt-na51068-pwm"
OBJECT_DECLARE_SIMPLE_TYPE(Na51068PwmState, NVT_NA51068_PWM)

#define PWM_NCH             3
#define PWM_START_CTRL      0x04
#define PWM_CTRL(n)         (0x10 + 0x10 * (n))
#define PWM_CNTB(n)         (0x14 + 0x10 * (n))
#define PWM_CMPB(n)         (0x18 + 0x10 * (n))
#define PWM_CNTO(n)         (0x1C + 0x10 * (n))
#define PWM_TMR_REV         0x90
#define PWM_NREGS           (0x180 / 4)
#define PWM_CTRL_UPDATE     (1u << 1)
#define PWM_CTRL_EN         (1u << 8)
#define PWM_CLK             12000000
#define PWM_REV             0x00010000

struct Na51068PwmState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[PWM_NREGS];
    uint32_t cntb[PWM_NCH];     /* loaded by UPDATE */
    uint32_t cmpb[PWM_NCH];
    int64_t start_ns[PWM_NCH];
};

static bool na51068_pwm_running(Na51068PwmState *s, int n)
{
    return (s->regs[PWM_START_CTRL / 4] & (1u << n)) &&
           (s->regs[PWM_CTRL(n) / 4] & PWM_CTRL_EN) && s->cntb[n];
}

static uint64_t na51068_pwm_read(void *opaque, hwaddr offset, unsigned size)
{
    Na51068PwmState *s = opaque;
    int n;

    for (n = 0; n < PWM_NCH; n++) {
        if (offset == PWM_CNTO(n)) {
            int64_t ticks;

            if (!na51068_pwm_running(s, n)) {
                return 0;
            }
            ticks = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                             s->start_ns[n], PWM_CLK, NANOSECONDS_PER_SECOND);
            return s->cntb[n] - ticks % s->cntb[n];
        }
    }
    if (offset == PWM_TMR_REV) {
        return PWM_REV;
    }
    if (offset < PWM_NREGS * 4) {
        return s->regs[offset / 4];
    }
    qemu_log_mask(LOG_GUEST_ERROR, "nvt-na51068-pwm: read 0x%" HWADDR_PRIx
                  "\n", offset);
    return 0;
}

static void na51068_pwm_write(void *opaque, hwaddr offset, uint64_t val,
                              unsigned size)
{
    Na51068PwmState *s = opaque;
    uint32_t started;
    int n;

    if (offset >= PWM_NREGS * 4 || offset == PWM_TMR_REV) {
        return;
    }
    started = ~s->regs[PWM_START_CTRL / 4];
    s->regs[offset / 4] = val;
    for (n = 0; n < PWM_NCH; n++) {
        if (offset == PWM_CTRL(n) && (val & PWM_CTRL_UPDATE)) {
            s->cntb[n] = s->regs[PWM_CNTB(n) / 4];
            s->cmpb[n] = MIN(s->regs[PWM_CMPB(n) / 4], s->cntb[n]);
            s->regs[offset / 4] &= ~PWM_CTRL_UPDATE;
        }
        if (offset == PWM_START_CTRL && (val & started & (1u << n))) {
            s->start_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
    }
}

static const MemoryRegionOps na51068_pwm_ops = {
    .read = na51068_pwm_read,
    .write = na51068_pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void na51068_pwm_get_ns(Object *obj, Visitor *v, const char *name,
                               void *opaque, Error **errp)
{
    Na51068PwmState *s = NVT_NA51068_PWM(obj);
    bool duty = name[0] == 'd';
    int n = name[strlen(name) - 1] - '0';
    uint64_t ticks = 0, ns;

    if (na51068_pwm_running(s, n)) {
        ticks = duty ? s->cntb[n] - s->cmpb[n] : s->cntb[n];
    }
    ns = muldiv64(ticks, NANOSECONDS_PER_SECOND, PWM_CLK);
    visit_type_uint64(v, name, &ns, errp);
}

static void na51068_pwm_reset(DeviceState *dev)
{
    Na51068PwmState *s = NVT_NA51068_PWM(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->cntb, 0, sizeof(s->cntb));
    memset(s->cmpb, 0, sizeof(s->cmpb));
}

static void na51068_pwm_init(Object *obj)
{
    Na51068PwmState *s = NVT_NA51068_PWM(obj);
    char name[16];
    int n;

    memory_region_init_io(&s->iomem, obj, &na51068_pwm_ops, s,
                          TYPE_NVT_NA51068_PWM, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    for (n = 0; n < PWM_NCH; n++) {
        snprintf(name, sizeof(name), "period-ns%d", n);
        object_property_add(obj, name, "uint64", na51068_pwm_get_ns, NULL,
                            NULL, NULL);
        snprintf(name, sizeof(name), "duty-ns%d", n);
        object_property_add(obj, name, "uint64", na51068_pwm_get_ns, NULL,
                            NULL, NULL);
    }
}

static const VMStateDescription vmstate_na51068_pwm = {
    .name = TYPE_NVT_NA51068_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Na51068PwmState, PWM_NREGS),
        VMSTATE_UINT32_ARRAY(cntb, Na51068PwmState, PWM_NCH),
        VMSTATE_UINT32_ARRAY(cmpb, Na51068PwmState, PWM_NCH),
        VMSTATE_INT64_ARRAY(start_ns, Na51068PwmState, PWM_NCH),
        VMSTATE_END_OF_LIST()
    }
};

static void na51068_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, na51068_pwm_reset);
    dc->vmsd = &vmstate_na51068_pwm;
}

/* ---- IR ----------------------------------------------------------------- */

#define TYPE_NVT_NA51068_IR "nvt-na51068-ir"
OBJECT_DECLARE_SIMPLE_TYPE(Na51068IrState, NVT_NA51068_IR)

#define IR_CTRL             0x00
#define IR_STS              0x10
#define IR_DATA_LO          0x14
#define IR_DATA_HI          0x18
#define IR_INT              0x1C
#define IR_NREGS            (0x40 / 4)
#define IR_EN               (1u << 0)
#define IR_RD               (1u << 0)
#define IR_REP              (1u << 1)
#define IR_STS_EVENTS       7
#define IR_AMOUNT_SHIFT     3

struct Na51068IrState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[IR_NREGS];
};

static void na51068_ir_update(Na51068IrState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[IR_STS / 4] & s->regs[IR_INT / 4] &
                            IR_STS_EVENTS));
}

static uint64_t na51068_ir_read(void *opaque, hwaddr offset, unsigned size)
{
    Na51068IrState *s = opaque;

    if (offset < IR_NREGS * 4) {
        return s->regs[offset / 4];
    }
    qemu_log_mask(LOG_GUEST_ERROR, "nvt-na51068-ir: read 0x%" HWADDR_PRIx
                  "\n", offset);
    return 0;
}

static void na51068_ir_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    Na51068IrState *s = opaque;

    switch (offset) {
    case IR_STS:
        s->regs[IR_STS / 4] &= ~(val & IR_STS_EVENTS);
        break;
    case IR_DATA_LO:
    case IR_DATA_HI:
        return;
    default:
        if (offset >= IR_NREGS * 4) {
            return;
        }
        s->regs[offset / 4] = val;
        break;
    }
    na51068_ir_update(s);
}

static const MemoryRegionOps na51068_ir_ops = {
    .read = na51068_ir_read,
    .write = na51068_ir_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void na51068_ir_set_frame(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    Na51068IrState *s = NVT_NA51068_IR(obj);
    bool repeat = !strcmp(name, "repeat");
    uint64_t frame = 0;
    uint32_t bits;

    if (!visit_type_uint64(v, name, &frame, errp)) {
        return;
    }
    if (!(s->regs[IR_CTRL / 4] & IR_EN)) {
        return;     /* the decoder is off: nothing is received */
    }
    bits = frame >> 32 ? 64 : 32;
    s->regs[IR_DATA_LO / 4] = frame;
    s->regs[IR_DATA_HI / 4] = frame >> 32;
    s->regs[IR_STS / 4] = (s->regs[IR_STS / 4] & IR_STS_EVENTS) |
                          (repeat ? IR_REP : IR_RD) |
                          ((bits & 0x3F) << IR_AMOUNT_SHIFT);
    /* the decoder stops after each frame; the driver re-enables it */
    s->regs[IR_CTRL / 4] &= ~IR_EN;
    na51068_ir_update(s);
}

static void na51068_ir_reset(DeviceState *dev)
{
    Na51068IrState *s = NVT_NA51068_IR(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void na51068_ir_init(Object *obj)
{
    Na51068IrState *s = NVT_NA51068_IR(obj);

    memory_region_init_io(&s->iomem, obj, &na51068_ir_ops, s,
                          TYPE_NVT_NA51068_IR, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    object_property_add(obj, "frame", "uint64", NULL, na51068_ir_set_frame,
                        NULL, NULL);
    object_property_add(obj, "repeat", "uint64", NULL, na51068_ir_set_frame,
                        NULL, NULL);
}

static const VMStateDescription vmstate_na51068_ir = {
    .name = TYPE_NVT_NA51068_IR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Na51068IrState, IR_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void na51068_ir_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, na51068_ir_reset);
    dc->vmsd = &vmstate_na51068_ir;
}

/* ---- thermal ------------------------------------------------------------ */

#define TYPE_NVT_NA51068_THERMAL "nvt-thermal"
OBJECT_DECLARE_SIMPLE_TYPE(Na51068ThermalState, NVT_NA51068_THERMAL)

#define TH_CTRL             0x04
#define TH_VALUE            0x08
#define TH_NREGS            (0x40 / 4)
#define TH_EN               (1u << 0)

struct Na51068ThermalState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[TH_NREGS];
    int32_t temperature;
    uint32_t trim;
    uint32_t step;          /* milli-degrees per count */
};

static uint64_t na51068_thermal_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    Na51068ThermalState *s = opaque;

    if (offset == TH_VALUE) {
        /*
         * The inverse of thermal_get_temp(), T = (v - trim) * step / 1000
         * + 30 in C's truncating arithmetic: the v nearest the trim that
         * gives T.
         */
        int32_t d = s->temperature - 30, st = MAX(s->step, 1);
        int32_t steps = d >= 0 ? (d * 1000 + st - 1) / st
                               : -((-d * 1000 + st - 1) / st);

        return (s->regs[TH_CTRL / 4] & TH_EN) ? s->trim + steps : 0;
    }
    if (offset < TH_NREGS * 4) {
        return s->regs[offset / 4];
    }
    return 0;
}

static void na51068_thermal_write(void *opaque, hwaddr offset, uint64_t val,
                                  unsigned size)
{
    Na51068ThermalState *s = opaque;

    if (offset < TH_NREGS * 4 && offset != TH_VALUE) {
        s->regs[offset / 4] = val;
    }
}

static const MemoryRegionOps na51068_thermal_ops = {
    .read = na51068_thermal_read,
    .write = na51068_thermal_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void na51068_thermal_reset(DeviceState *dev)
{
    Na51068ThermalState *s = NVT_NA51068_THERMAL(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void thermal_get_temp(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp);
static void thermal_set_temp(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp);

static void na51068_thermal_init(Object *obj)
{
    Na51068ThermalState *s = NVT_NA51068_THERMAL(obj);

    memory_region_init_io(&s->iomem, obj, &na51068_thermal_ops, s,
                          TYPE_NVT_NA51068_THERMAL, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->temperature = 45;
    object_property_add(obj, "temperature", "int32", thermal_get_temp,
                        thermal_set_temp, NULL, NULL);
}

static void thermal_get_temp(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    Na51068ThermalState *s = NVT_NA51068_THERMAL(obj);

    visit_type_int32(v, name, &s->temperature, errp);
}

static void thermal_set_temp(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    Na51068ThermalState *s = NVT_NA51068_THERMAL(obj);
    int32_t t;

    if (!visit_type_int32(v, name, &t, errp)) {
        return;
    }
    if (t < -40 || t > 150) {
        error_setg(errp, "temperature %d C out of range (-40..150)", t);
        return;
    }
    s->temperature = t;     /* settable at run time: "qom-set" */
}

static const Property na51068_thermal_properties[] = {
    DEFINE_PROP_UINT32("trim", Na51068ThermalState, trim, 231),
    DEFINE_PROP_UINT32("step", Na51068ThermalState, step, 1319),
};

static const VMStateDescription vmstate_na51068_thermal = {
    .name = TYPE_NVT_NA51068_THERMAL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Na51068ThermalState, TH_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void na51068_thermal_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, na51068_thermal_reset);
    device_class_set_props(dc, na51068_thermal_properties);
    dc->vmsd = &vmstate_na51068_thermal;
}

static const TypeInfo na51068_misc_types[] = {
    {
        .name          = TYPE_NVT_NA51068_PWM,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Na51068PwmState),
        .instance_init = na51068_pwm_init,
        .class_init    = na51068_pwm_class_init,
    }, {
        .name          = TYPE_NVT_NA51068_IR,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Na51068IrState),
        .instance_init = na51068_ir_init,
        .class_init    = na51068_ir_class_init,
    }, {
        .name          = TYPE_NVT_NA51068_THERMAL,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Na51068ThermalState),
        .instance_init = na51068_thermal_init,
        .class_init    = na51068_thermal_class_init,
    },
};

DEFINE_TYPES(na51068_misc_types)
