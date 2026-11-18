/*
 * Novatek ADC with its thermal sensor: NA51089 / NA51055 at 0xF0260000
 * (SPI 47), NS02501 at 0xF01D0000 (SPI 73).  drivers/iio/adc/nvt_adc.c,
 * plat-na51055/adc_reg.h:
 *
 *   0x00 CTRL      [3:0] channel n continuous (1) or one-shot, [16:12]
 *                  clock divider, [21:20] averaging, bit 23 enable,
 *                  [27:24] extra sample clocks
 *   0x04 ONESHOT   bit n: convert channel n once (self-clearing)
 *   0x08 INTCTRL   [3:0] data-ready enables, [6:4] value-trigger enables
 *   0x0C STATUS    [3:0] data ready, [6:4] value triggers; write 1 to clear
 *   0x10..0x1C     value triggers (storage)
 *   0x20 DIV0      per-channel dividers (storage)
 *   0x30 + 4n      AIN n data, 9 bits
 *   0x40 THERMAL   [2:0] average count, [5:4] its channel, [24:16] the
 *                  averaged thermal reading
 *
 * "layout" 1 is NS02501's variant (plat-ns02501_a32/adc_reg.h): a fifth,
 * bandgap channel (CTRL bit 9 continuous, bit 22 power-down, ONESHOT bit
 * 4, data at 0x84, status bit 7), INTCTRL at 0x8C and STATUS at 0x90, and
 * 0x40 an averaging register for any channel ([6:4] the channel, 4 the
 * bandgap; [24:16] its average) rather than the thermal reading.  The
 * bandgap reads "bandgap" (default 256).
 *
 * The inputs are the QOM properties "ain0".."ain3" (0..511, default 256,
 * half of the reference).  With the converter enabled, a continuous
 * channel's data-ready bit sets every millisecond, a one-shot's at once;
 * the interrupt is STATUS & INTCTRL.  The kernel reads the inputs by
 * polling (nvt_adc_poll_mode 1) and only enables the interrupt.
 *
 * The thermal reading is "trim" + the counts for "temperature" (degrees C,
 * default 45) at "step" milli-degrees a count, the inverse of
 * thermal_get_temp(): (reading - trim) * 1319 / 1000 + 30 on NA51089 and
 * NA51084, * 13 / 10 on NA51055.  The kernel's trim, with a blank efuse,
 * is 231 for NA51089 / NA51084 and 259 for NA51055
 * (thermal_get_trimdata()).
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

#define TYPE_NVT_ADC "nvt-adc"
OBJECT_DECLARE_SIMPLE_TYPE(NvtAdcState, NVT_ADC)

#define ADC_CTRL            0x00
#define ADC_ONESHOT         0x04
#define ADC_INTCTRL         0x08
#define ADC_STATUS          0x0C
#define ADC_DATA0           0x30
#define ADC_THERMAL         0x40        /* layout 1: the average */
#define ADC_BANDGAP         0x84        /* layout 1 */
#define ADC_INTCTRL_1       0x8C
#define ADC_STATUS_1        0x90
#define ADC_NREGS           (0x100 / 4)
#define ADC_CHANNELS        4

#define CTRL_EN             (1u << 23)
#define STATUS_BITS         0xFF
#define THERMAL_OUT_SHIFT   16
#define THERMAL_OUT_MASK    (0x1FFu << THERMAL_OUT_SHIFT)
#define ADC_PERIOD_NS       1000000     /* a continuous channel's rate */

struct NvtAdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[ADC_NREGS];
    uint32_t ain[ADC_CHANNELS];
    int32_t temperature;
    uint32_t trim;
    uint32_t step;
    uint32_t bandgap;
    uint8_t layout;
};

/* Where INTCTRL / STATUS are, and which status bits continuous channels
 * set: [3:0], and on NS02501 the bandgap's bit 7 for CTRL bit 9. */
static hwaddr adc_intctrl(NvtAdcState *s)
{
    return s->layout ? ADC_INTCTRL_1 : ADC_INTCTRL;
}

static hwaddr adc_status(NvtAdcState *s)
{
    return s->layout ? ADC_STATUS_1 : ADC_STATUS;
}

static uint32_t adc_cont_bits(NvtAdcState *s)
{
    uint32_t ctrl = s->regs[ADC_CTRL / 4], bits = ctrl & 0xF;

    if (s->layout && (ctrl & (1u << 9))) {
        bits |= 1u << 7;
    }
    return bits;
}

static void nvt_adc_update(NvtAdcState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[adc_status(s) / 4] &
                            s->regs[adc_intctrl(s) / 4] & STATUS_BITS));
}

static void nvt_adc_schedule(NvtAdcState *s)
{
    uint32_t ctrl = s->regs[ADC_CTRL / 4];

    if ((ctrl & CTRL_EN) && adc_cont_bits(s)) {
        if (!timer_pending(s->timer)) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      ADC_PERIOD_NS);
        }
    } else {
        timer_del(s->timer);
    }
}

static void nvt_adc_tick(void *opaque)
{
    NvtAdcState *s = opaque;

    s->regs[adc_status(s) / 4] |= adc_cont_bits(s);
    nvt_adc_update(s);
    nvt_adc_schedule(s);
}

static uint32_t nvt_adc_thermal_counts(NvtAdcState *s)
{
    int32_t d = s->temperature - 30, st = MAX(s->step, 1);
    int32_t steps = d >= 0 ? (d * 1000 + st - 1) / st
                           : -((-d * 1000 + st - 1) / st);

    return (uint32_t)((int32_t)s->trim + steps) & 0x1FF;
}

static uint64_t nvt_adc_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtAdcState *s = opaque;
    bool en = s->regs[ADC_CTRL / 4] & CTRL_EN;

    if (offset >= ADC_DATA0 && offset < ADC_DATA0 + 4 * ADC_CHANNELS) {
        return en ? s->ain[(offset - ADC_DATA0) / 4] & 0x1FF : 0;
    }
    if (s->layout && offset == ADC_BANDGAP) {
        return en ? s->bandgap & 0x1FF : 0;
    }
    if (s->layout && offset == ADC_THERMAL) {
        uint32_t id = (s->regs[ADC_THERMAL / 4] >> 4) & 7;
        uint32_t v = id < ADC_CHANNELS ? s->ain[id] : s->bandgap;

        return (s->regs[ADC_THERMAL / 4] & ~THERMAL_OUT_MASK) |
               (en ? (v & 0x1FF) << THERMAL_OUT_SHIFT : 0);
    }
    switch (offset) {
    case ADC_ONESHOT:
        return 0;
    case ADC_THERMAL:
        return (s->regs[ADC_THERMAL / 4] & ~THERMAL_OUT_MASK) |
               (en ? nvt_adc_thermal_counts(s) << THERMAL_OUT_SHIFT : 0);
    default:
        if (offset < ADC_NREGS * 4) {
            return s->regs[offset / 4];
        }
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-adc: read 0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void nvt_adc_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    NvtAdcState *s = opaque;

    if (offset == adc_status(s)) {
        s->regs[offset / 4] &= ~(val & STATUS_BITS);
        nvt_adc_update(s);
        return;
    }
    switch (offset) {
    case ADC_ONESHOT:
        if (s->regs[ADC_CTRL / 4] & CTRL_EN) {
            s->regs[adc_status(s) / 4] |= (val & 0xF) |
                (s->layout && (val & 0x10) ? 1u << 7 : 0);
        }
        break;
    case ADC_BANDGAP:
        if (s->layout) {
            return;
        }
        s->regs[offset / 4] = val;
        break;
    default:
        if (offset >= ADC_NREGS * 4 ||
            (offset >= ADC_DATA0 && offset < ADC_DATA0 + 4 * ADC_CHANNELS)) {
            return;
        }
        s->regs[offset / 4] = val;
        break;
    }
    nvt_adc_schedule(s);
    nvt_adc_update(s);
}

static const MemoryRegionOps nvt_adc_ops = {
    .read = nvt_adc_read,
    .write = nvt_adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_adc_get_ain(Object *obj, Visitor *v, const char *name,
                            void *opaque, Error **errp)
{
    NvtAdcState *s = NVT_ADC(obj);
    uint32_t val = s->ain[name[3] - '0'];

    visit_type_uint32(v, name, &val, errp);
}

static void nvt_adc_set_ain(Object *obj, Visitor *v, const char *name,
                            void *opaque, Error **errp)
{
    NvtAdcState *s = NVT_ADC(obj);
    uint32_t val;

    if (!visit_type_uint32(v, name, &val, errp)) {
        return;
    }
    if (val > 511) {
        error_setg(errp, "%s: the ADC is 9 bits (0..511)", name);
        return;
    }
    s->ain[name[3] - '0'] = val;
}

static void nvt_adc_reset(DeviceState *dev)
{
    NvtAdcState *s = NVT_ADC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->timer);
    qemu_irq_lower(s->irq);
}

static void adc_get_temp(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp);
static void adc_set_temp(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp);

static void nvt_adc_init(Object *obj)
{
    NvtAdcState *s = NVT_ADC(obj);
    char name[8];
    int n;

    memory_region_init_io(&s->iomem, obj, &nvt_adc_ops, s, TYPE_NVT_ADC,
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_adc_tick, s);
    s->temperature = 45;
    object_property_add(obj, "temperature", "int32", adc_get_temp,
                        adc_set_temp, NULL, NULL);
    for (n = 0; n < ADC_CHANNELS; n++) {
        s->ain[n] = 256;
        snprintf(name, sizeof(name), "ain%d", n);
        object_property_add(obj, name, "uint32", nvt_adc_get_ain,
                            nvt_adc_set_ain, NULL, NULL);
    }
}

static void adc_get_temp(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    NvtAdcState *s = NVT_ADC(obj);

    visit_type_int32(v, name, &s->temperature, errp);
}

static void adc_set_temp(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    NvtAdcState *s = NVT_ADC(obj);
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

static const Property nvt_adc_properties[] = {
    DEFINE_PROP_UINT32("trim", NvtAdcState, trim, 231),
    DEFINE_PROP_UINT32("step", NvtAdcState, step, 1319),
    DEFINE_PROP_UINT32("bandgap", NvtAdcState, bandgap, 256),
    DEFINE_PROP_UINT8("layout", NvtAdcState, layout, 0),
};

static const VMStateDescription vmstate_nvt_adc = {
    .name = TYPE_NVT_ADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtAdcState, ADC_NREGS),
        VMSTATE_UINT32_ARRAY(ain, NvtAdcState, ADC_CHANNELS),
        VMSTATE_TIMER_PTR(timer, NvtAdcState),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_adc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_adc_reset);
    device_class_set_props(dc, nvt_adc_properties);
    dc->vmsd = &vmstate_nvt_adc;
}

static const TypeInfo nvt_adc_info = {
    .name          = TYPE_NVT_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtAdcState),
    .instance_init = nvt_adc_init,
    .class_init    = nvt_adc_class_init,
};

static void nvt_adc_register_types(void)
{
    type_register_static(&nvt_adc_info);
}

type_init(nvt_adc_register_types)
