/*
 * Ingenic PWM: T41's eight-channel PWM controller at 0x13460000.
 *
 * T41 drives PWM from this block instead of the TCU channels of the
 * other parts (drivers/pwm/pwm-ingenic.c of the 4.4 kernel, "ingenic,
 * t41-pwm" in t41.dtsi, core INTC source 63):
 *
 *   0x000 CCFG0  0x004 CCFG1   per-channel clock dividers, 4 bits each
 *   0x010 ENS    0x014 ENC     set / clear channel enables, write only
 *   0x018 EN     the channel enables
 *   0x020 UPDATE write 1 << n: channel n takes its new WCFG
 *   0x024 BUSYR  bit n while channel n is still taking an update
 *   0x030 LVL    the idle level of each channel
 *   0x0B0 + 4 n  WCFG n: high cycles << 16 | low cycles
 *   0x100 DES    0x104 DEC     0x108 DE    DMA-mode enables, as ENS/ENC/EN
 *   0x110.. DCR0/1, DTRIG, DFER, DFSM, DSR (0x134 DSCR clears it), DINTC,
 *           DAR/DTLR n        DMA mode, storage: the driver does not
 *           use DMA mode, and no DMA is done here
 *   0x300 OEN    output enables
 *
 * An update completes at once (BUSYR reads 0; the driver spins on it
 * only while the channel is enabled).  While a channel and its output
 * are enabled, the WCFG it last took gives the waveform; otherwise its
 * pin sits at LVL.  The read-only "chanN" properties report the pin as
 * "high/period" in cycles, or "0" / "1" for a fixed level.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_PWM "ingenic-pwm"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicPwmState, INGENIC_PWM)

#define PWM_REGS        0x400
#define NCHAN           8

#define R_ENS           0x010
#define R_ENC           0x014
#define R_EN            0x018
#define R_UPDATE        0x020
#define R_BUSYR         0x024
#define R_LVL           0x030
#define R_WCFG          0x0B0
#define R_DES           0x100
#define R_DEC           0x104
#define R_DE            0x108
#define R_DSR           0x130
#define R_DSCR          0x134
#define R_OEN           0x300

struct IngenicPwmState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[PWM_REGS / 4];
    uint32_t wcfg[NCHAN];           /* the WCFG each channel has taken */
};

static uint32_t *pwm_reg(IngenicPwmState *s, hwaddr off)
{
    return &s->regs[off >> 2];
}

static uint64_t ingenic_pwm_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicPwmState *s = opaque;

    switch (off) {
    case R_ENS: case R_ENC: case R_UPDATE: case R_BUSYR:
    case R_DES: case R_DEC: case R_DSCR:
        return 0;
    }
    return off < PWM_REGS ? *pwm_reg(s, off) : 0;
}

static void ingenic_pwm_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicPwmState *s = opaque;
    int n;

    if (off >= PWM_REGS) {
        return;
    }
    switch (off) {
    case R_ENS:
        *pwm_reg(s, R_EN) |= val & 0xff;
        break;
    case R_ENC:
        *pwm_reg(s, R_EN) &= ~val;
        break;
    case R_DES:
        *pwm_reg(s, R_DE) |= val & 0xff;
        break;
    case R_DEC:
        *pwm_reg(s, R_DE) &= ~val;
        break;
    case R_DSCR:
        *pwm_reg(s, R_DSR) &= ~val;
        break;
    case R_UPDATE:
        for (n = 0; n < NCHAN; n++) {
            if (val & (1u << n)) {
                s->wcfg[n] = *pwm_reg(s, R_WCFG + 4 * n);
            }
        }
        break;
    case R_EN: case R_BUSYR: case R_DE: case R_DSR:
        break;                          /* read only */
    default:
        *pwm_reg(s, off) = val;
        break;
    }
}

static const MemoryRegionOps ingenic_pwm_ops = {
    .read = ingenic_pwm_read,
    .write = ingenic_pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_pwm_get_chan(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    IngenicPwmState *s = INGENIC_PWM(obj);
    int n = (uintptr_t)opaque;
    uint32_t on = *pwm_reg(s, R_EN) & *pwm_reg(s, R_OEN);
    uint32_t high = s->wcfg[n] >> 16, low = s->wcfg[n] & 0xffff;
    g_autofree char *str = NULL;

    if (!(on & (1u << n))) {
        str = g_strdup(extract32(*pwm_reg(s, R_LVL), n, 1) ? "1" : "0");
    } else if (!high || !low) {
        str = g_strdup(high ? "1" : "0");   /* a duty of 100 % or 0 % */
    } else {
        str = g_strdup_printf("%u/%u", high, high + low);
    }
    visit_type_str(v, name, &str, errp);
}

static void ingenic_pwm_reset(DeviceState *dev)
{
    IngenicPwmState *s = INGENIC_PWM(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->wcfg, 0, sizeof(s->wcfg));
}

static void ingenic_pwm_init(Object *obj)
{
    IngenicPwmState *s = INGENIC_PWM(obj);
    uintptr_t n;

    memory_region_init_io(&s->iomem, obj, &ingenic_pwm_ops, s,
                          TYPE_INGENIC_PWM, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (n = 0; n < NCHAN; n++) {
        g_autofree char *name = g_strdup_printf("chan%u", (unsigned)n);

        object_property_add(obj, name, "str", ingenic_pwm_get_chan,
                            NULL, NULL, (void *)n);
    }
}

static const VMStateDescription vmstate_ingenic_pwm = {
    .name = TYPE_INGENIC_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicPwmState, PWM_REGS / 4),
        VMSTATE_UINT32_ARRAY(wcfg, IngenicPwmState, NCHAN),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_pwm_reset);
    dc->vmsd = &vmstate_ingenic_pwm;
}

static const TypeInfo ingenic_pwm_info = {
    .name = TYPE_INGENIC_PWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicPwmState),
    .instance_init = ingenic_pwm_init,
    .class_init = ingenic_pwm_class_init,
};

static void ingenic_pwm_register_types(void)
{
    type_register_static(&ingenic_pwm_info);
}

type_init(ingenic_pwm_register_types)
