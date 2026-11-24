/*
 * Grain Media GM8126 system blocks: the PMU (FTPMU010, 0x99000000) and the
 * AHB controller (FTAHBC010, 0x90000000).
 *
 * PMU (platform-GM8126/pmu.c, U-Boot 2013.01 board/gm/gm8126): 0x00 the
 * chip ID, which the kernel reads shifted right by 8 (0x812620 is the
 * GM8126 MP); 0x30 PLL1, written with MUL at 11:3, DIV at 16:12 and the
 * clock mode from bit 24 -- 1:0 what HCLK is made of (PLL1, / 2, / 3 or
 * PLL2 / 2) -- and read back shifted left by one on MP parts: every
 * reader of it uses 12:4, 17:13 and 25 (inferred; it is what makes
 * rom.bin's and the kernel's clocks agree).  0x34 PLL2, MUL 11:3 and DIV
 * 16:12.  Both multiply 30 MHz.  PCLK is HCLK / 2, the "pclk" output.
 * A frequency change goes as on the GM813x: 0x0C bit 2 starts it, the core
 * waits in WFI for interrupt 8, whose handler clears 0x20 bit 17.  The
 * interrupt comes a little later, or the core would take it before it
 * sleeps and never wake.
 * The rest (gates, pin mux, dividers) is storage.  The reset values give
 * PLL1 540 MHz and HCLK 180 MHz: assumed, the real ones are a strap's.
 *
 * AHBC (second region): 0x18 the DDR slot, base in 31:20 and the size as
 * 1 MiB << [19:16], which the capture driver reads; 0x88 bit 0 remaps DDR
 * from 0x10000000 to 0 (it is at both here).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "hw/clock.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_SYS8126 "gm-sys8126"
OBJECT_DECLARE_SIMPLE_TYPE(GmSys8126State, GM_SYS8126)

#define SYS_SIZE    0x1000
#define SYS_NREGS   (0x100 / 4)
#define SYS_OSC     30000000
#define PMU_FCS_DONE (1u << 17)
#define PMU_FCS_NS   200000

struct GmSys8126State {
    SysBusDevice parent_obj;

    MemoryRegion pmu_mem;
    MemoryRegion ahbc_mem;
    Clock *pclk;
    qemu_irq fcs_irq;
    QEMUTimer *fcs_timer;
    uint32_t chip_id;
    uint64_t ram_size;
    uint32_t pmu[SYS_NREGS];
    uint32_t ahbc[SYS_NREGS];
};

static void sys8126_clocks(GmSys8126State *s)
{
    const uint32_t p1 = s->pmu[0x30 / 4], p2 = s->pmu[0x34 / 4];
    const uint32_t d1 = MAX(extract32(p1, 12, 5), 1);
    const uint32_t d2 = MAX(extract32(p2, 12, 5), 1);
    const uint64_t pll1 = (uint64_t)SYS_OSC / d1 * extract32(p1, 3, 9);
    const uint64_t pll2 = (uint64_t)SYS_OSC / d2 * extract32(p2, 3, 9);
    uint64_t hclk;

    switch (extract32(p1, 24, 2)) {
    case 0:
        hclk = pll1;
        break;
    case 1:
        hclk = pll1 / 2;
        break;
    case 2:
        hclk = pll1 / 3;
        break;
    default:
        hclk = pll2 / 2;
        break;
    }
    clock_update_hz(s->pclk, MAX(hclk / 2, 1000000));
}

static void sys8126_fcs_done(void *opaque)
{
    GmSys8126State *s = opaque;

    s->pmu[0x20 / 4] |= PMU_FCS_DONE;
    qemu_set_irq(s->fcs_irq, 1);
}

static uint64_t sys8126_pmu_read(void *opaque, hwaddr off, unsigned size)
{
    GmSys8126State *s = opaque;

    if (off == 0x30 && s->chip_id >> 8 != 0x812610) {
        return s->pmu[off / 4] << 1;
    }
    return off / 4 < SYS_NREGS ? s->pmu[off / 4] : 0;
}

static void sys8126_pmu_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    GmSys8126State *s = opaque;

    if (!off || off / 4 >= SYS_NREGS) {
        return;
    }
    if (off == 0x20) {
        s->pmu[off / 4] &= ~val;
        qemu_set_irq(s->fcs_irq, (s->pmu[off / 4] & PMU_FCS_DONE) != 0);
        return;
    }
    s->pmu[off / 4] = val;
    if (off == 0x0C && (val & (1u << 2))) {
        s->pmu[0x0C / 4] &= ~(1u << 2);
        timer_mod(s->fcs_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                PMU_FCS_NS);
    }
    if (off == 0x30 || off == 0x34) {
        sys8126_clocks(s);
    }
}

static uint64_t sys8126_ahbc_read(void *opaque, hwaddr off, unsigned size)
{
    GmSys8126State *s = opaque;

    return off / 4 < SYS_NREGS ? s->ahbc[off / 4] : 0;
}

static void sys8126_ahbc_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    GmSys8126State *s = opaque;

    if (off / 4 < SYS_NREGS) {
        s->ahbc[off / 4] = val;
    }
}

static const MemoryRegionOps sys8126_pmu_ops = {
    .read = sys8126_pmu_read,
    .write = sys8126_pmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const MemoryRegionOps sys8126_ahbc_ops = {
    .read = sys8126_ahbc_read,
    .write = sys8126_ahbc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void sys8126_reset(DeviceState *dev)
{
    GmSys8126State *s = GM_SYS8126(dev);

    timer_del(s->fcs_timer);
    memset(s->pmu, 0, sizeof(s->pmu));
    memset(s->ahbc, 0, sizeof(s->ahbc));
    s->pmu[0] = s->chip_id;
    s->pmu[0x30 / 4] = 18 << 3 | 1 << 12 | (2u << 2 | 2) << 24;
    s->pmu[0x34 / 4] = 20 << 3 | 1 << 12;
    s->ahbc[0x18 / 4] = ctz64(s->ram_size / MiB) << 16;
    qemu_set_irq(s->fcs_irq, 0);
    sys8126_clocks(s);
}

static void sys8126_init(Object *obj)
{
    GmSys8126State *s = GM_SYS8126(obj);

    s->pclk = qdev_init_clock_out(DEVICE(obj), "pclk");
}

static void sys8126_realize(DeviceState *dev, Error **errp)
{
    GmSys8126State *s = GM_SYS8126(dev);

    memory_region_init_io(&s->pmu_mem, OBJECT(s), &sys8126_pmu_ops, s,
                          TYPE_GM_SYS8126 ".pmu", SYS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->pmu_mem);
    memory_region_init_io(&s->ahbc_mem, OBJECT(s), &sys8126_ahbc_ops, s,
                          TYPE_GM_SYS8126 ".ahbc", SYS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->ahbc_mem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->fcs_irq);
    s->fcs_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sys8126_fcs_done, s);
}

static const Property sys8126_props[] = {
    DEFINE_PROP_UINT32("chip-id", GmSys8126State, chip_id, 0x81262000),
    DEFINE_PROP_UINT64("ram-size", GmSys8126State, ram_size, 128 * MiB),
};

static const VMStateDescription vmstate_sys8126 = {
    .name = TYPE_GM_SYS8126,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(fcs_timer, GmSys8126State),
        VMSTATE_CLOCK(pclk, GmSys8126State),
        VMSTATE_UINT32_ARRAY(pmu, GmSys8126State, SYS_NREGS),
        VMSTATE_UINT32_ARRAY(ahbc, GmSys8126State, SYS_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void sys8126_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sys8126_realize;
    device_class_set_legacy_reset(dc, sys8126_reset);
    device_class_set_props(dc, sys8126_props);
    dc->vmsd = &vmstate_sys8126;
}

static const TypeInfo sys8126_info = {
    .name = TYPE_GM_SYS8126,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GmSys8126State),
    .instance_init = sys8126_init,
    .class_init = sys8126_class_init,
};

static void sys8126_register_types(void)
{
    type_register_static(&sys8126_info);
}

type_init(sys8126_register_types)
