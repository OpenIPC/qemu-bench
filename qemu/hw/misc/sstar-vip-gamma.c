/*
 * MStar infinity3 VIP YUV gamma SRAM port (bank 0x1226 regs 0x40-0x4F).
 *
 * The scaler's VIP block keeps its Y, U and V gamma curves in SRAMs
 * reached through an indirect port in the SCNR bank (0x122600).  The
 * vendor kernel's Hal_VIP_SRAMDumpGammaYUV (drivers/mstar/scl/infinity3/
 * src/mxlib/hal/vip/halvip.c, u32reg = REG_VIP_SCNR_41_L) writes each
 * entry and, outside CMDQ mode, reads it back:
 *
 *   0x40  read: [5:3] the table (0/1 Y even/odd, 2/3 U, 4/5 V), bit 2 ren
 *   0x41  write enables: Y [1:0], U [3:2], V [5:4] (even, odd)
 *   0x42 / 0x43 / 0x44  the Y / U / V entry address, [6:0] (and [14:8])
 *   0x45 / 0x46  Y even / odd data, 12 bits; 0x47 / 0x48 U; 0x49 / 0x4A V
 *   0x4F  the entry ren last read
 *
 * A mismatch makes the driver wait up to 2 s for a scaler frame end and
 * write the entry again, ten times per entry, so without the SRAM the
 * kernel's boot stalls for hours in the scaler's probe.  Write enables
 * take effect on every write that has them set (the driver sets them
 * with a masked write and never clears them).
 *
 * sstar-rgb-gamma is the same for the RGB gamma SRAMs (Hal_VIP_SRAMDump-
 * GammaRGB, bank 0x1211 = REG_SCL_HVSP1): two ports of three registers,
 * GM10to12 at reg 0x7A and GM12to10 at reg 0x7D:
 *
 *   +0  [7:0] the entry, [10:8] the table (R/G/B even/odd), bit 11
 *       write, bit 12 read
 *   +1  the data to write, 12 bits
 *   +2  the entry last read
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_SSTAR_VIP_GAMMA "sstar-vip-gamma"
OBJECT_DECLARE_SIMPLE_TYPE(SstarVipGammaState, SSTAR_VIP_GAMMA)

#define GM_NREGS            0x10        /* bank regs 0x40-0x4F */
#define GM_RIDX             0x00
#define GM_WEN              0x01
#define GM_ADDR_Y           0x02
#define GM_DATA_Y           0x05
#define GM_RDATA            0x0F

#define GM_RIDX_REN         (1u << 2)
#define GM_TABLES           6           /* Y, U, V, each even and odd */
#define GM_ENTRIES          128

struct SstarVipGammaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint16_t regs[GM_NREGS];
    uint16_t sram[GM_TABLES][GM_ENTRIES];
};

static void sstar_vip_gamma_write_reg(SstarVipGammaState *s, int reg,
                                      uint16_t v)
{
    int c;

    if (reg == GM_RDATA) {
        return;                         /* read only */
    }
    s->regs[reg] = v;
    switch (reg) {
    case GM_WEN:
        for (c = 0; c < 3; c++) {       /* Y, U, V */
            uint8_t addr = s->regs[GM_ADDR_Y + c] & (GM_ENTRIES - 1);

            if (v & (1u << (2 * c))) {
                s->sram[2 * c][addr] = s->regs[GM_DATA_Y + 2 * c] & 0xFFF;
            }
            if (v & (2u << (2 * c))) {
                s->sram[2 * c + 1][addr] =
                    s->regs[GM_DATA_Y + 2 * c + 1] & 0xFFF;
            }
        }
        break;
    case GM_RIDX:
        if (v & GM_RIDX_REN) {
            int t = (v >> 3) & 7;

            if (t < GM_TABLES) {
                uint8_t addr = s->regs[GM_ADDR_Y + t / 2] & (GM_ENTRIES - 1);

                s->regs[GM_RDATA] = s->sram[t][addr];
            }
        }
        break;
    }
}

static uint64_t sstar_vip_gamma_read(void *opaque, hwaddr off, unsigned size)
{
    SstarVipGammaState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->regs[(off >> 2) & (GM_NREGS - 1)];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_vip_gamma_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    SstarVipGammaState *s = opaque;
    int reg = (off >> 2) & (GM_NREGS - 1);
    uint16_t v = val;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (s->regs[reg] & 0x00FF) | ((val & 0xFF) << 8)
                    : (s->regs[reg] & 0xFF00) | (val & 0xFF);
    }
    sstar_vip_gamma_write_reg(s, reg, v);
}

static const MemoryRegionOps sstar_vip_gamma_ops = {
    .read = sstar_vip_gamma_read,
    .write = sstar_vip_gamma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_vip_gamma_reset(DeviceState *dev)
{
    SstarVipGammaState *s = SSTAR_VIP_GAMMA(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void sstar_vip_gamma_init(Object *obj)
{
    SstarVipGammaState *s = SSTAR_VIP_GAMMA(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_vip_gamma_ops, s,
                          TYPE_SSTAR_VIP_GAMMA, GM_NREGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_sstar_vip_gamma = {
    .name = TYPE_SSTAR_VIP_GAMMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarVipGammaState, GM_NREGS),
        VMSTATE_UINT16_2DARRAY(sram, SstarVipGammaState, GM_TABLES,
                               GM_ENTRIES),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_vip_gamma_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_vip_gamma_reset);
    dc->vmsd = &vmstate_sstar_vip_gamma;
}

static const TypeInfo sstar_vip_gamma_info = {
    .name           = TYPE_SSTAR_VIP_GAMMA,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarVipGammaState),
    .instance_init  = sstar_vip_gamma_init,
    .class_init     = sstar_vip_gamma_class_init,
};

#define TYPE_SSTAR_RGB_GAMMA "sstar-rgb-gamma"
OBJECT_DECLARE_SIMPLE_TYPE(SstarRgbGammaState, SSTAR_RGB_GAMMA)

#define RGB_PORTS           2
#define RGB_NREGS           (RGB_PORTS * 3)
#define RGB_ENTRIES         256
#define RGB_CTRL_WEN        (1u << 11)
#define RGB_CTRL_REN        (1u << 12)

struct SstarRgbGammaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint16_t regs[RGB_NREGS];
    uint16_t sram[RGB_PORTS][GM_TABLES][RGB_ENTRIES];
};

static void sstar_rgb_gamma_write_reg(SstarRgbGammaState *s, int reg,
                                      uint16_t v)
{
    int port = reg / 3;
    int t = (v >> 8) & 7;
    uint8_t addr = v & 0xFF;

    switch (reg % 3) {
    case 0:
        s->regs[reg] = v;
        if (t >= GM_TABLES) {
            break;
        }
        if (v & RGB_CTRL_WEN) {
            s->sram[port][t][addr] = s->regs[reg + 1] & 0xFFF;
        }
        if (v & RGB_CTRL_REN) {
            s->regs[reg + 2] = s->sram[port][t][addr];
        }
        break;
    case 1:
        s->regs[reg] = v;
        break;
    }
}

static uint64_t sstar_rgb_gamma_read(void *opaque, hwaddr off, unsigned size)
{
    SstarRgbGammaState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2 || off >= RGB_NREGS * 4) {
        return 0;
    }
    v = s->regs[off >> 2];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_rgb_gamma_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    SstarRgbGammaState *s = opaque;
    int reg = off >> 2;
    uint16_t v = val;

    if ((off & 3) >= 2 || off >= RGB_NREGS * 4) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (s->regs[reg] & 0x00FF) | ((val & 0xFF) << 8)
                    : (s->regs[reg] & 0xFF00) | (val & 0xFF);
    }
    sstar_rgb_gamma_write_reg(s, reg, v);
}

static const MemoryRegionOps sstar_rgb_gamma_ops = {
    .read = sstar_rgb_gamma_read,
    .write = sstar_rgb_gamma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_rgb_gamma_reset(DeviceState *dev)
{
    SstarRgbGammaState *s = SSTAR_RGB_GAMMA(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void sstar_rgb_gamma_init(Object *obj)
{
    SstarRgbGammaState *s = SSTAR_RGB_GAMMA(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_rgb_gamma_ops, s,
                          TYPE_SSTAR_RGB_GAMMA, RGB_NREGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_sstar_rgb_gamma = {
    .name = TYPE_SSTAR_RGB_GAMMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarRgbGammaState, RGB_NREGS),
        VMSTATE_BUFFER_UNSAFE(sram, SstarRgbGammaState, 0,
                              sizeof(((SstarRgbGammaState *)0)->sram)),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_rgb_gamma_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_rgb_gamma_reset);
    dc->vmsd = &vmstate_sstar_rgb_gamma;
}

static const TypeInfo sstar_rgb_gamma_info = {
    .name           = TYPE_SSTAR_RGB_GAMMA,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarRgbGammaState),
    .instance_init  = sstar_rgb_gamma_init,
    .class_init     = sstar_rgb_gamma_class_init,
};

static void sstar_vip_gamma_register_types(void)
{
    type_register_static(&sstar_vip_gamma_info);
    type_register_static(&sstar_rgb_gamma_info);
}

type_init(sstar_vip_gamma_register_types)
