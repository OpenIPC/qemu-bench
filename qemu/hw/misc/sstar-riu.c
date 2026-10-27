/*
 * SigmaStar / MStar RIU (register interface unit) catch-all.
 *
 * Every SigmaStar block sits behind the RIU at 0x1F000000: banks of 128
 * 16-bit registers, each register on a 4-byte stride, so register r of
 * bank b is at 0x1F000000 + b * 0x200 + r * 4.  The vendor code names
 * banks by their RIU number (PM banks 0x0000-0x0FFF, the rest from
 * 0x1000 at 0x1F200000: "BASE_REG_CHIPTOP_PA ... 0x101E00" is bank
 * 0x101E).  A register's high byte is at +1 (INREG8(reg + 1)).
 *
 * This device is the storage behind every bank no model claims, mapped
 * under them: firmware read-modify-writes clock gates, pad muxes and PLL
 * words and expects to read its own writes back.  On top of that it
 * answers the few status bits the IPL, U-Boot and the kernel poll on
 * otherwise unmodelled banks -- PLL locks, MIU (DDR controller)
 * initialisation done, the chip ID -- each with the vendor source that
 * waits on it.  The machine maps the modelled blocks over it.
 *
 * LOG_UNIMP reports the first access to each bank no model claims, and
 * a "hot poll": one register read 64k times in a row, which is firmware
 * waiting on hardware that is not modelled.  The "trace" property, a
 * comma-separated list of bank numbers (-global sstar-riu.trace=0x1246),
 * logs every access to those banks.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/cutils.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "hw/misc/sstar-riu.h"
#include "system/runstate.h"

#define RIU_NREGS           (SSTAR_RIU_SIZE / 4)
#define RIU_NBANKS          (SSTAR_RIU_SIZE / 0x200)

/* bank 0x0E PM_SLEEP: reg 0x2E is the software reset key (reset.S) */
#define BANK_PMSLEEP        0x000E
#define PMSLEEP_SWRST       0x2E

static inline uint32_t riu_bank(hwaddr off)
{
    return off >> 9;
}

static inline uint32_t riu_reg(hwaddr off)
{
    return (off >> 2) & 0x7F;
}

uint16_t sstar_riu_get(SstarRiuState *s, uint32_t bank, uint32_t reg)
{
    return s->regs[(bank << 7) | (reg & 0x7F)];
}

static uint16_t sstar_riu_status(SstarRiuState *s, uint32_t bank,
                                 uint32_t reg, uint16_t v);

uint16_t sstar_riu_read16(SstarRiuState *s, uint32_t bank, uint32_t reg)
{
    return sstar_riu_status(s, bank, reg, sstar_riu_get(s, bank, reg));
}

void sstar_riu_set(SstarRiuState *s, uint32_t bank, uint32_t reg,
                   uint16_t val)
{
    s->regs[(bank << 7) | (reg & 0x7F)] = val;
}

/*
 * What reads differently from what was written: status bits the boot
 * chain spins on and read-only straps, on banks no model claims.  The
 * MIU and l3bridge answers are the ones Daniel Palmer found the SSD202D
 * boot chain blocking on (github.com/fifteenhex/qemu mstar-clean,
 * docs/system/arm/mstarv7/ddr.rst); DRAM here needs no training.
 */
static uint16_t sstar_riu_status(SstarRiuState *s, uint32_t bank,
                                 uint32_t reg, uint16_t v)
{
    switch (bank) {
    case 0x1012:            /* MIU digital */
        if (reg == 0x00) {
            v |= 0x8000;    /* CNTRL0: DRAM init done */
        } else if (reg == 0x70) {
            v |= 0x8000;    /* BIST control: done, no error */
        }
        break;
    case 0x1032:            /* CPU PLL: reg 0x5D bit 0 = LPF glide done */
        /*
         * The cpufreq driver moves the ARM PLL by its low-pass filter:
         * the target into regs 0x50-0x55, then it waits for this bit.
         */
        if (reg == 0x5D) {
            v |= 0x0001;
        }
        break;
    case 0x1022:            /* l3bridge: reg 0x10 bit 12 = flush done */
        if (reg == 0x10) {
            v |= 0x1000;
        }
        break;
    case 0x1421:            /* UTMI0-2 PHYs (ehci-mstar-chip.h) */
    case 0x1425:
    case 0x1429:
        /*
         * reg 0x1E: analog calibration.  ehci-mstar.c sets CA_START (bit
         * 0), spins on CA_END (bit 1), and takes a result in [15:4] of
         * all zeroes or all ones for a failed calibration.
         */
        if (reg == 0x1E) {
            v |= 0x0002;
            if ((v & 0xFFF0) == 0 || (v & 0xFFF0) == 0xFFF0) {
                v = (v & 0x000F) | 0x0600;
            }
        }
        break;
    case 0x0031:            /* ALBANY0: the EMAC's internal PHY, sstar-emac */
        switch (reg) {
        case 0x01:          /* BMSR: 10/100 half/full, AN done, link up */
            v = 0x786D;
            break;
        case 0x02:          /* PHY ID, as the kernel's Albany ops fake it */
            v = 0x1111;
            break;
        case 0x03:
            v = 0x2222;
            break;
        case 0x05:          /* ANLPAR: partner 100/10 full and half */
            v = 0x45E1;
            break;
        }
        break;
    case 0x1121:
        /*
         * Ifado: the IPL's SMF stub (the secure-world shim it enters
         * before IPL_CUST) sets reg 0x37 bit 2, spins until reg 0x38
         * reads 3, then trades bit 2 for bit 1.  What the block is the
         * stub does not say; the answer is all it needs.
         */
        if (s->smf_ack && reg == 0x38 && (s->regs[(bank << 7) | 0x37] & 4)) {
            v = 3;
        }
        break;
    case 0x1402:
    case 0x1410:
        /*
         * infinity3 FCIE (SD host) soft reset, reg 0x3F: bit 0 is the reset
         * (active low) and [3:1] the MIU, MIE and MCU reset states, which
         * follow it inverted.  Hal_SDMMC_Reset waits for them, and so does
         * the IPL ("[HSD] IP Reset Switch", its table at IPL 0x7670 names
         * FCIE1 and FCIE2).
         */
        if (s->infinity3 && reg == 0x3F) {
            v = (v & ~0x000E) | ((v & 1) ? 0 : 0x000E);
        }
        break;
    case 0x1210:
    case 0x1211:
    case 0x1212:
        /* infinity3 HVSP0-2: reg 0x41 bit 8, filter SRAM load busy */
        if (s->infinity3 && reg == 0x41) {
            v &= ~0x0100;
        }
        break;
    case 0x101E:            /* chiptop: reg 0x48 is the package bond strap */
        if (reg == 0x48) {
            v = s->bond;
        }
        break;
    case 0x0022:
        /*
         * OTP2: regs 0x7C-0x7F hold the die's unique ID, low word first.
         * The SSC375 kernel's msys_read_uuid reads them here (0x1F0045F0),
         * not at the 0x101F bank of the Ifliegen SDK's Ifado header.
         * Vendor licences are signed for it, so it is the board's own,
         * given as uuid=; unset, the fuses read as stored, zero.
         */
        if (s->uuid && reg >= 0x7C) {
            v = s->uuid >> ((reg - 0x7C) * 16);
        }
        break;
    }
    /*
     * Ifado PADTOP GPIO, one register per pad (pad 58 at reg 0x41, the
     * kernel's gpio table): bit 0 the level read, bit 1 the level driven,
     * bit 2 output disabled, bit 3 the pad in its function mode (the I2C
     * pads' "mask0x8").  An input or a function pad reads high, as the
     * board's pull-ups hold the I2C lines the MIIC driver checks before
     * each transfer; a GPIO output reads what it drives.
     */
    if (s->gpio_bank && bank == s->gpio_bank) {
        v &= ~1;
        v |= (v & 0xC) ? 1 : (v >> 1) & 1;
    }
    /*
     * The SAR pads as GPIOs (gpio 81.., PAD_SAR_GPIO0..), in the PM SAR
     * bank: reg 0x11 [15:8] output disabled, reg 0x12 [7:0] the level
     * driven, [15:8] the level read.  An input reads high: the camera's
     * reset button on SAR_GPIO0 is pulled up, and reading it low is a
     * held button, a factory reset and a reboot.
     */
    if (s->gpio_bank && bank == 0x0014 && reg == 0x12) {
        uint16_t oen = s->regs[(bank << 7) | 0x11] >> 8;

        v = (v & 0xFF) | (uint16_t)((oen | (v & ~oen)) & 0xFF) << 8;
    }
    return v;
}

#define RIU_HOT_POLL        0x10000

static uint64_t sstar_riu_read(void *opaque, hwaddr off, unsigned size)
{
    SstarRiuState *s = opaque;
    uint32_t bank = riu_bank(off);
    uint32_t reg = riu_reg(off);
    uint32_t idx = off >> 2;
    uint16_t v;

    if ((off & 3) >= 2 || idx >= RIU_NREGS) {
        return 0;
    }
    if (!test_and_set_bit(bank, s->seen)) {
        qemu_log_mask(LOG_UNIMP, "sstar-riu: bank 0x%04x first read "
                      "(reg 0x%02x)\n", bank, reg);
    }
    v = sstar_riu_status(s, bank, reg, s->regs[idx]);
    if (idx == s->poll_idx) {
        if (++s->poll_count == RIU_HOT_POLL) {
            qemu_log_mask(LOG_UNIMP, "sstar-riu: hot poll on bank 0x%04x "
                          "reg 0x%02x (0x%08" HWADDR_PRIx "), reads 0x%04x\n",
                          bank, reg, SSTAR_RIU_BASE + off, v);
        }
    } else {
        s->poll_idx = idx;
        s->poll_count = 0;
    }
    if (s->trace_banks && test_bit(bank, s->trace_banks)) {
        qemu_log("sstar-riu: R %04x:%02x (0x%08" HWADDR_PRIx ") = 0x%04x\n",
                 bank, reg, SSTAR_RIU_BASE + off, v);
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_riu_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    SstarRiuState *s = opaque;
    uint32_t bank = riu_bank(off);
    uint32_t reg = riu_reg(off);
    uint32_t idx = off >> 2;
    uint16_t v;

    if ((off & 3) >= 2 || idx >= RIU_NREGS) {
        return;
    }
    if (!test_and_set_bit(bank, s->seen)) {
        qemu_log_mask(LOG_UNIMP, "sstar-riu: bank 0x%04x first write "
                      "(reg 0x%02x = 0x%04" PRIx64 ")\n", bank, reg, val);
    }
    s->poll_idx = UINT32_MAX;
    if (s->trace_banks && test_bit(bank, s->trace_banks)) {
        qemu_log("sstar-riu: W %04x:%02x (0x%08" HWADDR_PRIx ") = 0x%04"
                 PRIx64 "%s\n", bank, reg, SSTAR_RIU_BASE + off, val,
                 size == 1 ? (off & 1 ? " (high byte)" : " (low byte)") : "");
    }
    v = s->regs[idx];
    if (size == 1) {
        if (off & 1) {
            v = (v & 0x00FF) | ((val & 0xFF) << 8);
        } else {
            v = (v & 0xFF00) | (val & 0xFF);
        }
    } else {
        v = val;
    }
    s->regs[idx] = v;

    /*
     * reset_cpu (U-Boot reset.S) and the kernel's restart hook write 0xFF
     * then 0x79 to PM_SLEEP 0x2E: the second write resets the chip.
     */
    if (bank == BANK_PMSLEEP && reg == PMSLEEP_SWRST && v == 0x79) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static const MemoryRegionOps sstar_riu_ops = {
    .read = sstar_riu_read,
    .write = sstar_riu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void sstar_riu_reset(DeviceState *dev)
{
    SstarRiuState *s = SSTAR_RIU(dev);
    uint32_t i;

    memset(s->regs, 0, RIU_NREGS * sizeof(uint16_t));
    for (i = 0; i < s->num_defaults; i++) {
        const SstarRegDefault *d;

        for (d = s->defaults[i]; d->addr; d++) {
            assert(d->addr - SSTAR_RIU_BASE < SSTAR_RIU_SIZE);
            s->regs[(d->addr - SSTAR_RIU_BASE) >> 2] = d->val;
        }
    }
}

void sstar_riu_add_defaults(SstarRiuState *s, const SstarRegDefault *table)
{
    assert(s->num_defaults < ARRAY_SIZE(s->defaults));
    s->defaults[s->num_defaults++] = table;
}

static void sstar_riu_realize(DeviceState *dev, Error **errp)
{
    SstarRiuState *s = SSTAR_RIU(dev);

    s->poll_idx = UINT32_MAX;
    if (s->trace && *s->trace) {
        g_auto(GStrv) banks = g_strsplit(s->trace, ",", -1);
        int i;

        s->trace_banks = bitmap_new(RIU_NBANKS);
        for (i = 0; banks[i]; i++) {
            uint64_t b;

            if (qemu_strtou64(banks[i], NULL, 0, &b) < 0 || b >= RIU_NBANKS) {
                error_setg(errp, "sstar-riu: bad bank '%s' in trace",
                           banks[i]);
                g_free(s->trace_banks);
                s->trace_banks = NULL;
                return;
            }
            set_bit(b, s->trace_banks);
        }
    }
    s->regs = g_new0(uint16_t, RIU_NREGS);
    s->seen = bitmap_new(RIU_NBANKS);
    memory_region_init_io(&s->iomem, OBJECT(s), &sstar_riu_ops, s,
                          "sstar-riu", SSTAR_RIU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static const Property sstar_riu_props[] = {
    DEFINE_PROP_UINT16("bond", SstarRiuState, bond, 0),
    DEFINE_PROP_BOOL("smf-ack", SstarRiuState, smf_ack, false),
    DEFINE_PROP_BOOL("infinity3", SstarRiuState, infinity3, false),
    DEFINE_PROP_UINT16("gpio-bank", SstarRiuState, gpio_bank, 0),
    DEFINE_PROP_UINT64("uuid", SstarRiuState, uuid, 0),
    DEFINE_PROP_STRING("trace", SstarRiuState, trace),
};

static const VMStateDescription vmstate_sstar_riu = {
    .name = TYPE_SSTAR_RIU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BUFFER_POINTER_UNSAFE(regs, SstarRiuState, 1,
                                      RIU_NREGS * sizeof(uint16_t)),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_riu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_riu_realize;
    device_class_set_legacy_reset(dc, sstar_riu_reset);
    device_class_set_props(dc, sstar_riu_props);
    dc->vmsd = &vmstate_sstar_riu;
}

static const TypeInfo sstar_riu_info = {
    .name           = TYPE_SSTAR_RIU,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarRiuState),
    .class_init     = sstar_riu_class_init,
};

static void sstar_riu_register_types(void)
{
    type_register_static(&sstar_riu_info);
}

type_init(sstar_riu_register_types)
