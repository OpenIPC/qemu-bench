/*
 * Ingenic CPM: clock, power and reset management at 0x10000000.
 *
 * Software only programs this block and polls it; nothing here clocks a
 * model.  What has to behave is what the SPL, U-Boot and the kernels wait
 * for, none of them with a timeout that would rescue a wrong answer
 * (U-Boot arch/mips/cpu/xburst/<soc>/{pll,clk}.c, the kernels'
 * soc-<soc>/common/clk/ and drivers/clk/ingenic/clk-t4x.c):
 *
 *   CPAPCR 0x10, CPMPCR 0x14, CPVPCR 0xE0 (and CPEPCR 0x58 on T30/T40):
 *     the PLLs.  Bit 0 enables; bit 3 (ON) -- and bit 2 (LOCK) where the
 *     layout has one -- follows it at once.
 *   CPCCR 0x00: the CPU/AHB/APB dividers.  Bits 22:20 ask for a change
 *     and read back clear; CPCSR 0xD4 bits 2:0 (busy) always read 0.
 *   the peripheral dividers (xxxCDR): a write with CE set completes
 *     at once, so BUSY reads 0.  BUSY is bit 28 in most of them, bit 27
 *     in the T21..T31 SSI/SFC and LPC dividers (stop 26, CE 28).
 *
 *   SRBC 0xC4: the soft resets.  Before resetting the ISP the tx-isp
 *     drivers set its stop request (bit 21) and wait up to a second for
 *     the stop acknowledge (bit 20; private_reset_tx_isp_module), which
 *     reads set while the request is.  The video engines' stop bits
 *     (30 HELIX/VPU, 27 RADIX) are acknowledged the same way in the bit
 *     below them, which the soc_vpu drivers poll before every task's
 *     reset.  Every SRBC bit is also a GPIO output ("srbc"), so that a
 *     block's soft-reset bit (HELIX/VPU 31, RADIX 28) resets its model.
 *     T30 has three more ISP blocks behind the same handshake in 0xAC:
 *     request 7 / ack 6 (LDC), 1 / 0 (NCU), 4 / 3 (the scaler) --
 *     tx-isp-t30.ko's reset table, rodata 0x770.
 *
 * Everything else is storage: gates, OPCR, the USB PHY controls, MPHYC
 * and the CPPSR/CPSPPR pair that the reboot code writes and the MXU
 * licence check reads back.
 *
 * The reset values are the state the vendor U-Boot leaves, in each SoC's
 * own PLL encoding, so that a -kernel boot (no SPL) computes the clock
 * rates it would on a camera.  The kernels oops before the console if a
 * divider's mux reads 3, or DDRCDR's reads 0 (stop), so those are set too.
 * Three PLL encodings exist:
 *
 *   T10/T20/T23/T31/T40  M[31:20] N[19:14] OD1[13:11] OD0[10:8]
 *                        F = 24 MHz * M / N / OD1 / OD0
 *   T21/T30              M[28:20] N[19:14] OD[13:11] RNG[7:5]
 *                        F = 24 MHz * 2(M+1) / (N+1) / 2^OD
 *   T41                  M[28:20] N[19:14] OD0[13:11] OD1[10:7] RG[6:4]
 *                        F = 24 MHz * 2(M+1) / (N+1) / 2^OD0 / (OD1+1)
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/mips/ingenic.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_CPM "ingenic-cpm"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicCpmState, INGENIC_CPM)

#define CPM_SIZE        0x100

#define R_CPCCR         0x00
#define R_RSR           0x08
#define R_CPAPCR        0x10
#define R_CPMPCR        0x14
#define R_CPAPACR       0x18
#define R_CPMPACR       0x1C
#define R_CLKGR0        0x20
#define R_OPCR          0x24
#define R_DDRCDR        0x2C
#define R_CPPSR         0x34
#define R_CPSPPR        0x38
#define R_USBPCR        0x3C
#define R_USBPCR1       0x48
#define R_CPEPCR        0x58
#define R_SRBC          0xC4
#define SRBC_ISP_ACK    (1u << 20)
#define SRBC_ISP_STP    (1u << 21)
#define SRBC_HELIX_ACK  (1u << 29)
#define SRBC_RADIX_ACK  (1u << 26)
#define R_SRBC1_T30     0xAC
#define SRBC1_T30_ACKS  ((1u << 6) | (1u << 3) | (1u << 0))
#define R_DRCG          0xD0
#define R_CPCSR         0xD4
#define R_CPVPCR        0xE0
#define R_CPVPACR       0xE4
#define R_MPHYC         0xE8

#define PLL_EN          (1u << 0)
#define PLL_LOCK        (1u << 2)
#define PLL_ON          (1u << 3)
#define CPCCR_CE        (7u << 20)
#define CDR_CE          (1u << 29)
#define CDR_BUSY        (1u << 28)
#define CDR_BUSY27      (1u << 27)
#define OPCR_ERCS       (1u << 2)
#define RSR_POR         (1u << 0)

struct IngenicCpmState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t soc;
    uint32_t cppsr;
    uint32_t regs[CPM_SIZE / 4];
    qemu_irq srbc_out[32];      /* "srbc": each SRBC bit's level */
};

typedef struct CpmReset {
    uint32_t cpccr, apll, mpll, vpll, epll, ddrcdr;
} CpmReset;

/* PLL encodings of the U-Boot defaults (isvp_<soc>.h), enable + ON set */
#define PLL_A(m, n, od1, od0) \
    (((m) << 20) | ((n) << 14) | ((od1) << 11) | ((od0) << 8) | PLL_ON | \
     PLL_LOCK | PLL_EN)
#define PLL_B(m, n, od, rng) \
    (((m) << 20) | ((n) << 14) | ((od) << 11) | ((rng) << 5) | PLL_ON | PLL_EN)
#define PLL_C(m, n, od0, od1, rg) \
    (((m) << 20) | ((n) << 14) | ((od0) << 11) | ((od1) << 7) | ((rg) << 4) | \
     PLL_ON | PLL_LOCK | PLL_EN)

static const CpmReset cpm_reset_values[] = {
    /* APLL 852, MPLL 1200 (DDR 400), PCLK/12 H2/6 H0/6 L2/2 */
    [INGENIC_T10] = { 0x9A0B5510, PLL_A(71, 2, 1, 1), PLL_A(100, 1, 2, 1),
                      0, 0, 0x80000002 },
    /* APLL 852, MPLL 1000 (DDR 500), VPLL 1200, /10 /5 /5 /2 */
    [INGENIC_T20] = { 0x9A094410, PLL_A(71, 2, 1, 1), PLL_A(125, 3, 1, 1),
                      PLL_A(100, 1, 2, 1), 0, 0x80000001 },
    /* APLL 864, MPLL 900 (DDR 450), VPLL 1200, /8 /4 /4 /2 */
    [INGENIC_T21] = { 0x9A073310, PLL_B(71, 1, 1, 2), PLL_B(74, 1, 1, 2),
                      PLL_B(49, 0, 1, 2), 0, 0x80000001 },
    /* APLL 1188, MPLL 1200 (DDR 600), /12 /6 /6 /2; no VPLL */
    [INGENIC_T23] = { 0x9A0B5510, PLL_A(297, 3, 2, 1), PLL_A(100, 1, 2, 1),
                      0, 0, 0x80000001 },
    /* APLL 900, MPLL 1000 (DDR 500), VPLL 1200, EPLL 1188 */
    [INGENIC_T30] = { 0x9A073310, PLL_B(74, 1, 1, 2), PLL_B(124, 2, 1, 1),
                      PLL_B(49, 0, 1, 2), PLL_B(197, 3, 1, 1), 0x80000001 },
    /* APLL 1404, MPLL 1000 (DDR 500), VPLL 1200, /8 /4 /4 /2 */
    [INGENIC_T31] = { 0x9A073310, PLL_A(117, 1, 2, 1), PLL_A(125, 1, 3, 1),
                      PLL_A(100, 1, 2, 1), 0, 0x80000001 },
    /* APLL 1200, MPLL 1200, VPLL 1200, EPLL 1200, /12 /6 /6 /2 */
    [INGENIC_T40] = { 0x9A0B5510, PLL_A(100, 1, 2, 1), PLL_A(100, 1, 2, 1),
                      PLL_A(100, 1, 2, 1), PLL_A(100, 1, 2, 1), 0x40000002 },
    /* APLL 1008, MPLL 1200, VPLL 1200 */
    [INGENIC_T41] = { 0x9A0B5510, PLL_C(83, 0, 1, 1, 3), PLL_C(99, 0, 1, 1, 3),
                      PLL_C(99, 0, 1, 1, 3), 0, 0x40000002 },
};

static bool cpm_is_pll(IngenicCpmState *s, hwaddr off)
{
    switch (off) {
    case R_CPAPCR:
    case R_CPMPCR:
    case R_CPVPCR:
        return true;
    case R_CPEPCR:
        return s->soc == INGENIC_T30 || s->soc == INGENIC_T40;
    default:
        return false;
    }
}

/* The bits a PLL reports once it is up: T21/T30 have no LOCK bit. */
static uint32_t cpm_pll_status(IngenicCpmState *s)
{
    return (s->soc == INGENIC_T21 || s->soc == INGENIC_T30) ?
           PLL_ON : PLL_ON | PLL_LOCK;
}

/*
 * The busy bit of the divider at off, or 0 where off is not a divider of
 * the CE/BUSY/STOP kind (I2S's M/N dividers keep M in bits 28:20).
 */
static uint32_t cpm_cdr_busy(IngenicCpmState *s, hwaddr off)
{
    if (off < R_DDRCDR || cpm_is_pll(s, off)) {
        return 0;
    }
    switch (off) {
    case R_CPPSR:
    case R_CPSPPR:
    case R_USBPCR ... R_USBPCR1:
    case 0x70: case 0x78: case 0x84: case 0x88:     /* I2S M/N */
    case 0xB0: case 0xB4:                           /* INTR/INTRE */
    case R_SRBC:
    case R_DRCG:
    case R_CPCSR:
    case R_CPVPACR:
    case R_MPHYC:
        return 0;
    }
    if (s->soc >= INGENIC_T21 && s->soc <= INGENIC_T31 &&
        (off == 0x64 || off == 0x74 || (s->soc == INGENIC_T23 && off == 0x60))) {
        return CDR_BUSY27;
    }
    return CDR_BUSY;
}

/* A PLL's output in Hz, in this SoC's encoding (above); 0 while it is off */
static uint64_t cpm_pll_hz(IngenicCpmState *s, hwaddr off)
{
    uint32_t v = s->regs[off >> 2];
    uint64_t m, n, od0, od1;

    if (!(v & PLL_ON)) {
        return 0;
    }
    switch (s->soc) {
    case INGENIC_T21:
    case INGENIC_T30:
        m = extract32(v, 20, 9);
        n = extract32(v, 14, 6);
        return 24000000ULL * 2 * (m + 1) / (n + 1) >> extract32(v, 11, 3);
    case INGENIC_T41:
        m = extract32(v, 20, 9);
        n = extract32(v, 14, 6);
        od0 = extract32(v, 11, 3);
        od1 = extract32(v, 7, 4);
        return 24000000ULL * 2 * (m + 1) / (n + 1) / (1u << od0) / (od1 + 1);
    default:
        m = extract32(v, 20, 12);
        n = extract32(v, 14, 6);
        od1 = extract32(v, 11, 3);
        od0 = extract32(v, 8, 3);
        return n && od1 && od0 ? 24000000ULL * m / n / od1 / od0 : 0;
    }
}

/*
 * The I2S speaker (tx) or microphone clock of T23/T31/T40/T41, the
 * AIC's master clock (256 fs: the audio drivers set it to rate * 256):
 * I2SSPKCDR / I2STCDR 0x70 and I2SMICCDR / I2SRCDR 0x84, CE bit 29,
 * F = parent * M[28:20] / N[19:0], parent bits 31:30 SCLKA, MPLL, VPLL,
 * EPLL (T40).  T23's and T31's microphone mux is read only; their
 * kernels take the speaker's (clk_cgu.c cgu_get_parent).  0 when off,
 * or on a part whose AIC is clocked by its codec.
 */
uint64_t ingenic_cpm_i2s_hz(DeviceState *dev, bool tx)
{
    IngenicCpmState *s = INGENIC_CPM(dev);
    uint32_t v = s->regs[(tx ? 0x70 : 0x84) >> 2];
    uint32_t sel = s->regs[0x70 >> 2] >> 30, m, n;
    uint64_t parent;

    switch (s->soc) {
    case INGENIC_T40:
    case INGENIC_T41:
        sel = v >> 30;
        break;
    case INGENIC_T23:
    case INGENIC_T31:
        break;
    default:
        return 0;
    }
    m = extract32(v, 20, 9);
    n = extract32(v, 0, 20);
    if (!(v & CDR_CE) || !m || !n) {
        return 0;
    }
    switch (sel) {
    case 0:                                         /* SCLKA */
        switch (s->regs[R_CPCCR >> 2] >> 30) {
        case 1:
            parent = 24000000;
            break;
        case 2:
            parent = cpm_pll_hz(s, R_CPAPCR);
            break;
        default:
            parent = 0;
            break;
        }
        break;
    case 1:
        parent = cpm_pll_hz(s, R_CPMPCR);
        break;
    case 2:
        parent = cpm_pll_hz(s, R_CPVPCR);
        break;
    default:
        parent = s->soc == INGENIC_T40 ? cpm_pll_hz(s, R_CPEPCR) : 0;
        break;
    }
    return parent * m / n;
}

static uint64_t ingenic_cpm_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicCpmState *s = opaque;
    uint32_t v = s->regs[(off & (CPM_SIZE - 1)) >> 2];

    if ((off & ~3) == R_CPCSR) {
        v &= ~7u;
    } else if ((off & ~3) == R_SRBC) {
        /* stop requests are acknowledged at once, in the bit below */
        v |= (v >> 1) & (SRBC_ISP_ACK | SRBC_HELIX_ACK | SRBC_RADIX_ACK);
    } else if ((off & ~3) == R_SRBC1_T30 && s->soc == INGENIC_T30) {
        v |= (v >> 1) & SRBC1_T30_ACKS;
    }
    return extract32(v, (off & 3) * 8, size * 8);
}

static void ingenic_cpm_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicCpmState *s = opaque;
    hwaddr reg = off & (CPM_SIZE - 4);
    uint32_t v = deposit32(s->regs[reg >> 2], (off & 3) * 8, size * 8, val);
    uint32_t busy;

    if (cpm_is_pll(s, reg)) {
        v &= ~(PLL_ON | PLL_LOCK);
        if (v & PLL_EN) {
            v |= cpm_pll_status(s);
        }
    } else if (reg == R_CPCCR) {
        v &= ~CPCCR_CE;
    } else if ((busy = cpm_cdr_busy(s, reg))) {
        v &= ~busy;
    }
    if (reg == R_SRBC) {
        uint32_t changed = v ^ s->regs[reg >> 2];
        int i;

        s->regs[reg >> 2] = v;
        for (i = 0; i < 32; i++) {
            if (changed & (1u << i)) {
                qemu_set_irq(s->srbc_out[i], extract32(v, i, 1));
            }
        }
        return;
    }
    s->regs[reg >> 2] = v;
}

static const MemoryRegionOps ingenic_cpm_ops = {
    .read = ingenic_cpm_read,
    .write = ingenic_cpm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void ingenic_cpm_reset(DeviceState *dev)
{
    IngenicCpmState *s = INGENIC_CPM(dev);
    const CpmReset *r = &cpm_reset_values[s->soc];

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_CPCCR >> 2] = r->cpccr;
    s->regs[R_CPAPCR >> 2] = r->apll;
    s->regs[R_CPMPCR >> 2] = r->mpll;
    s->regs[R_CPVPCR >> 2] = r->vpll;
    if (cpm_is_pll(s, R_CPEPCR)) {
        s->regs[R_CPEPCR >> 2] = r->epll;
    }
    s->regs[R_DDRCDR >> 2] = r->ddrcdr;
    s->regs[R_OPCR >> 2] = OPCR_ERCS;
    s->regs[R_RSR >> 2] = RSR_POR;
    s->regs[R_CPPSR >> 2] = s->cppsr;
}

static void ingenic_cpm_realize(DeviceState *dev, Error **errp)
{
    IngenicCpmState *s = INGENIC_CPM(dev);

    if (s->soc >= ARRAY_SIZE(cpm_reset_values)) {
        error_setg(errp, "ingenic-cpm: unknown soc %u", s->soc);
    }
}

static void ingenic_cpm_init(Object *obj)
{
    IngenicCpmState *s = INGENIC_CPM(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_cpm_ops, s,
                          TYPE_INGENIC_CPM, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), s->srbc_out, "srbc", 32);
}

static const Property ingenic_cpm_props[] = {
    DEFINE_PROP_UINT32("soc", IngenicCpmState, soc, INGENIC_T31),
    /* the low byte grades T10/T20/T21/T30/T31: 0x10 an L part, else 1 */
    DEFINE_PROP_UINT32("cppsr", IngenicCpmState, cppsr, 0x01),
};

static const VMStateDescription vmstate_ingenic_cpm = {
    .name = TYPE_INGENIC_CPM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicCpmState, CPM_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_cpm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ingenic_cpm_realize;
    device_class_set_legacy_reset(dc, ingenic_cpm_reset);
    device_class_set_props(dc, ingenic_cpm_props);
    dc->vmsd = &vmstate_ingenic_cpm;
}

static const TypeInfo ingenic_cpm_info = {
    .name = TYPE_INGENIC_CPM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicCpmState),
    .instance_init = ingenic_cpm_init,
    .class_init = ingenic_cpm_class_init,
};

static void ingenic_cpm_register_types(void)
{
    type_register_static(&ingenic_cpm_info);
}

type_init(ingenic_cpm_register_types)
