/*
 * Novatek NA51089 RSA engine at 0xF06A0000 (SPI 20): modular
 * exponentiation of 256 to 2048-bit numbers.
 *
 * drivers/crypto/nvt_ivot/na51089_rsa.c loads the operands into the
 * engine's SRAM through three ports, word by word, starting each at the
 * index its ADDR register is set to, and reads the result back through
 * the data port:
 *
 *   0x00 CFG       bit 0 software reset (clears itself); [3:1] key width
 *                  256 << n bits; [5:4] 0 normal, 1 / 2 CRC32 of N / of
 *                  E or D (key checks; not modelled)
 *   0x04 CTRL      bit 0 starts; it clears when the result is in
 *   0x08 INT_ENB   0x0C INT_STS: bit 0 done (write 1 to clear), bit 1
 *                  busy; SPI 20 = STS & ENB
 *   0x10 KEY_N     0x14 KEY_N_ADDR     the modulus
 *   0x18 KEY_ED    0x1C KEY_ED_ADDR    the exponent
 *   0x20 DATA      0x24 DATA_ADDR      the base, then the result
 *
 * Numbers are little-endian in the SRAM: the driver reverses the
 * kernel's big-endian strings before writing them.  The result is
 * DATA ^ ED mod N, as wide as the key.  The SRAM holds 2048 bits per
 * operand on this die (the driver's buffer for NA51089); the 4096-bit
 * width belongs to NA51084.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_RSA "nvt-rsa"
OBJECT_DECLARE_SIMPLE_TYPE(NvtRsaState, NVT_RSA)

#define RSA_REGS_SIZE   0x100
#define RSA_CFG         0x00
#define RSA_CTRL        0x04
#define RSA_INT_ENB     0x08
#define RSA_INT_STS     0x0C
#define RSA_KEY_N       0x10
#define RSA_KEY_N_ADDR  0x14
#define RSA_KEY_ED      0x18
#define RSA_KEY_ED_ADDR 0x1C
#define RSA_DATA        0x20
#define RSA_DATA_ADDR   0x24

#define RSA_WORDS       64              /* 2048 bits */

enum { PORT_N, PORT_ED, PORT_DATA, PORT_NUM };

struct NvtRsaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[RSA_REGS_SIZE / 4];
    uint32_t sram[PORT_NUM][RSA_WORDS];
};

/* Montgomery arithmetic on n-word little-endian numbers */

/* a >= b ? */
static bool bn_ge(const uint32_t *a, const uint32_t *b, int n)
{
    int i;

    for (i = n - 1; i >= 0; i--) {
        if (a[i] != b[i]) {
            return a[i] > b[i];
        }
    }
    return true;
}

/* a -= b, returning the borrow */
static uint32_t bn_sub(uint32_t *a, const uint32_t *b, int n)
{
    uint64_t borrow = 0;
    int i;

    for (i = 0; i < n; i++) {
        uint64_t d = (uint64_t)a[i] - b[i] - borrow;

        a[i] = d;
        borrow = (d >> 32) & 1;
    }
    return borrow;
}

/* r = a * b * 2^(-32n) mod m (CIOS); r < m when a * b < m * 2^(32n) */
static void bn_mont_mul(uint32_t *r, const uint32_t *a, const uint32_t *b,
                        const uint32_t *m, uint32_t m0inv, int n)
{
    uint32_t t[RSA_WORDS + 2] = { 0 };
    uint64_t c;
    uint32_t q;
    int i, j;

    for (i = 0; i < n; i++) {
        c = 0;
        for (j = 0; j < n; j++) {
            c += t[j] + (uint64_t)a[j] * b[i];
            t[j] = c;
            c >>= 32;
        }
        c += t[n];
        t[n] = c;
        t[n + 1] = c >> 32;

        q = t[0] * m0inv;
        c = (t[0] + (uint64_t)q * m[0]) >> 32;
        for (j = 1; j < n; j++) {
            c += t[j] + (uint64_t)q * m[j];
            t[j - 1] = c;
            c >>= 32;
        }
        c += t[n];
        t[n - 1] = c;
        t[n] = t[n + 1] + (c >> 32);
    }
    if (t[n] || bn_ge(t, m, n)) {
        bn_sub(t, m, n);
    }
    memcpy(r, t, n * sizeof(uint32_t));
}

/* r = b ^ e mod m, m odd */
static void bn_mod_exp(uint32_t *r, const uint32_t *b, const uint32_t *e,
                       const uint32_t *m, int n)
{
    uint32_t r2[RSA_WORDS] = { 0 }, x[RSA_WORDS], acc[RSA_WORDS];
    uint32_t one[RSA_WORDS] = { 1 }, m0inv = 1, top;
    int i, bits;

    /* -m^-1 mod 2^32, by Newton's iteration */
    for (i = 0; i < 5; i++) {
        m0inv *= 2 - m[0] * m0inv;
    }
    m0inv = -m0inv;

    /* R^2 mod m, R = 2^(32n): double 1 up 64n times */
    r2[0] = 1;
    for (i = 0; i < 64 * n; i++) {
        top = r2[n - 1] >> 31;
        for (bits = n - 1; bits > 0; bits--) {
            r2[bits] = (r2[bits] << 1) | (r2[bits - 1] >> 31);
        }
        r2[0] <<= 1;
        if (top || bn_ge(r2, m, n)) {
            bn_sub(r2, m, n);
        }
    }

    bn_mont_mul(x, b, r2, m, m0inv, n);         /* b R mod m */
    bn_mont_mul(acc, one, r2, m, m0inv, n);     /* R mod m */
    for (i = 32 * n - 1; i >= 0; i--) {
        bn_mont_mul(acc, acc, acc, m, m0inv, n);
        if ((e[i / 32] >> (i % 32)) & 1) {
            bn_mont_mul(acc, acc, x, m, m0inv, n);
        }
    }
    bn_mont_mul(r, acc, one, m, m0inv, n);
}

/* the engine */

static uint32_t *reg(NvtRsaState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_rsa_update_irq(NvtRsaState *s)
{
    qemu_set_irq(s->irq, !!(*reg(s, RSA_INT_STS) & *reg(s, RSA_INT_ENB)));
}

static void nvt_rsa_run(NvtRsaState *s)
{
    uint32_t cfg = *reg(s, RSA_CFG);
    int width = (cfg >> 1) & 7, mode = (cfg >> 4) & 3, n;
    uint32_t r[RSA_WORDS];

    if (mode) {
        qemu_log_mask(LOG_UNIMP, "nvt-rsa: CRC32 key check mode %d\n", mode);
        return;
    }
    if (width > 3) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-rsa: key width %d bits\n",
                      256 << width);
        return;
    }
    n = 8 << width;
    if (!(s->sram[PORT_N][0] & 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-rsa: even modulus\n");
        memset(s->sram[PORT_DATA], 0, n * 4);
        return;
    }
    bn_mod_exp(r, s->sram[PORT_DATA], s->sram[PORT_ED], s->sram[PORT_N], n);
    memcpy(s->sram[PORT_DATA], r, n * 4);
}

/* The port at 'off' and its index register. */
static int nvt_rsa_port(hwaddr off)
{
    switch (off) {
    case RSA_KEY_N:
        return PORT_N;
    case RSA_KEY_ED:
        return PORT_ED;
    case RSA_DATA:
        return PORT_DATA;
    }
    return -1;
}

static uint32_t *nvt_rsa_port_word(NvtRsaState *s, hwaddr off)
{
    int p = nvt_rsa_port(off);
    uint32_t *idx = reg(s, off + 4);
    uint32_t *w = &s->sram[p][*idx % RSA_WORDS];

    *idx = (*idx + 1) % RSA_WORDS;
    return w;
}

static uint64_t nvt_rsa_read(void *opaque, hwaddr off, unsigned size)
{
    NvtRsaState *s = opaque;

    if (nvt_rsa_port(off) >= 0) {
        return *nvt_rsa_port_word(s, off);
    }
    return *reg(s, off);
}

static void nvt_rsa_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    NvtRsaState *s = opaque;

    if (nvt_rsa_port(off) >= 0) {
        *nvt_rsa_port_word(s, off) = val;
        return;
    }
    switch (off) {
    case RSA_CFG:
        if (val & 1) {
            /* the reset bit clears itself at once */
            memset(s->regs, 0, sizeof(s->regs));
        } else {
            *reg(s, off) = val;
        }
        break;
    case RSA_CTRL:
        if (val & 1) {
            nvt_rsa_run(s);
            *reg(s, RSA_INT_STS) |= 1;
        }
        break;
    case RSA_INT_STS:
        *reg(s, off) &= ~(val & 1);
        break;
    default:
        *reg(s, off) = val;
        break;
    }
    nvt_rsa_update_irq(s);
}

static const MemoryRegionOps nvt_rsa_ops = {
    .read = nvt_rsa_read,
    .write = nvt_rsa_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_rsa_reset(DeviceState *dev)
{
    NvtRsaState *s = NVT_RSA(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->sram, 0, sizeof(s->sram));
    qemu_irq_lower(s->irq);
}

static void nvt_rsa_init(Object *obj)
{
    NvtRsaState *s = NVT_RSA(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_rsa_ops, s, TYPE_NVT_RSA,
                          RSA_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_nvt_rsa = {
    .name = TYPE_NVT_RSA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtRsaState, RSA_REGS_SIZE / 4),
        VMSTATE_UINT32_2DARRAY(sram, NvtRsaState, PORT_NUM, RSA_WORDS),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_rsa_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_rsa_reset);
    dc->vmsd = &vmstate_nvt_rsa;
}

static const TypeInfo nvt_rsa_info = {
    .name = TYPE_NVT_RSA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtRsaState),
    .instance_init = nvt_rsa_init,
    .class_init = nvt_rsa_class_init,
};

static void nvt_rsa_register_types(void)
{
    type_register_static(&nvt_rsa_info);
}

type_init(nvt_rsa_register_types)
