/*
 * SigmaStar / MStar SHA and RSA engines (RIU bank 0x1122, 0x1F224400).
 *
 * One bank holds three engines (the vendor boot/drivers/mstar/aesdma/hal/
 * halAESDMA.c: SHARNG_BASE_ADDR, RSA_BASE_ADDR and AESDMA_BASE_ADDR are
 * all 0x112200):
 *
 *   SHA   0x08 control: bit 0 fire, bit 6 clear, bit 7 reset, bit 9
 *         SHA-256 (else SHA-1), bit 11 no scatter-gather.  0x0A/0x0B the
 *         message's MIU address, 0x0C/0x0D its length in bytes, 0x0F
 *         status (bit 0 ready).  0x10-0x1F the digest, byte-reversed:
 *         the driver reads the 16 words out and reverses all 32 bytes.
 *   RSA   a RAM of 32-bit words behind an indirect port: 0x22 the word
 *         address (exponent E at 0x00, modulus N at 0x40, base A at 0x80,
 *         result Z at 0xC0), 0x23/0x24 a word's low and high halves to
 *         write, 0x25/0x26 to read, 0x21 its control (bit 1 write, bit 2
 *         address auto-increment).  Numbers are little-endian in words:
 *         word 0 is the least significant.  0x28 control (bit 0 reset,
 *         bit 1 hardware key, bit 2 public key, [13:8] the key length in
 *         words less one), 0x27 bit 0 starts Z = A^E mod N, 0x29 status
 *         (bit 0 busy, bit 1 done).
 *   AES   0x50-0x7F, the AES DMA; storage only so far.
 *
 * The Ifado (SSC375) boot chain checks its images with SHA-256 and a
 * 2048-bit public key through these (Dahua's DHBoot, MDrv_SHA_Run and
 * MDrv_RSA_Run); both complete at once.  The hardware key, in OTP, is
 * not modelled.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "crypto/hash.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_SSTAR_CRYPTO "sstar-crypto"
OBJECT_DECLARE_SIMPLE_TYPE(SstarCryptoState, SSTAR_CRYPTO)

#define SHA_CTRL            0x08
#define SHA_ADDR_L          0x0A
#define SHA_ADDR_H          0x0B
#define SHA_LEN_L           0x0C
#define SHA_LEN_H           0x0D
#define SHA_STATUS          0x0F
#define SHA_OUT             0x10        /* 16 words */

#define SHA_CTRL_FIRE       (1u << 0)
#define SHA_CTRL_RST        (1u << 7)
#define SHA_CTRL_SHA256     (1u << 9)
#define SHA_LEN_MAX         (256 * MiB)    /* the largest DRAM */
#define SHA_STATUS_READY    (1u << 0)

#define RSA_IND32_START     0x20
#define RSA_IND32_CTRL      0x21
#define RSA_IND32_ADDR      0x22
#define RSA_IND32_WDATA_L   0x23
#define RSA_IND32_WDATA_H   0x24
#define RSA_IND32_RDATA_L   0x25
#define RSA_IND32_RDATA_H   0x26
#define RSA_EXP             0x27
#define RSA_CTRL            0x28
#define RSA_STATUS          0x29

#define RSA_IND32_WRITE     (1u << 1)
#define RSA_IND32_AUTO_INC  (1u << 2)
#define RSA_EXP_START       (1u << 0)
#define RSA_CTRL_RST        (1u << 0)
#define RSA_CTRL_HW_KEY     (1u << 1)
#define RSA_STATUS_DONE     (1u << 1)

#define RSA_RAM_WORDS       0x100
#define RSA_E               0x00
#define RSA_N               0x40
#define RSA_A               0x80
#define RSA_Z               0xC0
#define RSA_WORDS           64          /* 2048 bits */

#define SS_DRAM_BASE        0x20000000

struct SstarCryptoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint16_t regs[0x80];
    uint32_t ram[RSA_RAM_WORDS];
    /* the halves of the current word moved so far: bit 0 low, bit 1 high */
    uint8_t wr_halves;
    uint8_t rd_halves;
};

/* Montgomery arithmetic on n-word little-endian numbers (as nvt-rsa.c) */

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

/* ---- SHA ------------------------------------------------------------ */

static void sstar_sha_run(SstarCryptoState *s)
{
    uint32_t addr = s->regs[SHA_ADDR_L] | (s->regs[SHA_ADDR_H] << 16);
    uint32_t len = s->regs[SHA_LEN_L] | (s->regs[SHA_LEN_H] << 16);
    bool sha256 = s->regs[SHA_CTRL] & SHA_CTRL_SHA256;
    g_autofree uint8_t *buf = NULL;
    g_autofree uint8_t *digest = NULL;
    size_t dlen = 0;
    uint8_t out[32] = { 0 };
    int i;

    /* An MIU address (the driver masks the CPU address's top nibble) */
    if (addr < SS_DRAM_BASE) {
        addr += SS_DRAM_BASE;
    }
    /*
     * A source beyond DRAM, or one that does not read, gives a zero
     * digest (which no signature matches) rather than one of whatever
     * the buffer held.
     */
    if (len > SHA_LEN_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-crypto: SHA length %u\n", len);
        goto done;
    }
    buf = g_malloc(len ? len : 1);
    if (dma_memory_read(&address_space_memory, addr, buf, len,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-crypto: SHA source 0x%08x+%u "
                      "unreadable\n", addr, len);
        goto done;
    }
    if (qcrypto_hash_bytes(sha256 ? QCRYPTO_HASH_ALGO_SHA256
                                  : QCRYPTO_HASH_ALGO_SHA1,
                           buf, len, &digest, &dlen, NULL) < 0 ||
        dlen > sizeof(out)) {
        qemu_log_mask(LOG_UNIMP, "sstar-crypto: SHA%s unavailable\n",
                      sha256 ? "-256" : "-1");
    } else {
        /* reversed, as the driver reverses the 32 bytes it reads */
        for (i = 0; i < dlen; i++) {
            out[sizeof(out) - 1 - i] = digest[i];
        }
    }
done:
    for (i = 0; i < 16; i++) {
        s->regs[SHA_OUT + i] = out[2 * i] | (out[2 * i + 1] << 8);
    }
    s->regs[SHA_STATUS] |= SHA_STATUS_READY;
}

/* ---- RSA ------------------------------------------------------------ */

static void sstar_rsa_run(SstarCryptoState *s)
{
    int n = ((s->regs[RSA_CTRL] >> 8) & 0x3F) + 1;

    if (s->regs[RSA_CTRL] & RSA_CTRL_HW_KEY) {
        qemu_log_mask(LOG_UNIMP, "sstar-crypto: RSA with the OTP key\n");
    } else if (!(s->ram[RSA_N] & 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-crypto: RSA even modulus\n");
    } else {
        bn_mod_exp(&s->ram[RSA_Z], &s->ram[RSA_A], &s->ram[RSA_E],
                   &s->ram[RSA_N], n);
    }
    s->regs[RSA_STATUS] = RSA_STATUS_DONE;
}

static uint32_t *rsa_word(SstarCryptoState *s)
{
    return &s->ram[s->regs[RSA_IND32_ADDR] & (RSA_RAM_WORDS - 1)];
}

/* Both halves of a word moved: step to the next, if asked to. */
static void rsa_word_done(SstarCryptoState *s, uint8_t *halves, int half)
{
    *halves |= 1 << half;
    if (*halves == 3) {
        *halves = 0;
        if (s->regs[RSA_IND32_CTRL] & RSA_IND32_AUTO_INC) {
            s->regs[RSA_IND32_ADDR]++;
        }
    }
}

/* ---- MMIO ----------------------------------------------------------- */

static uint16_t sstar_crypto_reg_read(SstarCryptoState *s, int reg)
{
    uint16_t v;

    switch (reg) {
    case RSA_IND32_RDATA_L:
    case RSA_IND32_RDATA_H:
        v = *rsa_word(s) >> (reg == RSA_IND32_RDATA_H ? 16 : 0);
        rsa_word_done(s, &s->rd_halves, reg == RSA_IND32_RDATA_H);
        return v;
    default:
        return s->regs[reg];
    }
}

static void sstar_crypto_reg_write(SstarCryptoState *s, int reg, uint16_t v)
{
    uint16_t old = s->regs[reg];
    uint32_t *w;

    s->regs[reg] = v;
    switch (reg) {
    case SHA_CTRL:
        if (v & SHA_CTRL_RST) {
            s->regs[SHA_STATUS] = 0;
        }
        if ((v & SHA_CTRL_FIRE) && !(old & SHA_CTRL_FIRE)) {
            sstar_sha_run(s);
        }
        break;
    case RSA_IND32_ADDR:
        s->wr_halves = s->rd_halves = 0;
        break;
    case RSA_IND32_WDATA_L:
    case RSA_IND32_WDATA_H:
        if (s->regs[RSA_IND32_CTRL] & RSA_IND32_WRITE) {
            w = rsa_word(s);
            if (reg == RSA_IND32_WDATA_H) {
                *w = (*w & 0xFFFF) | (v << 16);
            } else {
                *w = (*w & 0xFFFF0000) | v;
            }
            rsa_word_done(s, &s->wr_halves, reg == RSA_IND32_WDATA_H);
        }
        break;
    case RSA_EXP:
        if (v & RSA_EXP_START) {
            sstar_rsa_run(s);
        }
        s->regs[reg] = 0;
        break;
    case RSA_CTRL:
        if (v & RSA_CTRL_RST) {
            s->regs[RSA_STATUS] = 0;
            s->wr_halves = s->rd_halves = 0;
        }
        break;
    }
}

static uint64_t sstar_crypto_read(void *opaque, hwaddr off, unsigned size)
{
    SstarCryptoState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = sstar_crypto_reg_read(s, (off >> 2) & 0x7F);
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_crypto_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    SstarCryptoState *s = opaque;
    int reg = (off >> 2) & 0x7F;
    uint16_t v = val;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (s->regs[reg] & 0x00FF) | ((val & 0xFF) << 8)
                    : (s->regs[reg] & 0xFF00) | (val & 0xFF);
    }
    sstar_crypto_reg_write(s, reg, v);
}

static const MemoryRegionOps sstar_crypto_ops = {
    .read = sstar_crypto_read,
    .write = sstar_crypto_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_crypto_reset(DeviceState *dev)
{
    SstarCryptoState *s = SSTAR_CRYPTO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->ram, 0, sizeof(s->ram));
    s->wr_halves = s->rd_halves = 0;
}

static void sstar_crypto_init(Object *obj)
{
    SstarCryptoState *s = SSTAR_CRYPTO(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_crypto_ops, s,
                          TYPE_SSTAR_CRYPTO, 0x200);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_sstar_crypto = {
    .name = TYPE_SSTAR_CRYPTO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarCryptoState, 0x80),
        VMSTATE_UINT32_ARRAY(ram, SstarCryptoState, RSA_RAM_WORDS),
        VMSTATE_UINT8(wr_halves, SstarCryptoState),
        VMSTATE_UINT8(rd_halves, SstarCryptoState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_crypto_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_crypto_reset);
    dc->vmsd = &vmstate_sstar_crypto;
}

static const TypeInfo sstar_crypto_info = {
    .name           = TYPE_SSTAR_CRYPTO,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarCryptoState),
    .instance_init  = sstar_crypto_init,
    .class_init     = sstar_crypto_class_init,
};

static void sstar_crypto_register_types(void)
{
    type_register_static(&sstar_crypto_info);
}

type_init(sstar_crypto_register_types)
