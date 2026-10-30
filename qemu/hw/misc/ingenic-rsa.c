/*
 * Ingenic RSA: the modular exponentiation engine at 0x134C0000 (T23,
 * T31, T40, T41).
 *
 * As the SDKs' sample_rsa (jz-rsa.c) uses it, polling, without its
 * interrupt (INTC source 24, left unconnected):
 *
 *   0x00 RSAC  bit 0 off -> on: clear the operands; 1 start the pre-
 *              processing, 2 its done flag; 4 start the main run, 5 its
 *              done flag; 7 2048-bit (else 1024-bit) operands; 9 busy
 *              while a result word is not ready (never); 3 and 6 are
 *              flags too; 2, 3, 5, 6 write 1 to clear; 1 and 4 read back
 *              0; the rest is storage
 *   0x04 RSAE / 0x08 RSAN / 0x0C RSAM  the exponent, modulus and base,
 *              a 32-bit word per write, the most significant first
 *   0x10 RSAP  the result M^E mod N, a word per read, the least
 *              significant first
 *
 * The word orders are those that make the sample's test vectors
 * (sample_rsa/test/test_rsa.h, 100 pairs each of 1024 and 2048 bits)
 * decrypt to their plain texts.  The result is ready when the main run
 * is started.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_RSA "ingenic-rsa"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicRsaState, INGENIC_RSA)

#define R_RSAC          0x00
#define R_RSAE          0x04
#define R_RSAN          0x08
#define R_RSAM          0x0C
#define R_RSAP          0x10

#define RSAC_EN         (1u << 0)
#define RSAC_PRE        (1u << 1)
#define RSAC_PRE_DONE   (1u << 2)
#define RSAC_MAIN       (1u << 4)
#define RSAC_MAIN_DONE  (1u << 5)
#define RSAC_2048       (1u << 7)
#define RSAC_FLAGS      ((1u << 2) | (1u << 3) | (1u << 5) | (1u << 6))
#define RSAC_CMDS       (RSAC_PRE | RSAC_MAIN | (1u << 9))

#define MAXW            64          /* 2048 bits */

struct IngenicRsaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t rsac;
    uint32_t e[MAXW], n[MAXW], m[MAXW], p[MAXW];   /* as written */
    uint32_t ne, nn, nm, np;
};

/* --- multi-precision arithmetic, little-endian 32-bit limbs ------------- */

/* r = a * b * 2^(-32 k) mod n (Montgomery, CIOS); r may alias a or b */
static void mont_mul(uint32_t *r, const uint32_t *a, const uint32_t *b,
                     const uint32_t *n, uint32_t n0, int k)
{
    uint32_t t[MAXW + 2] = { 0 };
    int i, j;

    for (i = 0; i < k; i++) {
        uint64_t c = 0;
        uint32_t u;

        for (j = 0; j < k; j++) {
            c += (uint64_t)a[j] * b[i] + t[j];
            t[j] = c;
            c >>= 32;
        }
        c += t[k];
        t[k] = c;
        t[k + 1] = c >> 32;

        u = t[0] * n0;
        c = ((uint64_t)u * n[0] + t[0]) >> 32;
        for (j = 1; j < k; j++) {
            c += (uint64_t)u * n[j] + t[j];
            t[j - 1] = c;
            c >>= 32;
        }
        c += t[k];
        t[k - 1] = c;
        t[k] = t[k + 1] + (c >> 32);
    }
    /* t < 2 n: subtract n once if t >= n */
    if (!t[k]) {
        for (j = k - 1; j >= 0 && t[j] == n[j]; j--) {
            continue;
        }
        if (j >= 0 && t[j] < n[j]) {
            memcpy(r, t, k * 4);
            return;
        }
    }
    {
        int64_t bw = 0;

        for (j = 0; j < k; j++) {
            bw += (int64_t)t[j] - n[j];
            r[j] = bw;
            bw >>= 32;
        }
    }
}

/* r = b^e mod n, n odd */
static void mod_exp(uint32_t *r, const uint32_t *b, const uint32_t *e,
                    const uint32_t *n, int k)
{
    uint32_t r2[MAXW] = { 0 }, x[MAXW], bm[MAXW], one[MAXW] = { 1 };
    uint32_t n0 = 1;
    int i, j;

    /* n0 = -n^-1 mod 2^32 */
    for (i = 0; i < 5; i++) {
        n0 *= 2 - n[0] * n0;
    }
    n0 = -n0;

    /* r2 = 2^(64 k) mod n, by doubling 1 */
    r2[0] = 1;
    for (i = 0; i < 64 * k; i++) {
        uint32_t carry = 0;
        bool ge = true;

        for (j = 0; j < k; j++) {
            uint32_t v = r2[j];

            r2[j] = (v << 1) | carry;
            carry = v >> 31;
        }
        if (!carry) {
            for (j = k - 1; j >= 0 && r2[j] == n[j]; j--) {
                continue;
            }
            ge = j < 0 || r2[j] > n[j];
        }
        if (ge) {
            int64_t bw = 0;

            for (j = 0; j < k; j++) {
                bw += (int64_t)r2[j] - n[j];
                r2[j] = bw;
                bw >>= 32;
            }
        }
    }

    mont_mul(bm, b, r2, n, n0, k);
    mont_mul(x, one, r2, n, n0, k);
    for (i = 32 * k - 1; i >= 0; i--) {
        mont_mul(x, x, x, n, n0, k);
        if (extract32(e[i / 32], i % 32, 1)) {
            mont_mul(x, x, bm, n, n0, k);
        }
    }
    mont_mul(r, x, one, n, n0, k);
}

/* --- the device ------------------------------------------------------------ */

static void rsa_run(IngenicRsaState *s)
{
    int k = s->rsac & RSAC_2048 ? 64 : 32;
    uint32_t e[MAXW], n[MAXW], m[MAXW];
    int i;

    for (i = 0; i < k; i++) {
        e[i] = s->e[k - 1 - i];
        n[i] = s->n[k - 1 - i];
        m[i] = s->m[k - 1 - i];
    }
    if (!(n[0] & 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-rsa: even modulus\n");
        memset(s->p, 0, sizeof(s->p));
    } else {
        mod_exp(s->p, m, e, n, k);
    }
    s->np = 0;
}

static uint64_t ingenic_rsa_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicRsaState *s = opaque;

    switch (off) {
    case R_RSAC:
        return s->rsac;
    case R_RSAP:
        return s->p[s->np++ % MAXW];
    }
    return 0;
}

static void ingenic_rsa_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicRsaState *s = opaque;
    uint32_t old = s->rsac;

    switch (off) {
    case R_RSAC:
        s->rsac = (val & ~(RSAC_FLAGS | RSAC_CMDS)) |
                  (old & RSAC_FLAGS & ~val);
        if ((val & RSAC_EN) && !(old & RSAC_EN)) {
            memset(s->e, 0, sizeof(s->e));
            memset(s->n, 0, sizeof(s->n));
            memset(s->m, 0, sizeof(s->m));
            s->ne = s->nn = s->nm = s->np = 0;
        }
        if ((val & RSAC_PRE) && (s->rsac & RSAC_EN)) {
            s->rsac |= RSAC_PRE_DONE | (1u << 3);
        }
        if ((val & RSAC_MAIN) && (s->rsac & RSAC_EN)) {
            rsa_run(s);
            s->rsac |= RSAC_MAIN_DONE | (1u << 6);
        }
        break;
    case R_RSAE:
        s->e[s->ne++ % MAXW] = val;
        break;
    case R_RSAN:
        s->n[s->nn++ % MAXW] = val;
        break;
    case R_RSAM:
        s->m[s->nm++ % MAXW] = val;
        break;
    }
}

static const MemoryRegionOps ingenic_rsa_ops = {
    .read = ingenic_rsa_read,
    .write = ingenic_rsa_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_rsa_reset(DeviceState *dev)
{
    IngenicRsaState *s = INGENIC_RSA(dev);

    s->rsac = 0;
    memset(s->e, 0, sizeof(s->e));
    memset(s->n, 0, sizeof(s->n));
    memset(s->m, 0, sizeof(s->m));
    memset(s->p, 0, sizeof(s->p));
    s->ne = s->nn = s->nm = s->np = 0;
}

static void ingenic_rsa_init(Object *obj)
{
    IngenicRsaState *s = INGENIC_RSA(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_rsa_ops, s,
                          TYPE_INGENIC_RSA, 0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_ingenic_rsa = {
    .name = TYPE_INGENIC_RSA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(rsac, IngenicRsaState),
        VMSTATE_UINT32_ARRAY(e, IngenicRsaState, MAXW),
        VMSTATE_UINT32_ARRAY(n, IngenicRsaState, MAXW),
        VMSTATE_UINT32_ARRAY(m, IngenicRsaState, MAXW),
        VMSTATE_UINT32_ARRAY(p, IngenicRsaState, MAXW),
        VMSTATE_UINT32(ne, IngenicRsaState),
        VMSTATE_UINT32(nn, IngenicRsaState),
        VMSTATE_UINT32(nm, IngenicRsaState),
        VMSTATE_UINT32(np, IngenicRsaState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_rsa_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_rsa_reset);
    dc->vmsd = &vmstate_ingenic_rsa;
}

static const TypeInfo ingenic_rsa_info = {
    .name = TYPE_INGENIC_RSA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicRsaState),
    .instance_init = ingenic_rsa_init,
    .class_init = ingenic_rsa_class_init,
};

static void ingenic_rsa_register_types(void)
{
    type_register_static(&ingenic_rsa_info);
}

type_init(ingenic_rsa_register_types)
