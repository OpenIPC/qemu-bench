/*
 * Fullhan AES/DES engine (drivers/crypto/fh_aes.c), at 0xE8200000.
 *
 *   0x00 CONTROL   bit 0 decrypt, 3:1 cipher (0 DES, 1 3DES, 4/5/6
 *                  AES-128/192/256), 6:4 mode (0 ECB, 1 CBC, 2 CTR, 4 CFB,
 *                  5 OFB), bit 7 take the IV registers for this transfer
 *   0x10-0x2C KEY0-7, 0x30-0x3C IV0-3   big-endian words of the key / IV
 *   0x48 / 0x4C / 0x50 DMA source, destination, byte count
 *   0x54 DMA_CONTROL   1 starts the transfer
 *   0x5C INTR_ENABLE, 0x60 INTR_SRC (bit 0 done), 0x64 masked, 0x68 clear
 *   0x80-0x8C LAST_IV  the chaining value after the transfer
 *
 * The driver feeds a request in 2 KiB DMA transfers and loads the IV only
 * with the first, so the chaining value carries over between transfers.
 * CFB and OFB run on whole blocks (CFB-128 / CFB-64), CTR counts the whole
 * block big-endian -- the kernel's cfb(aes), ofb(aes), ctr(aes).  AES is
 * QEMU's own implementation, always built; DES and 3DES need a crypto
 * backend (gnutls, nettle or gcrypt) and are logged as unimplemented
 * without one.  Keys the
 * eFuse would load (its "transfer key" path) are not modelled: the blank
 * fuses give zeros.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "crypto/aes.h"
#include "crypto/cipher.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_FH_AES "fh-aes"
OBJECT_DECLARE_SIMPLE_TYPE(FhAesState, FH_AES)

#define FH_AES_SIZE         0x4000
#define FH_AES_NREGS        (0x90 / 4)
#define FH_AES_CHUNK        4096    /* bytes moved per step; a block multiple */

#define R_CONTROL           (0x00 / 4)
#define R_KEY0              (0x10 / 4)
#define R_IV0               (0x30 / 4)
#define R_DMA_SRC           (0x48 / 4)
#define R_DMA_DST           (0x4C / 4)
#define R_DMA_SIZE          (0x50 / 4)
#define R_DMA_CONTROL       (0x54 / 4)
#define R_INTR_ENABLE       (0x5C / 4)
#define R_INTR_SRC          (0x60 / 4)
#define R_INTR_MASKED       (0x64 / 4)
#define R_INTR_CLEAR        (0x68 / 4)
#define R_LAST_IV0          (0x80 / 4)

#define CTL_DECRYPT         (1 << 0)
#define CTL_LOAD_IV         (1 << 7)
#define INTR_DONE           (1 << 0)

enum { MODE_ECB = 0, MODE_CBC = 1, MODE_CTR = 2, MODE_CFB = 4, MODE_OFB = 5 };

struct FhAesState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[FH_AES_NREGS];
    uint8_t chain[16];          /* IV / counter / feedback */
};

static void fh_aes_update_irq(FhAesState *s)
{
    s->regs[R_INTR_MASKED] = s->regs[R_INTR_SRC] & s->regs[R_INTR_ENABLE];
    qemu_set_irq(s->irq, s->regs[R_INTR_MASKED] != 0);
}

/* The block cipher a transfer runs: QEMU's AES, or a backend's DES. */
typedef struct FhAesCipher {
    size_t bs;
    AES_KEY ek, dk;
    QCryptoCipher *des;
} FhAesCipher;

static bool fh_aes_cipher(FhAesState *s, FhAesCipher *c)
{
    uint8_t key[32];
    QCryptoCipherAlgo algo;
    Error *err = NULL;
    unsigned bits = 0;
    size_t klen;
    int i;

    for (i = 0; i < 8; i++) {
        stl_be_p(key + 4 * i, s->regs[R_KEY0 + i]);
    }
    switch ((s->regs[R_CONTROL] >> 1) & 7) {
    case 0: algo = QCRYPTO_CIPHER_ALGO_DES;  klen = 8;  break;
    case 1: algo = QCRYPTO_CIPHER_ALGO_3DES; klen = 24; break;
    case 4: bits = 128; break;
    case 5: bits = 192; break;
    case 6: bits = 256; break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "fh-aes: bad cipher in 0x%08x\n",
                      s->regs[R_CONTROL]);
        return false;
    }
    if (bits) {
        c->bs = AES_BLOCK_SIZE;
        AES_set_encrypt_key(key, bits, &c->ek);
        AES_set_decrypt_key(key, bits, &c->dk);
        return true;
    }
    c->bs = 8;
    c->des = qcrypto_cipher_new(algo, QCRYPTO_CIPHER_MODE_ECB, key, klen,
                                &err);
    if (!c->des) {
        qemu_log_mask(LOG_UNIMP, "fh-aes: %s\n", error_get_pretty(err));
        error_free(err);
        return false;
    }
    return true;
}

static void fh_aes_enc(FhAesCipher *c, const uint8_t *in, uint8_t *out)
{
    if (c->des) {
        qcrypto_cipher_encrypt(c->des, in, out, c->bs, NULL);
    } else {
        AES_encrypt(in, out, &c->ek);
    }
}

static void fh_aes_dec(FhAesCipher *c, const uint8_t *in, uint8_t *out)
{
    if (c->des) {
        qcrypto_cipher_decrypt(c->des, in, out, c->bs, NULL);
    } else {
        AES_decrypt(in, out, &c->dk);
    }
}

static void fh_aes_ctr_inc(uint8_t *ctr, size_t bs)
{
    for (int i = bs - 1; i >= 0 && !++ctr[i]; i--) {
    }
}

/* One block through the selected mode; in and out may not alias. */
static void fh_aes_block(FhAesState *s, FhAesCipher *c,
                         const uint8_t *in, uint8_t *out)
{
    size_t bs = c->bs;
    bool dec = s->regs[R_CONTROL] & CTL_DECRYPT;
    uint8_t t[16];
    size_t i;

    switch ((s->regs[R_CONTROL] >> 4) & 7) {
    case MODE_ECB:
        if (dec) {
            fh_aes_dec(c, in, out);
        } else {
            fh_aes_enc(c, in, out);
        }
        break;
    case MODE_CBC:
        if (dec) {
            fh_aes_dec(c, in, t);
            for (i = 0; i < bs; i++) {
                out[i] = t[i] ^ s->chain[i];
            }
            memcpy(s->chain, in, bs);
        } else {
            for (i = 0; i < bs; i++) {
                t[i] = in[i] ^ s->chain[i];
            }
            fh_aes_enc(c, t, out);
            memcpy(s->chain, out, bs);
        }
        break;
    case MODE_CTR:
        fh_aes_enc(c, s->chain, t);
        for (i = 0; i < bs; i++) {
            out[i] = in[i] ^ t[i];
        }
        fh_aes_ctr_inc(s->chain, bs);
        break;
    case MODE_CFB:
        fh_aes_enc(c, s->chain, t);
        for (i = 0; i < bs; i++) {
            out[i] = in[i] ^ t[i];
        }
        memcpy(s->chain, dec ? in : out, bs);
        break;
    case MODE_OFB:
        fh_aes_enc(c, s->chain, t);
        memcpy(s->chain, t, bs);
        for (i = 0; i < bs; i++) {
            out[i] = in[i] ^ t[i];
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "fh-aes: bad mode in 0x%08x\n",
                      s->regs[R_CONTROL]);
        memcpy(out, in, bs);
        break;
    }
}

static void fh_aes_run(FhAesState *s)
{
    uint32_t len = s->regs[R_DMA_SIZE];
    dma_addr_t src = s->regs[R_DMA_SRC], dst = s->regs[R_DMA_DST];
    FhAesCipher c = { 0 };
    uint8_t in[FH_AES_CHUNK], out[FH_AES_CHUNK];
    bool ok;
    size_t n, off;
    int i;

    ok = fh_aes_cipher(s, &c);
    if (s->regs[R_CONTROL] & CTL_LOAD_IV) {
        for (i = 0; i < 4; i++) {
            stl_be_p(s->chain + 4 * i, s->regs[R_IV0 + i]);
        }
    }
    if (ok) {
        /* A chunk at a time: the length is the guest's, unbounded. */
        len -= len % c.bs;
        while (len) {
            n = MIN(len, FH_AES_CHUNK);
            if (dma_memory_read(&address_space_memory, src, in, n,
                                MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
                qemu_log_mask(LOG_GUEST_ERROR, "fh-aes: bad source 0x%"
                              HWADDR_PRIx "\n", (hwaddr)src);
                break;
            }
            for (off = 0; off < n; off += c.bs) {
                fh_aes_block(s, &c, in + off, out + off);
            }
            dma_memory_write(&address_space_memory, dst, out, n,
                             MEMTXATTRS_UNSPECIFIED);
            src += n;
            dst += n;
            len -= n;
        }
    }
    if (c.des) {
        qcrypto_cipher_free(c.des);
    }
    for (i = 0; i < 4; i++) {
        s->regs[R_LAST_IV0 + i] = ldl_be_p(s->chain + 4 * i);
    }
    s->regs[R_DMA_CONTROL] = 0;
    s->regs[R_INTR_SRC] |= INTR_DONE;
    fh_aes_update_irq(s);
}

static uint64_t fh_aes_read(void *opaque, hwaddr off, unsigned size)
{
    FhAesState *s = FH_AES(opaque);

    if (off >= FH_AES_NREGS * 4) {
        return 0;
    }
    return extract32(s->regs[off >> 2], (off & 3) * 8, size * 8);
}

static void fh_aes_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    FhAesState *s = FH_AES(opaque);
    int r = off >> 2;

    if (off >= FH_AES_NREGS * 4) {
        return;
    }
    /* the driver memcpy()s the key and IV in */
    val = deposit32(s->regs[r], (off & 3) * 8, size * 8, val);

    switch (r) {
    case R_INTR_CLEAR:
        s->regs[R_INTR_SRC] &= ~val;
        fh_aes_update_irq(s);
        break;
    case R_INTR_ENABLE:
        s->regs[r] = val;
        fh_aes_update_irq(s);
        break;
    case R_DMA_CONTROL:
        s->regs[r] = val & 1;
        if (val & 1) {
            fh_aes_run(s);
        }
        break;
    case R_INTR_SRC:
    case R_INTR_MASKED:
    case R_LAST_IV0 ... R_LAST_IV0 + 3:
        break;
    default:
        s->regs[r] = val;
        break;
    }
}

static const MemoryRegionOps fh_aes_ops = {
    .read = fh_aes_read,
    .write = fh_aes_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void fh_aes_reset(DeviceState *dev)
{
    FhAesState *s = FH_AES(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->chain, 0, sizeof(s->chain));
    qemu_set_irq(s->irq, 0);
}

static void fh_aes_init(Object *obj)
{
    FhAesState *s = FH_AES(obj);

    memory_region_init_io(&s->iomem, obj, &fh_aes_ops, s, TYPE_FH_AES,
                          FH_AES_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_fh_aes = {
    .name = TYPE_FH_AES,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, FhAesState, FH_AES_NREGS),
        VMSTATE_UINT8_ARRAY(chain, FhAesState, 16),
        VMSTATE_END_OF_LIST()
    },
};

static void fh_aes_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, fh_aes_reset);
    dc->vmsd = &vmstate_fh_aes;
}

static const TypeInfo fh_aes_info = {
    .name           = TYPE_FH_AES,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhAesState),
    .instance_init  = fh_aes_init,
    .class_init     = fh_aes_class_init,
};

static void fh_aes_register_types(void)
{
    type_register_static(&fh_aes_info);
}

type_init(fh_aes_register_types)
