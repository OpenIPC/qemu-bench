/*
 * Ingenic AES: the AES engine at 0x13430000 (every part; INTC source 23).
 *
 * As the SDKs' sample_aes (jz-aes.c) and the 4.4 kernel's crypto driver
 * (drivers/crypto/ingenic-aes.c) use it:
 *
 *   0x00 ASCR   bit 0 enable, 1 load the IV, 2 expand the key, 3 one
 *               block from ASDI to ASDO, 4 decrypt, 5 CBC (else ECB),
 *               7:6 key length (128, 192, 256), 8 DMA enable, 9 DMA
 *               start, 10 clear the key and IV; 1, 2, 3, 9 and 10 are
 *               commands and read back 0
 *   0x04 ASSR   bit 0 key done, 1 block done, 2 DMA done; write 1 clears
 *   0x08 ASINTM bit n enables ASSR bit n's interrupt
 *   0x0C ASSA / 0x10 ASDA   DMA source / destination (physical)
 *   0x14 ASTC   DMA length in 16-byte blocks (the sample driver's len / 16)
 *   0x18 ASDI / 0x1C ASDO   one block in / out, four words each
 *   0x20 ASKY / 0x24 ASIV   the key / IV, a word per write
 *
 * Words are big-endian: the first byte of the key is the top byte of the
 * first word written to ASKY, and the DMA reads and writes memory as
 * little-endian words whose top byte comes first -- the 4.4 driver
 * byte-swaps every word of its data before and after (prep_sgdata).
 * Everything completes at once.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "crypto/aes.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "system/dma.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_AES "ingenic-aes"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicAesState, INGENIC_AES)

#define R_ASCR          0x00
#define R_ASSR          0x04
#define R_ASINTM        0x08
#define R_ASSA          0x0C
#define R_ASDA          0x10
#define R_ASTC          0x14
#define R_ASDI          0x18
#define R_ASDO          0x1C
#define R_ASKY          0x20
#define R_ASIV          0x24

#define ASCR_EN         (1u << 0)
#define ASCR_INIT_IV    (1u << 1)
#define ASCR_KEYS       (1u << 2)
#define ASCR_AESS       (1u << 3)
#define ASCR_DECE       (1u << 4)
#define ASCR_CBC        (1u << 5)
#define ASCR_DMAE       (1u << 8)
#define ASCR_DMAS       (1u << 9)
#define ASCR_CLR        (1u << 10)
#define ASCR_CMDS       (ASCR_INIT_IV | ASCR_KEYS | ASCR_AESS | ASCR_DMAS | \
                         ASCR_CLR)

#define ASSR_KEYD       (1u << 0)
#define ASSR_AESD       (1u << 1)
#define ASSR_DMAD       (1u << 2)

struct IngenicAesState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t ascr, assr, asintm, assa, asda, astc;
    uint32_t key[8], iv[4], din[4], dout[4];
    uint32_t nkey, niv, ndin, ndout;
    uint8_t chain[16];              /* the CBC chaining value */
    uint32_t keyd;                  /* the key is expanded */

    AES_KEY enc, dec;               /* rebuilt from key[] after migration */
};

static void aes_update(IngenicAesState *s)
{
    qemu_set_irq(s->irq, (s->assr & s->asintm & 7) != 0);
}

static void aes_expand(IngenicAesState *s)
{
    int bits = 128 + 64 * MIN(extract32(s->ascr, 6, 2), 2);
    uint8_t k[32];
    int i;

    for (i = 0; i < 8; i++) {
        stl_be_p(k + 4 * i, s->key[i]);
    }
    AES_set_encrypt_key(k, bits, &s->enc);
    AES_set_decrypt_key(k, bits, &s->dec);
}

/* one 16-byte block, in the AES's own byte order */
static void aes_block(IngenicAesState *s, const uint8_t *in, uint8_t *out)
{
    uint8_t t[16];
    int i;

    if (!(s->ascr & ASCR_DECE)) {
        memcpy(t, in, 16);
        if (s->ascr & ASCR_CBC) {
            for (i = 0; i < 16; i++) {
                t[i] ^= s->chain[i];
            }
        }
        AES_encrypt(t, out, &s->enc);
        memcpy(s->chain, out, 16);
    } else {
        AES_decrypt(in, t, &s->dec);
        if (s->ascr & ASCR_CBC) {
            for (i = 0; i < 16; i++) {
                t[i] ^= s->chain[i];
            }
        }
        memcpy(s->chain, in, 16);
        memcpy(out, t, 16);
    }
}

static void aes_dma(IngenicAesState *s)
{
    uint8_t in[16], out[16];
    uint32_t n;
    int i;

    for (n = 0; n < s->astc; n++) {
        if (dma_memory_read(&address_space_memory, s->assa + 16 * n, in, 16,
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-aes: bad source 0x%x\n",
                          s->assa + 16 * n);
            break;
        }
        for (i = 0; i < 16; i += 4) {
            stl_be_p(in + i, ldl_le_p(in + i));
        }
        aes_block(s, in, out);
        for (i = 0; i < 16; i += 4) {
            stl_le_p(out + i, ldl_be_p(out + i));
        }
        dma_memory_write(&address_space_memory, s->asda + 16 * n, out, 16,
                         MEMTXATTRS_UNSPECIFIED);
    }
    s->assr |= ASSR_DMAD;
}

static void aes_command(IngenicAesState *s, uint32_t cmd)
{
    uint8_t in[16], out[16];
    int i;

    if (cmd & ASCR_CLR) {
        memset(s->key, 0, sizeof(s->key));
        memset(s->iv, 0, sizeof(s->iv));
        memset(s->chain, 0, sizeof(s->chain));
        s->nkey = s->niv = s->ndin = s->ndout = 0;
        s->keyd = 0;
    }
    if (!(s->ascr & ASCR_EN)) {
        return;
    }
    if (cmd & ASCR_INIT_IV) {
        for (i = 0; i < 4; i++) {
            stl_be_p(s->chain + 4 * i, s->iv[i]);
        }
        s->niv = 0;
    }
    if (cmd & ASCR_KEYS) {
        aes_expand(s);
        s->keyd = 1;
        s->nkey = 0;
        s->assr |= ASSR_KEYD;
    }
    if ((cmd & ASCR_AESS) && s->keyd) {
        for (i = 0; i < 4; i++) {
            stl_be_p(in + 4 * i, s->din[i]);
        }
        aes_block(s, in, out);
        for (i = 0; i < 4; i++) {
            s->dout[i] = ldl_be_p(out + 4 * i);
        }
        s->ndin = s->ndout = 0;
        s->assr |= ASSR_AESD;
    }
    if ((cmd & ASCR_DMAS) && (s->ascr & ASCR_DMAE) && s->keyd) {
        aes_dma(s);
    }
}

static uint64_t ingenic_aes_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicAesState *s = opaque;

    switch (off) {
    case R_ASCR:
        return s->ascr;
    case R_ASSR:
        return s->assr;
    case R_ASINTM:
        return s->asintm;
    case R_ASSA:
        return s->assa;
    case R_ASDA:
        return s->asda;
    case R_ASTC:
        return s->astc;
    case R_ASDO:
        return s->dout[s->ndout++ & 3];
    }
    return 0;
}

static void ingenic_aes_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicAesState *s = opaque;

    switch (off) {
    case R_ASCR:
        s->ascr = val & ~ASCR_CMDS;
        aes_command(s, val & ASCR_CMDS);
        break;
    case R_ASSR:
        s->assr &= ~val;
        break;
    case R_ASINTM:
        s->asintm = val & 7;
        break;
    case R_ASSA:
        s->assa = val;
        break;
    case R_ASDA:
        s->asda = val;
        break;
    case R_ASTC:
        s->astc = val;
        break;
    case R_ASDI:
        s->din[s->ndin++ & 3] = val;
        break;
    case R_ASKY:
        s->key[s->nkey++ & 7] = val;
        break;
    case R_ASIV:
        s->iv[s->niv++ & 3] = val;
        break;
    }
    aes_update(s);
}

static const MemoryRegionOps ingenic_aes_ops = {
    .read = ingenic_aes_read,
    .write = ingenic_aes_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_aes_reset(DeviceState *dev)
{
    IngenicAesState *s = INGENIC_AES(dev);

    s->ascr = s->assr = s->asintm = s->assa = s->asda = s->astc = 0;
    memset(s->key, 0, sizeof(s->key));
    memset(s->iv, 0, sizeof(s->iv));
    memset(s->din, 0, sizeof(s->din));
    memset(s->dout, 0, sizeof(s->dout));
    memset(s->chain, 0, sizeof(s->chain));
    s->nkey = s->niv = s->ndin = s->ndout = 0;
    s->keyd = 0;
    aes_update(s);
}

static void ingenic_aes_init(Object *obj)
{
    IngenicAesState *s = INGENIC_AES(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_aes_ops, s,
                          TYPE_INGENIC_AES, 0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static int ingenic_aes_post_load(void *opaque, int version_id)
{
    IngenicAesState *s = opaque;

    if (s->keyd) {
        aes_expand(s);
    }
    return 0;
}

static const VMStateDescription vmstate_ingenic_aes = {
    .name = TYPE_INGENIC_AES,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ingenic_aes_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ascr, IngenicAesState),
        VMSTATE_UINT32(assr, IngenicAesState),
        VMSTATE_UINT32(asintm, IngenicAesState),
        VMSTATE_UINT32(assa, IngenicAesState),
        VMSTATE_UINT32(asda, IngenicAesState),
        VMSTATE_UINT32(astc, IngenicAesState),
        VMSTATE_UINT32_ARRAY(key, IngenicAesState, 8),
        VMSTATE_UINT32_ARRAY(iv, IngenicAesState, 4),
        VMSTATE_UINT32_ARRAY(din, IngenicAesState, 4),
        VMSTATE_UINT32_ARRAY(dout, IngenicAesState, 4),
        VMSTATE_UINT32(nkey, IngenicAesState),
        VMSTATE_UINT32(niv, IngenicAesState),
        VMSTATE_UINT32(ndin, IngenicAesState),
        VMSTATE_UINT32(ndout, IngenicAesState),
        VMSTATE_UINT8_ARRAY(chain, IngenicAesState, 16),
        VMSTATE_UINT32(keyd, IngenicAesState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_aes_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_aes_reset);
    dc->vmsd = &vmstate_ingenic_aes;
}

static const TypeInfo ingenic_aes_info = {
    .name = TYPE_INGENIC_AES,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicAesState),
    .instance_init = ingenic_aes_init,
    .class_init = ingenic_aes_class_init,
};

static void ingenic_aes_register_types(void)
{
    type_register_static(&ingenic_aes_info);
}

type_init(ingenic_aes_register_types)
