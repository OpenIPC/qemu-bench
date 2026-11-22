/*
 * Rockchip crypto v3 (RV1106 / RV1126 / RK3568 generation): the hash
 * engine only.
 *
 * The vendor SPL checks every FIT image's sha256 with it (FIT_HW_CRYPTO,
 * "Can't find expected crypto device" otherwise), and U-Boot does the
 * same for the kernel.  The driver is U-Boot's crypto_v2.c:
 *
 *   RST_CTL (0x04) self-clearing resets; CRYPTO_VERSION_NEW (0x6f0) and
 *   the per-algorithm capability words HASH_VERSION (0x68c) / HMAC_VERSION
 *   (0x690) say what the engine has.
 *   HASH_CTL (0x48): mode in bits 7:4 (SHA1 0, MD5 1, SHA256 2, SHA512 8,
 *   SHA384 9), HW_PAD bit 2, ENABLE bit 0.
 *   DMA_LLI_ADDR (0x14) points at a descriptor list:
 *     { src_addr, src_len, dst_addr, dst_len, user_define, reserve,
 *       dma_ctrl, next_addr }
 *   user_define bit 1 = start of the message, bit 2 = its last piece;
 *   dma_ctrl bit 0 = last descriptor, bit 1 = pause after this one.
 *   DMA_CTL (0x10, hiword) START walks from DMA_LLI_ADDR, RESTART goes on
 *   from the paused descriptor's next_addr.  Each stop raises
 *   DMA_INT_ST (0x0c, write 1 to clear) SRC_ITEM_DONE (bit 2).
 *   After the last piece HASH_VALID (0x3e4) reads 1 and HASH_DOUT_0..15
 *   (0x3a0) hold the digest, word i = big-endian bytes 4i..4i+3.
 *
 * The hashing itself is QEMU's crypto/hash API (glib backend at least),
 * which has no SM3 or SHA-224: HASH_VERSION does not advertise them.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "crypto/hash.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "system/dma.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_RK_CRYPTO "rk-crypto"
OBJECT_DECLARE_SIMPLE_TYPE(RkCryptoState, RK_CRYPTO)

#define R_RST_CTL       0x004
#define R_DMA_INT_EN    0x008
#define R_DMA_INT_ST    0x00c
#define R_DMA_CTL       0x010
#define R_DMA_LLI_ADDR  0x014
#define R_DMA_ST        0x018
#define R_HASH_CTL      0x048
#define R_HASH_DOUT     0x3a0
#define R_HASH_VALID    0x3e4
#define R_HASH_VERSION  0x68c
#define R_VERSION_NEW   0x6f0
#define REGS_SIZE       0x1000

#define INT_SRC_ITEM_DONE   (1u << 2)
#define INT_LIST_ERR        (1u << 5)
#define INT_ZERO_LEN        (1u << 6)

#define LLI_CTRL_LAST       (1u << 0)
#define LLI_CTRL_PAUSE      (1u << 1)
#define LLI_USER_LAST       (1u << 2)

/* SHA1 | SHA256 | SHA384 | SHA512 | MD5 */
#define HASH_CAPS           0x0000009d

struct RkCryptoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[REGS_SIZE / 4];
    QCryptoHash *hash;
    uint32_t next_lli;
    uint8_t digest[64];
    bool valid;
    uint32_t version;       /* CRYPTO_VERSION_NEW: 0x03000000 v3 (RV1106),
                             * 0x04000000 v4 (RV1103B) */
};

static void rk_crypto_update_irq(RkCryptoState *s)
{
    qemu_set_irq(s->irq, (s->regs[R_DMA_INT_ST / 4] &
                          s->regs[R_DMA_INT_EN / 4]) != 0);
}

static int rk_crypto_algo(RkCryptoState *s, size_t *len)
{
    switch (extract32(s->regs[R_HASH_CTL / 4], 4, 4)) {
    case 0:
        *len = 20;
        return QCRYPTO_HASH_ALGO_SHA1;
    case 1:
        *len = 16;
        return QCRYPTO_HASH_ALGO_MD5;
    case 2:
        *len = 32;
        return QCRYPTO_HASH_ALGO_SHA256;
    case 8:
        *len = 64;
        return QCRYPTO_HASH_ALGO_SHA512;
    case 9:
        *len = 48;
        return QCRYPTO_HASH_ALGO_SHA384;
    }
    return -1;
}

static void rk_crypto_drop(RkCryptoState *s)
{
    if (s->hash) {
        qcrypto_hash_free(s->hash);
        s->hash = NULL;
    }
}

static void rk_crypto_finish_hash(RkCryptoState *s)
{
    g_autofree uint8_t *out = NULL;
    size_t len = 0;
    Error *err = NULL;
    int i;

    if (!s->hash) {
        return;
    }
    if (qcrypto_hash_finalize_bytes(s->hash, &out, &len, &err) < 0) {
        error_report_err(err);
        len = 0;
    }
    rk_crypto_drop(s);
    memset(s->digest, 0, sizeof(s->digest));
    memcpy(s->digest, out, MIN(len, sizeof(s->digest)));
    for (i = 0; i < 16; i++) {
        s->regs[R_HASH_DOUT / 4 + i] = ldl_be_p(s->digest + 4 * i);
    }
    s->valid = true;
}

/* Walk descriptors from addr until one pauses or ends the list. */
static void rk_crypto_run(RkCryptoState *s, uint32_t addr)
{
    uint32_t d[8];
    int guard;

    for (guard = 0; guard < 4096; guard++) {
        g_autofree uint8_t *buf = NULL;
        Error *err = NULL;
        size_t dlen;

        if (dma_memory_read(&address_space_memory, addr, d, sizeof(d),
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            s->regs[R_DMA_INT_ST / 4] |= INT_LIST_ERR;
            break;
        }
        for (int i = 0; i < 8; i++) {
            d[i] = le32_to_cpu(d[i]);
        }
        if ((s->regs[R_HASH_CTL / 4] & 1) && !s->hash) {
            int algo = rk_crypto_algo(s, &dlen);

            if (algo < 0) {
                qemu_log_mask(LOG_UNIMP, "rk-crypto: hash mode 0x%x\n",
                              s->regs[R_HASH_CTL / 4]);
                s->regs[R_DMA_INT_ST / 4] |= INT_LIST_ERR;
                break;
            }
            s->hash = qcrypto_hash_new(algo, &err);
            if (!s->hash) {
                error_report_err(err);
                s->regs[R_DMA_INT_ST / 4] |= INT_LIST_ERR;
                break;
            }
        }
        if (d[1] && s->hash) {
            /* in 64 KiB pieces: the length is the guest's, up to 4 GiB */
            uint32_t done, n;

            buf = g_realloc(buf, 64 * KiB);
            for (done = 0; done < d[1]; done += n) {
                n = MIN(d[1] - done, 64 * KiB);
                dma_memory_read(&address_space_memory, d[0] + done, buf, n,
                                MEMTXATTRS_UNSPECIFIED);
                if (qcrypto_hash_update(s->hash, (char *)buf, n, &err) < 0) {
                    error_report_err(err);
                    break;
                }
            }
        }
        if (d[4] & LLI_USER_LAST) {
            rk_crypto_finish_hash(s);
        }
        s->regs[R_DMA_INT_ST / 4] |= d[1] ? INT_SRC_ITEM_DONE : INT_ZERO_LEN;
        if ((d[6] & (LLI_CTRL_LAST | LLI_CTRL_PAUSE)) || !d[7]) {
            s->next_lli = d[7];
            break;
        }
        addr = d[7];
    }
    rk_crypto_update_irq(s);
}

static uint64_t rk_crypto_read(void *opaque, hwaddr off, unsigned size)
{
    RkCryptoState *s = opaque;

    switch (off) {
    case R_RST_CTL:
    case R_DMA_ST:
        return 0;               /* resets done, DMA idle */
    case R_HASH_VALID:
        return s->valid;
    case R_HASH_VERSION:
        return HASH_CAPS;
    case R_VERSION_NEW:
        return s->version;
    default:
        return s->regs[off / 4];
    }
}

static void rk_crypto_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    RkCryptoState *s = opaque;
    uint32_t *r = &s->regs[off / 4];
    uint32_t mask = val >> 16;

    switch (off) {
    case R_RST_CTL:
        if (val & mask & 1) {           /* SW_CC_RESET */
            rk_crypto_drop(s);
            s->valid = false;
        }
        return;
    case R_DMA_INT_ST:
        *r &= ~val;
        rk_crypto_update_irq(s);
        return;
    case R_DMA_CTL:
        if (val & mask & 1) {           /* START */
            rk_crypto_drop(s);
            s->valid = false;
            rk_crypto_run(s, s->regs[R_DMA_LLI_ADDR / 4]);
        } else if (val & mask & 2) {    /* RESTART */
            rk_crypto_run(s, s->next_lli);
        }
        return;
    case R_HASH_VALID:
        s->valid = false;
        return;
    case R_HASH_CTL:
    case 0x000:                         /* CLK_CTL */
    case 0x040:                         /* FIFO_CTL */
        *r = (*r & ~mask) | (val & mask);
        if (off == R_HASH_CTL && !(*r & 1)) {
            rk_crypto_drop(s);
        }
        return;
    default:
        *r = val;
    }
}

static const MemoryRegionOps rk_crypto_ops = {
    .read = rk_crypto_read,
    .write = rk_crypto_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_crypto_reset(DeviceState *dev)
{
    RkCryptoState *s = RK_CRYPTO(dev);

    rk_crypto_drop(s);
    memset(s->regs, 0, sizeof(s->regs));
    s->valid = false;
    s->next_lli = 0;
    rk_crypto_update_irq(s);
}

static void rk_crypto_realize(DeviceState *dev, Error **errp)
{
    RkCryptoState *s = RK_CRYPTO(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &rk_crypto_ops, s,
                          TYPE_RK_CRYPTO, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static const Property rk_crypto_properties[] = {
    DEFINE_PROP_UINT32("version", RkCryptoState, version, 0x03000000),
};

static void rk_crypto_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_props(dc, rk_crypto_properties);
    dc->realize = rk_crypto_realize;
    device_class_set_legacy_reset(dc, rk_crypto_reset);
    dc->desc = "Rockchip crypto v3 / v4 (hash engine)";
}

static const TypeInfo rk_crypto_info = {
    .name = TYPE_RK_CRYPTO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkCryptoState),
    .class_init = rk_crypto_class_init,
};

static void rk_crypto_register_types(void)
{
    type_register_static(&rk_crypto_info);
}

type_init(rk_crypto_register_types)
