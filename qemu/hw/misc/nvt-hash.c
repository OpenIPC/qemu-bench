/*
 * Novatek NA51089 hash engine at 0xF0670000 (SPI 28): SHA-1, SHA-256 and
 * their HMACs, by DMA.
 *
 * drivers/crypto/nvt_ivot/na51089_hash.c feeds a request in chunks,
 * carrying the state between them through the IV registers.  Registers:
 *
 *   0x00 CFG       bit 0 software reset (clears itself); bit 1 starts a
 *                  DMA run; [5:4] 0 SHA-1, 1 SHA-256, 2 HMAC-SHA-1,
 *                  3 HMAC-SHA-256; bit 8 start from the IV registers (else
 *                  the algorithm's IV); bit 12 DMA (PIO through 0x90 is
 *                  not used, nor modelled); bit 13 pad the last block for
 *                  a message of PADLEN bytes; bit 21 HMAC: hash K ^ ipad
 *                  first; bit 22 HMAC: finish with the outer hash
 *   0x04 PADLEN    the whole message's length in bytes
 *   0x08 INT_ENB   0x0C INT_STS: bit 0 DMA done (write 1 to clear);
 *                  SPI 28 = STS & ENB
 *   0x10..0x4C     KEY, the HMAC key (64 bytes)
 *   0x50..0x6C     IV, 0x70..0x8C OUT: the chaining state or digest
 *   0x94 DMA_SRC   0x9C DMA_TX_SIZE: the chunk, in bytes
 *
 * Keys, data, IV and OUT are byte strings (a register holds the next four
 * bytes, little-endian), so OUT reads the digest as the driver copies it
 * out.  A chunk that is not the last is whole 64-byte blocks.  The HMAC
 * padding counts the K ^ ipad block too.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/dma.h"

#define TYPE_NVT_HASH "nvt-hash"
OBJECT_DECLARE_SIMPLE_TYPE(NvtHashState, NVT_HASH)

#define HS_REGS_SIZE    0x100
#define HS_CFG          0x00
#define HS_PADLEN       0x04
#define HS_INT_ENB      0x08
#define HS_INT_STS      0x0C
#define HS_KEY          0x10
#define HS_IV           0x50
#define HS_OUT          0x70
#define HS_DMA_SRC      0x94
#define HS_DMA_SIZE     0x9C

#define CFG_RESET       (1u << 0)
#define CFG_START       (1u << 1)
#define CFG_IV_REGS     (1u << 8)
#define CFG_DMA         (1u << 12)
#define CFG_PAD         (1u << 13)
#define CFG_HMAC_IPAD   (1u << 21)
#define CFG_HMAC_OPAD   (1u << 22)

struct NvtHashState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[HS_REGS_SIZE / 4];
};

/* SHA-1 / SHA-256 compression */

typedef struct {
    bool sha256;
    uint32_t h[8];
    uint64_t len;               /* bytes compressed */
} NvtSha;

static const uint32_t sha1_iv[5] = {
    0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0,
};

static const uint32_t sha256_iv[8] = {
    0x6A09E667, 0xBB67AE85, 0x3C6EF372, 0xA54FF53A,
    0x510E527F, 0x9B05688C, 0x1F83D9AB, 0x5BE0CD19,
};

static const uint32_t sha256_k[64] = {
    0x428A2F98, 0x71374491, 0xB5C0FBCF, 0xE9B5DBA5, 0x3956C25B, 0x59F111F1,
    0x923F82A4, 0xAB1C5ED5, 0xD807AA98, 0x12835B01, 0x243185BE, 0x550C7DC3,
    0x72BE5D74, 0x80DEB1FE, 0x9BDC06A7, 0xC19BF174, 0xE49B69C1, 0xEFBE4786,
    0x0FC19DC6, 0x240CA1CC, 0x2DE92C6F, 0x4A7484AA, 0x5CB0A9DC, 0x76F988DA,
    0x983E5152, 0xA831C66D, 0xB00327C8, 0xBF597FC7, 0xC6E00BF3, 0xD5A79147,
    0x06CA6351, 0x14292967, 0x27B70A85, 0x2E1B2138, 0x4D2C6DFC, 0x53380D13,
    0x650A7354, 0x766A0ABB, 0x81C2C92E, 0x92722C85, 0xA2BFE8A1, 0xA81A664B,
    0xC24B8B70, 0xC76C51A3, 0xD192E819, 0xD6990624, 0xF40E3585, 0x106AA070,
    0x19A4C116, 0x1E376C08, 0x2748774C, 0x34B0BCB5, 0x391C0CB3, 0x4ED8AA4A,
    0x5B9CCA4F, 0x682E6FF3, 0x748F82EE, 0x78A5636F, 0x84C87814, 0x8CC70208,
    0x90BEFFFA, 0xA4506CEB, 0xBEF9A3F7, 0xC67178F2,
};

static void sha1_block(uint32_t *h, const uint8_t *p)
{
    uint32_t w[80], a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f, k, t;
    int i;

    for (i = 0; i < 16; i++) {
        w[i] = ldl_be_p(p + 4 * i);
    }
    for (; i < 80; i++) {
        w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }
    for (i = 0; i < 80; i++) {
        if (i < 20) {
            f = (b & c) | (~b & d);
            k = 0x5A827999;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ED9EBA1;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8F1BBCDC;
        } else {
            f = b ^ c ^ d;
            k = 0xCA62C1D6;
        }
        t = rol32(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = rol32(b, 30);
        b = a;
        a = t;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
}

static void sha256_block(uint32_t *h, const uint8_t *p)
{
    uint32_t w[64], v[8], s0, s1, t1, t2;
    int i;

    for (i = 0; i < 16; i++) {
        w[i] = ldl_be_p(p + 4 * i);
    }
    for (; i < 64; i++) {
        s0 = ror32(w[i - 15], 7) ^ ror32(w[i - 15], 18) ^ (w[i - 15] >> 3);
        s1 = ror32(w[i - 2], 17) ^ ror32(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    memcpy(v, h, sizeof(v));
    for (i = 0; i < 64; i++) {
        s1 = ror32(v[4], 6) ^ ror32(v[4], 11) ^ ror32(v[4], 25);
        t1 = v[7] + s1 + ((v[4] & v[5]) ^ (~v[4] & v[6])) + sha256_k[i] + w[i];
        s0 = ror32(v[0], 2) ^ ror32(v[0], 13) ^ ror32(v[0], 22);
        t2 = s0 + ((v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]));
        memmove(v + 1, v, 7 * sizeof(v[0]));
        v[4] += t1;
        v[0] = t1 + t2;
    }
    for (i = 0; i < 8; i++) {
        h[i] += v[i];
    }
}

static void sha_init(NvtSha *c, bool sha256)
{
    c->sha256 = sha256;
    memset(c->h, 0, sizeof(c->h));
    if (sha256) {
        memcpy(c->h, sha256_iv, sizeof(sha256_iv));
    } else {
        memcpy(c->h, sha1_iv, sizeof(sha1_iv));
    }
    c->len = 0;
}

static void sha_block(NvtSha *c, const uint8_t *p)
{
    if (c->sha256) {
        sha256_block(c->h, p);
    } else {
        sha1_block(c->h, p);
    }
    c->len += 64;
}

/* The last bytes of a message of 'total' bytes: pad and finish. */
static void sha_final(NvtSha *c, const uint8_t *p, size_t n, uint64_t total)
{
    uint8_t blk[128] = { 0 };
    size_t padded = n < 56 ? 64 : 128;

    memcpy(blk, p, n);
    blk[n] = 0x80;
    stq_be_p(blk + padded - 8, total * 8);
    sha_block(c, blk);
    if (padded == 128) {
        sha_block(c, blk + 64);
    }
}

static int sha_digest_size(const NvtSha *c)
{
    return c->sha256 ? 32 : 20;
}

static void sha_digest(const NvtSha *c, uint8_t *out)
{
    int i;

    for (i = 0; i < sha_digest_size(c) / 4; i++) {
        stl_be_p(out + 4 * i, c->h[i]);
    }
}

/* the engine */

static uint32_t *reg(NvtHashState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_hash_update_irq(NvtHashState *s)
{
    qemu_set_irq(s->irq, !!(*reg(s, HS_INT_STS) & *reg(s, HS_INT_ENB)));
}

static void nvt_hash_key_pad(NvtHashState *s, uint8_t *blk, uint8_t pad)
{
    int i;

    for (i = 0; i < 64; i++) {
        blk[i] = (*reg(s, HS_KEY + (i & ~3)) >> (8 * (i & 3))) ^ pad;
    }
}

/* Run a chunk; false if its DMA failed, with OUT then cleared. */
static bool nvt_hash_run(NvtHashState *s, uint32_t cfg)
{
    int mode = (cfg >> 4) & 3;
    bool hmac = mode >= 2;
    uint32_t src = *reg(s, HS_DMA_SRC), len = *reg(s, HS_DMA_SIZE);
    uint8_t blk[64], digest[32];
    g_autofree uint8_t *buf = NULL;
    NvtSha c;
    uint32_t done = 0, n;
    int i;

    if (!(cfg & CFG_DMA)) {
        qemu_log_mask(LOG_UNIMP, "nvt-hash: PIO mode\n");
        return false;
    }
    sha_init(&c, mode & 1);
    if (cfg & CFG_IV_REGS) {
        for (i = 0; i < 8; i++) {
            c.h[i] = bswap32(*reg(s, HS_IV + 4 * i));
        }
    }
    if (hmac && (cfg & CFG_HMAC_IPAD)) {
        nvt_hash_key_pad(s, blk, 0x36);
        sha_block(&c, blk);
    }

    buf = g_malloc(MIN(len, 64 * KiB) + 64);
    while (len - done >= 64) {
        n = MIN((len - done) & ~63u, 64 * KiB);
        if (dma_memory_read(&address_space_memory, src + done, buf, n,
                            MEMTXATTRS_UNSPECIFIED)) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-hash: DMA read failed\n");
            return false;
        }
        for (i = 0; i < n; i += 64) {
            sha_block(&c, buf + i);
        }
        done += n;
    }
    if (cfg & CFG_PAD) {
        n = len - done;
        if (dma_memory_read(&address_space_memory, src + done, buf, n,
                            MEMTXATTRS_UNSPECIFIED)) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-hash: DMA read failed\n");
            return false;
        }
        sha_final(&c, buf, n, (uint64_t)*reg(s, HS_PADLEN) + (hmac ? 64 : 0));
        if (hmac && (cfg & CFG_HMAC_OPAD)) {
            sha_digest(&c, digest);
            sha_init(&c, mode & 1);
            nvt_hash_key_pad(s, blk, 0x5C);
            sha_block(&c, blk);
            sha_final(&c, digest, sha_digest_size(&c),
                      64 + sha_digest_size(&c));
        }
    } else if (len != done) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "nvt-hash: %u bytes past the last whole block\n",
                      len - done);
    }

    for (i = 0; i < 8; i++) {
        *reg(s, HS_OUT + 4 * i) = bswap32(c.h[i]);
    }
    return true;
}

static uint64_t nvt_hash_read(void *opaque, hwaddr off, unsigned size)
{
    NvtHashState *s = opaque;

    return *reg(s, off);
}

static void nvt_hash_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    NvtHashState *s = opaque;

    switch (off) {
    case HS_CFG:
        if (val & CFG_RESET) {
            memset(s->regs, 0, sizeof(s->regs));
            break;
        }
        *reg(s, off) = val;
        if (val & CFG_START) {
            /*
             * The whole chunk at once: it is done before the next access.
             * One the engine could not read leaves no digest behind --
             * not the last request's -- and is still done, so the driver
             * sees the zeros rather than waiting out its timeout.
             */
            if (!nvt_hash_run(s, val)) {
                memset(reg(s, HS_OUT), 0, 32);
            }
            *reg(s, HS_INT_STS) |= 1;
        }
        break;
    case HS_INT_STS:
        *reg(s, off) &= ~val;
        break;
    case HS_OUT ... HS_OUT + 0x1C:
        break;                          /* read only */
    default:
        *reg(s, off) = val;
        break;
    }
    nvt_hash_update_irq(s);
}

static const MemoryRegionOps nvt_hash_ops = {
    .read = nvt_hash_read,
    .write = nvt_hash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_hash_reset(DeviceState *dev)
{
    NvtHashState *s = NVT_HASH(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void nvt_hash_init(Object *obj)
{
    NvtHashState *s = NVT_HASH(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_hash_ops, s, TYPE_NVT_HASH,
                          HS_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_nvt_hash = {
    .name = TYPE_NVT_HASH,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtHashState, HS_REGS_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_hash_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_hash_reset);
    dc->vmsd = &vmstate_nvt_hash;
}

static const TypeInfo nvt_hash_info = {
    .name = TYPE_NVT_HASH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtHashState),
    .instance_init = nvt_hash_init,
    .class_init = nvt_hash_class_init,
};

static void nvt_hash_register_types(void)
{
    type_register_static(&nvt_hash_info);
}

type_init(nvt_hash_register_types)
