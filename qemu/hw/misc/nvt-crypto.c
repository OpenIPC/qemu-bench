/*
 * Novatek NA51089 crypto engine at 0xF0620000 (SPI 38): AES-128/256, DES
 * and 3DES in ECB, CBC, CFB, OFB, CTR and (AES) GCM.
 *
 * Linux drives it from drivers/crypto/nvt_ivot/na51089_crypto.c, the
 * vendor U-Boot from arch/arm/mach-novatek/nvt_na51089_a32/crypto.c
 * (nvt_encrypt, the encrypted-kernel path).  Registers:
 *
 *   0x00 CFG       bit 0 software reset (clears itself), bit 1 enable;
 *                  for PIO, [6:4] the cipher and bit 8 decrypt
 *   0x04 CTRL      bit 0 starts a PIO block, [7:4] start DMA channels 0..3;
 *                  each clears when its work is done
 *   0x08 INT_ENB   0x0C INT_STS (write 1 to clear): bit 0 PIO done,
 *                  [7:4] DMA channel n done.  SPI 38 = STS & ENB
 *   0x10..0x2C     KEY, 0x30..0x3C IN, 0x40..0x4C OUT (PIO, one ECB block)
 *   0x50..0x5C     DMA descriptor address of channels 0..3
 *
 * Ciphers: 0 DES, 1 3DES (EDE, three keys), 2 AES-128, 3 AES-256.  Modes:
 * 0 ECB, 1 CBC, 2 CFB, 3 OFB, 4 CTR, 5 GCM.  Keys, IVs and data are byte
 * strings, in memory and in the registers alike (a register holds the
 * next four bytes, little-endian).
 *
 * A DMA descriptor (struct na51089_crypto_dma_desc):
 *
 *   0x00 key[32]    0x20 IV[16]    0x30 counter step[16]
 *   0x40 header: bit 0 decrypt, [7:4] cipher, [11:8] mode, bit 12 key
 *        from the OTP key manager, bit 16 "get S0"
 *   0x50 CV[16]     0x60 S0[16]    0x70 GHASH[16]      (written back)
 *   0x80 32 blocks of {src, dst, length, cfg}: cfg bit 0 ends a group,
 *        bit 8 suppresses the output write
 *
 * The blocks form one data stream, up to the first that ends a group;
 * in GCM that first group is the associated data and a second group the
 * text.  The engine writes back CV, the IV that continues the stream
 * (the last cipher block in CBC and CFB, the last keystream block in OFB,
 * the next counter in CTR and GCM): the driver chains requests longer
 * than 32 blocks through it, and CCM takes its CBC-MAC from it.  S0 is
 * E(K, first counter block): GCM's E(K, J0), or, with "get S0" in CTR
 * mode, the A0 block CCM needs, the text then starting at the next
 * counter.  GHASH is the GCM hash over the associated data, the
 * ciphertext and their lengths: the driver makes the tag S0 ^ GHASH.
 * The CTR counter advances by the step (its last word the low one) over
 * the whole block; GCM's over its last 32 bits.
 *
 * The key manager loads a key from the OTP's key sets; a blank OTP (as
 * nvt-otp is) gives an all-zero key.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "crypto/aes.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/dma.h"

#define TYPE_NVT_CRYPTO "nvt-crypto"
OBJECT_DECLARE_SIMPLE_TYPE(NvtCryptoState, NVT_CRYPTO)

#define CR_REGS_SIZE    0x100
#define CR_CFG          0x00
#define CR_CTRL         0x04
#define CR_INT_ENB      0x08
#define CR_INT_STS      0x0C
#define CR_KEY          0x10
#define CR_IN           0x30
#define CR_OUT          0x40
#define CR_DMA_ADDR     0x50

#define CFG_RESET       (1u << 0)
#define CTRL_PIO        (1u << 0)
#define CR_DMA_CH       4

enum { CIPHER_DES, CIPHER_3DES, CIPHER_AES128, CIPHER_AES256 };
enum { OP_ECB, OP_CBC, OP_CFB, OP_OFB, OP_CTR, OP_GCM };

#define DESC_KEY        0x00
#define DESC_IV         0x20
#define DESC_COUNTER    0x30
#define DESC_HEADER     0x40
#define DESC_CV         0x50
#define DESC_S0         0x60
#define DESC_GHASH      0x70
#define DESC_BLOCKS     0x80
#define DESC_NBLOCKS    32

#define HDR_DECRYPT     (1u << 0)
#define HDR_KEY_MANAGER (1u << 12)
#define HDR_GET_S0      (1u << 16)
#define BLK_LAST        (1u << 0)
#define BLK_NO_WRITE    (1u << 8)

struct NvtCryptoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[CR_REGS_SIZE / 4];
};

/* DES */

static const uint8_t des_ip[64] = {
    58, 50, 42, 34, 26, 18, 10, 2, 60, 52, 44, 36, 28, 20, 12, 4,
    62, 54, 46, 38, 30, 22, 14, 6, 64, 56, 48, 40, 32, 24, 16, 8,
    57, 49, 41, 33, 25, 17, 9, 1, 59, 51, 43, 35, 27, 19, 11, 3,
    61, 53, 45, 37, 29, 21, 13, 5, 63, 55, 47, 39, 31, 23, 15, 7,
};

static const uint8_t des_fp[64] = {
    40, 8, 48, 16, 56, 24, 64, 32, 39, 7, 47, 15, 55, 23, 63, 31,
    38, 6, 46, 14, 54, 22, 62, 30, 37, 5, 45, 13, 53, 21, 61, 29,
    36, 4, 44, 12, 52, 20, 60, 28, 35, 3, 43, 11, 51, 19, 59, 27,
    34, 2, 42, 10, 50, 18, 58, 26, 33, 1, 41, 9, 49, 17, 57, 25,
};

static const uint8_t des_e[48] = {
    32, 1, 2, 3, 4, 5, 4, 5, 6, 7, 8, 9, 8, 9, 10, 11, 12, 13,
    12, 13, 14, 15, 16, 17, 16, 17, 18, 19, 20, 21, 20, 21, 22, 23, 24, 25,
    24, 25, 26, 27, 28, 29, 28, 29, 30, 31, 32, 1,
};

static const uint8_t des_p[32] = {
    16, 7, 20, 21, 29, 12, 28, 17, 1, 15, 23, 26, 5, 18, 31, 10,
    2, 8, 24, 14, 32, 27, 3, 9, 19, 13, 30, 6, 22, 11, 4, 25,
};

static const uint8_t des_pc1[56] = {
    57, 49, 41, 33, 25, 17, 9, 1, 58, 50, 42, 34, 26, 18,
    10, 2, 59, 51, 43, 35, 27, 19, 11, 3, 60, 52, 44, 36,
    63, 55, 47, 39, 31, 23, 15, 7, 62, 54, 46, 38, 30, 22,
    14, 6, 61, 53, 45, 37, 29, 21, 13, 5, 28, 20, 12, 4,
};

static const uint8_t des_pc2[48] = {
    14, 17, 11, 24, 1, 5, 3, 28, 15, 6, 21, 10,
    23, 19, 12, 4, 26, 8, 16, 7, 27, 20, 13, 2,
    41, 52, 31, 37, 47, 55, 30, 40, 51, 45, 33, 48,
    44, 49, 39, 56, 34, 53, 46, 42, 50, 36, 29, 32,
};

static const uint8_t des_shifts[16] = {
    1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1,
};

static const uint8_t des_sbox[8][64] = {
    { 14, 4, 13, 1, 2, 15, 11, 8, 3, 10, 6, 12, 5, 9, 0, 7,
      0, 15, 7, 4, 14, 2, 13, 1, 10, 6, 12, 11, 9, 5, 3, 8,
      4, 1, 14, 8, 13, 6, 2, 11, 15, 12, 9, 7, 3, 10, 5, 0,
      15, 12, 8, 2, 4, 9, 1, 7, 5, 11, 3, 14, 10, 0, 6, 13 },
    { 15, 1, 8, 14, 6, 11, 3, 4, 9, 7, 2, 13, 12, 0, 5, 10,
      3, 13, 4, 7, 15, 2, 8, 14, 12, 0, 1, 10, 6, 9, 11, 5,
      0, 14, 7, 11, 10, 4, 13, 1, 5, 8, 12, 6, 9, 3, 2, 15,
      13, 8, 10, 1, 3, 15, 4, 2, 11, 6, 7, 12, 0, 5, 14, 9 },
    { 10, 0, 9, 14, 6, 3, 15, 5, 1, 13, 12, 7, 11, 4, 2, 8,
      13, 7, 0, 9, 3, 4, 6, 10, 2, 8, 5, 14, 12, 11, 15, 1,
      13, 6, 4, 9, 8, 15, 3, 0, 11, 1, 2, 12, 5, 10, 14, 7,
      1, 10, 13, 0, 6, 9, 8, 7, 4, 15, 14, 3, 11, 5, 2, 12 },
    { 7, 13, 14, 3, 0, 6, 9, 10, 1, 2, 8, 5, 11, 12, 4, 15,
      13, 8, 11, 5, 6, 15, 0, 3, 4, 7, 2, 12, 1, 10, 14, 9,
      10, 6, 9, 0, 12, 11, 7, 13, 15, 1, 3, 14, 5, 2, 8, 4,
      3, 15, 0, 6, 10, 1, 13, 8, 9, 4, 5, 11, 12, 7, 2, 14 },
    { 2, 12, 4, 1, 7, 10, 11, 6, 8, 5, 3, 15, 13, 0, 14, 9,
      14, 11, 2, 12, 4, 7, 13, 1, 5, 0, 15, 10, 3, 9, 8, 6,
      4, 2, 1, 11, 10, 13, 7, 8, 15, 9, 12, 5, 6, 3, 0, 14,
      11, 8, 12, 7, 1, 14, 2, 13, 6, 15, 0, 9, 10, 4, 5, 3 },
    { 12, 1, 10, 15, 9, 2, 6, 8, 0, 13, 3, 4, 14, 7, 5, 11,
      10, 15, 4, 2, 7, 12, 9, 5, 6, 1, 13, 14, 0, 11, 3, 8,
      9, 14, 15, 5, 2, 8, 12, 3, 7, 0, 4, 10, 1, 13, 11, 6,
      4, 3, 2, 12, 9, 5, 15, 10, 11, 14, 1, 7, 6, 0, 8, 13 },
    { 4, 11, 2, 14, 15, 0, 8, 13, 3, 12, 9, 7, 5, 10, 6, 1,
      13, 0, 11, 7, 4, 9, 1, 10, 14, 3, 5, 12, 2, 15, 8, 6,
      1, 4, 11, 13, 12, 3, 7, 14, 10, 15, 6, 8, 0, 5, 9, 2,
      6, 11, 13, 8, 1, 4, 10, 7, 9, 5, 0, 15, 14, 2, 3, 12 },
    { 13, 2, 8, 4, 6, 15, 11, 1, 10, 9, 3, 14, 5, 0, 12, 7,
      1, 15, 13, 8, 10, 3, 7, 4, 12, 5, 6, 11, 0, 14, 9, 2,
      7, 11, 4, 1, 9, 12, 14, 2, 0, 6, 10, 13, 15, 3, 5, 8,
      2, 1, 14, 7, 4, 10, 8, 13, 15, 12, 9, 0, 3, 5, 6, 11 },
};

/* Bit n (1-based, from the MSB) of a w-bit value, moved to position i. */
static uint64_t des_permute(uint64_t in, int w, const uint8_t *tab, int n)
{
    uint64_t out = 0;
    int i;

    for (i = 0; i < n; i++) {
        out = (out << 1) | ((in >> (w - tab[i])) & 1);
    }
    return out;
}

typedef struct {
    uint64_t sub[16];           /* 48-bit round keys */
} DesKey;

static void des_set_key(DesKey *k, const uint8_t *key)
{
    uint64_t cd = des_permute(ldq_be_p(key), 64, des_pc1, 56);
    uint32_t c = cd >> 28, d = cd & 0xFFFFFFF;
    int i, j;

    for (i = 0; i < 16; i++) {
        for (j = 0; j < des_shifts[i]; j++) {
            c = ((c << 1) | (c >> 27)) & 0xFFFFFFF;
            d = ((d << 1) | (d >> 27)) & 0xFFFFFFF;
        }
        k->sub[i] = des_permute(((uint64_t)c << 28) | d, 56, des_pc2, 48);
    }
}

static uint32_t des_f(uint32_t r, uint64_t sub)
{
    uint64_t x = des_permute(r, 32, des_e, 48) ^ sub;
    uint32_t s = 0;
    int i, b;

    for (i = 0; i < 8; i++) {
        b = (x >> (42 - 6 * i)) & 0x3F;
        s = (s << 4) | des_sbox[i][((b & 0x20) | ((b & 1) << 4)) |
                                   ((b >> 1) & 0xF)];
    }
    return des_permute(s, 32, des_p, 32);
}

static void des_block(const DesKey *k, bool decrypt, const uint8_t *in,
                      uint8_t *out)
{
    uint64_t x = des_permute(ldq_be_p(in), 64, des_ip, 64);
    uint32_t l = x >> 32, r = x, t;
    int i;

    for (i = 0; i < 16; i++) {
        t = r;
        r = l ^ des_f(r, k->sub[decrypt ? 15 - i : i]);
        l = t;
    }
    stq_be_p(out, des_permute(((uint64_t)r << 32) | l, 64, des_fp, 64));
}

/* the block cipher a request runs */

typedef struct {
    int cipher;
    int bsize;
    AES_KEY aes_enc, aes_dec;
    DesKey des[3];
} NvtCipher;

static void nvt_cipher_init(NvtCipher *c, int cipher, const uint8_t *key)
{
    c->cipher = cipher;
    switch (cipher) {
    case CIPHER_DES:
        c->bsize = 8;
        des_set_key(&c->des[0], key);
        break;
    case CIPHER_3DES:
        c->bsize = 8;
        des_set_key(&c->des[0], key);
        des_set_key(&c->des[1], key + 8);
        des_set_key(&c->des[2], key + 16);
        break;
    default:
        c->bsize = 16;
        AES_set_encrypt_key(key, cipher == CIPHER_AES256 ? 256 : 128,
                            &c->aes_enc);
        AES_set_decrypt_key(key, cipher == CIPHER_AES256 ? 256 : 128,
                            &c->aes_dec);
        break;
    }
}

static void nvt_cipher_block(const NvtCipher *c, bool decrypt,
                             const uint8_t *in, uint8_t *out)
{
    uint8_t t[8];

    switch (c->cipher) {
    case CIPHER_DES:
        des_block(&c->des[0], decrypt, in, out);
        break;
    case CIPHER_3DES:
        /* EDE: E(K3, D(K2, E(K1, x))), and its inverse */
        if (decrypt) {
            des_block(&c->des[2], true, in, t);
            des_block(&c->des[1], false, t, t);
            des_block(&c->des[0], true, t, out);
        } else {
            des_block(&c->des[0], false, in, t);
            des_block(&c->des[1], true, t, t);
            des_block(&c->des[2], false, t, out);
        }
        break;
    default:
        if (decrypt) {
            AES_decrypt(in, out, &c->aes_dec);
        } else {
            AES_encrypt(in, out, &c->aes_enc);
        }
        break;
    }
}

/* GHASH */

typedef struct {
    uint8_t h[16];
    uint8_t y[16];
    uint8_t buf[16];
    int fill;
} NvtGhash;

/* y = (y ^ x) * h in GF(2^128), NIST SP 800-38D algorithm 1. */
static void ghash_block(NvtGhash *g, const uint8_t *x)
{
    uint64_t zh = 0, zl = 0, vh, vl, yh, yl;
    int i;

    yh = ldq_be_p(g->y) ^ ldq_be_p(x);
    yl = ldq_be_p(g->y + 8) ^ ldq_be_p(x + 8);
    vh = ldq_be_p(g->h);
    vl = ldq_be_p(g->h + 8);
    for (i = 0; i < 128; i++) {
        if ((i < 64 ? yh >> (63 - i) : yl >> (127 - i)) & 1) {
            zh ^= vh;
            zl ^= vl;
        }
        if (vl & 1) {
            vl = (vl >> 1) | (vh << 63);
            vh = (vh >> 1) ^ 0xE100000000000000ULL;
        } else {
            vl = (vl >> 1) | (vh << 63);
            vh >>= 1;
        }
    }
    stq_be_p(g->y, zh);
    stq_be_p(g->y + 8, zl);
}

static void ghash_update(NvtGhash *g, const uint8_t *p, size_t n)
{
    while (n--) {
        g->buf[g->fill++] = *p++;
        if (g->fill == 16) {
            ghash_block(g, g->buf);
            g->fill = 0;
        }
    }
}

/* Pad what is left to a block: GCM pads A and C separately. */
static void ghash_flush(NvtGhash *g)
{
    if (g->fill) {
        memset(g->buf + g->fill, 0, 16 - g->fill);
        ghash_block(g, g->buf);
        g->fill = 0;
    }
}

/* one DMA request */

typedef struct {
    NvtCipher c;
    int op;
    bool decrypt;
    uint8_t iv[16];             /* chaining value / counter block */
    uint8_t step[16];           /* CTR increment */
    uint8_t ks[16];             /* keystream block of the stream modes */
    uint8_t fb[16];             /* CFB: the cipher block being collected */
    int used;                   /* bytes of ks consumed; bsize = none */
    NvtGhash gh;
    uint64_t aad_len, text_len;
} NvtCryptoReq;

static void xor_bytes(uint8_t *d, const uint8_t *x, int n)
{
    while (n--) {
        *d++ ^= *x++;
    }
}

/* counter += step, big-endian over the last 'width' bytes */
static void ctr_add(uint8_t *ctr, const uint8_t *step, int bsize, int width)
{
    unsigned carry = 0;
    int i;

    for (i = bsize - 1; i >= bsize - width; i--) {
        carry += ctr[i] + step[i];
        ctr[i] = carry;
        carry >>= 8;
    }
}

static const uint8_t ctr_one[16] = { [15] = 1 };

/* The next keystream block of CTR, OFB, CFB or GCM. */
static void req_next_ks(NvtCryptoReq *r)
{
    int b = r->c.bsize;

    switch (r->op) {
    case OP_CTR:
        nvt_cipher_block(&r->c, false, r->iv, r->ks);
        ctr_add(r->iv, r->step, b, b);
        break;
    case OP_GCM:
        ctr_add(r->iv, ctr_one, 16, 4);
        nvt_cipher_block(&r->c, false, r->iv, r->ks);
        break;
    case OP_OFB:
        nvt_cipher_block(&r->c, false, r->iv, r->ks);
        memcpy(r->iv, r->ks, b);
        break;
    case OP_CFB:
        nvt_cipher_block(&r->c, false, r->iv, r->ks);
        break;
    }
    r->used = 0;
}

/* Transform n bytes in place. */
static void req_process(NvtCryptoReq *r, uint8_t *p, size_t n)
{
    int b = r->c.bsize;
    uint8_t in[16];
    size_t i;

    switch (r->op) {
    case OP_ECB:
    case OP_CBC:
        /* the driver keeps these block-aligned; a tail is left alone */
        for (i = 0; i + b <= n; i += b) {
            memcpy(in, p + i, b);
            if (r->op == OP_CBC && !r->decrypt) {
                xor_bytes(p + i, r->iv, b);
            }
            nvt_cipher_block(&r->c, r->decrypt, p + i, p + i);
            if (r->op == OP_CBC) {
                if (r->decrypt) {
                    xor_bytes(p + i, r->iv, b);
                    memcpy(r->iv, in, b);
                } else {
                    memcpy(r->iv, p + i, b);
                }
            }
        }
        break;
    default:
        if (r->op == OP_GCM && r->decrypt) {
            ghash_update(&r->gh, p, n);
        }
        for (i = 0; i < n; i++) {
            if (r->used == b) {
                req_next_ks(r);
            }
            if (r->op == OP_CFB && r->decrypt) {
                r->fb[r->used] = p[i];
            }
            p[i] ^= r->ks[r->used];
            if (r->op == OP_CFB && !r->decrypt) {
                r->fb[r->used] = p[i];
            }
            if (++r->used == b && r->op == OP_CFB) {
                memcpy(r->iv, r->fb, b);     /* the next block's feedback */
            }
        }
        if (r->op == OP_GCM && !r->decrypt) {
            ghash_update(&r->gh, p, n);
        }
        r->text_len += n;
        break;
    }
}

/* The chaining value the descriptor's CV gets. */
static void req_cv(NvtCryptoReq *r, uint8_t *cv)
{
    memset(cv, 0, 16);
    memcpy(cv, r->iv, r->c.bsize);
}

static bool nvt_crypto_dma(NvtCryptoState *s, hwaddr desc)
{
    AddressSpace *as = &address_space_memory;
    const MemTxAttrs attrs = MEMTXATTRS_UNSPECIFIED;
    NvtCryptoReq *r = g_new0(NvtCryptoReq, 1);
    uint8_t key[32] = { 0 }, s0[16] = { 0 }, cv[16], zero[16] = { 0 };
    uint8_t step[16];
    uint32_t hdr, blk[DESC_NBLOCKS][4];
    uint8_t *buf = NULL;
    size_t buf_size = 0;
    bool aad;
    int cipher, i, group_end;
    bool ok = false;

    if (dma_memory_read(as, desc + DESC_HEADER, &hdr, 4, attrs) ||
        dma_memory_read(as, desc + DESC_BLOCKS, blk, sizeof(blk), attrs)) {
        goto out;
    }
    hdr = le32_to_cpu(hdr);
    cipher = (hdr >> 4) & 0xF;
    r->op = (hdr >> 8) & 0xF;
    r->decrypt = hdr & HDR_DECRYPT;
    if (cipher > CIPHER_AES256 || r->op > OP_GCM ||
        (r->op == OP_GCM && cipher < CIPHER_AES128)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "nvt-crypto: cipher %d mode %d\n", cipher, r->op);
        goto out;
    }
    if (!(hdr & HDR_KEY_MANAGER) &&
        dma_memory_read(as, desc + DESC_KEY, key, sizeof(key), attrs)) {
        goto out;
    }
    nvt_cipher_init(&r->c, cipher, key);
    if (dma_memory_read(as, desc + DESC_IV, r->iv, 16, attrs) ||
        dma_memory_read(as, desc + DESC_COUNTER, step, 16, attrs)) {
        goto out;
    }
    /* the step is numeric words, most significant first */
    for (i = 0; i < r->c.bsize / 4; i++) {
        stl_be_p(r->step + 4 * i, ldl_le_p(step + 4 * i));
    }
    r->used = r->c.bsize;

    if (r->op == OP_GCM) {
        /* J0 = IV || 0^31 || 1; H = E(K, 0); S0 = E(K, J0) */
        stl_be_p(r->iv + 12, 1);
        nvt_cipher_block(&r->c, false, zero, r->gh.h);
        nvt_cipher_block(&r->c, false, r->iv, s0);
    } else if (r->op == OP_CTR && (hdr & HDR_GET_S0)) {
        nvt_cipher_block(&r->c, false, r->iv, s0);
        ctr_add(r->iv, r->step, r->c.bsize, r->c.bsize);
    }

    /* GCM: the associated data, up to the first group end, then the text */
    aad = r->op == OP_GCM;
    group_end = 0;
    for (i = 0; i < DESC_NBLOCKS; i++) {
        uint32_t src = le32_to_cpu(blk[i][0]), dst = le32_to_cpu(blk[i][1]);
        uint32_t len = le32_to_cpu(blk[i][2]), cfg = le32_to_cpu(blk[i][3]);
        uint32_t done, n;

        for (done = 0; done < len; done += n) {
            n = MIN(len - done, 64 * KiB);
            if (n > buf_size) {
                buf = g_realloc(buf, n);
                buf_size = n;
            }
            if (dma_memory_read(as, src + done, buf, n, attrs)) {
                goto out;
            }
            if (aad) {
                ghash_update(&r->gh, buf, n);
                r->aad_len += n;
                continue;
            }
            req_process(r, buf, n);
            if (!(cfg & BLK_NO_WRITE) &&
                dma_memory_write(as, dst + done, buf, n, attrs)) {
                goto out;
            }
        }
        if (cfg & BLK_LAST) {
            if (aad) {
                ghash_flush(&r->gh);
                aad = false;
                continue;
            }
            group_end = 1;
            break;
        }
    }
    if (!group_end) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "nvt-crypto: descriptor without a last block\n");
    }

    if (r->op == OP_GCM) {
        uint8_t lens[16];

        ghash_flush(&r->gh);
        stq_be_p(lens, r->aad_len * 8);
        stq_be_p(lens + 8, r->text_len * 8);
        ghash_update(&r->gh, lens, 16);
        if (dma_memory_write(as, desc + DESC_GHASH, r->gh.y, 16, attrs)) {
            goto out;
        }
    }
    req_cv(r, cv);
    ok = !dma_memory_write(as, desc + DESC_CV, cv, 16, attrs) &&
         !dma_memory_write(as, desc + DESC_S0, s0, 16, attrs);
out:
    g_free(buf);
    g_free(r);
    return ok;
}

/* registers */

static uint32_t *reg(NvtCryptoState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_crypto_update_irq(NvtCryptoState *s)
{
    qemu_set_irq(s->irq, !!(*reg(s, CR_INT_STS) & *reg(s, CR_INT_ENB)));
}

/* One ECB block from KEY and IN to OUT, as CFG says. */
static void nvt_crypto_pio(NvtCryptoState *s)
{
    uint32_t cfg = *reg(s, CR_CFG);
    int cipher = (cfg >> 4) & 0x7;
    uint8_t key[32], in[16], out[16] = { 0 };
    NvtCipher *c;
    int i;

    if (cipher > CIPHER_AES256) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-crypto: PIO cipher %d\n", cipher);
        return;
    }
    for (i = 0; i < 8; i++) {
        stl_le_p(key + 4 * i, *reg(s, CR_KEY + 4 * i));
    }
    for (i = 0; i < 4; i++) {
        stl_le_p(in + 4 * i, *reg(s, CR_IN + 4 * i));
    }
    c = g_new(NvtCipher, 1);
    nvt_cipher_init(c, cipher, key);
    nvt_cipher_block(c, cfg & (1u << 8), in, out);
    g_free(c);
    for (i = 0; i < 4; i++) {
        *reg(s, CR_OUT + 4 * i) = ldl_le_p(out + 4 * i);
    }
}

static void nvt_crypto_ctrl(NvtCryptoState *s, uint32_t val)
{
    int ch;

    if (val & CTRL_PIO) {
        nvt_crypto_pio(s);
        *reg(s, CR_INT_STS) |= 1;
    }
    for (ch = 0; ch < CR_DMA_CH; ch++) {
        if (!(val & (1u << (4 + ch)))) {
            continue;
        }
        if (!nvt_crypto_dma(s, *reg(s, CR_DMA_ADDR + 4 * ch))) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "nvt-crypto: DMA channel %d failed\n", ch);
        }
        /* Done either way: the driver's timeout is three seconds. */
        *reg(s, CR_INT_STS) |= 1u << (4 + ch);
    }
    /* Every request completes at once: CTRL never reads busy. */
    *reg(s, CR_CTRL) = val & ~(CTRL_PIO | 0xF0);
}

static uint64_t nvt_crypto_read(void *opaque, hwaddr off, unsigned size)
{
    NvtCryptoState *s = opaque;

    return *reg(s, off);
}

static void nvt_crypto_reset_regs(NvtCryptoState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
}

static void nvt_crypto_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    NvtCryptoState *s = opaque;

    switch (off) {
    case CR_CFG:
        if (val & CFG_RESET) {
            /* the reset bit clears itself at once */
            nvt_crypto_reset_regs(s);
        } else {
            *reg(s, off) = val;
        }
        break;
    case CR_CTRL:
        nvt_crypto_ctrl(s, val);
        break;
    case CR_INT_STS:
        *reg(s, off) &= ~val;
        break;
    case CR_OUT ... CR_OUT + 0xC:
        break;                          /* read only */
    default:
        *reg(s, off) = val;
        break;
    }
    nvt_crypto_update_irq(s);
}

static const MemoryRegionOps nvt_crypto_ops = {
    .read = nvt_crypto_read,
    .write = nvt_crypto_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_crypto_reset(DeviceState *dev)
{
    NvtCryptoState *s = NVT_CRYPTO(dev);

    nvt_crypto_reset_regs(s);
    qemu_irq_lower(s->irq);
}

static void nvt_crypto_init(Object *obj)
{
    NvtCryptoState *s = NVT_CRYPTO(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_crypto_ops, s,
                          TYPE_NVT_CRYPTO, CR_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_nvt_crypto = {
    .name = TYPE_NVT_CRYPTO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtCryptoState, CR_REGS_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_crypto_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_crypto_reset);
    dc->vmsd = &vmstate_nvt_crypto;
}

static const TypeInfo nvt_crypto_info = {
    .name = TYPE_NVT_CRYPTO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtCryptoState),
    .instance_init = nvt_crypto_init,
    .class_init = nvt_crypto_class_init,
};

static void nvt_crypto_register_types(void)
{
    type_register_static(&nvt_crypto_info);
}

type_init(nvt_crypto_register_types)
