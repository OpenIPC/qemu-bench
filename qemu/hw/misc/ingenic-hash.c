/*
 * Ingenic HASH: the MD5 / SHA engine at 0x13480000 (T21, T23, T31, T40,
 * T41; INTC source 22).
 *
 * As the SDKs' sample_hash (jz-hash.c) and the 4.4 kernel's crypto
 * driver (drivers/crypto/ingenic-hash.c) use it:
 *
 *   0x00 HSCR   bit 0 enable, 3:1 the algorithm (MD5, SHA-1, SHA-224,
 *               SHA-256, SHA-384, SHA-512), 4 load the initial state,
 *               5 DMA enable, 6 DMA start, 8:7 byte order (the drivers
 *               set what each algorithm needs; nothing to do here);
 *               4 and 6 are commands and read back 0
 *   0x04 HSSR   bit 0 a round done, 1 all rounds done; write 1 clears
 *   0x08 HSINTM bit n enables HSSR bit n's interrupt
 *   0x0C HSSA   DMA source (physical)
 *   0x10 HSTC   DMA length in 64-byte units
 *   0x14 HSDI   data by the CPU (below)
 *   0x18 HSDO   the state, a word per read
 *   0x1C HSCG   clock gate, storage
 *
 * The engine hashes blocks the drivers have already padded, carrying the
 * state from one DMA run to the next until the state is loaded again.
 * Each HSDO read gives the next four bytes of the digest as a little-
 * endian word: the 4.4 driver stores the words as they come and hands
 * the bytes to the crypto API, whose self-tests compare them with the
 * standard digests.
 *
 * CPU mode (HSDI) is inferred: no driver, sample or document here feeds
 * HSDI (the 4.4 driver defines HASH_FLAGS_CPU and never uses it).  It
 * is modelled as the DMA path fed by the CPU: with HSCR enabled and DMA
 * off, each HSDI word is four message bytes in memory order (what a
 * load from the DMA's buffer gives), and every 64 bytes (128 for SHA-384
 * and SHA-512) run a block: HSSR bit 0 a round done, and after HSTC
 * blocks (when HSTC is non-zero) bit 1 all rounds done, as at the end of
 * a DMA run.  Loading the state (HSCR bit 4) empties a part-filled
 * block.  HSCR bits 8:7 (byte order) are not acted on in either mode:
 * the 4.4 driver sets 3 for SHA and 0 for MD5, sample_hash sets 1 (bit
 * 7) for SHA and 2 (bit 8) for MD5, and both expect the standard digest,
 * so their meaning cannot be told from these sources.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "system/dma.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_HASH "ingenic-hash"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicHashState, INGENIC_HASH)

#define R_HSCR          0x00
#define R_HSSR          0x04
#define R_HSINTM        0x08
#define R_HSSA          0x0C
#define R_HSTC          0x10
#define R_HSDI          0x14
#define R_HSDO          0x18
#define R_HSCG          0x1C

#define HSCR_EN         (1u << 0)
#define HSCR_INIT       (1u << 4)
#define HSCR_DMAE       (1u << 5)
#define HSCR_DMAS       (1u << 6)
#define HSCR_CMDS       (HSCR_INIT | HSCR_DMAS)

#define HSSR_ORD        (1u << 0)
#define HSSR_MRD        (1u << 1)

enum { ALG_MD5, ALG_SHA1, ALG_SHA224, ALG_SHA256, ALG_SHA384, ALG_SHA512 };

struct IngenicHashState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t hscr, hssr, hsintm, hssa, hstc, hscg;
    uint64_t h[8];                  /* the state (32-bit words in h[0..7]
                                       for MD5 / SHA-1 / SHA-256) */
    uint32_t nout;
    uint8_t cpu_blk[128];           /* CPU mode: the block being filled */
    uint32_t cpu_fill, cpu_blocks;
};

/* --- the block functions ------------------------------------------------- */

static void md5_block(uint64_t *h, const uint8_t *p)
{
    static const uint32_t k[64] = {
        0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf,
        0x4787c62a, 0xa8304613, 0xfd469501, 0x698098d8, 0x8b44f7af,
        0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e,
        0x49b40821, 0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa,
        0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8, 0x21e1cde6,
        0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8,
        0x676f02d9, 0x8d2a4c8a, 0xfffa3942, 0x8771f681, 0x6d9d6122,
        0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
        0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039,
        0xe6db99e5, 0x1fa27cf8, 0xc4ac5665, 0xf4292244, 0x432aff97,
        0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d,
        0x85845dd1, 0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
        0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391,
    };
    static const uint8_t r[64] = {
        7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
        5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
        4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
        6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
    };
    uint32_t m[16], a = h[0], b = h[1], c = h[2], d = h[3];
    int i;

    for (i = 0; i < 16; i++) {
        m[i] = ldl_le_p(p + 4 * i);
    }
    for (i = 0; i < 64; i++) {
        uint32_t f, g, t;

        if (i < 16) {
            f = (b & c) | (~b & d);
            g = i;
        } else if (i < 32) {
            f = (d & b) | (~d & c);
            g = (5 * i + 1) & 15;
        } else if (i < 48) {
            f = b ^ c ^ d;
            g = (3 * i + 5) & 15;
        } else {
            f = c ^ (b | ~d);
            g = (7 * i) & 15;
        }
        t = d;
        d = c;
        c = b;
        b = b + rol32(a + f + k[i] + m[g], r[i]);
        a = t;
    }
    h[0] = (uint32_t)(h[0] + a);
    h[1] = (uint32_t)(h[1] + b);
    h[2] = (uint32_t)(h[2] + c);
    h[3] = (uint32_t)(h[3] + d);
}

static void sha1_block(uint64_t *h, const uint8_t *p)
{
    uint32_t w[80], a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    int i;

    for (i = 0; i < 16; i++) {
        w[i] = ldl_be_p(p + 4 * i);
    }
    for (; i < 80; i++) {
        w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }
    for (i = 0; i < 80; i++) {
        uint32_t f, k, t;

        if (i < 20) {
            f = (b & c) | (~b & d);
            k = 0x5a827999;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ed9eba1;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8f1bbcdc;
        } else {
            f = b ^ c ^ d;
            k = 0xca62c1d6;
        }
        t = rol32(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = rol32(b, 30);
        b = a;
        a = t;
    }
    h[0] = (uint32_t)(h[0] + a);
    h[1] = (uint32_t)(h[1] + b);
    h[2] = (uint32_t)(h[2] + c);
    h[3] = (uint32_t)(h[3] + d);
    h[4] = (uint32_t)(h[4] + e);
}

static void sha256_block(uint64_t *h, const uint8_t *p)
{
    static const uint32_t k[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b,
        0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01,
        0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7,
        0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
        0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152,
        0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
        0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819,
        0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08,
        0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f,
        0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
        0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
    };
    uint32_t w[64], v[8];
    int i;

    for (i = 0; i < 16; i++) {
        w[i] = ldl_be_p(p + 4 * i);
    }
    for (; i < 64; i++) {
        uint32_t s0 = ror32(w[i - 15], 7) ^ ror32(w[i - 15], 18) ^
                      (w[i - 15] >> 3);
        uint32_t s1 = ror32(w[i - 2], 17) ^ ror32(w[i - 2], 19) ^
                      (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    for (i = 0; i < 8; i++) {
        v[i] = h[i];
    }
    for (i = 0; i < 64; i++) {
        uint32_t s1 = ror32(v[4], 6) ^ ror32(v[4], 11) ^ ror32(v[4], 25);
        uint32_t ch = (v[4] & v[5]) ^ (~v[4] & v[6]);
        uint32_t t1 = v[7] + s1 + ch + k[i] + w[i];
        uint32_t s0 = ror32(v[0], 2) ^ ror32(v[0], 13) ^ ror32(v[0], 22);
        uint32_t mj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);

        memmove(v + 1, v, 7 * sizeof(v[0]));
        v[4] += t1;
        v[0] = t1 + s0 + mj;
    }
    for (i = 0; i < 8; i++) {
        h[i] = (uint32_t)(h[i] + v[i]);
    }
}

static void sha512_block(uint64_t *h, const uint8_t *p)
{
    static const uint64_t k[80] = {
        0x428a2f98d728ae22ULL, 0x7137449123ef65cdULL, 0xb5c0fbcfec4d3b2fULL,
        0xe9b5dba58189dbbcULL, 0x3956c25bf348b538ULL, 0x59f111f1b605d019ULL,
        0x923f82a4af194f9bULL, 0xab1c5ed5da6d8118ULL, 0xd807aa98a3030242ULL,
        0x12835b0145706fbeULL, 0x243185be4ee4b28cULL, 0x550c7dc3d5ffb4e2ULL,
        0x72be5d74f27b896fULL, 0x80deb1fe3b1696b1ULL, 0x9bdc06a725c71235ULL,
        0xc19bf174cf692694ULL, 0xe49b69c19ef14ad2ULL, 0xefbe4786384f25e3ULL,
        0x0fc19dc68b8cd5b5ULL, 0x240ca1cc77ac9c65ULL, 0x2de92c6f592b0275ULL,
        0x4a7484aa6ea6e483ULL, 0x5cb0a9dcbd41fbd4ULL, 0x76f988da831153b5ULL,
        0x983e5152ee66dfabULL, 0xa831c66d2db43210ULL, 0xb00327c898fb213fULL,
        0xbf597fc7beef0ee4ULL, 0xc6e00bf33da88fc2ULL, 0xd5a79147930aa725ULL,
        0x06ca6351e003826fULL, 0x142929670a0e6e70ULL, 0x27b70a8546d22ffcULL,
        0x2e1b21385c26c926ULL, 0x4d2c6dfc5ac42aedULL, 0x53380d139d95b3dfULL,
        0x650a73548baf63deULL, 0x766a0abb3c77b2a8ULL, 0x81c2c92e47edaee6ULL,
        0x92722c851482353bULL, 0xa2bfe8a14cf10364ULL, 0xa81a664bbc423001ULL,
        0xc24b8b70d0f89791ULL, 0xc76c51a30654be30ULL, 0xd192e819d6ef5218ULL,
        0xd69906245565a910ULL, 0xf40e35855771202aULL, 0x106aa07032bbd1b8ULL,
        0x19a4c116b8d2d0c8ULL, 0x1e376c085141ab53ULL, 0x2748774cdf8eeb99ULL,
        0x34b0bcb5e19b48a8ULL, 0x391c0cb3c5c95a63ULL, 0x4ed8aa4ae3418acbULL,
        0x5b9cca4f7763e373ULL, 0x682e6ff3d6b2b8a3ULL, 0x748f82ee5defb2fcULL,
        0x78a5636f43172f60ULL, 0x84c87814a1f0ab72ULL, 0x8cc702081a6439ecULL,
        0x90befffa23631e28ULL, 0xa4506cebde82bde9ULL, 0xbef9a3f7b2c67915ULL,
        0xc67178f2e372532bULL, 0xca273eceea26619cULL, 0xd186b8c721c0c207ULL,
        0xeada7dd6cde0eb1eULL, 0xf57d4f7fee6ed178ULL, 0x06f067aa72176fbaULL,
        0x0a637dc5a2c898a6ULL, 0x113f9804bef90daeULL, 0x1b710b35131c471bULL,
        0x28db77f523047d84ULL, 0x32caab7b40c72493ULL, 0x3c9ebe0a15c9bebcULL,
        0x431d67c49c100d4cULL, 0x4cc5d4becb3e42b6ULL, 0x597f299cfc657e2aULL,
        0x5fcb6fab3ad6faecULL, 0x6c44198c4a475817ULL,
    };
    uint64_t w[80], v[8];
    int i;

    for (i = 0; i < 16; i++) {
        w[i] = ldq_be_p(p + 8 * i);
    }
    for (; i < 80; i++) {
        uint64_t s0 = ror64(w[i - 15], 1) ^ ror64(w[i - 15], 8) ^
                      (w[i - 15] >> 7);
        uint64_t s1 = ror64(w[i - 2], 19) ^ ror64(w[i - 2], 61) ^
                      (w[i - 2] >> 6);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    memcpy(v, h, sizeof(v));
    for (i = 0; i < 80; i++) {
        uint64_t s1 = ror64(v[4], 14) ^ ror64(v[4], 18) ^ ror64(v[4], 41);
        uint64_t ch = (v[4] & v[5]) ^ (~v[4] & v[6]);
        uint64_t t1 = v[7] + s1 + ch + k[i] + w[i];
        uint64_t s0 = ror64(v[0], 28) ^ ror64(v[0], 34) ^ ror64(v[0], 39);
        uint64_t mj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);

        memmove(v + 1, v, 7 * sizeof(v[0]));
        v[4] += t1;
        v[0] = t1 + s0 + mj;
    }
    for (i = 0; i < 8; i++) {
        h[i] += v[i];
    }
}

static const uint32_t iv_md5[4] = {
    0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476,
};
static const uint32_t iv_sha1[5] = {
    0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0,
};
static const uint32_t iv_sha224[8] = {
    0xc1059ed8, 0x367cd507, 0x3070dd17, 0xf70e5939,
    0xffc00b31, 0x68581511, 0x64f98fa7, 0xbefa4fa4,
};
static const uint32_t iv_sha256[8] = {
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
};
static const uint64_t iv_sha384[8] = {
    0xcbbb9d5dc1059ed8ULL, 0x629a292a367cd507ULL, 0x9159015a3070dd17ULL,
    0x152fecd8f70e5939ULL, 0x67332667ffc00b31ULL, 0x8eb44a8768581511ULL,
    0xdb0c2e0d64f98fa7ULL, 0x47b5481dbefa4fa4ULL,
};
static const uint64_t iv_sha512[8] = {
    0x6a09e667f3bcc908ULL, 0xbb67ae8584caa73bULL, 0x3c6ef372fe94f82bULL,
    0xa54ff53a5f1d36f1ULL, 0x510e527fade682d1ULL, 0x9b05688c2b3e6c1fULL,
    0x1f83d9abfb41bd6bULL, 0x5be0cd19137e2179ULL,
};

static int hash_alg(IngenicHashState *s)
{
    return MIN(extract32(s->hscr, 1, 3), ALG_SHA512);
}

/* --- the device ------------------------------------------------------------ */

static void hash_update(IngenicHashState *s)
{
    qemu_set_irq(s->irq, (s->hssr & s->hsintm & 3) != 0);
}

static void hash_init_state(IngenicHashState *s)
{
    int i;

    memset(s->h, 0, sizeof(s->h));
    for (i = 0; i < 8; i++) {
        switch (hash_alg(s)) {
        case ALG_MD5:
            s->h[i] = i < 4 ? iv_md5[i] : 0;
            break;
        case ALG_SHA1:
            s->h[i] = i < 5 ? iv_sha1[i] : 0;
            break;
        case ALG_SHA224:
            s->h[i] = iv_sha224[i];
            break;
        case ALG_SHA256:
            s->h[i] = iv_sha256[i];
            break;
        case ALG_SHA384:
            s->h[i] = iv_sha384[i];
            break;
        default:
            s->h[i] = iv_sha512[i];
            break;
        }
    }
    s->nout = 0;
    s->cpu_fill = s->cpu_blocks = 0;
}

static void hash_block(IngenicHashState *s, const uint8_t *blk)
{
    switch (hash_alg(s)) {
    case ALG_MD5:
        md5_block(s->h, blk);
        break;
    case ALG_SHA1:
        sha1_block(s->h, blk);
        break;
    case ALG_SHA224:
    case ALG_SHA256:
        sha256_block(s->h, blk);
        break;
    default:
        sha512_block(s->h, blk);
        break;
    }
}

/* CPU mode (inferred, see the top): one word of the message by HSDI */
static void hash_cpu_word(IngenicHashState *s, uint32_t val)
{
    int bs = hash_alg(s) >= ALG_SHA384 ? 128 : 64;

    if ((s->hscr & (HSCR_EN | HSCR_DMAE)) != HSCR_EN) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-hash: HSDI with the engine "
                      "off or in DMA mode\n");
        return;
    }
    stl_le_p(s->cpu_blk + s->cpu_fill, val);
    s->cpu_fill += 4;
    if (s->cpu_fill < bs) {
        return;
    }
    hash_block(s, s->cpu_blk);
    s->cpu_fill = 0;
    s->nout = 0;
    s->hssr |= HSSR_ORD;
    if (s->hstc && ++s->cpu_blocks * (bs / 64) >= s->hstc) {
        s->hssr |= HSSR_MRD;
        s->cpu_blocks = 0;
    }
}

static void hash_dma(IngenicHashState *s)
{
    int alg = hash_alg(s), bs = alg >= ALG_SHA384 ? 128 : 64;
    uint32_t len = s->hstc * 64, off;
    uint8_t blk[128];

    for (off = 0; off + bs <= len; off += bs) {
        if (dma_memory_read(&address_space_memory, s->hssa + off, blk, bs,
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-hash: bad source 0x%x\n",
                          s->hssa + off);
            break;
        }
        hash_block(s, blk);
    }
    s->nout = 0;
    s->hssr |= HSSR_ORD | HSSR_MRD;
}

/* the digest's bytes 4 n .. 4 n + 3, as a little-endian word */
static uint32_t hash_out(IngenicHashState *s, uint32_t n)
{
    int alg = hash_alg(s);
    uint8_t b[4];

    switch (alg) {
    case ALG_MD5:
        return n < 4 ? s->h[n] : 0;
    case ALG_SHA1:
    case ALG_SHA224:
    case ALG_SHA256:
        stl_be_p(b, n < 8 ? s->h[n] : 0);
        break;
    default:
        stl_be_p(b, n < 16 ? s->h[n / 2] >> (n & 1 ? 0 : 32) : 0);
        break;
    }
    return ldl_le_p(b);
}

static uint64_t ingenic_hash_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicHashState *s = opaque;

    switch (off) {
    case R_HSCR:
        return s->hscr;
    case R_HSSR:
        return s->hssr;
    case R_HSINTM:
        return s->hsintm;
    case R_HSSA:
        return s->hssa;
    case R_HSTC:
        return s->hstc;
    case R_HSDO:
        return hash_out(s, s->nout++);
    case R_HSCG:
        return s->hscg;
    }
    return 0;
}

static void ingenic_hash_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    IngenicHashState *s = opaque;

    switch (off) {
    case R_HSCR:
        s->hscr = val & ~HSCR_CMDS;
        if ((val & HSCR_INIT) && (s->hscr & HSCR_EN)) {
            hash_init_state(s);
        }
        if ((val & HSCR_DMAS) && (s->hscr & (HSCR_EN | HSCR_DMAE)) ==
                                 (HSCR_EN | HSCR_DMAE)) {
            hash_dma(s);
        }
        break;
    case R_HSSR:
        s->hssr &= ~val;
        break;
    case R_HSINTM:
        s->hsintm = val & 3;
        break;
    case R_HSSA:
        s->hssa = val;
        break;
    case R_HSTC:
        s->hstc = val;
        s->cpu_blocks = 0;
        break;
    case R_HSDI:
        hash_cpu_word(s, val);
        break;
    case R_HSCG:
        s->hscg = val;
        break;
    }
    hash_update(s);
}

static const MemoryRegionOps ingenic_hash_ops = {
    .read = ingenic_hash_read,
    .write = ingenic_hash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_hash_reset(DeviceState *dev)
{
    IngenicHashState *s = INGENIC_HASH(dev);

    s->hscr = s->hssr = s->hsintm = s->hssa = s->hstc = s->hscg = 0;
    memset(s->h, 0, sizeof(s->h));
    s->nout = 0;
    s->cpu_fill = s->cpu_blocks = 0;
    hash_update(s);
}

static void ingenic_hash_init(Object *obj)
{
    IngenicHashState *s = INGENIC_HASH(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_hash_ops, s,
                          TYPE_INGENIC_HASH, 0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_ingenic_hash = {
    .name = TYPE_INGENIC_HASH,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(hscr, IngenicHashState),
        VMSTATE_UINT32(hssr, IngenicHashState),
        VMSTATE_UINT32(hsintm, IngenicHashState),
        VMSTATE_UINT32(hssa, IngenicHashState),
        VMSTATE_UINT32(hstc, IngenicHashState),
        VMSTATE_UINT32(hscg, IngenicHashState),
        VMSTATE_UINT64_ARRAY(h, IngenicHashState, 8),
        VMSTATE_UINT32(nout, IngenicHashState),
        VMSTATE_UINT8_ARRAY(cpu_blk, IngenicHashState, 128),
        VMSTATE_UINT32(cpu_fill, IngenicHashState),
        VMSTATE_UINT32(cpu_blocks, IngenicHashState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_hash_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_hash_reset);
    dc->vmsd = &vmstate_ingenic_hash;
}

static const TypeInfo ingenic_hash_info = {
    .name = TYPE_INGENIC_HASH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicHashState),
    .instance_init = ingenic_hash_init,
    .class_init = ingenic_hash_class_init,
};

static void ingenic_hash_register_types(void)
{
    type_register_static(&ingenic_hash_info);
}

type_init(ingenic_hash_register_types)
