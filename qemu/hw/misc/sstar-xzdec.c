/*
 * SigmaStar XZ decoder (Iford): IPL_CUST and U-Boot unpack through it.
 *
 * Banks, from the vendor's drivers/sstar/xzdec/hal/iford/hal_xzdec.c:
 *
 *   0x11D7  control: 0x00 start, 0x01 soft reset, 0x12 IRQ clear,
 *           0x13 final and 0x14 raw status (bit n-1: n blocks done),
 *           0x1A/0x1B header and data check, two bits per block (3 = OK),
 *           0x1C [3:0] the magic check (0xF = OK)
 *   0x11D8  input SGDMA, 0x11DA dictionary SGDMA: only the buffers the
 *           real decoder keeps its input and dictionary in
 *   0x11DC  output SGDMA: each block's destination in regs 6/7 + 0x20 * n,
 *           a MIU address in units of 16 bytes
 *
 * BDMA destination port 6 delivers the file.  "SXZ" (the vendor's sz tool:
 * sstar_sz.sh, szsplit, szrestore) is one file of up to four blocks, one
 * per decoder core:
 *
 *   "sstarXZ", block count, chunk size (0x20), then each block's stored
 *   and unpacked size as XZ VLIs, padded to 16 bytes; then the blocks'
 *   bytes round-robin, a chunk of each in turn (a finished block drops
 *   out, and a chunk's short tail is padded).  A trailing "SZ" ends it.
 *
 * Each block is an XZ block: header, LZMA2 data, padding, CRC32 of the
 * unpacked data.  The LZMA is not the standard one: a literal is eight
 * direct bits of the range coder, with no context and no match byte
 * (szdec's lzma_literal), so lc and lp mean nothing and the decoder needs
 * no literal probabilities.  Matches and lengths are as usual.  The CRC32s
 * of all of the SDK's images check out.
 *
 * Plain XZ (control 0x07, its file size in 0x02/0x03, the output in block
 * 0's registers): an ordinary .xz file through the same path, one output.
 * No image in the SDK is one, so what the silicon does with it is unseen;
 * here its literals are the standard LZMA ones, lc and lp included, as any
 * xz tool writes them, and its CRC32 or CRC64 is checked.  Only the LZMA2
 * filter alone is taken.
 *
 * Decoding happens inside the BDMA write that delivers the last block.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "hw/irq.h"
#include "hw/misc/sstar-xzdec.h"
#include "migration/blocker.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include <zlib.h>

#define XZD_BANKS           6               /* 0x11D7 .. 0x11DC */
#define XZD_NREGS           (XZD_BANKS * 0x80)
#define OUT_BANK            ((0x11DC - 0x11D7) * 0x80)

#define R_CTRL0             0x00
#define R_SW_RST            0x01
#define R_XZ_SIZE_L         0x02
#define R_XZ_SIZE_H         0x03
#define R_IRQ_CLR           0x12
#define R_IRQ_FINAL         0x13
#define R_IRQ_RAW           0x14
#define R_HDR_CHECK         0x1A
#define R_DATA_CHECK        0x1B
#define R_MAGIC_CHECK       0x1C

#define CTRL_PLAIN_XZ       0x07

#define SXZ_MAX_BLOCKS      4
#define SXZ_MAX_FILE        (256 * MiB)
#define SXZ_MAX_UNPACKED    (512 * MiB)
#define DRAM_BASE           0x20000000

struct SstarXzdecState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Error *migration_blocker;
    uint16_t regs[XZD_NREGS];
    GByteArray *in;
    bool done;
};

/* ---- the decoder ---------------------------------------------------- */

#define PROB_INIT           1024
#define NUM_STATES          12
#define NUM_POS_STATES      16
#define MATCH_MIN           2
#define LIT_SIZE            0x300

typedef uint16_t Prob;

typedef struct {
    const uint8_t *in;
    size_t len, pos;
    uint32_t range, code;
    bool overrun;
} Rc;

typedef struct {
    Prob choice[2];
    Prob low[NUM_POS_STATES][8];
    Prob mid[NUM_POS_STATES][8];
    Prob high[256];
} Len;

typedef struct {
    Prob is_match[NUM_STATES][NUM_POS_STATES];
    Prob is_rep[NUM_STATES], is_g0[NUM_STATES];
    Prob is_g1[NUM_STATES], is_g2[NUM_STATES];
    Prob rep0_long[NUM_STATES][NUM_POS_STATES];
    Prob slot[4][64];
    Prob pos[128];
    Prob align[16];
    Len len, rep_len;
    Prob lit[LIT_SIZE << 4];                /* plain XZ only */
    /* not probabilities */
    unsigned state, pb, lc, lp;
    bool std;
    uint32_t rep[4];
} Lz;

static void rc_normalize(Rc *rc)
{
    if (rc->range < (1u << 24)) {
        rc->range <<= 8;
        if (rc->pos < rc->len) {
            rc->code = (rc->code << 8) | rc->in[rc->pos++];
        } else {
            rc->overrun = true;
        }
    }
}

static unsigned rc_bit(Rc *rc, Prob *p)
{
    uint32_t bound;

    rc_normalize(rc);
    bound = (rc->range >> 11) * *p;
    if (rc->code < bound) {
        rc->range = bound;
        *p += (2048 - *p) >> 5;
        return 0;
    }
    rc->range -= bound;
    rc->code -= bound;
    *p -= *p >> 5;
    return 1;
}

static uint32_t rc_direct(Rc *rc, unsigned bits)
{
    uint32_t v = 0;

    while (bits--) {
        rc_normalize(rc);
        rc->range >>= 1;
        rc->code -= rc->range;
        if (rc->code & 0x80000000) {
            rc->code += rc->range;
            v <<= 1;
        } else {
            v = (v << 1) | 1;
        }
    }
    return v;
}

static unsigned rc_tree(Rc *rc, Prob *p, unsigned bits)
{
    unsigned m = 1, i;

    for (i = 0; i < bits; i++) {
        m = (m << 1) | rc_bit(rc, &p[m]);
    }
    return m - (1u << bits);
}

static unsigned rc_rtree(Rc *rc, Prob *p, unsigned bits)
{
    unsigned m = 1, v = 0, i;

    for (i = 0; i < bits; i++) {
        unsigned b = rc_bit(rc, &p[m]);

        m = (m << 1) | b;
        v |= b << i;
    }
    return v;
}

static unsigned len_decode(Rc *rc, Len *l, unsigned ps)
{
    if (!rc_bit(rc, &l->choice[0])) {
        return rc_tree(rc, l->low[ps], 3);
    }
    if (!rc_bit(rc, &l->choice[1])) {
        return 8 + rc_tree(rc, l->mid[ps], 3);
    }
    return 16 + rc_tree(rc, l->high, 8);
}

static void lz_reset(Lz *z, unsigned pb)
{
    Prob *p = (Prob *)z;
    size_t i;

    for (i = 0; i < offsetof(Lz, state) / sizeof(Prob); i++) {
        p[i] = PROB_INIT;
    }
    z->state = 0;
    z->pb = pb;
    memset(z->rep, 0, sizeof(z->rep));
}

/* The standard LZMA literal, which SXZ does not use */
static uint8_t lz_literal(Lz *z, Rc *rc, const uint8_t *out, size_t o)
{
    unsigned prev = o ? out[o - 1] : 0;
    unsigned ctx = ((o & ((1u << z->lp) - 1)) << z->lc) + (prev >> (8 - z->lc));
    Prob *p = z->lit + LIT_SIZE * ctx;
    unsigned m = 1;

    if (z->state >= 7 && z->rep[0] < o) {
        unsigned mb = out[o - z->rep[0] - 1];

        while (m < 0x100) {
            unsigned mbit = (mb >> 7) & 1, b;

            mb <<= 1;
            b = rc_bit(rc, &p[((1 + mbit) << 8) + m]);
            m = (m << 1) | b;
            if (mbit != b) {
                break;
            }
        }
    }
    while (m < 0x100) {
        m = (m << 1) | rc_bit(rc, &p[m]);
    }
    return m;
}

/* One LZMA chunk: "unpack" bytes on to out at *opos.  False if corrupt. */
static bool lz_chunk(Lz *z, Rc *rc, uint8_t *out, size_t *opos, size_t end)
{
    size_t o = *opos;

    while (o < end) {
        unsigned ps = o & ((1u << z->pb) - 1);
        unsigned st = z->state;
        unsigned len;

        if (rc->overrun) {
            return false;
        }
        if (!rc_bit(rc, &z->is_match[st][ps])) {
            out[o] = z->std ? lz_literal(z, rc, out, o) : rc_direct(rc, 8);
            o++;
            z->state = st < 4 ? 0 : st < 10 ? st - 3 : st - 6;
            continue;
        }
        if (rc_bit(rc, &z->is_rep[st])) {
            if (!rc_bit(rc, &z->is_g0[st])) {
                if (!rc_bit(rc, &z->rep0_long[st][ps])) {
                    if (z->rep[0] >= o) {
                        return false;
                    }
                    z->state = st < 7 ? 9 : 11;
                    out[o] = out[o - z->rep[0] - 1];
                    o++;
                    continue;
                }
            } else {
                uint32_t d;

                if (!rc_bit(rc, &z->is_g1[st])) {
                    d = z->rep[1];
                } else {
                    if (!rc_bit(rc, &z->is_g2[st])) {
                        d = z->rep[2];
                    } else {
                        d = z->rep[3];
                        z->rep[3] = z->rep[2];
                    }
                    z->rep[2] = z->rep[1];
                }
                z->rep[1] = z->rep[0];
                z->rep[0] = d;
            }
            len = len_decode(rc, &z->rep_len, ps);
            z->state = st < 7 ? 8 : 11;
        } else {
            unsigned slot;
            uint32_t d;

            z->rep[3] = z->rep[2];
            z->rep[2] = z->rep[1];
            z->rep[1] = z->rep[0];
            len = len_decode(rc, &z->len, ps);
            z->state = st < 7 ? 7 : 10;
            slot = rc_tree(rc, z->slot[MIN(len, 3)], 6);
            if (slot < 4) {
                d = slot;
            } else {
                unsigned nb = (slot >> 1) - 1;

                d = (2 | (slot & 1)) << nb;
                if (slot < 14) {
                    /* the reverse tree of slot sits at pos[d - slot] */
                    d += rc_rtree(rc, &z->pos[d - slot], nb);
                } else {
                    d += rc_direct(rc, nb - 4) << 4;
                    d += rc_rtree(rc, z->align, 4);
                }
            }
            z->rep[0] = d;
        }
        if (z->rep[0] >= o) {
            return false;
        }
        len += MATCH_MIN;
        if (len > end - o) {
            return false;
        }
        while (len--) {
            out[o] = out[o - z->rep[0] - 1];
            o++;
        }
    }
    *opos = o;
    return !rc->overrun;
}

/* A block's LZMA2 data at in; *used is how much of it there was. */
static bool lzma2_decode(const uint8_t *in, size_t len, bool std, uint8_t *out,
                         size_t outmax, size_t *outlen, size_t *used)
{
    g_autofree Lz *z = g_new0(Lz, 1);
    size_t p = 0, o = 0;
    unsigned pb = 0, lc = 0, lp = 0;
    bool have_state = false;

    for (;;) {
        unsigned c;

        if (p >= len) {
            return false;
        }
        c = in[p++];
        if (c == 0) {
            *outlen = o;
            *used = p;
            return true;
        }
        if (c < 0x80) {                     /* stored */
            size_t n;

            if (c > 2 || len - p < 2) {
                return false;
            }
            n = lduw_be_p(in + p) + 1;
            p += 2;
            if (n > len - p || n > outmax - o) {
                return false;
            }
            memcpy(out + o, in + p, n);
            p += n;
            o += n;
        } else {
            size_t unpack, packed;
            Rc rc;

            if (len - p < 4) {
                return false;
            }
            unpack = (((c & 0x1F) << 16) | lduw_be_p(in + p)) + 1;
            packed = lduw_be_p(in + p + 2) + 1;
            p += 4;
            if (c >= 0xC0) {
                if (p >= len || in[p] >= 225) {
                    return false;
                }
                pb = in[p] / 45;
                lc = (in[p] % 45) % 9;
                lp = (in[p] % 45) / 9;
                p++;
                if (std && lc + lp > 4) {
                    return false;
                }
            }
            if (c >= 0xA0) {
                lz_reset(z, pb);
                z->lc = lc;
                z->lp = lp;
                z->std = std;
                have_state = true;
            }
            if (!have_state || packed > len - p || packed < 5 ||
                unpack > outmax - o) {
                return false;
            }
            rc = (Rc) {
                .in = in + p, .len = packed, .pos = 5,
                .range = 0xFFFFFFFF, .code = ldl_be_p(in + p + 1),
            };
            if (!lz_chunk(z, &rc, out, &o, o + unpack)) {
                return false;
            }
            p += packed;
        }
    }
}

/* ---- the file ------------------------------------------------------- */

static bool vli(const uint8_t *b, size_t len, size_t *pos, uint32_t *val)
{
    uint64_t v = 0;
    unsigned sh;

    for (sh = 0; sh < 35 && *pos < len; sh += 7) {
        uint8_t c = b[(*pos)++];

        v |= (uint64_t)(c & 0x7F) << sh;
        if (!(c & 0x80)) {
            *val = v;
            return v <= UINT32_MAX;
        }
    }
    return false;
}

typedef struct {
    unsigned n, chunk;
    size_t data;                            /* where the chunks start */
    size_t total;                           /* and how long they run */
    uint32_t packed[SXZ_MAX_BLOCKS], unpacked[SXZ_MAX_BLOCKS];
} Sxz;

/* The header; false until it has all arrived or if it is not one. */
static bool sxz_parse(const uint8_t *b, size_t len, Sxz *x, bool *bad)
{
    size_t pos = 9, left[SXZ_MAX_BLOCKS];
    unsigned i;
    bool more = true;

    *bad = false;
    if (len < 0x40) {
        return false;
    }
    if (memcmp(b, "sstarXZ", 7) || !b[7] || b[7] > SXZ_MAX_BLOCKS || !b[8]) {
        *bad = true;
        return false;
    }
    x->n = b[7];
    x->chunk = b[8];
    for (i = 0; i < x->n; i++) {
        if (!vli(b, len, &pos, &x->packed[i]) ||
            !vli(b, len, &pos, &x->unpacked[i])) {
            *bad = true;
            return false;
        }
        left[i] = x->packed[i];
    }
    x->data = ROUND_UP(pos, 16);
    x->total = 0;
    while (more) {
        more = false;
        for (i = 0; i < x->n; i++) {
            if (left[i]) {
                left[i] -= MIN(left[i], x->chunk);
                x->total += x->chunk;
                more = true;
            }
        }
    }
    return true;
}

static void xzd_finish(SstarXzdecState *s, unsigned n, bool ok)
{
    uint16_t bit = 1u << (n - 1);

    s->done = true;
    s->regs[R_IRQ_RAW] = bit;
    s->regs[R_IRQ_FINAL] = bit;
    s->regs[R_HDR_CHECK] = ok ? 0xFF : 0;
    s->regs[R_DATA_CHECK] = ok ? 0xFF : 0;
    s->regs[R_MAGIC_CHECK] = ok ? 0x0F : 0;
    qemu_irq_raise(s->irq);
}

static bool xzd_block(SstarXzdecState *s, const uint8_t *b, size_t len,
                      uint32_t unpacked, hwaddr dst)
{
    g_autofree uint8_t *out = NULL;
    size_t hdr, pos = 2, got = 0, used = 0;
    uint32_t cz, uz;
    uint32_t crc;

    if (len < 12 || unpacked > SXZ_MAX_UNPACKED) {
        return false;
    }
    hdr = (b[0] + 1) * 4;
    if (hdr >= len || b[1] != 0xC0 || !vli(b, hdr, &pos, &cz) ||
        !vli(b, hdr, &pos, &uz) || uz != unpacked || cz > len - hdr) {
        return false;
    }
    out = g_malloc(unpacked);
    if (!lzma2_decode(b + hdr, cz, false, out, unpacked, &got, &used) ||
        got != unpacked || used != cz) {
        return false;
    }
    /* the block pads to four bytes, then its CRC32 */
    pos = hdr + ROUND_UP(cz, 4);
    if (pos + 4 > len) {
        return false;
    }
    crc = crc32(0, out, unpacked);
    if (crc != ldl_le_p(b + pos)) {
        return false;
    }
    dma_memory_write(&address_space_memory, dst, out, unpacked,
                     MEMTXATTRS_UNSPECIFIED);
    return true;
}

static void xzd_decode(SstarXzdecState *s, const Sxz *x)
{
    const uint8_t *in = s->in->data;
    uint8_t *flat[SXZ_MAX_BLOCKS] = { };
    size_t fill[SXZ_MAX_BLOCKS] = { };
    size_t pos = x->data;
    unsigned i;
    bool more = true, ok = true;

    for (i = 0; i < x->n; i++) {
        flat[i] = g_malloc(x->packed[i]);
    }
    while (more) {
        more = false;
        for (i = 0; i < x->n; i++) {
            size_t n = x->packed[i] - fill[i];

            if (!n) {
                continue;
            }
            n = MIN(n, x->chunk);
            memcpy(flat[i] + fill[i], in + pos, n);
            fill[i] += n;
            pos += x->chunk;
            more = true;
        }
    }
    for (i = 0; i < x->n; i++) {
        uint32_t w = s->regs[OUT_BANK + 0x06 + 0x20 * i] |
                     (uint32_t)s->regs[OUT_BANK + 0x07 + 0x20 * i] << 16;

        if (!xzd_block(s, flat[i], x->packed[i], x->unpacked[i],
                       DRAM_BASE + ((hwaddr)w << 4))) {
            qemu_log_mask(LOG_GUEST_ERROR, "sstar-xzdec: block %u corrupt\n",
                          i);
            ok = false;
        }
    }
    for (i = 0; i < x->n; i++) {
        g_free(flat[i]);
    }
    xzd_finish(s, x->n, ok);
}

static void xzd_decode_plain(SstarXzdecState *s);

void sstar_xzdec_push(SstarXzdecState *s, const uint8_t *buf, uint32_t len)
{
    Sxz x;
    bool bad;

    if (s->done || s->in->len + (uint64_t)len > SXZ_MAX_FILE) {
        return;
    }
    g_byte_array_append(s->in, buf, len);
    if (s->regs[R_CTRL0] == CTRL_PLAIN_XZ) {
        uint32_t size = s->regs[R_XZ_SIZE_L] |
                        (uint32_t)s->regs[R_XZ_SIZE_H] << 16;

        if (size && s->in->len >= size) {
            xzd_decode_plain(s);
        }
        return;
    }
    if (sxz_parse(s->in->data, s->in->len, &x, &bad)) {
        if (x.data + x.total <= s->in->len) {
            xzd_decode(s, &x);
        }
    } else if (bad) {
        qemu_log_mask(LOG_UNIMP, "sstar-xzdec: not an SXZ file\n");
        xzd_finish(s, 1, false);
    }
}

static uint64_t crc64_xz(const uint8_t *p, size_t n)
{
    uint64_t c = ~0ull;

    while (n--) {
        int i;

        c ^= *p++;
        for (i = 0; i < 8; i++) {
            c = (c >> 1) ^ (c & 1 ? 0xC96C5795D7870F42ull : 0);
        }
    }
    return ~c;
}

/* One block of an .xz stream at b; *blen is its length, padding and check. */
static bool xz_block(const uint8_t *b, size_t len, unsigned check, uint8_t *out,
                     size_t outmax, size_t *got, size_t *blen)
{
    size_t hdr, pos = 2, used = 0, end;
    uint32_t cz = 0, uz = 0;
    unsigned flags;

    if (len < 2) {
        return false;
    }
    hdr = (b[0] + 1) * 4;
    flags = b[1];
    if (hdr + 4 > len || (flags & 3) || (flags & 0x3C)) {
        return false;                       /* one filter, known flags */
    }
    if (crc32(0, b, hdr - 4) != (uint32_t)ldl_le_p(b + hdr - 4)) {
        return false;
    }
    if ((flags & 0x40) && !vli(b, hdr, &pos, &cz)) {
        return false;
    }
    if ((flags & 0x80) && !vli(b, hdr, &pos, &uz)) {
        return false;
    }
    if (pos + 3 > hdr || b[pos] != 0x21 || b[pos + 1] != 1) {
        return false;                       /* LZMA2, one property byte */
    }
    if (!lzma2_decode(b + hdr, len - hdr, true, out, outmax, got, &used)) {
        return false;
    }
    if (((flags & 0x40) && used != cz) || ((flags & 0x80) && *got != uz)) {
        return false;
    }
    end = hdr + ROUND_UP(used, 4);
    switch (check) {
    case 0:
        break;
    case 1:
        if (end + 4 > len ||
            crc32(0, out, *got) != (uint32_t)ldl_le_p(b + end)) {
            return false;
        }
        end += 4;
        break;
    case 4:
        if (end + 8 > len || crc64_xz(out, *got) != ldq_le_p(b + end)) {
            return false;
        }
        end += 8;
        break;
    default:
        return false;
    }
    *blen = end;
    return true;
}

/* The index at pos, of this many blocks, and the footer after it. */
static bool xz_tail(const uint8_t *b, size_t len, size_t pos, uint32_t blocks)
{
    size_t start = pos;
    uint32_t n, v, i;

    if (pos >= len || b[pos++]) {
        return false;
    }
    if (!vli(b, len, &pos, &n) || n != blocks) {
        return false;
    }
    for (i = 0; i < n * 2; i++) {
        if (!vli(b, len, &pos, &v)) {
            return false;
        }
    }
    pos = start + ROUND_UP(pos - start, 4);
    if (pos + 4 + 12 > len ||
        crc32(0, b + start, pos - start) != (uint32_t)ldl_le_p(b + pos)) {
        return false;
    }
    pos += 4;
    return crc32(0, b + pos + 4, 6) == (uint32_t)ldl_le_p(b + pos) &&
           !memcmp(b + pos + 8, b + 6, 2) && b[pos + 10] == 'Y' &&
           b[pos + 11] == 'Z';
}

static void xzd_decode_plain(SstarXzdecState *s)
{
    static const uint8_t magic[6] = { 0xFD, '7', 'z', 'X', 'Z', 0 };
    const uint8_t *b = s->in->data;
    size_t len = s->in->len, pos = 12, o = 0;
    uint32_t blocks = 0;
    g_autofree uint8_t *out = NULL;
    uint32_t w = s->regs[OUT_BANK + 0x06] |
                 (uint32_t)s->regs[OUT_BANK + 0x07] << 16;
    bool ok = len >= 24 && !memcmp(b, magic, sizeof(magic)) &&
              crc32(0, b + 6, 2) == (uint32_t)ldl_le_p(b + 8);

    if (ok) {
        out = g_try_malloc(SXZ_MAX_UNPACKED);
        ok = out != NULL;
    }
    while (ok && pos < len && b[pos]) {     /* 0 here starts the index */
        size_t got = 0, blen = 0;

        ok = xz_block(b + pos, len - pos, b[7] & 0xF, out + o,
                      SXZ_MAX_UNPACKED - o, &got, &blen);
        pos += blen;
        o += got;
        blocks++;
    }
    ok = ok && xz_tail(b, len, pos, blocks);
    if (ok) {
        dma_memory_write(&address_space_memory,
                         DRAM_BASE + ((hwaddr)w << 4), out, o,
                         MEMTXATTRS_UNSPECIFIED);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-xzdec: bad .xz file\n");
    }
    xzd_finish(s, 1, ok);
}

/* ---- the registers -------------------------------------------------- */

static void xzd_clear(SstarXzdecState *s)
{
    g_byte_array_set_size(s->in, 0);
    s->done = false;
    s->regs[R_IRQ_RAW] = 0;
    s->regs[R_IRQ_FINAL] = 0;
    s->regs[R_HDR_CHECK] = 0;
    s->regs[R_DATA_CHECK] = 0;
    s->regs[R_MAGIC_CHECK] = 0;
    qemu_irq_lower(s->irq);
}

static uint64_t sstar_xzdec_read(void *opaque, hwaddr off, unsigned size)
{
    SstarXzdecState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2 || (off >> 2) >= XZD_NREGS) {
        return 0;
    }
    v = s->regs[off >> 2];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_xzdec_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    SstarXzdecState *s = opaque;
    unsigned reg = off >> 2;
    uint16_t v = val;

    if ((off & 3) >= 2 || reg >= XZD_NREGS) {
        return;
    }
    if (size == 1) {
        v = (off & 1) ? (s->regs[reg] & 0x00FF) | ((val & 0xFF) << 8)
                      : (s->regs[reg] & 0xFF00) | (val & 0xFF);
    }
    switch (reg) {
    case R_SW_RST:
        s->regs[reg] = v;
        if (v) {
            xzd_clear(s);
        }
        break;
    case R_CTRL0:
        s->regs[reg] = v;
        xzd_clear(s);
        break;
    case R_IRQ_CLR:
        s->regs[R_IRQ_FINAL] &= ~v;
        s->regs[R_IRQ_RAW] &= ~v;
        if (!s->regs[R_IRQ_FINAL]) {
            qemu_irq_lower(s->irq);
        }
        break;
    case R_IRQ_FINAL:
    case R_IRQ_RAW:
    case R_HDR_CHECK:
    case R_DATA_CHECK:
    case R_MAGIC_CHECK:
        break;
    default:
        s->regs[reg] = v;
        break;
    }
}

static const MemoryRegionOps sstar_xzdec_ops = {
    .read = sstar_xzdec_read,
    .write = sstar_xzdec_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
};

static void sstar_xzdec_reset(DeviceState *dev)
{
    SstarXzdecState *s = SSTAR_XZDEC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    xzd_clear(s);
}

static void sstar_xzdec_realize(DeviceState *dev, Error **errp)
{
    SstarXzdecState *s = SSTAR_XZDEC(dev);

    /* the input in flight is not in the migration stream */
    error_setg(&s->migration_blocker, "sstar-xzdec is not migratable");
    if (migrate_add_blocker(&s->migration_blocker, errp) < 0) {
        return;
    }
}

static void sstar_xzdec_init(Object *obj)
{
    SstarXzdecState *s = SSTAR_XZDEC(obj);

    s->in = g_byte_array_new();
    memory_region_init_io(&s->iomem, obj, &sstar_xzdec_ops, s, TYPE_SSTAR_XZDEC,
                          XZD_BANKS * 0x200);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void sstar_xzdec_finalize(Object *obj)
{
    g_byte_array_unref(SSTAR_XZDEC(obj)->in);
}

static void sstar_xzdec_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_xzdec_realize;
    device_class_set_legacy_reset(dc, sstar_xzdec_reset);
}

static const TypeInfo sstar_xzdec_info = {
    .name = TYPE_SSTAR_XZDEC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SstarXzdecState),
    .instance_init = sstar_xzdec_init,
    .instance_finalize = sstar_xzdec_finalize,
    .class_init = sstar_xzdec_class_init,
};

static void sstar_xzdec_register(void)
{
    type_register_static(&sstar_xzdec_info);
}

type_init(sstar_xzdec_register)
