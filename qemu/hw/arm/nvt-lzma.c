/*
 * LZMA ("LZMA alone" / .lzma) decoder for the Novatek boot ROM stand-in.
 *
 * NA51103 boards keep their device tree and U-Boot LZMA-packed in flash,
 * and the closed loader unpacks them with its own decoder.  QEMU links no
 * LZMA library, so this is a plain decoder after Igor Pavlov's
 * public-domain reference (LzmaSpec.cpp), decoding into a flat buffer that
 * doubles as the dictionary: it runs once per boot, on a few hundred KiB.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/arm/nvt-lzma.h"

#define LZMA_HDR            13
#define PROB_BITS           11
#define PROB_INIT           (1 << (PROB_BITS - 1))
#define NUM_STATES          12
#define POS_STATES_MAX      16
#define END_POS_MODEL       14
#define FULL_DISTANCES      128
#define ALIGN_BITS          4
#define MATCH_MIN_LEN       2

typedef uint16_t Prob;

typedef struct {
    const uint8_t *in;
    size_t len, pos;
    uint32_t range, code;
    bool overrun;
} RangeDec;

typedef struct {
    Prob choice, choice2;
    Prob low[POS_STATES_MAX][1 << 3];
    Prob mid[POS_STATES_MAX][1 << 3];
    Prob high[1 << 8];
} LenDec;

typedef struct {
    RangeDec rc;
    unsigned lc, lp, pb;
    Prob *lit;
    Prob is_match[NUM_STATES * POS_STATES_MAX];
    Prob is_rep[NUM_STATES], is_rep_g0[NUM_STATES];
    Prob is_rep_g1[NUM_STATES], is_rep_g2[NUM_STATES];
    Prob is_rep0_long[NUM_STATES * POS_STATES_MAX];
    Prob pos_slot[4][1 << 6];
    Prob pos[1 + FULL_DISTANCES - END_POS_MODEL];
    Prob align[1 << ALIGN_BITS];
    LenDec len, rep_len;
} Lzma;

static uint8_t rc_byte(RangeDec *rc)
{
    if (rc->pos >= rc->len) {
        rc->overrun = true;
        return 0;
    }
    return rc->in[rc->pos++];
}

static void rc_normalize(RangeDec *rc)
{
    if (rc->range < (1u << 24)) {
        rc->range <<= 8;
        rc->code = (rc->code << 8) | rc_byte(rc);
    }
}

static unsigned rc_bit(RangeDec *rc, Prob *p)
{
    uint32_t bound = (rc->range >> PROB_BITS) * *p;
    unsigned bit;

    if (rc->code < bound) {
        *p += ((1 << PROB_BITS) - *p) >> 5;
        rc->range = bound;
        bit = 0;
    } else {
        *p -= *p >> 5;
        rc->code -= bound;
        rc->range -= bound;
        bit = 1;
    }
    rc_normalize(rc);
    return bit;
}

static uint32_t rc_direct(RangeDec *rc, unsigned bits)
{
    uint32_t res = 0;

    while (bits--) {
        uint32_t t;

        rc->range >>= 1;
        rc->code -= rc->range;
        t = 0 - (rc->code >> 31);
        rc->code += rc->range & t;
        rc_normalize(rc);
        res = (res << 1) + (t + 1);
    }
    return res;
}

static unsigned bittree(RangeDec *rc, Prob *probs, unsigned bits)
{
    unsigned m = 1, i;

    for (i = 0; i < bits; i++) {
        m = (m << 1) + rc_bit(rc, &probs[m]);
    }
    return m - (1u << bits);
}

static unsigned bittree_rev(RangeDec *rc, Prob *probs, unsigned bits)
{
    unsigned m = 1, sym = 0, i, bit;

    for (i = 0; i < bits; i++) {
        bit = rc_bit(rc, &probs[m]);
        m = (m << 1) + bit;
        sym |= bit << i;
    }
    return sym;
}

static unsigned len_decode(RangeDec *rc, LenDec *ld, unsigned pos_state)
{
    if (!rc_bit(rc, &ld->choice)) {
        return bittree(rc, ld->low[pos_state], 3);
    }
    if (!rc_bit(rc, &ld->choice2)) {
        return 8 + bittree(rc, ld->mid[pos_state], 3);
    }
    return 16 + bittree(rc, ld->high, 8);
}

static uint32_t dist_decode(Lzma *z, unsigned len)
{
    unsigned slot = bittree(&z->rc, z->pos_slot[MIN(len, 3u)], 6);
    unsigned direct;
    uint32_t dist;

    if (slot < 4) {
        return slot;
    }
    direct = (slot >> 1) - 1;
    dist = (2 | (slot & 1)) << direct;
    if (slot < END_POS_MODEL) {
        return dist + bittree_rev(&z->rc, z->pos + dist - slot, direct);
    }
    dist += rc_direct(&z->rc, direct - ALIGN_BITS) << ALIGN_BITS;
    return dist + bittree_rev(&z->rc, z->align, ALIGN_BITS);
}

static void probs_init(Prob *p, size_t n)
{
    while (n--) {
        *p++ = PROB_INIT;
    }
}

bool nvt_lzma_probe(const uint8_t *in, size_t inlen)
{
    return inlen > LZMA_HDR && in[0] < 9 * 5 * 5 && in[LZMA_HDR] == 0;
}

ssize_t nvt_lzma_decode(const uint8_t *in, size_t inlen,
                        uint8_t *out, size_t outmax)
{
    g_autofree Lzma *z = g_new0(Lzma, 1);
    g_autofree Prob *lit = NULL;
    uint64_t unpack = ldq_le_p(in + 5);
    bool sized = unpack != UINT64_MAX;
    uint32_t rep0 = 0, rep1 = 0, rep2 = 0, rep3 = 0;
    unsigned state = 0, d = in[0];
    size_t opos = 0, nlit;

    if (!nvt_lzma_probe(in, inlen) || (sized && unpack > outmax)) {
        return -1;
    }
    z->lc = d % 9;
    d /= 9;
    z->lp = d % 5;
    z->pb = d / 5;
    nlit = 0x300u << (z->lc + z->lp);
    lit = g_new(Prob, nlit);
    probs_init(lit, nlit);
    probs_init((Prob *)&z->is_match,
               (sizeof(*z) - offsetof(Lzma, is_match)) / sizeof(Prob));

    z->rc.in = in + LZMA_HDR;
    z->rc.len = inlen - LZMA_HDR;
    z->rc.range = 0xFFFFFFFF;
    rc_byte(&z->rc);                        /* always 0 */
    z->rc.code = (uint32_t)rc_byte(&z->rc) << 24;
    z->rc.code |= (uint32_t)rc_byte(&z->rc) << 16;
    z->rc.code |= (uint32_t)rc_byte(&z->rc) << 8;
    z->rc.code |= rc_byte(&z->rc);

    for (;;) {
        unsigned pos_state = opos & ((1u << z->pb) - 1);
        unsigned len;

        if (z->rc.overrun) {
            return -1;
        }
        if (sized && opos == unpack) {
            break;                          /* the marker is optional */
        }
        if (!rc_bit(&z->rc, &z->is_match[(state << 4) + pos_state])) {
            unsigned prev = opos ? out[opos - 1] : 0;
            Prob *p = lit + 0x300 * (((opos & ((1u << z->lp) - 1)) << z->lc) +
                                     (prev >> (8 - z->lc)));
            unsigned sym = 1;

            if (opos >= outmax) {
                return -1;
            }
            if (state >= 7) {
                unsigned match = out[opos - rep0 - 1];

                do {
                    unsigned mbit = (match >> 7) & 1, bit;

                    match <<= 1;
                    bit = rc_bit(&z->rc, &p[((1 + mbit) << 8) + sym]);
                    sym = (sym << 1) | bit;
                    if (mbit != bit) {
                        break;
                    }
                } while (sym < 0x100);
            }
            while (sym < 0x100) {
                sym = (sym << 1) | rc_bit(&z->rc, &p[sym]);
            }
            out[opos++] = sym - 0x100;
            state = state < 4 ? 0 : state < 10 ? state - 3 : state - 6;
            continue;
        }
        if (rc_bit(&z->rc, &z->is_rep[state])) {
            if (!opos) {
                return -1;
            }
            if (!rc_bit(&z->rc, &z->is_rep_g0[state])) {
                if (!rc_bit(&z->rc,
                            &z->is_rep0_long[(state << 4) + pos_state])) {
                    if (opos >= outmax) {
                        return -1;
                    }
                    state = state < 7 ? 9 : 11;
                    out[opos] = out[opos - rep0 - 1];
                    opos++;
                    continue;
                }
            } else {
                uint32_t dist;

                if (!rc_bit(&z->rc, &z->is_rep_g1[state])) {
                    dist = rep1;
                } else {
                    if (!rc_bit(&z->rc, &z->is_rep_g2[state])) {
                        dist = rep2;
                    } else {
                        dist = rep3;
                        rep3 = rep2;
                    }
                    rep2 = rep1;
                }
                rep1 = rep0;
                rep0 = dist;
            }
            len = len_decode(&z->rc, &z->rep_len, pos_state);
            state = state < 7 ? 8 : 11;
        } else {
            rep3 = rep2;
            rep2 = rep1;
            rep1 = rep0;
            len = len_decode(&z->rc, &z->len, pos_state);
            state = state < 7 ? 7 : 10;
            rep0 = dist_decode(z, len);
            if (rep0 == 0xFFFFFFFF) {       /* end marker */
                return z->rc.code == 0 && !z->rc.overrun ? opos : -1;
            }
            if (rep0 >= opos) {
                return -1;
            }
        }
        len += MATCH_MIN_LEN;
        if (len > outmax - opos || (sized && len > unpack - opos)) {
            return -1;
        }
        while (len--) {
            out[opos] = out[opos - rep0 - 1];
            opos++;
        }
    }
    return z->rc.overrun ? -1 : opos;
}
