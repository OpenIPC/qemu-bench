/*
 * Ingenic DES: the DES / triple-DES engine at 0x10061000 (T31, T40, T41),
 * as the SDKs' sample_des (jz-des.c) drives it:
 *
 *   0x00 DESCR1  bit 0 work; setting it empties the FIFOs and loads the
 *                CBC chain from the IV
 *   0x04 DESCR2  bit 4 little endian, 3 decrypt, 2 CBC (else ECB), 1
 *                triple DES (else single), 0 DMA
 *   0x08 DESSR   bit 0 the output FIFO is empty
 *   0x10..0x24   DESK1L/R .. DESK3L/R: the keys, L the high word
 *   0x28 DESIVL  0x2C DESIVR: the CBC initial vector
 *   0x30 DESDIN  input, a 32-bit word per write
 *   0x34 DESDOUT output, a 32-bit word per read
 *
 * Every second word written to DESDIN while working completes a 64-bit
 * block (L, then R), which is ciphered at once into the output FIFO (16
 * words).  The driver feeds short buffers word by word, polling DESSR
 * before each DESDOUT read, and longer ones (32 bytes and up) through
 * the PDMA: "drq-tx" (request type 0x2E) is high while working with DMA
 * set and the output FIFO has room for a block, "drq-rx" (0x2F) while
 * it holds a word.  Triple DES is encrypt-decrypt-encrypt with K1, K2,
 * K3.
 *
 * The byte order is the one the driver's round trip needs and nothing
 * pins down further: keys and IV are taken as written (L the high 32
 * bits of the 64), data words as written with bit 4 clear and byte-
 * swapped with it set -- so that a buffer's bytes, little-endian words
 * in memory, form the block in memory order.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_DES "ingenic-des"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDesState, INGENIC_DES)

#define R_CR1           0x00
#define R_CR2           0x04
#define R_SR            0x08
#define R_K1L           0x10
#define R_IVL           0x28
#define R_IVR           0x2C
#define R_DIN           0x30
#define R_DOUT          0x34
#define DES_REGS        0x38

#define CR1_WORK        (1u << 0)
#define CR2_LE          (1u << 4)
#define CR2_DEC         (1u << 3)
#define CR2_CBC         (1u << 2)
#define CR2_TDES        (1u << 1)
#define CR2_DMA         (1u << 0)

#define OUT_DEPTH       16

struct IngenicDesState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq drq_tx, drq_rx;

    uint32_t regs[DES_REGS / 4];
    uint32_t in[2], nin;
    uint32_t out[OUT_DEPTH], out_head, out_count;
    uint64_t chain;
};

/* ---- DES (FIPS 46-3) ---------------------------------------------------- */

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
static const uint8_t des_s[8][64] = {
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

/* bit i (1-based, from the most significant) of an n-bit value */
static uint64_t des_permute(uint64_t v, int n, const uint8_t *tab, int m)
{
    uint64_t r = 0;
    int i;

    for (i = 0; i < m; i++) {
        r = (r << 1) | ((v >> (n - tab[i])) & 1);
    }
    return r;
}

static void des_subkeys(uint64_t key, uint64_t k[16])
{
    uint64_t cd = des_permute(key, 64, des_pc1, 56);
    uint32_t c = cd >> 28, d = cd & 0xfffffff;
    int i;

    for (i = 0; i < 16; i++) {
        c = ((c << des_shifts[i]) | (c >> (28 - des_shifts[i]))) & 0xfffffff;
        d = ((d << des_shifts[i]) | (d >> (28 - des_shifts[i]))) & 0xfffffff;
        k[i] = des_permute(((uint64_t)c << 28) | d, 56, des_pc2, 48);
    }
}

static uint32_t des_f(uint32_t r, uint64_t k)
{
    uint64_t x = des_permute(r, 32, des_e, 48) ^ k;
    uint32_t s = 0;
    int i;

    for (i = 0; i < 8; i++) {
        unsigned six = (x >> (42 - 6 * i)) & 0x3f;
        unsigned row = ((six >> 4) & 2) | (six & 1), col = (six >> 1) & 0xf;

        s = (s << 4) | des_s[i][row * 16 + col];
    }
    return des_permute(s, 32, des_p, 32);
}

static uint64_t des_one(uint64_t block, uint64_t key, bool decrypt)
{
    uint64_t k[16], v = des_permute(block, 64, des_ip, 64);
    uint32_t l = v >> 32, r = v, t;
    int i;

    des_subkeys(key, k);
    for (i = 0; i < 16; i++) {
        t = r;
        r = l ^ des_f(r, k[decrypt ? 15 - i : i]);
        l = t;
    }
    return des_permute(((uint64_t)r << 32) | l, 64, des_fp, 64);
}

/* ---- the engine ----------------------------------------------------------- */

static uint64_t des_key(IngenicDesState *s, int n)
{
    return ((uint64_t)s->regs[(R_K1L + 8 * n) / 4] << 32) |
           s->regs[(R_K1L + 8 * n + 4) / 4];
}

static uint64_t des_cipher(IngenicDesState *s, uint64_t b, bool dec)
{
    if (!(s->regs[R_CR2 / 4] & CR2_TDES)) {
        return des_one(b, des_key(s, 0), dec);
    }
    if (!dec) {
        b = des_one(b, des_key(s, 0), false);
        b = des_one(b, des_key(s, 1), true);
        return des_one(b, des_key(s, 2), false);
    }
    b = des_one(b, des_key(s, 2), true);
    b = des_one(b, des_key(s, 1), false);
    return des_one(b, des_key(s, 0), true);
}

static uint32_t des_order(IngenicDesState *s, uint32_t w)
{
    return (s->regs[R_CR2 / 4] & CR2_LE) ? bswap32(w) : w;
}

static void des_push(IngenicDesState *s, uint32_t w)
{
    if (s->out_count < OUT_DEPTH) {
        s->out[(s->out_head + s->out_count++) % OUT_DEPTH] = w;
    }
}

static void des_block(IngenicDesState *s)
{
    uint32_t cr2 = s->regs[R_CR2 / 4];
    bool dec = cr2 & CR2_DEC, cbc = cr2 & CR2_CBC;
    uint64_t b = ((uint64_t)des_order(s, s->in[0]) << 32) |
                 des_order(s, s->in[1]);
    uint64_t o;

    if (!dec) {
        o = des_cipher(s, cbc ? b ^ s->chain : b, false);
        s->chain = o;
    } else {
        o = des_cipher(s, b, true) ^ (cbc ? s->chain : 0);
        s->chain = b;
    }
    des_push(s, des_order(s, o >> 32));
    des_push(s, des_order(s, o));
}

static void des_update(IngenicDesState *s)
{
    bool dma = (s->regs[R_CR2 / 4] & CR2_DMA) &&
               (s->regs[R_CR1 / 4] & CR1_WORK);

    qemu_set_irq(s->drq_tx, dma && s->out_count <= OUT_DEPTH - 2);
    qemu_set_irq(s->drq_rx, dma && s->out_count);
}

static uint64_t ingenic_des_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicDesState *s = opaque;
    uint32_t v;

    switch (off) {
    case R_SR:
        return !s->out_count;
    case R_DOUT:
        if (!s->out_count) {
            return 0;
        }
        v = s->out[s->out_head];
        s->out_head = (s->out_head + 1) % OUT_DEPTH;
        s->out_count--;
        des_update(s);
        return v;
    case R_DIN:
        return 0;
    }
    return off < DES_REGS ? s->regs[off / 4] : 0;
}

static void ingenic_des_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicDesState *s = opaque;

    switch (off) {
    case R_CR1:
        if ((val & CR1_WORK) && !(s->regs[R_CR1 / 4] & CR1_WORK)) {
            s->nin = 0;                 /* the FIFOs empty, the chain loads */
            s->out_count = 0;
            s->out_head = 0;
            s->chain = ((uint64_t)s->regs[R_IVL / 4] << 32) |
                       s->regs[R_IVR / 4];
        }
        s->regs[R_CR1 / 4] = val;
        break;
    case R_DIN:
        if (!(s->regs[R_CR1 / 4] & CR1_WORK)) {
            break;
        }
        s->in[s->nin++] = val;
        if (s->nin == 2) {
            s->nin = 0;
            des_block(s);
        }
        break;
    case R_SR:
    case R_DOUT:
        break;
    default:
        if (off < DES_REGS) {
            s->regs[off / 4] = val;
        }
        break;
    }
    des_update(s);
}

static const MemoryRegionOps ingenic_des_ops = {
    .read = ingenic_des_read,
    .write = ingenic_des_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_des_reset(DeviceState *dev)
{
    IngenicDesState *s = INGENIC_DES(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->nin = s->out_head = s->out_count = 0;
    s->chain = 0;
    des_update(s);
}

static void ingenic_des_init(Object *obj)
{
    IngenicDesState *s = INGENIC_DES(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_des_ops, s,
                          TYPE_INGENIC_DES, 0x1000);
    /* the PDMA moves words from within a DESDIN / DESDOUT access */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), &s->drq_tx, "drq-tx", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->drq_rx, "drq-rx", 1);
}

static const VMStateDescription vmstate_ingenic_des = {
    .name = TYPE_INGENIC_DES,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicDesState, DES_REGS / 4),
        VMSTATE_UINT32_ARRAY(in, IngenicDesState, 2),
        VMSTATE_UINT32(nin, IngenicDesState),
        VMSTATE_UINT32_ARRAY(out, IngenicDesState, OUT_DEPTH),
        VMSTATE_UINT32(out_head, IngenicDesState),
        VMSTATE_UINT32(out_count, IngenicDesState),
        VMSTATE_UINT64(chain, IngenicDesState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_des_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_des_reset);
    dc->vmsd = &vmstate_ingenic_des;
}

static const TypeInfo ingenic_des_info = {
    .name = TYPE_INGENIC_DES,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDesState),
    .instance_init = ingenic_des_init,
    .class_init = ingenic_des_class_init,
};

static void ingenic_des_register_types(void)
{
    type_register_static(&ingenic_des_info);
}

type_init(ingenic_des_register_types)
