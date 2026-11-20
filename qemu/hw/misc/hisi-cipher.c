/*
 * HiSilicon CIPHER (v1): the symmetric engine of the Hi3516CV200 /
 * Hi3518EV200 family, at 0x100C0000 (VIC 13).
 *
 * The register contract is hi_cipher.ko's (openhisilicon
 * kernel/cipher/hi3516cv200, drv_cipher_reg.h / hal_cipher.c /
 * drv_cipher.c):
 *
 *   channel 0  one 16-byte packet through registers: key 0x90, CTRL
 *              0x1000, IV in 0x1004, data in 0x1014, START = CHAN0_CFG
 *              (0x1410) bit 0, busy bit 1; data out 0x00, IV out 0x10;
 *              done is INT_RAW bit 8
 *   channel n  (1..7) two 128-entry rings in DDR of 16-byte descriptors
 *              { data address, flags, length, IV address } -- flags bit 21:
 *              take the IV from the IV address first, bit 22: end of list.
 *              Registers at 0x1000 + n * 0x80: IBUF_NUM / OBUF_NUM ring
 *              sizes, IBUF_CNT / OBUF_CNT the entries the driver hands
 *              over (a write adds), IEMPTY_CNT / OFULL_CNT the entries the
 *              engine has finished (a write subtracts), INT_OCNTCFG the
 *              OFULL_CNT that raises INT_RAW bit 8 + n.  The source and
 *              destination entries pair up one to one.  SRC_LST_RADDR /
 *              DEST_LST_RADDR (0x20 / 0x58) are the engine's positions as
 *              entry numbers, not addresses: DRV_CIPHER_ConfigChn takes
 *              them as the rings' heads.
 *   INT_STATUS 0x1400 = INT_RAW (0x1408, write 1 to clear) & INT_EN
 *              (0x1404, bit 31 the global enable)
 *
 * CTRL: [0] decrypt, [3:1] mode (AES: ECB, CBC, CFB, OFB, CTR), [5:4]
 * algorithm (2 = AES; DES / 3DES are not modelled), [7:6] width (128-bit
 * only), [8] IV change, [10:9] key length (128 / 192 / 256), [13] key from
 * the eFuse, [16:14] key slot.  Keys, IVs and data are little-endian
 * words of the byte strings, as the driver copies them.
 *
 * An eFuse key is a secret of the die; the model uses "efuse-key" (32 hex
 * digits, zeros by default) for all four, so what a firmware derives from
 * it differs from the camera's.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "crypto/aes.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_HISI_CIPHER "hisi-cipher"
OBJECT_DECLARE_SIMPLE_TYPE(HisiCipherState, HISI_CIPHER)

#define CI_SIZE             0x2000
#define CI_CHANS            8
#define CI_DOUT             0x0000
#define CI_IVOUT(n)         (0x0010 + (n) * 16)
#define CI_KEY(n)           (0x0090 + (n) * 32)
#define CI_CHAN(n)          (0x1000 + (n) * 0x80)
#define CI_CH0_CTRL         0x1000
#define CI_CH0_IVIN         0x1004
#define CI_CH0_DIN          0x1014
#define CI_IBUF_NUM         0x00
#define CI_IBUF_CNT         0x04
#define CI_IEMPTY_CNT       0x08
#define CI_INT_ICNTCFG      0x0c
#define CI_CTRL             0x10
#define CI_SRC_LST_SADDR    0x14
#define CI_SRC_LST_RADDR    0x20
#define CI_OBUF_NUM         0x3c
#define CI_OBUF_CNT         0x40
#define CI_OFULL_CNT        0x44
#define CI_INT_OCNTCFG      0x48
#define CI_DEST_LST_SADDR   0x4c
#define CI_DEST_LST_RADDR   0x58
#define CI_INT_STATUS       0x1400
#define CI_INT_EN           0x1404
#define CI_INT_RAW          0x1408
#define CI_CH0_CFG          0x1410

#define CI_FLAG_IVSET       (1u << 21)
#define CI_LEN_MASK         0xfffff
#define CI_JOB_NS           10000
#define CI_RING_MAX         128     /* the driver's CIPHER_MAX_LIST_NUM */

struct HisiCipherState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[CI_SIZE / 4];
    uint8_t iv[CI_CHANS][16];       /* chaining value, per channel */
    uint32_t in_ptr[CI_CHANS];      /* ring positions, in entries */
    uint32_t out_ptr[CI_CHANS];
    uint32_t pending;               /* INT_RAW bits the timer raises */

    char *efuse_key_hex;
    uint8_t efuse_key[16];
};

static uint32_t *ci_reg(HisiCipherState *s, hwaddr a)
{
    return &s->regs[a / 4];
}

static uint32_t *ci_chan(HisiCipherState *s, int n, hwaddr r)
{
    return ci_reg(s, CI_CHAN(n) + r);
}

static void ci_update_irq(HisiCipherState *s)
{
    uint32_t en = *ci_reg(s, CI_INT_EN);
    uint32_t st = (en & (1u << 31)) ? *ci_reg(s, CI_INT_RAW) & en & 0xffff
                                     : 0;

    *ci_reg(s, CI_INT_STATUS) = st;
    qemu_set_irq(s->irq, st != 0);
}

static void ci_words_to_bytes(const uint32_t *w, uint8_t *b, int n)
{
    for (int i = 0; i < n; i++) {
        stl_le_p(b + 4 * i, w[i]);
    }
}

/* One run of a channel's mode over len bytes (a whole number of blocks) */
static void ci_crypt(HisiCipherState *s, int ch, uint32_t ctrl,
                     uint8_t *buf, uint32_t len)
{
    static const int key_bits[4] = { 128, 192, 256, 128 };
    bool dec = ctrl & 1;
    int mode = (ctrl >> 1) & 7, alg = (ctrl >> 4) & 3;
    int bits = key_bits[(ctrl >> 9) & 3];
    uint8_t key[32], tmp[16], prev[16];
    uint8_t *iv = s->iv[ch];
    AES_KEY ek, dk;

    if (alg != 2) {
        qemu_log_mask(LOG_UNIMP, "hisi-cipher: chn %d algorithm %d (DES) "
                      "not modelled; data passed through\n", ch, alg);
        return;
    }
    if (ctrl & (1u << 13)) {
        memset(key, 0, sizeof(key));
        memcpy(key, s->efuse_key, 16);
        bits = 128;
    } else {
        ci_words_to_bytes(ci_reg(s, CI_KEY((ctrl >> 14) & 7)), key, 8);
    }
    AES_set_encrypt_key(key, bits, &ek);
    AES_set_decrypt_key(key, bits, &dk);

    for (uint32_t o = 0; o + 16 <= len; o += 16) {
        uint8_t *b = buf + o;

        switch (mode) {
        case 1:                                 /* CBC */
            if (dec) {
                memcpy(prev, b, 16);
                AES_decrypt(b, tmp, &dk);
                for (int i = 0; i < 16; i++) {
                    b[i] = tmp[i] ^ iv[i];
                }
                memcpy(iv, prev, 16);
            } else {
                for (int i = 0; i < 16; i++) {
                    b[i] ^= iv[i];
                }
                AES_encrypt(b, b, &ek);
                memcpy(iv, b, 16);
            }
            break;
        case 2:                                 /* CFB-128 */
            AES_encrypt(iv, tmp, &ek);
            memcpy(prev, b, 16);
            for (int i = 0; i < 16; i++) {
                b[i] ^= tmp[i];
            }
            memcpy(iv, dec ? prev : b, 16);
            break;
        case 3:                                 /* OFB */
            AES_encrypt(iv, iv, &ek);
            for (int i = 0; i < 16; i++) {
                b[i] ^= iv[i];
            }
            break;
        case 4:                                 /* CTR, big-endian counter */
            AES_encrypt(iv, tmp, &ek);
            for (int i = 0; i < 16; i++) {
                b[i] ^= tmp[i];
            }
            for (int i = 15; i >= 0 && !++iv[i]; i--) {
                /* carry */
            }
            break;
        default:                                /* ECB */
            if (dec) {
                AES_decrypt(b, b, &dk);
            } else {
                AES_encrypt(b, b, &ek);
            }
            break;
        }
    }

    /* A partial last block: the stream modes take that much keystream;
     * ECB and CBC need whole blocks (the driver pads to them) */
    if (len % 16) {
        uint8_t *b = buf + (len & ~15u);
        uint32_t n = len % 16;

        switch (mode) {
        case 2:                                 /* CFB-128 */
            AES_encrypt(iv, tmp, &ek);
            memcpy(prev, b, n);
            for (uint32_t i = 0; i < n; i++) {
                b[i] ^= tmp[i];
            }
            memcpy(iv, dec ? prev : b, n);
            break;
        case 3:                                 /* OFB */
            AES_encrypt(iv, iv, &ek);
            for (uint32_t i = 0; i < n; i++) {
                b[i] ^= iv[i];
            }
            break;
        case 4:                                 /* CTR */
            AES_encrypt(iv, tmp, &ek);
            for (uint32_t i = 0; i < n; i++) {
                b[i] ^= tmp[i];
            }
            for (int i = 15; i >= 0 && !++iv[i]; i--) {
                /* carry */
            }
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "hisi-cipher: chn %d: %u bytes "
                          "is not whole blocks for ECB / CBC; the last %u "
                          "are left as they are\n", ch, len, n);
            break;
        }
    }
}

static void ci_run_single(HisiCipherState *s)
{
    uint32_t ctrl = *ci_reg(s, CI_CH0_CTRL);
    uint8_t buf[16];

    if (ctrl & (1u << 8)) {
        ci_words_to_bytes(ci_reg(s, CI_CH0_IVIN), s->iv[0], 4);
    }
    ci_words_to_bytes(ci_reg(s, CI_CH0_DIN), buf, 4);
    ci_crypt(s, 0, ctrl, buf, 16);
    for (int i = 0; i < 4; i++) {
        *ci_reg(s, CI_DOUT + 4 * i) = ldl_le_p(buf + 4 * i);
        *ci_reg(s, CI_IVOUT(0) + 4 * i) = ldl_le_p(s->iv[0] + 4 * i);
    }
    s->pending |= 1u << 8;
}

/*
 * Runs at most one ring's worth of entries (the counts are the guest's to
 * write, and could ask for billions); returns whether more are waiting,
 * for the next timer tick.
 */
static bool ci_run_chan(HisiCipherState *s, int ch)
{
    uint32_t inum = MIN(*ci_chan(s, ch, CI_IBUF_NUM) ?: CI_RING_MAX,
                        CI_RING_MAX);
    uint32_t onum = MIN(*ci_chan(s, ch, CI_OBUF_NUM) ?: CI_RING_MAX,
                        CI_RING_MAX);
    uint32_t ctrl = *ci_chan(s, ch, CI_CTRL);
    uint32_t budget = MIN(inum, onum);
    bool done = false;

    while (*ci_chan(s, ch, CI_IBUF_CNT) && *ci_chan(s, ch, CI_OBUF_CNT) &&
           budget--) {
        uint32_t src[4], dst[4], len;
        hwaddr sa = *ci_chan(s, ch, CI_SRC_LST_SADDR) + s->in_ptr[ch] * 16;
        hwaddr da = *ci_chan(s, ch, CI_DEST_LST_SADDR) + s->out_ptr[ch] * 16;
        g_autofree uint8_t *buf = NULL;

        dma_memory_read(&address_space_memory, sa, src, sizeof(src),
                        MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory, da, dst, sizeof(dst),
                        MEMTXATTRS_UNSPECIFIED);
        if (le32_to_cpu(src[1]) & CI_FLAG_IVSET) {
            dma_memory_read(&address_space_memory, le32_to_cpu(src[3]),
                            s->iv[ch], 16, MEMTXATTRS_UNSPECIFIED);
        }
        len = MIN(le32_to_cpu(src[2]), le32_to_cpu(dst[2])) & CI_LEN_MASK;
        buf = g_malloc(len);
        dma_memory_read(&address_space_memory, le32_to_cpu(src[0]), buf, len,
                        MEMTXATTRS_UNSPECIFIED);
        ci_crypt(s, ch, ctrl, buf, len);
        dma_memory_write(&address_space_memory, le32_to_cpu(dst[0]), buf, len,
                         MEMTXATTRS_UNSPECIFIED);

        s->in_ptr[ch] = (s->in_ptr[ch] + 1) % inum;
        s->out_ptr[ch] = (s->out_ptr[ch] + 1) % onum;
        (*ci_chan(s, ch, CI_IBUF_CNT))--;
        (*ci_chan(s, ch, CI_OBUF_CNT))--;
        (*ci_chan(s, ch, CI_IEMPTY_CNT))++;
        (*ci_chan(s, ch, CI_OFULL_CNT))++;
        done = true;
    }
    *ci_chan(s, ch, CI_SRC_LST_RADDR) = s->in_ptr[ch];
    *ci_chan(s, ch, CI_DEST_LST_RADDR) = s->out_ptr[ch];
    if (done && *ci_chan(s, ch, CI_OFULL_CNT) >=
                MAX(*ci_chan(s, ch, CI_INT_OCNTCFG), 1)) {
        s->pending |= 1u << (8 + ch);
    }
    return *ci_chan(s, ch, CI_IBUF_CNT) && *ci_chan(s, ch, CI_OBUF_CNT);
}

/* Jobs complete on a timer, never inside the register write that starts
 * them: the driver's wait is armed after the write */
static void ci_kick(HisiCipherState *s)
{
    if (!timer_pending(s->timer)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CI_JOB_NS);
    }
}

static void ci_job_done(void *opaque)
{
    HisiCipherState *s = opaque;
    bool more = false;

    if (*ci_reg(s, CI_CH0_CFG) & 1) {
        ci_run_single(s);
        *ci_reg(s, CI_CH0_CFG) &= ~3u;
    }
    for (int ch = 1; ch < CI_CHANS; ch++) {
        more |= ci_run_chan(s, ch);
    }
    *ci_reg(s, CI_INT_RAW) |= s->pending;
    s->pending = 0;
    ci_update_irq(s);
    if (more) {
        ci_kick(s);
    }
}

static uint64_t hisi_cipher_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiCipherState *s = HISI_CIPHER(opaque);

    return offset < CI_SIZE ? s->regs[offset / 4] : 0;
}

static void hisi_cipher_write(void *opaque, hwaddr offset, uint64_t val64,
                              unsigned size)
{
    HisiCipherState *s = HISI_CIPHER(opaque);
    uint32_t val = val64;
    uint32_t *r;

    if (offset >= CI_SIZE) {
        return;
    }
    r = &s->regs[offset / 4];
    switch (offset) {
    case CI_INT_RAW:
        *r &= ~val;
        ci_update_irq(s);
        return;
    case CI_INT_EN:
        *r = val;
        ci_update_irq(s);
        return;
    case CI_INT_STATUS:
        return;
    case CI_CH0_CFG:
        *r = (*r & ~1u) | (val & 1);
        if (val & 1) {
            *r |= 2;                /* busy until the timer */
            ci_kick(s);
        }
        return;
    }
    if (offset >= CI_CHAN(1) && offset < CI_CHAN(CI_CHANS)) {
        int ch = (offset - CI_CHAN(0)) / 0x80;

        switch ((offset - CI_CHAN(0)) % 0x80) {
        case CI_IBUF_CNT:
        case CI_OBUF_CNT:
            *r += val;
            ci_kick(s);
            return;
        case CI_IEMPTY_CNT:
        case CI_OFULL_CNT:
            *r -= MIN(val, *r);
            return;
        case CI_SRC_LST_SADDR:
            s->in_ptr[ch] = 0;
            *ci_chan(s, ch, CI_SRC_LST_RADDR) = 0;
            break;
        case CI_DEST_LST_SADDR:
            s->out_ptr[ch] = 0;
            *ci_chan(s, ch, CI_DEST_LST_RADDR) = 0;
            break;
        case CI_SRC_LST_RADDR:
        case CI_DEST_LST_RADDR:
            return;
        }
    }
    *r = val;
}

static const MemoryRegionOps hisi_cipher_ops = {
    .read = hisi_cipher_read,
    .write = hisi_cipher_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_cipher_reset(DeviceState *dev)
{
    HisiCipherState *s = HISI_CIPHER(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->iv, 0, sizeof(s->iv));
    memset(s->in_ptr, 0, sizeof(s->in_ptr));
    memset(s->out_ptr, 0, sizeof(s->out_ptr));
    s->pending = 0;
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void hisi_cipher_realize(DeviceState *dev, Error **errp)
{
    HisiCipherState *s = HISI_CIPHER(dev);

    if (s->efuse_key_hex) {
        const char *h = s->efuse_key_hex;

        if (strlen(h) != 32) {
            error_setg(errp, "hisi-cipher: efuse-key must be 32 hex digits");
            return;
        }
        for (int i = 0; i < 16; i++) {
            char byte[3] = { h[2 * i], h[2 * i + 1], 0 };
            unsigned long v;

            if (qemu_strtoul(byte, NULL, 16, &v)) {
                error_setg(errp, "hisi-cipher: efuse-key is not hex");
                return;
            }
            s->efuse_key[i] = v;
        }
    }
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ci_job_done, s);
}

static void hisi_cipher_init(Object *obj)
{
    HisiCipherState *s = HISI_CIPHER(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_cipher_ops, s,
                          TYPE_HISI_CIPHER, CI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_hisi_cipher = {
    .name = TYPE_HISI_CIPHER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, HisiCipherState, CI_SIZE / 4),
        VMSTATE_UINT8_2DARRAY(iv, HisiCipherState, CI_CHANS, 16),
        VMSTATE_UINT32_ARRAY(in_ptr, HisiCipherState, CI_CHANS),
        VMSTATE_UINT32_ARRAY(out_ptr, HisiCipherState, CI_CHANS),
        VMSTATE_UINT32(pending, HisiCipherState),
        VMSTATE_TIMER_PTR(timer, HisiCipherState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property hisi_cipher_properties[] = {
    DEFINE_PROP_STRING("efuse-key", HisiCipherState, efuse_key_hex),
};

static void hisi_cipher_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_cipher_reset);
    device_class_set_props(dc, hisi_cipher_properties);
    dc->realize = hisi_cipher_realize;
    dc->vmsd = &vmstate_hisi_cipher;
}

static const TypeInfo hisi_cipher_info = {
    .name          = TYPE_HISI_CIPHER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiCipherState),
    .instance_init = hisi_cipher_init,
    .class_init    = hisi_cipher_class_init,
};

static void hisi_cipher_register_types(void)
{
    type_register_static(&hisi_cipher_info);
}

type_init(hisi_cipher_register_types)
