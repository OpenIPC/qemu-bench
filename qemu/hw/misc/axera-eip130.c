/*
 * Axera AX630C/AX620E EIP-130 AES offload.
 *
 * Modelled from the real open driver (drivers/crypto/axera/axera-eip130.c
 * in the onekvm/linux-6.18-ax620e tree), which documents the token
 * protocol precisely: a 64-word (256-byte) command token written to
 * mailbox 2 (offset 0x400), a doorbell bit in MBOX_CTRL (0x3f00), and a
 * completion word (status | sequence) read back from the same mailbox
 * after an interrupt. Firmware bootstrap (mailbox 1 at offset 0, FW_RAM
 * at 0x4000, MODULE_STATUS at 0x3fe0) is accepted unconditionally here --
 * this model has no real firmware blob to validate, and the guest driver
 * only checks the handshake bits, not firmware content.
 *
 * MBOX_CTRL is a set/ack register, not a plain store: INPUT/LINK/bit2
 * are "write 1 to set" (the driver's later plain `writel(MBOX_INPUT, …)`
 * would otherwise clobber the LINK bit raised earlier at firmware-link
 * time, which the driver relies on staying set); OUTPUT is "write 1 to
 * clear" (the driver's own ack-and-rearm pattern after reading a result).
 *
 * The actual AES operation is real (via QEMU's qcrypto_cipher_*), reading
 * the token's input/output DMA addresses and key/IV out of guest memory,
 * so Linux's dm-crypt/AF_ALG self-tests and real cipher traffic through
 * this driver produce correct ciphertext, not just a satisfied probe.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "crypto/cipher.h"
#include "hw/irq.h"
#include "hw/misc/axera-eip130.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "system/dma.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(AxeraEip130State, AXERA_EIP130)

#define EIP130_MMIO_SIZE  0x40000
#define MBOX1_OFF         0x0
#define MBOX2_OFF         0x400
#define TOKEN_WORDS       64
#define FW_RAM_OFF        0x4000
#define AIC_POL           0x3e00
#define AIC_TYPE          0x3e04
#define AIC_ENABLE        0x3e08
#define AIC_RAW           0x3e0c
#define AIC_ACK           0x3e10
#define IRQ_DONE          BIT(3)
#define MBOX_LOCKOUT      0x3f10
#define MBOX_CTRL         0x3f00
#define MBOX_INPUT        BIT(4)
#define MBOX_OUTPUT       BIT(5)
#define MBOX_LINK         BIT(6)
#define MODULE_STATUS     0x3fe0
#define FW_CRC_OK         BIT(9)
#define FW_WRITTEN        BIT(20)
#define FW_CHECKED        BIT(22)
#define FW_ACCEPTED       BIT(23)

struct AxeraEip130State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint8_t *mem;       /* mailbox 1/2 + FW_RAM backing storage */
    uint32_t aic_enable;
    uint32_t aic_raw;
    uint32_t mbox_ctrl;
};

static void axera_eip130_update_irq(AxeraEip130State *s)
{
    qemu_set_irq(s->irq, (s->aic_raw & s->aic_enable & IRQ_DONE) != 0);
}

static uint32_t token_word(const uint8_t *token, int idx)
{
    return ldl_le_p(token + idx * 4);
}

/*
 * AES ECB/CBC/CTR, exactly the three skcipher registrations the guest
 * driver exposes (ax_algs[]: mode 0 = ecb, 1 = cbc, 2 = ctr). The token
 * layout below is the guest driver's own ax_do_cipher() packing, read
 * back in the same order.
 */
static void axera_eip130_process_token(AxeraEip130State *s)
{
    uint8_t *token = s->mem + MBOX2_OFF;
    uint32_t length = token_word(token, 2);
    uint64_t in_addr = token_word(token, 3) | ((uint64_t)token_word(token, 4) << 32);
    uint64_t out_addr = token_word(token, 6) | ((uint64_t)token_word(token, 7) << 32);
    uint32_t ctrl = token_word(token, 11);
    int mode = (ctrl >> 4) & 0x3;
    int keylen = (((ctrl >> 16) & 0xffff) + 1) * 8;
    bool encrypt = (ctrl & BIT(15)) != 0;
    uint8_t key[32] = {};
    uint8_t iv[16] = {};
    g_autofree uint8_t *buf = NULL;
    QCryptoCipherAlgo algo;
    QCryptoCipherMode qmode;
    QCryptoCipher *cipher;
    Error *err = NULL;
    int i;

    if (length == 0 || length > EIP130_MMIO_SIZE) {
        goto complete;
    }

    for (i = 0; i < keylen / 4 && i < 8; i++) {
        stl_le_p(key + i * 4, token_word(token, 17 + i));
    }
    if (mode != 0) {
        for (i = 0; i < 4; i++) {
            stl_le_p(iv + i * 4, token_word(token, 13 + i));
        }
    }

    switch (keylen) {
    case 16: algo = QCRYPTO_CIPHER_ALGO_AES_128; break;
    case 24: algo = QCRYPTO_CIPHER_ALGO_AES_192; break;
    case 32: algo = QCRYPTO_CIPHER_ALGO_AES_256; break;
    default:
        goto complete;
    }
    switch (mode) {
    case 0: qmode = QCRYPTO_CIPHER_MODE_ECB; break;
    case 1: qmode = QCRYPTO_CIPHER_MODE_CBC; break;
    case 2: qmode = QCRYPTO_CIPHER_MODE_CTR; break;
    default:
        goto complete;
    }

    buf = g_malloc(length);
    if (dma_memory_read(&address_space_memory, in_addr, buf, length,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        goto complete;
    }

    cipher = qcrypto_cipher_new(algo, qmode, key, keylen, &err);
    if (!cipher) {
        error_free(err);
        goto complete;
    }
    if (qmode != QCRYPTO_CIPHER_MODE_ECB) {
        qcrypto_cipher_setiv(cipher, iv, sizeof(iv), &err);
    }
    /* CTR is its own inverse (XOR keystream); the guest driver always
     * requests "encrypt" from hardware for CTR regardless of direction. */
    if (encrypt || mode == 2) {
        qcrypto_cipher_encrypt(cipher, buf, buf, length, &err);
    } else {
        qcrypto_cipher_decrypt(cipher, buf, buf, length, &err);
    }
    qcrypto_cipher_free(cipher);

    dma_memory_write(&address_space_memory, out_addr, buf, length,
                     MEMTXATTRS_UNSPECIFIED);

complete:
    /* Completion word: low 16 bits must echo the token's sequence number
     * (token word 0, low 16 bits); no FATAL_ERROR bit -- happy path only.
     */
    stl_le_p(s->mem + MBOX2_OFF, token_word(token, 0) & 0xffff);
    memset(key, 0, sizeof(key));
    memset(iv, 0, sizeof(iv));
}

static void axera_eip130_mbox_ctrl_write(AxeraEip130State *s, uint32_t val)
{
    if (val & MBOX_INPUT) {
        axera_eip130_process_token(s);
        s->mbox_ctrl &= ~MBOX_INPUT;
        s->mbox_ctrl |= MBOX_OUTPUT;
        s->aic_raw |= IRQ_DONE;
        axera_eip130_update_irq(s);
    }
    if (val & MBOX_OUTPUT) {
        s->mbox_ctrl &= ~MBOX_OUTPUT;
    }
    /* LINK and the firmware-bootstrap mailbox-select bit (BIT(2)) are
     * sticky "write 1 to set" bits the driver never clears. */
    s->mbox_ctrl |= val & (MBOX_LINK | BIT(2) | BIT(0) | BIT(3));
}

static uint64_t axera_eip130_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraEip130State *s = opaque;

    switch (offset) {
    case AIC_ENABLE:
        return s->aic_enable;
    case AIC_RAW:
        return s->aic_raw;
    case MBOX_CTRL:
        return s->mbox_ctrl;
    case MODULE_STATUS:
        /* Always ready, CRC good, firmware accepted, never busy/fatal --
         * this model has no real firmware image to validate. */
        return FW_CRC_OK | FW_CHECKED | FW_ACCEPTED;
    default:
        if (offset + size <= EIP130_MMIO_SIZE) {
            uint64_t val = 0;
            memcpy(&val, s->mem + offset, size);
            return val;
        }
        return 0;
    }
}

static void axera_eip130_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    AxeraEip130State *s = opaque;

    switch (offset) {
    case AIC_POL:
    case AIC_TYPE:
        break;
    case AIC_ENABLE:
        s->aic_enable = value;
        axera_eip130_update_irq(s);
        break;
    case AIC_ACK:
        s->aic_raw &= ~value;
        axera_eip130_update_irq(s);
        break;
    case MBOX_CTRL:
        axera_eip130_mbox_ctrl_write(s, value);
        break;
    case MBOX_LOCKOUT:
        break;
    case MODULE_STATUS:
        /* The driver's only write here is FW_WRITTEN, meaning "I've
         * written the firmware image, please validate it" -- accept
         * unconditionally (see the file comment). */
        break;
    default:
        if (offset + size <= EIP130_MMIO_SIZE) {
            memcpy(s->mem + offset, &value, size);
        }
        break;
    }
}

static const MemoryRegionOps axera_eip130_ops = {
    .read = axera_eip130_read,
    .write = axera_eip130_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_eip130_reset(DeviceState *dev)
{
    AxeraEip130State *s = AXERA_EIP130(dev);

    memset(s->mem, 0, EIP130_MMIO_SIZE);
    s->aic_enable = 0;
    s->aic_raw = 0;
    s->mbox_ctrl = 0;
    qemu_set_irq(s->irq, 0);
}

static void axera_eip130_realize(DeviceState *dev, Error **errp)
{
    AxeraEip130State *s = AXERA_EIP130(dev);

    s->mem = g_malloc0(EIP130_MMIO_SIZE);
    memory_region_init_io(&s->iomem, OBJECT(s), &axera_eip130_ops, s,
                          TYPE_AXERA_EIP130, EIP130_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void axera_eip130_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_eip130_realize;
    device_class_set_legacy_reset(dc, axera_eip130_reset);
}

static const TypeInfo axera_eip130_info = {
    .name = TYPE_AXERA_EIP130,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraEip130State),
    .class_init = axera_eip130_class_init,
};

static void axera_eip130_register_types(void)
{
    type_register_static(&axera_eip130_info);
}

type_init(axera_eip130_register_types)
