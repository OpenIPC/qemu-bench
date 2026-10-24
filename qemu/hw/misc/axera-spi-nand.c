/*
 * Axera BL1's boot SPI NAND flash, modelled behind the same
 * DesignWare APB SSI controller shape as hw/misc/axera-spi-nor.c (see
 * that file's own comment for the controller-register side of this,
 * shared verbatim here: SSIENR 0->1 transitions, CTRL0's TMOD field,
 * chip-select handled off-controller via a GPIO BL1 never asks this
 * device about), but implementing the standard SPI NAND command set
 * (JEDEC-ish, not Axera-specific -- the same opcodes practically every
 * SPI NAND part uses) instead of NOR's flat linear read.
 *
 * Confirmed from the real vendor BL1 source
 * (maix_ax620e_sdk/boot/bl1/driver/spi/spinand.c,
 * driver/include/spinand.h) for the AX620E/AX630C/AX620Q family, which
 * shares this exact protocol with AX620U per the real AX620U
 * devicetree extracted from a deployed camera firmware dump
 * (openipc-linux-ax620u's ax620u.dts: spi-master@0x08000000,
 * compatible = "snps,dw-ssi", a child spi_nand@0 node) -- no AX620U
 * BL1 *source* exists in this vendor SDK at all (a different, distinct
 * SoC, "axera,ax620x" not "axera,ax620e"), so this model's grounding
 * for AX620U specifically is the real devicetree's controller identity
 * plus the real NAND part number a saved U-Boot env string names
 * (mx35lf1ge4ab, a Macronix MX35LF1GE4AB -- used for a plausible JEDEC
 * ID below), not a byte-traced real boot the way axera-spi-nor.c's
 * AX620E support was.
 *
 * Real read sequence (drivers/spi/spinand.c's spinand_read_page()):
 * PAGE_READ (0x13, 3-byte row number -- already divided out of the
 * logical address by the caller, not a byte address) loads a whole
 * 2048-byte page into the chip's internal cache and sets the busy bit
 * while it does; BL1 polls GET_FEATURE(REG_STATUS=0xc0) until busy
 * clears (spinand_wait()); READ_FROM_CACHE (0x03, 2-byte column, 1
 * dummy byte) then shifts the requested bytes out of that cache. This
 * model never goes busy (no real program/erase timing worth
 * modelling for a read-only boot path) and serves real file content
 * for the data phase, backed by a "file" property the same way
 * axera-spi-nor.c is.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AXERA_SPI_NAND "axera-spi-nand"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraSpiNandState, AXERA_SPI_NAND)

#define DW_SPI_CTRL0            0x00
#define DW_SPI_CTRL1            0x04
#define DW_SPI_SSIENR           0x08
#define DW_SPI_TXFLR            0x20
#define DW_SPI_RXFLR            0x24
#define DW_SPI_SR               0x28
#define DW_SPI_DR               0x60
#define DW_SPI_SIZE             0x100

#define SR_TF_NOT_FULL          BIT(1)
#define SR_TF_EMPT              BIT(2)
#define SR_RF_NOT_EMPT          BIT(3)

#define SPINAND_PAGE_SIZE       2048
#define SPINAND_OOB_SIZE        64

#define OP_RESET                0xff
#define OP_WREN                 0x06
#define OP_WRDI                 0x04
#define OP_READID               0x9f
#define OP_GET_FEATURE          0x0f
#define OP_SET_FEATURE          0x1f
#define OP_PAGE_READ            0x13
#define OP_READ_FROM_CACHE      0x03

#define REG_STATUS              0xc0

#define AXERA_SPI_NAND_RXQ_MAX  4096

struct AxeraSpiNandState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    char *file;
    uint8_t *flash;
    gsize flash_size;

    uint32_t mem[DW_SPI_SIZE / 4];
    bool ssienr;
    uint8_t cmd_buf[8];
    unsigned int cmd_len;
    int pending_set_feature_addr;      /* -1 when no SET_FEATURE pending */
    uint32_t loaded_row;

    uint8_t rxq[AXERA_SPI_NAND_RXQ_MAX];
    unsigned int rxq_head;
    unsigned int rxq_len;
};

static void rxq_reset(AxeraSpiNandState *s)
{
    s->rxq_head = 0;
    s->rxq_len = 0;
}

static void rxq_push(AxeraSpiNandState *s, uint8_t byte)
{
    if (s->rxq_len < AXERA_SPI_NAND_RXQ_MAX) {
        s->rxq[(s->rxq_head + s->rxq_len) % AXERA_SPI_NAND_RXQ_MAX] = byte;
        s->rxq_len++;
    }
}

/*
 * How many total cmd+addr(+dummy) bytes each supported opcode needs
 * before it either completes (no-data ops) or hands off to a data
 * phase (handled by the caller once this many bytes have arrived).
 */
static int axera_spi_nand_cmd_len(uint8_t opcode)
{
    switch (opcode) {
    case OP_RESET:
    case OP_WREN:
    case OP_WRDI:
        return 1;
    case OP_READID:            /* cmd + 1 dummy byte */
    case OP_GET_FEATURE:       /* cmd + 1 reg addr byte */
    case OP_SET_FEATURE:       /* cmd + 1 reg addr byte */
        return 2;
    case OP_PAGE_READ:         /* cmd + 3-byte row */
    case OP_READ_FROM_CACHE:   /* cmd + 2-byte column + 1 dummy byte */
        return 4;
    default:
        return 1;
    }
}

static void axera_spi_nand_latch(AxeraSpiNandState *s)
{
    uint8_t opcode = s->cmd_buf[0];
    uint32_t column, i;

    rxq_reset(s);

    switch (opcode) {
    case OP_PAGE_READ:
        s->loaded_row = (s->cmd_buf[1] << 16) | (s->cmd_buf[2] << 8) |
                        s->cmd_buf[3];
        break;
    case OP_SET_FEATURE:
        s->pending_set_feature_addr = s->cmd_buf[1];
        break;
    case OP_GET_FEATURE:
        /* Never busy, no uncorrectable ECC -- a read-only boot never
         * needs to see anything else out of this register. */
        rxq_push(s, 0x00);
        break;
    case OP_READID:
        {
            /* Macronix MX35LF1GE4AB (mx35lf1ge4ab, confirmed from a
             * real AX620U camera's own saved U-Boot env string) --
             * manufacturer 0xc2, a plausible device ID since BL1 only
             * logs this rather than gating boot on an exact match. */
            static const uint8_t id[] = { 0xc2, 0x12 };

            for (i = 0; i < sizeof(id); i++) {
                rxq_push(s, id[i]);
            }
        }
        break;
    case OP_READ_FROM_CACHE:
        column = (s->cmd_buf[1] << 8) | s->cmd_buf[2];
        for (i = 0; i < 4096; i++) {
            uint32_t page_off = column + i;

            if (page_off >= SPINAND_PAGE_SIZE) {
                rxq_push(s, 0xff);     /* OOB: no bad-block marker */
                continue;
            }
            uint64_t off = (uint64_t)s->loaded_row * SPINAND_PAGE_SIZE +
                           page_off;

            rxq_push(s, off < s->flash_size ? s->flash[off] : 0xff);
        }
        break;
    default:
        break;
    }
}

static void axera_spi_nand_ssienr_write(AxeraSpiNandState *s, uint32_t value)
{
    bool enable = (value & 1) != 0;

    if (enable && !s->ssienr) {
        s->cmd_len = 0;
    }
    s->ssienr = enable;
}

static uint64_t axera_spi_nand_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    AxeraSpiNandState *s = opaque;
    uint8_t val;

    switch (offset) {
    case DW_SPI_TXFLR:
        return 0;
    case DW_SPI_RXFLR:
        return s->rxq_len;
    case DW_SPI_SR:
        return SR_TF_NOT_FULL | SR_TF_EMPT |
               (s->rxq_len ? SR_RF_NOT_EMPT : 0);
    case DW_SPI_DR:
        if (s->rxq_len == 0) {
            return 0;
        }
        val = s->rxq[s->rxq_head];
        s->rxq_head = (s->rxq_head + 1) % AXERA_SPI_NAND_RXQ_MAX;
        s->rxq_len--;
        return val;
    default:
        if (offset < DW_SPI_SIZE) {
            return s->mem[offset / 4];
        }
        return 0;
    }
}

static void axera_spi_nand_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    AxeraSpiNandState *s = opaque;

    switch (offset) {
    case DW_SPI_SSIENR:
        axera_spi_nand_ssienr_write(s, value);
        s->mem[offset / 4] = value;
        return;
    case DW_SPI_TXFLR:
    case DW_SPI_RXFLR:
    case DW_SPI_SR:
        return;
    case DW_SPI_DR:
        if (s->pending_set_feature_addr >= 0) {
            /* The feature value byte: consume it, nothing a read-only
             * boot needs this model to actually honour. */
            s->pending_set_feature_addr = -1;
            return;
        }
        if (s->cmd_len < sizeof(s->cmd_buf)) {
            s->cmd_buf[s->cmd_len++] = value & 0xff;
        }
        if (s->cmd_len > 0 &&
            s->cmd_len == (unsigned)axera_spi_nand_cmd_len(s->cmd_buf[0])) {
            axera_spi_nand_latch(s);
            s->cmd_len = 0;
        }
        return;
    default:
        if (offset < DW_SPI_SIZE) {
            s->mem[offset / 4] = value;
        }
        return;
    }
}

static const MemoryRegionOps axera_spi_nand_ops = {
    .read = axera_spi_nand_read,
    .write = axera_spi_nand_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_spi_nand_realize(DeviceState *dev, Error **errp)
{
    AxeraSpiNandState *s = AXERA_SPI_NAND(dev);

    s->pending_set_feature_addr = -1;

    if (s->file && s->file[0]) {
        if (!g_file_get_contents(s->file, (gchar **)&s->flash,
                                 &s->flash_size, NULL)) {
            error_setg(errp, "axera-spi-nand: could not read file '%s'",
                      s->file);
            return;
        }
    } else {
        s->flash_size = 0;
    }

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_spi_nand_ops, s,
                          TYPE_AXERA_SPI_NAND, DW_SPI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static const Property axera_spi_nand_properties[] = {
    DEFINE_PROP_STRING("file", AxeraSpiNandState, file),
};

static void axera_spi_nand_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_spi_nand_realize;
    device_class_set_props(dc, axera_spi_nand_properties);
}

static const TypeInfo axera_spi_nand_info = {
    .name = TYPE_AXERA_SPI_NAND,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraSpiNandState),
    .class_init = axera_spi_nand_class_init,
};

static void axera_spi_nand_register_types(void)
{
    type_register_static(&axera_spi_nand_info);
}

type_init(axera_spi_nand_register_types)
