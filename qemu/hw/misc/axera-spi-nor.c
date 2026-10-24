/*
 * Axera BL1's boot SPI NOR controller + flash: DW_SPI_BASE @ 0x01a00000
 * (maix_ax620e_sdk/boot/bl1/driver/spi/dw_spi_mst.c, driver/include/
 * dw_spi.h) -- the SAME DesignWare APB SSI IP as the SPI2 panel
 * controller hw/misc/axera-dw-spi.c already models, but at a different
 * instance/address and in a role where the loopback trick that works
 * for the panel is not enough: a SPI NOR read's response bytes must be
 * the real flash content at the requested address, not an echo of
 * whatever was written.
 *
 * Confirmed directly from dw_spi_mst.c's real dw_spi_xfer()/
 * spi_mem_exec_op(): each SPI NOR operation is two dw_spi_xfer() calls
 * -- a TX-only "cmd+addr+dummy" phase, then an RX-only "data" phase --
 * each of which independently does spi_enable_chip(0) (SSIENR=0),
 * reprograms CTRL0 (data frame size + SPI_TMOD_TO/_RO in bits 10-11)
 * and, for the RX phase only, CTRL1 (entries-1), then spi_enable_chip(1)
 * (SSIENR=1). That SSIENR 0->1 transition is a clean, SPI-controller-
 * register-local synchronisation point: no need to model the real
 * chip-select signal at all (real silicon asserts/deasserts it via a
 * GPIO2 pin, external_cs_manage(), entirely outside this controller's
 * own register space).
 *
 * BL1 only ever uses qmode=0 (drivers/spi/spinor.c's spinor_qmode,
 * never set otherwise before the first boot read): opcode 0x03
 * (SPINOR_OP_READ) + a 3-byte big-endian address, no dummy cycles --
 * the simplest possible SPI NOR read, well within what this model
 * needs to get right. RDSR (0x05) and READID (0x90) are also modelled
 * since BL1's spinor_init()/spinor_wait_till_ready() use them, but
 * always answer "not busy" / a fixed ID rather than tracking real
 * flash state machine timing -- there is no real program/erase
 * latency worth modelling for a read-only boot path.
 *
 * Confirmed real partition layout for this flash (AX620E's own real
 * deployed camera firmware, General_IPC_AX620E_A6...dump, via its own
 * saved U-Boot env): mtdparts=spi_nor.0:0x10000(spl),0x50000(boot),
 * 0x4F0000(romfs),0x640000(usr),0x20000(web),0x3F0000(custom),
 * 0x60000(mtd) -- SPL and the second signed image (DDRINIT, not ATF/
 * OPTEE -- this NOR profile has neither) are RSA-signed, U-Boot itself
 * is plain/unsigned.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AXERA_SPI_NOR "axera-spi-nor"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraSpiNorState, AXERA_SPI_NOR)

#define DW_SPI_CTRL0            0x00
#define DW_SPI_CTRL1            0x04
#define DW_SPI_SSIENR           0x08
#define DW_SPI_SER              0x10
#define DW_SPI_TXFLR            0x20
#define DW_SPI_RXFLR            0x24
#define DW_SPI_SR               0x28
#define DW_SPI_DR               0x60
#define DW_SPI_SIZE             0x100

#define SPI_DFS_OFFSET          0
#define SPI_DFS_MASK            0x1f
#define SPI_TMOD_OFFSET         10
#define SPI_TMOD_MASK           (0x3 << SPI_TMOD_OFFSET)
#define SPI_TMOD_TO             0x1     /* xmit only: cmd+addr phase */
#define SPI_TMOD_RO             0x2     /* recv only: data phase */

#define SR_BUSY                 BIT(0)
#define SR_TF_NOT_FULL          BIT(1)
#define SR_TF_EMPT              BIT(2)
#define SR_RF_NOT_EMPT          BIT(3)

#define SPINOR_OP_RDSR          0x05
#define SPINOR_OP_READID        0x90
#define SPINOR_OP_READ          0x03

struct AxeraSpiNorState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    char *file;
    uint8_t *flash;
    gsize flash_size;

    uint32_t mem[DW_SPI_SIZE / 4];
    bool ssienr;
    bool in_cmd_phase;
    uint8_t cmd_buf[4];
    unsigned int cmd_len;
    uint8_t latched_opcode;
    uint32_t latched_addr;

    uint8_t *rxq;
    unsigned int rxq_cap;
    unsigned int rxq_head;
    unsigned int rxq_len;
};

/*
 * BL1's own dw_spi_mst.c reads flash in up to 32768-byte chunks (CTRL1's
 * entries-1 field is 16 bits wide on the real DesignWare APB SSI IP, so
 * a single RX phase can legitimately ask for far more than any small
 * fixed-size buffer could hold) -- size the queue to the exact transfer
 * instead of capping it, matching what the real FIFO-plus-shift-register
 * behaves as from software's point of view.
 */
static void rxq_reset(AxeraSpiNorState *s, unsigned int needed)
{
    if (needed > s->rxq_cap) {
        s->rxq = g_realloc(s->rxq, needed);
        s->rxq_cap = needed;
    }
    s->rxq_head = 0;
    s->rxq_len = 0;
}

static void rxq_push(AxeraSpiNorState *s, uint8_t byte)
{
    s->rxq[(s->rxq_head + s->rxq_len) % s->rxq_cap] = byte;
    s->rxq_len++;
}

/*
 * SSIENR 0->1 while CTRL0's TMOD says "receive only": the real flash's
 * data phase is starting. CTRL1 already holds entries-1 for this exact
 * transfer (dw_spi_xfer() writes it, for the RX call only, between
 * SSIENR=0 and SSIENR=1 -- real register-write ordering, not assumed),
 * and CTRL0's DFS field gives the frame size BL1 always sets to 8 bits
 * for this driver. Stage the real answer now so DR reads just drain it,
 * the same shape axera-dw-spi.c's panel loopback already uses.
 */
static void axera_spi_nor_start_data_phase(AxeraSpiNorState *s)
{
    uint32_t ctrl0 = s->mem[DW_SPI_CTRL0 / 4];
    uint32_t ctrl1 = s->mem[DW_SPI_CTRL1 / 4];
    uint32_t dfs = (ctrl0 & SPI_DFS_MASK) + 1;
    uint32_t frame_bytes = dfs > 8 ? dfs / 8 : 1;
    uint32_t entries = ctrl1 + 1;
    uint32_t total = entries * frame_bytes;
    uint32_t i;

    rxq_reset(s, total);

    switch (s->latched_opcode) {
    case SPINOR_OP_READ:
        for (i = 0; i < total; i++) {
            uint32_t addr = s->latched_addr + i;

            rxq_push(s, addr < s->flash_size ? s->flash[addr] : 0xff);
        }
        break;
    case SPINOR_OP_RDSR:
        /* Never busy, always write-enabled-or-not as BL1 expects. */
        for (i = 0; i < total; i++) {
            rxq_push(s, 0x00);
        }
        break;
    case SPINOR_OP_READID:
        /*
         * Manufacturer/device ID a 0x90 query would return; BL1's own
         * spinor_init() only logs this rather than gating boot on a
         * specific match, so any plausible non-zero ID is sufficient
         * (GD25Q128, a common real fit already used as this project's
         * own default assumption elsewhere -- see hw/arm/anyka.c's
         * ak_flash_jedec() fallback).
         */
        for (i = 0; i < total; i++) {
            static const uint8_t id[] = { 0xc8, 0x40, 0x18 };

            rxq_push(s, i < sizeof(id) ? id[i] : 0x00);
        }
        break;
    default:
        for (i = 0; i < total; i++) {
            rxq_push(s, 0x00);
        }
        break;
    }
}

static void axera_spi_nor_ssienr_write(AxeraSpiNorState *s, uint32_t value)
{
    bool enable = (value & 1) != 0;

    if (enable && !s->ssienr) {
        uint32_t tmod = (s->mem[DW_SPI_CTRL0 / 4] & SPI_TMOD_MASK) >>
                        SPI_TMOD_OFFSET;

        if (tmod == SPI_TMOD_TO) {
            s->in_cmd_phase = true;
            s->cmd_len = 0;
        } else if (tmod == SPI_TMOD_RO) {
            s->in_cmd_phase = false;
            s->latched_opcode = s->cmd_len > 0 ? s->cmd_buf[0] : 0;
            s->latched_addr = 0;
            if (s->cmd_len >= 4) {
                s->latched_addr = (s->cmd_buf[1] << 16) |
                                  (s->cmd_buf[2] << 8) | s->cmd_buf[3];
            }
            axera_spi_nor_start_data_phase(s);
        }
    }
    s->ssienr = enable;
}

static uint64_t axera_spi_nor_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraSpiNorState *s = opaque;
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
        s->rxq_head = (s->rxq_head + 1) % s->rxq_cap;
        s->rxq_len--;
        return val;
    default:
        if (offset < DW_SPI_SIZE) {
            return s->mem[offset / 4];
        }
        return 0;
    }
}

static void axera_spi_nor_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    AxeraSpiNorState *s = opaque;

    switch (offset) {
    case DW_SPI_SSIENR:
        axera_spi_nor_ssienr_write(s, value);
        s->mem[offset / 4] = value;
        return;
    case DW_SPI_TXFLR:
    case DW_SPI_RXFLR:
    case DW_SPI_SR:
        /* Read-only hardware status; ignore writes like real silicon. */
        return;
    case DW_SPI_DR:
        if (s->in_cmd_phase && s->cmd_len < sizeof(s->cmd_buf)) {
            s->cmd_buf[s->cmd_len++] = value & 0xff;
        }
        return;
    default:
        if (offset < DW_SPI_SIZE) {
            s->mem[offset / 4] = value;
        }
        return;
    }
}

static const MemoryRegionOps axera_spi_nor_ops = {
    .read = axera_spi_nor_read,
    .write = axera_spi_nor_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_spi_nor_realize(DeviceState *dev, Error **errp)
{
    AxeraSpiNorState *s = AXERA_SPI_NOR(dev);

    if (s->file && s->file[0]) {
        if (!g_file_get_contents(s->file, (gchar **)&s->flash,
                                 &s->flash_size, NULL)) {
            error_setg(errp, "axera-spi-nor: could not read file '%s'",
                      s->file);
            return;
        }
    } else {
        s->flash_size = 0;
    }

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_spi_nor_ops, s,
                          TYPE_AXERA_SPI_NOR, DW_SPI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static const Property axera_spi_nor_properties[] = {
    DEFINE_PROP_STRING("file", AxeraSpiNorState, file),
};

static void axera_spi_nor_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_spi_nor_realize;
    device_class_set_props(dc, axera_spi_nor_properties);
}

static const TypeInfo axera_spi_nor_info = {
    .name = TYPE_AXERA_SPI_NOR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraSpiNorState),
    .class_init = axera_spi_nor_class_init,
};

static void axera_spi_nor_register_types(void)
{
    type_register_static(&axera_spi_nor_info);
}

type_init(axera_spi_nor_register_types)
