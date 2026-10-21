/*
 * Novatek NA51055 / NA51089 SDIO host, with an SD card on its bus.
 *
 * The kernel's drivers/mmc/host/na51055_mmc{,host,reg}.{c,h} and U-Boot's
 * drivers/mmc/nvt_ivot_mmc{,reg}.{c,h} drive it:
 *
 *   0x00 CMD        [5:0] index, bit 6 NEED_RSP, bit 7 LONG_RSP (R2,
 *                   which the drivers send without NEED_RSP), bit 9
 *                   CMD_EN starts the command (and its data), bit 10
 *                   SDC_RST resets the state machine (clears when done)
 *   0x04 ARG
 *   0x08..0x14 RSP0..3  a short response in RSP0; R2 with bits 127..96
 *                   in RSP3 (the drivers read RSP3 into resp[0])
 *   0x18 RSP_CMD    the responding command's index
 *   0x1C DATA_CTRL  bit 6 DATA_EN, [31:16] block size
 *   0x28 STATUS     bit 2 RSP_TIMEOUT, 3 DATA_TIMEOUT, 4 RSP_CRC_OK, 5
 *                   DATA_CRC_OK, 6 CMD_SENT, 7 DATA_END; write 1 to clear
 *   0x30 INT_MASK   the same bits; SPI 30 / 31 is STATUS & INT_MASK
 *   0x40 BUS_STATUS bit 0 set when the card is not busy (sdiohost_getrdy)
 *   0x4C PHY        bit 0 PHY_SW_RST (clears when done)
 *   0x100 DATA_PORT PIO data, little-endian words
 *   0x104 DATA_LENGTH  the transfer's total bytes
 *   0x108 FIFO_STS  [5:0] words waiting, bit 8 empty, bit 9 full (16)
 *   0x10C FIFO_CTRL bit 0 FIFO_EN, bit 1 DMA (else PIO), bit 2 write
 *   0x110 DMA       a descriptor table, or with 0x1FC bit 0 the buffer
 *   0x114 DMA_CURR
 *   0x1FC           bit 0: plain DMA to the address in 0x110
 *
 * Both drivers set the data phase up (DATA_LENGTH, FIFO_CTRL, DATA_EN,
 * then FIFO_EN) before writing CMD_EN, so the data moves with the
 * command.  By DMA it completes at once: CMD_SENT, RSP_CRC_OK, DATA_END
 * and DATA_CRC_OK together, which the kernel's IRQ handler takes in that
 * order.  The Linux driver (SDIO_SCATTER_DMA) hands a table of 3-word
 * descriptors -- valid / end / act=2, length, address -- 128 at most;
 * U-Boot writes 0x1FC = 1 and a buffer address instead.  Linux never
 * clears 0x1FC: the clock framework's reset pulse when it prepares the
 * clock (nvt-cg "sys-reset", CG 0x84 bit 2 / 3) resets the block, as on
 * silicon, and that line is wired to the "reset" input here.
 *
 * PIO (buffers not cache-line aligned, such as the 8-byte SCR) leaves the
 * data to the driver after the command: it drains or fills DATA_PORT in
 * its command-done handler, 16 words at a time when FIFO_STS says full or
 * empty, then waits for DATA_END, raised with the last word.
 *
 * An empty slot answers every command that wants a response with
 * CMD_SENT | RSP_TIMEOUT, which is what U-Boot's probe for an update
 * image expects.  The card itself is QEMU's sd-card, on bus "sd-bus".
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sd/sd.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/dma.h"

#define TYPE_NVT_SDIO "nvt-sdio"
OBJECT_DECLARE_SIMPLE_TYPE(NvtSdioState, NVT_SDIO)

#define TYPE_NVT_SDIO_BUS "nvt-sdio-bus"

#define SDIO_REGS_SIZE      0x200
#define SDIO_CMD            0x00
#define SDIO_ARG            0x04
#define SDIO_RSP0           0x08
#define SDIO_RSP_CMD        0x18
#define SDIO_DATA_CTRL      0x1C
#define SDIO_STATUS         0x28
#define SDIO_INT_MASK       0x30
#define SDIO_BUS_STATUS     0x40
#define SDIO_PHY            0x4C
#define SDIO_DATA_PORT      0x100
#define SDIO_DATA_LENGTH    0x104
#define SDIO_FIFO_STS       0x108
#define SDIO_FIFO_CTRL      0x10C
#define SDIO_DMA_ADDR       0x110
#define SDIO_DMA_CURR       0x114
#define SDIO_DMA_MODE       0x1FC

#define CMD_IDX             0x3F
#define CMD_NEED_RSP        (1u << 6)
#define CMD_LONG_RSP        (1u << 7)
#define CMD_EN              (1u << 9)
#define CMD_SDC_RST         (1u << 10)
#define DATA_CTRL_EN        (1u << 6)
#define STS_RSP_TIMEOUT     (1u << 2)
#define STS_DATA_TIMEOUT    (1u << 3)
#define STS_RSP_CRC_OK      (1u << 4)
#define STS_DATA_CRC_OK     (1u << 5)
#define STS_CMD_SENT        (1u << 6)
#define STS_DATA_END        (1u << 7)
#define STS_DMA_ERROR       (1u << 19)
#define PHY_SW_RST          (1u << 0)
#define FIFO_EN             (1u << 0)
#define FIFO_DMA            (1u << 1)
#define FIFO_WRITE          (1u << 2)
#define FIFO_EMPTY          (1u << 8)
#define FIFO_FULL           (1u << 9)
#define FIFO_WORDS          16
#define DMA_MODE_PLAIN      (1u << 0)
#define BUS_READY           0x0F03      /* not busy, CMD and DAT0-3 high */

#define DES_VALID           (1u << 0)
#define DES_END             (1u << 1)
#define DES_ACT_SHIFT       4
#define DES_ACT_TRAN        2
#define DES_SIZE            12
#define DES_MAX             128
#define LEN_MASK            0x3FFFFFF

struct NvtSdioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    SDBus sdbus;
    uint32_t regs[SDIO_REGS_SIZE / 4];
    uint32_t pio_left;          /* PIO bytes still to go through DATA_PORT */
};

static uint32_t *reg(NvtSdioState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_sdio_update_irq(NvtSdioState *s)
{
    qemu_set_irq(s->irq, *reg(s, SDIO_STATUS) & *reg(s, SDIO_INT_MASK));
}

static void nvt_sdio_data_done(NvtSdioState *s, uint32_t sts)
{
    s->pio_left = 0;
    *reg(s, SDIO_FIFO_CTRL) &= ~FIFO_EN;
    *reg(s, SDIO_DATA_CTRL) &= ~DATA_CTRL_EN;
    *reg(s, SDIO_STATUS) |= sts;
}

/*
 * Move len bytes between the card and guest memory at addr; false if guest
 * memory failed, which stops there (no garbage reaches the card).
 */
static bool nvt_sdio_dma_chunk(NvtSdioState *s, dma_addr_t addr,
                               uint32_t len, bool write)
{
    uint8_t buf[512];
    uint32_t n;
    bool ok = true;

    while (len && ok) {
        n = MIN(len, sizeof(buf));
        if (write) {
            ok = dma_memory_read(&address_space_memory, addr, buf, n,
                                 MEMTXATTRS_UNSPECIFIED) == MEMTX_OK;
            if (ok) {
                sdbus_write_data(&s->sdbus, buf, n);
            }
        } else {
            sdbus_read_data(&s->sdbus, buf, n);
            ok = dma_memory_write(&address_space_memory, addr, buf, n,
                                  MEMTXATTRS_UNSPECIFIED) == MEMTX_OK;
        }
        addr += n;
        len -= n;
    }
    *reg(s, SDIO_DMA_CURR) = addr;
    if (!ok) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-sdio: DMA to bad address "
                      "0x%" PRIx64 "\n", (uint64_t)addr);
    }
    return ok;
}

/* True when all len bytes moved. */
static bool nvt_sdio_dma(NvtSdioState *s, uint32_t len, bool write)
{
    dma_addr_t des = *reg(s, SDIO_DMA_ADDR);
    uint32_t w[3], n;
    int i;

    if (*reg(s, SDIO_DMA_MODE) & DMA_MODE_PLAIN) {
        return nvt_sdio_dma_chunk(s, des, len, write);
    }
    for (i = 0; i < DES_MAX && len; i++, des += DES_SIZE) {
        if (dma_memory_read(&address_space_memory, des, w, sizeof(w),
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            break;
        }
        w[0] = le32_to_cpu(w[0]);
        if (!(w[0] & DES_VALID)) {
            break;
        }
        if (((w[0] >> DES_ACT_SHIFT) & 3) == DES_ACT_TRAN) {
            n = MIN(len, le32_to_cpu(w[1]) & LEN_MASK);
            if (!nvt_sdio_dma_chunk(s, le32_to_cpu(w[2]), n, write)) {
                return false;
            }
            len -= n;
        }
        if (w[0] & DES_END) {
            break;
        }
    }
    if (len) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-sdio: descriptors end 0x%x "
                      "bytes short of DATA_LENGTH\n", len);
    }
    return !len;
}

/* The data phase of the command just sent, when one was set up. */
static uint32_t nvt_sdio_data(NvtSdioState *s)
{
    uint32_t fifo = *reg(s, SDIO_FIFO_CTRL);
    uint32_t len = *reg(s, SDIO_DATA_LENGTH) & LEN_MASK;
    bool write = fifo & FIFO_WRITE;

    if (!(fifo & FIFO_EN) || !(*reg(s, SDIO_DATA_CTRL) & DATA_CTRL_EN) ||
        !len) {
        return 0;
    }
    if (write ? !sdbus_receive_ready(&s->sdbus) :
                !sdbus_data_ready(&s->sdbus)) {
        nvt_sdio_data_done(s, STS_DATA_TIMEOUT);
        return 0;
    }
    if (!(fifo & FIFO_DMA)) {
        s->pio_left = len;
        return 0;
    }
    if (!nvt_sdio_dma(s, len, write)) {
        /* What both drivers take as a failed transfer. */
        nvt_sdio_data_done(s, 0);
        return STS_DATA_TIMEOUT | STS_DMA_ERROR;
    }
    nvt_sdio_data_done(s, 0);
    return STS_DATA_END | STS_DATA_CRC_OK;
}

static void nvt_sdio_command(NvtSdioState *s)
{
    uint32_t cmd = *reg(s, SDIO_CMD);
    SDRequest req = {
        .cmd = cmd & CMD_IDX,
        .arg = *reg(s, SDIO_ARG),
    };
    uint8_t rsp[16];
    uint32_t sts = STS_CMD_SENT;
    size_t rlen;
    int i;

    rlen = sdbus_do_command(&s->sdbus, &req, rsp, sizeof(rsp));
    if (cmd & (CMD_NEED_RSP | CMD_LONG_RSP)) {
        if (rlen == 16) {
            for (i = 0; i < 4; i++) {
                *reg(s, SDIO_RSP0 + 4 * i) = ldl_be_p(rsp + 12 - 4 * i);
            }
        } else if (rlen == 4) {
            *reg(s, SDIO_RSP0) = ldl_be_p(rsp);
        }
        if (!rlen) {
            /* No data follows a command the card did not take. */
            nvt_sdio_data_done(s, sts | STS_RSP_TIMEOUT);
            return;
        }
        *reg(s, SDIO_RSP_CMD) = req.cmd;
        sts |= STS_RSP_CRC_OK;
    }
    sts |= nvt_sdio_data(s);
    *reg(s, SDIO_STATUS) |= sts;
}

static uint32_t nvt_sdio_fifo_sts(NvtSdioState *s)
{
    uint32_t words = 0;

    if (s->pio_left && !(*reg(s, SDIO_FIFO_CTRL) & FIFO_WRITE)) {
        words = MIN(DIV_ROUND_UP(s->pio_left, 4), FIFO_WORDS);
    }
    return words | (words ? 0 : FIFO_EMPTY) |
           (words == FIFO_WORDS ? FIFO_FULL : 0);
}

static uint32_t nvt_sdio_pio(NvtSdioState *s, uint32_t val, bool write)
{
    uint8_t b[4] = { 0 };
    uint32_t n;

    if (!s->pio_left ||
        write != !!(*reg(s, SDIO_FIFO_CTRL) & FIFO_WRITE)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "nvt-sdio: DATA_PORT %s with no transfer\n",
                      write ? "write" : "read");
        return 0;
    }
    n = MIN(s->pio_left, 4);
    if (write) {
        stl_le_p(b, val);
        sdbus_write_data(&s->sdbus, b, n);
    } else {
        sdbus_read_data(&s->sdbus, b, n);
    }
    s->pio_left -= n;
    if (!s->pio_left) {
        nvt_sdio_data_done(s, STS_DATA_END | STS_DATA_CRC_OK);
        nvt_sdio_update_irq(s);
    }
    return ldl_le_p(b);
}

static uint64_t nvt_sdio_read(void *opaque, hwaddr off, unsigned size)
{
    NvtSdioState *s = opaque;

    switch (off) {
    case SDIO_BUS_STATUS:
        return BUS_READY;
    case SDIO_DATA_PORT:
        return nvt_sdio_pio(s, 0, false);
    case SDIO_FIFO_STS:
        return nvt_sdio_fifo_sts(s);
    default:
        return off < SDIO_REGS_SIZE ? *reg(s, off) : 0;
    }
}

static void nvt_sdio_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    NvtSdioState *s = opaque;

    switch (off) {
    case SDIO_CMD:
        *reg(s, off) = val & ~(CMD_EN | CMD_SDC_RST);
        if (val & CMD_SDC_RST) {
            nvt_sdio_data_done(s, 0);       /* disarms any data phase */
            *reg(s, SDIO_STATUS) = 0;
        }
        if (val & CMD_EN) {
            nvt_sdio_command(s);
        }
        nvt_sdio_update_irq(s);
        break;
    case SDIO_STATUS:
        *reg(s, off) &= ~val;
        nvt_sdio_update_irq(s);
        break;
    case SDIO_INT_MASK:
        *reg(s, off) = val;
        nvt_sdio_update_irq(s);
        break;
    case SDIO_PHY:
        *reg(s, off) = val & ~PHY_SW_RST;
        break;
    case SDIO_DATA_PORT:
        nvt_sdio_pio(s, val, true);
        break;
    case SDIO_FIFO_CTRL:
        *reg(s, off) = val;
        if (!(val & FIFO_EN)) {
            s->pio_left = 0;        /* resetdata abandons a transfer */
        }
        break;
    case SDIO_FIFO_STS:
    case SDIO_BUS_STATUS:
        break;
    default:
        if (off < SDIO_REGS_SIZE) {
            *reg(s, off) = val;
        }
        break;
    }
}

static const MemoryRegionOps nvt_sdio_ops = {
    .read = nvt_sdio_read,
    .write = nvt_sdio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_sdio_reset(DeviceState *dev)
{
    NvtSdioState *s = NVT_SDIO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pio_left = 0;
    qemu_irq_lower(s->irq);
}

/* CG's SYS_RESET bit for this host: held in reset while high. */
static void nvt_sdio_reset_in(void *opaque, int n, int level)
{
    if (level) {
        nvt_sdio_reset(DEVICE(opaque));
    }
}

static void nvt_sdio_init(Object *obj)
{
    NvtSdioState *s = NVT_SDIO(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_sdio_ops, s, TYPE_NVT_SDIO,
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), nvt_sdio_reset_in, "reset", 1);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_NVT_SDIO_BUS, DEVICE(obj),
              "sd-bus");
}

static const VMStateDescription vmstate_nvt_sdio = {
    .name = TYPE_NVT_SDIO,
    .version_id = 2,
    .minimum_version_id = 1,        /* the empty-slot stub's state */
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtSdioState, SDIO_REGS_SIZE / 4),
        VMSTATE_UINT32_V(pio_left, NvtSdioState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_sdio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_sdio_reset);
    dc->vmsd = &vmstate_nvt_sdio;
}

static const TypeInfo nvt_sdio_types[] = {
    {
        .name = TYPE_NVT_SDIO,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(NvtSdioState),
        .instance_init = nvt_sdio_init,
        .class_init = nvt_sdio_class_init,
    }, {
        .name = TYPE_NVT_SDIO_BUS,
        .parent = TYPE_SD_BUS,
        .instance_size = sizeof(SDBus),
    },
};

DEFINE_TYPES(nvt_sdio_types)
