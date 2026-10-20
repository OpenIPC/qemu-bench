/*
 * Novatek NA51055 / NA51089 SPI NOR / NAND flash controller ("NAND
 * controller" in the SDK), with the boot SPI NOR behind it.
 *
 * drivers/mtd/nvt_flash_spi/nvt_flash_spi_{reg,int}.h, nvt_flash_spi.c,
 * nvt_flash_spi_cmd.c and drivers/mtd/spiflash/spinor.c; U-Boot carries the
 * same driver.  The controller is an operation sequencer: software loads
 * COMMAND (up to four command bytes), ROWADDR (the flash address, sent
 * MSB first in MODULE0.row_addr + 1 bytes) and DATALEN, then writes an
 * operation into CTRL0[5:0] together with OPER_EN:
 *
 *   0x05 BLK_ERSE    cmd 1, address, deselect, then cmd 2 (RDSR) polled
 *                    until STATUS_CHECK's masked bits match
 *   0x06 READ_ID     cmd 1, DATALEN bytes into the FIFO
 *   0x07 CMD         cmd 1
 *   0x0B CMD_STS     cmd 1, one byte into CTRL_STS[7:0]
 *   0x0C CMD_ADDR    cmd 1, address
 *   0x18 READ_N      DATALEN bytes, to the FIFO (PIO) or DMASTART (DMA)
 *   0x19 WRITE_N     DATALEN bytes, from the FIFO or DMASTART
 *   0x1D AUTO_STATUS cmd 1 polled like BLK_ERSE's cmd 2, byte in CTRL_STS
 *   0x1E DUMMY_CLOCK DUMMY_CLOCK_NUM + 1 clocks, as whole bytes
 *
 * Each completes with CTRL_STS bit 12, an interrupt on SPI 29 with
 * INTMASK bit 12, which the driver waits for after every operation; its
 * ISR writes the status back to clear it.  Chip select is the
 * controller's for each operation, or software's: SPI_CFG bit 1 selects
 * manual mode and bit 4 then drives CS (0 asserted), which is how a read
 * strings CMD_ADDR, DUMMY_CLOCK and READ_N into one transfer.  The PIO
 * FIFO holds 16 words; FIFO_STS reports the count, full and empty.  A PIO
 * read streams through a 4 KiB buffer and completes with its last byte; a
 * PIO write completes when the data port has taken DATALEN bytes, at once
 * for none.
 *
 * The flash is a generic SPI NOR answering with JEDEC ID "jedec"; erases
 * and programs complete at once and are written through to flash-file.
 * SPI NAND (the 566 boards) is not modelled yet.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/blocker.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "hw/ssi/nvt-flash.h"

#define REG_MODULE0     0x00
#define REG_DLL1        0x08
#define REG_SPI_CFG     0x14
#define REG_CTRL0       0x20
#define REG_COMMAND     0x34
#define REG_ROWADDR     0x3C
#define REG_INTMASK     0x44
#define REG_CTRL_STS    0x48
#define REG_STS_CHECK   0xBC
#define REG_DUMMY_CLK   0xC0
#define REG_DATAPORT    0x100
#define REG_DATALEN     0x104
#define REG_FIFO_STS    0x108
#define REG_FIFO_CTRL   0x10C
#define REG_DMASTART    0x110
#define REG_DMACURRENT  0x114

#define OP_BLK_ERSE     0x05
#define OP_READ_ID      0x06
#define OP_CMD          0x07
#define OP_CMD_STS      0x0B
#define OP_CMD_ADDR     0x0C
#define OP_READ_N       0x18
#define OP_WRITE_N      0x19
#define OP_AUTO_STATUS  0x1D
#define OP_DUMMY_CLOCK  0x1E

#define CTRL0_OPER_EN   (1u << 12)
#define CTRL0_SOFT_RST  (1u << 15)
#define STS_COMP        (1u << 12)
#define STS_W1C         (0x7Fu << 12)
#define SPI_CFG_MANUAL  (1u << 1)
#define SPI_CFG_CS_HIGH (1u << 4)
#define DLL1_SW_RESET   (1u << 0)
#define FIFO_CTRL_DMA   (1u << 1)
#define FIFO_EMPTY      (1u << 8)
#define FIFO_FULL       (1u << 9)
#define FIFO_WORDS      16

#define SR1_WIP         (1u << 0)
#define SR1_WEL         (1u << 1)

/* ---- the SPI NOR --------------------------------------------------- */

static void nvt_flash_persist(NvtFlashState *s)
{
    uint32_t off = s->dirty_lo, len;

    if (s->dirty_hi <= s->dirty_lo) {
        return;
    }
    len = s->dirty_hi - s->dirty_lo;
    s->dirty_lo = UINT32_MAX;
    s->dirty_hi = 0;
    while (s->fd >= 0 && len) {
        ssize_t n = pwrite(s->fd, s->array + off, len, off);

        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            error_report("nvt-flash: write-through to %s failed at 0x%x: %s",
                         s->flash_file, off,
                         n < 0 ? strerror(errno) : "no progress");
            exit(1);
        }
        off += n;
        len -= n;
    }
}

static void nvt_flash_dirty(NvtFlashState *s, uint32_t off, uint32_t len)
{
    s->dirty_lo = MIN(s->dirty_lo, off);
    s->dirty_hi = MAX(s->dirty_hi, off + len);
}

static void nvt_flash_erase(NvtFlashState *s, uint32_t addr, uint32_t len)
{
    addr &= ~(len - 1) & (s->size - 1);
    memset(s->array + addr, 0xFF, len);
    nvt_flash_dirty(s, addr, len);
}

/* Address bytes an opcode takes (the 0x1x / 0x2x / 0xDC forms are the
 * explicit 4-byte ones). */
static int nor_addr_len(NvtFlashState *s, uint8_t cmd)
{
    switch (cmd) {
    case 0x13: case 0x3C: case 0x6C: case 0x12: case 0x34:
    case 0x21: case 0xDC:
        return 4;
    default:
        return s->addr4 ? 4 : 3;
    }
}

static void spi_select(NvtFlashState *s)
{
    s->selected = true;
    s->pos = 0;
    s->addr = 0;
}

static void spi_deselect(NvtFlashState *s)
{
    if (!s->selected) {
        return;
    }
    s->selected = false;
    if (!s->pos) {
        return;
    }
    switch (s->cmd) {
    case 0x06:                              /* WREN */
        s->wel = true;
        break;
    case 0x04:                              /* WRDI */
    case 0x02: case 0x12: case 0x32: case 0x34:    /* PP, 4PP */
    case 0x01: case 0x31:                   /* WRSR, WRSR2 */
        s->wel = false;
        break;
    case 0xB7:                              /* EN4B */
        s->addr4 = true;
        break;
    case 0xE9:                              /* EX4B */
        s->addr4 = false;
        break;
    case 0x20: case 0x21:                   /* SE, 4 KiB */
    case 0x52:                              /* BE, 32 KiB */
    case 0xD8: case 0xDC:                   /* BE, 64 KiB */
        if (s->wel && s->pos > nor_addr_len(s, s->cmd)) {
            nvt_flash_erase(s, s->addr,
                            (s->cmd & ~1) == 0x20 ? 4 * KiB :
                            s->cmd == 0x52 ? 32 * KiB : 64 * KiB);
        }
        s->wel = false;
        break;
    case 0xC7: case 0x60:                   /* CE */
        if (s->wel) {
            nvt_flash_erase(s, 0, s->size);
        }
        s->wel = false;
        break;
    }
    nvt_flash_persist(s);
}

static uint8_t spi_xfer(NvtFlashState *s, uint8_t tx)
{
    uint32_t pos = s->pos++;
    uint8_t rx = 0xFF;
    int alen;

    if (pos == 0) {
        s->cmd = tx;
        return 0xFF;
    }
    alen = nor_addr_len(s, s->cmd);
    switch (s->cmd) {
    case 0x03: case 0x13:                   /* READ */
    case 0x0B:                              /* FAST_READ, one dummy byte */
    case 0x3B: case 0x3C:                   /* dual output read */
    case 0x6B: case 0x6C:                   /* quad output read */
        if (pos <= alen) {
            s->addr = (s->addr << 8) | tx;
        } else if ((s->cmd & ~0x10) == 0x03 || pos > alen + 1) {
            rx = s->array[s->addr & (s->size - 1)];
            s->addr++;
        }
        break;
    case 0x02: case 0x12:                   /* PP */
    case 0x32: case 0x34:                   /* quad PP */
        if (pos <= alen) {
            s->addr = (s->addr << 8) | tx;
        } else if (s->wel) {
            uint32_t a = s->addr & (s->size - 1);

            s->array[a] &= tx;
            nvt_flash_dirty(s, a, 1);
            /* Wraps within the 256-byte page, as NOR does. */
            s->addr = (s->addr & ~0xFFu) | ((s->addr + 1) & 0xFF);
        }
        break;
    case 0x20: case 0x21: case 0x52: case 0xD8: case 0xDC:
        if (pos <= alen) {
            s->addr = (s->addr << 8) | tx;
        }
        break;
    case 0x9F:                              /* RDID */
        rx = s->jedec >> (8 * (2 - (pos - 1) % 3));
        break;
    case 0x90:                              /* REMS: 3 address bytes */
        if (pos > 3) {
            rx = (pos - 4) & 1 ? (s->jedec & 0xFF) - 1 : s->jedec >> 16;
        }
        break;
    case 0xAB:                              /* RES: 3 dummy bytes */
        if (pos > 3) {
            rx = (s->jedec & 0xFF) - 1;
        }
        break;
    case 0x05:                              /* RDSR */
        rx = (s->sr[0] & ~(SR1_WIP | SR1_WEL)) | (s->wel ? SR1_WEL : 0);
        break;
    case 0x35:                              /* RDSR2 */
        rx = s->sr[1];
        break;
    case 0x01:                              /* WRSR: SR1 [SR2] */
        if (s->wel && pos <= 2) {
            s->sr[pos - 1] = tx;
        }
        break;
    case 0x31:                              /* WRSR2 */
        if (s->wel && pos == 1) {
            s->sr[1] = tx;
        }
        break;
    case 0x5A:                              /* SFDP: none */
        break;
    default:
        if (pos == 1) {
            qemu_log_mask(LOG_UNIMP, "nvt-flash: SPI opcode 0x%02x\n",
                          s->cmd);
        }
        break;
    }
    return rx;
}

void nvt_flash_read(NvtFlashState *s, uint32_t off, void *buf, uint32_t len)
{
    uint8_t *p = buf;

    while (len--) {
        *p++ = s->array[off++ & (s->size - 1)];
    }
}

/* ---- the controller ------------------------------------------------ */

static uint32_t *reg(NvtFlashState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static bool nvt_flash_manual_cs(NvtFlashState *s)
{
    return *reg(s, REG_SPI_CFG) & SPI_CFG_MANUAL;
}

static void nvt_flash_update_irq(NvtFlashState *s)
{
    qemu_set_irq(s->irq, *reg(s, REG_CTRL_STS) & *reg(s, REG_INTMASK) &
                 STS_W1C);
}

static void nvt_flash_complete(NvtFlashState *s)
{
    if (s->rd_left) {
        return;                     /* still streaming a PIO read */
    }
    if (!nvt_flash_manual_cs(s)) {
        spi_deselect(s);
    }
    s->busy = false;
    *reg(s, REG_CTRL_STS) |= STS_COMP;
    nvt_flash_update_irq(s);
}

static void nvt_flash_begin(NvtFlashState *s)
{
    if (!nvt_flash_manual_cs(s)) {
        spi_select(s);
    } else if (!s->selected) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "nvt-flash: operation with CS held high\n");
        spi_select(s);
    }
}

static void nvt_flash_send_addr(NvtFlashState *s)
{
    uint32_t addr = *reg(s, REG_ROWADDR);
    int n = (*reg(s, REG_MODULE0) >> 2 & 3) + 1;

    while (n--) {
        spi_xfer(s, addr >> (8 * n));
    }
}

/* Clock out cmd until (status & mask) == (value & mask). */
static void nvt_flash_poll_status(NvtFlashState *s, uint8_t cmd)
{
    uint32_t chk = *reg(s, REG_STS_CHECK);
    uint8_t value = chk & 0xFF, mask = (chk >> 8) & 0xFF, st;
    int tries = 1000;

    do {
        spi_select(s);
        spi_xfer(s, cmd);
        st = spi_xfer(s, 0);
        spi_deselect(s);
    } while ((st & mask) != (value & mask) && --tries);
    *reg(s, REG_CTRL_STS) = (*reg(s, REG_CTRL_STS) & ~0xFFu) | st;
}

/*
 * Clock the next part of a PIO read into the buffer the data port drains.
 * A read longer than the buffer is refilled as it empties, and completes
 * when its last byte has been clocked in.
 */
static void nvt_flash_fifo_load(NvtFlashState *s)
{
    uint32_t len = MIN(s->rd_left, NVT_FLASH_BUF), i;

    for (i = 0; i < len; i++) {
        s->buf[i] = spi_xfer(s, 0);
    }
    s->rd_left -= len;
    s->buf_len = ROUND_UP(len, 4);
    s->buf_pos = 0;
}

static void nvt_flash_run(NvtFlashState *s, uint32_t ctrl0)
{
    const uint32_t cmd = *reg(s, REG_COMMAND);
    const uint32_t len = *reg(s, REG_DATALEN) & 0x3FFFFFF;
    const bool dma = *reg(s, REG_FIFO_CTRL) & FIFO_CTRL_DMA;
    const uint32_t op = ctrl0 & 0x3F;
    uint8_t chunk[256];
    uint32_t done, n, i;
    dma_addr_t a;

    s->busy = true;
    s->buf_len = s->buf_pos = 0;
    s->rd_left = 0;

    switch (op) {
    case OP_CMD:
        nvt_flash_begin(s);
        spi_xfer(s, cmd);
        break;
    case OP_CMD_ADDR:
        nvt_flash_begin(s);
        spi_xfer(s, cmd);
        nvt_flash_send_addr(s);
        break;
    case OP_DUMMY_CLOCK:
        /*
         * The NOR here takes its dummy phase as whole bytes, so a partial
         * byte of clocks (DUMMY_CLOCK_NUM < 7) still spends one.
         */
        nvt_flash_begin(s);
        for (n = DIV_ROUND_UP((*reg(s, REG_DUMMY_CLK) & 7) + 1, 8); n; n--) {
            spi_xfer(s, 0);
        }
        break;
    case OP_CMD_STS:
        nvt_flash_begin(s);
        spi_xfer(s, cmd);
        *reg(s, REG_CTRL_STS) = (*reg(s, REG_CTRL_STS) & ~0xFFu) |
                                spi_xfer(s, 0);
        break;
    case OP_READ_ID:
        nvt_flash_begin(s);
        spi_xfer(s, cmd);
        s->rd_left = len;
        nvt_flash_fifo_load(s);
        break;
    case OP_READ_N:
        nvt_flash_begin(s);
        if (!dma) {
            s->rd_left = len;
            nvt_flash_fifo_load(s);
            break;
        }
        a = *reg(s, REG_DMASTART);
        for (done = 0; done < len; done += n) {
            n = MIN(len - done, sizeof(chunk));
            for (i = 0; i < n; i++) {
                chunk[i] = spi_xfer(s, 0);
            }
            dma_memory_write(&address_space_memory, a + done, chunk, n,
                             MEMTXATTRS_UNSPECIFIED);
        }
        *reg(s, REG_DMACURRENT) = a + len;
        break;
    case OP_WRITE_N:
        nvt_flash_begin(s);
        if (!dma && len) {
            /* Completes once the data port has taken DATALEN bytes. */
            s->wr_left = len;
            return;
        }
        if (!dma) {
            break;
        }
        a = *reg(s, REG_DMASTART);
        for (done = 0; done < len; done += n) {
            n = MIN(len - done, sizeof(chunk));
            dma_memory_read(&address_space_memory, a + done, chunk, n,
                            MEMTXATTRS_UNSPECIFIED);
            for (i = 0; i < n; i++) {
                spi_xfer(s, chunk[i]);
            }
        }
        *reg(s, REG_DMACURRENT) = a + len;
        break;
    case OP_AUTO_STATUS:
        spi_deselect(s);
        nvt_flash_poll_status(s, cmd & 0xFF);
        break;
    case OP_BLK_ERSE:
        spi_select(s);
        spi_xfer(s, cmd & 0xFF);
        nvt_flash_send_addr(s);
        spi_deselect(s);
        nvt_flash_poll_status(s, (cmd >> 8) & 0xFF);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "nvt-flash: operation 0x%02x\n", op);
        break;
    }
    nvt_flash_complete(s);
}

static uint32_t nvt_flash_fifo_sts(NvtFlashState *s)
{
    uint32_t words = (s->buf_len - s->buf_pos) / 4;

    if (s->wr_left) {
        return FIFO_EMPTY;
    }
    words = MIN(words, FIFO_WORDS);
    return words | (words ? 0 : FIFO_EMPTY) |
           (words == FIFO_WORDS ? FIFO_FULL : 0);
}

static uint64_t nvt_flash_mmio_read(void *opaque, hwaddr off, unsigned size)
{
    NvtFlashState *s = opaque;
    uint32_t v;

    switch (off) {
    case REG_DATAPORT:
        if (s->buf_pos + 4 > s->buf_len) {
            return 0;
        }
        v = ldl_le_p(s->buf + s->buf_pos);
        s->buf_pos += 4;
        if (s->buf_pos >= s->buf_len && s->rd_left) {
            nvt_flash_fifo_load(s);
            if (!s->rd_left) {
                nvt_flash_complete(s);
            }
        }
        return v;
    case REG_FIFO_STS:
        return nvt_flash_fifo_sts(s);
    case REG_CTRL0:
        return *reg(s, off) | (s->busy ? CTRL0_OPER_EN : 0);
    default:
        return off < NVT_FLASH_REGS_SIZE ? *reg(s, off) : 0;
    }
}

static void nvt_flash_mmio_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    NvtFlashState *s = opaque;
    uint32_t i;
    uint8_t b[4];

    switch (off) {
    case REG_CTRL0:
        *reg(s, off) = val & ~(CTRL0_OPER_EN | CTRL0_SOFT_RST);
        if (val & CTRL0_SOFT_RST) {
            spi_deselect(s);
            s->busy = false;
            s->wr_left = 0;
            s->rd_left = 0;
            s->buf_len = s->buf_pos = 0;
        }
        if (val & CTRL0_OPER_EN) {
            nvt_flash_run(s, val);
        }
        break;
    case REG_CTRL_STS:
        *reg(s, off) &= ~(val & STS_W1C);
        nvt_flash_update_irq(s);
        break;
    case REG_INTMASK:
        *reg(s, off) = val;
        nvt_flash_update_irq(s);
        break;
    case REG_SPI_CFG:
        *reg(s, off) = val;
        if (val & SPI_CFG_MANUAL) {
            if (!(val & SPI_CFG_CS_HIGH) && !s->selected) {
                spi_select(s);
            } else if ((val & SPI_CFG_CS_HIGH) && s->selected) {
                spi_deselect(s);
            }
        } else if (s->selected && !s->busy) {
            spi_deselect(s);
        }
        break;
    case REG_DLL1:
        *reg(s, off) = val & ~DLL1_SW_RESET;     /* resets at once */
        break;
    case REG_DATAPORT:
        if (!s->wr_left) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "nvt-flash: data port write with no transfer\n");
            break;
        }
        stl_le_p(b, val);
        for (i = 0; i < 4 && s->wr_left; i++, s->wr_left--) {
            spi_xfer(s, b[i]);
        }
        if (!s->wr_left) {
            nvt_flash_complete(s);
        }
        break;
    case REG_FIFO_STS:
        break;
    default:
        if (off < NVT_FLASH_REGS_SIZE) {
            *reg(s, off) = val;
        }
        break;
    }
}

static const MemoryRegionOps nvt_flash_ops = {
    .read = nvt_flash_mmio_read,
    .write = nvt_flash_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_flash_reset(DeviceState *dev)
{
    NvtFlashState *s = NVT_FLASH(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->selected = false;
    s->busy = false;
    s->wr_left = 0;
    s->rd_left = 0;
    s->buf_len = s->buf_pos = 0;
    s->wel = false;
    s->addr4 = false;
    s->pos = 0;
    qemu_irq_lower(s->irq);
}

static void nvt_flash_realize(DeviceState *dev, Error **errp)
{
    NvtFlashState *s = NVT_FLASH(dev);
    off_t len;

    s->fd = -1;
    s->dirty_lo = UINT32_MAX;
    s->dirty_hi = 0;
    if (s->flash_file && *s->flash_file) {
        s->fd = open(s->flash_file, O_RDWR);
        if (s->fd < 0) {
            error_setg_errno(errp, errno, "nvt-flash: cannot open %s",
                             s->flash_file);
            return;
        }
        /* Two machines on one image would undo each other's erases. */
        if (qemu_lock_fd(s->fd, 0, 0, true)) {
            error_setg(errp, "nvt-flash: %s is in use by another process",
                       s->flash_file);
            goto fail;
        }
        len = lseek(s->fd, 0, SEEK_END);
        if (len < 1 * MiB || len > 32 * MiB || (len & (len - 1))) {
            error_setg(errp, "nvt-flash: %s is not a power-of-two size "
                       "from 1 to 32 MiB", s->flash_file);
            goto fail;
        }
        error_setg(&s->migration_blocker, "nvt-flash: flash-file %s is "
                   "written through; it cannot be migrated", s->flash_file);
        if (migrate_add_blocker(&s->migration_blocker, errp) < 0) {
            goto fail;
        }
        s->size = len;
    }
    if (s->size < 1 * MiB || s->size > 32 * MiB ||
        (s->size & (s->size - 1))) {
        error_setg(errp, "nvt-flash: size must be a power of two from 1 to "
                   "32 MiB");
        goto fail;
    }
    s->array = g_malloc(s->size);
    memset(s->array, 0xFF, s->size);
    if (s->fd >= 0 && pread(s->fd, s->array, s->size, 0) != s->size) {
        error_setg(errp, "nvt-flash: cannot read %s", s->flash_file);
        goto fail;
    }
    if (!s->jedec) {
        /* A Winbond W25Q part of the image's size (capacity 0x14..0x19);
         * spinor.c's standard table matches on the capacity byte. */
        s->jedec = 0xEF4000 | ctz32(s->size);
    }

    memory_region_init_io(&s->iomem, OBJECT(s), &nvt_flash_ops, s,
                          TYPE_NVT_FLASH, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    return;

fail:
    if (s->fd >= 0) {
        close(s->fd);
        s->fd = -1;
    }
}

static const VMStateDescription vmstate_nvt_flash = {
    .name = TYPE_NVT_FLASH,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtFlashState, NVT_FLASH_REGS_SIZE / 4),
        VMSTATE_UINT8_ARRAY(buf, NvtFlashState, NVT_FLASH_BUF),
        VMSTATE_UINT32(buf_len, NvtFlashState),
        VMSTATE_UINT32(buf_pos, NvtFlashState),
        VMSTATE_UINT32(wr_left, NvtFlashState),
        VMSTATE_UINT32(rd_left, NvtFlashState),
        VMSTATE_BOOL(busy, NvtFlashState),
        VMSTATE_UINT8_ARRAY(sr, NvtFlashState, 2),
        VMSTATE_BOOL(wel, NvtFlashState),
        VMSTATE_BOOL(addr4, NvtFlashState),
        VMSTATE_UINT8(cmd, NvtFlashState),
        VMSTATE_UINT32(addr, NvtFlashState),
        VMSTATE_UINT32(pos, NvtFlashState),
        VMSTATE_BOOL(selected, NvtFlashState),
        VMSTATE_VBUFFER_UINT32(array, NvtFlashState, 1, NULL, size),
        VMSTATE_END_OF_LIST()
    }
};

static const Property nvt_flash_properties[] = {
    DEFINE_PROP_STRING("flash-file", NvtFlashState, flash_file),
    DEFINE_PROP_UINT32("size", NvtFlashState, size, 8 * MiB),
    DEFINE_PROP_UINT32("jedec", NvtFlashState, jedec, 0),
};

static void nvt_flash_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_flash_realize;
    device_class_set_legacy_reset(dc, nvt_flash_reset);
    device_class_set_props(dc, nvt_flash_properties);
    dc->vmsd = &vmstate_nvt_flash;
}

static const TypeInfo nvt_flash_info = {
    .name = TYPE_NVT_FLASH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtFlashState),
    .class_init = nvt_flash_class_init,
};

static void nvt_flash_register_types(void)
{
    type_register_static(&nvt_flash_info);
}

type_init(nvt_flash_register_types)
