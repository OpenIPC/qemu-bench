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
 *   0x15 SPI_PROG_CMD  (NAND) cmd 1; cmd 2, column, DATALEN bytes from
 *                    DMASTART and the spare SRAM; cmd 3, row; then cmd 4 +
 *                    COMMAND2[7:0] polled, the byte in CTRL_STS
 *   0x16 DATA_RD3    (NAND) cmd 1, row; cmd 2 + cmd 3 polled; cmd 4,
 *                    column, a dummy byte, DATALEN bytes to DMASTART (or
 *                    the FIFO), the OOB to the spare SRAM; then COMMAND2's
 *                    cmd 5 + cmd 6 once, the byte in CTRL_STS
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
 * MODULE0 bit 19 set with bit 20 clear is SPI NAND mode (nand_type,
 * spi_flash_type), the 566 boards' setting.  There a status read is a
 * GET_FEATURE plus its feature address -- two command bytes, COMMAND's
 * next byte -- for CMD_STS, AUTO_STATUS and BLK_ERSE's poll, and READ_ID
 * clocks one address byte before the ID.  The row address is the page,
 * the column (COLADDR, MODULE0.col_addr + 1 bytes) the byte in it.  The
 * spare SRAM at 0x200 holds the page's OOB as the flash has it: the
 * drivers (spinand.c) copy it word for word to and from their buffer.
 *
 * The flash is a generic SPI NOR answering with JEDEC ID "jedec", or, for
 * an image of 128 or 256 MiB, a Winbond W25N01GV / W25N02KV SPI NAND
 * (EF AA 21 / 22: 2 KiB pages, 64 bytes of OOB, 128 KiB blocks) with its
 * on-die ECC enabled and never reporting an error.  A NAND image is the
 * data alone, its OOB then kept in memory as erased, or every page
 * followed by its 64 OOB bytes (2112 bytes a page).  Erases and programs
 * complete at once and are written through to flash-file; block
 * protection (feature A0) is storage only.
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
#define REG_COLADDR     0x38
#define REG_ROWADDR     0x3C
#define REG_INTMASK     0x44
#define REG_CTRL_STS    0x48
#define REG_COMMAND2    0xB4
#define REG_STS_CHECK   0xBC
#define REG_DUMMY_CLK   0xC0
#define REG_DATAPORT    0x100
#define REG_DATALEN     0x104
#define REG_FIFO_STS    0x108
#define REG_FIFO_CTRL   0x10C
#define REG_DMASTART    0x110
#define REG_DMACURRENT  0x114
#define REG_SPARE       0x200

#define OP_BLK_ERSE     0x05
#define OP_READ_ID      0x06
#define OP_CMD          0x07
#define OP_CMD_STS      0x0B
#define OP_CMD_ADDR     0x0C
#define OP_SPI_PROG_CMD 0x15
#define OP_DATA_RD3     0x16
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

#define MODULE0_NAND    (1u << 19)
#define MODULE0_SPI_NOR (1u << 20)

#define SR1_WIP         (1u << 0)
#define SR1_WEL         (1u << 1)
#define SR3_WPS         (1u << 2)
#define SR3_RESET       0x60        /* W25Q128FV: DRV 11, WPS 0 */

#define NAND_FEAT_LOCK  0
#define NAND_FEAT_CFG   1
#define NAND_FEAT_STS   2
#define NAND_CFG_ECCEN  (1u << 4)
#define NAND_CFG_BUF    (1u << 3)
#define NAND_STS_WEL    (1u << 1)

/* ---- the SPI NOR --------------------------------------------------- */

static void nvt_flash_pwrite(NvtFlashState *s, const uint8_t *buf,
                             uint32_t len, uint32_t off)
{
    while (s->fd >= 0 && len) {
        ssize_t n = pwrite(s->fd, buf, len, off);

        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            error_report("nvt-flash: write-through to %s failed at 0x%x: %s",
                         s->flash_file, off,
                         n < 0 ? strerror(errno) : "no progress");
            exit(1);
        }
        buf += n;
        off += n;
        len -= n;
    }
}

static void nvt_flash_persist(NvtFlashState *s)
{
    uint32_t lo = s->dirty_lo, hi = s->dirty_hi, p;

    if (hi <= lo) {
        return;
    }
    s->dirty_lo = UINT32_MAX;
    s->dirty_hi = 0;
    if (!s->nand || s->file_oob) {
        nvt_flash_pwrite(s, s->array + lo, hi - lo, lo);
        return;
    }
    /* A data-only NAND image: write back the data of each page touched. */
    for (p = lo / NVT_NAND_RAW; p <= (hi - 1) / NVT_NAND_RAW; p++) {
        nvt_flash_pwrite(s, s->array + p * NVT_NAND_RAW, NVT_NAND_PAGE,
                         p * NVT_NAND_PAGE);
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

/* ---- the SPI NAND -------------------------------------------------- */

static uint8_t *nand_page(NvtFlashState *s, uint32_t row)
{
    return s->array + (row % s->pages) * NVT_NAND_RAW;
}

static uint8_t nand_get_feature(NvtFlashState *s, uint8_t addr)
{
    switch (addr >> 4) {
    case 0xA:
        return s->feat[NAND_FEAT_LOCK];
    case 0xB:
        return s->feat[NAND_FEAT_CFG];
    case 0xC:
        /* never busy (OIP), no ECC error, no failure */
        return s->feat[NAND_FEAT_STS] | (s->wel ? NAND_STS_WEL : 0);
    default:
        return 0;
    }
}

static void nand_set_feature(NvtFlashState *s, uint8_t addr, uint8_t val)
{
    switch (addr >> 4) {
    case 0xA:
        s->feat[NAND_FEAT_LOCK] = val;
        break;
    case 0xB:
        s->feat[NAND_FEAT_CFG] = val;
        break;
    default:
        break;
    }
}

/* What happens at CS deassertion: the array operations. */
static void nand_deselect(NvtFlashState *s)
{
    uint8_t *page;
    uint32_t i, off;

    switch (s->cmd) {
    case 0xFF:                              /* RESET: power-on state */
        s->wel = false;
        s->feat[NAND_FEAT_CFG] = NAND_CFG_ECCEN | NAND_CFG_BUF;
        s->feat[NAND_FEAT_STS] = 0;
        break;
    case 0x06:                              /* WRITE ENABLE */
        s->wel = true;
        break;
    case 0x04:                              /* WRITE DISABLE */
        s->wel = false;
        break;
    case 0x13:                              /* PAGE DATA READ */
        if (s->pos > 3) {
            memcpy(s->cache, nand_page(s, s->addr), NVT_NAND_RAW);
        }
        break;
    case 0x10:                              /* PROGRAM EXECUTE */
        if (s->wel && s->pos > 3) {
            page = nand_page(s, s->addr);
            for (i = 0; i < NVT_NAND_RAW; i++) {
                page[i] &= s->cache[i];
            }
            off = page - s->array;
            nvt_flash_dirty(s, off, NVT_NAND_RAW);
        }
        s->wel = false;
        break;
    case 0xD8:                              /* BLOCK ERASE */
        if (s->wel && s->pos > 3) {
            page = nand_page(s, s->addr & ~(NVT_NAND_BLOCK - 1));
            memset(page, 0xFF, NVT_NAND_BLOCK * NVT_NAND_RAW);
            off = page - s->array;
            nvt_flash_dirty(s, off, NVT_NAND_BLOCK * NVT_NAND_RAW);
        }
        s->wel = false;
        break;
    }
}

static uint8_t nand_xfer(NvtFlashState *s, uint32_t pos, uint8_t tx)
{
    uint32_t col;

    switch (s->cmd) {
    case 0x9F:                              /* JEDEC ID, after a dummy */
        if (pos >= 2 && pos <= 4) {
            return s->jedec >> (8 * (4 - pos));
        }
        return 0;
    case 0x0F: case 0x05:                   /* GET FEATURE */
        if (pos == 1) {
            s->addr = tx;
            return 0xFF;
        }
        return nand_get_feature(s, s->addr);
    case 0x1F: case 0x01:                   /* SET FEATURE */
        if (pos == 1) {
            s->addr = tx;
        } else if (pos == 2) {
            nand_set_feature(s, s->addr, tx);
        }
        return 0xFF;
    case 0x13: case 0x10: case 0xD8:        /* 24-bit page address */
        if (pos <= 3) {
            s->addr = (s->addr << 8) | tx;
        }
        return 0xFF;
    case 0x03: case 0x0B: case 0x3B: case 0x6B:
        /* READ from the cache: column, a dummy byte, data */
        if (pos <= 2) {
            s->addr = (s->addr << 8) | tx;
            return 0xFF;
        }
        if (pos == 3) {
            return 0xFF;
        }
        col = s->addr++ & 0xFFF;
        return col < NVT_NAND_RAW ? s->cache[col] : 0xFF;
    case 0x02: case 0x32:                   /* PROGRAM DATA LOAD */
        if (pos == 1) {
            memset(s->cache, 0xFF, sizeof(s->cache));
        }
        /* fall through */
    case 0x84: case 0x34:                   /* RANDOM PROGRAM DATA LOAD */
        if (pos <= 2) {
            s->addr = (s->addr << 8) | tx;
            return 0xFF;
        }
        col = s->addr++ & 0xFFF;
        if (col < NVT_NAND_RAW) {
            s->cache[col] = tx;
        }
        return 0xFF;
    case 0xFF: case 0x06: case 0x04:
        return 0xFF;
    default:
        if (pos == 1) {
            qemu_log_mask(LOG_UNIMP, "nvt-flash: SPI NAND opcode 0x%02x\n",
                          s->cmd);
        }
        return 0xFF;
    }
}

/*
 * W25Q individual block protection, which SR3's WPS turns on in place of
 * SR1's BP bits.  The lock bits are volatile and power up set; the top and
 * bottom 64 KiB blocks lock per 4 KiB sector, the rest per 64 KiB block.
 * The Xiongmai kernel's spinor.c sets WPS and unlocks only what it writes
 * (XmSpiNor_BlkLockMgr: 256 + 2 * 15 = 286 lock units on a 16 MiB part).
 */
static void nor_lock_range(NvtFlashState *s, uint32_t addr, bool lock)
{
    uint32_t a = addr & (s->size - 1), n = 1, i;

    if (a >= 64 * KiB && a < s->size - 64 * KiB) {
        a &= ~(64 * KiB - 1);
        n = 16;
    }
    for (i = a / (4 * KiB); i < a / (4 * KiB) + n; i++) {
        if (lock) {
            s->lock[i / 8] |= 1 << (i % 8);
        } else {
            s->lock[i / 8] &= ~(1 << (i % 8));
        }
    }
}

static bool nor_locked(NvtFlashState *s, uint32_t addr, uint32_t len)
{
    uint32_t i;

    if (!(s->sr3 & SR3_WPS)) {
        return false;
    }
    addr &= s->size - 1;
    for (i = addr / (4 * KiB); i <= (addr + len - 1) / (4 * KiB); i++) {
        if (s->lock[i / 8] & (1 << (i % 8))) {
            return true;
        }
    }
    return false;
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
    if (s->nand) {
        nand_deselect(s);
        nvt_flash_persist(s);
        return;
    }
    switch (s->cmd) {
    case 0x06:                              /* WREN */
        s->wel = true;
        break;
    case 0x50:                              /* volatile SR write enable */
        s->wel = true;
        break;
    case 0x04:                              /* WRDI */
    case 0x02: case 0x12: case 0x32: case 0x34:    /* PP, 4PP */
    case 0x01: case 0x31: case 0x11:        /* WRSR, WRSR2, WRSR3 */
        s->wel = false;
        break;
    case 0x36: case 0x39:                   /* individual block lock/unlock */
        if (s->wel && s->pos > nor_addr_len(s, s->cmd)) {
            nor_lock_range(s, s->addr, s->cmd == 0x36);
        }
        s->wel = false;
        break;
    case 0x7E: case 0x98:                   /* global block lock/unlock */
        if (s->wel) {
            memset(s->lock, s->cmd == 0x7E ? 0xFF : 0, sizeof(s->lock));
        }
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
            uint32_t len = (s->cmd & ~1) == 0x20 ? 4 * KiB :
                           s->cmd == 0x52 ? 32 * KiB : 64 * KiB;

            if (!nor_locked(s, s->addr & ~(len - 1), len)) {
                nvt_flash_erase(s, s->addr, len);
            }
        }
        s->wel = false;
        break;
    case 0xC7: case 0x60:                   /* CE */
        if (s->wel && !nor_locked(s, 0, s->size)) {
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
    if (s->nand) {
        return nand_xfer(s, pos, tx);
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
        } else if (s->wel && !nor_locked(s, s->addr, 1)) {
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
    case 0x15:                              /* RDSR3 */
        rx = s->sr3;
        break;
    case 0x11:                              /* WRSR3 */
        if (s->wel && pos == 1) {
            s->sr3 = tx;
        }
        break;
    case 0x36: case 0x39:                   /* block lock / unlock */
        if (pos <= alen) {
            s->addr = (s->addr << 8) | tx;
        }
        break;
    case 0x3D:                              /* read block lock */
        if (pos <= alen) {
            s->addr = (s->addr << 8) | tx;
        } else {
            uint32_t i = (s->addr & (s->size - 1)) / (4 * KiB);

            rx = (s->lock[i / 8] >> (i % 8)) & 1;
        }
        break;
    case 0x7E: case 0x98: case 0x50:
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
    uint32_t a;

    while (len--) {
        a = off++ & (s->size - 1);
        if (s->nand) {
            a = a / NVT_NAND_PAGE * NVT_NAND_RAW + a % NVT_NAND_PAGE;
        }
        *p++ = s->array[a];
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

/* SPI NAND mode: status reads are GET_FEATURE plus a feature address. */
static bool nvt_flash_nand_mode(NvtFlashState *s)
{
    return (*reg(s, REG_MODULE0) & (MODULE0_NAND | MODULE0_SPI_NOR)) ==
           MODULE0_NAND;
}

static void nvt_flash_send_addr(NvtFlashState *s)
{
    uint32_t addr = *reg(s, REG_ROWADDR);
    int n = (*reg(s, REG_MODULE0) >> 2 & 3) + 1;

    while (n--) {
        spi_xfer(s, addr >> (8 * n));
    }
}

static void nvt_flash_send_col(NvtFlashState *s)
{
    uint32_t col = *reg(s, REG_COLADDR);
    int n = (*reg(s, REG_MODULE0) & 3) + 1;

    while (n--) {
        spi_xfer(s, col >> (8 * n));
    }
}

static void nvt_flash_set_status(NvtFlashState *s, uint8_t st)
{
    *reg(s, REG_CTRL_STS) = (*reg(s, REG_CTRL_STS) & ~0xFFu) | st;
}

/* One status read: ncmd command bytes of cmd, low byte first. */
static uint8_t nvt_flash_read_status(NvtFlashState *s, uint32_t cmd,
                                     int ncmd)
{
    uint8_t st;
    int i;

    spi_select(s);
    for (i = 0; i < ncmd; i++) {
        spi_xfer(s, cmd >> (8 * i));
    }
    st = spi_xfer(s, 0);
    spi_deselect(s);
    return st;
}

/* Read the status until (status & mask) == (value & mask). */
static void nvt_flash_poll_status(NvtFlashState *s, uint32_t cmd, int ncmd)
{
    uint32_t chk = *reg(s, REG_STS_CHECK);
    uint8_t value = chk & 0xFF, mask = (chk >> 8) & 0xFF, st;
    int tries = 1000;

    do {
        st = nvt_flash_read_status(s, cmd, ncmd);
    } while ((st & mask) != (value & mask) && --tries);
    nvt_flash_set_status(s, st);
}

/* Clock len bytes between the flash and guest memory at a. */
static void nvt_flash_dma_in(NvtFlashState *s, dma_addr_t a, uint32_t len)
{
    uint8_t chunk[256];
    uint32_t done, n, i;

    for (done = 0; done < len; done += n) {
        n = MIN(len - done, sizeof(chunk));
        for (i = 0; i < n; i++) {
            chunk[i] = spi_xfer(s, 0);
        }
        dma_memory_write(&address_space_memory, a + done, chunk, n,
                         MEMTXATTRS_UNSPECIFIED);
    }
    *reg(s, REG_DMACURRENT) = a + len;
}

static void nvt_flash_dma_out(NvtFlashState *s, dma_addr_t a, uint32_t len)
{
    uint8_t chunk[256];
    uint32_t done, n, i;

    for (done = 0; done < len; done += n) {
        n = MIN(len - done, sizeof(chunk));
        dma_memory_read(&address_space_memory, a + done, chunk, n,
                        MEMTXATTRS_UNSPECIFIED);
        for (i = 0; i < n; i++) {
            spi_xfer(s, chunk[i]);
        }
    }
    *reg(s, REG_DMACURRENT) = a + len;
}

/*
 * DATA_RD3: load the page into the NAND's cache, wait for it, then read
 * from the column on -- DATALEN bytes of data to memory, the page's OOB to
 * the spare SRAM, or (column in the OOB) DATALEN OOB bytes to the SRAM --
 * and leave the status (COMMAND2's cmd 5 + cmd 6) in CTRL_STS.
 */
static void nvt_flash_data_rd3(NvtFlashState *s, uint32_t cmd, uint32_t len,
                               bool dma)
{
    uint32_t col = *reg(s, REG_COLADDR) & 0xFFF, i;
    uint8_t *spare = s->spare;

    spi_select(s);
    spi_xfer(s, cmd);
    nvt_flash_send_addr(s);
    spi_deselect(s);
    nvt_flash_poll_status(s, cmd >> 8, 2);

    spi_select(s);
    spi_xfer(s, cmd >> 24);
    nvt_flash_send_col(s);
    spi_xfer(s, 0);                         /* dummy */
    if (col >= NVT_NAND_PAGE) {
        /* Up to the page's last OOB byte: at most NVT_NAND_OOB, so
         * inside the spare SRAM. */
        if (col >= NVT_NAND_RAW) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-flash: DATA_RD3 column "
                          "0x%x past the page\n", col);
        }
        len = col < NVT_NAND_RAW ? MIN(len, NVT_NAND_RAW - col) : 0;
        for (i = 0; i < len; i++) {
            spare[i] = spi_xfer(s, 0);
        }
    } else {
        len = MIN(len, NVT_NAND_PAGE - col);
        if (dma) {
            nvt_flash_dma_in(s, *reg(s, REG_DMASTART), len);
        } else {
            /* Short enough to buffer whole; the FIFO drains it after. */
            for (i = 0; i < len; i++) {
                s->buf[i] = spi_xfer(s, 0);
            }
            s->buf_len = ROUND_UP(len, 4);
        }
        for (i = col + len; i < NVT_NAND_PAGE; i++) {
            spi_xfer(s, 0);
        }
        for (i = 0; i < NVT_NAND_OOB; i++) {
            spare[i] = spi_xfer(s, 0);
        }
    }
    spi_deselect(s);
    nvt_flash_set_status(s, nvt_flash_read_status(s, *reg(s, REG_COMMAND2),
                                                  2));
}

/*
 * SPI_PROG_CMD: write enable, load DATALEN bytes from memory at the
 * column and then the spare SRAM as the OOB, program the page, and poll
 * cmd 4 + COMMAND2[7:0] until it is done.
 */
static void nvt_flash_prog_cmd(NvtFlashState *s, uint32_t cmd, uint32_t len,
                               bool dma)
{
    uint32_t col = *reg(s, REG_COLADDR) & 0xFFF, i;
    uint8_t *spare = s->spare;

    spi_select(s);
    spi_xfer(s, cmd);
    spi_deselect(s);

    spi_select(s);
    spi_xfer(s, cmd >> 8);
    nvt_flash_send_col(s);
    if (col < NVT_NAND_PAGE) {
        len = MIN(len, NVT_NAND_PAGE - col);
        if (dma) {
            nvt_flash_dma_out(s, *reg(s, REG_DMASTART), len);
        } else {
            qemu_log_mask(LOG_UNIMP, "nvt-flash: SPI_PROG_CMD by PIO\n");
        }
        for (i = col + len; i < NVT_NAND_PAGE; i++) {
            spi_xfer(s, 0xFF);
        }
        for (i = 0; i < NVT_NAND_OOB; i++) {
            spi_xfer(s, spare[i]);
        }
    }
    spi_deselect(s);

    spi_select(s);
    spi_xfer(s, cmd >> 16);
    nvt_flash_send_addr(s);
    spi_deselect(s);
    nvt_flash_poll_status(s, (cmd >> 24) | (*reg(s, REG_COMMAND2) & 0xFF) << 8,
                          2);
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
    const bool nand = nvt_flash_nand_mode(s);
    uint32_t n;

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
        if (nand) {
            spi_xfer(s, cmd >> 8);
        }
        nvt_flash_set_status(s, spi_xfer(s, 0));
        break;
    case OP_READ_ID:
        nvt_flash_begin(s);
        spi_xfer(s, cmd);
        if (nand) {
            spi_xfer(s, 0);                 /* the address byte */
        }
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
        nvt_flash_dma_in(s, *reg(s, REG_DMASTART), len);
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
        nvt_flash_dma_out(s, *reg(s, REG_DMASTART), len);
        break;
    case OP_AUTO_STATUS:
        spi_deselect(s);
        nvt_flash_poll_status(s, cmd, nand ? 2 : 1);
        break;
    case OP_BLK_ERSE:
        spi_select(s);
        spi_xfer(s, cmd & 0xFF);
        nvt_flash_send_addr(s);
        spi_deselect(s);
        nvt_flash_poll_status(s, cmd >> 8, nand ? 2 : 1);
        break;
    case OP_DATA_RD3:
        nvt_flash_data_rd3(s, cmd, len, dma);
        break;
    case OP_SPI_PROG_CMD:
        nvt_flash_prog_cmd(s, cmd, len, dma);
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
    case REG_SPARE ... REG_SPARE + NVT_SPARE_SIZE - 4:
        return ldl_le_p(s->spare + (off - REG_SPARE));
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
    case REG_SPARE ... REG_SPARE + NVT_SPARE_SIZE - 4:
        stl_le_p(s->spare + (off - REG_SPARE), val);
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
    memset(s->lock, 0xFF, sizeof(s->lock));
    s->feat[NAND_FEAT_LOCK] = 0;
    s->feat[NAND_FEAT_CFG] = NAND_CFG_ECCEN | NAND_CFG_BUF;
    s->feat[NAND_FEAT_STS] = 0;
    qemu_irq_lower(s->irq);
}

/*
 * Which flash an image of len bytes is: a NOR of a power of two from 1 to
 * 32 MiB, or a NAND of 128 or 256 MiB, with or without the OOB.
 */
static bool nvt_flash_geometry(NvtFlashState *s, uint64_t len)
{
    uint64_t data = len;

    s->file_oob = false;
    if (len % NVT_NAND_RAW == 0) {
        data = len / NVT_NAND_RAW * NVT_NAND_PAGE;
        if (data == 128 * MiB || data == 256 * MiB) {
            s->file_oob = true;
        } else {
            data = len;
        }
    }
    if (data >= 1 * MiB && data <= 32 * MiB && !(data & (data - 1))) {
        s->nand = false;
        s->size = data;
        s->array_size = data;
        return true;
    }
    if (data == 128 * MiB || data == 256 * MiB) {
        s->nand = true;
        s->size = data;
        s->pages = data / NVT_NAND_PAGE;
        s->array_size = s->pages * NVT_NAND_RAW;
        return true;
    }
    return false;
}

static void nvt_flash_realize(DeviceState *dev, Error **errp)
{
    NvtFlashState *s = NVT_FLASH(dev);
    uint32_t p;
    off_t len;

    s->fd = -1;
    s->dirty_lo = UINT32_MAX;
    s->dirty_hi = 0;
    s->sr3 = SR3_RESET;     /* non-volatile, so not reset */
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
        if (len < 0 || !nvt_flash_geometry(s, len)) {
            error_setg(errp, "nvt-flash: %s is neither a SPI NOR image (a "
                       "power of two from 1 to 32 MiB) nor a SPI NAND one "
                       "(128 or 256 MiB, or 2112 bytes a 2 KiB page)",
                       s->flash_file);
            goto fail;
        }
        error_setg(&s->migration_blocker, "nvt-flash: flash-file %s is "
                   "written through; it cannot be migrated", s->flash_file);
        if (migrate_add_blocker(&s->migration_blocker, errp) < 0) {
            goto fail;
        }
    } else if (!nvt_flash_geometry(s, s->size) || s->file_oob) {
        error_setg(errp, "nvt-flash: size must be a power of two from 1 to "
                   "32 MiB (NOR), or 128 or 256 MiB (NAND)");
        goto fail;
    }
    s->array = g_malloc(s->array_size);
    memset(s->array, 0xFF, s->array_size);
    if (s->fd >= 0 && (s->file_oob || !s->nand) &&
        pread(s->fd, s->array, s->array_size, 0) != s->array_size) {
        error_setg(errp, "nvt-flash: cannot read %s", s->flash_file);
        goto fail;
    }
    /* A data-only NAND image: each page's data, its OOB left erased. */
    for (p = 0; s->fd >= 0 && s->nand && !s->file_oob && p < s->pages; p++) {
        if (pread(s->fd, s->array + p * NVT_NAND_RAW, NVT_NAND_PAGE,
                  (off_t)p * NVT_NAND_PAGE) != NVT_NAND_PAGE) {
            error_setg(errp, "nvt-flash: cannot read %s", s->flash_file);
            goto fail;
        }
    }
    if (!s->jedec && s->nand) {
        /* Winbond W25N01GV / W25N02KV, both in spinand.c's table. */
        s->jedec = s->size == 128 * MiB ? 0xEFAA21 : 0xEFAA22;
    } else if (!s->jedec) {
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
    .version_id = 3,
    .minimum_version_id = 1,        /* a NOR's v1 state still loads */
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
        /* array_size is size on a NOR, as v1 sized the buffer */
        VMSTATE_VBUFFER_UINT32(array, NvtFlashState, 1, NULL, array_size),
        VMSTATE_UINT8_ARRAY_V(cache, NvtFlashState, NVT_NAND_RAW, 2),
        VMSTATE_UINT8_ARRAY_V(feat, NvtFlashState, 3, 2),
        VMSTATE_UINT8_ARRAY_V(spare, NvtFlashState, NVT_SPARE_SIZE, 2),
        VMSTATE_UINT8_V(sr3, NvtFlashState, 3),
        VMSTATE_UINT8_ARRAY_V(lock, NvtFlashState, NVT_NOR_LOCK_BYTES, 3),
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
