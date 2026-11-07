/*
 * SigmaStar / MStar SPI NOR controller: ISP, FSP, QSPI and the XIU window.
 *
 * The boot NOR hangs off one controller with four faces
 * (boot/drivers/mstar/flash_isp/infinity2m/regSERFLASH.h):
 *
 *   ISP   RIU bank 0x0008 (0x1F001000)  byte-at-a-time master: write the
 *         opcode to 0x01 and bytes to 0x04, request a read byte with 0x0C
 *         and take it from 0x05, end the transfer with 0x08 (CECLR).
 *         0x15-0x17 are the read/write/command ready flags.
 *   FSP   RIU bank 0x0016 (0x1F002C00)  a three-command sequencer:
 *         write buffer 0x60-0x64 (10 bytes), read buffer 0x65-0x69,
 *         per-command write and read counts in nibbles of 0x6A/0x6B,
 *         0x6C control (bit 15 second command, bit 14 third command),
 *         0x6D fire, 0x6E done, 0x6F done clear.  The commands share one
 *         write buffer and one read buffer, each continuing where the
 *         previous left off.  0x78/0x79 the "outside" write buffer: with
 *         0x79 bit 12 set, write-buffer byte (0x79 & 0xFF) is replaced by
 *         0x78 bytes that BDMA has delivered -- U-Boot programs pages so
 *         (HAL_FSP_Write_BDMA: WREN; PP a2 a1 a0 <256 bytes>; RDSR).
 *   QSPI  RIU bank 0x0017 (0x1F002E00)  XIU read mode and timing; kept,
 *         but for 0x0A, the software chip select (below).
 *   XIU   0x14000000                    the flash mapped for reading,
 *         how the IPL and kernel read it besides the FSP.
 *
 * The later FSP-QSPI of the Ifado/Iford generation (the vendor
 * drivers/sstar/fsp_qspi/hal/iford/hal_fsp_qspi.c; "sw-cs" here) adds a
 * second write buffer at 0x70-0x72 (bytes 10-14), takes the outside size
 * as the byte count plus one, and has a software chip select: QSPI 0x0A
 * bit 0 enables it and bit 1 drives CS high.  While it is enabled the
 * sequencer leaves CS alone, so one transfer spans several fires; the
 * driver sends a quad page program's opcode and address in one fire and
 * its 256 data bytes from the outside buffer, with no write buffer, in
 * the next.
 *
 * The flash itself is a generic 3-byte-address SPI NOR answering with
 * the JEDEC ID "jedec"; erases and page programs complete at once and are
 * written through to flash-file, so the image is the flash.  The extended
 * address register (0xC5/0xC8) only reads back: 16 MiB needs no fourth
 * address byte.
 *
 * With "nand" the chip is a generic SPI NAND on the same four faces, the
 * vendor's drivers/mstar/spinand HAL: 0x13 loads a page into the chip's
 * page register, 0x03 or BDMA from the SPI port reads it by column, 0x02,
 * 0x32 and 0x84 fill it, 0x10 programs it and 0xD8 erases, with 0x0F/0x1F
 * reaching the protect (0xA0), feature (0xB0) and status (0xC0) registers.
 * BDMA's SPI source address is then a column, not a flash address.  The
 * file holds page + spare for every page, so bad-block marks persist.
 * Programs and erases complete at once and only clear bits or set them
 * all.  The on-die ECC is the status field alone: ecc-flip-page and
 * ecc-err-page report a corrected or an uncorrectable read of that page,
 * and fail-block fails programs and erases of that block; the data stay
 * as stored.  Reset leaves the BP bits set, as these parts power up.
 *
 * The driver opens a page load in one fire with the software chip select
 * off, and only then enables it for the data fires
 * (HAL_SPINAND_PROGRAM_BY_BDMA), which still reaches one transfer on
 * silicon.  So a load stays selected past its fire, until a later fire
 * without the software chip select, or the driver's own release.
 *
 * Copyright (c) 2026 OpenIPC.
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
#include "migration/vmstate.h"
#include "hw/ssi/sstar-fsp.h"
#include "migration/blocker.h"

/* ISP registers */
#define ISP_PASSWORD        0x00
#define ISP_SPI_COMMAND     0x01
#define ISP_SPI_WDATA       0x04
#define ISP_SPI_RDATA       0x05
#define ISP_SPI_CECLR       0x08
#define ISP_SPI_RDREQ       0x0C
#define ISP_RD_DATARDY      0x15
#define ISP_WR_DATARDY      0x16
#define ISP_WR_CMDRDY       0x17

/* FSP registers */
#define FSP_WBF             0x60
#define FSP_RBF             0x65
#define FSP_WBF_SIZE        0x6A
#define FSP_RBF_SIZE        0x6B
#define FSP_CTRL            0x6C
#define FSP_TRIGGER         0x6D
#define FSP_DONE_FLAG       0x6E
#define FSP_DONE_CLR        0x6F
#define FSP_WBF2            0x70        /* sw-cs: write buffer bytes 10-14 */
#define FSP_WBF_SIZE_OUT    0x78
#define FSP_WBF_OUTSIDE     0x79

#define FSP_CTRL_INT        (1u << 2)
#define FSP_CTRL_3RD        (1u << 14)
#define FSP_CTRL_2ND        (1u << 15)
#define FSP_OUT_ENABLE      (1u << 12)

/* QSPI registers */
#define QSPI_SW_MODE        0x0A
#define QSPI_SW_CS_EN       (1u << 0)
#define QSPI_SW_CS_HIGH     (1u << 1)

#define SR1_WIP             (1u << 0)
#define SR1_WEL             (1u << 1)

/* ---- the SPI NOR --------------------------------------------------- */

static void sstar_fsp_persist(SstarFspState *s)
{
    uint32_t off = s->dirty_lo, len;

    if (s->dirty_hi <= s->dirty_lo) {
        return;
    }
    len = s->dirty_hi - s->dirty_lo;
    s->dirty_lo = UINT32_MAX;
    s->dirty_hi = 0;
    if (!s->nand) {
        memory_region_set_dirty(&s->xiu, off, len);
    }
    while (s->fd >= 0 && len) {
        ssize_t n = pwrite(s->fd, s->array + off, len, off);

        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            error_report("sstar-fsp: write-through to %s failed at 0x%x: %s",
                         s->flash_file, off,
                         n < 0 ? strerror(errno) : "no progress");
            exit(1);
        }
        off += n;
        len -= n;
    }
}

static void sstar_fsp_dirty(SstarFspState *s, uint32_t off, uint32_t len)
{
    s->dirty_lo = MIN(s->dirty_lo, off);
    s->dirty_hi = MAX(s->dirty_hi, off + len);
}

static void sstar_fsp_erase(SstarFspState *s, uint32_t addr, uint32_t len)
{
    addr &= ~(len - 1) & (s->size - 1);
    memset(s->array + addr, 0xFF, len);
    sstar_fsp_dirty(s, addr, len);
}

/* ---- the SPI NAND -------------------------------------------------- */

#define NAND_REG_PROT       0xA0
#define NAND_REG_CONF       0xB0
#define NAND_REG_STAT       0xC0

#define NAND_PROT_BP        0x38
#define NAND_CONF_ECC_EN    (1u << 4)
#define NAND_CONF_BUF       (1u << 3)
#define NAND_STAT_WEL       (1u << 1)
#define NAND_STAT_E_FAIL    (1u << 2)
#define NAND_STAT_P_FAIL    (1u << 3)
#define NAND_STAT_ECC_SHIFT 4
#define NAND_STAT_ECC_MASK  (3u << NAND_STAT_ECC_SHIFT)

static uint32_t nand_page_raw(SstarFspState *s)
{
    return s->page_size + s->spare_size;
}

/* The bit above the page and its spare is the plane select. */
static uint32_t nand_col(SstarFspState *s, uint32_t col)
{
    return col & (s->page_size * 2 - 1);
}

static bool nand_cmd_is_load(uint8_t cmd)
{
    return cmd == 0x02 || cmd == 0x32 || cmd == 0x84 || cmd == 0x34;
}

static uint32_t nand_pages(SstarFspState *s)
{
    return s->size / nand_page_raw(s);
}

static void nand_page_read(SstarFspState *s)
{
    uint32_t page = s->addr;

    s->nand_page = page;
    s->feat_stat &= ~NAND_STAT_ECC_MASK;
    if (page >= nand_pages(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-fsp: NAND page 0x%x past the "
                      "end\n", page);
        memset(s->cache, 0xFF, nand_page_raw(s));
        return;
    }
    memcpy(s->cache, s->array + page * nand_page_raw(s), nand_page_raw(s));
    if (s->feat_conf & NAND_CONF_ECC_EN) {
        if (page == s->ecc_err_page) {
            s->feat_stat |= 2u << NAND_STAT_ECC_SHIFT;
        } else if (page == s->ecc_flip_page) {
            s->feat_stat |= 1u << NAND_STAT_ECC_SHIFT;
        }
    }
}

/*
 * Winbond's W25N parts (maker 0xEF) have a BUF bit in feature 0xB0, set at
 * power-up: with it clear a read from the cache runs on through the next
 * pages, their data only, and leaves the page register past the last one
 * (W25N01GV/W25N01JW datasheets, "Continuous Read Mode").  Iford's IPL
 * clears it to load a whole stage with one BDMA, the W25N01JW profile of
 * the SDK's flash_list-fb.sni telling it to.
 */
static bool nand_continuous(SstarFspState *s)
{
    return (s->nand_id >> (8 * ((32 - clz32(s->nand_id | 1) + 7) / 8 - 1)))
           == 0xEF && !(s->feat_conf & NAND_CONF_BUF);
}

static uint8_t nand_cache_next(SstarFspState *s)
{
    uint8_t b;

    if (!nand_continuous(s)) {
        return s->cache[nand_col(s, s->col++) % nand_page_raw(s)];
    }
    if (s->col >= s->page_size) {
        s->addr = s->nand_page + 1;
        nand_page_read(s);
        s->col = 0;
    }
    b = s->cache[s->col++];
    return b;
}

/* The write latch is spent either way; a refused operation reports a fail. */
static bool nand_may_write(SstarFspState *s, uint32_t page, uint8_t fail)
{
    bool wel = s->wel;

    s->feat_stat &= ~(NAND_STAT_E_FAIL | NAND_STAT_P_FAIL);
    s->wel = false;
    if (!wel) {
        return false;
    }
    if ((s->feat_prot & NAND_PROT_BP) || page >= nand_pages(s) ||
        page / s->block_pages == s->fail_block) {
        s->feat_stat |= fail;
        return false;
    }
    return true;
}

static void nand_page_program(SstarFspState *s)
{
    uint32_t raw = nand_page_raw(s);
    uint32_t off = s->addr * raw;
    uint32_t i;

    if (!nand_may_write(s, s->addr, NAND_STAT_P_FAIL)) {
        return;
    }
    for (i = 0; i < raw; i++) {
        s->array[off + i] &= s->cache[i];
    }
    sstar_fsp_dirty(s, off, raw);
}

static void nand_block_erase(SstarFspState *s)
{
    uint32_t page = s->addr & ~(s->block_pages - 1);
    uint32_t len = s->block_pages * nand_page_raw(s);

    if (!nand_may_write(s, page, NAND_STAT_E_FAIL)) {
        return;
    }
    memset(s->array + page * nand_page_raw(s), 0xFF, len);
    sstar_fsp_dirty(s, page * nand_page_raw(s), len);
}

static void nand_deselect(SstarFspState *s)
{
    switch (s->cmd) {
    case 0x06:                              /* WRITE ENABLE */
        s->wel = true;
        break;
    case 0x04:                              /* WRITE DISABLE */
        s->wel = false;
        break;
    case 0xFF:                              /* RESET */
        s->wel = false;
        s->feat_stat = 0;
        break;
    case 0x13:                              /* PAGE READ, to the register */
        if (s->pos >= 4) {
            nand_page_read(s);
        }
        break;
    case 0x10:                              /* PROGRAM EXECUTE */
        if (s->pos >= 4) {
            nand_page_program(s);
        }
        break;
    case 0xD8:                              /* BLOCK ERASE */
        if (s->pos >= 4) {
            nand_block_erase(s);
        }
        break;
    }
    sstar_fsp_persist(s);
}

static uint8_t nand_feature(SstarFspState *s, uint8_t reg)
{
    switch (reg) {
    case NAND_REG_PROT:
        return s->feat_prot;
    case NAND_REG_CONF:
        return s->feat_conf;
    case NAND_REG_STAT:
        return s->feat_stat | (s->wel ? NAND_STAT_WEL : 0);
    }
    return 0;
}

static uint8_t nand_xfer(SstarFspState *s, uint32_t pos, uint8_t tx)
{
    uint32_t raw = nand_page_raw(s);
    uint8_t rx = 0xFF;

    switch (s->cmd) {
    case 0x9F:                              /* READ ID, after one dummy */
        if (pos >= 2) {
            int n = (32 - clz32(s->nand_id | 1) + 7) / 8;
            int i = pos - 2;

            rx = i < n ? s->nand_id >> (8 * (n - 1 - i)) : 0;
        }
        break;
    case 0x0F:                              /* GET FEATURE */
        if (pos == 1) {
            s->addr = tx;
        } else {
            rx = nand_feature(s, s->addr);
        }
        break;
    case 0x1F:                              /* SET FEATURE */
        if (pos == 1) {
            s->addr = tx;
        } else if (pos == 2) {
            if (s->addr == NAND_REG_PROT) {
                s->feat_prot = tx;
            } else if (s->addr == NAND_REG_CONF) {
                s->feat_conf = tx;
            }
        }
        break;
    case 0x13: case 0x10: case 0xD8:        /* three page address bytes */
        if (pos <= 3) {
            s->addr = (s->addr << 8) | tx;
        }
        break;
    case 0x03: case 0x0B:                   /* READ FROM CACHE */
    case 0x3B: case 0x6B:                   /* dual, quad output */
        if (pos <= 2) {
            s->col = (s->col << 8) | tx;
        } else if (pos > 3) {
            rx = nand_cache_next(s);
        }
        break;
    case 0x02: case 0x32:                   /* PROGRAM LOAD: register to FF */
        if (pos == 1) {
            memset(s->cache, 0xFF, raw);
        }
        /* fall through */
    case 0x84: case 0x34:                   /* RANDOM LOAD: register kept */
        if (pos <= 2) {
            s->col = (s->col << 8) | tx;
        } else if (nand_col(s, s->col) < raw) {
            s->cache[nand_col(s, s->col++)] = tx;
        }
        break;
    case 0x06: case 0x04: case 0xFF:
        break;
    default:
        if (pos == 1) {
            qemu_log_mask(LOG_UNIMP, "sstar-fsp: NAND opcode 0x%02x\n",
                          s->cmd);
        }
        break;
    }
    return rx;
}

static void spi_select(SstarFspState *s)
{
    s->selected = true;
    s->pos = 0;
    s->addr = 0;
    s->col = 0;
}

static void spi_deselect(SstarFspState *s)
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
        return;
    }
    switch (s->cmd) {
    case 0x06:                              /* WREN */
        s->wel = true;
        break;
    case 0x04:                              /* WRDI */
        s->wel = false;
        break;
    case 0x02: case 0x32:                   /* PP, quad PP */
    case 0x01: case 0x31: case 0x11:        /* WRSR, WRSR2, WRSR3 */
    case 0xC5:                              /* WREAR */
    case 0x99:                              /* RST, after RSTEN (0x66) */
        s->wel = false;
        break;
    case 0x20:                              /* SE, 4 KiB */
    case 0x52:                              /* BE, 32 KiB */
    case 0xD8:                              /* BE, 64 KiB */
        if (s->wel && s->pos >= 4) {
            sstar_fsp_erase(s, s->addr, s->cmd == 0x20 ? 4 * KiB :
                            s->cmd == 0x52 ? 32 * KiB : 64 * KiB);
        }
        s->wel = false;
        break;
    case 0xC7: case 0x60:                   /* CE */
        if (s->wel) {
            sstar_fsp_erase(s, 0, s->size);
        }
        s->wel = false;
        break;
    }
    sstar_fsp_persist(s);
}

/*
 * One of a command's three address bytes; the extended address register
 * (WREAR) is address bits [31:24], which reach past 16 MiB.
 */
static void spi_addr_byte(SstarFspState *s, uint32_t pos, uint8_t tx)
{
    s->addr = (s->addr << 8) | tx;
    if (pos == 3) {
        s->addr = (s->addr & 0xFFFFFF) | (uint32_t)s->ear << 24;
    }
}

static uint8_t spi_xfer(SstarFspState *s, uint8_t tx)
{
    uint32_t pos = s->pos++;
    uint8_t rx = 0xFF;

    if (pos == 0) {
        s->cmd = tx;
        return 0xFF;
    }
    if (s->nand) {
        return nand_xfer(s, pos, tx);
    }
    switch (s->cmd) {
    case 0x03:                              /* READ */
    case 0x0B:                              /* FAST_READ, one dummy byte */
    case 0x3B: case 0x6B:                   /* dual / quad output read */
        if (pos <= 3) {
            spi_addr_byte(s, pos, tx);
        } else if (s->cmd == 0x03 || pos > 4) {
            rx = s->array[s->addr & (s->size - 1)];
            s->addr++;
        }
        break;
    case 0x02: case 0x32:                   /* PP, quad PP */
        if (pos <= 3) {
            spi_addr_byte(s, pos, tx);
        } else if (s->wel) {
            uint32_t a = s->addr & (s->size - 1);

            s->array[a] &= tx;
            sstar_fsp_dirty(s, a, 1);
            /* Wraps within the 256-byte page, as NOR does. */
            s->addr = (s->addr & ~0xFFu) | ((s->addr + 1) & 0xFF);
        }
        break;
    case 0x20: case 0x52: case 0xD8:
        if (pos <= 3) {
            spi_addr_byte(s, pos, tx);
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
    case 0x4B:                              /* unique ID: 4 dummy bytes */
        if (pos > 4) {
            rx = 0x51 + pos;
        }
        break;
    case 0x05:                              /* RDSR */
        rx = (s->sr[0] & ~(SR1_WIP | SR1_WEL)) | (s->wel ? SR1_WEL : 0);
        break;
    case 0x35:                              /* RDSR2 */
        rx = s->sr[1];
        break;
    case 0x15:                              /* RDSR3 */
        rx = s->sr[2];
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
    case 0x11:                              /* WRSR3 */
        if (s->wel && pos == 1) {
            s->sr[2] = tx;
        }
        break;
    case 0xC5:                              /* WREAR, after WREN */
        if (s->wel && pos == 1) {
            s->ear = tx;
        }
        break;
    case 0xC8:                              /* RDEAR */
        rx = s->ear;
        break;
    case 0x5A:                              /* SFDP: none */
        break;
    default:
        if (pos == 1) {
            qemu_log_mask(LOG_UNIMP, "sstar-fsp: SPI opcode 0x%02x\n",
                          s->cmd);
        }
        break;
    }
    return rx;
}

void sstar_fsp_read_flash(SstarFspState *s, uint32_t off, uint8_t *buf,
                          uint32_t len)
{
    if (s->nand && nand_continuous(s)) {
        /* BDMA asks in chunks, its offset running on: the stream does */
        if (off < s->page_size) {
            s->col = off;
        }
        while (len--) {
            *buf++ = nand_cache_next(s);
        }
        return;
    }
    if (s->nand) {
        while (len--) {
            *buf++ = s->cache[nand_col(s, off++) % nand_page_raw(s)];
        }
        return;
    }
    while (len--) {
        *buf++ = s->array[off++ & (s->size - 1)];
    }
}

static void fsp_fire(SstarFspState *s);
static uint32_t fsp_outside_len(SstarFspState *s);

void sstar_fsp_push_outside(SstarFspState *s, const uint8_t *buf,
                            uint32_t len)
{
    len = MIN(len, SSTAR_FSP_OUTSIDE_MAX - s->outside_len);
    memcpy(s->outside + s->outside_len, buf, len);
    s->outside_len += len;
    if (s->fire_wait && s->outside_len >= fsp_outside_len(s)) {
        fsp_fire(s);
    }
}

/* ---- FSP ------------------------------------------------------------ */

static uint8_t fsp_wbf(SstarFspState *s, int i)
{
    uint16_t w;

    if (i >= 10) {
        i -= 10;
        w = s->fsp[FSP_WBF2 + i / 2];
        return i & 1 ? w >> 8 : w & 0xFF;
    }
    w = s->fsp[FSP_WBF + i / 2];

    return i & 1 ? w >> 8 : w & 0xFF;
}

static void fsp_update_irq(SstarFspState *s)
{
    qemu_set_irq(s->irq, s->done && (s->fsp[FSP_CTRL] & FSP_CTRL_INT));
}

/* With the software chip select on, CS follows QSPI 0x0A, not the fires. */
static bool fsp_sw_cs(SstarFspState *s)
{
    return s->sw_cs && (s->qspi[QSPI_SW_MODE] & QSPI_SW_CS_EN);
}

/* How many bytes the outside buffer holds: the FSP-QSPI counts one more. */
static uint32_t fsp_outside_len(SstarFspState *s)
{
    uint32_t out_len = s->fsp[FSP_WBF_SIZE_OUT];

    return s->out_plus_one && out_len ? out_len - 1 : out_len;
}

/* The bytes BDMA queued, in place of write buffer byte (0x79 & 0xFF). */
static void fsp_send_outside(SstarFspState *s)
{
    uint32_t out_len = fsp_outside_len(s);
    uint32_t k;
    if (s->outside_len < out_len) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-fsp: outside buffer has %u "
                      "of %u bytes\n", s->outside_len, out_len);
    }
    for (k = 0; k < out_len; k++) {
        spi_xfer(s, k < s->outside_len ? s->outside[k] : 0xFF);
    }
    s->outside_len = 0;
}

static void fsp_fire(SstarFspState *s)
{
    uint16_t ctrl = s->fsp[FSP_CTRL];
    uint16_t out = s->fsp[FSP_WBF_OUTSIDE];
    bool out_pending = out & FSP_OUT_ENABLE;
    bool sw_cs = fsp_sw_cs(s);
    int wpos = 0, rpos = 0, n, j;

    /*
     * The Ifado/Iford driver fires first and starts BDMA after; the
     * sequencer waits for the bytes.  So does this, until
     * sstar_fsp_push_outside() has them.
     */
    s->fire_wait = false;
    if (out_pending && s->outside_len < fsp_outside_len(s)) {
        s->fire_wait = true;
        return;
    }
    memset(s->rbf, 0, sizeof(s->rbf));
    for (n = 0; n < 3; n++) {
        int wsz = (s->fsp[FSP_WBF_SIZE] >> (4 * n)) & 0xF;
        int rsz = (s->fsp[FSP_RBF_SIZE] >> (4 * n)) & 0xF;

        if ((n == 1 && !(ctrl & FSP_CTRL_2ND)) ||
            (n == 2 && !(ctrl & FSP_CTRL_3RD))) {
            continue;
        }
        if (!sw_cs) {
            spi_deselect(s);        /* a NAND page load left open */
            spi_select(s);
        } else if (!s->selected) {
            qemu_log_mask(LOG_GUEST_ERROR, "sstar-fsp: fire with the "
                          "software chip select high\n");
            break;
        }
        for (j = 0; j < wsz; j++, wpos++) {
            if (out_pending && wpos == (out & 0xFF)) {
                fsp_send_outside(s);
                out_pending = false;
                continue;
            }
            spi_xfer(s, wpos < 15 ? fsp_wbf(s, wpos) : 0xFF);
        }
        /* An outside buffer past the write buffer, as with no write buffer */
        if (out_pending && wpos == (out & 0xFF)) {
            fsp_send_outside(s);
            out_pending = false;
        }
        for (j = 0; j < rsz; j++, rpos++) {
            uint8_t b = spi_xfer(s, 0);

            if (rpos < sizeof(s->rbf)) {
                s->rbf[rpos] = b;
            }
        }
        if (!sw_cs && !(s->nand && nand_cmd_is_load(s->cmd))) {
            spi_deselect(s);
        }
    }
    s->done = true;
    fsp_update_irq(s);
}

static void qspi_reg_write(SstarFspState *s, int reg, uint16_t v)
{
    s->qspi[reg] = v;
    if (reg != QSPI_SW_MODE || !s->sw_cs) {
        return;
    }
    if ((v & QSPI_SW_CS_EN) && !(v & QSPI_SW_CS_HIGH)) {
        if (!s->selected) {
            spi_select(s);
        }
    } else {
        spi_deselect(s);
    }
}

static uint16_t fsp_reg_read(SstarFspState *s, int reg)
{
    switch (reg) {
    case FSP_RBF ... FSP_RBF + 4:
        return s->rbf[(reg - FSP_RBF) * 2] |
               (s->rbf[(reg - FSP_RBF) * 2 + 1] << 8);
    case FSP_DONE_FLAG:
        return s->done;
    default:
        return s->fsp[reg];
    }
}

static void fsp_reg_write(SstarFspState *s, int reg, uint16_t v)
{
    s->fsp[reg] = v;
    switch (reg) {
    case FSP_TRIGGER:
        if (v & 1) {
            fsp_fire(s);
        }
        s->fsp[reg] = 0;
        break;
    case FSP_DONE_CLR:
        if (v & 1) {
            s->done = false;
            fsp_update_irq(s);
        }
        s->fsp[reg] = 0;
        break;
    case FSP_CTRL:
        fsp_update_irq(s);
        break;
    }
}

/* ---- ISP ------------------------------------------------------------ */

static uint16_t isp_reg_read(SstarFspState *s, int reg)
{
    switch (reg) {
    case ISP_SPI_RDATA:
        return s->isp_rdata;
    case ISP_RD_DATARDY:
    case ISP_WR_DATARDY:
    case ISP_WR_CMDRDY:
        return 1;
    default:
        return s->isp[reg];
    }
}

static void isp_reg_write(SstarFspState *s, int reg, uint16_t v)
{
    s->isp[reg] = v;
    switch (reg) {
    case ISP_SPI_COMMAND:
        spi_deselect(s);
        spi_select(s);
        spi_xfer(s, v & 0xFF);
        break;
    case ISP_SPI_WDATA:
        if (!s->selected) {
            spi_select(s);
        }
        spi_xfer(s, v & 0xFF);
        break;
    case ISP_SPI_RDREQ:
        if (v & 1) {
            s->isp_rdata = s->selected ? spi_xfer(s, 0xFF) : 0xFF;
        }
        break;
    case ISP_SPI_CECLR:
        if (v & 1) {
            spi_deselect(s);
        }
        break;
    }
}

/* ---- MMIO ----------------------------------------------------------- */

typedef enum { BANK_ISP, BANK_FSP, BANK_QSPI } FspBank;

static uint16_t bank_read(SstarFspState *s, FspBank b, int reg)
{
    switch (b) {
    case BANK_ISP:
        return isp_reg_read(s, reg);
    case BANK_FSP:
        return fsp_reg_read(s, reg);
    default:
        return s->qspi[reg];
    }
}

static void bank_write(SstarFspState *s, FspBank b, int reg, uint16_t v)
{
    switch (b) {
    case BANK_ISP:
        isp_reg_write(s, reg, v);
        break;
    case BANK_FSP:
        fsp_reg_write(s, reg, v);
        break;
    default:
        qspi_reg_write(s, reg, v);
        break;
    }
}

static uint64_t riu_read(SstarFspState *s, FspBank b, hwaddr off,
                         unsigned size)
{
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = bank_read(s, b, (off >> 2) & 0x7F);
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void riu_write(SstarFspState *s, FspBank b, hwaddr off,
                      uint64_t val, unsigned size)
{
    int reg = (off >> 2) & 0x7F;
    uint16_t v = val;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        /* Byte writes merge into the stored word, not the live one. */
        uint16_t cur = b == BANK_ISP ? s->isp[reg] :
                       b == BANK_FSP ? s->fsp[reg] : s->qspi[reg];

        v = (off & 1) ? (cur & 0x00FF) | ((val & 0xFF) << 8)
                      : (cur & 0xFF00) | (val & 0xFF);
    }
    bank_write(s, b, reg, v);
}

#define DEFINE_BANK_OPS(name, bank)                                         \
    static uint64_t name##_read(void *o, hwaddr off, unsigned size)        \
    {                                                                       \
        return riu_read(o, bank, off, size);                                \
    }                                                                       \
    static void name##_write(void *o, hwaddr off, uint64_t v, unsigned sz) \
    {                                                                       \
        riu_write(o, bank, off, v, sz);                                     \
    }                                                                       \
    static const MemoryRegionOps name##_ops = {                             \
        .read = name##_read,                                                \
        .write = name##_write,                                              \
        .endianness = DEVICE_LITTLE_ENDIAN,                                 \
        .valid.min_access_size = 1,                                         \
        .valid.max_access_size = 4,                                         \
    };

DEFINE_BANK_OPS(isp, BANK_ISP)
DEFINE_BANK_OPS(fsp, BANK_FSP)
DEFINE_BANK_OPS(qspi, BANK_QSPI)

/* XIU window writes: the flash is not writable through it. */
static void xiu_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR, "sstar-fsp: write to the XIU window at "
                  "0x%" HWADDR_PRIx "\n", off);
}

static uint64_t xiu_read(void *opaque, hwaddr off, unsigned size)
{
    SstarFspState *s = opaque;
    uint8_t buf[8] = {};
    uint64_t v = 0;
    unsigned i;

    if (s->nand) {
        sstar_fsp_read_flash(s, off, buf, size);
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)(s->nand ? buf[i] :
                        s->array[(off + i) & (s->size - 1)]) << (8 * i);
    }
    return v;
}

static const MemoryRegionOps xiu_ops = {
    .read = xiu_read,
    .write = xiu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void sstar_fsp_reset(DeviceState *dev)
{
    SstarFspState *s = SSTAR_FSP(dev);

    memset(s->isp, 0, sizeof(s->isp));
    memset(s->fsp, 0, sizeof(s->fsp));
    memset(s->qspi, 0, sizeof(s->qspi));
    memset(s->rbf, 0, sizeof(s->rbf));
    memset(s->sr, 0, sizeof(s->sr));
    s->ear = 0;
    s->feat_prot = NAND_PROT_BP;
    s->feat_conf = NAND_CONF_ECC_EN;
    if (nand_continuous(s)) {
        s->feat_conf |= NAND_CONF_BUF;      /* W25N: buffer mode at power-up */
    }
    s->feat_stat = 0;
    s->col = 0;
    s->fire_wait = false;
    s->done = false;
    s->wel = false;
    s->selected = false;
    s->outside_len = 0;
    s->dirty_lo = UINT32_MAX;
    s->dirty_hi = 0;
    fsp_update_irq(s);
}

static void sstar_fsp_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    SstarFspState *s = SSTAR_FSP(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    uint32_t nand_block = (s->page_size + s->spare_size) * s->block_pages;
    ssize_t got;
    off_t len;

    s->fd = -1;
    if (s->flash_file && *s->flash_file) {
        s->fd = open(s->flash_file, O_RDWR);
        if (s->fd < 0) {
            error_setg_errno(errp, errno, "sstar-fsp: cannot open %s",
                             s->flash_file);
            return;
        }
        /*
         * Two machines on one image would each write back from their own
         * copy, undoing the other's erases.
         */
        if (qemu_lock_fd(s->fd, 0, 0, true)) {
            error_setg(errp, "sstar-fsp: %s is in use by another process",
                       s->flash_file);
            goto fail;
        }
        len = lseek(s->fd, 0, SEEK_END);
        if (s->nand) {
            if (!len || len > 1 * GiB || len % nand_block) {
                error_setg(errp, "sstar-fsp: %s is not whole NAND blocks of "
                           "%u bytes with their spare; "
                           "tools/sstar-nand-image.py makes one",
                           s->flash_file, nand_block);
                goto fail;
            }
        } else if (len < 1 * MiB || len > 32 * MiB || (len & (len - 1))) {
            error_setg(errp, "sstar-fsp: %s is not a power-of-two size "
                       "from 1 to 32 MiB", s->flash_file);
            goto fail;
        }
        error_setg(&s->migration_blocker, "sstar-fsp: flash-file %s is "
                   "written through; it cannot be migrated", s->flash_file);
        if (migrate_add_blocker(&s->migration_blocker, errp) < 0) {
            goto fail;
        }
        s->size = len;
    }
    if (s->nand) {
        if (s->fd < 0) {
            s->size = 1024 * nand_block;
            error_setg(&s->migration_blocker, "sstar-fsp: the NAND array "
                       "cannot be migrated");
            if (migrate_add_blocker(&s->migration_blocker, errp) < 0) {
                goto fail;
            }
        }
        memory_region_init_io(&s->xiu, OBJECT(s), &xiu_ops, s,
                              "sstar-fsp.xiu", 16 * MiB);
        s->array = g_malloc(s->size);
        s->cache = g_malloc(s->page_size + s->spare_size);
        memset(s->cache, 0xFF, s->page_size + s->spare_size);
    } else {
        if (s->size < 1 * MiB || s->size > 32 * MiB ||
            (s->size & (s->size - 1))) {
            error_setg(errp, "sstar-fsp: size must be a power of two from 1 "
                       "to 32 MiB");
            goto fail;
        }

        /* Reads straight from the array; writes trap. */
        if (!memory_region_init_rom_device(&s->xiu, OBJECT(s), &xiu_ops, s,
                                           "sstar-fsp.xiu", s->size, errp)) {
            goto fail;
        }
        s->array = memory_region_get_ram_ptr(&s->xiu);
    }
    memset(s->array, 0xFF, s->size);
    if (s->fd >= 0) {
        got = pread(s->fd, s->array, s->size, 0);
        if (got != s->size) {
            if (got < 0) {
                error_setg_errno(errp, errno, "sstar-fsp: cannot read %s",
                                 s->flash_file);
            } else {
                error_setg(errp, "sstar-fsp: short read of %s",
                           s->flash_file);
            }
            goto fail;
        }
    }

    memory_region_init_io(&s->isp_io, OBJECT(s), &isp_ops, s,
                          "sstar-fsp.isp", 0x200);
    memory_region_init_io(&s->fsp_io, OBJECT(s), &fsp_ops, s,
                          "sstar-fsp.fsp", 0x200);
    memory_region_init_io(&s->qspi_io, OBJECT(s), &qspi_ops, s,
                          "sstar-fsp.qspi", 0x200);
    sysbus_init_mmio(sbd, &s->isp_io);
    sysbus_init_mmio(sbd, &s->fsp_io);
    sysbus_init_mmio(sbd, &s->qspi_io);
    sysbus_init_mmio(sbd, &s->xiu);
    sysbus_init_irq(sbd, &s->irq);
    return;

fail:
    if (s->fd >= 0) {
        close(s->fd);
        s->fd = -1;
    }
}

static const Property sstar_fsp_props[] = {
    DEFINE_PROP_STRING("flash-file", SstarFspState, flash_file),
    /* Macronix MX25L12835F, a common 16 MiB part on these boards. */
    DEFINE_PROP_UINT32("jedec", SstarFspState, jedec, 0xC22018),
    DEFINE_PROP_UINT32("size", SstarFspState, size, 16 * MiB),
    /* the Ifado/Iford-generation FSP-QSPI: software chip select */
    DEFINE_PROP_BOOL("sw-cs", SstarFspState, sw_cs, false),
    /* the FSP-QSPI driver writes the "outside" size as the count + 1 */
    DEFINE_PROP_BOOL("out-plus-one", SstarFspState, out_plus_one, false),
    /* a SPI NAND in place of the NOR; 128 MiB unless the file says more */
    DEFINE_PROP_BOOL("nand", SstarFspState, nand, false),
    DEFINE_PROP_UINT32("nand-id", SstarFspState, nand_id, 0xC8F1),
    DEFINE_PROP_UINT32("page-size", SstarFspState, page_size, 2048),
    DEFINE_PROP_UINT32("spare-size", SstarFspState, spare_size, 64),
    DEFINE_PROP_UINT32("block-pages", SstarFspState, block_pages, 64),
    DEFINE_PROP_UINT32("ecc-flip-page", SstarFspState, ecc_flip_page,
                       UINT32_MAX),
    DEFINE_PROP_UINT32("ecc-err-page", SstarFspState, ecc_err_page,
                       UINT32_MAX),
    DEFINE_PROP_UINT32("fail-block", SstarFspState, fail_block, UINT32_MAX),
};

/*
 * The NOR array is the XIU ROM device's RAM, which migrates by itself; a
 * file-backed flash or a NAND array blocks migration.
 */
static const VMStateDescription vmstate_sstar_fsp = {
    .name = TYPE_SSTAR_FSP,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(isp, SstarFspState, 0x80),
        VMSTATE_UINT16_ARRAY(fsp, SstarFspState, 0x80),
        VMSTATE_UINT16_ARRAY(qspi, SstarFspState, 0x80),
        VMSTATE_UINT8_ARRAY(rbf, SstarFspState, 16),
        VMSTATE_BOOL(done, SstarFspState),
        VMSTATE_UINT8_ARRAY(sr, SstarFspState, 3),
        VMSTATE_BOOL(wel, SstarFspState),
        VMSTATE_UINT8(cmd, SstarFspState),
        VMSTATE_UINT32(addr, SstarFspState),
        VMSTATE_UINT32(pos, SstarFspState),
        VMSTATE_BOOL(selected, SstarFspState),
        VMSTATE_UINT8(isp_rdata, SstarFspState),
        VMSTATE_UINT8_ARRAY(outside, SstarFspState, SSTAR_FSP_OUTSIDE_MAX),
        VMSTATE_UINT32(outside_len, SstarFspState),
        VMSTATE_UINT8(ear, SstarFspState),
        VMSTATE_BOOL(fire_wait, SstarFspState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_fsp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_fsp_realize;
    device_class_set_legacy_reset(dc, sstar_fsp_reset);
    device_class_set_props(dc, sstar_fsp_props);
    dc->vmsd = &vmstate_sstar_fsp;
}

static const TypeInfo sstar_fsp_info = {
    .name           = TYPE_SSTAR_FSP,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarFspState),
    .class_init     = sstar_fsp_class_init,
};

static void sstar_fsp_register_types(void)
{
    type_register_static(&sstar_fsp_info);
}

type_init(sstar_fsp_register_types)
