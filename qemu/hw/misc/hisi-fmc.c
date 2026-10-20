/*
 * HiSilicon HiFMC V100 Flash Memory Controller emulation.
 *
 * Emulates the FMC found on Hi3516CV300 / Hi3516EV300 / Goke-V500 SoCs,
 * with a RAM-backed SPI NOR flash (Winbond W25Q64, 8 MiB) or SPI NAND
 * flash (GigaDevice GD5F1GM7, 128 MiB).  Supports both register-mode
 * (small reads via memory window) and DMA-mode transfers.
 *
 * The flash-type property selects which flash is physically present:
 *   0 = SPI NOR  (default, W25Q64 8 MiB)
 *   1 = SPI NAND (GD5F1GM7 128 MiB, 2K page + 128B OOB)
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "system/dma.h"
#include <sys/stat.h>

/* ── Register offsets ────────────────────────────────────────────────── */

#define FMC_CFG             0x00
#define FMC_GLOBAL_CFG      0x04
#define FMC_SPI_TIMING_CFG  0x08
#define FMC_INT             0x18
#define FMC_INT_EN          0x1C
#define FMC_INT_CLR         0x20
#define FMC_CMD             0x24
#define FMC_ADDRH           0x28
#define FMC_ADDRL           0x2C
#define FMC_OP_CFG          0x30
#define FMC_SPI_OP_ADDR     0x34
#define FMC_DATA_NUM        0x38
#define FMC_OP              0x3C
#define FMC_DMA_LEN         0x40
#define FMC_DMA_AHB_CTRL    0x48
#define FMC_DMA_SADDR_D0    0x4C
#define FMC_DMA_SADDR_OOB   0x5C
#define FMC_OP_CTRL         0x68
#define FMC_STATUS          0xAC
#define FMC_VERSION         0xBC
#define FMC_DMA_SADDRH_D0   0x200
#define FMC_DMA_SADDRH_OOB  0x210

/* FMC_CFG bits */
#define FMC_CFG_FLASH_SEL_SHIFT 1
#define FMC_CFG_FLASH_SEL_MASK  (0x3 << FMC_CFG_FLASH_SEL_SHIFT)
#define FLASH_TYPE_SPI_NOR      0
#define FLASH_TYPE_SPI_NAND     1

/* FMC_OP bits */
#define FMC_OP_REG_OP_START     BIT(0)
#define FMC_OP_READ_STATUS_EN   BIT(1)
#define FMC_OP_READ_DATA_EN     BIT(2)
#define FMC_OP_WRITE_DATA_EN    BIT(5)
#define FMC_OP_ADDR_EN          BIT(6)
#define FMC_OP_CMD1_EN          BIT(7)
/* fmc100 shuffles OP: bit1 is ADDR_EN there, not READ_STATUS_EN. */
#define FMC100_OP_ADDR_EN       BIT(1)

/* FMC_OP_CFG bits */
#define FMC_OP_CFG_FM_CS_SHIFT  11
#define FMC_OP_CFG_MEM_IF_SHIFT 7
#define FMC_OP_CFG_MEM_IF_MASK  (0x7 << FMC_OP_CFG_MEM_IF_SHIFT)
#define FMC_OP_CFG_FM_CS_MASK   (0x3 << FMC_OP_CFG_FM_CS_SHIFT)

/* FMC_OP_CTRL bits */
#define FMC_OP_CTRL_DMA_OP_READY BIT(0)
#define FMC_OP_CTRL_RW_OP        BIT(1)
#define FMC_OP_CTRL_DMA_OP       BIT(2)
#define FMC_OP_CTRL_RD_OP_SEL_SHIFT 4
#define FMC_OP_CTRL_RD_OPCODE_SHIFT 16

/* FMC_INT bits */
#define FMC_INT_OP_DONE     BIT(0)

/*
 * Register-map variant.  The Goke xmorca / GK7206 "lotus,fmc" (fmc100) is the
 * same controller IP as the HiFMC V100 modelled above, but its register block
 * is shuffled to a different offset layout (and reports VERSION 0x200 instead
 * of 0x100).  The *operational* model is byte-for-byte identical — register-
 * mode ops are triggered by the OP register's START bit (bit 0 in both), DMA
 * ops by OP_CTRL's DMA_OP_READY bit (bit 0 in both, RW_OP = bit 1 in both),
 * and results are served from the memory window.  So we translate an fmc100
 * offset back to the canonical HiFMC offset and reuse one set of handlers.
 */
#define FMC_VARIANT_HIFMC   0
#define FMC_VARIANT_FMC100  1

/* fmc100 register offsets (drivers/mtd/fmc100/fmc100_reg.h), control base is
 * the DT "control" reg (0x10000010 on xmorca). */
static hwaddr fmc100_to_canonical(hwaddr off)
{
    switch (off) {
    case 0x00: return FMC_SPI_TIMING_CFG; /* TIMING_SPI_CFG */
    case 0x08: return FMC_CFG;
    case 0x0c: return FMC_GLOBAL_CFG;
    case 0x10: return FMC_INT;
    case 0x14: return FMC_INT_EN;
    case 0x28: return FMC_INT_CLR;
    case 0x20: return FMC_ADDRH;
    case 0x24: return FMC_ADDRL;
    case 0x2c: return FMC_CMD;
    case 0x30: return FMC_DATA_NUM;
    case 0x34: return FMC_OP;
    case 0x38: return FMC_OP_CFG;
    case 0x40: return FMC_DMA_AHB_CTRL;
    case 0x44: return FMC_DMA_SADDR_D0;
    case 0x48: return FMC_DMA_LEN;
    case 0x54: return FMC_DMA_SADDR_OOB;
    case 0x60: return FMC_OP_CTRL;
    case 0xa4: return FMC_STATUS;
    case 0xb4: return FMC_VERSION;
    default:   return (hwaddr)-1;  /* unmapped: read 0 / ignore write */
    }
}

/* SPI commands (shared NOR + NAND) */
#define SPI_CMD_WRITE_ENABLE  0x06
#define SPI_CMD_READ_STATUS   0x05
#define SPI_CMD_READ_ID       0x9F
#define SPI_CMD_READ          0x03
#define SPI_CMD_FAST_READ     0x0B
#define SPI_CMD_PAGE_PROGRAM  0x02
#define SPI_CMD_SECTOR_ERASE  0xD8
#define SPI_CMD_CHIP_ERASE    0xC7

/* SPI NAND specific commands */
#define SPI_CMD_RESET         0xFF
#define SPI_CMD_GET_FEATURES  0x0F
#define SPI_CMD_SET_FEATURE   0x1F

/* NAND feature register addresses */
#define NAND_FEATURE_PROTECT  0xA0
#define NAND_FEATURE_FEATURE  0xB0
#define NAND_FEATURE_STATUS   0xC0

/* SPI NOR additional commands */
#define SPI_CMD_WRITE_DISABLE 0x04
#define SPI_CMD_WRITE_STATUS1 0x01
#define SPI_CMD_READ_STATUS2  0x35
#define SPI_CMD_WRITE_STATUS2 0x31
#define SPI_CMD_READ_STATUS3  0x15
#define SPI_CMD_WRITE_STATUS3 0x11
#define SPI_CMD_READ_SFDP     0x5A

/* SPI status register bits */
#define SPI_SR_WEL          BIT(1)

/* NOR flash identity defaults: Winbond W25Q64 (8 MiB) */
#define NOR_JEDEC_0         0xEF    /* Winbond */
#define NOR_JEDEC_1         0x40
#define NOR_JEDEC_2_8M      0x17    /* 64Mbit = 8 MiB (W25Q64) */
#define NOR_JEDEC_2_16M     0x18    /* 128Mbit = 16 MiB (W25Q128) */
#define NOR_FLASH_SIZE      (8 * 1024 * 1024)
#define NOR_SECTOR_SIZE     (64 * 1024)
#define NOR_PAGE_SIZE       256

/* NAND flash identity: GigaDevice GD5F1GM7UEYIG (128 MiB, on-die 24b/1K ECC)
 * READ_ID (0x9F) returns mfr 0xC8 (GigaDevice) + device 0x91; the hifmc100
 * driver compares id_len = 2 bytes.  Override with the `flash-jedec`
 * property (e.g. 0xC8AA21 to emulate a Winbond W25N01GV instead). */
#define NAND_JEDEC_0        0xC8    /* GigaDevice */
#define NAND_JEDEC_1        0x91    /* GD5F1GM7 device ID */
#define NAND_JEDEC_2        0x00
#define NAND_FLASH_SIZE     (128 * 1024 * 1024)
#define NAND_PAGE_SIZE      2048
#define NAND_OOB_SIZE       128     /* GD5F1GM7 spare area */
#define NAND_PAGES_PER_BLOCK 64
#define NAND_BLOCK_SIZE     (NAND_PAGE_SIZE * NAND_PAGES_PER_BLOCK)
#define NAND_BLOCK_COUNT    1024
#define NAND_TOTAL_PAGES    (NAND_BLOCK_COUNT * NAND_PAGES_PER_BLOCK)

#define IOBUF_SIZE          256
#define CTRL_REG_SIZE       0x1000
#define MEM_WINDOW_SIZE     0x1000000   /* 16 MiB — covers full SPI NOR flash */

/* ── Device state ────────────────────────────────────────────────────── */

#define TYPE_HISI_FMC "hisi-fmc"
OBJECT_DECLARE_SIMPLE_TYPE(HisiFmcState, HISI_FMC)

struct HisiFmcState {
    SysBusDevice parent_obj;

    MemoryRegion ctrl_iomem;
    MemoryRegion mem_iomem;
    qemu_irq irq;                 /* op/DMA-done IRQ (fmc100 uses it; V100 polls) */

    /* Configuration properties */
    uint32_t variant;             /* FMC_VARIANT_HIFMC (default) or _FMC100 */
    uint32_t flash_type;          /* 0=NOR, 1=NAND — type of the flash-file image */
    char    *flash_file;          /* optional: path to flash dump to load */
    char    *nand_file;           /* optional: 2nd image for dual NOR+NAND boards
                                   * (flash-file is NOR u-boot+env, nand-file is
                                   * the SPI-NAND rootfs).  See widgetii#115. */
    uint32_t flash_jedec;         /* optional: NOR JEDEC ID override */
    uint32_t nand_jedec;          /* optional: NAND READ_ID override (e.g. 0xEFAA21
                                   * to present W25N01GV instead of GD5F1GM7) */
    uint32_t nand_oob_size;       /* spare bytes/page (128 GD5F1GM7, 64 W25N01GV) */

    /* Which chips are physically present (dual-flash capable) + their
     * chip-selects.  The guest selects a CS via FMC_OP_CFG bits[12:11]
     * (OP_CFG_FM_CS); a chip must only answer on its own CS, otherwise the
     * NOR scan phantom-detects the NOR on every CS and claims them all
     * (shared hifmc_cs_user[]), starving the NAND of a free CS. */
    bool     has_nor;
    bool     has_nand;
    uint32_t nor_cs;              /* NOR chip-select (boot flash = 0) */
    uint32_t nand_cs;             /* NAND chip-select (0 alone, 1 if NOR present) */

    /* Control registers */
    uint32_t cfg;
    uint32_t global_cfg;
    uint32_t spi_timing;
    uint32_t fmc_int;
    uint32_t int_en;
    uint32_t cmd;
    uint32_t addrh;
    uint32_t addrl;
    uint32_t op;                  /* last FMC_OP written (ADDR_EN et al) */
    uint32_t op_cfg;
    uint32_t spi_op_addr;
    uint32_t data_num;
    uint32_t dma_len;
    uint32_t dma_saddr;
    uint32_t dma_ahb_ctrl;
    uint32_t dma_saddr_oob;
    uint32_t dma_saddrh_d0;
    uint32_t dma_saddrh_oob;
    uint32_t op_ctrl;
    uint32_t status;

    /* SPI flash state */
    uint8_t  sr;
    uint8_t  sr2;                 /* Status Register-2 (QE bit at bit 1) */
    uint8_t  sr3;                 /* Status Register-3 */
    uint8_t *flash;               /* SPI NOR data (XIP-able boot/env) */
    uint32_t flash_size;

    /* NAND-specific state */
    uint8_t *nand_data;           /* SPI NAND page data */
    uint32_t nand_size;           /* NAND data area size */
    uint8_t *nand_oob;            /* OOB area: nand_oob_size per page */
    uint8_t  nand_feature_protect; /* feature reg 0xA0 */
    uint8_t  nand_feature_config;  /* feature reg 0xB0 */

    /* WPS (Write Protection Scheme) per-block lock state.
     * When SR3 bit 2 (WPS) is set, protection uses individual block locks
     * instead of the global BP bits.  Each 64KB block has an independent
     * lock bit.
     * SPI commands: 0x36 = lock, 0x39 = unlock, 0x3D = read lock status.
     *
     * block_locked[]: current lock state (1=locked, 0=unlocked)
     * block_ever_unlocked[]: tracks which blocks firmware explicitly unlocked.
     *   Blocks that were never individually unlocked are considered truly
     *   read-only (e.g. romfs, usr partitions).  This lets us protect against
     *   errant erases while allowing writes to blocks the firmware intends
     *   to use (env, mtd) even during lock/unlock sequencing gaps. */
    uint8_t *block_locked;
    uint8_t *block_ever_unlocked;
    uint32_t num_blocks;

    /* When true, the chip starts in the as-shipped factory-locked state:
     * SR3.WPS = 1, every block locked, none ever-unlocked.  Mirrors what
     * Xiongmai-flashed Winbond W25Q128s come out of the factory with —
     * lets CI exercise the kernel/U-Boot/agent recovery-unlock path
     * (Winbond Global Block Unlock 0x98 + clear SR3.WPS) instead of the
     * default already-unlocked-by-runtime-firmware behaviour.  See
     * openhisilicon#83. */
    bool     nor_wps_locked;

    /* When true, the NOR answers only single-I/O reads: a DMA read issued
     * with any dual or quad interface type returns zeros, while writes,
     * erases and register ops keep working.  Mirrors the 0xc22017 part on
     * some Xiongmai hi3518ev200 boards, where every multi-I/O read comes
     * back blank and the kernel cannot find its root filesystem.  See
     * OpenIPC/firmware#646. */
    bool     nor_single_io_only;

    /* Register-mode I/O buffer (shared via memory window) */
    bool     iobuf_valid;         /* true after reg-mode op, cleared on next read */
    uint8_t  iobuf[IOBUF_SIZE];
};

/* ── Helpers ─────────────────────────────────────────────────────────── */

/* Return current flash selection from FMC_CFG register */
static inline int hisi_fmc_current_flash_sel(HisiFmcState *s)
{
    return (s->cfg >> FMC_CFG_FLASH_SEL_SHIFT) & 0x3;
}

/* Level-triggered op/DMA-done interrupt.  The fmc100 driver waits on a
 * completion signalled from the FMC ISR (INT & INT_EN), rather than polling;
 * the HiFMC V100 users leave the IRQ unconnected and poll, so this is a no-op
 * for them. */
static void hisi_fmc_update_irq(HisiFmcState *s)
{
    if (s->irq) {
        qemu_set_irq(s->irq, (s->fmc_int & s->int_en) ? 1 : 0);
    }
}

/*
 * Write-back modified flash data to a backing file so the on-disk image
 * stays in sync with the in-memory array (matches real flash persistence).
 */
static void hisi_fmc_flush(const char *file, const uint8_t *data,
                           uint32_t total, uint32_t offset, uint32_t len)
{
    if (!file || !file[0]) {
        return;
    }
    if (offset >= total) {
        return;
    }
    if (offset + len > total) {
        len = total - offset;
    }
    FILE *f = fopen(file, "r+b");
    if (!f) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fmc: cannot open '%s' for write-back\n", file);
        return;
    }
    fseek(f, offset, SEEK_SET);
    fwrite(&data[offset], 1, len, f);
    fclose(f);
}

/* NOR write-back: NOR always lives in flash-file. */
static void hisi_fmc_flush_to_file(HisiFmcState *s, uint32_t offset,
                                    uint32_t len)
{
    hisi_fmc_flush(s->flash_file, s->flash, s->flash_size, offset, len);
}

/* NAND write-back: backing file is flash-file when the NAND is the only
 * chip (flash_type=NAND), or the separate nand-file on dual NOR+NAND boards. */
static void hisi_fmc_flush_nand(HisiFmcState *s, uint32_t offset, uint32_t len)
{
    const char *file = (s->flash_type == FLASH_TYPE_SPI_NAND)
                       ? s->flash_file : s->nand_file;
    hisi_fmc_flush(file, s->nand_data, s->nand_size, offset, len);
}

/*
 * Check if a flash address is in a block that should be write-protected.
 *
 * Uses a two-layer approach:
 * 1) WPS must be enabled (SR3 bit 2) and block must be in locked state
 * 2) Only blocks that were never individually unlocked by the firmware
 *    are actually protected.  This prevents errant erases/writes from
 *    corrupting read-only SquashFS partitions while allowing the firmware
 *    to write to blocks it intends to use (env, mtd partitions).
 *
 * The ever_unlocked flag persists across Global Lock commands, handling
 * the XM firmware's pattern of: Global Lock → selective Unlock → use.
 */
static bool hisi_fmc_block_is_locked(HisiFmcState *s, uint32_t addr)
{
    if (!(s->sr3 & 0x04))  /* WPS bit in SR3 */
        return false;
    if (!s->block_locked)
        return false;
    uint32_t block = addr / NOR_SECTOR_SIZE;
    if (block >= s->num_blocks)
        return false;
    return s->block_locked[block] && !s->block_ever_unlocked[block];
}

/* Decode NAND page address from ADDRL/ADDRH */
static void hisi_fmc_nand_decode_addr(HisiFmcState *s,
                                       uint32_t *block_out,
                                       uint32_t *page_out,
                                       uint32_t *column_out)
{
    uint32_t block_lo = (s->addrl >> 22) & 0x3FF;
    uint32_t block_hi = (s->addrh & 0xFF) << 10;
    *block_out = block_hi | block_lo;
    *page_out = (s->addrl >> 16) & 0x3F;
    *column_out = s->addrl & 0xFFF;
}

/* ── SPI NOR command execution (register mode) ──────────────────────── */

/* Returns true if the command wrote results to iobuf */
static bool hisi_fmc_exec_nor_reg_op(HisiFmcState *s)
{
    uint8_t spi_cmd = s->cmd & 0xFF;
    uint32_t addr = s->addrl;


    uint32_t len = s->data_num & 0x3FFF;

    if (len > IOBUF_SIZE) {
        len = IOBUF_SIZE;
    }

    switch (spi_cmd) {
    case SPI_CMD_READ_ID:
        if (s->flash_jedec) {
            /* User-specified JEDEC ID (e.g. 0x0B4018 for XTX XT25F128B) */
            s->iobuf[0] = (s->flash_jedec >> 16) & 0xFF;
            s->iobuf[1] = (s->flash_jedec >> 8) & 0xFF;
            s->iobuf[2] = s->flash_jedec & 0xFF;
        } else {
            /* Default: Winbond, size derived from flash */
            s->iobuf[0] = NOR_JEDEC_0;
            s->iobuf[1] = NOR_JEDEC_1;
            s->iobuf[2] = (s->flash_size > NOR_FLASH_SIZE)
                           ? NOR_JEDEC_2_16M : NOR_JEDEC_2_8M;
        }
        if (len > 3) {
            memset(&s->iobuf[3], 0, len - 3);
        }
        break;

    case SPI_CMD_READ_STATUS:
        s->iobuf[0] = s->sr;
        break;

    case SPI_CMD_READ_STATUS2:
        s->iobuf[0] = s->sr2;
        break;

    case SPI_CMD_WRITE_STATUS2:
        if (s->sr & SPI_SR_WEL) {
            s->sr2 = s->iobuf[0];
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    case SPI_CMD_READ_STATUS3:
        s->iobuf[0] = s->sr3;
        break;

    case SPI_CMD_WRITE_STATUS3:
        if (s->sr & SPI_SR_WEL) {
            s->sr3 = s->iobuf[0];
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    case SPI_CMD_READ:
    case SPI_CMD_FAST_READ:
        if (addr + len > s->flash_size) {
            len = (addr < s->flash_size) ? s->flash_size - addr : 0;
        }
        if (len) {
            memcpy(s->iobuf, &s->flash[addr], len);
        }
        break;

    case SPI_CMD_WRITE_ENABLE:
        s->sr |= SPI_SR_WEL;
        break;

    case SPI_CMD_WRITE_DISABLE:
        s->sr &= ~SPI_SR_WEL;
        break;

    case SPI_CMD_WRITE_STATUS1:
        if (s->sr & SPI_SR_WEL) {
            s->sr = (s->iobuf[0] & ~SPI_SR_WEL);
            /* 2-byte write: cmd 0x01 with data_num>=2 writes SR1+SR2 */
            if (len >= 2) {
                s->sr2 = s->iobuf[1];
            }
        }
        break;

    case SPI_CMD_READ_SFDP:
        /* SFDP not implemented — return 0xFF (empty) */
        memset(s->iobuf, 0xFF, len);
        break;

    /* Winbond WPS (Write Protection Scheme) per-block lock commands.
     * All four commands require prior WREN and auto-clear WEL on completion,
     * matching real Winbond W25Q silicon behavior.  The XM vendor driver
     * verifies WEL=0 after unlock to confirm the command executed. */
    case 0x36: { /* Individual Block/Sector Lock */
        if (s->block_locked && addr / NOR_SECTOR_SIZE < s->num_blocks) {
            s->block_locked[addr / NOR_SECTOR_SIZE] = 1;
        }
        s->sr &= ~SPI_SR_WEL;
        return false;
    }
    case 0x39: { /* Individual Block/Sector Unlock */
        uint32_t blk = addr / NOR_SECTOR_SIZE;
        if (s->block_locked && blk < s->num_blocks) {
            s->block_locked[blk] = 0;
            s->block_ever_unlocked[blk] = 1;
        }
        s->sr &= ~SPI_SR_WEL;
        return false;
    }
    case 0x7E: /* Global Block/Sector Lock */
        if (s->block_locked) {
            memset(s->block_locked, 1, s->num_blocks);
        }
        s->sr &= ~SPI_SR_WEL;
        return false;
    case 0x98: /* Global Block/Sector Unlock */
        if (s->block_locked) {
            memset(s->block_locked, 0, s->num_blocks);
            memset(s->block_ever_unlocked, 1, s->num_blocks);
        }
        s->sr &= ~SPI_SR_WEL;
        return false;

    case 0x3D: { /* Read Block/Sector Lock status.
                  * Always report unlocked — the XM vendor driver caches
                  * lock state from these reads, and its unlock verification
                  * fails if we report locked here (even though our unlock
                  * handler correctly updates the lock bitmap).  Actual
                  * write protection is enforced in the erase/program/DMA
                  * write handlers. */
        s->iobuf[0] = 0x00;
        break;
    }

    /*
     * The FMC's command engine auto-issues WREN before every program and
     * erase, so these must not be gated on the SPI Write-Enable-Latch: a
     * guest that relies on the controller's auto-WREN never sets WEL, and
     * within a multi-op transfer WEL is cleared after the first op — which
     * silently dropped subsequent programs/erases (corrupting saveenv, jffs2
     * and flashcp/sysupgrade).  WPS block protection still applies.
     */
    case SPI_CMD_PAGE_PROGRAM:
        if (addr < s->flash_size) {
            if (hisi_fmc_block_is_locked(s, addr)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "hisi-fmc: PROGRAM blocked — addr 0x%x is WPS-locked\n",
                              addr);
            } else {
                uint32_t end = addr + len;
                if (end > s->flash_size) {
                    end = s->flash_size;
                }
                for (uint32_t i = addr; i < end; i++) {
                    /* NOR flash: can only clear bits (AND with data) */
                    s->flash[i] &= s->iobuf[i - addr];
                }
                hisi_fmc_flush_to_file(s, addr, end - addr);
            }
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    case SPI_CMD_SECTOR_ERASE: {
        /* Two callers, two conventions, and FMC_OP.ADDR_EN is what tells
         * them apart.
         *
         * ADDR_EN clear — the SPI-NOR core's default sector erase sends the
         * target address as big-endian data bytes through write_reg (memcpy_toio
         * into the IO buffer); bsp_spi_nor_op_reg never programs FMC_ADDRL.  So
         * the erase address lives in iobuf, NOT in the (stale) addrl register.
         * Reading addrl here made flash_eraseall/flashcp erase whatever block a
         * prior DMA op happened to leave in addrl, silently leaving the real
         * target intact (sysupgrade kernel/rootfs reflash failed).
         *
         * ADDR_EN set — U-Boot's hifmc100 driver has the controller emit the
         * address from FMC_ADDRL and uses FMC_DATA_NUM only as the address
         * *byte count*, so iobuf holds whatever the previous op left there.
         * Decoding iobuf in that case yielded 0xffffff, past the end of the
         * chip, and every erase became a silent no-op: the following page
         * programs then ANDed into un-erased flash.  That is what broke
         * U-Boot's SD-card auto-update (product/hiupdate) — kernel and rootfs
         * both landed as new & old, and the board would not boot.
         */
        uint32_t addr_en = (s->variant == FMC_VARIANT_FMC100)
                           ? FMC100_OP_ADDR_EN : FMC_OP_ADDR_EN;
        uint32_t erase_addr;
        if ((s->op & addr_en) || len < 3) {
            erase_addr = addr;
        } else {
            erase_addr = 0;
            for (uint32_t i = 0; i < len && i < 4; i++) {
                erase_addr = (erase_addr << 8) | s->iobuf[i];
            }
        }
        uint32_t base = erase_addr & ~(NOR_SECTOR_SIZE - 1);
        uint32_t end = base + NOR_SECTOR_SIZE;
        if (end > s->flash_size) {
            end = s->flash_size;
        }
        if (hisi_fmc_block_is_locked(s, base)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-fmc: ERASE blocked — sector 0x%x is WPS-locked\n",
                          base);
        } else if (base < s->flash_size) {
            memset(&s->flash[base], 0xFF, end - base);
            hisi_fmc_flush_to_file(s, base, end - base);
        }
        s->sr &= ~SPI_SR_WEL;
        break;
    }

    case SPI_CMD_CHIP_ERASE:
        memset(s->flash, 0xFF, s->flash_size);
        hisi_fmc_flush_to_file(s, 0, s->flash_size);
        s->sr &= ~SPI_SR_WEL;
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "hisi-fmc: NOR unhandled SPI command 0x%02x\n", spi_cmd);
        return false;
    }
    return true;
}

/* ── SPI NAND command execution (register mode) ─────────────────────── */

static void hisi_fmc_exec_nand_reg_op(HisiFmcState *s)
{
    uint8_t spi_cmd = s->cmd & 0xFF;
    uint32_t len = s->data_num & 0x3FFF;
    uint8_t feat_addr;

    if (len > IOBUF_SIZE) {
        len = IOBUF_SIZE;
    }

    switch (spi_cmd) {
    case SPI_CMD_READ_ID:
        if (s->nand_jedec) {
            /* User-specified NAND ID, e.g. 0xEFAA21 for Winbond W25N01GV */
            s->iobuf[0] = (s->nand_jedec >> 16) & 0xFF;
            s->iobuf[1] = (s->nand_jedec >> 8) & 0xFF;
            s->iobuf[2] = s->nand_jedec & 0xFF;
        } else {
            /* Default: GigaDevice GD5F1GM7 (0xC8 0x91) */
            s->iobuf[0] = NAND_JEDEC_0;
            s->iobuf[1] = NAND_JEDEC_1;
            s->iobuf[2] = NAND_JEDEC_2;
        }
        if (len > 3) {
            memset(&s->iobuf[3], 0, len - 3);
        }
        break;

    case SPI_CMD_RESET:
        s->sr = 0;
        s->nand_feature_protect = 0;
        s->nand_feature_config = 0;
        break;

    case SPI_CMD_WRITE_ENABLE:
        s->sr |= SPI_SR_WEL;
        break;

    case SPI_CMD_GET_FEATURES:
        feat_addr = s->addrl & 0xFF;
        switch (feat_addr) {
        case NAND_FEATURE_PROTECT:
            s->iobuf[0] = s->nand_feature_protect;
            break;
        case NAND_FEATURE_FEATURE:
            s->iobuf[0] = s->nand_feature_config;
            break;
        case NAND_FEATURE_STATUS:
            s->iobuf[0] = s->sr;
            break;
        default:
            s->iobuf[0] = 0;
            break;
        }
        break;

    case SPI_CMD_SET_FEATURE:
        feat_addr = s->addrl & 0xFF;
        switch (feat_addr) {
        case NAND_FEATURE_PROTECT:
            s->nand_feature_protect = s->iobuf[0];
            break;
        case NAND_FEATURE_FEATURE:
            s->nand_feature_config = s->iobuf[0];
            break;
        case NAND_FEATURE_STATUS:
            /* Status register: only WEL is writable by SET_FEATURE in practice */
            break;
        default:
            break;
        }
        break;

    case SPI_CMD_SECTOR_ERASE: { /* 0xD8 = Block Erase for NAND */
        if (s->sr & SPI_SR_WEL) {
            /*
             * Unlike page read/program (column first, so ADDRL = row << 16),
             * nand_base sends only the 3 row-address cycles for an erase and
             * hifmc100/fmc100 write them to ADDRL as-is:
             *   FMC_ADDRL_BLOCK_H_MASK(addr_value[1]) |
             *   FMC_ADDRL_BLOCK_L_MASK(addr_value[0])  == row & ~0x3f
             * Decoding this with hisi_fmc_nand_decode_addr() (>> 22) mapped
             * every erase to block 0, so saveenv wiped U-Boot.
             */
            uint32_t row = s->addrl & 0xFFFFFF;
            uint32_t block = row / NAND_PAGES_PER_BLOCK;
            uint32_t nblocks = s->nand_size / NAND_BLOCK_SIZE;
            if (block < nblocks) {
                uint32_t flash_off = block * NAND_BLOCK_SIZE;
                uint32_t oob_off = block * NAND_PAGES_PER_BLOCK *
                                   s->nand_oob_size;
                memset(&s->nand_data[flash_off], 0xFF, NAND_BLOCK_SIZE);
                memset(&s->nand_oob[oob_off], 0xFF,
                       NAND_PAGES_PER_BLOCK * s->nand_oob_size);
                hisi_fmc_flush_nand(s, flash_off, NAND_BLOCK_SIZE);
            }
            s->sr &= ~SPI_SR_WEL;
        }
        break;
    }

    default:
        qemu_log_mask(LOG_UNIMP,
                      "hisi-fmc: NAND unhandled SPI command 0x%02x\n", spi_cmd);
        break;
    }
}

/* ── Register-mode dispatch ──────────────────────────────────────────── */

/* Returns true if the command wrote results to iobuf.
 *
 * Routing is by the FMC's current flash-select (FMC_CFG) AND which chips are
 * physically present.  On dual NOR+NAND boards both are present and the guest
 * driver flips FMC_CFG before NOR vs NAND ops (hifmc_dev_type_switch); when a
 * chip is absent the bus floats, so we return 0x00 so its probe fails cleanly. */
static bool hisi_fmc_exec_reg_op(HisiFmcState *s)
{
    bool wrote_iobuf = true;
    int sel = hisi_fmc_current_flash_sel(s);
    uint32_t cs = (s->op_cfg & FMC_OP_CFG_FM_CS_MASK) >> FMC_OP_CFG_FM_CS_SHIFT;

    if (sel == FLASH_TYPE_SPI_NAND && s->has_nand && cs == s->nand_cs) {
        hisi_fmc_exec_nand_reg_op(s);
    } else if (sel == FLASH_TYPE_SPI_NOR && s->has_nor && cs == s->nor_cs) {
        wrote_iobuf = hisi_fmc_exec_nor_reg_op(s);
    } else {
        /* Selected chip not present — the SPI bus floats high, so a real
         * controller reads 0xFF (not 0x00).  U-Boot's NOR probe treats an
         * all-0x00 ID as a present-but-unknown chip and retries forever; 0xFF
         * is "no device" and it gives up cleanly. */
        uint32_t len = s->data_num & 0x3FFF;
        if (len > IOBUF_SIZE) {
            len = IOBUF_SIZE;
        }
        memset(s->iobuf, 0xFF, len);
    }
    s->fmc_int |= FMC_INT_OP_DONE;
    return wrote_iobuf;
}

/* ── DMA command execution ───────────────────────────────────────────── */

static void hisi_fmc_exec_dma_nor(HisiFmcState *s)
{
    uint32_t addr = s->addrl;
    uint32_t len = s->dma_len;
    bool is_write = s->op_ctrl & FMC_OP_CTRL_RW_OP;
    uint64_t dma_addr = ((uint64_t)s->dma_saddrh_d0 << 32) | s->dma_saddr;

    /*
     * Real FMC hardware requires OP_MODE_NORMAL (bit 0 = 1) in FMC_CFG
     * for DMA operations. In boot mode (bit 0 = 0), only memory-mapped
     * XIP reads work. If the kernel driver forgets to set this, DMA reads
     * return 0xFF (erased flash).
     */
    if (s->variant != FMC_VARIANT_FMC100 && !(s->cfg & 1)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fmc: DMA op with FMC_CFG=0x%x (not in NORMAL mode),"
                      " returning 0xFF\n", s->cfg);
        if (!is_write && len > 0) {
            uint8_t *erased = g_malloc(len);
            memset(erased, 0xFF, len);
            dma_memory_write(&address_space_memory, dma_addr,
                             erased, len, MEMTXATTRS_UNSPECIFIED);
            g_free(erased);
        }
        s->fmc_int |= FMC_INT_OP_DONE;
        return;
    }

    /*
     * Validate SPI timing is configured. Real hardware needs non-zero
     * chip-select timing for reliable DMA transfers.
     */
    if (s->spi_timing == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fmc: DMA op with FMC_SPI_TIMING_CFG=0 "
                      "(unconfigured timing)\n");
    }

    if (addr + len > s->flash_size) {
        len = (addr < s->flash_size) ? s->flash_size - addr : 0;
    }

    if (len == 0) {
        return;
    }

    if (!is_write && s->nor_single_io_only &&
        (s->op_cfg & FMC_OP_CFG_MEM_IF_MASK)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fmc: multi-I/O DMA read (OP_CFG=0x%x) on a "
                      "single-I/O-only NOR, returning zeros\n", s->op_cfg);
        uint8_t *blank = g_malloc0(len);
        dma_memory_write(&address_space_memory, dma_addr,
                         blank, len, MEMTXATTRS_UNSPECIFIED);
        g_free(blank);
    } else if (!is_write) {
        dma_memory_write(&address_space_memory, dma_addr,
                         &s->flash[addr], len, MEMTXATTRS_UNSPECIFIED);
    } else {
        /*
         * The FMC DMA program engine issues WREN before every page it writes,
         * so a multi-page bulk transfer must not depend on the single SPI
         * Write-Enable-Latch a guest sets once: WEL is cleared after the first
         * page program, so gating each DMA chunk on it silently drops every
         * page after the first.  That corrupted U-Boot saveenv (blank env ->
         * no soc -> sysupgrade aborts), jffs2 overlay writes ("Node totlen
         * 0xffffffff") and flashcp.  Always program; WPS protection applies.
         */
        if (hisi_fmc_block_is_locked(s, addr)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-fmc: DMA WRITE blocked — addr 0x%x is WPS-locked\n",
                          addr);
        } else {
            uint8_t *buf = g_malloc(len);
            dma_memory_read(&address_space_memory, dma_addr,
                            buf, len, MEMTXATTRS_UNSPECIFIED);
            for (uint32_t i = 0; i < len; i++) {
                s->flash[addr + i] &= buf[i];
            }
            g_free(buf);
            hisi_fmc_flush_to_file(s, addr, len);
        }
        s->sr &= ~SPI_SR_WEL;
    }
}

static void hisi_fmc_exec_dma_nand(HisiFmcState *s)
{
    uint32_t block, page, column;
    bool is_write = s->op_ctrl & FMC_OP_CTRL_RW_OP;
    uint64_t dma_addr = ((uint64_t)s->dma_saddrh_d0 << 32) | s->dma_saddr;
    uint64_t dma_oob_addr = ((uint64_t)s->dma_saddrh_oob << 32)
                            | s->dma_saddr_oob;

    hisi_fmc_nand_decode_addr(s, &block, &page, &column);

    uint32_t nblocks = s->nand_size / NAND_BLOCK_SIZE;
    if (block >= nblocks || page >= NAND_PAGES_PER_BLOCK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fmc: NAND DMA out of range block=%u page=%u\n",
                      block, page);
        return;
    }

    uint32_t page_idx = block * NAND_PAGES_PER_BLOCK + page;
    uint32_t flash_off = page_idx * NAND_PAGE_SIZE;
    uint32_t oob_off = page_idx * s->nand_oob_size;

    /*
     * The hifmc100 driver only programs FMC_DMA_LEN for OOB-only / ecc0
     * transfers (drivers/mtd/nand/hifmc100/hifmc100.c).  A normal page DMA
     * always moves a full page + spare, selected by FMC_OP_CTRL's RD_OP_SEL
     * field (READ_ALL_PAGE), NOT by FMC_DMA_LEN — which keeps a stale value
     * (e.g. the last OOB size) from a prior op.  Sizing the data copy by
     * s->dma_len therefore truncated every page read (corrupting the U-Boot
     * env CRC and all UBI reads).  Always transfer a whole page + OOB.
     */
    if (!is_write) {
        /* DMA read: flash → guest memory (full page data + spare) */
        dma_memory_write(&address_space_memory, dma_addr,
                         &s->nand_data[flash_off], NAND_PAGE_SIZE,
                         MEMTXATTRS_UNSPECIFIED);
        if (s->dma_saddr_oob) {
            dma_memory_write(&address_space_memory, dma_oob_addr,
                             &s->nand_oob[oob_off], s->nand_oob_size,
                             MEMTXATTRS_UNSPECIFIED);
        }
    } else {
        /* DMA write: guest memory → flash (full page program) */
        if (s->sr & SPI_SR_WEL) {
            uint8_t *buf = g_malloc(NAND_PAGE_SIZE);
            dma_memory_read(&address_space_memory, dma_addr,
                            buf, NAND_PAGE_SIZE, MEMTXATTRS_UNSPECIFIED);
            /* NAND program: can only clear bits */
            for (uint32_t i = 0; i < NAND_PAGE_SIZE; i++) {
                s->nand_data[flash_off + i] &= buf[i];
            }
            g_free(buf);
            /* OOB write */
            if (s->dma_saddr_oob) {
                uint8_t *oob_buf = g_malloc(s->nand_oob_size);
                dma_memory_read(&address_space_memory, dma_oob_addr,
                                oob_buf, s->nand_oob_size,
                                MEMTXATTRS_UNSPECIFIED);
                for (uint32_t i = 0; i < s->nand_oob_size; i++) {
                    s->nand_oob[oob_off + i] &= oob_buf[i];
                }
                g_free(oob_buf);
            }
            /* Persist the page so saveenv / UBI writes survive (matches the
             * NOR write-back behaviour; OOB lives only in RAM). */
            hisi_fmc_flush_nand(s, flash_off, NAND_PAGE_SIZE);
            s->sr &= ~SPI_SR_WEL;
        }
    }
}

static void hisi_fmc_exec_dma_op(HisiFmcState *s)
{
    int sel = hisi_fmc_current_flash_sel(s);

    if (sel == FLASH_TYPE_SPI_NAND && s->has_nand) {
        hisi_fmc_exec_dma_nand(s);
    } else if (sel == FLASH_TYPE_SPI_NOR && s->has_nor) {
        hisi_fmc_exec_dma_nor(s);
    }
    /* else: selected chip not present, no-op */

    s->fmc_int |= FMC_INT_OP_DONE;
}

/* ── Control register access ─────────────────────────────────────────── */

static uint64_t hisi_fmc_ctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiFmcState *s = HISI_FMC(opaque);

    if (s->variant == FMC_VARIANT_FMC100) {
        hwaddr c = fmc100_to_canonical(offset);
        if (c == (hwaddr)-1) {
            return 0;
        }
        offset = c;
    }

    switch (offset) {
    case FMC_VERSION:
        return s->variant == FMC_VARIANT_FMC100 ? 0x200 : 0x100;
    case FMC_CFG:           return s->cfg;
    case FMC_GLOBAL_CFG:    return s->global_cfg;
    case FMC_SPI_TIMING_CFG: return s->spi_timing;
    case FMC_INT:           return s->fmc_int;
    case FMC_INT_EN:        return s->int_en;
    case FMC_CMD:           return s->cmd;
    case FMC_ADDRH:         return s->addrh;
    case FMC_ADDRL:         return s->addrl;
    case FMC_OP_CFG:        return s->op_cfg;
    case FMC_SPI_OP_ADDR:   return s->spi_op_addr;
    case FMC_DATA_NUM:      return s->data_num;
    case FMC_OP:            return 0; /* REG_OP_START always reads as clear */
    case FMC_DMA_LEN:       return s->dma_len;
    case FMC_DMA_AHB_CTRL:  return s->dma_ahb_ctrl;
    case FMC_DMA_SADDR_D0:  return s->dma_saddr;
    case FMC_DMA_SADDR_OOB: return s->dma_saddr_oob;
    case FMC_OP_CTRL:       return 0; /* DMA_OP_READY always reads as clear */
    case FMC_STATUS:        return s->status;
    case FMC_DMA_SADDRH_D0: return s->dma_saddrh_d0;
    case FMC_DMA_SADDRH_OOB: return s->dma_saddrh_oob;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fmc: read from unknown reg 0x%03x\n",
                      (unsigned)offset);
        return 0;
    }
}

static void hisi_fmc_ctrl_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    HisiFmcState *s = HISI_FMC(opaque);

    if (s->variant == FMC_VARIANT_FMC100) {
        hwaddr c = fmc100_to_canonical(offset);
        if (c == (hwaddr)-1) {
            return;  /* unmapped fmc100 reg (OP_PARA, timing…): ignore */
        }
        offset = c;
    }

    switch (offset) {
    case FMC_CFG:           s->cfg = value;           break;
    case FMC_GLOBAL_CFG:    s->global_cfg = value;    break;
    case FMC_SPI_TIMING_CFG: s->spi_timing = value;   break;
    case FMC_INT_EN:        s->int_en = value;   hisi_fmc_update_irq(s); break;
    case FMC_INT_CLR:       s->fmc_int &= ~value; hisi_fmc_update_irq(s); break;
    case FMC_CMD:           s->cmd = value;           break;
    case FMC_ADDRH:         s->addrh = value;         break;
    case FMC_ADDRL:         s->addrl = value;         break;
    case FMC_OP_CFG:        s->op_cfg = value;        break;
    case FMC_SPI_OP_ADDR:   s->spi_op_addr = value;   break;
    case FMC_DATA_NUM:      s->data_num = value;      break;
    case FMC_DMA_LEN:       s->dma_len = value & 0x0FFFFFFF; break;
    case FMC_DMA_AHB_CTRL:  s->dma_ahb_ctrl = value;  break;
    case FMC_DMA_SADDR_D0:  s->dma_saddr = value;     break;
    case FMC_DMA_SADDR_OOB: s->dma_saddr_oob = value; break;
    case FMC_DMA_SADDRH_D0: s->dma_saddrh_d0 = value; break;
    case FMC_DMA_SADDRH_OOB: s->dma_saddrh_oob = value; break;

    case FMC_OP:
        s->iobuf_valid = false;
        s->op = value;
        if (value & FMC_OP_REG_OP_START) {
            /* On fmc100 bit1 is ADDR_EN, not READ_STATUS_EN — always dispatch
             * by SPI command (READ_STATUS 0x05 lands the SR in iobuf, which the
             * driver reads back from the memory window like every other op). */
            if (s->variant != FMC_VARIANT_FMC100 &&
                (value & FMC_OP_READ_STATUS_EN)) {
                /* The controller issues RDSR (05h) on its own and latches SR1
                 * into FMC_STATUS; FMC_CMD is not consulted.  The vendor
                 * drivers rely on that and leave FMC_CMD holding the previous
                 * op's opcode -- U-Boot 2022.10's QE setup reads SR2 with 35h
                 * just before.  Choosing the register by that stale opcode
                 * returned SR2's QE bit as WEL, write_enable skipped WREN, the
                 * WRSR was dropped and every boot printed "Disable Quad
                 * failed! reg: 0x2".  Measured on a gk7205v200: with WEL set
                 * (SR1 0x02) and SR3 0x08, FMC_STATUS reads 0x02 with FMC_CMD
                 * = 15h as with 05h. */
                s->status = s->sr;
                s->iobuf_valid = true;
            } else {
                /* Only mark iobuf valid if the command actually wrote to it.
                 * Unknown commands (block lock, etc.) must NOT set iobuf_valid
                 * or subsequent memory-window reads return stale iobuf data
                 * instead of actual flash content. */
                s->iobuf_valid = hisi_fmc_exec_reg_op(s);
            }
            s->fmc_int |= FMC_INT_OP_DONE;
            hisi_fmc_update_irq(s);
        }
        break;

    case FMC_OP_CTRL:
        s->op_ctrl = value;
        s->iobuf_valid = false;
        if (value & FMC_OP_CTRL_DMA_OP_READY) {
            hisi_fmc_exec_dma_op(s);
            hisi_fmc_update_irq(s);
        }
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fmc: write to unknown reg 0x%03x = 0x%08x\n",
                      (unsigned)offset, (unsigned)value);
        break;
    }
}

static const MemoryRegionOps hisi_fmc_ctrl_ops = {
    .read  = hisi_fmc_ctrl_read,
    .write = hisi_fmc_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ── Memory window access (register-mode I/O buffer) ─────────────────── */

static uint64_t hisi_fmc_mem_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiFmcState *s = HISI_FMC(opaque);

    /* After a register-mode SPI operation, the result is in iobuf.
     * U-Boot/kernel reads the result from low offsets of the memory window.
     * The flag stays set until the next FMC_OP write clears it. */
    if (s->iobuf_valid && offset < IOBUF_SIZE) {
        if (size == 4 && (offset & 3) == 0) {
            return ldl_le_p(&s->iobuf[offset]);
        }
        return s->iobuf[offset];
    }

    /* Direct flash read — memory-mapped XIP access used by the boot ROM and
     * U-Boot to read flash from the FMC memory window.  The boot/XIP chip is
     * NOR when present (dual NOR+NAND or NOR-only boards); on NAND-only boards
     * the boot ROM XIPs the NAND data area linearly (data-only dump). */
    const uint8_t *xip = s->has_nor ? s->flash : s->nand_data;
    uint32_t xip_size = s->has_nor ? s->flash_size : s->nand_size;
    if (xip && offset < xip_size) {
        if (size == 4 && (offset & 3) == 0) {
            return ldl_le_p(&xip[offset]);
        }
        if (size == 2 && (offset & 1) == 0) {
            return lduw_le_p(&xip[offset]);
        }
        return xip[offset];
    }

    return 0;
}

static void hisi_fmc_mem_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    HisiFmcState *s = HISI_FMC(opaque);

    if (offset < IOBUF_SIZE) {
        if (size == 4 && (offset & 3) == 0) {
            stl_le_p(&s->iobuf[offset], value);
        } else {
            s->iobuf[offset] = value & 0xFF;
        }
    }
}

static const MemoryRegionOps hisi_fmc_mem_ops = {
    .read  = hisi_fmc_mem_read,
    .write = hisi_fmc_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* Accept up to 64-bit accesses: the aarch64 (Hi3519DV500) fmc100 driver
     * reads the register-mode result (e.g. the JEDEC ID memcpy'd from the
     * FMC buffer window) with a single 8-byte load.  valid.max_access_size=4
     * rejected that, so the read returned 0xFF without ever reaching .read and
     * the NOR probe failed.  impl.max_access_size=4 makes the memory core split
     * a 64-bit access into the 4-byte handler above (which serves iobuf/XIP). */
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* ── Device lifecycle ────────────────────────────────────────────────── */

/* Load a backing file into a buffer of at least min_size, growing to fit a
 * larger image.  Returns the allocated buffer (0xFF-filled past the file) and
 * its size via *out_size, or sets errp and returns NULL. */
static uint8_t *hisi_fmc_load_image(const char *file, uint32_t min_size,
                                    uint32_t *out_size, Error **errp)
{
    struct stat st;
    uint32_t size = min_size;
    if (file && file[0]) {
        if (stat(file, &st) != 0) {
            error_setg(errp, "hisi-fmc: cannot stat flash file '%s'", file);
            return NULL;
        }
        if ((uint32_t)st.st_size > size) {
            size = (uint32_t)st.st_size;
        }
    }
    uint8_t *buf = g_malloc(size);
    memset(buf, 0xFF, size);
    if (file && file[0]) {
        FILE *f = fopen(file, "rb");
        if (!f) {
            error_setg(errp, "hisi-fmc: cannot open flash file '%s'", file);
            g_free(buf);
            return NULL;
        }
        size_t nread = fread(buf, 1, st.st_size, f);
        fclose(f);
        if (nread != (size_t)st.st_size) {
            error_setg(errp, "hisi-fmc: short read from '%s'", file);
            g_free(buf);
            return NULL;
        }
        qemu_log("hisi-fmc: loaded %u bytes from '%s'\n",
                 (uint32_t)st.st_size, file);
    }
    *out_size = size;
    return buf;
}

static void hisi_fmc_realize(DeviceState *dev, Error **errp)
{
    HisiFmcState *s = HISI_FMC(dev);

    /*
     * Chip topology:
     *   flash_type == NAND  → flash-file is the SPI-NAND image (NAND-only).
     *   flash_type == NOR   → flash-file is the SPI-NOR image (NOR present).
     *   nand-file given     → an additional SPI-NAND chip (dual NOR+NAND
     *                         board: NOR holds u-boot+env, NAND holds rootfs).
     */
    const char *nor_file  = (s->flash_type == FLASH_TYPE_SPI_NOR)
                            ? s->flash_file : NULL;
    const char *nand_file = (s->flash_type == FLASH_TYPE_SPI_NAND)
                            ? s->flash_file : s->nand_file;

    /* ── SPI NOR ── */
    if (s->flash_type == FLASH_TYPE_SPI_NOR) {
        s->has_nor = true;
        /* Many NOR chips ship with QE set in SR2; XM firmware expects QE=1. */
        s->sr2 = 0x02;
        s->flash = hisi_fmc_load_image(nor_file, NOR_FLASH_SIZE,
                                       &s->flash_size, errp);
        if (!s->flash) {
            return;
        }
        /* WPS block-lock arrays — all blocks start unlocked. */
        s->num_blocks = s->flash_size / NOR_SECTOR_SIZE;
        s->block_locked = g_malloc0(s->num_blocks);
        s->block_ever_unlocked = g_malloc0(s->num_blocks);
    }

    /* ── SPI NAND ── */
    if (nand_file || s->flash_type == FLASH_TYPE_SPI_NAND) {
        s->has_nand = true;
        /* NAND sits on its own chip-select.  Alone it is CS0; alongside a NOR
         * boot flash (CS0) it is CS1, matching dual NOR+NAND board wiring. */
        s->nand_cs = s->has_nor ? 1 : 0;
        s->nand_data = hisi_fmc_load_image(nand_file, NAND_FLASH_SIZE,
                                           &s->nand_size, errp);
        if (!s->nand_data) {
            return;
        }
        uint32_t npages = s->nand_size / NAND_PAGE_SIZE;
        s->nand_oob = g_malloc(npages * s->nand_oob_size);
        memset(s->nand_oob, 0xFF, npages * s->nand_oob_size);
        /* On NAND-only boards the boot ROM XIPs NAND and the guest starts in
         * NAND mode; on dual boards FMC_CFG stays NOR for the NOR-resident
         * u-boot+env and the driver flips it before NAND ops. */
        if (!s->has_nor) {
            s->cfg = (s->cfg & ~FMC_CFG_FLASH_SEL_MASK) |
                     (FLASH_TYPE_SPI_NAND << FMC_CFG_FLASH_SEL_SHIFT);
        }
    }

    if (s->has_nor) {
        /* Scan for SquashFS partitions and pre-mark non-SquashFS blocks as
         * ever_unlocked.  This protects SquashFS data from errant erases
         * while allowing the firmware to freely erase/write other blocks
         * (env, mtd, boot) without WPS interference.
         *
         * The scan finds SquashFS superblocks (magic 'hsqs') at sector
         * boundaries and marks blocks within each filesystem's extent as
         * protected (ever_unlocked=0).  All other blocks get ever_unlocked=1.
         */
        if (s->block_ever_unlocked && s->flash_type == FLASH_TYPE_SPI_NOR) {
            /* Start by marking ALL blocks as ever_unlocked (writable) */
            memset(s->block_ever_unlocked, 1, s->num_blocks);

            /* Then clear ever_unlocked for blocks inside SquashFS partitions */
            for (uint32_t off = 0; off + 4 <= s->flash_size;
                 off += NOR_SECTOR_SIZE) {
                if (s->flash[off]     == 0x68 && /* 'h' */
                    s->flash[off + 1] == 0x73 && /* 's' */
                    s->flash[off + 2] == 0x71 && /* 'q' */
                    s->flash[off + 3] == 0x73) { /* 's' */
                    /* Found SquashFS superblock — read bytes_used (LE64 at +40) */
                    uint64_t bytes_used = 0;
                    if (off + 48 <= s->flash_size) {
                        for (int i = 0; i < 8; i++) {
                            bytes_used |= (uint64_t)s->flash[off + 40 + i] << (8 * i);
                        }
                    }
                    if (bytes_used == 0 || bytes_used > s->flash_size) {
                        bytes_used = NOR_SECTOR_SIZE; /* fallback: protect one block */
                    }
                    uint32_t sqfs_end = off + (uint32_t)bytes_used;
                    if (sqfs_end > s->flash_size) sqfs_end = s->flash_size;
                    uint32_t blk_start = off / NOR_SECTOR_SIZE;
                    uint32_t blk_end = (sqfs_end + NOR_SECTOR_SIZE - 1)
                                       / NOR_SECTOR_SIZE;
                    if (blk_end > s->num_blocks) blk_end = s->num_blocks;
                    for (uint32_t b = blk_start; b < blk_end; b++) {
                        s->block_ever_unlocked[b] = 0;
                    }
                    qemu_log("hisi-fmc: protect SquashFS at 0x%x "
                             "(%" PRIu64 " bytes, blocks %u-%u)\n",
                             off, bytes_used, blk_start, blk_end - 1);
                }
            }
        }
    }

    /* Factory-locked NOR override (openhisilicon#83).
     *
     * Applied AFTER the SquashFS scan so it overrides whatever the scan
     * computed.  Real Xiongmai-flashed Winbond W25Q128s come out of the
     * factory with SR3.WPS = 1 and every block individually locked; the
     * runtime kernel/U-Boot/agent has to issue Global Block Unlock
     * (0x98) + clear SR3.WPS to make the chip writable.  Without this
     * knob, qemu-side flash starts already-unlocked because vendor
     * firmware (when actually run) would have done that work — but a
     * recovery boot ROM / fresh-U-Boot / bare-metal agent doesn't see
     * the locked state at all, so its global-unlock code path goes
     * untested.
     *
     * Default off preserves current behaviour. */
    if (s->nor_wps_locked && s->flash_type == FLASH_TYPE_SPI_NOR &&
        s->block_locked && s->block_ever_unlocked) {
        s->sr3 |= 0x04;                         /* WPS = 1 */
        memset(s->block_locked, 1, s->num_blocks);
        memset(s->block_ever_unlocked, 0, s->num_blocks);
        qemu_log("hisi-fmc: factory-locked NOR (SR3.WPS=1, all "
                 "%u blocks locked); firmware must Global-Unlock 0x98 "
                 "+ clear SR3.WPS before erase/program will succeed\n",
                 s->num_blocks);
    }
}

static void hisi_fmc_init(Object *obj)
{
    HisiFmcState *s = HISI_FMC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->ctrl_iomem, obj, &hisi_fmc_ctrl_ops, s,
                          "hisi-fmc.ctrl", CTRL_REG_SIZE);
    sysbus_init_mmio(sbd, &s->ctrl_iomem);

    memory_region_init_io(&s->mem_iomem, obj, &hisi_fmc_mem_ops, s,
                          "hisi-fmc.mem", MEM_WINDOW_SIZE);
    sysbus_init_mmio(sbd, &s->mem_iomem);

    sysbus_init_irq(sbd, &s->irq);
}

static void hisi_fmc_finalize(Object *obj)
{
    HisiFmcState *s = HISI_FMC(obj);
    g_free(s->flash);
    g_free(s->nand_data);
    g_free(s->nand_oob);
    g_free(s->block_locked);
    g_free(s->block_ever_unlocked);
}

/*
 * SC_SYSRES resets the controller, which is part of the SoC: its registers
 * return to their power-on values and a register-mode result still parked
 * in the I/O buffer is dropped, so the boot ROM's memory-window reads see
 * flash again rather than the last operation's data.
 *
 * The SPI flash chip is a separate part that the SoC reset does not reach,
 * so its state is deliberately kept: SR/SR2/SR3 (QE and WPS are
 * non-volatile), the WPS block locks and the SPI-NAND feature registers.
 * On a camera these survive a warm reboot as well.
 */
static void hisi_fmc_reset(DeviceState *dev)
{
    HisiFmcState *s = HISI_FMC(dev);

    s->cfg = 0;
    if (s->has_nand && !s->has_nor) {
        /* NAND-only board: the boot ROM XIPs NAND, as set up in realize. */
        s->cfg = FLASH_TYPE_SPI_NAND << FMC_CFG_FLASH_SEL_SHIFT;
    }
    s->global_cfg = 0;
    s->spi_timing = 0;
    s->fmc_int = 0;
    s->int_en = 0;
    s->cmd = 0;
    s->addrh = 0;
    s->addrl = 0;
    s->op = 0;
    s->op_cfg = 0;
    s->spi_op_addr = 0;
    s->data_num = 0;
    s->dma_len = 0;
    s->dma_saddr = 0;
    s->dma_ahb_ctrl = 0;
    s->dma_saddr_oob = 0;
    s->dma_saddrh_d0 = 0;
    s->dma_saddrh_oob = 0;
    s->op_ctrl = 0;
    s->status = 0;

    s->iobuf_valid = false;
    memset(s->iobuf, 0, sizeof(s->iobuf));

    hisi_fmc_update_irq(s);
}

static const Property hisi_fmc_properties[] = {
    /* Register-map variant: 0 = HiFMC V100 (default), 1 = fmc100 (lotus,fmc
     * on Goke xmorca / GK7206).  Same IP, shuffled register offsets. */
    DEFINE_PROP_UINT32("variant", HisiFmcState, variant, FMC_VARIANT_HIFMC),
    DEFINE_PROP_UINT32("flash-type", HisiFmcState, flash_type,
                       FLASH_TYPE_SPI_NOR),
    DEFINE_PROP_STRING("flash-file", HisiFmcState, flash_file),
    /* Second image for dual NOR+NAND boards: flash-file is the SPI-NOR
     * (u-boot + env), nand-file is the SPI-NAND (rootfs UBI). */
    DEFINE_PROP_STRING("nand-file", HisiFmcState, nand_file),
    DEFINE_PROP_UINT32("flash-jedec", HisiFmcState, flash_jedec, 0),
    /* NAND READ_ID override (default GigaDevice GD5F1GM7 0xC8 0x91); set
     * 0xEFAA21 to present Winbond W25N01GV for firmware whose SPI-NAND ID
     * table lacks GD5F1GM7. */
    DEFINE_PROP_UINT32("nand-jedec", HisiFmcState, nand_jedec, 0),
    /* NAND spare bytes per page: 128 (GD5F1GM7), 64 (W25N01GV). */
    DEFINE_PROP_UINT32("nand-oob-size", HisiFmcState, nand_oob_size, 128),
    /* Start the chip in the as-shipped factory-locked state for SPI NOR.
     * When on: SR3.WPS = 1, every block individually locked, no block
     * ever-unlocked.  Lets CI exercise the recovery-unlock path
     * (Winbond Global Block Unlock 0x98 + clear SR3.WPS) that runtime
     * vendor firmware would have already executed.  Default off
     * preserves the existing already-unlocked behaviour.  See
     * openhisilicon#83. */
    DEFINE_PROP_BOOL("nor-wps-locked", HisiFmcState, nor_wps_locked, false),
    /* Answer only single-I/O reads, like the NOR on the board in
     * OpenIPC/firmware#646.  Default off. */
    DEFINE_PROP_BOOL("nor-single-io-only", HisiFmcState, nor_single_io_only,
                     false),
};

static void hisi_fmc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = hisi_fmc_realize;
    device_class_set_legacy_reset(dc, hisi_fmc_reset);
    device_class_set_props(dc, hisi_fmc_properties);
}

static const TypeInfo hisi_fmc_info = {
    .name          = TYPE_HISI_FMC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiFmcState),
    .instance_init = hisi_fmc_init,
    .instance_finalize = hisi_fmc_finalize,
    .class_init    = hisi_fmc_class_init,
};

static void hisi_fmc_register_types(void)
{
    type_register_static(&hisi_fmc_info);
}

type_init(hisi_fmc_register_types)
