/*
 * HiSilicon HISFC350 SPI Flash Controller emulation.
 *
 * Found on Hi3516CV100 / Hi3518EV100 (V1 generation) SoCs.
 * RAM-backed SPI NOR flash (Winbond W25Q64, 8 MiB) with three
 * access modes: register-mode (command + 64-byte data buffer),
 * DMA-mode (bulk transfers), and bus-mode (memory-mapped reads).
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/host-utils.h"
#include "system/dma.h"
#include <sys/stat.h>

/* ── Register offsets ────────────────────────────────────────────────── */

#define SFC_GLOBAL_CONFIG       0x0100
#define SFC_TIMING              0x0110
#define SFC_INT_RAW_STATUS      0x0120
#define SFC_INT_STATUS          0x0124
#define SFC_INT_MASK            0x0128
#define SFC_INT_CLEAR           0x012C
#define SFC_VERSION             0x01F8

#define SFC_BUS_CONFIG1         0x0200
#define SFC_BUS_CONFIG2         0x0204
#define SFC_BUS_FLASH_SIZE      0x0210
#define SFC_BUS_BASE_ADDR_CS0   0x0214
#define SFC_BUS_BASE_ADDR_CS1   0x0218

#define SFC_BUS_DMA_CTRL        0x0240
#define SFC_BUS_DMA_MEM_SADDR   0x0244
#define SFC_BUS_DMA_FLASH_SADDR 0x0248
#define SFC_BUS_DMA_LEN         0x024C
#define SFC_BUS_DMA_AHB_CTRL    0x0250

#define SFC_CMD_CONFIG          0x0300
#define SFC_CMD_INS             0x0308
#define SFC_CMD_ADDR            0x030C
#define SFC_CMD_DATABUF0        0x0400

/* CMD_CONFIG bits */
#define CMD_CONFIG_START        BIT(0)
#define CMD_CONFIG_SEL_CS_SHIFT 1
#define CMD_CONFIG_ADDR_EN      BIT(3)
#define CMD_CONFIG_DATA_EN      BIT(7)
#define CMD_CONFIG_RW_READ      BIT(8)
#define CMD_CONFIG_DATA_CNT_SHIFT 9
#define CMD_CONFIG_DATA_CNT_MASK  0x3F

/* BUS_DMA_CTRL bits */
#define DMA_CTRL_START          BIT(0)
#define DMA_CTRL_RW_READ        BIT(1)
#define DMA_CTRL_CS_SHIFT       4

/* INT_RAW_STATUS bits */
#define INT_DMA_DONE            BIT(1)

/* SPI commands */
#define SPI_CMD_WREN            0x06
#define SPI_CMD_WRDI            0x04
#define SPI_CMD_RDSR            0x05
#define SPI_CMD_RDSR2           0x35
#define SPI_CMD_WRSR            0x01
#define SPI_CMD_WRSR2           0x31
#define SPI_CMD_RDID            0x9F
#define SPI_CMD_READ            0x03
#define SPI_CMD_FAST_READ       0x0B
#define SPI_CMD_PP              0x02
#define SPI_CMD_SE_4K           0x20
#define SPI_CMD_SE_32K          0x52
#define SPI_CMD_SE_64K          0xD8
#define SPI_CMD_CE              0xC7
#define SPI_CMD_FAST_READ_DUAL  0x3B
#define SPI_CMD_FAST_READ_QUAD  0x6B
#define SPI_CMD_EN4B            0xB7
#define SPI_CMD_EX4B            0xE9

/* SPI status register bits */
#define SPI_SR_WEL              BIT(1)
#define SPI_SR2_QE              BIT(1)

/* Flash: Winbond W25Q64 (8 MiB) unless a larger flash-file picks the
 * W25Q128/256 of its size (capacity byte = log2 of the size in bytes). */
#define NOR_JEDEC_MFR           0xEF
#define NOR_JEDEC_TYPE          0x40
#define NOR_FLASH_SIZE          (8 * 1024 * 1024)
#define NOR_SECTOR_SIZE         (64 * 1024)
#define NOR_PAGE_SIZE           256

/* Controller identity */
#define SFC350_VERSION_ID       0x350

#define CTRL_REG_SIZE           0x500
/* Window sized for the largest SPI NOR we boot in QEMU (MX25L25635E = 32 MiB
 * on hi3520dv200, openipc/firmware#2089).  mem_read returns 0xFF for offsets
 * past s->flash_size so smaller flash images stay correct. */
#define MEM_WINDOW_SIZE         (32 * 1024 * 1024)
#define DATABUF_REGS            16
#define DATABUF_BYTES           (DATABUF_REGS * 4)

/* ── Device state ────────────────────────────────────────────────────── */

#define TYPE_HISI_SFC350 "hisi-sfc350"
OBJECT_DECLARE_SIMPLE_TYPE(HisiSfc350State, HISI_SFC350)

struct HisiSfc350State {
    SysBusDevice parent_obj;
    MemoryRegion ctrl_iomem;
    MemoryRegion mem_iomem;

    /* Control registers */
    uint32_t global_config;
    uint32_t timing;
    uint32_t int_raw_status;
    uint32_t int_mask;
    uint32_t bus_config1;
    uint32_t bus_config2;
    uint32_t bus_flash_size;
    uint32_t bus_base_addr_cs0;
    uint32_t bus_base_addr_cs1;
    uint32_t bus_dma_ctrl;
    uint32_t bus_dma_mem_saddr;
    uint32_t bus_dma_flash_saddr;
    uint32_t bus_dma_len;
    uint32_t bus_dma_ahb_ctrl;
    uint32_t cmd_config;
    uint32_t cmd_ins;
    uint32_t cmd_addr;
    uint32_t cmd_databuf[DATABUF_REGS];

    /* Configuration properties */
    char    *flash_file;

    /* SPI flash state */
    uint8_t  sr;
    uint8_t  sr2;
    uint8_t  jedec_cap;
    uint8_t *flash;
    uint32_t flash_size;
};

/* Write-back modified flash data to the backing file. */
static void hisi_sfc350_flush_to_file(HisiSfc350State *s, uint32_t offset,
                                       uint32_t len)
{
    if (!s->flash_file || !s->flash_file[0]) {
        return;
    }
    if (offset + len > s->flash_size) {
        len = s->flash_size - offset;
    }
    FILE *f = fopen(s->flash_file, "r+b");
    if (!f) {
        return;
    }
    fseek(f, offset, SEEK_SET);
    fwrite(&s->flash[offset], 1, len, f);
    fclose(f);
}

/* ── Register-mode command execution ─────────────────────────────────── */

static void hisi_sfc350_exec_cmd(HisiSfc350State *s)
{
    uint8_t opcode = s->cmd_ins & 0xFF;
    uint32_t addr = s->cmd_addr;
    bool data_en = s->cmd_config & CMD_CONFIG_DATA_EN;
    bool rw_read = s->cmd_config & CMD_CONFIG_RW_READ;
    uint32_t data_cnt = ((s->cmd_config >> CMD_CONFIG_DATA_CNT_SHIFT)
                         & CMD_CONFIG_DATA_CNT_MASK) + 1;

    if (data_cnt > DATABUF_BYTES) {
        data_cnt = DATABUF_BYTES;
    }

    switch (opcode) {
    case SPI_CMD_RDID: {
        /* Only CS0 has a flash chip; CS1 returns 0 (no chip) */
        uint32_t cs = (s->cmd_config >> CMD_CONFIG_SEL_CS_SHIFT) & 1;
        if (cs == 0) {
            s->cmd_databuf[0] = NOR_JEDEC_MFR | (NOR_JEDEC_TYPE << 8) |
                                (s->jedec_cap << 16);
        } else {
            s->cmd_databuf[0] = 0;
        }
        s->cmd_databuf[1] = 0;
        break;
    }

    /* A status read longer than one byte gets the register again and again,
     * as from the chip; U-Boot's "Enable QE" reads SR2 two bytes long. */
    case SPI_CMD_RDSR:
    case SPI_CMD_RDSR2: {
        uint32_t rep = (opcode == SPI_CMD_RDSR ? s->sr : s->sr2) * 0x01010101u;
        for (uint32_t i = 0; i < DIV_ROUND_UP(data_cnt, 4); i++) {
            s->cmd_databuf[i] = rep;
        }
        break;
    }

    case SPI_CMD_WREN:
        s->sr |= SPI_SR_WEL;
        break;

    case SPI_CMD_WRDI:
        s->sr &= ~SPI_SR_WEL;
        break;

    case SPI_CMD_WRSR:
        /* SR1 is not kept (no protection modelled); a second data byte is
         * SR2, whose QE bit U-Boot sets and reads back ("Enable QE"). */
        if ((s->sr & SPI_SR_WEL) && data_en && !rw_read && data_cnt >= 2) {
            s->sr2 = (s->cmd_databuf[0] >> 8) & SPI_SR2_QE;
        }
        s->sr &= ~SPI_SR_WEL;
        break;

    case SPI_CMD_WRSR2:
        if ((s->sr & SPI_SR_WEL) && data_en && !rw_read) {
            s->sr2 = s->cmd_databuf[0] & SPI_SR2_QE;
        }
        s->sr &= ~SPI_SR_WEL;
        break;

    case SPI_CMD_READ:
    case SPI_CMD_FAST_READ:
    case SPI_CMD_FAST_READ_DUAL:
    case SPI_CMD_FAST_READ_QUAD:
        if (data_en && rw_read && addr < s->flash_size) {
            uint32_t len = data_cnt;
            if (addr + len > s->flash_size) {
                len = s->flash_size - addr;
            }
            memcpy(s->cmd_databuf, &s->flash[addr], len);
        }
        break;

    case SPI_CMD_PP:
        if (data_en && (s->sr & SPI_SR_WEL) && addr < s->flash_size) {
            uint32_t len = data_cnt;
            if (addr + len > s->flash_size) {
                len = s->flash_size - addr;
            }
            /* NOR page program: can only clear bits (AND with data) */
            uint8_t *src = (uint8_t *)s->cmd_databuf;
            for (uint32_t i = 0; i < len; i++) {
                s->flash[addr + i] &= src[i];
            }
            hisi_sfc350_flush_to_file(s, addr, len);
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    case SPI_CMD_SE_4K:
        if ((s->sr & SPI_SR_WEL) && addr < s->flash_size) {
            uint32_t base = addr & ~(4 * 1024 - 1);
            uint32_t end = base + 4 * 1024;
            if (end > s->flash_size) end = s->flash_size;
            memset(&s->flash[base], 0xFF, end - base);
            hisi_sfc350_flush_to_file(s, base, end - base);
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    case SPI_CMD_SE_32K:
        if ((s->sr & SPI_SR_WEL) && addr < s->flash_size) {
            uint32_t base = addr & ~(32 * 1024 - 1);
            uint32_t end = base + 32 * 1024;
            if (end > s->flash_size) end = s->flash_size;
            memset(&s->flash[base], 0xFF, end - base);
            hisi_sfc350_flush_to_file(s, base, end - base);
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    case SPI_CMD_SE_64K:
        if ((s->sr & SPI_SR_WEL) && addr < s->flash_size) {
            uint32_t base = addr & ~(NOR_SECTOR_SIZE - 1);
            uint32_t end = base + NOR_SECTOR_SIZE;
            if (end > s->flash_size) end = s->flash_size;
            memset(&s->flash[base], 0xFF, end - base);
            hisi_sfc350_flush_to_file(s, base, end - base);
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    case SPI_CMD_CE:
        if (s->sr & SPI_SR_WEL) {
            memset(s->flash, 0xFF, s->flash_size);
            hisi_sfc350_flush_to_file(s, 0, s->flash_size);
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    case SPI_CMD_EN4B:
    case SPI_CMD_EX4B:
        /* Accept silently — we don't enforce address width */
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "hisi-sfc350: unhandled SPI command 0x%02x\n",
                      opcode);
        break;
    }

    /* Command completes instantly: clear START bit */
    s->cmd_config &= ~CMD_CONFIG_START;
}

/* ── DMA execution ───────────────────────────────────────────────────── */

static void hisi_sfc350_exec_dma(HisiSfc350State *s)
{
    bool is_read = s->bus_dma_ctrl & DMA_CTRL_RW_READ;
    uint32_t flash_addr = s->bus_dma_flash_saddr;
    uint32_t mem_addr = s->bus_dma_mem_saddr;
    uint32_t len = (s->bus_dma_len & 0x0FFFFFFF) + 1;

    if (flash_addr + len > s->flash_size) {
        len = (flash_addr < s->flash_size) ? s->flash_size - flash_addr : 0;
    }

    if (len > 0) {
        if (is_read) {
            dma_memory_write(&address_space_memory, mem_addr,
                             &s->flash[flash_addr], len,
                             MEMTXATTRS_UNSPECIFIED);
        } else {
            uint8_t *buf = g_malloc(len);
            dma_memory_read(&address_space_memory, mem_addr,
                            buf, len, MEMTXATTRS_UNSPECIFIED);
            /* NOR program: AND semantics */
            for (uint32_t i = 0; i < len; i++) {
                s->flash[flash_addr + i] &= buf[i];
            }
            g_free(buf);
            hisi_sfc350_flush_to_file(s, flash_addr, len);
        }
    }

    /* DMA complete */
    s->bus_dma_ctrl &= ~DMA_CTRL_START;
    s->int_raw_status |= INT_DMA_DONE;
}

/* ── MMIO: control registers ─────────────────────────────────────────── */

static uint64_t hisi_sfc350_ctrl_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    HisiSfc350State *s = HISI_SFC350(opaque);

    switch (offset) {
    case SFC_GLOBAL_CONFIG:     return s->global_config;
    case SFC_TIMING:            return s->timing;
    case SFC_INT_RAW_STATUS:    return s->int_raw_status;
    case SFC_INT_STATUS:        return s->int_raw_status & s->int_mask;
    case SFC_INT_MASK:          return s->int_mask;
    case SFC_VERSION:           return SFC350_VERSION_ID;

    case SFC_BUS_CONFIG1:       return s->bus_config1;
    case SFC_BUS_CONFIG2:       return s->bus_config2;
    case SFC_BUS_FLASH_SIZE:    return s->bus_flash_size;
    case SFC_BUS_BASE_ADDR_CS0: return s->bus_base_addr_cs0;
    case SFC_BUS_BASE_ADDR_CS1: return s->bus_base_addr_cs1;

    case SFC_BUS_DMA_CTRL:      return s->bus_dma_ctrl;
    case SFC_BUS_DMA_MEM_SADDR: return s->bus_dma_mem_saddr;
    case SFC_BUS_DMA_FLASH_SADDR: return s->bus_dma_flash_saddr;
    case SFC_BUS_DMA_LEN:       return s->bus_dma_len;
    case SFC_BUS_DMA_AHB_CTRL:  return s->bus_dma_ahb_ctrl;

    case SFC_CMD_CONFIG:        return s->cmd_config;
    case SFC_CMD_INS:           return s->cmd_ins;
    case SFC_CMD_ADDR:          return s->cmd_addr;

    default:
        if (offset >= SFC_CMD_DATABUF0 &&
            offset < SFC_CMD_DATABUF0 + DATABUF_REGS * 4) {
            return s->cmd_databuf[(offset - SFC_CMD_DATABUF0) / 4];
        }
        return 0;
    }
}

static void hisi_sfc350_ctrl_write(void *opaque, hwaddr offset,
                                    uint64_t val, unsigned size)
{
    HisiSfc350State *s = HISI_SFC350(opaque);

    switch (offset) {
    case SFC_GLOBAL_CONFIG:     s->global_config = val; break;
    case SFC_TIMING:            s->timing = val; break;
    case SFC_INT_MASK:          s->int_mask = val; break;
    case SFC_INT_CLEAR:
        s->int_raw_status &= ~val;
        break;

    case SFC_BUS_CONFIG1:       s->bus_config1 = val; break;
    case SFC_BUS_CONFIG2:       s->bus_config2 = val; break;
    case SFC_BUS_FLASH_SIZE:    s->bus_flash_size = val; break;
    case SFC_BUS_BASE_ADDR_CS0: s->bus_base_addr_cs0 = val; break;
    case SFC_BUS_BASE_ADDR_CS1: s->bus_base_addr_cs1 = val; break;

    case SFC_BUS_DMA_MEM_SADDR:   s->bus_dma_mem_saddr = val; break;
    case SFC_BUS_DMA_FLASH_SADDR: s->bus_dma_flash_saddr = val; break;
    case SFC_BUS_DMA_LEN:         s->bus_dma_len = val; break;
    case SFC_BUS_DMA_AHB_CTRL:    s->bus_dma_ahb_ctrl = val; break;

    case SFC_BUS_DMA_CTRL:
        s->bus_dma_ctrl = val;
        if (val & DMA_CTRL_START) {
            hisi_sfc350_exec_dma(s);
        }
        break;

    case SFC_CMD_INS:   s->cmd_ins = val; break;
    case SFC_CMD_ADDR:  s->cmd_addr = val; break;

    case SFC_CMD_CONFIG:
        s->cmd_config = val;
        if (val & CMD_CONFIG_START) {
            hisi_sfc350_exec_cmd(s);
        }
        break;

    default:
        if (offset >= SFC_CMD_DATABUF0 &&
            offset < SFC_CMD_DATABUF0 + DATABUF_REGS * 4) {
            s->cmd_databuf[(offset - SFC_CMD_DATABUF0) / 4] = val;
            break;
        }
        break;
    }
}

static const MemoryRegionOps hisi_sfc350_ctrl_ops = {
    .read = hisi_sfc350_ctrl_read,
    .write = hisi_sfc350_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ── MMIO: memory window (bus-mode reads) ────────────────────────────── */

static uint64_t hisi_sfc350_mem_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    HisiSfc350State *s = HISI_SFC350(opaque);
    uint32_t val = 0xFFFFFFFF;

    if (offset < s->flash_size) {
        memcpy(&val, &s->flash[offset], size > 4 ? 4 : size);
    }
    return val;
}

static void hisi_sfc350_mem_write(void *opaque, hwaddr offset,
                                   uint64_t val, unsigned size)
{
    /* Bus-mode writes not supported (use register or DMA mode) */
}

static const MemoryRegionOps hisi_sfc350_mem_ops = {
    .read = hisi_sfc350_mem_read,
    .write = hisi_sfc350_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ── Device lifecycle ────────────────────────────────────────────────── */

static void hisi_sfc350_init(Object *obj)
{
    HisiSfc350State *s = HISI_SFC350(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->ctrl_iomem, obj, &hisi_sfc350_ctrl_ops, s,
                          "hisi-sfc350-ctrl", CTRL_REG_SIZE);
    sysbus_init_mmio(sbd, &s->ctrl_iomem);

    memory_region_init_io(&s->mem_iomem, obj, &hisi_sfc350_mem_ops, s,
                          "hisi-sfc350-mem", MEM_WINDOW_SIZE);
    sysbus_init_mmio(sbd, &s->mem_iomem);
}

static void hisi_sfc350_realize(DeviceState *dev, Error **errp)
{
    HisiSfc350State *s = HISI_SFC350(dev);

    s->flash_size = NOR_FLASH_SIZE;
    s->jedec_cap = 0x17;
    s->flash = g_malloc(s->flash_size);
    memset(s->flash, 0xFF, s->flash_size);

    /* SFC_BUS_FLASH_SIZE: bits [1:0] = CS0 flash size
     * 0=16MiB, 1=8MiB, 2=4MiB, 3=2MiB.
     * Default to 8 MiB (W25Q64) so the kernel detects the correct size. */
    s->bus_flash_size = 0x01;

    /* Load flash contents from file if specified */
    if (s->flash_file && s->flash_file[0]) {
        struct stat st;
        if (stat(s->flash_file, &st) != 0) {
            error_setg(errp, "hisi-sfc350: cannot stat flash file '%s'",
                       s->flash_file);
            return;
        }
        /* The window maps at most 32 MiB, so a bigger part could not be
         * read through it; refuse rather than show a chip half of which is
         * out of reach. */
        if (st.st_size > MEM_WINDOW_SIZE) {
            error_setg(errp, "hisi-sfc350: flash file '%s' is %lld bytes, "
                       "more than the %u the controller maps",
                       s->flash_file, (long long)st.st_size,
                       MEM_WINDOW_SIZE);
            return;
        }
        uint32_t file_size = (uint32_t)st.st_size;
        /* A part is a power of two in size: an odd-sized dump (a backup with
         * a trailer) gets the chip that holds it, the rest erased, so what
         * RDID says and what can be read are the same size. */
        if (file_size > s->flash_size) {
            g_free(s->flash);
            s->flash_size = pow2ceil(file_size);
            s->flash = g_malloc(s->flash_size);
            memset(s->flash, 0xFF, s->flash_size);
        }
        FILE *f = fopen(s->flash_file, "rb");
        if (!f) {
            error_setg(errp, "hisi-sfc350: cannot open flash file '%s'",
                       s->flash_file);
            return;
        }
        size_t nread = fread(s->flash, 1, file_size, f);
        fclose(f);
        if (nread != file_size) {
            error_setg(errp, "hisi-sfc350: short read from '%s' "
                       "(got %zu, expected %u)", s->flash_file,
                       nread, file_size);
            return;
        }
        qemu_log("hisi-sfc350: loaded %u bytes from '%s'\n",
                 file_size, s->flash_file);

        /* Answer RDID and BUS_FLASH_SIZE as the part of that size would: a
         * 16 MiB image read as a W25Q64 gave the kernel an 8 MiB MTD, so
         * an "-(rootfs_data)" tail partition came out wrong. */
        if (s->flash_size > NOR_FLASH_SIZE) {
            s->jedec_cap = 31 - clz32(s->flash_size);
            s->bus_flash_size = 0x00;        /* 16 MiB, the field's largest */
        }
    }
}

static void hisi_sfc350_finalize(Object *obj)
{
    HisiSfc350State *s = HISI_SFC350(obj);
    g_free(s->flash);
}

static const Property hisi_sfc350_properties[] = {
    DEFINE_PROP_STRING("flash-file", HisiSfc350State, flash_file),
};

static void hisi_sfc350_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = hisi_sfc350_realize;
    device_class_set_props(dc, hisi_sfc350_properties);
}

static const TypeInfo hisi_sfc350_info = {
    .name              = TYPE_HISI_SFC350,
    .parent            = TYPE_SYS_BUS_DEVICE,
    .instance_size     = sizeof(HisiSfc350State),
    .instance_init     = hisi_sfc350_init,
    .instance_finalize = hisi_sfc350_finalize,
    .class_init        = hisi_sfc350_class_init,
};

static void hisi_sfc350_register_types(void)
{
    type_register_static(&hisi_sfc350_info);
}

type_init(hisi_sfc350_register_types)
