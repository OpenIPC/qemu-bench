/*
 * Ambarella S3L NAND: the FIO flash controller, FDMA channel 0 and a chip.
 *
 * Software drives two engines for one page.  FDMA moves data between memory
 * and the FIO FIFO, with a separate spare-area transfer in dual-space mode, and
 * FIO DMA moves it between the FIFO and the chip.  Nothing moves until FIO DMA
 * is enabled, FDMA having been set up first, so the page is transferred in that
 * write.  A page takes three interrupts, and the driver waits for all of them:
 * FIO command (FI, cleared through FLASH_INT), FIO DMA (DMASTA) and FDMA (STA).
 * Commands with no data take FLASH_CMD, low nibble the operation, the rest of
 * the word and CTR's top nibble the byte address.
 *
 * The chip is a 128 MiB part: 2048 + 64 byte pages, 64 to a block.  It answers
 * 98 f1 80 15 00.  Only the first two bytes are the camera's.  The next two give
 * that geometry through the kernel's generic decode, and the fifth keeps it out
 * of nand_ids.c, whose 98 f1 80 15 72 entry has 128 byte spare areas.
 * Programming clears bits and erasing sets them, as on a chip.
 *
 * BCH is not modelled.  Nothing is ever corrupted here, so the ECC report stays
 * clean, and the parity bytes a real controller would put in the spare area are
 * not generated: spare bytes are stored as they are written.  So an image
 * cannot be taken from here to a chip.
 *
 * flash-file is 128 MiB of page data, or 132 MiB of pages with their spare
 * bytes behind each, as "nanddump -o" writes them.  With page data alone the
 * spare area starts blank.  Erases and programs are written through, the spare
 * bytes only to a file that has them.  A file that cannot be opened for writing
 * is left alone and the guest's changes stay in memory.
 *
 * A dump cannot show bad blocks: a block that read as nothing looks erased.
 * bad-blocks lists them, as byte offsets separated by colons: each gets the
 * factory marker in its first spare byte, and fails erase and program.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/blocker.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "system/dma.h"

#define TYPE_AMB_NAND "amb-nand"
OBJECT_DECLARE_SIMPLE_TYPE(AmbNandState, AMB_NAND)

#define NAND_PAGE       2048
#define NAND_SPARE      64
#define NAND_RAW        (NAND_PAGE + NAND_SPARE)
#define NAND_PAGES      65536
#define NAND_BLOCK_PAGES 64

/* FIO, in words */
#define FIO_STA         (0x004 / 4)
#define FIO_DMACTR      (0x080 / 4)
#define FIO_DMAADR      (0x084 / 4)
#define FIO_DMASTA      (0x08c / 4)
#define FLASH_CTR       (0x120 / 4)
#define FLASH_CMD       (0x124 / 4)
#define FLASH_STA       (0x140 / 4)
#define FLASH_ID        (0x144 / 4)
#define FLASH_INT       (0x150 / 4)
#define FLASH_EX_ID     (0x160 / 4)
#define FIO_REGS        (0x1000 / 4)

#define FIO_STA_FI      0x00000001
#define FIO_DMACTR_EN   0x80000000
#define FIO_DMACTR_RM   0x40000000  /* memory to device */
#define FIO_DMACTR_DEV  0x30000000  /* which device: 0 is flash */
#define FIO_DMACTR_LEN  0x003fffff
#define FIO_DMASTA_DN   0x01000000
#define FIO_DMASTA_AE   0x02000000

#define NAND_CTR_A_SHIFT 28
#define NAND_CTR_SA     0x08000000
#define NAND_CTR_WP     0x00000200

#define CMD_RESET       0x2
#define CMD_ERASE       0x9
#define CMD_READID      0xa
#define CMD_READSTATUS  0xc

#define NAND_ID         0x98f18015
#define NAND_ID5        0x00

#define STATUS_FAIL     0x01
#define STATUS_READY    0x40
#define STATUS_NOT_WP   0x80

/* FDMA, in words: channel 0 is FIO's */
#define FDMA_SPR_CNT    (0x200 / 4)
#define FDMA_SPR_SRC    (0x204 / 4)
#define FDMA_SPR_DST    (0x208 / 4)
#define FDMA_SPR_STA    (0x20c / 4)
#define FDMA_CTR        (0x300 / 4)
#define FDMA_SRC        (0x304 / 4)
#define FDMA_DST        (0x308 / 4)
#define FDMA_STA        (0x30c / 4)
#define FDMA_SPR_DA     (0x280 / 4)
#define FDMA_DA         (0x380 / 4)
#define FDMA_INT        (0x3f0 / 4)
#define FDMA_REGS       (0x1000 / 4)

#define FDMA_CTR_EN     0x80000000
#define FDMA_CTR_D      0x40000000  /* the transfer is in a descriptor */
#define FDMA_CTR_LEN    0x003fffff
#define FDMA_STA_AE     0x00800000
#define FDMA_STA_DN     0x00400000

struct AmbNandState {
    SysBusDevice parent_obj;

    MemoryRegion fio_io;
    MemoryRegion fdma_io;
    qemu_irq cmd_irq;
    qemu_irq dma_irq;
    qemu_irq fdma_irq;

    char *flash_file;
    char *bad_blocks;
    int fd;                     /* for write-through, or -1 */
    bool raw_file;              /* the file has spare bytes */
    Error *migration_blocker;

    uint8_t *array;             /* pages, each with its spare bytes behind */
    unsigned long *bad;         /* by block */
    uint8_t status_fail;

    uint32_t fio[FIO_REGS];
    uint32_t fdma[FDMA_REGS];
};

static void amb_nand_update(AmbNandState *s)
{
    qemu_set_irq(s->cmd_irq, !!(s->fio[FIO_STA] & FIO_STA_FI));
    qemu_set_irq(s->dma_irq, s->fio[FIO_DMASTA] != 0);
    qemu_set_irq(s->fdma_irq, s->fdma[FDMA_STA] != 0);
}

static void amb_nand_finish(AmbNandState *s)
{
    s->fio[FIO_STA] |= FIO_STA_FI;
    amb_nand_update(s);
}

static bool amb_nand_persist(AmbNandState *s, uint32_t page, uint32_t n)
{
    size_t unit = s->raw_file ? NAND_RAW : NAND_PAGE;

    if (s->fd < 0) {
        return true;
    }
    for (uint32_t i = page; i < page + n; i++) {
        if (pwrite(s->fd, s->array + (size_t)i * NAND_RAW, unit,
                   (off_t)i * unit) != (ssize_t)unit) {
            error_report("amb-nand: write-through to %s failed at page %u: %s",
                         s->flash_file, i, strerror(errno));
            return false;
        }
    }
    return true;
}

static bool amb_nand_writable(AmbNandState *s, uint32_t page)
{
    s->status_fail = 0;
    if ((s->fio[FLASH_CTR] & NAND_CTR_WP) ||
        test_bit(page / NAND_BLOCK_PAGES, s->bad)) {
        s->status_fail = STATUS_FAIL;
        return false;
    }
    return true;
}

static uint64_t amb_nand_address(AmbNandState *s, uint32_t low)
{
    uint64_t hi = s->fio[FLASH_CTR] >> NAND_CTR_A_SHIFT;

    return (hi << 32) | low;
}

static void amb_nand_command(AmbNandState *s, uint32_t val)
{
    uint64_t addr = amb_nand_address(s, val & ~0xf);
    uint32_t page = addr / NAND_PAGE;

    switch (val & 0xf) {
    case CMD_RESET:
        s->status_fail = 0;
        break;
    case CMD_READID:
    case CMD_READSTATUS:
        break;
    case CMD_ERASE:
        page &= ~(NAND_BLOCK_PAGES - 1);
        if (page >= NAND_PAGES) {
            s->status_fail = STATUS_FAIL;
        } else if (amb_nand_writable(s, page)) {
            memset(s->array + (size_t)page * NAND_RAW, 0xff,
                   (size_t)NAND_BLOCK_PAGES * NAND_RAW);
            if (!amb_nand_persist(s, page, NAND_BLOCK_PAGES)) {
                s->status_fail = STATUS_FAIL;
            }
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "amb-nand: command 0x%x\n", val & 0xf);
        break;
    }
    amb_nand_finish(s);
}

/* What FDMA would have moved through the FIFO, to or from one area */
static bool amb_nand_move(uint8_t *chip, hwaddr mem, uint32_t len, bool to_chip)
{
    if (!to_chip) {
        return dma_memory_write(&address_space_memory, mem, chip, len,
                                MEMTXATTRS_UNSPECIFIED) == MEMTX_OK;
    }

    g_autofree uint8_t *buf = g_malloc(len);

    if (dma_memory_read(&address_space_memory, mem, buf, len,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    for (uint32_t i = 0; i < len; i++) {
        chip[i] &= buf[i];
    }
    return true;
}

/*
 * A DMA descriptor, which the boot loader uses where the kernel programs the
 * registers.  Words 0 to 5: source, destination, next, where to write the
 * report, count, control.
 */
typedef struct AmbNandDesc {
    bool used;
    hwaddr src;
    hwaddr dst;
    hwaddr report;
    uint32_t count;
} AmbNandDesc;

static AmbNandDesc amb_nand_desc(hwaddr addr)
{
    AmbNandDesc d = { 0 };
    uint32_t w[5];

    if (addr && dma_memory_read(&address_space_memory, addr, w, sizeof(w),
                                MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
        d.used = true;
        d.src = le32_to_cpu(w[0]);
        d.dst = le32_to_cpu(w[1]);
        d.report = le32_to_cpu(w[3]);
        d.count = le32_to_cpu(w[4]);
    }
    return d;
}

static void amb_nand_desc_report(const AmbNandDesc *d, uint32_t status)
{
    uint32_t val = cpu_to_le32(status);

    if (d->used && d->report) {
        dma_memory_write(&address_space_memory, d->report, &val, sizeof(val),
                         MEMTXATTRS_UNSPECIFIED);
    }
}

static void amb_nand_transfer(AmbNandState *s, uint32_t ctr)
{
    uint64_t addr = amb_nand_address(s, s->fio[FIO_DMAADR]);
    uint32_t page = addr / NAND_PAGE;
    uint32_t column = addr % NAND_PAGE;
    bool to_chip = ctr & FIO_DMACTR_RM;
    uint32_t len = s->fdma[FDMA_CTR] & FDMA_CTR_LEN;
    uint32_t spare = s->fdma[FDMA_SPR_CNT];
    hwaddr main = to_chip ? s->fdma[FDMA_SRC] : s->fdma[FDMA_DST];
    hwaddr aux = to_chip ? s->fdma[FDMA_SPR_SRC] : s->fdma[FDMA_SPR_DST];
    uint8_t *chip;
    bool ok = true;
    AmbNandDesc desc = { 0 }, spare_desc = { 0 };

    if (s->fdma[FDMA_CTR] & FDMA_CTR_D) {
        desc = amb_nand_desc(s->fdma[FDMA_DA]);
        spare_desc = amb_nand_desc(s->fdma[FDMA_SPR_DA]);
        len = desc.count & FDMA_CTR_LEN;
        main = to_chip ? desc.src : desc.dst;
        spare = spare_desc.count;
        aux = to_chip ? spare_desc.src : spare_desc.dst;
    }

    /* Spare-only commands address the spare area with the main transfer */
    if (s->fio[FLASH_CTR] & NAND_CTR_SA) {
        column += NAND_PAGE;
        spare = 0;
    }

    s->fio[FIO_DMACTR] = ctr & ~FIO_DMACTR_EN;
    s->fdma[FDMA_CTR] &= ~FDMA_CTR_EN;

    /*
     * A read may run over many pages: the kernel asks for one, the BST for a
     * block at a time, its spare bytes following in their own transfer.
     */
    if (!to_chip && !column && len > NAND_PAGE && !(len % NAND_PAGE) &&
        (spare == 0 || spare == NAND_SPARE * (len / NAND_PAGE)) &&
        page + len / NAND_PAGE <= NAND_PAGES) {
        uint32_t pages = len / NAND_PAGE;
        uint32_t each = spare / pages;      /* 0 or NAND_SPARE */

        for (uint32_t i = 0; ok && i < pages; i++) {
            chip = s->array + (size_t)(page + i) * NAND_RAW;
            ok = amb_nand_move(chip, main + i * NAND_PAGE, NAND_PAGE, false);
            if (ok && each) {
                ok = amb_nand_move(chip + NAND_PAGE, aux + i * each, each,
                                   false);
            }
        }
    } else if (page >= NAND_PAGES || column + len > NAND_RAW ||
               spare > NAND_SPARE || (spare && len != NAND_PAGE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "amb-nand: page %u column %u len %u "
                      "spare %u\n", page, column, len, spare);
        ok = false;
    } else if (!to_chip || amb_nand_writable(s, page)) {
        chip = s->array + (size_t)page * NAND_RAW;
        ok = amb_nand_move(chip + column, main, len, to_chip);
        if (ok && spare) {
            ok = amb_nand_move(chip + NAND_PAGE, aux, spare, to_chip);
        }
        if (ok && to_chip) {
            ok = amb_nand_persist(s, page, 1);
        }
    }

    s->fio[FIO_DMASTA] = ok ? FIO_DMASTA_DN : FIO_DMASTA_AE;
    s->fdma[FDMA_STA] = ok ? FDMA_STA_DN : FDMA_STA_AE;
    s->fdma[FDMA_SPR_STA] = spare ? s->fdma[FDMA_STA] : 0;
    amb_nand_desc_report(&desc, s->fdma[FDMA_STA]);
    amb_nand_desc_report(&spare_desc, s->fdma[FDMA_STA]);
    amb_nand_finish(s);
}

static uint64_t amb_nand_fio_read(void *opaque, hwaddr off, unsigned size)
{
    AmbNandState *s = opaque;
    uint32_t r = off / 4;

    switch (r) {
    case FLASH_STA:
        return STATUS_READY | s->status_fail |
               (s->fio[FLASH_CTR] & NAND_CTR_WP ? 0 : STATUS_NOT_WP);
    case FLASH_ID:
        return NAND_ID;
    case FLASH_EX_ID:
        return NAND_ID5;
    case FLASH_INT:
        /* The kernel looks at FIO_STA, the BST polls this */
        return s->fio[FIO_STA] & FIO_STA_FI;
    default:
        return s->fio[r];
    }
}

static void amb_nand_fio_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    AmbNandState *s = opaque;
    uint32_t r = off / 4;

    switch (r) {
    case FIO_STA:
        break;
    case FIO_DMACTR:
        if ((val & FIO_DMACTR_EN) && !(val & FIO_DMACTR_DEV)) {
            amb_nand_transfer(s, val);
        } else {
            s->fio[r] = val;
        }
        break;
    case FIO_DMASTA:
        s->fio[r] = val;
        amb_nand_update(s);
        break;
    case FLASH_CMD:
        amb_nand_command(s, val);
        break;
    case FLASH_INT:
        s->fio[FIO_STA] &= ~FIO_STA_FI;
        amb_nand_update(s);
        break;
    default:
        s->fio[r] = val;
        break;
    }
}

static const MemoryRegionOps amb_nand_fio_ops = {
    .read = amb_nand_fio_read,
    .write = amb_nand_fio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t amb_nand_fdma_read(void *opaque, hwaddr off, unsigned size)
{
    AmbNandState *s = opaque;

    if (off / 4 == FDMA_INT) {
        return s->fdma[FDMA_STA] ? 1 : 0;
    }
    return s->fdma[off / 4];
}

static void amb_nand_fdma_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    AmbNandState *s = opaque;

    s->fdma[off / 4] = val;
    if (off / 4 == FDMA_STA) {
        amb_nand_update(s);
    }
}

static const MemoryRegionOps amb_nand_fdma_ops = {
    .read = amb_nand_fdma_read,
    .write = amb_nand_fdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void amb_nand_mark_bad(AmbNandState *s, Error **errp)
{
    g_auto(GStrv) list = NULL;

    if (!s->bad_blocks || !*s->bad_blocks) {
        return;
    }
    list = g_strsplit(s->bad_blocks, ":", -1);
    for (int i = 0; list[i]; i++) {
        uint64_t offset;
        uint32_t block;

        if (qemu_strtou64(list[i], NULL, 0, &offset) < 0 ||
            offset % (NAND_BLOCK_PAGES * NAND_PAGE) ||
            offset >= (uint64_t)NAND_PAGES * NAND_PAGE) {
            error_setg(errp, "amb-nand: bad-blocks: '%s' is not the offset of "
                       "a block", list[i]);
            return;
        }
        block = offset / (NAND_BLOCK_PAGES * NAND_PAGE);
        set_bit(block, s->bad);
        s->array[(size_t)block * NAND_BLOCK_PAGES * NAND_RAW + NAND_PAGE] = 0;
    }
}

static void amb_nand_load(AmbNandState *s, Error **errp)
{
    const size_t raw = (size_t)NAND_PAGES * NAND_RAW;
    const size_t data = (size_t)NAND_PAGES * NAND_PAGE;
    size_t unit;
    struct stat st;
    int fd;

    fd = open(s->flash_file, O_RDWR);
    if (fd < 0 && (errno == EACCES || errno == EROFS)) {
        fd = open(s->flash_file, O_RDONLY);
        if (fd >= 0) {
            warn_report("amb-nand: %s is read-only: the guest's writes stay "
                        "in memory", s->flash_file);
            s->fd = -2;
        }
    }
    if (fd < 0 || fstat(fd, &st) < 0) {
        error_setg_errno(errp, errno, "amb-nand: cannot open %s",
                         s->flash_file);
        return;
    }
    if ((size_t)st.st_size == data) {
        s->raw_file = false;
    } else if ((size_t)st.st_size == raw) {
        s->raw_file = true;
    } else {
        error_setg(errp, "amb-nand: %s is %" PRIu64 " bytes: expected %zu of "
                   "page data, or %zu with spare bytes", s->flash_file,
                   (uint64_t)st.st_size, data, raw);
        close(fd);
        return;
    }

    unit = s->raw_file ? NAND_RAW : NAND_PAGE;
    for (uint32_t i = 0; i < NAND_PAGES; i++) {
        if (pread(fd, s->array + (size_t)i * NAND_RAW, unit,
                  (off_t)i * unit) != (ssize_t)unit) {
            error_setg_errno(errp, errno, "amb-nand: reading %s",
                             s->flash_file);
            close(fd);
            return;
        }
    }

    if (s->fd == -2) {
        close(fd);
        s->fd = -1;
    } else {
        s->fd = fd;
    }
}

static void amb_nand_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    AmbNandState *s = AMB_NAND(dev);

    s->array = g_malloc((size_t)NAND_PAGES * NAND_RAW);
    memset(s->array, 0xff, (size_t)NAND_PAGES * NAND_RAW);
    s->bad = bitmap_new(NAND_PAGES / NAND_BLOCK_PAGES);

    if (!s->flash_file || !*s->flash_file) {
        error_setg(errp, "amb-nand: flash-file is required");
        return;
    }
    amb_nand_load(s, errp);
    if (*errp) {
        return;
    }
    amb_nand_mark_bad(s, errp);
    if (*errp) {
        return;
    }

    error_setg(&s->migration_blocker, "amb-nand: flash-file %s is written "
               "through; it cannot be migrated", s->flash_file);
    if (migrate_add_blocker(&s->migration_blocker, errp) < 0) {
        return;
    }
}

static void amb_nand_reset(DeviceState *dev)
{
    AmbNandState *s = AMB_NAND(dev);

    memset(s->fio, 0, sizeof(s->fio));
    memset(s->fdma, 0, sizeof(s->fdma));
    s->status_fail = 0;
    amb_nand_update(s);
}

static void amb_nand_init(Object *obj)
{
    AmbNandState *s = AMB_NAND(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->fd = -1;
    memory_region_init_io(&s->fio_io, obj, &amb_nand_fio_ops, s, "amb-fio",
                          0x1000);
    memory_region_init_io(&s->fdma_io, obj, &amb_nand_fdma_ops, s, "amb-fdma",
                          0x1000);
    sysbus_init_mmio(sbd, &s->fio_io);
    sysbus_init_mmio(sbd, &s->fdma_io);
    sysbus_init_irq(sbd, &s->cmd_irq);
    sysbus_init_irq(sbd, &s->dma_irq);
    sysbus_init_irq(sbd, &s->fdma_irq);
}

static const Property amb_nand_props[] = {
    DEFINE_PROP_STRING("flash-file", AmbNandState, flash_file),
    DEFINE_PROP_STRING("bad-blocks", AmbNandState, bad_blocks),
};

static void amb_nand_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = amb_nand_realize;
    device_class_set_legacy_reset(dc, amb_nand_reset);
    device_class_set_props(dc, amb_nand_props);
}

static const TypeInfo amb_nand_info = {
    .name = TYPE_AMB_NAND,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AmbNandState),
    .instance_init = amb_nand_init,
    .class_init = amb_nand_class_init,
};

static void amb_nand_register(void)
{
    type_register_static(&amb_nand_info);
}

type_init(amb_nand_register)
