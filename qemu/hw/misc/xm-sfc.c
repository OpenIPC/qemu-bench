/*
 * Xiongmai XM510 SPI-NOR flash controller ("xmsfc", v1).
 *
 * Not a HiSilicon hifmc: Xiongmai's own register block, as driven by
 * drivers/mtd/devices/xmsfc/xmsfc.c in the vendor 3.0.101 kernel and by the
 * board's U-Boot 2014.04.
 *
 * Two regions:
 *  - registers (0x800 at 0x100F0000):
 *      MODE      +0x00   ADDR  +0x04   OP    +0x08   CMD    +0x0C
 *      SRW       +0x10   WRNUM +0x14   RW_SR_BSEL +0x18
 *      DOUT      +0x34   ST    +0x38   SRR   +0x3C   MODE_ST +0x4C
 *      CACHE_CMD +0x80   BUF   +0x400 (256 bytes, one per 32-bit word)
 *    Software loads CMD (the SPI opcode) and ADDR/WRNUM/BUF, writes an
 *    operation number to OP, and polls ST for that operation's done bit:
 *      OP2 erase (CMD 0xD8/0x52/0x20/0xC7/0x60 at ADDR)
 *      OP3 read status/ID: RW_SR_BSEL+1 bytes into SRR, first byte highest
 *          (JEDEC 0x9F with BSEL 2 gives 0xef4017 for a W25Q64)
 *      OP4 write status (SRW, BSEL+1 bytes)
 *      OP6 page program: WRNUM+1 bytes from BUF to ADDR
 *    Every operation completes before the write to OP returns, so ST holds
 *    exactly the done bit of the last one.
 *  - the read window (16 MiB at 0x0): the kernel's mtd read is a memcpy from
 *    it, and U-Boot runs from it in place, so it is a ROM device that reads
 *    the flash array directly.
 *
 * Erases and programs are written through to flash-file, so what the guest
 * stores survives a reboot, as on a camera.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_XM_SFC "xm-sfc"
OBJECT_DECLARE_SIMPLE_TYPE(XmSfcState, XM_SFC)

#define R_MODE        0x00
#define R_ADDR        0x04
#define R_OP          0x08
#define R_CMD         0x0C
#define R_SRW         0x10
#define R_WRNUM       0x14
#define R_BSEL        0x18
#define R_DOUT        0x34
#define R_ST          0x38
#define R_SRR         0x3C
#define R_MODE_ST     0x4C
#define R_CACHE_CMD   0x80
#define R_BUF         0x400
#define BUF_BYTES     256
#define REGS_SIZE     0x800

#define WINDOW_SIZE   (16 * MiB)

struct XmSfcState {
    SysBusDevice parent_obj;

    MemoryRegion regs;
    MemoryRegion window;

    uint32_t mode, addr, cmd, srw, wrnum, bsel, st, srr, cache_cmd;
    uint8_t buf[BUF_BYTES];
    uint8_t sr1, sr2;

    uint8_t *array;           /* the flash, size bytes */
    int fd;                   /* flash-file, or -1 */

    char *flash_file;
    uint32_t size;
    uint32_t jedec;
};

static void xm_sfc_persist(XmSfcState *s, uint32_t off, uint32_t len)
{
    if (s->fd < 0) {
        return;
    }
    if (pwrite(s->fd, s->array + off, len, off) != (ssize_t)len) {
        error_report("xm-sfc: write-through to %s failed at 0x%x: %s",
                     s->flash_file, off, strerror(errno));
    }
}

static void xm_sfc_erase(XmSfcState *s)
{
    uint32_t len;

    switch (s->cmd) {
    case 0x20:
        len = 4 * KiB;
        break;
    case 0x52:
        len = 32 * KiB;
        break;
    case 0xD8:
        len = 64 * KiB;
        break;
    case 0xC7:
    case 0x60:
        memset(s->array, 0xff, s->size);
        memory_region_flush_rom_device(&s->window, 0, s->size);
        xm_sfc_persist(s, 0, s->size);
        return;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: erase opcode 0x%02x\n", s->cmd);
        return;
    }

    const uint32_t off = s->addr & ~(len - 1);
    if (off >= s->size) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-sfc: erase at 0x%x past the end\n",
                      s->addr);
        return;
    }
    len = MIN(len, s->size - off);
    memset(s->array + off, 0xff, len);
    memory_region_flush_rom_device(&s->window, off, len);
    xm_sfc_persist(s, off, len);
}

/* NOR programming can only clear bits, and a page program wraps within its
 * 256-byte page. */
static void xm_sfc_program(XmSfcState *s)
{
    const uint32_t n = MIN(s->wrnum + 1, BUF_BYTES);
    const uint32_t page = s->addr & ~(BUF_BYTES - 1);
    uint32_t i;

    if (page >= s->size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "xm-sfc: program at 0x%x past the end\n", s->addr);
        return;
    }
    for (i = 0; i < n; i++) {
        const uint32_t off = page + ((s->addr + i) & (BUF_BYTES - 1));
        s->array[off] &= s->buf[i];
    }
    memory_region_flush_rom_device(&s->window, page, BUF_BYTES);
    xm_sfc_persist(s, page, BUF_BYTES);
}

/* RW_SR_BSEL + 1 bytes of the opcode's answer, first byte highest. */
static void xm_sfc_read_reg(XmSfcState *s)
{
    uint8_t out[4] = { 0 };
    const uint32_t n = MIN(s->bsel + 1, 4);
    uint32_t v = 0, i;

    switch (s->cmd) {
    case 0x9F:
        out[0] = s->jedec >> 16;
        out[1] = s->jedec >> 8;
        out[2] = s->jedec;
        break;
    case 0x05:
        out[0] = s->sr1;      /* never busy: every operation is done */
        break;
    case 0x35:
        out[0] = s->sr2;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: read opcode 0x%02x\n", s->cmd);
        break;
    }
    for (i = 0; i < n; i++) {
        v = (v << 8) | out[i];
    }
    s->srr = v;
}

static void xm_sfc_write_reg(XmSfcState *s)
{
    if (s->cmd != 0x01) {
        qemu_log_mask(LOG_UNIMP, "xm-sfc: write opcode 0x%02x\n", s->cmd);
        return;
    }
    if (s->bsel >= 1) {
        s->sr1 = (s->srw >> 8) & 0xfc;
        s->sr2 = s->srw & 0xff;
    } else {
        s->sr1 = s->srw & 0xfc;
    }
}

static void xm_sfc_op(XmSfcState *s, uint32_t op)
{
    switch (op) {
    case 2:
        xm_sfc_erase(s);
        break;
    case 3:
        xm_sfc_read_reg(s);
        break;
    case 4:
        xm_sfc_write_reg(s);
        break;
    case 6:
        xm_sfc_program(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: operation %u\n", op);
        break;
    }
    s->st = op >= 1 && op <= 6 ? 1u << (op - 1) : 0;
}

static uint64_t xm_sfc_regs_read(void *opaque, hwaddr off, unsigned size)
{
    XmSfcState *s = opaque;

    if (off >= R_BUF && off < R_BUF + 4 * BUF_BYTES) {
        return s->buf[(off - R_BUF) / 4];
    }
    switch (off) {
    case R_MODE:
        return s->mode;
    case R_ADDR:
        return s->addr;
    case R_CMD:
        return s->cmd;
    case R_SRW:
        return s->srw;
    case R_WRNUM:
        return s->wrnum;
    case R_BSEL:
        return s->bsel;
    case R_ST:
        return s->st;
    case R_SRR:
        return s->srr;
    case R_MODE_ST:
        return 1;             /* a mode change is done at once */
    case R_CACHE_CMD:
        return s->cache_cmd;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
}

static void xm_sfc_regs_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    XmSfcState *s = opaque;

    if (off >= R_BUF && off < R_BUF + 4 * BUF_BYTES) {
        s->buf[(off - R_BUF) / 4] = val;
        return;
    }
    switch (off) {
    case R_MODE:
        s->mode = val;
        break;
    case R_ADDR:
        s->addr = val;
        break;
    case R_OP:
        xm_sfc_op(s, val);
        break;
    case R_CMD:
        s->cmd = val & 0xff;
        break;
    case R_SRW:
        s->srw = val;
        break;
    case R_WRNUM:
        s->wrnum = val;
        break;
    case R_BSEL:
        s->bsel = val;
        break;
    case R_CACHE_CMD:
        s->cache_cmd = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", off, val);
        break;
    }
}

static const MemoryRegionOps xm_sfc_regs_ops = {
    .read = xm_sfc_regs_read,
    .write = xm_sfc_regs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* The read window ignores writes, as the controller does. */
static void xm_sfc_window_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR,
                  "xm-sfc: write to the read window at 0x%" HWADDR_PRIx "\n",
                  off);
}

static const MemoryRegionOps xm_sfc_window_ops = {
    .write = xm_sfc_window_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void xm_sfc_realize(DeviceState *dev, Error **errp)
{
    XmSfcState *s = XM_SFC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->fd = -1;
    if (s->flash_file && *s->flash_file) {
        s->fd = open(s->flash_file, O_RDWR);
        if (s->fd < 0) {
            error_setg_errno(errp, errno, "xm-sfc: cannot open %s",
                             s->flash_file);
            return;
        }
        const off_t len = lseek(s->fd, 0, SEEK_END);
        if (len > 0 && len <= WINDOW_SIZE) {
            s->size = len;
        }
    }
    if (!s->size || s->size > WINDOW_SIZE) {
        error_setg(errp, "xm-sfc: flash size 0x%x not in (0, 16 MiB]",
                   s->size);
        return;
    }

    memory_region_init_io(&s->regs, OBJECT(s), &xm_sfc_regs_ops, s,
                          "xm-sfc.regs", REGS_SIZE);
    sysbus_init_mmio(sbd, &s->regs);

    memory_region_init_rom_device(&s->window, OBJECT(s), &xm_sfc_window_ops,
                                  s, "xm-sfc.window", s->size, errp);
    if (*errp) {
        return;
    }
    s->array = memory_region_get_ram_ptr(&s->window);
    memset(s->array, 0xff, s->size);
    if (s->fd >= 0 &&
        pread(s->fd, s->array, s->size, 0) != (ssize_t)s->size) {
        error_setg_errno(errp, errno, "xm-sfc: cannot read %s",
                         s->flash_file);
        return;
    }
    sysbus_init_mmio(sbd, &s->window);
}

static void xm_sfc_reset(DeviceState *dev)
{
    XmSfcState *s = XM_SFC(dev);

    s->mode = s->addr = s->cmd = s->srw = s->wrnum = 0;
    s->bsel = s->st = s->srr = s->cache_cmd = 0;
    s->sr1 = s->sr2 = 0;
}

static const Property xm_sfc_props[] = {
    DEFINE_PROP_STRING("flash-file", XmSfcState, flash_file),
    DEFINE_PROP_UINT32("size", XmSfcState, size, 8 * MiB),
    /* W25Q64CV: the first entry of the vendor driver's table. */
    DEFINE_PROP_UINT32("jedec", XmSfcState, jedec, 0xef4017),
};

static void xm_sfc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = xm_sfc_realize;
    device_class_set_legacy_reset(dc, xm_sfc_reset);
    device_class_set_props(dc, xm_sfc_props);
}

static const TypeInfo xm_sfc_info = {
    .name          = TYPE_XM_SFC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmSfcState),
    .class_init    = xm_sfc_class_init,
};

static void xm_sfc_register_types(void)
{
    type_register_static(&xm_sfc_info);
}

type_init(xm_sfc_register_types)
