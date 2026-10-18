/*
 * Goke GK710x serial-flash controller ("sflash") with a SPI-NOR behind it.
 *
 * A register-driven SPI master with no memory-mapped window
 * (arch/arm/mach-gk710x/include/mach/flash.h, U-Boot include/gk7101_sflash.h):
 *
 *   0x00 DATA      each access moves up to four bytes of the data phase
 *   0x04 COMMAND   one SPI transaction's shape (below)
 *   0x08 CE        chip enable / WP / HOLD lines, channel select (bit 6)
 *   0x0C SPEED     SCLK divider
 *   0x10 PARA_XIP
 *
 * COMMAND: [7:0] opcode, [8] send opcode, [11:9] address bytes, [14:12]
 * dummy bytes, [16:15] direction (0 read, 1 write, 3 none), [18:17],
 * [20:19], [22:21] lane widths of opcode / address / data, [27:23] data
 * bytes per DATA access (31 = four), [29:28] hold time.
 *
 * Writing COMMAND starts the transaction.  With address bytes, the first
 * DATA write is the address; a no-data command (erase) runs then, while
 * reads and programs stream through DATA from there on.  Without address
 * bytes a no-data command (WREN) runs at once and a read (RDSR, RDID)
 * answers every DATA read.
 *
 * Bytes are packed MSB first: a four-byte read returns the first byte from
 * flash in bits 31:24, and a program sends bits 31:24 first.  Software
 * moves whole words between memory and DATA, so every word of a Goke flash
 * image is stored byte-swapped -- which is why dumps of these cameras look
 * big-endian and why their boot header reads "GK7101" in a hex editor.
 *
 * Erases and programs are written back to the flash-file.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_GK_SFLASH "gk-sflash"
OBJECT_DECLARE_SIMPLE_TYPE(GkSflashState, GK_SFLASH)

#define SF_DATA         0x00
#define SF_COMMAND      0x04
#define SF_CE           0x08
#define SF_SPEED        0x0C
#define SF_PARA_XIP     0x10
#define SF_NREGS        8

#define CMD_CODE(c)     ((c) & 0xFF)
#define CMD_ADDR_NUM(c) (((c) >> 9) & 7)
#define CMD_RWN(c)      (((c) >> 15) & 3)
#define CMD_DATA_NUM(c) (((c) >> 23) & 0x1F)

#define RWN_READ        0
#define RWN_WRITE       1

#define SR1_WIP         0x01
#define SR1_WEL         0x02

struct GkSflashState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[SF_NREGS];

    char *flash_file;
    uint32_t jedec;
    int fd;
    uint8_t *array;
    uint32_t size;

    /* The transaction in progress */
    uint32_t cmd;
    bool need_addr;
    uint32_t addr;
    uint8_t sr[3];
    uint8_t ear;        /* extended address register: 16 MiB bank */
};

/*
 * The image is the flash: a change the guest sees must be on disk too.  A
 * write that cannot be completed leaves the two apart for good, so it ends
 * the run rather than letting the guest go on as if it had worked.
 */
static void gk_sflash_persist(GkSflashState *s, uint32_t off, uint32_t len)
{
    while (s->fd >= 0 && len) {
        ssize_t n = pwrite(s->fd, s->array + off, len, off);

        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            error_report("gk-sflash: write-through to %s failed at 0x%x: %s",
                         s->flash_file, off,
                         n < 0 ? strerror(errno) : "no progress");
            exit(1);
        }
        off += n;
        len -= n;
    }
}

static unsigned gk_sflash_xfer_len(uint32_t cmd)
{
    unsigned n = CMD_DATA_NUM(cmd);

    return (n == 0x1F || n > 4) ? 4 : n;
}

static void gk_sflash_erase(GkSflashState *s, uint32_t addr, uint32_t len)
{
    if (!(s->sr[0] & SR1_WEL)) {
        qemu_log_mask(LOG_GUEST_ERROR, "gk-sflash: erase without WREN\n");
        return;
    }
    addr &= ~(len - 1);
    if (addr >= s->size) {
        return;
    }
    len = MIN(len, s->size - addr);
    memset(s->array + addr, 0xFF, len);
    gk_sflash_persist(s, addr, len);
    s->sr[0] &= ~SR1_WEL;
}

/* An opcode with no data phase, once its address (if any) is in. */
static void gk_sflash_execute(GkSflashState *s)
{
    switch (CMD_CODE(s->cmd)) {
    case 0x06:                          /* WREN */
        s->sr[0] |= SR1_WEL;
        break;
    case 0x04:                          /* WRDI */
        s->sr[0] &= ~SR1_WEL;
        break;
    case 0x20:                          /* SE, 4 KiB */
        gk_sflash_erase(s, s->addr, 4 * KiB);
        break;
    case 0x52:                          /* BE, 32 KiB */
        gk_sflash_erase(s, s->addr, 32 * KiB);
        break;
    case 0xD8:                          /* BE, 64 KiB */
        gk_sflash_erase(s, s->addr, 64 * KiB);
        break;
    case 0x60:
    case 0xC7:                          /* CE */
        if (s->sr[0] & SR1_WEL) {
            memset(s->array, 0xFF, s->size);
            gk_sflash_persist(s, 0, s->size);
            s->sr[0] &= ~SR1_WEL;
        }
        break;
    default:
        break;
    }
}

static uint32_t gk_sflash_data_read(GkSflashState *s)
{
    const unsigned n = gk_sflash_xfer_len(s->cmd);
    uint32_t val = 0;
    unsigned i;

    if (CMD_RWN(s->cmd) != RWN_READ || s->need_addr) {
        return 0;
    }
    switch (CMD_CODE(s->cmd)) {
    case 0x9F:                          /* RDID */
        /* The ID's bytes in the order they leave the chip. */
        return n ? s->jedec >> (8 * (3 - MIN(n, 3))) : 0;
    case 0x05:                          /* RDSR */
        return s->sr[0];
    case 0x35:                          /* RDSR2 */
        return s->sr[1];
    case 0x15:                          /* RDSR3 */
        return s->sr[2];
    case 0xC8:                          /* RDEAR */
        return s->ear;
    case 0x03:
    case 0x0B:
    case 0x3B:
    case 0x6B:
    case 0xBB:
    case 0xEB:                          /* reads, any lane width */
        for (i = 0; i < n; i++) {
            uint32_t a = s->addr++;

            val = (val << 8) | (a < s->size ? s->array[a] : 0xFF);
        }
        return val;
    default:
        qemu_log_mask(LOG_UNIMP, "gk-sflash: read for opcode 0x%02x\n",
                      CMD_CODE(s->cmd));
        return 0;
    }
}

static void gk_sflash_data_write(GkSflashState *s, uint32_t val)
{
    const unsigned n = gk_sflash_xfer_len(s->cmd);
    unsigned i;

    if (s->need_addr) {
        const unsigned an = CMD_ADDR_NUM(s->cmd);

        s->addr = an >= 4 ? val : val & ((1u << (8 * an)) - 1);
        if (an == 3) {
            s->addr |= (uint32_t)s->ear << 24;
        }
        s->need_addr = false;
        if (CMD_RWN(s->cmd) != RWN_READ && CMD_RWN(s->cmd) != RWN_WRITE) {
            gk_sflash_execute(s);
        }
        return;
    }
    if (CMD_RWN(s->cmd) != RWN_WRITE) {
        return;
    }
    switch (CMD_CODE(s->cmd)) {
    case 0x01:
    case 0x31:
    case 0x11:
    case 0xC5:
        /* Status and extended-address writes need WREN, and use it up. */
        if (!(s->sr[0] & SR1_WEL)) {
            qemu_log_mask(LOG_GUEST_ERROR, "gk-sflash: register write "
                          "0x%02x without WREN\n", CMD_CODE(s->cmd));
            return;
        }
        switch (CMD_CODE(s->cmd)) {
        case 0x01:                      /* WRSR (+ SR2 as second byte) */
            if (n >= 2) {
                s->sr[0] = (s->sr[0] & 3) | ((val >> 8) & 0xFC);
                s->sr[1] = val & 0xFF;
            } else {
                s->sr[0] = (s->sr[0] & 3) | (val & 0xFC);
            }
            break;
        case 0x31:                      /* WRSR2 */
            s->sr[1] = val & 0xFF;
            break;
        case 0x11:                      /* WRSR3 */
            s->sr[2] = val & 0xFF;
            break;
        default:                        /* WREAR */
            s->ear = val & 0xFF;
            break;
        }
        s->sr[0] &= ~SR1_WEL;
        break;
    case 0x02:
    case 0x32:
    case 0x38: {                        /* page program, any lane width */
        uint32_t first = s->addr;

        if (!(s->sr[0] & SR1_WEL)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "gk-sflash: program without WREN\n");
            return;
        }
        for (i = 0; i < n; i++) {
            /* Within the page, as a NOR chip wraps it. */
            uint32_t a = s->addr;

            s->addr = (s->addr & ~0xFFu) | ((s->addr + 1) & 0xFF);
            if (a < s->size) {
                s->array[a] &= val >> (8 * (n - 1 - i));
            }
        }
        /* Written back once: up to the page end, and past a wrap. */
        if (first < s->size) {
            uint32_t run = MIN(n, 0x100 - (first & 0xFF));

            gk_sflash_persist(s, first, MIN(run, s->size - first));
            if (run < n) {
                gk_sflash_persist(s, first & ~0xFFu, n - run);
            }
        }
        break;
    }
    default:
        qemu_log_mask(LOG_UNIMP, "gk-sflash: write for opcode 0x%02x\n",
                      CMD_CODE(s->cmd));
        break;
    }
}

static void gk_sflash_command(GkSflashState *s, uint32_t cmd)
{
    /* A program sequence ends with the next command, which is when the
     * chip's write latch drops. */
    if (CMD_RWN(s->cmd) == RWN_WRITE &&
        (CMD_CODE(s->cmd) == 0x02 || CMD_CODE(s->cmd) == 0x32 ||
         CMD_CODE(s->cmd) == 0x38)) {
        s->sr[0] &= ~SR1_WEL;
    }
    s->cmd = cmd;
    s->need_addr = CMD_ADDR_NUM(cmd) != 0;
    if (!s->need_addr && CMD_RWN(cmd) != RWN_READ &&
        CMD_RWN(cmd) != RWN_WRITE) {
        gk_sflash_execute(s);
    }
}

static uint64_t gk_sflash_read(void *opaque, hwaddr offset, unsigned size)
{
    GkSflashState *s = opaque;

    if (offset == SF_DATA) {
        return gk_sflash_data_read(s);
    }
    if (offset == SF_COMMAND) {
        return s->cmd;
    }
    if (offset / 4 < SF_NREGS) {
        return s->regs[offset / 4];
    }
    return 0;
}

static void gk_sflash_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    GkSflashState *s = opaque;

    switch (offset) {
    case SF_DATA:
        gk_sflash_data_write(s, val);
        break;
    case SF_COMMAND:
        gk_sflash_command(s, val);
        break;
    default:
        if (offset / 4 < SF_NREGS) {
            s->regs[offset / 4] = val;
        }
        break;
    }
}

static const MemoryRegionOps gk_sflash_ops = {
    .read = gk_sflash_read,
    .write = gk_sflash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gk_sflash_reset(DeviceState *dev)
{
    GkSflashState *s = GK_SFLASH(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->cmd = 0;
    s->need_addr = false;
    s->addr = 0;
    s->sr[0] = 0;
    s->ear = 0;
}

static void gk_sflash_realize(DeviceState *dev, Error **errp)
{
    GkSflashState *s = GK_SFLASH(dev);
    off_t len = 0;

    s->fd = -1;
    if (s->flash_file && *s->flash_file) {
        s->fd = open(s->flash_file, O_RDWR);
        if (s->fd < 0) {
            error_setg_errno(errp, errno, "gk-sflash: cannot open %s",
                             s->flash_file);
            return;
        }
        /* Two machines on one image would each write back from their
         * own copy, undoing the other's erases. */
        if (qemu_lock_fd(s->fd, 0, 0, true)) {
            error_setg(errp, "gk-sflash: %s is in use by another process",
                       s->flash_file);
            close(s->fd);
            s->fd = -1;
            return;
        }
        len = lseek(s->fd, 0, SEEK_END);
        if (len <= 0 || len > 256 * MiB || (len & (64 * KiB - 1))) {
            error_setg(errp, "gk-sflash: %s is not a whole number of "
                       "64 KiB sectors up to 256 MiB", s->flash_file);
            return;
        }
        s->size = len;
    }
    if (!s->size) {
        s->size = 16 * MiB;
    }
    s->array = g_malloc(s->size);
    memset(s->array, 0xFF, s->size);
    if (s->fd >= 0 &&
        pread(s->fd, s->array, s->size, 0) != (ssize_t)s->size) {
        error_setg_errno(errp, errno, "gk-sflash: cannot read %s",
                         s->flash_file);
        return;
    }
    if (!s->jedec) {
        /* GigaDevice GD25Q-series of the flash size: what the GK7102 boards
         * carry, and in every vendor U-Boot's table. */
        s->jedec = 0xC84000 | ctz32(s->size);
    }
}

static void gk_sflash_init(Object *obj)
{
    GkSflashState *s = GK_SFLASH(obj);

    memory_region_init_io(&s->iomem, obj, &gk_sflash_ops, s, TYPE_GK_SFLASH,
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const Property gk_sflash_properties[] = {
    DEFINE_PROP_STRING("flash-file", GkSflashState, flash_file),
    DEFINE_PROP_UINT32("jedec", GkSflashState, jedec, 0),
};

static void gk_sflash_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = gk_sflash_realize;
    device_class_set_legacy_reset(dc, gk_sflash_reset);
    device_class_set_props(dc, gk_sflash_properties);
}

static const TypeInfo gk_sflash_info = {
    .name          = TYPE_GK_SFLASH,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GkSflashState),
    .instance_init = gk_sflash_init,
    .class_init    = gk_sflash_class_init,
};

static void gk_sflash_register_types(void)
{
    type_register_static(&gk_sflash_info);
}

type_init(gk_sflash_register_types)
