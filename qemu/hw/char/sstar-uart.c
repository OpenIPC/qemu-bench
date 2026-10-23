/*
 * SigmaStar / MStar UART.
 *
 * A 16550 with each register on its own 16-bit RIU slot, eight bytes
 * apart (drivers/sstar/serial/ms_uart.c: RBR/THR/DLL at 0x00, IER/DLH at
 * 0x08, IIR/FCR 0x10, LCR 0x18, MCR 0x20, LSR 0x28, MSR 0x30), followed
 * by DesignWare-style extras: USR at 0x38 (bit 0 busy, bit 1 TX FIFO not
 * full, bit 2 TX FIFO empty), TFL 0x40, RFL 0x48, RST 0x50.  The console
 * write path spins on USR "not full" (ms_uart.c line 916) and set_termios
 * on "not busy", so USR reports an idle UART with room in its FIFO.
 *
 * URDMA (the second "reg" of a "dma-enable" node, e.g. 0x1F220C00 for
 * UART0): the UART moves its bytes through two rings in DRAM instead
 * (ms_uart.c, reg_urdma).  16-bit registers on a 4-byte stride: 0x00
 * control (bit 0 reset, 1 URDMA mode, 2 TX enable, 3 RX enable, 6/7 TX/RX
 * reset, 12/13 TX/RX busy), 0x01 RX threshold, 0x02/0x03 TX ring MIU
 * address high/low, 0x04 its size in 8-byte units, 0x05 read pointer,
 * 0x06 write pointer, 0x08-0x0B the RX ring and its write pointer, 0x0D
 * interrupts.  The pointers index the last byte written and the last
 * sent, so (rptr, wptr] is pending; a TX write pointer moves, the bytes
 * go out at once and the read pointer catches up.  RX bytes land after
 * the RX write pointer, raising the timeout interrupt (0x0D bit 4, with
 * bit 7 the summary; bit 1 enables it, bit 0 clears).  The TX interrupt
 * bits moved on the Ifado generation ("urdma-tx-v2"): enable/clear/status
 * are bits 9/8/15 on infinity2m and 10/11/14 on Ifado.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "chardev/char-fe.h"
#include "hw/char/serial-mm.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_SSTAR_UART "sstar-uart"
OBJECT_DECLARE_SIMPLE_TYPE(SstarUartState, SSTAR_UART)

#define SSTAR_UART_SIZE     0x100
#define SSTAR_UART_EXTRA    0x38

#define USR_TFNF            (1u << 1)
#define USR_TFE             (1u << 2)

#define UR_SIZE             0x100
#define UR_CTRL             0x00
#define UR_TX_BASE_H        0x02
#define UR_TX_BASE_L        0x03
#define UR_TX_SIZE          0x04
#define UR_TX_RPTR          0x05
#define UR_TX_WPTR          0x06
#define UR_RX_BASE_H        0x08
#define UR_RX_BASE_L        0x09
#define UR_RX_SIZE          0x0A
#define UR_RX_WPTR          0x0B
#define UR_INTR             0x0D

#define UR_CTRL_RST         (1u << 0)
#define UR_CTRL_MODE        (1u << 1)
#define UR_CTRL_TX_EN       (1u << 2)
#define UR_CTRL_RX_EN       (1u << 3)
#define UR_CTRL_TX_RST      (1u << 6)
#define UR_CTRL_RX_RST      (1u << 7)

#define UR_INTR_RX_CLR      (1u << 0)
#define UR_INTR_RX1_EN      (1u << 1)
#define UR_INTR_RX1         (1u << 4)
#define UR_INTR_RX_MCU      (1u << 7)

#define SS_DRAM_BASE        0x20000000

struct SstarUartState {
    SysBusDevice parent_obj;

    MemoryRegion container;
    MemoryRegion extra;
    SerialMM serial;

    /* URDMA */
    bool has_urdma;
    bool urdma_tx_v2;
    MemoryRegion urdma;
    qemu_irq urdma_irq;
    uint16_t ur[UR_SIZE / 4];
    bool rx_wptr_reset;         /* the RX ring restarts at index 0 */
    IOCanReadHandler *serial_can_read;
    IOReadHandler *serial_read;
    IOEventHandler *serial_event;
    BackendChangeHandler *serial_be_change;
    void *serial_opaque;
};

static uint64_t sstar_uart_extra_read(void *opaque, hwaddr off,
                                      unsigned size)
{
    switch (off) {
    case 0x00:                  /* USR */
        return USR_TFNF | USR_TFE;
    default:                    /* TFL, RFL, RST: empty, idle */
        return 0;
    }
}

static void sstar_uart_extra_write(void *opaque, hwaddr off, uint64_t val,
                                   unsigned size)
{
}

static const MemoryRegionOps sstar_uart_extra_ops = {
    .read = sstar_uart_extra_read,
    .write = sstar_uart_extra_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---- URDMA ---------------------------------------------------------- */

static uint16_t ur_tx_en(SstarUartState *s)
{
    return s->urdma_tx_v2 ? 1u << 10 : 1u << 9;
}

static uint16_t ur_tx_clr(SstarUartState *s)
{
    return s->urdma_tx_v2 ? 1u << 11 : 1u << 8;
}

static uint16_t ur_tx_stat(SstarUartState *s)
{
    return s->urdma_tx_v2 ? 1u << 14 : 1u << 15;
}

static void ur_update_irq(SstarUartState *s)
{
    uint16_t i = s->ur[UR_INTR];

    qemu_set_irq(s->urdma_irq, (i & UR_INTR_RX_MCU) ||
                 ((i & ur_tx_stat(s)) && (i & ur_tx_en(s))));
}

static hwaddr ur_addr(uint16_t hi, uint16_t lo)
{
    return SS_DRAM_BASE + ((((uint32_t)hi & 0x0FFF) << 16) | lo);
}

static bool ur_active(SstarUartState *s, uint16_t en)
{
    return (s->ur[UR_CTRL] & (UR_CTRL_MODE | en)) == (UR_CTRL_MODE | en);
}

/* Send what lies between the read and the write pointer. */
static void ur_tx(SstarUartState *s)
{
    uint32_t size = s->ur[UR_TX_SIZE] * 8;
    uint32_t r = s->ur[UR_TX_RPTR], w = s->ur[UR_TX_WPTR];
    hwaddr base = ur_addr(s->ur[UR_TX_BASE_H], s->ur[UR_TX_BASE_L]);
    uint8_t buf[256];
    int n = 0;

    if (!ur_active(s, UR_CTRL_TX_EN) || !size || r == w) {
        return;
    }
    if (w >= size) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-uart: URDMA TX pointer %u "
                      "past the %u-byte ring\n", w, size);
        return;
    }
    while (r != w) {
        r = (r + 1) % size;
        dma_memory_read(&address_space_memory, base + r, &buf[n], 1,
                        MEMTXATTRS_UNSPECIFIED);
        if (++n == sizeof(buf)) {
            qemu_chr_fe_write_all(&s->serial.serial.chr, buf, n);
            n = 0;
        }
    }
    if (n) {
        qemu_chr_fe_write_all(&s->serial.serial.chr, buf, n);
    }
    s->ur[UR_TX_RPTR] = w;
    s->ur[UR_INTR] |= ur_tx_stat(s);
    ur_update_irq(s);
}

static int ur_can_receive(void *opaque)
{
    SstarUartState *s = opaque;

    if (ur_active(s, UR_CTRL_RX_EN)) {
        return s->ur[UR_RX_SIZE] ? 1 : 0;
    }
    return s->serial_can_read ? s->serial_can_read(s->serial_opaque) : 0;
}

static void ur_receive(void *opaque, const uint8_t *buf, int len)
{
    SstarUartState *s = opaque;
    uint32_t size = s->ur[UR_RX_SIZE] * 8;
    hwaddr base = ur_addr(s->ur[UR_RX_BASE_H], s->ur[UR_RX_BASE_L]);
    uint32_t w;
    int i;

    if (!ur_active(s, UR_CTRL_RX_EN)) {
        if (s->serial_read) {
            s->serial_read(s->serial_opaque, buf, len);
        }
        return;
    }
    if (!size) {
        return;
    }
    w = s->rx_wptr_reset ? size - 1 : s->ur[UR_RX_WPTR] % size;
    s->rx_wptr_reset = false;
    for (i = 0; i < len; i++) {
        w = (w + 1) % size;
        dma_memory_write(&address_space_memory, base + w, &buf[i], 1,
                         MEMTXATTRS_UNSPECIFIED);
    }
    s->ur[UR_RX_WPTR] = w;
    if (s->ur[UR_INTR] & UR_INTR_RX1_EN) {
        s->ur[UR_INTR] |= UR_INTR_RX1 | UR_INTR_RX_MCU;
    }
    ur_update_irq(s);
}

static void ur_event(void *opaque, QEMUChrEvent event)
{
    SstarUartState *s = opaque;

    if (s->serial_event) {
        s->serial_event(s->serial_opaque, event);
    }
}

static int ur_be_change(void *opaque)
{
    SstarUartState *s = opaque;

    return s->serial_be_change ? s->serial_be_change(s->serial_opaque) : 0;
}

static uint64_t sstar_urdma_read(void *opaque, hwaddr off, unsigned size)
{
    SstarUartState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->ur[off >> 2];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_urdma_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    SstarUartState *s = opaque;
    int reg = off >> 2;
    uint16_t v = val;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (s->ur[reg] & 0x00FF) | ((val & 0xFF) << 8)
                    : (s->ur[reg] & 0xFF00) | (val & 0xFF);
    }
    switch (reg) {
    case UR_CTRL:
        if (v & (UR_CTRL_RST | UR_CTRL_TX_RST)) {
            s->ur[UR_TX_RPTR] = s->ur[UR_TX_WPTR] = 0;
        }
        if (v & (UR_CTRL_RST | UR_CTRL_RX_RST)) {
            s->rx_wptr_reset = true;
        }
        s->ur[reg] = v & 0x0FFF;            /* never busy */
        ur_tx(s);
        break;
    case UR_TX_RPTR:
        break;                              /* the engine's */
    case UR_TX_WPTR:
        s->ur[reg] = v;
        ur_tx(s);
        break;
    case UR_RX_WPTR:
        break;
    case UR_INTR: {
        uint16_t stat = UR_INTR_RX1 | (1u << 5) | UR_INTR_RX_MCU |
                        ur_tx_stat(s);

        /* status bits are the engine's; the clear bits do not stay */
        s->ur[reg] = (v & ~(stat | UR_INTR_RX_CLR | ur_tx_clr(s))) |
                     (s->ur[reg] & stat);
        if (v & UR_INTR_RX_CLR) {
            s->ur[reg] &= ~(UR_INTR_RX1 | (1u << 5) | UR_INTR_RX_MCU);
        }
        if (v & ur_tx_clr(s)) {
            s->ur[reg] &= ~ur_tx_stat(s);
        }
        ur_update_irq(s);
        break;
    }
    default:
        if (reg < ARRAY_SIZE(s->ur)) {
            s->ur[reg] = v;
        }
        break;
    }
}

static const MemoryRegionOps sstar_urdma_ops = {
    .read = sstar_urdma_read,
    .write = sstar_urdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_uart_reset(DeviceState *dev)
{
    SstarUartState *s = SSTAR_UART(dev);

    memset(s->ur, 0, sizeof(s->ur));
    s->rx_wptr_reset = true;
    if (s->has_urdma) {
        ur_update_irq(s);
    }
}

static void sstar_uart_init(Object *obj)
{
    SstarUartState *s = SSTAR_UART(obj);

    object_initialize_child(obj, "serial", &s->serial, TYPE_SERIAL_MM);
    object_property_add_alias(obj, "chardev", OBJECT(&s->serial), "chardev");
}

static void sstar_uart_realize(DeviceState *dev, Error **errp)
{
    SstarUartState *s = SSTAR_UART(dev);
    DeviceState *sdev = DEVICE(&s->serial);

    qdev_prop_set_uint8(sdev, "regshift", 3);
    /* clk_uart 172.8 MHz; the console divisor is 172800000 / 16 / 115200. */
    qdev_prop_set_uint32(sdev, "baudbase", 172800000 / 16);
    qdev_prop_set_uint8(sdev, "endianness", DEVICE_LITTLE_ENDIAN);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->serial), errp)) {
        return;
    }
    sysbus_pass_irq(SYS_BUS_DEVICE(dev), SYS_BUS_DEVICE(&s->serial));

    memory_region_init(&s->container, OBJECT(s), "sstar-uart",
                       SSTAR_UART_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->container);
    memory_region_add_subregion(&s->container, 0,
        sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->serial), 0));
    memory_region_init_io(&s->extra, OBJECT(s), &sstar_uart_extra_ops, s,
                          "sstar-uart.extra",
                          SSTAR_UART_SIZE - SSTAR_UART_EXTRA);
    /* Over the 16550's scratch register, which shares USR's slot. */
    memory_region_add_subregion_overlap(&s->container, SSTAR_UART_EXTRA,
                                        &s->extra, 1);

    if (s->has_urdma) {
        CharFrontend *fe = &s->serial.serial.chr;

        memory_region_init_io(&s->urdma, OBJECT(s), &sstar_urdma_ops, s,
                              "sstar-uart.urdma", UR_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->urdma);
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->urdma_irq);
        /*
         * Received bytes go to the RX ring while URDMA has it, else to the
         * 16550 as before: stand between the backend and its handlers.
         */
        if (qemu_chr_fe_backend_connected(fe)) {
            s->serial_can_read = fe->chr_can_read;
            s->serial_read = fe->chr_read;
            s->serial_event = fe->chr_event;
            s->serial_be_change = fe->chr_be_change;
            s->serial_opaque = fe->opaque;
            qemu_chr_fe_set_handlers(fe, ur_can_receive, ur_receive,
                                     ur_event, ur_be_change, s, NULL, true);
        }
    }
}

static const Property sstar_uart_props[] = {
    DEFINE_PROP_BOOL("urdma", SstarUartState, has_urdma, false),
    DEFINE_PROP_BOOL("urdma-tx-v2", SstarUartState, urdma_tx_v2, false),
};

static const VMStateDescription vmstate_sstar_uart = {
    .name = TYPE_SSTAR_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(ur, SstarUartState, UR_SIZE / 4),
        VMSTATE_BOOL(rx_wptr_reset, SstarUartState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_uart_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_uart_realize;
    device_class_set_legacy_reset(dc, sstar_uart_reset);
    device_class_set_props(dc, sstar_uart_props);
    dc->vmsd = &vmstate_sstar_uart;
}

static const TypeInfo sstar_uart_info = {
    .name           = TYPE_SSTAR_UART,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarUartState),
    .instance_init  = sstar_uart_init,
    .class_init     = sstar_uart_class_init,
};

static void sstar_uart_register_types(void)
{
    type_register_static(&sstar_uart_info);
}

type_init(sstar_uart_register_types)
