/*
 * Anyka AK39xx L2 buffer controller: the SRAM ("L2 memory") that every
 * streaming peripheral moves its data through, and the DMA engine between
 * it and DDR.  See hw/misc/ak-l2.c.
 *
 * Peripherals are not wired to a fixed buffer: the driver picks one of the
 * eight 512-byte common buffers and writes its number into the device's
 * BUFASSIGN field (l2_select_combuf()).  A peripheral model asks which
 * buffer it has with ak_l2_dev_buf() and moves data with ak_l2_push() /
 * ak_l2_pop(), one 64-byte block at a time, like the hardware's buffer
 * status counter.  It is called back through its registered pump whenever
 * the CPU or the DMA engine has filled or drained a block, so a transfer
 * stalled on a full or empty buffer can go on.
 */
#ifndef HW_MISC_AK_L2_H
#define HW_MISC_AK_L2_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AK_L2 "ak-l2"
OBJECT_DECLARE_SIMPLE_TYPE(AkL2State, AK_L2)

#define AK_L2_MEM_SIZE      0x2000
#define AK_L2_BLOCK         64
#define AK_L2_COMBUF_NUM    8
#define AK_L2_COMBUF_SIZE   512
#define AK_L2_COMBUF_BLOCKS (AK_L2_COMBUF_SIZE / AK_L2_BLOCK)
#define AK_L2_UART_TX(n)    (0x1000 + 0x100 * (n))
#define AK_L2_UART_RX(n)    (0x1080 + 0x100 * (n))
#define AK_L2_UART_RX_SIZE  128
#define AK_L2_MAX_CLIENTS   4

/* BUFASSIGN device numbers (ak_l2_inner.h l2_device_t). */
enum {
    AK_L2_DEV_SPI0_RX = 7,
    AK_L2_DEV_SPI0_TX = 8,
    AK_L2_DEV_SPI1_RX = 10,
    AK_L2_DEV_SPI1_TX = 11,
};

typedef bool (*AkL2PumpFn)(void *opaque);

typedef struct AkL2Buf {
    uint8_t status;     /* filled blocks, 0..8 */
    uint8_t dev_blk;    /* peripheral side: next block it fills or drains */
    uint8_t mem_blk;    /* DMA side: next block the engine moves */
    uint32_t dma_left;  /* blocks the running DMA still has to move */
    uint32_t dma_addr;
} AkL2Buf;

struct AkL2State {
    SysBusDevice parent_obj;

    MemoryRegion ctrl;
    MemoryRegion mem;
    MemoryRegion mem_alias[7];
    qemu_irq irq;

    uint32_t regs[0x100 / 4];
    AkL2Buf buf[AK_L2_COMBUF_NUM];
    bool irq_pending;

    AkL2PumpFn pump[AK_L2_MAX_CLIENTS];
    void *pump_opaque[AK_L2_MAX_CLIENTS];
    int num_clients;
    bool running;
    bool rerun;
};

void ak_l2_register_client(AkL2State *s, AkL2PumpFn pump, void *opaque);
/* The common buffer assigned to BUFASSIGN device dev, or -1. */
int ak_l2_dev_buf(AkL2State *s, int dev);
/*
 * Peripheral produces: false while the buffer is full.  A short block
 * (len < 64) is the end of a transfer and still counts as one block.
 */
bool ak_l2_push(AkL2State *s, int buf, const uint8_t *data, int len);
/* Peripheral consumes one block of up to len bytes; false while empty. */
bool ak_l2_pop(AkL2State *s, int buf, uint8_t *data, int len);
/* Restart everything that may have been waiting on a buffer. */
void ak_l2_kick(AkL2State *s);
/* The SRAM, for peripherals with fixed rings in it (the UARTs). */
uint8_t *ak_l2_mem(AkL2State *s);

#endif /* HW_MISC_AK_L2_H */
