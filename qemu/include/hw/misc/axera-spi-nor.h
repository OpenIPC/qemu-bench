#ifndef HW_MISC_AXERA_SPI_NOR_H
#define HW_MISC_AXERA_SPI_NOR_H

#define TYPE_AXERA_SPI_NOR "axera-spi-nor"

/*
 * Named GPIO out: the DW SSI's dma_rx_req handshake, high while
 * DMACR.RDMAE is set and the RX FIFO holds data. Wired to AX_DMAPER's
 * request input 1, the id both U-Boot and the kernel route the SPI
 * RX channel to (reqmux field 0x01, dts "dmas = <&dma_per 1 ...>").
 */
#define AXERA_SPI_NOR_DMA_RX_REQ "dma-rx-req"

#endif /* HW_MISC_AXERA_SPI_NOR_H */
