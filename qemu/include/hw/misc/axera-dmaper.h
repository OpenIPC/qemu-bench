#ifndef HW_MISC_AXERA_DMAPER_H
#define HW_MISC_AXERA_DMAPER_H

#define TYPE_AXERA_DMAPER "axera-dmaper"

/*
 * Named GPIO inputs: peripheral DMA request lines, indexed by the
 * request id a devicetree "dmas" cell carries (e.g. AX620E i2s_mst
 * rx=0x0f tx=0x0e). Only request ids set in the "paced-reqs" property
 * are flow-controlled; all others keep the immediate-completion
 * behaviour the SPI flash path relies on.
 */
#define AXERA_DMAPER_REQ "dma-req"
#define AXERA_DMAPER_NR_REQ 64

#endif /* HW_MISC_AXERA_DMAPER_H */
