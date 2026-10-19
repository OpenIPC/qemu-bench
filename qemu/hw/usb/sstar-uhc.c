/*
 * SigmaStar / MStar UHC: the EHCI host controller behind the RIU.
 *
 * A Faraday FUSBH200-style EHCI (operational registers at 0x10, PORTSC
 * right after ASYNCLISTADDR at 0x30, the bus monitor control/status
 * "bmcs" at 0x40 giving the port speed in bits 10:9), one root port with
 * a built-in transaction translator.  The RIU splits each 32-bit EHCI
 * register G into two 16-bit words: bits 15:0 at RIU offset 2G and
 * 31:16 at 2G+4 (kernel drivers/usb/host/ehci.h, MP_USB_MSTAR
 * ehci_readl/ehci_writel, which always write the low half first).  One
 * RIU bank (0x200 bytes) holds the 0x100 bytes of EHCI registers.
 *
 * The EHCI itself is QEMU's fusbh200-ehci-usb, which has this layout.
 * Halves are combined here: a write commits the whole register when its
 * high half lands, so USBSTS and PORTSC see their write-1-to-clear bits
 * once.  The UTMI PHY and USBC wrapper banks beside each UHC are RIU
 * storage (sstar-riu).
 *
 * DMA: with "MIU lower bound address subtraction" on, the controller
 * takes both physical addresses (the kernel's, from 0x20000000) and MIU
 * bus addresses below that (U-Boot's ASYNCLISTADDR 0x03E97580 for a list
 * at 0x23E97580), so bus addresses under 512 MiB reach DRAM.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "system/address-spaces.h"
#include "hcd-ehci.h"
#include "qom/object.h"

#define TYPE_SSTAR_UHC "sstar-uhc"
OBJECT_DECLARE_SIMPLE_TYPE(SstarUhcState, SSTAR_UHC)

#define UHC_RIU_SIZE        0x200
#define UHC_EHCI_SIZE       0x100
#define UHC_DRAM_BASE       0x20000000

struct SstarUhcState {
    SysBusDevice parent_obj;

    MemoryRegion riu;
    FUSBH200EHCIState ehci;
    AddressSpace ehci_as;
    MemoryRegion dma_root;
    MemoryRegion dma_sys;
    MemoryRegion dma_low;
    AddressSpace dma_as;
    uint32_t pend_g;
    uint16_t pend_lo;
};

/* The EHCI's registers are a container (caps, operational, ports). */
static uint32_t uhc_ehci_read(SstarUhcState *s, hwaddr g)
{
    return address_space_ldl_le(&s->ehci_as, g, MEMTXATTRS_UNSPECIFIED, NULL);
}

static void uhc_ehci_write(SstarUhcState *s, hwaddr g, uint32_t v)
{
    address_space_stl_le(&s->ehci_as, g, v, MEMTXATTRS_UNSPECIFIED, NULL);
}

static uint64_t sstar_uhc_read(void *opaque, hwaddr off, unsigned size)
{
    SstarUhcState *s = opaque;
    uint32_t w = off >> 2, g = (w >> 1) * 4;
    uint16_t half;

    if ((off & 3) >= 2 || g >= UHC_EHCI_SIZE) {
        return 0;
    }
    half = uhc_ehci_read(s, g) >> ((w & 1) * 16);
    if (off & 1) {
        return half >> 8;
    }
    return size == 1 ? half & 0xff : half;
}

static void sstar_uhc_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    SstarUhcState *s = opaque;
    uint32_t w = off >> 2, g = (w >> 1) * 4;
    uint32_t cur;
    uint16_t half;

    if ((off & 3) >= 2 || g >= UHC_EHCI_SIZE) {
        return;
    }
    cur = uhc_ehci_read(s, g);
    half = cur >> ((w & 1) * 16);
    if (size == 1) {
        half = (off & 1) ? (half & 0x00ff) | ((val & 0xff) << 8)
                         : (half & 0xff00) | (val & 0xff);
    } else {
        half = val;
    }
    if (!(w & 1)) {
        /* The low half waits for its high half. */
        s->pend_g = g;
        s->pend_lo = half;
        return;
    }
    /*
     * A high half with no low half before it writes 0 there, rather than
     * the live value: that would acknowledge USBSTS and PORTSC's pending
     * write-1-to-clear bits.
     */
    uhc_ehci_write(s, g, (s->pend_g == g ? s->pend_lo : 0) |
                         ((uint32_t)half << 16));
    s->pend_g = UINT32_MAX;
}

static const MemoryRegionOps sstar_uhc_ops = {
    .read = sstar_uhc_read,
    .write = sstar_uhc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_uhc_reset(DeviceState *dev)
{
    SstarUhcState *s = SSTAR_UHC(dev);

    s->pend_g = UINT32_MAX;
}

static void sstar_uhc_init(Object *obj)
{
    SstarUhcState *s = SSTAR_UHC(obj);

    object_initialize_child(obj, "ehci", &s->ehci, TYPE_FUSBH200_EHCI);
}

static void sstar_uhc_realize(DeviceState *dev, Error **errp)
{
    SstarUhcState *s = SSTAR_UHC(dev);

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ehci), errp)) {
        return;
    }
    sysbus_pass_irq(SYS_BUS_DEVICE(dev), SYS_BUS_DEVICE(&s->ehci));
    /* Bus addresses under 512 MiB are DRAM; the rest are physical. */
    memory_region_init(&s->dma_root, OBJECT(s), "sstar-uhc.dma", UINT64_MAX);
    memory_region_init_alias(&s->dma_sys, OBJECT(s), "sstar-uhc.dma-sys",
                             get_system_memory(), 0, UINT64_MAX);
    memory_region_init_alias(&s->dma_low, OBJECT(s), "sstar-uhc.dma-miu",
                             get_system_memory(), UHC_DRAM_BASE,
                             UHC_DRAM_BASE);
    memory_region_add_subregion_overlap(&s->dma_root, 0, &s->dma_sys, 0);
    memory_region_add_subregion_overlap(&s->dma_root, 0, &s->dma_low, 1);
    address_space_init(&s->dma_as, &s->dma_root, "sstar-uhc.dma");
    s->ehci.parent_obj.ehci.as = &s->dma_as;

    address_space_init(&s->ehci_as,
                       sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->ehci), 0),
                       "sstar-uhc.ehci");
    memory_region_init_io(&s->riu, OBJECT(s), &sstar_uhc_ops, s, "sstar-uhc",
                          UHC_RIU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->riu);
}

static void sstar_uhc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_uhc_realize;
    device_class_set_legacy_reset(dc, sstar_uhc_reset);
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static const TypeInfo sstar_uhc_info = {
    .name           = TYPE_SSTAR_UHC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarUhcState),
    .instance_init  = sstar_uhc_init,
    .class_init     = sstar_uhc_class_init,
};

static void sstar_uhc_register_types(void)
{
    type_register_static(&sstar_uhc_info);
}

type_init(sstar_uhc_register_types)
