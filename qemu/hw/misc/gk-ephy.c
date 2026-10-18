/*
 * Goke GK710x internal Ethernet PHY.
 *
 * The 10/100 PHY inside the SoC is not on the GMAC's MDIO bus: its MII
 * registers are memory mapped, register n as a 16-bit value in the 32-bit
 * word at +4 * n (gk_eth_v1_00.c, gkhw_mdio_read with phytype=0, the
 * U-Boot default).  Past them sit the analog and mux settings the drivers
 * program (EPHY_SPEED +0x58, MII/RMII mux +0xC8, +0xE0, +0x540, +0x588 in
 * mach-gk710x/include/mach/eth.h), kept as plain storage.
 *
 * The PHY reports a link that has always negotiated 100 Mbit/s full duplex,
 * like the PHY of xm-dwmac, whose MAC and DMA serve the GMAC here.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GK_EPHY "gk-ephy"
OBJECT_DECLARE_SIMPLE_TYPE(GkEphyState, GK_EPHY)

#define GK_EPHY_SIZE    0x1000
#define MII_REGS        32

#define BMCR_RESET      0x8000
#define BMCR_ANRESTART  0x0200

struct GkEphyState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[GK_EPHY_SIZE / 4];
};

static uint64_t gk_ephy_read(void *opaque, hwaddr offset, unsigned size)
{
    GkEphyState *s = opaque;
    const unsigned n = offset / 4;

    switch (offset < MII_REGS * 4 ? n : MII_REGS) {
    case 1:     /* BMSR: 100/10 FD/HD, AN able and complete, link up */
        return 0x7809 | 0x0020 | 0x0004;
    case 2:     /* PHYID1 / PHYID2: Goke's OUI-less ID, non-zero for phylib */
        return 0x0007;
    case 3:
        return 0xC0F1;
    case 5:     /* ANLPAR: the partner advertises the same */
        return 0x45E1;
    default:
        return s->regs[n];
    }
}

static void gk_ephy_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    GkEphyState *s = opaque;

    if (offset == 0) {
        /* Reset and autoneg restart complete at once. */
        val &= ~(BMCR_RESET | BMCR_ANRESTART);
    }
    s->regs[offset / 4] = val;
}

static const MemoryRegionOps gk_ephy_ops = {
    .read = gk_ephy_read,
    .write = gk_ephy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void gk_ephy_reset(DeviceState *dev)
{
    GkEphyState *s = GK_EPHY(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = 0x3100;    /* BMCR: AN enabled, 100 Mbit/s, full duplex */
    s->regs[4] = 0x01E1;    /* ANAR: 100/10 FD/HD, 802.3 */
}

static void gk_ephy_init(Object *obj)
{
    GkEphyState *s = GK_EPHY(obj);

    memory_region_init_io(&s->iomem, obj, &gk_ephy_ops, s, TYPE_GK_EPHY,
                          GK_EPHY_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_gk_ephy = {
    .name = TYPE_GK_EPHY,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GkEphyState, GK_EPHY_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void gk_ephy_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, gk_ephy_reset);
    dc->vmsd = &vmstate_gk_ephy;
}

static const TypeInfo gk_ephy_info = {
    .name          = TYPE_GK_EPHY,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GkEphyState),
    .instance_init = gk_ephy_init,
    .class_init    = gk_ephy_class_init,
};

static void gk_ephy_register_types(void)
{
    type_register_static(&gk_ephy_info);
}

type_init(gk_ephy_register_types)
