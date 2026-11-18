#!/usr/bin/env python3
"""Assemble a flash image for Ifliegen (ssc034a, ssc034b): SPI NOR or NAND.

    sstar-ifliegen-image.py BOARD UBOOT KERNEL ROOTFS OUT
                            [--package qfn128|qfn88] [--nand [--part N]]

BOARD is the SDK's project/board/ifliegen/boot; UBOOT the U-Boot it builds
(u-boot.xz.img.bin from ifliegen_ssc034a_s01a_defconfig, or
u-boot_spinand.xz.img.bin from ..._spinand_defconfig); KERNEL a uImage that
fits the KERNEL partition (the SDK's uImage.xz); ROOTFS a squashfs, for
NAND inside a UBI image (a static volume "rootfs", which the kernel opens
with ubi.block=1,0).  The SDK ships no prebuilt images for this die, so this
does what its image.mk does with nor.squashfs.tfa.partition.config or
spinand.squashfs.tfa.partition.config, and writes the environment its
set_config would save, for the package's DRAM: QFN128 128 MiB (ssc034a),
QFN88 64 MiB (ssc034b).

NOR, 16 MiB:
  IPL | CIS | IPL_CUST | TF_A | UBOOT | ENV (the last page of the 64 KiB-
  aligned boot area) | KERNEL | rootfs | MISC | miservice | customer
The CIS is flash.nri, the PNI, then flash_list.nri with the CRC32 at 0x104
that nrigenerator adds.

SPI NAND, 2048+64 a page, 64 pages a block, 1024 blocks (a W25N01GV):
  CIS (10 blocks, a copy in each) | BOOT: IPL x3, IPL_CUST x3, TF_A, UBOOT
  | BOOT_BAK | ENV | KERNEL | KERNEL_BACKUP | rootfs | MISC | ubia
The CIS is flash.sni and flash_list.sni as snigenerator leaves them (the
boot blocks, BL0PBA and BL1PBA, in the SNIB and every SNIF; their CRCs),
the PNI between.  Its SNIF is part N of the flash list, as U-Boot's probe
writes it on a board: the IPL takes its timeouts from that record, and the
SDK's generic one has none.  N=57 is a W25N01GV, which the machine then
carries (its ID from the SNIF).

The PNI is pnigenerator's form (the same bytes as the Iford SDK's prebuilt
cis.bin): SSTARSEMICIS0001, a byte sum from the size field over the
records, then 0x20-byte records name, trunk, backup trunk, active, group,
offset, size.
"""
import argparse
import struct
import zlib

NOR_SIZE = 0x1000000
PG, BLK = 0x1000, 0x10000
NOR_SYS = (("KERNEL", 0x310000), ("rootfs", 0x220000), ("MISC", 0xD0000),
           ("miservice", 0x560000), ("customer", 0x410000))
NAND_PG, NAND_SPARE, NAND_BP, NAND_BC = 2048, 64, 64, 1024
NAND_BLK = NAND_PG * NAND_BP
NAND_SYS = (("ENV", 0x40000), ("KERNEL", 0x500000), ("KERNEL_BACKUP", 0x500000),
            ("rootfs", 0x600000), ("MISC", 0x100000))
ENV_SIZE = 0x1000           # CONFIG_ENV_SIZE
PACKAGE = {
    # NOR IPL variant, LX_MEM, MMA heap
    "qfn128": ("ifliegen_nor.QFN128=y", 0x7FE0000, 0x5000000),
    "qfn88": ("ifliegen_nor.QFN88=y", 0x3FE0000, 0x2000000),
}


def align(n, a):
    return (n + a - 1) // a * a


def read(path):
    with open(path, "rb") as f:
        return f.read()


def nri_crc(d):
    out = bytearray(d)
    for i in range(0, len(out), 1024):
        if out[i + 0x100:i + 0x104] == b"NRIF":
            struct.pack_into("<I", out, i + 0x104,
                             zlib.crc32(bytes(out[i + 0x108:i + 0x400])))
    return bytes(out)


def bdma_crc32(buf, crc=0xFFFFFFFF, poly=0xEDB88320):
    """snigenerator's FLASH_IMPL_BDMA_SW_CRC32, for the SNIB."""
    for b in buf:
        for i in range(8):
            if ((crc >> 31) ^ (b >> i)) & 1:
                crc = (((crc << 1) & 0xFFFFFFFF) >> 1) ^ (poly >> 1)
                crc = ((crc << 1) | 1) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc


def snigen(data, bl0, bl1, page=0, spare=0, blpinb=0, bakcnt=3, bakofs=1):
    """What snigenerator -a -b -c -d -e [-p -s] does to each 1 KiB record."""
    boot = bytes([bl0, bl1, blpinb, bakcnt, bakofs])
    out = bytearray()
    for i in range(0, len(data) - 1023, 1024):
        r = bytearray(data[i:i + 1024])
        if r[0x100:0x104] != b"SNIF":
            break
        snib = r[0:4] == b"SNIB"
        if snib:
            r[0x3F:0x44] = boot
            if page:
                struct.pack_into("<H", r, 0x36, page)
            if spare:
                struct.pack_into("<H", r, 0x34, spare)
        r[0x127:0x12C] = boot
        if page:
            struct.pack_into("<H", r, 0x11A, page)
        if spare:
            struct.pack_into("<H", r, 0x118, spare)
        struct.pack_into("<I", r, 0x104, zlib.crc32(bytes(r[0x108:0x400])))
        if snib:
            struct.pack_into("<I", r, 4, bdma_crc32(bytes(r[8:0x80])))
        out += r
    return bytes(out)


def pni(parts):
    recs = b"".join(name.encode().ljust(16, b"\0") + bytes(flags) +
                    struct.pack("<III", off, size, 0)
                    for name, off, size, flags in parts)
    body = struct.pack("<I", len(recs)) + recs
    return b"SSTARSEMICIS0001" + struct.pack("<I", sum(body)) + body


def env_block(pairs):
    data = b"".join(("%s=%s" % kv).encode() + b"\0" for kv in pairs) + b"\0"
    data = data.ljust(ENV_SIZE - 4, b"\0")
    return zlib.crc32(data).to_bytes(4, "little") + data


def place(img, off, size, data, name):
    if len(data) > size:
        raise SystemExit("%s: 0x%x bytes do not fit in 0x%x"
                         % (name, len(data), size))
    img[off:off + len(data)] = data


def nor(a, lx_mem, mma):
    b = a.board
    ipl_name = PACKAGE[a.package][0]
    boot = (("IPL", read("%s/nor/ipl/IPL.%s.bin" % (b, ipl_name))),
            ("CIS", None),
            ("IPL_CUST", read("%s/nor/ipl/IPL_CUST.%s.bin" % (b, ipl_name))),
            ("TF_A", read("%s/tf_a/u-bl32.bin" % b)),
            ("UBOOT", read(a.uboot)))
    flash = read("%s/nor/partition/flash.nri" % b)
    flist = nri_crc(read("%s/nor/partition/flash_list.nri" % b))
    cis = bytearray(b"\xff" * max(align(2 * PG + len(flist), PG), 0xA000))

    parts, blobs, off = [], {}, 0
    for name, data in boot:
        size = len(cis) if data is None else align(len(data), PG)
        parts.append((name, off, size, (0, 0, 1, 1)))
        blobs[name] = data
        off += size
    env_off = align(off + PG, BLK) - PG
    parts.append(("ENV", env_off, PG, (0, 0, 1, 0)))
    off = env_off + PG
    for name, size in NOR_SYS:
        parts.append((name, off, size, (0, 1, 1, 0)))
        off += size
    assert off <= NOR_SIZE

    cis[0:len(flash)] = flash
    table = pni(parts)
    cis[PG:PG + len(table)] = table
    cis[2 * PG:2 * PG + len(flist)] = flist
    blobs["CIS"] = bytes(cis)
    blobs["KERNEL"], blobs["rootfs"] = read(a.kernel), read(a.rootfs)

    img = bytearray(b"\xff" * NOR_SIZE)
    for name, off, size, _ in parts:
        if blobs.get(name) is not None:
            place(img, off, size, blobs[name], name)

    mtdparts = "nor0:" + ",".join(
        ["0x%X(BOOT)" % env_off, "0x%X(ENV)" % PG] +
        ["0x%X(%s)" % (size, name) for name, size in NOR_SYS])
    img[env_off:env_off + ENV_SIZE] = env_block((
        ("bootargs", "console=ttyS0,115200n8r root=/dev/mtdblock3 ro "
         "init=/linuxrc LX_MEM=0x%X mma_heap=mma_heap_name0,miu=0,sz=0x%X "
         "mma_memblock_remove=1 cma=2M mtdparts=%s" % (lx_mem, mma, mtdparts)),
        ("bootcmd", "dcache on; sf probe 0; loados nor 0x23000000 KERNEL "
         "by_header; bootm 0x23000000"),
        ("bootdelay", "0"),
        ("mtdids", "nor0=nor0"),
        ("mtdparts", "mtdparts=" + mtdparts),
    ))
    with open(a.out, "wb") as f:
        f.write(img)


def nand(a, lx_mem, mma):
    b = a.board
    boot = (("IPL", 3 * NAND_BLK, read("%s/spinand/ipl/IPL.ifliegen_nand.bin"
                                       % b), 3),
            ("IPL_CUST", 3 * NAND_BLK,
             read("%s/spinand/ipl/IPL_CUST.ifliegen_nand.bin" % b), 3),
            ("TF_A", None, read("%s/tf_a/u-bl32.bin" % b), 1),
            ("UBOOT", 0x80000 + 4 * NAND_BLK, read(a.uboot), 1))
    boot = [(n, s or align(len(d), NAND_BLK), d, c) for n, s, d, c in boot]
    cis_size = 10 * NAND_BLK
    boot_size = sum(s for _, s, _, _ in boot)
    bl0, bl1 = cis_size // NAND_BLK, (cis_size + boot_size) // NAND_BLK

    img = bytearray(b"\xff" * (NAND_BC * NAND_BLK))
    parts = [("CIS", 0, cis_size, (0, 1, 1, 0))]
    for copy, base in enumerate((cis_size, cis_size + boot_size)):
        off = base
        for name, size, data, copies in boot:
            parts.append((name, off, size,
                          (0, 1, 1, 1) if copy == 0 else (1, 0, 0, 2)))
            for c in range(copies):
                place(img, off + c * (size // copies), size // copies, data,
                      name)
            off += size
    where = {}
    for name, size in NAND_SYS:
        backup = name.endswith("_BACKUP")
        parts.append((name.replace("_BACKUP", ""), off, size,
                      (1, 0, 0, 0) if backup else (0, 1, 1, 0)))
        where[name] = (off, size)
        off += size
    parts.append(("ubia", off, NAND_BC * NAND_BLK - off, (0, 1, 1, 0)))

    sni = bytearray(snigen(read("%s/spinand/partition/flash.sni" % b),
                           bl0, bl1, page=NAND_PG, spare=NAND_SPARE))
    flist = snigen(read("%s/spinand/partition/flash_list.sni" % b), bl0, bl1)
    if not 0 <= a.part < len(flist) // 1024:
        raise SystemExit("--part %d: the flash list has %d records"
                         % (a.part, len(flist) // 1024))
    sni[0x100:0x400] = flist[a.part * 1024 + 0x100:a.part * 1024 + 0x400]
    cis = bytearray(b"\xff" * (2 * NAND_PG))
    cis[0:len(sni)] = sni
    table = pni(parts)
    cis[NAND_PG:NAND_PG + len(table)] = table
    cis = bytes(cis) + flist
    for blk in range(10):
        img[blk * NAND_BLK:blk * NAND_BLK + len(cis)] = cis
    kernel = read(a.kernel)
    # bootcmd falls back to KERNEL_BACKUP: the same kernel
    place(img, *where["KERNEL"], kernel, "KERNEL")
    place(img, *where["KERNEL_BACKUP"], kernel, "KERNEL_BACKUP")
    place(img, *where["rootfs"], read(a.rootfs), "rootfs")

    mtdparts = "nand0:" + ",".join(
        ["0x%X(CIS)" % cis_size, "0x%X(BOOT)" % boot_size,
         "0x%X(BOOT_BAK)" % boot_size] +
        ["0x%X(%s)" % (size, name) for name, size in NAND_SYS] + ["-(ubia)"])
    env = env_block((
        ("bootargs", "console=ttyS0,115200n8r ubi.mtd=ubia,%d "
         "ubi.mtd=rootfs,%d ubi.block=1,0 root=/dev/ubiblock1_0 "
         "rootfstype=squashfs ro init=/linuxrc LX_MEM=0x%X "
         "mma_heap=mma_heap_name0,miu=0,sz=0x%X mma_memblock_remove=1 cma=2M "
         "mtdparts=%s" % (NAND_PG, NAND_PG, lx_mem, mma, mtdparts)),
        ("bootcmd", "dcache on; loados nand 0x23000000 KERNEL by_header; "
         "bootm 0x23000000; dcache on; loados nand 0x23000000 KERNEL_BACKUP "
         "by_header; bootm 0x23000000"),
        ("bootdelay", "0"),
        ("mtdids", "nand0=nand0"),
        ("mtdparts", "mtdparts=" + mtdparts),
    ))
    place(img, where["ENV"][0], where["ENV"][1], env, "ENV")
    with open(a.out, "wb") as f:
        for i in range(0, len(img), NAND_PG):
            f.write(img[i:i + NAND_PG])
            f.write(b"\xff" * NAND_SPARE)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("board")
    ap.add_argument("uboot")
    ap.add_argument("kernel")
    ap.add_argument("rootfs")
    ap.add_argument("out")
    ap.add_argument("--package", choices=PACKAGE, default="qfn128")
    ap.add_argument("--nand", action="store_true")
    ap.add_argument("--part", type=int, default=57)
    a = ap.parse_args()
    _, lx_mem, mma = PACKAGE[a.package]
    (nand if a.nand else nor)(a, lx_mem, mma)


if __name__ == "__main__":
    main()
