#!/usr/bin/env python3
"""
Build a flash image for `-M rv1106` (or rv1103) from one OpenIPC release
package, in its layout since OpenIPC/firmware#2954, booted with -kernel:

  openipc.<soc>-nand-lite.tgz       SPI NAND (-M ...,flash-type=w25n01gv)
      rootfs.ubi.<soc>: UBI volumes rootfs (UBIFS, the kernel inside as
      /boot/fitImage) and rootfs_data; fitImage.<soc>, the same FIT
      (zImage and <soc>-nand.dtb), gives -kernel and -dtb.

  openipc.<soc>-nor-lite-split.tgz  16 MiB SPI NOR
      firmware.bin.<soc>: a multi-file uImage (zImage, <soc>-nor.dtb),
      then the squashfs at the next 64 KiB.  It goes at 0x50000, and the
      kernel divides it into kernel, rootfs and rootfs_data at boot
      (OpenIPC/linux split_layout.c), ignoring mtdparts= and root=.

NAND layout (128 MiB, 128 KiB blocks, data only, no OOB):

    0x000000  256K  boot   (empty: QEMU boots the kernel with -kernel)
    0x040000  256K  env    U-Boot environment, 64 KiB, CRC32 first
    0x080000  512K  spare
    0x100000  rest  ubi    rootfs.ubi.<soc>

NOR layout (16 MiB):

    0x000000  256K  boot   (empty)
    0x040000   64K  env    U-Boot environment, 64 KiB, CRC32 first
    0x050000  rest  firmware.bin.<soc>, then rootfs_data

The environment matters: OpenIPC's fw_printenv has no fw_env.config and
searches the MTD partitions in order for a block whose CRC32 matches,
mapping each partition whole (MAP_POPULATE).  Without one it maps the
127 MiB UBI partition into 96 MiB of RAM and the boot thrashes forever
in S30customizer's check_mac.

Usage:
    rv1106-openipc-image.py PACKAGE_TGZ OUTDIR
Writes OUTDIR/{nand.bin or nor.bin,zImage,rv1106.dtb,cmdline}.
"""

import binascii
import os
import struct
import sys
import tarfile

ENV_OFF, ENV_LEN = 0x40000, 0x10000

NAND_SIZE = 128 << 20
UBI_OFF = 0x100000
NAND_CMDLINE = ("console=ttyFIQ0 mtdparts=spi-nand0:256k(boot),256k(env),"
                "512k(spare),-(ubi) ubi.mtd=3 root=ubi0:rootfs "
                "rootfstype=ubifs")

NOR_SIZE = 16 << 20
FW_OFF = 0x50000
# What OpenIPC's U-Boot passes; the split layout overrides both.
NOR_CMDLINE = ("console=ttyFIQ0 mtdparts=sfc_nor:256k(boot),64k(env),"
               "-(firmware) root=/dev/mtdblock3 rootfstype=squashfs")

# The kernel's own CMA (32 MiB) does not fit an RV1103's 64 MiB ("cma:
# Failed to reserve"), and the camera's buffers then fail to allocate;
# the vendor U-Boot passes Rockchip's DMA-heap reservation instead.
EXTRA_ARGS = {"rv1103": " rk_dma_heap_cma=24M"}

FDT_MAGIC = b"\xd0\x0d\xfe\xed"


def member(tgz, prefix):
    """The file whose name starts with prefix: (data, its SoC suffix), or
    (None, None)."""
    with tarfile.open(tgz) as t:
        for m in t.getmembers():
            name = os.path.basename(m.name)
            if (m.isfile() and name.startswith(prefix)
                    and not name.endswith(".md5sum")):
                return t.extractfile(m).read(), name[len(prefix):]
    return None, None


def fdt_props(d):
    """{node path: {property: value}} of a flattened devicetree."""
    if d[:4] != FDT_MAGIC:
        sys.exit("not a flattened devicetree")
    total, off_struct, off_strings = struct.unpack_from(">3I", d, 4)
    if total > len(d):
        sys.exit("devicetree truncated")
    out, path, p = {}, [], off_struct
    while p < total:
        tok, = struct.unpack_from(">I", d, p)
        p += 4
        if tok == 1:                                # FDT_BEGIN_NODE
            end = d.index(b"\0", p)
            path.append(d[p:end].decode("latin-1"))
            out.setdefault("/".join(path), {})
            p = (end + 4) & ~3
        elif tok == 2:                              # FDT_END_NODE
            path.pop()
        elif tok == 3:                              # FDT_PROP
            ln, nameoff = struct.unpack_from(">2I", d, p)
            p += 8
            s = off_strings + nameoff
            name = d[s:d.index(b"\0", s)].decode("latin-1")
            out["/".join(path)][name] = d[p:p + ln]
            p = (p + ln + 3) & ~3
        elif tok == 4:                              # FDT_NOP
            pass
        elif tok == 9:                              # FDT_END
            break
        else:
            sys.exit("devicetree: bad token %#x" % tok)
    return out


def split_fit(d):
    """zImage and the devicetree of the default configuration of a FIT."""
    n = fdt_props(d)
    conf = n.get("/configurations", {}).get("default", b"").rstrip(b"\0")
    c = n.get("/configurations/" + conf.decode("latin-1"))
    if not c:
        sys.exit("fitImage: no default configuration")
    blobs = []
    for key in ("kernel", "fdt"):
        img = n.get("/images/" + c[key].rstrip(b"\0").decode("latin-1"), {})
        if "data" not in img:
            sys.exit("fitImage: no %s data" % key)
        blobs.append(img["data"])
    return blobs


def split_uimage(d):
    """zImage and the devicetree of a multi-file uImage."""
    magic, _, _, size, _, _, _, _, _, itype, _ = \
        struct.unpack_from(">7I4B", d, 0)
    if magic != 0x27051956 or itype != 4:
        sys.exit("firmware.bin: no multi-file uImage at 0")
    sizes, p = [], 64
    while True:
        n, = struct.unpack_from(">I", d, p)
        p += 4
        if not n:
            break
        sizes.append(n)
    parts = []
    for n in sizes:
        parts.append(d[p:p + n])
        p += (n + 3) & ~3
    if p > 64 + size:
        sys.exit("firmware.bin: uImage parts overrun its size")
    dtbs = [x for x in parts if x[:4] == FDT_MAGIC]
    if not dtbs:
        sys.exit("firmware.bin: no devicetree in the uImage")
    return parts[0], dtbs[0]


def env_block(pairs):
    data = b"".join(b"%s=%s\0" % (k.encode(), v.encode()) for k, v in pairs)
    data += b"\0"
    data = data.ljust(ENV_LEN - 4, b"\0")
    return struct.pack("<I", binascii.crc32(data) & 0xffffffff) + data


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    tgz, out = sys.argv[1:]

    ubi, soc = member(tgz, "rootfs.ubi.")
    if ubi is not None:
        if ubi[:4] != b"UBI#":
            sys.exit("rootfs.ubi: no UBI EC header")
        if UBI_OFF + len(ubi) > NAND_SIZE:
            sys.exit("rootfs.ubi: too big for 128 MiB")
        fit, _ = member(tgz, "fitImage.")
        if fit is None:
            sys.exit("%s: no fitImage.* (a release before firmware#2954?)"
                     % tgz)
        zimage, dtb = split_fit(fit)
        name, size, at, blob = "nand.bin", NAND_SIZE, UBI_OFF, ubi
        cmdline = NAND_CMDLINE
    else:
        fw, soc = member(tgz, "firmware.bin.")
        if fw is None:
            sys.exit("%s: neither rootfs.ubi.* nor firmware.bin.*" % tgz)
        if FW_OFF + len(fw) > NOR_SIZE:
            sys.exit("firmware.bin: too big for 16 MiB")
        zimage, dtb = split_uimage(fw)
        name, size, at, blob = "nor.bin", NOR_SIZE, FW_OFF, fw
        cmdline = NOR_CMDLINE
    cmdline += EXTRA_ARGS.get(soc, "")

    img = bytearray(b"\xff" * size)
    img[ENV_OFF:ENV_OFF + ENV_LEN] = env_block([
        ("bootargs", cmdline),
        ("ethaddr", "52:54:00:12:34:56"),
        ("soc", soc),
    ])
    img[at:at + len(blob)] = blob

    os.makedirs(out, exist_ok=True)
    for f, data in ((name, img), ("zImage", zimage), ("rv1106.dtb", dtb),
                    ("cmdline", cmdline.encode())):
        with open(os.path.join(out, f), "wb") as fh:
            fh.write(data)
    print("%s: %s (%s %d KiB at 0x%x), zImage %d KiB, dtb %d bytes"
          % (out, name, "UBI" if ubi is not None else "firmware.bin",
             len(blob) >> 10, at, len(zimage) >> 10, len(dtb)))


if __name__ == "__main__":
    main()
