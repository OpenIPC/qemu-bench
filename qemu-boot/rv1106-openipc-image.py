#!/usr/bin/env python3
"""
Build a SPI NAND image for `-M rv1106,flash-type=w25n01gv` (or rv1103)
from an OpenIPC release (openipc.rv1106-nand-lite.tgz: rootfs.ubi.rv1106,
UBI volumes kernel / rootfs / rootfs_data; the rv1103 release the same
with .rv1103).  The kernel comes from the NOR release's zboot.img.<soc>
(an Android boot image: zImage, then an RSCE resource blob holding the
devicetree); this script extracts both for -kernel/-dtb.

Layout (128 MiB, 128 KiB blocks, data only, no OOB):

    0x000000  256K  boot   (empty: QEMU boots the kernel with -kernel)
    0x040000  256K  env    U-Boot environment, 64 KiB, CRC32 first
    0x080000  512K  spare
    0x100000  rest  ubi    rootfs.ubi.<soc>

The environment matters: OpenIPC's fw_printenv has no fw_env.config and
searches the MTD partitions in order for a block whose CRC32 matches,
mapping each partition whole (MAP_POPULATE).  Without one it maps the
127 MiB UBI partition into 96 MiB of RAM and the boot thrashes forever
in S30customizer's check_mac.

Usage:
    rv1106-openipc-image.py NAND_TGZ NOR_TGZ OUTDIR
Writes OUTDIR/{nand.bin,zImage,rv1106.dtb,cmdline}.
"""

import binascii
import io
import os
import struct
import sys
import tarfile

PAGE = 2048
BLOCK = 128 * 1024
SIZE = 128 << 20
ENV_OFF, ENV_LEN = 0x40000, 0x10000
UBI_OFF = 0x100000

MTDPARTS = "spi-nand0:256k(boot),256k(env),512k(spare),-(ubi)"
CMDLINE = ("console=ttyFIQ0 mtdparts=" + MTDPARTS + " ubi.mtd=3 "
           "ubi.block=0,rootfs root=/dev/ubiblock0_1 rootfstype=squashfs")
# The kernel's own CMA (32 MiB) does not fit an RV1103's 64 MiB ("cma:
# Failed to reserve"), and the camera's buffers then fail to allocate;
# the vendor U-Boot passes Rockchip's DMA-heap reservation instead.
EXTRA_ARGS = {"rv1103": " rk_dma_heap_cma=24M"}


def member(tgz, prefix):
    """The one file whose name starts with prefix: (data, its SoC suffix)."""
    with tarfile.open(tgz) as t:
        for m in t.getmembers():
            name = os.path.basename(m.name)
            if name.startswith(prefix) and not name.endswith(".md5sum"):
                return t.extractfile(m).read(), name[len(prefix):]
    sys.exit("%s: no %s*" % (tgz, prefix))


def split_boot_img(d):
    """zImage and the devicetree from an Android boot image + RSCE."""
    if d[:8] != b"ANDROID!":
        sys.exit("zboot.img: not an Android boot image")
    ks, _, rs, _, ss, _, _, page = struct.unpack_from("<8I", d, 8)
    koff = page
    roff = koff + -(-ks // page) * page
    soff = roff + -(-rs // page) * page
    second = d[soff:soff + ss]
    i = second.find(b"\xd0\x0d\xfe\xed")
    if i < 0:
        sys.exit("zboot.img: no devicetree in the second stage")
    dtb_len = struct.unpack(">I", second[i + 4:i + 8])[0]
    return d[koff:koff + ks], second[i:i + dtb_len]


def env_block(pairs):
    data = b"".join(b"%s=%s\0" % (k.encode(), v.encode()) for k, v in pairs)
    data += b"\0"
    data = data.ljust(ENV_LEN - 4, b"\0")
    return struct.pack("<I", binascii.crc32(data) & 0xffffffff) + data


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    nand_tgz, nor_tgz, out = sys.argv[1:]
    os.makedirs(out, exist_ok=True)

    ubi, soc = member(nand_tgz, "rootfs.ubi.")
    if ubi[:4] != b"UBI#":
        sys.exit("rootfs.ubi: no UBI EC header")
    if UBI_OFF + len(ubi) > SIZE:
        sys.exit("rootfs.ubi: too big for 128 MiB")
    zimage, dtb = split_boot_img(member(nor_tgz, "zboot.img.")[0])
    cmdline = CMDLINE + EXTRA_ARGS.get(soc, "")

    img = bytearray(b"\xff" * SIZE)
    img[ENV_OFF:ENV_OFF + ENV_LEN] = env_block([
        ("bootargs", cmdline),
        ("ethaddr", "52:54:00:12:34:56"),
        ("soc", soc),
    ])
    img[UBI_OFF:UBI_OFF + len(ubi)] = ubi

    for name, blob in (("nand.bin", img), ("zImage", zimage),
                       ("rv1106.dtb", dtb), ("cmdline", cmdline.encode())):
        with open(os.path.join(out, name), "wb") as f:
            f.write(blob)
    print("%s: nand.bin (UBI %d KiB at 0x%x), zImage %d KiB, dtb %d bytes"
          % (out, len(ubi) >> 10, UBI_OFF, len(zimage) >> 10, len(dtb)))
    print("run: qemu-system-arm -M rv1106,flash-file=%s/nand.bin,"
          "flash-type=w25n01gv -kernel %s/zImage -dtb %s/rv1106.dtb "
          "-append \"$(cat %s/cmdline)\" -nic user -nographic"
          % (out, out, out, out))


if __name__ == "__main__":
    main()
