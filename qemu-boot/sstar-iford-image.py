#!/usr/bin/env python3
"""Assemble a 16 MiB SPI NOR image from an Iford SDK's prebuilt NOR images.

    sstar-iford-image.py <Image/ipc_iford.nor...>/images flash.bin

The SDK ships the partitions and the scripts its U-Boot would run to write
them over TFTP.  This does what those scripts do: places the files by
partition_layout.txt and writes the environment set_config would save.
"""
import os
import sys
import zlib

SIZE = 0x1000000
ENV_OFFSET, ENV_SIZE = 0x5F000, 0x1000
MTDPARTS = ("nor0:0x5F000(BOOT),0x1000(ENV),0x300000(KERNEL),0x220000(rootfs),"
            "0xB0000(MISC),0x4D0000(miservice),0x500000(customer)")
PARTS = (
    ("boot.bin", 0),
    ("kernel", 0x60000),
    ("rootfs.sqfs", 0x360000),
    ("misc.fwfs", 0x580000),
    ("miservice.sqfs", 0x630000),
    ("customer.jffs2", 0xB00000),
)
ENV = (
    ("bootargs", "console=ttyS0,115200n8r root=/dev/mtdblock3 ro init=/linuxrc "
     "LX_MEM=0x8000000 mma_heap=mma_heap_name0,miu=0,sz=0x4000000 "
     "mma_memblock_remove=1 cma=2M mtdparts=" + MTDPARTS + " nohz=off"),
    ("bootcmd", "dcache on; sf probe 0; loados nor 0x22000000 KERNEL "
     "by_header; bootm 0x22000000;"),
    ("bootdelay", "0"),
    ("mtdids", "nor0=nor0"),
    ("mtdparts", MTDPARTS),
    ("overdrive", "3"),
)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    images, out = sys.argv[1:]
    img = bytearray(b"\xff" * SIZE)
    ends = [ENV_OFFSET] + [off for _, off in PARTS[2:]] + [SIZE]
    for (name, off), end in zip(PARTS, ends):
        with open(os.path.join(images, name), "rb") as f:
            data = f.read()
        if off + len(data) > end:
            sys.exit("%s: 0x%x bytes do not fit before 0x%x"
                     % (name, len(data), end))
        img[off:off + len(data)] = data
    env = b"".join(("%s=%s" % kv).encode() + b"\0" for kv in ENV) + b"\0"
    env = env.ljust(ENV_SIZE - 4, b"\0")
    img[ENV_OFFSET:ENV_OFFSET + ENV_SIZE] = (
        zlib.crc32(env).to_bytes(4, "little") + env)
    with open(out, "wb") as f:
        f.write(img)


if __name__ == "__main__":
    main()
