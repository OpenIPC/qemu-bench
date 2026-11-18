#!/usr/bin/env python3
"""Lay published SigmaStar NAND images out as the raw flash qemu-bench boots.

    sstar-nand-image.py out.nand u-boot-ssc337de-nand.bin [rootfs.ubi]

The output has page + spare for every page, erased but for the pieces, which
is what the sstar-fsp NAND chip stores: the U-Boot image (CIS, IPL, IPL_CUST
and U-Boot) from offset 0 and the UBI image at the start of the ubi partition,
"1024k(boot0),1024k(boot1),256k(env),-(ubi)" in the images' U-Boot.  Blocks
listed with --bad get a factory bad-block mark, spare byte 0 of their first
page, cleared.  With --dump the first image is a whole chip read back without
its spares, which keeps its own layout.
"""
import argparse

UBI_OFFSET = 0x240000


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out")
    ap.add_argument("boot")
    ap.add_argument("ubi", nargs="?")
    ap.add_argument("--blocks", type=int, default=1024)
    ap.add_argument("--page", type=int, default=2048)
    ap.add_argument("--spare", type=int, default=64)
    ap.add_argument("--block-pages", type=int, default=64)
    ap.add_argument("--bad", type=int, action="append", default=[])
    ap.add_argument("--dump", action="store_true")
    args = ap.parse_args()

    PAGE, SPARE = args.page, args.spare
    BLOCK = PAGE * args.block_pages
    data = bytearray(b"\xff" * (args.blocks * BLOCK))
    boot = open(args.boot, "rb").read()
    if args.dump:
        if len(boot) != len(data):
            ap.error("a dump is the whole chip, %d bytes" % len(data))
    elif args.ubi and len(boot) > UBI_OFFSET:
        ap.error("the boot image runs into the ubi partition")
    if len(boot) > len(data):
        ap.error("the boot image does not fit")
    data[: len(boot)] = boot
    if args.ubi:
        ubi = open(args.ubi, "rb").read()
        if UBI_OFFSET + len(ubi) > len(data):
            ap.error("the UBI image does not fit")
        data[UBI_OFFSET : UBI_OFFSET + len(ubi)] = ubi

    # a skipped block's bytes are lost, and everything after it moves
    used = [(0, len(boot))]
    if args.ubi:
        used.append((UBI_OFFSET, UBI_OFFSET + len(ubi)))
    for block in args.bad:
        for start, end in used:
            if block * BLOCK < end and (block + 1) * BLOCK > start:
                ap.error("bad block %d holds part of an image" % block)

    with open(args.out, "wb") as out:
        for off in range(0, len(data), PAGE):
            spare = bytearray(b"\xff" * SPARE)
            if off % BLOCK == 0 and off // BLOCK in args.bad:
                spare[0] = 0
            out.write(data[off : off + PAGE])
            out.write(spare)


if __name__ == "__main__":
    main()
