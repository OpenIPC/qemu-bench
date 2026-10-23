#!/usr/bin/env python3
"""Lay Xiongmai upgrade images (*.img) out as a whole SPI NOR image.

Xiongmai firmware upgrades carry one legacy uImage per flash region
(loader.bin.img, u-boot.bin.img, u-boot.env.img, romfs-x.squashfs.img, ...).
Their headers are not load addresses: ih_load is the region's start in the
flash and ih_ep its end, and the burner writes the payload (without the
64-byte header) at ih_load.  Raw files can be placed too, as FILE@OFFSET,
for example an OpenIPC kernel and rootfs instead of Xiongmai's romfs:

    xmimg2flash.py -o nor.bin --size 8M u/loader.bin.img u/u-boot.bin.img \\
        u/u-boot.env.img uImage.nt98562@0x50000 rootfs.squashfs.nt98562@0x250000

Later arguments overwrite earlier ones where they overlap.
"""

import argparse
import struct
import sys

UIMAGE_MAGIC = 0x27051956


def size_arg(s):
    s = s.strip().upper()
    mult = 1
    if s.endswith("M"):
        mult, s = 1 << 20, s[:-1]
    elif s.endswith("K"):
        mult, s = 1 << 10, s[:-1]
    return int(s, 0) * mult


def piece(arg):
    """Return (offset, data, description) for one argument."""
    path, _, off = arg.rpartition("@")
    if path and off:
        with open(path, "rb") as f:
            return size_arg(off), f.read(), f"{path} (raw)"
    with open(arg, "rb") as f:
        blob = f.read()
    if len(blob) < 64 or struct.unpack(">I", blob[:4])[0] != UIMAGE_MAGIC:
        sys.exit(f"{arg}: not a uImage; give raw files as FILE@OFFSET")
    size, load, ep = struct.unpack(">III", blob[12:24])
    name = blob[32:64].split(b"\0")[0].decode(errors="replace")
    if len(blob) < 64 + size:
        sys.exit(f"{arg}: truncated: the header says {size:#x} bytes, "
                 f"the file holds {len(blob) - 64:#x}")
    data = blob[64:64 + size]
    if ep and ep > load and len(data) > ep - load:
        sys.exit(f"{arg}: {len(data):#x} bytes do not fit "
                 f"{load:#x}..{ep:#x}")
    return load, data, f"{arg} ('{name}', region {load:#x}..{ep:#x})"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--size", type=size_arg, default=8 << 20,
                    help="flash size (default 8M)")
    ap.add_argument("images", nargs="+")
    args = ap.parse_args()

    flash = bytearray(b"\xff" * args.size)
    for arg in args.images:
        off, data, desc = piece(arg)
        if off + len(data) > args.size:
            sys.exit(f"{desc}: ends at {off + len(data):#x}, past the "
                     f"{args.size:#x}-byte flash")
        flash[off:off + len(data)] = data
        print(f"{off:#09x} {len(data):#09x}  {desc}")
    with open(args.output, "wb") as f:
        f.write(flash)


if __name__ == "__main__":
    main()
