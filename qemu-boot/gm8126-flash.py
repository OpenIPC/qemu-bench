#!/usr/bin/env python3
"""Convert a GM8126 NOR image between U-Boot's view and the chip's.

The chip holds every 32-bit word byte-swapped: the loaders read it in
32-bit SPI frames.  A dump taken through U-Boot ("sf read") is the other
view; the gm8126 machine wants the chip's.  The conversion is its own
inverse; --check only says which view a file is.
"""
import argparse
import array
import sys

ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
ap.add_argument("image")
ap.add_argument("out", nargs="?")
ap.add_argument("--check", action="store_true")
args = ap.parse_args()

data = open(args.image, "rb").read()
view = {b"GM81": "uboot", b"18MG": "chip"}.get(data[:4])
if view is None:
    sys.exit("%s: no GM system header" % args.image)
if args.check:
    print(view)
    sys.exit(0)
if not args.out:
    ap.error("out is needed")
if len(data) % 4:
    sys.exit("%s: not a whole number of words" % args.image)
words = array.array("I", data)
assert words.itemsize == 4
words.byteswap()
with open(args.out, "wb") as f:
    f.write(words.tobytes())
print("%s: %s view -> %s view" % (args.image, view,
                                  "chip" if view == "uboot" else "uboot"))
