#!/usr/bin/env python3
"""Set U-Boot environment variables in a SigmaStar board's SPI NOR image.

    sstar-env.py nvr.bin xmuart=0 [name=value ...] [name= ...]

Edits the image in place (keep the original): the environment block at
--offset (default 0x40000, 64 KiB, as on infinity2m boards with the
vendor's CONFIG_MS_SAVE_ENV_IN_ISP_FLASH) is a CRC32 followed by
NUL-separated name=value pairs.  "name=" deletes a variable.  U-Boot takes a
valid block instead of its defaults, not on top of them, so a blank or
corrupt block is first seeded with U-Boot's built-in default environment,
found in the U-Boot image inside the flash (a uImage, LZMA/XZ/gzip
compressed or not).

On Xiongmai NVRs, xmuart=0 puts the kernel console on UART0
(docs/sigmastar.md).
"""
import argparse
import gzip
import lzma
import struct
import sys
import zlib

UIMAGE_MAGIC = b"\x27\x05\x19\x56"


def uboot_default_env(img):
    """The default environment compiled into the first U-Boot found."""
    pos = 0
    while True:
        pos = img.find(UIMAGE_MAGIC, pos)
        if pos < 0 or pos > 4 << 20:
            return None
        size = struct.unpack(">I", img[pos + 12:pos + 16])[0]
        comp = img[pos + 31]
        data = img[pos + 64:pos + 64 + size]
        # MStar images tag the payload: "MVX1##...XVM", 32 bytes
        if data[:3] == b"MVX" and data[29:32] == b"XVM":
            data = data[32:]
        try:
            if comp == 1:
                data = gzip.decompress(data)
            elif comp in (3, 5):                # LZMA, LZO-less XZ
                data = lzma.LZMADecompressor().decompress(data)
            elif data[:6] == b"\xfd7zXZ\x00":     # XZ under any type
                data = lzma.LZMADecompressor().decompress(data)
        except (OSError, lzma.LZMAError):
            pos += 4
            continue
        env = find_env(data)
        if env is not None:
            return env
        pos += 4


def find_env(data):
    """default_environment[]: name=value strings up to an empty one.  It
    need not follow a NUL; env_default.h puts bootargs first when the board
    has any, so start there, else at the text run holding bootcmd (or,
    on a board with no default bootcmd such as the MStar I3 SDK's, at
    bootdelay, which env_default.h puts first then)."""
    i = data.find(b"\0bootcmd=")
    if i < 0:
        start = data.find(b"bootdelay=")
        if start < 0:
            return None
        return parse_env(data, start)
    start = i
    while start > 0 and (32 <= data[start - 1] < 127 or
                         (data[start - 1] == 0 and data[start - 2] != 0)):
        start -= 1
    j = data.rfind(b"bootargs=", start, i)
    if j >= 0:
        start = j
    else:
        start = data.find(b"\0", start) + 1 if data[start] else start + 1
    return parse_env(data, start, i)


def parse_env(data, start, i=None):
    end = data.find(b"\0\0", start if i is None else i)
    env = {}
    for kv in data[start:end].split(b"\0"):
        if b"=" not in kv:
            return None
        k, v = kv.split(b"=", 1)
        env[k] = v
    return env


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("image")
    ap.add_argument("vars", nargs="*", metavar="name=value")
    ap.add_argument("--offset", type=lambda x: int(x, 0), default=0x40000)
    ap.add_argument("--size", type=lambda x: int(x, 0), default=0x10000)
    ap.add_argument("--print", action="store_true", dest="show",
                    help="print the environment")
    a = ap.parse_args()

    with open(a.image, "r+b") as f:
        img = f.read()
        f.seek(a.offset)
        blk = f.read(a.size)
        if len(blk) != a.size:
            sys.exit("image too small for the environment block")
        crc, data = struct.unpack("<I", blk[:4])[0], blk[4:]
        env = {}
        if zlib.crc32(data) & 0xFFFFFFFF == crc:
            for kv in data.split(b"\0\0", 1)[0].split(b"\0"):
                if b"=" in kv:
                    k, v = kv.split(b"=", 1)
                    env[k] = v
        elif a.vars:
            env = uboot_default_env(img)
            if env is None:
                sys.exit("no valid environment at 0x%x and no U-Boot default "
                         "environment found to start from" % a.offset)
            print("no valid environment at 0x%x: starting from U-Boot's "
                  "defaults (%d variables)" % (a.offset, len(env)),
                  file=sys.stderr)
        for kv in a.vars:
            if "=" not in kv:
                sys.exit("expected name=value, got %r" % kv)
            k, v = kv.encode().split(b"=", 1)
            if v:
                env[k] = v
            else:
                env.pop(k, None)
        if a.show:
            for k in sorted(env):
                print((k + b"=" + env[k]).decode(errors="replace"))
        if not a.vars:
            return
        body = b"".join(k + b"=" + env[k] + b"\0" for k in sorted(env)) + b"\0"
        if len(body) > a.size - 4:
            sys.exit("environment does not fit in 0x%x bytes" % a.size)
        body = body.ljust(a.size - 4, b"\0")
        f.seek(a.offset)
        f.write(struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF) + body)


if __name__ == "__main__":
    main()
