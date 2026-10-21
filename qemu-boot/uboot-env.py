#!/usr/bin/env python3
"""Write a U-Boot environment (CRC32 + key=value) into a flash image.

usage: uboot-env.py [--offset 0x50000] [--size 0x10000]
                    [--uboot u-boot.bin | --uboot-at 0x20000]
                    flash.bin [key=value ...]

Starts from U-Boot's built-in default environment (the block that begins
with bootargs=) and applies the given settings, as setenv + saveenv on the
board would.  U-Boot comes from --uboot, or is unpacked from the flash at
--uboot-at, where Xiongmai's NA51103 images keep it LZMA-packed
(partition_boot).  The defaults fit the Xiongmai NT98332 NOR (env at
0x50000, 64 KiB), whose env a dump may carry zeroed; for that firmware
xmuart=0 keeps the console on:

    qemu-boot/uboot-env.py nor.bin xmuart=0
"""
import argparse
import lzma
import zlib


def default_env(uboot):
    i = uboot.find(b'bootargs=')
    while i > 0 and uboot[i - 1] != 0:
        i = uboot.find(b'bootargs=', i + 1)
    if i < 0 or uboot.find(b'bootcmd=', i) < 0:
        raise SystemExit('no default environment in U-Boot')
    env = {}
    for kv in uboot[i:uboot.find(b'\0\0', i)].split(b'\0'):
        if b'=' in kv:
            k, v = kv.split(b'=', 1)
            env[k] = v
    return env


def main():
    ap = argparse.ArgumentParser()
    num = lambda v: int(v, 0)
    ap.add_argument('--offset', type=num, default=0x50000)
    ap.add_argument('--size', type=num, default=0x10000)
    ap.add_argument('--uboot')
    ap.add_argument('--uboot-at', type=num, default=0x20000)
    ap.add_argument('flash')
    ap.add_argument('sets', nargs='*', metavar='key=value')
    a = ap.parse_args()

    d = bytearray(open(a.flash, 'rb').read())
    if a.offset < 0 or a.size < 8 or a.offset + a.size > len(d):
        raise SystemExit('the environment at 0x%x (0x%x bytes) is not inside '
                         '%s (0x%x bytes)' % (a.offset, a.size, a.flash, len(d)))
    if not a.uboot and not 0 <= a.uboot_at < a.offset:
        raise SystemExit('U-Boot at 0x%x is not before the environment'
                         % a.uboot_at)
    if a.uboot:
        uboot = open(a.uboot, 'rb').read()
    else:
        dec = lzma.LZMADecompressor(format=lzma.FORMAT_ALONE)
        uboot = dec.decompress(bytes(d[a.uboot_at:a.offset]))
    env = default_env(uboot)
    for kv in a.sets:
        k, v = kv.encode().split(b'=', 1)
        env[k] = v

    body = b'\0'.join(k + b'=' + v for k, v in env.items()) + b'\0\0'
    if len(body) > a.size - 4:
        raise SystemExit('environment does not fit')
    body = body.ljust(a.size - 4, b'\0')
    d[a.offset:a.offset + 4] = zlib.crc32(body).to_bytes(4, 'little')
    d[a.offset + 4:a.offset + a.size] = body
    open(a.flash, 'wb').write(d)
    print('%d variables at 0x%x' % (len(env), a.offset))


main()
