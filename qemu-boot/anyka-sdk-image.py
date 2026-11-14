#!/usr/bin/env python3
"""Lay out an Anyka AnyCloud39AV100 SDK build as the camera's SPI NOR.

    anyka-sdk-image.py <sdk> <u-boot.bin> <out> [--ram 64|128]
                       [--dtb NAME.dtb] [--freq 888|860]

What the SDK's burn tool (tools/burntool, config_av100_<n>MB_spinor.txt)
writes, at its offsets, into an 8 MiB image:

    0x000000  UBOOT   u-boot.bin, its first 0x600 bytes the boot header
    0x037000  ENV     tools/burntool/env_av100_<n>M_spinor.img
    0x038000  ENVBK   the same
    0x039000  DTB     image/<board>.dtb
    0x049000  KERNEL  image/uImage
    0x1D9000  ROOTFS  image/root.sqsh4
    0x3FF000  CONFIG  image/usr.jffs2
    0x463000  APP     image/usr.sqsh4

The burn tool also patches the boot header that U-Boot's vectors.S
reserves (magic "ANYKAH3D" in the build, 0x200 bytes plus 1 KiB on the
AV100, CONFIG_SYS_TEXT_BASE 0x80DFFA00 = load address - 0x600): magic
"ANYKA322", the length the ROM copies, and the DDR register script from
tools/burntool/SDK_CPU_DDR2.ini (<n>M_Ram_Param1 = 888/444 MHz, Param2 =
860/430). The script is encoded the way the vendor's tool did on an
AK3918EV300L camera's flash: {address, value} pairs, a poll marker
0x66668881 taking the next pair as its operands (its own ini value is
dropped), the table closed after the 0x66668882 operation.

U-Boot comes from the SDK's os/uboot (ak3918av100_nor_defconfig); the SDK
ships no binary of it.
"""

import argparse
import os
import re
import struct
import sys

FLASH = 8 << 20
HDR = 0x600                 # AV100: 0x200 header + 1 KiB (vectors.S)
SCRIPT, SPIP = 0x2C, 0x18C
LOAD = 0x80E00000
KERNEL = 0x49000

POLL, OP, END = 0x66668881, 0x66668882, 0x88888888


def ddr_script(ini, ram, param):
    text = open(ini, 'rb').read().decode('ascii', 'replace')
    m = re.search(r'\[%dM_Ram_Param%d\][^\n]*\n(.*?)(\n\[|\Z)' % (ram, param),
                  text, re.S)
    if not m:
        sys.exit('%s: no [%dM_Ram_Param%d]' % (ini, ram, param))
    pairs = [tuple(int(x, 16) for x in v.split(','))
             for v in re.findall(r'reg_addr_value\d+\s*=\s*(\S+)', m.group(1))]
    out, i = [], 0
    while i < len(pairs):
        a, v = pairs[i]
        if a == POLL:
            # marker, then the polled register and the value it waits for
            out += [POLL, pairs[i + 1][0], pairs[i + 1][1]]
            i += 2
            continue
        out += [a, v]
        i += 1
        if a == OP:
            break
    out += [END, 0]
    if SCRIPT + 4 * len(out) > SPIP:
        sys.exit('DDR script runs into the SPIP descriptor')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('sdk')
    ap.add_argument('uboot')
    ap.add_argument('out')
    ap.add_argument('--ram', type=int, choices=(64, 128), default=64)
    ap.add_argument('--dtb')
    ap.add_argument('--freq', type=int, choices=(888, 860), default=888)
    a = ap.parse_args()

    burn = os.path.join(a.sdk, 'tools/burntool')
    image = os.path.join(a.sdk, 'image')
    dtb = a.dtb or ('EVB_CBDM_AK3918AV100%s_V1.0.0.dtb'
                    % ('N' if a.ram == 64 else 'X'))
    env = os.path.join(burn, 'env_av100_%dM_spinor.img' % a.ram)
    parts = [
        (0x037000, 0x1000, env),
        (0x038000, 0x1000, env),
        (0x039000, 0x10000, os.path.join(image, os.path.basename(dtb))
         if not os.path.isabs(dtb) else dtb),
        (KERNEL, 0x190000, os.path.join(image, 'uImage')),
        (0x1D9000, 0x226000, os.path.join(image, 'root.sqsh4')),
        (0x3FF000, 0x64000, os.path.join(image, 'usr.jffs2')),
        (0x463000, 0x39D000, os.path.join(image, 'usr.sqsh4')),
    ]

    uboot = bytearray(open(a.uboot, 'rb').read())
    if uboot[4:12] != b'ANYKAH3D' or len(uboot) > 0x37000:
        sys.exit('%s: not an AV100 U-Boot (ANYKAH3D header, < 220 KiB)'
                 % a.uboot)
    if struct.unpack_from('<I', uboot, 0x20)[0] != LOAD:
        sys.exit('%s: load address is not %#x' % (a.uboot, LOAD))
    uboot[4:12] = b'ANYKA322'
    # What the ROM copies: everything up to the kernel partition.
    struct.pack_into('<I', uboot, 0x0C, KERNEL - HDR)
    script = ddr_script(os.path.join(burn, 'SDK_CPU_DDR2.ini'), a.ram,
                        1 if a.freq == 888 else 2)
    uboot[SCRIPT:SPIP] = bytes(SPIP - SCRIPT)
    struct.pack_into('<%dI' % len(script), uboot, SCRIPT, *script)

    img = bytearray(b'\xff' * FLASH)
    img[:len(uboot)] = uboot
    for off, size, path in parts:
        data = open(path, 'rb').read()
        if len(data) > size:
            sys.exit('%s: %#x bytes, partition holds %#x'
                     % (path, len(data), size))
        img[off:off + len(data)] = data
    open(a.out, 'wb').write(img)
    print('%s: %d MiB DDR script (%d MHz), %s' % (a.out, a.ram, a.freq,
                                                 os.path.basename(dtb)))


if __name__ == '__main__':
    main()
