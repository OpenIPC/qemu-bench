#!/bin/bash
# Build an Ingenic SPI NAND image the way the vendor burner lays one out
# for the U-Boot isvp_<soc>_sfcnand configs (docs/ingenic.md, "SPI NAND"):
#
#   data offset   what
#   0             u-boot-with-spl.bin: SPL (NAND header, padded to 27 KiB),
#                 then the raw u-boot.bin at 0x6C00 (where the SPL looks)
#   0x6800        the burner's partition table (magic "nand"), which the
#                 3.10 jz_sfc_nand driver reads to make its MTD partitions
#   0xC0000       U-Boot environment (blank: U-Boot uses its default)
#   0x100000      kernel uImage   (bootcmd: nand read 0x80600000 0x100000 0x800000)
#   0x900000      UBI (ubi.mtd=2): volume 0 "rootfs", the SquashFS, static
#                 (root=/dev/ubiblock0_0 with ubi.block=0,0); volume 1
#                 "rootfs_data", UBIFS, the overlay OpenIPC's /init mounts
#                 (ubi0:rootfs_data), holding only /etc/fw_env.config
#   0x3100000     "data": blank
#
# i.e. mtdparts=sfc_nand:1M(boot),8M(kernel),40M(rootfs),-(data).
#
# --layout t41 is the vendor T41 U-Boot's (isvp_t41*_sfc0_nand: bootcmd
# "nand read 0x80600000 0x100000 0x300000", bootargs "ubi.mtd=2
# root=ubi0:rootfs rootfstype=ubifs mtdparts=sfc0_nand:1M(uboot),
# 3M(kernel),20M(root),-(appfs)"): the uImage at 0x100000, UBI at
# 0x400000 with one UBIFS volume "rootfs" made from the SquashFS's files.
# The 4.4 driver takes its partitions from mtdparts=.
#
# The image is the page data of the whole part (128 MiB for a 1 Gbit
# part), or with --oob every 2048-byte page followed by its OOB, FF
# (132 MiB for w25n01gv, 136 MiB for gd5f1gq4ub); the machine takes both.
#
#   bash qemu-boot/ingenic-nand-image.sh --uboot u-boot-with-spl.bin \
#       --kernel uImage --rootfs rootfs.squashfs --out nand.bin \
#       [--part w25n01gv|gd5f1gq4ub|gd5f2gq4ub|mx35lf2ge4ab] [--oob]
#       [--layout openipc|t41] [--env FILE]
#
# --env FILE puts a U-Boot environment (FILE: one name=value per line) at
# 0xC0000 in the redundant format both U-Boots use (CRC32, flag, 128 KiB).
#
# U-Boot is the OpenIPC u-boot-ingenic isvp_<soc>_sfcnand build with
# qemu-boot/ingenic-uboot-sfcnand.patch (bootargs and environment for
# this layout); ubinize and mkfs.ubifs are from mtd-utils, unsquashfs
# for --layout t41.
set -euo pipefail

UBOOT= KERNEL= ROOTFS= OUT= PART=w25n01gv OOB=0 LAYOUT=openipc ENV=
while [ $# -gt 0 ]; do
    case "$1" in
        --uboot) UBOOT=$2; shift 2 ;;
        --kernel) KERNEL=$2; shift 2 ;;
        --rootfs) ROOTFS=$2; shift 2 ;;
        --out) OUT=$2; shift 2 ;;
        --part) PART=$2; shift 2 ;;
        --oob) OOB=1; shift ;;
        --layout) LAYOUT=$2; shift 2 ;;
        --env) ENV=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
[ -n "$UBOOT" ] && [ -n "$OUT" ] || {
    echo "usage: $0 --uboot FILE [--kernel FILE] [--rootfs FILE]" \
         "--out FILE [--part PART] [--oob]" >&2; exit 2; }

case $PART in
    w25n01gv)     BLOCKS=1024 OOBSZ=64 ;;
    gd5f1gq4ub)   BLOCKS=1024 OOBSZ=128 ;;
    gd5f2gq4ub)   BLOCKS=2048 OOBSZ=128 ;;
    mx35lf2ge4ab) BLOCKS=2048 OOBSZ=64 ;;
    *) echo "unknown part $PART" >&2; exit 2 ;;
esac
PAGE=2048
PEB=$((64 * PAGE))
LEB=$((PEB - 2 * PAGE))
SIZE=$((BLOCKS * PEB))

M=$((1 << 20))
case $LAYOUT in
    openipc) KOFF=$M KMAX=$((8 * M)) UOFF=$((9 * M)) UMAX=$((40 * M)) ;;
    t41)     KOFF=$M KMAX=$((3 * M)) UOFF=$((4 * M)) UMAX=$((20 * M)) ;;
    *) echo "unknown layout $LAYOUT" >&2; exit 2 ;;
esac

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

if [ -n "$ROOTFS" ] && [ "$LAYOUT" = t41 ]; then
    unsquashfs -q -n -no-xattrs -d "$WORK/root" "$ROOTFS" >/dev/null
    # where the environment is, for OpenIPC's fw_printenv (else it reads
    # every partition through mtdblock at each call)
    printf '%s\t%s\t%s\t%s\n' /dev/mtd0 0xc0000 0x20000 0x20000 \
        /dev/mtd0 0xe0000 0x20000 0x20000 > "$WORK/root/etc/fw_env.config"
    PATH=$PATH:/usr/sbin:/sbin mkfs.ubifs -U -r "$WORK/root" -m $PAGE \
        -e $LEB -c 160 -x lzo -o "$WORK/rootfs.ubifs"
    cat > "$WORK/ubi.cfg" <<EOF
[rootfs]
mode=ubi
image=$WORK/rootfs.ubifs
vol_id=0
vol_type=dynamic
vol_name=rootfs
vol_flags=autoresize
EOF
    PATH=$PATH:/usr/sbin:/sbin ubinize -o "$WORK/rootfs.ubi" -p $PEB \
        -m $PAGE -s $PAGE "$WORK/ubi.cfg"
# the rootfs partition, 40 MiB: UBI with the SquashFS as a static volume
# (the root, through ubiblock) and a UBIFS volume rootfs_data, OpenIPC's
# overlay
elif [ -n "$ROOTFS" ]; then
    # padded to whole LEBs: ubiblock reads the last LEB in full, and a
    # static volume refuses reads past its data
    cp "$ROOTFS" "$WORK/rootfs.sqfs"
    truncate -s $(( ($(stat -c %s "$ROOTFS") + LEB - 1) / LEB * LEB )) \
        "$WORK/rootfs.sqfs"
    cat > "$WORK/ubi.cfg" <<EOF
[rootfs]
mode=ubi
image=$WORK/rootfs.sqfs
vol_id=0
vol_type=static
vol_name=rootfs
[rootfs_data]
mode=ubi
image=$WORK/rootfs_data.ubifs
vol_id=1
vol_type=dynamic
vol_name=rootfs_data
vol_flags=autoresize
EOF
    # The overlay (overlayfs upperdir=/overlay on 3.10) starts with the
    # U-Boot environment's place: OpenIPC's fw_printenv otherwise looks
    # for it by reading every partition through mtdblock, at each call.
    mkdir -p "$WORK/overlay/etc"
    printf '%s\t%s\t%s\t%s\n' /dev/mtd0 0xc0000 0x20000 0x20000 \
        /dev/mtd0 0xe0000 0x20000 0x20000 > "$WORK/overlay/etc/fw_env.config"
    PATH=$PATH:/usr/sbin:/sbin mkfs.ubifs -U -r "$WORK/overlay" -m $PAGE \
        -e $LEB -c 300 -o "$WORK/rootfs_data.ubifs"
    PATH=$PATH:/usr/sbin:/sbin ubinize -o "$WORK/rootfs.ubi" -p $PEB \
        -m $PAGE -s $PAGE "$WORK/ubi.cfg"
fi

python3 - "$WORK/data.bin" "$SIZE" "$UBOOT" "${KERNEL:-}" \
    "${ROOTFS:+$WORK/rootfs.ubi}" $KOFF $KMAX $UOFF $UMAX "$ENV" <<'EOF'
import struct, sys, zlib
out, size, uboot, kernel, ubi = sys.argv[1], int(sys.argv[2]), *sys.argv[3:6]
koff, kmax, uoff, umax = map(int, sys.argv[6:10])
env = sys.argv[10]
d = bytearray(b'\xff' * size)
def put(off, path, limit):
    b = open(path, 'rb').read()
    if len(b) > limit:
        sys.exit('%s is %d bytes, more than its 0x%x' % (path, len(b), limit))
    d[off:off + len(b)] = b
put(0, uboot, 0xC0000)
if d[:8] != bytes.fromhex('0605040302 55aa55') or d[8] != 0:
    sys.exit('%s has no SFC NAND SPL header (06 05 04 03 02 55 aa 55 00)'
             % uboot)
# the burner's partition table: magic, count, (pointer), then
# { name[32], size, offset, mask_flags, manager_mode (0 mtd, 1 ubi) }
M = 1 << 20
parts = [('boot', 0, M, 0), ('kernel', M, 8 * M, 0),
         ('rootfs', 9 * M, 40 * M, 1), ('data', 49 * M, size - 49 * M, 1)]
pt = struct.pack('<II', 0x646e616e, len(parts))
for name, off, sz, mode in parts:
    pt += struct.pack('<32sIIII', name.encode(), sz, off, 0, mode)
d[0x6800:0x6800 + len(pt)] = pt
if kernel:
    put(koff, kernel, kmax)
if ubi:
    put(uoff, ubi, umax)
if env:
    lines = [l.strip() for l in open(env) if l.strip() and l[0] != '#']
    data = b'\0'.join(l.encode() for l in lines) + b'\0\0'
    data = data.ljust(0x20000 - 5, b'\0')
    d[0xC0000:0xE0000] = struct.pack('<IB', zlib.crc32(data), 1) + data
open(out, 'wb').write(d)
EOF

if [ $OOB -eq 1 ]; then
    python3 - "$WORK/data.bin" "$OUT" $PAGE $OOBSZ <<'EOF'
import sys
src, dst, page, oob = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
d = open(src, 'rb').read()
pad = b'\xff' * oob
with open(dst, 'wb') as f:
    for i in range(0, len(d), page):
        f.write(d[i:i + page])
        f.write(pad)
EOF
else
    mv "$WORK/data.bin" "$OUT"
fi
echo "$OUT: $PART, $(stat -c %s "$OUT") bytes$([ $OOB -eq 1 ] && echo ', with OOB')"
