#!/bin/bash
# Build a Fullhan NOR image that boots OpenIPC through the vendor U-Boot,
# the way OpenIPC is installed on these cameras: OpenIPC publishes no
# Fullhan bootloader, so the camera keeps its own.
#
#   bash qemu-boot/fh-openipc-flash.sh <vendor-dump.bin> <soc> [DIR] [OUT]
#
# <vendor-dump.bin> is a full flash dump of a camera with the same SoC; its
# first 256 KiB (the mask ROM's "2BL*" header, DDR scripts and U-Boot) are
# kept.  uImage.<soc> and rootfs.squashfs.<soc> come from DIR (default:
# qemu-boot/, as fetch-openipc-lite.sh leaves them).  The layout is
# OpenIPC's 8 MiB NOR one, which the vendor U-Boot's own matches (env at
# 0x40000, 64 KiB, kernel at 0x50000):
#
#   256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)
#
# and the environment is U-Boot's default with OpenIPC's bootargs/bootcmd.
set -euo pipefail

DUMP=$1
SOC=$2
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DIR=${3:-$SCRIPT_DIR}
OUT=${4:-$DIR/openipc-$SOC-nor-8mb.bin}
KERNEL=$DIR/uImage.$SOC
ROOTFS=$DIR/rootfs.squashfs.$SOC

if [ "$(head -c 8 "$DUMP")" != "2BL*OABD" ]; then
    echo "$DUMP has no Fullhan boot header" >&2
    exit 1
fi
if [ "$(stat -c %s "$KERNEL")" -gt $((0x200000)) ] ||
   [ "$(stat -c %s "$ROOTFS")" -gt $((0x500000)) ]; then
    echo "$KERNEL or $ROOTFS does not fit its partition" >&2
    exit 1
fi

# The "uboot" record of the boot header: size at +0x20, offset at +0x24.
read -r size off < <(python3 - "$DUMP" <<'EOF'
import struct, sys
d = open(sys.argv[1], 'rb').read(0x300)
for r in range(0x100, 0x300, 0x40):
    if d[r:r + 6] == b'uboot\0':
        print(*struct.unpack('<2I', d[r + 0x20:r + 0x28]))
        break
else:
    sys.exit('no uboot record in the boot header')
EOF
)
UBOOT=$(mktemp)
trap 'rm -f "$UBOOT"' EXIT
dd if="$DUMP" of="$UBOOT" bs=1 skip="$off" count="$size" status=none

head -c $((8 << 20)) /dev/zero | tr '\0' '\377' > "$OUT"
dd if="$DUMP" of="$OUT" bs=64k count=4 conv=notrunc status=none
dd if="$KERNEL" of="$OUT" bs=64k seek=5 conv=notrunc status=none
dd if="$ROOTFS" of="$OUT" bs=64k seek=37 conv=notrunc status=none

# The MTD device's name: the 3.0.8 kernels' m25p80 platform data says
# "spi_flash", the 4.9 ones take the SPI device's.
case $SOC in
    *v100) MTDID=spi_flash ;;
    *)     MTDID=spi0.0 ;;
esac
MTDPARTS="$MTDID:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)"
python3 "$SCRIPT_DIR/uboot-env.py" --offset 0x40000 --size 0x10000 \
    --uboot "$UBOOT" "$OUT" \
    "bootargs=mem=40M console=ttyS0,115200 panic=20 root=/dev/mtdblock3 rootfstype=squashfs init=/init mtdparts=$MTDPARTS" \
    "bootcmd=sf probe 0; sf read 0xa1000000 0x50000 0x200000; bootm 0xa1000000" \
    "bootdelay=1"
ls -l "$OUT"
