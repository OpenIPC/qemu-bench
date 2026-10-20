#!/bin/bash
#
# Boot OpenIPC on emulated Novatek NT98562 (or NT98566: SOC=nt98566)
#
#   cd qemu-boot && curl -sL \
#     https://github.com/OpenIPC/firmware/releases/download/latest/openipc.nt98562-nor-lite.tgz \
#     | tar xz && cd ..
#   bash qemu-boot/run-nt98562.sh
#
# The release goes into an 8 MiB SPI NOR image laid out the OpenIPC way
# (qemu-boot/nor-<soc>.bin, made once and then kept: it is the camera's
# flash, rootfs_data and all), and the kernel mounts its root from it.
# The uImage carries no DTB; the machine builds one.
#
# FLASH=image.bin boots a whole flash image instead, the way the chip does:
# the machine's boot ROM loads the vendor U-Boot from it.  A Novatek SDK
# update package (FW9856xA.bin) becomes one with
# qemu-boot/nvtpack2flash.py.  Ethernet is not modelled yet (docs/novatek.md).
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"
SOC="${SOC:-nt98562}"

if [ -n "$FLASH" ]; then
    exec "$QEMU" -M "$SOC,flash-file=$FLASH" \
        -nographic -serial mon:stdio \
        -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-$SOC.log" \
        "$@"
fi

KERNEL="$SCRIPT_DIR/uImage.$SOC"
ROOTFS="$SCRIPT_DIR/rootfs.squashfs.$SOC"
NOR="$SCRIPT_DIR/nor-$SOC.bin"
if [ ! -f "$KERNEL" ] || [ ! -f "$ROOTFS" ]; then
    echo "need $KERNEL and $ROOTFS: unpack openipc.$SOC-nor-lite.tgz here" >&2
    exit 1
fi
if [ ! -f "$NOR" ]; then
    python3 - "$KERNEL" "$ROOTFS" "$NOR" <<'PY'
import sys
kernel, rootfs, nor = sys.argv[1:]
img = bytearray(b'\xff' * (8 << 20))
k = open(kernel, 'rb').read()
r = open(rootfs, 'rb').read()
img[0x50000:0x50000 + len(k)] = k
img[0x250000:0x250000 + len(r)] = r
open(nor, 'wb').write(img)
PY
fi

exec "$QEMU" -M "$SOC,flash-file=$NOR" \
    -kernel "$KERNEL" \
    -nographic -serial mon:stdio \
    -append "console=ttyS0,115200 root=/dev/mtdblock3 rootfstype=squashfs mtdparts=spi_nor.0:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)" \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-$SOC.log" \
    "$@"
