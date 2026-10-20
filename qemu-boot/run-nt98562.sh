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
# After a new release download the image still holds the old rootfs: the
# script says so, and REBUILD=1 lays the new one out (dropping what the
# old image had written to rootfs_data).
# The uImage carries no DTB; the machine builds one.
#
# FLASH=image.bin boots a whole flash image instead, the way the chip does:
# the machine's boot ROM loads the vendor U-Boot from it.  A Novatek SDK
# update package (FW9856xA.bin) becomes one with
# qemu-boot/nvtpack2flash.py.  Networking is bridged TAP when tap0 exists
# (qemu-boot/net-setup.sh), else SLIRP.
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"
SOC="${SOC:-nt98562}"

TAP="${TAP:-tap0}"
if ip link show "$TAP" &>/dev/null; then
    NIC_ARGS="-nic tap,ifname=$TAP,script=no,downscript=no"
    echo "Using bridged TAP networking ($TAP)"
else
    NIC_ARGS="-nic user"
    echo "TAP not available; using SLIRP user-mode networking"
fi

if [ -n "$FLASH" ]; then
    exec "$QEMU" -M "$SOC,flash-file=$FLASH" \
        -nographic -serial mon:stdio $NIC_ARGS \
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
if [ -f "$NOR" ] && [ -z "$REBUILD" ] && \
        { [ "$ROOTFS" -nt "$NOR" ] || [ "$KERNEL" -nt "$NOR" ]; }; then
    echo "$NOR predates $(basename "$ROOTFS")/$(basename "$KERNEL"):" \
         "it holds the previous release's rootfs." >&2
    echo "Run with REBUILD=1 to lay out the new release (rootfs_data is" \
         "lost), or delete it." >&2
    exit 1
fi
if [ ! -f "$NOR" ] || [ -n "$REBUILD" ]; then
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
    -nographic -serial mon:stdio $NIC_ARGS \
    -append "console=ttyS0,115200 root=/dev/mtdblock3 rootfstype=squashfs mtdparts=spi_nor.0:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)" \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-$SOC.log" \
    "$@"
