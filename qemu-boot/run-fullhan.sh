#!/bin/bash
#
# Boot a Fullhan camera's SPI NOR image (a flash dump, or what
# fh-openipc-flash.sh builds) on an emulated Fullhan SoC.
#
#   bash qemu-boot/run-fullhan.sh <machine> <flash.bin> [qemu args...]
#
# e.g. run-fullhan.sh fh8856v200 dump.bin.  Machines: fh8852v200,
# fh8856v200, fh8858v200, fh8652, fh8656, fh8658, fh8852v100, fh8856v100,
# fh8626v100, fh8632.  Erases and programs are written back to the image,
# so boot a copy of a dump.
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

if [ $# -lt 2 ]; then
    sed -n '3,11p' "$0" >&2
    exit 2
fi
MACHINE=$1
FLASH=$2
shift 2

TAP="${TAP:-tap0}"
if ip link show "$TAP" &>/dev/null 2>&1; then
    NIC_ARGS="-nic tap,ifname=$TAP,script=no,downscript=no"
    echo "Using bridged TAP networking ($TAP)"
else
    NIC_ARGS="-nic user"
    echo "TAP not available; using SLIRP user-mode networking"
fi

exec "$QEMU" -M "$MACHINE,flash-file=$FLASH" \
    -nographic -serial mon:stdio \
    $NIC_ARGS \
    "$@"
