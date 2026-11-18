#!/bin/bash
#
# Boot an Ingenic T-series machine interactively (docs/ingenic.md).
#
#   bash qemu-boot/run-ingenic.sh <soc>                # -kernel: uImage.<soc>
#                                                      # + rootfs.squashfs.<soc>
#   bash qemu-boot/run-ingenic.sh <soc> flash.bin      # a NOR image, from its SPL
#   bash qemu-boot/run-ingenic.sh <soc> flash.bin -m 256M -drive if=sd,file=sd.img
#
# <soc> is t10 t20 t21 t23 t30 t31 t40 t41.  fetch-openipc-lite.sh leaves
# the kernel and rootfs here.  A flash image is written to as the camera
# writes its flash: boot a copy of a dump you want to keep.  Extra
# arguments go to QEMU.  The network is SLIRP (tap0 if it exists).
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-mipsel"

SOC=${1:?usage: run-ingenic.sh <soc> [flash.bin] [qemu args...]}
shift
TAP="${TAP:-tap0}"
if ip link show "$TAP" &>/dev/null; then
    NIC_ARGS="-nic tap,ifname=$TAP,script=no,downscript=no"
else
    NIC_ARGS="-nic user"
fi

if [ -n "$1" ] && [ "${1#-}" = "$1" ]; then
    FLASH=$1
    shift
    exec "$QEMU" -M "$SOC,flash-file=$FLASH" -nographic -serial mon:stdio \
        $NIC_ARGS "$@"
fi
exec "$QEMU" -M "$SOC" -nographic -serial mon:stdio \
    -kernel "$SCRIPT_DIR/uImage.$SOC" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.$SOC" \
    $NIC_ARGS "$@"
