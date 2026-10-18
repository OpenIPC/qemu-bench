#!/bin/bash
#
# Boot an emulated Goke GK7102 (GK7101 die, 64 MiB in-package DDR).
#
# Either a camera's whole SPI-NOR dump, through its own U-Boot:
#
#   FLASH=dump.bin bash qemu-boot/run-gk7102.sh
#
# (erases and programs are written back to dump.bin: boot a copy), or the
# OpenIPC gk7102 release with -kernel / -initrd:
#
#   cd qemu-boot && curl -sL \
#     https://github.com/OpenIPC/firmware/releases/download/latest/openipc.gk7102-nor-lite.tgz \
#     | tar xz && cd ..
#   HAL=uboot-or-dump.bin bash qemu-boot/run-gk7102.sh
#
# The GK710x kernel calls into the vendor HAL blob that U-Boot leaves at
# 0xC0012000 (docs/goke-gk710x.md), so a -kernel boot needs one: HAL= names
# any image carrying it -- a GK7101/GK7102 vendor U-Boot binary (the SDK's
# boot/prebuild/uboot-gk7102-*.bin) or a camera's flash dump.  OpenIPC's
# kernel needs a HAL of version 20160324 or later (it prints the version as
# "hal version = ..."; older ones lack the SPI accessors it calls).
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

TAP="${TAP:-tap0}"
if ip link show "$TAP" &>/dev/null 2>&1; then
    NIC_ARGS="-nic tap,ifname=$TAP,script=no,downscript=no"
    echo "Using bridged TAP networking ($TAP)"
else
    NIC_ARGS="-nic user"
    echo "TAP not available; using SLIRP user-mode networking"
fi

if [ -n "$FLASH" ]; then
    exec "$QEMU" -M gk7102,flash-file="$FLASH" \
        -nographic -serial mon:stdio \
        $NIC_ARGS \
        -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-gk7102.log" \
        "$@"
fi

if [ -z "$HAL" ]; then
    echo "Set HAL=<vendor U-Boot or flash dump> (see the top of $0)" >&2
    exit 1
fi

exec "$QEMU" -M gk7102,hal-file="$HAL" \
    -kernel "$SCRIPT_DIR/uImage.gk7102" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.gk7102" \
    -nographic -serial mon:stdio \
    -append "console=ttySGK0,115200 root=/dev/ram0 rootfstype=squashfs mtdparts=gk_flash:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)" \
    $NIC_ARGS \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-gk7102.log" \
    "$@"
