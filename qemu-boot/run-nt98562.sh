#!/bin/bash
#
# Boot OpenIPC on emulated Novatek NT98562 (or NT98566: SOC=nt98566)
#
#   cd qemu-boot && curl -sL \
#     https://github.com/OpenIPC/firmware/releases/download/latest/openipc.nt98562-nor-lite.tgz \
#     | tar xz && cd ..
#
# The kernel carries no DTB; the machine builds one.  The rootfs is
# SquashFS and this kernel has no RAM disk driver, so it is repacked as an
# initramfs, which needs more than the 30 MB the device tree gives Linux
# (the rest is the HDAL media pool, unused without vendor modules): hence
# mem=.  Flash boot and Ethernet are not modelled yet (docs/novatek.md).
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"
SOC="${SOC:-nt98562}"
case "$SOC" in
    nt98566) MEM=128M ;;
    *)       MEM=64M ;;
esac

KERNEL="$SCRIPT_DIR/uImage.$SOC"
ROOTFS="$SCRIPT_DIR/rootfs.squashfs.$SOC"
CPIO="$SCRIPT_DIR/rootfs-$SOC.cpio"
if [ ! -f "$KERNEL" ] || [ ! -f "$ROOTFS" ]; then
    echo "need $KERNEL and $ROOTFS: unpack openipc.$SOC-nor-lite.tgz here" >&2
    exit 1
fi
if [ ! -f "$CPIO" ] || [ "$ROOTFS" -nt "$CPIO" ]; then
    tmp=$(mktemp -d)
    unsquashfs -q -d "$tmp/root" "$ROOTFS" >/dev/null
    (cd "$tmp/root" && find . | cpio -o -H newc 2>/dev/null) > "$CPIO"
    rm -rf "$tmp"
fi

exec "$QEMU" -M "$SOC" \
    -kernel "$KERNEL" \
    -initrd "$CPIO" \
    -nographic -serial mon:stdio \
    -append "console=ttyS0,115200 mem=$MEM rdinit=/sbin/init" \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-$SOC.log" \
    "$@"
