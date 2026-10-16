#!/bin/bash
#
# Boot OpenIPC on emulated Xiongmai XM510
#
#   cd qemu-boot && curl -sL \
#     https://github.com/OpenIPC/firmware/releases/download/latest/openipc.xm510-nor-lite.tgz \
#     | tar xz && cd ..
#
# The kernel is the vendor 3.0.101 one (ATAGs, machine 8888), so there is no
# DTB; mtdparts names the xmsfc controller "xm_sfc".
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

exec "$QEMU" -M xm510 \
    -kernel "$SCRIPT_DIR/uImage.xm510" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.xm510" \
    -nographic -serial mon:stdio \
    -append "console=ttyAMA0,115200 root=/dev/ram0 rootfstype=squashfs mtdparts=xm_sfc:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)" \
    $NIC_ARGS \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-xm510.log" \
    "$@"
