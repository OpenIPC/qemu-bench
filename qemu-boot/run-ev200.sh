#!/bin/bash
#
# Boot OpenIPC on emulated Hi3516EV200 (vendor-blob testing).
#
# Memory layout (mem= / mmz=) and the default IMX307 sensor are now
# defined in the SoC table (qemu/hw/arm/hisilicon.c) and injected
# automatically.  To attach a different sensor, append
# `-M hi3516ev200,sensor=NAME` (or sensor=none to skip).
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

TAP="${TAP:-tap0}"
# NIC_OPTS — extra comma-separated options appended to the chosen
# nic spec (typically used to add hostfwd entries for SLIRP, e.g.
# NIC_OPTS=,hostfwd=tcp::1234-:1234 to expose an in-guest port).
NIC_OPTS="${NIC_OPTS:-}"
if ip link show "$TAP" &>/dev/null 2>&1; then
    NIC_ARGS="-nic tap,ifname=$TAP,script=no,downscript=no${NIC_OPTS}"
    echo "Using bridged TAP networking ($TAP)"
else
    NIC_ARGS="-nic user${NIC_OPTS}"
    echo "TAP not available; using SLIRP user-mode networking"
fi

INIT="${INIT:-}"
CMDLINE="console=ttyAMA0,115200 earlyprintk vdso=0 root=/dev/ram0 rootfstype=squashfs mtdparts=hi_sfc:256k(boot),64k(env),3072k(kernel),10240k(rootfs),-(rootfs_data)"
if [ -n "$INIT" ]; then
    CMDLINE="$CMDLINE init=$INIT"
fi

exec "$QEMU" -M hi3516ev200 \
    -kernel "$SCRIPT_DIR/uImage.hi3516ev200" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.hi3516ev200" \
    -nographic -serial mon:stdio \
    -append "$CMDLINE" \
    $NIC_ARGS \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-ev200.log" \
    "$@"
