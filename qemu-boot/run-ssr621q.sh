#!/bin/bash
#
# Boot an emulated SigmaStar SSR621Q NVR from its whole SPI-NOR dump,
# through its own IPL and U-Boot to the vendor GUI (docs/sigmastar.md):
#
#   FLASH=nvr.bin bash qemu-boot/run-ssr621q.sh
#
# Erases and programs are written back to nvr.bin: boot a copy.  On a
# Xiongmai NVR the kernel only prints on the console with xmuart=0 in the
# U-Boot environment:
#
#   qemu-boot/sstar-env.py nvr.bin xmuart=0
#
# HDD=disk.img attaches a SATA disk; a blank one (truncate -s 8G disk.img)
# is formatted by the NVR at boot.  The GUI comes up on the graphic console (1280x1024); a USB
# tablet and keyboard are attached for it.  SLIRP is put on the NVR's
# default 192.168.1.0/24 (the NVR is 192.168.1.9).
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

if [ -z "$FLASH" ]; then
    echo "Set FLASH=<SPI NOR dump> (see the top of $0)" >&2
    exit 1
fi

DISK_ARGS=()
if [ -n "$HDD" ]; then
    DISK_ARGS=(-drive "if=ide,index=0,file=$HDD,format=raw")
fi

exec "$QEMU" -M ssr621q,flash-file="$FLASH" \
    -serial mon:stdio \
    -nic user,net=192.168.1.0/24,host=192.168.1.1,restrict=on \
    -device usb-tablet,bus=usb-bus.0 -device usb-kbd,bus=usb-bus.1 \
    "${DISK_ARGS[@]}" \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-ssr621q.log" \
    "$@"
