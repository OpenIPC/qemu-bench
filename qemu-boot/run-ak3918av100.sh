#!/bin/bash
#
# Boot an Anyka AK3918AV100 (64 MiB) or AK3918AV100X (128 MiB) from a
# SPI NOR image, the way the mask ROM does (ANYKA322 boot header at 0).
#
# Usage:
#   bash qemu-boot/run-ak3918av100.sh <flash.bin> [ak3918av100|ak3918av100x]
#
# Build <flash.bin> from an AnyCloud39AV100 SDK with:
#   python3 qemu-boot/anyka-sdk-image.py <sdk> <u-boot.bin> flash.bin \
#       --ram 64          # or --ram 128 for the X part
# (the SDK's os/uboot, ak3918av100_nor_defconfig, builds u-boot.bin).
#
# Serial goes to the terminal and /tmp/ak3918av100_serial.log.  Boot a
# copy: erases and programs are written back to the image.
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

FLASH_FILE="${1:?usage: $0 <flash.bin> [machine]}"
MACHINE="${2:-ak3918av100}"

if [ ! -f "$FLASH_FILE" ]; then
    echo "Error: flash image not found: $FLASH_FILE" >&2
    exit 1
fi

exec "$QEMU" -M "$MACHINE,flash-file=$FLASH_FILE" \
    -nographic -serial mon:stdio \
    -nic user,restrict=on \
    -d unimp,guest_errors \
    -D "$SCRIPT_DIR/qemu-ak3918av100.log" \
    2>&1 | tee /tmp/ak3918av100_serial.log
