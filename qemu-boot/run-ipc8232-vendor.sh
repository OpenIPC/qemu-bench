#!/bin/bash
#
# Boot the Rostelecom IPC8232SWC-WE-B VENDOR firmware from its real SPI-NAND
# dump, with the SP2305 sensor attached on hibvt-i2c bus 0 (0x12060000 @0x3C)
# so the vendor VI/ISP sensor bring-up succeeds and the VC/AgentGreen stack
# comes all the way up.
#
# Unlike run-ev300-nand.sh (sensor=none, tuned for OpenIPC), this passes
# sensor=sp2305: the hisi-sp2305 I2C stub ACKs the driver's writes and returns
# chip id 0x2735, so `SNS: is [sp2305]!` → libsns_sp2305.so → ISP init → the
# Agent-Updater / VC-SYS-Daemon / InterConn agent reaching QR-provisioning.
#
# Known floor: the sp2305 stub satisfies detection but feeds no pixels into the
# VI capture path, so the agent logs "No video frames for 10 sec" and recycles
# VideoInput. Producing frames is a separate VI/ISP modelling task.
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"

FLASH="${FLASH:-$REPO_ROOT/vendor-dumps/ipc8232swc-we-vendor-nand-128mib-20261119.bin}"
SENSOR="${SENSOR:-sp2305}"
if [ ! -f "$FLASH" ]; then
    echo "Error: NAND image not found: $FLASH" >&2
    exit 1
fi

# Boot from a throwaway copy (U-Boot saveenv / UBI write back to NAND).
IMG="$(mktemp /tmp/ipc8232-vendor.XXXXXX.img)"
cp -f "$FLASH" "$IMG"
trap 'rm -f "$IMG"' EXIT

"$QEMU" -M hi3516ev300,sensor="$SENSOR",flash-file="$IMG" \
    -global hisi-fmc.nand-jedec=0xEFAA21 \
    -global hisi-fmc.nand-oob-size=64 \
    -nographic -serial mon:stdio -nic user,restrict=on \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-ipc8232-vendor.log" \
    "$@"
