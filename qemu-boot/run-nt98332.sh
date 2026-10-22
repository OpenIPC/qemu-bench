#!/bin/bash
# Novatek NT98332 (NA51103): boot a whole 16 MiB SPI NOR image -- a dump of
# a Xiongmai NVR board -- through its own U-Boot, with the GUI on the
# QEMU display, a USB mouse, and a SATA disk.
#
#   FLASH=nor.bin [DISK=disk.img] [LAN=mcast] bash qemu-boot/run-nt98332.sh
#
# LAN=mcast puts it on a QEMU socket network instead of user networking,
# with qemu-boot/run-xm530-camera.sh as a camera to add (docs/novatek.md,
# "Camera video"); the web UI is then not forwarded to the host.
#
# The image is written to (env, logs, settings): keep a copy of the dump.
# A dump whose U-Boot env is zeroed boots on U-Boot's defaults, with the
# console off; qemu-boot/uboot-env.py nor.bin xmuart=0 turns it on.  The
# board's address is a static 192.168.1.10, so user networking is put on
# that subnet, with its web UI at http://127.0.0.1:8080/.
set -e
cd "$(dirname "$0")/.."

QEMU=${QEMU:-./qemu-src/build/qemu-system-aarch64}
: "${FLASH:?set FLASH to a NT98332 NOR image}"
DISK=${DISK:-qemu-boot/sata-nt98332.img}

[ -f "$DISK" ] || truncate -s 2G "$DISK"

if [ "${LAN:-}" = mcast ]; then
    NET=(-netdev socket,id=lan,mcast="${MCAST:-230.0.0.1:12345}"
         -net nic,netdev=lan,macaddr=52:54:00:00:00:10)
else
    NET=(-nic user,net=192.168.1.0/24,host=192.168.1.1,hostfwd=tcp:127.0.0.1:8080-192.168.1.10:80)
fi

exec "$QEMU" -M nt98332,flash-file="$FLASH" \
    -serial mon:stdio \
    -device usb-mouse,bus=usb-bus.1 -device usb-kbd,bus=usb-bus.0 \
    -drive if=ide,index=0,format=raw,file="$DISK" \
    "${NET[@]}" "$@"
