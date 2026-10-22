#!/bin/bash
# An emulated Xiongmai XM530 camera running the OpenIPC release, as a video
# source for the NT98332 NVR (qemu-boot/run-nt98332.sh with LAN=mcast): both
# join the same QEMU socket network.
#
#   bash qemu-boot/run-xm530-camera.sh            # console on stdio
#
# Needs qemu-boot/uImage.xm530 and rootfs.squashfs.xm530 from the OpenIPC
# release (openipc.xm530-nor-lite.tgz).  The flash image it builds,
# qemu-boot/nor-xm530-camera.bin, keeps the camera's settings between runs.
# Once, on the console (see docs/novatek.md, "Camera video"):
#   - log in as root, accept the EULA and set a password (RTSP uses it);
#   - cli -s .video0.codec h264; cli -s .video0.size 1920x1080
#     cli -s .video1.enabled true; cli -s .video1.codec h264
#     cli -s .video1.size 704x576; /etc/init.d/S95majestic restart
#   - a static 192.168.1.20 in /etc/network/interfaces.d/eth0.
set -e
cd "$(dirname "$0")/.."

QEMU=${QEMU:-./qemu-src/build/qemu-system-arm}
KERNEL=qemu-boot/uImage.xm530
ROOTFS=qemu-boot/rootfs.squashfs.xm530
FLASH=${FLASH:-qemu-boot/nor-xm530-camera.bin}
MCAST=${MCAST:-230.0.0.1:12345}

for f in "$KERNEL" "$ROOTFS"; do
    [ -f "$f" ] || { echo "missing $f (OpenIPC xm530 release)" >&2; exit 1; }
done
ROOTFS_KB=$(( ( ($(stat -c %s "$ROOTFS") + 65535) / 65536 ) * 64 ))
if [ ! -f "$FLASH" ]; then
    head -c $((8 * 1024 * 1024)) /dev/zero | tr '\000' '\377' > "$FLASH"
    dd if="$ROOTFS" of="$FLASH" bs=1k seek=$((256 + 64 + 2048)) \
        conv=notrunc 2>/dev/null
fi
MTD="xm_sfc:256k(boot),64k(env),2048k(kernel),${ROOTFS_KB}k(rootfs),-(rootfs_data)"

exec "$QEMU" -M xm530,flash-file="$FLASH" -smp 2 -m 64M -kernel "$KERNEL" \
    -display none -serial mon:stdio \
    -netdev socket,id=lan,mcast="$MCAST" \
    -net nic,netdev=lan,macaddr=52:54:00:00:00:20 \
    -append "mem=32M console=ttyAMA0,115200 root=/dev/mtdblock3 rootfstype=squashfs init=/init mtdparts=$MTD" \
    "$@"
