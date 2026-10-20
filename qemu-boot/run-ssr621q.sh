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
# CAMERA=1 also starts qemu-boot/fake-ipcam.py, an RTSP/ONVIF camera with a
# test pattern, and puts it at 192.168.1.10 for the NVR to add (the
# "Live view" section of docs/sigmastar.md); decoding it needs a QEMU built
# with libavcodec.
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

NIC=user,net=192.168.1.0/24,host=192.168.1.1,restrict=on
if [ -n "$CAMERA" ]; then
    CAM="$SCRIPT_DIR/fake-ipcam.py"
    CAM_HTTP=${CAM_HTTP:-18080}
    CAM_RTSP=${CAM_RTSP:-18554}
    # CAM_ARGS: more options, e.g. "--file cam.h264 --size 1920x1080 --fps 25"
    python3 "$CAM" --http "$CAM_HTTP" --rtsp "$CAM_RTSP" \
        --advertise 192.168.1.10 $CAM_ARGS &
    CAM_PID=$!
    trap 'kill $CAM_PID 2>/dev/null' EXIT
    # One relay process per guest connection (a tcp: chardev is one
    # connection that every guest connection would share).
    NIC="$NIC,guestfwd=tcp:192.168.1.10:80-cmd:python3 $CAM --relay $CAM_HTTP"
    NIC="$NIC,guestfwd=tcp:192.168.1.10:554-cmd:python3 $CAM --relay $CAM_RTSP"
fi

"$QEMU" -M ssr621q,flash-file="$FLASH" \
    -serial mon:stdio \
    -nic "$NIC" \
    -device usb-tablet,bus=usb-bus.0 -device usb-kbd,bus=usb-bus.1 \
    "${DISK_ARGS[@]}" \
    -d unimp,guest_errors -D "$SCRIPT_DIR/qemu-ssr621q.log" \
    "$@"
