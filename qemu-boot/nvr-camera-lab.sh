#!/bin/sh
# SPDX-License-Identifier: MIT
#
# Stand up a throwaway two-guest LAN for ONVIF / NVR troubleshooting: an NVR
# twin (a flash dump of a real recorder) and a camera twin (an OpenIPC
# image), bridged with DHCP inside an unprivileged network namespace. Nothing
# touches the host network; the namespace and its interfaces vanish on exit.
#
# Built for OpenIPC/majestic#379 (an NVR ignoring a camera's motion Notify):
# boot both, then run qemu-boot/onvif-notify-probe.py from the lab shell to
# capture exactly what the camera pushes, or point the NVR at the camera and
# watch its console.
#
#   bash qemu-boot/nvr-camera-lab.sh \
#       --nvr-flash ~/dumps/nvr.bin --nvr-machine hi3536cv100 \
#       --cam-flash ~/images/openipc-hi3516ev200-nor-lite-8mb.bin \
#       --cam-machine hi3516ev200,sensor=sc2315e
#
# --cam-flash should be a *full* OpenIPC flash image (openipc.org
# download_full_image) so the camera has a writable overlay — majestic needs
# it to store the EULA acceptance, the root password and onvif.password.
#
# Inside the lab shell:
#   br0 is 192.168.50.1; dnsmasq hands out 192.168.50.100-150.
#   NVR  console:  socat - UNIX-CONNECT:$WORK/nvr.sock
#   cam  console:  socat - UNIX-CONNECT:$WORK/cam.sock
#   cam  QMP:      $WORK/cam-qmp.sock   (qom-set /machine/scene light N)
#   capture Notify: python3 qemu-boot/onvif-notify-probe.py --host <cam-ip> \
#                       --user admin --password <pw> --seconds 20 --out /tmp/n
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
# Defaults that must survive the env hand-off to the re-exec'd second pass,
# so :- rather than a plain assignment.
QEMU=${QEMU:-"$HERE/../qemu-src/build/qemu-system-arm"}
NVR_MACHINE=${NVR_MACHINE:-}; NVR_FLASH=${NVR_FLASH:-}
CAM_MACHINE=${CAM_MACHINE:-}; CAM_FLASH=${CAM_FLASH:-}
CAM_KERNEL=${CAM_KERNEL:-}; CAM_ROOTFS=${CAM_ROOTFS:-}
WORK=${WORK:-$(mktemp -d /tmp/nvr-lab.XXXXXX)}

# First pass: parse args, then re-exec inside a fresh user+network namespace
# (so br0/taps need no root and are cleaned up automatically on exit). The
# second pass arrives with _NVRLAB_NS=1 and the config carried through the
# environment, so it skips parsing.
if [ "${_NVRLAB_NS:-}" != 1 ]; then
    while [ $# -gt 0 ]; do
        case $1 in
            --nvr-flash)   NVR_FLASH=$2; shift 2 ;;
            --nvr-machine) NVR_MACHINE=$2; shift 2 ;;
            --cam-flash)   CAM_FLASH=$2; shift 2 ;;
            --cam-machine) CAM_MACHINE=$2; shift 2 ;;
            --cam-kernel)  CAM_KERNEL=$2; shift 2 ;;
            --cam-rootfs)  CAM_ROOTFS=$2; shift 2 ;;
            --qemu)        QEMU=$2; shift 2 ;;
            --work)        WORK=$2; shift 2 ;;
            *) echo "unknown argument: $1" >&2; exit 2 ;;
        esac
    done

    [ -n "$NVR_FLASH" ] && [ -n "$NVR_MACHINE" ] || {
        echo "need --nvr-flash FILE --nvr-machine NAME" >&2; exit 2; }
    [ -n "$CAM_MACHINE" ] || { echo "need --cam-machine NAME" >&2; exit 2; }
    [ -n "$CAM_FLASH" ] || { [ -n "$CAM_KERNEL" ] && [ -n "$CAM_ROOTFS" ]; } || {
        echo "need --cam-flash FILE (preferred) or --cam-kernel + --cam-rootfs" >&2
        exit 2; }

    export _NVRLAB_NS=1 QEMU WORK NVR_FLASH NVR_MACHINE CAM_FLASH CAM_MACHINE \
           CAM_KERNEL CAM_ROOTFS
    exec unshare -rn "$0"
fi

mkdir -p "$WORK"
# Work on copies so the dumps/images are never written through.
cp -f "$NVR_FLASH" "$WORK/nvr-flash.bin"
[ -n "$CAM_FLASH" ] && cp -f "$CAM_FLASH" "$WORK/cam-flash.bin"

ip link set lo up
ip link add br0 type bridge
ip addr add 192.168.50.1/24 dev br0
for t in tap0 tap1; do
    ip tuntap add "$t" mode tap
    ip link set "$t" master br0
    ip link set "$t" up
done
ip link set br0 up

dnsmasq --no-daemon --conf-file=/dev/null --interface=br0 --bind-interfaces \
    --port=0 --dhcp-range=192.168.50.100,192.168.50.150,12h \
    --dhcp-leasefile="$WORK/leases" > "$WORK/dnsmasq.log" 2>&1 &
DNSMASQ=$!

# Xiongmai hi3536cv100 NVR device trees strap the HiGMAC PHY at MDIO address 2
# (mdio@100a03c0/ethernet-phy@2), where the OpenIPC default is 1; hand the
# model that address so the recorder's eth0 comes up. nvr_phy is empty or two
# fixed tokens, so the unquoted split below is intentional.
nvr_phy=""
case $NVR_MACHINE in
    hi3536cv100*) nvr_phy="-global hisi-gmac.phy-addr=2" ;;
esac

# shellcheck disable=SC2086
"$QEMU" -M "$NVR_MACHINE,flash-file=$WORK/nvr-flash.bin" -m 512M -nographic \
    -monitor none -serial "unix:$WORK/nvr.sock,server=on,wait=off" \
    -nic tap,ifname=tap0,script=no,downscript=no \
    $nvr_phy \
    > "$WORK/nvr-qemu.log" 2>&1 &
NVR=$!

# The camera's disk arguments vary (full image vs split kernel+rootfs) and may
# hold spaces, so pass them as positional parameters rather than splitting an
# unquoted string.
cam_boot() {
    "$QEMU" "$@" -icount shift=0 -nographic -monitor none \
        -serial "unix:$WORK/cam.sock,server=on,wait=off" \
        -qmp "unix:$WORK/cam-qmp.sock,server=on,wait=off" \
        -nic tap,ifname=tap1,script=no,downscript=no,mac=52:54:00:12:34:57 \
        > "$WORK/cam-qemu.log" 2>&1 &
}
if [ -n "$CAM_FLASH" ]; then
    cam_boot -M "$CAM_MACHINE,flash-file=$WORK/cam-flash.bin"
else
    cam_boot -M "$CAM_MACHINE" -kernel "$CAM_KERNEL" -initrd "$CAM_ROOTFS" \
        -append "console=ttyAMA0,115200 mem=32M mmz_allocator=hisi root=/dev/ram0 rootfstype=squashfs"
fi
CAM=$!

cleanup() { kill "$NVR" "$CAM" "$DNSMASQ" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

cat <<EOF

  NVR + camera lab is up (namespace-private, 192.168.50.0/24).
    work dir     : $WORK
    NVR  console : socat - UNIX-CONNECT:$WORK/nvr.sock
    cam  console : socat - UNIX-CONNECT:$WORK/cam.sock
    cam  QMP     : $WORK/cam-qmp.sock   (motion: qom-set /machine/scene light)
    DHCP leases  : $WORK/leases
    capture Notify once the camera has a password + onvif.password:
      python3 $HERE/onvif-notify-probe.py --host <cam-ip> \\
          --user admin --password <pw> --seconds 20 --out $WORK/notify

  Dropping you into a shell inside the lab namespace. Exit it to tear down.
EOF

# Run the shell as a child, not exec, so control returns here and the EXIT
# trap tears the guests and dnsmasq down when the shell is left.
"${SHELL:-/bin/sh}"
