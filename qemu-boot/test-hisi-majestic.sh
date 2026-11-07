#!/bin/bash
# majestic's video on an emulated HiSilicon V4 camera, end to end.
#
# Boots the OpenIPC release (kernel, squashfs root) the way U-Boot would
# start it (mem= and the allocator, load_hisilicon / load_goke sizing MMZ
# itself) with a 1080p sensor on the bus, and leaves majestic to start on
# its own.  The sensor has to be one the release has a config for in
# /etc/sensors: SC2315E for HiSilicon's releases, SC2335 for Goke's (which
# lack the SC2315E; majestic then stops at "Cannot start SDK", in syslog).
# Its pipeline is the vendor's, unmodified: sensor and ISP, VI online into
# VPSS (hisi-vi, hisi-vpss: the frame clock and the pictures), VENC on
# VEDU (hisi-vedu).  hisi-vedu keeps the stream it encodes
# (-global hisi-vedu.dump-dir) and the host checks it: ffmpeg decodes it
# with no errors, at least --min-frames pictures, each carrying hisi-vpss's
# frame counter, rising from picture to picture (a stale buffer fails).
#
# -icount shift=0: guest time follows the guest's instructions, so its
# interrupt handlers always finish within a frame of VI's (without it, a
# slow host lets frames overtake the driver and the counter can repeat).
#
#   test-hisi-majestic.sh [--soc hi3516ev200|hi3516ev300|gk7205v200|...]
#                         [--sensor NAME] [--qemu Q] [--work DIR]
#                         [--timeout SECONDS] [--min-frames N]
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
SOC=hi3516ev200
QEMU=$HERE/../qemu-src/build/qemu-system-arm
WORK=
TIMEOUT=420
MIN_FRAMES=50
SENSOR=

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --work) WORK=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    --min-frames) MIN_FRAMES=$2; shift ;;
    --sensor) SENSOR=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
case $SOC in
gk*) ALLOC=gk; SENSOR=${SENSOR:-sc2335} ;;
*) ALLOC=hisi; SENSOR=${SENSOR:-sc2315e} ;;
esac
WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)

if [ ! -f "$WORK/uImage.$SOC" ]; then
    bash "$HERE/fetch-openipc-lite.sh" "$SOC" "$WORK"
fi
DUMP=$WORK/dump
rm -rf "$DUMP"
mkdir -p "$DUMP"

# QEMU is stopped by the timeout: majestic streams until then
timeout "$TIMEOUT" "$QEMU" -M "$SOC,sensor=$SENSOR" \
    -kernel "$WORK/uImage.$SOC" -initrd "$WORK/rootfs.squashfs.$SOC" \
    -append "console=ttyAMA0,115200 mem=32M mmz_allocator=$ALLOC root=/dev/ram0 rootfstype=squashfs" \
    -icount shift=0 -global hisi-vedu.dump-dir="$DUMP" \
    -nographic -serial mon:stdio -nic user,restrict=on -no-reboot \
    > "$WORK/console.log" 2>&1 || true

if grep -qE "Oops|Kernel panic|nobody cared" "$WORK/console.log"; then
    grep -E -A3 "Oops|Kernel panic|nobody cared" "$WORK/console.log" | head -20
    echo "kernel trouble; console in $WORK/console.log" >&2
    exit 1
fi
if ! grep -q "Starting majestic: OK" "$WORK/console.log"; then
    echo "majestic did not start; console in $WORK/console.log" >&2
    exit 1
fi
python3 "$HERE/check-hisi-venc.py" --min-frames "$MIN_FRAMES" --rising \
    "$DUMP" h265
