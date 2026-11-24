#!/bin/bash
# A scenario in front of an emulated Grain Media GM8136 camera, and how its
# 3A loop answers (docs/grainmedia.md, "The digital twin").
#
# Boots the OpenIPC gm8136 release from NOR as test-grainmedia-boot.sh
# does, with an OV2715 on the ISP's I2C bus (sensor=ov2715):
# load_grainmedia finds it, loads fisp328.ko, fisp_algorithm.ko -- the
# vendor's in-kernel AE and AWB -- and fisp_ov2715.ko, and majestic
# streams.  hisi-scene plays a timeline (-global hisi-scene.timeline=) and
# gm-isp records every measured frame (-global gm-isp.record=).  The
# default scenario:
#
#    0 s   the room as it is, AE and AWB settle from the ISP's start
#   20 s   the light drops to a quarter: AE must open up 4x
#   35 s   back to full
#   50 s   a warmer light: AWB moves its gains
#   65 s   end
#
# (seconds of the scene's clock, from the first frame the ISP measures:
# gm-isp measures the scene while the driver has the ISP on).
#
# The vendor AWB keeps its gains inside the OV2715's calibrated colour
# range (isp328_ov2715.cfg [AWB]), so the room is lit at a grey it reaches:
# R 0.90 / B 0.81 of G (light-r 902, light-b 808), where it settles at R
# 1.08 / B 1.13.  The SUN_LIGHT preset's grey (R 164 / B 190 of 128) left
# it 10-17 % short.  The warm light (R 1.0 / B 0.76) moves along that
# range: R's gain falls, B's rises.
#
# AE stops within its dead band, converge_th 10 around target_y 110 in
# the AE pre-gamma's domain -- about +-20 % in linear light -- so the
# levels are held to 25 % (--tol), not twin-metrics' default 15 %.
#
# QEMU is stopped when the record passes the end; twin-metrics.py then
# checks each segment: AE settles within --settle-max seconds at the first
# segment's level, and white balance leaves a grey neutral.
#
#   test-grainmedia-scenario.sh [--qemu Q] [--work DIR] [--timeline FILE]
#                               [--end SECONDS] [--timeout SECONDS]
#                               [--settle-max SECONDS] [--late-latch N]
#
# --late-latch N: the sensor's Nth register latch lands a frame late.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
QEMU=$HERE/../qemu-src/build/qemu-system-arm
WORK=
TIMELINE=
END=65
TIMEOUT=900
SETTLE=8
LATE=0
SOC=gm8136

while [ $# -gt 0 ]; do
    case $1 in
    --qemu) QEMU=$2; shift ;;
    --work) WORK=$2; shift ;;
    --timeline) TIMELINE=$2; shift ;;
    --end) END=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    --settle-max) SETTLE=$2; shift ;;
    --late-latch) LATE=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)

if [ ! -f "$WORK/uImage.$SOC" ]; then
    bash "$HERE/fetch-openipc-lite.sh" "$SOC" "$WORK"
fi
if [ -z "$TIMELINE" ]; then
    TIMELINE=$WORK/timeline
    cat > "$TIMELINE" <<EOF
# the light drops to a quarter, comes back, then turns warmer
20 light=150
35 light=600
50 light-r=1000 light-b=760
EOF
fi
RECORD=$WORK/record.csv
IMG=$WORK/flash.bin
rm -f "$RECORD" "$IMG"

# the boot test's layout: kernel at 0x50000, root at 0x250000
head -c $((8 << 20)) /dev/zero | tr '\0' '\377' > "$IMG"
dd if="$WORK/uImage.$SOC" of="$IMG" bs=64k seek=5 conv=notrunc status=none
dd if="$WORK/rootfs.squashfs.$SOC" of="$IMG" bs=64k seek=37 conv=notrunc \
    status=none
APPEND="mem=128M gmmem=90M console=ttyS0,115200 panic=20 root=/dev/mtdblock3"
APPEND="$APPEND rootfstype=squashfs init=/init"
APPEND="$APPEND mtdparts=nor-flash:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)"

"$QEMU" -M "$SOC,flash-file=$IMG,sensor=ov2715" \
    -kernel "$WORK/uImage.$SOC" -append "$APPEND" \
    -icount shift=0 -global hisi-scene.timeline="$TIMELINE" \
    -global hisi-scene.light-r=902 -global hisi-scene.light-b=808 \
    -global gm-isp.record="$RECORD" \
    -global ov2710-sensor.late-latch="$LATE" \
    -display none -monitor none -serial "file:$WORK/console.log" \
    -nic user,restrict=on -no-reboot </dev/null &
QPID=$!
trap 'kill $QPID 2>/dev/null || true' EXIT

# until the scene's clock passes the end, or the timeout
deadline=$((SECONDS + TIMEOUT))
while kill -0 $QPID 2>/dev/null && [ $SECONDS -lt $deadline ]; do
    t=$(tail -n 1 "$RECORD" 2>/dev/null | cut -d, -f1)
    if [ -n "$t" ] && [ "$t" != t ] &&
       awk -v t="$t" -v e="$END" 'BEGIN { exit !(t >= e) }'; then
        break
    fi
    sleep 5
done
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true

if grep -qE "Oops|Kernel panic|nobody cared" "$WORK/console.log"; then
    grep -E -A3 "Oops|Kernel panic|nobody cared" "$WORK/console.log" | head -20
    exit 1
fi
if ! grep -q "Video Front End: ov2715" "$WORK/console.log"; then
    echo "load_grainmedia did not take the OV2715;" \
         "console in $WORK/console.log" >&2
    exit 1
fi
t=$(tail -n 1 "$RECORD" 2>/dev/null | cut -d, -f1)
if [ -z "$t" ] || ! awk -v t="$t" -v e="$END" 'BEGIN { exit !(t >= e) }'; then
    echo "the scenario did not reach ${END}s (record at ${t:-nothing});" \
         "console in $WORK/console.log" >&2
    exit 1
fi
python3 "$HERE/twin-metrics.py" "$RECORD" --timeline "$TIMELINE" \
    --settle-max "$SETTLE" --tol 0.25
