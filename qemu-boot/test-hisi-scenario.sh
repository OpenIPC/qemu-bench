#!/bin/bash
# A scenario in front of an emulated HiSilicon V4 camera, and how its 3A
# loop answers (docs/hisilicon-digital-twin.md, step 3).
#
# Boots the OpenIPC release as test-hisi-majestic.sh does and leaves
# majestic to stream, while hisi-scene plays a timeline (-global
# hisi-scene.timeline=) and hisi-vi records every measured frame
# (-global hisi-vi.record=).  The default scenario:
#
#    0 s   the room as it is, AE and AWB settle from boot
#   20 s   the light drops to a quarter: AE must open up 4x
#   35 s   back to full
#   50 s   a warmer light (R 0.82 / B 0.52 of G; on cv6xx, whose SC4336P
#          starts at R 0.625 / B 0.565, R 0.70 / B 0.52): AWB moves its gains
#   65 s   end
#
# (seconds of the scene's clock, from VI's first frame).  QEMU is stopped
# when the record passes the end; twin-metrics.py then checks each
# segment, and that every change the timeline plans shows in the record:
# AE settles within --settle-max seconds at the first segment's level,
# and white balance leaves a grey neutral.
#
#   test-hisi-scenario.sh [--soc hi3516ev200] [--sensor NAME] [--qemu Q]
#                         [--work DIR] [--timeline FILE] [--end SECONDS]
#                         [--timeout SECONDS] [--settle-max SECONDS]
#                         [--late-latch N]
#
# --late-latch N: the sensor's Nth register latch lands a frame late
# (its late-latch property), a fault in the boot AE sweep.  Only sensors
# that keep frame timing take it: SmartSens and the 16-bit GalaxyCore
# parts.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
SOC=hi3516ev200
QEMU=$HERE/../qemu-src/build/qemu-system-arm
WORK=
TIMELINE=
END=65
TIMEOUT=900
SETTLE=8
SENSOR=
LATE=0

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --work) WORK=$2; shift ;;
    --timeline) TIMELINE=$2; shift ;;
    --end) END=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    --settle-max) SETTLE=$2; shift ;;
    --sensor) SENSOR=$2; shift ;;
    --late-latch) LATE=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
REL=$SOC
LIGHT=()
WARM="light-r=820 light-b=520"
case $SOC in
gk*) ALLOC=gk; SENSOR=${SENSOR:-sc2335} ;;
hi3516cv6*)
    # one release for cv608 / cv610 / cv613, and an SC4336P: its AWB
    # stays inside its calibrated range, so the room is lit at its grey
    # (R 0.625 / B 0.565) and turns warmer along its colour line
    ALLOC=hisi; SENSOR=${SENSOR:-sc4336p}; REL=hi3516cv6xx
    LIGHT=(-global hisi-scene.light-r=625 -global hisi-scene.light-b=565)
    WARM="light-r=700 light-b=520" ;;
*) ALLOC=hisi; SENSOR=${SENSOR:-sc2315e} ;;
esac
if [ "$LATE" != 0 ]; then
    case $SENSOR in
    sc*|gc2083|gc2093|gc4653|gc4663) ;;
    *) echo "--late-latch: $SENSOR keeps no frame timing" >&2; exit 2 ;;
    esac
fi
WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)

if [ ! -f "$WORK/uImage.$REL" ]; then
    bash "$HERE/fetch-openipc-lite.sh" "$REL" "$WORK"
fi
if [ -z "$TIMELINE" ]; then
    TIMELINE=$WORK/timeline
    cat > "$TIMELINE" <<EOF
# the light drops to a quarter, comes back, then turns warmer
20 light=150
35 light=600
50 $WARM
EOF
fi
RECORD=$WORK/record.csv
rm -f "$RECORD"

"$QEMU" -M "$SOC,sensor=$SENSOR" \
    -kernel "$WORK/uImage.$REL" -initrd "$WORK/rootfs.squashfs.$REL" \
    -append "console=ttyAMA0,115200 mem=32M mmz_allocator=$ALLOC root=/dev/ram0 rootfstype=squashfs" \
    -icount shift=0 -global hisi-scene.timeline="$TIMELINE" "${LIGHT[@]}" \
    -global hisi-vi.record="$RECORD" \
    -global hisi-smartsens.late-latch="$LATE" \
    -global galaxycore-sensor.late-latch="$LATE" \
    -nographic -serial mon:stdio -nic user,restrict=on -no-reboot \
    > "$WORK/console.log" 2>&1 &
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
t=$(tail -n 1 "$RECORD" 2>/dev/null | cut -d, -f1)
if [ -z "$t" ] || ! awk -v t="$t" -v e="$END" 'BEGIN { exit !(t >= e) }'; then
    echo "the scenario did not reach ${END}s (record at ${t:-nothing});" \
         "console in $WORK/console.log" >&2
    exit 1
fi
# White balance is enforced.  A release whose open_isp.ko reads the boot
# copy of the statistics every other frame (OpenIPC/openhisilicon#295)
# freezes lib_hiawb in some runs (OpenIPC/qemu-bench#252).
python3 "$HERE/twin-metrics.py" "$RECORD" --timeline "$TIMELINE" \
    --settle-max "$SETTLE"
