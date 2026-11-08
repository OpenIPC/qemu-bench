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
#   50 s   a warmer light (R 0.82 / B 0.52 of G): AWB moves its gains
#   65 s   end
#
# (seconds of the scene's clock, from VI's first frame).  QEMU is stopped
# when the record passes the end; twin-metrics.py then checks each
# segment, and that every change the timeline plans shows in the record:
# AE settles within --settle-max seconds at the first segment's level;
# white balance (how neutral it leaves a grey) is reported.
#
#   test-hisi-scenario.sh [--soc hi3516ev200] [--sensor NAME] [--qemu Q]
#                         [--work DIR] [--timeline FILE] [--end SECONDS]
#                         [--timeout SECONDS] [--settle-max SECONDS]
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
if [ -z "$TIMELINE" ]; then
    TIMELINE=$WORK/timeline
    cat > "$TIMELINE" <<'EOF'
# the light drops to a quarter, comes back, then turns warmer
20 light=150
35 light=600
50 light-r=820 light-b=520
EOF
fi
RECORD=$WORK/record.csv
rm -f "$RECORD"

"$QEMU" -M "$SOC,sensor=$SENSOR" \
    -kernel "$WORK/uImage.$SOC" -initrd "$WORK/rootfs.squashfs.$SOC" \
    -append "console=ttyAMA0,115200 mem=32M mmz_allocator=$ALLOC root=/dev/ram0 rootfstype=squashfs" \
    -icount shift=0 -global hisi-scene.timeline="$TIMELINE" \
    -global hisi-vi.record="$RECORD" \
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
# White balance is reported, not enforced: in some runs lib_hiawb stops
# moving its gains seconds after boot and then ignores the light's colour
# (OpenIPC/qemu-bench#252).
python3 "$HERE/twin-metrics.py" "$RECORD" --timeline "$TIMELINE" \
    --settle-max "$SETTLE" --awb-advisory
