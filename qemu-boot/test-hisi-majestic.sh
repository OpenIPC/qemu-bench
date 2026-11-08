#!/bin/bash
# majestic's video on an emulated HiSilicon V4 camera, end to end.
#
# Boots the OpenIPC release (kernel, squashfs root) the way U-Boot would
# start it (mem= and the allocator, load_hisilicon / load_goke sizing MMZ
# itself) with a sensor on the bus (1080p by default), and leaves majestic to start on
# its own.  The sensor has to be one the release has a config for in
# /etc/sensors: SC2315E for HiSilicon's releases, SC2335 for Goke's (which
# lack the SC2315E; majestic then stops at "Cannot start SDK", in syslog).
# Its pipeline is the vendor's, unmodified: sensor and ISP, VI online into
# VPSS (hisi-vi, hisi-vpss: the frame clock and the pictures), or for a
# sensor wider than 2304 (--sensor imx335 / gc4653 on hi3516ev300) VI
# offline and VPSS running the driver's node list, VENC on
# VEDU (hisi-vedu).  hisi-vedu keeps the stream it encodes
# (-global hisi-vedu.dump-dir) and the host checks it: ffmpeg decodes it
# with no errors, at least --min-frames pictures, each carrying hisi-vpss's
# frame counter, rising from picture to picture (a stale buffer fails).
#
# The ISP's statistics follow the sensor's exposure (hisi-vi): with a
# sensor model that reports it, auto exposure must have settled the scene
# between 10 % and 70 % of full scale (pinned at its longest exposure and
# highest gain, it reads 0 or 65535), and where the white balance gains
# are visible (online) they must bring the light's R and B within 15 % of G.
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
    -global hisi-vi.stats-trace=on \
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
case $SENSOR in
sc*|gc*)        # SmartSens / GalaxyCore models report their exposure
    last=$(grep -a "^isp stats:" "$WORK/console.log" | tail -1)
    mean=$(echo "$last" | sed -n 's/.*G mean \([0-9]*\).*/\1/p')
    echo "AE: $last"
    if [ -z "$mean" ] || [ "$mean" -lt 6553 ] || [ "$mean" -gt 45875 ]; then
        echo "auto exposure did not settle; console in $WORK/console.log" >&2
        exit 1
    fi
    # AWB, where the driver programs the gains through MMIO (online): the
    # light's R and B, times its gains, within 15 % of G
    if ! echo "$last" | awk '
        function hex(s,   n, i) {            # POSIX awk has no strtonum
            n = 0; s = tolower(s); sub(/^0x/, "", s)
            for (i = 1; i <= length(s); i++)
                n = n * 16 + index("0123456789abcdef", substr(s, i, 1)) - 1
            return n
        }
        { for (i = 1; i <= NF; i++) {
            if ($i == "R" && $(i + 1) ~ /^[0-9]+$/) r = $(i + 1)
            if ($i == "B" && $(i + 1) ~ /^[0-9]+,?$/) b = $(i + 1) + 0
            if ($i == "mean") g = $(i + 1) + 0
            if ($i == "gains") {
                seen = 1; gr = hex($(i + 2)); gb = hex($(i + 4))
            }
        }
        if (!seen) exit 0           # offline: the gains are not visible here
        if (!gr || !gb || !g) {
            print "AWB: no gains programmed"
            exit 1
        }
        er = r * gr / 256 / g - 1; eb = b * gb / 256 / g - 1
        printf "AWB: R x%.2f B x%.2f -> %+.0f%% / %+.0f%% of G\n",
               gr / 256, gb / 256, er * 100, eb * 100
        exit (er < -0.15 || er > 0.15 || eb < -0.15 || eb > 0.15) }'; then
        echo "auto white balance did not settle; console in $WORK/console.log" >&2
        exit 1
    fi
    ;;
esac
python3 "$HERE/check-hisi-venc.py" --min-frames "$MIN_FRAMES" --rising \
    "$DUMP" h265
