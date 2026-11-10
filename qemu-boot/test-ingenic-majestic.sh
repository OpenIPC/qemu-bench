#!/bin/bash
# majestic on an emulated Ingenic T40N camera with a GC4653: a digital
# twin for reproducing what a T40 owner reports (docs/ingenic.md,
# "majestic").
#
# The OpenIPC t40 lite release's kernel and root, the root rebuilt with an
# init of our own: the release's load_ingenic loads the vendor modules
# (tx-isp-t40, the GC4653 driver, audio.ko, avpu) as on the camera, then
# majestic runs with a majestic.yaml written here.  --majestic puts another
# binary in the release's place, a local build to try a fix on.
#
# What is checked, from the console and from QEMU:
#   - majestic brings the SDK up ("Ingenic SDK started");
#   - with --audio, with the microphone on (audio.enabled: true), which a
#     T40 build could not do before OpenIPC/majestic#257;
#   - day/night from a light sensor: PB17 is nightMode.lightSensorPin, and
#     the host pulls it low (night) and lets it go (day) through
#     qom-set /machine/gpio.  majestic must change mode both ways, and the
#     switch to night must reach the ISP: ingenic-isp.trace-regs logs
#     every ISP core write, and the night window must write at least
#     --min-night-regs registers the day before it did not (the night
#     tuning set: about 980 on this release, against 8 to 15 -- the
#     per-frame curves' drift alone -- for a switch that never reached the
#     ISP, as in OpenIPC/majestic#258).
#
# One thing the camera does not need: a seed for the kernel's random pool.
# The machine gathers almost no entropy, and getrandom() would hold
# majestic before it reads its config for good; the release's busybox
# seedrng credits one.  libimp's MXU2 paths are off (/tmp/closesimd), as
# in the other libimp tests here.
#
#   test-ingenic-majestic.sh [--majestic BIN] [--audio] [--work DIR]
#                            [--qemu Q] [--timeout SECONDS]
#                            [--min-night-regs N] [--keep]
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
QEMU=$HERE/../qemu-src/build/qemu-system-mipsel
SOC=t40
WORK=
BIN=
AUDIO=false
TIMEOUT=360
MIN_NIGHT=100
KEEP=

while [ $# -gt 0 ]; do
    case $1 in
    --majestic) BIN=$2; shift ;;
    --audio) AUDIO=true ;;
    --work) WORK=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    --min-night-regs) MIN_NIGHT=$2; shift ;;
    --keep) KEEP=1 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)
[ -f "$WORK/uImage.$SOC" ] || bash "$HERE/fetch-openipc-lite.sh" "$SOC" "$WORK"

RFS=$WORK/rfs
rm -rf "$RFS"
unsquashfs -q -d "$RFS" "$WORK/rootfs.squashfs.$SOC" >/dev/null
if [ -n "$BIN" ]; then
    cp "$BIN" "$RFS/usr/bin/majestic"
    chmod 755 "$RFS/usr/bin/majestic"
fi
cat > "$RFS/etc/majestic.yaml" <<EOF
system:
  logLevel: debug
video0:
  enabled: true
  codec: h264
video1:
  enabled: false
jpeg:
  enabled: false
audio:
  enabled: $AUDIO
  outputEnabled: false
nightMode:
  colorToGray: true
  irCutPin1: 122
  irCutPin2: 123
  lightMonitor: true
  lightSensorPin: 49
EOF
# fw_printenv writes this on first use, but the root is read-only here
echo "/dev/mtd1 0x0 0x10000 0x8000" > "$RFS/etc/fw_env.config"
head -c 512 /dev/urandom > "$RFS/etc/twin-seed"
cat > "$RFS/usr/bin/twin-init" <<'EOF'
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t tmpfs tmp /var
mkdir -p /dev/shm /var/lock /var/run && mount -t tmpfs shm /dev/shm
ifconfig lo 127.0.0.1 up
mkdir -p /var/lib/seedrng && cp /etc/twin-seed /var/lib/seedrng/seed.credit
seedrng -d /var/lib/seedrng
load_ingenic -i
touch /tmp/closesimd
echo "TWIN modules: $(lsmod | cut -d' ' -f1 | tr '\n' ' ')"
exec /usr/bin/majestic
EOF
chmod 755 "$RFS/usr/bin/twin-init"
mksquashfs "$RFS" "$WORK/root.sq" -comp xz -noappend -quiet -all-root >/dev/null
[ -n "$KEEP" ] || rm -rf "$RFS"

# OpenIPC's NOR layout with a U-Boot environment that names the sensor
python3 - "$WORK/nor.bin" "$WORK/uImage.$SOC" "$WORK/root.sq" \
    sensor=gc4653 soc=$SOC <<'PY'
import struct, sys, zlib
img = bytearray(b'\xff' * (16 << 20))
for path, off in ((sys.argv[2], 0x50000), (sys.argv[3], 0x250000)):
    data = open(path, 'rb').read()
    assert len(data) <= 0x800000, path + ' does not fit'
    img[off:off + len(data)] = data
env = b''.join(kv.encode() + b'\0' for kv in sys.argv[4:]) + b'\0'
env += b'\0' * (0x10000 - 4 - len(env))
img[0x40000:0x50000] = struct.pack('<I', zlib.crc32(env)) + env
open(sys.argv[1], 'wb').write(img)
PY

CON=$WORK/console.log
TRACE=$WORK/isp-trace.log
MON=$(mktemp -u /tmp/twin.XXXXXX.sock)     # sun_path is 108 bytes
: > "$CON"
: > "$TRACE"
"$QEMU" -M "$SOC,variant=t40n,sensor=gc4653,flash-file=$WORK/nor.bin" -m 128M \
    -global ingenic-isp.trace-regs=on -display none -serial file:"$CON" \
    -monitor unix:"$MON",server,nowait -nic user,restrict=on \
    -kernel "$WORK/uImage.$SOC" \
    -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/usr/bin/twin-init mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data) mem=64M rmem=64M@0x4000000" \
    2>>"$TRACE" &
QPID=$!
trap 'kill $QPID 2>/dev/null; rm -f "$MON"' EXIT

mon() {
    python3 - "$MON" "$1" <<'PY'
import socket, sys, time
s = socket.socket(socket.AF_UNIX)
s.connect(sys.argv[1])
time.sleep(0.2)
s.recv(65536)
s.sendall(sys.argv[2].encode() + b'\n')
time.sleep(0.5)
PY
}
# the windows the trace is split into
mark() { echo "=== TWIN $1" >> "$TRACE"; }
waitfor() {
    for _ in $(seq "$2"); do
        grep -aq -E "$1" "$CON" && return 0
        kill -0 $QPID 2>/dev/null || return 1
        sleep 1
    done
    return 1
}

fail=0
check() {       # check DESCRIPTION COMMAND...
    local what=$1
    shift
    if "$@"; then echo "  PASS: $what"; else echo "  FAIL: $what"; fail=1; fi
}

waitfor 'Ingenic SDK started|Cannot start SDK' "$TIMEOUT" || true
echo "$SOC, majestic (audio: $AUDIO):"
check "majestic starts the SDK" grep -aq 'Ingenic SDK started' "$CON"
if grep -aq 'Ingenic SDK started' "$CON"; then
    sleep 20            # the ircut exercise and AE settle
    mark DAY
    sleep 10
    mark NIGHT
    mon "qom-set /machine/gpio pb 0xfffdffff"
    check "the light sensor turns night on" waitfor 'Change mode to NIGHT' 60
    sleep 20
    mark BACK
    mon "qom-set /machine/gpio pb 0xffffffff"
    check "and day again" waitfor 'Change mode to DAY' 60
    sleep 5
    mark END
    n=$(python3 - "$TRACE" <<'PY'
import collections, re, sys
rx = re.compile(r'ingenic-isp: core \[0x([0-9a-f]+)\] = 0x([0-9a-f]+)')
seen = collections.defaultdict(lambda: collections.defaultdict(set))
win = None
for line in open(sys.argv[1], errors='replace'):
    if line.startswith('=== TWIN '):
        win = line.split()[2]
        continue
    m = rx.search(line)
    if m and win:
        seen[win][m.group(1)].add(m.group(2))
day, night = seen['DAY'], seen['NIGHT']
print(sum(1 for r, v in night.items() if not v <= day.get(r, set())))
PY
)
    echo "  ($n ISP registers written at night that the day did not)"
    check "night reaches the ISP" [ "$n" -ge "$MIN_NIGHT" ]
fi
if grep -aq -E 'IMP_[A-Za-z_]+\(\) failed|Cannot start SDK' "$CON"; then
    grep -a -E 'IMP_[A-Za-z_]+\(\) failed|Cannot start SDK' "$CON" | sed 's/\x1b\[[0-9;]*m//g' | head -5
fi
if grep -aq -E 'Oops|Kernel panic' "$CON"; then
    echo "  FAIL: kernel trouble"
    fail=1
fi
[ $fail = 0 ] || echo "console in $CON, ISP trace in $TRACE"
exit $fail
