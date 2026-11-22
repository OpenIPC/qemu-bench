#!/bin/bash
# majestic's dimmable lamp on an emulated HiSilicon camera, end to end.
#
# Boots the OpenIPC release with majestic replaced by --majestic BIN (a build
# for that SoC's SDK), configured with an IR lamp on a PWM channel
# (nightMode.irLightPwmChannel, 400 Hz, 25..100 %), and claimed (root's
# password set: an unclaimed camera answers nothing but the claim flow).
# Then, over HTTP, the way an operator would:
#
#   /night/on      the lamp comes up at full brightness: the channel runs at
#                  400 Hz (a 2.5 ms period) with the pad high the whole period
#   /night/duty/50 held at half: the high time strictly inside the period
#   /night/off     the lamp is dark: on an active-high lamp, the block is off
#
# and each is read where it lands, from hisi-pwm's output over QMP
# (chN-enabled / chN-period-ns / chN-high-ns), so what is checked is the
# register state the silicon would drive the pad from -- after hisi-pwm has
# applied the clock gate, the source select, the latch rules and V5's
# minus-one encoding to whatever majestic wrote.  A block majestic forgot to
# ungate reads 0 here as it does on silicon, and its "did not take its
# period" line is in the console.
#
#   test-hisi-majestic-pwm.sh --soc hi3516cv610 --majestic BIN
#                             [--channel pwmN] [--sensor NAME] [--qemu Q]
#                             [--work DIR] [--timeout SECONDS]
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
SOC=
BIN=
CHANNEL=
SENSOR=
QEMU=$HERE/../qemu-src/build/qemu-system-arm
WORK=
TIMEOUT=600

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --majestic) BIN=$(realpath "$2"); shift ;;
    --channel) CHANNEL=$2; shift ;;
    --sensor) SENSOR=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --work) WORK=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
[ -n "$SOC" ] && [ -n "$BIN" ] || {
    echo "usage: $0 --soc SOC --majestic BIN [--channel pwmN]" >&2; exit 2; }

# Per SoC: the release, the kernel's memory arguments, and a channel whose pad
# ipctool knows -- the one a field camera of that family uses where there is
# one (a CV613's IR lamp on PWM1, OpenIPC/majestic#324; pwm3 on pad 16 on the
# EV300 family), and on av100 one in the second register bank.
REL=$SOC
MEM=
DEF=
case $SOC in
hi3516cv6*) REL=hi3516cv6xx; MEM="mem=32M mmz_allocator=hisi"
            SENSOR=${SENSOR:-sc4336p}; DEF=pwm1 ;;
hi3516ev*|hi3518ev300|hi3516dv200)
            MEM="mem=32M mmz_allocator=hisi"; SENSOR=${SENSOR:-sc2315e}
            DEF=pwm3 ;;
gk7205v2*|gk7205v3*|gk7202v3*)
            MEM="mem=32M mmz_allocator=gk"; SENSOR=${SENSOR:-sc2335}
            DEF=pwm3 ;;
hi3516cv500|hi3516av300|hi3516dv300) DEF=pwm1 ;;
hi3516av100|hi3516dv100) DEF=pwm5 ;;
hi3519v101|hi3516av200) DEF=pwm4 ;;
hi3516cv300|hi3516ev100) DEF=pwm0 ;;
hi3516cv200|hi3518ev200) DEF=pwm1 ;;
hi3516cv100|hi3518ev100) DEF=pwm1 ;;
*) echo "no PWM twin for $SOC" >&2; exit 2 ;;
esac
CHANNEL=${CHANNEL:-$DEF}
N=${CHANNEL#pwm}
N=${N%%@*}

WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)
if [ ! -f "$WORK/rootfs.squashfs.$REL" ]; then
    bash "$HERE/fetch-openipc-lite.sh" "$REL" "$WORK"
fi
KERNEL=$WORK/uImage.$REL
[ -f "$KERNEL" ] || KERNEL=$WORK/kernel.$REL

RFS=$WORK/rfs
rm -rf "$RFS"
unsquashfs -q -d "$RFS" "$WORK/rootfs.squashfs.$REL" >/dev/null
cp "$BIN" "$RFS/usr/bin/majestic"
chmod 755 "$RFS/usr/bin/majestic"
python3 - "$RFS/etc/majestic.yaml" "$CHANNEL" <<'PY'
import sys, yaml
path, channel = sys.argv[1:]
try:                      # the releases ship none: majestic has defaults
    cfg = yaml.safe_load(open(path)) or {}
except FileNotFoundError:
    cfg = {}
cfg.setdefault('system', {})['logLevel'] = 'debug'
night = cfg.setdefault('nightMode', {})
night.update({'irLightPwmChannel': channel, 'backlightPwmFreq': 400,
              'backlightPwmMin': 25, 'backlightPwmMax': 100})
yaml.safe_dump(cfg, open(path, 'w'), sort_keys=False)
PY
# claimed: an unclaimed camera streams nothing and answers 401
sed -i "s|^root:[^:]*:|root:$(openssl passwd -1 12345):|" "$RFS/etc/shadow"
# majestic logs to syslog; the lamp's lines are copied to the console
sed -i 's#^exit 0$#logread -f | grep -E "PWM|lamp|night|Cannot start SDK" > /dev/console 2>\&1 \&\nexit 0#' "$RFS/etc/rc.local"
grep -q '^logread -f' "$RFS/etc/rc.local" || {
    echo "rc.local: no exit 0 to insert before" >&2; exit 1; }
mksquashfs "$RFS" "$WORK/root.sq" -comp xz -noappend -quiet -all-root >/dev/null
rm -rf "$RFS"

QMP=$(mktemp -u /tmp/twin-pwm.XXXXXX.sock)     # sun_path is 108 bytes
HTTP=$(python3 -c 'import socket; s = socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1])')
CON=$WORK/console.log
: > "$CON"
MACH=$SOC
[ -z "$SENSOR" ] || MACH=$SOC,sensor=$SENSOR
timeout "$TIMEOUT" "$QEMU" -M "$MACH" \
    -kernel "$KERNEL" -initrd "$WORK/root.sq" \
    -append "console=ttyAMA0,115200 $MEM root=/dev/ram0 rootfstype=squashfs mtdparts=hi_sfc:256k(boot),64k(env),3072k(kernel),10240k(rootfs),-(rootfs_data)" \
    -icount shift=0 -display none -serial file:"$CON" \
    -qmp unix:"$QMP",server=on,wait=off \
    -nic user,restrict=on,hostfwd=tcp:127.0.0.1:$HTTP-:80 -no-reboot \
    </dev/null >"$WORK/qemu.log" 2>&1 &
QPID=$!
trap 'kill $QPID 2>/dev/null; rm -f "$QMP"' EXIT

fail() {
    echo "FAIL: $*" >&2
    grep -a -E "PWM|lamp|Cannot start SDK" "$CON" | tail -8 >&2 || true
    echo "console in $CON" >&2
    exit 1
}
waitfor() {
    for _ in $(seq "$2"); do
        grep -aq -E "$1" "$CON" 2>/dev/null && return 0
        kill -0 $QPID 2>/dev/null || return 1
        sleep 1
    done
    return 1
}
qom() {
    python3 - "$QMP" "/machine/pwm" "$1" <<'PY'
import json, socket, sys
s = socket.socket(socket.AF_UNIX); s.connect(sys.argv[1])
f = s.makefile('rw', buffering=1)
f.readline()
for cmd in ({'execute': 'qmp_capabilities'},
            {'execute': 'qom-get', 'arguments': {'path': sys.argv[2],
                                                 'property': sys.argv[3]}}):
    f.write(json.dumps(cmd) + '\n')
    while True:
        r = json.loads(f.readline())
        if 'return' in r or 'error' in r:
            break
print(json.dumps(r.get('return', r.get('error'))))
PY
}
get() {
    curl -s -m 30 -u root:12345 -o /dev/null -w '%{http_code}' \
        "http://127.0.0.1:$HTTP$1"
}

waitfor "Starting majestic: OK" "$TIMEOUT" || fail "majestic did not start"
# the web server, once majestic is past its start
for _ in $(seq 120); do
    [ "$(get /night/off)" = 200 ] && break
    kill -0 $QPID 2>/dev/null || fail "QEMU exited"
    sleep 2
done
[ "$(get /night/off)" = 200 ] || fail "majestic's web server never answered"
waitfor "PWM IR lamp on pad" 60 || fail "the PWM lamp did not come up"
grep -a "PWM IR lamp on pad" "$CON" | tail -1

# within 1 % of 2.5 ms: the clock divides 400 Hz exactly on every family
PERIOD=2500000
near() { python3 -c "import sys; a, b = int(sys.argv[1]), int(sys.argv[2]); sys.exit(abs(a - b) > b // 100)" "$1" "$2"; }
state() {
    EN=$(qom ch$N-enabled); P=$(qom ch$N-period-ns); H=$(qom ch$N-high-ns)
    echo "    ch$N enabled=$EN period=${P}ns high=${H}ns"
}

echo "/night/on"
[ "$(get /night/on)" = 200 ] || fail "/night/on"
sleep 3
state
[ "$EN" = true ] || fail "ch$N is not running at night"
near "$P" $PERIOD || fail "ch$N period $P ns, want $PERIOD"
# full brightness on the switch, which the controller may already be
# walking down towards what the scene needs -- never below backlightPwmMin
python3 -c "import sys; h, p = int(sys.argv[1]), int(sys.argv[2]); sys.exit(not (p // 5 <= h <= p))" "$H" "$P" ||
    fail "ch$N at night: high $H ns of $P, want between 25 % and 100 %"

echo "/night/duty/50"
[ "$(get /night/duty/50)" = 200 ] || fail "/night/duty/50"
sleep 3
state
[ "$EN" = true ] || fail "ch$N stopped at 50 %"
near "$P" $PERIOD || fail "ch$N period $P ns at 50 %"
python3 -c "import sys; h, p = int(sys.argv[1]), int(sys.argv[2]); sys.exit(not (p // 10 < h < p * 9 // 10))" "$H" "$P" ||
    fail "ch$N at 50 %: high $H ns of $P"

echo "/night/duty/auto, /night/off"
get /night/duty/auto >/dev/null
[ "$(get /night/off)" = 200 ] || fail "/night/off"
sleep 3
state
[ "$EN" = false ] || fail "ch$N still running in the day (the lamp is lit)"

echo "PASS: $SOC $CHANNEL"
