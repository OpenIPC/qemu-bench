#!/bin/bash
# OpenIPC's stepper motor driver on T31: steps are paced by TCU channel 2's
# full interrupt (motor.ko's jz_timer_interrupt), so a move only happens
# if the TCU raises it on time.
#
# Boots the OpenIPC T31 kernel and root (fetch-openipc-lite.sh layout in
# --fw), loads its motor.ko with four phase GPIOs per motor, moves the
# horizontal and vertical motors with ingenic-motor and checks the
# positions the driver reports, the TCU interrupt count in
# /proc/interrupts and the phase pins' levels moving.
#
#   bash qemu-boot/test-ingenic-motor.sh [--fw DIR] [--qemu PATH] [--out DIR]
#
# Copyright (c) 2026 OpenIPC.
# SPDX-License-Identifier: GPL-2.0-or-later

set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
FW=/home/ai/dimerr/local/ingenic/fw/t31
OUT=
TIMEOUT=300

while [ $# -gt 0 ]; do
    case $1 in
    --fw) FW=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
for f in "$FW/uImage.t31" "$FW/rootfs.squashfs.t31" "$QEMU"; do
    [ -e "$f" ] || { echo "SKIP: missing $f"; exit 0; }
done
WORK=$(mktemp -d)
trap 'kill $QPID 2>/dev/null || true; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.t31" >/dev/null
KO=$(cd "$WORK/rfs" && ls lib/modules/*/ingenic/motor.ko | head -1)
[ -n "$KO" ] || { echo "SKIP: no motor.ko in the root"; exit 0; }
# phase pins PC0..PC3 (horizontal) and PC4..PC7 (vertical); no limit
# switches.  The GPIO block's levels for port C are read through devmem.
cat > "$WORK/rfs/usr/bin/motor-init" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t devtmpfs dev /dev 2>/dev/null
echo 1 > /proc/sys/kernel/printk
insmod /$KO hst1=64 hst2=65 hst3=66 hst4=67 vst1=68 vst2=69 vst3=70 vst4=71 \\
    hmin=-1 hmax=-1 vmin=-1 vmax=-1 hmaxstep=2000 vmaxstep=600
irqs() { grep -i "tcu\|jz-tcu\|motor" /proc/interrupts | tr -s ' ' | tr '\n' ';'; }
echo "MOTORTEST irqs-before: \$(irqs)"
pins=""
for i in 1 2 3 4 5 6 7 8; do pins="\$pins \$(devmem 0x10012000 32)"; usleep 3000 2>/dev/null || sleep 1; done
ingenic-motor -d g -x 120 -y 40 -s 900 >/dev/null 2>&1 &
for i in 1 2 3 4 5 6 7 8 9 10 11 12; do pins="\$pins \$(devmem 0x10012000 32)"; usleep 20000 2>/dev/null || sleep 1; done
sleep 4
echo "MOTORTEST pins:\$pins"
echo "MOTORTEST pos: \$(ingenic-motor -j 2>&1 | tr -d '\n')"
echo "MOTORTEST irqs-after: \$(irqs)"
echo MOTORTEST-END
# the UART sends at the line rate: let the console drain (the host stops
# QEMU at the end marker)
sleep 60
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/motor-init"
mksquashfs "$WORK/rfs" "$WORK/rootfs.squashfs" -comp xz -noappend -quiet \
    -all-root >/dev/null
python3 - "$WORK/nor.bin" "$FW/uImage.t31" "$WORK/rootfs.squashfs" <<'PY'
import sys
img = bytearray(b'\xff' * (16 << 20))
for path, off in ((sys.argv[2], 0x50000), (sys.argv[3], 0x250000)):
    data = open(path, 'rb').read()
    img[off:off + len(data)] = data
open(sys.argv[1], 'wb').write(img)
PY

echo "== boot (t31)"
"$QEMU" -M t31,flash-file="$WORK/nor.bin" -m 64M \
    -display none -serial file:"$OUT/console.log" -nic none \
    -kernel "$FW/uImage.t31" \
    -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/usr/bin/motor-init mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data) mem=40M rmem=24M@0x2800000" &
QPID=$!
for i in $(seq "$TIMEOUT"); do
    grep -q MOTORTEST-END "$OUT/console.log" 2>/dev/null && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
sleep 1
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true
grep -a "MOTORTEST\|motor" "$OUT/console.log" | tr -d '\r' || true

echo "== host"
python3 - "$OUT/console.log" <<'PY'
import json, re, sys
log = open(sys.argv[1], 'rb').read().decode('latin-1').replace('\r', '')
fail = 0

def check(what, ok, detail=''):
    global fail
    print('  %s: %s%s' % ('PASS' if ok else 'FAIL', what,
                          ' (%s)' % detail if detail else ''))
    fail |= not ok

def line(tag):
    m = re.search(r'^MOTORTEST %s:(.*)$' % tag, log, re.M)
    return m.group(1).strip() if m else ''

pos = line('pos')
m = re.search(r'\{.*\}', pos)
try:
    j = json.loads(m.group(0)) if m else {}
except ValueError:
    j = {}
x, y = j.get('xpos'), j.get('ypos')
check('the driver reports the motors moved to (120, 40)',
      str(x) == '120' and str(y) == '40', pos)

def count(s):
    return sum(int(n) for n in re.findall(r'^\s*\d+:\s*(\d+)', s.replace(';', '\n'), re.M))

before, after = line('irqs-before'), line('irqs-after')
nb, na = count(before), count(after)
check('the TCU interrupt counted the steps', na - nb >= 120, '%d -> %d' % (nb, na))
pins = [int(v, 16) & 0xff for v in line('pins').split() if v.startswith('0x')]
check('the phase pins changed while moving', len(set(pins)) > 2,
      ' '.join('%02x' % p for p in pins))
print('PASS' if not fail else 'FAIL')
sys.exit(fail)
PY
