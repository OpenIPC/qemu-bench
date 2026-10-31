#!/bin/bash
# GPIO inputs driven from the host: boots the OpenIPC T31 kernel and root
# (fetch-openipc-lite.sh layout in --fw), exports PC0 through sysfs as an
# input with both-edge interrupts, and pulls it low and high again with
# qom-set /machine/gpio pc over QMP.  The guest must read each level and
# take an interrupt per edge (/proc/interrupts, "GPIO C" and gpiolib).
# It also reads the EFUSE chip ID straight from the fuse array (devmem):
# the OpenIPC kernel builds no EFUSE driver.
#
#   bash qemu-boot/test-ingenic-gpio.sh [--fw DIR] [--qemu PATH] [--out DIR]
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
cat > "$WORK/rfs/usr/bin/gpio-init" <<'EOF'
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t devtmpfs dev /dev 2>/dev/null
echo 1 > /proc/sys/kernel/printk
echo "GPIOTEST chipid: $(devmem 0x13540200 32) $(devmem 0x13540204 32) $(devmem 0x13540208 32)"
G=/sys/class/gpio/gpio64
echo 64 > /sys/class/gpio/export
echo in > $G/direction
echo both > $G/edge
irq() { grep "gpiolib\|GPIO C" /proc/interrupts | tr -s ' ' | tr '\n' ';'; }
echo "GPIOTEST irq0: $(irq)"
echo "GPIOTEST v0: $(cat $G/value)"
echo GPIOTEST-READY1
while [ "$(cat $G/value)" = 1 ]; do usleep 10000 2>/dev/null || sleep 1; done
echo "GPIOTEST v1: $(cat $G/value) irq1: $(irq)"
echo GPIOTEST-READY2
while [ "$(cat $G/value)" = 0 ]; do usleep 10000 2>/dev/null || sleep 1; done
echo "GPIOTEST v2: $(cat $G/value) irq2: $(irq)"
echo GPIOTEST-END
# the UART sends at the line rate: let the console drain (the host stops
# QEMU at the end marker)
sleep 60
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/gpio-init"
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
    -qmp unix:"$WORK/qmp.sock",server=on,wait=off \
    -kernel "$FW/uImage.t31" \
    -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/usr/bin/gpio-init mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data) mem=40M rmem=24M@0x2800000" &
QPID=$!
python3 - "$WORK/qmp.sock" "$OUT/console.log" "$TIMEOUT" <<'PY'
import json, socket, sys, time
sock, log, timeout = sys.argv[1], sys.argv[2], int(sys.argv[3])
end = time.time() + timeout

def wait_for(tag):
    while time.time() < end:
        try:
            if tag in open(log, 'rb').read().decode('latin-1'):
                return True
        except OSError:
            pass
        time.sleep(0.5)
    return False

def qmp(f, cmd, **args):
    f.write(json.dumps({'execute': cmd, 'arguments': args}) + '\n')
    f.flush()
    while True:
        r = json.loads(f.readline())
        if 'return' in r or 'error' in r:
            return r

while time.time() < end:
    try:
        s = socket.socket(socket.AF_UNIX)
        s.connect(sock)
        break
    except OSError:
        time.sleep(0.2)
f = s.makefile('rw')
f.readline()
qmp(f, 'qmp_capabilities')
if wait_for('GPIOTEST-READY1'):
    qmp(f, 'qom-set', path='/machine/gpio', property='pc', value=0xfffffffe)
if wait_for('GPIOTEST-READY2'):
    qmp(f, 'qom-set', path='/machine/gpio', property='pc', value=0xffffffff)
wait_for('GPIOTEST-END')
PY
sleep 1
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true
grep -a "GPIOTEST" "$OUT/console.log" | tr -d '\r' || true

echo "== host"
python3 - "$OUT/console.log" <<'PY'
import re, sys
log = open(sys.argv[1], 'rb').read().decode('latin-1').replace('\r', '')
fail = 0

def check(what, ok, detail=''):
    global fail
    print('  %s: %s%s' % ('PASS' if ok else 'FAIL', what,
                          ' (%s)' % detail if detail else ''))
    fail |= not ok

def val(tag):
    m = re.search(r'GPIOTEST .*?%s: (\S+)' % tag, log)
    return m.group(1) if m else None

def irqs(tag):
    m = re.search(r'%s: (.*)$' % tag, log, re.M)
    return sum(int(n) for n in re.findall(r'^\s*\d+:\s*(\d+)', (m.group(1) if m else '').replace(';', '\n'), re.M))

m = re.search(r'GPIOTEST chipid: (\S+) (\S+) (\S+)', log)
check('the EFUSE chip ID: the SoC ID, then "OpenIPC!"',
      bool(m) and [x.lower() for x in m.groups()] ==
      ['0x10031003', '0x6e65704f', '0x21435049'], m.group(0) if m else '')
check('PC0 reads high (pulled up)', val('v0') == '1', val('v0'))
check('PC0 reads low once the host pulls it', val('v1') == '0', val('v1'))
check('and high again', val('v2') == '1', val('v2'))
i0, i1, i2 = irqs('irq0'), irqs('irq1'), irqs('irq2')
check('an interrupt per edge', i1 > i0 and i2 > i1, '%d %d %d' % (i0, i1, i2))
print('PASS' if not fail else 'FAIL')
sys.exit(fail)
PY
