#!/bin/bash
# A USB AltoBeam dongle on the T31's DWC2 host, end to end: the atbm-wifi
# model (hw/usb/atbm-wifi.c) as the ATBM6012B-X of OpenIPC/firmware#1655
# (T31L, "current chiptype 6012B-X"), driven by OpenIPC's atbm-wifi
# package built for that chip (atbm6012b_wifi_usb.ko).  The guest loads
# cfg80211 and the driver, wpa_supplicant joins the emulated access point
# (open, or WPA2-PSK with --psk), udhcpc leases an address from slirp, and
# the host answers a ping and serves a token over TCP; lsmod returns and
# dmesg carries no oops or warning.
#
#   bash qemu-boot/test-ingenic-wifi-usb.sh --kernel uImage --driver DIR
#        [--fw DIR] [--module NAME.ko] [--pid 0x888b] [--chip-version 0x4a]
#        [--efuse 0x0500] [--chip NAME] [--psk PASSPHRASE] [--qemu PATH]
#        [--out DIR] [--timeout S]
#
# --kernel is an OpenIPC T31 kernel (t31.generic.config: DWC2 host,
# cfg80211 a module); --driver holds the module (--module, default
# atbm6012b_wifi_usb.ko) and that kernel's cfg80211.ko; --fw holds
# rootfs.squashfs.t31 (an image with wpa_supplicant: t31_ultimate).
# --pid / --chip-version / --efuse pick the dongle (0x888b / 0x4a / 0x0500:
# the 6012B-X, which the driver reports as "current chiptype 6012B-X", as
# the issue's does); --chip is the chiptype the driver must report.
# Without them the test skips, exit 0.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
KERNEL=
DRIVER=
FW=
MODULE=atbm6012b_wifi_usb.ko
PID=0x888b
CHIPV=0x4a
EFUSE=0x0500
CHIP=6012B-X
SSID=OpenIPC
PSK=
OUT=
TIMEOUT=300

while [ $# -gt 0 ]; do
    case $1 in
    --kernel) KERNEL=$2; shift ;;
    --driver) DRIVER=$2; shift ;;
    --fw) FW=$2; shift ;;
    --module) MODULE=$2; shift ;;
    --pid) PID=$2; shift ;;
    --chip-version) CHIPV=$2; shift ;;
    --efuse) EFUSE=$2; shift ;;
    --chip) CHIP=$2; shift ;;
    --psk) PSK=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
[ -n "$KERNEL" ] && [ -e "$KERNEL" ] || { echo "SKIP: no T31 kernel (--kernel)"; exit 0; }
[ -e "$DRIVER/$MODULE" ] || { echo "SKIP: no $MODULE (--driver)"; exit 0; }
[ -e "$DRIVER/cfg80211.ko" ] || { echo "SKIP: no cfg80211.ko (--driver)"; exit 0; }
[ -n "$FW" ] && [ -e "$FW/rootfs.squashfs.t31" ] || { echo "SKIP: no rootfs (--fw)"; exit 0; }
[ -x "$QEMU" ] || { echo "missing $QEMU" >&2; exit 2; }
command -v mksquashfs >/dev/null || { echo "SKIP: no mksquashfs"; exit 0; }

WORK=$(mktemp -d)
trap 'kill $QPID $HPID 2>/dev/null || true; rm -rf "$WORK"' EXIT
# the logs outlive the work directory: kept on failure, dropped on a pass
# unless --out named them
KEEP=$OUT
OUT=${OUT:-$(mktemp -d)}
mkdir -p "$OUT"

mkdir -p "$WORK/www"
TOKEN=atbm-usb-$RANDOM$RANDOM
echo "$TOKEN" > "$WORK/www/token"
PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
python3 -m http.server "$PORT" --bind 127.0.0.1 --directory "$WORK/www" \
    > "$OUT/http.log" 2>&1 &
HPID=$!

unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.t31" >/dev/null
mkdir -p "$WORK/rfs/root"
cp "$DRIVER/cfg80211.ko" "$DRIVER/$MODULE" "$WORK/rfs/root/"
if [ -n "$PSK" ]; then
    KM="key_mgmt=WPA-PSK
	psk=\"$PSK\""
else
    KM="key_mgmt=NONE"
fi
cat > "$WORK/rfs/root/wpa.conf" <<EOF
ctrl_interface=/tmp/wpa
network={
	ssid="$SSID"
	$KM
	scan_ssid=1
}
EOF
printf 'PORT=%s\nMODULE=%s\n' "$PORT" "$MODULE" > "$WORK/rfs/root/wifi.env"
cat > "$WORK/rfs/usr/bin/wifi-init" <<'EOF'
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t devtmpfs dev /dev 2>/dev/null
echo 8 > /proc/sys/kernel/printk
. /root/wifi.env
sleep 2
insmod /root/cfg80211.ko
insmod /root/$MODULE
echo "WIFI insmod: $?"
for i in $(seq 1 30); do
	[ -d /sys/class/net/wlan0 ] && break
	sleep 1
done
if [ ! -d /sys/class/net/wlan0 ]; then
	echo "WIFI wlan0 MISSING"
	dmesg | tail -40 | sed 's/^/DMESG /'
	echo WIFI-END; sleep 30; poweroff -f
fi
echo "WIFI wlan0 present"
ip link set wlan0 up
wpa_supplicant -B -i wlan0 -c /root/wpa.conf -D nl80211,wext 2>/tmp/wpa.log
for i in $(seq 1 30); do
	wpa_cli -p /tmp/wpa -i wlan0 status 2>/dev/null | grep -q 'wpa_state=COMPLETED' && break
	sleep 1
done
echo "WIFI assoc: $(wpa_cli -p /tmp/wpa -i wlan0 status 2>/dev/null | grep wpa_state=)"
udhcpc -i wlan0 -n -q -t 8 -T 2 >/tmp/udhcpc.log 2>&1
echo "WIFI addr: $(ip -4 addr show wlan0 | sed -n 's/.*inet \([0-9.]*\).*/\1/p')"
ping -c 3 -W 3 10.0.2.2 >/dev/null 2>&1 && echo "WIFI ping: ok" || echo "WIFI ping: fail"
T=$(wget -q -T 10 -O - "http://10.0.2.2:$PORT/token" 2>/dev/null)
echo "WIFI http: ${T:-none}"
( lsmod > /tmp/lsmod.out 2>&1; echo done > /tmp/lsmod.done ) &
for i in $(seq 1 10); do [ -e /tmp/lsmod.done ] && break; sleep 1; done
[ -e /tmp/lsmod.done ] && echo "WIFI lsmod: ok" || echo "WIFI lsmod: hung"
if dmesg | grep -E 'Oops|Unable to handle|BUG:|kernel BUG|WARNING:|Call Trace' > /tmp/bad; then
	echo "WIFI kernel: complaints"; sed 's/^/WIFI   /' /tmp/bad | head -20
else
	echo "WIFI kernel: clean"
fi
echo WIFI-END
sleep 30
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/wifi-init"
mksquashfs "$WORK/rfs" "$WORK/rootfs.squashfs" -comp xz -noappend -quiet \
    -all-root >/dev/null
# the guest's mtdparts: 2048k(kernel) at 0x50000, 8192k(rootfs) at 0x250000
python3 - "$WORK/nor.bin" "$KERNEL" "$WORK/rootfs.squashfs" <<'PY'
import os, sys
img = bytearray(b'\xff' * (16 << 20))
for path, off, cap in ((sys.argv[2], 0x50000, 2048 << 10),
                       (sys.argv[3], 0x250000, 8192 << 10)):
    data = open(path, 'rb').read()
    if len(data) > cap:
        sys.exit('%s: %d bytes, its partition holds %d'
                 % (os.path.basename(path), len(data), cap))
    img[off:off + len(data)] = data
open(sys.argv[1], 'wb').write(img)
PY

echo "== boot (t31l, USB atbm-wifi pid $PID chip $CHIPV efuse $EFUSE, $MODULE${PSK:+, WPA2-PSK})"
"$QEMU" -M t31,variant=t31l,flash-file="$WORK/nor.bin" -m 64M -display none \
    -d guest_errors -D "$OUT/qemu.log" -serial file:"$OUT/console.log" \
    -nic none -netdev user,id=w,net=10.0.2.0/24,host=10.0.2.2 \
    -device atbm-wifi,bus=usb-bus.0,netdev=w,ssid="$SSID"${PSK:+,psk="$PSK"},pid=$PID,chip-version=$CHIPV,efuse=$EFUSE,trace=on \
    -kernel "$KERNEL" \
    -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/usr/bin/wifi-init mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data) mem=40M rmem=24M@0x2800000" \
    > "$OUT/qemu.err" 2>&1 &
QPID=$!

for i in $(seq "$TIMEOUT"); do
    grep -aq 'WIFI-END' "$OUT/console.log" 2>/dev/null && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
sleep 1
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true

fail=0
check() {
    if grep -aq "$1" "$OUT/console.log"; then echo "  PASS: $2"; else
        echo "  FAIL: $2"; fail=1; fi
}
echo "results:"
check 'WIFI insmod: 0' "insmod returns 0"
check "current chiptype $CHIP" "the driver identifies a $CHIP"
check 'WIFI wlan0 present' "wlan0 created"
check 'WIFI assoc: wpa_state=COMPLETED' "associated to $SSID${PSK:+ (WPA2-PSK)}"
check 'WIFI addr: 10.0.2.' "DHCP address over Wi-Fi"
check 'WIFI ping: ok' "ping to the host"
check "WIFI http: $TOKEN" "TCP fetch from the host"
check 'WIFI lsmod: ok' "lsmod returns"
check 'WIFI kernel: clean' "no oops, BUG or WARNING in dmesg"
if [ $fail = 0 ]; then
    echo PASS
    [ -n "$KEEP" ] || rm -rf "$OUT"
else
    echo FAIL
    grep -a 'WIFI\|DMESG' "$OUT/console.log"
    echo "logs: $OUT"
fi
exit $fail
