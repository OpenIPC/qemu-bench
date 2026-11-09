#!/bin/bash
# The AltoBeam ATBM6441 IoT Wi-Fi end to end (hw/net/atbm6441-sdio.c,
# -M t31,wifi=atbm6441): the chip is already on its network, as it is on a
# doorbell that kept its credentials, and the host only bridges.  The guest
# does what OpenIPC/firmware#1974's S96wifi does -- rescan MSC1, load
# cfg80211 alone (no mac80211), insmod the atbm6441 driver, give wlan0 the
# address the chip leased (the driver applies none) -- and then checks
# what that issue reports broken with the vendor's binary driver:
#   - the probe ends without an oops or a warning;
#   - wlan0 carries traffic: ping, a TCP fetch from the host and 4 MiB
#     over TCP with its md5 intact;
#   - lsmod returns;
#   - /dev/atbm_ioctl answers an AT command (ATBM_FW_AT_CMD) and its
#     reader gets the chip's reply event;
#   - rmmod and a second insmod bring wlan0 back and it pings again.
# --mac80211 also loads the kernel's mac80211 first, which the issue's
# table says broke the vendor driver (the driver carries its own).
#
#   bash qemu-boot/test-ingenic-wifi-6441.sh --kernel uImage --driver DIR
#        [--fw DIR] [--at-tool atbm6441-at] [--mac80211] [--qemu PATH]
#        [--out DIR] [--timeout S]
#
# --kernel is an OpenIPC T31 kernel (t31.generic.config: MSC1 on,
# cfg80211 a module); --driver holds atbm6441_wifi_sdio.ko (gtxaspec/
# atbm6441 built with ATBM_WIFI_PLATFORM=22) and that kernel's
# cfg80211.ko; --at-tool is qemu-boot/atbm6441-at.c built static for the
# guest; --fw holds rootfs.squashfs.t31 (qemu-boot/fetch-openipc-lite.sh).
# Without them the test skips, exit 0.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
KERNEL=
DRIVER=
FW=
AT=
MAC80211=
OUT=
TIMEOUT=300

while [ $# -gt 0 ]; do
    case $1 in
    --kernel) KERNEL=$2; shift ;;
    --driver) DRIVER=$2; shift ;;
    --fw) FW=$2; shift ;;
    --at-tool) AT=$2; shift ;;
    --mac80211) MAC80211=1 ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
[ -n "$KERNEL" ] && [ -e "$KERNEL" ] || { echo "SKIP: no T31 kernel (--kernel)"; exit 0; }
[ -e "$DRIVER/atbm6441_wifi_sdio.ko" ] || { echo "SKIP: no atbm6441 driver (--driver)"; exit 0; }
[ -e "$DRIVER/cfg80211.ko" ] || { echo "SKIP: no cfg80211.ko (--driver)"; exit 0; }
[ -n "$FW" ] && [ -e "$FW/rootfs.squashfs.t31" ] || { echo "SKIP: no rootfs (--fw)"; exit 0; }
[ -x "$QEMU" ] || { echo "missing $QEMU" >&2; exit 2; }
command -v mksquashfs >/dev/null || { echo "SKIP: no mksquashfs"; exit 0; }

WORK=$(mktemp -d)
trap 'kill $QPID $HPID 2>/dev/null || true; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

# a host web server the guest fetches a token from over wlan0
mkdir -p "$WORK/www"
TOKEN=atbm6441-$RANDOM$RANDOM
echo "$TOKEN" > "$WORK/www/token"
head -c 4194304 /dev/urandom > "$WORK/www/bulk"
BULK=$(md5sum < "$WORK/www/bulk" | cut -d' ' -f1)
PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
python3 -m http.server "$PORT" --bind 127.0.0.1 --directory "$WORK/www" \
    > "$OUT/http.log" 2>&1 &
HPID=$!

unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.t31" >/dev/null
mkdir -p "$WORK/rfs/root"
cp "$DRIVER/cfg80211.ko" "$DRIVER/atbm6441_wifi_sdio.ko" "$WORK/rfs/root/"
[ -n "$AT" ] && cp "$AT" "$WORK/rfs/root/atbm6441-at"
if [ -n "$MAC80211" ]; then
    [ -e "$DRIVER/mac80211.ko" ] || { echo "SKIP: no mac80211.ko (--driver)"; exit 0; }
    cp "$DRIVER/mac80211.ko" "$WORK/rfs/root/"
fi
printf 'PORT=%s\n' "$PORT" > "$WORK/rfs/root/wifi.env"
cat > "$WORK/rfs/usr/bin/wifi-init" <<'EOF'
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t devtmpfs dev /dev 2>/dev/null
echo 8 > /proc/sys/kernel/printk
. /root/wifi.env

wlan_up() {
	for i in $(seq 1 30); do
		[ -d /sys/class/net/wlan0 ] && break
		sleep 1
	done
	[ -d /sys/class/net/wlan0 ] || return 1
	ip link set wlan0 up
	ip addr add 10.0.2.15/24 dev wlan0
	ip route add default via 10.0.2.2 dev wlan0 2>/dev/null
	sleep 1
}

# S96wifi: the SDIO rescan, cfg80211 only, the driver
echo INSERT > /sys/devices/platform/jzmmc_v1.2.1/present
sleep 2
dmesg | grep -q 'new SDIO card' && echo "WIFI sdio: card" || echo "WIFI sdio: none"
insmod /root/cfg80211.ko
[ -e /root/mac80211.ko ] && insmod /root/mac80211.ko && echo "WIFI mac80211: loaded"
insmod /root/atbm6441_wifi_sdio.ko
echo "WIFI insmod: $?"
if wlan_up; then echo "WIFI wlan0 present"; else
	echo "WIFI wlan0 MISSING"; dmesg | tail -40; echo WIFI-END; sleep 30; poweroff -f; fi
echo "WIFI mac: $(cat /sys/class/net/wlan0/address)"
ping -c 3 -W 3 10.0.2.2 >/dev/null 2>&1 && echo "WIFI ping: ok" || echo "WIFI ping: fail"
T=$(wget -q -T 10 -O - "http://10.0.2.2:$PORT/token" 2>/dev/null)
echo "WIFI http: ${T:-none}"
M=$(wget -q -T 60 -O - "http://10.0.2.2:$PORT/bulk" 2>/dev/null | md5sum | cut -d' ' -f1)
echo "WIFI bulk: $M"

# lsmod hung after the vendor driver's oops; give it 10 s
( lsmod > /tmp/lsmod.out 2>&1; echo done > /tmp/lsmod.done ) &
for i in $(seq 1 10); do [ -e /tmp/lsmod.done ] && break; sleep 1; done
if [ -e /tmp/lsmod.done ]; then
	echo "WIFI lsmod: ok $(grep -c . /tmp/lsmod.out) lines, $(grep -o '^atbm6441_wifi_sdio' /tmp/lsmod.out)"
else
	echo "WIFI lsmod: hung"
fi

if [ -x /root/atbm6441-at ]; then
	/root/atbm6441-at 'AT+SET_PWM 1 50 100 100000' 2>&1 | sed 's/^/WIFI /'
fi

# the reload path: rmmod, insmod again, traffic again
rmmod atbm6441_wifi_sdio && echo "WIFI rmmod: ok" || echo "WIFI rmmod: fail"
sleep 1
insmod /root/atbm6441_wifi_sdio.ko
echo "WIFI insmod2: $?"
if wlan_up; then
	ping -c 3 -W 3 10.0.2.2 >/dev/null 2>&1 && echo "WIFI ping2: ok" || echo "WIFI ping2: fail"
else
	echo "WIFI wlan0 after reload MISSING"
fi

if dmesg | grep -E 'Oops|Unable to handle|BUG:|kernel BUG|WARNING:|Call Trace' > /tmp/bad; then
	echo "WIFI kernel: complaints"; sed 's/^/WIFI   /' /tmp/bad | head -20
	dmesg | sed 's/^/DMESG /'
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
python3 - "$WORK/nor.bin" "$KERNEL" "$WORK/rootfs.squashfs" <<'PY'
import sys
img = bytearray(b'\xff' * (16 << 20))
for path, off in ((sys.argv[2], 0x50000), (sys.argv[3], 0x250000)):
    data = open(path, 'rb').read()
    img[off:off + len(data)] = data
open(sys.argv[1], 'wb').write(img)
PY

echo "== boot (t31l, wifi=atbm6441)"
"$QEMU" -M t31,variant=t31l,wifi=atbm6441,flash-file="$WORK/nor.bin" -m 64M \
    -display none -d guest_errors,unimp -D "$OUT/qemu.log" \
    -serial file:"$OUT/console.log" \
    -global atbm6441-sdio.trace=on \
    -nic user,model=atbm6441-sdio,net=10.0.2.0/24,host=10.0.2.2 \
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
check() {   # check <console pattern> <what>
    if grep -aq "$1" "$OUT/console.log"; then echo "  PASS: $2"; else
        echo "  FAIL: $2"; fail=1; fi
}
echo "results:"
check 'WIFI sdio: card' "the SDIO rescan finds the chip on MSC1"
[ -n "$MAC80211" ] && check 'WIFI mac80211: loaded' "the kernel's mac80211 loaded alongside"
check 'WIFI insmod: 0' "insmod returns 0"
check 'WIFI wlan0 present' "wlan0 created"
check 'WIFI ping: ok' "ping to the host over wlan0"
check "WIFI http: $TOKEN" "TCP fetch from the host over wlan0"
check "WIFI bulk: $BULK" "4 MiB over TCP, md5 intact"
check 'WIFI lsmod: ok .*atbm6441_wifi_sdio' "lsmod returns and lists the driver"
if [ -n "$AT" ]; then
    check 'WIFI AT ioctl: 0' "the AT command ioctl returns 0"
    check 'WIFI AT reply: AT+SET_PWM' "the chip's AT reply reaches /dev/atbm_ioctl"
    if grep -aq "AT command 'AT+SET_PWM 1 50 100 100000'" "$OUT/qemu.log"; then
        echo "  PASS: the chip received the AT command"; else
        echo "  FAIL: the chip did not receive the AT command"; fail=1; fi
fi
check 'WIFI rmmod: ok' "rmmod"
check 'WIFI insmod2: 0' "second insmod returns 0"
check 'WIFI ping2: ok' "ping after the reload"
check 'WIFI kernel: clean' "no oops, BUG or WARNING in dmesg"
[ $fail = 0 ] && echo PASS || { echo FAIL; grep -a 'WIFI' "$OUT/console.log"; }
exit $fail
