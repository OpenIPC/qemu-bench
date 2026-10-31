#!/bin/bash
# SDIO Wi-Fi end to end on T31: the AltoBeam atbm_wlan driver on the
# emulated ATBM6031 (hw/net/atbm-sdio.c, -M t31,wifi=atbm).  The driver
# enumerates the SDIO card on MSC1, downloads its firmware, brings up the
# chip and speaks WSM; the model answers as an access point on a QEMU user
# network, so the guest associates to an open SSID, gets an address over
# DHCP from slirp and reaches the host (10.0.2.2).
#
#   bash qemu-boot/test-ingenic-wifi.sh [--kernel uImage] [--driver DIR]
#        [--fw DIR] [--ssid NAME] [--qemu PATH] [--out DIR] [--timeout S]
#
# --kernel is an OpenIPC T31 kernel with CONFIG_JZMMC_V12_MMC1 (the SDIO
# host) -- the one the UART-DMA test builds serves.  --driver holds
# atbm_wifi.ko and cfg80211.ko built against it.  Without either (or the
# rootfs) the test skips, exit 0.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
FW=/home/ai/dimerr/local/ingenic/fw/t31
KERNEL=/home/ai/dimerr/tmp-scratch/pdma2/kbuild/arch/mips/boot/uImage
DRIVER=/home/ai/dimerr/tmp-scratch/sdio/atbm/driver_install
SSID=OpenIPC
OUT=
TIMEOUT=300

while [ $# -gt 0 ]; do
    case $1 in
    --kernel) KERNEL=$2; shift ;;
    --driver) DRIVER=$2; shift ;;
    --fw) FW=$2; shift ;;
    --ssid) SSID=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
[ -e "$KERNEL" ] || { echo "SKIP: no T31 kernel with MMC1 (--kernel)"; exit 0; }
[ -e "$DRIVER/atbm_wifi.ko" ] || { echo "SKIP: no atbm driver (--driver)"; exit 0; }
for f in "$FW/rootfs.squashfs.t31" "$QEMU"; do
    [ -e "$f" ] || { echo "missing $f" >&2; exit 2; }
done
command -v mksquashfs >/dev/null || { echo "SKIP: no mksquashfs"; exit 0; }

WORK=$(mktemp -d)
trap 'kill $QPID 2>/dev/null || true; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.t31" >/dev/null
mkdir -p "$WORK/rfs/root"
cp "$DRIVER/cfg80211.ko" "$DRIVER/atbm_wifi.ko" "$WORK/rfs/root/"
cat > "$WORK/rfs/root/open.conf" <<EOF
network={
	ssid="$SSID"
	key_mgmt=NONE
	scan_ssid=1
}
EOF
cat > "$WORK/rfs/usr/bin/wifi-init" <<'EOF'
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t devtmpfs dev /dev 2>/dev/null
echo 8 > /proc/sys/kernel/printk
insmod /root/cfg80211.ko
insmod /root/atbm_wifi.ko atbm_printk_mask=-1
# the driver plugs the SDIO card on mmc1 and brings the chip up
for i in $(seq 1 30); do
	[ -d /sys/class/net/wlan0 ] && break
	sleep 1
done
if [ -d /sys/class/net/wlan0 ]; then
	echo "WIFI wlan0 present"
else
	echo "WIFI wlan0 MISSING"
	echo "WIFI dmesg:"; dmesg | tail -40
	echo WIFI-END; sleep 60; poweroff -f
fi
ifconfig wlan0 up
sleep 1
wpa_supplicant -B -i wlan0 -c /root/open.conf -D nl80211,wext 2>/tmp/wpa.log
for i in $(seq 1 20); do
	wpa_cli -i wlan0 status 2>/dev/null | grep -q 'wpa_state=COMPLETED' && break
	sleep 1
done
echo "WIFI assoc: $(wpa_cli -i wlan0 status 2>/dev/null | grep wpa_state=)"
udhcpc -i wlan0 -n -q -t 8 -T 2 2>/tmp/udhcpc.log
IP=$(ifconfig wlan0 | sed -n 's/.*inet addr:\([0-9.]*\).*/\1/p')
echo "WIFI addr: ${IP:-none}"
if ping -c 2 -W 3 10.0.2.2 >/dev/null 2>&1; then
	echo "WIFI ping: ok"
else
	echo "WIFI ping: fail"
fi
echo WIFI-END
# the UART sends at the line rate: let the console drain before poweroff
sleep 60
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

echo "== boot (t31, wifi=atbm, SSID $SSID)"
"$QEMU" -M t31,wifi=atbm,flash-file="$WORK/nor.bin" -m 64M -display none \
    -serial file:"$OUT/console.log" \
    -global atbm-sdio.trace=on \
    -nic user,model=atbm-sdio,net=10.0.2.0/24,host=10.0.2.2 \
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
grep_q() { grep -aq "$1" "$OUT/console.log"; }
echo "results:"
if grep_q 'WIFI wlan0 present'; then echo "  PASS: wlan0 created"; else
    echo "  FAIL: wlan0 not created"; fail=1; fi
if grep_q 'WIFI assoc: wpa_state=COMPLETED'; then
    echo "  PASS: associated to $SSID"; else
    echo "  FAIL: not associated"; fail=1; fi
a=$(sed -n 's/.*WIFI addr: //p' "$OUT/console.log" | tr -d '\r')
if [ -n "$a" ] && [ "$a" != none ]; then echo "  PASS: DHCP address $a"; else
    echo "  FAIL: no DHCP address"; fail=1; fi
if grep_q 'WIFI ping: ok'; then echo "  PASS: ping to the host"; else
    echo "  FAIL: no ping to the host"; fail=1; fi
[ $fail = 0 ] && echo PASS || { echo FAIL; grep -a WIFI "$OUT/console.log"; }
exit $fail
