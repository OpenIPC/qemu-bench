#!/bin/bash
# SDIO Wi-Fi end to end: the AltoBeam atbm_wlan driver on the emulated
# ATBM6031 (hw/net/atbm-sdio.c, -M <soc>,wifi=atbm).  The driver
# enumerates the SDIO card on MSC1, downloads its firmware, brings up the
# chip and speaks WSM; the model answers as an access point on a QEMU user
# network, so the guest associates to an open SSID, gets an address over
# DHCP from slirp and reaches the host (10.0.2.2).  With --psk the access
# point is WPA2-PSK (wifi-psk=): wpa_supplicant runs the 4-way handshake
# against it, the driver installs the keys in the chip, and the model
# checks they are the handshake's; --wrong-psk gives the guest a different
# passphrase, and then the test passes only if the station does NOT get
# on (message 2's MIC fails).
#
#   bash qemu-boot/test-ingenic-wifi.sh [--soc t31|t40] [--kernel uImage]
#        [--driver DIR] [--fw DIR] [--ssid NAME] [--psk PASSPHRASE]
#        [--wrong-psk] [--qemu PATH] [--out DIR] [--timeout S]
#
# t31 (the default): MSC1 is the jzmmc host; --kernel is an OpenIPC T31
# kernel with CONFIG_JZMMC_V12_MMC1 (the SDIO host) -- the one the
# UART-DMA test builds serves.  t40: MSC1 is an SDHCI host
# (sdhci-ingenic); --kernel is OpenIPC's T40 kernel (t40.generic.config)
# rebuilt with MSC1 enabled in its built-in device tree (status "okay",
# ingenic,sdio_clk, no non-removable), which the stock one leaves off.
# --driver holds atbm_wifi.ko (platform INGENICT31 / INGENICT41) and
# cfg80211.ko built against that kernel.  Without either (or the rootfs)
# the test skips, exit 0.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
SOC=t31
FW=
KERNEL=
DRIVER=
SSID=OpenIPC
PSK=
WRONG=
OUT=
TIMEOUT=300

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --kernel) KERNEL=$2; shift ;;
    --driver) DRIVER=$2; shift ;;
    --fw) FW=$2; shift ;;
    --ssid) SSID=$2; shift ;;
    --psk) PSK=$2; shift ;;
    --wrong-psk) WRONG=1 ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
case $SOC in
t31) KERNEL=${KERNEL:-/home/ai/dimerr/tmp-scratch/pdma2/kbuild/arch/mips/boot/uImage}
     DRIVER=${DRIVER:-/home/ai/dimerr/tmp-scratch/sdio/atbm/driver_install}
     MEM="mem=40M rmem=24M@0x2800000"; RAM=64M ;;
t40) KERNEL=${KERNEL:-/home/ai/dimerr/tmp-scratch/sdio/kb40w/arch/mips/boot/uImage}
     DRIVER=${DRIVER:-/home/ai/dimerr/tmp-scratch/sdio/atbm40/driver_install}
     MEM="mem=64M rmem=64M@0x4000000"; RAM=128M ;;
*) echo "unsupported --soc $SOC" >&2; exit 2 ;;
esac
FW=${FW:-/home/ai/dimerr/local/ingenic/fw/$SOC}
[ -e "$KERNEL" ] || { echo "SKIP: no $SOC kernel with MSC1 (--kernel)"; exit 0; }
[ -e "$DRIVER/atbm_wifi.ko" ] || { echo "SKIP: no atbm driver (--driver)"; exit 0; }
for f in "$FW/rootfs.squashfs.$SOC" "$QEMU"; do
    [ -e "$f" ] || { echo "missing $f" >&2; exit 2; }
done
command -v mksquashfs >/dev/null || { echo "SKIP: no mksquashfs"; exit 0; }
if [ -n "$WRONG" ] && [ -z "$PSK" ]; then
    echo "--wrong-psk needs --psk" >&2; exit 2
fi

WORK=$(mktemp -d)
trap 'kill $QPID 2>/dev/null || true; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.$SOC" >/dev/null
mkdir -p "$WORK/rfs/root"
cp "$DRIVER/cfg80211.ko" "$DRIVER/atbm_wifi.ko" "$WORK/rfs/root/"
if [ -n "$PSK" ]; then
    GPSK=$PSK
    [ -n "$WRONG" ] && GPSK="${PSK}-wrong"
    cat > "$WORK/rfs/root/open.conf" <<EOF
ctrl_interface=/tmp/wpa
network={
	ssid="$SSID"
	key_mgmt=WPA-PSK
	proto=RSN
	pairwise=CCMP
	group=CCMP
	psk="$GPSK"
	scan_ssid=1
}
EOF
else
    cat > "$WORK/rfs/root/open.conf" <<EOF
ctrl_interface=/tmp/wpa
network={
	ssid="$SSID"
	key_mgmt=NONE
	scan_ssid=1
}
EOF
fi
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
	wpa_cli -p /tmp/wpa -i wlan0 status 2>/dev/null | grep -q 'wpa_state=COMPLETED' && break
	sleep 1
done
echo "WIFI assoc: $(wpa_cli -p /tmp/wpa -i wlan0 status 2>/dev/null | grep wpa_state=)"
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

echo "== boot ($SOC, wifi=atbm, SSID $SSID${PSK:+, WPA2-PSK}${WRONG:+, the guest with a wrong passphrase})"
"$QEMU" -M $SOC,wifi=atbm,wifi-ssid="$SSID"${PSK:+,wifi-psk="$PSK"},flash-file="$WORK/nor.bin" -m $RAM -display none \
    -d guest_errors -D "$OUT/qemu.log" \
    -serial file:"$OUT/console.log" \
    -global atbm-sdio.trace=on \
    -nic user,model=atbm-sdio,net=10.0.2.0/24,host=10.0.2.2 \
    -kernel "$KERNEL" \
    -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/usr/bin/wifi-init mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data) $MEM" \
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
if [ -n "$WRONG" ]; then
    # the wrong passphrase: no association, no address, the AP saw a bad MIC
    if grep_q 'WIFI assoc: wpa_state=COMPLETED'; then
        echo "  FAIL: associated with the wrong passphrase"; fail=1
    else
        echo "  PASS: not associated with the wrong passphrase"; fi
    if grep -aq 'MIC mismatch' "$OUT/qemu.log" 2>/dev/null; then
        echo "  PASS: the access point rejected message 2's MIC"; else
        echo "  FAIL: no MIC mismatch at the access point"; fail=1; fi
    if grep_q 'WIFI ping: ok'; then
        echo "  FAIL: traffic got through"; fail=1; fi
    [ $fail = 0 ] && echo PASS || { echo FAIL; grep -a WIFI "$OUT/console.log"; }
    exit $fail
fi
if grep_q 'WIFI assoc: wpa_state=COMPLETED'; then
    echo "  PASS: associated to $SSID${PSK:+ (WPA2-PSK)}"; else
    echo "  FAIL: not associated"; fail=1; fi
if [ -n "$PSK" ]; then
    for k in 'EAPOL message 4 ok' 'pairwise key installed, matching' \
             'group key installed, matching'; do
        if grep -aq "$k" "$OUT/qemu.log" 2>/dev/null; then
            echo "  PASS: access point: $k"; else
            echo "  FAIL: access point: no '$k'"; fail=1; fi
    done
fi
a=$(sed -n 's/.*WIFI addr: //p' "$OUT/console.log" | tr -d '\r')
case $a in 10.0.2.*) echo "  PASS: DHCP address $a" ;; *)
    echo "  FAIL: no DHCP address (${a:-none})"; fail=1 ;; esac
if grep_q 'WIFI ping: ok'; then echo "  PASS: ping to the host"; else
    echo "  FAIL: no ping to the host"; fail=1; fi
[ $fail = 0 ] && echo PASS || { echo FAIL; grep -a WIFI "$OUT/console.log"; }
exit $fail
