#!/bin/bash
# An OpenIPC Xiongmai release with the AltoBeam USB dongle (atbm-wifi,
# 007a:8888) on the DWC2 host, brought up the way a camera does it: the
# U-Boot environment names the Wi-Fi profile (wlandev) and the network
# (wlanssid / wlanpass), and S40network does the rest -- the profile in
# /etc/wireless/usb loads the USB host and the driver with modprobe, and
# wlan0's stanza runs wpa_supplicant and udhcpc.  Ethernet is unplugged, so
# a lease can only come over Wi-Fi.
#
#   --soc xm530v200   the release's own profile for the X2-WQ2 board
#                     (atbm603x-xm530v200-x2-wq2): wlan0 must come up, join
#                     the WPA2-PSK access point, take 10.0.2.15 and ping
#                     the host.  This is what OpenIPC/firmware#2854 fixed:
#                     modules.dep had no entry for dwc2, xm_usb0 or the
#                     ATBM driver, so every modprobe in the profile failed.
#   --soc xm530       booted on -M xm550 (the XM550 is the XM530 die and
#                     ships this image).  The release has no ATBM driver and
#                     no profile for these boards, so this checks the USB
#                     path the vendor modules give: dwc_otg and wifi_pdn
#                     (pad 96) enumerate the dongle.  --wlandev NAME with an
#                     image that has a profile (firmware#2316) runs the full
#                     check instead.
#
# No login and no claim: one init script is added to a copy of the root
# (as in test-ingenic-eth-reboot.sh) and reports on the console.
#
#   bash qemu-boot/test-xm-usb-wifi.sh [--soc xm530v200|xm530]
#       [--kernel uImage] [--rootfs rootfs.squashfs] [--wlandev NAME]
#       [--qemu PATH] [--dir qemu-boot] [--timeout 300] [--keep DIR]
set -euo pipefail

SOC=xm530v200
QEMU=./qemu-src/build/qemu-system-arm
DIR=qemu-boot
KERNEL=
ROOTFS=
WLANDEV=
TIMEOUT=300
KEEP=
PSK=openipc-twin
while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC=$2; shift 2 ;;
        --kernel) KERNEL=$2; shift 2 ;;
        --rootfs) ROOTFS=$2; shift 2 ;;
        --wlandev) WLANDEV=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --dir) DIR=$2; shift 2 ;;
        --timeout) TIMEOUT=$2; shift 2 ;;
        --keep) KEEP=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
case $SOC in
    xm530v200)
        MACHINE=xm530v200; MEM=63M; RAM=128M
        WLANDEV=${WLANDEV:-atbm603x-xm530v200-x2-wq2} ;;
    xm530)
        MACHINE=xm550; MEM=32M; RAM=128M ;;
    *) echo "--soc is xm530v200 or xm530" >&2; exit 2 ;;
esac
KERNEL=${KERNEL:-$DIR/uImage.$SOC}
ROOTFS=${ROOTFS:-$DIR/rootfs.squashfs.$SOC}
for f in "$KERNEL" "$ROOTFS"; do
    [ -f "$f" ] || { echo "$f missing (qemu-boot/fetch-openipc-lite.sh" \
                     "$SOC $DIR)" >&2; exit 2; }
done

WORK=$(mktemp -d)
QPID=
cleanup() {
    [ -n "$QPID" ] && kill "$QPID" 2>/dev/null || true
    if [ -n "$KEEP" ]; then
        mkdir -p "$KEEP" && cp "$WORK"/*.log "$KEEP"/ 2>/dev/null || true
    fi
    rm -rf "$WORK"
}
trap cleanup EXIT

unsquashfs -q -n -d "$WORK/root" "$ROOTFS" > /dev/null
cat > "$WORK/root/etc/init.d/S99zqemuwifi" <<'EOF'
#!/bin/sh
# qemu-boot/test-xm-usb-wifi.sh: report what S40network made of the dongle.
[ "$1" = start ] || exit 0
(
    exec > /dev/console 2>&1
    say() { echo "QEMU-WIFI: $*"; }
    dev=$(fw_printenv -n wlandev 2>/dev/null)
    if [ -z "$dev" ]; then
        # no profile: the vendor USB path alone
        modprobe dwc_otg && say "modprobe dwc_otg ok"
        modprobe wifi_pdn value=96 && say "modprobe wifi_pdn ok"
        sleep 5
    fi
    for d in /sys/bus/usb/devices/*; do
        [ -f "$d/idVendor" ] &&
            say "usb $(cat "$d/idVendor"):$(cat "$d/idProduct")"
    done
    if [ -n "$dev" ]; then
        i=0
        while [ $i -lt 90 ] && ! ip -4 addr show wlan0 2>/dev/null | grep -q inet; do
            sleep 1; i=$((i + 1))
        done
        lsmod | grep -E 'dwc|xm_usb|atbm|cfg80211|wifi_pdn' | sed 's/^/QEMU-WIFI: lsmod /'
        ip -4 addr show wlan0 2>/dev/null | grep inet | sed 's/^/QEMU-WIFI: addr /'
        ping -c 3 -W 3 10.0.2.2 2>&1 | grep transmitted | sed 's/^/QEMU-WIFI: ping /'
    fi
    say done
) &
EOF
chmod 755 "$WORK/root/etc/init.d/S99zqemuwifi"
COMP=$(unsquashfs -s "$ROOTFS" | sed -n 's/^Compression //p')
BLOCK=$(unsquashfs -s "$ROOTFS" | sed -n 's/^Block size //p')
PSEUDO=()
while read -r mode own path; do
    PSEUDO+=(-p "${path#squashfs-root/} m $mode ${own%/*} ${own#*/}")
done < <(unsquashfs -lln "$ROOTFS" | awk '$2 != "0/0" && $6 != "squashfs-root" {
    m = 0; s = substr($1, 2)
    for (i = 0; i < 9; i++) {
        c = substr(s, i + 1, 1)
        if (c != "-") m += 2 ^ (8 - i)
    }
    printf "%o %s %s\n", m, $2, $6 }')
mksquashfs "$WORK/root" "$WORK/rootfs.sqfs" -comp "$COMP" -b "$BLOCK" \
    -all-root "${PSEUDO[@]}" -noappend -quiet > /dev/null

# The environment fw_printenv finds by its CRC (OpenIPC's uboot-tools
# scans the flash for one): 64 KiB at 0x40000, the env partition.
ENV="bootdelay=1"
[ -n "$WLANDEV" ] &&
    ENV="$ENV wlandev=$WLANDEV wlanssid=OpenIPC wlanpass=$PSK wifipdn=96"
python3 - "$WORK/env.bin" $ENV <<'PY'
import sys, zlib
data = b''.join(v.encode() + b'\0' for v in sys.argv[2:]) + b'\0'
data = data.ljust(0x10000 - 4, b'\0')
open(sys.argv[1], 'wb').write(zlib.crc32(data).to_bytes(4, 'little') + data)
PY
KB=$(( ( $(stat -c %s "$WORK/rootfs.sqfs") + 65535 ) / 65536 * 64 ))
head -c $((16 << 20)) /dev/zero | tr '\0' '\377' > "$WORK/flash.bin"
dd if="$WORK/env.bin" of="$WORK/flash.bin" bs=1k seek=256 conv=notrunc status=none
dd if="$WORK/rootfs.sqfs" of="$WORK/flash.bin" bs=1k seek=$((256 + 64 + 2048)) \
    conv=notrunc status=none
MTD="xm_sfc:256k(boot),64k(env),2048k(kernel),${KB}k(rootfs),-(rootfs_data)"

timeout "$TIMEOUT" "$QEMU" -M "$MACHINE,flash-file=$WORK/flash.bin" -smp 2 \
    -m $RAM -kernel "$KERNEL" -nic none \
    -netdev user,id=w,net=10.0.2.0/24,host=10.0.2.2 \
    -device atbm-wifi,bus=usb-bus.0,port=1,netdev=w,ssid=OpenIPC,psk=$PSK,trace=on \
    -append "mem=$MEM console=ttyAMA0,115200 root=/dev/mtdblock3 rootfstype=squashfs init=/init mtdparts=$MTD" \
    -display none -monitor none -serial "file:$WORK/console.log" \
    -d guest_errors -D "$WORK/qemu.log" &
QPID=$!
for _ in $(seq "$TIMEOUT"); do
    grep -aq 'QEMU-WIFI: done' "$WORK/console.log" 2>/dev/null && break
    kill -0 "$QPID" 2>/dev/null || break
    sleep 1
done
kill "$QPID" 2>/dev/null || true
wait "$QPID" 2>/dev/null || true
QPID=

C=$WORK/console.log
fail=0
pass() { echo "  PASS: $1"; }
bad() { echo "  FAIL: $1"; fail=1; }
echo "$SOC on -M $MACHINE, atbm-wifi${WLANDEV:+, wlandev=$WLANDEV}:"
grep -a 'not found in modules.dep' "$C" | sort -u | sed 's/^/    /'
grep -aq 'QEMU-WIFI: done' "$C" && pass "the report ran" || bad "no report"
grep -aq 'QEMU-WIFI: usb 007a:8888' "$C" && pass "the dongle enumerated" ||
    bad "no 007a:8888 on the USB bus"
if [ -n "$WLANDEV" ]; then
    grep -aq 'EAPOL message 4 ok' "$WORK/qemu.log" &&
        pass "access point: WPA2 handshake completed" ||
        bad "no WPA2 handshake at the access point"
    grep -aq 'QEMU-WIFI: addr .*inet 10\.0\.2\.' "$C" &&
        pass "wlan0 took $(grep -ao 'inet 10\.0\.2\.[0-9]*' "$C" | head -1)" ||
        bad "no lease on wlan0"
    grep -aqE 'QEMU-WIFI: ping 3 packets transmitted, [1-3] (packets )?received' "$C" &&
        pass "the host answers over Wi-Fi" || bad "no ping over Wi-Fi"
else
    grep -aq 'QEMU-WIFI: modprobe dwc_otg ok' "$C" && pass "modprobe dwc_otg" ||
        bad "modprobe dwc_otg"
fi
grep -aqiE 'Oops|Unable to handle|BUG:' "$C" && bad "a kernel oops" || true
exit $fail
