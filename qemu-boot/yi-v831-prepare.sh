#!/bin/bash
# Make a copy of a Xiaoyi (YI) V831 camera's NOR dump run past pairing
# under QEMU (-M v831): with it the YI application believes it is on
# Wi-Fi and starts its video encoders, over the emulated Ethernet.
#
#   bash qemu-boot/yi-v831-prepare.sh full.bin yi-online.bin [SSID [PSK]]
#   ./qemu-src/build/qemu-system-arm -M v831,flash-file=yi-online.bin \
#       -drive if=sd,format=raw,file=sd.img -nic user -nographic
#
# Two changes, both on the copy:
#
# 1. The system config (conf partition, mtdblock7 = flash 0x7F0000, 0x670
#    bytes, first word 1): an SSID at +0x1C and a passphrase at +0x5C, so
#    dispatch does not start pairing ("reset because no wifi config"), and
#    the start_with_reset word at +0x18 cleared.  dispatch copies it to the
#    backup in mfg itself.
# 2. The home SquashFS (mtdblock3 = flash 0x350000, 3.25 MiB): the camera
#    has no wlan0 here (its Wi-Fi is USB), so lower_half_init.sh renames
#    eth0 to wlan0 (ifrename, built here), and wpa_cli is a stand-in that
#    reports the association dispatch waits for ("wpa_state=COMPLETED"),
#    after which the camera takes a DHCP lease on it and says "wifi
#    connected".  wpa_supplicant itself fails without nl80211, harmlessly.
#
# Offsets are this firmware's (y291ga, dispatch read_mtd_conf / get_config);
# the script checks them before writing.  Needs unsquashfs, mksquashfs and
# arm-linux-gnueabihf-gcc.
set -euo pipefail

IN=${1:?usage: yi-v831-prepare.sh IN.bin OUT.bin [SSID [PSK]]}
OUT=${2:?usage: yi-v831-prepare.sh IN.bin OUT.bin [SSID [PSK]]}
SSID=${3:-qemu}
PSK=${4:-qemuqemu}

HOME_OFF=$((0x350000))
HOME_SIZE=$((0x340000))

for t in unsquashfs mksquashfs arm-linux-gnueabihf-gcc python3; do
    command -v "$t" >/dev/null || { echo "need $t" >&2; exit 2; }
done

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

cp "$IN" "$WORK/img.bin"

# ── 1. system config ─────────────────────────────────────────────────────
python3 - "$WORK/img.bin" "$SSID" "$PSK" <<'PY'
import struct, sys
path, ssid, psk = sys.argv[1], sys.argv[2].encode(), sys.argv[3].encode()
img = bytearray(open(path, 'rb').read())
C = 0x7F0000
if struct.unpack('<I', img[C:C + 4])[0] != 1:
    sys.exit('no YI system config at 0x7F0000 (first word is not 1)')
if len(ssid) > 63 or len(psk) > 63:
    sys.exit('SSID and PSK must be at most 63 bytes')
img[C + 0x18:C + 0x1C] = bytes(4)                   # start_with_reset
img[C + 0x1C:C + 0x5C] = ssid.ljust(0x40, b'\0')    # ssid
img[C + 0x5C:C + 0x9C] = psk.ljust(0x40, b'\0')     # pwd
open(path, 'wb').write(img)
PY

# ── 2. home SquashFS ─────────────────────────────────────────────────────
dd if="$WORK/img.bin" of="$WORK/home.sqfs" bs=4096 skip=$((HOME_OFF / 4096)) \
   count=$((HOME_SIZE / 4096)) status=none
unsquashfs -q -d "$WORK/home" "$WORK/home.sqfs" >/dev/null
for f in app/lower_half_init.sh base/tools/wpa_cli app/localbin; do
    [ -e "$WORK/home/$f" ] || { echo "home has no $f: not y291ga?" >&2; exit 1; }
done

cat > "$WORK/ifrename.c" <<'C'
/* ifrename OLD NEW: rename a network interface (SIOCSIFNAME), down first. */
#include <net/if.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    struct ifreq r;
    int s = socket(AF_INET, SOCK_DGRAM, 0);

    if (argc != 3 || s < 0) {
        fprintf(stderr, "usage: ifrename OLD NEW\n");
        return 2;
    }
    memset(&r, 0, sizeof(r));
    strncpy(r.ifr_name, argv[1], IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFFLAGS, &r) == 0 && (r.ifr_flags & IFF_UP)) {
        r.ifr_flags &= ~IFF_UP;
        ioctl(s, SIOCSIFFLAGS, &r);
    }
    strncpy(r.ifr_newname, argv[2], IFNAMSIZ - 1);
    if (ioctl(s, SIOCSIFNAME, &r) < 0) {
        perror("SIOCSIFNAME");
        return 1;
    }
    return 0;
}
C
arm-linux-gnueabihf-gcc -static -Os -s -o "$WORK/home/app/localbin/ifrename" \
    "$WORK/ifrename.c"

cat > "$WORK/home/base/tools/wpa_cli" <<SH
#!/bin/sh
# Stand-in for wpa_cli under QEMU (qemu-boot/yi-v831-prepare.sh): wlan0 is
# the emulated Ethernet, renamed, and always "associated".
iface=wlan0
while [ \$# -gt 0 ]; do
    case "\$1" in
        -i) iface=\$2; shift 2 ;;
        -i*) iface=\${1#-i}; shift ;;
        -*) shift ;;
        *) break ;;
    esac
done
case "\$1" in
status)
    ip=\`ifconfig \$iface 2>/dev/null | grep 'inet addr' | sed 's/.*inet addr:\([0-9.]*\).*/\1/'\`
    mac=\`ifconfig \$iface 2>/dev/null | grep HWaddr | awk '{print \$5}'\`
    echo "bssid=52:54:00:12:35:02"
    echo "freq=2412"
    echo "ssid=$SSID"
    echo "id=0"
    echo "mode=station"
    echo "pairwise_cipher=CCMP"
    echo "group_cipher=CCMP"
    echo "key_mgmt=WPA2-PSK"
    echo "wpa_state=COMPLETED"
    [ -n "\$ip" ] && echo "ip_address=\$ip"
    echo "address=\$mac"
    ;;
mib)
    echo "dot11RSNA4WayHandshakeFailures=0"
    ;;
scan_results)
    echo "bssid / frequency / signal level / flags / ssid"
    echo "52:54:00:12:35:02	2412	-40	[WPA2-PSK-CCMP][ESS]	$SSID"
    ;;
add_network)
    echo 0
    ;;
signal_poll)
    echo "RSSI=-40"; echo "LINKSPEED=72"; echo "FREQUENCY=2412"
    ;;
*)
    echo OK
    ;;
esac
exit 0
SH
chmod 755 "$WORK/home/base/tools/wpa_cli"

python3 - "$WORK/home/app/lower_half_init.sh" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
old = 'ifconfig ${NETWORK_IFACE} up\n'
if s.count(old) != 1:
    sys.exit('lower_half_init.sh: expected one "ifconfig ${NETWORK_IFACE} up"')
s = s.replace(old, '# QEMU: the emulated Ethernet stands in for the Wi-Fi\n'
                   '/home/app/localbin/ifrename eth0 ${NETWORK_IFACE}\n' + old)
open(p, 'w').write(s)
PY

mksquashfs "$WORK/home" "$WORK/home-new.sqfs" -comp xz -b 262144 -noappend \
    -all-root -quiet >/dev/null
python3 - "$WORK/img.bin" "$WORK/home-new.sqfs" "$HOME_OFF" "$HOME_SIZE" <<'PY'
import sys
img = bytearray(open(sys.argv[1], 'rb').read())
sq = open(sys.argv[2], 'rb').read()
off, size = int(sys.argv[3]), int(sys.argv[4])
if len(sq) > size:
    sys.exit('new home SquashFS (%d bytes) does not fit %d' % (len(sq), size))
img[off:off + size] = sq + b'\xff' * (size - len(sq))
open(sys.argv[1], 'wb').write(img)
print('home: %d of %d bytes' % (len(sq), size))
PY

mv "$WORK/img.bin" "$OUT"
echo "wrote $OUT"
