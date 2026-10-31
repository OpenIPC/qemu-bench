#!/bin/bash
# Xiongmai XM530V200 with the AltoBeam USB Wi-Fi dongle (atbm-wifi): an
# OpenIPC flash image whose atbm603x driver brings up wlan0, scans for the
# emulated access point, associates (open, or WPA2-PSK with --psk), takes
# a DHCP lease from the dongle's netdev and pings the host through it.
#
#   bash qemu-boot/test-xm-wifi.sh --image openipc-xm530v200-nor.bin
#        [--psk PASSPHRASE [--wrong-psk]] [--proto rsn|wpa]
#        [--cipher ccmp|tkip] [--pmf optional|required] [--rekey S]
#        [--qemu PATH] [--out DIR]
#
# The image is an OpenIPC build for XM530V200 with the atbm-wifi package
# (atbm603x_comb_wifi_usb.ko) and wpa_supplicant; it is booted from a copy.
# The board's Ethernet is left unplugged, so the lease can only come over
# Wi-Fi.  --wrong-psk gives wpa_supplicant a different passphrase than the
# access point's: the access point must reject message 2's MIC and the
# station must stay unassociated.  --proto wpa, --cipher tkip, --pmf
# optional|required and --rekey N set the access point's security (the
# dongle's properties, shared with the SDIO model) and the station's to
# match; --rekey waits for two group rekeys and pings again.  Without an
# image the test skips (exit 0).
set -u
cd "$(dirname "$0")/.."

QEMU=./qemu-src/build/qemu-system-arm
IMAGE=
PSK=
WRONG=
PROTO=
CIPHER=
PMF=
REKEY=
OUT=
SSID=OpenIPC
while [ $# -gt 0 ]; do
    case $1 in
    --image) IMAGE=$2; shift ;;
    --psk) PSK=$2; shift ;;
    --wrong-psk) WRONG=1 ;;
    --proto) PROTO=$2; shift ;;
    --cipher) CIPHER=$2; shift ;;
    --pmf) PMF=$2; shift ;;
    --rekey) REKEY=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
[ -n "$WRONG" ] && [ -z "$PSK" ] && { echo "--wrong-psk needs --psk" >&2; exit 2; }
# the device alone, no image needed: the WPA2 known answers (PBKDF2, PRF,
# AES key wrap), the passphrase reaching the dongle, a short one refused
echo "== atbm-wifi device"
dev_fail=0
q=$(printf '%s\n' '{"execute":"qmp_capabilities"}' \
    '{"execute":"qom-get","arguments":{"path":"/machine/peripheral/wifi","property":"selftest"}}' \
    '{"execute":"qom-get","arguments":{"path":"/machine/peripheral/wifi","property":"psk"}}' \
    '{"execute":"quit"}' |
    timeout 60 "$QEMU" -M xm530v200 -S -display none -nic none -qmp stdio \
        -netdev user,id=w -device atbm-wifi,id=wifi,bus=usb-bus.0,port=1,netdev=w,psk=password 2>&1)
echo "$q" | grep -q '"return": true' && echo "  PASS: WPA2 known-answer self-test" ||
    { echo "  FAIL: WPA2 self-test"; dev_fail=1; }
echo "$q" | grep -q '"return": "password"' && echo "  PASS: the dongle takes the passphrase" ||
    { echo "  FAIL: psk property"; dev_fail=1; }
timeout 60 "$QEMU" -M xm530v200 -S -display none -nic none -netdev user,id=w \
    -device atbm-wifi,bus=usb-bus.0,port=1,netdev=w,psk=short 2>&1 |
    grep -q "psk must be 8..63" && echo "  PASS: a 5-character passphrase is refused" ||
    { echo "  FAIL: short psk accepted"; dev_fail=1; }

if [ -z "$IMAGE" ] || [ ! -f "$IMAGE" ]; then
    [ $dev_fail = 0 ] || exit 1
    echo "SKIP: no OpenIPC XM530V200 image with the atbm-wifi package (--image)"
    exit 0
fi
command -v python3 >/dev/null && python3 -c 'import pexpect' 2>/dev/null ||
    { echo "SKIP: python3 pexpect missing"; exit 0; }

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"
cp "$IMAGE" "$WORK/flash.bin"

DEV="atbm-wifi,bus=usb-bus.0,port=1,netdev=w,ssid=$SSID,trace=on"
[ -n "$PSK" ] && DEV="$DEV,psk=$PSK"
[ -n "$PROTO" ] && DEV="$DEV,proto=$PROTO"
[ -n "$CIPHER" ] && DEV="$DEV,cipher=$CIPHER"
[ -n "$PMF" ] && DEV="$DEV,pmf=$PMF"
[ -n "$REKEY" ] && DEV="$DEV,rekey=$REKEY"
case $PROTO in wpa) WPROTO=WPA; KM=WPA-PSK ;; *) WPROTO=RSN; KM=WPA2-PSK ;; esac
case $CIPHER in tkip) WCIPH=TKIP ;; *) WCIPH=CCMP ;; esac
case $PMF in optional) WPMF='\n ieee80211w=1' ;; required) WPMF='\n ieee80211w=2' ;;
    *) WPMF= ;; esac
STAPSK=$PSK
[ -n "$WRONG" ] && STAPSK="not-$PSK"
if [ -n "$PSK" ]; then
    NET="key_mgmt=WPA-PSK\n proto=$WPROTO\n pairwise=$WCIPH\n group=$WCIPH\n psk=\\\"$STAPSK\\\"$WPMF"
    echo "== XM530V200, atbm-wifi, $WPROTO $WCIPH${PMF:+ PMF $PMF}${REKEY:+ rekey ${REKEY}s}${WRONG:+ (wrong passphrase on the station)}"
else
    NET="key_mgmt=NONE"
    echo "== XM530V200, atbm-wifi, open network"
fi

QEMU_CMD="$QEMU -M xm530v200,flash-file=$WORK/flash.bin -nic none"
QEMU_CMD="$QEMU_CMD -netdev user,id=w,net=10.0.2.0/24,host=10.0.2.2"
QEMU_CMD="$QEMU_CMD -device $DEV -display none -monitor none -serial stdio"
QEMU_CMD="$QEMU_CMD -d guest_errors -D $OUT/qemu.log"
WAIT2=0
[ -n "$REKEY" ] && WAIT2=$((2 * REKEY + 4))
export QEMU_CMD SSID NET WRONG OUT WAIT2
timeout 900 python3 - <<'PY'
import os, pexpect, sys
log = open(os.path.join(os.environ['OUT'], 'console.log'), 'w')
p = pexpect.spawn('/bin/sh', ['-c', os.environ['QEMU_CMD']],
                  encoding='utf-8', codec_errors='replace', timeout=300)
p.logfile_read = log
p.expect('login:')
p.sendline('root')
while True:
    i = p.expect(['[Pp]assword: ', '# ', 'EULA.*\\? ', '\\[y/N\\]'])
    if i == 1:
        break
    p.sendline('y' if i >= 2 else 'OpenIPC1234')

def run(cmd, timeout=120):
    p.sendline(cmd + '; echo @@E""ND@@')
    p.expect('@@END@@\r*\n', timeout=timeout)
    out = p.before.replace('\r', '')
    p.expect('# ')
    # drop the echo of the command (the terminal wraps it at 80 columns)
    out = out.split('@@E""ND@@', 1)[-1].strip()
    print('$ ' + cmd)
    if out:
        print('  ' + out.replace('\n', '\n  '))
    return out

ssid, net = os.environ['SSID'], os.environ['NET']
run('modprobe atbm603x_comb_wifi_usb; sleep 10; ls /sys/class/net | tr "\\n" " "')
run('ifconfig wlan0 up; sleep 2; iw dev wlan0 scan 2>/dev/null | '
    'grep -E "SSID|RSN|Privacy" | head -4; '
    'iwlist wlan0 scan 2>/dev/null | grep -E "ESSID|Encryption" | head -2')
run('printf "ctrl_interface=/tmp/wpa\\nnetwork={\\n ssid=\\"%s\\"\\n %s\\n}\\n" '
    '> /tmp/w.conf; wpa_supplicant -B -i wlan0 -c /tmp/w.conf >/dev/null 2>&1;'
    ' sleep 15; wpa_cli -p /tmp/wpa -i wlan0 status 2>/dev/null | '
    'grep -E "^(wpa_state|ssid|key_mgmt|pairwise_cipher)="' % (ssid, net))
if not os.environ['WRONG']:
    run('udhcpc -i wlan0 -n -q -t 5 2>&1 | grep -iE "obtained|fail"; '
        'ifconfig wlan0 | grep "inet addr"', 90)
    run('ping -c 3 -W 3 10.0.2.2 2>&1 | grep -E "transmitted"', 60)
    w2 = int(os.environ['WAIT2'])
    if w2:
        # traffic across the group rekeys, then a ping and the state again
        run('ping -c %d -W 2 10.0.2.2 >/dev/null 2>&1; '
            'echo AFTER-$(ping -c 3 -W 3 10.0.2.2 2>&1 | grep -oE "[0-9] (packets )?received"); '
            'wpa_cli -p /tmp/wpa -i wlan0 status 2>/dev/null | '
            'sed -n "s/^wpa_state=/AFTER-STATE=/p"' % w2, w2 * 3 + 60)
run('echo OOPS=$(dmesg | grep -ciE "oops|unable to handle|BUG:")')
p.sendline('poweroff')
try:
    p.expect(pexpect.EOF, timeout=60)
except pexpect.TIMEOUT:
    p.terminate(force=True)
PY
rc=$?
[ $rc = 0 ] || echo "  (console driver exited $rc)"

echo "== checks"
fail=0
pass() { echo "  PASS: $1"; }
bad() { echo "  FAIL: $1"; fail=1; }
C=$OUT/console.log
Q=$OUT/qemu.log
grep -aq "wlan0" "$C" && pass "the driver brings up wlan0" || bad "no wlan0"
if [ -n "$WRONG" ]; then
    grep -aq "message 2 MIC mismatch" "$Q" &&
        pass "the access point rejected message 2's MIC" ||
        bad "no MIC rejection in the trace"
    grep -aq "wpa_state=COMPLETED" "$C" &&
        bad "associated with the wrong passphrase" ||
        pass "not associated with the wrong passphrase"
else
    grep -aq "wpa_state=COMPLETED" "$C" && pass "associated to $SSID" ||
        bad "wpa_supplicant did not complete"
    if [ -n "$PSK" ]; then
        KEYS=('EAPOL message 4 ok' 'pairwise key installed, matching'
              'group key installed, matching')
        [ "$PROTO" = wpa ] && KEYS+=('group message 2 ok, port open')
        [ -n "$PMF" ] && KEYS+=('IGTK installed, matching')
        for k in "${KEYS[@]}"; do
            grep -aq "$k" "$Q" && pass "access point: $k" || bad "no '$k'"
        done
        grep -aq "key_mgmt=$KM" "$C" && pass "wpa_supplicant: $KM" ||
            bad "wpa_supplicant key_mgmt not $KM"
        grep -aq "pairwise_cipher=$WCIPH" "$C" &&
            pass "wpa_supplicant: pairwise $WCIPH" ||
            bad "wpa_supplicant pairwise cipher not $WCIPH"
    fi
    ip=$(grep -ao "inet addr:10\.0\.2\.[0-9]*" "$C" | head -1 | cut -d: -f2)
    [ -n "$ip" ] && pass "DHCP address $ip" || bad "no 10.0.2.x lease on wlan0"
    grep -aqE "3 packets transmitted, [1-3] (packets )?received" "$C" &&
        pass "ping to the host" || bad "ping to 10.0.2.2"
fi
if [ -n "$REKEY" ] && [ -z "$WRONG" ]; then
    nr=$(grep -ac 'group rekey [0-9]* done' "$Q")
    [ "$nr" -ge 2 ] && pass "$nr group rekeys done" || bad "$nr group rekeys, want 2"
    grep -aqE "AFTER-[1-3] (packets )?received" "$C" &&
        grep -aq "AFTER-STATE=COMPLETED" "$C" &&
        pass "still associated and pinging after the rekeys" ||
        bad "lost after the rekeys"
fi
grep -aq "^OOPS=0" "$C" && pass "no kernel oops" || bad "kernel oops in dmesg"
[ $dev_fail = 0 ] || fail=1
[ $fail = 0 ] && echo PASS || echo FAIL
exit $fail
