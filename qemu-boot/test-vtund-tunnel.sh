#!/bin/bash
#
# vtund end to end between two guests of one OpenIPC image: OpenIPC/firmware#2835.
#
# The image ships vtund and its /usr/sbin/tunnel client, which needs a tun
# device: on seven neo kernels it had none, and tunnel stopped at
# "modprobe: module tun not found" without a word on the camera.  This boots
# the image twice, joined by a QEMU socket network, and runs:
#
#   server  vtund -s, a profile for the client, tap0 at 10.99.0.1
#   client  the shipped `tunnel 192.168.77.1`, unmodified, then a ping to
#           10.99.0.1 across the tunnel
#
# PASS needs the client's vtund session up and the ping answered.
#
# Each guest runs its script as PID 1 (init=/bin/sh -- -c "...") from the
# release SquashFS as a RAM disk, so nothing types at a shell or a getty, and
# no claim (root password) is involved.  tunnel derives its vtund profile and
# password from the default interface's MAC (profile = MAC without colons,
# password = sha1 of "MAC\n", upper case), so the client's MAC is pinned and
# the server is configured for it.
#
# The guests set addresses with ifconfig/route, not `ip addr ... dev eth0`:
# busybox ip resolves names through an AF_UNIX socket, which six neo kernels
# lacked until OpenIPC/firmware#2843.  tunnel itself needs only netlink
# (`ip r`), ifconfig, tunctl and vtund.
#
#   bash qemu-boot/test-vtund-tunnel.sh --soc hi3516cv200 [--machine M]
#        [--kernel F --rootfs F] [--qemu PATH] [--timeout SEC]
#        [--output-dir DIR] [--port N]
#
# Defaults: --machine = --soc; --kernel/--rootfs = qemu-boot/uImage.<soc> and
# qemu-boot/rootfs.squashfs.<soc>, as fetch-openipc-lite.sh leaves them
# (VARIANT=neo for a neo image).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"
SOC=""
MACHINE=""
KERNEL=""
ROOTFS=""
TIMEOUT=300
OUTPUT_DIR=""
PORT=""

while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC="$2"; shift 2 ;;
        --machine) MACHINE="$2"; shift 2 ;;
        --kernel) KERNEL="$2"; shift 2 ;;
        --rootfs) ROOTFS="$2"; shift 2 ;;
        --qemu) QEMU="$2"; shift 2 ;;
        --timeout) TIMEOUT="$2"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --port) PORT="$2"; shift 2 ;;
        -h|--help) sed -n '2,36p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
[ -n "$SOC" ] || { echo "--soc is required" >&2; exit 2; }
MACHINE=${MACHINE:-$SOC}
KERNEL=${KERNEL:-$SCRIPT_DIR/uImage.$SOC}
ROOTFS=${ROOTFS:-$SCRIPT_DIR/rootfs.squashfs.$SOC}
OUTPUT_DIR=${OUTPUT_DIR:-/tmp/vtund-$SOC-$$}
# A free port on the loopback, unless one was given.
PORT=${PORT:-$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1])')}
for f in "$QEMU" "$KERNEL" "$ROOTFS"; do
    [ -e "$f" ] || { echo "missing: $f" >&2; exit 2; }
done
mkdir -p "$OUTPUT_DIR"

CLIENT_MAC=02:11:22:33:44:55
PROFILE=$(echo "$CLIENT_MAC" | tr -d ':')
PASSWORD=$(echo "$CLIENT_MAC" | sha1sum | awk '{print $1}')

# Both guests: mount what busybox needs, wait for eth0 (the femac probe can
# finish after PID 1 starts), bring up lo.
PRE='mount -t proc p /proc;mount -t sysfs s /sys;mount -t tmpfs t /tmp'
PRE="$PRE;until [ -e /sys/class/net/eth0 ];do sleep 1;done;sleep 2"
PRE="$PRE;ifconfig lo 127.0.0.1 up"

# modprobe for kernels with tun as a module; built-in tun needs none.  vtund
# creates tap0 when the client's session starts and reads it at once: it
# must be up by then, or the read fails with EIO and the session closes, so
# vtund's own up {} brings it up, as the client's config does.  Its
# ifconfig needs double quotes, which cannot nest in the command line's
# quoted script: printf writes them as \042 (and %% as %%%%).
SERVER="$PRE;ifconfig eth0 192.168.77.1 netmask 255.255.255.0 up"
SERVER="$SERVER;modprobe tun 2>/dev/null"
SERVER="$SERVER;printf 'options { port 5000; ifconfig /sbin/ifconfig; } $PROFILE { password $PASSWORD; type ether; device tap0; proto tcp; up { ifconfig \\042%%%% 10.99.0.1 netmask 255.255.255.0 up\\042; }; }\\n' >/tmp/s.conf"
SERVER="$SERVER;vtund -s -f /tmp/s.conf"
SERVER="$SERVER;until netstat -ltn|grep -q ':5000 ';do sleep 1;done;m=listening;echo vtund-server-\$m"
SERVER="$SERVER;until ifconfig tap0 2>/dev/null|grep -q 10.99.0.1;do sleep 1;done"
SERVER="$SERVER;s=tap0;echo vtund-server-\$s-up;while :;do sleep 3600;done"

# tunnel backgrounds its own retry loop, which backs off 10 s more after each
# session that fails; the server is listening before the client boots, so the
# first attempt connects.  Poll for the session for up to two minutes.  The ps line shows vtund's own session state.
CLIENT="$PRE;ifconfig eth0 hw ether $CLIENT_MAC"
CLIENT="$CLIENT;ifconfig eth0 192.168.77.2 netmask 255.255.255.0 up"
CLIENT="$CLIENT;route add default gw 192.168.77.1"
CLIENT="$CLIENT;ls -l /dev/net/tun;tunnel 192.168.77.1"
CLIENT="$CLIENT;i=0;until ifconfig tunnel 10.99.0.2 netmask 255.255.255.0 up 2>/dev/null && ping -c1 -W2 10.99.0.1 >/dev/null"
CLIENT="$CLIENT;do i=\$((i+1));[ \$i -gt 30 ] && break;sleep 2;done"
CLIENT="$CLIENT;ps|grep 'vtund\\[c\\]'|grep -v grep"
# The guest echoes its command line at boot, so the result word is built at
# run time: the literal marker never appears in the log until the test ends.
CLIENT="$CLIENT;ping -c3 -W3 10.99.0.1 && r=pass || r=fail;echo vtund-result-\$r"
CLIENT="$CLIENT;while :;do sleep 3600;done"

# The machine appends its own mem=/mmz= after this; past "--" they reach
# sh as $0/$1 and are ignored.
APPEND="console=ttyAMA0,115200 root=/dev/ram0 rootfstype=squashfs init=/bin/sh --"

boot() { # name script nic
    # stdin from /dev/null: a backgrounded QEMU that reads the terminal is
    # stopped by SIGTTIN and the guest never runs.
    "$QEMU" -M "$MACHINE" -kernel "$KERNEL" -initrd "$ROOTFS" \
        -append "$APPEND -c \"$2\"" \
        -display none -monitor none -serial "file:$OUTPUT_DIR/$1.log" \
        -nic "$3" </dev/null >"$OUTPUT_DIR/$1.qemu.log" 2>&1 &
}

cleanup() { kill "${PIDS[@]}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT
PIDS=()
boot server "$SERVER" "socket,listen=127.0.0.1:$PORT,mac=02:11:22:33:44:01"
PIDS+=($!)
# The client starts once the server's vtund is listening, as a camera would
# reach a server that is already up: if the client's first attempt finds no
# listener, tunnel backs off and the session can miss the window.  That also
# means the server's socket netdev is listening when the client connects.
for _ in $(seq 1 "$TIMEOUT"); do
    tr -d '\r' 2>/dev/null < "$OUTPUT_DIR/server.log" | grep -aq 'vtund-server-listening' && break
    kill -0 "${PIDS[0]}" 2>/dev/null || break
    sleep 1
done
boot client "$CLIENT" "socket,connect=127.0.0.1:$PORT,mac=$CLIENT_MAC"
PIDS+=($!)

result=""
for _ in $(seq 1 "$TIMEOUT"); do
    result=$(tr -d '\r' 2>/dev/null < "$OUTPUT_DIR/client.log" | grep -aoE 'vtund-result-(pass|fail)' | head -1 || true)
    [ -n "$result" ] && break
    for p in "${PIDS[@]}"; do
        kill -0 "$p" 2>/dev/null || { result="qemu exited"; break 2; }
    done
    sleep 1
done
sleep 1

c() { tr -d '\r' < "$OUTPUT_DIR/$1.log" 2>/dev/null | sed -n '/Run \/bin\/sh as init/,$p'; }
echo "=== client ($SOC)"
c client | grep -avE '^\s*$' | head -30 || true
echo "=== server"
c server | grep -aE 'vtund|can.t|tap0' | head -10 || true

case $result in
    vtund-result-pass) echo "PASS: vtund session up and pinged across the tunnel ($SOC)"; exit 0 ;;
    "") echo "FAIL: no result within ${TIMEOUT}s; logs in $OUTPUT_DIR" ;;
    *) echo "FAIL: $result; logs in $OUTPUT_DIR" ;;
esac
exit 1
