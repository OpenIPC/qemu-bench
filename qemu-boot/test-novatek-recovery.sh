#!/bin/bash
# Recover a Novatek NT9856x with a blank flash over UART, the way a real
# board is recovered: OpenIPC's defib talks to the mask ROM model
# (nvt-bootrom's fallback), which takes the write loader, which takes the
# U-Boot pack and starts it; defib then stops U-Boot's autoboot and the
# prompt must answer.  Two ways in:
#
#   strap     boot-strap=0xc: the ROM goes straight to UART
#   fallback  rom-fallback=on, straps at NOR: the blank flash fails, the ROM
#             tries SD, Ethernet (no DHCP server here) and USB, then UART
#
# rom-fast=on cuts the waits before the UART stage tenfold.  The loader and
# the pack come from novatek-recovery.tar.gz on OpenIPC/u-boot's "novatek"
# release, which defib downloads itself.
#
# Usage: test-novatek-recovery.sh [--soc nt98562] [--defib "defib"]
#   WAYS="strap" runs one way only; KEEP=1 keeps the logs.

set -euo pipefail

SOC=nt98562
DEFIB=${DEFIB:-defib}
QEMU=${QEMU:-qemu-src/build/qemu-system-arm}

while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC=$2; shift 2 ;;
        --defib) DEFIB=$2; shift 2 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

WORK=$(mktemp -d)
QPID=
cleanup() {
    [ -n "$QPID" ] && kill "$QPID" 2>/dev/null || true
    if [ -n "${KEEP:-}" ]; then
        echo "logs kept in $WORK"
    else
        rm -rf "$WORK"
    fi
}
trap cleanup EXIT

python3 -c "open('$WORK/blank.bin', 'wb').write(b'\xff' * (8 << 20))"

# The prompt answers "version" with an OpenIPC U-Boot banner
prompt_answers() {
    python3 - "$1" <<'EOF'
import socket, sys, time
s = socket.socket(socket.AF_UNIX)
s.connect(sys.argv[1])
s.settimeout(0.2)
s.sendall(b'\rversion\r')
out = b''
end = time.time() + 5
while time.time() < end and b'U-Boot 20' not in out:
    try:
        out += s.recv(4096)
    except socket.timeout:
        pass
sys.exit(0 if b'U-Boot 20' in out else 1)
EOF
}

fails=0
for way in ${WAYS:-strap fallback}; do
    case $way in
        strap) opts="boot-strap=0xc" ;;
        fallback) opts="rom-fallback=on" ;;
    esac
    sock="$WORK/$way.sock"
    "$QEMU" -M "$SOC,flash-file=$WORK/blank.bin,rom-fast=on,$opts" \
        -display none -nodefaults \
        -chardev "socket,id=s0,path=$sock,server=on,wait=off" \
        -serial chardev:s0 > "$WORK/qemu-$way.log" 2>&1 &
    QPID=$!
    for _ in $(seq 100); do [ -S "$sock" ] && break; sleep 0.05; done

    echo "=== $SOC, $way: defib burn"
    if timeout 300 $DEFIB burn -c "$SOC" -p "socket://$sock" -b \
            > "$WORK/defib-$way.log" 2>&1 &&
       grep -q "U-Boot console ready" "$WORK/defib-$way.log" &&
       prompt_answers "$sock"; then
        echo "PASS: $SOC $way: U-Boot from RAM answers at its prompt"
    else
        echo "FAIL: $SOC $way"
        grep -v Downloading "$WORK/defib-$way.log" | tail -20
        fails=$((fails + 1))
    fi
    kill "$QPID" 2>/dev/null || true
    wait "$QPID" 2>/dev/null || true
    QPID=
done
exit $fails
