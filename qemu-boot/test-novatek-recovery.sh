#!/bin/bash
# Recover a Novatek NT9856x with a blank flash, the way a real board is
# recovered: OpenIPC's defib talks to the mask ROM model (nvt-bootrom's
# fallback), and U-Boot ends up running from RAM; defib then stops its
# autoboot and the prompt must answer.  Three ways in:
#
#   strap     boot-strap=0xc: the ROM goes straight to UART, takes the write
#             loader, which takes the U-Boot pack
#   fallback  rom-fallback=on, straps at NOR, no network: the blank flash
#             fails, the ROM tries SD, Ethernet (nobody answers) and USB,
#             then UART, as "strap"
#   eth       boot-strap=4 on a tap device: defib burn --nic answers the
#             ROM's DHCP and serves loader.bin and the pack over TFTP.
#             Needs sudo (the tap, ports 67 and 69).
#
# rom-fast=on cuts the waits before the UART stage tenfold.  The loader and
# the pack come from novatek-recovery.tar.gz on OpenIPC/u-boot's "novatek"
# release, which defib downloads itself.
#
# Usage: test-novatek-recovery.sh [--soc nt98562] [--defib "defib"]
#   WAYS="strap eth" picks the ways (default all three); KEEP=1 keeps the
#   logs; TAP names the tap device (default nvt-rec0, 192.168.77.1/24).
#   BIOS=re.elf runs a mask ROM image in place of the model: OpenIPC/
#   opennovatek's reconstruction of the NA51089 ROM, at silicon's timing,
#   so "fallback" reaches UART after about a minute.  defib probes all
#   along; UART 0 loses what comes in while the ROM is not reading it, as
#   silicon does (serial rx-idle-drop).

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
TAP=${TAP:-nvt-rec0}
TAP_IP=192.168.77.1/24
MADE_TAP=
ADDED_IP=
cleanup() {
    [ -n "$QPID" ] && kill "$QPID" 2>/dev/null || true
    if [ -n "$MADE_TAP" ]; then
        sudo ip link del "$TAP" 2>/dev/null || true
    elif [ -n "$ADDED_IP" ]; then
        sudo ip addr del "$TAP_IP" dev "$TAP" 2>/dev/null || true
    fi
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
for way in ${WAYS:-strap fallback eth}; do
    net=()
    burn=(timeout 300 $DEFIB burn -c "$SOC" -b)
    ready="U-Boot console ready"
    case $way in
        strap) opts="boot-strap=0xc" ;;
        fallback) opts="rom-fallback=on" ;;
        eth)
            opts="boot-strap=4"
            # A tap of the caller's keeps its state; one made here goes
            if ! ip link show "$TAP" > /dev/null 2>&1; then
                sudo ip tuntap add "$TAP" mode tap user "$(id -un)"
                MADE_TAP=1
                sudo ip link set "$TAP" up
            fi
            if ! ip -4 addr show dev "$TAP" | grep -q "inet ${TAP_IP%/*}/"; then
                sudo ip addr add "$TAP_IP" dev "$TAP"
                ADDED_IP=1
            fi
            net=(-nic "tap,ifname=$TAP,script=no,downscript=no")
            burn=(sudo timeout 300 "$(command -v $DEFIB)" burn -c "$SOC"
                  --nic "$TAP")
            ready="U-Boot is up from RAM"
            ;;
    esac
    sock="$WORK/$way.sock"
    rom=()
    if [ -n "${BIOS:-}" ]; then
        rom=(-bios "$BIOS")
    else
        opts="rom-fast=on,$opts"
    fi
    "$QEMU" -M "$SOC,flash-file=$WORK/blank.bin,$opts" "${rom[@]}" \
        -display none -nodefaults \
        -chardev "socket,id=s0,path=$sock,server=on,wait=off" \
        -serial chardev:s0 "${net[@]}" > "$WORK/qemu-$way.log" 2>&1 &
    QPID=$!
    for _ in $(seq 100); do [ -S "$sock" ] && break; sleep 0.05; done

    echo "=== $SOC, $way: defib burn"
    if "${burn[@]}" -p "socket://$sock" > "$WORK/defib-$way.log" 2>&1 &&
       grep -q "$ready" "$WORK/defib-$way.log" &&
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
