#!/bin/bash
# The UART's DMA mode end to end on T31: jz_uart.c with
# CONFIG_SERIAL_JZ47XX_UART0_DMA, which moves received bytes through the
# PDMA in 16-byte bursts into two 1 KiB buffers (the tail of a transfer
# by the timeout interrupt and PIO) and sends from the circular buffer by
# PDMA.  No shipped config enables it, so the kernel is the OpenIPC T31
# kernel built with UART0 and its DMA mode (--kernel; T31's PDMA map,
# platform.c jzdma_pdata, has channels for UART0 and UART1 only, so
# UART2_DMA finds none and its driver oopses); the root is the OpenIPC
# release's.  UART0 is a socket: the host sends random bytes, the guest
# receives them on /dev/ttyS0, writes them back, and the host
# compares both ways and checks that the PDMA interrupted (the DMA
# completions) while the transfer ran.
#
#   bash qemu-boot/test-ingenic-uart-dma.sh [--kernel uImage] [--fw DIR]
#        [--bytes N] [--qemu PATH] [--out DIR] [--timeout S]
#
# Without --kernel (or with a missing one) the test skips, exit 0.  To
# build one: the T31 kernel's .config plus CONFIG_SERIAL_JZ47XX_UART0=y
# and CONFIG_SERIAL_JZ47XX_UART0_DMA=y.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
FW=/home/ai/dimerr/local/ingenic/fw/t31
KERNEL=/home/ai/dimerr/tmp-scratch/pdma2/kbuild/arch/mips/boot/uImage
BYTES=32768
OUT=
TIMEOUT=300

while [ $# -gt 0 ]; do
    case $1 in
    --kernel) KERNEL=$2; shift ;;
    --fw) FW=$2; shift ;;
    --bytes) BYTES=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
[ -e "$KERNEL" ] || { echo "SKIP: no kernel with UART0 DMA (--kernel)"; exit 0; }
for f in "$FW/rootfs.squashfs.t31" "$QEMU"; do
    [ -e "$f" ] || { echo "missing $f" >&2; exit 2; }
done
WORK=$(mktemp -d)
trap 'kill $QPID 2>/dev/null || true; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.t31" >/dev/null
cat > "$WORK/rfs/usr/bin/uart-dma-init" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t tmpfs tmp /tmp
mount -t devtmpfs dev /dev 2>/dev/null
echo 1 > /proc/sys/kernel/printk
pdma() { awk '/pdma/ { print \$2; exit }' /proc/interrupts; }
# one open for the whole test: jz_uart.c's shutdown in DMA mode frees its
# receive buffer's pages wrongly ("Bad page state"), which a close and
# reopen (stty's own open) would run before the transfer; min 1: a read
# waits for data instead of returning 0 while the bytes come in
exec 3<>/dev/ttyS0
stty -F /dev/ttyS0 115200 raw -echo -ixon -ixoff -crtscts min 1 time 0
echo "UARTDMA dma: \$(dmesg | grep -c -i 'jz-uart.0\|ttyS0')"
echo "UARTDMA pdma-before \$(pdma)"
echo "UARTDMA ready"
head -c $BYTES <&3 > /tmp/rx
echo "UARTDMA rx \$(wc -c < /tmp/rx) \$(md5sum < /tmp/rx | cut -d' ' -f1)"
echo "UARTDMA pdma-rx \$(pdma)"
cat /tmp/rx >&3
sleep 1
echo "UARTDMA pdma-tx \$(pdma)"
echo UARTDMA-END
# the UART sends at the line rate: let the console drain (the host stops
# QEMU at the end marker)
sleep 60
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/uart-dma-init"
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
PORT=$(python3 -c 'import socket; s = socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1])')

echo "== boot (t31, UART0 on a socket)"
# serial 0 is UART1 (the console), serial 1 UART0
"$QEMU" -M t31,flash-file="$WORK/nor.bin" -m 64M -display none -nic none \
    -serial file:"$OUT/console.log" \
    -chardev socket,id=u0,host=127.0.0.1,port=$PORT,server=on,wait=off \
    -serial chardev:u0 -kernel "$KERNEL" \
    -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/usr/bin/uart-dma-init mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data) mem=40M rmem=24M@0x2800000" \
    > "$OUT/qemu.err" 2>&1 &
QPID=$!

# the host side: random bytes in once the guest is ready, the echo back
python3 - "$PORT" "$OUT/console.log" "$BYTES" "$TIMEOUT" "$OUT" <<'PY'
import hashlib, os, random, socket, sys, time
port, log, n, timeout, out = int(sys.argv[1]), sys.argv[2], int(sys.argv[3]), \
    int(sys.argv[4]), sys.argv[5]
end = time.time() + timeout
s = None
while time.time() < end and s is None:
    try:
        s = socket.create_connection(('127.0.0.1', port), timeout=5)
    except OSError:
        time.sleep(0.5)
def console():
    try:
        return open(log, 'rb').read().decode('latin-1')
    except OSError:
        return ''
while time.time() < end and 'UARTDMA ready' not in console():
    time.sleep(0.5)
data = random.Random(7).randbytes(n)
open(os.path.join(out, 'sent.bin'), 'wb').write(data)
s.settimeout(1)
s.sendall(data)
got = b''
while time.time() < end and len(got) < n:
    try:
        b = s.recv(65536)
    except socket.timeout:
        continue
    if not b:
        break
    got += b
open(os.path.join(out, 'echo.bin'), 'wb').write(got)
print('sent %d bytes, md5 %s' % (n, hashlib.md5(data).hexdigest()))
print('echo %d bytes, md5 %s' % (len(got), hashlib.md5(got).hexdigest()))
PY
for i in $(seq "$TIMEOUT"); do
    grep -q UARTDMA-END "$OUT/console.log" 2>/dev/null && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
sleep 1
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true

fail=0
pass() { echo "  PASS: $1"; }
bad() { echo "  FAIL: $1"; fail=1; }
grep -a "UARTDMA" "$OUT/console.log" | sed 's/^/  /'
want=$(md5sum < "$OUT/sent.bin" | cut -d' ' -f1)
rx=$(grep -a -m1 "UARTDMA rx " "$OUT/console.log" | tr -d '\r' | cut -d' ' -f3,4)
[ "$rx" = "$BYTES $want" ] && pass "the guest received $BYTES bytes intact" \
    || bad "the guest received '$rx', want '$BYTES $want'"
cmp -s "$OUT/sent.bin" "$OUT/echo.bin" && pass "the echo came back intact" \
    || bad "the echo differs ($(wc -c < "$OUT/echo.bin") bytes back)"
pv() { grep -a -m1 "UARTDMA $1 " "$OUT/console.log" | tr -d '\r' | awk '{ print $3 }'; }
b=$(pv pdma-before); r=$(pv pdma-rx); t=$(pv pdma-tx)
if [ -n "$b" ] && [ -n "$r" ] && [ -n "$t" ] && [ "$r" -gt "$b" ] && [ "$t" -gt "$r" ]; then
    pass "PDMA interrupts while receiving ($b -> $r) and sending (-> $t)"
else
    bad "PDMA interrupts: before '$b', after rx '$r', after tx '$t'"
fi
[ $fail = 0 ] && echo PASS || echo FAIL
exit $fail
