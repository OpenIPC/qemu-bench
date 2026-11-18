#!/bin/bash
# Boot an Ingenic SPI NAND image through the mask ROM stand-in and check
# every stage (docs/ingenic.md, "SPI NAND"):
#
#   boot 1 (flash): SPL (NAND header, SPL from the first pages) -> U-Boot
#           (loaded by the SPL with page reads; t31: saveenv, a NAND erase
#           and program) -> nand read of the kernel -> the kernel's SFC
#           NAND driver (3.10 jz_sfc_nand by PIO + interrupt, 4.4
#           ingenic_sfc_v2 by command table + descriptor DMA): partitions,
#           UBI attach, the root (t31: SquashFS on ubiblock + UBIFS
#           overlay; t41: UBIFS) -> the login prompt.  On t31 OpenIPC's
#           check_mac stores a MAC with fw_setenv: an erase and program of
#           the environment from Linux, checked in the image file after.
#   boot 2 (flash): the same image file again: no "bad CRC" (the
#           environment written in boot 1 is read back) and, on t31, the
#           stored MAC is kept.
#   boot 3/4 (-kernel, init=/bin/sh, the uImage taken from the image):
#           a file is written to a UBIFS volume and synced, then read back
#           after a restart -- programs reach the NAND and the image file.
#
# Nothing logs in: OpenIPC's first login asks its owner to accept the
# majestic EULA, which is not for a script to answer.
#
# The image comes from --image, or is built by ingenic-nand-image.sh from
# --uboot, --kernel and --rootfs (and for t41 --env, --layout t41).  The
# image is not changed: the boots run on a copy.
#
#   bash qemu-boot/test-ingenic-nand.sh --image nand-t31.bin
#   bash qemu-boot/test-ingenic-nand.sh --soc t41 --image nand-t41.bin
#   bash qemu-boot/test-ingenic-nand.sh --uboot u-boot-with-spl.bin \
#       --kernel uImage --rootfs rootfs.squashfs.t31 [--oob] [--part P]
#       [--qemu ./qemu-src/build/qemu-system-mipsel] [--timeout 900]
set -euo pipefail

SOC=t31
QEMU=./qemu-src/build/qemu-system-mipsel
TIMEOUT=900
IMAGE= UBOOT= KERNEL= ROOTFS= PART= OOB= ENVF=
while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --timeout) TIMEOUT=$2; shift 2 ;;
        --image) IMAGE=$2; shift 2 ;;
        --uboot) UBOOT=$2; shift 2 ;;
        --kernel) KERNEL=$2; shift 2 ;;
        --rootfs) ROOTFS=$2; shift 2 ;;
        --part) PART=$2; shift 2 ;;
        --oob) OOB=--oob; shift ;;
        --env) ENVF=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
case $SOC in
    t41) RAM=128M MTD=sfc0_nand LAYOUT=t41
         MEM="mem=48M@0x0 rmem=64M@0x3000000 nmem=16M@0x7000000 maxcpus=1"
         PARTS="1M(uboot),3M(kernel),20M(root),-(appfs)"
         SHELLROOT="ubi.mtd=2 root=ubi0:rootfs rootfstype=ubifs rw"
         VOL=ubi0:rootfs ;;
    t10|t20|t21|t23|t30|t31)
         RAM=64M MTD=sfc_nand LAYOUT=openipc
         MEM="mem=40M rmem=24M@0x2800000"
         PARTS="1M(boot),8M(kernel),40M(rootfs),-(data)"
         SHELLROOT="ubi.mtd=2 ubi.block=0,0 root=/dev/ubiblock0_0 rootfstype=squashfs"
         VOL=ubi0:rootfs_data ;;
    *) echo "no NAND layout for $SOC" >&2; exit 2 ;;
esac

WORK=$(mktemp -d)
QPID=
cleanup() {
    [ -n "$QPID" ] && kill "$QPID" 2>/dev/null
    rm -rf "$WORK"
}
trap cleanup EXIT
IMG=$WORK/nand.bin
if [ -n "$IMAGE" ]; then
    cp "$IMAGE" "$IMG"
else
    [ -n "$UBOOT" ] && [ -n "$KERNEL" ] && [ -n "$ROOTFS" ] || {
        echo "give --image, or --uboot, --kernel and --rootfs" >&2; exit 2; }
    bash "$(dirname "$0")/ingenic-nand-image.sh" --uboot "$UBOOT" \
        --kernel "$KERNEL" --rootfs "$ROOTFS" --layout $LAYOUT \
        ${PART:+--part $PART} ${ENVF:+--env $ENVF} $OOB --out "$IMG"
fi
TYPE=${PART:+,flash-type=$PART}

# The uImage at data offset 1 MiB (the page data, OOB or not).
python3 - "$IMG" "$WORK/uImage" <<'EOF'
import os, struct, sys
img, out = sys.argv[1], sys.argv[2]
size = os.path.getsize(img)
pages = {128 << 20: 65536, 256 << 20: 131072}.get(size)
stride = 2048 if pages else next(2048 + o for o in (64, 128)
                                 for n in (65536, 131072)
                                 if size == n * (2048 + o))
with open(img, 'rb') as f:
    def data(off, n):
        b = b''
        while n:
            page, col = divmod(off, 2048)
            f.seek(page * stride + col)
            k = min(n, 2048 - col)
            b += f.read(k)
            off += k
            n -= k
        return b
    hdr = data(1 << 20, 64)
    if struct.unpack('>I', hdr[:4])[0] != 0x27051956:
        sys.exit('no uImage at 1 MiB')
    open(out, 'wb').write(data(1 << 20, 64 + struct.unpack('>I', hdr[12:16])[0]))
EOF

# One boot: the console on a socket, driven by (wait-for, send) steps;
# QEMU is killed after the last one.  EXTRA: more QEMU arguments.
EXTRA=()
boot() {
    local log=$1; shift
    local port=$((20000 + RANDOM % 20000))
    "$QEMU" -M "$SOC,flash-file=$IMG$TYPE" -m $RAM -display none \
        -monitor none -nic none "${EXTRA[@]}" \
        -serial "tcp:127.0.0.1:$port,server" 2>"$log.qemu" &
    QPID=$!
    python3 - "$port" "$log" "$TIMEOUT" "$@" <<'EOF' || true
import re, socket, sys, time
port, log, tmo = int(sys.argv[1]), sys.argv[2], int(sys.argv[3])
steps = sys.argv[4:]
for _ in range(100):
    try:
        s = socket.create_connection(('127.0.0.1', port))
        break
    except OSError:
        time.sleep(0.1)
s.settimeout(0.5)
f = open(log, 'wb')
buf = b''
end = time.time() + tmo
for pat, send in zip(steps[0::2], steps[1::2]):
    mark = len(buf)
    while time.time() < end and not re.search(pat.encode(), buf[mark:]):
        try:
            d = s.recv(65536)
        except socket.timeout:
            continue
        if not d:
            sys.exit(1)
        buf += d
        f.write(d)
        f.flush()
    if not re.search(pat.encode(), buf[mark:]):
        sys.exit(1)
    time.sleep(0.5)
    if send.startswith('@sleep'):
        stop = time.time() + int(send[6:])
        while time.time() < stop:
            try:
                d = s.recv(65536)
                buf += d
                f.write(d)
            except socket.timeout:
                pass
    else:
        s.sendall(send.encode())
f.flush()
EOF
    kill "$QPID" 2>/dev/null || true
    wait "$QPID" 2>/dev/null || true
    QPID=
}

fail=0
check() {
    if grep -aqE -- "$3" "$2"; then
        echo "  PASS: $1"
    else
        echo "  FAIL: $1"
        fail=1
    fi
}
nocheck() {
    if grep -aqE -- "$3" "$2"; then
        echo "  FAIL: $1"
        fail=1
    else
        echo "  PASS: $1"
    fi
}
# The environment copies at 0xC0000 / 0xE0000 with a valid CRC: print
# NAME's value from the newest.
envget() {
    python3 - "$IMG" "$1" <<'EOF'
import os, struct, sys, zlib
img, name = sys.argv[1], sys.argv[2].encode()
size = os.path.getsize(img)
stride = 2048 if size in (128 << 20, 256 << 20) else \
    next(2048 + o for o in (64, 128) for n in (65536, 131072)
         if size == n * (2048 + o))
best = None
with open(img, 'rb') as f:
    for off in (0xC0000, 0xE0000):
        b = b''
        for p in range(64):
            f.seek((off // 2048 + p) * stride)
            b += f.read(2048)
        crc, flag = struct.unpack('<IB', b[:5])
        if zlib.crc32(b[5:]) == crc and (best is None or flag > best[0]):
            best = (flag, b[5:])
if best:
    for kv in best[1].split(b'\0'):
        if kv.startswith(name + b'='):
            print(kv[len(name) + 1:].decode())
EOF
}

echo "$SOC SPI NAND ($(stat -c %s "$IMG") bytes${TYPE:+, $TYPE}):"
L1=$WORK/boot1.log
boot "$L1" 'login:' '@sleep20'
echo " boot 1 (mask ROM -> SPL -> U-Boot -> kernel):"
check "SPL" "$L1" 'U-Boot SPL'
check "the SPL loads U-Boot from NAND" "$L1" 'image entry point: 0x80100000'
check "U-Boot's SFC NAND driver" "$L1" 'SFC_NAND'
[ $SOC != t41 ] && check "U-Boot saves its environment (erase + program)" \
    "$L1" 'NAND write: device 0 offset 0xc0000, size 0x20000'
check "U-Boot reads the kernel from NAND" "$L1" 'Starting kernel'
check "kernel up" "$L1" 'Linux version'
[ $SOC = t41 ] && check "the 4.4 driver identifies the part" "$L1" \
    'Found Supported nand device'
check "MTD partitions on $MTD" "$L1" "Creating 4 MTD partitions on \"$MTD\""
check "UBI attaches mtd2" "$L1" '(UBI|ubi0).*attached mtd2'
if [ $SOC = t41 ]; then
    check "root from NAND (UBIFS)" "$L1" 'VFS: Mounted root \(ubifs'
else
    check "root from NAND (SquashFS on ubiblock)" "$L1" \
        'VFS: Mounted root \(squashfs filesystem\) readonly on device 254'
    check "UBIFS overlay mounted" "$L1" 'name "rootfs_data"'
fi
check "login prompt" "$L1" 'login:'
nocheck "no kernel panic, no SFC errors" "$L1" \
    'Kernel panic|sfc_sync error|jz_sfc_nand_read error|ubi_read error'
MAC=$(grep -aoE 'check_mac: .*assigned ([0-9a-f:]{17})' "$L1" |
      grep -oE '[0-9a-f:]{17}$' | tail -1 || true)
if [ $SOC != t41 ]; then
    if [ -n "$MAC" ] && [ "$(envget ethaddr)" = "$MAC" ]; then
        echo "  PASS: Linux wrote ethaddr=$MAC into the U-Boot environment (fw_setenv: erase + program through jz_sfc_nand), in the image file"
    else
        echo "  FAIL: Linux wrote the MAC into the U-Boot environment (log: '$MAC', image: '$(envget ethaddr)')"
        fail=1
    fi
fi

L2=$WORK/boot2.log
boot "$L2" 'login:' '@sleep5'
echo " boot 2 (same image file):"
nocheck "U-Boot finds the environment saved before" "$L2" \
    'bad CRC, using default environment'
check "login prompt" "$L2" 'login:'
if [ $SOC != t41 ]; then
    nocheck "the MAC stored in boot 1 is kept" "$L2" 'check_mac: .*assigned'
fi

TOKEN=nand-$$-$RANDOM
APPEND="console=ttyS1,115200n8 $MEM $SHELLROOT init=/bin/sh mtdparts=$MTD:$PARTS"
EXTRA=(-kernel "$WORK/uImage" -append "$APPEND")
L3=$WORK/boot3.log
boot "$L3" \
    'job control' $'mount -t proc p /proc; mount -t ubifs '"$VOL"$' /mnt && echo '"$TOKEN"$' > /mnt/nand-test && sync && umount /mnt && echo DONE-$((6*7))\r' \
    'DONE-42' ''
L4=$WORK/boot4.log
boot "$L4" \
    'job control' $'mount -t ubifs '"$VOL"$' /mnt && echo GOT-$(cat /mnt/nand-test)\r' \
    'GOT-[a-z0-9-]*'$'\r' ''
echo " boot 3/4 (-kernel, a shell):"
check "a file written to $VOL and synced" "$L3" 'DONE-42'
check "read back after a restart" "$L4" "GOT-$TOKEN"

if [ $fail -ne 0 ]; then
    for l in "$L1" "$L2" "$L3" "$L4"; do
        echo "--- $(basename "$l") (tail) ---"
        tail -15 "$l" 2>/dev/null
    done
fi
exit $fail
