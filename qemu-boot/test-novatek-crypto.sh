#!/bin/bash
# The crypto engines of emulated NT9856x under the vendor's own drivers: a
# kernel with na51089_crypto.c, na51089_hash.c and na51089_rsa.c and the
# kernel's crypto self-tests on (qemu-boot/build-novatek-test-kernel.sh)
# boots to a PID-1 script that reports what /proc/crypto says of every
# algorithm the drivers registered.  testmgr runs each one, as it
# registers, against the kernel's known-answer vectors, so "passed" means
# the emulated engine computed them:
#
#   - nvt-crypto: ecb / cbc / cfb / ofb / ctr of AES, DES and 3DES, and
#     gcm(aes) and ccm(aes), whose tags the driver builds from the S0,
#     GHASH and CV words the engine writes back into its descriptor;
#   - nvt-hash: sha1, sha256 and their HMACs, in chunks chained through
#     the IV registers;
#   - nvt-rsa: rsa, 512 to 2048-bit vectors through the engine's SRAM.
#
# testmgr has no vectors for cfb / ofb of DES and 3DES ("No test"); those
# report passed without having run.  The busybox comes from the OpenIPC
# release rootfs.
#
#   bash qemu-boot/test-novatek-crypto.sh --kernel uImage.nt9856x-test \
#       [--soc nt98566] [--machine nt98566] [--dir qemu-boot] \
#       [--qemu ./qemu-src/build/qemu-system-arm]
set -euo pipefail

KERNEL=
SOC=nt98566
MACHINE=
QEMU=./qemu-src/build/qemu-system-arm
DIR=qemu-boot
while [ $# -gt 0 ]; do
    case "$1" in
        --kernel) KERNEL=$2; shift 2 ;;
        --soc) SOC=$2; shift 2 ;;
        --machine) MACHINE=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --dir) DIR=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
MACHINE=${MACHINE:-$SOC}
ROOTFS=$DIR/rootfs.squashfs.$SOC
if [ ! -f "$KERNEL" ]; then
    echo "need --kernel: a kernel with the NA51089 crypto drivers" \
         "(qemu-boot/build-novatek-test-kernel.sh)" >&2
    exit 2
fi
if [ ! -f "$ROOTFS" ]; then
    echo "need $ROOTFS: unpack openipc.$SOC-nor-lite.tgz in $DIR" >&2
    exit 2
fi

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

unsquashfs -q -d "$WORK/rootfs" "$ROOTFS" bin/busybox lib >/dev/null
mkdir -p "$WORK/initramfs"/{bin,lib,proc,sys,dev}
cp "$WORK/rootfs/bin/busybox" "$WORK/initramfs/bin/"
cp -a "$WORK/rootfs/lib/libc.so" "$WORK/rootfs/lib/ld-musl-armhf.so.1" \
    "$WORK/initramfs/lib/"
ln -s busybox "$WORK/initramfs/bin/sh"
cat > "$WORK/initramfs/init" <<'INIT'
#!/bin/sh
bb=/bin/busybox
$bb mount -t proc proc /proc
$bb awk '/^name/ { n = $3 } /^driver/ { d = $3 }
         /^selftest/ { if (d ~ /na51089/) print "NVT-CRYPTO-ALG:", n, d, $3 }' \
    /proc/crypto
echo "NVT-CRYPTO-OK"
$bb reboot -f
INIT
chmod +x "$WORK/initramfs/init"
(cd "$WORK/initramfs" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/initramfs.cpio"

echo "Booting $KERNEL on -M $MACHINE"
set +e
timeout 300 "$QEMU" -M "$MACHINE" -nographic -monitor none -serial stdio \
    -no-reboot -kernel "$KERNEL" -initrd "$WORK/initramfs.cpio" \
    -append "console=ttyS0,115200 rdinit=/init" -nic none \
    > "$WORK/console.log" 2>&1
rc=$?
set -e
cat "$WORK/console.log"

fail=0
expect() {
    if grep -q "$1" "$WORK/console.log"; then
        echo "PASS: $2"
    else
        echo "FAIL: $2"
        fail=1
    fi
}
for d in crypto hash rsa; do
    expect "nvt-$d driver registered" "the $d driver probes"
done
for alg in ecb cbc cfb ofb ctr; do
    for c in aes des des3; do
        expect "NVT-CRYPTO-ALG: .* na51089-$alg-$c passed" "$alg($c)"
    done
done
for alg in gcm-aes ccm-aes sha1 sha256 hmac-sha1 hmac-sha256; do
    expect "NVT-CRYPTO-ALG: .* na51089-$alg passed" "$alg"
done
expect 'NVT-CRYPTO-ALG: rsa rsa-na51089 passed' 'rsa'
if grep -a '^alg: .*na51089' "$WORK/console.log" | grep -v 'No test'; then
    echo "FAIL: testmgr reports a failure"
    fail=1
fi
expect 'NVT-CRYPTO-OK' 'PID 1 ran to the end'
if [ $rc -ne 0 ]; then
    echo "FAIL: QEMU exited $rc"
    fail=1
fi
exit $fail
