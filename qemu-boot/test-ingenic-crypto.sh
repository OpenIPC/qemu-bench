#!/bin/bash
# The Ingenic security blocks (AES, HASH, RSA, DTRNG, DES) end to end on
# T31: the kernel and rootfs of an OpenIPC release, the T31 SDK's sample
# drivers (sample_aes, sample_hash, sample_rsa, sample_dtrng, sample_des)
# built against that kernel, and a client (qemu-boot/ingenic-crypto-
# test.c) that runs known-answer vectors through /dev/aes, /dev/hash,
# /dev/rsa and /dev/des and reads /dev/dtrng.  The host checks the
# answers: FIPS-197 and SP 800-38A for AES, Python's hashlib for the
# digests, pow() and the sample's own test_rsa.h vectors for RSA, FIPS
# 46's worked example for DES (by the CPU, and by the PDMA as triple DES
# with three equal keys) and a triple-DES CBC round trip.
#
#   bash qemu-boot/test-ingenic-crypto.sh [--fw DIR] [--kdir DIR]
#        [--sdk DIR] [--qemu PATH] [--out DIR] [--timeout S]
#
# --fw holds uImage.t31 and rootfs.squashfs.t31 (an OpenIPC "lite"
# release); --kdir is a built tree of the same kernel (for the module
# vermagic); --sdk the T31 SDK's opensource/drivers/misc.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
FW=/home/ai/dimerr/local/ingenic/fw/t31
KDIR=/home/ai/dimerr/tmp-scratch/nand-build/kernel-t31
SDK=/home/ai/dimerr/local/ingenic/extract/isvp-t31/software/sdk/Ingenic-SDK-T31-1.1.5-20220514/opensource/drivers/misc
TC=/home/ai/toolchains/ingenic/mips-gcc540-uclibc0.9.33.2-64bit-r3.3.0.smaller/bin
CROSS=mips-linux-uclibc-
OUT=
TIMEOUT=300

while [ $# -gt 0 ]; do
    case $1 in
    --fw) FW=$2; shift ;;
    --kdir) KDIR=$2; shift ;;
    --sdk) SDK=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
for f in "$FW/uImage.t31" "$FW/rootfs.squashfs.t31" "$KDIR/Module.symvers" \
         "$SDK/sample_aes/jz-aes.c" "$QEMU" "$TC/${CROSS}gcc"; do
    [ -e "$f" ] || { echo "missing $f" >&2; exit 2; }
done
WORK=$(mktemp -d)
trap 'kill $QPID 2>/dev/null || true; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

echo "== the sample drivers and the client"
mkdir "$WORK/mod"
for m in aes hash rsa dtrng des; do
    cp "$SDK/sample_$m/jz-$m.c" "$SDK/sample_$m/jz-$m.h" "$WORK/mod/"
done
# T31's soc/base.h has no RSA_IOBASE
printf 'obj-m := jz-aes.o jz-hash.o jz-rsa.o jz-dtrng.o jz-des.o\nccflags-y += -DRSA_IOBASE=0x134C0000\n' \
    > "$WORK/mod/Kbuild"
PATH=$TC:$PATH make -s -C "$KDIR" M="$WORK/mod" ARCH=mips \
    CROSS_COMPILE=$CROSS modules >/dev/null
unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.t31" >/dev/null
cp "$WORK"/mod/*.ko "$WORK/rfs/usr/lib/" 2>/dev/null || cp "$WORK"/mod/*.ko "$WORK/rfs/lib/"
"$TC/${CROSS}gcc" -O2 -Wall -march=mips32r2 -static -s \
    -o "$WORK/rfs/usr/bin/crypto-test" "$HERE/ingenic-crypto-test.c"

# RSA vectors: random odd moduli, full-size exponents
python3 - > "$WORK/rfs/etc/rsa-vectors" <<'PY'
import random
r = random.Random(41)
for bits in (1024, 2048, 1024):
    n = r.getrandbits(bits) | 1 << (bits - 1) | 1
    e, m = r.getrandbits(bits), r.getrandbits(bits) % n
    h = lambda v: '%0*x' % (bits // 4, v)
    print(bits, h(e), h(n), h(m))
PY
# ... and the first two decryptions of each size from the sample's own
# test (test_rsa.h): D, N and the cipher text G give its plain text M,
# which the sample compares word by word with what the driver returns
python3 - "$SDK/sample_rsa/test/test_rsa.h" >> "$WORK/rfs/etc/rsa-vectors" <<'PY'
import re, struct, sys
s = open(sys.argv[1]).read()
def arr(name):
    m = re.search(r'unsigned int ' + name + r'(\[[^\]]*\])+\s*=\s*\{(.*?)\};', s, re.S)
    return [int(x, 16) for x in re.findall(r'0x[0-9a-fA-F]+', m.group(2))]
for bits in (1024, 2048):
    k = bits // 32
    d, n, m, g = (arr('test_%d_%s' % (bits, x)) for x in 'dnmg')
    for j in range(2):
        h = lambda a: ''.join('%08x' % w for w in a[k * j:k * j + k])
        want = ''.join(struct.pack('<I', w).hex() for w in m[k * j:k * j + k])
        print(bits, h(d), h(n), h(g), want)
PY
cp "$WORK/rfs/etc/rsa-vectors" "$OUT/"
KO=$(cd "$WORK/rfs" && ls usr/lib/jz-aes.ko >/dev/null 2>&1 && echo /usr/lib || echo /lib)
cat > "$WORK/rfs/usr/bin/crypto-init" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t devtmpfs dev /dev 2>/dev/null
for m in aes hash rsa dtrng des; do insmod $KO/jz-\$m.ko; done
echo "CRYPTOTEST devs: \$(ls /dev/aes /dev/hash /dev/rsa /dev/dtrng /dev/des 2>&1 | tr '\n' ' ')"
crypto-test /etc/rsa-vectors
dmesg | grep -i -E "aes|hash|rsa|dtrng|des" | grep -v "^.*random:" | tail -12 | sed 's/^/CRYPTOTEST dmesg: /'
echo CRYPTOTEST-END
# the UART sends at the line rate: let the console drain (the host stops
# QEMU at the end marker)
sleep 60
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/crypto-init"
mksquashfs "$WORK/rfs" "$WORK/rootfs.squashfs" -comp xz -noappend -quiet \
    -all-root >/dev/null
python3 - "$WORK/nor.bin" "$FW/uImage.t31" "$WORK/rootfs.squashfs" <<'PY'
import sys
img = bytearray(b'\xff' * (16 << 20))
for path, off in ((sys.argv[2], 0x50000), (sys.argv[3], 0x250000)):
    data = open(path, 'rb').read()
    img[off:off + len(data)] = data
open(sys.argv[1], 'wb').write(img)
PY

echo "== boot (t31)"
"$QEMU" -M t31,flash-file="$WORK/nor.bin" -m 64M \
    -display none -serial file:"$OUT/console.log" -nic none \
    -kernel "$FW/uImage.t31" \
    -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/usr/bin/crypto-init mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data) mem=40M rmem=24M@0x2800000" &
QPID=$!
for i in $(seq "$TIMEOUT"); do
    grep -q CRYPTOTEST-END "$OUT/console.log" 2>/dev/null && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
sleep 1
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true
grep -a "CRYPTOTEST" "$OUT/console.log" | tr -d '\r' || true

echo "== host"
python3 - "$OUT/console.log" "$OUT/rsa-vectors" <<'PY'
import hashlib, sys
log = open(sys.argv[1], 'rb').read().decode('latin-1').replace('\r', '')
lines = log.split('\n')
fail = 0

def check(what, ok, detail=''):
    global fail
    print('  %s: %s%s' % ('PASS' if ok else 'FAIL', what,
                          ' (%s)' % detail if detail else ''))
    fail |= not ok

def got(prefix):
    return [l[len(prefix):].strip() for l in lines if l.startswith(prefix)]

aes = {'ecb': '69c4e0d86a7b0430d8cdb78070b4c55a',
       'cbc': '7649abac8119b246cee98e9b12e9197d'
              '5086cb9b507219ee95db113a917678b2'}
for mode, want in aes.items():
    g = got('AES %s ' % mode)
    out = [x for x in g if not x.startswith('roundtrip')]
    check('AES-128 %s through /dev/aes' % mode.upper(), out == [want],
          out[0] if out else 'none')
    check('AES-128 %s decrypts back' % mode.upper(),
          'roundtrip ok' in g)
msg = b'The quick brown fox jumps over the lazy dog'
for alg in ('md5', 'sha1', 'sha224', 'sha256', 'sha384', 'sha512'):
    g = got('HASH %s ' % alg)
    want = hashlib.new(alg, msg).hexdigest()
    check('%s through /dev/hash' % alg, g == [want], g[0] if g else 'none')
vec = [l.split() for l in open(sys.argv[2])]
res = got('RSA ')
for i, v in enumerate(vec):
    bits, e, n, m = v[:4]
    r = res[i].split()[1] if i < len(res) else ''
    if len(v) > 4:
        check('RSA-%s: the SDK test_rsa.h decryption through /dev/rsa'
              % bits, r == v[4])
    else:
        want = '%0*x' % (int(bits) // 4,
                         pow(int(m, 16), int(e, 16), int(n, 16)))
        check('RSA-%s M^E mod N through /dev/rsa' % bits, r == want)
cpu, irq = got('DTRNG cpu '), got('DTRNG irq ')
check('DTRNG: 4 words by polling, all different',
      len(cpu) == 4 and len(set(cpu)) == 4, ' '.join(cpu))
check('DTRNG: 2 words by interrupt', len(irq) == 2 and len(set(irq)) == 2,
      ' '.join(irq))
fips = '85e813540f0ab405'
d = got('DES cpu ')
check('DES ECB through /dev/des by the CPU (FIPS 46)', d == [fips],
      d[0] if d else 'none')
d = got('DES dma ')
check('triple DES (three equal keys) ECB by the PDMA, 64 bytes',
      d == [fips * 8], d[0][:32] if d else 'none')
check('triple DES CBC, 4 KiB by the PDMA, and back',
      got('DES cbc roundtrip ') == ['ok, ciphered'])
print('PASS' if not fail else 'FAIL')
sys.exit(fail)
PY
