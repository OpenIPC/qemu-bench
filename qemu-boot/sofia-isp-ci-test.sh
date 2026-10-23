#!/bin/bash
#
# Sofia ISP regression test — CI variant.
#
# Boots emulated Hi3516EV200 with sensor=sc2315e, modprobes the
# vendor (XiongMai/Sofia) module chain, chroots into Sofia, watches
# for the "ISP Dev 0 running" log line.  The guest fetches Sofia +
# fonts + isp-profile from a small HTTP server running on the host
# (no NFS dependency).
#
# Required fixtures (bundled in the qemu-bench GitHub release
# under tag $FIXTURES_TAG, default "sofia-fixtures-v1"):
#   sofia_full.tar.gz  — Sofia binary + libs
#   Font.bin.web       — vendor font for /mnt/web/Fonts/Font.bin
#   Font.bin.custom    — vendor font for /mnt/custom/data/Fonts/Font.bin
#   isp-profile        — ARM static binary (informational; we don't
#                        need it for the regression check itself but
#                        keep it for ad-hoc debugging on CI)
#
# Exit 0 if Sofia logs "ISP Dev 0 running" within ISP_READY_TIMEOUT
# (default 30 s).  Exit non-zero with last 30 lines of QEMU output
# otherwise.
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="${QEMU:-$REPO_ROOT/qemu-src/build/qemu-system-arm}"

FIXTURES_TAG="${FIXTURES_TAG:-sofia-fixtures-v1}"
FIXTURES_REPO="${FIXTURES_REPO:-OpenIPC/qemu-bench}"
FIXTURES_BASE_URL="${FIXTURES_BASE_URL:-https://github.com/${FIXTURES_REPO}/releases/download/${FIXTURES_TAG}}"

ISP_READY_TIMEOUT="${ISP_READY_TIMEOUT:-30}"
HTTP_PORT="${HTTP_PORT:-8765}"
QEMU_TIMEOUT="${QEMU_TIMEOUT:-180}"

WORK="${WORK:-$(mktemp -d -t sofia-ci-XXXXXX)}"
SOFIA_LOG="$WORK/qemu-output.txt"

cleanup() {
    [ -n "$HTTP_PID" ] && kill "$HTTP_PID" 2>/dev/null || true
    [ -n "$QEMU_PID" ] && kill "$QEMU_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "==> Fetching Sofia fixtures from $FIXTURES_BASE_URL"
mkdir -p "$WORK/fixtures"
for f in sofia_full.tar.gz Font.bin.web Font.bin.custom isp-profile; do
    curl -fsSL "$FIXTURES_BASE_URL/$f" -o "$WORK/fixtures/$f"
done
chmod +x "$WORK/fixtures/isp-profile"
ls -la "$WORK/fixtures"

echo "==> Starting host HTTP server on :$HTTP_PORT"
python3 -m http.server "$HTTP_PORT" --directory "$WORK/fixtures" \
    >"$WORK/http.log" 2>&1 &
HTTP_PID=$!
sleep 1

echo "==> Booting Hi3516EV200 with sensor=sc2315e (timeout ${QEMU_TIMEOUT}s)"

# Inline guest setup script — fed to /bin/sh as PID 1 inside the
# guest.  Mirrors qemu-boot/run-ev200-sofia.sh except that it pulls
# fixtures over HTTP from the SLIRP host (10.0.2.2) instead of NFS.
GUEST_SETUP=$(cat <<'EOF'
set +e
echo "===setup: filesystems==="
mount -t proc proc /proc 2>/dev/null
mount -t tmpfs none /tmp

echo "===setup: network==="
udhcpc -q -i eth0 2>/dev/null || ifconfig eth0 10.0.2.15 netmask 255.255.255.0 up
route add default gw 10.0.2.2 2>/dev/null

echo "===setup: fetch fixtures==="
mkdir -p /tmp/fixtures
for f in sofia_full.tar.gz Font.bin.web Font.bin.custom isp-profile; do
    wget -q "http://10.0.2.2:HTTP_PORT_PLACEHOLDER/$f" -O "/tmp/fixtures/$f" \
        || { echo "WGET FAIL: $f"; exit 1; }
done
chmod +x /tmp/fixtures/isp-profile

echo "===setup: stage /tmp/vendor==="
mkdir -p /tmp/vendor/proc /tmp/vendor/dev /tmp/vendor/tmp \
         /tmp/vendor/mnt/custom/data/Fonts /tmp/vendor/mnt/mtd/Config \
         /tmp/vendor/mnt/web/Fonts /tmp/vendor/var
gunzip < /tmp/fixtures/sofia_full.tar.gz | tar xf - -C /tmp/vendor/
rm -f /tmp/vendor/usr/Fonts/Font.bin
cp /tmp/fixtures/Font.bin.web    /tmp/vendor/mnt/web/Fonts/Font.bin
cp /tmp/fixtures/Font.bin.custom /tmp/vendor/mnt/custom/data/Fonts/Font.bin
ln -sf /mnt/web/Fonts/Font.bin /tmp/vendor/usr/Fonts/Font.bin
mount --bind /dev  /tmp/vendor/dev  2>/dev/null
mount -t proc proc /tmp/vendor/proc 2>/dev/null

echo "===setup: modprobe vendor chain==="
cd /lib/modules/4.9.37/hisilicon
for m in sys_config hi_osal hi3516ev200_base hi3516ev200_sys \
         hi3516ev200_rgn hi3516ev200_vgs hi_mipi_rx hi3516ev200_vi \
         hi3516ev200_isp hi_sensor_i2c hi_sensor_spi \
         hi3516ev200_vpss hi3516ev200_chnl hi3516ev200_vedu \
         hi3516ev200_rc hi3516ev200_venc hi3516ev200_h264e \
         hi3516ev200_h265e hi3516ev200_jpege \
         hi3516ev200_aio hi3516ev200_ai hi3516ev200_ao \
         hi3516ev200_aenc hi3516ev200_adec hi3516ev200_acodec \
         hi3516ev200_pm hi3516ev200_ive hi_pwm; do
    [ -e ${m}.ko ] || continue
    case $m in
        sys_config) ARGS="chip=hi3516ev200 sensors=sc2315e g_cmos_yuv_flag=0 board=demo";;
        hi_osal)    ARGS="anony=1 mmz_allocator=hisi mmz=anonymous,0,0x46000000,32M";;
        hi3516ev200_vgs) ARGS="max_vgs_job=20 max_vgs_node=20 max_vgs_task=20";;
        hi3516ev200_ive) ARGS="save_power=0";;
        *)          ARGS="";;
    esac
    insmod ${m}.ko $ARGS 2>&1 | head -2
done

echo "===setup: launch Sofia==="
chroot /tmp/vendor /Sofia > /tmp/sofia.log 2>&1 &
SPID=$!
echo "Sofia pid=$SPID"

for i in $(seq 1 ISP_READY_TIMEOUT_PLACEHOLDER); do
    sleep 1
    if grep -q "ISP Dev 0 running" /tmp/sofia.log 2>/dev/null; then
        echo "ISP_DEV_0_RUNNING_DETECTED at t=${i}s"
        break
    fi
    if ! kill -0 $SPID 2>/dev/null; then
        echo "SOFIA_DIED at t=${i}s"
        echo "===sofia.log tail==="
        tail -30 /tmp/sofia.log
        echo "===END_FAIL==="
        poweroff -f 2>/dev/null
        exit 1
    fi
done

if ! grep -q "ISP_DEV_0_RUNNING_DETECTED" /proc/self/cmdline 2>/dev/null && \
   ! grep -q "ISP Dev 0 running" /tmp/sofia.log 2>/dev/null; then
    echo "ISP_NOT_READY_TIMEOUT"
    echo "===sofia.log tail==="
    tail -30 /tmp/sofia.log
    echo "===END_FAIL==="
    poweroff -f 2>/dev/null
    exit 1
fi

echo "===END_OK==="
poweroff -f 2>/dev/null
EOF
)

# Substitute placeholders
GUEST_SETUP="${GUEST_SETUP//HTTP_PORT_PLACEHOLDER/$HTTP_PORT}"
GUEST_SETUP="${GUEST_SETUP//ISP_READY_TIMEOUT_PLACEHOLDER/$ISP_READY_TIMEOUT}"

CMDLINE="console=ttyAMA0,115200 earlyprintk vdso=0 root=/dev/ram0 rootfstype=squashfs mtdparts=hi_sfc:256k(boot),64k(env),3072k(kernel),10240k(rootfs),-(rootfs_data) init=/bin/sh"

{
    sleep 8
    printf '%s\n' "$GUEST_SETUP"
} | timeout "$QEMU_TIMEOUT" "$QEMU" \
    -M hi3516ev200,sensor=sc2315e \
    -kernel "$SCRIPT_DIR/uImage.hi3516ev200" \
    -initrd "$SCRIPT_DIR/rootfs.squashfs.hi3516ev200" \
    -nographic -serial mon:stdio -nic user \
    -append "$CMDLINE" \
    -d unimp,guest_errors -D "$WORK/qemu-debug.log" \
    > "$SOFIA_LOG" 2>&1 &
QEMU_PID=$!

wait $QEMU_PID || true
QEMU_PID=

echo
if grep -q "ISP_DEV_0_RUNNING_DETECTED" "$SOFIA_LOG"; then
    echo "=== PASS: Sofia reached 'ISP Dev 0 running' ==="
    grep "ISP_DEV_0_RUNNING_DETECTED" "$SOFIA_LOG"
    exit 0
fi

echo "=== FAIL: Sofia did NOT reach 'ISP Dev 0 running' within ${ISP_READY_TIMEOUT}s ==="
echo "--- Last 60 lines of QEMU output ---"
tail -60 "$SOFIA_LOG"
exit 1
