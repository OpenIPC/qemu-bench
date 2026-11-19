#!/bin/bash
#
# Boot emulated Hi3516EV200, mount the lab NFS share, modprobe the
# vendor (XiongMai/Sofia) module chain, chroot into Sofia.  Drops the
# user into an interactive shell after Sofia reaches "ISP Dev 0
# running" — useful for poking at /utils/isp-profile, ipcinfo, the
# Sofia HTTP API, etc.
#
# NOTE: Sofia's I2C-driven sensor init relies on the TX FIFO being
# reset between transactions; without that, Sofia's per-probe extra
# byte corrupts subsequent transactions.  Make sure qemu/hw/i2c/
# hisi-i2c.c clears txf_len on DATA1 write (fixed 2026-04-25).
#
# The /utils/sofia_full/usr/Fonts/Font.bin shipped in the tarball is
# a dangling symlink to /mnt/web/Fonts/Font.bin (real camera mounts
# /mnt/web from a separate cramfs partition).  Without a real
# Font.bin, Sofia asserts in FontManager.cpp:324.  We stage both
# real Font.bin files (pulled from camera 10.216.128.106) from
# /utils/sofia_fonts/ into the chroot before launching.
#
# Vendor sysroot, Font.bin, and the ARM static `isp-profile` binary
# are expected on the NFS export at:
#   10.216.128.227:/srv/nfsroot/sofia_full/   (Sofia + libs + /usr/Fonts)
#   10.216.128.227:/srv/nfsroot/isp-profile   (ARM static, musl)
#
# On the host, that share is at /mnt/noc/.  Captures and other
# artefacts written under /utils/ in the guest land in /mnt/noc/ on
# the host automatically.
#
# Usage:
#   bash qemu-boot/run-ev200-sofia.sh [--sensor sc2315e] [--debug]
#
# --debug enables Sofia's verbose env-var logs (XMVIDEO_DEBUG_ON,
#         XMCAP_DEBUG_ON, XMIMP_DEBUG_ON, JSON_PARSE_RES_OUTPUT) so
#         Sofia traces high-level events (switchDayNightColor,
#         setVideoStd, AE-weight changes, gamma day/night, etc.) in
#         the chroot's /tmp/sofia.log.  Note: HI_MPI_ISP_* per-call
#         tracing requires gdb breakpoints — see
#         tools/isp-profile/README.md "Driving Sofia via gdb on a
#         camera" appendix in the majestic repo.
#
# Idempotent: if /utils is already an NFS mount and /tmp/vendor/Sofia
# already exists, skip remount and untar so repeated runs are fast.
# (Each fresh QEMU boot starts from scratch though — this only
# matters across re-execs of the inline shell script after Sofia
# exits or is killed.)
#
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

SENSOR="${SENSOR:-sc2315e}"
NFS_HOST="${NFS_HOST:-10.216.128.227}"
NFS_PATH="${NFS_PATH:-/srv/nfsroot}"
DEBUG=""

while [ $# -gt 0 ]; do
    case "$1" in
        --sensor)   SENSOR="$2"; shift 2;;
        --sensor=*) SENSOR="${1#--sensor=}"; shift;;
        --nfs)      NFS_HOST="$2"; shift 2;;
        --debug)    DEBUG=1; shift;;
        *)          break;;
    esac
done

# Inline guest setup script, fed to /bin/sh as PID 1 inside the guest.
# After running, `exec sh -i` keeps stdin open so the user gets a
# prompt back while Sofia continues running in the background.
read -r -d '' GUEST_SETUP <<EOF || true
set +e
echo "===setup: mount essentials==="
mount -t proc proc /proc 2>/dev/null
mount -t tmpfs none /tmp
mount -t devtmpfs none /dev 2>/dev/null

echo "===setup: bring up eth0 (DHCP under SLIRP)==="
udhcpc -q -i eth0 2>/dev/null || ifconfig eth0 10.0.2.15 netmask 255.255.255.0 up
route add default gw 10.0.2.2 2>/dev/null

echo "===setup: NFS mount /utils==="
mkdir -p /utils
if mount | grep -q ' /utils type nfs'; then
    echo "already mounted"
else
    mount -o nolock,vers=3 ${NFS_HOST}:${NFS_PATH} /utils
    [ \$? -ne 0 ] && { echo "NFS MOUNT FAILED"; exec sh -i; }
fi

echo "===setup: stage /tmp/vendor==="
mkdir -p /tmp/vendor /tmp/vendor/proc /tmp/vendor/dev /tmp/vendor/tmp
mkdir -p /tmp/vendor/mnt/custom/data/Fonts /tmp/vendor/mnt/mtd/Config /tmp/vendor/var
mkdir -p /tmp/vendor/mnt/web/Fonts
if [ ! -x /tmp/vendor/Sofia ]; then
    cp -a /utils/sofia_full/. /tmp/vendor/
    # Replace the dangling /usr/Fonts/Font.bin symlink (which points
    # to /mnt/web/Fonts/Font.bin on real cameras) with real font
    # blobs pulled from camera 10.216.128.106.
    rm -f /tmp/vendor/usr/Fonts/Font.bin
    cp /utils/sofia_fonts/Font.bin.web    /tmp/vendor/mnt/web/Fonts/Font.bin
    cp /utils/sofia_fonts/Font.bin.custom /tmp/vendor/mnt/custom/data/Fonts/Font.bin
    ln -sf /mnt/web/Fonts/Font.bin /tmp/vendor/usr/Fonts/Font.bin
fi
mount --bind /dev  /tmp/vendor/dev  2>/dev/null
mount -t proc proc /tmp/vendor/proc 2>/dev/null

echo "===setup: modprobe vendor chain==="
cd /lib/modules/4.9.37/hisilicon
# Module set lifted from the working previous-session sequence:
# sys → rgn → vgs → mipi_rx → vi → isp → sensor i2c/spi → vpss →
# chnl → vedu/rc/venc → codec → audio chain → pm/ive/pwm
for m in sys_config hi_osal hi3516ev200_base hi3516ev200_sys \
         hi3516ev200_rgn hi3516ev200_vgs hi_mipi_rx hi3516ev200_vi \
         hi3516ev200_isp hi_sensor_i2c hi_sensor_spi \
         hi3516ev200_vpss hi3516ev200_chnl hi3516ev200_vedu \
         hi3516ev200_rc hi3516ev200_venc hi3516ev200_h264e \
         hi3516ev200_h265e hi3516ev200_jpege \
         hi3516ev200_aio hi3516ev200_ai hi3516ev200_ao \
         hi3516ev200_aenc hi3516ev200_adec hi3516ev200_acodec \
         hi3516ev200_pm hi3516ev200_ive hi_pwm; do
    [ -e \${m}.ko ] || continue
    case \$m in
        sys_config) ARGS="chip=hi3516ev200 sensors=${SENSOR} g_cmos_yuv_flag=0 board=demo";;
        hi_osal)    ARGS="anony=1 mmz_allocator=hisi mmz=anonymous,0,0x46000000,32M";;
        hi3516ev200_vgs) ARGS="max_vgs_job=20 max_vgs_node=20 max_vgs_task=20";;
        hi3516ev200_ive) ARGS="save_power=0";;
        *)          ARGS="";;
    esac
    insmod \${m}.ko \$ARGS 2>&1 | head -2
done

echo "===setup: launch Sofia==="
${DEBUG:+XMVIDEO_DEBUG_ON=1 XMCAP_DEBUG_ON=1 XMIMP_DEBUG_ON=1 JSON_PARSE_RES_OUTPUT=1} \\
    chroot /tmp/vendor /Sofia > /tmp/sofia.log 2>&1 &
SPID=\$!
echo "Sofia pid=\$SPID"

for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25; do
    sleep 1
    if grep -q "ISP Dev 0 running" /tmp/sofia.log 2>/dev/null; then
        echo "ISP READY at t=\${i}s"
        break
    fi
    if ! kill -0 \$SPID 2>/dev/null; then
        echo "Sofia DIED at t=\${i}s"
        break
    fi
done

echo "===Sofia status==="
kill -0 \$SPID 2>/dev/null && echo ALIVE || echo DEAD
echo "===tail /tmp/sofia.log==="
tail -10 /tmp/sofia.log 2>/dev/null

cat <<'TIPS'
=========================================================
Sofia is running.  Interactive tips:
  /utils/isp-profile export /utils/isp-captures/${SENSOR}_qemu_day.isp ${SENSOR}
  ipcinfo --short_sensor
  i2cdetect -y -r 0
  tail -20 /tmp/sofia.log
  kill \$SPID                  # stop Sofia
  exit                          # poweroff QEMU
=========================================================
TIPS

exec sh -i
EOF

echo "Booting Hi3516EV200 with sensor=${SENSOR}…"
echo "(Ctrl-A x to kill QEMU)"

# Wait for the kernel to reach the /bin/sh prompt before piping the
# setup script.  Without the lead-in sleep, our commands hit the
# serial port before sh exists and are dropped on the floor.  After
# the setup runs, `cat` keeps stdin open so the user gets an
# interactive shell.
{
    sleep 8
    printf '%s\n' "${GUEST_SETUP}"
    cat
} | env INIT=/bin/sh SENSOR="${SENSOR}" bash "${SCRIPT_DIR}/run-ev200.sh"
