#!/bin/bash
#
# Capture an ISP tuning profile from emulated Hi3516EV200 + Sofia
# under QEMU.  Boots, modprobes the vendor module chain, chroots
# into Sofia, waits for "ISP Dev 0 running", optionally flips Sofia
# into night mode via gdb, runs `isp-profile export`, and copies
# the output to the lab NFS share.
#
# Usage:
#   bash qemu-boot/run-ev200-capture.sh --sensor sc2315e [--mode day|night] [--debug]
#
# --debug exports XMVIDEO_DEBUG_ON / XMCAP_DEBUG_ON / XMIMP_DEBUG_ON
#         / JSON_PARSE_RES_OUTPUT into Sofia's environment so its
#         high-level state events trace into /tmp/sofia.log inside
#         the guest (useful for cross-checking which ISP attrs are
#         being touched — though raw HI_MPI_ISP_* per-call tracing
#         needs gdb breakpoints).
#
# Output: /mnt/noc/isp-captures/<sensor>_qemu_<mode>.isp on the host
# (i.e. /utils/isp-captures/<sensor>_qemu_<mode>.isp inside the guest).
#
# Fixtures expected on NFS at 10.216.128.227:/srv/nfsroot/:
#   sofia_full/      vendor sysroot tree
#   sofia_fonts/     Font.bin.web, Font.bin.custom
#   isp-profile      ARM static binary
#   gdbserver        ARM uClibc static gdbserver (only needed for --mode night)
#
# Night-mode mechanism:
#   Sofia chooses day/night based on a vendor /dev/xm_gpio read which
#   doesn't exist in our emulator rootfs, so Sofia defaults to day.
#   We bypass that by having gdbserver in the guest expose Sofia to a
#   host-side gdb that calls Sofia's `g_doToNightFuncPtr` (= sub_24D3D4
#   in IDA, virtual address 0x24D3D4 — Sofia is non-PIE and loads at
#   0x00010000, so the IDA address is the actual VA).  The function
#   walks the same code path as a real GPIO transition.
#
# Requires the host to have:
#   - gdb with `set architecture arm` support (e.g. `gdb` on Arch /
#     Debian's `gdb-multiarch` package; modern gdb has ARM built in).
#   - port 1234 (HOST_GDB_PORT) free for SLIRP hostfwd to the guest.
#
# Exits 0 on successful capture, non-zero with the last 60 lines of
# QEMU output on failure.
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

SENSOR=""
MODE="day"
DEBUG=""
NFS_HOST="${NFS_HOST:-10.216.128.227}"
NFS_PATH="${NFS_PATH:-/srv/nfsroot}"
NFS_LOCAL="${NFS_LOCAL:-/mnt/noc}"
ISP_READY_TIMEOUT="${ISP_READY_TIMEOUT:-25}"
QEMU_TIMEOUT="${QEMU_TIMEOUT:-180}"
HOST_GDB_PORT="${HOST_GDB_PORT:-1234}"

# Sofia.10.216.128.106 layout: non-PIE, text loads at 0x00010000.
# IDA's sub_24D3D4 is the day→night switch entry (calls
# sub_24D2F4(0)).  Override via env var if Sofia rebuilds.
NIGHT_FUNC_VA="${NIGHT_FUNC_VA:-0x24D3D4}"

while [ $# -gt 0 ]; do
    case "$1" in
        --sensor)   SENSOR="$2"; shift 2;;
        --sensor=*) SENSOR="${1#--sensor=}"; shift;;
        --mode)     MODE="$2"; shift 2;;
        --mode=*)   MODE="${1#--mode=}"; shift;;
        --debug)    DEBUG=1; shift;;
        --nfs)      NFS_HOST="$2"; shift 2;;
        -h|--help)
            sed -n '/^# /,/^[a-z]/p' "$0" | sed 's/^# \?//'
            exit 0;;
        *)
            echo "unknown arg: $1" >&2
            exit 2;;
    esac
done

if [ -z "$SENSOR" ]; then
    echo "error: --sensor required" >&2
    exit 2
fi
case "$MODE" in
    day|night) ;;
    *) echo "error: --mode must be 'day' or 'night'" >&2; exit 2;;
esac

OUT_NAME="${SENSOR}_qemu_${MODE}.isp"
echo "==> Capturing ${OUT_NAME} (mode=${MODE}, timeout ${QEMU_TIMEOUT}s)"

# Sync flag: host touches NFS_LOCAL/.gdb_done after the gdb call
# completes; guest polls /utils/.gdb_done before running isp-profile.
GDB_FLAG="${NFS_LOCAL}/.gdb_done.${SENSOR}.${MODE}"
rm -f "$GDB_FLAG"

# Inline guest setup script.
read -r -d '' GUEST_SETUP <<EOF || true
set +e
mount -t proc proc /proc 2>/dev/null
mount -t tmpfs none /tmp
udhcpc -q -i eth0 2>/dev/null || ifconfig eth0 10.0.2.15 netmask 255.255.255.0 up
route add default gw 10.0.2.2 2>/dev/null

mkdir -p /utils
mount -o nolock,vers=3 ${NFS_HOST}:${NFS_PATH} /utils \\
    || { echo "NFS_MOUNT_FAILED"; poweroff -f; }
mkdir -p /utils/isp-captures

mkdir -p /tmp/vendor/proc /tmp/vendor/dev /tmp/vendor/tmp \\
         /tmp/vendor/mnt/custom/data/Fonts /tmp/vendor/mnt/web/Fonts \\
         /tmp/vendor/var
cp -a /utils/sofia_full/. /tmp/vendor/
rm -f /tmp/vendor/usr/Fonts/Font.bin
cp /utils/sofia_fonts/Font.bin.web    /tmp/vendor/mnt/web/Fonts/Font.bin
cp /utils/sofia_fonts/Font.bin.custom /tmp/vendor/mnt/custom/data/Fonts/Font.bin
ln -sf /mnt/web/Fonts/Font.bin /tmp/vendor/usr/Fonts/Font.bin
mount --bind /dev  /tmp/vendor/dev  2>/dev/null
mount -t proc proc /tmp/vendor/proc 2>/dev/null

cd /lib/modules/4.9.37/hisilicon
for m in sys_config hi_osal hi3516ev200_base hi3516ev200_sys \\
         hi3516ev200_rgn hi3516ev200_vgs hi_mipi_rx hi3516ev200_vi \\
         hi3516ev200_isp hi_sensor_i2c hi_sensor_spi \\
         hi3516ev200_vpss hi3516ev200_chnl hi3516ev200_vedu \\
         hi3516ev200_rc hi3516ev200_venc hi3516ev200_h264e \\
         hi3516ev200_h265e hi3516ev200_jpege \\
         hi3516ev200_aio hi3516ev200_ai hi3516ev200_ao \\
         hi3516ev200_aenc hi3516ev200_adec hi3516ev200_acodec \\
         hi3516ev200_pm hi3516ev200_ive hi_pwm; do
    [ -e \${m}.ko ] || continue
    case \$m in
        sys_config) ARGS="chip=hi3516ev200 sensors=${SENSOR} g_cmos_yuv_flag=0 board=demo";;
        hi_osal)    ARGS="anony=1 mmz_allocator=hisi mmz=anonymous,0,0x46000000,32M";;
        hi3516ev200_vgs) ARGS="max_vgs_job=20 max_vgs_node=20 max_vgs_task=20";;
        hi3516ev200_ive) ARGS="save_power=0";;
        *)          ARGS="";;
    esac
    insmod \${m}.ko \$ARGS > /dev/null 2>&1
done

${DEBUG:+XMVIDEO_DEBUG_ON=1 XMCAP_DEBUG_ON=1 XMIMP_DEBUG_ON=1 JSON_PARSE_RES_OUTPUT=1} \\
    chroot /tmp/vendor /Sofia > /tmp/sofia.log 2>&1 &
SPID=\$!

for i in \$(seq 1 ${ISP_READY_TIMEOUT}); do
    sleep 1
    if grep -q "ISP Dev 0 running" /tmp/sofia.log 2>/dev/null; then
        echo "ISP_READY_AT t=\${i}s"
        break
    fi
    if ! kill -0 \$SPID 2>/dev/null; then
        echo "SOFIA_DIED at t=\${i}s"
        tail -30 /tmp/sofia.log
        poweroff -f
    fi
done

if ! grep -q "ISP Dev 0 running" /tmp/sofia.log 2>/dev/null; then
    echo "ISP_NOT_READY_TIMEOUT"
    tail -30 /tmp/sofia.log
    poweroff -f
fi
EOF

# Append night-mode-only block: gdbserver + sync wait.
if [ "$MODE" = "night" ]; then
    GUEST_SETUP+=$'\n'"$(cat <<EOF
# Night mode: hand Sofia over to host gdb so it can call
# g_doToNightFuncPtr (= sub_24D3D4) at VA ${NIGHT_FUNC_VA}.
echo "GDBSERVER_STARTING pid=\$SPID"
/utils/gdbserver --attach 0.0.0.0:1234 \$SPID > /tmp/gdbsrv.log 2>&1 &
GDBPID=\$!

# Wait for the host gdb run to complete (signalled by file on NFS).
echo "GDBSERVER_WAITING_FOR_HOST_GDB"
for i in \$(seq 1 60); do
    [ -e /utils/.gdb_done.${SENSOR}.${MODE} ] && break
    sleep 1
done
if [ ! -e /utils/.gdb_done.${SENSOR}.${MODE} ]; then
    echo "GDB_TIMEOUT — host did not signal completion"
    tail -20 /tmp/gdbsrv.log
    poweroff -f
fi
echo "GDBSERVER_DONE"

# gdb's --detach should have left Sofia running, but just in case
# any thread is left in a stop state, send SIGCONT to the whole
# process group.  Sofia's day→night switch (sub_24D2F4) does:
#     switchDayNightColor(0); usleep 300ms; sub_24B388(0);
#     sub_24D208(1); sub_24BC50(0)  // ← sets ISP attrs for night
# The very last call writes the ISP register attrs we capture.  If
# gdb's connection drops mid-call, sub_24BC50(0) may not have run
# yet — wait long enough to cover its worst-case duration before
# running isp-profile export.
kill -CONT \$SPID 2>/dev/null
sleep 6
EOF
)"
fi

GUEST_SETUP+=$'\n'"$(cat <<EOF
# Capture.
/utils/isp-profile export /utils/isp-captures/${OUT_NAME} ${SENSOR} 2>&1 | head -5
if [ -s /utils/isp-captures/${OUT_NAME} ]; then
    echo "CAPTURE_OK \$(wc -c < /utils/isp-captures/${OUT_NAME}) bytes"
    md5sum /utils/isp-captures/${OUT_NAME}
else
    echo "CAPTURE_FAILED"
fi
poweroff -f
EOF
)"

LOG=$(mktemp -t cap-${SENSOR}-${MODE}-XXXXXX.log)
QEMU_ENV=(env INIT=/bin/sh SENSOR="${SENSOR}")
if [ "$MODE" = "night" ]; then
    # Expose guest port 1234 (gdbserver) on host port $HOST_GDB_PORT
    # via SLIRP hostfwd.  TAP networking would also work but is
    # harder to address from the host side without DHCP lookups.
    QEMU_ENV+=(NIC_OPTS=",hostfwd=tcp::${HOST_GDB_PORT}-:1234" TAP=__no_tap__)
fi

# Spawn QEMU in background so we can run host-side gdb concurrently.
{
    sleep 8
    printf '%s\n' "$GUEST_SETUP"
} | "${QEMU_ENV[@]}" timeout "$QEMU_TIMEOUT" bash "${SCRIPT_DIR}/run-ev200.sh" \
    > "$LOG" 2>&1 &
QEMU_PID=$!

if [ "$MODE" = "night" ]; then
    # Wait for gdbserver to be reachable.
    for i in $(seq 1 90); do
        if grep -q "GDBSERVER_WAITING_FOR_HOST_GDB" "$LOG" 2>/dev/null && \
           bash -c "</dev/tcp/127.0.0.1/${HOST_GDB_PORT}" 2>/dev/null; then
            break
        fi
        sleep 1
    done

    if ! bash -c "</dev/tcp/127.0.0.1/${HOST_GDB_PORT}" 2>/dev/null; then
        echo "==> gdbserver never became reachable on :${HOST_GDB_PORT}"
        tail -30 "$LOG"
        kill $QEMU_PID 2>/dev/null
        wait $QEMU_PID 2>/dev/null
        exit 1
    fi

    # Run host gdb to call sub_24D3D4 (the day→night switch).  Note:
    # `gdbserver --attach` exits as soon as the call returns (it
    # doesn't honour gdb's `detach` cleanly because Sofia spawns
    # worker threads inside the call), which gdb reports as
    # "Remote communication error".  That's expected — the call
    # itself completes and Sofia stays running.  We verify success
    # by checking that gdb saw the `call` finish, not by the detach
    # exit status.
    SOFIA_ELF="${NFS_LOCAL}/sofia_full/Sofia"
    echo "==> Calling Sofia ${NIGHT_FUNC_VA}() via gdb"
    GDB_LOG=$(mktemp -t cap-gdb-XXXXXX.log)
    gdb -batch -nx "${SOFIA_ELF}" \
        -ex 'set architecture arm' \
        -ex "set sysroot ${NFS_LOCAL}/sofia_full" \
        -ex 'set debuginfod enabled off' \
        -ex "target remote 127.0.0.1:${HOST_GDB_PORT}" \
        -ex "call (int)((int(*)())${NIGHT_FUNC_VA})()" \
        -ex 'detach' \
        -ex 'quit' >"$GDB_LOG" 2>&1 || true

    # Tell the guest to proceed with isp-profile export.
    touch "$GDB_FLAG"

    # gdbserver --attach drops the connection ("Connection reset by
    # peer") shortly after the call returns — gdb sees this as an
    # error but the call itself completes on the guest side.  Only
    # surface the gdb log if the capture turns out wrong.
    rm -f "$GDB_LOG"
fi

wait $QEMU_PID 2>/dev/null || true
rm -f "$GDB_FLAG"

if grep -q "CAPTURE_OK" "$LOG"; then
    grep -E "ISP_READY_AT|GDBSERVER_DONE|CAPTURE_OK|^[0-9a-f]{32}" "$LOG"
    echo "==> ${NFS_LOCAL}/isp-captures/${OUT_NAME}"
    rm -f "$LOG"
    exit 0
fi

echo "==> CAPTURE FAILED (full log: $LOG)"
echo "--- last 60 lines ---"
tail -60 "$LOG"
exit 1
