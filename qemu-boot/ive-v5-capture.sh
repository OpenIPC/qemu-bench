#!/bin/bash
# Reference outputs of the V5 IVE ops from a real board, and the same from
# the emulated Hi3516CV610, compared file by file (#268).
#
# Builds ive-v5-capture.c statically against the SDK's MPI libraries and
# sample helpers, then:
#
#   --board HOST   copies it to a cv608 / cv610 camera running OpenIPC
#                  (ssh root@HOST, ive.ko loaded by load_hisilicon), stops
#                  majestic for the run (it holds the MMZ), runs it, fetches
#                  the outputs into DIR/board, and starts majestic again
#   --qemu         runs it on the emulated hi3516cv610 under the OpenIPC
#                  hi3516cv6xx release (initramfs); the guest prints the
#                  outputs as base64 between markers, unpacked into DIR/qemu
#
# and compares them: every file one side wrote must be byte-identical on
# the other.  Without --board the comparison uses the outputs a CV608
# gave, kept in qemu/tests/data/ive-v5-cv608-ref.tar.xz, so a model change
# can be checked against silicon without the camera.
#
# This runs locally, not in CI: the binary links the SDK's MPI libraries,
# which are not public.  What CI guards is the same arithmetic through the
# register test (qemu/tests/test-hisi-ive-npu.py), which computes every
# op's expected output in Python from the rules this capture established.
# So a regression in an op fails CI; what needs the camera, and this
# script, is establishing or revising a rule.
#
#   ive-v5-capture.sh --sdk SDK [--board HOST] [--qemu] [--work DIR] [--qemu-bin Q]
#
# Needs arm-openipc-linux-musleabi-gcc (or CC), ssh, unsquashfs, cpio.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
SDK= BOARD= RUNQEMU=0 WORK=
QEMU=$HERE/../qemu-src/build/qemu-system-arm
CC=${CC:-arm-openipc-linux-musleabi-gcc}

while [ $# -gt 0 ]; do
    case $1 in
    --sdk) SDK=$2; shift ;;
    --board) BOARD=$2; shift ;;
    --qemu) RUNQEMU=1 ;;
    --work) WORK=$2; shift ;;
    --qemu-bin) QEMU=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
SRC=$SDK/smp/a7_linux/source
OUT=$SRC/out
SMP=$SRC/mpp/sample
SVP=$SMP/svp
if [ ! -f "$OUT/lib/libss_mpi_ive.a" ]; then
    echo "--sdk: no Hi3516CV610 SDK (out/lib) in '$SDK'" >&2
    exit 2
fi
WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)
REL=hi3516cv6xx

# --- build ----------------------------------------------------------------
OBJ=$WORK/obj
rm -rf "$OBJ"
mkdir -p "$OBJ"
SNS=SC4336P_MIPI_4M_30FPS_10BIT
CFLAGS="-O2 -I$OUT/include -I$SMP/common -I$SVP/common -I$SVP/ive/sample
    -Dhi3516cv610 -DOT_XXXX -DISP_V2
    -DSENSOR0_TYPE=$SNS -DSENSOR1_TYPE=$SNS -D${SNS}_SELECT=y
    -DBOARD_TYPE=DMEB_QFN -fsigned-char -D_GNU_SOURCE
    -DVER_X=1 -DVER_Y=0 -DVER_Z=0 -DVER_P=0 -DVER_B=10
    -mcpu=cortex-a7 -mfloat-abi=softfp -mfpu=neon-vfpv4"
objs=()
for f in "$SMP"/common/{sample_comm_sys,sample_comm_isp,sample_comm_vi,sample_comm_vpss,loadbmp,ot_confaccess,sample_comm_venc,sample_comm_region,sample_comm_osd_drawline,sample_ipc}.c \
         "$SVP"/common/{sample_common_svp,sample_common_ive}.c \
         "$HERE/ive-v5-capture.c"; do
    o=$OBJ/$(basename "$f" .c).o
    # shellcheck disable=SC2086
    "$CC" $CFLAGS -c -o "$o" "$f"
    objs+=("$o")
done
L=$OUT/lib
"$CC" -static -o "$WORK/ive-v5-capture" "${objs[@]}" \
    -Wl,--start-group "$L"/libss_mpi.a "$L"/libss_mpi_ive.a \
    "$L"/libss_mpi_sysbind.a "$L"/libss_mpi_sysmem.a "$L"/libss_mpi_ae.a \
    "$L"/libss_mpi_isp.a "$L"/libot_mpi_isp.a "$L"/libss_mpi_awb.a \
    "$L"/libdehaze.a "$L"/libextend_stats.a "$L"/libdrc.a "$L"/libldci.a \
    "$L"/libbnr.a "$L"/libcalcflicker.a "$L"/libir_auto.a "$L"/libacs.a \
    "$L"/libsns_sc4336p.a "$L"/libot_osal.a "$L"/libss_mpi_syskol.a \
    "$L"/libsecurec.a -Wl,--end-group -lpthread -lm -ldl -lstdc++

# --- the board ------------------------------------------------------------
BOARD_RC=0
if [ -n "$BOARD" ]; then
    SSH="ssh -o BatchMode=yes -o ConnectTimeout=10 root@$BOARD"
    # majestic holds the MMZ, so the run needs it stopped -- and the camera
    # must not be left that way, whatever fails in between
    restart_majestic() {
        $SSH '/etc/init.d/S95majestic start >/dev/null 2>&1' ||
            echo "WARNING: could not start majestic on $BOARD again" >&2
    }
    trap restart_majestic EXIT
    $SSH 'cat > /tmp/ive-v5-capture && chmod +x /tmp/ive-v5-capture' \
        < "$WORK/ive-v5-capture"
    $SSH '/etc/init.d/S95majestic stop >/dev/null 2>&1; n=0
          while pidof majestic >/dev/null && [ $n -lt 20 ]; do sleep 1; n=$((n+1)); done
          rm -rf /tmp/ivecap; cd /tmp && ./ive-v5-capture /tmp/ivecap; rc=$?
          tar -C /tmp/ivecap -cf /tmp/ivecap.tar .; echo "rc=$rc" >&2' \
        > "$WORK/board.log" 2>&1 || true
    rm -rf "$WORK/board"
    mkdir -p "$WORK/board"
    $SSH 'cat /tmp/ivecap.tar' | tar -C "$WORK/board" -xf -
    $SSH 'rm -rf /tmp/ivecap /tmp/ivecap.tar /tmp/ive-v5-capture' || true
    BOARD_RC=$(sed -n 's/^rc=//p' "$WORK/board.log" | tail -1)
    BOARD_RC=${BOARD_RC:-1}
    echo "board: $(grep -c '^ok ' "$WORK/board.log") ok," \
         "$(grep -c '^FAIL' "$WORK/board.log") failed; $(ls "$WORK/board" | wc -l) files," \
         "capture exit $BOARD_RC"
fi

# --- QEMU -----------------------------------------------------------------
QEMU_RC=0
if [ "$RUNQEMU" = 1 ]; then
    if [ ! -f "$WORK/uImage.$REL" ]; then
        bash "$HERE/fetch-openipc-lite.sh" "$REL" "$WORK"
    fi
    INITRD=$WORK/initrd
    rm -rf "$INITRD"
    unsquashfs -q -d "$INITRD" "$WORK/rootfs.squashfs.$REL" >/dev/null
    cp "$WORK/ive-v5-capture" "$INITRD/usr/bin/"
    cat > "$INITRD/init" <<'EOF'
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t devtmpfs dev /dev
mount -t tmpfs tmp /tmp
load_hisilicon -i -sensor0 sc4336p
cd /tmp
ive-v5-capture /tmp/ivecap
echo "=== rc=$?"
echo "=== tar begin"
# Quiet the console first: the vendor modules print without a newline, so
# a message lands inside whatever line is being written -- and since its
# letters are valid base64, no prefix or filter on this side can tell it
# from data.  The B64: prefix stays as a second line of defence against
# anything that still gets through.
dmesg -n 1 2>/dev/null
tar -C /tmp/ivecap -cf - . | base64 | sed 's/^/B64:/'
echo "=== tar end"
echo b > /proc/sysrq-trigger
EOF
    chmod +x "$INITRD/init"
    (cd "$INITRD" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/cap.cpio"
    timeout 900 "$QEMU" -M hi3516cv610,sensor=sc4336p -m 128M \
        -kernel "$WORK/uImage.$REL" -initrd "$WORK/cap.cpio" \
        -append "console=ttyAMA0,115200 rdinit=/init mem=64M mmz_allocator=hisi mmz=anonymous,0,0x44000000,64M" \
        -icount shift=0 -nographic -serial mon:stdio -nic user,restrict=on \
        -no-reboot > "$WORK/qemu.log" 2>&1 || true
    rm -rf "$WORK/qemu"
    mkdir -p "$WORK/qemu"
    # only the leading base64 run of each prefixed line: a vendor module
    # writes to the console without a newline first, so its message lands
    # on the end of whatever line was being printed
    tr -d '\r' < "$WORK/qemu.log" |
        sed -n 's|^B64:\([A-Za-z0-9+/=]*\).*|\1|p' | base64 -d |
        tar -C "$WORK/qemu" -xf -
    QEMU_RC=$(sed -n 's/^=== rc=//p' "$WORK/qemu.log" | tr -d '\r' | tail -1)
    QEMU_RC=${QEMU_RC:-1}
    echo "qemu: $(grep -ac '^ok ' "$WORK/qemu.log") ok," \
         "$(grep -ac '^FAIL' "$WORK/qemu.log") failed; $(ls "$WORK/qemu" | wc -l) files," \
         "capture exit $QEMU_RC"
fi

# --- compare --------------------------------------------------------------
REF=$HERE/../qemu/tests/data/ive-v5-cv608-ref.tar.xz
if [ ! -d "$WORK/board" ] && [ -f "$REF" ]; then
    mkdir -p "$WORK/board"
    tar -C "$WORK/board" -xJf "$REF"
    echo "board: the stored CV608 reference ($(ls "$WORK/board" | wc -l) files)"
fi
if [ -d "$WORK/board" ] && [ -d "$WORK/qemu" ]; then
    python3 - "$WORK/board" "$WORK/qemu" <<'EOF'
import os, sys
b, q = sys.argv[1:]
same = diff = 0
for n in sorted(set(os.listdir(b)) | set(os.listdir(q))):
    pb, pq = os.path.join(b, n), os.path.join(q, n)
    x = open(pb, 'rb').read() if os.path.exists(pb) else None
    y = open(pq, 'rb').read() if os.path.exists(pq) else None
    if x is not None and x == y:
        same += 1
        continue
    diff += 1
    if x is None:
        print('EXTRA %s: only the QEMU run wrote it' % n)
    elif y is None:
        print('MISSING %s' % n)
    else:
        bad = sum(1 for i, j in zip(x, y) if i != j) + abs(len(x) - len(y))
        print('DIFF %s: %d of %d bytes' % (n, bad, len(x)))
print('%d identical, %d differ' % (same, diff))
sys.exit(1 if diff else 0)
EOF
    rc=$?
    if [ "$BOARD_RC" != 0 ] || [ "$QEMU_RC" != 0 ]; then
        echo "a capture itself failed (board $BOARD_RC, qemu $QEMU_RC):" \
             "the comparison above covers only what was written" >&2
        rc=1
    fi
    exit $rc
fi
