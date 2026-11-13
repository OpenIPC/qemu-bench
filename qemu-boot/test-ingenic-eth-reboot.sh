#!/bin/bash
# Reproduce OpenIPC/firmware#2839 (T40N: after a soft reboot eth0 is gone,
# only a power cycle brings it back) on the Ingenic machine, and check a
# kernel that fixes it (docs/ingenic.md, "PHY power-down and reboot").
#
# On the board: ingenic_mac_close() -- every ifdown, so the `reboot` that
# runs rcK and S40network stop -- sets the PHY's BMCR.PDOWN.  The SoC's
# warm reset does not reach the PHY, which then drives no RMII REF_CLK, so
# the next kernel's probe finds DmaBusMode.SWR stuck ("Bus Mode Reg after
# reset: 0x00020101" x10, "PROB:synopGMAC_reset timeout") and registers no
# eth0.  `reboot -f` skips rcK, the PHY stays up and eth0 survives.  The
# GMAC model does the same (hw/net/ingenic-gmac.c: the PHY keeps its state
# across a system reset, swr-needs-phy-clk), with the PHY's reset GPIO not
# wired -- that board -- unless --phy-rst-gpio says otherwise.
#
# The release is booted as test-ingenic-boot.sh --mode kernel does it
# (-kernel uImage, root=/dev/mtdblock3 on an 8 MiB NOR image, -nic user),
# with one init script added to a copy of its root (no login, no EULA):
# on the first boot it writes a marker to /overlay, waits for the DHCP
# lease and reboots (`reboot` or `reboot -f`, from qemu_eth_reboot= on the
# command line); on the second it prints `ip link show eth0` and the
# driver's probe lines from dmesg.  Two runs, side by side:
#
#   reboot -f keeps eth0   must PASS: a regression otherwise (exit 1);
#   reboot keeps eth0      the bug.  On the release it FAILs, reported as
#                          the known failure (with the SWR timeout seen);
#                          with --expect-fixed it must PASS too (exit 1
#                          otherwise), for checking a kernel fix (--kernel)
#                          or a wired reset line (--phy-rst-gpio 71: PC7,
#                          the release DT's ingenic,rst-gpio).
#
# Both runs must boot twice with eth0 and a lease on the first boot.
# Mind the time: on the second boot, with the overlay now holding data,
# /init takes one to four minutes before the JFFS2 mount completes (the
# first boot, on an erased rootfs_data, takes seconds), so a run takes
# 4-8 minutes.
#
#   bash qemu-boot/test-ingenic-eth-reboot.sh [--soc t40] [--kernel uImage]
#       [--rootfs rootfs.squashfs] [--qemu ./qemu-src/build/qemu-system-mipsel]
#       [--dir qemu-boot] [--timeout 600] [--phy-rst-gpio N] [--expect-fixed]
#       [--keep DIR]
set -euo pipefail

SOC=t40
QEMU=./qemu-src/build/qemu-system-mipsel
DIR=qemu-boot
KERNEL=
ROOTFS=
TIMEOUT=600
PHYRST=
EXPECT_FIXED=0
KEEP=
while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC=$2; shift 2 ;;
        --kernel) KERNEL=$2; shift 2 ;;
        --rootfs) ROOTFS=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --dir) DIR=$2; shift 2 ;;
        --timeout) TIMEOUT=$2; shift 2 ;;
        --phy-rst-gpio) PHYRST=$2; shift 2 ;;
        --expect-fixed) EXPECT_FIXED=1; shift ;;
        --keep) KEEP=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
case $SOC in
    t40|t41) MEM="mem=64M rmem=64M@0x4000000"; RAM=128M ;;
    *)       MEM="mem=40M rmem=24M@0x2800000"; RAM=64M ;;
esac
KERNEL=${KERNEL:-$DIR/uImage.$SOC}
ROOTFS=${ROOTFS:-$DIR/rootfs.squashfs.$SOC}
for f in "$KERNEL" "$ROOTFS"; do
    [ -f "$f" ] || { echo "$f missing (qemu-boot/fetch-openipc-lite.sh" \
                     "$SOC $DIR)" >&2; exit 2; }
done

WORK=$(mktemp -d)
QPIDS=()
cleanup() {
    for p in "${QPIDS[@]}"; do kill "$p" 2>/dev/null || true; done
    if [ -n "$KEEP" ]; then
        mkdir -p "$KEEP" && cp "$WORK"/*.log "$KEEP"/ 2>/dev/null || true
    fi
    rm -rf "$WORK"
}
trap cleanup EXIT

# The root with the reboot script.  unsquashfs as a user is enough: the
# image has no device nodes (devtmpfs) and one file not owned by root.
unsquashfs -q -n -d "$WORK/root" "$ROOTFS" > /dev/null
cat > "$WORK/root/etc/init.d/S99zqemueth" <<'EOF'
#!/bin/sh
# qemu-boot/test-ingenic-eth-reboot.sh: boot 1 reboots, boot 2 reports.
[ "$1" = start ] || exit 0
how=$(sed -n 's/.*qemu_eth_reboot=\([a-z]*\).*/\1/p' /proc/cmdline)
[ -n "$how" ] || exit 0
mark=/overlay/qemu-eth-reboot
[ -d /overlay ] || mark=/etc/qemu-eth-reboot
(
    exec > /dev/console 2>&1
    eth() {
        if [ -e /sys/class/net/eth0 ]; then
            echo "QEMU-ETH: boot $1: eth0 present"
        else
            echo "QEMU-ETH: boot $1: eth0 missing"
        fi
        ip link show eth0
    }
    probe() {   # klogd empties the kernel's ring: syslogd's has them too
        { dmesg; logread; } 2>/dev/null |
            grep -E 'synopGMAC|Bus Mode|dwc-mac|mac-hw-rst' |
            sed "s/^/QEMU-ETH: dmesg: /"
    }
    lease() {
        i=0
        while [ $i -lt 60 ] && [ -e /sys/class/net/eth0 ] &&
              ! ip -4 addr show eth0 | grep -q inet; do
            sleep 1; i=$((i + 1))
        done
        ip -4 addr show eth0 2>/dev/null | grep inet |
            sed "s/^/QEMU-ETH: addr: /"
    }
    if [ ! -f "$mark" ]; then
        echo 1 > "$mark"; sync
        lease
        eth 1
        sleep 5
        echo "QEMU-ETH: boot 1: rebooting ($how)"
        sync
        if [ "$how" = force ]; then reboot -f; else reboot; fi
    else
        echo 2 > "$mark"; sync
        probe
        lease
        eth 2
        echo "QEMU-ETH: done"
    fi
) &
EOF
chmod 755 "$WORK/root/etc/init.d/S99zqemueth"
COMP=$(unsquashfs -s "$ROOTFS" | sed -n 's/^Compression //p')
BLOCK=$(unsquashfs -s "$ROOTFS" | sed -n 's/^Block size //p')
PSEUDO=()
while read -r mode own path; do
    PSEUDO+=(-p "${path#squashfs-root/} m $mode ${own%/*} ${own#*/}")
done < <(unsquashfs -lln "$ROOTFS" | awk '$2 != "0/0" && $6 != "squashfs-root" {
    m = 0; s = substr($1, 2)
    for (i = 0; i < 9; i++) {
        c = substr(s, i + 1, 1)
        if (c != "-") m += 2 ^ (8 - i)
    }
    printf "%o %s %s\n", m, $2, $6 }')
mksquashfs "$WORK/root" "$WORK/rootfs.sqfs" -comp "$COMP" -b "$BLOCK" \
    -all-root "${PSEUDO[@]}" -noappend -quiet > /dev/null
if [ "$(stat -c %s "$WORK/rootfs.sqfs")" -gt $((5120 << 10)) ]; then
    echo "the rebuilt root does not fit the 5120k rootfs partition" >&2
    exit 2
fi

APPEND="$MEM console=ttyS1,115200n8 panic=20 root=/dev/mtdblock3"
APPEND="$APPEND rootfstype=squashfs init=/init"
APPEND="$APPEND mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)"
GLOBAL=()
[ -n "$PHYRST" ] && GLOBAL=(-global "ingenic-gmac.phy-rst-gpio=$PHYRST")

# One run per reboot flavour, in parallel: its own NOR image and log.
start() {
    local how=$1 img=$WORK/flash-$1.bin
    head -c $((8 << 20)) /dev/zero | tr '\0' '\377' > "$img"
    dd if="$KERNEL" of="$img" bs=64k seek=5 conv=notrunc status=none
    dd if="$WORK/rootfs.sqfs" of="$img" bs=64k seek=37 conv=notrunc status=none
    timeout "$TIMEOUT" "$QEMU" -M "$SOC,flash-file=$img" -m $RAM \
        -kernel "$KERNEL" -append "$APPEND qemu_eth_reboot=$how" \
        "${GLOBAL[@]}" -display none -monitor none \
        -serial "file:$WORK/$how.log" -nic user,restrict=on &
    QPIDS+=($!)
}
start force
start clean

for _ in $(seq "$TIMEOUT"); do
    done=0
    for how in force clean; do
        grep -aq 'QEMU-ETH: done' "$WORK/$how.log" 2>/dev/null &&
            done=$((done + 1))
    done
    alive=0
    for p in "${QPIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ $done -eq 2 ] || [ $alive -eq 0 ] && break
    sleep 1
done
for p in "${QPIDS[@]}"; do kill "$p" 2>/dev/null || true; wait "$p" 2>/dev/null || true; done
QPIDS=()

fail=0
# ok WHAT LOG: one run's checks; prints them, returns 0 if eth0 survived
ok() {
    local what=$1 log=$2 good=0
    local boot1 boot2
    boot1=$(sed -n '1,/QEMU-ETH: boot 1: rebooting/p' "$log")
    boot2=$(sed -n '/QEMU-ETH: boot 1: rebooting/,$p' "$log")
    say() { echo "    $1: $2"; }
    if grep -aq 'QEMU-ETH: boot 1: eth0 present' <<< "$boot1" &&
       grep -aq 'lease of 10.0.2.15' <<< "$boot1"; then
        say PASS "boot 1: eth0 and a DHCP lease"
    else
        say FAIL "boot 1: eth0 and a DHCP lease"; good=2
    fi
    if [ "$(grep -ac 'Linux version' "$log")" -ge 2 ] &&
       grep -aq 'QEMU-ETH: done' <<< "$boot2"; then
        say PASS "boot 2 reached the report"
    else
        say FAIL "boot 2 reached the report"; good=2
    fi
    grep -a 'QEMU-ETH: boot 2\|QEMU-ETH: dmesg' <<< "$boot2" |
        grep -av 'cnt=[1-8]' | head -6 | sed 's/^/      /'
    if grep -aq 'synopGMAC_reset timeout' <<< "$boot2"; then
        say INFO "boot 2: DmaBusMode.SWR timed out (firmware#2839)"
    fi
    if grep -aq 'QEMU-ETH: boot 2: eth0 present' <<< "$boot2" &&
       grep -aq 'lease of 10.0.2.15' <<< "$boot2"; then
        [ $good -eq 0 ] || good=2
    else
        [ $good -eq 2 ] || good=1
    fi
    return $good
}

echo "$SOC, eth0 across a reboot${PHYRST:+ (phy-rst-gpio=$PHYRST)}:"
echo "  reboot -f (no ifdown):"
if ok force "$WORK/force.log"; then
    echo "  PASS: reboot -f keeps eth0"
else
    echo "  FAIL: reboot -f keeps eth0"
    fail=1
fi
echo "  reboot (rcK, ifdown eth0):"
r=0; ok clean "$WORK/clean.log" || r=$?
if [ $r -eq 0 ]; then
    echo "  PASS: reboot keeps eth0"
    [ $EXPECT_FIXED -eq 1 ] ||
        echo "  NOTE: firmware#2839 not reproduced: a fixed kernel or a wired" \
             "PHY reset? (--expect-fixed to require it)"
elif [ $r -eq 1 ] && [ $EXPECT_FIXED -eq 0 ]; then
    echo "  FAIL: reboot keeps eth0 (known: OpenIPC/firmware#2839," \
         "not counted; --expect-fixed counts it)"
else
    echo "  FAIL: reboot keeps eth0"
    fail=1
fi
if grep -aq 'Kernel panic' "$WORK/force.log" "$WORK/clean.log"; then
    echo "  FAIL: no kernel panic"
    fail=1
fi
if [ $fail -ne 0 ]; then
    for how in force clean; do
        echo "--- $how: serial log (tail) ---"
        tail -25 "$WORK/$how.log"
    done
fi
exit $fail
