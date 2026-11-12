#!/bin/bash
# Fetch an OpenIPC lite NOR release and leave uImage.<soc> and
# rootfs.squashfs.<soc> in DIR (default: the current directory), whichever
# shape the release publishes it in.
#
# openipc.<soc>-nor-lite.tgz holds the two files.  Since OpenIPC/firmware#2638
# SoCs move, one nightly at a time, to openipc.<soc>-nor-lite-split.tgz,
# which holds firmware.bin.<soc>: the NOR image from the kernel partition on
# -- a legacy uImage at 0 (64-byte header, data size big-endian at byte 12),
# then the SquashFS at a later erase-block boundary.  That is split here.
# A SoC that moved can keep its last openipc.<soc>-nor-lite.tgz in the
# release, frozen at the move (hi3516ev200's at 2026-10-25), next to the
# nightly split bundle; others publish both, fresh.  So when both exist the
# newer one (Last-Modified) is taken.
#
#   bash qemu-boot/fetch-openipc-lite.sh <soc> [DIR] [RELEASE]
set -euo pipefail

SOC=$1
DIR=${2:-.}
REL=${3:-latest}
URL=https://github.com/OpenIPC/firmware/releases/download/$REL
TGZ=$(mktemp)
trap 'rm -f "$TGZ"' EXIT

# An asset's Last-Modified in seconds, or nothing when it is not published.
# Only a 404 means "not published": a request that failed for any other
# reason fails the fetch.  Taking it as absent would hand the choice to the
# other archive, and for a SoC that moved that is the frozen plain one -- a
# network blip would boot a weeks-old image without a word.
modified() {
    local h code d
    h=$(mktemp)
    code=$(curl -sIL --retry 3 -D "$h" -o /dev/null -w '%{http_code}' "$URL/$1") ||
        code=000
    d=$(tr -d '\r' < "$h" | sed -n 's/^[Ll]ast-[Mm]odified: //p' | tail -1)
    rm -f "$h"
    case $code in
    200) if [ -n "$d" ]; then date -d "$d" +%s; else echo 0; fi ;;
    404) ;;
    *) echo "$URL/$1: HTTP $code, cannot tell whether it is published" >&2
       return 1 ;;
    esac
}
PLAIN=$(modified "openipc.$SOC-nor-lite.tgz") || exit 1
SPLIT=$(modified "openipc.$SOC-nor-lite-split.tgz") || exit 1

if [ -n "$PLAIN" ] && [ "${SPLIT:-0}" -le "$PLAIN" ] &&
   curl -fsL --retry 3 -o "$TGZ" "$URL/openipc.$SOC-nor-lite.tgz"; then
    tar xzf "$TGZ" -C "$DIR"
elif curl -fsSL --retry 3 -o "$TGZ" "$URL/openipc.$SOC-nor-lite-split.tgz"; then
    tar xzf "$TGZ" -C "$DIR"
    FW=$DIR/firmware.bin.$SOC
    if [ "$(od -An -tx1 -N4 "$FW" | tr -d ' \n')" != 27051956 ]; then
        echo "$FW does not start with a uImage" >&2
        exit 1
    fi
    set -- $(od -An -tu1 -j12 -N4 "$FW")
    head -c $((64 + (($1 << 24) | ($2 << 16) | ($3 << 8) | $4))) "$FW" \
        > "$DIR/uImage.$SOC"
    # The SquashFS superblock's bytes_used (little-endian at +40), rounded
    # up to 4 KiB as the release's own rootfs files are: the erased flash
    # after it is left out.
    sq=$(grep -abo hsqs "$FW" | head -1 | cut -d: -f1)
    if [ -z "$sq" ]; then
        echo "$FW has no SquashFS" >&2
        exit 1
    fi
    set -- $(od -An -tu1 -j$((sq + 40)) -N4 "$FW")
    len=$(((($4 << 24) | ($3 << 16) | ($2 << 8) | $1) + 4095 & ~4095))
    tail -c +$((sq + 1)) "$FW" | head -c $len > "$DIR/rootfs.squashfs.$SOC"
else
    echo "$REL has no lite NOR release for $SOC" >&2
    exit 1
fi
ls -l "$DIR/uImage.$SOC" "$DIR/rootfs.squashfs.$SOC"
