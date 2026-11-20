#!/usr/bin/env python3
"""Rebuild a flat SPI NAND image for the rv1126 / rv1109 machines from a
camera's logical NAND dump.

The RV1109 dump this was written for (docs/rockchip.md) is what a dumper
reading the MTD device with bad blocks skipped leaves:

  - it starts at NAND 0x300000: the first 3 MiB -- the ID block with the
    DDR blob, TPL and SPL -- are not in it (the machine's SPL stand-in
    loads U-Boot from the uboot partition instead);
  - every bad block is missing, so whatever follows one sits too early;
  - the primary GPT, in the missing start, is gone, and the backup GPT
    is the dump's last sector, short of the device's last LBA.

The rebuilt image is the flash the SFC's gd5f2gq4ub model serves
(256 MiB, data only), erased (FF) wherever the dump has nothing:

  - the dump's partitions are taken from its backup GPT; each one's
    blocks go to the partition's own start.  A UBI partition is placed
    by the image sequence number in its erase-counter headers, so the
    blocks skipped inside it become erased blocks at its end, which UBI
    takes as free;
  - the backup GPT goes to the device's last LBA and a primary GPT,
    rebuilt from it, to LBA 1, with both headers' CRCs fixed, behind a
    protective MBR at LBA 0 (U-Boot's part_test_efi wants its 0xEE
    entry before it looks at the GPT).

    python3 qemu-boot/rv1126-nand-image.py full.dump nand.bin
        [--dump-offset 0x300000] [--device-size 0xff80000]

--device-size is the MTD device U-Boot sees (the part's 256 MiB less the
four blocks Rockchip's driver keeps for its bad-block table).
"""
import argparse, struct, sys, zlib

BLOCK = 0x20000
FLASH = 0x10000000


def gpt_header_at(d, off):
    return d[off:off + 8] == b'EFI PART'


def find_backup_gpt(d):
    for off in range(len(d) - 512, max(len(d) - 0x100000, 0), -512):
        if gpt_header_at(d, off):
            return off
    sys.exit('no backup GPT near the end of the dump')


def parse_gpt(hdr, entries):
    _, _, hsize, _, _, cur, alt, first, last = struct.unpack_from(
        '<8sIIIIQQQQ', hdr)
    ent_lba, nent, esz = struct.unpack_from('<QII', hdr, 72)
    parts = []
    for i in range(nent):
        e = entries[i * esz:(i + 1) * esz]
        if e[:16] == bytes(16):
            continue
        start, end = struct.unpack_from('<QQ', e, 32)
        name = e[56:128].decode('utf-16le').rstrip('\0')
        parts.append((name, start * 512, (end - start + 1) * 512))
    return hsize, nent, esz, parts


def make_header(hdr, hsize, cur, alt, ent_lba, ent_crc):
    h = bytearray(hdr[:hsize])
    struct.pack_into('<QQ', h, 24, cur, alt)
    struct.pack_into('<Q', h, 72, ent_lba)
    struct.pack_into('<I', h, 88, ent_crc)
    struct.pack_into('<I', h, 16, 0)
    struct.pack_into('<I', h, 16, zlib.crc32(bytes(h)))
    return bytes(h) + bytes(512 - hsize)


def ubi_seq(d, off):
    if d[off:off + 4] != b'UBI#':
        return None
    return struct.unpack_from('>I', d, off + 24)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dump')
    ap.add_argument('out')
    ap.add_argument('--dump-offset', type=lambda x: int(x, 0),
                    default=0x300000)
    ap.add_argument('--device-size', type=lambda x: int(x, 0),
                    default=0xff80000)
    a = ap.parse_args()

    d = open(a.dump, 'rb').read()
    img = bytearray(b'\xff' * FLASH)
    base = a.dump_offset

    bk = find_backup_gpt(d)
    hdr = d[bk:bk + 512]
    hsize, nent, esz, parts = parse_gpt(hdr, d[bk - nent_bytes(hdr):bk])
    print('backup GPT at dump 0x%x: %d partitions' % (bk, len(parts)))

    # Walk the dump in partition order.  Before the first UBI partition
    # the dump is in place (nothing skipped); from there on, each UBI
    # partition's run of blocks with its own image sequence is placed at
    # the partition's start.
    src = 0
    for name, start, size in parts:
        if start >= a.device_size:
            print('  %-10s 0x%08x  past the device' % (name, start))
            continue
        if start + size > a.device_size:
            size = a.device_size - start - 34 * 512
        if start < base:
            print('  %-10s 0x%08x  not in the dump' % (name, start))
            continue
        seq = ubi_seq(d, start - base) if start - base < len(d) else None
        if src == 0 and seq is None:
            # still in place: copy straight
            n = min(size, len(d) - (start - base))
            img[start:start + n] = d[start - base:start - base + n]
            print('  %-10s 0x%08x  0x%x bytes in place' % (name, start, n))
            continue
        if src == 0:
            src = start - base
        seq = ubi_seq(d, src)
        n = 0
        while (src < len(d) and n < size and
               (seq is None or ubi_seq(d, src) == seq)):
            img[start + n:start + n + BLOCK] = d[src:src + BLOCK]
            src += BLOCK
            n += BLOCK
        missing = (size - n) // BLOCK
        print('  %-10s 0x%08x  %d blocks%s' % (
            name, start, n // BLOCK,
            ', %d skipped (bad) block(s) left erased' % missing
            if missing and src < len(d) else ''))

    # GPTs: backup at the device's last LBA, primary at LBA 1
    last = a.device_size // 512 - 1
    ent_len = nent_bytes(hdr)
    entries = d[bk - ent_len:bk]
    ecrc = zlib.crc32(entries)
    img[last * 512 - ent_len:last * 512] = entries
    img[last * 512:last * 512 + 512] = make_header(
        hdr, hsize, last, 1, last - ent_len // 512, ecrc)
    img[1024:1024 + ent_len] = entries
    mbr = bytearray(512)
    struct.pack_into('<B3sB3sII', mbr, 446, 0, b'\x00\x02\x00', 0xee,
                     b'\xff\xff\xff', 1, min(last, 0xffffffff))
    mbr[510:512] = b'\x55\xaa'
    img[0:512] = mbr
    img[512:1024] = make_header(hdr, hsize, 1, last, 2, ecrc)
    open(a.out, 'wb').write(img)
    print('wrote %s: GPT at LBA 1 and 0x%x' % (a.out, last))


def nent_bytes(hdr):
    _, nent, esz = struct.unpack_from('<QII', hdr, 72)
    return nent * esz


if __name__ == '__main__':
    main()
