#!/usr/bin/env python3
"""Turn a Novatek NVTPACK firmware (FW9856xA.bin) into a raw flash image.

Novatek SDKs ship update packages, not flash dumps.  An NVTPACK starts
with the GUID {D6012E07-10BC-4F91-B28A-352F82261A50}; the partition count
is at 0x18, and a table of (offset, size, partition index) words follows
the 0x80-byte header.  Partition 1 is the device tree, and it says where
the rest go: its nor (or nand) node's nvtpack/index/idN/partition_name
names the partition_<name> node whose reg is that partition's flash range.

A SPI-NAND pack (566_NAND) becomes the NAND's data, every page's OOB left
erased, which is what `-M nt98566,flash-file=` takes for a 128 MiB image;
the UBI rootfs goes in as ubinize made it.

The layout matches what the vendor U-Boot's updater (nvt_ivot_fw_update.c)
writes.  The rootfs and app images carry a 64-byte CKSM header, which it
strips, and so does this; the rest (the FDT, the BCL1-compressed U-Boot,
the uImage) go to flash as they are.  The loader is not in the pack, and
`-M nt98562,flash-file=` boots without one: the machine's boot ROM does
the loader's job.  --loader puts LD9856xA.bin at 0 anyway, for a
complete image.

    nvtpack2flash.py FW98562A.bin nor.bin [--size MIB] [--loader LD98562A.bin]
"""
import argparse
import struct
import sys

GUID = bytes.fromhex('072e01d6bc10914fb28a352f82261a50')
FDT_MAGIC = 0xD00DFEED
CKSM = b'CKSM'


def fdt_nodes(blob):
    """Yield (path, {prop: value}) for every node of a flattened tree."""
    magic, _, off_struct, off_strings = struct.unpack_from('>IIII', blob)
    if magic != FDT_MAGIC:
        raise ValueError('partition 1 is not a device tree')
    pos = off_struct
    stack = []
    props = []

    def name_at(off):
        end = blob.index(b'\0', off_strings + off)
        return blob[off_strings + off:end].decode()

    while True:
        tag, = struct.unpack_from('>I', blob, pos)
        pos += 4
        if tag == 1:                                    # BEGIN_NODE
            end = blob.index(b'\0', pos)
            stack.append(blob[pos:end].decode())
            props.append({})
            pos = (end + 4) & ~3
        elif tag == 3:                                  # PROP
            length, nameoff = struct.unpack_from('>II', blob, pos)
            pos += 8
            props[-1][name_at(nameoff)] = blob[pos:pos + length]
            pos = (pos + length + 3) & ~3
        elif tag == 2:                                  # END_NODE
            yield '/'.join(stack), props.pop()
            stack.pop()
        elif tag == 4:                                  # NOP
            continue
        elif tag == 9:                                  # END
            return
        else:
            raise ValueError('bad FDT tag %d' % tag)


def flash_layout(fdt):
    """Map NVTPACK partition index -> (name, flash offset, size), and the
    flash size partition_all gives (None without one)."""
    nodes = dict(fdt_nodes(fdt))
    nor = next(p for p, v in nodes.items()
               if v.get('compatible', b'').startswith((b'nvt,nvt_spinor',
                                                       b'nvt,nvt_spinand')))
    layout = {}
    for path, props in nodes.items():
        prefix = nor + '/nvtpack/index/id'
        if not path.startswith(prefix) or 'partition_name' not in props:
            continue
        idx = int(path[len(prefix):])
        name = props['partition_name'].rstrip(b'\0').decode()
        reg = nodes.get(nor + '/partition_' + name, {}).get('reg')
        if reg is None:
            continue
        cells = struct.unpack('>%dI' % (len(reg) // 4), reg)
        # #address-cells = #size-cells = 2 under the nor node
        off = (cells[0] << 32) | cells[1]
        size = (cells[2] << 32) | cells[3]
        layout[idx] = (name, off, size)
    total = nodes.get(nor + '/partition_all', {}).get('reg')
    if total is not None:
        cells = struct.unpack('>4I', total)
        total = (cells[2] << 32) | cells[3]
    return layout, total


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('pack')
    ap.add_argument('out')
    ap.add_argument('--size', type=int,
                    help="flash MiB (default: the pack's partition_all)")
    ap.add_argument('--loader', help='LD9856xA.bin to put at offset 0')
    args = ap.parse_args()

    pack = open(args.pack, 'rb').read()
    if pack[:16] != GUID:
        sys.exit('%s: not an NVTPACK (GUID mismatch)' % args.pack)
    count, = struct.unpack_from('<I', pack, 0x18)
    parts = {}
    for i in range(count):
        off, size, idx = struct.unpack_from('<III', pack, 0x80 + 12 * i)
        parts[idx] = pack[off:off + size]
    if 1 not in parts:
        sys.exit('no device tree (partition 1) in the pack')

    layout, total = flash_layout(parts[1])
    size = args.size << 20 if args.size else total or 16 << 20
    img = bytearray(b'\xff' * size)
    if args.loader:
        ld = open(args.loader, 'rb').read()
        img[:len(ld)] = ld
    for idx, data in sorted(parts.items()):
        if idx not in layout:
            print('partition %d: not in the device tree, skipped' % idx)
            continue
        name, off, size = layout[idx]
        if data[:4] == CKSM:
            data = data[64:]
        if len(data) > size:
            sys.exit('partition %d (%s): 0x%x bytes do not fit 0x%x' %
                     (idx, name, len(data), size))
        if off + len(data) > len(img):
            sys.exit('partition %d (%s) ends past %d MiB' %
                     (idx, name, len(img) >> 20))
        img[off:off + len(data)] = data
        print('partition %d %-12s 0x%07x  0x%07x bytes' %
              (idx, name, off, len(data)))
    open(args.out, 'wb').write(img)


if __name__ == '__main__':
    main()
