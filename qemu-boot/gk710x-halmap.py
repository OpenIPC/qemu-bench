#!/usr/bin/env python3
"""Map the Goke GK7101/GK7102 vendor HAL: fictitious register -> silicon.

The GK710x U-Boot and kernel never touch hardware directly.  Every register
access goes through a HAL blob that U-Boot carries (board/goke/gk7101/
gk7101_hal_data.h in the SDK, linked at 0xC0012000) and hands to the kernel.
Drivers use a fictitious map (AHB 0x90000000, APB 0xA0000000); the HAL moves
each block to the real one (0x60000000 / 0x70000000), shuffles registers
inside it and permutes the bits of some.  docs/goke-gk710x.md has the result.

This runs the HAL's hw_writel / hw_readl under Unicorn for every fictitious
address and prints where each lands, and how values are transformed when
they are not passed through:

    a000500c -> 70005010 W{0:2 1:3 2:0 3:1 4:6 6:4 7:- ...} R{...}

reads "the driver's bit 0 is written to silicon bit 2, ..., bit 7 is
dropped"; R{} maps the other way, silicon bit to the bit the driver sees.  The HAL comes from any
image holding it -- a raw blob, a U-Boot binary or a flash dump (whose words
are byte-swapped; both orders are tried).

    pip install unicorn
    gk710x-halmap.py uboot.bin                     # the whole map
    gk710x-halmap.py dump.bin a0005000-a0005020    # UART0 only
"""
import struct
import sys

from unicorn import Uc, UcError, UC_ARCH_ARM, UC_MODE_ARM
from unicorn import UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE, UC_MEM_WRITE
from unicorn.arm_const import (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2,
                               UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR)

HAL_BASE = 0xC0012000
HAL_SIZE = 0x4000
PROLOGUE = struct.pack('<4I', 0xE92D45F8, 0xE1A04000, 0xE1A05001, 0xE1A06002)
STOP = 0xC0F00000
DEFAULT_RANGES = [(0x90000000, 0x900C4000), (0xA0000000, 0xA0180000)]


def find_hal(img):
    for swap in (False, True):
        data = img
        if swap:
            n = len(img) // 4 * 4
            data = b''.join(img[i:i + 4][::-1] for i in range(0, n, 4))
        off = data.find(PROLOGUE)
        while off >= 0 and off % 4:
            off = data.find(PROLOGUE, off + 1)
        if off >= 0:
            return data[off:off + HAL_SIZE]
    sys.exit('no GK710x HAL (hal_init prologue) in the image')


class Hal:
    def __init__(self, blob):
        self.mu = Uc(UC_ARCH_ARM, UC_MODE_ARM)
        for base in (0xC0000000, 0x60000000, 0x70000000,
                     0x90000000, 0xA0000000):
            self.mu.mem_map(base, 0x01000000)
        self.mu.mem_write(HAL_BASE, blob)
        self.mu.mem_write(STOP, b'\xfe\xff\xff\xea')     # b .
        self.acc = []
        self.mu.hook_add(UC_HOOK_MEM_READ | UC_HOOK_MEM_WRITE, self._hook)
        # hal_init(real AHB, real APB, fictitious AHB, fictitious APB, 0):
        # 0 for the real bases selects the HAL's own 0x60000000/0x70000000.
        ops = self.call(HAL_BASE, 0, 0, 0x90000000, 0xA0000000, 0)
        fn = struct.unpack('<8I', self.mu.mem_read(ops, 32))
        self.version = self.call(fn[0])
        self.readl, self.writel = fn[4], fn[7]

    def _hook(self, uc, access, addr, size, value, data):
        if 0x60000000 <= addr < 0x80000000 or 0x90000000 <= addr < 0xB0000000:
            self.acc.append((access == UC_MEM_WRITE, addr, value))

    def call(self, fn, *args):
        regs = (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3)
        for reg, val in zip(regs, args[:4]):
            self.mu.reg_write(reg, val & 0xFFFFFFFF)
        sp = 0xC0E00000
        if len(args) > 4:
            sp -= 4
            self.mu.mem_write(sp, struct.pack('<I', args[4]))
        self.mu.reg_write(UC_ARM_REG_SP, sp)
        self.mu.reg_write(UC_ARM_REG_LR, STOP)
        self.acc = []
        self.mu.emu_start(fn, STOP, count=100000)
        return self.mu.reg_read(UC_ARM_REG_R0)

    def write(self, addr, val):
        """(real address, value written) of the HAL's last store, or None."""
        self.call(self.writel, val, addr)
        stores = [a for a in self.acc if a[0]]
        return (stores[-1][1], stores[-1][2]) if stores else None

    def read(self, addr, real, val):
        self.mu.mem_write(real, struct.pack('<I', val))
        return self.call(self.readl, addr)


def bitmap(f):
    """What a value transform f does to each bit, or None if nothing:
    bit index, '-' if the bit is dropped, hex if it spreads."""
    out = {}
    for i in range(32):
        v = f(1 << i)
        if v == 1 << i:
            continue
        if not v:
            out[i] = '-'
        elif not v & (v - 1):
            out[i] = str(v.bit_length() - 1)
        else:
            out[i] = hex(v)
    return out or None


def fmt(m):
    """'0:2 1:3 8-31:-': runs of dropped bits are folded."""
    out, run = [], None
    for k in sorted(m) + [None]:
        if k is not None and m[k] == '-' and run and run[1] == k - 1:
            run[1] = k
            continue
        if run:
            out.append('%d:-' % run[0] if run[0] == run[1]
                       else '%d-%d:-' % tuple(run))
            run = None
        if k is None:
            break
        if m[k] == '-':
            run = [k, k]
        else:
            out.append('%d:%s' % (k, m[k]))
    return ' '.join(out)


def main():
    hal = Hal(find_hal(open(sys.argv[1], 'rb').read()))
    ranges = [tuple(int(x, 16) for x in r.split('-')) for r in sys.argv[2:]]
    print('# HAL version %08x' % hal.version)
    for lo, hi in ranges or DEFAULT_RANGES:
        for addr in range(lo, hi, 4):
            try:
                hit = hal.write(addr, 0x12345678)
            except UcError as e:
                print('%08x error %s' % (addr, e))
                continue
            if not hit:
                continue
            real, val = hit
            if real == addr:
                continue                        # passed through untranslated
            line = '%08x -> %08x' % (addr, real)
            plain = (real & 0xFFF == addr & 0xFFF and val == 0x12345678 and
                     hal.read(addr, real, 0x9ABCDEF1) == 0x9ABCDEF1)
            if plain and not ranges and addr & 0xFFF:
                continue                        # the page's move is printed
            if not plain:
                w = bitmap(lambda v: hal.write(addr, v)[1])
                r = bitmap(lambda v: hal.read(addr, real, v))
                if w:
                    line += ' W{%s}' % fmt(w)
                if r:
                    line += ' R{%s}' % fmt(r)
            print(line)


if __name__ == '__main__':
    main()
