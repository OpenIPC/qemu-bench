#!/usr/bin/env python3
"""The V3s and V851s machines (hw/arm/allwinner-cam.c) and the Allwinner
models they share with the V831 one, driven the way OpenIPC's U-Boot and
the vendor kernels drive them:

- the boot ROM stand-in: eGON.BT0 from the SD card before the NOR, its
  boot_media word, the jump stub at 0 when the ROM sits there (V851s), and
  an SPL larger than the SRAM refused;
- aw-ccu: PLL lock bits, the sun6i layout's self-clearing update bits and
  reset presets (V3s);
- aw-dramc: single-rank training (the dual-rank probe fails);
- aw-dramcom: the DRAM window aliases as the programmed geometry maps it
  onto the part, and is plain RAM once the geometry is the part's;
- aw-wdt: rewriting MODE does not hold off a reset, the restart key does.

No guest: QEMU runs under qtest and this script writes the registers.

    python3 qemu/tests/test-allwinner-cam.py [--qemu PATH]
"""
import argparse, os, shutil, struct, subprocess, sys, tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
jt = import_module('test-xm-h1-jpeg')

DRAM = 0x40000000
SOC = {
    #          SRAM     SRAM size  CCU         DRAMC (PHY)  DRAM COM    WDT
    'v3s':   (0x00000, 0x10000, 0x01C20000, 0x01C63000, 0x01C62000,
              0x01C20CA0),
    'v851s': (0x20000, 0x40000, 0x02001000, 0x03103000, 0x03102000,
              0x020500A0),
}
EGON_BOOT_MEDIA = 0x28
PIR, PGSR0, DTCR, DX0GSR0 = 0x000, 0x010, 0x0C0, 0x348
WDT_CTRL, WDT_CFG, WDT_MODE = 0x10, 0x14, 0x18
WDT_KEY = 0xA57 << 1


def egon(tag, size=0x2000):
    """An eGON.BT0 image of `size` bytes, `tag` right after the header."""
    img = bytearray(size)
    img[4:12] = b'eGON.BT0'
    struct.pack_into('<I', img, 0x10, size)
    img[0x60:0x60 + len(tag)] = tag
    return bytes(img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    a = ap.parse_args()
    fails = 0
    tmp = tempfile.mkdtemp()

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    def images(mach, nor_spl, sd_spl=None):
        nor = os.path.join(tmp, mach + '-nor.bin')
        with open(nor, 'wb') as f:
            f.write(nor_spl + b'\xff' * ((8 << 20) - len(nor_spl)))
        extra = []
        if sd_spl:
            sd = os.path.join(tmp, mach + '-sd.img')
            with open(sd, 'wb') as f:
                f.write(bytes(0x2000) + sd_spl +
                        bytes((16 << 20) - 0x2000 - len(sd_spl)))
            extra = ['-drive', 'if=sd,format=raw,file=' + sd]
        return '%s,flash-file=%s' % (mach, nor), extra

    for mach, (sram, sram_size, ccu, dramc, com, wdt) in SOC.items():
        # The NOR alone: its SPL in SRAM, boot_media 3 (SPI NOR)
        m, extra = images(mach, egon(b'NOR'))
        q = jt.QTest(a.qemu, m, extra)
        try:
            check(q.read(sram + 4, 8) == b'eGON.BT0' and
                  q.read(sram + 0x60, 3) == b'NOR' and
                  q.read(sram + EGON_BOOT_MEDIA, 1) == b'\x03',
                  '%s: the NOR SPL is in SRAM at %#x, boot_media 3'
                  % (mach, sram))
            if sram:
                check(q.readl(0) == 0xE51FF004 and q.readl(4) == sram,
                      '%s: the ROM stub at 0 jumps to %#x' % (mach, sram))

            # aw-ccu: an enabled PLL reads locked
            q.writel(ccu + 0x010, 1 << 31)
            check(q.readl(ccu + 0x010) & (1 << 28),
                  '%s: an enabled PLL reads locked' % mach)
            if mach == 'v3s':
                check(q.readl(ccu + 0x028) == 0x90041811 and
                      q.readl(ccu + 0x020) == 0,
                      'v3s: PLL_PERIPH0 at 0x028 runs from reset, PLL_DDR '
                      'at 0x020 is off')
                q.writel(ccu + 0x0F4, 1 << 16)
                q.writel(ccu + 0x020, (1 << 31) | (1 << 20))
                check(not q.readl(ccu + 0x0F4) & (1 << 16) and
                      not q.readl(ccu + 0x020) & (1 << 20),
                      'v3s: the MBUS and PLL_DDR update bits self-clear')

            # aw-dramc: asked for two ranks, rank 1 fails gate training
            q.writel(dramc + DTCR, 3 << 24)
            q.writel(dramc + PIR, 1)
            check(q.readl(dramc + PGSR0) & (1 << 22) and
                  (q.readl(dramc + DX0GSR0) >> 24) & 3 == 2,
                  '%s: dual-rank training fails (single-rank part)' % mach)

            # aw-dramcom: 4 KiB pages programmed on a 2 KiB-page part, so
            # column bit 11 is not there and 0x800 aliases 0
            q.writel(DRAM, 0x11111111)
            q.writel(com, (9 << 8) | (12 << 4))
            q.writel(DRAM + 0x800, 0xA5A5A5A5)
            check(q.readl(DRAM) == 0xA5A5A5A5,
                  '%s: with 4 KiB pages programmed, +0x800 aliases +0' % mach)
            q.writel(com, (8 << 8) | (12 << 4))
            q.writel(DRAM + 0x800, 0x5A5A5A5A)
            check(q.readl(DRAM) == 0xA5A5A5A5 and
                  q.readl(DRAM + 0x800) == 0x5A5A5A5A,
                  '%s: with the part\'s geometry, plain RAM' % mach)

            # aw-wdt: restarted with the key every 0.2 s, it stays armed
            step = lambda s: q.cmd('clock_step %d' % int(s * 1e9))
            q.writel(wdt + WDT_CFG, 1)
            q.writel(wdt + WDT_MODE, 1)         # 0.5 s, enabled
            for _ in range(10):
                step(0.2)
                q.writel(wdt + WDT_CTRL, WDT_KEY | 1)
            check(q.readl(wdt + WDT_MODE) == 1,
                  '%s: restarted every 0.2 s for 2 s, the watchdog is still '
                  'armed' % mach)
            # MODE rewritten as a restart loop does, at 0.2 and 0.4 s: the
            # reset still comes at 0.5 s (a reload on each write would put
            # it at 0.9 s), and puts the watchdog back to its reset state
            q.writel(wdt + WDT_CTRL, WDT_KEY | 1)
            for _ in range(2):
                step(0.2)
                q.writel(wdt + WDT_MODE, 1)
            step(0.2)
            check(q.readl(wdt + WDT_MODE) == 0,
                  '%s: MODE rewritten at 0.2 and 0.4 s, the machine resets '
                  'at 0.5 s' % mach)
        finally:
            q.close()

        # An SD card with an SPL wins over the NOR: boot_media 0
        m, extra = images(mach, egon(b'NOR'), egon(b'SD!'))
        q = jt.QTest(a.qemu, m, extra)
        try:
            check(q.read(sram + 0x60, 3) == b'SD!' and
                  q.read(sram + EGON_BOOT_MEDIA, 1) == b'\x00',
                  '%s: the SD card SPL wins, boot_media 0' % mach)
        finally:
            q.close()

    # An SPL past the V3s's 64 KiB of SRAM is refused, not truncated
    m, extra = images('v3s', egon(b'BIG', 0x14000))
    r = subprocess.run([a.qemu, '-M', m, '-machine', 'accel=qtest',
                        '-display', 'none', '-nodefaults', '-qtest', 'null',
                        '-serial', 'null'], capture_output=True, text=True,
                       timeout=20)
    check(r.returncode == 1 and 'over the' in r.stderr,
          'v3s: an 80 KiB SPL is refused (%s)' % r.stderr.strip())

    shutil.rmtree(tmp, ignore_errors=True)
    print('=== %s: allwinner-cam ===' % ('PASS' if not fails else 'FAIL'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
