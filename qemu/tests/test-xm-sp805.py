#!/usr/bin/env python3
"""The XM510/XM530 watchdog (arm-sp805) at 0x20060000: it identifies as an
ARM SP805, so Linux's sp805_wdt binds to it, and it resets the machine when
left unfed, and not while it is fed.

No guest: QEMU runs under qtest and this script writes the registers.

    python3 qemu/tests/test-xm-sp805.py [--qemu PATH]
"""
import argparse, os, sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
jt = import_module('test-xm-h1-jpeg')

WDT = 0x20060000
LOAD, VALUE, CONTROL, INTCLR, LOCK = 0x000, 0x004, 0x008, 0x00C, 0xC00
UNLOCK = 0x1ACCE551
INTEN, RESEN = 1, 2
# What a lab XM550 and XM510 read back at +0xFD0..+0xFFC.
SP805_ID = [0x00, 0x00, 0x00, 0x00, 0x05, 0x18, 0x14, 0x00,
            0x0D, 0xF0, 0x05, 0xB1]
RATE = {'xm510': 93750, 'xm530': 46875}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    for mach in ('xm510', 'xm530'):
        q = jt.QTest(a.qemu, mach)
        try:
            ids = [q.readl(WDT + 0xFD0 + 4 * i) & 0xFF for i in range(12)]
            check(ids == SP805_ID, '%s: reads the SP805 identity %s'
                  % (mach, ' '.join('%02x' % v for v in ids)))

            # One period is LOAD+1 ticks; the reset comes at the end of the
            # second. 1 s at the SoC's clock, so the reset is due at ~2 s.
            ticks = RATE[mach]
            step = lambda s: q.cmd('clock_step %d' % int(s * 1e9))
            q.writel(WDT + LOCK, UNLOCK)
            q.writel(WDT + LOAD, ticks)
            q.writel(WDT + CONTROL, INTEN | RESEN)
            q.writel(WDT + LOCK, 0)

            # Fed every half second for 5 s: never reaches a second expiry.
            for _ in range(10):
                step(0.5)
                q.writel(WDT + LOCK, UNLOCK)
                q.writel(WDT + INTCLR, 1)
                q.writel(WDT + LOCK, 0)
            check(q.readl(WDT + CONTROL) == INTEN | RESEN,
                  '%s: fed every 0.5 s for 5 s, it is still armed' % mach)

            # Unfed: the second expiry resets the machine, which puts the
            # watchdog itself back to its reset state.
            step(2.5)
            check(q.readl(WDT + CONTROL) == 0 and
                  q.readl(WDT + LOAD) == 0xFFFFFFFF,
                  '%s: unfed for 2.5 s with a 1 s period, the machine reset'
                  % mach)
        finally:
            q.close()

    print('=== %s: arm-sp805 ===' % ('PASS' if not fails else 'FAIL'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
