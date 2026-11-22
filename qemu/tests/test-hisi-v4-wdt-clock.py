#!/usr/bin/env python3
"""Goke V500 (xm720) watchdog clock rate — cmsdk-apb-watchdog at 0x12030000.

The xm720 SDK's wtd.ko (WTD_SocStart/WTD_SocReset) programs the SP805
WDOGLOAD as timeout_s * 24000000 / 2, i.e. the watchdog runs at 24 MHz, not
the 3 MHz that was the family default.  A wrong rate makes every watchdog
timeout N-times too long: the SD-2N-4G's post-upgrade reboot is a feed-
starvation reset (hunter arms a 60 s dog and feeds it; the upgrade flow stops
feeding), so at 3 MHz the twin only rebooted ~8 min after an upgrade instead
of ~60 s.

This drives the watchdog under qtest (no guest) and checks the counter
decrements at 24 MHz by stepping the vm clock a known amount.

    python3 qemu/tests/test-hisi-v4-wdt-clock.py [--qemu PATH]
"""
import argparse
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from qtest_harness import QTest  # noqa: E402

WDT = 0x12030000
LOAD, VALUE, CONTROL, INTCLR, RIS, LOCK = 0x00, 0x04, 0x08, 0x0C, 0x10, 0xC00
UNLOCK = 0x1ACCE551
RATE = 24000000                 # Hz, from wtd.ko's 0x16e3600 constant
MACHINES = ('gk7205v500', 'gk7205v510', 'gk7205v530', 'gk7202v330')


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

    for machine in MACHINES:
        q = QTest(a.qemu, machine)
        try:
            step = lambda s: q.cmd('clock_step %d' % int(s * 1e9))
            check(q.readl(WDT + LOAD) == 0xFFFFFFFF and
                  q.readl(WDT + VALUE) == 0xFFFFFFFF,
                  '%s: LOAD/VALUE reset to 0xFFFFFFFF' % machine)

            # Arm with INTEN only (bit 0), no RESEN, so the counter just free-
            # runs from a full 0xFFFFFFFF load (~179 s at 24 MHz) and never
            # expires or resets the machine inside the measurement window.
            q.writel(WDT + LOCK, UNLOCK)
            q.writel(WDT + CONTROL, 0)
            q.writel(WDT + LOAD, 0xFFFFFFFF)
            q.writel(WDT + INTCLR, 1)
            q.writel(WDT + CONTROL, 1)
            q.writel(WDT + LOCK, 1)

            v0 = q.readl(WDT + VALUE)
            step(0.5)
            v1 = q.readl(WDT + VALUE)
            delta = v0 - v1
            expect = RATE // 2              # 0.5 s worth of counts
            # ±2% covers ptimer rounding; 3 MHz would give ~1.5 M, far outside.
            check(abs(delta - expect) < expect * 0.02,
                  '%s: 0.5 s vm clock -> %d counts (24 MHz ~= %d)'
                  % (machine, delta, expect))

            # Two-stage reset timing: a 2 s timeout (LOAD = RATE, one stage =
            # 1 s) must raise RIS at the first expiry ~1 s on, which at 3 MHz
            # (~8 s/stage) would not have happened yet.
            q.writel(WDT + LOCK, UNLOCK)
            q.writel(WDT + CONTROL, 0)
            q.writel(WDT + LOAD, RATE)
            q.writel(WDT + INTCLR, 1)
            q.writel(WDT + CONTROL, 3)      # INTEN | RESEN
            q.writel(WDT + LOCK, 1)
            step(0.8)
            check(q.readl(WDT + RIS) == 0,
                  '%s: no expiry before 1 s' % machine)
            step(0.4)                        # total 1.2 s
            check(q.readl(WDT + RIS) == 1,
                  '%s: first expiry ~1 s (RIS=1)' % machine)
            step(1.2)                        # past the 2nd expiry -> reset
            check(q.readl(WDT + LOAD) == 0xFFFFFFFF and
                  q.readl(WDT + LOCK) == 0,
                  '%s: unfed past 2nd expiry -> machine reset' % machine)
        finally:
            q.close()

    print('=== %s: goke V500 watchdog clock (24 MHz) ===' %
          ('PASS' if not fails else 'FAIL'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
