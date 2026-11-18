#!/usr/bin/env python3
"""GK7206 (xmorca) RTC and watchdog block, driven the way the SDK kernel's
lotus/drivers/rtc/rtc-lotus.c and drivers/watchdog/sp805_wdt.c drive them.

- lotus-rtc at 0x120e0000: SPI-bridge RTC, counters at internal 0x01..0x06;
  starts at -rtc base, LORD.LOAD sets it, LORD.LOCK latches it, and it keeps
  time across a system reset (battery domain).
- lotus-wdt at 0x12030000: SP805 cores at 0x00/0x20/0x40, one lock at 0xC00;
  fed it stays armed, unfed the second expiry resets the machine.

No guest: QEMU runs under qtest and this script writes the registers.

    python3 qemu/tests/test-hisi-lotus-rtc-wdt.py [--qemu PATH]
"""
import argparse, calendar, os, sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
jt = import_module('test-xm-h1-jpeg')

RTC = 0x120e0000
SPI_RW = RTC + 4
WDT = 0x12030000
LOAD, VALUE, CONTROL, INTCLR, RIS, LOCK = 0x00, 0x04, 0x08, 0x0C, 0x10, 0xC00
UNLOCK = 0x1ACCE551
RATE = 24000000
BASE = calendar.timegm((2001, 2, 3, 4, 5, 6))


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

    q = jt.QTest(a.qemu, 'gk7206v11a',
                 ('-rtc', 'base=2001-02-03T04:05:06,clock=vm'))
    try:
        step = lambda s: q.cmd('clock_step %d' % int(s * 1e9))

        def spi_w(reg, v):
            q.writel(SPI_RW, (1 << 24) | (reg << 16) | (v & 0xff))
            return q.readl(SPI_RW) >> 31

        def spi_r(reg):
            q.writel(SPI_RW, (1 << 24) | (1 << 23) | (reg << 16))
            return (q.readl(SPI_RW) >> 8) & 0xff

        def rtc_time():
            spi_w(0x13, spi_r(0x13) | 2)            # LORD.LOCK: latch
            s, m, h, dl, dh = (spi_r(r) for r in range(2, 7))
            return (dl | dh << 8) * 86400 + h * 3600 + m * 60 + s

        def rtc_set(t):
            d, r = divmod(t, 86400)
            for reg, v in ((0x0d, 0), (0x0e, r % 60), (0x0f, r // 60 % 60),
                           (0x10, r // 3600), (0x11, d & 0xff), (0x12, d >> 8)):
                spi_w(reg, v)
            spi_w(0x13, 1)                          # LORD.LOAD
            return spi_r(0x13) & 1

        t0 = rtc_time()
        check(abs(t0 - BASE) <= 1, 'rtc: starts at -rtc base (%d vs %d)'
              % (t0, BASE))
        check(spi_r(0x13) & 3 == 0, 'rtc: LORD lock/load bits self-clear')
        step(5)
        check(rtc_time() - t0 == 5, 'rtc: 5 s of vm clock -> +5 s')
        SET = calendar.timegm((2031, 5, 6, 7, 8, 9))
        check(rtc_set(SET) == 0, 'rtc: LR_* + LORD.LOAD completes')
        check(rtc_time() == SET, 'rtc: reads back the loaded time')

        # Watchdog core 1 (wdt1, 0x12030020), the kernel's sequence.
        W1 = WDT + 0x20
        ids = [q.readl(WDT + o) for o in (0x00, 0x04, 0x20, 0x24, 0x40, 0x44)]
        check(ids == [0xFFFFFFFF] * 6,
              'wdt: three cores reset to LOAD/VALUE 0xFFFFFFFF')
        load = RATE // 2 * 2 - 1                    # timeout 2 s
        q.writel(WDT + LOCK, UNLOCK)
        q.writel(W1 + LOAD, load)
        q.writel(W1 + INTCLR, 1)
        q.writel(W1 + CONTROL, 3)
        q.writel(WDT + LOCK, 1)
        check(q.readl(WDT + LOCK) == 1, 'wdt: shared lock reads 1 when locked')
        q.writel(W1 + CONTROL, 0)
        check(q.readl(W1 + CONTROL) == 3, 'wdt: locked, CONTROL write ignored')
        for _ in range(10):                         # fed every 0.5 s for 5 s
            step(0.5)
            q.writel(WDT + LOCK, UNLOCK)
            q.writel(W1 + LOAD, load)
            q.writel(W1 + INTCLR, 1)
            q.writel(WDT + LOCK, 1)
        check(q.readl(W1 + CONTROL) == 3 and q.readl(W1 + LOAD) == load,
              'wdt: fed every 0.5 s for 5 s, still armed')
        step(1.2)
        check(q.readl(W1 + RIS) == 1, 'wdt: first expiry raises RIS')
        step(1.2)
        check(q.readl(W1 + CONTROL) == 0 and q.readl(W1 + LOAD) == 0xFFFFFFFF
              and q.readl(WDT + LOCK) == 0,
              'wdt: unfed past the 2nd expiry, the machine reset')
        t = rtc_time()
        check(SET + 7 <= t <= SET + 9,
              'rtc: kept its time across the reset (%+d s)' % (t - SET))
    finally:
        q.close()

    print('=== %s: lotus-rtc / lotus-wdt ===' % ('PASS' if not fails else 'FAIL'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
