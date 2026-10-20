#!/usr/bin/env python3
"""Register-level checks of the Novatek NT9856x models, no guest needed.

QEMU runs the nt98562 (and nt98566) machine under qtest and this script
plays the vendor drivers (docs/novatek.md):

  - nvt-top: the chip ID nvt_get_chip_id() and U-Boot read, pinmux
    storage;
  - nvt-cg: the PLL8 ratio and CPU clock select the clock driver turns
    into periph_clk, which must be the 100 MHz QEMU's A9 timers tick at;
    PLL lock status following PLL_EN;
  - nvt-wdt: a CTRL write without the key is lost; an interrupt-mode
    expiry raises SPI 57 and the status write-back acknowledges it; a
    reset-mode expiry and the kernel's manual reset both reset the machine;
  - nvt-gpio: SET / CLR / DIR / DATA, input pads, a rising-edge interrupt
    on C_GPIO 3 and a level-low one on D_GPIO 0 through SPI 24; the "out"
    lines follow the pad drivers through direction changes and a reset;
  - nvt-rtc: a kept-alive RTC (key, oscillator) reading the host's year;
    a CSET commits the time and raises SPI 56; the daily alarm and the
    driver's wake alarm (PWRALM, committed by CSET) go off; powering off with the
    power-on alarm armed is the vendor reboot, and resets the machine;
  - the 16550 UART at its reg-shift-2 offsets;
  - the generated device tree: compatible, the clock nodes in the order
    of_clk_init() needs, the pinmux children in the driver's order;
  - outside qtest: machine state migrates to a file and back.

    python3 qemu/tests/test-novatek.py [--qemu PATH]
"""
import argparse
import os
import socket
import subprocess
import sys
import tempfile
import time

TOP = 0xF0010000
CG = 0xF0020000
WDT = 0xF0050000
RTC = 0xF0060000
GPIO = 0xF0070000
UART0 = 0xF0290000
GICD = 0xFFD01000

SPI_GPIO = 24
SPI_RTC = 56
SPI_WDT = 57

WDT_KEY = 0x5A96 << 16


class QTest:
    def __init__(self, qemu, machine, extra=()):
        self.dir = tempfile.mkdtemp()
        path = os.path.join(self.dir, 'qtest.sock')
        srv = socket.socket(socket.AF_UNIX)
        srv.bind(path)
        srv.listen(1)
        self.proc = subprocess.Popen(
            [qemu, '-M', machine, '-machine', 'accel=qtest',
             '-display', 'none', '-nodefaults', '-qtest', 'unix:' + path] +
            list(extra),
            stdout=subprocess.DEVNULL,
            stderr=open(os.path.join(self.dir, 'qemu.err'), 'w'))
        srv.settimeout(20)
        self.sock, _ = srv.accept()
        self.f = self.sock.makefile('rw', buffering=1)
        self.irqs = {}

    def cmd(self, line):
        self.f.write(line + '\n')
        self.f.flush()
        while True:
            r = self.f.readline().strip()
            if r.startswith('IRQ '):
                _, how, n = r.split()
                self.irqs[int(n)] = how == 'raise'
            elif r.startswith(('OK', 'FAIL', 'ERR')):
                return r

    def writel(self, a, v):
        self.cmd('writel 0x%x 0x%x' % (a, v))

    def readl(self, a):
        return int(self.cmd('readl 0x%x' % a).split()[1], 16)

    def step(self, ns):
        self.cmd('clock_step %d' % ns)

    def pin(self, n, level):
        self.cmd('set_irq_in /machine/nvt-gpio in %d %d' % (n, level))

    def close(self):
        self.proc.kill()
        self.proc.wait()


failures = 0


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    shown = got if isinstance(got, (bool, str)) else hex(got)
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what, shown))


def gic_enable(qt, spi):
    n = 32 + spi
    qt.writel(GICD + 0x100 + (n // 32) * 4, 1 << (n % 32))


def gic_pending(qt, spi):
    n = 32 + spi
    return bool(qt.readl(GICD + 0x200 + (n // 32) * 4) & (1 << (n % 32)))


def test_top_cg(qt):
    print('nvt-top, nvt-cg')
    check('chip ID 0x7021 in TOP_VERSION[31:16]', qt.readl(TOP + 0xF0) >> 16,
          0x7021)
    qt.writel(TOP + 0xF0, 0)
    check('TOP_VERSION is read-only', qt.readl(TOP + 0xF0) >> 16, 0x7021)
    qt.writel(TOP + 0x04, 0x12345678)
    check('pinmux word keeps its value', qt.readl(TOP + 0x04), 0x12345678)

    check('CPU clock from PLL8', qt.readl(CG + 0x10) & 3, 1)
    ratio = (qt.readl(CG + 0x4420) & 0xFF) | \
        ((qt.readl(CG + 0x4424) & 0xFF) << 8) | \
        ((qt.readl(CG + 0x4428) & 0xFF) << 16)
    periph = 12000000 * ratio // 131072 // 8
    check('periph_clk = PLL8 / 8 within 1 kHz of 100 MHz',
          abs(periph - 100000000) < 1000, True)
    qt.writel(CG + 0x00, 0x00000100)
    check('PLL_STAT follows PLL_EN', qt.readl(CG + 0x04), 0x00000100)


def test_wdt(qt):
    print('nvt-wdt')
    gic_enable(qt, SPI_WDT)
    qt.writel(WDT + 0x00, 0x00000001)
    check('CTRL without the key is ignored', qt.readl(WDT + 0x00), 0)
    # MSB 0: 0xFFF ticks of 12 MHz / 1024, 0.35 s.  Interrupt mode.
    qt.writel(WDT + 0x00, WDT_KEY | 0x01)
    check('STS reports enabled', bool(qt.readl(WDT + 0x04) & (1 << 30)),
          True)
    qt.step(200 * 1000 * 1000)
    check('counting up', 0 < (qt.readl(WDT + 0x04) & 0xFFFFF) < 0xFFF, True)
    qt.writel(WDT + 0x08, 1)
    check('TRIG restarts the count', (qt.readl(WDT + 0x04) & 0xFFFFF) < 2,
          True)
    qt.step(400 * 1000 * 1000)
    check('expired', bool(qt.readl(WDT + 0x04) & (1 << 31)), True)
    check('SPI 57 pending', gic_pending(qt, SPI_WDT), True)
    qt.writel(WDT + 0x04, qt.readl(WDT + 0x04))
    check('status write-back acknowledges', gic_pending(qt, SPI_WDT), False)
    qt.writel(WDT + 0x00, WDT_KEY)

    # A reset puts CG back to its reset value: use that as the witness.
    qt.writel(CG + 0x10, 0)
    qt.writel(WDT + 0x84, 0xCAFE)
    qt.writel(WDT + 0x00, WDT_KEY | 0x03)
    qt.step(400 * 1000 * 1000)
    check('reset-mode expiry resets the machine', qt.readl(CG + 0x10) & 3, 1)
    check('watchdog off after the reset', qt.readl(WDT + 0x00), 0)
    check('UDATA survives it', qt.readl(WDT + 0x84), 0xCAFE)

    # plat-na51089/reset.c: nvt_trigger_wdt_external()
    qt.writel(CG + 0x10, 0)
    qt.writel(WDT + 0x00, 0x5A960112)
    qt.writel(WDT + 0x00, 0x5A960113)
    qt.writel(WDT + 0x0C, 1)
    qt.step(1000)
    check('manual reset resets the machine', qt.readl(CG + 0x10) & 3, 1)


def test_gpio(qt):
    print('nvt-gpio')
    gic_enable(qt, SPI_GPIO)
    qt.cmd('irq_intercept_out /machine/nvt-gpio out')
    # P_GPIO 5 (bank 1) as an output
    qt.writel(GPIO + 0x24, 1 << 5)
    qt.writel(GPIO + 0x44, 1 << 5)
    check('SET drives the pad high', qt.readl(GPIO + 0x04) & (1 << 5),
          1 << 5)
    qt.writel(GPIO + 0x64, 1 << 5)
    check('CLR drives it low', qt.readl(GPIO + 0x04) & (1 << 5), 0)
    check('DIR reads back', qt.readl(GPIO + 0x24), 1 << 5)
    qt.pin(0x20 + 6, 1)
    check('an input pad reads its level', qt.readl(GPIO + 0x04) & (1 << 6),
          1 << 6)
    qt.pin(0x20 + 5, 1)
    check('an output pad ignores its input', qt.readl(GPIO + 0x04) &
          (1 << 5), 0)
    # out line 37 = P_GPIO 5: high output, to input, latch cleared, output
    qt.writel(GPIO + 0x44, 1 << 5)
    check('out line follows a high output', qt.irqs.get(0x25), True)
    qt.pin(0x20 + 5, 0)
    qt.writel(GPIO + 0x24, 0)
    check('released as an input', qt.irqs.get(0x25), False)
    qt.writel(GPIO + 0x64, 1 << 5)
    qt.writel(GPIO + 0x24, 1 << 5)
    check('a low output again stays low', qt.irqs.get(0x25), False)
    check('DATA agrees', qt.readl(GPIO + 0x04) & (1 << 5), 0)
    qt.writel(GPIO + 0x44, 1 << 5)

    # Interrupt 0 is C_GPIO 3: rising edge
    qt.writel(GPIO + 0xA0, 0)
    qt.writel(GPIO + 0xA4, 0)
    qt.writel(GPIO + 0xA8, 0)
    qt.writel(GPIO + 0x90, 1)
    qt.pin(3, 1)
    check('rising edge on C_GPIO 3 latches STS bit 0',
          qt.readl(GPIO + 0x80) & 1, 1)
    check('SPI 24 pending', gic_pending(qt, SPI_GPIO), True)
    qt.writel(GPIO + 0x80, 1)
    check('write-1-to-clear', qt.readl(GPIO + 0x80) & 1, 0)
    check('SPI 24 released', gic_pending(qt, SPI_GPIO), False)
    qt.pin(3, 0)
    check('the falling edge does not fire', qt.readl(GPIO + 0x80) & 1, 0)
    qt.writel(GPIO + 0x90, 0)

    # Interrupt 32 is D_GPIO 0 (DGPIO bit 0): level low, stays pending
    # while the pad is low
    qt.writel(GPIO + 0xE0, 1)
    qt.writel(GPIO + 0xE4, 1)
    qt.writel(GPIO + 0xD0, 1)
    check('level-low D_GPIO 0 latches DGPIO STS bit 0',
          qt.readl(GPIO + 0xC0) & 1, 1)
    qt.writel(GPIO + 0xC0, 1)
    check('and re-latches while the level holds', qt.readl(GPIO + 0xC0) & 1,
          1)
    qt.pin(0x80, 1)
    qt.writel(GPIO + 0xC0, 1)
    check('cleared once the pad goes high', qt.readl(GPIO + 0xC0) & 1, 0)
    check('SPI 24 idle', gic_pending(qt, SPI_GPIO), False)

    # A reset makes every pin an input again (manual watchdog reset)
    check('P_GPIO 5 still driven high', qt.irqs.get(0x25), True)
    qt.writel(WDT + 0x00, WDT_KEY | 0x01)
    qt.writel(WDT + 0x0C, 1)
    qt.step(1000)
    check('reset releases it', qt.irqs.get(0x25), False)


def rtc_tod(qt):
    t = qt.readl(RTC + 0x00)
    return (t & 0x3F) + ((t >> 6) & 0x3F) * 60 + ((t >> 12) & 0x1F) * 3600


def rtc_year(qt):
    days = qt.readl(RTC + 0x04) & 0xFFFF
    return 1900 + int(days / 365.2425)


def test_rtc(qt):
    print('nvt-rtc')
    check('key 0xA: no power loss', (qt.readl(RTC + 0x04) >> 16) & 0xF, 0xA)
    check('oscillator setting 0x9', qt.readl(RTC + 0x24) & 0xFF, 0x9)
    check('host year', rtc_year(qt), time.gmtime().tm_year)
    gic_enable(qt, SPI_RTC)
    # rtc-na51055.c set_time: select, shadow writes, CSET with interrupt
    qt.writel(RTC + 0x10, (1 << 3) | (1 << 4))
    qt.writel(RTC + 0x00, 5 | (4 << 6) | (3 << 12))
    qt.writel(RTC + 0x04, (0xA << 16) | 36524)      # 2000-01-01
    qt.writel(RTC + 0x10, qt.readl(RTC + 0x10) | (1 << 2) | (1 << 8))
    check('CSET done', bool(qt.readl(RTC + 0x0C) & (1 << 8)), True)
    check('CSET bit self-clears', qt.readl(RTC + 0x10) & (1 << 2), 0)
    check('SPI 56 pending', gic_pending(qt, SPI_RTC), True)
    qt.writel(RTC + 0x0C, 1 << 8)
    check('acknowledged', gic_pending(qt, SPI_RTC), False)
    check('time committed', qt.readl(RTC + 0x00) & 0x1FFC0,
          (4 << 6) | (3 << 12))
    check('day committed', qt.readl(RTC + 0x04) & 0xFFFF, 36524)
    # The daily alarm, 3 s ahead, with the alarm interrupt on
    tod = rtc_tod(qt) + 3
    qt.writel(RTC + 0x08, (tod % 60) | ((tod // 60 % 60) << 6) |
              ((tod // 3600 % 24) << 12))
    qt.writel(RTC + 0x10, 1)
    qt.step(1500 * 1000 * 1000)
    check('daily alarm not yet', qt.readl(RTC + 0x0C) & 1, 0)
    qt.step(3000 * 1000 * 1000)
    check('daily alarm goes off', qt.readl(RTC + 0x0C) & 1, 1)
    check('SPI 56 pending for it', gic_pending(qt, SPI_RTC), True)
    qt.writel(RTC + 0x0C, 1)
    check('acknowledged', gic_pending(qt, SPI_RTC), False)
    qt.writel(RTC + 0x08, 0x1FFFF)

    # rtc-na51055.c set_alarm: the wake alarm in PWRALM, through a CSET
    day = qt.readl(RTC + 0x04) & 0xFFFF
    tod = rtc_tod(qt) + 3
    qt.writel(RTC + 0x10, (1 << 6) | (1 << 7) | 1)
    qt.writel(RTC + 0x20, (tod % 60) | ((tod // 60 % 60) << 6) |
              ((tod // 3600 % 24) << 12) | ((day & 0x1F) << 20))
    check('PWRALM is shadowed until the CSET', qt.readl(RTC + 0x20), 0)
    qt.writel(RTC + 0x10, qt.readl(RTC + 0x10) | (1 << 2))
    qt.writel(RTC + 0x0C, 1 << 8)
    qt.step(4500 * 1000 * 1000)
    check('wake alarm goes off', qt.readl(RTC + 0x0C) & 1, 1)
    qt.writel(RTC + 0x0C, 1)

    # plat-na51089/reset.c reboot: arm the power-on alarm, power off
    qt.writel(CG + 0x10, 0)
    qt.writel(RTC + 0x18, 1 << 3)
    check('alarm armed', qt.readl(RTC + 0x1C) & 1, 1)
    check('enable bit self-clears', qt.readl(RTC + 0x18) & (1 << 3), 0)
    qt.writel(RTC + 0x18, 1 << 1)
    qt.step(1000)
    check('power off with the alarm armed resets', qt.readl(CG + 0x10) & 3, 1)
    check('the RTC keeps its day across it',
          qt.readl(RTC + 0x04) & 0xFFFF, 36524)


def test_uart(qt):
    print('UART (16550, reg-shift 2)')
    check('LSR: transmitter empty', qt.readl(UART0 + 0x14) & 0x60, 0x60)
    qt.writel(UART0 + 0x0C, 0x83)
    qt.writel(UART0 + 0x00, 13)
    qt.writel(UART0 + 0x0C, 0x03)
    check('LCR reads back', qt.readl(UART0 + 0x0C), 0x03)


def find_nodes(dtb, names):
    pos = [dtb.find(n.encode() + b'\0') for n in names]
    return pos


def test_fdt(qemu, tmp):
    print('generated device tree')
    kernel = os.path.join(tmp, 'kernel.bin')
    open(kernel, 'wb').write(b'\xfe\xff\xff\xea')       # b .
    dtb = os.path.join(tmp, 'nt98562.dtb')
    subprocess.run([qemu, '-M', 'nt98562', '-machine', 'dumpdtb=' + dtb,
                    '-kernel', kernel, '-display', 'none', '-nodefaults'],
                   capture_output=True, timeout=30)
    data = open(dtb, 'rb').read() if os.path.exists(dtb) else b''
    check('dumped', data[:4] == b'\xd0\x0d\xfe\xed', True)
    check('root compatible matches mach-nvt-ivot',
          b'novatek,na51089\0nvt,ca9\0' in data, True)
    cg, pclk = find_nodes(data, ['cg@f0020000', 'periph_clk'])
    check('nvt,core_clk before nvt,periph_clk', 0 < cg < pclk, True)
    order = find_nodes(data, ['sdio', 'sdio2', 'uart', 'eth', 'misc'])
    check('pinmux children in PIN_FUNC order',
          all(0 < a < b for a, b in zip(order, order[1:])), True)
    check('chip-ID node for nvt_get_chip_id()', b'nvt,nvt_top' in data, True)


def hmp(qemu, args, cmds):
    """Run QEMU with an HMP monitor on stdio; return what it printed."""
    p = subprocess.run([qemu] + args + ['-display', 'none', '-nodefaults',
                                        '-monitor', 'stdio'],
                       input='\n'.join(cmds + ['quit']) + '\n',
                       capture_output=True, text=True, timeout=60)
    return p.stdout + p.stderr


def test_migration(qemu, tmp):
    print('migration')
    for m in ('nt98562', 'nt98566'):
        mig = os.path.join(tmp, m + '.mig')
        out = hmp(qemu, ['-M', m, '-S'],
                  ['migrate file:' + mig, 'info migrate'])
        check('%s: state saved' % m, 'completed' in out and
              'Assertion' not in out and os.path.getsize(mig) > 0, True)
        out = hmp(qemu, ['-M', m, '-S', '-incoming', 'file:' + mig],
                  ['info status'])
        check('%s: and loaded back' % m, 'error' not in out.lower() and
              'Assertion' not in out, True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default='./qemu-src/build/qemu-system-arm')
    args = ap.parse_args()
    tmp = tempfile.mkdtemp()

    # The RTC on the virtual clock, so clock_step moves it.
    qt = QTest(args.qemu, 'nt98562', ['-rtc', 'clock=vm'])
    try:
        test_top_cg(qt)
        test_wdt(qt)
        test_gpio(qt)
        test_rtc(qt)
        test_uart(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        print('nt98566')
        check('same die, same chip ID', qt.readl(TOP + 0xF0) >> 16, 0x7021)
    finally:
        qt.close()

    test_fdt(args.qemu, tmp)
    test_migration(args.qemu, tmp)

    print('%d failure(s)' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
