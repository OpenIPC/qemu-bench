#!/usr/bin/env python3
"""Register-level checks of the Ingenic T-series models, no guest needed.

QEMU runs the machines under qtest and this script plays the boot ROM,
the SPL, U-Boot and the kernel drivers (docs/ingenic.md):

  - the mask ROM stand-in: the SPL from a flash image staged in the
    cache-as-RAM window, the stub at 0x1FC00000;
  - ingenic-cpm: PLL lock following enable in each SoC's encoding, the
    CPCCR change bits and CPCSR busy bits, divider BUSY bits (bit 27 in the
    T31 SSI divider, 28 elsewhere, untouched in the I2S M/N dividers), the
    post-U-Boot reset values the kernels' clock code needs;
  - ingenic-ddr: what the DWC (T20), Inno (T31), T40 and T41 DDR inits poll;
  - ingenic-tcu: the boot loader's OST at EXTAL/4 with its latched high
    half, and the watchdog's count;
  - ingenic-ost: the kernel's clock event (match, flag, wrap, IP4) and its
    64-bit clock source; the XBurst2 global and core layouts;
  - ingenic-intc / ingenic-ccu: masks and pending through an OST line;
  - ingenic-sfc + NOR: JEDEC ID by PIO, a PIO read (END before the FIFO is
    drained), a DMA read, erase and program written through, and on v2 a
    command-table read through a DMA descriptor at a KSEG1-style address;
  - ingenic-sfc + SPI NAND (ingenic-spinand): the mask ROM's NAND path,
    READ ID with and without the dummy byte, features (power-up lock, ECC
    and BUF bits), page read to cache with a status-poll phase on OIP and
    the polled status in DEV_STA_RT, read from cache of data and OOB (and
    the plane bit ignored by a one-plane part), program load + execute as
    one three-phase transfer, erase, P_FAIL/E_FAIL while locked, all
    written through to a data-only image; on v2 the kernel's command-table
    chain (read to cache, poll, read from cache by descriptor DMA) on an
    image with OOB, and a NAND on T41's second SFC;
  - ingenic-gpio: shadow-port function changes, outputs read back, inputs
    high; ingenic-i2c: an absent address aborts with TXABRT; ingenic-rtc:
    WRDY and the WENR unlock; ingenic-efuse: read done, blank fuses;
  - the SoC ID word at 0x1300002C and the grade codes (CPPSR, EFUSE
    SUBSOCTYPE words) ipctool tells the variants apart by;
  - ingenic-isp (T31, sensor=gc2053): the GC2053's ID on I2C0 at 0x37,
    CPM SRBC's ISP stop acknowledge, the VIC control's self-clearing reset
    and configure commands, the MSCA address FIFOs (push, pop, empty flag,
    clear) and a frame painted into a queued buffer under clock_step, with
    the ISP and VIC status bits and INTC sources 29 and 30; a frame
    through the T21, T23, T40 and T41 layouts; ingenic-isp-apical (T20): the slot
    events, a DMA writer's bank painted, the interrupt block, a dropped
    frame; the same on T10, whose VIC control is at 0x20;
  - every machine: realizes, and a migration round trip of t31 and t41.

    python3 qemu/tests/test-ingenic.py [--qemu PATH]
"""
import argparse
import json
import math
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile

CPM = 0x10000000
INTC = 0x10001000
TCU = 0x10002000
RTC = 0x10003000
GPIO = 0x10010000
I2C0 = 0x10050000
OST = 0x12000000
CORE_OST = 0x12100000
CCU = 0x12200000
CORE_INTC = 0x12300000
DDRPHY = 0x13011000
DDRAPB = 0x13012000
DDRC = 0x134F0000
SFC = 0x13440000
SFC1 = 0x13450000
EFUSE = 0x13540000
CAR = 0x1F000000
BOOTROM = 0x1FC00000
SCRATCH = 0x00800000
ISP = 0x13300000
SADC = 0x10070000
PDMA = 0x13420000
AIC = 0x10020000
CODEC = 0x10021000
VIC = 0x133E0000


class QTest:
    def __init__(self, qemu, machine, extra=()):
        self.dir = tempfile.mkdtemp()
        path = os.path.join(self.dir, 'qtest.sock')
        self.qmp_path = os.path.join(self.dir, 'qmp.sock')
        self.qmp_f = None
        srv = socket.socket(socket.AF_UNIX)
        srv.bind(path)
        srv.listen(1)
        self.proc = subprocess.Popen(
            [qemu, '-M', machine, '-machine', 'accel=qtest',
             '-display', 'none', '-nodefaults', '-qtest', 'unix:' + path,
             '-qmp', 'unix:%s,server=on,wait=off' % self.qmp_path] +
            list(extra),
            stdout=subprocess.DEVNULL,
            stderr=open(os.path.join(self.dir, 'qemu.err'), 'w'))
        srv.settimeout(20)
        self.sock, _ = srv.accept()
        self.f = self.sock.makefile('rw', buffering=1)

    def cmd(self, line):
        self.f.write(line + '\n')
        self.f.flush()
        while True:
            r = self.f.readline().strip()
            if r.startswith(('OK', 'FAIL', 'ERR')):
                return r

    def writel(self, a, v):
        self.cmd('writel 0x%x 0x%x' % (a, v))

    def readl(self, a):
        return int(self.cmd('readl 0x%x' % a).split()[1], 16)

    def write(self, a, data):
        self.cmd('write 0x%x 0x%x 0x%s' % (a, len(data), data.hex()))

    def read(self, a, n):
        return bytes.fromhex(self.cmd('read 0x%x 0x%x' % (a, n)).split()[1][2:])

    def step(self, ns):
        self.cmd('clock_step %d' % ns)

    def qmp(self, cmd, **args):
        if self.qmp_f is None:
            q = socket.socket(socket.AF_UNIX)
            q.connect(self.qmp_path)
            self.qmp_f = q.makefile('rw', buffering=1)
            self.qmp_f.readline()                   # the greeting
            self.qmp('qmp_capabilities')
        self.qmp_f.write(json.dumps({'execute': cmd, 'arguments': args}) +
                         '\n')
        self.qmp_f.flush()
        while True:
            r = json.loads(self.qmp_f.readline())
            if 'return' in r or 'error' in r:
                return r.get('return', r.get('error'))

    def qom_get(self, path, prop):
        return self.qmp('qom-get', path=path, property=prop)

    def qom_set(self, path, prop, value):
        return self.qmp('qom-set', path=path, property=prop, value=value)

    def close(self):
        self.proc.kill()
        self.proc.wait()
        shutil.rmtree(self.dir, ignore_errors=True)


failures = 0


def shown(v):
    if isinstance(v, (bool, str, tuple)):
        return str(v)
    if isinstance(v, (bytes, list)):
        return bytes(v)[:8].hex() + ('...' if len(v) > 8 else '')
    return hex(v)


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what, shown(got)))


def crc7(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x12) & 0xff if crc & 0x80 else (crc << 1) & 0xff
    return crc >> 1


def make_image(path, size=16 << 20):
    """An SPL header (06 05 04 03 02 55 aa 55 aa, CRC7, length) with
    'j .' at 0x800, and a byte pattern from 1 MiB on."""
    img = bytearray(b'\xFF' * size)
    spl_len = 0x1000
    img[0:spl_len] = bytes(spl_len)
    img[0:9] = bytes([0x06, 0x05, 0x04, 0x03, 0x02, 0x55, 0xaa, 0x55, 0xaa])
    struct.pack_into('<I', img, 0x800, 0x08000600)          # j 0x80001800
    struct.pack_into('<I', img, 12, spl_len)
    img[9] = crc7(img[0x800:spl_len])
    for i in range(0x100000, 0x110000):
        img[i] = i * 7 & 0xFF
    open(path, 'wb').write(img)
    return img


# --- the boot ROM ----------------------------------------------------------

def test_boot_rom(qt, img):
    print('mask ROM stand-in')
    check('SPL header in the cache-as-RAM window',
          qt.read(CAR + 0x1000, 9), bytes(img[0:9]))
    check('SPL code at 0x80001800', qt.readl(CAR + 0x1800), 0x08000600)
    check('DDR below it untouched', qt.readl(0x1800), 0)
    check('ROM stub present', qt.readl(BOOTROM) != 0, True)


# --- CPM -------------------------------------------------------------------

def test_cpm(qt, soc):
    print('ingenic-cpm (%s)' % soc)
    apll = qt.readl(CPM + 0x10)
    check('APLL out of reset: enabled and on', apll & 9, 9)
    check('CPCCR: SCLKA = APLL', qt.readl(CPM) >> 30, 2)
    check('DDRCDR mux is a clock, not stop', qt.readl(CPM + 0x2C) >> 30 != 0,
          True)
    qt.writel(CPM + 0x10, 0)
    check('PLL off: ON clear', qt.readl(CPM + 0x10) & 8, 0)
    qt.writel(CPM + 0x10, (100 << 20) | (1 << 14) | (2 << 11) | (1 << 8) | 1)
    check('PLL on: ON follows EN', qt.readl(CPM + 0x10) & 8, 8)
    qt.writel(CPM, 0x9A073310 | (7 << 20))
    check('CPCCR change bits read back clear', qt.readl(CPM) & (7 << 20), 0)
    check('CPCSR not busy', qt.readl(CPM + 0xD4) & 7, 0)
    qt.writel(CPM + 0x68, (1 << 29) | (1 << 28) | 5)
    check('MSC0CDR: BUSY clear after CE', qt.readl(CPM + 0x68) & (1 << 28), 0)
    if soc == 't31':
        qt.writel(CPM + 0x74, (1 << 28) | (1 << 27) | 13)
        check('SSICDR: BUSY is bit 27', qt.readl(CPM + 0x74) & (1 << 27), 0)
    qt.writel(CPM + 0x70, (0x1F << 20) | 3)
    check('I2S M/N divider kept whole', qt.readl(CPM + 0x70), (0x1F << 20) | 3)
    qt.writel(CPM + 0x38, 0x5A5A)
    qt.writel(CPM + 0x34, 0x1A1A)
    check('CPPSR reads back (reboot recovery)', qt.readl(CPM + 0x34), 0x1A1A)


def test_cpm_t21(qt):
    print('ingenic-cpm (t21 encoding)')
    apll = qt.readl(CPM + 0x10)
    m, n, od = apll >> 20 & 0x1ff, apll >> 14 & 0x3f, apll >> 11 & 7
    check('APLL decodes to 864 MHz', 24 * 2 * (m + 1) // (n + 1) // (1 << od),
          864)
    check('no LOCK bit on this layout', apll & 4, 0)


# --- DDR -------------------------------------------------------------------

def test_ddr(qt, soc):
    print('ingenic-ddr (%s)' % soc)
    if soc == 't20':
        check('DWC PGSR done, no errors', qt.readl(DDRPHY + 0x0C) & 0x7f, 0x1f)
        check('ZQ calibration ok', qt.readl(DDRPHY + 0x188) & (1 << 30), 0)
    elif soc == 't31':
        check('PHY PLL lock', qt.readl(DDRPHY + 0xC8) & 8, 8)
        check('APB init bits 1-2', qt.readl(DDRAPB + 0x8C) & 6, 6)
        check('init complete', qt.readl(DDRPHY + 0xD0) != 0, True)
        check('calibration == 3', qt.readl(DDRPHY + 0xCC), 3)
        qt.writel(DDRC + 0x0C, 0x400001)
        check('LMR start self-clears', qt.readl(DDRC + 0x0C) & 1, 0)
        qt.writel(DDRC + 0x240, 0x00010003)
        check('bus priority stored', qt.readl(DDRC + 0x240), 0x00010003)
    elif soc == 't40':
        check('PHY PLL lock', qt.readl(DDRPHY + 0x108) & 8, 8)
        check('calibration nibble', qt.readl(DDRPHY + 0x10C) & 0xf, 0xf)
        check('DFI init complete', qt.readl(DDRAPB + 0x04) & 1, 1)
    elif soc == 't41':
        check('PHY PLL lock', qt.readl(DDRPHY + 0x180) & 4, 4)
        check('calibration done == 3', qt.readl(DDRPHY + 0x184) & 0xf, 3)
        qt.writel(DDRC + 0x18, 0x400003)
        check('LMR start self-clears', qt.readl(DDRC + 0x18) & 1, 0)


# --- timers ----------------------------------------------------------------

def test_tcu(qt):
    print('ingenic-tcu')
    qt.writel(TCU + 0xEC, 0x800C)               # free-running, EXTAL/4
    qt.writel(TCU + 0xE4, 0)
    qt.writel(TCU + 0x14, 1 << 15)              # TESR: OST on
    a = qt.readl(TCU + 0xE4)
    qt.step(1000000)
    b = qt.readl(TCU + 0xE4)
    check('OST at 6 MHz', 5900 <= b - a <= 6100, True)
    check('high half latched in HBUF', qt.readl(TCU + 0xFC), 0)
    qt.writel(TCU + 0x3C, 1 << 16)              # TSCR: watchdog clock on
    qt.writel(TCU + 0x0C, 4 | (0 << 3))         # EXTAL/1
    qt.writel(TCU + 0x00, 0xFFFF)
    qt.writel(TCU + 0x08, 0)
    qt.writel(TCU + 0x04, 1)
    qt.step(1000)
    check('watchdog counts', 10 <= qt.readl(TCU + 0x08) <= 40, True)
    qt.writel(TCU + 0x04, 0)


def test_tcu_xburst2(qt):
    """T40/T41: the watchdog ignores TSR bit 16 and TCSR bit 10 zeroes the
    counter -- what the 4.4 kernels' restart and ping rely on."""
    print('ingenic-tcu (XBurst2 watchdog)')
    qt.writel(TCU + 0x2C, 1 << 16)              # TSSR: the driver's stop
    qt.writel(TCU + 0x0C, 4)                    # EXTAL/1
    qt.writel(TCU + 0x00, 0xFFFF)
    qt.writel(TCU + 0x04, 1)
    qt.step(1000)
    n = qt.readl(TCU + 0x08)
    check('watchdog counts with TSR bit 16 set', 10 <= n <= 40, True)
    qt.writel(TCU + 0x0C, 4 | (1 << 10))        # counter clear
    check('TCSR bit 10 zeroes the counter', qt.readl(TCU + 0x08) < 5, True)
    check('and reads back clear', qt.readl(TCU + 0x0C), 4)
    qt.writel(TCU + 0x04, 0)


def test_ost_v1(qt):
    print('ingenic-ost (system OST)')
    qt.writel(OST + 0x00, 0xA)                  # both /16: 1.5 MHz
    qt.writel(OST + 0x10, 1)
    qt.writel(OST + 0x38, 1)
    qt.writel(OST + 0x08, 1)
    qt.writel(OST + 0x0C, 0)
    qt.writel(OST + 0x14, 14999)                # 10 ms
    qt.writel(OST + 0x34, 1)
    qt.writel(OST + 0x10, 0xFFFFFFFE)
    qt.step(9000000)
    check('no match at 9 ms', qt.readl(OST + 0x0C) & 1, 0)
    qt.step(2000000)
    check('match flag at 11 ms', qt.readl(OST + 0x0C) & 1, 1)
    check('counter wrapped', qt.readl(OST + 0x18) < 2000, True)
    qt.writel(OST + 0x0C, 0xFFFFFFFE)
    check('flag cleared by ~1', qt.readl(OST + 0x0C) & 1, 0)
    qt.writel(OST + 0x34, 2)
    a = qt.readl(OST + 0x20)
    qt.step(1000000)
    b = qt.readl(OST + 0x20)
    check('clock source at 1.5 MHz', 1490 <= b - a <= 1510, True)


def test_ost_xburst2(qt):
    print('ingenic-ost (XBurst2 global and core)')
    qt.writel(OST + 0x04, 0)
    qt.writel(OST + 0x00, 0)
    qt.writel(OST + 0x08, 1)
    qt.writel(OST + 0x04, 1)
    a = qt.readl(OST + 0x10)
    qt.step(1000000)
    b = qt.readl(OST + 0x10)
    check('global OST at 24 MHz', 23900 <= b - a <= 24100, True)
    check('CNTB latched', qt.readl(OST + 0x14), 0)
    qt.writel(CORE_OST + 0x04, 0)
    qt.writel(CORE_OST + 0x00, 0)
    qt.writel(CORE_OST + 0x10, 0)
    qt.writel(CORE_OST + 0x14, 24000)           # 1 ms
    qt.writel(CORE_OST + 0x08, 1)
    qt.writel(CORE_OST + 0x04, 1)
    qt.step(1100000)
    check('core OST flag', qt.readl(CORE_OST + 0x0C) & 1, 1)
    check('CCU sees it in OIPR', qt.readl(CCU + 0x180) & 1, 1)
    check('core 0 OST enable out of reset', qt.readl(CCU + 0x1A0) & 1, 1)
    check('core 1 held in reset', qt.readl(CCU + 0x40) & 2, 2)
    qt.writel(CORE_OST + 0x0C, 0)
    check('flag cleared', qt.readl(CCU + 0x180) & 1, 0)
    # the T41 kernel's set_next_event / shutdown stop the counter with the
    # flag up, and its handler clears the flag afterwards: the line must
    # drop with the counter off (left high, core 1 storms on "ost couldn't
    # find ostfr")
    qt.step(1100000)
    check('flag again', qt.readl(CCU + 0x180) & 1, 1)
    qt.writel(CORE_OST + 0x04, 0)
    qt.writel(CORE_OST + 0x0C, 0)
    check('flag cleared with the counter stopped', qt.readl(CCU + 0x180) & 1, 0)


# --- interrupt controllers -------------------------------------------------

def test_intc(qt):
    print('ingenic-intc')
    check('everything masked out of reset', qt.readl(INTC + 0x04), 0xFFFFFFFF)
    qt.writel(INTC + 0x0C, 1 << 7)
    check('IMCR unmasks', qt.readl(INTC + 0x04), 0xFFFFFF7F)
    qt.writel(INTC + 0x08, 1 << 7)
    check('IMSR masks', qt.readl(INTC + 0x04), 0xFFFFFFFF)
    check('bank 1 at +0x20', qt.readl(INTC + 0x24), 0xFFFFFFFF)


def test_core_intc(qt):
    print('ingenic-ccu core INTC')
    check('core 0 masked out of reset', qt.readl(CORE_INTC + 0x04),
          0xFFFFFFFF)
    qt.writel(CORE_INTC + 0x0C, 1)
    check('core 0 unmask', qt.readl(CORE_INTC + 0x04), 0xFFFFFFFE)
    check('core 1 bank separate', qt.readl(CORE_INTC + 0x104), 0xFFFFFFFF)
    qt.writel(CCU + 0x1000 + 4, 1)
    check('mailbox 1 pending in MIPR', qt.readl(CCU + 0x140) & 2, 2)
    qt.writel(CCU + 0x1000 + 4, 0)


# --- SADC ------------------------------------------------------------------

def test_sadc(qt, intc=INTC):
    print('ingenic-sadc')
    rb = lambda a: int(qt.cmd('readb 0x%x' % a).split()[1], 16)
    rw = lambda a: int(qt.cmd('readw 0x%x' % a).split()[1], 16)
    check('powered down out of reset', rw(SADC + 0x00), 0x8000)
    # jz_adc_probe: power cycle, mask everything, dividers
    qt.cmd('writew 0x%x 0x8000' % SADC)
    qt.cmd('writew 0x%x 0xffff' % (SADC + 0x08))
    qt.cmd('writew 0x%x 0x0000' % SADC)
    qt.writel(SADC + 0x20, 29 | (1 << 8) | (199 << 16))
    check('CLKDIV is storage', qt.readl(SADC + 0x20), 29 | (1 << 8) | (199 << 16))
    # a read of AUX1: unmask, enable, wait for the interrupt
    qt.cmd('writeb 0x%x 0x%x' % (SADC + 0x08, 0xff & ~2))
    qt.cmd('writew 0x%x 0x2' % SADC)
    check('no result before the conversion ends', rb(SADC + 0x0C), 0)
    qt.step(100000)
    check('AUX1 converted', rb(SADC + 0x0C), 2)
    check('the enable clears', rw(SADC + 0x00), 0)
    check('the SADC line pending in the INTC',
          (qt.readl(intc + 0x00) >> 18) & 1, 1)
    check('AUX1 as a halfword at 0x12', rw(SADC + 0x12), 0x810)
    check('AUX1 as the top of the word at 0x10',
          (qt.readl(SADC + 0x10) >> 16) & 0xfff, 0x810)
    qt.cmd('writeb 0x%x 0x2' % (SADC + 0x0C))
    check('STATUS write-1-clear drops the line',
          (rb(SADC + 0x0C), (qt.readl(intc + 0x00) >> 18) & 1), (0, 0))
    # 4.4's channels 4..7
    qt.cmd('writew 0x%x 0x80' % SADC)
    qt.step(100000)
    check('AUX7 at 0x1E', rw(SADC + 0x1E), 0x870)
    check('a masked channel holds the line low',
          (rb(SADC + 0x0C), (qt.readl(intc + 0x00) >> 18) & 1), (0x80, 0))
    qt.cmd('writew 0x%x 0x8001' % SADC)
    qt.step(100000)
    check('no conversion while powered down', rb(SADC + 0x0C), 0x80)
    # a level set on the pin at run time
    qt.qom_set('/machine/sadc', 'aux0', 0x123)
    qt.cmd('writew 0x%x 0x1' % SADC)
    qt.step(100000)
    check('AUX0 reads the level qom-set on its pin', rw(SADC + 0x10), 0x123)


def test_pwm(qt):
    print('ingenic-pwm (T41)')
    PWM = 0x13460000
    chan = lambda n: qt.qom_get('/machine/pwm', 'chan%d' % n)
    check('idle pin at LVL 0', chan(2), '0')
    # ingenic_pwm_request, __pwm_config, ingenic_pwm_enable for channel 2:
    # 1 kHz at 50 MHz, 25 % duty
    qt.writel(PWM + 0x00, qt.readl(PWM + 0x00) & ~(0xf << 8))
    qt.writel(PWM + 0xB0 + 8, (12500 << 16) | 37500)
    qt.writel(PWM + 0x20, 1 << 2)
    qt.writel(PWM + 0x10, 1 << 2)
    qt.writel(PWM + 0x300, qt.readl(PWM + 0x300) | (1 << 2))
    check('EN after ENS', qt.readl(PWM + 0x18), 1 << 2)
    check('BUSYR clear', qt.readl(PWM + 0x24), 0)
    check('the pin carries 25 % of 50000 cycles', chan(2), '12500/50000')
    # a new duty while running: the driver spins on BUSYR, then updates
    qt.writel(PWM + 0xB0 + 8, (50000 << 16) | 0)
    qt.writel(PWM + 0x30, 1 << 2)
    qt.writel(PWM + 0x20, 1 << 2)
    check('a duty of 100 % is a high pin', chan(2), '1')
    qt.writel(PWM + 0x14, 1 << 2)
    check('ENC clears EN', qt.readl(PWM + 0x18), 0)
    check('a disabled pin sits at LVL', chan(2), '1')
    check('other channels untouched', chan(0), '0')


# --- sound: PDMA, AIC, codec ----------------------------------------------

def pdma_ring(qt, ch, desc, buf, period, nper, drt, dcm):
    """jz_dma.c's cyclic transfer: a ring of 8-word linked descriptors"""
    for i in range(nper):
        nxt = desc + 32 * ((i + 1) % nper)
        dsa, dta = (AIC + 0x34, buf + i * period) if drt in (5, 7) else \
            (buf + i * period, AIC + 0x34)
        qt.write(desc + 32 * i, struct.pack('<8I', dcm, dsa, dta,
                 (((nxt >> 4) & 0xff) << 24) | (period // 2), 0, drt, 0, 0))
    c = PDMA + 0x20 * ch
    qt.writel(c + 0x10, 1 << 30)                # DES8
    qt.writel(c + 0x18, desc)                   # DDA
    qt.writel(PDMA + 0x100C, 1 << ch)           # DDRS
    qt.writel(c + 0x10, (1 << 30) | 1)          # CTE


def test_sound(qt, intc=INTC):
    print('ingenic-pdma, ingenic-aic')
    qt.writel(PDMA + 0x1000, 1 | (0x3f << 16))  # DMAC: DMAE (jzdma_probe)
    # memcpy, no descriptor, 4-byte units, interrupt at the end
    src, dst = SCRATCH, SCRATCH + 0x1000
    data = bytes(range(256)) * 4
    qt.write(src, data)
    c = PDMA + 0x20 * 2
    qt.writel(c + 0x00, src)
    qt.writel(c + 0x04, dst)
    qt.writel(c + 0x08, len(data) // 4)
    qt.writel(c + 0x0C, 8)                      # AUTO
    qt.writel(c + 0x14, (1 << 23) | (1 << 22) | (0 << 8) | 2)
    qt.writel(c + 0x10, (1 << 31) | 1)          # NDES, CTE
    check('memcpy copied', qt.read(dst, len(data)), data)
    check('DTC counted down, TT', (qt.readl(c + 0x08), qt.readl(c + 0x10) & 8),
          (0, 8))
    check('DIRQP bit 2', qt.readl(PDMA + 0x1004), 4)
    check('the PDMA line pending', (qt.readl(intc) >> 10) & 1, 1)
    qt.writel(PDMA + 0x1004, 0)
    qt.writel(c + 0x10, 0)
    check('DIRQP cleared by writing 0', (qt.readl(PDMA + 0x1004),
          (qt.readl(intc) >> 10) & 1), (0, 0))
    # the codec at 16 kHz, its registers bytes read back
    qt.writel(CODEC + 0x110, 0x5)
    check('codec CSRR reads back', qt.readl(CODEC + 0x110), 5)
    # record: 16-bit, right channel only, DMA, as i2s_dma_enable does
    ring, desc, period = SCRATCH + 0x2000, SCRATCH + 0x3000, 64
    qt.write(ring, bytes(4 * period))
    dcm16 = (2 << 14) | (2 << 12) | (2 << 8) | 3   # 16-bit ports and units
    pdma_ring(qt, 4, desc, ring, period, 4, 7, dcm16 | (1 << 22))
    qt.writel(AIC + 0x00, 1)                                    # ENB
    qt.writel(AIC + 0x04, (1 << 16) | (1 << 15) | (1 << 12) | 1)
    qt.step(10 * 1000 * 1000)
    dta = qt.readl(PDMA + 0x20 * 4 + 0x04)
    got = struct.unpack('<%dh' % (2 * period), qt.read(ring, 4 * period))
    want = [round(8192 * math.sin(2 * math.pi * 1000 * k / 16000))
            for k in range(160)]
    check('10 ms at 16 kHz recorded: 160 samples round the 128 of the ring',
          dta - ring, 2 * (160 - 128))
    check('the samples are the 1 kHz tone, the ring overwritten in turn',
          tuple(got[32:48]) + tuple(got[:8]),
          tuple(want[32:48]) + tuple(want[128:136]))
    check('a linked ring raises nothing', qt.readl(PDMA + 0x1004), 0)
    check('RX FIFO drained', (qt.readl(AIC + 0x14) >> 24) & 0x3f, 0)
    qt.writel(PDMA + 0x20 * 4 + 0x10, 0)
    # play: mono to stereo, 16-bit, DMA from a ring
    play = SCRATCH + 0x4000
    qt.write(play, struct.pack('<128h', *range(128)))
    pdma_ring(qt, 5, desc + 0x200, play, period, 4, 6, dcm16 | (1 << 23))
    qt.writel(AIC + 0x04, (1 << 19) | (1 << 14) | (1 << 11) | 2)
    check('the TX FIFO filled at once', (qt.readl(AIC + 0x14) >> 8) & 0x3f,
          63)
    dsa0 = qt.readl(PDMA + 0x20 * 5)
    qt.step(10 * 1000 * 1000)
    dsa1 = qt.readl(PDMA + 0x20 * 5)
    check('10 ms played: 160 samples taken', (dsa1 - dsa0) % 256,
          (160 * 2) % 256)
    check('no underrun', qt.readl(AIC + 0x14) & (1 << 5), 0)
    qt.writel(AIC + 0x04, 0)
    qt.writel(PDMA + 0x20 * 5 + 0x10, 0)
    qt.writel(AIC + 0x00, 1 | (1 << 3))
    check('RST empties the FIFOs and self-clears',
          (qt.readl(AIC + 0x00) & 8, (qt.readl(AIC + 0x14) >> 8) & 0x3f),
          (0, 0))


# --- SFC + NOR -------------------------------------------------------------

def sfc_v1(qt, cmd, addr=0, alen=0, length=0, data=True, write=False,
           dma=None, dummy=0):
    qt.writel(SFC + 0x64, 0x6)                  # STOP | FLUSH
    glb = (31 << 7) | (1 << 3) | (int(write) << 13)
    if dma is not None:
        glb |= 1 << 6
        qt.writel(SFC + 0x60, dma)
    qt.writel(SFC + 0x00, glb)
    qt.writel(SFC + 0x2C, length)
    qt.writel(SFC + 0x14, (alen << 26) | (1 << 24) | (int(data) << 16) |
              (dummy << 17) | cmd)
    qt.writel(SFC + 0x30, addr)
    qt.writel(SFC + 0x48, 0)
    qt.writel(SFC + 0x6C, 0x1F)
    qt.writel(SFC + 0x64, 1)                    # START


def test_sfc_v1(qt, img):
    print('ingenic-sfc v1 + NOR')
    sfc_v1(qt, 0x9F, length=3)
    check('RREQ for the ID', qt.readl(SFC + 0x68) & 4, 4)
    check('ID word', qt.readl(SFC + 0x1000) & 0xFFFFFF, 0x1820C2)
    check('END after the drain', qt.readl(SFC + 0x68) & 0x10, 0x10)
    sfc_v1(qt, 0x03, addr=0x100000, alen=3, length=1024)
    check('1 KiB read: END while the FIFO holds the rest',
          qt.readl(SFC + 0x68) & 0x14, 0x04)
    words = [qt.readl(SFC + 0x1000) for _ in range(256)]
    got = b''.join(struct.pack('<I', w) for w in words)
    check('PIO read data', got, bytes(img[0x100000:0x100400]))
    check('END once drained', qt.readl(SFC + 0x68) & 0x10, 0x10)
    sfc_v1(qt, 0x03, addr=0x100000, alen=3, length=4096, dma=SCRATCH)
    check('DMA read: END at once', qt.readl(SFC + 0x68) & 0x10, 0x10)
    check('DMA read data', qt.read(SCRATCH, 4096),
          bytes(img[0x100000:0x101000]))
    # erase 4 KiB at 1 MiB + 64 KiB, program 8 bytes, read back
    sfc_v1(qt, 0x06, data=False)
    sfc_v1(qt, 0x20, addr=0x110000, alen=3, data=False)
    sfc_v1(qt, 0x06, data=False)
    qt.write(SCRATCH, b'OpenIPC!')
    sfc_v1(qt, 0x02, addr=0x110000, alen=3, length=8, write=True,
           dma=SCRATCH)
    sfc_v1(qt, 0x05, length=1, dma=SCRATCH + 0x100)
    check('status: not busy', qt.read(SCRATCH + 0x100, 1)[0] & 1, 0)
    sfc_v1(qt, 0x03, addr=0x110000, alen=3, length=16, dma=SCRATCH + 0x200)
    check('programmed after erase', qt.read(SCRATCH + 0x200, 16),
          b'OpenIPC!' + b'\xff' * 8)


def test_sfc_v2(qt, img, path):
    print('ingenic-sfc v2 command table + descriptor DMA')
    desc = SCRATCH + 0x8000
    # entry 5: READ, ROW_ADDR, data
    qt.writel(SFC + 0x800 + 5 * 16, 1)                          # ROW_ADDR
    qt.writel(SFC + 0x800 + 5 * 16 + 4,
              (3 << 26) | (1 << 24) | (1 << 16) | 0x03)
    qt.write(desc, struct.pack('<4I', 0, SCRATCH, 512, 0))
    qt.writel(SFC + 0x00, (1 << 15) | (1 << 14) | (1 << 6) | (31 << 7))
    qt.writel(SFC + 0x2C, 512)
    qt.writel(SFC + 0x7C, (1 << 31) | 5)
    qt.writel(SFC + 0x84, 0x100200)
    qt.writel(SFC + 0x90, 0x20000000 | desc)    # virt_to_phys(KSEG1 buf)
    qt.writel(SFC + 0x6C, 0x1F)
    qt.writel(SFC + 0x70, 0)
    qt.writel(SFC + 0x64, 1)
    check('END', qt.readl(SFC + 0x68) & 0x10, 0x10)
    check('data through the descriptor', qt.read(SCRATCH, 512),
          bytes(img[0x100200:0x100400]))


# --- SFC + SPI NAND --------------------------------------------------------

NAND_PAGE = 2048


def make_nand_image(path, oob):
    """A 1 Gbit SPI NAND image, data only (128 MiB, w25n01gv) or with a
    128-byte OOB per page (136 MiB, gd5f1gq4ub): the NAND SPL header
    (byte 8 = 00, PPB/32 and BPP/1024 in bytes 10-11) with 'j .' at
    0x800, a pattern in page 100 and its OOB."""
    stride = NAND_PAGE + oob
    pages = 65536
    img = bytearray(b'\xFF' * (pages * stride))
    spl = bytearray(0x1000)
    spl[0:9] = bytes([0x06, 0x05, 0x04, 0x03, 0x02, 0x55, 0xaa, 0x55, 0x00])
    spl[10], spl[11] = 64 // 32, (NAND_PAGE + (oob or 64)) // 1024
    struct.pack_into('<I', spl, 0x800, 0x08000600)          # j 0x80001800
    struct.pack_into('<I', spl, 12, len(spl))
    spl[9] = crc7(spl[0x800:])
    for i in range(0, len(spl), NAND_PAGE):         # the data of pages 0, 1
        at = i // NAND_PAGE * stride
        img[at:at + NAND_PAGE] = spl[i:i + NAND_PAGE]
    page = bytes((i * 13 + 5) & 0xFF for i in range(NAND_PAGE))
    img[100 * stride:100 * stride + NAND_PAGE] = page
    if oob:
        img[100 * stride + NAND_PAGE:101 * stride] = bytes(range(oob))
    open(path, 'wb').write(img)
    return bytes(spl), page


def sfc_phases(qt, phases, length=0, write=False, dma=None,
               exp=0, msk=0):
    """A v1 multi-phase transfer: phases are (cmd, addr, alen, dummy,
    data, poll) tuples; the data stage is the one with data set."""
    qt.writel(SFC + 0x64, 0x6)
    glb = (31 << 7) | (len(phases) << 3) | (int(write) << 13)
    if dma is not None:
        glb |= 1 << 6
        qt.writel(SFC + 0x60, dma)
    qt.writel(SFC + 0x00, glb)
    qt.writel(SFC + 0x2C, length)
    qt.writel(SFC + 0x08, exp)
    qt.writel(SFC + 0x10, msk)
    for n, (cmd, addr, alen, dummy, data, poll) in enumerate(phases):
        qt.writel(SFC + 0x14 + 4 * n, (alen << 26) | (int(poll) << 25) |
                  (1 << 24) | (dummy << 17) | (int(data) << 16) | cmd)
        qt.writel(SFC + 0x30 + 4 * n, addr)
        qt.writel(SFC + 0x48 + 4 * n, 0)
    qt.writel(SFC + 0x6C, 0x1F)
    qt.writel(SFC + 0x64, 1)


def nand_feature(qt, addr, val=None):
    if val is None:
        sfc_v1(qt, 0x0F, addr=addr, alen=1, length=1)
        return qt.readl(SFC + 0x1000) & 0xFF
    sfc_v1(qt, 0x1F, addr=addr, alen=1, length=1, write=True)
    qt.writel(SFC + 0x1000, val)
    return None


POLL_OIP = (0x0F, 0xC0, 1, 0, False, True)


def nand_read(qt, row, col, length, dma):
    """Page read + status poll on OIP, then a fast read from the cache."""
    sfc_phases(qt, [(0x13, row, 3, 0, False, False), POLL_OIP],
               exp=0, msk=1)
    sfc_v1(qt, 0x0B, addr=col, alen=2, length=length, dma=dma, dummy=8)
    return qt.read(dma, length)


def test_nand_boot_rom(qt, spl):
    print('mask ROM stand-in, SPI NAND')
    check('NAND SPL header in the cache-as-RAM window',
          qt.read(CAR + 0x1000, 12), spl[0:12])
    check('SPL code at 0x80001800', qt.readl(CAR + 0x1800), 0x08000600)


def test_sfc_nand_v1(qt, page, path):
    print('ingenic-sfc v1 + SPI NAND (w25n01gv, data-only image)')
    sfc_v1(qt, 0x9F, length=2)
    check('READ ID, no address byte: the dummy slot, then the maker',
          qt.readl(SFC + 0x1000) & 0xFFFF, 0xEF00)
    sfc_v1(qt, 0x9F, alen=1, length=3)
    check('READ ID after the dummy byte', qt.readl(SFC + 0x1000) & 0xFFFFFF,
          0x21AAEF)
    check('protection at power-up: locked', nand_feature(qt, 0xA0), 0x7C)
    check('configuration: ECC-E and BUF', nand_feature(qt, 0xB0), 0x18)
    nand_feature(qt, 0xB0, 0x19)
    check('set feature', nand_feature(qt, 0xB0), 0x19)
    nand_feature(qt, 0xB0, 0x18)
    # page read with OIP showing for three status reads (busy-polls=3)
    sfc_phases(qt, [(0x13, 100, 3, 0, False, False), POLL_OIP],
               exp=0, msk=1)
    check('page read + poll phase: END', qt.readl(SFC + 0x68) & 0x10, 0x10)
    check('DEV_STA_RT: the polled status, OIP clear',
          qt.readl(SFC + 0x0C) & 0x01, 0)
    sfc_v1(qt, 0x0B, addr=0, alen=2, length=NAND_PAGE, dma=SCRATCH, dummy=8)
    check('read from cache (0x0B, 8 dummy clocks)',
          qt.read(SCRATCH, NAND_PAGE), page)
    sfc_v1(qt, 0x03, addr=0x800, alen=2, length=64, dma=SCRATCH, dummy=8)
    check('OOB: all FF (a good block)', qt.read(SCRATCH, 64), b'\xff' * 64)
    sfc_v1(qt, 0x6B, addr=0x1000 | 16, alen=2, length=16, dma=SCRATCH,
           dummy=8)
    check('quad read, plane bit set: ignored by a one-plane part',
          qt.read(SCRATCH, 16), page[16:32])
    sfc_v1(qt, 0x0B, addr=4, alen=2, length=8, dummy=8)
    got = struct.pack('<2I', qt.readl(SFC + 0x1000), qt.readl(SFC + 0x1000))
    check('PIO read from cache', got, page[4:12])

    # locked: program and erase fail
    sfc_phases(qt, [(0x06, 0, 0, 0, False, False),
                    (0x10, 300, 3, 0, False, False), POLL_OIP], msk=1)
    check('program execute while locked: P_FAIL',
          qt.readl(SFC + 0x0C) & 0x08, 0x08)
    sfc_phases(qt, [(0x06, 0, 0, 0, False, False),
                    (0xD8, 300, 3, 0, False, False), POLL_OIP], msk=1)
    check('erase while locked: E_FAIL', qt.readl(SFC + 0x0C) & 0x04, 0x04)
    nand_feature(qt, 0xA0, 0x00)

    # write enable, program load and execute in one transfer (the 3.10
    # kernel's message), the data by PIO, then the status poll
    sfc_phases(qt, [(0x06, 0, 0, 0, False, False),
                    (0x02, 0, 2, 0, True, False),
                    (0x10, 300, 3, 0, False, False)], length=8, write=True)
    check('TREQ for the program data', qt.readl(SFC + 0x68) & 0x08, 0x08)
    qt.writel(SFC + 0x1000, struct.unpack('<I', b'Open')[0])
    qt.writel(SFC + 0x1000, struct.unpack('<I', b'IPC!')[0])
    check('END after the data and the execute phase',
          qt.readl(SFC + 0x68) & 0x10, 0x10)
    sfc_phases(qt, [POLL_OIP], msk=1)
    check('program: WEL and P_FAIL clear', qt.readl(SFC + 0x0C) & 0x0A, 0)
    check('programmed page reads back', nand_read(qt, 300, 0, 16, SCRATCH),
          b'OpenIPC!' + b'\xff' * 8)
    with open(path, 'rb') as f:
        f.seek(300 * NAND_PAGE)
        check('program written through to the image', f.read(8),
              b'OpenIPC!')
    # random load into page 300's cache contents: keeps the rest
    nand_read(qt, 300, 0, 4, SCRATCH)
    qt.write(SCRATCH + 0x100, b'\x00' * 4)
    sfc_phases(qt, [(0x06, 0, 0, 0, False, False),
                    (0x84, 8, 2, 0, True, False),
                    (0x10, 301, 3, 0, False, False)], length=4, write=True,
               dma=SCRATCH + 0x100)
    check('random load (DMA) + execute to the next page',
          nand_read(qt, 301, 0, 16, SCRATCH),
          b'OpenIPC!' + b'\x00' * 4 + b'\xff' * 4)
    # erase the block (pages 256..319)
    sfc_phases(qt, [(0x06, 0, 0, 0, False, False),
                    (0xD8, 300, 3, 0, False, False), POLL_OIP], msk=1)
    check('erase: E_FAIL clear', qt.readl(SFC + 0x0C) & 0x04, 0)
    check('erased page reads FF', nand_read(qt, 300, 0, 16, SCRATCH),
          b'\xff' * 16)
    with open(path, 'rb') as f:
        f.seek(256 * NAND_PAGE)
        check('erase written through to the image',
              f.read(64 * NAND_PAGE) == b'\xff' * (64 * NAND_PAGE), True)
    sfc_v1(qt, 0x10, addr=302, alen=3, data=False)
    check('program execute without WEL: ignored',
          nand_read(qt, 302, 0, 4, SCRATCH), b'\xff' * 4)


def test_sfc_nand_v2(qt, page):
    print('ingenic-sfc v2 command table + SPI NAND (gd5f1gq4ub, with OOB)')
    desc = SCRATCH + 0x8000
    cdt = SFC + 0x800
    # 10: read to cache (ROW, link); 11: get feature, poll OIP (STA0,
    # link); 12: read from cache (COL, data, 8 dummy clocks)
    for n, (link, xfer, exp, msk) in enumerate([
            ((1 << 31) | 1, (3 << 26) | (1 << 24) | 0x13, 0, 0),
            ((1 << 31) | 2, (1 << 26) | (1 << 25) | (1 << 24) | 0x0F, 0, 1),
            (0, (2 << 26) | (1 << 24) | (8 << 17) | (1 << 16) | 0x0B, 0, 0)]):
        qt.write(cdt + (10 + n) * 16, struct.pack('<4I', link, xfer, exp, msk))
    sfc_v1(qt, 0x9F, alen=1, length=2)
    check('READ ID', qt.readl(SFC + 0x1000) & 0xFFFF, 0xD1C8)
    qt.write(desc, struct.pack('<4I', 0, SCRATCH, NAND_PAGE + 128, 0))
    qt.writel(SFC + 0x00, (1 << 15) | (1 << 14) | (1 << 6) | (31 << 7))
    qt.writel(SFC + 0x2C, NAND_PAGE + 128)
    qt.writel(SFC + 0x7C, (1 << 31) | 10)
    qt.writel(SFC + 0x80, 1 << 12)              # CONVERT_COL_ADDR's plane bit
    qt.writel(SFC + 0x84, 100)
    qt.writel(SFC + 0x88, 0xC0)
    qt.writel(SFC + 0x90, 0x20000000 | desc)
    qt.writel(SFC + 0x6C, 0x1F)
    qt.writel(SFC + 0x70, 0)
    qt.writel(SFC + 0x64, 1)
    check('END', qt.readl(SFC + 0x68) & 0x10, 0x10)
    check('page data through the descriptor', qt.read(SCRATCH, NAND_PAGE),
          page)
    check('OOB from the image', qt.read(SCRATCH + NAND_PAGE, 128),
          bytes(range(128)))


# --- small blocks ----------------------------------------------------------

def test_gpio(qt):
    print('ingenic-gpio')
    pb = GPIO + 0x1000
    check('inputs read high', qt.readl(pb + 0x00), 0xFFFFFFFF)
    # PB3 -> output 0 through the shadow port (gpio_set_func)
    z = GPIO + 0x7000
    qt.writel(z + 0x18, 1 << 3)                 # INTC
    qt.writel(z + 0x24, 1 << 3)                 # MSKS
    qt.writel(z + 0x38, 1 << 3)                 # PAT1C
    qt.writel(z + 0x48, 1 << 3)                 # PAT0C
    qt.writel(z + 0xF0, 1)                      # GID2LD port B
    check('shadow load made PB3 an output', qt.readl(pb + 0x30) & 8, 0)
    check('PB3 reads 0', qt.readl(pb + 0x00) & 8, 0)
    qt.writel(pb + 0x44, 1 << 3)
    check('PAT0S drives it high', qt.readl(pb + 0x00) & 8, 8)


def test_i2c(qt):
    print('ingenic-i2c')
    qt.writel(I2C0 + 0x6C, 1)
    check('ENSTA follows ENB', qt.readl(I2C0 + 0x9C) & 1, 1)
    qt.writel(I2C0 + 0x04, 0x30)
    qt.writel(I2C0 + 0x10, 0x0A | (1 << 9))
    check('absent target: TXABRT 7-bit address NACK',
          qt.readl(I2C0 + 0x80), 1)
    check('TXABT raw', qt.readl(I2C0 + 0x34) & (1 << 6), 1 << 6)
    qt.readl(I2C0 + 0x54)
    check('cleared by CTXABRT', qt.readl(I2C0 + 0x80), 0)
    qt.writel(I2C0 + 0x6C, 0)
    check('ENSTA drops', qt.readl(I2C0 + 0x9C) & 1, 0)


def i2c_read_reg(qt, base, addr, reg, n=1):
    qt.writel(base + 0x6C, 0)
    qt.writel(base + 0x04, addr)
    qt.writel(base + 0x6C, 1)
    qt.writel(base + 0x10, reg)
    for i in range(n):
        qt.writel(base + 0x10, (1 << 8) | (1 << 10 if i == 0 else 0) |
                  (1 << 9 if i == n - 1 else 0))
    return [qt.readl(base + 0x10) & 0xFF for i in range(qt.readl(base + 0x78))]


def test_isp(qt):
    print('ingenic-isp (t31, sensor=gc2053)')
    check('GC2053 ID at I2C0 0x37', i2c_read_reg(qt, I2C0, 0x37, 0xF0, 2),
          [0x20, 0x53])
    qt.writel(CPM + 0xC4, 1 << 21)
    check('SRBC: ISP stop request acknowledged',
          qt.readl(CPM + 0xC4) & (3 << 20), 3 << 20)
    qt.writel(CPM + 0xC4, 0)
    for cmd in (2, 4):
        qt.writel(VIC + 0x00, cmd)
        check('VIC control %d self-clears' % cmd, qt.readl(VIC + 0x00), 0)
    qt.writel(VIC + 0x04, 1920 << 16 | 1080)
    check('VIC storage', qt.readl(VIC + 0x04), 1920 << 16 | 1080)
    check('CSI ERR1 reads 0', qt.readl(0x10023020), 0)

    w, h = 64, 32
    y0, uv0 = 0x00400000, 0x00400000 + w * h
    y1, uv1 = 0x00410000, 0x00410000 + w * h
    ch0 = ISP + 0x9800
    qt.writel(ch0 + 0x100, w << 16 | h)
    qt.writel(ch0 + 0x12C, w << 16 | h)
    qt.writel(ch0 + 0x180, w)
    qt.writel(ch0 + 0x198, w)
    for y, uv in ((y0, uv0), (y1, uv1)):
        qt.writel(ch0 + 0x16C, y)
        qt.writel(ch0 + 0x184, uv)
    check('done FIFO empty before streaming', qt.readl(ch0 + 0x17C) & 1, 1)
    qt.writel(ISP + 0x9804, 0xF0001)
    qt.writel(VIC + 0x00, 1)
    qt.step(100 * 1000 * 1000)
    check('no frames until the ISP interrupts are enabled',
          qt.readl(ISP + 0xB4), 0)
    qt.writel(ISP + 0xB0, 0xFFFFFFFF)
    qt.step(41 * 1000 * 1000)
    check('ISP status: frame start + channel 0 done',
          qt.readl(ISP + 0xB4) & 0x1001, 0x1001)
    check('VIC status: frame done', qt.readl(VIC + 0x1E0) & 1, 1)
    check('INTC sources 29 (ISP) and 30 (VIC) pending',
          (qt.readl(INTC + 0x00) >> 29) & 3, 3)
    check('done FIFO not empty', qt.readl(ch0 + 0x17C) & 1, 0)
    check('pop: the first queued Y address', qt.readl(ch0 + 0x174), y0)
    check('popped entry: its UV address', qt.readl(ch0 + 0x18C), uv0)
    check('done FIFO empty again', qt.readl(ch0 + 0x17C) & 1, 1)
    f0 = qt.read(y0, w * h)
    c0 = qt.read(uv0, w * h // 2)
    check('Y: navy background in the band under the logo',
          36 <= f0[22 * w + 1] <= 50, True)
    check('UV: navy chroma (Cb 146, Cr 122)', (c0[11 * w], c0[11 * w + 1]),
          (146, 122))
    check('the logo is drawn', max(f0[10 * w:16 * w]) > 200, True)
    qt.writel(ISP + 0xB8, qt.readl(ISP + 0xB4))
    qt.writel(VIC + 0x1F0, 1)
    check('ISP status cleared by 0xB8', qt.readl(ISP + 0xB4), 0)
    check('VIC status cleared by 0x1F0', qt.readl(VIC + 0x1E0), 0)
    check('INTC sources 29 and 30 drop', (qt.readl(INTC + 0x00) >> 29) & 3,
          0)
    qt.step(40 * 1000 * 1000)
    check('second frame into the second buffer', qt.readl(ch0 + 0x174), y1)
    f1 = qt.read(y1, w * h)
    check('the frame counter advanced', f1[:7 * w] != f0[:7 * w], True)
    qt.writel(ISP + 0xB8, qt.readl(ISP + 0xB4))
    qt.step(40 * 1000 * 1000)
    check('no buffer queued: the frame is dropped',
          (qt.readl(ISP + 0xB4) & 0x1001, qt.readl(ch0 + 0x17C) & 1),
          (0x1000, 1))
    qt.writel(ch0 + 0x16C, y0)
    qt.writel(ch0 + 0x184, uv0)
    qt.writel(ch0 + 0x19C, 1)
    qt.writel(ISP + 0xB8, qt.readl(ISP + 0xB4))
    qt.step(40 * 1000 * 1000)
    check('FIFO clear drops the queued buffer',
          (qt.readl(ISP + 0xB4) & 1, qt.readl(ch0 + 0x17C) & 1), (0, 1))
    qt.writel(ISP + 0xB0, 0)
    qt.writel(ISP + 0xB8, 0xFFFFFFFF)
    qt.step(80 * 1000 * 1000)
    check('stream off: the frame clock stops', qt.readl(ISP + 0xB4), 0)


def i2c_write_reg(qt, base, addr, reg, val):
    qt.writel(base + 0x6C, 0)
    qt.writel(base + 0x04, addr)
    qt.writel(base + 0x6C, 1)
    qt.writel(base + 0x10, reg)
    qt.writel(base + 0x10, val | (1 << 9))


def isp_bits(data, bit, n):
    return (int.from_bytes(data, 'little') >> bit) & ((1 << n) - 1)


def test_isp_stats(qt):
    """The Tiziano AE / AWB statistics as tisp_init sets them up (T31)."""
    print('ingenic-isp 3A statistics (t31, sensor=gc2053)')
    AE, AWB = ISP + 0xA000, ISP + 0xB000
    zb, hb, ab = 0x00600000, 0x00604000, 0x00610000

    def expose(lines):
        for reg, val in ((0xFE, 0), (0x03, lines >> 8), (0x04, lines & 0xFF),
                         (0xB8, 1), (0xB9, 0)):
            i2c_write_reg(qt, I2C0, 0x37, reg, val)

    def frame():
        qt.writel(ISP + 0xB8, qt.readl(ISP + 0xB4))
        qt.step(40 * 1000 * 1000)
        idx = qt.readl(AE + 0x50)
        return (qt.read(zb + 0x1000 * ((idx >> 4) & 3), 225 * 16),
                qt.read(hb + 0x800 * (idx & 3), 0x800),
                qt.read(ab + 0x1000 * qt.readl(AWB + 0x50), 225 * 16))

    def zone_mean(zones, n):
        z = zones[16 * n:16 * n + 16]
        return sum(isp_bits(z, b, 21) for b in (0, 21, 42)) / (64 * 36)

    expose(1000)
    for blk in (AE, AWB):
        qt.writel(blk + 0x04, 0xF001F001)
        for i in range(4):
            qt.writel(blk + 0x08 + 4 * i, 0x40404040)
            qt.writel(blk + 0x18 + 4 * i, 0x24242424)
    qt.writel(AE + 0x28, 0x0011991E)
    for i in range(4):
        qt.writel(AE + 0x2C + 4 * i, zb + 0x1000 * i)
        qt.writel(AE + 0x3C + 4 * i, hb + 0x800 * i)
        qt.writel(AWB + 0x3C + 4 * i, ab + 0x1000 * i)
    qt.writel(AE + 0x4C, 0x33)
    qt.writel(AWB + 0x4C, 3)
    qt.writel(AE, 1)
    qt.writel(AWB, 1)
    qt.writel(VIC + 0x00, 1)
    qt.writel(ISP + 0xB0, 0xFFFFFFFF)
    zones, hist, awb = frame()
    check('AE0 static, AE0 histogram and AWB bits in 0xB4',
          qt.readl(ISP + 0xB4) & (0x4C << 24), 0x4C << 24)
    i0 = qt.readl(AE + 0x50)
    m0 = zone_mean(zones, 0)
    check('top left zone: the navy, in linear light (~4 of 255)',
          3 < m0 < 6, True)
    centre = zones[16 * (7 * 15 + 7):16 * (7 * 15 + 8)]
    check('the logo zone counts bright quads', isp_bits(centre, 76, 13) > 0,
          True)
    check('histogram: every quad of the 960 x 540 counted once',
          abs(sum(isp_bits(hist[4 * i:4 * i + 4], 0, 21)
                  for i in range(256)) - 960 * 540) < 960 * 540 // 100, True)
    check('histogram: second window the same', hist[:0x400], hist[0x400:])
    z0 = awb[:16]
    check('AWB zone: all 2304 quads summed', isp_bits(z0, 84, 13), 64 * 36)
    check('AWB zone: navy under daylight raw, B > R',
          isp_bits(z0, 42, 21) > isp_bits(z0, 0, 21), True)
    zones, hist, awb = frame()
    check('the banks rotate', qt.readl(AE + 0x50) != i0, True)
    expose(2000)
    zones, hist, awb = frame()
    check('twice the exposure: twice the linear light',
          abs(zone_mean(zones, 0) / m0 - 2) < 0.15, True)
    qt.qom_set('/machine/isp', 'light', 50)
    zones, hist, awb = frame()
    check('half the light: back where it was',
          abs(zone_mean(zones, 0) - m0) < 0.5, True)
    qt.qom_set('/machine/isp', 'light-r', 1100)
    zones, hist, awb = frame()
    z1 = awb[:16]
    check('a redder light: more R in the AWB zone',
          isp_bits(z1, 0, 21) > isp_bits(z0, 0, 21), True)


ISP_LAYOUTS = {
    # en, sta, clr, ch0, ch_en, push_y, push_uv, pop_y, empty, size, vic,
    # vic status, vic clear
    't21': (0x80, 0x84, 0x88, 0x2400, 0x2304, 0x6C, 0x84, 0x74, 0x7C, 0x2C,
            0x13310000, 0x100, 0x108),
    't23': (0xB0, 0xB4, 0xB8, 0xD100, 0xD040, 0x30, 0x50, 0x38, 0x3C, 0x0C,
            VIC, 0x1E0, 0x1F0),
    't40': (0x40020, 0x40028, 0x40024, 0x16100, 0x16008, 0x6C, 0x84, 0x74,
            0x7C, 0x2C, 0x13380000, 0x1E0, 0x1F0),
    't41': (0x40020, 0x40028, 0x40024, 0xF0100, 0xF0008, 0x6C, 0x84, 0x74,
            0x7C, 0x2C, 0x13380000, 0x1E0, 0x1F0),
}


def test_isp_layout(qt, soc):
    print('ingenic-isp (%s layout)' % soc)
    (en, sta, clr, ch0, ch_en, push_y, push_uv, pop_y, empty, size, vic,
     vsta, vclr) = ISP_LAYOUTS[soc]
    w, h = 32, 16
    ch0 += ISP
    qt.writel(ch0 + size, w << 16 | h)
    qt.writel(ch0 + push_y, 0x00400000)
    qt.writel(ch0 + push_uv, 0x00400000 + w * h)
    qt.writel(ISP + ch_en, 1)
    qt.writel(vic + 0x00, 2)
    check('VIC control reads 0 after 2', qt.readl(vic + 0x00), 0)
    qt.writel(vic + 0x00, 1)
    qt.writel(ISP + en, 0xFFFFFFFF)
    qt.step(41 * 1000 * 1000)
    check('channel 0 done', qt.readl(ISP + sta) & 1, 1)
    check('VIC frame done', qt.readl(vic + vsta) & 1, 1)
    if soc in ('t40', 't41'):
        check('VIC frame start (bit 4)', qt.readl(vic + vsta) & 0x10, 0x10)
    check('pop: the queued buffer', (qt.readl(ch0 + empty) & 1,
                                     qt.readl(ch0 + pop_y),
                                     qt.readl(ch0 + empty) & 1),
          (0, 0x00400000, 1))
    check('painted', 36 <= qt.read(0x00400000 + 12 * w, 1)[0] <= 50, True)
    qt.writel(ISP + clr, qt.readl(ISP + sta))
    qt.writel(vic + vclr, qt.readl(vic + vsta))
    check('status cleared', (qt.readl(ISP + sta), qt.readl(vic + vsta)),
          (0, 0))
    if soc == 't41':
        # s_stream / suspend: set 0x200 bit 0, spin on 0x204 bit 0
        qt.writel(ISP + 0x200, 1)
        check('0x204 bit 0 follows 0x200 bit 0', qt.readl(ISP + 0x204) & 1, 1)
        qt.writel(ISP + 0x200, 0)
        check('and drops with it', qt.readl(ISP + 0x204) & 1, 0)


def test_isp_apical(qt, soc='t20'):
    print('ingenic-isp-apical (%s)' % soc)
    core, vic, irqb = 0x13380000, 0x13300000, 0x13320000
    vctl = vic + (0x20 if soc == 't10' else 0)
    if soc == 't20':
        check('JX-F22 ID at I2C0 0x40',
              i2c_read_reg(qt, I2C0, 0x40, 0x0A, 2), [0x0F, 0x22])
    check('CSI PHY_STATE: lanes in stop state', qt.readl(0x10040014), 0x630)
    qt.writel(core + 0x130, 1)
    check('0x134 follows the run state', qt.readl(core + 0x134) & 7, 1)
    qt.writel(core + 0x130, 0)
    check('... and its stop', qt.readl(core + 0x134) & 7, 0)
    # slots: 0 = frame end (23), 5 = channel 0 done (14), 7 = frame start (24)
    qt.writel(core + 0x80, 23)
    qt.writel(core + 0x88, 14 << 16)
    qt.writel(core + 0x8C, 24 << 16)
    w, h, wr = 32, 16, core + 0xB00
    qt.writel(wr + 0x04, w | h << 16)
    qt.writel(wr + 0x08, 0x00400000)
    qt.writel(wr + 0x88, 0x00400000 + w * h)
    qt.writel(wr + 0x1C, 0)
    qt.writel(wr + 0x20, w)
    qt.writel(wr + 0xA0, w)
    qt.writel(wr + 0x80, 0x200 | 0x4D)
    qt.writel(wr + 0x00, 0x200 | 0xD)
    qt.writel(irqb + 0x14, 0x1FFFF)
    qt.writel(vctl, 2)
    check('VIC reset reads back 0', qt.readl(vctl), 0)
    qt.writel(vctl, 1)
    qt.step(41 * 1000 * 1000)
    check('frame start: slot 7', qt.readl(core + 0xA0), 1 << 7)
    check('INTC source 29 pending', (qt.readl(INTC + 0x00) >> 29) & 1, 1)
    qt.step(20 * 1000 * 1000)
    check('frame end: slots 0, 5, 7', qt.readl(core + 0xA0),
          1 << 7 | 1 << 5 | 1)
    check('interrupt block: the slots and VIC frame done',
          qt.readl(irqb + 0x0C), 1 << 16 | 1 << 7 | 1 << 5 | 1)
    check('writer status: bank 0, not busy', qt.readl(wr + 0x24) & 0x10700, 0)
    check('painted', 36 <= qt.read(0x00400000 + 12 * w, 1)[0] <= 50, True)
    check('NV12 chroma', tuple(qt.read(0x00400000 + w * h + 5 * w, 2)),
          (146, 122))
    qt.writel(core + 0xA4, 0)
    qt.writel(core + 0xA4, qt.readl(core + 0xA0))
    qt.writel(irqb + 0x04, qt.readl(irqb + 0x0C))
    check('cleared through 0xA4 and the block\'s ack',
          (qt.readl(core + 0xA0), qt.readl(irqb + 0x0C)), (0, 0))
    check('INTC source 29 drops', (qt.readl(INTC + 0x00) >> 29) & 1, 0)
    qt.writel(wr + 0x24, 3)
    qt.step(40 * 1000 * 1000)
    check('control 3: the next frame is dropped (no done slot)',
          qt.readl(core + 0xA0) & (1 << 5), 0)


def test_ipu(qt, soc):
    print('ingenic-ipu (%s)' % soc)
    ipu, bg, osd = 0x13080000, 0x00400000, 0x00410000
    w, h = 8, 4
    qt.write(bg, bytes([40]) * (w * h) + bytes([128]) * (w * h // 2))
    # 2x2 ARGB1555: opaque white, transparent, opaque black, opaque white
    qt.write(osd, bytes([0xFF, 0xFF, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF]))
    qt.writel(ipu + 0x74, 1 << 3)                       # reset
    qt.writel(ipu + 0x18, w << 16 | h)
    qt.writel(ipu + 0x0C, bg)
    qt.writel(ipu + 0x10, bg + w * h)
    qt.writel(ipu + 0x1C, w)
    qt.writel(ipu + 0x20, w << 16 | w)
    qt.writel(ipu + 0x24, bg)
    qt.writel(ipu + 0xB4, bg + w * h)
    qt.writel(ipu + 0x2C, w)
    qt.writel(ipu + 0xB8, w)
    qt.writel(ipu + 0x12C, 2 << 11 | 1 << 18 | 1)      # NV12 background
    qt.writel(ipu + 0xBC, osd)
    qt.writel(ipu + 0xC0, 4)
    qt.writel(ipu + 0xFC, 4 << 16 | 2)                 # 2 pixels = 4 bytes
    qt.writel(ipu + 0x10C, 2 << 16 | 0)                # at 2,0
    # the driver's RGB -> YUV matrix (_ipu_set_osd_chx_route)
    for i, v in enumerate((0x133 | 0x25A << 20, 0x75 | 0x97 << 20,
                           0x128 | 0x1BF << 20, 0x276 | 0x210 << 20, 0x67,
                           0x10 | 0x80 << 16)):
        qt.writel(ipu + 0x148 + 4 * i, v)
    qt.writel(ipu + 0x11C, 0xD << 14 | 4 << 11 | 1)    # 1555, pixel alpha
    qt.writel(ipu + 0x7C, 1)                           # IRQ_EN
    qt.writel(ipu + 0x74, 1)                           # run
    check('busy until the interrupt', qt.readl(ipu + 0x04) & 1, 0)
    qt.step(200 * 1000)
    check('OUT_END', qt.readl(ipu + 0x04) & 1, 1)
    if soc not in ('t40', 't41'):
        check('INTC source 6 pending', (qt.readl(INTC + 0x00) >> 6) & 1, 1)
    check('row 0: white over, transparent kept',
          tuple(qt.read(bg, w)), (40, 40, 255, 40, 40, 40, 40, 40))
    check('row 1: black (Y 16), white', tuple(qt.read(bg + w, 4)),
          (40, 40, 16, 255))
    qt.writel(ipu + 0x7C, 0)
    if soc not in ('t40', 't41'):
        check('IRQ_EN off drops it', (qt.readl(INTC + 0x00) >> 6) & 1, 0)
    qt.writel(ipu + 0x74, 1 << 3)
    check('reset clears the channel', qt.readl(ipu + 0x11C), 0)


def test_rtc_efuse(qt, rtc=RTC):
    print('ingenic-rtc, ingenic-efuse')
    check('WRDY', qt.readl(rtc + 0x00) & 0x80, 0x80)
    qt.writel(rtc + 0x3C, 0xA55A)
    check('WEN after the magic', qt.readl(rtc + 0x3C) >> 31, 1)
    s = qt.readl(rtc + 0x04)
    check('seconds since 2020', s > 1577836800, True)
    check('EFUSE read done', qt.readl(EFUSE + 0x08) & 3, 3)
    qt.writel(EFUSE + 0x00, (3 << 21) | 1)
    check('EFUSE enable self-clears', qt.readl(EFUSE + 0x00) & 1, 0)
    check('blank fuses', qt.readl(EFUSE + 0x23C), 0)


def test_ids(qemu):
    print('SoC identification (HARB0 0x2C, CPPSR, EFUSE grade words)')
    want = {'t10': 0x10005003, 't20': 0x12000002, 't21': 0x10021003,
            't23': 0x10023003, 't30': 0x10030005, 't31': 0x10031003,
            't40': 0x10040003, 't41': 0x10041003}
    for soc, sid in want.items():
        qt = QTest(qemu, soc)
        check('%s SoC ID' % soc, qt.readl(0x1300002C), sid)
        qt.close()
    for mach, off, code, cppsr in (('t31,variant=t31x', 0x238, 0x2222, 1),
                                   ('t31,variant=t31lc', 0x238, 0xeeee, 1),
                                   ('t30,variant=t30l', 0x238, 0, 0x10),
                                   ('t40,variant=t40xp', 0x250, 0x7777, 1),
                                   ('t41,variant=t41lq', 0x250, 0x9999, 1)):
        qt = QTest(qemu, mach)
        check('%s grade code' % mach, qt.readl(EFUSE + off) >> 16, code)
        check('%s CPPSR low byte' % mach, qt.readl(CPM + 0x34) & 0xff, cppsr)
        if off == 0x238:
            qt.writel(EFUSE + 0x00, ((off - 0x200) // 4) << 21 | 1)
            check('%s read through CTRL/DATA' % mach,
                  qt.readl(EFUSE + 0x0C) >> 16, code)
        qt.close()


def test_migration(qemu, machine, tmp):
    print('migration round trip (%s)' % machine)
    out = os.path.join(tmp, 'mig')
    p = subprocess.run(
        [qemu, '-M', machine, '-machine', 'accel=qtest', '-display', 'none',
         '-nodefaults', '-S',
         '-monitor', 'stdio'],
        input='migrate "exec:cat > %s"\nquit\n' % out, text=True,
        capture_output=True, timeout=60)
    ok = os.path.exists(out) and os.path.getsize(out) > 0
    check('state saved', ok, True)
    if ok:
        p = subprocess.run(
            [qemu, '-M', machine, '-machine', 'accel=qtest', '-display',
             'none', '-nodefaults', '-monitor', 'stdio', '-incoming', 'exec:cat %s' % out],
            input='info status\nquit\n', text=True, capture_output=True,
            timeout=60)
        check('state loaded', 'paused' in p.stdout or 'running' in p.stdout,
              True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(
        os.path.dirname(__file__), '..', '..', 'qemu-src', 'build',
        'qemu-system-mipsel'))
    a = ap.parse_args()
    tmp = tempfile.mkdtemp()
    try:
        img_path = os.path.join(tmp, 'flash.bin')
        img = make_image(img_path)

        for soc in ('t10', 't20', 't21', 't23', 't30', 't31', 't40', 't41'):
            qt = QTest(a.qemu, soc)
            check('%s realizes' % soc, qt.readl(CPM + 0x10) & 1, 1)
            qt.close()

        shutil.copy(img_path, img_path + '.t31')
        qt = QTest(a.qemu, 't31,flash-file=%s.t31' % img_path)
        test_boot_rom(qt, img)
        test_cpm(qt, 't31')
        test_ddr(qt, 't31')
        test_tcu(qt)
        test_ost_v1(qt)
        test_intc(qt)
        test_sfc_v1(qt, img)
        test_gpio(qt)
        test_i2c(qt)
        test_rtc_efuse(qt)
        test_sadc(qt)
        test_sound(qt)
        qt.close()
        qt = QTest(a.qemu, 't41')
        test_sadc(qt, CORE_INTC)
        test_sound(qt, CORE_INTC)
        test_pwm(qt)
        qt.close()

        qt = QTest(a.qemu, 't31,sensor=gc2053')
        test_isp(qt)
        qt.close()
        qt = QTest(a.qemu, 't31,sensor=gc2053')
        test_isp_stats(qt)
        qt.close()
        for soc in ISP_LAYOUTS:
            qt = QTest(a.qemu, soc)
            test_isp_layout(qt, soc)
            qt.close()
        qt = QTest(a.qemu, 't20,sensor=jxf22')
        test_isp_apical(qt)
        qt.close()
        qt = QTest(a.qemu, 't10,sensor=jxh62')
        test_isp_apical(qt, 't10')
        qt.close()
        for soc in ('t30', 't31', 't41'):
            qt = QTest(a.qemu, soc)
            test_ipu(qt, soc)
            qt.close()

        for soc in ('t20', 't40', 't41'):
            qt = QTest(a.qemu, soc)
            test_ddr(qt, soc)
            qt.close()

        qt = QTest(a.qemu, 't21')
        test_cpm_t21(qt)
        qt.close()

        shutil.copy(img_path, img_path + '.t41')
        qt = QTest(a.qemu, 't41,flash-file=%s.t41' % img_path)
        test_ost_xburst2(qt)
        test_tcu_xburst2(qt)
        test_core_intc(qt)
        test_sfc_v2(qt, img, img_path)
        qt.close()

        nand = os.path.join(tmp, 'nand.bin')
        spl, page = make_nand_image(nand, 0)
        qt = QTest(a.qemu, 't31,flash-file=%s' % nand,
                   ['-global', 'w25n01gv.busy-polls=3'])
        test_nand_boot_rom(qt, spl)
        test_sfc_nand_v1(qt, page, nand)
        qt.close()
        os.unlink(nand)
        spl, page = make_nand_image(nand, 128)
        qt = QTest(a.qemu, 't41,flash-file=%s' % nand,
                   ['-global', 'gd5f1gq4ub.busy-polls=2'])
        test_nand_boot_rom(qt, spl)
        test_sfc_nand_v2(qt, page)
        qt.close()
        qt = QTest(a.qemu, 't41', ['-drive', 'if=mtd,index=1,format=raw,'
                                   'file=%s' % nand])
        print("T41's second SFC (0x13450000)")
        qt.writel(SFC1 + 0x00, (31 << 7) | (1 << 3))
        qt.writel(SFC1 + 0x2C, 2)
        qt.writel(SFC1 + 0x14, (1 << 26) | (1 << 24) | (1 << 16) | 0x9F)
        qt.writel(SFC1 + 0x30, 0)
        qt.writel(SFC1 + 0x64, 1)
        check('READ ID of the NAND on SFC1',
              qt.readl(SFC1 + 0x1000) & 0xFFFF, 0xD1C8)
        qt.close()
        os.unlink(nand)

        test_ids(a.qemu)
        test_migration(a.qemu, 't31', tmp)
        test_migration(a.qemu, 't31,sensor=gc2053', tmp)
        test_migration(a.qemu, 't41', tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('%d failure(s)' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
