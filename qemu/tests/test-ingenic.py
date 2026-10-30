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
import hashlib
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
I2D = 0x130B0000
DBOX = 0x130D0000
PDMA = 0x13420000
AIC = 0x10020000
DMIC = 0x10034000
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


def test_drawbox(qt, intc=CORE_INTC):
    print('ingenic-drawbox (T40/T41)')
    W, H = 64, 32
    y_base, uv_base = SCRATCH, SCRATCH + W * H
    qt.write(y_base, bytes([0x80]) * (W * H))
    qt.write(uv_base, bytes([0x80]) * (W * H // 2))
    # dbox_start: reset, the picture, the driver's palette, two boxes
    qt.writel(DBOX + 0x00, 1 << 1)
    check('RESET clears itself', qt.readl(DBOX + 0x00) & 2, 0)
    qt.writel(DBOX + 0x10, W | (H << 16))
    qt.writel(DBOX + 0x0C, W | (W << 16))
    qt.writel(DBOX + 0x14, 0xe195004c)
    qt.writel(DBOX + 0x18, 0x002b8054)
    qt.writel(DBOX + 0x1C, 0x941580ff)
    qt.writel(DBOX + 0x04, y_base)
    qt.writel(DBOX + 0x08, uv_base)
    def box(x, y, w, h, col, mode, lw, ll=0):
        qt.writel(DBOX + 0x20, x | (y << 12) | ((w & 0xff) << 24))
        qt.writel(DBOX + 0x20, (w >> 8) | (h << 4) | (col << 16) |
                  (mode << 18) | (lw << 20) | (ll << 23))
    box(8, 4, 24, 16, 0, 0, 2)          # a red rectangle, 2 pixels wide
    box(40, 20, 16, 0, 3, 2, 1)         # a yellow horizontal line
    check('two boxes in the RAM (write pointer)',
          (qt.readl(DBOX + 0x00) >> 9) & 0xff, 2)
    qt.writel(DBOX + 0x00, qt.readl(DBOX + 0x00) & ~(1 << 3))
    qt.writel(DBOX + 0x00, qt.readl(DBOX + 0x00) | 1)
    qt.step(200000)
    check('done bit, START cleared', qt.readl(DBOX + 0x00) & 0x11, 0x10)
    check('the DRAWBOX line pending', (qt.readl(intc + 0x20) >> 10) & 1, 1)
    y = qt.read(y_base, W * H)
    uv = qt.read(uv_base, W * H // 2)
    check('rectangle edge: red luma', (y[4 * W + 8], y[5 * W + 20]),
          (0x4c, 0x4c))
    check('rectangle edge: red chroma', (uv[2 * W + 8], uv[2 * W + 9]),
          (0x54, 0xff))
    check('inside the rectangle untouched', y[10 * W + 16], 0x80)
    check('right and bottom edges', (y[10 * W + 31], y[19 * W + 16]),
          (0x4c, 0x4c))
    check('outside untouched', (y[10 * W + 33], y[21 * W + 16]),
          (0x80, 0x80))
    check('yellow line, 1 pixel high', (y[20 * W + 40], y[20 * W + 55],
                                        y[21 * W + 40]), (0xe1, 0xe1, 0x80))
    # the handler clears the done bit by writing CTRL back without it
    qt.writel(DBOX + 0x00, qt.readl(DBOX + 0x00) & ~(1 << 4))
    check('done cleared, line low', (qt.readl(DBOX + 0x00) & 0x10,
          (qt.readl(intc + 0x20) >> 10) & 1), (0, 0))


def test_i2d(qt, intc=CORE_INTC):
    print('ingenic-i2d (T40/T41)')
    W, H = 8, 4
    src, dst = SCRATCH, SCRATCH + 0x10000
    pic = bytes(range(W * H))
    uv = bytes(range(0x80, 0x80 + W * H // 2))
    qt.write(src, pic)
    qt.write(src + W * H, uv)
    def run(mode, dst_stride):
        qt.writel(I2D + 0x10, 1 << 4)                    # safe reset
        qt.writel(I2D + 0x18, mode)
        qt.writel(I2D + 0x14, (W << 16) | H)
        qt.writel(I2D + 0x20, src)
        qt.writel(I2D + 0x24, src + W * H)
        qt.writel(I2D + 0x30, dst)
        qt.writel(I2D + 0x34, dst + dst_stride * (W if mode & 4 else H))
        qt.writel(I2D + 0x28, W)
        qt.writel(I2D + 0x2C, W)
        qt.writel(I2D + 0x38, dst_stride)
        qt.writel(I2D + 0x3C, dst_stride)
        qt.writel(I2D + 0x08, 0xffffffff)
        qt.writel(I2D + 0x10, 1)
        qt.step(200000)
    run(1 << 2, H)                                       # NV12, 90
    check('frame done', qt.readl(I2D + 0x80) & 1, 1)
    check('the I2D line pending', (qt.readl(intc + 0x20) >> 13) & 1, 1)
    out = qt.read(dst, W * H)
    # 90 clockwise: destination (x, y) = source (row H-1-x, column y)
    want = bytes(pic[(H - 1 - x) * W + y] for y in range(W) for x in range(H))
    check('Y rotated 90 degrees clockwise', out, want)
    ouv = qt.read(dst + H * W, W * H // 2)
    # CbCr: 4x2 pairs -> 2x4 pairs
    pairs = [uv[i:i + 2] for i in range(0, len(uv), 2)]
    wuv = b''.join(pairs[(H // 2 - 1 - x) * (W // 2) + y]
                   for y in range(W // 2) for x in range(H // 2))
    check('CbCr pairs rotated with it', ouv, wuv)
    qt.writel(I2D + 0x84, 1)
    check('IRQ_CLEAR drops the line', (qt.readl(I2D + 0x80) & 1,
          (qt.readl(intc + 0x20) >> 13) & 1), (0, 0))
    run(1 << 1, W)                                       # mirror
    out = qt.read(dst, W * H)
    check('mirror', out, bytes(pic[r * W + W - 1 - c]
                               for r in range(H) for c in range(W)))
    run(1, W)                                            # flip
    out = qt.read(dst, W * H)
    check('flip', out, bytes(pic[(H - 1 - r) * W + c]
                             for r in range(H) for c in range(W)))


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


# --- TCU channels as PWM ----------------------------------------------------

def test_tcu_pwm(qt):
    print('ingenic-tcu PWM pins')
    pwm = lambda n: qt.qom_get('/machine/tcu', 'pwm%d' % n)
    ch = TCU + 0x40 + 0x10 * 2
    check('a pin the TCU does not drive', pwm(2), 'off')
    # jz_tcu_config_chn: initial level high, EXTAL, PWM output on; period
    # 1000, duty 250 (sample_pwm's polarity normal)
    qt.writel(ch + 0xC, (1 << 8) | 4 | (1 << 7))
    qt.writel(ch + 0x0, 1000)
    qt.writel(ch + 0x4, 250)
    check('enabled, not counting: the initial level', pwm(2), '1')
    qt.writel(TCU + 0x3C, 1 << 2)           # TSCR: clock on
    qt.writel(TCU + 0x14, 1 << 2)           # TESR: count
    check('counting: high for TDHR of TDFR', pwm(2), '250/1000')
    qt.writel(ch + 0xC, 4 | (1 << 7))
    check('initial level low: high for the rest', pwm(2), '750/1000')
    qt.writel(ch + 0x4, 0)
    check('half at 0: flipped all the time', pwm(2), '1')
    qt.writel(ch + 0x4, 1000)
    check('half at full: the initial level all the time', pwm(2), '0')
    qt.writel(TCU + 0x18, 1 << 2)           # TECR
    qt.writel(ch + 0xC, 4)
    check('PWM output off', pwm(2), 'off')


# --- security blocks: DTRNG, AES, HASH, RSA ---------------------------------

DTRNG = 0x10072000
AES = 0x13430000
HASH = 0x13480000
RSA = 0x134C0000


def test_dtrng(qt, intc=INTC):
    print('ingenic-dtrng')
    check('nothing ready while off', qt.readl(DTRNG + 0x08), 0)
    # sample_dtrng's CPU mode: divider 8, interrupt masked, enable
    qt.writel(DTRNG, (8 << 1) | (1 << 11) | 1)
    check('not ready at once', qt.readl(DTRNG + 0x08), 0)
    qt.step(100000)
    check('ready after a while', qt.readl(DTRNG + 0x08), 1)
    a, b = qt.readl(DTRNG + 0x04), qt.readl(DTRNG + 0x04)
    check('two reads, two numbers', a != b, True)
    check('masked: no interrupt', (qt.readl(intc + 0x20) >> 2) & 1, 0)
    qt.writel(DTRNG, (8 << 1) | 1)
    check('unmasked: source 34 raised', (qt.readl(intc + 0x20) >> 2) & 1, 1)
    # the interrupt handler: clear ready, disable, mask, release the clear
    qt.writel(DTRNG, (8 << 1) | 1 | (1 << 12))
    qt.writel(DTRNG, (8 << 1) | (1 << 12))
    qt.writel(DTRNG, (8 << 1) | (1 << 11) | (1 << 12))
    qt.writel(DTRNG, (8 << 1) | (1 << 11))
    check('cleared: not ready, line low',
          (qt.readl(DTRNG + 0x08), (qt.readl(intc + 0x20) >> 2) & 1), (0, 0))


def be_words(data):
    """the AES DMA's memory format: each word's top byte comes first"""
    return b''.join(data[i:i + 4][::-1] for i in range(0, len(data), 4))


def aes_dma(qt, key, data, mode, iv=None, dec=False):
    """jz-aes.c's sequence, in DMA mode"""
    src, dst = SCRATCH, SCRATCH + 0x1000
    qt.writel(AES + 0x04, qt.readl(AES + 0x04))
    qt.writel(AES + 0x08, 0x5)
    qt.writel(AES + 0x00, 3 << 28 | 1 << 10 | 1)
    ascr = 3 << 28 | 1 << 8 | 1 | (dec << 4) | ((mode == 'cbc') << 5)
    qt.writel(AES + 0x00, ascr)
    if mode == 'cbc':
        for i in range(4):
            qt.writel(AES + 0x24, struct.unpack('>I', iv[4 * i:4 * i + 4])[0])
        qt.writel(AES + 0x00, qt.readl(AES + 0x00) | 1 << 1)
    qt.writel(AES + 0x14, len(data) // 16)
    for i in range(4):
        qt.writel(AES + 0x20, struct.unpack('>I', key[4 * i:4 * i + 4])[0])
    qt.writel(AES + 0x00, qt.readl(AES + 0x00) | 1 << 2)
    keyd = qt.readl(AES + 0x04) & 1
    qt.write(src, be_words(data))
    qt.writel(AES + 0x0C, src)
    qt.writel(AES + 0x10, dst)
    qt.writel(AES + 0x00, qt.readl(AES + 0x00) | 1 << 9)
    return keyd, qt.readl(AES + 0x04), be_words(qt.read(dst, len(data)))


def test_aes(qt, intc=INTC):
    print('ingenic-aes')
    # FIPS-197 C.1 and SP 800-38A F.2.1
    k1 = bytes(range(16))
    p1 = bytes.fromhex('00112233445566778899aabbccddeeff')
    c1 = bytes.fromhex('69c4e0d86a7b0430d8cdb78070b4c55a')
    keyd, st, out = aes_dma(qt, k1, p1, 'ecb')
    check('key done', keyd, 1)
    check('ECB encrypt: FIPS-197', out, c1)
    check('DMA done and the INTC line', (st & 4, (qt.readl(intc) >> 23) & 1),
          (4, 1))
    qt.writel(AES + 0x04, 7)
    check('ASSR write 1 clears the line', (qt.readl(intc) >> 23) & 1, 0)
    _, _, out = aes_dma(qt, k1, c1, 'ecb', dec=True)
    check('ECB decrypt', out, p1)
    k2 = bytes.fromhex('2b7e151628aed2a6abf7158809cf4f3c')
    iv = bytes(range(16))
    p2 = bytes.fromhex('6bc1bee22e409f96e93d7e117393172a'
                       'ae2d8a571e03ac9c9eb76fac45af8e51')
    c2 = bytes.fromhex('7649abac8119b246cee98e9b12e9197d'
                       '5086cb9b507219ee95db113a917678b2')
    _, _, out = aes_dma(qt, k2, p2, 'cbc', iv)
    check('CBC encrypt, two blocks: SP 800-38A', out, c2)
    _, _, out = aes_dma(qt, k2, c2, 'cbc', iv, dec=True)
    check('CBC decrypt', out, p2)
    # one block through ASDI / ASDO
    qt.writel(AES + 0x00, 3 << 28 | 1 << 10 | 1)
    qt.writel(AES + 0x00, 3 << 28 | 1)
    for i in range(4):
        qt.writel(AES + 0x20, struct.unpack('>I', k1[4 * i:4 * i + 4])[0])
    qt.writel(AES + 0x00, qt.readl(AES + 0x00) | 1 << 2)
    for i in range(4):
        qt.writel(AES + 0x18, struct.unpack('>I', p1[4 * i:4 * i + 4])[0])
    qt.writel(AES + 0x00, qt.readl(AES + 0x00) | 1 << 3)
    out = b''.join(struct.pack('>I', qt.readl(AES + 0x1C)) for i in range(4))
    check('one block by the CPU', (qt.readl(AES + 0x04) & 2, out), (2, c1))


def md_pad(msg, alg):
    """the padding sample_hash's test does in user space"""
    bs, lb = (128, 16) if alg in ('sha384', 'sha512') else (64, 8)
    n = len(msg) * 8
    p = msg + b'\x80'
    p += b'\0' * ((-len(p) - lb) % bs)
    return p + (n.to_bytes(lb, 'little' if alg == 'md5' else 'big'))


def test_hash(qt, intc=INTC):
    print('ingenic-hash')
    algs = ('md5', 'sha1', 'sha224', 'sha256', 'sha384', 'sha512')
    msg = b'The quick brown fox jumps over the lazy dog, ' * 3
    for code, alg in enumerate(algs):
        data = md_pad(msg, alg)
        qt.write(SCRATCH, data)
        # jz-hash.c: defaults, the algorithm, enable, byte order, initial
        # state; then each run: clear, unmask, DMA enable, length, source,
        # start
        qt.writel(HASH + 0x00, 0x30000000)
        qt.writel(HASH + 0x04, 3)
        qt.writel(HASH + 0x1C, 1)
        qt.writel(HASH + 0x00, qt.readl(HASH + 0x00) | code << 1)
        qt.writel(HASH + 0x00, qt.readl(HASH + 0x00) | 1)
        qt.writel(HASH + 0x00, qt.readl(HASH + 0x00) |
                  (1 << 8 if alg == 'md5' else 1 << 7))
        qt.writel(HASH + 0x00, qt.readl(HASH + 0x00) | 1 << 4)
        # in two runs, to carry the state over
        half = (len(data) // 2) & ~127 or len(data)
        for off, n in ((0, half), (half, len(data) - half)):
            if not n:
                continue
            qt.writel(HASH + 0x04, 3)
            qt.writel(HASH + 0x08, 3)
            qt.writel(HASH + 0x00, qt.readl(HASH + 0x00) | 1 << 5)
            qt.writel(HASH + 0x10, n // 64)
            qt.writel(HASH + 0x0C, SCRATCH + off)
            qt.writel(HASH + 0x00, qt.readl(HASH + 0x00) | 1 << 6)
        want = hashlib.new(alg, msg).digest()
        got = b''.join(struct.pack('<I', qt.readl(HASH + 0x18))
                       for i in range(len(want) // 4))
        check('%s of %d bytes' % (alg, len(msg)), got, want)
    check('all rounds done and the INTC line',
          (qt.readl(HASH + 0x04) & 3, (qt.readl(intc) >> 22) & 1), (3, 1))
    qt.writel(HASH + 0x04, 3)
    check('HSSR write 1 clears the line', (qt.readl(intc) >> 22) & 1, 0)


def test_rsa(qt):
    print('ingenic-rsa')
    import random
    rng = random.Random(31)
    for bits in (1024, 2048):
        k = bits // 32
        n = rng.getrandbits(bits) | 1 << (bits - 1) | 1
        e = rng.getrandbits(bits)
        m = rng.getrandbits(bits) % n
        words = lambda v: [(v >> (32 * (k - 1 - i))) & 0xffffffff
                           for i in range(k)]
        # jz-rsa.c: clear, mode, E, N, M (most significant word first),
        # pre-process, main run, the result least significant word first
        qt.writel(RSA, qt.readl(RSA) & ~1)
        qt.writel(RSA, qt.readl(RSA) | 1)
        qt.writel(RSA, (qt.readl(RSA) | 1 << 7) if bits == 2048 else
                  (qt.readl(RSA) & ~(1 << 7)))
        for reg, v in ((0x04, e), (0x08, n), (0x0C, m)):
            for w in words(v):
                qt.writel(RSA + reg, w)
        qt.writel(RSA, qt.readl(RSA) | 1 << 1)
        pre = qt.readl(RSA) & 1 << 2
        qt.writel(RSA, qt.readl(RSA) | 1 << 4)
        main = qt.readl(RSA) & 1 << 5
        r = 0
        for i in range(k):
            r |= qt.readl(RSA + 0x10) << (32 * i)
        check('RSA-%d: both stages done' % bits, (pre, main), (4, 32))
        check('RSA-%d: M^E mod N' % bits, r == pow(m, e, n), True)
        qt.writel(RSA, 1 << 2 | 1 << 3 | 1 << 5 | 1 << 6)
        check('RSA-%d: flags clear' % bits, qt.readl(RSA) & 0x6c, 0)


def test_usbphy(qt):
    print('OTG PHY registers')
    qt.cmd('writeb 0x10060078 0x20')
    qt.cmd('writeb 0x10060070 0x00')
    check('cpm_usb.c tuning bits read back',
          int(qt.cmd('readb 0x10060078').split()[1], 16), 0x20)


# --- SSI (SPI masters) -------------------------------------------------------

SSI0 = 0x10043000
SSI1 = 0x10044000


def ssi_setup(qt, base, cr0_extra=0):
    """jz_spi.c: 8-bit frames, SPI format, triggers, enable"""
    qt.writel(base + 0x04, 0)
    qt.writel(base + 0x08, (6 << 3) | (1 << 16) | (14 << 8))
    qt.writel(base + 0x04, (1 << 15) | (1 << 7) | 0x6 | cr0_extra)


def test_ssi(qt, intc=INTC):
    print('ingenic-ssi')
    sr = lambda b: qt.readl(b + 0x0C)
    check('idle: END, RX empty', sr(SSI0) & 0x90, 0x90)
    # READ ID from the w25q80 on SSI0's CE, CE held by UNFIN
    ssi_setup(qt, SSI0)
    qt.writel(SSI0 + 0x08, qt.readl(SSI0 + 0x08) | 1 << 23)
    for b in (0x9F, 0, 0, 0):
        qt.writel(SSI0, b)
    check('4 frames received', (sr(SSI0) >> 8) & 0xff, 4)
    check('underrun with UNFIN, no END', sr(SSI0) & 0x82, 0x02)
    rx = [qt.readl(SSI0) for _ in range(4)]
    check('READ ID: QEMU\'s w25q80 (ef 50 14)', rx[1:], [0xEF, 0x50, 0x14])
    qt.writel(SSI0 + 0x08, qt.readl(SSI0 + 0x08) & ~(1 << 23))
    check('UNFIN cleared: END, CE released', sr(SSI0) & 0x80, 0x80)
    # the underrun interrupt the PIO transfers end on
    qt.writel(SSI0 + 0x08, qt.readl(SSI0 + 0x08) | 1 << 23)
    qt.writel(SSI0 + 0x04, qt.readl(SSI0 + 0x04) | 1 << 12)   # TEIE
    qt.writel(SSI0, 0x05)                                     # RDSR
    qt.writel(SSI0, 0x00)
    check('TEIE + UNDR: source 9 raised', (qt.readl(intc) >> 9) & 1, 1)
    qt.writel(SSI0 + 0x0C, sr(SSI0) & ~3)                     # clear_errors
    check('clear_errors drops it', (qt.readl(intc) >> 9) & 1, 0)
    qt.writel(SSI0 + 0x04, qt.readl(SSI0 + 0x04) | 0x2)       # RFLUSH
    check('RFLUSH empties RX', (sr(SSI0) >> 8) & 0xff, 0)
    qt.writel(SSI0 + 0x08, qt.readl(SSI0 + 0x08) & ~(1 << 23))
    qt.writel(SSI0 + 0x04, 0)
    # SSI1 in loopback, 16-bit frames, by DMA: 300 frames, past the FIFO
    n = 300
    data = struct.pack('<%dH' % n, *[(i * 257) & 0xffff for i in range(n)])
    src, dst = SCRATCH, SCRATCH + 0x1000
    qt.write(src, data)
    qt.write(dst, bytes(len(data)))
    qt.writel(SSI1 + 0x04, 0)
    qt.writel(SSI1 + 0x08, (14 << 3) | (1 << 23))
    qt.writel(SSI1 + 0x04, (1 << 15) | (1 << 10) | 0x6)       # SSIE, LOOP
    qt.writel(PDMA + 0x1000, 1)
    dcm16 = (2 << 14) | (2 << 12) | (1 << 8)                  # 16-bit, bytes
    rxc, txc = PDMA + 0x20 * 6, PDMA + 0x20 * 7
    qt.writel(rxc + 0x00, SSI1)                               # DR -> memory
    qt.writel(rxc + 0x04, dst)
    qt.writel(rxc + 0x08, len(data))
    qt.writel(rxc + 0x0C, 0x19)
    qt.writel(rxc + 0x14, dcm16 | (1 << 22) | 2)
    qt.writel(rxc + 0x10, (1 << 31) | 1)
    qt.writel(txc + 0x00, src)                                # memory -> DR
    qt.writel(txc + 0x04, SSI1)
    qt.writel(txc + 0x08, len(data))
    qt.writel(txc + 0x0C, 0x18)
    qt.writel(txc + 0x14, dcm16 | (1 << 23))
    qt.writel(txc + 0x10, (1 << 31) | 1)
    check('loopback by DMA: 300 frames back', qt.read(dst, len(data)), data)
    check('both channels done', (qt.readl(txc + 0x10) & 8,
                                 qt.readl(rxc + 0x10) & 8), (8, 8))
    qt.writel(txc + 0x10, 0)
    qt.writel(rxc + 0x10, 0)
    qt.writel(PDMA + 0x1004, 0)
    qt.writel(SSI1 + 0x04, 0)


# --- sound: PDMA, AIC, codec ----------------------------------------------

def pdma_ring(qt, ch, desc, buf, period, nper, drt, dcm, port=AIC + 0x34,
              unit=2):
    """jz_dma.c's cyclic transfer: a ring of 8-word linked descriptors"""
    for i in range(nper):
        nxt = desc + 32 * ((i + 1) % nper)
        dsa, dta = (port, buf + i * period) if drt in (5, 7, 13) else \
            (buf + i * period, port)
        qt.write(desc + 32 * i, struct.pack('<8I', dcm, dsa, dta,
                 (((nxt >> 4) & 0xff) << 24) | (period // unit), 0, drt, 0,
                 0))
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
    # the microphone path on, ALC on (T31's codec_set_buildin_mic)
    qt.writel(CODEC + 0x8C, 0x90)
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


def test_codec_gain(qt):
    """T31's codec: the record gain and the ADC switch, by PIO"""
    print('ingenic-aic codec gains (T31 layout)')

    def peak(cmiccr, calcgr):
        qt.writel(AIC + 0x04, 0)
        qt.writel(AIC + 0x00, 1 | (1 << 3))             # empty the FIFOs
        qt.writel(CODEC + 0x110, 0x5)
        qt.writel(CODEC + 0x8C, cmiccr)
        qt.writel(CODEC + 0x94, calcgr)
        qt.writel(AIC + 0x04, (1 << 16) | (1 << 12) | 1)  # EREC, mono
        qt.step(1500 * 1000)                            # 24 frames
        v = [qt.readl(AIC + 0x34) & 0xffff for _ in range(24)]
        return max(abs(x - 65536 if x >= 32768 else x) for x in v)

    check('ALC on: a quarter of full scale whatever the gain',
          peak(0x90, 0x1f), 8192)
    check('ALC off, gain 0x10: unity', peak(0x80, 0x10), 8192)
    check('ALC off, gain 0x14: +6 dB', peak(0x80, 0x14), 16345)
    check('ALC off, gain 0x0C: -6 dB', peak(0x80, 0x0C), 4106)
    check('the ADC off: silence', peak(0x10, 0x10), 0)
    qt.writel(AIC + 0x04, 0)


def test_dmic(qt):
    print('ingenic-dmic')
    qt.writel(PDMA + 0x1000, 1 | (0x3f << 16))
    check('every interrupt masked out of reset', qt.readl(DMIC + 0x08), 0x3f)
    # audio_dmic.c: 16 kHz, one microphone, packed, DMA on request 13
    ring, desc, period = SCRATCH + 0x6000, SCRATCH + 0x7000, 128
    qt.write(ring, bytes(4 * period))
    dcm32 = (0 << 14) | (0 << 12) | (0 << 8) | 3    # 32-bit ports and units
    pdma_ring(qt, 6, desc, ring, period, 4, 13, dcm32 | (1 << 22),
              port=DMIC + 0x30, unit=4)
    qt.writel(DMIC + 0x34, (1 << 31) | 32)              # RDMS, threshold
    qt.writel(DMIC + 0x00, (1 << 31) | (1 << 8) | (1 << 6) | 1)
    check('RESET self-clears', qt.readl(DMIC + 0x00) >> 30, 0)
    qt.step(10 * 1000 * 1000)
    dta = qt.readl(PDMA + 0x20 * 6 + 0x04)
    got = struct.unpack('<160h', qt.read(ring, 320))
    want = [round(8192 * math.sin(2 * math.pi * 1000 * k / 16000))
            for k in range(160)]
    check('10 ms at 16 kHz: 160 samples, two to an entry', dta - ring, 320)
    check('the samples are the 1 kHz tone, the earlier in bits 15:0',
          got[:24], tuple(want[:24]))
    check('the FIFO drained, EMPTY', qt.readl(DMIC + 0x38) & 0x1007f,
          1 << 16)
    qt.writel(DMIC + 0x34, 32)                          # RDMS off
    qt.step(10 * 1000 * 1000)
    check('without DMA the FIFO fills: FULL, level 64',
          qt.readl(DMIC + 0x38) & 0x8007f, (1 << 19) | 64)
    check('a DR read takes an entry', (qt.readl(DMIC + 0x30) != 0,
          qt.readl(DMIC + 0x38) & 0x7f), (True, 63))
    qt.writel(DMIC + 0x00, 0)
    qt.writel(PDMA + 0x20 * 6 + 0x10, 0)


def test_i2s_clock(qt):
    print('the AIC at the CPM\'s I2S clocks (T31)')
    qt.writel(PDMA + 0x1000, 1 | (0x3f << 16))
    qt.writel(CODEC + 0x110, 0x5)                       # the codec: 16 kHz
    # the speaker clock: VPLL 1200 MHz * 256 / 25000 = 12.288 MHz, 48 kHz
    qt.writel(CPM + 0x70, (2 << 30) | (1 << 29) | (256 << 20) | 25000)
    play, desc, period = SCRATCH + 0x8000, SCRATCH + 0x9000, 1024
    qt.write(play, bytes(4 * period))
    dcm16 = (2 << 14) | (2 << 12) | (2 << 8) | 3
    pdma_ring(qt, 5, desc, play, period, 4, 6, dcm16 | (1 << 23))
    qt.writel(AIC + 0x00, 1)
    qt.writel(AIC + 0x04, (1 << 19) | (1 << 14) | (1 << 11) | 2)
    dsa0 = qt.readl(PDMA + 0x20 * 5)
    qt.step(10 * 1000 * 1000)
    dsa1 = qt.readl(PDMA + 0x20 * 5)
    check('10 ms played at 48 kHz, not the codec\'s 16: 480 samples',
          (dsa1 - dsa0) // 2, 480)
    qt.writel(AIC + 0x04, 0)
    qt.writel(PDMA + 0x20 * 5 + 0x10, 0)
    # the speaker clock off: the codec's rate again
    qt.writel(CPM + 0x70, (2 << 30) | (256 << 20) | 25000)
    pdma_ring(qt, 5, desc, play, period, 4, 6, dcm16 | (1 << 23))
    qt.writel(AIC + 0x04, (1 << 19) | (1 << 14) | (1 << 11) | 2)
    dsa0 = qt.readl(PDMA + 0x20 * 5)
    qt.step(10 * 1000 * 1000)
    dsa1 = qt.readl(PDMA + 0x20 * 5)
    check('with the clock gated, the codec\'s 16 kHz: 160 samples',
          (dsa1 - dsa0) // 2, 160)
    qt.writel(AIC + 0x04, 0)
    qt.writel(PDMA + 0x20 * 5 + 0x10, 0)


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


def test_isp_stats_t21(qt):
    """T21's statistics: AE 0x700, AWB 0x800, no enable bits (tisp_init)."""
    print('ingenic-isp 3A statistics (t21, sensor=jxf37)')
    AE, AWB, vic = ISP + 0x700, ISP + 0x800, 0x13310000
    zb, hb, ab = 0x00600000, 0x00604000, 0x00610000
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
    qt.writel(vic + 0x00, 1)
    qt.writel(ISP + 0x80, 0xFFFFFFFF)
    qt.step(41 * 1000 * 1000)
    check('AE static, AE histogram and AWB: int_sta bits 16, 17, 18',
          qt.readl(ISP + 0x84) & (7 << 16), 7 << 16)
    idx = qt.readl(AE + 0x50)
    zones = qt.read(zb + 0x1000 * ((idx >> 4) & 3), 225 * 16)
    hist = qt.read(hb + 0x800 * (idx & 3), 0x400)
    awb = qt.read(ab + 0x1000 * qt.readl(AWB + 0x50), 16)
    check('histogram counts in 20 bits',
          max(struct.unpack('<256I', hist)) <= 0xFFFFF, True)
    z = zones[:16]
    m = sum(isp_bits(z, b, 21) for b in (0, 21, 42)) / (64 * 36)
    check('top left zone: the navy at a quarter scale (~1 of 255)',
          0 < m < 2.5, True)
    check('AWB zone: the count at bit 63, no IR field',
          isp_bits(awb, 63, 13), 64 * 36)


def test_isp_stats_apical(qt):
    """The Apical AE histogram and AWB zones (T20, sensor=jxf22)."""
    print('ingenic-isp-apical 3A statistics (t20)')
    core, vic, irqb = 0x13380000, 0x13300000, 0x13320000
    # slot 3 on event 11 (AE statistics), slot 4 on event 9 (AWB)
    qt.writel(core + 0x84, 11 << 16)
    qt.writel(core + 0x88, 9)
    w, h, wr = 1920, 1080, core + 0xB00
    qt.writel(wr + 0x04, w | h << 16)
    qt.writel(wr + 0x08, 0x00400000)
    qt.writel(wr + 0x88, 0x00400000 + w * h)
    qt.writel(wr + 0x20, w)
    qt.writel(wr + 0xA0, w)
    qt.writel(wr + 0x80, 0x200 | 0x4D)
    qt.writel(wr + 0x00, 0x200 | 0xD)
    qt.writel(irqb + 0x14, 0x1FFFF)
    qt.writel(vic, 1)
    qt.step(61 * 1000 * 1000)
    check('AE and AWB statistics: slots 3 and 4',
          qt.readl(core + 0xA0) & (3 << 3), 3 << 3)

    def bin_count(v):
        e, m = (v >> 12) & 0xF, v & 0xFFF
        return m if not e else (m | 0x1000) << (e - 1)
    total = sum(bin_count(qt.readl(core + 0x10000 + 4 * i))
                for i in range(256))
    check('histogram: every pixel counted once',
          abs(total - w * h) < w * h // 50, True)
    z0 = qt.readl(core + 0x8740)
    check('AWB zone 0: navy under daylight raw, B/G > R/G',
          (z0 >> 16) & 0xFFF > z0 & 0xFFF, True)
    check('AWB zone 0: its weight', qt.readl(core + 0x8744),
          (w // 15) * (h // 15) & 0xFFFF)
    qt.qom_set('/machine/isp', 'light-r', 1100)
    qt.step(40 * 1000 * 1000)
    check('a redder light: a higher R/G in zone 0',
          qt.readl(core + 0x8740) & 0xFFF > z0 & 0xFFF, True)


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


def test_isp_t40_pipe1(qt):
    """T40's second pipe: VIC1, channels 3..5, its interrupts and 3A"""
    print('ingenic-isp (T40 second pipe)')
    vic0, vic1 = 0x13380000, 0x13390000
    w, h = 216, 108                     # a counter cell of 2 pixels
    buf = [0x00400000, 0x00500000]      # pipe 0's, pipe 1's
    for p, (ch, en) in enumerate(((0x16100, 0x16008), (0x17100, 0x17008))):
        qt.writel(ISP + ch + 0x2C, w << 16 | h)
        qt.writel(ISP + ch + 0x6C, buf[p])
        qt.writel(ISP + ch + 0x84, buf[p] + w * h)
        qt.writel(ISP + en, 1)
    # pipe 1 alone: VIC1 started, its interrupts on
    qt.writel(vic1 + 0x00, 1)
    qt.writel(ISP + 0x40030, 0xFFFFFFFF)
    # its AE block: one zone of 8 x 4 quads, a zone and a histogram bank
    qt.writel(ISP + 0x19200, 1)
    qt.writel(ISP + 0x19204, 0x10001000)
    qt.writel(ISP + 0x19208, 8)
    qt.writel(ISP + 0x19218, 4)
    qt.writel(ISP + 0x1922C, 0x00600000)
    qt.writel(ISP + 0x1923C, 0x00601000)
    qt.write(0x00600000, bytes(16))
    qt.step(41 * 1000 * 1000)
    check('pipe 1: channel 3 done in 0x40038', qt.readl(ISP + 0x40038) & 1, 1)
    check('pipe 0 idle: nothing in 0x40028', qt.readl(ISP + 0x40028), 0)
    check('VIC1 frame done and start', qt.readl(vic1 + 0x1E0) & 0x11, 0x11)
    check('VIC1 interrupts on INTC 29, VIC0 quiet',
          ((qt.readl(CORE_INTC) >> 29) & 1, (qt.readl(CORE_INTC) >> 30) & 1),
          (1, 0))
    check('pipe 1 AE statistics: bits 4 and 5 of 0x40038',
          qt.readl(ISP + 0x40038) & 0x30, 0x30)
    check('its zone bank written', qt.read(0x00600000, 16) != bytes(16), True)
    check('pop channel 3: the queued buffer',
          (qt.readl(ISP + 0x17174), qt.readl(ISP + 0x1717C) & 1),
          (buf[1], 1))
    qt.writel(ISP + 0x40034, qt.readl(ISP + 0x40038))
    qt.writel(vic1 + 0x1F0, qt.readl(vic1 + 0x1E0))
    check('pipe 1 status cleared through 0x40034, VIC1 through 0x1F0',
          (qt.readl(ISP + 0x40038), qt.readl(vic1 + 0x1E0) & 0x11), (0, 0))
    # both pipes, the same frame: pipe 1's counter is 500000 ahead
    qt.writel(ISP + 0x16100 + 0x6C, buf[0])
    qt.writel(ISP + 0x16100 + 0x84, buf[0] + w * h)
    qt.writel(ISP + 0x17100 + 0x6C, buf[1])
    qt.writel(ISP + 0x17100 + 0x84, buf[1] + w * h)
    qt.writel(vic0 + 0x00, 1)
    qt.writel(ISP + 0x40020, 0xFFFFFFFF)
    qt.step(41 * 1000 * 1000)
    check('both pipes done', (qt.readl(ISP + 0x40028) & 1,
                              qt.readl(ISP + 0x40038) & 1), (1, 1))
    # the counter's second cell row (cells of 2 px from (4, 4)): the
    # leading digit is 0 on pipe 0 and 5 on pipe 1 -- 101 and 100 there
    row0 = qt.read(buf[0] + 6 * w, 64)
    row1 = qt.read(buf[1] + 6 * w, 64)
    check('pipe 0 lights the 0\'s right cell, pipe 1 the 5\'s does not',
          (row0[8] > 128, row1[8] > 128), (True, False))
    check('the two pictures differ in the counter', row0 != row1, True)
    check('and match in the scene below it',
          qt.read(buf[0] + 60 * w, w) == qt.read(buf[1] + 60 * w, w), True)
    qt.writel(vic0 + 0x00, 0)
    qt.writel(vic1 + 0x00, 0)


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
    # CSC to HSV (D_FMT 0x4, ipu_csc_nv12_to_hsv): a grey pixel pair and a
    # pure red pair (Y 81 Cb 90 Cr 240 is BT.601 red)
    w, h, out = 4, 2, 0x00420000
    qt.write(bg, bytes([128, 128, 81, 81] * 2) + bytes([128, 128, 90, 240]))
    qt.writel(ipu + 0x74, 1 << 3)
    qt.writel(ipu + 0x18, w << 16 | h)
    qt.writel(ipu + 0x0C, bg)
    qt.writel(ipu + 0x10, bg + w * h)
    qt.writel(ipu + 0x1C, w)
    qt.writel(ipu + 0x20, w << 16 | w)
    qt.writel(ipu + 0x08, 0x4)
    qt.writel(ipu + 0x24, out + 2 * w * h)             # S
    qt.writel(ipu + 0x2C, w)
    qt.writel(ipu + 0xB4, out)                         # H
    qt.writel(ipu + 0xB8, 2 * w)
    qt.writel(ipu + 0x30, out + 3 * w * h)             # V
    qt.writel(ipu + 0x34, w)
    qt.writel(ipu + 0x12C, 1 << 20)
    qt.writel(ipu + 0x74, 1)
    qt.step(200 * 1000)
    hp = struct.unpack('<4H', qt.read(out, 8))
    sp = tuple(qt.read(out + 2 * w * h, 4))
    vp = tuple(qt.read(out + 3 * w * h, 4))
    check('HSV: grey has no saturation', (sp[0], sp[1]), (0, 0))
    check('HSV: grey V is its level', vp[0], 130)
    check('HSV: red hue about 0', min(hp[2], 360 - hp[2]) <= 3, True)
    check('HSV: red saturated and bright', (sp[2] > 240, vp[2] > 240),
          (True, True))


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
        test_dmic(qt)
        test_codec_gain(qt)
        test_tcu_pwm(qt)
        test_dtrng(qt)
        test_aes(qt)
        test_hash(qt)
        test_rsa(qt)
        test_usbphy(qt)
        qt.close()
        qt = QTest(a.qemu, 't31')
        test_i2s_clock(qt)
        qt.close()
        qt = QTest(a.qemu, 't31', ['-device', 'w25q80,bus=ssi0,cs=0'])
        test_ssi(qt)
        qt.close()
        qt = QTest(a.qemu, 't41', ['-device', 'w25q80,bus=ssi0,cs=0'])
        test_ssi(qt, CORE_INTC)
        qt.close()
        qt = QTest(a.qemu, 't41')
        test_sadc(qt, CORE_INTC)
        test_sound(qt, CORE_INTC)
        test_pwm(qt)
        test_drawbox(qt)
        test_i2d(qt)
        test_dtrng(qt, CORE_INTC)
        test_aes(qt, CORE_INTC)
        test_hash(qt, CORE_INTC)
        test_rsa(qt)
        qt.close()
        qt = QTest(a.qemu, 't40')
        test_drawbox(qt)
        test_i2d(qt)
        qt.close()
        qt = QTest(a.qemu, 't20')
        test_aes(qt)
        test_tcu_pwm(qt)
        qt.close()

        qt = QTest(a.qemu, 't31,sensor=gc2053')
        test_isp(qt)
        qt.close()
        qt = QTest(a.qemu, 't31,sensor=gc2053')
        test_isp_stats(qt)
        qt.close()
        qt = QTest(a.qemu, 't21,sensor=jxf37')
        test_isp_stats_t21(qt)
        qt.close()
        qt = QTest(a.qemu, 't20,sensor=jxf22')
        test_isp_stats_apical(qt)
        qt.close()
        qt = QTest(a.qemu, 't40,sensor=gc4653,sensor1=gc4653')
        test_isp_t40_pipe1(qt)
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
