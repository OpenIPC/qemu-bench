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
MSC1 = 0x13460000
EFUSE = 0x13540000
CAR = 0x1F000000
BOOTROM = 0x1FC00000
SCRATCH = 0x00800000
AIP = 0x12B00000
AIP_ALIAS = 0x13090000
MONITOR = 0x130A0000
BSCALER = 0x13090000
VO = 0x130C0000
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


def test_tcu_irq(qt, intc=INTC):
    """Channel 2 as the motor drivers run it: EXTAL/64, full at 375 ticks
    (1 ms), its full interrupt unmasked -- TCU2 rises on time without
    any register being read."""
    print('ingenic-tcu channel interrupts')
    line = lambda: (qt.readl(intc + 0x00) >> 25) & 1
    qt.writel(TCU + 0x40 + 0x20 + 0x0C, 4 | (3 << 3))  # TCSR2: EXTAL / 64
    qt.writel(TCU + 0x40 + 0x20 + 0x00, 375)           # TDFR2
    qt.writel(TCU + 0x40 + 0x20 + 0x04, 375)           # TDHR2: no half
    qt.writel(TCU + 0x40 + 0x20 + 0x08, 0)
    qt.writel(TCU + 0x28, 0x00040004)                  # TFCR
    qt.writel(TCU + 0x38, 1 << 2)                      # TMCR: full 2 on
    qt.writel(TCU + 0x14, 1 << 2)                      # TESR: count
    qt.step(900 * 1000)
    check('no interrupt before the first full', line(), 0)
    qt.step(200 * 1000)
    check('TCU2 at 1 ms', line(), 1)
    n = 0
    for _ in range(10):
        qt.writel(TCU + 0x28, 1 << 2)                  # ack
        if line():
            break
        qt.step(1000 * 1000)
        n += line()
    check('one interrupt per millisecond for 10 ms', n, 10)
    qt.writel(TCU + 0x34, 1 << 2)                      # TMSR: masked
    qt.writel(TCU + 0x28, 1 << 2)
    qt.step(3 * 1000 * 1000)
    check('masked: the flag sets, the line stays low',
          ((qt.readl(TCU + 0x20) >> 2) & 1, line()), (1, 0))
    qt.writel(TCU + 0x18, 1 << 2)                      # TECR: stop
    qt.writel(TCU + 0x28, 0x00ff00ff)


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


# --- T41's UART4 / UART5 ---------------------------------------------------

def test_uart45(qt):
    print("T41's UART4 / UART5")
    for n, base, src in ((4, 0x10034000, 57), (5, 0x10035000, 56)):
        qt.writel(base + 0x1C, 0x5A)                    # SCR
        check('UART%d: SCR reads back' % n, qt.readl(base + 0x1C) & 0xff, 0x5A)
        check('UART%d: LSR, transmitter empty' % n,
              qt.readl(base + 0x14) & 0x60, 0x60)
        qt.writel(base + 0x04, 0x02)                    # IER: THRE
        check('UART%d: THRE interrupt on core INTC %d' % (n, src),
              (qt.readl(CORE_INTC + 0x20) >> (src - 32)) & 1, 1)
        qt.writel(base + 0x04, 0)
        check('UART%d: the line drops with IER clear' % n,
              (qt.readl(CORE_INTC + 0x20) >> (src - 32)) & 1, 0)


# --- SADC ------------------------------------------------------------------

def _msc_cmd(qt, cmd, arg, rtype):
    """Run one MSC1 command, return its three RES halfwords."""
    qt.writel(MSC1 + 0x2C, cmd)         # CMD
    qt.writel(MSC1 + 0x30, arg)         # ARG
    qt.writel(MSC1 + 0x0C, rtype)       # CMDAT: response format in bits 2:0
    qt.writel(MSC1 + 0x00, 1 << 2)      # CTRL START_OP
    return tuple(qt.readl(MSC1 + 0x34) & 0xffff for _ in range(3))


def _sdio52(qt, fn, addr, data=None):
    """CMD52 (IO_RW_DIRECT); data!=None writes.  Returns the data byte."""
    arg = (fn << 28) | ((addr & 0x1ffff) << 9)
    if data is not None:
        arg |= (1 << 31) | (data & 0xff)
    return _msc_cmd(qt, 52, arg, 5)[2] & 0xff


def test_sdio(qt):
    print('atbm-sdio (SDIO enumeration on MSC1)')
    qt.writel(MSC1 + 0x00, 1 << 2 | 2)          # start the clock
    # CMD5 IO_SEND_OP_COND: the inquiry (arg 0) is not ready; once a
    # voltage is offered the card reports ready (R4 bit 31)
    r0, _, _ = _msc_cmd(qt, 5, 0, 4)
    check('CMD5 inquiry: not ready', r0 & 0x80, 0)
    r0, _, _ = _msc_cmd(qt, 5, 0x300000, 4)
    check('CMD5 with voltage: card ready', r0 & 0x80, 0x80)
    # CMD3 SEND_RELATIVE_ADDR -> RCA 1
    _, r1, _ = _msc_cmd(qt, 3, 0, 6)
    rca = r1 >> 8
    check('CMD3: RCA assigned', rca, 1)
    # CMD7 select
    _msc_cmd(qt, 7, rca << 16, 1)
    # CCCR: revision 3.00, card capability
    check('CCCR revision', _sdio52(qt, 0, 0x00), 0x43)
    check('CCCR card capability', _sdio52(qt, 0, 0x08), 0xc2)
    # the common CIS pointer, then the MANFID tuple it points at
    ptr = (_sdio52(qt, 0, 0x09) | _sdio52(qt, 0, 0x0a) << 8 |
           _sdio52(qt, 0, 0x0b) << 16)
    check('common CIS pointer', ptr, 0x1000)
    check('CIS tuple is MANFID', _sdio52(qt, 0, ptr), 0x20)
    check('CIS MANFID length', _sdio52(qt, 0, ptr + 1), 4)
    vid = _sdio52(qt, 0, ptr + 2) | _sdio52(qt, 0, ptr + 3) << 8
    pid = _sdio52(qt, 0, ptr + 4) | _sdio52(qt, 0, ptr + 5) << 8
    check('CIS vendor id', vid, 0x007a)
    check('CIS product id', pid, 0x6011)
    # function 1: enable it, read it back ready; FBR class
    _sdio52(qt, 0, 0x02, 0x02)
    check('function 1 enabled and ready', _sdio52(qt, 0, 0x03) & 2, 2)
    check('FBR1 standard SDIO function', _sdio52(qt, 0, 0x100) & 0x0f, 0x0c)


SDHCI1 = 0x13070000


def _sdhci_cmd(qt, cmd, arg, resp=0x02, data=None):
    """One SDHCI1 command (48-bit response by default); data: a write."""
    rw = lambda a, v: qt.cmd('writew 0x%x 0x%x' % (a, v))
    qt.writel(SDHCI1 + 0x30, 0xffffffff)            # clear NORINTSTS
    flags = resp
    if data is not None:
        rw(SDHCI1 + 0x04, len(data))                # BLKSIZE
        rw(SDHCI1 + 0x06, 1)                        # BLKCNT
        flags |= 0x20                               # data present
    qt.writel(SDHCI1 + 0x08, arg)
    qt.writel(SDHCI1 + 0x0C, (cmd << 8 | flags) << 16)  # TRNMOD 0: write
    if data is not None:
        qt.writel(SDHCI1 + 0x20, struct.unpack('<I', data)[0])
    return qt.readl(SDHCI1 + 0x10)


def test_sdio_sdhci(qt):
    print('atbm-sdio (SDIO on the XBurst2 SDHCI MSC1, card interrupt)')
    qt.writel(SDHCI1 + 0x28, 0x0f << 8)             # bus power 3.3 V
    qt.writel(SDHCI1 + 0x2C, 0x5)                   # internal + SD clock
    qt.writel(SDHCI1 + 0x34, 0x01ff0000 | 0x00ff)   # status enables
    r = _sdhci_cmd(qt, 5, 0)
    check('CMD5 inquiry: not ready', r >> 31, 0)
    r = _sdhci_cmd(qt, 5, 0x300000)
    check('CMD5 with voltage: card ready', r >> 31, 1)
    rca = _sdhci_cmd(qt, 3, 0) >> 16
    check('CMD3: RCA assigned', rca, 1)
    _sdhci_cmd(qt, 7, rca << 16, 0x1b)
    check('CMD52 CCCR revision', _sdhci_cmd(qt, 52, 0x00) & 0xff, 0x43)
    # function-1 CMD53 to the HIF CONFIG register: set, then clear the
    # chip's CPU reset; its startup indication then holds the interrupt
    cfg = lambda v: _sdhci_cmd(qt, 53, 1 << 31 | 1 << 28 | 1 << 26 | 4,
                               data=struct.pack('<I', v))
    cfg(1 << 14)
    cfg(0)
    nis = lambda: qt.readl(SDHCI1 + 0x30) & 0xffff
    qt.step(50 * 1000 * 1000)
    check('no card interrupt while it is disabled', nis() & 0x100, 0)
    qt.writel(SDHCI1 + 0x34, 0x01ff0000 | 0x01ff)   # + card interrupt
    check('card interrupt status once enabled', nis() & 0x100, 0x100)
    qt.writel(SDHCI1 + 0x30, 0x100)
    check('it is level: write-1-clear leaves it', nis() & 0x100, 0x100)
    qt.writel(SDHCI1 + 0x38, 0x100)                 # signal enable
    check('signalled: MSC1 pending in the INTC',
          (qt.readl(CORE_INTC + 0x20) >> (36 - 32)) & 1, 1)
    qt.writel(SDHCI1 + 0x34, 0x01ff0000 | 0x00ff)
    check('status enable off drops it', nis() & 0x100, 0)


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

def test_sadc_dma(qt):
    """inferred DMA mode: ADCFG.DMA_EN, request 9, words at 0x10"""
    print('ingenic-sadc DMA (inferred)')
    qt.writel(PDMA + 0x1000, 1 | (0x3f << 16))
    qt.cmd('writew 0x%x 0x0000' % SADC)
    qt.cmd('writeb 0x%x 0xff' % (SADC + 0x08))
    for n in range(4):
        qt.qom_set('/machine/sadc', 'aux%d' % n, 0x100 * (n + 1) + n)
    ring = SCRATCH + 0x4000
    qt.write(ring, bytes(64))
    # 32-bit fixed source, incrementing target, 4-byte units: 8 words
    pdma_chan(qt, 5, SADC + 0x10, ring, 8, 9, (1 << 22) | (0 << 8))
    qt.writel(SADC + 0x04, 1 << 15)             # DMA_EN
    qt.cmd('writew 0x%x 0x000f' % SADC)         # AUX0..3, continuous
    qt.step(200000)
    words = struct.unpack('<8I', qt.read(ring, 32))
    pair = lambda a, b: (0x100 * (b + 1) + b) << 16 | (0x100 * (a + 1) + a)
    check('scans arrive as AUX1:AUX0, AUX3:AUX2 words',
          words, (pair(0, 1), pair(2, 3)) * 4)
    check('the channel moved its 8 words and stopped',
          (qt.readl(PDMA + 0xA0 + 0x10) >> 3) & 1, 1)
    rw = lambda a: int(qt.cmd('readw 0x%x' % a).split()[1], 16)
    check('ENA keeps its bits in DMA mode', rw(SADC) & 0xff, 0xf)
    qt.writel(SADC + 0x04, 0)
    qt.cmd('writew 0x%x 0x0000' % SADC)
    qt.step(100000)
    check('DMA off: a conversion clears ENA again', rw(SADC) & 0xff, 0)
    for n in range(4):
        qt.qom_set('/machine/sadc', 'aux%d' % n, 0x800 + 0x10 * n)



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


def test_aip(qt, intc=CORE_INTC):
    print('ingenic-aip (T41)')
    T, F, P = AIP, AIP + 0x200, AIP + 0x300
    src, dst, dst2, chain = (SCRATCH, SCRATCH + 0x10000, SCRATCH + 0x18000,
                             SCRATCH + 0x20000)
    W, H = 8, 4
    pic = bytes((x * 16 + y * 3) & 0xff for y in range(H) for x in range(W))
    qt.write(src, pic)

    def f_regs(d, sx, phase):
        return {0x10: sx, 0x14: sx, 0x18: phase, 0x1C: phase, 0x20: src,
                0x28: d, 0x30: (H << 16) | W,
                0x34: ((H * 0x10000 // sx) << 16) | (W * 0x10000 // sx),
                0x38: ((W * 0x10000 // sx) << 16) | W}

    def halve(p, w, h):
        # bilinear at the half-pixel phase of a 2:1 scale: the 2x2 mean
        return bytes((p[(2 * y) * w + 2 * x] + p[(2 * y) * w + 2 * x + 1] +
                      p[(2 * y + 1) * w + 2 * x] +
                      p[(2 * y + 1) * w + 2 * x + 1] + 2) // 4
                     for y in range(h // 2) for x in range(w // 2))

    qt.writel(F, 2)
    check('reset clears itself', qt.readl(F), 0)
    qt.writel(F + 0x08, 0x14)                       # one byte a pixel
    qt.writel(F + 0x0C, 0xffffffff)
    for r, v in f_regs(dst, 0x20000, 0x8010).items():
        qt.writel(F + r, v)
    qt.writel(F, 1)
    check('no status before the job ends', qt.readl(F + 0x04), 0)
    qt.step(100000)
    check('F: job done', qt.readl(F + 0x04) & 1, 1)
    check('F: the AIP1 line pending', (qt.readl(intc + 0x20) >> 7) & 1, 1)
    check('F: 8x4 -> 4x2 is the 2x2 means', qt.read(dst, 8), halve(pic, W, H))
    qt.writel(F + 0x04, 1)
    check('F: status write-1-clear drops the line',
          (qt.readl(F + 0x04), (qt.readl(intc + 0x20) >> 7) & 1), (0, 0))
    check('the units also at 0x13090000', qt.readl(AIP_ALIAS + 0x208), 0x14)

    # a chain of two nodes through the 0x1309xxxx addresses
    words = []
    for d in (dst, dst2):
        for r, v in f_regs(d, 0x20000, 0x8010).items():
            words += [v, 0x13090200 + r]
        words += [1, 0x93090200]
    qt.write(chain, struct.pack('<%dI' % len(words), *words))
    qt.write(dst, bytes(8))
    qt.writel(F + 0x3C, chain)
    qt.writel(F + 0x40, len(words) * 4)
    qt.writel(F, 4)
    qt.step(100000)
    check('F: chain done', qt.readl(F + 0x04) & 2, 2)
    check('F: both chain nodes ran', (qt.read(dst, 8), qt.read(dst2, 8)),
          (halve(pic, W, H), halve(pic, W, H)))
    qt.writel(F + 0x04, 3)

    # T: NV12 2x2 -> BGRA, BT.601 in Q20, the G terms negated
    coef = [1220542, 0, 1673527, 1220542, -409993, -852492, 1220542,
            2116026, 0]
    Y, U, V = 81, 90, 240
    qt.write(src, bytes([Y, Y, Y, Y, U, V]))
    qt.writel(T + 0x08, 0x1c)
    qt.writel(T + 0x0C, 0xffffffff)
    qt.writel(T + 0x14, src)
    qt.writel(T + 0x18, src + 4)
    qt.writel(T + 0x1C, dst)
    qt.writel(T + 0x20, (8 << 16) | 2)
    qt.writel(T + 0x24, (2 << 16) | 2)
    for i, c in enumerate(coef):
        qt.writel(T + 0x28 + 4 * i, c & 0xffffffff)
    def rgb():
        cl = lambda a: max(0, min(255, (a + (1 << 19)) >> 20))
        return [cl(coef[3 * i] * (Y - 16) + coef[3 * i + 1] * (U - 128) +
                   coef[3 * i + 2] * (V - 128)) for i in range(3)]
    r, g, b = rgb()
    for order, want in ((0, bytes([b, g, r, 0])), (5, bytes([r, g, b, 0])),
                        (13, bytes([0, r, g, b]))):
        qt.writel(T + 0x4C, (order << 24) | (128 << 8) | 16)
        qt.writel(T, 1)
        qt.step(100000)
        check('T: NV12 -> order %d' % order, qt.read(dst, 4), want)
        qt.writel(T + 0x04, 1)
    check('T: the AIP0 line low again', (qt.readl(intc + 0x20) >> 8) & 1, 0)

    # P: a 4-byte picture shifted by (1, 2), the pad value outside
    pic4 = bytes(range(4 * 4 * 4))
    qt.write(src, pic4)
    qt.writel(P + 0x08, 0x14)
    qt.writel(P + 0x0C, 0xffffffff)
    regs = {0x10: 3, 0x14: src, 0x1C: 16, 0x20: dst, 0x24: 16,
            0x28: (4 << 16) | 4, 0x2C: (4 << 16) | 4, 0x30: 0x55}
    # dst -> src: x' = x + 1, y' = y - 2 (Q16; 48 / 48 / 59-bit terms)
    m = [1, 0, 1, 0, 1, -2, 0, 0, 1]
    for row in range(3):
        a, b2, c = (v * 65536 & ((1 << 64) - 1) for v in m[3 * row:3 * row + 3])
        base = 0x34 + row * 0x14
        regs[base] = a & 0xffffffff
        regs[base + 4] = ((a >> 32) & 0xffff) | ((b2 & 0xffff) << 16)
        regs[base + 8] = (b2 >> 16) & 0xffffffff
        regs[base + 12] = c & 0xffffffff
        regs[base + 16] = (c >> 32) & 0x7ffffff
    for r, v in regs.items():
        qt.writel(P + r, v)
    qt.writel(P, 1)
    qt.step(100000)
    out = qt.read(dst, 64)
    want = b''.join(pic4[((y - 2) * 4 + x + 1) * 4:((y - 2) * 4 + x + 2) * 4]
                    if 0 <= y - 2 < 4 and x + 1 < 4 else b'\x55' * 4
                    for y in range(4) for x in range(4))
    check('P: shifted by the matrix, the pad value outside', out, want)
    check('P: the AIP2 line', (qt.readl(intc + 0x20) >> 6) & 1, 1)
    qt.writel(P + 0x04, 1)


def nndma_desc(ddr, oram_off, nbytes, more):
    """libvenus's descriptor (dma_func.cc w_d2o_orasetrun)"""
    n = (nbytes >> 6) - 1
    w0 = ddr | (((n >> 12) & 0x1f) << 1) | (1 if more else 0)
    w1 = ((n & 0xfff) << 20) | (oram_off >> 6)
    return struct.pack('<II', w0, w1)


def test_nndma(qt):
    print('ingenic-nndma (T40/T41 NNA DMA)')
    NNDMA, ORAMP = 0x12500000, 0x12600000
    src = bytes((i * 7 + 3) & 0xff for i in range(12288))
    qt.write(SCRATCH, src)
    # a chain of two: 8 KiB to ORAM +0x20000, 4 KiB to ORAM +0x40000
    qt.write(NNDMA + 8 * 5, nndma_desc(SCRATCH, 0x20000, 8192, True) +
             nndma_desc(SCRATCH + 8192, 0x40000, 4096, False))
    qt.writel(NNDMA + 0x8004, (1 << 31) | 5)            # read channel 1
    check('the chain reached ORAM, both descriptors',
          (qt.read(ORAMP + 0x20000, 8192) == src[:8192],
           qt.read(ORAMP + 0x40000, 4096) == src[8192:]), (True, True))
    check('the channel reads back done', qt.readl(NNDMA + 0x8004) >> 31, 0)
    # back to DDR through write channel 0, with a byte limit of 4 KiB
    qt.write(ORAMP + 0x20000, bytes(range(256)) * 32)
    qt.write(NNDMA + 8 * 9, nndma_desc(SCRATCH + 0x10000, 0x20000, 4096, True)
             + nndma_desc(SCRATCH + 0x11000, 0x21000, 4096, False))
    qt.write(SCRATCH + 0x11000, b'\xaa' * 64)
    qt.writel(NNDMA + 0x8010, (1 << 31) | ((4096 >> 6) << 12) | 9)
    check('write channel: ORAM to DDR', qt.read(SCRATCH + 0x10000, 4096),
          bytes(range(256)) * 16)
    check('the byte limit stops the chain', qt.read(SCRATCH + 0x11000, 64),
          b'\xaa' * 64)
    check('the 1 MiB ORAM: its last word', (qt.writel(ORAMP + 0xFFFFC, 0x5a5a),
          qt.readl(ORAMP + 0xFFFFC))[1], 0x5a5a)


def test_bscaler(qt, intc=CORE_INTC):
    print('ingenic-bscaler (T40)')
    src, dst, chain = SCRATCH, SCRATCH + 0x10000, SCRATCH + 0x20000
    W, H = 8, 4
    # libvenus's default YUV -> colour matrix: rows B, G, R, all positive
    coef = [1220542, 2116026, 0, 1220542, 409993, 852492, 1220542, 0,
            1673527]
    for i, c in enumerate(coef):
        qt.writel(BSCALER + 0xC0 + 4 * i, c)
    qt.writel(BSCALER + 0xE4, (128 << 24) | (16 << 16))

    def run_chain(regs):
        # (value, register) pairs as bscaler_hal writes them, 0x00 last
        words = []
        for r, v in regs:
            words += [v, r]
        words += [0x201, 0x80000000]
        qt.write(chain, struct.pack('<%dI' % len(words), *words))
        qt.writel(BSCALER + 0x3C, 0xffffffff)
        qt.writel(BSCALER + 0x34, chain)
        qt.writel(BSCALER + 0x38, len(words) * 4)
        qt.writel(BSCALER + 0x30, 0x21)

    def tile(fmt, sy, suv, ss, sw, sh, planes, ds, dw, dh, m):
        r = [(0x08, sy), (0x0C, suv)]
        r += [(0x18, p) for p in planes]
        r += [(0x04, fmt), (0x10, (sh << 16) | sw), (0x14, ss),
              (0x1C, (dh << 16) | dw), (0x20, ds), (0x40, 0), (0x44, 0)]
        r += [(0x48 + 4 * i, v & 0xffffffff) for i, v in enumerate(m)]
        return r

    # a 2:1 resize of BGRA: scale 2, half-pixel phase 0.5 -> the 2x2 means
    pic = bytes((x * 20 + y * 7 + c * 50) & 0xff for y in range(H)
                for x in range(W) for c in range(4))
    qt.write(src, pic)
    qt.writel(BSCALER + 0x30, 1 << 16)
    check('C soft reset clears itself', qt.readl(BSCALER + 0x30) >> 16, 0)
    run_chain(tile(0x0f, src, 0, W * 4, W, H, [dst], W * 2, W // 2, H // 2,
                   [0x20000, 0, 0x8000, 0, 0x20000, 0x8000, 0, 0, 0x10000]))
    check('C: no status before the chain ends',
          qt.readl(BSCALER + 0x30) & 0x10, 0)
    qt.step(100000)
    check('C: chain done (0x30 bit 4, 0x00 bit 8)',
          (qt.readl(BSCALER + 0x30) & 0x10, qt.readl(BSCALER) & 0x100),
          (0x10, 0x100))
    check('C: the BSCALER1 line (39) pending',
          (qt.readl(intc + 0x20) >> 7) & 1, 1)
    want = bytes((pic[((2 * y) * W + 2 * x) * 4 + c] +
                  pic[((2 * y) * W + 2 * x + 1) * 4 + c] +
                  pic[((2 * y + 1) * W + 2 * x) * 4 + c] +
                  pic[((2 * y + 1) * W + 2 * x + 1) * 4 + c] + 2) // 4
                 for y in range(H // 2) for x in range(W // 2)
                 for c in range(4))
    check('C: 8x4 BGRA -> 4x2 is the 2x2 means', qt.read(dst, 32), want)
    qt.writel(BSCALER, qt.readl(BSCALER) & ~0x100)
    qt.writel(BSCALER + 0x30, qt.readl(BSCALER + 0x30) & ~0x50)
    check('C: the driver\'s clears drop the line',
          (qt.readl(intc + 0x20) >> 7) & 1, 0)

    # NV12 -> RGBA (order 5) at the same size: BT.601 by the matrix
    Y, U, V = 81, 90, 240
    qt.write(src, bytes([Y] * 16 + [U, V] * 4))
    qt.writel(BSCALER + 0xE4, (128 << 24) | (16 << 16) | (5 << 4))
    run_chain(tile(0x0e, src, src + 16, 4, 4, 4, [dst], 16, 4, 4,
                   [0x10000, 0, 0, 0, 0x10000, 0, 0, 0, 0x10000]))
    qt.step(100000)
    yy, u, v = Y - 16, U - 128, V - 128
    r = round((1220542 * yy + 1673527 * v) / (1 << 20))
    g = round((1220542 * yy - 409993 * u - 852492 * v) / (1 << 20))
    b = round((1220542 * yy + 2116026 * u) / (1 << 20))
    rgb = bytes(max(0, min(255, c)) for c in (r, g, b))
    check('C: NV12 -> RGBA, BT.601', qt.read(dst + 20, 4), rgb + b'\0')
    qt.writel(BSCALER + 0x30, qt.readl(BSCALER + 0x30) & ~0x50)

    # a translation off the source: the pad value from 0x04 [15:8]
    qt.write(src, pic)
    run_chain(tile(0x0f | (77 << 8), src, 0, W * 4, W, H, [dst], W * 4, W,
                   H, [0x10000, 0, 6 << 16, 0, 0x10000, 0, 0, 0, 0x10000]))
    qt.step(100000)
    out = qt.read(dst, W * 4)
    check('C: x + 6 brings in source column 6, then the pad',
          (out[0:4], out[4 * 4:4 * 4 + 4]), (pic[6 * 4:7 * 4], bytes([77] * 4)))
    qt.writel(BSCALER + 0x30, qt.readl(BSCALER + 0x30) & ~0x50)

    # T: NV12 -> BGRA by lines (bs_covert_cfg / bs_covert_step_start)
    qt.write(src, bytes([Y] * 16 + [U, V] * 4))
    qt.write(dst, bytes(64))
    qt.writel(BSCALER + 0xE4, (128 << 24) | (16 << 16))
    for r_, v_ in ((0x88, src), (0x8C, src + 16), (0x90, (4 << 16) | 4),
                   (0x94, 4), (0x98, dst), (0xA4, 16), (0xAC, 4),
                   (0xBC, 0xffffffff), (0x80, 0x201)):
        qt.writel(BSCALER + r_, v_)
    qt.writel(BSCALER + 0x84, (2 << 16) | 1)
    qt.step(100000)
    check('T: step done (0x84 bit 2), the BSCALER0 line (40)',
          (qt.readl(BSCALER + 0x84) & 4, (qt.readl(intc + 0x20) >> 8) & 1),
          (4, 1))
    check('T: two lines of BGRA, the rest untouched',
          (qt.read(dst + 16, 4), qt.read(dst + 32, 4)),
          (bytes([rgb[2], rgb[1], rgb[0], 0]), bytes(4)))
    qt.writel(BSCALER + 0x84, qt.readl(BSCALER + 0x84) & ~4)
    qt.writel(BSCALER + 0x98, dst + 32)
    qt.writel(BSCALER + 0x84, (2 << 16) | 1)
    qt.step(100000)
    check('T: the next step carries on at line 2',
          qt.read(dst + 32, 4), bytes([rgb[2], rgb[1], rgb[0], 0]))
    check('T: the frame through: 0x80 bit 0 drops, bit 8 set',
          qt.readl(BSCALER + 0x80) & 0x101, 0x100)
    qt.writel(BSCALER + 0x84, qt.readl(BSCALER + 0x84) & ~4)


def test_monitor(qt, intc=CORE_INTC):
    print('ingenic-monitor (T40/T41)')
    ch = 2 * 0x100                                  # the sample's channel 2
    ctrl = lambda: qt.readl(MONITOR + ch)
    # bus_monitor.c monitor_start, monitor mode (2), a short window
    qt.writel(MONITOR + ch, 0x0000ff00)
    qt.writel(MONITOR + ch, ctrl() | (1 << 30))     # soft reset
    qt.writel(MONITOR + ch, ctrl() & ~(1 << 30))
    qt.writel(MONITOR + ch, ctrl() | (1 << 17))     # monitor mode
    qt.writel(MONITOR + ch, ctrl() | 0x80000000)    # clock
    qt.writel(MONITOR + ch + 0x04, 3000)            # 3000 cycles: 10 us
    qt.writel(MONITOR + ch + 0x0C, 0xffffffff)
    qt.writel(MONITOR + ch + 0x74, 0)               # unmask
    check('idle: no cycles counted', qt.readl(MONITOR + ch + 0x10), 0)
    qt.writel(MONITOR + ch, ctrl() | (1 << 29))     # enable
    qt.step(5000)
    cyc = qt.readl(MONITOR + ch + 0x10)
    check('5 us at 300 MHz: 1500 cycles', cyc, 1500)
    check('no slot interrupt yet', qt.readl(MONITOR + ch + 0x70), 0)
    qt.writel(MONITOR + ch, ctrl() | 1 | 4)         # lock, capture
    qt.step(1000)
    check('lock holds the captured count', qt.readl(MONITOR + ch + 0x2C), 1500)
    qt.step(10000)
    check('the window ends: TIME_SLOT_REACH', qt.readl(MONITOR + ch + 0x70)
          & 0x10, 0x10)
    check('the MONITOR line (53) pending', (qt.readl(intc + 0x20) >> 21) & 1, 1)
    qt.writel(MONITOR + ch + 0x78, 0x10)
    check('INT_CLR drops it', (qt.readl(MONITOR + ch + 0x70),
          (qt.readl(intc + 0x20) >> 21) & 1), (0, 0))
    qt.writel(MONITOR + ch, ctrl() | (1 << 30))
    check('soft reset stops counting', qt.readl(MONITOR + ch + 0x10), 0)
    qt.writel(MONITOR + 0x500 + 0x74, 3)
    check('the AHB channel at 0x500', qt.readl(MONITOR + 0x574), 3)


def test_vo(qt, intc=CORE_INTC):
    print('ingenic-vo (T41)')
    W, H = 64, 32
    ya, uva = SCRATCH, SCRATCH + 0x10000
    # left half red (Y 81, Cb 90, Cr 240), right half blue (41, 240, 110)
    qt.write(ya, b''.join(bytes([81] * (W // 2) + [41] * (W // 2))
                          for _ in range(H)))
    qt.write(uva, b''.join(bytes([90, 240] * (W // 4) + [240, 110] * (W // 4))
                           for _ in range(H // 2)))
    # ingenic_vo.c soft_reset_vo, work_init_vo, vo_reg_set
    qt.writel(VO + 0x004, qt.readl(VO + 0x004) | 1)
    check('DMA reset not acknowledged at once', qt.readl(VO + 0x008) & 1, 0)
    qt.step(20000)
    check('then acknowledged (TOP_STATUS bit 0)', qt.readl(VO + 0x008) & 1, 1)
    qt.writel(VO + 0x004, qt.readl(VO + 0x004) | 2)
    qt.writel(VO + 0x004, qt.readl(VO + 0x004) & ~2)
    qt.writel(VO + 0x004, qt.readl(VO + 0x004) & ~1)
    qt.writel(VO + 0x100, (H << 16) | W)
    qt.writel(VO + 0x104, (W << 16) | W)
    qt.writel(VO + 0x114, (64 << 16) | (1 << 12) | 0)
    qt.writel(VO + 0x118, (4 << 16) | 4)
    qt.writel(VO + 0x11C, 8)
    qt.writel(VO + 0x00C, (1 << 12) | (1 << 8) | 1)
    qt.writel(VO + 0x120, 1)
    check('no frame queued', qt.readl(VO + 0x128) & 8, 0)
    qt.writel(VO + 0x008, 1)
    qt.writel(VO + 0x108, ya)
    qt.writel(VO + 0x10C, uva)
    check('the frame queued (ADDR_INFO bit 3)', qt.readl(VO + 0x128) & 8, 8)
    qt.step(40 * 1000 * 1000)
    check('frame done after a frame time', qt.readl(VO + 0x010) & 1, 1)
    check('the VO line (43) pending', (qt.readl(intc + 0x20) >> 11) & 1, 1)
    check('the queue empty again', qt.readl(VO + 0x128) & 8, 0)
    check('one frame out', qt.qom_get('/machine/vo', 'frames'), 1)
    ppm = os.path.join(qt.dir, 'vo.ppm')
    qt.qmp('screendump', filename=ppm)
    with open(ppm, 'rb') as f:
        data = f.read()
    hdr = data.split(b'\n', 3)
    px = hdr[3]
    left = px[(H // 2 * W + 8) * 3:(H // 2 * W + 8) * 3 + 3]
    right = px[(H // 2 * W + W - 8) * 3:(H // 2 * W + W - 8) * 3 + 3]
    check('the display shows it: %s, red left, blue right' % hdr[1].decode(),
          (hdr[1], left[0] > 200 and left[2] < 60, right[2] > 200 and
           right[0] < 60), (b'%d %d' % (W, H), True, True))
    qt.writel(VO + 0x014, qt.readl(VO + 0x014) | 1)
    qt.writel(VO + 0x014, qt.readl(VO + 0x014) & ~1)
    check('INT_CLC clears it', (qt.readl(VO + 0x010) & 1,
          (qt.readl(intc + 0x20) >> 11) & 1), (0, 0))


def test_dpu(qt, intc=CORE_INTC):
    print('ingenic-dpu (T40/T41 LCD controller)')
    DPU, W, H = 0x13050000, 160, 120
    FD, LD, BUF = SCRATCH, SCRATCH + 0x1000, SCRATCH + 0x10000
    le = lambda *w: b''.join(struct.pack('<I', v) for v in w)
    # layer 0: RGB565 red, the whole frame; layer 1: ARGB8888 blue, half
    # transparent, 40x40 at (100, 60); layer 2: NV12 white 32x32 at (8, 8)
    # scaled to 64x64
    l0, l1, l2 = BUF, BUF + 0x10000, BUF + 0x20000
    qt.write(l0, struct.pack('<H', 0xF800) * (W * H))
    qt.write(l1, struct.pack('<I', 0x800000FF) * (40 * 40))
    qt.write(l2, bytes([235]) * (32 * 32) + bytes([128]) * (32 * 16))
    size = lambda w, h: w | (h << 16)
    qt.write(LD, le(size(W, H), 2 << 16, l0, 0, 0, 0, 0, 0, 0, W, 0, 0))
    qt.write(LD + 0x40, le(size(40, 40), 5 << 16, l1, 0, 0, 0,
                           size(100, 60), 0, 0, 40, 0, 0))
    qt.write(LD + 0x80, le(size(32, 32), 8 << 16, l2, size(64, 64), 0, 0,
                           size(8, 8), 0, 0, 32, l2 + 32 * 32, 32))
    en = (7 << 4) | (0 << 8) | (1 << 10) | (2 << 12)
    qt.write(FD, le(FD, size(W, H), 1, 0, 0, LD, LD + 0x40, LD + 0x80, 0,
                    en, 0))
    qt.writel(DPU + 0x200C, (1 << 17) | (1 << 14) | (1 << 6))   # INTC
    qt.writel(DPU + 0x0000, FD)
    qt.writel(DPU + 0x0004, 1)                                  # START
    check('WORKING after START', qt.readl(DPU + 0x2004) & 1, 1)
    qt.step(20 * 1000 * 1000)
    st = qt.readl(DPU + 0x2004)
    check('one frame: CMP_START, layer 0..2 ends, CMP_END, DISP_END; '
          'stopped (the descriptor\'s stop bit)',
          st & 0x27E01, (1 << 17) | (1 << 14) | (7 << 10) | (1 << 9))
    check('INT_FLAG = ST & INTC, line 20 up',
          (qt.readl(DPU + 0x2010), (qt.readl(intc) >> 20) & 1),
          ((1 << 17) | (1 << 14), 1))
    check('one frame shown', qt.qom_get('/machine/dpu', 'frames'), 1)
    ppm = os.path.join(qt.dir, 'dpu.ppm')
    qt.qmp('screendump', filename=ppm, device='dpu')
    data = open(ppm, 'rb').read()
    hdr = data.split(b'\n', 3)
    px = lambda x, y: tuple(hdr[3][(y * W + x) * 3:(y * W + x) * 3 + 3])
    check('the display is %s' % hdr[1].decode(), hdr[1], b'%d %d' % (W, H))
    check('the RGB565 background is red', px(90, 20), (255, 0, 0))
    check('the ARGB layer blends half blue over it',
          px(120, 80), (127, 0, 128))
    check('the NV12 layer, scaled 2x, is white to (71, 71)',
          (px(10, 10), px(70, 70), px(74, 74)),
          ((255, 255, 255), (255, 255, 255), (255, 0, 0)))
    qt.writel(DPU + 0x2008, (1 << 17) | (1 << 14))
    check('CLR_ST clears them, the line drops',
          (qt.readl(DPU + 0x2010), (qt.readl(intc) >> 20) & 1), (0, 0))
    # a looping chain: the next descriptor is itself, no stop bit
    qt.write(FD + 8, le(0))
    qt.writel(DPU + 0x0004, 1)
    qt.step(60 * 1000 * 1000)
    check('a looping chain keeps running', (qt.readl(DPU + 0x2004) & 1,
          qt.qom_get('/machine/dpu', 'frames') >= 3), (1, True))
    qt.writel(DPU + 0x2000, 1 << 3)                             # GEN_STP
    qt.step(20 * 1000 * 1000)
    check('a general stop ends after the frame, STOP_DISP_ACK',
          qt.readl(DPU + 0x2004) & 0x41, 0x40)


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


def test_hash_cpu(qt, intc=INTC):
    """CPU mode, as modelled (inferred): HSDI words in memory order"""
    print('ingenic-hash CPU mode (HSDI)')
    algs = ('md5', 'sha1', 'sha224', 'sha256', 'sha384', 'sha512')
    for code, alg in enumerate(algs):
        for msg in (b'abc', bytes(range(256)) + b'quick brown fox' * 3):
            data = md_pad(msg, alg)
            qt.writel(HASH + 0x00, 0x30000000 | code << 1 | 1 |
                      (0 if alg == 'md5' else 3 << 7))
            qt.writel(HASH + 0x00, qt.readl(HASH + 0x00) | 1 << 4)
            qt.writel(HASH + 0x04, 3)
            qt.writel(HASH + 0x08, 3)
            qt.writel(HASH + 0x10, len(data) // 64)
            for i in range(0, len(data), 4):
                qt.writel(HASH + 0x14, struct.unpack_from('<I', data, i)[0])
            want = hashlib.new(alg, msg).digest()
            got = b''.join(struct.pack('<I', qt.readl(HASH + 0x18))
                           for i in range(len(want) // 4))
            check('%s of %d bytes, %d blocks' %
                  (alg, len(msg), len(data) // (128 if code >= 4 else 64)),
                  got, want)
    check('all rounds done after HSTC blocks, the INTC line',
          (qt.readl(HASH + 0x04) & 3, (qt.readl(intc) >> 22) & 1), (3, 1))
    qt.writel(HASH + 0x04, 3)
    # a block's round, then the state loaded again empties a part block
    qt.writel(HASH + 0x00, 0x30000000 | 3 << 1 | 1 | 3 << 7 | 1 << 4)
    qt.writel(HASH + 0x10, 0)
    for i in range(16):
        qt.writel(HASH + 0x14, 0)
    check('one block: round done only (HSTC 0)', qt.readl(HASH + 0x04) & 3, 1)
    qt.writel(HASH + 0x04, 3)
    qt.writel(HASH + 0x14, 0x11111111)
    qt.writel(HASH + 0x00, qt.readl(HASH + 0x00) | 1 << 4)
    data = md_pad(b'', 'sha256')
    for i in range(0, 64, 4):
        qt.writel(HASH + 0x14, struct.unpack_from('<I', data, i)[0])
    got = b''.join(struct.pack('<I', qt.readl(HASH + 0x18)) for i in range(8))
    check('loading the state drops a part-filled block',
          got, hashlib.sha256(b'').digest())
    qt.writel(HASH + 0x00, 0)
    qt.writel(HASH + 0x04, 3)
    qt.writel(HASH + 0x08, 0)


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


def test_uart_dma(qt):
    """jz_uart.c's DMA mode: 32-byte bursts at the trigger level, the
    tail for the receive timeout (IER bit 4 alone), on UART2 looped back"""
    print('UART DMA (T31 UART2, loopback)')
    U = 0x10032000
    rb = lambda r: int(qt.cmd('readb 0x%x' % (U + 4 * r)).split()[1], 16)
    wb = lambda r, v: qt.cmd('writeb 0x%x 0x%x' % (U + 4 * r, v))
    wb(3, 0x80)
    wb(0, 1)                                    # DLL: 1.5 Mbaud
    wb(1, 0)
    wb(3, 0x03)                                 # 8N1
    wb(4, 0x10)                                 # MCR: loopback
    wb(2, 0x8F)                                 # FE, both resets, DMS, 32
    wb(1, 0x10)                                 # IER: the RX timeout only
    check('IER keeps bit 4 (RTOIE)', rb(1), 0x10)
    qt.writel(PDMA + 0x1000, 1 | (0x3f << 16))
    data = bytes((i * 37 + 5) & 0xff for i in range(72))
    tx, rx = SCRATCH + 0xA000, SCRATCH + 0xB000
    qt.write(tx, data)
    qt.write(rx, bytes(72))
    # RX: RBR -> memory, 8-bit ports, 32-byte units, two of them
    c = PDMA + 0x20 * 5
    qt.writel(c + 0x00, U)
    qt.writel(c + 0x04, rx)
    qt.writel(c + 0x08, 2)
    qt.writel(c + 0x0C, 0x11)                   # UART2 RX
    qt.writel(c + 0x14, (1 << 22) | (1 << 14) | (1 << 12) | (4 << 8) | 2)
    qt.writel(c + 0x10, (1 << 31) | 1)
    # TX: memory -> THR, byte units
    c = PDMA + 0x20 * 4
    qt.writel(c + 0x00, tx)
    qt.writel(c + 0x04, U)
    qt.writel(c + 0x08, len(data))
    qt.writel(c + 0x0C, 0x10)                   # UART2 TX
    qt.writel(c + 0x14, (1 << 23) | (1 << 14) | (1 << 12) | (1 << 8) | 2)
    qt.writel(c + 0x10, (1 << 31) | 1)
    qt.step(2 * 1000 * 1000)                    # 72 characters and then some
    check('TX: all 72 bytes went out, the channel done',
          (qt.readl(PDMA + 0x80 + 0x08), qt.readl(PDMA + 0x80 + 0x10) & 8),
          (0, 8))
    check('RX: two 32-byte bursts by DMA',
          (qt.read(rx, 64), qt.readl(PDMA + 0xA0 + 0x08)), (data[:64], 0))
    check('both channels interrupted', qt.readl(PDMA + 0x1004) & 0x30, 0x30)
    check('the 8-byte tail is left for the timeout interrupt: CTI',
          (rb(5) & 1, rb(2) & 0x0F), (1, 0x0C))
    check('PIO reads take the tail', bytes(rb(0) for _ in range(8)),
          data[64:])
    check('the FIFO empty again', rb(5) & 1, 0)
    qt.writel(PDMA + 0x1004, 0)


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


def test_des(qt):
    print('ingenic-des')
    DES = 0x10061000

    def run(cr2, keys, words, iv=(0, 0)):
        qt.writel(DES + 0x00, 0)
        qt.writel(DES + 0x04, cr2)
        for i, k in enumerate(keys):
            qt.writel(DES + 0x10 + 4 * i, k)
        qt.writel(DES + 0x28, iv[0])
        qt.writel(DES + 0x2C, iv[1])
        qt.writel(DES + 0x00, 1)                        # work
        for w in words:
            qt.writel(DES + 0x30, w)
        out = []
        for _ in words:
            if qt.readl(DES + 0x08) & 1:
                break
            out.append(qt.readl(DES + 0x34))
        return tuple(out)

    k = (0x13345779, 0x9BBCDFF1)
    # FIPS 46's worked example
    check('DES ECB: 0123456789ABCDEF -> 85E813540F0AB405',
          run(0, k, (0x01234567, 0x89ABCDEF)), (0x85E81354, 0x0F0AB405))
    check('and back (decrypt)', run(1 << 3, k, (0x85E81354, 0x0F0AB405)),
          (0x01234567, 0x89ABCDEF))
    check('triple DES with K1 = K2 = K3 is DES',
          run(1 << 1, k * 3, (0x01234567, 0x89ABCDEF)), (0x85E81354, 0x0F0AB405))
    two = (0x01234567, 0x89ABCDEF, 0x01234567, 0x89ABCDEF)
    cbc = run(1 << 2, k, two, (0x12345678, 0x9ABCDEF0))
    check('CBC: equal blocks cipher differently', cbc[:2] != cbc[2:], True)
    check('CBC and back', run((1 << 2) | (1 << 3), k, cbc,
                              (0x12345678, 0x9ABCDEF0)), two)
    check('the output FIFO empty again', qt.readl(DES + 0x08) & 1, 1)
    # the PDMA: 64 bytes in, little endian, triple DES ECB, out
    src, dst = SCRATCH + 0xC000, SCRATCH + 0xD000
    data = bytes(range(64))
    qt.write(src, data)
    qt.write(dst, bytes(64))
    tk = (0x01020304, 0x05060708, 0x11121314, 0x15161718, 0x21222324,
          0x25262728)
    qt.writel(DES + 0x00, 0)
    qt.writel(DES + 0x04, (1 << 4) | (1 << 1) | 1)      # LE, TDES, DMA
    for i, w in enumerate(tk):
        qt.writel(DES + 0x10 + 4 * i, w)
    qt.writel(DES + 0x00, 1)
    qt.writel(PDMA + 0x1000, 1)
    rxc, txc = PDMA + 0x20 * 8, PDMA + 0x20 * 9
    qt.writel(rxc + 0x00, DES + 0x34)
    qt.writel(rxc + 0x04, dst)
    qt.writel(rxc + 0x08, 16)
    qt.writel(rxc + 0x0C, 0x2F)
    qt.writel(rxc + 0x14, 1 << 22)                      # 32-bit, DAI
    qt.writel(rxc + 0x10, (1 << 31) | 1)
    qt.writel(txc + 0x00, src)
    qt.writel(txc + 0x04, DES + 0x30)
    qt.writel(txc + 0x08, 16)
    qt.writel(txc + 0x0C, 0x2E)
    qt.writel(txc + 0x14, 1 << 23)                      # 32-bit, SAI
    qt.writel(txc + 0x10, (1 << 31) | 1)
    enc = qt.read(dst, 64)
    check('both channels done', (qt.readl(txc + 0x10) & 8,
                                 qt.readl(rxc + 0x10) & 8), (8, 8))
    check('the DMA output is ciphered', enc != data and enc != bytes(64), True)
    words = struct.unpack('<16I', enc)
    qt.writel(txc + 0x10, 0)
    qt.writel(rxc + 0x10, 0)
    qt.writel(PDMA + 0x1004, 0)
    back = run((1 << 4) | (1 << 3) | (1 << 1), tk, words)
    check('decrypted by the CPU, the input again', struct.pack('<16I', *back),
          data)
    qt.writel(DES + 0x00, 0)


# --- sound: PDMA, AIC, codec ----------------------------------------------

def pdma_ring(qt, ch, desc, buf, period, nper, drt, dcm, port=AIC + 0x34,
              unit=2):
    """jz_dma.c's cyclic transfer: a ring of 8-word linked descriptors"""
    for i in range(nper):
        nxt = desc + 32 * ((i + 1) % nper)
        dsa, dta = (port, buf + i * period) if drt in (5, 7, 13, 0x21) else \
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
    # the microphone path on, ALC on (T31's codec_set_buildin_mic); on
    # T41 the ADC on and the gain the driver leaves (0x1F, its unity)
    qt.writel(CODEC + 0x8C, 0x90)
    qt.writel(CODEC + 0x98, 0x1f)
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


def test_codec_gain_t41(qt):
    """T41's codec: record gain, ALC, digital volume, ADC switch, by PIO"""
    print('ingenic-aic codec gains (T41 layout)')

    def peak(cmiccr, calcgr, cgainr=0, clavr=0xC3):
        qt.writel(AIC + 0x04, 0)
        qt.writel(AIC + 0x00, 1 | (1 << 3))             # empty the FIFOs
        qt.writel(CODEC + 0x110, 0x5)
        qt.writel(CODEC + 0x8C, cmiccr)
        qt.writel(CODEC + 0x98, calcgr)
        qt.writel(CODEC + 0x28, cgainr)
        qt.writel(CODEC + 0x20, clavr)
        qt.writel(AIC + 0x04, (1 << 16) | (1 << 12) | 1)  # EREC, mono
        qt.step(1500 * 1000)                            # 24 frames
        v = [qt.readl(AIC + 0x34) & 0xffff for _ in range(24)]
        return max(abs(x - 65536 if x >= 32768 else x) for x in v)

    check('the digital volumes come out of reset at 0 dB',
          (qt.readl(CODEC + 0x18), qt.readl(CODEC + 0x20)), (0xF1, 0xC3))
    check('gain 0x1F (the driver\'s): unity', peak(0x80, 0x1f), 8192)
    check('gain 0x1B: -6 dB', peak(0x80, 0x1b), 4106)
    check('ALC (CGAINR bit 4) holds the level', peak(0x80, 0x1b, 0x10), 8192)
    check('CLAVR 0xCF: +6 dB digital', peak(0x80, 0x1f, 0, 0xCF), 16345)
    check('CLAVR 0: muted', peak(0x80, 0x1f, 0, 0), 0)
    check('the ADC off: silence', peak(0x00, 0x1f), 0)
    qt.writel(AIC + 0x04, 0)


def test_asdmic(qt):
    """T41's audio system: DMIC to FIFO 5 to DMA channel 5, as as-dma.c"""
    print('ingenic-asdmic (T41)')
    AS = 0x134D0000
    ring, descs, frag = SCRATCH + 0xA000, SCRATCH + 0xB000, 320
    qt.write(ring, bytes(2 * frag))
    for i in range(2):                  # two linked descriptors, no TIE
        d0 = (1 << 2) | (1 << 4) | (1 << 8)     # LINK, TSZ 1 (4 words), BAI
        qt.write(descs + 16 * i, struct.pack('<4I', d0, ring + i * frag,
                                             descs + 16 * ((i + 1) % 2),
                                             frag // 16))
    qt.writel(AS + 0xA000, 1 << 31)                     # set_dmic_reset
    check('CR0 RESET self-clears', qt.readl(AS + 0xA000) >> 31, 0)
    qt.writel(AS + 0xA000, (1 << 6))                    # 16 kHz, 1 mic, 16-bit
    qt.writel(AS + 0x0C + 0x18 * 5, 0x3c)               # ingenic_dmic_dma_start
    qt.writel(AS + 0x14 + 0x18 * 5, descs)
    qt.writel(AS + 0x08 + 0x18 * 5, 1)
    qt.writel(AS + 0x1004 + 0x10 * 5, 1)
    qt.writel(AS + 0xA000, (1 << 6) | 1)                # EN
    qt.step(10 * 1000 * 1000)
    dba = qt.readl(AS + 0x18 * 5)
    check('10 ms at 16 kHz: 160 samples into the ring of 2 x 160',
          dba, ring + 320)
    got = struct.unpack('<160h', qt.read(ring, 320))
    want = [round(8192 * math.sin(2 * math.pi * 1000 * k / 16000))
            for k in range(160)]
    check('the samples are the 1 kHz tone, two to a word',
          got[:24], tuple(want[:24]))
    qt.step(15 * 1000 * 1000)
    check('the ring wraps through the link (25 ms: 400 samples)',
          qt.readl(AS + 0x18 * 5), ring + 160)
    check('no interrupt without TIE', qt.readl(AS + 0x110), 0)
    qt.writel(AS + 0xA000, 1 << 6)                      # stop
    qt.writel(AS + 0x08 + 0x18 * 5, 0)
    dba = qt.readl(AS + 0x18 * 5)
    qt.step(10 * 1000 * 1000)
    check('the DMA stops with the DMIC', qt.readl(AS + 0x18 * 5), dba)


def test_dmic_trigger(qt):
    """The voice trigger, as the voice-wakeup firmware arms it"""
    print('ingenic-dmic voice trigger')
    fired = lambda: ((qt.readl(DMIC + 0x0C) & 1),
                     (qt.readl(DMIC + 0x38) >> 18) & 1)
    line = lambda: (qt.readl(INTC + 0x00) >> 12) & 1
    qt.writel(DMIC + 0x00, 1 << 31)                     # empty the FIFO
    qt.writel(DMIC + 0x34, 32)
    qt.writel(DMIC + 0x0C, 0x3f)
    qt.writel(DMIC + 0x08, 0x3f)
    # DEEP_SLEEP: 16 kHz prefetch, data path off (SR 3), threshold 5000
    qt.writel(DMIC + 0x18, 5000)
    qt.writel(DMIC + 0x10, (1 << 3) | (2 << 1))          # mode 0
    qt.qom_set('/machine/dmic', 'tone-amp', 4000)
    qt.writel(DMIC + 0x00, (3 << 6) | (1 << 8) | (1 << 12) | 3)
    qt.writel(DMIC + 0x08, 0x3f & ~((1 << 0) | (1 << 4)))
    qt.step(20 * 1000 * 1000)
    check('below THRL (peak 4000 vs 5000): no trigger', fired(), (0, 0))
    check('...and the data path off: nothing in the FIFO',
          qt.readl(DMIC + 0x38) & 0x7f, 0)
    qt.qom_set('/machine/dmic', 'tone-amp', 8192)
    qt.step(2 * 1000 * 1000)
    check('over THRL: TRIFLG and FSR TRIGS', fired(), (1, 1))
    check('the trigger interrupts (INTC 12)', line(), 1)
    qt.writel(DMIC + 0x0C, 0x3f)
    check('INTCR write-1-clear drops the line but TRIGS stays latched',
          (fired(), line()), ((0, 1), 0))
    qt.step(5 * 1000 * 1000)
    check('latched: no new flag until rearmed', fired(), (0, 1))
    # mode 2: more than TRINMAX samples over THRL
    qt.writel(DMIC + 0x10, (2 << 16) | (1 << 3) | (2 << 1) | 1)  # TRI_CLR
    check('TRI_CLR rearms (and reads back clear)',
          (fired(), qt.readl(DMIC + 0x10) & 1), ((0, 0), 0))
    qt.writel(DMIC + 0x20, 1000)
    qt.step(int(500e6 / 16000))                         # 500 samples
    check('mode 2: fewer than TRINMAX samples over THRL, no trigger',
          fired(), (0, 0))
    qt.step(200 * 1000 * 1000)
    check('mode 2: past TRINMAX, the trigger fires', fired(), (1, 1))
    qt.writel(DMIC + 0x00, (3 << 6) | (1 << 30) | 3)     # RESET_TRI
    check('RESET_TRI rearms and clears itself',
          ((qt.readl(DMIC + 0x38) >> 18) & 1,
           (qt.readl(DMIC + 0x00) >> 30) & 1), (0, 0))
    qt.qom_set('/machine/dmic', 'tone-amp', 0)
    qt.writel(DMIC + 0x0C, 0x3f)
    qt.step(20 * 1000 * 1000)
    check('silence: no trigger', fired(), (0, 0))
    qt.writel(DMIC + 0x00, 0)
    qt.writel(DMIC + 0x08, 0x3f)
    qt.writel(DMIC + 0x0C, 0x3f)
    qt.qom_set('/machine/dmic', 'tone-amp', 8192)


def test_dmic(qt, DMIC=DMIC, drq=13, name='T31'):
    print('ingenic-dmic (%s)' % name)
    qt.writel(PDMA + 0x1000, 1 | (0x3f << 16))
    check('every interrupt masked out of reset', qt.readl(DMIC + 0x08), 0x3f)
    # audio_dmic.c: 16 kHz, one microphone, packed, DMA on request 13
    ring, desc, period = SCRATCH + 0x6000, SCRATCH + 0x7000, 128
    qt.write(ring, bytes(4 * period))
    dcm32 = (0 << 14) | (0 << 12) | (0 << 8) | 3    # 32-bit ports and units
    pdma_ring(qt, 6, desc, ring, period, 4, drq, dcm32 | (1 << 22),
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
    # Block Erase 32KB (52h) at 1 MiB + 128 KiB: the block, not its
    # neighbours
    sfc_v1(qt, 0x06, data=False)
    sfc_v1(qt, 0x52, addr=0x120000, alen=3, data=False)
    for a, want in ((0x11FFF0, img[0x11FFF0:0x120000]),
                    (0x120000, b'\xff' * 16), (0x127FF0, b'\xff' * 16),
                    (0x128000, img[0x128000:0x128010])):
        sfc_v1(qt, 0x03, addr=a, alen=3, length=16, dma=SCRATCH + 0x300)
        check('32 KiB erase at 0x%x' % a, qt.read(SCRATCH + 0x300, 16),
              bytes(want))


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
    # PC0 a falling-edge interrupt, driven through /machine/gpio pc
    pc = GPIO + 0x2000
    qt.writel(pc + 0x14, 1)                     # INTS
    qt.writel(pc + 0x34, 1)                     # PAT1S: edge
    qt.writel(pc + 0x48, 1)                     # PAT0C: falling
    qt.writel(pc + 0x28, 1)                     # MSKC
    qt.writel(pc + 0x58, 0xFFFFFFFF)            # FLGC
    line = lambda: (qt.readl(INTC + 0x00) >> 15) & 1
    check('no interrupt while PC0 stays high', line(), 0)
    qt.qom_set('/machine/gpio', 'pc', 0xFFFFFFFE)
    check('pulling PC0 low reads 0 and flags it',
          (qt.readl(pc + 0x00) & 1, qt.readl(pc + 0x50) & 1, line()), (0, 1, 1))
    qt.writel(pc + 0x58, 1)
    qt.qom_set('/machine/gpio', 'pc', 0xFFFFFFFF)
    check('a rising edge does not', (qt.readl(pc + 0x50) & 1, line()), (0, 0))
    check('qom-get reads the levels back',
          qt.qom_get('/machine/gpio', 'pc'), 0xFFFFFFFF)


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


def pdma_chan(qt, ch, dsa, dta, dtc, drt, dcm):
    """a channel without descriptors (NDES), started at once"""
    c = PDMA + 0x20 * ch
    qt.writel(c + 0x10, 0)
    qt.writel(c + 0x00, dsa)
    qt.writel(c + 0x04, dta)
    qt.writel(c + 0x08, dtc)
    qt.writel(c + 0x0C, drt)
    qt.writel(c + 0x14, dcm)
    qt.writel(c + 0x10, (1 << 31) | 1)


def test_i2c_dma(qt):
    """DW_apb_i2c DMA handshake: GC2053 at 0x37 on I2C0, PDMA 0x24/0x25"""
    print('ingenic-i2c DMA (PDMA requests 0x24 / 0x25)')
    qt.writel(PDMA + 0x1000, 1 | (0x3f << 16))
    qt.writel(I2C0 + 0x6C, 0)
    qt.writel(I2C0 + 0x04, 0x37)
    qt.writel(I2C0 + 0x8C, 0)                   # DMATDLR
    qt.writel(I2C0 + 0x90, 0)                   # DMARDLR: a byte requests
    qt.writel(I2C0 + 0x6C, 1)
    qt.writel(I2C0 + 0x88, 3)                   # RDMAE | TDMAE
    cmds, rx = SCRATCH + 0x3000, SCRATCH + 0x3100
    qt.write(rx, bytes(4))
    # the ID read: register 0xF0, then two reads (restart, then stop)
    qt.write(cmds, struct.pack('<3H', 0xF0, 0x100 | 0x400, 0x100 | 0x200))
    sp16 = (2 << 14) | (2 << 12) | (2 << 8)     # 16-bit ports and units
    sp8 = (1 << 14) | (1 << 12) | (1 << 8)      # 8-bit
    pdma_chan(qt, 4, I2C0 + 0x10, rx, 2, 0x25, (1 << 22) | sp8)
    pdma_chan(qt, 3, cmds, I2C0 + 0x10, 3, 0x24, (1 << 23) | sp16)
    check('three commands out, two bytes in, both channels done',
          ((qt.readl(PDMA + 0x60 + 0x10) >> 3) & 1,
           (qt.readl(PDMA + 0x80 + 0x10) >> 3) & 1), (1, 1))
    check('the GC2053 ID by DMA', list(qt.read(rx, 2)), [0x20, 0x53])
    check('the RX FIFO drained by the reads', qt.readl(I2C0 + 0x78), 0)
    # a register written by DMA, read back by PIO
    qt.write(cmds, struct.pack('<2H', 0x80, 0x5A | 0x200))
    pdma_chan(qt, 3, cmds, I2C0 + 0x10, 2, 0x24, (1 << 23) | sp16)
    check('a write by DMA lands', i2c_read_reg(qt, I2C0, 0x37, 0x80), [0x5A])
    # RX requests wait for DMARDLR + 1 bytes
    qt.writel(I2C0 + 0x88, 1)
    qt.writel(I2C0 + 0x90, 1)
    qt.write(rx, bytes(4))
    pdma_chan(qt, 4, I2C0 + 0x10, rx, 2, 0x25, (1 << 22) | sp8)
    qt.writel(I2C0 + 0x10, 0xF0)
    qt.writel(I2C0 + 0x10, 0x100 | 0x400)
    check('one byte under DMARDLR + 1: no request', list(qt.read(rx, 2)),
          [0, 0])
    qt.writel(I2C0 + 0x10, 0x100 | 0x200)
    check('the second byte: both move', list(qt.read(rx, 2)), [0x20, 0x53])
    qt.writel(I2C0 + 0x88, 0)
    qt.writel(I2C0 + 0x6C, 0)


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


def test_isp_xstats(qt):
    """T31's other statistics: AE1, AF, ADR, defog, WDR (tisp_init's banks)."""
    print('ingenic-isp AE1 / AF / ADR / defog / WDR statistics (t31)')
    AE, AE1, AF = ISP + 0xA000, ISP + 0xA800, ISP + 0xB800
    zb, hb, z1, h1 = 0x00600000, 0x00604000, 0x00620000, 0x00624000
    afb, adrb, dfb, wdrb = 0x00640000, 0x00650000, 0x00660000, 0x00680000
    for blk in (AE, AE1):
        qt.writel(blk + 0x04, 0xF001F001)
        for i in range(4):
            qt.writel(blk + 0x08 + 4 * i, 0x40404040)
            qt.writel(blk + 0x18 + 4 * i, 0x24242424)
        qt.writel(blk + 0x28, 0x0011991E)
        qt.writel(blk + 0x4C, 0x33)
        qt.writel(blk, 1)
    for i in range(4):
        qt.writel(AE + 0x2C + 4 * i, zb + 0x1000 * i)
        qt.writel(AE + 0x3C + 4 * i, hb + 0x800 * i)
        qt.writel(AE1 + 0x2C + 4 * i, z1 + 0x1000 * i)
        qt.writel(AE1 + 0x3C + 4 * i, h1 + 0x800 * i)
        qt.writel(AF + 0xA8 + 4 * i, afb + 0x1000 * i)
        qt.writel(ISP + 0x4494 + 4 * i, adrb + 0x1000 * i)
        qt.writel(ISP + 0x5B84 + 4 * i, dfb + 0x1000 * i)
        qt.writel(ISP + 0x2010 + 4 * i, wdrb + 0x2000 * i)
    # tiziano_af_set_hardware_param for 1920 x 1080: 8 x 8 zones
    qt.writel(AF + 0x04, 0x80018003)
    for i in range(4):
        qt.writel(AF + 0x08 + 4 * i, 0xEEEEEEEE)
        qt.writel(AF + 0x18 + 4 * i, 0x86868686)
    for reg in (AF + 0xB8, ISP + 0x4490, ISP + 0x5B80, ISP + 0x2024):
        qt.writel(reg, 3)
    qt.writel(AF, 1)
    qt.writel(ISP + 0x0C, 0xB5742249)     # tisp_init's linear-mode bypasses
    qt.writel(VIC + 0x00, 1)
    qt.writel(ISP + 0xB0, 0xFFFFFFFF)
    qt.writel(ISP + 0xB8, qt.readl(ISP + 0xB4))
    qt.step(40 * 1000 * 1000)
    sta = qt.readl(ISP + 0xB4)
    check('AF (31), ADR (18), defog (20) raised',
          sta & 0x80140000, 0x80140000)
    check('no AE1 (28, 29) or WDR (11) statistics in a linear mode '
          '(0x0C bit 0)', sta & 0x30000800, 0)
    af = qt.read(afb + 0x1000 * qt.readl(AF + 0xBC), 64 * 16)

    def af_zone(n):
        z = af[16 * n:16 * n + 16]
        return [isp_bits(z, 0, 22), isp_bits(z, 22, 22), isp_bits(z, 44, 22),
                isp_bits(z, 66, 22), isp_bits(z, 88, 23)]
    corner, logo = af_zone(0), af_zone(3 * 8 + 3)
    check('AF: a luma sum in every zone', corner[4] > 0 and logo[4] > 0, True)
    check('AF: the logo zone answers the edge filters, the navy corner '
          'hardly', logo[0] > 20 * max(corner[0], 1), True)
    check('AF: the two-pixel filters answer more than the one-pixel ones',
          logo[1] >= logo[0] and logo[3] >= logo[2], True)
    adr_last = qt.readl(ISP + 0x44B0)
    check('ADR: 0x44B0 holds the bank written',
          adr_last in [adrb + 0x1000 * i for i in range(4)], True)
    adr = qt.read(adr_last, 0x1000)
    total = sum(isp_bits(adr[0x308 + 8 * (i // 3):], 21 * (i % 3), 21)
                for i in range(512))
    check('ADR: the 512-bin histogram counts every quad of 960 x 540',
          abs(total - 960 * 540) < 960 * 540 // 50, True)
    check('ADR: a block luma sum', isp_bits(adr, 0, 28) > 0, True)
    df_last = qt.readl(ISP + 0x5BA4)
    df = qt.read(df_last, 0x1000)
    b0 = df[8:24]
    check('defog: the navy block sums more B than R',
          isp_bits(b0, 0, 24) > (isp_bits(b0, 48, 12) |
                                 isp_bits(b0, 112, 12) << 12), True)
    qt.writel(ISP + 0x0C, 0xB5742248)     # a WDR mode: bit 0 clear
    qt.writel(ISP + 0xB8, qt.readl(ISP + 0xB4))
    qt.step(40 * 1000 * 1000)
    check('WDR (11) and AE1 (28, 29) statistics in a WDR mode',
          qt.readl(ISP + 0xB4) & 0x30000800, 0x30000800)
    idx, idx1 = qt.readl(AE + 0x50), qt.readl(AE1 + 0x50)
    check('AE1 zones: the AE0 ones, in its own bank',
          qt.read(z1 + 0x1000 * ((idx1 >> 4) & 3), 225 * 16),
          qt.read(zb + 0x1000 * ((idx >> 4) & 3), 225 * 16))
    w = qt.read(qt.readl(ISP + 0x2680), 0x2000)
    logical = b''.join(w[1024 * i:1024 * i + 516] for i in range(8))
    gsum = sum(isp_bits(logical[16 * i:], 21, 21) for i in range(256))
    check('WDR: the G histogram, packed in 516-byte rows, counts every quad',
          abs(gsum - 960 * 540) < 960 * 540 // 50, True)
    check('WDR: the short exposure the same as the long in a linear frame',
          all(logical[16 * i:16 * i + 8] == logical[16 * i + 8:16 * i + 16]
              for i in range(256)), True)



def i2c_write_reg16(qt, base, addr, reg, val):
    """A register of a 16-bit-addressed sensor (GC4653 / GC4663)"""
    qt.writel(base + 0x6C, 0)
    qt.writel(base + 0x04, addr)
    qt.writel(base + 0x6C, 1)
    qt.writel(base + 0x10, reg >> 8)
    qt.writel(base + 0x10, reg & 0xFF)
    qt.writel(base + 0x10, val | (1 << 9))


def gc4653_expose(qt, lines, short):
    """gc4653_set_integration_time / _short, gain 1.0 (0x02B8:0x02B9)"""
    for reg, val in ((0x0202, lines >> 8), (0x0203, lines & 0xFF),
                     (0x0200, short >> 8), (0x0201, short & 0xFF),
                     (0x02B8, 1), (0x02B9, 0)):
        i2c_write_reg16(qt, I2C0, 0x29, reg, val)


def test_isp_wdr(qt):
    """T31's DOL WDR: AE1 and the WDR statistics of the short frame."""
    print('ingenic-isp WDR, two exposures (t31, sensor=gc4653)')
    AE, AE1 = ISP + 0xA000, ISP + 0xA800
    zb, hb, z1, h1, wdrb = (0x00600000, 0x00604000, 0x00620000, 0x00624000,
                            0x00680000)
    for blk in (AE, AE1):
        qt.writel(blk + 0x04, 0xF001F001)
        for i in range(4):
            qt.writel(blk + 0x08 + 4 * i, 0x20202020)
            qt.writel(blk + 0x18 + 4 * i, 0x12121212)
        qt.writel(blk + 0x28, 0x0011991E)
        qt.writel(blk + 0x4C, 0x33)
    for i in range(4):
        qt.writel(AE + 0x2C + 4 * i, zb + 0x1000 * i)
        qt.writel(AE + 0x3C + 4 * i, hb + 0x800 * i)
        qt.writel(AE1 + 0x2C + 4 * i, z1 + 0x1000 * i)
        qt.writel(AE1 + 0x3C + 4 * i, h1 + 0x800 * i)
        qt.writel(ISP + 0x2010 + 4 * i, wdrb + 0x2000 * i)
    qt.writel(ISP + 0x2024, 3)
    qt.writel(AE, 1)
    qt.writel(VIC + 0x00, 1)
    qt.writel(ISP + 0xB0, 0xFFFFFFFF)
    # a bright light, so that the long frame clips where the short holds
    qt.qom_set('/machine/isp', 'light', 400)
    gc4653_expose(qt, 1000, 125)
    qt.writel(ISP + 0x0C, 0xB5742248)     # tisp_init's WDR: bit 0 clear
    qt.writel(ISP + 0xB8, qt.readl(ISP + 0xB4))
    qt.step(40 * 1000 * 1000)
    check('WDR (11) and AE1 (28, 29) statistics in the WDR mode',
          qt.readl(ISP + 0xB4) & 0x30000800, 0x30000800)
    idx, idx1 = qt.readl(AE + 0x50), qt.readl(AE1 + 0x50)
    ae0 = qt.read(zb + 0x1000 * ((idx >> 4) & 3), 225 * 16)
    ae1 = qt.read(z1 + 0x1000 * ((idx1 >> 4) & 3), 225 * 16)

    def mean(zones, n):
        z = zones[16 * n:16 * n + 16]
        return sum(isp_bits(z, b, 21) for b in (0, 21, 42)) / (32 * 18)
    m0, m1 = mean(ae0, 0), mean(ae1, 0)
    check('AE1: the short frame, an eighth of the long one\'s light in the '
          'navy (%.1f / %.1f)' % (m1, m0), abs(m0 / max(m1, 0.01) - 8) < 1.5,
          True)
    logo0, logo1 = mean(ae0, 7 * 15 + 7), mean(ae1, 7 * 15 + 7)
    check('AE1: the logo the long frame clips is below full scale in the '
          'short one (%.0f / %.0f)' % (logo1, logo0),
          logo1 < logo0 and logo1 < 200, True)
    w = qt.read(qt.readl(ISP + 0x2680), 0x2000)
    logical = b''.join(w[1024 * i:1024 * i + 516] for i in range(8))
    top = [sum(isp_bits(logical[16 * i + 8 * e:], 21, 21)
               for i in range(240, 256)) for e in (0, 1)]
    check('WDR: the long frame\'s G histogram reaches full scale, the short '
          'one\'s hardly (%d, %d)' % tuple(top), top[0] > 10 * max(top[1], 1),
          True)
    ys = [isp_bits(logical[0x1010 + 8 * e:], 0, 32) for e in (0, 1)]
    check('WDR: the short frame\'s luma sum the smaller (%d, %d)' % tuple(ys),
          ys[1] < ys[0], True)
    qt.writel(ISP + 0x0C, 0xB5742249)     # linear again
    qt.writel(ISP + 0xB8, qt.readl(ISP + 0xB4))
    qt.step(40 * 1000 * 1000)
    check('back in a linear mode: no WDR or AE1 statistics',
          qt.readl(ISP + 0xB4) & 0x30000800, 0)


def test_isp_wdr_t41(qt):
    """T41's DOL WDR statistics: 6 KiB banks at 0x6010, bit 11."""
    print('ingenic-isp WDR statistics (t41, sensor=gc4663)')
    AE, vic, wb = ISP + 0x19000, 0x13380000, 0x00680000
    qt.writel(AE + 0x04, 0xF001F001)
    for i in range(4):
        qt.writel(AE + 0x08 + 4 * i, 0x10101010)
        qt.writel(AE + 0x18 + 4 * i, 0x09090909)
        qt.writel(ISP + 0x6010 + 4 * i, wb + 0x1800 * i)
    qt.writel(ISP + 0x6030, 3)
    qt.writel(ISP + 0x80, 480 << 16 | 270)
    qt.writel(vic + 0x00, 1)
    qt.writel(ISP + 0x40020, 0xFFFFFFFF)
    qt.qom_set('/machine/isp', 'light', 400)
    gc4653_expose(qt, 1000, 125)
    qt.writel(ISP + 0x40, 0xFFD00049)     # linear: bit 0 set
    qt.step(41 * 1000 * 1000)
    check('no WDR statistics (11) in a linear mode',
          qt.readl(ISP + 0x40028) & 0x800, 0)
    qt.writel(ISP + 0x40, 0xFF9100E0)     # the WDR mode: bit 0 clear
    qt.writel(ISP + 0x40024, qt.readl(ISP + 0x40028))
    qt.step(41 * 1000 * 1000)
    check('WDR statistics (11) in the WDR mode',
          qt.readl(ISP + 0x40028) & 0x800, 0x800)
    idx = qt.readl(ISP + 0x6030)
    b = qt.read(wb + 0x1800 * idx, 0x1800)
    navy = [isp_bits(b, 24 * (e * 15), 24) for e in (0, 1)]
    check('blocks: the long frame\'s then the short one\'s, 15 a row, '
          'the short darker (%d, %d)' % tuple(navy),
          navy[0] > 4 * max(navy[1], 1), True)
    top = [sum(isp_bits(b[1848 + 2048 * e + 8 * i:], 21, 21)
               for i in range(240, 256)) for e in (0, 1)]
    check('histograms: the long frame clips, the short one hardly (%d, %d)'
          % tuple(top), top[0] > 10 * max(top[1], 1), True)


def test_isp_xstats_t41(qt):
    """T41's other statistics: AF, ADR, defog, GSM, LCE, TMO."""
    print('ingenic-isp AF / ADR / defog / GSM / LCE / TMO statistics (t41)')
    AE, AF, vic = ISP + 0x19000, ISP + 0x1A000, 0x13380000
    afb, adrb, dfb, tmob = 0x00640000, 0x00650000, 0x00660000, 0x00670000
    w, h = 480, 270
    # the AE grid gives the scene's size: 15 x 15 zones of 16 x 9 quads
    qt.writel(AE + 0x04, 0xF001F001)
    for i in range(4):
        qt.writel(AE + 0x08 + 4 * i, 0x10101010)
        qt.writel(AE + 0x18 + 4 * i, 0x09090909)
    qt.writel(ISP + 0x80, w << 16 | h)
    # tisp_af_set_hardware_param's form: 15 x 15, widths as halfwords
    qt.writel(AF + 0x04, 0xF001F003)
    for i in range(8):
        qt.writel(AF + 0x08 + 4 * i, 0x00200020)
    for i in range(4):
        qt.writel(AF + 0x28 + 4 * i, 0x12121212)
    for i in range(4):
        qt.writel(AF + 0xCC + 4 * i, afb + 0x1000 * i)
        qt.writel(ISP + 0x9514 + 4 * i, adrb + 0x1000 * i)
        qt.writel(ISP + 0xC290 + 4 * i, dfb + 0x800 * i)
    for reg in (AF + 0xDC, ISP + 0x9510, ISP + 0xC28C):
        qt.writel(reg, 3)
    qt.writel(ISP + 0x1E010, tmob)
    qt.writel(ISP + 0x1E014, tmob + 0x4000)
    qt.writel(vic + 0x00, 1)
    qt.writel(ISP + 0x40020, 0xFFFFFFFF)
    qt.step(41 * 1000 * 1000)
    sta = qt.readl(ISP + 0x40028)
    check('AF (6), GSM (7), LCE (8), ADR (9), defog (10), TMO (17) raised',
          sta & 0x207C0, 0x207C0)
    af = qt.read(afb + 0x1000 * qt.readl(AF + 0xE0), 15 * 256)

    def af_zone(row, col):
        z = af[256 * row:]                      # 2 * 32 words a row
        return [isp_bits(z, 136 * col + o, n)
                for o, n in ((0, 23), (23, 23), (92, 24))]
    corner, logo = af_zone(0, 0), af_zone(5, 7)
    check('AF: 136-bit zones, a luma sum in each', (corner[2] > 0,
                                                     logo[2] > 0), (True, True))
    check('AF: the logo zone answers the edge filters, the navy corner '
          'hardly', logo[0] > 20 * max(corner[0], 1), True)
    adr = qt.read(qt.readl(ISP + 0x9540), 0x1000)
    total = sum(isp_bits(adr[0x548 + 8 * (i // 3):], 21 * (i % 3), 21)
                for i in range(512))
    check('ADR: the 512-bin histogram from +0x548 counts every quad',
          abs(total - w * h // 4) < w * h // 4 // 50, True)
    check('ADR: 6 x 4 blocks of 56 bytes, a luma sum in the last',
          isp_bits(adr[56 * 23:], 0, 29) > 0, True)
    df = qt.read(qt.readl(ISP + 0xC2B4), 0x800)
    check('defog: 8-byte blocks, the navy one sums more B than R',
          isp_bits(df[8:], 0, 21) > isp_bits(df[8:], 42, 21), True)
    check('GSM: both flags up', (qt.readl(ISP + 0x1B004),
                                 qt.readl(ISP + 0x1B044)), (1, 1))
    gsm = []
    for ctl, data in ((0x501A0, 0x501A8), (0x501B0, 0x501B8)):
        qt.writel(ISP + ctl, 0x201)
        gsm.append([qt.readl(ISP + data) for _ in range(256)])
    check('GSM: the two RAMs add up to width x height',
          sum(gsm[0]) + sum(gsm[1]), w * h)
    qt.writel(ISP + 0x501C0, 0x201)
    lce = [qt.readl(ISP + 0x501C8) for _ in range(32)]
    check('LCE: a block\'s 32 bins count its quads, no more',
          0 < sum(lce) <= w * h // 4 // 45, True)
    tmo = qt.read(tmob + 0x4000 * qt.readl(ISP + 0x1E028), 3750 * 4)
    check('TMO: a 75 x 50 grid of halfword pairs, the mean at most the max',
          all(struct.unpack_from('<H', tmo, 4 * i)[0] <=
              struct.unpack_from('<H', tmo, 4 * i + 2)[0]
              for i in range(0, 3750, 50)), True)

def hldc_calc(k1, k2, w, h):
    """tisp_hldc_calc_para for a plane centred in its middle"""
    cx, cy = w // 2, h // 2
    dx = (cx - 1, w - cx - 1)
    dy = max(cy - 1, h - cy - 1)
    r2max = max(dx) ** 2 + dy ** 2
    inv = 0x3fffffff // r2max
    f = [((r * r) >> 30) * k2 + (256 << 32) - k1 * r
         for r in (r2max * inv, min(dx) ** 2 * inv)]
    return inv, (0x80000 << 32) // min(f) - 5


def test_isp_hldc_t41(qt):
    """T41's HLDC: tisp_hldc_write_reg's registers, 0x40 bit 20 bypass"""
    print('ingenic-isp HLDC (T41)')
    (en, sta, clr, ch0, ch_en, push_y, push_uv, pop_y, empty, size, vic,
     vsta, vclr) = ISP_LAYOUTS['t41']
    w, h, k1, k2 = 256, 144, 200, 40
    ch0 += ISP
    qt.writel(ISP + 0x80, w << 16 | h)          # the sensor's picture
    qt.writel(ch0 + size, w << 16 | h)
    qt.writel(ISP + ch_en, 1)
    qt.writel(vic + 0x00, 1)
    qt.writel(ISP + en, 0xFFFFFFFF)

    def frame(addr):
        qt.writel(ch0 + push_y, addr)
        qt.writel(ch0 + push_uv, addr + w * h)
        qt.step(41 * 1000 * 1000)
        qt.writel(ISP + clr, qt.readl(ISP + sta))
        qt.readl(ch0 + pop_y)
        qt.readl(ch0 + pop_y + 0x18)
        return qt.read(addr, w * h)

    inv, zoom = hldc_calc(k1, k2, w, h)
    uinv, uzoom = hldc_calc(k1, k2, w // 2, h // 2)
    regs = {0x15000: 0x00010001, 0x15004: k2 << 16 | k1,
            0x15008: k2 << 16 | k1, 0x1500C: zoom, 0x15010: uzoom,
            0x15014: inv, 0x15018: uinv, 0x1501C: 0, 0x15020: 0,
            0x15044: 3}
    for r, v in regs.items():
        qt.writel(ISP + r, v)
    qt.writel(ISP + 0x40, 0xffd00049)           # linear, HLDC bypassed
    a = frame(0x00400000)
    qt.writel(ISP + 0x40, 0xffc00049)           # bitBypassHLDC clear
    b = frame(0x00500000)
    # the reading of the registers (docs): output d from the centre shows
    # the picture at d * f(rho) / 256 * zoom / 2048, bilinear
    cx, cy = w // 2, h // 2
    near = far = 0
    n = 0
    for y in range(h // 4, h, 3):               # below the frame counter
        for x in range(0, w, 3):
            dx, dy = x - cx, y - cy
            rho = (dx * dx + dy * dy) * inv / 2.0 ** 30
            g = (256 - k1 * rho / 4 + k2 * rho * rho / 4) / 256 * zoom / 2048
            ux, uy = cx + dx * g, cy + dy * g
            x0, y0 = int(math.floor(ux)), int(math.floor(uy))
            fx, fy = ux - x0, uy - y0
            px = lambda yy, xx: a[min(max(yy, 0), h - 1) * w +
                                  min(max(xx, 0), w - 1)]
            ref = ((px(y0, x0) * (1 - fx) + px(y0, x0 + 1) * fx) * (1 - fy) +
                   (px(y0 + 1, x0) * (1 - fx) + px(y0 + 1, x0 + 1) * fx) * fy)
            near += abs(b[y * w + x] - ref) <= 2
            far += abs(b[y * w + x] - a[y * w + x]) <= 2
            n += 1
    check('HLDC on: the picture remapped (%d%% within 2 of the reading)'
          % (100 * near // n), near >= 0.95 * n, True)
    check('and moved (%d%% unchanged)' % (100 * far // n), far < 0.92 * n,
          True)
    qt.writel(ISP + 0x40, 0xffd00049)
    c = frame(0x00600000)
    same = sum(abs(c[i] - a[i]) <= 2 for i in range(w * h // 4, w * h, 7))
    check('bypassed again: the plain picture', same >= 0.95 * len(
        range(w * h // 4, w * h, 7)), True)


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
    # the AWB's pixel filter, as tiziano_awb_set_hardware_param programs
    # it from the JX-F37 tuning: R/G 100..300, B/G 70..250, R/G + B/G
    # 220..450, luma 50..230; the navy (top left) passes none, a white
    # under daylight (the logo's middle row) passes
    qt.writel(AWB + 0x28, (300 << 16) | 100)
    qt.writel(AWB + 0x2C, (250 << 16) | 70)
    qt.writel(AWB + 0x30, (220 << 16) | 256)
    qt.writel(AWB + 0x34, (450 << 16) | 256)
    qt.writel(AWB + 0x38, (1 << 16) | (230 << 8) | 50)
    qt.qom_set('/machine/isp', 'light', 400)
    qt.step(81 * 1000 * 1000)
    awb = qt.read(ab + 0x1000 * qt.readl(AWB + 0x50), 225 * 16)

    def zone(i):
        z = awb[16 * i:16 * i + 16]
        return [isp_bits(z, b, 21) for b in (0, 21, 42)] + [isp_bits(z, 63, 13)]

    check('filtered: the navy zone counts no quad', zone(0)[3], 0)
    # the darkest zone the filter passes: a grey patch that does not clip
    best = min((i for i in range(225) if zone(i)[3] > 0),
               key=lambda i: zone(i)[1] / zone(i)[3], default=0)
    r, g, b, n = zone(best)
    check('filtered: a grey zone counts some quads', n > 0, True)
    check('its R/G and B/G inside the windows',
          (100 <= 256 * r // g <= 300, 70 <= 256 * b // g <= 250), (True, True))
    # a redder light: the same zone's R/G follows it
    qt.qom_set('/machine/isp', 'light-r', 650)
    qt.step(81 * 1000 * 1000)
    awb = qt.read(ab + 0x1000 * qt.readl(AWB + 0x50), 225 * 16)
    r2, g2, b2, n2 = zone(best)
    check('a light of R 0.65 for 0.55: that zone\'s R/G x 1.18',
          n2 > 0 and abs((r2 / g2) / (r / g) - 650 / 550) < 0.03, True)
    qt.qom_set('/machine/isp', 'light-r', 550)
    qt.qom_set('/machine/isp', 'light', 100)


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
    # a joint picture, main above (ispcore_pad_event_handle, joint mode
    # 3): channel 0's buffer to both pipes, the secondary's part h lines
    # down, its CbCr h / 2 lines down the CbCr plane of the w x 2h picture
    qt.writel(ISP + 0x40024, qt.readl(ISP + 0x40028))
    qt.writel(ISP + 0x40034, qt.readl(ISP + 0x40038))
    j = 0x00700000
    qt.write(j, bytes(w * 2 * h * 3 // 2))
    qt.writel(ISP + 0x16100 + 0x6C, j)
    qt.writel(ISP + 0x16100 + 0x84, j + w * 2 * h)
    qt.writel(ISP + 0x17100 + 0x6C, j + w * h)
    qt.writel(ISP + 0x17100 + 0x84, j + w * 2 * h + w * h // 2)
    # pipe 0 still holds buf[0] from the first setup: two frames
    qt.step(82 * 1000 * 1000)
    top = qt.read(j + 6 * w, 64)
    bottom = qt.read(j + (h + 6) * w, 64)
    check('joint: the main counter (0) in the top part, the secondary\'s '
          '(5) in the bottom one', (top[8] > 128, bottom[8] > 128),
          (True, False))
    check('joint: both parts carry the scene, CbCr included',
          (qt.read(j + 60 * w, w) == qt.read(j + (h + 60) * w, w),
           qt.read(j + 2 * h * w + 30 * w, w) ==
           qt.read(j + 2 * h * w + (h // 2 + 30) * w, w)), (True, True))
    qt.writel(vic0 + 0x00, 0)
    qt.writel(vic1 + 0x00, 0)


def test_isp_t40_mdma(qt):
    """T40's VIC MDMA (snapraw) and VIC2, the third sensor's"""
    print('ingenic-isp (T40 VIC MDMA, VIC2)')
    vic0, vic2 = 0x13380000, 0x133A0000
    w, h = 64, 32
    raw = 0x00700000
    # vic_mdma_enable: size, strides, five buffers, 2 frames, on
    qt.writel(vic0 + 0x308, 1)
    qt.writel(vic0 + 0x304, w << 16 | h)
    qt.writel(vic0 + 0x310, 2 * w)
    qt.writel(vic0 + 0x314, 2 * w)
    for k in range(5):
        qt.writel(vic0 + 0x318 + 4 * k, raw + k * 2 * w * h)
    qt.write(raw, bytes(4 * w * h))
    qt.writel(vic0 + 0x300, 0x80000020 | 2 << 16)
    qt.writel(vic0 + 0x00, 1)
    qt.writel(ISP + 0x40020, 0xFFFFFFFF)
    qt.step(41 * 1000 * 1000)
    check('an MDMA frame: status2 bit 0 (0x1E4)',
          qt.readl(vic0 + 0x1E4) & 1, 1)
    check('0x380 names the buffer written', qt.readl(vic0 + 0x380), raw)
    check('the frame count counts down', (qt.readl(vic0 + 0x300) >> 16) & 15, 1)
    px = struct.unpack('<%dH' % (2 * w), qt.read(raw + 16 * 2 * w, 4 * w))
    # row 16 R G R G.., row 17 G B G B..: the navy background, 10 bits
    r, g, b = px[0], px[1], px[w + 1]
    check('a raw16 RGGB mosaic of the navy scene: B > G > R, 10 bits',
          (b > g > r, max(px) < 1024), (True, True))
    qt.writel(vic0 + 0x1F4, 1)
    qt.step(41 * 1000 * 1000)
    check('the second frame into buffer 1, then the count is out',
          (qt.readl(vic0 + 0x380), (qt.readl(vic0 + 0x300) >> 16) & 15),
          (raw + 2 * w * h, 0))
    qt.writel(vic0 + 0x1F4, 1)
    qt.step(41 * 1000 * 1000)
    check('no third frame', qt.readl(vic0 + 0x1E4) & 1, 0)
    # VIC2: started, its frames end (no frame start: no pipe to feed)
    qt.writel(vic2 + 0x00, 1)
    qt.step(41 * 1000 * 1000)
    check('VIC2 frame done, no frame start',
          qt.readl(vic2 + 0x1E0) & 0x11, 1)
    check('VIC2 interrupts on INTC 28', (qt.readl(CORE_INTC) >> 28) & 1, 1)
    qt.writel(vic0 + 0x00, 0)
    qt.writel(vic2 + 0x00, 0)


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
        # direct_mode: channel 0 queued the IVDC pair (0x80000000 /
        # 0x81000000) writes into the IVDC's store, raises bit 20 for the
        # frame source and leaves the done FIFO empty
        qt.writel(ch0 + push_y, 0x80000000)
        qt.writel(ch0 + push_uv, 0x81000000)
        qt.step(41 * 1000 * 1000)
        check('IVDC: bit 20 and channel 0 done',
              qt.readl(ISP + sta) & (1 << 20 | 1), 1 << 20 | 1)
        check('IVDC: nothing to dequeue', qt.readl(ch0 + empty) & 1, 1)
        check('IVDC: painted into its store',
              36 <= qt.read(0x80000000 + 12 * w, 1)[0] <= 50, True)
        # the IVDC's registers (0x13200000, core INTC 13): its reset
        # self-clears, and each picture into the store is an ISP frame
        # start and frame done, under ivdc_core_ops_init's enable mask
        IVDC = 0x13200000
        qt.writel(IVDC + 0x00, 1)
        check('IVDC: the reset bit reads back 0', qt.readl(IVDC + 0x00), 0)
        check('IVDC: ISP frame start and done (bits 17, 16)',
              qt.readl(IVDC + 0x44) & 0x30000, 0x30000)
        check('IVDC: masked, no interrupt',
              (qt.readl(CORE_INTC + 0x00) >> 13) & 1, 0)
        qt.writel(IVDC + 0x40, 0x244FC0FF)
        check('IVDC: enabled, core INTC 13 pending',
              (qt.readl(CORE_INTC + 0x00) >> 13) & 1, 1)
        qt.writel(IVDC + 0x54, qt.readl(IVDC + 0x44))
        check('IVDC: status cleared through 0x54, the line drops',
              (qt.readl(IVDC + 0x44), (qt.readl(CORE_INTC + 0x00) >> 13) & 1),
              (0, 0))

    if soc == 't23':
        # direct_mode: the IVDC pair queued to channel 0 is painted into
        # the IVDC's store; T23's ISR sends the pair at frame start and on
        # channel 0's done bit pops it to queue it again, so it comes back
        # through the done FIFO (no T41 bit 20)
        qt.writel(ch0 + push_y, 0x80000000)
        qt.writel(ch0 + push_uv, 0x81000000)
        qt.step(41 * 1000 * 1000)
        check('IVDC: frame start and channel 0 done, no bit 20',
              qt.readl(ISP + sta) & (1 << 20 | 0x1000 | 1), 0x1000 | 1)
        check('IVDC: the pair comes back through the done FIFO',
              (qt.readl(ch0 + empty) & 1, qt.readl(ch0 + pop_y)),
              (0, 0x80000000))
        check('IVDC: painted into its store',
              36 <= qt.read(0x80000000 + 12 * w, 1)[0] <= 50, True)
        # the IVDC's registers (0x13200000, INTC 38: tx-isp-t23.ko's
        # resource, Linux IRQ 46)
        IVDC = 0x13200000
        qt.writel(IVDC + 0x00, 1)
        check('IVDC: the reset bit reads back 0', qt.readl(IVDC + 0x00), 0)
        check('IVDC: ISP frame start and done (bits 17, 16)',
              qt.readl(IVDC + 0x44) & 0x30000, 0x30000)
        qt.writel(IVDC + 0x40, 0x30000)
        check('IVDC: enabled, INTC 38 pending',
              (qt.readl(INTC + 0x20) >> 6) & 1, 1)
        qt.writel(IVDC + 0x54, qt.readl(IVDC + 0x44))
        check('IVDC: status cleared, the line drops',
              (qt.readl(IVDC + 0x44), (qt.readl(INTC + 0x20) >> 6) & 1),
              (0, 0))


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


def test_jpgc_decode(qt, soc):
    """HELIX's JPGC decoding a scan on its own (libimp's IMP_Decoder,
    T20_JPEGD_SliceInit's chain): a 16x16 4:2:0 picture of DC-only blocks,
    every table in libimp's format, the picture back as 16x16 tiles"""
    print('ingenic-vpu JPEG decode (%s)' % soc)
    base, irq = {'t23': (0x13100000, 40), 't30': (0x13200000, 62)}[soc]
    taskrg = 0x84 if soc == 't23' else 0x10008
    chain, scan, p0a, p1a = SCRATCH, SCRATCH + 0x10000, SCRATCH + 0x20000, \
        SCRATCH + 0x30000
    # Huffman tables: DC symbols 0 ('00') and 3 ('01'), AC only EOB ('0')
    tabs = {(1, 0): ([1] + [0] * 15, [0]),
            (0, 0): ([0, 2] + [0] * 14, [0, 3]),
            (1, 1): ([1] + [0] * 15, [0]),
            (0, 1): ([0, 2] + [0] * 14, [0, 3])}
    offs = {(1, 0): 0, (0, 0): 162, (1, 1): 174, (0, 1): 162}
    hufm, hufb, hufs = [0] * 16, [0] * 64, [0] * 336
    for g, k in enumerate([(1, 0), (0, 0), (1, 1), (0, 1)]):
        cnt, sym = tabs[k]
        code = idx = top = 0
        a = []
        for l in range(16):
            a.append(code & ((1 << min(l + 1, 8)) - 1))
            hufb[16 * g + l] = (idx - code + offs[k]) & 0x1ff
            code, idx = (code + cnt[l]) << 1, idx + cnt[l]
        for l in range(8):
            top = (top << (l + 1)) | a[l]
        hufm[4 * g:4 * g + 4] = [a[12] << 24 | a[13] << 16 | a[14] << 8 |
                                 a[15], a[8] << 24 | a[9] << 16 | a[10] << 8 |
                                 a[11], top & 0xffffffff, top >> 32]
        for i, v in enumerate(sym):
            hufs[offs[k] + i] |= v if k[0] or not k[1] else v << 4
    # the blocks' DC differences: luma +5, 0, -5, 0; Cb +6; Cr -6; each a
    # DC code, its bits, EOB.  Quantiser 8 everywhere: a pixel is 128 + DC.
    bits = ''
    for d in (5, 0, -5, 0, 6, -6):
        bits += ('01' + format(d if d > 0 else d + 7, '03b') if d else '00') \
            + '0'
    bits += '1' * (-len(bits) % 8)
    data = bytes(int(bits[i:i + 8], 2) for i in range(0, len(bits), 8))
    regs = [(0xE0004, 1)] + \
        [(0xE1300 + 4 * i, v) for i, v in enumerate(hufm)] + \
        [(0xE1200 + 4 * i, v) for i, v in enumerate(hufb)] + \
        [(0xE1800 + 4 * i, v) for i, v in enumerate(hufs)] + \
        [(0xE1400 + 4 * i, 8) for i in range(256)] + \
        [(0xE0008, 0), (0xE000C, scan), (0xE0010, p0a), (0xE0014, p1a),
         (0xE0028, 0), (0xE002C, 0), (0xE0030, 0x30), (0xE0034, 7),
         (0xE0038, 7), (0xE003C, 0xF0), (0xE0004, 0xA0009 | (2 << 4)),
         (0xE0000, 7)]
    words = b''
    for i, (r, v) in enumerate(regs):
        words += struct.pack('<II', v, (1 << 31) | r |
                             ((1 << 30) if i == len(regs) - 1 else 0))
    qt.write(chain, words)
    qt.write(scan, data + b'\xff\xd9')
    qt.write(p0a, bytes(256))
    qt.write(p1a, bytes(128))
    qt.writel(base + 0x00, 1 << 16)                 # SCH_GLBC: INTE_ENDF
    qt.writel(base + taskrg, chain | 1)
    qt.step(2000000)
    check('done: JPGC_STAT end, SCH_STAT end and JPGEND',
          (qt.readl(base + 0xE0008) >> 31, qt.readl(base + 0x34) & 0x11),
          (1, 0x11))
    y, c = qt.read(p0a, 256), qt.read(p1a, 128)
    check('luma blocks in the tile\'s quadrants: 133 133 / 128 128',
          (y[0], y[8], y[128], y[255]), (133, 133, 128, 128))
    check('a luma block is flat', len(set(y[:8])), 1)
    check('chroma rows: 8 Cb (134) then 8 Cr (122)', (c[0], c[7], c[8],
                                                      c[127]), (134, 134,
                                                                122, 122))


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
    check('the chip ID: the SoC ID, then "OpenIPC!"',
          tuple(qt.readl(EFUSE + 0x200 + 4 * i) for i in range(3)),
          (qt.readl(0x1300002C), 0x6E65704F, 0x21435049))
    qt.writel(EFUSE + 0x00, (1 << 21) | 1)      # through CTRL / DATA
    check('chip ID word 1 through DATA', qt.readl(EFUSE + 0x0C), 0x6E65704F)
    # the UARTs' registers past the 16550: UMR, UACR hold what is written
    qt.writel(0x10031024, 16)
    qt.writel(0x10031028, 0x1A5)
    check('UART1 UMR / UACR', (qt.readl(0x10031024), qt.readl(0x10031028)),
          (16, 0x1A5))


def test_rtc_alarm(qt):
    """The RTC's alarm and 1 Hz tick on INTC 32 (T31), as rtc-jz.c sets
    them, the wake latch, and the CPM's power-domain handshake that the
    3.10 suspend path spins on."""
    print('ingenic-rtc alarm, CPM LCR')
    line = lambda: qt.readl(INTC + 0x20) & 1             # source 32
    now = qt.readl(RTC + 0x04)
    qt.writel(RTC + 0x08, now + 3)                       # RTCSAR
    qt.writel(RTC + 0x2C, 1)                             # HWCR.EALM
    qt.writel(RTC + 0x00, 1 | (1 << 2) | (1 << 3))       # RTCE AE AIE
    qt.step(1500 * 1000 * 1000)
    check('no alarm before RTCSAR', (qt.readl(RTC + 0x00) >> 4) & 1, 0)
    qt.step(2500 * 1000 * 1000)
    check('alarm: AF and the line',
          ((qt.readl(RTC + 0x00) >> 4) & 1, line()), (1, 1))
    check('HWRSR.ALM latched for the wake', qt.readl(RTC + 0x30) & 1, 1)
    qt.writel(RTC + 0x00, 1 | (1 << 2) | (1 << 3))       # AF written 0
    check('AF cleared, the line drops', line(), 0)
    qt.writel(RTC + 0x00, 1 | (1 << 5))                  # 1HZIE
    qt.step(1100 * 1000 * 1000)
    check('the 1 Hz tick interrupts', ((qt.readl(RTC + 0x00) >> 6) & 1, line()),
          (1, 1))
    qt.writel(RTC + 0x00, 1)
    lcr = CPM + 0x04
    qt.writel(lcr, qt.readl(lcr) | (0xF << 28))
    check('LCR: domains off report off', (qt.readl(lcr) >> 24) & 0xF, 0xF)
    qt.writel(lcr, qt.readl(lcr) & ~(0xF << 28))
    check('and on again', (qt.readl(lcr) >> 24) & 0xF, 0)


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
        test_tcu_irq(qt)
        test_ost_v1(qt)
        test_intc(qt)
        test_sfc_v1(qt, img)
        test_gpio(qt)
        test_i2c(qt)
        test_rtc_efuse(qt)
        test_sadc(qt)
        test_sadc_dma(qt)
        test_sound(qt)
        test_dmic(qt)
        test_dmic_trigger(qt)
        test_codec_gain(qt)
        test_tcu_pwm(qt)
        test_dtrng(qt)
        test_aes(qt)
        test_hash(qt)
        test_hash_cpu(qt)
        test_rsa(qt)
        test_usbphy(qt)
        qt.close()
        qt = QTest(a.qemu, 't31')
        test_i2s_clock(qt)
        qt.close()
        qt = QTest(a.qemu, 't31,wifi=atbm', ['-nic', 'user'])
        test_sdio(qt)
        qt.close()
        qt = QTest(a.qemu, 't40,wifi=atbm', ['-nic', 'user'])
        test_sdio_sdhci(qt)
        qt.close()
        qt = QTest(a.qemu, 't31', ['-device', 'w25q80,bus=ssi0,cs=0'])
        test_ssi(qt)
        qt.close()
        qt = QTest(a.qemu, 't31')
        test_uart_dma(qt)
        qt.close()
        qt = QTest(a.qemu, 't41', ['-device', 'w25q80,bus=ssi0,cs=0'])
        test_ssi(qt, CORE_INTC)
        qt.close()
        qt = QTest(a.qemu, 't41')
        test_uart45(qt)
        test_sadc(qt, CORE_INTC)
        test_sound(qt, CORE_INTC)
        test_codec_gain_t41(qt)
        test_asdmic(qt)
        test_pwm(qt)
        test_drawbox(qt)
        test_i2d(qt)
        test_aip(qt)
        test_vo(qt)
        test_nndma(qt)
        test_dpu(qt)
        test_dtrng(qt, CORE_INTC)
        test_aes(qt, CORE_INTC)
        test_hash(qt, CORE_INTC)
        test_hash_cpu(qt, CORE_INTC)
        test_rsa(qt)
        test_des(qt)
        qt.close()
        qt = QTest(a.qemu, 't40')
        test_drawbox(qt)
        test_i2d(qt)
        test_monitor(qt)
        test_bscaler(qt)
        test_nndma(qt)
        qt.close()
        qt = QTest(a.qemu, 't20')
        test_aes(qt)
        test_tcu_pwm(qt)
        qt.close()

        qt = QTest(a.qemu, 't31', ['-rtc', 'clock=vm'])
        test_rtc_alarm(qt)
        qt.close()
        qt = QTest(a.qemu, 't30')
        test_dmic(qt, 0x10023000, 0x21, 'T30')
        qt.close()
        qt = QTest(a.qemu, 't20')
        test_dmic(qt, 0x10022000, 0x21, 'T20')
        qt.close()

        qt = QTest(a.qemu, 't31,sensor=gc2053')
        test_isp(qt)
        test_i2c_dma(qt)
        qt.close()
        qt = QTest(a.qemu, 't31,sensor=gc2053')
        test_isp_stats(qt)
        qt.close()
        qt = QTest(a.qemu, 't31,sensor=gc2053')
        test_isp_xstats(qt)
        qt.close()
        qt = QTest(a.qemu, 't41,sensor=gc4663')
        test_isp_xstats_t41(qt)
        qt.close()
        qt = QTest(a.qemu, 't41')
        test_isp_hldc_t41(qt)
        qt.close()
        qt = QTest(a.qemu, 't31,sensor=gc4653')
        test_isp_wdr(qt)
        qt.close()
        qt = QTest(a.qemu, 't41,sensor=gc4663')
        test_isp_wdr_t41(qt)
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
        qt = QTest(a.qemu, 't40,sensor=gc4653,sensor1=gc4653,sensor2=gc4653')
        test_isp_t40_mdma(qt)
        qt.close()
        for soc in ISP_LAYOUTS:
            qt = QTest(a.qemu, soc)
            test_isp_layout(qt, soc)
            qt.close()
        # T23's second sensor (the SDK's gc2053s1) beside the first on I2C0
        qt = QTest(a.qemu, 't23,sensor=gc2053,sensor1=gc2053s1')
        print('t23 sensor1=')
        check('GC2053 ID at I2C0 0x37', i2c_read_reg(qt, I2C0, 0x37, 0xF0, 2),
              [0x20, 0x53])
        check('gc2053s1 ID at I2C0 0x3f', i2c_read_reg(qt, I2C0, 0x3F, 0xF0, 2),
              [0x20, 0x53])
        qt.close()
        qt = QTest(a.qemu, 't20,sensor=jxf22')
        test_isp_apical(qt)
        qt.close()
        qt = QTest(a.qemu, 't10,sensor=jxh62')
        test_isp_apical(qt, 't10')
        qt.close()
        for soc in ('t23', 't30'):
            qt = QTest(a.qemu, soc)
            test_jpgc_decode(qt, soc)
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
        test_migration(a.qemu, 't40,sensor=gc4653,sensor1=gc4653,sensor2=gc4653',
                       tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('%d failure(s)' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
