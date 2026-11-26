#!/usr/bin/env python3
"""Register-level checks of the Anyka AK39xx models, no guest needed.

QEMU runs the ak3918ev300 (and ak3919ev330) machine under qtest and this
script plays the vendor drivers (docs/anyka.md):

  - the mask ROM: a synthetic boot header's register script and payload are
    staged where the ROM puts them (stub + table at 0, U-Boot at its load
    address, the header behind it and in L2 memory at 0x48000200);
  - ak-sysctrl: chip ID, a one-shot timer expiring into the timer's
    sub-interrupt and the cascaded module interrupt, the RTC link, the
    SAR-ADC1 samples from the ain0 / ain1 / bat properties (millivolts):
    EV330's three packed 10-bit fields, EV300's 12-bit conversion on
    demand of a powered, enabled AIN0;
  - ak-uart: a byte put in L2 memory and started from CONF2 reaches the
    serial backend, TX-done reads as idle;
  - ak-spi + ak-spinor: JEDEC ID by PIO, a fast read whose dummy byte is a
    plain TX byte, a 256-byte read streamed through L2 buffer 1 by DMA,
    sector erase and page program written back to the image;
  - ak-gpio pull-ups, the DDRC init-done flag, both I2C layouts against an
    empty bus;
  - the GMAC (xm-dwmac): the PHY over MDIO, and a TX ring of the driver's
    24-byte descriptors (descriptor skip length 2) walked to its end;
  - ak-mci with no card: a command times out, interrupting through MASK,
    and reading STATUS clears it;
  - a full-duplex PIO transfer that fills the RX FIFO loses no TX byte, and
    an L2 read whose DMA ends in a short fraction drains the buffer;
  - ak3918ev300l: an AK_H322L header whose script carries the one-word
    ROM operations, the chip ID, the clock block's self-clearing apply /
    valid bits, SPI0 at 0x21100000 with WORK_MODE, ROM word 0x24 a
    harmless in-ROM pointer;
  - ak3918av100 / ak3918av100x: the ANYKA322 header whose payload the ROM
    copies from flash 0x600 (secure-boot keys behind the 0x200 header), the
    chip ID, and the eFuse read the vendor ak_efuse.ko polls;
  - the ak-sysctrl watchdog: a feed defers expiry, magic close disables it,
    an expiry is reported with when it was last fed, and the kernel's
    two-count restart resets without being reported;
  - ak3918av130's USB, a MUSB host (hcd-musb behind ak-usb) with a
    high-speed 007a:6162 on its port, driven as the vendor's ak_hcd drives
    it: CSRs written a byte at a time, EP0's interrupt disabled and INTRTX
    polled, the device named by FADDR; GET_DESCRIPTOR, SET_ADDRESS with
    its status stage reaching the device, and the descriptor again at the
    new address (OpenIPC/firmware#2962);
  - outside qtest: a flash image smaller than an erase block is refused,
    machine state migrates to a file and back on five- and ten-timer
    parts, and a file-backed flash blocks migration.

    python3 qemu/tests/test-anyka.py [--qemu PATH]
"""
import argparse
import atexit
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

SYSCTRL = 0x08000000
SPI0 = 0x20120000
UART0 = 0x20130000
L2 = 0x20140000
L2MEM = 0x48000000
I2C0 = 0x20150000
GPIO = 0x20170000
DDRC = 0x21000000
DDR = 0x80000000
MAC = 0x20300000
MCI0 = 0x20100000

LOAD = 0x80E00000
JEDEC = 0xC84017                    # GD25Q64, as the SPIP descriptor says


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

    def writeb(self, a, v):
        self.cmd('writeb 0x%x 0x%x' % (a, v))

    def readb(self, a):
        return int(self.cmd('readb 0x%x' % a).split()[1], 16)

    def writew(self, a, v):
        self.cmd('writew 0x%x 0x%x' % (a, v))

    def readw(self, a):
        return int(self.cmd('readw 0x%x' % a).split()[1], 16)

    def step(self, ns):
        self.cmd('clock_step %d' % ns)

    def close(self):
        self.proc.kill()
        self.proc.wait()
        shutil.rmtree(self.dir, ignore_errors=True)


failures = 0


def shown(v):
    if isinstance(v, (bool, str)):
        return v
    if isinstance(v, tuple):
        return '(%s)' % ', '.join(str(shown(x)) for x in v)
    if isinstance(v, bytes):
        return v[:8].hex() + ('...' if len(v) > 8 else '')
    return hex(v)


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what, shown(got)))


def make_image(path, magic=b'ANYKA_H3'):
    """8 MiB: a boot header with a two-pair register script and an SPIP
    descriptor, a two-word "U-Boot" after it, a pattern from 0x10000 on."""
    code = [0xE1A00000, 0xEAFFFFFE]         # nop; b .
    img = bytearray(b'\xFF' * (8 << 20))
    hdr = bytearray(0x200)
    struct.pack_into('<I', hdr, 0, 0xEA00007E)
    hdr[4:12] = magic
    struct.pack_into('<I', hdr, 0x0C, 4 * len(code))
    struct.pack_into('<I', hdr, 0x20, LOAD)
    script = [DDRC + 0x04, 0x0D493315,      # a DDR timing register
              0x66668888, 0xC8,             # a delay, skipped
              DDRC + 0x0C, 0x00000A8B,
              0x88888888, 0x88888888]
    struct.pack_into('<%dI' % len(script), hdr, 0x2C, *script)
    hdr[0x18C:0x190] = b'SPIP'
    struct.pack_into('<I', hdr, 0x190, 0x001740C8)
    img[:0x200] = hdr
    struct.pack_into('<%dI' % len(code), img, 0x200, *code)
    for i in range(0x10000, 0x20000):
        img[i] = i * 7 & 0xFF
    open(path, 'wb').write(img)
    return code


def test_boot_rom(qt, code):
    print('mask ROM')
    stub = [0xE28F2020, 0xE8B20003, 0xE3500000, 0x15801000, 0x1AFFFFFB,
            0xE592F000, 0, 0, 0, 0x7F00]
    check('ROM stub at 0', [qt.readl(4 * i) for i in range(10)] == stub, True)
    table = [qt.readl(0x28 + 4 * i) for i in range(7)]
    check('script table, delay dropped, then {0, 0} and the entry',
          table == [DDRC + 0x04, 0x0D493315, DDRC + 0x0C, 0x00000A8B,
                    0, 0, LOAD], True)
    check('U-Boot word 0 at the load address', qt.readl(LOAD), code[0])
    check('U-Boot word 1', qt.readl(LOAD + 4), code[1])
    check('header in front of U-Boot (TEXT_BASE)', qt.readl(LOAD - 0x200 + 4),
          0x4B594E41)
    check('header left in L2 memory: SPIP at 0x4800038C',
          qt.readl(L2MEM + 0x200 + 0x18C), 0x50495053)


def test_sysctrl(qt):
    print('ak-sysctrl')
    check('chip ID', qt.readl(SYSCTRL), 0x20160100)
    # Timer 1 (CTRL1/2 at 0xB4/0xB8) one-shot, 1 ms; its sub-interrupt is
    # bit 5, the sub-controller is module input 3.
    qt.writel(SYSCTRL + 0x2C, 1 << 5)
    qt.writel(SYSCTRL + 0x24, 1 << 3)
    qt.writel(SYSCTRL + 0xB4, 12000)
    qt.writel(SYSCTRL + 0xB8, (1 << 28) | (1 << 24) | (1 << 29))
    check('not expired yet', (qt.readl(SYSCTRL + 0xB8) >> 27) & 1, 0)
    qt.writel(SYSCTRL + 0xB8, (1 << 28) | (1 << 24) | (1 << 26))
    live = qt.readl(SYSCTRL + 0xB4)
    check('READ_SEL: CTRL1 is the live count', 0 < live <= 12000, True)
    qt.writel(SYSCTRL + 0xB8, (1 << 28) | (1 << 24))
    qt.step(2000000)
    check('expired flag', (qt.readl(SYSCTRL + 0xB8) >> 27) & 1, 1)
    check('timer 1 sub-interrupt', qt.readl(SYSCTRL + 0x30) & (1 << 5),
          1 << 5)
    check('module interrupt 3', qt.readl(SYSCTRL + 0x4C) & (1 << 3), 1 << 3)
    qt.writel(SYSCTRL + 0xB8, (1 << 28) | (1 << 24) | (1 << 30))
    check('CLEAR drops it', qt.readl(SYSCTRL + 0x4C) & (1 << 3), 0)
    # RTC: write register 3, read it back through the serial link.
    qt.writel(SYSCTRL + 0x50, (1 << 21) | (2 << 18) | (3 << 14) | 0x1234)
    qt.writel(SYSCTRL + 0x50, (1 << 21) | (2 << 18) | (1 << 17) | (3 << 14))
    check('RTC register round trip', qt.readl(SYSCTRL + 0x54) & 0x3FFF,
          0x1234)
    check('RTC ready', (qt.readl(SYSCTRL + 0x30) >> 7) & 1, 1)
    # SAR-ADC1 on an EV300: nothing in the sample register until a
    # conversion -- what the EV330 reader got here (0 mV, builder#234) --
    # then a lit room, saturating the 3.0 V reference.
    check('EV300 SAR: no sample without a conversion',
          qt.readl(SYSCTRL + 0x68), 0)
    check('EV300 SAR: AIN0 in a lit room', ev300_sar_ain0(qt),
          (True, 0xFFF))


WDT_CFG1 = SYSCTRL + 0xE4
WDT_CFG2 = SYSCTRL + 0xE8
WDT_HZ = 46875                      # ak_top_wdt.c: 21.333 us a count


def wdt_feed(qt, counts):
    """What ak_top_wdt.c's wdt_keepalive() writes: the load, enable, then
    enable + feed."""
    qt.writel(WDT_CFG1, 0x55 << 24 | counts)
    qt.writel(WDT_CFG2, 0xAA << 24 | 1)
    qt.writel(WDT_CFG2, 0xAA << 24 | 3)


def test_watchdog(qemu, path):
    print('ak-sysctrl watchdog')
    qt = QTest(qemu, 'ak3918ev300,flash-file=%s' % path)
    err = os.path.join(qt.dir, 'qemu.err')

    def log():
        return open(err).read()

    try:
        wdt_feed(qt, WDT_HZ)                    # 1 s
        qt.step(500 * 1000000)
        wdt_feed(qt, WDT_HZ)                    # fed again at 0.5 s
        qt.step(800 * 1000000)
        check('a feed defers expiry', log().count('watchdog expired'), 0)
        qt.step(500 * 1000000)
        check('unfed, it expires and says so',
              log().count('watchdog expired'), 1)
        m = re.search(r'last fed ([0-9.]+) s ago with a ([0-9.]+) s timeout',
                      log())
        check('... with when it was last fed and the timeout',
              bool(m) and abs(float(m.group(1)) - 1) < 0.01 and
              m.group(2) == '1.000', True)
        # Disabled after a feed, as the driver's magic close does.
        wdt_feed(qt, WDT_HZ)
        qt.writel(WDT_CFG2, 0xAA << 24)
        qt.step(2000 * 1000000)
        check('disabled, it stays quiet', log().count('watchdog expired'), 1)
        # The kernel's restart hook: wdt_keepalive(2). A reboot, not an
        # expiry worth reporting.
        wdt_feed(qt, 2)
        qt.step(1000000)
        check('restart (2 counts) is not reported',
              log().count('watchdog expired'), 1)
    finally:
        qt.close()


def test_watchdog_migration(qemu, tmp):
    """An armed watchdog migrated mid-count is still an expiry on the other
    side -- reported, and through -action watchdog= -- not a reboot."""
    print('ak-sysctrl watchdog across migration')
    mon = os.path.join(tmp, 'wdt-mon.sock')
    mig = os.path.join(tmp, 'wdt.mig')
    src = QTest(qemu, 'ak3918ev300',
                ['-monitor', 'unix:%s,server=on,wait=off' % mon])
    try:
        wdt_feed(src, WDT_HZ)                   # 1 s
        src.step(200 * 1000000)
        m = socket.socket(socket.AF_UNIX)
        m.connect(mon)
        m.sendall(('migrate file:%s\n' % mig).encode())
        for _ in range(100):
            m.sendall(b'info migrate\n')
            time.sleep(0.1)
            m.settimeout(0.2)
            try:
                if b'completed' in m.recv(65536):
                    break
            except socket.timeout:
                pass
        m.close()
    finally:
        src.close()
    dst = QTest(qemu, 'ak3918ev300',
                ['-incoming', 'file:' + mig, '-action', 'watchdog=none'])
    try:
        time.sleep(1)
        dst.step(2000 * 1000000)
        log = open(os.path.join(dst.dir, 'qemu.err')).read()
        check('migrated, a run-out is still an expiry',
              log.count('watchdog expired'), 1)
    finally:
        dst.close()


def ev300_sar_ain0(qt, power=True, enable=True):
    """mach-anycloud ak_adc1.c's EV300 sequence (adc1_ev300_init(), then
    adc1_ev300_read() of AIN0): returns (ready, 12-bit sample)."""
    def update(off, mask, val):
        v = qt.readl(SYSCTRL + off)
        qt.writel(SYSCTRL + off, (v & ~mask) | (val & mask))
    update(0x0C, 1 << 3, 0)                     # clock off, divider, on
    update(0x0C, 7, 7)
    update(0x0C, 1 << 3, 1 << 3)
    update(0x20, 1 << 30, 1 << 30)              # out of reset
    update(0x60, 0xFFFFFF, 1500 | 18 << 16)
    update(0x5C, 0x3FFFFF, 1 << 8 | 1 << 14)    # idle
    update(0xA0, 1 | 1 << 3 | 1 << 5, 1)        # powered down, 3.0 V ref
    update(0x6C, 1, 0)
    update(0xA0, 1 | 1 << 5, (0 if power else 1) | (1 << 5 if enable else 0))
    update(0x5C, 7 << 5, 1 << 5)                # AIN0
    update(0x6C, 1, 0)
    update(0x5C, 1, 1)                          # start
    ready = bool(qt.readl(SYSCTRL + 0x6C) & 1)
    sample = qt.readl(SYSCTRL + 0x68) & 0xFFF
    update(0x5C, 1 | 7 << 5, 0)
    update(0xA0, 1 | 1 << 5, 1)
    update(0x6C, 1, 0)
    return ready, sample


def test_sar_inputs(qemu, path, path330):
    print('ak-sysctrl SAR inputs')
    # The properties are millivolts at the pin; a covered photocell is
    # about 39 mV on the PE3020-W (builder#234).
    extra = ['-global', 'ak-sysctrl.ain0=39',
             '-global', 'ak-sysctrl.bat=3300']
    qt = QTest(qemu, 'ak3919ev330,flash-file=%s' % path330, extra)
    try:
        check('EV330: AIN0 covered, AIN1 lit, BAT full, 10 bits of 3.3 V',
              qt.readl(SYSCTRL + 0x68), 12 | 930 << 10 | 0x3ff << 20)
    finally:
        qt.close()
    qt = QTest(qemu, 'ak3918ev300,flash-file=%s' % path, extra)
    try:
        check('EV300: AIN0 covered, 12 bits of 3.0 V', ev300_sar_ain0(qt),
              (True, 39 * 4096 // 3000))
        check('EV300: an input left disabled reads 0',
              ev300_sar_ain0(qt, enable=False), (True, 0))
        check('EV300: powered down, it reads 0',
              ev300_sar_ain0(qt, power=False), (True, 0))
    finally:
        qt.close()


def test_uart(qt, out):
    print('ak-uart')
    qt.writel(L2MEM + 0x1000, ord('K'))
    qt.writel(UART0 + 0x00, 1 << 28)                # clear the TX state
    qt.writel(UART0 + 0x04, (1 << 4) | (1 << 16))   # 1 byte, go
    check('TX done reads as idle', (qt.readl(UART0 + 0x04) >> 19) & 1, 1)
    time.sleep(0.2)
    check('byte reached the backend', open(out, 'rb').read(), b'K')


def spi_tx(qt, data):
    """The drivers' PIO write: ARRM, CNT, then OUT words while half empty."""
    qt.writel(SPI0, 0x272)
    qt.writel(SPI0 + 0x0C, len(data))
    for i in range(0, len(data), 4):
        qt.writel(SPI0 + 0x18, int.from_bytes(data[i:i + 4], 'little'))
    return qt.readl(SPI0 + 0x04) & (1 << 8)


def spi_rx(qt, n):
    qt.writel(SPI0, 0x271)
    qt.writel(SPI0 + 0x0C, n)
    got = b''
    while len(got) < n:
        got += qt.readl(SPI0 + 0x1C).to_bytes(4, 'little')
    return got[:n]


def spi_end(qt):
    qt.writel(SPI0, 0x250)


def test_spi(qt, path):
    print('ak-spi + ak-spinor')
    spi_tx(qt, b'\x9F')
    check('RDID', int.from_bytes(spi_rx(qt, 3), 'big'), JEDEC)
    spi_end(qt)
    img = open(path, 'rb').read()
    check('fast read: TRANSF after the command', spi_tx(
        qt, b'\x0B\x01\x00\x10\x00'), 1 << 8)
    check('fast read, dummy as a TX byte', spi_rx(qt, 12),
          img[0x10010:0x1001C])
    spi_end(qt)

    # 256 bytes through L2 buffer 1: buffer and its DMA on, SPI0 RX
    # (device 7) assigned to it, then the transfer and a 4-block DMA.
    qt.writel(L2 + 0x88, (1 << 1) | (1 << 17))
    qt.writel(L2 + 0x90, 1 << 21)
    qt.writel(L2 + 0x88, (1 << 1) | (1 << 17) | (1 << 25))
    spi_tx(qt, b'\x03\x01\x80\x00')
    qt.writel(SPI0, 0x271)
    qt.writel(SPI0 + 0x14, 0x10001)
    qt.writel(SPI0 + 0x0C, 256)
    check('L2 buffer full, SPI stalled', (qt.readl(L2 + 0xA0) >> 4) & 0xF,
          4)
    qt.writel(L2 + 0x04, DDR + 0x100000)
    qt.writel(L2 + 0x44, 4)
    qt.writel(L2 + 0x80, (1 << 25) | 1)
    check('DMA done', qt.readl(L2 + 0x80) & (1 << 25), 0)
    check('SPI count drained', qt.readl(SPI0 + 0x0C), 0)
    got = b''.join(qt.readl(DDR + 0x100000 + i).to_bytes(4, 'little')
                   for i in range(0, 256, 4))
    check('256 bytes via L2 DMA', got, img[0x18000:0x18100])
    qt.writel(SPI0 + 0x14, 0)
    qt.writel(L2 + 0x88, 0)
    spi_end(qt)

    # Erase the 4 KiB sector at 0x11000, program 4 bytes into it.
    spi_tx(qt, b'\x06')
    spi_end(qt)
    spi_tx(qt, b'\x20\x01\x10\x00')
    spi_end(qt)
    spi_tx(qt, b'\x06')
    spi_end(qt)
    spi_tx(qt, b'\x02\x01\x10\x00' + b'\x11\x22\x33\x44')
    spi_end(qt)
    spi_tx(qt, b'\x05')
    check('WEL cleared after the program', spi_rx(qt, 1)[0] & 2, 0)
    spi_end(qt)
    img = open(path, 'rb').read()
    check('programmed bytes in the image file', img[0x11000:0x11004],
          b'\x11\x22\x33\x44')
    check('rest of the sector erased', img[0x11004:0x12000],
          b'\xFF' * 0xFFC)
    check('next sector untouched', img[0x12000:0x12004],
          bytes(i * 7 & 0xFF for i in range(0x12000, 0x12004)))


def test_spi_more(qt, path):
    print('ak-spi: duplex with a full RX FIFO, L2 fraction tail')
    img = open(path, 'rb').read()
    # RDID shifted out with 11 filler bytes, both directions, OUT written
    # three words ahead of IN: the third waits for FIFO room, intact.
    qt.writel(SPI0, 0x270)
    qt.writel(SPI0 + 0x0C, 12)
    for w in (0x9F, 0, 0):
        qt.writel(SPI0 + 0x18, w)
    got = b''.join(qt.readl(SPI0 + 0x1C).to_bytes(4, 'little')
                   for _ in range(3))
    check('12 bytes in step with the 12 sent', got,
          b'\xFF' + bytes.fromhex('c84017') * 3 + bytes.fromhex('c840'))
    spi_end(qt)

    # 100 bytes: one 64-byte block by DMA, the 36-byte tail by fraction
    # DMA out of block 1 of buffer 1.
    dst = DDR + 0x300000
    qt.writel(L2 + 0x88, (1 << 1) | (1 << 17))
    qt.writel(L2 + 0x90, 1 << 21)
    qt.writel(L2 + 0x88, (1 << 1) | (1 << 17) | (1 << 25))
    spi_tx(qt, b'\x03\x01\x90\x00')
    qt.writel(SPI0, 0x271)
    qt.writel(SPI0 + 0x14, 0x10001)
    qt.writel(SPI0 + 0x0C, 100)
    qt.writel(L2 + 0x04, dst)
    qt.writel(L2 + 0x44, 1)
    qt.writel(L2 + 0x80, (1 << 25) | 1)
    qt.writel(L2 + 0x84, (3 << 28) | ((dst + 64) & 0x0FFFFFFF))
    qt.writel(L2 + 0x80, 1 | (1 << 9) | (35 << 10) | (((1 << 3) | 1) << 1))
    check('fraction DMA done', qt.readl(L2 + 0x80) & (1 << 9), 0)
    check('buffer drained', (qt.readl(L2 + 0xA0) >> 4) & 0xF, 0)
    got = b''.join(qt.readl(dst + i).to_bytes(4, 'little')
                   for i in range(0, 100, 4))
    check('100 bytes via block + fraction DMA', got, img[0x19000:0x19064])
    qt.writel(SPI0 + 0x14, 0)
    qt.writel(L2 + 0x88, 0)
    spi_end(qt)


def hmp(qemu, args, cmds):
    """Run QEMU with an HMP monitor on stdio; return what it printed."""
    p = subprocess.run([qemu] + args + ['-display', 'none', '-nodefaults',
                                        '-monitor', 'stdio'],
                       input='\n'.join(cmds + ['quit']) + '\n',
                       capture_output=True, text=True, timeout=60)
    return p.stdout + p.stderr


def test_rejections_and_migration(qemu, tmp, path):
    print('flash size, migration')
    small = os.path.join(tmp, 'small.bin')
    open(small, 'wb').write(b'\xFF' * 0x8000)
    out = subprocess.run([qemu, '-M', 'ak3918ev300,flash-file=' + small,
                          '-display', 'none', '-nodefaults', '-S'],
                         capture_output=True, text=True, timeout=30)
    check('a 32 KiB flash image is refused',
          out.returncode != 0 and '64 KiB' in out.stderr, True)
    for m in ('ak3918ev300', 'ak3919ev330', 'ak3918av130'):
        mig = os.path.join(tmp, m + '.mig')
        out = hmp(qemu, ['-M', m, '-S'],
                  ['migrate file:' + mig, 'info migrate'])
        check('%s: state saved' % m, 'completed' in out and
              'Assertion' not in out and os.path.getsize(mig) > 0, True)
        out = hmp(qemu, ['-M', m, '-S', '-incoming', 'file:' + mig],
                  ['info status'])
        check('%s: and loaded back' % m, 'error' not in out.lower() and
              'Assertion' not in out, True)
    out = hmp(qemu, ['-M', 'ak3918ev300,flash-file=' + path, '-S'],
              ['migrate file:' + os.path.join(tmp, 'blocked.mig')])
    check('file-backed flash blocks migration', 'written through' in out,
          True)


def test_misc(qt):
    print('ak-gpio, ak-ddrc, ak-i2c')
    check('GPIO inputs pulled up', qt.readl(GPIO + 0x18), 0xFFFFFFFF)
    qt.writel(GPIO + 0x00, 1 << 5)
    qt.writel(GPIO + 0x0C, 0)
    check('an output pin reads its level', qt.readl(GPIO + 0x18),
          0xFFFFFFDF)
    check('DDR init done', qt.readl(DDRC + 0x24) & 6, 2)
    # Read 2 bytes from 0x30, which is not there.
    qt.writel(I2C0 + 0x10, (1 << 18) | (1 << 17) | (0x30 << 1) | 1)
    qt.writel(I2C0, (1 << 14) | (1 << 15) | (1 << 13) | (1 << 9))
    check('I2C done', qt.readl(I2C0) & (1 << 4), 1 << 4)
    check('absent device reads 0xFF', qt.readl(I2C0 + 0x20) & 0xFFFF,
          0xFFFF)


def mdio_read(qt, reg):
    qt.writel(MAC + 0x10, (1 << 11) | (reg << 6) | 1)
    return qt.readl(MAC + 0x14)


def test_mac(qt):
    print('GMAC (xm-dwmac)')
    check('PHY ID1', mdio_read(qt, 2), 0x0007)
    check('BMSR: link up, autonegotiation done',
          mdio_read(qt, 1) & 0x24, 0x24)
    check('ANER: partner negotiates', mdio_read(qt, 6) & 1, 1)
    # Two TX descriptors, 4 words + 2 software words each, the second
    # ending the ring; a 60-byte frame in each.
    ring = DDR + 0x200000
    for i in range(2):
        d = ring + 24 * i
        qt.writel(d, 1 << 31)
        qt.writel(d + 4, (1 << 29) | (1 << 30) | (1 << 31) | 60 |
                  ((1 << 25) if i else 0))
        qt.writel(d + 8, DDR + 0x210000)
        qt.writel(d + 12, 0)
    qt.writel(MAC + 0x1000, 2 << 2)                 # DSL = 2
    qt.writel(MAC + 0x1010, ring)
    qt.writel(MAC + 0x00, 1 << 3)                   # TE
    qt.writel(MAC + 0x1018, 1 << 13)                # ST
    check('descriptor 0 handed back', qt.readl(ring) >> 31, 0)
    check('descriptor 1 (24 bytes on) handed back', qt.readl(ring + 24) >> 31,
          0)
    check('back at the ring base', qt.readl(MAC + 0x1048), ring)
    check('TX interrupt status', qt.readl(MAC + 0x1014) & 1, 1)


def test_mci(qt):
    print('ak-mci, no card')
    qt.writel(MCI0 + 0x38, 1 << 2)                  # RESPTIMEOUT
    qt.writel(MCI0 + 0x08, 0x1AA)
    qt.writel(MCI0 + 0x0C, 1 | (8 << 1) | (1 << 7)) # CMD8, short response
    check('module interrupt 4 raised', qt.readl(SYSCTRL + 0x4C) & (1 << 4),
          1 << 4)
    check('response timeout', qt.readl(MCI0 + 0x34) & 0x14, 1 << 2)
    check('STATUS read cleared it', qt.readl(MCI0 + 0x34) & 0x14, 0)
    check('and the interrupt', qt.readl(SYSCTRL + 0x4C) & (1 << 4), 0)


def test_ev330(qemu, path):
    print('ak3919ev330: I2C v2, eFuse')
    qt = QTest(qemu, 'ak3919ev330,flash-file=%s' % path)
    try:
        check('chip ID', qt.readl(SYSCTRL), 0x20160101)
        qt.writel(SYSCTRL + 0x8C, 0xA0000000)
        check('eFuse read done', qt.readl(SYSCTRL + 0x8C) >> 30, 0)
        qt.writel(I2C0 + 0x08, (0x30 << 1) | 1 | 0x300)
        qt.writel(I2C0, (2 << 17) | (7 << 22) | (3 << 15) | (1 << 26))
        st = qt.readl(I2C0)
        check('I2C v2 done', st & (1 << 13), 1 << 13)
        check('I2C v2 NAK from an absent device', st & (1 << 27), 1 << 27)
    finally:
        qt.close()


SPI0_ASPEN = 0x21100000
UART2_ASPEN = 0x201A0000


def test_ev300l(qemu, path):
    print('ak3918ev300l: AK_H322L header, clocks, SPI0 at 0x21100000')
    img = bytearray(open(path, 'rb').read())
    script = [DDRC + 0x14, 0x00423E71,
              0x66668881, DDRC + 0xA4, 0x10000,     # poll: three words
              DDRC + 0x0C, 0x00001A5D,
              0x66668882, 0x54535150,               # a one-word ROM op
              0x66668880, DDRC + 0x24, 0xFFFFFFFB,  # RMW: three words
              0x88888888, 0]
    img[0x2C:0x18C] = b'\0' * 0x160
    struct.pack_into('<%dI' % len(script), img, 0x2C, *script)
    img[0x190:0x194] = b'\0' * 4                   # SPIP with no ID
    open(path, 'wb').write(img)
    out2 = path + '.uart2'
    qt = QTest(qemu, 'ak3918ev300l,flash-file=%s' % path,
               ['-serial', 'null', '-serial', 'null',
                '-serial', 'file:' + out2])
    try:
        table = [qt.readl(0x28 + 4 * i) for i in range(7)]
        check('script: poll and ROM ops dropped', table ==
              [DDRC + 0x14, 0x00423E71, DDRC + 0x0C, 0x00001A5D,
               0, 0, LOAD], True)
        check('ROM word 0x24 points into the ROM', qt.readl(0x24), 0x7F00)
        check('chip ID', qt.readl(SYSCTRL), 0x53543400)
        qt.writel(SYSCTRL + 0x04, 0x40000024)
        check('PLL1 adjust bit self-clears', qt.readl(SYSCTRL + 0x04),
              0x24)
        qt.writel(SYSCTRL + 0x08, 0x02280063)
        check('GCLK divider valid bit self-clears',
              qt.readl(SYSCTRL + 0x08), 0x00280063)
        qt.writel(SYSCTRL + 0x0C, (1 << 19) | (1 << 18) | (2 << 10))
        check('SPI0 HS clock divider valid bit self-clears',
              qt.readl(SYSCTRL + 0x0C), (1 << 18) | (2 << 10))
        qt.writel(SYSCTRL + 0x264, 0x1234)
        check('pad registers past 0x200 are storage',
              qt.readl(SYSCTRL + 0x264), 0x1234)
        s = SPI0_ASPEN
        qt.writel(s + 0x24, qt.readl(s + 0x24) | 1)
        check('WORK_MODE normal mode reads back', qt.readl(s + 0x24) & 1, 1)
        qt.writel(s, 0x72)
        qt.writel(s + 0x0C, 1)
        qt.writel(s + 0x18, 0x9F)
        qt.writel(s, 0x71)
        qt.writel(s + 0x0C, 3)
        jedec = qt.readl(s + 0x1C) & 0xFFFFFF
        qt.writel(s, 0x51)
        check('RDID through SPI0 at 0x21100000 (empty SPIP: GD25Q64)',
              int.from_bytes(jedec.to_bytes(3, 'little'), 'big'), 0xC84017)
        # UART 2 at 0x201A0000, its TX ring at L2 + 0x1200
        qt.writel(L2MEM + 0x1200, ord('2'))
        qt.writel(UART2_ASPEN + 0x00, 1 << 28)
        qt.writel(UART2_ASPEN + 0x04, (1 << 4) | (1 << 16))
        check('UART2: TX done reads as idle',
              (qt.readl(UART2_ASPEN + 0x04) >> 19) & 1, 1)
        time.sleep(0.2)
        check('UART2: byte reached the third serial port',
              open(out2, 'rb').read(), b'2')
    finally:
        qt.close()
def test_av100(qemu, tmp):
    print('ak3918av100 / ak3918av100x: ANYKA322 header, eFuse')
    # The AV100 keeps 0x400 bytes for secure boot behind the 0x200 header
    # (pubkey 0x200, signature 0x300). The ROM copies from flash 0x200, as
    # on the other chips; the vendor's header covers the secure-boot space
    # (load 0x80DFFC00), so U-Boot at 0x600 lands at 0x80E00000.
    code = [0xE1A00000, 0xEAFFFFFE]
    img = bytearray(b'\xFF' * (8 << 20))
    struct.pack_into('<I', img, 0, 0xEA00017E)
    img[4:12] = b'ANYKA322'
    struct.pack_into('<I', img, 0x0C, 0x400 + 4 * len(code))
    struct.pack_into('<I', img, 0x20, LOAD - 0x400)
    script = [DDRC + 0x14, 0x00423E71,
              0x66668881, DDRC + 0xA4, 0x10000,
              0x88888888, 0]
    struct.pack_into('<%dI' % len(script), img, 0x2C, *script)
    img[0x200:0x600] = bytes(0x400)    # unsigned: no key, no signature
    struct.pack_into('<%dI' % len(code), img, 0x600, *code)
    path = os.path.join(tmp, 'av100.bin')
    open(path, 'wb').write(img)

    for machine, ram, chip in (('ak3918av100', 64, 0x53533500),
                               ('ak3918av100x', 128, 0x53533500)):
        qt = QTest(qemu, '%s,flash-file=%s' % (machine, path))
        try:
            check('%s chip ID' % machine, qt.readl(SYSCTRL), chip)
            check('%s: flash 0x600 at 0x80E00000' % machine,
                  qt.readl(LOAD), code[0])
            check('%s: secure-boot space copied in front of it' % machine,
                  qt.readl(LOAD - 0x400), 0)
            check('%s: header in front of that (TEXT_BASE)' % machine,
                  qt.readl(LOAD - 0x600 + 4), 0x4B594E41)
            # DDR size is the one difference between the two: the last word
            # of RAM holds a value, the word past it does not.
            end = 0x80000000 + (ram << 20)
            qt.writel(end - 4, 0x5AA5C33C)
            check('%s: RAM up to %d MiB' % (machine, ram),
                  qt.readl(end - 4), 0x5AA5C33C)
            qt.writel(end, 0x5AA5C33C)
            check('%s: no RAM past %d MiB' % (machine, ram),
                  qt.readl(end) != 0x5AA5C33C, True)
            # eFuse read: bit 31 start/busy, cleared when done (the AV100's
            # ak_efuse.ko polls SYSCTRL + 0x8C the way the EV330 reader does).
            qt.writel(SYSCTRL + 0x8C, (1 << 31) | (1 << 21))
            check('%s: eFuse read completes' % machine,
                  qt.readl(SYSCTRL + 0x8C) & (3 << 30), 0)
        finally:
            qt.close()


def test_av130(qemu, tmp):
    print('ak3918av130: an AKH322LS header as the first AV130 dump has it, '
          'timers and interrupts at 0x21100000, FIFO UART')
    # As owner report r-5uwdebux's flash has it: magic AKH322LS, the payload
    # from flash 0x200 (0x400 of secure-boot space, then U-Boot at 0x600) to
    # 0x80DFFC00, so U-Boot lands at 0x80E00000; the script ends with the
    # three-word read-modify-write that a pair-wise reading made a store to
    # 0xFFFFFFFB.
    code = [0xE1A00000, 0xEAFFFFFE]
    img = bytearray(b'\xFF' * (8 << 20))
    struct.pack_into('<I', img, 0, 0xEA000238)
    img[4:12] = b'AKH322LS'
    struct.pack_into('<I', img, 0x0C, 0x400 + 4 * len(code))
    struct.pack_into('<I', img, 0x20, LOAD - 0x400)
    script = [DDRC + 0x14, 0x00423E71,
              0x66668881, DDRC + 0xA4, 0x10000,
              DDRC + 0x24, 0x1,
              0x66668882, 0x54535150,
              0x66668880, DDRC + 0x24, 0xFFFFFFFB,
              0x88888888, 0]
    struct.pack_into('<%dI' % len(script), img, 0x2C, *script)
    img[0x200:0x600] = b'\0' * 0x400
    struct.pack_into('<%dI' % len(code), img, 0x600, *code)
    for i in range(0x10000, 0x10100):          # what the SFC reads back
        img[i] = i * 7 & 0xFF
    path = os.path.join(tmp, 'av130.bin')
    open(path, 'wb').write(img)
    out = os.path.join(tmp, 'av130-uart0.txt')
    rx = os.path.join(tmp, 'av130-uart0-in.txt')
    open(rx, 'wb').write(b'hi')                 # what UART0 receives
    qt = QTest(qemu, 'ak3918av130,flash-file=%s' % path,
               ['-chardev', 'file,id=u0,path=%s,input-path=%s' % (out, rx),
                '-serial', 'chardev:u0'])
    try:
        table = [qt.readl(0x28 + 4 * i) for i in range(9)]
        check('script: poll, ROM op and RMW dropped, nothing at 0xFFFFFFFB',
              table[:7] == [DDRC + 0x14, 0x00423E71, DDRC + 0x24, 0x1,
                            0, 0, LOAD - 0x400], True)
        check('U-Boot lands at 0x80E00000', qt.readl(LOAD), code[0])
        qt.writel(0x80000000 + (64 << 20) - 4, 0x5AA5C33C)
        check('64 MiB of RAM', qt.readl(0x80000000 + (64 << 20) - 4),
              0x5AA5C33C)
        qt.writel(0x80000000 + (64 << 20), 0x5AA5C33C)
        check('none past 64 MiB', qt.readl(0x80000000 + (64 << 20)) !=
              0x5AA5C33C, True)
        # The timers and interrupt controller at 0x21100000, as the AV130
        # SDK's kernel drives them (ak_timer_common.h, ak_irq_v2.c) and its
        # U-Boot's get_timer() reads timer 0: LOAD 0x00, VALUE 0x04, CTRL
        # 0x08 (0 enable, 2:1 mode, 3 reload now), INTCLR 0x0C, INT_EN 0x10.
        intc, t0, t1 = 0x21100000, 0x21110000, 0x21111000
        qt.writel(t0 + 0x00, 12000)             # U-Boot's timer_init()
        qt.writel(t0 + 0x14, 1)
        qt.writel(t0 + 0x10, 1)
        qt.writel(t0 + 0x08, 1)                 # free running
        a = qt.readl(t0 + 0x04)
        qt.step(250000)                         # 250 us at 12 MHz
        qt.writel(t0 + 0x08, 1)                 # get_timer() sets bit 0 again
        b = qt.readl(t0 + 0x04)
        check('timer 0 counts down 3000 in 250 us', (a - b) & 0xFFFFFFFF,
              3000)
        check('CTRL bit 0 is the enable: it stays set',
              qt.readl(t0 + 0x08) & 1, 1)
        qt.writel(t0 + 0x08, (1 << 3) | 1)       # the kernel's clocksource
        check('reload: VALUE starts again from LOAD',
              qt.readl(t0 + 0x04), 12000)
        qt.writel(intc + 0x04, 1 << 28)          # IRQ mask: timer 1
        qt.writel(t1 + 0x00, 12000)
        qt.writel(t1 + 0x10, 1)
        qt.writel(t1 + 0x08, (2 << 1) | (1 << 3) | 1)  # one-shot clock event
        qt.step(500000)
        check('timer 1 one-shot: not before it expires',
              qt.readl(t1 + 0x18), 0)
        qt.step(600000)
        check('timer 1 one-shot: INT_STA', qt.readl(t1 + 0x18), 1)
        check('top status bit 28 is timer 1', qt.readl(intc + 0x08) >> 28 & 1,
              1)
        qt.writel(t1 + 0x0C, 1)
        check('INTCLR clears it', qt.readl(intc + 0x08) >> 28 & 1, 0)
        qt.writel(intc + 0x0C, 1)                # TWI bank mask: I2C 0
        check('an idle TWI bank raises no cascade bit (13)',
              qt.readl(intc + 0x08) >> 13 & 1, 0)
        check('chip ID at SYSCTRL + 0x3FC', qt.readl(SYSCTRL + 0x3FC),
              0x56573500)
        # Periodic: VALUE runs LOAD..0, a period of LOAD + 1 ticks, and
        # INT_STA comes as it reaches 0, not a tick before.
        t2 = 0x21112000
        qt.writel(t2 + 0x00, 1199)               # 100 us at 12 MHz
        qt.writel(t2 + 0x10, 1)
        qt.writel(t2 + 0x08, (1 << 1) | (1 << 3) | 1)
        qt.step(99916)                           # tick 1198
        check('periodic: not before VALUE reaches 0',
              (qt.readl(t2 + 0x18), qt.readl(t2 + 0x04)), (0, 1))
        qt.step(1)
        check('periodic: INT_STA as VALUE reaches 0',
              (qt.readl(t2 + 0x18), qt.readl(t2 + 0x04)), (1, 0))
        qt.writel(t2 + 0x0C, 1)
        qt.step(100000)                          # one period, LOAD + 1 ticks
        check('periodic: the next one LOAD + 1 ticks later',
              (qt.readl(t2 + 0x18), qt.readl(t2 + 0x04)), (1, 0))
        qt.writel(t2 + 0x0C, 1)
        qt.writel(t2 + 0x00, 599)                # a new LOAD, at VALUE 0
        qt.step(49999)
        check('new LOAD: not before its period ends', qt.readl(t2 + 0x18), 0)
        qt.step(1)
        check('new LOAD: INT_STA after LOAD + 1 = 600 ticks',
              (qt.readl(t2 + 0x18), qt.readl(t2 + 0x04)), (1, 0))
        qt.writel(t2 + 0x08, 0)
        qt.writel(t2 + 0x0C, 1)
        # The system controller's own events are bank 1: their mask and
        # status stay at SYSCTRL + 0x2C / 0x30, and top bit 3 raises them.
        qt.writel(intc + 0x04, 1 << 3)
        qt.writel(SYSCTRL + 0x2C, 1 << 5)        # its timer 1
        qt.writel(SYSCTRL + 0xB4, 12000)
        qt.writel(SYSCTRL + 0xB8, (1 << 28) | (1 << 24) | (1 << 29))
        qt.writel(SYSCTRL + 0xB8, (1 << 28) | (1 << 24))
        qt.step(2000000)
        check('a system controller timer reaches top bit 3',
              qt.readl(intc + 0x08) >> 3 & 1, 1)
        qt.writel(SYSCTRL + 0xB8, (1 << 28) | (1 << 24) | (1 << 30))
        check('and clearing it there drops it', qt.readl(intc + 0x08) >> 3 & 1,
              0)
        # The kernel's reboot (ak_pm.c REBOOT_DISABLE_CACHE_MMU and
        # REBOOT_SELF_REFRESH): the instruction after the MMU goes off is
        # fetched from L2 memory at its virtual address, 0xF1000000, and the
        # DRAM's self-refresh request (0x21000010 bit 23) reads back done.
        qt.writel(0x48000040, 0xE1A0F001)        # mov pc, r1
        check('L2 memory answers at the kernel\'s AK_VA_L2MEM',
              qt.readl(0xF1000040), 0xE1A0F001)
        qt.writel(0x21000010, (1 << 26) | (1 << 23) | (1 << 22))
        check('DDRC: the self-refresh request (bit 23) reads back done',
              qt.readl(0x21000010), (1 << 26) | (1 << 22))
        uart = 0x20130000
        check('UART: TX FIFO never full', qt.readl(uart + 0x18) & 4, 0)
        for ch in b'OK':
            qt.writel(uart + 0x0C, ch << 16)
        check('UART: bits 23:16 of DATA go out', open(out, 'rb').read(),
              b'OK')
        check('UART: TX FIFO empty and TX end after a send',
              qt.readl(uart + 0x18) & ((1 << 1) | (1 << 17)),
              (1 << 1) | (1 << 17))
        qt.writel(uart + 0x18, 1 << 17)
        check('UART: TX end is write-1-to-clear',
              qt.readl(uart + 0x18) >> 17 & 1, 0)
        test_av130_sfc(qt)
        # RX: two bytes from the input file, their events on UART0's
        # interrupt (top bit 10) only while bytes wait.
        qt.writel(intc + 0x04, 1 << 10)
        qt.writel(uart + 0x20, (1 << 8) | (1 << 10))
        check('UART: two bytes received', (qt.readl(uart + 0x38) >> 16) & 0x3FF,
              2)
        check('UART: RX threshold and timeout raised',
              (qt.readl(uart + 0x18) >> 8) & 5, 5)
        check('UART: on interrupt 10', qt.readl(intc + 0x08) >> 10 & 1, 1)
        got = bytes(qt.readl(uart + 0x0C) & 0xFF for _ in range(2))
        check('UART: the bytes, in order', got, b'hi')
        check('UART: the RX events end with the FIFO',
              ((qt.readl(uart + 0x18) >> 8) & 5, qt.readl(intc + 0x08) >> 10 & 1),
              (0, 0))
        qt.writel(uart + 0x00, (1 << 28) | (1 << 25))
        check('UART: the RX FIFO clear (CFG 28) self-clears',
              qt.readl(uart + 0x00), 1 << 25)
    finally:
        qt.close()


SFC = 0x20120000
SFC_XIP = 0x30000000


USB = 0x20200000
AV130_INTC = 0x21100000


def usb_ep0(qt, setup=None, csr0=None):
    """One EP0 stage as ak_hcd runs it; returns INTRTX and CSR0 after."""
    qt.writeb(USB + 0x0E, 0)                    # INDEX
    if setup is not None:
        for b in setup:
            qt.writeb(USB + 0x20, b)            # the EP0 FIFO
    qt.writeb(USB + 0x12, csr0)                 # CSR0, a byte write
    qt.step(1000000)
    return qt.readw(USB + 0x02), qt.readb(USB + 0x12)


def usb_get_device_descriptor(qt):
    tx, csr = usb_ep0(qt, [0x80, 6, 0, 1, 0, 0, 0x40, 0], 0x0A)
    ok = tx == 1 and not csr & 0x02             # SETUPPKT | TXPKTRDY sent
    tx, csr = usb_ep0(qt, csr0=0x20)            # REQPKT
    n = qt.readw(USB + 0x18)                    # COUNT0
    data = bytes(qt.readb(USB + 0x20) for _ in range(n))
    ok = ok and tx == 1 and csr & 0x01          # RXPKTRDY
    usb_ep0(qt, csr0=0x00)                      # RXPKTRDY cleared
    tx, _ = usb_ep0(qt, csr0=0x42)              # STATUSPKT | TXPKTRDY
    return ok and tx == 1, data


def test_av130_usb(qemu):
    print('ak3918av130: USB host, a high-speed 007a:6162 enumerated over '
          'EP0 as ak_hcd does it')
    qt = QTest(qemu, 'ak3918av130',
               ['-device', 'atbm-wifi,pid=0x6162,port=1'])
    try:
        qt.writeb(USB + 0x01, 0x21)             # POWER: HSENAB | SUSPENDM
        qt.writeb(USB + 0x0B, 0xF7)             # INTRUSBE, but SOF
        qt.writew(USB + 0x06, 0)                # INTRTXE: EP0 polled
        qt.writew(USB + 0x08, 0)                # INTRRXE
        qt.writeb(USB + 0x60, 1)                # DEVCTL: session
        check('connect on interrupt 18 (the DT\'s first)',
              qt.readl(AV130_INTC + 0x08) >> 18 & 1, 1)
        check('INTRUSB: connect, cleared by the read',
              (qt.readb(USB + 0x0A), qt.readb(USB + 0x0A)), (0x10, 0))
        check('DEVCTL: host, a full / high-speed device',
              qt.readb(USB + 0x60) & 0x44, 0x44)
        qt.writeb(USB + 0x01, 0x29)             # bus reset
        qt.writeb(USB + 0x01, 0x21)
        check('POWER: high speed negotiated (HSENAB | HSMODE)',
              qt.readb(USB + 0x01) & 0x30, 0x30)

        qt.writeb(USB + 0x00, 0)                # FADDR: the default address
        ok, desc = usb_get_device_descriptor(qt)
        check('GET_DESCRIPTOR at address 0: SETUP, IN, status, INTRTX '
              'latched with INTRTXE clear', ok, True)
        check('the 18-byte device descriptor, 007a:6162, EP0 64 bytes',
              (len(desc), desc[:2], desc[7], desc[8:12]),
              (18, b'\x12\x01', 64, b'\x7a\x00\x62\x61'))
        check('no interrupt line once INTRTX is read',
              (qt.readw(USB + 0x02), qt.readl(AV130_INTC + 0x08) >> 18 & 1),
              (0, 0))

        tx, _ = usb_ep0(qt, [0, 5, 5, 0, 0, 0, 0, 0], 0x0A)
        st, csr = usb_ep0(qt, csr0=0x60)        # status IN: REQPKT
        check('SET_ADDRESS 5: SETUP, and a status IN to the device',
              (tx, st, csr & 0x41), (1, 1, 0x41))
        check('the status IN carried no data', qt.readw(USB + 0x18), 0)
        usb_ep0(qt, csr0=0x00)
        qt.writeb(USB + 0x13, 0x01)             # CSR0 high byte: FLUSHFIFO
        check('a write to the high byte of CSR0 sends nothing',
              qt.readw(USB + 0x02), 0)

        qt.writeb(USB + 0x00, 5)                # FADDR names the device
        ok, desc = usb_get_device_descriptor(qt)
        check('GET_DESCRIPTOR at address 5 (FADDR, not TXFUNCADDR)',
              (ok, desc[8:12]), (True, b'\x7a\x00\x62\x61'))

        qt.writeb(USB + 0x0E, 1)                # INDEX 1
        qt.writel(USB + 0x64, 0x0010 << 16 | 0x0008)
        check('TX / RX FIFOADDR set by one 32-bit write',
              (qt.readw(USB + 0x64), qt.readw(USB + 0x66)), (0x0008, 0x0010))
    finally:
        qt.close()


def sfc_bank(qt, n, op, iface, length=0, addr=0, mask=0, match=0):
    b = SFC + 0x200 + 0x40 * n
    qt.writel(b + 0x00, op)
    qt.writel(b + 0x04, addr)
    qt.writel(b + 0x18, mask)
    qt.writel(b + 0x1C, match)
    qt.writel(b + 0x20, iface)
    qt.writel(b + 0x24, length)


def sfc_run(qt, last, dma=False):
    qt.writel(SFC + 0x180, last | (1 << 31) | ((1 << 30) if dma else 0))


def test_av130_sfc(qt):
    # The AV130 U-Boot's SFC v2 driver, as hw/ssi/ak-sfc.c models it.
    qt.writel(SFC + 0x100, 1)
    check('SFC: XIP stop acknowledged (0x100 bit 1)',
          qt.readl(SFC + 0x100) & 2, 2)
    qt.writel(SFC + 0x100, 0x10)
    check('SFC: FIFO reset self-clears', qt.readl(SFC + 0x100) & 0x10, 0)
    check('SFC: idle', qt.readl(SFC + 0xD0) & 0x20, 0x20)
    qt.writel(SFC + 0xC0, 0x0504)
    # RDID: 6 bytes clocked through the bank's data register, first 4 kept
    sfc_bank(qt, 0, 0x9F, 0x10000440, 5 << 6)
    sfc_run(qt, 0, dma=True)
    check('SFC: RDID done (0x0D0 bit 4)', qt.readl(SFC + 0xD0) & 0x10, 0x10)
    check('SFC: RDID in bank 0 +0x0C, first byte low',
          qt.readl(SFC + 0x200 + 0x0C) & 0xFFFFFF, 0x1740C8)
    qt.writel(SFC + 0xD0, 0x10)
    check('SFC: done is write-1-to-clear', qt.readl(SFC + 0xD0) & 0x10, 0)
    check('SFC: start clears itself', qt.readl(SFC + 0x180) >> 31, 0)
    # a FIFO read of 16 bytes from the pattern at 0x10000
    qt.writel(SFC + 0x100, 0x10)
    sfc_bank(qt, 0, 0x03, 0x4C0, 8 | (15 << 6), 0x10000)
    sfc_run(qt, 0)
    words = [qt.readl(SFC + 0x184) for _ in range(4)]
    want = [struct.unpack_from('<I', bytes((0x10000 + i) * 7 & 0xFF
                                           for i in range(16)), 4 * k)[0]
            for k in range(4)]
    check('SFC: FIFO read returns the flash, little-endian', words == want,
          True)
    check('SFC: an empty FIFO reads 0', qt.readl(SFC + 0x184), 0)
    # WREN, then a FIFO page program: it waits for its bytes
    sfc_bank(qt, 0, 0x06, 0x10004040)
    sfc_run(qt, 0, dma=True)
    qt.writel(SFC + 0xD0, 0x10)
    sfc_bank(qt, 0, 0x02, 0x44C0, 8 | (7 << 6), 0x20000)
    sfc_run(qt, 0)
    check('SFC: a program waits for the FIFO', qt.readl(SFC + 0xD0) & 0x30, 0)
    qt.writel(SFC + 0x184, 0x44332211)
    qt.writel(SFC + 0x184, 0x88776655)
    check('SFC: then completes', qt.readl(SFC + 0xD0) & 0x30, 0x30)
    check('SFC: XIP window reads what was programmed',
          (qt.readl(SFC_XIP + 0x20000), qt.readl(SFC_XIP + 0x20004)) ==
          (0x44332211, 0x88776655), True)
    qt.writel(SFC + 0xD0, 0x10)
    # WREN, then a 4 KiB erase with an RDSR poll bank behind it
    sfc_bank(qt, 0, 0x06, 0x10004040)
    sfc_run(qt, 0, dma=True)
    qt.writel(SFC + 0xD0, 0x10)
    sfc_bank(qt, 0, 0x20, 0x100040C0, 8, 0x20000)
    sfc_bank(qt, 1, 0x05, 0x18000040, 0, 0, 0xFFFFFFFE, 0)
    sfc_run(qt, 1, dma=True)
    check('SFC: erase and its status poll complete',
          qt.readl(SFC + 0xD0) & 0x10, 0x10)
    check('SFC: the sector reads erased', qt.readl(SFC_XIP + 0x20000),
          0xFFFFFFFF)
    qt.writel(SFC + 0xD0, 0x10)
    # programs one after another, no FIFO reset between: the FIFO recycles
    ok = True
    for k in range(3):
        sfc_bank(qt, 0, 0x06, 0x10004040)
        sfc_run(qt, 0, dma=True)
        qt.writel(SFC + 0xD0, 0x10)
        sfc_bank(qt, 0, 0x02, 0x44C0, 8 | (5 << 6), 0x21000 + 0x10 * k)
        sfc_run(qt, 0)
        qt.writel(SFC + 0x184, 0xA0A0A000 | k)
        qt.writel(SFC + 0x184, 0x0000B0B0)        # 6 bytes, 2 of padding
        ok &= bool(qt.readl(SFC + 0xD0) & 0x10)
        ok &= qt.readl(SFC_XIP + 0x21000 + 0x10 * k) == 0xA0A0A000 | k
        ok &= qt.readl(SFC_XIP + 0x21004 + 0x10 * k) & 0xFFFF == 0xB0B0
        qt.writel(SFC + 0xD0, 0x10)
    check('SFC: repeated FIFO programs, padding dropped', ok, True)
    # 4-byte addressing (0xB7 ... 0xE9): the XIP window follows the NOR
    sfc_bank(qt, 0, 0xB7, 0x10004040)
    sfc_run(qt, 0, dma=True)
    qt.writel(SFC + 0xD0, 0x10)
    b4 = qt.readl(SFC_XIP + 0x10000)
    sfc_bank(qt, 0, 0xE9, 0x10004040)
    sfc_run(qt, 0, dma=True)
    qt.writel(SFC + 0xD0, 0x10)
    check('SFC: XIP reads right in 4-byte mode and after it',
          (b4, qt.readl(SFC_XIP + 0x10000)) == (want[0], want[0]), True)
    # a poll for a status the NOR never shows: busy, not done, until the
    # FIFO reset abandons it
    sfc_bank(qt, 0, 0x05, 0x18000040, 0, 0, 0xFFFFFFFE, 1)
    sfc_run(qt, 0, dma=True)
    check('SFC: a poll that never matches is not done',
          qt.readl(SFC + 0xD0) & 0x30, 0)
    qt.writel(SFC + 0x100, 0x10)
    check('SFC: the FIFO reset abandons it', qt.readl(SFC + 0xD0) & 0x20,
          0x20)


def main():
    ap = argparse.ArgumentParser()
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    ap.add_argument('--qemu', default=os.path.join(
        root, 'qemu-src/build/qemu-system-arm'))
    args = ap.parse_args()

    tmp = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, tmp, True)
    path = os.path.join(tmp, 'flash.bin')
    out = os.path.join(tmp, 'uart0.txt')
    code = make_image(path)
    qt = QTest(args.qemu, 'ak3918ev300,flash-file=%s' % path,
               ['-serial', 'file:' + out])
    try:
        test_boot_rom(qt, code)
        test_sysctrl(qt)
        test_uart(qt, out)
        test_spi(qt, path)
        test_spi_more(qt, path)
        test_misc(qt)
        test_mac(qt)
        test_mci(qt)
    finally:
        qt.close()
    path330 = os.path.join(tmp, 'flash330.bin')
    make_image(path330, b'ANYKAH3B')
    test_ev330(args.qemu, path330)
    path300l = os.path.join(tmp, 'flash300l.bin')
    make_image(path300l, b'AK_H322L')
    test_ev300l(args.qemu, path300l)
    test_sar_inputs(args.qemu, path, path330)
    test_watchdog(args.qemu, path)
    test_watchdog_migration(args.qemu, tmp)
    test_av100(args.qemu, tmp)
    test_av130(args.qemu, tmp)
    test_av130_usb(args.qemu)
    test_rejections_and_migration(args.qemu, tmp, path)
    print('%s: %d failure(s)' % ('FAIL' if failures else 'PASS', failures))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
