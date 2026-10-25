#!/usr/bin/env python3
"""Register-level checks of the Fullhan FH885xV200 models, no guest needed.

QEMU runs the fh8856v200 machine under qtest and this script plays the
vendor drivers (docs/fullhan.md):

  - the mask ROM: a synthetic "2BL*" boot header's uboot record is staged
    at its load address, with the stub at 0 that enters it;
  - fh-pmu: chip, package and DDR IDs, the NOR boot strap U-Boot keys its
    environment on, resets that read back finished, the PTS latch;
  - fh-intc: a timer interrupt through enable and mask, FINALSTATUS read a
    byte at a time as the kernel's find_first_bit() does;
  - fh-timer: a disabled channel reads 0, a periodic one interrupts until
    EOI;
  - fh-uart: USR's FIFO bits;
  - fh-spi + the W25Q64 on GPIO 6: JEDEC ID by TX+RX, a fast read whose one
    dummy byte stands for 8 cycles, an RX-only read in 16-bit swapped
    frames, and one drained by the AXI DMAC through the data window on
    handshake 4;
  - fh-axi-dmac: the kernel's channel count probe, a memory-to-memory list;
  - fh-i2c: an absent target aborts with 7B_ADDR_NOACK, then STOP_DET;
  - every machine: its chip / package IDs, then a migration round trip;
  - fh-aes: FIPS-197 / SP 800-38A vectors through the DMA engine (ECB,
    CBC chained across two transfers, CTR, CFB, OFB, DES), its interrupt;
  - fh-sadc: a v3 conversion's done bit and interrupt;
  - sensor=: the default sensors' chip IDs read through I2C0 the way the
    vendor apps read them (GC4653's 16-bit registers, GC2053's 8-bit), and
    a 16-bit part ignoring ipctool's one-byte probe;
  - fh-rtc: the packed counter, a written time read back, the offset LUT
    words beside it;
  - fh-wdt: interrupt on the first expiry in response mode, paused by the
    PMU;
  - the GMAC: an IP101G at MDIO address 0;
  - fh8852v100: its chip IDs, the timer clock the PMU's divider sets (the
    kernel's 50 MHz), and a flash read on GPIO 54 drained by the AHB DMAC
    on handshake 2 through a descriptor list;
  - fh8652: the GIC-400 distributor where its kernel's init_irq maps it;
  - fh8626v100: the block resets at 0x04C, SPI0 at 0xF0E00000 reading the
    flash's ID, and the PLL1 / 60 timer clock its kernel runs at 10 MHz;
  - fh8632: its chip and variant IDs, the PTS counter at 0xE7000040, and
    the RTC v1's direct counter, sync bits and set-up magic;
  - outside qtest: machine state migrates to a file and back.

    python3 qemu/tests/test-fullhan.py [--qemu PATH]
"""
import argparse
import atexit
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

PMU = 0xF0000000
INTC = 0xE0200000
DMAC = 0xE0300000
GMAC = 0xE0600000
SDC0 = 0xE2000000
I2C0 = 0xF0200000
GPIO0 = 0xF0300000
SPI0 = 0xF0500000
UART0 = 0xF0700000
TIMER = 0xF0C00000
WDT = 0xF0D00000
RTC = 0xF1500000
DDR = 0xA0000000

LOAD = 0xA0800000
JEDEC = 0xEF4017                    # W25Q64, for an 8 MiB image


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

    def readb(self, a):
        return int(self.cmd('readb 0x%x' % a).split()[1], 16)

    def readw(self, a):
        return int(self.cmd('readw 0x%x' % a).split()[1], 16)

    def write(self, a, data):
        self.cmd('write 0x%x 0x%x 0x%s' % (a, len(data), data.hex()))

    def read(self, a, n):
        return bytes.fromhex(self.cmd('read 0x%x 0x%x' % (a, n)).split()[1][2:])

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
    if isinstance(v, (bytes, list)):
        return bytes(v)[:8].hex() + ('...' if len(v) > 8 else '')
    return hex(v)


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what, shown(got)))


def make_image(path):
    """8 MiB: a 2BL header whose uboot record points at a two-word "U-Boot"
    at flash 0x10000, and a pattern from 0x100000 on."""
    code = [0xE1A00000, 0xEAFFFFFE]         # nop; b .
    img = bytearray(b'\xFF' * (8 << 20))
    img[0:8] = b'2BL*OABD'
    struct.pack_into('<3I', img, 8, 1, 1, 7)
    rec = 0x2C0
    img[rec:rec + 0x40] = bytes(0x40)
    img[rec:rec + 5] = b'uboot'
    struct.pack_into('<4I', img, rec + 0x20, 4 * len(code), 0x10000, LOAD,
                     LOAD)
    struct.pack_into('<%dI' % len(code), img, 0x10000, *code)
    for i in range(0x100000, 0x110000):
        img[i] = i * 7 & 0xFF
    open(path, 'wb').write(img)
    return code, img


def test_boot_rom(qt, code):
    print('mask ROM')
    check('stub at 0: ldr pc, [pc, #-4]', qt.readl(0), 0xE51FF004)
    check('the entry after it', qt.readl(4), LOAD)
    check('U-Boot at its load address', [qt.readl(LOAD), qt.readl(LOAD + 4)]
          == code, True)


def test_pmu(qt):
    print('fh-pmu')
    check('platform ID', qt.readl(PMU), 0x19112201)
    check('package ID (FH8856V200)', qt.readl(PMU + 0x33C), 0x00100001)
    check('DDR size in Mbit', qt.readl(PMU + 0x334), 1024)
    check('boot strap: SPI NOR', qt.readl(PMU + 0x330), 2)
    qt.writel(PMU + 0x48, 0xFFFFFFFB)
    check('an AHB reset reads finished', qt.readl(PMU + 0x48), 0xFFFFFFFF)
    qt.writel(PMU + 0x22C, 1)
    a = qt.readl(PMU + 0x22C)
    qt.step(5000000)
    qt.writel(PMU + 0x22C, 1)
    b = qt.readl(PMU + 0x22C)
    check('PTS latch counts microseconds', 4900 <= b - a <= 5100, True)


def test_timer_intc(qt):
    print('fh-timer, fh-intc')
    check('a disabled channel reads 0', qt.readl(TIMER + 0x04), 0)
    qt.writel(INTC + 0x00, 1 << 3)          # enable the timer source
    qt.writel(TIMER + 0x00, 1000)           # 1 ms at 1 MHz
    qt.writel(TIMER + 0x08, 0x3)            # enable, periodic, unmasked
    v = qt.readl(TIMER + 0x04)
    check('it counts down from LOADCOUNT', 0 < v <= 1000, True)
    qt.step(1500000)
    check('raw interrupt', qt.readl(TIMER + 0xA8) & 1, 1)
    check('INTC raw status bit 3', qt.readl(INTC + 0x18) & (1 << 3), 1 << 3)
    check('FINALSTATUS byte 0', qt.readb(INTC + 0x30), 1 << 3)
    check('FINALSTATUS byte 1', qt.readb(INTC + 0x31), 0)
    qt.writel(INTC + 0x08, 1 << 3)          # mask it
    check('masked: FINALSTATUS clear', qt.readl(INTC + 0x30), 0)
    qt.writel(INTC + 0x08, 0)
    qt.readl(TIMER + 0x0C)                  # EOI
    check('EOI clears it', qt.readl(INTC + 0x30), 0)
    qt.step(1000000)
    check('periodic: again a period later', qt.readl(INTC + 0x30), 1 << 3)
    qt.writel(TIMER + 0x08, 0)
    qt.readl(TIMER + 0x0C)


def test_uart(qt):
    print('fh-uart')
    check('USR: TX FIFO empty and not full, RX empty',
          qt.readl(UART0 + 0x7C), 0x6)
    check('component type', qt.readl(UART0 + 0xFC), 0x44570110)


def spi_cs(qt, low):
    """GPIO 6, the flash's select, as gpio_direction_output() drives it."""
    qt.writel(GPIO0 + 0x00, 0 if low else 1 << 6)
    qt.writel(GPIO0 + 0x04, 1 << 6)


def spi_mode(qt, ctrlr0):
    qt.writel(SPI0 + 0x08, 0)
    qt.writel(SPI0 + 0x00, ctrlr0)
    qt.writel(SPI0 + 0x08, 1)


def spi_txrx(qt, data):
    spi_mode(qt, 0x07)                      # 8-bit frames, TX+RX
    for b in data:
        qt.writel(SPI0 + 0x60, b)
    return [qt.readl(SPI0 + 0x60) for _ in data]


def test_spi(qt, img):
    print('fh-spi, W25Q64 on GPIO 6')
    spi_cs(qt, True)
    rx = spi_txrx(qt, [0x9F, 0, 0, 0])
    spi_cs(qt, False)
    check('JEDEC ID', (rx[1] << 16) | (rx[2] << 8) | rx[3], JEDEC)
    spi_cs(qt, True)
    rx = spi_txrx(qt, [0x0B, 0x10, 0x00, 0x00, 0x00] + [0xFF] * 4)
    spi_cs(qt, False)
    check('fast read: one dummy byte is its 8 cycles', bytes(rx[5:]),
          bytes(img[0x100000:0x100004]))
    # RX-only in 16-bit frames with the swap: the command goes out in
    # TX-only mode, then CTRLR1 + 1 frames come in on a dummy write.
    spi_cs(qt, True)
    spi_mode(qt, 0x107)
    for b in (0x03, 0x10, 0x00, 0x10):
        qt.writel(SPI0 + 0x60, b)
    qt.writel(SPI0 + 0x08, 0)
    qt.writel(SPI0 + 0xF4, 1 << 12)
    qt.writel(SPI0 + 0x00, 0x20F)
    qt.writel(SPI0 + 0x04, 3)
    qt.writel(SPI0 + 0x08, 1)
    qt.writel(SPI0 + 0x60, 0xFFFFFFFF)
    check('RX FIFO holds the 4 frames', qt.readl(SPI0 + 0x24), 4)
    got = b''.join(struct.pack('<H', qt.readl(SPI0 + 0x60))
                   for _ in range(4))
    spi_cs(qt, False)
    check('swapped frames are memory order', got,
          bytes(img[0x100010:0x100018]))
    qt.writel(SPI0 + 0x08, 0)
    qt.writel(SPI0 + 0xF4, 0)


def lli(qt, at, sar, dar, items, ctl_lo, last=True, nxt=0):
    words = [sar, 0, dar, 0, items - 1, 0, nxt, 0, ctl_lo,
             (1 << 31) | ((1 << 30) if last else 0)]
    for i, w in enumerate(words):
        qt.writel(at + 4 * i, w)


def test_dmac(qt, img):
    print('fh-axi-dmac')
    qt.writel(DMAC + 0x58, 1)
    check('RESET self-clears', qt.readl(DMAC + 0x58), 0)
    qt.writel(DMAC + 0x10, 1)
    n = 0
    for i in range(8):
        qt.writel(DMAC + 0x18, (1 << i) | (1 << (8 + i)))
        if qt.readl(DMAC + 0x18) < (1 << i):
            break
        n += 1
    check('the kernel\'s probe counts 8 channels', n, 8)
    qt.writel(DMAC + 0x58, 1)
    check('RESET drops them', qt.readl(DMAC + 0x18), 0)
    qt.writel(DMAC + 0x10, 1)

    # Memory to memory, 32-bit items, both sides incrementing.
    src, dst, desc = DDR + 0x100000, DDR + 0x200000, DDR + 0x300000
    for i in range(16):
        qt.writel(src + 4 * i, 0x01010101 * i)
    lli(qt, desc, src, dst, 16, (2 << 8) | (2 << 11))
    ch = DMAC + 0x100
    qt.writel(ch + 0x20, 0xF)
    qt.writel(ch + 0x24, 0)
    qt.writel(ch + 0x28, desc)
    qt.writel(ch + 0x80, 3)
    qt.writel(DMAC + 0x18, 0x101)
    qt.step(1000)
    check('DMA_TFR', qt.readl(ch + 0x88) & 2, 2)
    check('the channel enable dropped', qt.readl(DMAC + 0x18) & 1, 0)
    check('data copied', [qt.readl(dst + 4 * i) for i in range(16)] ==
          [0x01010101 * i for i in range(16)], True)
    qt.writel(ch + 0x98, 0xFFFFFFFF)

    # SPI0 to memory on handshake 4, U-Boot's flash read: 16-bit swapped
    # frames from the data window, its address reloaded per descriptor.
    dst = DDR + 0x400000
    lli(qt, desc, SPI0 + 0x1000, dst, 8, (1 << 8) | (1 << 11) | (1 << 4))
    qt.writel(ch + 0x24, 2 | (4 << 7))
    qt.writel(ch + 0x28, desc)
    qt.writel(DMAC + 0x18, 0x101)
    spi_cs(qt, True)
    spi_mode(qt, 0x107)
    for b in (0x03, 0x10, 0x00, 0x20):
        qt.writel(SPI0 + 0x60, b)
    qt.writel(SPI0 + 0x08, 0)
    qt.writel(SPI0 + 0xF4, 1 << 12)
    qt.writel(SPI0 + 0x00, 0x20F)
    qt.writel(SPI0 + 0x04, 7)
    qt.writel(SPI0 + 0x08, 1)
    qt.writel(SPI0 + 0x4C, 1)               # RX DMA on
    qt.writel(SPI0 + 0x60, 0xFFFFFFFF)
    qt.step(1000)
    spi_cs(qt, False)
    check('handshake transfer done', qt.readl(ch + 0x88) & 2, 2)
    got = b''.join(struct.pack('<I', qt.readl(dst + 4 * i)) for i in range(4))
    check('flash bytes in memory order', got, bytes(img[0x100020:0x100030]))
    qt.writel(SPI0 + 0x4C, 0)
    qt.writel(SPI0 + 0x08, 0)
    qt.writel(SPI0 + 0xF4, 0)


def test_i2c(qt):
    print('fh-i2c')
    check('FIFO depths in COMP_PARAM_1', (qt.readl(I2C0 + 0xF4) >> 8) & 0xFF,
          15)
    qt.writel(I2C0 + 0x6C, 0)
    qt.writel(I2C0 + 0x04, 0x30)
    qt.writel(I2C0 + 0x6C, 1)
    qt.writel(I2C0 + 0x10, 0x100 | 0x200)   # read one byte, then STOP
    raw = qt.readl(I2C0 + 0x34)
    check('TX_ABRT', raw & (1 << 6), 1 << 6)
    check('STOP_DET', raw & (1 << 9), 1 << 9)
    check('7B_ADDR_NOACK', qt.readl(I2C0 + 0x80), 1)
    qt.readl(I2C0 + 0x54)
    check('CLR_TX_ABRT clears it', qt.readl(I2C0 + 0x34) & (1 << 6), 0)


def i2c_xfer(qt, addr, wr, nrd):
    """Write wr, then read nrd bytes (a restart), the way I2C_RDWR does."""
    qt.writel(I2C0 + 0x6C, 0)
    qt.writel(I2C0 + 0x04, addr)
    qt.writel(I2C0 + 0x6C, 1)
    for b in wr:
        qt.writel(I2C0 + 0x10, b)
    for i in range(nrd):
        qt.writel(I2C0 + 0x10, 0x100 | (0x200 if i == nrd - 1 else 0))
    return [qt.readl(I2C0 + 0x10) & 0xFF for _ in range(nrd)]


def test_sensor(qt, name, addr, reg16, idreg, ident):
    print('sensor: %s at 0x%02x' % (name, addr))
    a = [idreg >> 8, idreg & 0xFF] if reg16 else [idreg]
    check('chip ID', i2c_xfer(qt, addr, a, 2), [ident >> 8, ident & 0xFF])
    if reg16:
        # ipctool's narrow probe: one address byte must not find the ID
        check('8-bit probe of a 16-bit part', i2c_xfer(qt, addr, [0xF0], 2),
              [0, 0])
    a = [0x03, 0x40] if reg16 else [0x40]
    i2c_xfer(qt, addr, a + [0x5A, 0xA5], 0)
    check('init table writes read back', i2c_xfer(qt, addr, a, 2),
          [0x5A, 0xA5])


AES = 0xE8200000


def aes_run(qt, ctl, key, iv, data):
    """One transfer, programmed the way fh_aes.c does it."""
    src, dst = DDR + 0x100000, DDR + 0x110000
    qt.writel(AES + 0x00, ctl)
    for i in range(0, len(key), 4):
        qt.writel(AES + 0x10 + i, int.from_bytes(key[i:i + 4], 'big'))
    if iv:
        for i in range(0, len(iv), 4):
            qt.writel(AES + 0x30 + i, int.from_bytes(iv[i:i + 4], 'big'))
    qt.write(src, data)
    qt.writel(AES + 0x48, src)
    qt.writel(AES + 0x4C, dst)
    qt.writel(AES + 0x50, len(data))
    qt.writel(AES + 0x5C, 1)
    qt.writel(AES + 0x54, 1)
    return qt.read(dst, len(data))


def test_aes(qt):
    print('fh-aes')
    h = bytes.fromhex
    qt.writel(INTC + 0x00, 1 << 16)
    # FIPS-197 C.1
    out = aes_run(qt, 4 << 1, h('000102030405060708090a0b0c0d0e0f'), None,
                  h('00112233445566778899aabbccddeeff'))
    check('AES-128 ECB', out.hex(), '69c4e0d86a7b0430d8cdb78070b4c55a')
    check('done interrupt (IRQ 16)', qt.readl(INTC + 0x18) & (1 << 16),
          1 << 16)
    qt.writel(AES + 0x68, 7)
    check('cleared', qt.readl(INTC + 0x18) & (1 << 16), 0)
    out = aes_run(qt, 4 << 1 | 1, h('000102030405060708090a0b0c0d0e0f'),
                  None, out)
    check('AES-128 ECB decrypt', out.hex(), '00112233445566778899aabbccddeeff')
    # SP 800-38A F.2.1, the second block in a transfer of its own
    key = h('2b7e151628aed2a6abf7158809cf4f3c')
    out = aes_run(qt, 1 << 7 | 1 << 4 | 4 << 1, key,
                  h('000102030405060708090a0b0c0d0e0f'),
                  h('6bc1bee22e409f96e93d7e117393172a'))
    check('AES-128 CBC', out.hex(), '7649abac8119b246cee98e9b12e9197d')
    out = aes_run(qt, 1 << 4 | 4 << 1, key, None,
                  h('ae2d8a571e03ac9c9eb76fac45af8e51'))
    check('CBC chains across transfers', out.hex(),
          '5086cb9b507219ee95db113a917678b2')
    check('LAST_IV', qt.readl(AES + 0x80), 0x5086cb9b)
    # F.5.1
    out = aes_run(qt, 1 << 7 | 2 << 4 | 4 << 1, key,
                  h('f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff'),
                  h('6bc1bee22e409f96e93d7e117393172a'
                    'ae2d8a571e03ac9c9eb76fac45af8e51'))
    check('AES-128 CTR', out.hex(),
          '874d6191b620e3261bef6864990db6ce'
          '9806f66b7970fdff8617187bb9fffdff')
    # F.3.13 CFB128, F.4.1 OFB
    out = aes_run(qt, 1 << 7 | 4 << 4 | 4 << 1, key,
                  h('000102030405060708090a0b0c0d0e0f'),
                  h('6bc1bee22e409f96e93d7e117393172a'))
    check('AES-128 CFB', out.hex(), '3b3fd92eb72dad20333449f8e83cfb4a')
    out = aes_run(qt, 1 << 7 | 5 << 4 | 4 << 1, key,
                  h('000102030405060708090a0b0c0d0e0f'),
                  h('6bc1bee22e409f96e93d7e117393172a'
                    'ae2d8a571e03ac9c9eb76fac45af8e51'))
    check('AES-128 OFB', out.hex(),
          '3b3fd92eb72dad20333449f8e83cfb4a'
          '7789508d16918f03f53c52dac54ed825')
    out = aes_run(qt, 0, h('133457799bbcdff1'), None, h('0123456789abcdef'))
    check('DES ECB', out.hex(), '85e813540f0ab405')
    qt.writel(AES + 0x68, 7)


SADC = 0xF1200000


def test_sadc(qt):
    print('fh-sadc (v3)')
    qt.writel(INTC + 0x00, 1 << 16 | 1 << 20)
    qt.writel(SADC + 0x00, 1)                   # enable
    qt.writel(SADC + 0x40, 1 << 3)              # input 3's done interrupt
    qt.writel(SADC + 0x04, 1)                   # take the configuration
    check('not done at once', qt.readl(SADC + 0x44), 0)
    qt.step(200000)
    check('input 3 done', qt.readl(SADC + 0x44), 1 << 3)
    check('interrupt 20', qt.readl(INTC + 0x18) & (1 << 20), 1 << 20)
    qt.writel(SADC + 0x44, ~(1 << 3) & 0xFFFFFFFF)
    check('cleared by the complement', qt.readl(INTC + 0x18) & (1 << 20), 0)


rtc_busy = 0


def rtc_cmd(qt, cmd, data=None):
    """The driver's handshake: clear CORE_IDLE, command, poll it back."""
    global rtc_busy
    st = qt.readl(RTC + 0x00)
    qt.writel(RTC + 0x00, st & ~(1 << 5))
    if data is not None:
        qt.writel(RTC + 0x1C, data)
    qt.writel(RTC + 0x14, cmd)
    rtc_busy += not qt.readl(RTC + 0x00) & (1 << 5)
    return qt.readl(RTC + 0x18)


def test_rtc(qt):
    print('fh-rtc')
    v = rtc_cmd(qt, 1)
    now = int(time.time())
    secs = (v >> 17) * 86400 + ((v >> 12) & 0x1F) * 3600 + \
        ((v >> 6) & 0x3F) * 60 + (v & 0x3F)
    check('counter is the host time', abs(secs - now) < 5, True)
    t = (20000 << 17) | (12 << 12) | (34 << 6) | 56
    rtc_cmd(qt, 2, t)
    for i in range(12):                     # rtc_adjust()'s offset LUT
        rtc_cmd(qt, (9 + i) << 4 | 2, 0x11111111 * (i & 0xF))
    check('a written time reads back', rtc_cmd(qt, 1) >> 6, t >> 6)
    check('the last LUT word', rtc_cmd(qt, (20 << 4) | 1), 0xBBBBBBBB)
    check('every command left the core idle', rtc_busy, 0)


def test_wdt(qt):
    print('fh-wdt')
    qt.writel(INTC + 0x00, 1 << 2)
    qt.writel(WDT + 0x04, 0)                # 2^16 cycles: 65.5 ms
    qt.writel(WDT + 0x00, 3)                # enable, interrupt first
    qt.writel(WDT + 0x0C, 0x76)
    qt.writel(PMU + 0x1FC, 0x100)           # paused
    qt.step(100000000)
    check('paused: no expiry', qt.readl(WDT + 0x10), 0)
    qt.writel(PMU + 0x1FC, 0)
    qt.step(70000000)
    check('first expiry interrupts', qt.readl(WDT + 0x10), 1)
    check('on INTC source 2', qt.readl(INTC + 0x30) & (1 << 2), 1 << 2)
    qt.writel(WDT + 0x0C, 0x76)
    check('a kick clears it', qt.readl(WDT + 0x10), 0)
    qt.writel(PMU + 0x1FC, 0x100)


def mdio_read(qt, phy, reg):
    qt.writel(GMAC + 0x10, (phy << 11) | (reg << 6) | 1)
    return qt.readl(GMAC + 0x14)


def test_gmac_sd(qt):
    print('GMAC, SD')
    check('IP101G at MDIO address 0',
          (mdio_read(qt, 0, 2) << 16) | mdio_read(qt, 0, 3), 0x02430C54)
    check('nothing at address 1', mdio_read(qt, 1, 2), 0xFFFF)
    check('SD0 empty: CDETECT reads no card', qt.readl(SDC0 + 0x50) & 1, 1)


def test_v100(qemu, path, img):
    print('fh8852v100: AHB DMAC, timer clock')
    qt = QTest(qemu, 'fh8852v100,flash-file=%s' % path)
    gpio1 = 0xF4000000
    try:
        check('platform ID', qt.readl(PMU), 0x17092901)
        check('package ID (FH8852)', qt.readl(PMU + 0x004), 0xC)
        # The kernel's clk_set_rate(tmr0_clk, 50 MHz): DIV5[23:16] = 1.
        div5 = qt.readl(PMU + 0x38)
        qt.writel(PMU + 0x38, (div5 & ~0xFF0000) | (1 << 16))
        qt.writel(TIMER + 0x00, 50000)          # 1 ms at 50 MHz
        qt.writel(TIMER + 0x08, 0x3)
        qt.step(900000)
        check('50 MHz: not yet at 0.9 ms', qt.readl(TIMER + 0xA8) & 1, 0)
        qt.step(200000)
        check('expired by 1.1 ms', qt.readl(TIMER + 0xA8) & 1, 1)
        qt.writel(TIMER + 0x08, 0)

        dmac = DMAC
        check('FH_PARAMS: encoded, 8 channels',
              qt.readl(dmac + 0x3F4) & ((1 << 28) | (7 << 8)),
              (1 << 28) | (7 << 8))
        # Two descriptors of 4 16-bit items each, from SPI0's DR to memory.
        desc, dst = DDR + 0x300000, DDR + 0x400000
        ctl = (1 << 4) | (1 << 1) | (2 << 9) | (2 << 20)
        for i, (sar, dar, nxt, lo) in enumerate((
                (SPI0 + 0x60, dst, desc + 0x20, ctl | (3 << 27)),
                (SPI0 + 0x60, dst + 8, 0, ctl))):
            for j, w in enumerate((sar, dar, nxt, lo, 4)):
                qt.writel(desc + 0x20 * i + 4 * j, w)
        qt.writel(dmac + 0x398, 1)
        qt.writel(dmac + 0x44, 2 << 7)          # SRC_PER 2: SPI0 RX
        qt.writel(dmac + 0x10, desc)
        qt.writel(dmac + 0x18, 3 << 27)         # LLP enabled: fetch first
        qt.writel(dmac + 0x3A0, 0x101)
        # GPIO 54 (GPIO1 bit 22) selects the flash.
        qt.writel(gpio1 + 0x00, 0)
        qt.writel(gpio1 + 0x04, 1 << 22)
        spi_mode(qt, 0x107)
        for b in (0x03, 0x10, 0x00, 0x40):
            qt.writel(SPI0 + 0x60, b)
        qt.writel(SPI0 + 0x08, 0)
        qt.writel(SPI0 + 0xF4, 1 << 12)
        qt.writel(SPI0 + 0x00, 0x20F)
        qt.writel(SPI0 + 0x04, 7)
        qt.writel(SPI0 + 0x08, 1)
        qt.writel(SPI0 + 0x4C, 1)
        qt.writel(SPI0 + 0x60, 0xFF)
        qt.step(1000)
        qt.writel(gpio1 + 0x00, 1 << 22)
        check('RAW.XFER for channel 0', qt.readl(dmac + 0x2C0) & 1, 1)
        check('the channel enable dropped', qt.readl(dmac + 0x3A0) & 1, 0)
        check('DONE written back to the first descriptor',
              (qt.readl(desc + 16) >> 12) & 1, 1)
        got = b''.join(struct.pack('<I', qt.readl(dst + 4 * i))
                       for i in range(4))
        check('flash bytes in memory order', got,
              bytes(img[0x100040:0x100050]))
    finally:
        qt.close()


def test_fh8652(qemu):
    print('fh8652: GIC')
    qt = QTest(qemu, 'fh8652')
    gicd = 0x30001000
    try:
        check('platform ID', qt.readl(PMU), 0x20031601)
        # The distributor where init_irq maps it.  qtest's accesses are
        # non-secure, and the interrupts reset to secure group 0, so
        # delivery itself is left to booting the dump.
        check('GICD_TYPER at 0x30001004: 64 SPIs',
              qt.readl(gicd + 0x004) & 0x1F, 2)
    finally:
        qt.close()


def test_fh8626(qemu, path):
    print('fh8626v100: resets, SPI0, timer clock')
    qt = QTest(qemu, 'fh8626v100,flash-file=%s' % path)
    spi0, gpio1 = 0xF0E00000, 0xF4000000
    try:
        test_sensor(qt, 'gc2053 (FH8626V100 default)', 0x37, False, 0xF0,
                    0x2053)
        check('platform ID', qt.readl(PMU), 0x18112301)
        qt.writel(PMU + 0x54, 0xFFFFEFFF)       # the SD0 reset
        check('AHB resets at 0x054 read finished', qt.readl(PMU + 0x54),
              0xFFFFFFFF)
        qt.writel(gpio1 + 0x00, 0)
        qt.writel(gpio1 + 0x04, 1 << 22)        # GPIO 54
        qt.writel(spi0 + 0x08, 0)
        qt.writel(spi0 + 0x00, 0x07)
        qt.writel(spi0 + 0x08, 1)
        for b in (0x9F, 0, 0, 0):
            qt.writel(spi0 + 0x60, b)
        rx = [qt.readl(spi0 + 0x60) for _ in range(4)]
        qt.writel(gpio1 + 0x00, 1 << 22)
        check('JEDEC ID through SPI0 at 0xF0E00000',
              (rx[1] << 16) | (rx[2] << 8) | rx[3], JEDEC)
        # Its kernel's tmr0_clk: PLL1 600 / (DIV5[23:16] 0x3B + 1), no
        # prediv -- 10 MHz from the reset value.
        check('DIV5[23:16] at reset', (qt.readl(PMU + 0x38) >> 16) & 0xFF,
              0x3B)
        qt.writel(TIMER + 0x00, 10000)          # 1 ms at 10 MHz
        qt.writel(TIMER + 0x08, 0x3)
        qt.step(900000)
        check('10 MHz: not yet at 0.9 ms', qt.readl(TIMER + 0xA8) & 1, 0)
        qt.step(200000)
        check('expired by 1.1 ms', qt.readl(TIMER + 0xA8) & 1, 1)
    finally:
        qt.close()


def test_fh8632(qemu):
    print('fh8632: PTS, RTC v1')
    qt = QTest(qemu, 'fh8632')
    pts = 0xE7000040
    try:
        check('platform ID', qt.readl(PMU), 0x46488302)
        check('variant FH8632 (PMU 0x054)', qt.readl(PMU + 0x54) & 0x3F, 0x37)
        a = qt.readl(pts)
        qt.step(2000000)
        b = qt.readl(pts)
        check('PTS counts microseconds', 1900 <= (b - a) & 0xFFFFFFFF <= 2100,
              True)
        check('RTC v1 set up (USER_REG)', qt.readl(RTC + 0x20), 0x55555555)
        check('sync bits read done', qt.readl(RTC + 0x18) & 3, 3)
        t = (20000 << 17) | (1 << 12) | (2 << 6) | 3
        qt.writel(RTC + 0x00, t)
        check('a written counter reads back', qt.readl(RTC + 0x00) >> 6,
              t >> 6)
    finally:
        qt.close()


def hmp(qemu, args, cmds):
    """Run QEMU with an HMP monitor on stdio; return what it printed."""
    p = subprocess.run([qemu] + args + ['-display', 'none', '-nodefaults',
                                        '-monitor', 'stdio'],
                       input='\n'.join(cmds + ['quit']) + '\n',
                       capture_output=True, text=True, timeout=60)
    return p.stdout + p.stderr


# Every machine, with its PMU chip ID and the package register that tells
# the die's siblings apart.
MACHINES = (
    ('fh8852v200', 0x19112201, 0x33C, 0x00000001),
    ('fh8856v200', 0x19112201, 0x33C, 0x00100001),
    ('fh8858v200', 0x19112201, 0x33C, 0x00410001),
    ('fh8652', 0x20031601, 0x33C, 0xC),
    ('fh8656', 0x20031601, 0x33C, 0x8),
    ('fh8658', 0x20031601, 0x33C, 0x4),
    ('fh8852v100', 0x17092901, 0x004, 0xC),
    ('fh8856v100', 0x17092901, 0x004, 0xD),
    ('fh8626v100', 0x18112301, None, None),
    ('fh8632', 0x46488302, 0x054, 0x37),
)


def test_all_machines(qemu):
    print('every machine comes up')
    for m, chip, preg, pval in MACHINES:
        qt = QTest(qemu, m)
        try:
            check('%s: chip ID' % m, qt.readl(PMU), chip)
            if preg is not None:
                check('%s: package register 0x%03x' % (m, preg),
                      qt.readl(PMU + preg) & 0xFFFFFF, pval)
            check('%s: NOR boot strap' % m, qt.readl(PMU + 0x330), 2)
        finally:
            qt.close()


def test_migration(qemu, tmp):
    print('migration')
    for m, _, _, _ in MACHINES:
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
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    ap.add_argument('--qemu', default=os.path.join(
        root, 'qemu-src/build/qemu-system-arm'))
    args = ap.parse_args()

    tmp = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, tmp, True)
    path = os.path.join(tmp, 'flash.bin')
    code, img = make_image(path)
    qt = QTest(args.qemu, 'fh8856v200,flash-file=%s' % path)
    try:
        test_boot_rom(qt, code)
        test_pmu(qt)
        test_timer_intc(qt)
        test_uart(qt)
        test_spi(qt, img)
        test_dmac(qt, img)
        test_i2c(qt)
        test_sensor(qt, 'gc4653 (FH885xV200 default)', 0x29, True, 0x03F0,
                    0x4653)
        test_aes(qt)
        test_sadc(qt)
        test_rtc(qt)
        test_wdt(qt)
        test_gmac_sd(qt)
    finally:
        qt.close()
    path = os.path.join(tmp, 'flash100.bin')
    code, img = make_image(path)
    test_v100(args.qemu, path, img)
    test_fh8652(args.qemu)
    test_fh8626(args.qemu, path)
    test_fh8632(args.qemu)
    test_all_machines(args.qemu)
    test_migration(args.qemu, tmp)
    print('%s: %d failure(s)' % ('FAIL' if failures else 'PASS', failures))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
