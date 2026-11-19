#!/usr/bin/env python3
"""Register-level checks of the Fullhan FH885xV200 models, no guest needed.

QEMU runs the fh8856v200 machine under qtest and this script plays the
vendor drivers (docs/fullhan.md):

  - the mask ROM: a synthetic "2BL*" boot header's uboot record is staged
    at its load address, with the stub at 0 that enters it -- also from a
    header longer than 0x300;
  - the V100 GMAC's receive start: the first SR after the ring is set up
    waits for the descriptor fetch, later ones take a waiting frame at once;
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
  - fh8626v100: the block resets at 0x04C, UART0 answering across its
    1 MB slot, SPI0 at 0xF0E00000 reading the flash's ID, and the
    timer clock: 10 MHz from reset (the vendor kernel), 50 MHz once
    DIV5[7:0] is 1 (OpenIPC's 4.19);
  - fh8626v100's ARC600 (fh-arc): a program booted through the PMU's
    boot words, run and reset bits -- ALU, shifts, flags and conditions,
    BL / J / BRcc, loads and stores, LR, the doorbells both ways and IRQ
    32, the instruction-error exception, a second boot after a reset;
  - fh8626v100's CSI, ISP and VPU (fh-csi, fh-isp) behind a GC1054,
    programmed as OpenIPC's drivers do: frames only with the CSI out of
    reset, at the sensor's 40 ms, their interrupts, NV12 at the address
    latched at the frame's start, zone and AWB statistics at the light's
    ratios, a sensor exposure showing three frame events later;
  - fh8626v100's JPEG (fh-jpeg) and H.264 (fh-pae) engines, programmed as
    OpenIPC's drivers do: the JPEG's ID, a whole JFIF at the coded size,
    rotation, decoded by PIL where it is installed; the PAE's reset, its
    header FIFO leading the escaped payload, the bit count and interrupt,
    a P picture predicted from the IDR's reconstruction, a small ring;
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
    check('no slot mirror off FH8626V100', qt.readl(UART0 + 0x20014), 0)


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
    qt.write(dst, bytes(len(data)))
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
    if out == bytes(8):
        # DES comes from QEMU's crypto backend; a build without one has none
        print('  SKIP: DES ECB (no crypto backend in this build)')
    else:
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


def make_long_header(path):
    """A header longer than 0x300, as a Vatilon PB4's (FH8856V200): ten
    DDR parameter records, then "ram" (type 0x10, in SRAM) at 0x380 and
    "uboot" (type 0x11, in DDR) at 0x3C0, the size 0x400 at +0x14."""
    code = [0xE1A00000, 0xE1A00000, 0xEAFFFFFE]
    img = bytearray(b'\xFF' * (8 << 20))
    img[0:8] = b'2BL*OABD'
    struct.pack_into('<4I', img, 8, 1, 2, 10, 0x400)
    for i in range(10):
        rec = 0x100 + 0x40 * i
        img[rec:rec + 0x40] = bytes(0x40)
        img[rec:rec + 4] = b'DDR%d' % (i // 3)
        struct.pack_into('<5I', img, rec + 0x20, 0x1080, 0x1000 + 0x1400 * i,
                         0x10000800, 0x10000800, 0x43)
    for rec, name, size, off, load, typ in (
            (0x380, b'ram', 0x3600, 0x10000, 0x10008000, 0x10),
            (0x3C0, b'uboot', 4 * len(code), 0x14000, LOAD, 0x11)):
        img[rec:rec + 0x40] = bytes(0x40)
        img[rec:rec + len(name)] = name
        struct.pack_into('<5I', img, rec + 0x20, size, off, load, load, typ)
    struct.pack_into('<%dI' % len(code), img, 0x14000, *code)
    open(path, 'wb').write(img)
    return code


def test_long_header(qemu, path):
    print('fh8856v200: a boot header longer than 0x300')
    code = make_long_header(path)
    qt = QTest(qemu, 'fh8856v200,flash-file=%s' % path)
    try:
        check('the entry is the uboot record past 0x300', qt.readl(4), LOAD)
        check('U-Boot at its load address',
              [qt.readl(LOAD + 4 * i) for i in range(len(code))] == code, True)
    finally:
        qt.close()


def test_gmac_rx_start(qemu):
    """Receive DMA (SR) on the V100 GMAC.  Linux sets SR once after setting
    up the ring and needs the first descriptor fetch to come later; the V100
    U-Boot sets and clears SR around every poll, so later starts must take a
    waiting frame at once.  Frames come in over a dgram netdev; under qtest
    the virtual clock moves only by clock_step, so a frame taken without one
    was taken without the delay."""
    print('fh8852v100: GMAC receive start')
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(('127.0.0.1', 0))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.bind(('127.0.0.1', 0))
    qport = tx.getsockname()[1]
    tx.close()
    qt = QTest(qemu, 'fh8852v100', (
        '-netdev', 'dgram,id=n0,local.type=inet,local.host=127.0.0.1,'
        'local.port=%d,remote.type=inet,remote.host=127.0.0.1,'
        'remote.port=%d' % (qport, rx.getsockname()[1]),
        '-net', 'nic,netdev=n0'))
    ring, bufs = DDR + 0x500000, DDR + 0x510000
    dma = GMAC + 0x1000

    def setup_ring():
        for i in range(4):
            qt.writel(ring + 16 * i, 1 << 31)
            qt.writel(ring + 16 * i + 4, 1536 | ((1 << 25) if i == 3 else 0))
            qt.writel(ring + 16 * i + 8, bufs + 0x800 * i)
            qt.writel(ring + 16 * i + 12, 0)
        qt.writel(dma + 0x0C, ring)                 # RX_BASE

    def send(tag):
        frame = b'\xff' * 6 + bytes(range(6)) + b'\x88\xb5' + tag * 50
        rx.sendto(frame, ('127.0.0.1', qport))
        time.sleep(0.2)
        return len(frame)

    def taken(i):
        return not qt.readl(ring + 16 * i) >> 31

    try:
        qt.writel(GMAC + 0x00, 1 << 2)              # RE
        setup_ring()
        qt.writel(dma + 0x18, 1 << 1)               # SR
        n = send(b'a')
        check('first SR after the ring: no frame before the fetch delay',
              taken(0), False)
        qt.step(1500000)
        time.sleep(0.2)
        check('... and the frame once it has passed', taken(0), True)
        check('... whole with its FCS, first and last',
              qt.readl(ring) & 0x3FFF0300, ((n + 4) << 16) | 0x300)
        qt.writel(dma + 0x18, 0)                    # SR off, as U-Boot does
        send(b'b')
        check('SR off: the frame waits', taken(1), False)
        qt.writel(dma + 0x18, 1 << 1)
        time.sleep(0.2)
        check('SR on again: taken with no clock step', taken(1), True)
        check('... the next frame, in the next buffer',
              qt.read(bufs + 0x800 + 14, 4), b'bbbb')
        qt.writel(dma + 0x18, 0)
        setup_ring()
        qt.writel(dma + 0x18, 1 << 1)
        send(b'c')
        check('a new ring: the first SR is delayed again', taken(0), False)
    finally:
        qt.close()
        rx.close()


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
        # tmr0_clk: PLL1 600 / 6 / (DIV5[7:0] + 1) (OpenIPC 4.19's
        # clk-fh8626v100.c) -- 10 MHz from the reset value, which the vendor
        # 3.0.8 kernel runs at without touching the divider.
        div5 = qt.readl(PMU + 0x38)
        check('DIV5[7:0] at reset', div5 & 0xFF, 9)
        check('DIV5[23:16] at reset', (div5 >> 16) & 0xFF, 0x3B)
        qt.writel(TIMER + 0x00, 10000)          # 1 ms at 10 MHz
        qt.writel(TIMER + 0x08, 0x3)
        qt.step(900000)
        check('10 MHz: not yet at 0.9 ms', qt.readl(TIMER + 0xA8) & 1, 0)
        qt.step(200000)
        check('expired by 1.1 ms', qt.readl(TIMER + 0xA8) & 1, 1)
        # The 4.19 kernel asks for its DT's 50 MHz: DIV5[7:0] = 1.
        qt.writel(TIMER + 0x08, 0)
        qt.writel(PMU + 0x38, (div5 & ~0xFF) | 1)
        qt.writel(TIMER + 0x00, 50000)          # 1 ms at 50 MHz
        qt.writel(TIMER + 0x08, 0x3)
        qt.readl(TIMER + 0x0C)
        qt.step(900000)
        check('50 MHz: not yet at 0.9 ms', qt.readl(TIMER + 0xA8) & 1, 0)
        qt.step(200000)
        check('expired by 1.1 ms', qt.readl(TIMER + 0xA8) & 1, 1)
        # Its kernel's DEBUG_LL polls LSR through a 1 MB section at
        # 0xF0720000; UART0 answers across its slot.
        check('LSR mirrored at 0xF0720014: THRE | TEMT',
              qt.readl(UART0 + 0x20014) & 0x60, 0x60)
        qt.writel(UART0 + 0x2001C, 0xA5)        # SCR through the mirror
        check('SCR written through the mirror', qt.readl(UART0 + 0x1C), 0xA5)
        check('UART2 mirrored at 0xF13FFF14', qt.readl(0xF13FFF14) & 0x60,
              0x60)
    finally:
        qt.close()


# fh-arc: an ARCompact encoder for the formats the model decodes. Every
# format was checked against binutils' ARC assembler: the echo firmware's
# own words come out of it unchanged.
ARC_LIMM = 62


def arc_b(b):
    return (b & 7) << 24 | (b >> 3) << 12


def arc_alu(op, a, b, c=None, u6=None, s12=None, f=0, major=4, limm=None,
            cc=None):
    w = major << 27 | arc_b(b) | op << 16 | f << 15
    if cc is not None:
        w |= 3 << 22 | cc | ((u6 << 6 | 0x20) if u6 is not None else c << 6)
    elif s12 is not None:
        v = s12 & 0xFFF
        w |= 2 << 22 | (v & 0x3F) << 6 | v >> 6
    elif u6 is not None:
        w |= 1 << 22 | u6 << 6 | a
    else:
        w |= c << 6 | a
    return [w] + ([limm & 0xFFFFFFFF] if limm is not None else [])


def arc_mov(b, u6=None, s12=None, limm=None, cc=None):
    if limm is not None:
        return arc_alu(0x0A, 0, b, ARC_LIMM, limm=limm)
    return arc_alu(0x0A, 0, b, u6=u6, s12=s12, cc=cc)


def arc_ld(a, b, s9=0, zz=0, x=0, aa=0):
    v = s9 & 0x1FF
    return [2 << 27 | arc_b(b) | (v & 0xFF) << 16 | (v >> 8) << 15 |
            1 << 11 | aa << 9 | zz << 7 | x << 6 | a]


def arc_st(c, b, s9=0, zz=0, aa=0):
    v = s9 & 0x1FF
    return [3 << 27 | arc_b(b) | (v & 0xFF) << 16 | (v >> 8) << 15 |
            c << 6 | 1 << 5 | aa << 3 | zz << 1]


def arc_bcc(disp, cc):
    d = disp & 0x1FFFFF
    return [((d >> 1) & 0x3FF) << 17 | ((d >> 11) & 0x3FF) << 6 | cc]


def arc_bself():
    return [1 << 16]                          # b 0


def arc_bl(disp):
    d = disp & 0x1FFFFFF
    return [1 << 27 | ((d >> 2) & 0x1FF) << 18 | 1 << 17 |
            ((d >> 11) & 0x3FF) << 6 | d >> 21]


def arc_br(b, disp, cc, c=None, u6=None):
    d = disp & 0x1FF
    w = 1 << 27 | arc_b(b) | ((d >> 1) & 0x7F) << 17 | (d >> 8) << 15 | \
        1 << 16 | cc
    return [w | ((u6 << 6 | 0x10) if u6 is not None else c << 6)]


def arc_j(limm):
    return arc_alu(0x20, 0, 0, ARC_LIMM, limm=limm)


def arc_words(words):
    """An instruction is stored as halfwords, the most significant first."""
    return b''.join(struct.pack('<HH', w >> 16, w & 0xFFFF) for w in words)


class ArcProg:
    """Words at known addresses, so branches can name their targets."""
    def __init__(self, base):
        self.base, self.words, self.fix = base, [], []

    def pc(self):
        return self.base + 4 * len(self.words)

    def add(self, words):
        self.words += words

    def branch(self, make, label):
        self.fix.append((len(self.words), make, label))
        self.words += make(0)

    def link(self, labels):
        for i, make, label in self.fix:
            disp = labels[label] - ((self.base + 4 * i) & ~3)
            self.words[i:i + len(make(0))] = make(disp)
        return arc_words(self.words)


def test_fh8626_arc(qemu):
    """FH8626V100's ARC600: booted as OpenIPC's fh8626_arc.c boots it."""
    print('fh8626v100: the ARC (fh-arc)')
    entry, data = 0xA1000000, 0xA1002000
    a, L = ArcProg(entry), {}
    a.add(arc_j(entry + 0x100))                 # vector 0: reset
    a.branch(lambda d: arc_j(0), 'x')           # 0x08: memory error
    a.branch(lambda d: arc_j(0), 'x')           # 0x10: instruction error
    a.words += [0] * ((0x100 - 4 * len(a.words)) // 4)
    a.add(arc_mov(25, limm=data))
    a.add(arc_ld(0, 25))                        # runs, counted in memory
    a.add(arc_alu(0x00, 0, 0, u6=1))
    a.add(arc_st(0, 25))
    a.add(arc_mov(1, limm=0x12345678))
    a.add(arc_mov(2, s12=-5))
    a.add(arc_alu(0x00, 3, 1, 2))                       # add
    a.add(arc_alu(0x02, 4, 1, u6=8))                    # sub
    a.add(arc_alu(0x04, 5, 1, ARC_LIMM, limm=0xFF00FF00))   # and
    a.add(arc_alu(0x05, 6, 5, u6=0x3F))                 # or
    a.add(arc_alu(0x06, 7, 1, ARC_LIMM, limm=0xFFFF))   # bic
    a.add(arc_alu(0x00, 8, 1, u6=4, major=5))           # asl
    a.add(arc_alu(0x01, 9, 1, u6=8, major=5))           # lsr
    a.add(arc_alu(0x02, 10, 2, u6=1, major=5))          # asr
    a.add(arc_alu(0x03, 11, 1, u6=4, major=5))          # ror
    for n, r in enumerate(range(3, 12)):
        a.add(arc_st(r, 25, 4 + 4 * n))
    # cmp -5, 0x12345678: unsigned higher, signed less
    a.add(arc_alu(0x0C, 0, 2, 1, f=1))
    a.add(arc_mov(12, u6=0))
    a.branch(lambda d: arc_bcc(d, 0x0D), 'hi')         # bhi
    a.add(arc_mov(12, u6=0x10))
    L['hi'] = a.pc()
    a.add(arc_mov(13, u6=1, cc=0x0B))                   # mov.lt
    a.add(arc_mov(13, u6=2, cc=0x0A))                   # mov.ge: not
    a.add(arc_st(12, 25, 40))
    a.add(arc_st(13, 25, 44))
    a.branch(lambda d: arc_bl(d), 'sub')                # bl, j [blink]
    a.add(arc_st(14, 25, 48))
    a.add(arc_mov(15, u6=7))
    a.branch(lambda d: arc_br(2, d, 2, c=1), 'lt')      # brlt r2, r1
    a.add(arc_mov(15, u6=0x30))
    L['lt'] = a.pc()
    a.add(arc_st(15, 25, 52))
    # loads and stores: sizes, sign extension, post-increment
    a.add(arc_mov(16, limm=data + 0x100))
    a.add(arc_st(1, 16))
    a.add(arc_st(2, 16, 4))
    a.add(arc_st(1, 16, 8, zz=1))                       # stb
    a.add(arc_st(1, 16, 10, zz=2))                      # sth
    a.add(arc_ld(17, 16, 4, zz=1, x=1))                 # ldb.x
    a.add(arc_ld(18, 16, 2, zz=2))                      # ldh
    a.add(arc_ld(19, 16, 4, aa=2))                      # ld.ab
    a.add(arc_st(17, 25, 56))
    a.add(arc_st(18, 25, 60))
    a.add(arc_st(19, 25, 64))
    a.add(arc_st(16, 25, 68))
    a.add(arc_alu(0x2A, 0, 20, u6=4))                   # lr IDENTITY
    a.add(arc_st(20, 25, 72))
    a.add(arc_ld(20, 16, 1, aa=3))                      # ld.as [r16, 1]
    a.add(arc_ld(21, 16, 1, zz=2, aa=3))                # ldh.as [r16, 1]
    a.add(arc_st(20, 25, 84))
    a.add(arc_st(21, 25, 88))
    # faults: the handler ('x') logs ILINK2 and the bases, goes on at r28
    a.add(arc_mov(24, limm=data + 0x80))
    a.add(arc_mov(26, limm=0x40000000))         # nothing there
    a.add(arc_mov(27, limm=data))
    a.branch(lambda d: arc_mov(28, limm=entry + d), 'f1')
    L['memerr'] = a.pc()
    a.add(arc_ld(23, 26, 4, aa=2))              # ld.ab: memory error
    L['f1'] = a.pc()
    a.branch(lambda d: arc_mov(28, limm=entry + d), 'f2')
    L['rsv'] = a.pc()
    a.add(arc_ld(23, 27, 4, zz=3, aa=1))        # ld.a, reserved size
    L['f2'] = a.pc()
    # the doorbell to the ARM, then wait for the ARM's
    a.add(arc_mov(21, limm=0xF00001F0))
    a.add(arc_mov(22, u6=1))
    a.add(arc_st(22, 21))
    a.add(arc_mov(21, limm=0xF00001E4))
    L['kick'] = a.pc()
    a.add(arc_ld(23, 21))
    a.branch(lambda d: arc_br(23, d, 0, u6=0), 'kick')  # breq r23, 0
    a.add(arc_st(23, 25, 76))
    a.branch(lambda d: arc_mov(28, limm=entry + d), 'end')
    L['bad'] = a.pc()
    a.add([0x78E078E0])                         # nop_s: 16-bit, not modelled
    L['sub'] = a.pc()
    a.add(arc_mov(14, u6=0x2A))
    a.add(arc_alu(0x20, 0, 0, 31))                      # j [blink]
    L['x'] = a.pc()
    a.add(arc_st(30, 24, 4, aa=2))              # ILINK2: where it faulted
    a.add(arc_st(26, 24, 4, aa=2))
    a.add(arc_st(27, 24, 4, aa=2))
    a.add(arc_alu(0x20, 0, 0, 28))                      # j [r28]
    L['end'] = a.pc()
    a.add(arc_bself())
    # the continuations are absolute: mov r28, limm
    for i, make, label in a.fix:
        if a.words[i] >> 27 == 4:
            a.words[i + 1] = L[label]
    a.fix = [f for f in a.fix if a.words[f[0]] >> 27 != 4]
    prog = a.link(L)
    # the error vectors' long immediates: both go to 'x'
    prog = prog[:8] + arc_words(arc_j(L['x'])) * 2 + prog[24:]

    qt = QTest(qemu, 'fh8626v100')
    try:
        qt.write(entry, prog)
        qt.write(data, bytes(0x200))

        def ror16(w):
            return (w >> 16 | w << 16) & 0xFFFFFFFF
        for i, w in enumerate((0x266B7940, entry, 0x20200F80, entry)):
            qt.writel(PMU + 0x2000 + 4 * i, ror16(w))
        qt.writel(PMU + 0x2010, 1 << 4)         # run, still in reset
        qt.step(2000000)
        check('held in reset: nothing ran', qt.readl(data), 0)
        qt.writel(PMU + 0x1C4, 1 << 18)         # released
        qt.step(2000000)
        d = [qt.readl(data + 4 * i) for i in range(21)]

        def w(v):
            return struct.pack('<%dI' % len(v), *v)
        check('ran once from the PMU boot words', d[0], 1)
        check('add, sub, and, or, bic',
              w(d[1:6]), w([0x12345673, 0x12345670, 0x12005600, 0x1200563F,
                          0x12340000]))
        check('asl, lsr, asr, ror',
              w(d[6:10]), w([0x23456780, 0x00123456, 0xFFFFFFFD, 0x81234567]))
        check('cmp then bhi taken', d[10], 0)
        check('mov.lt taken, mov.ge not', d[11], 1)
        check('bl, j [blink]', d[12], 0x2A)
        check('brlt taken', d[13], 7)
        check('ldb.x, ldh, ld.ab', w(d[14:17]),
              w([0xFFFFFFFB, 0x1234, 0x12345678]))
        check('ld.ab wrote the base back', d[17], data + 0x104)
        check('stb and sth', qt.readl(data + 0x108), 0x56780078)
        check('LR IDENTITY: ARC600', d[18], 0x22)
        # from r16 = data + 0x104, after the ld.ab
        check('ld.as, ldh.as: scaled by the size (+4, +2)',
              w([qt.readl(data + 84), qt.readl(data + 88)]),
              w([0x56780078, 0xFFFF]))
        f = [qt.readl(data + 0x80 + 4 * i) for i in range(9)]
        check('ld.ab from nothing: memory error, base kept',
              w(f[0:2]), w([L['memerr'], 0x40000000]))
        check('ld.a of a reserved size: instruction error, base kept',
              w([f[3], f[5]]), w([L['rsv'], data]))
        check('doorbell 0x1F0 bit 0', qt.readl(PMU + 0x1F0), 1)
        check('raises IRQ 32', qt.readl(INTC + 0x1C) & 1, 1)
        qt.writel(PMU + 0x1F0, 0)
        check('cleared by the ARM', qt.readl(INTC + 0x1C) & 1, 0)
        check('waits for its doorbell', d[19], 0)
        qt.writel(PMU + 0x1E4, 5)
        qt.step(1000)
        check('kicked: goes on at once', qt.readl(data + 76), 5)
        check('16-bit nop_s: instruction error, ILINK2',
              qt.readl(data + 0x80 + 24), L['bad'])
        qt.writel(PMU + 0x1C4, 0)               # reset, then again
        qt.writel(PMU + 0x1C4, 1 << 18)
        qt.step(2000000)
        check('a reset runs it again from the boot words', qt.readl(data), 2)
    finally:
        qt.close()


ISP = 0xE8400000
CSI = 0xF1100000


def gc1054_write(qt, page, regs):
    i2c_xfer(qt, 0x21, [0xFE, page], 0)
    for reg, val in regs:
        i2c_xfer(qt, 0x21, [reg, val], 0)


def isp_frame_words(qt, addr, n):
    return struct.unpack('<%dI' % n, qt.read(addr, 4 * n))


def test_fh8626_isp(qemu):
    """FH8626V100's CSI -> ISP -> VPU path, programmed as OpenIPC's
    fh8626-csi.c / fh8626-isp.c / fh8626-vpu.c do it, a GC1054 in front."""
    print('fh8626v100: CSI, ISP and VPU frames, statistics (fh-isp)')
    qt = QTest(qemu, 'fh8626v100,sensor=gc1054')
    buf_a, buf_b, stat = 0xA1000000, 0xA1200000, 0xA1400000
    buf_c, w2, h2 = 0xA1600000, 320, 180         # channel 2's default
    w, h = 1280, 720
    try:
        qt.write(buf_a, bytes(w * h * 3 // 2))
        qt.write(buf_b, bytes(64))
        # GC1054: VB 160 (25 frames/s), 891 rows, 1x, MIPI on (gc1054.c)
        gc1054_write(qt, 0, [(0x07, 0), (0x08, 160), (0x03, 891 >> 8),
                             (0x04, 891 & 0xFF)])
        gc1054_write(qt, 1, [(0xB6, 0), (0xB1, 1), (0xB2, 0)])
        # ISP: geometry, statistics on, RGGB, gains 1x, VPU channel 0
        qt.writel(ISP + 0x30, (h - 1) << 16 | (w - 1))
        qt.writel(ISP + 0x24, 1 << 13)
        qt.writel(ISP + 0x168, 64 << 3)
        qt.writel(ISP + 0x224, 512 | 512 << 16)
        qt.writel(ISP + 0x228, 512 | 512 << 16)
        qt.writel(ISP + 0x1D8, stat)
        qt.writel(ISP + 0x62C, w - 1)
        qt.writel(ISP + 0x634, h - 1)
        qt.writel(ISP + 0x9B4, w | w << 16)
        qt.writel(ISP + 0x9AC, buf_a)
        qt.writel(ISP + 0x9B0, buf_a + w * h)
        qt.write(buf_c, bytes(w2 * h2 * 3 // 2))
        qt.writel(ISP + 0x958, (h2 - 1) << 16 | (w2 - 1))
        qt.writel(ISP + 0x9B4 + 24, w2 | w2 << 16)
        qt.writel(ISP + 0x9AC + 24, buf_c)
        qt.writel(ISP + 0x9B0 + 24, buf_c + w2 * h2)
        qt.writel(ISP + 0x2C, 1 << 17 | 1 << 19)    # channels 0 and 2
        qt.writel(ISP + 0x08, ~(1 << 3) & 0xFFFFFFFF)
        qt.writel(ISP + 0x18, 1)
        qt.writel(ISP + 0x68, 1)
        gc1054_write(qt, 3, [(0x10, 0x90)])
        qt.step(100000000)
        check('CSI in reset: no frames', qt.readl(ISP + 0x04), 0)
        for off in (0x08, 0x40, 0x44):             # RESETN, PHY, D-PHY
            qt.writel(CSI + off, 1)
        qt.step(1000)
        st = qt.readl(ISP + 0x04)
        check('frame start: PICTURE_START | VPU_START', st, 1 << 13 | 1 << 15)
        check('ISP IRQ 5', qt.readl(INTC + 0x18) & (1 << 5), 1 << 5)
        qt.writel(ISP + 0x04, st)
        check('write one to clear', qt.readl(INTC + 0x18) & (1 << 5), 0)
        # the next frame's address, written at the start as the driver does
        qt.writel(ISP + 0x9AC, buf_b)
        qt.writel(ISP + 0x9B0, buf_b + w * h)
        qt.step(33000000)                       # 720 of 896 rows: 32.1 ms
        check('frame end: FRAME_END | VPU_END', qt.readl(ISP + 0x04),
              1 << 0 | 1 << 2)
        y = qt.read(buf_a, w * h)
        c = qt.read(buf_a + w * h, w * h // 2)
        check('the frame went to the address latched at its start',
              qt.read(buf_b, 64), bytes(64))
        check('Y plane: picture, limited range',
              16 < sum(y[::97]) // len(y[::97]) < 235 and min(y) >= 16, True)
        check('CbCr plane written', c[0] != 0 and c[1] != 0, True)
        y2 = qt.read(buf_c, w2 * h2 * 3 // 2)
        check('channel 2, 320x180: its last row and CbCr written',
              min(y2[(h2 - 1) * w2:h2 * w2]) >= 16 and
              min(y2[w2 * h2:]) > 0, True)
        cells = isp_frame_words(qt, stat, 4 * 16)       # bank 0 (R), row 0
        g = isp_frame_words(qt, stat + 0x1000, 4)       # bank 1 (Gr)
        b = isp_frame_words(qt, stat + 0x3000, 4)       # bank 3 (B)
        check('zone cell: count per site', cells[1], (w // 16) * (h // 16) // 4)
        check('R / G at the light (0.739), %', round(100 * cells[0] / g[0]), 74)
        check('B / G at the light (0.399), %', round(100 * b[0] / g[0]), 40)
        awb = [qt.readl(ISP + 0xAE0 + 4 * i) for i in range(4)]
        check('AWB block 0: G (two sites) / R, %',
              round(100 * awb[1] / awb[0]), round(200 / 0.739))
        g0 = g[0]
        qt.writel(ISP + 0x04, 0xFFFFFFFF)
        # half the rows, written after frame 2's start event: the stats
        # published 3 events later -- frame 4's -- show it (fh8626-3ad's
        # README, measured on a GC1054 board)
        qt.step(8000000)                        # frame 2 starts
        check('40 ms frames', qt.readl(ISP + 0x04) & (1 << 13), 1 << 13)
        gc1054_write(qt, 0, [(0x03, 445 >> 8), (0x04, 445 & 0xFF)])
        qt.step(33000000)                       # frame 2 ends
        check('frame 2 at the old exposure',
              isp_frame_words(qt, stat + 0x1000, 1)[0], g0)
        check('frame 2 went to the new address',
              qt.read(buf_b, 16) != bytes(16), True)
        qt.step(40000000)                       # frame 3 ends
        check('frame 3 still at the old exposure',
              isp_frame_words(qt, stat + 0x1000, 1)[0], g0)
        qt.step(40000000)                       # frame 4 ends
        g4 = isp_frame_words(qt, stat + 0x1000, 1)[0]
        check('frame 4 at half the exposure, %', round(100 * g4 / g0), 50)
        if os.environ.get('FH_ISP_PPM'):
            with open(os.environ['FH_ISP_PPM'], 'wb') as f:
                f.write(b'P5 %d %d 255\n' % (w, h) + y)
        # a stride past the largest frame is not written (host memory)
        qt.write(buf_c, bytes(64))
        qt.writel(ISP + 0x9B4 + 24, 0xFFFF)
        qt.step(80000000)
        check('channel 2 with a 65535-byte stride: not written',
              qt.read(buf_c, 64), bytes(64))
        qt.writel(CSI + 0x08, 0)
        qt.writel(ISP + 0x04, 0xFFFFFFFF)
        qt.step(100000000)
        check('CSI back in reset: frames stop', qt.readl(ISP + 0x04), 0)
    finally:
        qt.close()


def gc2083_write(qt, regs):
    for reg, val in regs:
        i2c_xfer(qt, 0x37, [reg >> 8, reg & 0xFF, val], 0)


def isp_wait_start(qt, limit_ns, step_ns=100000):
    """Step until PICTURE_START; the ns it took, 0 past @limit_ns."""
    t = 0
    while t < limit_ns:
        qt.step(step_ns)
        t += step_ns
        if qt.readl(ISP + 0x04) & (1 << 13):
            return t
    return 0


def test_fh8626_gc2083(qemu):
    """A GC2083 in front of FH8626V100's CSI and ISP, programmed as
    OpenIPC/linux's 4.19 gc2083.c does: its frame length and line length
    set the frame period, its column gain and pregain the exposure."""
    print('fh8626v100: GC2083 (galaxycore-sensor) frames through fh-isp')
    qt = QTest(qemu, 'fh8626v100,sensor=gc2083')
    stat, w, h = 0xA1400000, 1920, 1080
    try:
        check('chip ID at 0x03F0', i2c_xfer(qt, 0x37, [0x03, 0xF0], 2),
              [0x20, 0x83])
        check('frame length at reset: 1125', i2c_xfer(qt, 0x37, [0x0D, 0x41],
                                                       2), [0x04, 0x65])
        # the reference 1080p30 timing, 300 rows, 1x; MIPI still off
        gc2083_write(qt, [(0x0D05, 0x05), (0x0D06, 0xC9),
                          (0x0D41, 1125 >> 8), (0x0D42, 1125 & 0xFF),
                          (0x0D03, 300 >> 8), (0x0D04, 300 & 0xFF),
                          (0x00B8, 0x01), (0x00B9, 0x00),
                          (0x00B1, 0x01), (0x00B2, 0x00)])
        qt.writel(ISP + 0x30, (h - 1) << 16 | (w - 1))
        qt.writel(ISP + 0x24, 1 << 13)
        qt.writel(ISP + 0x168, 64 << 3)
        qt.writel(ISP + 0x224, 512 | 512 << 16)
        qt.writel(ISP + 0x228, 512 | 512 << 16)
        qt.writel(ISP + 0x1D8, stat)
        qt.writel(ISP + 0x08, ~(1 << 3) & 0xFFFFFFFF)
        qt.writel(ISP + 0x18, 1)
        qt.writel(ISP + 0x68, 1)
        for off in (0x08, 0x40, 0x44):             # RESETN, PHY, D-PHY
            qt.writel(CSI + off, 1)
        check('MIPI output off: no frames', isp_wait_start(qt, 100000000,
                                                           10000000), 0)
        gc2083_write(qt, [(0x023E, 0x99)])
        check('stream on: a frame starts',
              isp_wait_start(qt, 50000000) > 0, True)
        qt.writel(ISP + 0x04, 0xFFFFFFFF)
        # 1125 rows of 2 x 0x5C9 clocks at 100 MHz: 33.3225 ms
        t = isp_wait_start(qt, 50000000)
        check('30 frames/s: the next start 33.3 ms on, ms', round(t / 1e6), 33)
        qt.writel(ISP + 0x04, 0xFFFFFFFF)
        g1 = isp_frame_words(qt, stat + 0x1000, 1)[0]
        # twice the frame length and 3x (column gain 2x, pregain 1.5x),
        # written at this frame's start: the frame after it takes them
        gc2083_write(qt, [(0x0D41, 2250 >> 8), (0x0D42, 2250 & 0xFF),
                          (0x00B8, 0x02), (0x00B9, 0x00),
                          (0x00B1, 0x01), (0x00B2, 32 << 2)])
        t = isp_wait_start(qt, 50000000)
        check('the frame in progress keeps its length, ms', round(t / 1e6),
              33)
        qt.writel(ISP + 0x04, 0xFFFFFFFF)
        t = isp_wait_start(qt, 100000000)
        check('the next one takes 2250 rows (66.6 ms), ms', round(t / 1e6),
              67)
        qt.writel(ISP + 0x04, 0xFFFFFFFF)
        for _ in range(3):
            isp_wait_start(qt, 100000000)
            qt.writel(ISP + 0x04, 0xFFFFFFFF)
        qt.step(70000000)
        g3 = isp_frame_words(qt, stat + 0x1000, 1)[0]
        check('statistics at 2x column gain x 1.5x pregain, %',
              round(100 * g3 / g1), 300)
        gc2083_write(qt, [(0x023E, 0x00)])
        qt.step(70000000)
        qt.writel(ISP + 0x04, 0xFFFFFFFF)
        check('MIPI output off again: frames stop',
              isp_wait_start(qt, 100000000, 10000000), 0)
    finally:
        qt.close()


def test_fh8626_gc2083_vts_delay(qemu):
    """The opt-in vts-delay: the GC2083's frame length a frame behind the
    exposure and gain written with it (a what-if, not the chip's known
    behaviour)."""
    print('fh8626v100: GC2083 with vts-delay=1')
    qt = QTest(qemu, 'fh8626v100,sensor=gc2083',
               ['-global', 'galaxycore-sensor.vts-delay=1'])
    w, h = 1920, 1080
    try:
        gc2083_write(qt, [(0x0D05, 0x05), (0x0D06, 0xC9),
                          (0x0D41, 1125 >> 8), (0x0D42, 1125 & 0xFF)])
        qt.writel(ISP + 0x30, (h - 1) << 16 | (w - 1))
        qt.writel(ISP + 0x08, ~(1 << 3) & 0xFFFFFFFF)
        qt.writel(ISP + 0x18, 1)
        qt.writel(ISP + 0x68, 1)
        for off in (0x08, 0x40, 0x44):
            qt.writel(CSI + off, 1)
        gc2083_write(qt, [(0x023E, 0x99)])
        check('stream on: a frame starts', isp_wait_start(qt, 50000000) > 0,
              True)
        qt.writel(ISP + 0x04, 0xFFFFFFFF)
        check('1125 rows, ms', round(isp_wait_start(qt, 50000000) / 1e6), 33)
        qt.writel(ISP + 0x04, 0xFFFFFFFF)
        gc2083_write(qt, [(0x0D41, 2250 >> 8), (0x0D42, 2250 & 0xFF)])
        got = []
        for _ in range(3):
            got.append(round(isp_wait_start(qt, 100000000) / 1e6))
            qt.writel(ISP + 0x04, 0xFFFFFFFF)
        check('2250 rows a frame later than without the delay',
              got == [33, 33, 67], True)
    finally:
        qt.close()


JPEG = 0xE8300000
PAE = 0xE7000000
# Annex K luminance and chrominance quantisers, natural order, and the
# zigzag scan the driver writes them in (fh8626-jpeg.c)
JPEG_ZIGZAG = [
    0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63]
JPEG_QL = [
    16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56, 14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68, 109, 103, 77, 24, 35, 55, 64, 81, 104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99]


def jpeg_picture(qt, w, h, rot, src, dst, stride=None):
    """Program a picture the way fh8626_jpeg_worker() does; its SOF0 size."""
    cw, ch = (h, w) if rot & 1 else (w, h)
    q = bytes(JPEG_QL[z] for z in JPEG_ZIGZAG) * 2
    for i in range(0, 128, 4):
        qt.writel(JPEG + 0x80 + i, struct.unpack('<I', q[i:i + 4])[0])
    qt.writel(JPEG + 0x14, ch << 16 | 0x0112)
    qt.writel(JPEG + 0x1C, cw << 16)
    qt.writel(JPEG + 0x3C, src)
    stride = stride or w
    qt.writel(JPEG + 0x40, src + stride * h)
    qt.writel(JPEG + 0x58, stride << 16 | rot << 8 | 2 << 4 | 1)
    qt.writel(JPEG + 0x08, 3)
    qt.writel(JPEG + 0x34, dst)
    qt.writel(JPEG + 0x38, dst)
    qt.writel(JPEG + 0x50, 0x10000)
    qt.writel(JPEG + 0x10, 0x80000001)
    qt.step(3000000)
    n = qt.readl(JPEG + 0x0C)
    data = qt.read(dst, n)
    i = data.find(b'\xff\xc0')
    sof = data[i + 5:i + 9] if i > 0 else b''
    return n, data, sof


def test_fh8626_jpeg(qemu):
    """FH8626V100's JPEG engine as OpenIPC's fh8626-jpeg.c drives it."""
    print('fh8626v100: the JPEG encoder (fh-jpeg)')
    qt = QTest(qemu, 'fh8626v100')
    src, dst = 0xA1000000, 0xA1100000
    w, h = 64, 32
    try:
        check('HW_ID', qt.readl(JPEG), 0x00038401)
        # a horizontal ramp, grey chroma
        qt.write(src, bytes(4 * x for x in range(w)) * h +
                 bytes([128]) * (w * h // 2))
        n, data, sof = jpeg_picture(qt, w, h, 0, src, dst)
        check('done: INT_STATUS bit 0, IRQ 8',
              (qt.readl(JPEG + 0x04), qt.readl(INTC + 0x18) >> 8 & 1) ==
              (1, 1), True)
        qt.writel(JPEG + 0x04, 1)
        check('cleared', qt.readl(INTC + 0x18) >> 8 & 1, 0)
        check('a whole JFIF: SOI ... EOI', data[:2] + data[-2:],
              b'\xff\xd8\xff\xd9')
        check('SOF0 at the coded size', sof, struct.pack('>HH', h, w))
        try:
            from PIL import Image
            import io
            im = Image.open(io.BytesIO(data)).convert('L')
            row = [im.getpixel((x, 16)) for x in (2, 32, 61)]
            check('decodes, the ramp rising left to right',
                  im.size == (w, h) and row[0] < row[1] < row[2], True)
        except ImportError:
            print('  SKIP: decoding (no PIL)')
        n, data, sof = jpeg_picture(qt, w, h, 1, src, dst)
        check('rotated 90 degrees: SOF0 turned', sof,
              struct.pack('>HH', w, h))
        try:
            from PIL import Image
            import io
            im = Image.open(io.BytesIO(data)).convert('L')
            col = [im.getpixel((16, y)) for y in (2, 32, 61)]
            check('the ramp runs top to bottom', col[0] < col[1] < col[2],
                  True)
        except ImportError:
            pass
        # 180 degrees from rows padded to 96 bytes with white: the padding
        # stays out of the picture
        st = 96
        qt.write(src, (bytes(4 * x for x in range(w)) + b'\xff' * (st - w))
                 * h + (bytes([128]) * w + b'\xff' * (st - w)) * (h // 2))
        n, data, sof = jpeg_picture(qt, w, h, 2, src, dst, stride=st)
        try:
            from PIL import Image
            import io
            im = Image.open(io.BytesIO(data)).convert('L')
            row = [im.getpixel((x, 16)) for x in (2, 32, 61)]
            check('180 degrees from padded rows: the ramp falling, no '
                  'padding', row[0] > row[1] > row[2] and row[0] < 250, True)
        except ImportError:
            check('180 degrees from padded rows: a whole JFIF',
                  data[:2] + data[-2:], b'\xff\xd8\xff\xd9')
    finally:
        qt.close()


def pae_bits(fields):
    """An unescaped bit string from (value, width) and ('ue', v) fields."""
    bits = ''
    for f in fields:
        if f[0] == 'ue':
            v = f[1] + 1
            bits += '0' * (v.bit_length() - 1) + format(v, 'b')
        elif f[0] == 'se':
            v = f[1]
            v = 2 * v - 1 if v > 0 else -2 * v
            v += 1
            bits += '0' * (v.bit_length() - 1) + format(v, 'b')
        else:
            bits += format(f[0], '0%db' % f[1])
    bits += '1' * (-len(bits) % 8)                  # cabac_alignment_one_bit
    return bytes(int(bits[i:i + 8], 2) for i in range(0, len(bits), 8))


def pae_picture(qt, idr, hdr, src, w, h, ring, ring_bytes, qp=30):
    """Program a picture the way program_frame() does; the payload."""
    bus = 0xA0000000
    p = bytearray(0x1BC)
    struct.pack_into('<HH', p, 4, w, h)
    p[0x0C] = p[0x0D] = qp
    p[0xC3] = 2 if idr else 0
    struct.pack_into('<II', p, 0x134, (src - bus) >> 10,
                     (src + w * h - bus) >> 10)
    struct.pack_into('<I', p, 0x140, 0x1234)           # the context
    struct.pack_into('<I', p, 0x14C, (ring - bus) >> 10)
    struct.pack_into('<H', p, 0x15A, ring_bytes >> 10)
    qt.write(0x10007E00, bytes(p))
    qt.writel(PAE + 0x08, 3)
    qt.writel(PAE + 0x0C, 0x3F)
    qt.writel(PAE + 0x3000 + 0xD0, 0x01011000)
    for i in range(0, len(hdr), 4):
        chunk = hdr[i:i + 4]
        qt.writel(PAE + 0x3000 + 0xB0, int.from_bytes(chunk, 'big'))
        qt.writel(PAE + 0x3000 + 0xD0, 2 * 8 * len(chunk) - 1)
    qt.writel(PAE + 0x3000 + 0xD0, 0x10000000)
    qt.step(11000000)
    bits = qt.readl(PAE + 0x3000 + 0x110)
    return qt.readl(PAE + 0x08), bits, qt.read(ring + 16, (bits + 7) // 8)


def test_fh8626_pae(qemu):
    """FH8626V100's PAE H.264 engine as OpenIPC's fh8626-pae-hw.c drives
    it; ffmpeg decodes what it makes in test-fullhan-boot.sh."""
    print('fh8626v100: the H.264 encoder (fh-pae)')
    qt = QTest(qemu, 'fh8626v100')
    src, ring = 0xA1000000, 0xA1100000
    w, h = 64, 32
    try:
        check('engine reset acknowledged',
              (qt.writel(PAE + 0x18, 1 << 16), qt.readl(PAE + 0x20))[1], 1)
        qt.write(src, bytes((x * 3 + y) & 0xFF for y in range(h)
                            for x in range(w)) + bytes([128]) * (w * h // 2))
        # first_mb 0, IDR (7), PPS 0, frame_num, idr_pic_id 0, POC, the two
        # IDR flags, slice_qp_delta (QP 30 on pic_init_qp 26)
        idr_hdr = pae_bits([('ue', 0), ('ue', 7), ('ue', 0), (0, 16),
                            ('ue', 0), (0, 16), (0, 2), ('se', 4)])
        st, bits, out = pae_picture(qt, True, idr_hdr, src, w, h, ring,
                                    64 * 1024)
        check('IDR: SLICE done, IRQ 4',
              st & 1 == 1 and qt.readl(INTC + 0x18) >> 4 & 1 == 1, True)
        check('the pushed header leads the payload',
              out[:len(idr_hdr)], idr_hdr)
        check('slice data follows it', len(out) > len(idr_hdr) + 8, True)
        check('emulation prevention: no 00 00 0x (x <= 3) left',
              any(out[i:i + 2] == b'\0\0' and out[i + 2] <= 3
                  for i in range(len(out) - 2)), False)
        idr_len = len(out)
        # P, frame_num 1, POC 2; the same picture: it codes much smaller
        p_hdr = pae_bits([('ue', 0), ('ue', 5), ('ue', 0), (1, 16),
                          (2, 16), (0, 2), (0, 1), ('ue', 0), ('se', 4)])
        st, bits, out = pae_picture(qt, False, p_hdr, src, w, h, ring,
                                    64 * 1024)
        check('P from the IDR\'s reconstruction: smaller', len(out) <
              idr_len // 2 and out[:len(p_hdr)] == p_hdr, True)
        # noise at QP 0, 256x128: far more than a 1 KiB ring
        qt.write(src, bytes((i * 7919 + (i >> 5) * 104729) & 0xFF
                            for i in range(256 * 128 * 3 // 2)))
        qt.write(ring + 1024, b'\xa5' * 64)
        st, bits, out = pae_picture(qt, True, idr_hdr, src, 256, 128, ring,
                                    1024, qp=0)
        check('a 1 KiB ring: more bits reported than it holds (-ENOSPC)',
              bits > (1024 - 16) * 8, True)
        check('and nothing written past it', qt.read(ring + 1024, 64),
              b'\xa5' * 64)
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
    ('fh8856v100', 0x17092901, 0x004, 0x7840D),
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
    test_gmac_rx_start(args.qemu)
    test_long_header(args.qemu, os.path.join(tmp, 'flash-long.bin'))
    test_fh8652(args.qemu)
    test_fh8626(args.qemu, path)
    test_fh8626_arc(args.qemu)
    test_fh8626_isp(args.qemu)
    test_fh8626_gc2083(args.qemu)
    test_fh8626_gc2083_vts_delay(args.qemu)
    test_fh8626_jpeg(args.qemu)
    test_fh8626_pae(args.qemu)
    test_fh8632(args.qemu)
    test_all_machines(args.qemu)
    test_migration(args.qemu, tmp)
    print('%s: %d failure(s)' % ('FAIL' if failures else 'PASS', failures))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
