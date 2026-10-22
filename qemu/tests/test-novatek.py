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
  - nvt-flash, the way nvt_flash_spi_cmd.c drives it: the JEDEC ID by
    PIO, a manual-CS read (CMD_ADDR then READ_N), a DMA read into RAM, a
    block erase with its RDSR poll and a PIO page program, both written
    back to the image file, and the completion interrupt on SPI 29;
  - nvt-flash in SPI NAND mode, the way spinand.c drives a W25N01GV, on
    a 128 MiB image of data alone and on one with each page's OOB: the
    ID after its address byte, GET / SET_FEATURE, DATA_RD3 page reads (by
    DMA, the OOB into the spare SRAM, and the OOB alone), a block erase
    and an SPI_PROG_CMD page program polled through GET_FEATURE C0, the
    image file afterwards; a malformed image size is refused;
  - nvt-sdio with an SD card, the way na51055_mmc.c and U-Boot drive it:
    card init (CMD0/8/55/41/2/3/7, R2 in RSP3..0 as the drivers read
    it), single- and multi-block reads by descriptor-table DMA (Linux)
    and by plain DMA with 0x1FC set (U-Boot), a DMA write, PIO reads and
    writes through the data port with FIFO_STS full / count / empty and
    DATA_END after the last word, the 8-byte SCR by PIO, the status
    interrupt on SPI 30; the CG reset pulse (0x84 bit 2) returning the
    host to descriptor mode; card detect on C_GPIO 9; an empty slot
    timing out; SDIO3 with an eMMC: CMD1 / 2 / 3 / 7, the 512-byte
    EXT_CSD by descriptor DMA, and its CG reset bit (0x84 bit 14);
  - nvt-usb, the FOTG200 host as ehci-na51089.c sees it: EHCI
    capabilities (CAPLENGTH 0x10, one port) at 0xF0600000 and at the
    0xFF600000 alias the driver uses on NA51089, PORTSC at 0x30 showing a
    usb-storage stick, the OTG register's [23:22] speed (high), the glue
    registers and PHY as storage, a port reset enabling the port; with no
    device, no connect and speed 0;
  - nvt-i2c, the way i2c-nvt.c drives it, against an SC401AI on I2C 0
    (sensor=sc401ai): the chip ID ipcinfo reads (a register-address write,
    then a repeated-START read), a write and a read that each span two
    four-byte chunks, the DT / DR / BERR status and SPI 41, a NACK from an
    empty address followed by the driver's lone STOP, an i2cdetect-style
    quick write, and the empty second and third controllers;
  - nvt-pwm, the way pwm-na51055.c drives it: PERIOD and its extension,
    ENABLE starting a channel (and its "out" line, high with a nonzero
    duty), DISABLE stopping it with INT_STS and SPI 26, a cycle-count run
    ending on its own after cycles x base period at 30 MHz, a divider load
    flagging CLKDIV_LOAD_STS;
  - nvt-timer, the way timer_cfg() and U-Boot drive it: timer 0
    running free at 1 MHz out of reset, a one-shot timer counting to its
    target, flagging STS and SPI 0 and stopping, a free-running one on
    divider 1 wrapping and going on, a divider change mid-period (and
    repeated ones at 600 kHz keeping every tick), 3 MHz periods without
    drift, a switch to one shot mid-period, RELOAD,
    a disabled timer holding its count, STS without INT raising nothing,
    and a machine reset restoring timer 0;
  - nvt-crypto, the way na51089_crypto.c and U-Boot drive it: a PIO
    AES block (FIPS-197), and by descriptor DMA AES-CBC and CTR
    (SP 800-38A; CV written back, a partial last block, a block split
    across two descriptor entries), GCM (the McGrew
    and Viega cases; the tag from S0 and GHASH), DES and 3DES, the OTP
    key manager's (blank, zero) key and "get S0", with SPI 38;
  - nvt-hash: SHA-1 / SHA-256 padded by the engine, a message in two
    chunks chained through the IV registers, HMAC-SHA-1 / -256 (RFC 2202,
    4231), an unreadable chunk leaving no digest, SPI 28;
  - nvt-rsa: 256, 512 and 2048-bit modular exponentiation through the
    SRAM ports, against Python's pow(), SPI 20; a CRC32 key check never
    reporting done;
  - nvt-media, the HDAL engines' handshakes: SIE frames in three steps
    (VD, break points, ends) with single-mode outputs completing; an IME
    linked list of two jobs, its updates applied with their byte enables,
    frame / job / list end and SPI 6; the graphics engine's list end (no
    DMA idle), single job and reset; ISE's list; H.26x flags, reset and a
    picture (BS_LEN, a filler NAL); a JPEG of four grey MCUs; the EAC
    strobes; the DAI's capture DMA (load, buffer full with silence, stop);
  - nvt-otp: power-up and ready, a read starting, finishing and reading a
    blank word, the done bit cleared by writing it back;
  - the flash boot ROM: a synthetic NVT image (device tree at 0x10000,
    BCL1/LZ-packed "U-Boot") leaves U-Boot unpacked at
    /nvt_memory_cfg/uboot, the device tree and the "LD_NVT" SHMINFO where
    the tree says, and a branch at 0; an image with no tree is refused;
    a NAND image (device tree in block 2, a plain U-Boot) boots the same
    way, its SHMINFO naming SPI NAND;
  - nvt-eqos: the embedded PHY over MDIO, the HW feature bits the driver
    keys on (MDIO present, one queue), the software reset, the Rx DMA
    state yinit() polls for, a frame sent through a Tx descriptor (seen in
    a filter-dump pcap) with its write-back and interrupt, and the
    Novatek ETH_ISR summary the ISR checks first; a frame split across two
    tail writes, an over-long one dropped with ES / JT; frames injected
    through a socket netdev: unicast filtering, a frame spanning 64-byte
    buffers, Rx stopping at its tail pointer until it moves;
  - the 16550 UART at its reg-shift-2 offsets, and input that arrived
    before the guest set the UART up still flowing after it resets the
    receive FIFO (qemu/patches/0003, the vendor U-Boot's autoboot stop);
  - the generated device tree: compatible, the clock nodes in the order
    of_clk_init() needs, the pinmux children in the driver's order; the
    flash node spinand.c binds when the image is a NAND;
  - outside qtest: machine state migrates to a file and back.

    python3 qemu/tests/test-novatek.py [--qemu PATH]
"""
import argparse
import hashlib
import lzma
import os
import random
import re
import shutil
import socket
import struct
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
FLASH = 0xF0400000
ETH = 0xF02B0000
GICD = 0xFFD01000

SPI_GPIO = 24
SPI_FLASH = 29
SPI_ETH = 34
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

    def read(self, a, n):
        return bytes.fromhex(self.cmd('read 0x%x %d' % (a, n)).split()[1][2:])

    def write(self, a, data):
        self.cmd('write 0x%x %d 0x%s' % (a, len(data), data.hex()))

    def step(self, ns):
        self.cmd('clock_step %d' % ns)

    def pin(self, n, level):
        self.cmd('set_irq_in /machine/nvt-gpio in %d %d' % (n, level))

    def close(self):
        self.proc.kill()
        self.proc.wait()
        self.sock.close()
        shutil.rmtree(self.dir, ignore_errors=True)


failures = 0


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    if isinstance(got, bytes):
        shown = got[:8].hex() + ('...' if len(got) > 8 else '')
    else:
        shown = got if isinstance(got, (bool, str, tuple, list)) else hex(got)
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


# ---- nvt-flash ---------------------------------------------------------

OP_READ_ID, OP_CMD, OP_CMD_ADDR, OP_BLK_ERSE = 0x06, 0x07, 0x0C, 0x05
OP_READ_N, OP_WRITE_N = 0x18, 0x19


def flash_op(qt, op, cmd, row=0, length=0, dma=False, write=False):
    qt.writel(FLASH + 0x48, 1 << 12)            # clear COMP
    qt.writel(FLASH + 0x34, cmd)
    qt.writel(FLASH + 0x3C, row)
    qt.writel(FLASH + 0x104, length)
    qt.writel(FLASH + 0x10C, 1 | (2 if dma else 0) | (4 if write else 0))
    qt.writel(FLASH + 0x20, op | (1 << 12))


def flash_done(qt):
    return bool(qt.readl(FLASH + 0x48) & (1 << 12))


def test_flash(qt, path):
    print('nvt-flash')
    img = open(path, 'rb').read()
    gic_enable(qt, SPI_FLASH)
    qt.writel(FLASH + 0x44, 1 << 12)            # completion interrupt
    qt.writel(FLASH + 0x00, 2 << 2)             # 3 row-address bytes

    flash_op(qt, OP_READ_ID, 0x9F, length=4)
    check('READ_ID completes', flash_done(qt), True)
    check('SPI 29 pending', gic_pending(qt, SPI_FLASH), True)
    check('JEDEC ID through the FIFO (W25Q64 for 8 MiB)',
          qt.readl(FLASH + 0x100) & 0xFFFFFF, 0x1740EF)
    qt.writel(FLASH + 0x48, 1 << 12)
    check('status write-back acknowledges', gic_pending(qt, SPI_FLASH),
          False)

    # spiflash_read_data: CS manual and low across CMD_ADDR + READ_N
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    flash_op(qt, OP_CMD_ADDR, 0x03, row=0x1234)
    flash_op(qt, OP_READ_N, 0x03, length=16)
    words = b''.join(qt.readl(FLASH + 0x100).to_bytes(4, 'little')
                     for _ in range(4))
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    check('PIO read under manual CS', words, img[0x1234:0x1244])

    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    flash_op(qt, OP_CMD_ADDR, 0x03, row=0x20000)
    qt.writel(FLASH + 0x110, 0x01000000)
    flash_op(qt, OP_READ_N, 0x03, length=256, dma=True)
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    got = b''.join(qt.readl(0x01000000 + i).to_bytes(4, 'little')
                   for i in range(0, 256, 4))
    check('DMA read into RAM', got, img[0x20000:0x20100])

    # spinor_erase_sector: WREN, then BLK_ERSE (cmd, address, RDSR polled)
    flash_op(qt, OP_CMD, 0x06)
    qt.writel(FLASH + 0xBC, (1 << 8) | 0)       # mask WIP, want 0
    flash_op(qt, OP_BLK_ERSE, 0xD8 | (0x05 << 8), row=0x30000)
    check('block erase completes', flash_done(qt), True)
    # spiflash_page_program, PIO: WREN, CMD_ADDR, WRITE_N, data port
    flash_op(qt, OP_CMD, 0x06)
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    flash_op(qt, OP_CMD_ADDR, 0x02, row=0x30010)
    flash_op(qt, OP_WRITE_N, 0x02, length=8, write=True)
    check('a PIO write waits for its data', flash_done(qt), False)
    qt.writel(FLASH + 0x100, 0x44332211)
    qt.writel(FLASH + 0x100, 0x88776655)
    check('and completes with it', flash_done(qt), True)
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    img = open(path, 'rb').read()
    check('erase reached the image file',
          img[0x30000:0x30010] + img[0x30018:0x40000] ==
          b'\xff' * (0x10000 - 8), True)
    check('program reached the image file', img[0x30010:0x30018],
          bytes(range(0x11, 0x99, 0x11)))

    check('past the registers reads 0', qt.readl(FLASH + 0x300), 0)
    flash_op(qt, OP_WRITE_N, 0x02, length=0, write=True)
    check('a zero-length PIO write completes', flash_done(qt), True)
    check('and interrupts', gic_pending(qt, SPI_FLASH), True)
    qt.writel(FLASH + 0x48, 1 << 12)

    # A PIO read longer than the 4 KiB buffer streams through it
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    flash_op(qt, OP_CMD_ADDR, 0x03, row=0x50000)
    flash_op(qt, OP_READ_N, 0x03, length=0x2000)
    check('an 8 KiB PIO read is not complete before it is drained',
          flash_done(qt), False)
    got = b''.join(qt.readl(FLASH + 0x100).to_bytes(4, 'little')
                   for _ in range(0x800))
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    check('8 KiB PIO read', got, img[0x50000:0x52000])
    check('and completes with its last word', flash_done(qt), True)

    # FAST_READ with a dummy setting under a byte of clocks
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    flash_op(qt, OP_CMD_ADDR, 0x0B, row=0x60000)
    qt.writel(FLASH + 0xC0, 3)
    flash_op(qt, 0x1E, 0x0B)
    flash_op(qt, OP_READ_N, 0x0B, length=16)
    got = b''.join(qt.readl(FLASH + 0x100).to_bytes(4, 'little')
                   for _ in range(4))
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    check('FAST_READ after four dummy clocks', got, img[0x60000:0x60010])


def nor_cmd(qt, cmd, row=None, read=0, data=None):
    """One SPI NOR command under manual CS: cmd [address] [data | read]."""
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 1 << 1)
    if row is None:
        flash_op(qt, OP_CMD, cmd)
    else:
        flash_op(qt, OP_CMD_ADDR, cmd, row=row)
    got = None
    if data is not None:
        flash_op(qt, OP_WRITE_N, cmd, length=1, write=True)
        qt.writel(FLASH + 0x100, data)
    elif read:
        flash_op(qt, OP_READ_N, cmd, length=read)
        got = qt.readl(FLASH + 0x100) & 0xFF
    qt.writel(FLASH + 0x14, (1 << 1) | (1 << 4))
    qt.writel(FLASH + 0x14, 0)
    return got


def test_flash_locks(qt, path):
    """W25Q per-block protection, as the NT98332 (Xiongmai) kernel uses it:
    SR3's WPS on, lock bits powering up set, 0x39 to unlock what it writes."""
    print('nvt-flash: W25Q block locks')
    qt.writel(FLASH + 0x00, 2 << 2)             # 3 row-address bytes

    def erase(row):
        flash_op(qt, OP_CMD, 0x06)
        qt.writel(FLASH + 0xBC, (1 << 8) | 0)
        flash_op(qt, OP_BLK_ERSE, 0xD8 | (0x05 << 8), row=row)

    def erased(row):
        return open(path, 'rb').read()[row:row + 0x10000] == \
            b'\xff' * 0x10000

    def locked(row):
        return nor_cmd(qt, 0x3D, row=row, read=1) & 1

    check('SR3 out of reset: 0x60, WPS clear', nor_cmd(qt, 0x15, read=1),
          0x60)
    erase(0x30000)
    check('WPS clear: the locks do not apply', erased(0x30000), True)

    flash_op(qt, OP_CMD, 0x06)
    nor_cmd(qt, 0x11, data=0x64)
    check('WRSR3 sets WPS', nor_cmd(qt, 0x15, read=1), 0x64)
    check('the lock bits power up set', locked(0x40000), 1)
    erase(0x40000)
    check('an erase of a locked block is dropped', erased(0x40000), False)

    flash_op(qt, OP_CMD, 0x06)
    nor_cmd(qt, 0x39, row=0x40000)
    check('0x39 unlocks the block', locked(0x40000), 0)
    check('all of its 64 KiB', locked(0x4F000), 0)
    check('and no more', locked(0x50000), 1)
    erase(0x40000)
    check('then the erase lands', erased(0x40000), True)

    flash_op(qt, OP_CMD, 0x06)
    nor_cmd(qt, 0x39, row=0x7F0000)
    check('the top block unlocks per 4 KiB sector', locked(0x7F0000), 0)
    check('its next sector stays locked', locked(0x7F1000), 1)

    flash_op(qt, OP_CMD, 0x06)
    nor_cmd(qt, 0x36, row=0x40000)
    check('0x36 locks it again', locked(0x40000), 1)
    nor_cmd(qt, 0x39, row=0x50000)
    check('an unlock without WREN is ignored', locked(0x50000), 1)
    flash_op(qt, OP_CMD, 0x06)
    nor_cmd(qt, 0x98)
    check('0x98 unlocks everything', (locked(0x50000), locked(0x7F1000)),
          (0, 0))


# ---- nvt-flash, SPI NAND --------------------------------------------------

OP_CMD_STS, OP_SPI_PROG_CMD, OP_DATA_RD3 = 0x0B, 0x15, 0x16
NAND_MODE = 1 << 19                 # MODULE0.nand_type, spi_flash_type 0
PAGE, OOB, RAW = 2048, 64, 2112
NAND_SIZE = 128 << 20


def make_nand(path, with_oob):
    """A 128 MiB NAND: pages 0..191 numbered, page 5's OOB (if kept) set."""
    data = bytearray(b'\xff' * NAND_SIZE)
    for p in range(192):
        data[p * PAGE:(p + 1) * PAGE] = (struct.pack('<I', p) * 512)
    if not with_oob:
        open(path, 'wb').write(data)
        return
    raw = bytearray(b'\xff' * (NAND_SIZE // PAGE * RAW))
    for p in range(192):
        raw[p * RAW:p * RAW + PAGE] = data[p * PAGE:(p + 1) * PAGE]
    raw[5 * RAW + PAGE:6 * RAW] = bytes(range(0x40, 0x80))
    open(path, 'wb').write(raw)


def nand_page(path, with_oob, p):
    """(data, oob) of page p in the image file."""
    with open(path, 'rb') as f:
        if with_oob:
            f.seek(p * RAW)
            b = f.read(RAW)
            return b[:PAGE], b[PAGE:]
        f.seek(p * PAGE)
        return f.read(PAGE), None


def nand_module0(qt, row, col=2):
    qt.writel(FLASH + 0x00, NAND_MODE | ((row - 1) << 2) | (col - 1))


def nand_read_page(qt, page, col=0, length=PAGE, dma=True):
    """spinand_read_page: PAGE_READ, poll, CACHE_READX4, status."""
    nand_module0(qt, 3)
    qt.writel(FLASH + 0x38, col)
    qt.writel(FLASH + 0xB4, 0x0F | (0xC0 << 8))     # COMMAND2: cmd 5, 6
    qt.writel(FLASH + 0xBC, 1 << 8)                 # OIP == 0
    qt.writel(FLASH + 0x110, 0x01000000)
    flash_op(qt, OP_DATA_RD3, 0x13 | (0x0F << 8) | (0xC0 << 16) | (0x6B << 24),
             row=page, length=length, dma=dma)


def nand_feature(qt, addr):
    nand_module0(qt, 1)
    flash_op(qt, OP_CMD_STS, 0x0F | (addr << 8), length=1)
    return qt.readl(FLASH + 0x48) & 0xFF


def test_nand(qt, path, with_oob):
    print('nvt-flash, SPI NAND (%s)' % ('with OOB' if with_oob else
                                        'data only'))
    gic_enable(qt, SPI_FLASH)
    qt.writel(FLASH + 0x44, 1 << 12)

    nand_module0(qt, 1)
    flash_op(qt, OP_READ_ID, 0x9F, length=4)
    check('READ_ID: W25N01GV after the address byte',
          qt.readl(FLASH + 0x100), 0x0021AAEF)
    check('configuration B0: on-die ECC, buffer mode', nand_feature(qt, 0xB0),
          0x18)

    # spi_nand_unlock_bp / nand_cmd_write_status: CMD_ADDR, 2 "row" bytes
    nand_module0(qt, 2)
    flash_op(qt, OP_CMD_ADDR, 0x1F, row=(0xA0 << 8) | 0x38)
    check('SET_FEATURE A0', nand_feature(qt, 0xA0), 0x38)
    nand_module0(qt, 2)
    flash_op(qt, OP_CMD_ADDR, 0x1F, row=(0xA0 << 8) | 0x00)
    check('and back to unlocked', nand_feature(qt, 0xA0), 0)

    data, oob = nand_page(path, with_oob, 5)
    nand_read_page(qt, 5)
    check('DATA_RD3 completes', flash_done(qt), True)
    check('and interrupts', gic_pending(qt, SPI_FLASH), True)
    check('page 5 by DMA', qt.read(0x01000000, PAGE), data)
    check('its OOB in the spare SRAM', qt.read(FLASH + 0x200, OOB),
          oob or b'\xff' * OOB)
    check('status (cmd 5 + 6) in CTRL_STS: ready, no ECC error',
          qt.readl(FLASH + 0x48) & 0xFF, 0)
    qt.writel(FLASH + 0x48, 1 << 12)

    qt.write(FLASH + 0x200, b'\0' * OOB)
    nand_read_page(qt, 5, col=PAGE, length=OOB, dma=False)
    check('a spare-only read completes undrained', flash_done(qt), True)
    check('with the OOB in the spare SRAM', qt.read(FLASH + 0x200, OOB),
          oob or b'\xff' * OOB)

    # A column past the OOB reads nothing, whatever DATALEN says
    qt.write(FLASH + 0x200, b'\x5a' * 0x80)
    nand_read_page(qt, 5, col=0xFFF, length=0x100000, dma=False)
    check('a column past the page completes', flash_done(qt), True)
    check('and leaves the spare SRAM alone', qt.read(FLASH + 0x200, 0x80),
          b'\x5a' * 0x80)
    check('the controller still answers', nand_feature(qt, 0xB0), 0x18)

    # RESET returns B0 to its power-on value
    nand_module0(qt, 2)
    flash_op(qt, OP_CMD_ADDR, 0x1F, row=(0xB0 << 8) | 0x08)
    check('SET_FEATURE B0: ECC off', nand_feature(qt, 0xB0), 0x08)
    flash_op(qt, OP_CMD, 0xFF)
    check('RESET restores ECC-E', nand_feature(qt, 0xB0), 0x18)

    # nand_cmd_erase_block: WEN, then BLK_ERSE polling GET_FEATURE C0
    nand_module0(qt, 1)
    flash_op(qt, OP_CMD, 0x06)
    check('WEN sets WEL', nand_feature(qt, 0xC0), 0x02)
    nand_module0(qt, 3)
    qt.writel(FLASH + 0xBC, 1 << 8)
    flash_op(qt, OP_BLK_ERSE, 0xD8 | (0x0F << 8) | (0xC0 << 16), row=64)
    check('block erase completes', flash_done(qt), True)
    check('with E_FAIL clear and WEL dropped',
          qt.readl(FLASH + 0x48) & 0xFF, 0)
    check('block 1 erased in the file',
          all(nand_page(path, with_oob, p)[0] == b'\xff' * PAGE
              for p in (64, 100, 127)), True)
    check('block 2 untouched', nand_page(path, with_oob, 128)[0],
          struct.pack('<I', 128) * 512)

    # spinand_program_page: spare SRAM, then SPI_PROG_CMD by DMA
    payload = random.Random(5).randbytes(PAGE)
    spare = bytes(range(0xC0, 0x100))
    qt.write(0x01000000, payload)
    qt.write(FLASH + 0x200, spare)
    nand_module0(qt, 3)
    qt.writel(FLASH + 0x38, 0)
    qt.writel(FLASH + 0xB4, 0xC0)                   # COMMAND2: cmd 5
    qt.writel(FLASH + 0xBC, 1 << 8)
    qt.writel(FLASH + 0x110, 0x01000000)
    flash_op(qt, OP_SPI_PROG_CMD, 0x06 | (0x32 << 8) | (0x10 << 16) |
             (0x0F << 24), row=67, length=PAGE, dma=True, write=True)
    check('SPI_PROG_CMD completes', flash_done(qt), True)
    check('with P_FAIL clear', qt.readl(FLASH + 0x48) & 0x08, 0)
    d, o = nand_page(path, with_oob, 67)
    check('page 67 programmed in the file', d, payload)
    if with_oob:
        check('its OOB from the spare SRAM', o, spare)
    qt.write(0x01000000, b'\0' * PAGE)
    nand_read_page(qt, 67)
    check('and reads back', qt.read(0x01000000, PAGE), payload)

    # PROGRAM EXECUTE without WEN programs nothing (W25N)
    nand_module0(qt, 3)
    flash_op(qt, OP_CMD_ADDR, 0x10, row=68)
    check('no program without WEL', nand_page(path, with_oob, 68)[0],
          b'\xff' * PAGE)


# ---- nvt-sdio --------------------------------------------------------------

SDIO = 0xF0420000
SDIO2 = 0xF0500000
SPI_SDIO = 30
SD_CMD_SENT, SD_RSP_OK, SD_RSP_TIMEOUT = 0x40, 0x10, 0x04
SD_DATA_END, SD_DATA_OK = 0x80, 0x20


def sd_cmd(qt, idx, arg=0, rsp='short', base=SDIO):
    """sdiohost_sendcmd: clear the status, ARG, CMD, then CMD_EN."""
    qt.writel(base + 0x28, 0xFFFFFFFF)
    qt.writel(base + 0x04, arg)
    v = idx | {'none': 0, 'short': 1 << 6, 'long': 1 << 7}[rsp]
    qt.writel(base, v)
    qt.writel(base, v | (1 << 9))
    return qt.readl(base + 0x28)


def sd_data(qt, length, blksz=512, dma=True, write=False, addr=0,
            base=SDIO):
    """sdiohost_setupdatatransfer{dma,pio}: before the command."""
    qt.writel(base + 0x1C, (blksz << 16))
    qt.writel(base + 0x110, addr)
    qt.writel(base + 0x104, length)
    fifo = (2 if dma else 0) | (4 if write else 0)
    qt.writel(base + 0x10C, fifo)
    qt.writel(base + 0x1C, (blksz << 16) | (1 << 6))
    qt.writel(base + 0x10C, fifo | 1)


SDIO3 = 0xF0510000


def test_emmc(qt):
    print('nvt-sdio: SDIO3 with an eMMC')
    qt.writel(SDIO3 + 0x30, 0xFF)
    sd_cmd(qt, 0, rsp='none', base=SDIO3)
    for _ in range(10):                             # CMD1 until not busy
        sd_cmd(qt, 1, 0x40FF8080, base=SDIO3)
        if qt.readl(SDIO3 + 0x08) >> 31:
            break
    check('CMD1 (SEND_OP_COND): powered up (OCR bit 31)',
          qt.readl(SDIO3 + 0x08) >> 31, 1)
    sd_cmd(qt, 2, rsp='long', base=SDIO3)
    sd_cmd(qt, 3, 1 << 16, base=SDIO3)              # the host picks the RCA
    sd_cmd(qt, 7, 1 << 16, base=SDIO3)
    qt.write(0x00800000, struct.pack('<3I', 1 | 2 | (2 << 4), 512, 0x01000000))
    sd_data(qt, 512, addr=0x00800000, base=SDIO3)
    sts = sd_cmd(qt, 8, 0, base=SDIO3)
    ext = qt.read(0x01000000, 512)
    check('CMD8 (SEND_EXT_CSD) by descriptor DMA', sts & SD_DATA_END,
          SD_DATA_END)
    check('EXT_CSD: a revision, and SEC_COUNT for 4 MiB',
          (ext[192] > 0, struct.unpack_from('<I', ext, 212)[0]),
          (True, (4 << 20) // 512))
    qt.writel(SDIO3 + 0x1FC, 1)
    qt.writel(CG + 0x84, qt.readl(CG + 0x84) & ~(1 << 14))
    qt.writel(CG + 0x84, qt.readl(CG + 0x84) | (1 << 14))
    check('CG 0x84 bit 14 resets SDIO3', qt.readl(SDIO3 + 0x1FC), 0)


def test_sdio(qt, card):
    print('nvt-sdio')
    img = open(card, 'rb').read()
    gic_enable(qt, SPI_SDIO)
    check('BUS_STATUS: card not busy', qt.readl(SDIO + 0x40) & 1, 1)
    check('card detect: C_GPIO 9 low with a card in',
          qt.readl(GPIO + 0x00) >> 9 & 1, 0)
    qt.writel(SDIO, 1 << 10)
    check('SDC_RST clears itself', qt.readl(SDIO) & (1 << 10), 0)
    qt.writel(SDIO + 0x30, 0xFF)

    check('CMD0: CMD_SENT', sd_cmd(qt, 0, rsp='none'), SD_CMD_SENT)
    check('and SPI 30', gic_pending(qt, SPI_SDIO), True)
    check('CMD8: R7 echoes the check pattern',
          (sd_cmd(qt, 8, 0x1AA), qt.readl(SDIO + 0x08) & 0xFFF),
          (SD_CMD_SENT | SD_RSP_OK, 0x1AA))
    sd_cmd(qt, 55)
    sd_cmd(qt, 41, 0x40FF8000)
    check('ACMD41: powered up', qt.readl(SDIO + 0x08) >> 31, 1)
    sd_cmd(qt, 2, rsp='long')
    check('CMD2: CID MID in RSP3[31:24] (resp[0])',
          qt.readl(SDIO + 0x14) >> 24, 0xAA)
    check('and the end bit in RSP0[0]', qt.readl(SDIO + 0x08) & 1, 1)
    sd_cmd(qt, 3)
    rca = qt.readl(SDIO + 0x08) >> 16
    check('CMD3: an RCA', rca != 0, True)
    sd_cmd(qt, 7, rca << 16)
    sd_cmd(qt, 16, 512)

    # Linux: a descriptor table, here two 256-byte segments
    qt.write(0x00800000, struct.pack('<6I', 1 | (2 << 4), 256, 0x01000000,
                                     1 | 2 | (2 << 4), 256, 0x01100000))
    sd_data(qt, 512, addr=0x00800000)
    sts = sd_cmd(qt, 17, 3 * 512)
    check('CMD17 by descriptor DMA: command and data done together', sts,
          SD_CMD_SENT | SD_RSP_OK | SD_DATA_END | SD_DATA_OK)
    check('block 3 in the two segments',
          qt.read(0x01000000, 256) + qt.read(0x01100000, 256),
          img[3 * 512:4 * 512])
    check('FIFO_EN clears with the transfer', qt.readl(SDIO + 0x10C) & 1, 0)

    # U-Boot: 0x1FC = 1, the buffer address itself
    qt.writel(SDIO + 0x1FC, 1)
    sd_data(qt, 4 * 512, addr=0x01200000)
    sts = sd_cmd(qt, 18, 20 * 512)
    sd_cmd(qt, 12)
    check('CMD18 by plain DMA: four blocks', qt.read(0x01200000, 2048),
          img[20 * 512:24 * 512])
    check('DATA_END', sts & SD_DATA_END, SD_DATA_END)
    payload = random.Random(11).randbytes(512)
    qt.write(0x01300000, payload)
    sd_data(qt, 512, addr=0x01300000, write=True)
    sts = sd_cmd(qt, 24, 10 * 512)
    check('CMD24 by DMA completes', sts & SD_DATA_END, SD_DATA_END)
    check('block 10 written to the card image',
          open(card, 'rb').read()[10 * 512:11 * 512], payload)

    # A descriptor table shorter than DATA_LENGTH fails, not succeeds
    qt.writel(SDIO + 0x1FC, 0)
    qt.write(0x00800000, struct.pack('<3I', 1 | 2 | (2 << 4), 256, 0x01000000))
    sd_data(qt, 512, addr=0x00800000)
    sts = sd_cmd(qt, 17, 3 * 512)
    check('descriptors 256 bytes short: DATA_TIMEOUT | DMA_ERROR, no DATA_END',
          sts & (0x08 | (1 << 19) | SD_DATA_END), 0x08 | (1 << 19))
    sd_cmd(qt, 12)
    # A command the card does not answer leaves no data phase armed
    sd_data(qt, 512, addr=0x00800000)
    qt.writel(SDIO + 0x28, 0xFFFFFFFF)
    qt.writel(SDIO + 0x04, 0)
    qt.writel(SDIO, 5 | (1 << 6) | (1 << 9))          # CMD5: no SDIO
    check('a timed-out command disarms its data phase',
          (qt.readl(SDIO + 0x28) & 0x0C, qt.readl(SDIO + 0x10C) & 1,
           qt.readl(SDIO + 0x1C) >> 6 & 1), (0x04, 0, 0))
    qt.writel(SDIO + 0x1FC, 1)

    # PIO, as for unaligned buffers
    sd_data(qt, 512, dma=False)
    sts = sd_cmd(qt, 17, 3 * 512)
    check('PIO CMD17: command done, data waiting', sts & SD_DATA_END, 0)
    check('FIFO_STS full, 16 words', qt.readl(SDIO + 0x108), 0x200 | 16)
    got = b''.join(qt.readl(SDIO + 0x100).to_bytes(4, 'little')
                   for _ in range(128))
    check('block 3 through the data port', got, img[3 * 512:4 * 512])
    check('DATA_END after the last word',
          qt.readl(SDIO + 0x28) & SD_DATA_END, SD_DATA_END)
    check('FIFO_STS empty', qt.readl(SDIO + 0x108), 0x100)
    payload = random.Random(12).randbytes(512)
    sd_data(qt, 512, dma=False, write=True)
    sd_cmd(qt, 24, 11 * 512)
    for i in range(0, 512, 4):
        qt.writel(SDIO + 0x100, int.from_bytes(payload[i:i + 4], 'little'))
    check('PIO CMD24: DATA_END after the last word',
          qt.readl(SDIO + 0x28) & SD_DATA_END, SD_DATA_END)
    check('block 11 written to the card image',
          open(card, 'rb').read()[11 * 512:12 * 512], payload)
    sd_cmd(qt, 55, rca << 16)
    sd_data(qt, 8, blksz=8, dma=False)
    sd_cmd(qt, 51)
    check('ACMD51: the 8-byte SCR waits as two words',
          qt.readl(SDIO + 0x108) & 0x3F, 2)
    scr = qt.readl(SDIO + 0x100).to_bytes(4, 'little')
    qt.readl(SDIO + 0x100)
    check('SCR structure version 0 (byte 0 high nibble)', scr[0] >> 4, 0)
    check('and DATA_END', qt.readl(SDIO + 0x28) & SD_DATA_END, SD_DATA_END)

    # nvt_composite_gate_clk_prepare: SYS_RESET1 bit 2 low, then high
    qt.writel(CG + 0x84, qt.readl(CG + 0x84) & ~(1 << 2))
    qt.writel(CG + 0x84, qt.readl(CG + 0x84) | (1 << 2))
    check('the CG reset pulse returns it to descriptor DMA',
          (qt.readl(SDIO + 0x1FC), qt.readl(SDIO + 0x30)), (0, 0))
    check('and drops the interrupt', gic_pending(qt, SPI_SDIO), False)

    check('the empty SDIO2 slot times out',
          sd_cmd(qt, 8, 0x1AA, base=SDIO2), SD_CMD_SENT | SD_RSP_TIMEOUT)


# ---- nvt-i2c -----------------------------------------------------------------

I2C0, I2C1, I2C2 = 0xF0220000, 0xF0350000, 0xF03A0000
SPI_I2C0 = 41
I2C_DT, I2C_DR, I2C_BERR = 1 << 4, 1 << 5, 1 << 6
I2C_INTR_ALL = 0x1473                       # i2c_nvtim_init()


def i2c_chunk(qt, n, data=None, start=False, stop=False, nack=False,
              base=I2C0):
    """nvtim_set_data / nvtim_set_rxconfig: one chunk of n bytes."""
    cfg = n << 16
    if start:
        cfg |= 1 << 8
    if stop:
        cfg |= 1 << (12 + n - 1)
    if nack:
        cfg |= 1 << (4 + n - 1)
    qt.writel(base + 0x04, 0xFFFFFFFF)              # clear status
    qt.writel(base + 0x08, cfg)
    if data is not None:
        qt.writel(base + 0x10, data)
    qt.writel(base + 0x00, qt.readl(base + 0x00) | (1 << 3))
    return qt.readl(base + 0x04)


def i2c_write(qt, addr, payload, stop=True):
    """i2c_nvtim_xfer_msg() for a write: the address, then four-byte
    chunks; returns the last chunk's status."""
    stream = bytes([addr << 1]) + bytes(payload)
    for i in range(0, len(stream), 4):
        part = stream[i:i + 4]
        last = i + 4 >= len(stream)
        sts = i2c_chunk(qt, len(part), int.from_bytes(part, 'little'),
                        start=(i == 0), stop=last and stop)
        if sts & I2C_BERR:
            return sts
    return sts


def i2c_read(qt, addr, length):
    """The read side: the first chunk carries the address byte."""
    out = b''
    remain = length + 1
    first = True
    while remain:
        n = min(remain, 4)
        remain -= n
        sts = i2c_chunk(qt, n, (addr << 1 | 1) if first else None,
                        start=first, stop=not remain, nack=not remain)
        if sts & I2C_BERR:
            return sts, None
        data = qt.readl(I2C0 + 0x10).to_bytes(4, 'little')
        out += data[1 if first else 0:n]
        first = False
    return sts, out


def test_i2c(qt):
    print('nvt-i2c (sensor=sc401ai)')
    gic_enable(qt, SPI_I2C0)
    qt.writel(I2C0 + 0x00, 2)                       # I2C_CTRL_REG_DEFAULT
    check('idle: BUSFREE set, BUSBUSY clear',
          qt.readl(I2C0 + 0x04) & ((1 << 28) | (1 << 3)), 1 << 28)
    qt.writel(I2C0 + 0x00, I2C_INTR_ALL)
    sts = i2c_write(qt, 0x30, [0x31, 0x07], stop=False)
    check('register address 0x3107: DT', sts & (I2C_DT | I2C_DR | I2C_BERR),
          I2C_DT)
    check('and SPI 41', gic_pending(qt, SPI_I2C0), True)
    sts, data = i2c_read(qt, 0x30, 2)
    check('repeated START, read two: DR', sts & (I2C_DR | I2C_BERR), I2C_DR)
    check('chip ID 0xCD2E, what ipcinfo calls SC401AI', data, b'\xcd\x2e')

    i2c_write(qt, 0x30, [0x30, 0x00, 0x11, 0x22, 0x33, 0x44])
    i2c_write(qt, 0x30, [0x30, 0x00], stop=False)
    sts, data = i2c_read(qt, 0x30, 4)
    check('six bytes written and four read, two chunks each', data,
          b'\x11\x22\x33\x44')

    sts = i2c_write(qt, 0x31, [0x00])
    check('an empty address NACKs: BERR', sts & I2C_BERR, I2C_BERR)
    # i2c_nvtim_handle_nack(): a lone STOP
    sts = i2c_chunk(qt, 1, stop=True)
    check('the STOP after it completes cleanly', sts & I2C_BERR, 0)
    # i2cdetect's quick write: address, a 0 byte, STOP after it
    sts = i2c_chunk(qt, 2, 0x30 << 1, start=True, stop=True)
    check('i2cdetect quick write finds 0x30', sts & (I2C_DT | I2C_BERR),
          I2C_DT)
    qt.writel(I2C0 + 0x04, 0xFFFFFFFF)
    check('status write-back drops SPI 41', gic_pending(qt, SPI_I2C0), False)

    for base, name in ((I2C1, 'I2C 1'), (I2C2, 'I2C 2')):
        sts = i2c_chunk(qt, 1, 0x30 << 1, start=True, stop=True, base=base)
        check('%s is empty' % name, sts & I2C_BERR, I2C_BERR)


# ---- nvt-pwm -----------------------------------------------------------------

PWM = 0xF0210000
SPI_PWM = 26


def test_pwm(qt):
    print('nvt-pwm')
    gic_enable(qt, SPI_PWM)
    qt.cmd('irq_intercept_out /machine/nvt-pwm out')
    # nvt_pwm_config(): 1 ms at 50 %, base 30000 / fall 15000
    qt.writel(PWM + 0x04, (0x30 << 16) | (0x98 << 8))
    qt.writel(PWM + 0x230, (0x75 << 16) | (0x3A << 8))
    qt.writel(PWM + 0xE0, 1)                        # INT_ENABLE channel 0
    qt.writel(PWM + 0x100, 1)
    check('ENABLE reads channel 0 running', qt.readl(PWM + 0x100), 1)
    check('its out line is high', qt.irqs.get(0), True)
    qt.writel(PWM + 0x104, 1)
    check('DISABLE stops it', qt.readl(PWM + 0x100), 0)
    check('INT_STS bit 0: stopped', qt.readl(PWM + 0xF0) & 1, 1)
    check('SPI 26', gic_pending(qt, SPI_PWM), True)
    check('out low again', qt.irqs.get(0), False)
    qt.writel(PWM + 0xF0, 1)
    check('INT_STS write-back drops it', gic_pending(qt, SPI_PWM), False)

    # nvt_pwm_set_cycle(): 20 cycles x 30000 ticks at 30 MHz = 20 ms
    qt.writel(PWM + 0x00, 20)
    qt.writel(PWM + 0x100, 1)
    qt.step(19 * 1000 * 1000)
    check('a 20-cycle run: still running at 19 ms', qt.readl(PWM + 0x100), 1)
    qt.step(2 * 1000 * 1000)
    check('stopped by 21 ms', qt.readl(PWM + 0x100), 0)
    check('with INT_STS and SPI 26',
          (qt.readl(PWM + 0xF0) & 1, gic_pending(qt, SPI_PWM)), (1, True))
    qt.writel(PWM + 0xF0, 1)

    qt.writel(PWM + 0x04, 0x30 << 16)               # duty 0: fall 0 in
    qt.writel(PWM + 0x230, 0x75 << 16)              # both bytes
    qt.writel(PWM + 0x00, 0)
    qt.writel(PWM + 0x100, 1)
    check('a zero duty keeps out low', qt.irqs.get(0), False)
    qt.writel(PWM + 0x104, 1)
    qt.writel(PWM + 0x10C, 0x2)
    check('CLKDIV_LOAD flags CLKDIV_LOAD_STS', qt.readl(PWM + 0xFC), 0x2)
    qt.writel(PWM + 0xF4, 0x55)
    check('the micro-step status word is storage', qt.readl(PWM + 0xF4), 0x55)


# ---- nvt-timer ---------------------------------------------------------------

TIMER = 0xF0040000
SPI_TIMER = 0


def tmr(n, off):
    return TIMER + 0x100 + n * 0x10 + off


def timer_cfg(qt, n, target, ctrl):
    """timer_cfg(): stop, clear status, enable the interrupt, start."""
    qt.writel(tmr(n, 0), 0)
    qt.writel(TIMER + 0x10, 1 << n)
    qt.writel(TIMER + 0x20, qt.readl(TIMER + 0x20) | 1 << n)
    qt.writel(tmr(n, 4), target)
    qt.writel(tmr(n, 0), ctrl)


def test_timer(qt):
    print('nvt-timer')
    gic_enable(qt, SPI_TIMER)
    check('timer 0 runs free out of reset (U-Boot\'s microsecond clock)',
          qt.readl(tmr(0, 0)), 3)
    t0 = qt.readl(tmr(0, 8))
    qt.step(5000 * 1000)
    check('COUNTER 0 advances 5000 in 5 ms: 1 MHz',
          qt.readl(tmr(0, 8)) - t0, 5000)

    # one shot, divider 0: 1000 ticks
    timer_cfg(qt, 3, 999, 1)
    check('enable reads back at once (timer_cfg() polls it)',
          qt.readl(tmr(3, 0)) & 1, 1)
    qt.step(400 * 1000)
    check('one shot: 400 counted at 400 us', qt.readl(tmr(3, 8)), 400)
    qt.step(599 * 1000)
    check('no status before the target', qt.readl(TIMER + 0x10) & 1 << 3, 0)
    qt.step(1000)
    check('STS bit 3 at 1 ms, and SPI 0',
          (qt.readl(TIMER + 0x10) & 1 << 3, gic_pending(qt, SPI_TIMER)),
          (1 << 3, True))
    check('a one-shot timer stops, enable clear and COUNTER at its target',
          (qt.readl(tmr(3, 0)) & 1, qt.readl(tmr(3, 8))), (0, 999))
    qt.writel(TIMER + 0x10, 1 << 3)
    check('STS write-back drops SPI 0', gic_pending(qt, SPI_TIMER), False)

    # free run, divider 1 at 3 MHz / (5 + 1) = 500 kHz: a 2 ms period
    qt.writel(TIMER + 0x30, (5 << 8) | 1 << 17)
    timer_cfg(qt, 5, 999, 7)
    qt.step(2000 * 1000)
    check('free run on divider 1: STS at 2 ms (500 kHz)',
          qt.readl(TIMER + 0x10) & 1 << 5, 1 << 5)
    qt.writel(TIMER + 0x10, 1 << 5)
    check('it goes on', qt.readl(tmr(5, 0)) & 1, 1)
    qt.step(1000 * 1000)
    check('wrapped: 500 counted 1 ms into the next period',
          qt.readl(tmr(5, 8)), 500)
    qt.writel(TIMER + 0x30, (2 << 8) | 1 << 17)     # 1 MHz
    qt.step(250 * 1000)
    check('a divider change carries the count on at the new rate',
          qt.readl(tmr(5, 8)), 750)
    qt.writel(tmr(5, 0xC), 1)
    qt.step(100 * 1000)
    check('RELOAD starts a new period', qt.readl(tmr(5, 8)), 100)
    qt.writel(tmr(5, 0), 6)
    held = qt.readl(tmr(5, 8))
    qt.step(1000 * 1000)
    check('disabled, the count holds', qt.readl(tmr(5, 8)), held)
    qt.step(2000 * 1000)
    check('and no more status', qt.readl(TIMER + 0x10) & 1 << 5, 0)
    qt.writel(TIMER + 0x20, 0)
    qt.writel(tmr(7, 4), 9)
    qt.writel(tmr(7, 0), 1)
    qt.step(20 * 1000)
    check('a status bit without its INT bit raises nothing',
          (qt.readl(TIMER + 0x10) & 1 << 7, gic_pending(qt, SPI_TIMER)),
          (1 << 7, False))
    qt.writel(TIMER + 0x10, 1 << 7)
    qt.writel(tmr(5, 8), 1234)
    check('COUNTER is read-only', qt.readl(tmr(5, 8)), held)
    # divider 1 at 3 MHz / (4 + 1) = 600 kHz: a tick is 1666.7 ns, so a
    # rebase that went through nanoseconds and back would lose it
    qt.writel(TIMER + 0x30, (4 << 8) | 1 << 17)
    timer_cfg(qt, 11, 0xFFFFFF, 5)
    qt.step(1700)
    counts = []
    for _ in range(5):
        qt.writel(TIMER + 0x30, (4 << 8) | 1 << 17)
        counts.append(qt.readl(tmr(11, 8)))
    check('divider rewrites at 600 kHz keep the count (1, not back to 0)',
          counts, [1] * 5)
    qt.writel(tmr(11, 0), 0)
    # 3 MHz, a 1 us period: no drift from rounding each period to 333 ns
    qt.writel(TIMER + 0x30, 1 << 17)
    timer_cfg(qt, 12, 2, 7)
    qt.writel(TIMER + 0x20, 0)
    qt.step(10 * 1000 * 1000 + 334)
    check('3 MHz, period 3 ticks: 10 ms and 334 ns in, count 1',
          qt.readl(tmr(12, 8)), 1)
    qt.writel(tmr(12, 0), 5)                        # free run -> one shot
    qt.step(1000)
    check('switched to one shot mid-period: it ends there and stops',
          (qt.readl(tmr(12, 0)) & 1, qt.readl(tmr(12, 8))), (0, 2))
    qt.writel(TIMER + 0x10, 0xFFFFF)
    # The machine reset (the kernel's manual watchdog reset) restarts
    # timer 0 as the loader leaves it, and stops the rest.
    qt.writel(tmr(9, 4), 0xFFFF)
    qt.writel(tmr(9, 0), 1)
    qt.writel(WDT + 0x00, 0x5A960112)
    qt.writel(WDT + 0x00, 0x5A960113)
    qt.writel(WDT + 0x0C, 1)
    qt.step(1000)
    check('a machine reset: timer 0 free-running again, timer 9 stopped',
          (qt.readl(tmr(0, 0)), qt.readl(tmr(9, 0))), (3, 0))


# ---- nvt-crypto, nvt-hash, nvt-rsa -------------------------------------------

CRYPTO = 0xF0620000
HASH = 0xF0670000
RSA = 0xF06A0000
SPI_CRYPTO, SPI_HASH, SPI_RSA = 38, 28, 20
DESC, SRC, DST = 0x01000000, 0x01001000, 0x01002000


def crypto_dma(qt, cipher, op, data, key, iv=b'', decrypt=False, aad=None,
               get_s0=False, step=True, keymgr=False):
    """One na51089_crypto_start() request; returns the output, CV, S0
    and GHASH."""
    hdr = (int(decrypt) | cipher << 4 | op << 8 | int(keymgr) << 12 |
           int(get_s0) << 16)
    desc = bytearray(0x80 + 32 * 16)
    desc[0:len(key)] = key
    desc[0x20:0x20 + len(iv)] = iv
    if op == 4 and step:                    # CTR: counter step 1
        struct.pack_into('<I', desc, 0x3C if cipher >= 2 else 0x34, 1)
    struct.pack_into('<I', desc, 0x40, hdr)
    blk = 0
    if op == 5:                             # GCM: the AAD group first
        a = aad or b''
        qt.write(SRC + 0x800, a or b'\0')
        struct.pack_into('<IIII', desc, 0x80, SRC + 0x800, 0, len(a), 1)
        blk = 1
    qt.write(SRC, data)
    struct.pack_into('<IIII', desc, 0x80 + 16 * blk, SRC, DST, len(data), 1)
    qt.write(DESC, bytes(desc))
    qt.writel(CRYPTO + 0x00, 0x2)
    qt.writel(CRYPTO + 0x08, 0x10)
    qt.writel(CRYPTO + 0x0C, 0x10)
    qt.writel(CRYPTO + 0x50, DESC)
    qt.writel(CRYPTO + 0x04, 0x10)
    out = qt.read(DST, len(data))
    return (out, qt.read(DESC + 0x50, 16), qt.read(DESC + 0x60, 16),
            qt.read(DESC + 0x70, 16))


def test_crypto(qt):
    print('nvt-crypto')
    gic_enable(qt, SPI_CRYPTO)
    # PIO: FIPS-197 C.1, AES-128 of 00112233..ff
    key = bytes(range(16))
    pt = bytes.fromhex('00112233445566778899aabbccddeeff')
    for i in range(4):
        qt.writel(CRYPTO + 0x10 + 4 * i, struct.unpack_from('<I', key, 4 * i)[0])
        qt.writel(CRYPTO + 0x30 + 4 * i, struct.unpack_from('<I', pt, 4 * i)[0])
    qt.writel(CRYPTO + 0x00, 0x2 | 2 << 4)
    qt.writel(CRYPTO + 0x04, 1)
    out = b''.join(struct.pack('<I', qt.readl(CRYPTO + 0x40 + 4 * i))
                   for i in range(4))
    check('PIO AES-128 (FIPS-197 C.1)', out.hex(),
          '69c4e0d86a7b0430d8cdb78070b4c55a')
    check('PIO done in INT_STS bit 0, CTRL idle',
          (qt.readl(CRYPTO + 0x0C) & 1, qt.readl(CRYPTO + 0x04) & 1), (1, 0))
    qt.writel(CRYPTO + 0x0C, 1)

    k = bytes.fromhex('2b7e151628aed2a6abf7158809cf4f3c')
    p = bytes.fromhex('6bc1bee22e409f96e93d7e117393172a'
                      'ae2d8a571e03ac9c9eb76fac45af8e51')
    out, cv, _, _ = crypto_dma(qt, 2, 1, p, k, bytes(range(16)))
    check('DMA AES-128-CBC (SP 800-38A F.2.1)', out.hex(),
          '7649abac8119b246cee98e9b12e9197d5086cb9b507219ee95db113a917678b2')
    check('CV: the last cipher block, the next request\'s IV', cv, out[16:])
    check('DMA channel 0 done: INT_STS bit 4 and SPI 38',
          (qt.readl(CRYPTO + 0x0C) & 0x10, gic_pending(qt, SPI_CRYPTO)),
          (0x10, True))
    qt.writel(CRYPTO + 0x0C, 0x10)
    check('the write-back drops SPI 38', gic_pending(qt, SPI_CRYPTO), False)
    back, _, _, _ = crypto_dma(qt, 2, 1, out, k, bytes(range(16)), True)
    check('and decrypted back', back, p)

    # The same two blocks in two entries split 7 bytes in, each with its
    # own destination: the block across the split still encrypts.
    desc = bytearray(0x80 + 32 * 16)
    desc[0:16] = k
    desc[0x20:0x30] = bytes(range(16))
    struct.pack_into('<I', desc, 0x40, 2 << 4 | 1 << 8)
    struct.pack_into('<IIII', desc, 0x80, SRC, DST, 7, 0)
    struct.pack_into('<IIII', desc, 0x90, SRC + 0x100, DST + 0x100, 25, 1)
    qt.write(SRC, p[:7])
    qt.write(SRC + 0x100, p[7:])
    qt.write(DESC, bytes(desc))
    qt.writel(CRYPTO + 0x50, DESC)
    qt.writel(CRYPTO + 0x04, 0x10)
    check('AES-CBC, a block split across two entries',
          (qt.read(DST, 7) + qt.read(DST + 0x100, 25)).hex(),
          '7649abac8119b246cee98e9b12e9197d5086cb9b507219ee95db113a917678b2')
    qt.writel(CRYPTO + 0x0C, 0x10)

    ctr = bytes.fromhex('f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff')
    out, cv, _, _ = crypto_dma(qt, 2, 4, p[:16] + p[16:21], k, ctr)
    check('AES-128-CTR (F.5.1), 21 bytes: a partial last block',
          out.hex(), '874d6191b620e3261bef6864990db6ce9806f66b79')
    check('CV: the counter after both blocks, carried across bytes',
          cv.hex(), 'f0f1f2f3f4f5f6f7f8f9fafbfcfdff01')

    # GCM test case 2 (McGrew & Viega): K, IV, P all zero
    out, _, s0, gh = crypto_dma(qt, 2, 5, bytes(16), bytes(16), bytes(12))
    check('AES-GCM: the ciphertext', out.hex(),
          '0388dace60b6a392f328c2b971b2fe78')
    check('tag = S0 ^ GHASH, as the driver forms it',
          bytes(a ^ b for a, b in zip(s0, gh)).hex(),
          'ab6e47d42cec13bdf53a67b21257bddf')
    # test case 4: a partial last block and associated data
    k4 = bytes.fromhex('feffe9928665731c6d6a8f9467308308')
    iv4 = bytes.fromhex('cafebabefacedbaddecaf888')
    p4 = bytes.fromhex('d9313225f88406e5a55909c5aff5269a86a7a9531534f7da'
                       '2e4c303d8a318a721c3c0c95956809532fcf0e2449a6b525'
                       'b16aedf5aa0de657ba637b39')
    a4 = bytes.fromhex('feedfacedeadbeeffeedfacedeadbeefabaddad2')
    out, _, s0, gh = crypto_dma(qt, 2, 5, p4, k4, iv4, aad=a4)
    check('AES-GCM test case 4: 60 bytes, 20 of AAD',
          (out[:8].hex(), bytes(a ^ b for a, b in zip(s0, gh)).hex()),
          ('42831ec2217774244b7221b784d0d49c'[:16],
           '5bc94fbc3221a5db94fae95ae7121a47'))
    back, _, s0, gh = crypto_dma(qt, 2, 5, out, k4, iv4, True, aad=a4)
    check('decrypting it: the plaintext and the same tag (GHASH over the input)',
          (back == p4, bytes(a ^ b for a, b in zip(s0, gh)).hex()),
          (True, '5bc94fbc3221a5db94fae95ae7121a47'))

    out, _, _, _ = crypto_dma(qt, 0, 0, bytes.fromhex('0123456789abcdef'),
                              bytes.fromhex('133457799bbcdff1'))
    check('DES-ECB (the textbook example)', out.hex(), '85e813540f0ab405')
    k3 = bytes.fromhex('0123456789abcdef23456789abcdef01456789abcdef0123')
    out, _, _, _ = crypto_dma(qt, 1, 0, b'The quic', k3)
    # openssl enc -des-ede3 -K <k3> -nopad
    check('3DES-EDE-ECB, three keys', out.hex(), '1ccf23869d09333e')
    # The key from the OTP key manager, not the descriptor: blank, so zero
    out, _, s0, _ = crypto_dma(qt, 2, 4, bytes(16), b'\x55' * 16, bytes(16),
                               get_s0=True, keymgr=True)
    check('key manager key (zero); "get S0": S0 = E(K, IV), the text '
          'from the next counter',
          (s0.hex(), out.hex()),
          ('66e94bd4ef8a2c3b884cfa59ca342b2e',
           '58e2fccefa7e3061367f1d57a4e7455a'))


def hash_run(qt, mode, data, cfg_extra, padlen=None, key=None):
    if key is not None:
        k = key.ljust(64, b'\0')
        for i in range(16):
            qt.writel(HASH + 0x10 + 4 * i, struct.unpack_from('<I', k, 4 * i)[0])
    qt.write(SRC, data or b'\0')
    qt.writel(HASH + 0x04, len(data) if padlen is None else padlen)
    qt.writel(HASH + 0x94, SRC)
    qt.writel(HASH + 0x9C, len(data))
    qt.writel(HASH + 0x0C, 1)
    qt.writel(HASH + 0x08, 1)
    qt.writel(HASH + 0x00, 0x2 | mode << 4 | 1 << 12 | cfg_extra)
    return b''.join(struct.pack('<I', qt.readl(HASH + 0x70 + 4 * i))
                    for i in range(8))


def test_hash(qt):
    print('nvt-hash')
    gic_enable(qt, SPI_HASH)
    d = hash_run(qt, 1, b'abc', 1 << 13)
    check('SHA-256("abc"), padded by the engine', d.hex(),
          'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad')
    check('done: INT_STS bit 0 and SPI 28',
          (qt.readl(HASH + 0x0C) & 1, gic_pending(qt, SPI_HASH)), (1, True))
    qt.writel(HASH + 0x0C, 1)
    d = hash_run(qt, 0, b'abc', 1 << 13)
    check('SHA-1("abc")', d[:20].hex(),
          'a9993e364706816aba3e25717850c26c9cd0d89d')
    # two chunks: a whole block, then the rest from the IV registers
    msg = b'abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq' * 2
    mid = hash_run(qt, 1, msg[:64], 0)
    for i in range(8):
        qt.writel(HASH + 0x50 + 4 * i, struct.unpack_from('<I', mid, 4 * i)[0])
    d = hash_run(qt, 1, msg[64:], 1 << 8 | 1 << 13, padlen=len(msg))
    check('SHA-256 in two chunks, chained through the IV registers', d.hex(),
          hashlib.sha256(msg).hexdigest())
    d = hash_run(qt, 2, b'Hi There', 1 << 13 | 1 << 21 | 1 << 22,
                 key=b'\x0b' * 20)
    check('HMAC-SHA-1 (RFC 2202 case 1): ipad and opad by the engine',
          d[:20].hex(), 'b617318655057264e28bc0b6fb378c8ef146be00')
    d = hash_run(qt, 3, b'what do ya want for nothing?',
                 1 << 13 | 1 << 21 | 1 << 22, key=b'Jefe')
    check('HMAC-SHA-256 (RFC 4231 case 2)', d.hex(),
          '5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843')
    qt.writel(HASH + 0x0C, 1)
    qt.writel(HASH + 0x94, 0xE0000000)          # nothing there
    qt.writel(HASH + 0x9C, 64)
    qt.writel(HASH + 0x00, 0x2 | 1 << 4 | 1 << 12 | 1 << 13)
    check('a chunk it cannot read: done, and no digest left behind',
          (qt.readl(HASH + 0x0C) & 1, qt.readl(HASH + 0x70)), (1, 0))


def rsa_load(qt, port, value, words):
    qt.writel(RSA + port + 4, 0)
    for i in range(words):
        qt.writel(RSA + port, (value >> (32 * i)) & 0xFFFFFFFF)


def test_rsa(qt):
    print('nvt-rsa')
    gic_enable(qt, SPI_RSA)
    rng = random.Random(5)
    for width, bits in ((0, 256), (1, 512), (3, 2048)):
        n = rng.getrandbits(bits) | 1 << (bits - 1) | 1
        e = 65537 if bits < 2048 else rng.getrandbits(bits) % n
        m = rng.getrandbits(bits) % n
        words = 64                                  # what the driver writes
        rsa_load(qt, 0x20, m, words)
        rsa_load(qt, 0x10, n, words)
        rsa_load(qt, 0x18, e, words)
        qt.writel(RSA + 0x00, width << 1)
        qt.writel(RSA + 0x0C, 1)
        qt.writel(RSA + 0x08, 1)
        qt.writel(RSA + 0x04, 1)
        qt.writel(RSA + 0x24, 0)
        got = sum(qt.readl(RSA + 0x20) << (32 * i) for i in range(bits // 32))
        check('%d-bit m^e mod n through the SRAM ports' % bits, got == pow(m, e, n),
              True)
    check('done: INT_STS bit 0 and SPI 20',
          (qt.readl(RSA + 0x0C) & 1, gic_pending(qt, SPI_RSA)), (1, True))
    qt.writel(RSA + 0x0C, 1)
    check('the write-back drops SPI 20', gic_pending(qt, SPI_RSA), False)
    qt.writel(RSA + 0x00, 1 << 4 | 3 << 1)          # CRC32 of N
    qt.writel(RSA + 0x04, 1)
    check('a CRC32 key check (not modelled) never reports done',
          qt.readl(RSA + 0x0C) & 1, 0)


# ---- nvt-media: the HDAL engines ---------------------------------------------

SIE = 0xF0C00000
IME = 0xF0C40000
GRPH = 0xF0C80000
ISE = 0xF0C90000
H26X = 0xF0A10000
JPG = 0xF0A00000
EAC = 0xF0640000
DAI = 0xF0630000
LL = 0x01100000
FRAME_NS = 1000 * 1000 * 1000 // 30


def ll_upd(off, val, byte_en=0xF):
    return struct.pack('<Q', 4 << 61 | byte_en << 44 | off << 32 | val)


def ll_nxtll(addr):
    return struct.pack('<Q', 1 << 61 | addr << 8)


def test_media(qt):
    print('nvt-media')
    # SIE: a frame in three steps, single-mode outputs complete
    gic_enable(qt, 1)
    qt.writel(SIE + 0x08, 0x2008F)                  # VD, BP3, OUT0 end, CRPEND
    qt.writel(SIE + 0xA4, 0x70000000)               # three outputs, single
    qt.writel(SIE + 0x00, 0x6)                      # ACT_EN | LOAD
    check('SIE: LOAD clears itself, DMA idle reads set',
          qt.readl(SIE + 0x00) & 0x02000006, 0x02000004)
    qt.step(FRAME_NS // 3 + 1000)
    check('SIE frame start: VD and the window starts, SPI 1',
          (qt.readl(SIE + 0x0C), gic_pending(qt, 1)), (0x31, True))
    qt.writel(SIE + 0x0C, 0xFFFFFFFF)
    qt.step(FRAME_NS // 3)
    check('half way: the line break points', qt.readl(SIE + 0x0C), 0xE)
    qt.writel(SIE + 0x0C, 0xFFFFFFFF)
    qt.step(FRAME_NS // 3)
    check('frame end: the window ends and the DRAM outputs\' ends',
          qt.readl(SIE + 0x0C), 0x31F80)
    check('a single-mode output clears its enable once written',
          qt.readl(SIE + 0xA4), 0)
    qt.writel(SIE + 0x00, 0)
    qt.writel(SIE + 0x0C, 0xFFFFFFFF)
    qt.step(FRAME_NS)
    check('stopped: no more frames', qt.readl(SIE + 0x0C), 0)

    # IME: a linked list of two jobs; its updates set the enables
    gic_enable(qt, 6)
    second = LL + 0x100
    qt.write(LL, ll_upd(0x018, 0xA0000009) + ll_upd(0x100, 0xBEEF1234, 0x3) +
             ll_nxtll(second))
    qt.write(second, ll_upd(0x104, 0x5A5A5A5A) + struct.pack('<Q', 0))
    qt.writel(IME + 0x100, 0xFFFF0000)
    qt.writel(IME + 0x14, LL)
    qt.writel(IME + 0x00, 1 << 28)
    check('IME: the fire bit does not stay', qt.readl(IME + 0x00), 0)
    qt.step(1000 * 1000)
    check('job 1: its updates applied, bytes as enabled',
          (qt.readl(IME + 0x18), qt.readl(IME + 0x100)),
          (0xA0000009, 0xFFFF1234))
    check('frame start / end and job end in status, SPI 6',
          (qt.readl(IME + 0x1C), gic_pending(qt, 6)), (0xA0000008, True))
    qt.writel(IME + 0x1C, 0xFFFFFFFF)
    check('status written back: SPI 6 drops', gic_pending(qt, 6), False)
    qt.step(1000 * 1000)
    check('job 2, the last: list end too',
          (qt.readl(IME + 0x104), qt.readl(IME + 0x1C)),
          (0x5A5A5A5A, 0xA0000009))
    qt.writel(IME + 0x1C, 0xFFFFFFFF)

    # 2D graphics: list end without DMA idle (an abort's); single jobs
    gic_enable(qt, 18)
    qt.write(LL, ll_upd(0x008, 0x16) + struct.pack('<Q', 0))
    qt.writel(GRPH + 0xD0, LL)
    qt.writel(GRPH + 0x00, 1 << 28)
    qt.step(1000 * 1000)
    check('graphics list: list end only, SPI 18',
          (qt.readl(GRPH + 0x0C), gic_pending(qt, 18)), (0x2, True))
    qt.writel(GRPH + 0x0C, 0xFF)
    qt.writel(GRPH + 0x00, 0x2)
    check('software reset takes effect at once', qt.readl(GRPH + 0x00), 0)
    qt.writel(GRPH + 0x08, 0x1)
    qt.writel(GRPH + 0x00, 0x1)
    qt.step(1000 * 1000)
    check('a single job: done in status bit 0, its trigger dropped',
          (qt.readl(GRPH + 0x0C), qt.readl(GRPH + 0x00)), (0x1, 0))
    qt.writel(GRPH + 0x0C, 0xFF)

    # ISE: list end and frame end
    qt.write(LL, struct.pack('<Q', 0))
    qt.writel(ISE + 0x2C, LL)
    qt.writel(ISE + 0x00, 1 << 28)
    qt.step(1000 * 1000)
    check('ISE list: frame end and list end', qt.readl(ISE + 0x0C),
          0x80000001)

    # H.26x: INT_FLAG write 1 to clear, reset, a picture
    gic_enable(qt, 16)
    qt.writel(H26X + 0x004, 0x01101711)
    check('H.26x: INT_FLAG clears by writing ones', qt.readl(H26X + 0x004), 0)
    qt.writel(H26X + 0x000, 0x10)
    check('software reset: its finish flag', qt.readl(H26X + 0x004),
          0x10000000)
    qt.writel(H26X + 0x004, 0xFFFFFFFF)
    qt.writel(H26X + 0x070, DST)
    qt.writel(H26X + 0x07C, 0x10000)
    qt.writel(H26X + 0x01C, 0x1)
    qt.writel(H26X + 0x000, 0x100)
    qt.step(1000 * 1000)
    check('a picture: finish flag and SPI 16, start dropped',
          (qt.readl(H26X + 0x004), gic_pending(qt, 16),
           qt.readl(H26X + 0x000)), (1, True, 0))
    nal = qt.read(DST, 64)
    check('BS_LEN 64, a filler NAL unit in the buffer',
          (qt.readl(H26X + 0x268), nal[:6].hex(), nal[63]),
          (64, '000000014c01', 0x80))
    qt.writel(H26X + 0x004, 0xFFFFFFFF)
    qt.write(DST, b'\x55' * 8)
    qt.writel(H26X + 0x07C, 0)
    qt.writel(H26X + 0x000, 0x100)
    qt.step(1000 * 1000)
    check('no bitstream buffer: finishes with BS_LEN 0, writes nothing',
          (qt.readl(H26X + 0x004) & 1, qt.readl(H26X + 0x268),
           qt.read(DST, 8)), (1, 0, b'\x55' * 8))
    qt.writel(H26X + 0x004, 0xFFFFFFFF)

    # JPEG: reset, DMA idle, a grey 4-MCU picture
    gic_enable(qt, 17)
    qt.writel(JPG + 0x00, 0x8)
    check('JPEG: reset clears itself, DMA idle reads set',
          qt.readl(JPG + 0x00), 0x20)
    qt.writel(JPG + 0x2C, 3)                        # four MCUs, minus one
    qt.writel(JPG + 0x40, DST)
    qt.writel(JPG + 0x44, 0x1000)
    qt.writel(JPG + 0x18, 1)
    qt.writel(JPG + 0x00, 1)
    qt.step(1000 * 1000)
    check('frame end, SPI 17, act dropped',
          (qt.readl(JPG + 0x1C), gic_pending(qt, 17),
           qt.readl(JPG + 0x00) & 1), (1, True, 0))
    check('no picture to read: flat grey, DC 0 / EOB per block, 4 bytes '
          'per MCU, then EOI',
          (qt.readl(JPG + 0x4C), qt.read(DST, 18).hex()),
          (18, '28a28a00' * 4 + 'ffd9'))
    qt.writel(JPG + 0x1C, 1)

    # EAC: strobes the driver waits on
    qt.writel(EAC + 0x04, 1)
    qt.writel(EAC + 0x10, 0x17000)
    check('EAC: LOAD and the DC-cancel load clear themselves',
          (qt.readl(EAC + 0x04), qt.readl(EAC + 0x10)), (0, 0x15000))

    # DAI: capture DMA, double-buffered
    gic_enable(qt, 15)
    qt.write(DST, b'\xAA' * 1024)
    qt.writel(DAI + 0x48, DST)
    qt.writel(DAI + 0x68, 256)                      # words: 1 KiB
    qt.writel(DAI + 0x20, 0x444)                    # RX load, BFI, stop
    qt.writel(DAI + 0x00, 0x41)
    check('DAI: RX DMA on latches the buffer (DMALOAD), SPI 15',
          (qt.readl(DAI + 0x28), gic_pending(qt, 15)), (0x40, True))
    qt.writel(DAI + 0x28, 0xFFFFFFFF)
    qt.step(32 * 1000 * 1000)
    check('1 KiB at 32 KB/s later: buffer full and the next load',
          qt.readl(DAI + 0x28), 0x44)
    check('the buffer holds silence', qt.read(DST, 1024), bytes(1024))
    qt.writel(DAI + 0x28, 0xFFFFFFFF)
    qt.writel(DAI + 0x00, 0x1)
    check('RX DMA off: stop', qt.readl(DAI + 0x28), 0x400)


PX_RAW, PX_YUV = 0x01200000, 0x01400000


GRPH2 = 0xF0D10000
GA, GB, GC = 0x01500000, 0x01510000, 0x01520000


def gfx_job(qt, base, regs, lladdr=0xD0):
    """One job through the engine's linked list; regs {offset: value}."""
    qt.write(LL, b''.join(ll_upd(o, v) for o, v in regs.items()) +
             struct.pack('<Q', 0))
    qt.writel(base + lladdr, LL)
    qt.writel(base + 0x00, 1 << 28)
    qt.step(1000 * 1000)
    qt.writel(base + 0x0C, 0xFFFFFFFF)


def grph(qt, cfg, w, h, a=GA, alofs=0, b=GB, blofs=0, c=GC, clofs=0,
         const=(0, 0), base=GRPH):
    gfx_job(qt, base, {0x04: cfg, 0x20: 0, 0x24: 0, 0x28: 0,
                       0x60: const[0], 0x64: const[1],
                       0x70: a, 0x74: alofs or w, 0x80: b, 0x84: blofs or w,
                       0x90: c, 0x94: clofs or w, 0x78: h, 0x7C: w})


def test_gfx(qt):
    print('nvt-media graphics and ISE')
    # A_COPY, 8-bit: 16 bytes x 2 lines between line offsets 32 and 16
    src = bytes(range(64))
    qt.write(GA, src)
    grph(qt, 0x00000011, 16, 2, alofs=32, clofs=16)
    check('A_COPY: two lines of 16 bytes, line offset 32 to 16',
          qt.read(GC, 32).hex(), (src[:16] + src[32:48]).hex())
    # MINUS_SHF, 16-bit, as hd_gfx_arithmetic: (6000 + i) - i
    qt.write(GA, struct.pack('<8H', *[6000 + i for i in range(8)]))
    qt.write(GB, struct.pack('<8H', *range(8)))
    grph(qt, 0x02000021, 16, 1)
    check('MINUS_SHF 16-bit: (6000 + i) - i',
          struct.unpack('<8H', qt.read(GC, 16)), (6000,) * 8)
    qt.write(GA, struct.pack('<2H', 5, 9))
    qt.write(GB, struct.pack('<2H', 9, 5))
    grph(qt, 0x02000021, 4, 1, const=(1 << 31, 0))
    check('and its magnitude (CONST1 bit 31): |5 - 9|, |9 - 5|',
          struct.unpack('<2H', qt.read(GC, 4)), (4, 4))
    qt.write(GA, bytes([200, 10]))
    qt.write(GB, bytes([100, 20]))
    grph(qt, 0x01000011, 2, 1)
    check('PLUS_SHF 8-bit saturates: 200 + 100, 10 + 20',
          list(qt.read(GC, 2)), [255, 30])
    # TEXT_COPY, 8-bit: an NV12 green U / V pair laid over the words
    grph(qt, 0x08000011, 6, 2, c=GC + 2, clofs=8, const=(0x142A142A, 0))
    check('TEXT_COPY 8-bit: U V pairs from an address 2 mod 4, two lines',
          (qt.read(GC + 2, 6).hex(), qt.read(GC + 10, 6).hex()),
          ('2a142a142a14', '2a142a142a14'))
    # PACKING: U and V planes into pairs
    qt.write(GA, bytes([1, 2, 3, 4]))
    qt.write(GB, bytes([5, 6, 7, 8]))
    grph(qt, 0x10000011, 8, 1)
    check('PACKING: U from A, V from B, interleaved',
          list(qt.read(GC, 8)), [1, 5, 2, 6, 3, 7, 4, 8])
    # RGBYUV_BLEND, ARGB4444 over NV12: two opaque green pixels, two
    # transparent ones, two lines
    qt.write(GA, struct.pack('<8H', *([0xF0F0] * 2 + [0x0F00] * 2) * 2))
    qt.write(GB, bytes([0x10]) * 8)
    qt.write(GC, bytes([0x80]) * 4)
    grph(qt, 0x18000421, 8, 2, blofs=4, clofs=4, const=(0x21, 0))
    check('RGBYUV_BLEND ARGB4444: opaque green over Y, transparent leaves it',
          (qt.read(GB, 4).hex(), qt.read(GB + 4, 4).hex()),
          ('93931010', '93931010'))
    check('and over the 4:2:0 U / V pairs', qt.read(GC, 4).hex(), '2a148080')
    # GOP on engine 2: a vertical flip (C at the last line), then a
    # horizontal one at 16 bits
    qt.write(GA, bytes([1, 1, 2, 2, 3, 3]))
    grph(qt, 0x00040010, 2, 3, c=GC + 4, base=GRPH2)
    check('VTC_FLIP: line y of A lands y lines above C',
          list(qt.read(GC, 6)), [3, 3, 2, 2, 1, 1])
    qt.write(GA, struct.pack('<4H', 1, 2, 3, 4))
    grph(qt, 0x00030020, 8, 1, base=GRPH2)
    check('HRZ_FLIP 16-bit: a line reversed in 16-bit units',
          struct.unpack('<4H', qt.read(GC, 8)), (4, 3, 2, 1))

    # ISE: a 4x4 Y8 ramp (10 x) up to 7x7, bilinear; and down to 2x2,
    # integration; U / V pairs, nearest
    qt.write(GA, bytes(10 * x + y for y in range(4) for x in range(4)))
    def ise(ctrl, iw, ih, ow, oh, ilofs, olofs):
        gfx_job(qt, ISE, {0x04: ctrl, 0x10: ih << 16 | iw, 0x18: ilofs,
                          0x20: GA, 0x38: oh << 16 | ow, 0x90: olofs,
                          0x98: GC}, lladdr=0x2C)
    ise(0x100, 4, 4, 7, 7, 4, 8)
    check('ISE bilinear 4x4 to 7x7: corners kept, half steps between',
          (list(qt.read(GC, 7)), qt.read(GC + 6 * 8 + 6, 1)[0]),
          ([0, 5, 10, 15, 20, 25, 30], 33))
    ise(0x300, 4, 4, 2, 2, 4, 4)
    check('ISE integration 4x4 to 2x2: each output the average it covers',
          (list(qt.read(GC, 2)), list(qt.read(GC + 4, 2))),
          ([6, 26], [8, 28]))
    qt.write(GA, bytes([10, 20, 30, 40, 50, 60, 70, 80]))
    ise(0x203, 4, 1, 2, 1, 8, 4)
    check('ISE nearest, U / V pairs: every other pair',
          list(qt.read(GC, 4)), [10, 20, 50, 60])


def test_pixels(qt):
    print('nvt-media pixels')
    # SIE: the test pattern, RGGB, 12-bit packed, at DRAM output 0
    w, h = 64, 48
    qt.writel(SIE + 0x2C, h << 16 | w)
    qt.writel(SIE + 0x5C, PX_RAW)
    qt.writel(SIE + 0x60, 2 << 16 | 96)             # 12 bits, 96 bytes a line
    qt.writel(SIE + 0x00, 0x4)
    qt.step(FRAME_NS + 1000)
    qt.writel(SIE + 0x00, 0)
    def raw12(line, x):
        b = 3 * (x // 2)
        v = line[b] | line[b + 1] << 8 | line[b + 2] << 16
        return v >> (12 * (x & 1)) & 0xFFF

    line = qt.read(PX_RAW, 96)
    check('SIE writes packed 12-bit RAW: the white bar, R G R G',
          [raw12(line, x) for x in range(4)], [3071] * 4)
    line = qt.read(PX_RAW + 96, 96)
    check('line 1 is G B: G in the white bar, B in the blue bar (x 49), '
          'none in the red (x 41)',
          (raw12(line, 0), raw12(line, 49), raw12(line, 41)), (3071, 3071, 0))
    # IME: that RAW through IFE / IME to YUV 4:2:0, 32x24 (scaled by half)
    qt.writel(0xF0C70000 + 0x20, h << 16 | w)       # IFE input
    qt.writel(0xF0C70000 + 0x30, PX_RAW)
    qt.writel(0xF0C70000 + 0x34, 96)
    qt.writel(0xF0C70000 + 0x04, 2 << 5)            # 12 bits
    qt.writel(IME + 0x04, 1 << 2)                   # path 1
    qt.writel(IME + 0x84, 24 << 16 | 32)
    qt.writel(IME + 0x94, PX_YUV)
    qt.writel(IME + 0x98, PX_YUV + 0x1000)
    qt.writel(IME + 0x8C, 32)
    qt.writel(IME + 0x90, 32)
    qt.write(LL, struct.pack('<Q', 0))
    qt.writel(IME + 0x14, LL)
    qt.writel(IME + 0x00, 1 << 28)
    qt.step(1000 * 1000)
    y = qt.read(PX_YUV, 32)
    uv = qt.read(PX_YUV + 0x1000, 32)
    check('IME path 1: Y of the bars, white to black (BT.601, limited)',
          [y[2], y[6], y[30]], [180, 161, 16])
    check('UV of the white bar neutral, of the blue bar blue-shifted',
          (uv[0], uv[1], uv[24] > 200), (128, 128, True))
    qt.writel(IME + 0x1C, 0xFFFFFFFF)
    # 16-bit RAW spans the whole range, and IME reads it back as 12-bit;
    # an odd output size (31x23) still writes every byte of its extent
    qt.writel(SIE + 0x60, 3 << 16 | 128)            # 16 bits, 128 bytes a line
    qt.writel(SIE + 0x00, 0x4)
    qt.step(FRAME_NS + 1000)
    qt.writel(SIE + 0x00, 0)
    line = qt.read(PX_RAW, 128)
    check('SIE writes 16-bit RAW scaled to the range: white bar 3071 << 4',
          struct.unpack('<2H', line[:4]), (3071 << 4, 3071 << 4))
    qt.write(PX_YUV, bytes([0xAA]) * 0x2000)
    qt.writel(0xF0C70000 + 0x34, 128)
    qt.writel(0xF0C70000 + 0x04, 3 << 5)            # 16 bits
    qt.writel(IME + 0x84, 23 << 16 | 31)
    qt.write(LL, struct.pack('<Q', 0))
    qt.writel(IME + 0x14, LL)
    qt.writel(IME + 0x00, 1 << 28)
    qt.step(1000 * 1000)
    y = qt.read(PX_YUV, 32 * 23)
    uv = qt.read(PX_YUV + 0x1000, 32 * 11)
    check('IME from 16-bit RAW: the same Y of the bars',
          [y[2], y[6], y[29]], [180, 161, 16])
    check('odd output: the last column and row written, the stride gap not',
          (y[30] != 0xAA, y[22 * 32 + 30] != 0xAA, y[31], uv[30] != 0xAA,
           uv[31]),
          (True, True, 0xAA, True, 0xAA))
    qt.writel(IME + 0x1C, 0xFFFFFFFF)
    # JPEG: a 16x16 picture, the driver's quantisation via the LUT port
    pic = bytes((x * 16) & 0xFF for yy in range(16) for x in range(16))
    qt.write(PX_YUV, pic)
    qt.write(PX_YUV + 0x1000, bytes([128]) * 128)
    qt.writel(JPG + 0x10, 1 << 12 | 1 << 16)        # table 1, auto-increment
    for i in range(32):
        qt.writel(JPG + 0x14, 0x01010101)           # all ones: lossless-ish
    qt.writel(JPG + 0x28, 0)                        # one MCU
    qt.writel(JPG + 0x54, PX_YUV)
    qt.writel(JPG + 0x58, PX_YUV + 0x1000)
    qt.writel(JPG + 0x60, 15)
    qt.writel(JPG + 0x64, 15)
    qt.writel(JPG + 0x40, DST)
    qt.writel(JPG + 0x44, 0x1000)
    qt.writel(JPG + 0x00, 1)
    qt.step(1000 * 1000)
    n = qt.readl(JPG + 0x4C)
    scan = qt.read(DST, n)
    check('JPEG of a real picture: a scan ending in EOI', scan[-2:].hex(),
          'ffd9')
    try:
        from PIL import Image
        import io
        hdr = bytearray(b'\xff\xd8')
        hdr += b'\xff\xdb\x00\x84\x00' + bytes([1] * 64) + b'\x01' + \
            bytes([1] * 64)
        hdr += b'\xff\xc0\x00\x11\x08\x00\x10\x00\x10\x03' + \
            b'\x01\x22\x00\x02\x11\x01\x03\x11\x01'
        hdr += jpeg_std_dht()
        hdr += b'\xff\xda\x00\x0c\x03\x01\x00\x02\x11\x03\x11\x00\x3f\x00'
        im = Image.open(io.BytesIO(bytes(hdr) + scan)).convert('L')
        got = [im.getpixel((x, 8)) for x in (0, 7, 15)]
        check('decodes (PIL) to the picture: the ramp, within 2',
              all(abs(g - e) <= 2 for g, e in zip(got, (0, 112, 240))), True)
    except ImportError:
        print('  SKIP: decoding the JPEG (no Python PIL)')
    qt.writel(JPG + 0x1C, 1)


def jpeg_std_dht():
    """The Annex K Huffman tables, as the driver's JPEG header has them."""
    return bytes.fromhex(
        'ffc401a20000010501010101010100000000000000000102030405060708090a'
        '0b100002010303020403050504040000017d0102030004110512213141061351'
        '6107227114328191a1082342b1c11552d1f02433627282090a161718191a2526'
        '2728292a3435363738393a434445464748494a535455565758595a6364656667'
        '68696a737475767778797a838485868788898a92939495969798999aa2a3a4a5'
        'a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9da'
        'e1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9fa010003010101010101010101'
        '0000000000000102030405060708090a0b110002010204040304070504040001'
        '0277000102031104052131061241510761711322328108144291a1b1c1092333'
        '52f0156272d10a162434e125f11718191a262728292a35363738393a43444546'
        '4748494a535455565758595a636465666768696a737475767778797a82838485'
        '868788898a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9ba'
        'c2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae2e3e4e5e6e7e8e9eaf2f3f4f5f6'
        'f7f8f9fa')


# ---- nvt-usb, nvt-otp -------------------------------------------------------

USB = 0xF0600000
USB_ALIAS = 0xFF600000
OTP = 0xF0660000


def test_usb(qt, stick):
    print('nvt-usb' + (' (a usb-storage stick)' if stick else ' (no device)'))
    if not stick:
        check('no device: PORTSC connect clear', qt.readl(USB + 0x30) & 1, 0)
        check('and OTG speed 0', qt.readl(USB + 0x80) >> 22 & 3, 0)
        return
    for base, where in ((USB, '0xF0600000'), (USB_ALIAS, '0xFF600000')):
        caps = qt.readl(base)
        check('%s: CAPLENGTH 0x10, EHCI 1.00' % where,
              (caps & 0xFF, caps >> 16), (0x10, 0x0100))
        check('%s: HCSPARAMS one port' % where, qt.readl(base + 4) & 0xF, 1)
    check('PORTSC at 0x30: the stick is connected',
          qt.readl(USB + 0x30) & 3, 3)
    check('OTG register [23:22]: high speed (ehci_port_speed)',
          qt.readl(USB + 0x80) >> 22 & 3, 2)
    qt.writel(USB + 0x80, 1 << 4)                       # A_BUS_REQ
    check('OTG register keeps what is written', qt.readl(USB + 0x80),
          (1 << 4) | (2 << 22))
    for off in (0x100, 0x1C8, 0x30C, 0x400, 0x1000 + 0x51 * 4):
        qt.writel(USB + off, 0x5A5A0000 | off)
    check('glue and PHY registers are storage',
          [qt.readl(USB + off) for off in (0x100, 0x400, 0x1144)],
          [0x5A5A0100, 0x5A5A0400, 0x5A5A1144])
    check('the alias reaches the same glue', qt.readl(USB_ALIAS + 0x400),
          0x5A5A0400)
    # ehci_hub_control SetPortFeature(RESET), then clear it
    qt.writel(USB + 0x30, (1 << 8))
    qt.writel(USB + 0x30, 0)
    check('a port reset enables the port', qt.readl(USB + 0x30) >> 2 & 1, 1)
    # A machine reset (the kernel's manual watchdog reset) clears the glue
    # as well as the EHCI.
    qt.writel(WDT + 0x00, 0x5A960112)
    qt.writel(WDT + 0x00, 0x5A960113)
    qt.writel(WDT + 0x0C, 1)
    qt.step(1000)
    check('a machine reset clears the glue, PHY and OTG bits',
          (qt.readl(USB + 0x400), qt.readl(USB + 0x1144),
           qt.readl(USB + 0x80) & 0x30), (0, 0, 0))
    check('and the EHCI: port disabled again',
          qt.readl(USB + 0x30) >> 2 & 1, 0)


def test_otp(qt):
    print('nvt-otp')
    check('powered down: not ready', qt.readl(OTP + 0x30) & 1, 0)
    qt.writel(OTP + 0x20, 1)
    check('efuse_open: CTRL bit 0, then STATUS bit 0',
          qt.readl(OTP + 0x30) & 1, 1)
    qt.writel(OTP + 0x04, 5)
    qt.writel(OTP + 0x20, 1 | (1 << 4))
    check('efuse_read: the start bit clears',
          qt.readl(OTP + 0x20) & (1 << 4), 0)
    check('STATUS bit 1: read done', qt.readl(OTP + 0x30) & 2, 2)
    check('a blank array reads 0', qt.readl(OTP + 0x08), 0)
    qt.writel(OTP + 0x30, 2)
    check('writing bit 1 back clears it', qt.readl(OTP + 0x30) & 2, 0)
    qt.writel(OTP + 0x20, 0)
    check('efuse_close: not ready again', qt.readl(OTP + 0x30) & 1, 0)


# ---- flash boot ROM ----------------------------------------------------

def fdt_blob(tree):
    """Flatten {name: {prop: bytes | {child...}}} into a DTB."""
    strings = bytearray()
    struct_ = bytearray()

    def string(name):
        i = strings.find(name.encode() + b'\0')
        if i < 0:
            i = len(strings)
            strings.extend(name.encode() + b'\0')
        return i

    def node(name, body):
        struct_.extend(struct.pack('>I', 1) + name.encode() + b'\0')
        while len(struct_) % 4:
            struct_.append(0)
        for k, v in body.items():
            if isinstance(v, bytes):
                struct_.extend(struct.pack('>III', 3, len(v), string(k)) + v)
                while len(struct_) % 4:
                    struct_.append(0)
        for k, v in body.items():
            if isinstance(v, dict):
                node(k, v)
        struct_.extend(struct.pack('>I', 2))

    node('', tree)
    struct_.extend(struct.pack('>I', 9))
    rsv = b'\0' * 16
    off_rsv = 40
    off_struct = off_rsv + len(rsv)
    off_strings = off_struct + len(struct_)
    total = off_strings + len(strings)
    hdr = struct.pack('>10I', 0xD00DFEED, total, off_struct, off_strings,
                      off_rsv, 17, 16, 0, len(strings), len(struct_))
    return hdr + rsv + bytes(struct_) + bytes(strings)


def cells(*v):
    return struct.pack('>%dI' % len(v), *v)


def lz_pack(data, marker=0xF7):
    """BCL's LZ77: literals, marker escapes, (length, offset) references."""
    def var(v):
        out = [v & 0x7F]
        v >>= 7
        while v:
            out.append(0x80 | (v & 0x7F))
            v >>= 7
        return bytes(reversed(out))
    out = bytearray([marker])
    i = 0
    while i < len(data):
        # a reference back to a 64-byte-aligned repeat, when there is one
        if i >= 64 and data[i:i + 64] == data[i - 64:i]:
            n = 64
            while i + n < len(data) and data[i + n] == data[i + n - 64]:
                n += 1
            out += bytes([marker]) + var(n) + var(64)
            i += n
            continue
        out.append(data[i])
        if data[i] == marker:
            out.append(0)
        i += 1
    return bytes(out)


def make_boot_image(path, part_size=0x40000, bcl=None, nand=False):
    rng = random.Random(7)
    head = bytes(rng.randrange(256) for _ in range(300)) + b'\xf7' * 3
    payload = head + head[-64:] * 40 + bytes(range(256))
    packed = lz_pack(payload)
    if bcl is None:
        bcl = b'BCL1' + struct.pack('>III', 9, len(payload), len(packed)) + \
            packed
    tree = {
        'nvt_memory_cfg': {
            '#address-cells': cells(1), '#size-cells': cells(1),
            'fdt': {'reg': cells(0x100000, 0x100000)},
            'shmem': {'reg': cells(0x200000, 0x100000)},
            'uboot': {'reg': cells(0x3900000, 0x6C0000)},
        },
        ('nand@f0400000' if nand else 'nor@f0400000'): {
            'compatible': b'nvt,nvt_spinand\0' if nand else
                          b'nvt,nvt_spinor\0',
            '#address-cells': cells(2), '#size-cells': cells(2),
            'partition_uboot': {'reg': cells(0, 0xC0000 if nand else 0x50000,
                                             0, part_size)},
        },
    }
    dtb = fdt_blob(tree)
    if nand:
        # cfg_566_IPCAM1_EVB: the tree in block 2, U-Boot as built
        img = bytearray(b'\xff' * NAND_SIZE)
        img[0x40000:0x40000 + len(dtb)] = dtb
        img[0xC0000:0xC0000 + len(payload)] = payload
    else:
        img = bytearray(b'\xff' * (8 << 20))
        img[0x10000:0x10000 + len(dtb)] = dtb
        img[0x50000:0x50000 + len(bcl)] = bcl
    open(path, 'wb').write(img)
    return payload, dtb


def test_boot_rom(qemu, tmp):
    print('flash boot ROM')
    path = os.path.join(tmp, 'boot.bin')
    payload, dtb = make_boot_image(path)
    qt = QTest(qemu, 'nt98562,flash-file=' + path)
    try:
        def ram(a, n):
            return b''.join(qt.readl(a + i).to_bytes(4, 'little')
                            for i in range(0, n, 4))[:n]
        check('U-Boot unpacked from BCL1 at /nvt_memory_cfg/uboot',
              ram(0x3900000, len(payload)), payload)
        check('device tree copied to /nvt_memory_cfg/fdt',
              ram(0x100000, len(dtb)), dtb)
        shm = ram(0x200000, 48)
        check('SHMINFO says LD_NVT', shm[:6], b'LD_NVT')
        check('and points at the device tree',
              struct.unpack_from('<I', shm, 44)[0], 0x100000)
        check('a branch to U-Boot at 0',
              (qt.readl(0), qt.readl(4)) == (0xE51FF004, 0x3900000), True)
    finally:
        qt.close()
    def refused(img, what):
        out = subprocess.run([qemu, '-M', 'nt98562,flash-file=' + img,
                              '-display', 'none', '-nodefaults', '-S'],
                             capture_output=True, text=True, timeout=30)
        # exit(1) with a message, not a signal
        check(what, out.returncode == 1 and
              ('novatek:' in out.stderr or 'nvt-flash:' in out.stderr), True)

    blank = os.path.join(tmp, 'blank.bin')
    open(blank, 'wb').write(b'\xff' * (8 << 20))
    refused(blank, 'an image without a device tree is refused')
    bad = os.path.join(tmp, 'bad.bin')
    make_boot_image(bad, part_size=0)
    refused(bad, 'a zero-length uboot partition is refused')
    make_boot_image(bad, part_size=8, bcl=b'BCL1\0\0\0\x09')
    refused(bad, 'a BCL1 partition shorter than its header is refused')
    make_boot_image(bad, bcl=b'BCL1' + struct.pack('>III', 9, 0xF0000000, 8) +
                    b'\xf7' * 8)
    refused(bad, 'a BCL1 header declaring 3.75 GiB is refused')
    open(bad, 'wb').write(b'\xff' * (100 << 20))
    refused(bad, 'a 100 MiB image, neither NOR nor NAND, is refused')

    nand = os.path.join(tmp, 'boot-nand.bin')
    payload, dtb = make_boot_image(nand, nand=True)
    qt = QTest(qemu, 'nt98566,flash-file=' + nand)
    try:
        check('NAND: the plain U-Boot copied to /nvt_memory_cfg/uboot',
              qt.read(0x3900000, len(payload)), payload)
        check('the device tree from block 2', qt.read(0x100000, len(dtb)),
              dtb)
        check('SHMINFO boot.LdStorage: SPI NAND',
              struct.unpack_from('<H', qt.read(0x200000 + 42, 2))[0], 2)
    finally:
        qt.close()
    os.unlink(nand)


# ---- nt98332: the NA51103 flash boot ----------------------------------

NA51103_UBOOT = 0x1E000000


def make_nt98332_image(path, uboot_part):
    """A 16 MiB NOR laid out as Xiongmai's NA51103 images are: the device
    tree LZMA-packed behind "XMFDT32G" at 0x10000, U-Boot in
    partition_boot at 0x20000."""
    tree = {
        '#address-cells': cells(2), '#size-cells': cells(2),
        'nvt_memory_cfg': {
            '#address-cells': cells(1), '#size-cells': cells(1),
            'fdt': {'reg': cells(0x100000, 0x100000)},
            'shmem': {'reg': cells(0xA00000, 0x100000)},
            'uboot': {'reg': cells(NA51103_UBOOT, 0x2000000)},
        },
        'nor@2,f0180000': {
            'compatible': b'nvt,nvt_spinor\0',
            '#address-cells': cells(2), '#size-cells': cells(2),
            'partition_boot': {'reg': cells(0, 0x20000, 0, 0x40000)},
        },
    }
    dtb = fdt_blob(tree)
    packed = lzma.compress(dtb, format=lzma.FORMAT_ALONE)
    img = bytearray(b'\xff' * (16 << 20))
    img[0x10000:0x10008] = b'XMFDT32G'
    img[0x10008:0x10008 + len(packed)] = packed
    img[0x20000:0x20000 + len(uboot_part)] = uboot_part
    open(path, 'wb').write(img)
    return dtb


def test_nt98332_boot(qemu, tmp):
    print('nt98332 flash boot')
    path = os.path.join(tmp, 'nt98332.bin')
    rng = random.Random(11)
    # an AArch64 U-Boot's head: b reset, nop, then _TEXT_BASE as a quad
    head = struct.pack('<IIQ', 0x1400000A, 0xD503201F, NA51103_UBOOT)
    payload = head + bytes(rng.randrange(256) for _ in range(4096)) * 4

    def boot(part, what, want):
        dtb = make_nt98332_image(path, part)
        qt = QTest(qemu, 'nt98332,flash-file=' + path)
        try:
            check(what, qt.read(NA51103_UBOOT, len(want)), want)
            check('  the device tree unpacked to /nvt_memory_cfg/fdt',
                  qt.read(0x100000, len(dtb)), dtb)
            check('  SHMINFO says LD_NVT', qt.read(0xA00000, 6), b'LD_NVT')
            check('  an AArch64 branch to U-Boot at 0',
                  struct.unpack('<IIQ', qt.read(0, 16)),
                  (0x58000041, 0xD61F0020, NA51103_UBOOT))
        finally:
            qt.close()

    boot(lzma.compress(payload, format=lzma.FORMAT_ALONE),
         'U-Boot unpacked from LZMA (end marker, size unknown)', payload)
    boot(lzma.compress(payload, format=lzma.FORMAT_ALONE,
                       filters=[{'id': lzma.FILTER_LZMA1,
                                 'dict_size': 3 << 20}]),
         'LZMA with a 3 MiB dictionary', payload)
    boot(payload, 'a plain AArch64 U-Boot is not taken for LZMA', payload)
    bad = bytearray(lzma.compress(payload, format=lzma.FORMAT_ALONE))
    bad[64:96] = bytes(32)
    boot(bytes(bad), 'LZMA that does not unpack falls back to plain',
         bytes(bad[:4096]))
    os.unlink(path)


# ---- nvt-eqos ----------------------------------------------------------

def mdio_read(qt, phy, reg):
    qt.writel(ETH + 0x200, (phy << 21) | (reg << 16) | (3 << 2) | 1)
    check('MDIO read %d completes' % reg, qt.readl(ETH + 0x200) & 1, 0)
    return qt.readl(ETH + 0x204) & 0xFFFF


def test_eqos(qt, pcap):
    print('nvt-eqos')
    gic_enable(qt, SPI_ETH)
    check('HW_FEATURE0: MII and MDIO (SMASEL)',
          qt.readl(ETH + 0x11C) & 0x21, 0x21)
    check('HW_FEATURE2: one Tx and one Rx queue', qt.readl(ETH + 0x124), 0)
    check('BMSR at PHY 1: link up, autoneg done',
          mdio_read(qt, 1, 1) & 0x24, 0x24)
    check('no PHY at address 0', mdio_read(qt, 0, 1), 0xFFFF)

    qt.writel(ETH + 0x1000, 1)
    check('DMA software reset is immediate', qt.readl(ETH + 0x1000) & 1, 0)

    # One Tx descriptor at 0x00100000, a 60-byte frame at 0x00101000
    frame = bytes.fromhex('ffffffffffff525400123456') + b'\x08\x06' + \
        bytes(range(46))
    for i in range(0, len(frame), 4):
        qt.writel(0x00101000 + i,
                  int.from_bytes(frame[i:i + 4].ljust(4, b'\0'), 'little'))
    qt.writel(0x00100000, 0x00101000)                       # TDES0
    qt.writel(0x00100004, 0)                                # TDES1
    qt.writel(0x00100008, (1 << 31) | len(frame))           # IOC, length
    qt.writel(0x0010000C, (1 << 31) | (1 << 29) | (1 << 28) | len(frame))
    qt.writel(ETH + 0x1100 + 0x14, 0x00100000)              # TDLAR
    qt.writel(ETH + 0x1100 + 0x2C, 3)                       # 4 descriptors
    qt.writel(ETH + 0x1100 + 0x34, (1 << 15) | 1)           # NIE, TIE
    qt.writel(ETH + 0x0000, 3)                              # TE, RE
    qt.writel(ETH + 0x1100 + 0x04, 1)                       # ST
    qt.writel(ETH + 0x1100 + 0x08, 1)                       # SR
    check('Rx DMA running, waiting for a packet (RPS0 = 3)',
          (qt.readl(ETH + 0x100C) >> 8) & 0xF, 3)
    qt.writel(ETH + 0x1100 + 0x20, 0x00100010)              # tail: 1 desc
    tdes3 = qt.readl(0x0010000C)
    check('Tx write-back: OWN clear, FD/LD, no error bits',
          tdes3, (1 << 29) | (1 << 28))
    check('TI in the channel status', qt.readl(ETH + 0x1160) & 1, 1)
    check('SPI 34 pending', gic_pending(qt, SPI_ETH), True)
    check('ETH_ISR summarises it',
          (qt.readl(ETH + 0x3008) & 0x3F) != 0, True)
    qt.writel(ETH + 0x1160, 1)
    check('write-1-to-clear releases it', gic_pending(qt, SPI_ETH), False)
    check('and ETH_ISR with it', qt.readl(ETH + 0x3008), 0)
    qt.cmd('clock_step 1000')
    data = open(pcap, 'rb').read()
    check('the frame went out on the wire', frame in data, True)

    # A frame in two descriptors, the tail moved past only the first
    part1, part2 = frame[:20], bytes(reversed(frame[20:]))
    write_bytes(qt, 0x00102000, part1)
    write_bytes(qt, 0x00103000, part2)
    tx_desc(qt, 1, 0x00102000, len(part1), (1 << 29))              # FD
    tx_desc(qt, 2, 0x00103000, len(part2), (1 << 28))              # LD
    qt.writel(ETH + 0x1100 + 0x20, 0x00100020)
    qt.writel(ETH + 0x1100 + 0x20, 0x00100030)
    qt.cmd('clock_step 1000')
    data = open(pcap, 'rb').read()
    check('a frame split across tail writes goes out whole',
          (part1 + part2) in data, True)

    # Over-long: 2 x 8000 bytes, more than the MAC takes
    tx_desc(qt, 3, 0x00110000, 8000, (1 << 29))
    tx_desc(qt, 0, 0x00110000, 8000, (1 << 28))
    qt.writel(ETH + 0x1100 + 0x20, 0x00100010)
    check('an over-long frame is dropped with ES / JT',
          qt.readl(0x0010000C) & 0xC000, 0xC000)


def write_bytes(qt, addr, data):
    for i in range(0, len(data), 4):
        qt.writel(addr + i,
                  int.from_bytes(data[i:i + 4].ljust(4, b'\0'), 'little'))


def tx_desc(qt, idx, buf, length, flags):
    a = 0x00100000 + 16 * idx
    qt.writel(a, buf)
    qt.writel(a + 4, 0)
    qt.writel(a + 8, length)
    qt.writel(a + 12, (1 << 31) | flags | length)


MAC = bytes.fromhex('525400123456')


def test_eqos_rx(qt, sock, port):
    print('nvt-eqos receive')
    ring, bufs = 0x00200000, 0x00210000
    qt.writel(ETH + 0x300, 0x80000000 | MAC[4] | (MAC[5] << 8))
    qt.writel(ETH + 0x304, int.from_bytes(MAC[:4], 'little'))
    for i in range(4):
        qt.writel(ring + 16 * i, bufs + 0x100 * i)
        qt.writel(ring + 16 * i + 12, (1 << 31) | (1 << 30) | (1 << 24))
    qt.writel(ETH + 0x1100 + 0x1C, ring)                    # RDLAR
    qt.writel(ETH + 0x1100 + 0x30, 3)                       # 4 descriptors
    qt.writel(ETH + 0x1100 + 0x28, ring + 0x30)             # tail: desc 3
    qt.writel(ETH + 0x0000, 3)
    qt.writel(ETH + 0x1100 + 0x08, 1 | (64 << 1))           # SR, RBSZ 64

    def send(frame):
        sock.sendto(frame, ('127.0.0.1', port))
        for _ in range(20):
            qt.cmd('clock_step 1000000')

    def rdes3(i):
        return qt.readl(ring + 16 * i + 12)

    other = bytes.fromhex('020000000001') + MAC + b'\x08\x00' + bytes(50)
    send(other)
    check('a frame for another station is filtered out',
          rdes3(0) >> 31, 1)
    big = MAC + bytes.fromhex('020000000002') + b'\x08\x00' + \
        bytes(range(86))
    send(big)
    check('a 100-byte frame spans two 64-byte buffers: FD on the first',
          rdes3(0) & 0xB0000000, 0x20000000)
    check('LD and the frame length on the second',
          rdes3(1) & 0xB0007FFF, 0x10000000 | len(big))
    got = b''.join(qt.readl(bufs + i).to_bytes(4, 'little')
                   for i in range(0, 64, 4)) + \
        b''.join(qt.readl(bufs + 0x100 + i).to_bytes(4, 'little')
                 for i in range(0, 36, 4))
    check('and the bytes are the frame', got[:len(big)], big)
    small = MAC + bytes.fromhex('020000000003') + b'\x08\x00' + bytes(30)
    send(small)
    check('the next frame takes descriptor 2', rdes3(2) >> 28 & 3, 3)
    send(small)
    check('descriptor 3 is the tail: the DMA stops short of it',
          rdes3(3) >> 31, 1)
    qt.writel(ETH + 0x1100 + 0x28, ring)                    # tail: desc 0
    for _ in range(20):
        qt.cmd('clock_step 1000000')
    check('and fills it once the tail moves on', rdes3(3) >> 28 & 3, 3)


# ldr r0,=0xF0290000; FCR = 7; 1: wait for LSR.DR; RBR -> THR; b 1b
UART_ECHO = [0xE59F001C, 0xE3A01007, 0xE5801008, 0xE5901014, 0xE3110001,
             0x0AFFFFFC, 0xE5902000, 0xE5802000, 0xEAFFFFF9, 0xF0290000]


def test_uart_rx_after_fifo_reset(qemu, tmp):
    """Bytes queued on the backend before the guest initialises its UART:
    one lands in RBR (FIFO off) and the backend stops polling.  The guest
    then resets the receive FIFO and spins on LSR, never reading RBR -- as
    U-Boot's ns16550 tstc() does at "Hit any key to stop autoboot".  A
    spinning vCPU makes no main-loop events, so without
    qemu/patches/0003 nothing more ever arrives.  Needs a real guest:
    qtest's own commands wake the main loop and hide the stall."""
    print('UART input across a receive-FIFO reset')
    kernel = os.path.join(tmp, 'echo.bin')
    open(kernel, 'wb').write(struct.pack('<%dI' % len(UART_ECHO), *UART_ECHO))
    ser_path = os.path.join(tmp, 'echo-ser.sock')
    mon_path = os.path.join(tmp, 'echo-mon.sock')
    proc = subprocess.Popen(
        [qemu, '-M', 'nt98562', '-display', 'none', '-nodefaults', '-S',
         '-kernel', kernel,
         '-serial', 'unix:%s,server=on,wait=off' % ser_path,
         '-monitor', 'unix:%s,server=on,wait=off' % mon_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(100):
            if os.path.exists(ser_path) and os.path.exists(mon_path):
                break
            time.sleep(0.05)
        ser = socket.socket(socket.AF_UNIX)
        ser.connect(ser_path)
        mon = socket.socket(socket.AF_UNIX)
        mon.connect(mon_path)
        ser.sendall(b'abc')
        # Still paused: wait until 'a' really sits in RBR (LSR.DR, read
        # through the monitor, which does not touch RBR) -- the state the
        # stall needs -- before the guest runs.
        mon.settimeout(0.2)
        dr = False
        for _ in range(100):
            mon.sendall(b'xp /1wx 0x%x\n' % (UART0 + 0x14))
            time.sleep(0.05)
            try:
                if re.search(rb': 0x[0-9a-f]*[13579bdf]\b', mon.recv(4096)):
                    dr = True
                    break
            except socket.timeout:
                pass
        check('a byte waits in RBR before the guest runs', dr, True)
        mon.sendall(b'cont\n')
        got = b''
        ser.settimeout(0.2)
        end = time.time() + 3
        while time.time() < end and b'c' not in got:
            try:
                got += ser.recv(64)
            except socket.timeout:
                pass
        # 'a' sat in RBR and went with the FIFO reset, as on silicon
        check('input queued before UART init is echoed after its FIFO '
              'reset', got, b'bc')
        mon.sendall(b'quit\n')
        proc.wait(timeout=10)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def find_nodes(dtb, names):
    """Offsets of FDT_BEGIN_NODE tokens naming these nodes, -1 if absent."""
    return [dtb.find(b'\0\0\0\x01' + n.encode() + b'\0') for n in names]


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
    check('the USB host and OTP nodes ehci-na51089.c needs',
          b'nvt,ehci-nvtivot\0' in data and b'nvt,nvt_otp\0' in data, True)
    check('the PWM node pwm-na51055.c binds', b'nvt,nvt_pwm\0' in data, True)
    check('the HDAL media nodes, units and limits',
          find_nodes(data, ['sie@f0c00000', 'ime@f0c40000', 'h26x@f0a10000',
                            'jpg@f0a00000', 'dai@f0630000', 'nvtmpp',
                            'isf_vdoenc', 'hdal-maxpath-cfg', 'hdal-memory',
                            'pll_preset@0']).count(-1), 0)
    check('the crypto, hash and RSA nodes the na51089 drivers bind',
          find_nodes(data, ['crypto@f0620000', 'hash@f0670000',
                            'rsa@f06a0000']).count(-1) == 0 and
          all(b'nvt,nvt_%s\0' % n in data for n in (b'crypto', b'hash', b'rsa')),
          True)
    check('the timer node kdrv_comm.ko binds: timer@f0040000, nvt,nvt_timer',
          find_nodes(data, ['timer@f0040000'])[0] > 0 and
          b'nvt,nvt_timer\0' in data, True)
    check('the eMMC host node: mmc@f0510000, nvt,nvt_mmc3',
          find_nodes(data, ['mmc@f0510000'])[0] > 0 and
          b'nvt,nvt_mmc3\0' in data, True)
    check('three I2C nodes under the names i2c-nvt.c clocks by',
          find_nodes(data, ['i2c@f0220000', 'i2c2@f0350000',
                            'i2c3@f03a0000']).count(-1), 0)
    check('the NOR under spinor.c', b'nvt,nvt_spinor\0' in data and
          b'spi_nor.0\0' in data, True)

    subprocess.run([qemu, '-M', 'nt98562', '-m', '64M',
                    '-machine', 'dumpdtb=' + dtb, '-kernel', kernel,
                    '-display', 'none', '-nodefaults'],
                   capture_output=True, timeout=30)
    small = open(dtb, 'rb').read()
    check('-m 64: the SDK split, Linux 30 MB, the media pool the rest',
          (find_nodes(small, ['hdal-memory'])[0] > 0,
           cells(0, 0x01E00000) in small,
           cells(0x01E00000, 0x02200000) in small), (True, True, True))
    check('224 MB: Linux 128 MB, the pool 96 MB after it',
          (cells(0, 0x08000000) in data, cells(0x08000000, 0x06000000) in data),
          (True, True))

    nand = os.path.join(tmp, 'fdt-nand.bin')
    with open(nand, 'wb') as f:
        f.truncate(NAND_SIZE)
    subprocess.run([qemu, '-M', 'nt98566,flash-file=' + nand,
                    '-machine', 'dumpdtb=' + dtb, '-kernel', kernel,
                    '-display', 'none', '-nodefaults'],
                   capture_output=True, timeout=30)
    data = open(dtb, 'rb').read()
    check('a NAND image gets the node spinand.c binds',
          b'nvt,nvt_spinand\0' in data and b'spi_nand.0\0' in data and
          b'nvt,nvt_spinor' not in data, True)
    os.unlink(nand)


def hmp(qemu, args, cmds):
    """Run QEMU with an HMP monitor on stdio; return what it printed."""
    p = subprocess.run([qemu] + args + ['-display', 'none', '-nodefaults',
                                        '-monitor', 'stdio'],
                       input='\n'.join(cmds + ['quit']) + '\n',
                       capture_output=True, text=True, timeout=60)
    return p.stdout + p.stderr


def test_migration(qemu, tmp):
    print('migration')
    for m in ('nt98562', 'nt98566', 'nt98566,sensor=sc401ai'):
        mig = os.path.join(tmp, m.replace(',', '_') + '.mig')
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
    ap.add_argument('--qemu-aarch64',
                    default='./qemu-src/build/qemu-system-aarch64')
    args = ap.parse_args()
    # Images up to 128 MiB go here, under /tmp: always remove them.
    with tempfile.TemporaryDirectory() as tmp:
        run(args, tmp)


def run(args, tmp):

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

    flash = os.path.join(tmp, 'flash.bin')
    open(flash, 'wb').write(random.Random(3).randbytes(8 << 20))
    # -kernel keeps the boot ROM, which wants an NVT image, out of it
    stub = os.path.join(tmp, 'stub.bin')
    open(stub, 'wb').write(b'\xfe\xff\xff\xea')     # b .
    qt = QTest(args.qemu, 'nt98562,flash-file=' + flash, ['-kernel', stub])
    try:
        test_flash(qt, flash)
    finally:
        qt.close()
    test_boot_rom(args.qemu, tmp)

    for with_oob in (False, True):
        nand = os.path.join(tmp, 'nand.bin')
        make_nand(nand, with_oob)
        qt = QTest(args.qemu, 'nt98566,flash-file=' + nand, ['-kernel', stub])
        try:
            test_nand(qt, nand, with_oob)
        finally:
            qt.close()
        os.unlink(nand)

    pcap = os.path.join(tmp, 'eth.pcap')
    qt = QTest(args.qemu, 'nt98562',
               ['-netdev', 'user,id=n0,restrict=on',
                '-nic', 'none',
                '-global', 'nvt-eqos.netdev=n0',
                '-object', 'filter-dump,id=d0,netdev=n0,file=' + pcap])
    try:
        test_eqos(qt, pcap)
    finally:
        qt.close()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    peer = sock.getsockname()[1]
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(('127.0.0.1', 0))
    port = probe.getsockname()[1]
    probe.close()
    qt = QTest(args.qemu, 'nt98562',
               ['-netdev', 'socket,id=n0,udp=127.0.0.1:%d,'
                'localaddr=127.0.0.1:%d' % (peer, port),
                '-nic', 'none', '-global', 'nvt-eqos.netdev=n0'])
    try:
        test_eqos_rx(qt, sock, port)
    finally:
        qt.close()
        sock.close()

    card = os.path.join(tmp, 'sd.img')
    open(card, 'wb').write(random.Random(9).randbytes(4 << 20))
    qt = QTest(args.qemu, 'nt98562',
               ['-drive', 'if=sd,index=0,format=raw,file=' + card])
    try:
        test_sdio(qt, card)
    finally:
        qt.close()

    emmc = os.path.join(tmp, 'emmc.img')
    open(emmc, 'wb').write(bytes(4 << 20))
    qt = QTest(args.qemu, 'nt98566',
               ['-drive', 'if=sd,index=2,format=raw,file=' + emmc])
    try:
        test_emmc(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566,sensor=sc401ai')
    try:
        test_i2c(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_pwm(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98562')
    try:
        test_timer(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_media(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_pixels(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_gfx(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_crypto(qt)
        test_hash(qt)
        test_rsa(qt)
    finally:
        qt.close()

    stick = os.path.join(tmp, 'stick.img')
    open(stick, 'wb').write(bytes(1 << 20))
    qt = QTest(args.qemu, 'nt98566',
               ['-drive', 'if=none,id=s,format=raw,file=' + stick,
                '-device', 'usb-storage,drive=s'])
    try:
        test_usb(qt, True)
        test_otp(qt)
    finally:
        qt.close()

    qt = QTest(args.qemu, 'nt98566')
    try:
        test_usb(qt, False)
        print('nt98566')
        check('same die, same chip ID', qt.readl(TOP + 0xF0) >> 16, 0x7021)
        check('no card: C_GPIO 9 high', qt.readl(GPIO + 0x00) >> 9 & 1, 1)
    finally:
        qt.close()

    locks = os.path.join(tmp, 'locks.bin')
    open(locks, 'wb').write(random.Random(13).randbytes(8 << 20))
    qt = QTest(args.qemu, 'nt98562,flash-file=' + locks, ['-kernel', stub])
    try:
        test_flash_locks(qt, locks)
    finally:
        qt.close()
    os.unlink(locks)

    if os.path.exists(args.qemu_aarch64):
        test_nt98332_boot(args.qemu_aarch64, tmp)
    else:
        print('nt98332: %s not built, skipped' % args.qemu_aarch64)

    test_uart_rx_after_fifo_reset(args.qemu, tmp)
    test_fdt(args.qemu, tmp)
    test_migration(args.qemu, tmp)

    print('%d failure(s)' % failures)
    if failures:
        sys.exit(1)


if __name__ == '__main__':
    main()
