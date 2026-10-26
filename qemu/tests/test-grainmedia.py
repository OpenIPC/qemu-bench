#!/usr/bin/env python3
"""Register-level checks of the Grain Media GM813x models, no guest needed.

QEMU runs the gm8136 machine under qtest and this script plays the vendor
loaders and drivers (docs/grainmedia.md):

  - the mask ROM: nsboot from flash 0x1000 staged in SRAM, the stub at 0
    that enters it;
  - gm-pmu: chip ID and NOR boot strap, the frequency change nsboot
    waits for in WFI (PMODE bit 2 -> interrupt 8, cleared through 0x20);
  - gm-intc: an interrupt routed through the CPU 0 target register,
    level and edge inputs, group 1's registers at 0x20;
  - gm-ddrc: the DRAM size code U-Boot picks its command line by, and the
    initialisation nsboot polls for;
  - gm-timer: a periodic channel at PCLK (100 MHz from the PLL defaults)
    raising its overflow status, the kernel's one-shot (LOAD 0), the
    clocksource counting down;
  - gm-spi020 + the MX25L12805D: the JEDEC ID waited for before it is
    drained (U-Boot's order), a fast read by PIO through the RX FIFO, a
    hardware-polled status read, and a fast read drained by the DMAC on
    request line 8, as the kernel does;
  - gm-dmac: a memory-to-memory copy through a linked list;
  - gm-gpio: undriven inputs read high, outputs read back;
  - gm-i2c + sensor=ov2715: the chip ID read byte by byte as the ISP
    module reads it, an absent address NAKed;
  - gm-wdt: an expiry in interrupt mode;
  - gm-otg: the OTG role and port speed, the EHCI behind it;
  - gm-h264e: SPS, PPS and slice headers packed through the header FIFO
    as favc_enc.ko does, an I and a P slice coded from a packed 4:2:2
    picture in CAVLC (Baseline) and in CABAC (Main) -- decoded with
    ffmpeg when --ffmpeg names one (or it is on PATH): every frame
    decodes, and the I frame's macroblock means match the source;
  - gm-vcap: the capability word gmlib reads, a link-list table run on
    the sync timer painting a frame and completing its status slot;
  - gm-3dnr / gm-scaler: a job through each engine's link list;
  - the SD hosts' SDHCI capabilities;
  - every machine: its chip ID; and a migration round trip.

    python3 qemu/tests/test-grainmedia.py [--qemu PATH]
"""
import argparse
import os
import shlex
import shutil
import socket
import struct
import subprocess
import sys
import tempfile

PMU = 0x90C00000
TIMER = 0x90D00000
WDT = 0x90E00000
GPIO0 = 0x91000000
I2C = 0x91200000
SRAM = 0x92000000
SPI = 0x92300000
DMAC = 0x92600000
SDC0 = 0x92800000
OTG = 0x93000000
INTC = 0x96000000
DDRC = 0x9A100000
H264E = 0x92C00000
VCAP = 0x96100000
DNR = 0x9B700000
SCALER = 0x96200000
MCP100 = 0x92020000
MCP_NODE = 0x92008000
DMAWRP = 0x92400000
GM2D = 0x92200000
SSP1 = 0x91400000
AES = 0x92700000
SARADC = 0x90A00000
RTC = 0x91C00000
PWM = 0x91B00000
IRDET = 0x91600000
PMON = 0x91D00000

JEDEC = [0xC2, 0x20, 0x18]          # MX25L12805D, for a 16 MiB image
SCRATCH = 0x01000000


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


def make_image(path):
    """16 MiB: the GM8136 system header, an "nsboot" at 0x1000 that
    branches to itself, and a pattern from 0x100000 on."""
    img = bytearray(b'\xFF' * (16 << 20))
    img[0:0x100] = bytes(0x100)
    img[0:6] = b'GM8136'
    struct.pack_into('<3I', img, 8, 0x10000, 0x50000, 0)
    img[0xFE:0x100] = b'\x55\xAA'
    struct.pack_into('<I', img, 0x1000, 0xEAFFFFFE)        # b .
    for i in range(0x100000, 0x110000):
        img[i] = i * 7 & 0xFF
    open(path, 'wb').write(img)
    return img


def test_boot_rom(qt):
    print('mask ROM')
    check('stub at 0: ldr pc, [pc, #-4]', qt.readl(0), 0xE51FF004)
    check('entering SRAM', qt.readl(4), SRAM)
    check('nsboot from flash 0x1000 in SRAM', qt.readl(SRAM), 0xEAFFFFFE)


def test_pmu_intc(qt):
    print('gm-pmu, gm-intc')
    check('chip ID (GM8136, revision B)', qt.readl(PMU), 0x08136100)
    check('boot strap: SPI NOR', qt.readl(PMU + 0x04) & (1 << 7), 0)
    qt.writel(INTC + 0x04, 0xFFFFFFFF)          # what the kernel enables
    qt.writel(INTC + 0x45C, 1 << 8)             # nsboot routes the FCS
    qt.writel(PMU + 0x0C, 1 << 2)               # start the change
    check('FCS done in PMU 0x20', qt.readl(PMU + 0x20), 1 << 17)
    check('interrupt 8 pending', qt.readl(INTC + 0x14), 1 << 8)
    qt.writel(PMU + 0x20, 1 << 17)
    check('cleared through PMU 0x20', qt.readl(INTC + 0x14), 0)
    check('group 1 enable at 0x24', qt.readl(INTC + 0x24), 0)
    qt.writel(INTC + 0x24, 0xFFFFFFFF)
    check('group 1 enable written', qt.readl(INTC + 0x24), 0xFFFFFFFF)


def test_ddrc(qt):
    print('gm-ddrc')
    check('size code 3 (128 MiB) in 0x10', qt.readl(DDRC + 0x10) & 7, 3)
    qt.writel(DDRC + 0x10, 0x5454)              # nsboot's 256 MiB table
    check('a table value replaces it', qt.readl(DDRC + 0x10) & 7, 4)
    qt.writel(DDRC + 0x04, 1)
    check('initialisation done (bit 8)', qt.readl(DDRC + 0x04) & 0x100, 0x100)


def test_timer(qt):
    print('gm-timer')
    qt.writel(INTC + 0x04, 0xFFFFFFFF)
    qt.writel(INTC + 0x45C, 1 << 14)
    qt.writel(TIMER + 0x04, 100000 - 1)         # 1 ms at 100 MHz PCLK
    qt.writel(TIMER + 0x00, 100000 - 1)
    qt.writel(TIMER + 0x30, 0x5)                # timer 1 on, overflow irq
    v = qt.readl(TIMER + 0x00)
    check('timer 1 counts down from COUNTER', 0 < v < 100000, True)
    qt.step(500000)
    check('no overflow at 0.5 ms', qt.readl(TIMER + 0x34) & 4, 0)
    qt.step(600000)
    check('overflow status at 1.1 ms', qt.readl(TIMER + 0x34) & 4, 4)
    check('interrupt 14 pending', qt.readl(INTC + 0x14) & (1 << 14),
          1 << 14)
    qt.writel(TIMER + 0x34, 0x1FF)
    check('write 1 clears', qt.readl(INTC + 0x14) & (1 << 14), 0)
    # the kernel's one-shot: LOAD 0, COUNTER = delta
    qt.writel(TIMER + 0x04, 0)
    qt.writel(TIMER + 0x00, 1000)
    qt.step(20000)
    check('one-shot fires', qt.readl(TIMER + 0x34) & 4, 4)
    qt.writel(TIMER + 0x34, 0x1FF)
    qt.step(2000000)
    check('and holds at 0 without firing again',
          'status %d count %d' % (qt.readl(TIMER + 0x34) & 4,
                                  qt.readl(TIMER + 0x00)),
          'status 0 count 0')
    # the clocksource: timer 2 free-running down from ~0
    qt.writel(TIMER + 0x14, 0xFFFFFFFF)
    qt.writel(TIMER + 0x10, 0xFFFFFFFF)
    qt.writel(TIMER + 0x30, 0x5 | 0x8)
    a = qt.readl(TIMER + 0x10)
    qt.step(1000000)
    b = qt.readl(TIMER + 0x10)
    check('timer 2 runs at 100 MHz', 99000 <= a - b <= 101000, True)


def spi_cmd(qt, op, addr=0, alen=0, dummies=0, count=0, write=False,
            status=False):
    qt.writel(SPI + 0x00, addr)
    qt.writel(SPI + 0x04, (1 << 24) | (dummies << 16) | alen)
    qt.writel(SPI + 0x08, count)
    qt.writel(SPI + 0x0C, (op << 24) | (int(status) << 2) |
              (int(write) << 1) | 1)


def test_spi(qt, img):
    print('gm-spi020 + MX25L12805D')
    check('FEATURE: 32-word FIFOs', (qt.readl(SPI + 0x54) >> 8) & 0xFF, 32)
    spi_cmd(qt, 0x9F, count=3)
    check('a 3-byte ID read completes into the FIFO',
          qt.readl(SPI + 0x24) & 1, 1)
    qt.writel(SPI + 0x24, 1)
    check('RX ready', qt.readl(SPI + 0x18) & 2, 2)
    check('JEDEC ID drained afterwards',
          [qt.readb(SPI + 0x100) for _ in range(3)], JEDEC)
    check('FIFO empty', qt.readl(SPI + 0x18) & 2, 0)

    # nsboot's PIO fast read: 8 dummy cycles, then 32-bit pops
    spi_cmd(qt, 0x0B, addr=0x100000, alen=3, dummies=8, count=512)
    check('a long read waits on the FIFO', qt.readl(SPI + 0x24) & 1, 0)
    data = b''.join(struct.pack('<I', qt.readl(SPI + 0x100))
                    for _ in range(128))
    check('fast read by PIO', data, bytes(img[0x100000:0x100200]))
    check('complete after the last pop', qt.readl(SPI + 0x24) & 1, 1)
    qt.writel(SPI + 0x24, 1)

    spi_cmd(qt, 0x05, status=True)              # hardware-polled status
    check('status read: not busy', qt.readl(SPI + 0x28) & 1, 0)
    check('and complete', qt.readl(SPI + 0x24) & 1, 1)
    qt.writel(SPI + 0x24, 1)

    # the kernel's DMA read: channel 0 from the data port, request line 8
    qt.writel(SPI + 0x20, 0x3)                  # DMA handshake + interrupt
    qt.writel(DMAC + 0x24, 1)
    qt.writel(DMAC + 0x108, SPI + 0x100)        # SRC: the data port
    qt.writel(DMAC + 0x10C, SCRATCH)
    qt.writel(DMAC + 0x110, 0)
    qt.writel(DMAC + 0x114, 0x1000)             # bytes (8-bit width)
    qt.writel(DMAC + 0x104, (1 << 7) | (8 << 3))   # source request 8
    qt.writel(DMAC + 0x100, (2 << 5) | (1 << 7) | 1)  # src fixed, hs, on
    spi_cmd(qt, 0x0B, addr=0x100000, alen=3, dummies=8, count=0x1000)
    qt.step(1000)
    check('DMA drained the read', qt.read(SCRATCH, 0x1000),
          bytes(img[0x100000:0x101000]))
    check('channel 0 terminal count', qt.readl(DMAC + 0x14) & 1, 1)
    check('command complete: interrupt 54 (group 1 bit 22)',
          qt.readl(INTC + 0x34) & (1 << 22), 1 << 22)
    qt.writel(DMAC + 0x08, 0xFF)
    qt.writel(SPI + 0x24, 1)
    qt.writel(SPI + 0x20, 0)


def test_dmac(qt):
    print('gm-dmac')
    src, dst, lld = SCRATCH + 0x10000, SCRATCH + 0x20000, SCRATCH + 0x30000
    pattern = bytes((i * 13) & 0xFF for i in range(512))
    qt.write(src, pattern)
    # second block: the next 256 bytes, 32-bit wide, no TC mask
    ctrl = (2 << 22) | (2 << 25)
    qt.write(lld, struct.pack('<5I', src + 256, dst + 256, 0, ctrl, 64))
    qt.writel(DMAC + 0x24, 1)
    qt.writel(DMAC + 0x128, src)                # channel 1
    qt.writel(DMAC + 0x12C, dst)
    qt.writel(DMAC + 0x130, lld | 1)
    qt.writel(DMAC + 0x134, 64)                 # 64 words
    qt.writel(DMAC + 0x124, 0)
    qt.writel(DMAC + 0x120, (1 << 31) | (2 << 11) | (2 << 8) | 1)
    qt.step(1000)
    check('two-block linked list copied', qt.read(dst, 512), pattern)
    check('one terminal count, channel 1', qt.readl(DMAC + 0x14), 2)
    check('channel disabled again', qt.readl(DMAC + 0x1C) & 2, 0)
    qt.writel(DMAC + 0x08, 0xFF)


def test_gpio(qt):
    print('gm-gpio')
    check('undriven inputs read high', qt.readl(GPIO0 + 0x04), 0xFFFFFFFF)
    qt.writel(GPIO0 + 0x08, 0x0000FFFF)
    qt.writel(GPIO0 + 0x00, 0x00001234)
    check('outputs read back', qt.readl(GPIO0 + 0x04), 0xFFFF1234)


def i2c_byte(qt, data=None, start=False, stop=False, nak=False):
    if data is not None:
        qt.writel(I2C + 0x0C, data)
    qt.writel(I2C + 0x00, 0x6 | 0x80 | (start << 4) | (stop << 5) |
              (nak << 6))
    qt.step(20000)
    return qt.readl(I2C + 0x04)


def test_i2c(qt):
    print('gm-i2c + OV2715')
    qt.writel(I2C + 0x00, 1)
    check('reset clears itself', qt.readl(I2C + 0x00) & 1, 0)
    sr = i2c_byte(qt, 0x6C, start=True)         # OV2715 write
    check('address ACKed: byte transmitted', sr & 0x10, 0x10)
    i2c_byte(qt, 0x30)
    i2c_byte(qt, 0x0A)
    sr = i2c_byte(qt, 0x6D, start=True)         # repeated START, read
    check('read address ACKed', sr & 0x10, 0x10)
    i2c_byte(qt)
    hi = qt.readl(I2C + 0x0C)
    sr = i2c_byte(qt, stop=True, nak=True)
    lo = qt.readl(I2C + 0x0C)
    check('chip ID 0x300A/B', (hi << 8) | lo, 0x2715)
    check('received and stopped', sr & 0xA0, 0xA0)
    sr = i2c_byte(qt, 0x42, start=True)
    check('absent address: bus error (NAK)', sr & 0x40, 0x40)


def test_wdt(qt):
    print('gm-wdt')
    qt.writel(INTC + 0x45C, 1 << 16)
    qt.writel(WDT + 0x04, 100000)               # 1 ms at PCLK
    qt.writel(WDT + 0x08, 0x5AB9)
    qt.writel(WDT + 0x0C, 0x5)                  # enable, interrupt only
    qt.step(500000)
    check('not yet at 0.5 ms', qt.readl(WDT + 0x10), 0)
    qt.step(700000)
    check('timed out', qt.readl(WDT + 0x10), 1)
    check('interrupt 16 pending', qt.readl(INTC + 0x14) & (1 << 16),
          1 << 16)
    qt.writel(WDT + 0x0C, 0)
    qt.writel(WDT + 0x14, 1)
    check('status cleared', qt.readl(WDT + 0x10), 0)


def test_usb_sd(qt):
    print('gm-otg, SDHCI')
    check('EHCI HCIVERSION 1.00', qt.readl(OTG) >> 16, 0x0100)
    csr = qt.readl(OTG + 0x80)
    check('A device, host role', csr & (3 << 20), 0)
    check('port speed: high', (csr >> 22) & 3, 2)
    check('SDHCI: 3.3 V and ADMA2', qt.readl(SDC0 + 0x40) & 0x01080000,
          0x01080000)


# ---- the H.264 encoder ----------------------------------------------------

class Hdr:
    """Header FIFO entries, as enc_driver/vlc.c builds them."""
    def __init__(self):
        self.e = []

    def u(self, v, n):
        self.e.append((0x40 | n, v))

    def u_prev(self, v, n):            # no emulation prevention
        self.e.append((n, v))

    def ue(self, v):
        self.e.append((0xC0, v))

    def se(self, v):
        self.ue(2 * v - 1 if v > 0 else -2 * v)

    def trailing(self):
        c, v = self.e[-1]
        self.e[-1] = (c | 0x100, v)

    def pack(self, qt):
        assert len(self.e) <= 32               # the FIFO's pairs
        c, v = self.e[-1]
        self.e[-1] = (c | 0x200, v)
        for i, (c, v) in enumerate(self.e):
            qt.writel(H264E + 0x400 + 8 * i, c)
            qt.writel(H264E + 0x404 + 8 * i, v)
        qt.writel(H264E + 0x88, 1 << 12)
        self.e = []


def h264_stream(qt, cabac, mbw, mbh, srcs, idc=0):
    """An IDR I picture and a P picture, the way slice.c drives a frame."""
    bs = SCRATCH + 0x100000
    qt.writel(H264E + 0x144, 0x80000)
    qt.writel(H264E + 0x148, bs)
    qt.writel(H264E + 0x88, 1 << 11)            # clear the count
    out = b''
    ref = SCRATCH + 0x80000                     # REC == ME_REF, one buffer
    for idr, src in zip((True, False), srcs):
        h = Hdr()
        if idr:                                 # SPS, PPS
            h.u_prev(0, 16); h.u_prev(1, 16); h.u_prev(0x67, 8)
            h.u(77 if cabac else 66, 8); h.u(0, 8); h.u(31, 8)
            h.ue(0); h.ue(0); h.ue(2); h.ue(1); h.u(0, 1)
            h.ue(mbw - 1); h.ue(mbh - 1); h.u(1, 1); h.u(1, 1)
            h.u(0, 1); h.u(0, 1); h.trailing()
            h.pack(qt)
            h.u_prev(0, 16); h.u(1, 16); h.u_prev(0x68, 8)
            h.ue(0); h.ue(0); h.u(int(cabac), 1); h.u(0, 1); h.ue(0)
            h.ue(0); h.ue(0); h.u(0, 1); h.u(0, 2); h.se(0); h.se(0)
            h.se(0); h.u(1, 1); h.u(0, 1); h.u(0, 1); h.trailing()
            h.pack(qt)
        h.u_prev(0, 16); h.u_prev(1, 16); h.u_prev(0x65 if idr else 0x41, 8)
        h.ue(0); h.ue(7 if idr else 5); h.ue(0); h.u(0 if idr else 1, 4)
        if idr:
            h.ue(0)
        else:
            h.u(0, 1); h.u(0, 1)                # ref idx override, reorder
        h.u(0, 1)
        if idr:
            h.u(0, 1)
        if cabac and not idr:
            h.ue(idc)                           # cabac_init_idc
        h.se(0)                                 # QP 26
        h.ue(1)                                 # deblocking off
        h.pack(qt)
        qt.writel(H264E + 0x00, ((mbh - 1) << 24) | ((mbw - 1) << 8) |
                  (mbw - 1))
        qt.writel(H264E + 0x04, ((mbh - 1) << 24) | ((2 if idr else 0) << 12))
        qt.writel(H264E + 0x0C, 26 << 26)
        qt.writel(H264E + 0x14, 1 << 31)
        qt.writel(H264E + 0x18, int(cabac) | (idc << 1))
        qt.writel(H264E + 0x100, src)
        for r in (0x108, 0x120):                # luma: REF, REC
            qt.writel(H264E + r, ref)
        for r in (0x10C, 0x124):                # chroma (NV12)
            qt.writel(H264E + r, ref + mbw * mbh * 256)
        qt.writel(H264E + 0x80, 1)
        qt.step(2000000)
        sts = qt.readl(H264E + 0xC8)
        check('%s %s slice done' % ('CABAC' if cabac else 'CAVLC',
                                    'I' if idr else 'P'), sts & 3, 1)
        qt.writel(H264E + 0xC8, 0x1F)
        n = qt.readl(H264E + 0xC4) & 0xFFFFFF
        out += qt.read(bs, n)
        qt.writel(H264E + 0x88, 1 << 11)
    return out


def test_h264e(qt, ffmpeg, tmp):
    print('gm-h264e')
    check('STS0: DMA idle, packer done', qt.readl(H264E + 0xC0) & 0xA, 0xA)
    check('VCACHE DMA stopped', qt.readl(0x92D00004) & 2, 2)
    mbw, mbh = 4, 3
    w, h = mbw * 16, mbh * 16
    ys = [16 + (x * 7 + y * 3) % 220 for y in range(h) for x in range(w)]
    pic = bytearray()
    for y in range(h):
        for x in range(0, w, 2):
            cb, cr = 64 + x * 2, 200 - y * 2
            pic += bytes((cb, ys[y * w + x], cr, ys[y * w + x + 1]))
    src = SCRATCH + 0x40000
    qt.write(src, bytes(pic))
    # the P picture: macroblock (2, 1) turns a flat 200
    ys2 = list(ys)
    pic2 = bytearray(pic)
    for y in range(16, 32):
        for x in range(32, 48):
            ys2[y * w + x] = 200
            pic2[(y * w + x) * 2 + 1] = 200
    src2 = SCRATCH + 0x60000
    qt.write(src2, bytes(pic2))
    for cabac, idc in ((False, 0), (True, 0), (True, 1), (True, 2)):
        stream = h264_stream(qt, cabac, mbw, mbh, (src, src2), idc)
        name = 'cabac%d' % idc if cabac else 'cavlc'
        check('%s stream starts with an SPS' % name, stream[:5],
              b'\x00\x00\x00\x01\x67')
        path = os.path.join(tmp, name + '.h264')
        open(path, 'wb').write(stream)
        if not ffmpeg:
            print('  SKIP: no ffmpeg to decode %s' % name)
            continue
        r = subprocess.run(shlex.split(ffmpeg) + [
            '-v', 'error', '-i', path, '-f', 'rawvideo', '-pix_fmt', 'yuv420p',
            '-'], capture_output=True)
        fs = w * h * 3 // 2                     # luma as coded, no rescale
        y = r.stdout[:w * h] + r.stdout[fs:fs + w * h] \
            if len(r.stdout) == 2 * fs else b''
        errs = [l for l in r.stderr.decode(errors='replace').splitlines()
                if l.strip() and not l.startswith('docker:')]
        check('%s: decodes without errors' % name,
              r.returncode == 0 and not errs, True)
        for l in errs[:5]:
            print('    ' + l)
        check('%s: two frames' % name, len(y), 2 * w * h)
        if len(y) != 2 * w * h:
            continue
        worst = 0
        for my in range(mbh):
            for mx in range(mbw):
                blk = [(ys[(my * 16 + j) * w + mx * 16 + i],
                        y[(my * 16 + j) * w + mx * 16 + i])
                       for j in range(16) for i in range(16)]
                d = abs(sum(a for a, _ in blk) - sum(b for _, b in blk)) // 256
                worst = max(worst, d)
        check('%s: macroblock means within 3 (worst %d)' % (name, worst),
              worst <= 3, True)
        p, i = y[w * h:], y[:w * h]
        moved = [(yy, xx) for yy in range(16, 32) for xx in range(32, 48)]
        check('%s: P frame: the changed macroblock exact (I_PCM)' % name,
              all(p[yy * w + xx] == 200 for yy, xx in moved), True)
        check('%s: P frame: the rest repeats the I frame (P_Skip)' % name,
              all(p[k] == i[k] for k in range(w * h)
                  if (k // w, k % w) not in set(moved)), True)


# ---- capture, 3DNR, scaler -------------------------------------------------

def test_vcap(qt):
    print('gm-vcap')
    check('version', qt.readl(VCAP + 0x55E8), 0x20140311)
    check('capability: scaling, four scalers', qt.readl(VCAP + 0x55EC) & 0xFF,
          0x74)
    cas, lli = VCAP + 0x3E00, VCAP + 0x40000
    buf = SCRATCH + 0x200000
    # path 0: a 64x16 target crop at buf
    tbl = [(0, 0x20000000 | (32 << 23)),                        # STATUS
           (buf, 0x40780000 | (0x3E00 + 0x198)),                # DMA
           (0, 0x40780000 | (0x3E00 + 0x19C)),
           (63 << 16, 0x40780000 | (0x3E00 + 0x160)),           # crop x
           (15 << 16, 0x40780000 | (0x3E00 + 0x164)),           # crop y
           (2, 0x40080000 | 0x3E00),                            # path on
           (0, 0x80000000 | (0x40000 + 0x100))]                 # NEXT_LLI
    for i, (lo, hi) in enumerate(tbl):
        qt.writel(lli + 0x40 + 8 * i, lo)
        qt.writel(lli + 0x44 + 8 * i, hi)
    qt.writel(lli + 0x100, 0)                                   # NULL
    qt.writel(lli + 0x104, 0x00080000 | 0x3E00)
    qt.writel(cas + 0x1C8, 0x40)                                # next
    qt.writel(VCAP + 0x552C, 0xFFFFFFFE)                        # tick only
    qt.writel(VCAP + 0x540C, 1)
    qt.writel(VCAP + 0x5404, 1 << 31)
    qt.step(40000000)
    check('sync timer ticked', qt.readl(VCAP + 0x55AC) & 1, 1)
    check('cascade LL done', qt.readl(VCAP + 0x55B4) & 1, 1)
    st0, st1 = qt.readl(lli + 0x40), qt.readl(lli + 0x44)
    check('status: ready, frame mode', st0 & 3, 3)
    check('status: DMA written, ids kept', st1 & 0x1F800002,
          (32 << 23) | 2)
    check('next moved to the NULL table', qt.readl(cas + 0x1C8) & 0xFFFF,
          0x100)
    px = qt.read(buf, 4)
    check('navy in UYVY', (px[0], px[1], px[2]), (146, 50, 122))
    qt.writel(VCAP + 0x5404, 0)


def test_3dnr(qt):
    print('gm-3dnr')
    src, dst = SCRATCH + 0x300000, SCRATCH + 0x310000
    qt.write(src, bytes(range(256)) * 32)               # 64x64 x 2 bytes
    cmds = [(0, 0x20000000), (64 | 32 << 16, 0x5E000004),
            (src, 0x5E0001E4), (dst, 0x5E0001F0),
            (0, 0x80000000 | 0x4C0)]
    for i, (lo, hi) in enumerate(cmds):
        qt.writel(DNR + 0x400 + 8 * i, lo)
        qt.writel(DNR + 0x404 + 8 * i, hi)
    qt.writel(DNR + 0x4C4, 0)                           # stop
    qt.writel(DNR + 0x000, 1 << 24)
    qt.step(5000000)
    check('frame copied', qt.read(dst, 64 * 32 * 2), bytes(range(256)) * 16)
    check('status slot done', qt.readl(DNR + 0x404) & 0x20000001, 0x20000001)
    check('frame and list done', qt.readl(DNR + 0x208) & 0x88, 0x88)
    qt.writel(DNR + 0x208, 0xC8)


def test_scaler(qt):
    print('gm-scaler')
    dst = SCRATCH + 0x400000
    blk = SCALER + 0x40000
    cmds = [(0, 0x20000000),
            (0x2, 0x40780000),                          # path 0 scales
            (16 | 8 << 16, 0x40780010),                 # 16x8 out
            (dst, 0x40780180), (0, 0x40780184),
            (32 | 16 << 16 | 1 << 31, 0x40785200),      # fill, 32x16 in
            (0xEB80EB80, 0x40785204),                   # white: Cb Y0 Cr Y1
            (0, 0)]
    for i, (lo, hi) in enumerate(cmds):
        qt.writel(blk + 0x100 + 8 * i, lo)
        qt.writel(blk + 0x104 + 8 * i, hi)
    qt.writel(SCALER + 0x5530, 0xFFFFFFFE)
    qt.writel(SCALER + 0x1A8, 0x100)
    qt.writel(SCALER + 0x5404, 1 << 28)
    check('fire clears itself', qt.readl(SCALER + 0x5404) & (1 << 28), 0)
    qt.step(5000000)
    check('list done', qt.readl(SCALER + 0x55B0) & 1, 1)
    check('status: done, output 0', qt.readl(blk + 0x104), 0x20000011)
    check('16x8 filled', qt.read(dst, 16 * 8 * 2),
          bytes.fromhex('80eb80eb') * 64)


# ---- MJPEG, 2D, audio, small peripherals ----------------------------------

# the standard (Annex K) Huffman tables: BITS, HUFFVAL
JPEG_DHT = (
    (0x0, bytes([0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0]),
     bytes.fromhex(
        '000102030405060708090a0b')),
    (0x10, bytes([0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 125]),
     bytes.fromhex(
        '01020300041105122131410613516107227114328191a1082342b1c1'
        '1552d1f02433627282090a161718191a25262728292a343536373839'
        '3a434445464748494a535455565758595a636465666768696a737475'
        '767778797a838485868788898a92939495969798999aa2a3a4a5a6a7'
        'a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8'
        'd9dae1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9fa')),
    (0x1, bytes([0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0]),
     bytes.fromhex(
        '000102030405060708090a0b')),
    (0x11, bytes([0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 119]),
     bytes.fromhex(
        '000102031104052131061241510761711322328108144291a1b1c109'
        '233352f0156272d10a162434e125f11718191a262728292a35363738'
        '393a434445464748494a535455565758595a636465666768696a7374'
        '75767778797a82838485868788898a92939495969798999aa2a3a4a5'
        'a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6'
        'd7d8d9dae2e3e4e5e6e7e8e9eaf2f3f4f5f6f7f8f9fa')),
)


def jpeg_headers(w, h, qy, qc, restart):
    """What jcmarker.c writes through the bit packer: SOI .. SOS."""
    def seg(code, body):
        return bytes((0xFF, code)) + struct.pack('>H', len(body) + 2) + body
    out = b'\xff\xd8' + seg(0xE0, b'JFIF\x00\x01\x01\x00\x00\x01\x00\x01'
                              b'\x00\x00')
    out += seg(0xDB, b'\x00' + qy) + seg(0xDB, b'\x01' + qc)
    out += seg(0xC0, struct.pack('>BHHB', 8, h, w, 3) +
               bytes((1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1)))
    for tc, bits, vals in JPEG_DHT:
        out += seg(0xC4, bytes((tc,)) + bits + vals)
    out += seg(0xDD, struct.pack('>H', restart))
    out += seg(0xDA, bytes((3, 1, 0x00, 2, 0x11, 3, 0x11, 0, 63, 0)))
    return out


def test_mjpeg(qt, ffmpeg, tmp):
    print('gm-mcp100 (MJPEG)')
    qt.writel(MCP100 + 0x40C, 1 << 27)
    check('reset completes', qt.readl(MCP100 + 0x40C) & (1 << 27), 0)
    qt.writeb(MCP_NODE, 0x55)
    qt.writel(MCP100 + 0x7C, 3 << 18)           # start the embedded CPU
    check('CPU running', qt.readl(MCP100 + 0x7C) & (1 << 23), 1 << 23)
    check('firmware clears the command byte', qt.readb(MCP_NODE), 0)
    # a 48x32 ROI one MCU into a 64x32 packed UYVY picture
    width, mw, mh = 64, 3, 2
    w, h = mw * 16, mh * 16
    ys = [16 + (x * 5 + y * 3) % 200 for y in range(h) for x in range(width)]
    pic = bytearray()
    for y in range(h):
        for x in range(0, width, 2):
            pic += bytes((90 + x, ys[y * width + x], 160 - y,
                          ys[y * width + x + 1]))
    src = SCRATCH + 0x200000
    qt.write(src, bytes(pic))
    for off, v in ((0x00, 1), (0x04, 0), (0x08, mw), (0x0C, mh),
                   (0x18, (src + 16 * 2) >> 2), (0x1C, 0x65), (0x20, 1)):
        qt.writel(DMAWRP + off, v)
    qy = bytes(2 + i // 8 for i in range(64))
    qc = bytes(3 + i // 6 for i in range(64))
    hdr = jpeg_headers(w, h, qy, qc, 2)
    bs = SCRATCH + 0x300000
    qt.writel(MCP100 + 0x78, bs | 1)
    check('VLDSTS ready', qt.readl(MCP100 + 0x74) & (1 << 10), 1 << 10)
    i = 0
    while i < len(hdr):                         # 16 and 8 bits at a time
        if i + 1 < len(hdr) and i % 3 == 0:
            qt.writel(MCP100 + 0x4C, hdr[i] << 8 | hdr[i + 1])
            qt.writel(MCP100 + 0x50, 16)
            i += 2
        else:
            qt.writel(MCP100 + 0x4C, hdr[i])
            qt.writel(MCP100 + 0x50, 8)
            i += 1
    check('headers packed to the output', qt.read(bs, len(hdr)), hdr)
    node = struct.pack('<IIIIIIIIBxxxIIII', 0, bs, width, 2, 3, mw, 0, 0, 4,
                       1, 0, 4, 2)
    qt.write(MCP_NODE, node)
    qt.writel(MCP_NODE + 0x44, 0x10000)        # max_bs_thd
    qt.writel(MCP_NODE + 0xD8, 0)
    qt.writeb(MCP_NODE, 8)                      # ENCODE_JPEG
    qt.step(10000000)
    check('command byte done (0x80)', qt.readb(MCP_NODE), 0x80)
    check('jpgenc_done', qt.readl(MCP_NODE + 0xD8), 1)
    flag = qt.readl(MCP100 + 0x98)
    check('INT_FLAG bit 31', flag & (1 << 31), 1 << 31)
    check('interrupt 31 raised', qt.readl(INTC) & (1 << 31), 1 << 31)
    qt.writel(MCP100 + 0x98, flag)
    check('and cleared by writing INT_FLAG back', qt.readl(INTC) & (1 << 31),
          0)
    n = qt.readl(MCP_NODE + 0xD4)
    jpg = qt.read(bs, n) if 0 < n < 0x10000 else b''
    check('bitslen covers headers, scan and EOI', jpg[-2:], b'\xff\xd9')
    check('restart markers every 2 MCUs', (b'\xff\xd0' in jpg[len(hdr):],
          b'\xff\xd1' in jpg[len(hdr):]), (True, True))
    if not ffmpeg:
        print('  SKIP: no ffmpeg to decode the JPEG')
        return
    path = os.path.join(tmp, 'mjpeg.jpg')
    open(path, 'wb').write(jpg)
    r = subprocess.run(shlex.split(ffmpeg) + [
        '-v', 'error', '-i', path, '-f', 'rawvideo', '-pix_fmt', 'gray', '-'],
        capture_output=True)
    errs = [l for l in r.stderr.decode(errors='replace').splitlines()
            if l.strip() and not l.startswith('docker:')]
    check('decodes without errors', r.returncode == 0 and not errs, True)
    for l in errs[:5]:
        print('    ' + l)
    y = r.stdout
    check('48x32 luma', len(y), w * h)
    if len(y) != w * h:
        return
    worst = 0
    for by in range(h // 8):
        for bx in range(w // 8):
            a = sum(ys[(by * 8 + j) * width + 16 + bx * 8 + i]
                    for j in range(8) for i in range(8))
            b = sum(y[(by * 8 + j) * w + bx * 8 + i]
                    for j in range(8) for i in range(8))
            worst = max(worst, abs(a - b) // 64)
    check('8x8 block means within 3 (worst %d)' % worst, worst <= 3, True)


def gm2d_list(qt, words):
    lst = SCRATCH + 0x3F0000
    qt.write(lst, struct.pack('<%dI' % len(words), *words))
    qt.writel(GM2D + 0xF8, 2)
    qt.writel(GM2D + 0xF0, lst)
    qt.writel(GM2D + 0xF4, len(words))


def test_2d(qt):
    print('gm-2d')
    check('magic', qt.readl(GM2D + 0xEC), 0x84465203)
    dst = SCRATCH + 0x3A0000
    qt.write(dst, bytes(8 * 4 * 2))
    # the privacy mask: a filled rectangle into a 8x4 RGB565 target
    gm2d_list(qt, [0x08, dst, 0x0C, 16, 0x10, 4 << 16 | 8, 0x00, 4,
                   0x04, 0x10002, 0x18, 0, 0x1C, 4 << 16 | 8,
                   0x2C, 0xF8FC0000 | 0xFF, 0x24, 1 << 16 | 2,
                   0x28, 2 << 16 | 5, 0x80000020, 2])
    check('list done: STATUS idle', qt.readl(GM2D + 0xFC), 0)
    check('interrupt 4 raised', qt.readl(INTC) & (1 << 4), 1 << 4)
    qt.writel(GM2D + 0xF8, 2)
    check('and acknowledged', qt.readl(INTC) & (1 << 4), 0)
    px = struct.unpack('<32H', qt.read(dst, 64))
    inside = [px[y * 8 + x] for y in (1, 2) for x in range(2, 6)]
    check('rectangle filled 0xFFE0', tuple(inside), (0xFFE0,) * 8)
    check('outside untouched', sum(px) - sum(inside), 0)
    # the OSD: an RGB565 bitmap, keyed on 0x0020, onto a UYVY frame
    osd = SCRATCH + 0x3B0000
    qt.write(osd, struct.pack('<4H', 0xFFFF, 0x0020, 0xF800, 0x0000))
    frame = SCRATCH + 0x3C0000
    qt.write(frame, bytes((128, 16)) * 32)
    gm2d_list(qt, [0x08, frame, 0x0C, 16, 0x10, 4 << 16 | 8, 0x00, 0xD,
                   0x04, 0x10002, 0x18, 0, 0x1C, 4 << 16 | 8,
                   0x3C, 8, 0x40, 0x0020, 0x34, osd, 0x38, 1 << 16 | 4,
                   0x48, 1 << 16 | 2, 0x80000030, (1 << 27) | 4])
    row = qt.read(frame + 16, 16)
    check('white: Y 235', row[2 * 2 + 1], 235)
    check('keyed pixel left alone', row[3 * 2:3 * 2 + 2], bytes((128, 16)))
    check('red: Y 82, U 90', (row[4 * 2 + 1], row[4 * 2]), (82, 90))
    check('black: Y 16', row[5 * 2 + 1], 16)
    # the LCD path: UYVY grey to RGB565
    lcd, grey = SCRATCH + 0x3D0000, SCRATCH + 0x3D1000
    qt.write(lcd, bytes(16))
    qt.write(grey, bytes((128, 126)) * 4)
    gm2d_list(qt, [0x08, lcd, 0x0C, 8, 0x10, 1 << 16 | 4, 0x00, 4,
                   0x04, 0x10002, 0x18, 0, 0x1C, 1 << 16 | 4,
                   0x3C, 8, 0x34, grey, 0x38, 1 << 16 | 4,
                   0x48, 0, 0x80000030, 0xD])
    check('UYVY grey 126 -> RGB565 0x8410',
          struct.unpack('<4H', qt.read(lcd, 8)), (0x8410,) * 4)
    qt.writel(GM2D + 0xF8, 2)


def test_ssp(qt):
    print('gm-ssp (I2S, ADDA308)')
    check('TX FIFO not full at reset', qt.readl(SSP1 + 0x0C) & 2, 2)
    for _ in range(16):                         # the driver's zero fill
        qt.writel(SSP1 + 0x18, 0)
    check('TX FIFO full after 16', qt.readl(SSP1 + 0x0C) & 2, 0)
    qt.writel(SSP1 + 0x08, 1 << 3)              # clear TX
    qt.writel(SSP1 + 0x00, 0x3120)              # I2S, mono
    qt.writel(SSP1 + 0x10, 4 << 7)
    qt.writel(SSP1 + 0x08, 0x81)                # enable, RX
    qt.step(2000000)                            # 16 samples at 8 kHz
    lvl = (qt.readl(SSP1 + 0x0C) >> 4) & 0x1F
    check('RX FIFO filling at 8 kHz', lvl >= 12, True)
    smp = [qt.readl(SSP1 + 0x18) for _ in range(8)]
    smp = [v - 0x10000 if v & 0x8000 else v for v in smp]
    check('a 1 kHz tone: 8 samples per period, peak 8000',
          (smp[0], max(smp) > 7000, min(smp) < -7000), (0, True, True))
    check('never overruns', qt.readl(SSP1 + 0x14), 0)
    qt.writel(SSP1 + 0x08, 0)
    qt.writel(0x90B00000, 0xF << 4)             # ADDA: 48 kHz capture
    check('ADDA308 holds its registers', qt.readl(0x90B00000), 0xF0)


def aes_run(qt, src, dst, ctl, chunks):
    qt.writel(AES + 0x00, ctl)
    for i, k in enumerate((0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F)):
        qt.writel(AES + 0x10 + 4 * i, k)
    for i in range(4):
        qt.writel(AES + 0x30 + 4 * i, 0)
    qt.writel(AES + 0x5C, 1)
    qt.writel(AES + 0x48, src)
    qt.writel(AES + 0x4C, dst)
    for n in chunks:
        qt.writel(AES + 0x50, n)
        qt.writel(AES + 0x54, 1)
        qt.writel(AES + 0x68, 1)


def test_aes(qt):
    print('fh-aes (FTAES020)')
    pt = bytes.fromhex('00112233445566778899aabbccddeeff')
    ct = bytes.fromhex('69c4e0d86a7b0430d8cdb78070b4c55a')     # FIPS-197
    src, dst = SCRATCH + 0x3E0000, SCRATCH + 0x3E1000
    qt.write(src, pt * 4)
    qt.writel(AES + 0x5C, 1)
    aes_run(qt, src, dst, 8, (64,))             # AES-128 ECB
    check('AES-128 ECB: FIPS-197 vector', qt.read(dst, 16), ct)
    aes_run(qt, src, dst, 8, (16, 16, 32))
    check('chunks continue at the advanced addresses', qt.read(dst, 64),
          ct * 4)
    aes_run(qt, src, dst, 0x80 | 0x10 | 8, (64,))
    whole = qt.read(dst, 64)
    aes_run(qt, src, dst, 0x80 | 0x10 | 8, (16, 48))
    check('CBC in chunks chains like one transfer', qt.read(dst, 64), whole)
    check('CBC first block = ECB of the plaintext', whole[:16], ct)
    check('done acknowledged: interrupt 19 low', qt.readl(INTC) & (1 << 19),
          0)


def test_small(qt):
    print('gm-saradc, gm-pwm, gm-irdet, gm-pmonitor')
    check('SAR ADC: XGAIN2 reads "no CVBS cable"', qt.readl(SARADC + 0x10),
          0x80)
    qt.writel(PWM + 0x14, 999)
    qt.writel(PWM + 0x10, (1 << 8) | (1 << 1))
    check('PWM: update loads CNTB, bit clears',
          (qt.readl(PWM + 0x10) & 2, qt.readl(PWM + 0x1C)), (0, 999))
    check('PWM: INT_PRIO 0 (no interrupt)', qt.readl(PWM + 0x160), 0)
    qt.writel(IRDET + 0x00, 5)
    check('IR: idle status', qt.readl(IRDET + 0x10), 0)
    qt.writel(PMON + 0x04, 1000)
    qt.writel(PMON + 0x00, 7)
    qt.step(2000000)
    check('pMonitor: window ends, interrupt 44', qt.readl(INTC + 0x20) &
          (1 << 12), 1 << 12)
    qt.writel(PMON + 0x00, 2)
    check('and clears', qt.readl(INTC + 0x20) & (1 << 12), 0)


def test_rtc(qemu, machine):
    print('gm-rtc')
    qt = QTest(qemu, machine, ('-rtc', 'base=2020-01-01T00:00:00,clock=vm'))
    try:
        check('the time from -rtc base: day 18262',
              (qt.readl(RTC + 0x0C), qt.readl(RTC + 0x08)), (18262, 0))
        check('never busy', qt.readl(RTC + 0x40) & (1 << 8), 0)
        for off, v in ((0x24, 4), (0x28, 3), (0x2C, 2), (0x30, 1)):
            qt.writel(RTC + off, v)
        qt.writel(RTC + 0x20, 0x41)             # load, enable
        check('loaded 1d 2h 3m 4s', tuple(qt.readl(RTC + o)
              for o in (0x0C, 0x08, 0x04, 0x00)), (1, 2, 3, 4))
        qt.writel(RTC + 0x20, 0x83)             # what the driver sets
        qt.step(1000000000)
        check('a second later', qt.readl(RTC + 0x00), 5)
        check('per-second interrupt 50', qt.readl(INTC + 0x20) & (1 << 18),
              1 << 18)
        qt.writel(RTC + 0x34, 1)
        check('cleared through INTR_STATE', qt.readl(INTC + 0x20) &
              (1 << 18), 0)
    finally:
        qt.close()


def test_machines(qemu):
    print('machines')
    for m, cid in (('gm8135s', 0x08136110), ('gm8136', 0x08136100),
                   ('gm8136s', 0x08136150)):
        qt = QTest(qemu, m)
        check('%s chip ID' % m, qt.readl(PMU), cid)
        qt.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default='./qemu-src/build/qemu-system-arm')
    ap.add_argument('--ffmpeg', default=shutil.which('ffmpeg'),
                    help='ffmpeg command to decode the encoder\'s streams '
                         'with (e.g. "docker run --rm -i -v /tmp:/tmp '
                         'jrottenberg/ffmpeg:6.1-alpine")')
    args = ap.parse_args()

    tmp = tempfile.mkdtemp()
    try:
        path = os.path.join(tmp, 'nor.bin')
        img = make_image(path)
        qt = QTest(args.qemu, 'gm8136,sensor=ov2715,flash-file=' + path)
        try:
            test_boot_rom(qt)
            test_pmu_intc(qt)
            test_ddrc(qt)
            test_timer(qt)
            test_spi(qt, img)
            test_dmac(qt)
            test_gpio(qt)
            test_i2c(qt)
            test_wdt(qt)
            test_usb_sd(qt)
            test_h264e(qt, args.ffmpeg, tmp)
            test_vcap(qt)
            test_3dnr(qt)
            test_scaler(qt)
            test_mjpeg(qt, args.ffmpeg, tmp)
            test_2d(qt)
            test_ssp(qt)
            test_aes(qt)
            test_small(qt)
        finally:
            qt.close()
        test_rtc(args.qemu, 'gm8136,flash-file=' + path)
        test_machines(args.qemu)
        print('migration')
        state = os.path.join(tmp, 'state')
        r = subprocess.run(
            [args.qemu, '-M', 'gm8136,sensor=ov2715,flash-file=' + path,
             '-display', 'none', '-nodefaults', '-S', '-monitor', 'stdio'],
            input='migrate "exec:cat > %s"\nquit\n' % state,
            capture_output=True, text=True, timeout=60)
        check('state saved', os.path.getsize(state) > 0, True)
        r = subprocess.run(
            [args.qemu, '-M', 'gm8136,sensor=ov2715,flash-file=' + path,
             '-display', 'none', '-nodefaults', '-S', '-monitor', 'stdio',
             '-incoming', 'exec:cat %s' % state],
            input='info status\nquit\n', capture_output=True, text=True,
            timeout=60)
        check('and loaded back', 'paused' in r.stdout and
              'error' not in (r.stdout + r.stderr).lower(), True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('%d failure(s)' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
