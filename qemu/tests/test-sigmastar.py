#!/usr/bin/env python3
"""Register-level checks of the SigmaStar infinity2m models, no guest needed.

QEMU runs the ssr621q machine under qtest and this script plays the vendor
IPL, U-Boot and kernel drivers (docs/sigmastar.md):

  - the mask ROM: the IPL is staged in IMI and the ROM stub at 0 ends in a
    jump to it;
  - sstar-riu: storage with byte lanes, the MIU / l3bridge / UTMI status
    bits, the bond strap and chip ID;
  - sstar-timer: timer 0 counts the 12 MHz crystal, its high half latched
    by a low read; timer 1 expiring into the main-intc FIQ status;
  - sstar-uart: a byte reaches the backend, USR reads idle;
  - sstar-fsp: RDID and READ through the FSP, WREN+SE+RDSR as three
    commands, a page programmed from DRAM through BDMA and the FSP's outside
    buffer, all written back to the image and seen in the XIU window;
  - sstar-bdma: SPI to DRAM;
  - sstar-emac: MDIO through the RIU view's 16-bit halves, the MAC address
    in both views, a frame sent with TAR/TCR captured on the netdev;
  - sstar-uhc: EHCI capability registers through the halves, a USB
    keyboard on the root port;
  - SATA: the AHCI capability registers, a disk on port 0;
  - mstar-ge reads idle; mstar-disp latches vsync and clears it on W1C;
  - outside qtest: machine state migrates to a file and back, and a
    file-backed flash blocks migration.

and then the msc313e (infinity3) machine: the ROM stages the IPL from NOR
0x4000, the chip ID, the IPL's "[HSD] IP Reset Switch" acknowledge,
sstar-rtc (latch, load, count, alarm), the VIP gamma SRAM ports, the JPEG
encoder (Q table port, the stale start the reset cancels, a scan into DRAM
ending in EOI with its size and interrupt), the H.265 encoder (a frame
from its trigger to swreg1 = IRQ | FRAME_RDY, the size in swreg9, the
interrupt and its mask, IDR then TRAIL_R NALs in DRAM), sstar-i3-gpio (pad
levels in and out, the SAR ADC, PM-sleep edges cascaded into MS_IRQ 2 with
mask and ack, daynight= through qom-set), sstar-i3-bach (capture and
playback levels, TRIG, the period and XRUN interrupts on MS_IRQ 42 rising
and falling as the ISR masks them) and the ISP pipeline (a JX-F22 exposure
set over MIIC1 into AE/AWB block statistics that scale with it, WB gains
that neutralise the logo's white in the scaler's NV12 output, a gamma table
through the menuload, the test pattern with the ISP off).

Then the infinity6 dies (ssc325, ssc337, ssc338q, ssc377): chip IDs and boot
straps, the MIU BIST's pattern in DRAM, the I6E MMU, the I6E FSP following
the bond, and card detect as an input pad; SPI NAND in place of the NOR.

Ifliegen (ssc034a, ssc034b): the bond defaults, the GIC-400 and its 224
interrupts, a timer on a level MS_IRQ line, the URDMA TX FIFO, the AES and
RSA reset-done bits, CPU1 started Secure by TF-A's smpctrl wake; both SD
slots with their card-detect pads and a write by auto-trigger, and the
encoder firmware's start, command handshake and an encode command
through its ring to a result block and an H.265 stream.

    python3 qemu/tests/test-sigmastar.py [--qemu PATH]
"""
import re
import argparse
import base64
import lzma
import atexit
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import zlib

RIU = 0x1F000000
IMI = 0xA0000000
DDR = 0x20000000
DRAM = 0x20000000
XIU = 0x14000000


def riu(bank, reg):
    return RIU + bank * 0x200 + reg * 4


ISP = riu(0x0008, 0)
FSP = riu(0x0016, 0)
TIMER = riu(0x0030, 0)
BDMA = riu(0x1002, 0)
INTC = riu(0x1009, 0)
UART0 = 0x1F221000
EMAC = 0x1F2A2000
EMAC_X32 = 0x1F343C00
UHC0 = riu(0x1424, 0)
SATA = 0x1F345000
VPU = 0x1F344800
GE = riu(0x1409, 0)
TOP = riu(0x1128, 0)

IPL_SIZE = 0x200
JEDEC = 0xC22018


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
        self.irqs = []

    def cmd(self, line):
        self.f.write(line + '\n')
        self.f.flush()
        while True:
            r = self.f.readline().strip()
            if r.startswith('IRQ'):
                self.irqs.append(r)
            elif r.startswith(('OK', 'FAIL', 'ERR')):
                return r

    def writel(self, a, v):
        self.cmd('writel 0x%x 0x%x' % (a, v))

    def writew(self, a, v):
        self.cmd('writew 0x%x 0x%x' % (a, v))

    def writeb(self, a, v):
        self.cmd('writeb 0x%x 0x%x' % (a, v))

    def readl(self, a):
        return int(self.cmd('readl 0x%x' % a).split()[1], 16)

    def readw(self, a):
        return int(self.cmd('readw 0x%x' % a).split()[1], 16)

    def readb(self, a):
        return int(self.cmd('readb 0x%x' % a).split()[1], 16)

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
    if isinstance(v, bytes):
        return v[:8].hex() + ('...' if len(v) > 8 else '')
    if isinstance(v, (list, tuple)):
        return ' '.join(str(shown(x)) for x in v)
    return hex(v)


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what, shown(got)))


def make_image(path):
    """16 MiB: an IPL header and body at 0, a pattern from 0x10000 on."""
    img = bytearray(b'\xFF' * (16 << 20))
    struct.pack_into('<IIHHI', img, 0, 0xEA000002, 0x5F4C5049, IPL_SIZE,
                     0, 0x12345678)
    for i in range(0x10, IPL_SIZE):
        img[i] = i & 0xFF
    for i in range(0x10000, 0x20000):
        img[i] = i * 7 & 0xFF
    open(path, 'wb').write(img)


def test_boot_rom(qt):
    print('mask ROM')
    check('ROM stub: sp, then the table runner',
          [qt.readl(4 * i) for i in range(4)],
          [0xE59FD014, 0xE28F2014, 0xE8B20003, 0xE3500000])
    check('stack in IMI', qt.readl(0x1C), 0xA000D000)
    check('first ROM write: watchdog MAX_PRD_L', qt.readl(0x20), 0x1F006010)
    check('IPL magic in IMI', qt.readl(IMI + 4), 0x5F4C5049)
    check('IPL body in IMI', qt.readl(IMI + 0x1FC), 0xFFFEFDFC)
    check('nothing past the IPL size', qt.readl(IMI + IPL_SIZE), 0)


def test_riu(qt):
    print('sstar-riu')
    a = riu(0x1033, 0x10)
    qt.writew(a, 0x1234)
    qt.writeb(a + 1, 0xAB)
    check('16-bit storage, byte write to the high lane', qt.readw(a), 0xAB34)
    check('bytes 2-3 of a slot are not there', qt.readw(a + 2), 0)
    check('MIU init done', qt.readw(riu(0x1012, 0x00)) & 0x8000, 0x8000)
    check('MIU BIST done', qt.readw(riu(0x1012, 0x70)) & 0x8000, 0x8000)
    check('l3bridge flush done', qt.readw(riu(0x1022, 0x10)) & 0x1000,
          0x1000)
    check('bond strap', qt.readw(riu(0x101E, 0x48)), 0x1E)
    check('chip ID', qt.readw(riu(0x001E, 0x00)), 0xF0)
    check('UTMI calibration done', qt.readw(riu(0x1425, 0x1E)) & 2, 2)
    check('clkgen reset value', qt.readw(riu(0x1038, 0x33)), 0x1101)


def test_timer(qt):
    print('sstar-timer + sstar-intc')
    t0 = TIMER + 0x40
    qt.writew(t0 + 0x00, 0)
    qt.writew(t0 + 0x08, 0xFFFF)
    qt.writew(t0 + 0x0C, 0xFFFF)
    qt.writew(t0 + 0x00, 1)
    qt.step(10000000)                               # 10 ms
    lo = qt.readw(t0 + 0x10)
    hi = qt.readw(t0 + 0x14)
    check('12 MHz: 120000 ticks in 10 ms', (hi << 16) | lo, 120000)
    # Timer 1, interrupt enabled, wraps after 12000 ticks (1 ms): MS_FIQ 1.
    t1 = TIMER + 0x80
    qt.writew(INTC + 0x4C * 4, 0xFFFF)              # clear FIQ status
    qt.writew(t1 + 0x08, 11999)
    qt.writew(t1 + 0x0C, 0)
    qt.writew(t1 + 0x00, 0x101)
    check('not expired yet', qt.readw(INTC + 0x4C * 4) & 2, 0)
    qt.step(1500000)
    check('timer 1 latched in the FIQ status', qt.readw(INTC + 0x4C * 4) & 2,
          2)
    qt.writew(INTC + 0x4C * 4, 2)
    check('write 1 clears it', qt.readw(INTC + 0x4C * 4) & 2, 0)
    check('IRQ lines come out masked', qt.readw(INTC + 0x54 * 4), 0xFFFF)


def test_uart(qt, out):
    print('sstar-uart')
    check('USR: TX FIFO empty, not full', qt.readb(UART0 + 0x38), 0x06)
    qt.writeb(UART0, ord('S'))
    time.sleep(0.2)
    check('byte reached the backend', open(out, 'rb').read(), b'S')


def fsp_run(qt, wbytes, wsizes, rsizes, out=None):
    for i in range(0, 10, 2):
        lo = wbytes[i] if i < len(wbytes) else 0
        hi = wbytes[i + 1] if i + 1 < len(wbytes) else 0
        qt.writew(FSP + (0x60 + i // 2) * 4, lo | (hi << 8))
    qt.writew(FSP + 0x6A * 4, wsizes[0] | (wsizes[1] << 4) | (wsizes[2] << 8))
    qt.writew(FSP + 0x6B * 4, rsizes[0] | (rsizes[1] << 4) | (rsizes[2] << 8))
    ctrl = 0x7 | (0x8000 if wsizes[1] else 0) | (0x4000 if wsizes[2] else 0)
    qt.writew(FSP + 0x6C * 4, ctrl)
    if out:
        qt.writew(FSP + 0x78 * 4, out[0])
        qt.writew(FSP + 0x79 * 4, 0x1000 | (1 << 8) | out[1])
    qt.writew(FSP + 0x6D * 4, 1)
    done = qt.readw(FSP + 0x6E * 4) & 1
    qt.writew(FSP + 0x6F * 4, 1)
    qt.writew(FSP + 0x78 * 4, 0)
    qt.writew(FSP + 0x79 * 4, 0)
    rbf = []
    for i in range(10):
        w = qt.readw(FSP + (0x65 + i // 2) * 4)
        rbf.append(w >> 8 if i & 1 else w & 0xFF)
    return done, rbf


def bdma_run(qt, ch, path, src, dst, size, misc=0):
    b = BDMA + ch * 0x80
    qt.writew(b + 0x02 * 4, path)
    qt.writew(b + 0x03 * 4, misc)
    qt.writew(b + 0x04 * 4, src & 0xFFFF)
    qt.writew(b + 0x05 * 4, src >> 16)
    qt.writew(b + 0x06 * 4, dst & 0xFFFF)
    qt.writew(b + 0x07 * 4, dst >> 16)
    qt.writew(b + 0x08 * 4, size & 0xFFFF)
    qt.writew(b + 0x09 * 4, size >> 16)
    qt.writew(b + 0x00 * 4, 1)
    st = qt.readw(b + 0x01 * 4)
    qt.writew(b + 0x01 * 4, 8)
    return st


def test_flash(qt, path):
    print('sstar-fsp + sstar-bdma')
    done, rbf = fsp_run(qt, [0x9F], (1, 0, 0), (3, 0, 0))
    check('FSP done', done, 1)
    check('RDID', (rbf[0] << 16) | (rbf[1] << 8) | rbf[2], JEDEC)
    done, rbf = fsp_run(qt, [0x03, 0x01, 0x00, 0x10], (4, 0, 0), (4, 0, 0))
    check('READ 0x10010', rbf[:4], [0x10010 * 7 + i * 7 & 0xFF
                                     for i in range(4)])
    check('XIU window', qt.readl(XIU + 0x10010),
          int.from_bytes(bytes(rbf[:4]), 'little'))
    # BDMA SPI -> DRAM (bus address 0x100000)
    st = bdma_run(qt, 0, 0x0005, 0x10000, 0x100000, 0x40)
    check('BDMA done', st & 8, 8)
    check('SPI -> DRAM copied', qt.readl(DRAM + 0x100000 + 4),
          int.from_bytes(bytes(0x10004 * 7 + i * 7 & 0xFF for i in range(4)),
                         'little'))
    # Sector erase at 0x11000: WREN; SE a2 a1 a0; RDSR
    done, rbf = fsp_run(qt, [0x06, 0x20, 0x01, 0x10, 0x00, 0x05],
                        (1, 4, 1), (0, 0, 1))
    check('erase: RDSR after it, WIP and WEL clear', rbf[0] & 3, 0)
    check('erased in the XIU window', qt.readl(XIU + 0x11000), 0xFFFFFFFF)
    # Page program from DRAM: BDMA MIU -> FSP outside buffer, then
    # WREN; PP a2 a1 a0 <outside>; RDSR (U-Boot's HAL_FSP_Write_BDMA).
    for i in range(4):
        qt.writel(DRAM + 0x200000 + 4 * i, 0xC0DE0000 + i)
    st = bdma_run(qt, 0, 0x2B40, 0x200000, 0, 16)
    check('BDMA MIU -> FSP done', st & 8, 8)
    fsp_run(qt, [0x06, 0x02, 0x01, 0x10, 0x00, 0x00, 0x05],
            (1, 5, 1), (0, 0, 1), out=(16, 5))
    check('programmed, XIU window', qt.readl(XIU + 0x11004), 0xC0DE0001)
    data = open(path, 'rb').read()
    check('erase written back', data[0x11010:0x11014], b'\xFF' * 4)
    check('program written back', data[0x11000:0x11004],
          struct.pack('<I', 0xC0DE0000))


def emac_riu_write(qt, g, v):
    qt.writew(EMAC + 2 * g, v & 0xFFFF)
    qt.writew(EMAC + 2 * g + 4, v >> 16)


def test_emac(qt, pcap):
    print('sstar-emac')
    qt.writew(EMAC + 0x00, 0x10)                    # CTL: MDIO enable
    emac_riu_write(qt, 0x34, 0x60020000 | (1 << 18))    # read PHY 0 reg 1
    check('MDIO idle (SR bit 2)', qt.readw(EMAC + 2 * 0x08) & 4, 4)
    check('BMSR: link up, autoneg done', qt.readw(EMAC + 2 * 0x34), 0x786D)
    check('same PHY at 0x1F006200', qt.readw(0x1F006204), 0x786D)
    emac_riu_write(qt, 0x34, 0x60020000 | (1 << 23) | (1 << 18))
    check('no PHY at address 1', qt.readw(EMAC + 2 * 0x34), 0xFFFF)
    emac_riu_write(qt, 0x98, 0x3F0B0000)
    check('SA1 written in the RIU view, read in X32',
          qt.readl(EMAC_X32 + 2 * 0x98), 0x3F0B0000)
    check('TSR: TX free', qt.readl(EMAC_X32 + 2 * 0x14), 0x1F98)
    frame = bytes.fromhex('ffffffffffff000b3f000001' '0806') + bytes(46)
    for i in range(0, len(frame), 4):
        qt.writel(DRAM + 0x300000 + i,
                  int.from_bytes(frame[i:i + 4], 'little'))
    qt.writel(EMAC_X32 + 2 * 0x00, 0x0C)            # RE | TE
    qt.writel(EMAC_X32 + 2 * 0x0C, 0x300000)        # TAR, bus address
    qt.writel(EMAC_X32 + 2 * 0x10, len(frame))      # TCR sends
    time.sleep(0.3)
    data = open(pcap, 'rb').read()
    check('frame on the wire', data[-len(frame):], frame)


def test_uhc(qt):
    print('sstar-uhc')
    check('CAPLENGTH', qt.readw(UHC0) & 0xFF, 0x10)
    check('HCIVERSION (high half)', qt.readw(UHC0 + 4), 0x0100)
    check('HCSPARAMS: one port', qt.readw(UHC0 + 8) & 0xF, 1)
    portsc = qt.readw(UHC0 + 2 * 0x30)
    check('PORTSC: the keyboard is connected', portsc & 1, 1)
    check('bmcs: high speed', (qt.readw(UHC0 + 2 * 0x40) >> 9) & 3, 2)
    # USBCMD run, through the halves; USBSTS no longer halted.
    qt.writew(UHC0 + 2 * 0x10, 1)
    qt.writew(UHC0 + 2 * 0x10 + 4, 0x0008)
    check('USBCMD reads back', qt.readw(UHC0 + 2 * 0x10) & 1, 1)
    check('USBSTS: running', qt.readw(UHC0 + 2 * 0x14) & 0x1000, 0)
    # A lone low half (U-Boot's stack writes them) lands at once.
    qt.writew(UHC0 + 2 * 0x10, 0)
    check('lone low-half write stops it', qt.readw(UHC0 + 2 * 0x14) & 0x1000,
          0x1000)


def test_sata(qt):
    print('SATA (sysbus-ahci)')
    cap = qt.readl(SATA)
    check('CAP: one port', cap & 0x1F, 0)
    check('PI: port 0', qt.readl(SATA + 0x0C), 1)
    check('port 0 SStatus: the disk is there, PHY up',
          qt.readl(SATA + 0x100 + 0x28) & 0xF, 3)


def test_display(qt):
    print('mstar-ge + mstar-disp')
    check('GE idle, command queue free', qt.readw(GE + 0x1C), 0xF880)
    qt.writew(TOP + 0x0C, 0)                        # unmask vsync
    qt.step(20000000)
    check('vsync latched', qt.readw(TOP + 0x08) & 8, 8)
    qt.writew(TOP + 0x08, 8)
    check('W1C clears it', qt.readw(TOP + 0x08) & 8, 0)


def vpu_cmd(qt, cmd, inst=None, opt=None):
    """Issue a WAVE5 command; return RET_SUCCESS."""
    if inst is not None:
        qt.writel(VPU + 0x110, inst)
    if opt is not None:
        qt.writel(VPU + 0x104, opt)
    qt.writel(VPU + 0x70, 1)                        # BUSY_STATUS
    qt.writel(VPU + 0x100, cmd)
    qt.writel(VPU + 0x38, 1)                        # HOST_INT_REQ
    check('command 0x%x: not busy after' % cmd, qt.readl(VPU + 0x70), 0)
    return qt.readl(VPU + 0x108)


def test_vpu(qt):
    print('sstar-vpu')
    qt.writel(VPU + 0x20, 0xFE14)                   # FIO read, bus status
    check('FIO ready', qt.readl(VPU + 0x20) >> 31, 1)
    check('backbone bus idle', qt.readl(VPU + 0x24), 0x3F)
    check('firmware not running', qt.readl(VPU + 0x04), 0)
    qt.writel(VPU + 0x100, 1)                       # INIT_VPU
    qt.writel(VPU + 0x6C, 1)                        # REMAP_CORE_START
    check('INIT_VPU done', (qt.readl(VPU + 0x70), qt.readl(VPU + 0x108)),
          (0, 1))
    check('firmware running', qt.readl(VPU + 0x04) != 0, True)
    check('GET_VPU_INFO', vpu_cmd(qt, 0x4000, opt=0), 1)
    check('decoders: AVC and HEVC', qt.readl(VPU + 0x128), 0x0C)
    check('product name', qt.readl(VPU + 0x11C), 0x57353131)
    qt.writel(VPU + 0x48, 0x8940)                   # VINT_ENABLE, as mi_vdec
    # a ring QEMU would have to copy gigabytes out of: refused
    qt.writel(VPU + 0x11C, 0)
    qt.writel(VPU + 0x120, 0xFFFFFFFF)
    check('CREATE_INSTANCE: 4 GiB ring refused',
          vpu_cmd(qt, 0x08, inst=0x20000), 0)
    BS, BS_SIZE = 0x400000, 0x100000
    qt.writel(VPU + 0x11C, BS)
    qt.writel(VPU + 0x120, BS_SIZE)
    check('CREATE_INSTANCE, H.264', vpu_cmd(qt, 0x08, inst=0x20000), 1)
    # INIT_SEQ on an empty ring: queued, done from the timer
    qt.writel(VPU + 0x118, BS)
    qt.writel(VPU + 0x11C, BS)
    qt.writel(VPU + 0x120, 0x80000001)
    check('INIT_SEQ queued', vpu_cmd(qt, 0x40, inst=0x20000, opt=1), 1)
    check('instance queue: 1', (qt.readl(VPU + 0x1F4) >> 16) & 0xFF, 1)
    check('no interrupt yet', qt.readl(VPU + 0x44), 0)
    qt.step(10000000)
    check('done: INIT_SEQ reason, instance 0 in QUEUE_CMD_DONE_INST',
          (qt.readl(VPU + 0x44), qt.readl(VPU + 0x4C), qt.readl(VPU + 0x1E8)),
          (1, 0x40, 1))
    qt.writel(VPU + 0x1E8, 0)
    qt.writel(VPU + 0x34, 0x40)                     # VINT_REASON_CLR
    qt.writel(VPU + 0x3C, 1)                        # VINT_CLEAR
    check('interrupt cleared', qt.readl(VPU + 0x44), 0)
    check('GET_RESULT', vpu_cmd(qt, 0x4000, inst=0x20000, opt=2), 1)
    check('empty ring: no sequence', qt.readl(VPU + 0x1DC), 0)
    check('a second GET_RESULT: not ready',
          (vpu_cmd(qt, 0x4000, inst=0x20000, opt=2), qt.readl(VPU + 0x10C)),
          (0, 0x800))
    vpu_decode(qt, BS)


def vpu_decode(qt, bs):
    """Decode the fake camera's first picture, if libavcodec is built in."""
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(
        __file__)), '../../qemu-boot'))
    import importlib
    sys.dont_write_bytecode = True
    cam = importlib.import_module('fake-ipcam')
    pat = cam.H264Pattern(64, 48)
    idr, p1 = pat.picture()[0], pat.picture()[0]
    stream = idr + p1
    stream += bytes(-len(stream) % 4)
    for i in range(0, len(stream), 4):
        qt.writel(DRAM + bs + i, int.from_bytes(stream[i:i + 4], 'little'))
    qt.writel(VPU + 0x118, bs)
    qt.writel(VPU + 0x11C, bs + len(idr))
    qt.writel(VPU + 0x120, 0x80000001)
    vpu_cmd(qt, 0x40, inst=0x20000, opt=1)
    qt.step(10000000)
    qt.writel(VPU + 0x1E8, 0)
    vpu_cmd(qt, 0x4000, inst=0x20000, opt=2)
    if not qt.readl(VPU + 0x1DC):
        print('  decoding skipped: built without libavcodec')
        return
    check('sequence: 64x48', qt.readl(VPU + 0x14C), (64 << 16) | 48)
    # SET_FB, the compressed group: three buffers (only counted)
    qt.writel(VPU + 0x118, 0)
    qt.writel(VPU + 0x120, 2)
    check('SET_FB', vpu_cmd(qt, 0x80, inst=0x20000, opt=0x18), 1)
    # Two DEC_PICs: the host's chunks need not be pictures.  The first
    # holds half the IDR, the second its rest and the next picture's start.
    OUT, VLC = 0x600000, 0x5F0000
    half = len(idr) // 2

    def dec_pic(wr):
        qt.writel(VPU + 0x11C, bs + wr)
        qt.writel(VPU + 0x120, 0x80000001)
        qt.writel(VPU + 0x150, 0x8221)
        qt.writel(VPU + 0x154, (32 << 16) | 24)
        qt.writel(VPU + 0x158, OUT)
        qt.writel(VPU + 0x15C, OUT + 32 * 24)
        qt.writel(VPU + 0x164, 32)
        qt.writel(VPU + 0x168, VLC)
        check('DEC_PIC queued', vpu_cmd(qt, 0x100, inst=0x20000, opt=0), 1)
        qt.step(10000000)
        qt.writel(VPU + 0x1E8, 0)
        vpu_cmd(qt, 0x4000, inst=0x20000, opt=2)

    dec_pic(half)
    check('half a picture: nothing decoded, black shown',
          (qt.readl(VPU + 0x170), qt.readl(VPU + 0x174),
           qt.readb(DRAM + OUT)), (0xFFFFFFFE, 0, 16))
    check('done: instance index in 0x1FC', qt.readl(VPU + 0x1FC), 0)
    dec_pic(len(stream))
    check('the IDR, once the next picture starts: decoded, displayed',
          (qt.readl(VPU + 0x170), qt.readl(VPU + 0x174)), (0, 0))
    check('picture: IDR', qt.readl(VPU + 0x160), (5 << 4) | 1)
    check('output buffer and VLC buffer given back',
          (qt.readl(VPU + 0x188), qt.readl(VPU + 0x1A0)), (OUT, VLC))
    # row 5, left of the counter: the first colour bar, Y 235, Cb/Cr 128
    check('NV12 written: luma', qt.readb(DRAM + OUT + 5 * 32), 235)
    check('NV12 written: chroma', qt.readw(DRAM + OUT + 32 * 24 + 2 * 32),
          0x8080)


def make_i3_image(path):
    """8 MiB, blank but for an IPL at 0x4000, where the I3 ROM looks."""
    img = bytearray(b'\xFF' * (8 << 20))
    struct.pack_into('<IIHHI', img, 0x4000, 0xEA000002, 0x5F4C5049,
                     IPL_SIZE, 0, 0x12345678)
    for i in range(0x10, IPL_SIZE):
        img[0x4000 + i] = i & 0xFF
    open(path, 'wb').write(img)


def test_msc313e(qemu, tmp):
    print('msc313e (infinity3)')
    path = os.path.join(tmp, 'i3.bin')
    make_i3_image(path)
    qt = QTest(qemu, 'msc313e,flash-file=' + path, ['-rtc', 'clock=vm'])
    try:
        check('IPL from NOR 0x4000 in IMI', qt.readl(IMI + 4), 0x5F4C5049)
        check('its body', qt.readl(IMI + 0x1FC), 0xFFFEFDFC)
        check('chip ID', qt.readw(riu(0x001E, 0x00)), 0xC2)
        for bank in (0x1410, 0x1402):
            a = riu(bank, 0x3F)
            qt.writew(a, 0)
            check('bank 0x%x: IP reset switch low acknowledged' % bank,
                  qt.readw(a) & 0xE, 0xE)
            qt.writew(a, 1)
            check('bank 0x%x: and high' % bank, qt.readw(a) & 0xE, 0)

        rtc = riu(0x0012, 0)
        qt.writew(rtc + 0x0C, 0x5678)               # LOAD_VAL
        qt.writew(rtc + 0x10, 0x1234)
        qt.writew(rtc, 0x0B | 0x20)                 # RSTZ CNT_EN LOAD_EN
        check('RTC: LOAD_EN clears itself', qt.readw(rtc) & 0x08, 0)
        qt.writew(rtc, 0x13 | 0x20)                 # READ_EN
        check('RTC: READ_EN clears itself', qt.readw(rtc) & 0x10, 0)
        check('RTC: the loaded count latched',
              qt.readw(rtc + 0x20) | qt.readw(rtc + 0x24) << 16, 0x12345678)
        qt.step(3 * 10 ** 9)
        qt.writew(rtc, 0x13 | 0x20)
        check('RTC: three seconds later',
              qt.readw(rtc + 0x20) | qt.readw(rtc + 0x24) << 16, 0x1234567B)
        qt.writew(rtc + 0x14, 0x567D)               # MATCH_VAL = now + 2
        qt.writew(rtc + 0x18, 0x1234)
        qt.writew(riu(0x1009, 0x54 + 2), 0)         # unmask MS_IRQ 32-47
        qt.writew(rtc, 0x03)                        # INT_MASK clear
        qt.step(3 * 10 ** 9)
        check('RTC: the alarm reaches GIC SPI 76 (intc IRQ 44)',
              qt.readw(riu(0x1009, 0x5C + 2)) >> 12 & 1, 1)
        qt.writew(rtc, 0x83)                        # INT_CLEAR
        check('RTC: INT_CLEAR drops it',
              qt.readw(riu(0x1009, 0x5C + 2)) >> 12 & 1, 0)

        g = riu(0x1226, 0x40)                       # YUV gamma port
        qt.writew(g + 0x10, 0x0505)                 # V address 5
        qt.writew(g + 0x24, 0x0ABC)                 # V even, odd data
        qt.writew(g + 0x28, 0x0DEF)
        qt.writew(g + 0x04, 0x30)                   # V write enables
        qt.writew(g, (4 << 3) | 4)                  # read V even
        check('YUV gamma SRAM: V even entry reads back',
              qt.readw(g + 0x3C), 0xABC)
        qt.writew(g, (5 << 3) | 4)
        check('and V odd', qt.readw(g + 0x3C), 0xDEF)
        qt.writew(g, (0 << 3) | 4)
        check('Y untouched', qt.readw(g + 0x3C), 0)

        r = riu(0x1211, 0x7D)                       # GM12to10 port
        qt.writew(r + 4, 0x0123)
        qt.writew(r, 0x07 | 3 << 8 | 1 << 11)       # entry 7 of table 3
        qt.writew(r, 0x07 | 3 << 8 | 1 << 12)
        check('RGB gamma SRAM: entry reads back', qt.readw(r + 8), 0x123)
        qt.writew(riu(0x1211, 0x7A), 0x07 | 3 << 8 | 1 << 12)
        check('the other port has its own SRAM', qt.readw(riu(0x1211, 0x7C)),
              0)
        test_i3_gpio(qt)
        test_i3_pmsleep(qt)
        test_i3_bach_capture(qt)
        test_i3_bach_playback(qt)
        test_i3_jpe(qt)
        test_i3_vhe(qt)
        test_i3_mfe(qt)
        test_i3_isp(qt)
    finally:
        qt.close()
    out = hmp(qemu, ['-M', 'msc313e', '-S'],
              ['xp /1hx 0x%x' % riu(0x0014, 0x42),
               'qom-set /machine daynight night',
               'xp /1hx 0x%x' % riu(0x0014, 0x42),
               'xp /1hx 0x%x' % riu(0x000F, 0x14)])
    vals = [int(l.split(':')[1], 16) for l in out.splitlines()
            if l.startswith('00000000%08x' % riu(0x0014, 0x42)) or
            l.startswith('00000000%08x' % riu(0x000F, 0x14))]
    check('daynight: the light sensor bright, then dark after qom-set, '
          'PM_IRIN low', vals, [0x3FF, 0x8, 0x11])


def i2c_device_at(qtree, model):
    """The I2C address of a @model in `info qtree` output; None if absent."""
    m = re.search(r'dev: %s, id ""[\s\S]*?address = \d+ \(0x([0-9a-f]+)\)'
                  % model, qtree)
    return int(m.group(1), 16) if m else None


def test_msc316(qemu, tmp):
    print('msc316dc, msc316dm: the DDR comes from the IPL, not from presets')
    path = os.path.join(tmp, 'i3-316.bin')
    make_i3_image(path)
    for machine in ('msc316dc', 'msc316dm'):
        qt = QTest(qemu, machine + ',flash-file=' + path, [])
        try:
            check(machine + ': IPL from NOR 0x4000', qt.readl(IMI + 4),
                  0x5F4C5049)
            check(machine + ': chip ID', qt.readw(riu(0x001E, 0x00)), 0xC2)
            check(machine + ': 128 MiB, no wrap at 64',
                  qt.writel(DRAM + 0x4000000, 0x31364D53) or
                  qt.writel(DRAM, 0) or qt.readl(DRAM + 0x4000000),
                  0x31364D53)
            # 0x0809 here, the SSD202D capture's, sends its IPL round the
            # DDR set-up altogether: "D-ff" and nothing written to the PHY
            check(machine + ': no SSD202D efuse', qt.readw(0x1F004014), 0)
            check(machine + ': SPI NOR on the DID key strap',
                  qt.readw(0x1F0071C0), 0x20)
            check(machine + ': nothing presets the DDR',
                  [qt.readw(riu(0x1010, r)) for r in (0x00, 0x18, 0x19)] +
                  [qt.readw(riu(0x1012, 0x69))], [0, 0, 0, 0])
        finally:
            qt.close()


IVE0 = 0x1F2A4000
IVE1 = 0x1F2A4200
IVE_SRC = DRAM + 0x100000
IVE_SRC2 = DRAM + 0x110000
IVE_DST = DRAM + 0x120000
IVE_DST2 = DRAM + 0x130000


def ive_run(qt, op, mode, w, h, a, b=b'', regs=(), fmt=0, sa=None, da=None,
            stride=None):
    """hal_ive.c's order: reset, operation, images, coefficients, fire."""
    b64write(qt, IVE_SRC, a)
    if b:
        b64write(qt, IVE_SRC2, b)
    qt.writew(IVE0 + 0x02 * 4, 1)
    qt.writew(IVE0 + 0x02 * 4, 0)
    qt.writew(IVE0 + 0x10 * 4, 0xFE)
    qt.writew(IVE1 + 0x04 * 4, op | mode << 8)
    qt.writew(IVE1 + 0x05 * 4, fmt)
    qt.writew(IVE1 + 0x06 * 4, w - 1)
    qt.writew(IVE1 + 0x07 * 4, h - 1)
    src = sa if sa is not None else IVE_SRC - DRAM
    for reg, addr in ((0x08, src), (0x0A, IVE_DST - DRAM),
                      (0x0C, IVE_SRC2 - DRAM), (0x0E, IVE_DST2 - DRAM)):
        qt.writew(IVE1 + reg * 4, addr & 0xFFFF)
        qt.writew(IVE1 + (reg + 1) * 4, addr >> 16)
    for reg in (0x14, 0x15, 0x16, 0x17):
        qt.writew(IVE1 + reg * 4, (stride or w) - 1)
    for reg, v in regs:
        qt.writew(IVE1 + reg * 4, v)
    qt.writew(IVE0, 1)
    check('IVE op 0x%x: frame done' % op, qt.readw(IVE0 + 0x13 * 4) & 1, 1)
    qt.writew(IVE0 + 0x15 * 4, 1)
    return b64read(qt, IVE_DST, 0x400)


def ive_mask(values, shift=0):
    regs = []
    v = [x & 0xFF for x in values]
    for i in range(0, 24, 2):
        regs.append((0x1A + i // 2, v[i] | v[i + 1] << 8))
    regs.append((0x26, v[24] | shift << 8))
    return regs


def test_i3_ive(qemu, tmp):
    print('infinity3 IVE: the driver\'s sequence, each operation')
    path = os.path.join(tmp, 'i3-ive.bin')
    make_i3_image(path)
    qt = QTest(qemu, 'msc313e,flash-file=' + path, [])
    try:
        w, h = 16, 5
        ramp = bytes((x * 10 + y) & 0xFF for y in range(h) for x in range(w))
        other = bytes((x + y * 3) & 0xFF for y in range(h) for x in range(w))
        n = w * h
        check('interrupts masked from reset', qt.readw(IVE0 + 0x10 * 4), 0xFF)

        # identity filter
        out = ive_run(qt, 0x00, 0, w, h, ramp,
                      regs=ive_mask([0] * 12 + [1] + [0] * 12))
        check('filter: identity', out[:n], ramp)
        # a shift of 1 halves a mask of 2
        out = ive_run(qt, 0x00, 0, w, h, ramp,
                      regs=ive_mask([0] * 12 + [2] + [0] * 12, shift=1))
        check('filter: the mask\'s shift', out[:n], ramp)
        # negative coefficients clip to 0
        out = ive_run(qt, 0x00, 0, w, h, bytes([100] * n),
                      regs=ive_mask([0] * 12 + [-1] + [0] * 12))
        check('filter: clips at 0', out[:4], bytes(4))

        # stride in pixels
        padded = b''.join(ramp[y * w:(y + 1) * w] + b'\xEE' * 4
                          for y in range(h))
        out = ive_run(qt, 0x00, 0, w, h, padded,
                      regs=ive_mask([0] * 12 + [1] + [0] * 12), stride=w + 4)
        check('source and destination strides', out[w + 4:2 * w + 4],
              ramp[w:2 * w])

        # dilate and erode a single bright pixel
        dot = bytearray(n)
        dot[2 * w + 8] = 255
        out = ive_run(qt, 0x07, 0, w, h, bytes(dot),
                      regs=ive_mask([0] * 7 + [255] * 3 + [0, 255, 255, 255, 0]
                                    + [255] * 3 + [0] * 7 + [0] * 0))
        check('dilate spreads under the mask', out[2 * w + 7], 255)
        check('dilate stops at its edge', out[2 * w + 5], 0)
        out = ive_run(qt, 0x08, 0, w, h, bytes(dot),
                      regs=ive_mask([255] * 25))
        check('erode removes it', out[2 * w + 8], 0)

        # thresholds: mi_ive_datatype.h's table
        src = bytes(range(0, 160, 2))
        for mode, want in ((0, (1, 3, 3)), (1, (10, 3, 3)), (2, (1, 40, 90)),
                           (3, (1, 2, 3)), (4, (10, 2, 3)), (5, (1, 2, 90)),
                           (6, (1, 40, 3)), (7, (10, 2, 90))):
            out = ive_run(qt, 0x09, mode, w, h, src,
                          regs=((0x1A, 1 | 2 << 8), (0x1B, 3),
                                (0x28, 20), (0x29, 60)))
            check('threshold mode %d' % mode,
                  (out[5], out[20], out[45]), want)

        # logic and arithmetic
        out = ive_run(qt, 0x0C, 0, w, h, ramp, other)
        check('and', out[:n], bytes(a & b for a, b in zip(ramp, other)))
        out = ive_run(qt, 0x0D, 0, w, h, ramp, other)
        check('or', out[:n], bytes(a | b for a, b in zip(ramp, other)))
        out = ive_run(qt, 0x0E, 0, w, h, ramp, other)
        check('xor', out[:n], bytes(a ^ b for a, b in zip(ramp, other)))
        out = ive_run(qt, 0x0F, 0, w, h, ramp, other,
                      regs=((0x2B, 0x4000), (0x2C, 0xC000)))
        check('add: xA + yB in Q16', out[:n],
              bytes(min(255, (a * 0x4000 + b * 0xC000 + 0x8000) >> 16)
                    for a, b in zip(ramp, other)))
        out = ive_run(qt, 0x10, 0, w, h, ramp, other)
        check('sub: absolute', out[:n],
              bytes(abs(a - b) for a, b in zip(ramp, other)))
        out = ive_run(qt, 0x10, 1, w, h, ramp, other)
        check('sub: shifted, sign kept', out[:n],
              bytes(((a - b) >> 1) & 0xFF for a, b in zip(ramp, other)))

        # the map table in source 1, the image in source 2
        table = bytes(255 - i for i in range(256))
        out = ive_run(qt, 0x12, 0, w, h, table, ramp)
        check('map', out[:n], bytes(255 - a for a in ramp))

        out = ive_run(qt, 0x13, 0, w, h, bytes([7] * 30 + [9] * 50))
        check('histogram', struct.unpack('<256I', out[:1024])[7:10],
              (30, 0, 50))

        # integral image, sum only
        out = ive_run(qt, 0x14, 1, w, h, bytes([2] * n))
        check('integral: bottom right', struct.unpack('<I', out[n * 4 - 4:n * 4])[0],
              2 * n)

        # SAD over 4x4 blocks into 16 bits
        a = bytes([10] * 64)
        b = bytes([13] * 64)
        out = ive_run(qt, 0x15, 0, 16, 4, a + b'\0' * 0, b,
                      regs=((0x05, 2 << 8), ))
        check('SAD: 4x4 of 3', struct.unpack('<H', out[:2])[0], 48)

        out = ive_run(qt, 0x16, 0, w, h, bytes([2] * n), bytes([3] * n))
        check('NCC: numerator, sum1, sum2',
              struct.unpack('<3Q', out[:24]), (6 * n, 4 * n, 9 * n))

        # LBP: a brighter neighbour at the top left sets bit 0
        lbp = bytearray([50] * n)
        lbp[1 * w + 3] = 90
        out = ive_run(qt, 0x18, 0, w, h, bytes(lbp), regs=((0x28, 9),))
        check('LBP: a neighbour over the threshold', out[2 * w + 4], 1)

        out = ive_run(qt, 0x18, 0, w, h, bytes(lbp), regs=((0x28, 40),))
        check('LBP: the driver\'s thresh - 1, so at it is not over',
              out[2 * w + 4], 0)

        # strides count pixels, 16-bit ones too
        rows = [struct.pack('<%dH' % w, *([300 + 20 * y] * w)) + b'\xEE' * 8
                for y in range(h)]
        out = ive_run(qt, 0x11, 3, w, h, b''.join(rows),
                      regs=((0x2A, 0x8000),), stride=w + 4)
        check('16 to 8: a 16-bit stride', out[w + 4], 160)

        # 16 to 8: 1/2 of a 16-bit value
        v = struct.pack('<%dH' % n, *([300] * n))
        out = ive_run(qt, 0x11, 3, w, h, v, regs=((0x2A, 0x8000),))
        check('16 to 8: U16 scaled', out[:2], bytes([150, 150]))

        # order statistics
        spot = bytearray([10] * n)
        spot[2 * w + 8] = 200
        out = ive_run(qt, 0x05, 0, w, h, bytes(spot))
        check('median removes a spot', out[2 * w + 8], 10)
        out = ive_run(qt, 0x05, 1, w, h, bytes(spot))
        check('max spreads it', out[2 * w + 7], 200)

        # Sobel, horizontal gradient of a ramp
        s = [0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, -2, 0, 2, 0, 0, -1, 0, 1, 0,
             0, 0, 0, 0, 0]
        grad = bytes(x * 10 for _ in range(h) for x in range(w))
        out = ive_run(qt, 0x03, 0, w, h, grad, regs=ive_mask(s) +
                      [(0x05, 1 << 8)])
        check('sobel: horizontal, S16',
              struct.unpack('<h', out[(2 * w + 5) * 2:(2 * w + 5) * 2 + 2])[0],
              80)

        # Bernsen 3x3: the local midpoint
        out = ive_run(qt, 0x06, 0, w, h, bytes(spot))
        check('bernsen: over the midpoint', out[2 * w + 8], 255)
        check('bernsen: under it', out[2 * w + 7], 0)

        # the 16-bit thresholds: low, between, high
        v = struct.pack('<%dh' % n, *([-100, 5, 100] * n)[:n])
        out = ive_run(qt, 0x0A, 0, w, h, v,
                      regs=((0x1A, 1 | 2 << 8), (0x1B, 3),
                            (0x28, 0xFFF6), (0x29, 50)))
        check('threshold S16: signed against signed', out[:3],
              bytes([1, 2, 3]))
        v = struct.pack('<%dH' % n, *([10, 300, 4000] * n)[:n])
        out = ive_run(qt, 0x0B, 1, w, h, v,
                      regs=((0x1A, 1 | 2 << 8), (0x1B, 3),
                            (0x28, 200), (0x29, 500)))
        check('threshold U16: the original between', out[:3],
              bytes([1, 44, 3]))

        # magnitude, the angle second
        out = ive_run(qt, 0x04, 1, w, h, grad, regs=ive_mask(s) +
                      [(0x05, 1 << 8), (0x28, 1)])
        check('magnitude: |Gx| + |Gy|',
              struct.unpack('<H', out[(2 * w + 5) * 2:(2 * w + 5) * 2 + 2])[0],
              80)
        check('magnitude: a horizontal gradient\'s angle is 0',
              qt.readb(IVE_DST2 + 2 * w + 5), 0)

        # integral, sum and square sum together
        out = ive_run(qt, 0x14, 0, w, h, bytes([2] * n))
        q = struct.unpack('<Q', out[(n - 1) * 8:n * 8])[0] if n * 8 <= 0x400 \
            else 0
        check('integral both: sum under the square sum',
              (q & 0xFFFFFFF, q >> 28), (2 * n, 4 * n))

        # CSC, packed: unity with an offset and a clamp
        rgb = bytes([10, 20, 30] * n)
        regs = [(0x30, 0x100), (0x31, 0), (0x32, 0), (0x33, 0), (0x34, 0x100),
                (0x35, 0), (0x36, 0), (0x37, 0), (0x38, 0x100),
                (0x39, 5), (0x3A, 0), (0x3B, 0xFFF),
                (0x3C, 0xFF00), (0x3D, 0x0F00), (0x3E, 0xFF21)]
        out = ive_run(qt, 0x01, 0, w, h, rgb, regs=regs, fmt=4 | 4 << 8)
        check('CSC: Q8, signed offsets, both clamps', out[:3],
              bytes([15, 15, 33]))

        # filter and CSC together: the filter reaches the first channel
        b64write(qt, IVE_SRC2 + 0x8000, bytes([30] * n))
        out = ive_run(qt, 0x02, 0, w, h, bytes([100] * n), bytes([20] * n),
                      regs=ive_mask([0] * 12 + [2] + [0] * 12) + regs +
                      [(0x10, (IVE_SRC2 + 0x8000 - DRAM) & 0xFFFF),
                       (0x11, (IVE_SRC2 + 0x8000 - DRAM) >> 16),
                       (0x18, w - 1), (0x19, w - 1)],
                      fmt=2 | 4 << 8)
        check('filter and CSC: the filtered channel', out[:3],
              bytes([205, 15, 33]))

        # bits 0 clear: the interrupt masked in, then cleared
        check('frame done cleared', qt.readw(IVE0 + 0x13 * 4) & 1, 0)
        check('cycle counter ran', qt.readw(IVE0 + 0x40 * 4) != 0, True)
    finally:
        qt.close()


def test_sensor_catalogue(qemu):
    print('image sensor catalogue (hw/i2c/image-sensor.c)')
    for name, model, addr in (('imx307', 'hisi-imx307', 0x1A),
                              ('imx323', 'imx323', 0x1A),
                              ('sc2310', 'hisi-smartsens', 0x30),
                              ('sc2315e', 'hisi-smartsens', 0x30),
                              ('gc4653', 'galaxycore-sensor', 0x29),
                              ('ov2735', 'ingenic-ov2735', 0x3C)):
        out = hmp(qemu, ['-M', 'msc313e,sensor=' + name, '-S'], ['info qtree'])
        check('sensor=%s: %s at 0x%02x' % (name, model, addr),
              i2c_device_at(out, model), addr)
    out = hmp(qemu, ['-M', 'msc313e,sensor=nosuch', '-S'], [])
    check('an unknown sensor is refused, the catalogue listed',
          "unknown sensor 'nosuch'" in out and 'imx307' in out, True)
    out = hmp(qemu, ['-M', 'msc313e,rtc=ds1338', '-S'], ['info qtree'])
    check('rtc=ds1338: a DS1307 at 0x68', i2c_device_at(out, 'ds1338'), 0x68)
    out = hmp(qemu, ['-M', 'msc313e,rtc=nosuch', '-S'], [])
    check('an unknown rtc is refused', "unknown rtc 'nosuch'" in out, True)
    out = hmp(qemu, ['-M', 'ssd201,rtc=ds1338', '-S'], [])
    check('rtc= is refused where there is no I2C bus',
          'no I2C bus for an rtc' in out, True)
    p = subprocess.run([qemu, '-M', 'none', '-display', 'none', '-nodefaults',
                        '-qmp', 'stdio'], capture_output=True, text=True,
                       timeout=60,
                       input='{"execute":"qmp_capabilities"}\n'
                             '{"execute":"qom-list-types","arguments":'
                             '{"implements":"image-sensor"}}\n'
                             '{"execute":"quit"}\n')
    for model in ('galaxycore-sensor', 'ingenic-ov2735', 'hisi-smartsens',
                  'hisi-imx307', 'imx323', 'hisi-jxf22'):
        check('%s gives an ISP its exposure' % model,
              '"%s"' % model in p.stdout, True)


PMGPIO = riu(0x000F, 0)
PADGPIO = riu(0x103C, 0)
SAR = riu(0x0014, 0)
PMSLEEP = riu(0x000E, 0)
BACH1 = riu(0x1502, 0)
BACH2 = riu(0x1503, 0)


SDIO0 = riu(0x1410, 0)
SDIO0_CIFD = riu(0x1411, 0)
SD_BLOCKS = 8192                                # a 4 MiB card


def sd_pattern(block):
    return bytes((block * 7 + i) & 0xFF for i in range(512))


def sd_cmd(qt, idx, arg, rsp=5, data=0, mode=0):
    """One FCIE5 job as hal_sdmmc_v5.c runs it; (MIE_EVENT, SD_STS, rsp bytes)"""
    r = lambda n: SDIO0 + n * 4
    qt.writew(r(0x20), (arg >> 24 & 0xFF) << 8 | 0x40 | idx)
    qt.writew(r(0x21), (arg >> 8 & 0xFF) << 8 | (arg >> 16 & 0xFF))
    qt.writew(r(0x22), arg & 0xFF)
    qt.writew(r(0x0A), 5 << 8 | rsp)
    qt.writew(r(0x07), 0x04)                    # SDIO_MODE
    qt.writew(r(0x0B), 0x03 | mode)             # CLK_EN, 4-bit
    qt.writew(r(0x0D), 0x3F)                    # SD_STS clear
    qt.writew(r(0x00), 0x3F)                    # MIE_EVENT clear
    ctl = (3 if rsp == 16 else 2 if rsp else 0) | data
    qt.writew(r(0x0C), ctl)
    qt.writew(r(0x0C), ctl | 0x04 | 0x40)       # CMD_EN, JOB_START
    ev, sts = qt.readw(r(0x00)), qt.readw(r(0x0D)) & 0x3F
    out = []
    for i in range(rsp):
        w = qt.readw(r(0x20 + i // 2))
        out.append(w >> 8 if i & 1 else w & 0xFF)
    return ev, sts, out


def sd_job(qt, blocks, size, addr):
    r = lambda n: SDIO0 + n * 4
    qt.writew(r(0x09), size)
    qt.writew(r(0x08), blocks)
    qt.writew(r(0x05), blocks * size & 0xFFFF)
    qt.writew(r(0x06), blocks * size >> 16 & 0xFFFF)
    qt.writew(r(0x03), addr & 0xFFFF)
    qt.writew(r(0x04), addr >> 16 & 0xFFFF)


def sd_card_init(qt):
    sd_cmd(qt, 0, 0, rsp=0)
    ev, sts, rsp = sd_cmd(qt, 8, 0x1AA)
    check('SD: CMD8 echoes its check pattern', rsp[4] if rsp else None, 0xAA)
    for _ in range(20):
        sd_cmd(qt, 55, 0)
        ev, sts, rsp = sd_cmd(qt, 41, 0x00300000)
        if rsp[1] & 0x80:
            break
        qt.step(10 ** 8)
    check('SD: ACMD41 reports the card powered up', rsp[1] >> 7 & 1, 1)
    ev, sts, rsp = sd_cmd(qt, 2, 0, rsp=16)
    check('SD: CMD2 (R2) returns 0x3F, then the CID', rsp[0], 0x3F)
    ev, sts, rsp = sd_cmd(qt, 3, 0)
    rca = rsp[1] << 8 | rsp[2]
    sd_cmd(qt, 7, rca << 16)
    sd_cmd(qt, 16, 512)
    return rca


def test_msc313e_sd(qemu, tmp):
    print('msc313e SD card on SDIO0 (hal_sdmmc_v5.c, hal_card_platform_iNF3.c)')
    path = os.path.join(tmp, 'i3.bin')
    make_i3_image(path)
    card = os.path.join(tmp, 'sd.img')
    with open(card, 'wb') as f:
        for b in range(SD_BLOCKS):
            f.write(sd_pattern(b))
    r = lambda n: SDIO0 + n * 4

    qt = QTest(qemu, 'msc313e,flash-file=' + path, ['-rtc', 'clock=vm'])
    try:
        check('SD: no card -> PM_SD_CDZ reads high',
              qt.readw(riu(0x000F, 0x47)) >> 2 & 1, 1)
        ev, sts, rsp = sd_cmd(qt, 8, 0x1AA)
        check('SD: no card -> CMD_END with CMD_NORSP', (ev & 2, sts & 8), (2, 8))
    finally:
        qt.close()

    qt = QTest(qemu, 'msc313e,flash-file=' + path,
               ['-rtc', 'clock=vm', '-drive', 'if=sd,format=raw,file=' + card])
    try:
        check('SD: card in -> PM_SD_CDZ reads low',
              qt.readw(riu(0x000F, 0x47)) >> 2 & 1, 0)
        qt.writew(r(0x3F), 0)
        qt.writew(r(0x3F), 1)
        sd_card_init(qt)

        sd_job(qt, 2, 512, 0x100000)            # DMA read, blocks 5-6
        ev, sts, _ = sd_cmd(qt, 18, 5 * 512, data=0x08)
        got = bytes(qt.readb(DRAM + 0x100000 + i) for i in (0, 1, 511, 512, 1023))
        want = sd_pattern(5)[0:2] + sd_pattern(5)[511:512] + \
            sd_pattern(6)[0:1] + sd_pattern(6)[511:512]
        check('SD: DMA read of two blocks ends with DATA_END', (ev & 3, sts), (3, 0))
        check('SD: and lands in DDR', got, want)
        sd_cmd(qt, 12, 0)

        for i in range(0, 512, 4):              # DMA write, block 9
            qt.writel(DRAM + 0x200000 + i, 0xA5A50000 | i)
        sd_job(qt, 1, 512, 0x200000)
        ev, sts, _ = sd_cmd(qt, 24, 9 * 512, data=0x10)
        qt.writew(r(0x00), 0x3F)
        qt.writew(r(0x0C), 0x10 | 0x08 | 0x40)  # JOB_DIR, DTRX_EN, JOB_START
        check('SD: DMA write ends with DATA_END', qt.readw(r(0x00)) & 1, 1)
        sd_job(qt, 1, 512, 0x300000)
        sd_cmd(qt, 17, 9 * 512, data=0x08)
        check('SD: and reads back', qt.readl(DRAM + 0x300000 + 0x40), 0xA5A50040)

        # A buffered (CIFD, DEST_R2N) read that fits one 64-byte buffer
        sd_cmd(qt, 16, 64)
        sd_job(qt, 1, 64, 0)
        ev, sts, _ = sd_cmd(qt, 17, 3 * 512, data=0x08, mode=0x10)
        check('SD: CIFD read fills the buffer', qt.readw(r(0x30)) & 1, 1)
        check('SD: its first bytes', qt.readw(SDIO0_CIFD), (3 * 7 + 1) << 8 | 3 * 7)
        qt.writew(r(0x30), 0x01)                # WBUF_FULL done
        qt.writew(r(0x30), 0x02)                # WBUF_EMPTY_TRIG
        check('SD: a one-buffer CIFD read ends with DATA_END', qt.readw(r(0x00)) & 1, 1)
        sd_cmd(qt, 16, 512)

        # A single-block read with a job of 0xFFFF blocks of 0xFFFF bytes:
        # the card sends one block, then nothing
        sd_job(qt, 0xFFFF, 0xFFFF, 0x100000)
        ev, sts, _ = sd_cmd(qt, 17, 1 * 512, data=0x08)
        check('SD: a ~4 GiB job on a one-block read ends in RD_TOUT, no DATA_END',
              (ev & 1, sts & 0x20), (0, 0x20))
        sd_job(qt, 1, 512, 0x100000)
        ev, sts, _ = sd_cmd(qt, 17, 2 * 512, data=0x08)
        check('SD: and the host still reads after it', (ev & 1, qt.readb(DRAM + 0x100001)),
              (1, (2 * 7 + 1) & 0xFF))

        # DMA to and from an address with nothing behind it (MIU 0x50000000)
        sd_job(qt, 1, 512, 0x50000000)
        ev, sts, _ = sd_cmd(qt, 17, 4 * 512, data=0x08)
        check('SD: DMA read to nowhere: an error, no DATA_END',
              (ev & 5, sts & 0x20), (4, 0x20))
        sd_job(qt, 1, 512, 0x50000000)
        ev, sts, _ = sd_cmd(qt, 24, 10 * 512, data=0x10)
        qt.writew(r(0x00), 0x3F)
        qt.writew(r(0x0C), 0x10 | 0x08 | 0x40)
        check('SD: DMA write from nowhere: an error, no DATA_END',
              (qt.readw(r(0x00)) & 5, qt.readw(r(0x0D)) & 0x04), (4, 0x04))
        qt.writew(r(0x3F), 0)
        qt.writew(r(0x3F), 1)
        sd_cmd(qt, 12, 0)
        sd_job(qt, 1, 512, 0x300000)
        sd_cmd(qt, 17, 10 * 512, data=0x08)
        check('SD: and block 10 kept its data', qt.readb(DRAM + 0x300000 + 3),
              (10 * 7 + 3) & 0xFF)
    finally:
        qt.close()


def intc_raw(qt, n):
    """The main intc's raw MS_IRQ input n (the GIC SPI 32 + n)."""
    return qt.readw(riu(0x1009, 0x5C + n // 16)) >> (n % 16) & 1


def test_i3_gpio(qt):
    print('msc313e GPIO pads and SAR (mhal_gpio.c, mdrv_sar.c)')
    check('PM_SD_CDZ input, high: no SD card',
          qt.readb(PMGPIO + 0x47 * 4) & 5, 5)
    check('PM_IRIN by day: input, high', qt.readb(PMGPIO + 0x14 * 4) & 5, 5)
    a = PMGPIO + 0x01 * 4                       # PM_GPIO1, the amp enable
    qt.writeb(a, qt.readb(a) & ~1 | 2)          # Set_High: OEN off, OUT on
    check('PM_GPIO1 driven high reads high', qt.readb(a) & 4, 4)
    qt.writeb(a, qt.readb(a) & ~2)
    check('and driven low reads low', qt.readb(a) & 4, 0)
    qt.writeb(a, qt.readb(a) | 1)               # Pad_Odn: an input again
    check('released, its pull-up', qt.readb(a) & 4, 4)
    a = PADGPIO + 0x05 * 4                      # GPIO5
    check('PADGPIO GPIO5 input reads high', qt.readb(a) & 0x31, 0x21)
    qt.writeb(a, qt.readb(a) & ~0x20 & ~0x10)
    check('driven low reads low', qt.readb(a) & 1, 0)
    qt.writeb(a, qt.readb(a) | 0x10)
    check('driven high reads high', qt.readb(a) & 1, 1)
    check('a register that is no pad is storage',
          (qt.writew(PADGPIO + 0x12 * 4, 0x1234),
           qt.readw(PADGPIO + 0x12 * 4))[1], 0x1234)
    check('SAR_GPIO0-3 inputs read high', qt.readb(SAR + 0x12 * 4 + 1), 0x0F)
    qt.writeb(SAR + 0x11 * 4 + 1, 0x0C)         # SAR_GPIO0/1 outputs
    qt.writeb(SAR + 0x12 * 4, 0x02)             # 0 low, 1 high
    check('SAR_GPIO0/1 (the IR-cut motor) read what they drive',
          qt.readb(SAR + 0x12 * 4 + 1), 0x0E)
    qt.writew(SAR, 0x0A20 | 0x4000)             # ms_sar_get: start
    check('SAR conversion done at once', qt.readw(SAR) & 0x4000, 0)
    check('SAR channel 2, the light sensor, by day', qt.readw(SAR + 0x42 * 4),
          0x3FF)


def test_i3_pmsleep(qt):
    print('msc313e PM-sleep interrupts (irqchip.c, ms_ircut.c)')
    irin = PMGPIO + 0x14 * 4                    # hwirq 22
    qt.writew(riu(0x1009, 0x54), 0xFFFB)        # unmask MS_IRQ 2
    check('nothing pending: the cascade low', intc_raw(qt, 2), 0)
    qt.writew(irin, qt.readw(irin) | 0x80)      # falling edge
    qt.writew(irin, qt.readw(irin) & ~0x10)     # unmask
    check('unmasked with no edge: still low', intc_raw(qt, 2), 0)
    qt.cmd('set_irq_in /machine/gpio pm-pad 20 0')  # night falls
    check('PM_IRIN reads low', qt.readw(irin) & 4, 0)
    check('the falling edge in the final status (bit 22)',
          qt.readw(PMSLEEP + 4 * 4) | qt.readw(PMSLEEP + 5 * 4) << 16,
          1 << 22)
    check('and on MS_IRQ 2, GIC SPI 34', intc_raw(qt, 2), 1)
    qt.writew(irin, qt.readw(irin) | 0x10)      # mask
    check('masked: final status clear', qt.readw(PMSLEEP + 5 * 4), 0)
    check('and the cascade low', intc_raw(qt, 2), 0)
    qt.writew(irin, qt.readw(irin) & ~0x10)
    check('unmasked: the edge is still latched', intc_raw(qt, 2), 1)
    qt.writew(irin, qt.readw(irin) | 0x40)      # ack (SETREG16 BIT6)
    check('ack clears it', (qt.readw(PMSLEEP + 5 * 4), intc_raw(qt, 2)),
          (0, 0))
    check('and the clear bit reads 0', qt.readw(irin) & 0x40, 0)
    qt.cmd('set_irq_in /machine/gpio pm-pad 20 1')  # day: a rising edge
    check('a rising edge with the falling polarity: nothing',
          intc_raw(qt, 2), 0)
    qt.writew(irin, qt.readw(irin) & ~0x80)     # the handler: rising next
    qt.cmd('set_irq_in /machine/gpio pm-pad 20 0')
    qt.cmd('set_irq_in /machine/gpio pm-pad 20 1')
    check('the next rising edge raises it', intc_raw(qt, 2), 1)
    qt.writew(irin, qt.readw(irin) | 0x40 | 0x10)
    check('acked and masked: low', intc_raw(qt, 2), 0)
    qt.cmd('set_irq_in /machine/gpio pm-irq 1 1')   # SAR, hwirq 79
    check('a PM IRQ-group input while masked: low', intc_raw(qt, 2), 0)
    qt.writeb(PMSLEEP + 8 * 4, 0xFD)            # unmask hwirq 79
    check('unmasked: in 0x0E and on the cascade',
          (qt.readw(PMSLEEP + 0x0E * 4), intc_raw(qt, 2)), (2, 1))
    qt.cmd('set_irq_in /machine/gpio pm-irq 1 0')
    check('and gone with its level', intc_raw(qt, 2), 0)
    qt.writeb(PMSLEEP + 8 * 4, 0xFF)


def test_i3_bach_capture(qt):
    print('msc313e BACH capture (infinity.c, infinity_pcm.c)')
    buf = DRAM + 0x01000000                     # MIU 0x01000000
    ctl0, wr, flags = BACH1 + 0x40 * 4, BACH1 + 0x49 * 4, BACH1 + 0x48 * 4

    def level():
        qt.writew(wr, qt.readw(wr) | 0x1000)    # latch
        v = qt.readw(BACH1 + 0x4F * 4)
        qt.writew(wr, qt.readw(wr) & ~0x1000)
        return v

    qt.writew(BACH2 + 0x07 * 4, 0x0002)         # REG_DMA_INT_EN
    qt.writew(BACH1 + 0x01 * 4, 1 << 8 | 1 << 10)   # writer 16 kHz
    qt.writew(BACH1 + 0x77 * 4, 0x4000)         # WR_MONO
    qt.writew(wr, 0x000)                        # address (8-byte units)
    qt.writew(BACH1 + 0x4A * 4, 0x200)
    qt.writew(BACH1 + 0x4B * 4, 4096 // 8)      # 4 KiB buffer
    qt.writew(BACH1 + 0x4D * 4, 1024 // 8)      # overrun: the period
    qt.writew(BACH1 + 0x4E * 4, 256 // 8)       # underrun: 256 bytes
    qt.writew(ctl0, 0x0200)                     # WR_FULL clear pulse
    qt.writew(ctl0, 0x0000)
    qt.writew(ctl0, 0x4800)                     # FULL, OVERRUN enables
    qt.writew(wr, 0x8000)                       # WR_ENABLE
    qt.step(20 * 10 ** 6)
    check('20 ms into a 32 ms period: nothing yet', (level(),
          intc_raw(qt, 42)), (0, 0))
    qt.step(15 * 10 ** 6)
    check('a period in, plus the local buffer', level(), 128 + 8)
    check('WR_OVERRUN raises GIC SPI 74 (MS_IRQ 42)',
          (qt.readw(flags) & 0x23, intc_raw(qt, 42)), (0x02, 1))
    check('the tone in DRAM', qt.readw(buf + 4), 5656)
    qt.writew(BACH2 + 0x07 * 4, 0)
    check('BANK2 0x0E bit 1 gates it', intc_raw(qt, 42), 0)
    qt.writew(BACH2 + 0x07 * 4, 0x0002)
    qt.writew(ctl0, 0x8800)                     # ISR: OVERRUN off, UNDER on
    check('the ISR masks the overrun: low', intc_raw(qt, 42), 0)
    qt.writew(BACH1 + 0x4C * 4, 128)            # WR_SIZE: a period read
    qt.writew(wr, qt.readw(wr) ^ 0x2000)        # TRIG toggles
    check('TRIG takes WR_SIZE off the level', level(), 8)
    check('WR_UNDERRUN re-arms', (qt.readw(flags) & 1, intc_raw(qt, 42)),
          (1, 1))
    qt.writew(ctl0, 0x4800)                     # ISR: back to OVERRUN
    check('swapped back: low', intc_raw(qt, 42), 0)
    qt.step(200 * 10 ** 6)
    check('nobody reads: WR_FULL latches (an XRUN)',
          (qt.readw(flags) & 0x20, intc_raw(qt, 42)), (0x20, 1))
    qt.writew(ctl0, 0)                          # stop: enables off
    qt.writew(wr, qt.readw(wr) & ~0x8000)
    qt.writew(wr, qt.readw(wr) | 0x4000)        # INIT pulse
    qt.writew(wr, qt.readw(wr) & ~0x4000)
    qt.writew(ctl0, 0x0200)
    qt.writew(ctl0, 0x0000)
    check('stopped and INIT: level 0, FULL cleared, low',
          (level(), qt.readw(flags) & 0x20, intc_raw(qt, 42)), (0, 0, 0))


def test_i3_bach_playback(qt):
    print('msc313e BACH playback')
    ctl0, rd, flags = BACH1 + 0x40 * 4, BACH1 + 0x41 * 4, BACH1 + 0x48 * 4

    def level():
        qt.writew(rd, qt.readw(rd) | 0x1000)
        v = qt.readw(BACH1 + 0x47 * 4)
        qt.writew(rd, qt.readw(rd) & ~0x1000)
        return v

    def trig(units):
        qt.writew(BACH1 + 0x44 * 4, units)      # RD_SIZE
        qt.writew(rd, qt.readw(rd) ^ 0x2000)

    qt.writew(BACH1 + 0x01 * 4, 0 << 4)         # reader 8 kHz
    qt.writew(BACH1 + 0x77 * 4, 0xA000)         # RD_MONO | MONO_COPY
    qt.writew(rd, 0x000)
    qt.writew(BACH1 + 0x42 * 4, 0x220)          # MIU 0x01100000
    qt.writew(BACH1 + 0x43 * 4, 2048 // 8)
    qt.writew(BACH1 + 0x45 * 4, (2048 + 1024) // 8)
    qt.writew(BACH1 + 0x46 * 4, (2048 - 512) // 8)
    trig(2048 // 8)                             # the prefill
    check('TRIG adds RD_SIZE to the level', level(), 256)
    qt.writew(ctl0, 0x0100)                     # RD_EMPTY clear pulse
    qt.writew(ctl0, 0x0000)
    qt.writew(ctl0, 0x2400 | 0x0002)            # EMPTY, UNDERRUN; prefetch
    qt.writew(rd, qt.readw(rd) | 0x8000)        # RD_ENABLE
    qt.step(20 * 10 ** 6)
    check('20 ms at 16000 bytes/s played', level(), 256 - 40)
    check('above buffer - period: low', intc_raw(qt, 42), 0)
    qt.step(20 * 10 ** 6)
    check('a period played: RD_UNDERRUN raises it',
          (qt.readw(flags) & 0x1C, intc_raw(qt, 42)), (0x04, 1))
    qt.writew(ctl0, qt.readw(ctl0) & ~0x2000)   # ISR masks the underrun
    check('masked: low', intc_raw(qt, 42), 0)
    trig(512 // 8)                              # the copy refills a period
    qt.writew(ctl0, qt.readw(ctl0) | 0x2000)
    check('refilled and unmasked: low', (qt.readw(flags) & 0x04,
          intc_raw(qt, 42)), (0, 0))
    qt.step(300 * 10 ** 6)
    check('run dry: RD_EMPTY latches', qt.readw(flags) & 0x10, 0x10)
    qt.writew(ctl0, 0x0000)
    qt.writew(rd, qt.readw(rd) & ~0x8000)
    qt.writew(rd, qt.readw(rd) | 0x4000)
    qt.writew(rd, qt.readw(rd) & ~0x4000)
    check('stopped: low', intc_raw(qt, 42), 0)


I3_JPE = 0x1F264000
I3_VHE_CONF = 0x1F265000
I3_VHE_SW = 0x1F265200
I3_MFE = 0x1F264800                     # banks 0x1324-0x1325


def b64write(qt, a, data):
    qt.cmd('b64write 0x%x 0x%x %s' % (a, len(data),
                                       base64.b64encode(data).decode()))


def b64read(qt, a, n):
    return base64.b64decode(qt.cmd('b64read 0x%x 0x%x' % (a, n)).split()[1])


def test_i3_jpe(qt):
    def r(i):
        return I3_JPE + 4 * i

    def irq():                                  # GIC SPI 61, intc IRQ 29
        return qt.readw(riu(0x1009, 0x5C + 1)) >> 13 & 1

    w, h, src, out = 64, 32, 0x00100000, 0x00200000
    qt.writew(riu(0x1009, 0x54 + 1), 0)         # unmask MS_IRQ 16-31
    b64write(qt, DRAM + src,
             bytes([200] * (w * h)) + bytes([128] * (w * h // 2)))
    b64write(qt, DRAM + out, bytes(256))
    qt.writew(r(0x1C), 0)
    qt.writew(r(0x00), 0x03)                    # the stale start of _EncFire
    qt.writew(r(0x00), 0x00)                    # soft_rstz low
    qt.writew(r(0x00), 0x02)
    for i, (a, v) in enumerate(((out, 0), (out + 0xFFF, 1))):
        qt.writew(r(0x3C), a >> 2 & 0xFFFF)
        qt.writew(r(0x3D), a >> 18 | v << 14)
        qt.writew(r(0x3E), 1)
    qt.writew(r(0x3B), 1)                       # set_adr
    qt.writew(r(0x1D), 0x1FF)
    check('JPE: irq clear reads back 0', qt.readw(r(0x1D)), 0)
    qt.writew(r(0x01), w)
    qt.writew(r(0x02), h)
    qt.writew(r(0x5E), (w >> 4) * 0x101)
    qt.writew(r(0x03), 0)                       # tbc_mode: software
    for i in range(128):
        qt.writew(r(0x4A), 1 + (i & 7))
        qt.writew(r(0x49), 1 | i << 8)          # write entry i
        qt.writew(r(0x48), 1 << 12)
    check('JPE: table write done', qt.readw(r(0x49)) & 4, 4)
    qt.writew(r(0x49), qt.readw(r(0x49)) | 2)
    check('JPE: done_clr clears it', qt.readw(r(0x49)) & 4, 0)
    qt.writew(r(0x49), 77 << 8)                 # read entry 77 back
    qt.writew(r(0x48), 1 << 12)
    check('JPE: Q table entry reads back', (qt.readw(r(0x49)) & 4,
                                            qt.readw(r(0x4B))), (4, 6))
    qt.writew(r(0x03), 1 << 6)
    qt.writew(r(0x06), src >> 8 & 0xFFFF)
    qt.writew(r(0x07), src >> 24)
    qt.writew(r(0x0A), (src + w * h) >> 8 & 0xFFFF)
    qt.writew(r(0x0B), (src + w * h) >> 24)
    qt.step(10 ** 7)
    check('JPE: the stale start was cancelled by the reset',
          (qt.readw(r(0x1E)), irq()), (0, 0))
    qt.writew(r(0x00), 0x03)                    # frame_start
    check('JPE: frame_start reads back 0', qt.readw(r(0x00)) & 1, 0)
    qt.step(10 ** 7)
    check('JPE: frame done, GIC SPI 61 (intc IRQ 29)',
          (qt.readw(r(0x1E)), irq()), (1, 1))
    qt.writew(r(0x3B), 1 << 13)                 # rchk_en: s0 instead
    s0 = (qt.readw(r(0x44)) | qt.readw(r(0x45)) << 16) << 2
    qt.writew(r(0x3B), 0)
    wptr = (qt.readw(r(0x44)) | qt.readw(r(0x45)) << 16) << 2
    size = wptr - s0 + (qt.readw(r(0x42)) & 3) + 1
    check('JPE: rchk_en reads the start back', s0, out)
    scan = b64read(qt, DRAM + out, size + 4)
    check('JPE: a scan of more than the EOI', size > 16, True)
    check('JPE: it ends in EOI at the reported size',
          scan[size - 2:size + 1], b'\xff\xd9\x00')
    qt.writew(r(0x1D), 1)
    check('JPE: clearing frame done drops the line',
          (qt.readw(r(0x1E)), irq()), (0, 0))


def test_i3_vhe(qt):
    def sw(i, v):
        qt.writel(I3_VHE_SW + 8 * i, v & 0xFFFF)
        qt.writel(I3_VHE_SW + 8 * i + 4, v >> 16)

    def swr(i):
        return (qt.readl(I3_VHE_SW + 8 * i) & 0xFFFF |
                (qt.readl(I3_VHE_SW + 8 * i + 4) & 0xFFFF) << 16)

    def irq():                                  # GIC SPI 53, intc IRQ 21
        return qt.readw(riu(0x1009, 0x5C + 1)) >> 5 & 1

    w, h, src, out, nals = 64, 64, 0x00300000, 0x00400000, 0x00500000
    rec = (0x00600000, 0x00700000)
    pic = bytes((x * 4 if y < 32 else 255 - x * 4) & 0xFF
                for y in range(h) for x in range(w))
    b64write(qt, DRAM + src, pic + bytes([128] * (w * h // 2)))
    check('VHE: swreg0, the H2 product ID', swr(0) >> 16, 0x4832)
    for f in range(2):
        qt.writew(I3_VHE_CONF + 4 * 0x01, 0)    # _sw_rst
        qt.writew(I3_VHE_CONF + 4 * 0x01, 1)
        qt.writew(I3_VHE_CONF + 4 * 0x04, 0xF00)
        qt.writew(I3_VHE_CONF + 4 * 0x48, 0)
        if not f:
            check('VHE: conf irq clear reads back 0',
                  qt.readw(I3_VHE_CONF + 4 * 0x04), 0)
        regs = {
            4: 1 << 29 | 3 << 23 | 2 << 19 | 1 << 6 | 2 << 3 | 4,
            5: (w // 8) << 22 | (h // 8) << 11 | (0 if f else 1) << 1,
            6: 2 << 20 | 0xE << 11 | 5 << 7 | 1 << 1,
            7: 30 << 26 | 30 << 8,
            8: out + 100, 9: 0x10000, 10: nals, 11: f,
            12: src, 13: src + w * h,
            15: rec[f], 16: rec[f] + w * h,
            17: 1 << 6, 18: rec[0], 19: rec[0] + w * h,
            38: 1 << 28 | w << 6,
            164: (1 if f else 19) << 26,
        }
        for i in range(1, 165):
            sw(i, regs.get(i, 0))
        sw(5, regs[5] | 1)                      # ENC_E
        qt.step(2 * 10 ** 7)
        size = swr(9)
        data = b64read(qt, DRAM + out + 100, size)
        what = 'VHE: %s frame: ' % ('P' if f else 'I')
        check(what + 'swreg1 = IRQ | FRAME_RDY, ENC_E cleared',
              (swr(1), swr(5) & 1), (5, 0))
        check(what + 'interrupt on GIC SPI 53 (intc IRQ 21)', irq(), 1)
        check(what + 'NAL at BASE_OUTBS_ADDR (start code, type)',
              data[:6], bytes.fromhex('00000001' + ('0201' if f else '2601')))
        check(what + 'swreg9 bytes, ending in the trailing bits',
              size > 8 and data[-1] != 0, True)
        check(what + 'the NAL size table', struct.unpack(
            '<II', b64read(qt, DRAM + nals, 8)), (size, 0))
        qt.writew(I3_VHE_CONF + 4 * 0x48, 0xF)
        check(what + 'conf 0x48 masks the line', irq(), 0)
        sw(1, 0)                                # what _isr_func() does
        qt.writew(I3_VHE_CONF + 4 * 0x48, 0)
        check(what + 'swreg1 = 0 drops it', irq(), 0)
    check('VHE: the reconstruction is in DRAM for the next frame',
          b64read(qt, DRAM + rec[0], 4) != bytes(4), True)
    # a row length of 1, YUYV then NV12: chroma read inside the row
    for fmt in (2, 1):
        sw(38, fmt << 28 | 1 << 6)
        sw(9, 0x10000)                          # swreg9 holds the last size
        sw(5, regs[5] | 1)
        qt.step(2 * 10 ** 7)
        check('VHE: a row length of 1 (%s) still codes a frame' %
              ('YUYV' if fmt == 2 else 'NV12'), (swr(1), swr(5) & 1), (5, 0))
        sw(1, 0)


# The header bits h264_enc.c writes for a 32x32 stream at QP 26: SPS, PPS
# and the IDR slice header, Main profile with CABAC (ending in the
# cabac_alignment_one_bits), then a P slice header (cabac_init_idc 0); and
# Baseline with CAVLC, 225 bits.
MFE_CABAC_IDR = bytes.fromhex('00000001674d401e95a25a6c040000000168ee388000'
                              '00000165b804ff')
MFE_CABAC_P = bytes.fromhex('0000000141e023')
MFE_CAVLC_IDR = bytes.fromhex('000000016742401e95a25a6c040000000168ce388000'
                              '00000165b80480')
MFE_W, MFE_H, MFE_SRC, MFE_OUT = 32, 32, 0x00800000, 0x00900000
MFE_REC = (0x00A00000, 0x00B00000)


def mfe_frame(qt, f, pic, hdr, nbits, cabac=True, pitch=MFE_W, yuyv=False):
    """One frame as mfe6_ios.c _enc_fire() runs it; returns the output."""
    def r(i):
        return I3_MFE + 4 * i

    w, h, src, rec = MFE_W, MFE_H, MFE_SRC, MFE_REC
    b64write(qt, DRAM + src, pic)
    b64write(qt, DRAM + rec[f & 1], bytes(w * h * 3 // 2))
    qt.writew(r(0x1C), 0)
    qt.writew(r(0x03), cabac << 13 | 1 << 6)
    reg00 = 2 << 2 | (1 if f else 0) << 4 | 26 << 8     # H.264, P, qscale
    qt.writew(r(0x00), reg00)                   # soft_rstz low, then high
    qt.writew(r(0x00), reg00 | 2)
    qt.writew(r(0x01), w)
    qt.writew(r(0x02), h)
    qt.writew(r(0x90), pitch)
    qt.writew(r(0xF3), int(yuyv))
    for reg, a in ((0x06, src), (0x08, src + w * h),
                   (0x0A, rec[~f & 1]), (0x0E, rec[~f & 1] + w * h),
                   (0x12, rec[f & 1]), (0x14, rec[f & 1] + w * h)):
        qt.writew(r(reg), a >> 8 & 0xFFFF)
        qt.writew(r(reg + 1), a >> 24)
    for a, i in ((MFE_OUT, 0), (MFE_OUT + 0xFFFF, 1)):  # set_outbs_addr
        qt.writew(r(0x3E), 0)
        qt.writew(r(0x3C), a >> 3 & 0xFFFF)
        qt.writew(r(0x3D), a >> 19 | i << 14)
        qt.writew(r(0x3E), 1)
    qt.writew(r(0x1D), 0xFF)
    qt.writew(r(0x00), reg00 | 3)               # frame start
    words = (nbits + 15) // 16
    qt.writew(r(0x47), (words - 1) << 5)
    for i in range(words):                      # put_coded_bits()
        n = min(16, nbits - 16 * i)
        qt.writew(r(0x46), hdr[2 * i] << 8 | (hdr[2 * i + 1] if n > 8 else 0))
        qt.writew(r(0x47), (words - 1) << 5 | (n if i == words - 1 else 15))
        qt.writew(r(0x48), 1 << 11)
    qt.step(2 * 10 ** 7)
    bits = qt.readw(r(0x42)) | (qt.readw(r(0x43)) & 0xFF) << 16
    return bits, b64read(qt, DRAM + MFE_OUT, bits // 8)


def test_i3_mfe(qt):
    def irq():                                  # GIC SPI 60, intc IRQ 28
        return qt.readw(riu(0x1009, 0x5C + 1)) >> 12 & 1

    w, h = MFE_W, MFE_H
    size = w * h * 3 // 2
    # luma 100, chroma 128: MB 0 has no neighbours to predict 100 from
    # (I_PCM), the others are Intra16x16 with H or V prediction
    flat = bytes([100] * (w * h)) + bytes([128] * (w * h // 2))
    moved = bytearray(flat)
    for y in range(16, 32):                     # MB 3 changes: I_PCM
        moved[y * w + 16:y * w + 32] = bytes(range(y, y + 16))
    moved = bytes(moved)

    hdr = MFE_CABAC_IDR
    bits, data = mfe_frame(qt, 0, flat, hdr, 8 * len(hdr))
    what = 'MFE: CABAC IDR frame: '
    check(what + 'frame done, GIC SPI 60 (intc IRQ 28)',
          (qt.readw(I3_MFE + 4 * 0x1E) & 0xFF, irq()), (1, 1))
    check(what + 'the FDC header bits lead the output', data[:len(hdr)], hdr)
    pcm = data.find(bytes([100] * 256))
    check(what + 'I_PCM samples from a byte boundary after the flush, '
          'whole bytes, the trailing bits', (bits % 8, pcm, data[-1] != 0),
          (0, len(hdr) + 2, True))
    # what ffmpeg -err_detect explode decodes to the picture (the I_PCM
    # bin and flush before the samples, the Intra16x16 mb_types, chroma
    # modes, mb_qp_delta, coded_block_flags and end_of_slice after them)
    check(what + 'the slice data', (data[len(hdr):pcm].hex(),
                                    data[pcm + 384:].hex()),
          ('fef8', 'fac9242e0e'))
    check(what + 'the reconstruction', b64read(qt, DRAM + MFE_REC[0], size),
          flat)
    check('MFE: the sum of the qsteps of all MBs but the first (msb2)',
          qt.readw(I3_MFE + 4 * 0xF5) | qt.readw(I3_MFE + 4 * 0xF6) << 16, 0)
    qt.writew(I3_MFE + 4 * 0x27, 650)           # qp2qs(26)
    bits, data = mfe_frame(qt, 0, flat, hdr, 8 * len(hdr))
    check('MFE: ... with the frame qstep set, 3 x it',
          qt.readw(I3_MFE + 4 * 0xF5) | qt.readw(I3_MFE + 4 * 0xF6) << 16,
          3 * 650)
    qt.writew(I3_MFE + 4 * 0x27, 0)
    qt.writew(I3_MFE + 4 * 0x1D, 1)
    check('MFE: clearing frame done drops the line',
          (qt.readw(I3_MFE + 4 * 0x1E), irq()), (0, 0))

    hdr = MFE_CABAC_P
    bits, data = mfe_frame(qt, 1, flat, hdr, 8 * len(hdr))
    check('MFE: CABAC P frame, nothing moved: P_Skip, end_of_slice_flag',
          (qt.readw(I3_MFE + 4 * 0x1E) & 0xFF, data[:len(hdr)],
           data[len(hdr):].hex()), (1, hdr, '2fa0'))
    bits, data = mfe_frame(qt, 2, moved, hdr, 8 * len(hdr))
    pcm = data.find(moved[16 * w + 16:16 * w + 32])
    check('MFE: CABAC P frame, one MB moved: three P_Skip and an I_PCM',
          (data[len(hdr):pcm].hex(), data[pcm + 384:].hex()),
          ('45ec40', 'fe80'))
    check('MFE: the reconstruction, the reference of the next frame',
          b64read(qt, DRAM + MFE_REC[0], size), moved)

    hdr = MFE_CAVLC_IDR
    bits, data = mfe_frame(qt, 0, flat, hdr, 225, cabac=False)
    check('MFE: CAVLC IDR frame: its header, then the slice data',
          (qt.readw(I3_MFE + 4 * 0x1E) & 0xFF, data[:len(hdr) - 1],
           data.find(bytes([100] * 256)) > 0), (1, hdr[:-1], True))
    # a YUYV source with a pitch shorter than its row: what lies past the
    # pitch reads as 0, nothing past the model's row buffer
    qt.writew(I3_MFE + 4 * 0x1D, 1)
    bits, data = mfe_frame(qt, 0, flat, hdr, 225, cabac=False, pitch=1,
                           yuyv=True)
    check('MFE: a YUYV pitch of 1 still codes a frame',
          (qt.readw(I3_MFE + 4 * 0x1E) & 0xFF, data[:len(hdr) - 1]),
          (1, hdr[:-1]))
    qt.writew(I3_MFE + 4 * 0xF3, 0)


I3_ISP0 = riu(0x1302, 0)
I3_MIIC1 = 0x1F223200
ISP_W, ISP_H = 320, 240
ISP_AE = 0x02000000                             # MIU addresses
ISP_AWB = 0x02002000
ISP_ML = 0x02004000
ISP_HIS = 0x02006000
ISP_Y = 0x02010000
ISP_C = 0x02020000


def i3_sensor(qt, reg, val):
    """One JX-F22 register over MIIC1 in byte mode (START, 0x80, reg, val)."""
    for r, v in ((0x01, 1), (0x02, 0x80), (0x02, reg), (0x02, val),
                 (0x01, 0x100)):
        qt.writew(I3_MIIC1 + 4 * r, v)


def i3_isp_frames(qt, n):
    qt.step(n * 10 ** 9 // 30 + 1)


def i3_isp_block(qt, base, i, bpb):
    return list(b64read(qt, DRAM + base + i * bpb, bpb))


def test_i3_isp(qt):
    print('msc313e ISP pipeline and statistics, scaler output')
    w, h = ISP_W, ISP_H
    # the sensor: 1x gain, 300 rows of 1125, a 320x240 window
    for reg, val in ((0x00, 0), (0x01, 300 & 0xFF), (0x02, 300 >> 8),
                     (0x22, 0x65), (0x23, 0x04), (0x24, w // 2),
                     (0x25, h), (0x26, 0)):
        i3_sensor(qt, reg, val)
    # ISP on with OBC and WBG1, BGGR, the crop; the app's BT.601 matrix
    # (rows Cr, Y, Cb) with Cb/Cr + 128; both gammas off
    qt.writew(I3_ISP0 + 4 * 0x00, 0x17)
    qt.writew(I3_ISP0 + 4 * 0x11, 2)
    qt.writew(I3_ISP0 + 4 * 0x14, w - 1)
    qt.writew(I3_ISP0 + 4 * 0x15, h - 1)
    for i, v in enumerate((0x200, 0x1E53, 0x1FAD, 0x132, 0x259, 0x75,
                           0x1F53, 0x1EAD, 0x200)):
        qt.writew(I3_ISP0 + 4 * (0x3D + i), v)
    qt.writew(I3_ISP0 + 4 * 0x3C, 0x181)
    qt.writew(riu(0x1303, 0x67), 0x4040)        # black 0x40 (10-bit)
    qt.writew(riu(0x1303, 0x69), 0x4040)
    for reg in range(0x6A, 0x6E):               # OBC gains 1.0
        qt.writew(riu(0x1303, reg), 0x400)
    for reg in (0x48, 0x4A, 0x4C, 0x4E):        # WBG1 gains 1.0
        qt.writew(riu(0x1308, reg), 0x400)
    # 8x8 AE and AWB blocks of 40x30, and where they go
    for reg, v in ((0x01, w // 8 - 1), (0x02, h // 8 - 1), (0x03, 0x0707),
                   (0x51, w // 8 - 1), (0x52, h // 8 - 1), (0x53, 0x0707)):
        qt.writew(riu(0x1305, reg), v)
    # the histogram on the same 8x8 grid: window 0 all of it, window 1
    # the 2x2 blocks from (2, 2)
    for reg, v in ((0x61, w // 8 - 1), (0x62, h // 8 - 1), (0x63, 0x0707),
                   (0x70, 0x0700), (0x71, 0x0700), (0x72, 0x0102),
                   (0x73, 0x0102), (0x60, 0x0C)):
        qt.writew(riu(0x1305, reg), v)
    for reg, a in ((0x06, ISP_AE), (0x10, ISP_AE + 0x1000),
                   (0x08, ISP_AWB), (0x12, ISP_AWB + 0x1000),
                   (0x0A, ISP_HIS), (0x14, ISP_HIS + 0x2F0)):
        qt.writew(riu(0x1305, reg), a >> 4 & 0xFFFF)
        qt.writew(riu(0x1305, reg + 1), a >> 20)
    # SC1 FRM, one 64x48 NV12 buffer
    dma = 0x1213
    for reg, v in ((0x08, ISP_Y >> 3 & 0xFFFF), (0x09, ISP_Y >> 19),
                   (0x10, ISP_C >> 3 & 0xFFFF), (0x11, ISP_C >> 19),
                   (0x18, 0), (0x1A, 64), (0x1B, 48), (0x1E, 0x8),
                   (0x01, 0x8000)):
        qt.writew(riu(dma, reg), v)
    # the R, G, B histograms in front of the luma windows are not the
    # model's to write: mark them and see the mark survive the frames
    b64write(qt, DRAM + ISP_HIS, b'\x5a' * 240)
    qt.writew(I3_ISP0 + 4 * 0x28, 0xBFFF)       # unmask VSYNC: streaming
    i3_isp_frames(qt, 2)

    # block (2, 3), the middle of the logo: its 80 % white under the
    # tungsten light (R 0.82, B 0.40 of G) over the navy, less black 0 by
    # OBC, as 12 bits >> 4
    logo = i3_isp_block(qt, ISP_AE, 26, 4)
    check('ISP AE block: R, G, B, Y of the logo under tungsten', logo,
          [36, 45, 19, 39])
    check('ISP AWB block: R, G, B', i3_isp_block(qt, ISP_AWB, 26, 3),
          [36, 45, 19])
    his = struct.unpack('<256H', b64read(qt, DRAM + ISP_HIS + 240, 512))
    check('ISP histogram: window 0 counts its 64 blocks by Y / 2, the '
          'logo block among them; window 1 its 4',
          (sum(his[:128]), his[39 >> 1] >= 1, sum(his[128:])), (64, True, 4))
    check('ISP histogram: the R, G, B parts before the windows untouched',
          b64read(qt, DRAM + ISP_HIS, 240), b'\x5a' * 240)
    i3_sensor(qt, 0x01, 600 & 0xFF)             # twice the exposure
    i3_sensor(qt, 0x02, 600 >> 8)
    i3_isp_frames(qt, 1)
    check('ISP AE block: twice the exposure, twice the statistics',
          i3_isp_block(qt, ISP_AE, 26, 4), [73, 91, 39, 79])
    i3_sensor(qt, 0x00, 0x10)                   # and 2x gain: B doubles,
    i3_isp_frames(qt, 1)                        # R and G clip in the white
    check('ISP AE block: and 2x analog gain',
          i3_isp_block(qt, ISP_AE, 26, 4)[2], 79)
    i3_sensor(qt, 0x00, 0)
    i3_sensor(qt, 0x01, 300 & 0xFF)
    i3_sensor(qt, 0x02, 300 >> 8)
    i3_isp_frames(qt, 1)

    def nv12(x, y):
        yy = b64read(qt, DRAM + ISP_Y + y * 64 + x, 1)[0]
        c = b64read(qt, DRAM + ISP_C + y // 2 * 64 + (x & ~1), 2)
        return [yy, c[0], c[1]]

    # a stroke of the logo, linear (no gamma): yellow-green before WB
    check('scaler: the ISP picture, the logo yellow-green before WB',
          nv12(17, 19), [90, 101, 124])
    qt.writew(riu(0x1308, 0x48), 1249)          # WBG1 R 1/0.82
    qt.writew(riu(0x1308, 0x4C), 2560)          # WBG1 B 1/0.40
    i3_isp_frames(qt, 1)
    check('scaler: WB gains make it neutral', nv12(17, 19), [102, 128, 128])
    check('ISP statistics stay before WB', i3_isp_block(qt, ISP_AE, 26, 4),
          logo)

    # a 10-to-10 gamma through the menuload (SRAMs 10-12): all to 512
    qt.cmd('b64write 0x%x 0x200 %s' % (DRAM + ISP_ML, base64.b64encode(
        struct.pack('<256H', *[512] * 256)).decode()))
    for sram in (10, 11, 12):
        for reg, v in ((0x00, 0x9100 | sram), (0x03, ISP_ML >> 4 & 0xFFFF),
                       (0x04, ISP_ML >> 20), (0x01, 255), (0x02, 0),
                       (0x07, 1)):
            qt.writew(riu(0x1306, reg), v)
    qt.writew(riu(0x1306, 0x07), 2)
    qt.writew(I3_ISP0 + 4 * 0x46, 0xFFF)        # on, the top at 0x3FF
    i3_isp_frames(qt, 1)
    check('ISP gamma from the menuload: a flat curve, a flat picture',
          [nv12(4, 44), nv12(36, 20)], [[128, 128, 128]] * 2)
    qt.writew(I3_ISP0 + 4 * 0x46, 0)
    qt.writew(I3_ISP0 + 4 * 0x00, 0)            # ISP off: the test pattern
    i3_isp_frames(qt, 1)
    check('scaler: the test pattern with the ISP off: navy, the logo',
          [nv12(0, 0), nv12(14, 18)[0]], [[50, 146, 122], 235])
    qt.writew(I3_ISP0 + 4 * 0x28, 0xFFFF)
    qt.writew(riu(dma, 0x01), 0)


def hmp(qemu, args, cmds):
    """Run QEMU with an HMP monitor on stdio; return what it printed."""
    p = subprocess.run([qemu] + args + ['-display', 'none', '-nodefaults',
                                        '-monitor', 'stdio'],
                       input='\n'.join(cmds + ['quit']) + '\n',
                       capture_output=True, text=True, timeout=60)
    return p.stdout + p.stderr


def test_migration(qemu, tmp, path):
    print('migration')
    mig = os.path.join(tmp, 'ssr621q.mig')
    out = hmp(qemu, ['-M', 'ssr621q', '-S'],
              ['migrate file:' + mig, 'info migrate'])
    check('state saved', 'completed' in out and 'Assertion' not in out and
          os.path.getsize(mig) > 0, True)
    out = hmp(qemu, ['-M', 'ssr621q', '-S', '-incoming', 'file:' + mig],
              ['info status', 'xp /1hx 0x%x' % riu(0x1038, 0x33)])
    check('and loaded back, RIU state included',
          'error' not in out.lower() and '0x1101' in out, True)
    out = hmp(qemu, ['-M', 'ssr621q,flash-file=' + path, '-S'],
              ['migrate file:' + os.path.join(tmp, 'blocked.mig')])
    check('file-backed flash blocks migration', 'written through' in out,
          True)


def sxz_rc_literals(data, pb=2):
    """LZMA2 chunk of literals as the vendor's sz codes them: direct bits."""
    low, rng, cache, cache_size, out = 0, 0xFFFFFFFF, 0, 1, bytearray()
    prob = [1024] * (12 * 16)

    def shift():
        nonlocal low, cache, cache_size
        if low < 0xFF000000 or low >= 1 << 32:
            carry = low >> 32
            byte = cache
            while cache_size:
                out.append((byte + carry) & 0xFF)
                byte = 0xFF
                cache_size -= 1
            cache = (low >> 24) & 0xFF
        cache_size += 1
        low = (low << 8) & 0xFFFFFFFF

    def norm():
        nonlocal rng
        while rng < 1 << 24:
            rng = (rng << 8) & 0xFFFFFFFF
            shift()

    for pos, byte in enumerate(data):
        i = pos & ((1 << pb) - 1)           # state 0: is_match is 0
        bound = (rng >> 11) * prob[i]
        rng = bound
        prob[i] += (2048 - prob[i]) >> 5
        norm()
        for bit in range(7, -1, -1):
            rng >>= 1
            if (byte >> bit) & 1:
                low += rng
            norm()
    for _ in range(5):
        shift()
    packed = bytes(out)
    un, pk = len(data) - 1, len(packed) - 1
    return bytes([0xE0 | (un >> 16), (un >> 8) & 0xFF, un & 0xFF,
                  pk >> 8, pk & 0xFF, 0x5D]) + packed + b'\0'


def sxz_vli(v):
    out = bytearray()
    while v >= 0x80:
        out.append((v & 0x7F) | 0x80)
        v >>= 7
    out.append(v)
    return bytes(out)


def sxz_block(data, stored):
    """An XZ block: header, LZMA2, padding, CRC32 of the unpacked data."""
    if stored:
        lz = bytes([1, (len(data) - 1) >> 8, (len(data) - 1) & 0xFF]) + data + b'\0'
    else:
        lz = sxz_rc_literals(data)
    hdr = b'\xC0' + sxz_vli(len(lz)) + sxz_vli(len(data)) + b'\x21\x01\x00'
    hdr = bytes([4]) + hdr
    hdr = hdr.ljust(16, b'\0')
    hdr += struct.pack('<I', zlib.crc32(hdr))
    return hdr + lz + bytes(-len(lz) & 3) + struct.pack('<I', zlib.crc32(data))


def sxz_file(blocks):
    """The vendor's multi-core file: the blocks round-robin in 0x20 chunks."""
    head = b'sstarXZ' + bytes([len(blocks), 0x20])
    for packed, data in blocks:
        head += sxz_vli(len(packed)) + sxz_vli(len(data))
    head = head.ljust((len(head) + 15) & ~15, b'\0')
    body, off, more = bytearray(), 0, True
    while more:
        more = False
        for packed, _ in blocks:
            if off < len(packed):
                body += packed[off:off + 0x20].ljust(0x20, b'\0')
                more = True
        off += 0x20
    return head + bytes(body) + b'SZ'


def test_ssc029_rtcpwc(qemu, tmp):
    print('ssc029: RTC, the alarm, power down and wake on it')
    path = os.path.join(tmp, 'iford-rtc.bin')
    img = b'\x06\x00\x00\xEAIPL_' + struct.pack('<I', 0x200)
    open(path, 'wb').write(img.ljust(0x1000000, b'\xFF'))
    qt = QTest(qemu, 'ssc029,flash-file=' + path, ['-rtc', 'clock=vm'])
    pwc = riu(0x0034, 0)
    iso = lambda: (qt.writew(pwc + 0x30 * 4, 1), qt.writew(pwc + 0x32 * 4, 1))

    def write32(flag, val):
        qt.writew(pwc + 0x04 * 4, val & 0xFFFF)
        qt.writew(pwc + 0x05 * 4, val >> 16)
        qt.writew(pwc + 0x00 * 4, flag)
        iso()
        qt.writew(pwc + 0x00 * 4, 0)

    def count():
        qt.writew(pwc + 0x0E * 4, 1)
        return qt.readw(pwc + 0x0C * 4) | qt.readw(pwc + 0x0D * 4) << 16

    def alarm_int():
        return qt.readw(pwc + 0x08 * 4) >> 1 & 1

    try:
        check('RTC: its 32 kHz clock', qt.readw(pwc + 0x13 * 4) & 2, 2)
        qt.writew(pwc + 0x30 * 4, 1)
        check('RTC: ISO acks', qt.readw(pwc + 0x31 * 4) & 1, 1)
        qt.writew(pwc + 0x32 * 4, 1)
        check('RTC: and clears', qt.readw(pwc + 0x31 * 4) & 1, 0)
        write32(1 << 1, 0x12345678)
        qt.writew(pwc + 0x00 * 4, 1 << 2)
        iso()
        qt.writew(pwc + 0x00 * 4, 0)
        check('RTC: the base reads back',
              qt.readw(pwc + 0x0A * 4) << 16 | qt.readw(pwc + 0x09 * 4),
              0x12345678)
        write32(1 << 5, 0xBABE)
        qt.writew(pwc + 0x00 * 4, 1 << 7)
        iso()
        qt.writew(pwc + 0x00 * 4, 0)
        check('RTC: SW0 reads back', qt.readw(pwc + 0x09 * 4), 0xBABE)
        write32(1 << 3, 0)
        qt.step(3 * 10 ** 9)
        check('RTC: the counter counts seconds', count(), 3)

        qt.writew(riu(0x1009, 0x54), 0xFEFF)        # MS_IRQ 8: SPI 40
        qt.writew(pwc + 0x01 * 4, 1 << 2)
        write32(1 << 4, 5)
        check('RTC: alarm enabled', qt.readw(pwc + 0x08 * 4) & 1, 1)
        qt.step(10 ** 9)
        check('RTC: not yet', alarm_int(), 0)
        qt.step(10 ** 9)
        check('RTC: the alarm at second 5', alarm_int(), 1)
        check('RTC: on MS_IRQ 8', qt.readw(riu(0x1009, 0x5C)) >> 8 & 1, 1)
        qt.writew(pwc + 0x01 * 4, (1 << 2) | (1 << 3))
        iso()
        qt.writew(pwc + 0x01 * 4, 1 << 2)
        check('RTC: the alarm clears', alarm_int(), 0)

    finally:
        qt.close()

    # Power down pauses the guest's clock, so this runs on the host's.
    qt = QTest(qemu, 'ssc029,flash-file=' + path, [])
    try:
        write32(1 << 1, 0x12345678)
        write32(1 << 3, 0)
        qt.writew(pwc + 0x01 * 4, 1 << 2)
        write32(1 << 4, 2)
        # sram_suspend_imi's last steps
        qt.writew(riu(0x0600, 1), 8)
        qt.writew(riu(0x1659, 0x0D), 0x5A)
        qt.writeb(pwc + 0x0F * 4, 0)
        qt.writew(pwc + 0x32 * 4, 1)
        check('power down: not awake before its alarm',
              qt.readw(riu(0x1659, 0x0D)) & 0xFF, 0x5A)
        time.sleep(3)
        check('power down: the alarm reset the chip',
              qt.readw(riu(0x1659, 0x0D)) & 0xFF != 0x5A, True)
        check('power down: the warm-reset flag survives',
              qt.readw(riu(0x0600, 1)) & 8, 8)
        check('power down: the alarm is pending', alarm_int(), 1)
        qt.writew(pwc + 0x00 * 4, 1 << 2)
        iso()
        qt.writew(pwc + 0x00 * 4, 0)
        check('power down: the RTC domain keeps its base',
              qt.readw(pwc + 0x0A * 4) << 16 | qt.readw(pwc + 0x09 * 4),
              0x12345678)
    finally:
        qt.close()


# CPU0 sets CPU1's entry in smpctrl and wakes it Non-secure; CPU1 stores
# 0x5A000000 | MPIDR.Aff0 << 8 | its mode at 0x20100000, then both idle.
IFORD_SMP_IPL = bytes.fromhex(
    '060000ea49504c5f000200000000000000000000000000000000000000000000'
    '84009fe584109fe52128a0e14c2080e50128a0e12228a0e1502080e50c20a0e3'
    '542080e503f020e3fdffffea0000000000000000000000000000000000000000'
    '0000000000000000000000000000000000000000000000000000000000000000'
    '2c009fe500100fe11f1001e2b02f10ee032002e2021481e15a24a0e3021081e1'
    '001080e503f020e3fdffffea0040201f800000a000001020')


def test_ssc029_power_off(qemu, tmp):
    print('ssc029: power off, with and without an alarm to come')
    path = os.path.join(tmp, 'iford-off.bin')
    img = b'\x06\x00\x00\xEAIPL_' + struct.pack('<I', 0x200)
    open(path, 'wb').write(img.ljust(0x1000000, b'\xFF'))
    pwc = riu(0x0034, 0)

    def write32(qt, flag, val):
        qt.writew(pwc + 0x04 * 4, val & 0xFFFF)
        qt.writew(pwc + 0x05 * 4, val >> 16)
        qt.writew(pwc + 0x00 * 4, flag)
        qt.writew(pwc + 0x30 * 4, 1)
        qt.writew(pwc + 0x32 * 4, 1)
        qt.writew(pwc + 0x00 * 4, 0)

    # hal_rtc_power_off with the alarm enabled, as it is from init, but with
    # no alarm to come: the chip stays off
    qt = QTest(qemu, 'ssc029,flash-file=' + path, [])
    try:
        write32(qt, 1 << 3, 0)
        qt.writew(pwc + 0x01 * 4, 1 << 2)
        write32(qt, 1 << 4, 0)
        time.sleep(1.2)
        qt.writew(pwc + 0x01 * 4, 1 << 2)
        write32(qt, 0, 0)
        qt.writeb(pwc + 0x0F * 4, 2)
        try:
            qt.writew(pwc + 0x32 * 4, 1)
        except OSError:
            pass
        try:
            rc = qt.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            rc = 'still running'
        check('power off: a stale alarm does not wake it', rc, 0)
    finally:
        qt.close()

    # an alarm due between the suspend request and the suspend itself: all
    # of this reaches QEMU at once, so its main loop has not suspended yet.
    # The wake that follows needs the host's clock, which cannot be raced.
    qt = QTest(qemu, 'ssc029,flash-file=' + path, ['-rtc', 'clock=vm'])
    try:
        write32(qt, 1 << 3, 0)
        qt.writew(pwc + 0x01 * 4, 1 << 2)
        write32(qt, 1 << 4, 1)
        qt.f.write('writeb 0x%x 0x0\nwritew 0x%x 0x1\nclock_step %d\n' %
                   (pwc + 0x0F * 4, pwc + 0x32 * 4, 2 * 10 ** 9))
        qt.f.flush()
        for _ in range(3):
            qt.f.readline()
        check('power off: an alarm that races it does not abort QEMU',
              qt.proc.poll() is None, True)
    finally:
        qt.close()


def test_infinity2m_rtcpwc(qemu, tmp):
    print('infinity2m: the RTCPWC, as ms_rtcpwc.c drives it')
    path = os.path.join(tmp, 'i2m-rtc.bin')
    img = b'\x06\x00\x00\xEAIPL_' + struct.pack('<I', 0x200)
    open(path, 'wb').write(img.ljust(0x1000000, b'\xFF'))
    qt = QTest(qemu, 'ssd201,flash-file=' + path,
               ['-rtc', 'clock=vm,base=2021-08-26T00:42:00'])
    pwc = riu(0x0034, 0)
    seq = (0, 1, 3, 7, 5, 1, 0)

    def iso():
        acks = []
        for state in seq:
            qt.writew(pwc + 0x03 * 4, state)
            acks.append(qt.readw(pwc + 0x08 * 4) >> 3 & 1)
        return acks

    def read32(flag):
        qt.writew(pwc + 0x00 * 4, flag)
        iso()
        qt.writew(pwc + 0x00 * 4, 0)
        return qt.readw(pwc + 0x0A * 4) << 16 | qt.readw(pwc + 0x09 * 4)

    def write32(flag, val):
        qt.writew(pwc + 0x00 * 4, flag)
        qt.writew(pwc + 0x04 * 4, val & 0xFFFF)
        qt.writew(pwc + 0x05 * 4, val >> 16)
        iso()
        qt.writew(pwc + 0x00 * 4, 0)

    def count():
        qt.writew(pwc + 0x01 * 4, 1)
        iso()
        qt.writew(pwc + 0x0E * 4, 1)
        v = qt.readw(pwc + 0x0D * 4) << 16 | qt.readw(pwc + 0x0C * 4)
        qt.writew(pwc + 0x01 * 4, 0)
        return v

    try:
        check('RTC: ISO_EN and the 1 kHz clock, no power warning',
              qt.readw(pwc + 0x15 * 4) & 0x21, 0x21)
        check('RTC: each ISO step acks as the driver waits for',
              iso(), [0, 1, 0, 1, 0, 1, 0])
        check('RTC: the battery kept the driver\'s magic in SW0',
              read32(1 << 7), 0xBABE)
        check('RTC: the base is the machine\'s RTC time',
              read32(1 << 2), 1629938520)
        write32(1 << 1, 0x12345678)
        check('RTC: a new base reads back', read32(1 << 2), 0x12345678)
        write32(1 << 5, 0xCAFE)
        check('RTC: SW0 writes back', read32(1 << 7), 0xCAFE)
        write32(1 << 3, 0)
        qt.step(3 * 10 ** 9)
        check('RTC: the counter counts seconds', count(), 3)
        qt.writew(pwc + 0x00 * 4, 1 << 3)
        for state in (0, 1, 3):
            qt.writew(pwc + 0x03 * 4, state)
        qt.writew(pwc + 0x03 * 4, 0)
        qt.writew(pwc + 0x00 * 4, 0)
        check('RTC: a sequence cut short sends nothing', count(), 3)
        for cut, name in (((0, 1, 3, 7, 0), 'after 111'),
                          ((0, 1, 3, 7, 5, 0), 'after 101'),
                          ((0, 1, 3, 7, 1, 0), 'that skips 101')):
            qt.writew(pwc + 0x00 * 4, 1 << 3)
            for state in cut:
                qt.writew(pwc + 0x03 * 4, state)
            qt.writew(pwc + 0x00 * 4, 0)
            check('RTC: a sequence cut ' + name + ' sends nothing',
                  count(), 3)
        qt.writew(pwc + 0x00 * 4, 1 << 3)
        for state in (0, 1, 3, 0):
            qt.writew(pwc + 0x03 * 4, state)
        iso()
        qt.writew(pwc + 0x00 * 4, 0)
        check('RTC: the sequence retried from 000 sends', count(), 0)
    finally:
        qt.close()

    # The counter survives a change of rtc_clock; a saved reading would not
    mig = os.path.join(tmp, 'i2m-rtc.mig')
    out = hmp(qemu, ['-M', 'ssd201', '-machine', 'accel=qtest', '-S', '-rtc',
                     'clock=host,base=2015-01-01T00:00:00'],
              ['migrate file:' + mig, 'info migrate'])
    check('RTC: ssd201 migrates', 'completed' in out, True)
    qt = QTest(qemu, 'ssd201', ['-rtc', 'clock=vm,base=2021-08-26T00:42:00',
                                '-incoming', 'file:' + mig])
    try:
        for _ in range(500):
            if read32(1 << 2) == 1420070400:
                break
            time.sleep(0.01)
        check('RTC: the base came across', read32(1 << 2), 1420070400)
        qt.writew(pwc + 0x0E * 4, 1)
        c = qt.readw(pwc + 0x0D * 4) << 16 | qt.readw(pwc + 0x0C * 4)
        check('RTC: the counter came across', c < 60, True)
    finally:
        qt.close()


def test_ssc029_smp(qemu, tmp):
    print('ssc029: CPU1 wakes through smpctrl')
    path = os.path.join(tmp, 'iford-smp.bin')
    open(path, 'wb').write(IFORD_SMP_IPL.ljust(0x1000000, b'\xFF'))
    p = subprocess.Popen([qemu, '-M', 'ssc029,flash-file=' + path,
                          '-display', 'none', '-nodefaults',
                          '-monitor', 'stdio'], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True)
    time.sleep(2)
    out, _ = p.communicate('xp /1xw 0x20100000\nquit\n', timeout=30)
    m = re.search(r'0000000020100000: (0x[0-9a-f]+)', out)
    check('CPU1 ran from the address in smpctrl, Non-secure SVC or lower',
          m and int(m.group(1), 16) & 0xFFFFFF00, 0x5A000100)


# The same smpctrl wake as Ifliegen's TF-A gives for PSCI CPU_ON, 0x15 = 0x0C;
# CPU1 switches to Monitor mode, which only a Secure core reaches, and stores
# 0x5A000000 | MPIDR.Aff0 << 8 | its mode at 0x20100000.
IFLIEGEN_SMP_IPL = bytes.fromhex(
    '060000ea49504c5f000200000000000000000000000000000000000000000000'
    '24009fe524109fe52128a0e14c2080e50128a0e12228a0e1502080e50c20a0e3'
    '542080e503f020e3fdffffea0040201f800000a0000000000000000000000000'
    '0000000000000000000000000000000000000000000000000000000000000000'
    '160002f120009fe500100fe11f1001e2b02f10ee032002e2021481e15a1481e3'
    '001080e503f020e3fdffffea00001020')


def test_ssc034(qemu, tmp):
    print('ssc034a/b (Ifliegen): GIC-400, its lines, the URDMA TX FIFO, '
          'crypto resets, CPU1')
    path = os.path.join(tmp, 'ifliegen.bin')
    img = b'\x06\x00\x00\xEAIPL_' + struct.pack('<I', 0x200)
    open(path, 'wb').write(img.ljust(0x1000000, b'\xFF'))
    out = os.path.join(tmp, 'ifliegen-uart.out')
    for m, bond in (('ssc034a', 0x09), ('ssc034b', 0x00)):
        qt = QTest(qemu, m + ',flash-file=' + path, ['-serial', 'file:' + out])
        try:
            check(m + ': bond, the IPL DRAM profile of its size',
                  qt.readw(riu(0x101E, 0x48)), bond)
            if m == 'ssc034b':
                continue
            check('IPL in IMI', qt.readl(IMI + 4), 0x5F4C5049)
            check('GIC-400: the distributor at 0x16001000, 224 interrupts',
                  qt.readl(0x16001004) & 0x1F, 6)
            check('and its CPU interface at 0x16002000 (GICC_IIDR)',
                  qt.readl(0x16002000 + 0xFC) & 0xFFF, 0x43B)
            # timer1, INT_IRQ_TIMER1: an MS_IRQ line, SPI 102, intc line 70
            t1 = TIMER + 0x80
            qt.cmd('irq_intercept_out /machine/intc')
            qt.writew(riu(0x1009, 0x54 + 70 // 16), ~(1 << 70 % 16) & 0xFFFF)
            qt.writew(t1 + 0x08, 11999)
            qt.writew(t1 + 0x0C, 0)
            qt.writew(t1 + 0x00, 0x101)
            qt.step(1500000)
            check('timer1 raises MS_IRQ 70', intc_raw(qt, 70), 1)
            check('out of the intc, to GIC SPI 102', 'IRQ raise 70' in qt.irqs,
                  True)
            check('a level: it stays up until HIT is cleared',
                  qt.irqs.count('IRQ raise 70'), 1)
            qt.writew(t1 + 0x04, 1)
            check('and clearing HIT drops it', qt.irqs[-1], 'IRQ lower 70')
            ur = 0x1F220E00
            check('URDMA TX FIFO: both FIFOs empty', qt.readw(ur + 0x1C * 4),
                  0x0011)
            qt.writeb(ur + 0x17 * 4, ord('L'))
            time.sleep(0.2)
            check('a byte written to it goes out', open(out, 'rb').read(),
                  b'L')
            aes = riu(0x1122, 0)
            qt.writew(aes + 0x79 * 4, 0x8000)
            check('AES: software reset shows done in 0x7F bit 4',
                  qt.readw(aes + 0x7F * 4) & 0x10, 0x10)
            qt.writew(aes + 0x28 * 4, 1)
            check('RSA: reset shows done in 0x29 bit 3',
                  qt.readw(aes + 0x29 * 4) & 0xC, 0x8)
        finally:
            qt.close()

    path = os.path.join(tmp, 'ifliegen-smp.bin')
    open(path, 'wb').write(IFLIEGEN_SMP_IPL.ljust(0x1000000, b'\xFF'))
    p = subprocess.Popen([qemu, '-M', 'ssc034a,flash-file=' + path,
                          '-display', 'none', '-nodefaults',
                          '-monitor', 'stdio'], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True)
    time.sleep(2)
    out, _ = p.communicate('xp /1xw 0x20100000\nquit\n', timeout=30)
    m = re.search(r'0000000020100000: (0x[0-9a-f]+)', out)
    check('CPU1 woken with 0x0C starts Secure: it reached Monitor mode',
          m and int(m.group(1), 16), 0x5A000116)


def test_ssc034_sd_venc(qemu, tmp):
    print('ssc034a: SD on both FCIEs, card detect, a write by auto-trigger; '
          'the encoder\'s firmware handshake')
    global SDIO0
    path = os.path.join(tmp, 'ifliegen-sd.bin')
    img = b'\x06\x00\x00\xEAIPL_' + struct.pack('<I', 0x200)
    open(path, 'wb').write(img.ljust(0x1000000, b'\xFF'))
    card = os.path.join(tmp, 'ifliegen-sd.img')
    with open(card, 'wb') as f:
        for b in range(SD_BLOCKS):
            f.write(sd_pattern(b))
    qt = QTest(qemu, 'ssc034a,flash-file=' + path, [])
    try:
        check('SD: no cards -> PAD_PM_SD_CDZ and PAD_SD1_IO6 read high',
              (qt.readw(riu(0x103E, 0x16)) & 1, qt.readw(riu(0x103E, 0x06)) & 1),
              (1, 1))
        ctrl = 0x1F34C800
        qt.writel(ctrl + 0x14, 0)
        check('VENC: the firmware reports itself running (ctrl+0x14)',
              qt.readl(ctrl + 0x14), 2)
        qt.writel(ctrl + 0x18, 1)
        check('VENC: a synchronous command is taken (ctrl+0x18 bit 0)',
              qt.readl(ctrl + 0x18) & 1, 0)
        check('VENC: the wrapper has its buffers in MIU (0x1F34E404)',
              qt.readl(0x1F34E404) & 0x18000000, 0x10000000)

        # One encode command, as mi_venc.ko queues it (sstar-venc.c, cmd-ring)
        ring, src, ctl, bsb = 0x00100000, 0x00200000, 0x00300000, 0x00400000
        e = DRAM + ring
        for off, val in ((0x00, 0), (0x10, 8), (0x4C, 64), (0x58, src),
                         (0x64, 64), (0x70, src + 0x1000), (0xD0, ctl),
                         (0x19C, 2), (0xFC4, 64 | 64 << 16)):
            qt.writel(e + off, val)
        for i in range(0, 0x1800, 4):
            qt.writel(DRAM + src + i, 0x80808080)
        qt.writel(DRAM + ctl + 0x00, bsb)
        qt.writel(DRAM + ctl + 0x08, 0x10000)
        qt.writel(DRAM + ctl + 0x40, 0)
        res = DRAM + ring + 7 * 0x1B40
        qt.writel(res + 0x160, 1)
        qt.writel(ctrl + 0x04, ring)                # the ring, then indexes
        qt.writel(ctrl + 0x04, 1)
        qt.step(6000000)
        check('VENC: the firmware took the command (ctrl+0x08)',
              qt.readl(ctrl + 0x08), 1)
        check('VENC: and says so, message 9 for stream 0 (ctrl+0x10)',
              qt.readl(ctrl + 0x10), 0x90001)
        check('VENC: the result block: done, a picture out, at the ring',
              (qt.readl(res + 0x160), qt.readl(res + 0x08) >> 24,
               qt.readl(res + 0x10)), (2, 1, bsb))
        bits = qt.readl(res + 0x18)
        check('VENC: its size in bits, whole bytes', bits > 0 and bits % 8, 0)
        check('VENC: the write offset moved past it, 16-byte aligned',
              qt.readl(DRAM + ctl + 0x40),
              (bits // 8 + 15) // 16 * 16)
        check('VENC: an H.265 stream, a VPS first',
              qt.readl(DRAM + bsb) | qt.readw(DRAM + bsb + 4) << 32,
              0x0140_01000000)
        # a ring too small for the picture: an error, nothing written
        qt.writel(DRAM + ctl + 0x08, 8)
        qt.writel(DRAM + ctl + 0x40, 0)
        qt.writel(res + 0x160, 1)
        qt.writel(ctrl + 0x04, 2)                   # the same entry again
        e1 = DRAM + ring + 0x1B40
        for off in range(0, 0x1B40, 4):
            qt.writel(e1 + off, qt.readl(e + off))
        qt.step(6000000)
        check('VENC: a picture that does not fit is an error (status 7), '
              'the ring untouched',
              (qt.readl(res + 0x160), qt.readl(res + 0x08) >> 24,
               qt.readl(DRAM + ctl + 0x40)), (7, 0, 0))
    finally:
        qt.close()

    saved = SDIO0
    try:
        for index, bank, cdz in ((0, 0x1413, 0x16), (1, 0x1416, 0x06)):
            SDIO0 = riu(bank, 0)
            r = lambda n: SDIO0 + n * 4
            qt = QTest(qemu, 'ssc034a,flash-file=' + path,
                       ['-rtc', 'clock=vm', '-drive',
                        'if=sd,index=%d,format=raw,file=%s' % (index, card)])
            try:
                check('SD slot %d: a card -> its detect pad reads low' % index,
                      qt.readw(riu(0x103E, cdz)) & 1, 0)
                qt.writew(r(0x3F), 0)
                qt.writew(r(0x3F), 1)
                sd_card_init(qt)
                if index:
                    continue
                for i in range(0, 512, 4):
                    qt.writel(DRAM + 0x200000 + i, 0x5A5A0000 | i)
                sd_job(qt, 1, 512, 0x200000)
                # EN_AUTO_TRIG: the data follows the command in one job
                ev, sts, _ = sd_cmd(qt, 24, 9 * 512, data=0x10 | 0x1000)
                check('SD: a write with CMD_END_AUTO_TRIG_EN ends with '
                      'DATA_END', (ev & 3, sts), (3, 0))
                sd_job(qt, 1, 512, 0x300000)
                sd_cmd(qt, 17, 9 * 512, data=0x08)
                check('SD: and reads back', qt.readl(DRAM + 0x300000 + 0x40),
                      0x5A5A0040)
            finally:
                qt.close()
    finally:
        SDIO0 = saved


# IFLIEGEN_SMP_IPL with 0x15 = 0x04, a Secure wake: on Iford CPU1 leaves the
# ROM through the model's stub, then stores its mode as above.
IFORD_SMP_SECURE_IPL = IFLIEGEN_SMP_IPL.replace(bytes.fromhex('0c20a0e3'),
                                                bytes.fromhex('0420a0e3'))


def test_iford_storage(qemu, tmp):
    print('Iford: SD card detect, DRAM size, the DRAM fill engine, CPU1 '
          'Secure; W25N continuous read; infinity6c FCIE ECC decode')
    path = os.path.join(tmp, 'iford-st.bin')
    img = b'\x06\x00\x00\xEAIPL_' + struct.pack('<I', 0x200)
    open(path, 'wb').write(img.ljust(0x1000000, b'\xFF'))
    card = os.path.join(tmp, 'iford-sd.img')
    open(card, 'wb').write(bytes(SD_BLOCKS * 512))
    for extra, want in (([], 1), (['-drive', 'if=sd,format=raw,file=' + card], 0)):
        qt = QTest(qemu, 'ssc029,flash-file=' + path, extra)
        try:
            check('Iford SD: PAD_SD0_CDZ (0x103E reg 0x14) %s'
                  % ('high, no card' if want else 'low, a card'),
                  qt.readw(riu(0x103E, 0x14)) & 1, want)
        finally:
            qt.close()
    for m, log2 in (('ssc029a', 9), ('ssc029b,bond=0x10', 8), ('ssc034b', 6)):
        qt = QTest(qemu, m + ',flash-file=' + path, [])
        try:
            check(m + ': the MIU size register is its DRAM, log2 MiB',
                  qt.readw(riu(0x1659, 0x0D)) & 0xFF, log2)
        finally:
            qt.close()

    qt = QTest(qemu, 'ssc029,flash-file=' + path, [])
    try:
        bist = riu(0x1668, 0)
        qt.writel(DRAM + 0x10000, 0)
        qt.writel(DRAM + 0x100FC, 0)
        qt.writel(DRAM + 0x10100, 0x11223344)
        qt.writew(bist + 0x52 * 4, 0x18)            # fill
        qt.writew(bist + 0x56 * 4, 0xA55A)
        qt.writew(bist + 0x54 * 4, 16)              # 256 bytes
        qt.writew(bist + 0x55 * 4, 0)
        qt.writew(bist + 0x57 * 4, 64)              # at 64 KiB
        qt.writew(bist + 0x58 * 4, 0)
        qt.writew(bist + 0x50 * 4, 1)
        check('DRAM fill: done in 0x59 [2:0]', qt.readw(bist + 0x59 * 4) & 7, 4)
        check('DRAM fill: the pattern from 64 KiB, 256 bytes',
              (qt.readl(DRAM + 0x10000), qt.readl(DRAM + 0x100FC),
               qt.readl(DRAM + 0x10100)), (0xA55AA55A, 0xA55AA55A, 0x11223344))
        # at 128 MiB on a 128 MiB part the MIU wraps: what the IPL sizes by
        qt.writew(bist + 0x56 * 4, 0x5AA5)
        qt.writew(bist + 0x57 * 4, (128 << 10) & 0xFFFF)
        qt.writew(bist + 0x58 * 4, (128 << 10) >> 16)
        qt.writew(bist + 0x50 * 4, 0)
        qt.writew(bist + 0x50 * 4, 1)
        check('DRAM fill: past the end it lands at 0, where DRAM wraps',
              qt.readl(DRAM + 0x10000) & 0xFFFF, 0xA55A)
        check('and at its start', qt.readl(DRAM), 0x5AA55AA5)
    finally:
        qt.close()

    smp = os.path.join(tmp, 'iford-smp-s.bin')
    open(smp, 'wb').write(IFORD_SMP_SECURE_IPL.ljust(0x1000000, b'\xFF'))
    p = subprocess.Popen([qemu, '-M', 'ssc029,flash-file=' + smp,
                          '-display', 'none', '-nodefaults',
                          '-monitor', 'stdio'], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True)
    time.sleep(2)
    out, _ = p.communicate('xp /1xw 0x20100000\nquit\n', timeout=30)
    m = re.search(r'0000000020100000: (0x[0-9a-f]+)', out)
    check('Iford CPU1 woken Secure (0x15 = 0x04) runs its entry Secure, '
          'through the ROM stub', m and int(m.group(1), 16), 0x5A000116)

    # W25N: with BUF clear a read runs on into the next page
    nand = nand_image(tmp, 'w25n.nand')
    data = bytearray(open(nand, 'rb').read())
    tail = 5 * NAND_BLOCK + 2 * NAND_RAW + 2048 - 16
    data[tail:tail + 16] = b'TAIL-OF-PAGE-TWO'
    open(nand, 'wb').write(data)
    qt = QTest(qemu, 'ssc337,flash-file=' + nand + ',flash-nand-id=0xEFAA21',
               [])
    try:
        check('W25N: BUF (feature 0xB0 bit 3) set at power-up',
              fsp_run(qt, [0x0F, 0xB0], (2, 0, 0), (1, 0, 0))[1][0] & 8, 8)
        nand_page_read(qt, 5 * 64 + 2)
        bdma_run(qt, 0, 0x4035, 2048 - 16, 0x100000, 32)
        check('W25N buffer mode: the page and its spare',
              bytes(qt.readb(DRAM + 0x100000 + i) for i in range(20)),
              b'TAIL-OF-PAGE-TWO\xff\xff\xff\xff')
        fsp_run(qt, [0x1F, 0xB0, 0x00], (3, 0, 0), (0, 0, 0))
        nand_page_read(qt, 5 * 64 + 2)
        bdma_run(qt, 0, 0x4035, 2048 - 16, 0x200000, 32)
        check('W25N continuous read: on into the next page, no spare',
              bytes(qt.readb(DRAM + 0x200000 + i) for i in range(20)),
              b'TAIL-OF-PAGE-TWOPAGE')
    finally:
        qt.close()

    # infinity6c SoC ECC: a decode job copies a sector and its spare
    i6c = os.path.join(tmp, 'i6c.bin')
    open(i6c, 'wb').write(img.ljust(0x1000000, b'\xFF'))
    qt = QTest(qemu, 'ssc377,flash-file=' + i6c, [])
    try:
        nc = lambda n: riu(0x1410, n)
        for i in range(0, 512, 4):
            qt.writel(DRAM + 0x100000 + i, 0xC0DE0000 | i)
        for i in range(0, 16, 4):
            qt.writel(DRAM + 0x101000 + i, 0x5EA50000 | i)
        for reg, v in ((0x78, 0x100000 >> 4), (0x7A, 0x101000 >> 4),
                       (0x64, 0x200000), (0x68, 0x201000)):
            qt.writew(nc(reg), v & 0xFFFF)
            qt.writew(nc(reg + 1), v >> 16)
        qt.writew(nc(0x47), 0)                      # one sector
        qt.writew(nc(0x7C), 0)                      # of 512 bytes
        qt.writew(nc(0x7D), 16)
        qt.writew(nc(0x60), 1)
        qt.writew(nc(0x44), 0x98)                   # SER_DIN, decode
        qt.writew(nc(0x45), 1)
        check('FCIE ECC decode: JOB_END', qt.readw(nc(0x60)) & 1, 1)
        check('FCIE ECC decode: the sector and its spare where asked',
              (qt.readl(DRAM + 0x200000 + 0x1FC), qt.readl(DRAM + 0x20100C)),
              (0xC0DE01FC, 0x5EA5000C))
    finally:
        qt.close()


def test_ssc029(qemu, tmp):
    print('ssc029 (Iford): GICv3, the boot FSP, the DRAM test and the XZ decoder')
    path = os.path.join(tmp, 'iford.bin')
    img = bytearray(b'\xFF' * 0x1000000)
    img[0:12] = b'\x06\x00\x00\xEAIPL_' + struct.pack('<I', 0x200)
    for i in range(0x10, 0x200):
        img[i] = i & 0xFF
    img[0x1234:0x1238] = b'NOR!'
    open(path, 'wb').write(img)
    qt = QTest(qemu, 'ssc029,flash-file=' + path, [])
    try:
        qt.cmd('irq_intercept_out /machine/intc')
        check('IPL in IMI', qt.readl(IMI + 4), 0x5F4C5049)
        check('GICv3: GICD_TYPER names 224 SPIs', qt.readl(0x16000004) & 0x1F, 7)
        check('GICv3: redistributor at the DTS address',
              qt.readl(0x16040008) & 0xFF != 0xFF, True)
        check('boot FSP is at bank 0x100D: done clears',
              qt.readw(riu(0x100D, 0x6E)) & 1, 0)
        # the DMA that overwrote the IPL when IMI addresses wrapped at 64 KiB
        qt.writel(IMI + 0x980, 0xC0FFEE00)
        st = bdma_run(qt, 0, 0x0005, 0x1234, 0x10980, 4, misc=0x2000)
        check('BDMA SPI -> IMI above 64 KiB done', st & 8, 8)
        check('and lands there', qt.readl(IMI + 0x10980), 0x21524F4E)
        check('not 64 KiB below it', qt.readl(IMI + 0x980), 0xC0FFEE00)
        a = riu(0x1668, 0x50)
        check('DRAM test idle', qt.readw(riu(0x1668, 0x59)) & 7, 0)
        qt.writew(a, 1)
        check('DRAM test passes', qt.readw(riu(0x1668, 0x59)) & 7, 4)
        qt.writew(riu(0x1659, 0x00), 0x30)
        check('MIU MMU: reset says its RAMs are up',
              qt.readw(riu(0x1659, 0x00)) & 0x3000, 0x3000)
        check('MIU: 128 MiB', qt.readw(riu(0x1659, 0x0D)) & 0xFF, 7)
        qt.writew(riu(0x0042, 0x3F), 0)
        check('SDIO FCIE reset acknowledged',
              qt.readw(riu(0x0042, 0x3F)) & 0xE, 0xE)

        # MIU -> XZ decoder (port 6): the output's address is in 16-byte units
        def unpack(file, dsts):
            file = file + bytes(-len(file) & 3)
            for i in range(0, len(file), 4):
                qt.writel(DDR + 0x200000 + i,
                          struct.unpack_from('<I', file, i)[0])
            xz = riu(0x11D7, 0)
            qt.writew(xz + 0x01 * 4, 7)
            qt.writew(xz + 0x01 * 4, 0)
            for n, dst in enumerate(dsts):
                out = riu(0x11DC, 0x06 + 0x20 * n)
                qt.writew(out, (dst >> 4) & 0xFFFF)
                qt.writew(out + 4, dst >> 20)
            qt.writew(xz, 3)
            bdma_run(qt, 0, 0x0600, 0x200000, 0, len(file))
            return qt.readw(riu(0x11D7, 0x13)), qt.readw(riu(0x11D7, 0x1B))

        data = [bytes(range(64)) * 3, b'SigmaStar direct literals. ' * 9]
        st, ok = unpack(sxz_file([(sxz_block(data[0], True), data[0]),
                                  (sxz_block(data[1], False), data[1])]),
                        [0x300000, 0x310000])
        check('XZ: two blocks done', st, 2)
        check('XZ: their CRCs passed', ok & 0xF, 0xF)
        check('XZ: stored block in DRAM', qt.readl(DDR + 0x300000 + 64),
              struct.unpack_from('<I', data[0], 64)[0])
        check('XZ: LZMA literals in DRAM', qt.readl(DDR + 0x310000 + 4),
              struct.unpack_from('<I', data[1], 4)[0])

        bad = bytearray(sxz_block(data[1], False))
        bad[-1] ^= 1
        st, ok = unpack(sxz_file([(bytes(bad), data[1])]), [0x320000])
        check('XZ: a bad CRC still ends', st, 1)
        check('XZ: and fails its check', ok, 0)
        check('XZ: and writes nothing', qt.readl(DDR + 0x320000), 0)
        st, ok = unpack(b'not an sxz file'.ljust(0x80, b'x'), [0x320000])
        check('XZ: a stranger is refused', (st, ok), (1, 0))

        # plain .xz, as a stock xz writes it: real literals, control 0x07
        def unpack_plain(file, dst):
            xz = riu(0x11D7, 0)
            qt.writew(xz + 0x02 * 4, len(file) & 0xFFFF)
            qt.writew(xz + 0x03 * 4, len(file) >> 16)
            out = riu(0x11DC, 0x06)
            qt.writew(out, (dst >> 4) & 0xFFFF)
            qt.writew(out + 4, dst >> 20)
            pad = file + bytes(-len(file) & 3)
            for i in range(0, len(pad), 4):
                qt.writel(DDR + 0x200000 + i,
                          struct.unpack_from('<I', pad, i)[0])
            qt.writew(xz, 7)
            bdma_run(qt, 0, 0x0600, 0x200000, 0, len(pad))
            return qt.readw(riu(0x11D7, 0x13)), qt.readw(riu(0x11D7, 0x1B))

        text = (b'plain xz needs its context literals, ' * 40 +
                bytes((i * i) & 0xFF for i in range(256)))
        for name, check_id in (('CRC64', lzma.CHECK_CRC64),
                               ('CRC32', lzma.CHECK_CRC32)):
            st, ok = unpack_plain(lzma.compress(text, check=check_id),
                                  0x340000)
            check('plain XZ %s: done' % name, st, 1)
            check('plain XZ %s: its check passed' % name, ok & 3, 3)
            check('plain XZ %s: the text in DRAM' % name,
                  qt.readl(DDR + 0x340000 + 4),
                  struct.unpack_from('<I', text, 4)[0])
            check('plain XZ %s: its tail in DRAM' % name,
                  qt.readl(DDR + 0x340000 + len(text) - 4),
                  struct.unpack_from('<I', text, len(text) - 4)[0])
            qt.writel(DDR + 0x340000 + 4, 0)
        odd = bytearray(lzma.compress(text))
        odd[len(odd) // 2] ^= 1
        st, ok = unpack_plain(bytes(odd), 0x350000)
        check('plain XZ: damage fails its check', (st, ok & 3), (1, 0))
        good = lzma.compress(text)
        idx = (struct.unpack_from('<I', good, len(good) - 8)[0] + 1) * 4
        cut = len(good) - 12 - idx
        for name, file in (
                ('no index', good[:cut]),
                ('only the index byte', good[:cut + 1]),
                ('a stray byte for a block', good[:cut] + b'\x01'),
                ('no footer', good[:-12])):
            st, ok = unpack_plain(file, 0x350000)
            check('plain XZ: refused, %s' % name, (st, ok & 3), (1, 0))
        for name, at in (('stream header', 8),
                         ('block header', 12 + (good[12] + 1) * 4 - 1),
                         ('index', cut + idx - 1),
                         ('footer', len(good) - 12)):
            bad = bytearray(good)
            bad[at] ^= 1
            st, ok = unpack_plain(bytes(bad), 0x350000)
            check('plain XZ: refused, a bad %s CRC' % name, (st, ok & 3),
                  (1, 0))
        st, ok = unpack_plain(good + bytes(4), 0x340000)
        check('plain XZ: stream padding is fine', (st, ok & 3), (1, 3))

        # the main intc: XZ's line (MS_IRQ 54, GIC SPI 86) stays behind it
        unpack_plain(lzma.compress(text), 0x340000)
        def line(n):
            qt.step(1)
            up = [i for i in qt.irqs if i.split()[-1] == str(n)]
            return bool(up) and up[-1].startswith('IRQ raise')

        check('intc: Iford\'s upper lines come out masked',
              qt.readw(riu(0x1009, 0x5B)), 0xFFFF)
        check('intc: XZ raw high', qt.readw(riu(0x1009, 0x5C + 3)) >> 6 & 1, 1)
        check('intc: masked, nothing out', line(54), False)
        qt.writew(riu(0x1009, 0x54 + 3), 0xFFBF)
        check('intc: unmasked, it leaves for SPI 86', line(54), True)
        qt.writew(riu(0x1009, 0x54 + 3), 0xFFFF)
        check('intc: masked again, it stops', line(54), False)

        # the MIU MMU: 32 KiB pages, VPA region 8 (MIU 0x40000000), PA 0
        mmu = riu(0x1659, 0)
        qt.writew(mmu + 0x01 * 4, 0x0008)
        for vpa, pa in ((0, 0x800), (1, 0x7FF)):    # out of order in DRAM
            qt.writew(mmu + 0x03 * 4, pa)
            qt.writew(mmu + 0x02 * 4, 0x8000 | vpa)
            qt.writew(mmu + 0x0A * 4, 3)
        qt.writew(mmu, 1)
        qt.writel(0x60000010, 0x5AFE0001)
        qt.writel(0x60008010, 0x5AFE0002)
        check('MMU: the CPU window reaches page 0x800',
              qt.readl(DDR + 0x04000010), 0x5AFE0001)
        check('MMU: and the next VPA page 0x7FF, not 0x801',
              qt.readl(DDR + 0x03FF8010), 0x5AFE0002)
        check('MMU: reads back through the window',
              qt.readl(0x60008010), 0x5AFE0002)

        # the scaler's frame reaches that MMU memory, in order, across pages
        scl = riu(0x1253, 0)
        qt.writew(scl + 0x02 * 4, 0x50)             # 1280
        qt.writew(scl + 0x03 * 4, 16)
        qt.writew(scl + 0x05 * 4, 0x500)
        qt.writew(scl + 0x06 * 4, 0)                # Y: MIU 0x40000000
        qt.writew(scl + 0x07 * 4, 0x400)
        qt.writew(scl + (0x20 + 0x06) * 4, 0x0200)  # C: 8 KiB on
        qt.writew(scl + (0x20 + 0x07) * 4, 0x400)
        qt.writew(scl, 0x10)
        check('SCL: a frame in the Y buffer, not blank',
              qt.readl(0x60000000) != 0, True)
        check('SCL: chroma is the scene\'s, not zero',
              qt.readb(0x60002000) != 0, True)

        # The image pipe: sensor, statistics, white balance, the scaler.  The
        # ISP's frame clock, group 0 bit 12, with the stats grids the driver
        # sets; the sensor's exposure over I2C, as the vendor's AE writes it.
        qt.writew(mmu, 0)
        st = riu(0x131B, 0)
        ae, awb, histo = 0x500000, 0x501000, 0x509700
        for reg, base in ((0x08, ae), (0x0A, awb), (0x0C, histo)):
            qt.writew(st + reg * 4, (base >> 4) & 0xFFFF)
            qt.writew(st + (reg + 1) * 4, base >> 20)
        for reg, val in ((0x30, 79), (0x31, 44), (0x32, 31), (0x33, 31),
                         (0x40, 19), (0x41, 15), (0x42, 127), (0x43, 89),
                         (0x02, 0x1111)):
            qt.writew(st + reg * 4, val)

        def sensor(page, reg, val):
            i2c = 0x1F222A00
            for r, v in ((0xFD, page), (reg, val)):
                qt.writew(i2c + 0x00 * 4, 0)
                qt.writeb(i2c + 0x01 * 4, 1)
                for b in (0x3C << 1, r, v):
                    qt.writew(i2c + 0x02 * 4, b)
                qt.writeb(i2c + 0x01 * 4 + 1, 1)

        def frame():
            qt.step(50000000)

        def means():
            d = [qt.readb(DDR + ae + 4 * i + 3) for i in range(32 * 32)]
            return sum(d) / len(d), max(d)

        def expose(lines):
            sensor(1, 0x03, lines >> 8)
            sensor(1, 0x04, lines & 0xFF)
            frame()

        qt.writew(riu(0x1302, 0x28), 0xEFFF)
        expose(50)
        low, _ = means()
        check('ISP: AE statistics reach DRAM', low > 0, True)
        expose(100)
        high, peak = means()
        check('ISP: twice the rows, twice the mean Y (%.1f, %.1f)' %
              (low, high), 1.7 < high / low < 2.3, True)
        check('ISP: the logo is brighter than the mean', peak > high * 2, True)
        expose(1500)
        rgb = [qt.readb(DDR + awb + 3 * (128 * 80 + 2) + c) for c in range(3)]
        check('ISP: AWB at the wedge\'s band, the light\'s colour %r' % rgb,
              rgb[1] > rgb[2] > rgb[0] > 0, True)
        bins = [qt.readw(DDR + histo + 2 * i) for i in range(128)]
        check('ISP: the histogram counts AWB\'s 128 x 90 blocks', sum(bins),
              128 * 90)
        check('ISP: histogram 1 likewise',
              sum(qt.readw(DDR + histo + 0x200 + 2 * i) for i in range(128)),
              128 * 90)

        def chroma():
            """U, V of the band at the foot of the picture, through SCL"""
            qt.writew(scl + 0x03 * 4, 720)
            for n, a in ((0, 0x600000), (0x20, 0x700000)):
                qt.writew(scl + (n + 0x06) * 4, (a >> 4) & 0xFFFF)
                qt.writew(scl + (n + 0x07) * 4, a >> 20)
            frame()
            qt.writew(scl, 0x10)
            o = DDR + 0x700000 + 330 * 0x500 + 8
            return qt.readb(o), qt.readb(o + 1)

        wbg = riu(0x1312, 0)
        qt.writew(wbg + 0x40 * 4, 0)
        u, v = chroma()
        check('ISP: gains off, the band keeps the light\'s tint (%d, %d)' %
              (u, v), v < 124 or u > 132, True)
        qt.writew(wbg + 0x41 * 4, 2089)
        qt.writew(wbg + 0x44 * 4, 1024)
        qt.writew(wbg + 0x47 * 4, 1024)
        qt.writew(wbg + 0x4A * 4, 1765)
        qt.writew(wbg + 0x40 * 4, 1)
        u, v = chroma()
        check('ISP: WBG takes the 5000 K tint out (%d, %d)' % (u, v),
              abs(u - 128) <= 2 and abs(v - 128) <= 2, True)

        # VIF's write DMA: a queued buffer takes the frame
        wd, di = riu(0x1358, 0), riu(0x130C, 0)
        qt.writew(wd + 0x04 * 4, 3200 // 16)
        qt.writew(wd, 1)
        qt.writew(wd + 0x06 * 4, (0x800000 >> 4) & 0xFFFF)
        qt.writew(wd + 0x07 * 4, 0x800000 >> 20)
        qt.writeb(wd + 0x0C * 4 + 1, 1)
        qt.writew(wd, 0x11)
        check('WDMA: the buffer in the FIFO', qt.readw(wd + 0x0E * 4) & 7, 1)
        qt.writew(di + 0x03 * 0, 0xFFFE)
        qt.step(50000000)
        check('WDMA: the frame\'s end raised', qt.readw(di + 0x03 * 4) & 1, 1)
        check('WDMA: and its start', qt.readw(di + 0x17 * 4) & 1, 1)
        check('WDMA: the FIFO emptied', qt.readw(wd + 0x0E * 4) & 7, 0)
        qt.writeb(wd + 0x0C * 4 + 1, 0)
        check('WDMA: reports the buffer done',
              qt.readw(wd + 0x0E * 4) | qt.readw(wd + 0x0F * 4) << 16,
              0x800000 >> 4)
        check('WDMA: Bayer data at the row of the logo',
              any(qt.readl(DDR + 0x800000 + 700 * 3200 + x) for x in
                  range(800, 2400, 80)), True)
        qt.writew(di + 0x02 * 4, 1)
        check('WDMA: the status clears', qt.readw(di + 0x03 * 4) & 1, 0)

        # the IPU's supply and space, the RNG
        chip = riu(0x101E, 0x6C)
        qt.writew(chip, 1)
        check('IPU: the supply reads up', qt.readw(chip) & 0x100, 0x100)
        qt.writew(chip, 0x11)
        check('IPU: power off takes it down', qt.readw(chip) & 0x100, 0)
        qt.writew(chip, 0)
        qt.writel(0x19000000 + 0x1360000 - 0x1000000 + 0xA4, 0x4000)
        check('IPU: its registers hold (storage: no block behind them)',
              qt.readl(0x19000000 + 0x1360000 - 0x1000000 + 0xA4), 0x4000)
        check('RNG: ready', qt.readw(riu(0x112F, 0x23)) >> 15, 1)
        a = [qt.readw(riu(0x112F, 0x22)) for i in range(8)]
        check('RNG: its words change', len(set(a)) > 1, True)
    finally:
        qt.close()


def i6_image(tmp, name):
    path = os.path.join(tmp, name)
    img = bytearray(b'\xFF' * 0x1000000)
    img[0:12] = b'\x06\x00\x00\xEAIPL_' + struct.pack('<I', 0x200)
    img[0x1234:0x1238] = b'NOR!'
    open(path, 'wb').write(img)
    return path


def test_i6e_media(qemu, tmp):
    print('infinity6e media: VIF, ISP, the Wave5 encoder, SCL interrupts')
    path = i6_image(tmp, 'i6m.bin')
    wave = 0x1F345200
    bs = 0x01000000     # MIU; DRAM holds it at +0x20000000
    src = 0x01100000
    qt = QTest(qemu, 'ssc338q,flash-file=' + path, [])
    try:
        # MIIC1 DMA: a NACK is reported, as HAL_HWI2C_DMA_WaitDone reads it
        i2c = 0x1F223200
        def dma_read(addr):
            qt.writew(i2c + 0x2A * 4, 1)
            qt.writew(i2c + 0x29 * 4, 0)
            qt.writew(i2c + 0x23 * 4, 1 << 6)
            qt.writew(i2c + 0x21 * 4, 0)
            qt.writew(i2c + 0x22 * 4, 0x0120)
            qt.writew(i2c + 0x2E * 4, addr)
            qt.writew(i2c + 0x2F * 4, 1)
            qt.writew(i2c + 0x24 * 4, 1)
            return (qt.readw(i2c + 0x02 * 4) >> 8) & 1
        check('MIIC DMA: nobody at 0x40 is a NACK', dma_read(0x40), 1)
        check('MIIC DMA: the imx415 at 0x1A answers', dma_read(0x1A), 0)
        # VIF: unmasked but not enabled, no frames
        vif = riu(0x1304, 0)
        qt.writew(vif + 0x0B * 4, 0xFFDE)
        qt.step(100 * 1000000)
        check('VIF: no frames before the channel is enabled',
              qt.readw(vif + 0x0E * 4), 0)
        qt.writew(vif, 0x8000)
        qt.step(50 * 1000000)
        check('VIF: frame start and frame end once enabled',
              qt.readw(vif + 0x0E * 4) & 0x21, 0x21)
        qt.writew(vif + 0x0D * 4, 0x21)
        check('VIF: status clears', qt.readw(vif + 0x0E * 4) & 0x21, 0)
        qt.writew(vif, 0)
        # ISP: five groups, so reg 0x40 is not a mask reset to all ones
        isp = riu(0x1302, 0)
        check('ISP: group 1 masked from reset', qt.readw(isp + 0x28 * 4), 0xFFFF)
        check('ISP: reg 0x40 (WDMA status, polled for bit 9) idle',
              qt.readw(isp + 0x40 * 4) & 0x200, 0)
        qt.writew(isp + 0x28 * 4, 0)
        qt.step(100 * 1000000)
        check('ISP: no frames before ISP_EnableISP', qt.readw(isp + 0x2B * 4), 0)
        qt.writew(isp, 1)
        qt.step(50 * 1000000)
        check('ISP: frames once enabled',
              (qt.readw(isp + 0x2B * 4) >> 12) & 1, 1)
        qt.writew(isp, 0)
        # Wave5: boot
        check('Wave5: not running before INIT_VPU', qt.readl(wave + 0x04), 0)
        qt.writel(wave + 0x70, 1)
        qt.writel(wave + 0x100, 1)
        qt.writel(wave + 0x6C, 1)
        check('Wave5: INIT_VPU ends busy', qt.readl(wave + 0x70), 0)
        check('Wave5: INIT_VPU succeeded', qt.readl(wave + 0x108), 1)
        check('Wave5: running', qt.readl(wave + 0x04) != 0, True)
        check('Wave5: FIO ready', qt.readl(wave + 0x20) >> 31, 1)

        def cmd(c, opt=0, inst=0):
            qt.writel(wave + 0x104, opt)
            qt.writel(wave + 0x110, inst)
            qt.writel(wave + 0x70, 1)
            qt.writel(wave + 0x100, c)
            qt.writel(wave + 0x38, 1)

        cmd(0x0008)
        qt.writel(wave + 0x11C, 64 << 16 | 64)
        qt.writel(wave + 0x130, 2 << 16)
        cmd(0x0200)
        cmd(0x4000, 0x40)
        check('Wave5: sequence query succeeds', qt.readl(wave + 0x1DC), 1)
        check('Wave5: one framebuffer asked for', qt.readl(wave + 0x11C), 1)
        # headers
        qt.writel(wave + 0x118, bs)
        qt.writel(wave + 0x11C, 0x10000)
        cmd(0x0100, 0x10)
        qt.step(10 * 1000000)
        cmd(0x4000, 0x02)
        n = qt.readl(wave + 0x14C)
        check('Wave5: headers written', n > 0, True)
        head = [qt.readl(0x20000000 + bs + i) for i in range(0, 8, 4)]
        check('Wave5: headers start with a VPS',
              [head[0], head[1] & 0x7E], [0x01000000, 32 << 1])
        # pictures: a source, IDR then P with a GOP of 2
        for i in range(0, 64 * 96, 4):
            qt.writel(0x20000000 + src + i, 0x80808080 if i >= 64 * 64
                      else 0x20406080)
        sizes = []
        for f in range(3):
            qt.writel(wave + 0x118, bs)
            qt.writel(wave + 0x11C, 0x10000)
            qt.writel(wave + 0x148, src)
            qt.writel(wave + 0x14C, src + 64 * 64)
            qt.writel(wave + 0x154, 64 << 16 | 64)
            qt.writel(wave + 0x1E8, 0)
            qt.writel(wave + 0x34, 0xFFFFFFFF)
            qt.writel(wave + 0x48, 0x100)
            cmd(0x0100, 0x02)
            check('Wave5: picture %d busy until done' % f,
                  qt.readl(wave + 0x70), 1)
            qt.step(10 * 1000000)
            check('Wave5: picture %d done' % f, qt.readl(wave + 0x70), 0)
            check('Wave5: picture %d in the done-instance bits' % f,
                  qt.readl(wave + 0x1E8), 1)
            check('Wave5: picture %d raises the encoder bit' % f,
                  qt.readl(wave + 0x4C) & 0x100, 0x100)
            cmd(0x4000, 0x02)
            check('Wave5: picture %d type' % f, qt.readl(wave + 0x124),
                  [0, 1, 0][f])
            check('Wave5: no stale warning', qt.readl(wave + 0x1D4), 0)
            sizes.append(qt.readl(wave + 0x14C))
            w = qt.readl(0x20000000 + bs)
            b = qt.readl(0x20000000 + bs + 4) & 0xFF
            check('Wave5: picture %d starts with %s' % (f, ['a VPS', 'a slice',
                                                            'a VPS'][f]),
                  [w, (b >> 1) & 0x3F], [0x01000000, [32, 1, 32][f]])
        check('Wave5: a P picture of the same source is smaller',
              sizes[1] < sizes[0], True)
        # strides narrower than the picture, and a stream buffer of 4 GiB
        qt.writel(wave + 0x118, bs)
        qt.writel(wave + 0x11C, 0xFFFFFFFF)
        qt.writel(wave + 0x148, src)
        qt.writel(wave + 0x14C, src + 64 * 64)
        qt.writel(wave + 0x154, 1 << 16 | 1)
        cmd(0x0100, 0x02)
        qt.step(10 * 1000000)
        check('Wave5: survives them', qt.proc.poll() is None, True)
        cmd(0x4000, 0x02)
        # headers asked for mid-GOP, as mi_venc does, leave the stream alone
        mid = 2
        cmd(0x0008, 0, mid)
        qt.writel(wave + 0x11C, 64 << 16 | 64)
        qt.writel(wave + 0x130, 30 << 16)
        cmd(0x0200, 0, mid)
        pocs = []
        for f in range(4):
            qt.writel(wave + 0x118, bs)
            qt.writel(wave + 0x11C, 0x10000)
            if f == 2:
                cmd(0x0100, 0x10, mid)
                qt.step(10 * 1000000)
                cmd(0x4000, 0x02, mid)
                qt.writel(wave + 0x118, bs)
                qt.writel(wave + 0x11C, 0x10000)
            qt.writel(wave + 0x148, src)
            qt.writel(wave + 0x14C, src + 64 * 64)
            qt.writel(wave + 0x154, 64 << 16 | 64)
            cmd(0x0100, 0x02, mid)
            qt.step(10 * 1000000)
            cmd(0x4000, 0x02, mid)
            pocs.append(qt.readl(wave + 0x128))
        check('Wave5: POC runs on across a header request',
              pocs[3] - pocs[0], 3)
        # H.264: the instance register's high half, as mi_venc writes it
        avc = 3 << 16 | 1
        cmd(0x0008, 0, avc)
        qt.writel(wave + 0x11C, 64 << 16 | 64)
        qt.writel(wave + 0x130, 2 << 16)
        cmd(0x0200, 0, avc)
        qt.writel(wave + 0x118, bs)
        qt.writel(wave + 0x11C, 0x10000)
        cmd(0x0100, 0x10, avc)
        qt.step(10 * 1000000)
        cmd(0x4000, 0x02, avc)
        check('AVC: headers are an SPS', [qt.readl(0x20000000 + bs),
              qt.readb(0x20000000 + bs + 4)], [0x01000000, 0x67])
        check('AVC: and no VPS', qt.readb(0x20000000 + bs + 4) >> 1 & 0x3F != 32,
              True)
        nal = []
        for f in range(2):
            qt.writel(wave + 0x118, bs)
            qt.writel(wave + 0x11C, 0x10000)
            qt.writel(wave + 0x148, src)
            qt.writel(wave + 0x14C, src + 64 * 64)
            qt.writel(wave + 0x154, 64 << 16 | 64)
            cmd(0x0100, 0x02, avc)
            qt.step(10 * 1000000)
            cmd(0x4000, 0x02, avc)
            nal.append(qt.readb(0x20000000 + bs + 4))
        check('AVC: the IDR brings its SPS, the next is a slice', nal,
              [0x67, 0x41])
        # SCL interrupts
        sc = riu(0x1218, 0)
        check('SCL: masked from reset', qt.readw(sc + 0x08 * 4), 0xFFFF)
    finally:
        qt.close()


def test_sd2(qemu, tmp):
    print('second SD hosts: their bank, their own card')
    card = os.path.join(tmp, 'card2.img')
    with open(card, 'wb') as f:
        f.truncate(16 * 1024 * 1024)
    extra = ['-drive', 'if=sd,index=1,format=raw,file=' + card]
    path = i6_image(tmp, 'i6sd.bin')
    for machine in ('ssc325', 'ssc337', 'ssc338q', 'ssc377'):
        qt = QTest(qemu, machine + ',flash-file=' + path, extra)
        try:
            check(machine + ': a host at bank 0x1410 answers its reset',
                  (qt.writew(riu(0x1410, 0x3F), 0) or
                   qt.readw(riu(0x1410, 0x3F))) & 0xE, 0xE)
        finally:
            qt.close()


def test_i6_media(qemu, tmp):
    print('infinity6 media: the wiring OpenIPC\'s mhal.ko expects')
    path = i6_image(tmp, 'i6m.bin')
    qt = QTest(qemu, 'ssc325,flash-file=' + path, [])
    try:
        isp = riu(0x1302, 0)
        for reg in (0x28, 0x56, 0x4A, 0x5B, 0x2D):
            check('ISP: a group masked from reset at reg 0x%x' % reg,
                  qt.readw(isp + reg * 4), 0xFFFF)
        qt.writew(isp + 0x28 * 4, 0)
        qt.writew(isp, 1)
        qt.step(50 * 1000000)
        check('ISP: frame start and process done, DrvIspISR',
              qt.readw(isp + 0x2B * 4) >> 14, 3)
        check('ISP: no ROT ERROR in the fifth group',
              qt.readw(isp + 0x30 * 4), 0)
        vif = riu(0x1319, 0)
        qt.writew(vif, 0x8000)
        qt.writew(vif + 0x0B * 4, 0)
        qt.step(50 * 1000000)
        check('VIF: at bank 0x1319, frame end on bit 1',
              qt.readw(vif + 0x0E * 4) & 3, 3)
        cq = riu(0x1120, 0)
        check('CMDQ: idle, with HAL_CMDQ_Read_Irq_Status\'s error bits clear',
              qt.readw(cq + 0x45 * 4) & 0x1F00, 0x0800)
        check('H.265 is not modelled here: no Wave5 at 0x1F345200',
              qt.readl(0x1F345204), 0)
        mfe = 0x1F264E00
        qt.writew(mfe + 0x1C * 4, 0xFE)
        check('MFE: at the DTS "mfe-0"', qt.readw(mfe + 0x1C * 4) & 0xFF, 0xFE)
    finally:
        qt.close()


def test_i6_jpe(qemu, tmp):
    print('infinity6 family JPE: jpe2\'s registers, its own interrupt mask')
    path = i6_image(tmp, 'i6j.bin')
    jpe = 0x1F2C4000
    qt = QTest(qemu, 'ssc337,flash-file=' + path, [])
    try:
        check('masked from reset', qt.readw(jpe + 0x22 * 4), 0xFF)
        qt.writew(jpe + 0x01 * 4, 64)
        qt.writew(jpe + 0x02 * 4, 64)
        qt.writew(jpe + 0x5E * 4, 0x0404)
        for i in range(0, 64 * 96, 4):
            qt.writel(DRAM + 0x100000 + i, 0x80808080)
        qt.writew(jpe + 0x06 * 4, 0x1000)
        qt.writew(jpe + 0x07 * 4, 0)
        qt.writew(jpe + 0x0A * 4, 0x1100)
        qt.writew(jpe + 0x0B * 4, 0)
        out, end = 0x200000 >> 2, 0x20FFFF >> 2
        for addr, name in ((out, 0), (end, 1)):
            qt.writew(jpe + 0x3C * 4, addr & 0xFFFF)
            qt.writew(jpe + 0x3D * 4, addr >> 16 | name << 14)
            qt.writew(jpe + 0x3E * 4, 1)
        qt.writew(jpe + 0x22 * 4, 0xFE)
        qt.writew(jpe + 0x00 * 4, 3)
        qt.step(100 * 1000000)
        check('frame done, past the mask at 0x22',
              qt.readw(jpe + 0x1E * 4) & 1, 1)
        check('a scan in DRAM', qt.readl(DRAM + 0x200000) != 0, True)
        qt.writew(jpe + 0x1D * 4, 1)
        check('0x1D clears it', qt.readw(jpe + 0x1E * 4) & 1, 0)
        # infinity6e's driver resets just after the start it means
        qt.writew(jpe + 0x00 * 4, 3)
        qt.writew(jpe + 0x06 * 4, 1)
        qt.writew(jpe + 0x00 * 4, 0xF0)
        qt.writew(jpe + 0x00 * 4, 0xF2)
        qt.step(100 * 1000000)
        check('a reset after the start leaves the frame',
              qt.readw(jpe + 0x1E * 4) & 1, 1)
        check('and its source', qt.readw(jpe + 0x06 * 4), 0x1000)
        # a source past DRAM, unmapped: the scaler's pattern, not wrapped DRAM
        def scan(page):
            qt.writew(jpe + 0x06 * 4, page & 0xFFFF)
            qt.writew(jpe + 0x07 * 4, page >> 16)
            qt.writew(jpe + 0x1D * 4, 0xFF)
            qt.writew(jpe + 0x00 * 4, 3)
            qt.step(100 * 1000000)
            return qt.readw(jpe + 0x44 * 4)

        # 128 MiB here: past it reads as nothing, unless it is the pattern
        check('a source past DRAM is the test pattern',
              scan(0x100000) != scan(0x7000), True)
        qt.writew(jpe + 0x06 * 4, 0x1000)
        qt.writew(jpe + 0x07 * 4, 0)
        # infinity6e's driver finishes a picture from a record the engine
        # writes: _IsrFunc() and _EncDone()
        dump = 0x300000
        qt.writew(jpe + 0x52 * 4, (dump >> 4) & 0xFFFF)
        qt.writew(jpe + 0x53 * 4, dump >> 20)
        qt.writew(jpe + 0x1D * 4, 1)
        qt.writew(jpe + 0x00 * 4, 3)
        qt.step(100 * 1000000)
        check('no record while 0x53 bit 15 is off',
              qt.readl(DRAM + dump + 4), 0)
        qt.writew(jpe + 0x53 * 4, dump >> 20 | 0x8000)
        # a size that ends inside a word, so the index is not 0
        for height in range(64, 160, 8):
            qt.writew(jpe + 0x02 * 4, height)
            qt.writel(DRAM + 0x100000 + height, 0x8000FF40 + height)
            qt.writew(jpe + 0x1D * 4, 1)
            qt.writew(jpe + 0x00 * 4, 3)
            qt.step(100 * 1000000)
            if qt.readw(jpe + 0x42 * 4) & 3:
                break
        check('a scan ending inside a word',
              qt.readw(jpe + 0x42 * 4) & 3 != 0, True)
        word = qt.readl(DRAM + dump + 4)
        check('the record names a finished picture', word & 0x1FF, 1)
        check('from the output buffer\'s start',
              qt.readl(DRAM + dump + 8), out)
        wptr = qt.readw(jpe + 0x44 * 4) | qt.readw(jpe + 0x45 * 4) << 16
        check('to the last word', qt.readl(DRAM + dump + 12), wptr)
        check('and the byte in it', word >> 24 & 3,
              qt.readw(jpe + 0x42 * 4) & 3)
        # a full buffer is a picture in progress to the driver
        qt.writel(DRAM + dump + 4, 0)
        for addr, name in ((out, 0), (out + 4, 1)):
            qt.writew(jpe + 0x3C * 4, addr & 0xFFFF)
            qt.writew(jpe + 0x3D * 4, addr >> 16 | name << 14)
            qt.writew(jpe + 0x3E * 4, 1)
        qt.writew(jpe + 0x1D * 4, 0xFF)
        qt.writew(jpe + 0x22 * 4, 0xFC)
        qt.writew(jpe + 0x00 * 4, 3)
        qt.step(100 * 1000000)
        check('buffer full', qt.readw(jpe + 0x1E * 4) & 2, 2)
        check('leaves no record', qt.readl(DRAM + dump + 4), 0)
    finally:
        qt.close()


def test_i6_mask(qemu, tmp):
    print('infinity6 proper: JPE and the scaler\'s first interrupt')
    path = i6_image(tmp, 'i6m.bin')
    jpe = 0x1F264000
    qt = QTest(qemu, 'ssc325,flash-file=' + path, [])
    try:
        check('the JPE mask stays at 0x1C', qt.readw(jpe + 0x1C * 4), 0xFF)
        check('0x22 is free', qt.readw(jpe + 0x22 * 4), 0)
        scl = riu(0x1218, 0)
        # no buffer programmed: the driver waits for this one to set it
        for r in range(3):
            qt.writew(scl + (0x08 + r) * 4, 0)
        isp = riu(0x1302, 0)
        qt.writew(isp + 0x28 * 4, 0)
        qt.writew(isp, 1)
        qt.step(100 * 1000000)
        check('the scaler interrupts with no buffer',
              qt.readw(scl + 0x13 * 4) != 0, True)
    finally:
        qt.close()


def test_i6_mfe(qemu, tmp):
    print('infinity6 MFE: the ven banks, the record, I and P')
    path = i6_image(tmp, 'i6f.bin')
    mfe, ven, out = 0x1F264E00, riu(0x1329, 0), riu(0x132B, 0)
    w, h, src, obuf = 128, 96, 0x100000, 0x200000
    rec_a, rec_b = 0x300000, 0x310000

    def job(qt, rec, intra, start=True):
        for r, v in ((0x11, w), (0x12, h), (0x15, w),
                     (0x20, src & 0xFFFF), (0x21, src >> 16),
                     (0x22, (src + w * h) & 0xFFFF),
                     (0x23, (src + w * h) >> 16)):
            qt.writew(ven + r * 4, v)
        for r, v in ((0x03, obuf & 0xFFFF), (0x04, obuf >> 16),
                     (0x05, 0xFFF0), (0x06, (obuf >> 16))):
            qt.writew(out + r * 4, v)
        qt.writew(mfe + 0x1C * 4, 0)
        qt.writew(mfe + 0x1D * 4, 0xFF)
        qt.writew(mfe + 0x00 * 4, 0xE200)
        qt.writew(mfe + 0x01 * 4, (rec >> 4) & 0xFFFF)
        qt.writew(mfe + 0x02 * 4, rec >> 20)
        qt.writew(mfe + 0x03 * 4, 0x0243)
        qt.writew(mfe + 0x39 * 4, 0x9C if intra else 0x8C)
        qt.writew(ven + 0x10 * 4, 0x0609 if start else 0x0608)
        qt.writew(mfe + 0x47 * 4, 0x10)
        qt.writew(mfe + 0x46 * 4, 0x1234)
        qt.writew(mfe + 0x48 * 4, 0x0800)
        return qt.readw(mfe + 0x42 * 4) | qt.readw(mfe + 0x43 * 4) << 16

    qt = QTest(qemu, 'ssc325,flash-file=' + path, [])
    try:
        x = 1
        # noise in Y only, so that the planes cannot be swapped unseen
        for i in range(0, w * h, 4):
            x = (x * 1103515245 + 12345) & 0xFFFFFFFF
            qt.writel(DRAM + src + i, x)
        for i in range(0, w * h // 2, 4):
            qt.writel(DRAM + src + w * h + i, 0x80808080)
        job(qt, rec_a, True, start=False)
        check('no start bit, no frame', qt.readw(mfe + 0x1E * 4) & 1, 0)
        check('and no record', qt.readl(DRAM + rec_a), 0)
        bits_i = job(qt, rec_a, True)
        check('frame done', qt.readw(mfe + 0x1E * 4) & 1, 1)
        check('the record: the bit count',
              qt.readw(DRAM + rec_a) | qt.readw(DRAM + rec_a + 2) << 16,
              bits_i)
        check('more bits than 16 can hold', bits_i > 0xFFFF, True)
        check('the record: done', qt.readw(DRAM + rec_a + 0xBE) & 1, 1)
        check('the header, at the output buffer in 0x132B',
              qt.readw(DRAM + obuf), 0x3412)
        check('a whole picture of noise, at 0x1329\'s size',
              bits_i > w * h * 8, True)
        qt.writew(mfe + 0x90 * 4, 1)
        check('the pitch is 0x1329\'s, not 0x90\'s',
              job(qt, rec_a, True), bits_i)
        bits_p = job(qt, rec_a, False)
        check('0x39 bit 4 clear: P, skipped against its reference',
              bits_p < bits_i / 8, True)
        bits_other = job(qt, rec_b, False)
        check('another stream has its own reference',
              bits_other > bits_i / 2, True)
        check('its record is its own',
              qt.readw(DRAM + rec_b + 0xBE) & 1, 1)
        qt.writew(mfe + 0x1D * 4, 0xFF)
        qt.writew(mfe + 0x1C * 4, 0x0100)
        check('CMDQ\'s force raises frame done',
              qt.readw(mfe + 0x1E * 4) & 1, 1)
    finally:
        qt.close()


def test_i6_mhe(qemu, tmp):
    print('infinity6 MHE: the packer, the record, I and P')
    path = i6_image(tmp, 'i6h.bin')
    mhe, ven, out = 0x1F264800, riu(0x1329, 0), riu(0x132B, 0)
    w, h, src, obuf = 640, 480, 0x100000, 0x200000
    rec_a, rec_b = 0x300000, 0x310000
    idr = (0x0000, 0x0001, 0x2601, 0xAC10, 0xC000)
    trail = (0x0000, 0x0001, 0x0201, 0xD00C, 0x8218)

    def job(qt, rec, words, qp=30, start=True, rec_on=True):
        for r, v in ((0x13, w - 1), (0x14, h - 1), (0x15, w),
                     (0x20, src & 0xFFFF), (0x21, src >> 16),
                     (0x22, (src + w * h) & 0xFFFF),
                     (0x23, (src + w * h) >> 16)):
            qt.writew(ven + r * 4, v)
        for r, v in ((0x03, obuf & 0xFFFF), (0x04, obuf >> 16),
                     (0x05, 0xFFF0), (0x06, (obuf >> 16))):
            qt.writew(out + r * 4, v)
        qt.writew(mhe + 0x1C * 4, 0)
        qt.writew(mhe + 0x1D * 4, 0xFF)
        qt.writew(mhe + 0x48 * 4, qp)
        qt.writew(mhe + 0x400 + 0x65 * 4, rec & 0xFFFF)
        qt.writew(mhe + 0x400 + 0x66 * 4, rec >> 16)
        qt.writew(mhe + 0x400 + 0x67 * 4, 1 if rec_on else 0)
        qt.writew(ven + 0x10 * 4, 0x0603 if start else 0x0602)
        qt.writew(mhe + 0x4A * 4, (len(words) - 1) << 5 | 8)
        for v in words:
            qt.writew(mhe + 0x49 * 4, v)
            qt.writew(mhe + 0x4B * 4, 0x2F38)
        return (qt.readw(mhe + 0x200 + 0x73 * 4) |
                qt.readw(mhe + 0x200 + 0x74 * 4) << 16)

    qt = QTest(qemu, 'ssc325,flash-file=' + path, [])
    try:
        # hevc-enc codes a block's level, so flat 8x8 blocks of noise
        x = 1
        for by in range(0, h, 8):
            row = []
            for bx in range(0, w, 8):
                x = (x * 1103515245 + 12345) & 0xFFFFFFFF
                row.append(((x >> 16) & 0xFF) * 0x01010101)
            for line in range(8):
                base = DRAM + src + (by + line) * w
                for i, v in enumerate(row):
                    qt.writel(base + i * 8, v)
                    qt.writel(base + i * 8 + 4, v)
        for i in range(0, w * h // 2, 4):
            qt.writel(DRAM + src + w * h + i, 0x80808080)
        job(qt, rec_a, idr, start=False)
        check('no start bit, no frame', qt.readw(mhe + 0x1E * 4) & 1, 0)
        job(qt, rec_a, idr, rec_on=False)
        check('frame done', qt.readw(mhe + 0x1E * 4) & 1, 1)
        check('no record while 0x67 bit 0 is off', qt.readl(DRAM + rec_a + 8),
              0)
        bits_i = job(qt, rec_a, idr)
        check('the record: the bit count',
              qt.readw(DRAM + rec_a + 8) | qt.readw(DRAM + rec_a + 10) << 16,
              bits_i)
        check('more bits than 16 can hold', bits_i > 0xFFFF, True)
        check('the record: done', qt.readw(DRAM + rec_a + 0x10) & 1, 1)
        check('the packer\'s bytes first, in 0x132B\'s buffer',
              qt.readl(DRAM + obuf + 4), 0x10AC0126)
        check('the slice data past them',
              qt.readl(DRAM + obuf + 9) != 0, True)
        check('0x48 is the QP', job(qt, rec_a, idr, qp=8) > bits_i * 5 / 4,
              True)
        bits_p = job(qt, rec_a, trail)
        check('a TRAIL NAL: P, skipped against its reference',
              bits_p < bits_i / 8, True)
        bits_other = job(qt, rec_b, trail)
        check('another stream has its own reference',
              bits_other > bits_i / 2, True)
        qt.writew(mhe + 0x1D * 4, 0xFF)
        qt.writew(mhe + 0x1C * 4, 0x0100)
        check('CMDQ\'s force raises frame done',
              qt.readw(mhe + 0x1E * 4) & 1, 1)
    finally:
        qt.close()


def test_i6_all_machines(qemu, tmp):
    print('every infinity6 family and msc316 machine starts')
    i6 = i6_image(tmp, 'i6all.bin')
    for machine in ('ssc325', 'ssc325de', 'ssc333', 'ssc335', 'ssc335de',
                    'ssc337', 'ssc337de', 'ssc338q', 'ssc30kd', 'ssc30kq',
                    'ssc377', 'ssc377d', 'ssc377de', 'ssc377qe', 'ssc378de',
                    'ssc378qe'):
        qt = QTest(qemu, machine + ',flash-file=' + i6, [])
        try:
            qt.step(1000000)
            check(machine + ': starts', qt.proc.poll() is None, True)
        finally:
            qt.close()


def test_guest_limits(qemu, tmp):
    print('what a guest or an image cannot do to the host')
    path = os.path.join(tmp, 'i3-lim.bin')
    make_i3_image(path)
    qt = QTest(qemu, 'msc313e,flash-file=' + path, [])
    try:
        ive = ((0x00, 0, 0xFFFF, 0xFFFF, 'a 65536-square picture'),
               (0x00, 0, 0xFFFF, 0, 'a 65536-wide picture'),
               (0x01, 0, 0, 4, 'colour conversion one pixel wide'))
        for op, mode, w, h, name in ive:
            qt.writew(IVE1 + 0x04 * 4, op | mode << 8)
            qt.writew(IVE1 + 0x05 * 4, 0)
            qt.writew(IVE1 + 0x06 * 4, w)
            qt.writew(IVE1 + 0x07 * 4, h)
            qt.writew(IVE0, 1)
            check('IVE: ' + name + ' is refused',
                  qt.readw(IVE0 + 0x40 * 4), 0)
        qt.writew(IVE1 + 0x04 * 4, 0x15 | 30 << 8)
        qt.writew(IVE1 + 0x06 * 4, 63)
        qt.writew(IVE1 + 0x07 * 4, 63)
        qt.writew(IVE0, 1)
        check('IVE: an absurd SAD block mode', qt.proc.poll() is None, True)
    finally:
        qt.close()
    bad = os.path.join(tmp, 'nand-bad.bin')
    with open(bad, 'wb') as f:
        f.write(b'MSTARSEMIUSFDCIS' + bytes(0x10000 - 16))
    p = subprocess.run([qemu, '-M', 'ssc337,flash-file=' + bad, '-display',
                        'none', '-nodefaults', '-S'], capture_output=True,
                       timeout=30)
    check('NAND: a zero geometry is an error, not a crash', p.returncode, 1)
    boot = os.path.join(tmp, 'boot.bin')
    with open(boot, 'wb') as f:
        f.write(bytes(4096))
    tool = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                        'tools', 'sstar-nand-image.py')
    p = subprocess.run([sys.executable, '-I', tool,
                        os.path.join(tmp, 'o.nand'), boot, '--bad', '0'],
                       capture_output=True, timeout=30)
    check('NAND tool: a bad block under an image is refused',
          p.returncode != 0, True)
    i6 = i6_image(tmp, 'i6lim.bin')
    out = hmp(qemu, ['-M', 'ssc338q,flash-file=' + i6, '-machine',
                     'accel=qtest', '-S'],
              ['migrate file:' + bad + '.mig', 'info migrate'])
    check('Wave5: migration is refused, not lossy',
          'sstar-wave5' in out, True)


def test_i6e_ring(qemu, tmp):
    print('infinity6e scaler ring: what the MMU maps, and who reads it')
    path = i6_image(tmp, 'i6r.bin')
    mmu = riu(0x1013, 0)
    w, h, ya = 1280, 736, 0x30100000
    ca = ya + w * h
    guard = 0x1000000

    def map_pages(qt, pages):
        qt.writew(mmu + 0x50 * 4, 0x0610)
        qt.writew(mmu + 0x50 * 4, 0x0601)
        for i, pa in enumerate(pages):
            qt.writew(mmu + 0x51 * 4, 0x8000 | ((ya >> 17) + i) & 0x3FF)
            qt.writew(mmu + 0x52 * 4, pa)
            qt.writew(mmu + 0x5B * 4, 1)

    def frame(qt):
        dma, stride = riu(0x1213, 0), riu(0x1215, 0)
        qt.writew(dma + 0x08 * 4, (ya >> 3) & 0xFFFF)
        qt.writew(dma + 0x09 * 4, ya >> 19)
        qt.writew(dma + 0x10 * 4, (ca >> 3) & 0xFFFF)
        qt.writew(dma + 0x11 * 4, ca >> 19)
        qt.writew(stride + 0x54 * 4, w)
        isp = riu(0x1302, 0)
        qt.writew(isp + 0x28 * 4, 0)
        qt.writew(isp, 1)
        qt.step(200 * 1000000)

    qt = QTest(qemu, 'ssc338q,flash-file=' + path, [])
    try:
        map_pages(qt, range(0x80, 0x80 + 12))
        qt.writel(DRAM + guard, 0xAAAAAAAA)
        frame(qt)
        check('a whole buffer is written',
              qt.readl(DRAM + guard) != 0xAAAAAAAA, True)
    finally:
        qt.close()
    qt = QTest(qemu, 'ssc338q,flash-file=' + path, [])
    try:
        # the pages wrap, as a ring's few lines do: nothing unmapped
        map_pages(qt, [0x80, 0x81, 0x82] * 4)
        qt.writel(DRAM + guard, 0xAAAAAAAA)
        frame(qt)
        check('a ring that wraps is not written',
              qt.readl(DRAM + guard), 0xAAAAAAAA)

        def jpe_size(y, c):
            qt.writew(jpe + 0x06 * 4, (y >> 8) & 0xFFFF)
            qt.writew(jpe + 0x07 * 4, y >> 24)
            qt.writew(jpe + 0x0A * 4, (c >> 8) & 0xFFFF)
            qt.writew(jpe + 0x0B * 4, c >> 24)
            qt.writew(jpe + 0x1D * 4, 0xFF)
            qt.writew(jpe + 0x00 * 4, 3)
            qt.step(300 * 1000000)
            check('the JPE got a picture', qt.readw(jpe + 0x1E * 4) & 1, 1)
            return (qt.readw(jpe + 0x44 * 4) |
                    qt.readw(jpe + 0x45 * 4) << 16) - out
        jpe = 0x1F2C4000
        qt.writew(jpe + 0x01 * 4, 1280)
        qt.writew(jpe + 0x02 * 4, 720)
        qt.writew(jpe + 0x5E * 4, 0x5050)
        out, end = 0x2000000 >> 2, 0x20FFFFF >> 2
        for addr, name in ((out, 0), (end, 1)):
            qt.writew(jpe + 0x3C * 4, addr & 0xFFFF)
            qt.writew(jpe + 0x3D * 4, addr >> 16 | name << 14)
            qt.writew(jpe + 0x3E * 4, 1)
        qt.writew(jpe + 0x22 * 4, 0xFE)
        flat = jpe_size(0x300000, 0x400000)
        scene = jpe_size(ya, ca)
        check('the scene, not the ring\'s flat memory (%d > %d)' % (scene, flat),
              scene > flat * 3 / 2, True)
        qt.writew(jpe + 0x01 * 4, 1)
        qt.writew(jpe + 0x1D * 4, 0xFF)
        qt.writew(jpe + 0x00 * 4, 3)
        qt.step(300 * 1000000)
        check('a picture one pixel wide', qt.proc.poll() is None, True)
    finally:
        qt.close()
    qt = QTest(qemu, 'ssc338q,flash-file=' + path, [])
    try:
        # the same with the pages past the first few not mapped at all
        map_pages(qt, [0x80, 0x81, 0x82])
        qt.writel(DRAM + guard, 0xAAAAAAAA)
        frame(qt)
        check('a ring with pages missing is not written',
              qt.readl(DRAM + guard), 0xAAAAAAAA)
    finally:
        qt.close()


def test_i6_stats_audio(qemu, tmp):
    print('infinity6 family: the AE grid, frame done, audio units')
    path = i6_image(tmp, 'i6s.bin')
    qt = QTest(qemu, 'ssc377,flash-file=' + path, [])
    try:
        st = riu(0x131B, 0)
        base = 0x1000000
        qt.writew(st + 0x02 * 4, 1)
        qt.writew(st + 0x08 * 4, (base >> 4) & 0xFFFF)
        qt.writew(st + 0x09 * 4, base >> 20)
        last = DRAM + base + (32 * 32 - 1) * 4
        beyond = DRAM + base + 32 * 32 * 4
        qt.writel(last, 0xAAAAAAAA)
        qt.writel(beyond, 0xAAAAAAAA)
        isp = riu(0x1302, 0)
        qt.writew(isp + 0x28 * 4, 0)
        qt.writew(isp, 1)
        qt.step(100 * 1000000)
        check('AE: the 32 x 32 grid reaches its last block',
              qt.readl(last) != 0xAAAAAAAA, True)
        check('AE: and stops there', qt.readl(beyond), 0xAAAAAAAA)
    finally:
        qt.close()
    qt = QTest(qemu, 'ssc377,flash-file=' + path, [])
    try:
        isp = riu(0x1302, 0)
        qt.writew(isp + 0x28 * 4, 0)
        qt.writew(isp, 1)
        qt.step(100 * 1000000)
        check('infinity6c ISP: frame done and its arming, HalIspIsr',
              (qt.readw(isp + 0x2B * 4) >> 9) & 3, 3)
        bach = riu(0x1502, 0)
        qt.writew(bach + (0x94 >> 1) * 4, 0x10)
        qt.writew(bach + (0x96 >> 1) * 4, 0x100)
        qt.writew(bach + (0x92 >> 1) * 4, 0x8000)
        qt.step(100 * 1000000)
        tone = [qt.readw(DRAM + 0x100000 + i) for i in (4, 6, 8, 10)]
        check('audio: 16-byte units, the ring where the driver put it',
              tone != [0, 0, 0, 0], True)
        check('audio: not at twice the address',
              qt.readl(DRAM + 0x200000 + 8), 0)
        check('audio: two interleaved channels', tone[0], tone[1])
        check('audio: and the next sample differs', tone[2] != tone[0], True)
    finally:
        qt.close()
    qt = QTest(qemu, 'ssc337,flash-file=' + path, [])
    try:
        bach = riu(0x1502, 0)
        qt.writew(bach + (0x94 >> 1) * 4, 0x20)
        qt.writew(bach + (0x96 >> 1) * 4, 0x200)
        qt.writew(bach + (0x92 >> 1) * 4, 0x8000)
        qt.step(100 * 1000000)
        check('audio: 8-byte units elsewhere',
              qt.readl(DRAM + 0x100000 + 8) != 0, True)
    finally:
        qt.close()


def test_i6b0_media(qemu, tmp):
    print('infinity6b0 media: the frame mi_vpe counts')
    path = i6_image(tmp, 'i6b.bin')
    qt = QTest(qemu, 'ssc337,flash-file=' + path, [])
    try:
        isp = riu(0x1302, 0)
        qt.writew(isp + 0x28 * 4, 0)
        qt.writew(isp, 1)
        qt.step(50 * 1000000)
        # HalIspCheckIrqMapTable: without it mi_vpe drops a realtime channel
        check('ISP: group 0 bit 1, _MI_VPE_IspIsr\'s frame count',
              (qt.readw(isp + 0x2B * 4) >> 1) & 1, 1)
    finally:
        qt.close()


def test_i6c_media(qemu, tmp):
    print('infinity6c media: CMDQ, MIU MMU, the Wave5 queue and results')
    path = i6_image(tmp, 'i6c.bin')
    wave = 0x1F34D000
    base = 0x3FF00000       # mem-base: the addresses the firmware is given
    dram = 0x22000000       # what the MMU maps it to, from the CPU
    src = 0x01100000        # absolute
    qt = QTest(qemu, 'ssc377,flash-file=' + path, [])
    try:
        # bit 28 would stall HAL_SCL_UTILITY_RegFire
        cmdq = riu(0x1120, 0)
        check('CMDQ: idle is bit 11 of the high half alone',
              qt.readw(cmdq + 0x45 * 4) & 0x1800, 0x0800)
        check('CMDQ: the twelfth engine is mapped',
              qt.readw(riu(0x1363, 0x45)) & 0x1800, 0x0800)
        # the MMU: bank 0x1659, regions in regs 5 and 6, init done bit 9
        mmu = riu(0x1659, 0)
        qt.writew(mmu, 0x0030)
        check('MMU: reset reports init done', (qt.readw(mmu) >> 9) & 1, 1)
        qt.writew(mmu, 0)
        qt.writew(mmu + 0x05 * 4, 1)        # 0x20000000 up
        qt.writew(mmu + 0x06 * 4, 0)
        for n in range(3):                  # 128 KiB pages
            qt.writew(mmu + 0x03 * 4, 0x100 + n)
            qt.writew(mmu + 0x02 * 4, 0x8000 | ((base >> 17) + n) & 0xFFF)
            qt.writew(mmu + 0x0A * 4, 3)
        qt.writew(mmu, (2 << 1) | 1)
        # boot; the queue's address is taken at INIT_VPU
        qt.writel(wave + 0x48, 0xBD40)
        qt.writel(wave + 0x16C, 0)
        qt.writel(wave + 0x100, 1)
        qt.writel(wave + 0x6C, 1)
        check('Wave5: running', qt.readl(wave + 0x04) != 0, True)

        def cmd(c, opt=0, inst=0):
            qt.writel(wave + 0x104, opt)
            qt.writel(wave + 0x110, inst)
            qt.writel(wave + 0x70, 1)
            qt.writel(wave + 0x100, c)
            qt.writel(wave + 0x38, 1)

        cmd(0x4000)
        check('Wave5: the version chagall.bin must match',
              qt.readl(wave + 0x118), 0x23121119)
        qt.writel(wave + 0x14C, 0x20000)    # the results' buffer
        cmd(0x0008)
        check('Wave5: a polled command raises no reason',
              qt.readl(wave + 0x4C), 0)
        qt.writel(wave + 0x11C, 64 << 16 | 64)
        qt.writel(wave + 0x130, 2 << 16)
        cmd(0x0200)
        for i in range(0, 64 * 96, 4):
            qt.writel(0x20000000 + src + i, 0x20406080)

        def page(n, fire):
            p = dram + n * 0x100
            for i in range(0, 0x100, 4):
                qt.writel(p + i, 0)
            qt.writel(p + 0x00, 0x0100)
            qt.writel(p + 0x04, 0x02)
            qt.writel(p + 0x18, 0x40000)
            qt.writel(p + 0x1C, 0x10000)
            qt.writel(p + 0x48, src)
            qt.writel(p + 0x4C, src + 64 * 64)
            qt.writel(p + 0x54, 64 << 16 | 64)
            qt.writel(p + 0xC0, fire)

        for n, fire in enumerate((0x55, 0x66)):
            page(n, fire)
            qt.writel(wave + 0x1F8, n + 1)
            qt.step(10 * 1000000)
            check('Wave5: queue page %d taken' % n, qt.readl(wave + 0x1F4),
                  n + 1)
            check('Wave5: done, bit 24 with the instance, not a bitmask',
                  qt.readl(wave + 0x1E8), 1 << 24)
            check('Wave5: the picture raises the encoder bit',
                  qt.readl(wave + 0x4C), 0x100)
            qt.writel(wave + 0x34, 0x100)
            qt.writel(wave + 0x3C, 1)
            dump = dram + 0x20000
            check('Wave5: results name the picture', qt.readl(dump), fire)
            check('Wave5: results carry no error bits', qt.readl(dump + 4), 0)
            check('Wave5: results succeed', qt.readl(dump + 8), 1)
            check('Wave5: results hold its bytes', qt.readl(dump + 0x4C) > 0,
                  True)
        check('Wave5: the stream is relative to mem-base, and starts a VPS',
              qt.readl(dram + 0x40000), 0x01000000)
    finally:
        qt.close()


NAND_RAW = 2048 + 64
NAND_BLOCK = 64 * NAND_RAW


def nand_image(tmp, name, bad_bl0=False):
    """A CIS in block 0, the IPL in blocks 2 and 4, as the writer lays them."""
    path = os.path.join(tmp, name)
    img = bytearray(b'\xFF' * (64 * NAND_BLOCK))
    img[0:16] = b'MSTARSEMIUSFDCIS'
    img[0x10:0x13] = bytes([2, 0xC8, 0xF1])
    img[0x20:0x26] = struct.pack('<HHH', 64, 2048, 64)
    img[0x2F:0x31] = bytes([2, 4])
    img[NAND_RAW:NAND_RAW + 4] = b'PNI!'
    ipl = b'\x06\x00\x00\xEAIPL_' + struct.pack('<I', 0x200)
    img[2 * NAND_BLOCK:2 * NAND_BLOCK + 12] = ipl
    img[4 * NAND_BLOCK:4 * NAND_BLOCK + 12] = ipl
    img[2 * NAND_BLOCK + 12:2 * NAND_BLOCK + 16] = b'BL-0'
    img[4 * NAND_BLOCK + 12:4 * NAND_BLOCK + 16] = b'BL-1'
    img[5 * NAND_BLOCK + 3 * NAND_RAW:
        5 * NAND_BLOCK + 3 * NAND_RAW + 4] = b'PAGE'
    if bad_bl0:
        img[2 * NAND_BLOCK + 2048] = 0
    open(path, 'wb').write(img)
    return path


def nand_page_read(qt, page):
    fsp_run(qt, [0x13, page >> 16, (page >> 8) & 0xFF, page & 0xFF],
            (4, 0, 0), (0, 0, 0))


def nand_status(qt):
    return fsp_run(qt, [0x0F, 0xC0], (2, 0, 0), (1, 0, 0))[1][0]


def nand_erase(qt, page):
    fsp_run(qt, [0x06], (1, 0, 0), (0, 0, 0))
    fsp_run(qt, [0xD8, page >> 16, (page >> 8) & 0xFF, page & 0xFF],
            (4, 0, 0), (0, 0, 0))


def nand_unlock(qt):
    fsp_run(qt, [0x1F, 0xA0, 0x00], (3, 0, 0), (0, 0, 0))


def nand_program_bdma(qt, page, src, size):
    """HAL_SPINAND_PROGRAM_BY_BDMA, the load open across two fires."""
    qspi = riu(0x0017, 0)
    fsp_run(qt, [0x06], (1, 0, 0), (0, 0, 0))
    fsp_run(qt, [0x02, 0, 0], (3, 0, 0), (0, 0, 0))
    qt.writew(qspi + 0x0A * 4, 1)
    bdma_run(qt, 0, 0x2B40, src, 0, size)
    fsp_run(qt, [], (0, 0, 0), (0, 0, 0), out=(size + 1, 0))
    qt.writew(qspi + 0x0A * 4, 2)
    fsp_run(qt, [0x10, page >> 16, (page >> 8) & 0xFF, page & 0xFF],
            (4, 0, 0), (0, 0, 0))


def test_nand(qemu, tmp):
    print('SPI NAND: the straps, the ROM\'s CIS, reads, programs, failures')
    path = nand_image(tmp, 'nand.bin')
    qt = QTest(qemu, 'ssc337,flash-file=' + path +
               ',flash-nand-id=0xC8F1',
               ['-global', 'sstar-fsp.ecc-flip-page=%d' % (5 * 64 + 3),
                '-global', 'sstar-fsp.ecc-err-page=%d' % (5 * 64 + 4),
                '-global', 'sstar-fsp.fail-block=7'])
    try:
        check('strap: U-Boot sees SPI NAND',
              (qt.readw(0x1F001C7C) >> 10) & 0x19, 0x11)
        check('strap: the kernel sees SPI NAND',
              qt.readw(0x1F0071C0) & 0x3C, 0x04)
        check('the IPL staged from block 2', qt.readl(IMI + 12),
              int.from_bytes(b'BL-0', 'little'))
        check('the ROM left the SNI in IMI', qt.readl(IMI + 0x10000),
              int.from_bytes(b'MSTA', 'little'))
        check('the ROM left the PNI in IMI', qt.readl(IMI + 0x10A00),
              int.from_bytes(b'PNI!', 'little'))
        check('READ ID after its dummy',
              fsp_run(qt, [0x9F, 0], (2, 0, 0), (2, 0, 0))[1][:2],
              [0xC8, 0xF1])
        check('BP bits set at power-on',
              fsp_run(qt, [0x0F, 0xA0], (2, 0, 0), (1, 0, 0))[1][0], 0x38)
        page = 5 * 64 + 3
        nand_page_read(qt, page)
        check('corrected read reports its bitflips',
              (nand_status(qt) >> 4) & 3, 1)
        check('read from cache by column',
              fsp_run(qt, [0x03, 0, 0, 0], (4, 0, 0), (4, 0, 0))[1][:4],
              list(b'PAGE'))
        bdma_run(qt, 0, 0x4035, 2, 0x100000, 4)
        check('BDMA reads the page register by column',
              qt.readw(DRAM + 0x100000), int.from_bytes(b'GE', 'little'))
        nand_page_read(qt, page + 1)
        check('uncorrectable read reports it',
              (nand_status(qt) >> 4) & 3, 2)
        nand_page_read(qt, page + 2)
        check('a clean read clears the ECC field',
              (nand_status(qt) >> 4) & 3, 0)

        nand_erase(qt, 6 * 64)
        check('erase of a locked block: E_FAIL', nand_status(qt) & 4, 4)
        nand_unlock(qt)
        nand_erase(qt, 2 * 64)
        check('erase: no E_FAIL', nand_status(qt) & 4, 0)
        check('erase spends the write latch', nand_status(qt) & 2, 0)
        nand_erase(qt, 7 * 64)
        check('erase of the failing block: E_FAIL', nand_status(qt) & 4, 4)
        for i in range(64):
            qt.writel(DRAM + 0x200000 + i * 4, 0xC0DE0000 + i)
        nand_program_bdma(qt, 6 * 64 + 1, 0x200000, 256)
        check('program: no P_FAIL', nand_status(qt) & 8, 0)
        data = open(path, 'rb').read()
        off = 6 * NAND_BLOCK + NAND_RAW
        check('program through BDMA written to the file',
              data[off + 4:off + 8], struct.pack('<I', 0xC0DE0001))
        check('the load left the rest of the page erased',
              data[off + 256:off + 260], b'\xFF' * 4)
        check('erase wrote the file, spare included',
              data[2 * NAND_BLOCK:2 * NAND_BLOCK + 4] +
              data[2 * NAND_BLOCK + 2048:2 * NAND_BLOCK + 2052], b'\xFF' * 8)
        # a bad-block mark in the spare, loaded at column 2048 through the RIU
        fsp_run(qt, [0x06], (1, 0, 0), (0, 0, 0))
        fsp_run(qt, [0x02, 0x08, 0x00, 0x00], (4, 0, 0), (0, 0, 0))
        fsp_run(qt, [0x10, 0, 0x01, 0x40], (4, 0, 0), (0, 0, 0))
        data = open(path, 'rb').read()
        check('a page load closed by the next fire programs the spare',
              data[5 * NAND_BLOCK + 2048], 0)
        # the page register full of "PAGE" from the read: a load wipes it
        page = 6 * 64 + 2
        nand_page_read(qt, 5 * 64 + 3)
        for byte in (0xF0, 0x0F):
            fsp_run(qt, [0x06], (1, 0, 0), (0, 0, 0))
            fsp_run(qt, [0x02, 0x00, 0x10, byte], (4, 0, 0), (0, 0, 0))
            fsp_run(qt, [0x10, page >> 16, (page >> 8) & 0xFF, page & 0xFF],
                    (4, 0, 0), (0, 0, 0))
        data = open(path, 'rb').read()
        off = page * NAND_RAW
        check('PROGRAM LOAD clears the page register, a read\'s bytes gone',
              data[off:off + 4], b'\xFF' * 4)
        check('programming only clears bits', data[off + 16], 0)
    finally:
        qt.close()

    path = nand_image(tmp, 'nand-bad.bin', bad_bl0=True)
    qt = QTest(qemu, 'ssc337,flash-file=' + path, [])
    try:
        check('the bad IPL block skipped for its copy', qt.readl(IMI + 12),
              int.from_bytes(b'BL-1', 'little'))
        check('the part the SNI names',
              fsp_run(qt, [0x9F, 0], (2, 0, 0), (2, 0, 0))[1][:2],
              [0xC8, 0xF1])
    finally:
        qt.close()

    # a 4 KiB page part: the geometry comes from the SNI
    raw4 = 4096 + 128
    img = bytearray(b'\xFF' * (64 * 8 * raw4))
    img[0:16] = b'MSTARSEMIUSFDCIS'
    img[0x10:0x13] = bytes([2, 0xC8, 0xF1])
    img[0x20:0x26] = struct.pack('<HHH', 128, 4096, 64)
    img[0x2F:0x31] = bytes([2, 4])
    img[raw4:raw4 + 4] = b'PNI!'
    block = 64 * raw4
    img[2 * block:2 * block + 12] = (b'\x06\x00\x00\xEAIPL_' +
                                     struct.pack('<I', 0x200))
    img[2 * block + 16 * 0 + 12:2 * block + 16] = b'BL4K'
    img[5 * raw4 + 4100:5 * raw4 + 4104] = b'SPAR'
    path = os.path.join(tmp, 'nand4k.bin')
    open(path, 'wb').write(img)
    qt = QTest(qemu, 'ssc337,flash-file=' + path, [])
    try:
        check('4 KiB pages: the IPL found by the SNI\'s geometry',
              qt.readl(IMI + 12), int.from_bytes(b'BL4K', 'little'))
        nand_page_read(qt, 5)
        check('4 KiB pages: a read past 4096 reaches the spare',
              fsp_run(qt, [0x03, 0x10, 0x04, 0], (4, 0, 0), (4, 0, 0))[1][:4],
              list(b'SPAR'))
    finally:
        qt.close()

    path = nand_image(tmp, 'nand-e.bin')
    qt = QTest(qemu, 'ssc338q,flash-file=' + path, [])
    try:
        check('infinity6e strap: DID key BIT4',
              qt.readw(0x1F0071C0) & 0x10BE, 0x10)
    finally:
        qt.close()

    for machine, says in (('ssc338q', b'DRAM size is unknown'),
                          ('ssc325de', b'DRAM size is unknown'),
                          ('ssc337de', b'')):
        r = subprocess.run([qemu, '-M', machine, '-display', 'none', '-S',
                            '-nodefaults', '-qmp', 'stdio'],
                           input=b'{"execute":"qmp_capabilities"}'
                                 b'{"execute":"quit"}',
                           capture_output=True, timeout=20)
        check(machine + ': says when its DRAM size is a guess',
              b'DRAM size is unknown' in r.stderr, bool(says))
    r = subprocess.run([qemu, '-M', 'ssc338q', '-m', '128M', '-display',
                        'none', '-S', '-nodefaults', '-qmp', 'stdio'],
                       input=b'{"execute":"qmp_capabilities"}'
                             b'{"execute":"quit"}',
                       capture_output=True, timeout=20)
    check('ssc338q: -m settles it', b'DRAM size is unknown' in r.stderr,
          False)

    path = os.path.join(tmp, 'nand-short.bin')
    data = bytearray(open(nand_image(tmp, 'nand-s.bin'), 'rb').read())
    open(path, 'wb').write(data[:0x200000])
    r = subprocess.run([qemu, '-M', 'ssc337,flash-file=' + path,
                        '-display', 'none', '-S'], capture_output=True,
                       stdin=subprocess.DEVNULL, timeout=20)
    check('an image without spares is refused by name',
          b'sstar-nand-image' in r.stderr, True)


def test_i6(qemu, tmp):
    print('infinity6 dies: straps, the MIU BIST, BDMA, card detect, the bond')
    path = i6_image(tmp, 'i6.bin')
    want = {'ssc325': 0xEF, 'ssc337': 0xF2, 'ssc338q': 0xF1, 'ssc377': 0xF9}
    for machine, chip in want.items():
        qt = QTest(qemu, machine + ',flash-file=' + path, [])
        try:
            check(machine + ': chip ID', qt.readw(riu(0x001E, 0)) & 0xFF, chip)
            check(machine + ': SPI NOR on the PM clkgen strap',
                  qt.readw(0x1F001C7C) >> 14 & 1, 1)
            check(machine + ': SPI NOR on the DID key strap',
                  qt.readw(0x1F0071C0) & 0x10BE, 0x20)
            check(machine + ': none of the SSD202D fuses',
                  qt.readw(0x1F004014), 0)
            check(machine + ': the IPL sets the DRAM size, not the model',
                  qt.readw(riu(0x1012, 0x69)), 0)
            # the BIST: 63 MiB of 0x5AA5, as the IPL runs it on 64 MiB
            qt.writew(riu(0x1012, 0x71), 0)
            qt.writew(riu(0x1012, 0x72), 0xFFFF)
            qt.writew(riu(0x1012, 0x73), 0x003E)
            qt.writew(riu(0x1012, 0x74), 0x5AA5)
            qt.writew(riu(0x1012, 0x70), 1)
            check(machine + ': BIST done', qt.readw(riu(0x1012, 0x70)) >> 15, 1)
            check(machine + ': BIST pattern in DRAM',
                  (qt.readb(0x20000000), qt.readb(0x20000001)), (0xA5, 0x5A))
            check(machine + ': BIST reaches IPL_CUST\'s header byte',
                  qt.readb(0x23D0001F), 0x5A)
            check(machine + ': BIST stops at 63 MiB',
                  qt.readb(0x20000000 + 0x3F00000), 0)
        finally:
            qt.close()

    # The I6E MMU: BDMA and the CPU reach the region through the table, an
    # address with no entry reaches nothing, and low DRAM is left alone.
    qt = QTest(qemu, 'ssc338q,flash-file=' + path, ['-m', '256M'])
    try:
        ctl = riu(0x1013, 0x50)
        qt.writew(ctl, 0x30)
        check('MMU: reset, init done', qt.readw(ctl) >> 7 & 1, 1)
        qt.writew(ctl, 0)
        qt.writew(ctl, 0x0333)
        qt.writew(riu(0x1013, 0x52), 0x186)
        qt.writew(riu(0x1013, 0x51), 0x8000)
        qt.writew(riu(0x1013, 0x5B), 1)
        qt.writel(0x20004000, 0x11223344)
        ch = riu(0x1002, 0)
        qt.writew(ch + 0x02 * 4, 0x4035)
        qt.writew(ch + 0x03 * 4, 0)
        qt.writew(ch + 0x04 * 4, 0x1234)
        qt.writew(ch + 0x05 * 4, 0)
        qt.writew(ch + 0x06 * 4, 0x0100)
        qt.writew(ch + 0x07 * 4, 0x3000)
        qt.writew(ch + 0x08 * 4, 4)
        qt.writew(ch + 0x09 * 4, 0)
        qt.writew(ch, 1)
        pa = 0x20000000 + (0x186 << 18) + 0x100
        check('MMU: BDMA to MIU 0x30000100 lands on the mapped page',
              qt.readl(pa), 0x21524F4E)
        check('MMU: the CPU sees it at 0x50000100', qt.readl(0x50000100),
              0x21524F4E)
        qt.writel(0x50000104, 0xCAFEF00D)
        check('MMU: a CPU write goes through', qt.readl(pa + 4), 0xCAFEF00D)
        qt.writel(0x50040000, 0xDEADBEEF)
        check('MMU: no entry reads nothing', qt.readl(0x50040000), 0)
        check('MMU: and no write reached DRAM', qt.readl(0x20000000 + 0x40000),
              0)
        qt.writew(ch + 0x06 * 4, 0x4000)
        qt.writew(ch + 0x07 * 4, 0x3004)
        qt.writew(ch, 1)
        check('MMU: a DMA miss spares the kernel\'s page table',
              qt.readl(0x20004000), 0x11223344)
        check('outside the region DRAM still repeats, for the IPL\'s sizing',
              qt.readl(0x30004000), 0x11223344)
    finally:
        qt.close()

    # the I6E FSP follows the bond, vendor halSERFLASH.c
    for bond, bank in ((0x00, 0x0016), (0x10, 0x100D), (0x30, 0x100D)):
        qt = QTest(qemu, 'ssc338q,bond=0x%x,flash-file=%s' % (bond, path), [])
        try:
            check('bond 0x%02x in CHIPTOP' % bond,
                  qt.readw(riu(0x101E, 0x48)), bond)
            base = riu(bank, 0)
            qt.writew(base + 0x60 * 4, 0x9F)
            qt.writew(base + 0x6A * 4, 1)
            qt.writew(base + 0x6B * 4, 3)
            qt.writew(base + 0x6C * 4, 0x0007)
            qt.writew(base + 0x6D * 4, 1)
            check('bond 0x%02x: RDID on FSP 0x%04x' % (bond, bank),
                  qt.readw(base + 0x65 * 4) & 0xFF, 0xEF)
        finally:
            qt.close()

    # Card detect is an input: firmware's writes do not move it
    card = os.path.join(tmp, 'i6-sd.img')
    open(card, 'wb').truncate(16 << 20)
    pad = {'ssc337': (riu(0x000F, 0x47), 4), 'ssc338q': (riu(0x103C, 0x11), 1),
           'ssc377': (riu(0x103E, 0x06), 1), 'ssd202d': (riu(0x000F, 0x47), 4)}
    for machine, (addr, bit) in pad.items():
        for sd, level in ((True, 0), (False, bit)):
            qt = QTest(qemu, machine, ['-drive', 'if=sd,format=raw,file=' + card]
                       if sd else [])
            try:
                qt.writew(addr, 0xFFFF if sd else 0)
                check('%s: CDZ %s a card' % (machine, 'with' if sd else
                                             'without'),
                      qt.readw(addr) & bit, level)
            finally:
                qt.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '../../qemu-src/build/qemu-system-arm'))
    args = ap.parse_args()
    tmp = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, tmp, True)
    path = os.path.join(tmp, 'nor.bin')
    out = os.path.join(tmp, 'uart.out')
    pcap = os.path.join(tmp, 'net.pcap')
    disk = os.path.join(tmp, 'disk.img')
    make_image(path)
    open(disk, 'wb').truncate(1 << 20)
    qt = QTest(args.qemu, 'ssr621q,flash-file=' + path,
               ['-chardev', 'file,id=c0,path=' + out, '-serial', 'chardev:c0',
                '-netdev', 'user,id=n0,restrict=on',
                '-net', 'nic,netdev=n0,model=sstar-emac',
                '-object', 'filter-dump,id=f0,netdev=n0,file=' + pcap,
                '-device', 'usb-kbd,bus=usb-bus.0',
                '-drive', 'if=ide,index=0,format=raw,file=' + disk])
    try:
        test_boot_rom(qt)
        test_riu(qt)
        test_timer(qt)
        test_uart(qt, out)
        test_flash(qt, path)
        test_emac(qt, pcap)
        test_uhc(qt)
        test_sata(qt)
        test_display(qt)
        test_vpu(qt)
    finally:
        qt.close()
    test_migration(args.qemu, tmp, path)
    test_msc313e(args.qemu, tmp)
    test_msc316(args.qemu, tmp)
    test_i3_ive(args.qemu, tmp)
    test_sensor_catalogue(args.qemu)
    test_msc313e_sd(args.qemu, tmp)
    test_ssc029(args.qemu, tmp)
    test_ssc029_rtcpwc(args.qemu, tmp)
    test_ssc029_power_off(args.qemu, tmp)
    test_ssc029_smp(args.qemu, tmp)
    test_ssc034(args.qemu, tmp)
    test_ssc034_sd_venc(args.qemu, tmp)
    test_iford_storage(args.qemu, tmp)
    test_infinity2m_rtcpwc(args.qemu, tmp)
    test_i6(args.qemu, tmp)
    test_nand(args.qemu, tmp)
    test_i6e_media(args.qemu, tmp)
    test_i6_media(args.qemu, tmp)
    test_sd2(args.qemu, tmp)
    test_i6_jpe(args.qemu, tmp)
    test_i6e_ring(args.qemu, tmp)
    test_i6_mask(args.qemu, tmp)
    test_i6_mfe(args.qemu, tmp)
    test_i6_mhe(args.qemu, tmp)
    test_i6_all_machines(args.qemu, tmp)
    test_guest_limits(args.qemu, tmp)
    test_i6_stats_audio(args.qemu, tmp)
    test_i6b0_media(args.qemu, tmp)
    test_i6c_media(args.qemu, tmp)
    print('%d failure(s)' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
