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

    python3 qemu/tests/test-sigmastar.py [--qemu PATH]
"""
import argparse
import base64
import atexit
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

RIU = 0x1F000000
IMI = 0xA0000000
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

    def cmd(self, line):
        self.f.write(line + '\n')
        self.f.flush()
        while True:
            r = self.f.readline().strip()
            if r.startswith(('OK', 'FAIL', 'ERR')):
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
    test_msc313e_sd(args.qemu, tmp)
    print('%d failure(s)' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
