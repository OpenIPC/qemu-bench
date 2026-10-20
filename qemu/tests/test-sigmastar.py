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

    python3 qemu/tests/test-sigmastar.py [--qemu PATH]
"""
import argparse
import os
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


failures = 0


def shown(v):
    if isinstance(v, (bool, str)):
        return v
    if isinstance(v, bytes):
        return v[:8].hex() + ('...' if len(v) > 8 else '')
    if isinstance(v, list):
        return ' '.join(hex(x) for x in v)
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
    finally:
        qt.close()
    test_migration(args.qemu, tmp, path)
    print('%d failure(s)' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
