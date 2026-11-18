#!/usr/bin/env python3
"""QTest: drive a QEMU machine over the qtest protocol, with no guest.

Lifted out of test-xm-h1-jpeg.py, which half the device tests import purely
for this class -- and which imports Pillow at load time, so every one of them
needed Pillow to start whether or not it touched an image.  Nothing here
depends on anything beyond the standard library.
"""
import os, shutil, socket, subprocess, tempfile


class QTest:
    def __init__(self, qemu, machine, extra=()):
        self.dir = tempfile.mkdtemp()
        path = os.path.join(self.dir, 'qtest.sock')
        srv = socket.socket(socket.AF_UNIX); srv.bind(path); srv.listen(1)
        self.proc = subprocess.Popen(
            [qemu, '-M', machine, '-machine', 'accel=qtest', '-display', 'none',
             '-nodefaults', '-qtest', 'unix:' + path, '-serial', 'null', *extra],
            stdout=subprocess.DEVNULL,
            stderr=open(os.path.join(self.dir, 'qemu.err'), 'w'))
        srv.settimeout(20)
        self.sock, _ = srv.accept()
        self.f = self.sock.makefile('rw', buffering=1)

    def cmd(self, line):
        self.f.write(line + '\n'); self.f.flush()
        while True:
            r = self.f.readline().strip()
            if r.startswith('OK') or r.startswith('FAIL') or r.startswith('ERR'):
                return r

    def writel(self, a, v): self.cmd('writel 0x%x 0x%x' % (a, v))
    def readl(self, a): return int(self.cmd('readl 0x%x' % a).split()[1], 16)

    def write(self, a, data):
        for o in range(0, len(data), 4096):
            chunk = data[o:o + 4096]
            self.cmd('write 0x%x 0x%x 0x%s' % (a + o, len(chunk), chunk.hex()))

    def read(self, a, n):
        return bytes.fromhex(self.cmd('read 0x%x 0x%x' % (a, n)).split()[1][2:])

    def close(self):
        self.proc.kill(); self.proc.wait()
        # QEMU's stderr outlives the dir: test-xm-gpio and test-xm510-gpio
        # check its -d unimp lines after close().
        try:
            self.err = open(os.path.join(self.dir, 'qemu.err')).read()
        except OSError:
            self.err = ''
        shutil.rmtree(self.dir, ignore_errors=True)
