#!/usr/bin/env python3
"""hisi-femac: a soft reset must forget the RX buffers the driver posted.

GLB_SOFT_RESET used to be a plain store -- the model recorded the value and
kept the RX ring, which holds bare guest addresses and no lengths.  The driver
treats a reset as "the MAC has forgotten everything": hisi_femac_net_close()
runs hisi_femac_free_skb_rings(), which dma_unmap_single()s and kfree_skb()s
every posted RX skb, and open/close are bracketed by hisi_femac_port_reset()
asserting SOFT_RESET_ALL.  So after an interface down/up the model still held
addresses the guest had freed and the page allocator had re-issued, and the
next frame to arrive was DMA'd into them.  The guest then died in whatever
later walked those pages -- unmap_vmas, set_page_dirty,
anon_vma_interval_tree_remove -- with nothing pointing back at the NIC.

No guest: QEMU runs under qtest and this script writes the registers.  Frames
are injected through a socket netdev, so the DMA check is end to end.

The two halves run in separate QEMU instances on purpose.  The register half
leaves a full ring behind when the model is broken, and the DMA half needs an
empty one to be meaningful -- sharing an instance made the DMA check pass on
the unfixed build, because the frame landed in a buffer left over from the
register half rather than in the one under test.

    python3 qemu/tests/test-hisi-femac-reset.py [--qemu PATH]
"""
import argparse, os, socket, sys, time

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
jt = import_module('test-xm-h1-jpeg')

MACHINE = 'hi3516cv200'
FEMAC = 0x10090000
RAM = 0x80000000

MAC_SET = FEMAC + 0x0210
IQ_ADDR = FEMAC + 0x0358
ADDRQ_STAT = FEMAC + 0x036C
GLB_SOFT_RESET = FEMAC + 0x1308
GLB_IRQ_RAW = FEMAC + 0x1338

SOFT_RESET_ALL = 1 << 0
ADDRQ_RX_READY = 1 << 25          # ring has room for more buffers
IRQ_RX_RDY = 1 << 0
IRQ_MULTI_RXRDY = 1 << 7
RX_RING_SIZE = 64

# Two scratch buffers in RAM, well clear of anything a machine init writes.
BUF_A = RAM + 0x0100_0000
BUF_B = RAM + 0x0101_0000
BUFLEN = 256
SENTINEL = bytes(range(256))

# A plain broadcast ARP-shaped frame; the model does not filter, it just DMAs.
FRAME = (b'\xff' * 6 + b'\x52\x54\x00\x12\x34\x56' + b'\x08\x06' +
         bytes(range(46)))


def free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    p = s.getsockname()[1]
    s.close()
    return p


def wait_until_written(q, addr, deadline_s):
    """Poll guest memory until the frame lands.  True if it changed."""
    end = time.time() + deadline_s
    while time.time() < end:
        if q.read(addr, BUFLEN) != SENTINEL:
            return True
        q.readl(ADDRQ_STAT)      # a round-trip, which also pumps the main loop
        time.sleep(0.02)
    return False


def register_checks(qemu, check):
    """The ring is emptied -- observed through ADDRQ_STAT, no networking."""
    q = jt.QTest(qemu, MACHINE)
    try:
        # Bit 25 clears only when the ring is full, so fill it: that makes the
        # occupancy visible without a backdoor into device state.
        for i in range(RX_RING_SIZE):
            q.writel(IQ_ADDR, BUF_A + i * BUFLEN)
        check(not (q.readl(ADDRQ_STAT) & ADDRQ_RX_READY),
              'a full RX ring reports no room (ADDRQ_STAT bit 25 clear)')

        q.writel(GLB_SOFT_RESET, SOFT_RESET_ALL)
        check(bool(q.readl(ADDRQ_STAT) & ADDRQ_RX_READY),
              'SOFT_RESET_ALL empties the RX ring')
        check(not (q.readl(GLB_IRQ_RAW) & (IRQ_RX_RDY | IRQ_MULTI_RXRDY)),
              'SOFT_RESET_ALL clears the pending RX interrupts')
        q.writel(GLB_SOFT_RESET, 0)

        # The clear is the other half of hisi_femac_port_reset(); flushing on
        # it too would drop buffers posted by a driver that resets before it
        # posts.  Edge-triggered, so posting across the deassert must survive.
        for i in range(RX_RING_SIZE):
            q.writel(IQ_ADDR, BUF_A + i * BUFLEN)
            if i == 0:
                q.writel(GLB_SOFT_RESET, 0)
        check(not (q.readl(ADDRQ_STAT) & ADDRQ_RX_READY),
              'writing SOFT_RESET_ALL low again does not drop posted buffers')
    finally:
        q.close()


def dma_checks(qemu, check):
    """End to end: a frame must not reach a buffer the reset dropped.

    Fresh instance, so the ring starts empty on a fixed and an unfixed build
    alike.  Exactly one frame is injected per step: the control's frame
    consumes the one posted buffer, which leaves the ring empty and the
    backend quiet for the step that matters, without relying on a reset to
    get there.
    """
    host = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    host.bind(('127.0.0.1', 0))
    guest_port = free_udp_port()
    q = jt.QTest(qemu, MACHINE, extra=(
        '-nic', 'socket,model=hisi-femac,udp=127.0.0.1:%d,localaddr=127.0.0.1:%d'
        % (host.getsockname()[1], guest_port),))
    try:
        q.writel(MAC_SET, 1600)

        # Positive control.  Without it the check below passes for the wrong
        # reason on any mistake that stops frames arriving at all.
        q.write(BUF_A, SENTINEL)
        q.writel(IQ_ADDR, BUF_A)
        host.sendto(FRAME, ('127.0.0.1', guest_port))
        landed = wait_until_written(q, BUF_A, 5.0)
        check(landed, 'control: a frame is DMA\'d into the posted buffer')
        if not landed:
            return          # nothing below can mean anything

        # The bug.  The guest frees the skb right where the reset is written.
        q.write(BUF_B, SENTINEL)
        q.writel(IQ_ADDR, BUF_B)
        q.writel(GLB_SOFT_RESET, SOFT_RESET_ALL)
        q.writel(GLB_SOFT_RESET, 0)
        host.sendto(FRAME, ('127.0.0.1', guest_port))
        check(not wait_until_written(q, BUF_B, 2.0),
              'no DMA into a buffer the reset dropped (the guest freed it)')
    finally:
        q.close()
        host.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    register_checks(a.qemu, check)
    dma_checks(a.qemu, check)

    print('=== %s: hisi-femac soft reset ===' % ('FAIL' if fails else 'PASS'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
