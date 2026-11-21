#!/usr/bin/env python3
"""Register-level checks of the Rockchip RV1103 / RV1106 models, no guest
needed.

QEMU runs the rv1106 machine under qtest on a made-up NOR image, and this
script plays the mask ROM's successors (docs/rockchip.md):

  - the boot ROM stand-in: the SPL after the DDR blob in the RKNS ID block
    lands at DRAM 0, data images at their address, the boot source at
    SRAM + 0x10, and the ATAGs the DDR blob leaves (DDR_MEM's sizes after
    its bases, SOC_INFO with the thunder-boot flag); no ID block, or an
    image past the end of the flash, is refused;
  - rk-sfc, the way U-Boot's rockchip_sfc.c drives it: JEDEC ID through
    the DATA FIFO, a read by DMA with the V4 length register, a page
    program and its read-back, and the interrupt status;
  - rk-crypto, the way crypto_v2.c hashes a FIT image: one descriptor, and
    a message split over two with a pause and RESTART between them;
  - rk-decom, the way rockchip_decompress.c unpacks a gzip'd image: the
    job finishes in virtual time, not at ENR, then STAT, ISR and TSIZE;
    LZ4 frames with overlapping matches, stored blocks and every checksum,
    and the error bit for each bad part;
  - rk-syscon: hiword-mask writes, PLL LOCK, the OTP and ADC constants;
  - the UART's DesignWare USR, and the three machines' sizes;
  - what Linux drives once it is up: rk-i2c's register-mode read of the
    sensor's chip ID, a TX write and a NACK; rk-gpio's hiword halves,
    output, input and a both-edge interrupt with only BOTHEDGE set (as
    the kernel leaves it); rk-trng; rk-rtc's calendar, STATUS1 and trim
    test; the mailbox's "MCU done" stand-in; the DW watchdog's fixed
    TOPs; the PL330's PrimeCell ID at the top of its 16 KiB; the GMAC's
    version and its PHY at MDIO 2; the DW MSHC's version;
  - the rv1103b machine (the RV1103B / RV1106B die, peripherals at
    0x20000000): the same boot ROM contract with IRAM at 0x210F6000,
    UART0 1.5 Mbaud and SoC 0x1103 in the ATAGs; the FSPI (the SFC's
    registers) reading the flash; crypto v4; the "rkrng" RNG; the sensor
    on I2C4;
  - the rv1109 machine (RV1126 / RV1109, SPI NAND, OP-TEE): the PMU's
    power-domain and bus-idle handshakes, its core power status, the
    rk3288-type timer counting down to an interrupt, the parallel NAND
    controller answering "no chip", the crypto-v2 RNG inside the crypto
    block, and the SPI NAND's ID and page read through the SFC with the
    dummy byte U-Boot folds into the address.

    python3 qemu/tests/test-rockchip.py [--qemu PATH]
"""
import time
import argparse, atexit, gzip, hashlib, os, shutil, struct, subprocess, sys
import socket, tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from qtest_harness import QTest  # noqa: E402

GRF = 0xFF000000
CRU = 0xFF3A0000
TOPCRU = CRU + 0x10000
SARADC = 0xFF3C0000
OTP = 0xFF3D0000
CRYPTO = 0xFF440000
UART2 = 0xFF4C0000
DECOM = 0xFF520000
I2C4 = 0xFF470000
GPIO0 = 0xFF380000
TRNG = 0xFF448000
RTC = 0xFF1C0000
MBOX = 0xFF5C0000
WDT = 0xFF5A0000
DMAC = 0xFF420000
GMAC = 0xFFA80000
SDMMC = 0xFFAA0000
SRAM = 0xFF6C0000
SFC = 0xFFAC0000
ATAGS = 0x1FE000

ATAG_CORE, ATAG_SERIAL, ATAG_DDR_MEM = 0x54410001, 0x54410050, 0x54410052
ATAG_SOC_INFO = 0x54410057

FLASH_SIZE = 16 << 20
IDB = 0x10000
DATA_AT = 0x40000               # a data image the ROM copies to DRAM
PATTERN_AT = 0x100000           # flash bytes read back through the SFC
ERASED_AT = 0x200000            # left 0xff for the page program

failures = 0


def check(cond, what):
    global failures
    print('%s %s' % ('ok  ' if cond else 'FAIL', what))
    if not cond:
        failures += 1


def entry(off_sec, nsec, addr):
    """header0_info_v2 image entry: size_and_off, address, flag, counter."""
    e = struct.pack('<IIII', nsec << 16 | off_sec, addr, 0, 0)
    return e.ljust(0x58, b'\0')


def make_image(path, entries=None):
    img = bytearray(b'\xff' * FLASH_SIZE)
    ddr = b'DDRBLOB!' * 64                      # 512 bytes, never run
    spl = bytes(range(256)) * 4                 # 1 KiB
    data = b'FASTAE' + bytes(506)               # 512 bytes, to DATA_AT
    if entries is None:
        entries = [entry(4, 1, 0xffffffff),     # DDR blob at IDB + 0x800
                   entry(8, 1, DATA_AT),        # data at IDB + 0x1000
                   entry(12, 2, 0xffffffff)]    # SPL at IDB + 0x1800
    hdr = bytearray(0x800)
    hdr[0:4] = b'RKNS'
    struct.pack_into('<I', hdr, 8, len(entries) << 16)
    for i, e in enumerate(entries):
        hdr[0x78 + 0x58 * i:0x78 + 0x58 * (i + 1)] = e
    img[IDB:IDB + 0x800] = hdr
    img[IDB + 0x800:IDB + 0xa00] = ddr
    img[IDB + 0x1000:IDB + 0x1200] = data
    img[IDB + 0x1800:IDB + 0x1c00] = spl
    pattern = bytes((i * 7 + 3) & 0xff for i in range(0x2000))
    img[PATTERN_AT:PATTERN_AT + len(pattern)] = pattern
    open(path, 'wb').write(img)
    return spl, data, pattern


def read_atags(qt):
    tags, a = {}, ATAGS
    for _ in range(16):
        size, tag = qt.readl(a), qt.readl(a + 4)
        if not size:
            break
        tags[tag] = [qt.readl(a + 8 + 4 * i) for i in range(size - 2)]
        a += 4 * size
    return tags


def test_boot_rom(qt, spl, data, ram):
    check(qt.read(0, len(spl)) == spl, 'SPL (second runnable image) at DRAM 0')
    check(qt.read(DATA_AT, len(data)) == data,
          'data image copied to its address (0x%x)' % DATA_AT)
    check(qt.read(0, 8) != b'DDRBLOB!', 'DDR blob not run')
    check(qt.readl(SRAM + 0x10) == 3, 'boot source at SRAM+0x10 = SPI NOR')
    tags = read_atags(qt)
    check(ATAG_CORE in tags, 'ATAG_CORE first')
    ser = tags.get(ATAG_SERIAL, [0] * 9)
    check(ser[2] == UART2 and ser[4] == 115200 and ser[6] == 2,
          'ATAG_SERIAL: UART2 at 0x%x, 115200' % ser[2])
    ddr = tags.get(ATAG_DDR_MEM, [0] * 45)
    # count, version, bank[] as u64: bank[0] base, bank[count] size
    check(ddr[0] == 1 and ddr[2] == 0 and ddr[4] == ram,
          'ATAG_DDR_MEM: one bank, size 0x%x after the base' % ddr[4])
    soc = tags.get(ATAG_SOC_INFO, [0] * 10)
    check(soc[1] == 0x1106 and soc[2] & 1,
          'ATAG_SOC_INFO: 0x%x, SOC_FLAGS_TDBT (thunder boot)' % soc[1])
    check(all(t[-1] == 0 for t in tags.values()), 'ATAG hashes 0 (accept)')


def sfc_cmd(qt, op, write=False, addr_bytes=0, dummy=0, length=0):
    """Program LEN_EXT and CMD; mode 1 = 24-bit address."""
    qt.writel(SFC + 0x88, 1)                    # LEN_CTRL: use LEN_EXT
    qt.writel(SFC + 0x8c, length)
    qt.writel(SFC + 0x08, 0xffffffff)           # ICLR
    mode = {0: 0, 3: 1, 4: 2}[addr_bytes]
    qt.writel(SFC + 0x100, op | dummy << 8 | int(write) << 12 | mode << 14)


def test_sfc(qt, pattern):
    check(qt.readl(SFC + 0x2c) == 8, 'SFC VER 8 (LEN_EXT present)')
    sfc_cmd(qt, 0x9f, length=3)
    fsr = qt.readl(SFC + 0x20)
    check((fsr >> 16) & 0x1f == 1 and not fsr & 4,
          'FSR: one RX word waiting (0x%x)' % fsr)
    jedec = qt.readl(SFC + 0x108) & 0xffffff
    check(jedec == 0x1820c2, 'JEDEC ID through DATA: %06x (MX25L128)' % jedec)
    check(qt.readl(SFC + 0x28) & 0x10, 'RISR TRAN_FINISH after the last byte')

    n, buf = 0x1800, 0x00300000
    sfc_cmd(qt, 0x03, addr_bytes=3, length=n)
    qt.writel(SFC + 0x104, PATTERN_AT)
    qt.writel(SFC + 0x84, buf)
    qt.writel(SFC + 0x80, 1)
    check(qt.read(buf, n) == pattern[:n], 'read 0x%x bytes by DMA' % n)
    risr = qt.readl(SFC + 0x28)
    check(risr & 0x90 == 0x90, 'RISR DMA + TRAN_FINISH (0x%x)' % risr)
    fast = 0x00340000
    sfc_cmd(qt, 0x0b, addr_bytes=3, dummy=8, length=0x100)
    qt.writel(SFC + 0x104, PATTERN_AT + 0x100)
    qt.writel(SFC + 0x84, fast)
    qt.writel(SFC + 0x80, 1)
    check(qt.read(fast, 0x100) == pattern[0x100:0x200],
          'fast read with 8 dummy clocks')

    sfc_cmd(qt, 0x06)                           # write enable
    sfc_cmd(qt, 0x02, write=True, addr_bytes=3, length=8)
    qt.writel(SFC + 0x104, ERASED_AT)
    qt.writel(SFC + 0x108, 0x44332211)
    qt.writel(SFC + 0x108, 0x88776655)
    for _ in range(100):                        # WIP clears
        sfc_cmd(qt, 0x05, length=1)
        if not qt.readl(SFC + 0x108) & 1:
            break
    sfc_cmd(qt, 0x03, addr_bytes=3, length=12)
    qt.writel(SFC + 0x104, ERASED_AT)
    got = b''.join(struct.pack('<I', qt.readl(SFC + 0x108)) for _ in range(3))
    check(got == bytes(range(0x11, 0x99, 0x11)) + b'\xff' * 4,
          'page program through DATA, read back: %s' % got.hex())


def crypto_hash(qt, chunks, mode=2, hwaddr=0x00400000):
    """Hash chunks, each in its own descriptor; pause after each but the
    last and RESTART, as crypto_v2.c does for a multi-part update."""
    desc = hwaddr + 0x10000
    addrs, a = [], hwaddr
    for c in chunks:
        qt.write(a, c)
        addrs.append(a)
        a += (len(c) + 63) & ~63
    for i, c in enumerate(chunks):
        last = i == len(chunks) - 1
        user = (2 if i == 0 else 0) | (4 if last else 0)
        ctrl = 1 if last else 2
        nxt = 0 if last else desc + 32 * (i + 1)
        qt.write(desc + 32 * i, struct.pack('<8I', addrs[i], len(c), 0, 0,
                                            user, 0, ctrl, nxt))
    qt.writel(CRYPTO + 0x04, 0x00010001)        # SW_CC_RESET
    qt.writel(CRYPTO + 0x48, 0xffff0000 | mode << 4 | 4 | 1)
    qt.writel(CRYPTO + 0x14, desc)
    valid_between = []
    for i in range(len(chunks)):
        qt.writel(CRYPTO + 0x0c, 0xffffffff)
        qt.writel(CRYPTO + 0x10, 0x00010001 if i == 0 else 0x00020002)
        if i < len(chunks) - 1:
            valid_between.append(qt.readl(CRYPTO + 0x3e4))
    st = qt.readl(CRYPTO + 0x0c)
    valid = qt.readl(CRYPTO + 0x3e4)
    words = [qt.readl(CRYPTO + 0x3a0 + 4 * i) for i in range(16)]
    return valid, valid_between, st, b''.join(struct.pack('>I', w)
                                              for w in words)


def test_crypto(qt):
    check(qt.readl(CRYPTO + 0x6f0) == 0x03000000, 'crypto VERSION_NEW v3')
    check(qt.readl(CRYPTO + 0x68c) & 0x04, 'HASH_VERSION has SHA256')
    msg = bytes((i * 13) & 0xff for i in range(3000))
    valid, _, st, out = crypto_hash(qt, [msg])
    check(valid == 1 and st & 4, 'SHA256 one descriptor: VALID, SRC_ITEM_DONE')
    check(out[:32] == hashlib.sha256(msg).digest(),
          'SHA256 digest in DOUT, big-endian words')
    valid, between, _, out = crypto_hash(qt, [msg[:1000], msg[1000:]])
    check(between == [0] and valid == 1,
          'paused after part 1 (not VALID), RESTART finishes')
    check(out[:32] == hashlib.sha256(msg).digest(), 'SHA256 over two parts')
    valid, _, _, out = crypto_hash(qt, [msg], mode=8)
    check(out == hashlib.sha512(msg).digest(), 'SHA512 (mode 8)')
    valid, _, _, out = crypto_hash(qt, [msg], mode=1)
    check(out[:16] == hashlib.md5(msg).digest(), 'MD5 (mode 1)')


def decom_job(qt, src, dst, comp, ctrl, limit):
    qt.write(src, comp)
    qt.writel(DECOM + 0x28, 0xffffffff)         # ISR, write 1 to clear
    qt.writel(DECOM + 0x00, ctrl)
    qt.writel(DECOM + 0x08, src)
    qt.writel(DECOM + 0x0c, dst)
    qt.writel(DECOM + 0x50, limit)
    qt.writel(DECOM + 0x54, 0)
    qt.writel(DECOM + 0x04, 1)


def test_decom(qt):
    raw = bytes((i * 31 + (i >> 8)) & 0xff for i in range(0x30000))
    src, dst = 0x00500000, 0x00600000
    check(qt.readl(DECOM + 0x30) & 0x10, 'decom AXI idle before a job')
    decom_job(qt, src, dst, gzip.compress(raw), 0x11, len(raw) + 16)
    check(not qt.readl(DECOM + 0x24) & 1,
          'gzip job: not COMPLETE at the ENR write')
    check(qt.read(dst, 16) != raw[:16], 'output not in DRAM yet')
    qt.cmd('clock_step 10000000')               # 10 ms at 200 bytes/us
    check(qt.readl(DECOM + 0x24) & 1, 'COMPLETE after virtual time passes')
    check(qt.readl(DECOM + 0x28) & 1, 'ISR done')
    check(qt.readl(DECOM + 0x34) == len(raw) and qt.readl(DECOM + 0x38) == 0,
          'TSIZE = 0x%x bytes' % qt.readl(DECOM + 0x34))
    check(qt.read(dst, len(raw)) == raw, 'gzip output matches')
    qt.writel(DECOM + 0x04, 0)
    qt.writel(DECOM + 0x28, 1)
    check(not qt.readl(DECOM + 0x28) & 1, 'ISR write-1-to-clear')

    import zlib
    decom_job(qt, src, dst + 0x100000, zlib.compress(raw[:0x1000]), 0x21,
              0x1010)
    qt.cmd('clock_step 1000000')
    check(qt.read(dst + 0x100000, 0x1000) == raw[:0x1000], 'zlib job')

    lz = 0x0e    # mode 0 = LZ4, + the header / block / content checks
    data = b'RV1106 thunder boot ' * 50 + raw[:3000]
    frame = lz4_frame(data)
    qt.writel(DECOM + 0x28, 0xffffffff)
    decom_job(qt, src, dst, frame, lz, len(data) + 16)
    qt.cmd('clock_step 1000000')
    isr = qt.readl(DECOM + 0x28)
    check(isr & 1 and qt.read(dst, len(data)) == data,
          'LZ4 frame: overlapping match, literals, both blocks (ISR 0x%x)' % isr)
    check(qt.readl(DECOM + 0x34) == len(data), 'LZ4 TSIZE')
    for what, bad, bit in (('magic', b'\0' + frame[1:], 1 << 1),
                           ('header checksum', frame[:6] + bytes([frame[6] ^ 1])
                            + frame[7:], 1 << 13),
                           ('content checksum', frame[:-1] + bytes([frame[-1] ^ 1]),
                            1 << 15)):
        qt.writel(DECOM + 0x28, 0xffffffff)
        decom_job(qt, src, dst, bad, lz, len(data) + 16)
        qt.cmd('clock_step 1000000')
        isr = qt.readl(DECOM + 0x28)
        check(isr & bit and not isr & 1, 'LZ4 bad %s: ISR 0x%x' % (what, isr))
    try:
        import lz4.frame
    except ImportError:
        return
    big = raw * 4
    qt.writel(DECOM + 0x28, 0xffffffff)
    decom_job(qt, src, dst, lz4.frame.compress(
        big, block_size=lz4.frame.BLOCKSIZE_MAX64KB, content_checksum=True,
        block_checksum=True, block_linked=True), lz, len(big) + 16)
    qt.cmd('clock_step 10000000')
    check(qt.read(dst, len(big)) == big,
          'python-lz4 frame, linked 64 KiB blocks with checksums')


def xxh32(b, seed=0):
    P1, P2, P3, P4, P5 = (2654435761, 2246822519, 3266489917, 668265263,
                          374761393)
    M = 0xffffffff
    rol = lambda x, r: ((x << r) | (x >> (32 - r))) & M
    rnd = lambda a, i: rol((a + i * P2) & M, 13) * P1 & M
    n, i = len(b), 0
    if n >= 16:
        v = [(seed + P1 + P2) & M, (seed + P2) & M, seed, (seed - P1) & M]
        while i + 16 <= n:
            v = [rnd(v[k], struct.unpack_from('<I', b, i + 4 * k)[0])
                 for k in range(4)]
            i += 16
        h = (rol(v[0], 1) + rol(v[1], 7) + rol(v[2], 12) + rol(v[3], 18)) & M
    else:
        h = (seed + P5) & M
    h = (h + n) & M
    while i + 4 <= n:
        h = rol((h + struct.unpack_from('<I', b, i)[0] * P3) & M, 17) * P4 & M
        i += 4
    while i < n:
        h = rol((h + b[i] * P5) & M, 11) * P1 & M
        i += 1
    h ^= h >> 15; h = h * P2 & M; h ^= h >> 13; h = h * P3 & M
    return h ^ (h >> 16)


def lz4_frame(data):
    """An LZ4 frame by hand: block 1 is a run of 20 literals and one long
    overlapping match back over them, block 2 stored raw; every checksum."""
    period = 20
    first = data[:period * 50]
    assert first == first[:period] * 50
    m = len(first) - period - 4 - 15             # beyond 4 + the nibble's 15
    seq = bytes([0xff]) + bytes([period - 15])   # 20 literals, ml 15+...
    seq += first[:period] + struct.pack('<H', period)
    while m >= 255:
        seq += b'\xff'; m -= 255
    seq += bytes([m])
    # an LZ4 block ends with literals: the last 5+ bytes go in a raw block
    body = data[len(first):]
    desc = bytes([0x40 | 0x10 | 0x08 | 0x04, 0x40]) + struct.pack('<Q', len(data))
    out = struct.pack('<I', 0x184D2204) + desc + bytes([(xxh32(desc) >> 8) & 0xff])
    out += struct.pack('<I', len(seq)) + seq + struct.pack('<I', xxh32(seq))
    out += struct.pack('<I', len(body) | 0x80000000) + body
    out += struct.pack('<I', xxh32(body))
    out += struct.pack('<I', 0) + struct.pack('<I', xxh32(data))
    return out


def test_syscon(qt):
    reg = GRF + 0x100
    qt.writel(reg, 0x00ff00a5)
    check(qt.readl(reg) == 0xa5, 'GRF hiword write sets the masked bits')
    qt.writel(reg, 0x0000005a)
    check(qt.readl(reg) == 0xa5, 'GRF write with an empty mask changes nothing')
    qt.writel(reg, 0x000f0000)
    check(qt.readl(reg) == 0xa0, 'GRF mask 0x000f clears the low nibble')
    for name, off in (('APLL', 0x04), ('CPLL', 0x24), ('DPLL', 0x44),
                      ('GPLL', 0x64)):
        check(qt.readl(TOPCRU + off) & (1 << 10), '%s CON1 LOCK' % name)
    qt.writel(TOPCRU + 0x04, 0x04000000)        # try to clear LOCK
    check(qt.readl(TOPCRU + 0x04) & (1 << 10), 'LOCK reads set whatever is written')
    check(qt.readl(TOPCRU + 0x280) == 0x15, 'MODE_CON: PLLs in normal mode')
    qt.writel(OTP + 0x304, 0xffffffff)
    check(qt.readl(OTP + 0x304) == 0xffff, 'OTP INT_STATUS stays done')
    check(qt.readl(OTP + 0x8304) == 0xffff, 'secure OTP INT_STATUS done')
    check(qt.readl(SARADC + 0x802c) == 530, 'TSADC channel 0: code 530, ~40 C')
    check(qt.readl(SARADC + 0x110) == 1 and qt.readl(SARADC + 0x120) == 0x3ff,
          'SARADC: end of conversion, keys released (0x3ff)')


def test_isp32(qt):
    P = 0xFFA00000
    buf = 0x01000000                # a 64x16 NV12 frame in DRAM
    qt.writel(P + 0x59c, 64)        # ISP_OUT_H_SIZE
    qt.writel(P + 0x5a0, 16)        # ISP_OUT_V_SIZE
    qt.writel(P + 0x1408, buf)      # MP Y base
    qt.writel(P + 0x140c, 64 * 16)
    qt.writel(P + 0x141c, buf + 64 * 16)
    qt.writel(P + 0x1420, 64 * 8)
    qt.writel(P + 0x15e8, 64)       # Y line length
    qt.writel(P + 0x1400, 1 | 1 << 13)  # MI: mainpath on, auto-update
    qt.writel(P + 0x1404, 1 << 4)   # MI_WR_INIT: soft update, shadows now
    qt.writel(P + 0x5bc, 0x62)      # IMSC: V_START, FRAME_IN, FRAME
    qt.writel(P + 0x14f8, 1)        # MI IMSC: MP_FRAME
    i2c_reg_write(qt, 0x30, 0x0100, 1)  # the sensor streams
    qt.writel(0xFFA10780, 1)        # VICAP to-ISP channel on: frames flow
    qt.writel(P + 0x400, 1)         # ISP on
    qt.cmd('clock_step 16700000')   # half a frame at 30 fps
    check(qt.readl(P + 0x5c4) == 0x40, 'ISP32 V_START at frame start')
    check(qt.readl(P + 0x1474) == 0x10001 and qt.readl(P + 0x1478) == buf,
          'ISP32 MI shadows from the soft update')
    qt.writel(P + 0x5c8, 0x40)
    qt.cmd('clock_step 16700000')
    check(qt.readl(P + 0x5c4) == 0x22 and qt.readl(P + 0x1500) == 1,
          'ISP32 FRAME_IN, FRAME and MI MP_FRAME at frame end')
    y = qt.readl(buf)
    check(y != 0, 'ISP32 wrote the test picture (Y 0x%08x)' % y)
    # 3A: an enabled block gets MEAS_DONE and its 3A_RIS bit; W1C clears
    qt.writel(P + 0x5000, 1)                    # RAWAWB on
    qt.writel(P + 0x4500, 1)                    # RAWAE lite on
    qt.cmd('clock_step 33400000')               # the next frame end
    check(qt.readl(P + 0x5000) >> 31 and qt.readl(P + 0x4500) >> 31 and
          qt.readl(P + 0x5d4) & 0x802 == 0x802,
          'ISP32 3A: MEAS_DONE and 3A_RIS for AWB and AE lite')
    g = qt.readl(P + 0x4510) & 0xfff
    check(0x40 <= g < 0xfff and qt.readl(P + 0x534c),
          'ISP32 3A: the scene\'s AE mean (G %d of 4095), AWB white points'
          % g)
    qt.writel(P + 0x4800, 1)                    # RAWHIST big on
    qt.cmd('clock_step 33400000')
    total = sum(qt.readl(P + 0x4880) for _ in range(256))
    check(total == 64 * 16, 'ISP32 3A: the histogram counts the frame\'s '
          '%d pixels (%d)' % (64 * 16, total))
    qt.writel(P + 0x5000, qt.readl(P + 0x5000))  # the driver's done write
    check(qt.readl(P + 0x5000) == 1, 'ISP32 3A: MEAS_DONE cleared by W1C')
    # colour processing: contrast 0.5, brightness +40 (the picture here
    # is the flat grey of an unprogrammed exposure: Y only)
    y0 = qt.read(buf, 1)[0]
    qt.writel(P + 0x804, 0x40)
    qt.writel(P + 0x808, 40)
    qt.writel(P + 0x80c, 0x80)
    qt.writel(P + 0x800, 1)
    qt.cmd('clock_step 33400000')
    y1 = qt.read(buf, 1)[0]
    check(y1 == round(y0 * 0.5) + 40,
          'ISP32 CPROC: Y %d -> %d (contrast 0.5, brightness +40)' % (y0, y1))
    qt.writel(P + 0x800, 0)
    # the driver's restart of a paused path: enable, self-update, disable;
    # the shadows must take the new buffer and the enable at once
    qt.writel(P + 0x1400, 0)                    # paused
    qt.writel(P + 0x1408, buf + 0x10000)        # the new buffer
    qt.writel(P + 0x1400, 1)
    qt.writel(P + 0x1800, 1 << 4)               # MP self-update
    qt.writel(P + 0x1400, 0)
    check(qt.readl(P + 0x1474) == 0x10001 and
          qt.readl(P + 0x1478) == buf + 0x10000 and
          not qt.readl(P + 0x1800) & (1 << 4),
          'ISP32 MI self-update latches the MP shadows at once')
    qt.writel(P + 0x400, 0)


def test_vicap_dma(qt):
    # MIPI host 0's ID0 DMA capture: a 64x16 RAW10 frame, MIPI-packed
    # (80 bytes a line), BGGR, at ping-pong buffer 0
    V = 0xFFA10000
    buf = 0x01300000
    w, h = 64, 16
    qt.write(buf, bytes(w * 2 * h))
    i2c_reg_write(qt, 0x30, 0x0100, 1)
    qt.writel(V + 0x104, h << 16 | w)           # ID0_CTRL1: size
    qt.writel(V + 0x134, w * 10 // 8)           # VLW
    qt.writel(V + 0x124, buf)                   # FRM0
    qt.writel(V + 0x128, buf + 0x10000)         # FRM1
    qt.writel(V + 0x174, 0x0303)                # INTEN: FS, FE
    qt.writel(V + 0x100, 0x13)                  # capture, RAW10
    qt.cmd('clock_step 40000000')               # past the first frame end
    st = qt.readl(V + 0x178)
    check(st & 0x101 == 0x101,
          'VICAP DMA: frame 0 start and end in INTSTAT (0x%x)' % st)
    qt.writel(V + 0x178, st)
    check(qt.readl(V + 0x178) == 0, 'VICAP DMA: INTSTAT write-1-to-clear')
    d = qt.read(buf, 2 * w * 10 // 8)

    def px(line, i):
        g = line[i // 4 * 5:i // 4 * 5 + 5]
        return g[i % 4] << 2 | (g[4] >> (2 * (i % 4)) & 3)
    even, odd = d[:80], d[80:160]
    b, g0, g1, r = px(even, 0), px(even, 1), px(odd, 0), px(odd, 1)
    check(g0 > 0 and abs(g0 - g1) <= 1 and b < g0 and r < g0,
          'VICAP DMA: the grey scene in BGGR through a sensor\'s response '
          '(B %d G %d G %d R %d)' % (b, g0, g1, r))
    check(abs(r / g0 - 0.606) < 0.02 and abs(b / g0 - 0.588) < 0.02,
          'VICAP DMA: R/G %.3f B/G %.3f (0.606, 0.588)' % (r / g0, b / g0))
    qt.writel(V + 0x100, 0)


def test_isp32_colour(qt):
    # white balance, colour matrix and gamma out on a 64x40 frame: block
    # row 0 is the grey gradient, row 2 the colour bars (bar 0 red at x 8)
    P = 0xFFA00000
    w, h = 64, 40
    buf = 0x01000000
    qt.writel(P + 0x59c, w)
    qt.writel(P + 0x5a0, h)
    qt.writel(P + 0x1408, buf)
    qt.writel(P + 0x140c, w * h)
    qt.writel(P + 0x141c, buf + w * h)
    qt.writel(P + 0x1420, w * h // 2)
    qt.writel(P + 0x15e8, w)
    qt.writel(P + 0x1400, 1 | 1 << 13)
    qt.writel(P + 0x1404, 1 << 4)
    qt.writel(0xFFA10780, 1)

    def frame():
        qt.cmd('clock_step 33400000')
        qt.cmd('clock_step 33400000')
        uv = qt.read(buf + w * h, w * h // 2)
        y = qt.read(buf, w * h)
        # (Y, Cb, Cr) at block (bx, by)
        return lambda bx, by: (y[(by * 8 + 4) * w + bx * 8 + 4],
                               uv[(by * 4 + 2) * w + bx * 8 + 4],
                               uv[(by * 4 + 2) * w + bx * 8 + 5])

    qt.writel(P + 0x400, 1)                     # ISP on, no AWB gains
    px = frame()
    raw = px(1, 0)
    check(raw[1] < 128 and raw[2] < 128,
          'ISP32: the raw response is green, like a Bayer sensor\'s '
          '(grey Cb/Cr %d/%d)' % raw[1:])
    # AWB gains that undo the response (R/G 0.606, B/G 0.588), Q8
    qt.writel(P + 0x538, 256 << 16 | 256)
    qt.writel(P + 0x53c, round(256 / 0.606) << 16 | round(256 / 0.588))
    qt.writel(P + 0x530, 256 << 16 | 256)       # AWB1: unity
    qt.writel(P + 0x534, 256 << 16 | 256)
    qt.writel(P + 0x400, 1 | 1 << 7)            # AWB gains on
    px = frame()
    g = px(1, 0)
    check(abs(g[1] - 128) <= 1 and abs(g[2] - 128) <= 1,
          'ISP32 AWB gains: grey comes out neutral (Cb/Cr %d/%d)' % g[1:])
    red = px(1, 2)
    check(red[2] > 140 and red[1] < 128,
          'ISP32: the red bar is red (Cb/Cr %d/%d)' % red[1:])
    # a colour matrix swapping R and B: (I + C / 128), C rows Q7
    def pack(a, b):
        return (a & 0xffff) | (b & 0xffff) << 16
    qt.writel(P + 0x704, pack(-128, 0))         # R row: -R + 0 G
    qt.writel(P + 0x708, pack(128, 0))          #        + B
    qt.writel(P + 0x70c, pack(0, 0))            # G row: identity
    qt.writel(P + 0x710, pack(0, 0))
    qt.writel(P + 0x714, pack(128, 0))          # B row: + R
    qt.writel(P + 0x718, pack(-128, 0))         #        - B
    qt.writel(P + 0x700, 1)
    px = frame()
    blue = px(1, 2)
    check(blue[1] > 140 and blue[2] < 128,
          'ISP32 CCM: R and B swapped, the red bar turns blue (Cb/Cr %d/%d)'
          % blue[1:])
    qt.writel(P + 0x700, 0)
    # gamma out: a flat curve at full scale
    for i in range(25):
        qt.writel(P + 0x910 + 4 * i, 4095 << 16 | 4095)
    qt.writel(P + 0x900, 5)
    px = frame()
    check(px(1, 0)[0] == 235, 'ISP32 GOC: a curve flat at 4095 gives '
          'Y 235 (%d)' % px(1, 0)[0])
    qt.writel(P + 0x900, 0)
    qt.writel(P + 0x400, 0)


def test_rknpu(qt):
    # RKNPU (rknpu_job.c): a 3-task job ends after 3 x 200 us with
    # INT_STATUS = INT_MASK and the task counter at 3; INT_CLEAR clears
    N = 0xFF660000
    qt.writel(N + 0x20, 0x300)                  # INT_MASK: the last task's
    qt.writel(N + 0x24, 0x1ffff)                # INT_CLEAR
    qt.writel(N + 0x30, (0x6 << 16) | 3)        # 3 tasks
    qt.writel(N + 0x10, 0x01000000)             # PC_DATA_ADDR
    qt.writel(N + 0x08, 1)
    qt.writel(N + 0x08, 0)
    qt.cmd('clock_step 500000')
    check(qt.readl(N + 0x28) == 0, 'RKNPU: not done before 3 tasks\' time')
    qt.cmd('clock_step 200000')
    check(qt.readl(N + 0x28) == 0x300 and qt.readl(N + 0x3c) == 3,
          'RKNPU: INT_STATUS = INT_MASK, 3 tasks done')
    qt.writel(N + 0x24, 0x1ffff)
    check(qt.readl(N + 0x28) == 0, 'RKNPU: INT_CLEAR')
    # a one-task job whose DPU writes a 2x1x16 cube at DMA base + 0x1000:
    # the cube reads INT8's minimum after it, the byte past it untouched
    base = 0x01000000
    cmds = [(0x1001, 0x4020, 0x1000), (0x1001, 0x4024, 0),
            (0x1001, 0x4030, 1), (0x1001, 0x4034, 0), (0x1001, 0x403c, 15),
            (0x0101, 0x0010, 0), (0x0081, 0x0008, 0x1d)]
    qt.write(base, b''.join(struct.pack('<Q', t << 48 | v << 16 | r)
                            for t, r, v in cmds))
    qt.write(base + 0x1000, bytes(33))
    qt.writel(N + 0x30, (0x6 << 16) | 1)
    qt.writel(N + 0x10, base)
    qt.writel(N + 0x34, base)                   # PC_DMA_BASE_ADDR
    qt.writel(N + 0x08, 1)
    qt.writel(N + 0x08, 0)
    qt.cmd('clock_step 300000')
    out = qt.read(base + 0x1000, 33)
    check(out[:32] == b'\x80' * 32 and out[32] == 0,
          'RKNPU: the DPU output cube blanked to 0x80 (%s)' % out.hex())
    qt.writel(N + 0x24, 0x1ffff)


def test_vepu_pp(qt):
    # RKVENC-PP (kmpp mpp_rkvenc_pp.c): ENC_STRT, then INT_STA bit 0
    P = 0xFFA60000
    qt.writel(P + 0x20, 0x3f)                   # INT_EN
    qt.writel(P + 0x28, 0x3f)
    qt.writel(P + 0x10, 1)                      # ENC_STRT
    qt.cmd('clock_step 600000')
    check(qt.readl(P + 0x2c) == 1, 'RKVENC-PP: the job done (INT_STA bit 0)')
    qt.writel(P + 0x28, 0x3f)
    check(qt.readl(P + 0x2c) == 0, 'RKVENC-PP: INT_CLR clears it')
    # motion: a 128x64 picture, two jobs; a 32x32 square moves into
    # block (2, 1) -- its bit (byte 64 + 0, bit 2) is the only one set
    src, md, w, h = 0x01000000, 0x01100000, 128, 64
    qt.write(src, bytes(w * h))
    qt.writel(P + 0x038, (w // 8 - 1) | (h // 8 - 1) << 16)
    qt.writel(P + 0x10c, src)
    qt.writel(P + 0x118, w)
    qt.writel(P + 0x400, md)
    qt.writel(P + 0x404, 40 << 4 | 3)           # thres_sad 40, cur + ref
    qt.writel(P + 0x40c, 64)
    qt.writel(P + 0x10, 1)
    for y in range(32, 64):
        qt.write(src + y * w + 64, b'\xc8' * 32)
    qt.write(md, b'\xff' * 128)
    qt.writel(P + 0x10, 1)
    flags = qt.read(md, 128)
    check(flags[64] == 1 << 2 and flags.count(0) == 127,
          'RKVENC-PP motion: only block (2, 1) flagged (%s)'
          % flags[:2].hex() + flags[64:66].hex())


def test_vepu540c(qt):
    V = 0xFFA50000
    src, bs = 0x01100000, 0x01200000
    w, h = 64, 16
    y = bytes((16 + x + r) & 0xff for r in range(h) for x in range(w))
    qt.write(src, y)
    qt.write(src + w * h, bytes([128]) * (w * h // 2))
    regs = {
        0x280: src, 0x284: src + w * h,         # NV12 planes
        0x334: w, 0x338: w,                     # strides
        0x28c: 0x01300000, 0x294: 0x01300000,   # reconstruction
        0x2b0: bs + 0xffff, 0x2b4: bs,          # stream buffer top, bottom
        0x2b8: bs, 0x2bc: bs,                   # write and read pointers
        0x300: 1 | (30 << 8),                   # H.265, QP 30
        0x310: (w // 8 - 1) | ((h // 8 - 1) << 16),
        0x318: 6 << 2,                          # NV12
        0x3b0: 19,                              # IDR_W_RADL
        0x3bc: 2 << 15,                         # I slice
        0x3c0: 30 << 16,                        # slice QP
        0x020: 1,                               # frame-done interrupt
    }
    for off, v in regs.items():
        qt.writel(V + off, v)
    qt.writel(V + 0x10, 0x100)                  # one frame
    qt.cmd('clock_step 2000000')
    check(qt.readl(V + 0x2c) & 1, 'VEPU540C frame done after the start')
    n = qt.readl(V + 0x4000)
    head = qt.readl(bs)
    check(n > 0 and head == 0x01000000,
          'VEPU540C wrote %d bytes of Annex B stream (0x%08x)' % (n, head))
    check(qt.readl(V + 0x400c) == 30 * (w // 8) * (h // 8),
          'VEPU540C QP_SUM is the QP times the 8x8 blocks')
    qt.writel(V + 0x28, 1)
    check(qt.readl(V + 0x2c) == 0, 'VEPU540C INT_CLR clears it')


def test_vepu540c_osd(qt):
    """VEPU540C OSD (kmpp's vepu540c_osd_reg at 0x3000): an ARGB1555
    region of opaque white pixels lands in the coded picture."""
    V = 0xFFA50000
    src, bs, rec, osd = 0x01100000, 0x01200000, 0x01300000, 0x01400000
    qt.write(bs, bytes(256))
    w, h = 64, 16
    qt.write(src, bytes([60]) * (w * h))
    qt.write(src + w * h, bytes([128]) * (w * h // 2))
    qt.write(osd, bytes([0xff, 0xff]) * (16 * 8))   # A=1, R=G=B=31
    regs = {
        0x280: src, 0x284: src + w * h, 0x334: w, 0x338: w,
        0x28c: rec, 0x294: rec,
        0x2b0: bs + 0xffff, 0x2b4: bs, 0x2b8: bs, 0x2bc: bs,
        0x300: 1 | (10 << 8), 0x310: (w // 8 - 1) | ((h // 8 - 1) << 16),
        0x318: 6 << 2, 0x3b0: 19, 0x3bc: 2 << 15, 0x3c0: 10 << 16,
        0x020: 1,
        # region 0: on, ARGB1555, foreground alpha 255, x 16..31, y 0..7
        0x3000: 1 | 3 << 8 | 255 << 22, 0x3004: 16 | 0 << 14,
        0x3008: 31 | 7 << 14, 0x300c: osd, 0x3014: 32,
    }
    for off, v in regs.items():
        qt.writel(V + off, v)
    qt.writel(V + 0x10, 0x100)
    qt.cmd('clock_step 2000000')
    import shutil
    import subprocess
    if not shutil.which('ffmpeg'):
        print('skip VEPU540C OSD: no ffmpeg')
        qt.writel(V + 0x3000, 0)
        qt.writel(V + 0x28, 1)
        return
    stream = qt.read(bs, qt.readl(V + 0x4000))
    r = subprocess.run(['ffmpeg', '-v', 'error', '-f', 'hevc', '-i', '-',
                        '-f', 'rawvideo', '-pix_fmt', 'gray', '-'],
                       input=stream, capture_output=True)
    y = r.stdout[:w * h]
    if len(y) < w * h:
        check(False, 'VEPU540C OSD: the picture decodes (%s)' %
              r.stderr.decode()[:80])
        return
    inside, outside = y[2 * w + 20], y[2 * w + 40]
    check(inside > 200 and 50 < outside < 70,
          'VEPU540C OSD: white over the region (Y %d), the source beside '
          'it (Y %d)' % (inside, outside))
    qt.writel(V + 0x3000, 0)
    qt.writel(V + 0x28, 1)


def test_vepu540c_hevc_detail(qt):
    """H.265 with the whole intra residual: a 2-pixel checkerboard over a
    gradient (all AC) decodes in ffmpeg close to its source, IDR and P."""
    import shutil
    import subprocess
    if not shutil.which('ffmpeg'):
        print('skip VEPU540C H.265 detail: no ffmpeg')
        return
    V = 0xFFA50000
    w, h = 64, 32
    src, bs = 0x01900000, 0x01a00000
    pics = [bytes(min(255, 40 + 2 * x + 3 * r + (60 if (x // 2 + r // 2 + sh)
                                                 % 2 else 0))
                  for r in range(h) for x in range(w)) for sh in (0, 0, 1)]
    stream = b''
    for k, y in enumerate(pics):
        qt.write(src, y + bytes([128]) * (w * h // 2))
        rf = 0x01b00000 + 0x20000 * k
        at = bs + len(stream)
        regs = {
            0x280: src, 0x284: src + w * h, 0x334: w, 0x338: w,
            0x28c: rf, 0x294: rf - 0x20000 if k else rf,
            0x2b0: bs + 0xfffff, 0x2b4: bs, 0x2b8: at, 0x2bc: bs,
            0x300: 1 | (22 << 8),
            0x310: (w // 8 - 1) | ((h // 8 - 1) << 16),
            0x318: 6 << 2,
            0x3b0: 19 if k == 0 else 1,             # IDR_W_RADL, TRAIL_R
            0x3bc: (2 if k == 0 else 1) << 15,      # I, P
            0x3c0: 22 << 16, 0x3c4: k,
            0x020: 1,
        }
        for off, v in regs.items():
            qt.writel(V + off, v)
        qt.writel(V + 0x10, 0x100)
        qt.cmd('clock_step 2000000')
        n = qt.readl(V + 0x4000)
        qt.writel(V + 0x28, 1)
        stream += qt.read(at, n)
    r = subprocess.run(['ffmpeg', '-v', 'error', '-f', 'hevc', '-i', '-',
                        '-f', 'rawvideo', '-pix_fmt', 'gray', '-'],
                       input=stream, capture_output=True)
    dec = r.stdout
    errs = []
    for k in range(min(len(dec) // (w * h), 3)):
        d = dec[k * w * h:(k + 1) * w * h]
        e = [abs(a - b) for a, b in zip(d, pics[k])]
        errs.append((max(e), round(sum(e) / len(e), 2)))
    check(len(errs) == 3 and not r.stderr and
          all(m <= 32 and mean <= 10 for m, mean in errs),   # DC only: 64, 28
          'VEPU540C H.265 detail (AC residual): 3 pictures decode clean, '
          'Y error (max, mean) %s %s' % (errs, r.stderr[:80]))


def test_vepu540c_h264(qt):
    """libx264's SPS / PPS as the HAL's (baseline: CAVLC, main: CABAC); an
    IDR, a P picture of the same source and one of a shifted source decode
    in ffmpeg to their sources."""
    import shutil
    import subprocess
    import tempfile
    if not shutil.which('ffmpeg'):
        print('skip VEPU540C H.264: no ffmpeg')
        return
    w, h = 64, 32
    V = 0xFFA50000
    src, bs = 0x01600000, 0x01700000
    pics = [bytes((40 + x + sh + 2 * r) & 0xff for r in range(h)
                  for x in range(w)) for sh in (0, 0, 3)]
    for prof in ('baseline', 'main'):
        with tempfile.TemporaryDirectory() as d:
            r = subprocess.run(['ffmpeg', '-v', 'error', '-f', 'lavfi', '-i',
                                'testsrc=s=%dx%d:d=0.04' % (w, h), '-c:v',
                                'libx264', '-pix_fmt', 'yuv420p',
                                '-profile:v', prof, '-bsf:v',
                                'filter_units=pass_types=7-8', '-f', 'h264',
                                d + '/hdr.h264'], capture_output=True)
            if r.returncode:
                print('skip VEPU540C H.264: no libx264')
                return
            hdr = open(d + '/hdr.h264', 'rb').read()
            qt.write(bs, hdr)
            at = bs + len(hdr)
            stream = hdr
            for k, y in enumerate(pics):
                idr = k == 0
                qt.write(src, y + bytes([128]) * (w * h // 2))
                rf = 0x01800000 + 0x20000 * k
                regs = {
                    0x280: src, 0x284: src + w * h, 0x334: w, 0x338: w,
                    0x28c: rf, 0x294: rf - 0x20000 if k else rf,
                    0x2b0: bs + 0xfffff, 0x2b4: bs, 0x2b8: at, 0x2bc: bs,
                    0x300: 0 | (28 << 8),           # H.264, QP 28
                    0x310: (w // 8 - 1) | ((h // 8 - 1) << 16),
                    0x318: 6 << 2, 0x3b0: ((5 if idr else 1) << 2) | 3,
                    0x3bc: 2 if idr else 0,         # I / P slice
                    0x47c: 0, 0x020: 1,
                }
                for off, v in regs.items():
                    qt.writel(V + off, v)
                qt.writel(V + 0x10, 0x100)
                qt.cmd('clock_step 2000000')
                n = qt.readl(V + 0x4000)
                check(qt.readl(V + 0x2c) & 1 and n > 4,
                      'VEPU540C H.264 %s %s: %d bytes' %
                      (prof, 'IDR' if idr else 'P', n))
                stream += qt.read(at, n)
                at += n
                qt.writel(V + 0x28, 1)
            open(d + '/out.h264', 'wb').write(stream)
            r = subprocess.run(['ffmpeg', '-v', 'error', '-i', d + '/out.h264',
                                '-f', 'rawvideo', '-pix_fmt', 'gray', '-'],
                               capture_output=True)
            frames = [r.stdout[k * w * h:(k + 1) * w * h]
                      for k in range(len(r.stdout) // (w * h))]
            errs = [max(abs(a - b) for a, b in zip(f, y))
                    for f, y in zip(frames, pics)]
            check(not r.stderr and len(frames) == 3 and max(errs) <= 16,
                  'VEPU540C H.264 %s decodes: %d pictures, Y error max %s %s'
                  % (prof, len(frames), errs, r.stderr.decode()[:120]))


def test_vepu540c_jpeg(qt):
    """The HAL's header (here PIL's: Annex K tables, 4:2:0) + the model's
    scan and EOI decode to the source picture."""
    try:
        from PIL import Image
        import io
    except ImportError:
        print('skip VEPU540C JPEG: no PIL')
        return
    V = 0xFFA50000
    w, h = 64, 32
    src, bs = 0x01400000, 0x01500000
    y = bytes(40 + x + 2 * r for r in range(h) for x in range(w))
    uv = bytes([100, 160] * (w // 2) * (h // 2))
    qt.write(src, y + uv)
    img = Image.frombytes('L', (w, h), y).convert('YCbCr')
    ref = io.BytesIO()
    img.save(ref, 'JPEG', quality=90, subsampling=2)
    jfif = ref.getvalue()
    sos = jfif.index(b'\xff\xda')
    hdr = jfif[:sos + 2 + (jfif[sos + 2] << 8 | jfif[sos + 3])]
    qt.write(bs, hdr)
    regs = {
        0x404: bs, 0x40c: bs + len(hdr), 0x408: bs, 0x400: bs + 0xffff,
        0x420: src, 0x424: src + w * h, 0x464: w, 0x468: w,
        0x440: (w // 8 - 1) | ((h // 8 - 1) << 16),
        0x300: 2,                               # JPEG alone
        0x47c: 1 << 31,                         # JPEG
        0x020: 1,
    }
    for off, v in regs.items():
        qt.writel(V + off, v)
    qt.writel(V + 0x10, 0x100)
    for _ in range(200):                # the scan runs on a host thread
        qt.cmd('clock_step 2000000')
        if qt.readl(V + 0x2c) & 1:
            break
        time.sleep(0.01)
    n = qt.readl(V + 0x4064)
    check(qt.readl(V + 0x2c) & 1 and n > 2,
          'VEPU540C JPEG frame done, %d bytes of scan' % n)
    data = qt.read(bs, len(hdr) + n)
    check(data.endswith(b'\xff\xd9'), 'VEPU540C JPEG ends with EOI')
    try:
        dec = Image.open(io.BytesIO(data)).convert('YCbCr')
        dec.load()
        dy = dec.split()[0].tobytes()
        cb = dec.split()[1].tobytes()
        errs = [abs(a - b) for a, b in zip(dy, y)]
        mean = sum(errs) / len(errs)
        check(max(errs) <= 8 and mean <= 1.5 and
              abs(cb[w * h // 2] - 100) <= 6,
              'VEPU540C JPEG decodes to the source (Y error max %d, mean '
              '%.2f; Cb %d)' % (max(errs), mean, cb[w * h // 2]))
    except Exception as e:
        check(False, 'VEPU540C JPEG decodes: %s' % e)
    qt.writel(V + 0x28, 1)
    qt.writel(V + 0x47c, 0)


def test_vepu2_jpeg(qt):
    """RV1126 VEPU2 JPEG: the HAL's header before the output address, the
    model's scan + EOI after it; 64 x 30 (a 2-line y fill), NV12."""
    try:
        from PIL import Image
        import io
    except ImportError:
        print('skip VEPU2 JPEG: no PIL')
        return
    V = 0xFFB90000
    w, h = 64, 30
    src, bs = 0x08800000, 0x08900000
    y = bytes(40 + x + 3 * r for r in range(32) for x in range(w))
    qt.write(src, y + bytes([100, 160] * (w // 2) * 16))
    ref = io.BytesIO()
    Image.frombytes('L', (w, h), y[:w * h]).convert('YCbCr').save(
        ref, 'JPEG', quality=90, subsampling=2)
    jfif = ref.getvalue()
    sos = jfif.index(b'\xff\xda')
    hdr = jfif[:sos + 2 + (jfif[sos + 2] << 8 | jfif[sos + 3])]
    qt.write(bs, hdr)
    out = bs + len(hdr)
    for idx, v in {48: src, 49: src + w * 32, 50: src + w * 32,
                   53: 0x10000, 60: ((out & 7) * 8) << 16 | 2, 61: w,
                   74: 1 << 4, 77: out}.items():
        qt.writel(V + 4 * idx, v)
    qt.writel(V + 4 * 103, (w // 16) << 8 | (32 // 16) << 20 | 1 << 6 |
              2 << 4 | 1)
    qt.cmd('clock_step 3000000')
    st = qt.readl(V + 4 * 109)
    n = qt.readl(V + 4 * 53) // 8 - (out & 7)
    check(st & 3 == 3 and n > 2, 'VEPU2 JPEG done (INT 0x%x), %d bytes' %
          (st, n))
    data = qt.read(bs, len(hdr) + n)
    check(data.endswith(b'\xff\xd9'), 'VEPU2 JPEG ends with EOI')
    try:
        dec = Image.open(io.BytesIO(data)).convert('YCbCr')
        dy = dec.split()[0].tobytes()
        errs = [abs(a - b) for a, b in zip(dy, y[:w * h])]
        check(dec.size == (w, h) and max(errs) <= 8,
              'VEPU2 JPEG decodes to the source (Y error max %d)' % max(errs))
    except Exception as e:
        check(False, 'VEPU2 JPEG decodes: %s' % e)
    qt.writel(V + 4 * 109, 0)


def test_pwm(qt, P, who):
    """A PWM v2 block: channel 1 continuous counts, channel 2 one-shot ends
    with its interrupt status and its enable cleared."""
    qt.writel(P + 0x14, 24000)                  # 1 ms at 24 MHz
    qt.writel(P + 0x18, 6000)
    qt.writel(P + 0x1c, 1 << 5 | 1 << 1 | 1)    # left, continuous, on
    check(qt.readl(P + 0x14) == 24000 and qt.readl(P + 0x18) == 6000 and
          qt.readl(P + 0x1c) & 1, '%s PWM: period, duty, CTRL read back' % who)
    qt.cmd('clock_step 250000')
    c = qt.readl(P + 0x10)
    check(5000 < c < 7000, '%s PWM: the counter runs at 24 MHz (%d)' % (who, c))
    qt.cmd('clock_step 1000000')
    c2 = qt.readl(P + 0x10)
    check(abs(c2 - c) < 200, '%s PWM: and wraps at the period (%d)' % (who, c2))
    qt.writel(P + 0x44, 0xf)
    qt.writel(P + 0x24, 2400)                   # 100 us
    qt.writel(P + 0x2c, 2 << 24 | 1)            # one-shot, 3 periods, on
    qt.cmd('clock_step 250000')
    check(qt.readl(P + 0x2c) & 1 and not qt.readl(P + 0x40),
          '%s PWM one-shot: still running at 250 us' % who)
    qt.cmd('clock_step 100000')
    check(not qt.readl(P + 0x2c) & 1 and qt.readl(P + 0x40) == 4,
          '%s PWM one-shot: done after 3 periods, INTSTS bit 2' % who)
    qt.writel(P + 0x40, 4)
    check(qt.readl(P + 0x40) == 0, '%s PWM: INTSTS write 1 to clear' % who)


def qmp_qom_get(sock_path, path, prop):
    import json
    s = socket.socket(socket.AF_UNIX)
    s.connect(sock_path)
    f = s.makefile('rw', buffering=1)
    f.readline()                                            # greeting
    for c in ({'execute': 'qmp_capabilities'},
              {'execute': 'qom-get',
               'arguments': {'path': path, 'property': prop}}):
        f.write(json.dumps(c) + '\n'); f.flush()
        while True:
            r = json.loads(f.readline())
            if 'return' in r or 'error' in r:
                break
    s.close()
    return r.get('return')


def test_pwm_output(qemu, path):
    # rk-pwm's outputN: the pin's high time in permille, polarity-aware
    tmp = tempfile.mkdtemp()
    qsock = os.path.join(tmp, 'qmp.sock')
    qt = QTest(qemu, 'rv1106,flash-file=' + path,
               ('-qmp', 'unix:%s,server=on,wait=off' % qsock))
    try:
        P = 0xFF350000
        get = lambda n: qmp_qom_get(qsock, '/machine/pwm0', 'output%d' % n)
        check(get(1) == 0, 'PWM output1: stopped, inactive low -> 0')
        qt.writel(P + 0x1c, 1 << 4)                 # stopped, inactive high
        check(get(1) == 1000, 'PWM output1: stopped, inactive high -> 1000')
        qt.writel(P + 0x14, 999)                    # 1000 clocks
        qt.writel(P + 0x18, 250)
        qt.writel(P + 0x1c, 1 << 3 | 1 << 1 | 1)    # duty high, continuous
        check(get(1) == 250, 'PWM output1: 25%% duty high -> %r' % get(1))
        qt.writel(P + 0x1c, 1 << 1 | 1)             # duty low
        check(get(1) == 750, 'PWM output1: 25%% duty low -> %r' % get(1))
    finally:
        qt.close()
        shutil.rmtree(tmp, ignore_errors=True)


def test_sd_write(qemu, path):
    # DW MSHC SD write with BYTCNT 0 (undefined length, as the RV1106
    # vendor kernel writes): the IDMAC chain says how much, CMD12 ends it
    tmp = tempfile.mkdtemp()
    img = os.path.join(tmp, 'sd.img')
    with open(img, 'wb') as f:
        f.truncate(8 << 20)
    qt = QTest(qemu, 'rv1106,flash-file=' + path,
               ('-drive', 'if=sd,index=0,format=raw,file=' + img))
    try:
        M = SDMMC
        R = 1 << 6

        def cmd(idx, arg=0, flags=R):
            qt.writel(M + 0x44, 0xffffffff)         # RINTSTS clear
            qt.writel(M + 0x28, arg)
            qt.writel(M + 0x2c, 1 << 31 | flags | idx)
            return qt.readl(M + 0x30)

        qt.writel(M + 0x04, 1)                      # PWREN
        cmd(0, 0, 0)
        cmd(8, 0x1aa)
        for _ in range(10):
            cmd(55)
            if cmd(41, 0x40ff8000) & 1 << 31:
                break
        cmd(2, 0, R | 1 << 7)
        rca = cmd(3) & 0xffff0000
        cmd(7, rca)
        buf, desc = 0x01000000, 0x01100000
        qt.write(buf, b'OPENENDED' * 113 + b'!!!!!!!')         # 1024 bytes
        qt.write(desc, struct.pack('<4I', 1 << 31 | 1 << 2 | 1 << 3,
                                   1024, buf, 0))  # OWN, LD, FS
        qt.writel(M + 0x00, 1 << 25)                # CTRL: use IDMAC
        qt.writel(M + 0x80, 1 << 7 | 1 << 1)        # BMOD: DE, FB
        qt.writel(M + 0x88, desc)                   # DBADDR
        qt.writel(M + 0x1c, 512)                    # BLKSIZ
        qt.writel(M + 0x20, 0)                      # BYTCNT 0
        # sector 8; the 8 MiB card is standard capacity: a byte address
        cmd(25, 8 * 512, R | 1 << 8 | 1 << 9 | 1 << 10)
        cmd(12, 0, R | 1 << 14)
        d = qt.read(desc, 4)
    finally:
        qt.close()
    data = open(img, 'rb').read()
    shutil.rmtree(tmp, ignore_errors=True)
    check(data[8 * 512:8 * 512 + 1024] == b'OPENENDED' * 113 + b'!!!!!!!'
          and not struct.unpack('<I', d)[0] & 1 << 31,
          'SD: CMD25 with BYTCNT 0 writes the IDMAC chain (2 blocks)')


def test_i2s_rv1106(qt):
    I = 0xFFAE0000                  # no CRU-derived clock: word-rate 96000
    qt.writel(I + 0x1c, 2)          # RX start
    qt.cmd('clock_step 100000')     # 0.1 ms: 9.6 words
    lvl = qt.readl(I + 0x2c)
    check(6 <= lvl <= 12, 'RV1106 I2S RX FIFO at the fixed word rate, to a '
          'tick (%d of 9.6)'
          % lvl)
    qt.writel(I + 0x1c, 0)


def test_i2s_tone(qemu, path):
    """-global rk-i2s-tdm.rx-tone-hz: capture reads a sine, not silence."""
    qt = QTest(qemu, 'rv1106,flash-file=' + path,
               ('-global', 'rk-i2s-tdm.rx-tone-hz=1000'))
    try:
        I = 0xFFAE0000              # word-rate 96000: 96 kHz packed stereo
        qt.writel(I + 0x04, 15)     # RXCR: 16-bit, two channels, packed
        qt.writel(I + 0x1c, 2)
        qt.cmd('clock_step 2000000')
        w = [qt.readl(I + 0x28) for _ in range(96)]
        v = [(x & 0xffff) - ((x & 0x8000) << 1) for x in w]
        check(all(x >> 16 == x & 0xffff for x in w) and
              7900 < max(v) <= 8191 and -8191 <= min(v) < -7900 and
              v[0] == 0 and v[24] > 7900,
              'I2S RX tone: 1 kHz at a quarter scale, both channels alike '
              '(%d .. %d)' % (min(v), max(v)))
    finally:
        qt.close()


def test_i2s_files(qemu, path, tmp):
    """rx-file: capture reads a WAV's samples; tx-file: playback is saved."""
    import wave
    rx, tx = os.path.join(tmp, 'mic.wav'), os.path.join(tmp, 'spk.wav')
    with wave.open(rx, 'wb') as f:
        f.setnchannels(2)
        f.setsampwidth(2)
        f.setframerate(48000)
        f.writeframes(b''.join(struct.pack('<hh', 100 * i, -100 * i)
                               for i in range(64)))
    qt = QTest(qemu, 'rv1106,flash-file=' + path,
               ('-global', 'rk-i2s-tdm.rx-file=' + rx,
                '-global', 'rk-i2s-tdm.tx-file=' + tx))
    try:
        I = 0xFFAE0000
        qt.writel(I + 0x04, 15)     # RXCR: 16-bit stereo, packed
        qt.writel(I + 0x00, 15)     # TXCR: the same
        qt.writel(I + 0x1c, 2)      # RX start
        qt.cmd('clock_step 1000000')
        w = [qt.readl(I + 0x28) for _ in range(8)]
        got = [((x & 0xffff) ^ 0x8000) - 0x8000 for x in w] + \
              [((x >> 16) ^ 0x8000) - 0x8000 for x in w]
        check(got[:8] == [100 * i for i in range(8)] and
              got[8:] == [-100 * i for i in range(8)],
              'I2S rx-file: the WAV\'s left and right samples in order')
        for i in range(16):         # 16 stereo frames played
            qt.writel(I + 0x24, (i * 7 & 0xffff) | ((-i * 7) & 0xffff) << 16)
        qt.writel(I + 0x1c, 3)      # TX on too
        qt.cmd('clock_step 1000000')
        qt.writel(I + 0x1c, 0)      # stop: the header's sizes are written
    finally:
        qt.close()
    with wave.open(tx, 'rb') as f:
        fr = f.readframes(f.getnframes())
        smp = struct.unpack('<%dh' % (len(fr) // 2), fr)
        check(f.getnchannels() == 2 and f.getsampwidth() == 2 and
              list(smp[:8]) == [0, 0, 7, -7, 14, -14, 21, -21],
              'I2S tx-file: a 16-bit stereo WAV of what was played '
              '(%d Hz, %d frames)' % (f.getframerate(), f.getnframes()))


def test_dwc3_gadget(qemu, path):
    """usb-otg=device: the DWC3's device side and the host built into it,
    driven as the dwc3 driver does for the first control transfer."""
    qt = QTest(qemu, 'rv1106,flash-file=%s,usb-otg=device' % path)
    try:
        U, EV, T0, T1, BUF = (0xFFB00000, 0x07a00000, 0x07a10000,
                              0x07a10100, 0x07a20000)
        check(qt.readl(U + 0xc140) & 3 == 0 and qt.readl(U + 0xc70c) >> 22 & 1,
              'DWC3 device: a device-only core, halted until RUN')
        qt.writel(U + 0xc400, EV)
        qt.writel(U + 0xc404, 0)
        qt.writel(U + 0xc408, 0x1000)
        trb = struct.pack('<4I', BUF, 0, 8, 1 | 2 | 0x20 | 0x400 | 0x800)
        qt.write(T0, trb)                        # SETUP TRB, HWO
        qt.writel(U + 0xc804, T0)                # ep0 PAR1
        qt.writel(U + 0xc80c, 0x400 | 6)         # Start Transfer
        r = qt.readl(U + 0xc80c)
        check(not r & 0x400 and (r >> 16) & 0x7f,
              'DWC3 Start Transfer: done at once, a resource index')
        qt.writel(U + 0xc704, 1 << 31)           # RUN
        check(not qt.readl(U + 0xc70c) >> 22 & 1, 'DWC3 running: not halted')
        qt.cmd('clock_step 1000000')
        n = qt.readl(U + 0xc40c)
        ev = struct.unpack('<%dI' % (n // 4), qt.read(EV, n))
        check(ev == (0x101, 0x201, 0x40),
              'DWC3 host: reset, connect done, SETUP XferComplete (%s)' %
              ' '.join('%x' % e for e in ev))
        check(qt.read(BUF, 8) == bytes([0x80, 6, 0, 1, 0, 0, 64, 0]) and
              not struct.unpack('<I', qt.read(T0 + 12, 4))[0] & 1,
              'DWC3 host: GET_DESCRIPTOR(device) in the setup TRB, HWO off')
        qt.writel(U + 0xc40c, n)
        check(qt.readl(U + 0xc40c) == 0, 'DWC3 GEVNTCOUNT: the driver\'s '
              'write takes the handled events off')
        desc = bytes([18, 1, 0, 2, 0, 0, 0, 64, 0x6b, 0x1d, 4, 1, 0, 1,
                      1, 2, 3, 1])
        qt.write(BUF + 0x100, desc)
        qt.write(T1, struct.pack('<4I', BUF + 0x100, 0, 18,
                                 1 | 2 | 0x50 | 0x400 | 0x800))
        qt.writel(U + 0xc814, T1)                # ep1 IN: the data stage
        qt.writel(U + 0xc81c, 0x400 | 6)
        qt.cmd('clock_step 1000000')
        n = qt.readl(U + 0xc40c)
        ev = struct.unpack('<%dI' % (n // 4), qt.read(EV + 12, n))
        check(ev[:2] == (0x42, 0x20c0),
              'DWC3 host: data IN XferComplete, then XferNotReady(STATUS) '
              'on ep0 OUT (%s)' % ' '.join('%x' % e for e in ev))
    finally:
        qt.close()


def test_uart(qt):
    usr = qt.readl(UART2 + 0x7c)
    check(usr & 0x6 == 0x6, 'UART2 USR: TX FIFO not full and empty (0x%x)' % usr)


def i2c_reg_read(qt, addr7, reg, n):
    """The kernel's combined write/read: MRXADDR + MRXRADDR, START, count."""
    qt.writel(I2C4 + 0x1c, 0xff)                         # IPD clear
    qt.writel(I2C4 + 0x08, (addr7 << 1) | (1 << 24))     # MRXADDR, valid
    qt.writel(I2C4 + 0x0c, (reg >> 8) | (reg & 0xff) << 8 | 3 << 24)
    qt.writel(I2C4 + 0x00, 1 | 1 << 1 | 1 << 3 | 1 << 5 | 1 << 6)
    qt.writel(I2C4 + 0x14, n)
    ipd = qt.readl(I2C4 + 0x1c)
    data = struct.pack('<8I', *[qt.readl(I2C4 + 0x200 + 4 * i)
                                for i in range(8)])[:n]
    qt.writel(I2C4 + 0x00, qt.readl(I2C4) & ~(1 << 3) | 1 << 4)  # STOP
    return ipd, data, qt.readl(I2C4 + 0x1c)


def i2c_reg_write(qt, addr7, reg, val):
    """A 16-bit register write: START, address + register + value, STOP."""
    qt.writel(I2C4 + 0x1c, 0xff)
    qt.writel(I2C4 + 0x100, (addr7 << 1) | (reg >> 8) << 8 |
              (reg & 0xff) << 16 | val << 24)
    qt.writel(I2C4 + 0x00, 1 | 1 << 3 | 1 << 6)
    qt.writel(I2C4 + 0x10, 4)
    qt.writel(I2C4 + 0x00, 1 | 1 << 4)


def test_i2c(qt):
    ipd, data, after = i2c_reg_read(qt, 0x30, 0x3107, 2)
    check(ipd & 0x18 == 0x18 and not ipd & 0x40,
          'I2C4 register read: START + MBRF, no NACK (0x%x)' % ipd)
    check(data == b'\x9b\x3a', 'sc2336p chip ID 0x9b3a at 0x30 (%s)' % data.hex())
    check(after & 0x20, 'STOP interrupt pending after STOP')
    # TX: START, address + register + value from TXBUF0, MTXCNT 4
    qt.writel(I2C4 + 0x1c, 0xff)
    qt.writel(I2C4 + 0x100, 0x60 | 0x3e << 8 | 0x01 << 16 | 0x5a << 24)
    qt.writel(I2C4 + 0x00, 1 | 1 << 3 | 1 << 6)
    qt.writel(I2C4 + 0x10, 4)
    check(qt.readl(I2C4 + 0x1c) & 0x04, 'TX of 4 bytes: MBTF')
    check(qt.readl(I2C4 + 0x20) == 4, 'FCNT 4 after the TX')
    qt.writel(I2C4 + 0x00, 1 | 1 << 4)
    _, data, _ = i2c_reg_read(qt, 0x30, 0x3e01, 1)
    check(data == b'\x5a', 'the written sensor register reads back')
    ipd, _, _ = i2c_reg_read(qt, 0x32, 0x3107, 2)
    check(ipd & 0x40 and not ipd & 0x08, 'no device at 0x32: NAKRCV, no MBRF')
    check(qt.readl(I2C4) >> 16 == 0, 'CON version 0: no auto stop')


def gpio_w2(qt, off, val, mask=0xffffffff):
    """gpio_writel_v2: two hiword halves."""
    qt.writel(GPIO0 + off, (val & 0xffff) | (mask & 0xffff) << 16)
    qt.writel(GPIO0 + off + 4, (val >> 16) | (mask >> 16) << 16)


def test_gpio(qt):
    check(qt.readl(GPIO0 + 0x78) == 0x01000C2B, 'GPIO VERSION_ID v2')
    gpio_w2(qt, 0x08, 1 << 5 | 1 << 20)                    # DDR: outputs
    gpio_w2(qt, 0x00, 1 << 20, 1 << 20)                    # DR: pin 20 high
    check(qt.readl(GPIO0 + 0x08 + 4) == 0x10, 'DDR high half reads pins 16..31')
    check(qt.readl(GPIO0 + 0x70) & (1 << 20) and not qt.readl(GPIO0 + 0x70) & 32,
          'EXT_PORT: output 20 high, output 5 low')
    gpio_w2(qt, 0x00, 0, 1 << 5)                           # empty mask
    qt.writel(GPIO0 + 0x00, 1 << 5)                        # no mask bits
    check(not qt.readl(GPIO0 + 0x70) & 32, 'a write without mask bits is ignored')
    # pin 2 as the power key: both edges, INT_TYPE left at level, polarity 0
    gpio_w2(qt, 0x30, 1 << 2, 1 << 2)                      # BOTHEDGE
    gpio_w2(qt, 0x10, 1 << 2, 1 << 2)                      # INT_EN
    check(qt.readl(GPIO0 + 0x58) == 0,
          'both-edge pin low at polarity 0 is not a level interrupt')
    qt.cmd('set_irq_in /machine/gpio0 unnamed-gpio-in 2 1')
    check(qt.readl(GPIO0 + 0x70) & 4, 'input pin 2 driven high')
    check(qt.readl(GPIO0 + 0x50) & 4, 'rising edge latched in INT_STATUS')
    gpio_w2(qt, 0x60, 1 << 2, 1 << 2)                      # EOI
    check(not qt.readl(GPIO0 + 0x58) & 4, 'EOI clears the edge')
    qt.cmd('set_irq_in /machine/gpio0 unnamed-gpio-in 2 0')
    check(qt.readl(GPIO0 + 0x58) & 4, 'falling edge latched too (both edges)')
    gpio_w2(qt, 0x18, 1 << 2, 1 << 2)                      # INT_MASK
    check(qt.readl(GPIO0 + 0x58) & 4 and not qt.readl(GPIO0 + 0x50) & 4,
          'masked: RAWSTATUS set, INT_STATUS clear')


def test_trng(qt):
    check(qt.readl(TRNG + 0xf0) == 0x46bc, 'TRNG VERSION 0x46bc')
    check(qt.readl(TRNG + 0x04) & (1 << 9), 'TRNG STAT seeded')
    words = []
    for _ in range(2):
        qt.writel(TRNG + 0x14, qt.readl(TRNG + 0x14))
        qt.writel(TRNG + 0x08, 1 << 3)                     # 256-bit
        qt.writel(TRNG + 0x00, 1)                          # RAND
        check(qt.readl(TRNG + 0x14) & 1, 'TRNG ISTAT RAND_RDY')
        words.append([qt.readl(TRNG + 0x20 + 4 * i) for i in range(8)])
    check(words[0] != words[1] and any(words[0]), 'two different 256-bit draws')
    qt.writel(TRNG + 0x14, 1)
    check(not qt.readl(TRNG + 0x14) & 1, 'ISTAT write-1-to-clear')


def test_rtc(qt):
    K = 0xc4522900                                         # the driver's key
    check(qt.readl(RTC + 0x44) & 1, 'RTC running from reset: STATUS1 bit 0')
    qt.writel(RTC + 0x3c, K | 0x80)                        # stop
    check(not qt.readl(RTC + 0x44) & 1, 'STATUS1 follows CTRL.START off')
    for i, v in enumerate((0x56, 0x34, 0x12, 0x15, 0x06, 0x30, 0x00, 0x06)):
        qt.writel(RTC + 4 * i, K | v)                      # 2030-06-15 12:34:56
    qt.writel(RTC + 0x3c, K | 0x81)                        # start
    t = [qt.readl(RTC + 4 * i) for i in range(8)]
    check(t[1:7] == [0x34, 0x12, 0x15, 0x06, 0x30, 0x00] and
          0x56 <= t[0] <= 0x58, 'calendar set while stopped, then running '
          '(%s)' % ' '.join('%02x' % v for v in t))
    qt.writel(RTC + 0x88, K | 1)                           # TEST_LEN
    qt.writel(RTC + 0x84, K | 1)                           # TEST start
    check(qt.readl(RTC + 0x84) & 4, 'CLK32K test done at once')
    cnt = sum(qt.readl(RTC + 0x8c + 4 * i) << (8 * i) for i in range(4))
    check(cnt == 48000000, '24 MHz count over 2 s: no trim (%d)' % cnt)


def test_misc(qt):
    check(qt.readl(MBOX + 0x2c) == 2, 'mailbox: MCU message waiting on B2A 1')
    check(qt.readl(MBOX + 0x38) == 0xf00d and qt.readl(MBOX + 0x3c) == 0xdeadbeef,
          'mailbox B2A 1: CMD_MCU_STATUS / MCU_STATUS_DONE')
    qt.writel(MBOX + 0x2c, 2)
    check(qt.readl(MBOX + 0x2c) == 0, 'B2A_STATUS write-1-to-clear')
    check(qt.readl(WDT + 0xf4) & 0x40, 'watchdog COMP_PARAMS_1 USE_FIX_TOP')
    cid = [qt.readl(DMAC + 0x3ff0 + 4 * i) & 0xff for i in range(4)]
    check(cid == [0x0d, 0xf0, 0x05, 0xb1],
          'PL330 PrimeCell ID at the top of 16 KiB (%s)' % cid)
    check(qt.readl(DMAC + 0x3fe0) & 0xff == 0x30, 'PL330 PID0 0x30')
    check(qt.readl(GMAC + 0x110) & 0xff == 0x42, 'GMAC SNPSVER 4.20')
    ids = []
    for ra in (2, 3):
        qt.writel(GMAC + 0x200, 2 << 21 | ra << 16 | 3 << 2 | 1)
        ids.append(qt.readl(GMAC + 0x204) & 0xffff)
    check(ids == [0x0044, 0x1400], 'PHY at MDIO 2: ID 0044.1400')
    qt.writel(GMAC + 0x200, 1 << 21 | 2 << 16 | 3 << 2 | 1)
    check(qt.readl(GMAC + 0x204) & 0xffff == 0xffff, 'nothing at MDIO 1')
    check(qt.readl(SDMMC + 0x6c) & 0xffff == 0x270a, 'SD controller VERID 270a')
    check(qt.readl(SDMMC + 0x50) & 1, 'no card: CDETECT bit 0 set')


def test_refused(qemu, tmp):
    def run(path):
        p = subprocess.run([qemu, '-M', 'rv1106,flash-file=' + path,
                            '-display', 'none', '-nodefaults',
                            '-serial', 'null', '-S', '-machine', 'accel=qtest',
                            '-qtest', 'null'],
                           capture_output=True, text=True, timeout=30)
        return p.returncode, p.stderr
    blank = os.path.join(tmp, 'blank.bin')
    open(blank, 'wb').write(b'\xff' * FLASH_SIZE)
    rc, err = run(blank)
    check(rc != 0 and 'no RKNS' in err, 'a flash without an ID block is refused')
    bad = os.path.join(tmp, 'bad.bin')
    # sector 0x7fff from the ID block is 0x100fe00: past a 16 MiB flash
    make_image(bad, [entry(4, 1, 0xffffffff), entry(0x7fff, 0x10, 0xffffffff)])
    rc, err = run(bad)
    check(rc != 0 and 'outside the flash image' in err,
          'an image past the end of the flash is refused')
    one = os.path.join(tmp, 'one.bin')
    make_image(one, [entry(4, 1, 0xffffffff)])
    rc, err = run(one)
    check(rc != 0 and 'no SPL' in err, 'an ID block with no SPL is refused')


def test_machines(qemu, path):
    for m, ram in (('rv1103', 64 << 20), ('rv1106', 128 << 20),
                   ('rv1106g3', 256 << 20)):
        qt = QTest(qemu, '%s,flash-file=%s' % (m, path))
        try:
            ddr = read_atags(qt).get(ATAG_DDR_MEM, [0] * 5)
            check(ddr[4] == ram, '%s: %d MiB in ATAG_DDR_MEM' % (m, ram >> 20))
        finally:
            qt.close()


def test_tb_isp(qemu, path):
    """tb-isp=: the MCU's thunder-boot result in the ISP's reserved memory."""
    qt = QTest(qemu, 'rv1106,flash-file=%s,tb-isp=0x8b0000:2' % path)
    try:
        def w16(a):
            return int.from_bytes(qt.read(a, 2), 'little')
        A, S = 0x8b0000, 11245
        check(w16(A) == 1 and w16(A + 2) == 1 and w16(A + 14) == 2 and
              w16(A + 16) == 0 and w16(A + S + 16) == 1 and
              w16(A + S + 2) == 1 and w16(A + 10) == 1920,
              'tb-isp: both cameras\' heads: enable, complete OK, 2 cameras, '
              'index 0 / 1, 1920 wide')
        check(qt.readl(A + 20) == 655 and qt.readl(A + 32) == 65536 and
              w16(A + 100 + 4645 + 6) == 0x185,
              'tb-isp: 10 ms at unity gain (Q16), AWB1 R gain 1.52 (Q8)')
    finally:
        qt.close()


def test_gpio_high(qemu, path):
    # a reset button's pull-up: GPIO4_C0 (pin 16) and GPIO0_A2 idle high
    qt = QTest(qemu, 'rv1106,flash-file=%s,gpio-high=4C0/0A2' % path)
    try:
        check(qt.readl(0xFF560070) == 1 << 16, 'gpio-high: GPIO4 EXT_PORT')
        check(qt.readl(0xFF380070) == 1 << 2, 'gpio-high: GPIO0 EXT_PORT')
        check(qt.readl(0xFF530070) == 0, 'gpio-high: GPIO1 reads 0')
    finally:
        qt.close()


def gc6160_xfer(qt, wr, nread=0):
    # bit-banged I2C on GPIO2 (0x20840000): CLK pin 6, DATA pin 11,
    # open drain as XmMotor.ko drives it -- a 1 is the pin turned input
    G, CLK, DAT = 0x20840000, 1 << 6, 1 << 11

    def pin(bit, level):
        if level:
            qt.writel(G + 0x08, bit << 16)              # DDR: input
        else:
            qt.writel(G + 0x00, bit << 16)              # DR 0
            qt.writel(G + 0x08, bit << 16 | bit)        # DDR: output

    def sda():
        return 1 if qt.readl(G + 0x70) & DAT else 0

    def clock(bit):
        pin(DAT, bit)
        pin(CLK, 1)
        b = sda()
        pin(CLK, 0)
        return b

    def byte(v):
        for i in range(7, -1, -1):
            clock((v >> i) & 1)
        return clock(1) == 0                            # ACK

    def start():
        pin(DAT, 1); pin(CLK, 1); pin(DAT, 0); pin(CLK, 0)

    acks = []
    start()
    for v in wr:
        acks.append(byte(v))
    out = b''
    if nread:
        start()                                         # repeated start
        acks.append(byte(0x27))                         # 0x26 | read
        for n in range(nread):
            v = 0
            for _ in range(8):
                v = v << 1 | clock(1)
            clock(1 if n == nread - 1 else 0)           # NACK the last
            out += bytes([v])
    pin(DAT, 0); pin(CLK, 1); pin(DAT, 1)               # stop
    return acks, out


def test_rv1103b(qemu, path):
    qt = QTest(qemu, 'rv1103b,flash-file=' + path + ',gc6160=70:75')
    try:
        # GC6160 (XM PTZ motor / IR cut) on bit-banged I2C, GPIO2 6 / 11
        acks, _ = gc6160_xfer(qt, [0x26, 0x16, 0x5a])
        acks2, out = gc6160_xfer(qt, [0x26, 0x16], 1)
        check(all(acks) and all(acks2) and out == b'\x5a',
              'rv1103b gc6160: register 0x16 written and read back (%s %s)'
              % (acks + acks2, out.hex()))
        acks, _ = gc6160_xfer(qt, [0x28, 0x00])
        check(not acks[0], 'rv1103b gc6160: no device at 0x28 (NACK)')
        check(qt.read(0, 16) == bytes(range(16)), 'rv1103b: SPL at DRAM 0')
        check(qt.readl(0x210F6000 + 0x10) == 3,
              'rv1103b: boot source at IRAM 0x210F6000 + 0x10')
        tags = read_atags(qt)
        ser = tags.get(ATAG_SERIAL, [0] * 9)
        check(ser[2] == 0x20540000 and ser[4] == 1500000,
              'rv1103b ATAG_SERIAL: UART0 0x20540000, 1500000')
        check(tags.get(ATAG_DDR_MEM, [0] * 5)[4] == 64 << 20,
              'rv1103b: 64 MiB in ATAG_DDR_MEM')
        check(tags.get(ATAG_SOC_INFO, [0] * 3)[1] == 0x1103,
              'rv1103b ATAG_SOC_INFO 0x1103')
        global SFC
        saved, SFC = SFC, 0x20D40000
        try:
            sfc_cmd(qt, 0x9f, length=3)
            jedec = qt.readl(SFC + 0x108) & 0xffffff
        finally:
            SFC = saved
        check(jedec == 0x1820c2, 'rv1103b FSPI: JEDEC ID %06x' % jedec)
        check(qt.readl(0x20680000 + 0x6f0) == 0x04000000, 'crypto v4')
        rng = 0x20690000
        qt.writel(rng + 0x10, 0xffff0000)
        qt.writel(rng + 0x10, 0xffff0008)                  # SW_DRNG_REQ
        check(qt.readl(rng + 0x14) & 8, 'rkrng STATE SW_DRNG_ACK')
        a = [qt.readl(rng + 0x70 + 4 * i) for i in range(8)]
        qt.writel(rng + 0x14, 8)
        check(not qt.readl(rng + 0x14) & 8, 'rkrng STATE write-1-to-clear')
        qt.writel(rng + 0x10, 0xffff0000)
        qt.writel(rng + 0x10, 0xffff0008)
        b = [qt.readl(rng + 0x70 + 4 * i) for i in range(8)]
        check(a != b and any(a), 'rkrng: two different 256-bit draws')
        global I2C4
        saved, I2C4 = I2C4, 0x20940000
        try:
            _, data, _ = i2c_reg_read(qt, 0x30, 0x3107, 2)
        finally:
            I2C4 = saved
        check(data == b'\x9b\x3a', 'rv1103b I2C4: sensor ID at 0x30')
        check(qt.readl(0x20D20000 + 0x6c) & 0xffff == 0x270a,
              'rv1103b SD controller at 0x20D20000')
        test_rk_dmac(qt)
        sai = 0x20660000
        check(qt.readl(sai + 0x70) == 0x24031103 and qt.readl(sai + 0x6c) & 2,
              'SAI VERSION 2403, STATUS FS idle')
        # HPMCU: held in reset (rockit's vmcu_probe insists), released by
        # a hiword write; its mailbox keeps A2B and posts no event before
        # an ISP frame
        check(qt.readl(0x20000A28) & 0xf0 == 0xf0,
              'rv1103b CRU SOFTRST(10): HPMCU held in reset')
        qt.writel(0x20000A28, 0x00f00000)
        check(qt.readl(0x20000A28) & 0xf0 == 0, 'HPMCU reset released')
        mb = 0x20A10000
        qt.writel(mb + 0x0c, 1)
        qt.writel(mb + 0x08, 3)                         # wrap start, pipe 1
        check(qt.readl(mb + 0x08) == 3 and qt.readl(mb + 0x0c) == 1,
              'HPMCU mailbox: A2B_CMD / A2B_DAT')
        check(qt.readl(mb + 0x18) == 0, 'HPMCU: no event before a frame')
    finally:
        qt.close()


def test_rk_dmac(qt):
    """rockchip-dma.c's way: a command entry whose LLP_NXT is the block."""
    D = 0x20740000
    cap0 = qt.readl(D + 0x30)
    check((cap0 & 0x3f) + 1 == 16 and ((cap0 >> 6) & 0x3f) + 1 == 8,
          'rk-dmac CAP0: 16 logical, 8 physical channels')
    cmd, lli, src, dst = 0x300000, 0x300040, 0x310000, 0x320000
    payload = bytes((i * 7 + 3) & 0xff for i in range(0x1000))
    qt.write(src, payload)
    qt.write(cmd, struct.pack('<16I', *([0] * 5 + [lli] + [0] * 10)))
    qt.write(lli, struct.pack('<16I', 1 | 2 | 8, 1 | 1 << 16 | 2 << 2, src,
                              dst, len(payload), 0, 0, 0, *[0] * 8))
    qt.writel(D + 0x04, 0x00050005)                     # CMN_CFG: en, IE
    L = D + 0x40 + 3 * 0x40                             # logical channel 3
    qt.writel(L + 0x00, 0x00010001)                     # CTL enable
    qt.writel(L + 0x18, 1)                              # IE: DMA done
    qt.writel(L + 0x08, cmd)
    qt.writel(L + 0x0c, 0x00010001)                     # TRF_CMD start
    check(not qt.readl(D + 0x18) & 8, 'rk-dmac: not done at the start write')
    qt.cmd('clock_step 1000000')
    check(qt.readl(D + 0x18) == 8 and qt.readl(L + 0x14) & 1,
          'rk-dmac: CMN_IS0 bit 3, LCH IS DMA done')
    check(qt.read(dst, len(payload)) == payload, 'rk-dmac: 4 KiB copied')
    pos = struct.unpack('<2I', qt.read(cmd + 8, 8))
    check(pos == (src + 0x1000, dst + 0x1000),
          'rk-dmac: the command entry holds the end position (residue)')
    qt.writel(L + 0x14, 1)
    qt.writel(D + 0x18, 8)
    check(qt.readl(D + 0x18) == 0, 'rk-dmac: IS write-1-to-clear')


def test_sai_files(qemu, path):
    # RV1103B SAI at 8 kHz stereo S16 from the CRU (mclk 2.048 MHz:
    # GPLL 1188 MHz x 64 / 37125, MDIV 4, FW 64): its DMA rate, packed
    # capture words from rx-file, playback into tx-file, and an RKDMA
    # capture block paced at the SAI's rate
    tmp = tempfile.mkdtemp()
    rx, tx = os.path.join(tmp, 'rx.wav'), os.path.join(tmp, 'tx.wav')
    pcm = b''.join(struct.pack('<hh', 100 * i, -100 * i) for i in range(64))
    with open(rx, 'wb') as f:
        f.write(b'RIFF' + struct.pack('<I', 36 + len(pcm)) + b'WAVEfmt ' +
                struct.pack('<IHHIIHH', 16, 1, 2, 8000, 32000, 4, 16) +
                b'data' + struct.pack('<I', len(pcm)) + pcm)
    qt = QTest(qemu, 'rv1103b,flash-file=' + path,
               ('-global', 'rk-sai.rx-file=' + rx,
                '-global', 'rk-sai.tx-file=' + tx))
    try:
        cru, sai = 0x20000000, 0x20660000
        qt.writel(cru + 0x60300 + 20 * 4, 0x001f0000)       # src = GPLL
        qt.writel(cru + 0x60300 + 21 * 4, 64 << 16 | 37125)  # the fraction
        qt.writel(cru + 0x60300 + 35 * 4, 0x0c000400)       # mclk: frac
        qt.writel(sai + 0x18, 3 << 3)                       # CKR MDIV 4
        qt.writel(sai + 0x04, 63)                           # FSCR FW 64
        qt.writel(sai + 0x08, 15 | 1 << 11)                 # RX S16, 2 ch
        qt.writel(sai + 0x00, 15 | 1 << 11)                 # TX the same
        words = [qt.readl(sai + 0x34) for _ in range(3)]
        want = [(100 * i & 0xffff) | (-100 * i & 0xffff) << 16
                for i in range(3)]
        check(words == want, 'SAI RXDR: S16 stereo frames, packed (%s)'
              % ['%08x' % w for w in words])
        qt.writel(sai + 0x10, 1 << 2)                       # XFER TXS
        for i in range(4):
            qt.writel(sai + 0x30, (i & 0xffff) | (-i & 0xffff) << 16)
        qt.writel(sai + 0x10, 0)                            # stop: synced
        # an RKDMA capture block from RXDR: 3200 bytes at 32000 B/s
        D, cmd, lli, dst = 0x20740000, 0x300000, 0x300040, 0x320000
        qt.writel(sai + 0x10, 0)
        qt.writel(sai + 0x10, 1 << 3)                       # XFER RXS
        qt.write(cmd, struct.pack('<16I', *([0] * 5 + [lli] + [0] * 10)))
        qt.write(lli, struct.pack('<16I', 1 | 2 | 8,
                                  2 << 2 | 1 << 16 | 2 << 18,
                                  sai + 0x34, dst, 3200, 0, 0, 2 << 8,
                                  *[0] * 8))
        qt.writel(D + 0x04, 0x00050005)
        L = D + 0x40 + 2 * 0x40
        qt.writel(L + 0x00, 0x00010001)
        qt.writel(L + 0x18, 1)
        qt.writel(L + 0x08, cmd)
        qt.writel(L + 0x0c, 0x00010001)
        qt.cmd('clock_step 90000000')                       # 90 ms
        early = qt.readl(L + 0x14) & 1
        qt.cmd('clock_step 20000000')                       # 110 ms
        check(not early and qt.readl(L + 0x14) & 1,
              'RKDMA from the SAI: 3200 bytes take 100 ms at 32000 B/s')
        frames = struct.unpack('<4h', qt.read(dst, 8))
        check(frames == (0, 0, 100, -100),
              'RKDMA from the SAI: the file\'s frames in memory %s'
              % (frames,))
    finally:
        qt.close()
    w = open(tx, 'rb').read() if os.path.exists(tx) else b''
    shutil.rmtree(tmp, ignore_errors=True)
    ok = (len(w) >= 44 + 16 and struct.unpack('<HI', w[22:28]) == (2, 8000)
          and struct.unpack('<8h', w[44:60]) ==
          (0, 0, 1, -1, 2, -2, 3, -3))
    check(ok, 'SAI tx-file: 8 kHz stereo WAV, the words unpacked')


def fdt(tree):
    """A flattened devicetree from {name: value | {subnodes}}: enough for a
    FIT's /images (strings and u32s)."""
    strings, struct_ = b'', b''
    def name_off(n):
        nonlocal strings
        i = strings.find(n.encode() + b'\0')
        if i < 0:
            i = len(strings)
            strings += n.encode() + b'\0'
        return i
    def node(name, props):
        nonlocal struct_
        nm = name.encode() + b'\0'
        struct_ += struct.pack('>I', 1) + nm + bytes(-len(nm) % 4)
        for k, v in props.items():
            if isinstance(v, dict):
                continue
            data = (v.encode() + b'\0') if isinstance(v, str) else \
                struct.pack('>I', v)
            struct_ += struct.pack('>III', 3, len(data), name_off(k))
            struct_ += data + bytes(-len(data) % 4)
        for k, v in props.items():
            if isinstance(v, dict):
                node(k, v)
        struct_ += struct.pack('>I', 2)
    node('', tree)
    struct_ += struct.pack('>I', 9)
    off_rsv = 40
    off_struct = off_rsv + 16
    off_strings = off_struct + len(struct_)
    total = off_strings + len(strings)
    hdr = struct.pack('>10I', 0xd00dfeed, total, off_struct, off_strings,
                      off_rsv, 17, 16, 0, len(strings), len(struct_))
    return hdr + bytes(16) + struct_ + strings


def test_rv1109(qemu, tmp):
    nand = os.path.join(tmp, 'nand.bin')
    uboot = b'UBOOT!' + bytes(range(64))
    fit = fdt({'description': 'test', 'images': {'uboot': {
        'os': 'U-Boot', 'type': 'standalone', 'load': 0x400000,
        'data-position': 0x1000, 'data-size': len(uboot)}}})
    with open(nand, 'wb') as f:                     # 256 MiB, data only
        f.truncate(0x10000000)
        f.seek(0x400000)                            # the uboot partition
        f.write(fit)
        f.seek(0x401000)
        f.write(uboot)
        f.seek(0x500000)
        f.write(b'PAGE@0x500000..' + bytes(range(256)))
    qt = QTest(qemu, 'rv1109,flash-file=' + nand)
    try:
        # the board's SN65DSI8x DSI -> LVDS bridge on I2C3 at 0x2c: its ID
        # register file ("DSI85" reversed) answers an 8-bit address read
        i2c3 = 0xFF520000
        qt.writel(i2c3 + 0x1c, 0xff)
        qt.writel(i2c3 + 0x08, (0x2c << 1) | (1 << 24))         # MRXADDR
        qt.writel(i2c3 + 0x0c, 0x00 | 1 << 24)                  # reg 0x00
        qt.writel(i2c3 + 0x00, 1 | 1 << 1 | 1 << 3 | 1 << 5 | 1 << 6)
        qt.writel(i2c3 + 0x14, 8)
        ipd = qt.readl(i2c3 + 0x1c)
        idb = struct.pack('<2I', qt.readl(i2c3 + 0x200),
                          qt.readl(i2c3 + 0x204))
        qt.writel(i2c3 + 0x00, qt.readl(i2c3) & ~(1 << 3) | 1 << 4)
        check(not ipd & 0x40 and idb == b'58ISD   ',
              'rv1109 I2C3 0x2c: SN65DSI8x ID %r' % idb)
        check(qt.read(0x400000, len(uboot)) == uboot,
              'rv1109 SPL stand-in: U-Boot from the FIT at its load address')
        stub = struct.unpack('<9I', qt.read(0, 36))
        check(stub[6] == 0x400000 and stub[8] == 0x400000,
              'entry stub: r1 = lr = U-Boot, no OP-TEE: straight to it')
        A = 0xFF5E0000                          # SARADC v1 (rk3399's)
        qt.writel(A + 0x08, 1 << 3 | 1 << 5 | 2)    # power up, irq, ch 2
        check(qt.readl(A + 0x08) & (1 << 6) and qt.readl(A + 0x00) == 0x3ff,
              'SARADC v1: conversion done at once, full scale (keys up)')
        qt.writel(A + 0x08, 0)
        check(not qt.readl(A + 0x08) & (1 << 6), 'SARADC v1: done cleared')
        check(qt.readl(0xFF700010) == 4, 'boot source SPI NAND at IRAM + 0x10')
        PMU = 0xFF3E0000
        qt.writel(PMU + 0x110, 0x00020002)          # power off ISPP (bit 1)
        check(qt.readl(PMU + 0x108) & 2, 'PMU: power status follows PWR')
        qt.writel(PMU + 0x0c4, 0x00040004)          # NPU idle request
        check(qt.readl(PMU + 0x0d0) & (1 << 18) and qt.readl(PMU + 0x0d8)
              & (1 << 18), 'PMU: 0xc4 idle request acked 16 bits up')
        st = qt.readl(PMU + 0x9c)
        check(st & 0x22 == 0x22 and not st & 0x11,
              'PMU core status: core 1 off, core 0 on (0x%x)' % st)
        T = 0xFF660000
        qt.writel(T + 0x00, 24000)                  # 1 ms at 24 MHz
        qt.writel(T + 0x10, 7)                      # enable, one shot, IRQ
        check(qt.readl(T + 0x08) <= 24000, 'rk-timer counts down')
        qt.cmd('clock_step 2000000')
        check(qt.readl(T + 0x18) == 1, 'rk-timer: INT_STATUS at 0')
        qt.writel(T + 0x18, 1)
        check(qt.readl(T + 0x18) == 0, 'rk-timer: INT_STATUS W1C')
        check(qt.readl(0xFFC80000) & 0x200 and qt.readl(0xFFC80800) == 0xff,
              'parallel NANDC: ready, no chip (FF)')
        for O, who in ((0xFF5C0000, 'OTP'), (0xFF5D0000, 'secure OTP')):
            qt.writel(O + 0x00, 0)                      # CEB
            check(qt.readl(O + 0x18) & 3 == 1, '%s: CEB done' % who)
            qt.writel(O + 0x04, 1)                      # RSTB
            check(qt.readl(O + 0x18) & 0xf == 0xd,
                  '%s: out of reset, ST 0xd as OP-TEE waits for' % who)
        O = 0xFF5C0000
        qt.writel(O + 0x34, 0x10 * 8 + 1)               # program bits 1, 3
        qt.writel(O + 0x38, 3 - 1)
        qt.writel(O + 0x3c, 0b101)
        qt.writel(O + 0x44, 1)
        qt.writel(O + 0x1c, 0x10)
        qt.writel(O + 0x20, 1)
        check(qt.readl(O + 0x30) == 0 and qt.readl(O + 0x24) == 0x0a,
              'OTP: programmed bits read back (0x%x)' % qt.readl(O + 0x24))
        M = 0xFFB00F00                              # the VOP's IOMMU
        qt.writel(M + 0x00, 0xCAFEBABE)
        check(qt.readl(M) == 0xCAFEB000, 'IOMMU: DTE_ADDR reads back masked')
        qt.writel(M + 0x08, 6)                      # force reset
        check(qt.readl(M) == 0, 'IOMMU: force reset clears DTE_ADDR')
        qt.writel(M + 0x08, 0)                      # enable paging
        qt.writel(M + 0x08, 2)                      # enable stall
        st = qt.readl(M + 0x04)
        check(st & 0x1d == 0x1d and not st >> 31,
              'IOMMU STATUS: paging, stalled, idle (0x%x)' % st)
        caps = qt.readl(0xFFE00000)
        check(caps >> 16 == 0x100 and caps & 0xff,
              'EHCI at 0xFFE00000: HCIVERSION 1.00 (0x%x)' % caps)
        check(qt.readl(0xFFE10000) & 0xff == 0x10, 'OHCI revision 1.0')
        check(qt.readl(0xFF4C0034) & 0x10, 'USB2 PHY: Rterm calibration done')
        N = 0xFFBC0000                              # the NPU (Vivante)
        check([qt.readl(N + o) for o in (0x20, 0x24, 0x30, 0xa8, 0xe8)] ==
              [0x8000, 0x8003, 0xb5, 0x05080009, 0x04000000],
              'NPU identity: galcore feature database entry')
        check(qt.readl(N) >> 16 & 3 == 3, 'NPU: IDLE_3D / IDLE_2D after reset')
        cb = 0x08000000                             # LOAD_STATE event 5, END
        qt.write(cb, struct.pack('<4I', 0x08010e01, 5, 0x10000000, 0))
        qt.writel(N + 0x14, 0xffffffff)
        qt.writel(N + 0x654, cb)
        qt.writel(N + 0x658, 1 << 16)
        qt.cmd('clock_step 100000')
        check(qt.readl(N + 0x10) == 1 << 5 and qt.readl(N + 0x10) == 0,
              'NPU front end: AQ_EVENT 5 pending, cleared by the read')
        check(qt.readl(N + 0x664) == cb + 8, 'NPU front end stopped at END')
        global I2C4
        saved, I2C4 = I2C4, 0xFF510000              # I2C1: the sensor
        try:
            _, sid, _ = i2c_reg_read(qt, 0x1a, 0x311a, 1)
        finally:
            I2C4 = 0xFF3F0000                       # I2C0: the RTC
        try:
            qt.writel(I2C4 + 0x1c, 0xff)            # 8-bit register 0x02
            qt.writel(I2C4 + 0x08, (0x51 << 1) | (1 << 24))
            qt.writel(I2C4 + 0x0c, 0x02 | 1 << 24)
            qt.writel(I2C4 + 0x00, 1 | 1 << 1 | 1 << 3 | 1 << 5 | 1 << 6)
            qt.writel(I2C4 + 0x14, 7)
            t = struct.pack('<2I', qt.readl(I2C4 + 0x200),
                            qt.readl(I2C4 + 0x204))[:7]
            qt.writel(I2C4 + 0x00, 1 | 1 << 4)
        finally:
            I2C4 = saved
        check(sid == b'\xe0', 'rv1109 board: IMX415 at 0x1a, chip ID 0xE0')
        # the camera: CIF frames while the IMX415 streams, ISP20 read back,
        # the ISPP's pictures, the RGA's copy of one
        C = 0xFFAE0000
        qt.writel(C + 0x124, 0xffff)                # INTEN: FS / FE
        qt.writel(C + 0x84, 3840 | 2160 << 16)      # ID0_CTRL1
        qt.writel(C + 0x80, 1)                      # ID0 capture on
        qt.cmd('clock_step 100000000')
        check(qt.readl(C + 0x128) == 0,
              'CIF: no frames before the sensor streams')
        saved, I2C4 = I2C4, 0xFF510000
        try:
            i2c_reg_write(qt, 0x1a, 0x3000, 0)      # IMX415 out of standby
            for r, v in ((0x3024, 0xd0), (0x3025, 0x07), (0x3026, 0),
                         (0x3050, 0xe8), (0x3051, 0x03), (0x3052, 0)):
                i2c_reg_write(qt, 0x1a, r, v)       # VMAX 2000, SHR0 1000
        finally:
            I2C4 = saved
        qt.cmd('clock_step 100000000')              # two and a half frames
        st = qt.readl(C + 0x128)
        check(st & 0x303 == 0x303 and qt.readl(C + 0x14c) >= 2 and
              qt.readl(C + 0x134) & 0x3fff == 2160,
              'CIF: frame 0 / 1 start and end at 25 fps, frame number, '
              'lines (0x%x)' % st)
        qt.writel(C + 0x128, st)
        check(qt.readl(C + 0x128) == 0, 'CIF: INTSTAT write 1 to clear')
        qt.writel(C + 0x80, 0)
        P = 0xFFB50000
        qt.writel(P + 0x59c, 64)                    # ISP out 64 x 32
        qt.writel(P + 0x5a0, 32)
        qt.writel(P + 0x5bc, 0x62)                  # V_START, FRAME_IN, FRAME
        qt.writel(P + 0x14f8, 1 << 31)              # MPFBC frame
        qt.writel(P + 0x18c0, 1)                    # FBC on: the bridge
        qt.writel(P + 0x5d0, 1)
        qt.writel(P + 0x4400, 1)                    # RAWAE big 1 on
        qt.writel(P + 0x1c00, 0x1501)               # read back, 2 passes
        qt.cmd('clock_step 3000000')
        check(qt.readl(P + 0x5c0) & 0x40 and not qt.readl(P + 0x14fc),
              'ISP20: first pass, V_START, no MI end yet')
        qt.cmd('clock_step 10000000')
        check(qt.readl(P + 0x5c0) & 0x62 == 0x62 and
              qt.readl(P + 0x14fc) == 1 << 31 and qt.readl(P + 0x5d4) & 1 and
              qt.readl(P + 0x4400) >> 31,
              'ISP20 read back: frame ends, MPFBC_FRAME, RAWAE done')
        Q = 0xFFB60000
        Y0, UV0, Y1, UV1 = 0x08100000, 0x08100800, 0x08200000, 0x08200400
        qt.write(Y1, bytes(32 * 18))
        qt.writel(Q + 0x020, 0x7ff)                 # INT_MSK
        qt.writel(Q + 0x014, 32 << 16 | 64)         # CTRL_SIZE 64 x 32
        qt.writel(Q + 0x604, 64 // 4)               # SHARP_WR stride
        qt.writel(Q + 0x608, Y0)
        qt.writel(Q + 0x60c, UV0)
        qt.writel(Q + 0x800, 1)                     # SCL0 on, NV12
        qt.writel(Q + 0x804, 32 // 4)
        qt.writel(Q + 0x808, Y1)
        qt.writel(Q + 0x80c, UV1)
        qt.writel(Q + 0x818, ((15 * 8192) // 31 + 1) << 16 |
                  ((31 * 8192) // 63 + 1))          # to 32 x 16
        qt.writel(Q + 0x000, 2)                     # NR + SHP start
        check(qt.readl(Q + 0x03c) & 0x13 == 0x12, 'ISPP: NR, SHP, SCL0 working')
        qt.cmd('clock_step 4000000')
        raw = qt.readl(Q + 0x024)
        check(raw & 0x118 == 0x118 and qt.readl(Q + 0x028) == raw and
              qt.readl(Q + 0x03c) == 0,
              'ISPP: NR_INT, SHP_INT, SCL0_INT, idle (0x%x)' % raw)
        qt.writel(Q + 0x030, raw)
        check(qt.readl(Q + 0x028) == 0, 'ISPP: INT_CLR')
        y = qt.read(Y0, 64 * 32)
        check(y[0] < y[31 * 64] and qt.read(UV0, 64)[0] in range(120, 137),
              'ISPP main picture: the scene, darker at the top, grey chroma '
              '(%d, %d)' % (y[0], y[31 * 64]))
        s1 = qt.read(Y1, 32 * 18)
        check(s1[0] and s1[15 * 32] and not s1[16 * 32],
              'ISPP SCL0: 32 x 16 from the factor, nothing past it')
        G = 0xFFAF0000
        cmd, D = 0x08300000, 0x08400000
        c = [0] * 32
        c[0x04 // 4] = 0xa | 1 << 8                 # NV12, BT.601 limited
        c[0x08 // 4], c[0x0c // 4] = Y0, UV0
        c[0x18 // 4] = 64 // 4
        c[0x1c // 4] = 63 | 31 << 16
        c[0x38 // 4] = 0                            # RGBA8888
        c[0x3c // 4] = D
        c[0x48 // 4] = 32
        c[0x4c // 4] = 31 | 15 << 16                # scaled to 32 x 16
        qt.write(cmd, struct.pack('<32I', *c))
        qt.writel(G + 0x010, 0x700)                 # done interrupts on
        qt.writel(G + 0x008, cmd)
        qt.writel(G + 0x004, 1)
        qt.cmd('clock_step 1000000')
        check(qt.readl(G + 0x010) & 4, 'RGA2: all commands done')
        px = qt.read(D, 4)
        check(px[3] == 255 and abs(px[0] - px[1]) < 4 and
              abs(px[1] - px[2]) < 4 and 0 < px[0] < 128,
              'RGA2 NV12 -> RGBA, scaled: grey of the scene (%s)' % px.hex())
        qt.writel(G + 0x010, 0x740)                 # clear
        check(not qt.readl(G + 0x010) & 4, 'RGA2: INT cleared by bit 6')
        c = [0] * 32
        c[0], c[0x2c // 4] = 2, 0xff0000ff          # colour fill, red
        c[0x3c // 4], c[0x48 // 4], c[0x4c // 4] = D, 32, 31 | 15 << 16
        qt.write(cmd, struct.pack('<32I', *c))
        qt.writel(G + 0x004, 1)
        qt.cmd('clock_step 1000000')
        check(qt.read(D + 4 * 32 * 15 + 4 * 31, 4) == b'\xff\x00\x00\xff',
              'RGA2 colour fill: the last pixel red')
        # VEPU541: the VEPU540C model at the RV1126's offsets (IOMMU off)
        V = 0xFFBB0000
        src, bs = 0x08500000, 0x08600000
        w, h = 64, 16
        qt.write(src, bytes((16 + x + r) & 0xff for r in range(h)
                            for x in range(w)))
        qt.write(src + w * h, bytes([128]) * (w * h // 2))
        for off, v in {
            0x118: src, 0x11c: src + w * h, 0x114: w | w << 16,
            0x128: 0x08700000, 0x130: 0x08700000,
            0x14c: bs + 0xffff, 0x150: bs, 0x154: bs, 0x158: bs,
            0x034: 1 | 30 << 8,                     # H.265, QP 30
            0x030: (w // 8 - 1) | (h // 8 - 1) << 16,
            0x044: 6 << 2,                          # NV12
            0x19c: 19, 0x1a8: 2 << 15, 0x1ac: 30 << 16,
            0x010: 1,
        }.items():
            qt.writel(V + off, v)
        qt.writel(V + 0x004, 0x100)
        qt.cmd('clock_step 2000000')
        n = qt.readl(V + 0x210)
        check(qt.readl(V + 0x01c) & 1 and n > 0 and
              qt.readl(bs) == 0x01000000 and
              qt.readl(V + 0x218) == 30 * (w // 8) * (h // 8),
              'VEPU541: H.265 IDR, %d bytes, QP sum at 0x218' % n)
        qt.writel(V + 0x018, 1)
        check(qt.readl(V + 0x01c) == 0, 'VEPU541: INT_CLR at 0x018')
        # its OSD: region 0, the left 16 x 16, palette entry 1 opaque
        # Y 235 (Y 7:0, U 15:8, V 23:16, alpha 31:24)
        osd = 0x08780000
        qt.write(osd, bytes([1]) * 256)
        qt.writel(V + 0x400 + 4, 235 | 128 << 8 | 128 << 16 | 255 << 24)
        qt.writel(V + 0x1d0, 0)                 # (0, 0) - (0, 0) blocks
        qt.writel(V + 0x1f0, osd)
        qt.writel(V + 0x1c0, 1)
        qt.writel(V + 0x004, 0x100)
        qt.cmd('clock_step 2000000')
        n = qt.readl(V + 0x210)
        qt.writel(V + 0x018, 1)
        qt.writel(V + 0x1c0, 0)
        if shutil.which('ffmpeg'):
            import subprocess
            r = subprocess.run(['ffmpeg', '-v', 'error', '-f', 'hevc', '-i',
                                '-', '-f', 'rawvideo', '-pix_fmt', 'gray',
                                '-'], input=qt.read(bs, n),
                               capture_output=True)
            g = r.stdout[:w * h]
            osd_y = [g[r_ * w + x] for r_ in range(16) for x in range(16)]
            rest = [g[r_ * w + x] for r_ in range(16) for x in range(32, 64)]
            check(len(g) == w * h and min(osd_y) > 215 and max(rest) < 120,
                  'VEPU541 OSD: the region drawn opaque into the picture '
                  '(%s .. %s)' % (min(osd_y or [0]), max(rest or [0])))
        else:
            print('skip VEPU541 OSD decode: no ffmpeg')
        test_pwm(qt, 0xFF430000, 'rv1109')
        test_vepu2_jpeg(qt)
        check(qt.readl(0xFFD0C120) == 0x5533330a and
              qt.readl(0xFFD00000) >> 16 == 0x100,
              'rv1109 OTG DWC3: GSNPSID, its xHCI at 0xFFD00000')
        import datetime
        now = datetime.datetime.now(datetime.timezone.utc)
        yy = (t[6] >> 4) * 10 + (t[6] & 15)
        check(t[0] < 0x60 and yy == now.year % 100 and t[5] & 0x1f,
              'BM8563 / PCF8563 at 0x51 on I2C0: the calendar in BCD (%s)'
              % t.hex())
        # I2S0 RX sample clock from the CRU: xin12m / 4 / 64 = 46875 frames
        # a second, 16-bit stereo packed: a word a frame
        I = 0xFF800000
        qt.writel(0xFF490000 + 0x100 + 30 * 4, 0x000c000c)  # RX mux: xin12m
        qt.writel(I + 0x38, 0x0300)                 # CLKDIV RXM: 4
        qt.writel(I + 0x08, 0x3f00)                 # CKR RSD: 64
        qt.writel(I + 0x04, 0x0f)                   # RXCR: VDW 16, 2 ch
        qt.writel(I + 0x1c, 2)                      # XFER: RX start
        qt.cmd('clock_step 400000')                 # 0.4 ms: ~18.75 words
        lvl = qt.readl(I + 0x2c)
        check(14 <= lvl <= 22, 'I2S RX FIFO fills at the CRU-derived rate '
              '(%d words in 0.4 ms, 18.75 expected)' % lvl)
        qt.writel(I + 0x1c, 0)
        V = 0xFFB00000
        qt.cmd('clock_step 50000000')
        check(qt.readl(V + 0x3c) == 0, 'VOP: nothing latches while disabled')
        qt.writel(V + 0x18, 0)                      # out of standby
        qt.writel(V + 0x34, 0x00010001)             # frame start enabled
        qt.cmd('clock_step 20000000')               # > one 60 Hz frame
        check(qt.readl(V + 0x3c) == 1, 'VOP: frame start each frame, enabled '
              'bit only (0x%x)' % qt.readl(V + 0x3c))
        qt.writel(V + 0x38, 0x00010001)
        check(qt.readl(V + 0x3c) == 0, 'VOP: INTR_CLEAR (hiword)')
        R = 0xFF500400
        qt.writel(R, 0xffff0033)                    # 256 bits, enable, start
        check(not qt.readl(R) & 1, 'crypto-v2 RNG: START clears')
        a = [qt.readl(R + 0x10 + 4 * i) for i in range(8)]
        qt.writel(R, 0xffff0033)
        b = [qt.readl(R + 0x10 + 4 * i) for i in range(8)]
        check(a != b and any(a), 'crypto-v2 RNG: two different draws')
        global SFC
        saved, SFC = SFC, 0xFFC90000
        try:
            sfc_cmd(qt, 0x9f, addr_bytes=0, length=3)
            nid = qt.readl(SFC + 0x108) & 0xffffff
            sfc_cmd(qt, 0x13, addr_bytes=3, length=0)   # page read: row 0xa00
            qt.writel(SFC + 0x104, 0xa00)
            sfc_cmd(qt, 0x03, addr_bytes=3, length=16)  # column + dummy byte
            qt.writel(SFC + 0x104, 0)
            data = struct.pack('<4I', *[qt.readl(SFC + 0x108)
                                        for _ in range(4)])
        finally:
            SFC = saved
        check(nid & 0xffff == 0xd2c8 or (nid >> 8) & 0xffff == 0xd2c8,
              'SPI NAND ID c8 d2 through the SFC (0x%06x)' % nid)
        check(data == b'PAGE@0x500000..\x00',
              'SPI NAND page read, dummy as an address byte (%r)' % data)
    finally:
        qt.close()


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
    spl, data, pattern = make_image(path)
    qt = QTest(args.qemu, 'rv1106,flash-file=' + path)
    try:
        test_boot_rom(qt, spl, data, 128 << 20)
        test_sfc(qt, pattern)
        test_crypto(qt)
        test_decom(qt)
        test_syscon(qt)
        test_uart(qt)
        test_i2s_rv1106(qt)
        test_pwm(qt, 0xFF350000, 'rv1106')
        U = 0xFFB00000                          # USB OTG: DWC3 + xHCI
        check(qt.readl(U + 0xc120) == 0x5533330a and
              qt.readl(U + 0xc140) & 7 == 1,
              'DWC3: GSNPSID 0x5533330a, a host-only core (GHWPARAMS0)')
        cap = qt.readl(U)
        check(cap & 0xff and cap >> 16 == 0x100,
              'DWC3 xHCI at its base: CAPLENGTH, HCIVERSION 1.00 (0x%x)' % cap)
        test_rknpu(qt)
        test_vepu_pp(qt)
        test_isp32(qt)
        test_isp32_colour(qt)
        test_vicap_dma(qt)
        test_vepu540c(qt)
        test_vepu540c_jpeg(qt)
        test_vepu540c_osd(qt)
        test_vepu540c_hevc_detail(qt)
        test_vepu540c_h264(qt)
        test_i2c(qt)
        test_gpio(qt)
        test_trng(qt)
        test_rtc(qt)
        test_misc(qt)
    finally:
        qt.close()
    test_i2s_tone(args.qemu, path)
    test_i2s_files(args.qemu, path, tmp)
    test_dwc3_gadget(args.qemu, path)
    test_refused(args.qemu, tmp)
    test_machines(args.qemu, path)
    test_gpio_high(args.qemu, path)
    test_tb_isp(args.qemu, path)
    test_rv1103b(args.qemu, path)
    test_pwm_output(args.qemu, path)
    test_sd_write(args.qemu, path)
    test_sai_files(args.qemu, path)
    test_rv1109(args.qemu, tmp)
    print('%s: %d failure(s)' % ('FAIL' if failures else 'PASS', failures))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
