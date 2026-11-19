#!/usr/bin/env python3
"""Capture the ISP tuning a Xiongmai App applies to a sensor, by running the
App's own ISP code under Unicorn.

    xm-isp-capture.py --app App --profile profiles/<build>.json \
        --layout build/layout.json --base base/ --sensor SC465SL \
        --iso 25600 --out day/

The App (the vendor's /usr/bin/App from a firmware dump) links the XMedia SDK
statically and keeps its per-sensor tables in Xiongmai's own layout, behind
LibXmVideoParam_*_getParam(). Rather than decode each table, this runs the
App's ISP monitor thread from its entry: its init, its tables, its first
monitor loops. Every libc import is a Python stub, every call into the
statically linked SDK returns 0, and the SDK's attribute setters and getters
(named in the profile) are caught: a setter's structure is recorded, a getter
is answered with the last value set for that block, or else the camera's own
(--base, from `attrtool dump`). The result is what the App would have put in
force on the camera, block by block, as the SDK's own structures -- ready for
`attrtool apply` and a PQ bin export.

The AE and AWB readings the App's monitor reacts to are synthetic: ISO
--iso, colour temperature --ct. --then ADDR:ARG calls another App function
after the monitor stops: the profile's apply_day_night with 0 applies the
App's IR night set, with 1 its day set.
"""
import argparse
import json
import os
import struct
import sys

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_MODE_ARM, UC_HOOK_CODE, \
    UC_HOOK_MEM_UNMAPPED
from unicorn.arm_const import *

STUB, HEAP, STACK = 0xe0000000, 0xd0000000, 0xc0000000
END = STUB + 0xfff0
SLEEPERS = ('usleep', 'sleep', 'nanosleep', 'pthread_cond_timedwait',
            'select', 'sem_timedwait')


def num(s):
    return int(s, 0) if isinstance(s, str) else s


class Capture:
    def __init__(self, a):
        self.a = a
        self.prof = json.load(open(a.profile))
        self.layout = json.load(open(a.layout))
        self.sizes = self.layout['sizes']
        self.off = self.layout['offsets']
        self.sdk = {num(k): v for k, v in self.prof['sdk'].items()}
        self.sdk_lo, self.sdk_hi = (num(x) for x in self.prof['sdk_range'])
        self.force = {num(k): v for k, v in self.prof['force_return'].items()}
        self.calls, self.state, self.sleeps, self.last_get = [], {}, 0, ''
        self.unknown = {}
        self.heap = HEAP + 0x1000
        self.load()

    # --- memory ------------------------------------------------------------
    def malloc(self, n):
        p = (self.heap + 15) & ~15
        self.heap = p + max(n, 1)
        return p

    def rd(self, p, n):
        return bytes(self.mu.mem_read(p, n))

    def cstr(self, p):
        s = b''
        while True:
            c = self.rd(p, 64)
            i = c.find(b'\0')
            if i >= 0:
                return s + c[:i]
            s, p = s + c, p + 64

    def string(self, s):
        p = self.malloc(len(s) + 1)
        self.mu.mem_write(p, s.encode() + b'\0')
        return p

    def load(self):
        f = open(self.a.app, 'rb')
        elf = ELFFile(f)
        mu = self.mu = Uc(UC_ARCH_ARM, UC_MODE_ARM)
        for seg in elf.iter_segments():
            if seg['p_type'] != 'PT_LOAD':
                continue
            lo = seg['p_vaddr'] & ~0xfff
            hi = (seg['p_vaddr'] + seg['p_memsz'] + 0xfff) & ~0xfff
            mu.mem_map(lo, hi - lo)
            mu.mem_write(seg['p_vaddr'], seg.data())
        mu.mem_map(STUB, 0x10000)
        mu.mem_map(HEAP, 0x4000000)
        mu.mem_map(STACK - 0x200000, 0x200000)
        # Every PLT import goes to a stub address; the hook runs it.
        dynsym = elf.get_section_by_name('.dynsym')
        self.stubs = {}
        for i, r in enumerate(elf.get_section_by_name('.rel.plt').iter_relocations()):
            self.stubs[STUB + 4 * i] = dynsym.get_symbol(r['r_info_sym']).name
            mu.mem_write(r['r_offset'], struct.pack('<I', (STUB + 4 * i) | 1))
        for r in elf.get_section_by_name('.rel.dyn').iter_relocations():
            if r['r_info_type'] == 21:  # R_ARM_GLOB_DAT
                v = dynsym.get_symbol(r['r_info_sym'])['st_value'] or self.malloc(64)
                mu.mem_write(r['r_offset'], struct.pack('<I', v))
        # VI pipe 0 configured, pipe 1 not: the monitor waits for it.
        pt, stride = num(self.prof['pipe_table']), self.prof['pipe_stride']
        mu.mem_write(pt, struct.pack('<i', 0))
        mu.mem_write(pt + stride, struct.pack('<i', -1))
        self.strings = {
            'LibXmDvr_System_getSensorName': self.string(self.a.sensor),
            'LibXmDvr_System_getChipName': self.string(self.prof['chip']),
            'LibXmDvr_System_getProductName': self.string(self.prof['product']),
            # No saved calibration: the App falls back to its tables.
            'LibXmDvr_System_getWritableDir': self.string('/nonexistent'),
        }
        mu.hook_add(UC_HOOK_CODE, self.on_stub, begin=STUB, end=STUB + 0x10000)
        mu.hook_add(UC_HOOK_CODE, self.on_sdk, begin=self.sdk_lo, end=self.sdk_hi)
        for p in self.force:
            mu.hook_add(UC_HOOK_CODE, self.on_force, begin=p, end=p)
        mu.hook_add(UC_HOOK_MEM_UNMAPPED, self.on_bad)
        # VFP on: the App is hard-float.
        mu.reg_write(UC_ARM_REG_C1_C0_2, mu.reg_read(UC_ARM_REG_C1_C0_2) | (0xf << 20))
        mu.reg_write(UC_ARM_REG_FPEXC, 0x40000000)

    # --- synthetic readings -------------------------------------------------
    def exp_info(self):
        o, iso = self.off['xmedia_ae_exp_info'], self.a.iso
        b = bytearray(self.sizes['xmedia_ae_exp_info'])
        et = 10000 if iso <= 400 else 40000
        gain = max(1024, iso * et // 10000 * 1024 // 100)
        struct.pack_into('<I', b, o['exp_time'], et)
        struct.pack_into('<I', b, o['again'], min(gain, 1024 * 64))
        struct.pack_into('<I', b, o['dgain'], max(1024, gain // 64))
        struct.pack_into('<I', b, o['isp_dgain'], 1024)
        struct.pack_into('<I', b, o['exposure'], et * iso // 100)
        b[o['avg_luma']] = 56
        struct.pack_into('<I', b, o['fps'], 2500)
        for f in ('iso', 'iso_sf', 'iso_calibrate'):
            struct.pack_into('<I', b, o[f], iso)
        return bytes(b)

    def awb_info(self):
        o = self.off['xmedia_awb_info']
        b = bytearray(self.sizes['xmedia_awb_info'])
        struct.pack_into('<H', b, o['rgain'], 400)
        struct.pack_into('<H', b, o['grgain'], 256)
        struct.pack_into('<H', b, o['gbgain'], 256)
        struct.pack_into('<H', b, o['bgain'], 500)
        struct.pack_into('<H', b, o['saturation'], 128)
        struct.pack_into('<H', b, o['ct'], self.a.ct)
        struct.pack_into('<H', b, o['ls0_ct'], self.a.ct)
        return bytes(b)

    # --- hooks ---------------------------------------------------------------
    def ret(self, v):
        self.mu.reg_write(UC_ARM_REG_R0, v & 0xffffffff)
        self.mu.reg_write(UC_ARM_REG_PC, self.mu.reg_read(UC_ARM_REG_LR))

    def libc(self, name, r0, r1, r2):
        mu = self.mu
        if name in ('memcpy', 'memmove'):
            if r2:
                mu.mem_write(r0, self.rd(r1, r2))
            return r0
        if name == 'memset':
            if r2:
                mu.mem_write(r0, bytes([r1 & 0xff]) * r2)
            return r0
        if name == 'malloc':
            return self.malloc(r0)
        if name == 'calloc':
            p = self.malloc(r0 * r1)
            mu.mem_write(p, b'\0' * (r0 * r1))
            return p
        if name == 'strlen':
            return len(self.cstr(r0))
        if name == 'strcpy':
            mu.mem_write(r0, self.cstr(r1) + b'\0')
            return r0
        if name == 'strncpy':
            s = self.cstr(r1)[:r2]
            mu.mem_write(r0, s + b'\0' * (r2 - len(s)))
            return r0
        if name == 'strcat':
            mu.mem_write(r0 + len(self.cstr(r0)), self.cstr(r1) + b'\0')
            return r0
        if name in ('strcmp', 'strcasecmp', 'strncmp'):
            x, y = self.cstr(r0), self.cstr(r1)
            if name == 'strcasecmp':
                x, y = x.lower(), y.lower()
            if name == 'strncmp':
                x, y = x[:r2], y[:r2]
            return (x > y) - (x < y)
        if name == 'strstr':
            i = self.cstr(r0).find(self.cstr(r1))
            return 0 if i < 0 else r0 + i
        if name == 'strchr':
            i = self.cstr(r0).find(bytes([r1 & 0xff]))
            return 0 if i < 0 else r0 + i
        if name in self.strings:
            return self.strings[name]
        if name in SLEEPERS:
            self.sleeps += 1
            if self.sleeps >= self.a.sleeps:
                mu.emu_stop()
            return 0
        if name == 'time':
            return 1700000000
        if name in ('exit', 'abort', '__stack_chk_fail'):
            print('stopped at', name, file=sys.stderr)
            mu.emu_stop()
        return 0

    def on_stub(self, mu, addr, size, _):
        name = self.stubs.get(addr & ~1, '?')
        regs = [mu.reg_read(r) for r in (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2)]
        try:
            v = self.libc(name, *regs)
        except UcError as e:
            print('%s(%s) failed: %s' % (name, ', '.join(map(hex, regs)), e), file=sys.stderr)
            v = 0
        self.ret(v)

    def on_sdk(self, mu, addr, size, _):
        r0, r1 = mu.reg_read(UC_ARM_REG_R0), mu.reg_read(UC_ARM_REG_R1)
        e = self.sdk.get(addr & ~1)
        if not e:
            self.unknown[addr] = self.unknown.get(addr, 0) + 1
            return self.ret(0)
        kind, name, typ = e
        n = self.sizes.get(typ, 0) if typ else 0
        if kind == 'get' and n and r1:
            if name == 'ae_exp_info':
                data = self.exp_info()
            elif name == 'awb_info':
                data = self.awb_info()
            elif name in self.state:
                data = self.state[name]
            elif self.a.base and os.path.exists('%s/%s.bin' % (self.a.base, name)):
                data = open('%s/%s.bin' % (self.a.base, name), 'rb').read()
            else:
                data = b'\0' * n
            mu.mem_write(r1, data[:n])
            self.last_get = name
        elif kind == 'set':
            if typ is None:
                # One setter the log strings could not tell apart: it is the
                # block the App last read.
                if self.last_get in ('awb_scene_adj', 'awb_color_sector'):
                    name = self.last_get
                    typ = {'awb_scene_adj': 'xmedia_awb_scene_adj_attr',
                           'awb_color_sector': 'xmedia_color_sector_attr'}[name]
                    n = self.sizes[typ]
            if r1 and n:
                self.state[name] = self.rd(r1, n)
            self.calls.append({'name': name, 'pipe': r0, 'size': n})
        self.ret(0)

    def on_force(self, mu, addr, size, _):
        self.ret(self.force[addr])

    def on_bad(self, mu, access, addr, size, value, _):
        print('unmapped %#x at pc %#x' % (addr, mu.reg_read(UC_ARM_REG_PC)), file=sys.stderr)
        return False

    # --- run -------------------------------------------------------------------
    def call(self, entry, arg=None):
        mu = self.mu
        self.sleeps = 0
        mu.reg_write(UC_ARM_REG_SP, STACK - 0x1000)
        mu.reg_write(UC_ARM_REG_LR, END)
        if arg is not None:
            mu.reg_write(UC_ARM_REG_R0, arg)
        try:
            mu.emu_start(entry, END, count=200_000_000)
        except UcError as e:
            print('stopped: %s at %#x' % (e, mu.reg_read(UC_ARM_REG_PC)), file=sys.stderr)

    def run(self):
        self.call(num(self.prof['monitor_entry']))
        for spec in self.a.then:
            p, arg = (int(x, 0) for x in spec.split(':'))
            self.calls.append({'name': 'phase', 'entry': hex(p), 'arg': arg})
            self.call(p, arg)
        os.makedirs(self.a.out, exist_ok=True)
        for name, data in self.state.items():
            open('%s/%s.bin' % (self.a.out, name), 'wb').write(data)
        json.dump({'calls': self.calls,
                   'unknown_sdk_calls': {hex(k): v for k, v in self.unknown.items()}},
                  open('%s/calls.json' % self.a.out, 'w'), indent=1)
        print('%d set calls, %d blocks in %s' % (
            sum(1 for c in self.calls if c['name'] != 'phase'), len(self.state),
            self.a.out), file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--app', required=True, help="the vendor's /usr/bin/App")
    ap.add_argument('--profile', required=True, help='addresses for that App build')
    ap.add_argument('--layout', required=True, help='SDK structure sizes (build.sh)')
    ap.add_argument('--base', help="the camera's own blocks, from `attrtool dump`")
    ap.add_argument('--sensor', required=True, help='as the App names it: SC465SL')
    ap.add_argument('--iso', type=int, default=100)
    ap.add_argument('--ct', type=int, default=5000)
    ap.add_argument('--sleeps', type=int, default=120,
                    help='monitor sleeps before stopping')
    ap.add_argument('--then', action='append', default=[], metavar='ADDR:ARG')
    ap.add_argument('--out', required=True)
    Capture(ap.parse_args()).run()


if __name__ == '__main__':
    main()
