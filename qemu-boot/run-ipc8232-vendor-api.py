#!/usr/bin/env python3
"""Boot the IPC8232SWC-WE vendor twin and force AgentGreen's local /api/v2 on.

Mirrors the real-camera non-destructive unlock, automated over the emulated
serial console:
  U-Boot -> rdinit=/bin/sh -> export AG_FORCE_DIRECT_ACCESS=1 -> exec /sbin/init

QEMU user-net forwards host ports to the guest's API/RTSP so the twin is
reachable exactly like the unlocked camera (admin:admin54321).

Run with: uv run --with pexpect python3 run-ipc8232-vendor-api.py
Then from the host:  curl -s http://127.0.0.1:18080/api/v2/device/info
"""
import os
import sys
import time

import pexpect

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QEMU = os.path.join(REPO, "qemu-src/build/qemu-system-arm")
FLASH = os.environ.get(
    "FLASH", os.path.join(REPO, "vendor-dumps/ipc8232swc-we-vendor-nand-128mib-20261119.bin"))
# vendor kernel cmdline (from a stock boot) + rdinit for the pre-init shell
BOOTARGS = ("mem=70M console=ttyAMA0,115200 rootfstype=ramfs "
            "mtdparts=hinand:1024K(boot),1024K(tech),8192K(kernel),"
            "16384K(app),-(config) hw_type=1202 ubi.mtd=3,2048 rdinit=/bin/sh")

# A unique throwaway copy per run (U-Boot saveenv / UBI write back to it), so
# concurrent runs don't clobber each other; removed in the finally below. An
# explicit WORK override is honoured and then left for the caller to manage.
import shutil
import tempfile
_work_override = os.environ.get("WORK")
if _work_override:
    WORK, OWN_WORK = _work_override, False
else:
    fd, WORK = tempfile.mkstemp(prefix="ipc8232-api.", suffix=".img")
    os.close(fd)
    OWN_WORK = True
shutil.copyfile(FLASH, WORK)

# restrict=on isolates the guest (it must not reach the real vendor cloud with
# the dump's identity); the host forwards bind to loopback only, since the API
# reproduces a guest-root command injection and must not be LAN-reachable.
cmd = [
    QEMU, "-M", "hi3516ev300,sensor=sp2305,flash-file=" + WORK,
    "-global", "hisi-fmc.nand-jedec=0xEFAA21",
    "-global", "hisi-fmc.nand-oob-size=64",
    "-nic", "user,restrict=on,hostfwd=tcp:127.0.0.1:18080-:8080,"
            "hostfwd=tcp:127.0.0.1:18081-:80,hostfwd=tcp:127.0.0.1:15540-:554",
    "-nographic", "-serial", "mon:stdio",
]
print("launching:", " ".join(cmd), flush=True)
child = pexpect.spawn(cmd[0], cmd[1:], encoding="latin1", timeout=120)
child.logfile_read = sys.stdout

import atexit


def _cleanup():
    try:
        child.close(force=True)
    except Exception:
        pass
    if OWN_WORK:
        try:
            os.unlink(WORK)
        except OSError:
            pass


atexit.register(_cleanup)  # fires on normal exit, exceptions and Ctrl-C
import signal
# SIGTERM would otherwise skip atexit; turn it into a clean SystemExit
signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))

# 1) interrupt U-Boot autoboot
child.expect("Hit any key to stop autoboot")
child.send("\r\n")
# vendor U-Boot prompt
child.expect(r"hi3516ev300-vc #", timeout=30)
print("\n[api] at U-Boot prompt", flush=True)

# 2) load the kernel from NAND and boot with rdinit=/bin/sh
child.sendline("nand read 0x42000000 0x200000 0x800000")
child.expect(r"bytes read: OK", timeout=30)
child.expect(r"hi3516ev300-vc #", timeout=10)
child.sendline("setenv bootargs '%s'" % BOOTARGS)
child.expect(r"hi3516ev300-vc #", timeout=10)
child.sendline("bootm 0x42000000")

# 3) land in the pre-init shell (PID 1 = /bin/sh)
child.expect([r"Starting kernel"], timeout=30)
# the busybox shell prompt after init runs; be liberal
child.expect([r"/ #", r"~ #", r"\n# "], timeout=120)
print("\n[api] pre-init shell is up", flush=True)
child.sendline("")
child.expect(r"#", timeout=10)

# 4) force direct access, default creds, then hand off to the real init
child.sendline("export AG_FORCE_DIRECT_ACCESS=1")
child.expect(r"#", timeout=10)
# mount config so we can drop the seller state cache -> default admin:admin54321
child.sendline("mkdir -p /config 2>/dev/null; "
               "mount -t jffs2 mtd:config /config 2>/dev/null || "
               "mount -t ubifs ubi1:config /config 2>/dev/null; "
               "rm -f /config/vc/agent-state-cache.json 2>/dev/null; echo CFGDONE")
child.expect(r"CFGDONE", timeout=20)
child.expect(r"#", timeout=10)
child.sendline("exec /sbin/init")

# 5) wait for AgentGreen to bind the local API
child.expect([r"direct.?access", r"binding to", r"DIRECT_ACCESS",
              r"Hello from", r"HTTP", r"api"], timeout=240)
print("\n[api] AgentGreen/direct-access activity seen; holding for probes",
      flush=True)

# keep the guest alive so the host can curl it
deadline = time.time() + int(os.environ.get("HOLD_S", "600"))
while time.time() < deadline:
    try:
        child.expect(pexpect.TIMEOUT, timeout=10)
    except Exception:
        break
child.close()
