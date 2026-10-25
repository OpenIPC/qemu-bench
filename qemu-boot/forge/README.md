# VPSS state introspection / forge work-in-progress

Goal: enable `hisi-vi-fp.autoclear_mask=0x7` (VPSS IRQ delivery) without
crashing the kernel.  The block is that `VPSS_COMM_OfflineIrqProc` in
`open_vpss.ko` NULL-derefs at offset `+8` of a per-node group state
pointer at `bss + 0x1740 + node_id*4` (node_id always 0 since
`VPSS_DRV_GetIdByIrq` returns 0 unconditionally).

This module dumps that array's contents at runtime so we can:
1. confirm whether `HI_MPI_VPSS_CreateGrp` populates it (it apparently
   doesn't — we saw NULL deref even with Majestic running);
2. learn the live struct shape if it IS populated, to forge a fake one;
3. feed forged state from a follow-up `vpss-forge.ko`.

## Build env

```sh
# 1. Get the same 4.9.37 kernel source the OpenIPC images use
git clone --depth 1 -b hisilicon-hi3516ev200 \
    https://github.com/openipc/linux /tmp/openipc-linux-4.9

# 2. Configure to match the QEMU-target running kernel.
#    NOTE: EXACT kernel CONFIG must match — see "Open issue" below.
cd /tmp/openipc-linux-4.9
PATH=/home/john-1/git/firmware/output/per-package/hisilicon-opensdk/host/opt/ext-toolchain/bin:$PATH \
  ARCH=arm CROSS_COMPILE=arm-linux- make hi3516ev300_full_defconfig
./scripts/config --disable SMP    # running kernel is UP, not SMP
PATH=/home/john-1/git/firmware/output/per-package/hisilicon-opensdk/host/opt/ext-toolchain/bin:$PATH \
  ARCH=arm CROSS_COMPILE=arm-linux- make olddefconfig modules_prepare -j$(nproc)

# 3. Build the module
cd qemu-boot/forge
PATH=/home/john-1/git/firmware/output/per-package/hisilicon-opensdk/host/opt/ext-toolchain/bin:$PATH \
  KDIR=/tmp/openipc-linux-4.9 make ARCH=arm CROSS_COMPILE=arm-linux-

# Resulting vpss-introspect.ko has vermagic
#   "4.9.37 mod_unload ARMv7 p2v8"
# matching the running QEMU kernel.
```

## Running

Inside QEMU guest (after vendor MPP modules are loaded):

```sh
# tftp the module onto the guest (use QEMU SLIRP TFTP: -nic user,tftp=DIR)
cd /tmp && tftp -g -r vpss-introspect.ko 10.0.2.2

# Pass anchor=<bss_base>; look up the runtime address with:
#   grep '.LANCHOR1.*open_vpss' /proc/kallsyms
# Per current build: 0xbf246900 (will vary across builds)
insmod /tmp/vpss-introspect.ko anchor=0xbf246900

# Output goes to dmesg
dmesg | grep vpss-introspect
```

## Open issue blocking insmod

The module loads with **vermagic match** but triggers a kernel oops
inside `trace_module_notify`:

```
Unable to handle kernel paging request at virtual address ...
PC is at trace_module_notify+0x94/0x160
Code: e59f90c8 e0868108 e1580006 9affffe8 (e5967000)
... [stack backtrace through load_module] ...
Modules linked in: vpss_introspect(O+) ...
```

Disabling `CONFIG_FTRACE / CONFIG_TRACEPOINTS` in our build .config
didn't fix it.  Adding `-funwind-tables` introduced a different error
(`Unknown symbol __aeabi_unwind_cpp_pr1`).

This is a `struct module` ABI offset mismatch between our build's
.config and the actual running kernel's .config.  The trace_module_notify
walker iterates a list at a struct member offset that differs, so it
walks past valid memory and faults.

Resolving requires:
1. Recovering the **exact** running-kernel CONFIG (no `/proc/config.gz`
   in the OpenIPC image — feature not enabled).  Possible via
   `extract-ikconfig` on a vmlinux dump, OR by tracking the OpenIPC
   firmware-build commit that produced this `uImage.hi3516ev300` and
   reusing its build artifacts directly.
2. Or building this module **inside** the OpenIPC firmware tree (via
   `make BOARD=hi3516ev200_lite br-hisilicon-opensdk` with our source
   added to `kernel/Kbuild`), so it gets the same toolchain/CONFIG
   automatically.

Path (2) is the cleaner approach for the next session.

## Once introspection works

Two outcomes:

- **`ARR[0] != NULL`**: vendor populates the slot already; the deeper
  `[+8]` deref chain is what fails.  Dump first 64 bytes of the group
  struct, walk further pointers, identify the specific NULL
  sub-pointer.
- **`ARR[0] == NULL`**: vendor only populates on first frame (chicken-
  and-egg).  Build `vpss-forge.ko` to allocate a fake `VpssGroup`,
  initialise key fields, store the pointer.

Either way, follow-up PRs would land:
- `vpss-forge.ko` — the actual state forge
- `qemu-boot/forge/Makefile` — Kbuild-style build instructions
- An updated `hisi-vi-fp.c` that arms `mask=0x7` only after
  `/proc/vpss-forge-armed` reports ready
