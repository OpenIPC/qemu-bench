# Allwinner V3s and V851s

`hw/arm/allwinner-cam.c`, machines `v3s` (sun8iw8) and `v851s` (sun8iw21):
one Cortex-A7, 64 MiB DDR2 in the package, SPI NOR on SPI0 and an SD card
slot. Enough of each SoC for OpenIPC's mainline U-Boot (`openipc-allwinner`
on OpenIPC/u-boot) and the kernels it boots. The devices are the V831
machine's ([allwinner-v83x.md](allwinner-v83x.md)) at these SoCs' addresses.

```bash
bash qemu/setup.sh
./qemu-src/build/qemu-system-arm -M v851s,flash-file=flash.bin \
    -drive if=sd,format=raw,file=sd.img -nographic -serial mon:stdio
```

`flash-file` must be a power of two from 64 KiB; writes go back to it.

## Boot

The machine does the mask ROM's job: an eGON.BT0 at SD card sector 16
(8 KiB) wins, else the one at NOR offset 0; it is copied to SRAM and
`boot_media` in its header says where from (0 SD, 3 SPI NOR).

| | V3s | V851s |
|---|---|---|
| SRAM (the SPL) | `0x00000000`, the core starts there | `0x00020000`; a stub at the ROM's `0` jumps to it |
| DDR | `0x40000000`, 64 MiB | `0x40000000`, 64 MiB |
| CCU | `0x01C20000`, `aw-ccu gen=1` (the sun6i PLL lock bits) | `0x02001000`, `aw-ccu` |
| DRAM controller | `0x01C62000` COM, `0x01C63000` CTL | `0x03102000` COM, `0x03103000` PHY |
| UART0 | `0x01C28000` | `0x02500000` |
| SPI0 | `0x01C68000` | `0x04025000`, its DRQ 22 on `aw-dma` at `0x03002000` |
| SD (SMHC0) | `0x01C0F000`, sun5i host | `0x04020000`, A64 host |
| EMAC | `0x01C30000`, the internal PHY at 1 | `0x04500000` |
| timers / watchdog | `0x01C20C00` / `+0xA0` | `0x02050000` / `+0xA0` |
| GIC-400 | `0x01C80000` | `0x03020000` |

## DRAM sizing

U-Boot's DRAM drivers size the part by writing a pattern at each row,
bank and column bit and looking for where it wraps. `aw-dramcom` models
that: while the geometry the driver programmed in the COM block's first
register is not the part's (13 row bits, 4 banks, 2 KiB pages here; the
V833's 8 banks in `allwinner-v83x.c`), the 2 GiB window above `0x40000000`
is remapped as the controller would map it, so the probe aliases where it
does on silicon. `aw-dramc` reports rank training as a single-rank part
does (DX GSR0 and PGSR0 bits), so the dual-rank probe fails as it should.
The V3s's, V831's, V833's and V851s's U-Boot report 64/64/128/64 MiB, as
the vendor's V833 boot0 now does too (4160 MiB before).

## Tested

- `openipc-allwinner` U-Boot, both SoCs, `-nor` and `-mmc` images: two
  boots (the environment saved, then read back), `reset`, `run ub` from
  the card onto an erased NOR, then a boot from the NOR alone.
- V851s: OpenIPC's `uImage.v851s` and `rootfs.squashfs.v851s` (Linux 4.9,
  device tree appended) boot from the NOR to a shell, the root read
  through SPI DMA; `reboot -f` comes back through the SPL. The published
  `u-boot-v851s-nor.bin` (OpenIPC/u-boot-allwinner, mainline 2024.10)
  runs to its prompt; after it and after `openipc-allwinner`'s, the DRAM,
  CCU and PRCM registers are equal.

## Not modelled

Video, audio, the NPU and the V851s's RISC-V core; the V3s's EMAC is
there, the V851s's is mapped but untested (OpenIPC's V851s U-Boot has no
Ethernet). DRAM timing and training are not checked: training completes
at once.
