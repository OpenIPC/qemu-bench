/*
 * HiSilicon Boot ROM fastboot serial protocol — public interface.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_HISI_FASTBOOT_H
#define HW_MISC_HISI_FASTBOOT_H

#define TYPE_HISI_FASTBOOT "hisi-fastboot"

/*
 * Configure the fastboot device after qdev_new() but before realize.
 *
 * v500: select the HiSilicon V500 boot-ROM protocol (Goke GK7206) instead
 *       of the classic one.  chip_id is the value the ROM returns in the
 *       handshake reply (the SoC's soc_id), ignored when v500 is false.
 */
void hisi_fastboot_setup(DeviceState *dev, CPUState *cpu,
                         Chardev *chardev, hwaddr uart0_base,
                         qemu_irq uart0_irq, bool v500, uint32_t chip_id);

#endif /* HW_MISC_HISI_FASTBOOT_H */
