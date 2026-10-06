/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Realtek RTD129x PCIe: paged MMIO accessors.
 *
 * The RTD129x root ports do not decode the PCI memory window (0xc0000000 /
 * 0xc1000000) as a CPU address. The only CPU-visible path to a device's
 * BARs is the 4 KiB translation window at the controller's second register
 * range (0x9804f000 / 0x9803c000), whose PCI base is REG_PCI_TRANS. The
 * vendor 4.9 tree exposed this as rtk_pcie1_read()/rtk_pcie1_write() and
 * its out-of-tree WLAN drivers used them instead of readl() on an ioremap
 * of the BAR. Mainline rtlwifi's io ops are function pointers, so it can
 * be pointed here without touching the chip-specific code.
 */
#ifndef _LINUX_PCIE_RTD129X_H
#define _LINUX_PCIE_RTD129X_H

#include <linux/types.h>

struct pci_dev;

#if IS_ENABLED(CONFIG_PCIE_RTD129X)
bool rtd129x_pcie_dev_is_paged_mmio(struct pci_dev *pdev);
u32 rtd129x_pcie_mmio_read(struct pci_dev *pdev, u32 offset, u8 size);
void rtd129x_pcie_mmio_write(struct pci_dev *pdev, u32 offset, u8 size, u32 val);
#else
static inline bool rtd129x_pcie_dev_is_paged_mmio(struct pci_dev *pdev) { return false; }
static inline u32 rtd129x_pcie_mmio_read(struct pci_dev *pdev, u32 offset, u8 size) { return ~0U; }
static inline void rtd129x_pcie_mmio_write(struct pci_dev *pdev, u32 offset, u8 size, u32 val) { }
#endif

#endif
