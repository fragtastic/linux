// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek RTD1295 PCIe
 *
 * Copyright (c) 2017 Andreas Färber
 *
 * Authors:
 *   James Tai <james.tai@realtek.com>
 *   Andreas Färber
 */

#include <linux/clk.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/pci_hotplug.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "../pci.h"
#include <linux/pcie-rtd129x.h>

#define REG_SYS_CTR	0xc00
#define REG_INT_CTR	0xc04
#define REG_GNR_INT	0xc08
#define REG_PCIE_INT	0xc0c
#define REG_INDIR_CTR	0xc14
#define REG_DIR_CTR	0xc18
#define REG_MDIO_CTR	0xc1c
#define REG_CFG_CT	0xc38
#define REG_CFG_EN	0xc3c
#define REG_CFG_ST	0xc40
#define REG_CFG_ADDR	0xc44	/* PCI CFG format, spec: 3.2.2.3.2 */
#define REG_CFG_WDATA	0xc48
#define REG_CFG_RDATA	0xc4c
#define REG_PHY_CTR	0xc68
#define REG_DIR_EN	0xc78
#define REG_RCPL_ST	0xc7c
#define REG_MAC_ST	0xcb4
#define REG_MSI_TRAN	0xcd0
#define REG_MSI_DATA	0xcd4
#define REG_LINK_ST	0xce8
#define REG_PCI_BASE	0xcfc
#define REG_PCI_MASK	0xd00
#define REG_PCI_TRANS	0xd04

#define SYS_CTR_APP_LTSSM_EN		BIT(1)
#define SYS_CTR_DIR_CFG_EN		BIT(4)
#define SYS_CTR_INDIR_CFG_EN		BIT(5)
#define SYS_CTR_PHY_MDIO_RSTN		BIT(17)
#define SYS_CTR_PHY_MDIO_OE		BIT(18)
#define SYS_CTR_MM_IO_TYPE		BIT(19)
#define SYS_CTR_MM_IO_TYPE_IO		FIELD_PREP(SYS_CTR_MM_IO_TYPE, 0)
#define SYS_CTR_MM_IO_TYPE_MM		FIELD_PREP(SYS_CTR_MM_IO_TYPE, 1)
#define SYS_CTR_TRAN_EN		BIT(20)

#define INT_CTR_PCIE_LEGACY_MSI_EN	BIT(12)
#define INT_CTR_LINK_UP_INTP_EN	BIT(13)

#define GNR_INT_PCIE_LEGACY_INT	BIT(11)
#define GNR_INT_PCIE_LEGACY_MSI_INT	BIT(14)
#define GNR_INT_LINK_UP_INT		BIT(15)

#define INDIR_CTR_REQ_INFO_FMT		GENMASK(1, 0)
#define INDIR_CTR_REQ_INFO_TYPE	GENMASK(6, 2)
#define INDIR_CTR_REQ_INFO_TC		GENMASK(9, 7)

#define MDIO_CTR_MDIO_RDWR		BIT(0)
#define MDIO_CTR_MDIO_RDWR_READ	FIELD_PREP(MDIO_CTR_MDIO_RDWR, 0)
#define MDIO_CTR_MDIO_RDWR_WRITE	FIELD_PREP(MDIO_CTR_MDIO_RDWR, 1)
#define MDIO_CTR_MDIO_SRST		BIT(1)
#define MDIO_CTR_MCLK_RATE		GENMASK(3, 2)
#define MDIO_CTR_MDIO_RDY		BIT(4)
#define MDIO_CTR_MDIO_ST		GENMASK(6, 5)
#define MDIO_CTR_MDIO_BUSY		BIT(7)
#define MDIO_CTR_PHY_REG		GENMASK(12, 8)
#define MDIO_CTR_PHY_ADDR		GENMASK(15, 13)
#define MDIO_CTR_DATA			GENMASK(31, 16)

#define CFG_CT_GO_CT			BIT(0)

#define CFG_EN_WRRD_EN				BIT(0)
#define CFG_EN_WRRD_EN_READ			FIELD_PREP(CFG_EN_WRRD_EN, 0)
#define CFG_EN_WRRD_EN_WRITE			FIELD_PREP(CFG_EN_WRRD_EN, 1)
#define CFG_EN_BYTE_EN				BIT(1)
#define CFG_EN_BYTE_EN_ALL			FIELD_PREP(CFG_EN_BYTE_EN, 0)
#define CFG_EN_BYTE_EN_BYTE_CNT_7TO4_0C	FIELD_PREP(CFG_EN_BYTE_EN, 1)
#define CFG_EN_ERROR_EN			BIT(2)
#define CFG_EN_BYTE_CNT			GENMASK(7, 4)
#define CFG_EN_FUN_NUM				GENMASK(10, 8)
#define CFG_EN_DEV_NUM				GENMASK(15, 11)
#define CFG_EN_BUS_NUM				GENMASK(23, 16)

#define CFG_ST_DONE_ST			BIT(0)
#define CFG_ST_ERROR_ST		BIT(1)

#define PHY_CTR_REG_PLLDVR		GENMASK(2, 0)
#define PHY_CTR_REG_PLLDVR_OFF_DEVICE	FIELD_PREP(PHY_CTR_REG_PLLDVR, 0x0)
#define PHY_CTR_REG_PLLDVR_ON_HOST	FIELD_PREP(PHY_CTR_REG_PLLDVR, 0x1)
#define PHY_CTR_POW_PCIEX		BIT(3)
#define PHY_CTR_RX50_LINK		BIT(4)
#define PCIE2_PHY_CTR_ISOLATE		BIT(5)

#define DIR_EN_TIMEOUT_EN		BIT(0)
#define DIR_EN_TIMEOUT_CNT_VALUE	GENMASK(31, 8)

#define RCPL_ST_RCPL_TIMEOUT_ST	BIT(0)
#define RCPL_ST_ECRC_ERROR_ST		BIT(1)
#define RCPL_ST_DLLP_ABORT_ST		BIT(2)
#define RCPL_ST_TLP_ABORT_ST		BIT(3)
#define RCPL_ST_RCPL_ERROR_ST		BIT(4)
#define RCPL_ST_RCPL_STATUS		GENMASK(7, 5)

#define MAC_ST_XMLH_LINK_UP		BIT(11)
#define MAC_ST_RDLH_LINK_UP		BIT(15)
#define MAC_ST_CLK_RDY			BIT(16)

#define MSI_DATA_MSI_DATA		GENMASK(15, 0)
#define MSI_DATA_MSI_DATA_ST		BIT(16)

#define LINK_ST_LINK_UP_ST		BIT(0)

#define RTD129X_PCIE_USE_MSI		1

#define RTD129X_PCIE_QUIRK_SB4		BIT(0)
#define RTD129X_PCIE_QUIRK_PHY_A	BIT(1)

struct rtd129x_pcie_quirks {
	u32 flags;
};

#define RTD129X_PCIE_CAP_PCIE_2_0	BIT(0)

struct rtd129x_pcie_info {
	u32 cap;
};

struct rtd129x_pcie_priv {
	struct platform_device *pdev;
	const struct rtd129x_pcie_info *info;
	void __iomem *ctrl_base;
	void __iomem *cfg_base;
	struct resource *cfg_res;
	struct resource *trans_res;
	struct regmap *sb4;
	struct clk *clk;
	struct reset_control *pcie_stitch_reset;
	struct reset_control *pcie_reset;
	struct reset_control *pcie_core_reset;
	struct reset_control *pcie_power_reset;
	struct reset_control *pcie_nonstitch_reset;
	struct reset_control *pcie_phy_reset;
	struct reset_control *pcie_phy_mdio_reset;
	struct gpio_desc *reset_gpiod;
	spinlock_t mmio_lock;	/* paged MMIO window (REG_PCI_TRANS/MASK) */
	struct irq_domain *intx_irq_domain;
	struct irq_domain *msi_irq_domain;
	struct irq_domain *msi_domain;
	bool msi_allocated;
	void *msi_data;
	dma_addr_t msi_data_dma;
	struct hotplug_slot hotplug_slot;
	struct workqueue_struct *wq;
	struct work_struct link_work;
	int irq;
	int gen;
	u32 quirks;
};

#ifdef CONFIG_HOTPLUG_PCI
static int rtd129x_pcie_get_adapter_status(struct hotplug_slot *slot, u8 *value)
{
	struct rtd129x_pcie_priv *data = container_of(slot, struct rtd129x_pcie_priv, hotplug_slot);
	u32 val;

	val = readl_relaxed(data->ctrl_base + REG_MAC_ST);
	*value = (val & MAC_ST_XMLH_LINK_UP) ? 1 : 0;

	return 0;
}

static const struct hotplug_slot_ops rtd129x_pcie_hotplug_slot_ops = {
	.get_adapter_status	= rtd129x_pcie_get_adapter_status,
};

static int rtd129x_pcie_init_hotplug(struct rtd129x_pcie_priv *data)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(data);
	int ret;

	data->hotplug_slot.ops = &rtd129x_pcie_hotplug_slot_ops;

	ret = pci_hp_initialize(&data->hotplug_slot, bridge->bus, 0, "rtd129x");
	if (ret) {
		dev_err(&data->pdev->dev, "hotplug init failed (%d)\n", ret);
		return ret;
	}

	ret = pci_hp_add(&data->hotplug_slot);
	if (ret) {
		dev_err(&data->pdev->dev, "hotplug add failed (%d)\n", ret);
		return ret;
	}

	return 0;
}
#endif

static void rtd129x_pcie_link_init(struct rtd129x_pcie_priv *data)
{
	u32 val;

	/* Make sure DBI is working */
	writel(0x7, data->ctrl_base + 0x004);

	/* Set limit and base registers */
	val = 0xfff0;
	writel_relaxed(val, data->ctrl_base + 0x020);
	writel_relaxed(val, data->ctrl_base + 0x024);
}

static void rtd129x_pcie_link_changed(struct work_struct *ws)
{
	struct rtd129x_pcie_priv *data = container_of(ws, struct rtd129x_pcie_priv, link_work);
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(data);
	struct pci_dev *dev, *temp;
	u32 val;
	int ret;

	val = readl_relaxed(data->ctrl_base + REG_MAC_ST);
	if (val & MAC_ST_XMLH_LINK_UP) {
		rtd129x_pcie_link_init(data);

		pci_lock_rescan_remove();

		ret = pci_scan_slot(bridge->bus, PCI_DEVFN(0, 0));
		dev_info(&data->pdev->dev, "scan slot (%d)\n", ret);

		for_each_pci_bridge(dev, bridge->bus)
			pci_hp_add_bridge(dev);

		pci_assign_unassigned_bus_resources(bridge->bus);
		pcie_bus_configure_settings(bridge->bus);
		pci_bus_add_devices(bridge->bus);

		pci_unlock_rescan_remove();
	} else {
		pci_walk_bus(bridge->bus, pci_dev_set_disconnected, NULL);

		pci_lock_rescan_remove();

		list_for_each_entry_safe_reverse(dev, temp, &bridge->bus->devices, bus_list) {
			pci_dev_get(dev);
			pci_stop_and_remove_bus_device(dev);
			pci_dev_put(dev);
		}

		pci_unlock_rescan_remove();
	}
}

static void rtd129x_pcie_irq_handle(struct irq_desc *desc)
{
	struct rtd129x_pcie_priv *data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 gnr_int, pcie_int, link_st;
	int i;

	chained_irq_enter(chip, desc);

	gnr_int = readl_relaxed(data->ctrl_base + REG_GNR_INT);

	if (RTD129X_PCIE_USE_MSI) {
		u32 msi_data_reg = readl_relaxed(data->ctrl_base + REG_MSI_DATA);
		/* v44: この診断ログ(dev_info_ratelimited)がハンドラ入口で無条件に
		 * 発火し、2026-09-21実測でカーネルログの91%(1分1230行中1115行)を
		 * 占有してdmesgリングバッファを実質使用不能にしていた。
		 * journalctl -k と /proc/interrupts の60秒差分で検証済み:
		 *   親(このハンドラの発火数、推定下限3536) ≒ 子(EPドライバのMSI処理数3951)
		 *   → 重複処理ではなく、Wi-Fi 2枚(8192EE+8812AE)が単純に高頻度で
		 *     通信しているだけ(比率0.89、ほぼ1:1)。IRQ自体は正常。
		 * また下記の「救済」分岐は実測サンプル全行でGNR_INT bit14が
		 * 既に立っており(=既存条件で処理可能)、一度も踏まれていないことを
		 * journalctl -k の生ログで確認済み。無害なため触らず残す。
		 * → 削るのはこの診断ログだけで十分。全経緯=
		 *   ~/.claude/skills/quastation/CHANGELOG.md 該当エントリ参照。
		 */
		/* diag: ram[0]==0 && ST=0 なら EP からの TLP 未到着とみなす。
		 * MSI_DATA_ST が立っているのに GNR_INT bit14 が立たない場合は
		 * 受信経路の enable 漏れとみなし、暫定で救済してハンドラへ回す。 */
		if (!(gnr_int & GNR_INT_PCIE_LEGACY_MSI_INT) && (msi_data_reg & MSI_DATA_MSI_DATA_ST))
			gnr_int |= GNR_INT_PCIE_LEGACY_MSI_INT;
	}

	if (RTD129X_PCIE_USE_MSI && (gnr_int & GNR_INT_PCIE_LEGACY_MSI_INT)) {
		generic_handle_irq(irq_find_mapping(data->msi_irq_domain, 0));
	}

	if (gnr_int & GNR_INT_PCIE_LEGACY_INT) {
		pcie_int = readl_relaxed(data->ctrl_base + REG_PCIE_INT);
		while (pcie_int) {
			i = __ffs(pcie_int);
			pcie_int &= ~BIT(i);

			generic_handle_irq(irq_find_mapping(data->intx_irq_domain, i));
		}
	}

	if (gnr_int & GNR_INT_LINK_UP_INT) {
		link_st = readl_relaxed(data->ctrl_base + REG_LINK_ST);
		if (link_st & LINK_ST_LINK_UP_ST) {
			queue_work(data->wq, &data->link_work);
			link_st = LINK_ST_LINK_UP_ST;
			writel_relaxed(link_st, data->ctrl_base + REG_LINK_ST);
		}
	}

	chained_irq_exit(chip, desc);
}

static void rtd129x_pcie_msi_compose_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct rtd129x_pcie_priv *data = irq_data_get_irq_chip_data(d);
	u32 val;
	dma_addr_t addr;

	/* virt_to_phys()はdma_alloc_coherent()の戻りアドレスに対して正しい保証がない。
	 * MSI_TRANに実際に設定したdma_handle(msi_data_dma)をそのまま使う。 */
	addr = data->msi_data_dma;
	msg->address_hi = upper_32_bits(addr);
	msg->address_lo = lower_32_bits(addr);
	/* 受信レジスタ REG_MSI_DATA を読んで data にするのは循環定義(起動時0)。固定値にする。 */
	msg->data = 0x0001;

	dev_dbg(&data->pdev->dev, "msi#%d address_hi %#x address_lo %#x data %04x\n",
		(int)d->hwirq, msg->address_hi, msg->address_lo, msg->data);
}

static void rtd129x_pcie_msi_ack(struct irq_data *d)
{
	struct rtd129x_pcie_priv *data = irq_data_get_irq_chip_data(d);

	writel_relaxed(MSI_DATA_MSI_DATA_ST, data->ctrl_base + REG_MSI_DATA);
}

static struct irq_chip rtd129x_pcie_msi_bottom_irq_chip = {
	.name			= "RTD129x MSI",
	.irq_compose_msi_msg	= rtd129x_pcie_msi_compose_msg,
	.irq_ack		= rtd129x_pcie_msi_ack,
};

static struct irq_chip rtd129x_pcie_msi_irq_chip = {
	.name		= "RTD129x PCIe MSI",
	.irq_ack	= irq_chip_ack_parent,
	.irq_mask	= pci_msi_mask_irq,
	.irq_unmask	= pci_msi_unmask_irq,
};

static struct msi_domain_info rtd129x_pcie_msi_domain_info = {
	/*
	 * run28e: MSI_FLAG_NO_AFFINITY is required. Without it
	 * msi_domain_update_chip_ops() (kernel/irq/msi.c) installs
	 * msi_domain_set_affinity() on the top chip because it has no
	 * .irq_set_affinity of its own, and that helper calls the *parent*
	 * chip's ->irq_set_affinity without a NULL check. The parent here
	 * only implements compose_msi_msg/ack, so the first request_irq()
	 * on an MSI (rtlwifi enables MSI by default) jumped to address 0
	 * and panicked. This controller has a single, non-steerable MSI
	 * line, so declaring "no affinity" is also what it actually is --
	 * same as pcie-altera-msi and friends.
	 */
	.flags	= MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		  MSI_FLAG_PCI_MSIX | MSI_FLAG_NO_AFFINITY,
	.chip	= &rtd129x_pcie_msi_irq_chip,
};

static int rtd129x_pcie_msi_domain_alloc(struct irq_domain *domain, unsigned int virq,
					 unsigned int nr_irqs, void *args)
{
	struct rtd129x_pcie_priv *data = domain->host_data;

	WARN_ON(nr_irqs != 1);

	if (data->msi_allocated)
		return -ENOSPC;

	data->msi_allocated = true;
	irq_domain_set_info(domain, virq, 0, &rtd129x_pcie_msi_bottom_irq_chip,
			    domain->host_data, handle_edge_irq,
			    NULL, NULL);

	return 0;
}

static void rtd129x_pcie_msi_domain_free(struct irq_domain *domain, unsigned int virq,
					 unsigned int nr_irqs)
{
	struct rtd129x_pcie_priv *data = domain->host_data;

	WARN_ON(nr_irqs != 1);

	data->msi_allocated = false;
	irq_domain_free_irqs_parent(domain, virq, nr_irqs);
}

static const struct irq_domain_ops rtd129x_pcie_msi_domain_ops = {
	.alloc	= rtd129x_pcie_msi_domain_alloc,
	.free	= rtd129x_pcie_msi_domain_free,
};

static int rtd129x_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
	irq_hw_number_t hwirq)
{
	struct rtd129x_pcie_priv *data = domain->host_data;

	dev_info(&data->pdev->dev, "%s irq %u hwirq %lu\n", __func__, irq, hwirq);

	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_level_irq);
	irq_set_handler_data(irq, data);

	return 0;
}

static const struct irq_domain_ops rtd129x_pcie_intx_domain_ops = {
	.xlate	= irq_domain_xlate_onecell,
	.map	= rtd129x_pcie_intx_map,
};

static void rtd129x_pcie_phy_reset(struct rtd129x_pcie_priv *s)
{
	u32 val;

	val = MDIO_CTR_MDIO_SRST | MDIO_CTR_MDIO_RDWR_WRITE;
	//dev_info(&s->pdev->dev, "MDIO reset: 0x%08x\n", val);
	writel(val, s->ctrl_base + REG_MDIO_CTR);
	mdelay(1);
}

static void rtd129x_pcie_phy_write(struct rtd129x_pcie_priv *s,
	u8 addr, u8 reg, u16 data)
{
	u32 val;

	val = FIELD_PREP(MDIO_CTR_DATA, data) |
	      FIELD_PREP(MDIO_CTR_PHY_ADDR, addr) |
	      FIELD_PREP(MDIO_CTR_PHY_REG, reg) |
	      MDIO_CTR_MDIO_RDWR_WRITE;
	//dev_info(&s->pdev->dev, "MDIO write: 0x%08x\n", val);
	writel(val, s->ctrl_base + REG_MDIO_CTR);
	mdelay(1);
}

static u32 rtd129x_pcie_cfg_address(int busnr, unsigned int devfn, int reg)
{
	return ((u32)busnr << 24) | (PCI_SLOT(devfn) << 19) | (PCI_FUNC(devfn) << 16) | reg;
}

static u32 rtd129x_pcie_cfg_mask(int reg, int size)
{
	u32 offset = reg & 0x3;

	return GENMASK(offset + size - 1, offset);
}

static int rtd129x_pcie_read_conf(struct pci_bus *bus, unsigned int devfn,
	int where, int size, u32 *pval)
{
	struct rtd129x_pcie_priv *data = bus->sysdata;
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(data);
	u32 addr, mask, val;
	int ret;

	//dev_info(&data->pdev->dev, "read_conf bus %d devfn 0x%x where 0x%x (off %u) size %u\n", bus->number, devfn, where, where & 0x3, size);

	if (bus->number != bridge->busnr || devfn != PCI_DEVFN(0, 0))
		return PCIBIOS_DEVICE_NOT_FOUND;

	addr = rtd129x_pcie_cfg_address(bus->number, devfn, where);

	mask = rtd129x_pcie_cfg_mask(where, size);
	//dev_info(&data->pdev->dev, "read_conf mask 0x%x\n", mask);
	if (!mask)
		return PCIBIOS_SET_FAILED;

	writel_relaxed(FIELD_PREP(INDIR_CTR_REQ_INFO_TYPE, 0x4), data->ctrl_base + REG_INDIR_CTR);
	writel_relaxed(CFG_ST_ERROR_ST | CFG_ST_DONE_ST, data->ctrl_base + REG_CFG_ST);
	writel_relaxed(addr & ~0x3, data->ctrl_base + REG_CFG_ADDR);
	val = FIELD_PREP(CFG_EN_BYTE_CNT, mask) |
	      CFG_EN_BYTE_EN_BYTE_CNT_7TO4_0C |
	      CFG_EN_WRRD_EN_READ;
	writel_relaxed(val, data->ctrl_base + REG_CFG_EN);
	writel(CFG_CT_GO_CT, data->ctrl_base + REG_CFG_CT);

	ret = readl_relaxed_poll_timeout_atomic(data->ctrl_base + REG_CFG_ST, val, (val & CFG_ST_DONE_ST), 10, 100000);
	if (ret) {
		dev_warn(&data->pdev->dev, "cfg read timed out (0x%08x)\n", val);
		ret = PCIBIOS_SET_FAILED;
		goto out;
	}
	if (val & CFG_ST_ERROR_ST) {
		dev_warn(&data->pdev->dev, "cfg read error (0x%08x)\n", val);
		ret = PCIBIOS_SET_FAILED;
		goto out;
	}
	val = readl_relaxed(data->ctrl_base + REG_CFG_RDATA);
	mask = GENMASK(((where & 0x3) + size) * BITS_PER_BYTE - 1, (where & 0x3) * BITS_PER_BYTE);
	*pval = (val & mask) >> ((where & 0x3) * BITS_PER_BYTE);
	ret = PCIBIOS_SUCCESSFUL;

	//dev_info(&data->pdev->dev, "read_conf = 0x%x (0x%x mask 0x%x)\n", *pval, val, mask);

	val = readl_relaxed(data->ctrl_base + REG_RCPL_ST);
	if (val & (RCPL_ST_RCPL_ERROR_ST | RCPL_ST_TLP_ABORT_ST | RCPL_ST_DLLP_ABORT_ST |
		   RCPL_ST_ECRC_ERROR_ST | RCPL_ST_RCPL_TIMEOUT_ST)) {
		dev_warn(&data->pdev->dev, "cfg read status 0x%x\n", val);
	}

out:
	writel_relaxed(CFG_ST_ERROR_ST | CFG_ST_DONE_ST, data->ctrl_base + REG_CFG_ST);
	return ret;
}

static int rtd129x_pcie_write_conf(struct pci_bus *bus, unsigned int devfn,
	int where, int size, u32 val)
{
	struct rtd129x_pcie_priv *data = bus->sysdata;
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(data);
	u32 addr, mask, tmp;
	int ret;

	//dev_info(&data->pdev->dev, "write_conf bus %d devfn 0x%x where 0x%x (off %u) size %u val 0x%x\n", bus->number, devfn, where, where & 0x3, size, val);

	if (bus->number != bridge->busnr || devfn != PCI_DEVFN(0, 0))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if ((where == 0x10 || where == 0x18 || where == 0x20 || where == 0x24) &&
	    (val & 0xc0000000) == 0xc0000000) {
		unsigned long mmio_flags;

		/*
		 * Serialise against rtd129x_pcie_mmio_xfer(), which saves and
		 * restores REG_PCI_TRANS around every windowed access. BAR
		 * writes normally only happen during enumeration, but
		 * pci_restore_state() on resume would otherwise be able to
		 * land in the middle of a windowed access and have its new
		 * base overwritten by the stale saved one.
		 */
		spin_lock_irqsave(&data->mmio_lock, mmio_flags);
		writel_relaxed(val & 0xfffffff0, data->ctrl_base + REG_PCI_TRANS);
		spin_unlock_irqrestore(&data->mmio_lock, mmio_flags);
		//dev_info(&data->pdev->dev, "cfg write trans 0x%08x\n", val & 0xfffffff0);
	}

	addr = rtd129x_pcie_cfg_address(bus->number, devfn, where);

	mask = rtd129x_pcie_cfg_mask(where, size);
	//dev_info(&data->pdev->dev, "write_conf mask 0x%x\n", mask);
	if (!mask)
		return PCIBIOS_SET_FAILED;

	tmp = FIELD_PREP(INDIR_CTR_REQ_INFO_TYPE, 0x4) |
	      FIELD_PREP(INDIR_CTR_REQ_INFO_FMT, 0x2);
	writel_relaxed(tmp, data->ctrl_base + REG_INDIR_CTR);
	writel_relaxed(CFG_ST_ERROR_ST | CFG_ST_DONE_ST, data->ctrl_base + REG_CFG_ST);
	writel_relaxed(addr & ~0x3, data->ctrl_base + REG_CFG_ADDR);
	writel_relaxed(val << ((where & 0x3) * BITS_PER_BYTE), data->ctrl_base + REG_CFG_WDATA);
	tmp = CFG_EN_WRRD_EN_WRITE;
	if (size == 4)
		tmp |= CFG_EN_BYTE_EN_ALL;
	else
		tmp |= FIELD_PREP(CFG_EN_BYTE_CNT, mask) |
		       CFG_EN_BYTE_EN_BYTE_CNT_7TO4_0C;
	writel_relaxed(tmp, data->ctrl_base + REG_CFG_EN);
	writel(CFG_CT_GO_CT, data->ctrl_base + REG_CFG_CT);

	ret = readl_relaxed_poll_timeout_atomic(data->ctrl_base + REG_CFG_ST, val, (val & CFG_ST_DONE_ST), 10, 50000);
	if (ret) {
		dev_warn(&data->pdev->dev, "cfg write timed out (0x%08x)\n", val);
		ret = PCIBIOS_SET_FAILED;
		goto out;
	}
	if (val & CFG_ST_ERROR_ST) {
		dev_warn(&data->pdev->dev, "cfg write error (0x%08x)\n", val);
		ret = PCIBIOS_SET_FAILED;
		goto out;
	}
	ret = PCIBIOS_SUCCESSFUL;

out:
	writel_relaxed(CFG_ST_ERROR_ST | CFG_ST_DONE_ST, data->ctrl_base + REG_CFG_ST);
	return ret;
}

/*
 * Paged MMIO through the 4 KiB translation window (see
 * include/linux/pcie-rtd129x.h). Mirrors vendor rtk_pcie1_read/write():
 *  - REG_PCI_TRANS normally holds the BAR base (written on BAR assignment
 *    in rtd129x_pcie_write_conf()); for an access we OR in the 4 KiB page
 *    of the offset and use the window at (offset & 0xfff).
 *  - Window offsets 0xc68 / 0xcec alias the controller's own registers
 *    (129x hardware issue, vendor comment), so for those two the window
 *    is narrowed to 2 KiB (REG_PCI_MASK) and the page shifts by 0x800.
 *  - A DLLP/RCPL error flagged in REG_RCPL_ST is cleared and the access
 *    retried (vendor: up to 5 times).
 */
static struct pci_ops rtd129x_pcie_ops;

#define RTD129X_MMIO_MASK_4K	0xfffff000
#define RTD129X_MMIO_MASK_2K	0xfffff800

static struct rtd129x_pcie_priv *rtd129x_pcie_priv_of(struct pci_dev *pdev)
{
	if (!pdev || !pdev->bus || pdev->bus->ops != &rtd129x_pcie_ops)
		return NULL;
	return pdev->bus->sysdata;
}

bool rtd129x_pcie_dev_is_paged_mmio(struct pci_dev *pdev)
{
	return rtd129x_pcie_priv_of(pdev) != NULL;
}
EXPORT_SYMBOL_GPL(rtd129x_pcie_dev_is_paged_mmio);

static u32 rtd129x_pcie_mmio_xfer(struct rtd129x_pcie_priv *data, u32 offset,
				  u8 size, bool is_write, u32 wval)
{
	void __iomem *win;
	unsigned long flags;
	u32 mask, trans, st, rval = 0;
	int retry = 0;

	if ((offset & 0xfff) == 0xc68 || (offset & 0xfff) == 0xcec)
		mask = RTD129X_MMIO_MASK_2K;
	else
		mask = RTD129X_MMIO_MASK_4K;

	spin_lock_irqsave(&data->mmio_lock, flags);

	trans = readl_relaxed(data->ctrl_base + REG_PCI_TRANS);
	if (mask == RTD129X_MMIO_MASK_2K)
		writel_relaxed(mask, data->ctrl_base + REG_PCI_MASK);
	writel(trans | (offset & mask), data->ctrl_base + REG_PCI_TRANS);
	win = data->cfg_base + (offset & ~mask);

again:
	switch (size) {
	case 1:
		if (is_write) writeb(wval, win); else rval = readb(win);
		break;
	case 2:
		if (is_write) writew(wval, win); else rval = readw(win);
		break;
	default:
		if (is_write) writel(wval, win); else rval = readl(win);
		break;
	}

	st = readl_relaxed(data->ctrl_base + REG_RCPL_ST);
	if (st & 0x1f) {
		writel_relaxed(st, data->ctrl_base + REG_RCPL_ST);
		if (retry++ < 5) {
			dev_dbg(&data->pdev->dev, "mmio %s off 0x%x DLLP st 0x%x retry %d\n",
				is_write ? "write" : "read", offset, st, retry);
			goto again;
		}
		dev_warn_ratelimited(&data->pdev->dev, "mmio off 0x%x DLLP st 0x%x\n", offset, st);
	}

	writel(trans, data->ctrl_base + REG_PCI_TRANS);
	if (mask == RTD129X_MMIO_MASK_2K)
		writel_relaxed(RTD129X_MMIO_MASK_4K, data->ctrl_base + REG_PCI_MASK);

	spin_unlock_irqrestore(&data->mmio_lock, flags);
	return rval;
}

u32 rtd129x_pcie_mmio_read(struct pci_dev *pdev, u32 offset, u8 size)
{
	struct rtd129x_pcie_priv *data = rtd129x_pcie_priv_of(pdev);

	if (!data)
		return ~0U;
	return rtd129x_pcie_mmio_xfer(data, offset, size, false, 0);
}
EXPORT_SYMBOL_GPL(rtd129x_pcie_mmio_read);

void rtd129x_pcie_mmio_write(struct pci_dev *pdev, u32 offset, u8 size, u32 val)
{
	struct rtd129x_pcie_priv *data = rtd129x_pcie_priv_of(pdev);

	if (data)
		rtd129x_pcie_mmio_xfer(data, offset, size, true, val);
}
EXPORT_SYMBOL_GPL(rtd129x_pcie_mmio_write);

static struct pci_ops rtd129x_pcie_ops = {
	.read	= rtd129x_pcie_read_conf,
	.write	= rtd129x_pcie_write_conf,
};

#define RTD129X_MSI_TRAN_FLAGS		0	/* 実験1: 0 / 実験2: 1 (意味不明・要実測) */

static int rtd129x_pcie_init_msi(struct rtd129x_pcie_priv *data)
{
	dma_addr_t dma_handle;

	data->msi_irq_domain = irq_domain_add_linear(NULL, 1,
						     &rtd129x_pcie_msi_domain_ops, data);
	if (!data->msi_irq_domain)
		return -ENOMEM;

	data->msi_domain = pci_msi_create_irq_domain(data->pdev->dev.fwnode,
						     &rtd129x_pcie_msi_domain_info,
						     data->msi_irq_domain);
	if (!data->msi_domain) {
		irq_domain_remove(data->msi_irq_domain);
		return -ENOMEM;
	}

	/* diag: EPからのDMA書き込みをキャッシュのステイル読みなしで観測するため
	 * coherent(uncached)割り当てにする。alloc_pages+page_address のキャッシャブルRAM
	 * だと ram[0] の読みが古い値のままになりうる。 */
	data->msi_data = dma_alloc_coherent(&data->pdev->dev, PAGE_SIZE, &dma_handle, GFP_KERNEL);
	if (!data->msi_data) {
		irq_domain_remove(data->msi_domain);
		irq_domain_remove(data->msi_irq_domain);
		return -ENOMEM;
	}
	data->msi_data_dma = dma_handle;
	writel_relaxed((lower_32_bits(dma_handle) & ~0x3) | RTD129X_MSI_TRAN_FLAGS,
		       data->ctrl_base + REG_MSI_TRAN);
	dev_info(&data->pdev->dev, "MSI doorbell phys=%pad MSI_TRAN=%08x INT_CTR=%08x\n",
		 &dma_handle, readl_relaxed(data->ctrl_base + REG_MSI_TRAN),
		 readl_relaxed(data->ctrl_base + REG_INT_CTR));

	return 0;
}

static int rtd129x_pcie_init_irq(struct rtd129x_pcie_priv *data)
{
	struct device_node *node;
	u32 val;
	int ret;

	node = of_get_child_by_name(data->pdev->dev.of_node, "interrupt-controller");
	if (node) {
		data->intx_irq_domain = irq_domain_add_linear(node, 4,
							      &rtd129x_pcie_intx_domain_ops, data);
		of_node_put(node);
		if (!data->intx_irq_domain)
			return -ENOMEM;
	} else
		dev_dbg(&data->pdev->dev, "no interrupt-controller child node\n");

	/*
	 * run28c: MSI disabled for now. The legacy (non per-device) MSI
	 * domain setup below dates from ~5.x; on 6.18 the first
	 * pci_enable_msi() from rtlwifi (msi_support=1 by default for
	 * rtl8192ee/rtl8821ae) ended in a NULL pointer dereference right
	 * after "rtlwifi: wireless switch is on", i.e. in
	 * rtl_pci_intr_mode_decide(). Without an MSI domain, pci_enable_msi()
	 * fails cleanly and rtlwifi falls back to INTx, which this driver
	 * routes through the intx irq domain. Revisit with msi_parent_ops.
	 */
	if (RTD129X_PCIE_USE_MSI) {
		ret = rtd129x_pcie_init_msi(data);
		if (ret)
			return ret;
	}

	data->wq = alloc_workqueue("rtd129x_pcie_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
	INIT_WORK(&data->link_work, rtd129x_pcie_link_changed);

	writel_relaxed(0, data->ctrl_base + REG_INT_CTR);

	irq_set_chained_handler_and_data(data->irq, rtd129x_pcie_irq_handle, data);

	writel_relaxed(~0, data->ctrl_base + REG_LINK_ST);
	val = INT_CTR_LINK_UP_INTP_EN;
	if (RTD129X_PCIE_USE_MSI)
		val |= INT_CTR_PCIE_LEGACY_MSI_EN;
	writel_relaxed(val, data->ctrl_base + REG_INT_CTR);

	return 0;
}

static int rtd129x_pcie_init(struct rtd129x_pcie_priv *data)
{
	u32 val;
	u16 phyval;
	int ret;

	if (data->quirks & RTD129X_PCIE_QUIRK_SB4) {
		dev_info(&data->pdev->dev, "applying SB4 quirk\n");

		ret = regmap_read(data->sb4, 0x14, &val);
		val &= ~GENMASK(2, 0);
		val |= 0x1;
		ret = regmap_write(data->sb4, 0x14, val);

		ret = regmap_read(data->sb4, 0x00, &val);
		if (data->info->cap & RTD129X_PCIE_CAP_PCIE_2_0)
			val &= ~GENMASK(19, 16);
		else
			val &= ~GENMASK(3, 0);
		ret = regmap_write(data->sb4, 0x00, val);

		ret = regmap_read(data->sb4, 0x08, &val);
		val &= ~GENMASK(7, 0);
		val |= 0x51;
		ret = regmap_write(data->sb4, 0x08, val);
	}

	reset_control_deassert(data->pcie_stitch_reset);
	reset_control_deassert(data->pcie_reset);
	reset_control_deassert(data->pcie_core_reset);
	reset_control_deassert(data->pcie_power_reset);
	reset_control_deassert(data->pcie_nonstitch_reset);
	reset_control_deassert(data->pcie_phy_reset);
	reset_control_deassert(data->pcie_phy_mdio_reset);

	ret = clk_prepare_enable(data->clk);
	if (ret)
		return ret;

	val = SYS_CTR_TRAN_EN | SYS_CTR_PHY_MDIO_OE | SYS_CTR_DIR_CFG_EN;
	val &= ~SYS_CTR_PHY_MDIO_RSTN;
	writel(val, data->ctrl_base + REG_SYS_CTR);

	if ((data->info->cap & RTD129X_PCIE_CAP_PCIE_2_0) && data->gen == 1) {
		dev_info(&data->pdev->dev, "forcing gen1\n");

		val = readl_relaxed(data->ctrl_base + 0x0a0);
		val &= ~0xf;
		val |= 0x1;
		writel_relaxed(val, data->ctrl_base + 0x0a0);
	}

	if (!(data->quirks & RTD129X_PCIE_QUIRK_PHY_A))
		rtd129x_pcie_phy_reset(data);

	if (data->info->cap & RTD129X_PCIE_CAP_PCIE_2_0) {

		rtd129x_pcie_phy_write(data, 0, 0x3, 0x27f1);

		/* #F code, close SSC */
		rtd129x_pcie_phy_write(data, 0, 0x4, 0x52f5);

		/* #modify N code */
		rtd129x_pcie_phy_write(data, 0, 0x5, 0xead7);

		/* #modify CMU ICP (TX jitter) */
		rtd129x_pcie_phy_write(data, 0, 0x6, 0x000c);

		/* #modify CMU RS (TX jitter) */
		rtd129x_pcie_phy_write(data, 0, 0xa, 0xa653);

		/* #modify AMP */
		rtd129x_pcie_phy_write(data, 1, 0x0, 0xd466);

		/* #modify Rx parameter */
		rtd129x_pcie_phy_write(data, 0, 0x1, 0xa84a);

		/* #clk driving */
		rtd129x_pcie_phy_write(data, 1, 0xb, 0xb803);

		/* #EQ */
		rtd129x_pcie_phy_write(data, 2, 0x3, 0x27e9);

		/* #F code, close SSC */
		rtd129x_pcie_phy_write(data, 2, 0x4, 0x52f5);

		/* #modify N code */
		rtd129x_pcie_phy_write(data, 2, 0x5, 0xead7);

		/* #modify CMU ICP (TX jitter) */
		rtd129x_pcie_phy_write(data, 2, 0x6, 0x000c);

		/* #modify CMU RS (TX jitter) */
		rtd129x_pcie_phy_write(data, 2, 0xa, 0xa653);

		/* #modify AMP */
		rtd129x_pcie_phy_write(data, 3, 0x0, 0xd477);

		/* #modify Rx parameter */
		rtd129x_pcie_phy_write(data, 2, 0x1, 0xa84a);

		/* #clk driving */
		rtd129x_pcie_phy_write(data, 3, 0xb, 0xa803);
		rtd129x_pcie_phy_write(data, 2, 0x1a, 0x0122);

		phyval = 0x521c;
		rtd129x_pcie_phy_write(data, 0, 0x9, phyval);
		phyval &= ~BIT(9);
		rtd129x_pcie_phy_write(data, 0, 0x9, phyval);
		phyval |= BIT(9);
		rtd129x_pcie_phy_write(data, 0, 0x9, phyval);

		rtd129x_pcie_phy_write(data, 1, 0x4, 0x0828);
		rtd129x_pcie_phy_write(data, 3, 0x4, 0x0828);

	} else if (data->quirks & RTD129X_PCIE_QUIRK_PHY_A) {

		rtd129x_pcie_phy_write(data, 0, 0x8, 0x3181);
		rtd129x_pcie_phy_write(data, 0, 0x11, 0xb200);
		rtd129x_pcie_phy_write(data, 0, 0x0, 0x0010);

	} else {

		rtd129x_pcie_phy_write(data, 0, 0x0, 0x0090);
		rtd129x_pcie_phy_write(data, 0, 0x4, 0x4008);
		rtd129x_pcie_phy_write(data, 0, 0x8, 0x5ac1);
		rtd129x_pcie_phy_write(data, 0, 0xf, 0x32b4);
		rtd129x_pcie_phy_write(data, 0, 0x11, 0x7200);

	}

	/* after phy mdio set */
	gpiod_direction_output(data->reset_gpiod, 1);
	mdelay(100);
	gpiod_direction_output(data->reset_gpiod, 0);

	/* set to MMIO */
	val = SYS_CTR_PHY_MDIO_OE | SYS_CTR_APP_LTSSM_EN;
	if (false)
		val |= SYS_CTR_MM_IO_TYPE_IO | SYS_CTR_DIR_CFG_EN;
	else
		val |= SYS_CTR_TRAN_EN | SYS_CTR_MM_IO_TYPE_MM | SYS_CTR_PHY_MDIO_RSTN | SYS_CTR_INDIR_CFG_EN;
	writel(val, data->ctrl_base + REG_SYS_CTR);
	mdelay(50);

	/* #Link initial setting */
	writel(0x00010120, data->ctrl_base + 0x710);

	ret = readl_relaxed_poll_timeout(data->ctrl_base + REG_MAC_ST, val, (val & MAC_ST_XMLH_LINK_UP), 10, 60000);
	if (ret) {
		/*
		 * run28: bail out like the vendor driver does (reset the block
		 * and drop the clock) instead of returning success on a dead
		 * link -- otherwise pci_host_probe() fires config reads into a
		 * linkless port, and an indirect cfg cycle that never completes
		 * can stall the whole boot. An empty slot must be harmless.
		 */
		dev_err(&data->pdev->dev, "link down (0x%08x)\n", val);
		ret = -ENODEV;
		goto err_disable_clk;
	}

	writel_relaxed(data->cfg_res->start, data->ctrl_base + REG_PCI_BASE);
	writel(~(resource_size(data->cfg_res) - 1), data->ctrl_base + REG_PCI_MASK);

	if (data->trans_res)
		writel(data->trans_res->start, data->ctrl_base + REG_PCI_TRANS);

	dev_dbg(&data->pdev->dev, "base 0x%08x (0x%08x) -> 0x%08x\n",
		readl_relaxed(data->ctrl_base + REG_PCI_BASE),
		readl_relaxed(data->ctrl_base + REG_PCI_MASK),
		readl_relaxed(data->ctrl_base + REG_PCI_TRANS));

	/* Prevent PCIe hang if dllp error occurs */
	val = FIELD_PREP(DIR_EN_TIMEOUT_CNT_VALUE,
			 (data->info->cap & RTD129X_PCIE_CAP_PCIE_2_0) ? 0x10000 : 0x2000) |
	      DIR_EN_TIMEOUT_EN;
	writel_relaxed(val, data->ctrl_base + REG_DIR_EN);

	val = readl_relaxed(data->ctrl_base + REG_PHY_CTR);
	dev_info(&data->pdev->dev, "PHY_CTR 0x%08x\n", val);
	val &= ~PHY_CTR_REG_PLLDVR;
	val |= PHY_CTR_RX50_LINK | PHY_CTR_POW_PCIEX | PHY_CTR_REG_PLLDVR_ON_HOST;
	writel_relaxed(val, data->ctrl_base + REG_PHY_CTR);

	rtd129x_pcie_link_init(data);

	return 0;

err_disable_clk:
	reset_control_assert(data->pcie_phy_mdio_reset);
	reset_control_assert(data->pcie_phy_reset);
	reset_control_assert(data->pcie_nonstitch_reset);
	reset_control_assert(data->pcie_power_reset);
	reset_control_assert(data->pcie_core_reset);
	reset_control_assert(data->pcie_reset);
	reset_control_assert(data->pcie_stitch_reset);
	clk_disable_unprepare(data->clk);
	return ret;
}

static const struct rtd129x_pcie_quirks rtd129x_pcie_a00_quirks = {
	.flags = RTD129X_PCIE_QUIRK_PHY_A | RTD129X_PCIE_QUIRK_SB4,
};

static const struct rtd129x_pcie_quirks rtd129x_pcie_a0x_quirks = {
	.flags = RTD129X_PCIE_QUIRK_PHY_A,
};

/*
 * QuaStation port (run28): mainline 6.18 has no realtek chip-info /
 * soc_device driver, so read the silicon revision straight from the SB2
 * CHIP_REV register (SB2 + 0x204, vendor include/soc/realtek/rtk_cpu.h:
 * A00 = 0x00000000, A01 = 0x00010000, B00 = 0x00020000) through the sb2
 * syscon named by "realtek,sb2-syscon". Live QuaStation: 0x00010000 = A01.
 * Vendor pcie-rtd129x-slot1.c applies the SB4 tweak only on A00 and the
 * PHY_A table on A00/A01, which is what the two quirk sets below encode.
 */
#define RTD129X_SB2_CHIP_REV		0x204
#define RTD129X_CHIP_REV_A00		0x00000000
#define RTD129X_CHIP_REV_A01		0x00010000

static int rtd129x_pcie_get_quirks(struct platform_device *pdev, u32 *quirks)
{
	struct regmap *sb2;
	u32 rev;
	int ret;

	sb2 = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "realtek,sb2-syscon");
	if (IS_ERR(sb2))
		return dev_err_probe(&pdev->dev, PTR_ERR(sb2), "no realtek,sb2-syscon\n");

	ret = regmap_read(sb2, RTD129X_SB2_CHIP_REV, &rev);
	if (ret)
		return ret;

	switch (rev) {
	case RTD129X_CHIP_REV_A00:
		*quirks = rtd129x_pcie_a00_quirks.flags;
		break;
	case RTD129X_CHIP_REV_A01:
		*quirks = rtd129x_pcie_a0x_quirks.flags;
		break;
	default:
		*quirks = 0;
		break;
	}
	dev_info(&pdev->dev, "chip rev 0x%08x -> quirks 0x%x\n", rev, *quirks);
	return 0;
}

static const struct rtd129x_pcie_info rtd129x_pcie11 = {
};

static const struct rtd129x_pcie_info rtd129x_pcie20 = {
	.cap = RTD129X_PCIE_CAP_PCIE_2_0,
};

static const struct of_device_id rtd129x_pcie_dt_ids[] = {
	{ .compatible = "realtek,rtd1295-pcie-1.1", .data = &rtd129x_pcie11 },
	{ .compatible = "realtek,rtd1295-pcie-2.0", .data = &rtd129x_pcie20 },
	{ }
};


static int rtd129x_pcie_probe(struct platform_device *pdev)
{
	struct pci_host_bridge *bridge;
	struct rtd129x_pcie_priv *data;
	u32 quirks;
	int ret;

	ret = rtd129x_pcie_get_quirks(pdev, &quirks);
	if (ret)
		return ret;

	bridge = devm_pci_alloc_host_bridge(&pdev->dev, sizeof(*data));
	if (!bridge)
		return -ENOMEM;

	data = pci_host_bridge_priv(bridge);
	data->pdev = pdev;
	spin_lock_init(&data->mmio_lock);
	platform_set_drvdata(pdev, data);

	data->quirks = quirks;

	data->info = device_get_match_data(&pdev->dev);
	if (!data->info)
		return -EINVAL;

	data->gen = of_pci_get_max_link_speed(pdev->dev.of_node);
	if (data->gen < 0)
		return data->gen;

	data->ctrl_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->ctrl_base))
		return PTR_ERR(data->ctrl_base);

	data->cfg_base = devm_platform_get_and_ioremap_resource(pdev, 1, &data->cfg_res);
	if (IS_ERR(data->cfg_base))
		return PTR_ERR(data->cfg_base);

	data->trans_res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!data->trans_res)
		dev_warn(&pdev->dev, "no trans region specified\n");

	if (data->quirks & RTD129X_PCIE_QUIRK_SB4) {
		data->sb4 = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "sb4-syscon");
		if (IS_ERR(data->sb4)) {
			ret = PTR_ERR(data->sb4);
			if (ret == -EPROBE_DEFER)
				return ret;
			dev_warn(&pdev->dev, "no SB4 syscon specified\n");
			data->quirks &= ~RTD129X_PCIE_QUIRK_SB4;
		}
	}

	data->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->clk))
		return PTR_ERR(data->clk);

	data->pcie_stitch_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_stitch");
	if (IS_ERR(data->pcie_stitch_reset))
		return PTR_ERR(data->pcie_stitch_reset);

	data->pcie_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie");
	if (IS_ERR(data->pcie_reset))
		return PTR_ERR(data->pcie_reset);

	data->pcie_core_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_core");
	if (IS_ERR(data->pcie_core_reset))
		return PTR_ERR(data->pcie_core_reset);

	data->pcie_power_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_power");
	if (IS_ERR(data->pcie_power_reset))
		return PTR_ERR(data->pcie_power_reset);

	data->pcie_nonstitch_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_nonstitch");
	if (IS_ERR(data->pcie_nonstitch_reset))
		return PTR_ERR(data->pcie_nonstitch_reset);

	data->pcie_phy_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_phy");
	if (IS_ERR(data->pcie_phy_reset))
		return PTR_ERR(data->pcie_phy_reset);

	data->pcie_phy_mdio_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_phy_mdio");
	if (IS_ERR(data->pcie_phy_mdio_reset))
		return PTR_ERR(data->pcie_phy_mdio_reset);

	data->reset_gpiod = devm_gpiod_get(&pdev->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(data->reset_gpiod))
		return PTR_ERR(data->reset_gpiod);

	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0)
		return data->irq;

	reset_control_assert(data->pcie_stitch_reset);
	reset_control_assert(data->pcie_reset);
	reset_control_assert(data->pcie_core_reset);
	reset_control_assert(data->pcie_power_reset);
	reset_control_assert(data->pcie_nonstitch_reset);
	reset_control_assert(data->pcie_phy_reset);
	reset_control_assert(data->pcie_phy_mdio_reset);

	ret = rtd129x_pcie_init(data);
	if (ret) {
		dev_err(&pdev->dev, "init failed (%d)\n", ret);
		return ret;
	}

	ret = rtd129x_pcie_init_irq(data);
	if (ret) {
		dev_err(&pdev->dev, "irq init failed (%d)\n", ret);
		return ret;
	}

	bridge->sysdata = data;
	bridge->ops = &rtd129x_pcie_ops;

	ret = pci_host_probe(bridge);
	if (ret) {
		dev_err(&pdev->dev, "host probe failed (%d)\n", ret);
		return ret;
	}

#ifdef CONFIG_HOTPLUG_PCI
	ret = rtd129x_pcie_init_hotplug(data);
	if (ret) {
		dev_err(&pdev->dev, "hotplug init failed (%d)\n", ret);
		return ret;
	}
#endif

	return 0;
}

static struct platform_driver rtd129x_pcie_platform_driver = {
	.driver = {
		.name = "pcie-rtd129x",
		.of_match_table = rtd129x_pcie_dt_ids,
	},
	.probe = rtd129x_pcie_probe,
};
builtin_platform_driver(rtd129x_pcie_platform_driver);
