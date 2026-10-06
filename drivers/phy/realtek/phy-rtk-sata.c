// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTD129x SATA PHY (vendor-style DT binding)
 *
 * Ported from vendor kernel 4.9.330 (phy-rtk-sata.c) for bring-up.
 *
 * DT binding (vendor):
 *   sata_phy@... {
 *     compatible = "Realtek,rtk-sata-phy";
 *     reg = <...>, <...>;
 *     #phy-cells = <1>;
 *
 *     sata-phy@0 { reg = <0>; ... };
 *   };
 *
 * This is not upstream-quality. It's meant to get WD My Cloud Home booting
 * with a working SATA link.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define RTK_SATA_PHY_MAX	2
#define PHY_MAX_CLK		5
#define PHY_MAX_RST		5

#define PHY_CLK_EN		0x18
#define MDIO_CTR		0x60
#define MDIO_CTR1		0x64
#define PHY_SPD			0x68

struct phy_rtk_desc {
	unsigned int index;
	unsigned int param_size;
	unsigned int *param_table;
	unsigned int txdrv_size;
	unsigned int *txdrv_table;
	unsigned int rxsense_size;
	unsigned int *rxsense_table;
	unsigned int ssc_en;
	unsigned int speed;
	struct phy *phy;
	struct reset_control *rsts[PHY_MAX_RST];
};

struct phy_rtk_priv {
	void __iomem *base;
	void __iomem *sb2base;
	struct clk *clks[PHY_MAX_CLK];
	struct reset_control *rsts[PHY_MAX_RST];
	struct phy_rtk_desc **phys;
	struct device *dev;
	unsigned int nphys;
};

/*
 * Default parameter tables (copied from vendor; matches RTD129x defaults).
 * Kept as-is to minimize the number of moving parts during bring-up.
 */
static const unsigned int PHY_PARA_TABLE[] = {
	0x00001111, 0x00005111, 0x00009111,
	0x336a0511, 0x336a4511, 0x336a8511,
	0xE0700111, 0xE05C4111, 0xE04A8111,
	0x00150611, 0x00154611, 0x00158611,
	0xC6000A11, 0xC6004A11, 0xC6008A11,
	0x70000211, 0x70004211, 0x70008211,
	0xC6600A11, 0xC6604A11, 0xC6608A11,
	0x20041911, 0x20045911, 0x20049911,
	0x94aa2011, 0x94aa6011, 0x94aaa011,
	0x17171511, 0x17175511, 0x17179511,
	0x07701611, 0x07705611, 0x07709611,
	0x40002a11, 0x40006a11, 0x4000aa11,
	0x29001011, 0x29005011, 0x29009011,
	0x40000C11, 0x40004C11, 0x40008C11,
	0x00271711, 0x00275711, 0x00279711,
};

static const unsigned int TX_DRV_TABLE[][6] = {
	/* max_500mV */
	{ 0x94a52011, 0x94a56011, 0x94a5a011, 0x385a2111, 0x385a6111, 0x385aa111 },
	/* max_573mV */
	{ 0x94a62011, 0x94a66011, 0x94a6a011, 0x486a2111, 0x486a6111, 0x486aa111 },
	/* max_683mV */
	{ 0x94a72011, 0x94a76011, 0x94a7a011, 0x587a2111, 0x587a6111, 0x587aa111 },
	/* max_786mV */
	{ 0x94aa2011, 0x94aa6011, 0x94aaa011, 0x88aa2111, 0x88aa6111, 0x88aaa111 },
};

static const unsigned int PHY_SENSITIVITY[][6] = {
	{ 0x72100911, 0x72104911, 0x72108911, 0x27730311, 0x27684311, 0x27688311 },
	{ 0x42100911, 0x42104911, 0x42108911, 0x276f0311, 0x276d4311, 0x276d8311 },
	{ 0x42100911, 0x42104911, 0x42108911, 0x276a0311, 0x276a4311, 0x27688311 },
};

static const unsigned int SSCEN_SET_TABLE[] = {
	0x738E0411, 0x738E0411, 0x738E8411,
	0x35910811, 0x35914811, 0x35918811,
	0x02342711, 0x02346711, 0x0234a711,
};

static const unsigned int SSCDIS_SET_TABLE[] = {
	0x538E0411, 0x538E4411, 0x538E8411,
};

/*
 * SB2 bus-gate bits for this PHY instance. Without these open, the MDIO
 * bus segment this PHY's MDIO_CTR/MDIO_CTR1/PHY_SPD registers sit behind
 * doesn't respond: every readl() on it stalls (observed as several tens
 * of ms per access -- not the driver's own udelay(10) polling interval,
 * which is negligible in comparison) before eventually giving back a bus
 * abort/poison pattern, and downstream AHCI HBA register reads on the
 * same bus segment do the same (0xdeadbeef).
 *
 * The mainline phy framework calls phy_init() before phy_power_on() (see
 * ahci_platform_enable_phys()), but phy_rtk_sata_init() below does all
 * its MDIO_CTR programming -- this gate needs to already be open by
 * then, not just by the time power_on() runs afterwards. This driver is
 * shared, byte-for-byte identical, between two board ports:
 * symops/pelican-6.18 (WD My Cloud Home Duo, dual-bay, RTD1296) and
 * symops/monarch-6.18 (WD My Cloud Home, single-bay, RTD1295) --
 * matching the vendor 4.9.330 source's non-RTD129X chip_id path, which
 * calls this same gate-open unconditionally from both .init() and
 * .power_on() too. Whether the call in .init() actually does anything
 * depends on the bootloader: confirmed on real Duo hardware, where this
 * gate is NOT pre-opened, that skipping it here is fatal -- every one of
 * the ~63 MDIO_CTR writes phy_rtk_sata_init() makes per PHY times out
 * and logs "mdio busy" (ignored -- see write_mdio_reg()), stretching
 * each PHY's init to ~3.3s, and the AHCI controller reset that follows
 * fails outright with the same poison value. On Monarch the bootloader
 * already opens this gate, so the call in .init() is a no-op there in
 * practice -- kept anyway for the two ports' correctness and parity.
 */
static void phy_rtk_sata_sb2_gate_open(struct phy_rtk_priv *priv, unsigned int index)
{
	u32 reg;

	reg = readl(priv->sb2base);
	if (index == 0)
		reg |= BIT(0) | BIT(2) | BIT(4) | BIT(8);
	else
		reg |= BIT(1) | BIT(3) | BIT(5);
	writel(reg, priv->sb2base);
}

static int write_mdio_reg(u32 value, void __iomem *address)
{
	unsigned int cnt = 0;

	while (readl(address) & 0x80) {
		udelay(10);
		if (++cnt > 5) {
			pr_err("rtk-sata-phy: mdio busy\n");
			return -ETIMEDOUT;
		}
	}

	writel(value, address);
	return 0;
}

static struct phy *phy_rtk_sata_xlate(struct device *dev,
				      const struct of_phandle_args *args)
{
	struct phy_rtk_priv *priv = dev_get_drvdata(dev);
	unsigned int i;

	for (i = 0; i < priv->nphys; i++) {
		if (priv->phys[i]->index == args->args[0])
			return priv->phys[i]->phy;
	}

	return ERR_PTR(-ENODEV);
}

static int phy_rtk_sata_init(struct phy *phy)
{
	struct phy_rtk_desc *desc = phy_get_drvdata(phy);
	struct phy_rtk_priv *priv = dev_get_drvdata(phy->dev.parent);
	void __iomem *base = priv->base;
	unsigned int *table;
	unsigned int reg;
	unsigned int size;
	int i;

	/* Must happen before any MDIO_CTR access below -- see the comment
	 * above phy_rtk_sata_sb2_gate_open().
	 */
	phy_rtk_sata_sb2_gate_open(priv, desc->index);

	/* select PHY instance */
	writel(desc->index, base + MDIO_CTR1);

	for (i = 0; i < desc->param_size; i++)
		write_mdio_reg(desc->param_table[i], base + MDIO_CTR);

	for (i = 0; i < desc->txdrv_size; i++)
		write_mdio_reg(desc->txdrv_table[i], base + MDIO_CTR);

	for (i = 0; i < desc->rxsense_size; i++)
		write_mdio_reg(desc->rxsense_table[i], base + MDIO_CTR);

	table = (unsigned int *)(desc->ssc_en ? SSCEN_SET_TABLE : SSCDIS_SET_TABLE);
	size = desc->ssc_en ? ARRAY_SIZE(SSCEN_SET_TABLE) : ARRAY_SIZE(SSCDIS_SET_TABLE);
	for (i = 0; i < size; i++)
		write_mdio_reg(table[i], base + MDIO_CTR);

	/* speed-limit: 0 = gen1, 2 = gen2, 1 = gen3 (?) vendor behavior */
	if (desc->speed == 0)
		writel(0xA, base + PHY_SPD);
	else if (desc->speed == 2)
		writel(0x5, base + PHY_SPD);
	else if (desc->speed == 1)
		writel(0x0, base + PHY_SPD);

	/* enable clock bits if present */
	reg = readl(base + PHY_CLK_EN);
	writel(reg, base + PHY_CLK_EN);

	dev_info(priv->dev, "rtk-sata-phy: init phy%u OK\n", desc->index);
	return 0;
}

static int phy_rtk_sata_power_on(struct phy *phy)
{
	struct phy_rtk_desc *desc = phy_get_drvdata(phy);
	struct phy_rtk_priv *priv = dev_get_drvdata(phy->dev.parent);
	int i;

	for (i = 0; i < PHY_MAX_RST; i++) {
		if (!desc->rsts[i])
			break;
		reset_control_deassert(desc->rsts[i]);
	}

	phy_rtk_sata_sb2_gate_open(priv, desc->index);

	return 0;
}

static int phy_rtk_sata_power_off(struct phy *phy)
{
	struct phy_rtk_desc *desc = phy_get_drvdata(phy);
	int i;

	for (i = 0; i < PHY_MAX_RST; i++) {
		if (!desc->rsts[i])
			break;
		reset_control_assert(desc->rsts[i]);
	}

	return 0;
}

static int phy_rtk_sata_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	/* vendor uses this for power save; keep minimal semantics */
	(void)submode;
	return mode ? phy_rtk_sata_power_off(phy) : phy_rtk_sata_power_on(phy);
}

static const struct phy_ops phy_rtk_sata_ops = {
	.init		= phy_rtk_sata_init,
	.power_on	= phy_rtk_sata_power_on,
	.power_off	= phy_rtk_sata_power_off,
	.set_mode	= phy_rtk_sata_set_mode,
	.owner		= THIS_MODULE,
};

static int get_phy_parameter(struct device *dev, struct device_node *node,
			     struct phy_rtk_desc *desc)
{
	const void *prop;
	unsigned int drv_level = 3;
	unsigned int size;
	unsigned int *table;
	unsigned int num = 1;

	prop = of_get_property(node, "phy-param", NULL);
	size = prop ? of_property_count_u32_elems(node, "phy-param") :
		      ARRAY_SIZE(PHY_PARA_TABLE);

	table = devm_kcalloc(dev, size, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	if (!prop)
		memcpy(table, PHY_PARA_TABLE, sizeof(PHY_PARA_TABLE));
	else
		of_property_read_u32_array(node, "phy-param", table, size);

	desc->param_size = size;
	desc->param_table = table;

	prop = of_get_property(node, "tx-driving-tbl", NULL);
	size = prop ? of_property_count_u32_elems(node, "tx-driving-tbl") :
		      ARRAY_SIZE(TX_DRV_TABLE[0]);

	table = devm_kcalloc(dev, size, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	if (!prop) {
		of_property_read_u32(node, "tx-driving", &drv_level);
		if (drv_level >= ARRAY_SIZE(TX_DRV_TABLE))
			drv_level = 3;
		memcpy(table, TX_DRV_TABLE[drv_level], sizeof(TX_DRV_TABLE[0]));
	} else {
		of_property_read_u32_array(node, "tx-driving-tbl", table, size);
	}

	desc->txdrv_size = size;
	desc->txdrv_table = table;

	prop = of_get_property(node, "rx-sense-tbl", NULL);
	size = prop ? of_property_count_u32_elems(node, "rx-sense-tbl") :
		      ARRAY_SIZE(PHY_SENSITIVITY[0]);

	table = devm_kcalloc(dev, size, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	if (!prop) {
		memcpy(table, PHY_SENSITIVITY[num], sizeof(PHY_SENSITIVITY[0]));
	} else {
		of_property_read_u32_array(node, "rx-sense-tbl", table, size);
	}

	desc->rxsense_size = size;
	desc->rxsense_table = table;

	of_property_read_u32(node, "spread-spectrum", &desc->ssc_en);
	of_property_read_u32(node, "speed-limit", &desc->speed);

	return 0;
}

static void phy_rtk_sata_enable(struct phy_rtk_priv *priv)
{
	int i;

	for (i = 0; i < PHY_MAX_CLK; i++) {
		if (!priv->clks[i])
			break;
		clk_prepare_enable(priv->clks[i]);
	}

	for (i = 0; i < PHY_MAX_RST; i++) {
		if (!priv->rsts[i])
			break;
		reset_control_deassert(priv->rsts[i]);
	}
}

static int phy_rtk_sata_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *child;
	struct phy_rtk_priv *priv;
	struct phy_provider *provider;
	struct clk *clk;
	struct reset_control *rst;
	unsigned int phy_id, i, cnt = 0;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->sb2base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(priv->sb2base))
		return PTR_ERR(priv->sb2base);

	/* optional clocks/resets */
	for (i = 0; i < PHY_MAX_CLK; i++) {
		clk = of_clk_get(dev->of_node, i);
		if (IS_ERR(clk))
			break;
		priv->clks[i] = clk;
	}

	for (i = 0; i < PHY_MAX_RST; i++) {
		rst = of_reset_control_get_by_index(dev->of_node, i);
		if (IS_ERR(rst))
			break;
		priv->rsts[i] = rst;
	}

	phy_rtk_sata_enable(priv);

	priv->nphys = of_get_child_count(dev->of_node);
	if (!priv->nphys || priv->nphys > RTK_SATA_PHY_MAX)
		return -ENODEV;

	priv->dev = dev;
	priv->phys = devm_kcalloc(dev, priv->nphys, sizeof(*priv->phys), GFP_KERNEL);
	if (!priv->phys)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	for_each_available_child_of_node(dev->of_node, child) {
		struct phy_rtk_desc *desc;
		struct phy *phy;

		desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
		if (!desc)
			return -ENOMEM;

		if (of_property_read_u32(child, "reg", &phy_id))
			return -EINVAL;
		if (phy_id >= RTK_SATA_PHY_MAX)
			return -EINVAL;

		for (i = 0; i < PHY_MAX_RST; i++) {
			rst = of_reset_control_get_by_index(child, i);
			if (IS_ERR(rst))
				break;
			desc->rsts[i] = rst;
			reset_control_assert(desc->rsts[i]);
		}

		ret = get_phy_parameter(dev, child, desc);
		if (ret)
			return ret;

		phy = devm_phy_create(dev, NULL, &phy_rtk_sata_ops);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		desc->phy = phy;
		desc->index = phy_id;
		phy_set_drvdata(phy, desc);
		priv->phys[cnt++] = desc;
	}

	provider = devm_of_phy_provider_register(dev, phy_rtk_sata_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id phy_rtk_sata_of_match[] = {
	{ .compatible = "Realtek,rtk-sata-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, phy_rtk_sata_of_match);

static struct platform_driver phy_rtk_sata_driver = {
	.probe = phy_rtk_sata_probe,
	.driver = {
		.name = "phy-rtk-sata",
		.of_match_table = phy_rtk_sata_of_match,
	},
};
module_platform_driver(phy_rtk_sata_driver);

MODULE_DESCRIPTION("Realtek RTD129x SATA PHY (vendor) driver");
MODULE_LICENSE("GPL");


