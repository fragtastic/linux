// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek RTD1295 AHCI SATA platform driver
 *
 * Minimal glue on top of the standard AHCI platform driver: the RTD129x
 * AHCI block needs two vendor-extension register writes (offsets 0xf20 and
 * 0xC in the AHCI MMIO window, both undocumented outside the vendor tree)
 * before the SATA PHY will ever report a link. Without them the port stays
 * at SStatus 0 forever, silently, and the AHCI IRQ never fires even on
 * physical hotplug.
 *
 * Forward-ported from the vendor 4.9.330 tree's drivers/ata/ahci_rtk.c,
 * the RTD129X-only code path in rtk_sata_init(). The rest of that 757-line
 * driver (RTD1619/RTD1319 variants, runtime power-save, sysfs presence
 * attributes) is not needed for RTD1295 bring-up.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/ahci_platform.h>
#include <linux/libata.h>
#include <linux/workqueue.h>

#include "ahci.h"

#define DRV_NAME "ahci_rtd1295"

/*
 * Vendor-extension registers beyond the standard AHCI register block.
 * 0xf20 sits outside the DT "reg" window on purpose (kept narrow so it
 * doesn't overlap the sibling sata-phy node's own MMIO region), so it is
 * mapped separately and non-exclusively below, same as the vendor driver
 * does for its own one-off CRT register pokes.
 */
#define RTD1295_MASK_ERR_SEL		0xf20
#define RTD1295_MASK_ERR_SEL_VAL	0x3c300
#define RTD1295_PORT_MAP_FIX		0xC

static const struct ata_port_info ahci_rtd1295_port_info = {
	.flags		= AHCI_FLAG_COMMON,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_platform_ops,
};

static const struct scsi_host_template ahci_platform_sht = {
	AHCI_SHT(DRV_NAME),
};

/*
 * The vendor driver never calls ahci_platform_init_host() synchronously in
 * probe(): unless a "hostinit-mode" DT property is set (the Monarch board
 * doesn't set it), it defers host init -- i.e. the first COMRESET attempt
 * -- by 800ms via schedule_delayed_work(). That gap is load-bearing: with
 * it removed (host init called synchronously, as ahci_platform normally
 * does), the SATA link only ever comes up if something else (U-Boot's own
 * AHCI probe on its normal boot path) already trained/powered the drive
 * beforehand. On a genuinely cold link -- e.g. the USB-rescue-button boot
 * path, which skips U-Boot's SATA probe entirely -- the drive bay and PHY
 * apparently need this settling time before the first hardreset can
 * succeed. Reproduced here for the same reason.
 */
#define RTD1295_HOSTINIT_DELAY_MS	800

struct ahci_rtd1295_data {
	struct platform_device *pdev;
	struct ahci_host_priv *hpriv;
	struct delayed_work init_work;
};

static void ahci_rtd1295_init_work(struct work_struct *work)
{
	struct ahci_rtd1295_data *data =
		container_of(work, struct ahci_rtd1295_data, init_work.work);
	int rc;

	rc = ahci_platform_init_host(data->pdev, data->hpriv,
				      &ahci_rtd1295_port_info,
				      &ahci_platform_sht);
	if (rc)
		dev_err(&data->pdev->dev,
			"deferred host init failed: %d\n", rc);
}

static int ahci_rtd1295_quirk_init(struct platform_device *pdev,
				    struct ahci_host_priv *hpriv)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *ext;
	u32 val;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	ext = ioremap(res->start + RTD1295_MASK_ERR_SEL, 4);
	if (!ext)
		return -ENOMEM;
	dev_info(dev, "rx error select to mac original\n");
	writel(RTD1295_MASK_ERR_SEL_VAL, ext);
	iounmap(ext);

	val = readl(hpriv->mmio + RTD1295_PORT_MAP_FIX);
	writel(val | 0x3, hpriv->mmio + RTD1295_PORT_MAP_FIX);

	return 0;
}

static int ahci_rtd1295_probe(struct platform_device *pdev)
{
	struct ahci_rtd1295_data *data;
	struct ahci_host_priv *hpriv;
	int rc;

	hpriv = ahci_platform_get_resources(pdev, AHCI_PLATFORM_GET_RESETS);
	if (IS_ERR(hpriv))
		return PTR_ERR(hpriv);

	rc = ahci_platform_enable_resources(hpriv);
	if (rc)
		return rc;

	rc = ahci_rtd1295_quirk_init(pdev, hpriv);
	if (rc)
		goto disable_resources;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		rc = -ENOMEM;
		goto disable_resources;
	}
	data->pdev = pdev;
	data->hpriv = hpriv;
	INIT_DELAYED_WORK(&data->init_work, ahci_rtd1295_init_work);
	platform_set_drvdata(pdev, data);
	schedule_delayed_work(&data->init_work,
			      msecs_to_jiffies(RTD1295_HOSTINIT_DELAY_MS));

	return 0;

disable_resources:
	ahci_platform_disable_resources(hpriv);
	return rc;
}

static SIMPLE_DEV_PM_OPS(ahci_rtd1295_pm_ops, ahci_platform_suspend,
			  ahci_platform_resume);

static const struct of_device_id ahci_rtd1295_of_match[] = {
	{ .compatible = "realtek,rtd1295-ahci", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ahci_rtd1295_of_match);

static struct platform_driver ahci_rtd1295_driver = {
	.probe = ahci_rtd1295_probe,
	.remove = ata_platform_remove_one,
	.shutdown = ahci_platform_shutdown,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = ahci_rtd1295_of_match,
		.pm = &ahci_rtd1295_pm_ops,
	},
};
module_platform_driver(ahci_rtd1295_driver);

MODULE_DESCRIPTION("Realtek RTD1295 AHCI SATA platform driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ahci_rtd1295");
