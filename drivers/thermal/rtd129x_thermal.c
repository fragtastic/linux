// SPDX-License-Identifier: GPL-2.0
/*
 * Realtek RTD129x thermal sensor
 *
 * Ported from the vendor 4.9 sensor-rtd129x.c (83 lines): two magic
 * writes to CTRL2 arm the sensor, STATUS1 returns an 18-bit signed
 * value in units of 1/1024 degC. Sensor block lives in scpu_wrapper.
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#include "thermal_hwmon.h"

#define TM_SENSOR_CTRL2		0x08
#define TM_SENSOR_STATUS1	0x18

#define RTD129X_SENSOR_ARM	0x01904001
#define RTD129X_SENSOR_GO	0x01924001

struct rtd129x_thermal {
	void __iomem *base;
};

static int rtd129x_thermal_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct rtd129x_thermal *priv = thermal_zone_device_priv(tz);
	u32 val;

	val = readl_relaxed(priv->base + TM_SENSOR_STATUS1) & GENMASK(17, 0);
	*temp = sign_extend32(val, 17) * 1000 / 1024;

	return 0;
}

static const struct thermal_zone_device_ops rtd129x_thermal_ops = {
	.get_temp = rtd129x_thermal_get_temp,
};

static int rtd129x_thermal_probe(struct platform_device *pdev)
{
	struct rtd129x_thermal *priv;
	struct thermal_zone_device *tz;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	writel(RTD129X_SENSOR_ARM, priv->base + TM_SENSOR_CTRL2);
	writel(RTD129X_SENSOR_GO, priv->base + TM_SENSOR_CTRL2);

	tz = devm_thermal_of_zone_register(&pdev->dev, 0, priv,
					   &rtd129x_thermal_ops);
	if (IS_ERR(tz))
		return dev_err_probe(&pdev->dev, PTR_ERR(tz),
				     "failed to register thermal zone\n");

	/*
	 * thermal_of registers DT zones with no_hwmon = true, so the zone
	 * is invisible to lm-sensors unless the driver adds the hwmon
	 * bridge itself (same pattern as qcom/tsens.c).
	 */
	devm_thermal_add_hwmon_sysfs(&pdev->dev, tz);

	return 0;
}

static const struct of_device_id rtd129x_thermal_dt_ids[] = {
	{ .compatible = "realtek,rtd129x-thermal-sensor" },
	{ }
};

static struct platform_driver rtd129x_thermal_driver = {
	.probe = rtd129x_thermal_probe,
	.driver = {
		.name = "rtd129x-thermal",
		.of_match_table = rtd129x_thermal_dt_ids,
	},
};
builtin_platform_driver(rtd129x_thermal_driver);
