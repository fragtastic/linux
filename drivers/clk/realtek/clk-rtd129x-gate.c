// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTD129x clock gate controller
 *
 * Each controller is a single 32-bit MMIO register in which one bit enables
 * one clock, so the DT index handed to a consumer is the bit position.  The
 * names come from "clock-output-names"; empty strings mark unimplemented bits
 * and are skipped.
 *
 * Derived from the vendor BSP driver (drivers/clk/realtek/cgc.c,
 * Copyright (C) 2017 Realtek Semiconductor Corporation, Cheng-Yu Lee).  The
 * vendor's clk_mmio_gate ops do a plain read-modify-write of BIT(bit_idx)
 * (drivers/clk/realtek/clk-mmio-gate.c, common.h clk_reg_update), which is
 * exactly what the generic clk-gate in clk-provider.h does, so this driver
 * just wires the generic gate up to the DT.
 *
 * The vendor also supports a write-enable variant (CGC_HW_USE_WRITE_EN, a
 * second bit at bit_idx+1 that must be set for the write to take), but that
 * is only selected when the DT node carries "has-write-en".  The RTD1295
 * device tree taken off this machine's running 4.9 kernel has no such
 * property on any of its four clk-en nodes, so the plain RMW path is the
 * only one that can be reached here and the variant is not implemented.
 *
 * Two other things the vendor driver has and this one does not:
 *
 *  - "ignore-pm-clocks", which excludes a gate from the mask the vendor
 *    restores in its resume_noirq handler.  The 4.9 DTB uses it once
 *    ("clk_en_hdmi" on the 0x9800000c node).  There is no suspend support
 *    here yet, so nothing reads it; wire it up before adding one.
 *
 *  - the sb2 hardware semaphore the vendor takes around register access
 *    (soc/realtek/rtk_mmio.h), which guards against the secure-side
 *    firmware touching the same register.  Unverified whether that matters
 *    on this board.
 *
 * Of the four clk-en nodes in the 4.9 DTB, 0x98000450 is deliberately left
 * out of the device tree: it carries a single gate ("clk_en_lsadc" at bit 2)
 * and nothing in this tree consumes it yet.
 */

#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>

/* one 32-bit register, one bit per gate */
#define RTD129X_CGC_MAX_GATES	32

struct rtd129x_cgc {
	void __iomem		*reg;
	spinlock_t		lock;	/* serialises the RMW on the shared register */
	struct clk_hw_onecell_data data;
};

static bool rtd129x_cgc_name_listed(struct device_node *np, const char *prop,
				    const char *name)
{
	struct property *p;
	const char *s;

	of_property_for_each_string(np, prop, p, s)
		if (!strcmp(s, name))
			return true;

	return false;
}

static void rtd129x_cgc_unmap(void *reg)
{
	iounmap(reg);
}

static int rtd129x_cgc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rtd129x_cgc *cgc;
	int num, i, ret;

	num = of_property_count_strings(np, "clock-output-names");
	if (num <= 0)
		return dev_err_probe(dev, num ?: -EINVAL,
				     "clock-output-names is missing or empty\n");
	if (num > RTD129X_CGC_MAX_GATES)
		return dev_err_probe(dev, -EINVAL,
				     "%d names for a %d-bit register\n",
				     num, RTD129X_CGC_MAX_GATES);

	cgc = devm_kzalloc(dev, struct_size(cgc, data.hws, num), GFP_KERNEL);
	if (!cgc)
		return -ENOMEM;

	spin_lock_init(&cgc->lock);
	cgc->data.num = num;

	/*
	 * of_iomap(), not devm_of_iomap(): these nodes live inside the "crt"
	 * and "iso" syscon windows, and devm_of_iomap() would request the
	 * region and collide with them.  Same reason the mux_intc node in this
	 * tree maps its window without reserving it.
	 */
	cgc->reg = of_iomap(np, 0);
	if (!cgc->reg)
		return dev_err_probe(dev, -ENOMEM, "failed to map register\n");

	ret = devm_add_action_or_reset(dev, rtd129x_cgc_unmap, cgc->reg);
	if (ret)
		return ret;

	for (i = 0; i < num; i++) {
		unsigned long flags = 0;
		const char *name;
		struct clk_hw *hw;

		ret = of_property_read_string_index(np, "clock-output-names",
						    i, &name);
		if (ret || !name || !name[0]) {
			/* unimplemented bit: leave a hole in the provider */
			cgc->data.hws[i] = ERR_PTR(-ENOENT);
			continue;
		}

		/*
		 * "ignore-unused-clocks" lists gates that must survive
		 * clk_disable_unused() even with no consumer bound.  Dropping
		 * this would let the core switch off clocks the machine is
		 * still running on.
		 */
		if (rtd129x_cgc_name_listed(np, "ignore-unused-clocks", name))
			flags |= CLK_IGNORE_UNUSED;

		/*
		 * No parent: the gate only passes a clock whose rate this
		 * register cannot describe, which is how the vendor driver
		 * registers them too.  Consumers use prepare/enable, not
		 * clk_get_rate().
		 */
		hw = devm_clk_hw_register_gate(dev, name, NULL, flags,
					       cgc->reg, i, 0, &cgc->lock);
		if (IS_ERR(hw))
			return dev_err_probe(dev, PTR_ERR(hw),
					     "failed to register %s (bit %d)\n",
					     name, i);

		cgc->data.hws[i] = hw;
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					   &cgc->data);
}

static const struct of_device_id rtd129x_cgc_match[] = {
	{ .compatible = "realtek,rtd1295-clock-gate" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtd129x_cgc_match);

static struct platform_driver rtd129x_cgc_driver = {
	.probe = rtd129x_cgc_probe,
	.driver = {
		.name = "clk-rtd129x-gate",
		.of_match_table = rtd129x_cgc_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(rtd129x_cgc_driver);
