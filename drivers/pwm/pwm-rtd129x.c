// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTD129x PWM controller
 *
 * Forward-ported from the vendor 4.9.330 tree's drivers/pwm/pwm-rtk.c to
 * the modern pwm_ops .apply()/.get_state() API. The vendor driver's ~700
 * lines of custom per-channel sysfs attributes (duplicating what mainline
 * already exposes generically under /sys/class/pwm/) are dropped; only
 * the register math and probe/apply/get_state core is kept.
 *
 * get_state() reads the live hardware registers rather than mirroring the
 * vendor's software-shadow state array: this board's SYS LED (channel 3)
 * is left running by the boot loader with "default-state = keep" in the
 * board DTS, and the PWM core calls get_state() once at pwmchip_add()
 * time to seed pwm->state before any consumer touches it -- a
 * freshly-probed zeroed shadow would misreport the LED as off.
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/time64.h>

#define RTD129X_PWM_NUM		4
#define RTD129X_PWM_BASE_FREQ_HZ	27000000

#define RTD129X_PWM_OCD		0x0	/* clkout_div, 8 bits/channel */
#define RTD129X_PWM_CD			0x4	/* clk_duty,   8 bits/channel */
#define RTD129X_PWM_CSD			0x8	/* clksrc_div, 4 bits/channel */

#define RTD129X_PWM_OCD_MASK		0xff
#define RTD129X_PWM_CD_MASK		0xff
#define RTD129X_PWM_CSD_MASK		0xf

struct rtd129x_pwm_chip {
	void __iomem *base;
};

static inline struct rtd129x_pwm_chip *to_rtd129x_pwm_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static void rtd129x_pwm_write_field(struct rtd129x_pwm_chip *pc, unsigned int reg,
				     unsigned int mask, unsigned int shift_unit,
				     unsigned int hwpwm, unsigned int val)
{
	unsigned int shift = hwpwm * shift_unit;
	u32 tmp = readl(pc->base + reg);

	tmp &= ~(mask << shift);
	tmp |= (val & mask) << shift;
	writel(tmp, pc->base + reg);
}

static unsigned int rtd129x_pwm_read_field(struct rtd129x_pwm_chip *pc, unsigned int reg,
					    unsigned int mask, unsigned int shift_unit,
					    unsigned int hwpwm)
{
	unsigned int shift = hwpwm * shift_unit;

	return (readl(pc->base + reg) >> shift) & mask;
}

/*
 * div = 2^(csd+1) * (ocd+1), real_freq = base_freq / div. Pick csd/ocd for
 * the finest achievable division of the requested period -- same
 * derivation as the vendor's set_real_freq_by_target_freq() (which found
 * the highest set bit of `div` by shifting until it hit bit 31; that's
 * just the bit position of its MSB), fed from period_ns directly instead
 * of a pre-converted target frequency.
 */
static void rtd129x_pwm_calc_div(u64 period_ns, unsigned int *ocd, unsigned int *csd)
{
	u32 target_freq = div64_u64(NSEC_PER_SEC, period_ns);
	u32 div = RTD129X_PWM_BASE_FREQ_HZ / max(target_freq, 1U);
	int msb = fls(div) - 1;

	*csd = clamp_t(int, msb - 8, 0, RTD129X_PWM_CSD_MASK);
	*ocd = clamp_t(int, (div >> (*csd + 1)) - 1, 0, RTD129X_PWM_OCD_MASK);
}

static int rtd129x_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			      const struct pwm_state *state)
{
	struct rtd129x_pwm_chip *pc = to_rtd129x_pwm_chip(chip);
	unsigned int hwpwm = pwm->hwpwm;
	unsigned int ocd = 0, csd = 0, cd = 0;

	if (state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;

	if (state->enabled && state->duty_cycle) {
		rtd129x_pwm_calc_div(state->period, &ocd, &csd);
		cd = clamp_t(u64, div64_u64((u64)state->duty_cycle * (ocd + 1),
					     state->period),
			     1, ocd + 1) - 1;
	}

	rtd129x_pwm_write_field(pc, RTD129X_PWM_OCD, RTD129X_PWM_OCD_MASK, 8, hwpwm, ocd);
	rtd129x_pwm_write_field(pc, RTD129X_PWM_CD, RTD129X_PWM_CD_MASK, 8, hwpwm, cd);
	rtd129x_pwm_write_field(pc, RTD129X_PWM_CSD, RTD129X_PWM_CSD_MASK, 4, hwpwm, csd);

	return 0;
}

static int rtd129x_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				  struct pwm_state *state)
{
	struct rtd129x_pwm_chip *pc = to_rtd129x_pwm_chip(chip);
	unsigned int hwpwm = pwm->hwpwm;
	unsigned int ocd, cd, csd;
	u32 div, freq;

	ocd = rtd129x_pwm_read_field(pc, RTD129X_PWM_OCD, RTD129X_PWM_OCD_MASK, 8, hwpwm);
	cd = rtd129x_pwm_read_field(pc, RTD129X_PWM_CD, RTD129X_PWM_CD_MASK, 8, hwpwm);
	csd = rtd129x_pwm_read_field(pc, RTD129X_PWM_CSD, RTD129X_PWM_CSD_MASK, 4, hwpwm);

	state->polarity = PWM_POLARITY_NORMAL;

	/* ocd == 0 is how apply() (and the boot loader) encodes "disabled". */
	if (ocd == 0) {
		state->enabled = false;
		state->period = 0;
		state->duty_cycle = 0;
		return 0;
	}

	div = BIT(csd + 1) * (ocd + 1);
	freq = RTD129X_PWM_BASE_FREQ_HZ / div;

	state->enabled = true;
	state->period = DIV_ROUND_UP(NSEC_PER_SEC, freq);
	state->duty_cycle = DIV_ROUND_UP((u64)state->period * (cd + 1), ocd + 1);

	return 0;
}

static const struct pwm_ops rtd129x_pwm_ops = {
	.apply = rtd129x_pwm_apply,
	.get_state = rtd129x_pwm_get_state,
};

static int rtd129x_pwm_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	struct rtd129x_pwm_chip *pc;

	chip = devm_pwmchip_alloc(&pdev->dev, RTD129X_PWM_NUM, sizeof(*pc));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	pc = to_rtd129x_pwm_chip(chip);

	pc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->base))
		return PTR_ERR(pc->base);

	chip->ops = &rtd129x_pwm_ops;

	return devm_pwmchip_add(&pdev->dev, chip);
}

static const struct of_device_id rtd129x_pwm_of_match[] = {
	{ .compatible = "realtek,rtd129x-pwm" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtd129x_pwm_of_match);

static struct platform_driver rtd129x_pwm_driver = {
	.driver = {
		.name = "rtd129x-pwm",
		.of_match_table = rtd129x_pwm_of_match,
	},
	.probe = rtd129x_pwm_probe,
};
module_platform_driver(rtd129x_pwm_driver);

MODULE_DESCRIPTION("Realtek RTD129x PWM controller driver");
MODULE_LICENSE("GPL");
