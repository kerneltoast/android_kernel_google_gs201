// SPDX-License-Identifier: GPL-2.0-only
/*
 *  exynos-reboot.c - Samsung Exynos SoC reset code
 *
 * Copyright (c) 2019-2020 Samsung Electronics Co., Ltd.
 *
 * Author: Hyunki Koo <hyunki00.koo@samsung.com>
 *	   Youngmin Nam <youngmin.nam@samsung.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#if IS_ENABLED(CONFIG_GS_ACPM)
#include <soc/google/acpm_ipc_ctrl.h>
#endif
#include <soc/google/exynos-el3_mon.h>
/* TODO: temporary workaround. must remove. see b/169128860  */
#include <linux/soc/samsung/exynos-smc.h>

static struct regmap *pmureg;
static u32 reboot_offset, reboot_trigger;
static u32 reboot_cmd_offset;
static u32 shutdown_offset, shutdown_trigger;
static phys_addr_t pmu_alive_base;

static void exynos_power_off(void)
{
	u32 poweroff_try = 0;
	int power_gpio = -1;
	unsigned int keycode = 0;
	struct device_node *np, *pp;
	int ret;

	np = of_find_node_by_path("/gpio_keys");
	if (!np)
		return;

	for_each_child_of_node(np, pp) {
		if (!of_find_property(pp, "gpios", NULL))
			continue;
		of_property_read_u32(pp, "linux,code", &keycode);
		if (keycode == KEY_POWER) {
			pr_info("%s: <%u>\n", __func__,  keycode);
			power_gpio = of_get_gpio(pp, 0);
			break;
		}
	}

	of_node_put(np);

	if (!gpio_is_valid(power_gpio)) {
		pr_err("Couldn't find power key node\n");
		return;
	}

	while (1) {
		/* wait for power button release */
		if (gpio_get_value(power_gpio)) {
#if IS_ENABLED(CONFIG_GS_ACPM)
			exynos_acpm_reboot();
#endif
			pr_emerg("Set PS_HOLD Low.\n");
			ret = rmw_priv_reg(pmu_alive_base + shutdown_offset,
					   shutdown_trigger, 0);
			/* TODO: remove following fallback. see b/169128860 */
			if (ret)
				regmap_update_bits(pmureg, shutdown_offset,
						   shutdown_trigger, 0);

			++poweroff_try;
			pr_emerg("Should not reach here! (poweroff_try:%d)\n",
				 poweroff_try);
		} else {
			/*
			 * if power button is not released,
			 * wait and check TA again
			 */
			pr_info("PWR Key is not released.\n");
		}
		mdelay(1000);
	}
}

#define EXYNOS_PMU_SYSIP_DAT0		(0x0810)
#define REBOOT_MODE_NORMAL		(0x00)
#define REBOOT_MODE_CHARGE		(0x0A)
#define REBOOT_MODE_FASTBOOT		(0xFC)
#define REBOOT_MODE_FACTORY		(0xFD)
#define REBOOT_MODE_RECOVERY		(0xFF)

static void exynos_reboot_parse(const char *cmd)
{
	int ret;

	if (cmd) {
		pr_info("Reboot command: '%s'\n", cmd);

		if (!strcmp(cmd, "charge")) {
			ret = set_priv_reg(pmu_alive_base + reboot_cmd_offset,
					   REBOOT_MODE_CHARGE);
			/* TODO: remove following fallback. see b/169128860 */
			if (ret)
				regmap_write(pmureg, reboot_cmd_offset,
					     REBOOT_MODE_CHARGE);
		} else if (!strcmp(cmd, "bootloader") ||
			   !strcmp(cmd, "fastboot") ||
			   !strcmp(cmd, "bl") ||
			   !strcmp(cmd, "fb")) {
			ret = set_priv_reg(pmu_alive_base + reboot_cmd_offset,
					   REBOOT_MODE_FASTBOOT);
			if (ret) {
				pr_warn("%s(): priv_reg: failed to set addr: 0x%lx\n",
					__func__, pmu_alive_base + reboot_cmd_offset);
				regmap_write(pmureg, reboot_cmd_offset,
					     REBOOT_MODE_FASTBOOT);
			}
		} else if (!strcmp(cmd, "recovery")) {
			ret = set_priv_reg(pmu_alive_base + reboot_cmd_offset,
					   REBOOT_MODE_RECOVERY);
			/* TODO: remove following fallback. see b/169128860 */
			if (ret)
				regmap_write(pmureg, reboot_cmd_offset,
					     REBOOT_MODE_RECOVERY);
		} else {
			pr_err("Unknown reboot command: '%s'\n", cmd);
		}
	}
}

static int exynos_restart_handler(struct notifier_block *this,
				  unsigned long mode, void *cmd)
{
	int ret;
#if IS_ENABLED(CONFIG_GS_ACPM)
	exynos_acpm_reboot();
#endif
	exynos_reboot_parse(cmd);

	/* Do S/W Reset */
	pr_emerg("%s: Exynos SoC reset right now\n", __func__);

	ret = set_priv_reg(pmu_alive_base + reboot_offset, reboot_trigger);

	/* TODO: this is a temporary workaround. must remove. see b/169128860 */
	if (ret == SMC_CMD_PRIV_REG || ret == -EINVAL)
		regmap_write(pmureg, reboot_offset, reboot_trigger);

	while (1)
		wfi();

	return NOTIFY_DONE;
}

static struct notifier_block exynos_restart_nb = {
	.notifier_call = exynos_restart_handler,
	.priority = 128,
};

static int exynos_reboot_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *syscon_np;
	struct resource res;
	int err;

	pmureg = syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(pmureg)) {
		dev_err(dev, "Fail to get regmap of PMU\n");
		return PTR_ERR(pmureg);
	}

	syscon_np = of_parse_phandle(np, "syscon", 0);
	if (!syscon_np) {
		dev_err(dev, "syscon device node not found\n");
		return -EINVAL;
	}

	if (of_address_to_resource(syscon_np, 0, &res)) {
		dev_err(dev, "failed to get syscon base address\n");
		return -ENOMEM;
	}

	pmu_alive_base = res.start;

	if (of_property_read_u32(np, "reboot-offset", &reboot_offset) < 0) {
		pr_err("failed to find reboot-offset property\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "reboot-trigger", &reboot_trigger) < 0) {
		pr_err("failed to find reboot-trigger property\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "shutdown-offset", &shutdown_offset) < 0) {
		pr_err("failed to find shutdown-offset property\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "shutdown-trigger",
				 &shutdown_trigger) < 0) {
		pr_err("failed to find shutdown-trigger property\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "reboot-cmd-offset",
				 &reboot_cmd_offset) < 0) {
		pr_info("failed to find reboot-offset property, using default\n");
		reboot_cmd_offset = EXYNOS_PMU_SYSIP_DAT0;
	}

	err = register_restart_handler(&exynos_restart_nb);
	if (err) {
		dev_err(dev, "cannot register restart handler (err=%d)\n",
			err);
	}
	pm_power_off = exynos_power_off;

	dev_info(dev, "register restart handler successfully\n");

	return err;
}

static const struct of_device_id exynos_reboot_of_match[] = {
	{ .compatible = "samsung,exynos-reboot" },
	{}
};

static struct platform_driver exynos_reboot_driver = {
	.probe = exynos_reboot_probe,
	.driver = {
		.name = "exynos-reboot",
		.of_match_table = exynos_reboot_of_match,
	},
};
module_platform_driver(exynos_reboot_driver);

MODULE_DESCRIPTION("Exynos Reboot driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:exynos-reboot");
