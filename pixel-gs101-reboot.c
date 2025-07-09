// SPDX-License-Identifier: GPL-2.0-only
/*
 *  pixel-reboot.c - Google Pixel SoC reset code
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
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#if IS_ENABLED(CONFIG_GS_ACPM)
#include <soc/google/acpm_ipc_ctrl.h>
#endif
#include <soc/google/exynos-el3_mon.h>
#include "../../bms/google_bms.h"

#define EXYNOS_PMU_SYSIP_DAT0		(0x0810)

#define BMS_RSBM_VALID			BIT(31)

#define DUMP_GPR_MODE			(0xDAB)

static struct regmap *pmureg;
static u32 warm_reboot_offset, warm_reboot_trigger;
static u32 cold_reboot_offset, cold_reboot_trigger;
static u32 reboot_cmd_offset;
static u32 shutdown_offset, shutdown_trigger;
static u32 dump_gpr_offset;
static phys_addr_t pmu_alive_base;
static bool rsbm_supported;
static bool force_warm_reboot_on_thermal_shutdown;

enum pon_reboot_mode {
	REBOOT_MODE_NORMAL		= 0x00,
	REBOOT_MODE_CHARGE		= 0x0A,

	REBOOT_MODE_DMVERITY_CORRUPTED	= 0x50,
	REBOOT_MODE_SHUTDOWN_THERMAL	= 0x51,
	REBOOT_MODE_AB_UPDATE		= 0x52,

	REBOOT_MODE_RESCUE		= 0xF9,
	REBOOT_MODE_FASTBOOT		= 0xFA,
	REBOOT_MODE_BOOTLOADER		= 0xFC,
	REBOOT_MODE_FACTORY		= 0xFD,
	REBOOT_MODE_RECOVERY		= 0xFF,
};

static void pixel_power_off(void)
{
	u32 poweroff_try = 0;
	int power_gpio = -1;
	unsigned int keycode = 0;
	struct device_node *np, *pp;

	np = of_find_node_by_path("/gpio_keys");
	if (!np)
		return;

	for_each_child_of_node(np, pp) {
		if (!of_find_property(pp, "gpios", NULL))
			continue;
		of_property_read_u32(pp, "linux,code", &keycode);
		if (keycode == KEY_POWER) {
			pr_info("%s: <%u>\n", __func__, keycode);
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
			rmw_priv_reg(pmu_alive_base + shutdown_offset, shutdown_trigger, 0);
			++poweroff_try;
			pr_emerg("Should not reach here! (poweroff_try:%d)\n", poweroff_try);
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

static void pixel_reboot_mode_set(u32 val)
{
	int ret;
	u32 mode;
	phys_addr_t reboot_cmd_addr = pmu_alive_base + reboot_cmd_offset;

	set_priv_reg(reboot_cmd_addr, val);

	if (s2mpg10_get_rev_id() > S2MPG10_EVT0 && rsbm_supported) {
		mode = val | BMS_RSBM_VALID;
		ret = gbms_storage_write(GBMS_TAG_RSBM, &mode, sizeof(mode));
		if (ret < 0)
			pr_err("%s(): failed to write gbms storage: %d(%d)\n", __func__,
			       GBMS_TAG_RSBM, ret);
	}
}

static void pixel_reboot_parse(const char *cmd)
{
	if (cmd) {
		u32 value = U32_MAX;
		bool force_warm_reboot = false;

		pr_info("Reboot command: '%s'\n", cmd);

		if (!strcmp(cmd, "charge")) {
			value = REBOOT_MODE_CHARGE;
		} else if (!strcmp(cmd, "bootloader")) {
			value = REBOOT_MODE_BOOTLOADER;
		} else if (!strcmp(cmd, "fastboot")) {
			value = REBOOT_MODE_FASTBOOT;
		} else if (!strcmp(cmd, "recovery")) {
			value = REBOOT_MODE_RECOVERY;
		} else if (!strcmp(cmd, "dm-verity device corrupted")) {
			value = REBOOT_MODE_DMVERITY_CORRUPTED;
		}  else if (!strcmp(cmd, "rescue")) {
			value = REBOOT_MODE_RESCUE;
		} else if (!strncmp(cmd, "shutdown-thermal", strlen("shutdown-thermal")) ||
			   !strncmp(cmd, "shutdown,thermal", strlen("shutdown,thermal"))) {
			if (force_warm_reboot_on_thermal_shutdown)
				force_warm_reboot = true;
			value = REBOOT_MODE_SHUTDOWN_THERMAL;
		} else if (!strcmp(cmd, "reboot-ab-update")) {
			value = REBOOT_MODE_AB_UPDATE;
		} else if (!strcmp(cmd, "from_fastboot") ||
			   !strcmp(cmd, "shell") ||
			   !strcmp(cmd, "userrequested") ||
			   !strcmp(cmd, "userrequested,fastboot") ||
			   !strcmp(cmd, "userrequested,recovery") ||
			   !strcmp(cmd, "userrequested,recovery,ui")) {
			value = REBOOT_MODE_NORMAL;
		} else {
			pr_err("Unknown reboot command: '%s'\n", cmd);
		}

		/* check for warm_reboot */
		if (force_warm_reboot)
			reboot_mode = REBOOT_WARM;

		if (value != U32_MAX)
			pixel_reboot_mode_set(value);
	}
}

static int pixel_reboot_handler(struct notifier_block *nb, unsigned long mode, void *cmd)
{
	u32 data;
	int ret;

	ret = gbms_storage_read(GBMS_TAG_RSBM, &data, sizeof(data));
	if (ret < 0)
		pr_err("%s(): failed to read gbms storage: %d(%d)\n", __func__, GBMS_TAG_RSBM, ret);

	rsbm_supported = ret != -ENOENT;

	pixel_reboot_parse(cmd);

	return NOTIFY_DONE;
}

static struct notifier_block pixel_reboot_nb = {
	.notifier_call = pixel_reboot_handler,
	.priority = INT_MAX,
};

static int pixel_restart_handler(struct notifier_block *this, unsigned long mode, void *cmd)
{
#if IS_ENABLED(CONFIG_GS_ACPM)
	exynos_acpm_reboot();
#endif

	/* Do S/W Reset */
	pr_emerg("%s: Pixel SoC reset right now\n", __func__);

	if (reboot_mode == REBOOT_WARM || reboot_mode == REBOOT_SOFT)
		set_priv_reg(pmu_alive_base + dump_gpr_offset, DUMP_GPR_MODE);
	else
		set_priv_reg(pmu_alive_base + dump_gpr_offset, 0x0);

	if (s2mpg10_get_rev_id() == S2MPG10_EVT0 || !rsbm_supported ||
	    reboot_mode == REBOOT_WARM || reboot_mode == REBOOT_SOFT) {
		set_priv_reg(pmu_alive_base + warm_reboot_offset, warm_reboot_trigger);
	} else {
		pr_emerg("Set PS_HOLD Low.\n");
		mdelay(2);
		rmw_priv_reg(pmu_alive_base + cold_reboot_offset, cold_reboot_trigger, 0);
	}

	while (1)
		wfi();

	return NOTIFY_DONE;
}

static struct notifier_block pixel_restart_nb = {
	.notifier_call = pixel_restart_handler,
	.priority = 130,
};

static int pixel_reboot_probe(struct platform_device *pdev)
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

	if (of_property_read_u32(np, "swreset-system-offset", &warm_reboot_offset) < 0) {
		dev_err(dev, "failed to find swreset-system-offset property\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "swreset-system-trigger", &warm_reboot_trigger) < 0) {
		dev_err(dev, "failed to find swreset-system-trigger property\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "pshold-control-offset", &cold_reboot_offset) < 0) {
		dev_err(dev, "failed to find pshold-control-offset property\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "pshold-control-trigger", &cold_reboot_trigger) < 0) {
		dev_err(dev, "failed to find shutdown-trigger property\n");
		return -EINVAL;
	}

	shutdown_offset = cold_reboot_offset;
	shutdown_trigger = cold_reboot_trigger;

	if (of_property_read_u32(np, "reboot-cmd-offset", &reboot_cmd_offset) < 0) {
		dev_info(dev, "failed to find reboot-offset property, using default\n");
		reboot_cmd_offset = EXYNOS_PMU_SYSIP_DAT0;
	}

	if (of_property_read_u32(np, "dump-gpr-offset", &dump_gpr_offset) < 0) {
		dev_err(dev, "failed to find dump-gpr-offset property\n");
		return -EINVAL;
	}

	force_warm_reboot_on_thermal_shutdown = of_property_read_bool(np,
						"force-warm-reboot-on-thermal-shutdown");

	err = register_reboot_notifier(&pixel_reboot_nb);
	if (err) {
		dev_err(dev, "cannot register reboot handler (err=%d)\n", err);
		return err;
	}

	err = register_restart_handler(&pixel_restart_nb);
	if (err) {
		dev_err(dev, "cannot register restart handler (err=%d)\n", err);
		unregister_reboot_notifier(&pixel_reboot_nb);
		return err;
	}

	pm_power_off = pixel_power_off;
	dev_info(dev, "register restart handler successfully\n");

	return 0;
}

static const struct of_device_id pixel_reboot_of_match[] = {
	{ .compatible = "google,pixel-reboot" },
	{}
};
MODULE_DEVICE_TABLE(of, pixel_reboot_of_match);

static struct platform_driver pixel_reboot_driver = {
	.probe = pixel_reboot_probe,
	.driver = {
		.name = "pixel-reboot",
		.of_match_table = pixel_reboot_of_match,
	},
};
module_platform_driver(pixel_reboot_driver);

MODULE_DESCRIPTION("Pixel Reboot driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pixel-reboot");
