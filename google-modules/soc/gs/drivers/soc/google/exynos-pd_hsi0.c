// SPDX-License-Identifier: GPL-2.0-only
/*
 * Regulator control for Exynos PM HSI0 domain
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <soc/google/exynos-pd_hsi0.h>

static struct exynos_pd_hsi0_data *exynos_pd_hsi0_get_struct(void)
{
	struct device_node *np;
	struct platform_device *pdev;

	np = of_find_compatible_node(NULL, NULL, "exynos-pd-hsi0");
	if (np) {
		pdev = of_find_device_by_node(np);
		of_node_put(np);
		if (pdev)
			return platform_get_drvdata(pdev);
	}

	pr_err("%s: fail to get exynos_pd_hsi0_data\n", __func__);
	return NULL;
}

static void exynos_pd_hsi0_ldo_control(struct exynos_pd_hsi0_data *hsi0_data, bool on)
{
	int ret1, ret2, ret3;

	if (hsi0_data->vdd_low == NULL || hsi0_data->vdd_medium == NULL ||
		hsi0_data->vdd_high == NULL) {
		dev_err(hsi0_data->dev, "not defined regulators\n");
		return;
	}

	if (on) {
		ret1 = regulator_enable(hsi0_data->vdd_low);
		if (ret1) {
			dev_err(hsi0_data->dev, "failed to enable vdd_low\n");
			return;
		}

		ret1 = regulator_enable(hsi0_data->vdd_medium);
		if (ret1) {
			dev_err(hsi0_data->dev, "failed to enable vdd_medium\n");
			regulator_disable(hsi0_data->vdd_low);
			return;
		}

		ret1 = regulator_enable(hsi0_data->vdd_high);
		if (ret1) {
			dev_err(hsi0_data->dev, "failed to enable vdd_high\n");
			regulator_disable(hsi0_data->vdd_low);
			regulator_disable(hsi0_data->vdd_medium);
			return;
		}
	} else {
		ret1 = regulator_disable(hsi0_data->vdd_low);
		ret2 = regulator_disable(hsi0_data->vdd_medium);
		ret3 = regulator_disable(hsi0_data->vdd_high);
		if (ret1 || ret2 || ret3)
			dev_err(hsi0_data->dev, "failed to disable LDOs: %d %d %d\n",
				ret1, ret2, ret3);
	}

	return;
}

#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
static void exynos_pd_hsi0_vdd_hsi_control(struct exynos_pd_hsi0_data *hsi0_data, bool on)
{
	int ret;

	if (hsi0_data->vdd_hsi == NULL) {
		dev_err(hsi0_data->dev, "not defined vdd_hsi regulator\n");
		return;
	}

	if (on) {
		ret = regulator_enable(hsi0_data->vdd_hsi);
		if (ret)
			dev_err(hsi0_data->dev, "failed to enable vdd_hsi\n");
	} else {
		ret = regulator_disable(hsi0_data->vdd_hsi);
		if (ret)
			dev_err(hsi0_data->dev, "failed to disable vdd_hsi\n");
	}

	return;
}
#endif

int exynos_pd_hsi0_ldo_manual_control(bool on)
{
	struct exynos_pd_hsi0_data *hsi0_data;

	pr_info("%s ldo = %d\n", __func__, on);

	hsi0_data = exynos_pd_hsi0_get_struct();
	if (!hsi0_data)
		return -ENODEV;

	exynos_pd_hsi0_ldo_control(hsi0_data, on);

	eusb_repeater_update_usb_state(on);

	return 0;

}
EXPORT_SYMBOL_GPL(exynos_pd_hsi0_ldo_manual_control);

#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
int exynos_pd_hsi0_vdd_hsi_manual_control(bool on)
{
	struct exynos_pd_hsi0_data *hsi0_data;

	hsi0_data = exynos_pd_hsi0_get_struct();
	if (!hsi0_data)
		return -ENODEV;

	exynos_pd_hsi0_vdd_hsi_control(hsi0_data, on);

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pd_hsi0_vdd_hsi_manual_control);
#endif

bool exynos_pd_hsi0_get_ldo_status(void)
{
	struct exynos_pd_hsi0_data *hsi0_data;

	hsi0_data = exynos_pd_hsi0_get_struct();
	if (!hsi0_data)
		return false;

	if (regulator_is_enabled(hsi0_data->vdd_low) && regulator_is_enabled(hsi0_data->vdd_medium) &&
	    regulator_is_enabled(hsi0_data->vdd_high))
		return true;

	return false;
}
EXPORT_SYMBOL_GPL(exynos_pd_hsi0_get_ldo_status);

static int exynos_pd_hsi0_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_pd_hsi0_data *hsi0_data;

	hsi0_data = devm_kzalloc(dev, sizeof(*hsi0_data), GFP_KERNEL);
	if (!hsi0_data)
		return -ENOMEM;

	platform_set_drvdata(pdev, hsi0_data);
	hsi0_data->dev = dev;

	/* vdd_hsi, vdd30, vdd18, and vdd085 are used in gs101/gs201 */
	/* vdd33, vdd12, and vdd075 are used in zuma+ */
#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
	hsi0_data->vdd_hsi = devm_regulator_get(dev, "vdd_hsi");
	if (IS_ERR(hsi0_data->vdd_hsi)) {
		dev_err(dev, "get vdd_hsi regulator failed: %ld\n", PTR_ERR(hsi0_data->vdd_hsi));
		return PTR_ERR(hsi0_data->vdd_hsi);
	}
#endif

#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
	hsi0_data->vdd_high = devm_regulator_get(dev, "vdd30");
#else
	hsi0_data->vdd_high = devm_regulator_get(dev, "vdd33");
#endif
	if (IS_ERR(hsi0_data->vdd_high)) {
		dev_err(dev, "get vdd_high regulator failed: %ld\n", PTR_ERR(hsi0_data->vdd_high));
		return PTR_ERR(hsi0_data->vdd_high);
	}

#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
	hsi0_data->vdd_medium = devm_regulator_get(dev, "vdd18");
#else
	hsi0_data->vdd_medium = devm_regulator_get(dev, "vdd12");
#endif
	if (IS_ERR(hsi0_data->vdd_medium)) {
		dev_err(dev, "get vdd_medium regulator failed: %ld\n", PTR_ERR(hsi0_data->vdd_medium));
		return PTR_ERR(hsi0_data->vdd_medium);
	}

#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
	hsi0_data->vdd_low = devm_regulator_get(dev, "vdd085");
#else
	hsi0_data->vdd_low = devm_regulator_get(dev, "vdd075");
#endif
	if (IS_ERR(hsi0_data->vdd_low)) {
		dev_err(dev, "get vdd_low regulator failed: %ld\n", PTR_ERR(hsi0_data->vdd_low));
		return PTR_ERR(hsi0_data->vdd_low);
	}

	/* vote on due to the regulators already turned on */
#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
	exynos_pd_hsi0_vdd_hsi_manual_control(true);
#endif
	exynos_pd_hsi0_ldo_manual_control(true);

	pm_runtime_enable(dev);
	return 0;
}

static int exynos_pd_hsi0_remove(struct platform_device *pdev)
{
	struct exynos_pd_hsi0_data *hsi0_data = platform_get_drvdata(pdev);

	kfree(hsi0_data);

	return 0;
}

static const struct of_device_id hsi0_of_match[] = {
	{ .compatible = "exynos-pd-hsi0", },
	{},
};
MODULE_DEVICE_TABLE(of, hsi0_of_match);

static struct platform_driver exynos_pd_hsi0 = {
	.probe = exynos_pd_hsi0_probe,
	.remove = exynos_pd_hsi0_remove,
	.driver = {
		.name = "exynos_pd_hsi0",
		.of_match_table = hsi0_of_match,
		.owner = THIS_MODULE,
	},
};

static int exynos_pd_hsi0_init(void)
{
	return platform_driver_register(&exynos_pd_hsi0);
}
fs_initcall(exynos_pd_hsi0_init);

static void exynos_pd_hsi0_exit(void)
{
	platform_driver_unregister(&exynos_pd_hsi0);
}
module_exit(exynos_pd_hsi0_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("control regulators for exynos pd_hsi0 domain");
