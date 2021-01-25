// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos PM domain support for PMUCAL 3.0 interface.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Implementation of Exynos specific power domain control which is used in
 * conjunction with runtime-pm.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <soc/google/exynos-pd.h>
#include <soc/google/bts.h>
#include <soc/google/cal-if.h>
#include <linux/module.h>
#include <soc/google/exynos-cpupm.h>

struct exynos_pm_domain *exynos_pd_lookup_name(const char *domain_name)
{
	struct exynos_pm_domain *exypd = NULL;
	struct device_node *np;

	if (IS_ERR_OR_NULL(domain_name))
		return NULL;

	for_each_compatible_node(np, NULL, "samsung,exynos-pd") {
		struct platform_device *pdev;
		struct exynos_pm_domain *pd;

		if (of_device_is_available(np)) {
			pdev = of_find_device_by_node(np);
			if (!pdev)
				continue;
			pd = platform_get_drvdata(pdev);
			if (!strcmp(pd->name, domain_name)) {
				exypd = pd;
				break;
			}
		}
	}
	return exypd;
}
EXPORT_SYMBOL(exynos_pd_lookup_name);

int exynos_pd_status(struct exynos_pm_domain *pd)
{
	int status;

	if (unlikely(!pd))
		return -EINVAL;

	mutex_lock(&pd->access_lock);
	status = cal_pd_status(pd->cal_pdid);
	mutex_unlock(&pd->access_lock);

	return status;
}
EXPORT_SYMBOL(exynos_pd_status);

int exynos_pd_get_pd_stat(struct exynos_pm_domain *pd, struct exynos_pd_stat *s)
{
	if (unlikely(!pd || !s))
		return -EINVAL;

	mutex_lock(&pd->access_lock);

	s->on_count = pd->pd_stat.on_count;
	s->last_on_time = pd->pd_stat.last_on_time;
	s->last_off_time = pd->pd_stat.last_off_time;

	/* If current state is on then include time elapsed since last on */
	if (cal_pd_status(pd->cal_pdid) > 0) {
		s->total_on_time =
			ktime_add(pd->pd_stat.total_on_time,
				  ktime_sub(ktime_get_boottime(),
					    pd->pd_stat.last_on_time));
	} else {
		s->total_on_time = pd->pd_stat.total_on_time;
	}

	mutex_unlock(&pd->access_lock);

	return 0;
}
EXPORT_SYMBOL(exynos_pd_get_pd_stat);

/* Power domain on sequence.
 * on_pre, on_post functions are registered as notification handler at CAL code.
 */
static void exynos_pd_power_on_pre(struct exynos_pm_domain *pd)
{
	if (!pd->skip_idle_ip)
		exynos_update_ip_idle_status(pd->idle_ip_index, 0);

	if (pd->devfreq_index >= 0)
		exynos_bts_scitoken_setting(true);

	if (pd->cal_pdid == HSI0_CAL_PDID)
		exynos_usbdrd_ldo_manual_control(1);
}

static void exynos_pd_power_off_post(struct exynos_pm_domain *pd)
{
	if (!pd->skip_idle_ip)
		exynos_update_ip_idle_status(pd->idle_ip_index, 1);

	if (pd->devfreq_index >= 0)
		exynos_bts_scitoken_setting(false);

	if (pd->cal_pdid == HSI0_CAL_PDID)
		exynos_usbdrd_ldo_manual_control(0);
}

/**
 * returns 0 if the domain was already ON, 1 if was successfully turned ON or waiting for sync
 * and negative in case of error
 */
static int __exynos_pd_power_on(struct exynos_pm_domain *pd)
{
	int ret = 0;

	pr_debug("pd_power_on:(%s)+\n", pd->name);

	if (atomic_read(&pd->need_sync) > 0) {
		pd->turn_off_on_sync = false;
		ret = 1;
		goto acc_unlock;
	}

	if (unlikely(!pd->pd_control)) {
		pr_debug("%s is logical sub power domain, does not have power on control\n",
			 pd->name);
		goto acc_unlock;
	}

	if (pd->power_down_skipped) {
		pr_info("%s power-on is skipped.\n", pd->name);
		goto acc_unlock;
	}

	if (cal_pd_status(pd->cal_pdid)) {
		pr_debug("pd_power_on:(%s) is already ON\n", pd->name);
		goto acc_unlock;
	}

	exynos_pd_power_on_pre(pd);

	ret = pd->pd_control(pd->cal_pdid, 1);
	if (ret) {
		pr_err("%s cannot be powered on\n", pd->name);
		exynos_pd_power_off_post(pd);
		ret = -EAGAIN;
		goto acc_unlock;
	}

	pd->pd_stat.on_count++;
	pd->pd_stat.last_on_time = ktime_get_boottime();

	ret = 1;

acc_unlock:
	pr_debug("pd_power_on:(%s)-, ret = %d\n", pd->name, ret);

	return ret;
}

int exynos_pd_power_on(struct exynos_pm_domain *pd)
{
	int ret;

	mutex_lock(&pd->access_lock);
	ret = __exynos_pd_power_on(pd);
	mutex_unlock(&pd->access_lock);

	return ret;
}
EXPORT_SYMBOL(exynos_pd_power_on);

static int genpd_power_on(struct generic_pm_domain *genpd)
{
	struct exynos_pm_domain *pd =
	    container_of(genpd, struct exynos_pm_domain, genpd);
	int ret = exynos_pd_power_on(pd);

	return ret < 0 ? ret : 0;
}

/**
 * returns 0 if the domain was already OFF, 1 if was successfully turned OFF or waiting for sync
 * and negative in case of error
 */
static int __exynos_pd_power_off(struct exynos_pm_domain *pd)
{
	int ret = 0;
	ktime_t now;

	pr_debug("pd_power_off:(%s)+\n", pd->name);

	/* Until all consumers have probed, delay actual turn OFF of PD until
	 * sync, tell GENPD that PD was turned OFF successfully to get correct
	 * accounting, GENPD will request it to be back ON if needed.
	 */
	if (atomic_read(&pd->need_sync) > 0) {
		pd->turn_off_on_sync = true;
		ret = 1;
		goto acc_unlock;
	}

	if (unlikely(!pd->pd_control)) {
		pr_debug("%s is logical sub power domain, does not have power off control\n",
			 pd->name);
		goto acc_unlock;
	}

	if (pd->power_down_ok && !pd->power_down_ok()) {
		pr_info("%s power-off is skipped.\n", pd->name);
		pd->power_down_skipped = true;
		goto acc_unlock;
	}

	if (!cal_pd_status(pd->cal_pdid)) {
		pr_debug("pd_power_off:(%s) is already OFF\n", pd->name);
		goto acc_unlock;
	}

	ret = pd->pd_control(pd->cal_pdid, 0);

	exynos_pd_power_off_post(pd);
	pd->power_down_skipped = false;

	now = ktime_get_boottime();
	pd->pd_stat.total_on_time =
		ktime_add(pd->pd_stat.total_on_time,
			  ktime_sub(now, pd->pd_stat.last_on_time));
	pd->pd_stat.last_off_time = now;

	ret = ret < 0 ? ret : 1;

acc_unlock:
	pr_debug("pd_power_off:(%s)-, ret = %d\n", pd->name, ret);

	return ret;
}

int exynos_pd_power_off(struct exynos_pm_domain *pd)
{
	int ret;

	mutex_lock(&pd->access_lock);
	ret = __exynos_pd_power_off(pd);
	mutex_unlock(&pd->access_lock);

	return ret;
}
EXPORT_SYMBOL(exynos_pd_power_off);

static int genpd_power_off(struct generic_pm_domain *genpd)
{
	struct exynos_pm_domain *pd =
		container_of(genpd, struct exynos_pm_domain, genpd);
	int ret = exynos_pd_power_off(pd);

	return ret < 0 ? ret : 0;
}

/**
 *  of_get_devfreq_sync_volt_idx - check devfreq sync voltage idx
 *
 *  Returns the index if the "devfreq-sync-voltage" is described in DT pd node,
 *  -ENOENT otherwise.
 */
static int of_get_devfreq_sync_volt_idx(const struct device_node *device)
{
	int ret;
	u32 val;

	ret = of_property_read_u32(device, "devfreq-sync-voltage", &val);
	if (ret)
		return -ENOENT;

	return val;
}

static bool exynos_pd_power_down_ok_usb(void)
{
#if IS_ENABLED(CONFIG_USB_DWC3_EXYNOS)
	return !otg_is_connect();
#else
	return true;
#endif
}

static void of_get_power_down_ok(struct exynos_pm_domain *pd)
{
	int ret;
	u32 val;
	struct device_node *device = pd->of_node;

	ret = of_property_read_u32(device, "power-down-ok", &val);
	if (ret)
		return;

	switch (val) {
	case PD_OK_USB:
		pd->power_down_ok = exynos_pd_power_down_ok_usb;
		break;
	default:
		break;
	}
}

static int exynos_pd_genpd_init(struct exynos_pm_domain *pd, int state)
{
	pd->genpd.name = pd->name;
	pd->genpd.power_off = genpd_power_off;
	pd->genpd.power_on = genpd_power_on;

	/* pd power on/off latency is less than 1ms */
	pm_genpd_init(&pd->genpd, NULL, state ? false : true);

	pd->genpd.states = kzalloc(sizeof(*pd->genpd.states), GFP_KERNEL);

	if (!pd->genpd.states)
		return -ENOMEM;

	pd->genpd.states[0].power_on_latency_ns = 1000000;
	pd->genpd.states[0].power_off_latency_ns = 1000000;

	return 0;
}

static int exynos_pd_suspend_late(struct device *dev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct exynos_pm_domain *data = platform_get_drvdata(pdev);

	if (IS_ERR(&data->genpd))
		return 0;

	/* Suspend callback function might be registered if necessary */
	if (of_property_read_bool(dev->of_node, "pd-always-on")) {
		data->genpd.flags = 0;
		dev_info(dev, "    %-9s flag set to 0\n", data->genpd.name);
	}

	return 0;
}

static int exynos_pd_resume_early(struct device *dev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct exynos_pm_domain *data = (struct exynos_pm_domain *)platform_get_drvdata(pdev);

	if (IS_ERR(&data->genpd))
		return 0;

	/* Resume callback function might be registered if necessary */
	if (of_property_read_bool(dev->of_node, "pd-always-on")) {
		data->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
		dev_info(dev, "    %-9s - %s\n", data->genpd.name, "on,  always");
	}

	return 0;
}

static int exynos_pd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct exynos_pm_domain *pd;
	struct exynos_pm_domain *parent_pd;
	struct device_node *parent;
	struct platform_device *parent_pd_pdev;

	int initial_state;
	int ret;
	ktime_t now;

	if (!of_have_populated_dt()) {
		dev_err(dev, "Could not find required device tree entries\n");
		return -EPERM;
	}

	/* skip unmanaged power domain */
	if (!of_device_is_available(np))
		return -EINVAL;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	/* init exynos_pm_domain's members  */
	pd->name = kstrdup(np->name, GFP_KERNEL);
	ret = of_property_read_u32(np, "cal_id", (u32 *)&pd->cal_pdid);
	if (ret) {
		dev_err(dev, "failed to get cal_pdid from of %s\n", pd->name);
		return -ENODEV;
	}

	pd->of_node = np;
	pd->pd_control = cal_pd_control;
	pd->check_status = exynos_pd_status;
	pd->devfreq_index = of_get_devfreq_sync_volt_idx(pd->of_node);
	of_get_power_down_ok(pd);
	pd->power_down_skipped = false;

	ret = of_property_read_u32(np, "need_smc", (u32 *)&pd->need_smc);
	if (ret) {
		pd->need_smc = 0x0;
	} else {
		cal_pd_set_smc_id(pd->cal_pdid, pd->need_smc);
		dev_dbg(dev, "read need_smc 0x%x successfully.!\n",
			pd->need_smc);
	}
	initial_state = cal_pd_status(pd->cal_pdid);
	if (initial_state == -1) {
		dev_err(dev, "%s is in unknown state\n", pd->name);
		kfree(pd->name);
		return -EINVAL;
	}

	now = ktime_get_boottime();
	if (initial_state > 0) {
		pd->pd_stat.last_on_time = now;
		pd->pd_stat.on_count = 1;
		atomic_set(&pd->need_sync, 1);
	} else {
		pd->pd_stat.last_off_time = now;
		atomic_set(&pd->need_sync, 0);
	}

	if (of_property_read_bool(np, "skip-idle-ip"))
		pd->skip_idle_ip = true;
	else
		pd->idle_ip_index = exynos_get_idle_ip_index(pd->name);

	mutex_init(&pd->access_lock);
	platform_set_drvdata(pdev, pd);

	ret = exynos_pd_genpd_init(pd, initial_state);
	if (ret) {
		dev_err(dev, "exynos_pd_genpd_init fail: ret:%d\n", ret);
		kfree(pd->name);
		return ret;
	}

	of_genpd_add_provider_simple(np, &pd->genpd);

	if (of_property_read_bool(np, "pd-always-on")) {
		pd->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
		dev_info(dev, " - %s\n", "on,  always");
	} else {
		dev_info(dev, " - %-3s\n", pd->genpd.name,
			 cal_pd_status(pd->cal_pdid) ? "on" : "off");
	}

	parent = of_parse_phandle(np, "power-domains", 0);
	if (parent) {
		parent_pd_pdev = of_find_device_by_node(parent);
		if (parent_pd_pdev) {
			parent_pd = platform_get_drvdata(parent_pd_pdev);
			if (parent_pd) {
				if (pm_genpd_add_subdomain(&parent_pd->genpd,
							   &pd->genpd)) {
					dev_err(&parent_pd_pdev->dev, "cannot add subdomain %s\n",
						pd->name);
				} else {
					pd->parent = parent_pd;
					atomic_add(atomic_read(&pd->need_sync),
						   &parent_pd->need_sync);
					dev_info(&parent_pd_pdev->dev, "has new subdomain %s\n",
						 pd->name);
				}
			}
		}
	}

	pm_runtime_enable(&pdev->dev);

	dev_info(dev, "PM Domain Initialized\n");
	return ret;
}

static void exynos_pd_sync_state(struct exynos_pm_domain *pd)
{
	int need_sync;

	mutex_lock(&pd->access_lock);
	need_sync = atomic_dec_return(&pd->need_sync);

	/* PD never needed sync or other child dec first */
	if (need_sync < 0)
		goto sync_unlock;

	if (need_sync == 0) {
		pr_info("%s sync_state: turn_off = %d\n", pd->name, pd->turn_off_on_sync);
		if (pd->turn_off_on_sync)
			__exynos_pd_power_off(pd);
		if (pd->parent)
			exynos_pd_sync_state(pd->parent);
	} else {
		pr_info("%s sync_state: children need sync\n", pd->name);
	}

sync_unlock:
	mutex_unlock(&pd->access_lock);
}

static void dev_sync_state(struct device *dev)
{
	struct exynos_pm_domain *pd;

	pd = platform_get_drvdata(to_platform_device(dev));
	exynos_pd_sync_state(pd);
}

static const struct of_device_id of_exynos_pd_match[] = {
	{ .compatible = "samsung,exynos-pd", },
	{ },
};
MODULE_DEVICE_TABLE(of, of_exynos_pd_match);

static const struct dev_pm_ops exynos_pd_pm_ops = {
	.suspend_late	= exynos_pd_suspend_late,
	.resume_early	= exynos_pd_resume_early,
};

static const struct platform_device_id exynos_pd_ids[] = {
	{ "exynos-pd", },
	{ }
};

static struct platform_driver exynos_pd_driver = {
	.driver = {
		.name = "exynos-pd",
		.of_match_table = of_exynos_pd_match,
		.sync_state = dev_sync_state,
		.pm = &exynos_pd_pm_ops
	},
	.probe		= exynos_pd_probe,
	.id_table	= exynos_pd_ids,
};

static int exynos_pd_init(void)
{
	return platform_driver_register(&exynos_pd_driver);
}
subsys_initcall(exynos_pd_init);

static void exynos_pd_exit(void)
{
	return platform_driver_unregister(&exynos_pd_driver);
}
module_exit(exynos_pd_exit);

MODULE_SOFTDEP("pre: clk_exynos");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exynos PM domain Support");
