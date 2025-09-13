// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/wakeup_reason.h>
#include <linux/syscore_ops.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/debugfs.h>
#include <linux/smp.h>
#include <linux/rtc.h>

#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#include <soc/google/cal-if.h>

#define EXYNOS_EINT_PEND(b, x)      ((b) + 0xA00 + (((x) >> 3) * 4))
#define SHARED_SR0 0x80
#define WS_BIT_MAILBOX_AOC2AP		(7)
#define WS2_BIT_MAILBOX_AOCA322AP	(5)
#define WS2_BIT_MAILBOX_AOCF12AP	(6)
#define WS2_BIT_VGPIO2PMU_EINT		(18)

static struct exynos_pm_info *pm_info;
static struct exynos_pm_dbg *pm_dbg;

static void inline exynos_update_eint_wakeup_mask(void)
{
	exynos_pmu_read(pm_info->eint_wakeup_mask_offset[0], &exynos_eint_wake_mask_array[0]);
	exynos_pmu_read(pm_info->eint_wakeup_mask_offset[1], &exynos_eint_wake_mask_array[1]);
	exynos_pmu_read(pm_info->eint_wakeup_mask_offset[2], &exynos_eint_wake_mask_array[2]);
}

static void exynos_show_wakeup_reason_sysint(unsigned int stat,
					     struct wakeup_stat_name *ws_names,
					     int wakeup_stat_id)
{
	int bit;
	unsigned long lstat = stat;
	char wake_reason[MAX_SUSPEND_ABORT_LEN];
	int str_idx = 0;
	u32 aoc_id;

	for_each_set_bit(bit, &lstat, 32) {
		if (!ws_names->name[bit])
			continue;

		/*
		 * Ignore logging VGPIO2PMU_EINT wakeup reasons because they
		 * will get logged in the function s2mpg14_irq_thread().
		 */
		if (wakeup_stat_id == 1 && bit == WS2_BIT_VGPIO2PMU_EINT)
			continue;

		if (str_idx) {
			str_idx += strscpy(wake_reason + str_idx, ",",
					   MAX_SUSPEND_ABORT_LEN - str_idx);
		}
		str_idx += strscpy(wake_reason + str_idx, ws_names->name[bit],
				   MAX_SUSPEND_ABORT_LEN - str_idx);
		if ((wakeup_stat_id == 0 && bit == WS_BIT_MAILBOX_AOC2AP) ||
		    (wakeup_stat_id == 1 && bit == WS2_BIT_MAILBOX_AOCA322AP)) {
			aoc_id = __raw_readl(pm_info->mbox_aoc + SHARED_SR0);
			str_idx += scnprintf(wake_reason + str_idx,
					     MAX_SUSPEND_ABORT_LEN - str_idx,
					     "x%X", aoc_id);
		} else if (wakeup_stat_id == 1 && bit == WS2_BIT_MAILBOX_AOCF12AP &&
				!IS_ERR(pm_info->mbox_aocf1)) {
			aoc_id = __raw_readl(pm_info->mbox_aocf1 + SHARED_SR0);
			str_idx += scnprintf(wake_reason + str_idx,
					     MAX_SUSPEND_ABORT_LEN - str_idx,
					     "x%X", aoc_id);
		}
	}
#ifdef CONFIG_SUSPEND
	if (str_idx)
		log_abnormal_wakeup_reason(wake_reason);
#endif
}

static void exynos_show_wakeup_reason_detail(unsigned int wakeup_stat)
{
	int i;
	unsigned int wss;

	if ((wakeup_stat & pm_info->wakeup_stat_eint))
		exynos_update_eint_wakeup_mask();

	if (unlikely(!pm_info->ws_names))
		return;

	for (i = 0; i < pm_info->num_wakeup_stat; i++) {
		if (i == 0)
			wss = wakeup_stat & ~(pm_info->wakeup_stat_eint);
		else
			exynos_pmu_read(pm_info->wakeup_stat_offset[i], &wss);
		if (!wss)
			continue;

		exynos_show_wakeup_reason_sysint(wss, &pm_info->ws_names[i], i);
	}
}

static void exynos_show_wakeup_reason(bool sleep_abort)
{
	unsigned int wakeup_stat;
	int i;

	if (sleep_abort) {
		pr_info("%s early wakeup! Dumping pending registers...\n", EXYNOS_PM_PREFIX);

		if (pm_info->gic_base) {
			pr_info("GIC_PEND:\n");
			for (i = 0; i < pm_info->num_gic; i++)
				pr_info("GICD_ISPENDR[%d] = 0x%x\n", i,
					__raw_readl(pm_info->gic_base + i * 4));
		} else {
#ifdef CONFIG_SUSPEND
			// If there is no pending wakeup, it should be aborted
			// by TF-A or ACPM.
			log_abnormal_wakeup_reason("sleep abort (by tf-a or acpm)");
#endif
                }

		pr_info("%s done.\n", EXYNOS_PM_PREFIX);
		return;
	}

	if (!pm_info->num_wakeup_stat)
		return;

	exynos_pmu_read(pm_info->wakeup_stat_offset[0], &wakeup_stat);

	exynos_show_wakeup_reason_detail(wakeup_stat);

	if (wakeup_stat & (1 << pm_info->wakeup_stat_rtc)) {
		pr_info("%s Resume caused by RTC alarm\n", EXYNOS_PM_PREFIX);
	} else {
		for (i = 0; i < pm_info->num_wakeup_stat; i++) {
			exynos_pmu_read(pm_info->wakeup_stat_offset[i], &wakeup_stat);
			pr_info("%s Resume caused by wakeup%d_stat 0x%08x\n",
				EXYNOS_PM_PREFIX, i + 1, wakeup_stat);
		}
	}
}

static void exynos_set_wakeupmask(enum sys_powerdown mode)
{
	int i;
	u32 wakeup_int_en = 0;

	/* Set external interrupt mask */
	for (i = 0; i < pm_info->num_eint_wakeup_mask; i++) {
		exynos_pmu_write(pm_info->eint_wakeup_mask_offset[i],
				 exynos_eint_wake_mask_array[i]);
	}

	for (i = 0; i < pm_info->num_wakeup_int_en; i++) {
		exynos_pmu_write(pm_info->wakeup_stat_offset[i], 0);
		wakeup_int_en = pm_info->wakeup_int_en[i];

		exynos_pmu_write(pm_info->wakeup_int_en_offset[i], wakeup_int_en);
	}
}

static int exynos_prepare_sys_powerdown(enum sys_powerdown mode)
{
	int ret;

	exynos_set_wakeupmask(mode);

	ret = cal_pm_enter(mode);
	if (ret)
		pr_err("CAL Fail to set powermode\n");

	return ret;
}

static void exynos_wakeup_sys_powerdown(enum sys_powerdown mode, bool early_wakeup)
{
	if (early_wakeup)
		cal_pm_earlywakeup(mode);
	else
		cal_pm_exit(mode);
}

u64 get_frc_time(void);
static int exynos_pm_syscore_suspend(void)
{
#ifdef CONFIG_CP_PMUCAL
	if (!exynos_check_cp_status()) {
		pr_info("%s syscore_suspend: sleep canceled by CP reset\n",
			EXYNOS_PM_PREFIX);
		return -EINVAL;
	}
#endif

	pm_info->is_pcieon_suspend = false;
	if (pm_info->pcieon_suspend_available) {
		if (!IS_ERR_OR_NULL(pm_info->pcie_is_connect))
			pm_info->is_pcieon_suspend = !!pm_info->pcie_is_connect();
	}

	if (pm_info->is_pcieon_suspend || pm_dbg->test_pcieon_suspend) {
		exynos_prepare_sys_powerdown(pm_info->pcieon_suspend_mode_idx);
		pr_debug("%s syscore_suspend: Enter Suspend scenario. pcieon_mode_idx = %d)\n",
			EXYNOS_PM_PREFIX, pm_info->pcieon_suspend_mode_idx);
	} else {
		exynos_prepare_sys_powerdown(pm_info->suspend_mode_idx);
		pr_debug("%s syscore_suspend: Enter Suspend scenario. suspend_mode_idx = %d)\n",
			EXYNOS_PM_PREFIX, pm_info->suspend_mode_idx);
	}

	/* Send an IPI if test_early_wakeup flag is set */
//	if (pm_dbg->test_early_wakeup)
//		arch_send_call_function_single_ipi(0);

	pm_dbg->mifdn_cnt_prev = acpm_get_mifdn_count();
	pm_info->apdn_cnt_prev = acpm_get_apsocdn_count();
	pm_dbg->mif_req = acpm_get_mif_request();

	pm_dbg->mifdn_early_wakeup_prev = acpm_get_early_wakeup_count();

	pr_info("%s frc:%llu prev mif_count:%d, apsoc_count:%d, seq_early_wakeup_count:%d\n",
		EXYNOS_PM_PREFIX, get_frc_time(), pm_dbg->mifdn_cnt_prev,
		pm_info->apdn_cnt_prev, pm_dbg->mifdn_early_wakeup_prev);

	return 0;
}

static void exynos_pm_syscore_resume(void)
{
	pm_dbg->mifdn_cnt = acpm_get_mifdn_count();
	pm_info->apdn_cnt = acpm_get_apsocdn_count();
	pm_dbg->mifdn_early_wakeup_cnt = acpm_get_early_wakeup_count();

	pr_info("%s frc:%llu post mif_count:%d, apsoc_count:%d, seq_early_wakeup_count:%d\n",
		EXYNOS_PM_PREFIX, get_frc_time(), pm_dbg->mifdn_cnt,
		pm_info->apdn_cnt, pm_dbg->mifdn_early_wakeup_cnt);

	if (pm_info->apdn_cnt == pm_info->apdn_cnt_prev) {
		pm_info->is_early_wakeup = true;
		pr_info("%s syscore_resume: return to originator\n",
			EXYNOS_PM_PREFIX);
	} else {
		pm_info->is_early_wakeup = false;
	}

	if (pm_dbg->mifdn_early_wakeup_cnt != pm_dbg->mifdn_early_wakeup_prev)
		pr_debug("%s Sequence early wakeup\n", EXYNOS_PM_PREFIX);

	if (pm_dbg->mifdn_cnt == pm_dbg->mifdn_cnt_prev)
		pr_info("%s MIF blocked. MIF request Mster was  0x%x\n",
			EXYNOS_PM_PREFIX, pm_dbg->mif_req);
	else
		pr_info("%s MIF down. cur_count: %d, acc_count: %d\n",
			EXYNOS_PM_PREFIX,
			pm_dbg->mifdn_cnt - pm_dbg->mifdn_cnt_prev,
			pm_dbg->mifdn_cnt);

	if (pm_info->is_pcieon_suspend || pm_dbg->test_pcieon_suspend)
		exynos_wakeup_sys_powerdown(pm_info->pcieon_suspend_mode_idx,
					    pm_info->is_early_wakeup);
	else
		exynos_wakeup_sys_powerdown(pm_info->suspend_mode_idx,
					    pm_info->is_early_wakeup);

	exynos_show_wakeup_reason(pm_info->is_early_wakeup);

	if (!pm_info->is_early_wakeup)
		pr_debug("%s syscore_resume: post sleep, preparing to return\n",
			 EXYNOS_PM_PREFIX);
}

int register_pcie_is_connect(u32 (*func)(void))
{
	if (func) {
		pm_info->pcie_is_connect = func;
		pr_info("Registered pcie_is_connect func\n");
		return 0;
	}

	pr_err("%s: function pointer is NULL\n", __func__);
	return -ENXIO;
}
EXPORT_SYMBOL_GPL(register_pcie_is_connect);

bool is_test_pcieon_suspend_set(void)
{
	if (!pm_dbg)
		return false;
	return pm_dbg->test_pcieon_suspend;
}
EXPORT_SYMBOL_GPL(is_test_pcieon_suspend_set);

static struct syscore_ops exynos_pm_syscore_ops = {
	.suspend	= exynos_pm_syscore_suspend,
	.resume		= exynos_pm_syscore_resume,
};

#ifdef CONFIG_DEBUG_FS
static int wake_lock_get(void *data, unsigned long long *val)
{
	*val = (unsigned long long)pm_info->is_stay_awake;
	return 0;
}

static int wake_lock_set(void *data, unsigned long long val)
{
	bool before = pm_info->is_stay_awake;

	pm_info->is_stay_awake = (bool)val;

	if (before != pm_info->is_stay_awake) {
		if (pm_info->is_stay_awake)
			__pm_stay_awake(pm_info->ws);
		else
			__pm_relax(pm_info->ws);
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(wake_lock_fops, wake_lock_get, wake_lock_set, "%llu\n");

static void exynos_pm_debugfs_init(void)
{
	struct dentry *root;

	root = debugfs_create_dir("exynos-pm", NULL);
	if (!root) {
		pr_err("%s debugfs_init: could't create debugfs dir\n",
		       EXYNOS_PM_PREFIX);
		return;
	}

	debugfs_create_u32("test_pcieon_suspend", 0644, root,
			   &pm_dbg->test_pcieon_suspend);

	debugfs_create_u32("test_early_wakeup", 0644, root, &pm_dbg->test_early_wakeup);

	if (pm_info->ws)
		debugfs_create_file("wake_lock", 0644, root, NULL, &wake_lock_fops);
}
#endif

static void exynos_pm_suspend_marker(char *annotation)
{
	struct timespec64 ts;
	struct rtc_time tm;

	ktime_get_real_ts64(&ts);
	rtc_time64_to_tm(ts.tv_sec, &tm);
	pr_info("PM: suspend %s %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		annotation, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
}

static int exynos_pm_notification_handler(struct notifier_block *notifier,
		unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		exynos_pm_suspend_marker("entry");
		break;
	case PM_POST_SUSPEND:
		exynos_pm_suspend_marker("exit");
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block exynos_pm_notifier_block = {
	.notifier_call = exynos_pm_notification_handler,
};

static int parse_dt_wakeup_stat_names(struct device *dev, struct device_node *np)
{
	struct device_node *root, *child;
	int ret;
	int size, n, idx = 0;

	root = of_find_node_by_name(np, "wakeup_stats");
	n = of_get_child_count(root);

	if (pm_info->num_wakeup_stat != n || !n) {
		dev_err(dev, "failed to get wakeup_stats(%d)\n", n);
		return -EINVAL;
	}

	pm_info->ws_names = devm_kcalloc(dev, n, sizeof(*pm_info->ws_names), GFP_KERNEL);
	if (!pm_info->ws_names)
		return -ENOMEM;

	for_each_child_of_node(root, child) {
		size = of_property_count_strings(child, "ws-name");
		if (size < 0 || size > 32) {
			dev_err(dev, "failed to get wakeup_stats name cnt(%d)\n", size);
			return -EINVAL;
		}

		ret = of_property_read_string_array(child, "ws-name",
						    pm_info->ws_names[idx].name, size);
		if (ret < 0) {
			dev_err(dev, "failed to read wakeup_stats name(%d)\n", ret);
			return ret;
		}

		idx++;
	}

	return 0;
}

static int exynos_pm_drvinit(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	unsigned int wake_lock = 0;
	int ret;

	pm_info = devm_kzalloc(dev, sizeof(*pm_info), GFP_KERNEL);
	if (IS_ERR(pm_info))
		return PTR_ERR(pm_info);

	pm_dbg = devm_kzalloc(dev, sizeof(*pm_dbg), GFP_KERNEL);
	if (IS_ERR(pm_dbg))
		return PTR_ERR(pm_dbg);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pm_info->eint_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pm_info->eint_base))
		return PTR_ERR(pm_info->eint_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	pm_info->eint_far_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pm_info->eint_far_base))
		return PTR_ERR(pm_info->eint_far_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	pm_info->gic_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pm_info->gic_base)) {
		// GIC now claims its MMIO. So for kernels 5.19+, we can't map this
		// overlapping memory region. Refer to b/237296335 for more details.
		dev_warn(dev, "drvinit: unable to map gic dist registers, ret=%ld\n",
			 PTR_ERR(pm_info->gic_base));
		pm_info->gic_base = 0;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	pm_info->mbox_aoc = devm_ioremap_resource(dev, res);
	if (IS_ERR(pm_info->mbox_aoc))
		return PTR_ERR(pm_info->mbox_aoc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 4);
	pm_info->mbox_aocf1 = devm_ioremap_resource(dev, res);
	if (IS_ERR(pm_info->mbox_aocf1))
		dev_warn(dev, "unable to get the mapped address of mbox_aocf1\n");

	ret = of_property_read_u32(np, "num-gic", &pm_info->num_gic);
	if (ret)
		dev_warn(dev, "unable to get the number of gic from DT\n");

	ret = of_property_read_u32(np, "wakeup-stat-eint", &pm_info->wakeup_stat_eint);
	if (ret)
		dev_warn(dev, "unable to get the eint bit of WAKEUP_STAT from DT\n");

	ret = of_property_read_u32(np, "wakeup-stat-rtc", &pm_info->wakeup_stat_rtc);
	if (ret)
		dev_warn(dev, "unable to get the rtc bit of WAKEUP_STAT from DT\n");

	ret = of_property_read_u32(np, "suspend_mode_idx", &pm_info->suspend_mode_idx);
	if (ret)
		dev_warn(dev, "unable to get suspend_mode_idx from DT\n");

	ret = of_property_count_u32_elems(np, "wakeup_stat_offset");
	if (ret > 0) {
		pm_info->num_wakeup_stat = ret;
		pm_info->wakeup_stat_offset = devm_kcalloc(dev, ret, sizeof(unsigned int),
							   GFP_KERNEL);
		of_property_read_u32_array(np, "wakeup_stat_offset",
					   pm_info->wakeup_stat_offset, ret);
	} else
		dev_warn(dev, "unable to get wakeup_stat value from DT\n");

	ret = of_property_count_u32_elems(np, "wakeup_int_en_offset");
	if (ret > 0) {
		pm_info->num_wakeup_int_en = ret;
		pm_info->wakeup_int_en_offset = devm_kcalloc(dev, ret, sizeof(unsigned int),
							     GFP_KERNEL);
		of_property_read_u32_array(np, "wakeup_int_en_offset",
					   pm_info->wakeup_int_en_offset, ret);
	} else
		dev_warn(dev, "unable to get wakeup_int_en_offset value from DT\n");

	ret = of_property_count_u32_elems(np, "wakeup_int_en");
	if (ret > 0) {
		pm_info->wakeup_int_en = devm_kcalloc(dev, ret, sizeof(unsigned int), GFP_KERNEL);
		of_property_read_u32_array(np, "wakeup_int_en",
					   pm_info->wakeup_int_en, ret);
	} else
		dev_warn(dev, "unable to get wakeup_int_en value from DT\n");

	ret = of_property_count_u32_elems(np, "eint_wakeup_mask_offset");
	if (ret > 0) {
		pm_info->num_eint_wakeup_mask = ret;
		pm_info->eint_wakeup_mask_offset = devm_kcalloc(dev, ret, sizeof(unsigned int),
								GFP_KERNEL);
		of_property_read_u32_array(np, "eint_wakeup_mask_offset",
					   pm_info->eint_wakeup_mask_offset, ret);
	} else
		dev_warn(dev, "unable to get eint_wakeup_mask_offset from DT\n");

	ret = of_property_read_u32(np, "wake_lock", &wake_lock);
	if (!ret) {
		pm_info->ws = wakeup_source_register(NULL, "exynos-pm");
		if (!pm_info->ws)
			WARN_ON(1);

		pm_info->is_stay_awake = (bool)wake_lock;

		if (pm_info->is_stay_awake)
			__pm_stay_awake(pm_info->ws);
	} else
		dev_warn(dev, "unable to get wake_lock from DT\n");

	ret = of_property_read_u32(np, "pcieon_suspend_available",
				   &pm_info->pcieon_suspend_available);
	if (!ret) {
		ret = of_property_read_u32(np, "pcieon_suspend_mode_idx",
					   &pm_info->pcieon_suspend_mode_idx);
		if (ret)
			dev_warn(dev, "unable to get pcieon_suspemd_mode_idx from DT\n");
	} else
		dev_warn(dev, "No support for pcieon_suspend mode\n");

	ret = parse_dt_wakeup_stat_names(dev, np);
	if (ret < 0)
		return ret;

	register_syscore_ops(&exynos_pm_syscore_ops);
	register_pm_notifier(&exynos_pm_notifier_block);
#ifdef CONFIG_DEBUG_FS
	exynos_pm_debugfs_init();
#endif

	dev_dbg(dev, "initialized\n");
	return 0;
}

static const struct of_device_id of_exynos_pm_match[] = {
	{ .compatible = "samsung,exynos-pm", },
	{ },
};
MODULE_DEVICE_TABLE(of, of_exynos_pm_match);

static const struct platform_device_id exynos_pm_ids[] = {
	{ "exynos-pm", },
	{ }
};

static struct platform_driver exynos_pm_driver = {
	.driver = {
		.name = "exynos-pm",
		.of_match_table = of_exynos_pm_match,
	},
	.probe		= exynos_pm_drvinit,
	.id_table	= exynos_pm_ids,
};

static int exynos_pm_init(void)
{
	return platform_driver_register(&exynos_pm_driver);
}
arch_initcall(exynos_pm_init);

static void exynos_pm_exit(void)
{
	return platform_driver_unregister(&exynos_pm_driver);
}
module_exit(exynos_pm_exit);

MODULE_LICENSE("GPL");
