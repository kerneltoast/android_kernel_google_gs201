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

static struct exynos_pm_info *pm_info;
static struct exynos_pm_dbg *pm_dbg;

static void exynos_show_wakeup_reason_eint(void)
{
	int bit;
	int i;
	unsigned long ext_int_pend;
	u32 eint_wakeup_mask[3];
	bool found = 0;
	unsigned long gpa_mask[3] = {0, 0, 0};
	unsigned int gpa_cnt = 0, shift_cnt = 0, gpa_idx = 0;

	exynos_pmu_read(pm_info->eint_wakeup_mask_offset[0], &eint_wakeup_mask[0]);
	exynos_pmu_read(pm_info->eint_wakeup_mask_offset[1], &eint_wakeup_mask[1]);
	exynos_pmu_read(pm_info->eint_wakeup_mask_offset[2], &eint_wakeup_mask[2]);

	for (i = 0; i < pm_info->num_gpa; i++) {
		if (i * 8 < pm_info->num_eint)
			ext_int_pend = __raw_readl(EXYNOS_EINT_PEND(pm_info->eint_base, i * 8));
		else
			ext_int_pend = __raw_readl(EXYNOS_EINT_PEND(pm_info->eint_far_base,
								    i * 8 - pm_info->num_eint));

		shift_cnt = pm_info->gpa_use[i];
		ext_int_pend = ext_int_pend << (32 - shift_cnt);
		ext_int_pend = ext_int_pend >> (32 - shift_cnt);

		gpa_mask[gpa_idx] |= ext_int_pend << gpa_cnt;
		gpa_cnt += shift_cnt;

		if (gpa_cnt > 32) {
			shift_cnt = gpa_cnt - 32 + 1;
			ext_int_pend = ext_int_pend << (32 - pm_info->gpa_use[i]);
			ext_int_pend = ext_int_pend >> (32 - pm_info->gpa_use[i] + shift_cnt);
			gpa_mask[++gpa_idx] |= ext_int_pend;
			gpa_cnt = pm_info->gpa_use[i] - shift_cnt;
		}
	}

	for (i = 0; i < gpa_idx; i++) {
		for_each_set_bit(bit, &gpa_mask[i], 32) {
			u32 gpio;
			int irq;

			if (eint_wakeup_mask[i] & (1 << bit))
				continue;

			gpio = exynos_eint_to_pin_num(i * 32 + bit);
			irq = gpio_to_irq(gpio);

			pr_info("%s Resume caused by EINT num: %d\n", EXYNOS_PM_PREFIX, irq);

			found = 1;
		}
	}

	if (!found)
		pr_info("%s Resume caused by unknown EINT\n", EXYNOS_PM_PREFIX);
}

static void exynos_show_wakeup_registers(unsigned int wakeup_stat)
{
	int i, size;

	pr_info("WAKEUP_STAT:\n");
	for (i = 0; i < pm_info->num_wakeup_stat; i++) {
		exynos_pmu_read(pm_info->wakeup_stat_offset[i], &wakeup_stat);
		pr_info("0x%08x\n", wakeup_stat);
	}

	pr_info("EINT_PEND: ");
	for (i = 0, size = 8; i < pm_info->num_eint; i += size)
		pr_info("0x%02x ", __raw_readl(EXYNOS_EINT_PEND(pm_info->eint_base, i)));

	pr_info("EINT_FAR_PEND: ");
	for (i = 0, size = 8; i < pm_info->num_eint_far; i += size)
		pr_info("0x%02x ", __raw_readl(EXYNOS_EINT_PEND(pm_info->eint_far_base, i)));
}

static void exynos_show_wakeup_reason_sysint(unsigned int stat,
					     struct wakeup_stat_name *ws_names)
{
	int bit;
	unsigned long lstat = stat;
	char wake_reason[MAX_SUSPEND_ABORT_LEN];
	int str_idx = 0;
	u32 aoc_id;

	for_each_set_bit(bit, &lstat, 32) {
		if (!ws_names->name[bit])
			continue;
		if (str_idx) {
			str_idx += strscpy(wake_reason + str_idx, ",",
					   MAX_SUSPEND_ABORT_LEN - str_idx);
		}
		str_idx += strscpy(wake_reason + str_idx, ws_names->name[bit],
				   MAX_SUSPEND_ABORT_LEN - str_idx);
		if (bit == 7) {	/* MAILBOX_AOC2AP */
			aoc_id = __raw_readl(pm_info->mbox_aoc + SHARED_SR0);
			str_idx += scnprintf(wake_reason + str_idx,
					     MAX_SUSPEND_ABORT_LEN - str_idx,
					     "%d", aoc_id);
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
		exynos_show_wakeup_reason_eint();

	if (unlikely(!pm_info->ws_names))
		return;

	for (i = 0; i < pm_info->num_wakeup_stat; i++) {
		if (i == 0)
			wss = wakeup_stat & ~(pm_info->wakeup_stat_eint);
		else
			exynos_pmu_read(pm_info->wakeup_stat_offset[i], &wss);
		if (!wss)
			continue;

		exynos_show_wakeup_reason_sysint(wss, &pm_info->ws_names[i]);
	}
}

static void exynos_show_wakeup_reason(bool sleep_abort)
{
	unsigned int wakeup_stat;
	int i, size;

	if (sleep_abort) {
		pr_info("%s early wakeup! Dumping pending registers...\n", EXYNOS_PM_PREFIX);

		pr_info("EINT_PEND:\n");
		for (i = 0, size = 8; i < pm_info->num_eint; i += size)
			pr_info("0x%x\n", __raw_readl(EXYNOS_EINT_PEND(pm_info->eint_base, i)));

		pr_info("EINT_FAR_PEND:\n");
		for (i = 0, size = 8; i < pm_info->num_eint_far; i += size)
			pr_info("0x%x\n", __raw_readl(EXYNOS_EINT_PEND(pm_info->eint_far_base, i)));

		pr_info("GIC_PEND:\n");
		for (i = 0; i < pm_info->num_gic; i++)
			pr_info("GICD_ISPENDR[%d] = 0x%x\n", i,
				__raw_readl(pm_info->gic_base + i * 4));

		pr_info("%s done.\n", EXYNOS_PM_PREFIX);
		return;
	}

	if (!pm_info->num_wakeup_stat)
		return;

	exynos_pmu_read(pm_info->wakeup_stat_offset[0], &wakeup_stat);
	exynos_show_wakeup_registers(wakeup_stat);

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

	pr_info("%s: prev mif_count:%d, apsoc_count:%d, seq_early_wakeup_count:%d\n",
		EXYNOS_PM_PREFIX, pm_dbg->mifdn_cnt_prev,
		pm_info->apdn_cnt_prev, pm_dbg->mifdn_early_wakeup_prev);

	return 0;
}

static void exynos_pm_syscore_resume(void)
{
	pm_dbg->mifdn_cnt = acpm_get_mifdn_count();
	pm_info->apdn_cnt = acpm_get_apsocdn_count();
	pm_dbg->mifdn_early_wakeup_cnt = acpm_get_early_wakeup_count();

	pr_info("%s: post mif_count:%d, apsoc_count:%d, seq_early_wakeup_count:%d\n",
		EXYNOS_PM_PREFIX, pm_dbg->mifdn_cnt,
		pm_info->apdn_cnt, pm_dbg->mifdn_early_wakeup_cnt);

	if (pm_info->apdn_cnt == pm_info->apdn_cnt_prev) {
		pm_info->is_early_wakeup = true;
		pr_info("%s syscore_resume: return to originator\n",
			EXYNOS_PM_PREFIX);
	} else {
		pm_info->is_early_wakeup = false;
	}

	if (pm_dbg->mifdn_early_wakeup_cnt != pm_dbg->mifdn_early_wakeup_prev)
		pr_debug("%s: Sequence early wakeup\n", EXYNOS_PM_PREFIX);

	if (pm_dbg->mifdn_cnt == pm_dbg->mifdn_cnt_prev)
		pr_info("%s: MIF blocked. MIF request Mster was  0x%x\n",
			EXYNOS_PM_PREFIX, pm_dbg->mif_req);
	else
		pr_info("%s: MIF down. cur_count: %d, acc_count: %d\n",
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

static void parse_dt_wakeup_stat_names(struct device *dev, struct device_node *np)
{
	struct device_node *root, *child;
	int ret;
	int size, n, idx = 0;

	root = of_find_node_by_name(np, "wakeup_stats");
	n = of_get_child_count(root);

	if (pm_info->num_wakeup_stat != n || !n) {
		pr_err("drvinit: failed to get wakeup_stats(%d)\n", n);
		return;
	}

	pm_info->ws_names = devm_kcalloc(dev, n, sizeof(*pm_info->ws_names), GFP_KERNEL);
	if (!pm_info->ws_names)
		return;

	for_each_child_of_node(root, child) {
		size = of_property_count_strings(child, "ws-name");
		if (size <= 0 || size > 32) {
			pr_err("drvinit: failed to get wakeup_stat name cnt(%d)\n", size);
			return;
		}

		ret = of_property_read_string_array(child, "ws-name",
						    pm_info->ws_names[idx].name, size);
		if (ret < 0) {
			pr_err("drvinit: failed to read wakeup_stat name(%d)\n", ret);
			return;
		}

		idx++;
	}
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
	if (IS_ERR(pm_info->gic_base))
		return PTR_ERR(pm_info->gic_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	pm_info->mbox_aoc = devm_ioremap_resource(dev, res);
	if (IS_ERR(pm_info->mbox_aoc))
		return PTR_ERR(pm_info->mbox_aoc);

	ret = of_property_read_u32(np, "num-eint", &pm_info->num_eint);
	if (ret) {
		dev_err(dev, "drvinit: unabled to get the number of eint from DT\n");
		WARN_ON(1);
	}

	ret = of_property_read_u32(np, "num-eint-far", &pm_info->num_eint_far);
	if (ret) {
		dev_err(dev, "drvinit: unabled to get the number of eint-far from DT\n");
		WARN_ON(1);
	}

	ret = of_property_count_u32_elems(np, "gpa-use");
	if (!ret) {
		dev_err(dev, "drvinit: unabled to get num-gpa-use from DT\n");
	} else if (ret > 0) {
		pm_info->num_gpa = ret;
		pm_info->gpa_use = devm_kcalloc(dev, ret, sizeof(unsigned int), GFP_KERNEL);
		of_property_read_u32_array(np, "gpa-use", pm_info->gpa_use, ret);
	}

	ret = of_property_read_u32(np, "num-gic", &pm_info->num_gic);
	if (ret) {
		dev_err(dev, "drvinit: unabled to get the number of gic from DT\n");
		WARN_ON(1);
	}

	ret = of_property_read_u32(np, "wakeup-stat-eint", &pm_info->wakeup_stat_eint);
	if (ret) {
		dev_err(dev, "drvinit: unabled to get the eint bit of WAKEUP_STAT from DT\n");
		WARN_ON(1);
	}

	ret = of_property_read_u32(np, "wakeup-stat-rtc", &pm_info->wakeup_stat_rtc);
	if (ret) {
		dev_err(dev, "drvinit: unabled to get the rtc bit of WAKEUP_STAT from DT\n");
		WARN_ON(1);
	}

	ret = of_property_read_u32(np, "suspend_mode_idx", &pm_info->suspend_mode_idx);
	if (ret) {
		dev_err(dev, "drvinit: unabled to get suspend_mode_idx from DT\n");
		WARN_ON(1);
	}

	ret = of_property_count_u32_elems(np, "wakeup_stat_offset");
	if (!ret) {
		dev_err(dev, "drvinit: unabled to get wakeup_stat value from DT\n");
		WARN_ON(1);
	} else if (ret > 0) {
		pm_info->num_wakeup_stat = ret;
		pm_info->wakeup_stat_offset = devm_kcalloc(dev, ret, sizeof(unsigned int),
							   GFP_KERNEL);
		of_property_read_u32_array(np, "wakeup_stat_offset",
					   pm_info->wakeup_stat_offset, ret);
	}

	ret = of_property_count_u32_elems(np, "wakeup_int_en_offset");
	if (!ret) {
		dev_err(dev, "drvinit: unabled to get wakeup_int_en_offset value from DT\n");
		WARN_ON(1);
	} else if (ret > 0) {
		pm_info->num_wakeup_int_en = ret;
		pm_info->wakeup_int_en_offset = devm_kcalloc(dev, ret, sizeof(unsigned int),
							     GFP_KERNEL);
		of_property_read_u32_array(np, "wakeup_int_en_offset",
					   pm_info->wakeup_int_en_offset, ret);
	}

	ret = of_property_count_u32_elems(np, "wakeup_int_en");
	if (!ret) {
		dev_err(dev, "drvinit: unabled to get wakeup_int_en value from DT\n");
		WARN_ON(1);
	} else if (ret > 0) {
		pm_info->wakeup_int_en = devm_kcalloc(dev, ret, sizeof(unsigned int), GFP_KERNEL);
		of_property_read_u32_array(np, "wakeup_int_en",
					   pm_info->wakeup_int_en, ret);
	}

	ret = of_property_count_u32_elems(np, "usbl2_wakeup_int_en");
	if (!ret) {
		dev_err(dev, "drvinit: dose not support usbl2 sleep\n");
	} else if (ret > 0) {
		pm_info->usbl2_wakeup_int_en = devm_kcalloc(dev, ret, sizeof(unsigned int),
							    GFP_KERNEL);
		of_property_read_u32_array(np, "usbl2_wakeup_int_en",
					   pm_info->usbl2_wakeup_int_en, ret);
	}

	ret = of_property_count_u32_elems(np, "eint_wakeup_mask_offset");
	if (!ret) {
		dev_err(dev, "drvinit: unabled to get eint_wakeup_mask_offset from DT\n");
		WARN_ON(1);
	} else if (ret > 0) {
		pm_info->num_eint_wakeup_mask = ret;
		pm_info->eint_wakeup_mask_offset = devm_kcalloc(dev, ret, sizeof(unsigned int),
								GFP_KERNEL);
		of_property_read_u32_array(np, "eint_wakeup_mask_offset",
					   pm_info->eint_wakeup_mask_offset, ret);
	}

	ret = of_property_read_u32(np, "wake_lock", &wake_lock);
	if (ret) {
		dev_info(dev, "drvinit: unabled to get wake_lock from DT\n");
	} else {
		pm_info->ws = wakeup_source_register(NULL, "exynos-pm");
		if (!pm_info->ws)
			WARN_ON(1);

		pm_info->is_stay_awake = (bool)wake_lock;

		if (pm_info->is_stay_awake)
			__pm_stay_awake(pm_info->ws);
	}

	ret = of_property_read_u32(np, "pcieon_suspend_available",
				   &pm_info->pcieon_suspend_available);
	if (ret) {
		dev_info(dev, "drvinit: Not support pcieon_suspend mode\n");
	} else {
		ret = of_property_read_u32(np, "pcieon_suspend_mode_idx",
					   &pm_info->pcieon_suspend_mode_idx);
		if (ret) {
			dev_err(dev, "drvinit: unabled to get pcieon_suspemd_mode_idx from DT\n");
			WARN_ON(1);
		}
	}

	parse_dt_wakeup_stat_names(dev, np);

	register_syscore_ops(&exynos_pm_syscore_ops);
	register_pm_notifier(&exynos_pm_notifier_block);
#ifdef CONFIG_DEBUG_FS
	exynos_pm_debugfs_init();
#endif

	dev_info(dev, "initialized\n");
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
