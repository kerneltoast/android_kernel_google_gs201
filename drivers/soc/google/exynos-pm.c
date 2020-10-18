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

#include <soc/google/exynos-pm.h>
#include <soc/google/exynos-pmu-if.h>
#include <soc/google/cal-if.h>

#define EXYNOS_EINT_PEND(b, x)      ((b) + 0xA00 + (((x) >> 3) * 4))

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

	if (wakeup_stat & (1 << pm_info->wakeup_stat_rtc)) {
		pr_info("%s Resume caused by RTC alarm\n", EXYNOS_PM_PREFIX);
	} else if (wakeup_stat & pm_info->wakeup_stat_eint) {
		exynos_show_wakeup_reason_eint();
	} else {
		for (i = 0; i < pm_info->num_wakeup_stat; i++) {
			exynos_pmu_read(pm_info->wakeup_stat_offset[i], &wakeup_stat);
			pr_info("%s Resume caused by wakeup%d_stat 0x%08x\n",
				EXYNOS_PM_PREFIX, i + 1, wakeup_stat);
		}
	}
}

extern u32 otg_is_connect(void);

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

	exynos_prepare_sys_powerdown(pm_info->suspend_mode_idx);
	pr_info("%s syscore_suspend: Enter Suspend scenario. suspend_mode_idx = %d)\n",
		EXYNOS_PM_PREFIX, pm_info->suspend_mode_idx);

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

	exynos_wakeup_sys_powerdown(pm_info->suspend_mode_idx, pm_info->is_early_wakeup);
	exynos_show_wakeup_reason(pm_info->is_early_wakeup);

	if (!pm_info->is_early_wakeup)
		pr_debug("%s syscore_resume: post sleep, preparing to return\n",
			 EXYNOS_PM_PREFIX);
}

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

	debugfs_create_u32("test_early_wakeup", 0644, root, &pm_dbg->test_early_wakeup);

	if (pm_info->ws)
		debugfs_create_file("wake_lock", 0644, root, NULL, &wake_lock_fops);
}
#endif

static int exynos_pm_drvinit(void)
{
	int ret;

	pm_info = kzalloc(sizeof(*pm_info), GFP_KERNEL);
	WARN_ON(!pm_info);

	pm_dbg = kzalloc(sizeof(*pm_dbg), GFP_KERNEL);
	WARN_ON(!pm_dbg);

	if (of_have_populated_dt()) {
		struct device_node *np;
		unsigned int wake_lock = 0;

		np = of_find_compatible_node(NULL, NULL, "samsung,exynos-pm");
		if (!np) {
			pr_err("%s drvinit: unabled to find compatible node (%s)\n",
			       EXYNOS_PM_PREFIX, "samsung,exynos-pm");
			WARN_ON(1);
		}

		pm_info->eint_base = of_iomap(np, 0);
		WARN_ON(!pm_info->eint_base);

		pm_info->eint_far_base = of_iomap(np, 1);
		WARN_ON(!pm_info->eint_far_base);

		pm_info->gic_base = of_iomap(np, 2);
		WARN_ON(!pm_info->gic_base);

		ret = of_property_read_u32(np, "num-eint", &pm_info->num_eint);
		if (ret) {
			pr_err("%s drvinit: unabled to get the number of eint from DT\n",
			       EXYNOS_PM_PREFIX);
			WARN_ON(1);
		}

		ret = of_property_read_u32(np, "num-eint-far", &pm_info->num_eint_far);
		if (ret) {
			pr_err("%s drvinit: unabled to get the number of eint-far from DT\n",
			       EXYNOS_PM_PREFIX);
			WARN_ON(1);
		}

		ret = of_property_count_u32_elems(np, "gpa-use");
		if (!ret) {
			pr_err("%s drvinit: unabled to get num-gpa-use from DT\n",
			       EXYNOS_PM_PREFIX);
		} else if (ret > 0) {
			pm_info->num_gpa = ret;
			pm_info->gpa_use = kcalloc(ret, sizeof(unsigned int), GFP_KERNEL);
			of_property_read_u32_array(np, "gpa-use", pm_info->gpa_use, ret);
		}

		ret = of_property_read_u32(np, "num-gic", &pm_info->num_gic);
		if (ret) {
			pr_err("%s drvinit: unabled to get the number of gic from DT\n",
			       EXYNOS_PM_PREFIX);
			WARN_ON(1);
		}

		ret = of_property_read_u32(np, "wakeup-stat-eint", &pm_info->wakeup_stat_eint);
		if (ret) {
			pr_err("%s drvinit: unabled to get the eint bit of WAKEUP_STAT from DT\n",
			       EXYNOS_PM_PREFIX);
			WARN_ON(1);
		}

		ret = of_property_read_u32(np, "wakeup-stat-rtc", &pm_info->wakeup_stat_rtc);
		if (ret) {
			pr_err("%s drvinit: unabled to get the rtc bit of WAKEUP_STAT from DT\n",
			       EXYNOS_PM_PREFIX);
			WARN_ON(1);
		}

		ret = of_property_read_u32(np, "suspend_mode_idx", &pm_info->suspend_mode_idx);
		if (ret) {
			pr_err("%s drvinit: unabled to get suspend_mode_idx from DT\n",
			       EXYNOS_PM_PREFIX);
			WARN_ON(1);
		}

		ret = of_property_count_u32_elems(np, "wakeup_stat_offset");
		if (!ret) {
			pr_err("%s drvinit: unabled to get wakeup_stat value from DT\n",
			       EXYNOS_PM_PREFIX);
			WARN_ON(1);
		} else if (ret > 0) {
			pm_info->num_wakeup_stat = ret;
			pm_info->wakeup_stat_offset = kcalloc(ret, sizeof(unsigned int),
							      GFP_KERNEL);
			of_property_read_u32_array(np, "wakeup_stat_offset",
						   pm_info->wakeup_stat_offset, ret);
		}

		ret = of_property_count_u32_elems(np, "wakeup_int_en_offset");
		if (!ret) {
			pr_err("%s drvinit: unabled to get wakeup_int_en_offset value from DT\n",
			       EXYNOS_PM_PREFIX);
			WARN_ON(1);
		} else if (ret > 0) {
			pm_info->num_wakeup_int_en = ret;
			pm_info->wakeup_int_en_offset = kcalloc(ret, sizeof(unsigned int),
								GFP_KERNEL);
			of_property_read_u32_array(np, "wakeup_int_en_offset",
						   pm_info->wakeup_int_en_offset, ret);
		}

		ret = of_property_count_u32_elems(np, "wakeup_int_en");
		if (!ret) {
			pr_err("%s drvinit: unabled to get wakeup_int_en value from DT\n",
			       EXYNOS_PM_PREFIX);
			WARN_ON(1);
		} else if (ret > 0) {
			pm_info->wakeup_int_en = kcalloc(ret, sizeof(unsigned int), GFP_KERNEL);
			of_property_read_u32_array(np, "wakeup_int_en",
						   pm_info->wakeup_int_en, ret);
		}

		ret = of_property_count_u32_elems(np, "usbl2_wakeup_int_en");
		if (!ret) {
			pr_err("%s drvinit: dose not support usbl2 sleep\n", EXYNOS_PM_PREFIX);
		} else if (ret > 0) {
			pm_info->usbl2_wakeup_int_en = kcalloc(ret, sizeof(unsigned int),
							       GFP_KERNEL);
			of_property_read_u32_array(np, "usbl2_wakeup_int_en",
						   pm_info->usbl2_wakeup_int_en, ret);
		}

		ret = of_property_count_u32_elems(np, "eint_wakeup_mask_offset");
		if (!ret) {
			pr_err("%s drvinit: unabled to get eint_wakeup_mask_offset from DT\n",
			       EXYNOS_PM_PREFIX);
			WARN_ON(1);
		} else if (ret > 0) {
			pm_info->num_eint_wakeup_mask = ret;
			pm_info->eint_wakeup_mask_offset = kcalloc(ret, sizeof(unsigned int),
								   GFP_KERNEL);
			of_property_read_u32_array(np, "eint_wakeup_mask_offset",
						   pm_info->eint_wakeup_mask_offset, ret);
		}

		ret = of_property_read_u32(np, "wake_lock", &wake_lock);
		if (ret) {
			pr_info("%s drvinit: unabled to get wake_lock from DT\n",
				EXYNOS_PM_PREFIX);
		} else {
			pm_info->ws = wakeup_source_register(NULL, "exynos-pm");
			if (!pm_info->ws)
				WARN_ON(1);

			pm_info->is_stay_awake = (bool)wake_lock;

			if (pm_info->is_stay_awake)
				__pm_stay_awake(pm_info->ws);
		}

	} else {
		pr_err("%s drvinit: failed to have populated device tree\n",
		       EXYNOS_PM_PREFIX);
		WARN_ON(1);
	}

	register_syscore_ops(&exynos_pm_syscore_ops);
#ifdef CONFIG_DEBUG_FS
	exynos_pm_debugfs_init();
#endif

	return 0;
}
arch_initcall(exynos_pm_drvinit);

MODULE_LICENSE("GPL");
