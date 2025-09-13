// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Google Zuma SoC devfreq driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/sched/clock.h>
#include <linux/clk.h>
#include <linux/thermal.h>
#include <soc/google/cal-if.h>
#include <soc/google/bts.h>
#include <linux/of_platform.h>
#include <trace/events/power.h>
#include <trace/hooks/systrace.h>
#include <dt-bindings/soc/google/zuma-devfreq.h>
#include "../../soc/google/cal-if/acpm_dvfs.h"
#include <soc/google/exynos-pd.h>

#include <soc/google/exynos-devfreq.h>
#include <soc/google/ect_parser.h>
#include <soc/google/acpm_ipc_ctrl.h>
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
#include <soc/google/exynos-dm.h>
#endif
#if IS_ENABLED(CONFIG_GS_ACPM)
#include "../../soc/google/acpm/acpm.h"
#include "../../soc/google/acpm/acpm_ipc.h"
#endif
#include <soc/google/debug-snapshot.h>

#include "governor.h"

#include "gs-ppc.h"
#include "thermal_core.h"

#define HZ_PER_KHZ	1000

static struct exynos_devfreq_data **devfreq_data;

static u32 freq_array[6];
static u32 boot_array[2];

static struct devfreq *find_exynos_devfreq_device(void *devdata);
static int find_exynos_devfreq_dm_type(struct device *dev, int *dm_type);
static int exynos_devfreq_target(struct device *dev,
				 unsigned long *target_freq, u32 flags);
static u32 exynos_devfreq_opp_round_freq(
				struct exynos_devfreq_opp_table *table,
				unsigned int size, u32 freq);

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
static struct srcu_notifier_head exynos_alt_notifier;

void exynos_alt_call_chain(void)
{
	srcu_notifier_call_chain(&exynos_alt_notifier, 0, NULL);
}

static int exynos_alt_register_notifier(struct notifier_block *nb)
{
	return srcu_notifier_chain_register(&exynos_alt_notifier, nb);
}

static int exynos_alt_unregister_notifier(struct notifier_block *nb)
{
	return srcu_notifier_chain_unregister(&exynos_alt_notifier, nb);
}

static int init_alt_notifier_list(void)
{
	srcu_init_notifier_head(&exynos_alt_notifier);
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)

static int exynos_devfreq_dm_call(struct device *parent,
				  unsigned long *target_freq, u32 flags)
{
	int err = 0;
	struct platform_device *pdev;
	struct exynos_devfreq_data *data;
	unsigned long str_freq;
	int dm_type;
	unsigned long exynos_pm_qos_max;
	struct devfreq *devfreq;
	struct devfreq_simple_interactive_data *gov_data;
	struct dev_pm_opp *max_opp;

	pdev = container_of(parent, struct platform_device, dev);
	if (!pdev)
		return -ENODEV;

	data = platform_get_drvdata(pdev);
	if  (!data || data->devfreq_disabled)
		return -EPROBE_DEFER;

	str_freq = data->suspend_freq;
	devfreq = data->devfreq;
	gov_data = devfreq->data;

	err = find_exynos_devfreq_dm_type(devfreq->dev.parent, &dm_type);
	if (err)
		return -EINVAL;
	ATRACE_BEGIN(__func__);
	exynos_pm_qos_max = (unsigned long)HZ_PER_KHZ * dev_pm_qos_read_value(devfreq->dev.parent,
						  DEV_PM_QOS_MAX_FREQUENCY);

	if (!strcmp(devfreq->governor->name, "interactive") && gov_data->pm_qos_class_max)
		exynos_pm_qos_max =
			(unsigned long)exynos_pm_qos_request(gov_data->pm_qos_class_max);

	max_opp = dev_pm_opp_find_freq_floor(devfreq->dev.parent, &exynos_pm_qos_max);
	if (max_opp == ERR_PTR(-ERANGE)) {
		exynos_pm_qos_max = data->min_freq;
		max_opp = dev_pm_opp_find_freq_ceil(devfreq->dev.parent, &exynos_pm_qos_max);
	}

	dev_pm_opp_put(max_opp);

	if (data->suspend_flag)
		policy_update_call_to_DM(dm_type, str_freq,
					 str_freq);
	else
		policy_update_call_to_DM(dm_type, *target_freq, exynos_pm_qos_max);
	DM_CALL(dm_type, target_freq);

	*target_freq = data->previous_freq;
	ATRACE_END();
	return err;
}

static int exynos_devfreq_get_cur_freq(struct device *dev,
				       unsigned long *freq)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ret = 0;

	*freq = data->previous_freq;

	return ret;
}

/**
 * exynos_devfreq_get_freq_level() - Lookup freq_table for the frequency
 * @devfreq:	the devfreq instance
 * @freq:	the target frequency
 */
static int exynos_devfreq_get_freq_level(struct devfreq *devfreq, unsigned long freq)
{
	int lev;

	for (lev = 0; lev < devfreq->profile->max_state; lev++)
		if (freq == devfreq->profile->freq_table[lev])
			return lev;

	return -EINVAL;
}

/**
 * exynos_devfreq_update_status() - Update statistics of devfreq behavior
 * @devfreq:	the devfreq instance
 * @freq:	the update target frequency
 */
static int exynos_devfreq_update_status(struct exynos_devfreq_data *devdata)
{
	int prev_lev, ret = 0;
	unsigned long cur_time;
	struct devfreq *devfreq;

	devfreq = find_exynos_devfreq_device(devdata);

	cur_time = jiffies;

	/* Immediately exit if previous_freq is not initialized yet. */
	if (!devdata->previous_freq)
		goto out;

	prev_lev = exynos_devfreq_get_freq_level(devfreq, devdata->previous_freq);
	if (prev_lev < 0) {
		ret = prev_lev;
		goto out;
	}

	devdata->time_in_state[prev_lev] +=
			 cur_time - devdata->last_stat_updated;
out:
	devdata->last_stat_updated = cur_time;
	return ret;
}

static int devfreq_frequency_scaler(int dm_type, void *devdata,
				    u32 target_freq, unsigned int relation)
{
	struct devfreq *devfreq;
	unsigned long freq = target_freq;
	unsigned long max_freq, min_freq;
	u32 flags = 0;
	int err = 0;

	ATRACE_BEGIN(__func__);
	devfreq = find_exynos_devfreq_device(devdata);
	if (IS_ERR_OR_NULL(devfreq)) {
		pr_err("%s: No such devfreq for dm_type(%d)\n", __func__, dm_type);
		err = -ENODEV;
		goto err_out;
	}

	/*
	 * Adjust the freuqency with user freq and QoS.
	 *
	 * List from the highest proiority
	 * max_freq (probably called by thermal when it's too hot)
	 * min_freq
	 */

	min_freq = (unsigned long)HZ_PER_KHZ * dev_pm_qos_read_value(devfreq->dev.parent,
					 DEV_PM_QOS_MIN_FREQUENCY);
	if (min_freq && freq < min_freq) {
		freq = min_freq;
		flags &= ~DEVFREQ_FLAG_LEAST_UPPER_BOUND; /* Use GLB */
	}
	max_freq = (unsigned long)HZ_PER_KHZ * dev_pm_qos_read_value(devfreq->dev.parent,
					 DEV_PM_QOS_MAX_FREQUENCY);
	if (max_freq && freq > max_freq) {
		freq = max_freq;
		flags |= DEVFREQ_FLAG_LEAST_UPPER_BOUND; /* Use LUB */
	}

	err = exynos_devfreq_target(devfreq->dev.parent, &freq, flags);

err_out:
	ATRACE_END();
	return err;
}
#endif

static int exynos_devfreq_update_fvp(struct exynos_devfreq_data *data,
				     u32 min_freq, u32 max_freq)
{
	int ret, ch_num, size, i, use_level = 0;
	u32 cmd[4];
	struct ipc_config config;
	int nr_constraint = 0;
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	int j;
	struct exynos_dm_constraint *constraint;

	nr_constraint = data->nr_constraint;
#endif
	ret = acpm_ipc_request_channel(data->dev->of_node, NULL, &ch_num,
				       &size);
	if (ret) {
		dev_err(data->dev,
			"acpm request channel is failed, id:%u, size:%u\n",
			ch_num, size);
		return -EINVAL;
	}
	config.cmd = cmd;
	config.response = true;

	/* constraint info update */
	if (nr_constraint == 0) {
		for (i = 0; i < data->max_state; i++) {
			if (data->opp_list[i].freq > max_freq ||
			    data->opp_list[i].freq < min_freq)
				continue;

			config.cmd[0] = use_level;
			config.cmd[1] = data->opp_list[i].freq;
			config.cmd[2] = DATA_INIT;
			config.cmd[3] = 0;
			ret = acpm_ipc_send_data(ch_num, &config);
			if (ret) {
				dev_err(data->dev,
					"make constraint table is failed");
				return -EINVAL;
			}
			use_level++;
		}
	} else {
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
		for (i = 0; i < data->nr_constraint; i++) {
			constraint = data->constraint[i];
			for (j = 0; j < data->max_state; j++) {
				if (data->opp_list[j].freq > max_freq ||
				    data->opp_list[j].freq < min_freq)
					continue;

				config.cmd[0] = use_level;
				config.cmd[1] = data->opp_list[j].freq;
				config.cmd[2] = DATA_INIT;
				config.cmd[3] =
					constraint->freq_table[use_level]
						.constraint_freq;
				ret = acpm_ipc_send_data(ch_num, &config);
				if (ret) {
					dev_err(data->dev,
						"make constraint table is failed");
					return -EINVAL;
				}
				use_level++;
			}
		}
		/* Send MIF initial freq and the number of constraint data to FVP */
		config.cmd[0] = use_level;
		config.cmd[1] =
			(unsigned int)data->devfreq_profile.initial_freq;
		config.cmd[2] = DATA_INIT;
		config.cmd[3] = SET_CONST;

		ret = acpm_ipc_send_data(ch_num, &config);
		if (ret) {
			dev_err(data->dev,
				"failed to send nr_constraint and init freq");
			return -EINVAL;
		}
	}
#endif

	return 0;
}

static int exynos_devfreq_reboot(struct exynos_devfreq_data *data)
{
	if (exynos_pm_qos_request_active(&data->default_pm_qos_max))
		exynos_pm_qos_update_request(&data->default_pm_qos_max,
					     data->reboot_freq);
	return 0;
}

static unsigned long _exynos_devfreq_get_freq(unsigned int devfreq_type)
{
	struct exynos_devfreq_data *data = NULL;
	unsigned long freq;

	if (devfreq_data)
		data = devfreq_data[devfreq_type];

	if (!data) {
		pr_err("Fail to get exynos_devfreq_data\n");
		return 0;
	}

	if (data->clk)
		freq = data->old_freq;
	else
		freq = cal_dfs_get_rate(data->dfs_id);

	if ((u32)freq == 0) {
		if (data->clk)
			dev_err(data->dev,
				"failed get frequency from clock framework\n");
		else
			dev_err(data->dev, "failed get frequency from CAL\n");

		return 0;
	}

	return freq;
}

unsigned long exynos_devfreq_get_domain_freq(unsigned int devfreq_type)
{
	return _exynos_devfreq_get_freq(devfreq_type);
}
EXPORT_SYMBOL(exynos_devfreq_get_domain_freq);

static int exynos_devfreq_get_freq(struct device *dev, u32 *cur_freq,
				   struct clk *clk,
				   struct exynos_devfreq_data *data)
{
	if (data->pm_domain) {
		if (!exynos_pd_status(data->pm_domain)) {
			dev_err(dev, "power domain %s is offed\n",
				data->pm_domain->name);
			*cur_freq = 0;
			return -EINVAL;
		}
	}

	*cur_freq = (u32)_exynos_devfreq_get_freq(data->devfreq_type);

	if (*cur_freq == 0) {
		dev_err(dev, "failed get frequency\n");
		return -EINVAL;
	}

	return 0;
}

int exynos_devfreq_get_boundary(unsigned int devfreq_type,
				unsigned int *max_freq, unsigned int *min_freq)
{
	struct exynos_devfreq_data *data = NULL;

	if ((devfreq_type >= DEVFREQ_MIF) && (devfreq_type < DEVFREQ_TYPE_END)) {
		data = devfreq_data[devfreq_type];
	} else
		return -EINVAL;

	if (!data) {
		pr_err("%s, Fail to get exynos_devfreq_data\n", __func__);
		return -ENOMEM;
	}

	*max_freq = data->max_freq;
	*min_freq = data->min_freq;

	return 0;
}
EXPORT_SYMBOL(exynos_devfreq_get_boundary);

int exynos_devfreq_lock_freq(unsigned int devfreq_type, unsigned int target_freq)
{
	struct exynos_devfreq_data *data = NULL;
	int ret;

	if ((devfreq_type >= DEVFREQ_MIF) && (devfreq_type < DEVFREQ_TYPE_END)) {
		data = devfreq_data[devfreq_type];
	} else
		return -EINVAL;

	if (!data) {
		pr_err("%s, Fail to get exynos_devfreq_data\n", __func__);
		return -ENOMEM;
	}

	ret = devfreq_frequency_scaler(devfreq_type, data, target_freq, 0);

	return ret;
}
EXPORT_SYMBOL(exynos_devfreq_lock_freq);

static int exynos_devfreq_set_freq(struct device *dev, u32 new_freq,
				   struct clk *clk,
				   struct exynos_devfreq_data *data)
{
	if (data->bts_update) {
		if (data->new_freq < data->old_freq)
			bts_update_scen(BS_MIF_CHANGE, data->new_freq);
	}

	if (data->pm_domain) {
		if (!exynos_pd_status(data->pm_domain)) {
			dev_err(dev, "power domain %s is offed\n",
				data->pm_domain->name);
			return -EINVAL;
		}
	}

	if (cal_dfs_set_rate(data->dfs_id, (unsigned long)new_freq)) {
		dev_err(dev, "failed set frequency to CAL (%uKhz)\n", new_freq);
		return -EINVAL;
	}
	trace_clock_set_rate(dev_name(data->dev), new_freq, raw_smp_processor_id());
	if (data->bts_update) {
		if (data->new_freq > data->old_freq)
			bts_update_scen(BS_MIF_CHANGE, data->new_freq);
	}

	return 0;
}

static u32 exynos_devfreq_nudge_freq(struct device *dev, u32 freq, bool find_higher)
{
	unsigned long tmp_freq = freq;
	u32 flags = 0;
	struct dev_pm_opp *target_opp;

	if (!find_higher)
		flags |= DEVFREQ_FLAG_LEAST_UPPER_BOUND;

	target_opp = devfreq_recommended_opp(dev,
					     &tmp_freq,
					     flags);
	if (IS_ERR(target_opp)) {
		dev_err(dev, "No valid OPP to nudge freq %u to!\n", freq);
		return 0;
	}

	tmp_freq = (u32)dev_pm_opp_get_freq(target_opp);
	dev_pm_opp_put(target_opp);

	if (freq != tmp_freq) {
		dev_info(dev,
			 "Nudged %u KHz to the %s %lu KHz freq.\n",
			 freq,
			 freq > tmp_freq ? "higher" : "lower",
			 tmp_freq);
	}

	return tmp_freq;
}

int exynos_devfreq_init_freq_table(struct exynos_devfreq_data *data)
{
	u32 max_freq, min_freq;
	u32 nudged_freq;
	int i, ret;

	max_freq = (u32)cal_dfs_get_max_freq(data->dfs_id);
	if (!max_freq) {
		dev_err(data->dev, "failed get max frequency\n");
		return -EINVAL;
	}

	dev_dbg(data->dev, "max_freq: %uKhz, get_max_freq: %uKhz\n",
		 data->max_freq, max_freq);

	if (max_freq < data->max_freq)
		dev_warn(data->dev, "DT max freq is higher than ECT max freq!\n");

	nudged_freq = exynos_devfreq_nudge_freq(data->dev, data->max_freq, /*find_higher*/ 0);
	if (!nudged_freq)
		return -EINVAL;
	data->max_freq = nudged_freq;

	/* min ferquency must be equal or under max frequency */
	if (data->min_freq > data->max_freq)
		data->min_freq = data->max_freq;

	min_freq = (u32)cal_dfs_get_min_freq(data->dfs_id);
	if (!min_freq) {
		dev_err(data->dev, "failed get min frequency\n");
		return -EINVAL;
	}

	dev_dbg(data->dev, "min_freq: %uKhz, get_min_freq: %uKhz\n",
		 data->min_freq, min_freq);

	if (min_freq > data->min_freq)
		dev_warn(data->dev, "DT min freq is lower than ECT min freq!\n");

	nudged_freq = exynos_devfreq_nudge_freq(data->dev, data->min_freq, /*find_higher*/ 1);
	if (!nudged_freq)
		return -EINVAL;
	data->min_freq = nudged_freq;

	dev_dbg(data->dev, "min_freq: %uKhz, max_freq: %uKhz\n",
		 data->min_freq, data->max_freq);

	for (i = 0; i < data->max_state; i++) {
		if (data->opp_list[i].freq > data->max_freq ||
		    data->opp_list[i].freq < data->min_freq)
			dev_pm_opp_disable(data->dev,
					   (unsigned long)data->opp_list[i].freq);
	}

	data->devfreq_profile.initial_freq =
		cal_dfs_get_boot_freq(data->dfs_id);
	data->suspend_freq =
		cal_dfs_get_resume_freq(data->dfs_id);
	data->devfreq_profile.initial_freq =
		min(data->devfreq_profile.initial_freq,
		    (unsigned long)data->max_freq);
	data->devfreq_profile.initial_freq =
		max(data->devfreq_profile.initial_freq,
		    (unsigned long)data->min_freq);
	data->suspend_freq =
		min(data->suspend_freq,
		    (unsigned long)data->max_freq);
	data->suspend_freq =
		max(data->suspend_freq,
		    (unsigned long)data->min_freq);

	data->devfreq_profile.initial_freq =
		exynos_devfreq_opp_round_freq(data->opp_list, data->max_state,
			data->devfreq_profile.initial_freq);
	data->max_freq =
		exynos_devfreq_opp_round_freq(data->opp_list, data->max_state,
			data->max_freq);
	data->min_freq =
		exynos_devfreq_opp_round_freq(data->opp_list, data->max_state,
			data->min_freq);
	data->suspend_freq =
		exynos_devfreq_opp_round_freq(data->opp_list, data->max_state,
			data->suspend_freq);

	dev_dbg(data->dev, "initial_freq: %luKhz, suspend_freq: %luKhz\n",
		 data->devfreq_profile.initial_freq,
		 data->suspend_freq);

	if (data->update_fvp)
		exynos_devfreq_update_fvp(data, min_freq, max_freq);

	if (data->use_acpm) {
		ret = exynos_acpm_set_init_freq(data->dfs_id,
						data->devfreq_profile.initial_freq);
		if (ret) {
			dev_err(data->dev, "failed to set init freq\n");
			return -EINVAL;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_devfreq_init_freq_table);

static ssize_t show_exynos_devfreq_info(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count = 0;
	int i;

	count = snprintf(buf, PAGE_SIZE,
			 "[Exynos DEVFREQ Data]\n"
			 "devfreq dev name : %20s\n"
			 "devfreq type     : %20d\n"
			 "Exynos SS flag   : %20u\n",
			 dev_name(data->dev), data->devfreq_type,
			 data->ess_flag);

	count += snprintf(buf + count, PAGE_SIZE,
			  "\n<Frequency data>\n"
			  "OPP list length  : %20u\n",
			  data->max_state);
	count += snprintf(buf + count, PAGE_SIZE, "freq opp table\n");
	count += snprintf(buf + count, PAGE_SIZE,
			  "\t  idx      freq       volt\n");

	for (i = 0; i < data->max_state; i++)
		count += snprintf(buf + count, PAGE_SIZE, "\t%5u %10u %10u\n",
				  data->opp_list[i].idx, data->opp_list[i].freq,
				  data->opp_list[i].volt);

	count += snprintf(buf + count, PAGE_SIZE,
			  "default_qos     : %20u\n"
			  "initial_freq    : %20lu\n"
			  "min_freq        : %20u\n"
			  "max_freq        : %20u\n"
			  "boot_timeout(s) : %20u\n"
			  "max_state       : %20u\n",
			  data->default_qos, data->devfreq_profile.initial_freq,
			  data->min_freq, data->max_freq,
			  data->boot_qos_timeout, data->max_state);

	count += snprintf(buf + count, PAGE_SIZE, "\n<Governor data>\n");
	count += snprintf(buf + count, PAGE_SIZE, "governor_name   : %20s\n",
			  data->governor_name);
	return count;
}

static ssize_t show_exynos_devfreq_get_freq(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count = 0;
	u32 get_freq = 0;

	if (exynos_devfreq_get_freq(data->dev, &get_freq, data->clk, data))
		dev_err(data->dev, "failed get freq\n");

	count = snprintf(buf, PAGE_SIZE, "%10u Khz\n", get_freq);

	return count;
}

static ssize_t show_exynos_devfreq_cmu_dump(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	ssize_t count = 0;

	count = snprintf(buf, PAGE_SIZE, "Done\n");

	return count;
}

static ssize_t show_debug_scaling_devfreq_max(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count = 0;
	int val;

	if (data->pm_qos_class_max) {
		val = exynos_pm_qos_read_req_value(data->pm_qos_class_max,
						   &data->debug_pm_qos_max);
		if (val < 0) {
			dev_err(dev, "failed to read requested value\n");
			return count;
		}
		count += snprintf(buf, PAGE_SIZE, "%d\n", val);
	}

	return count;
}

static ssize_t store_debug_scaling_devfreq_max(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ret;
	u32 qos_value;

	ret = sscanf(buf, "%u", &qos_value);
	if (ret != 1)
		return -EINVAL;

	if (data->pm_qos_class_max) {
		if (exynos_pm_qos_request_active(&data->debug_pm_qos_max))
			exynos_pm_qos_update_request(&data->debug_pm_qos_max,
						     qos_value);
	}

	return count;
}

static ssize_t show_soft_max_freq(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count = 0;
	int val;

	if (data->pm_qos_class_max) {
		val = exynos_pm_qos_read_req_value(data->pm_qos_class_max,
						   &data->pm_qos_soft_max_freq);
		if (val < 0) {
			dev_err(dev, "failed to read requested value\n");
			return count;
		}
		count += snprintf(buf, PAGE_SIZE, "%d\n", val);
	}

	return count;
}

static ssize_t store_soft_max_freq(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ret;
	u32 qos_value;

	ret = sscanf(buf, "%u", &qos_value);
	if (ret != 1)
		return -EINVAL;

	if (data->pm_qos_class_max) {
		if (exynos_pm_qos_request_active(&data->pm_qos_soft_max_freq))
			exynos_pm_qos_update_request(&data->pm_qos_soft_max_freq,
						     qos_value);
	}

	return count;
}


static ssize_t show_debug_scaling_devfreq_min(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count = 0;
	int val;

	val = exynos_pm_qos_read_req_value(data->pm_qos_class,
					   &data->debug_pm_qos_min);
	if (val < 0) {
		dev_err(dev, "failed to read requested value\n");
		return count;
	}

	count += snprintf(buf, PAGE_SIZE, "%d\n", val);

	return count;
}

static ssize_t store_debug_scaling_devfreq_min(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ret;
	u32 qos_value;

	ret = sscanf(buf, "%u", &qos_value);
	if (ret != 1)
		return -EINVAL;

	if (exynos_pm_qos_request_active(&data->debug_pm_qos_min))
		exynos_pm_qos_update_request(&data->debug_pm_qos_min, qos_value);

	return count;
}

static ssize_t cancel_boot_freq_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ret;
	bool cancel_flag;

	ret = kstrtobool(buf, &cancel_flag);
	if (ret) {
		dev_err(dev, "Failed to store cancel_boot\n");
		return ret;
	}

	if (cancel_flag) {
		exynos_pm_qos_update_request_timeout(&data->boot_pm_qos,
						     data->boot_freq,
						     0);
	}
	return count;
}

static ssize_t show_alt_dvfs_info(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count = 0;
	int i;

	mutex_lock(&data->devfreq->lock);

	for (i = 0; i < data->simple_interactive_data.alt_data.num_target_load;
	     i++) {
		count += snprintf(buf + count, PAGE_SIZE, "%d%s",
				  data->simple_interactive_data.alt_data.target_load[i],
				  (i ==
				   (data->simple_interactive_data.alt_data.num_target_load - 1)) ?
				   "" : " ");
	}
	count += snprintf(buf + count, PAGE_SIZE, "\n");
	/* Parameters */
	count += snprintf(buf + count, PAGE_SIZE, "MIN SAMPLE TIME: %u\n",
			  data->simple_interactive_data.alt_data.min_sample_time);
	count += snprintf(buf + count, PAGE_SIZE, "HOLD SAMPLE TIME: %u\n",
			  data->simple_interactive_data.alt_data.hold_sample_time);
	count += snprintf(buf + count, PAGE_SIZE, "HISPEED LOAD: %u\n",
			  data->simple_interactive_data.alt_data.hispeed_load);
	count += snprintf(buf + count, PAGE_SIZE, "HISPEED FREQ: %u\n",
			  data->simple_interactive_data.alt_data.hispeed_freq);

	mutex_unlock(&data->devfreq->lock);

	return count;
#else
	return 0;
#endif
}

static ssize_t fw_freq_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	long freq =  cal_dfs_get_rate_acpm(data->dfs_id);

	return sysfs_emit(buf, "%ld\n", freq);
}

static DEVICE_ATTR(alt_dvfs_info, 0640, show_alt_dvfs_info, NULL);

static DEVICE_ATTR(exynos_devfreq_info, 0640, show_exynos_devfreq_info, NULL);
static DEVICE_ATTR(exynos_devfreq_get_freq, 0640, show_exynos_devfreq_get_freq,
		   NULL);
static DEVICE_ATTR(exynos_devfreq_cmu_dump, 0640, show_exynos_devfreq_cmu_dump,
		   NULL);
static DEVICE_ATTR(debug_scaling_devfreq_min, 0640,
		   show_debug_scaling_devfreq_min,
		   store_debug_scaling_devfreq_min);
static DEVICE_ATTR(debug_scaling_devfreq_max, 0640,
		   show_debug_scaling_devfreq_max,
		   store_debug_scaling_devfreq_max);
static DEVICE_ATTR(soft_max_freq, 0640,
		   show_soft_max_freq,
		   store_soft_max_freq);
static DEVICE_ATTR_WO(cancel_boot_freq);
static DEVICE_ATTR_RO(fw_freq);

static ssize_t time_in_state_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev->parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	struct devfreq *devfreq = to_devfreq(dev);
	ssize_t len = 0;
	int i, err;
	unsigned int max_state = devfreq->profile->max_state;

	mutex_lock(&data->lock);
	err = exynos_devfreq_update_status(data);
	if (err)
		return 0;

	for (i = 0; i < max_state; i++) {
		len += sprintf(buf + len, "%8lu",
				devfreq->profile->freq_table[i]);
		len += sprintf(buf + len, "%10u\n",
			jiffies_to_msecs(data->time_in_state[i]));
	}
	mutex_unlock(&data->lock);
	return len;
}
static DEVICE_ATTR_RO(time_in_state);

static struct attribute *exynos_devfreq_sysfs_entries[] = {
	&dev_attr_exynos_devfreq_info.attr,
	&dev_attr_exynos_devfreq_get_freq.attr,
	&dev_attr_exynos_devfreq_cmu_dump.attr,
	&dev_attr_debug_scaling_devfreq_min.attr,
	&dev_attr_debug_scaling_devfreq_max.attr,
	&dev_attr_soft_max_freq.attr,
	&dev_attr_alt_dvfs_info.attr,
	&dev_attr_cancel_boot_freq.attr,
	&dev_attr_fw_freq.attr,
	NULL,
};

static struct attribute_group exynos_devfreq_attr_group = {
	.name = "exynos_data",
	.attrs = exynos_devfreq_sysfs_entries,
};

static ssize_t show_scaling_devfreq_min(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count = 0;
	int val;

	val = exynos_pm_qos_read_req_value(data->pm_qos_class, &data->sys_pm_qos_min);
	if (val < 0) {
		dev_err(dev, "failed to read requested value\n");
		return count;
	}

	count += snprintf(buf, PAGE_SIZE, "%d\n", val);

	return count;
}

static ssize_t store_scaling_devfreq_min(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ret;
	u32 qos_value;

	ret = sscanf(buf, "%u", &qos_value);
	if (ret != 1)
		return -EINVAL;

	if (exynos_pm_qos_request_active(&data->sys_pm_qos_min))
		exynos_pm_qos_update_request(&data->sys_pm_qos_min, qos_value);

	return count;
}

static DEVICE_ATTR(scaling_devfreq_min, 0640, show_scaling_devfreq_min,
		   store_scaling_devfreq_min);

/* get frequency and delay time data from string */
static unsigned int *get_tokenized_data(const char *buf, int *num_tokens)
{
	const char *cp;
	int i;
	int ntokens = 1;
	unsigned int *tokenized_data;
	int err = -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	tokenized_data = kmalloc_array(ntokens, sizeof(unsigned int), GFP_KERNEL);
	if (!tokenized_data) {
		err = -ENOMEM;
		goto err;
	}

	cp = buf;
	i = 0;
	while (i < ntokens) {
		if (sscanf(cp, "%u", &tokenized_data[i++]) != 1)
			goto err_kfree;

		cp = strpbrk(cp, " :");
		if (!cp)
			break;
		cp++;
	}

	if (i != ntokens)
		goto err_kfree;

	*num_tokens = ntokens;
	return tokenized_data;

err_kfree:
	kfree(tokenized_data);
err:
	return ERR_PTR(err);
}

static ssize_t show_use_delay_time(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count = 0;

	mutex_lock(&data->devfreq->lock);
	count += snprintf(buf, PAGE_SIZE, "%s\n",
			  (data->simple_interactive_data.use_delay_time) ?
				  "true" :
				  "false");
	mutex_unlock(&data->devfreq->lock);
	return count;
}

static ssize_t store_use_delay_time(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ret, use_delay_time;

	ret = sscanf(buf, "%d", &use_delay_time);

	if (ret != 1)
		return -EINVAL;

	if (use_delay_time == 0 || use_delay_time == 1) {
		mutex_lock(&data->devfreq->lock);
		data->simple_interactive_data.use_delay_time =
			use_delay_time ? true : false;
		mutex_unlock(&data->devfreq->lock);
	} else {
		dev_info(data->dev, "This is invalid value: %d\n",
			 use_delay_time);
	}

	return count;
}

static ssize_t show_delay_time(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count = 0;
	int i;

	mutex_lock(&data->devfreq->lock);
	for (i = 0; i < data->simple_interactive_data.ndelay_time; i++) {
		count += snprintf(buf + count, PAGE_SIZE, "%d%s",
				  data->simple_interactive_data.delay_time[i],
				  (i == data->simple_interactive_data.ndelay_time - 1) ?
				  "" :
				  (i % 2) ? ":" : " ");
	}
	count += snprintf(buf + count, PAGE_SIZE, "\n");
	mutex_unlock(&data->devfreq->lock);
	return count;
}

static ssize_t store_delay_time(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ntokens;
	int *new_delay_time = NULL;

	new_delay_time = get_tokenized_data(buf, &ntokens);

	if (!(ntokens & 0x1)) {
		dev_err(data->dev,
			"Only odd number of tokens allowed\n");
		return -EINVAL;
	}
	if (IS_ERR(new_delay_time))
		return PTR_ERR(new_delay_time);

	mutex_lock(&data->devfreq->lock);
	kfree(data->simple_interactive_data.delay_time);
	data->simple_interactive_data.delay_time = new_delay_time;
	data->simple_interactive_data.ndelay_time = ntokens;
	mutex_unlock(&data->devfreq->lock);

	return count;
}

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
static ssize_t show_target_load(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count = 0;
	int i;

	mutex_lock(&data->devfreq->lock);
	for (i = 0; i < data->simple_interactive_data.alt_data.num_target_load;
	     i++) {
		count += snprintf(buf + count, PAGE_SIZE, "%d%s",
				  data->simple_interactive_data.alt_data.target_load[i],
				  (i ==
				   (data->simple_interactive_data.alt_data.num_target_load - 1) ?
				   "" : " "));
	}
	count += snprintf(buf + count, PAGE_SIZE, "\n");
	mutex_unlock(&data->devfreq->lock);
	return count;
}

static ssize_t store_target_load(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ntokens;
	int *new_target_load = NULL;

	new_target_load = get_tokenized_data(buf, &ntokens);
	if (ntokens != data->simple_interactive_data.alt_data.num_target_load) {
		dev_err(data->dev, "Invalid counts of target_load\n");
		return -EINVAL;
	}

	if (IS_ERR(new_target_load))
		return PTR_ERR(new_target_load);

	mutex_lock(&data->devfreq->lock);
	kfree(data->simple_interactive_data.alt_data.target_load);
	data->simple_interactive_data.alt_data.target_load = new_target_load;
	data->simple_interactive_data.alt_data.num_target_load = ntokens;
	mutex_unlock(&data->devfreq->lock);

	return count;
}

static ssize_t show_hold_sample_time(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count = 0;

	mutex_lock(&data->devfreq->lock);
	count += snprintf(buf, PAGE_SIZE, "%u\n",
			  data->simple_interactive_data.alt_data.hold_sample_time);
	mutex_unlock(&data->devfreq->lock);
	return count;
}

static ssize_t store_hold_sample_time(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ret;

	mutex_lock(&data->devfreq->lock);
	ret = sscanf(buf, "%u",
		     &data->simple_interactive_data.alt_data.hold_sample_time);
	mutex_unlock(&data->devfreq->lock);
	if (ret != 1)
		return -EINVAL;

	return count;
}

static ssize_t min_sample_time_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	ssize_t count;

	mutex_lock(&data->devfreq->lock);
	count = snprintf(buf, PAGE_SIZE, "%u\n",
			 data->simple_interactive_data.alt_data.min_sample_time);
	mutex_unlock(&data->devfreq->lock);
	return count;
}

static ssize_t min_sample_time_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct device *parent = dev->parent;
	struct platform_device *pdev =
		container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	unsigned int min_sample_time;

	if (kstrtouint(buf, 10, &min_sample_time) || !min_sample_time)
		return -EINVAL;

	mutex_lock(&data->devfreq->lock);
	data->simple_interactive_data.alt_data.min_sample_time =
		min_sample_time;
	mutex_unlock(&data->devfreq->lock);

	return count;
}
#endif

static ssize_t ppc_read_reset_disable_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	ssize_t count = 0;
	struct device *parent = dev->parent;
	struct platform_device *pdev =
			container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);

	count += sysfs_emit_at(buf, count, "%s\n",
              data->um_data.ppc_read_reset_disable==true? "true":"false");

	return count;
}

static ssize_t ppc_read_reset_disable_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int ret;
	struct device *parent = dev->parent;
	struct platform_device *pdev =
			container_of(parent, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);

	ret = kstrtobool(buf, &data->um_data.ppc_read_reset_disable);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR(use_delay_time, 0640, show_use_delay_time,
		   store_use_delay_time);
static DEVICE_ATTR(delay_time, 0640, show_delay_time, store_delay_time);
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
static DEVICE_ATTR(target_load, 0640, show_target_load, store_target_load);
static DEVICE_ATTR(hold_sample_time, 0640, show_hold_sample_time,
		   store_hold_sample_time);
static DEVICE_ATTR_RW(min_sample_time);
#endif
static DEVICE_ATTR_RW(ppc_read_reset_disable);

static struct attribute *devfreq_interactive_sysfs_entries[] = {
	&dev_attr_use_delay_time.attr,
	&dev_attr_delay_time.attr,
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	&dev_attr_target_load.attr,
	&dev_attr_hold_sample_time.attr,
	&dev_attr_min_sample_time.attr,
#endif
	NULL,
};

static struct attribute_group devfreq_delay_time_attr_group = {
	.name = "interactive",
	.attrs = devfreq_interactive_sysfs_entries,
};

#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
static int find_exynos_devfreq_dm_type(struct device *dev, int *dm_type)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);

	*dm_type = data->dm_type;

	return 0;
}

static struct devfreq *find_exynos_devfreq_device(void *devdata)
{
	struct exynos_devfreq_data *data = devdata;

	if (!devdata) {
		pr_err("%s: failed get Devfreq type\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	return data->devfreq;
}
#endif

#ifdef CONFIG_OF
#if IS_ENABLED(CONFIG_ECT)
int exynos_devfreq_parse_ect(struct exynos_devfreq_data *data,
				    const char *dvfs_domain_name)
{
	int i, start_index;
	void *dvfs_block;
	struct ect_dvfs_domain *dvfs_domain;

	dvfs_block = ect_get_block(BLOCK_DVFS);
	if (!dvfs_block)
		return -ENODEV;

	dvfs_domain = ect_dvfs_get_domain(dvfs_block, (char *)dvfs_domain_name);
	if (!dvfs_domain)
		return -ENODEV;

	for (i = 0; i < dvfs_domain->num_of_level; i++) {
		if (data->max_freq >= dvfs_domain->list_level[i].level)
			break;
	}
	start_index = i;

	if (start_index >= dvfs_domain->num_of_level)
		return -EINVAL;

	data->max_state = dvfs_domain->num_of_level - start_index;

	data->opp_list = kcalloc(data->max_state,
				 sizeof(struct exynos_devfreq_opp_table),
				 GFP_KERNEL);
	if (!data->opp_list) {
		pr_err("%s: failed to allocate opp_list\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < data->max_state; i++) {
		data->opp_list[i].idx = i;
		data->opp_list[i].freq = dvfs_domain->list_level[i + start_index].level;
		data->opp_list[i].volt = 0;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(exynos_devfreq_parse_ect);
#endif

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
#define NUM_COLS 2
#endif

static int exynos_devfreq_parse_dt(struct device_node *np,
				   struct exynos_devfreq_data *data)
{
	const char *use_acpm, *bts_update;
	const char *use_get_dev;
#if IS_ENABLED(CONFIG_ECT)
	const char *devfreq_domain_name;
#endif
	const char *buf;
	const char *use_delay_time;
	const char *pd_name;
	const char *update_fvp;
	int ntokens;
	int not_using_ect = true;
	u32 count_total = 0;
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	struct devfreq_alt_dvfs_data *alt_data;
	u32 *of_data_int_array = NULL;
	int map_cnt;
#endif
	unsigned int i;
	unsigned int val;
	unsigned int *volt_table;

	if (!np)
		return -ENODEV;

	if (of_property_read_u32(np, "devfreq_type", &data->devfreq_type))
		return -ENODEV;
	if (of_property_read_u32(np, "pm_qos_class", &data->pm_qos_class))
		return -ENODEV;
	if (of_property_read_u32(np, "pm_qos_class_max",
				 &data->pm_qos_class_max))
		return -ENODEV;
	if (of_property_read_u32(np, "ess_flag", &data->ess_flag))
		return -ENODEV;

	if (of_property_read_u32_array(np, "freq_info", (u32 *)&freq_array,
				       (size_t)(ARRAY_SIZE(freq_array))))
		return -ENODEV;

	data->devfreq_profile.initial_freq = freq_array[0];
	data->default_qos = freq_array[1];
	data->suspend_freq = freq_array[2];
	data->min_freq = freq_array[3];
	data->max_freq = freq_array[4];
	data->reboot_freq = freq_array[5];

#if IS_ENABLED(CONFIG_ECT)
	if (of_property_read_string(np, "devfreq_domain_name",
				    &devfreq_domain_name))
		return -ENODEV;
	not_using_ect = exynos_devfreq_parse_ect(data, devfreq_domain_name);
#endif
	if (not_using_ect) {
		dev_err(data->dev, "cannot parse the DVFS info in ECT");
		return -ENODEV;
	}

	if (of_property_read_string(np, "pd_name", &pd_name)) {
		dev_dbg(data->dev, "no power domain\n");
		data->pm_domain = NULL;
	} else {
		dev_dbg(data->dev, "power domain: %s\n", pd_name);
		data->pm_domain = exynos_pd_lookup_name(pd_name);
	}

	data->clk = devm_clk_get(data->dev, "DEVFREQ");
#if IS_ENABLED(CONFIG_ECT)
	if (data->clk && !IS_ERR(data->clk))
		dev_info(data->dev, "%s clock info exist\n",
			 devfreq_domain_name);
	else
		data->clk = NULL;
#endif

	if (of_property_read_u32(np, "dfs_id", &data->dfs_id) &&
	    of_property_match_string(np, "clock-names", buf))
		return -ENODEV;

	data->soft_max_freq = (u32)cal_dfs_get_max_freq(data->dfs_id);

	/* Optionally read softmax (as potential ECT override) */
	of_property_read_u32(np, "soft_max_freq",
				 &data->soft_max_freq);

	if (!data->soft_max_freq)
		data->soft_max_freq = data->max_freq;
	else
		data->soft_max_freq = min(data->soft_max_freq,
			data->max_freq);

	if (data->devfreq_profile.initial_freq)
		data->devfreq_profile.initial_freq = min_t(
			u32,
			data->devfreq_profile.initial_freq,
			data->soft_max_freq);
	else
		data->devfreq_profile.initial_freq = (u32)
			(data->soft_max_freq);

	if (data->default_qos)
		data->default_qos = min_t(
			u32,
			data->default_qos,
			data->soft_max_freq);
	else
		data->default_qos = (u32)(data->soft_max_freq);

	if (!of_property_read_u32(np, "max-volt", &val) &&
	    !of_property_read_u32(np, "dfs_id", &data->dfs_id)) {
		volt_table = kzalloc(sizeof(unsigned int) * data->max_state,
				     GFP_KERNEL);
		if (!volt_table)
			return -ENOMEM;
		cal_dfs_get_asv_table(data->dfs_id, volt_table);
		for (i = 0; i < data->max_state; i++) {
			if (volt_table[i] <= val)
				break;
		}
		data->max_freq = min(data->max_freq, data->opp_list[i].freq);
		kfree(volt_table);
	}

	if (of_property_read_u32_array(np, "boot_info", (u32 *)&boot_array,
				       (size_t)(ARRAY_SIZE(boot_array)))) {
		data->boot_qos_timeout = 0;
		data->boot_freq = 0;
		dev_dbg(data->dev, "This doesn't use boot value\n");
	} else {
		data->boot_qos_timeout = boot_array[0];
		data->boot_freq = boot_array[1];
	}
	data->boot_freq = min(data->boot_freq, data->soft_max_freq);

	if (of_property_read_u32(np, "governor", &data->gov_type))
		return -ENODEV;
	if (data->gov_type == SIMPLE_INTERACTIVE) {
		data->governor_name = "interactive";
	} else {
		dev_err(data->dev, "invalid governor name (%s)\n",
			data->governor_name);
		return -EINVAL;
	}

	if (!of_property_read_string(np, "use_acpm", &use_acpm)) {
		if (!strcmp(use_acpm, "true")) {
			data->use_acpm = true;
		} else {
			data->use_acpm = false;
			dev_dbg(data->dev, "This does not use acpm\n");
		}
	} else {
		dev_dbg(data->dev, "This does not use acpm\n");
		data->use_acpm = false;
	}

	if (!of_property_read_string(np, "bts_update", &bts_update)) {
		if (!strcmp(bts_update, "true")) {
			data->bts_update = true;
		} else {
			data->bts_update = false;
			dev_dbg(data->dev, "This does not bts update\n");
		}
	} else {
		dev_dbg(data->dev, "This does not bts update\n");
		data->bts_update = false;
	}

	if (!of_property_read_string(np, "update_fvp", &update_fvp)) {
		if (!strcmp(update_fvp, "true")) {
			data->update_fvp = true;
		} else {
			data->update_fvp = false;
			dev_dbg(data->dev, "This does not update fvp\n");
		}
	} else {
		dev_dbg(data->dev, "This does not update fvp\n");
		data->update_fvp = false;
	}

	if (!of_property_read_string(np, "use_get_dev", &use_get_dev)) {
		if (!strcmp(use_get_dev, "true")) {
			data->use_get_dev = true;
		} else if (!strcmp(use_get_dev, "false")) {
			data->use_get_dev = false;
		} else {
			dev_err(data->dev, "invalid use_get_dev string (%s)\n",
				use_get_dev);
			return -EINVAL;
		}
	} else {
		dev_dbg(data->dev,
			 "Operation function get_dev_status will not be registered.\n");
		data->use_get_dev = false;
	}

	of_property_read_u32(np, "polling_ms",
			     &data->devfreq_profile.polling_ms);

	if (data->gov_type == SIMPLE_INTERACTIVE) {
		if (of_property_read_string(np, "use_delay_time",
					    &use_delay_time))
			return -ENODEV;

		if (!strcmp(use_delay_time, "true")) {
			data->simple_interactive_data.use_delay_time = true;
		} else if (!strcmp(use_delay_time, "false")) {
			data->simple_interactive_data.use_delay_time = false;
		} else {
			dev_err(data->dev, "invalid use_delay_time : (%s)\n",
				use_delay_time);
			return -EINVAL;
		}

		if (data->simple_interactive_data.use_delay_time) {
			if (of_property_read_string(np, "delay_time_list",
						    &buf)) {
				/*
				 * If there is not delay time list,
				 * delay time will be filled with default time
				 */
				data->simple_interactive_data.delay_time =
					kmalloc(sizeof(unsigned int),
						GFP_KERNEL);
				if (!data->simple_interactive_data.delay_time) {
					dev_err(data->dev,
						"Fail to allocate delay_time memory\n");
					return -ENOMEM;
				}
				*data->simple_interactive_data.delay_time =
					DEFAULT_DELAY_TIME;
				data->simple_interactive_data.ndelay_time =
					DEFAULT_NDELAY_TIME;
				dev_dbg(data->dev,
					 "set default delay time %d ms\n",
					 DEFAULT_DELAY_TIME);
			} else {
				data->simple_interactive_data.delay_time =
					get_tokenized_data(buf, &ntokens);
				if (!(ntokens & 0x1)) {
					dev_err(data->dev,
						"Only odd number of tokens allowed\n");
					return -EINVAL;
				}
				if (IS_ERR(data->simple_interactive_data.delay_time))
					return PTR_ERR(
						data->simple_interactive_data.delay_time);
				data->simple_interactive_data.ndelay_time =
					ntokens;
			}
		}

		/* Polling ms must be registered */
		/*	 */
		if (data->use_get_dev) {
			/* Register um_data and um_list for tracing load */
			int i;

			data->um_data.um_group =
				of_property_count_elems_of_size(np, "um_count",
								sizeof(u32));
			if (data->um_data.um_group <= 0)
				return -ENODEV;
			data->um_data.um_count =
				devm_kcalloc(data->dev, data->um_data.um_group,
					     sizeof(u32), GFP_KERNEL);
			if (!data->um_data.um_count) {
				dev_err(data->dev,
					"Failed to allocate memory for um_count\n");
				return -ENOMEM;
			}
			if (of_property_read_u32_array(np, "um_count",
						       (u32 *)data->um_data.um_count,
						       (size_t)data->um_data.um_group))
				return -ENODEV;
			data->um_data.ppc_val =
				devm_kcalloc(data->dev, data->um_data.um_group,
					     sizeof(struct ppc_data),
					     GFP_KERNEL);
			if (data->um_data.ppc_val == NULL) {
				dev_err(data->dev,
					"failed to allocate memory for PPC val\n");
				return -ENOMEM;
			}
			for (i = 0; i < data->um_data.um_group; i++)
				count_total += data->um_data.um_count[i];
			data->um_data.um_count_total = count_total;
			data->um_data.pa_base =
				kcalloc(data->um_data.um_count_total, sizeof(u32),
					GFP_KERNEL);

			if (!data->um_data.pa_base) {
				dev_err(data->dev,
					"failed to allocate memory for PA base\n");
				return -ENOMEM;
			}

			if (of_property_read_u32_array(np, "um_list",
						       (u32 *)data->um_data.pa_base,
						       (size_t)(data->um_data.um_count_total)))
				return -ENODEV;

			data->um_data.va_base = kcalloc(data->um_data.um_count_total,
							sizeof(void __iomem *),
							GFP_KERNEL);

			if (!data->um_data.va_base) {
				dev_err(data->dev,
					"failed to allocate memory for va base");
				kfree(data->um_data.pa_base);
				return -ENOMEM;
			}

			for (i = 0; i < data->um_data.um_count_total; i++) {
				data->um_data.va_base[i] = ioremap(data->um_data.pa_base[i],
								   SZ_4K);
			}
		} else {
			dev_info(data->dev, "Not registered UM monitor data\n");
		}

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
		/* Parse ALT-DVFS related parameters */
		if (of_property_read_bool(np, "use_alt_dvfs")) {
			int i;
			alt_data = &data->simple_interactive_data.alt_data;

			alt_data->num_target_load =
				of_property_count_elems_of_size(np, "target_load",
								sizeof(u32));
			if (alt_data->num_target_load > 0) {
				alt_data->target_load =
					kmalloc_array(alt_data->num_target_load,
						      sizeof(u32), GFP_KERNEL);
				if (!alt_data->target_load) {
					dev_err(data->dev,
						"Failed to allocate memory for target_load\n");
					return -ENOMEM;
				}
				if (of_property_read_u32_array(np, "target_load",
							(u32 *)alt_data->target_load,
							(size_t)alt_data->num_target_load))
					return -ENODEV;
			} else {
				/* Fix target load as defined ALTDVFS_TARGET_LOAD */
				alt_data->target_load = kmalloc(sizeof(unsigned int), GFP_KERNEL);
				if (!alt_data->target_load)
					return -ENOMEM;

				*alt_data->target_load = ALTDVFS_TARGET_LOAD;
				alt_data->num_target_load =
					ALTDVFS_NUM_TARGET_LOAD;
			}

			if (of_property_read_u32(np, "min_sample_time",
						 &alt_data->min_sample_time))
				alt_data->min_sample_time =
					ALTDVFS_MIN_SAMPLE_TIME;
			if (of_property_read_u32(np, "hold_sample_time",
						 &alt_data->hold_sample_time))
				alt_data->hold_sample_time =
					ALTDVFS_HOLD_SAMPLE_TIME;
			if (of_property_read_u32(np, "hispeed_load",
						 &alt_data->hispeed_load))
				alt_data->hispeed_load = ALTDVFS_HISPEED_LOAD;
			if (of_property_read_u32(np, "hispeed_freq",
						 &alt_data->hispeed_freq))
				alt_data->hispeed_freq = ALTDVFS_HISPEED_FREQ;
			if (of_property_read_u32(np, "tolerance",
						 &alt_data->tolerance))
				alt_data->tolerance = ALTDVFS_TOLERANCE;

			/* Initial buffer and load setup */
			alt_data->track_group = data->um_data.um_group;
			alt_data->track = devm_kcalloc(data->dev,
						alt_data->track_group,
						sizeof(struct devfreq_alt_track),
						GFP_KERNEL);
			if (!alt_data->track) {
				dev_err(data->dev,
					"Failed to allocate memory\n");
				return -ENOMEM;
			}
			for (i = 0; i < alt_data->track_group; i++) {
				alt_data->track[i].front = alt_data->track[i].buffer;
				alt_data->track[i].rear = alt_data->track[i].buffer;
				alt_data->track[i].min_load = 100;
			}

			/* Initial governor freq setup */
			data->simple_interactive_data.governor_freq = 0;

			map_cnt = of_property_count_elems_of_size(np, "mif_int_map",
								  sizeof(u32));
			if (map_cnt <= 0 || map_cnt % NUM_COLS) {
				dev_info(data->dev,
					 "No valid mif_int_map available\n");
				goto err_map;
			}
			of_data_int_array = kcalloc(map_cnt, sizeof(u32), GFP_KERNEL);
			if (!of_data_int_array) {
				dev_err(data->dev,
					"Failed to allocate memory\n");
				return -ENOMEM;
			}
			if (of_property_read_u32_array(np, "mif_int_map",
						       of_data_int_array,
						       (size_t)map_cnt)) {
				kfree(of_data_int_array);
				return -ENODEV;
			}
			alt_data->map_row_cnt = map_cnt / NUM_COLS;
			alt_data->mif_int_tbl =
				devm_kcalloc(data->dev, alt_data->map_row_cnt,
					     sizeof(*alt_data->mif_int_tbl), GFP_KERNEL);
			if (!alt_data->mif_int_tbl) {
				dev_err(data->dev,
					"Failed to allocate memory for mif_int_tbl\n");
				kfree(of_data_int_array);
				return -ENOMEM;
			}
			for (i = 0; i < alt_data->map_row_cnt; i++) {
				alt_data->mif_int_tbl[i].mif_freq =
					of_data_int_array[i * NUM_COLS];
				alt_data->mif_int_tbl[i].int_freq =
					of_data_int_array[i * NUM_COLS + 1];
			}
err_map:
			kfree(of_data_int_array);
		} else {
			dev_info(data->dev,
				 "ALT-DVFS is not declared by device tree.\n");
		}
#endif
	} else {
		dev_err(data->dev, "not support governor type %u\n",
			data->gov_type);
		return -EINVAL;
	}

#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	if (of_property_read_u32(np, "dm-index", &data->dm_type)) {
		dev_err(data->dev, "not support dvfs manager\n");
		return -ENODEV;
	}
#endif
	return 0;
}
#else
static int exynos_devfreq_parse_dt(struct device_node *np,
				   struct exynos_devfrq_data *data)
{
	return -EINVAL;
}
#endif

s32 exynos_devfreq_get_opp_idx(struct exynos_devfreq_opp_table *table,
			       unsigned int size, u32 freq)
{
	int i;

	for (i = 0; i < size; ++i) {
		if (table[i].freq == freq)
			return i;
	}

	return -ENODEV;
}

/*
 * Round freq to the opp with closest but lower or equal frequency
 * If none are found, the lowest opp freq is given
 */
static u32 exynos_devfreq_opp_round_freq(
				struct exynos_devfreq_opp_table *table,
				unsigned int size, u32 freq)
{
	int i;

	for (i = 0; i < size; ++i) {
		if (table[i].freq > freq)
			continue;
		return table[i].freq;
	}
	return table[size - 1].freq;
}

static int exynos_init_freq_table(struct exynos_devfreq_data *data)
{
	int i, ret;
	u32 freq, volt;

	for (i = 0; i < data->max_state; i++) {
		freq = data->opp_list[i].freq;
		volt = data->opp_list[i].volt;

		data->devfreq_profile.freq_table[i] = freq;

		ret = dev_pm_opp_add(data->dev, freq, volt);
		if (ret) {
			dev_err(data->dev, "failed to add opp entries %uKhz\n",
				freq);
			return ret;
		}

		dev_dbg(data->dev, "DEVFREQ : %8uKhz, %8uuV\n", freq,
			 volt);
	}

	ret = exynos_devfreq_init_freq_table(data);
	if (ret) {
		dev_err(data->dev, "failed init frequency table\n");
		return ret;
	}

	return 0;
}

static int exynos_devfreq_reboot_notifier(struct notifier_block *nb,
					  unsigned long val, void *v)
{
	struct exynos_devfreq_data *data =
		container_of(nb, struct exynos_devfreq_data, reboot_notifier);

	if (exynos_pm_qos_request_active(&data->default_pm_qos_min))
		exynos_pm_qos_update_request(&data->default_pm_qos_min,
					     data->reboot_freq);

	if (exynos_devfreq_reboot(data)) {
		dev_err(data->dev, "failed reboot\n");
		return NOTIFY_BAD;
	}

	return NOTIFY_OK;
}

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
static int exynos_devfreq_notifier(struct notifier_block *nb, unsigned long val,
				   void *v)
{
	struct devfreq_notifier_block *um_nb =
		container_of(nb, struct devfreq_notifier_block, nb);
	int err;

	mutex_lock(&um_nb->df->lock);
	err = update_devfreq(um_nb->df);
	if (err && err != -EAGAIN) {
		dev_err(&um_nb->df->dev, "devfreq failed with (%d) error\n",
			err);
		mutex_unlock(&um_nb->df->lock);
		return NOTIFY_BAD;
	}
	mutex_unlock(&um_nb->df->lock);

	return NOTIFY_OK;
}
#endif

static int exynos_devfreq_target(struct device *dev, unsigned long *target_freq,
				 u32 flags)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	struct dev_pm_opp *target_opp;
	u64 before_target, after_target, before_setfreq, after_setfreq;
	u32 target_volt;
	s32 target_idx;
	s32 target_time = 0;
	int ret = 0;
	s32 soft_max_freq;

	if (data->devfreq_disabled)
		return -EAGAIN;

	before_target = sched_clock();

	mutex_lock(&data->lock);

	target_opp = devfreq_recommended_opp(dev, target_freq, flags);
	if (IS_ERR(target_opp)) {
		dev_err(dev, "not found valid OPP table\n");
		ret = PTR_ERR(target_opp);
		goto out;
	}

	*target_freq = dev_pm_opp_get_freq(target_opp);
	target_volt = (u32)dev_pm_opp_get_voltage(target_opp);
	dev_pm_opp_put(target_opp);

	soft_max_freq = exynos_pm_qos_read_req_value(data->pm_qos_class_max,
						     &data->pm_qos_soft_max_freq);
	if (*target_freq > soft_max_freq) {
		ret = -EINVAL;
		goto out;
	}

	target_idx = exynos_devfreq_get_opp_idx(data->opp_list, data->max_state,
						*target_freq);
	if (target_idx < 0) {
		ret = -EINVAL;
		goto out;
	}

	data->new_freq = (u32)(*target_freq);
	data->new_idx = target_idx;
	data->new_volt = target_volt;

	if (data->old_freq == data->new_freq)
		goto out;

	dev_dbg(dev, "LV_%d, %uKhz, %uuV ======> LV_%d, %uKhz, %uuV\n",
		data->old_idx, data->old_freq, data->old_volt, data->new_idx,
		data->new_freq, data->new_volt);
#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
	dbg_snapshot_freq(data->ess_flag, data->old_freq, data->new_freq,
			  DSS_FLAG_IN);
#endif
	before_setfreq = sched_clock();

	ret = exynos_devfreq_set_freq(dev, data->new_freq, data->clk, data);
	if (ret) {
		dev_err(dev, "failed set frequency (%uKhz --> %uKhz)\n",
			data->old_freq, data->new_freq);
		goto out;
	}

	after_setfreq = sched_clock();
#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
	dbg_snapshot_freq(data->ess_flag, data->old_freq, data->new_freq,
			  DSS_FLAG_OUT);
#endif

	data->old_freq = data->new_freq;
	data->old_idx = data->new_idx;
	data->old_volt = data->new_volt;

	if (data->devfreq->profile->freq_table)
		if (exynos_devfreq_update_status(data))
			dev_err(dev,
				"Couldn't update frequency transition information.\n");

	data->previous_freq = data->new_freq;
out:
	mutex_unlock(&data->lock);

	after_target = sched_clock();

	target_time = after_target - before_target;

	data->target_delay = target_time;

	dev_dbg(dev, "target time: %d usec\n", target_time);

	return ret;
}

static int exynos_devfreq_suspend(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	int ret = 0;
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
#if IS_ENABLED(CONFIG_GS_ACPM)
	int size, ch_num;
	unsigned int cmd[4];
	struct ipc_config config;
#endif
#endif
	u32 get_freq = 0;

#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	if (data->use_acpm) {
		mutex_lock(&data->devfreq->lock);
		//send flag
#if IS_ENABLED(CONFIG_GS_ACPM)
		ret = acpm_ipc_request_channel(dev->of_node, NULL, &ch_num,
					       &size);
		if (ret) {
			dev_err(dev,
				"acpm request channel is failed, id:%u, size:%u\n",
				ch_num, size);
			mutex_unlock(&data->devfreq->lock);
			return -EINVAL;
		}
		/* Initial value of release flag is true.
		 * "true" means state of AP is running
		 * "false means state of AP is sleep.
		 */
		config.cmd = cmd;
		config.response = true;
		config.cmd[0] = data->devfreq_type;
		config.cmd[1] = false;
		config.cmd[2] = DATA_INIT;
		config.cmd[3] = RELEASE;

		ret = acpm_ipc_send_data(ch_num, &config);
		if (ret) {
			dev_err(dev,
				"failed to send release information to FVP");
			mutex_unlock(&data->devfreq->lock);
			return -EINVAL;
		}
#endif
		data->suspend_flag = true;
		ret = update_devfreq(data->devfreq);
		if (ret && ret != -EAGAIN) {
			dev_err(&data->devfreq->dev,
				"devfreq failed with (%d) error\n", ret);
			mutex_unlock(&data->devfreq->lock);
			return NOTIFY_BAD;
		}
		mutex_unlock(&data->devfreq->lock);
	}
#endif
	if (!data->use_acpm && exynos_pm_qos_request_active(&data->default_pm_qos_min))
		exynos_pm_qos_update_request(&data->default_pm_qos_min,
					     data->suspend_freq);

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	exynos_devfreq_um_exit(data);
#endif

	dev->power.must_resume = true;
	dev_dbg(data->dev, "Suspend frequency is %u\n",
		exynos_devfreq_get_freq(data->dev, &get_freq, data->clk, data) ?
		0 : get_freq);

	return ret;
}

static int exynos_devfreq_resume(struct device *dev)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
	struct devfreq_simple_interactive_data *gov_data = data->devfreq->data;
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
#if IS_ENABLED(CONFIG_GS_ACPM)
	int size, ch_num;
	unsigned int cmd[4];
	struct ipc_config config;
#endif
#endif
	int ret = 0;
	u32 cur_freq;

	dev_dbg(data->dev, "Resume_frequency is %u\n",
		exynos_devfreq_get_freq(data->dev, &cur_freq, data->clk, data) ?
		0 : cur_freq);

#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	if (data->use_acpm) {
		mutex_lock(&data->devfreq->lock);
		//send flag
#if IS_ENABLED(CONFIG_GS_ACPM)
		ret = acpm_ipc_request_channel(dev->of_node, NULL, &ch_num,
					       &size);
		if (ret) {
			dev_err(dev,
				"acpm request channel is failed, id:%u, size:%u\n",
				ch_num, size);
			mutex_unlock(&data->devfreq->lock);
			return -EINVAL;
		}

		config.cmd = cmd;
		config.response = true;
		config.cmd[0] = data->devfreq_type;
		config.cmd[1] = true;
		config.cmd[2] = DATA_INIT;
		config.cmd[3] = RELEASE;

		ret = acpm_ipc_send_data(ch_num, &config);
		if (ret) {
			dev_err(dev,
				"failed to send release information to FVP");
			mutex_unlock(&data->devfreq->lock);
			return -EINVAL;
		}
#endif
		data->suspend_flag = false;
		gov_data->df_update_enable = true;
		ret = update_devfreq(data->devfreq);
		gov_data->df_update_enable = false;
		if (ret && ret != -EAGAIN) {
			dev_err(&data->devfreq->dev,
				"devfreq failed with (%d) error\n", ret);
			mutex_unlock(&data->devfreq->lock);
			return NOTIFY_BAD;
		}
		mutex_unlock(&data->devfreq->lock);
	}
#endif
	if (!data->use_acpm && exynos_pm_qos_request_active(&data->default_pm_qos_min))
		exynos_pm_qos_update_request(&data->default_pm_qos_min,
					     data->default_qos);

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	ret = exynos_devfreq_um_init(data);
	if (ret)
		dev_err(data->dev, "failed to restart um\n");
#endif

	return ret;
}

ssize_t
user_vote_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct exynos_devfreq_data *devfreq_cdev = cdev->devdata;

	if (!devfreq_cdev)
		return -ENODEV;

	return sprintf(buf, "%lu\n", devfreq_cdev->sysfs_req);
}

ssize_t user_vote_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct exynos_devfreq_data *devfreq_cdev = cdev->devdata;
	int ret;
	unsigned long state;

	if (!devfreq_cdev)
		return -ENODEV;

	ret = kstrtoul(buf, 0, &state);
	if (ret)
		return ret;

	if (state > devfreq_cdev->max_state)
		return -EINVAL;

	mutex_lock(&cdev->lock);
	devfreq_cdev->sysfs_req = state;
	cdev->updated = false;
	mutex_unlock(&cdev->lock);
	thermal_cdev_update(cdev);
	return count;
}

static DEVICE_ATTR_RW(user_vote);

static int exynos_devfreq_get_max_state(struct thermal_cooling_device *cdev,
			     unsigned long *state)
{
	struct exynos_devfreq_data *data = cdev->devdata;
	*state = data->max_state;

	return 0;
}

static int exynos_devfreq_get_cur_state(struct thermal_cooling_device *cdev,
			     unsigned long *state)
{
	struct exynos_devfreq_data *data = cdev->devdata;
	*state = data->cooling_state;

	return 0;
}

static int exynos_devfreq_set_cur_state(struct thermal_cooling_device *cdev,
			     unsigned long state)
{
	struct exynos_devfreq_data *data = cdev->devdata;

	/* Request state should be less than max_level */
	if (WARN_ON(state > data->max_state))
		return -EINVAL;

	state = max(data->sysfs_req, state);
	/* Check if the old cooling action is same as new cooling action */
	if (data->cooling_state == state)
		return -EALREADY;

	data->cooling_state = state;

	dev_info(data->dev, "Set %s cur_state %lu, freq: %8luKhz\n", cdev->type,
			 state, data->devfreq_profile.freq_table[state]);
	if (exynos_pm_qos_request_active(&data->thermal_pm_qos_max))
		exynos_pm_qos_update_request(&data->thermal_pm_qos_max,
					     data->devfreq_profile.freq_table[state]);

	return 0;
}

static struct thermal_cooling_device_ops exynos_devfreq_cooling_ops = {
	.get_max_state = exynos_devfreq_get_max_state,
	.get_cur_state = exynos_devfreq_get_cur_state,
	.set_cur_state = exynos_devfreq_set_cur_state,
};

static int exynos_devfreq_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct exynos_devfreq_data *data;
	struct dev_pm_opp *init_opp;
	unsigned long init_freq = 0;
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	int nr_constraint;
	int dm_type;
	int err;
#endif
#if IS_ENABLED(CONFIG_ECT)
	const char *devfreq_domain_name;
#endif

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err_data;
	}

	data->dev = &pdev->dev;

	mutex_init(&data->lock);

	/* parsing devfreq dts data for exynos */
	ret = exynos_devfreq_parse_dt(data->dev->of_node, data);
	if (ret) {
		dev_err(data->dev, "failed to parse private data\n");
		goto err_parse_dt;
	}

	data->devfreq_profile.max_state = data->max_state;
	data->devfreq_profile.target = exynos_devfreq_dm_call;
	data->devfreq_profile.get_cur_freq = exynos_devfreq_get_cur_freq;
	if (data->gov_type == SIMPLE_INTERACTIVE) {
		data->simple_interactive_data.pm_qos_class = data->pm_qos_class;
		data->simple_interactive_data.pm_qos_class_max =
			data->pm_qos_class_max;
		data->governor_data = &data->simple_interactive_data;
	}

	data->devfreq_profile.freq_table = kcalloc(data->max_state,
						   sizeof(*data->devfreq_profile.freq_table),
						   GFP_KERNEL);
	if (!data->devfreq_profile.freq_table) {
		dev_err(data->dev, "failed to allocate for freq_table\n");
		ret = -ENOMEM;
		goto err_freqtable;
	}

	ret = exynos_init_freq_table(data);
	if (ret) {
		dev_err(data->dev, "failed initailize freq_table\n");
		goto err_init_table;
	}

	devfreq_data[data->devfreq_type] = data;
	platform_set_drvdata(pdev, data);

	data->old_freq = (u32)data->devfreq_profile.initial_freq;
	data->previous_freq = data->old_freq;
	data->last_stat_updated = jiffies;
	data->old_idx = exynos_devfreq_get_opp_idx(data->opp_list,
						   data->max_state, data->old_freq);
	if (data->old_idx < 0) {
		ret = -EINVAL;
		goto err_old_idx;
	}

	init_freq = (unsigned long)data->old_freq;
	init_opp = devfreq_recommended_opp(data->dev, &init_freq, 0);
	if (IS_ERR(init_opp)) {
		dev_err(data->dev, "not found valid OPP table for sync\n");
		ret = PTR_ERR(init_opp);
		goto err_get_opp;
	}
	data->new_volt = (u32)dev_pm_opp_get_voltage(init_opp);
	dev_pm_opp_put(init_opp);

	dev_dbg(data->dev, "Initial Frequency: %ld, Initial Voltage: %d\n",
		 init_freq, data->new_volt);

	data->old_volt = data->new_volt;
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	ret = exynos_dm_data_init(data->dm_type, data, data->min_freq,
				  data->max_freq, data->old_freq);
	if (ret) {
		dev_err(data->dev, "failed DVFS Manager data init\n");
		goto err_dm_data_init;
	}

	for (nr_constraint = 0; nr_constraint < data->nr_constraint;
	     nr_constraint++) {
		if (data->constraint[nr_constraint]) {
			ret = register_exynos_dm_constraint_table(data->dm_type,
								  data->constraint[nr_constraint]);
			if (ret) {
				dev_err(data->dev,
					"failed registration constraint table(%d)\n",
					nr_constraint);
				goto err_dm_table;
			}
		}
	}
#endif
	/* This flag guarantees initial frequency during boot time */
	data->devfreq_disabled = true;

	data->devfreq =
		devfreq_add_device(data->dev, &data->devfreq_profile,
				   data->governor_name, data->governor_data);
	if (IS_ERR(data->devfreq)) {
		dev_err(data->dev, "failed devfreq device added\n");
		ret = -EINVAL;
		goto err_devfreq;
	}

	/* Register device tracing for ALT-DVFS */
	if (data->use_get_dev)
		register_get_dev_status(data);

	data->time_in_state = devm_kcalloc(data->dev,
					   data->devfreq->profile->max_state,
					   sizeof(unsigned long),
					   GFP_KERNEL);
	if (!data->time_in_state) {
		err = -ENOMEM;
		goto err_devfreq;
	}

	lockdep_register_key(&data->devfreq_lock_key);
	lockdep_set_class(&data->devfreq->lock, &data->devfreq_lock_key);

#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	err = find_exynos_devfreq_dm_type(data->dev, &dm_type);
	if (err)
		goto err_dm_type;

	err = register_exynos_dm_freq_scaler(dm_type, devfreq_frequency_scaler);
	if (err)
		goto err_dm_scaler;
#endif

	if (data->min_freq) {
		err = dev_pm_qos_update_request(
			&data->devfreq->user_min_freq_req, data->min_freq / HZ_PER_KHZ);
		if (err < 0)
			goto err_dm_scaler;
	}
	if (data->max_freq) {
		err = dev_pm_qos_update_request(
			&data->devfreq->user_max_freq_req, data->max_freq / HZ_PER_KHZ);
		if (err < 0)
			goto err_dm_scaler;
	}

	exynos_pm_qos_add_request(&data->sys_pm_qos_min, (int)data->pm_qos_class,
				  data->min_freq);
	exynos_pm_qos_add_request(&data->debug_pm_qos_min, (int)data->pm_qos_class,
				  data->min_freq);
	exynos_pm_qos_add_request(&data->debug_pm_qos_max, (int)data->pm_qos_class_max,
				  data->max_freq);
	exynos_pm_qos_add_request(&data->pm_qos_soft_max_freq, (int)data->pm_qos_class_max,
				  data->soft_max_freq);
	exynos_pm_qos_add_request(&data->thermal_pm_qos_max, (int)data->pm_qos_class_max,
				  data->max_freq);
	if (data->pm_qos_class_max)
		exynos_pm_qos_add_request(&data->default_pm_qos_max,
					  (int)data->pm_qos_class_max, data->max_freq);
	exynos_pm_qos_add_request(&data->default_pm_qos_min, (int)data->pm_qos_class,
				  data->default_qos);
	exynos_pm_qos_add_request(&data->boot_pm_qos, (int)data->pm_qos_class,
				  data->devfreq_profile.initial_freq);

	/* Initialize ALT-DVFS */
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	if (data->use_get_dev) {
		init_alt_notifier_list();

		/* if polling_ms is 0, update_devfreq function is called by um */
		if (data->devfreq_profile.polling_ms == 0) {
			data->um_nb =
				kzalloc(sizeof(struct devfreq_notifier_block),
					GFP_KERNEL);
			if (!data->um_nb) {
				dev_err(data->dev,
					"failed to allocate notifier block\n");
				ret = -ENOMEM;
				goto err_um_nb;
			}

			data->um_nb->df = data->devfreq;
			data->um_nb->nb.notifier_call = exynos_devfreq_notifier;
			exynos_alt_register_notifier(&data->um_nb->nb);
			data->last_monitor_time = sched_clock();
		}

		/*
		 * UM data should be register.
		 * And if polling_ms is 0, um notifier should be register in callback.
		 */
		ret = exynos_devfreq_um_init(data);
		if (ret) {
			dev_err(data->dev, "failed register um\n");
			goto err_um;
		}
	}
	exynos_pm_qos_add_request(&data->bus_pm_qos_min, PM_QOS_DEVICE_THROUGHPUT, 0);

#endif
	ret = devfreq_register_opp_notifier(data->dev, data->devfreq);
	if (ret) {
		dev_err(data->dev, "failed register opp notifier\n");
		goto err_opp_noti;
	}

	data->reboot_notifier.notifier_call = exynos_devfreq_reboot_notifier;
	ret = register_reboot_notifier(&data->reboot_notifier);
	if (ret) {
		dev_err(data->dev, "failed register reboot notifier\n");
		goto err_reboot_noti;
	}

	ret = sysfs_create_file(&data->devfreq->dev.kobj,
				&dev_attr_scaling_devfreq_min.attr);
	if (ret)
		dev_warn(data->dev,
			 "failed create sysfs for devfreq pm_qos_min\n");

	ret = sysfs_create_file(&data->devfreq->dev.kobj,
				&dev_attr_time_in_state.attr);
	if (ret)
		dev_warn(data->dev,
			 "failed create sysfs for devfreq time_in_state\n");

	ret = sysfs_create_file(&data->devfreq->dev.kobj,
				&dev_attr_ppc_read_reset_disable.attr);
	if (ret)
		dev_warn(data->dev,
			 "failed create sysfs for devfreq ppc_read_reset_disable\n");

	ret = sysfs_create_group(&data->devfreq->dev.kobj,
				 &exynos_devfreq_attr_group);
	if (ret)
		dev_warn(data->dev, "failed create sysfs for devfreq data\n");

	ret = sysfs_create_group(&data->devfreq->dev.kobj,
				 &devfreq_delay_time_attr_group);
	if (ret)
		dev_warn(data->dev, "failed create sysfs for devfreq data\n");

	data->devfreq_disabled = false;

	if (!data->pm_domain) {
		/* set booting frequency during booting time */
		exynos_pm_qos_update_request_timeout(&data->boot_pm_qos,
						     data->boot_freq,
						     data->boot_qos_timeout * USEC_PER_SEC);
	} else {
		pm_runtime_enable(&pdev->dev);
		pm_runtime_get_sync(&pdev->dev);
		exynos_pm_qos_update_request(&data->boot_pm_qos, data->default_qos);
		pm_runtime_put_sync(&pdev->dev);
	}

#if IS_ENABLED(CONFIG_ECT)
	if (of_property_read_string(data->dev->of_node, "devfreq_domain_name",
				    &devfreq_domain_name))
		return -ENODEV;

	// Register cooling device
	data->cooling_dev = devm_thermal_of_cooling_device_register(data->dev,
			dev_of_node(data->dev), (char *)devfreq_domain_name,
			data, &exynos_devfreq_cooling_ops);

	if (IS_ERR(data->cooling_dev)) {
		ret = PTR_ERR(data->cooling_dev);
		dev_err(data->dev, "%s: failed to register cooling device: %d\n",
			devfreq_domain_name, ret);
	} else {
		dev_info(data->dev, "%s cdev is initialized!!\n", devfreq_domain_name);
		ret = device_create_file(&data->cooling_dev->device, &dev_attr_user_vote);
		if (ret)
			thermal_cooling_device_unregister(data->cooling_dev);
	}
#endif
	dev_info(data->dev, "devfreq is initialized!!\n");

	return ret;

err_reboot_noti:
	devfreq_unregister_opp_notifier(data->dev, data->devfreq);
err_opp_noti:
	exynos_pm_qos_remove_request(&data->boot_pm_qos);
	exynos_pm_qos_remove_request(&data->default_pm_qos_min);
	if (data->pm_qos_class_max)
		exynos_pm_qos_remove_request(&data->default_pm_qos_max);
	exynos_pm_qos_remove_request(&data->debug_pm_qos_min);
	exynos_pm_qos_remove_request(&data->debug_pm_qos_max);
	exynos_pm_qos_remove_request(&data->thermal_pm_qos_max);
	exynos_pm_qos_remove_request(&data->sys_pm_qos_min);
	devfreq_remove_device(data->devfreq);
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	exynos_pm_qos_remove_request(&data->bus_pm_qos_min);
err_um:
	if (data->um_nb) {
		exynos_alt_unregister_notifier(&data->um_nb->nb);
		kfree(data->um_nb);
	}
err_um_nb:
#endif
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	unregister_exynos_dm_freq_scaler(dm_type);
err_dm_scaler:
err_dm_type:
#endif
	lockdep_unregister_key(&data->devfreq_lock_key);
err_devfreq:
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	for (; nr_constraint >= 0; nr_constraint--) {
		if (data->constraint[nr_constraint])
			unregister_exynos_dm_constraint_table(data->dm_type,
							      data->constraint[nr_constraint]);
	}
err_dm_table:
err_dm_data_init:
#endif
err_get_opp:
err_old_idx:
	platform_set_drvdata(pdev, NULL);
err_init_table:
	kfree(data->devfreq_profile.freq_table);
err_freqtable:
err_parse_dt:
	mutex_destroy(&data->lock);
	kfree(data);
err_data:

	return ret;
}

static int exynos_devfreq_remove(struct platform_device *pdev)
{
	struct exynos_devfreq_data *data = platform_get_drvdata(pdev);
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	int nr_constraint;
#endif
	sysfs_remove_file(&data->devfreq->dev.kobj,
			  &dev_attr_scaling_devfreq_min.attr);
	sysfs_remove_group(&data->devfreq->dev.kobj,
			   &exynos_devfreq_attr_group);
	sysfs_remove_group(&data->devfreq->dev.kobj,
			   &devfreq_delay_time_attr_group);

	unregister_reboot_notifier(&data->reboot_notifier);
	devfreq_unregister_opp_notifier(data->dev, data->devfreq);

	exynos_pm_qos_remove_request(&data->boot_pm_qos);
	exynos_pm_qos_remove_request(&data->default_pm_qos_min);
	if (data->pm_qos_class_max)
		exynos_pm_qos_remove_request(&data->default_pm_qos_max);
	exynos_pm_qos_remove_request(&data->debug_pm_qos_min);
	exynos_pm_qos_remove_request(&data->debug_pm_qos_max);
	exynos_pm_qos_remove_request(&data->thermal_pm_qos_max);
	exynos_pm_qos_remove_request(&data->sys_pm_qos_min);
#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
	exynos_alt_unregister_notifier(&data->um_nb->nb);
	exynos_devfreq_um_exit(data);
#endif
	lockdep_unregister_key(&data->devfreq_lock_key);
	devfreq_remove_device(data->devfreq);
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
	for (nr_constraint = 0; nr_constraint < data->nr_constraint;
	     nr_constraint++) {
		if (data->constraint[nr_constraint])
			unregister_exynos_dm_constraint_table(data->dm_type,
							      data->constraint[nr_constraint]);
	}
#endif
	platform_set_drvdata(pdev, NULL);
	kfree(data->devfreq_profile.freq_table);
	mutex_destroy(&data->lock);
	kfree(data);

	return 0;
}

static struct platform_device_id exynos_devfreq_driver_ids[] = {
	{
		.name = EXYNOS_DEVFREQ_MODULE_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(platform, exynos_devfreq_driver_ids);

static const struct of_device_id exynos_devfreq_match[] = {
	{
		.compatible = "samsung,exynos-devfreq",
	},
	{},
};

MODULE_DEVICE_TABLE(of, exynos_devfreq_match);

static const struct dev_pm_ops exynos_devfreq_pm_ops = {
	.suspend_late = exynos_devfreq_suspend,
	.resume_early = exynos_devfreq_resume,
};

static struct platform_driver exynos_devfreq_driver = {
	.probe = exynos_devfreq_probe,
	.remove = exynos_devfreq_remove,
	.id_table = exynos_devfreq_driver_ids,
	.driver = {
		.name = EXYNOS_DEVFREQ_MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &exynos_devfreq_pm_ops,
		.of_match_table = exynos_devfreq_match,
	},
};

static int exynos_devfreq_root_probe(struct platform_device *pdev)
{
	struct device_node *np;
	int num_domains;

	devfreq_simple_interactive_init();

	np = pdev->dev.of_node;

	platform_driver_register(&exynos_devfreq_driver);

	/* alloc memory for devfreq data structure */
	num_domains = of_get_child_count(np);
	devfreq_data = kcalloc(num_domains,
			       sizeof(struct exynos_devfreq_data *), GFP_KERNEL);

	/* probe each devfreq node */
	of_platform_populate(np, NULL, NULL, NULL);

	return 0;
}

static const struct of_device_id exynos_devfreq_root_match[] = {
	{
		.compatible = "samsung,exynos-devfreq-root",
	},
	{},
};

static struct platform_driver exynos_devfreq_root_driver = {
	.probe = exynos_devfreq_root_probe,
	.driver = {
		.name = "exynos-devfreq-root",
		.owner = THIS_MODULE,
		.of_match_table = exynos_devfreq_root_match,
	},
};

module_platform_driver(exynos_devfreq_root_driver);
MODULE_AUTHOR("Taekki Kim <taekki.kim@samsung.com>");
MODULE_DESCRIPTION("Samsung EXYNOS Soc series devfreq common driver");
MODULE_LICENSE("GPL");
