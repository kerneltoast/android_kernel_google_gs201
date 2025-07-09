// SPDX-License-Identifier: GPL-2.0
/*
 * cdev_uclamp.c Cooling device to place thermal uclamp vote.
 *
 * Copyright (c) 2024, Google LLC. All rights reserved.
 *
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

#include "thermal_core.h"
#include "../../soc/google/vh/include/sched.h"

struct thermal_uclamp_cdev {
	unsigned int cpu;
	unsigned int cur_state;
	unsigned int max_state;
	unsigned int related_cpu_cnt;
	struct thermal_cooling_device *cdev;
	struct em_perf_domain *em;
	struct list_head cdev_list;
};

static LIST_HEAD(therm_uclamp_cdev_list);
static DEFINE_MUTEX(therm_cdev_list_lock);

static int thermal_uclamp_get_max_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct thermal_uclamp_cdev *uclamp_cdev = cdev->devdata;

	*state = uclamp_cdev->max_state;

	return 0;
}

static int thermal_uclamp_get_cur_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct thermal_uclamp_cdev *uclamp_cdev = cdev->devdata;

	*state = uclamp_cdev->cur_state;

	return 0;
}

static int thermal_uclamp_set_cur_state(struct thermal_cooling_device *cdev,
					unsigned long state)
{
	struct thermal_uclamp_cdev *uclamp_cdev = cdev->devdata;
	int idx = 0;

	if (state > uclamp_cdev->max_state)
		return -EINVAL;

	mutex_lock(&therm_cdev_list_lock);
	if (state != uclamp_cdev->cur_state) {
		idx = uclamp_cdev->max_state - state;
		pr_debug("cdev:[%s] new state request:[%lu] frequeny:[%lu]\n",
			 cdev->type, state,
			 uclamp_cdev->em->table[idx].frequency);
		uclamp_cdev->cur_state = state;
		sched_thermal_freq_cap(uclamp_cdev->cpu,
				       uclamp_cdev->em->table[idx].frequency);
	}
	mutex_unlock(&therm_cdev_list_lock);

	return 0;
}

static unsigned long thermal_uclamp_cpupower_to_state(struct thermal_uclamp_cdev *uclamp_cdev,
				    u32 power)
{
	unsigned long idx = 0;

	for (idx = 0; idx < uclamp_cdev->max_state; idx++) {
		if (uclamp_cdev->em->table[idx].power >= power)
			break;
	}

	return idx;
}

static unsigned long thermal_uclamp_cpufreq_to_state(struct thermal_uclamp_cdev *uclamp_cdev,
					   unsigned long freq)
{
	unsigned long idx = 0;

	for (idx = 0; idx < uclamp_cdev->max_state; idx++) {
		if (uclamp_cdev->em->table[idx].frequency >= freq)
			break;
	}

	return idx;
}

static int thermal_uclamp_state2power(struct thermal_cooling_device *cdev,
				      unsigned long state, u32 *power)
{
	struct thermal_uclamp_cdev *uclamp_cdev = cdev->devdata;
	unsigned int idx = 0;

	if (state > uclamp_cdev->max_state)
		return -EINVAL;

	idx = uclamp_cdev->max_state - state;
	*power = uclamp_cdev->em->table[idx].power * uclamp_cdev->related_cpu_cnt;

	return 0;
}

static int thermal_uclamp_get_requested_power(struct thermal_cooling_device *cdev,
					      u32 *power)
{
	struct thermal_uclamp_cdev *uclamp_cdev = cdev->devdata;
	unsigned long freq = 0, state = 0;
	unsigned int idx = 0;

	freq = cpufreq_quick_get(uclamp_cdev->cpu);
	state = thermal_uclamp_cpufreq_to_state(uclamp_cdev, freq);
	idx = uclamp_cdev->max_state - state;

	*power = uclamp_cdev->em->table[idx].power * uclamp_cdev->related_cpu_cnt;

	return 0;
}

static int thermal_uclamp_power2state(struct thermal_cooling_device *cdev,
				      u32 power, unsigned long *state)
{
	struct thermal_uclamp_cdev *uclamp_cdev = cdev->devdata;

	*state = thermal_uclamp_cpupower_to_state(uclamp_cdev, power);

	return 0;
}

static struct thermal_cooling_device_ops thermal_uclamp_cdev_ops = {
	.get_max_state = thermal_uclamp_get_max_state,
	.get_cur_state = thermal_uclamp_get_cur_state,
	.set_cur_state = thermal_uclamp_set_cur_state,
	.get_requested_power = thermal_uclamp_get_requested_power,
	.state2power = thermal_uclamp_state2power,
	.power2state = thermal_uclamp_power2state,
};

ssize_t
state2power_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev = to_cooling_device(dev);
	struct thermal_uclamp_cdev *uclamp_cdev = cdev->devdata;
	int i, count = 0;
	u32 power;

	if (!uclamp_cdev)
		return -ENODEV;

	for (i = 0; i <= uclamp_cdev->max_state; i++) {
		thermal_uclamp_state2power(cdev, i, &power);
		count += sysfs_emit_at(buf, count, "%u ", DIV_ROUND_UP(power, 1000));
	}
	count += sysfs_emit_at(buf, count, "\n");

	return count;
}

static DEVICE_ATTR_RO(state2power_table);

static void thermal_uclamp_cleanup(void)
{
	struct thermal_uclamp_cdev *uclamp_cdev = NULL, *n;

	mutex_lock(&therm_cdev_list_lock);
	list_for_each_entry_safe(uclamp_cdev, n, &therm_uclamp_cdev_list, cdev_list) {
		list_del(&uclamp_cdev->cdev_list);
		device_remove_file(&uclamp_cdev->cdev->device,
				   &dev_attr_state2power_table);
		if (uclamp_cdev->cdev) {
			pr_info("Un-registered cdev:%s\n",
				uclamp_cdev->cdev->type);
			thermal_cooling_device_unregister(uclamp_cdev->cdev);
			uclamp_cdev->cdev = NULL;
		}
		sched_thermal_freq_cap(
				uclamp_cdev->cpu,
				uclamp_cdev->em->table[uclamp_cdev->max_state].frequency);
		kfree(uclamp_cdev);
	}
	mutex_unlock(&therm_cdev_list_lock);
}

static int __init thermal_uclamp_init(void)
{
	int ret = 0;
	unsigned int cpu = 0;
	struct cpufreq_policy *policy = 0;
	struct thermal_uclamp_cdev *uclamp_cdev = NULL;
	char *name = NULL;

	if (!list_empty(&therm_uclamp_cdev_list)) {
		pr_err("Thermal uclamp cdev already initialized\n");
		return -EALREADY;
	}

	mutex_lock(&therm_cdev_list_lock);
	for (; cpu < num_possible_cpus(); )
	{
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
			pr_err("cpufreq policy %d fetch error. Trying again...\n", cpu);
			ret = -EPROBE_DEFER;
			goto cleanup_exit;
		}
		uclamp_cdev = kzalloc(sizeof(*uclamp_cdev), GFP_KERNEL);
		if (!uclamp_cdev) {
			ret = -ENOMEM;
			goto cleanup_exit;
		}
		uclamp_cdev->cpu = policy->cpu;
		uclamp_cdev->related_cpu_cnt = cpumask_weight(policy->related_cpus);
		cpu += uclamp_cdev->related_cpu_cnt;

		uclamp_cdev->max_state = cpufreq_table_count_valid_entries(policy) - 1;
		if (!uclamp_cdev->max_state) {
			pr_err("CPU:%d frequency table not available.\n", policy->cpu);
			ret = -EPROBE_DEFER;
			goto cleanup_exit;
		}

		uclamp_cdev->em = em_cpu_get(policy->cpu);
		if (!uclamp_cdev->em) {
			pr_err("CPU:%d energy model not available.\n", policy->cpu);
			goto cleanup_exit;
		}

		cpufreq_cpu_put(policy);
		policy = NULL;

		list_add_tail(&uclamp_cdev->cdev_list, &therm_uclamp_cdev_list);
		sched_thermal_freq_cap(
				uclamp_cdev->cpu,
				uclamp_cdev->em->table[uclamp_cdev->max_state].frequency);
		name = kasprintf(GFP_KERNEL, "thermal-uclamp-%d", uclamp_cdev->cpu);
		uclamp_cdev->cdev = thermal_cooling_device_register(name, uclamp_cdev,
								    &thermal_uclamp_cdev_ops);
		if (IS_ERR(uclamp_cdev->cdev)) {
			ret = PTR_ERR(uclamp_cdev->cdev);
			pr_err("uclamp cdev:[%s] register error. err:%d\n", name, ret);
			uclamp_cdev->cdev = NULL;
			goto cleanup_exit;
		}
		ret = device_create_file(&uclamp_cdev->cdev->device,
					 &dev_attr_state2power_table);
		if (ret) {
			pr_err("cdev:[%s] state2power attr failed. err:%d\n",
			       name, ret);
			goto cleanup_exit;
		}
		pr_info("Registered cdev:%s\n", name);
	}
	mutex_unlock(&therm_cdev_list_lock);

	return ret;

cleanup_exit:
	mutex_unlock(&therm_cdev_list_lock);
	if (policy) {
		cpufreq_cpu_put(policy);
		policy = NULL;
	}

	thermal_uclamp_cleanup();

	return ret;
}
module_init(thermal_uclamp_init);

static void __exit thermal_uclamp_exit(void)
{
	thermal_uclamp_cleanup();
}
module_exit(thermal_uclamp_exit);

MODULE_DESCRIPTION("Cooling device for placing uclamp max for CPU clusters");
MODULE_AUTHOR("Ram Chandrasekar <rchandrasekar@google.com>");
MODULE_LICENSE("GPL");
