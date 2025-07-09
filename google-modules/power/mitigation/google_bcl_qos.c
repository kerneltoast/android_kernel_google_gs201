// SPDX-License-Identifier: GPL-2.0
/*
 * google_bcl_qos.c Google bcl PMQOS driver
 *
 * Copyright (c) 2023, Google LLC. All rights reserved.
 *
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14)
#include <linux/mfd/samsung/s2mpg1415.h>
#include <linux/mfd/samsung/s2mpg1415-register.h>
#elif IS_ENABLED(CONFIG_REGULATOR_S2MPG12) || IS_ENABLED(CONFIG_REGULATOR_S2MPG10)
#include <linux/mfd/samsung/s2mpg1x.h>
#include <linux/mfd/samsung/s2mpg1x-register.h>
#endif
#include "bcl.h"

#define CREATE_TRACE_POINTS
#include <trace/events/bcl_exynos.h>


struct qos_data {
	struct list_head list;
	struct bcl_zone *zone;
};

static LIST_HEAD(qos_data_list);

static void trace_qos(bool throttle, const char *devname)
{
	char buf[64];
	if (!trace_clock_set_rate_enabled())
		return;
	snprintf(buf, sizeof(buf), "BCL_ZONE_%s_QOS", devname);
	trace_clock_set_rate(buf, throttle ? 1 : 0, raw_smp_processor_id());
}

static void add_qos_request(struct bcl_device *bcl_dev, struct bcl_zone *zone)
{
	struct qos_data *data = kmalloc(sizeof(struct qos_data), GFP_KERNEL);

	if (!data)
		return;

	data->zone = zone;
	if (mutex_lock_interruptible(&bcl_dev->qos_update_lock)) {
		kfree(data);
		return;
	}
	list_add_tail(&data->list, &qos_data_list);
	mutex_unlock(&bcl_dev->qos_update_lock);
}

static void process_qos_request(struct work_struct *work)
{
	struct bcl_device *bcl_dev = container_of(work, struct bcl_device, qos_work.work);
	struct list_head *n, *pos;

	list_for_each_safe(pos, n, &qos_data_list) {
		struct qos_data *data = list_entry(pos, struct qos_data, list);
		struct qos_throttle_limit *qos;

		qos = data->zone->bcl_qos;

		if (bcl_dev->cpu0_cluster_on)
			freq_qos_update_request(&qos->cpu0_max_qos_req, qos->cpu0_freq);
		if (bcl_dev->cpu1_cluster_on)
			freq_qos_update_request(&qos->cpu1_max_qos_req, qos->cpu1_freq);
		if (bcl_dev->cpu2_cluster_on)
			freq_qos_update_request(&qos->cpu2_max_qos_req, qos->cpu2_freq);

		if (exynos_pm_qos_request_active(&qos->tpu_qos_max))
			exynos_pm_qos_update_request_async(&qos->tpu_qos_max, qos->tpu_freq);
		if (exynos_pm_qos_request_active(&qos->gpu_qos_max))
			exynos_pm_qos_update_request_async(&qos->gpu_qos_max, qos->gpu_freq);
		usleep_range(TIMEOUT_10000US, TIMEOUT_10000US + 100);
		if (mutex_lock_interruptible(&bcl_dev->qos_update_lock))
			continue;
		list_del(&data->list);
		kfree(data);
		mutex_unlock(&bcl_dev->qos_update_lock);
	}

}

void google_bcl_qos_update(struct bcl_zone *zone, bool throttle)
{
	struct bcl_device *bcl_dev;
	int i;
	int cpu0_freq = INT_MAX, cpu1_freq = INT_MAX, cpu2_freq = INT_MAX;
	int tpu_freq = INT_MAX, gpu_freq = INT_MAX;

	if (!zone->bcl_qos)
		return;
	bcl_dev = zone->parent;

	if (zone->throttle && (zone->throttle == throttle))
		return;
	zone->throttle = throttle;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		if (bcl_dev->zone[i] && bcl_dev->zone[i]->bcl_qos && zone->throttle) {
			cpu0_freq = min(cpu0_freq, bcl_dev->zone[i]->bcl_qos->cpu0_limit);
			cpu1_freq = min(cpu1_freq, bcl_dev->zone[i]->bcl_qos->cpu1_limit);
			cpu2_freq = min(cpu2_freq, bcl_dev->zone[i]->bcl_qos->cpu2_limit);
			tpu_freq = min(tpu_freq, bcl_dev->zone[i]->bcl_qos->tpu_limit);
			gpu_freq = min(gpu_freq, bcl_dev->zone[i]->bcl_qos->gpu_limit);
		}
	}
	zone->bcl_qos->cpu0_freq = cpu0_freq;
	zone->bcl_qos->cpu1_freq = cpu1_freq;
	zone->bcl_qos->cpu2_freq = cpu2_freq;
	zone->bcl_qos->tpu_freq = tpu_freq;
	zone->bcl_qos->gpu_freq = gpu_freq;

	add_qos_request(bcl_dev, zone);

	if (smp_load_acquire(&bcl_dev->enabled))
		schedule_delayed_work(&bcl_dev->qos_work, 0);
	trace_bcl_irq_trigger(zone->idx, zone->throttle, cpu0_freq, cpu1_freq, cpu2_freq,
			      tpu_freq, gpu_freq, zone->bcl_stats.voltage,
			      zone->bcl_stats.capacity);
	trace_qos(zone->throttle, zone->devname);

}

static int init_freq_qos(struct bcl_device *bcl_dev, struct qos_throttle_limit *throttle)
{
	struct cpufreq_policy *policy = NULL;
	int ret;

	policy = cpufreq_cpu_get(bcl_dev->cpu0_cluster);
	if (!policy)
		return -EINVAL;

	bcl_dev->cpu0_cluster_on = true;
	ret = freq_qos_add_request(&policy->constraints, &throttle->cpu0_max_qos_req,
				   FREQ_QOS_MAX, INT_MAX);
	cpufreq_cpu_put(policy);
	if (ret < 0)
		return ret;

	policy = cpufreq_cpu_get(bcl_dev->cpu1_cluster);
	if (!policy)
		return ret;

	bcl_dev->cpu1_cluster_on = true;
	ret = freq_qos_add_request(&policy->constraints, &throttle->cpu1_max_qos_req,
				   FREQ_QOS_MAX, INT_MAX);
	cpufreq_cpu_put(policy);
	if (ret < 0)
		goto fail1;

	policy = cpufreq_cpu_get(bcl_dev->cpu2_cluster);
	if (!policy)
		return ret;

	bcl_dev->cpu2_cluster_on = true;
	ret = freq_qos_add_request(&policy->constraints, &throttle->cpu2_max_qos_req,
				   FREQ_QOS_MAX, INT_MAX);
	cpufreq_cpu_put(policy);
	if (ret < 0)
		goto fail2;

	return 0;
fail2:
	freq_qos_remove_request(&throttle->cpu1_max_qos_req);
fail1:
	freq_qos_remove_request(&throttle->cpu0_max_qos_req);
	return ret;
}

int google_bcl_setup_qos(struct bcl_device *bcl_dev)
{
	int ret = 0;
	int i;
	struct bcl_zone *zone;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		zone = bcl_dev->zone[i];
		if ((!zone) || (!zone->bcl_qos))
			continue;

		ret = init_freq_qos(bcl_dev, zone->bcl_qos);
		if (ret < 0) {
			dev_err(bcl_dev->device, "Cannot init pm qos on %d for cpu\n",
				zone->idx);
			goto fail;
		}
		exynos_pm_qos_add_request(&zone->bcl_qos->tpu_qos_max, PM_QOS_TPU_FREQ_MAX,
				  	  INT_MAX);
		exynos_pm_qos_add_request(&zone->bcl_qos->gpu_qos_max, PM_QOS_GPU_FREQ_MAX,
				  	  INT_MAX);
		zone->conf_qos = true;
		zone->throttle = false;
	}
	mutex_init(&bcl_dev->qos_update_lock);
	INIT_DELAYED_WORK(&bcl_dev->qos_work, process_qos_request);
	INIT_LIST_HEAD(&qos_data_list);
	return 0;
fail:
	google_bcl_remove_qos(bcl_dev);
	return ret;
}

void google_bcl_remove_qos(struct bcl_device *bcl_dev)
{
	int i;
	struct bcl_zone *zone;
	struct list_head *n, *pos;

	for (i = 0; i < TRIGGERED_SOURCE_MAX; i++) {
		zone = bcl_dev->zone[i];
		if ((!zone) || (!zone->bcl_qos))
			continue;
		if (zone->conf_qos) {
			if (bcl_dev->cpu0_cluster_on)
				freq_qos_remove_request(&zone->bcl_qos->cpu0_max_qos_req);
			if (bcl_dev->cpu1_cluster_on)
				freq_qos_remove_request(&zone->bcl_qos->cpu1_max_qos_req);
			if (bcl_dev->cpu2_cluster_on)
				freq_qos_remove_request(&zone->bcl_qos->cpu2_max_qos_req);
			exynos_pm_qos_remove_request(&zone->bcl_qos->tpu_qos_max);
			exynos_pm_qos_remove_request(&zone->bcl_qos->gpu_qos_max);
			zone->bcl_qos = NULL;
			zone->conf_qos = false;
		}
	}

	list_for_each_safe(pos, n, &qos_data_list) {
		struct qos_data *data = list_entry(pos, struct qos_data, list);

		list_del(&data->list);
		kfree(data);
	}
	if (bcl_dev->qos_work.work.func != NULL)
		cancel_delayed_work_sync(&bcl_dev->qos_work);

	mutex_destroy(&bcl_dev->qos_update_lock);

}
