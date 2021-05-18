// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019 Google LLC
 */

#include "eh_internal.h"
#include <linux/device.h>
#include <linux/platform_device.h>

static ssize_t queued_comp_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int size;
	int written = 0;

	spin_lock(&eh_dev->fifo_prod_lock);
	size = (eh_dev->write_index - eh_dev->complete_index) &
	       eh_dev->fifo_color_mask;
	spin_unlock(&eh_dev->fifo_prod_lock);

	written += scnprintf(buf + written, PAGE_SIZE - written, "%lu\n", size);
	return written;
}
DEVICE_ATTR_RO(queued_comp);

#ifdef CONFIG_GOOGLE_EH_LATENCY_STAT
int eh_init_latency_stat(struct eh_device *eh_dev)
{
	int cpu;

	eh_dev->stats = alloc_percpu(struct eh_stats);
	if (!eh_dev->stats)
		return -ENOMEM;

	for_each_possible_cpu(cpu) {
		int i;

		for (i = 0; i < NR_EH_EVENT_TYPE; i++)
			per_cpu_ptr(eh_dev->stats, cpu)->min_lat[i] = -1UL;
	}

	return 0;
}

void eh_deinit_latency_stat(struct eh_device *eh_dev)
{
	if (eh_dev->stats) {
		free_percpu(eh_dev->stats);
		eh_dev->stats = NULL;
	}
}

void eh_update_latency(struct eh_device *eh_dev, unsigned long start,
			      unsigned long event_count,
			      enum eh_stat_event type)
{
	unsigned long prev_avg, new_avg;
	unsigned long count;
	unsigned long delta, end;

	WARN_ON(start == 0);

	preempt_disable();

	end = ktime_get_ns();
	WARN_ON(start > end);

	count = __this_cpu_read(eh_dev->stats->events[type]);
	__this_cpu_add(eh_dev->stats->events[type], event_count);

	delta = end - start;
	prev_avg = __this_cpu_read(eh_dev->stats->avg_lat[type]);

	new_avg = (prev_avg * count + delta) / (count + event_count);
	__this_cpu_write(eh_dev->stats->avg_lat[type], new_avg);

	if (delta > __this_cpu_read(eh_dev->stats->max_lat[type]))
		__this_cpu_write(eh_dev->stats->max_lat[type], delta);

	if (delta < __this_cpu_read(eh_dev->stats->min_lat[type]))
		__this_cpu_write(eh_dev->stats->min_lat[type], delta);

	preempt_enable();
}

void set_submit_ts(struct eh_completion *cmpl, unsigned long ts)
{
	cmpl->submit_ts = ts;
}

unsigned long get_submit_ts(struct eh_completion *cmpl)
{
	return cmpl->submit_ts;
}

static ssize_t cmd_stat_show(struct device *dev,
			     struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int cpu, i;
	int written = 0;
	unsigned long x[NR_EH_EVENT_TYPE] = {
		0,
	};

	for_each_possible_cpu (cpu)
		for (i = 0; i < NR_EH_EVENT_TYPE; i++)
			x[i] += per_cpu_ptr(eh_dev->stats, cpu)->events[i];

	for (i = 0; i < NR_EH_EVENT_TYPE; i++)
		written += scnprintf(buf + written, PAGE_SIZE - written, "%lu ",
				     x[i]);

	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");
	return written;
}
DEVICE_ATTR_RO(cmd_stat);

static ssize_t avg_latency_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int cpu, i;
	int written = 0;
	unsigned long x[NR_EH_EVENT_TYPE] = {
		0,
	};
	int nr_cpu;

	for (i = 0; i < NR_EH_EVENT_TYPE; i++) {
		nr_cpu = 0;
		for_each_possible_cpu (cpu) {
			if (per_cpu_ptr(eh_dev->stats, cpu)->avg_lat[i] == 0)
				continue;
			x[i] += per_cpu_ptr(eh_dev->stats, cpu)->avg_lat[i];
			nr_cpu++;
		}

		x[i] = nr_cpu ? x[i] / nr_cpu : 0;
	}

	for (i = 0; i < NR_EH_EVENT_TYPE; i++)
		written += scnprintf(buf + written, PAGE_SIZE - written, "%lu ",
				     x[i]);

	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");
	return written;
}
DEVICE_ATTR_RO(avg_latency);

static ssize_t max_latency_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int cpu, i;
	int written = 0;
	unsigned long x[NR_EH_EVENT_TYPE] = {
		0,
	};

	for_each_online_cpu (cpu)
		for (i = 0; i < NR_EH_EVENT_TYPE; i++)
			x[i] = max(
				x[i],
				per_cpu_ptr(eh_dev->stats, cpu)->max_lat[i]);

	for (i = 0; i < NR_EH_EVENT_TYPE; i++)
		written += scnprintf(buf + written, PAGE_SIZE - written, "%lu ",
				     x[i]);

	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");
	return written;
}
DEVICE_ATTR_RO(max_latency);

static ssize_t min_latency_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int cpu, i;
	int written = 0;
	unsigned long x[NR_EH_EVENT_TYPE];

	/* initialize array with maximum unsigned value */
	memset(x, -1, sizeof(x));

	for_each_possible_cpu (cpu)
		for (i = 0; i < NR_EH_EVENT_TYPE; i++)
			x[i] = min(
				x[i],
				per_cpu_ptr(eh_dev->stats, cpu)->min_lat[i]);

	/* show zero value instead of maximum unsigned value for idle cpu */
	for (i = 0; i < NR_EH_EVENT_TYPE; i++)
		if (x[i] == -1UL)
			x[i] = 0;

	for (i = 0; i < NR_EH_EVENT_TYPE; i++)
		written += scnprintf(buf + written, PAGE_SIZE - written, "%lu ",
				     x[i]);

	written += scnprintf(buf + written, PAGE_SIZE - written, "\n");
	return written;
}
DEVICE_ATTR_RO(min_latency);

static ssize_t reset_latency_store(struct device *dev,
				   struct device_attribute *dev_attr,
				   const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct eh_device *eh_dev = platform_get_drvdata(pdev);
	int cpu, i;

	for_each_possible_cpu (cpu) {
		for (i = 0; i < NR_EH_EVENT_TYPE; i++) {
			per_cpu_ptr(eh_dev->stats, cpu)->min_lat[i] = -1UL;
			per_cpu_ptr(eh_dev->stats, cpu)->max_lat[i] = 0;
			per_cpu_ptr(eh_dev->stats, cpu)->avg_lat[i] = 0;
			per_cpu_ptr(eh_dev->stats, cpu)->events[i] = 0;
		}
	}
	return count;
}
DEVICE_ATTR_WO(reset_latency);
#endif

static struct attribute *eh_dev_attrs[] = {
	&dev_attr_queued_comp.attr,
#ifdef CONFIG_GOOGLE_EH_LATENCY_STAT
	&dev_attr_cmd_stat.attr,
	&dev_attr_avg_latency.attr,
	&dev_attr_max_latency.attr,
	&dev_attr_min_latency.attr,
	&dev_attr_reset_latency.attr,
#endif
	NULL
};
ATTRIBUTE_GROUPS(eh_dev);

int eh_sysfs_init(struct device *device)
{
	return device_add_groups(device, eh_dev_groups);
}
