// SPDX-License-Identifier: GPL-2.0
/*
 * pixel-suspend-diag.c - use to diagnose suspend/resume perforance
 *
 * Copyright 2023 Google LLC
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched/clock.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

#include <soc/google/pixel-suspend-diag.h>
#include "debug-snapshot-local.h"

#define PIXEL_SUSPEND_DIAG_DELTA_TIME_H_MASK		(0xFFFFU)
#define PIXEL_SUSPEND_DIAG_DELTA_TIME_H_SHIFT		(32)
#define PIXEL_SUSPEND_DIAG_DELTA_TIME_L_MASK		(0xFFFFFFFFU)
#define PIXEL_SUSPEND_DIAG_DELTA_TIME_L_SHIFT		(0)

#define PIXEL_SUSPEND_DIAG_GET_DELTA_TIME_H(delta) \
	(((delta) >> PIXEL_SUSPEND_DIAG_DELTA_TIME_H_SHIFT) & PIXEL_SUSPEND_DIAG_DELTA_TIME_H_MASK)

#define PIXEL_SUSPEND_DIAG_GET_DELTA_TIME_L(delta) \
	(((delta) >> PIXEL_SUSPEND_DIAG_DELTA_TIME_L_SHIFT) & PIXEL_SUSPEND_DIAG_DELTA_TIME_L_MASK)

#define PIXEL_SUSPEND_DIAG_DELTA_TIME(delta_h, delta_l) \
	((((uint64_t)(delta_h) & PIXEL_SUSPEND_DIAG_DELTA_TIME_H_MASK) << \
	  PIXEL_SUSPEND_DIAG_DELTA_TIME_H_SHIFT) | \
	 (((uint64_t)(delta_l) & PIXEL_SUSPEND_DIAG_DELTA_TIME_L_MASK) << \
	  PIXEL_SUSPEND_DIAG_DELTA_TIME_L_SHIFT))

enum pixel_suspend_diag_item_index {
	PIXEL_SYNC_FILESYSTEMS_ID = 0,
	PIXEL_FREEZE_PROCESSES_ID,
	PIXEL_SUSPEND_ENTER_ID,
	PIXEL_DPM_PREPARE_ID,
	PIXEL_DPM_SUSPEND_ID,
	PIXEL_DPM_SUSPEND_LATE_ID,
	PIXEL_DPM_SUSPEND_NOIRQ_ID,
	PIXEL_CPU_OFF_ID,
	PIXEL_SYSCORE_SUSPEND_ID,
	PIXEL_MACHINE_SUSPEND_ID,
	PIXEL_SYSCORE_RESUME_ID,
	PIXEL_CPU_ON_ID,
	PIXEL_DPM_RESUME_NOIRQ_ID,
	PIXEL_DPM_RESUME_EARLY_ID,
	PIXEL_DPM_RESUME_ID,
	PIXEL_DPM_COMPLETE_ID,
	PIXEL_RESUME_CONSOLE_ID,
	PIXEL_THAW_PROCESSES_ID,
};

struct pixel_suspend_diag_item {
	const char *action;
	uint64_t timeout;
	bool enabled;
};

static struct pixel_suspend_diag_item pixel_suspend_diag_items[] = {
	[PIXEL_SYNC_FILESYSTEMS_ID]	= {"sync_filesystems", 3 * NSEC_PER_SEC, false},
	[PIXEL_FREEZE_PROCESSES_ID]	= {"freeze_processes", NSEC_PER_SEC, false},
	[PIXEL_SUSPEND_ENTER_ID]	= {"suspend_enter", NSEC_PER_SEC, false},
	[PIXEL_DPM_PREPARE_ID]		= {"dpm_prepare", NSEC_PER_SEC, true},
	[PIXEL_DPM_SUSPEND_ID]		= {"dpm_suspend", NSEC_PER_SEC, true},
	[PIXEL_DPM_SUSPEND_LATE_ID]	= {"dpm_suspend_late", NSEC_PER_SEC, true},
	[PIXEL_DPM_SUSPEND_NOIRQ_ID]	= {"dpm_suspend_noirq", NSEC_PER_SEC, true},
	[PIXEL_CPU_OFF_ID]		= {"cpu_off", NSEC_PER_SEC, false},
	[PIXEL_SYSCORE_SUSPEND_ID]	= {"syscore_suspend", NSEC_PER_SEC, false},
	[PIXEL_MACHINE_SUSPEND_ID]	= {"machine_suspend", NSEC_PER_SEC, false},
	[PIXEL_SYSCORE_RESUME_ID]	= {"syscore_resume", NSEC_PER_SEC, false},
	[PIXEL_CPU_ON_ID]		= {"cpu_on", NSEC_PER_SEC, false},
	[PIXEL_DPM_RESUME_NOIRQ_ID]	= {"dpm_resume_noirq", NSEC_PER_SEC, true},
	[PIXEL_DPM_RESUME_EARLY_ID]	= {"dpm_resume_early", NSEC_PER_SEC, true},
	[PIXEL_DPM_RESUME_ID]		= {"dpm_resume", NSEC_PER_SEC, true},
	[PIXEL_DPM_COMPLETE_ID]		= {"dpm_complete", NSEC_PER_SEC, true},
	[PIXEL_RESUME_CONSOLE_ID]	= {"resume_console", NSEC_PER_SEC, false},
	[PIXEL_THAW_PROCESSES_ID]	= {"thaw_processes", NSEC_PER_SEC, false},
};

struct pixel_suspend_diag_info {
	uint32_t enable;
	uint32_t force_panic;
	uint64_t last_index;
	uint64_t curr_index;
	uint64_t timeout;
	char action[32];
} __packed;

static struct pixel_suspend_diag_info pixel_suspend_diag_inst;

void *pixel_suspend_diag_get_info(void)
{
	return (void *)&pixel_suspend_diag_inst;
}
EXPORT_SYMBOL_GPL(pixel_suspend_diag_get_info);

static void pixel_suspend_diag_handle_suspend_resume(struct dbg_snapshot_log *dss_log,
						     uint64_t last_idx, uint64_t curr_idx)
{
	uint64_t idx = (last_idx + 1) % ARRAY_SIZE(dss_log->suspend);
	bool has_dev_pm_cb = (idx != curr_idx);
	uint64_t delta_time = 0;
	int i;

	if (!has_dev_pm_cb) {
		delta_time = dss_log->suspend[curr_idx].time - dss_log->suspend[last_idx].time;
	} else {
		/*
		 * dev_pm_cb have been run by multi cores between last_idx and curr_idx
		 * so we can't use dss_log->suspend[curr_idx].time - dss_log->suspend[last_idx].time
		 * directly to determine delta time
		 */
		while (idx != curr_idx) {
			delta_time +=
				PIXEL_SUSPEND_DIAG_DELTA_TIME(dss_log->suspend[idx].delta_time_h,
							      dss_log->suspend[idx].delta_time_l);
			idx = (idx + 1) % ARRAY_SIZE(dss_log->suspend);
		}
	}

	for (i = 0; i < ARRAY_SIZE(pixel_suspend_diag_items); i++) {
		if (!strcmp(dss_log->suspend[curr_idx].log, pixel_suspend_diag_items[i].action))
			break;
	}

	if (i == ARRAY_SIZE(pixel_suspend_diag_items))
		return;

	if (delta_time < pixel_suspend_diag_items[i].timeout)
		return;

	if (strlen(pixel_suspend_diag_inst.action) == 0 && pixel_suspend_diag_items[i].enabled)
		goto crash;

	if (strcmp(pixel_suspend_diag_inst.action, dss_log->suspend[curr_idx].log))
		return;

crash:
	pixel_suspend_diag_inst.force_panic = 0x1;
	pixel_suspend_diag_inst.timeout = pixel_suspend_diag_items[i].timeout;
	panic("%s: %s%s(%llu) to %s%s(%llu) %stook %llu.%llu s\n", __func__,
	      dss_log->suspend[last_idx].log ? dss_log->suspend[last_idx].log : "",
	      dss_log->suspend[last_idx].en == DSS_FLAG_IN ? " IN" : " OUT", last_idx,
	      dss_log->suspend[curr_idx].log ? dss_log->suspend[curr_idx].log : "",
	      dss_log->suspend[curr_idx].en == DSS_FLAG_IN ? " IN" : " OUT", curr_idx,
	      has_dev_pm_cb ? "callbacks " : "",
	      delta_time / NSEC_PER_SEC, (delta_time % NSEC_PER_SEC) / USEC_PER_SEC);
}

void pixel_suspend_diag_suspend_resume(void *dbg_snapshot_log, const char *action, bool start,
				       uint64_t curr_index)
{
	pixel_suspend_diag_inst.curr_index = curr_index;

	if (!pixel_suspend_diag_inst.enable)
		return;

	if (start || !action)
		goto backup;

	pixel_suspend_diag_handle_suspend_resume(dbg_snapshot_log,
						 pixel_suspend_diag_inst.last_index,
						 pixel_suspend_diag_inst.curr_index);

backup:
	pixel_suspend_diag_inst.last_index = pixel_suspend_diag_inst.curr_index;
}

EXPORT_SYMBOL_GPL(pixel_suspend_diag_suspend_resume);

bool pixel_suspend_diag_dev_pm_cb_end(void *dbg_snapshot_log, uint64_t first_log_idx,
				      uint64_t last_log_idx, struct device *dev)
{
	uint64_t i;
	struct dbg_snapshot_log *dss_log = dbg_snapshot_log;
	uint64_t end_time;
	uint64_t delta_time;

	if (!pixel_suspend_diag_inst.enable)
		return false;

	end_time = local_clock();

	i = last_log_idx;
	while(i != first_log_idx) {
		if (dev && end_time >= dss_log->suspend[i].time &&
		    dss_log->suspend[i].dev == dev_name(dev)) {
		    delta_time = end_time - dss_log->suspend[i].time;
		    dss_log->suspend[i].delta_time_h =
				PIXEL_SUSPEND_DIAG_GET_DELTA_TIME_H(delta_time);
		    dss_log->suspend[i].delta_time_l =
				PIXEL_SUSPEND_DIAG_GET_DELTA_TIME_L(delta_time);
			break;
		}
		i = (i - 1) % ARRAY_SIZE(dss_log->suspend);
	}

	return true;
}

EXPORT_SYMBOL_GPL(pixel_suspend_diag_dev_pm_cb_end);

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
			    size_t count)
{
	uint64_t val;
	int ret;

	ret = kstrtoul(buf, 10, (unsigned long *)&val);

	if (!ret)
		pixel_suspend_diag_inst.enable = val;

	return count;
}

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%sable\n",
			 !!pixel_suspend_diag_inst.enable ? "en" : "dis");
}

static struct kobj_attribute pixel_suspend_diag_attr_enable = __ATTR_RW_MODE(enable, 0660);

static ssize_t timeout_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
			     size_t count)
{
	int i;
	char item_action[32];
	char *item_timeout, *space;
	int action_size;
	uint64_t val;

	/*
	 * Extract buf before the first space.
	 */
	space = strchr(buf, ' ');
	if (!space) {
		pr_warn("invalid parameters format in buffer [%s]\n", buf);
		return -EINVAL;
	}

	action_size = space - buf + 1;
	item_timeout = space + 1;
	if (action_size > sizeof(item_action)) {
		pr_warn("invalid action parameter in buffer [%s]\n", buf);
		return -EINVAL;
	}
	strlcpy(item_action, buf, action_size);

	if (kstrtoll(item_timeout, 10, &val)) {
		pr_warn("invalid timeout parameter in buffer [%s]\n", buf);
		return -EINVAL;
	}

	if (!strcmp(item_action, "all")) {
		for (i = 0; i < ARRAY_SIZE(pixel_suspend_diag_items); i++) {
			pixel_suspend_diag_items[i].timeout = val;
		}
		return count;
	}

	for (i = 0; i < ARRAY_SIZE(pixel_suspend_diag_items); i++) {
		if (!strcmp(item_action, pixel_suspend_diag_items[i].action)) {
			pixel_suspend_diag_items[i].timeout = val;
			return count;
		}
	}

	pr_warn("item action doesn't exist in default list [%s]\n", item_action);
	return -EEXIST;
}

static ssize_t timeout_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i;
	ssize_t size = 0;

	for (i = 0; i < ARRAY_SIZE(pixel_suspend_diag_items); i++) {
		size += scnprintf(buf + size, PAGE_SIZE - size, "%s: %llu(ns)\n",
				  pixel_suspend_diag_items[i].action,
				  pixel_suspend_diag_items[i].timeout);
	}

	return size;
}

static struct kobj_attribute pixel_suspend_diag_attr_timeout = __ATTR_RW_MODE(timeout, 0660);

static ssize_t action_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
			    size_t count)
{
	char *newline = NULL;

	strlcpy(pixel_suspend_diag_inst.action, buf, sizeof(pixel_suspend_diag_inst.action));
	newline = strchr(pixel_suspend_diag_inst.action, '\n');
	if (newline)
		*newline = '\0';

	return count;
}

static ssize_t action_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", pixel_suspend_diag_inst.action);
}

static struct kobj_attribute pixel_suspend_diag_attr_action = __ATTR_RW_MODE(action, 0660);

static struct attribute *pixel_suspend_diag_attrs[] = {
	&pixel_suspend_diag_attr_enable.attr,
	&pixel_suspend_diag_attr_timeout.attr,
	&pixel_suspend_diag_attr_action.attr,
	NULL
};

static const struct attribute_group pixel_suspend_diag_attr_group = {
	.attrs = pixel_suspend_diag_attrs,
	.name = "suspend_diag"
};

static int pixel_suspend_diag_probe(struct platform_device *pdev)
{
	struct kobject *pixel_suspend_diag_kobj;

	pixel_suspend_diag_kobj = kobject_create_and_add("dbg_snapshot", kernel_kobj);
	if (!pixel_suspend_diag_kobj)
		return -EINVAL;

	if (sysfs_create_group(pixel_suspend_diag_kobj, &pixel_suspend_diag_attr_group)) {
		dev_err(&pdev->dev, "cannot create files in ../dbg_snapshot/suspend_diag\n");
		kobject_put(pixel_suspend_diag_kobj);
		return -EINVAL;
	}

	return 0;
}

static const struct of_device_id pixel_suspend_diag_of_match[] = {
	{ .compatible	= "google,pixel-suspend-diag" },
	{},
};
MODULE_DEVICE_TABLE(of, pixel_suspend_diag_of_match);

static struct platform_driver pixel_suspend_diag_driver = {
	.probe = pixel_suspend_diag_probe,
	.driver = {
		.name = "pixel-suspend-diag",
		.of_match_table = of_match_ptr(pixel_suspend_diag_of_match),
	},
};
module_platform_driver(pixel_suspend_diag_driver);

MODULE_AUTHOR("Jone Chou <jonechou@google.com>");
MODULE_DESCRIPTION("Pixel Suspend Diag Driver");
MODULE_LICENSE("GPL v2");
