// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/cpu_pm.h>
#include <linux/suspend.h>

#include <soc/google/exynos-adv-tracer.h>
#include <soc/google/exynos-ehld.h>

enum ehld_ipc_cmd {
	eEHLD_IPC_CMD_SET_ENABLE,
	eEHLD_IPC_CMD_GET_ENABLE,
	eEHLD_IPC_CMD_SET_INTERVAL,
	eEHLD_IPC_CMD_SET_LOCKUP_WARN_VAL,
	eEHLD_IPC_CMD_SET_LOCKUP_VAL,
	eEHLD_IPC_CMD_SET_INIT_VAL,
	eEHLD_IPC_CMD_NOTI_CPU_ON,
	eEHLD_IPC_CMD_NOTI_CPU_OFF,
	eEHLD_IPC_CMD_LOCKUP_DETECT_WARN,
	eEHLD_IPC_CMD_LOCKUP_DETECT_SW,
	eEHLD_IPC_CMD_LOCKUP_DETECT_HW,
	eEHLD_IPC_CMD_SET_PMU_CNTR_ID,
};

static struct plugin_ehld_info {
	struct adv_tracer_plugin *ehld_dev;
	unsigned int enable;
	unsigned int interval;
} plugin_ehld;

int adv_tracer_ehld_get_enable(void)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret = 0;

	cmd.cmd_raw.cmd = eEHLD_IPC_CMD_GET_ENABLE;
	ret = adv_tracer_ipc_send_data_polling(plugin_ehld.ehld_dev->id,
					(struct adv_tracer_ipc_cmd *)&cmd);
	if (ret < 0) {
		pr_err("ehld ipc cannot get enable\n");
		return ret;
	}
	plugin_ehld.enable = cmd.buffer[1];

	pr_info("EHLD %sabled\n", plugin_ehld.enable ? "en" : "dis");

	return plugin_ehld.enable;
}

int adv_tracer_ehld_set_enable(int en)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret = 0;

	cmd.cmd_raw.cmd = eEHLD_IPC_CMD_SET_ENABLE;
	cmd.buffer[1] = en;
	ret = adv_tracer_ipc_send_data_polling(plugin_ehld.ehld_dev->id, &cmd);
	if (ret < 0) {
		pr_err("ehld ipc cannot enable setting\n");
		return ret;
	}
	plugin_ehld.enable = en;
	return 0;
}

int adv_tracer_ehld_set_init_val(u32 interval, u32 count, u32 cpu_mask)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret = 0;

	if (!interval)
		return -EINVAL;

	cmd.cmd_raw.cmd = eEHLD_IPC_CMD_SET_INIT_VAL;
	cmd.buffer[1] = interval;
	cmd.buffer[2] = count;
	cmd.buffer[3] = cpu_mask;

	ret = adv_tracer_ipc_send_data_polling(plugin_ehld.ehld_dev->id, &cmd);
	if (ret < 0) {
		pr_err("adv-tracer-ehld failed to init value\n");
		return ret;
	}
	plugin_ehld.interval = interval;
	pr_info("adv-tracer-ehld set interval to %u ms\n", interval);

	return 0;
}

int adv_tracer_ehld_set_interval(u32 interval)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret = 0;

	if (!interval)
		return -EINVAL;

	cmd.cmd_raw.cmd = eEHLD_IPC_CMD_SET_INTERVAL;
	cmd.buffer[1] = interval;
	ret = adv_tracer_ipc_send_data(plugin_ehld.ehld_dev->id, &cmd);
	if (ret < 0) {
		pr_err("ehld ipc can't set the interval\n");
		return ret;
	}
	plugin_ehld.interval = interval;
	pr_info("adv-tracer-ehld set interval to %u ms\n", interval);

	return 0;
}

int adv_tracer_ehld_set_warn_count(u32 count)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret = 0;

	if (!count)
		return -EINVAL;

	cmd.cmd_raw.cmd = eEHLD_IPC_CMD_SET_LOCKUP_WARN_VAL;
	cmd.buffer[1] = count;
	ret = adv_tracer_ipc_send_data(plugin_ehld.ehld_dev->id, &cmd);
	if (ret < 0) {
		pr_err("ehld ipc can't set lockup warning count\n");
		return ret;
	}
	pr_info("adv-tracer-ehld set warning count for %u times\n", count);

	return 0;
}

int adv_tracer_ehld_set_lockup_count(u32 count)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret = 0;

	if (!count)
		return 0;

	cmd.cmd_raw.cmd = eEHLD_IPC_CMD_SET_LOCKUP_VAL;
	cmd.buffer[1] = count;
	ret = adv_tracer_ipc_send_data(plugin_ehld.ehld_dev->id, &cmd);
	if (ret < 0) {
		pr_err("ehld ipc can't set lockup count\n");
		return ret;
	}
	pr_info("adv-tracer-ehld set lockup count for %u times\n", count);

	return 0;
}

u32 adv_tracer_ehld_get_interval(void)
{
	return plugin_ehld.interval;
}

int adv_tracer_ehld_noti_cpu_state(int cpu, int en)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret = 0;

	if (!ret)
		return -EINVAL;

	if (en)
		cmd.cmd_raw.cmd = eEHLD_IPC_CMD_NOTI_CPU_ON;
	else
		cmd.cmd_raw.cmd = eEHLD_IPC_CMD_NOTI_CPU_OFF;

	cmd.buffer[1] = cpu;

	ret = adv_tracer_ipc_send_data_polling(plugin_ehld.ehld_dev->id, &cmd);
	if (ret < 0) {
		pr_err("ehld ipc cannot cmd state\n");
		return ret;
	}

	return 0;
}

int adv_tracer_ehld_set_pmu_cntr_id(int cpu, int en, int cntr_id)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret = 0;

	cmd.cmd_raw.cmd = eEHLD_IPC_CMD_SET_PMU_CNTR_ID;

	cmd.buffer[1] = cpu;
	cmd.buffer[2] = en;
	cmd.buffer[3] = cntr_id;

	ret = adv_tracer_ipc_send_data_polling(plugin_ehld.ehld_dev->id, &cmd);
	if (ret < 0) {
		pr_err("ehld ipc cannot cmd state\n");
		return ret;
	}

	return 0;
}

static void adv_tracer_ehld_handler(struct adv_tracer_ipc_cmd *cmd,
				    unsigned int len)
{
	switch (cmd->cmd_raw.cmd) {
	case eEHLD_IPC_CMD_LOCKUP_DETECT_WARN:
		pr_err("CPU%u is Early Hardlockup Detected Warning - counter:%#x\n",
							cmd->buffer[1], cmd->buffer[2]);
		exynos_ehld_do_policy();
		break;
	case eEHLD_IPC_CMD_LOCKUP_DETECT_HW:
		pr_err("CPU%u is Early Hardlockup Detected - counter:%#x, Caused by HW\n",
							cmd->buffer[1], cmd->buffer[2]);
		exynos_ehld_do_policy();
		break;
	case eEHLD_IPC_CMD_LOCKUP_DETECT_SW:
		pr_err("CPU%u is Early Hardlockup Detected - counter:%#x, Caused by SW\n",
							cmd->buffer[1], cmd->buffer[2]);
		exynos_ehld_do_policy();
		break;
	}
}

static ssize_t ehld_enable_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);

	if (!ret)
		adv_tracer_ehld_set_enable(!!val);

	return count;
}

static ssize_t ehld_enable_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%sable\n",
			plugin_ehld.enable ? "en" : "dis");
}

static struct kobj_attribute ehld_enable_attr = __ATTR_RW_MODE(ehld_enable, 0660);

static struct bus_type ehld_subsys = {
	.name = "exynos-adv-tracer-ehld",
	.dev_name = "exynos-adv-tracer-ehld",
};

static struct attribute *adv_tracer_ehld_sysfs_attrs[] = {
	&ehld_enable_attr.attr,
	NULL,
};

static struct attribute_group adv_tracer_ehld_sysfs_group = {
	.attrs = adv_tracer_ehld_sysfs_attrs,
};

static const struct attribute_group *adv_tracer_ehld_sysfs_groups[] = {
	&adv_tracer_ehld_sysfs_group,
	NULL,
};

static int adv_tracer_ehld_sysfs_init(void)
{
	return subsys_system_register(&ehld_subsys, adv_tracer_ehld_sysfs_groups);
}

static int adv_tracer_ehld_c2_pm_notifier(struct notifier_block *self,
						unsigned long action, void *v)
{
	if (!plugin_ehld.enable)
		return NOTIFY_OK;

	switch (action) {
	case CPU_PM_ENTER:
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		break;
	case CPU_CLUSTER_PM_ENTER:
		break;
	case CPU_CLUSTER_PM_ENTER_FAILED:
	case CPU_CLUSTER_PM_EXIT:
		break;
	}
	return NOTIFY_OK;
}

static int adv_tracer_ehld_pm_notifier(struct notifier_block *notifier,
				       unsigned long pm_event, void *v)
{
	if (!plugin_ehld.enable)
		return NOTIFY_OK;

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		break;
	case PM_POST_SUSPEND:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block adv_tracer_ehld_nb = {
	.notifier_call = adv_tracer_ehld_pm_notifier,
};

static struct notifier_block adv_tracer_ehld_c2_pm_nb = {
	.notifier_call = adv_tracer_ehld_c2_pm_notifier,
};

int adv_tracer_ehld_remove(void)
{
	struct adv_tracer_plugin *ehld = plugin_ehld.ehld_dev;

	adv_tracer_ipc_release_channel(ehld->id);
	kfree(ehld);

	return 0;
}

int adv_tracer_ehld_init(void *data)
{
	struct device_node *node = (struct device_node *)data;
	struct adv_tracer_plugin *ehld = NULL;
	int ret;

	ehld = kzalloc(sizeof(struct adv_tracer_plugin), GFP_KERNEL);
	if (!ehld) {
		ret = -ENOMEM;
		goto err_ehld_info;
	}

	plugin_ehld.ehld_dev = ehld;
	ret = adv_tracer_ipc_request_channel(node,
					(ipc_callback)adv_tracer_ehld_handler,
					&ehld->id, &ehld->len);
	if (ret < 0) {
		pr_err("ehld ipc request fail(%d)\n", ret);
		ret = -ENODEV;
		goto err_sysfs_probe;
	}

	ret = adv_tracer_ehld_sysfs_init();
	if (ret) {
		pr_err("adv-tracer-ehld fail to register sysfs.\n");
		return ret;
	}

	/* register pm notifier */
	register_pm_notifier(&adv_tracer_ehld_nb);

	/* register cpu pm notifier for C2 */
	cpu_pm_register_notifier(&adv_tracer_ehld_c2_pm_nb);

	pr_info("adv-tracer-ehld init successful.\n");
	return 0;

err_sysfs_probe:
	kfree(ehld);
err_ehld_info:
	return ret;
}
