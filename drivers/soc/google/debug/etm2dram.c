// SPDX-License-Identifier: GPL-2.0-only
/*
 * Module to access debugcore for controlling ETM2DRAM functions.
 *
 * Copyright (C) 2023 Google LLC.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s() " fmt, __func__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dynamic_debug.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/types.h>

#include <soc/google/exynos-adv-tracer.h>

#define INVALID_ID (-1)
#define DEFAULT_DBUF_SIZE (0x400000)

static int dbuf_size = DEFAULT_DBUF_SIZE;
module_param(dbuf_size, int, 0644);
MODULE_PARM_DESC(dbuf_size, "Data buffer size in bytes");

enum etm2dram_ipc_cmd {
	ETM2DRAM_IPC_CMD_SET_DATABUF,
	ETM2DRAM_IPC_CMD_SET_DEFERRED_ENABLE,
	ETM2DRAM_IPC_CMD_CANCEL_DEFERRED_ENABLE,
	ETM2DRAM_IPC_CMD_SET_ARM,
};

static unsigned int etm2dram_channel_id = INVALID_ID;

struct etm2dram_private {
	bool is_armed;
	/* protects alignment between 'is_armed' and outcomes of 'arm_store' */
	struct mutex arm_lock;
	dma_addr_t dbuf_base;
	size_t dbuf_size;
	struct notifier_block pm_nb;
};

int etm2dram_delayed_start(int delay_secs)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret;

	if (etm2dram_channel_id == INVALID_ID)
		return -EPERM;

	cmd.cmd_raw.cmd = ETM2DRAM_IPC_CMD_SET_DEFERRED_ENABLE;
	cmd.buffer[1] = delay_secs;

	pr_debug("delayed start is set after %d\n", delay_secs);

	ret = adv_tracer_ipc_send_data_polling(etm2dram_channel_id, &cmd);
	if (ret != 0)
		pr_err("ipc (channel=%d) failed: %d\n", etm2dram_channel_id, ret);
	else
		pr_debug("ipc (channel=%d) success\n", etm2dram_channel_id);

	return ret;
}
EXPORT_SYMBOL_GPL(etm2dram_delayed_start);

int etm2dram_cancel_delayed_start(void)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret;

	if (etm2dram_channel_id == INVALID_ID)
		return -EPERM;

	cmd.cmd_raw.cmd = ETM2DRAM_IPC_CMD_CANCEL_DEFERRED_ENABLE;

	pr_debug("cancelling delayed start ...\n");

	ret = adv_tracer_ipc_send_data_polling(etm2dram_channel_id, &cmd);
	if (ret != 0)
		pr_err("ipc (channel=%d) failed: %d\n", etm2dram_channel_id, ret);
	else
		pr_debug("ipc (channel=%d) success\n", etm2dram_channel_id);

	return ret;
}
EXPORT_SYMBOL_GPL(etm2dram_cancel_delayed_start);

static int etm2dram_set_arm_locked(bool enable)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret;

	if (etm2dram_channel_id == INVALID_ID)
		return -EPERM;

	cmd.cmd_raw.cmd = ETM2DRAM_IPC_CMD_SET_ARM;
	cmd.buffer[1] = enable;

	pr_debug("updating arm ...\n");

	ret = adv_tracer_ipc_send_data_polling(etm2dram_channel_id, &cmd);
	if (ret != 0)
		pr_err("ipc (channel=%d) failed: %d\n", etm2dram_channel_id, ret);
	else
		pr_debug("ipc (channel=%d) success\n", etm2dram_channel_id);

	return ret;
}

static int arm_update(struct etm2dram_private *data, bool new_value)
{
	int ret = 0;

	mutex_lock(&data->arm_lock);
	if (data->is_armed != new_value) {
		ret = etm2dram_set_arm_locked(new_value);
		if (!ret)
			data->is_armed = new_value;
	}
	mutex_unlock(&data->arm_lock);

	return ret;
}

static void arm_suspend(struct etm2dram_private *data)
{
	mutex_lock(&data->arm_lock);
	if (data->is_armed)
		etm2dram_set_arm_locked(false);
	mutex_unlock(&data->arm_lock);
}

static void arm_resume(struct etm2dram_private *data)
{
	mutex_lock(&data->arm_lock);
	if (data->is_armed)
		etm2dram_set_arm_locked(true);
	mutex_unlock(&data->arm_lock);
}

static int etm2dram_set_databuf(dma_addr_t base, int size)
{
	struct adv_tracer_ipc_cmd cmd;
	int ret;

	if (etm2dram_channel_id == INVALID_ID)
		return -EPERM;

	cmd.cmd_raw.cmd = ETM2DRAM_IPC_CMD_SET_DATABUF;
	cmd.buffer[1] = base & 0xffffffff;
	cmd.buffer[2] = (base >> 32) & 0xffffffff;
	cmd.buffer[3] = size;

	pr_debug("updating data buffer ...\n");

	ret = adv_tracer_ipc_send_data_polling(etm2dram_channel_id, &cmd);
	if (ret != 0)
		pr_err("ipc (channel=%d) failed: %d\n", etm2dram_channel_id, ret);
	else
		pr_debug("ipc (channel=%d) success\n", etm2dram_channel_id);

	return ret;
}

static int etm2dram_pm_notifier(struct notifier_block *notifier,
				unsigned long pm_event, void *v)
{
	struct etm2dram_private *data = container_of(notifier, struct etm2dram_private, pm_nb);

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		arm_suspend(data);
		break;
	case PM_POST_SUSPEND:
		arm_resume(data);
		break;
	}
	return NOTIFY_OK;
}

static void devm_unregister_pm_notifier(struct device *dev, void *res)
{
	int ret;
	struct notifier_block **nb_res = res;

	dev_dbg(dev, "unregister notifier %ps\n", *nb_res);
	ret = unregister_pm_notifier(*nb_res);
	if (ret)
		dev_err(dev, "unregister notifier failed: %d\n", ret);
}

static int devm_register_pm_notifier(struct device *dev, struct notifier_block *nb)
{
	int ret;
	struct notifier_block **nb_res;

	nb_res = devres_alloc(devm_unregister_pm_notifier, sizeof(*nb_res), GFP_KERNEL);
	if (!nb_res)
		return -ENOMEM;

	ret = register_pm_notifier(nb);
	if (ret) {
		dev_err(dev, "failed to register notifier: %d\n", ret);
		devres_free(nb_res);
		return ret;
	}

	dev_dbg(dev, "registered notifier %ps\n", nb);
	*nb_res = nb;
	devres_add(dev, nb_res);
	return 0;
}

static void devm_ipc_channel_release(struct device *dev, void *res)
{
	int ret;
	unsigned int **dr_data = res;

	dev_dbg(dev, "release ipc %d\n", **dr_data);
	ret = adv_tracer_ipc_release_channel(**dr_data);
	if (ret)
		dev_err(dev, "ipc release failed: %d\n", ret);

	**dr_data = INVALID_ID;
}

static int devm_ipc_channel_alloc(struct device *dev, unsigned int *id, unsigned int *len)
{
	int ret;
	unsigned int **dr_data;

	dr_data = devres_alloc(devm_ipc_channel_release, sizeof(*dr_data), GFP_KERNEL);
	if (!dr_data)
		return -ENOMEM;

	ret = adv_tracer_ipc_request_channel(dev->of_node, NULL, id, len);
	if (ret) {
		dev_err(dev, "failed to request ipc: %d\n", ret);
		devres_free(dr_data);
		return ret;
	}

	dev_dbg(dev, "request ipc at: %d len %d\n", *id, *len);
	*dr_data = id;
	devres_add(dev, dr_data);
	return 0;
}

struct devm_dma_data {
	void *mem;
	dma_addr_t mem_paddr;
	size_t mem_size;
};

static void devm_dma_free(struct device *dev, void *res)
{
	struct devm_dma_data *dr_data = res;

	dev_dbg(dev, "alloc dma at %pK\n", dr_data->mem);
	dma_free_coherent(dev, dr_data->mem_size, dr_data->mem, dr_data->mem_paddr);
}

static int devm_dma_alloc(struct device *dev, size_t size, dma_addr_t *dma_handle, gfp_t gfp)
{
	void *mem;
	struct devm_dma_data *dr_data;

	dr_data = devres_alloc(devm_dma_free, sizeof(*dr_data), GFP_KERNEL);
	if (!dr_data)
		return -ENOMEM;

	mem = dma_alloc_coherent(dev, size, dma_handle, gfp);
	if (!mem) {
		devres_free(dr_data);
		return -ENOMEM;
	}

	dev_dbg(dev, "alloc dma at %pK\n", mem);
	dr_data->mem = mem;
	dr_data->mem_paddr = *dma_handle;
	dr_data->mem_size = size;
	devres_add(dev, dr_data);
	return 0;
}

static void devm_debugfs_remove_recursive(struct device *dev, void *res)
{
	struct dentry **dentry_res = res;

	dev_dbg(dev, "unregister debugfs dentry %ps\n", *dentry_res);
	debugfs_remove_recursive(*dentry_res);
}

static struct dentry *devm_debugfs_create_dir(struct device *dev, const char *name, void *data)
{
	struct dentry **res;
	struct dentry *dentry;

	res = devres_alloc(devm_debugfs_remove_recursive, sizeof(*res), GFP_KERNEL);
	if (!res)
		return ERR_PTR(-ENOMEM);

	dentry = debugfs_create_dir(name, data);
	if (IS_ERR_OR_NULL(dentry)) {
		dev_err(dev, "failed to register debugfs dentry\n");
		devres_free(res);
		return ERR_PTR(-EFAULT);
	}

	dev_dbg(dev, "registered debugfs dentry %ps\n", dentry);
	*res = dentry;
	devres_add(dev, res);
	return dentry;
}

static int status_show(struct seq_file *s, void *unused)
{
	struct etm2dram_private *data = dev_get_drvdata(s->private);

	seq_puts(s, "reserved_mem:\n");
	seq_printf(s, " .base: %pad\n", &data->dbuf_base);
	seq_printf(s, " .size: %#zx\n", data->dbuf_size);
	seq_printf(s, "IPC channel ID: %d\n", etm2dram_channel_id);

	return 0;
}

static ssize_t
arm_store(struct device *dev, struct device_attribute *attr,
	  const char *buf, size_t count)
{
	bool new_value;
	int ret;
	struct etm2dram_private *data = dev_get_drvdata(dev);

	ret = strtobool(buf, &new_value);
	if (ret < 0)
		return ret;

	ret = arm_update(data, new_value);
	return ret ? ret : count;
}

static DEVICE_ATTR_WO(arm);

static struct attribute *etm2dram_sysfs_attrs[] = {
	&dev_attr_arm.attr,
	NULL,
};

static const struct attribute_group etm2dram_sysfs_group = {
	.attrs = etm2dram_sysfs_attrs,
};

static int etm2dram_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int channel_len;
	struct etm2dram_private *data;
	struct dentry *debug_dir;

	if (etm2dram_channel_id != INVALID_ID)
		return -EBUSY;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, data);

	data->is_armed = false;
	mutex_init(&data->arm_lock);
	data->dbuf_size = PAGE_ALIGN(dbuf_size);

	ret = devm_dma_alloc(&pdev->dev, data->dbuf_size, &data->dbuf_base, GFP_KERNEL);
	if (ret != 0) {
		dev_err(&pdev->dev, "data buffer allocation failed: %d\n", ret);
		return ret;
	}

	ret = devm_ipc_channel_alloc(&pdev->dev, &etm2dram_channel_id, &channel_len);
	if (ret != 0) {
		dev_err(&pdev->dev, "ipc request failed: %d\n", ret);
		return ret;
	}

	if (etm2dram_channel_id == INVALID_ID) {
		dev_err(&pdev->dev, "channel ID is invalid\n");
		return -EFAULT;
	}

	ret = devm_device_add_group(&pdev->dev, &etm2dram_sysfs_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to add sysfs group: %d\n", ret);
		return ret;
	}

	debug_dir = devm_debugfs_create_dir(&pdev->dev, "etm2dram", NULL);
	if (IS_ERR_OR_NULL(debug_dir)) {
		dev_err(&pdev->dev, "failed to create debugfs dir\n");
		return -EFAULT;
	}

	debugfs_create_devm_seqfile(&pdev->dev, "status", debug_dir, status_show);

	data->pm_nb.notifier_call = etm2dram_pm_notifier;
	ret = devm_register_pm_notifier(&pdev->dev, &data->pm_nb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register pm notifier: %d\n", ret);
		return ret;
	}

	return etm2dram_set_databuf(data->dbuf_base, data->dbuf_size);
}

static int etm2dram_remove(struct platform_device *pdev)
{
	return etm2dram_set_databuf(0, 0);
}

static const struct of_device_id etm2dram_matches[] = {
	{.compatible = "google,etm2dram"},
	{},
};
MODULE_DEVICE_TABLE(of, etm2dram_matches);

static struct platform_driver etm2dram_driver = {
	.probe		= etm2dram_probe,
	.remove		= etm2dram_remove,
	.driver		= {
		.name	= "etm2dram",
		.of_match_table	= of_match_ptr(etm2dram_matches),
	},
};
module_platform_driver(etm2dram_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Module to control ETM2DRAM functions.");
MODULE_AUTHOR("Woody Lin <woodylin@google.com>");
