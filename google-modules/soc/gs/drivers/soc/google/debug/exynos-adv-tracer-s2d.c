// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
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
#include <linux/keydebug.h>
#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-adv-tracer.h>
#include <soc/google/exynos-pmu-if.h>
#if IS_ENABLED(CONFIG_EXYNOS_CPUIDLE)
#include <soc/google/exynos-cpupm.h>
#endif
#include <dt-bindings/soc/google/debug-snapshot-def.h>

enum s2d_ipc_cmd {
	eS2D_IPC_CMD_SET_ALL_BLK = 1,
	eS2D_IPC_CMD_GET_ALL_BLK,
	eS2D_IPC_CMD_SET_ENABLE,
	eS2D_IPC_CMD_GET_ENABLE,
	eS2D_IPC_CMD_SET_BLK,
	eS2D_IPC_CMD_SET_DISABLE_GPR,
	eS2D_IPC_CMD_GET_DISABLE_GPR,
};

struct plugin_s2d_info {
	struct adv_tracer_plugin *s2d_dev;
	struct device *dev;
	unsigned int enable;
	unsigned int burnin_ctrl;
	int sel_scanmode;
	int dbgsel_sw;
	bool arraydump_done;
	int blk_count;
	const char **blk_names;
};

#define DONE_ARRYDUMP 0xADADADAD

static struct plugin_s2d_info plugin_s2d;

int adv_tracer_s2d_scandump(void)
{
	if (!plugin_s2d.burnin_ctrl || plugin_s2d.sel_scanmode < 0 ||
			plugin_s2d.dbgsel_sw < 0) {
		dev_err(plugin_s2d.dev, "pmu offset no data\n");
		return -1;
	}
	exynos_pmu_update(plugin_s2d.burnin_ctrl,
			BIT(plugin_s2d.sel_scanmode),
			BIT(plugin_s2d.sel_scanmode));
	dev_info(plugin_s2d.dev, "enter scandump mode!\n");
	exynos_pmu_update(plugin_s2d.burnin_ctrl,
			BIT(plugin_s2d.dbgsel_sw), BIT(plugin_s2d.dbgsel_sw));
	return 0;
}

int adv_tracer_s2d_arraydump(void)
{
	struct adv_tracer_ipc_cmd cmd = { 0 };
	int ret = 0;
	u32 cpu_mask;

	bitmap_to_arr32(&cpu_mask, cpumask_bits(cpu_possible_mask), 32);
	if (!plugin_s2d.s2d_dev)
		return -ENODEV;

	if (plugin_s2d.arraydump_done == DONE_ARRYDUMP) {
		dev_info(plugin_s2d.dev, "Arraydump already done(0x%x)\n",
				cpu_mask);
		return -1;
	}
	plugin_s2d.arraydump_done = DONE_ARRYDUMP;

	dev_info(plugin_s2d.dev, "Start Arraydump (0x%x)\n", cpu_mask);
	cmd.cmd_raw.cmd = EAT_IPC_CMD_ARRAYDUMP;
	cmd.cmd_raw.id = ARR_IPC_CMD_ID_KERNEL_ARRAYDUMP;
	cmd.buffer[1] = dbg_snapshot_get_item_paddr(DSS_ITEM_ARRDUMP_PANIC);
	cmd.buffer[2] = cpu_mask;
	ret = adv_tracer_ipc_send_data_polling_timeout(EAT_FRM_CHANNEL,
			&cmd, EAT_IPC_TIMEOUT * 100);
	if (ret < 0)
		goto end;

	dev_info(plugin_s2d.dev, "Finish Arraydump (0x%x)\n", cmd.buffer[1]);
end:
	return ret;
}

static int adv_tracer_s2d_get_enable(void)
{
	struct adv_tracer_ipc_cmd cmd = { 0 };
	int ret = 0;

	cmd.cmd_raw.cmd = eS2D_IPC_CMD_GET_ENABLE;
	ret = adv_tracer_ipc_send_data(plugin_s2d.s2d_dev->id, &cmd);
	if (ret < 0) {
		dev_err(plugin_s2d.dev, "ipc can't get enable\n");
		return ret;
	}
	plugin_s2d.enable = cmd.buffer[1];

	return plugin_s2d.enable;
}

static int adv_tracer_s2d_set_enable(int en)
{
	struct adv_tracer_ipc_cmd cmd = { 0 };
	int ret = 0;

	cmd.cmd_raw.cmd = eS2D_IPC_CMD_SET_ENABLE;
	cmd.buffer[1] = en;
	ret = adv_tracer_ipc_send_data(plugin_s2d.s2d_dev->id, &cmd);
	if (ret < 0) {
		dev_err(plugin_s2d.dev, "ipc can't enable setting\n");
		return ret;
	}
	plugin_s2d.enable = en;
	return 0;
}

static int adv_tracer_s2d_get_all_blk(unsigned long *p_blocks)
{
	struct adv_tracer_ipc_cmd cmd = { 0 };
	int ret = 0;

	cmd.cmd_raw.cmd = eS2D_IPC_CMD_GET_ALL_BLK;
	ret = adv_tracer_ipc_send_data(plugin_s2d.s2d_dev->id, &cmd);
	if (ret < 0) {
		dev_err(plugin_s2d.dev, "cannot get blk list\n");
		return ret;
	}

	*p_blocks = (unsigned long)cmd.buffer[2] << 32 | cmd.buffer[1];

	return 0;
}

int adv_tracer_s2d_get_blk_by_idx(unsigned int index)
{
	unsigned long blocks;
	int ret;

	if (index >= plugin_s2d.blk_count)
		return -EINVAL;

	ret = adv_tracer_s2d_get_all_blk(&blocks);

	if (ret)
		return ret;

	return !!(blocks & BIT(index));
}

int adv_tracer_s2d_get_blk_by_name(const char *name)
{
	unsigned int i;

	for (i = 0; i < plugin_s2d.blk_count; i++) {
		if (!strcmp(name, plugin_s2d.blk_names[i]))
			return adv_tracer_s2d_get_blk_by_idx(i);
	}

	return -EINVAL;
}

int adv_tracer_s2d_set_blk_by_idx(bool enabled, unsigned int index)
{
	struct adv_tracer_ipc_cmd cmd = { 0 };
	int ret = 0;

	if (index >= plugin_s2d.blk_count)
		return -EINVAL;

	cmd.cmd_raw.cmd = eS2D_IPC_CMD_SET_BLK;
	cmd.buffer[1] = enabled;
	cmd.buffer[2] = index;
	ret = adv_tracer_ipc_send_data(plugin_s2d.s2d_dev->id, &cmd);
	if (ret < 0) {
		dev_err(plugin_s2d.dev, "cannot %sable %s blk\n",
			enabled ? "en" : "dis",
			plugin_s2d.blk_names[index]);
		return ret;
	}

	if (cmd.cmd_raw.ret_err) {
		dev_err(plugin_s2d.dev, "%sabling %s blk rejected\n",
			enabled ? "en" : "dis",
			plugin_s2d.blk_names[index]);
		return -EPERM;
	}

	return 0;
}

int adv_tracer_s2d_set_blk_by_name(bool enabled, const char *name)
{
	unsigned int i;

	for (i = 0; i < plugin_s2d.blk_count; i++) {
		if (!strcmp(name, plugin_s2d.blk_names[i]))
			return adv_tracer_s2d_set_blk_by_idx(enabled, i);
	}

	return -ENOENT;
}

int adv_tracer_s2d_set_all_blk(bool en)
{
	struct adv_tracer_ipc_cmd cmd = { 0 };
	int ret = 0;

	cmd.cmd_raw.cmd = eS2D_IPC_CMD_SET_ALL_BLK;
	cmd.buffer[1] = en;
	ret = adv_tracer_ipc_send_data(plugin_s2d.s2d_dev->id, &cmd);
	if (ret < 0) {
		dev_err(plugin_s2d.dev, "cannot set all blk\n");
		return ret;
	}

	return 0;
}

static int adv_tracer_s2d_set_disable_gpr(int en)
{
	struct adv_tracer_ipc_cmd cmd = { 0 };
	int ret = 0;

	cmd.cmd_raw.cmd = eS2D_IPC_CMD_SET_DISABLE_GPR;
	cmd.buffer[1] = en;
	ret = adv_tracer_ipc_send_data(plugin_s2d.s2d_dev->id, &cmd);
	if (ret < 0) {
		dev_err(plugin_s2d.dev, "ipc can't set disable_gpr (rc = %d)\n", ret);
		return ret;
	}

	return 0;
}

static int adv_tracer_s2d_get_disable_gpr(void)
{
	struct adv_tracer_ipc_cmd cmd = { 0 };
	int ret = 0;

	cmd.cmd_raw.cmd = eS2D_IPC_CMD_GET_DISABLE_GPR;
	ret = adv_tracer_ipc_send_data(plugin_s2d.s2d_dev->id, &cmd);
	if (ret < 0) {
		dev_err(plugin_s2d.dev, "ipc can't get disable_gpr (rc = %d)\n", ret);
		return ret;
	}

	return cmd.buffer[1];
}

static void adv_tracer_s2d_handler(struct adv_tracer_ipc_cmd *cmd,
		unsigned int len)
{
}

static ssize_t s2d_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	adv_tracer_s2d_set_enable(!!val);

	return size;
}

static ssize_t s2d_enable_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%sable\n",
			plugin_s2d.enable ? "en" : "dis");
}

static DEVICE_ATTR_RW(s2d_enable);

static ssize_t enable_block_by_name_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	char *name;
	int ret = -EINVAL;

	name = devm_kzalloc(dev, size + 1, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	if (sscanf(buf, "%s", name) == 1)
		ret = adv_tracer_s2d_set_blk_by_name(true, name);

	devm_kfree(dev, name);
	return ret ? ret : size;
}

static DEVICE_ATTR_WO(enable_block_by_name);

static ssize_t disable_block_by_name_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	char *name;
	int ret = -EINVAL;

	name = devm_kzalloc(dev, size + 1, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	if (sscanf(buf, "%s", name) == 1)
		ret = adv_tracer_s2d_set_blk_by_name(false, name);

	devm_kfree(dev, name);
	return ret ? ret : size;
}

static DEVICE_ATTR_WO(disable_block_by_name);

static ssize_t enable_block_by_index_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	int ret = 0;
	unsigned int index;

	ret = kstrtouint(buf, 10, &index);
	if (ret < 0)
		return ret;

	ret = adv_tracer_s2d_set_blk_by_idx(true, index);

	return ret ? ret : size;
}

static DEVICE_ATTR_WO(enable_block_by_index);

static ssize_t disable_block_by_index_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t size)
{
	int ret = 0;
	unsigned int index;

	ret = kstrtouint(buf, 10, &index);
	if (ret < 0)
		return ret;

	ret = adv_tracer_s2d_set_blk_by_idx(false, index);

	return ret ? ret : size;
}

static DEVICE_ATTR_WO(disable_block_by_index);

static ssize_t switch_all_block_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int ret = 0;
	unsigned int all;

	ret = kstrtouint(buf, 10, &all);
	if (ret < 0)
		return ret;

	ret = adv_tracer_s2d_set_all_blk(!!all);

	return ret ? ret : size;
}

static DEVICE_ATTR_WO(switch_all_block);

static ssize_t print_all_block_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned int i, sz = 0;
	unsigned long blocks;
	int ret;

	ret = adv_tracer_s2d_get_all_blk(&blocks);

	if (ret)
		return ret;

	for (i = 0; i < plugin_s2d.blk_count; i++) {
		sz += scnprintf(buf + sz, PAGE_SIZE - sz, "[%02u : %3s] %s\n",
				i, blocks & BIT(i) ? "on" : "off",
				plugin_s2d.blk_names[i]);
	}

	sz += scnprintf(buf + sz, PAGE_SIZE - sz, "\n");

	return sz;
}

static DEVICE_ATTR_RO(print_all_block);

static ssize_t disable_gpr_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	ret = adv_tracer_s2d_set_disable_gpr(!!val);
	if (ret)
		return ret;

	return size;
}

static ssize_t disable_gpr_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int ret;

	ret = adv_tracer_s2d_get_disable_gpr();
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%s\n", ret ? "Disabled" : "Not disabled");
}

static DEVICE_ATTR_RW(disable_gpr);

static struct attribute *adv_tracer_s2d_sysfs_attrs[] = {
	&dev_attr_s2d_enable.attr,
	&dev_attr_enable_block_by_name.attr,
	&dev_attr_enable_block_by_index.attr,
	&dev_attr_disable_block_by_name.attr,
	&dev_attr_disable_block_by_index.attr,
	&dev_attr_switch_all_block.attr,
	&dev_attr_print_all_block.attr,
	&dev_attr_disable_gpr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(adv_tracer_s2d_sysfs);

static int adv_tracer_s2d_dt_init(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;

	if (of_property_read_u32(node, "pmu-burnin-ctrl",
				&plugin_s2d.burnin_ctrl))
		dev_err(&pdev->dev, "pmu-burnin-ctrl is no data\n");
	if (of_property_read_u32(node, "sel-scanmode-bit",
				&plugin_s2d.sel_scanmode))
		plugin_s2d.sel_scanmode = -1;
	if (of_property_read_u32(node, "dbgsel-sw-bit",
				&plugin_s2d.dbgsel_sw))
		plugin_s2d.dbgsel_sw = -1;

	plugin_s2d.blk_count = of_property_count_strings(node, "blk-list");
	if (plugin_s2d.blk_count <= 0 || plugin_s2d.blk_count > 64) {
		dev_err(&pdev->dev, "Failed to get blk list.\n");
		return -EINVAL;
	}

	plugin_s2d.blk_names = devm_kcalloc(&pdev->dev, plugin_s2d.blk_count,
			sizeof(char *), GFP_KERNEL);
	if (!plugin_s2d.blk_names)
		return -ENOMEM;

	of_property_read_string_array(node, "blk-list",
			plugin_s2d.blk_names, plugin_s2d.blk_count);

	return 0;
}

static int adv_tracer_s2d_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct adv_tracer_plugin *s2d = NULL;
	int ret;

	s2d = devm_kzalloc(&pdev->dev, sizeof(struct adv_tracer_plugin),
			GFP_KERNEL);
	if (!s2d) {
		dev_err(&pdev->dev, "can not allocate mem for s2d\n");
		ret = -ENOMEM;
		goto err_s2d_info;
	}

	plugin_s2d.dev = &pdev->dev;
	plugin_s2d.s2d_dev = s2d;

	ret = adv_tracer_ipc_request_channel(node, adv_tracer_s2d_handler,
			&s2d->id, &s2d->len);
	if (ret < 0) {
		dev_err(&pdev->dev, "s2d ipc request fail(%d)\n", ret);
		goto err_sysfs_probe;
	}

	ret = adv_tracer_s2d_get_enable();
	if (ret < 0)
		goto err_sysfs_probe;
	dev_info(&pdev->dev, "S2D %sabled\n", plugin_s2d.enable ? "en" : "dis");
	ret = adv_tracer_s2d_dt_init(pdev);
	if (ret < 0)
		goto err_sysfs_probe;

	platform_set_drvdata(pdev, s2d);
	ret = sysfs_create_groups(&pdev->dev.kobj, adv_tracer_s2d_sysfs_groups);
	if (ret) {
		dev_err(&pdev->dev, "fail to register sysfs.\n");
		return ret;
	}

	dbg_snapshot_register_debug_ops(NULL,
					adv_tracer_s2d_arraydump,
					adv_tracer_s2d_scandump);

	keydebug_register_s2d_ops(adv_tracer_s2d_get_enable, adv_tracer_s2d_set_enable);

	dev_info(&pdev->dev, "%s successful.\n", __func__);
	return 0;

err_sysfs_probe:
	devm_kfree(&pdev->dev, s2d);
err_s2d_info:
	return ret;
}

static int adv_tracer_s2d_remove(struct platform_device *pdev)
{
	struct adv_tracer_plugin *s2d = platform_get_drvdata(pdev);

	adv_tracer_ipc_release_channel(s2d->id);

	return 0;
}

static const struct of_device_id adv_tracer_s2d_match[] = {
	{ .compatible = "google,exynos-adv-tracer-s2d", },
	{},
};
MODULE_DEVICE_TABLE(of, adv_tracer_s2d_match);

static struct platform_driver adv_tracer_s2d_driver = {
	.probe          = adv_tracer_s2d_probe,
	.remove         = adv_tracer_s2d_remove,
	.driver         = {
		.name   = "exynos-adv-tracer-s2d",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(adv_tracer_s2d_match),
	},
};
module_platform_driver(adv_tracer_s2d_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("EXYNOS Advanced Tracer for S2D Driver");
