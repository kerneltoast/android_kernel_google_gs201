// SPDX-License-Identifier: MIT
/*
 * Copyright 2024 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#include <linux/of_device.h>
#include <video/mipi_display.h>
#include "gs_panel/gs_panel_test.h"
#include "trace/panel_trace.h"

#define MAX_PANEL_REG_SIZE 128
#define MAX_VALUE_PER_LINE 10

/* Register dump nodes */

/* keep sorted by register address */
static struct gs_panel_register common_panel_registers[] = {
	GS_PANEL_REG("compression_mode", MIPI_DCS_GET_COMPRESSION_MODE),
	GS_PANEL_REG("display_id", MIPI_DCS_GET_DISPLAY_ID),
	GS_PANEL_REG("error_count_on_DSI", MIPI_DCS_GET_ERROR_COUNT_ON_DSI),
	GS_PANEL_REG("red_channel", MIPI_DCS_GET_RED_CHANNEL),
	GS_PANEL_REG("green_channel", MIPI_DCS_GET_GREEN_CHANNEL),
	GS_PANEL_REG("blue_channel", MIPI_DCS_GET_BLUE_CHANNEL),
	GS_PANEL_REG("display_status", MIPI_DCS_GET_DISPLAY_STATUS),
	GS_PANEL_REG("power_mode", MIPI_DCS_GET_POWER_MODE),
	GS_PANEL_REG("address_mode", MIPI_DCS_GET_ADDRESS_MODE),
	GS_PANEL_REG("pixel_format", MIPI_DCS_GET_PIXEL_FORMAT),
	GS_PANEL_REG("display_mode", MIPI_DCS_GET_DISPLAY_MODE),
	GS_PANEL_REG("signal_mode", MIPI_DCS_GET_SIGNAL_MODE),
	GS_PANEL_REG("diagnostic_result", MIPI_DCS_GET_DIAGNOSTIC_RESULT),

	GS_PANEL_REG("checksum_rgb", MIPI_DCS_GET_IMAGE_CHECKSUM_RGB),
	GS_PANEL_REG("checksum_ct", MIPI_DCS_GET_IMAGE_CHECKSUM_CT),

	GS_PANEL_REG("control_3d", MIPI_DCS_GET_3D_CONTROL),
	GS_PANEL_REG("scanline", MIPI_DCS_GET_SCANLINE),
	GS_PANEL_REG_LONG("brightness", MIPI_DCS_GET_DISPLAY_BRIGHTNESS, 2),
	GS_PANEL_REG("ctrld", MIPI_DCS_GET_CONTROL_DISPLAY),
	GS_PANEL_REG("power_save", MIPI_DCS_GET_POWER_SAVE),
	GS_PANEL_REG("cabc_min_br", MIPI_DCS_GET_CABC_MIN_BRIGHTNESS),
	GS_PANEL_REG_LONG("pps", MIPI_DCS_READ_PPS_START, 88),
	{ .name = "id", .address = 0xDA, .size = 3, .read_individually = true },
};

static void send_cmdset_to_panel(struct gs_panel *ctx, const struct gs_dsi_cmdset *cmds)
{
	if (cmds && cmds->num_cmd)
		gs_panel_send_cmdset(ctx, cmds);
}

int gs_panel_read_register_value(struct gs_panel_test *test, const struct gs_panel_register *reg,
				 u8 *value)
{
	struct gs_panel *ctx = test->ctx;
	struct device *dev = ctx->dev;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	int i, ret;

	if (reg->size > MAX_PANEL_REG_SIZE) {
		dev_warn(dev, "Register size is bigger than buffer size\n");
		return -E2BIG;
	}

	/* TODO: Optional for common registers */
	send_cmdset_to_panel(ctx, gs_panel_test_get_global_pre_read_cmds(test));
	send_cmdset_to_panel(ctx, reg->pre_read_cmdset);

	if (reg->read_individually) {
		for (i = 0; i < reg->size; i++) {
			ret = mipi_dsi_dcs_read(dsi, reg->address + i, value + i, 1);
			if (ret != 1)
				dev_warn(dev,
					 "Failed to read %s (0x%x) register. Returned value: %d\n",
					 reg->name, reg->address + i, ret);
			else
				ret = 0;
		}
	} else {
		ret = mipi_dsi_dcs_read(dsi, reg->address, value, reg->size);

		if (ret != (int)reg->size)
			dev_warn(dev, "Failed to read %s (0x%x) register. Returned value: %d\n",
				 reg->name, reg->address, ret);
		else
			ret = 0;
	}

	send_cmdset_to_panel(ctx, reg->post_read_cmdset);
	send_cmdset_to_panel(ctx, gs_panel_test_get_global_post_read_cmds(test));

	return ret;
}
EXPORT_SYMBOL_GPL(gs_panel_read_register_value);

static u8 reg_value[MAX_PANEL_REG_SIZE];

static void dump_register_to_debugfs(struct gs_panel_test *test,
				     const struct gs_panel_register *reg, struct seq_file *m)
{
	int cnt;

	gs_panel_read_register_value(test, reg, reg_value);
	if (reg->size <= MAX_VALUE_PER_LINE) {
		seq_printf(m, "%s (0x%02x) : %*ph\n", reg->name, reg->address, reg->size,
			   reg_value);
	} else {
		seq_printf(m, "%s (0x%02x) :\n", reg->name, reg->address);
		for (cnt = 0; cnt < reg->size; cnt += MAX_VALUE_PER_LINE) {
			seq_printf(m, "%*ph\n", min(MAX_VALUE_PER_LINE, (int)reg->size - cnt),
				   reg_value + cnt);
		}
	}
}

static int gs_panel_dump_registers_to_debugfs(struct gs_panel_test *test, struct seq_file *m)
{
	int i;
	const struct gs_panel_registers_desc *registers_desc;

	seq_puts(m, "MIPI\n----\n");

	for (i = 0; i < ARRAY_SIZE(common_panel_registers); i++)
		dump_register_to_debugfs(test, &common_panel_registers[i], m);

	if (!gs_panel_test_has_registers_desc(test))
		return 0;

	registers_desc = test->test_desc->regs_desc;

	seq_puts(m, "Panel specific\n--------------\n");
	for (i = 0; i < registers_desc->register_count; i++)
		dump_register_to_debugfs(test, &registers_desc->registers[i], m);

	return 0;
}

static int gs_panel_regs_show(struct seq_file *m, void *data)
{
	return gs_panel_dump_registers_to_debugfs(m->private, m);
}
DEFINE_SHOW_ATTRIBUTE(gs_panel_regs);

struct register_dentry_data {
	struct gs_panel_test *test;
	const struct gs_panel_register *reg;
};

static int gs_panel_reg_show(struct seq_file *m, void *data)
{
	struct register_dentry_data *reg_data = m->private;

	dump_register_to_debugfs(reg_data->test, reg_data->reg, m);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gs_panel_reg);

static int debugfs_add_panel_register_nodes(struct gs_panel_test *test, struct dentry *test_root)
{
	int i;
	struct dentry *regs_root;
	const struct gs_panel_registers_desc *regs_desc;

	regs_root = debugfs_create_dir("regs", test_root);
	if (!regs_root)
		return -EFAULT;

	debugfs_create_file("reg_values", 0600, test_root, test, &gs_panel_regs_fops);

	for (i = 0; i < ARRAY_SIZE(common_panel_registers); i++) {
		struct register_dentry_data *data =
			kmalloc(sizeof(struct register_dentry_data), GFP_KERNEL);
		data->test = test;
		data->reg = &common_panel_registers[i];
		debugfs_create_file(common_panel_registers[i].name, 0600, regs_root, data,
				    &gs_panel_reg_fops);
	}

	if (!gs_panel_test_has_registers_desc(test))
		return 0;

	regs_desc = test->test_desc->regs_desc;

	if (!regs_desc)
		return 0;

	for (i = 0; i < regs_desc->register_count; i++) {
		struct register_dentry_data *data =
			kmalloc(sizeof(struct register_dentry_data), GFP_KERNEL);
		data->test = test;
		data->reg = &regs_desc->registers[i];
		debugfs_create_file(regs_desc->registers[i].name, 0600, regs_root, data,
				    &gs_panel_reg_fops);
	}

	return 0;
}

/* Query nodes */

struct query_dentry_data {
	struct gs_panel_test *test;
	const char *query_node_name;
};

static int gs_panel_query_node_show(struct seq_file *m, void *data)
{
	struct query_dentry_data *query_data = m->private;
	struct gs_panel_test *test;
	const struct gs_panel_query_funcs *query_func;
	const char *name;

	if (!query_data)
		return -EFAULT;

	test = query_data->test;
	name = query_data->query_node_name;

	if (!gs_panel_test_has_query_func(test))
		return -EOPNOTSUPP;

	query_func = test->test_desc->query_desc;

#define MATCH_QUERY_FUNC_NAME(test, node_name)                                    \
	{                                                                         \
		if (!strcmp(name, #node_name) && query_func->get_##node_name) {   \
			seq_printf(m, "%d\n", query_func->get_##node_name(test)); \
			return 0;                                                 \
		}                                                                 \
	}

	MATCH_QUERY_FUNC_NAME(test, refresh_rate);
	MATCH_QUERY_FUNC_NAME(test, irc_on);
	MATCH_QUERY_FUNC_NAME(test, aod_on);

	return -EOPNOTSUPP;
}
DEFINE_SHOW_ATTRIBUTE(gs_panel_query_node);

static int debugfs_add_query_nodes(struct gs_panel_test *test, struct dentry *test_root)
{
	int i;
	struct dentry *query_root;
	static const char *const names[] = {
		"refresh_rate",
		"irc_on",
		"aod_on",
	};

	if (!gs_panel_test_has_query_func(test))
		return 0;

	query_root = debugfs_create_dir("query", test_root);
	if (!query_root)
		return -EFAULT;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		struct query_dentry_data *data =
			kmalloc(sizeof(struct register_dentry_data), GFP_KERNEL);
		data->test = test;
		data->query_node_name = names[i];

		debugfs_create_file(names[i], 0600, query_root, data, &gs_panel_query_node_fops);
	}

	return 0;
}

static int debugfs_add_test_folder(struct gs_panel_test *test)
{
	struct dentry *test_root, *panel_root = test->ctx->debugfs_entries.panel;
	int ret;

	if (!panel_root)
		return -EFAULT;

	test_root = debugfs_create_dir("test", panel_root);
	if (!test_root)
		return -EFAULT;

	ret = debugfs_add_panel_register_nodes(test, test_root);
	if (ret)
		return ret;

	ret = debugfs_add_query_nodes(test, test_root);
	if (ret)
		return ret;

	if (gs_panel_test_has_debugfs_init(test))
		test->test_desc->test_funcs->debugfs_init(test, test_root);

	return 0;
}

static int debugfs_remove_test_folder(struct gs_panel_test *test)
{
	struct dentry *panel_root, *test_root;

	panel_root = test->ctx->debugfs_entries.panel;
	if (!panel_root)
		return 0;

	test_root = debugfs_lookup("test", panel_root);
	if (!test_root)
		return 0;

	debugfs_remove_recursive(test_root);

	return 0;
}

int gs_panel_test_common_init(struct platform_device *pdev, struct gs_panel_test *test)
{
	struct device *dev = &pdev->dev;
	struct device *parent = dev->parent;
	struct gs_panel *ctx;

	if (!parent)
		return 0;

	ctx = dev_get_drvdata(parent);
	if (!ctx)
		return 0;

	PANEL_ATRACE_BEGIN("panel_test_init");

	test->ctx = ctx;
	test->dev = dev;
	test->test_desc = of_device_get_match_data(dev);
	dev_set_drvdata(dev, test);

#ifdef CONFIG_DEBUG_FS
	debugfs_add_test_folder(test);
#endif

	PANEL_ATRACE_END("panel_test_init");

	return 0;
}
EXPORT_SYMBOL_GPL(gs_panel_test_common_init);

int gs_panel_test_common_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs_panel_test *test = dev_get_drvdata(dev);

	if (!test)
		return 0;

	PANEL_ATRACE_BEGIN("panel_test_remove");
	debugfs_remove_test_folder(test);
	PANEL_ATRACE_END("panel_test_remove");

	return 0;
}
EXPORT_SYMBOL_GPL(gs_panel_test_common_remove);

MODULE_AUTHOR("Safayat Ullah <safayat@google.com>");
MODULE_DESCRIPTION("MIPI-DSI panel driver test abstraction for use across panel vendors");
MODULE_LICENSE("Dual MIT/GPL");
