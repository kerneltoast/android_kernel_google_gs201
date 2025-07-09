/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2024 Google LLC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#include <linux/debugfs.h>
#include <linux/platform_device.h>

#include "gs_panel/gs_panel.h"

struct gs_panel_register {
	char *name;
	u8 address;
	int size;
	bool read_individually;
	const struct gs_dsi_cmdset *pre_read_cmdset;
	const struct gs_dsi_cmdset *post_read_cmdset;
};

struct gs_panel_registers_desc {
	int register_count;
	const struct gs_panel_register *registers;

	const struct gs_dsi_cmdset *global_pre_read_cmdset;
	const struct gs_dsi_cmdset *global_post_read_cmdset;
};

struct gs_panel_test;

struct gs_panel_test_funcs {
	/**
	 * @debugfs_init:
	 *
	 * Allows panel tests to create panels-specific debugfs files.
	 */
	void (*debugfs_init)(struct gs_panel_test *test, struct dentry *test_root);
};

struct gs_panel_test;

struct gs_panel_query_funcs {
	/**
	 * @get_refresh_rate get current refresh rate from register logic
	 *
	 * Returns current refresh rate
	 */
	int (*get_refresh_rate)(struct gs_panel_test *gs_panel);

	/**
	 * @get_irc_on : get irc status from register read
	 *
	 * Returns current irc status
	 */
	int (*get_irc_on)(struct gs_panel_test *gs_panel);

	/**
	 * @get_aod_on: read aod status from register
	 *
	 * Returns aod status
	 */
	int (*get_aod_on)(struct gs_panel_test *gs_panel);
};

struct gs_panel_test_desc {
	const struct gs_panel_test_funcs *test_funcs;
	const struct gs_panel_registers_desc *regs_desc;
	const struct gs_panel_query_funcs *query_desc;
};

struct gs_panel_test {
	struct gs_panel *ctx;
	struct device *dev;
	const struct gs_panel_test_desc *test_desc;
};

/**
 * @gs_panel_read_register_value() reads the register and store it in value
 *
 * @test: handle for gs_panel_test
 * @reg: register
 * @value: array to store the read value
 * Returns 0 for success
 */
int gs_panel_read_register_value(struct gs_panel_test *test, const struct gs_panel_register *reg,
				 u8 *value);

int gs_panel_test_common_init(struct platform_device *pdev, struct gs_panel_test *test);
int gs_panel_test_common_remove(struct platform_device *pdev);

#define GS_PANEL_REG(_name, _address)                         \
	{                                                     \
		.name = _name, .address = _address, .size = 1 \
	}

#define GS_PANEL_REG_LONG(_name, _address, _size)                 \
	{                                                         \
		.name = _name, .address = _address, .size = _size \
	}

#define GS_PANEL_REG_WITH_CMDS(_name, _address, _cmdset)                                  \
	{                                                                                 \
		.name = _name, .address = _address, .size = 1, .pre_read_cmdset = _cmdset \
	}

#define GS_PANEL_REG_LONG_WITH_CMDS(_name, _address, _size, _cmdset)                          \
	{                                                                                     \
		.name = _name, .address = _address, .size = _size, .pre_read_cmdset = _cmdset \
	}

#define gs_panel_test_has_debugfs_init(test)                             \
	((test) && (test->test_desc) && (test->test_desc->test_funcs) && \
	 (test->test_desc->test_funcs->debugfs_init))

#define gs_panel_test_has_registers_desc(test) \
	((test) && (test->test_desc) && (test->test_desc->regs_desc))

#define gs_panel_test_has_query_func(test) \
	((test) && (test->test_desc) && (test->test_desc->query_desc))

#define gs_panel_test_get_global_pre_read_cmds(test)                  \
	(gs_panel_test_has_registers_desc(test) ?                     \
		 test->test_desc->regs_desc->global_pre_read_cmdset : \
		 NULL)

#define gs_panel_test_get_global_post_read_cmds(test)                  \
	(gs_panel_test_has_registers_desc(test) ?                      \
		 test->test_desc->regs_desc->global_post_read_cmdset : \
		 NULL)
