// SPDX-License-Identifier: GPL-2.0-only
/* vh_i2c.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <trace/hooks/i2c.h>
#include <linux/i2c.h>
#include <linux/module.h>

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/

/*
 * This part of code is vendor hook functions, which modify or extend the
 * original functions.
 */
static void vh_of_i2c_get_board_info_mod(void *data, struct device_node *node,
					 const char **dev_name)
{
	of_property_read_string(node, "dev-name", dev_name);
}

static int vh_i2c_init(void)
{
	return register_trace_android_vh_of_i2c_get_board_info(vh_of_i2c_get_board_info_mod,
								NULL);
}

module_init(vh_i2c_init);
MODULE_LICENSE("GPL v2");
