// SPDX-License-Identifier: GPL-2.0-only
/*
 * pixel-debug-test.c
 *
 * Utility module to trigger various intentional errors.
 *
 * Copyright (C) 2019 Google LLC.
 */

#define pr_fmt(fmt) "PIXEL DEBUG TEST: %s() " fmt, __func__

#include <linux/init.h>
#include <linux/module.h>

/*
 * Module init and exit
 */
static int __init pixel_debug_test_init(void)
{
	pr_info("initialized!");
	return 0;
}

static void __exit pixel_debug_test_exit(void)
{
	pr_info("module exit!");
}

module_init(pixel_debug_test_init);
module_exit(pixel_debug_test_exit);

MODULE_DESCRIPTION("Module to trigger intentional errors.");
MODULE_AUTHOR("Siyuan Zhou <siyuanzhou@google.com>");
MODULE_LICENSE("GPL v2");
