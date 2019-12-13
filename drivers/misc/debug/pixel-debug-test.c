// SPDX-License-Identifier: GPL-2.0-only
/*
 * pixel-debug-test.c
 *
 * Utility module to trigger various intentional errors.
 *
 * Copyright (C) 2019 Google LLC.
 */

/*
 * When the module is loaded, it exposes the interface at
 * /sys/kernel/pixel_debug/trigger. Writing into this file with the
 * corresponding test name and parameters would trigger the
 * corresponding error.
 *
 * The test names are in the list of force_error_vector.errcmd.
 * E.g., echo panic>/sys/kernel/pixel_debug/trigger can trigger the
 * intentional panic.
 */

#define pr_fmt(fmt) "PIXEL DEBUG TEST: %s() " fmt, __func__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/fs.h>

/*
 * Error trigger functions
 */
static void simulate_panic(char *arg)
{
	pr_crit("called!\n");

	panic("simulate_panic");

	/* Should not reach here */
	pr_crit("failed!\n");
}

/*
 * Error trigger definitions
 */
typedef void (*force_error_func)(char *arg);

enum {
	FORCE_PANIC = 0,
	NR_FORCE_ERROR,
};

struct force_error_item {
	char errcmd[SZ_32];
	force_error_func errfunc;
};

static const struct force_error_item force_error_vector[NR_FORCE_ERROR] = {
	{ "panic",		&simulate_panic },
};

static void parse_and_trigger(const char *buf)
{
	int i;
	char *cmd;
	char *space = NULL, *param = NULL;
	int cmd_size = strlen(buf);

	pr_debug("cmd_size=%d, PAGE_SIZE=%d\n", cmd_size, PAGE_SIZE);
	/*
	 * Extract the command before the first space.
	 */
	space = strchr(buf, ' ');
	if (space != NULL) {
		cmd_size = space - buf;
		param = space + 1;
	}

	cmd = kstrndup(buf, cmd_size, GFP_KERNEL);
	if (!cmd)
		return;

	for (i = 0; i < NR_FORCE_ERROR; i++) {
		if (strcmp(cmd, force_error_vector[i].errcmd) != 0)
			continue;
		pr_debug("func param=%s\n", param);
		force_error_vector[i].errfunc(param);
		break;
	}

	if (i == NR_FORCE_ERROR)
		pr_crit("no trigger exists for command [%s]!\n", cmd);

	kfree(cmd);
}

/*
 * Sysfs structures
 */
static struct kobject *pixel_debug_kobj;
static char *trigger;

static void replace_newline_with_null(char *input)
{
	char *newline = strchr(input, '\n');

	if (newline != NULL)
		*newline = '\0';
}

static ssize_t trigger_write(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	pr_crit("count=%d, buf=%s", count, buf);
	strlcpy(trigger, buf, PAGE_SIZE);

	/*
	 * "echo" command appends a newline char by default. Replacing the
	 * newline char with '\0' because it is not part of the command.
	 */
	replace_newline_with_null(trigger);

	parse_and_trigger(trigger);
	return count;
}

static struct kobj_attribute debug_attr =
	__ATTR(trigger, 0220, NULL, trigger_write);

/*
 * Module init and exit
 */
static int __init pixel_debug_test_init(void)
{
	trigger = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!trigger)
		return -ENOMEM;

	pixel_debug_kobj = kobject_create_and_add("pixel_debug", kernel_kobj);
	if (!pixel_debug_kobj) {
		pr_err("cannot create kobj for pixel_debug!");
		goto error;
	}
	if (sysfs_create_file(pixel_debug_kobj, &debug_attr.attr)) {
		pr_err("cannot create file in pixel_debug!");
		kobject_put(pixel_debug_kobj);
		goto error;
	}
	pr_info("initialized!");
	return 0;
error:
	kfree(trigger);
	trigger = NULL;
	return -ENOMEM;
}

static void __exit pixel_debug_test_exit(void)
{
	kfree(trigger);
	trigger = NULL;

	kobject_put(pixel_debug_kobj);
	pr_info("module exit!");
}

module_init(pixel_debug_test_init);
module_exit(pixel_debug_test_exit);

MODULE_DESCRIPTION("Module to trigger intentional errors.");
MODULE_AUTHOR("Siyuan Zhou <siyuanzhou@google.com>");
MODULE_LICENSE("GPL v2");
