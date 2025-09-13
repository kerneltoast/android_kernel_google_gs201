/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2024 Google LLC. All Rights Reserved.
 *
 * aoc service to monitor unit test status
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>

#include "aoc.h"
#include "aoc-interface.h"

#define AOC_UNIT_TEST_SERVICE_DEV_NAME "aoc_unit_test_dev"
#define AOC_SERVICE_NAME "unit_test"

enum AOC_UNIT_TEST_STATUS {
	AOC_UNIT_TEST_RUNNING,
	AOC_UNIT_TEST_FAILED,
	AOC_UNIT_TEST_PASSED
};

static struct aoc_service_dev *aoc_unit_test_service = NULL;
static const char * const service_names[] = {
	AOC_SERVICE_NAME,
	NULL,
};

static enum AOC_UNIT_TEST_STATUS unit_test_status = AOC_UNIT_TEST_RUNNING;

static ssize_t unit_test_status_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	static const char *status[3] = { "RUNNING", "FAILED", "PASSED" };
	return scnprintf(buf, sizeof(status[unit_test_status]), "%s\n", status[unit_test_status]);
}

static DEVICE_ATTR_RO(unit_test_status);

static ssize_t unit_test_reset_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	unit_test_status = AOC_UNIT_TEST_RUNNING;
	return 1;
}

static DEVICE_ATTR_WO(unit_test_reset);

static struct attribute *aoc_unit_test_attrs[] = {
	&dev_attr_unit_test_status.attr,
	&dev_attr_unit_test_reset.attr,
	NULL
};

ATTRIBUTE_GROUPS(aoc_unit_test);

static void handle_aoc_msg(struct aoc_service_dev *dev)
{
	char buf[64];
	int ret;

	ret = aoc_service_read(dev, buf, 64, true);
	if (ret < 0) {
		dev_err(&dev->dev, "Read failed with %d\n", ret);
		return;
	}

	if (strcmp(buf, "PASSED") == 0) {
		unit_test_status = AOC_UNIT_TEST_PASSED;
		dev_info(&dev->dev, "Unit tests passed!\n");
	} else if (strcmp(buf, "FAILED") == 0) {
		unit_test_status = AOC_UNIT_TEST_FAILED;
		dev_info(&dev->dev, "Unit tests failed!\n");
	}

	return;
}

static int aoc_unit_test_service_probe(struct aoc_service_dev *sd)
{
	struct device *dev = &sd->dev;
	int ret;

	aoc_unit_test_service = sd;

	ret = device_add_groups(dev, aoc_unit_test_groups);
	if (ret)
		dev_err(dev, "Failed to add unit test attributes device groups\n");

	sd->handler = handle_aoc_msg;

	return 0;
}

static int aoc_unit_test_service_remove(struct aoc_service_dev *sd)
{
	struct device *dev = &sd->dev;

	aoc_unit_test_service = NULL;
	device_remove_groups(dev, aoc_unit_test_groups);
	return 0;
}

static struct aoc_driver aoc_unit_test_sdev = {
	.drv = {
		.name = AOC_UNIT_TEST_SERVICE_DEV_NAME,
	},
	.service_names = service_names,
	.probe = aoc_unit_test_service_probe,
	.remove = aoc_unit_test_service_remove,
};

static int __init aoc_unit_test_service_init(void)
{
	aoc_driver_register(&aoc_unit_test_sdev);
	return 0;
}

static void __exit aoc_unit_test_service_exit(void)
{
	aoc_driver_unregister(&aoc_unit_test_sdev);
}

module_init(aoc_unit_test_service_init);
module_exit(aoc_unit_test_service_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AoC Unit Test Driver");
MODULE_AUTHOR("Alex Iacobucci <alexiacobucci@google.com>");