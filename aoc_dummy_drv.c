// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2019 Google LLC. All Rights Reserved.
 *
 * Character device interface for AoC services
 * ACD - AoC Character Device
 */

#define pr_fmt(fmt) "aoc_dummy: " fmt

#include <linux/module.h>

#include "aoc.h"

#define AOC_DUMMY_NAME "aoc_dummy"

/* Driver methods */
static int aoc_dummy_probe(struct aoc_service_dev *dev);
static int aoc_dummy_remove(struct aoc_service_dev *dev);

const char * const service_names[] = {
	"com.google.dummy*",
	"dummy",
	NULL,
};

static struct aoc_driver aoc_dummy_driver = {
	.drv = {
			.name = AOC_DUMMY_NAME,
		},
	.service_names = service_names,
	.probe = aoc_dummy_probe,
	.remove = aoc_dummy_remove,
};

static int aoc_dummy_probe(struct aoc_service_dev *dev)
{
	pr_notice("probe service with name %s\n", dev_name(&dev->dev));

	return 0;
}

static int aoc_dummy_remove(struct aoc_service_dev *dev)
{
	pr_notice("remove service with name %s\n", dev_name(&dev->dev));

	return 0;
}

module_aoc_driver(aoc_dummy_driver);

MODULE_LICENSE("GPL v2");
