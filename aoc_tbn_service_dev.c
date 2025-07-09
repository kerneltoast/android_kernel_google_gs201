/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright 2020 Google LLC. All Rights Reserved.
 *
 * aoc service to send cmds to aoc
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>

#include "aoc.h"
#include "aoc-interface.h"
#include "aoc_tbn_service_dev.h"

#define AOC_TBN_SERVICE_DEV_NAME "aoc_tbn_sdev"
#define AOC_SERVICE_NAME "com.google.tbn_service"
#define SEND_TIMEOUT_JIFFY (200)

static struct aoc_service_dev *aoc_tbn_service = NULL;
static const char * const service_names[] = {
	AOC_SERVICE_NAME,
	NULL,
};

static int aoc_tbn_service_probe(struct aoc_service_dev *sd)
{
	struct device *dev = &sd->dev;

	dev_dbg(dev, "probe service sd=%p\n", sd);
	aoc_tbn_service = sd;
	return 0;
}

static int aoc_tbn_service_remove(struct aoc_service_dev *sd)
{
	aoc_tbn_service = NULL;
	return 0;
}

static struct aoc_driver aoc_tbn_sdev = {
	.drv = {
		.name = AOC_TBN_SERVICE_DEV_NAME,
	},
	.service_names = service_names,
	.probe = aoc_tbn_service_probe,
	.remove = aoc_tbn_service_remove,
};

static int __init aoc_tbn_service_init(void)
{
	aoc_driver_register(&aoc_tbn_sdev);
	return 0;
}

static void __exit aoc_tbn_service_exit(void)
{
	aoc_driver_unregister(&aoc_tbn_sdev);
}

ssize_t aoc_tbn_service_write(void *cmd, size_t size)
{
	return aoc_service_write_timeout(aoc_tbn_service, cmd, size, SEND_TIMEOUT_JIFFY);
}
EXPORT_SYMBOL_GPL(aoc_tbn_service_write);

ssize_t aoc_tbn_service_read(void *cmd, size_t size)
{
	return aoc_service_read(aoc_tbn_service, cmd, size, true);
}
EXPORT_SYMBOL_GPL(aoc_tbn_service_read);

bool aoc_tbn_service_ready(void)
{
	return (aoc_tbn_service != NULL);
}
EXPORT_SYMBOL_GPL(aoc_tbn_service_ready);

module_init(aoc_tbn_service_init);
module_exit(aoc_tbn_service_exit);

MODULE_LICENSE("GPL v2");
