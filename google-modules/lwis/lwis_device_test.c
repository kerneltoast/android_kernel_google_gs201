// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Test Device Driver
 *
 * Copyright (c) 2022 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-test-dev: " fmt

#include "lwis_device_test.h"

#include <linux/module.h>
#include <linux/platform_device.h>

#include "lwis_commands.h"
#include "lwis_device.h"
#include "lwis_platform.h"

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_DRIVER_NAME "lwis-test"

static int lwis_test_device_enable(struct lwis_device *lwis_dev);
static int lwis_test_device_disable(struct lwis_device *lwis_dev);
static int lwis_test_register_io(struct lwis_device *lwis_dev, struct lwis_io_entry *entry,
				 int access_size);

static struct lwis_device_subclass_operations test_vops = {
	.register_io = lwis_test_register_io,
	.register_io_barrier = NULL,
	.device_enable = lwis_test_device_enable,
	.device_disable = lwis_test_device_disable,
	.event_enable = NULL,
	.event_flags_updated = NULL,
	.close = NULL,
};

static int lwis_test_device_enable(struct lwis_device *lwis_dev)
{
	return 0;
}

static int lwis_test_device_disable(struct lwis_device *lwis_dev)
{
	return 0;
}

static int lwis_test_register_io(struct lwis_device *lwis_dev, struct lwis_io_entry *entry,
				 int access_size)
{
	struct lwis_test_device *test_dev =
		container_of(lwis_dev, struct lwis_test_device, base_dev);

	if (!entry) {
		dev_err(test_dev->base_dev.dev, "IO entry is NULL.\n");
		return -EINVAL;
	}

	lwis_save_register_io_info(lwis_dev, entry, access_size);

	switch (entry->type) {
	case LWIS_IO_ENTRY_READ:
	case LWIS_IO_ENTRY_READ_V2: {
		struct lwis_io_entry_rw *rw;

		rw = &entry->rw;
		if (rw->offset >= SCRATCH_TEST_DEV_MEMORY_SIZE) {
			dev_err(test_dev->base_dev.dev, "Offset (%llu) must be < %d\n", rw->offset,
				SCRATCH_TEST_DEV_MEMORY_SIZE);
			return -EINVAL;
		}
		rw->val = test_dev->scratch_mem[rw->offset];
		break;
	}
	case LWIS_IO_ENTRY_READ_BATCH:
	case LWIS_IO_ENTRY_READ_BATCH_V2: {
		struct lwis_io_entry_rw_batch *rw_batch;

		rw_batch = &entry->rw_batch;
		if (rw_batch->offset + rw_batch->size_in_bytes >= SCRATCH_TEST_DEV_MEMORY_SIZE) {
			dev_err(test_dev->base_dev.dev,
				"Read range[offset(%llu) + size_in_bytes(%zu)] exceeds scratch memory (%d)\n",
				rw_batch->offset, rw_batch->size_in_bytes,
				SCRATCH_TEST_DEV_MEMORY_SIZE);
			return -EINVAL;
		}
		memcpy(rw_batch->buf, &test_dev->scratch_mem[rw_batch->offset],
		       rw_batch->size_in_bytes);
		break;
	}
	case LWIS_IO_ENTRY_WRITE:
	case LWIS_IO_ENTRY_WRITE_V2: {
		struct lwis_io_entry_rw *rw;

		rw = &entry->rw;
		if (rw->offset >= SCRATCH_TEST_DEV_MEMORY_SIZE) {
			dev_err(test_dev->base_dev.dev, "Offset (%llu) must be < %d\n", rw->offset,
				SCRATCH_TEST_DEV_MEMORY_SIZE);
			return -EINVAL;
		}
		test_dev->scratch_mem[rw->offset] = rw->val;
		break;
	}
	case LWIS_IO_ENTRY_WRITE_BATCH:
	case LWIS_IO_ENTRY_WRITE_BATCH_V2: {
		struct lwis_io_entry_rw_batch *rw_batch;

		rw_batch = &entry->rw_batch;
		if (rw_batch->offset + rw_batch->size_in_bytes >= SCRATCH_TEST_DEV_MEMORY_SIZE) {
			dev_err(test_dev->base_dev.dev,
				"Write range[offset(%llu) + size_in_bytes(%zu)] exceeds scratch memory (%d)\n",
				rw_batch->offset, rw_batch->size_in_bytes,
				SCRATCH_TEST_DEV_MEMORY_SIZE);
			return -EINVAL;
		}
		memcpy(&test_dev->scratch_mem[rw_batch->offset], rw_batch->buf,
		       rw_batch->size_in_bytes);
		break;
	}
	case LWIS_IO_ENTRY_MODIFY: {
		struct lwis_io_entry_modify *mod;
		uint64_t reg_value;

		mod = &entry->mod;
		if (mod->offset >= SCRATCH_TEST_DEV_MEMORY_SIZE) {
			dev_err(test_dev->base_dev.dev, "Offset (%llu) must be < %d\n", mod->offset,
				SCRATCH_TEST_DEV_MEMORY_SIZE);
			return -EINVAL;
		}
		reg_value = test_dev->scratch_mem[mod->offset];
		reg_value &= ~mod->val_mask;
		reg_value |= mod->val_mask & mod->val;
		test_dev->scratch_mem[mod->offset] = reg_value;
		break;
	}
	case LWIS_IO_ENTRY_IGNORE: {
		break;
	}
	default:
		dev_err(test_dev->base_dev.dev, "Invalid IO entry type: %d\n", entry->type);
		return -EINVAL;
	}

	return 0;
}

static int test_device_setup(struct lwis_test_device *test_dev)
{
	int ret = 0;

#ifdef CONFIG_OF
	/* Parse device tree for device configurations */
	ret = lwis_test_device_parse_dt(test_dev);
	if (ret)
		dev_err(test_dev->base_dev.dev, "Failed to parse device tree\n");
#else
	/* Non-device-tree init: Save for future implementation */
	ret = -EINVAL;
#endif

	return ret;
}

static int lwis_test_device_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct lwis_test_device *test_dev;
	struct device *dev = &plat_dev->dev;

	/* Allocate test device specific data construct */
	test_dev = devm_kzalloc(dev, sizeof(struct lwis_test_device), GFP_KERNEL);
	if (!test_dev)
		return -ENOMEM;

	test_dev->base_dev.type = DEVICE_TYPE_TEST;
	test_dev->base_dev.vops = test_vops;
	test_dev->base_dev.plat_dev = plat_dev;
	test_dev->base_dev.k_dev = &plat_dev->dev;

	/* Call the base device probe function */
	ret = lwis_base_probe(&test_dev->base_dev);
	if (ret) {
		dev_err(dev, "TEST device: Error in lwis base probe: %d\n", ret);
		return ret;
	}
	platform_set_drvdata(plat_dev, &test_dev->base_dev);

	/* Call TEST device specific setup function */
	ret = test_device_setup(test_dev);
	if (ret) {
		dev_err(test_dev->base_dev.dev, "Error in TEST device initialization\n");
		lwis_base_unprobe(&test_dev->base_dev);
		return ret;
	}

	dev_info(test_dev->base_dev.dev, "TEST Device Probe: Success\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lwis_id_match[] = {
	{ .compatible = LWIS_TEST_DEVICE_COMPAT },
	{},
};
// MODULE_DEVICE_TABLE(of, lwis_id_match);

static struct platform_driver lwis_driver = {
	.probe = lwis_test_device_probe,
	.driver = {
		.name = LWIS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lwis_id_match,
	},
};

#else /* CONFIG_OF not defined */
static struct platform_device_id lwis_driver_id[] = {
	{
		.name = LWIS_DRIVER_NAME,
		.driver_data = 0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, lwis_driver_id);

static struct platform_driver lwis_driver = { .probe = lwis_test_device_probe,
					      .id_table = lwis_driver_id,
					      .driver = {
						      .name = LWIS_DRIVER_NAME,
						      .owner = THIS_MODULE,
					      } };
#endif /* CONFIG_OF */

/*
 *  lwis_test_device_init: Init function that will be called by the kernel
 *  initialization routines.
 */
int __init lwis_test_device_init(void)
{
	int ret = 0;

	pr_info("TEST device initialization\n");

	ret = platform_driver_register(&lwis_driver);
	if (ret)
		pr_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

int lwis_test_device_deinit(void)
{
	platform_driver_unregister(&lwis_driver);
	return 0;
}
