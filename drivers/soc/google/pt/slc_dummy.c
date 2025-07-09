// SPDX-License-Identifier: GPL-2.0-only
/*
 * slc_dummy_c
 *
 * System cache management dummy driver.
 *
 * Copyright 2019 Google LLC
 *
 * Author: cozette@google.com
 */

#include <linux/list.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <soc/google/pt.h>

#define PT_PTID_MAX 64
#define PT_HIGHEST_BIT 32

static ptid_t slc_dummy_alloc(void *data,
				int property_index,
				void *resize_data,
				void (*resize)(void *data, size_t size));
static void slc_dummy_free(void *data, ptid_t ptid);
static void slc_dummy_enable(void *data, ptid_t ptid);
static void slc_dummy_disable(void *data, ptid_t ptid);
static ptid_t slc_dummy_mutate(void *data, ptid_t ptid,
				void *resize_data,
				int new_property_index);
static int slc_dummy_pbha(void *data, ptid_t ptid);

struct pt_ops slc_dummy_ops = {
	.alloc   = slc_dummy_alloc,
	.free    = slc_dummy_free,
	.enable  = slc_dummy_enable,
	.disable = slc_dummy_disable,
	.mutate  = slc_dummy_mutate,
	.pbha    = slc_dummy_pbha,
};

struct slc_dummy_driver_data {
	struct {
		u32 size_bits;
		int pbha;
		void *data;
		void (*resize)(void *data, size_t size);
		bool enabled;
		bool used;
	} ptids[PT_PTID_MAX];
	struct pt_driver *driver;
	struct platform_device *pdev;
};

static int slc_dummy_next_bit(u32 size_bits, int previous)
{
	if (previous < PT_HIGHEST_BIT)
		size_bits = size_bits - ((size_bits >> previous) << previous);
	else
		previous = PT_HIGHEST_BIT - 1;
	while ((previous >= 0) && ((size_bits >> previous) == 0))
		previous--;
	return previous;
}

static int slc_dummy_first_bit(u32 size_bits)
{
	return slc_dummy_next_bit(size_bits, PT_HIGHEST_BIT);
}

static int slc_dummy_bit_to_size(u32 bit)
{
	int result = 1 << (bit/2);

	if (bit == 0)
		return 0;
	if (bit & 1)
		result = result + result / 2;
	return result * 4096;
}

static ptid_t slc_dummy_alloc(void *data, int property_index, void *resize_data,
		void (*resize)(void *data, size_t size))
{
	struct slc_dummy_driver_data *driver_data =
		(struct slc_dummy_driver_data *)data;
	u32 vptid;
	u32 size_bits;
	u32 pbha;

	if (pt_driver_get_property_value(driver_data->driver,
		property_index, 0, &vptid) < 0)
		return PT_PTID_INVALID;
	if (pt_driver_get_property_value(driver_data->driver,
		property_index, 1, &size_bits) < 0)
		return PT_PTID_INVALID;
	if (pt_driver_get_property_value(driver_data->driver,
		property_index, 3, &pbha) < 0)
		pbha = 0;
	if (vptid >= PT_PTID_MAX)
		return PT_PTID_INVALID;
	if (driver_data->ptids[vptid].used)
		return PT_PTID_INVALID;
	driver_data->ptids[vptid].used = true;
	driver_data->ptids[vptid].data = resize_data;
	driver_data->ptids[vptid].size_bits = size_bits;
	driver_data->ptids[vptid].pbha = pbha;
	driver_data->ptids[vptid].resize = resize;
	return (ptid_t)vptid;
}

static ptid_t slc_dummy_mutate(void *data, ptid_t ptid, void *resize_data,
				int new_property_index)
{
	struct slc_dummy_driver_data *driver_data =
		(struct slc_dummy_driver_data *)data;
	u32 vptid;
	u32 size_bits;
	int size;

	if (pt_driver_get_property_value(driver_data->driver,
		new_property_index, 0, &vptid) < 0)
		return PT_PTID_INVALID;
	if (pt_driver_get_property_value(driver_data->driver,
		new_property_index, 1, &size_bits) < 0)
		return PT_PTID_INVALID;
	if (vptid != ptid)
		return PT_PTID_INVALID;
	size = slc_dummy_bit_to_size(slc_dummy_first_bit(size_bits));
	driver_data->ptids[vptid].size_bits = size_bits;
	driver_data->ptids[ptid].data = resize_data;
	driver_data->ptids[ptid].resize(resize_data, size);
	return ptid;
}

static ptpbha_t slc_dummy_pbha(void *data, ptid_t ptid)
{
	struct slc_dummy_driver_data *driver_data =
		(struct slc_dummy_driver_data *)data;

	if (!(driver_data->ptids[ptid].pbha & PT_PBHA_ENABLE))
		return PT_PBHA_INVALID;
	return (ptpbha_t)(driver_data->ptids[ptid].pbha & PT_PBHA_MASK);
}

static void slc_dummy_free(void *data, ptid_t ptid)
{
	struct slc_dummy_driver_data *driver_data =
				(struct slc_dummy_driver_data *)data;

	driver_data->ptids[ptid].used = false;
}

static void slc_dummy_enable(void *data, ptid_t ptid)
{
	struct slc_dummy_driver_data *driver_data =
		(struct slc_dummy_driver_data *)data;
	int size;
	u32 size_bits = driver_data->ptids[ptid].size_bits;

	driver_data->ptids[ptid].enabled = true;
	size = slc_dummy_bit_to_size(slc_dummy_first_bit(size_bits));
	driver_data->ptids[ptid].resize(driver_data->ptids[ptid].data, size);
}

static void slc_dummy_disable(void *data, ptid_t ptid)
{
	struct slc_dummy_driver_data *driver_data =
		(struct slc_dummy_driver_data *)data;

	driver_data->ptids[ptid].enabled = false;
	driver_data->ptids[ptid].resize(driver_data->ptids[ptid].data, 0);
}

static int slc_dummy_probe(struct platform_device *pdev)
{
	struct slc_dummy_driver_data *driver_data =
		kmalloc(sizeof(struct slc_dummy_driver_data), GFP_KERNEL);

	if (driver_data == NULL)
		return -ENOMEM;
	memset(driver_data, 0, sizeof(struct slc_dummy_driver_data));
	platform_set_drvdata(pdev, driver_data);
	driver_data->pdev = pdev;
	driver_data->driver =
		pt_driver_register(pdev->dev.of_node,
				   &slc_dummy_ops,
				   driver_data);
	return 0;
}

static const struct of_device_id slc_dummy_of_match_table[] = {
	{ .compatible = "google,pt", },
	{ },
};
MODULE_DEVICE_TABLE(of, slc_dummy_of_match_table);

static struct platform_driver slc_dummy_driver = {
	.probe = slc_dummy_probe,
	.driver = {
		.name = "google,slc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(slc_dummy_of_match_table),
	}
};

static int __init slc_dummy_init(void)
{
	return platform_driver_register(&slc_dummy_driver);
}

static void __exit slc_dummy_exit(void)
{
	platform_driver_unregister(&slc_dummy_driver);
}

module_init(slc_dummy_init);
module_exit(slc_dummy_exit);

MODULE_DESCRIPTION("SLC driver");
MODULE_AUTHOR("<cozette@google.com>");
MODULE_LICENSE("GPL");

// TODO: check license
// TODO: add unload
