// SPDX-License-Identifier: GPL-2.0-only
/*
 * slc_acpm_c
 *
 * System cache management acpm driver.
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
#include "pt_trace.h"
#include "slc_pmon.h"
#include "../cal-if/acpm_dvfs.h"
#include <soc/google/acpm_ipc_ctrl.h>
#include <linux/errno.h>
#include <linux/debugfs.h>

#define PT_PTID_MAX 64

#define PT_ENABLE       0xd001
#define PT_DISABLE      0xd002
#define PT_CHECK        0xd003
#define PT_MUTATE       0xd004
#define PT_VERSION      0xd005
#define PT_VERSION_KNOWN 0x1
#define PT_VERSION_ASYNC 0x10008
#define PT_VERSION_INVALID 0x0

#define SLC_PLDTRC 0x6a
#define SLC_STOP  0x6b

#ifdef CONFIG_SOC_ZUMA
#define SLC_SIZE_MULTIPLIER 8
#else
#define SLC_SIZE_MULTIPLIER 4
#endif

enum pt_property_index {
	PT_PROPERTY_INDEX_VPTID = 0,
	PT_PROPERTY_INDEX_SIZE_BITS = 1, // Allowed size
	PT_PROPERTY_INDEX_PRIORITY = 2,
	PT_PROPERTY_INDEX_PBHA = 3
};

static ptid_t slc_acpm_alloc(void *data, int property_index, void *resize_data,
		void (*resize)(void *data, size_t size));
static void slc_acpm_free(void *data, ptid_t ptid);
static void slc_acpm_enable(void *data, ptid_t ptid);
static void slc_acpm_disable(void *data, ptid_t ptid);
static int slc_acpm_mutate(void *data, ptid_t ptid, void *resize_data,
	int new_property_index);
static int slc_acpm_pbha(void *data, ptid_t ptid);

static int slc_acpm_ioctl(void *data, int arg_cnt, int *args);

struct pt_ops slc_acpm_ops = {
	.alloc = slc_acpm_alloc,
	.free = slc_acpm_free,
	.enable = slc_acpm_enable,
	.disable = slc_acpm_disable,
	.mutate = slc_acpm_mutate,
	.pbha = slc_acpm_pbha,
	.ioctl = slc_acpm_ioctl,
};

struct slc_acpm_driver_data {
	struct {
		u32 size_bits; /* Allowed size definition known by FVP */
		int pbha;
		int vptid;
		void *data;
		void (*resize)(void *data, size_t size);
	} ptids[PT_PTID_MAX];
	struct pt_driver *driver;
	struct platform_device *pdev;
	bool inited; /* allow delaying init to first pt call */
	int version; /* ACPM FVP version */

	/* Synchronous command */
	unsigned int id; /* ACPM FVP_PT channel id */
	unsigned int size; /* ACPM FVP_PT channel queue sizes */

	/* Asynchronous notification from ACPM */
	unsigned int async_id; /* ACPM FVP_PT_ASYNC channel id */
	unsigned int async_size; /* ACPM VPT_PT_ASYNC channel queue sizes */

	spinlock_t sl;
};

/*
 * Needed because acpm_ipc_callback doesn't give anything to find data.
 */
static struct slc_acpm_driver_data *slc_acpm_driver_data;

/*
 * Internal helper functions
 */


/*
 * Extact ptid/data from acpm reply
 */
static void pt_ptid_data_decode(int ptid_data, ptid_t *ptid, int *data)
{
	if (ptid_data < 0) {
		*ptid = -1;
		*data = -1;
	} else {
		*ptid = ptid_data >> 22;
		*data = ptid_data & 0x3fffff;
	}
}

/*
 * Send a command to APM and get back the return value.
 */
static int slc_acpm(struct slc_acpm_driver_data *driver_data,
		    unsigned int command, unsigned int arg, unsigned long arg1,
		    uint32_t *opt_buffer)
{
	struct ipc_config config;
	unsigned int cmd[8];
	int ret;

	config.cmd = opt_buffer ? opt_buffer : cmd;
	config.response = true;
	config.cmd[0] = 0;
	config.cmd[1] = arg;
	config.cmd[2] = command;
	config.cmd[3] = arg1;

	ret = acpm_ipc_send_data(driver_data->id, &config);
	if (ret == 0)
		ret = config.cmd[1];
	pt_driver_log_module(driver_data->pdev->dev.of_node->name, __func__,
			    command, arg, arg1, 0, ret,
			    ((u64)config.cmd[2]) << 32 | config.cmd[3],
			    ((u64)config.cmd[4]) << 32 | config.cmd[5],
			    ((u64)config.cmd[6]) << 32 | config.cmd[7]);
	return ret;
}

static void slc_acpm_ipc_callback(unsigned int *cmd, unsigned int size);

/*
 * Get communication channel with APM and check if the ACPM version is good
 */
static bool slc_version_check(struct slc_acpm_driver_data *driver_data)
{
	/*
	 * version = (MAJOR << 16) | MINOR.
	 * A change of MAJOR means no more compatible, a change of
	 * MINOR stay compatble.
	 * PT_VERSION_KNOWN is the MAJOR expected to follow the protocol
	 * of this driver.
	 */
	struct device_node *sub_node;

	if ((driver_data->inited)
		&& ((driver_data->version >> 16) == PT_VERSION_KNOWN))
		return true;
	if ((driver_data->inited)
		&& ((driver_data->version >> 16) != PT_VERSION_KNOWN))
		return false;
	driver_data->inited = true;
	driver_data->version = PT_VERSION_INVALID;
	if (acpm_ipc_request_channel(driver_data->pdev->dev.of_node,
					NULL, &driver_data->id,
					&driver_data->size) < 0) {
		dev_err(&driver_data->pdev->dev, "Can't get acpm channel\n");
		return false;
	}
	dev_info(&driver_data->pdev->dev, "channel %d\n", driver_data->id);
	driver_data->version = slc_acpm(driver_data, PT_VERSION, 0, 0, NULL);
	if ((driver_data->version >> 16) != PT_VERSION_KNOWN) {
		dev_err(&driver_data->pdev->dev,
			"found invalid acpm firmware %d.%d.\n",
			driver_data->version >> 16,
			driver_data->version & 0xffff);
		return false;
	}
	driver_data->inited = true;
	dev_info(&driver_data->pdev->dev,
		"found valid acpm firmware %d.%d.\n",
		driver_data->version >> 16,
		driver_data->version & 0xffff);

	sub_node = of_find_node_by_name(driver_data->pdev->dev.of_node,
					"async");
	if (IS_ERR(sub_node) || (driver_data->version < PT_VERSION_ASYNC)) {
		dev_err(&driver_data->pdev->dev, "No asynchronous node");
		return true;
	}

	slc_acpm_driver_data = driver_data;
	if (acpm_ipc_request_channel(sub_node, slc_acpm_ipc_callback,
					&driver_data->async_id,
					&driver_data->async_size) < 0) {
		dev_err(&driver_data->pdev->dev, "No asynchronous channel");
		return true;
	}
	dev_info(&driver_data->pdev->dev,
			"Asynchronous notification enabled");
	return true;
}

/*
 * Check if any partition size changed
 */
static void slc_acpm_check(struct slc_acpm_driver_data *driver_data)
{
	int ret;
	ptid_t ptid;
	int sizeReply;
	unsigned long flags;

	while ((ret = slc_acpm(driver_data, PT_CHECK, 0, 0, NULL)) > 0) {
		pt_ptid_data_decode(ret, &ptid, &sizeReply);
		if ((ptid >= PT_PTID_MAX) || (ptid < 0)) {
			dev_err(&driver_data->pdev->dev,
				"wrong ptid %d size %dK\n",
				ptid, SLC_SIZE_MULTIPLIER * sizeReply);
			/* An out-of-range PTID could be a sign of an ACPM
			 * protocol error (e.g. ACPM crash or protocol version
			 * mismatch). Break out of this loop to avoid a
			 * potential infinite loop.
			 */
			break;
		}
		dev_dbg(&driver_data->pdev->dev,
			 "ptid %d size %dK\n",
			 ptid, SLC_SIZE_MULTIPLIER * sizeReply);
		if (!driver_data->ptids[ptid].resize) {
			/*
			 * ptid was freed
			 */
			continue;
		}
		spin_lock_irqsave(&driver_data->sl, flags);
		if (driver_data->ptids[ptid].resize)
			driver_data->ptids[ptid].resize(
				driver_data->ptids[ptid].data,
				SLC_SIZE_MULTIPLIER * 1024 * sizeReply);
		spin_unlock_irqrestore(&driver_data->sl, flags);
	}
}

/*
 * Send as is parameters to SLC
 */
static int slc_acpm_ioctl(void *data, int arg_cnt, int *args)
{
	struct slc_acpm_driver_data *driver_data = data;

	if (arg_cnt < 3)
		return -EINVAL;
	if (!slc_version_check(driver_data))
		return -ENOENT;
	args[0] = slc_acpm(driver_data, args[0], args[1], args[2], NULL);
	return 0;
}

/*
 * Apply the last changes of ptid to SLC by calling APM
 */
static void slc_acpm_apply(struct slc_acpm_driver_data *driver_data,
	ptid_t ptid, bool zero_size)
{
	int ret;
	unsigned int arg;
	unsigned int arg1;

	arg = (ptid << 8) | (driver_data->ptids[ptid].vptid << 16);
	arg1 = zero_size ? 1 : driver_data->ptids[ptid].size_bits;
	ret = slc_acpm(driver_data, PT_MUTATE, arg, arg1, NULL);
}


static void slc_acpm_ipc_callback(unsigned int *cmd, unsigned int size)
{
	struct slc_acpm_driver_data *driver_data = slc_acpm_driver_data;

	pt_driver_log_module(driver_data->pdev->dev.of_node->name, __func__, 0,
			0, 0, 0, 0, 0, 0, 0);
	/*
	 * This callback happen in acpm thread context,
	 * it is ok to wait
	 */
	slc_acpm_check(driver_data);
}


/*
 * External functions
 */


/*
 * Life of a PTID (partition):
 * - slc_acpm_alloc will create/allocate PTID with zero size,
 * to have ptidallocated while doing nothing.
 * - slc_acpm_enable will move its size to the expected one.
 * - slc_acpm_disable will move its size to zero.
 * - slc_acpm_free will destroy/free it.
 */
static ptid_t slc_acpm_alloc(void *data, int property_index, void *resize_data,
		void (*resize)(void *data, size_t size))
{
	struct slc_acpm_driver_data *driver_data =
		(struct slc_acpm_driver_data *)data;
	u32 vptid;
	u32 size_bits;
	u32 pbha;
	u32 priority;
	unsigned int arg;
	int ret;
	ptid_t ptid;
	int retnb;
	unsigned long flags;

	if (!slc_version_check(driver_data))
		return PT_PTID_INVALID;
	if (pt_driver_get_property_value(driver_data->driver,
		property_index, PT_PROPERTY_INDEX_VPTID, &vptid) < 0)
		return PT_PTID_INVALID;
	if (pt_driver_get_property_value(driver_data->driver,
		property_index, PT_PROPERTY_INDEX_SIZE_BITS, &size_bits) < 0)
		return PT_PTID_INVALID;
	if (pt_driver_get_property_value(driver_data->driver,
		property_index, PT_PROPERTY_INDEX_PRIORITY, &priority) < 0)
		return PT_PTID_INVALID;
	if (pt_driver_get_property_value(driver_data->driver,
		property_index, PT_PROPERTY_INDEX_PBHA, &pbha) < 0) {
		pbha = 0;
	} else {
		pbha = pbha & PT_PBHA_MASK;
	}

	arg = priority | (vptid << 8) | ((pbha & 0xff) << 16);
	ret = slc_acpm(driver_data, PT_ENABLE, arg, 1 /* 0 size alloc */, NULL);
	if (ret < 0)
		return ret;

	pt_ptid_data_decode(ret, &ptid, &retnb);


	spin_lock_irqsave(&driver_data->sl, flags);
	driver_data->ptids[ptid].vptid = vptid;
	driver_data->ptids[ptid].size_bits = size_bits;
	driver_data->ptids[ptid].data = resize_data;
	driver_data->ptids[ptid].pbha = pbha;
	driver_data->ptids[ptid].resize = resize;
	spin_unlock_irqrestore(&driver_data->sl, flags);

	slc_acpm_check(driver_data);

	dev_dbg(&driver_data->pdev->dev, "allocated ptid %d\n", ptid);
	return (int)ptid;
}

static ptid_t slc_acpm_mutate(void *data, ptid_t ptid, void *resize_data,
	int new_property_index)
{
	struct slc_acpm_driver_data *driver_data =
		(struct slc_acpm_driver_data *)data;
	u32 vptid;
	u32 size_bits;
	u32 pbha;
	u32 priority;

	if (!slc_version_check(driver_data))
		return PT_PTID_INVALID;

	if (pt_driver_get_property_value(driver_data->driver,
		new_property_index, PT_PROPERTY_INDEX_VPTID,
		&vptid) < 0)
		return PT_PTID_INVALID;
	if (pt_driver_get_property_value(driver_data->driver,
		new_property_index, PT_PROPERTY_INDEX_SIZE_BITS,
		&size_bits) < 0)
		return PT_PTID_INVALID;
	if (pt_driver_get_property_value(driver_data->driver,
		new_property_index, PT_PROPERTY_INDEX_PRIORITY,
		&priority) < 0)
		return PT_PTID_INVALID;
	if (pt_driver_get_property_value(driver_data->driver,
		new_property_index, PT_PROPERTY_INDEX_PBHA,
		&pbha) < 0)
		pbha = 0;
	if (driver_data->ptids[ptid].pbha != (pbha & PT_PBHA_MASK))
		return PT_PTID_INVALID;

	driver_data->ptids[ptid].data = resize_data;
	driver_data->ptids[ptid].vptid = vptid;
	driver_data->ptids[ptid].size_bits = size_bits;
	slc_acpm_apply(driver_data, ptid, false);
	slc_acpm_check(driver_data);
	return ptid;
}

static ptpbha_t slc_acpm_pbha(void *data, ptid_t ptid)
{
	struct slc_acpm_driver_data *driver_data =
		(struct slc_acpm_driver_data *)data;

	return driver_data->ptids[ptid].pbha & PT_PBHA_MASK;
}

static void slc_acpm_free(void *data, ptid_t ptid)
{
	struct slc_acpm_driver_data *driver_data =
		(struct slc_acpm_driver_data *)data;
	unsigned int arg;
	unsigned long flags;

	if (!slc_version_check(driver_data))
		return;
	arg = (unsigned int)ptid;
	WARN_ON(slc_acpm(driver_data, PT_DISABLE, arg, 0 /* unused */, NULL) <
		0);
	slc_acpm_check(driver_data);
	spin_lock_irqsave(&driver_data->sl, flags);
	memset(&driver_data->ptids[ptid], 0, sizeof(driver_data->ptids[ptid]));
	spin_unlock_irqrestore(&driver_data->sl, flags);
}

static void slc_acpm_enable(void *data, ptid_t ptid)
{
	struct slc_acpm_driver_data *driver_data =
				(struct slc_acpm_driver_data *)data;

	if (!slc_version_check(driver_data))
		return;
	slc_acpm_apply(driver_data, ptid, false);
	slc_acpm_check(driver_data);
	dev_dbg(&driver_data->pdev->dev, "enabled ptid %d\n", ptid);
}

static void slc_acpm_disable(void *data, ptid_t ptid)
{
	struct slc_acpm_driver_data *driver_data =
				(struct slc_acpm_driver_data *)data;

	if (!slc_version_check(driver_data))
		return;
	slc_acpm_apply(driver_data, ptid, true);
	slc_acpm_check(driver_data);
}

static int slc_pldtrc_set(void *data, unsigned long long val)
{
	struct slc_acpm_driver_data *driver_data =
				(struct slc_acpm_driver_data *)data;
	int arg1, arg2;
	uint64_t cmd;

	if (!slc_version_check(driver_data))
		return -ENOENT;

	arg1 = (val & 0xff00) >> 8;
	arg2 = (val & 0xff);
	cmd = (arg1 == 0 || arg2 == 0) ? SLC_STOP : SLC_PLDTRC;

	dev_info(&driver_data->pdev->dev, "send cmd:%llx with arg1:%x arg2:%x to slc",
		cmd, arg1, arg2);

	return slc_acpm(driver_data, cmd, arg1, arg2, NULL);
}

DEFINE_SIMPLE_ATTRIBUTE(slc_pldtrc_ctl_fops,
		NULL, slc_pldtrc_set, "%llu\n");

static void slc_debugfs_init(struct slc_acpm_driver_data *driver_data)
{
	struct dentry *den;

	den = debugfs_lookup("acpm_framework", NULL);
	if (!den)
		den = debugfs_create_dir("acpm_framework", NULL);

	debugfs_create_file("slc_pldtrc_ctl", 0644, den, driver_data,
			    &slc_pldtrc_ctl_fops);

}
static int slc_acpm_probe(struct platform_device *pdev)
{
	struct slc_acpm_driver_data *driver_data =
		kmalloc(sizeof(struct slc_acpm_driver_data), GFP_KERNEL);

	if (driver_data == NULL)
		return -ENOMEM;
	memset(driver_data, 0, sizeof(struct slc_acpm_driver_data));
	spin_lock_init(&driver_data->sl);
	platform_set_drvdata(pdev, driver_data);
	driver_data->pdev = pdev;
	slc_version_check(driver_data);

	driver_data->driver = pt_driver_register(pdev->dev.of_node,
			&slc_acpm_ops, driver_data);
	WARN_ON(driver_data->driver == NULL);

	slc_debugfs_init(driver_data);

	slc_pmon_init(driver_data, slc_acpm);

	return 0;
}

static const struct of_device_id slc_acpm_of_match_table[] = {
	{ .compatible = "google,slc-acpm", },
	{ },

};
MODULE_DEVICE_TABLE(of, slc_acpm_of_match_table);

static struct platform_driver slc_acpm_driver = {
	.probe = slc_acpm_probe,
	.driver = {
		.name = "google,slc-acpm",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(slc_acpm_of_match_table),
	}
};

static int __init slc_acpm_init(void)
{
	return platform_driver_register(&slc_acpm_driver);
}

static void __exit slc_acpm_exit(void)
{
	slc_pmon_exit();
	platform_driver_unregister(&slc_acpm_driver);
}

module_init(slc_acpm_init);
module_exit(slc_acpm_exit);

MODULE_DESCRIPTION("SLC driver");
MODULE_AUTHOR("<cozette@google.com>");
MODULE_LICENSE("GPL");
