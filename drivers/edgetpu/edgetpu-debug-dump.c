// SPDX-License-Identifier: GPL-2.0
/*
 * Module that defines structures and functions to retrieve debug dump segments
 * from edgetpu firmware.
 *
 * Copyright (C) 2020-2021 Google LLC
 */

#include <linux/debugfs.h>
#include <linux/workqueue.h>

#include "edgetpu-config.h"
#include "edgetpu-debug-dump.h"
#include "edgetpu-device-group.h"
#include "edgetpu-iremap-pool.h"
#include "edgetpu-kci.h"
#include "edgetpu-pm.h"

static int edgetpu_get_debug_dump_set(void *data, u64 val)
{
	struct edgetpu_dev *etdev = data;
	int ret = edgetpu_pm_get(etdev->pm);

	if (ret)
		return ret;
	ret = edgetpu_get_debug_dump(etdev, val);
	if (ret > 0) {
		etdev_warn(etdev, "FW refused debug dump request: %d", ret);
		ret = -EOPNOTSUPP;
	}
	edgetpu_pm_put(etdev->pm);
	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_get_debug_dump, NULL, edgetpu_get_debug_dump_set, "%llu\n");

/*
 * Creates debugFS entries for interacting with debug dump functions.
 *
 * This is expected to be called by edgetpu_debug_dump_init().
 */
static inline void edgetpu_setup_debug_dump_fs(struct edgetpu_dev *etdev)
{
	/* forwards write requests to edgetpu_get_debug_dump() */
	debugfs_create_file("get_debug_dump", 0220, etdev->d_entry, etdev, &fops_get_debug_dump);
}

int edgetpu_get_debug_dump(struct edgetpu_dev *etdev, u64 type)
{
	int ret;
	struct edgetpu_debug_dump_setup *dump_setup;
	bool init_fw_dump_buffer = false;

	if (!etdev->debug_dump_mem.vaddr) {
		etdev_dbg(etdev, "Debug dump not allocated");
		return -EINVAL;
	}

	if (type) {
		dump_setup =
			(struct edgetpu_debug_dump_setup *)etdev->debug_dump_mem.vaddr;
		dump_setup->type = type;
	} else {
		init_fw_dump_buffer = true;
	}
	/* Signal the type of dump and buffer address to firmware */
	ret = edgetpu_kci_get_debug_dump(etdev->kci,
					 etdev->debug_dump_mem.tpu_addr,
					 etdev->debug_dump_mem.size, init_fw_dump_buffer);
	etdev_dbg(etdev, "Sent debug dump request, tpu addr: %llx",
		  (u64)etdev->debug_dump_mem.tpu_addr);
	if (ret) {
		if (ret == KCI_ERROR_UNIMPLEMENTED) {
			etdev_dbg(etdev, "Debug dump KCI not implemented");
		} else {
			if (init_fw_dump_buffer)
				etdev_err(etdev, "failed to init dump buffer in FW");
			else
				etdev_err(etdev, "Debug dump KCI req failed: %d", ret);
		}
	}

	return ret;
}

static void edgetpu_reset_debug_dump(struct edgetpu_dev *etdev)
{
	memset(etdev->debug_dump_mem.vaddr, 0, etdev->debug_dump_mem.size);
}

static void edgetpu_debug_dump_work(struct work_struct *work)
{
	struct edgetpu_dev *etdev;
	struct edgetpu_debug_dump_setup *dump_setup;
	struct edgetpu_debug_dump *debug_dump;
	int ret;
	u64 dump_reason;

	etdev = container_of(work, struct edgetpu_dev, debug_dump_work);
	dump_setup =
		(struct edgetpu_debug_dump_setup *)etdev->debug_dump_mem.vaddr;
	debug_dump = (struct edgetpu_debug_dump *)(dump_setup + 1);

	if (!etdev->debug_dump_handlers) {
		etdev_err(etdev, "Failed to generate coredump as handler is NULL");
		edgetpu_reset_debug_dump(etdev);
		return;
	}

	dump_reason = debug_dump->dump_reason;
	if (dump_reason >= DUMP_REASON_NUM ||
	    !etdev->debug_dump_handlers[dump_reason]) {
		etdev_err(etdev,
			  "Failed to generate coredump as handler is NULL for dump request reason: %#llx",
			  dump_reason);
		edgetpu_reset_debug_dump(etdev);
		return;
	}

	ret = etdev->debug_dump_handlers[dump_reason]((void *)etdev, (void *)dump_setup);
	if (ret)
		etdev_err(etdev, "Failed to generate coredump: %d\n", ret);
	edgetpu_reset_debug_dump(etdev);
}

void edgetpu_debug_dump_resp_handler(struct edgetpu_dev *etdev)
{
	struct edgetpu_debug_dump_setup *dump_setup;
	struct edgetpu_debug_dump *debug_dump;

	if (!etdev->debug_dump_mem.vaddr)
		return;
	dump_setup =
		(struct edgetpu_debug_dump_setup *)etdev->debug_dump_mem.vaddr;
	debug_dump = (struct edgetpu_debug_dump *)(dump_setup + 1);
	if (!debug_dump->host_dump_available_to_read)
		return;

	debug_dump->host_dump_available_to_read = false;

	if (!etdev->debug_dump_work.func)
		INIT_WORK(&etdev->debug_dump_work, edgetpu_debug_dump_work);

	schedule_work(&etdev->debug_dump_work);
}
