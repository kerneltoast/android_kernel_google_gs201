// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Misc Utility Functions and Wrappers
 *
 * Copyright (c) 2018 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-util: " fmt

#include <linux/slab.h>
#include <linux/byteorder/generic.h>
#include <uapi/linux/sched/types.h>
#include "lwis_util.h"
#include "lwis_device.h"

int lwis_device_single_register_write(struct lwis_device *lwis_dev, int bid, uint64_t offset,
				      uint64_t value, int access_size)
{
	int ret = 0;
	struct lwis_io_entry entry = {};

	if (!lwis_dev)
		return -ENODEV;

	if (lwis_dev->vops.register_io == NULL) {
		dev_err(lwis_dev->dev, "%s: register_io undefined\n", __func__);
		return -EINVAL;
	}

	entry.type = LWIS_IO_ENTRY_WRITE;
	entry.rw.offset = offset;
	entry.rw.val = value;
	entry.rw.bid = bid;

	if (lwis_dev->vops.register_io_barrier) {
		lwis_dev->vops.register_io_barrier(lwis_dev, /*use_read_barrier=*/false,
						   /*use_write_barrier=*/true);
	}
	ret = lwis_dev->vops.register_io(lwis_dev, &entry, access_size);
	if (ret) {
		dev_err(lwis_dev->dev,
			"Register write bid %d offset 0x%llx value 0x%llx failed: %d", bid, offset,
			value, ret);
	}
	return ret;
}

int lwis_device_single_register_read(struct lwis_device *lwis_dev, int bid, uint64_t offset,
				     uint64_t *value, int access_size)
{
	int ret = -EINVAL;
	struct lwis_io_entry entry = {};

	if (!lwis_dev)
		return -ENODEV;

	if (lwis_dev->vops.register_io == NULL) {
		dev_err(lwis_dev->dev, "%s: register_io undefined\n", __func__);
		return -EINVAL;
	}

	entry.type = LWIS_IO_ENTRY_READ;
	entry.rw.offset = offset;
	entry.rw.bid = bid;

	ret = lwis_dev->vops.register_io(lwis_dev, &entry, access_size);
	if (lwis_dev->vops.register_io_barrier) {
		lwis_dev->vops.register_io_barrier(lwis_dev, /*use_read_barrier=*/true,
						   /*use_write_barrier=*/false);
	}
	if (!ret && value)
		*value = entry.rw.val;

	return ret;
}

const char *lwis_device_type_to_string(int32_t type)
{
	switch (type) {
	case DEVICE_TYPE_TOP:
		return "TOP";
	case DEVICE_TYPE_I2C:
		return "I2C";
	case DEVICE_TYPE_IOREG:
		return "IOREG";
	case DEVICE_TYPE_SLC:
		return "SLC";
	case DEVICE_TYPE_TEST:
		return "TEST";
	case DEVICE_TYPE_SPI:
		return "SPI";
	case DEVICE_TYPE_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}

const char *trigger_condition_node_operator_to_string(int32_t type)
{
	switch (type) {
	case LWIS_TRIGGER_NODE_OPERATOR_AND:
		return "AND";
	case LWIS_TRIGGER_NODE_OPERATOR_OR:
		return "OR";
	case LWIS_TRIGGER_NODE_OPERATOR_NONE:
	default:
		return "NONE";
	}
}

int lwis_create_kthread_workers(struct lwis_device *lwis_dev)
{
	char t_name[LWIS_MAX_NAME_STRING_LEN];

	if (!lwis_dev)
		return -ENODEV;

	scnprintf(t_name, LWIS_MAX_NAME_STRING_LEN, "lwis_t_%s", lwis_dev->name);

	kthread_init_worker(&lwis_dev->transaction_worker);
	lwis_dev->transaction_worker_thread =
		kthread_run(kthread_worker_fn, &lwis_dev->transaction_worker, t_name);
	if (IS_ERR_OR_NULL(lwis_dev->transaction_worker_thread)) {
		dev_err(lwis_dev->dev, "transaction kthread_run failed\n");
		return -EINVAL;
	}

	return 0;
}

int lwis_set_kthread_priority(struct lwis_device *lwis_dev, struct task_struct *task, u32 priority)
{
	int policy;
	struct sched_param param;
	int ret;

	if (priority >= MAX_PRIO) {
		dev_err(lwis_dev->dev, "transaction_thread_priority(%d) >= Max(%d)", priority,
			MAX_PRIO);
		return -EINVAL;
	}
	if (priority < MAX_RT_PRIO) {
		policy = SCHED_FIFO;
		param.sched_priority = MAX_RT_PRIO - priority;
	} else {
		policy = SCHED_NORMAL;
		param.sched_priority = 0;
		task->prio = priority;
		task->static_prio = priority;
		task->normal_prio = priority;
	}
	ret = sched_setscheduler_nocheck(task, policy, &param);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to set kthread priority (%d)", ret);
		return ret;
	}

	return 0;
}

bool lwis_check_device_type(struct lwis_device *lwis_dev, int32_t type)
{
	return ((lwis_dev) && (lwis_dev->type == type));
}

void lwis_value_to_be_buf(uint64_t value, uint8_t *buf, int buf_size)
{
	if (buf_size == 1) {
		buf[0] = value;
	} else if (buf_size == 2) {
		buf[0] = (value >> 8) & 0xFF;
		buf[1] = value & 0xFF;
	} else if (buf_size == 4) {
		buf[0] = (value >> 24) & 0xFF;
		buf[1] = (value >> 16) & 0xFF;
		buf[2] = (value >> 8) & 0xFF;
		buf[3] = value & 0xFF;
	} else {
		pr_err("Unsupported buffer size %d used for value_to_buf\n", buf_size);
	}
}

uint64_t lwis_be_buf_to_value(uint8_t *buf, int buf_size)
{
	if (buf_size == 1)
		return buf[0];

	if (buf_size == 2)
		return be16_to_cpup((const uint16_t *)buf);

	if (buf_size == 4)
		return be32_to_cpup((const uint32_t *)buf);

	pr_err("Unsupported buffer size %d used for buf_to_value\n", buf_size);
	return 0;
}
