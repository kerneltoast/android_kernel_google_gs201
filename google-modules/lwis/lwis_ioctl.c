/*
 * Google LWIS IOCTL Handler
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-ioctl: " fmt

#include "lwis_ioctl.h"

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "lwis_allocator.h"
#include "lwis_buffer.h"
#include "lwis_commands.h"
#include "lwis_device.h"
#include "lwis_device_dpm.h"
#include "lwis_device_i2c.h"
#include "lwis_device_ioreg.h"
#include "lwis_event.h"
#include "lwis_i2c.h"
#include "lwis_io_entry.h"
#include "lwis_ioreg.h"
#include "lwis_periodic_io.h"
#include "lwis_platform.h"
#include "lwis_regulator.h"
#include "lwis_transaction.h"
#include "lwis_util.h"

#define IOCTL_TO_ENUM(x) _IOC_NR(x)
#define IOCTL_ARG_SIZE(x) _IOC_SIZE(x)
#define STRINGIFY(x) #x

static void lwis_ioctl_pr_err(struct lwis_device *lwis_dev, unsigned int ioctl_type, int errno)
{
	unsigned int type = IOCTL_TO_ENUM(ioctl_type);
	static char type_name[32];
	size_t exp_size;

	switch (type) {
	case IOCTL_TO_ENUM(LWIS_GET_DEVICE_INFO):
		strlcpy(type_name, STRINGIFY(LWIS_GET_DEVICE_INFO), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_GET_DEVICE_INFO);
		break;
	case IOCTL_TO_ENUM(LWIS_BUFFER_ALLOC):
		strlcpy(type_name, STRINGIFY(LWIS_BUFFER_ALLOC), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_BUFFER_ALLOC);
		break;
	case IOCTL_TO_ENUM(LWIS_BUFFER_FREE):
		strlcpy(type_name, STRINGIFY(LWIS_BUFFER_FREE), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_BUFFER_FREE);
		break;
	case IOCTL_TO_ENUM(LWIS_BUFFER_ENROLL):
		strlcpy(type_name, STRINGIFY(LWIS_BUFFER_ENROLL), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_BUFFER_ENROLL);
		break;
	case IOCTL_TO_ENUM(LWIS_BUFFER_DISENROLL):
		strlcpy(type_name, STRINGIFY(LWIS_BUFFER_DISENROLL), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_BUFFER_DISENROLL);
		break;
	case IOCTL_TO_ENUM(LWIS_BUFFER_CPU_ACCESS):
		strlcpy(type_name, STRINGIFY(LWIS_BUFFER_CPU_ACCESS), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_BUFFER_CPU_ACCESS);
		break;
	case IOCTL_TO_ENUM(LWIS_REG_IO):
		strlcpy(type_name, STRINGIFY(LWIS_REG_IO), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_REG_IO);
		break;
	case IOCTL_TO_ENUM(LWIS_DEVICE_ENABLE):
		strlcpy(type_name, STRINGIFY(LWIS_DEVICE_ENABLE), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_DEVICE_ENABLE);
		break;
	case IOCTL_TO_ENUM(LWIS_DEVICE_DISABLE):
		strlcpy(type_name, STRINGIFY(LWIS_DEVICE_DISABLE), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_DEVICE_DISABLE);
		break;
	case IOCTL_TO_ENUM(LWIS_DEVICE_RESET):
		strlcpy(type_name, STRINGIFY(LWIS_DEVICE_RESET), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_DEVICE_RESET);
		break;
	case IOCTL_TO_ENUM(LWIS_EVENT_CONTROL_GET):
		strlcpy(type_name, STRINGIFY(LWIS_EVENT_CONTROL_GET), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_EVENT_CONTROL_GET);
		break;
	case IOCTL_TO_ENUM(LWIS_EVENT_CONTROL_SET):
		strlcpy(type_name, STRINGIFY(LWIS_EVENT_CONTROL_SET), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_EVENT_CONTROL_SET);
		break;
	case IOCTL_TO_ENUM(LWIS_EVENT_DEQUEUE):
		strlcpy(type_name, STRINGIFY(LWIS_EVENT_DEQUEUE), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_EVENT_DEQUEUE);
		break;
	case IOCTL_TO_ENUM(LWIS_TIME_QUERY):
		strlcpy(type_name, STRINGIFY(LWIS_TIME_QUERY), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_TIME_QUERY);
		break;
	case IOCTL_TO_ENUM(LWIS_TRANSACTION_SUBMIT):
		strlcpy(type_name, STRINGIFY(LWIS_TRANSACTION_SUBMIT), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_TRANSACTION_SUBMIT);
		break;
	case IOCTL_TO_ENUM(LWIS_TRANSACTION_CANCEL):
		strlcpy(type_name, STRINGIFY(LWIS_TRANSACTION_CANCEL), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_TRANSACTION_CANCEL);
		break;
	case IOCTL_TO_ENUM(LWIS_TRANSACTION_REPLACE):
		strlcpy(type_name, STRINGIFY(LWIS_TRANSACTION_REPLACE), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_TRANSACTION_REPLACE);
		break;
	case IOCTL_TO_ENUM(LWIS_DPM_CLK_UPDATE):
		strlcpy(type_name, STRINGIFY(LWIS_DPM_CLK_UPDATE), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_DPM_CLK_UPDATE);
		break;
	case IOCTL_TO_ENUM(LWIS_ECHO):
		strlcpy(type_name, STRINGIFY(LWIS_ECHO), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_ECHO);
		break;
	case IOCTL_TO_ENUM(LWIS_DPM_QOS_UPDATE):
		strlcpy(type_name, STRINGIFY(LWIS_DPM_QOS_UPDATE), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_DPM_QOS_UPDATE);
		break;
	case IOCTL_TO_ENUM(LWIS_DPM_GET_CLOCK):
		strlcpy(type_name, STRINGIFY(LWIS_DPM_GET_CLOCK), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_DPM_GET_CLOCK);
		break;
	case IOCTL_TO_ENUM(LWIS_PERIODIC_IO_SUBMIT):
		strlcpy(type_name, STRINGIFY(LWIS_PERIODIC_IO_SUBMIT), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_PERIODIC_IO_SUBMIT);
		break;
	case IOCTL_TO_ENUM(LWIS_PERIODIC_IO_CANCEL):
		strlcpy(type_name, STRINGIFY(LWIS_PERIODIC_IO_CANCEL), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_PERIODIC_IO_CANCEL);
		break;
	default:
		strlcpy(type_name, "UNDEFINED", sizeof(type_name));
		exp_size = 0;
		break;
	};

	if (strcmp(type_name, "UNDEFINED") && exp_size != IOCTL_ARG_SIZE(ioctl_type)) {
		dev_err_ratelimited(
			lwis_dev->dev,
			"Failed to process %s (errno: %d), expecting argument with length of %zu, got length of %d. Mismatch kernel version?\n",
			type_name, errno, exp_size, IOCTL_ARG_SIZE(ioctl_type));
	} else {
		dev_err_ratelimited(lwis_dev->dev, "Failed to process %s (errno: %d)\n", type_name,
				    errno);
	}
}

static int ioctl_get_device_info(struct lwis_device *lwis_dev, struct lwis_device_info *msg)
{
	int i;
	struct lwis_device_info k_info = { .id = lwis_dev->id,
					   .type = lwis_dev->type,
					   .num_clks = 0,
					   .num_regs = 0,
					   .transaction_worker_thread_pid = -1,
					   .periodic_io_thread_pid = -1 };
	strlcpy(k_info.name, lwis_dev->name, LWIS_MAX_NAME_STRING_LEN);

	if (lwis_dev->clocks) {
		k_info.num_clks = lwis_dev->clocks->count;
		for (i = 0; i < lwis_dev->clocks->count; i++) {
			if (i >= LWIS_MAX_CLOCK_NUM) {
				dev_err(lwis_dev->dev,
					"Clock count larger than LWIS_MAX_CLOCK_NUM\n");
				break;
			}
			strlcpy(k_info.clks[i].name, lwis_dev->clocks->clk[i].name,
				LWIS_MAX_NAME_STRING_LEN);
			k_info.clks[i].clk_index = i;
			k_info.clks[i].frequency = 0;
		}
	}

	if (lwis_dev->type == DEVICE_TYPE_IOREG) {
		struct lwis_ioreg_device *ioreg_dev;
		ioreg_dev = container_of(lwis_dev, struct lwis_ioreg_device, base_dev);
		if (ioreg_dev->reg_list.count > 0) {
			k_info.num_regs = ioreg_dev->reg_list.count;
			for (i = 0; i < ioreg_dev->reg_list.count; i++) {
				if (i >= LWIS_MAX_REG_NUM) {
					dev_err(lwis_dev->dev,
						"Reg count larger than LWIS_MAX_REG_NUM\n");
					break;
				}
				strlcpy(k_info.regs[i].name, ioreg_dev->reg_list.block[i].name,
					LWIS_MAX_NAME_STRING_LEN);
				k_info.regs[i].reg_index = i;
				k_info.regs[i].start = ioreg_dev->reg_list.block[i].start;
				k_info.regs[i].size = ioreg_dev->reg_list.block[i].size;
			}
		}
	}

	if (lwis_dev->transaction_worker_thread) {
		k_info.transaction_worker_thread_pid = lwis_dev->transaction_worker_thread->pid;
	}

	if (lwis_dev->periodic_io_worker_thread) {
		k_info.periodic_io_thread_pid = lwis_dev->periodic_io_worker_thread->pid;
	}

	if (copy_to_user((void __user *)msg, &k_info, sizeof(k_info))) {
		dev_err(lwis_dev->dev, "Failed to copy device info to userspace\n");
		return -EFAULT;
	}

	return 0;
}

static int register_read(struct lwis_device *lwis_dev, struct lwis_io_entry *read_entry,
			 struct lwis_io_entry *user_msg)
{
	int ret = 0;
	uint8_t *user_buf;
	bool batch_mode = false;

	if (read_entry->type == LWIS_IO_ENTRY_READ_BATCH) {
		batch_mode = true;
		/* Save the userspace buffer address */
		user_buf = read_entry->rw_batch.buf;
		/* Allocate read buffer */
		read_entry->rw_batch.buf =
			lwis_allocator_allocate(lwis_dev, read_entry->rw_batch.size_in_bytes);
		if (!read_entry->rw_batch.buf) {
			dev_err_ratelimited(lwis_dev->dev,
					    "Failed to allocate register read buffer\n");
			return -ENOMEM;
		}
	} else if (read_entry->type != LWIS_IO_ENTRY_READ) {
		/* Type must be either READ or READ_BATCH */
		dev_err(lwis_dev->dev, "Invalid io_entry type for REGISTER_READ\n");
		return -EINVAL;
	}

	ret = lwis_dev->vops.register_io(lwis_dev, read_entry, lwis_dev->native_value_bitwidth);
	if (ret) {
		dev_err_ratelimited(lwis_dev->dev, "Failed to read registers\n");
		goto reg_read_exit;
	}

	/* Copy read data back to userspace */
	if (batch_mode) {
		if (copy_to_user((void __user *)user_buf, read_entry->rw_batch.buf,
				 read_entry->rw_batch.size_in_bytes)) {
			ret = -EFAULT;
			dev_err_ratelimited(
				lwis_dev->dev,
				"Failed to copy register read buffer back to userspace\n");
		}
	} else {
		if (copy_to_user((void __user *)user_msg, read_entry, sizeof(*read_entry))) {
			ret = -EFAULT;
			dev_err_ratelimited(
				lwis_dev->dev,
				"Failed to copy register read entry back to userspace\n");
		}
	}

reg_read_exit:
	if (batch_mode) {
		lwis_allocator_free(lwis_dev, read_entry->rw_batch.buf);
		read_entry->rw_batch.buf = NULL;
	}
	return ret;
}

static int register_write(struct lwis_device *lwis_dev, struct lwis_io_entry *write_entry)
{
	int ret = 0;
	uint8_t *user_buf;
	bool batch_mode = false;

	if (write_entry->type == LWIS_IO_ENTRY_WRITE_BATCH) {
		batch_mode = true;
		/* Save the userspace buffer address */
		user_buf = write_entry->rw_batch.buf;
		/* Allocate write buffer and copy contents from userspace */
		write_entry->rw_batch.buf =
			lwis_allocator_allocate(lwis_dev, write_entry->rw_batch.size_in_bytes);
		if (!write_entry->rw_batch.buf) {
			dev_err_ratelimited(lwis_dev->dev,
					    "Failed to allocate register write buffer\n");
			return -ENOMEM;
		}

		if (copy_from_user(write_entry->rw_batch.buf, (void __user *)user_buf,
				   write_entry->rw_batch.size_in_bytes)) {
			ret = -EFAULT;
			dev_err_ratelimited(lwis_dev->dev,
					    "Failed to copy write buffer from userspace\n");
			goto reg_write_exit;
		}
	} else if (write_entry->type != LWIS_IO_ENTRY_WRITE) {
		/* Type must be either WRITE or WRITE_BATCH */
		dev_err(lwis_dev->dev, "Invalid io_entry type for REGISTER_WRITE\n");
		return -EINVAL;
	}

	ret = lwis_dev->vops.register_io(lwis_dev, write_entry, lwis_dev->native_value_bitwidth);
	if (ret) {
		dev_err_ratelimited(lwis_dev->dev, "Failed to write registers\n");
	}

reg_write_exit:
	if (batch_mode) {
		lwis_allocator_free(lwis_dev, write_entry->rw_batch.buf);
		write_entry->rw_batch.buf = NULL;
	}
	return ret;
}

static int register_modify(struct lwis_device *lwis_dev, struct lwis_io_entry *modify_entry)
{
	int ret = 0;

	ret = lwis_dev->vops.register_io(lwis_dev, modify_entry, lwis_dev->native_value_bitwidth);
	if (ret) {
		dev_err_ratelimited(lwis_dev->dev, "Failed to read registers for modify\n");
	}

	return ret;
}

static int copy_io_entries(struct lwis_device *lwis_dev, struct lwis_io_entries *user_msg,
			   struct lwis_io_entries *k_msg, struct lwis_io_entry **k_entries)
{
	int ret = 0;
	struct lwis_io_entry *io_entries;
	uint32_t buf_size;

	/* Register io is not supported for the lwis device, return */
	if (!lwis_dev->vops.register_io) {
		dev_err(lwis_dev->dev, "Register IO not supported on this LWIS device\n");
		return -EINVAL;
	}

	/* Copy io_entries from userspace */
	if (copy_from_user(k_msg, (void __user *)user_msg, sizeof(*k_msg))) {
		dev_err(lwis_dev->dev, "Failed to copy io_entries header from userspace.\n");
		return -EFAULT;
	}
	buf_size = sizeof(struct lwis_io_entry) * k_msg->num_io_entries;
	if (buf_size / sizeof(struct lwis_io_entry) != k_msg->num_io_entries) {
		dev_err(lwis_dev->dev, "Failed to copy io_entries due to integer overflow.\n");
		return -EOVERFLOW;
	}
	io_entries = lwis_allocator_allocate(lwis_dev, buf_size);
	if (!io_entries) {
		dev_err(lwis_dev->dev, "Failed to allocate io_entries buffer\n");
		return -ENOMEM;
	}
	if (copy_from_user(io_entries, (void __user *)k_msg->io_entries, buf_size)) {
		ret = -EFAULT;
		lwis_allocator_free(lwis_dev, io_entries);
		dev_err(lwis_dev->dev, "Failed to copy io_entries from userspace.\n");
		return ret;
	}
	*k_entries = io_entries;

	return 0;
}

static int synchronous_process_io_entries(struct lwis_device *lwis_dev, int num_io_entries,
					  struct lwis_io_entry *io_entries,
					  struct lwis_io_entry *user_msg)
{
	int ret = 0, i = 0;

	/* Use write memory barrier at the beginning of I/O entries if the access protocol
	 * allows it */
	if (lwis_dev->vops.register_io_barrier != NULL) {
		lwis_dev->vops.register_io_barrier(lwis_dev,
						   /*use_read_barrier=*/false,
						   /*use_write_barrier=*/true);
	}
	for (i = 0; i < num_io_entries; i++) {
		switch (io_entries[i].type) {
		case LWIS_IO_ENTRY_MODIFY:
			ret = register_modify(lwis_dev, &io_entries[i]);
			break;
		case LWIS_IO_ENTRY_READ:
		case LWIS_IO_ENTRY_READ_BATCH:
			ret = register_read(lwis_dev, &io_entries[i], user_msg + i);
			break;
		case LWIS_IO_ENTRY_WRITE:
		case LWIS_IO_ENTRY_WRITE_BATCH:
			ret = register_write(lwis_dev, &io_entries[i]);
			break;
		case LWIS_IO_ENTRY_POLL:
			ret = lwis_io_entry_poll(lwis_dev, &io_entries[i], /*non_blocking=*/false);
			break;
		case LWIS_IO_ENTRY_READ_ASSERT:
			ret = lwis_io_entry_read_assert(lwis_dev, &io_entries[i]);
			break;
		default:
			dev_err(lwis_dev->dev, "Unknown io_entry operation\n");
			ret = -EINVAL;
		}
		if (ret) {
			dev_err(lwis_dev->dev, "Register io_entry failed\n");
			goto exit;
		}
	}
exit:
	/* Use read memory barrier at the end of I/O entries if the access protocol
	 * allows it */
	if (lwis_dev->vops.register_io_barrier != NULL) {
		lwis_dev->vops.register_io_barrier(lwis_dev,
						   /*use_read_barrier=*/true,
						   /*use_write_barrier=*/false);
	}
	return ret;
}

static int ioctl_reg_io(struct lwis_device *lwis_dev, struct lwis_io_entries *user_msg)
{
	int ret = 0;
	struct lwis_io_entries k_msg;
	struct lwis_io_entry *k_entries = NULL;

	ret = copy_io_entries(lwis_dev, user_msg, &k_msg, &k_entries);
	if (ret) {
		goto reg_io_exit;
	}

	/* Walk through and execute the entries */
	ret = synchronous_process_io_entries(lwis_dev, k_msg.num_io_entries, k_entries,
					     k_msg.io_entries);

reg_io_exit:
	if (k_entries) {
		lwis_allocator_free(lwis_dev, k_entries);
	}
	return ret;
}

static int ioctl_buffer_alloc(struct lwis_client *lwis_client,
			      struct lwis_alloc_buffer_info __user *msg)
{
	unsigned long ret = 0;
	struct lwis_alloc_buffer_info alloc_info;
	struct lwis_allocated_buffer *buffer;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	buffer = kmalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		dev_err(lwis_dev->dev, "Failed to allocated lwis_allocated_buffer\n");
		return -ENOMEM;
	}

	if (copy_from_user((void *)&alloc_info, (void __user *)msg, sizeof(alloc_info))) {
		ret = -EFAULT;
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes from user\n", sizeof(alloc_info));
		goto error_alloc;
	}

	ret = lwis_buffer_alloc(lwis_client, &alloc_info, buffer);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to allocate buffer\n");
		goto error_alloc;
	}

	if (copy_to_user((void __user *)msg, (void *)&alloc_info, sizeof(alloc_info))) {
		ret = -EFAULT;
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes to user\n", sizeof(alloc_info));
		lwis_buffer_free(lwis_client, buffer);
		goto error_alloc;
	}

	return 0;

error_alloc:
	kfree(buffer);
	return ret;
}

static int ioctl_buffer_free(struct lwis_client *lwis_client, int __user *msg)
{
	int ret = 0;
	int fd;
	struct lwis_allocated_buffer *buffer;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (copy_from_user((void *)&fd, (void __user *)msg, sizeof(fd))) {
		dev_err(lwis_dev->dev, "Failed to copy file descriptor from user\n");
		return -EFAULT;
	}

	buffer = lwis_client_allocated_buffer_find(lwis_client, fd);
	if (!buffer) {
		dev_err(lwis_dev->dev, "Cannot find allocated buffer FD %d\n", fd);
		return -ENOENT;
	}

	ret = lwis_buffer_free(lwis_client, buffer);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to free buffer FD %d\n", fd);
		return ret;
	}

	kfree(buffer);

	return 0;
}

static int ioctl_buffer_enroll(struct lwis_client *lwis_client, struct lwis_buffer_info __user *msg)
{
	unsigned long ret = 0;
	struct lwis_enrolled_buffer *buffer;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	buffer = kmalloc(sizeof(struct lwis_enrolled_buffer), GFP_KERNEL);
	if (!buffer) {
		dev_err(lwis_dev->dev, "Failed to allocate lwis_enrolled_buffer struct\n");
		return -ENOMEM;
	}

	if (copy_from_user((void *)&buffer->info, (void __user *)msg, sizeof(buffer->info))) {
		ret = -EFAULT;
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes from user\n",
			sizeof(buffer->info));
		goto error_enroll;
	}

	ret = lwis_buffer_enroll(lwis_client, buffer);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to enroll buffer\n");
		goto error_enroll;
	}

	if (copy_to_user((void __user *)msg, (void *)&buffer->info, sizeof(buffer->info))) {
		ret = -EFAULT;
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes to user\n", sizeof(buffer->info));
		lwis_buffer_disenroll(lwis_client, buffer);
		goto error_enroll;
	}

	return 0;

error_enroll:
	kfree(buffer);
	return ret;
}

static int ioctl_buffer_disenroll(struct lwis_client *lwis_client,
				  struct lwis_enrolled_buffer_info __user *msg)
{
	unsigned long ret = 0;
	struct lwis_enrolled_buffer_info info;
	struct lwis_enrolled_buffer *buffer;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (copy_from_user((void *)&info, (void __user *)msg, sizeof(info))) {
		dev_err(lwis_dev->dev, "Failed to copy DMA virtual address from user\n");
		return -EFAULT;
	}

	buffer = lwis_client_enrolled_buffer_find(lwis_client, info.fd, info.dma_vaddr);

	if (!buffer) {
		dev_err(lwis_dev->dev, "Failed to find dma buffer for fd %d vaddr %pad\n", info.fd,
			&info.dma_vaddr);
		return -ENOENT;
	}

	ret = lwis_buffer_disenroll(lwis_client, buffer);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to disenroll dma buffer for fd %d vaddr %pad\n",
			info.fd, &info.dma_vaddr);
		return ret;
	}

	kfree(buffer);

	return 0;
}

static int ioctl_buffer_cpu_access(struct lwis_client *lwis_client,
				   struct lwis_buffer_cpu_access_op __user *msg)
{
	int ret = 0;
	struct lwis_buffer_cpu_access_op op;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (copy_from_user((void *)&op, (void __user *)msg, sizeof(op))) {
		dev_err(lwis_dev->dev, "Failed to copy buffer CPU access operation from user\n");
		return -EFAULT;
	}

	ret = lwis_buffer_cpu_access(lwis_client, &op);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to prepare for cpu access for fd %d\n", op.fd);
		return ret;
	}

	return 0;
}

static int ioctl_device_enable(struct lwis_client *lwis_client)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (lwis_client->is_enabled) {
		return ret;
	}
	mutex_lock(&lwis_dev->client_lock);
	if (lwis_dev->enabled > 0 && lwis_dev->enabled < INT_MAX) {
		lwis_dev->enabled++;
		lwis_client->is_enabled = true;
		mutex_unlock(&lwis_dev->client_lock);
		return 0;
	} else if (lwis_dev->enabled == INT_MAX) {
		dev_err(lwis_dev->dev, "Enable counter overflow\n");
		ret = -EINVAL;
		goto error_locked;
	}

	/* Clear event queues to make sure there is no stale event from
	 * previous session */
	lwis_client_event_queue_clear(lwis_client);
	lwis_client_error_event_queue_clear(lwis_client);

	ret = lwis_dev_power_up_locked(lwis_dev);
	if (ret < 0) {
		dev_err(lwis_dev->dev, "Failed to power up device\n");
		goto error_locked;
	}

	lwis_dev->enabled++;
	lwis_client->is_enabled = true;
	dev_info(lwis_dev->dev, "Device enabled\n");
error_locked:
	mutex_unlock(&lwis_dev->client_lock);
	return ret;
}

static int ioctl_device_disable(struct lwis_client *lwis_client)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (!lwis_client->is_enabled) {
		return ret;
	}

	mutex_lock(&lwis_dev->client_lock);
	/* Clear event states for this client */
	lwis_client_event_states_clear(lwis_client);
	mutex_unlock(&lwis_dev->client_lock);

	/* Flush all periodic io to complete */
	ret = lwis_periodic_io_client_flush(lwis_client);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to wait for in-process periodic io to complete\n");
	}

	/* Flush all pending transactions */
	ret = lwis_transaction_client_flush(lwis_client);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to flush pending transactions\n");
	}

	/* Run cleanup transactions. */
	lwis_transaction_client_cleanup(lwis_client);

	mutex_lock(&lwis_dev->client_lock);
	if (lwis_dev->enabled > 1) {
		lwis_dev->enabled--;
		lwis_client->is_enabled = false;
		mutex_unlock(&lwis_dev->client_lock);
		return 0;
	} else if (lwis_dev->enabled <= 0) {
		dev_err(lwis_dev->dev, "Disabling a device that is already disabled\n");
		ret = -EINVAL;
		goto error_locked;
	}

	ret = lwis_dev_power_down_locked(lwis_dev);
	if (ret < 0) {
		dev_err(lwis_dev->dev, "Failed to power down device\n");
		goto error_locked;
	}
	lwis_device_event_states_clear_locked(lwis_dev);

	lwis_dev->enabled--;
	lwis_client->is_enabled = false;
	dev_info(lwis_dev->dev, "Device disabled\n");
error_locked:
	mutex_unlock(&lwis_dev->client_lock);
	return ret;
}

static int ioctl_echo(struct lwis_device *lwis_dev, struct lwis_echo __user *msg)
{
	struct lwis_echo echo_msg;
	char *buffer;

	if (copy_from_user((void *)&echo_msg, (void __user *)msg, sizeof(echo_msg))) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes from user\n", sizeof(echo_msg));
		return -EFAULT;
	}

	if (echo_msg.size == 0) {
		return 0;
	}

	buffer = kmalloc(echo_msg.size + 1, GFP_KERNEL);
	if (!buffer) {
		dev_err(lwis_dev->dev, "Failed to allocate buffer for echo message\n");
		return -ENOMEM;
	}
	if (copy_from_user(buffer, (void __user *)echo_msg.msg, echo_msg.size)) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes echo message from user\n",
			echo_msg.size);
		kfree(buffer);
		return -EFAULT;
	}
	buffer[echo_msg.size] = '\0';

	if (echo_msg.kernel_log) {
		dev_info(lwis_dev->dev, "LWIS_ECHO: %s\n", buffer);
	}
	kfree(buffer);
	return 0;
}

static int ioctl_device_reset(struct lwis_client *lwis_client, struct lwis_io_entries *user_msg)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	struct lwis_io_entries k_msg;
	struct lwis_io_entry *k_entries = NULL;
	unsigned long flags;
	bool device_enabled = false;

	ret = copy_io_entries(lwis_dev, user_msg, &k_msg, &k_entries);
	if (ret) {
		goto soft_reset_exit;
	}

	/* Clear event states, event queues and transactions for this client */
	mutex_lock(&lwis_dev->client_lock);
	lwis_client_event_states_clear(lwis_client);
	lwis_client_event_queue_clear(lwis_client);
	lwis_client_error_event_queue_clear(lwis_client);
	device_enabled = lwis_dev->enabled;
	mutex_unlock(&lwis_dev->client_lock);

	/* Flush all periodic io to complete */
	ret = lwis_periodic_io_client_flush(lwis_client);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to wait for in-process periodic io to complete\n");
	}

	/* Flush all pending transactions */
	ret = lwis_transaction_client_flush(lwis_client);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to flush all pending transactions\n");
	}

	/* Perform reset routine defined by the io_entries */
	if (device_enabled) {
		ret = synchronous_process_io_entries(lwis_dev, k_msg.num_io_entries, k_entries,
						     k_msg.io_entries);
	} else {
		dev_warn(lwis_dev->dev,
			 "Device is not enabled, IoEntries will not be executed in DEVICE_RESET\n");
	}

	spin_lock_irqsave(&lwis_dev->lock, flags);
	lwis_device_event_states_clear_locked(lwis_dev);
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
soft_reset_exit:
	if (k_entries) {
		lwis_allocator_free(lwis_dev, k_entries);
	}
	return ret;
}

static int ioctl_event_control_get(struct lwis_client *lwis_client,
				   struct lwis_event_control __user *msg)
{
	unsigned long ret = 0;
	struct lwis_event_control control;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (copy_from_user((void *)&control, (void __user *)msg, sizeof(control))) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes from user\n", sizeof(control));
		return -EFAULT;
	}

	ret = lwis_client_event_control_get(lwis_client, control.event_id, &control);

	if (ret) {
		dev_err(lwis_dev->dev, "Failed to get event: %lld (err:%ld)\n", control.event_id,
			ret);
		return -EINVAL;
	}

	if (copy_to_user((void __user *)msg, (void *)&control, sizeof(control))) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes to user\n", sizeof(control));
		return -EFAULT;
	}

	return 0;
}

static int ioctl_event_control_set(struct lwis_client *lwis_client,
				   struct lwis_event_control_list __user *msg)
{
	struct lwis_event_control_list k_msg;
	struct lwis_event_control *k_event_controls;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	int ret = 0;
	int i;
	size_t buf_size;

	if (copy_from_user((void *)&k_msg, (void __user *)msg,
			   sizeof(struct lwis_event_control_list))) {
		ret = -EFAULT;
		dev_err(lwis_dev->dev, "Failed to copy ioctl message from user\n");
		return ret;
	}

	/*  Copy event controls from user buffer. */
	buf_size = sizeof(struct lwis_event_control) * k_msg.num_event_controls;
	if (buf_size / sizeof(struct lwis_event_control) != k_msg.num_event_controls) {
		dev_err(lwis_dev->dev, "Failed to copy event controls due to integer overflow.\n");
		return -EOVERFLOW;
	}
	k_event_controls = kmalloc(buf_size, GFP_KERNEL);
	if (!k_event_controls) {
		dev_err(lwis_dev->dev, "Failed to allocate event controls\n");
		return -ENOMEM;
	}
	if (copy_from_user(k_event_controls, (void __user *)k_msg.event_controls, buf_size)) {
		ret = -EFAULT;
		dev_err(lwis_dev->dev, "Failed to copy event controls from user\n");
		goto out;
	}

	for (i = 0; i < k_msg.num_event_controls; i++) {
		ret = lwis_client_event_control_set(lwis_client, &k_event_controls[i]);
		if (ret) {
			dev_err(lwis_dev->dev, "Failed to apply event control 0x%llx\n",
				k_event_controls[i].event_id);
			goto out;
		}
	}
out:
	kfree(k_event_controls);
	return ret;
}

static int ioctl_event_dequeue(struct lwis_client *lwis_client, struct lwis_event_info __user *msg)
{
	unsigned long ret = 0;
	unsigned long err = 0;
	struct lwis_event_entry *event;
	struct lwis_event_info info_user;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	bool is_error_event = false;

	if (copy_from_user((void *)&info_user, (void __user *)msg, sizeof(info_user))) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes from user\n", sizeof(info_user));
		return -EFAULT;
	}

	mutex_lock(&lwis_dev->client_lock);
	/* Peek at the front element of error event queue first */
	ret = lwis_client_error_event_peek_front(lwis_client, &event);
	if (ret == 0) {
		is_error_event = true;
	} else if (ret != -ENOENT) {
		dev_err(lwis_dev->dev, "Error dequeueing error event: %ld\n", ret);
		mutex_unlock(&lwis_dev->client_lock);
		return ret;
	} else {
		/* Nothing at error event queue, continue to check normal
		 * event queue */
		ret = lwis_client_event_peek_front(lwis_client, &event);
		if (ret) {
			if (ret != -ENOENT) {
				dev_err(lwis_dev->dev, "Error dequeueing event: %ld\n", ret);
			}
			mutex_unlock(&lwis_dev->client_lock);
			return ret;
		}
	}

	/* We need to check if we have an adequate payload buffer */
	if (event->event_info.payload_size > info_user.payload_buffer_size) {
		/* Nope, we don't. Let's inform the user and bail */
		info_user.payload_size = event->event_info.payload_size;
		err = -EAGAIN;
	} else {
		/*
		 * Let's save the IOCTL inputs because they'll get overwritten
		 */
		size_t user_buffer_size = info_user.payload_buffer_size;
		void *user_buffer = info_user.payload_buffer;

		/* Copy over the rest of the info */
		memcpy(&info_user, &event->event_info, sizeof(info_user));

		/* Restore the IOCTL inputs */
		info_user.payload_buffer_size = user_buffer_size;
		info_user.payload_buffer = user_buffer;

		/* Here we have a payload and the buffer is big enough */
		if (event->event_info.payload_size > 0 && info_user.payload_buffer) {
			/* Copy over the payload buffer to userspace */
			if (copy_to_user((void __user *)info_user.payload_buffer,
					 (void *)event->event_info.payload_buffer,
					 event->event_info.payload_size)) {
				dev_err(lwis_dev->dev, "Failed to copy %zu bytes to user\n",
					event->event_info.payload_size);
				mutex_unlock(&lwis_dev->client_lock);
				return -EFAULT;
			}
		}
	}
	/* If we didn't -EAGAIN up above, we can pop and discard the front of
	 * the event queue because we're done dealing with it. If we got the
	 * -EAGAIN case, we didn't actually dequeue this event and userspace
	 * should try again with a bigger payload_buffer.
	 */
	if (!err) {
		if (is_error_event) {
			ret = lwis_client_error_event_pop_front(lwis_client, NULL);
		} else {
			ret = lwis_client_event_pop_front(lwis_client, NULL);
		}
		if (ret) {
			dev_err(lwis_dev->dev, "Error dequeueing event: %ld\n", ret);
			mutex_unlock(&lwis_dev->client_lock);
			return ret;
		}
	}
	mutex_unlock(&lwis_dev->client_lock);
	/* Now let's copy the actual info struct back to user */
	if (copy_to_user((void __user *)msg, (void *)&info_user, sizeof(info_user))) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes to user\n", sizeof(info_user));
		return -EFAULT;
	}
	return err;
}

static int ioctl_time_query(struct lwis_client *client, int64_t __user *msg)
{
	int ret = 0;
	int64_t timestamp = ktime_to_ns(lwis_get_time());

	if (copy_to_user((void __user *)msg, &timestamp, sizeof(timestamp))) {
		ret = -EFAULT;
		dev_err(client->lwis_dev->dev, "Failed to copy timestamp to userspace\n");
	}

	return ret;
}

static int construct_io_entry(struct lwis_client *client, struct lwis_io_entry *user_entries,
			      size_t num_io_entries, struct lwis_io_entry **io_entries)
{
	int i;
	int ret = 0;
	int last_buf_alloc_idx = -1;
	size_t entry_size;
	struct lwis_io_entry *k_entries;
	uint8_t *user_buf;
	uint8_t *k_buf;
	struct lwis_device *lwis_dev = client->lwis_dev;

	entry_size = num_io_entries * sizeof(struct lwis_io_entry);
	if (entry_size / sizeof(struct lwis_io_entry) != num_io_entries) {
		dev_err(lwis_dev->dev, "Failed to prepare io entries due to integer overflow\n");
		return -EOVERFLOW;
	}
	k_entries = lwis_allocator_allocate(lwis_dev, entry_size);
	if (!k_entries) {
		dev_err(lwis_dev->dev, "Failed to allocate io entries\n");
		return -ENOMEM;
	}

	if (copy_from_user((void *)k_entries, (void __user *)user_entries, entry_size)) {
		ret = -EFAULT;
		dev_err(lwis_dev->dev, "Failed to copy io entries from user\n");
		goto error_free_entries;
	}

	/* For batch writes, ened to allocate kernel buffers to deep copy the
	 * write values. Don't need to do this for batch reads because memory
	 * will be allocated in the form of lwis_io_result in io processing.
	 */
	for (i = 0; i < num_io_entries; ++i) {
		if (k_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH) {
			user_buf = k_entries[i].rw_batch.buf;
			k_buf = lwis_allocator_allocate(lwis_dev,
							k_entries[i].rw_batch.size_in_bytes);
			if (!k_buf) {
				dev_err_ratelimited(lwis_dev->dev,
					"Failed to allocate io write buffer\n");
				ret = -ENOMEM;
				goto error_free_buf;
			}
			last_buf_alloc_idx = i;
			k_entries[i].rw_batch.buf = k_buf;
			if (copy_from_user(k_buf, (void __user *)user_buf,
					   k_entries[i].rw_batch.size_in_bytes)) {
				ret = -EFAULT;
				dev_err_ratelimited(lwis_dev->dev,
					"Failed to copy io write buffer from userspace\n");
				goto error_free_buf;
			}
		}
	}

	*io_entries = k_entries;
	return 0;

error_free_buf:
	for (i = 0; i <= last_buf_alloc_idx; ++i) {
		if (k_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH) {
			lwis_allocator_free(lwis_dev, k_entries[i].rw_batch.buf);
			k_entries[i].rw_batch.buf = NULL;
		}
	}
error_free_entries:
	lwis_allocator_free(lwis_dev, k_entries);
	*io_entries = NULL;
	return ret;
}

static int construct_transaction(struct lwis_client *client,
				 struct lwis_transaction_info __user *msg,
				 struct lwis_transaction **transaction)
{
	int ret;
	struct lwis_transaction *k_transaction;
	struct lwis_transaction_info *user_transaction;
	struct lwis_device *lwis_dev = client->lwis_dev;

	k_transaction = kmalloc(sizeof(struct lwis_transaction), GFP_KERNEL);
	if (!k_transaction) {
		dev_err(lwis_dev->dev, "Failed to allocate transaction info\n");
		return -ENOMEM;
	}

	user_transaction = (struct lwis_transaction_info *)msg;
	if (copy_from_user((void *)&k_transaction->info, (void __user *)user_transaction,
			   sizeof(struct lwis_transaction_info))) {
		ret = -EFAULT;
		dev_err(lwis_dev->dev, "Failed to copy transaction info from user\n");
		goto error_free_transaction;
	}

	ret = construct_io_entry(client, k_transaction->info.io_entries,
				 k_transaction->info.num_io_entries,
				 &k_transaction->info.io_entries);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to prepare lwis io entries for transaction\n");
		goto error_free_transaction;
	}

	k_transaction->resp = NULL;
	INIT_LIST_HEAD(&k_transaction->event_list_node);
	INIT_LIST_HEAD(&k_transaction->process_queue_node);

	*transaction = k_transaction;
	return 0;

error_free_transaction:
	kfree(k_transaction);
	return ret;
}

static int ioctl_transaction_submit(struct lwis_client *client,
				    struct lwis_transaction_info __user *msg)
{
	int ret = 0;
	unsigned long flags;
	struct lwis_transaction *k_transaction = NULL;
	struct lwis_transaction_info k_transaction_info;
	struct lwis_device *lwis_dev = client->lwis_dev;

	if (lwis_dev->type == DEVICE_TYPE_SLC) {
		dev_err(lwis_dev->dev, "not supported device type: %d\n", lwis_dev->type);
		return -EINVAL;
	}

	ret = construct_transaction(client, msg, &k_transaction);
	if (ret) {
		return ret;
	}

	spin_lock_irqsave(&client->transaction_lock, flags);
	ret = lwis_transaction_submit_locked(client, k_transaction);
	k_transaction_info = k_transaction->info;
	spin_unlock_irqrestore(&client->transaction_lock, flags);

	if (ret) {
		k_transaction_info.id = LWIS_ID_INVALID;
		lwis_transaction_free(lwis_dev, k_transaction);
	}

	if (copy_to_user((void __user *)msg, &k_transaction_info,
			 sizeof(struct lwis_transaction_info))) {
		ret = -EFAULT;
		dev_err_ratelimited(lwis_dev->dev,
				    "Failed to copy transaction results to userspace\n");
	}

	return ret;
}

static int ioctl_transaction_replace(struct lwis_client *client,
				     struct lwis_transaction_info __user *msg)
{
	int ret = 0;
	unsigned long flags;
	struct lwis_transaction *k_transaction = NULL;
	struct lwis_transaction_info k_transaction_info;
	struct lwis_device *lwis_dev = client->lwis_dev;

	ret = construct_transaction(client, msg, &k_transaction);
	if (ret) {
		return ret;
	}

	spin_lock_irqsave(&client->transaction_lock, flags);
	ret = lwis_transaction_replace_locked(client, k_transaction);
	k_transaction_info = k_transaction->info;
	spin_unlock_irqrestore(&client->transaction_lock, flags);

	if (ret) {
		k_transaction_info.id = LWIS_ID_INVALID;
		lwis_transaction_free(lwis_dev, k_transaction);
	}

	if (copy_to_user((void __user *)msg, &k_transaction_info,
			 sizeof(struct lwis_transaction_info))) {
		ret = -EFAULT;
		dev_err_ratelimited(lwis_dev->dev,
				    "Failed to copy transaction results to userspace\n");
	}

	return ret;
}

static int ioctl_transaction_cancel(struct lwis_client *client, int64_t __user *msg)
{
	int ret = 0;
	int64_t id;
	struct lwis_device *lwis_dev = client->lwis_dev;

	if (copy_from_user((void *)&id, (void __user *)msg, sizeof(id))) {
		dev_err(lwis_dev->dev, "Failed to copy transaction ID from user\n");
		return -EFAULT;
	}

	ret = lwis_transaction_cancel(client, id);
	if (ret) {
		dev_warn_ratelimited(lwis_dev->dev, "Failed to cancel transaction id 0x%llx (%d)\n",
				     id, ret);
		return ret;
	}

	return 0;
}

static int construct_periodic_io(struct lwis_client *client,
				 struct lwis_periodic_io_info __user *msg,
				 struct lwis_periodic_io **periodic_io)
{
	int ret = 0;
	struct lwis_periodic_io *k_periodic_io;
	struct lwis_periodic_io_info *user_periodic_io;
	struct lwis_device *lwis_dev = client->lwis_dev;

	k_periodic_io = kmalloc(sizeof(struct lwis_periodic_io), GFP_KERNEL);
	if (!k_periodic_io) {
		dev_err(lwis_dev->dev, "Failed to allocate periodic io\n");
		return -ENOMEM;
	}

	user_periodic_io = (struct lwis_periodic_io_info *)msg;
	if (copy_from_user((void *)&k_periodic_io->info, (void __user *)user_periodic_io,
			   sizeof(struct lwis_periodic_io_info))) {
		ret = -EFAULT;
		dev_err(lwis_dev->dev, "Failed to copy periodic io info from user\n");
		goto error_free_periodic_io;
	}

	ret = construct_io_entry(client, k_periodic_io->info.io_entries,
				 k_periodic_io->info.num_io_entries,
				 &k_periodic_io->info.io_entries);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to prepare lwis io entries for periodic io\n");
		goto error_free_periodic_io;
	}

	k_periodic_io->resp = NULL;
	k_periodic_io->periodic_io_list = NULL;

	*periodic_io = k_periodic_io;
	return 0;

error_free_periodic_io:
	kfree(k_periodic_io);
	return ret;
}

static int ioctl_periodic_io_submit(struct lwis_client *client,
				    struct lwis_periodic_io_info __user *msg)
{
	int ret = 0;
	struct lwis_periodic_io *k_periodic_io = NULL;
	struct lwis_device *lwis_dev = client->lwis_dev;

	ret = construct_periodic_io(client, msg, &k_periodic_io);
	if (ret) {
		return ret;
	}

	ret = lwis_periodic_io_submit(client, k_periodic_io);
	if (ret) {
		k_periodic_io->info.id = LWIS_ID_INVALID;
		if (copy_to_user((void __user *)msg, &k_periodic_io->info,
				 sizeof(struct lwis_periodic_io_info))) {
			dev_err_ratelimited(lwis_dev->dev, "Failed to return info to userspace\n");
		}
		lwis_periodic_io_free(lwis_dev, k_periodic_io);
		return ret;
	}

	if (copy_to_user((void __user *)msg, &k_periodic_io->info,
			 sizeof(struct lwis_periodic_io_info))) {
		dev_err_ratelimited(lwis_dev->dev,
				    "Failed to copy periodic io results to userspace\n");
		return -EFAULT;
	}

	return ret;
}

static int ioctl_periodic_io_cancel(struct lwis_client *client, int64_t __user *msg)
{
	int ret = 0;
	int64_t id;
	struct lwis_device *lwis_dev = client->lwis_dev;

	if (copy_from_user((void *)&id, (void __user *)msg, sizeof(id))) {
		dev_err(lwis_dev->dev, "Failed to copy periodic io ID from user\n");
		return -EFAULT;
	}

	ret = lwis_periodic_io_cancel(client, id);
	if (ret) {
		dev_err_ratelimited(lwis_dev->dev, "Failed to clear periodic io id 0x%llx\n", id);
		return ret;
	}

	return 0;
}

static int ioctl_dpm_clk_update(struct lwis_device *lwis_dev,
				struct lwis_dpm_clk_settings __user *msg)
{
	int ret;
	struct lwis_dpm_clk_settings k_msg;
	struct lwis_clk_setting *clk_settings;
	size_t buf_size;

	if (copy_from_user((void *)&k_msg, (void __user *)msg,
			   sizeof(struct lwis_dpm_clk_settings))) {
		dev_err(lwis_dev->dev, "Failed to copy ioctl message from user\n");
		return -EFAULT;
	}

	buf_size = sizeof(struct lwis_clk_setting) * k_msg.num_settings;
	if (buf_size / sizeof(struct lwis_clk_setting) != k_msg.num_settings) {
		dev_err(lwis_dev->dev, "Failed to copy clk settings due to integer overflow.\n");
		return -EOVERFLOW;
	}
	clk_settings = kmalloc(buf_size, GFP_KERNEL);
	if (!clk_settings) {
		dev_err(lwis_dev->dev, "Failed to allocate clock settings\n");
		return -ENOMEM;
	}

	if (copy_from_user(clk_settings, (void __user *)k_msg.settings, buf_size)) {
		dev_err(lwis_dev->dev, "Failed to copy clk settings from user\n");
		kfree(clk_settings);
		return -EFAULT;
	}

	ret = lwis_dpm_update_clock(lwis_dev, clk_settings, k_msg.num_settings);
	kfree(clk_settings);
	return ret;
}

static int ioctl_dpm_qos_update(struct lwis_device *lwis_dev,
				struct lwis_dpm_qos_requirements __user *msg)
{
	struct lwis_dpm_qos_requirements k_msg;
	struct lwis_qos_setting *k_qos_settings;
	int ret = 0;
	int i;
	size_t buf_size;

	if (lwis_dev->type != DEVICE_TYPE_DPM) {
		dev_err(lwis_dev->dev, "not supported device type: %d\n", lwis_dev->type);
		return -EINVAL;
	}

	if (copy_from_user((void *)&k_msg, (void __user *)msg,
			   sizeof(struct lwis_dpm_qos_requirements))) {
		dev_err(lwis_dev->dev, "Failed to copy ioctl message from user\n");
		return -EFAULT;
	}

	// Copy qos settings from user buffer.
	buf_size = sizeof(struct lwis_qos_setting) * k_msg.num_settings;
	if (buf_size / sizeof(struct lwis_qos_setting) != k_msg.num_settings) {
		dev_err(lwis_dev->dev, "Failed to copy qos settings due to integer overflow.\n");
		return -EOVERFLOW;
	}
	k_qos_settings = kmalloc(buf_size, GFP_KERNEL);
	if (!k_qos_settings) {
		dev_err(lwis_dev->dev, "Failed to allocate qos settings\n");
		return -ENOMEM;
	}
	if (copy_from_user(k_qos_settings, (void __user *)k_msg.qos_settings, buf_size)) {
		ret = -EFAULT;
		dev_err(lwis_dev->dev, "Failed to copy clk settings from user\n");
		goto out;
	}

	for (i = 0; i < k_msg.num_settings; i++) {
		ret = lwis_dpm_update_qos(lwis_dev, &k_qos_settings[i]);
		if (ret) {
			dev_err(lwis_dev->dev, "Failed to apply qos setting, ret: %d\n", ret);
			goto out;
		}
	}
out:
	kfree(k_qos_settings);
	return ret;
}

static int ioctl_dpm_get_clock(struct lwis_device *lwis_dev, struct lwis_qos_setting __user *msg)
{
	struct lwis_qos_setting current_setting;
	struct lwis_device *target_device;

	if (lwis_dev->type != DEVICE_TYPE_DPM) {
		dev_err(lwis_dev->dev, "not supported device type: %d\n", lwis_dev->type);
		return -EINVAL;
	}

	if (copy_from_user((void *)&current_setting, (void __user *)msg,
			   sizeof(struct lwis_qos_setting))) {
		dev_err(lwis_dev->dev, "failed to copy from user\n");
		return -EFAULT;
	}

	target_device = lwis_find_dev_by_id(current_setting.device_id);
	if (!target_device) {
		dev_err(lwis_dev->dev, "could not find lwis device by id %d\n",
			current_setting.device_id);
		return -ENODEV;
	}

	if (target_device->enabled == 0 && target_device->type != DEVICE_TYPE_DPM) {
		dev_warn(target_device->dev, "%s disabled, can't get clk\n", target_device->name);
		return -EPERM;
	}

	current_setting.frequency_hz = (int64_t)lwis_dpm_read_clock(target_device);
	if (copy_to_user((void __user *)msg, &current_setting, sizeof(struct lwis_qos_setting))) {
		dev_err(lwis_dev->dev, "failed to copy to user\n");
		return -EFAULT;
	}

	return 0;
}

int lwis_ioctl_handler(struct lwis_client *lwis_client, unsigned int type, unsigned long param)
{
	int ret = 0;
	bool device_disabled;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	// Skip the lock for LWIS_EVENT_DEQUEUE because we want to emit events ASAP. The internal
	// handler function of LWIS_EVENT_DEQUEUE will acquire the necessary lock.
	if (type != LWIS_EVENT_DEQUEUE) {
		mutex_lock(&lwis_client->lock);
	}

	mutex_lock(&lwis_dev->client_lock);
	device_disabled = (lwis_dev->enabled == 0);
	mutex_unlock(&lwis_dev->client_lock);
	/* Buffer dis/enroll is added here temporarily. Will need a proper
	   fix to ensure buffer enrollment when device is enabled. */
	if (lwis_dev->type != DEVICE_TYPE_TOP && device_disabled && type != LWIS_GET_DEVICE_INFO &&
	    type != LWIS_DEVICE_ENABLE && type != LWIS_DEVICE_RESET &&
	    type != LWIS_EVENT_CONTROL_GET && type != LWIS_TIME_QUERY &&
	    type != LWIS_EVENT_DEQUEUE && type != LWIS_BUFFER_ENROLL &&
	    type != LWIS_BUFFER_DISENROLL && type != LWIS_BUFFER_FREE &&
	    type != LWIS_DPM_QOS_UPDATE && type != LWIS_DPM_GET_CLOCK) {
		ret = -EBADFD;
		dev_err_ratelimited(lwis_dev->dev, "Unsupported IOCTL on disabled device.\n");
		goto out;
	}

	switch (type) {
	case LWIS_GET_DEVICE_INFO:
		ret = ioctl_get_device_info(lwis_dev, (struct lwis_device_info *)param);
		break;
	case LWIS_BUFFER_ALLOC:
		ret = ioctl_buffer_alloc(lwis_client, (struct lwis_alloc_buffer_info *)param);
		break;
	case LWIS_BUFFER_FREE:
		ret = ioctl_buffer_free(lwis_client, (int *)param);
		break;
	case LWIS_BUFFER_ENROLL:
		ret = ioctl_buffer_enroll(lwis_client, (struct lwis_buffer_info *)param);
		break;
	case LWIS_BUFFER_DISENROLL:
		ret = ioctl_buffer_disenroll(lwis_client,
					     (struct lwis_enrolled_buffer_info *)param);
		break;
	case LWIS_BUFFER_CPU_ACCESS:
		ret = ioctl_buffer_cpu_access(lwis_client,
					      (struct lwis_buffer_cpu_access_op *)param);
		break;
	case LWIS_REG_IO:
		ret = ioctl_reg_io(lwis_dev, (struct lwis_io_entries *)param);
		break;
	case LWIS_DEVICE_ENABLE:
		ret = ioctl_device_enable(lwis_client);
		break;
	case LWIS_DEVICE_DISABLE:
		ret = ioctl_device_disable(lwis_client);
		break;
	case LWIS_ECHO:
		ret = ioctl_echo(lwis_dev, (struct lwis_echo *)param);
		break;
	case LWIS_DEVICE_RESET:
		ret = ioctl_device_reset(lwis_client, (struct lwis_io_entries *)param);
		break;
	case LWIS_EVENT_CONTROL_GET:
		ret = ioctl_event_control_get(lwis_client, (struct lwis_event_control *)param);
		break;
	case LWIS_EVENT_CONTROL_SET:
		ret = ioctl_event_control_set(lwis_client, (struct lwis_event_control_list *)param);
		break;
	case LWIS_EVENT_DEQUEUE:
		ret = ioctl_event_dequeue(lwis_client, (struct lwis_event_info *)param);
		break;
	case LWIS_TIME_QUERY:
		ret = ioctl_time_query(lwis_client, (int64_t *)param);
		break;
	case LWIS_TRANSACTION_SUBMIT:
		ret = ioctl_transaction_submit(lwis_client, (struct lwis_transaction_info *)param);
		break;
	case LWIS_TRANSACTION_CANCEL:
		ret = ioctl_transaction_cancel(lwis_client, (int64_t *)param);
		break;
	case LWIS_TRANSACTION_REPLACE:
		ret = ioctl_transaction_replace(lwis_client, (struct lwis_transaction_info *)param);
		break;
	case LWIS_PERIODIC_IO_SUBMIT:
		ret = ioctl_periodic_io_submit(lwis_client, (struct lwis_periodic_io_info *)param);
		break;
	case LWIS_PERIODIC_IO_CANCEL:
		ret = ioctl_periodic_io_cancel(lwis_client, (int64_t *)param);
		break;
	case LWIS_DPM_CLK_UPDATE:
		ret = ioctl_dpm_clk_update(lwis_dev, (struct lwis_dpm_clk_settings *)param);
		break;
	case LWIS_DPM_QOS_UPDATE:
		ret = ioctl_dpm_qos_update(lwis_dev, (struct lwis_dpm_qos_requirements *)param);
		break;
	case LWIS_DPM_GET_CLOCK:
		ret = ioctl_dpm_get_clock(lwis_dev, (struct lwis_qos_setting *)param);
		break;
	default:
		dev_err_ratelimited(lwis_dev->dev, "Unknown IOCTL operation\n");
		ret = -EINVAL;
	};

out:
	if (type != LWIS_EVENT_DEQUEUE) {
		mutex_unlock(&lwis_client->lock);
	}

	if (ret && ret != -ENOENT && ret != -ETIMEDOUT && ret != -EAGAIN) {
		lwis_ioctl_pr_err(lwis_dev, type, ret);
	}

	return ret;
}
