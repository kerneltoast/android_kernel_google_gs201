// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS IOCTL Handler
 *
 * Copyright (c) 2018 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-ioctl: " fmt

#include "lwis_ioctl.h"

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/compiler_types.h>
#include <linux/uaccess.h>

#include "lwis_ioctl_past.h"
#include "lwis_allocator.h"
#include "lwis_buffer.h"
#include "lwis_commands.h"
#include "lwis_debug.h"
#include "lwis_device.h"
#include "lwis_device_dpm.h"
#include "lwis_device_i2c.h"
#include "lwis_device_ioreg.h"
#include "lwis_device_test.h"
#include "lwis_device_top.h"
#include "lwis_event.h"
#include "lwis_fence.h"
#include "lwis_io_buffer.h"
#include "lwis_io_entry.h"
#include "lwis_periodic_io.h"
#include "lwis_transaction.h"
#include "lwis_util.h"
#include "lwis_bus_manager.h"

#define IOCTL_TO_ENUM(x) _IOC_NR(x)
#define IOCTL_ARG_SIZE(x) _IOC_SIZE(x)
#define STRINGIFY(x) #x

#define MAX_CMD_COUNT 10
#define MAX_ECHO_SIZE (40 * 1024)

static void create_top_device_worker_thread(struct lwis_client *client)
{
	lwis_start_top_device_worker(client);
}

static void ioctl_pr_err(struct lwis_device *lwis_dev, unsigned int ioctl_type, int errno)
{
	unsigned int type = IOCTL_TO_ENUM(ioctl_type);
	static char type_name[32];
	size_t exp_size;

	switch (type) {
	case IOCTL_TO_ENUM(LWIS_CMD_PACKET):
		strscpy(type_name, STRINGIFY(LWIS_CMD_PACKET), sizeof(type_name));
		exp_size = IOCTL_ARG_SIZE(LWIS_CMD_PACKET);
		break;
	default:
		strscpy(type_name, "UNDEFINED", sizeof(type_name));
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

static int register_read(struct lwis_device *lwis_dev, struct lwis_io_entry *read_entry,
			 struct lwis_io_entry *user_msg)
{
	int ret = 0;
	uint8_t *user_buf;
	bool batch_mode = false;

	if (read_entry->type == LWIS_IO_ENTRY_READ_BATCH ||
	    read_entry->type == LWIS_IO_ENTRY_READ_BATCH_V2) {
		batch_mode = true;
		/* Save the userspace buffer address */
		user_buf = read_entry->rw_batch.buf;
		/* Allocate read buffer */
		read_entry->rw_batch.buf = lwis_allocator_allocate(
			lwis_dev, read_entry->rw_batch.size_in_bytes, GFP_KERNEL);
		if (!read_entry->rw_batch.buf) {
			dev_err_ratelimited(lwis_dev->dev,
					    "Failed to allocate register read buffer\n");
			return -ENOMEM;
		}
	} else if (read_entry->type != LWIS_IO_ENTRY_READ &&
		   read_entry->type != LWIS_IO_ENTRY_READ_V2) {
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

	if (write_entry->type == LWIS_IO_ENTRY_WRITE_BATCH ||
	    write_entry->type == LWIS_IO_ENTRY_WRITE_BATCH_V2) {
		batch_mode = true;
		/* Save the userspace buffer address */
		user_buf = write_entry->rw_batch.buf;
		/* Allocate write buffer and copy contents from userspace */
		write_entry->rw_batch.buf = lwis_allocator_allocate(
			lwis_dev, write_entry->rw_batch.size_in_bytes, GFP_KERNEL);
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
	} else if (write_entry->type != LWIS_IO_ENTRY_WRITE &&
		   write_entry->type != LWIS_IO_ENTRY_WRITE_V2) {
		/* Type must be either WRITE or WRITE_BATCH */
		dev_err(lwis_dev->dev, "Invalid io_entry type for REGISTER_WRITE\n");
		return -EINVAL;
	}

	ret = lwis_dev->vops.register_io(lwis_dev, write_entry, lwis_dev->native_value_bitwidth);
	if (ret)
		dev_err_ratelimited(lwis_dev->dev, "Failed to write registers\n");

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
	if (ret)
		dev_err_ratelimited(lwis_dev->dev, "Failed to read registers for modify\n");

	return ret;
}

static int synchronous_process_io_entries(struct lwis_device *lwis_dev, int num_io_entries,
					  struct lwis_io_entry *io_entries,
					  struct lwis_io_entry *user_msg, bool skip_error)
{
	int ret = 0;
	int last_error = 0;
	int i = 0;

	lwis_bus_manager_lock_bus(lwis_dev);
	/* Use write memory barrier at the beginning of I/O entries if the access protocol
	 * allows it
	 */
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
		case LWIS_IO_ENTRY_READ_V2:
		case LWIS_IO_ENTRY_READ_BATCH:
		case LWIS_IO_ENTRY_READ_BATCH_V2:
			ret = register_read(lwis_dev, &io_entries[i], user_msg + i);
			break;
		case LWIS_IO_ENTRY_WRITE:
		case LWIS_IO_ENTRY_WRITE_V2:
		case LWIS_IO_ENTRY_WRITE_BATCH:
		case LWIS_IO_ENTRY_WRITE_BATCH_V2:
			ret = register_write(lwis_dev, &io_entries[i]);
			break;
		case LWIS_IO_ENTRY_POLL:
			ret = lwis_io_entry_poll(lwis_dev, &io_entries[i], /*is_short=*/false);
			break;
		case LWIS_IO_ENTRY_POLL_SHORT:
			ret = lwis_io_entry_poll(lwis_dev, &io_entries[i], /*is_short=*/true);
			break;
		case LWIS_IO_ENTRY_WAIT:
			ret = lwis_io_entry_wait(lwis_dev, &io_entries[i]);
			break;
		case LWIS_IO_ENTRY_READ_ASSERT:
			ret = lwis_io_entry_read_assert(lwis_dev, &io_entries[i]);
			break;
		case LWIS_IO_ENTRY_WRITE_TO_BUFFER:
			ret = lwis_io_buffer_write(lwis_dev, &io_entries[i]);
			break;
		case LWIS_IO_ENTRY_IGNORE:
			ret = 0;
			break;
		default:
			dev_err(lwis_dev->dev, "Unknown io_entry operation\n");
			ret = -EINVAL;
		}

		if (ret) {
			last_error = ret;
			if (skip_error) {
				dev_warn(
					lwis_dev->dev,
					"IO type %d processing failed, skipping error and running next command\n",
					io_entries[i].type);
			} else {
				dev_err(lwis_dev->dev, "Register io_entry failed\n");
				goto exit;
			}
		}
	}
exit:
	/* Use read memory barrier at the end of I/O entries if the access protocol
	 * allows it
	 */
	if (lwis_dev->vops.register_io_barrier != NULL) {
		lwis_dev->vops.register_io_barrier(lwis_dev,
						   /*use_read_barrier=*/true,
						   /*use_write_barrier=*/false);
	}
	lwis_bus_manager_unlock_bus(lwis_dev);

	return last_error;
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
	/* Following variables are used to avoid lwis integer overflow */
	int read_entries = 0;
	size_t read_buf_size = 0;
	const int reg_value_bytewidth = client->lwis_dev->native_value_bitwidth / 8;

	entry_size = num_io_entries * sizeof(struct lwis_io_entry);
	if (entry_size / sizeof(struct lwis_io_entry) != num_io_entries) {
		dev_err(lwis_dev->dev, "Failed to prepare io entries due to integer overflow\n");
		return -EOVERFLOW;
	}
	k_entries = lwis_allocator_allocate(lwis_dev, entry_size, GFP_KERNEL);
	if (!k_entries) {
		dev_err(lwis_dev->dev, "Failed to allocate io entries\n");
		return -ENOMEM;
	}

	if (copy_from_user((void *)k_entries, (void __user *)user_entries, entry_size)) {
		ret = -EFAULT;
		dev_err(lwis_dev->dev, "Failed to copy io entries from user\n");
		goto error_free_entries;
	}

	/*
	 * For batch writes, need to allocate kernel buffers to deep copy the
	 * write values. Don't need to do this for batch reads because memory
	 * will be allocated in the form of lwis_io_result in io processing.
	 */
	for (i = 0; i < num_io_entries; ++i) {
		const size_t remaining_capacity = LWIS_IO_ENTRY_READ_RESTRICTION - read_buf_size -
						  read_entries * sizeof(struct lwis_io_result);
		if (k_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH ||
		    k_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH_V2) {
			user_buf = k_entries[i].rw_batch.buf;
			k_buf = lwis_allocator_allocate(
				lwis_dev, k_entries[i].rw_batch.size_in_bytes, GFP_KERNEL);
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
				dev_err_ratelimited(
					lwis_dev->dev,
					"Failed to copy io write buffer from userspace\n");
				goto error_free_buf;
			}
		} else if (k_entries[i].type == LWIS_IO_ENTRY_WRITE_TO_BUFFER) {
			struct dma_buf *dma_buffer = dma_buf_get(k_entries[i].write_to_buffer.fd);
			struct iosys_map *sys_map = NULL;
			uint8_t *k_bytes_from_userspace = NULL;

			if (IS_ERR(dma_buffer)) {
				dev_err(lwis_dev->dev,
					"PDMA buffer IO failed because dma_buf_get failed");
				ret = -EFAULT;
				goto error_free_buf;
			}

			sys_map = kmalloc(sizeof(struct iosys_map), GFP_KERNEL);
			if (!sys_map) {
				dma_buf_put(dma_buffer);
				ret = -EFAULT;
				goto error_free_buf;
			}

			k_bytes_from_userspace =
				kmalloc(k_entries[i].write_to_buffer.size_in_bytes, GFP_KERNEL);
			if (!k_bytes_from_userspace) {
				dma_buf_put(dma_buffer);
				ret = -EFAULT;
				kfree(sys_map);
				goto error_free_buf;
			}

			if (copy_from_user(k_bytes_from_userspace,
					   (void __user *)(k_entries[i].write_to_buffer.bytes),
					   k_entries[i].write_to_buffer.size_in_bytes)) {
				dev_err(lwis_dev->dev,
					"PDMA buffer IO failed because bytes cannot be copied from user space");
				dma_buf_put(dma_buffer);
				ret = -EFAULT;
				kfree(sys_map);
				kfree(k_bytes_from_userspace);
				goto error_free_buf;
			}
			k_entries[i].write_to_buffer.bytes = k_bytes_from_userspace;

			if (dma_buf_vmap(dma_buffer, sys_map) < 0) {
				dev_err(lwis_dev->dev, "PDMA buffer IO failed because vmap failed");
				dma_buf_put(dma_buffer);
				ret = -EFAULT;
				kfree(sys_map);
				kfree(k_bytes_from_userspace);
				goto error_free_buf;
			}

			if (dma_buf_begin_cpu_access(dma_buffer, DMA_BIDIRECTIONAL) < 0) {
				dev_err(lwis_dev->dev,
					"PDMA buffer IO failed because CPU cannot have access to the buffer");
				dma_buf_vunmap(dma_buffer, sys_map);
				dma_buf_put(dma_buffer);
				ret = -EFAULT;
				kfree(sys_map);
				kfree(k_bytes_from_userspace);
				goto error_free_buf;
			}
			k_entries[i].write_to_buffer.buffer =
				kmalloc(sizeof(struct pdma_buffer), GFP_KERNEL);

			if (!k_entries[i].write_to_buffer.buffer) {
				dma_buf_end_cpu_access(dma_buffer, DMA_BIDIRECTIONAL);
				dma_buf_vunmap(dma_buffer, sys_map);
				dma_buf_put(dma_buffer);
				ret = -EFAULT;
				kfree(sys_map);
				kfree(k_bytes_from_userspace);
				goto error_free_buf;
			}

			k_entries[i].write_to_buffer.buffer->io_sys_map = sys_map;
			k_entries[i].write_to_buffer.buffer->dma_buf = dma_buffer;
			last_buf_alloc_idx = i;
		} else if (k_entries[i].type == LWIS_IO_ENTRY_READ ||
			   k_entries[i].type == LWIS_IO_ENTRY_READ_V2) {
			/* Check for size_t overflow. */
			if (reg_value_bytewidth > remaining_capacity ||
			    ++read_entries >= LWIS_IO_ENTRY_READ_OVERFLOW_BOUND) {
				ret = -EOVERFLOW;
				goto error_free_buf;
			}
			read_buf_size += reg_value_bytewidth;
		} else if (k_entries[i].type == LWIS_IO_ENTRY_READ_BATCH ||
			   k_entries[i].type == LWIS_IO_ENTRY_READ_BATCH_V2) {
			/* Check for size_t overflow. */
			if (k_entries[i].rw_batch.size_in_bytes > remaining_capacity ||
			    ++read_entries >= LWIS_IO_ENTRY_READ_OVERFLOW_BOUND) {
				ret = -EOVERFLOW;
				goto error_free_buf;
			}
			read_buf_size += k_entries[i].rw_batch.size_in_bytes;
		}
	}

	*io_entries = k_entries;
	return 0;

error_free_buf:
	for (i = 0; i <= last_buf_alloc_idx; ++i) {
		if (k_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH ||
		    k_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH_V2) {
			lwis_allocator_free(lwis_dev, k_entries[i].rw_batch.buf);
			k_entries[i].rw_batch.buf = NULL;
		} else if (k_entries[i].type == LWIS_IO_ENTRY_WRITE_TO_BUFFER) {
			void *sys_map = k_entries[i].write_to_buffer.buffer->io_sys_map;
			void *dma_buffer = k_entries[i].write_to_buffer.buffer->dma_buf;

			dma_buf_end_cpu_access(dma_buffer, DMA_BIDIRECTIONAL);
			dma_buf_vunmap(dma_buffer, sys_map);
			dma_buf_put(dma_buffer);
			kfree(sys_map);
			kfree(k_entries[i].write_to_buffer.bytes);
			kfree(k_entries[i].write_to_buffer.buffer);
		}
	}
error_free_entries:
	lwis_allocator_free(lwis_dev, k_entries);
	*io_entries = NULL;
	return ret;
}

static int copy_pkt_to_user(struct lwis_device *lwis_dev, void __user *u_msg, void *k_msg,
			    size_t size)
{
	if (copy_to_user(u_msg, k_msg, size)) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes to user\n", size);
		return -EFAULT;
	}

	return 0;
}

static int cmd_echo(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
		    struct lwis_cmd_echo __user *u_msg)
{
	struct lwis_cmd_echo echo_msg;
	char *buffer = NULL;

	if (copy_from_user((void *)&echo_msg, (void __user *)u_msg, sizeof(echo_msg))) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes from user\n", sizeof(echo_msg));
		return -EFAULT;
	}

	if (echo_msg.msg.size == 0) {
		header->ret_code = 0;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	if (echo_msg.msg.size > MAX_ECHO_SIZE) {
		dev_err(lwis_dev->dev, "Echo message size over the limit\n");
		header->ret_code = -EINVAL;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	buffer = kmalloc(echo_msg.msg.size + 1, GFP_KERNEL);
	if (!buffer) {
		header->ret_code = -ENOMEM;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}
	if (copy_from_user(buffer, (void __user *)echo_msg.msg.msg, echo_msg.msg.size)) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes echo message from user\n",
			echo_msg.msg.size);
		kfree(buffer);
		header->ret_code = -EFAULT;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}
	buffer[echo_msg.msg.size] = '\0';

	if (echo_msg.msg.kernel_log)
		dev_info(lwis_dev->dev, "LWIS_ECHO: %s\n", buffer);

	kfree(buffer);

	header->ret_code = 0;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_time_query(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
			  struct lwis_cmd_time_query __user *u_msg)
{
	struct lwis_cmd_time_query time_query;

	time_query.timestamp_ns = ktime_to_ns(lwis_get_time());
	time_query.header.cmd_id = header->cmd_id;
	time_query.header.next = header->next;
	time_query.header.ret_code = 0;

	return copy_pkt_to_user(lwis_dev, u_msg, (void *)&time_query, sizeof(time_query));
}

static int cmd_get_device_info(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
			       struct lwis_cmd_device_info __user *u_msg)
{
	int ret = 0;
	int i;
	struct lwis_cmd_device_info *k_info;

	k_info = kmalloc(sizeof(*k_info), GFP_KERNEL);
	if (!k_info)
		return -ENOMEM;

	k_info->header.cmd_id = header->cmd_id;
	k_info->header.next = header->next;
	k_info->info.id = lwis_dev->id;
	k_info->info.type = lwis_dev->type;
	k_info->info.num_clks = 0;
	k_info->info.num_regs = 0;
	k_info->info.transaction_worker_thread_pid = -1;
	k_info->info.periodic_io_thread_pid = -1;
	strscpy(k_info->info.name, lwis_dev->name, LWIS_MAX_NAME_STRING_LEN);

	mutex_lock(&lwis_dev->interclient_lock);
	if (lwis_dev->clocks) {
		k_info->info.num_clks = lwis_dev->clocks->count;
		for (i = 0; i < lwis_dev->clocks->count; i++) {
			if (i >= LWIS_MAX_CLOCK_NUM) {
				dev_err(lwis_dev->dev,
					"Clock count larger than LWIS_MAX_CLOCK_NUM\n");
				break;
			}
			strscpy(k_info->info.clks[i].name, lwis_dev->clocks->clk[i].name,
				LWIS_MAX_NAME_STRING_LEN);
			k_info->info.clks[i].clk_index = i;
			k_info->info.clks[i].frequency = 0;
		}
	}

	if (lwis_dev->type == DEVICE_TYPE_IOREG) {
		struct lwis_ioreg_device *ioreg_dev;

		ioreg_dev = container_of(lwis_dev, struct lwis_ioreg_device, base_dev);
		if (ioreg_dev->reg_list.count > 0) {
			k_info->info.num_regs = ioreg_dev->reg_list.count;
			for (i = 0; i < ioreg_dev->reg_list.count; i++) {
				if (i >= LWIS_MAX_REG_NUM) {
					dev_err(lwis_dev->dev,
						"Reg count larger than LWIS_MAX_REG_NUM\n");
					break;
				}
				strscpy(k_info->info.regs[i].name,
					ioreg_dev->reg_list.block[i].name,
					LWIS_MAX_NAME_STRING_LEN);
				k_info->info.regs[i].reg_index = i;
				k_info->info.regs[i].start = ioreg_dev->reg_list.block[i].start;
				k_info->info.regs[i].size = ioreg_dev->reg_list.block[i].size;
			}
		}
	}

	/*
	 * Send kworker thread pid to userspace so that they can be added to the camera vendor
	 * group for correct performance settings.
	 */
	if (lwis_dev->type == DEVICE_TYPE_I2C) {
		/* For I2C devices, transactions are being run in the I2C bus manager thread */
		struct lwis_i2c_device *i2c_dev;

		i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
		k_info->info.transaction_worker_thread_pid =
			i2c_dev->i2c_bus_manager->bus_worker_thread->pid;
	} else if (lwis_dev->type == DEVICE_TYPE_IOREG) {
		/*
		 * For IOREG devices, default behaviour is to run on their own transaction threads.
		 * Devices that are grouped will run transactions in the IOREG bus manager thread.
		 */
		struct lwis_ioreg_device *ioreg_dev;

		ioreg_dev = container_of(lwis_dev, struct lwis_ioreg_device, base_dev);
		if (ioreg_dev->ioreg_bus_manager) {
			k_info->info.transaction_worker_thread_pid =
				ioreg_dev->ioreg_bus_manager->bus_worker_thread->pid;
		} else {
			k_info->info.transaction_worker_thread_pid =
				lwis_dev->transaction_worker_thread->pid;
		}
	} else if (lwis_dev->type == DEVICE_TYPE_TOP) {
		/* For top device, the event subscription thread is the main worker thread */
		struct lwis_top_device *top_dev;

		top_dev = container_of(lwis_dev, struct lwis_top_device, base_dev);
		k_info->info.transaction_worker_thread_pid = top_dev->subscribe_worker_thread->pid;
	} else if (lwis_dev->transaction_worker_thread) {
		/* For all other device types, transaction threads are the main worker threads */
		k_info->info.transaction_worker_thread_pid =
			lwis_dev->transaction_worker_thread->pid;
	}
	mutex_unlock(&lwis_dev->interclient_lock);

	k_info->header.ret_code = 0;
	ret = copy_pkt_to_user(lwis_dev, u_msg, k_info, sizeof(*k_info));
	kfree(k_info);
	return ret;
}

static int cmd_device_enable(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
			     struct lwis_cmd_pkt __user *u_msg)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	mutex_lock(&lwis_client->lock);
	if (lwis_client->is_enabled) {
		goto exit_client_locked;
	}

	mutex_lock(&lwis_dev->interclient_lock);
	if (lwis_dev->enabled > 0 && lwis_dev->enabled < INT_MAX) {
		lwis_dev->enabled++;
		lwis_client->is_enabled = true;
		ret = 0;
		goto exit_locked;
	} else if (lwis_dev->enabled == INT_MAX) {
		dev_err(lwis_dev->dev, "Enable counter overflow\n");
		ret = -EINVAL;
		goto exit_locked;
	}

	/* Clear event queues to make sure there is no stale event from previous session. */
	lwis_client_event_queue_clear(lwis_client);
	lwis_client_error_event_queue_clear(lwis_client);

	ret = lwis_dev_power_up_locked(lwis_dev);
	if (ret < 0) {
		dev_err(lwis_dev->dev, "Failed to power up device\n");
		goto exit_locked;
	}

	lwis_dev->enabled++;
	lwis_client->is_enabled = true;
	lwis_dev->is_suspended = lwis_dev->power_up_to_suspend;
exit_locked:
	mutex_unlock(&lwis_dev->interclient_lock);
exit_client_locked:
	mutex_unlock(&lwis_client->lock);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_device_disable(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
			      struct lwis_cmd_pkt __user *u_msg)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	mutex_lock(&lwis_client->lock);
	if (!lwis_client->is_enabled) {
		goto exit_client_locked;
	}

	mutex_lock(&lwis_dev->interclient_lock);
	/* Clear event states for this client */
	lwis_client_event_states_clear(lwis_client);
	mutex_unlock(&lwis_dev->interclient_lock);

	/* Flush all periodic io to complete */
	ret = lwis_periodic_io_client_flush(lwis_client);
	if (ret)
		dev_err(lwis_dev->dev, "Failed to wait for in-process periodic io to complete\n");

	/* Flush all pending transactions */
	ret = lwis_transaction_client_flush(lwis_client);
	if (ret)
		dev_err(lwis_dev->dev, "Failed to flush pending transactions\n");

	/* Run cleanup transactions. */
	lwis_transaction_client_cleanup(lwis_client);

	mutex_lock(&lwis_dev->interclient_lock);
	if (lwis_dev->enabled > 1) {
		lwis_dev->enabled--;
		lwis_client->is_enabled = false;
		ret = 0;
		goto exit_locked;
	} else if (lwis_dev->enabled <= 0) {
		dev_err(lwis_dev->dev, "Disabling a device that is already disabled\n");
		ret = -EINVAL;
		goto exit_locked;
	}

	ret = lwis_dev_power_down_locked(lwis_dev);
	if (ret < 0) {
		dev_err(lwis_dev->dev, "Failed to power down device\n");
		goto exit_locked;
	}
	lwis_device_event_states_clear_locked(lwis_dev);

	lwis_dev->enabled--;
	lwis_client->is_enabled = false;
	lwis_dev->is_suspended = false;
	dev_info(lwis_dev->dev, "Device disabled\n");
exit_locked:
	mutex_unlock(&lwis_dev->interclient_lock);
exit_client_locked:
	mutex_unlock(&lwis_client->lock);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int copy_io_entries_from_cmd(struct lwis_device *lwis_dev,
				    struct lwis_cmd_io_entries_v2 *k_msg,
				    struct lwis_io_entry **k_entries)
{
	struct lwis_io_entry *io_entries;
	uint32_t buf_size;

	buf_size = sizeof(struct lwis_io_entry) * k_msg->io.num_io_entries;
	if (buf_size / sizeof(struct lwis_io_entry) != k_msg->io.num_io_entries) {
		dev_err(lwis_dev->dev, "Failed to copy io_entries due to integer overflow.\n");
		return -EOVERFLOW;
	}
	io_entries = lwis_allocator_allocate(lwis_dev, buf_size, GFP_KERNEL);
	if (!io_entries) {
		dev_err(lwis_dev->dev, "Failed to allocate io_entries buffer\n");
		return -ENOMEM;
	}
	if (copy_from_user(io_entries, (void __user *)k_msg->io.io_entries, buf_size)) {
		dev_err(lwis_dev->dev, "Failed to copy io_entries from userspace.\n");
		lwis_allocator_free(lwis_dev, io_entries);
		return -EFAULT;
	}
	*k_entries = io_entries;

	return 0;
}

static int cmd_device_reset(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
			    struct lwis_cmd_io_entries __user *u_msg)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	struct lwis_cmd_io_entries_v2 k_msg;
	struct lwis_io_entry *k_entries = NULL;
	bool device_enabled = false;

	/* Register io is not supported for the lwis device, return */
	if (!lwis_dev->vops.register_io) {
		dev_err(lwis_dev->dev, "Register IO not supported on this LWIS device\n");
		return -EINVAL;
	}

	/* Copy io_entries from userspace */
	if (copy_from_user((void *)&k_msg, (void __user *)u_msg,
			   sizeof(struct lwis_cmd_io_entries))) {
		dev_err(lwis_dev->dev, "Failed to copy io_entries header from userspace.\n");
		return -EFAULT;
	}
	k_msg.skip_error = false;

	ret = copy_io_entries_from_cmd(lwis_dev, &k_msg, &k_entries);
	if (ret)
		goto soft_reset_exit;

	mutex_lock(&lwis_client->lock);
	mutex_lock(&lwis_dev->interclient_lock);
	/* Clear event states, event queues and transactions for this client */
	lwis_client_event_states_clear(lwis_client);
	lwis_client_event_queue_clear(lwis_client);
	lwis_client_error_event_queue_clear(lwis_client);
	device_enabled = lwis_dev->enabled;
	mutex_unlock(&lwis_dev->interclient_lock);

	/* Flush all periodic io to complete */
	ret = lwis_periodic_io_client_flush(lwis_client);
	if (ret)
		dev_err(lwis_dev->dev, "Failed to wait for in-process periodic io to complete\n");

	/* Flush all pending transactions */
	ret = lwis_transaction_client_flush(lwis_client);
	if (ret)
		dev_err(lwis_dev->dev, "Failed to flush all pending transactions\n");

	/* Perform reset routine defined by the io_entries */
	if (device_enabled)
		ret = synchronous_process_io_entries(lwis_dev, k_msg.io.num_io_entries, k_entries,
						     k_msg.io.io_entries, k_msg.skip_error);
	else
		dev_warn(lwis_dev->dev,
			 "Device is not enabled, IoEntries will not be executed in DEVICE_RESET\n");

	mutex_lock(&lwis_dev->interclient_lock);
	lwis_device_event_states_clear_locked(lwis_dev);
	mutex_unlock(&lwis_dev->interclient_lock);
	mutex_unlock(&lwis_client->lock);
soft_reset_exit:
	if (k_entries)
		lwis_allocator_free(lwis_dev, k_entries);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_device_suspend(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
			      struct lwis_cmd_pkt __user *u_msg)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (!lwis_dev->suspend_sequence) {
		dev_err(lwis_dev->dev, "No suspend sequence defined\n");
		header->ret_code = 0;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	mutex_lock(&lwis_client->lock);
	if (!lwis_client->is_enabled) {
		dev_err(lwis_dev->dev, "Trying to suspend a disabled device\n");
		ret = -EINVAL;
		goto exit_client_locked;
	}

	mutex_lock(&lwis_dev->interclient_lock);
	if (lwis_dev->is_suspended) {
		goto exit_locked;
	}

	/* Clear event states for this client */
	lwis_client_event_states_clear(lwis_client);
	mutex_unlock(&lwis_dev->interclient_lock);

	/* Flush all periodic io to complete */
	ret = lwis_periodic_io_client_flush(lwis_client);
	if (ret)
		dev_err(lwis_dev->dev, "Failed to wait for in-process periodic io to complete\n");

	/* Flush all pending transactions */
	ret = lwis_transaction_client_flush(lwis_client);
	if (ret)
		dev_err(lwis_dev->dev, "Failed to flush pending transactions\n");

	/* Run cleanup transactions. */
	lwis_transaction_client_cleanup(lwis_client);

	mutex_lock(&lwis_dev->interclient_lock);
	ret = lwis_dev_process_power_sequence(lwis_dev, lwis_dev->suspend_sequence,
					      /*set_active=*/false, /*skip_error=*/false);
	if (ret) {
		dev_err(lwis_dev->dev, "Error lwis_dev_process_power_sequence (%d)\n", ret);
		goto exit_locked;
	}

	lwis_device_event_states_clear_locked(lwis_dev);

	lwis_dev->is_suspended = true;
	dev_info(lwis_dev->dev, "Device suspended\n");
exit_locked:
	mutex_unlock(&lwis_dev->interclient_lock);
exit_client_locked:
	mutex_unlock(&lwis_client->lock);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_device_resume(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
			     struct lwis_cmd_pkt __user *u_msg)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (!lwis_dev->resume_sequence) {
		dev_err(lwis_dev->dev, "No resume sequence defined\n");
		header->ret_code = 0;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	mutex_lock(&lwis_client->lock);
	mutex_lock(&lwis_dev->interclient_lock);
	if (!lwis_dev->is_suspended) {
		goto exit_locked;
	}

	/* Clear event queues to make sure there is no stale event from
	 * previous session
	 */
	lwis_client_event_queue_clear(lwis_client);
	lwis_client_error_event_queue_clear(lwis_client);

	ret = lwis_dev_process_power_sequence(lwis_dev, lwis_dev->resume_sequence,
					      /*set_active=*/true, /*skip_error=*/false);
	if (ret) {
		dev_err(lwis_dev->dev, "Error lwis_dev_process_power_sequence (%d)\n", ret);
		goto exit_locked;
	}

	lwis_dev->is_suspended = false;
	dev_info(lwis_dev->dev, "Device resumed\n");
exit_locked:
	mutex_unlock(&lwis_dev->interclient_lock);
	mutex_unlock(&lwis_client->lock);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_dump_debug_state(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
				struct lwis_cmd_pkt __user *u_msg)
{
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	mutex_lock(&lwis_dev->interclient_lock);
	/* Dump lwis device crash info */
	lwis_debug_crash_info_dump(lwis_dev);
	mutex_unlock(&lwis_dev->interclient_lock);

	header->ret_code = 0;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_get_device_enable_state(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
				       struct lwis_cmd_get_device_enable_state __user *u_msg)
{
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	struct lwis_cmd_get_device_enable_state enable_state;

	if (copy_from_user((void *)&enable_state, (void __user *)u_msg, sizeof(enable_state))) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes from user\n",
			sizeof(enable_state));
		return -EFAULT;
	}

	mutex_lock(&lwis_client->lock);
	mutex_lock(&lwis_dev->interclient_lock);
	if (lwis_dev->enabled) {
		if (lwis_dev->is_suspended)
			enable_state.state = DEVICE_ENABLE_STATE_SUSPEND;
		else
			enable_state.state = DEVICE_ENABLE_STATE_ENABLE;
	} else {
		enable_state.state = DEVICE_ENABLE_STATE_DISABLE;
	}
	mutex_unlock(&lwis_dev->interclient_lock);
	mutex_unlock(&lwis_client->lock);
	enable_state.header.ret_code = 0;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)&enable_state, sizeof(enable_state));
}

static int cmd_dma_buffer_enroll(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
				 struct lwis_cmd_dma_buffer_enroll __user *u_msg)
{
	int ret = 0;
	struct lwis_cmd_dma_buffer_enroll buf_info;
	struct lwis_enrolled_buffer *buffer;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	buffer = kmalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		header->ret_code = -ENOMEM;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	if (copy_from_user((void *)&buf_info, (void __user *)u_msg, sizeof(buf_info))) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes from user\n", sizeof(buf_info));
		ret = -EFAULT;
		goto error_enroll;
	}

	buffer->info.fd = buf_info.info.fd;
	buffer->info.dma_read = buf_info.info.dma_read;
	buffer->info.dma_write = buf_info.info.dma_write;

	mutex_lock(&lwis_client->lock);
	ret = lwis_buffer_enroll(lwis_client, buffer);
	mutex_unlock(&lwis_client->lock);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to enroll buffer\n");
		goto error_enroll;
	}

	buf_info.info.dma_vaddr = buffer->info.dma_vaddr;
	buf_info.header.cmd_id = header->cmd_id;
	buf_info.header.next = header->next;
	buf_info.header.ret_code = ret;
	ret = copy_pkt_to_user(lwis_dev, u_msg, (void *)&buf_info, sizeof(buf_info));
	if (ret) {
		mutex_lock(&lwis_client->lock);
		lwis_buffer_disenroll(lwis_client, buffer);
		mutex_unlock(&lwis_client->lock);
		goto error_enroll;
	}

	return ret;

error_enroll:
	kfree(buffer);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_dma_buffer_disenroll(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
				    struct lwis_cmd_dma_buffer_disenroll __user *u_msg)
{
	int ret = 0;
	struct lwis_cmd_dma_buffer_disenroll info;
	struct lwis_enrolled_buffer *buffer;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (copy_from_user((void *)&info, (void __user *)u_msg, sizeof(info))) {
		dev_err(lwis_dev->dev, "Failed to copy DMA virtual address from user\n");
		return -EFAULT;
	}

	mutex_lock(&lwis_client->lock);
	buffer = lwis_client_enrolled_buffer_find(lwis_client, info.info.fd, info.info.dma_vaddr);
	if (!buffer) {
		mutex_unlock(&lwis_client->lock);
		dev_err(lwis_dev->dev, "Failed to find dma buffer for fd %d vaddr %pad\n",
			info.info.fd, &info.info.dma_vaddr);
		header->ret_code = -ENOENT;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	ret = lwis_buffer_disenroll(lwis_client, buffer);
	mutex_unlock(&lwis_client->lock);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to disenroll dma buffer for fd %d vaddr %pad\n",
			info.info.fd, &info.info.dma_vaddr);
		header->ret_code = ret;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	kfree(buffer);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_dma_buffer_cpu_access(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
				     struct lwis_cmd_dma_buffer_cpu_access __user *u_msg)
{
	int ret = 0;
	struct lwis_cmd_dma_buffer_cpu_access op;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (copy_from_user((void *)&op, (void __user *)u_msg, sizeof(op))) {
		dev_err(lwis_dev->dev, "Failed to copy buffer CPU access operation from user\n");
		return -EFAULT;
	}

	mutex_lock(&lwis_client->lock);
	ret = lwis_buffer_cpu_access(lwis_client, &op.op);
	mutex_unlock(&lwis_client->lock);

	if (ret)
		dev_err(lwis_dev->dev, "Failed to prepare for cpu access for fd %d\n", op.op.fd);

	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_dma_buffer_alloc(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
				struct lwis_cmd_dma_buffer_alloc __user *u_msg)
{
	int ret = 0;
	struct lwis_cmd_dma_buffer_alloc alloc_info;
	struct lwis_allocated_buffer *buffer;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	buffer = kmalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	if (copy_from_user((void *)&alloc_info, (void __user *)u_msg, sizeof(alloc_info))) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes from user\n", sizeof(alloc_info));
		ret = -EFAULT;
		goto error_alloc;
	}

	mutex_lock(&lwis_client->lock);
	ret = lwis_buffer_alloc(lwis_client, &alloc_info.info, buffer);
	mutex_unlock(&lwis_client->lock);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to allocate buffer\n");
		goto error_alloc;
	}

	alloc_info.header.ret_code = 0;
	ret = copy_pkt_to_user(lwis_dev, u_msg, (void *)&alloc_info, sizeof(alloc_info));
	if (ret) {
		mutex_lock(&lwis_client->lock);
		lwis_buffer_free(lwis_client, buffer);
		mutex_unlock(&lwis_client->lock);
		ret = -EFAULT;
		goto error_alloc;
	}

	return ret;

error_alloc:
	kfree(buffer);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_dma_buffer_free(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
			       struct lwis_cmd_dma_buffer_free __user *u_msg)
{
	int ret = 0;
	struct lwis_cmd_dma_buffer_free info;
	struct lwis_allocated_buffer *buffer;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (copy_from_user((void *)&info, (void __user *)u_msg, sizeof(info))) {
		dev_err(lwis_dev->dev, "Failed to copy file descriptor from user\n");
		return -EFAULT;
	}

	mutex_lock(&lwis_client->lock);
	buffer = lwis_client_allocated_buffer_find(lwis_client, info.fd);
	if (!buffer) {
		mutex_unlock(&lwis_client->lock);
		dev_err(lwis_dev->dev, "Cannot find allocated buffer FD %d\n", info.fd);
		header->ret_code = -ENOENT;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	ret = lwis_buffer_free(lwis_client, buffer);
	mutex_unlock(&lwis_client->lock);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to free buffer FD %d\n", info.fd);
		header->ret_code = ret;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	kfree(buffer);

	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_reg_io(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
		      struct lwis_cmd_io_entries __user *u_msg)
{
	int ret = 0;
	struct lwis_cmd_io_entries_v2 k_msg;
	struct lwis_io_entry *k_entries = NULL;

	/* Register io is not supported for the lwis device, return */
	if (!lwis_dev->vops.register_io) {
		dev_err(lwis_dev->dev, "Register IO not supported on this LWIS device\n");
		return -EINVAL;
	}

	/* Copy io_entries from userspace */
	if (copy_from_user((void *)&k_msg, (void __user *)u_msg,
			   sizeof(struct lwis_cmd_io_entries))) {
		dev_err(lwis_dev->dev, "Failed to copy io_entries header from userspace.\n");
		return -EFAULT;
	}
	k_msg.skip_error = false;

	ret = copy_io_entries_from_cmd(lwis_dev, &k_msg, &k_entries);
	if (ret)
		goto reg_io_exit;

	/* Walk through and execute the entries */
	mutex_lock(&lwis_dev->interclient_lock);
	ret = synchronous_process_io_entries(lwis_dev, k_msg.io.num_io_entries, k_entries,
					     k_msg.io.io_entries, k_msg.skip_error);
	mutex_unlock(&lwis_dev->interclient_lock);

reg_io_exit:
	if (k_entries)
		lwis_allocator_free(lwis_dev, k_entries);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_reg_io_v2(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
			 struct lwis_cmd_io_entries_v2 __user *u_msg)
{
	int ret = 0;
	struct lwis_cmd_io_entries_v2 k_msg;
	struct lwis_io_entry *k_entries = NULL;

	/* Register io is not supported for the lwis device, return */
	if (!lwis_dev->vops.register_io) {
		dev_err(lwis_dev->dev, "Register IO not supported on this LWIS device\n");
		return -EINVAL;
	}

	/* Copy io_entries from userspace */
	if (copy_from_user((void *)&k_msg, (void __user *)u_msg, sizeof(k_msg))) {
		dev_err(lwis_dev->dev, "Failed to copy io_entries header from userspace.\n");
		return -EFAULT;
	}

	ret = copy_io_entries_from_cmd(lwis_dev, &k_msg, &k_entries);
	if (ret)
		goto reg_io_exit;

	mutex_lock(&lwis_dev->interclient_lock);
	/* Walk through and execute the entries */
	ret = synchronous_process_io_entries(lwis_dev, k_msg.io.num_io_entries, k_entries,
					     k_msg.io.io_entries, k_msg.skip_error);
	mutex_unlock(&lwis_dev->interclient_lock);

reg_io_exit:
	if (k_entries)
		lwis_allocator_free(lwis_dev, k_entries);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_event_control_get(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
				 struct lwis_cmd_event_control_get __user *u_msg)
{
	int ret = 0;
	struct lwis_cmd_event_control_get control;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (copy_from_user((void *)&control, (void __user *)u_msg, sizeof(control))) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes from user\n", sizeof(control));
		return -EFAULT;
	}

	ret = lwis_client_event_control_get(lwis_client, control.ctl.event_id, &control.ctl);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to get event: %lld (err:%d)\n", control.ctl.event_id,
			ret);
		header->ret_code = ret;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	control.header.ret_code = 0;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)&control, sizeof(control));
}

static int cmd_event_control_set(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
				 struct lwis_cmd_event_control_set __user *u_msg)
{
	struct lwis_cmd_event_control_set k_msg;
	struct lwis_event_control *k_event_controls;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	int ret = 0;
	int i;
	size_t buf_size;

	if (copy_from_user((void *)&k_msg, (void __user *)u_msg, sizeof(k_msg))) {
		dev_err(lwis_dev->dev, "Failed to copy ioctl message from user\n");
		return -EFAULT;
	}

	/*  Copy event controls from user buffer. */
	buf_size = sizeof(struct lwis_event_control) * k_msg.list.num_event_controls;
	if (buf_size / sizeof(struct lwis_event_control) != k_msg.list.num_event_controls) {
		dev_err(lwis_dev->dev, "Failed to copy event controls due to integer overflow.\n");
		header->ret_code = -EOVERFLOW;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}
	k_event_controls = kmalloc(buf_size, GFP_KERNEL);
	if (!k_event_controls) {
		header->ret_code = -ENOMEM;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}
	if (copy_from_user(k_event_controls, (void __user *)k_msg.list.event_controls, buf_size)) {
		dev_err(lwis_dev->dev, "Failed to copy event controls from user\n");
		ret = -EFAULT;
		goto exit;
	}

	for (i = 0; i < k_msg.list.num_event_controls; i++) {
		ret = lwis_client_event_control_set(lwis_client, &k_event_controls[i]);
		if (ret) {
			dev_err(lwis_dev->dev, "Failed to apply event control 0x%llx\n",
				k_event_controls[i].event_id);
			goto exit;
		}
	}

	mutex_lock(&lwis_dev->interclient_lock);
	if (lwis_dev->irqs) {
		ret = lwis_interrupt_write_combined_mask_value(lwis_dev->irqs);
		if (ret) {
			dev_err(lwis_dev->dev, "Failed to write combined mask value: %d\n", ret);
			goto exit_locked;
		}
	}
exit_locked:
	mutex_unlock(&lwis_dev->interclient_lock);
exit:
	kfree(k_event_controls);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_event_dequeue(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
			     struct lwis_cmd_event_dequeue __user *u_msg)
{
	struct lwis_cmd_event_dequeue info;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	struct lwis_event_entry *event;
	int ret = 0;
	int err = 0;
	bool is_error_event = false;

	if (copy_from_user((void *)&info, (void __user *)u_msg, sizeof(info))) {
		dev_err(lwis_dev->dev, "Failed to copy %zu bytes from user\n", sizeof(info));
		return -EFAULT;
	}

	mutex_lock(&lwis_dev->interclient_lock);
	/* Peek at the front element of error event queue first */
	ret = lwis_client_error_event_peek_front(lwis_client, &event);
	if (ret == 0) {
		is_error_event = true;
	} else if (ret != -ENOENT) {
		dev_err(lwis_dev->dev, "Error dequeueing error event: %d\n", ret);
		mutex_unlock(&lwis_dev->interclient_lock);
		header->ret_code = ret;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	} else {
		/* Nothing at error event queue, continue to check normal event queue */
		ret = lwis_client_event_peek_front(lwis_client, &event);
		if (ret) {
			if (ret != -ENOENT)
				dev_err(lwis_dev->dev, "Error dequeueing event: %d\n", ret);
			mutex_unlock(&lwis_dev->interclient_lock);
			header->ret_code = ret;
			return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
		}
	}

	/* We need to check if we have an adequate payload buffer */
	if (event->event_info.payload_size > info.info.payload_buffer_size) {
		/* Nope, we don't. Let's inform the user and bail */
		info.info.payload_size = event->event_info.payload_size;
		err = -EAGAIN;
	} else {
		info.info.event_id = event->event_info.event_id;
		info.info.event_counter = event->event_info.event_counter;
		info.info.timestamp_ns = event->event_info.timestamp_ns;
		info.info.payload_size = event->event_info.payload_size;

		/* Here we have a payload and the buffer is big enough */
		if (event->event_info.payload_size > 0 && info.info.payload_buffer) {
			/* Copy over the payload buffer to userspace */
			if (copy_to_user((void __user *)info.info.payload_buffer,
					 (void *)event->event_info.payload_buffer,
					 event->event_info.payload_size)) {
				dev_err(lwis_dev->dev, "Failed to copy %zu bytes to user\n",
					event->event_info.payload_size);
				mutex_unlock(&lwis_dev->interclient_lock);
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
		if (is_error_event)
			ret = lwis_client_error_event_pop_front(lwis_client, NULL);
		else
			ret = lwis_client_event_pop_front(lwis_client, NULL);

		if (ret) {
			dev_err(lwis_dev->dev, "Error dequeueing event: %d\n", ret);
			mutex_unlock(&lwis_dev->interclient_lock);
			header->ret_code = ret;
			return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
		}
	}
	mutex_unlock(&lwis_dev->interclient_lock);
	/* Now let's copy the actual info struct back to user */
	info.header.ret_code = err;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)&info, sizeof(info));
}

static int cmd_fake_event_inject(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
				 struct lwis_cmd_pkt __user *u_msg)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	int rt_irq;

	mutex_lock(&lwis_dev->interclient_lock);
	struct lwis_interrupt_list *list = lwis_dev->irqs;

	if (lwis_dev->type != DEVICE_TYPE_TEST || list->count != TEST_DEVICE_IRQ_CNT) {
		mutex_unlock(&lwis_dev->interclient_lock);
		return -EINVAL;
	}

	/* Fake Event Injection */
	rt_irq = lwis_fake_event_inject(&list->irq[0]);
	mutex_unlock(&lwis_dev->interclient_lock);

	if (rt_irq != TEST_DEVICE_FAKE_INJECTION_IRQ) {
		dev_err(lwis_dev->dev, "Error fake injection: rt_irq = %d, expect rt_irq = %d\n",
			rt_irq, TEST_DEVICE_FAKE_INJECTION_IRQ);
		ret = -1;
	}

	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_transaction_submit(struct lwis_client *client, struct lwis_cmd_pkt *header,
				  struct lwis_cmd_pkt __user *u_msg,
				  struct cmd_transaction_submit_ops *ops)
{
	struct lwis_transaction *k_transaction = NULL;
	struct lwis_cmd_pkt *resp_header = NULL;
	struct lwis_device *lwis_dev = client->lwis_dev;
	void *cmd;
	int ret = 0;
	unsigned long flags;

	if (lwis_dev->type == DEVICE_TYPE_SLC || lwis_dev->type == DEVICE_TYPE_DPM) {
		dev_err(lwis_dev->dev, "not supported device type: %d\n", lwis_dev->type);
		ret = -EINVAL;
		goto err_exit;
	}

	cmd = kmalloc(ops->cmd_size, GFP_KERNEL);
	if (cmd == NULL) {
		ret = -ENOMEM;
		goto err_exit;
	}

	if (copy_from_user(cmd, (void __user *)u_msg, ops->cmd_size)) {
		dev_err(client->lwis_dev->dev, "Failed to copy transaction info from user\n");
		ret = -EFAULT;
		goto err_free_cmd;
	}

	k_transaction = kzalloc(sizeof(struct lwis_transaction), GFP_KERNEL);
	if (!k_transaction) {
		ret = -ENOMEM;
		goto err_free_cmd;
	}

	ops->populate_transaction_info_from_cmd(cmd, k_transaction);

	if (k_transaction->info.trigger_condition.num_nodes < 0) {
		dev_err(lwis_dev->dev, "Invalid trigger condition node count %lu\n",
			k_transaction->info.trigger_condition.num_nodes);
		ret = -EINVAL;
		goto err_free_cmd;
	}

	if (k_transaction->info.trigger_condition.num_nodes > LWIS_TRIGGER_NODES_MAX_NUM) {
		dev_err(lwis_dev->dev,
			"Trigger condition contains %lu node, more than the limit of %d\n",
			k_transaction->info.trigger_condition.num_nodes,
			LWIS_TRIGGER_NODES_MAX_NUM);
		ret = -EINVAL;
		goto err_free_cmd;
	}

	ret = construct_io_entry(client, k_transaction->info.io_entries,
				 k_transaction->info.num_io_entries,
				 &k_transaction->info.io_entries);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to prepare lwis io entries for transaction\n");
		goto err_free_cmd;
	}

	k_transaction->legacy_lwis_fence = (header->cmd_id == LWIS_CMD_ID_TRANSACTION_SUBMIT_V5 ||
					    header->cmd_id == LWIS_CMD_ID_TRANSACTION_SUBMIT_V4);
	k_transaction->resp = NULL;
	k_transaction->is_weak_transaction = false;
	k_transaction->remaining_entries_to_process = k_transaction->info.num_io_entries;
	k_transaction->starting_read_buf = NULL;
	INIT_LIST_HEAD(&k_transaction->event_list_node);
	INIT_LIST_HEAD(&k_transaction->process_queue_node);
	INIT_LIST_HEAD(&k_transaction->trigger_fences);
	INIT_LIST_HEAD(&k_transaction->completion_fence_list);

	ret = lwis_initialize_transaction_fences(client, k_transaction);
	if (ret) {
		lwis_transaction_free(lwis_dev, &k_transaction);
		goto err_free_cmd;
	}

	/*
	 * Create top device thread only when user space submits
	 * a transaction to be executed on the top device.
	 * This will ensure that a worker thread is not created
	 * for the top device by default.
	 */
	if (lwis_dev->type == DEVICE_TYPE_TOP)
		create_top_device_worker_thread(client);

	spin_lock_irqsave(&client->transaction_lock, flags);
	ret = lwis_transaction_submit_locked(client, k_transaction);
	ops->populate_cmd_info_from_transaction(cmd, k_transaction, ret);
	spin_unlock_irqrestore(&client->transaction_lock, flags);

	if (ret)
		lwis_transaction_free(lwis_dev, &k_transaction);

	resp_header = cmd;
	resp_header->cmd_id = header->cmd_id;
	resp_header->next = header->next;
	resp_header->ret_code = ret;
	ret = copy_pkt_to_user(lwis_dev, u_msg, cmd, ops->cmd_size);
	if (ret)
		goto err_free_cmd;

	kfree(cmd);
	return 0;

err_free_cmd:
	kfree(cmd);

err_exit:
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static void populate_transaction_info_from_cmd(void *_cmd, struct lwis_transaction *k_transaction)
{
	struct lwis_cmd_transaction_info *cmd = _cmd;

	k_transaction->info = cmd->info;
}

static void populate_cmd_info_from_transaction(void *_cmd, struct lwis_transaction *k_transaction,
					       int error)
{
	struct lwis_cmd_transaction_info *cmd = _cmd;

	cmd->info = k_transaction->info;
	if (error != 0)
		cmd->info.id = LWIS_ID_INVALID;
}

struct cmd_transaction_submit_ops current_version_cmd_transaction_ops = {
	.cmd_size = sizeof(struct lwis_cmd_transaction_info),
	.populate_transaction_info_from_cmd = populate_transaction_info_from_cmd,
	.populate_cmd_info_from_transaction = populate_cmd_info_from_transaction,
};

static int cmd_transaction_cancel(struct lwis_client *client, struct lwis_cmd_pkt *header,
				  struct lwis_cmd_transaction_cancel __user *u_msg)
{
	int ret = 0;
	struct lwis_cmd_transaction_cancel k_msg;
	struct lwis_device *lwis_dev = client->lwis_dev;

	if (copy_from_user((void *)&k_msg, (void __user *)u_msg, sizeof(k_msg))) {
		dev_err(lwis_dev->dev, "Failed to copy transaction ID from user\n");
		return -EFAULT;
	}

	ret = lwis_transaction_cancel(client, k_msg.id);
	if (ret) {
		dev_info_ratelimited(
			lwis_dev->dev,
			"Transaction id 0x%llx does not exist or is already done, not available for cancel(%d)\n",
			k_msg.id, ret);
	}

	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int construct_periodic_io_from_cmd(struct lwis_client *client,
					  struct lwis_cmd_periodic_io_info __user *u_msg,
					  struct lwis_periodic_io **periodic_io)
{
	int ret = 0;
	struct lwis_periodic_io *k_periodic_io;
	struct lwis_cmd_periodic_io_info k_info;
	struct lwis_device *lwis_dev = client->lwis_dev;

	k_periodic_io = kmalloc(sizeof(struct lwis_periodic_io), GFP_KERNEL);
	if (!k_periodic_io)
		return -ENOMEM;

	if (copy_from_user((void *)&k_info, (void __user *)u_msg, sizeof(k_info))) {
		dev_err(lwis_dev->dev, "Failed to copy periodic io info from user\n");
		ret = -EFAULT;
		goto error_free_periodic_io;
	}

	memcpy(&k_periodic_io->info, &k_info.info, sizeof(k_periodic_io->info));

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

static int cmd_periodic_io_submit(struct lwis_client *client, struct lwis_cmd_pkt *header,
				  struct lwis_cmd_periodic_io_info __user *u_msg)
{
	int ret = 0;
	struct lwis_cmd_periodic_io_info k_periodic_io_info;
	struct lwis_periodic_io *k_periodic_io = NULL;
	struct lwis_device *lwis_dev = client->lwis_dev;

	ret = construct_periodic_io_from_cmd(client, u_msg, &k_periodic_io);
	if (ret)
		goto err_exit;

	/*
	 * Create top device thread only when user space submits
	 * a periodic IO to be executed on the top device.
	 * This will ensure that a worker thread is not created
	 * for the top device by default.
	 */
	if (lwis_dev->type == DEVICE_TYPE_TOP)
		create_top_device_worker_thread(client);

	mutex_lock(&client->lock);
	ret = lwis_periodic_io_submit(client, k_periodic_io);

	k_periodic_io_info.info = k_periodic_io->info;
	if (ret) {
		k_periodic_io_info.info.id = LWIS_ID_INVALID;
		lwis_periodic_io_free(lwis_dev, k_periodic_io);
		goto err_exit;
	}

	mutex_unlock(&client->lock);
	k_periodic_io_info.header.cmd_id = header->cmd_id;
	k_periodic_io_info.header.next = header->next;
	k_periodic_io_info.header.ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)&k_periodic_io_info,
				sizeof(k_periodic_io_info));

err_exit:
	mutex_unlock(&client->lock);
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_periodic_io_cancel(struct lwis_client *client, struct lwis_cmd_pkt *header,
				  struct lwis_cmd_periodic_io_cancel __user *u_msg)
{
	int ret = 0;
	struct lwis_cmd_periodic_io_cancel k_msg;
	struct lwis_device *lwis_dev = client->lwis_dev;

	if (copy_from_user((void *)&k_msg, (void __user *)u_msg, sizeof(k_msg))) {
		dev_err(lwis_dev->dev, "Failed to copy periodic io ID from user\n");
		return -EFAULT;
	}

	mutex_lock(&client->lock);
	ret = lwis_periodic_io_cancel(client, k_msg.id);
	mutex_unlock(&client->lock);
	if (ret) {
		dev_err_ratelimited(lwis_dev->dev, "Failed to clear periodic io id 0x%llx\n",
				    k_msg.id);
	}

	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_dpm_clk_update(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
			      struct lwis_cmd_dpm_clk_update __user *u_msg)
{
	int ret;
	struct lwis_cmd_dpm_clk_update k_msg;
	struct lwis_clk_setting *clk_settings;
	size_t buf_size;

	if (copy_from_user((void *)&k_msg, (void __user *)u_msg, sizeof(k_msg))) {
		dev_err(lwis_dev->dev, "Failed to copy ioctl message from user\n");
		return -EFAULT;
	}

	buf_size = sizeof(struct lwis_clk_setting) * k_msg.settings.num_settings;
	if (buf_size / sizeof(struct lwis_clk_setting) != k_msg.settings.num_settings) {
		dev_err(lwis_dev->dev, "Failed to copy clk settings due to integer overflow.\n");
		ret = -EOVERFLOW;
		goto exit;
	}
	clk_settings = kmalloc(buf_size, GFP_KERNEL);
	if (!clk_settings) {
		ret = -ENOMEM;
		goto exit;
	}

	if (copy_from_user(clk_settings, (void __user *)k_msg.settings.settings, buf_size)) {
		dev_err(lwis_dev->dev, "Failed to copy clk settings from user\n");
		kfree(clk_settings);
		ret = -EFAULT;
		goto exit;
	}

	mutex_lock(&lwis_dev->interclient_lock);
	ret = lwis_dpm_update_clock(lwis_dev, clk_settings, k_msg.settings.num_settings);
	mutex_unlock(&lwis_dev->interclient_lock);
	kfree(clk_settings);
exit:
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_dpm_qos_update(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
			      struct lwis_cmd_dpm_qos_update __user *u_msg)
{
	struct lwis_cmd_dpm_qos_update k_msg;
	struct lwis_qos_setting *k_qos_settings;
	int ret = 0;
	int i;
	size_t buf_size;

	if (lwis_dev->type != DEVICE_TYPE_DPM) {
		dev_err(lwis_dev->dev, "not supported device type: %d\n", lwis_dev->type);
		ret = -EINVAL;
		goto exit;
	}

	if (copy_from_user((void *)&k_msg, (void __user *)u_msg, sizeof(k_msg))) {
		dev_err(lwis_dev->dev, "Failed to copy ioctl message from user\n");
		return -EFAULT;
	}

	// Copy qos settings from user buffer.
	buf_size = sizeof(struct lwis_qos_setting) * k_msg.reqs.num_settings;
	if (buf_size / sizeof(struct lwis_qos_setting) != k_msg.reqs.num_settings) {
		dev_err(lwis_dev->dev, "Failed to copy qos settings due to integer overflow.\n");
		ret = -EOVERFLOW;
		goto exit;
	}
	k_qos_settings = kmalloc(buf_size, GFP_KERNEL);
	if (!k_qos_settings) {
		ret = -ENOMEM;
		goto exit;
	}
	if (copy_from_user(k_qos_settings, (void __user *)k_msg.reqs.qos_settings, buf_size)) {
		dev_err(lwis_dev->dev, "Failed to copy clk settings from user\n");
		kfree(k_qos_settings);
		ret = -EFAULT;
		goto exit;
	}

	mutex_lock(&lwis_dev->interclient_lock);
	for (i = 0; i < k_msg.reqs.num_settings; i++) {
		struct lwis_qos_setting_v3 k_qos_setting_v3;

		memcpy(&k_qos_setting_v3, &k_qos_settings[i], sizeof(struct lwis_qos_setting));
		k_qos_setting_v3.bts_block_name[0] = '\0';
		k_qos_setting_v3.qos_family_name[0] = '\0';
		ret = lwis_dpm_update_qos(lwis_dev, &k_qos_setting_v3);
		if (ret) {
			dev_err(lwis_dev->dev, "Failed to apply qos setting, ret: %d\n", ret);
			kfree(k_qos_settings);
			goto exit_locked;
		}
	}
	kfree(k_qos_settings);
exit_locked:
	mutex_unlock(&lwis_dev->interclient_lock);
exit:
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_dpm_qos_update_v2(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
				 struct lwis_cmd_dpm_qos_update_v2 __user *u_msg)
{
	struct lwis_cmd_dpm_qos_update_v2 k_msg;
	struct lwis_qos_setting_v2 *k_qos_settings;
	int ret = 0;
	int i;
	size_t buf_size;

	if (lwis_dev->type != DEVICE_TYPE_DPM) {
		dev_err(lwis_dev->dev, "not supported device type: %d\n", lwis_dev->type);
		ret = -EINVAL;
		goto exit;
	}

	if (copy_from_user((void *)&k_msg, (void __user *)u_msg, sizeof(k_msg))) {
		dev_err(lwis_dev->dev, "Failed to copy ioctl message from user\n");
		return -EFAULT;
	}

	// Copy qos settings from user buffer.
	buf_size = sizeof(struct lwis_qos_setting_v2) * k_msg.reqs.num_settings;
	if (buf_size / sizeof(struct lwis_qos_setting_v2) != k_msg.reqs.num_settings) {
		dev_err(lwis_dev->dev, "Failed to copy qos settings due to integer overflow.\n");
		ret = -EOVERFLOW;
		goto exit;
	}
	k_qos_settings = kmalloc(buf_size, GFP_KERNEL);
	if (!k_qos_settings) {
		ret = -ENOMEM;
		goto exit;
	}
	if (copy_from_user(k_qos_settings, (void __user *)k_msg.reqs.qos_settings, buf_size)) {
		dev_err(lwis_dev->dev, "Failed to copy clk settings from user\n");
		kfree(k_qos_settings);
		ret = -EFAULT;
		goto exit;
	}

	for (i = 0; i < k_msg.reqs.num_settings; i++) {
		struct lwis_qos_setting_v3 k_qos_setting_v3;

		memcpy(&k_qos_setting_v3, &k_qos_settings[i], sizeof(struct lwis_qos_setting_v2));
		k_qos_setting_v3.qos_family_name[0] = '\0';
		ret = lwis_dpm_update_qos(lwis_dev, &k_qos_setting_v3);
		if (ret) {
			dev_err(lwis_dev->dev, "Failed to apply qos setting, ret: %d\n", ret);
			kfree(k_qos_settings);
			goto exit;
		}
	}
	kfree(k_qos_settings);
exit:
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_dpm_qos_update_v3(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
				 struct lwis_cmd_dpm_qos_update_v3 __user *u_msg)
{
	struct lwis_cmd_dpm_qos_update_v3 k_msg;
	struct lwis_qos_setting_v3 *k_qos_settings;
	int ret = 0;
	int i;
	size_t buf_size;

	if (lwis_dev->type != DEVICE_TYPE_DPM) {
		dev_err(lwis_dev->dev, "not supported device type: %d\n", lwis_dev->type);
		ret = -EINVAL;
		goto exit;
	}

	if (copy_from_user((void *)&k_msg, (void __user *)u_msg, sizeof(k_msg))) {
		dev_err(lwis_dev->dev, "Failed to copy ioctl message from user\n");
		return -EFAULT;
	}

	// Copy qos settings from user buffer.
	buf_size = sizeof(struct lwis_qos_setting_v3) * k_msg.reqs.num_settings;
	if (buf_size / sizeof(struct lwis_qos_setting_v3) != k_msg.reqs.num_settings) {
		dev_err(lwis_dev->dev, "Failed to copy qos settings due to integer overflow.\n");
		ret = -EOVERFLOW;
		goto exit;
	}
	k_qos_settings = kmalloc(buf_size, GFP_KERNEL);
	if (!k_qos_settings) {
		ret = -ENOMEM;
		goto exit;
	}
	if (copy_from_user(k_qos_settings, (void __user *)k_msg.reqs.qos_settings, buf_size)) {
		dev_err(lwis_dev->dev, "Failed to copy clk settings from user\n");
		kfree(k_qos_settings);
		ret = -EFAULT;
		goto exit;
	}

	for (i = 0; i < k_msg.reqs.num_settings; i++) {
		ret = lwis_dpm_update_qos(lwis_dev, &k_qos_settings[i]);
		if (ret) {
			dev_err(lwis_dev->dev, "Failed to apply qos setting, ret: %d\n", ret);
			kfree(k_qos_settings);
			goto exit;
		}
	}
	kfree(k_qos_settings);
exit:
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_dpm_get_clock(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
			     struct lwis_cmd_dpm_clk_get __user *u_msg)
{
	struct lwis_cmd_dpm_clk_get current_setting;
	struct lwis_device *target_device;
	int ret = 0;

	if (lwis_dev->type != DEVICE_TYPE_DPM) {
		dev_err(lwis_dev->dev, "not supported device type: %d\n", lwis_dev->type);
		ret = -EINVAL;
		goto err_exit;
	}

	if (copy_from_user((void *)&current_setting, (void __user *)u_msg,
			   sizeof(current_setting))) {
		dev_err(lwis_dev->dev, "failed to copy from user\n");
		return -EFAULT;
	}

	target_device = lwis_find_dev_by_id(current_setting.setting.device_id);
	if (!target_device) {
		dev_err(lwis_dev->dev, "could not find lwis device by id %d\n",
			current_setting.setting.device_id);
		ret = -ENODEV;
		goto err_exit;
	}

	if (target_device->enabled == 0 && target_device->type != DEVICE_TYPE_DPM) {
		dev_warn(target_device->dev, "%s disabled, can't get clk\n", target_device->name);
		ret = -EPERM;
		goto err_exit;
	}

	mutex_lock(&target_device->interclient_lock);
	current_setting.setting.frequency_hz = (int64_t)lwis_dpm_read_clock(target_device);
	mutex_unlock(&target_device->interclient_lock);
	current_setting.header.ret_code = 0;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)&current_setting, sizeof(current_setting));

err_exit:
	header->ret_code = ret;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
}

static int cmd_fence_create_v0(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
			       struct lwis_cmd_fence_create_v0 __user *u_msg)
{
	struct lwis_fence_fds fence_fds;
	struct lwis_cmd_fence_create_v0 fence_create;

	if (copy_from_user((void *)&fence_create, (void __user *)u_msg, sizeof(fence_create))) {
		dev_err(lwis_dev->dev, "failed to copy from user\n");
		return -EFAULT;
	}

	fence_fds = lwis_fence_legacy_create(lwis_dev);
	if (fence_fds.error != 0) {
		header->ret_code = fence_fds.error;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	fence_create.fd = fence_fds.fd;
	fence_create.header.ret_code = 0;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)&fence_create, sizeof(fence_create));
}

static int cmd_fence_create(struct lwis_device *lwis_dev, struct lwis_cmd_pkt *header,
			    struct lwis_cmd_fence_create __user *u_msg)
{
	struct lwis_fence_fds fence_fds;
	struct lwis_cmd_fence_create fence_create;

	if (copy_from_user((void *)&fence_create, (void __user *)u_msg, sizeof(fence_create))) {
		dev_err(lwis_dev->dev, "failed to copy from user\n");
		return -EFAULT;
	}

	fence_fds = lwis_fence_create(lwis_dev);
	if (fence_fds.error != 0) {
		header->ret_code = fence_fds.error;
		return copy_pkt_to_user(lwis_dev, u_msg, (void *)header, sizeof(*header));
	}

	fence_create.fd = fence_fds.fd;
	fence_create.signal_fd = fence_fds.signal_fd;
	fence_create.header.ret_code = 0;
	return copy_pkt_to_user(lwis_dev, u_msg, (void *)&fence_create, sizeof(fence_create));
}

static int handle_cmd_pkt(struct lwis_client *lwis_client, struct lwis_cmd_pkt *header,
			  struct lwis_cmd_pkt __user *user_msg)
{
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	int ret = 0;

	switch (header->cmd_id) {
	case LWIS_CMD_ID_ECHO:
		ret = cmd_echo(lwis_dev, header, (struct lwis_cmd_echo __user *)user_msg);
		break;
	case LWIS_CMD_ID_TIME_QUERY:
		ret = cmd_time_query(lwis_dev, header,
				     (struct lwis_cmd_time_query __user *)user_msg);
		break;
	case LWIS_CMD_ID_GET_DEVICE_INFO:
		ret = cmd_get_device_info(lwis_dev, header,
					  (struct lwis_cmd_device_info __user *)user_msg);
		break;
	case LWIS_CMD_ID_DEVICE_ENABLE:
		ret = cmd_device_enable(lwis_client, header,
					(struct lwis_cmd_pkt __user *)user_msg);
		break;
	case LWIS_CMD_ID_DEVICE_DISABLE:
		ret = cmd_device_disable(lwis_client, header,
					 (struct lwis_cmd_pkt __user *)user_msg);
		break;
	case LWIS_CMD_ID_DEVICE_RESET:
		ret = cmd_device_reset(lwis_client, header,
				       (struct lwis_cmd_io_entries __user *)user_msg);
		break;
	case LWIS_CMD_ID_DEVICE_SUSPEND:
		ret = cmd_device_suspend(lwis_client, header,
					 (struct lwis_cmd_pkt __user *)user_msg);
		break;
	case LWIS_CMD_ID_DEVICE_RESUME:
		ret = cmd_device_resume(lwis_client, header,
					(struct lwis_cmd_pkt __user *)user_msg);
		break;
	case LWIS_CMD_ID_DUMP_DEBUG_STATE:
		ret = cmd_dump_debug_state(lwis_client, header,
					   (struct lwis_cmd_pkt __user *)user_msg);
		break;
	case LWIS_CMD_ID_GET_DEVICE_ENABLE_STATE:
		ret = cmd_get_device_enable_state(
			lwis_client, header,
			(struct lwis_cmd_get_device_enable_state __user *)user_msg);
		break;
	case LWIS_CMD_ID_DMA_BUFFER_ENROLL:
		ret = cmd_dma_buffer_enroll(lwis_client, header,
					    (struct lwis_cmd_dma_buffer_enroll __user *)user_msg);
		break;
	case LWIS_CMD_ID_DMA_BUFFER_DISENROLL:
		ret = cmd_dma_buffer_disenroll(
			lwis_client, header,
			(struct lwis_cmd_dma_buffer_disenroll __user *)user_msg);
		break;
	case LWIS_CMD_ID_DMA_BUFFER_CPU_ACCESS:
		ret = cmd_dma_buffer_cpu_access(
			lwis_client, header,
			(struct lwis_cmd_dma_buffer_cpu_access __user *)user_msg);
		break;
	case LWIS_CMD_ID_DMA_BUFFER_ALLOC:
		ret = cmd_dma_buffer_alloc(lwis_client, header,
					   (struct lwis_cmd_dma_buffer_alloc __user *)user_msg);
		break;
	case LWIS_CMD_ID_DMA_BUFFER_FREE:
		ret = cmd_dma_buffer_free(lwis_client, header,
					  (struct lwis_cmd_dma_buffer_free __user *)user_msg);
		break;
	case LWIS_CMD_ID_REG_IO:
		ret = cmd_reg_io(lwis_dev, header, (struct lwis_cmd_io_entries __user *)user_msg);
		break;
	case LWIS_CMD_ID_REG_IO_V2:
		ret = cmd_reg_io_v2(lwis_dev, header,
				    (struct lwis_cmd_io_entries_v2 __user *)user_msg);
		break;
	case LWIS_CMD_ID_EVENT_CONTROL_GET:
		ret = cmd_event_control_get(lwis_client, header,
					    (struct lwis_cmd_event_control_get __user *)user_msg);
		break;
	case LWIS_CMD_ID_EVENT_CONTROL_SET:
		ret = cmd_event_control_set(lwis_client, header,
					    (struct lwis_cmd_event_control_set __user *)user_msg);
		break;
	case LWIS_CMD_ID_EVENT_DEQUEUE:
		ret = cmd_event_dequeue(lwis_client, header,
					(struct lwis_cmd_event_dequeue __user *)user_msg);
		break;
	case LWIS_CMD_ID_TRANSACTION_SUBMIT_V4:
		ret = cmd_transaction_submit(lwis_client, header,
					     (struct lwis_cmd_pkt __user *)user_msg,
					     &transaction_cmd_v4_ops);
		break;
	case LWIS_CMD_ID_TRANSACTION_SUBMIT_V5:
		ret = cmd_transaction_submit(lwis_client, header,
					     (struct lwis_cmd_pkt __user *)user_msg,
					     &transaction_cmd_v5_ops);
		break;
	case LWIS_CMD_ID_TRANSACTION_SUBMIT:
		ret = cmd_transaction_submit(lwis_client, header,
					     (struct lwis_cmd_pkt __user *)user_msg,
					     &current_version_cmd_transaction_ops);
		break;
	case LWIS_CMD_ID_TRANSACTION_CANCEL:
		ret = cmd_transaction_cancel(lwis_client, header,
					     (struct lwis_cmd_transaction_cancel __user *)user_msg);
		break;
	case LWIS_CMD_ID_PERIODIC_IO_SUBMIT:
		ret = cmd_periodic_io_submit(lwis_client, header,
					     (struct lwis_cmd_periodic_io_info __user *)user_msg);
		break;
	case LWIS_CMD_ID_PERIODIC_IO_CANCEL:
		ret = cmd_periodic_io_cancel(lwis_client, header,
					     (struct lwis_cmd_periodic_io_cancel __user *)user_msg);
		break;
	case LWIS_CMD_ID_DPM_CLK_UPDATE:
		ret = cmd_dpm_clk_update(lwis_dev, header,
					 (struct lwis_cmd_dpm_clk_update __user *)user_msg);
		break;
	case LWIS_CMD_ID_DPM_QOS_UPDATE:
		ret = cmd_dpm_qos_update(lwis_dev, header,
					 (struct lwis_cmd_dpm_qos_update __user *)user_msg);
		break;
	case LWIS_CMD_ID_DPM_QOS_UPDATE_V2:
		ret = cmd_dpm_qos_update_v2(lwis_dev, header,
					    (struct lwis_cmd_dpm_qos_update_v2 __user *)user_msg);
		break;
	case LWIS_CMD_ID_DPM_QOS_UPDATE_V3:
		ret = cmd_dpm_qos_update_v3(lwis_dev, header,
					    (struct lwis_cmd_dpm_qos_update_v3 __user *)user_msg);
		break;
	case LWIS_CMD_ID_DPM_GET_CLOCK:
		ret = cmd_dpm_get_clock(lwis_dev, header,
					(struct lwis_cmd_dpm_clk_get __user *)user_msg);
		break;
	case LWIS_CMD_ID_FENCE_CREATE_V0:
		ret = cmd_fence_create_v0(lwis_dev, header,
					  (struct lwis_cmd_fence_create_v0 __user *)user_msg);
		break;
	case LWIS_CMD_ID_FENCE_CREATE:
		ret = cmd_fence_create(lwis_dev, header,
				       (struct lwis_cmd_fence_create __user *)user_msg);
		break;
	case LWIS_CMD_ID_EVENT_INJECTION:
		ret = cmd_fake_event_inject(lwis_client, header,
					    (struct lwis_cmd_pkt __user *)user_msg);
		break;
	default:
		header->ret_code = -ENOSYS;
		ret = copy_pkt_to_user(lwis_dev, user_msg, (void *)header, sizeof(*header));
	}

	return ret;
}

static int ioctl_handle_cmd_pkt(struct lwis_client *lwis_client,
				struct lwis_cmd_pkt __user *user_msg)
{
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	struct lwis_cmd_pkt header;
	int ret = 0;
	bool device_disabled;
	int cmd_count = 0;

	while (user_msg && cmd_count < MAX_CMD_COUNT) {
		/* Copy cmd packet header from userspace */
		if (copy_from_user(&header, (void __user *)user_msg, sizeof(header))) {
			dev_err(lwis_dev->dev,
				"Failed to copy cmd packet header from userspace.\n");
			return -EFAULT;
		}

		mutex_lock(&lwis_dev->interclient_lock);
		device_disabled = (lwis_dev->enabled == 0);
		mutex_unlock(&lwis_dev->interclient_lock);
		if (lwis_dev->type != DEVICE_TYPE_TOP && device_disabled &&
		    (header.cmd_id == LWIS_CMD_ID_DMA_BUFFER_ALLOC ||
		     header.cmd_id == LWIS_CMD_ID_REG_IO ||
		     header.cmd_id == LWIS_CMD_ID_TRANSACTION_SUBMIT_V4 ||
		     header.cmd_id == LWIS_CMD_ID_TRANSACTION_SUBMIT_V5 ||
		     header.cmd_id == LWIS_CMD_ID_TRANSACTION_SUBMIT ||
		     header.cmd_id == LWIS_CMD_ID_PERIODIC_IO_SUBMIT ||
		     header.cmd_id == LWIS_CMD_ID_EVENT_CONTROL_SET ||
		     header.cmd_id == LWIS_CMD_ID_DEVICE_RESET)) {
			dev_err_ratelimited(lwis_dev->dev,
					    "Unsupported cmd_id(0x%x) on a disabled device.\n",
					    header.cmd_id);
			header.ret_code = -EBADFD;
			return copy_pkt_to_user(lwis_dev, user_msg, (void *)&header,
						sizeof(header));
		}

		ret = handle_cmd_pkt(lwis_client, &header, user_msg);
		if (ret)
			return ret;

		user_msg = header.next;
		++cmd_count;
	}

	if (cmd_count >= MAX_CMD_COUNT)
		return -EOVERFLOW;

	return ret;
}

int lwis_ioctl_handler(struct lwis_client *lwis_client, unsigned int type, unsigned long param)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	switch (type) {
	case LWIS_CMD_PACKET:
		ret = ioctl_handle_cmd_pkt(lwis_client, (struct lwis_cmd_pkt *)param);
		break;
	default:
		dev_err_ratelimited(lwis_dev->dev, "Unknown IOCTL operation\n");
		ret = -EINVAL;
	};

	if (ret && ret != -ENOENT && ret != -ETIMEDOUT && ret != -EAGAIN)
		ioctl_pr_err(lwis_dev, type, ret);

	return ret;
}
