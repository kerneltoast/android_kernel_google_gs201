// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Debug Utilities
 *
 * Copyright (c) 2020 Google, LLC
 */

#include <linux/err.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/list.h>
#include <linux/string.h>

#include "lwis_buffer.h"
#include "lwis_debug.h"
#include "lwis_device.h"
#include "lwis_event.h"
#include "lwis_transaction.h"
#include "lwis_util.h"

#define PRINT_BUFFER_SIZE 128
/* Printing the log buffer line by line as printk does not work well with large chunks of data */
static void print_to_log(struct lwis_device *lwis_dev, char *buffer)
{
	int size;
	char tmpbuf[PRINT_BUFFER_SIZE + 1];
	char *start = buffer;
	char *end = strchr(buffer, '\n');

	while (end != NULL) {
		size = end - start + 1;
		if (size > PRINT_BUFFER_SIZE)
			size = PRINT_BUFFER_SIZE;

		memcpy(tmpbuf, start, size);
		tmpbuf[size] = '\0';
		dev_info(lwis_dev->dev, "%s", tmpbuf);
		start = end + 1;
		end = strchr(start, '\n');
	}
}

static int list_transactions(struct lwis_client *client, char *buffer, size_t buffer_size)
{
	int i;
	int hist_idx;
	int count;
	unsigned long flags;
	struct lwis_transaction_event_list *transaction_list;
	struct lwis_transaction *transaction;
	struct list_head *it_tran;
	struct lwis_transaction_history *trans_hist;

	spin_lock_irqsave(&client->transaction_lock, flags);
	if (hash_empty(client->transaction_list)) {
		count = scnprintf(buffer, buffer_size, "No transactions pending\n");
		goto exit;
	}
	count = scnprintf(buffer, buffer_size, "Pending Transactions:\n");
	hash_for_each(client->transaction_list, i, transaction_list, node) {
		if (list_empty(&transaction_list->list)) {
			count += scnprintf(buffer + count, buffer_size - count,
					   "No pending transaction for event %#llx\n",
					   transaction_list->event_id);
			continue;
		}
		list_for_each(it_tran, &transaction_list->list) {
			transaction = list_entry(it_tran, struct lwis_transaction, event_list_node);
			count += scnprintf(
				buffer + count, buffer_size - count,
				"ID: %#llx Trigger Event: %#llx Count: %#llx Submitted: %lld\n"
				"  Emit Success: %#llx Error: %llx\n",
				transaction->info.id, transaction->info.trigger_event_id,
				transaction->info.trigger_event_counter,
				transaction->info.submission_timestamp_ns,
				transaction->info.emit_success_event_id,
				transaction->info.emit_error_event_id);
		}
	}

	count += scnprintf(buffer + count, buffer_size - count, "Last Transactions:\n");
	hist_idx = client->debug_info.cur_transaction_hist_idx;
	for (i = 0; i < TRANSACTION_DEBUG_HISTORY_SIZE; ++i) {
		trans_hist = &client->debug_info.transaction_hist[hist_idx];
		/* Skip uninitialized entries */
		if (trans_hist->process_timestamp != 0) {
			count +=
				scnprintf(buffer + count, buffer_size - count,
					  "[%2d] ID: %#llx Trigger Event: %#llx Count: %#llx\n"
					  "     Emit Success: %#llx Error: %llx\n",
					  i, trans_hist->info.id, trans_hist->info.trigger_event_id,
					  trans_hist->info.trigger_event_counter,
					  trans_hist->info.emit_success_event_id,
					  trans_hist->info.emit_error_event_id);
			/* Process timestamp not recorded */
			if (trans_hist->process_timestamp == -1) {
				count += scnprintf(buffer + count, buffer_size - count,
						   "     Num Entries: %zu\n",
						   trans_hist->info.num_io_entries);
			} else {
				count += scnprintf(
					buffer + count, buffer_size - count,
					"     Num Entries: %zu Processed @ %lld for %lldns\n",
					trans_hist->info.num_io_entries,
					trans_hist->process_timestamp,
					trans_hist->process_duration_ns);
			}
		}
		++hist_idx;
		if (hist_idx >= TRANSACTION_DEBUG_HISTORY_SIZE)
			hist_idx = 0;
	}
exit:
	spin_unlock_irqrestore(&client->transaction_lock, flags);
	return count;
}

static int list_allocated_buffers(struct lwis_client *client, char *buffer, size_t buffer_size)
{
	struct lwis_allocated_buffer *alloc_buffer;
	int i;
	int idx = 0;
	int count;

	if (hash_empty(client->allocated_buffers))
		return scnprintf(buffer, buffer_size, "Allocated buffers: None\n");

	count = scnprintf(buffer, buffer_size, "Allocated buffers:\n");
	hash_for_each(client->allocated_buffers, i, alloc_buffer, node) {
		count += scnprintf(buffer + count, buffer_size - count, "[%2d] FD: %d Size: %zu\n",
				   ++idx, alloc_buffer->fd, alloc_buffer->size);
	}
	return count;
}

static int list_enrolled_buffers(struct lwis_client *client, char *buffer, size_t buffer_size)
{
	struct lwis_buffer_enrollment_list *enrollment_list;
	struct list_head *it_enrollment;
	struct lwis_enrolled_buffer *enrolled_buffer;
	dma_addr_t end_dma_vaddr;
	int i;
	int idx = 0;
	int count;

	if (hash_empty(client->enrolled_buffers))
		return scnprintf(buffer, buffer_size, "Enrolled buffers: None\n");

	count = scnprintf(buffer, buffer_size, "Enrolled buffers:\n");
	hash_for_each(client->enrolled_buffers, i, enrollment_list, node) {
		list_for_each(it_enrollment, &enrollment_list->list) {
			enrolled_buffer =
				list_entry(it_enrollment, struct lwis_enrolled_buffer, list_node);
			if (IS_ERR_VALUE(enrolled_buffer->info.dma_vaddr)) {
				count += scnprintf(buffer + count, buffer_size - count,
						   "Enrolled buffers: dma_vaddr %pad is invalid\n",
						   &enrolled_buffer->info.dma_vaddr);
				continue;
			}
			end_dma_vaddr = enrolled_buffer->info.dma_vaddr +
					(enrolled_buffer->dma_buf->size - 1);
			count += scnprintf(buffer + count, buffer_size - count,
					   "[%2d] FD: %d Mode: %s%s Addr:[%pad ~ %pad] Size: %zu\n",
					   idx++, enrolled_buffer->info.fd,
					   enrolled_buffer->info.dma_read ? "r" : "",
					   enrolled_buffer->info.dma_write ? "w" : "",
					   &enrolled_buffer->info.dma_vaddr, &end_dma_vaddr,
					   enrolled_buffer->dma_buf->size);
		}
	}
	return count;
}

static int generate_device_info(struct lwis_device *lwis_dev, char *buffer, size_t buffer_size)
{
	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}
	scnprintf(buffer, buffer_size, "%s Device Name: %s ID: %d State: %s\n",
		  lwis_device_type_to_string(lwis_dev->type), lwis_dev->name, lwis_dev->id,
		  lwis_dev->enabled ? "Enabled" : "Disabled");
	return 0;
}

static int generate_event_states_info(struct lwis_device *lwis_dev, char *buffer,
				      size_t buffer_size, int lwis_event_dump_cnt)
{
	int i;
	int idx = 0;
	int count;
	unsigned long flags;
	struct lwis_device_event_state *state;
	bool enabled_event_present = false;
	int traverse_last_events_size;

	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}

	count = scnprintf(buffer, buffer_size, "=== LWIS EVENT STATES INFO: %s ===\n",
			  lwis_dev->name);
	if (lwis_event_dump_cnt >= 0 && lwis_event_dump_cnt <= EVENT_DEBUG_HISTORY_SIZE)
		traverse_last_events_size = lwis_event_dump_cnt;
	else
		traverse_last_events_size = EVENT_DEBUG_HISTORY_SIZE;

	spin_lock_irqsave(&lwis_dev->lock, flags);
	if (hash_empty(lwis_dev->event_states)) {
		count += scnprintf(buffer + count, buffer_size - count,
				   "  No events being monitored\n");
		goto exit;
	}
	count += scnprintf(buffer + count, buffer_size - count, "Event Counts:\n");
	hash_for_each(lwis_dev->event_states, i, state, node) {
		if (state->event_counter > 0) {
			count += scnprintf(buffer + count, buffer_size - count,
					   "[%2d] ID: %#llx Counter: %#llx\n", ++idx,
					   state->event_id, state->event_counter);
			enabled_event_present = true;
		}
	}
	if (!enabled_event_present)
		count += scnprintf(buffer + count, buffer_size - count, "  No enabled events\n");

	count += scnprintf(buffer + count, buffer_size - count, "Last Events:\n");
	idx = lwis_dev->debug_info.cur_event_hist_idx;
	for (i = 0; i < traverse_last_events_size; ++i) {
		--idx;
		if (idx < 0)
			idx = EVENT_DEBUG_HISTORY_SIZE - 1;

		state = &lwis_dev->debug_info.event_hist[idx].state;
		/* Skip uninitialized entries */
		if (state->event_id != 0) {
			count += scnprintf(buffer + count, buffer_size - count,
					   "[%2d] ID: %#llx Counter: %#llx Timestamp: %lld\n", i,
					   state->event_id, state->event_counter,
					   lwis_dev->debug_info.event_hist[idx].timestamp);
		}
	}

exit:
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
	return 0;
}

static int generate_transaction_info(struct lwis_device *lwis_dev, char *buffer, size_t buffer_size)
{
	struct lwis_client *client;
	int idx = 0;
	int count;

	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}

	if (list_empty(&lwis_dev->clients)) {
		scnprintf(buffer, buffer_size, "No clients opened\n");
		return 0;
	}

	count = scnprintf(buffer, buffer_size, "=== LWIS TRANSACTION INFO: %s ===\n",
			  lwis_dev->name);
	list_for_each_entry(client, &lwis_dev->clients, node) {
		count += scnprintf(buffer + count, buffer_size - count, "Client %d:\n", idx);
		count += list_transactions(client, buffer + count, buffer_size - count);
		++idx;
	}

	return 0;
}

static int generate_buffer_info(struct lwis_device *lwis_dev, char *buffer, size_t buffer_size)
{
	struct lwis_client *client;
	int idx = 0;
	int count;

	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}

	if (list_empty(&lwis_dev->clients)) {
		scnprintf(buffer, buffer_size, "No clients opened\n");
		return 0;
	}

	count = scnprintf(buffer, buffer_size, "=== LWIS BUFFER INFO: %s ===\n", lwis_dev->name);
	list_for_each_entry(client, &lwis_dev->clients, node) {
		count += scnprintf(buffer + count, buffer_size - count, "Client %d:\n", idx);
		count += list_allocated_buffers(client, buffer + count, buffer_size - count);
		count += list_enrolled_buffers(client, buffer + count, buffer_size - count);
		++idx;
	}

	return 0;
}

static int generate_register_io_history(struct lwis_device *lwis_dev, char *buffer,
					size_t buffer_size)
{
	struct lwis_register_io_info *reg_io;
	int count;
	int hist_idx;
	int i;

	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}

	count = scnprintf(buffer, buffer_size, "=== LWIS REGISTER IO INFO: %s ===\n",
			  lwis_dev->name);
	count += scnprintf(buffer + count, buffer_size - count, "Last register read/writes:\n");
	hist_idx = lwis_dev->debug_info.cur_io_entry_hist_idx;
	for (i = 0; i < IO_ENTRY_DEBUG_HISTORY_SIZE; ++i) {
		reg_io = &lwis_dev->debug_info.io_entry_hist[hist_idx];
		/* Skip uninitialized entries */
		if (reg_io->start_timestamp != 0) {
			if (reg_io->io_entry.type == LWIS_IO_ENTRY_READ) {
				count += scnprintf(
					buffer + count, buffer_size - count,
					"READ: bid %d, offset %llu, val %llu, access_size %lu, start_timestamp %llu\n",
					reg_io->io_entry.rw.bid, reg_io->io_entry.rw.offset,
					reg_io->io_entry.rw.val, reg_io->access_size,
					reg_io->start_timestamp);
			} else if (reg_io->io_entry.type == LWIS_IO_ENTRY_READ_BATCH) {
				count += scnprintf(
					buffer + count, buffer_size - count,
					"READ_BATCH: bid %d, offset %llu, size_in_bytes %lu, access_size %lu, start_timestamp %llu\n",
					reg_io->io_entry.rw_batch.bid,
					reg_io->io_entry.rw_batch.offset,
					reg_io->io_entry.rw_batch.size_in_bytes,
					reg_io->access_size, reg_io->start_timestamp);
			} else if (reg_io->io_entry.type == LWIS_IO_ENTRY_WRITE) {
				count += scnprintf(
					buffer + count, buffer_size - count,
					"WRITE: bid %d, offset %llu, val %llu, access_size %lu, start_timestamp %llu\n",
					reg_io->io_entry.rw.bid, reg_io->io_entry.rw.offset,
					reg_io->io_entry.rw.val, reg_io->access_size,
					reg_io->start_timestamp);
			} else if (reg_io->io_entry.type == LWIS_IO_ENTRY_WRITE_BATCH) {
				count += scnprintf(
					buffer + count, buffer_size - count,
					"WRITE_BATCH: bid %d, offset %llu, size_in_bytes %lu, access_size %lu, start_timestamp %llu\n",
					reg_io->io_entry.rw_batch.bid,
					reg_io->io_entry.rw_batch.offset,
					reg_io->io_entry.rw_batch.size_in_bytes,
					reg_io->access_size, reg_io->start_timestamp);
			} else if (reg_io->io_entry.type == LWIS_IO_ENTRY_MODIFY) {
				count += scnprintf(
					buffer + count, buffer_size - count,
					"MODIFY: bid %d, offset %llu, access_size %lu, start_timestamp %llu\n",
					reg_io->io_entry.mod.bid, reg_io->io_entry.mod.offset,
					reg_io->access_size, reg_io->start_timestamp);
			} else if (reg_io->io_entry.type == LWIS_IO_ENTRY_READ_V2) {
				count += scnprintf(
					buffer + count, buffer_size - count,
					"READ: bid %d, offset %llu, val %llu, speed_hz %u, access_size %lu, start_timestamp %llu\n",
					reg_io->io_entry.rw_v2.bid, reg_io->io_entry.rw_v2.offset,
					reg_io->io_entry.rw_v2.val, reg_io->io_entry.rw_v2.speed_hz,
					reg_io->access_size, reg_io->start_timestamp);
			} else if (reg_io->io_entry.type == LWIS_IO_ENTRY_READ_BATCH_V2) {
				count += scnprintf(
					buffer + count, buffer_size - count,
					"READ_BATCH: bid %d, offset %llu, size_in_bytes %lu, speed_hz %u, access_size %lu, start_timestamp %llu\n",
					reg_io->io_entry.rw_batch_v2.bid,
					reg_io->io_entry.rw_batch_v2.offset,
					reg_io->io_entry.rw_batch_v2.size_in_bytes,
					reg_io->io_entry.rw_batch_v2.speed_hz, reg_io->access_size,
					reg_io->start_timestamp);
			} else if (reg_io->io_entry.type == LWIS_IO_ENTRY_WRITE_V2) {
				count += scnprintf(
					buffer + count, buffer_size - count,
					"WRITE: bid %d, offset %llu, val %llu, speed_hz %u,  access_size %lu, start_timestamp %llu\n",
					reg_io->io_entry.rw_v2.bid, reg_io->io_entry.rw_v2.offset,
					reg_io->io_entry.rw_v2.val, reg_io->io_entry.rw_v2.speed_hz,
					reg_io->access_size, reg_io->start_timestamp);
			} else if (reg_io->io_entry.type == LWIS_IO_ENTRY_WRITE_BATCH_V2) {
				count += scnprintf(
					buffer + count, buffer_size - count,
					"WRITE_BATCH: bid %d, offset %llu, size_in_bytes %lu, speed_hz %u, access_size %lu, start_timestamp %llu\n",
					reg_io->io_entry.rw_batch_v2.bid,
					reg_io->io_entry.rw_batch_v2.offset,
					reg_io->io_entry.rw_batch_v2.size_in_bytes,
					reg_io->io_entry.rw_batch_v2.speed_hz, reg_io->access_size,
					reg_io->start_timestamp);
			}
		}
		++hist_idx;
		if (hist_idx >= IO_ENTRY_DEBUG_HISTORY_SIZE)
			hist_idx = 0;
	}

	return 0;
}

int lwis_debug_print_register_io_history(struct lwis_device *lwis_dev)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 10240;
	char *buffer = kmalloc(buffer_size, GFP_KERNEL);

	if (!buffer)
		return -ENOMEM;

	ret = generate_register_io_history(lwis_dev, buffer, buffer_size);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate register io history");
		goto exit;
	}
	print_to_log(lwis_dev, buffer);

exit:
	kfree(buffer);
	return 0;
}

int lwis_debug_print_device_info(struct lwis_device *lwis_dev)
{
	int ret = 0;
	/* Buffer to store information */
	char buffer[256] = {};
	const size_t buffer_size = sizeof(buffer);

	ret = generate_device_info(lwis_dev, buffer, buffer_size);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate device info");
		return ret;
	}
	print_to_log(lwis_dev, buffer);
	return 0;
}

int lwis_debug_print_event_states_info(struct lwis_device *lwis_dev, int lwis_event_dump_cnt)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 8192;
	char *buffer = kmalloc(buffer_size, GFP_KERNEL);

	if (!buffer)
		return -ENOMEM;

	ret = generate_event_states_info(lwis_dev, buffer, buffer_size, lwis_event_dump_cnt);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate event states info");
		goto exit;
	}
	print_to_log(lwis_dev, buffer);
exit:
	kfree(buffer);
	return ret;
}

int lwis_debug_print_transaction_info(struct lwis_device *lwis_dev)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 10240;
	char *buffer = kmalloc(buffer_size, GFP_KERNEL);

	if (!buffer)
		return -ENOMEM;

	ret = generate_transaction_info(lwis_dev, buffer, buffer_size);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate transaction info");
		goto exit;
	}
	print_to_log(lwis_dev, buffer);
exit:
	kfree(buffer);
	return ret;
}

int lwis_debug_print_buffer_info(struct lwis_device *lwis_dev)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 2048;
	char *buffer = kmalloc(buffer_size, GFP_KERNEL);

	if (!buffer)
		return -ENOMEM;

	ret = generate_buffer_info(lwis_dev, buffer, buffer_size);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate buffer info");
		goto exit;
	}
	print_to_log(lwis_dev, buffer);
exit:
	kfree(buffer);
	return ret;
}

void lwis_debug_crash_info_dump(struct lwis_device *lwis_dev)
{
	const int event_dump_count = 5;

	/* State dump is only meaningful for I2C and IOREG devices */
	if (lwis_dev->type != DEVICE_TYPE_I2C && lwis_dev->type != DEVICE_TYPE_IOREG)
		return;

	dev_info(lwis_dev->dev, "LWIS Device (%s) Crash Info Dump:\n", lwis_dev->name);

	/* Dump event states and last 5 received events */
	lwis_debug_print_event_states_info(lwis_dev, event_dump_count);
}

/* DebugFS specific functions */
#ifdef CONFIG_DEBUG_FS

static ssize_t dev_info_read(struct file *fp, char __user *user_buf, size_t count, loff_t *position)
{
	int ret = 0;
	/* Buffer to store information */
	char buffer[256] = {};
	const size_t buffer_size = sizeof(buffer);
	struct lwis_device *lwis_dev = fp->f_inode->i_private;

	ret = generate_device_info(lwis_dev, buffer, buffer_size);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate device info");
		return ret;
	}

	return simple_read_from_buffer(user_buf, count, position, buffer, strlen(buffer));
}

static ssize_t event_states_read(struct file *fp, char __user *user_buf, size_t count,
				 loff_t *position)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 8192;
	struct lwis_device *lwis_dev = fp->f_inode->i_private;
	char *buffer = kmalloc(buffer_size, GFP_KERNEL);

	if (!buffer)
		return -ENOMEM;

	ret = generate_event_states_info(lwis_dev, buffer, buffer_size, /*lwis_event_dump_cnt=*/-1);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate event states info");
		goto exit;
	}
	ret = simple_read_from_buffer(user_buf, count, position, buffer, strlen(buffer));
exit:
	kfree(buffer);
	return ret;
}

static ssize_t transaction_info_read(struct file *fp, char __user *user_buf, size_t count,
				     loff_t *position)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 10240;
	struct lwis_device *lwis_dev = fp->f_inode->i_private;
	char *buffer = kmalloc(buffer_size, GFP_KERNEL);

	if (!buffer)
		return -ENOMEM;

	ret = generate_transaction_info(lwis_dev, buffer, buffer_size);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate transaction info\n");
		goto exit;
	}

	ret = simple_read_from_buffer(user_buf, count, position, buffer, strlen(buffer));
exit:
	kfree(buffer);
	return ret;
}

static ssize_t buffer_info_read(struct file *fp, char __user *user_buf, size_t count,
				loff_t *position)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 2048;
	struct lwis_device *lwis_dev = fp->f_inode->i_private;
	char *buffer = kmalloc(buffer_size, GFP_KERNEL);

	if (!buffer)
		return -ENOMEM;

	ret = generate_buffer_info(lwis_dev, buffer, buffer_size);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate buffer info\n");
		goto exit;
	}

	ret = simple_read_from_buffer(user_buf, count, position, buffer, strlen(buffer));
exit:
	kfree(buffer);
	return ret;
}

static ssize_t register_io_history_read(struct file *fp, char __user *user_buf, size_t count,
					loff_t *position)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 10240;
	struct lwis_device *lwis_dev = fp->f_inode->i_private;
	char *buffer = kmalloc(buffer_size, GFP_KERNEL);

	if (!buffer)
		return -ENOMEM;

	ret = generate_register_io_history(lwis_dev, buffer, buffer_size);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate register io history");
		goto exit;
	}

	ret = simple_read_from_buffer(user_buf, count, position, buffer, strlen(buffer));
exit:
	kfree(buffer);
	return ret;
}

static const struct file_operations dev_info_fops = {
	.owner = THIS_MODULE,
	.read = dev_info_read,
};

static const struct file_operations event_states_fops = {
	.owner = THIS_MODULE,
	.read = event_states_read,
};

static const struct file_operations transaction_info_fops = {
	.owner = THIS_MODULE,
	.read = transaction_info_read,
};

static const struct file_operations buffer_info_fops = {
	.owner = THIS_MODULE,
	.read = buffer_info_read,
};

static const struct file_operations register_io_history_fops = {
	.owner = THIS_MODULE,
	.read = register_io_history_read,
};

int lwis_device_debugfs_setup(struct lwis_device *lwis_dev, struct dentry *dbg_root)
{
	struct dentry *dbg_dir;
	struct dentry *dbg_dev_info_file;
	struct dentry *dbg_event_file;
	struct dentry *dbg_transaction_file;
	struct dentry *dbg_buffer_file;
	struct dentry *dbg_reg_io_file;

	/* DebugFS not present, just return */
	if (dbg_root == NULL)
		return 0;

	dbg_dir = debugfs_create_dir(lwis_dev->name, dbg_root);
	if (IS_ERR_OR_NULL(dbg_dir)) {
		dev_err(lwis_dev->dev, "Failed to create DebugFS directory - %ld",
			PTR_ERR(dbg_dir));
		return PTR_ERR(dbg_dir);
	}

	dbg_dev_info_file =
		debugfs_create_file("dev_info", 0444, dbg_dir, lwis_dev, &dev_info_fops);
	if (IS_ERR_OR_NULL(dbg_dev_info_file)) {
		dev_warn(lwis_dev->dev, "Failed to create DebugFS dev_info - %ld",
			 PTR_ERR(dbg_dev_info_file));
		dbg_dev_info_file = NULL;
	}

	dbg_event_file =
		debugfs_create_file("event_state", 0444, dbg_dir, lwis_dev, &event_states_fops);
	if (IS_ERR_OR_NULL(dbg_event_file)) {
		dev_warn(lwis_dev->dev, "Failed to create DebugFS event_state - %ld",
			 PTR_ERR(dbg_event_file));
		dbg_event_file = NULL;
	}

	dbg_transaction_file = debugfs_create_file("transaction_info", 0444, dbg_dir, lwis_dev,
						   &transaction_info_fops);
	if (IS_ERR_OR_NULL(dbg_transaction_file)) {
		dev_warn(lwis_dev->dev, "Failed to create DebugFS transaction_info - %ld",
			 PTR_ERR(dbg_transaction_file));
		dbg_transaction_file = NULL;
	}

	dbg_buffer_file =
		debugfs_create_file("buffer_info", 0444, dbg_dir, lwis_dev, &buffer_info_fops);
	if (IS_ERR_OR_NULL(dbg_buffer_file)) {
		dev_warn(lwis_dev->dev, "Failed to create DebugFS buffer_info - %ld",
			 PTR_ERR(dbg_buffer_file));
		dbg_buffer_file = NULL;
	}

	dbg_reg_io_file = debugfs_create_file("io_history", 0444, dbg_dir, lwis_dev,
					      &register_io_history_fops);
	if (IS_ERR_OR_NULL(dbg_reg_io_file)) {
		dev_warn(lwis_dev->dev, "Failed to create DebugFS io_history - %ld",
			 PTR_ERR(dbg_reg_io_file));
		dbg_reg_io_file = NULL;
	}

	lwis_dev->dbg_dir = dbg_dir;
	lwis_dev->dbg_dev_info_file = dbg_dev_info_file;
	lwis_dev->dbg_event_file = dbg_event_file;
	lwis_dev->dbg_transaction_file = dbg_transaction_file;
	lwis_dev->dbg_buffer_file = dbg_buffer_file;
	lwis_dev->dbg_reg_io_file = dbg_reg_io_file;

	return 0;
}

int lwis_device_debugfs_cleanup(struct lwis_device *lwis_dev)
{
	/* DebugFS not present, just return */
	if (lwis_dev->dbg_dir == NULL)
		return 0;

	debugfs_remove_recursive(lwis_dev->dbg_dir);
	lwis_dev->dbg_dir = NULL;
	lwis_dev->dbg_dev_info_file = NULL;
	lwis_dev->dbg_event_file = NULL;
	lwis_dev->dbg_transaction_file = NULL;
	lwis_dev->dbg_buffer_file = NULL;
	lwis_dev->dbg_reg_io_file = NULL;
	return 0;
}

#else /* CONFIG_DEBUG_FS */

int lwis_device_debugfs_setup(struct lwis_device *lwis_dev, struct dentry *dbg_root)
{
	return 0;
}

int lwis_device_debugfs_cleanup(struct lwis_device *lwis_dev)
{
	return 0;
}

#endif
