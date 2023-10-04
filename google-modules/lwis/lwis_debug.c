/*
 * Google LWIS Debug Utilities
 *
 * Copyright (c) 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
static void print_to_log(char *buffer)
{
	int size;
	char tmpbuf[PRINT_BUFFER_SIZE + 1];
	char *start = buffer;
	char *end = strchr(buffer, '\n');
	while (end != NULL) {
		size = end - start + 1;
		if (size > PRINT_BUFFER_SIZE) {
			size = PRINT_BUFFER_SIZE;
		}
		memcpy(tmpbuf, start, size);
		tmpbuf[size] = '\0';
		pr_info("%s", tmpbuf);
		start = end + 1;
		end = strchr(start, '\n');
	}
}

static void list_transactions(struct lwis_client *client, char *k_buf, size_t k_buf_size)
{
	/* Temporary buffer to be concatenated to the main buffer. */
	char tmp_buf[128] = {};
	int i, hist_idx;
	unsigned long flags;
	struct lwis_transaction_event_list *transaction_list;
	struct lwis_transaction *transaction;
	struct list_head *it_tran;
	struct lwis_transaction_history *trans_hist;

	spin_lock_irqsave(&client->transaction_lock, flags);
	if (hash_empty(client->transaction_list)) {
		strlcat(k_buf, "No transactions pending\n", k_buf_size);
		goto exit;
	}
	strlcat(k_buf, "Pending Transactions:\n", k_buf_size);
	hash_for_each (client->transaction_list, i, transaction_list, node) {
		if (list_empty(&transaction_list->list)) {
			scnprintf(tmp_buf, sizeof(tmp_buf),
				  "No pending transaction for event 0x%llx\n",
				  transaction_list->event_id);
			strlcat(k_buf, tmp_buf, k_buf_size);
			continue;
		}
		list_for_each (it_tran, &transaction_list->list) {
			transaction = list_entry(it_tran, struct lwis_transaction, event_list_node);
			scnprintf(
				tmp_buf, sizeof(tmp_buf),
				"ID: 0x%llx Trigger Event: 0x%llx Count: 0x%llx Submitted: %lld\n",
				transaction->info.id, transaction->info.trigger_event_id,
				transaction->info.trigger_event_counter,
				transaction->info.submission_timestamp_ns);
			strlcat(k_buf, tmp_buf, k_buf_size);
			scnprintf(tmp_buf, sizeof(tmp_buf), "  Emit Success: 0x%llx Error: %llx\n",
				  transaction->info.emit_success_event_id,
				  transaction->info.emit_error_event_id);
			strlcat(k_buf, tmp_buf, k_buf_size);
		}
	}

	strlcat(k_buf, "Last Transactions:\n", k_buf_size);
	hist_idx = client->debug_info.cur_transaction_hist_idx;
	for (i = 0; i < TRANSACTION_DEBUG_HISTORY_SIZE; ++i) {
		trans_hist = &client->debug_info.transaction_hist[hist_idx];
		/* Skip uninitialized entries */
		if (trans_hist->process_timestamp != 0) {
			scnprintf(tmp_buf, sizeof(tmp_buf),
				  "[%2d] ID: 0x%llx Trigger Event: 0x%llx Count: 0x%llx\n", i,
				  trans_hist->info.id, trans_hist->info.trigger_event_id,
				  trans_hist->info.trigger_event_counter);
			strlcat(k_buf, tmp_buf, k_buf_size);
			scnprintf(tmp_buf, sizeof(tmp_buf),
				  "     Emit Success: 0x%llx Error: %llx\n",
				  trans_hist->info.emit_success_event_id,
				  trans_hist->info.emit_error_event_id);
			strlcat(k_buf, tmp_buf, k_buf_size);
			scnprintf(tmp_buf, sizeof(tmp_buf),
				  "     Num Entries: %zu Processed @ %lld for %lldns\n",
				  trans_hist->info.num_io_entries, trans_hist->process_timestamp,
				  trans_hist->process_duration_ns);
			strlcat(k_buf, tmp_buf, k_buf_size);
		}
		hist_idx++;
		if (hist_idx >= TRANSACTION_DEBUG_HISTORY_SIZE) {
			hist_idx = 0;
		}
	}
exit:
	spin_unlock_irqrestore(&client->transaction_lock, flags);
}

static void list_allocated_buffers(struct lwis_client *client, char *k_buf, size_t k_buf_size)
{
	char tmp_buf[64] = {};
	struct lwis_allocated_buffer *buffer;
	int i;
	int idx = 0;

	if (hash_empty(client->allocated_buffers)) {
		strlcat(k_buf, "Allocated buffers: None\n", k_buf_size);
		return;
	}

	strlcat(k_buf, "Allocated buffers:\n", k_buf_size);
	hash_for_each (client->allocated_buffers, i, buffer, node) {
		scnprintf(tmp_buf, sizeof(tmp_buf), "[%2d] FD: %d Size: %zu\n", idx++, buffer->fd,
			  buffer->size);
		strlcat(k_buf, tmp_buf, k_buf_size);
	}
}

static void list_enrolled_buffers(struct lwis_client *client, char *k_buf, size_t k_buf_size)
{
	char tmp_buf[128] = {};
	struct lwis_buffer_enrollment_list *enrollment_list;
	struct list_head *it_enrollment;
	struct lwis_enrolled_buffer *buffer;
	dma_addr_t end_dma_vaddr;
	int i;
	int idx = 0;

	if (hash_empty(client->enrolled_buffers)) {
		strlcat(k_buf, "Enrolled buffers: None\n", k_buf_size);
		return;
	}

	strlcat(k_buf, "Enrolled buffers:\n", k_buf_size);
	hash_for_each (client->enrolled_buffers, i, enrollment_list, node) {
		list_for_each (it_enrollment, &enrollment_list->list) {
			buffer = list_entry(it_enrollment, struct lwis_enrolled_buffer, list_node);
			end_dma_vaddr = buffer->info.dma_vaddr + (buffer->dma_buf->size - 1);
			scnprintf(tmp_buf, sizeof(tmp_buf),
				  "[%2d] FD: %d Mode: %s%s Addr:[%pad ~ %pad] Size: %zu\n", idx++,
				  buffer->info.fd, buffer->info.dma_read ? "r" : "",
				  buffer->info.dma_write ? "w" : "", &buffer->info.dma_vaddr,
				  &end_dma_vaddr, buffer->dma_buf->size);
			strlcat(k_buf, tmp_buf, k_buf_size);
		}
	}
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
				      size_t buffer_size)
{
	/* Temporary buffer to be concatenated to the main buffer. */
	char tmp_buf[96] = {};
	int i;
	int idx = 0;
	unsigned long flags;
	struct lwis_device_event_state *state;
	bool enabled_event_present = false;

	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}

	scnprintf(buffer, buffer_size, "=== LWIS EVENT STATES INFO: %s ===\n", lwis_dev->name);

	spin_lock_irqsave(&lwis_dev->lock, flags);
	if (hash_empty(lwis_dev->event_states)) {
		strlcat(buffer, "  No events being monitored\n", buffer_size);
		goto exit;
	}
	strlcat(buffer, "Enabled Device Events:\n", buffer_size);
	hash_for_each (lwis_dev->event_states, i, state, node) {
		if (state->enable_counter > 0) {
			scnprintf(tmp_buf, sizeof(tmp_buf), "[%2d] ID: 0x%llx Counter: 0x%llx\n",
				  idx++, state->event_id, state->event_counter);
			strlcat(buffer, tmp_buf, buffer_size);
			enabled_event_present = true;
		}
	}
	if (!enabled_event_present) {
		strlcat(buffer, "No enabled events\n", buffer_size);
	}
	strlcat(buffer, "Last Events:\n", buffer_size);
	idx = lwis_dev->debug_info.cur_event_hist_idx;
	for (i = 0; i < EVENT_DEBUG_HISTORY_SIZE; ++i) {
		state = &lwis_dev->debug_info.event_hist[idx].state;
		/* Skip uninitialized entries */
		if (state->event_id != 0) {
			scnprintf(tmp_buf, sizeof(tmp_buf),
				  "[%2d] ID: 0x%llx Counter: 0x%llx Timestamp: %lld\n", i,
				  state->event_id, state->event_counter,
				  lwis_dev->debug_info.event_hist[idx].timestamp);
			strlcat(buffer, tmp_buf, buffer_size);
		}
		idx++;
		if (idx >= EVENT_DEBUG_HISTORY_SIZE) {
			idx = 0;
		}
	}

exit:
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
	return 0;
}

static int generate_transaction_info(struct lwis_device *lwis_dev, char *buffer, size_t buffer_size)
{
	/* Temporary buffer to be concatenated to the main buffer. */
	char tmp_buf[64] = {};
	struct lwis_client *client;
	int idx = 0;

	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}

	scnprintf(buffer, buffer_size, "=== LWIS TRANSACTION INFO: %s ===\n", lwis_dev->name);

	if (list_empty(&lwis_dev->clients)) {
		scnprintf(buffer, buffer_size, "No clients opened\n");
		return 0;
	}
	list_for_each_entry (client, &lwis_dev->clients, node) {
		scnprintf(tmp_buf, sizeof(tmp_buf), "Client %d:\n", idx);
		strlcat(buffer, tmp_buf, buffer_size);
		list_transactions(client, buffer, buffer_size);
		idx++;
	}

	return 0;
}

static int generate_buffer_info(struct lwis_device *lwis_dev, char *buffer, size_t buffer_size)
{
	/* Temporary buffer to be concatenated to the main buffer. */
	char tmp_buf[64] = {};
	struct lwis_client *client;
	int idx = 0;
	unsigned long flags;

	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}

	scnprintf(buffer, buffer_size, "=== LWIS BUFFER INFO: %s ===\n", lwis_dev->name);

	if (list_empty(&lwis_dev->clients)) {
		scnprintf(buffer, buffer_size, "No clients opened\n");
		return 0;
	}

	spin_lock_irqsave(&lwis_dev->lock, flags);
	list_for_each_entry (client, &lwis_dev->clients, node) {
		scnprintf(tmp_buf, sizeof(tmp_buf), "Client %d:\n", idx);
		strlcat(buffer, tmp_buf, buffer_size);
		list_allocated_buffers(client, buffer, buffer_size);
		list_enrolled_buffers(client, buffer, buffer_size);
		idx++;
	}
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

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
	print_to_log(buffer);
	return 0;
}

int lwis_debug_print_event_states_info(struct lwis_device *lwis_dev)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 8192;
	char *buffer = kzalloc(buffer_size, GFP_KERNEL);
	if (!buffer) {
		dev_err(lwis_dev->dev, "Failed to allocate event states log buffer\n");
		return -ENOMEM;
	}

	ret = generate_event_states_info(lwis_dev, buffer, buffer_size);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate event states info");
		goto exit;
	}
	print_to_log(buffer);
exit:
	kfree(buffer);
	return ret;
}

int lwis_debug_print_transaction_info(struct lwis_device *lwis_dev)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 10240;
	char *buffer = kzalloc(buffer_size, GFP_KERNEL);
	if (!buffer) {
		dev_err(lwis_dev->dev, "Failed to allocate transaction info log buffer\n");
		return -ENOMEM;
	}

	ret = generate_transaction_info(lwis_dev, buffer, buffer_size);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate transaction info");
		goto exit;
	}
	print_to_log(buffer);
exit:
	kfree(buffer);
	return ret;
}

int lwis_debug_print_buffer_info(struct lwis_device *lwis_dev)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 2048;
	char *buffer = kzalloc(buffer_size, GFP_KERNEL);
	if (!buffer) {
		dev_err(lwis_dev->dev, "Failed to allocate buffer info log buffer\n");
		return -ENOMEM;
	}

	ret = generate_buffer_info(lwis_dev, buffer, buffer_size);
	if (ret) {
		dev_err(lwis_dev->dev, "Failed to generate buffer info");
		goto exit;
	}
	print_to_log(buffer);
exit:
	kfree(buffer);
	return ret;
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
	char *buffer = kzalloc(buffer_size, GFP_KERNEL);
	if (!buffer) {
		dev_err(lwis_dev->dev, "Failed to allocate event states log buffer\n");
		return -ENOMEM;
	}

	ret = generate_event_states_info(lwis_dev, buffer, buffer_size);
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
	char *buffer = kzalloc(buffer_size, GFP_KERNEL);
	if (!buffer) {
		dev_err(lwis_dev->dev, "Failed to allocate transaction info log buffer\n");
		return -ENOMEM;
	}

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
	char *buffer = kzalloc(buffer_size, GFP_KERNEL);
	if (!buffer) {
		dev_err(lwis_dev->dev, "Failed to allocate buffer info log buffer\n");
		return -ENOMEM;
	}

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

static struct file_operations dev_info_fops = {
	.owner = THIS_MODULE,
	.read = dev_info_read,
};

static struct file_operations event_states_fops = {
	.owner = THIS_MODULE,
	.read = event_states_read,
};

static struct file_operations transaction_info_fops = {
	.owner = THIS_MODULE,
	.read = transaction_info_read,
};

static struct file_operations buffer_info_fops = {
	.owner = THIS_MODULE,
	.read = buffer_info_read,
};

int lwis_device_debugfs_setup(struct lwis_device *lwis_dev, struct dentry *dbg_root)
{
	struct dentry *dbg_dir;
	struct dentry *dbg_dev_info_file;
	struct dentry *dbg_event_file;
	struct dentry *dbg_transaction_file;
	struct dentry *dbg_buffer_file;

	/* DebugFS not present, just return */
	if (dbg_root == NULL) {
		return 0;
	}

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

	lwis_dev->dbg_dir = dbg_dir;
	lwis_dev->dbg_dev_info_file = dbg_dev_info_file;
	lwis_dev->dbg_event_file = dbg_event_file;
	lwis_dev->dbg_transaction_file = dbg_transaction_file;
	lwis_dev->dbg_buffer_file = dbg_buffer_file;

	return 0;
}

int lwis_device_debugfs_cleanup(struct lwis_device *lwis_dev)
{
	/* DebugFS not present, just return */
	if (lwis_dev->dbg_dir == NULL) {
		return 0;
	}
	debugfs_remove_recursive(lwis_dev->dbg_dir);
	lwis_dev->dbg_dir = NULL;
	lwis_dev->dbg_dev_info_file = NULL;
	lwis_dev->dbg_event_file = NULL;
	lwis_dev->dbg_transaction_file = NULL;
	lwis_dev->dbg_buffer_file = NULL;
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
