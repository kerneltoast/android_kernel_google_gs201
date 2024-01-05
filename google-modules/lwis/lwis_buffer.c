// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS DMA Buffer Utilities
 *
 * Copyright (c) 2018 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-buffer: " fmt

#include <linux/dma-buf.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <soc/google/pt.h>

#include "lwis_buffer.h"
#include "lwis_commands.h"
#include "lwis_device.h"
#include "lwis_device_slc.h"
#include "lwis_platform_dma.h"
#include "lwis_trace.h"

static struct lwis_buffer_enrollment_list *enrollment_list_find(struct lwis_client *client,
								dma_addr_t dma_vaddr)
{
	struct lwis_buffer_enrollment_list *list;

	hash_for_each_possible(client->enrolled_buffers, list, node, dma_vaddr) {
		if (list->vaddr == dma_vaddr)
			return list;
	}
	return NULL;
}

static struct lwis_buffer_enrollment_list *enrollment_list_create(struct lwis_client *client,
								  dma_addr_t dma_vaddr)
{
	struct lwis_buffer_enrollment_list *enrollment_list =
		kmalloc(sizeof(struct lwis_buffer_enrollment_list), GFP_KERNEL);
	if (!enrollment_list)
		return NULL;

	enrollment_list->vaddr = dma_vaddr;
	INIT_LIST_HEAD(&enrollment_list->list);
	hash_add(client->enrolled_buffers, &enrollment_list->node, dma_vaddr);
	return enrollment_list;
}

static struct lwis_buffer_enrollment_list *
enrollment_list_find_or_create(struct lwis_client *client, dma_addr_t dma_vaddr)
{
	struct lwis_buffer_enrollment_list *list = enrollment_list_find(client, dma_vaddr);

	return (list == NULL) ? enrollment_list_create(client, dma_vaddr) : list;
}

static void dump_total_enrolled_buffer_size(struct lwis_device *lwis_dev)
{
	struct lwis_client *client;
	unsigned long flags;
	int i;
	struct lwis_buffer_enrollment_list *enrollment_list;
	struct lwis_enrolled_buffer *buffer;
	size_t total_enrolled_size = 0;
	int num_enrolled_buffers = 0;

	spin_lock_irqsave(&lwis_dev->lock, flags);
	list_for_each_entry(client, &lwis_dev->clients, node) {
		if (hash_empty(client->enrolled_buffers))
			continue;

		hash_for_each(client->enrolled_buffers, i, enrollment_list, node) {
			buffer = list_first_entry(&enrollment_list->list,
						  struct lwis_enrolled_buffer, list_node);
			total_enrolled_size += buffer->dma_buf->size;
			num_enrolled_buffers++;
		}
	}
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	if (total_enrolled_size > 0) {
		pr_info("%-16s: %16d %16lu kB\n", lwis_dev->name, num_enrolled_buffers,
			total_enrolled_size / 1024);
	}
}

int lwis_buffer_alloc(struct lwis_client *lwis_client, struct lwis_alloc_buffer_info *alloc_info,
		      struct lwis_allocated_buffer *buffer)
{
	struct dma_buf *dma_buf;
	int ret = 0;

	if (!lwis_client) {
		pr_err("Alloc: LWIS client is NULL\n");
		return -ENODEV;
	}
	if (!alloc_info || !buffer) {
		pr_err("Alloc: alloc_info and/or buffer is NULL\n");
		return -EINVAL;
	}

	if (alloc_info->flags & LWIS_DMA_SYSTEM_CACHE_RESERVATION) {
		if (lwis_client->lwis_dev->type == DEVICE_TYPE_SLC) {
			ret = lwis_slc_buffer_alloc(lwis_client->lwis_dev, alloc_info);
			if (ret)
				return ret;

			dma_buf = NULL;
		} else {
			pr_err("Can't allocate system cache buffer on non-slc device\n");
			return -EINVAL;
		}
	} else {
		alloc_info->size = PAGE_ALIGN(alloc_info->size);
		dma_buf = lwis_platform_dma_buffer_alloc(alloc_info->size, alloc_info->flags);
		if (IS_ERR_OR_NULL(dma_buf)) {
			pr_err("lwis_platform_dma_buffer_alloc failed (%ld)\n", PTR_ERR(dma_buf));
			return -ENOMEM;
		}

		/*
		 * Increment refcount of the fd to 1 first before dma_buf_fd()
		 * which is increment refcount of the fd to 2.
		 * Both userspace's close(fd) and kernel's lwis_buffer_free()
		 * will decrement the refcount by 1. Whoever reaches 0 refcount
		 * frees the buffer.
		 */
		get_dma_buf(dma_buf);

		alloc_info->dma_fd = dma_buf_fd(dma_buf, O_CLOEXEC);
		if (alloc_info->dma_fd < 0) {
			pr_err("dma_buf_fd failed (%d)\n", alloc_info->dma_fd);
			dma_buf_put(dma_buf);
			return alloc_info->dma_fd;
		}

		alloc_info->partition_id = PT_PTID_INVALID;
	}

	buffer->fd = alloc_info->dma_fd;
	buffer->size = alloc_info->size;
	buffer->dma_buf = dma_buf;
	hash_add(lwis_client->allocated_buffers, &buffer->node, buffer->fd);

	return 0;
}

int lwis_buffer_free(struct lwis_client *lwis_client, struct lwis_allocated_buffer *buffer)
{
	if (!lwis_client) {
		pr_err("Free: LWIS client is NULL\n");
		return -ENODEV;
	}
	if (!buffer) {
		pr_err("Free: buffer is NULL\n");
		return -EINVAL;
	}

	if (buffer->dma_buf == NULL) {
		if (lwis_client->lwis_dev->type == DEVICE_TYPE_SLC) {
			lwis_slc_buffer_free(lwis_client->lwis_dev, buffer->fd);
		} else {
			pr_err("Unexpected NULL dma_buf\n");
			return -EINVAL;
		}
	} else {
		dma_buf_put(buffer->dma_buf);
	}
	hash_del(&buffer->node);
	return 0;
}

int lwis_buffer_enroll(struct lwis_client *lwis_client, struct lwis_enrolled_buffer *buffer)
{
	struct lwis_buffer_enrollment_list *enrollment_list;
	struct list_head *it_enrollment;
	struct lwis_enrolled_buffer *old_buffer;
	char buffer_name[LWIS_MAX_NAME_STRING_LEN];
	char trace_name[LWIS_MAX_NAME_STRING_LEN];

	if (!lwis_client) {
		pr_err("Enroll: LWIS client is NULL\n");
		return -ENODEV;
	}
	if (!buffer) {
		pr_err("Enroll: buffer is NULL\n");
		return -EINVAL;
	}

	if (buffer->info.dma_read && buffer->info.dma_write)
		buffer->dma_direction = DMA_BIDIRECTIONAL;
	else if (buffer->info.dma_read)
		buffer->dma_direction = DMA_TO_DEVICE;
	else if (buffer->info.dma_write)
		buffer->dma_direction = DMA_FROM_DEVICE;
	else
		buffer->dma_direction = DMA_NONE;

	if (!valid_dma_direction(buffer->dma_direction)) {
		dev_err(lwis_client->lwis_dev->dev, "Enroll: buffer->dma_direction is invalid\n");
		return -EINVAL;
	}

	buffer->dma_buf = dma_buf_get(buffer->info.fd);
	if (IS_ERR_OR_NULL(buffer->dma_buf)) {
		dev_err(lwis_client->lwis_dev->dev,
			"Could not get dma buffer for fd: %d (errno: %ld)", buffer->info.fd,
			PTR_ERR(buffer->dma_buf));
		return PTR_ERR(buffer->dma_buf);
	}
	scnprintf(buffer_name, sizeof(buffer_name), "lwis:%s", lwis_client->lwis_dev->name);
	dma_buf_set_name(buffer->dma_buf, buffer_name);

	buffer->dma_buf_attachment = dma_buf_attach(buffer->dma_buf, lwis_client->lwis_dev->k_dev);
	if (IS_ERR_OR_NULL(buffer->dma_buf_attachment)) {
		dev_err(lwis_client->lwis_dev->dev,
			"Could not attach dma buffer for fd: %d (errno: %ld)", buffer->info.fd,
			PTR_ERR(buffer->dma_buf_attachment));
		dma_buf_put(buffer->dma_buf);
		return PTR_ERR(buffer->dma_buf_attachment);
	}

	scnprintf(trace_name, LWIS_MAX_NAME_STRING_LEN, "lwis:buf_enroll_%s",
		  lwis_client->lwis_dev->name);
	LWIS_ATRACE_FUNC_BEGIN(lwis_client->lwis_dev, trace_name);
	buffer->sg_table =
		dma_buf_map_attachment(buffer->dma_buf_attachment, buffer->dma_direction);
	LWIS_ATRACE_FUNC_END(lwis_client->lwis_dev, trace_name);
	if (IS_ERR_OR_NULL(buffer->sg_table)) {
		dev_err(lwis_client->lwis_dev->dev,
			"Could not map dma attachment for fd: %d (errno: %ld)", buffer->info.fd,
			PTR_ERR(buffer->sg_table));
		if (PTR_ERR(buffer->sg_table) == -ENOMEM) {
			lwis_device_info_dump("Enroll buffer sizes",
					      dump_total_enrolled_buffer_size);
		}
		dma_buf_detach(buffer->dma_buf, buffer->dma_buf_attachment);
		dma_buf_put(buffer->dma_buf);
		return PTR_ERR(buffer->sg_table);
	}

	buffer->info.dma_vaddr = sg_dma_address(buffer->sg_table->sgl);
	if (IS_ERR_OR_NULL((void *)buffer->info.dma_vaddr)) {
		dev_err(lwis_client->lwis_dev->dev, "Could not map dma vaddr for fd: %d",
			buffer->info.fd);
		goto err;
	}

	// Insert the new enrollment to the enrolled_buffers hashtable.
	enrollment_list = enrollment_list_find_or_create(lwis_client, buffer->info.dma_vaddr);
	if (!enrollment_list) {
		dev_err(lwis_client->lwis_dev->dev, "Cannot create enrollment list\n");
		goto err;
	}

	// Check if there was duplicated identical enrollment.
	list_for_each(it_enrollment, &enrollment_list->list) {
		old_buffer = list_entry(it_enrollment, struct lwis_enrolled_buffer, list_node);
		if (old_buffer->info.fd == buffer->info.fd &&
		    old_buffer->info.dma_vaddr == buffer->info.dma_vaddr) {
			dev_err(lwis_client->lwis_dev->dev, "Duplicate vaddr %pad for fd %d",
				&buffer->info.dma_vaddr, buffer->info.fd);
			goto err;
		}
	}

	list_add_tail(&buffer->list_node, &enrollment_list->list);
	buffer->enrollment_list = enrollment_list;

	return 0;
err:
	dma_buf_unmap_attachment(buffer->dma_buf_attachment, buffer->sg_table,
				 buffer->dma_direction);
	dma_buf_detach(buffer->dma_buf, buffer->dma_buf_attachment);
	dma_buf_put(buffer->dma_buf);
	return -EINVAL;
}

int lwis_buffer_disenroll(struct lwis_client *lwis_client, struct lwis_enrolled_buffer *buffer)
{
	char trace_name[LWIS_MAX_NAME_STRING_LEN];
	unsigned long flags;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	if (!lwis_client) {
		pr_err("Disenroll: LWIS client is NULL\n");
		return -ENODEV;
	}
	if (!buffer) {
		pr_err("Disenroll: buffer is NULL\n");
		return -EINVAL;
	}

	scnprintf(trace_name, LWIS_MAX_NAME_STRING_LEN, "lwis:buf_disenroll_%s",
		  lwis_client->lwis_dev->name);
	LWIS_ATRACE_FUNC_BEGIN(lwis_client->lwis_dev, trace_name);
	lwis_platform_dma_buffer_unmap(lwis_client->lwis_dev, buffer->dma_buf_attachment,
				       buffer->info.dma_vaddr);
	dma_buf_unmap_attachment(buffer->dma_buf_attachment, buffer->sg_table,
				 buffer->dma_direction);
	dma_buf_detach(buffer->dma_buf, buffer->dma_buf_attachment);
	dma_buf_put(buffer->dma_buf);
	LWIS_ATRACE_FUNC_END(lwis_client->lwis_dev, trace_name);
	spin_lock_irqsave(&lwis_dev->lock, flags);
	/* Delete the node from the hash table */
	list_del(&buffer->list_node);
	if (list_empty(&buffer->enrollment_list->list)) {
		hash_del(&buffer->enrollment_list->node);
		kfree(buffer->enrollment_list);
	}
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
	return 0;
}

struct lwis_enrolled_buffer *lwis_client_enrolled_buffer_find(struct lwis_client *lwis_client,
							      int fd, dma_addr_t dma_vaddr)
{
	struct lwis_buffer_enrollment_list *enrollment_list;
	struct lwis_enrolled_buffer *buffer;
	struct list_head *it_enrollment;

	if (!lwis_client) {
		pr_err("%s: LWIS client is NULL\n", __func__);
		return NULL;
	}

	enrollment_list = enrollment_list_find(lwis_client, dma_vaddr);
	if (!enrollment_list || list_empty(&enrollment_list->list))
		return NULL;

	list_for_each(it_enrollment, &enrollment_list->list) {
		buffer = list_entry(it_enrollment, struct lwis_enrolled_buffer, list_node);
		if (buffer->info.fd == fd && buffer->info.dma_vaddr == dma_vaddr)
			return buffer;
	}

	return NULL;
}

int lwis_buffer_cpu_access(struct lwis_client *lwis_client, struct lwis_buffer_cpu_access_op *op)
{
	struct dma_buf *dma_buf;
	enum dma_data_direction dma_direction;
	int ret = 0;

	if (!lwis_client) {
		pr_err("BufferCpuAccess: LWIS client is NULL\n");
		return -ENODEV;
	}

	if (op->read && op->write)
		dma_direction = DMA_BIDIRECTIONAL;
	else if (op->read)
		dma_direction = DMA_FROM_DEVICE;
	else if (op->write)
		dma_direction = DMA_TO_DEVICE;
	else
		dma_direction = DMA_NONE;

	if (!valid_dma_direction(dma_direction)) {
		dev_err(lwis_client->lwis_dev->dev, "BufferCpuAccess: dma_direction is invalid\n");
		return -EINVAL;
	}

	dma_buf = dma_buf_get(op->fd);
	if (IS_ERR_OR_NULL(dma_buf)) {
		pr_err("Could not get dma buffer for fd: %d", op->fd);
		return -EINVAL;
	}

	if (op->start)
		ret = dma_buf_begin_cpu_access_partial(dma_buf, dma_direction, op->offset, op->len);
	else
		ret = dma_buf_end_cpu_access_partial(dma_buf, dma_direction, op->offset, op->len);

	dma_buf_put(dma_buf);
	return ret;
}

int lwis_client_enrolled_buffers_clear(struct lwis_client *lwis_client)
{
	/* Our hash table iterator */
	struct lwis_buffer_enrollment_list *enrollment_list;
	/* Temporary vars for hash table traversal */
	struct hlist_node *n;
	int i;
	/* Enrollment list iterator */
	struct list_head *it_enrollment, *it_enrollment_tmp;
	struct lwis_enrolled_buffer *buffer;

	if (!lwis_client) {
		pr_err("%s: LWIS client is NULL\n", __func__);
		return -ENODEV;
	}

	/* Iterate over the entire hash table */
	hash_for_each_safe(lwis_client->enrolled_buffers, i, n, enrollment_list, node) {
		list_for_each_safe(it_enrollment, it_enrollment_tmp, &enrollment_list->list) {
			bool done = list_is_singular(&enrollment_list->list);
			buffer = list_entry(it_enrollment, struct lwis_enrolled_buffer, list_node);
			/* Disenroll the buffer */
			lwis_buffer_disenroll(lwis_client, buffer);
			/* Free the object */
			kfree(buffer);
			/* Stop now if enrollment_list->list was freed */
			if (done)
				break;
		}
	}

	return 0;
}

struct lwis_allocated_buffer *lwis_client_allocated_buffer_find(struct lwis_client *lwis_client,
								int fd)
{
	struct lwis_allocated_buffer *p;

	if (!lwis_client) {
		pr_err("%s: LWIS client is NULL\n", __func__);
		return NULL;
	}

	hash_for_each_possible(lwis_client->allocated_buffers, p, node, fd) {
		if (p->fd == fd)
			return p;
	}
	return NULL;
}

int lwis_client_allocated_buffers_clear(struct lwis_client *lwis_client)
{
	struct lwis_allocated_buffer *buffer;
	struct hlist_node *n;
	int i;

	if (!lwis_client) {
		pr_err("%s: LWIS client is NULL\n", __func__);
		return -ENODEV;
	}

	hash_for_each_safe(lwis_client->allocated_buffers, i, n, buffer, node) {
		if (lwis_client->lwis_dev->type != DEVICE_TYPE_SLC)
			lwis_buffer_free(lwis_client, buffer);
		else
			hash_del(&buffer->node);

		kfree(buffer);
	}
	return 0;
}
