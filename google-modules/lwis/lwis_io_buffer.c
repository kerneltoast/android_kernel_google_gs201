// SPDX-License-Identifier: GPL-2.0-only
/*
 * LWIS Buffer I/O Implementation
 *
 * Copyright (c) 2024 Google LLC.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-io_buffer: " fmt

#include <linux/bitops.h>
#include <linux/dma-buf.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "lwis_io_buffer.h"
#include "lwis_util.h"

struct pdma_buffer {
	struct iosys_map io_sys_map;
	struct dma_buf *dma_buf;
};

int lwis_io_buffer_map(struct lwis_device *lwis_dev, struct lwis_io_entry *entry)
{
	int ret;
	struct pdma_buffer *pdma_buffer;
	struct dma_buf *dma_buffer;
	uint8_t *k_data_to_write;

	pdma_buffer = kmalloc(sizeof(struct pdma_buffer), GFP_KERNEL);
	if (!pdma_buffer)
		return -ENOMEM;

	k_data_to_write = kmalloc(entry->write_to_buffer.size_in_bytes, GFP_KERNEL);
	if (!k_data_to_write) {
		ret = -ENOMEM;
		goto err_free_buf;
	}

	if (copy_from_user(k_data_to_write, (void __user *)(entry->write_to_buffer.bytes),
			   entry->write_to_buffer.size_in_bytes)) {
		dev_err(lwis_dev->dev,
			"PDMA buffer IO failed because bytes cannot be copied from user space");
		ret = -EFAULT;
		goto err_free_buf;
	}

	dma_buffer = dma_buf_get(entry->write_to_buffer.fd);
	if (IS_ERR(dma_buffer)) {
		dev_err(lwis_dev->dev, "PDMA buffer IO failed because dma_buf_get failed");
		ret = PTR_ERR(dma_buffer);
		goto err_free_buf;
	}

	ret = dma_buf_vmap(dma_buffer, &pdma_buffer->io_sys_map);
	if (ret) {
		dev_err(lwis_dev->dev, "PDMA buffer IO failed because vmap failed");
		goto err_dma_put;
	}

	ret = dma_buf_begin_cpu_access(dma_buffer, DMA_BIDIRECTIONAL);
	if (ret) {
		dev_err(lwis_dev->dev,
			"PDMA buffer IO failed because CPU cannot have access to the buffer");
		goto err_dma_vunmap;
	}

	pdma_buffer->dma_buf = dma_buffer;
	entry->write_to_buffer.buffer = pdma_buffer;
	entry->write_to_buffer.bytes = k_data_to_write;

	return 0;

err_dma_vunmap:
	dma_buf_vunmap(dma_buffer, &pdma_buffer->io_sys_map);
err_dma_put:
	dma_buf_put(dma_buffer);
err_free_buf:
	kfree(k_data_to_write);
	kfree(pdma_buffer);

	return ret;
}

void lwis_io_buffer_unmap(struct lwis_io_entry *entry)
{
	struct pdma_buffer *pdma_buffer = entry->write_to_buffer.buffer;

	dma_buf_end_cpu_access(pdma_buffer->dma_buf, DMA_BIDIRECTIONAL);
	dma_buf_vunmap(pdma_buffer->dma_buf, &pdma_buffer->io_sys_map);
	dma_buf_put(pdma_buffer->dma_buf);
	kfree(entry->write_to_buffer.bytes);
	kfree(entry->write_to_buffer.buffer);

	entry->write_to_buffer.bytes = NULL;
	entry->write_to_buffer.buffer = NULL;
}

int lwis_io_buffer_write(struct lwis_device *lwis_dev, struct lwis_io_entry *entry)
{
	struct pdma_buffer *pdma_buffer = entry->write_to_buffer.buffer;
	void *kernel_address;

	if (pdma_buffer->io_sys_map.is_iomem)
		kernel_address = pdma_buffer->io_sys_map.vaddr_iomem;
	else
		kernel_address = pdma_buffer->io_sys_map.vaddr;

	memcpy(kernel_address + entry->write_to_buffer.offset, entry->write_to_buffer.bytes,
	       entry->write_to_buffer.size_in_bytes);
	return 0;
}
