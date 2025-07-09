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

int lwis_io_buffer_write(struct lwis_device *lwis_dev, struct lwis_io_entry *entry)
{
	void *kernel_address;
	struct iosys_map *sys_map = entry->write_to_buffer.buffer->io_sys_map;

	if (sys_map->is_iomem)
		kernel_address = sys_map->vaddr_iomem;
	else
		kernel_address = sys_map->vaddr;

	memcpy(kernel_address + entry->write_to_buffer.offset, entry->write_to_buffer.bytes,
	       entry->write_to_buffer.size_in_bytes);
	return 0;
}
