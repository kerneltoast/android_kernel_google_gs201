/* SPDX-License-Identifier: GPL-2.0 */
/*
 * EdgeTPU support for buffers backed by dma-buf.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_DMABUF_H__
#define __EDGETPU_DMABUF_H__

#include <linux/seq_file.h>

#include "edgetpu-device-group.h"
#include "edgetpu-internal.h"
#include "edgetpu.h"

/*
 * Maps a dma-buf to a device group.
 *
 * @arg->device_address will be set as the mapped TPU VA on success.
 *
 * Returns zero on success or a negative errno on error.
 */
int edgetpu_map_dmabuf(struct edgetpu_device_group *group,
		       struct edgetpu_map_dmabuf_ioctl *arg);
/* unmap the dma-buf backed buffer from a device group */
int edgetpu_unmap_dmabuf(struct edgetpu_device_group *group, u32 die_index,
			 tpu_addr_t tpu_addr);
/*
 * Maps a list of dma-buf FDs to a device group.
 *
 * @arg->device_address will be set as the mapped TPU VA on success.
 *
 * Returns zero on success or a negative errno on error.
 */
int edgetpu_map_bulk_dmabuf(struct edgetpu_device_group *group,
			    struct edgetpu_map_bulk_dmabuf_ioctl *arg);
/* reverts edgetpu_map_bulk_dmabuf */
int edgetpu_unmap_bulk_dmabuf(struct edgetpu_device_group *group,
			      tpu_addr_t tpu_addr);
/* Create a DMA sync fence via ioctl */
int edgetpu_sync_fence_create(struct edgetpu_device_group *group,
			      struct edgetpu_create_sync_fence_data *datap);
/* Signal a DMA sync fence, optionally specifying error status */
int edgetpu_sync_fence_signal(struct edgetpu_signal_sync_fence_data *datap);
/* Return DMA sync fence status */
int edgetpu_sync_fence_status(struct edgetpu_sync_fence_status *datap);
/*
 * Send error signal to any remaining unsignalled DMA sync fences in a group being disbanded.
 * Caller holds group lock.
 */
void edgetpu_sync_fence_group_shutdown(struct edgetpu_device_group *group);
/* Dump sync fence info from debugfs */
int edgetpu_sync_fence_debugfs_show(struct seq_file *s, void *unused);

#endif /* __EDGETPU_DMABUF_H__ */
