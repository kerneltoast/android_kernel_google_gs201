/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Defines the interface of device DRAM management.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_DRAM_H__
#define __EDGETPU_DRAM_H__

#include <linux/dma-buf.h>
#include <linux/seq_file.h>

#include "edgetpu-internal.h"
#include "edgetpu-mmu.h"
#include "edgetpu.h"

#if IS_ENABLED(CONFIG_EDGETPU_DEVICE_DRAM)

#define EDGETPU_HAS_DEVICE_DRAM

/* Initializes structures for device DRAM management. */
int edgetpu_device_dram_init(struct edgetpu_dev *etdev);

/* Teardown actions for device DRAM management. */
void edgetpu_device_dram_exit(struct edgetpu_dev *etdev);

/*
 * Returns the dma-buf FD allocated on the @client's DRAM with size @size.
 */
int edgetpu_device_dram_getfd(struct edgetpu_client *client, u64 size);

/*
 * Sets translations up to have dies access other dies' DRAM.
 *
 * This function is called when finalizing @group, @group's state is WAITING but
 * edgetpu_device_group_nth_etdev() is ready to use.
 *
 * Caller holds @group->lock.
 *
 * Returns 0 on success, or -errno on error.
 */
int edgetpu_group_setup_remote_dram(struct edgetpu_device_group *group);

/*
 * Reverts edgetpu_group_setup_remote_dram().
 * This function is called when disbanding @group.
 *
 * Caller holds @group->lock.
 */
void edgetpu_group_remove_remote_dram(struct edgetpu_device_group *group);

/*
 * Allocate device DRAM for internal driver use.
 * Returns page-aligned kernel VA with length @size, or NULL if not available.
 *
 * @size must be page-aligned.
 * @tpu_phys returns the TPU space physical address of the allocation.
 */
void __iomem *edgetpu_device_dram_alloc(struct edgetpu_dev *etdev, u64 size,
					phys_addr_t *tpu_phys);

/* Free device DRAM allocated via edgetpu_device_dram_alloc. */
void edgetpu_device_dram_free(struct edgetpu_dev *etdev, void __iomem *kva,
			      u64 size);

/* Add any private debug info related to @dmabuf in seq_file @s. */
void edgetpu_device_dram_dmabuf_info_show(struct dma_buf *dmabuf,
					  struct seq_file *s);

/* Return amount of on-device DRAM currently used in bytes. */
size_t edgetpu_device_dram_used(struct edgetpu_dev *etdev);

/* Return the amount of free device dram in bytes */
size_t edgetpu_device_dram_available(struct edgetpu_dev *etdev);

#else /* !CONFIG_EDGETPU_DEVICE_DRAM */

static inline int edgetpu_device_dram_init(struct edgetpu_dev *etdev)
{
	/*
	 * Returns zero since nothing has to be done in initialization if the
	 * device DRAM is not supported.
	 */
	return 0;
}

static inline void edgetpu_device_dram_exit(struct edgetpu_dev *etdev)
{
}

static inline int edgetpu_device_dram_getfd(struct edgetpu_client *client,
					    u64 size)
{
	return -ENODEV;
}

static inline int
edgetpu_group_setup_remote_dram(struct edgetpu_device_group *group)
{
	/*
	 * Returns 0 since chip without on-device DRAM won't access remote
	 * DRAM.
	 */
	return 0;
}

static inline void
edgetpu_group_remove_remote_dram(struct edgetpu_device_group *group)
{
}

static inline void __iomem *edgetpu_device_dram_alloc(struct edgetpu_dev *etdev,
						      u64 size,
						      phys_addr_t *tpu_phys)
{
	return NULL;
}

static inline void edgetpu_device_dram_free(struct edgetpu_dev *etdev,
					    void __iomem *kva, u64 size)
{
}

static inline void edgetpu_device_dram_dmabuf_info_show(struct dma_buf *dmabuf,
							struct seq_file *s)
{
}

static inline size_t edgetpu_device_dram_used(struct edgetpu_dev *etdev)
{
	return 0;
}

static inline size_t edgetpu_device_dram_available(struct edgetpu_dev *etdev)
{
	return 0;
}
#endif /* CONFIG_EDGETPU_DEVICE_DRAM */

#endif /* __EDGETPU_DRAM_H__ */
