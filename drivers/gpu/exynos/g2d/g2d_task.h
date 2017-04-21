/*
 * linux/drivers/gpu/exynos/g2d/g2d_task.h
 *
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 *
 * Contact: Hyesoo Yu <hyesoo.yu@samsung.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __EXYNOS_G2D_TASK_H__
#define __EXYNOS_G2D_TASK_H__

#include <linux/dma-buf.h>

#include "g2d_format.h"

#define G2D_MAX_IMAGES		16
#define G2D_MAX_JOBS		16
#define G2D_CMD_LIST_SIZE	8192

struct g2d_buffer {
	union {
		struct {
			unsigned int			offset;
			struct dma_buf			*dmabuf;
			struct dma_buf_attachment	*attachment;
			struct sg_table			*sgt;
		} dmabuf;
		struct {
			unsigned long			addr;
			struct vm_area_struct		*vma;
		} userptr;
	};
	unsigned int	length;
	unsigned int	payload;
	dma_addr_t	dma_addr;
};

struct g2d_layer {
	struct g2d_task		*task;
	int			flags;
	int			buffer_type;
	int			num_buffers;
	struct g2d_buffer	buffer[G2D_MAX_PLANES];
};

#define G2D_TASKSTATE_WAITING		1
#define G2D_TASKSTATE_UNPREPARED	2
#define G2D_TASKSTATE_PREPARED		3
#define G2D_TASKSTATE_ACTIVE		4
#define G2D_TASKSTATE_PROCESSED		5
#define G2D_TASKSTATE_ERROR		6
#define G2D_TASKSTATE_KILLED		7
#define G2D_TASKSTATE_TIMEOUT		8

struct g2d_context;
struct g2d_device;

struct g2d_task {
	struct list_head	node;
	struct g2d_task		*next;
	struct g2d_device	*g2d_dev;

	unsigned int		job_id;

	struct g2d_layer	source[G2D_MAX_IMAGES];
	struct g2d_layer	target;
	unsigned int		num_source;

	/* Command list */
	struct page		*cmd_page;
	dma_addr_t		cmd_addr;
	unsigned int		cmd_count;

	unsigned int		priority;
};

void g2d_destroy_tasks(struct g2d_device *g2d_dev);
int g2d_create_tasks(struct g2d_device *g2d_dev);

#endif /*__EXYNOS_G2D_TASK_H__*/
