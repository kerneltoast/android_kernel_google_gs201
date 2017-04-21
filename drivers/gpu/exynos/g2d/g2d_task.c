/*
 * linux/drivers/gpu/exynos/g2d/g2d_task.c
 *
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/exynos_iovmm.h>

#include "g2d.h"
#include "g2d_task.h"

void g2d_destroy_tasks(struct g2d_device *g2d_dev)
{
	struct g2d_task *task, *next;
	unsigned long flags;

	spin_lock_irqsave(&g2d_dev->lock_task, flags);

	task = g2d_dev->tasks;
	while (task != NULL) {
		next = task->next;

		list_del(&task->node);

		iovmm_unmap(g2d_dev->dev, task->cmd_addr);

		__free_pages(task->cmd_page, get_order(G2D_CMD_LIST_SIZE));

		kfree(task);

		task = next;
	}

	spin_unlock_irqrestore(&g2d_dev->lock_task, flags);
}

static struct g2d_task *g2d_create_task(struct g2d_device *g2d_dev)
{
	struct g2d_task *task;
	struct scatterlist sgl;
	int i;

	task = kzalloc(sizeof(*task), GFP_KERNEL);
	if (!task)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&task->node);

	task->cmd_page = alloc_pages(GFP_KERNEL, get_order(G2D_CMD_LIST_SIZE));
	if (!task->cmd_page)
		goto err_page;

	/* mapping the command data */
	sg_init_table(&sgl, 1);
	sg_set_page(&sgl, task->cmd_page, G2D_CMD_LIST_SIZE, 0);
	task->cmd_addr = iovmm_map(g2d_dev->dev, &sgl, 0, G2D_CMD_LIST_SIZE,
				   DMA_TO_DEVICE, IOMMU_READ | IOMMU_CACHE);

	for (i = 0; i < G2D_MAX_IMAGES; i++)
		task->source[i].task = task;
	task->target.task = task;

	task->g2d_dev = g2d_dev;

	return task;
err_page:
	kfree(task);

	return ERR_PTR(-ENOMEM);
}

int g2d_create_tasks(struct g2d_device *g2d_dev)
{
	struct g2d_task *task;
	unsigned int i;

	for (i = 0; i < G2D_MAX_JOBS; i++) {
		task = g2d_create_task(g2d_dev);

		if (IS_ERR(task)) {
			g2d_destroy_tasks(g2d_dev);
			return PTR_ERR(task);
		}

		task->job_id = i;

		task->next = g2d_dev->tasks;
		g2d_dev->tasks = task;
		list_add(&task->node, &g2d_dev->tasks_free);
	}

	return 0;
}
