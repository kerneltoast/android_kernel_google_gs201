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
#include <linux/pm_runtime.h>
#include <linux/exynos_iovmm.h>

#include "g2d.h"
#include "g2d_task.h"

static void g2d_schedule_task(struct g2d_task *task)
{
	struct g2d_device *g2d_dev = task->g2d_dev;
	unsigned long flags;
	int ret;

	/*
	 * Unconditional invocation of pm_runtime_get_sync() has no side effect
	 * in g2d_schedule(). It just increases the usage count of RPM if this
	 * function skips calling g2d_device_run(). The skip only happens when
	 * there is no task to run in g2d_dev->tasks_prepared.
	 * If pm_runtime_get_sync() enabled power, there must be a task in
	 * g2d_dev->tasks_prepared.
	 */
	ret = pm_runtime_get_sync(g2d_dev->dev);
	if (ret < 0) {
		dev_err(g2d_dev->dev, "Failed to enable power (%d)\n", ret);
		/* TODO: cancel task */
		return;
	}

	spin_lock_irqsave(&g2d_dev->lock_task, flags);

	list_add_tail(&task->node, &g2d_dev->tasks_active);

	change_task_state_prepared(task);
	change_task_state_active(task);
	/*
	 * g2d_device_run() is not reentrant while g2d_schedule() is
	 * reentrant g2d_device_run() should be protected with
	 * g2d_dev->lock_task from race.
	 */
	if (g2d_device_run(g2d_dev, task) < 0) {
		pm_runtime_put(g2d_dev->dev);
		/* TODO: cancel task */
	}

	spin_unlock_irqrestore(&g2d_dev->lock_task, flags);
}

static void g2d_task_schedule_work(struct work_struct *work)
{
	g2d_schedule_task(container_of(work, struct g2d_task, work));
}

static void g2d_queuework_task(struct kref *kref)
{
	struct g2d_task *task = container_of(kref, struct g2d_task, starter);
	struct g2d_device *g2d_dev = task->g2d_dev;
	bool failed;

	failed = !queue_work(g2d_dev->schedule_workq, &task->work);

	BUG_ON(failed);
}

void g2d_start_task(struct g2d_task *task)
{
	reinit_completion(&task->completion);

	kref_put(&task->starter, g2d_queuework_task);
}

struct g2d_task *g2d_get_free_task(struct g2d_device *g2d_dev)
{
	struct g2d_task *task;
	unsigned long flags;

	spin_lock_irqsave(&g2d_dev->lock_task, flags);

	if (list_empty(&g2d_dev->tasks_free)) {
		spin_unlock_irqrestore(&g2d_dev->lock_task, flags);
		return NULL;
	}

	task = list_first_entry(&g2d_dev->tasks_free, struct g2d_task, node);
	list_del_init(&task->node);
	INIT_WORK(&task->work, g2d_task_schedule_work);

	init_task_state(task);

	spin_unlock_irqrestore(&g2d_dev->lock_task, flags);

	return task;
}

void g2d_put_free_task(struct g2d_device *g2d_dev, struct g2d_task *task)
{
	unsigned long flags;

	spin_lock_irqsave(&g2d_dev->lock_task, flags);

	clear_task_state(task);

	list_add(&task->node, &g2d_dev->tasks_free);

	spin_unlock_irqrestore(&g2d_dev->lock_task, flags);
}

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

	destroy_workqueue(g2d_dev->schedule_workq);
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

	init_completion(&task->completion);

	return task;
err_page:
	kfree(task);

	return ERR_PTR(-ENOMEM);
}

int g2d_create_tasks(struct g2d_device *g2d_dev)
{
	struct g2d_task *task;
	unsigned int i;

	g2d_dev->schedule_workq = create_singlethread_workqueue("g2dscheduler");
	if (!g2d_dev->schedule_workq)
		return -ENOMEM;

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
