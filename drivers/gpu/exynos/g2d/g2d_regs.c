/*
 * linux/drivers/gpu/exynos/g2d/g2d_regs.c
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
#include <linux/io.h>

#include "g2d.h"
#include "g2d_regs.h"
#include "g2d_task.h"

void g2d_hw_push_task(struct g2d_device *g2d_dev, struct g2d_task *task)
{
	u32 state = g2d_hw_get_job_state(g2d_dev, task->job_id);

	if (state != G2D_JOB_STATE_DONE)
		dev_err(g2d_dev->dev, "%s: Unexpected state %#x of JOB %d\n",
			__func__, state, task->job_id);

	writel_relaxed(G2D_JOB_HEADER_DATA(task->priority, task->job_id),
			g2d_dev->reg + G2D_JOB_HEADER_REG);

	writel_relaxed(G2D_ERR_INT_ENABLE, g2d_dev->reg + G2D_INTEN_REG);

	writel_relaxed(task->cmd_addr, g2d_dev->reg + G2D_JOB_BASEADDR_REG);
	writel_relaxed(task->cmd_count, g2d_dev->reg + G2D_JOB_SFRNUM_REG);
	writel_relaxed(1 << task->job_id, g2d_dev->reg + G2D_JOB_INT_ID_REG);
	writel(G2D_JOBPUSH_INT_ENABLE, g2d_dev->reg + G2D_JOB_PUSH_REG);
}

static const char *error_desc[3] = {
	"AFBC Stuck",
	"No read response",
	"No write response",
};

u32 g2d_hw_errint_status(struct g2d_device *g2d_dev)
{
	u32 status = readl_relaxed(g2d_dev->reg + G2D_INTC_PEND_REG);
	u32 errstatus = status;
	int idx;

	/* IRQPEND_SCF should not be set because we don't use ABP mode */
	BUG_ON((status & 0x1) == 1);

	errstatus >>= 16;
	errstatus &= 0x7;

	if (errstatus == 0)
		return 0;

	for (idx = 0; idx < 3; idx++) {
		if (errstatus & (1 << idx))
			dev_err(g2d_dev->dev,
				"G2D ERROR INTERRUPT: %s\n", error_desc[idx]);
	}

	dev_err(g2d_dev->dev, "G2D FIFO STATUS: %#x\n",
		g2d_hw_fifo_status(g2d_dev));

	return status;
}

int g2d_hw_get_current_task(struct g2d_device *g2d_dev)
{
	int i, val;

	for (i = 0; i < G2D_MAX_JOBS; i++) {
		val = readl_relaxed(g2d_dev->reg + G2D_JOB_IDn_STATE_REG(i));
		if ((val & G2D_JOB_STATE_MASK) == G2D_JOB_STATE_RUNNING)
			return i;
	}

	return -1;
}
