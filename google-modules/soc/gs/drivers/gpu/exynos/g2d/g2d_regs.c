// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/property.h>

#include <linux/dma-map-ops.h>

#include "g2d.h"
#include "g2d_regs.h"
#include "g2d_task.h"
#include "g2d_uapi.h"
#include "g2d_debug.h"
#include "g2d_secure.h"

void g2d_hw_push_task(struct g2d_device *g2d_dev, struct g2d_task *task)
{
	u32 state = g2d_hw_get_job_state(g2d_dev, g2d_task_id(task));

	if (state != G2D_JOB_STATE_DONE)
		perrfndev(g2d_dev, "Unexpected state %#x of JOB %d",
			  state, g2d_task_id(task));

	if (IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)) {
		unsigned int i;

		state = 0;

		for (i = 0; i < task->num_source; i++)
			if (task->source[i].flags & G2D_LAYERFLAG_SECURE)
#if IS_ENABLED(CONFIG_SOC_ZUMA)
				state |= 1 << (i + 1);
#else
				state |= 1 << i;
#endif

		if ((task->target.flags & G2D_LAYERFLAG_SECURE) || state)
			state |= 1 << 24;

		writel_relaxed(state,
			       g2d_dev->reg + G2D_JOB_N_LAYER_SECURE_REG(g2d_task_id(task)));
	}

	if (device_get_dma_attr(g2d_dev->dev) != DEV_DMA_COHERENT)
		dma_sync_single_for_device(g2d_dev->dev, task->cmd_addr,
					   G2D_CMD_LIST_SIZE, DMA_TO_DEVICE);

	writel_relaxed(G2D_JOB_HEADER_DATA(task->sec.priority, g2d_task_id(task)),
		       g2d_dev->reg + G2D_JOB_HEADER_REG);

	writel_relaxed(G2D_ERR_INT_ENABLE, g2d_dev->reg + G2D_INTEN_REG);

	writel_relaxed(task->cmd_addr, g2d_dev->reg + G2D_JOB_BASEADDR_REG);
	writel_relaxed(task->sec.cmd_count, g2d_dev->reg + G2D_JOB_SFRNUM_REG);
	writel_relaxed(1 << g2d_task_id(task), g2d_dev->reg + G2D_JOB_INT_ID_REG);
	writel(G2D_JOBPUSH_INT_ENABLE, g2d_dev->reg + G2D_JOB_PUSH_REG);
}

bool g2d_hw_stuck_state(struct g2d_device *g2d_dev)
{
	int i, val;
	int retry_count = 10;
	bool is_idle = true;
	bool is_running = false;

	while (retry_count-- > 0) {
		for (i = 0; i < G2D_MAX_JOBS; i++) {
			val = readl_relaxed(g2d_dev->reg + G2D_JOB_ID_N_STATE_REG(i));

			val &= G2D_JOB_STATE_MASK;

			if (val != G2D_JOB_STATE_DONE)
				is_idle = false;

			if (val == G2D_JOB_STATE_RUNNING)
				is_running = true;
		}

		if (is_idle || is_running)
			return false;

		is_idle = true;
	}

	return true;
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
	WARN((status & 0x1) == 1, "Unexpected interrupt status bit(0) in %#x", status);

	errstatus >>= 16;
	errstatus &= 0x7;

	if (errstatus == 0)
		return 0;

	for (idx = 0; idx < 3; idx++)
		if (errstatus & (1 << idx))
			perrdev(g2d_dev, "G2D ERROR INTERRUPT: %s", error_desc[idx]);

	perrdev(g2d_dev, "G2D FIFO STATUS: %#x", g2d_hw_fifo_status(g2d_dev));

	return status;
}

/*
 * This is called when H/W is judged to be in operation,
 * for example, when a sysmmu fault and an error interrupt occurs.
 * If there is no task in progress, the status of all task on H/W
 * is taken and print for debugging
 */
int g2d_hw_get_current_task(struct g2d_device *g2d_dev)
{
	int i, val;

	for (i = 0; i < G2D_MAX_JOBS; i++) {
		val = readl_relaxed(g2d_dev->reg + G2D_JOB_ID_N_STATE_REG(i));
		if ((val & G2D_JOB_STATE_MASK) == G2D_JOB_STATE_RUNNING)
			return i;
	}

	for (i = 0; i < G2D_MAX_JOBS; i++) {
		val = readl_relaxed(g2d_dev->reg + G2D_JOB_ID_N_STATE_REG(i));
		perrdev(g2d_dev, "G2D TASK[%03d] STATE : %d", i, val);
	}

	return -1;
}
