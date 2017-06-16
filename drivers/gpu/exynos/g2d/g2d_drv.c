/*
 * linux/drivers/gpu/exynos/g2d/g2d_drv.c
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

#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/exynos_iovmm.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "g2d.h"
#include "g2d_regs.h"
#include "g2d_task.h"
#include "g2d_uapi_process.h"
#include "g2d_debug.h"
#include "g2d_perf.h"

#define MODULE_NAME "exynos-g2d"

static int g2d_update_priority(struct g2d_context *ctx,
					    enum g2d_priority priority)
{
	struct g2d_device *g2d_dev = ctx->g2d_dev;
	struct g2d_task *task;
	unsigned long flags;

	if (ctx->priority == priority)
		return 0;

	atomic_dec(&g2d_dev->prior_stats[ctx->priority]);
	atomic_inc(&g2d_dev->prior_stats[priority]);
	ctx->priority = priority;

	/*
	 * check lower priority task in use, and return EBUSY
	 * for higher priority to avoid waiting lower task completion
	 */
	spin_lock_irqsave(&g2d_dev->lock_task, flags);

	for (task = g2d_dev->tasks; task != NULL; task = task->next) {
		if (!is_task_state_idle(task) && (task->priority < priority)) {
			spin_unlock_irqrestore(&g2d_dev->lock_task, flags);
			return -EBUSY;
		}
	}

	spin_unlock_irqrestore(&g2d_dev->lock_task, flags);

	return 0;
}

void g2d_hw_timeout_handler(unsigned long arg)
{
	struct g2d_task *task = (struct g2d_task *)arg;
	struct g2d_device *g2d_dev = task->g2d_dev;
	unsigned long flags;
	u32 job_state;

	/* TODO: Dump of internal state of G2D */

	dev_err(g2d_dev->dev, "%s: Time is up: %d msec for job %d\n",
		__func__, G2D_HW_TIMEOUT_MSEC, task->job_id);

	spin_lock_irqsave(&g2d_dev->lock_task, flags);

	if (!is_task_state_active(task))
		/*
		 * The task timed out is not currently running in H/W.
		 * It might be just finished by interrupt.
		 */
		goto out;

	job_state = g2d_hw_get_job_state(g2d_dev, task->job_id);
	if (job_state == G2D_JOB_STATE_DONE)
		/*
		 * The task timed out is not currently running in H/W.
		 * It will be processed in the interrupt handler.
		 */
		goto out;

	if (is_task_state_killed(task)) {
		/* The killed task is not died in the time out priod. */
		g2d_hw_global_reset(g2d_dev);

		g2d_flush_all_tasks(g2d_dev);

		dev_err(g2d_dev->dev,
			"GLOBAL RESET: killed task not dead in %d msec.\n",
			G2D_HW_TIMEOUT_MSEC);
		goto out;
	}

	mod_timer(&task->timer,
	  jiffies + msecs_to_jiffies(G2D_HW_TIMEOUT_MSEC));

	if (job_state != G2D_JOB_STATE_RUNNING)
		/* G2D_JOB_STATE_QUEUEING or G2D_JOB_STATE_SUSPENDING */
		/* Time out is not caused by this task */
		goto out;

	g2d_stamp_task(task, G2D_STAMP_STATE_TIMEOUT_HW);

	mark_task_state_killed(task);

	g2d_hw_kill_task(g2d_dev, task->job_id);

out:
	spin_unlock_irqrestore(&g2d_dev->lock_task, flags);
}

int g2d_device_run(struct g2d_device *g2d_dev, struct g2d_task *task)
{
	g2d_hw_push_task(g2d_dev, task);

	g2d_stamp_task(task, G2D_STAMP_STATE_PUSH);

	if (IS_HWFC(task->flags))
		hwfc_set_valid_buffer(task->job_id, task->job_id);

	return 0;
}

static irqreturn_t g2d_irq_handler(int irq, void *priv)
{
	struct g2d_device *g2d_dev = priv;
	unsigned int id;
	u32 intflags, errstatus;

	spin_lock(&g2d_dev->lock_task);

	intflags = g2d_hw_finished_job_ids(g2d_dev);
	if (intflags != 0) {
		for (id = 0; id < G2D_MAX_JOBS; id++) {
			if ((intflags & (1 << id)) == 0)
				continue;

			g2d_finish_task_with_id(g2d_dev, id, true);
		}

		g2d_hw_clear_job_ids(g2d_dev, intflags);
	}

	errstatus = g2d_hw_errint_status(g2d_dev);
	if (errstatus != 0) {
		int job_id = g2d_hw_get_current_task(g2d_dev);
		struct g2d_task *task =
				g2d_get_active_task_from_id(g2d_dev, job_id);

		if (job_id < 0) {
			dev_err(g2d_dev->dev, "No task is running in HW\n");
		} else if (task == NULL) {
			dev_err(g2d_dev->dev,
				"%s: Current job %d in HW is not active\n",
				__func__, job_id);
		} else {
			dev_err(g2d_dev->dev,
				"%s: Error occurred during running job %d\n",
				__func__, job_id);

			g2d_stamp_task(task, G2D_STAMP_STATE_ERR_INT);
		}

		g2d_flush_all_tasks(g2d_dev);

		g2d_hw_global_reset(g2d_dev);

		g2d_hw_clear_int(g2d_dev, errstatus);
	}

	spin_unlock(&g2d_dev->lock_task);

	wake_up(&g2d_dev->freeze_wait);

	return IRQ_HANDLED;
}

#ifdef CONFIG_EXYNOS_IOVMM
static int g2d_iommu_fault_handler(struct iommu_domain *domain,
				struct device *dev, unsigned long fault_addr,
				int fault_flags, void *token)
{
	struct g2d_device *g2d_dev = token;
	struct g2d_task *task;
	int job_id = g2d_hw_get_current_task(g2d_dev);
	unsigned long flags;

	spin_lock_irqsave(&g2d_dev->lock_task, flags);
	task = g2d_get_active_task_from_id(g2d_dev, job_id);
	spin_unlock_irqrestore(&g2d_dev->lock_task, flags);

	g2d_stamp_task(task, G2D_STAMP_STATE_MMUFAULT);

	return 0;
}
#endif

static __u32 get_hw_version(struct g2d_device *g2d_dev, __u32 *version)
{
	int ret;

	ret = pm_runtime_get_sync(g2d_dev->dev);
	if (ret < 0) {
		dev_err(g2d_dev->dev, "Failed to enable power (%d)\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(g2d_dev->clock);
	if (ret < 0) {
		dev_err(g2d_dev->dev, "Failed to enable clock (%d)\n", ret);
	} else {
		*version = readl_relaxed(g2d_dev->reg + G2D_VERSION_INFO_REG);
		clk_disable(g2d_dev->clock);
	}

	pm_runtime_put(g2d_dev->dev);

	return ret;
}

static int g2d_open(struct inode *inode, struct file *filp)
{
	struct g2d_device *g2d_dev = container_of(filp->private_data,
						  struct g2d_device, misc);
	struct g2d_context *g2d_ctx;

	g2d_ctx = kzalloc(sizeof(*g2d_ctx), GFP_KERNEL);
	if (!g2d_ctx)
		return -ENOMEM;

	filp->private_data = g2d_ctx;

	g2d_ctx->g2d_dev = g2d_dev;

	g2d_ctx->priority = G2D_DEFAULT_PRIORITY;
	atomic_inc(&g2d_dev->prior_stats[g2d_ctx->priority]);

	INIT_LIST_HEAD(&g2d_ctx->qos_node);

	return 0;
}

static int g2d_release(struct inode *inode, struct file *filp)
{
	struct g2d_context *g2d_ctx = filp->private_data;
	struct g2d_device *g2d_dev = g2d_ctx->g2d_dev;

	atomic_dec(&g2d_dev->prior_stats[g2d_ctx->priority]);

	if (g2d_ctx->hwfc_info) {
		int i;

		for (i = 0; i < g2d_ctx->hwfc_info->buffer_count; i++)
			dma_buf_put(g2d_ctx->hwfc_info->bufs[i]);
		kfree(g2d_ctx->hwfc_info);
	}

	g2d_put_performance(g2d_ctx);

	kfree(g2d_ctx);

	return 0;
}

static long g2d_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct g2d_context *ctx = filp->private_data;
	struct g2d_device *g2d_dev = ctx->g2d_dev;
	int ret = 0;

	switch (cmd) {
	case G2D_IOC_PROCESS:
	{
		struct g2d_task_data __user *uptr =
				(struct g2d_task_data __user *)arg;
		struct g2d_task_data data;
		struct g2d_task *task;
		int i;

		/*
		 * A process that has lower priority is not allowed
		 * to execute and simply returns -EBUSY
		 */
		for (i = ctx->priority + 1; i < G2D_PRIORITY_END; i++) {
			if (atomic_read(&g2d_dev->prior_stats[i]) > 0) {
				ret =  -EBUSY;
				break;
			}
		}
		if (ret)
			break;

		if (copy_from_user(&data, uptr, sizeof(data))) {
			dev_err(g2d_dev->dev,
				"%s: Failed to read g2d_task_data\n", __func__);
			ret = -EFAULT;
			break;
		}

		/*
		 * If the task should be run by hardware flow control with MFC,
		 * driver must request shared buffer to repeater driver if it
		 * has not been previously requested at current context.
		 * hwfc_request_buffer has increased the reference count inside
		 * function, so driver must reduce the reference
		 * when context releases.
		 */
		if (IS_HWFC(data.flags) && !ctx->hwfc_info) {
			ctx->hwfc_info = kzalloc(
					sizeof(*ctx->hwfc_info), GFP_KERNEL);
			if (!ctx->hwfc_info) {
				ret = -ENOMEM;
				break;
			}

			ret = hwfc_request_buffer(ctx->hwfc_info, 0);
			if (ret ||
				(ctx->hwfc_info->buffer_count >
				 MAX_SHARED_BUF_NUM)) {
				kfree(ctx->hwfc_info);
				ctx->hwfc_info = NULL;
				dev_err(g2d_dev->dev,
					"%s: Failed to read hwfc info\n",
					__func__);
				break;
			}
		}

		task = g2d_get_free_task(g2d_dev, ctx, IS_HWFC(data.flags));
		if (task == NULL) {
			ret = -EBUSY;
			break;
		}

		kref_init(&task->starter);

		ret = g2d_get_userdata(g2d_dev, ctx, task, &data);
		if (ret < 0) {
			g2d_put_free_task(g2d_dev, task);
			break;
		}

		g2d_stamp_task(task, G2D_STAMP_STATE_BEGIN);

		g2d_start_task(task);

		if (!(task->flags & G2D_FLAG_NONBLOCK))
			ret = g2d_wait_put_user(g2d_dev, task,
						uptr, data.flags);

		break;
	}
	case G2D_IOC_PRIORITY:
	{
		enum g2d_priority data;

		if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
			dev_err(g2d_dev->dev,
				"%s: Failed to get priority\n", __func__);
			ret = -EFAULT;
			break;
		}

		if ((data < G2D_LOW_PRIORITY) || (data >= G2D_PRIORITY_END)) {
			dev_err(g2d_dev->dev,
				"%s: Wrong priority %u\n", __func__, data);
			ret = -EINVAL;
			break;
		}

		ret = g2d_update_priority(ctx, data);

		break;
	}
	case G2D_IOC_PERFORMANCE:
	{
		struct g2d_performance_data data;

		if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
			dev_err(g2d_dev->dev,
				"%s: Failed to read perf data\n", __func__);
			ret = -EFAULT;
			break;
		}
		g2d_set_performance(ctx, &data);

		break;
	}
	}

	return ret;
}

static long g2d_compat_ioctl(struct file *filp,
			     unsigned int cmd, unsigned long arg)
{
	return 0;
}

static const struct file_operations g2d_fops = {
	.owner          = THIS_MODULE,
	.open           = g2d_open,
	.release        = g2d_release,
	.unlocked_ioctl	= g2d_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= g2d_compat_ioctl,
#endif
};

static int g2d_notifier_event(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	struct g2d_device *g2d_dev;

	g2d_dev = container_of(this, struct g2d_device, pm_notifier);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		g2d_prepare_suspend(g2d_dev);
		break;

	case PM_POST_SUSPEND:
		g2d_suspend_finish(g2d_dev);
		break;
	}

	return NOTIFY_OK;
}

static unsigned int g2d_default_ppc[G2D_PPC_END] =
	{3500, 3200, 3500, 3000,
	3500, 3100, 3000, 2800,
	3800, 3500, 2800, 2500};

static struct g2d_dvfs_table g2d_default_dvfs_table[] = {
	{534000, 711000},
	{400000, 534000},
	{336000, 400000},
	{267000, 356000},
	{178000, 200000},
	{107000, 134000},
};

static int g2d_parse_dt(struct g2d_device *g2d_dev)
{
	struct device *dev = g2d_dev->dev;
	int i, len;

	if (of_property_read_u32_array(dev->of_node, "hw_ppc",
			(u32 *)g2d_dev->hw_ppc,
			(size_t)(ARRAY_SIZE(g2d_dev->hw_ppc)))) {
		dev_err(dev, "Failed to parse device tree for hw ppc");

		for (i = 0; i < G2D_PPC_END; i++)
			g2d_dev->hw_ppc[i] = g2d_default_ppc[i];
	}

	len = of_property_count_u32_elems(dev->of_node, "g2d_dvfs_table");
	if (len < 0)
		g2d_dev->dvfs_table_cnt = ARRAY_SIZE(g2d_default_dvfs_table);
	else
		g2d_dev->dvfs_table_cnt = len / 2;

	g2d_dev->dvfs_table = devm_kzalloc(dev,
				sizeof(struct g2d_dvfs_table) *
				g2d_dev->dvfs_table_cnt,
				GFP_KERNEL);
	if (!g2d_dev->dvfs_table)
		return -ENOMEM;

	if (len < 0) {
		memcpy(g2d_dev->dvfs_table, g2d_default_dvfs_table,
			sizeof(struct g2d_dvfs_table) *
			g2d_dev->dvfs_table_cnt);
	} else {
		of_property_read_u32_array(dev->of_node, "g2d_dvfs_table",
				(unsigned int *)g2d_dev->dvfs_table, len);
	}

	return 0;
}

static int g2d_probe(struct platform_device *pdev)
{
	struct g2d_device *g2d_dev;
	struct resource *res;
	__u32 version;
	int ret;

	g2d_dev = devm_kzalloc(&pdev->dev, sizeof(*g2d_dev), GFP_KERNEL);
	if (!g2d_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, g2d_dev);
	g2d_dev->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	g2d_dev->reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(g2d_dev->reg))
		return PTR_ERR(g2d_dev->reg);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get IRQ resource");
		return -ENOENT;
	}

	ret = devm_request_irq(&pdev->dev, res->start,
			       g2d_irq_handler, 0, pdev->name, g2d_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to install IRQ handler");
		return ret;
	}

	g2d_dev->clock = devm_clk_get(&pdev->dev, "gate");
	if (IS_ERR(g2d_dev->clock)) {
		dev_err(&pdev->dev, "Failed to get clock (%ld)\n",
					PTR_ERR(g2d_dev->clock));
		return PTR_ERR(g2d_dev->clock);
	}

	iovmm_set_fault_handler(&pdev->dev, g2d_iommu_fault_handler, g2d_dev);

	ret = g2d_parse_dt(g2d_dev);
	if (ret < 0)
		return ret;

	ret = iovmm_activate(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to activate iommu\n");
		return ret;
	}

	/* prepare clock and enable runtime pm */
	pm_runtime_enable(&pdev->dev);

	ret = get_hw_version(g2d_dev, &version);
	if (ret < 0)
		goto err;

	g2d_dev->misc.minor = MISC_DYNAMIC_MINOR;
	g2d_dev->misc.name = "g2d";
	g2d_dev->misc.fops = &g2d_fops;

	/* misc register */
	ret = misc_register(&g2d_dev->misc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register misc device");
		goto err;
	}

	spin_lock_init(&g2d_dev->lock_task);

	INIT_LIST_HEAD(&g2d_dev->tasks_free);
	INIT_LIST_HEAD(&g2d_dev->tasks_free_hwfc);
	INIT_LIST_HEAD(&g2d_dev->tasks_prepared);
	INIT_LIST_HEAD(&g2d_dev->tasks_active);
	INIT_LIST_HEAD(&g2d_dev->qos_contexts);

	mutex_init(&g2d_dev->lock_qos);

	ret = g2d_create_tasks(g2d_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to create tasks");
		goto err_task;
	}

	init_waitqueue_head(&g2d_dev->freeze_wait);

	g2d_dev->pm_notifier.notifier_call = &g2d_notifier_event;
	ret = register_pm_notifier(&g2d_dev->pm_notifier);
	if (ret)
		goto err_pm;

	spin_lock_init(&g2d_dev->fence_lock);
	g2d_dev->fence_context = dma_fence_context_alloc(1);

	dev_info(&pdev->dev, "Probed FIMG2D version %#010x\n", version);

	g2d_init_debug(g2d_dev);

	return 0;
err_pm:
	g2d_destroy_tasks(g2d_dev);
err_task:
	misc_deregister(&g2d_dev->misc);
err:
	pm_runtime_disable(&pdev->dev);
	iovmm_deactivate(g2d_dev->dev);

	dev_err(&pdev->dev, "Failed to probe FIMG2D\n");

	return ret;
}

static void g2d_shutdown(struct platform_device *pdev)
{
	struct g2d_device *g2d_dev = platform_get_drvdata(pdev);

	g2d_stamp_task(NULL, G2D_STAMP_STATE_SHUTDOWN_S);
	g2d_prepare_suspend(g2d_dev);

	wait_event(g2d_dev->freeze_wait, list_empty(&g2d_dev->tasks_active));

	if (test_and_set_bit(G2D_DEVICE_STATE_IOVMM_DISABLED, &g2d_dev->state))
		iovmm_deactivate(g2d_dev->dev);

	g2d_stamp_task(NULL, G2D_STAMP_STATE_SHUTDOWN_E);
}

static int g2d_remove(struct platform_device *pdev)
{
	struct g2d_device *g2d_dev = platform_get_drvdata(pdev);

	g2d_destroy_debug(g2d_dev);

	g2d_shutdown(pdev);

	g2d_destroy_tasks(g2d_dev);

	misc_deregister(&g2d_dev->misc);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int g2d_runtime_resume(struct device *dev)
{
	g2d_stamp_task(NULL, G2D_STAMP_STATE_PM_RESUME);

	return 0;
}

static int g2d_runtime_suspend(struct device *dev)
{
	struct g2d_device *g2d_dev = dev_get_drvdata(dev);

	clk_unprepare(g2d_dev->clock);
	g2d_stamp_task(NULL, G2D_STAMP_STATE_PM_SUSPEND);

	return 0;
}
#endif

static const struct dev_pm_ops g2d_pm_ops = {
	SET_RUNTIME_PM_OPS(NULL, g2d_runtime_resume, g2d_runtime_suspend)
};

static const struct of_device_id of_g2d_match[] = {
	{
		.compatible = "samsung,exynos9810-g2d",
	},
	{},
};

static struct platform_driver g2d_driver = {
	.probe		= g2d_probe,
	.remove		= g2d_remove,
	.shutdown	= g2d_shutdown,
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &g2d_pm_ops,
		.of_match_table = of_match_ptr(of_g2d_match),
	}
};

module_platform_driver(g2d_driver);
