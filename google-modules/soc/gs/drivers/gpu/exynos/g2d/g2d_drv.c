// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 *
 * Contact: Hyesoo Yu <hyesoo.yu@samsung.com>
 */

#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/sched/task.h>
#include <linux/dma-map-ops.h>
#include <soc/google/exynos-itmon.h>

#include "g2d.h"
#include "g2d_regs.h"
#include "g2d_task.h"
#include "g2d_uapi_process.h"
#include "g2d_debug.h"
#include "g2d_perf.h"

#define CREATE_TRACE_POINTS
#include "g2d_trace.h"

#define MODULE_NAME "exynos-g2d"

static int g2d_update_priority(struct g2d_context *ctx, enum g2d_priority priority)
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

	for (task = g2d_dev->tasks; task; task = task->next) {
		if (!is_task_state_idle(task) && task->sec.priority < priority) {
			spin_unlock_irqrestore(&g2d_dev->lock_task, flags);
			return -EBUSY;
		}
	}

	spin_unlock_irqrestore(&g2d_dev->lock_task, flags);

	return 0;
}

void g2d_hw_timeout_handler(struct timer_list *arg)
{
	struct g2d_task *task = from_timer(task, arg, hw_timer);
	struct g2d_device *g2d_dev = task->g2d_dev;
	unsigned long flags;
	u32 state;
	bool ret;

	spin_lock_irqsave(&g2d_dev->lock_task, flags);

	state = g2d_hw_get_job_state(g2d_dev, g2d_task_id(task));

	g2d_stamp_task(task, G2D_STAMP_STATE_TIMEOUT_HW, state);

	perrfndev(g2d_dev, "Time is up: %d msec for job %u %lu %u",
		  G2D_HW_TIMEOUT_MSEC, g2d_task_id(task), task->state, state);

	/*
	 * The task timed out is not currently running in H/W.
	 * It might be just finished by interrupt.
	 * Or, it will be processed in the interrupt handler.
	 */
	if (!is_task_state_active(task) || state == G2D_JOB_STATE_DONE) {
		spin_unlock_irqrestore(&g2d_dev->lock_task, flags);

		return;
	}

	g2d_dump_info(g2d_dev, task);

	ret = g2d_hw_global_reset(g2d_dev);

	perrdev(g2d_dev, "GLOBAL RESET %s : H/W timeout",
		ret ? "SUCCESS" : "FAIL");

	g2d_finish_tasks(g2d_dev, 0, false);

	spin_unlock_irqrestore(&g2d_dev->lock_task, flags);

	wake_up(&g2d_dev->freeze_wait);
}

#if !IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
static inline int hwfc_set_valid_buffer(int buf_idx, int capture_idx)
{
	return 0;
}
#endif

int g2d_device_run(struct g2d_device *g2d_dev, struct g2d_task *task)
{
	g2d_hw_push_task(g2d_dev, task);

	task->ktime_end = ktime_get();

	/* record the time between user request and H/W push */
	g2d_stamp_task(task, G2D_STAMP_STATE_PUSH,
		       (int)ktime_us_delta(task->ktime_end, task->ktime_begin));

	task->ktime_begin = ktime_get();

	if (IS_HWFC(task->flags))
		hwfc_set_valid_buffer(g2d_task_id(task), g2d_task_id(task));

	return 0;
}

static irqreturn_t g2d_irq_handler(int irq, void *priv)
{
	struct g2d_device *g2d_dev = priv;
	u32 intflags, errstatus;

	G2D_ATRACE_BEGIN("g2d_irq");
	spin_lock(&g2d_dev->lock_task);

	intflags = g2d_hw_finished_job_ids(g2d_dev);
	g2d_hw_clear_job_ids(g2d_dev, intflags);

	g2d_stamp_task(NULL, G2D_STAMP_STATE_INT, intflags);

	errstatus = g2d_hw_errint_status(g2d_dev);
	if (errstatus != 0) {
		int job_id = g2d_hw_get_current_task(g2d_dev);
		struct g2d_task *task = g2d_get_active_task_from_id(g2d_dev, job_id);
		bool ret;

		if (job_id < 0)
			perrdev(g2d_dev, "No task is running in HW");
		else if (!task)
			perrfndev(g2d_dev, "Current job %d in HW is not active", job_id);
		else
			perrfndev(g2d_dev, "Error occurred during running job %d", job_id);

		g2d_stamp_task(task, G2D_STAMP_STATE_ERR_INT, errstatus);

		g2d_dump_info(g2d_dev, task);

		ret = g2d_hw_global_reset(g2d_dev);

		perrdev(g2d_dev, "GLOBAL RESET %s : error interrupt",
			ret ? "SUCCESS" : "FAIL");
	}

	g2d_finish_tasks(g2d_dev, intflags, !errstatus);

	spin_unlock(&g2d_dev->lock_task);
	G2D_ATRACE_END();

	wake_up(&g2d_dev->freeze_wait);

	return IRQ_HANDLED;
}

static int g2d_fault_handler(struct iommu_fault *fault, void *data)
{
	struct g2d_device *g2d_dev = data;
	struct g2d_task *task;
	int job_id = g2d_hw_get_current_task(g2d_dev);
	unsigned long flags;

	spin_lock_irqsave(&g2d_dev->lock_task, flags);
	task = g2d_get_active_task_from_id(g2d_dev, job_id);
	spin_unlock_irqrestore(&g2d_dev->lock_task, flags);

	g2d_dump_info(g2d_dev, task);

	g2d_stamp_task(task, G2D_STAMP_STATE_MMUFAULT, 0);

	return 0;
}

static __u32 get_hw_version(struct g2d_device *g2d_dev, __u32 *version)
{
	int ret;

	ret = pm_runtime_get_sync(g2d_dev->dev);
	if (ret < 0) {
		perrdev(g2d_dev, "Failed to enable power (%d)", ret);
		return ret;
	}

	ret = clk_enable(g2d_dev->clock);
	if (ret < 0) {
		perrdev(g2d_dev, "Failed to enable clock (%d)", ret);
	} else {
		*version = readl_relaxed(g2d_dev->reg + G2D_VERSION_INFO_REG);
		clk_disable(g2d_dev->clock);
	}

	pm_runtime_put(g2d_dev->dev);

	return ret;
}

static void g2d_timeout_perf_work(struct work_struct *work)
{
	struct g2d_device *g2d_dev = container_of(work, struct g2d_device,
						  dwork.work);
	struct g2d_context *ctx, *tmp;

	mutex_lock(&g2d_dev->lock_qos);

	list_for_each_entry_safe(ctx, tmp, &g2d_dev->qos_contexts, qos_node)
		list_del_init(&ctx->qos_node);
	g2d_update_performance(g2d_dev);

	mutex_unlock(&g2d_dev->lock_qos);
}

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
static int g2d_init_hwfc_info(struct g2d_context *ctx)
{
	mutex_lock(&ctx->lock_hwfc_info);

	if (!ctx->hwfc_info) {
		int ret;

		ctx->hwfc_info = kzalloc(sizeof(*ctx->hwfc_info), GFP_KERNEL);
		if (!ctx->hwfc_info) {
			mutex_unlock(&ctx->lock_hwfc_info);
			return -ENOMEM;
		}

		ret = hwfc_request_buffer(ctx->hwfc_info, 0);
		if (!ret && ctx->hwfc_info->buffer_count > MAX_SHARED_BUF_NUM) {
			perrfndev(ctx->g2d_dev, "Invalid HWFC buffer count %d (max %d)",
				  ctx->hwfc_info->buffer_count, MAX_SHARED_BUF_NUM);
			ret = -EINVAL;
		}
		if (ret) {
			kfree(ctx->hwfc_info);
			ctx->hwfc_info = NULL;
			mutex_unlock(&ctx->lock_hwfc_info);
			perrfndev(ctx->g2d_dev, "Failed to get hwfc info");
			return ret;
		}
	}

	mutex_unlock(&ctx->lock_hwfc_info);

	return 0;
}

static void g2d_release_hwfc_info(struct g2d_context *g2d_ctx)
{
	if (g2d_ctx->hwfc_info) {
		int i;

		for (i = 0; i < g2d_ctx->hwfc_info->buffer_count; i++)
			dma_buf_put(g2d_ctx->hwfc_info->bufs[i]);
		kfree(g2d_ctx->hwfc_info);
	}
}
#else
static inline int g2d_init_hwfc_info(struct g2d_context *ctx)
{
	perrdev(ctx->g2d_dev, "HWFC is not supported");
	return -ENOTSUPP;
}

static inline void g2d_release_hwfc_info(struct g2d_context *g2d_ctx)
{
}

#endif

static int g2d_open(struct inode *inode, struct file *filp)
{
	struct g2d_device *g2d_dev;
	struct g2d_context *g2d_ctx;
	struct miscdevice *misc = filp->private_data;

	g2d_ctx = kzalloc(sizeof(*g2d_ctx), GFP_KERNEL);
	if (!g2d_ctx)
		return -ENOMEM;

	if (!strcmp(misc->name, "g2d")) {
		g2d_dev = container_of(misc, struct g2d_device, misc[0]);
		g2d_ctx->authority = G2D_AUTHORITY_HIGHUSER;
	} else {
		g2d_dev = container_of(misc, struct g2d_device, misc[1]);
	}

	filp->private_data = g2d_ctx;

	g2d_ctx->g2d_dev = g2d_dev;

	g2d_ctx->priority = G2D_DEFAULT_PRIORITY;
	atomic_inc(&g2d_dev->prior_stats[g2d_ctx->priority]);

	get_task_struct(current->group_leader);
	g2d_ctx->owner = current->group_leader;

	spin_lock(&g2d_dev->lock_ctx_list);
	list_add(&g2d_ctx->node, &g2d_dev->ctx_list);
	spin_unlock(&g2d_dev->lock_ctx_list);

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
	mutex_init(&g2d_ctx->lock_hwfc_info);
#endif

	INIT_LIST_HEAD(&g2d_ctx->qos_node);

	return 0;
}

static int g2d_release(struct inode *inode, struct file *filp)
{
	struct g2d_context *g2d_ctx = filp->private_data;
	struct g2d_device *g2d_dev = g2d_ctx->g2d_dev;

	atomic_dec(&g2d_dev->prior_stats[g2d_ctx->priority]);

	g2d_release_hwfc_info(g2d_ctx);

	mutex_lock(&g2d_dev->lock_qos);

	list_del_init(&g2d_ctx->qos_node);
	g2d_update_performance(g2d_dev);

	mutex_unlock(&g2d_dev->lock_qos);

	spin_lock(&g2d_dev->lock_ctx_list);
	list_del(&g2d_ctx->node);
	spin_unlock(&g2d_dev->lock_ctx_list);

	put_task_struct(g2d_ctx->owner);

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

		if (ret) {
			if (ctx->authority == G2D_AUTHORITY_HIGHUSER)
				perrfndev(g2d_dev, "prio %d/%d found higher than %d", i,
					  atomic_read(&g2d_dev->prior_stats[i]),
					  ctx->priority);
			break;
		}

		if (copy_from_user(&data, uptr, sizeof(data))) {
			perrfndev(g2d_dev, "Failed to read g2d_task_data");
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
		if (IS_HWFC(data.flags)) {
			if (ctx->authority != G2D_AUTHORITY_HIGHUSER) {
				ret = -EPERM;
				break;
			}

			ret = g2d_init_hwfc_info(ctx);
			if (ret)
				break;
		}

		task = g2d_get_free_task(g2d_dev, ctx, IS_HWFC(data.flags));
		if (IS_ERR(task)) {
			ret = PTR_ERR(task);
			break;
		}

		kref_init(&task->starter);

		G2D_ATRACE_BEGIN("g2d_get_userdata");
		ret = g2d_get_userdata(g2d_dev, ctx, task, &data);
		G2D_ATRACE_END();
		if (ret < 0) {
			/* release hwfc buffer */
			if (IS_HWFC(task->flags) && task->bufidx >= 0)
				hwfc_set_valid_buffer(task->bufidx, -1);
			g2d_put_free_task(g2d_dev, task);
			break;
		}

		g2d_stamp_task(task, G2D_STAMP_STATE_BEGIN,
			       atomic_read(&task->starter.refcount.refs));

		G2D_ATRACE_BEGIN("g2d_start_task");
		g2d_start_task(task);
		G2D_ATRACE_END();

		if (!(task->flags & G2D_FLAG_NONBLOCK))
			ret = g2d_wait_put_user(g2d_dev, task,
						uptr, data.flags);

		break;
	}
	case G2D_IOC_PRIORITY:
	{
		enum g2d_priority data;

		if (!(ctx->authority & G2D_AUTHORITY_HIGHUSER)) {
			ret = -EPERM;
			break;
		}

		if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
			perrfndev(g2d_dev, "Failed to get priority");
			ret = -EFAULT;
			break;
		}

		if (data < G2D_LOW_PRIORITY || data >= G2D_PRIORITY_END) {
			perrfndev(g2d_dev, "Wrong priority %u", data);
			ret = -EINVAL;
			break;
		}

		ret = g2d_update_priority(ctx, data);

		break;
	}
	case G2D_IOC_PERFORMANCE:
	{
		struct g2d_performance_data data;
		int i;
		struct g2d_qos qos = {0, };

		if (!(ctx->authority & G2D_AUTHORITY_HIGHUSER)) {
			ret = -EPERM;
			break;
		}

		if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
			perrfndev(g2d_dev, "Failed to read perf data");
			ret = -EFAULT;
			break;
		}

		if (data.num_frame > G2D_PERF_MAX_FRAMES)
			return -EINVAL;

		for (i = 0; i < data.num_frame; i++) {
			if (data.frame[i].num_layers > g2d_dev->max_layers)
				return -EINVAL;
		}

		for (i = 0; i < data.num_frame; i++) {
			qos.rbw += data.frame[i].bandwidth_read;
			qos.wbw += data.frame[i].bandwidth_write;
		}
		qos.devfreq = g2d_calc_device_frequency(g2d_dev, &data);

		ctx->ctxqos = qos;

		mutex_lock(&g2d_dev->lock_qos);
		if (!qos.rbw && !qos.wbw && !qos.devfreq)
			list_del_init(&ctx->qos_node);
		else if (list_empty(&ctx->qos_node))
			list_add(&ctx->qos_node, &g2d_dev->qos_contexts);
		g2d_update_performance(g2d_dev);
		mutex_unlock(&g2d_dev->lock_qos);

		break;
	}
	case G2D_IOC_VERSION:
	{
		u32 version = 0x1;

		if (copy_to_user((void __user *)arg, &version, _IOC_SIZE(cmd)))
			return -EFAULT;

		break;
	}
	default:
	{
		perrfn("unknown ioctl %#x", cmd);
		return -ENOTTY;
	}
	}

	return ret;
}

static const struct file_operations g2d_fops = {
	.owner          = THIS_MODULE,
	.open           = g2d_open,
	.release        = g2d_release,
	.unlocked_ioctl	= g2d_ioctl,
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

static unsigned int g2d_default_ppc[] = {
	/* sc_up none x1 x1/4 x1/9 x1/16 */
	3800, 3200, 2100, 2600, 3300, 3600, //rgb32 non-rotated
	3500, 3600, 2100, 2700, 3300, 3800, //rgb32 rotated
	3700, 3500, 3600, 4300, 4300, 3500, //yuv2p non-rotated
	2600, 2800, 3200, 3900, 4300, 3600, // yuv2p rotated
	3600, 2600, 1400, 900, 1000, 1000, // sbwc non-rotated
	2600, 2600, 1400, 900, 1000, 1000, // sbwc rotated
	1600, 3400, 300, 500, 800, 700, //rgb afbc non-rotated
	1400, 3600, 300, 800, 900, 900, //rgb afbc rotated
	2900, 2000, 800, 300, 300, 400, //yuv2p afbc non-rotated
	2600, 2000, 800, 300, 300, 400, //yuv2p afbc rotated
	2800, //colorfill
};

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
	int len;

	if (of_property_read_u32_array(dev->of_node, "hw_ppc", (u32 *)g2d_dev->hw_ppc,
				       (size_t)(ARRAY_SIZE(g2d_dev->hw_ppc)))) {
		perrdev(g2d_dev, "Failed to parse device tree for hw ppc");
		memcpy(g2d_dev->hw_ppc, g2d_default_ppc, sizeof(g2d_default_ppc[0]) * PPC_END);
	}

	len = of_property_count_u32_elems(dev->of_node, "g2d_dvfs_table");
	if (len <= 0)
		g2d_dev->dvfs_table_cnt = ARRAY_SIZE(g2d_default_dvfs_table);
	else
		g2d_dev->dvfs_table_cnt = len / 2;

	g2d_dev->dvfs_table = devm_kcalloc(dev, g2d_dev->dvfs_table_cnt,
					   sizeof(struct g2d_dvfs_table), GFP_KERNEL);
	if (!g2d_dev->dvfs_table)
		return -ENOMEM;

	if (len < 0)
		memcpy(g2d_dev->dvfs_table, g2d_default_dvfs_table,
		       sizeof(struct g2d_dvfs_table) * g2d_dev->dvfs_table_cnt);
	else
		of_property_read_u32_array(dev->of_node, "g2d_dvfs_table",
					   (unsigned int *)g2d_dev->dvfs_table, len);

	if (of_property_read_u32(dev->of_node, "dvfs_int", &g2d_dev->dvfs_int))
		g2d_dev->dvfs_int = 0;

	if (of_property_read_u32(dev->of_node, "dvfs_mif", &g2d_dev->dvfs_mif))
		g2d_dev->dvfs_mif = 0;

	return 0;
}

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)

#define MAX_ITMON_STRATTR 4

bool g2d_itmon_check(struct g2d_device *g2d_dev, char *str_itmon, char *str_attr)
{
	const char *name[MAX_ITMON_STRATTR];
	int size, i;

	if (!str_itmon)
		return false;

	size = of_property_count_strings(g2d_dev->dev->of_node, str_attr);
	if (size < 0 || size > MAX_ITMON_STRATTR)
		return false;

	of_property_read_string_array(g2d_dev->dev->of_node,
				      str_attr, name, size);
	for (i = 0; i < size; i++) {
		if (strncmp(str_itmon, name[i], strlen(name[i])) == 0)
			return true;
	}

	return false;
}

int g2d_itmon_notifier(struct notifier_block *nb, unsigned long action, void *nb_data)
{
	struct g2d_device *g2d_dev = container_of(nb, struct g2d_device, itmon_nb);
	struct itmon_notifier *itmon_info = nb_data;
	struct g2d_task *task;
	static int called_count;
	bool is_power_on = false, is_g2d_itmon = true;

	if (g2d_itmon_check(g2d_dev, itmon_info->port, "itmon,port"))
		is_power_on = true;
	else if (g2d_itmon_check(g2d_dev, itmon_info->dest, "itmon,dest"))
		is_power_on = (itmon_info->onoff) ? true : false;
	else
		is_g2d_itmon = false;

	if (is_g2d_itmon) {
		unsigned long flags;
		int job_id;

		if (called_count++ != 0) {
			perrfndev(g2d_dev,
				  "called %d times, ignore it.", called_count);
			return NOTIFY_DONE;
		}

		g2d_show_task_status(g2d_dev);

		if (!is_power_on)
			return NOTIFY_DONE;

		job_id = g2d_hw_get_current_task(g2d_dev);

		spin_lock_irqsave(&g2d_dev->lock_task, flags);

		task = g2d_get_active_task_from_id(g2d_dev, job_id);

		spin_unlock_irqrestore(&g2d_dev->lock_task, flags);

		g2d_dump_info(g2d_dev, task);
	}

	return NOTIFY_DONE;
}
#endif

struct g2d_device_data {
	unsigned long caps;
	unsigned int max_layers;
	unsigned short fmts_src;
	unsigned short fmts_dst;
};

const struct g2d_device_data g2d_9610_data = {
	.max_layers = G2D_MAX_IMAGES_HALF,
};

const struct g2d_device_data g2d_9810_data = {
	.caps = G2D_DEVICE_CAPS_HDR10,
	.max_layers = G2D_MAX_IMAGES,
};

const struct g2d_device_data g2d_9820_data = {
	.caps = G2D_DEVICE_CAPS_SELF_PROTECTION | G2D_DEVICE_CAPS_YUV_BITDEPTH |
		G2D_DEVICE_CAPS_HWFC | G2D_DEVICE_CAPS_HDR10,
	.max_layers = G2D_MAX_IMAGES,
};

const struct g2d_device_data g2d_gs101_data = {
	.caps = G2D_DEVICE_CAPS_SELF_PROTECTION | G2D_DEVICE_CAPS_YUV_BITDEPTH |
		G2D_DEVICE_CAPS_SBWC | G2D_DEVICE_CAPS_AFBC_V12 |
		G2D_DEVICE_CAPS_POLYFILTER | G2D_DEVICE_CAPS_HDR10PLUS,
	.max_layers = G2D_MAX_IMAGES_QUARTER,
	.fmts_src = BIT(G2D_FMT_IDX_8888) | BIT(G2D_FMT_IDX_565) |
		    BIT(G2D_FMT_IDX_4444) | BIT(G2D_FMT_IDX_888) |
		    BIT(G2D_FMT_IDX_1555) | BIT(G2D_FMT_IDX_5551) |
		    BIT(G2D_FMT_IDX_YUV420SP) | BIT(G2D_FMT_IDX_YUV420P) |
		    BIT(G2D_FMT_IDX_YUV422SP) | BIT(G2D_FMT_IDX_2101010) |
		    BIT(G2D_FMT_IDX_1010102),
	.fmts_dst = BIT(G2D_FMT_IDX_8888) | BIT(G2D_FMT_IDX_565) |
		    BIT(G2D_FMT_IDX_4444) | BIT(G2D_FMT_IDX_888) |
		    BIT(G2D_FMT_IDX_1555) | BIT(G2D_FMT_IDX_5551) |
		    BIT(G2D_FMT_IDX_YUV420SP) | BIT(G2D_FMT_IDX_YUV420P) |
		    BIT(G2D_FMT_IDX_YUV422SP) | BIT(G2D_FMT_IDX_2101010) |
		    BIT(G2D_FMT_IDX_1010102),
};

static const struct g2d_device_data g2d_gs201_data = {
	.caps = G2D_DEVICE_CAPS_SELF_PROTECTION | G2D_DEVICE_CAPS_YUV_BITDEPTH |
		G2D_DEVICE_CAPS_SBWC | G2D_DEVICE_CAPS_AFBC_V12 |
		G2D_DEVICE_CAPS_POLYFILTER | G2D_DEVICE_CAPS_HDR10PLUS | G2D_DEVICE_CAPS_SBWC_LOSSY,
	.max_layers = G2D_MAX_IMAGES_QUARTER,
	.fmts_src = BIT(G2D_FMT_IDX_8888) | BIT(G2D_FMT_IDX_565) |
		    BIT(G2D_FMT_IDX_4444) | BIT(G2D_FMT_IDX_888) |
		    BIT(G2D_FMT_IDX_1555) | BIT(G2D_FMT_IDX_5551) |
		    BIT(G2D_FMT_IDX_YUV420SP) | BIT(G2D_FMT_IDX_YUV420P) |
		    BIT(G2D_FMT_IDX_YUV422SP) | BIT(G2D_FMT_IDX_2101010) |
		    BIT(G2D_FMT_IDX_1010102),
	.fmts_dst = BIT(G2D_FMT_IDX_8888) | BIT(G2D_FMT_IDX_565) |
		    BIT(G2D_FMT_IDX_4444) | BIT(G2D_FMT_IDX_888) |
		    BIT(G2D_FMT_IDX_1555) | BIT(G2D_FMT_IDX_5551) |
		    BIT(G2D_FMT_IDX_YUV420SP) | BIT(G2D_FMT_IDX_YUV420P) |
		    BIT(G2D_FMT_IDX_YUV422SP) | BIT(G2D_FMT_IDX_2101010) |
		    BIT(G2D_FMT_IDX_1010102),
};

const struct g2d_device_data g2d_zuma_data = {
	.caps = G2D_DEVICE_CAPS_SELF_PROTECTION | G2D_DEVICE_CAPS_YUV_BITDEPTH |
		G2D_DEVICE_CAPS_SBWC | G2D_DEVICE_CAPS_AFBC_V12 |
		G2D_DEVICE_CAPS_POLYFILTER | G2D_DEVICE_CAPS_HDR10PLUS | G2D_DEVICE_CAPS_SBWC_LOSSY,
	.max_layers = G2D_MAX_LAYERS,
	.fmts_src = BIT(G2D_FMT_IDX_8888) | BIT(G2D_FMT_IDX_565) |
		    BIT(G2D_FMT_IDX_4444) | BIT(G2D_FMT_IDX_888) |
		    BIT(G2D_FMT_IDX_1555) | BIT(G2D_FMT_IDX_5551) |
		    BIT(G2D_FMT_IDX_YUV420SP) | BIT(G2D_FMT_IDX_YUV420P) |
		    BIT(G2D_FMT_IDX_YUV422SP) | BIT(G2D_FMT_IDX_2101010) |
		    BIT(G2D_FMT_IDX_1010102),
	.fmts_dst = BIT(G2D_FMT_IDX_8888) | BIT(G2D_FMT_IDX_565) |
		    BIT(G2D_FMT_IDX_4444) | BIT(G2D_FMT_IDX_888) |
		    BIT(G2D_FMT_IDX_1555) | BIT(G2D_FMT_IDX_5551) |
		    BIT(G2D_FMT_IDX_YUV420SP) | BIT(G2D_FMT_IDX_YUV420P) |
		    BIT(G2D_FMT_IDX_YUV422SP) | BIT(G2D_FMT_IDX_2101010) |
		    BIT(G2D_FMT_IDX_1010102),
};

static const struct of_device_id of_g2d_match[] __refconst = {
	{
		.compatible = "samsung,exynos9810-g2d",
		.data = &g2d_9810_data,
	}, {
		.compatible = "samsung,exynos9610-g2d",
		.data = &g2d_9610_data,
	}, {
		.compatible = "samsung,exynos9820-g2d",
		.data = &g2d_9820_data,
	}, {
		.compatible = "samsung,gs101-g2d",
		.data = &g2d_gs101_data,
	}, {
		.compatible = "samsung,gs201-g2d",
		.data = &g2d_gs201_data,
	}, {
		.compatible = "samsung,zuma-g2d",
		.data = &g2d_zuma_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, of_g2d_match);

static int g2d_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	struct g2d_device *g2d_dev;
	struct resource *res;
	__u32 version;
	int ret;
	int irq;

	g2d_dev = devm_kzalloc(&pdev->dev, sizeof(*g2d_dev), GFP_KERNEL);
	if (!g2d_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, g2d_dev);
	g2d_dev->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	g2d_dev->reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(g2d_dev->reg))
		return PTR_ERR(g2d_dev->reg);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		perrdev(g2d_dev, "Failed to get IRQ");
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq,
			       g2d_irq_handler, 0, pdev->name, g2d_dev);
	if (ret) {
		perrdev(g2d_dev, "Failed to install IRQ handler");
		return ret;
	}

	dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));

	g2d_dev->clock = devm_clk_get(&pdev->dev, "gate");
	if (PTR_ERR(g2d_dev->clock) == -ENOENT) {
		perrdev(g2d_dev, "'gate' not found. Ignoring clock gating..\n");
		g2d_dev->clock = NULL;
	} else if (IS_ERR(g2d_dev->clock)) {
		ret = PTR_ERR(g2d_dev->clock);
		perrdev(g2d_dev, "Failed to get clock (%d)", ret);
		return ret;
	}

	/* it is okay if fault handler is not registered since it is just for debugging */
	ret = iommu_register_device_fault_handler(&pdev->dev, g2d_fault_handler, g2d_dev);
	if (ret)
		perrdev(g2d_dev, "Failed to register IOMMU fault handler (%d)", ret);

	ret = g2d_parse_dt(g2d_dev);
	if (ret < 0)
		goto err_dt;

	of_id = of_match_node(of_g2d_match, pdev->dev.of_node);
	if (of_id->data) {
		const struct g2d_device_data *devdata = of_id->data;

		g2d_dev->caps = devdata->caps;
		g2d_dev->max_layers = devdata->max_layers;
		g2d_dev->fmts_src = devdata->fmts_src;
		g2d_dev->fmts_dst = devdata->fmts_dst;
	}

	if (!(g2d_dev->caps & G2D_DEVICE_CAPS_SELF_PROTECTION)) {
		if (!dev_is_dma_coherent(&pdev->dev)) {
			g2d_dev->noncoherent_dev = &pdev->dev;
		} else {
			g2d_dev->noncoherent_dev = devm_kzalloc(&pdev->dev, sizeof(struct device),
								GFP_KERNEL);
			if (!g2d_dev->noncoherent_dev)
				return -ENOMEM;
			device_initialize(g2d_dev->noncoherent_dev);
		}
	}
	/* prepare clock and enable runtime pm */
	pm_runtime_enable(&pdev->dev);

	ret = get_hw_version(g2d_dev, &version);
	if (ret < 0)
		goto err;

	g2d_dev->misc[0].minor = MISC_DYNAMIC_MINOR;
	g2d_dev->misc[0].name = "g2d";
	g2d_dev->misc[0].fops = &g2d_fops;

	/* misc register */
	ret = misc_register(&g2d_dev->misc[0]);
	if (ret) {
		perrdev(g2d_dev, "Failed to register misc device for 0");
		goto err;
	}

	g2d_dev->misc[1].minor = MISC_DYNAMIC_MINOR;
	g2d_dev->misc[1].name = "fimg2d";
	g2d_dev->misc[1].fops = &g2d_fops;

	ret = misc_register(&g2d_dev->misc[1]);
	if (ret) {
		perrdev(g2d_dev, "Failed to register misc device for 1");
		goto err_misc;
	}

	spin_lock_init(&g2d_dev->lock_task);
	spin_lock_init(&g2d_dev->lock_ctx_list);

	INIT_LIST_HEAD(&g2d_dev->tasks_free);
	INIT_LIST_HEAD(&g2d_dev->tasks_free_hwfc);
	INIT_LIST_HEAD(&g2d_dev->tasks_prepared);
	INIT_LIST_HEAD(&g2d_dev->tasks_active);
	INIT_LIST_HEAD(&g2d_dev->qos_contexts);
	INIT_LIST_HEAD(&g2d_dev->ctx_list);

	mutex_init(&g2d_dev->lock_qos);

	ret = g2d_create_tasks(g2d_dev);
	if (ret < 0) {
		perrdev(g2d_dev, "Failed to create tasks");
		goto err_task;
	}

	init_waitqueue_head(&g2d_dev->freeze_wait);
	init_waitqueue_head(&g2d_dev->queued_wait);

	g2d_dev->pm_notifier.notifier_call = &g2d_notifier_event;
	ret = register_pm_notifier(&g2d_dev->pm_notifier);
	if (ret)
		goto err_pm;

	spin_lock_init(&g2d_dev->fence_lock);
	g2d_dev->fence_context = dma_fence_context_alloc(1);

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	g2d_dev->itmon_nb.notifier_call = g2d_itmon_notifier;
	itmon_notifier_chain_register(&g2d_dev->itmon_nb);
#endif
	dev_info(&pdev->dev, "Probed FIMG2D version %#010x", version);

	INIT_DELAYED_WORK(&g2d_dev->dwork, g2d_timeout_perf_work);

	g2d_init_debug(g2d_dev);

	return 0;
err_pm:
	g2d_destroy_tasks(g2d_dev);
err_task:
	misc_deregister(&g2d_dev->misc[1]);
err_misc:
	misc_deregister(&g2d_dev->misc[0]);
err:
	pm_runtime_disable(&pdev->dev);
err_dt:
	iommu_unregister_device_fault_handler(&pdev->dev);

	perrdev(g2d_dev, "Failed to probe FIMG2D");

	return ret;
}

static void g2d_shutdown(struct platform_device *pdev)
{
	struct g2d_device *g2d_dev = platform_get_drvdata(pdev);

	g2d_stamp_task(NULL, G2D_STAMP_STATE_SHUTDOWN, 0);
	g2d_prepare_suspend(g2d_dev);

	wait_event(g2d_dev->freeze_wait, list_empty(&g2d_dev->tasks_active));

	g2d_stamp_task(NULL, G2D_STAMP_STATE_SHUTDOWN, 1);
}

static int g2d_remove(struct platform_device *pdev)
{
	struct g2d_device *g2d_dev = platform_get_drvdata(pdev);

	g2d_destroy_debug(g2d_dev);

	g2d_shutdown(pdev);

	g2d_destroy_tasks(g2d_dev);

	misc_deregister(&g2d_dev->misc[0]);
	misc_deregister(&g2d_dev->misc[1]);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int g2d_runtime_resume(struct device *dev)
{
	struct g2d_device *g2d_dev = dev_get_drvdata(dev);

	clk_prepare(g2d_dev->clock);
	g2d_stamp_task(NULL, G2D_STAMP_STATE_RUNTIME_PM, 0);

	return 0;
}

static int g2d_runtime_suspend(struct device *dev)
{
	struct g2d_device *g2d_dev = dev_get_drvdata(dev);

	clk_unprepare(g2d_dev->clock);
	g2d_stamp_task(NULL, G2D_STAMP_STATE_RUNTIME_PM, 1);

	return 0;
}
#endif

static const struct dev_pm_ops g2d_pm_ops = {
	SET_RUNTIME_PM_OPS(NULL, g2d_runtime_resume, g2d_runtime_suspend)
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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cho KyongHo <pullip.cho@samsung.com>");
MODULE_AUTHOR("Hyesoo Yu <hyesoo.yu@samsung.com>");
MODULE_DESCRIPTION("Exynos FIMG2D device driver");
MODULE_IMPORT_NS(DMA_BUF);
