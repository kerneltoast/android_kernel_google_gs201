/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *
 * Samsung Graphics 2D driver
 */

#ifndef __EXYNOS_G2D_H__
#define __EXYNOS_G2D_H__

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
#include <media/exynos_repeater.h>
#endif

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
#include <soc/google/exynos_pm_qos.h>
#endif

struct g2d_task; /* defined in g2d_task.h */

enum g2d_priority {
	G2D_LOW_PRIORITY,
	G2D_MEDIUM_PRIORITY,
	G2D_DEFAULT_PRIORITY = G2D_MEDIUM_PRIORITY,
	G2D_HIGH_PRIORITY,
	G2D_HIGHEST_PRIORITY,
	G2D_PRIORITY_END
};

/*
 * G2D_DEVICE_STATE_SUSPEND should be treated under g2d_dev->lock_task held
 * because it should be consistent with the state of all tasks attached to
 * g2d_dev->tasks_active.
 */
#define G2D_DEVICE_STATE_SUSPEND	1

enum g2d_hw_ppc_rot {
	PPC_NO_ROTATE,
	PPC_ROTATE,
	PPC_ROT,
};

enum g2d_hw_ppc_fmt {
	PPC_RGB,
	PPC_YUV2P,
	PPC_SBWC,
	PPC_RGB_AFBC,
	PPC_YUV_AFBC,
	PPC_FMT,
};

enum g2d_hw_ppc_sc {
	PPC_SC_UP,
	PPC_NO_SCALE,
	PPC_SC_DOWN_1,
	PPC_SC_DOWN_4,
	PPC_SC_DOWN_9,
	PPC_SC_DOWN_16,
	PPC_SC,
};

enum g2d_hw_ppc {
	PPC_COLORFILL = PPC_FMT * PPC_ROT * PPC_SC,
	PPC_END,
};

struct g2d_dvfs_table {
	u32 lv;
	u32 freq;
};

struct g2d_qos {
	u64	rbw;
	u64	wbw;
	u32	devfreq;
};

/* Proved that G2D does not leak protected conents that it is processing. */
#define G2D_DEVICE_CAPS_SELF_PROTECTION		BIT(0)
/* Separate bitfield to select YCbCr Bitdepth at REG_COLORMODE[29:28] */
#define G2D_DEVICE_CAPS_YUV_BITDEPTH		BIT(1)
/* Supports HWFC */
#define G2D_DEVICE_CAPS_HWFC			BIT(2)
/* Support SBWC format */
#define G2D_DEVICE_CAPS_SBWC			BIT(3)
/* Support AFBC v1.2 */
#define G2D_DEVICE_CAPS_AFBC_V12		BIT(4)
/* Force address alignment of compressed formats to 64-byte */
#define G2D_DEVICE_CAPS_COMP_ALIGN_64		BIT(5)
/* G2D has poly-phase filter and nearest/bilinear are absent */
#define G2D_DEVICE_CAPS_POLYFILTER		BIT(6)
/* Support HDR conversion for HDR10 */
#define G2D_DEVICE_CAPS_HDR10			BIT(7)
/* Support HDR conversion for HDR10+ */
#define G2D_DEVICE_CAPS_HDR10PLUS		BIT(8)
/* Support SBWC Lossy format */
#define G2D_DEVICE_CAPS_SBWC_LOSSY		BIT(9)

struct g2d_device {
	unsigned long		state;
	unsigned long		caps;

	struct miscdevice	misc[2];
	struct device		*dev;
	struct clk		*clock;
	void __iomem		*reg;
	struct device		*noncoherent_dev;

	u64			fence_context;
	atomic_t		fence_timeline;
	spinlock_t		fence_lock; /* dma_fence->lock for fences created in this driver */

	spinlock_t		lock_ctx_list; /* lock for ctx_list of struct g2d_ctx */
	struct list_head	ctx_list;

	/* task management */
	spinlock_t		lock_task;
	struct g2d_task		*tasks;
	struct list_head	tasks_free;
	struct list_head	tasks_free_hwfc;
	struct list_head	tasks_prepared;
	struct list_head	tasks_active;
	struct kthread_worker	*completion_workq;
	struct kthread_worker	*schedule_workq;

	struct notifier_block	pm_notifier;
	wait_queue_head_t	freeze_wait;
	wait_queue_head_t	queued_wait;

	struct dentry *debug_root;

	atomic_t	prior_stats[G2D_PRIORITY_END];
	/*
	 * lock for updating qos_contexts
	 * and the state of performance requests to pm_qos and devfreq.
	 */
	struct mutex			lock_qos;
	struct list_head		qos_contexts;

	struct g2d_qos			qos;
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
	struct exynos_pm_qos_request	req;
#endif

	u32 hw_ppc[PPC_END];
	u32				max_layers;

	struct g2d_dvfs_table *dvfs_table;
	u32 dvfs_table_cnt;
	/* bitflags of available values in LAYERn_COLOR_MODE_REG[19:16] */
	unsigned short		fmts_src;
	/* bitflags of available values in DST_COLOR_MODE_REG[19:16] */
	unsigned short		fmts_dst;

	struct notifier_block	itmon_nb;

	u32 dvfs_int;
	u32 dvfs_mif;

	struct delayed_work dwork;
};

#define G2D_AUTHORITY_HIGHUSER 1

struct g2d_context {
	struct list_head	node;
	struct g2d_device	*g2d_dev;
#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
	struct shared_buffer_info *hwfc_info;
	struct mutex		lock_hwfc_info; /* lock for upating hwfc_info */
#endif
	u32 priority;
	int authority;
	struct task_struct	*owner;

	struct list_head qos_node;
	struct g2d_qos	ctxqos;
};

#if !IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
/* MAX_SHARED_BUF_NUM is defined in media/exynos_repeater.h if exists */
#define MAX_SHARED_BUF_NUM 0
#endif

#define IPPREFIX "[Exynos][G2D] "
#define perr(format, arg...) \
	pr_err(IPPREFIX format "\n", ##arg)

#define perrfn(format, arg...) \
	pr_err(IPPREFIX  "%s: " format "\n", __func__, ##arg)

#define perrdev(g2d, format, arg...) \
	dev_err((g2d)->dev, IPPREFIX format "\n", ##arg)

#define perrfndev(g2d, format, arg...) \
	dev_err((g2d)->dev, IPPREFIX  "%s: " format "\n", __func__, ##arg)

int g2d_device_run(struct g2d_device *g2d_dev, struct g2d_task *task);
void g2d_hw_timeout_handler(struct timer_list *arg);

#endif /* __EXYNOS_G2D_H__ */
