/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#ifndef _BIGO_PRIV_H_
#define _BIGO_PRIV_H_

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <soc/google/exynos_pm_qos.h>

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
#include <soc/google/exynos-itmon.h>
#endif

#include "uapi/linux/bigo.h"

#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)
#include <soc/google/pt.h>
#endif

#define AVG_CNT 30
#define PEAK_CNT 5
#define BUS_WIDTH 16
#define BO_MAX_PRIO 2

struct bufinfo {
	struct list_head list;
	struct dma_buf *dmabuf;
	struct sg_table *sgt;
	struct dma_buf_attachment *attachment;
	size_t size;
	off_t offset;
	int fd;
	dma_addr_t iova;
};

struct bigo_opp {
	struct list_head list;
	u32 freq_khz;
	u32 load_pps;
};

struct bigo_bw {
	struct list_head list;
	u32 load_pps;
	u32 rd_bw;
	u32 wr_bw;
	u32 pk_bw;
	u32 rd_bw_afbc;
	u32 wr_bw_afbc;
	u32 pk_bw_afbc;
	u32 rd_bw_enc;
	u32 wr_bw_enc;
	u32 pk_bw_enc;
};

struct power_manager {
	int bwindex;
	struct exynos_pm_qos_request qos_bigo;
	struct exynos_pm_qos_request qos_req_mif;
	struct list_head opps;
	struct list_head bw;
	u32 max_load;
};

struct slc_manager {
#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)
	struct pt_handle *pt_hnd;
	ptid_t pid;
#endif
	void __iomem *ssmt_pid_base;
	size_t size;
};

struct bigo_job {
	struct list_head list;
	void *regs;
	size_t regs_size;
	int status;
};

struct bigo_debugfs {
	struct dentry *root;
	u32 set_freq;
	u32 trigger_ssr;
	u32 timeout;
};

struct bigo_prio_array {
	spinlock_t lock;
	unsigned long bitmap;
	struct list_head queue[BO_MAX_PRIO];
};

struct bigo_core {
	struct class *_class;
	struct cdev cdev;
	struct device *svc_dev;
	struct device *dev;
	dev_t devno;
	/* mutex protecting this data structure */
	struct mutex lock;
	void __iomem *base;
	int irq;
	struct completion frame_done;
	struct list_head instances;
	struct ion_client *mem_client;
	u32 stat_with_irq;
	struct power_manager pm;
	struct slc_manager slc;
	unsigned int regs_size;
	struct bigo_inst *curr_inst;
	phys_addr_t paddr;
	struct bigo_debugfs debugfs;
	spinlock_t status_lock;
	struct task_struct *worker_thread;
	wait_queue_head_t worker;
	struct bigo_prio_array prioq;
	u32 qos_dirty;
#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	struct notifier_block itmon_nb;
#endif
	u32 ip_ver;
};

struct bigo_inst {
	struct list_head list;
	struct list_head buffers;
	/* mutex protecting this data structure */
	struct mutex lock;
	struct kref refcount;
	struct bigo_core *core;
	u32 height;
	u32 width;
	u32 fps;
	u32 is_secure;
	int priority;
	struct bigo_bw avg_bw[AVG_CNT];
	struct bigo_bw pk_bw[AVG_CNT];
	int job_cnt;
	u32 hw_cycles[AVG_CNT];
	struct completion job_comp;
	struct bigo_job job;
	/* bytes per pixel */
	u32 bpp;
	bool idle;
        bool is_decoder_usage;
        bool afbc;
};

inline void set_curr_inst(struct bigo_core *core, struct bigo_inst *inst);
inline struct bigo_inst *get_curr_inst(struct bigo_core *core);

#endif //_BIGO_PRIV_H_
