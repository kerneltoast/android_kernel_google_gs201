// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/sched/task.h>

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
#include <media/exynos_tsmux.h>
#endif

#include "g2d.h"
#include "g2d_task.h"
#include "g2d_uapi.h"
#include "g2d_debug.h"
#include "g2d_regs.h"
#include "g2d_perf.h"
#include "g2d_trace.h"

unsigned int g2d_debug;

#define G2D_MAX_STAMP_ID 1024
#define G2D_STAMP_CLAMP_ID(id) ((id) & (G2D_MAX_STAMP_ID - 1))

static struct g2d_stamp {
	ktime_t time;
	unsigned long state;
	unsigned short stamp;
	unsigned char job_id;
	unsigned char cpu;
	union {
		u32 val[3];
		struct {
			char name[8];
			unsigned int seqno;
		} fence;
	};
} g2d_stamp_list[G2D_MAX_STAMP_ID];

static atomic_t g2d_stamp_id = ATOMIC_INIT(-1);

enum {
	G2D_STAMPTYPE_NONE,
	G2D_STAMPTYPE_NUM,
	G2D_STAMPTYPE_HEX,
	G2D_STAMPTYPE_USEC,
	G2D_STAMPTYPE_PERF,
	G2D_STAMPTYPE_INOUT,
	G2D_STAMPTYPE_ALLOCFREE,
	G2D_STAMPTYPE_FENCE,
};

static struct g2d_stamp_type {
	const char *name;
	int type;
	bool task_specific;
} g2d_stamp_types[G2D_STAMP_STATE_NUM] = {
	{"runtime_pm",    G2D_STAMPTYPE_INOUT,     false},
	{"task_alloc",    G2D_STAMPTYPE_ALLOCFREE, true},
	{"task_begin",    G2D_STAMPTYPE_NUM,       true},
	{"task_push",     G2D_STAMPTYPE_PERF,      true},
	{"irq",           G2D_STAMPTYPE_HEX,       false},
	{"task_done",     G2D_STAMPTYPE_USEC,      true},
	{"fence_timeout", G2D_STAMPTYPE_HEX,       true},
	{"hw_timeout",    G2D_STAMPTYPE_HEX,       true},
	{"irq_error",     G2D_STAMPTYPE_HEX,       true},
	{"mmu_fault",     G2D_STAMPTYPE_NONE,      true},
	{"shutdown",      G2D_STAMPTYPE_INOUT,     false},
	{"suspend",       G2D_STAMPTYPE_INOUT,     false},
	{"resume",        G2D_STAMPTYPE_INOUT,     false},
	{"hwfc_job",      G2D_STAMPTYPE_NUM,       true},
	{"pending",       G2D_STAMPTYPE_NUM,       false},
	{"fence",         G2D_STAMPTYPE_FENCE,     true},
};

static bool g2d_stamp_show_single(struct seq_file *s, struct g2d_stamp *stamp)
{
	if (stamp->time == 0)
		return false;

	seq_printf(s, "[%u:%12lld] %13s: ", stamp->cpu,
		   ktime_to_us(stamp->time),
		   g2d_stamp_types[stamp->stamp].name);

	if (g2d_stamp_types[stamp->stamp].task_specific)
		seq_printf(s, "JOB ID %2u (STATE %#05lx) - ",
			   stamp->job_id, stamp->state);

	switch (g2d_stamp_types[stamp->stamp].type) {
	case G2D_STAMPTYPE_NUM:
		seq_printf(s, "%u", stamp->val[0]);
		break;
	case G2D_STAMPTYPE_HEX:
		seq_printf(s, "%#x", stamp->val[0]);
		break;
	case G2D_STAMPTYPE_USEC:
		seq_printf(s, "%u usec.", stamp->val[0]);
		break;
	case G2D_STAMPTYPE_PERF:
		seq_printf(s, "%u usec. (INT %u MIF %u)", stamp->val[0],
			   stamp->val[1], stamp->val[2]);
		break;
	case G2D_STAMPTYPE_INOUT:
		seq_printf(s, "%s", stamp->val[0] ? "out" : "in");
		break;
	case G2D_STAMPTYPE_ALLOCFREE:
		seq_printf(s, "%s", stamp->val[0] ? "free" : "alloc");
		break;
	case G2D_STAMPTYPE_FENCE:
		seq_printf(s, "%u@%s", stamp->fence.seqno, stamp->fence.name);
		break;
	}

	seq_puts(s, "\n");

	return true;
}

static int g2d_stamp_show(struct seq_file *s, void *unused)
{
	int idx = G2D_STAMP_CLAMP_ID(atomic_read(&g2d_stamp_id) + 1);
	int i;

	/* in chronological order */
	for (i = idx; i < G2D_MAX_STAMP_ID; i++)
		if (!g2d_stamp_show_single(s, &g2d_stamp_list[i]))
			break;

	for (i = 0; i < idx; i++)
		if (!g2d_stamp_show_single(s, &g2d_stamp_list[i]))
			break;

	return 0;
}

static int g2d_debug_logs_open(struct inode *inode, struct file *file)
{
	return single_open(file, g2d_stamp_show, inode->i_private);
}

static const struct file_operations g2d_debug_logs_fops = {
	.open = g2d_debug_logs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int g2d_debug_contexts_show(struct seq_file *s, void *unused)
{
	struct g2d_device *g2d_dev = s->private;
	struct g2d_context *ctx;

	seq_printf(s, "%16s %6s %4s %6s %10s %10s %10s\n",
		   "task", "pid", "prio", "dev", "rbw", "wbw", "devfreq");
	seq_puts(s, "------------------------------------------------------\n");

	spin_lock(&g2d_dev->lock_ctx_list);

	list_for_each_entry(ctx, &g2d_dev->ctx_list, node) {
		task_lock(ctx->owner);
		seq_printf(s, "%16s %6u %4d %6s %10llu %10llu %10u\n",
			   ctx->owner->comm, ctx->owner->pid, ctx->priority,
			   g2d_dev->misc[(ctx->authority + 1) & 1].name,
			   ctx->ctxqos.rbw, ctx->ctxqos.wbw,
			   ctx->ctxqos.devfreq);

		task_unlock(ctx->owner);
	}

	spin_unlock(&g2d_dev->lock_ctx_list);

	seq_puts(s, "------------------------------------------------------\n");

	seq_puts(s, "priorities:\n");
	seq_printf(s, "\tlow(0)    : %d\n",
		   atomic_read(&g2d_dev->prior_stats[G2D_LOW_PRIORITY]));
	seq_printf(s, "\tmedium(1) : %d\n",
		   atomic_read(&g2d_dev->prior_stats[G2D_MEDIUM_PRIORITY]));
	seq_printf(s, "\thigh(2)   : %d\n",
		   atomic_read(&g2d_dev->prior_stats[G2D_HIGH_PRIORITY]));
	seq_printf(s, "\thighest(3): %d\n",
		   atomic_read(&g2d_dev->prior_stats[G2D_HIGHEST_PRIORITY]));

	return 0;
}

static int g2d_debug_contexts_open(struct inode *inode, struct file *file)
{
	return single_open(file, g2d_debug_contexts_show, inode->i_private);
}

static const struct file_operations g2d_debug_contexts_fops = {
	.open = g2d_debug_contexts_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int g2d_debug_tasks_show(struct seq_file *s, void *unused)
{
	struct g2d_device *g2d_dev = s->private;
	struct g2d_task *task;

	for (task = g2d_dev->tasks; task; task = task->next) {
		seq_printf(s, "TASK[%d]: state %#lx flags %#x ",
			   g2d_task_id(task), task->state, task->flags);
		seq_printf(s, "prio %d begin@%llu end@%llu nr_src %d ",
			   task->sec.priority, ktime_to_us(task->ktime_begin),
			   ktime_to_us(task->ktime_end), task->num_source);
		seq_printf(s, "rbw %llu wbw %llu devfreq %u\n",
			   task->taskqos.rbw, task->taskqos.wbw,
			   task->taskqos.devfreq);
	}

	return 0;
}

static int g2d_debug_tasks_open(struct inode *inode, struct file *file)
{
	return single_open(file, g2d_debug_tasks_show, inode->i_private);
}

static const struct file_operations g2d_debug_tasks_fops = {
	.open = g2d_debug_tasks_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void g2d_init_debug(struct g2d_device *g2d_dev)
{
	g2d_dev->debug_root = debugfs_create_dir("g2d", NULL);
	if (!g2d_dev->debug_root) {
		perrdev(g2d_dev, "debugfs: failed to create root directory");
		return;
	}

	debugfs_create_u32("debug", 0644, g2d_dev->debug_root, &g2d_debug);

	debugfs_create_file("logs", 0444, g2d_dev->debug_root,
			    g2d_dev, &g2d_debug_logs_fops);

	debugfs_create_file("contexts", 0400, g2d_dev->debug_root,
			    g2d_dev, &g2d_debug_contexts_fops);

	debugfs_create_file("tasks", 0400, g2d_dev->debug_root,
			    g2d_dev, &g2d_debug_tasks_fops);
}

void g2d_destroy_debug(struct g2d_device *g2d_dev)
{
	debugfs_remove_recursive(g2d_dev->debug_root);
}

static struct regs_info g2d_reg_info_common[] = {
	/* Start, Size, Name */
	{ 0x0,		0x20,	"General" },
	{ 0x34,		0x10,	"Secure Layer" },
	{ 0x80,		0x70,	"Job manager" },
	{ 0x2000,	0x90,	"Layer CSC Coefficient" },
	{ 0x2100,	0x24,	"Dest CSC Coefficient" },
};

static struct regs_info g2d_reg_info_afbc = { 0xF0, 0x10, "AFBC debugging" };

static struct regs_info g2d_reg_info_hdr10[] = {
	{ 0x3000,	0x110,	"HDR EOTF" },
	{ 0x3200,	0x110,	"DEGAMMA EOTF" },
	{ 0x3400,	0x30,	"HDR GM" },
	{ 0x3500,	0x30,	"DEGAMMA 2.2" },
	{ 0x3600,	0x90,	"HDR TM" },
	{ 0x3700,	0x90,	"DEGAMMA TM" },
};

static struct regs_info g2d_reg_info_hwfc = {0x8000, 0x100, "HW flow control"};

static struct regs_info g2d_reg_info_dst = {0x120, 0xE0, "Destination"};

#define LAYER_REG_INFO(n) { 0x200 + 0x100 * (n), 0x100, "Layer" #n }
static struct regs_info g2d_reg_info_layer[] = {
	LAYER_REG_INFO(0), LAYER_REG_INFO(1),
	LAYER_REG_INFO(2), LAYER_REG_INFO(3),
	LAYER_REG_INFO(4), LAYER_REG_INFO(5),
	LAYER_REG_INFO(6), LAYER_REG_INFO(7),
	LAYER_REG_INFO(8), LAYER_REG_INFO(9),
	LAYER_REG_INFO(10), LAYER_REG_INFO(11),
	LAYER_REG_INFO(12), LAYER_REG_INFO(13),
	LAYER_REG_INFO(14), LAYER_REG_INFO(15)
};

static struct regs_info g2d_reg_info_hdr10p_ctrl = {0x3000, 8, "HDR_CONTROL"};

#define HDR10P_INFO_OETF(n) { 0x3000 + 0x800 * (n) + 8, 0x88, "L" #n "_MOD_CTRL+L" #n "_OETF" }
#define HDR10P_INFO_EOTF(n) { 0x3000 + 0x800 * (n) + 0x94, 0x308, "L" #n "_EOTF" }
#define HDR10P_INFO_GM(n)   { 0x3000 + 0x800 * (n) + 0x39C, 0x30, "L" #n "_GM" }
#define HDR10P_INFO_TM(n)   { 0x3000 + 0x800 * (n) + 0x3CC, 0xD4, "L" #n "_TM" }

static struct regs_info g2d_reg_info_hdr10p_oetf[] = {
	HDR10P_INFO_OETF(0), HDR10P_INFO_OETF(1),
	HDR10P_INFO_OETF(2), HDR10P_INFO_OETF(3)
};

static struct regs_info g2d_reg_info_hdr10p_eotf[] = {
	HDR10P_INFO_EOTF(0), HDR10P_INFO_EOTF(1),
	HDR10P_INFO_EOTF(2), HDR10P_INFO_EOTF(3)
};

static struct regs_info g2d_reg_info_hdr10p_gm[] = {
	HDR10P_INFO_GM(0), HDR10P_INFO_GM(1),
	HDR10P_INFO_GM(2), HDR10P_INFO_GM(3)
};

static struct regs_info g2d_reg_info_hdr10p_tm[] = {
	HDR10P_INFO_TM(0), HDR10P_INFO_TM(1),
	HDR10P_INFO_TM(2), HDR10P_INFO_TM(3)
};

#define FILTER_REG_INFO_Y_V(n) { 0x6000 + (n) * 0x400, 0x90, "FILTER" #n "_Y_VCOEF" }
#define FILTER_REG_INFO_Y_H(n) { 0x6000 + (n) * 0x400 + 0x90, 0x120, "FILTER" #n "_Y_HCOEF" }
#define FILTER_REG_INFO_C_V(n) { 0x6000 + (n) * 0x400 + 0x200, 0x90, "FILTER" #n "_C_VCOEF" }
#define FILTER_REG_INFO_C_H(n) { 0x6000 + (n) * 0x400 + 0x290, 0x120, "FILTER" #n "_C_HCOEF" }

static struct regs_info g2d_reg_info_filter_yh[] = {
	FILTER_REG_INFO_Y_H(0), FILTER_REG_INFO_Y_H(1),
	FILTER_REG_INFO_Y_H(2), FILTER_REG_INFO_Y_H(3)
};

static struct regs_info g2d_reg_info_filter_yv[] = {
	FILTER_REG_INFO_Y_V(0), FILTER_REG_INFO_Y_V(1),
	FILTER_REG_INFO_Y_V(2), FILTER_REG_INFO_Y_V(3)
};

static struct regs_info g2d_reg_info_filter_ch[] = {
	FILTER_REG_INFO_C_H(0), FILTER_REG_INFO_C_H(1),
	FILTER_REG_INFO_C_H(2), FILTER_REG_INFO_C_H(3)
};

static struct regs_info g2d_reg_info_filter_cv[] = {
	FILTER_REG_INFO_C_V(0),FILTER_REG_INFO_C_V(1),
	FILTER_REG_INFO_C_V(2), FILTER_REG_INFO_C_V(3)
};

#define G2D_COMP_DEBUG_DATA_COUNT 16

void g2d_dump_afbcdata(struct g2d_device *g2d_dev)
{
	int i, cluster;
	u32 cfg;

	/* AFBC debugging register is not present with AFBCv1.2 */
	if (!!(g2d_dev->caps & G2D_DEVICE_CAPS_AFBC_V12))
		return;

	for (cluster = 0; cluster < 2; cluster++) {
		for (i = 0; i < G2D_COMP_DEBUG_DATA_COUNT; i++) {
			writel_relaxed(i | cluster << 5, g2d_dev->reg + G2D_COMP_DEBUG_ADDR_REG);
			cfg = readl_relaxed(g2d_dev->reg + G2D_COMP_DEBUG_DATA_REG);

			pr_err("AFBC_DEBUGGING_DATA cluster%d [%d] %#x", cluster, i, cfg);
		}
	}
}

static void g2d_dump_sfr_entry(const char *name, void *base,
			       unsigned int offset, size_t len)
{
	pr_info("[%s: %#04x .. %#04lx]\n", name, offset, offset + len - 4);
	print_hex_dump(KERN_INFO, "+", DUMP_PREFIX_OFFSET, 32, 4,
		       base + offset, len, false);
}

static void __g2d_dump_sfr(void *base, struct regs_info *regs, size_t count)
{
	size_t i;

	for (i = 0; i < count; i++)
		g2d_dump_sfr_entry(regs[i].name, base,
				   regs[i].start, regs[i].size);
}

static void g2d_dump_common_sfr(struct g2d_device *g2d_dev,
				struct g2d_task *task,
				unsigned int *hdrmap, unsigned int *filtermap)
{
	unsigned int nr_layer = g2d_dev->max_layers;
	unsigned int i;

	__g2d_dump_sfr(g2d_dev->reg, g2d_reg_info_common, ARRAY_SIZE(g2d_reg_info_common));
	__g2d_dump_sfr(g2d_dev->reg, &g2d_reg_info_dst, 1);

	for (i = 0; i < nr_layer; i++) {
		struct regs_info *info = &g2d_reg_info_layer[i];
		unsigned int val;

		g2d_dump_sfr_entry(info->name, g2d_dev->reg, info->start, info->size);

		if (!!(g2d_dev->caps & G2D_DEVICE_CAPS_POLYFILTER)) {
			val = readl_relaxed(g2d_dev->reg + info->start + 0x48);
			if ((val & 0x3) == 3) {
				val >>= 4;
				*filtermap |= 1 << (val & 0x3);
			}
		}

		if (!!(g2d_dev->caps & G2D_DEVICE_CAPS_HDR10PLUS)) {
			val = readl_relaxed(g2d_dev->reg + info->start + 0x90);
			if (!!(val & (1 << 12)))
				*hdrmap |= 1 << (val & 0x3);
		}
	}
}

static void __g2d_dump_hdr10p(void __iomem *reg, unsigned int map)
{
	unsigned int i;

	if (!map)
		return;

	__g2d_dump_sfr(reg, &g2d_reg_info_hdr10p_ctrl, 1);

	for (i = 0; i < 4; i++) {
		if (!(map & (1 << i)))
			continue;

		g2d_dump_sfr_entry(g2d_reg_info_hdr10p_eotf[i].name, reg,
				   g2d_reg_info_hdr10p_eotf[i].start,
				   g2d_reg_info_hdr10p_eotf[i].size);
		g2d_dump_sfr_entry(g2d_reg_info_hdr10p_oetf[i].name, reg,
				   g2d_reg_info_hdr10p_oetf[i].start,
				   g2d_reg_info_hdr10p_oetf[i].size);
		g2d_dump_sfr_entry(g2d_reg_info_hdr10p_gm[i].name, reg,
				   g2d_reg_info_hdr10p_gm[i].start,
				   g2d_reg_info_hdr10p_gm[i].size);
		g2d_dump_sfr_entry(g2d_reg_info_hdr10p_tm[i].name, reg,
				   g2d_reg_info_hdr10p_tm[i].start,
				   g2d_reg_info_hdr10p_tm[i].size);
	}
}

static void __g2d_dump_filter(void __iomem *reg, unsigned int map)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		g2d_dump_sfr_entry(g2d_reg_info_filter_yv[i].name, reg,
				   g2d_reg_info_filter_yv[i].start,
				   g2d_reg_info_filter_yv[i].size);
		g2d_dump_sfr_entry(g2d_reg_info_filter_yh[i].name, reg,
				   g2d_reg_info_filter_yh[i].start,
				   g2d_reg_info_filter_yh[i].size);
		g2d_dump_sfr_entry(g2d_reg_info_filter_cv[i].name, reg,
				   g2d_reg_info_filter_cv[i].start,
				   g2d_reg_info_filter_cv[i].size);
		g2d_dump_sfr_entry(g2d_reg_info_filter_ch[i].name, reg,
				   g2d_reg_info_filter_ch[i].start,
				   g2d_reg_info_filter_ch[i].size);
	}
}

void g2d_dump_sfr(struct g2d_device *g2d_dev, struct g2d_task *task)
{
	unsigned int hdrmap = task ? 0 : ~0;
	unsigned int filtermap = task ? 0 : ~0;

	g2d_dump_common_sfr(g2d_dev, task, &hdrmap, &filtermap);

	if (!!(g2d_dev->caps & G2D_DEVICE_CAPS_AFBC_V12))
		__g2d_dump_sfr(g2d_dev->reg, &g2d_reg_info_afbc, 1);

	if (!!(g2d_dev->caps & G2D_DEVICE_CAPS_HWFC))
		__g2d_dump_sfr(g2d_dev->reg, &g2d_reg_info_hwfc, 1);

	if (!!(g2d_dev->caps & G2D_DEVICE_CAPS_HDR10))
		__g2d_dump_sfr(g2d_dev->reg, g2d_reg_info_hdr10, ARRAY_SIZE(g2d_reg_info_hdr10));

	if (!!(g2d_dev->caps & G2D_DEVICE_CAPS_HDR10PLUS))
		__g2d_dump_hdr10p(g2d_dev->reg, hdrmap);

	if (!!(g2d_dev->caps & G2D_DEVICE_CAPS_POLYFILTER))
		__g2d_dump_filter(g2d_dev->reg, filtermap);
}

void g2d_dump_cmd(struct g2d_task *task)
{
	unsigned int i;
	struct g2d_reg *regs;

	if (!task)
		return;

	regs = page_address(task->cmd_page);

	for (i = 0; i < task->sec.cmd_count; i++)
		pr_info("G2D: CMD[%03d] %#06x, %#010x\n",
			i, regs[i].offset, regs[i].value);
}

/*
 * If it happens error interrupts, mmu errors, and H/W timeouts,
 * dump the SFR and job command list of task, AFBC debugging information
 */
void g2d_dump_info(struct g2d_device *g2d_dev, struct g2d_task *task)
{
	g2d_dump_cmd(task);
	g2d_dump_sfr(g2d_dev, task);
	g2d_dump_afbcdata(g2d_dev);
}

static void g2d_frame_state(struct g2d_task *task, u32 stampid, s32 val)
{
	char trace_name[32];

	switch (stampid) {
	case G2D_STAMP_STATE_TASK_RESOURCE:
		if (val) {
			scnprintf(trace_name, sizeof(trace_name), "g2d_frame_sw#%d",
				g2d_task_id(task));
			G2D_ATRACE_INT_PID(trace_name, 0, 0);
		} else {
			scnprintf(trace_name, sizeof(trace_name), "g2d_frame_sw#%d",
				g2d_task_id(task));
			G2D_ATRACE_INT_PID(trace_name, 1, 0);
		}
	break;
	case G2D_STAMP_STATE_PUSH:
		scnprintf(trace_name, sizeof(trace_name), "g2d_frame_hw#%d", g2d_task_id(task));
		G2D_ATRACE_INT_PID(trace_name, 1, 0);
	break;
	case G2D_STAMP_STATE_DONE:
		scnprintf(trace_name, sizeof(trace_name), "g2d_frame_hw#%d", g2d_task_id(task));
		G2D_ATRACE_INT_PID(trace_name, 0, 0);
	break;
	}
}

#if !IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
static inline int g2d_blending_start(int32_t index)
{
	return 0;
}

static inline int g2d_blending_end(int32_t index)
{
	return 0;
}
#endif

void g2d_stamp_task(struct g2d_task *task, u32 stampid, u64 val)
{
	int idx = G2D_STAMP_CLAMP_ID(atomic_inc_return(&g2d_stamp_id));
	struct g2d_stamp *stamp = &g2d_stamp_list[idx];

	g2d_frame_state(task, stampid, val);

	if (task) {
		stamp->state = task->state;
		stamp->job_id = g2d_task_id(task);
	} else {
		stamp->job_id = 0;
		stamp->state = 0;
	}

	if (g2d_stamp_types[stampid].type == G2D_STAMPTYPE_FENCE) {
		struct dma_fence *fence = (struct dma_fence *)val;

		strlcpy(stamp->fence.name, fence->ops->get_driver_name(fence),
			sizeof(stamp->fence.name));
		stamp->fence.seqno = fence->seqno;
	} else {
		stamp->val[0] = (u32)val;

		if (g2d_stamp_types[stampid].type == G2D_STAMPTYPE_PERF) {
			struct g2d_device *g2d_dev = task->g2d_dev;

			stamp->val[1] =	g2d_get_current_freq(g2d_dev->dvfs_int);
			stamp->val[2] = g2d_get_current_freq(g2d_dev->dvfs_mif);
		}
	}

	if (stampid == G2D_STAMP_STATE_DONE) {
		struct g2d_device *g2d_dev = task->g2d_dev;

		if (g2d_debug & (1 << DBG_DEBUG))
			g2d_dump_info(g2d_dev, task);

		g2d_info("Task %2d consumed %10lu us (int %lu mif %lu)\n",
			 stamp->job_id, (unsigned long)val,
			 g2d_get_current_freq(g2d_dev->dvfs_int),
			 g2d_get_current_freq(g2d_dev->dvfs_mif));
	}

	stamp->time = ktime_get();
	stamp->stamp = stampid;
	stamp->cpu = raw_smp_processor_id();

	/* LLWFD latency measure */
	/* media/exynos_tsmux.h includes below functions */
	if (task && IS_HWFC(task->flags)) {
		if (stampid == G2D_STAMP_STATE_PUSH)
			g2d_blending_start(g2d_task_id(task));
		if (stampid == G2D_STAMP_STATE_DONE)
			g2d_blending_end(g2d_task_id(task));
	}
}
