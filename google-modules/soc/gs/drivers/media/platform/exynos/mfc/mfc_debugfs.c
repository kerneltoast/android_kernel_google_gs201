/*
 * drivers/media/platform/exynos/mfc/mfc_debug.c
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include "mfc_core_pm.h"

#include "mfc_debugfs.h"
#include "mfc_sync.h"
#include "mfc_meminfo.h"

#include "mfc_queue.h"

unsigned int debug_level;
unsigned int debug_ts;
unsigned int debug_mode_en;
unsigned int dbg_enable;
unsigned int nal_q_dump;
unsigned int nal_q_disable;
unsigned int nal_q_parallel_disable;
unsigned int otf_dump;
unsigned int perf_measure_option;
unsigned int sfr_dump;
unsigned int llc_disable;
unsigned int slc_disable;
unsigned int slc_option;
unsigned int slc_partial_height_ratio;
unsigned int perf_boost_mode;
unsigned int drm_predict_disable;
unsigned int reg_test;
unsigned int meminfo_enable;
unsigned int memlog_level = 2;
unsigned int logging_option = MFC_LOGGING_ALL;
unsigned int feature_option;
unsigned int regression_option;
unsigned int core_balance;
unsigned int sbwc_disable;
unsigned int sscd_report;
unsigned int hdr_dump;
unsigned int idle_suspend_enable = 0;

static int __mfc_info_show(struct seq_file *s, void *unused)
{
	struct mfc_dev *dev = s->private;
	struct mfc_core *core = NULL;
	struct mfc_ctx *ctx = NULL;
	struct mfc_core_ctx *core_ctx = NULL;
	int i, j;
	char *codec_name = NULL;

	seq_puts(s, ">>> MFC common device information\n");
	seq_printf(s, " [DEBUG MODE] dt: %s sysfs: %s\n",
			dev->pdata->debug_mode ? "enabled" : "disabled",
			debug_mode_en ? "enabled" : "disabled");
	seq_printf(s, " [PERF BOOST] %s\n",
			perf_boost_mode ? "enabled" : "disabled");
	seq_printf(s, " [FEATURES] nal_q: %d(0x%x), skype: %d(0x%x), black_bar: %d(0x%x)\n",
			dev->pdata->nal_q.support, dev->pdata->nal_q.version,
			dev->pdata->skype.support, dev->pdata->skype.version,
			dev->pdata->black_bar.support,
			dev->pdata->black_bar.version);
	seq_printf(s, "           color_aspect_dec: %d(0x%x), enc: %d(0x%x)\n",
			dev->pdata->color_aspect_dec.support,
			dev->pdata->color_aspect_dec.version,
			dev->pdata->color_aspect_enc.support,
			dev->pdata->color_aspect_enc.version);
	seq_printf(s, "           static_info_dec: %d(0x%x), enc: %d(0x%x)\n",
			dev->pdata->static_info_dec.support,
			dev->pdata->static_info_dec.version,
			dev->pdata->static_info_enc.support,
			dev->pdata->static_info_enc.version);
	seq_printf(s, " [FORMATS] 10bit: %s, 422: %s, RGB: %s\n",
			dev->pdata->support_10bit ? "supported" : "not supported",
			dev->pdata->support_422 ? "supported" : "not supported",
			dev->pdata->support_rgb ? "supported" : "not supported");
	seq_printf(s, " [LOWMEM] is_low_mem: %d\n", IS_LOW_MEM);


	for (j = 0; j < dev->num_core; j++) {
		core = dev->core[j];
		if (!core) {
			mfc_dev_debug(2, "There is no core[%d]\n", j);
			continue;
		}
		seq_printf(s, ">>> MFC core-%d device information\n", j);
		seq_printf(s, " [VERSION] H/W: v%x, F/W: %06x(%c, normal: %#x, drm: %#x), DRV: %d\n",
				core->core_pdata->ip_ver, core->fw.date, core->fw.fimv_info,
				core->fw.status, core->fw.drm_status, MFC_DRIVER_INFO);
		seq_printf(s, " [PM] power: %d, clock: %d, clk_get %s, QoS level: %d\n",
				mfc_core_pm_get_pwr_ref_cnt(core),
				mfc_core_pm_get_clk_ref_cnt(core),
				IS_ERR(core->pm.clock) ? "failed" : "succeeded",
				atomic_read(&core->qos_req_cur) - 1);
		seq_printf(s, " [CTX] num_inst: %d, num_drm_inst: %d, curr_ctx: %d(is_drm: %d)\n",
				core->num_inst, core->num_drm_inst,
				core->curr_core_ctx,
				core->curr_core_ctx_is_drm);
		seq_printf(s, " [HWLOCK] bits: %#lx, dev: %#lx, owned_by_irq = %d, wl_count = %d\n",
				core->hwlock.bits, core->hwlock.dev,
				core->hwlock.owned_by_irq,
				core->hwlock.wl_count);
		if (core->nal_q_handle)
			seq_printf(s, " [NAL-Q] state: %d\n",
					core->nal_q_handle->nal_q_state);
		seq_printf(s, "  >>> MFC core-%d instance information\n", j);
		for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
			core_ctx = core->core_ctx[i];
			if (core_ctx) {
				seq_printf(s, "    [CORECTX:%d] state: %d, queue(src: %d, dst: %d)\n",
					i, core_ctx->state,
					mfc_get_queue_count(&core_ctx->buf_queue_lock,
						&core_ctx->src_buf_queue),
					mfc_get_queue_count(&core_ctx->buf_queue_lock,
						&core_ctx->dst_buf_queue));
			}
		}
	}
	seq_puts(s, ">>> MFC instance information\n");
	for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
		ctx = dev->ctx[i];
		if (ctx) {
			if (ctx->type == MFCINST_DECODER)
				codec_name = ctx->src_fmt->name;
			else
				codec_name = ctx->dst_fmt->name;

			seq_printf(s, "  [CTX:%d] %s %s, %s, %s, size: %dx%d@%ldfps(op: %ldfps), crop: %d %d %d %d\n",
				ctx->num,
				ctx->type == MFCINST_DECODER ? "DEC" : "ENC",
				ctx->is_drm ? "Secure" : "Normal",
				ctx->src_fmt->name, ctx->dst_fmt->name,
				ctx->img_width, ctx->img_height,
				ctx->last_framerate / 1000,
				ctx->operating_framerate,
				ctx->crop_width, ctx->crop_height,
				ctx->crop_left, ctx->crop_top);
			seq_printf(s, "        main core-%d, op_mode: %d, prio %d, rt %d, queue(src: %d, dst: %d, src_nal: %d, dst_nal: %d, ref: %d)\n",
				ctx->op_core_num[MFC_CORE_MAIN], ctx->op_mode,
				ctx->prio, ctx->rt,
				mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->src_buf_ready_queue),
				mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->dst_buf_queue),
				mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->src_buf_nal_queue),
				mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->dst_buf_nal_queue),
				mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->ref_buf_queue));
		}
	}

	return 0;
}

static int __mfc_debug_info_show(struct seq_file *s, void *unused)
{
	seq_puts(s, ">> MFC debug information\n");

	seq_puts(s, "-----SFR dump options (bit setting)\n");
	seq_puts(s, "ex) echo 0xff > /d/mfc/sfr_dump (all dump mode)\n");
	seq_puts(s, "1   (1 << 0): dec SEQ_START\n");
	seq_puts(s, "2   (1 << 1): dec INIT_BUFS\n");
	seq_puts(s, "4   (1 << 2): dec first NAL_START\n");
	seq_puts(s, "8   (1 << 3): enc SEQ_START\n");
	seq_puts(s, "16  (1 << 4): enc INIT_BUFS\n");
	seq_puts(s, "32  (1 << 5): enc first NAL_START\n");
	seq_puts(s, "64  (1 << 6): ERR interrupt\n");
	seq_puts(s, "128 (1 << 7): WARN interrupt\n");
	seq_puts(s, "256 (1 << 8): dec NAL_START\n");
	seq_puts(s, "512 (1 << 9): dec FRAME_DONE\n");
	seq_puts(s, "1024 (1 << 10): enc NAL_START\n");
	seq_puts(s, "2048 (1 << 11): enc FRAME_DONE\n");
	seq_puts(s, "4096 (1 << 12): MOVE_INSTANCE_RET\n");

	seq_puts(s, "-----Performance boost options (bit setting)\n");
	seq_puts(s, "ex) echo 7 > /d/mfc/perf_boost_mode (max freq)\n");
	seq_puts(s, "1   (1 << 0): DVFS (INT/MFC/MIF)\n");
	seq_puts(s, "2   (1 << 1): MO value\n");
	seq_puts(s, "4   (1 << 2): CPU frequency\n");

	seq_puts(s, "-----Feature options (bit setting)\n");
	seq_puts(s, "ex) echo 1 > /d/mfc/feture_option (recon sbwc off)\n");
	seq_puts(s, "1   (1 << 0): recon SBWC disable\n");
	seq_puts(s, "2   (1 << 1): decoding order\n");
	seq_puts(s, "4   (1 << 2): meerkat disable\n");
	seq_puts(s, "8   (1 << 3): OTF path test enable\n");
	seq_puts(s, "16  (1 << 4): multi core disable\n");
	seq_puts(s, "32  (1 << 5): force multi core enable\n");
	seq_puts(s, "64  (1 << 6): black bar enable\n");

	seq_puts(s, "-----Logging options (bit setting)\n");
	seq_puts(s, "ex) echo 7 > /d/mfc/logging_option (all logging option)\n");
	seq_puts(s, "1   (1 << 0): kernel printk\n");
	seq_puts(s, "2   (1 << 1): memlog printf\n");
	seq_puts(s, "4   (1 << 2): memlog sfr dump\n");

#ifdef CONFIG_MFC_USE_COREDUMP
	seq_puts(s, "-----Forcely run sscordump\n");
	seq_puts(s, "ex) echo 1 > /d/mfc/sscd_report\n");
#endif
	return 0;
}

#ifdef CONFIG_MFC_REG_TEST
static int __mfc_reg_info_show(struct seq_file *s, void *unused)
{
	struct mfc_dev *dev = g_mfc_dev;
	unsigned int i;

	seq_puts(s, ">> MFC REG test(encoder)\n");

	seq_printf(s, "-----Register test on/off: %s\n", reg_test ? "on" : "off");
	seq_printf(s, "-----Register number: %d\n", dev->reg_cnt);

	if (dev->reg_val) {
		seq_puts(s, "-----Register values\n");
		for (i = 0; i < dev->reg_cnt; i++) {
			seq_printf(s, "%08x ", dev->reg_val[i]);
			if ((i % 8) == 7)
				seq_printf(s, "\n");
		}
	} else {
		seq_puts(s, "-----There is no reg_buf, set the register value\n");
	}

	return 0;
}

static ssize_t __mfc_reg_info_write(struct file *file, const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct mfc_dev *dev = g_mfc_dev;
	ssize_t len;
	unsigned int index = 0;
	char temp_char;

	if (!dev->reg_buf) {
		dev->reg_buf = kzalloc(SZ_1M, GFP_KERNEL);
		mfc_dev_info("[REGTEST] alloc reg_buf\n");
	}

	if (!dev->reg_val) {
		dev->reg_val = kzalloc(SZ_1M, GFP_KERNEL);
		mfc_dev_info("[REGTEST] alloc reg_val\n");
	}

	len = simple_write_to_buffer(dev->reg_buf, SZ_1M - 1,
					ppos, user_buf, count);
	if (len <= 0)
		return len;

	mfc_dev_info("[REGTEST] len: %d, ppos: %llu\n", len, *ppos);

	dev->reg_buf[*ppos] = '\0';

	dev->reg_cnt = 0;
	while (index + 7 < *ppos) {
		temp_char = dev->reg_buf[index];
		if ((temp_char >= '0' && temp_char <= '9') ||
				(temp_char >= 'A' && temp_char <= 'F') ||
				(temp_char >= 'a' && temp_char <= 'f')) {
			sscanf(&dev->reg_buf[index], "%08x", &dev->reg_val[dev->reg_cnt]);
			index += 8;
			dev->reg_cnt++;
		} else {
			index++;
		}
	}

	return len;
}
#endif

static void __mfc_meminfo_show_all(struct seq_file *s, struct mfc_meminfo *meminfo, int cnt)
{
	char *type;
	int i;

	seq_puts(s, "buffer info:\n");
	seq_printf(s, "%10s %20s %14s %7s %10s %10s\n",
			"type", "buffer", "buf size", "count", "size", "size(hex)");

	for (i = 0; i < cnt; i++) {
		switch (meminfo[i].type) {
		case MFC_MEMINFO_FW:
			type = "FW";
			break;
		case MFC_MEMINFO_INTERNAL:
			type = "INTERNAL";
			break;
		case MFC_MEMINFO_INPUT:
			type = "INPUT";
			break;
		case MFC_MEMINFO_OUTPUT:
			type = "OUTPUT";
			break;
		default:
			type = "";
			break;
		}

		seq_printf(s, "%10s %20s %14zu %7d %10zu %#10zx\n",
				type,
				meminfo[i].name,
				meminfo[i].size,
				meminfo[i].count,
				meminfo[i].total,
				meminfo[i].total);
	}
}

static int __mfc_meminfo_show(struct seq_file *s, void *unused)
{
	struct mfc_dev *dev = s->private;
	struct mfc_ctx *ctx = NULL;
	char *codec_name = NULL, *fmt_name = NULL;
	size_t total = 0, total_max = 0;
	int i, num;

	if (!meminfo_enable) {
		seq_puts(s, "meminfo_enable is not set. ""echo 1 > meminfo_enable""\n");
		return 0;
	}

	seq_puts(s, "\n\n-----------------------------------------\n");
	seq_puts(s, ">> MFC memory information\n");
	seq_puts(s, "-----------------------------------------\n");

	num = mfc_meminfo_get_dev(dev);
	seq_printf(s, "\n>> [DEV] memory size: %zu kB\n",
			(dev->meminfo[MFC_MEMINFO_DEV_ALL].total / 1024));
	total += dev->meminfo[MFC_MEMINFO_DEV_ALL].total;
	total_max += dev->meminfo[MFC_MEMINFO_DEV_ALL].total;

	__mfc_meminfo_show_all(s, dev->meminfo, num);

	for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
		ctx = dev->ctx[i];
		if (ctx) {
			if (ctx->type == MFCINST_DECODER) {
				codec_name = ctx->src_fmt->name;
				fmt_name = ctx->dst_fmt->name;
			} else {
				codec_name = ctx->dst_fmt->name;
				fmt_name = ctx->src_fmt->name;
			}

//			num = mfc_meminfo_get_ctx(ctx);
			seq_puts(s, "\n-----------------------------------------\n");
			seq_printf(s, ">> [CTX:%d] %s memory size: %zu kB (MAX: %zu kB)\n",
				ctx->num,
				ctx->type == MFCINST_DECODER ? "Decoder" : "Encoder",
				(ctx->meminfo_size[MFC_MEMINFO_CTX_ALL] / 1024),
				(ctx->meminfo_size[MFC_MEMINFO_CTX_MAX] / 1024));
			seq_printf(s, "Input buffer		%zu kB\n",
				(ctx->meminfo_size[MFC_MEMINFO_INPUT] / 1024));
			seq_printf(s, "Output buffer		%zu kB\n",
				(ctx->meminfo_size[MFC_MEMINFO_OUTPUT] / 1024));
			seq_printf(s, "Internal buffer		%zu kB\n",
				(ctx->meminfo_size[MFC_MEMINFO_INTERNAL] / 1024));
			seq_puts(s, "info:\n");
			seq_printf(s, "codec type   %s\n", codec_name);
			seq_printf(s, "frame format %s\n", fmt_name);
			seq_printf(s, "frame size   %d x %d\n", ctx->img_width, ctx->img_height);
			if (ctx->type == MFCINST_DECODER)
				seq_printf(s, "dpb count    %d\n", (ctx->dpb_count + MFC_NUM_EXTRA_DPB));

			total += ctx->meminfo_size[MFC_MEMINFO_CTX_ALL];
			total_max += ctx->meminfo_size[MFC_MEMINFO_CTX_MAX];
			__mfc_meminfo_show_all(s, ctx->meminfo, num);
		}
	}
	seq_puts(s, "\n=========================================\n");
	seq_printf(s, "MFC MEMORY SIZE: %zu kB (MAX: %zu kB)\n",
			total / 1024, total_max / 1024);
	seq_puts(s, "=========================================\n");

	return 0;
}

static int __mfc_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, __mfc_info_show, inode->i_private);
}

static int __mfc_debug_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, __mfc_debug_info_show, inode->i_private);
}

static int __mfc_regression_result_show(struct seq_file *s, void *unused)
{
	struct mfc_dev *dev = g_mfc_dev;
	int i;


	if (dev->regression_val) {
		for (i = 0; i < dev->regression_cnt; i++) {
			if (regression_option & MFC_TEST_ENC_QP)
				seq_printf(s, "%d ", dev->regression_val[i]);
			else
				seq_printf(s, "%08x ", dev->regression_val[i]);
			if ((dev->regression_val[i] == 0xDEADC0DE) ||
				(regression_option & MFC_TEST_ENC_QP))
				seq_printf(s, "\n");
		}
	} else {
		seq_puts(s, "-----There is no regression result\n");
	}

	return 0;
}

#ifdef CONFIG_MFC_REG_TEST
static int __mfc_reg_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, __mfc_reg_info_show, inode->i_private);
}
#endif

static int __mfc_meminfo_open(struct inode *inode, struct file *file)
{
	return single_open(file, __mfc_meminfo_show, inode->i_private);
}

static int __mfc_regression_result_open(struct inode *inode, struct file *file)
{
	return single_open(file, __mfc_regression_result_show, inode->i_private);
}

static const struct file_operations mfc_info_fops = {
	.open = __mfc_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations debug_info_fops = {
	.open = __mfc_debug_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#ifdef CONFIG_MFC_REG_TEST
static const struct file_operations reg_info_fops = {
	.open = __mfc_reg_info_open,
	.read = seq_read,
	.write = __mfc_reg_info_write,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static const struct file_operations mfc_meminfo_fops = {
	.open = __mfc_meminfo_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations regression_result_fops = {
	.open = __mfc_regression_result_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void mfc_init_debugfs(struct mfc_dev *dev)
{
	struct mfc_debugfs *debugfs = &dev->debugfs;

	debugfs->root = debugfs_create_dir("mfc", NULL);
	if (!debugfs->root) {
		mfc_dev_err("debugfs: failed to create root directory\n");
		return;
	}

	debugfs_create_file("mfc_info",
			0444, debugfs->root, dev, &mfc_info_fops);
	debugfs_create_file("debug_info",
			0444, debugfs->root, dev, &debug_info_fops);
#ifdef CONFIG_MFC_REG_TEST
	debugfs_create_file("reg_info",
			0644, debugfs->root, dev, &reg_info_fops);
	debugfs_create_u32("reg_test",
			0644, debugfs->root, &reg_test);
#endif
	debugfs_create_u32("regression_option",
			0644, debugfs->root, &regression_option);
	debugfs_create_file("regression_result",
			0444, debugfs->root, dev, &regression_result_fops);
	debugfs_create_u32("debug",
			0644, debugfs->root, &debug_level);
	debugfs_create_u32("debug_ts",
			0644, debugfs->root, &debug_ts);
	debugfs_create_u32("debug_mode_en",
			0644, debugfs->root, &debug_mode_en);
	debugfs_create_u32("dbg_enable",
			0644, debugfs->root, &dbg_enable);
	debugfs_create_u32("nal_q_dump",
			0644, debugfs->root, &nal_q_dump);
	debugfs_create_u32("nal_q_disable",
			0644, debugfs->root, &nal_q_disable);
	debugfs_create_u32("nal_q_parallel_disable",
			0644, debugfs->root, &nal_q_parallel_disable);
	debugfs_create_u32("otf_dump",
			0644, debugfs->root, &otf_dump);
	debugfs_create_u32("perf_measure_option",
			0644, debugfs->root, &perf_measure_option);
	debugfs_create_u32("sfr_dump",
			0644, debugfs->root, &sfr_dump);
	debugfs_create_u32("llc_disable",
			0644, debugfs->root, &llc_disable);
	debugfs_create_u32("slc_disable",
			0644, debugfs->root, &slc_disable);
	debugfs_create_u32("slc_option",
			0644, debugfs->root, &slc_option);
	debugfs_create_u32("slc_partial_height_ratio",
			0644, debugfs->root, &slc_partial_height_ratio);
	debugfs_create_u32("perf_boost_mode",
			0644, debugfs->root, &perf_boost_mode);
	debugfs_create_u32("drm_predict_disable",
			0644, debugfs->root, &drm_predict_disable);
	debugfs_create_file("meminfo",
			0444, debugfs->root, dev, &mfc_meminfo_fops);
	debugfs_create_u32("meminfo_enable",
			0644, debugfs->root, &meminfo_enable);
	debugfs_create_u32("feature_option",
			0644, debugfs->root, &feature_option);
	debugfs_create_u32("core_balance",
			0644, debugfs->root, &core_balance);
	debugfs_create_u32("memlog_level",
			0644, debugfs->root, &memlog_level);
	debugfs_create_u32("logging_option",
			0644, debugfs->root, &logging_option);
	debugfs_create_u32("sbwc_disable",
			0644, debugfs->root, &sbwc_disable);
	debugfs_create_u32("idle_suspend_enable",
			0644, debugfs->root, &idle_suspend_enable);
#ifdef CONFIG_MFC_USE_COREDUMP
	debugfs_create_u32("sscd_report",
			0644, debugfs->root, &sscd_report);
#endif
	debugfs_create_u32("hdr_dump",
			0644, debugfs->root, &hdr_dump);
}

void mfc_deinit_debugfs(struct mfc_dev *dev)
{
	struct mfc_debugfs *debugfs = &dev->debugfs;

	if (debugfs->root)
		debugfs_remove_recursive(debugfs->root);
}
