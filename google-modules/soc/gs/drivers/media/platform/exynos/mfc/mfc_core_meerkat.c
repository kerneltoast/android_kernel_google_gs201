/*
 * drivers/media/platform/exynos/mfc/mfc_core_meerkat.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#if IS_ENABLED(CONFIG_SEC_DEBUG_EXTRA_INFO)
#include <linux/sec_debug.h>
#endif
#include <linux/workqueue.h>
#include <soc/google/debug-snapshot.h>

#include "mfc_rm.h"

#include "mfc_core_meerkat.h"

#include "mfc_core_pm.h"
#include "mfc_core_reg_api.h"
#include "mfc_core_hw_reg_api.h"
#include "mfc_core_otf.h"

#include "mfc_sync.h"
#include "mfc_queue.h"

#define MFC_SFR_AREA_COUNT	22
static void __mfc_dump_regs(struct mfc_core *core)
{
	int i;
	struct mfc_ctx_buf_size *buf_size = NULL;
	int addr[MFC_SFR_AREA_COUNT][2] = {
		{ 0x0, 0x80 },
		{ 0x1000, 0xCD0 },
		{ 0xF000, 0xFF8 },
		{ 0x2000, 0xA00 },
		{ 0x2f00, 0x6C },
		{ 0x3000, 0x40 },
		{ 0x3094, 0x4 },
		{ 0x30b4, 0x8 },
		{ 0x3110, 0x10 },
		{ 0x5000, 0x130 },
		{ 0x5200, 0x300 },
		{ 0x5600, 0x100 },
		{ 0x5800, 0x100 },
		{ 0x5A00, 0x100 },
		{ 0x6000, 0x188 },
		{ 0x7000, 0x21C },
		{ 0x7300, 0xFC },
		{ 0x8000, 0x20C },
		{ 0x9000, 0x10C },
		{ 0xA000, 0x500 },
		{ 0xB000, 0x444 },
		{ 0xC000, 0x84 },
	};

	dev_err(core->device, "-----------dumping MFC registers\n");

	if (!mfc_core_pm_get_pwr_ref_cnt(core)) {
		dev_err(core->device, "Power(%d) is not enabled\n",
				mfc_core_pm_get_pwr_ref_cnt(core));
		return;
	}

	mfc_core_enable_all_clocks(core);

	for (i = 0; i < MFC_SFR_AREA_COUNT; i++) {
		printk("[%04X .. %04X]\n", addr[i][0], addr[i][0] + addr[i][1]);
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 32, 4, core->regs_base + addr[i][0],
				addr[i][1], false);
		printk("...\n");
	}

	if (dbg_enable) {
		buf_size = core->dev->variant->buf_size->ctx_buf;
		printk("[DBG INFO dump]\n");
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 32, 4, core->dbg_info_buf.vaddr,
			buf_size->dbg_info_buf, false);
		printk("...\n");
	}

	if (core->has_mfc_votf) {
		printk("[MFC vOTF dump]\n");
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 32, 4, core->votf_base,
			0x1000, false);
		printk("...\n");
	}
}

/* common register */
const u32 mfc_logging_sfr_set0[MFC_SFR_LOGGING_COUNT_SET0] = {
	0x0070, 0x1000, 0x1004, 0x100C, 0x1010, 0x01B4, 0xF144, 0xF148,
	0xF088, 0xFFD0
};

/* AxID: the other */
const u32 mfc_logging_sfr_set1[MFC_SFR_LOGGING_COUNT_SET1] = {
	0x3000, 0x3004, 0x3010, 0x301C, 0x3110, 0x3140, 0x3144, 0x5068,
	0x506C, 0x5254, 0x5280, 0x529C, 0x52A0, 0x5A54, 0x5A80, 0x5A88,
	0x5A94, 0x5A9C, 0x6038, 0x603C, 0x6050, 0x6064, 0x6168, 0x616C,
	0x2020, 0x2028, 0x202C, 0x20B4
};

/* AxID: 0 ~ 3 (READ): PX */
const u32 mfc_logging_sfr_set2[MFC_SFR_LOGGING_COUNT_SET2] = {
	0xA100, 0xA104, 0xA108, 0xA10C, 0xA110, 0xA114, 0xA118, 0xA11C,
	0xA120, 0xA124, 0xA128, 0xA12C, 0xA120, 0xA124, 0xA128, 0xA12C,
	0xA180, 0xA184, 0xA188, 0xA18C, 0xA190, 0xA194, 0xA198, 0xA19C,
	0xA1A0, 0xA1A4, 0xA1A8, 0xA1AC, 0xA1B0, 0xA1B4, 0xA1B8, 0xA1BC
};

int __mfc_change_hex_to_ascii(struct mfc_core *core, u32 hex, u32 byte, char *ascii, int idx)
{
	int i;
	char tmp;

	for (i = 0; i < byte; i++) {
		if (idx >= MFC_LOGGING_DATA_SIZE) {
			dev_err(core->device, "logging data size exceed: %d\n", idx);
			return idx;
		}

		tmp = (hex >> ((byte - 1 - i) * 4)) & 0xF;
		if (tmp > 9)
			ascii[idx] = tmp + 'a' - 0xa;
		else if (tmp <= 9)
			ascii[idx] = tmp + '0';
		idx++;
	}

	/* space */
	if (idx < MFC_LOGGING_DATA_SIZE)
		ascii[idx] = ' ';

	return ++idx;
}

static void __mfc_merge_errorinfo_data(struct mfc_core *core, bool px_fault)
{
	char *errorinfo;
	int i, idx = 0;
	int trace_cnt, ret, cnt;

	errorinfo = core->logging_data->errorinfo;

	/* FW info */
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->fw_version, 8, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->cause, 8, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->fault_status, 2, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->fault_trans_info, 8, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->fault_addr, 8, errorinfo, idx);
	for (i = 0; i < MFC_SFR_LOGGING_COUNT_SET0; i++)
		idx = __mfc_change_hex_to_ascii(core, core->logging_data->SFRs_set0[i], 8, errorinfo, idx);
	if (px_fault) {
		for (i = 0; i < MFC_SFR_LOGGING_COUNT_SET2; i++)
			idx = __mfc_change_hex_to_ascii(core, core->logging_data->SFRs_set2[i], 8, errorinfo, idx);
	} else {
		for (i = 0; i < MFC_SFR_LOGGING_COUNT_SET1; i++)
			idx = __mfc_change_hex_to_ascii(core, core->logging_data->SFRs_set1[i], 8, errorinfo, idx);
	}

	/* driver info */
	if (idx >= MFC_LOGGING_DATA_SIZE) {
		dev_err(core->device, "logging data size exceed: %d\n", idx);
		return;
	}
	ret = snprintf(errorinfo + idx, 3, "/");
	idx += ret;
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->curr_ctx, 2, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->state, 2, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->last_cmd, 2, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->last_cmd_sec, 8, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core,
				core->logging_data->last_cmd_nsec / NSEC_PER_USEC, 8, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->last_int, 2, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->last_int_sec, 8, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core,
				core->logging_data->last_int_nsec / NSEC_PER_USEC, 8, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->frame_cnt, 8, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->hwlock_dev, 2, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->hwlock_ctx, 8, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->num_inst, 2, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->num_drm_inst, 2, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->power_cnt, 2, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->clock_cnt, 2, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->dynamic_used, 8, errorinfo, idx);
	idx = __mfc_change_hex_to_ascii(core, core->logging_data->last_src_addr, 8, errorinfo, idx);
	for (i = 0; i < MFC_MAX_PLANES; i++)
		idx = __mfc_change_hex_to_ascii(core, core->logging_data->last_dst_addr[i], 8, errorinfo, idx);

	/* last trace info */
	if (idx >= MFC_LOGGING_DATA_SIZE) {
		dev_err(core->device, "logging data size exceed: %d\n", idx);
		return;
	}
	ret = snprintf(errorinfo + idx, 3, "/");
	idx += ret;

	/* last processing is first printed */
	trace_cnt = atomic_read(&core->trace_ref_log);
	for (i = 0; i < MFC_TRACE_LOG_COUNT_PRINT; i++) {
		if (idx >= (MFC_LOGGING_DATA_SIZE - MFC_TRACE_LOG_STR_LEN)) {
			dev_err(core->device, "logging data size exceed: %d\n", idx);
			break;
		}
		cnt = ((trace_cnt + MFC_TRACE_LOG_COUNT_MAX) - i) % MFC_TRACE_LOG_COUNT_MAX;
		ret = snprintf(errorinfo + idx, MFC_TRACE_LOG_STR_LEN, "%llu:%s ",
				core->mfc_trace_logging[cnt].time,
				core->mfc_trace_logging[cnt].str);
		idx += ret;
	}

	dev_err(core->device, "%s\n", errorinfo);

#if IS_ENABLED(CONFIG_SEC_DEBUG_EXTRA_INFO)
	sec_debug_set_extra_info_mfc_error(errorinfo);
#endif
}

static int __mfc_get_curr_ctx(struct mfc_core *core)
{
	struct mfc_core_ctx *core_ctx = NULL;
	nal_queue_handle *nal_q_handle = core->nal_q_handle;
	unsigned int index, offset;
	DecoderInputStr *pStr;
	int i;
	int curr_ctx = core->curr_core_ctx;

	if (nal_q_handle) {
		if (nal_q_handle->nal_q_state == NAL_Q_STATE_STARTED) {
			index = nal_q_handle->nal_q_in_handle->in_exe_count % NAL_Q_QUEUE_SIZE;
			offset = core->dev->pdata->nal_q_entry_size * index;
			pStr = (DecoderInputStr *)(nal_q_handle->nal_q_in_handle->nal_q_in_addr + offset);
			curr_ctx = pStr->InstanceId;
		}
	}

	core_ctx = core->core_ctx[curr_ctx];
	if (!core_ctx) {
		for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
			if (core->core_ctx[i]) {
				core_ctx = core->core_ctx[i];
				break;
			}
		}

		if (!core_ctx) {
			dev_err(core->device, "there is no ctx structure for dumpping\n");
			return -EINVAL;
		}
		dev_err(core->device, "curr ctx is changed %d -> %d\n",
				curr_ctx, core_ctx->num);
	}

	return core_ctx->num;
}

static void __mfc_save_logging_sfr(struct mfc_core *core)
{
	struct mfc_core_ctx *core_ctx = NULL;
	struct mfc_ctx *ctx = NULL;
	int i;
	bool px_fault = false;
	int curr_ctx = __mfc_get_curr_ctx(core);

	dev_err(core->device, "-----------logging MFC error info-----------\n");
	if (mfc_core_pm_get_pwr_ref_cnt(core)) {
		core->logging_data->cause |= (1 << MFC_LAST_INFO_POWER);
		core->logging_data->fw_version = mfc_core_get_fw_ver_all();

		for (i = 0; i < MFC_SFR_LOGGING_COUNT_SET0; i++)
			core->logging_data->SFRs_set0[i] = MFC_CORE_READL(mfc_logging_sfr_set0[i]);
		for (i = 0; i < MFC_SFR_LOGGING_COUNT_SET1; i++)
			core->logging_data->SFRs_set1[i] = MFC_CORE_READL(mfc_logging_sfr_set1[i]);

		/* READ PAGE FAULT at AxID 0 ~ 3: PX */
		if ((core->logging_data->cause & (1 << MFC_CAUSE_0PAGE_FAULT)) ||
				(core->logging_data->cause & (1 << MFC_CAUSE_1PAGE_FAULT))) {
			if ((core->logging_data->fault_trans_info & 0xff) <= 3) {
				px_fault = true;
				for (i = 0; i < MFC_SFR_LOGGING_COUNT_SET2; i++)
					core->logging_data->SFRs_set2[i] = MFC_CORE_READL(mfc_logging_sfr_set2[i]);
			}
		}
	}

	if (mfc_core_pm_get_clk_ref_cnt(core))
		core->logging_data->cause |= (1 << MFC_LAST_INFO_CLOCK);

	if (core->nal_q_handle && (core->nal_q_handle->nal_q_state == NAL_Q_STATE_STARTED))
		core->logging_data->cause |= (1 << MFC_LAST_INFO_NAL_QUEUE);

	core->logging_data->cause |= (core->shutdown << MFC_LAST_INFO_SHUTDOWN);
	core->logging_data->cause |= (core->curr_core_ctx_is_drm << MFC_LAST_INFO_DRM);

	if (curr_ctx >= 0) {
		core_ctx = core->core_ctx[curr_ctx];
		ctx = core_ctx->ctx;
	}

	/* last information */
	core->logging_data->curr_ctx = curr_ctx;
	core->logging_data->last_cmd = core->last_cmd;
	core->logging_data->last_cmd_sec = (u32)(core->last_cmd_time.tv_sec);
	core->logging_data->last_cmd_nsec = (u32)(core->last_cmd_time.tv_nsec);
	core->logging_data->last_int = core->last_int;
	core->logging_data->last_int_sec = (u32)(core->last_int_time.tv_sec);
	core->logging_data->last_int_nsec = (u32)(core->last_int_time.tv_nsec);
	core->logging_data->hwlock_dev = core->hwlock.dev;
	core->logging_data->hwlock_ctx = (u32)(core->hwlock.bits);
	core->logging_data->num_inst = core->num_inst;
	core->logging_data->num_drm_inst = core->num_drm_inst;
	core->logging_data->power_cnt = mfc_core_pm_get_pwr_ref_cnt(core);
	core->logging_data->clock_cnt = mfc_core_pm_get_clk_ref_cnt(core);

	if (ctx) {
		core->logging_data->state = core_ctx->state;
		core->logging_data->frame_cnt = ctx->frame_cnt;
		if (ctx->type == MFCINST_DECODER) {
			struct mfc_dec *dec = ctx->dec_priv;

			if (dec) {
				core->logging_data->dynamic_used = dec->dynamic_used;
				core->logging_data->cause |= (dec->detect_black_bar << MFC_LAST_INFO_BLACK_BAR);
			}
			core->logging_data->last_src_addr = ctx->last_src_addr;
			for (i = 0; i < MFC_MAX_PLANES; i++)
				core->logging_data->last_dst_addr[i] = ctx->last_dst_addr[i];
		}
	}

	__mfc_merge_errorinfo_data(core, px_fault);
}

void mfc_dump_state(struct mfc_dev *dev)
{
	int i;

	dev_err(dev->device, "-----------dumping MFC device info-----------\n");
	dev_err(dev->device, "options debug_level:%d, debug_mode:%d (%d), perf_boost:%d, wait_fw_status %d\n",
			debug_level, dev->pdata->debug_mode, debug_mode_en,
			perf_boost_mode, dev->pdata->wait_fw_status.support);

	for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
		if (dev->ctx[i]) {
			dev_err(dev->device, "- ctx[%d] %s %s, %s, %s, size: %dx%d@%ldfps(op: %ldfps), crop: %d %d %d %d\n",
				dev->ctx[i]->num,
				dev->ctx[i]->type == MFCINST_DECODER ? "DEC" : "ENC",
				dev->ctx[i]->is_drm ? "Secure" : "Normal",
				dev->ctx[i]->src_fmt->name,
				dev->ctx[i]->dst_fmt->name,
				dev->ctx[i]->img_width, dev->ctx[i]->img_height,
				dev->ctx[i]->last_framerate / 1000,
				dev->ctx[i]->operating_framerate,
				dev->ctx[i]->crop_width, dev->ctx[i]->crop_height,
				dev->ctx[i]->crop_left, dev->ctx[i]->crop_top);
			dev_err(dev->device, "\tmain core-%d, op_mode: %d, prio %d, rt %d, queue_cnt(src:%d, dst:%d, ref:%d, qsrc:%d, qdst:%d)\n",
				dev->ctx[i]->op_core_num[MFC_CORE_MAIN],
				dev->ctx[i]->op_mode,
				dev->ctx[i]->prio, dev->ctx[i]->rt,
				mfc_get_queue_count(&dev->ctx[i]->buf_queue_lock, &dev->ctx[i]->src_buf_ready_queue),
				mfc_get_queue_count(&dev->ctx[i]->buf_queue_lock, &dev->ctx[i]->dst_buf_queue),
				mfc_get_queue_count(&dev->ctx[i]->buf_queue_lock, &dev->ctx[i]->ref_buf_queue),
				mfc_get_queue_count(&dev->ctx[i]->buf_queue_lock, &dev->ctx[i]->src_buf_nal_queue),
				mfc_get_queue_count(&dev->ctx[i]->buf_queue_lock, &dev->ctx[i]->dst_buf_nal_queue));
		}
	}
}

void __mfc_core_dump_state(struct mfc_core *core, int curr_ctx)
{
	nal_queue_handle *nal_q_handle = core->nal_q_handle;
	int i;

	dev_err(core->device, "-----------dumping MFC core info-----------\n");
	dev_err(core->device, "has llc:%d, slc: %d, itmon_notified:%d\n",
			core->has_llc, core->has_slc, core->itmon_notified);
	dev_err(core->device, "power:%d, clock:%d, continue_clock_on:%d, num_inst:%d, num_drm_inst:%d, fw_status normal: %#x, drm: %#x\n",
			mfc_core_pm_get_pwr_ref_cnt(core),
			mfc_core_pm_get_clk_ref_cnt(core),
			core->continue_clock_on, core->num_inst,
			core->num_drm_inst, core->fw.status, core->fw.drm_status);
	dev_err(core->device, "hwlock bits:%#lx / core:%#lx, curr_ctx:%d (is_drm:%d),"
			" preempt_ctx:%d, work_bits:%#lx\n",
			core->hwlock.bits, core->hwlock.dev,
			curr_ctx, core->curr_core_ctx_is_drm,
			core->preempt_core_ctx, mfc_get_bits(&core->work_bits));
	dev_err(core->device, "has 2sysmmu:%d, has hwfc:%d, has_mfc_votf:%d, has_gdc_votf:%d, has_dpu_votf:%d\n",
			core->has_2sysmmu, core->has_hwfc, core->has_mfc_votf,
			core->has_gdc_votf, core->has_dpu_votf);
	dev_err(core->device, "shutdown:%d, sleep:%d, QoS level:%d\n",
			core->shutdown, core->sleep, atomic_read(&core->qos_req_cur) - 1);
	dev_err(core->device, "fw addr %#llx size %08zu drm_fw addr %#llx\n",
			core->fw_buf.daddr, core->fw_buf.size,
			core->drm_fw_buf.daddr);
	if (nal_q_handle)
		dev_err(core->device, "NAL-Q state:%d, exception:%d, in_exe_cnt: %d, out_exe_cnt: %d, stop cause: %#x\n",
				nal_q_handle->nal_q_state,
				nal_q_handle->nal_q_exception,
				nal_q_handle->nal_q_in_handle->in_exe_count,
				nal_q_handle->nal_q_out_handle->out_exe_count,
				core->nal_q_stop_cause);

	for (i = 0; i < MFC_NUM_CONTEXTS; i++)
		if (core->core_ctx[i])
			dev_err(core->device, "- core_ctx[%d]%s state: %d, int(cond:%d, type:%d, err:%d), queue_cnt(src:%d, dst:%d)\n",
				i, curr_ctx == i ? "(curr_ctx!)" : "",
				core->core_ctx[i]->state,
				core->core_ctx[i]->int_condition,
				core->core_ctx[i]->int_reason,
				core->core_ctx[i]->int_err,
				mfc_get_queue_count(&core->core_ctx[i]->buf_queue_lock,
					&core->core_ctx[i]->src_buf_queue),
				mfc_get_queue_count(&core->core_ctx[i]->buf_queue_lock,
					&core->core_ctx[i]->dst_buf_queue));
}

static void __mfc_dump_trace(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;
	int i, cnt, trace_cnt;

	dev_err(core->device, "-----------dumping MFC trace info-----------\n");

	trace_cnt = atomic_read(&dev->trace_ref);
	for (i = MFC_TRACE_COUNT_PRINT - 1; i >= 0; i--) {
		cnt = ((trace_cnt + MFC_TRACE_COUNT_MAX) - i) % MFC_TRACE_COUNT_MAX;
		dev_err(core->device, "MFC trace[%d]: time=%llu, str=%s", cnt,
				dev->mfc_trace[cnt].time, dev->mfc_trace[cnt].str);
	}
}

void __mfc_dump_nal_q_buffer_info(struct mfc_core *core, int curr_ctx)
{
	struct mfc_dev *dev = core->dev;
	struct mfc_ctx *ctx = dev->ctx[curr_ctx];
	nal_queue_in_handle *nal_q_in_handle = core->nal_q_handle->nal_q_in_handle;
	nal_queue_out_handle *nal_q_out_handle = core->nal_q_handle->nal_q_out_handle;
	EncoderInputStr *pEncIn = NULL;
	EncoderOutputStr *pEncOut = NULL;
	DecoderInputStr *pDecIn = NULL;
	DecoderOutputStr *pDecOut = NULL;
	int i, offset, Inindex, cnt;

	Inindex = mfc_core_get_nal_q_input_count() % NAL_Q_QUEUE_SIZE;

	if (ctx->type == MFCINST_DECODER) {
		dev_err(core->device, "Decoder scratch:%#x++%#x, static(vp9):%#x++%#x, MV:++%#lx\n",
				MFC_CORE_READL(MFC_REG_D_SCRATCH_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_D_SCRATCH_BUFFER_SIZE),
				MFC_CORE_READL(MFC_REG_D_STATIC_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_D_STATIC_BUFFER_SIZE), ctx->mv_size);
		dev_err(core->device, "CPB:++%#x, DPB[0]:++%#x, [1]:++%#x, [2]plane:++%#x\n",
				MFC_CORE_READL(MFC_REG_D_CPB_BUFFER_SIZE),
				ctx->raw_buf.plane_size[0], ctx->raw_buf.plane_size[1],
				ctx->raw_buf.plane_size[2]);
		if (ctx->mv_size)
			print_hex_dump(KERN_ERR, "MV buffer ", DUMP_PREFIX_OFFSET, 32, 4,
					core->regs_base + MFC_REG_D_MV_BUFFER0, 0x100, false);
	} else if (ctx->type == MFCINST_ENCODER) {
		dev_err(core->device, "Encoder scratch:%#x++%#x, recon[0]:++%#lx, [1]:++%#lx, ME:++%#lx\n",
				MFC_CORE_READL(MFC_REG_E_SCRATCH_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_E_SCRATCH_BUFFER_SIZE),
				ctx->enc_priv->luma_dpb_size,
				ctx->enc_priv->chroma_dpb_size,
				ctx->enc_priv->me_buffer_size);
		dev_err(core->device, "SRC[0]:++%#x, [1]:++%#x, [2]:++%#x, DST:++%#x\n",
				MFC_CORE_READL(MFC_REG_E_STREAM_BUFFER_SIZE),
				ctx->raw_buf.plane_size[0], ctx->raw_buf.plane_size[1],
				ctx->raw_buf.plane_size[2]);
		print_hex_dump(KERN_ERR, "ME buffer ", DUMP_PREFIX_OFFSET, 32, 4,
				core->regs_base + MFC_REG_E_ME_BUFFER, 0x44, false);
	} else {
		dev_err(core->device, "invalid MFC instance type(%d)\n", ctx->type);
	}

	dev_err(core->device, "-----------dumping NAL queue In(in%d-exe%d), Out(%d)\n",
			mfc_core_get_nal_q_input_count(),
			nal_q_in_handle->in_exe_count,
			mfc_core_get_nal_q_output_count());

	/* Input queue */
	for (i = MFC_TRACE_NAL_QUEUE_PRINT - 1; i >= 0; i--) {
		cnt = ((Inindex + NAL_Q_QUEUE_SIZE) - i) % NAL_Q_QUEUE_SIZE;
		offset = dev->pdata->nal_q_entry_size * cnt;
		pDecIn = (DecoderInputStr *)(nal_q_in_handle->nal_q_in_addr + offset);
		pEncIn = (EncoderInputStr *)(nal_q_in_handle->nal_q_in_addr + offset);

		if (pDecIn->StartCode == NAL_Q_DECODER_MARKER)
			dev_err(core->device, "[%d] In DEC[ctx:%d] id %d cpb %#x used %#x(%d) dpb [0]%#x [1]%#x\n",
					cnt, pDecIn->InstanceId, pDecIn->CommandId,
					pDecIn->CpbBufferAddr, pDecIn->DynamicDpbFlagLower,
					ffs(pDecIn->DynamicDpbFlagLower) - 1,
					pDecIn->FrameAddr[0], pDecIn->FrameAddr[1]);
		else if (pEncIn->StartCode == NAL_Q_ENCODER_MARKER)
			dev_err(core->device, "[%d] In ENC[ctx:%d] id %d frame [0]%#x [1]%#x stream %#x\n",
					cnt, pEncIn->InstanceId, pEncIn->CommandId,
					pEncIn->FrameAddr[0], pEncIn->FrameAddr[1],
					pEncIn->StreamBufferAddr);
	}

	/* Output queue */
	for (i = MFC_TRACE_NAL_QUEUE_PRINT - 1; i >= 0; i--) {
		cnt = ((Inindex + NAL_Q_QUEUE_SIZE) - i) % NAL_Q_QUEUE_SIZE;
		offset = dev->pdata->nal_q_entry_size * cnt;
		pDecOut = (DecoderOutputStr *)(nal_q_out_handle->nal_q_out_addr + offset);
		pEncOut = (EncoderOutputStr *)(nal_q_out_handle->nal_q_out_addr + offset);

		if (pDecOut->StartCode == NAL_Q_DECODER_MARKER)
			dev_err(core->device, "[%d] Out DEC[ctx:%d] id %d disp stat %#x disp [0]%#x [1]%#x used %#x dec stat %#x dec [0]%#x [1]%#x type %d\n",
					cnt, pDecOut->InstanceId, pDecOut->CommandId,
					pDecOut->DisplayStatus, pDecOut->DisplayAddr[0],
					pDecOut->DisplayAddr[1], pDecOut->UsedDpbFlagLower,
					pDecOut->DecodedStatus, pDecOut->DecodedAddr[0],
					pDecOut->DecodedAddr[1], pDecOut->DecodedFrameType);
		else if (pEncOut->StartCode == NAL_Q_ENCODER_MARKER)
			dev_err(core->device, "[%d] Out ENC[ctx:%d] id %d frame [0]%#x [1]%#x stream %#x type %d recon [0]%#x [1]%#x\n",
					cnt, pEncIn->InstanceId, pEncOut->CommandId,
					pEncOut->EncodedFrameAddr[0], pEncOut->EncodedFrameAddr[1],
					pEncOut->StreamBufferAddr, pEncOut->SliceType,
					pEncOut->ReconLumaDpbAddr, pEncOut->ReconChromaDpbAddr);
	}
}

void __mfc_dump_buffer_info(struct mfc_core *core)
{
	int curr_ctx = __mfc_get_curr_ctx(core);
	struct mfc_ctx *ctx;
	struct mfc_core_ctx *core_ctx;

	dev_err(core->device, "-----------dumping MFC buffer info (fault at: %#x)\n",
			core->logging_data->fault_addr);
	dev_err(core->device, "Normal FW:%llx~%#llx (common ctx buf:%#llx~%#llx)\n",
			core->fw_buf.daddr,
			core->fw_buf.daddr + core->fw_buf.size,
			core->common_ctx_buf.daddr,
			core->common_ctx_buf.daddr + PAGE_ALIGN(0x7800));
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	dev_err(core->device, "Secure FW:%llx~%#llx (common ctx buf:%#llx~%#llx)\n",
			core->drm_fw_buf.daddr,
			core->drm_fw_buf.daddr + core->drm_fw_buf.size,
			core->drm_common_ctx_buf.daddr,
			core->drm_common_ctx_buf.daddr + PAGE_ALIGN(0x7800));
#endif

	if (curr_ctx < 0)
		return;

	core_ctx = core->core_ctx[curr_ctx];
	ctx = core_ctx->ctx;

	dev_err(core->device, "instance buf:%#llx~%#llx, codec buf:%#llx~%#llx\n",
			core_ctx->instance_ctx_buf.daddr,
			core_ctx->instance_ctx_buf.daddr + core_ctx->instance_ctx_buf.size,
			core_ctx->codec_buf.daddr,
			core_ctx->codec_buf.daddr + core_ctx->codec_buf.size);

	if (core->nal_q_handle && (core->nal_q_handle->nal_q_state == NAL_Q_STATE_STARTED)) {
		__mfc_dump_nal_q_buffer_info(core, curr_ctx);
		return;
	}

	if (ctx->type == MFCINST_DECODER) {
		dev_err(core->device, "Decoder CPB:%#x++%#x, scratch:%#x++%#x, static(vp9):%#x++%#x\n",
				MFC_CORE_READL(MFC_REG_D_CPB_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_D_CPB_BUFFER_SIZE),
				MFC_CORE_READL(MFC_REG_D_SCRATCH_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_D_SCRATCH_BUFFER_SIZE),
				MFC_CORE_READL(MFC_REG_D_STATIC_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_D_STATIC_BUFFER_SIZE));
		dev_err(core->device, "DPB [0]plane:++%#x, [1]plane:++%#x, [2]plane:++%#x, MV buffer:++%#lx\n",
				ctx->raw_buf.plane_size[0], ctx->raw_buf.plane_size[1],
				ctx->raw_buf.plane_size[2], ctx->mv_size);
		print_hex_dump(KERN_ERR, "[0] plane ", DUMP_PREFIX_OFFSET, 32, 4,
				core->regs_base + MFC_REG_D_FIRST_PLANE_DPB0, 0x100, false);
		print_hex_dump(KERN_ERR, "[1] plane ", DUMP_PREFIX_OFFSET, 32, 4,
				core->regs_base + MFC_REG_D_SECOND_PLANE_DPB0, 0x100, false);
		if (ctx->dst_fmt->num_planes == 3)
			print_hex_dump(KERN_ERR, "[2] plane ", DUMP_PREFIX_OFFSET, 32, 4,
					core->regs_base + MFC_REG_D_THIRD_PLANE_DPB0,
					0x100, false);
		if (ctx->mv_size)
			print_hex_dump(KERN_ERR, "MV buffer ", DUMP_PREFIX_OFFSET, 32, 4,
					core->regs_base + MFC_REG_D_MV_BUFFER0, 0x100, false);
	} else if (ctx->type == MFCINST_ENCODER) {
		dev_err(core->device, "Encoder SRC %dplane, [0]:%#x++%#x, [1]:%#x++%#x, [2]:%#x++%#x\n",
				ctx->src_fmt->num_planes,
				MFC_CORE_READL(MFC_REG_E_SOURCE_FIRST_ADDR),
				ctx->raw_buf.plane_size[0],
				MFC_CORE_READL(MFC_REG_E_SOURCE_SECOND_ADDR),
				ctx->raw_buf.plane_size[1],
				MFC_CORE_READL(MFC_REG_E_SOURCE_THIRD_ADDR),
				ctx->raw_buf.plane_size[2]);
		dev_err(core->device, "DST:%#x++%#x, scratch:%#x++%#x\n",
				MFC_CORE_READL(MFC_REG_E_STREAM_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_E_STREAM_BUFFER_SIZE),
				MFC_CORE_READL(MFC_REG_E_SCRATCH_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_E_SCRATCH_BUFFER_SIZE));
		dev_err(core->device, "recon [0] plane:++%#lx, [1] plane:++%#lx, ME buffer:++%#lx\n",
				ctx->enc_priv->luma_dpb_size,
				ctx->enc_priv->chroma_dpb_size,
				ctx->enc_priv->me_buffer_size);
		print_hex_dump(KERN_ERR, "[0] plane ", DUMP_PREFIX_OFFSET, 32, 4,
				core->regs_base + MFC_REG_E_LUMA_DPB, 0x44, false);
		print_hex_dump(KERN_ERR, "[1] plane ", DUMP_PREFIX_OFFSET, 32, 4,
				core->regs_base + MFC_REG_E_CHROMA_DPB, 0x44, false);
		print_hex_dump(KERN_ERR, "ME buffer ", DUMP_PREFIX_OFFSET, 32, 4,
				core->regs_base + MFC_REG_E_ME_BUFFER, 0x44, false);
	} else {
		dev_err(core->device, "invalid MFC instnace type(%d)\n", ctx->type);
	}
}

static void __mfc_dump_dpb(struct mfc_core *core, int curr_ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[curr_ctx];
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_buf *mfc_buf = NULL;
	int i, found, in_nal_q;

	if (ctx->type != MFCINST_DECODER || dec == NULL)
		return;

	dev_err(core->device, "-----------dumping MFC DPB queue-----------\n");
	if (!list_empty(&ctx->dst_buf_queue.head))
		list_for_each_entry(mfc_buf, &ctx->dst_buf_queue.head, list)
			dev_err(core->device, "dst[%d][%d] %pad used: %d\n",
					mfc_buf->vb.vb2_buf.index, mfc_buf->dpb_index,
					&mfc_buf->addr[0][0], mfc_buf->used);
	if (!list_empty(&ctx->dst_buf_nal_queue.head))
		list_for_each_entry(mfc_buf, &ctx->dst_buf_nal_queue.head, list)
			dev_err(core->device, "dst_nal[%d][%d] %pad used: %d\n",
					mfc_buf->vb.vb2_buf.index, mfc_buf->dpb_index,
					&mfc_buf->addr[0][0], mfc_buf->used);

	dev_err(core->device, "-----------dumping MFC DPB table-----------\n");
	dev_err(core->device, "dynamic_used: %#lx, queued: %#lx, table_used: %#lx, dynamic_set: %#lx(dec: %#lx)\n",
			dec->dynamic_used, dec->queued_dpb, dec->dpb_table_used,
			core_ctx->dynamic_set, dec->dynamic_set);
	for (i = 0; i < MFC_MAX_DPBS; i++) {
		found = 0;
		in_nal_q = 0;
		list_for_each_entry(mfc_buf, &ctx->dst_buf_queue.head, list) {
			if (i == mfc_buf->dpb_index) {
				found = 1;
				break;
			}
		}
		if (!found) {
			list_for_each_entry(mfc_buf, &ctx->dst_buf_nal_queue.head, list) {
				if (i == mfc_buf->dpb_index) {
					found = 1;
					in_nal_q = 1;
					break;
				}
			}
		}
		dev_err(core->device, "[%d] dpb [%d] %pad %pad %pap fd %d(%d) (%s, %s, %s%s)\n",
				i, found ? mfc_buf->vb.vb2_buf.index : -1,
				&dec->dpb[i].addr[0], &dec->dpb[i].addr[1],
				&dec->dpb[i].paddr, dec->dpb[i].fd[0], dec->dpb[i].new_fd,
				dec->dpb[i].mapcnt ? "map" : "unmap",
				dec->dpb[i].ref ? "ref" : "free",
				dec->dpb[i].queued ? "Q" : "DQ",
				in_nal_q ? " in NALQ" : "");
	}
}

static void __mfc_dump_info_without_regs(struct mfc_core *core)
{
	int curr_ctx = __mfc_get_curr_ctx(core);

	mfc_dump_state(core->dev);
	__mfc_core_dump_state(core, curr_ctx);
	__mfc_dump_trace(core);

	/* If there was no struct mfc_ctx, skip access the *ctx */
	if (curr_ctx < 0)
		return;

	__mfc_dump_dpb(core, curr_ctx);
}

static void __mfc_dump_info(struct mfc_core *core)
{
	__mfc_dump_info_without_regs(core);

	if (!mfc_core_pm_get_pwr_ref_cnt(core)) {
		dev_err(core->device, "Power(%d) is not enabled\n",
				mfc_core_pm_get_pwr_ref_cnt(core));
		return;
	}

	__mfc_dump_buffer_info(core);
	__mfc_dump_regs(core);
	__mfc_save_logging_sfr(core);
}

static void __mfc_store_dump_buf(char *buf, int *idx, int size, const char *fmt, ...)
{
	va_list args;
	int ret;

	if ((*idx + size) > MFC_DUMP_BUF_SIZE)
		return;

	va_start(args, fmt);
	ret = vsnprintf((buf + *idx), size, fmt, args);
	va_end(args);

	*idx += ret;
}

static int __mfc_store_dump_state(struct mfc_core *core, int idx)
{
	struct mfc_dev *dev = core->dev;
	char *buf = core->dbg_info.addr;
	int i;

	__mfc_store_dump_buf(buf, &idx, 50, "\n-----------dumping MFC device info-----------\n");
	__mfc_store_dump_buf(buf, &idx, 100,
			"options debug_level:%d, debug_mode:%d (%d), perf_boost:%d, wait_fw_status %d\n",
			debug_level, dev->pdata->debug_mode, debug_mode_en,
			perf_boost_mode, dev->pdata->wait_fw_status.support);

	for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
		if (dev->ctx[i]) {
			__mfc_store_dump_buf(buf, &idx, 300,
			"- ctx[%d] %s %s, %s, %s, size: %dx%d@%ldfps(op: %ldfps), crop: %d %d %d %d\n",
				dev->ctx[i]->num,
				dev->ctx[i]->type == MFCINST_DECODER ? "DEC" : "ENC",
				dev->ctx[i]->is_drm ? "Secure" : "Normal",
				dev->ctx[i]->src_fmt->name,
				dev->ctx[i]->dst_fmt->name,
				dev->ctx[i]->img_width, dev->ctx[i]->img_height,
				dev->ctx[i]->last_framerate / 1000,
				dev->ctx[i]->operating_framerate,
				dev->ctx[i]->crop_width, dev->ctx[i]->crop_height,
				dev->ctx[i]->crop_left, dev->ctx[i]->crop_top);
			__mfc_store_dump_buf(buf, &idx, 300,
			"\tmain core-%d, op_mode: %d, queue_cnt(src:%d, dst:%d, ref:%d, qsrc:%d, qdst:%d)\n",
				dev->ctx[i]->op_core_num[MFC_CORE_MAIN],
				dev->ctx[i]->op_mode,
				mfc_get_queue_count(&dev->ctx[i]->buf_queue_lock, &dev->ctx[i]->src_buf_ready_queue),
				mfc_get_queue_count(&dev->ctx[i]->buf_queue_lock, &dev->ctx[i]->dst_buf_queue),
				mfc_get_queue_count(&dev->ctx[i]->buf_queue_lock, &dev->ctx[i]->ref_buf_queue),
				mfc_get_queue_count(&dev->ctx[i]->buf_queue_lock, &dev->ctx[i]->src_buf_nal_queue),
				mfc_get_queue_count(&dev->ctx[i]->buf_queue_lock, &dev->ctx[i]->dst_buf_nal_queue));
		}
	}

	return idx;
}

static int __mfc_store_dump_trace(struct mfc_core *core, int idx)
{
	struct mfc_dev *dev = core->dev;
	char *buf = core->dbg_info.addr;
	int i, cnt, trace_cnt;

	__mfc_store_dump_buf(buf, &idx, 50, "\n-----------dumping MFC trace info-----------\n");

	trace_cnt = atomic_read(&dev->trace_ref);
	for (i = MFC_TRACE_COUNT_PRINT_LONG - 1; i >= 0; i--) {
		cnt = ((trace_cnt + MFC_TRACE_COUNT_MAX) - i) % MFC_TRACE_COUNT_MAX;
		__mfc_store_dump_buf(buf, &idx, 150, "MFC trace[%d]: time=%llu, str=%s", cnt,
				dev->mfc_trace[cnt].time, dev->mfc_trace[cnt].str);
	}

	return idx;
}

static void __mfc_store_dump_hex(char *buf, int *idx, void *data_buf, size_t len)
{
	const u8 *ptr = data_buf;
	int i, linelen, remaining = len;
	int rowsize = 32, groupsize = 4;
	int ret;

	for (i = 0; i < len; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;

		if ((*idx + linelen + 15) > MFC_DUMP_BUF_SIZE)
			return;

		ret = snprintf((buf + *idx), 12, "%.8x: ", i);
		ret += hex_dump_to_buffer((ptr + i), linelen, rowsize, groupsize,
				(buf + *idx + ret), (rowsize * groupsize + 3), false);
		ret += snprintf((buf + *idx + ret), 3, "\n");

		*idx += ret;
	}
}

static int __mfc_store_dump_nal_q_buffer_info(struct mfc_core *core, int curr_ctx, int idx)
{
	char *buf = core->dbg_info.addr;
	struct mfc_dev *dev = core->dev;
	struct mfc_ctx *ctx = dev->ctx[curr_ctx];
	nal_queue_in_handle *nal_q_in_handle = core->nal_q_handle->nal_q_in_handle;
	nal_queue_out_handle *nal_q_out_handle = core->nal_q_handle->nal_q_out_handle;
	EncoderInputStr *pEncIn = NULL;
	EncoderOutputStr *pEncOut = NULL;
	DecoderInputStr *pDecIn = NULL;
	DecoderOutputStr *pDecOut = NULL;
	int i, offset, Inindex, cnt;

	Inindex = mfc_core_get_nal_q_input_count() % NAL_Q_QUEUE_SIZE;

	if (ctx->type == MFCINST_DECODER) {
		__mfc_store_dump_buf(buf, &idx, 100, "Decoder scratch:%#x++%#x, static(vp9):%#x++%#x, MV:++%#lx\n",
				MFC_CORE_READL(MFC_REG_D_SCRATCH_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_D_SCRATCH_BUFFER_SIZE),
				MFC_CORE_READL(MFC_REG_D_STATIC_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_D_STATIC_BUFFER_SIZE), ctx->mv_size);
		__mfc_store_dump_buf(buf, &idx, 100, "CPB:++%#x, DPB[0]:++%#x, [1]:++%#x, [2]plane:++%#x\n",
				MFC_CORE_READL(MFC_REG_D_CPB_BUFFER_SIZE),
				ctx->raw_buf.plane_size[0], ctx->raw_buf.plane_size[1],
				ctx->raw_buf.plane_size[2]);
		if (ctx->mv_size) {
			__mfc_store_dump_buf(buf, &idx, 25, "MV buffer\n");
			__mfc_store_dump_hex(buf, &idx,
					(core->regs_base + MFC_REG_D_MV_BUFFER0), 0x100);
		}
		__mfc_store_dump_buf(buf, &idx, 300, "NALQ In(in%d-exe%d): ID CPB DPBFlag DPB0 DPB1\
		/ Out(%d): ID dispstat diap0 disp1 used decstat dec0 dec1 type\n",
				mfc_core_get_nal_q_input_count(),
				nal_q_in_handle->in_exe_count,
				mfc_core_get_nal_q_output_count());
		for (i = MFC_TRACE_NAL_QUEUE_PRINT - 1; i >= 0; i--) {
			cnt = ((Inindex + NAL_Q_QUEUE_SIZE) - i) % NAL_Q_QUEUE_SIZE;
			offset = dev->pdata->nal_q_entry_size * cnt;
			pDecIn = (DecoderInputStr *)(nal_q_in_handle->nal_q_in_addr + offset);
			pDecOut = (DecoderOutputStr *)(nal_q_out_handle->nal_q_out_addr + offset);
			__mfc_store_dump_buf(buf, &idx, 300,
					"[%d] In: %d %x %x[%d] %x %x / Out: %d %d %x %x %x %d %x %x %d\n",
					cnt, pDecIn->CommandId, pDecIn->CpbBufferAddr,
					pDecIn->DynamicDpbFlagLower, ffs(pDecIn->DynamicDpbFlagLower) - 1,
					pDecIn->FrameAddr[0], pDecIn->FrameAddr[1],
					pDecOut->CommandId, pDecOut->DisplayStatus,
					pDecOut->DisplayAddr[0], pDecOut->DisplayAddr[1],
					pDecOut->UsedDpbFlagLower, pDecOut->DecodedStatus,
					pDecOut->DecodedAddr[0], pDecOut->DecodedAddr[1],
					pDecOut->DecodedFrameType);
		}
	} else if (ctx->type == MFCINST_ENCODER) {
		__mfc_store_dump_buf(buf, &idx, 300,
				"Encoder scratch:%#x++%#x, recon[0]:++%#lx, [1]:++%#lx, ME:++%#lx\n",
				MFC_CORE_READL(MFC_REG_E_SCRATCH_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_E_SCRATCH_BUFFER_SIZE),
				ctx->enc_priv->luma_dpb_size,
				ctx->enc_priv->chroma_dpb_size,
				ctx->enc_priv->me_buffer_size);
		__mfc_store_dump_buf(buf, &idx, 300, "SRC[0]:++%#x, [1]:++%#x, [2]:++%#x, DST:++%#x\n",
				MFC_CORE_READL(MFC_REG_E_STREAM_BUFFER_SIZE),
				ctx->raw_buf.plane_size[0], ctx->raw_buf.plane_size[1],
				ctx->raw_buf.plane_size[2]);
		__mfc_store_dump_buf(buf, &idx, 25, "ME buffer\n");
		__mfc_store_dump_hex(buf, &idx,
				(core->regs_base + MFC_REG_E_ME_BUFFER), 0x44);
		__mfc_store_dump_buf(buf, &idx, 300, "NALQ In: ID src0 src1 dst\
			/ Out: ID enc0 enc1 strm recon0 recon1 type\n");
		for (i = MFC_TRACE_NAL_QUEUE_PRINT - 1; i >= 0; i--) {
			cnt = ((Inindex + NAL_Q_QUEUE_SIZE) - i) % NAL_Q_QUEUE_SIZE;
			offset = dev->pdata->nal_q_entry_size * cnt;
			pEncIn = (EncoderInputStr *)(nal_q_in_handle->nal_q_in_addr + offset);
			pEncOut = (EncoderOutputStr *)(nal_q_out_handle->nal_q_out_addr + offset);
			__mfc_store_dump_buf(buf, &idx, 300,
					"[%d] In: %d %#x %#x %#x / Out: %d %#x %#x %#x %d %#x %#x\n",
					cnt, pEncIn->CommandId, pEncIn->FrameAddr[0],
					pEncIn->FrameAddr[1], pEncIn->StreamBufferAddr,
					pEncOut->CommandId, pEncOut->EncodedFrameAddr[0],
					pEncOut->EncodedFrameAddr[1], pEncOut->StreamBufferAddr,
					pEncOut->SliceType, pEncOut->ReconLumaDpbAddr,
					pEncOut->ReconChromaDpbAddr);
		}
	}

	__mfc_store_dump_buf(buf, &idx, 300,
			"-----------dumping NAL queue buf In(in%d-exe%d), Out(%d)\n",
			mfc_core_get_nal_q_input_count(),
			nal_q_in_handle->in_exe_count,
			mfc_core_get_nal_q_output_count());

	/* Input queue */
	for (i = MFC_TRACE_NAL_QUEUE_PRINT - 1; i >= 0; i--) {
		cnt = ((Inindex + NAL_Q_QUEUE_SIZE) - i) % NAL_Q_QUEUE_SIZE;
		offset = dev->pdata->nal_q_entry_size * cnt;
		pDecIn = (DecoderInputStr *)(nal_q_in_handle->nal_q_in_addr + offset);
		pEncIn = (EncoderInputStr *)(nal_q_in_handle->nal_q_in_addr + offset);
		if (pDecIn->StartCode == NAL_Q_DECODER_MARKER)
			__mfc_store_dump_buf(buf, &idx, 300,
					"[%d] In DEC[ctx:%d] id %d cpb %#x used %#x[%d] dpb[0] %#x [1] %#x\n",
					cnt, pDecIn->InstanceId, pDecIn->CommandId,
					pDecIn->CpbBufferAddr, pDecIn->DynamicDpbFlagLower,
					ffs(pDecIn->DynamicDpbFlagLower) - 1,
					pDecIn->FrameAddr[0], pDecIn->FrameAddr[1]);
		else if (pEncIn->StartCode == NAL_Q_ENCODER_MARKER)
			__mfc_store_dump_buf(buf, &idx, 300,
					"[%d] In ENC[ctx:%d] id %d frame[0] %#x [1] %#x stream %#x\n",
					cnt, pEncIn->InstanceId, pEncIn->CommandId,
					pEncIn->FrameAddr[0], pEncIn->FrameAddr[1],
					pEncIn->StreamBufferAddr);
	}

	/* Output queue */
	for (i = MFC_TRACE_NAL_QUEUE_PRINT - 1; i >= 0; i--) {
		cnt = ((Inindex + NAL_Q_QUEUE_SIZE) - i) % NAL_Q_QUEUE_SIZE;
		offset = dev->pdata->nal_q_entry_size * cnt;
		pDecOut = (DecoderOutputStr *)(nal_q_out_handle->nal_q_out_addr + offset);
		pEncOut = (EncoderOutputStr *)(nal_q_out_handle->nal_q_out_addr + offset);

		if (pDecOut->StartCode == NAL_Q_DECODER_MARKER)
			__mfc_store_dump_buf(buf, &idx, 300,
					"[%d] Out DEC[ctx:%d] id %d disp stat %#x disp[0] %#x [1] %#x used %#x dec stat %#x dec[0] %#x [1] %x type %d\n",
					cnt, pDecOut->InstanceId, pDecOut->CommandId,
					pDecOut->DisplayStatus, pDecOut->DisplayAddr[0],
					pDecOut->DisplayAddr[1], pDecOut->UsedDpbFlagLower,
					pDecOut->DecodedStatus, pDecOut->DecodedAddr[0],
					pDecOut->DecodedAddr[1], pDecOut->DecodedFrameType);
		else if (pEncOut->StartCode == NAL_Q_ENCODER_MARKER)
			__mfc_store_dump_buf(buf, &idx, 300,
					"[%d] Out ENC[ctx:%d] id %d frame[0] %#x [1] %#x stream %#x type %d recon[0] %#x [1] %#x\n",
					cnt, pEncIn->InstanceId, pEncOut->CommandId,
					pEncOut->EncodedFrameAddr[0], pEncOut->EncodedFrameAddr[1],
					pEncOut->StreamBufferAddr, pEncOut->SliceType,
					pEncOut->ReconLumaDpbAddr, pEncOut->ReconChromaDpbAddr);
	}

	return idx;
}

static int __mfc_store_dump_buffer_info(struct mfc_core *core, int idx)
{
	char *buf = core->dbg_info.addr;
	int curr_ctx = __mfc_get_curr_ctx(core);
	struct mfc_ctx *ctx;
	struct mfc_core_ctx *core_ctx;

	__mfc_store_dump_buf(buf, &idx, 100, "\n-----------dumping MFC buffer info (fault at: %#x)\n",
			core->logging_data->fault_addr);
	__mfc_store_dump_buf(buf, &idx, 100, "Normal FW:%llx~%#llx (common ctx buf:%#llx~%#llx)\n",
			core->fw_buf.daddr,
			core->fw_buf.daddr + core->fw_buf.size,
			core->common_ctx_buf.daddr,
			core->common_ctx_buf.daddr + PAGE_ALIGN(0x7800));
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	__mfc_store_dump_buf(buf, &idx, 100, "Secure FW:%llx~%#llx (common ctx buf:%#llx~%#llx)\n",
			core->drm_fw_buf.daddr,
			core->drm_fw_buf.daddr + core->drm_fw_buf.size,
			core->drm_common_ctx_buf.daddr,
			core->drm_common_ctx_buf.daddr + PAGE_ALIGN(0x7800));
#endif

	if (curr_ctx < 0)
		return idx;

	core_ctx = core->core_ctx[curr_ctx];
	ctx = core_ctx->ctx;

	__mfc_store_dump_buf(buf, &idx, 100, "instance buf:%#llx~%#llx, codec buf:%#llx~%#llx\n",
			core_ctx->instance_ctx_buf.daddr,
			core_ctx->instance_ctx_buf.daddr + core_ctx->instance_ctx_buf.size,
			core_ctx->codec_buf.daddr,
			core_ctx->codec_buf.daddr + core_ctx->codec_buf.size);

	if (core->nal_q_handle && (core->nal_q_handle->nal_q_state == NAL_Q_STATE_STARTED)) {
		idx += __mfc_store_dump_nal_q_buffer_info(core, curr_ctx, idx);
		return idx;
	}

	if (ctx->type == MFCINST_DECODER) {
		__mfc_store_dump_buf(buf, &idx, 300,
				"Decoder CPB:%#x++%#x, scratch:%#x++%#x, static(vp9):%#x++%#x\n",
				MFC_CORE_READL(MFC_REG_D_CPB_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_D_CPB_BUFFER_SIZE),
				MFC_CORE_READL(MFC_REG_D_SCRATCH_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_D_SCRATCH_BUFFER_SIZE),
				MFC_CORE_READL(MFC_REG_D_STATIC_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_D_STATIC_BUFFER_SIZE));
		__mfc_store_dump_buf(buf, &idx, 300,
				"DPB [0]plane:++%#x, [1]plane:++%#x, [2]plane:++%#x, MV buffer:++%#lx\n",
				ctx->raw_buf.plane_size[0], ctx->raw_buf.plane_size[1],
				ctx->raw_buf.plane_size[2], ctx->mv_size);
		__mfc_store_dump_buf(buf, &idx, 25, "[0] plane:\n");
		__mfc_store_dump_hex(buf, &idx,
				(core->regs_base + MFC_REG_D_FIRST_PLANE_DPB0), 0x100);
		__mfc_store_dump_buf(buf, &idx, 25, "[1] plane:\n");
		__mfc_store_dump_hex(buf, &idx,
				(core->regs_base + MFC_REG_D_SECOND_PLANE_DPB0), 0x100);

		if (ctx->dst_fmt->num_planes == 3) {
			__mfc_store_dump_buf(buf, &idx, 25, "[2] plane:\n");
			__mfc_store_dump_hex(buf, &idx,
					(core->regs_base + MFC_REG_D_THIRD_PLANE_DPB0), 0x100);
		}
		if (ctx->mv_size) {
			__mfc_store_dump_buf(buf, &idx, 25, "MV buffer\n");
			__mfc_store_dump_hex(buf, &idx,
					(core->regs_base + MFC_REG_D_MV_BUFFER0), 0x100);
		}
	} else if (ctx->type == MFCINST_ENCODER) {
		__mfc_store_dump_buf(buf, &idx, 300,
				"Encoder SRC %dplane, [0]:%#x++%#x, [1]:%#x++%#x, [2]:%#x++%#x\n",
				ctx->src_fmt->num_planes,
				MFC_CORE_READL(MFC_REG_E_SOURCE_FIRST_ADDR),
				ctx->raw_buf.plane_size[0],
				MFC_CORE_READL(MFC_REG_E_SOURCE_SECOND_ADDR),
				ctx->raw_buf.plane_size[1],
				MFC_CORE_READL(MFC_REG_E_SOURCE_THIRD_ADDR),
				ctx->raw_buf.plane_size[2]);
		__mfc_store_dump_buf(buf, &idx, 300, "DST:%#x++%#x, scratch:%#x++%#x\n",
				MFC_CORE_READL(MFC_REG_E_STREAM_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_E_STREAM_BUFFER_SIZE),
				MFC_CORE_READL(MFC_REG_E_SCRATCH_BUFFER_ADDR),
				MFC_CORE_READL(MFC_REG_E_SCRATCH_BUFFER_SIZE));
		__mfc_store_dump_buf(buf, &idx, 300,
				"recon [0] plane:++%#lx, [1] plane:++%#lx, ME buffer:++%#lx\n",
				ctx->enc_priv->luma_dpb_size,
				ctx->enc_priv->chroma_dpb_size,
				ctx->enc_priv->me_buffer_size);

		__mfc_store_dump_buf(buf, &idx, 25, "[0] plane:\n");
		__mfc_store_dump_hex(buf, &idx,
				(core->regs_base + MFC_REG_E_LUMA_DPB), 0x44);

		__mfc_store_dump_buf(buf, &idx, 25, "[1] plane:\n");
		__mfc_store_dump_hex(buf, &idx,
				(core->regs_base + MFC_REG_E_CHROMA_DPB), 0x44);

		__mfc_store_dump_buf(buf, &idx, 25, "ME buffer\n");
		__mfc_store_dump_hex(buf, &idx,
				(core->regs_base + MFC_REG_E_ME_BUFFER), 0x44);
	}

	return idx;
}

static int __mfc_store_dump_regs(struct mfc_core *core, int idx)
{
	char *buf = core->dbg_info.addr;
	int i;
	int addr[MFC_SFR_AREA_COUNT][2] = {
		{ 0x0, 0x80 },
		{ 0x1000, 0xCD0 },
		{ 0xF000, 0xFF8 },
		{ 0x2000, 0xA00 },
		{ 0x2f00, 0x6C },
		{ 0x3000, 0x40 },
		{ 0x3094, 0x4 },
		{ 0x30b4, 0x8 },
		{ 0x3110, 0x10 },
		{ 0x5000, 0x130 },
		{ 0x5200, 0x300 },
		{ 0x5600, 0x100 },
		{ 0x5800, 0x100 },
		{ 0x5A00, 0x100 },
		{ 0x6000, 0x188 },
		{ 0x7000, 0x21C },
		{ 0x7300, 0xFC },
		{ 0x8000, 0x20C },
		{ 0x9000, 0x10C },
		{ 0xA000, 0x500 },
		{ 0xB000, 0x444 },
		{ 0xC000, 0x84 },
	};
	nal_queue_handle *nal_q_handle = core->nal_q_handle;
	struct mfc_core_ctx *core_ctx = NULL;
	size_t buf_size;
	int curr_ctx;

	if (!mfc_core_pm_get_pwr_ref_cnt(core)) {
		dev_err(core->device, "Power(%d) is not enabled\n",
				mfc_core_pm_get_pwr_ref_cnt(core));
		return 0;
	}

	mfc_core_enable_all_clocks(core);

	__mfc_store_dump_buf(buf, &idx, 50, "\n-----------dumping MFC0 registers-----------\n");

	for (i = 0; i < MFC_SFR_AREA_COUNT; i++) {
		__mfc_store_dump_buf(buf, &idx, 25, "[%04X .. %04X]\n", addr[i][0], addr[i][0] + addr[i][1]);
		__mfc_store_dump_hex(buf, &idx, (core->regs_base + addr[i][0]), addr[i][1]);
		__mfc_store_dump_buf(buf, &idx, 5, "...\n");
	}

	/* dump COMMON context */
	if (core->common_ctx_buf.vaddr) {
		buf_size = core->common_ctx_buf.size;
		__mfc_store_dump_buf(buf, &idx, 50,
				     "\n-----------dumping MFC common ctx-----------\n");
		__mfc_store_dump_hex(buf, &idx, core->common_ctx_buf.vaddr, buf_size);
		__mfc_store_dump_buf(buf, &idx, 5, "...\n");
	}

	/* dump INSTANCE context */
	curr_ctx = __mfc_get_curr_ctx(core);
	if (curr_ctx < 0)
		goto nal_q;
	core_ctx = core->core_ctx[curr_ctx];
	if (core_ctx->instance_ctx_buf.vaddr) {
		buf_size = core_ctx->instance_ctx_buf.size;
		__mfc_store_dump_buf(buf, &idx, 50,
				     "\n-----------dumping MFC inst ctx-----------\n");
		__mfc_store_dump_hex(buf, &idx, core_ctx->instance_ctx_buf.vaddr, buf_size);
		__mfc_store_dump_buf(buf, &idx, 5, "...\n");
	}

nal_q:
	if (nal_q_handle) {
		if (nal_q_handle->nal_q_in_handle && nal_q_handle->nal_q_in_handle->in_buf.vaddr) {
			buf_size = nal_q_handle->nal_q_in_handle->in_buf.size;
			__mfc_store_dump_buf(buf, &idx, 50, "\n-----------dumping MFC NALQ in----------\n");
			__mfc_store_dump_hex(buf, &idx,
					nal_q_handle->nal_q_in_handle->in_buf.vaddr, buf_size);
			__mfc_store_dump_buf(buf, &idx, 5, "...\n");
		}
		if (nal_q_handle->nal_q_out_handle &&
		    nal_q_handle->nal_q_out_handle->out_buf.vaddr) {
			buf_size = nal_q_handle->nal_q_out_handle->out_buf.size;
			__mfc_store_dump_buf(buf, &idx, 50, "\n-----------dumping MFC NALQ out----------\n");
			__mfc_store_dump_hex(buf, &idx,
					nal_q_handle->nal_q_out_handle->out_buf.vaddr, buf_size);
			__mfc_store_dump_buf(buf, &idx, 5, "...\n");
		}
	}
	return idx;
}

static int __mfc_store_debug_info(struct mfc_core *core)
{
	char *buf = core->dbg_info.addr;
	int idx = 0;

	if (!buf)
		return -EINVAL;

	dev_err(core->device, "[COREDUMP] -----------store MFC info\n");
	idx += __mfc_store_dump_state(core, idx);
	idx += __mfc_store_dump_trace(core, idx);
	idx += __mfc_store_dump_buffer_info(core, idx);
	idx += __mfc_store_dump_regs(core, idx);

	return idx;
}

static int __mfc_do_sscoredump(struct mfc_core *core)
{
#ifdef CONFIG_MFC_USE_COREDUMP
	struct sscd_platform_data *sscd_platdata;
	struct sscd_segment seg;
	int ret;

	if (!core->sscd_dev)
		return -EINVAL;

	mfc_core_info("[SSCD] ---------------MFC core dump\n");
	sscd_platdata = dev_get_platdata(&core->sscd_dev->dev);
	if (!sscd_platdata->sscd_report) {
		mfc_core_err("[SSCD] there is no sscd_report\n");
		return -EINVAL;
	}

	memset(&seg, 0, sizeof(seg));
	seg.addr = (void *)core->dbg_info.addr;
	seg.size = core->dbg_info.size;
	seg.flags = 0;

	ret = sscd_platdata->sscd_report(core->sscd_dev, &seg, 1,
		SSCD_FLAGS_ELFARM64HDR, core->crash_info);

	return ret;
#else
	return -EINVAL;
#endif
}

static void __mfc_dump_info_and_stop_hw(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;
	struct mfc_core *main_core;
	struct mfc_ctx *ctx;
	int curr_ctx = __mfc_get_curr_ctx(core);
	bool itmon_notified = core->itmon_notified;

	MFC_TRACE_CORE("** %s will stop!!!\n", core->name);

	/* dump issued core first */
	__mfc_dump_info(core);

	/* Broadcast meerkat to multi mode core */
	if (curr_ctx < 0)
		goto panic;

	ctx = dev->ctx[curr_ctx];
	if (!IS_SINGLE_MODE(ctx)) {
		dev_err(dev->device, "[2CORE] there is multi core mode (op mode: %d)\n",
				ctx->op_mode);
		main_core = mfc_get_main_core(dev, ctx);
		if (core == main_core)
			/* issued maincore, dump subcore also */
			core = mfc_get_sub_core(dev, ctx);
		else
			/* issued subcore, dump maincore also */
			core = main_core;
		__mfc_dump_info(core);
	}

	if (dev->otf_inst_bits) {
		dev_err(dev->device, "-----------dumping TS-MUX info-----------\n");
#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
		tsmux_sfr_dump();
#endif
	}


panic:
	if (!itmon_notified)
		dbg_snapshot_emergency_reboot("MFC H/W issue\n");
}

static void __mfc_dump_info_and_stop_hw_debug(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;

	/*
	* If coredump is supported, recovery after coredump
	* Otherwise,
	*	If debug mode is disable, recovery
	*	If debug mode is enabled, show logs and kernel panic
	*/
	if (__mfc_store_debug_info(core) > 0)
		if (!__mfc_do_sscoredump(core))
			return;

	if (!dev->pdata->debug_mode && !debug_mode_en)
		return;

	__mfc_dump_info_and_stop_hw(core);
}

void mfc_core_meerkat_worker(struct work_struct *work)
{
	struct mfc_core *core;
	int cmd;
	int max_tick_cnt = 3 * MEERKAT_TICK_CNT_TO_START_MEERKAT;
	struct mfc_core_ctx *core_ctx = NULL;
	int curr_ctx;

	core = container_of(work, struct mfc_core, meerkat_work);

	if (atomic_read(&core->meerkat_run)) {
		mfc_core_err("meerkat already running???\n");
		return;
	}

	if (!atomic_read(&core->meerkat_tick_cnt)) {
		mfc_core_err("interrupt handler is called\n");
		return;
	}

	if (feature_option & MFC_OPTION_MEERKAT_DISABLE) {
		mfc_core_info("meerkat disable: no interrupt ignore\n");
		return;
	}

	curr_ctx = __mfc_get_curr_ctx(core);
	if (curr_ctx >= 0) {
		core_ctx = core->core_ctx[curr_ctx];
		if ((core_ctx->state == MFCINST_ERROR) || (core->state == MFCCORE_ERROR)) {
			mfc_core_info("[MSR] It's Error state: core_ctx %d core %d\n",
					core_ctx->state, core->state);
			mfc_core_meerkat_stop_tick(core);
			return;
		}
	}

	cmd = mfc_core_check_risc2host(core);
	if (cmd) {
		if (atomic_read(&core->meerkat_tick_cnt) == max_tick_cnt) {
			mfc_core_err("MFC driver waited for upward of %dsec\n",
					max_tick_cnt);
			core->logging_data->cause |= (1 << MFC_CAUSE_NO_SCHEDULING);
		} else {
			mfc_core_err("interrupt(%d) is occurred, wait scheduling\n", cmd);
			return;
		}
	} else {
		core->logging_data->cause |= (1 << MFC_CAUSE_NO_INTERRUPT);
		mfc_core_err("Driver timeout error handling\n");
	}

	/* Run meerkat worker */
	atomic_set(&core->meerkat_run, 1);

	/* Reset the timeout meerkat */
	atomic_set(&core->meerkat_tick_cnt, 0);

	/* Stop after dumping information */
	__mfc_dump_info_and_stop_hw(core);
}

struct mfc_core_dump_ops mfc_core_dump_ops = {
	.dump_regs			= __mfc_dump_regs,
	.dump_info			= __mfc_dump_info,
	.dump_info_without_regs		= __mfc_dump_info_without_regs,
	.dump_and_stop_always		= __mfc_dump_info_and_stop_hw,
	.dump_and_stop_debug_mode	= __mfc_dump_info_and_stop_hw_debug,
};
