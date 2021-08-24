/*
 * drivers/media/platform/exynos/mfc/mfc_core_run.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "mfc_core_run.h"

#include "mfc_sync.h"

#include "mfc_core_hwlock.h"
#include "mfc_core_pm.h"
#include "mfc_core_cmd.h"
#include "mfc_core_hw_reg_api.h"
#include "mfc_core_enc_param.h"

#include "mfc_queue.h"
#include "mfc_utils.h"
#include "mfc_mem.h"

/* Initialize hardware */
static int __mfc_init_hw(struct mfc_core *core, enum mfc_buf_usage_type buf_type)
{
	int fw_ver;
	int ret = 0;
	int curr_ctx_is_drm_backup;

	mfc_core_debug_enter();

	curr_ctx_is_drm_backup = core->curr_core_ctx_is_drm;

	if (!core->fw_buf.sgt)
		return -EINVAL;

	/* At init time, do not call secure API */
	if (buf_type == MFCBUF_NORMAL)
		core->curr_core_ctx_is_drm = 0;
	else if (buf_type == MFCBUF_DRM)
		core->curr_core_ctx_is_drm = 1;

	/* 0. MFC reset */
	ret = mfc_core_pm_clock_on(core);
	if (ret) {
		mfc_core_err("Failed to enable clock before reset(%d)\n", ret);
		core->curr_core_ctx_is_drm = curr_ctx_is_drm_backup;
		return ret;
	}
	mfc_core_reg_clear(core);
	mfc_core_debug(2, "Done register clear\n");

	if (core->curr_core_ctx_is_drm)
		mfc_core_protection_on(core);

	mfc_core_reset_mfc(core, buf_type);
	mfc_core_debug(2, "Done MFC reset\n");

	/* 1. Set DRAM base Addr */
	mfc_core_set_risc_base_addr(core, buf_type);

	/* 2. Release reset signal to the RISC */
	if (!(core->dev->pdata->security_ctrl && (buf_type == MFCBUF_DRM))) {
		mfc_core_risc_on(core);

		mfc_core_debug(2, "Will now wait for completion of firmware transfer\n");
		if (mfc_wait_for_done_core(core, MFC_REG_R2H_CMD_FW_STATUS_RET)) {
			mfc_core_err("Failed to RISC_ON\n");
			mfc_core_clean_dev_int_flags(core);
			ret = -EIO;
			goto err_init_hw;
		}
	}

	/* 3. Initialize firmware */
	mfc_core_cmd_sys_init(core, buf_type);

	mfc_core_debug(2, "Ok, now will write a command to init the system\n");
	if (mfc_wait_for_done_core(core, MFC_REG_R2H_CMD_SYS_INIT_RET)) {
		mfc_core_err("Failed to SYS_INIT\n");
		mfc_core_clean_dev_int_flags(core);
		ret = -EIO;
		goto err_init_hw;
	}

	core->int_condition = 0;
	if (core->int_err != 0 || core->int_reason != MFC_REG_R2H_CMD_SYS_INIT_RET) {
		/* Failure. */
		mfc_core_err("Failed to init firmware - error: %d, int: %d\n",
				core->int_err, core->int_reason);
		ret = -EIO;
		goto err_init_hw;
	}

	core->fw.fimv_info = mfc_core_get_fimv_info();
	if (core->fw.fimv_info != 'D' && core->fw.fimv_info != 'E')
		core->fw.fimv_info = 'N';

	mfc_core_info("[F/W] MFC v%x, %02xyy %02xmm %02xdd (%c)\n",
		 core->core_pdata->ip_ver,
		 mfc_core_get_fw_ver_year(),
		 mfc_core_get_fw_ver_month(),
		 mfc_core_get_fw_ver_date(),
		 core->fw.fimv_info);

	core->fw.date = mfc_core_get_fw_ver_all();
	/* Check MFC version and F/W version */
	fw_ver = mfc_core_get_mfc_version();
	if (fw_ver != core->core_pdata->ip_ver) {
		mfc_core_err("Invalid F/W version(0x%x) for MFC H/W(0x%x)\n",
				fw_ver, core->core_pdata->ip_ver);
		ret = -EIO;
		goto err_init_hw;
	}

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	mfc_core_cmd_cache_flush(core);
	if (mfc_wait_for_done_core(core, MFC_REG_R2H_CMD_CACHE_FLUSH_RET)) {
		mfc_core_err("Failed to CACHE_FLUSH\n");
		mfc_core_clean_dev_int_flags(core);
		ret = -EIO;
		goto err_init_hw;
	}

	if (buf_type == MFCBUF_DRM && !curr_ctx_is_drm_backup) {
		core->curr_core_ctx_is_drm = curr_ctx_is_drm_backup;
		mfc_core_protection_off(core);
	}
#endif

err_init_hw:
	mfc_core_pm_clock_off(core);
	core->curr_core_ctx_is_drm = curr_ctx_is_drm_backup;
	mfc_core_debug_leave();

	return ret;
}

/* Wrapper : Initialize hardware */
int mfc_core_run_init_hw(struct mfc_core *core)
{
	int ret;

	ret = __mfc_init_hw(core, MFCBUF_NORMAL);
	if (ret)
		return ret;

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (core->fw.drm_status)
		ret = __mfc_init_hw(core, MFCBUF_DRM);
#endif

	return ret;
}

/* Deinitialize hardware */
void mfc_core_run_deinit_hw(struct mfc_core *core)
{
	int ret;

	mfc_core_debug(2, "mfc deinit start\n");

	ret = mfc_core_pm_clock_on(core);
	if (ret) {
		mfc_core_err("Failed to enable clock before reset(%d)\n", ret);
		return;
	}

	mfc_core_mfc_off(core);

	mfc_core_pm_clock_off(core);

	if (core->curr_core_ctx_is_drm)
		mfc_core_protection_off(core);

	mfc_core_debug(2, "mfc deinit completed\n");
}

int mfc_core_run_sleep(struct mfc_core *core)
{
	struct mfc_core_ctx *core_ctx;
	int i;
	int drm_switch = 0;

	mfc_core_debug_enter();

	core_ctx = core->core_ctx[core->curr_core_ctx];
	if (!core_ctx) {
		for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
			if (core->core_ctx[i]) {
				core_ctx = core->core_ctx[i];
				break;
			}
		}

		if (!core_ctx) {
			mfc_core_err("no mfc context to run\n");
			return -EINVAL;
		}
		mfc_core_info("ctx is changed %d -> %d\n",
				core->curr_core_ctx, core_ctx->num);

		core->curr_core_ctx = core_ctx->num;
		if (core->curr_core_ctx_is_drm != core_ctx->is_drm) {
			drm_switch = 1;
			mfc_core_info("DRM attribute is changed %d->%d\n",
					core->curr_core_ctx_is_drm, core_ctx->is_drm);
		}
	}
	mfc_core_info("curr_core_ctx_is_drm:%d\n", core->curr_core_ctx_is_drm);

	mfc_core_pm_clock_on(core);

	if (drm_switch)
		mfc_core_cache_flush(core, core_ctx->is_drm, MFC_CACHEFLUSH, drm_switch);

	mfc_core_cmd_sleep(core);

	if (mfc_wait_for_done_core(core, MFC_REG_R2H_CMD_SLEEP_RET)) {
		mfc_core_err("Failed to SLEEP\n");
		core->logging_data->cause |= (1 << MFC_CAUSE_FAIL_SLEEP);
		call_dop(core, dump_and_stop_always, core);
		return -EBUSY;
	}

	core->int_condition = 0;
	if (core->int_err != 0 || core->int_reason != MFC_REG_R2H_CMD_SLEEP_RET) {
		/* Failure. */
		snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
			"Failed to sleep - error: %d, int: %d\n",
				core->int_err, core->int_reason);
		mfc_core_err("%s", core->crash_info);
		call_dop(core, dump_and_stop_debug_mode, core);
		return -EBUSY;
	}

	core->sleep = 1;

	mfc_core_mfc_off(core);
	mfc_core_pm_clock_off(core);

	if (core->curr_core_ctx_is_drm)
		mfc_core_protection_off(core);

	mfc_core_debug_leave();

	return 0;
}

int mfc_core_run_wakeup(struct mfc_core *core)
{
	enum mfc_buf_usage_type buf_type;
	int ret = 0;

	mfc_core_debug_enter();

	mfc_core_info("curr_core_ctx_is_drm:%d\n", core->curr_core_ctx_is_drm);
	if (core->curr_core_ctx_is_drm)
		buf_type = MFCBUF_DRM;
	else
		buf_type = MFCBUF_NORMAL;

	/* 0. MFC reset */
	ret = mfc_core_pm_clock_on(core);
	if (ret) {
		mfc_core_err("Failed to enable clock before reset(%d)\n", ret);
		return ret;
	}
	mfc_core_reg_clear(core);
	mfc_core_debug(2, "Done register clear\n");

	if (core->curr_core_ctx_is_drm)
		mfc_core_protection_on(core);

	mfc_core_reset_mfc(core, buf_type);
	mfc_core_debug(2, "Done MFC reset\n");

	/* 1. Set DRAM base Addr */
	mfc_core_set_risc_base_addr(core, buf_type);

	/* 2. Release reset signal to the RISC */
	if (!(core->dev->pdata->security_ctrl && (buf_type == MFCBUF_DRM))) {
		mfc_core_risc_on(core);

		mfc_core_debug(2, "Will now wait for completion of firmware transfer\n");
		if (mfc_wait_for_done_core(core, MFC_REG_R2H_CMD_FW_STATUS_RET)) {
			mfc_core_err("Failed to RISC_ON\n");
			core->logging_data->cause |= (1 << MFC_CAUSE_FAIL_RISC_ON);
			call_dop(core, dump_and_stop_always, core);
			return -EBUSY;
		}
	}

	mfc_core_debug(2, "Ok, now will write a command to wakeup the system\n");
	mfc_core_cmd_wakeup(core);

	mfc_core_debug(2, "Will now wait for completion of firmware wake up\n");
	if (mfc_wait_for_done_core(core, MFC_REG_R2H_CMD_WAKEUP_RET)) {
		mfc_core_err("Failed to WAKEUP\n");
		core->logging_data->cause |= (1 << MFC_CAUSE_FAIL_WAKEUP);
		call_dop(core, dump_and_stop_always, core);
		return -EBUSY;
	}

	core->int_condition = 0;
	if (core->int_err != 0 || core->int_reason != MFC_REG_R2H_CMD_WAKEUP_RET) {
		/* Failure. */
		snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
			"Failed to wakeup - error: %d, int: %d\n",
				core->int_err, core->int_reason);
		mfc_core_err("%s", core->crash_info);
		call_dop(core, dump_and_stop_debug_mode, core);
	}

	core->sleep = 0;

	mfc_core_pm_clock_off(core);

	mfc_core_debug_leave();

	return ret;
}

int mfc_core_run_dec_init(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_buf *src_mb;

	/* Initializing decoding - parsing header */

	/* Get the next source buffer */
	src_mb = mfc_get_buf(ctx, &core_ctx->src_buf_queue, MFC_BUF_SET_USED);
	if (!src_mb) {
		mfc_err("no src buffers\n");
		return -EAGAIN;
	}

	mfc_debug(2, "Preparing to init decoding\n");
	mfc_debug(2, "[STREAM] Header size: %d, (offset: %lu)\n",
		src_mb->vb.vb2_buf.planes[0].bytesused, dec->consumed);

	if (dec->consumed)
		mfc_core_set_dec_stream_buffer(core, ctx, src_mb,
				dec->consumed, dec->remained_size);
	else
		mfc_core_set_dec_stream_buffer(core, ctx, src_mb,
				0, src_mb->vb.vb2_buf.planes[0].bytesused);

	mfc_debug(2, "[BUFINFO] Header addr: 0x%08llx\n", src_mb->addr[0][0]);
	mfc_clean_core_ctx_int_flags(core->core_ctx[ctx->num]);
	mfc_core_cmd_dec_seq_header(core, ctx);

	return 0;
}

static int __mfc_check_last_frame(struct mfc_core_ctx *core_ctx,
			struct mfc_buf *mfc_buf)
{
	struct mfc_core *core = core_ctx->core;

	if (mfc_check_mb_flag(mfc_buf, MFC_FLAG_LAST_FRAME)) {
		mfc_core_debug(2, "Setting core_ctx->state to FINISHING\n");
		mfc_change_state(core_ctx, MFCINST_FINISHING);
		return 1;
	}

	return 0;
}

int mfc_core_run_dec_frame(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_buf *src_mb, *dst_mb;
	int last_frame = 0;
	unsigned int index, src_index;
	int ret;

	/* Get the next source buffer */
	if (IS_TWO_MODE2(ctx)) {
		src_mb = mfc_get_buf_no_used(ctx, &core_ctx->src_buf_queue,
				MFC_BUF_SET_USED);
		if (!src_mb) {
			mfc_debug(2, "no src buffers\n");
			return -EAGAIN;
		}
	} else {
		src_mb = mfc_get_buf(ctx, &core_ctx->src_buf_queue,
				MFC_BUF_SET_USED);
		if (!src_mb) {
			mfc_debug(2, "no src buffers\n");
			return -EAGAIN;
		}
	}

	/* Try to use the non-referenced DPB on dst-queue */
	dst_mb = mfc_search_for_dpb(core_ctx);
	if (!dst_mb) {
		src_mb->used = MFC_BUF_RESET_USED;
		mfc_debug(2, "[DPB] couldn't find dst buffers\n");
		return -EAGAIN;
	}

	index = src_mb->vb.vb2_buf.index;
	src_index = src_mb->src_index;

	if (mfc_check_mb_flag(src_mb, MFC_FLAG_EMPTY_DATA))
		src_mb->vb.vb2_buf.planes[0].bytesused = 0;

	if (dec->consumed)
		mfc_core_set_dec_stream_buffer(core, ctx, src_mb,
				dec->consumed, dec->remained_size);
	else
		mfc_core_set_dec_stream_buffer(core, ctx, src_mb,
				0, src_mb->vb.vb2_buf.planes[0].bytesused);

	if (call_cop(ctx, core_set_buf_ctrls_val, core, ctx,
				&ctx->src_ctrls[index]) < 0)
		mfc_err("failed in core_set_buf_ctrls_val\n");
	mfc_core_update_tag(core, ctx, dec->stored_tag);

	mfc_core_set_dynamic_dpb(core, ctx, dst_mb);

	mfc_clean_core_ctx_int_flags(core_ctx);

	last_frame = __mfc_check_last_frame(core_ctx, src_mb);
	ret = mfc_core_cmd_dec_one_frame(core, ctx, last_frame, src_index);

	return ret;
}

int mfc_core_run_dec_last_frames(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_buf *src_mb, *dst_mb;
	unsigned int src_index;

	if (mfc_is_queue_count_same(&ctx->buf_queue_lock, &ctx->dst_buf_queue, 0)) {
		mfc_debug(2, "no dst buffer\n");
		return -EAGAIN;
	}

	/* Try to use the non-referenced DPB on dst-queue */
	dst_mb = mfc_search_for_dpb(core_ctx);
	if (!dst_mb) {
		mfc_debug(2, "[DPB] couldn't find dst buffers\n");
		return -EAGAIN;
	}

	/* Get the next source buffer */
	src_mb = mfc_get_buf(ctx, &core_ctx->src_buf_queue, MFC_BUF_SET_USED);

	/* Frames are being decoded */
	if (!src_mb) {
		mfc_debug(2, "no src buffers\n");
		mfc_core_set_dec_stream_buffer(core, ctx, 0, 0, 0);
		src_index = ctx->curr_src_index + 1;
	} else {
		if (dec->consumed)
			mfc_core_set_dec_stream_buffer(core, ctx, src_mb,
					dec->consumed, dec->remained_size);
		else
			mfc_core_set_dec_stream_buffer(core, ctx, src_mb, 0, 0);
		src_index = src_mb->src_index;
	}

	mfc_core_set_dynamic_dpb(core, ctx, dst_mb);

	mfc_clean_core_ctx_int_flags(core_ctx);
	mfc_core_cmd_dec_one_frame(core, ctx, 1, src_index);

	return 0;
}

int mfc_core_run_enc_init(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_buf *dst_mb;
	int ret;

	dst_mb = mfc_get_buf(ctx, &ctx->dst_buf_queue, MFC_BUF_SET_USED);
	if (!dst_mb) {
		mfc_debug(2, "no dst buffers\n");
		return -EAGAIN;
	}

	mfc_core_set_enc_stream_buffer(core, ctx, dst_mb);

	mfc_core_set_enc_stride(core, ctx);

	mfc_debug(2, "[BUFINFO] Header addr: 0x%08llx\n", dst_mb->addr[0][0]);
	mfc_clean_core_ctx_int_flags(core->core_ctx[ctx->num]);

	ret = mfc_core_cmd_enc_seq_header(core, ctx);
	return ret;
}

int mfc_core_run_enc_frame(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_buf *dst_mb;
	struct mfc_buf *src_mb;
	struct mfc_raw_info *raw;
	struct hdr10_plus_meta dst_sei_meta, *src_sei_meta;
	unsigned int index;
	int last_frame = 0;
	int is_uncomp = 0;

	raw = &ctx->raw_buf;

	/* Get the next source buffer */
	src_mb = mfc_get_buf(ctx, &core_ctx->src_buf_queue, MFC_BUF_SET_USED);
	if (!src_mb) {
		mfc_debug(2, "no src buffers\n");
		return -EAGAIN;
	}

	if (src_mb->num_valid_bufs > 0) {
		/* last image in a buffer container */
		if (src_mb->next_index == (src_mb->num_valid_bufs - 1)) {
			mfc_debug(4, "[BUFCON] last image in a container\n");
			last_frame = __mfc_check_last_frame(core_ctx, src_mb);
		}
	} else {
		last_frame = __mfc_check_last_frame(core_ctx, src_mb);
	}

	/* Support per-frame SBWC change for encoder source */
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->sbwc_enc_src_ctrl)
			&& ctx->is_sbwc) {
		if (mfc_check_mb_flag(src_mb, MFC_FLAG_ENC_SRC_UNCOMP)) {
			is_uncomp = 1;
			mfc_debug(2, "[SBWC] src is uncomp\n");
		} else {
			is_uncomp = 0;
		}

		mfc_core_set_enc_src_sbwc(core,
			(is_uncomp ? MFC_ENC_SRC_SBWC_OFF : MFC_ENC_SRC_SBWC_ON));
		mfc_set_linear_stride_size(ctx,
			(is_uncomp ? enc->uncomp_fmt : ctx->src_fmt));
		mfc_core_set_enc_stride(core, ctx);
	}

	if (mfc_check_mb_flag(src_mb, MFC_FLAG_ENC_SRC_FAKE)) {
		enc->fake_src = 1;
		mfc_debug(2, "src is fake\n");
	}

	index = src_mb->vb.vb2_buf.index;
	if (mfc_check_mb_flag(src_mb, MFC_FLAG_EMPTY_DATA)) {
		enc->empty_data = 1;
		mfc_debug(2, "[FRAME] src is empty data\n");
		mfc_core_set_enc_frame_buffer(core, ctx, 0, raw->num_planes);
	} else {
		mfc_core_set_enc_frame_buffer(core, ctx, src_mb, raw->num_planes);
	}

	/* HDR10+ sei meta */
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->hdr10_plus)) {
		if (enc->sh_handle_hdr.fd == -1) {
			mfc_debug(3, "[HDR+] there is no handle for SEI meta\n");
		} else {
			src_sei_meta = (struct hdr10_plus_meta *)enc->sh_handle_hdr.vaddr + index;
			if (src_sei_meta->valid) {
				mfc_debug(3, "[HDR+] there is valid SEI meta data in buf[%d]\n",
						index);
				memcpy(&dst_sei_meta, src_sei_meta, sizeof(struct hdr10_plus_meta));
				mfc_core_set_hdr_plus_info(core, ctx, &dst_sei_meta);
			}
		}
	}

	dst_mb = mfc_get_buf(ctx, &ctx->dst_buf_queue, MFC_BUF_SET_USED);
	if (!dst_mb) {
		mfc_debug(2, "no dst buffers\n");
		return -EAGAIN;
	}

	mfc_debug(2, "nal start : src index from src_buf_queue:%d\n",
		src_mb->vb.vb2_buf.index);
	mfc_debug(2, "nal start : dst index from dst_buf_queue:%d\n",
		dst_mb->vb.vb2_buf.index);

	mfc_core_set_enc_stream_buffer(core, ctx, dst_mb);

	if (call_cop(ctx, core_set_buf_ctrls_val, core, ctx,
				&ctx->src_ctrls[index]) < 0)
		mfc_err("failed in core_set_buf_ctrls_val\n");

	mfc_clean_core_ctx_int_flags(core_ctx);

	if (enc->is_cbr_fix && dev->pdata->enc_min_bit_cnt)
		mfc_core_set_min_bit_count(core, ctx);
	if (IS_H264_ENC(ctx))
		mfc_core_set_aso_slice_order_h264(core, ctx);
	if (!reg_test)
		mfc_core_set_slice_mode(core, ctx);
	mfc_core_set_enc_config_qp(core, ctx);

	mfc_core_cmd_enc_one_frame(core, ctx, last_frame);

	return 0;
}

int mfc_core_run_enc_last_frames(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_buf *dst_mb = NULL;
	struct mfc_raw_info *raw;

	raw = &ctx->raw_buf;

	dst_mb = mfc_get_buf(ctx, &ctx->dst_buf_queue, MFC_BUF_SET_USED);
	if (!dst_mb) {
		mfc_debug(2, "no dst buffers set to zero\n");

		if (mfc_core_get_enc_bframe(ctx)) {
			mfc_ctx_info("B frame encoding should be dst buffer\n");
			return -EINVAL;
		}
	}

	mfc_debug(2, "Set address zero for all planes\n");
	mfc_core_set_enc_frame_buffer(core, ctx, 0, raw->num_planes);
	mfc_core_set_enc_stream_buffer(core, ctx, dst_mb);

	mfc_clean_core_ctx_int_flags(core->core_ctx[ctx->num]);
	mfc_core_cmd_enc_one_frame(core, ctx, 1);

	return 0;
}
