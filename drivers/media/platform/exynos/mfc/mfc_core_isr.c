/*
 * drivers/media/platform/exynos/mfc/mfc_core_isr.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "mfc_rm.h"

#include "mfc_core_isr.h"

#include "mfc_core_hwlock.h"
#include "mfc_core_intlock.h"
#include "mfc_core_nal_q.h"
#include "mfc_core_otf.h"
#include "mfc_sync.h"

#include "mfc_core_pm.h"
#include "mfc_core_hw_reg_api.h"
#include "mfc_core_qos.h"

#include "mfc_perf_measure.h"
#include "mfc_core_reg_api.h"
#include "mfc_llc.h"
#include "mfc_slc.h"
#include "mfc_qos.h"

#include "mfc_queue.h"
#include "mfc_buf.h"
#include "mfc_mem.h"

static void __mfc_handle_black_bar_info(struct mfc_core *core,
				struct mfc_ctx *ctx)
{
	struct v4l2_rect new_black_bar;
	int black_bar_info;
	struct mfc_dec *dec = ctx->dec_priv;

	black_bar_info = mfc_core_get_black_bar_detection();
	mfc_debug(3, "[BLACKBAR] type: %#x\n", black_bar_info);

	if (black_bar_info == MFC_REG_DISP_STATUS_BLACK_BAR) {
		new_black_bar.left = mfc_core_get_black_bar_pos_x();
		new_black_bar.top = mfc_core_get_black_bar_pos_y();
		new_black_bar.width = mfc_core_get_black_bar_image_w();
		new_black_bar.height = mfc_core_get_black_bar_image_h();
	} else if (black_bar_info == MFC_REG_DISP_STATUS_BLACK_SCREEN) {
		new_black_bar.left = -1;
		new_black_bar.top = -1;
		new_black_bar.width = ctx->img_width;
		new_black_bar.height = ctx->img_height;
	} else if (black_bar_info == MFC_REG_DISP_STATUS_NOT_DETECTED) {
		new_black_bar.left = 0;
		new_black_bar.top = 0;
		new_black_bar.width = ctx->img_width;
		new_black_bar.height = ctx->img_height;
	} else {
		mfc_ctx_err("[BLACKBAR] Not supported type: %#x\n", black_bar_info);
		dec->black_bar_updated = 0;
		return;
	}

	if ((new_black_bar.left == dec->black_bar.left) &&
			(new_black_bar.top == dec->black_bar.top) &&
			(new_black_bar.width == dec->black_bar.width) &&
			(new_black_bar.height == dec->black_bar.height)) {
		mfc_debug(4, "[BLACKBAR] information was not changed\n");
		dec->black_bar_updated = 0;
		return;
	}

	dec->black_bar = new_black_bar;
	dec->black_bar_updated = 1;
}

static unsigned int __mfc_handle_frame_field(struct mfc_core *core,
		struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	unsigned int interlace_type = 0, is_interlace = 0;
	unsigned int field;

	if (CODEC_INTERLACED(ctx))
		is_interlace = mfc_core_is_interlace_picture();

	if (CODEC_MBAFF(ctx))
		dec->is_mbaff = mfc_core_is_mbaff_picture();

	if (is_interlace) {
		interlace_type = mfc_core_get_interlace_type();
		if (interlace_type)
			field = V4L2_FIELD_INTERLACED_TB;
		else
			field = V4L2_FIELD_INTERLACED_BT;
	} else if (dec->is_mbaff) {
		field = V4L2_FIELD_INTERLACED_TB;
	} else {
		field = V4L2_FIELD_NONE;
	}

	mfc_debug(2, "[INTERLACE] is_interlace: %d (type : %d), is_mbaff: %d, field: 0x%#x\n",
			is_interlace, interlace_type, dec->is_mbaff, field);

	return field;
}

static struct mfc_buf *__mfc_handle_last_frame(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_buf *dst_mb;
	int index, i;

	mfc_debug(2, "DQ empty DPB with stored tag\n");

	dst_mb = mfc_get_del_buf(ctx, &ctx->dst_buf_queue,
			MFC_BUF_NO_TOUCH_USED);
	if (!dst_mb) {
		mfc_ctx_err("there is no dst buffer for EOS tag\n");
		return NULL;
	}

	mfc_debug(2, "Cleaning up buffer: [%d][%d]\n",
			dst_mb->vb.vb2_buf.index, dst_mb->dpb_index);

	index = dst_mb->vb.vb2_buf.index;

	for (i = 0; i < ctx->dst_fmt->mem_planes; i++)
		vb2_set_plane_payload(&dst_mb->vb.vb2_buf, i, 0);

	dst_mb->vb.sequence = (++ctx->sequence);
	dst_mb->vb.field = __mfc_handle_frame_field(core, ctx);
	mfc_clear_mb_flag(dst_mb);

	if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
				&ctx->dst_ctrls[index]) < 0)
		mfc_ctx_err("failed in core_get_buf_ctrls_val\n");

	call_cop(ctx, get_buf_update_val, ctx, &ctx->dst_ctrls[index],
			V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG, dec->stored_tag);

	mutex_lock(&dec->dpb_mutex);

	index = dst_mb->dpb_index;
	dec->dpb[index].queued = 0;
	clear_bit(index, &dec->queued_dpb);
	dec->display_index = dst_mb->dpb_index;

	mutex_unlock(&dec->dpb_mutex);
	mfc_debug(2, "[DPB] Cleand up index = %d, used_flag = %#lx, queued = %#lx\n",
			index, dec->dynamic_used, dec->queued_dpb);

	mfc_handle_force_change_status(core->core_ctx[ctx->num]);

	mfc_debug(2, "It can be continue decoding again\n");

	return dst_mb;
}

static void __mfc_handle_frame_unused_output(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_buf *mfc_buf = NULL;
	unsigned int index;

	while (1) {
		mfc_buf = mfc_get_del_buf(ctx, &ctx->dst_buf_err_queue, MFC_BUF_NO_TOUCH_USED);
		if (!mfc_buf)
			break;

		index = mfc_buf->vb.vb2_buf.index;

		mfc_clear_mb_flag(mfc_buf);
		mfc_buf->vb.flags &= ~(V4L2_BUF_FLAG_KEYFRAME |
					V4L2_BUF_FLAG_PFRAME |
					V4L2_BUF_FLAG_BFRAME |
					V4L2_BUF_FLAG_ERROR);

		if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
					&ctx->dst_ctrls[index]) < 0)
			mfc_ctx_err("failed in core_get_buf_ctrls_val\n");

		call_cop(ctx, get_buf_update_val, ctx,
				&ctx->dst_ctrls[index],
				V4L2_CID_MPEG_MFC51_VIDEO_DISPLAY_STATUS,
				MFC_REG_DEC_STATUS_DECODING_ONLY);

		call_cop(ctx, get_buf_update_val, ctx,
				&ctx->dst_ctrls[index],
				V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG,
				UNUSED_TAG);

		dec->ref_buf[dec->refcnt].fd[0] = mfc_buf->vb.vb2_buf.planes[0].m.fd;
		if (dec->refcnt < MFC_MAX_BUFFERS - 1)
			dec->refcnt++;

		vb2_buffer_done(&mfc_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		mfc_debug(2, "[DPB] dst index [%d][%d] fd: %d is buffer done (not used)\n",
				mfc_buf->vb.vb2_buf.index, mfc_buf->dpb_index,
				mfc_buf->vb.vb2_buf.planes[0].m.fd);
	}
}

static void __mfc_handle_frame_all_extracted(struct mfc_core *core,
				struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_buf *dst_mb;
	int index, i, is_first = 1;

	mfc_debug(2, "Decided to finish\n");
	ctx->sequence++;

	if (core_ctx->state == MFCINST_RES_CHANGE_FLUSH)
		is_first = 0;

	while (1) {
		dst_mb = mfc_get_del_buf(ctx, &ctx->dst_buf_queue, MFC_BUF_NO_TOUCH_USED);
		if (!dst_mb)
			break;

		mfc_debug(2, "Cleaning up buffer: [%d][%d]\n",
					  dst_mb->vb.vb2_buf.index, dst_mb->dpb_index);

		index = dst_mb->vb.vb2_buf.index;

		for (i = 0; i < ctx->dst_fmt->mem_planes; i++)
			vb2_set_plane_payload(&dst_mb->vb.vb2_buf, i, 0);

		dst_mb->vb.sequence = (ctx->sequence++);
		dst_mb->vb.field = __mfc_handle_frame_field(core, ctx);
		mfc_clear_mb_flag(dst_mb);

		if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
					&ctx->dst_ctrls[index]) < 0)
			mfc_err("failed in core_get_buf_ctrls_val\n");

		if (is_first) {
			call_cop(ctx, get_buf_update_val, ctx,
				&ctx->dst_ctrls[index],
				V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG,
				dec->stored_tag);
			is_first = 0;
		} else {
			call_cop(ctx, get_buf_update_val, ctx,
				&ctx->dst_ctrls[index],
				V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG,
				DEFAULT_TAG);
			call_cop(ctx, get_buf_update_val, ctx,
				&ctx->dst_ctrls[index],
				V4L2_CID_MPEG_VIDEO_H264_SEI_FP_AVAIL,
				0);
		}

		mutex_lock(&dec->dpb_mutex);

		index = dst_mb->dpb_index;
		dec->dpb[index].queued = 0;
		clear_bit(index, &dec->queued_dpb);

		mutex_unlock(&dec->dpb_mutex);

		vb2_buffer_done(&dst_mb->vb.vb2_buf, VB2_BUF_STATE_DONE);
		mfc_debug(2, "[DPB] Cleand up index = %d, used_flag = %#lx, queued = %#lx\n",
				index, dec->dynamic_used, dec->queued_dpb);
	}

	/* dequeue unused DPB */
	__mfc_handle_frame_unused_output(core, ctx);

	mfc_handle_force_change_status(core_ctx);
	mfc_debug(2, "After cleanup\n");

	mfc_cleanup_iovmm_except_used(ctx);
	mfc_print_dpb_table(ctx);
}

static void __mfc_handle_frame_copy_timestamp(struct mfc_core_ctx *core_ctx,
				dma_addr_t dec_y_addr)
{
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct mfc_buf *dst_mb, *src_mb;

	/* Get the source buffer */
	src_mb = mfc_get_buf(ctx, &core_ctx->src_buf_queue, MFC_BUF_NO_TOUCH_USED);
	if (!src_mb) {
		mfc_err("[TS] no src buffers\n");
		return;
	}

	dst_mb = mfc_find_buf(ctx, &ctx->dst_buf_queue, dec_y_addr);
	if (dst_mb)
		dst_mb->vb.vb2_buf.timestamp = src_mb->vb.vb2_buf.timestamp;
}

static struct mfc_buf *__mfc_handle_frame_output_del(struct mfc_core *core,
		struct mfc_ctx *ctx, unsigned int err)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_raw_info *raw = &ctx->raw_buf;
	struct mfc_buf *dst_mb = NULL;
	dma_addr_t dspl_y_addr;
	unsigned int frame_type;
	unsigned int dst_frame_status;
	unsigned int is_video_signal_type = 0, is_colour_description = 0;
	unsigned int is_content_light = 0, is_display_colour = 0;
	unsigned int is_hdr10_plus_sei = 0, is_av1_film_grain_sei = 0;
	unsigned int is_hdr10_plus_full = 0;
	unsigned int is_uncomp = 0;
	unsigned int i, index, idr_flag, is_last_display;

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->color_aspect_dec)) {
		is_video_signal_type = mfc_core_get_video_signal_type();
		is_colour_description = mfc_core_get_colour_description();
	}

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->static_info_dec)) {
		is_content_light = mfc_core_get_sei_avail_content_light();
		is_display_colour = mfc_core_get_sei_avail_mastering_display();
	}

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->hdr10_plus))
		is_hdr10_plus_sei = mfc_core_get_sei_avail_st_2094_40();

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->hdr10_plus_full))
		is_hdr10_plus_full = mfc_core_get_sei_nal_meta_status();

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->av1_film_grain))
		is_av1_film_grain_sei = mfc_core_get_sei_avail_film_grain();

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->black_bar)
			&& dec->detect_black_bar)
		__mfc_handle_black_bar_info(core, ctx);
	else
		dec->black_bar_updated = 0;

	if (dec->immediate_display == 1) {
		dspl_y_addr = (dma_addr_t)mfc_core_get_dec_y_addr();
		frame_type = mfc_core_get_dec_frame_type();
		idr_flag = mfc_core_get_dec_idr_flag();
	} else {
		dspl_y_addr = (dma_addr_t)mfc_core_get_disp_y_addr();
		frame_type = mfc_core_get_disp_frame_type();
		idr_flag = mfc_core_get_disp_idr_flag();
	}

	is_last_display = mfc_core_get_last_display();

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->sbwc_uncomp) && ctx->is_sbwc)
		is_uncomp = mfc_core_get_uncomp();

	dst_mb = mfc_find_del_buf(ctx, &ctx->dst_buf_queue, dspl_y_addr);
	if (dst_mb) {
		index = dst_mb->vb.vb2_buf.index;
		/* Check if this is the buffer we're looking for */
		mfc_debug(2, "[BUFINFO][DPB] ctx[%d] get dst index: [%d][%d], addr[0]: 0x%08llx\n",
				ctx->num, index, dst_mb->dpb_index,
				dst_mb->addr[0][0]);

		dst_mb->vb.sequence = ctx->sequence;
		dst_mb->vb.field = __mfc_handle_frame_field(core, ctx);

		/* Set flag bits in order to inform SEI information */
		mfc_clear_mb_flag(dst_mb);

		if (is_content_light) {
			mfc_set_mb_flag(dst_mb, MFC_FLAG_HDR_CONTENT_LIGHT);
			mfc_debug(2, "[HDR] content light level parsed\n");
		}

		if (is_display_colour) {
			mfc_set_mb_flag(dst_mb, MFC_FLAG_HDR_DISPLAY_COLOUR);
			mfc_debug(2, "[HDR] mastering display colour parsed\n");
		}

		if (is_video_signal_type) {
			mfc_set_mb_flag(dst_mb, MFC_FLAG_HDR_VIDEO_SIGNAL_TYPE);
			mfc_debug(2, "[HDR] video signal type parsed\n");
			if (is_colour_description) {
				mfc_set_mb_flag(dst_mb,
						MFC_FLAG_HDR_MAXTIX_COEFF);
				mfc_debug(2, "[HDR] matrix coefficients parsed\n");
				mfc_set_mb_flag(dst_mb,
						MFC_FLAG_HDR_COLOUR_DESC);
				mfc_debug(2, "[HDR] colour description parsed\n");
			}
		}

		if (IS_VP9_DEC(ctx) && MFC_FEATURE_SUPPORT(dev,
					dev->pdata->color_aspect_dec)) {
			if (dec->color_space != MFC_REG_D_COLOR_UNKNOWN) {
				mfc_set_mb_flag(dst_mb,
						MFC_FLAG_HDR_COLOUR_DESC);
				mfc_debug(2, "[HDR] color space parsed\n");
			}
			mfc_set_mb_flag(dst_mb, MFC_FLAG_HDR_VIDEO_SIGNAL_TYPE);
			mfc_debug(2, "[HDR] color range parsed\n");
		}

		if ((IS_VP9_DEC(ctx) && mfc_core_get_disp_res_change()) ||
			(IS_AV1_DEC(ctx) && mfc_core_get_disp_res_change_av1())) {
			mfc_ctx_info("[FRAME][DRC] display resolution changed\n");
			mutex_lock(&ctx->drc_wait_mutex);
			ctx->wait_state = WAIT_G_FMT;
			mfc_core_get_img_size(core, ctx, MFC_GET_RESOL_SIZE);
			mfc_set_mb_flag(dst_mb, MFC_FLAG_DISP_RES_CHANGE);
			mutex_unlock(&ctx->drc_wait_mutex);
		}

		if (dec->black_bar_updated) {
			mfc_set_mb_flag(dst_mb, MFC_FLAG_BLACKBAR_DETECT);
			mfc_debug(3, "[BLACKBAR] black bar detected\n");
		}

		if (is_hdr10_plus_sei) {
			if (dec->hdr10_plus_info) {
				mfc_core_get_hdr_plus_info(core, ctx,
						&dec->hdr10_plus_info[index]);
				mfc_set_mb_flag(dst_mb, MFC_FLAG_HDR_PLUS);
				mfc_debug(2, "[HDR+] HDR10 plus dynamic SEI metadata parsed\n");
			} else {
				mfc_ctx_err("[HDR+] HDR10 plus cannot be copied\n");
			}
		} else {
			if (dec->hdr10_plus_info)
				dec->hdr10_plus_info[index].valid = 0;
		}

		if (is_hdr10_plus_full) {
			if (dec->hdr10_plus_full) {
				mfc_core_get_dec_metadata_sei_nal(core, ctx, index);
				mfc_set_mb_flag(dst_mb, MFC_FLAG_HDR_PLUS);
				mfc_debug(2, "[HDR+] HDR10 plus full SEI metadata parsed\n");
			} else {
				mfc_ctx_err("[HDR+] HDR10 plus full cannot be copied\n");
			}
		}

		if (is_av1_film_grain_sei) {
			if (dec->av1_film_grain_info) {
				mfc_core_get_av1_film_grain_info(core, ctx,
						&dec->av1_film_grain_info[index]);
				mfc_set_mb_flag(dst_mb, MFC_FLAG_AV1_FILM_GRAIN);
				mfc_debug(2, "[FILMGR] AV1 Film Grain SEI metadata parsed\n");
			} else {
				mfc_ctx_err("[FILMGR] AV1 Film Grain cannot be copied\n");
			}
		} else {
			if (dec->av1_film_grain_info)
				dec->av1_film_grain_info[index].apply_grain = 0;
		}

		if (is_uncomp) {
			mfc_set_mb_flag(dst_mb, MFC_FLAG_UNCOMP);
			mfc_debug(2, "[SBWC] Uncompressed\n");
		}

		if (ctx->update_framerate) {
			mfc_set_mb_flag(dst_mb, MFC_FLAG_FRAMERATE_CH);
			ctx->update_framerate = false;
			mfc_debug(2, "[QoS] framerate changed\n");
		}

		if (is_last_display) {
			mfc_set_mb_flag(dst_mb, MFC_FLAG_LAST_FRAME);
			mfc_debug(2, "[FRAME] Last display frame\n");
		}

		if ((IS_VP9_DEC(ctx) || IS_AV1_DEC(ctx)) && dec->has_multiframe) {
			mfc_set_mb_flag(dst_mb, MFC_FLAG_MULTIFRAME);
			mfc_debug(2, "[MULTIFRAME] multiframe detected\n");
		}

		if (ctx->dst_fmt->mem_planes == 1) {
			vb2_set_plane_payload(&dst_mb->vb.vb2_buf, 0,
					raw->total_plane_size);
			mfc_debug(5, "single plane payload: %d\n",
					raw->total_plane_size);
		} else {
			for (i = 0; i < ctx->dst_fmt->mem_planes; i++) {
				vb2_set_plane_payload(&dst_mb->vb.vb2_buf, i,
						raw->plane_size[i]);
			}
		}

		dst_mb->vb.flags &= ~(V4L2_BUF_FLAG_KEYFRAME |
					V4L2_BUF_FLAG_PFRAME |
					V4L2_BUF_FLAG_BFRAME |
					V4L2_BUF_FLAG_ERROR);

		switch (frame_type) {
			case MFC_REG_DISPLAY_FRAME_I:
				dst_mb->vb.flags |= V4L2_BUF_FLAG_KEYFRAME;
				if (!(CODEC_HAS_IDR(ctx) && !idr_flag)) {
					mfc_set_mb_flag(dst_mb, MFC_FLAG_SYNC_FRAME);
					mfc_debug(2, "[FRAME] syncframe IDR\n");
				}
				break;
			case MFC_REG_DISPLAY_FRAME_P:
				dst_mb->vb.flags |= V4L2_BUF_FLAG_PFRAME;
				break;
			case MFC_REG_DISPLAY_FRAME_B:
				dst_mb->vb.flags |= V4L2_BUF_FLAG_BFRAME;
				break;
			default:
				break;
		}

		if (mfc_get_warn(err)) {
			mfc_ctx_err("Warning for displayed frame: %d\n",
					mfc_get_warn(err));
			dst_mb->vb.flags |= V4L2_BUF_FLAG_ERROR;
		}

		if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
					&ctx->dst_ctrls[index]) < 0)
			mfc_ctx_err("failed in core_get_buf_ctrls_val\n");

		if (dec->immediate_display == 1) {
			dst_frame_status = mfc_core_get_dec_status();

			call_cop(ctx, get_buf_update_val, ctx,
					&ctx->dst_ctrls[index],
					V4L2_CID_MPEG_MFC51_VIDEO_DISPLAY_STATUS,
					dst_frame_status);

			call_cop(ctx, get_buf_update_val, ctx,
					&ctx->dst_ctrls[index],
					V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG,
					dec->stored_tag);

			dec->immediate_display = 0;
		}

		/* Update frame tag for packed PB */
		if (CODEC_MULTIFRAME(ctx) && (dec->y_addr_for_pb == dspl_y_addr)) {
			call_cop(ctx, get_buf_update_val, ctx,
					&ctx->dst_ctrls[index],
					V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG,
					dec->stored_tag);
			dec->y_addr_for_pb = 0;
		}

		mfc_qos_update_last_framerate(ctx, dst_mb->vb.vb2_buf.timestamp);

		mutex_lock(&dec->dpb_mutex);

		dec->dpb[dst_mb->dpb_index].queued = 0;
		clear_bit(dst_mb->dpb_index, &dec->queued_dpb);
		dec->display_index = dst_mb->dpb_index;

		mutex_unlock(&dec->dpb_mutex);
	} else {
		if (IS_AV1_DEC(ctx) && mfc_core_get_multiple_show_frame())
			dec->is_multiple_show = 1;
	}

	return dst_mb;
}

static void __mfc_handle_released_buf(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	unsigned long prev_flag, released_flag = 0;
	unsigned long flag;
	int i;

	mutex_lock(&dec->dpb_mutex);

	prev_flag = dec->dynamic_used;
	dec->dynamic_used = mfc_core_get_dec_used_flag();
	released_flag = prev_flag & (~dec->dynamic_used);
	mfc_debug(2, "[DPB] Used flag: old = %#lx, new = %#lx, released = %#lx, queued = %#lx\n",
			prev_flag, dec->dynamic_used, released_flag, dec->queued_dpb);

	flag = dec->dynamic_used | released_flag;
	for (i = __ffs(flag); i < MFC_MAX_DPBS;) {
		if (dec->dynamic_used & (1UL << i)) {
			dec->dpb[i].ref = 1;
			if (dec->dpb[i].mapcnt == 0) {
				snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
					"[DPB] %d index is no dpb table\n", i);
				mfc_ctx_err("%s", core->crash_info);
				call_dop(core, dump_and_stop_debug_mode, core);
			}
		}
		if (released_flag & (1UL << i)) {
			dec->dpb[i].ref = 0;
			if (dec->dpb[i].queued && (dec->dpb[i].new_fd != -1)) {
				dec->ref_buf[dec->refcnt].fd[0] = dec->dpb[i].fd[0];
				if (dec->refcnt < MFC_MAX_BUFFERS - 1)
					dec->refcnt++;
				mfc_debug(3, "[REFINFO] Queued DPB[%d] released fd: %d\n",
						i, dec->dpb[i].fd[0]);
				dec->dpb[i].fd[0] = dec->dpb[i].new_fd;
				dec->dpb[i].new_fd = -1;
				mfc_debug(3, "[REFINFO] Queued DPB[%d] reuse fd: %d\n",
						i, dec->dpb[i].fd[0]);
			} else if (!dec->dpb[i].queued) {
				dec->ref_buf[dec->refcnt].fd[0] = dec->dpb[i].fd[0];
				if (dec->refcnt < MFC_MAX_BUFFERS - 1)
					dec->refcnt++;
				mfc_debug(3, "[REFINFO] Dqueued DPB[%d] released fd: %d\n",
						i, dec->dpb[i].fd[0]);
				/*
				 * Except queued buffer,
				 * the released DPB is deleted from dpb_table
				 */
				dec->dpb_table_used &= ~(1UL << i);
				mfc_put_iovmm(ctx, dec->dpb, ctx->dst_fmt->mem_planes, i);
			} else {
				mfc_debug(3, "[REFINFO] Queued DPB[%d] reuse fd: %d\n",
						i, dec->dpb[i].fd[0]);
			}
		}
		flag &= ~(1UL << i);
		if (flag == 0)
			break;
		i = __ffs(flag);
	}

	/* The displayed and not referenced buffer must be freed from dpb_table */
	if (dec->display_index >= 0) {
		i = dec->display_index;
		if (!(dec->dynamic_used & (1UL << i)) && dec->dpb[i].mapcnt
				&& !dec->dpb[i].queued) {
			dec->ref_buf[dec->refcnt].fd[0] = dec->dpb[i].fd[0];
			if (dec->refcnt < MFC_MAX_BUFFERS - 1)
				dec->refcnt++;
			mfc_debug(3, "[REFINFO] display DPB[%d] released fd: %d\n",
					i, dec->dpb[i].fd[0]);
			dec->dpb_table_used &= ~(1UL << i);
			mfc_put_iovmm(ctx, dec->dpb, ctx->dst_fmt->mem_planes, i);
		}
		dec->display_index = -1;
	}

	mfc_print_dpb_table(ctx);

	mutex_unlock(&dec->dpb_mutex);
}

static struct mfc_buf *__mfc_handle_frame_output(struct mfc_core *core,
		struct mfc_ctx *ctx, unsigned int err)
{
	struct mfc_dec *dec = ctx->dec_priv;
	dma_addr_t dspl_y_addr;
	unsigned int frame_type;
	int mvc_view_id;

	frame_type = mfc_core_get_disp_frame_type();
	mvc_view_id = mfc_core_get_mvc_disp_view_id();

	if (IS_H264_MVC_DEC(ctx)) {
		if (mvc_view_id == 0)
			ctx->sequence++;
	} else {
		ctx->sequence++;
	}

	dspl_y_addr = mfc_core_get_disp_y_addr();

	if (dec->immediate_display == 1) {
		dspl_y_addr = (dma_addr_t)mfc_core_get_dec_y_addr();
		frame_type = mfc_core_get_dec_frame_type();
	}

	mfc_debug(2, "[FRAME] frame type: %d\n", frame_type);

	/* If frame is same as previous then skip and do not dequeue */
	if (frame_type == MFC_REG_DISPLAY_FRAME_NOT_CODED)
		if (!CODEC_NOT_CODED(ctx))
			return NULL;

	/* Dequeued display buffer for user */
	return __mfc_handle_frame_output_del(core, ctx, err);
}

static void __mfc_handle_error_state(struct mfc_ctx *ctx, struct mfc_core_ctx *core_ctx)
{
	mfc_err("[MSR] It's Error state: cleanup queue\n");
	MFC_TRACE_CORE_CTX("*** ERROR state\n");

	mfc_change_state(core_ctx, MFCINST_ERROR);

	/* Mark all dst buffers as having an error */
	mfc_cleanup_queue(&ctx->buf_queue_lock, &ctx->dst_buf_queue);
	if (ctx->type == MFCINST_DECODER)
		mfc_cleanup_queue(&ctx->buf_queue_lock, &ctx->dst_buf_err_queue);
	/* Mark all src buffers as having an error */
	mfc_cleanup_queue(&ctx->buf_queue_lock, &ctx->src_buf_ready_queue);
	mfc_cleanup_queue(&ctx->buf_queue_lock, &core_ctx->src_buf_queue);
	if (ctx->type == MFCINST_ENCODER)
		mfc_cleanup_queue(&ctx->buf_queue_lock, &ctx->ref_buf_queue);
	/* Mark all NAL_Q buffers as having an error */
	mfc_cleanup_nal_queue(core_ctx);
}

void mfc_core_handle_error(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;
	struct mfc_core_ctx *core_ctx;
	int i;

	mfc_core_err("[MSR] >>>>>>>> MFC CORE is Error state <<<<<<<<\n");
	mfc_core_change_state(core, MFCCORE_ERROR);

	mutex_lock(&dev->mfc_mutex);
	for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
		if (!core->core_ctx[i])
			continue;
		/* TODO: need to check two core mode */
		core_ctx = core->core_ctx[i];
		__mfc_handle_error_state(core_ctx->ctx, core_ctx);
	}
	mutex_unlock(&dev->mfc_mutex);
}

/* Error handling for interrupt */
static inline void __mfc_handle_error(struct mfc_core *core, struct mfc_ctx *ctx,
	unsigned int reason, unsigned int err)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_buf *src_mb;

	mfc_err("Interrupt Error: display: %d, decoded: %d\n",
			mfc_get_warn(err), mfc_get_err(err));
	err = mfc_get_err(err);

	/* Error recovery is dependent on the state of context */
	switch (core_ctx->state) {
	case MFCINST_RES_CHANGE_END:
	case MFCINST_GOT_INST:
		/* This error had to happen while parsing the header */
		src_mb = mfc_get_del_buf(ctx, &core_ctx->src_buf_queue,
				MFC_BUF_NO_TOUCH_USED);
		if (src_mb) {
			if (!ctx->is_drm) {
				unsigned char *stream_vir = NULL;
				unsigned int strm_size = 0;

				stream_vir = src_mb->vir_addr;
				strm_size = src_mb->vb.vb2_buf.planes[0].bytesused;
				if (strm_size > 640)
					strm_size = 640;

				if (stream_vir && strm_size)
					print_hex_dump(KERN_ERR, "No header: ",
							DUMP_PREFIX_OFFSET, 32, 4,
							stream_vir, strm_size, false);
			}

			mfc_clear_mb_flag(src_mb);
			mfc_set_mb_flag(src_mb, MFC_FLAG_CONSUMED_ONLY);
			vb2_buffer_done(&src_mb->vb.vb2_buf, VB2_BUF_STATE_DONE);
		}
		break;
	case MFCINST_INIT:
		/* This error had to happen while acquireing instance */
	case MFCINST_RETURN_INST:
		/* This error had to happen while releasing instance */
	case MFCINST_DPB_FLUSHING:
		/* This error had to happen while flushing DPB */
	case MFCINST_SPECIAL_PARSING:
	case MFCINST_SPECIAL_PARSING_NAL:
		/* This error had to happen while special parsing */
		break;
	case MFCINST_HEAD_PARSED:
		/* This error had to happen while setting dst buffers */
	case MFCINST_RES_CHANGE_INIT:
	case MFCINST_RES_CHANGE_FLUSH:
		/* This error has to happen while resolution change */
	case MFCINST_ABORT_INST:
		/* This error has to happen while buffer full handling */
	case MFCINST_FINISHING:
		/* It is higly probable that an error occured
		 * while decoding a frame */
		__mfc_handle_error_state(ctx, core_ctx);
		break;
	default:
		mfc_err("Encountered an error interrupt which had not been handled\n");
		mfc_err("core_ctx->state = %d, core_ctx->inst_no = %d\n",
						core_ctx->state, core_ctx->inst_no);
		break;
	}

	mfc_wake_up_core(core, reason, err);

	return;
}

static void __mfc_handle_frame_error(struct mfc_core *core, struct mfc_ctx *ctx,
		unsigned int reason, unsigned int err)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_dec *dec;
	struct mfc_buf *src_mb;
	unsigned int index;
	enum vb2_buffer_state vb2_state;

	if (ctx->type == MFCINST_ENCODER) {
		mfc_err("Encoder Interrupt Error (err: %d, warn: %d)\n",
				mfc_get_err(err), mfc_get_warn(err));

		if (mfc_get_err(err) == MFC_REG_ERR_UNDEFINED_EXCEPTION)
			mfc_core_handle_error(core);

		return;
	}

	dec = ctx->dec_priv;
	if (!dec) {
		mfc_err("no mfc decoder to run\n");
		return;
	}

	mfc_err("Interrupt Error: %d\n", err);

	/* Get the source buffer */
	src_mb = mfc_get_del_buf(ctx, &core_ctx->src_buf_queue,
			MFC_BUF_NO_TOUCH_USED);
	if (!src_mb) {
		mfc_err("no src buffers\n");
	} else {
		index = src_mb->vb.vb2_buf.index;
		if (call_cop(ctx, core_recover_buf_ctrls_val, core, ctx,
					&ctx->src_ctrls[index]) < 0)
			mfc_err("failed in core_recover_buf_ctrls_val\n");

		mfc_debug(2, "MFC needs next buffer\n");
		dec->consumed = 0;
		mfc_clear_mb_flag(src_mb);
		mfc_set_mb_flag(src_mb, MFC_FLAG_CONSUMED_ONLY);

		if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
					&ctx->src_ctrls[index]) < 0)
			mfc_err("failed in core_get_buf_ctrls_val\n");

		vb2_state = __mfc_get_buf_state(mfc_get_err(err));
		mfc_debug(2, "[STREAM] consumed only by error (state: %d)\n", vb2_state);
		vb2_buffer_done(&src_mb->vb.vb2_buf, vb2_state);
	}

	if (mfc_get_err(err) == MFC_REG_ERR_UNDEFINED_EXCEPTION)
		mfc_core_handle_error(core);

	mfc_debug(2, "Assesing whether this context should be run again\n");
}

static void __mfc_handle_frame_input(struct mfc_core *core,
		struct mfc_ctx *ctx, unsigned int err)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_buf *src_mb;
	unsigned int index;
	int deleted = 0;
	unsigned int consumed;

	if (mfc_get_err(err) == MFC_REG_ERR_NON_PAIRED_FIELD) {
		/*
		 * For non-paired field, the same buffer need to be
		 * resubmitted and the consumed stream will be 0
		 */
		mfc_debug(2, "Not paired field. Running again the same buffer\n");
		return;
	}

	/* Get the source buffer */
	consumed = mfc_core_get_consumed_stream();
	src_mb = mfc_get_del_if_consumed(ctx, &core_ctx->src_buf_queue,
			consumed, STUFF_BYTE, err, &deleted);
	if (!src_mb) {
		mfc_err("no src buffers\n");
		return;
	}

	index = src_mb->vb.vb2_buf.index;
	mfc_debug(2, "[BUFINFO] ctx[%d] get src index: %d(%d), addr: 0x%08llx\n",
			ctx->num, index, src_mb->src_index, src_mb->addr[0][0]);

	if (!deleted) {
		/* Run MFC again on the same buffer */
		mfc_debug(2, "[MULTIFRAME] Running again the same buffer\n");

		if (CODEC_MULTIFRAME(ctx))
			dec->y_addr_for_pb = (dma_addr_t)mfc_core_get_dec_y_addr();

		dec->consumed += consumed;
		dec->has_multiframe = 1;

		MFC_TRACE_CORE_CTX("** consumed:%d, remained:%d, addr:0x%08llx\n",
			dec->consumed, mfc_dec_get_strm_size(ctx, src_mb), dec->y_addr_for_pb);
		/* Do not move src buffer to done_list */
		return;
	}

	if (call_cop(ctx, core_recover_buf_ctrls_val, core, ctx,
				&ctx->src_ctrls[index]) < 0)
		mfc_err("failed in core_recover_buf_ctrls_val\n");

	mfc_clear_mb_flag(src_mb);

	if ((IS_VP9_DEC(ctx) || IS_AV1_DEC(ctx)) && dec->has_multiframe &&
		(mfc_core_get_disp_status() == MFC_REG_DEC_STATUS_DECODING_ONLY)) {
		mfc_set_mb_flag(src_mb, MFC_FLAG_CONSUMED_ONLY);
		mfc_debug(2, "[STREAM][MULTIFRAME] last frame is decoding only\n");
	}

	/*
	 * VP8 decoder has decoding only frame,
	 * it will be used for reference frame only not displayed.
	 * So, driver inform to user this input has no destination.
	 */
	if (((IS_VP8_DEC(ctx) || IS_VP9_DEC(ctx)) &&
		(mfc_core_get_disp_status() == MFC_REG_DEC_STATUS_DECODING_ONLY)) ||
		(mfc_core_get_int_reason() == MFC_REG_R2H_CMD_FIELD_DONE_RET)) {
		mfc_set_mb_flag(src_mb, MFC_FLAG_CONSUMED_ONLY);
		mfc_debug(2, "[STREAM] %s decoding only stream has no buffer to DQ\n",
				ctx->src_fmt->name);
	}

	/*
	 * Because AV1 has a no show frame, there are two cases that
	 * driver should inform to user this input has no destination buffer.
	 * 1) If it's decoding only and it's not showable frame,
	 *   it will be used for reference frame only not displayed.
	 * 2) If the buffer that has already DQ to display comes to new display,
	 *   it is multiple show frame.
	 */
	if (IS_AV1_DEC(ctx)) {
		if ((mfc_core_get_disp_status() == MFC_REG_DEC_STATUS_DECODING_ONLY) &&
				!mfc_core_get_showable_frame()) {
			mfc_set_mb_flag(src_mb, MFC_FLAG_CONSUMED_ONLY);
			mfc_debug(2, "[STREAM] AV1 no showable frame has no buffer to DQ\n");
		}
		if (dec->is_multiple_show) {
			mfc_set_mb_flag(src_mb, MFC_FLAG_CONSUMED_ONLY);
			dec->is_multiple_show = 0;
			mfc_ctx_info("[STREAM] AV1 multiple show frame has no buffer to DQ\n");
		}
	}

	/* If pic_output_flag is 0 in HEVC, it is no destination buffer */
	if (IS_HEVC_DEC(ctx) &&
			MFC_FEATURE_SUPPORT(dev, dev->pdata->hevc_pic_output_flag) &&
			!mfc_core_get_hevc_pic_output_flag()) {
		mfc_set_mb_flag(src_mb, MFC_FLAG_CONSUMED_ONLY);
		mfc_debug(2, "[STREAM] HEVC pic_output_flag off has no buffer to DQ\n");
	}

	if ((mfc_core_get_disp_status() == MFC_REG_DEC_STATUS_DECODING_ONLY) &&
			(mfc_core_get_dec_y_addr() == 0)) {
		mfc_set_mb_flag(src_mb, MFC_FLAG_CONSUMED_ONLY);
		mfc_debug(2, "[STREAM] decoding only but there is no address\n");
	}

	if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
				&ctx->src_ctrls[index]) < 0)
		mfc_err("failed in core_get_buf_ctrls_val\n");

	dec->consumed = 0;
	if (IS_VP9_DEC(ctx) || IS_AV1_DEC(ctx))
		dec->has_multiframe = 0;

	vb2_buffer_done(&src_mb->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

/* Handle frame decoding interrupt */
static void __mfc_handle_frame(struct mfc_core *core, struct mfc_ctx *ctx,
			unsigned int reason, unsigned int err)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	unsigned int dst_frame_status, sei_avail_frame_pack;
	unsigned int res_change, need_dpb_change, need_scratch_change;
	struct dec_dpb_ref_info *ref_info = NULL;
	struct mfc_buf *mfc_buf = NULL;
	int i;

	mfc_perf_trace(ctx, "type", mfc_core_get_dec_frame_type());

	dst_frame_status = mfc_core_get_disp_status();
	res_change = mfc_core_get_res_change();
	need_dpb_change = mfc_core_get_dpb_change();
	need_scratch_change = mfc_core_get_scratch_change();
	sei_avail_frame_pack = mfc_core_get_sei_avail_frame_pack();

	if (dec->immediate_display == 1)
		dst_frame_status = mfc_core_get_dec_status();

	mfc_debug(2, "[FRAME] frame status: %d\n", dst_frame_status);
	mfc_debug(2, "[FRAME] display status: %d, type: %d, yaddr: %#x\n",
			mfc_core_get_disp_status(),
			mfc_core_get_disp_frame_type(),
			mfc_core_get_disp_y_addr());
	mfc_debug(2, "[FRAME] decoded status: %d, type: %d, yaddr: %#x\n",
			mfc_core_get_dec_status(),
			mfc_core_get_dec_frame_type(),
			mfc_core_get_dec_y_addr());

	mfc_debug(4, "[HDR] SEI available status: 0x%08x\n",
			mfc_core_get_sei_avail());
	mfc_debug(4, "[HDR] SEI content light: 0x%08x\n",
			mfc_core_get_sei_content_light());
	mfc_debug(4, "[HDR] SEI luminance: 0x%08x, 0x%08x white point: 0x%08x\n",
			mfc_core_get_sei_mastering0(),
			mfc_core_get_sei_mastering1(),
			mfc_core_get_sei_mastering2());
	mfc_debug(4, "[HDR] SEI display primaries: 0x%08x, 0x%08x, 0x%08x\n",
			mfc_core_get_sei_mastering3(),
			mfc_core_get_sei_mastering4(),
			mfc_core_get_sei_mastering5());

	if (core_ctx->state == MFCINST_RES_CHANGE_INIT)
		mfc_change_state(core_ctx, MFCINST_RES_CHANGE_FLUSH);

	if (res_change) {
		mfc_debug(2, "[DRC] Resolution change set to %d\n", res_change);
		mutex_lock(&ctx->drc_wait_mutex);
		mfc_change_state(core_ctx, MFCINST_RES_CHANGE_INIT);
		ctx->wait_state = WAIT_G_FMT | WAIT_STOP;
		mfc_debug(2, "[DRC] Decoding waiting! : %d\n", ctx->wait_state);
		mutex_unlock(&ctx->drc_wait_mutex);
		return;
	}

	if (need_dpb_change || need_scratch_change) {
		mfc_ctx_info("[DRC] Interframe resolution changed\n");
		mutex_lock(&ctx->drc_wait_mutex);
		ctx->wait_state = WAIT_G_FMT | WAIT_STOP;
		mfc_core_get_img_size(core, ctx, MFC_GET_RESOL_DPB_SIZE);
		dec->inter_res_change = 1;
		mutex_unlock(&ctx->drc_wait_mutex);
		__mfc_handle_frame_all_extracted(core, ctx);
		return;
	}

	if (mfc_is_queue_count_same(&ctx->buf_queue_lock, &core_ctx->src_buf_queue, 0) &&
		mfc_is_queue_count_same(&ctx->buf_queue_lock, &ctx->dst_buf_queue, 0)) {
		mfc_err("Queue count is zero for src and dst\n");
		goto leave_handle_frame;
	}

	if (IS_H264_DEC(ctx) && sei_avail_frame_pack &&
		dst_frame_status == MFC_REG_DEC_STATUS_DECODING_ONLY) {
		mfc_debug(2, "Frame packing SEI exists for a frame\n");
		mfc_debug(2, "Reallocate DPBs and issue init_buffer\n");
		mutex_lock(&ctx->drc_wait_mutex);
		ctx->is_dpb_realloc = 1;
		mfc_change_state(core_ctx, MFCINST_HEAD_PARSED);
		ctx->capture_state = QUEUE_FREE;
		ctx->wait_state = WAIT_STOP;
		mutex_unlock(&ctx->drc_wait_mutex);
		__mfc_handle_frame_all_extracted(core, ctx);
		goto leave_handle_frame;
	}

	/* All frames remaining in the buffer have been extracted  */
	if (dst_frame_status == MFC_REG_DEC_STATUS_DECODING_EMPTY) {
		if (core_ctx->state == MFCINST_RES_CHANGE_FLUSH) {
			struct mfc_timestamp *temp_ts = NULL;

			mfc_debug(2, "[DRC] Last frame received after resolution change\n");
			__mfc_handle_frame_all_extracted(core, ctx);
			mfc_change_state(core_ctx, MFCINST_RES_CHANGE_END);

			if (IS_MULTI_CORE_DEVICE(dev))
				mfc_rm_load_balancing(ctx, MFC_RM_LOAD_DELETE);

			/* empty the timestamp queue */
			while (!list_empty(&ctx->ts_list)) {
				temp_ts = list_entry((&ctx->ts_list)->next,
						struct mfc_timestamp, list);
				list_del(&temp_ts->list);
			}
			ctx->ts_count = 0;
			ctx->ts_is_full = 0;
			mfc_qos_reset_last_framerate(ctx);
			mfc_qos_set_framerate(ctx, DEC_DEFAULT_FPS);
			mfc_core_qos_on(core, ctx);

			goto leave_handle_frame;
		} else {
			mfc_buf = __mfc_handle_last_frame(core, ctx);
		}
	}

	/* Detection for QoS weight */
	if (!dec->num_of_tile_over_4 && mfc_core_get_num_of_tile() >= 4) {
		dec->num_of_tile_over_4 = 1;
		mfc_core_qos_on(core, ctx);
	}
	if (!dec->super64_bframe && IS_SUPER64_BFRAME(ctx,
				mfc_core_get_lcu_size(),
				mfc_core_get_dec_frame_type())) {
		dec->super64_bframe = 1;
		mfc_core_qos_on(core, ctx);
	}

	/* copy decoded timestamp */
	if (mfc_dec_status_decoding(dst_frame_status))
		__mfc_handle_frame_copy_timestamp(core_ctx,
				mfc_core_get_dec_y_addr());

	/* Mark source buffer as complete */
	if (dst_frame_status != MFC_REG_DEC_STATUS_DISPLAY_ONLY)
		__mfc_handle_frame_input(core, ctx, err);

	/* A frame has been decoded and is in the buffer  */
	if (mfc_dec_status_display(dst_frame_status))
		mfc_buf = __mfc_handle_frame_output(core, ctx, err);

	/* arrangement of assigned dpb table */
	__mfc_handle_released_buf(core, ctx);

	/* dequeue unused DPB */
	__mfc_handle_frame_unused_output(core, ctx);

	/* There is display buffer for user, update reference information */
	if (mfc_buf) {
		ref_info = &dec->ref_info[mfc_buf->vb.vb2_buf.index];
		for (i = 0; i < dec->refcnt; i++)
			ref_info->dpb[i].fd[0] = dec->ref_buf[i].fd[0];
		if (dec->refcnt != MFC_MAX_BUFFERS)
			ref_info->dpb[i].fd[0] = MFC_INFO_INIT_FD;
		dec->refcnt = 0;

		mfc_debug(2, "[DPB] dst index [%d][%d] fd: %d is buffer done\n",
				mfc_buf->vb.vb2_buf.index, mfc_buf->dpb_index,
				mfc_buf->vb.vb2_buf.planes[0].m.fd);
		vb2_buffer_done(&mfc_buf->vb.vb2_buf, mfc_get_warn(err) ?
				VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	} else {
		for (i = 0; i < dec->refcnt; i++)
			mfc_debug(2, "[REFINFO] Released FD = %d will update with display buffer\n",
					dec->ref_buf[i].fd[0]);
	}

	if (regression_option & MFC_TEST_DEC_PER_FRAME)
		mfc_core_dec_save_regression_result(core);

#ifdef CONFIG_MFC_USE_COREDUMP
	if (sscd_report && (ctx->frame_cnt == 200)) {
		snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
			"Manual trgger MFC SSR in decoding.\n");
		call_dop(core, dump_and_stop_debug_mode, core);
	}
#endif

leave_handle_frame:
	mfc_debug(2, "Assesing whether this context should be run again\n");
}

static void __mfc_handle_stream_copy_timestamp(struct mfc_ctx *ctx, struct mfc_buf *src_mb)
{
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_enc_params *p = &enc->params;
	struct mfc_buf *dst_mb;
	u64 interval;
	u64 start_timestamp;
	u64 new_timestamp;

	start_timestamp = src_mb->vb.vb2_buf.timestamp;
	interval = NSEC_PER_SEC / p->rc_framerate;
	if (debug_ts == 1)
		mfc_ctx_info("[BUFCON][TS] %dfps, start timestamp: %lld, base interval: %lld\n",
				p->rc_framerate, start_timestamp, interval);

	new_timestamp = start_timestamp + (interval * src_mb->done_index);
	if (debug_ts == 1)
		mfc_ctx_info("[BUFCON][TS] new timestamp: %lld, interval: %lld\n",
				new_timestamp, interval * src_mb->done_index);

	/* Get the destination buffer */
	dst_mb = mfc_get_buf(ctx, &ctx->dst_buf_queue, MFC_BUF_NO_TOUCH_USED);
	if (dst_mb)
		dst_mb->vb.vb2_buf.timestamp = new_timestamp;
}

static void __mfc_handle_stream_input(struct mfc_core *core, struct mfc_ctx *ctx,
					int consumed_only)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_raw_info *raw;
	struct mfc_buf *ref_mb, *src_mb;
	dma_addr_t enc_addr[3] = { 0, 0, 0 };
	struct mfc_enc *enc = ctx->enc_priv;
	int i, found_in_src_queue = 0;
	unsigned int index;

	raw = &ctx->raw_buf;

	mfc_core_get_enc_frame_buffer(core, ctx, &enc_addr[0], raw->num_planes);
	if (enc_addr[0] == 0) {
		mfc_debug(3, "no encoded src\n");

		if (enc->fake_src && enc->params.num_b_frame) {
			mfc_change_state(core_ctx, MFCINST_FINISHING);
			enc->fake_src = 0;
			mfc_debug(2, "clear fake_src and change to FINISHING\n");
		}

		goto move_buf;
	}
	for (i = 0; i < raw->num_planes; i++)
		mfc_debug(2, "[BUFINFO] ctx[%d] get src addr[%d]: 0x%08llx\n",
				ctx->num, i, enc_addr[i]);

	if (IS_BUFFER_BATCH_MODE(ctx)) {
		src_mb = mfc_find_first_buf(ctx, &core_ctx->src_buf_queue, enc_addr[0]);
		if (src_mb) {
			found_in_src_queue = 1;

			__mfc_handle_stream_copy_timestamp(ctx, src_mb);
			src_mb->done_index++;
			mfc_debug(4, "[BUFCON] batch buf done_index: %d\n", src_mb->done_index);

			index = src_mb->vb.vb2_buf.index;
			/* single buffer || last image in a buffer container */
			if (!src_mb->num_valid_bufs || src_mb->done_index == src_mb->num_valid_bufs) {
				if (consumed_only) {
					mfc_clear_mb_flag(src_mb);
					mfc_set_mb_flag(src_mb, MFC_FLAG_CONSUMED_ONLY);
				}

				if (call_cop(ctx, core_recover_buf_ctrls_val, core, ctx,
							&ctx->src_ctrls[index]) < 0)
					mfc_err("failed in core_recover_buf_ctrls_val\n");

				if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
							&ctx->src_ctrls[index]) < 0)
					mfc_err("failed in core_get_buf_ctrls_val\n");

				src_mb = mfc_find_del_buf(ctx, &core_ctx->src_buf_queue, enc_addr[0]);
				if (src_mb) {
					for (i = 0; i < raw->num_planes; i++)
						mfc_bufcon_put_daddr(ctx, src_mb, i);
					vb2_buffer_done(&src_mb->vb.vb2_buf, VB2_BUF_STATE_DONE);
				}
			}
		}
	} else {
		/* normal single buffer */
		src_mb = mfc_find_del_buf(ctx, &core_ctx->src_buf_queue, enc_addr[0]);
		if (src_mb) {
			found_in_src_queue = 1;
			index = src_mb->vb.vb2_buf.index;
			if (consumed_only) {
				mfc_clear_mb_flag(src_mb);
				mfc_set_mb_flag(src_mb, MFC_FLAG_CONSUMED_ONLY);
			}

			if (call_cop(ctx, core_recover_buf_ctrls_val, core, ctx,
						&ctx->src_ctrls[index]) < 0)
				mfc_err("failed in core_recover_buf_ctrls_val\n");

			if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
						&ctx->src_ctrls[index]) < 0)
				mfc_err("failed in core_get_buf_ctrls_val\n");

			mfc_debug(3, "find src buf in src_queue\n");
			vb2_buffer_done(&src_mb->vb.vb2_buf, VB2_BUF_STATE_DONE);
		} else {
			mfc_debug(3, "no src buf in src_queue\n");
			ref_mb = mfc_find_del_buf(ctx, &ctx->ref_buf_queue, enc_addr[0]);
			if (ref_mb) {
				index = ref_mb->vb.vb2_buf.index;
				if (consumed_only) {
					mfc_clear_mb_flag(ref_mb);
					mfc_set_mb_flag(ref_mb, MFC_FLAG_CONSUMED_ONLY);
				}

				if (call_cop(ctx, core_recover_buf_ctrls_val, core, ctx,
							&ctx->src_ctrls[index]) < 0)
					mfc_err("failed in core_recover_buf_ctrls_val\n");

				if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
							&ctx->src_ctrls[index]) < 0)
					mfc_err("failed in core_get_buf_ctrls_val\n");

				mfc_debug(3, "find src buf in ref_queue\n");
				vb2_buffer_done(&ref_mb->vb.vb2_buf, VB2_BUF_STATE_DONE);
			} else {
				mfc_err("couldn't find src buffer\n");
			}
		}

	}

move_buf:
	/* move enqueued src buffer: src queue -> ref queue */
	if (!found_in_src_queue && core_ctx->state != MFCINST_FINISHING) {
		mfc_get_move_buf_used(ctx, &ctx->ref_buf_queue, &core_ctx->src_buf_queue);

		mfc_debug(2, "enc src_buf_queue(%d) -> ref_buf_queue(%d)\n",
				mfc_get_queue_count(&ctx->buf_queue_lock, &core_ctx->src_buf_queue),
				mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->ref_buf_queue));
	}
}

static void __mfc_handle_stream_output(struct mfc_core *core,
		struct mfc_ctx *ctx, int slice_type, unsigned int strm_size)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_buf *dst_mb;
	unsigned int index, idr_flag = 1;

	if (strm_size == 0) {
		mfc_debug(3, "no encoded dst (reuse)\n");
		return;
	}

	/* at least one more dest. buffers exist always  */
	dst_mb = mfc_get_del_buf(ctx, &ctx->dst_buf_queue, MFC_BUF_NO_TOUCH_USED);
	if (!dst_mb) {
		mfc_ctx_err("no dst buffers\n");
		return;
	}

	mfc_debug(2, "[BUFINFO] ctx[%d] get dst addr: 0x%08llx\n",
			ctx->num, dst_mb->addr[0][0]);

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->enc_idr_flag))
		idr_flag = mfc_core_get_enc_idr_flag();

	mfc_clear_mb_flag(dst_mb);
	dst_mb->vb.flags &= ~(V4L2_BUF_FLAG_KEYFRAME |
				V4L2_BUF_FLAG_PFRAME |
				V4L2_BUF_FLAG_BFRAME);
	switch (slice_type) {
	case MFC_REG_E_SLICE_TYPE_I:
		dst_mb->vb.flags |= V4L2_BUF_FLAG_KEYFRAME;
		if (!(CODEC_HAS_IDR(ctx) && !idr_flag)) {
			mfc_set_mb_flag(dst_mb, MFC_FLAG_SYNC_FRAME);
			mfc_debug(2, "[STREAM] syncframe IDR\n");
		}
		break;
	case MFC_REG_E_SLICE_TYPE_P:
		dst_mb->vb.flags |= V4L2_BUF_FLAG_PFRAME;
		break;
	case MFC_REG_E_SLICE_TYPE_B:
		dst_mb->vb.flags |= V4L2_BUF_FLAG_BFRAME;
		break;
	default:
		dst_mb->vb.flags |= V4L2_BUF_FLAG_KEYFRAME;
		break;
	}
	mfc_debug(2, "[STREAM] Slice type flag: %d\n", dst_mb->vb.flags);

	if (IS_BPG_ENC(ctx)) {
		strm_size += enc->header_size;
		mfc_debug(2, "bpg total stream size: %d\n", strm_size);
	}
	vb2_set_plane_payload(&dst_mb->vb.vb2_buf, 0, strm_size);
	mfc_qos_update_framerate(ctx, strm_size);

	index = dst_mb->vb.vb2_buf.index;
	if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
				&ctx->dst_ctrls[index]) < 0)
		mfc_ctx_err("failed in core_get_buf_ctrls_val\n");

	vb2_buffer_done(&dst_mb->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

static void __mfc_handle_stream_last_output(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_buf *dst_mb;
	unsigned int index;

	/* at least one more dest. buffers exist always  */
	dst_mb = mfc_get_del_buf(ctx, &ctx->dst_buf_queue, MFC_BUF_NO_TOUCH_USED);
	if (!dst_mb) {
		mfc_ctx_err("no dst buffers\n");
		return;
	}

	mfc_debug(2, "[BUFINFO] ctx[%d] get dst addr: 0x%08llx\n",
			ctx->num, dst_mb->addr[0][0]);

	dst_mb->vb.flags &= ~(V4L2_BUF_FLAG_KEYFRAME |
				V4L2_BUF_FLAG_PFRAME |
				V4L2_BUF_FLAG_BFRAME);

	vb2_set_plane_payload(&dst_mb->vb.vb2_buf, 0, 0);

	index = dst_mb->vb.vb2_buf.index;
	if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
				&ctx->dst_ctrls[index]) < 0)
		mfc_ctx_err("failed in core_get_buf_ctrls_val\n");

	mfc_debug(2, "[STREAM] update tag for last stream\n");
	call_cop(ctx, get_buf_update_val, ctx, &ctx->dst_ctrls[index],
			V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG, enc->stored_tag);

	vb2_buffer_done(&dst_mb->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

/* Handle frame encoding interrupt */
static int __mfc_handle_stream(struct mfc_core *core, struct mfc_ctx *ctx, unsigned int reason)
{
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int slice_type, consumed_only = 0;
	unsigned int strm_size;
	unsigned int pic_count;
	unsigned int sbwc_err;

	slice_type = mfc_core_get_enc_slice_type();
	strm_size = mfc_core_get_enc_strm_size();
	pic_count = mfc_core_get_enc_pic_count();

	mfc_perf_trace(ctx, "type", slice_type);
	mfc_perf_trace(ctx, "size", strm_size);
	mfc_perf_trace(ctx, "count", pic_count);

	mfc_debug(2, "[STREAM] encoded slice type: %d, size: %d, display order: %d\n",
			slice_type, strm_size, pic_count);

	/* clear SBWC enable */
	mfc_core_clear_enc_src_sbwc(core);

	/* buffer full handling */
	if (enc->buf_full) {
		mfc_change_state(core_ctx, MFCINST_ABORT_INST);
		return 0;
	}
	if (core_ctx->state == MFCINST_RUNNING_BUF_FULL)
		mfc_change_state(core_ctx, MFCINST_RUNNING);

	/* set encoded frame type */
	enc->frame_type = slice_type;
	ctx->sequence++;

	if (slice_type == MFC_REG_E_SLICE_TYPE_I && reason == MFC_REG_R2H_CMD_FRAME_DONE_RET) {
		mfc_debug(2, "[FRAME] first frame with two pass for initial qpe is done\n");
		enc->nal_q_disable_for_qpe_two_pass = false;
	}

	if (enc->in_slice) {
		if (mfc_is_queue_count_same(&ctx->buf_queue_lock, &ctx->dst_buf_queue, 0)) {
			mfc_clear_bit(ctx->num, &core->work_bits);
		}
		return 0;
	}

	if (strm_size == 0 && !(enc->empty_data && reason == MFC_REG_R2H_CMD_COMPLETE_SEQ_RET)) {
		mfc_debug(2, "[FRAME] dst buffer is not returned\n");
		consumed_only = 1;
	}

	sbwc_err = mfc_core_get_enc_comp_err();
	if (sbwc_err) {
		mfc_ctx_err("[SBWC] Compressor error detected (Source: %d, DPB: %d)\n",
				(sbwc_err >> 1) & 0x1, sbwc_err & 0x1);
		mfc_ctx_err("[SBWC] sbwc: %d, lossy: %d(%d), option: %d, FORMAT: %#x, OPTIONS: %#x\n",
				ctx->is_sbwc, ctx->is_sbwc_lossy,
				ctx->sbwcl_ratio, enc->sbwc_option,
				MFC_CORE_READL(MFC_REG_PIXEL_FORMAT),
				MFC_CORE_READL(MFC_REG_E_ENC_OPTIONS));
	}

	/* handle source buffer */
	__mfc_handle_stream_input(core, ctx, consumed_only);

	/* handle destination buffer */
	if (enc->empty_data && reason == MFC_REG_R2H_CMD_COMPLETE_SEQ_RET) {
		enc->empty_data = 0;
		mfc_debug(2, "[FRAME] handle EOS for empty data\n");
		__mfc_handle_stream_last_output(core, ctx);
	} else {
		__mfc_handle_stream_output(core, ctx, slice_type, strm_size);
	}

	if (regression_option)
		mfc_core_enc_save_regression_result(core);

	return 0;
}

static inline int __mfc_handle_done_frame(struct mfc_core *core,
		struct mfc_ctx *ctx, unsigned int reason, unsigned int err)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_enc *enc = NULL;

	if (ctx->type == MFCINST_DECODER) {
		if (sfr_dump & MFC_DUMP_DEC_FRAME_DONE)
			call_dop(core, dump_regs, core);
		if (core_ctx->state == MFCINST_SPECIAL_PARSING_NAL) {
			mfc_core_clear_int();
			mfc_core_pm_clock_off(core);
			mfc_clear_bit(ctx->num, &core->work_bits);
			mfc_change_state(core_ctx, MFCINST_RUNNING);
			mfc_wake_up_core_ctx(core_ctx, reason, err);
			return 0;
		}
		__mfc_handle_frame(core, ctx, reason, err);
	} else if (ctx->type == MFCINST_ENCODER) {
		if (sfr_dump & MFC_DUMP_ENC_FRAME_DONE)
			call_dop(core, dump_regs, core);
		if (ctx->otf_handle) {
			mfc_core_otf_handle_stream(core, ctx);
			return 1;
		}
		enc = ctx->enc_priv;
		if (reason == MFC_REG_R2H_CMD_SLICE_DONE_RET) {
			core->preempt_core_ctx = ctx->num;
			enc->buf_full = 0;
			enc->in_slice = 1;
		} else if (reason == MFC_REG_R2H_CMD_ENC_BUFFER_FULL_RET) {
			mfc_err("stream buffer size(%d) isn't enough\n",
					mfc_core_get_enc_strm_size());
			core->preempt_core_ctx = ctx->num;
			enc->buf_full = 1;
			enc->in_slice = 0;
		} else {
			enc->buf_full = 0;
			enc->in_slice = 0;
		}
		__mfc_handle_stream(core, ctx, reason);
	}

	return 1;
}

/* Handle header decoder interrupt */
static int __mfc_handle_seq_dec(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_buf *src_mb;
	int i, is_interlace, is_hdr10_sbwc_off = 0;
	unsigned int strm_size, consumed;

	if (ctx->src_fmt->fourcc != V4L2_PIX_FMT_FIMV1) {
		ctx->img_width = mfc_core_get_img_width();
		ctx->img_height = mfc_core_get_img_height();
		ctx->crop_width = ctx->img_width;
		ctx->crop_height = ctx->img_height;
		mfc_ctx_info("[STREAM] resolution w: %d, h: %d\n",
				ctx->img_width, ctx->img_height);
	}

	if (IS_AV1_DEC(ctx) || (IS_VP9_DEC(ctx) && UNDER_4K_RES(ctx)))
		ctx->dpb_count = mfc_core_get_dpb_count() + 7 - MFC_EXTRA_DPB;
	else
		ctx->dpb_count = mfc_core_get_dpb_count();

	ctx->scratch_buf_size = mfc_core_get_scratch_size();

	mfc_core_dec_get_crop_info(core, ctx);
	dec->mv_count = mfc_core_get_mv_count();
	if (CODEC_10BIT(ctx) && dev->pdata->support_10bit) {
		if (mfc_core_get_luma_bit_depth_minus8() ||
		mfc_core_get_chroma_bit_depth_minus8()) {
			ctx->is_10bit = 1;
			mfc_ctx_info("[STREAM][10BIT] 10bit contents, profile: %d, depth: %d/%d\n",
				mfc_core_get_profile(),
				mfc_core_get_luma_bit_depth_minus8() + 8,
				mfc_core_get_chroma_bit_depth_minus8() + 8);
		} else {
			ctx->is_10bit = 0;
		}
	}

	if (CODEC_422FORMAT(ctx) && dev->pdata->support_422) {
		if (mfc_core_get_chroma_format() == MFC_REG_D_CHROMA_422) {
			ctx->is_422 = 1;
			mfc_ctx_info("[STREAM] 422 chroma format\n");
		}
	}

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->color_aspect_dec)
			&& dev->pdata->sbwc_dec_hdr10_off)
		if (mfc_core_get_video_signal_type() && mfc_core_get_colour_description())
			is_hdr10_sbwc_off = IS_HDR10(ctx, mfc_core_get_primaries(),
					mfc_core_get_transfer(),
					mfc_core_get_matrix_coeff());

	if (ctx->img_width == 0 || ctx->img_height == 0) {
		mfc_err("[STREAM] wrong resolution w: %d, h: %d\n",
				ctx->img_width, ctx->img_height);
	} else {
		is_interlace = mfc_core_is_interlace_picture();
		dec->is_mbaff = mfc_core_is_mbaff_picture();
		if (is_interlace || dec->is_mbaff)
			dec->is_interlaced = 1;
		mfc_debug(2, "[INTERLACE] interlace: %d, mbaff: %d\n",
				is_interlace, dec->is_mbaff);

		if (dev->pdata->support_sbwc) {
			ctx->is_sbwc = mfc_core_is_sbwc_avail();
			if (ctx->is_sbwc && ((ctx->img_width > dev->pdata->sbwc_dec_max_width) ||
				(ctx->img_height > dev->pdata->sbwc_dec_max_height))) {
				ctx->is_sbwc = 0;
				ctx->sbwc_disabled = 1;
				mfc_debug(2, "[SBWC] disable sbwc, (%dx%d) > (%dx%d)\n",
					ctx->img_width, ctx->img_height,
					dev->pdata->sbwc_dec_max_width, dev->pdata->sbwc_dec_max_height);
			} else if (ctx->is_sbwc && is_hdr10_sbwc_off) {
				ctx->is_sbwc = 0;
				ctx->sbwc_disabled = 1;
				mfc_debug(2, "[SBWC] disable sbwc, HDR10\n");
			} else if (ctx->is_sbwc && sbwc_disable) {
				ctx->is_sbwc = 0;
				ctx->sbwc_disabled = 1;
				mfc_debug(2, "[SBWC] disable sbwc, sbwc_disable command was set\n");
			}
			MFC_TRACE_CORE_CTX("*** is_sbwc %d\n", ctx->is_sbwc);
			mfc_debug(2, "[SBWC] is_sbwc %d\n", ctx->is_sbwc);
		}
	}

	for (i = 0; i < ctx->dst_fmt->num_planes; i++) {
		ctx->min_dpb_size[i] = mfc_core_get_min_dpb_size(i);
		if (IS_2BIT_NEED(ctx))
			ctx->min_dpb_size_2bits[i] =
				mfc_core_get_min_dpb_size_2bit(i);
		mfc_debug(2, "[FRAME] min_dpb_size[%d]: %d, min_dpb_size_2bits[%d]: %d\n",
				i, ctx->min_dpb_size[i], i, ctx->min_dpb_size_2bits[i]);
	}

	if (core->has_llc && core->llc_on_status)
		mfc_llc_handle_resol(core, ctx);

	if (IS_MULTI_CORE_DEVICE(dev) && mfc_core_get_two_core_mode()) {
		if (feature_option & MFC_OPTION_MULTI_CORE_DISABLE) {
			mfc_ctx_info("[2CORE] op_mode: %d stream, but multi core disable\n",
					mfc_core_get_two_core_mode());
		} else {
			if (dev->num_inst > 1)
				mfc_debug(2, "[2CORE] multi core bits: %#lx, num inst: %d\n",
						dev->multi_core_inst_bits, dev->num_inst);
			mfc_change_op_mode(ctx, (enum mfc_op_mode)mfc_core_get_two_core_mode());
			set_bit(ctx->num, &dev->multi_core_inst_bits);
			mfc_ctx_info("[2CORE] This stream need to multi core op_mode(%d)\n",
					ctx->op_mode);
		}
	}

	src_mb = mfc_get_buf(ctx, &core_ctx->src_buf_queue,
			MFC_BUF_NO_TOUCH_USED);
	if (src_mb && (IS_H264_DEC(ctx) || IS_H264_MVC_DEC(ctx) || IS_HEVC_DEC(ctx))) {
		consumed = mfc_core_get_consumed_stream();
		strm_size = mfc_dec_get_strm_size(ctx, src_mb);
		mfc_debug(2, "[STREAM] header size: %d, consumed: %d\n",
				strm_size, consumed);
		if ((consumed > 0) && (strm_size > consumed)) {
			dec->consumed += consumed;
			mfc_debug(2, "[STREAM] there is remained bytes(%d) after header parsing\n",
				(strm_size - consumed));
		} else {
			dec->consumed = 0;
		}
	}

	if (src_mb && IS_MULTI_MODE(ctx)) {
		src_mb = mfc_get_move_buf(ctx, &ctx->src_buf_ready_queue,
				&core_ctx->src_buf_queue,
				MFC_BUF_RESET_USED, MFC_QUEUE_ADD_TOP);
		MFC_TRACE_CORE_CTX("SEQ: Move src[%d] to ready_queue\n", src_mb->src_index);
	}

	dec->frame_display_delay = mfc_core_get_display_delay();
	mfc_debug(2, "[FRAME] display delay for first frame %d\n",
			dec->frame_display_delay);

	if (IS_VP9_DEC(ctx)) {
		dec->color_range = mfc_core_get_color_range();
		dec->color_space = mfc_core_get_color_space();
		mfc_debug(2, "color range: %d, color space: %d, It's valid for VP9\n",
				dec->color_range, dec->color_space);
	}

	if (IS_AV1_DEC(ctx)) {
		dec->av1_film_grain_present = mfc_core_get_av1_filmgrain_present();
		mfc_debug(2, "[FILMGR] AV1 Film Grain presented in header: %d\n",
			dec->av1_film_grain_present);
	}

	mfc_change_state(core_ctx, MFCINST_HEAD_PARSED);

	if (regression_option)
		mfc_core_dec_save_regression_result(core);

	return 0;
}

/* Handle header encoder interrupt */
static int __mfc_handle_seq_enc(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_enc_params *p = &enc->params;
	struct mfc_buf *dst_mb;
	int index;

	enc->header_size = mfc_core_get_enc_strm_size();
	mfc_debug(2, "[STREAM] encoded slice type: %d, header size: %d, display order: %d\n",
			mfc_core_get_enc_slice_type(), enc->header_size,
			mfc_core_get_enc_pic_count());

	if (core->has_llc && core->llc_on_status)
		mfc_llc_handle_resol(core, ctx);

	if (IS_BPG_ENC(ctx)) {
		dst_mb = mfc_get_buf(ctx, &ctx->dst_buf_queue,
				MFC_BUF_NO_TOUCH_USED);
		if (!dst_mb) {
			mfc_err("no dst buffers\n");
			return -EAGAIN;
		}

		dst_mb->vb.vb2_buf.planes[0].data_offset += (enc->header_size +
					p->codec.bpg.thumb_size +
					p->codec.bpg.exif_size);
		mfc_debug(2, "offset for NAL_START: %d (header: %d + thumb: %d + exif: %d)\n",
				dst_mb->vb.vb2_buf.planes[0].data_offset,
				enc->header_size,
				p->codec.bpg.thumb_size,
				p->codec.bpg.exif_size);
	} else {
		if (!IS_NO_HEADER_GENERATE(ctx, p)) {
			dst_mb = mfc_get_del_buf(ctx, &ctx->dst_buf_queue,
					MFC_BUF_NO_TOUCH_USED);
			if (!dst_mb) {
				mfc_err("no dst buffers\n");
				return -EAGAIN;
			}

			vb2_set_plane_payload(&dst_mb->vb.vb2_buf, 0,
					mfc_core_get_enc_strm_size());

			index = dst_mb->vb.vb2_buf.index;
			if (call_cop(ctx, core_get_buf_ctrls_val, core, ctx,
						&ctx->dst_ctrls[index]) < 0)
				mfc_err("failed in core_get_buf_ctrls_val\n");
			call_cop(ctx, get_buf_update_val, ctx,
					&ctx->dst_ctrls[index],
					V4L2_CID_MPEG_MFC51_VIDEO_FRAME_TAG,
					HEADER_TAG);

			vb2_buffer_done(&dst_mb->vb.vb2_buf, VB2_BUF_STATE_DONE);
		}
	}

	ctx->dpb_count = mfc_core_get_enc_dpb_count();
	ctx->scratch_buf_size = mfc_core_get_enc_scratch_size();
	ctx->min_dpb_size[0] = mfc_core_get_enc_luma_size();
	ctx->min_dpb_size[1] = mfc_core_get_enc_chroma_size();

	/* If the ROI is enabled at SEQ_START, clear ROI_ENABLE bit */
	mfc_core_clear_roi_enable(core);

	mfc_change_state(core_ctx, MFCINST_HEAD_PARSED);

	return 0;
}

static inline void __mfc_handle_nal_abort(struct mfc_core *core,
				struct mfc_ctx *ctx, unsigned int reason)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_enc *enc = ctx->enc_priv;

	if (ctx->type == MFCINST_ENCODER) {
		mfc_change_state(core_ctx, MFCINST_RUNNING_BUF_FULL);
		enc->buf_full = 0;
		if (IS_VP8_ENC(ctx))
			mfc_err("stream buffer size isn't enough\n");
		__mfc_handle_stream(core, ctx, reason);
	} else {
		mfc_change_state(core_ctx, MFCINST_ABORT);
	}
}

irqreturn_t mfc_core_top_half_irq(int irq, void *priv)
{
	struct mfc_core *core = priv;
	struct mfc_core_ctx *core_ctx;
	struct mfc_ctx *ctx;
	unsigned int err;
	unsigned int reason;

	core_ctx = core->core_ctx[core->curr_core_ctx];
	if (!core_ctx) {
		mfc_core_err("no mfc context to run\n");
		return IRQ_WAKE_THREAD;
	}

	ctx = core_ctx->ctx;
	reason = mfc_core_get_int_reason();
	err = mfc_core_get_int_err();

	core->last_int = reason;
	core->last_int_time = ktime_to_timespec64(ktime_get());

	if ((reason == MFC_REG_R2H_CMD_SEQ_DONE_RET) ||
			(reason == MFC_REG_R2H_CMD_INIT_BUFFERS_RET) ||
			(reason == MFC_REG_R2H_CMD_FRAME_DONE_RET) ||
			(reason == MFC_REG_R2H_CMD_QUEUE_DONE_RET))
		ctx->frame_cnt++;

	mfc_core_debug(2, "[c:%d] Int reason: %d (err: %d)\n",
			core->curr_core_ctx, reason, err);
	MFC_TRACE_CORE_CTX("<< INT(top): %d\n", reason);
	MFC_TRACE_LOG_CORE("I%d", reason);

	mfc_perf_measure_off(core);

	mfc_perf_trace(ctx, "irq", reason);

	if (reason == MFC_REG_R2H_CMD_FRAME_DONE_RET) {
		mfc_perf_trace(ctx, "frame", 0);
	}

	/* Reset trace counter when NAL QUEUE be stopped */
	if (reason == MFC_REG_R2H_CMD_COMPLETE_QUEUE_RET) {
		mfc_perf_trace(ctx, "frame", 0);
		ctx->nal_q_cnt = 0;
	}

	return IRQ_WAKE_THREAD;
}

/*
 * Return value description
 *  0: NAL-Q is handled successfully
 *  1: NAL_START command
 * -1: Error
*/
static inline int __mfc_nal_q_irq(struct mfc_core *core,
		unsigned int reason, unsigned int err)
{
	int ret = -1;
	unsigned int errcode;
	int ctx_num;

	nal_queue_handle *nal_q_handle = core->nal_q_handle;
	EncoderOutputStr *pOutStr;

	switch (reason) {
	case MFC_REG_R2H_CMD_QUEUE_DONE_RET:
		pOutStr = mfc_core_nal_q_dequeue_out_buf(core,
			nal_q_handle->nal_q_out_handle, &errcode);
		if (pOutStr) {
			if (mfc_core_nal_q_handle_out_buf(core, pOutStr))
				mfc_core_err("[NALQ] Failed to handle out buf\n");
		} else {
			mfc_core_err("[NALQ] pOutStr is NULL\n");
		}

		ctx_num = nal_q_handle->nal_q_out_handle->nal_q_ctx;

		if (nal_q_handle->nal_q_exception)
			mfc_set_bit(ctx_num, &core->work_bits);

		mfc_core_clear_int();

		if (!nal_q_handle->nal_q_exception)
			mfc_core_nal_q_clock_off(core, nal_q_handle, ctx_num);

		if (ctx_num < 0)
			mfc_core_err("[NALQ] Can't find ctx in nal q\n");
		else
			mfc_ctx_ready_set_bit(core->core_ctx[ctx_num], &core->work_bits);

		ret = 0;
		break;
	case MFC_REG_R2H_CMD_COMPLETE_QUEUE_RET:
		mfc_core_meerkat_stop_tick(core);
		nal_q_handle->nal_q_state = NAL_Q_STATE_CREATED;
		MFC_TRACE_CORE("** NAL Q state : %d\n", nal_q_handle->nal_q_state);
		mfc_core_debug(2, "[NALQ] return to created state\n");
		mfc_core_nal_q_cleanup_queue(core);
		mfc_core_nal_q_cleanup_clock(core);
		mfc_core_clear_int();
		mfc_core_pm_clock_off(core);
		mfc_wake_up_core(core, reason, err);

		ret = 0;
		break;
	default:
		if (nal_q_handle->nal_q_state == NAL_Q_STATE_STARTED ||
			nal_q_handle->nal_q_state == NAL_Q_STATE_STOPPED) {
			mfc_core_err("[NALQ] Should not be here! state: %d, int reason : %d\n",
				nal_q_handle->nal_q_state, reason);
			mfc_core_clear_int();
			mfc_core_handle_error(core);

			ret = -1;
		} else {
			/* NAL START */
			ret = 1;
		}

		break;
	}

	if (ret == 0)
		queue_work(core->butler_wq, &core->butler_work);

	return ret;
}

static int __mfc_irq_dev(struct mfc_core *core, unsigned int reason, unsigned int err)
{
	/* Stop the timeout meerkat */
	if (reason != MFC_REG_R2H_CMD_FW_STATUS_RET)
		mfc_core_meerkat_stop_tick(core);

	switch (reason) {
	case MFC_REG_R2H_CMD_CACHE_FLUSH_RET:
	case MFC_REG_R2H_CMD_SYS_INIT_RET:
	case MFC_REG_R2H_CMD_FW_STATUS_RET:
	case MFC_REG_R2H_CMD_SLEEP_RET:
	case MFC_REG_R2H_CMD_WAKEUP_RET:
		mfc_core_clear_int();
		mfc_wake_up_core(core, reason, err);
		return 0;
	}

	return 1;
}

static int __mfc_irq_ctx(struct mfc_core *core, struct mfc_ctx *ctx,
		unsigned int reason, unsigned int err)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];

	switch (reason) {
	case MFC_REG_R2H_CMD_ERR_RET:
		if (ctx->otf_handle) {
			mfc_core_otf_handle_error(core, ctx, reason, err);
			break;
		}
		/* An error has occured */
		if (core_ctx->state == MFCINST_RUNNING || core_ctx->state == MFCINST_ABORT) {
			if ((mfc_get_err(err) >= MFC_REG_ERR_FRAME_CONCEAL) &&
				(mfc_get_err(err) <= MFC_REG_ERR_WARNINGS_END))
				__mfc_handle_frame(core, ctx, reason, err);
			else
				__mfc_handle_frame_error(core, ctx, reason, err);
		} else {
			__mfc_handle_error(core, ctx, reason, err);
		}
		break;
	case MFC_REG_R2H_CMD_SLICE_DONE_RET:
	case MFC_REG_R2H_CMD_FIELD_DONE_RET:
	case MFC_REG_R2H_CMD_FRAME_DONE_RET:
	case MFC_REG_R2H_CMD_ENC_BUFFER_FULL_RET:
		return __mfc_handle_done_frame(core, ctx, reason, err);
	case MFC_REG_R2H_CMD_COMPLETE_SEQ_RET:
		if (ctx->type == MFCINST_ENCODER) {
			__mfc_handle_stream(core, ctx, reason);
			mfc_change_state(core_ctx, MFCINST_FINISHED);
		} else if (ctx->type == MFCINST_DECODER) {
			return __mfc_handle_done_frame(core, ctx, reason, err);
		}
		break;
	case MFC_REG_R2H_CMD_SEQ_DONE_RET:
		if (ctx->type == MFCINST_ENCODER) {
			if (ctx->otf_handle) {
				mfc_core_otf_handle_seq(core, ctx);
				break;
			}
			__mfc_handle_seq_enc(core, ctx);
		} else if (ctx->type == MFCINST_DECODER) {
			__mfc_handle_seq_dec(core, ctx);
		}
		break;
	case MFC_REG_R2H_CMD_OPEN_INSTANCE_RET:
		core_ctx->inst_no = mfc_core_get_inst_no();
		mfc_change_state(core_ctx, MFCINST_GOT_INST);
		break;
	case MFC_REG_R2H_CMD_CLOSE_INSTANCE_RET:
		mfc_change_state(core_ctx, MFCINST_FREE);
		break;
	case MFC_REG_R2H_CMD_NAL_ABORT_RET:
		__mfc_handle_nal_abort(core, ctx, reason);
		break;
	case MFC_REG_R2H_CMD_DPB_FLUSH_RET:
		mfc_change_state(core_ctx, MFCINST_ABORT);
		break;
	case MFC_REG_R2H_CMD_INIT_BUFFERS_RET:
		if (err != 0) {
			mfc_err("INIT_BUFFERS_RET error: %d\n", err);
			break;
		}

		mfc_change_state(core_ctx, MFCINST_RUNNING);
		if (ctx->type == MFCINST_DECODER) {
			if (ctx->is_dpb_realloc)
				ctx->is_dpb_realloc = 0;
		}

		if (ctx->otf_handle && (feature_option & MFC_OPTION_OTF_PATH_TEST_ENABLE))
			mfc_core_otf_path_test(ctx);

		break;
	case MFC_REG_R2H_CMD_MOVE_INSTANCE_RET:
		if (sfr_dump & MFC_DUMP_MOVE_INSTANCE_RET)
			call_dop(core, dump_regs, core);
		break;
	default:
		mfc_err("Unknown int reason: %d\n", reason);
		mfc_core_handle_error(core);
	}

	return 1;
}

/* Interrupt processing */
irqreturn_t mfc_core_irq(int irq, void *priv)
{
	struct mfc_core *core = priv;
	struct mfc_core_ctx *core_ctx;
	struct mfc_ctx *ctx;
	unsigned int reason;
	unsigned int err;
	int ret = -1;

	mfc_core_debug_enter();

	if (mfc_core_pm_get_pwr_ref_cnt(core) == 0) {
		snprintf(core->crash_info, MFC_CRASH_INFO_LEN, "no mfc power on\n");
		mfc_core_err("%s", core->crash_info);
		call_dop(core, dump_and_stop_debug_mode, core);
		goto irq_end;
	}

	/* Get the reason of interrupt and the error code */
	reason = mfc_core_get_int_reason();
	err = mfc_core_get_int_err();
	mfc_core_debug(1, "[c:%d] Int reason: %d (err: %d, warn: %d)\n",
			core->curr_core_ctx, reason,
			mfc_get_err(err), mfc_get_warn(err));
	MFC_TRACE_CORE("<< INT: %d (err: %d)\n", reason, err);

	core->preempt_core_ctx = MFC_NO_INSTANCE_SET;

	if (dbg_enable && (reason != MFC_REG_R2H_CMD_QUEUE_DONE_RET))
		mfc_core_dbg_disable(core);

	if ((sfr_dump & MFC_DUMP_ERR_INT) && (reason == MFC_REG_R2H_CMD_ERR_RET))
		call_dop(core, dump_regs, core);

	if ((sfr_dump & MFC_DUMP_WARN_INT) &&
			(err && (reason != MFC_REG_R2H_CMD_ERR_RET)))
		call_dop(core, dump_regs, core);

	if (__mfc_core_is_err_condition(err)) {
		snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
			"MFC is in err:%d, so calling SSR\n", err);
		call_dop(core, dump_and_stop_debug_mode, core);
	}

	if (core->nal_q_handle) {
		ret = __mfc_nal_q_irq(core, reason, err);
		if (ret == 0) {
			mfc_core_debug(2, "[NALQ] command was handled\n");
			goto irq_end;
		} else if (ret == 1){
			/* Path through */
			mfc_core_debug(2, "NAL_START command will be handled\n");
		} else {
			mfc_core_debug(2, "[NALQ] command handling Error\n");
			goto irq_end;
		}
	}

	ret = __mfc_irq_dev(core, reason, err);
	if (!ret)
		goto irq_end;

	core_ctx = core->core_ctx[core->curr_core_ctx];
	if (!core_ctx) {
		mfc_core_err("no mfc context to run\n");
		mfc_core_clear_int();
		mfc_core_pm_clock_off(core);
		goto irq_end;
	}
	ctx = core_ctx->ctx;

	ret = mfc_get_core_intlock(core_ctx);
	if (ret) {
		mfc_core_clear_int_only();
		mfc_core_meerkat_start_tick(core);
		goto irq_end;
	}

	ret = __mfc_irq_ctx(core, ctx, reason, err);
	if (!ret)
		goto irq_end;

	/* clean-up interrupt */
	mfc_core_clear_int();

	mfc_release_core_intlock(core_ctx);

	if (core_ctx->state != MFCINST_RES_CHANGE_INIT)
		mfc_ctx_ready_clear_bit(core_ctx, &core->work_bits);

	if (ctx->otf_handle) {
		if (mfc_core_otf_ctx_ready_set_bit(core_ctx, &core->work_bits) == 0)
			mfc_core_otf_ctx_ready_clear_bit(core_ctx, &core->work_bits);
	}

	mfc_core_hwlock_handler_irq(core, ctx, reason, err);

irq_end:
	mfc_core_debug_leave();
	return IRQ_HANDLED;
}
