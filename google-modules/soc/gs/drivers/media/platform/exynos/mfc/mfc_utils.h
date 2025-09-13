/*
 * drivers/media/platform/exynos/mfc/mfc_utils.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_UTILS_H
#define __MFC_UTILS_H __FILE__

#include "mfc_common.h"

/* bit operation */
#define mfc_clear_bits(reg, mask, shift)	(reg &= ~(mask << shift))
#define mfc_set_bits(reg, mask, shift, value)	(reg |= (value & mask) << shift)
#define mfc_clear_set_bits(reg, mask, shift, value)	\
	do {						\
		reg &= ~(mask << shift);		\
		reg |= (value & mask) << shift;		\
	} while (0)

#define mfc_get_upper(x)	(((unsigned long)(x) >> 32) & 0xffffffff)
#define mfc_get_lower(x)	((x) & 0xffffffff)

static inline void mfc_core_clean_dev_int_flags(struct mfc_core *core)
{
	core->int_condition = 0;
	core->int_reason = 0;
	core->int_err = 0;
}

static inline void mfc_clean_core_ctx_int_flags(struct mfc_core_ctx *core_ctx)
{
	core_ctx->int_condition = 0;
	core_ctx->int_reason = 0;
	core_ctx->int_err = 0;
}

static inline void mfc_change_state(struct mfc_core_ctx *core_ctx, enum mfc_inst_state state)
{
	MFC_TRACE_CORE_CTX("** state : %d\n", state);
	core_ctx->state = state;
}

static inline void mfc_change_op_mode(struct mfc_ctx *ctx, enum mfc_op_mode op_mode)
{
	MFC_TRACE_CTX("** op_mode : %d\n", op_mode);
	ctx->op_mode = op_mode;
}

static inline void mfc_core_change_state(struct mfc_core *core, enum mfc_core_state state)
{
	MFC_TRACE_CORE("** core state : %d\n", state);
	core->state = state;
}

static inline void mfc_core_change_attribute(struct mfc_core *core, int is_drm)
{
	MFC_TRACE_CORE("** ctx_is_drm %d -> %d\n",
			core->curr_core_ctx_is_drm, is_drm);
	mfc_core_debug(3, "curr_core_ctx_is_drm %d -> %d\n",
			core->curr_core_ctx_is_drm, is_drm);
	core->curr_core_ctx_is_drm = is_drm;
}

static inline void mfc_core_change_fw_state(struct mfc_core *core, int is_drm,
		enum mfc_fw_status state, int set)
{
	enum mfc_fw_status prev_stat;

	if (is_drm) {
		prev_stat = core->fw.drm_status;
		if (set)
			core->fw.drm_status |= state;
		else
			core->fw.drm_status &= ~state;
		MFC_TRACE_CORE("** DRM FW status %#x -> %#x (%s: %#x)\n",
				prev_stat, core->fw.drm_status, set ? "set" : "clear", state);
		mfc_core_debug(2, "[F/W] DRM status: %#x -> %#x (%s: %#x)\n",
				prev_stat, core->fw.drm_status, set ? "set" : "clear", state);
	} else {
		prev_stat = core->fw.status;
		if (set)
			core->fw.status |= state;
		else
			core->fw.status &= ~state;
		MFC_TRACE_CORE("** normal FW status %#x -> %#x (%s: %#x)\n",
				prev_stat, core->fw.status, set ? "set" : "clear", state);
		mfc_core_debug(2, "[F/W] normal status: %#x -> %#x (%s: %#x)\n",
				prev_stat, core->fw.status, set ? "set" : "clear", state);
	}
}

static inline enum mfc_node_type mfc_get_node_type(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct mfc_dev *dev;
	enum mfc_node_type node_type;

	if (!vdev) {
		mfc_pr_err("mfc failed to get video_device\n");
		return MFCNODE_INVALID;
	}
	dev = video_drvdata(file);

	mfc_dev_debug(2, "video_device index: %d\n", vdev->index);

	switch (vdev->index) {
	case 0:
		node_type = MFCNODE_DECODER;
		break;
	case 1:
		node_type = MFCNODE_ENCODER;
		break;
	case 2:
		node_type = MFCNODE_DECODER_DRM;
		break;
	case 3:
		node_type = MFCNODE_ENCODER_DRM;
		break;
	case 4:
		node_type = MFCNODE_ENCODER_OTF;
		break;
	case 5:
		node_type = MFCNODE_ENCODER_OTF_DRM;
		break;
	default:
		node_type = MFCNODE_INVALID;
		break;
	}

	return node_type;
}

static inline int mfc_is_decoder_node(enum mfc_node_type node)
{
	if (node == MFCNODE_DECODER || node == MFCNODE_DECODER_DRM)
		return 1;

	return 0;
}

static inline int mfc_is_drm_node(enum mfc_node_type node)
{
	if (node == MFCNODE_DECODER_DRM || node == MFCNODE_ENCODER_DRM ||
			node == MFCNODE_ENCODER_OTF_DRM)
		return 1;

	return 0;
}

static inline int mfc_is_encoder_otf_node(enum mfc_node_type node)
{
	if (node == MFCNODE_ENCODER_OTF || node == MFCNODE_ENCODER_OTF_DRM)
		return 1;

	return 0;
}

static inline void mfc_clear_mb_flag(struct mfc_buf *mfc_buf)
{
	mfc_buf->flag = 0;
}

static inline void mfc_set_mb_flag(struct mfc_buf *mfc_buf, enum mfc_mb_flag f)
{
	mfc_buf->flag |= (1 << f);
}

static inline int mfc_check_mb_flag(struct mfc_buf *mfc_buf, enum mfc_mb_flag f)
{
	if (mfc_buf->flag & (1 << f))
		return 1;

	return 0;
}

int mfc_check_vb_with_fmt(struct mfc_fmt *fmt, struct vb2_buffer *vb);
unsigned int mfc_get_uncomp_format(struct mfc_ctx *ctx, u32 org_fmt);
void mfc_set_linear_stride_size(struct mfc_ctx *ctx, struct mfc_raw_info *raw, struct mfc_fmt *fmt);
void mfc_dec_calc_dpb_size(struct mfc_ctx *ctx, struct mfc_raw_info *raw, struct mfc_fmt *fmt);
void mfc_enc_calc_src_size(struct mfc_ctx *ctx);
void mfc_calc_base_addr(struct mfc_ctx *ctx, struct vb2_buffer *vb, struct mfc_fmt *fmt);

static inline u32 mfc_dec_get_strm_size(struct mfc_ctx *ctx, struct mfc_buf *src_mb)
{
	struct vb2_plane *vb_plane;
	struct mfc_dec *dec = ctx->dec_priv;
	unsigned int strm_size;

	/*
	 * struct v4l2_plane data_offset is included in bytesused.
	 * So the size of image is bytesused - data_offset from start of the plane.
	 * And the dec->consumed is cumulate-decoded size.
	 */
	vb_plane = &src_mb->vb.vb2_buf.planes[0];
	if (!vb_plane->bytesused) {
		strm_size = 0;
	} else if (vb_plane->bytesused > vb_plane->data_offset) {
		strm_size = vb_plane->bytesused - vb_plane->data_offset;
	} else {
		strm_size = vb_plane->bytesused;
		mfc_ctx_info("[STREAM] invalid offset (bytesused %d, data_offset: %d)\n",
				vb_plane->bytesused, vb_plane->data_offset);
	}

	if (dec->consumed) {
		if (strm_size > dec->consumed) {
			strm_size -= dec->consumed;
		} else {
			mfc_ctx_info("[STREAM] invalid consumed (strm_size: %d, consumed: %d)",
					strm_size, dec->consumed);
		}
	}

	mfc_debug(2, "[STREAM] strm_size: %d (bytesused %d, data_offset %d, consumed %d)\n",
			strm_size, vb_plane->bytesused, vb_plane->data_offset, dec->consumed);
	return strm_size;
}

static inline int mfc_dec_get_strm_offset(struct mfc_ctx *ctx, struct mfc_buf *src_mb)
{
	struct vb2_plane *vb_plane;
	struct mfc_dec *dec = ctx->dec_priv;
	unsigned int offset;

	vb_plane = &src_mb->vb.vb2_buf.planes[0];
	offset = vb_plane->data_offset;
	if (dec->consumed)
		offset += dec->consumed;

	mfc_debug(2, "[STREAM] offset: %d (bytesused %d, data_offset %d, consumed %d)\n",
			offset, vb_plane->bytesused, vb_plane->data_offset, dec->consumed);

	return offset;
}

static inline int mfc_dec_status_decoding(unsigned int dst_frame_status)
{
	if (dst_frame_status == MFC_REG_DEC_STATUS_DECODING_DISPLAY ||
	    dst_frame_status == MFC_REG_DEC_STATUS_DECODING_ONLY)
		return 1;
	return 0;
}

static inline int mfc_dec_status_display(unsigned int dst_frame_status)
{
	if (dst_frame_status == MFC_REG_DEC_STATUS_DISPLAY_ONLY ||
	    dst_frame_status == MFC_REG_DEC_STATUS_DECODING_DISPLAY)
		return 1;

	return 0;
}

/* AV1 bit shift */
static const unsigned char av1_bitmask_shift[16] = {
	0, 8, 16, 24,
	0, 8, 16, 24,
	0, 8, 16, 24,
	0, 8, 16, 24,
};

/* Meerkat interval */
#define MEERKAT_TICK_INTERVAL   1000
/* After how many executions meerkat should assume lock up */
#define MEERKAT_TICK_CNT_TO_START_MEERKAT        5

void mfc_core_meerkat_tick(struct timer_list *t);
void mfc_core_meerkat_start_tick(struct mfc_core *core);
void mfc_core_meerkat_stop_tick(struct mfc_core *core);
void mfc_core_meerkat_reset_tick(struct mfc_core *core);

/* MFC idle checker interval */
#define MFCIDLE_TICK_INTERVAL	1500

void mfc_core_idle_checker(struct timer_list *t);

static inline void mfc_core_idle_checker_start_tick(struct mfc_core *core)
{
	mod_timer(&core->mfc_idle_timer, jiffies +
		msecs_to_jiffies(MFCIDLE_TICK_INTERVAL));
	atomic_set(&core->hw_run_cnt, 0);
	atomic_set(&core->dev->queued_cnt, 0);
}

static inline void mfc_core_change_idle_mode(struct mfc_core *core,
			enum mfc_idle_mode idle_mode)
{
	MFC_TRACE_CORE("** idle mode : %d\n", idle_mode);
	core->idle_mode = idle_mode;

	if (core->idle_mode == MFC_IDLE_MODE_NONE)
		mfc_core_idle_checker_start_tick(core);
}

static inline int mfc_enc_get_ts_delta(struct mfc_ctx *ctx)
{
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_enc_params *p = &enc->params;
	int ts_delta = 0;

	if (!ctx->ts_last_interval) {
		ts_delta = p->rc_framerate_res / p->rc_framerate;
		mfc_debug(3, "[DFR] default delta: %d\n", ts_delta);
	} else {
		/*
		 * FRAME_DELTA specifies the amount of
		 * increment of frame modulo base time.
		 * - delta unit = framerate resolution / fps
		 * - fps = 1000000(usec per sec) / timestamp interval
		 * For the sophistication of calculation, we will divide later.
		 * Excluding H.263, resolution is fixed to 10000,
		 * so thie is also divided into pre-calculated 100.
		 * (Preventing both overflow and calculation duplication)
		 */
		if (IS_H263_ENC(ctx))
			ts_delta = ctx->ts_last_interval *
				p->rc_framerate_res / 1000000;
		else
			ts_delta = ctx->ts_last_interval / 100;
	}
	return ts_delta;
}
#endif /* __MFC_UTILS_H */
