/*
 * drivers/media/platform/exynos/mfc/mfc_cmd.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <trace/events/mfc.h>

#include "mfc_cmd.h"

#include "mfc_perf_measure.h"
#include "mfc_reg_api.h"
#include "mfc_hw_reg_api.h"
#include "mfc_enc_param.h"
#include "mfc_mmcache.h"
#include "mfc_llc.h"
#include "mfc_slc.h"

#include "mfc_utils.h"
#include "mfc_buf.h"

void mfc_cmd_sys_init(struct mfc_dev *dev,
					enum mfc_buf_usage_type buf_type)
{
	struct mfc_ctx_buf_size *buf_size;
	struct mfc_special_buf *ctx_buf;

	mfc_dev_debug_enter();

	mfc_clean_dev_int_flags(dev);

	buf_size = dev->variant->buf_size->ctx_buf;
	ctx_buf = &dev->common_ctx_buf;
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (buf_type == MFCBUF_DRM)
		ctx_buf = &dev->drm_common_ctx_buf;
#endif
	MFC_WRITEL(ctx_buf->daddr, MFC_REG_CONTEXT_MEM_ADDR);
	MFC_WRITEL(buf_size->dev_ctx, MFC_REG_CONTEXT_MEM_SIZE);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_SYS_INIT);

	mfc_dev_debug_leave();
}

void mfc_cmd_sleep(struct mfc_dev *dev)
{
	mfc_dev_debug_enter();

	mfc_clean_dev_int_flags(dev);
	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_SLEEP);

	mfc_dev_debug_leave();
}

void mfc_cmd_wakeup(struct mfc_dev *dev)
{
	mfc_dev_debug_enter();

	mfc_clean_dev_int_flags(dev);
	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_WAKEUP);

	mfc_dev_debug_leave();
}

/* Open a new instance and get its number */
void mfc_cmd_open_inst(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	unsigned int reg;

	mfc_debug_enter();

	/* Preparing decoding - getting instance number */
	mfc_debug(2, "Getting instance number\n");
	mfc_clean_ctx_int_flags(ctx);

	reg = MFC_READL(MFC_REG_CODEC_CONTROL);
	/* Clear OTF_CONTROL[2:1] & OTF_DEBUG[3] */
	reg &= ~(0x7 << 1);
	if (ctx->otf_handle) {
		/* Set OTF_CONTROL[2:1], 0: Non-OTF, 1: OTF+HWFC, 2: OTF only */
		reg |= (0x1 << 1);
		mfc_ctx_info("[OTF] HWFC + OTF enabled\n");
		if (otf_dump && !ctx->is_drm) {
			/* Set OTF_DEBUG[3] for OTF path dump */
			reg |= (0x1 << 3);
			mfc_ctx_info("[OTF] Debugging mode enabled\n");
		}
	}
	MFC_WRITEL(reg, MFC_REG_CODEC_CONTROL);

	mfc_debug(2, "Requested codec mode: %d\n", ctx->codec_mode);
	reg = ctx->codec_mode & MFC_REG_CODEC_TYPE_MASK;
	reg |= (0x1 << MFC_REG_CLEAR_CTX_MEM_SHIFT);
	mfc_debug(2, "Enable to clear context memory: %#x\n", reg);
	MFC_WRITEL(reg, MFC_REG_CODEC_TYPE);

	MFC_WRITEL(ctx->instance_ctx_buf.daddr, MFC_REG_CONTEXT_MEM_ADDR);
	MFC_WRITEL(ctx->instance_ctx_buf.size, MFC_REG_CONTEXT_MEM_SIZE);
	if (ctx->type == MFCINST_DECODER)
		MFC_WRITEL(ctx->dec_priv->crc_enable, MFC_REG_D_CRC_CTRL);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_OPEN_INSTANCE);

	mfc_debug_leave();
}

/* Close instance */
int mfc_cmd_close_inst(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;

	mfc_debug_enter();

	/* Closing decoding instance  */
	mfc_debug(2, "Returning instance number\n");
	mfc_clean_ctx_int_flags(ctx);
	if (ctx->state == MFCINST_FREE) {
		mfc_ctx_err("ctx already free status\n");
		return -EINVAL;
	}

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_CLOSE_INSTANCE);

	mfc_debug_leave();

	return 0;
}

void mfc_cmd_abort_inst(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;

	mfc_clean_ctx_int_flags(ctx);

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);
	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_NAL_ABORT);
}

void mfc_cmd_dpb_flush(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;

	if (ON_RES_CHANGE(ctx))
		mfc_ctx_err("dpb flush on res change(state:%d)\n", ctx->state);

	mfc_clean_ctx_int_flags(ctx);

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);
	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_DPB_FLUSH);
}

void mfc_cmd_cache_flush(struct mfc_dev *dev)
{
	mfc_clean_dev_int_flags(dev);
	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_CACHE_FLUSH);
}

/* Initialize decoding */
void mfc_cmd_dec_seq_header(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_dec *dec = ctx->dec_priv;
	unsigned int reg = 0;
	int fmo_aso_ctrl = 0;

	mfc_debug_enter();

	mfc_debug(2, "InstNo: %d/%d\n", ctx->inst_no, MFC_REG_H2R_CMD_SEQ_HEADER);
	mfc_debug(2, "BUFs: %08x\n", MFC_READL(MFC_REG_D_CPB_BUFFER_ADDR));

	/*
	 * When user sets desplay_delay to 0,
	 * It works as "display_delay enable" and delay set to 0.
	 * If user wants display_delay disable, It should be
	 * set to negative value.
	 */
	if (dec->display_delay >= 0) {
		reg |= (0x1 << MFC_REG_D_DEC_OPT_DISPLAY_DELAY_EN_SHIFT);
		MFC_WRITEL(dec->display_delay, MFC_REG_D_DISPLAY_DELAY);
	}

	/* FMO_ASO_CTRL - 0: Enable, 1: Disable */
	reg |= ((fmo_aso_ctrl & MFC_REG_D_DEC_OPT_FMO_ASO_CTRL_MASK)
			<< MFC_REG_D_DEC_OPT_FMO_ASO_CTRL_SHIFT);

	reg |= ((dec->idr_decoding & MFC_REG_D_DEC_OPT_IDR_DECODING_MASK)
			<< MFC_REG_D_DEC_OPT_IDR_DECODING_SHIFT);

	/* VC1 RCV: Discard to parse additional header as default */
	if (IS_VC1_RCV_DEC(ctx))
		reg |= (0x1 << MFC_REG_D_DEC_OPT_DISCARD_RCV_HEADER_SHIFT);

	/* conceal control to specific color */
	reg |= (0x4 << MFC_REG_D_DEC_OPT_CONCEAL_CONTROL_SHIFT);

	/* Disable parallel processing if nal_q_parallel_disable was set */
	if (nal_q_parallel_disable)
		reg |= (0x2 << MFC_REG_D_DEC_OPT_PARALLEL_DISABLE_SHIFT);

	/* Realloc buffer for resolution decrease case in NAL QUEUE mode */
	reg |= (0x1 << MFC_REG_D_DEC_OPT_REALLOC_CONTROL_SHIFT);

	/* Parsing all including PPS */
	reg |= (0x1 << MFC_REG_D_DEC_OPT_SPECIAL_PARSING_SHIFT);

	/* Enabe decoding order */
	if (dec->decoding_order || (feature_option & MFC_OPTION_DECODING_ORDER))
		reg |= (0x1 << MFC_REG_D_DEC_OPT_DECODING_ORDER_ENABLE);

	MFC_WRITEL(reg, MFC_REG_D_DEC_OPTIONS);

	MFC_WRITEL(MFC_CONCEAL_COLOR, MFC_REG_D_FORCE_PIXEL_VAL);

	if (IS_FIMV1_DEC(ctx)) {
		mfc_debug(2, "Setting FIMV1 resolution to %dx%d\n",
					ctx->img_width, ctx->img_height);
		MFC_WRITEL(ctx->img_width, MFC_REG_D_SET_FRAME_WIDTH);
		MFC_WRITEL(ctx->img_height, MFC_REG_D_SET_FRAME_HEIGHT);
	}

	mfc_set_pixel_format(ctx, ctx->dst_fmt->fourcc);

	reg = 0;
	/* Enable realloc interface if SEI is enabled */
	if (dec->sei_parse)
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_NEED_INIT_BUFFER_SHIFT);
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->static_info_dec)) {
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_CONTENT_LIGHT_SHIFT);
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_MASTERING_DISPLAY_SHIFT);
	}
	reg |= (0x1 << MFC_REG_D_SEI_ENABLE_RECOVERY_PARSING_SHIFT);
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->hdr10_plus))
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_ST_2094_40_SEI_SHIFT);

	MFC_WRITEL(reg, MFC_REG_D_SEI_ENABLE);
	mfc_debug(2, "SEI enable was set, 0x%x\n", MFC_READL(MFC_REG_D_SEI_ENABLE));

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if (sfr_dump & MFC_DUMP_DEC_SEQ_START)
		call_dop(dev, dump_regs, dev);

	if (dec->crc_enable && dev->pdata->support_sbwc &&
			IS_SBWC_FMT(ctx->dst_fmt))
		MFC_WRITEL(0x2, MFC_REG_DBG_INFO_ENABLE);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_SEQ_HEADER);

	mfc_debug_leave();
}

int mfc_cmd_enc_seq_header(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	int ret;

	mfc_debug(2, "++\n");

	ret = mfc_set_enc_params(ctx);
	if (ret) {
		mfc_debug(2, "fail to set enc params\n");
		return ret;
	}

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if (reg_test)
		mfc_set_test_params(dev);

	if (sfr_dump & MFC_DUMP_ENC_SEQ_START)
		call_dop(dev, dump_regs, dev);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_SEQ_HEADER);

	mfc_debug(2, "--\n");

	return 0;
}

int mfc_cmd_dec_init_buffers(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	int ret;

	mfc_set_pixel_format(ctx, ctx->dst_fmt->fourcc);

	mfc_clean_ctx_int_flags(ctx);
	ret = mfc_set_dec_codec_buffers(ctx);
	if (ret) {
		mfc_ctx_info("isn't enough codec buffer size, re-alloc!\n");

		if (dev->has_mmcache && dev->mmcache.is_on_status)
			mfc_invalidate_mmcache(dev);

		if (dev->has_llc && dev->llc_on_status)
			mfc_llc_flush(dev);

		if (dev->has_slc && dev->slc_on_status)
			mfc_slc_flush(dev);

		mfc_release_codec_buffers(ctx);
		ret = mfc_alloc_codec_buffers(ctx);
		if (ret) {
			mfc_ctx_err("Failed to allocate decoding buffers\n");
			return ret;
		}
		ret = mfc_set_dec_codec_buffers(ctx);
		if (ret) {
			mfc_ctx_err("Failed to alloc frame mem\n");
			return ret;
		}
	}

	if (dev->has_slc && dev->slc_on_status) {
		MFC_WRITEL(0x4055100, MFC_REG_D_AXI_RD_ATTR0_SLC);
		MFC_WRITEL(0x4055100, MFC_REG_D_AXI_WR_ATTR0_SLC);
	}

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if (sfr_dump & MFC_DUMP_DEC_INIT_BUFS)
		call_dop(dev, dump_regs, dev);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_INIT_BUFFERS);

	return ret;
}

int mfc_cmd_enc_init_buffers(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	int ret;

	/*
	 * Header was generated now starting processing
	 * First set the reference frame buffers
	 */
	if (!ctx->codec_buffer_allocated) {
		mfc_ctx_info("there isn't codec buffer, re-alloc!\n");
		ret = mfc_alloc_codec_buffers(ctx);
		if (ret) {
			mfc_ctx_err("Failed to allocate encoding buffers\n");
			return ret;
		}
	}

	mfc_clean_ctx_int_flags(ctx);
	ret = mfc_set_enc_codec_buffers(ctx);
	if (ret) {
		mfc_ctx_info("isn't enough codec buffer size, re-alloc!\n");

		if (dev->has_mmcache && dev->mmcache.is_on_status)
			mfc_invalidate_mmcache(dev);

		if (dev->has_llc && dev->llc_on_status)
			mfc_llc_flush(dev);

		if (dev->has_slc && dev->slc_on_status)
			mfc_slc_flush(dev);

		mfc_release_codec_buffers(ctx);
		ret = mfc_alloc_codec_buffers(ctx);
		if (ret) {
			mfc_ctx_err("Failed to allocate encoding buffers\n");
			return ret;
		}
		ret = mfc_set_enc_codec_buffers(ctx);
		if (ret) {
			mfc_ctx_err("Failed to set enc codec buffers\n");
			return ret;
		}
	}

	if (dev->has_slc && dev->slc_on_status) {
		MFC_WRITEL(0x4055100, MFC_REG_E_AXI_RD_ATTR0_SLC);
		MFC_WRITEL(0x4055100, MFC_REG_E_AXI_WR_ATTR0_SLC);
	}

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if (sfr_dump & MFC_DUMP_ENC_INIT_BUFS)
		call_dop(dev, dump_regs, dev);

	mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_INIT_BUFFERS);

	return ret;
}

int __mfc_set_scratch_dpb_buffer(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_raw_info *raw;
	int i, ret;

	if (dev->has_mmcache && dev->mmcache.is_on_status)
		mfc_invalidate_mmcache(dev);

	if (dev->has_llc && dev->llc_on_status)
		mfc_llc_flush(dev);

	if (dev->has_slc && dev->slc_on_status)
		mfc_slc_flush(dev);

	ret = mfc_alloc_scratch_buffer(ctx);
	if (ret) {
		mfc_ctx_err("Failed to allocate scratch buffers\n");
		return ret;
	}

	raw = &ctx->raw_buf;
	/* set decoder DPB size, stride */
	MFC_WRITEL(dec->total_dpb_count, MFC_REG_D_NUM_DPB);
	for (i = 0; i < raw->num_planes; i++) {
		mfc_debug(2, "[FRAME] buf[%d] size: %d, stride: %d\n",
				i, raw->plane_size[i], raw->stride[i]);
		MFC_WRITEL(raw->plane_size[i],
			MFC_REG_D_FIRST_PLANE_DPB_SIZE + (i * 4));
		MFC_WRITEL(ctx->raw_buf.stride[i],
			MFC_REG_D_FIRST_PLANE_DPB_STRIDE_SIZE + (i * 4));
		if (IS_2BIT_NEED(ctx)) {
			MFC_WRITEL(raw->stride_2bits[i],
				MFC_REG_D_FIRST_PLANE_2BIT_DPB_STRIDE_SIZE +
				(i * 4));
			MFC_WRITEL(raw->plane_size_2bits[i],
				MFC_REG_D_FIRST_PLANE_2BIT_DPB_SIZE + (i * 4));
			mfc_debug(2, "[FRAME]%s%s 2bits buf[%d] size: %d, stride: %d\n",
					(ctx->is_10bit ? "[10BIT]" : ""),
					(ctx->is_sbwc ? "[SBWC]" : ""),
					i, raw->plane_size_2bits[i],
					raw->stride_2bits[i]);
		}
	}

	/* set scratch buffers */
	MFC_WRITEL(ctx->scratch_buf.daddr, MFC_REG_D_SCRATCH_BUFFER_ADDR);
	MFC_WRITEL(ctx->scratch_buf_size, MFC_REG_D_SCRATCH_BUFFER_SIZE);
	mfc_debug(2, "[FRAME] scratch buf addr: 0x%#llx size %ld\n",
			ctx->scratch_buf.daddr, ctx->scratch_buf_size);

	return 0;
}

/* Decode a single frame */
int mfc_cmd_dec_one_frame(struct mfc_ctx *ctx, int last_frame)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_dec *dec = ctx->dec_priv;
	u32 reg = 0;
	int ret = 0;

	mfc_debug(2, "[DPB] set dpb: %#lx, used: %#lx, available: %#lx\n",
			dec->dynamic_set, dec->dynamic_used,
			dec->available_dpb);

	reg = MFC_READL(MFC_REG_D_NAL_START_OPTIONS);
	reg &= ~(0x1 << MFC_REG_D_NAL_START_OPT_BLACK_BAR_SHIFT);
	reg |= ((dec->detect_black_bar & 0x1) << MFC_REG_D_NAL_START_OPT_BLACK_BAR_SHIFT);
	if (dec->inter_res_change) {
		ret = __mfc_set_scratch_dpb_buffer(ctx);
		if (ret)
			return ret;
		reg |= (0x1 << MFC_REG_D_NAL_START_OPT_NEW_SCRATCH_SHIFT);
		reg |= (0x1 << MFC_REG_D_NAL_START_OPT_NEW_DPB_SHIFT);
		dec->inter_res_change = 0;
	} else {
		reg &= ~(0x1 << MFC_REG_D_NAL_START_OPT_NEW_SCRATCH_SHIFT);
		reg &= ~(0x1 << MFC_REG_D_NAL_START_OPT_NEW_DPB_SHIFT);
	}
	MFC_WRITEL(reg, MFC_REG_D_NAL_START_OPTIONS);
	mfc_debug(3, "[BLACKBAR] black bar detect set: %#x\n", reg);

	MFC_WRITEL(mfc_get_lower(dec->dynamic_set),
			MFC_REG_D_DYNAMIC_DPB_FLAG_LOWER);
	MFC_WRITEL(mfc_get_upper(dec->dynamic_set),
			MFC_REG_D_DYNAMIC_DPB_FLAG_UPPER);
	MFC_WRITEL(mfc_get_lower(dec->available_dpb),
			MFC_REG_D_AVAILABLE_DPB_FLAG_LOWER);
	MFC_WRITEL(mfc_get_upper(dec->available_dpb),
			MFC_REG_D_AVAILABLE_DPB_FLAG_UPPER);

	MFC_WRITEL(dec->slice_enable, MFC_REG_D_SLICE_IF_ENABLE);
	MFC_WRITEL(MFC_TIMEOUT_VALUE, MFC_REG_DEC_TIMEOUT_VALUE);

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if ((sfr_dump & MFC_DUMP_DEC_NAL_START) && !ctx->check_dump) {
		call_dop(dev, dump_regs, dev);
		ctx->check_dump = 1;
	}

	/*
	 * Issue different commands to instance basing on whether it
	 * is the last frame or not.
	 */
	switch (last_frame) {
	case 0:
		mfc_perf_measure_on(dev);

		mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_NAL_START);
		break;
	case 1:
		mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_LAST_FRAME);
		break;
	}

	mfc_debug(2, "Decoding a usual frame\n");
	return 0;
}

/* Encode a single frame */
void mfc_cmd_enc_one_frame(struct mfc_ctx *ctx, int last_frame)
{
	struct mfc_dev *dev = ctx->dev;

	mfc_debug(2, "++\n");

	MFC_WRITEL(ctx->inst_no, MFC_REG_INSTANCE_ID);

	if ((sfr_dump & MFC_DUMP_ENC_NAL_START) && !ctx->check_dump) {
		call_dop(dev, dump_regs, dev);
		ctx->check_dump = 1;
	}

	/*
	 * Issue different commands to instance basing on whether it
	 * is the last frame or not.
	 */
	switch (last_frame) {
	case 0:
		mfc_perf_measure_on(dev);

		mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_NAL_START);
		break;
	case 1:
		mfc_cmd_host2risc(dev, MFC_REG_H2R_CMD_LAST_FRAME);
		break;
	}

	mfc_debug(2, "--\n");
}
