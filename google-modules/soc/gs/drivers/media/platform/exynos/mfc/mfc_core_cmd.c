/*
 * drivers/media/platform/exynos/mfc/mfc_core_cmd.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <trace/events/mfc.h>

#include "mfc_core_cmd.h"

#include "mfc_perf_measure.h"
#include "mfc_core_reg_api.h"
#include "mfc_core_hw_reg_api.h"
#include "mfc_core_enc_param.h"
#include "mfc_llc.h"
#include "mfc_slc.h"
#include "mfc_memlog.h"

#include "mfc_utils.h"
#include "mfc_buf.h"

void mfc_core_cmd_sys_init(struct mfc_core *core,
					enum mfc_buf_usage_type buf_type)
{
	struct mfc_dev *dev = core->dev;
	struct mfc_ctx_buf_size *buf_size;
	struct mfc_special_buf *ctx_buf;

	mfc_core_debug_enter();

	mfc_core_clean_dev_int_flags(core);

	buf_size = dev->variant->buf_size->ctx_buf;
	ctx_buf = &core->common_ctx_buf;
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (buf_type == MFCBUF_DRM)
		ctx_buf = &core->drm_common_ctx_buf;
#endif
	MFC_CORE_WRITEL(ctx_buf->daddr, MFC_REG_CONTEXT_MEM_ADDR);
	MFC_CORE_WRITEL(buf_size->dev_ctx, MFC_REG_CONTEXT_MEM_SIZE);

	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_SYS_INIT);

	mfc_core_debug_leave();
}

void mfc_core_cmd_sleep(struct mfc_core *core)
{
	mfc_core_debug_enter();

	mfc_core_clean_dev_int_flags(core);
	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_SLEEP);

	mfc_core_debug_leave();
}

void mfc_core_cmd_wakeup(struct mfc_core *core)
{
	mfc_core_debug_enter();

	mfc_core_clean_dev_int_flags(core);
	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_WAKEUP);

	mfc_core_debug_leave();
}

/* Open a new instance and get its number */
void mfc_core_cmd_open_inst(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	unsigned int reg;

	mfc_debug_enter();

	/* Preparing decoding - getting instance number */
	mfc_debug(2, "Getting instance number\n");
	mfc_clean_core_ctx_int_flags(core_ctx);

	reg = MFC_CORE_READL(MFC_REG_CODEC_CONTROL);
	/* Clear OTF_CONTROL[2:1] & OTF_DEBUG[3] */
	reg &= ~(0x7 << 1);
	if (ctx->otf_handle) {
		/* Set OTF_CONTROL[2:1], 0: Non-OTF, 1: OTF+HWFC, 2: OTF only, 3: OTF+vOTF */
		if (feature_option & MFC_OPTION_OTF_PATH_TEST_ENABLE) {
			reg |= (0x2 << 1);
			mfc_ctx_info("[OTF] OTF enabled\n");
		} else if (core->has_dpu_votf && core->has_mfc_votf) {
			reg |= (0x3 << 1);
			mfc_ctx_info("[OTF] vOTF + OTF enabled\n");
		} else if (core->has_hwfc) {
			reg |= (0x1 << 1);
			mfc_ctx_info("[OTF] HWFC + OTF enabled\n");
		}
		if (otf_dump && !ctx->is_drm) {
			/* Set OTF_DEBUG[3] for OTF path dump */
			reg |= (0x1 << 3);
			mfc_ctx_info("[OTF] Debugging mode enabled\n");
		}
	}
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->metadata_interface)) {
		/* Set SECURE_CODEC[4], 0: Normal, 1: Secure */
		if (ctx->is_drm)
			reg |= (0x1 << 4);
		else
			reg &= ~(0x1 << 4);
	}
	MFC_CORE_WRITEL(reg, MFC_REG_CODEC_CONTROL);

	mfc_debug(2, "Requested codec mode: %d\n", ctx->codec_mode);
	reg = ctx->codec_mode & MFC_REG_CODEC_TYPE_MASK;
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->mem_clear)) {
		reg |= (0x1 << MFC_REG_CLEAR_CTX_MEM_SHIFT);
		mfc_debug(2, "Enable to clear context memory: %#x\n", reg);
	}
	MFC_CORE_WRITEL(reg, MFC_REG_CODEC_TYPE);

	MFC_CORE_WRITEL(core_ctx->instance_ctx_buf.daddr, MFC_REG_CONTEXT_MEM_ADDR);
	MFC_CORE_WRITEL(core_ctx->instance_ctx_buf.size, MFC_REG_CONTEXT_MEM_SIZE);
	if (ctx->type == MFCINST_DECODER)
		MFC_CORE_WRITEL(ctx->dec_priv->crc_enable, MFC_REG_D_CRC_CTRL);

	if (feature_option & MFC_OPTION_SET_MULTI_CORE_FORCE) {
		MFC_CORE_WRITEL(0x4, MFC_REG_DBG_INFO_ENABLE);
		mfc_ctx_info("[2CORE] Forcely enable multi core mode\n");
	}

	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_OPEN_INSTANCE);

	mfc_debug_leave();
}

/* Close instance */
int mfc_core_cmd_close_inst(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];

	mfc_debug_enter();

	/* Closing decoding instance  */
	mfc_debug(2, "Returning instance number\n");
	mfc_clean_core_ctx_int_flags(core_ctx);
	if (core_ctx->state == MFCINST_FREE) {
		mfc_err("ctx already free status\n");
		return -EINVAL;
	}

	MFC_CORE_WRITEL(core_ctx->inst_no, MFC_REG_INSTANCE_ID);

	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_CLOSE_INSTANCE);

	mfc_debug_leave();

	return 0;
}

void mfc_core_cmd_abort_inst(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];

	mfc_clean_core_ctx_int_flags(core_ctx);

	MFC_CORE_WRITEL(core_ctx->inst_no, MFC_REG_INSTANCE_ID);
	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_NAL_ABORT);
}

void mfc_core_cmd_move_inst(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];

	MFC_CORE_WRITEL(core_ctx->inst_no, MFC_REG_INSTANCE_ID);
	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_MOVE_INSTANCE);
}

void mfc_core_cmd_dpb_flush(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];

	if (ON_RES_CHANGE(core_ctx))
		mfc_err("dpb flush on res change(state:%d)\n",
				core_ctx->state);

	if (ctx->op_mode == MFC_OP_SWITCH_BUT_MODE2) {
		ctx->op_mode = MFC_OP_SWITCH_TO_SINGLE;
		mfc_debug(2, "[2CORE] change op_mode to %d\n", ctx->op_mode);
		MFC_TRACE_RM("[c:%d] will change op_mode: 5 -> %d\n",
				core_ctx->num, ctx->op_mode);
	}

	mfc_clean_core_ctx_int_flags(core_ctx);

	MFC_CORE_WRITEL(core_ctx->inst_no, MFC_REG_INSTANCE_ID);
	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_DPB_FLUSH);
}

void mfc_core_cmd_cache_flush(struct mfc_core *core)
{
	mfc_core_clean_dev_int_flags(core);
	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_CACHE_FLUSH);
}

/* Initialize decoding */
void mfc_core_cmd_dec_seq_header(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	unsigned int reg = 0;
	int fmo_aso_ctrl = 0;

	mfc_debug_enter();

	mfc_debug(2, "inst_no: %d/%d\n", core_ctx->inst_no, MFC_REG_H2R_CMD_SEQ_HEADER);
	mfc_debug(2, "BUFs: %08x\n", MFC_CORE_READL(MFC_REG_D_CPB_BUFFER_ADDR));

	/*
	 * When user sets desplay_delay to 0,
	 * It works as "display_delay enable" and delay set to 0.
	 * If user wants display_delay disable, It should be
	 * set to negative value.
	 */
	if (dec->display_delay >= 0) {
		reg |= (0x1 << MFC_REG_D_DEC_OPT_DISPLAY_DELAY_EN_SHIFT);
		MFC_CORE_WRITEL(dec->display_delay, MFC_REG_D_DISPLAY_DELAY);
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

	/* Todo: How to check ANNEX B Format feature? */
	/* Enabe ANNEX B Format for AV1 */
#if 0
	if (IS_AV1_DEC(ctx) && ctx->is_av1_annex_b)
		reg |= (0x1 << MFC_REG_D_DEC_OPT_AV1_ANNEX_B_FORMAT_SHIFT);
#endif

	MFC_CORE_WRITEL(reg, MFC_REG_D_DEC_OPTIONS);

	MFC_CORE_WRITEL(MFC_CONCEAL_COLOR, MFC_REG_D_FORCE_PIXEL_VAL);

	if (IS_FIMV1_DEC(ctx)) {
		mfc_debug(2, "Setting FIMV1 resolution to %dx%d\n",
					ctx->img_width, ctx->img_height);
		MFC_CORE_WRITEL(ctx->img_width, MFC_REG_D_SET_FRAME_WIDTH);
		MFC_CORE_WRITEL(ctx->img_height, MFC_REG_D_SET_FRAME_HEIGHT);
	}

	mfc_core_set_pixel_format(core, ctx, ctx->dst_fmt->fourcc);

	reg = 0;
	/* Enable realloc interface if SEI is enabled */
	if (dec->sei_parse)
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_NEED_INIT_BUFFER_SHIFT);
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->static_info_dec)) {
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_CONTENT_LIGHT_SHIFT);
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_MASTERING_DISPLAY_SHIFT);
	}
	reg |= (0x1 << MFC_REG_D_SEI_ENABLE_RECOVERY_PARSING_SHIFT);
	/* If the metadata interface is supported, the SEI interface is not enabled. */
	if (!MFC_FEATURE_SUPPORT(dev, dev->pdata->hdr10_plus_full) &&
			MFC_FEATURE_SUPPORT(dev, dev->pdata->hdr10_plus))
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_ST_2094_40_SEI_SHIFT);

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->av1_film_grain))
		reg |= (0x1 << MFC_REG_D_SEI_ENABLE_FILM_GRAIN_SHIFT);

	MFC_CORE_WRITEL(reg, MFC_REG_D_SEI_ENABLE);
	mfc_debug(2, "SEI enable was set, 0x%x\n", MFC_CORE_READL(MFC_REG_D_SEI_ENABLE));

	/* Enable metadata */
	reg = 0;
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->hdr10_plus_full))
		reg |= (0x1 << MFC_REG_SEI_NAL_ENABLE_SHIFT);
	MFC_CORE_WRITEL(reg, MFC_REG_METADATA_ENABLE);

	MFC_CORE_WRITEL(core_ctx->inst_no, MFC_REG_INSTANCE_ID);

	if (sfr_dump & MFC_DUMP_DEC_SEQ_START)
		call_dop(core, dump_regs, core);

	if (dec->crc_enable && dev->pdata->support_sbwc && IS_SBWC_FMT(ctx->dst_fmt))
		MFC_CORE_WRITEL(0x2, MFC_REG_DBG_INFO_ENABLE);

	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_SEQ_HEADER);

	mfc_debug_leave();
}

int mfc_core_cmd_enc_seq_header(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int ret;

	mfc_debug(2, "++\n");

	ret = mfc_core_set_enc_params(core, ctx);
	if (ret) {
		mfc_debug(2, "fail to set enc params\n");
		return ret;
	}

	MFC_CORE_WRITEL(core_ctx->inst_no, MFC_REG_INSTANCE_ID);

	if (reg_test)
		mfc_core_set_test_params(core);

	if (ctx->gdc_votf && core->has_gdc_votf && core->has_mfc_votf)
		mfc_core_set_gdc_votf(core, ctx);

	if (ctx->otf_handle && core->has_dpu_votf && core->has_mfc_votf)
		mfc_core_set_dpu_votf(core, ctx);

	if (sfr_dump & MFC_DUMP_ENC_SEQ_START)
		call_dop(core, dump_regs, core);

	if ((core->memlog.sfr_enable) && (logging_option & MFC_LOGGING_MEMLOG_SFR_DUMP))
		memlog_do_dump(core->memlog.sfr_obj, MEMLOG_LEVEL_INFO);

	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_SEQ_HEADER);

	mfc_debug(2, "--\n");

	return 0;
}


static void __mfc_core_set_slc_option(struct mfc_core *core, struct mfc_ctx *ctx)
{
	unsigned int reg = 0;
	unsigned int mfc_reg_axi_wr_attr0_slc = 0;
	unsigned int mfc_reg_axi_wr_attr1_slc = 0;
	unsigned int mfc_reg_axi_rd_attr0_slc = 0;
	unsigned int mfc_reg_axi_rd_attr1_slc = 0;
	unsigned int mfc_reg_lf_pixel_posx_slc = 0;
	unsigned int mfc_reg_lf_pixel_posy_slc = 0;

	if (ctx->type == MFCINST_INVALID)
		return;

	if (ctx->type == MFCINST_DECODER) {
		mfc_reg_axi_wr_attr0_slc = MFC_REG_D_AXI_WR_ATTR0_SLC;
		mfc_reg_axi_wr_attr1_slc = MFC_REG_D_AXI_WR_ATTR1_SLC;
		mfc_reg_axi_rd_attr0_slc = MFC_REG_D_AXI_RD_ATTR0_SLC;
		mfc_reg_axi_rd_attr1_slc = MFC_REG_D_AXI_RD_ATTR1_SLC;
		mfc_reg_lf_pixel_posx_slc = MFC_REG_D_LF_PIXEL_POSX_SLC;
		mfc_reg_lf_pixel_posy_slc = MFC_REG_D_LF_PIXEL_POSY_SLC;
	} else if (ctx->type == MFCINST_ENCODER) {
		mfc_reg_axi_wr_attr0_slc = MFC_REG_E_AXI_WR_ATTR0_SLC;
		mfc_reg_axi_wr_attr1_slc = MFC_REG_E_AXI_WR_ATTR1_SLC;
		mfc_reg_axi_rd_attr0_slc = MFC_REG_E_AXI_RD_ATTR0_SLC;
		mfc_reg_axi_rd_attr1_slc = MFC_REG_E_AXI_RD_ATTR1_SLC;
		mfc_reg_lf_pixel_posx_slc = MFC_REG_E_LF_PIXEL_POSX_SLC;
		mfc_reg_lf_pixel_posy_slc = MFC_REG_E_LF_PIXEL_POSY_SLC;
	}

	if (core->curr_slc_option & MFC_SLC_OPTION_INTERNAL) {
		/* Read, Write allocation: Internal */
		MFC_SLC_ALLOC_FULL(reg, 4);
		MFC_SLC_ALLOC_FULL(reg, 6);
		MFC_SLC_ALLOC_FULL(reg, 7);
		MFC_SLC_ALLOC_FULL(reg, 8);
		MFC_SLC_ALLOC_FULL(reg, 9);
		MFC_SLC_ALLOC_FULL(reg, 13);
		MFC_CORE_WRITEL(reg, mfc_reg_axi_rd_attr0_slc);
		MFC_CORE_WRITEL(reg, mfc_reg_axi_wr_attr0_slc);
	}

	if (core->curr_slc_option & MFC_SLC_OPTION_DPB_FULL_W) {
		if (core->curr_slc_option & MFC_SLC_OPTION_DPB_CHROMA_W) {
			reg = MFC_CORE_READL(mfc_reg_axi_wr_attr0_slc);
			/* Write allocation: DPB CHROMA */
			MFC_SLC_ALLOC_FULL(reg, 10);
			MFC_CORE_WRITEL(reg, mfc_reg_axi_wr_attr0_slc);
		}
		if (core->curr_slc_option & MFC_SLC_OPTION_DPB_LUMA_W) {
			reg = MFC_CORE_READL(mfc_reg_axi_wr_attr1_slc);
			/* Write allocation: DPB LUMA */
			MFC_SLC_ALLOC_FULL(reg, 3);
			MFC_CORE_WRITEL(reg, mfc_reg_axi_wr_attr1_slc);
		}
	} else if (core->curr_slc_option & MFC_SLC_OPTION_DPB_PARTIAL_W) {
		unsigned int left = 0;
		unsigned int right = ALIGN(ctx->img_width, 64);
		unsigned int lower = 0;
		unsigned int upper = ALIGN(ctx->img_height, 64);
		if (core->curr_slc_option & MFC_SLC_OPTION_DPB_CHROMA_W) {
			reg = MFC_CORE_READL(mfc_reg_axi_wr_attr0_slc);
			/* Write allocation: DPB CHROMA */
			MFC_SLC_ALLOC_PARTIAL(reg, 10);
			MFC_CORE_WRITEL(reg, mfc_reg_axi_wr_attr0_slc);
		}
		if (core->curr_slc_option & MFC_SLC_OPTION_DPB_LUMA_W) {
			reg = MFC_CORE_READL(mfc_reg_axi_wr_attr1_slc);
			/* Write allocation: DPB LUMA */
			MFC_SLC_ALLOC_PARTIAL(reg, 3);
			MFC_CORE_WRITEL(reg, mfc_reg_axi_wr_attr1_slc);
		}

		if (slc_partial_height_ratio > 0) {
			upper = upper / slc_partial_height_ratio;
			upper = ALIGN(upper, 64);
		} else {
			/*
			 * Default Partial ROI assumption:
			 * 1/2 of height for upper bound for 8 bit video
			 * 1/4 of height for upper bound for 10 bit video
			 */
			if (ctx->is_10bit)
				upper = upper / 4;
			else
				upper = upper / 2;

			upper = ALIGN(upper, 64);
		}

		mfc_debug(2, "Partial cache: left %d, right %d, lower %d, upper %d",
				left, right, lower, upper);
		reg = left;
		reg |= ((right) << 16);

		MFC_CORE_WRITEL(reg, mfc_reg_lf_pixel_posx_slc);

		reg = upper;
		reg |= ((lower) << 16);
		MFC_CORE_WRITEL(reg, mfc_reg_lf_pixel_posy_slc);
	}

	if (core->curr_slc_option & MFC_SLC_OPTION_REF_PXL_R) {
		reg = MFC_CORE_READL(mfc_reg_axi_rd_attr0_slc);
		/* Read allocation: Reference Pixel */
		MFC_SLC_ALLOC_FULL(reg, 0);
		MFC_SLC_ALLOC_FULL(reg, 1);
		MFC_SLC_ALLOC_FULL(reg, 2);
		MFC_SLC_ALLOC_FULL(reg, 3);
		MFC_CORE_WRITEL(reg, mfc_reg_axi_rd_attr0_slc);

		reg = MFC_CORE_READL(mfc_reg_axi_rd_attr1_slc);
		MFC_SLC_ALLOC_FULL(reg, 0);
		MFC_SLC_ALLOC_FULL(reg, 1);
		MFC_SLC_ALLOC_FULL(reg, 2);
		MFC_SLC_ALLOC_FULL(reg, 3);
		MFC_SLC_ALLOC_FULL(reg, 4);
		MFC_SLC_ALLOC_FULL(reg, 5);
		MFC_SLC_ALLOC_FULL(reg, 6);
		MFC_SLC_ALLOC_FULL(reg, 7);
		MFC_CORE_WRITEL(reg, mfc_reg_axi_rd_attr1_slc);
	}
}

int mfc_core_cmd_dec_init_buffers(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = core->dev;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	unsigned int reg = 0;
	int ret;

	mfc_core_set_pixel_format(core, ctx, ctx->dst_fmt->fourcc);

	mfc_clean_core_ctx_int_flags(core_ctx);
	ret = mfc_core_set_dec_codec_buffers(core_ctx);
	if (ret) {
		mfc_ctx_info("isn't enough codec buffer size, re-alloc!\n");

		if (core->has_llc && core->llc_on_status)
			mfc_llc_flush(core);

		if (core->has_slc && core->slc_on_status)
			mfc_slc_flush(core, ctx);

		mfc_release_codec_buffers(core_ctx);
		ret = mfc_alloc_codec_buffers(core_ctx);
		if (ret) {
			mfc_err("Failed to allocate decoding buffers\n");
			return ret;
		}
		ret = mfc_core_set_dec_codec_buffers(core_ctx);
		if (ret) {
			mfc_err("Failed to alloc frame mem\n");
			return ret;
		}
	}

	if (core->has_slc && core->slc_on_status) {
		mfc_slc_update_partition(core, ctx);
		__mfc_core_set_slc_option(core, ctx);
	}

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->metadata_interface) &&
			ctx->metadata_buffer_allocated)
		mfc_core_set_dec_metadata_buffer(core, ctx);


	if (IS_TWO_MODE2(ctx)) {
		reg |= ((ctx->subcore_inst_no & MFC_REG_RET_INSTANCE_ID_OF_MFC1_MASK) <<
			MFC_REG_RET_INSTANCE_ID_OF_MFC1_SHIFT);
		reg |= (core_ctx->inst_no & MFC_REG_RET_INSTANCE_ID_MASK);
		MFC_CORE_WRITEL(reg, MFC_REG_INSTANCE_ID);
	} else {
		MFC_CORE_WRITEL(core_ctx->inst_no, MFC_REG_INSTANCE_ID);
	}

	if (sfr_dump & MFC_DUMP_DEC_INIT_BUFS)
		call_dop(core, dump_regs, core);

	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_INIT_BUFFERS);

	return ret;
}

int mfc_core_cmd_enc_init_buffers(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int ret;

	/*
	 * Header was generated now starting processing
	 * First set the reference frame buffers
	 */
	if (!core_ctx->codec_buffer_allocated) {
		ret = mfc_alloc_codec_buffers(core_ctx);
		if (ret) {
			mfc_err("Failed to allocate encoding buffers\n");
			mfc_change_state(core_ctx, MFCINST_ERROR);
			return ret;
		}
	}

	mfc_clean_core_ctx_int_flags(core_ctx);
	ret = mfc_core_set_enc_codec_buffers(core_ctx);
	if (ret) {
		mfc_ctx_info("isn't enough codec buffer size, re-alloc!\n");

		if (core->has_llc && core->llc_on_status)
			mfc_llc_flush(core);

		if (core->has_slc && core->slc_on_status)
			mfc_slc_flush(core, ctx);

		mfc_release_codec_buffers(core_ctx);
		ret = mfc_alloc_codec_buffers(core_ctx);
		if (ret) {
			mfc_err("Failed to allocate encoding buffers\n");
			return ret;
		}
		ret = mfc_core_set_enc_codec_buffers(core_ctx);
		if (ret) {
			mfc_err("Failed to set enc codec buffers\n");
			return ret;
		}
	}

	if (core->has_slc && core->slc_on_status) {
		mfc_slc_update_partition(core, ctx);
		__mfc_core_set_slc_option(core, ctx);
	}

	MFC_CORE_WRITEL(core_ctx->inst_no, MFC_REG_INSTANCE_ID);

	if (sfr_dump & MFC_DUMP_ENC_INIT_BUFS)
		call_dop(core, dump_regs, core);

	mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_INIT_BUFFERS);

	return ret;
}

static int __mfc_set_scratch_dpb_buffer(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_raw_info *raw;
	int i, ret;

	if (core->has_llc && core->llc_on_status)
		mfc_llc_flush(core);

	if (core->has_slc && core->slc_on_status)
		mfc_slc_flush(core, ctx);

	ret = mfc_alloc_scratch_buffer(core_ctx);
	if (ret) {
		mfc_err("Failed to allocate scratch buffers\n");
		return ret;
	}

	raw = &ctx->raw_buf;
	/* set decoder DPB size, stride */
	MFC_CORE_WRITEL(dec->total_dpb_count, MFC_REG_D_NUM_DPB);
	for (i = 0; i < raw->num_planes; i++) {
		mfc_debug(2, "[FRAME] buf[%d] size: %d, stride: %d\n",
				i, raw->plane_size[i], raw->stride[i]);
		MFC_CORE_WRITEL(raw->plane_size[i], MFC_REG_D_FIRST_PLANE_DPB_SIZE + (i * 4));
		MFC_CORE_WRITEL(ctx->raw_buf.stride[i],
				MFC_REG_D_FIRST_PLANE_DPB_STRIDE_SIZE + (i * 4));
		if (IS_2BIT_NEED(ctx)) {
			MFC_CORE_WRITEL(raw->stride_2bits[i], MFC_REG_D_FIRST_PLANE_2BIT_DPB_STRIDE_SIZE + (i * 4));
			MFC_CORE_WRITEL(raw->plane_size_2bits[i], MFC_REG_D_FIRST_PLANE_2BIT_DPB_SIZE + (i * 4));
			mfc_debug(2, "[FRAME]%s%s 2bits buf[%d] size: %d, stride: %d\n",
					(ctx->is_10bit ? "[10BIT]" : ""),
					(ctx->is_sbwc ? "[SBWC]" : ""),
					i, raw->plane_size_2bits[i], raw->stride_2bits[i]);
		}
	}

	/* set scratch buffers */
	MFC_CORE_WRITEL(core_ctx->scratch_buf.daddr, MFC_REG_D_SCRATCH_BUFFER_ADDR);
	MFC_CORE_WRITEL(ctx->scratch_buf_size, MFC_REG_D_SCRATCH_BUFFER_SIZE);
	mfc_debug(2, "[FRAME] scratch buf addr: 0x%#llx size %ld\n",
			core_ctx->scratch_buf.daddr, ctx->scratch_buf_size);

	return 0;
}

/* Decode a single frame */
int mfc_core_cmd_dec_one_frame(struct mfc_core *core, struct mfc_ctx *ctx,
		int last_frame, int src_index)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_dec *dec = ctx->dec_priv;
	u32 reg = 0;
	int ret = 0;
	u32 timeout_value = MFC_TIMEOUT_VALUE;

	mfc_debug(2, "[MFC-%d][DPB] set dpb: %#lx, used: %#lx\n",
			core->id, core_ctx->dynamic_set, dec->dynamic_used);

	reg = MFC_CORE_READL(MFC_REG_D_NAL_START_OPTIONS);
	/* Black bar */
	reg &= ~(0x1 << MFC_REG_D_NAL_START_OPT_BLACK_BAR_SHIFT);
	reg |= ((dec->detect_black_bar & 0x1) << MFC_REG_D_NAL_START_OPT_BLACK_BAR_SHIFT);
	if (feature_option & MFC_OPTION_BLACK_BAR_ENABLE)
		reg |= (1 << MFC_REG_D_NAL_START_OPT_BLACK_BAR_SHIFT);
	/* Scratch buf & DPB changes when interframe resolution change */
	if (dec->inter_res_change) {
		ret = __mfc_set_scratch_dpb_buffer(core, ctx);
		if (ret)
			return ret;
		reg |= (0x1 << MFC_REG_D_NAL_START_OPT_NEW_SCRATCH_SHIFT);
		reg |= (0x1 << MFC_REG_D_NAL_START_OPT_NEW_DPB_SHIFT);
		dec->inter_res_change = 0;
	} else {
		reg &= ~(0x1 << MFC_REG_D_NAL_START_OPT_NEW_SCRATCH_SHIFT);
		reg &= ~(0x1 << MFC_REG_D_NAL_START_OPT_NEW_DPB_SHIFT);
	}
	/* Operation core mode */
	reg &= ~(0x1 << MFC_REG_D_NAL_START_OPT_TWO_MFC_ENABLE_SHIFT);
	if (IS_MULTI_MODE(ctx) || ctx->op_mode == MFC_OP_SWITCH_BUT_MODE2)
		reg |= (1 << MFC_REG_D_NAL_START_OPT_TWO_MFC_ENABLE_SHIFT);
	else
		reg |= (0 << MFC_REG_D_NAL_START_OPT_TWO_MFC_ENABLE_SHIFT);
	if (ctx->op_mode == MFC_OP_SWITCH_BUT_MODE2) {
		mfc_debug(2, "[2CORE] operate once op_mode %d\n", ctx->op_mode);
		ctx->op_mode = MFC_OP_SWITCH_TO_SINGLE;
	}
	/* For debugging */
	if (ctx->op_mode == MFC_OP_SWITCHING)
		mfc_err("[2CORE] It is a mode that can not operate\n");
	MFC_CORE_WRITEL(reg, MFC_REG_D_NAL_START_OPTIONS);
	mfc_debug(3, "NAL_START_OPTIONS: %#x, op_mode: %d\n", reg, ctx->op_mode);

	if (core->last_mfc_freq)
		timeout_value = (core->last_mfc_freq * MFC_TIMEOUT_VALUE_IN_MSEC);
	mfc_debug(2, "Last MFC Freq: %d, Timeout Value: %d\n", core->last_mfc_freq, timeout_value);

	MFC_CORE_WRITEL(mfc_get_lower(core_ctx->dynamic_set), MFC_REG_D_DYNAMIC_DPB_FLAG_LOWER);
	MFC_CORE_WRITEL(mfc_get_upper(core_ctx->dynamic_set), MFC_REG_D_DYNAMIC_DPB_FLAG_UPPER);
	MFC_CORE_WRITEL(mfc_get_lower(core_ctx->dynamic_set), MFC_REG_D_AVAILABLE_DPB_FLAG_LOWER);
	MFC_CORE_WRITEL(mfc_get_upper(core_ctx->dynamic_set), MFC_REG_D_AVAILABLE_DPB_FLAG_UPPER);

	MFC_CORE_WRITEL(dec->slice_enable, MFC_REG_D_SLICE_IF_ENABLE);
	MFC_CORE_WRITEL(timeout_value, MFC_REG_TIMEOUT_VALUE);
	MFC_CORE_WRITEL(core_ctx->inst_no, MFC_REG_INSTANCE_ID);

	if ((sfr_dump & MFC_DUMP_DEC_FIRST_NAL_START) && !core_ctx->check_dump) {
		call_dop(core, dump_regs, core);
		core_ctx->check_dump = 1;
	}
	if (sfr_dump & MFC_DUMP_DEC_NAL_START)
		call_dop(core, dump_regs, core);

	/* source index for 2core mode2 should set just before sent command */
	if (IS_TWO_MODE2(ctx) || IS_SWITCH_SINGLE_MODE(ctx)) {
		mutex_lock(&ctx->cpb_mutex);
		mfc_debug(2, "[MFC-%d][STREAM] set cpb: %d, curr index: %d\n",
				core->id, src_index, ctx->curr_src_index);
		ctx->curr_src_index = src_index;
	}

	/*
	 * Issue different commands to instance basing on whether it
	 * is the last frame or not.
	 */
	switch (last_frame) {
	case 0:
		mfc_perf_measure_on(core);

		mfc_perf_trace(ctx, "frame", 1);
		mfc_perf_trace(ctx, "fps", ctx->framerate / 1000);

		mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_NAL_START);
		break;
	case 1:
		mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_LAST_FRAME);
		break;
	}

	if (IS_TWO_MODE2(ctx) || IS_SWITCH_SINGLE_MODE(ctx))
		mutex_unlock(&ctx->cpb_mutex);

	mfc_debug(2, "Decoding a usual frame\n");
	return 0;
}

/* Encode a single frame */
void mfc_core_cmd_enc_one_frame(struct mfc_core *core, struct mfc_ctx *ctx,
		int last_frame)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	u32 timeout_value = MFC_TIMEOUT_VALUE;

	mfc_debug(2, "++\n");

	if (core->last_mfc_freq)
		timeout_value = (core->last_mfc_freq * MFC_TIMEOUT_VALUE_IN_MSEC);
	mfc_debug(2, "Last MFC Freq: %d, Timeout Value: %d\n", core->last_mfc_freq, timeout_value);

	MFC_CORE_WRITEL(timeout_value, MFC_REG_TIMEOUT_VALUE);
	MFC_CORE_WRITEL(core_ctx->inst_no, MFC_REG_INSTANCE_ID);

	if ((sfr_dump & MFC_DUMP_ENC_FIRST_NAL_START) && !core_ctx->check_dump) {
		call_dop(core, dump_regs, core);
		core_ctx->check_dump = 1;
	}
	if (sfr_dump & MFC_DUMP_ENC_NAL_START)
		call_dop(core, dump_regs, core);

	/*
	 * Issue different commands to instance basing on whether it
	 * is the last frame or not.
	 */
	switch (last_frame) {
	case 0:
		mfc_perf_measure_on(core);

		mfc_perf_trace(ctx, "frame", 1);
		mfc_perf_trace(ctx, "fps", ctx->framerate / 1000);

		mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_NAL_START);
		break;
	case 1:
		mfc_core_cmd_host2risc(core, MFC_REG_H2R_CMD_LAST_FRAME);
		break;
	}

	mfc_debug(2, "--\n");
}
