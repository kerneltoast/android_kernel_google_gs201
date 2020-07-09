/*
 * drivers/media/platform/exynos/mfc/mfc_buf.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/soc/samsung/exynos-smc.h>
#include <linux/firmware.h>
#include <trace/events/mfc.h>
#include <linux/iommu.h>
#include <linux/dma-iommu.h>

#include "mfc_buf.h"

#include "mfc_mem.h"

static void __mfc_alloc_common_context(struct mfc_dev *dev,
					enum mfc_buf_usage_type buf_type)
{
	struct mfc_special_buf *ctx_buf;
	int firmware_size;
	unsigned long fw_daddr;

	mfc_dev_debug_enter();

	ctx_buf = &dev->common_ctx_buf;
	fw_daddr = dev->fw_buf.daddr;

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (buf_type == MFCBUF_DRM) {
		ctx_buf = &dev->drm_common_ctx_buf;
		fw_daddr = dev->drm_fw_buf.daddr;
	}
#endif

	firmware_size = dev->variant->buf_size->firmware_code;

	ctx_buf->dma_buf = NULL;
	ctx_buf->vaddr = NULL;
	ctx_buf->daddr = fw_daddr + firmware_size;

	mfc_dev_debug_leave();
}

/* Wrapper : allocate context buffers for SYS_INIT */
void mfc_alloc_common_context(struct mfc_dev *dev)
{
	__mfc_alloc_common_context(dev, MFCBUF_NORMAL);

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (dev->fw.drm_status)
		__mfc_alloc_common_context(dev, MFCBUF_DRM);
#endif
}

/* Release context buffers for SYS_INIT */
static void __mfc_release_common_context(struct mfc_dev *dev,
					enum mfc_buf_usage_type buf_type)
{
	struct mfc_special_buf *ctx_buf;

	ctx_buf = &dev->common_ctx_buf;
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (buf_type == MFCBUF_DRM)
		ctx_buf = &dev->drm_common_ctx_buf;
#endif

	ctx_buf->dma_buf = NULL;
	ctx_buf->vaddr = NULL;
	ctx_buf->daddr = 0;
}

/* Release context buffers for SYS_INIT */
void mfc_release_common_context(struct mfc_dev *dev)
{
	__mfc_release_common_context(dev, MFCBUF_NORMAL);

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	__mfc_release_common_context(dev, MFCBUF_DRM);
#endif
}

/* Allocate memory for instance data buffer */
int mfc_alloc_instance_context(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_ctx_buf_size *buf_size;

	mfc_debug_enter();

	buf_size = dev->variant->buf_size->ctx_buf;

	switch (ctx->codec_mode) {
	case MFC_REG_CODEC_H264_DEC:
	case MFC_REG_CODEC_H264_MVC_DEC:
	case MFC_REG_CODEC_HEVC_DEC:
	case MFC_REG_CODEC_BPG_DEC:
		ctx->instance_ctx_buf.size = buf_size->h264_dec_ctx;
		break;
	case MFC_REG_CODEC_MPEG4_DEC:
	case MFC_REG_CODEC_H263_DEC:
	case MFC_REG_CODEC_VC1_RCV_DEC:
	case MFC_REG_CODEC_VC1_DEC:
	case MFC_REG_CODEC_MPEG2_DEC:
	case MFC_REG_CODEC_VP8_DEC:
	case MFC_REG_CODEC_VP9_DEC:
	case MFC_REG_CODEC_FIMV1_DEC:
	case MFC_REG_CODEC_FIMV2_DEC:
	case MFC_REG_CODEC_FIMV3_DEC:
	case MFC_REG_CODEC_FIMV4_DEC:
		ctx->instance_ctx_buf.size = buf_size->other_dec_ctx;
		break;
	case MFC_REG_CODEC_H264_ENC:
		ctx->instance_ctx_buf.size = buf_size->h264_enc_ctx;
		break;
	case MFC_REG_CODEC_HEVC_ENC:
	case MFC_REG_CODEC_BPG_ENC:
		ctx->instance_ctx_buf.size = buf_size->hevc_enc_ctx;
		break;
	case MFC_REG_CODEC_MPEG4_ENC:
	case MFC_REG_CODEC_H263_ENC:
	case MFC_REG_CODEC_VP8_ENC:
	case MFC_REG_CODEC_VP9_ENC:
		ctx->instance_ctx_buf.size = buf_size->other_enc_ctx;
		break;
	default:
		ctx->instance_ctx_buf.size = 0;
		mfc_ctx_err("Codec type(%d) should be checked!\n",
				ctx->codec_mode);
		return -ENOMEM;
	}

	if (ctx->is_drm)
		ctx->instance_ctx_buf.buftype = MFCBUF_DRM;
	else
		ctx->instance_ctx_buf.buftype = MFCBUF_NORMAL;

	if (mfc_mem_ion_alloc(dev, &ctx->instance_ctx_buf)) {
		mfc_ctx_err("Allocating context buffer failed\n");
		return -ENOMEM;
	}

	mfc_debug(2, "[MEMINFO] Instance buf ctx[%d] size: %ld, daddr: 0x%08llx\n",
			ctx->num, ctx->instance_ctx_buf.size, ctx->instance_ctx_buf.daddr);
	mfc_debug_leave();

	return 0;
}

/* Release instance buffer */
void mfc_release_instance_context(struct mfc_ctx *ctx)
{
	mfc_debug_enter();

	mfc_mem_ion_free(&ctx->instance_ctx_buf);
	mfc_debug(2, "[MEMINFO] Release the instance buffer ctx[%d]\n", ctx->num);

	mfc_debug_leave();
}

static void __mfc_dec_calc_codec_buffer_size(struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;

	/* Codecs have different memory requirements */
	switch (ctx->codec_mode) {
	case MFC_REG_CODEC_H264_DEC:
	case MFC_REG_CODEC_H264_MVC_DEC:
		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size =
			ctx->scratch_buf_size +
			(dec->mv_count * ctx->mv_size);
		break;
	case MFC_REG_CODEC_MPEG4_DEC:
	case MFC_REG_CODEC_FIMV1_DEC:
	case MFC_REG_CODEC_FIMV2_DEC:
	case MFC_REG_CODEC_FIMV3_DEC:
	case MFC_REG_CODEC_FIMV4_DEC:
		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		if (dec->loop_filter_mpeg4) {
			ctx->loopfilter_luma_size = ALIGN(ctx->raw_buf.plane_size[0], 256);
			ctx->loopfilter_chroma_size = ALIGN(ctx->raw_buf.plane_size[1] +
							ctx->raw_buf.plane_size[2], 256);
			ctx->codec_buf.size = ctx->scratch_buf_size +
				(NUM_MPEG4_LF_BUF * (ctx->loopfilter_luma_size +
						     ctx->loopfilter_chroma_size));
		} else {
			ctx->codec_buf.size = ctx->scratch_buf_size;
		}
		break;
	case MFC_REG_CODEC_VC1_RCV_DEC:
	case MFC_REG_CODEC_VC1_DEC:
		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size = ctx->scratch_buf_size;
		break;
	case MFC_REG_CODEC_MPEG2_DEC:
		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size = ctx->scratch_buf_size;
		break;
	case MFC_REG_CODEC_H263_DEC:
		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size = ctx->scratch_buf_size;
		break;
	case MFC_REG_CODEC_VP8_DEC:
		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size = ctx->scratch_buf_size;
		break;
	case MFC_REG_CODEC_VP9_DEC:
		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size =
			ctx->scratch_buf_size +
			DEC_STATIC_BUFFER_SIZE;
		break;
	case MFC_REG_CODEC_HEVC_DEC:
	case MFC_REG_CODEC_BPG_DEC:
		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size =
			ctx->scratch_buf_size +
			(dec->mv_count * ctx->mv_size);
		break;
	default:
		ctx->codec_buf.size = 0;
		mfc_ctx_err("invalid codec type: %d\n", ctx->codec_mode);
		break;
	}

	mfc_debug(2, "[MEMINFO] scratch: %zu, MV: %zu x count %d\n",
			ctx->scratch_buf_size, ctx->mv_size, dec->mv_count);
	if (dec->loop_filter_mpeg4)
		mfc_debug(2, "[MEMINFO] (loopfilter luma: %zu, chroma: %zu) x count %d\n",
				ctx->loopfilter_luma_size, ctx->loopfilter_chroma_size,
				NUM_MPEG4_LF_BUF);
}

static void __mfc_enc_calc_codec_buffer_size(struct mfc_ctx *ctx)
{
	struct mfc_enc *enc;
	unsigned int mb_width, mb_height;
	unsigned int lcu_width = 0, lcu_height = 0;
	int width = ctx->crop_width, height = ctx->crop_height;

	enc = ctx->enc_priv;
	enc->tmv_buffer_size = 0;

	mb_width = WIDTH_MB(ctx->crop_width);
	mb_height = HEIGHT_MB(ctx->crop_height);

	lcu_width = ENC_LCU_WIDTH(ctx->crop_width);
	lcu_height = ENC_LCU_HEIGHT(ctx->crop_height);

	if (IS_COMPRESS_DPB(ctx) && ctx->is_10bit) {
		enc->luma_dpb_size =
			ENC_SBWC_LUMA_10B_DPB_SIZE(width, height);
		enc->chroma_dpb_size =
			ENC_SBWC_CHROMA_10B_DPB_SIZE(width, height);
	} else if (IS_COMPRESS_DPB(ctx) && !ctx->is_10bit) {
		enc->luma_dpb_size =
			ENC_SBWC_LUMA_8B_DPB_SIZE(width, height);
		enc->chroma_dpb_size =
			ENC_SBWC_CHROMA_8B_DPB_SIZE(width, height);
	} else {
		/*
		 * default recon buffer size,
		 * it can be changed in case of 422, 10bit
		 */
		enc->luma_dpb_size =
			ALIGN(ENC_LUMA_DPB_SIZE(width, height), 64);
		enc->chroma_dpb_size =
			ALIGN(ENC_CHROMA_DPB_SIZE(width, height), 64);
	}

	/* Codecs have different memory requirements */
	switch (ctx->codec_mode) {
	case MFC_REG_CODEC_H264_ENC:
		enc->me_buffer_size =
			ALIGN(ENC_V100_H264_ME_SIZE(mb_width, mb_height), 256);

		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size =
			ctx->scratch_buf_size + enc->tmv_buffer_size +
			(ctx->dpb_count * (enc->luma_dpb_size +
			enc->chroma_dpb_size + enc->me_buffer_size));
		break;
	case MFC_REG_CODEC_MPEG4_ENC:
	case MFC_REG_CODEC_H263_ENC:
		enc->me_buffer_size =
			ALIGN(ENC_V100_MPEG4_ME_SIZE(mb_width, mb_height), 256);

		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size =
			ctx->scratch_buf_size + enc->tmv_buffer_size +
			(ctx->dpb_count * (enc->luma_dpb_size +
			enc->chroma_dpb_size + enc->me_buffer_size));
		break;
	case MFC_REG_CODEC_VP8_ENC:
		enc->me_buffer_size =
			ALIGN(ENC_V100_VP8_ME_SIZE(mb_width, mb_height), 256);

		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size =
			ctx->scratch_buf_size + enc->tmv_buffer_size +
			(ctx->dpb_count * (enc->luma_dpb_size +
			enc->chroma_dpb_size + enc->me_buffer_size));
		break;
	case MFC_REG_CODEC_VP9_ENC:
		if (!IS_COMPRESS_DPB(ctx) && (ctx->is_10bit || ctx->is_422)) {
			enc->luma_dpb_size =
				ALIGN(ENC_VP9_LUMA_DPB_10B_SIZE(ctx->crop_width, ctx->crop_height), 64);
			enc->chroma_dpb_size =
				ALIGN(ENC_VP9_CHROMA_DPB_10B_SIZE(ctx->crop_width, ctx->crop_height), 64);
			mfc_debug(2, "[10BIT] VP9 10bit or 422 recon luma size: %zu chroma size: %zu\n",
					enc->luma_dpb_size, enc->chroma_dpb_size);
		}
		enc->me_buffer_size =
			ALIGN(ENC_V100_VP9_ME_SIZE(lcu_width, lcu_height), 256);

		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size =
			ctx->scratch_buf_size + enc->tmv_buffer_size +
			(ctx->dpb_count * (enc->luma_dpb_size +
					   enc->chroma_dpb_size + enc->me_buffer_size));
		break;
	case MFC_REG_CODEC_HEVC_ENC:
	case MFC_REG_CODEC_BPG_ENC:
		if (!IS_COMPRESS_DPB(ctx) && (ctx->is_10bit || ctx->is_422)) {
			enc->luma_dpb_size =
				ALIGN(ENC_HEVC_LUMA_DPB_10B_SIZE(ctx->crop_width, ctx->crop_height), 64);
			enc->chroma_dpb_size =
				ALIGN(ENC_HEVC_CHROMA_DPB_10B_SIZE(ctx->crop_width, ctx->crop_height), 64);
			mfc_debug(2, "[10BIT] HEVC 10bit or 422 recon luma size: %zu chroma size: %zu\n",
					enc->luma_dpb_size, enc->chroma_dpb_size);
		}
		enc->me_buffer_size =
			ALIGN(ENC_V100_HEVC_ME_SIZE(lcu_width, lcu_height), 256);

		ctx->scratch_buf_size = ALIGN(ctx->scratch_buf_size, 256);
		ctx->codec_buf.size =
			ctx->scratch_buf_size + enc->tmv_buffer_size +
			(ctx->dpb_count * (enc->luma_dpb_size +
					   enc->chroma_dpb_size + enc->me_buffer_size));
		break;
	default:
		ctx->codec_buf.size = 0;
		mfc_ctx_err("invalid codec type: %d\n", ctx->codec_mode);
		break;
	}

	mfc_debug(2, "[MEMINFO] scratch: %zu, TMV: %zu, (recon luma: %zu, chroma: %zu, me: %zu) x count %d\n",
			ctx->scratch_buf_size, enc->tmv_buffer_size,
			enc->luma_dpb_size, enc->chroma_dpb_size, enc->me_buffer_size,
			ctx->dpb_count);
}

/* Allocate codec buffers */
int mfc_alloc_codec_buffers(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;

	mfc_debug_enter();

	if (ctx->type == MFCINST_DECODER) {
		__mfc_dec_calc_codec_buffer_size(ctx);
	} else if (ctx->type == MFCINST_ENCODER) {
		__mfc_enc_calc_codec_buffer_size(ctx);
	} else {
		mfc_ctx_err("invalid type: %d\n", ctx->type);
		return -EINVAL;
	}

	if (ctx->is_drm)
		ctx->codec_buf.buftype = MFCBUF_DRM;
	else
		ctx->codec_buf.buftype = MFCBUF_NORMAL;

	if (ctx->codec_buf.size > 0) {
		if (mfc_mem_ion_alloc(dev, &ctx->codec_buf)) {
			mfc_ctx_err("Allocating codec buffer failed\n");
			return -ENOMEM;
		}
		ctx->codec_buffer_allocated = 1;
	} else if (ctx->codec_mode == MFC_REG_CODEC_MPEG2_DEC) {
		ctx->codec_buffer_allocated = 1;
	}

	mfc_debug(2, "[MEMINFO] Codec buf ctx[%d] size: %ld, addr: 0x%08llx\n",
			ctx->num, ctx->codec_buf.size, ctx->codec_buf.daddr);
	mfc_debug_leave();

	return 0;
}

/* Release buffers allocated for codec */
void mfc_release_codec_buffers(struct mfc_ctx *ctx)
{
	mfc_mem_ion_free(&ctx->codec_buf);
	ctx->codec_buffer_allocated = 0;
	mfc_release_scratch_buffer(ctx);
	mfc_debug(2, "[MEMINFO] Release the codec buffer ctx[%d]\n", ctx->num);
}

int mfc_alloc_scratch_buffer(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;

	mfc_debug_enter();

	if (ctx->scratch_buffer_allocated) {
		mfc_mem_ion_free(&ctx->scratch_buf);
		ctx->scratch_buffer_allocated = 0;
		mfc_debug(2, "[MEMINFO] Release the scratch buffer ctx[%d]\n",
							ctx->num);
	}

	if (ctx->is_drm)
		ctx->scratch_buf.buftype = MFCBUF_DRM;
	else
		ctx->scratch_buf.buftype = MFCBUF_NORMAL;

	ctx->scratch_buf.size =  ALIGN(ctx->scratch_buf_size, 256);
	if (ctx->scratch_buf.size > 0) {
		if (mfc_mem_ion_alloc(dev, &ctx->scratch_buf)) {
			mfc_ctx_err("Allocating scratch_buf buffer failed\n");
			return -ENOMEM;
		}
		ctx->scratch_buffer_allocated = 1;
	}

	mfc_debug(2, "[MEMINFO] scratch buf ctx[%d] size: %ld, addr: 0x%08llx\n",
			ctx->num, ctx->scratch_buf_size,
			ctx->scratch_buf.daddr);

	mfc_debug_leave();
	return 0;
}

void mfc_release_scratch_buffer(struct mfc_ctx *ctx)
{
	mfc_debug_enter();
	if (ctx->scratch_buffer_allocated) {
		mfc_mem_ion_free(&ctx->scratch_buf);
		ctx->scratch_buffer_allocated = 0;
		mfc_debug(2, "[MEMINFO] Release the scratch buffer ctx[%d]\n",
							ctx->num);
	}
	mfc_debug_leave();
}

/* Allocation buffer of debug infor memory for FW debugging */
int mfc_alloc_dbg_info_buffer(struct mfc_dev *dev)
{
	struct mfc_ctx_buf_size *buf_size = dev->variant->buf_size->ctx_buf;

	mfc_dev_debug(2, "Allocate a debug-info buffer\n");

	dev->dbg_info_buf.buftype = MFCBUF_NORMAL;
	dev->dbg_info_buf.size = buf_size->dbg_info_buf;
	if (mfc_mem_ion_alloc(dev, &dev->dbg_info_buf)) {
		mfc_dev_err("Allocating debug info buffer failed\n");
		return -ENOMEM;
	}
	mfc_dev_debug(2, "[MEMINFO] debug info buf size: %ld, daddr: 0x%08llx, vaddr: 0x%p\n",
			dev->dbg_info_buf.size, dev->dbg_info_buf.daddr, dev->dbg_info_buf.vaddr);

	return 0;
}

/* Release buffer of debug infor memory for FW debugging */
void mfc_release_dbg_info_buffer(struct mfc_dev *dev)
{
	if (!dev->dbg_info_buf.dma_buf)
		mfc_dev_debug(2, "debug info buffer is already freed\n");

	mfc_mem_ion_free(&dev->dbg_info_buf);
	mfc_dev_debug(2, "[MEMINFO] Release the debug info buffer\n");
}

/* Allocation buffer of ROI macroblock information */
static int __mfc_alloc_enc_roi_buffer(struct mfc_ctx *ctx, size_t size,
				struct mfc_special_buf *roi_buf)
{
	struct mfc_dev *dev = ctx->dev;

	roi_buf->size = size;
	roi_buf->buftype = MFCBUF_NORMAL;

	if (roi_buf->dma_buf == NULL) {
		if (mfc_mem_ion_alloc(dev, roi_buf)) {
			mfc_ctx_err("[ROI] Allocating ROI buffer failed\n");
			return -ENOMEM;
		}
	}
	mfc_debug(2, "[MEMINFO][ROI] roi buf ctx[%d] size: %ld, daddr: 0x%08llx, vaddr: 0x%p\n",
			ctx->num, roi_buf->size,
			roi_buf->daddr, roi_buf->vaddr);

	memset(roi_buf->vaddr, 0, roi_buf->size);

	return 0;
}

/* Wrapper : allocation ROI buffers */
int mfc_alloc_enc_roi_buffer(struct mfc_ctx *ctx)
{
	struct mfc_enc *enc = ctx->enc_priv;
	unsigned int mb_width, mb_height;
	unsigned int lcu_width = 0, lcu_height = 0;
	size_t size;
	int i, ret;

	mb_width = WIDTH_MB(ctx->crop_width);
	mb_height = HEIGHT_MB(ctx->crop_height);

	switch (ctx->codec_mode) {
	case MFC_REG_CODEC_H264_ENC:
		size = (((mb_width * (mb_height + 1) / 2) + 15) / 16) * 16 * 2;
		break;
	case MFC_REG_CODEC_MPEG4_ENC:
	case MFC_REG_CODEC_VP8_ENC:
		size = mb_width * mb_height;
		break;
	case MFC_REG_CODEC_VP9_ENC:
		lcu_width = (ctx->crop_width + 63) / 64;
		lcu_height = (ctx->crop_height + 63) / 64;
		size = lcu_width * lcu_height * 4;
		break;
	case MFC_REG_CODEC_HEVC_ENC:
		lcu_width = (ctx->crop_width + 31) / 32;
		lcu_height = (ctx->crop_height + 31) / 32;
		size = lcu_width * lcu_height;
		break;
	default:
		mfc_debug(2, "ROI not supported codec type(%d). Allocate with default size\n",
				ctx->codec_mode);
		size = mb_width * mb_height;
		break;
	}

	for (i = 0; i < MFC_MAX_EXTRA_BUF; i++) {
		ret = __mfc_alloc_enc_roi_buffer(ctx, size, &enc->roi_buf[i]);
		if (ret < 0) {
			mfc_ctx_err("[ROI] Allocating remapping buffer[%d] failed\n",
					i);
			return ret;
		}
	}

	return 0;
}

/* Release buffer of ROI macroblock information */
void mfc_release_enc_roi_buffer(struct mfc_ctx *ctx)
{
	struct mfc_enc *enc = ctx->enc_priv;
	int i;

	for (i = 0; i < MFC_MAX_EXTRA_BUF; i++)
		if (enc->roi_buf[i].dma_buf)
			mfc_mem_ion_free(&enc->roi_buf[i]);

	mfc_debug(2, "[MEMINFO][ROI] Release the ROI buffer\n");
}

int mfc_otf_alloc_stream_buf(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_debug *debug = &handle->otf_debug;
	struct mfc_special_buf *buf;
	struct mfc_raw_info *raw = &ctx->raw_buf;
	int i;

	mfc_debug_enter();

	for (i = 0; i < OTF_MAX_BUF; i++) {
		buf = &debug->stream_buf[i];
		buf->buftype = MFCBUF_NORMAL;
		buf->size = raw->total_plane_size;
		if (mfc_mem_ion_alloc(dev, buf)) {
			mfc_ctx_err("[OTF] Allocating stream buffer failed\n");
			return -EINVAL;
		}
		mfc_debug(2, "[OTF][MEMINFO] OTF stream buf[%d] size: %ld, daddr: 0x%08llx, vaddr: 0x%p\n",
				i, buf->size, buf->daddr, buf->vaddr);
		memset(buf->vaddr, 0, raw->total_plane_size);
	}

	mfc_debug_leave();

	return 0;
}

void mfc_otf_release_stream_buf(struct mfc_ctx *ctx)
{
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_debug *debug = &handle->otf_debug;
	struct mfc_special_buf *buf;
	int i;

	mfc_debug_enter();

	for (i = 0; i < OTF_MAX_BUF; i++) {
		buf = &debug->stream_buf[i];
		if (buf->dma_buf)
			mfc_mem_ion_free(buf);
	}

	mfc_debug(2, "[OTF][MEMINFO] Release the OTF stream buffer\n");
	mfc_debug_leave();
}

static int __mfc_remap_firmware(struct mfc_dev *dev, struct mfc_special_buf *fw_buf)
{
	dma_addr_t fw_base_addr;
	int ret;

	fw_base_addr = MFC_BASE_ADDR + dev->fw_base_offset;

	fw_buf->domain = iommu_get_domain_for_dev(dev->device);
	fw_buf->map_size = iommu_map_sg(fw_buf->domain, fw_base_addr,
			fw_buf->sgt->sgl,
			fw_buf->sgt->nents,
			IOMMU_READ|IOMMU_WRITE);
	if (!fw_buf->map_size) {
		mfc_dev_err("Failed to remap iova (err %#llx)\n",
				fw_buf->daddr);
		return -ENOMEM;
	}

	fw_buf->daddr = fw_base_addr;
	dev->fw_base_offset += fw_buf->map_size;

	if (fw_base_addr == MFC_BASE_ADDR) {
		ret = iommu_dma_reserve_iova(dev->device, 0x0, MFC_BASE_ADDR);
		if (ret) {
			mfc_dev_err("failed to reserve dva for firmware %d\n", ret);
			return -ENOMEM;
		}
	}

	ret = iommu_dma_reserve_iova(dev->device, fw_buf->daddr,
					fw_buf->map_size);
	if (ret) {
		mfc_dev_err("failed to reserve dva for firmware %d\n", ret);
		return -ENOMEM;
	}

	return 0;
}

/* Allocate firmware */
int mfc_alloc_firmware(struct mfc_dev *dev)
{
	size_t firmware_size;
	struct mfc_ctx_buf_size *buf_size;
	struct mfc_special_buf *fw_buf;

	mfc_dev_debug_enter();

	buf_size = dev->variant->buf_size->ctx_buf;
	firmware_size = dev->variant->buf_size->firmware_code;
	dev->fw.size = firmware_size + buf_size->dev_ctx;

	if (dev->fw_buf.dma_buf)
		return 0;

	mfc_dev_debug(4, "[F/W] Allocating memory for firmware\n");
	trace_mfc_loadfw_start(dev->fw.size, firmware_size);

	dev->fw_buf.buftype = MFCBUF_NORMAL;
	dev->fw_buf.size = dev->fw.size;
	if (mfc_mem_ion_alloc(dev, &dev->fw_buf)) {
		mfc_dev_err("[F/W] Allocating normal firmware buffer failed\n");
		return -ENOMEM;
	}

	fw_buf = &dev->fw_buf;
	if (__mfc_remap_firmware(dev, fw_buf))
		goto err_reserve_iova;

	mfc_dev_info("[MEMINFO][F/W] FW normal: 0x%08llx (vaddr: 0x%p), size: %08zu\n",
			dev->fw_buf.daddr, dev->fw_buf.vaddr,
			dev->fw_buf.size);

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	dev->drm_fw_buf.buftype = MFCBUF_DRM_FW;
	dev->drm_fw_buf.size = dev->fw.size;
	if (mfc_mem_ion_alloc(dev, &dev->drm_fw_buf)) {
		mfc_dev_err("[F/W] Allocating DRM firmware buffer failed\n");
		goto err_daddr;
	}

	mfc_dev_info("[MEMINFO][F/W] FW DRM: 0x%08llx (vaddr: 0x%p), size: %08zu\n",
			dev->drm_fw_buf.daddr, dev->drm_fw_buf.vaddr,
			dev->drm_fw_buf.size);
#endif

	mfc_dev_debug_leave();

	return 0;

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
err_daddr:
#endif
err_reserve_iova:
	iommu_unmap(fw_buf->domain, MFC_BASE_ADDR, fw_buf->map_size);
	return -ENOMEM;
}

/* Load firmware to MFC */
int mfc_load_firmware(struct mfc_dev *dev)
{
	struct firmware *fw_blob;
	size_t firmware_size;
	int err;

	firmware_size = dev->variant->buf_size->firmware_code;

	/* Firmare has to be present as a separate file or compiled
	 * into kernel. */
	mfc_dev_debug_enter();
	mfc_dev_debug(4, "[F/W] Requesting F/W\n");
	err = request_firmware((const struct firmware **)&fw_blob,
					MFC_FW_NAME, dev->v4l2_dev.dev);

	if (err != 0) {
		mfc_dev_err("[F/W] Couldn't find the F/W invalid path\n");
		release_firmware(fw_blob);
		return -EINVAL;
	}

	mfc_dev_debug(2, "[MEMINFO][F/W] loaded F/W Size: %zu\n",
						fw_blob->size);

	if (fw_blob->size > firmware_size) {
		mfc_dev_err("[MEMINFO][F/W] MFC firmware(%zu) is too big to be loaded in memory(%zu)\n",
				fw_blob->size, firmware_size);
		release_firmware(fw_blob);
		return -ENOMEM;
	}

	if (dev->fw_buf.dma_buf == NULL || dev->fw_buf.daddr == 0) {
		mfc_dev_err("[F/W] MFC firmware is not allocated or was not mapped correctly\n");
		release_firmware(fw_blob);
		return -EINVAL;
	}

	/*  This adds to clear with '0' for firmware memory except code region. */
	mfc_dev_debug(4, "[F/W] memset before memcpy for normal fw\n");
	memset((dev->fw_buf.vaddr + fw_blob->size), 0, (firmware_size - fw_blob->size));
	memcpy(dev->fw_buf.vaddr, fw_blob->data, fw_blob->size);
	if (dev->drm_fw_buf.vaddr) {
		mfc_dev_debug(4, "[F/W] memset before memcpy for secure fw\n");
		memset((dev->drm_fw_buf.vaddr + fw_blob->size), 0, (firmware_size - fw_blob->size));
		memcpy(dev->drm_fw_buf.vaddr, fw_blob->data, fw_blob->size);
		mfc_dev_debug(4, "[F/W] copy firmware to secure region\n");
	}
	release_firmware(fw_blob);
	trace_mfc_loadfw_end(dev->fw.size, firmware_size);
	mfc_dev_debug_leave();
	return 0;
}

/* Release firmware memory */
int mfc_release_firmware(struct mfc_dev *dev)
{
	struct mfc_special_buf *fw_buf;

	/* Before calling this function one has to make sure
	 * that MFC is no longer processing */
	fw_buf = &dev->fw_buf;
	if (!fw_buf->dma_buf) {
		mfc_dev_err("[F/W] firmware memory is already freed\n");
		return -EINVAL;
	}
	iommu_unmap(fw_buf->domain, fw_buf->daddr, fw_buf->map_size);

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	mfc_mem_ion_free(&dev->drm_fw_buf);
#endif

	mfc_mem_ion_free(&dev->fw_buf);

	return 0;
}
