/*
 * drivers/media/platform/exynos/mfc/mfc_core_otf.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>

#include "mfc_rm.h"

#include "mfc_core_otf.h"
#include "mfc_hwfc_internal.h"

#include "mfc_core_reg_api.h"
#include "mfc_core_cmd.h"
#include "mfc_sync.h"
#include "mfc_llc.h"

#include "mfc_utils.h"
#include "mfc_buf.h"
#include "mfc_qos.h"

static struct mfc_fmt *__mfc_core_otf_find_hwfc_format(struct mfc_dev *dev, unsigned int pixelformat)
{
	unsigned long i;

	mfc_dev_debug_enter();

	for (i = 0; i < NUM_FORMATS; i++) {
		if (enc_hwfc_formats[i].fourcc == pixelformat)
			return (struct mfc_fmt *)&enc_hwfc_formats[i];
	}

	mfc_dev_debug_leave();

	return NULL;
}

static int __mfc_core_otf_set_buf_info(struct mfc_ctx *ctx)
{
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_buf_info *buf_info = &handle->otf_buf_info;
	struct mfc_dev *dev = ctx->dev;

	mfc_debug_enter();

	ctx->src_fmt = __mfc_core_otf_find_hwfc_format(dev, buf_info->pixel_format);
	if (!ctx->src_fmt) {
		mfc_ctx_err("[OTF] failed to set source format\n");
		return -EINVAL;
	}

	mfc_debug(2, "[OTF][FRAME] resolution w: %d, h: %d, format: %s, bufcnt: %d\n",
			buf_info->width, buf_info->height,
			ctx->src_fmt->name, buf_info->buffer_count);

	/* set source information */
	ctx->raw_buf.num_planes = ctx->src_fmt->num_planes;
	ctx->img_width = buf_info->width;
	ctx->img_height = buf_info->height;
	ctx->crop_width = buf_info->width;
	ctx->crop_height = buf_info->height;
	if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_NV12N) {
		ctx->bytesperline[0] = NV12N_STRIDE(ctx->img_width);
		ctx->bytesperline[1] = NV12N_STRIDE(ctx->img_width);
	} else {
		mfc_ctx_err("[OTF] not supported format(0x%x)\n", ctx->src_fmt->fourcc);
		return -EINVAL;
	}

	/* calculate source size */
	mfc_enc_calc_src_size(ctx);

	mfc_debug_leave();

	return 0;
}

static int __mfc_core_otf_map_buf(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_buf_addr *buf_addr = &handle->otf_buf_addr;
	struct _otf_buf_info *buf_info = &handle->otf_buf_info;
	int i;

	mfc_debug_enter();

	mfc_debug(2, "[OTF] buffer count: %d\n", buf_info->buffer_count);
	/* map buffers */
	for (i = 0; i < buf_info->buffer_count; i++) {
		mfc_debug(2, "[OTF] dma_buf: 0x%p\n", buf_info->bufs[i]);
		buf_addr->otf_buf_attach[i] = dma_buf_attach(buf_info->bufs[i],
				dev->device);
		if (IS_ERR(buf_addr->otf_buf_attach[i])) {
			mfc_ctx_err("[OTF] Failed to get attachment (err %ld)",
				PTR_ERR(buf_addr->otf_buf_attach[i]));
			buf_addr->otf_buf_attach[i] = 0;
			return -EINVAL;
		}
		buf_addr->sgt[i] = dma_buf_map_attachment(buf_addr->otf_buf_attach[i],
				DMA_BIDIRECTIONAL);
		if (IS_ERR(buf_addr->sgt[i])) {
			mfc_ctx_err("[OTF] Failed to map attach (err %ld)", PTR_ERR(buf_addr->sgt[i]));
			return -EINVAL;
		}

		buf_addr->otf_daddr[i][0] = sg_dma_address(buf_addr->sgt[i]->sgl);
		if (IS_ERR_VALUE(buf_addr->otf_daddr[i][0])) {
			mfc_ctx_err("[OTF] Failed to get daddr (0x%08llx)",
					buf_addr->otf_daddr[i][0]);
			buf_addr->otf_daddr[i][0] = 0;
			return -EINVAL;
		}
		if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_NV12N) {
			buf_addr->otf_daddr[i][1] = NV12N_CBCR_BASE(buf_addr->otf_daddr[i][0],
					ctx->img_width, ctx->img_height);
		} else {
			mfc_ctx_err("[OTF] not supported format(0x%x)\n", ctx->src_fmt->fourcc);
			return -EINVAL;
		}
		mfc_debug(2, "[OTF] index: %d, addr[0]: 0x%08llx, addr[1]: 0x%08llx\n",
				i, buf_addr->otf_daddr[i][0], buf_addr->otf_daddr[i][1]);
	}

	mfc_debug_leave();

	return 0;
}

static void __mfc_core_otf_unmap_buf(struct mfc_ctx *ctx)
{
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_buf_addr *buf_addr = &handle->otf_buf_addr;
	struct _otf_buf_info *buf_info = &handle->otf_buf_info;
	int i;

	mfc_debug_enter();

	for (i = 0; i < buf_info->buffer_count; i++) {
		if (buf_addr->sgt[i]) {
			dma_buf_unmap_attachment(buf_addr->otf_buf_attach[i],
					buf_addr->sgt[i], DMA_BIDIRECTIONAL);
			buf_addr->otf_daddr[i][0] = 0;
		}
		if (buf_addr->otf_buf_attach[i]) {
			dma_buf_detach(buf_info->bufs[i], buf_addr->otf_buf_attach[i]);
			buf_addr->otf_buf_attach[i] = 0;
		}
	}

	mfc_debug_leave();
}

static void __mfc_core_otf_put_buf(struct mfc_ctx *ctx)
{
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_buf_info *buf_info = &handle->otf_buf_info;
	int i;

	mfc_debug_enter();

	for (i = 0; i < buf_info->buffer_count; i++) {
		if (buf_info->bufs[i]) {
			dma_buf_put(buf_info->bufs[i]);
			buf_info->bufs[i] = NULL;
		}
	}

	mfc_debug_leave();
}

static int __mfc_core_otf_init_hwfc_buf(struct mfc_ctx *ctx)
{
#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
	struct shared_buffer_info *shared_buf_info;
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_buf_info *buf_info = &handle->otf_buf_info;
#endif

	mfc_debug_enter();

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
	shared_buf_info = (struct shared_buffer_info *)buf_info;
	/* request buffers */
	if (repeater_request_buffer(shared_buf_info, 1)) {
		mfc_ctx_err("[OTF] request_buffer failed\n");
		return -EFAULT;
	}

	if (repeater_register_encode_cb(mfc_hwfc_encode)) {
		mfc_ctx_err("[OTF] failed to register encode call back function\n");
		return -EFAULT;
	}
#endif
	mfc_debug(2, "[OTF] recieved buffer information\n");

	/* set buffer information to ctx, and calculate buffer size */
	if (__mfc_core_otf_set_buf_info(ctx)) {
		mfc_ctx_err("[OTF] failed to set buffer information\n");
		__mfc_core_otf_put_buf(ctx);
		return -EINVAL;
	}

	if (__mfc_core_otf_map_buf(ctx)) {
		mfc_ctx_err("[OTF] failed to map buffers\n");
		__mfc_core_otf_unmap_buf(ctx);
		__mfc_core_otf_put_buf(ctx);
		return -EINVAL;
	}
	mfc_debug(2, "[OTF] HWFC buffer initialized\n");

	mfc_debug_leave();

	return 0;
}

static void __mfc_core_otf_deinit_hwfc_buf(struct mfc_ctx *ctx)
{
	mfc_debug_enter();

	__mfc_core_otf_unmap_buf(ctx);
	__mfc_core_otf_put_buf(ctx);

	mfc_debug(2, "[OTF] HWFC buffer de-initialized\n");

	mfc_debug_leave();
}

static int __mfc_core_otf_create_handle(struct mfc_ctx *ctx)
{
	struct _otf_handle *otf_handle;

	if (!ctx) {
		mfc_pr_err("[OTF] no mfc context to run\n");
		return -EINVAL;
	}

	mfc_debug_enter();

	ctx->otf_handle = kzalloc(sizeof(*otf_handle), GFP_KERNEL);
	if (!ctx->otf_handle) {
		mfc_ctx_err("[OTF] no otf_handle\n");
		return -EINVAL;
	}
	mfc_debug(2, "[OTF] otf_handle created\n");

	mfc_debug_leave();

	return 0;
}

static void __mfc_core_otf_destroy_handle(struct mfc_ctx *ctx)
{
	if (!ctx) {
		mfc_pr_err("[OTF] no mfc context to run\n");
		return;
	}

	mfc_debug_enter();

	kfree(ctx->otf_handle);
	ctx->otf_handle = NULL;
	mfc_debug(2, "[OTF] otf_handle destroyed\n");

	mfc_debug_leave();
}

int mfc_core_otf_create(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	int i;

	mfc_debug_enter();

	for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
		if (test_bit(i, &dev->otf_inst_bits)) {
			mfc_ctx_err("[OTF] otf_handle is already created, ctx: %d\n", i);
			return -EINVAL;
		}
	}

	if (__mfc_core_otf_create_handle(ctx)) {
		mfc_ctx_err("[OTF] otf_handle is not created\n");
		return -EINVAL;
	}

	if (otf_dump) {
		/* It is for debugging. Do not return error */
		if (mfc_otf_alloc_stream_buf(ctx)) {
			mfc_ctx_err("[OTF] stream buffer allocation failed\n");
			mfc_otf_release_stream_buf(ctx);
		}
	}

	set_bit(ctx->num, &dev->otf_inst_bits);
	mfc_debug(2, "[OTF] otf_create is completed\n");

	mfc_debug_leave();

	return 0;
}

void mfc_core_otf_destroy(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev;

	if (!ctx) {
		mfc_pr_err("[OTF] no mfc context to run\n");
		return;
	}

	mfc_debug_enter();

	dev = ctx->dev;

	mfc_otf_release_stream_buf(ctx);
	__mfc_core_otf_destroy_handle(ctx);

	clear_bit(ctx->num, &dev->otf_inst_bits);
	mfc_debug(2, "[OTF] otf_destroy is completed\n");

	mfc_debug_leave();
}

int mfc_core_otf_init(struct mfc_ctx *ctx)
{
	int ret;

	if (!ctx) {
		mfc_pr_err("[OTF] no mfc context to run\n");
		return -EINVAL;
	}

	if (!ctx->otf_handle) {
		mfc_ctx_err("[OTF] otf_handle was not created\n");
		return -EINVAL;
	}

	ret = __mfc_core_otf_init_hwfc_buf(ctx);
	if (ret) {
		mfc_ctx_err("[OTF] HWFC init failed\n");
		return ret;
	}

	mfc_debug(2, "[OTF] otf_init is completed\n");

	return 0;
}

void mfc_core_otf_deinit(struct mfc_ctx *ctx)
{
	if (!ctx) {
		mfc_pr_err("[OTF] no mfc context to run\n");
		return;
	}

	mfc_debug_enter();

	__mfc_core_otf_deinit_hwfc_buf(ctx);
	mfc_debug(2, "[OTF] deinit_otf is completed\n");

	mfc_debug_leave();
}

int mfc_core_otf_ctx_ready_set_bit(struct mfc_core_ctx *core_ctx, struct mfc_bits *data)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct _otf_handle *handle;
	unsigned long flags;
	int is_ready = 0;

	mfc_debug_enter();

	if (!ctx->otf_handle)
		return 0;

	handle = ctx->otf_handle;

	mfc_debug(1, "[OTF] [c:%d] state = %d, otf_work_bit = %d\n",
			ctx->num, core_ctx->state, handle->otf_work_bit);
	/* If shutdown is called, do not try any cmd */
	if (core->shutdown)
		return 0;

	/* The ready condition check and set work_bit should be synchronized */
	spin_lock_irqsave(&data->lock, flags);

	/* Context is to parse header */
	if (core_ctx->state == MFCINST_GOT_INST)
		is_ready = 1;

	/* Context is to set buffers */
	else if (core_ctx->state == MFCINST_HEAD_PARSED)
		is_ready = 1;

	else if (core_ctx->state == MFCINST_RUNNING && handle->otf_work_bit)
		is_ready = 1;

	if (is_ready == 1)
		__set_bit(ctx->num, &data->bits);
	else
		mfc_debug(2, "[OTF] ctx is not ready\n");

	spin_unlock_irqrestore(&data->lock, flags);

	mfc_debug_leave();

	return is_ready;
}

int mfc_core_otf_ctx_ready_clear_bit(struct mfc_core_ctx *core_ctx, struct mfc_bits *data)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct _otf_handle *handle;
	unsigned long flags;
	int is_ready = 0;

	mfc_debug_enter();

	if (!ctx->otf_handle)
		return 0;

	handle = ctx->otf_handle;

	mfc_debug(1, "[OTF] [c:%d] state = %d, otf_work_bit = %d\n",
			ctx->num, core_ctx->state, handle->otf_work_bit);
	/* If shutdown is called, do not try any cmd */
	if (core->shutdown)
		return 0;

	/* The ready condition check and set work_bit should be synchronized */
	spin_lock_irqsave(&data->lock, flags);

	/* Context is to parse header */
	if (core_ctx->state == MFCINST_GOT_INST)
		is_ready = 1;

	/* Context is to set buffers */
	else if (core_ctx->state == MFCINST_HEAD_PARSED)
		is_ready = 1;

	else if (core_ctx->state == MFCINST_RUNNING && handle->otf_work_bit)
		is_ready = 1;

	if (is_ready == 0)
		__clear_bit(ctx->num, &data->bits);
	else
		mfc_debug(2, "[OTF] ctx is ready\n");

	spin_unlock_irqrestore(&data->lock, flags);

	mfc_debug_leave();

	return is_ready;
}

static int __check_disable_header_gen(struct mfc_dev *dev)
{
	unsigned int base_addr = 0xF000;
	unsigned int shift = 6;
	unsigned int mask = 1;

	if (!dev->reg_val)
		return 0;

	if ((dev->reg_val[MFC_REG_E_ENC_OPTIONS - base_addr] >> shift) & mask)
		return 1;

	return 0;
}

int mfc_core_otf_run_enc_init(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_raw_info *raw = &ctx->raw_buf;
#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
	struct packetizing_param packet_param;
#endif
	int ret;

	mfc_debug_enter();

	mfc_core_set_enc_stride(core, ctx);
	mfc_clean_core_ctx_int_flags(core_ctx);

	if (reg_test && !__check_disable_header_gen(dev)) {
#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
		packet_param.time_stamp = 0;
		ret = tsmux_packetize(&packet_param);
		if (ret)
			return ret;
#endif
		mfc_core_otf_set_stream_size(core, ctx, raw->total_plane_size);
	}

	ret = mfc_core_cmd_enc_seq_header(core, ctx);

	mfc_debug_leave();

	return ret;
}

int mfc_core_otf_run_enc_frame(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct _otf_handle *handle = ctx->otf_handle;
	struct mfc_raw_info *raw;

	mfc_debug_enter();

	raw = &ctx->raw_buf;

	if (!handle) {
		mfc_err("[OTF] There is no otf_handle, handle: 0x%p\n", handle);
		return -EINVAL;
	}

	if (!handle->otf_work_bit) {
		mfc_err("[OTF] Can't run OTF encoder, otf_work_bit: %d\n",
				handle->otf_work_bit);
		return -EINVAL;
	}

	if (!core->has_hwfc && (!core->has_dpu_votf || !core->has_mfc_votf)) {
		mfc_err("[OTF] HWFC and vOTF register didn't mapped\n");
		return -EINVAL;
	}

	mfc_core_otf_set_frame_addr(core, ctx, raw->num_planes);
	mfc_core_otf_set_stream_size(core, ctx, raw->total_plane_size);
	if (!(feature_option & MFC_OPTION_OTF_PATH_TEST_ENABLE)) {
		if (core->has_dpu_votf && core->has_mfc_votf)
			mfc_core_otf_set_votf_index(core, ctx, handle->otf_job_id);
		else if (core->has_hwfc)
			mfc_core_otf_set_hwfc_index(core, ctx, handle->otf_job_id);
	}

	if (call_cop(ctx, init_buf_ctrls, ctx, MFC_CTRL_TYPE_SRC, handle->otf_buf_index) < 0)
		mfc_err("failed in init_buf_ctrls\n");
	if (call_cop(ctx, to_buf_ctrls, ctx, &ctx->src_ctrls[handle->otf_buf_index]) < 0)
		mfc_err("failed in to_buf_ctrls\n");
	if (call_cop(ctx, core_set_buf_ctrls_val, core, ctx, &ctx->src_ctrls[handle->otf_buf_index]) < 0)
		mfc_err("[OTF] failed in set_buf_ctrls_val\n");

	/* Change timestamp usec -> nsec */
	mfc_qos_update_last_framerate(ctx, handle->otf_time_stamp * 1000);
	mfc_qos_update_framerate(ctx, 0);
	mfc_rm_qos_control(ctx, MFC_QOS_TRIGGER);

	/* Set stream buffer size to handle buffer full */
	mfc_clean_core_ctx_int_flags(core_ctx);
	mfc_core_cmd_enc_one_frame(core, ctx, 0);

	mfc_debug_leave();

	return 0;
}

int mfc_core_otf_handle_seq(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_enc *enc = ctx->enc_priv;

	mfc_debug_enter();

	enc->header_size = mfc_core_get_enc_strm_size();
#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
	if (enc->header_size)
		tsmux_set_es_size(enc->header_size);
#endif
	ctx->dpb_count = mfc_core_get_enc_dpb_count();
	ctx->scratch_buf_size = mfc_core_get_enc_scratch_size();
	mfc_debug(2, "[OTF][STREAM] encoded slice type: %d, header size: %d, display order: %d\n",
			mfc_core_get_enc_slice_type(), enc->header_size,
			mfc_core_get_enc_pic_count());
	mfc_debug(2, "[OTF] cpb_count: %d, scratch size: %zu\n",
			ctx->dpb_count, ctx->scratch_buf_size);

	mfc_change_state(core_ctx, MFCINST_HEAD_PARSED);

	if (core_ctx->codec_buffer_allocated &&
			(ctx->dpb_count > MFC_OTF_DEFAULT_DPB_COUNT ||
			ctx->scratch_buf_size > MFC_OTF_DEFAULT_SCRATCH_SIZE)) {
		mfc_debug(2, "[OTF] codec buffer will be reallocated. scratch: %zu, count %d\n",
				ctx->scratch_buf_size, ctx->dpb_count);

		if (core->has_llc && core->llc_on_status)
			mfc_llc_flush(core);

		mfc_release_codec_buffers(core_ctx);
	}

	if (!core_ctx->codec_buffer_allocated) {
		if (mfc_alloc_codec_buffers(core_ctx)) {
			mfc_err("[OTF] Failed to allocate encoding buffers\n");
			return -ENOMEM;
		}
	}

	mfc_debug_leave();

	return 0;
}

int mfc_core_otf_handle_stream(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_enc *enc = ctx->enc_priv;
	struct _otf_handle *handle = ctx->otf_handle;
	struct _otf_debug *debug = &handle->otf_debug;
	struct mfc_special_buf *buf;
	struct _otf_buf_addr *buf_addr = &handle->otf_buf_addr;
	struct mfc_raw_info *raw;
	dma_addr_t enc_addr[3] = { 0, 0, 0 };
	int slice_type, i;
	unsigned int strm_size;
	unsigned int pic_count;
	int enc_ret = HWFC_ERR_NONE;
	unsigned int print_size;

	mfc_debug_enter();

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
	tsmux_encoding_end();
#endif

	slice_type = mfc_core_get_enc_slice_type();
	pic_count = mfc_core_get_enc_pic_count();
	strm_size = mfc_core_get_enc_strm_size();
#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
	if (strm_size)
		tsmux_set_es_size(strm_size);
#endif

	mfc_debug(2, "[OTF][STREAM] encoded slice type: %d, size: %d, display order: %d\n",
			slice_type, strm_size, pic_count);

	/* set encoded frame type */
	enc->frame_type = slice_type;
	raw = &ctx->raw_buf;

	if (strm_size > 0) {
		mfc_core_get_enc_frame_buffer(core, ctx, &enc_addr[0], raw->num_planes);

		for (i = 0; i < raw->num_planes; i++)
			mfc_debug(2, "[OTF][BUFINFO] ctx[%d] get src addr[%d]: 0x%08llx\n",
					ctx->num, i, enc_addr[i]);
		if (enc_addr[0] !=  buf_addr->otf_daddr[handle->otf_buf_index][0]) {
			mfc_ctx_err("[OTF] address is not matched. 0x%08llx != 0x%08llx\n",
					enc_addr[0], buf_addr->otf_daddr[handle->otf_buf_index][0]);
			enc_ret = -HWFC_ERR_MFC;
		}
	} else {
		mfc_ctx_err("[OTF] stream size is zero\n");
		enc_ret = -HWFC_ERR_MFC;
	}

	if (otf_dump && !ctx->is_drm) {
		buf = &debug->stream_buf[debug->frame_cnt];
		debug->stream_size[debug->frame_cnt] = strm_size;
		debug->frame_cnt++;
		if (debug->frame_cnt >= OTF_MAX_BUF)
			debug->frame_cnt = 0;
		/* print stream dump */
		print_size = (strm_size * 2) + 64;

		if (buf->vaddr)
			print_hex_dump(KERN_ERR, "OTF dump: ",
					DUMP_PREFIX_OFFSET, print_size, 0,
					buf->vaddr, print_size, false);
	}

	if (call_cop(ctx, core_recover_buf_ctrls_val, core, ctx,
				&ctx->src_ctrls[handle->otf_buf_index]) < 0)
		mfc_ctx_err("[OTF] failed in recover_buf_ctrls_val\n");
	if (call_cop(ctx, cleanup_buf_ctrls, ctx,
				MFC_CTRL_TYPE_SRC, handle->otf_buf_index) < 0)
		mfc_ctx_err("[OTF] failed in cleanup_buf_ctrls\n");

	handle->otf_work_bit = 0;
	handle->otf_buf_index = 0;
	handle->otf_job_id = 0;

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
	repeater_encoding_done(enc_ret);
#endif

	mfc_debug_leave();

	return 0;
}

void mfc_core_otf_handle_error(struct mfc_core *core, struct mfc_ctx *ctx,
		unsigned int reason, unsigned int err)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct _otf_handle *handle = ctx->otf_handle;
	int enc_ret = -HWFC_ERR_MFC;

	mfc_debug_enter();

	mfc_err("[OTF] Interrupt Error: display: %d, decoded: %d\n",
			mfc_get_warn(err), mfc_get_err(err));
	err = mfc_get_err(err);

	/* Error recovery is dependent on the state of context */
	switch (core_ctx->state) {
	case MFCINST_GOT_INST:
	case MFCINST_INIT:
	case MFCINST_RETURN_INST:
	case MFCINST_HEAD_PARSED:
		mfc_err("[OTF] error happened during init/de-init\n");
		break;
	case MFCINST_RUNNING:
		if (err == MFC_REG_ERR_MFC_TIMEOUT) {
			mfc_err("[OTF] MFC TIMEOUT. go to error state\n");
			mfc_change_state(core_ctx, MFCINST_ERROR);
			enc_ret = -HWFC_ERR_MFC_TIMEOUT;
		} else if (err == MFC_REG_ERR_TS_MUX_TIMEOUT ||
				err == MFC_REG_ERR_G2D_TIMEOUT) {
			mfc_err("[OTF] TS-MUX or G2D TIMEOUT. skip this frame\n");
			enc_ret = -HWFC_ERR_MFC_TIMEOUT;
		} else {
			mfc_err("[OTF] MFC ERROR. skip this frame\n");
			enc_ret = -HWFC_ERR_MFC;
		}

		handle->otf_work_bit = 0;
		handle->otf_buf_index = 0;
		handle->otf_job_id = 0;

		if (call_cop(ctx, core_recover_buf_ctrls_val, core, ctx,
					&ctx->src_ctrls[handle->otf_buf_index]) < 0)
			mfc_err("[OTF] failed in recover_buf_ctrls_val\n");
		if (call_cop(ctx, cleanup_buf_ctrls, ctx,
					MFC_CTRL_TYPE_SRC, handle->otf_buf_index) < 0)
			mfc_err("[OTF] failed in cleanup_buf_ctrls\n");

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
		repeater_encoding_done(enc_ret);
#endif
		break;
	default:
		mfc_err("Encountered an error interrupt which had not been handled\n");
		mfc_err("core_ctx->state = %d, core_ctx->inst_no = %d\n",
						core_ctx->state, core_ctx->inst_no);
		break;
	}

	mfc_wake_up_core(core, reason, err);

	mfc_debug_leave();
}

void mfc_core_otf_path_test(struct mfc_ctx *ctx)
{
#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
	struct repeater_encoding_param enc_param;
#endif
	int ret;
#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
	/* After generating the header, time is required until TS-MUX is ready */
	msleep(20);

	enc_param.time_stamp = 0;
	ret = mfc_hwfc_encode(0, 0, &enc_param);
#else
	ret = 0;
#endif
	if (ret)
		mfc_ctx_err("[OTF] OTF path test is failed (err: -%d)\n", ret);
}

int __mfc_hwfc_check_run(struct mfc_core_ctx *core_ctx)
{
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct _otf_handle *handle = ctx->otf_handle;

	mfc_debug_enter();

	if (!handle) {
		mfc_err("[OTF] there is no handle for OTF\n");
		return -EINVAL;
	}
	if (handle->otf_work_bit) {
		mfc_err("[OTF] OTF is already working\n");
		return -EINVAL;
	}
	if (core_ctx->state != MFCINST_RUNNING) {
		mfc_err("[OTF] mfc is not running state\n");
		return -EINVAL;
	}

	mfc_debug_leave();

	return 0;
}

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
int mfc_hwfc_encode(int buf_index, int job_id,
		struct repeater_encoding_param *param)
{
	struct mfc_dev *dev = g_mfc_dev;
	struct mfc_core *core = NULL;
	struct mfc_core_ctx *core_ctx;
	struct _otf_handle *handle;
	struct mfc_ctx *ctx = NULL;
#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
	struct packetizing_param packet_param;
#endif
	int i;

	mfc_dev_debug_enter();

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
	tsmux_encoding_start(buf_index);
#endif

	for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
		if (test_bit(i, &dev->otf_inst_bits)) {
			ctx = dev->ctx[i];
			break;
		}
	}

	if (!ctx) {
		mfc_dev_err("[OTF] there is no context to run\n");
		return -HWFC_ERR_MFC_NOT_PREPARED;
	}

	core = mfc_get_main_core(dev, ctx);
	if (!core) {
		mfc_ctx_err("[OTF] There is no main core\n");
		return -HWFC_ERR_MFC_NOT_PREPARED;
	}

	core_ctx = core->core_ctx[ctx->num];

	if (__mfc_hwfc_check_run(core_ctx)) {
		mfc_err("[OTF] mfc is not prepared\n");
		return -HWFC_ERR_MFC_NOT_PREPARED;
	}

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_TSMUX)
	packet_param.time_stamp = param->time_stamp;
	if (debug_ts == 1)
		mfc_ctx_info("[OTF][TS] timestamp: %llu\n", param->time_stamp);
	if (tsmux_packetize(&packet_param)) {
		mfc_err("[OTF] packetize failed\n");
		return -HWFC_ERR_TSMUX;
	}
#endif

	if (ctx->otf_handle) {
		handle = ctx->otf_handle;
	} else {
		mfc_err("[OTF] there is no otf_handle\n");
		return -HWFC_ERR_MFC_NOT_PREPARED;
	}
	handle->otf_work_bit = 1;
	handle->otf_buf_index = buf_index;
	handle->otf_job_id = job_id;
	handle->otf_time_stamp = param->time_stamp;

	mfc_core_otf_ctx_ready_set_bit(core_ctx, &core->work_bits);
	if (mfc_core_is_work_to_do(core))
		queue_work(core->butler_wq, &core->butler_work);

	mfc_dev_debug_leave();

	return HWFC_ERR_NONE;
}
#endif

