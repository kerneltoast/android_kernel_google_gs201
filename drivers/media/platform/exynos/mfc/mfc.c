/*
 * drivers/media/platform/exynos/mfc/mfc.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/soc/samsung/exynos-smc.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/poll.h>

#include "mfc_common.h"

#include "mfc_dec_v4l2.h"
#include "mfc_enc_v4l2.h"
#include "mfc_rm.h"

#include "mfc_core_run.h"
#include "mfc_core_otf.h"
#include "mfc_debugfs.h"
#include "mfc_sync.h"
#include "mfc_meminfo.h"
#include "mfc_memlog.h"

#include "mfc_core_hwlock.h"
#include "mfc_core_pm.h"
#include "mfc_core_hw_reg_api.h"
#include "mfc_llc.h"

#include "mfc_qos.h"
#include "mfc_queue.h"
#include "mfc_utils.h"
#include "mfc_buf.h"
#include "mfc_mem.h"

#define CREATE_TRACE_POINTS
#include <trace/events/mfc.h>

#define MFC_NAME			"s5p-mfc"
#define MFC_DEC_NAME			"s5p-mfc-dec"
#define MFC_ENC_NAME			"s5p-mfc-enc"
#define MFC_DEC_DRM_NAME		"s5p-mfc-dec-secure"
#define MFC_ENC_DRM_NAME		"s5p-mfc-enc-secure"
#define MFC_ENC_OTF_NAME		"s5p-mfc-enc-otf"
#define MFC_ENC_OTF_DRM_NAME		"s5p-mfc-enc-otf-secure"

struct _mfc_trace g_mfc_trace[MFC_TRACE_COUNT_MAX];
struct _mfc_trace g_mfc_trace_rm[MFC_TRACE_COUNT_MAX];
struct _mfc_trace g_mfc_trace_longterm[MFC_TRACE_COUNT_MAX];
struct mfc_dev *g_mfc_dev;

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
static struct proc_dir_entry *mfc_proc_entry;

#define MFC_PROC_ROOT			"mfc"
#define MFC_PROC_INSTANCE_NUMBER	"instance_number"
#define MFC_PROC_DRM_INSTANCE_NUMBER	"drm_instance_number"
#define MFC_PROC_FW_STATUS		"fw_status"
#endif

#define DEF_DEC_SRC_FMT	9
#define DEF_DEC_DST_FMT	5

#define DEF_ENC_SRC_FMT	5
#define DEF_ENC_DST_FMT	13

extern struct mfc_ctrls_ops decoder_ctrls_ops;
extern struct vb2_ops mfc_dec_qops;
extern struct mfc_fmt dec_formats[];

void mfc_butler_worker(struct work_struct *work)
{
	struct mfc_dev *dev;
	struct mfc_ctx *ctx;
	int i;

	dev = container_of(work, struct mfc_dev, butler_work);

	/* If there is multi core instance, it has high priority */
	if (dev->multi_core_inst_bits) {
		i = __ffs(dev->multi_core_inst_bits);
		ctx = dev->ctx[i];
		if (!ctx) {
			mfc_dev_err("[RM] There is no ctx\n");
			return;
		}

		if (!IS_TWO_MODE2(ctx))
			return;

		mfc_rm_request_work(dev, MFC_WORK_TRY, ctx);
	} else {
		mfc_rm_request_work(dev, MFC_WORK_BUTLER, NULL);
	}
}

static void __mfc_deinit_dec_ctx(struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;

	mfc_cleanup_iovmm(ctx);

	mfc_delete_queue(&ctx->src_buf_ready_queue);
	mfc_delete_queue(&ctx->dst_buf_queue);
	mfc_delete_queue(&ctx->dst_buf_err_queue);
	mfc_delete_queue(&ctx->src_buf_nal_queue);
	mfc_delete_queue(&ctx->dst_buf_nal_queue);
	mfc_delete_queue(&ctx->meminfo_inbuf_q);

	mfc_mem_cleanup_user_shared_handle(ctx, &dec->sh_handle_dpb);
	mfc_mem_cleanup_user_shared_handle(ctx, &dec->sh_handle_hdr);

	if (dec->ref_info)
		vfree(dec->ref_info);

	if (dec->hdr10_plus_info)
		vfree(dec->hdr10_plus_info);

	if (dec->av1_film_grain_info)
		vfree(dec->av1_film_grain_info);

	kfree(dec);
}

static int __mfc_init_dec_ctx(struct mfc_ctx *ctx)
{
	struct mfc_dec *dec;
	int ret = 0;
	int i;

	dec = kzalloc(sizeof(struct mfc_dec), GFP_KERNEL);
	if (!dec) {
		mfc_ctx_err("failed to allocate decoder private data\n");
		return -ENOMEM;
	}
	ctx->dec_priv = dec;

	ctx->subcore_inst_no = MFC_NO_INSTANCE_SET;
	ctx->curr_src_index = -1;

	mfc_create_queue(&ctx->src_buf_ready_queue);
	mfc_create_queue(&ctx->dst_buf_queue);
	mfc_create_queue(&ctx->dst_buf_err_queue);
	mfc_create_queue(&ctx->src_buf_nal_queue);
	mfc_create_queue(&ctx->dst_buf_nal_queue);
	mfc_create_queue(&ctx->meminfo_inbuf_q);

	for (i = 0; i < MFC_MAX_BUFFERS; i++) {
		INIT_LIST_HEAD(&ctx->src_ctrls[i]);
		INIT_LIST_HEAD(&ctx->dst_ctrls[i]);
	}
	ctx->src_ctrls_avail = 0;
	ctx->dst_ctrls_avail = 0;

	ctx->capture_state = QUEUE_FREE;
	ctx->output_state = QUEUE_FREE;

	ctx->type = MFCINST_DECODER;
	ctx->c_ops = &decoder_ctrls_ops;
	ctx->src_fmt = &dec_formats[DEF_DEC_SRC_FMT];
	ctx->dst_fmt = &dec_formats[DEF_DEC_DST_FMT];

	mfc_qos_reset_framerate(ctx);

	ctx->qos_ratio = 100;
	INIT_LIST_HEAD(&ctx->bitrate_list);
	INIT_LIST_HEAD(&ctx->ts_list);

	dec->display_delay = -1;
	dec->is_interlaced = 0;
	dec->immediate_display = 0;
	dec->is_dts_mode = 0;
	dec->inter_res_change = 0;
	dec->disp_drc.disp_res_change = 0;
	dec->disp_drc.push_idx = 0;
	dec->disp_drc.pop_idx = 0;

	dec->is_dynamic_dpb = 1;
	dec->dynamic_used = 0;
	dec->is_dpb_full = 0;
	dec->queued_dpb = 0;
	dec->display_index = -1;
	dec->dpb_table_used = 0;
	dec->sh_handle_dpb.fd = -1;
	mutex_init(&dec->dpb_mutex);
	mutex_init(&ctx->cpb_mutex);

	mfc_init_dpb_table(ctx);

	dec->sh_handle_dpb.data_size = sizeof(struct dec_dpb_ref_info) * MFC_MAX_BUFFERS;
	dec->ref_info = vmalloc(dec->sh_handle_dpb.data_size);
	if (!dec->ref_info) {
		mfc_ctx_err("failed to allocate decoder information data\n");
		ret = -ENOMEM;
		goto fail_dec_init;
	}
	for (i = 0; i < MFC_MAX_BUFFERS; i++)
		dec->ref_info[i].dpb[0].fd[0] = MFC_INFO_INIT_FD;

	dec->sh_handle_hdr.fd = -1;

	/* Init videobuf2 queue for OUTPUT */
	ctx->vq_src.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	ctx->vq_src.drv_priv = ctx;
	ctx->vq_src.buf_struct_size = sizeof(struct mfc_buf);
	ctx->vq_src.io_modes = VB2_USERPTR | VB2_DMABUF;
	ctx->vq_src.ops = &mfc_dec_qops;
	ctx->vq_src.mem_ops = mfc_mem_ops();
	ctx->vq_src.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	ret = vb2_queue_init(&ctx->vq_src);
	if (ret) {
		mfc_ctx_err("Failed to initialize videobuf2 queue(output)\n");
		goto fail_dec_init;
	}
	/* Init videobuf2 queue for CAPTURE */
	ctx->vq_dst.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	ctx->vq_dst.drv_priv = ctx;
	ctx->vq_dst.buf_struct_size = sizeof(struct mfc_buf);
	ctx->vq_dst.io_modes = VB2_USERPTR | VB2_DMABUF;
	ctx->vq_dst.ops = &mfc_dec_qops;
	ctx->vq_dst.mem_ops = mfc_mem_ops();
	ctx->vq_dst.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	ret = vb2_queue_init(&ctx->vq_dst);
	if (ret) {
		mfc_ctx_err("Failed to initialize videobuf2 queue(capture)\n");
		goto fail_dec_init;
	}

	return ret;

fail_dec_init:
	__mfc_deinit_dec_ctx(ctx);
	return ret;
}

extern struct mfc_ctrls_ops encoder_ctrls_ops;
extern struct vb2_ops mfc_enc_qops;
extern struct mfc_fmt enc_formats[];

static void __mfc_deinit_enc_ctx(struct mfc_ctx *ctx)
{
	struct mfc_enc *enc = ctx->enc_priv;

	mfc_delete_queue(&ctx->src_buf_ready_queue);
	mfc_delete_queue(&ctx->dst_buf_queue);
	mfc_delete_queue(&ctx->src_buf_nal_queue);
	mfc_delete_queue(&ctx->dst_buf_nal_queue);
	mfc_delete_queue(&ctx->ref_buf_queue);
	mfc_delete_queue(&ctx->meminfo_inbuf_q);
	mfc_delete_queue(&ctx->meminfo_outbuf_q);

	mfc_mem_cleanup_user_shared_handle(ctx, &enc->sh_handle_svc);
	mfc_mem_cleanup_user_shared_handle(ctx, &enc->sh_handle_roi);
	mfc_mem_cleanup_user_shared_handle(ctx, &enc->sh_handle_hdr);
	kfree(enc);
}

static int __mfc_init_enc_ctx(struct mfc_ctx *ctx)
{
	struct mfc_enc *enc;
	struct mfc_enc_params *p;
	int ret = 0;
	int i;

	enc = kzalloc(sizeof(struct mfc_enc), GFP_KERNEL);
	if (!enc) {
		mfc_ctx_err("failed to allocate encoder private data\n");
		return -ENOMEM;
	}
	ctx->enc_priv = enc;

	mfc_create_queue(&ctx->src_buf_ready_queue);
	mfc_create_queue(&ctx->dst_buf_queue);
	mfc_create_queue(&ctx->src_buf_nal_queue);
	mfc_create_queue(&ctx->dst_buf_nal_queue);
	mfc_create_queue(&ctx->ref_buf_queue);
	mfc_create_queue(&ctx->meminfo_inbuf_q);
	mfc_create_queue(&ctx->meminfo_outbuf_q);

	for (i = 0; i < MFC_MAX_BUFFERS; i++) {
		INIT_LIST_HEAD(&ctx->src_ctrls[i]);
		INIT_LIST_HEAD(&ctx->dst_ctrls[i]);
	}
	ctx->src_ctrls_avail = 0;
	ctx->dst_ctrls_avail = 0;

	ctx->type = MFCINST_ENCODER;
	ctx->c_ops = &encoder_ctrls_ops;
	ctx->src_fmt = &enc_formats[DEF_ENC_SRC_FMT];
	ctx->dst_fmt = &enc_formats[DEF_ENC_DST_FMT];

	mfc_qos_reset_framerate(ctx);

	ctx->qos_ratio = 100;

	/* disable IVF header by default (VP8, VP9) */
	p = &enc->params;
	p->ivf_header_disable = 1;

	INIT_LIST_HEAD(&ctx->bitrate_list);
	INIT_LIST_HEAD(&ctx->ts_list);

	enc->sh_handle_svc.fd = -1;
	enc->sh_handle_roi.fd = -1;
	enc->sh_handle_hdr.fd = -1;
	enc->sh_handle_svc.data_size = sizeof(struct temporal_layer_info);
	enc->sh_handle_roi.data_size = sizeof(struct mfc_enc_roi_info);
	enc->sh_handle_hdr.data_size = sizeof(struct hdr10_plus_meta) * MFC_MAX_BUFFERS;

	/* Init videobuf2 queue for OUTPUT */
	ctx->vq_src.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	ctx->vq_src.drv_priv = ctx;
	ctx->vq_src.buf_struct_size = sizeof(struct mfc_buf);
	ctx->vq_src.io_modes = VB2_USERPTR | VB2_DMABUF;
	ctx->vq_src.ops = &mfc_enc_qops;
	ctx->vq_src.mem_ops = mfc_mem_ops();
	ctx->vq_src.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	ret = vb2_queue_init(&ctx->vq_src);
	if (ret) {
		mfc_ctx_err("Failed to initialize videobuf2 queue(output)\n");
		goto fail_enc_init;
	}

	/* Init videobuf2 queue for CAPTURE */
	ctx->vq_dst.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	ctx->vq_dst.drv_priv = ctx;
	ctx->vq_dst.buf_struct_size = sizeof(struct mfc_buf);
	ctx->vq_dst.io_modes = VB2_USERPTR | VB2_DMABUF;
	ctx->vq_dst.ops = &mfc_enc_qops;
	ctx->vq_dst.mem_ops = mfc_mem_ops();
	ctx->vq_dst.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	ret = vb2_queue_init(&ctx->vq_dst);
	if (ret) {
		mfc_ctx_err("Failed to initialize videobuf2 queue(capture)\n");
		goto fail_enc_init;
	}

	return 0;

fail_enc_init:
	__mfc_deinit_enc_ctx(ctx);
	return 0;
}

/* Open an MFC node */
static int mfc_open(struct file *file)
{
	struct mfc_ctx *ctx = NULL;
	struct mfc_dev *dev = video_drvdata(file);
	int i, ret = 0;
	enum mfc_node_type node;
	struct video_device *vdev = NULL;

	if (!dev) {
		mfc_pr_err("no mfc device to run\n");
		goto err_no_device;
	}

	mfc_dev_debug(2, "mfc driver open called\n");

	if (mutex_lock_interruptible(&dev->mfc_mutex))
		return -ERESTARTSYS;

	node = mfc_get_node_type(file);
	if (node == MFCNODE_INVALID) {
		mfc_dev_err("cannot specify node type\n");
		ret = -ENOENT;
		goto err_node_type;
	}

	dev->num_inst++;	/* It is guarded by mfc_mutex in vfd */

	/* Allocate memory for context */
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		mfc_dev_err("Not enough memory\n");
		ret = -ENOMEM;
		goto err_ctx_alloc;
	}

	switch (node) {
	case MFCNODE_DECODER:
		vdev = dev->vfd_dec;
		break;
	case MFCNODE_ENCODER:
		vdev = dev->vfd_enc;
		break;
	case MFCNODE_DECODER_DRM:
		vdev = dev->vfd_dec_drm;
		break;
	case MFCNODE_ENCODER_DRM:
		vdev = dev->vfd_enc_drm;
		break;
	case MFCNODE_ENCODER_OTF:
		vdev = dev->vfd_enc_otf;
		break;
	case MFCNODE_ENCODER_OTF_DRM:
		vdev = dev->vfd_enc_otf_drm;
		break;
	default:
		mfc_dev_err("Invalid node(%d)\n", node);
		break;
	}

	if (!vdev)
		goto err_vdev;

	v4l2_fh_init(&ctx->fh, vdev);
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	ctx->dev = dev;

	/* Get context number */
	ctx->num = 0;
	while (dev->ctx[ctx->num]) {
		ctx->num++;
		if (ctx->num >= MFC_NUM_CONTEXTS) {
			mfc_ctx_err("Too many open contexts\n");
			mfc_ctx_err("Print information to check if there was an error or not\n");
			call_dop(dev, dump_info_context, dev);
			ret = -EBUSY;
			goto err_ctx_num;
		}
	}

	spin_lock_init(&ctx->buf_queue_lock);
	spin_lock_init(&ctx->meminfo_queue_lock);
	spin_lock_init(&ctx->corelock.lock);
	mutex_init(&ctx->intlock.core_mutex);
	mutex_init(&ctx->drc_wait_mutex);
	init_waitqueue_head(&ctx->migrate_wq);
	init_waitqueue_head(&ctx->corelock.wq);
	init_waitqueue_head(&ctx->corelock.migrate_wq);

	if (mfc_is_decoder_node(node))
		ret = __mfc_init_dec_ctx(ctx);
	else
		ret = __mfc_init_enc_ctx(ctx);
	if (ret)
		goto err_ctx_init;

	if (dev->num_inst == 1) {
		/* regression test val */
		if (regression_option) {
			dev->regression_val = vmalloc(SZ_1M);
			if (!dev->regression_val)
				mfc_ctx_err("[MFCREGRESSION] failed to allocate regression result data\n");
		}

		/* all of the ctx list */
		INIT_LIST_HEAD(&dev->ctx_list);
		spin_lock_init(&dev->ctx_list_lock);
	}

	ret = call_cop(ctx, init_ctx_ctrls, ctx);
	if (ret) {
		mfc_ctx_err("failed in init_ctx_ctrls\n");
		goto err_ctx_ctrls;
	}

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (mfc_is_drm_node(node)) {
		if (dev->num_drm_inst < MFC_MAX_DRM_CTX) {
			dev->num_drm_inst++;
			ctx->is_drm = 1;

			mfc_ctx_info("DRM instance is opened [%d:%d]\n",
					dev->num_drm_inst, dev->num_inst);
		} else {
			mfc_ctx_err("Too many instance are opened for DRM\n");
			mfc_ctx_err("Print information to check if there was an error or not\n");
			call_dop(dev, dump_info_context, dev);
			ret = -EINVAL;
			goto err_drm_start;
		}
	} else {
		mfc_ctx_info("NORMAL instance is opened [%d:%d]\n",
				dev->num_drm_inst, dev->num_inst);
	}
#endif

	/* Mark context as idle */
	dev->ctx[ctx->num] = ctx;
	for (i = 0; i < MFC_NUM_CORE; i++)
		ctx->op_core_num[i] = MFC_CORE_INVALID;

	ret = mfc_rm_instance_init(dev, ctx);
	if (ret) {
		mfc_ctx_err("rm_instance_init failed\n");
		goto err_inst_init;
	}

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
	if (mfc_is_encoder_otf_node(node)) {
		ret = mfc_core_otf_create(ctx);
		if (ret)
			mfc_ctx_err("[OTF] otf_create failed\n");
	}
#endif

	trace_mfc_node_open(ctx->num, dev->num_inst, ctx->type, ctx->is_drm);
	mfc_ctx_info("MFC open completed [%d:%d] version = %d\n",
			dev->num_drm_inst, dev->num_inst, MFC_DRIVER_INFO);
	MFC_TRACE_CTX_LT("[INFO] %s %s opened (ctx:%d, total:%d)\n", ctx->is_drm ? "DRM" : "Normal",
			mfc_is_decoder_node(node) ? "DEC" : "ENC", ctx->num, dev->num_inst);

	queue_work(dev->butler_wq, &dev->butler_work);

	mutex_unlock(&dev->mfc_mutex);
	return ret;

	/* Deinit when failure occured */
err_inst_init:
	if (ctx->is_drm)
		dev->num_drm_inst--;

err_drm_start:
	call_cop(ctx, cleanup_ctx_ctrls, ctx);

err_ctx_ctrls:
	if ((dev->num_inst == 1) && dev->regression_val)
		vfree(dev->regression_val);

err_ctx_init:
	dev->ctx[ctx->num] = 0;

err_ctx_num:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

err_vdev:
	kfree(ctx);

err_ctx_alloc:
	dev->num_inst--;

err_node_type:
	mfc_dev_info("MFC driver open is failed [%d:%d]\n",
			dev->num_drm_inst, dev->num_inst);
	mutex_unlock(&dev->mfc_mutex);

err_no_device:

	return ret;
}

/* Release MFC context */
static int mfc_release(struct file *file)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dev *dev = ctx->dev;
	struct mfc_ctx *move_ctx;
	struct mfc_core *maincore;
	struct mfc_core *subcore;
	int ret = 0;
	int i;

	mutex_lock(&dev->sysmmu_fault_mutex);
	mutex_lock(&dev->mfc_mutex);
	mutex_lock(&dev->mfc_migrate_mutex);

	mfc_ctx_info("MFC driver release is called [%d:%d], is_drm(%d)\n",
			dev->num_drm_inst, dev->num_inst, ctx->is_drm);

	MFC_TRACE_CTX_LT("[INFO] release is called (ctx:%d, total:%d)\n", ctx->num, dev->num_inst);

	/* Free resources */
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	maincore = mfc_get_main_core_wait(dev, ctx);
	atomic_inc(&maincore->during_release);
	subcore = mfc_get_sub_core(dev, ctx);
	if (subcore)
		atomic_inc(&subcore->during_release);

	/* Trigger idle resume if core is in the idle mode for stopping NAL_Q */
	mfc_rm_qos_control(ctx, MFC_QOS_TRIGGER);

	/*
	 * mfc_release() can be called without a streamoff
	 * when the application is forcibly terminated.
	 * At that time, stop_streaming() is called by vb2_queue_release.
	 * So, we need to performed stop_streaming
	 * before instance de-init(CLOSE_INSTANCE).
	 */
	vb2_queue_release(&ctx->vq_src);
	vb2_queue_release(&ctx->vq_dst);

	if (call_cop(ctx, cleanup_ctx_ctrls, ctx) < 0)
		mfc_ctx_err("failed in cleanup_ctx_ctrl\n");

	ret = mfc_rm_instance_deinit(dev, ctx);
	if (ret) {
		mfc_dev_err("failed to rm_instance_deinit\n");
		goto end_release;
	}

	if (ctx->is_drm)
		dev->num_drm_inst--;
	dev->num_inst--;
	dev->regression_cnt = 0;

	if (IS_MULTI_CORE_DEVICE(dev))
		mfc_rm_load_balancing(ctx, MFC_RM_LOAD_DELETE_UPDATE);

	mfc_meminfo_cleanup_inbuf_q(ctx);
	if (ctx->type == MFCINST_ENCODER)
		mfc_meminfo_cleanup_outbuf_q(ctx);

	if ((dev->num_inst == 0) && dev->regression_val)
		vfree(dev->regression_val);

	if (ctx->type == MFCINST_DECODER)
		__mfc_deinit_dec_ctx(ctx);
	else if (ctx->type == MFCINST_ENCODER)
		__mfc_deinit_enc_ctx(ctx);

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
	if (ctx->otf_handle) {
		mfc_core_otf_deinit(ctx);
		mfc_core_otf_destroy(ctx);
	}
#endif

	trace_mfc_node_close(ctx->num, dev->num_inst, ctx->type, ctx->is_drm);

	MFC_TRACE_CTX_LT("[INFO] Release finished (ctx:%d, total:%d)\n", ctx->num, dev->num_inst);

	/* If ctx is move_ctx in migration worker, clear move_ctx */
	for (i = 0; i < dev->move_ctx_cnt; i++) {
		move_ctx = dev->move_ctx[i];
		if (move_ctx && (move_ctx->num == ctx->num)) {
			dev->move_ctx[i] = NULL;
			break;
		}
	}

	dev->ctx[ctx->num] = NULL;
	kfree(ctx);

	mfc_dev_info("mfc driver release finished [%d:%d]\n", dev->num_drm_inst, dev->num_inst);

	queue_work(dev->butler_wq, &dev->butler_work);

end_release:
	atomic_dec(&maincore->during_release);
	if (subcore)
		atomic_dec(&subcore->during_release);
	mutex_unlock(&dev->mfc_migrate_mutex);
	mutex_unlock(&dev->mfc_mutex);
	mutex_unlock(&dev->sysmmu_fault_mutex);
	return ret;
}

/* Poll */
static __poll_t mfc_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	unsigned long req_events = poll_requested_events(wait);
	__poll_t ret = 0;

	mfc_debug_enter();

	if (mfc_rm_query_state(ctx, EQUAL, MFCINST_ERROR)) {
		if (req_events & (POLLOUT | POLLWRNORM))
			mfc_ctx_err("SRC: Call on POLL after unrecoverable error\n");
		else
			mfc_ctx_err("DST: Call on POLL after unrecoverable error\n");
		return EPOLLERR;
	}

	if (req_events & (POLLOUT | POLLWRNORM)) {
		mfc_debug(2, "wait source buffer\n");
		ret = vb2_poll(&ctx->vq_src, file, wait);
	} else if (req_events & (POLLIN | POLLRDNORM)) {
		mfc_debug(2, "wait destination buffer\n");
		ret = vb2_poll(&ctx->vq_dst, file, wait);
	}

	mfc_debug_leave();
	return ret;
}

/* Mmap */
static int mfc_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret;

	mfc_debug_enter();

	if (offset < DST_QUEUE_OFF_BASE) {
		mfc_debug(2, "mmaping source\n");
		ret = vb2_mmap(&ctx->vq_src, vma);
	} else {		/* capture */
		mfc_debug(2, "mmaping destination\n");
		vma->vm_pgoff -= (DST_QUEUE_OFF_BASE >> PAGE_SHIFT);
		ret = vb2_mmap(&ctx->vq_dst, vma);
	}
	mfc_debug_leave();
	return ret;
}

/* v4l2 ops */
static const struct v4l2_file_operations mfc_fops = {
	.owner = THIS_MODULE,
	.open = mfc_open,
	.release = mfc_release,
	.poll = mfc_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = mfc_mmap,
};

static void __mfc_create_bitrate_table(struct mfc_dev *dev)
{
	struct mfc_platdata *pdata = dev->pdata;
	int i;
	int max_freq, freq_ratio;

	max_freq = pdata->mfc_freqs[pdata->num_mfc_freq - 1];
	dev->bps_ratio = pdata->max_Kbps[0] / dev->pdata->max_Kbps[1];

	for (i = 0; i < pdata->num_mfc_freq; i++) {
		freq_ratio = pdata->mfc_freqs[i] * 100 / max_freq;
		dev->bitrate_table[i].bps_interval = pdata->max_Kbps[0] * freq_ratio / 100;
		dev->bitrate_table[i].mfc_freq = pdata->mfc_freqs[i];
		mfc_dev_info("[QoS] bitrate table[%d] %dKHz: ~ %dKbps\n",
				i, dev->bitrate_table[i].mfc_freq,
				dev->bitrate_table[i].bps_interval);
	}
}

static int __mfc_parse_dt(struct device_node *np, struct mfc_dev *mfc)
{
	struct mfc_platdata *pdata = mfc->pdata;

	if (!np) {
		dev_err(mfc->device, "there is no device node\n");
		return -EINVAL;
	}

	/* MFC version */
	of_property_read_u32(np, "ip_ver", &pdata->ip_ver);

	/* Debug mode */
	of_property_read_u32(np, "debug_mode", &pdata->debug_mode);

	/* NAL-Q size */
	of_property_read_u32(np, "nal_q_entry_size", &pdata->nal_q_entry_size);
	of_property_read_u32(np, "nal_q_dump_size", &pdata->nal_q_dump_size);

	/* Features */
	of_property_read_u32_array(np, "nal_q", &pdata->nal_q.support, 2);
	of_property_read_u32_array(np, "skype", &pdata->skype.support, 2);
	of_property_read_u32_array(np, "black_bar",
			&pdata->black_bar.support, 2);
	of_property_read_u32_array(np, "color_aspect_dec",
			&pdata->color_aspect_dec.support, 2);
	of_property_read_u32_array(np, "static_info_dec",
			&pdata->static_info_dec.support, 2);
	of_property_read_u32_array(np, "color_aspect_enc",
			&pdata->color_aspect_enc.support, 2);
	of_property_read_u32_array(np, "static_info_enc",
			&pdata->static_info_enc.support, 2);
	of_property_read_u32_array(np, "hdr10_plus",
			&pdata->hdr10_plus.support, 2);
	of_property_read_u32_array(np, "vp9_stride_align",
			&pdata->vp9_stride_align.support, 2);
	of_property_read_u32_array(np, "sbwc_uncomp",
			&pdata->sbwc_uncomp.support, 2);
	of_property_read_u32_array(np, "mem_clear",
			&pdata->mem_clear.support, 2);
	of_property_read_u32_array(np, "wait_fw_status",
			&pdata->wait_fw_status.support, 2);
	of_property_read_u32_array(np, "wait_nalq_status",
			&pdata->wait_nalq_status.support, 2);
	of_property_read_u32_array(np, "drm_switch_predict",
			&pdata->drm_switch_predict.support, 2);
	of_property_read_u32_array(np, "sbwc_enc_src_ctrl",
			&pdata->sbwc_enc_src_ctrl.support, 2);
	of_property_read_u32_array(np, "average_qp",
			&pdata->average_qp.support, 2);
	of_property_read_u32_array(np, "mv_search_mode",
			&pdata->mv_search_mode.support, 2);
	of_property_read_u32_array(np, "enc_idr_flag",
			&pdata->enc_idr_flag.support, 2);
	of_property_read_u32_array(np, "min_quality_mode",
			&pdata->min_quality_mode.support, 2);
	of_property_read_u32_array(np, "hevc_pic_output_flag",
			&pdata->hevc_pic_output_flag.support, 2);
	of_property_read_u32_array(np, "metadata_interface",
			&pdata->metadata_interface.support, 2);
	of_property_read_u32_array(np, "hdr10_plus_full",
			&pdata->hdr10_plus_full.support, 2);
	of_property_read_u32_array(np, "enc_capability",
			&pdata->enc_capability.support, 2);
	of_property_read_u32_array(np, "enc_sub_gop",
			&pdata->enc_sub_gop.support, 2);
	of_property_read_u32_array(np, "enc_ts_delta",
			&pdata->enc_ts_delta.support, 2);

	/* Determine whether to enable AV1 decoder */
	of_property_read_u32(np, "support_av1_dec", &pdata->support_av1_dec);
	of_property_read_u32_array(np, "av1_film_grain",
			&pdata->av1_film_grain.support, 2);
	/* Override value if AV1 decoder is not supported. */
	if (!pdata->support_av1_dec)
		pdata->av1_film_grain.support = 0;

	/* H/W limitation or option */
	of_property_read_u32(np, "P010_decoding", &pdata->P010_decoding);
	of_property_read_u32(np, "dithering_enable", &pdata->dithering_enable);
	of_property_read_u32(np, "stride_align", &pdata->stride_align);
	of_property_read_u32(np, "stride_type", &pdata->stride_type);
	of_property_read_u32(np, "support_8K_cavlc", &pdata->support_8K_cavlc);

	/* Formats */
	of_property_read_u32(np, "support_10bit", &pdata->support_10bit);
	of_property_read_u32(np, "support_422", &pdata->support_422);
	of_property_read_u32(np, "support_rgb", &pdata->support_rgb);

	/* SBWC */
	of_property_read_u32(np, "support_sbwc", &pdata->support_sbwc);
	of_property_read_u32(np, "support_sbwcl", &pdata->support_sbwcl);

	/* SBWC */
	of_property_read_u32(np, "sbwc_dec_max_width", &pdata->sbwc_dec_max_width);
	of_property_read_u32(np, "sbwc_dec_max_height", &pdata->sbwc_dec_max_height);
	of_property_read_u32(np, "sbwc_dec_hdr10_off", &pdata->sbwc_dec_hdr10_off);

	/* HDR10+ num max window */
	of_property_read_u32(np, "max_hdr_win", &pdata->max_hdr_win);

	/* HDR10+ num max window */
	of_property_read_u32(np, "display_err_type", &pdata->display_err_type);

	/* security ctrl */
	of_property_read_u32(np, "security_ctrl", &pdata->security_ctrl);

	/* Encoder min bit count control */
	of_property_read_u32(np, "enc_min_bit_cnt", &pdata->enc_min_bit_cnt);

	/* Encoder default parameter */
	of_property_read_u32(np, "enc_param_num", &pdata->enc_param_num);
	if (pdata->enc_param_num) {
		of_property_read_u32_array(np, "enc_param_addr",
				pdata->enc_param_addr, pdata->enc_param_num);
		of_property_read_u32_array(np, "enc_param_val",
				pdata->enc_param_val, pdata->enc_param_num);
	}

	/* MFC bandwidth information */
	of_property_read_u32_array(np, "bw_enc_h264",
			&pdata->mfc_bw_info.bw_enc_h264.peak, 3);
	of_property_read_u32_array(np, "bw_enc_hevc",
			&pdata->mfc_bw_info.bw_enc_hevc.peak, 3);
	of_property_read_u32_array(np, "bw_enc_hevc_10bit",
			&pdata->mfc_bw_info.bw_enc_hevc_10bit.peak, 3);
	of_property_read_u32_array(np, "bw_enc_vp8",
			&pdata->mfc_bw_info.bw_enc_vp8.peak, 3);
	of_property_read_u32_array(np, "bw_enc_vp9",
			&pdata->mfc_bw_info.bw_enc_vp9.peak, 3);
	of_property_read_u32_array(np, "bw_enc_vp9_10bit",
			&pdata->mfc_bw_info.bw_enc_vp9_10bit.peak, 3);
	of_property_read_u32_array(np, "bw_enc_mpeg4",
			&pdata->mfc_bw_info.bw_enc_mpeg4.peak, 3);
	of_property_read_u32_array(np, "bw_dec_h264",
			&pdata->mfc_bw_info.bw_dec_h264.peak, 3);
	of_property_read_u32_array(np, "bw_dec_hevc",
			&pdata->mfc_bw_info.bw_dec_hevc.peak, 3);
	of_property_read_u32_array(np, "bw_dec_hevc_10bit",
			&pdata->mfc_bw_info.bw_dec_hevc_10bit.peak, 3);
	of_property_read_u32_array(np, "bw_dec_vp8",
			&pdata->mfc_bw_info.bw_dec_vp8.peak, 3);
	of_property_read_u32_array(np, "bw_dec_vp9",
			&pdata->mfc_bw_info.bw_dec_vp9.peak, 3);
	of_property_read_u32_array(np, "bw_dec_vp9_10bit",
			&pdata->mfc_bw_info.bw_dec_vp9_10bit.peak, 3);
	of_property_read_u32_array(np, "bw_dec_av1",
			&pdata->mfc_bw_info.bw_dec_av1.peak, 3);
	of_property_read_u32_array(np, "bw_dec_av1_10bit",
			&pdata->mfc_bw_info.bw_dec_av1_10bit.peak, 3);
	of_property_read_u32_array(np, "bw_dec_mpeg4",
			&pdata->mfc_bw_info.bw_dec_mpeg4.peak, 3);

	if (pdata->support_sbwc) {
		of_property_read_u32_array(np, "sbwc_bw_enc_h264",
			&pdata->mfc_bw_info_sbwc.bw_enc_h264.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_enc_hevc",
			&pdata->mfc_bw_info_sbwc.bw_enc_hevc.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_enc_hevc_10bit",
			&pdata->mfc_bw_info_sbwc.bw_enc_hevc_10bit.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_enc_vp8",
			&pdata->mfc_bw_info_sbwc.bw_enc_vp8.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_enc_vp9",
			&pdata->mfc_bw_info_sbwc.bw_enc_vp9.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_enc_vp9_10bit",
			&pdata->mfc_bw_info_sbwc.bw_enc_vp9_10bit.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_enc_mpeg4",
			&pdata->mfc_bw_info_sbwc.bw_enc_mpeg4.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_dec_h264",
			&pdata->mfc_bw_info_sbwc.bw_dec_h264.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_dec_hevc",
			&pdata->mfc_bw_info_sbwc.bw_dec_hevc.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_dec_hevc_10bit",
			&pdata->mfc_bw_info_sbwc.bw_dec_hevc_10bit.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_dec_vp8",
			&pdata->mfc_bw_info_sbwc.bw_dec_vp8.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_dec_vp9",
			&pdata->mfc_bw_info_sbwc.bw_dec_vp9.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_dec_vp9_10bit",
			&pdata->mfc_bw_info_sbwc.bw_dec_vp9_10bit.peak, 3);
		of_property_read_u32_array(np, "sbwc_bw_dec_mpeg4",
			&pdata->mfc_bw_info_sbwc.bw_dec_mpeg4.peak, 3);

		of_property_read_u32_array(np, "dpb_sbwc_bw_enc_h264",
			&pdata->mfc_bw_info_dpb_sbwc.bw_enc_h264.peak, 3);
		of_property_read_u32_array(np, "dpb_sbwc_bw_enc_hevc",
			&pdata->mfc_bw_info_dpb_sbwc.bw_enc_hevc.peak, 3);
		of_property_read_u32_array(np, "dpb_sbwc_bw_enc_hevc_10bit",
			&pdata->mfc_bw_info_dpb_sbwc.bw_enc_hevc_10bit.peak, 3);
		of_property_read_u32_array(np, "dpb_sbwc_bw_enc_vp9",
			&pdata->mfc_bw_info_dpb_sbwc.bw_enc_vp9.peak, 3);
		of_property_read_u32_array(np, "dpb_sbwc_bw_enc_vp9_10bit",
			&pdata->mfc_bw_info_dpb_sbwc.bw_enc_vp9_10bit.peak, 3);
	}

	/* QoS weight */
	of_property_read_u32(np, "qos_weight_h264_hevc",
			&pdata->qos_weight.weight_h264_hevc);
	of_property_read_u32(np, "qos_weight_vp8_vp9",
			&pdata->qos_weight.weight_vp8_vp9);
	of_property_read_u32(np, "qos_weight_other_codec",
			&pdata->qos_weight.weight_other_codec);
	of_property_read_u32(np, "qos_weight_3plane",
			&pdata->qos_weight.weight_3plane);
	of_property_read_u32(np, "qos_weight_10bit",
			&pdata->qos_weight.weight_10bit);
	of_property_read_u32(np, "qos_weight_422",
			&pdata->qos_weight.weight_422);
	of_property_read_u32(np, "qos_weight_bframe",
			&pdata->qos_weight.weight_bframe);
	of_property_read_u32(np, "qos_weight_num_of_ref",
			&pdata->qos_weight.weight_num_of_ref);
	of_property_read_u32(np, "qos_weight_gpb",
			&pdata->qos_weight.weight_gpb);
	of_property_read_u32(np, "qos_weight_num_of_tile",
			&pdata->qos_weight.weight_num_of_tile);
	of_property_read_u32(np, "qos_weight_super64_bframe",
			&pdata->qos_weight.weight_super64_bframe);
	of_property_read_u32(np, "qos_weight_mbaff",
			&pdata->qos_weight.weight_mbaff);

	/* Bitrate control for QoS */
	of_property_read_u32(np, "num_mfc_freq", &pdata->num_mfc_freq);
	if (pdata->num_mfc_freq)
		of_property_read_u32_array(np, "mfc_freqs",
				pdata->mfc_freqs, pdata->num_mfc_freq);
	of_property_read_u32_array(np, "max_Kbps", pdata->max_Kbps,
			MAX_NUM_MFC_BPS);
	__mfc_create_bitrate_table(mfc);

	/* Core balance(%) for resource managing */
	of_property_read_u32(np, "core_balance", &pdata->core_balance);

	/* MFC IOVA threshold */
	of_property_read_u32(np, "iova_threshold", &pdata->iova_threshold);

	/* MFC IOVA threshold */
	of_property_read_u32(np, "idle_clk_ctrl", &pdata->idle_clk_ctrl);

	/* Encoder RGB CSC formula by VUI from F/W */
	of_property_read_u32(np, "enc_rgb_csc_by_fw", &pdata->enc_rgb_csc_by_fw);

	return 0;
}

static void *__mfc_get_drv_data(struct platform_device *pdev);

static struct video_device *__mfc_video_device_register(struct mfc_dev *dev,
				char *name, int node_num)
{
	struct video_device *vfd;
	int ret = 0;

	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&dev->v4l2_dev, "Failed to allocate video device\n");
		return NULL;
	}
	strncpy(vfd->name, name, sizeof(vfd->name) - 1);
	vfd->fops = &mfc_fops;
	vfd->minor = -1;
	vfd->release = video_device_release;

	if (IS_DEC_NODE(node_num))
		vfd->ioctl_ops = mfc_get_dec_v4l2_ioctl_ops();
	else if(IS_ENC_NODE(node_num))
		vfd->ioctl_ops = mfc_get_enc_v4l2_ioctl_ops();

	vfd->lock = &dev->mfc_mutex;
	vfd->v4l2_dev = &dev->v4l2_dev;
	vfd->vfl_dir = VFL_DIR_M2M;
	set_bit(V4L2_FL_QUIRK_INVERTED_CROP, &vfd->flags);
	vfd->device_caps = V4L2_CAP_VIDEO_CAPTURE
			| V4L2_CAP_VIDEO_OUTPUT
			| V4L2_CAP_VIDEO_CAPTURE_MPLANE
			| V4L2_CAP_VIDEO_OUTPUT_MPLANE
			| V4L2_CAP_STREAMING;

	snprintf(vfd->name, sizeof(vfd->name), "%s", vfd->name);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, node_num);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device /dev/video%d\n", node_num);
		video_device_release(vfd);
		return NULL;
	}
	v4l2_info(&dev->v4l2_dev, "video device registered as /dev/video%d\n",
								vfd->num);
	video_set_drvdata(vfd, dev);

	return vfd;
}

/* MFC probe function */
static int mfc_probe(struct platform_device *pdev)
{
	struct device *device = &pdev->dev;
	struct device_node *np = device->of_node;
	struct mfc_dev *dev;
	int ret = -ENOENT;

	dev_info(&pdev->dev, "%s is called\n", __func__);

	dev = devm_kzalloc(&pdev->dev, sizeof(struct mfc_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Not enough memory for MFC device\n");
		return -ENOMEM;
	}

	/* empty device for CPU cache flush with dma_sync_* API */
	dev->cache_op_dev = devm_kzalloc(&pdev->dev, sizeof(struct device), GFP_KERNEL);
	device_initialize(dev->cache_op_dev);
	dma_coerce_mask_and_coherent(dev->cache_op_dev, DMA_BIT_MASK(36));

	dev->device = &pdev->dev;
	dev->variant = __mfc_get_drv_data(pdev);
	platform_set_drvdata(pdev, dev);
	mfc_dev_init_memlog(pdev);

	dev->pdata = devm_kzalloc(&pdev->dev, sizeof(struct mfc_platdata), GFP_KERNEL);
	if (!dev->pdata) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_res_mem;
	}

	ret = __mfc_parse_dt(dev->device->of_node, dev);
	if (ret)
		goto err_res_mem;

	atomic_set(&dev->trace_ref, 0);
	atomic_set(&dev->trace_ref_longterm, 0);
	dev->mfc_trace = g_mfc_trace;
	dev->mfc_trace_rm = g_mfc_trace_rm;
	dev->mfc_trace_longterm = g_mfc_trace_longterm;

	dma_set_mask(&pdev->dev, DMA_BIT_MASK(36));

	mutex_init(&dev->mfc_mutex);
	mutex_init(&dev->mfc_migrate_mutex);
	mutex_init(&dev->sysmmu_fault_mutex);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto err_v4l2_dev;

	/* decoder */
	dev->vfd_dec = __mfc_video_device_register(dev, MFC_DEC_NAME,
			EXYNOS_VIDEONODE_MFC_DEC);
	if (!dev->vfd_dec) {
		ret = -ENOMEM;
		goto alloc_vdev_dec;
	}

	/* encoder */
	dev->vfd_enc = __mfc_video_device_register(dev, MFC_ENC_NAME,
			EXYNOS_VIDEONODE_MFC_ENC);
	if (!dev->vfd_enc) {
		ret = -ENOMEM;
		goto alloc_vdev_enc;
	}

	/* secure decoder */
	dev->vfd_dec_drm = __mfc_video_device_register(dev, MFC_DEC_DRM_NAME,
			EXYNOS_VIDEONODE_MFC_DEC_DRM);
	if (!dev->vfd_dec_drm) {
		ret = -ENOMEM;
		goto alloc_vdev_dec_drm;
	}

	/* secure encoder */
	dev->vfd_enc_drm = __mfc_video_device_register(dev, MFC_ENC_DRM_NAME,
			EXYNOS_VIDEONODE_MFC_ENC_DRM);
	if (!dev->vfd_enc_drm) {
		ret = -ENOMEM;
		goto alloc_vdev_enc_drm;
	}

	/* OTF encoder */
	dev->vfd_enc_otf = __mfc_video_device_register(dev, MFC_ENC_OTF_NAME,
			EXYNOS_VIDEONODE_MFC_ENC_OTF);
	if (!dev->vfd_enc_otf) {
		ret = -ENOMEM;
		goto alloc_vdev_enc_otf;
	}

	/* OTF secure encoder */
	dev->vfd_enc_otf_drm = __mfc_video_device_register(dev, MFC_ENC_OTF_DRM_NAME,
			EXYNOS_VIDEONODE_MFC_ENC_OTF_DRM);
	if (!dev->vfd_enc_otf_drm) {
		ret = -ENOMEM;
		goto alloc_vdev_enc_otf_drm;
	}
	/* end of node setting*/

	/* instance migration worker */
	dev->migration_wq = alloc_workqueue("mfc/inst_migration", WQ_UNBOUND
					| WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	if (dev->migration_wq == NULL) {
		dev_err(&pdev->dev, "failed to create workqueue for migration wq\n");
		goto err_migration_work;
	}
	INIT_WORK(&dev->migration_work, mfc_rm_migration_worker);

	/* main butler worker */
	dev->butler_wq = alloc_workqueue("mfc/butler", WQ_UNBOUND
					| WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	if (dev->butler_wq == NULL) {
		dev_err(&pdev->dev, "failed to create workqueue for butler\n");
		goto err_butler_wq;
	}
	INIT_WORK(&dev->butler_work, mfc_butler_worker);

	/* dump information call-back function */
	dev->dump_ops = &mfc_dump_ops;

	g_mfc_dev = dev;

	mfc_init_debugfs(dev);
	__platform_driver_register(&mfc_core_driver, THIS_MODULE);
	of_platform_populate(np, NULL, NULL, device);

	dev_info(&pdev->dev, "%s is completed\n", __func__);

	return 0;

/* Deinit MFC if probe had failed */
err_butler_wq:
	destroy_workqueue(dev->migration_wq);
err_migration_work:
	video_unregister_device(dev->vfd_enc_otf_drm);
alloc_vdev_enc_otf_drm:
	video_unregister_device(dev->vfd_enc_otf);
alloc_vdev_enc_otf:
	video_unregister_device(dev->vfd_enc_drm);
alloc_vdev_enc_drm:
	video_unregister_device(dev->vfd_dec_drm);
alloc_vdev_dec_drm:
	video_unregister_device(dev->vfd_enc);
alloc_vdev_enc:
	video_unregister_device(dev->vfd_dec);
alloc_vdev_dec:
	v4l2_device_unregister(&dev->v4l2_dev);
err_v4l2_dev:
	mutex_destroy(&dev->mfc_mutex);
	mutex_destroy(&dev->mfc_migrate_mutex);
	mutex_destroy(&dev->sysmmu_fault_mutex);
err_res_mem:
	mfc_dev_deinit_memlog(dev);
	return ret;
}

/* Remove the driver */
static int mfc_remove(struct platform_device *pdev)
{
	struct mfc_dev *dev = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s++\n", __func__);
	v4l2_info(&dev->v4l2_dev, "Removing %s\n", pdev->name);
	flush_workqueue(dev->butler_wq);
	destroy_workqueue(dev->butler_wq);
	flush_workqueue(dev->migration_wq);
	destroy_workqueue(dev->migration_wq);
	mfc_deinit_debugfs(dev);
	video_unregister_device(dev->vfd_enc);
	video_unregister_device(dev->vfd_dec);
	video_unregister_device(dev->vfd_dec_drm);
	video_unregister_device(dev->vfd_enc_drm);
	video_unregister_device(dev->vfd_enc_otf);
	video_unregister_device(dev->vfd_enc_otf_drm);
	v4l2_device_unregister(&dev->v4l2_dev);
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	remove_proc_entry(MFC_PROC_FW_STATUS, mfc_proc_entry);
	remove_proc_entry(MFC_PROC_DRM_INSTANCE_NUMBER, mfc_proc_entry);
	remove_proc_entry(MFC_PROC_INSTANCE_NUMBER, mfc_proc_entry);
	remove_proc_entry(MFC_PROC_ROOT, NULL);
#endif
	mfc_dev_deinit_memlog(dev);
	mfc_dev_debug(2, "Will now deinit HW\n");

	dev_dbg(&pdev->dev, "%s--\n", __func__);
	return 0;
}

static void mfc_shutdown(struct platform_device *pdev)
{
	struct platform_driver *pcoredrv = &mfc_core_driver;
	struct mfc_dev *dev = platform_get_drvdata(pdev);
	struct mfc_core *core;
	int i;

	for (i = 0; i < dev->num_core; i++) {
		core = dev->core[i];
		if (!core) {
			mfc_dev_debug(2, "There is no core[%d]\n", i);
			continue;
		}

		if (!core->shutdown) {
			mfc_core_info("%s core shutdown was not performed\n", core->name);
			pcoredrv->shutdown(to_platform_device(core->device));
		}
	}

	mfc_dev_info("MFC shutdown is completed\n");
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int mfc_suspend(struct device *device)
{
	struct mfc_dev *dev = platform_get_drvdata(to_platform_device(device));

	if (!dev) {
		dev_err(device, "no mfc device to run\n");
		return -EINVAL;
	}

	mfc_dev_debug(2, "MFC suspend will be handled by core driver\n");

	return 0;
}

static int mfc_resume(struct device *device)
{
	struct mfc_dev *dev = platform_get_drvdata(to_platform_device(device));

	if (!dev) {
		dev_err(device, "no mfc device to run\n");
		return -EINVAL;
	}

	mfc_dev_debug(2, "MFC resume will be handle by core driver\n");

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_PM)
static int mfc_runtime_suspend(struct device *device)
{
	struct mfc_dev *dev = platform_get_drvdata(to_platform_device(device));

	mfc_dev_debug(3, "mfc runtime suspend\n");

	return 0;
}

static int mfc_runtime_idle(struct device *dev)
{
	return 0;
}

static int mfc_runtime_resume(struct device *device)
{
	struct mfc_dev *dev = platform_get_drvdata(to_platform_device(device));

	mfc_dev_debug(3, "mfc runtime resume\n");

	return 0;
}
#endif

/* Power management */
static const struct dev_pm_ops mfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mfc_suspend, mfc_resume)
	SET_RUNTIME_PM_OPS(
			mfc_runtime_suspend,
			mfc_runtime_resume,
			mfc_runtime_idle
	)
};

struct mfc_ctx_buf_size mfc_ctx_buf_size = {
	.dev_ctx	= PAGE_ALIGN(0x7800),	/*  30KB */
	.h264_dec_ctx	= PAGE_ALIGN(0x200000),	/* 1.6MB */
	.other_dec_ctx	= PAGE_ALIGN(0xF000),	/*  60KB */
	.h264_enc_ctx	= PAGE_ALIGN(0x19000),	/* 100KB */
	.hevc_enc_ctx	= PAGE_ALIGN(0xC800),	/*  50KB */
	.other_enc_ctx	= PAGE_ALIGN(0xC800),	/*  50KB */
	.dbg_info_buf	= PAGE_ALIGN(0x1000),	/* 4KB for DEBUG INFO */
};

struct mfc_buf_size mfc_buf_size = {
	.firmware_code	= PAGE_ALIGN(0x100000),	/* 1MB */
	.cpb_buf	= PAGE_ALIGN(0x300000),	/* 3MB */
	.ctx_buf	= &mfc_ctx_buf_size,
};

static struct mfc_variant mfc_drvdata = {
	.buf_size = &mfc_buf_size,
	.num_entities = 2,
};

static const struct of_device_id exynos_mfc_match[] = {
	{
		.compatible = "samsung,exynos-mfc",
		.data = &mfc_drvdata,
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_mfc_match);

static void *__mfc_get_drv_data(struct platform_device *pdev)
{
	struct mfc_variant *driver_data = NULL;

	if (pdev->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_node(of_match_ptr(exynos_mfc_match),
				pdev->dev.of_node);
		if (match)
			driver_data = (struct mfc_variant *)match->data;
	} else {
		driver_data = (struct mfc_variant *)
			platform_get_device_id(pdev)->driver_data;
	}
	return driver_data;
}

static struct platform_driver mfc_driver = {
	.probe		= mfc_probe,
	.remove		= mfc_remove,
	.shutdown	= mfc_shutdown,
	.driver	= {
		.name	= MFC_NAME,
		.owner	= THIS_MODULE,
		.pm	= &mfc_pm_ops,
		.of_match_table = exynos_mfc_match,
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(mfc_driver);
MODULE_SOFTDEP("pre: samsung_iommu");
MODULE_SOFTDEP("pre: samsung_dma_heap");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kamil Debski <k.debski@samsung.com>");
MODULE_IMPORT_NS(DMA_BUF);
