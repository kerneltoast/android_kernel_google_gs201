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
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/soc/samsung/exynos-smc.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/iommu.h>
#include <linux/dma-iommu.h>

#include "mfc_common.h"

#include "mfc_isr.h"
#include "mfc_dec_v4l2.h"
#include "mfc_enc_v4l2.h"

#include "mfc_run.h"
#include "mfc_hwlock.h"
#include "mfc_nal_q.h"
#include "mfc_otf.h"
#include "mfc_watchdog.h"
#include "mfc_debugfs.h"
#include "mfc_sync.h"

#include "mfc_pm.h"
#include "mfc_perf_measure.h"
#include "mfc_reg_api.h"
#include "mfc_hw_reg_api.h"
#include "mfc_mmcache.h"
#include "mfc_llc.h"
#include "mfc_slc.h"

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
struct _mfc_trace g_mfc_trace_longterm[MFC_TRACE_COUNT_MAX];
struct _mfc_trace_logging g_mfc_trace_logging[MFC_TRACE_LOG_COUNT_MAX];
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

void mfc_butler_worker(struct work_struct *work)
{
	struct mfc_dev *dev;

	dev = container_of(work, struct mfc_dev, butler_work);

	mfc_try_run(dev);
}

extern struct mfc_ctrls_ops decoder_ctrls_ops;
extern struct vb2_ops mfc_dec_qops;
extern struct mfc_fmt dec_formats[];

static void __mfc_deinit_dec_ctx(struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;

	mfc_cleanup_iovmm(ctx);

	mfc_delete_queue(&ctx->src_buf_queue);
	mfc_delete_queue(&ctx->dst_buf_queue);
	mfc_delete_queue(&ctx->src_buf_nal_queue);
	mfc_delete_queue(&ctx->dst_buf_nal_queue);

	mfc_mem_cleanup_user_shared_handle(ctx, &dec->sh_handle_hdr);
	if (dec->hdr10_plus_info)
		vfree(dec->hdr10_plus_info);
	kfree(dec);
}

static int __mfc_init_dec_ctx(struct mfc_ctx *ctx)
{
	struct mfc_dec *dec;
	int ret = 0;
	int i;

	dec = kzalloc(sizeof(struct mfc_dec), GFP_KERNEL);
	if (!dec)
		return -ENOMEM;

	ctx->dec_priv = dec;

	ctx->inst_no = MFC_NO_INSTANCE_SET;

	mfc_create_queue(&ctx->src_buf_queue);
	mfc_create_queue(&ctx->dst_buf_queue);
	mfc_create_queue(&ctx->src_buf_nal_queue);
	mfc_create_queue(&ctx->dst_buf_nal_queue);

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
	INIT_LIST_HEAD(&ctx->qos_list);
	INIT_LIST_HEAD(&ctx->bitrate_list);
	INIT_LIST_HEAD(&ctx->ts_list);

	dec->display_delay = -1;
	dec->is_interlaced = 0;
	dec->immediate_display = 0;
	dec->is_dts_mode = 0;
	dec->inter_res_change = 0;
	dec->disp_res_change = 0;

	dec->is_dynamic_dpb = 1;
	dec->dynamic_used = 0;
	dec->is_dpb_full = 0;
	dec->queued_dpb = 0;
	dec->display_index = -1;
	dec->dpb_table_used = 0;
	mutex_init(&dec->dpb_mutex);

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

	mfc_delete_queue(&ctx->src_buf_queue);
	mfc_delete_queue(&ctx->dst_buf_queue);
	mfc_delete_queue(&ctx->src_buf_nal_queue);
	mfc_delete_queue(&ctx->dst_buf_nal_queue);
	mfc_delete_queue(&ctx->ref_buf_queue);

	mfc_mem_cleanup_user_shared_handle(ctx, &enc->sh_handle_svc);
	mfc_mem_cleanup_user_shared_handle(ctx, &enc->sh_handle_roi);
	mfc_mem_cleanup_user_shared_handle(ctx, &enc->sh_handle_hdr);
	mfc_release_enc_roi_buffer(ctx);
	kfree(enc);
}

static int __mfc_init_enc_ctx(struct mfc_ctx *ctx)
{
	struct mfc_enc *enc;
	struct mfc_enc_params *p;
	int ret = 0;
	int i;

	enc = kzalloc(sizeof(struct mfc_enc), GFP_KERNEL);
	if (!enc)
		return -ENOMEM;

	ctx->enc_priv = enc;

	ctx->inst_no = MFC_NO_INSTANCE_SET;

	mfc_create_queue(&ctx->src_buf_queue);
	mfc_create_queue(&ctx->dst_buf_queue);
	mfc_create_queue(&ctx->src_buf_nal_queue);
	mfc_create_queue(&ctx->dst_buf_nal_queue);
	mfc_create_queue(&ctx->ref_buf_queue);

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

	INIT_LIST_HEAD(&ctx->qos_list);
	INIT_LIST_HEAD(&ctx->ts_list);

	enc->sh_handle_svc.fd = -1;
	enc->sh_handle_roi.fd = -1;
	enc->sh_handle_hdr.fd = -1;

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

static int __mfc_init_instance(struct mfc_dev *dev, struct mfc_ctx *ctx)
{
	int ret = 0;

	/* set watchdog timer */
	mod_timer(&dev->watchdog_timer, jiffies + msecs_to_jiffies(WATCHDOG_TICK_INTERVAL));

	/* set MFC idle timer */
	atomic_set(&dev->hw_run_cnt, 0);
	mfc_change_idle_mode(dev, MFC_IDLE_MODE_NONE);

	/* Load the FW */
	if (!dev->fw.status) {
		ret = mfc_alloc_firmware(dev);
		if (ret)
			goto err_fw_alloc;
		dev->fw.status = 1;
	}

	ret = mfc_load_firmware(dev);
	if (ret)
		goto err_fw_load;

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	trace_mfc_dcpp_start(ctx->num, 1, dev->fw.drm_status);
	if (!dev->drm_fw_buf.daddr) {
		mfc_ctx_err("DRM F/W buffer is not allocated\n");
		dev->fw.drm_status = 0;
	} else {
		/* Request buffer protection for DRM F/W */
		ret = exynos_smc(SMC_DRM_PPMP_MFCFW_PROT,
				dev->drm_fw_buf.daddr, 0, 0);
		if (ret != DRMDRV_OK) {
			mfc_ctx_err("failed MFC DRM F/W prot(%#x)\n", ret);
			call_dop(dev, dump_and_stop_debug_mode, dev);
			dev->fw.drm_status = 0;
		} else {
			dev->fw.drm_status = 1;
		}
	}
#endif
	trace_mfc_dcpp_end(ctx->num, 1, dev->fw.drm_status);

	mfc_alloc_common_context(dev);

	if (dbg_enable)
		mfc_alloc_dbg_info_buffer(dev);

	ret = mfc_get_hwlock_dev(dev);
	if (ret < 0) {
		mfc_ctx_err("Failed to get hwlock\n");
		mfc_ctx_err("dev.hwlock.dev = 0x%lx, bits = 0x%lx, owned_by_irq = %d, wl_count = %d, transfer_owner = %d\n",
				dev->hwlock.dev, dev->hwlock.bits, dev->hwlock.owned_by_irq,
				dev->hwlock.wl_count, dev->hwlock.transfer_owner);
		goto err_hw_lock;
	}

	mfc_debug(2, "power on\n");
	ret = mfc_pm_power_on(dev);
	if (ret < 0) {
		mfc_ctx_err("power on failed\n");
		goto err_pwr_enable;
	}

	dev->curr_ctx = ctx->num;
	dev->preempt_ctx = MFC_NO_INSTANCE_SET;
	dev->curr_ctx_is_drm = ctx->is_drm;

	ret = mfc_run_init_hw(dev);
	if (ret) {
		mfc_ctx_err("Failed to init mfc h/w\n");
		goto err_hw_init;
	}

	if (dev->has_mmcache && (dev->mmcache.is_on_status == 0))
		mfc_mmcache_enable(dev);

	if (dev->has_llc && (dev->llc_on_status == 0))
		mfc_llc_enable(dev);

	if (dev->has_slc && (dev->slc_on_status == 0))
		mfc_slc_enable(dev);

	mfc_release_hwlock_dev(dev);

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->nal_q)) {
		dev->nal_q_handle = mfc_nal_q_create(dev);
		if (dev->nal_q_handle == NULL)
			mfc_ctx_err("[NALQ] Can't create nal q\n");
	}

	return ret;

err_hw_init:
	mfc_pm_power_off(dev);

err_pwr_enable:
	mfc_release_hwlock_dev(dev);

err_hw_lock:
	mfc_release_common_context(dev);

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (dev->fw.drm_status) {
		int smc_ret = 0;
		dev->fw.drm_status = 0;
		/* Request buffer unprotection for DRM F/W */
		smc_ret = exynos_smc(SMC_DRM_PPMP_MFCFW_UNPROT,
					dev->drm_fw_buf.daddr, 0, 0);
		if (smc_ret != DRMDRV_OK) {
			mfc_ctx_err("failed MFC DRM F/W unprot(%#x)\n",
						smc_ret);
			call_dop(dev, dump_and_stop_debug_mode, dev);
		}
	}
#endif

err_fw_load:
err_fw_alloc:
	del_timer(&dev->watchdog_timer);
	del_timer(&dev->mfc_idle_timer);

	mfc_ctx_err("failed to init first instance\n");
	return ret;
}

/* Open an MFC node */
static int mfc_open(struct file *file)
{
	struct mfc_ctx *ctx = NULL;
	struct mfc_dev *dev = video_drvdata(file);
	int ret = 0;
	enum mfc_node_type node;
	struct video_device *vdev = NULL;

	if (!dev) {
		mfc_ctx_err("no mfc device to run\n");
		goto err_no_device;
	}

	mfc_dev_debug(2, "mfc driver open called\n");

	if (mutex_lock_interruptible(&dev->mfc_mutex))
		return -ERESTARTSYS;

	node = mfc_get_node_type(file);
	if (node == MFCNODE_INVALID) {
		mfc_ctx_err("cannot specify node type\n");
		ret = -ENOENT;
		goto err_node_type;
	}

	dev->num_inst++;	/* It is guarded by mfc_mutex in vfd */

	/* Allocate memory for context */
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
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

	init_waitqueue_head(&ctx->cmd_wq);
	mfc_init_listable_wq_ctx(ctx);
	spin_lock_init(&ctx->buf_queue_lock);

	if (mfc_is_decoder_node(node))
		ret = __mfc_init_dec_ctx(ctx);
	else
		ret = __mfc_init_enc_ctx(ctx);
	if (ret)
		goto err_ctx_init;

	/* regression test val */
	if (regression_option)
		ctx->regression_val = vmalloc(SZ_1M);

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
	mfc_clear_bit(ctx->num, &dev->work_bits);
	dev->ctx[ctx->num] = ctx;

	/* Load firmware if this is the first instance */
	if (dev->num_inst == 1) {
		ret = __mfc_init_instance(dev, ctx);
		if (ret)
			goto err_init_inst;

		if (perf_boost_mode)
			mfc_perf_boost_enable(dev);
	}

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
	if (mfc_is_encoder_otf_node(node)) {
		ret = mfc_otf_create(ctx);
		if (ret)
			mfc_ctx_err("[OTF] otf_create failed\n");
	}
#endif

	mfc_perf_init(dev);
	trace_mfc_node_open(ctx->num, dev->num_inst, ctx->type, ctx->is_drm);
	mfc_ctx_info("MFC open completed [%d:%d] version = %d\n",
			dev->num_drm_inst, dev->num_inst, MFC_DRIVER_INFO);
	MFC_TRACE_CTX_LT("[INFO] %s %s opened (ctx:%d, total:%d)\n", ctx->is_drm ? "DRM" : "Normal",
			mfc_is_decoder_node(node) ? "DEC" : "ENC", ctx->num, dev->num_inst);
	mutex_unlock(&dev->mfc_mutex);
	return ret;

	/* Deinit when failure occured */
err_init_inst:
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (ctx->is_drm)
		dev->num_drm_inst--;

err_drm_start:
#endif
	call_cop(ctx, cleanup_ctx_ctrls, ctx);

err_ctx_ctrls:
	vfree(ctx->regression_val);

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

static int __mfc_wait_close_inst(struct mfc_dev *dev, struct mfc_ctx *ctx)
{
	int ret;

	if (atomic_read(&dev->watchdog_run)) {
		mfc_ctx_err("watchdog already running!\n");
		return 0;
	}

	if (ctx->inst_no == MFC_NO_INSTANCE_SET) {
		mfc_debug(2, "mfc no instance already\n");
		return 0;
	}

	mfc_clean_ctx_int_flags(ctx);
	mfc_change_state(ctx, MFCINST_RETURN_INST);
	mfc_set_bit(ctx->num, &dev->work_bits);

	/* To issue the command 'CLOSE_INSTANCE' */
	if (mfc_just_run(dev, ctx->num)) {
		mfc_ctx_err("failed to run MFC, state: %d\n", ctx->state);
		MFC_TRACE_CTX_LT("[ERR][Release] failed to run MFC, state: %d\n", ctx->state);
		return -EIO;
	}

	/* Wait until instance is returned or timeout occured */
	ret = mfc_wait_for_done_ctx(ctx, MFC_REG_R2H_CMD_CLOSE_INSTANCE_RET);
	if (ret == 1) {
		mfc_ctx_err("failed to wait CLOSE_INSTANCE(timeout)\n");

		if (mfc_wait_for_done_ctx(ctx,
					MFC_REG_R2H_CMD_CLOSE_INSTANCE_RET)) {
			mfc_ctx_err("waited once more but failed to wait CLOSE_INSTANCE\n");
			dev->logging_data->cause |= (1 << MFC_CAUSE_FAIL_CLOSE_INST);
			call_dop(dev, dump_and_stop_always, dev);
		}
	} else if (ret == -1) {
		mfc_ctx_err("failed to wait CLOSE_INSTANCE(err)\n");
		call_dop(dev, dump_and_stop_debug_mode, dev);
	}

	ctx->inst_no = MFC_NO_INSTANCE_SET;

	return 0;
}

/* Release MFC context */
static int mfc_release(struct file *file)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	struct mfc_dev *dev = ctx->dev;
	int ret = 0;

	mutex_lock(&dev->mfc_mutex);

	mfc_ctx_info("MFC driver release is called [%d:%d], is_drm(%d)\n",
			dev->num_drm_inst, dev->num_inst, ctx->is_drm);

	MFC_TRACE_CTX_LT("[INFO] release is called (ctx:%d, total:%d)\n", ctx->num, dev->num_inst);

	mfc_clear_bit(ctx->num, &dev->work_bits);

	/* If a H/W operation is in progress, wait for it complete */
	if (need_to_wait_nal_abort(ctx)) {
		if (mfc_wait_for_done_ctx(ctx, MFC_REG_R2H_CMD_NAL_ABORT_RET)) {
			mfc_ctx_err("Failed to wait nal abort\n");
			mfc_cleanup_work_bit_and_try_run(ctx);
		}
	}

	ret = mfc_get_hwlock_ctx(ctx);
	if (ret < 0) {
		mfc_ctx_err("Failed to get hwlock\n");
		MFC_TRACE_CTX_LT("[ERR][Release] failed to get hwlock (shutdown: %d)\n", dev->shutdown);
		mutex_unlock(&dev->mfc_mutex);
		return -EBUSY;
	}

	if (call_cop(ctx, cleanup_ctx_ctrls, ctx) < 0)
		mfc_ctx_err("failed in cleanup_ctx_ctrl\n");

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	/* Mark context as idle */
	mfc_clear_bit(ctx->num, &dev->work_bits);

	/* If instance was initialised then
	 * return instance and free reosurces */
	ret = __mfc_wait_close_inst(dev, ctx);
	if (ret)
		goto err_release_try;

	if (ctx->is_drm)
		dev->num_drm_inst--;
	dev->num_inst--;

	if (dev->num_inst == 0) {
		mfc_run_deinit_hw(dev);

		if (perf_boost_mode)
			mfc_perf_boost_disable(dev);

		del_timer(&dev->watchdog_timer);
		del_timer(&dev->mfc_idle_timer);

		flush_workqueue(dev->butler_wq);

		mfc_debug(2, "power off\n");
		mfc_pm_power_off(dev);

		if (dbg_enable)
			mfc_release_dbg_info_buffer(dev);

		mfc_release_common_context(dev);

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
		if (dev->fw.drm_status) {
			dev->fw.drm_status = 0;
			/* Request buffer unprotection for DRM F/W */
			ret = exynos_smc(SMC_DRM_PPMP_MFCFW_UNPROT,
					dev->drm_fw_buf.daddr, 0, 0);
			if (ret != DRMDRV_OK) {
				mfc_ctx_err("failed MFC DRM F/W unprot(%#x)\n",
							ret);
				call_dop(dev, dump_and_stop_debug_mode, dev);
			}
		}
#endif

		if (dev->nal_q_handle)
			mfc_nal_q_destroy(dev, dev->nal_q_handle);
	}

	mfc_qos_off(ctx);

	if (dev->has_mmcache && dev->mmcache.is_on_status) {
		mfc_invalidate_mmcache(dev);

		if (dev->num_inst == 0)
			mfc_mmcache_disable(dev);
	}

	if (dev->has_llc && dev->llc_on_status) {
		mfc_llc_flush(dev);

		if (dev->num_inst == 0)
			mfc_llc_disable(dev);
	}

	if (dev->has_slc && dev->slc_on_status)
		mfc_slc_disable(dev);

	mfc_release_codec_buffers(ctx);
	mfc_release_instance_context(ctx);

	mfc_release_hwlock_ctx(ctx);

	/* Free resources */
	vb2_queue_release(&ctx->vq_src);
	vb2_queue_release(&ctx->vq_dst);

	if (ctx->regression_val)
		vfree(ctx->regression_val);
	if (ctx->type == MFCINST_DECODER)
		__mfc_deinit_dec_ctx(ctx);
	else if (ctx->type == MFCINST_ENCODER)
		__mfc_deinit_enc_ctx(ctx);

#if IS_ENABLED(CONFIG_VIDEO_EXYNOS_REPEATER)
	if (ctx->otf_handle) {
		mfc_otf_deinit(ctx);
		mfc_otf_destroy(ctx);
	}
#endif

	mfc_destroy_listable_wq_ctx(ctx);

	trace_mfc_node_close(ctx->num, dev->num_inst, ctx->type, ctx->is_drm);

	MFC_TRACE_CTX_LT("[INFO] Release finished (ctx:%d, total:%d)\n", ctx->num, dev->num_inst);

	dev->ctx[ctx->num] = 0;
	kfree(ctx);

	mfc_perf_print();

	mfc_dev_info("mfc driver release finished [%d:%d]\n",
			dev->num_drm_inst, dev->num_inst);

	if (mfc_is_work_to_do(dev))
		queue_work(dev->butler_wq, &dev->butler_work);

	mutex_unlock(&dev->mfc_mutex);
	return ret;

err_release_try:
	mfc_release_hwlock_ctx(ctx);
	mfc_cleanup_work_bit_and_try_run(ctx);
	mutex_unlock(&dev->mfc_mutex);
	return ret;
}

/* Poll */
static unsigned int mfc_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct mfc_ctx *ctx = fh_to_mfc_ctx(file->private_data);
	unsigned long req_events = poll_requested_events(wait);
	unsigned int ret = 0;

	mfc_debug_enter();

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

static int __mfc_parse_mfc_qos_platdata(struct device_node *np, char *node_name,
	struct mfc_qos *qosdata, struct mfc_dev *dev)
{
	struct device_node *np_qos;

	np_qos = of_find_node_by_name(np, node_name);
	if (!np_qos) {
		dev_err(dev->device, "%s: could not find mfc_qos_platdata node\n",
			node_name);
		return -EINVAL;
	}

	of_property_read_u32(np_qos, "thrd_mb", &qosdata->threshold_mb);
	of_property_read_u32(np_qos, "freq_mfc", &qosdata->freq_mfc);
	of_property_read_u32(np_qos, "freq_int", &qosdata->freq_int);
	of_property_read_u32(np_qos, "freq_mif", &qosdata->freq_mif);
	of_property_read_u32(np_qos, "mo_value", &qosdata->mo_value);
	of_property_read_u32(np_qos, "mo_10bit_value", &qosdata->mo_10bit_value);
	of_property_read_u32(np_qos, "mo_uhd_enc60_value", &qosdata->mo_uhd_enc60_value);
	of_property_read_u32(np_qos, "time_fw", &qosdata->time_fw);

	of_property_read_string(np_qos, "bts_scen", &qosdata->name);
	if (!qosdata->name) {
		pr_err("[QoS] bts_scen is missing in '%s' node", node_name);
		return -EINVAL;
	}

#ifdef MFC_USE_BTS
#ifndef MFC_NO_RENEWAL_BTS
	qosdata->bts_scen_idx = bts_get_scenindex(qosdata->name);
#endif
#endif

	return 0;
}

int mfc_sysmmu_fault_handler(struct iommu_fault *fault, void *param)
{
	struct mfc_dev *dev = (struct mfc_dev *)param;
	unsigned int trans_info;

	if (dev->pdata->trans_info_offset)
		trans_info = dev->pdata->trans_info_offset;
	else
		trans_info = MFC_MMU_FAULT_TRANS_INFO;

	/* [OTF] If AxID is 1 in SYSMMU1 fault info, it is TS-MUX fault */
	if (dev->has_hwfc && dev->has_2sysmmu) {
		if (MFC_MMU1_READL(MFC_MMU_INTERRUPT_STATUS) &&
				((MFC_MMU1_READL(trans_info) &
				  MFC_MMU_FAULT_TRANS_INFO_AXID_MASK) == 1)) {
			mfc_dev_err("There is TS-MUX page fault. skip SFR dump\n");
			return 0;
		}
	}

	/* If sysmmu is used with other IPs, it should be checked whether it's an MFC fault */
	if (dev->pdata->share_sysmmu) {
		if ((MFC_MMU0_READL(trans_info) & dev->pdata->axid_mask)
				!= dev->pdata->mfc_fault_num) {
			mfc_dev_err("This is not a MFC page fault\n");
			return 0;
		}
	}

	if (MFC_MMU0_READL(MFC_MMU_INTERRUPT_STATUS)) {
		if (MFC_MMU0_READL(trans_info) &
				MFC_MMU_FAULT_TRANS_INFO_RW_MASK)
			dev->logging_data->cause |= (1 << MFC_CAUSE_0WRITE_PAGE_FAULT);
		else
			dev->logging_data->cause |= (1 << MFC_CAUSE_0READ_PAGE_FAULT);
		dev->logging_data->fault_status = MFC_MMU0_READL(MFC_MMU_INTERRUPT_STATUS);
		dev->logging_data->fault_trans_info =
				MFC_MMU0_READL(trans_info);
	}

	if (dev->has_2sysmmu) {
		if (MFC_MMU1_READL(MFC_MMU_INTERRUPT_STATUS)) {
			if (MFC_MMU1_READL(trans_info) &
					MFC_MMU_FAULT_TRANS_INFO_RW_MASK)
				dev->logging_data->cause |= (1 << MFC_CAUSE_1WRITE_PAGE_FAULT);
			else
				dev->logging_data->cause |= (1 << MFC_CAUSE_1READ_PAGE_FAULT);
			dev->logging_data->fault_status = MFC_MMU1_READL(MFC_MMU_INTERRUPT_STATUS);
			dev->logging_data->fault_trans_info =
					MFC_MMU1_READL(trans_info);
		}
	}
	dev->logging_data->fault_addr = (unsigned int)(fault->event.addr);

	call_dop(dev, dump_info, dev);
	s3c2410wdt_set_emergency_reset(3, 0);

	return 0;
}

static void __mfc_create_bitrate_table(struct mfc_dev *dev)
{
	struct mfc_platdata *pdata = dev->pdata;
	int i, interval;

	interval = pdata->max_Kbps[0] / pdata->num_mfc_freq;
	dev->bps_ratio = pdata->max_Kbps[0] / dev->pdata->max_Kbps[1];
	for (i = 0; i < pdata->num_mfc_freq; i++) {
		dev->bitrate_table[i].bps_interval = interval * (i + 1);
		dev->bitrate_table[i].mfc_freq = pdata->mfc_freqs[i];
		mfc_dev_info("[QoS] bitrate table[%d] %dKHz: ~ %dKbps\n",
				i, dev->bitrate_table[i].mfc_freq,
				dev->bitrate_table[i].bps_interval);
	}
}

static int __mfc_parse_dt(struct device_node *np, struct mfc_dev *mfc)
{
	struct mfc_platdata	*pdata = mfc->pdata;
	struct device_node *np_qos;
	char node[50];
	int i;

	if (!np) {
		pr_err("there is no device node\n");
		return -EINVAL;
	}

	/* MFC version */
	of_property_read_u32(np, "ip_ver", &pdata->ip_ver);

	/* Debug mode */
	of_property_read_u32(np, "debug_mode", &pdata->debug_mode);

	/* Sysmmu check */
	of_property_read_u32(np, "share_sysmmu", &pdata->share_sysmmu);
	of_property_read_u32(np, "axid_mask", &pdata->axid_mask);
	of_property_read_u32(np, "mfc_fault_num", &pdata->mfc_fault_num);
	of_property_read_u32(np, "trans_info_offset",
						&pdata->trans_info_offset);

	/* LLC(Last Level Cache) */
	mfc->has_llc = of_property_read_bool(np, "llc");

	/* NAL-Q size */
	of_property_read_u32(np, "nal_q_entry_size", &pdata->nal_q_entry_size);
	of_property_read_u32(np, "nal_q_dump_size", &pdata->nal_q_dump_size);

	/* Features */
	of_property_read_u32_array(np, "nal_q", &pdata->nal_q.support, 2);
	of_property_read_u32_array(np, "skype", &pdata->skype.support, 2);
	of_property_read_u32_array(np, "black_bar", &pdata->black_bar.support, 2);
	of_property_read_u32_array(np, "color_aspect_dec", &pdata->color_aspect_dec.support, 2);
	of_property_read_u32_array(np, "static_info_dec", &pdata->static_info_dec.support, 2);
	of_property_read_u32_array(np, "color_aspect_enc", &pdata->color_aspect_enc.support, 2);
	of_property_read_u32_array(np, "static_info_enc", &pdata->static_info_enc.support, 2);
	of_property_read_u32_array(np, "hdr10_plus", &pdata->hdr10_plus.support, 2);
	of_property_read_u32_array(np, "vp9_stride_align",
			&pdata->vp9_stride_align.support, 2);
	of_property_read_u32_array(np, "drm_switch_predict",
			&pdata->drm_switch_predict.support, 2);
	of_property_read_u32_array(np, "wait_fw_status",
			&pdata->wait_fw_status.support, 2);
	of_property_read_u32_array(np, "wait_nalq_status",
			&pdata->wait_nalq_status.support, 2);
	of_property_read_u32_array(np, "sbwc_enc_src_ctrl",
			&pdata->sbwc_enc_src_ctrl.support, 2);

	/* Ditermine whether to display the broken buffer */
	of_property_read_u32(np, "broken_display", &pdata->broken_display);
	/* Default 10bit format for decoding */
	of_property_read_u32(np, "P010_decoding", &pdata->P010_decoding);

	/* Formats */
	of_property_read_u32(np, "support_10bit", &pdata->support_10bit);
	of_property_read_u32(np, "support_422", &pdata->support_422);
	of_property_read_u32(np, "support_rgb", &pdata->support_rgb);

	/* Compress format */
	of_property_read_u32(np, "support_sbwc", &pdata->support_sbwc);
	of_property_read_u32(np, "support_sbwcl", &pdata->support_sbwcl);
	of_property_read_u32(np, "support_afbc", &pdata->support_afbc);

	/* HDR10+ num max window */
	of_property_read_u32(np, "max_hdr_win", &pdata->max_hdr_win);

	/* Encoder default parameter */
	of_property_read_u32(np, "enc_param_num", &pdata->enc_param_num);
	if (pdata->enc_param_num) {
		of_property_read_u32_array(np, "enc_param_addr",
				pdata->enc_param_addr, pdata->enc_param_num);
		of_property_read_u32_array(np, "enc_param_val",
				pdata->enc_param_val, pdata->enc_param_num);
	}

	of_property_read_u32_array(np, "bw_enc_h264", &pdata->mfc_bw_info.bw_enc_h264.peak, 3);
	of_property_read_u32_array(np, "bw_enc_hevc", &pdata->mfc_bw_info.bw_enc_hevc.peak, 3);
	of_property_read_u32_array(np, "bw_enc_hevc_10bit", &pdata->mfc_bw_info.bw_enc_hevc_10bit.peak, 3);
	of_property_read_u32_array(np, "bw_enc_vp8", &pdata->mfc_bw_info.bw_enc_vp8.peak, 3);
	of_property_read_u32_array(np, "bw_enc_vp9", &pdata->mfc_bw_info.bw_enc_vp9.peak, 3);
	of_property_read_u32_array(np, "bw_enc_vp9_10bit", &pdata->mfc_bw_info.bw_enc_vp9_10bit.peak, 3);
	of_property_read_u32_array(np, "bw_enc_mpeg4", &pdata->mfc_bw_info.bw_enc_mpeg4.peak, 3);
	of_property_read_u32_array(np, "bw_dec_h264", &pdata->mfc_bw_info.bw_dec_h264.peak, 3);
	of_property_read_u32_array(np, "bw_dec_hevc", &pdata->mfc_bw_info.bw_dec_hevc.peak, 3);
	of_property_read_u32_array(np, "bw_dec_hevc_10bit", &pdata->mfc_bw_info.bw_dec_hevc_10bit.peak, 3);
	of_property_read_u32_array(np, "bw_dec_vp8", &pdata->mfc_bw_info.bw_dec_vp8.peak, 3);
	of_property_read_u32_array(np, "bw_dec_vp9", &pdata->mfc_bw_info.bw_dec_vp9.peak, 3);
	of_property_read_u32_array(np, "bw_dec_vp9_10bit", &pdata->mfc_bw_info.bw_dec_vp9_10bit.peak, 3);
	of_property_read_u32_array(np, "bw_dec_mpeg4", &pdata->mfc_bw_info.bw_dec_mpeg4.peak, 3);

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
	}

#ifdef MFC_USE_BTS
#ifndef MFC_NO_RENEWAL_BTS
	pdata->mfc_bw_index = bts_get_bwindex("mfc");
#endif
#endif

	/* QoS */
	of_property_read_u32(np, "num_default_qos_steps",
			&pdata->num_default_qos_steps);
	of_property_read_u32(np, "num_encoder_qos_steps",
			&pdata->num_encoder_qos_steps);
	of_property_read_u32(np, "max_mb", &pdata->max_mb);
	of_property_read_u32(np, "mfc_freq_control", &pdata->mfc_freq_control);
	of_property_read_u32(np, "mo_control", &pdata->mo_control);
	of_property_read_u32(np, "bw_control", &pdata->bw_control);

	pdata->default_qos_table = devm_kzalloc(mfc->device,
			sizeof(struct mfc_qos) * pdata->num_default_qos_steps,
			GFP_KERNEL);
	for (i = 0; i < pdata->num_default_qos_steps; i++) {
		snprintf(node, sizeof(node), "mfc_d_qos_variant_%d", i);
		__mfc_parse_mfc_qos_platdata(np, node,
				&pdata->default_qos_table[i], mfc);
	}

	pdata->encoder_qos_table = devm_kzalloc(mfc->device,
			sizeof(struct mfc_qos) * pdata->num_encoder_qos_steps,
			GFP_KERNEL);
	for (i = 0; i < pdata->num_encoder_qos_steps; i++) {
		snprintf(node, sizeof(node), "mfc_e_qos_variant_%d", i);
		__mfc_parse_mfc_qos_platdata(np, node,
				&pdata->encoder_qos_table[i], mfc);
	}

	/* performance boost mode */
	pdata->qos_boost_table = devm_kzalloc(mfc->device,
			sizeof(struct mfc_qos_boost), GFP_KERNEL);
	np_qos = of_find_node_by_name(np, "mfc_perf_boost_table");
	if (!np_qos) {
		dev_err(mfc->device, "[QoS][BOOST] could not find mfc_perf_boost_table node\n");
		return -EINVAL;
	}
	of_property_read_u32(np_qos, "num_cluster", &pdata->qos_boost_table->num_cluster);
	of_property_read_u32(np_qos, "freq_mfc", &pdata->qos_boost_table->freq_mfc);
	of_property_read_u32(np_qos, "freq_int", &pdata->qos_boost_table->freq_int);
	of_property_read_u32(np_qos, "freq_mif", &pdata->qos_boost_table->freq_mif);
	of_property_read_u32_array(np_qos, "freq_cluster", &pdata->qos_boost_table->freq_cluster[0],
			pdata->qos_boost_table->num_cluster);

	of_property_read_string(np_qos, "bts_scen",
			&pdata->qos_boost_table->name);
	if (!pdata->qos_boost_table->name) {
		pr_err("[QoS][BOOST] bts_scen is missing in qos_boost node");
		return -EINVAL;
	}

#ifdef MFC_USE_BTS
#ifndef MFC_NO_RENEWAL_BTS
	pdata->qos_boost_table->bts_scen_idx =
		bts_get_scenindex(pdata->qos_boost_table->name);
#endif
#endif

	/* QoS weight */
	of_property_read_u32(np, "qos_weight_h264_hevc", &pdata->qos_weight.weight_h264_hevc);
	of_property_read_u32(np, "qos_weight_vp8_vp9", &pdata->qos_weight.weight_vp8_vp9);
	of_property_read_u32(np, "qos_weight_other_codec", &pdata->qos_weight.weight_other_codec);
	of_property_read_u32(np, "qos_weight_3plane", &pdata->qos_weight.weight_3plane);
	of_property_read_u32(np, "qos_weight_10bit", &pdata->qos_weight.weight_10bit);
	of_property_read_u32(np, "qos_weight_422", &pdata->qos_weight.weight_422);
	of_property_read_u32(np, "qos_weight_bframe", &pdata->qos_weight.weight_bframe);
	of_property_read_u32(np, "qos_weight_num_of_ref", &pdata->qos_weight.weight_num_of_ref);
	of_property_read_u32(np, "qos_weight_gpb", &pdata->qos_weight.weight_gpb);
	of_property_read_u32(np, "qos_weight_num_of_tile", &pdata->qos_weight.weight_num_of_tile);
	of_property_read_u32(np, "qos_weight_super64_bframe", &pdata->qos_weight.weight_super64_bframe);

	/* Bitrate control for QoS */
	of_property_read_u32(np, "num_mfc_freq", &pdata->num_mfc_freq);
	if (pdata->num_mfc_freq)
		of_property_read_u32_array(np, "mfc_freqs", pdata->mfc_freqs,
				pdata->num_mfc_freq);
	of_property_read_u32_array(np, "max_Kbps", pdata->max_Kbps,
			MAX_NUM_MFC_BPS);
	__mfc_create_bitrate_table(mfc);

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
	else if (IS_ENC_NODE(node_num))
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

	snprintf(vfd->name, sizeof(vfd->name), "%s%d", vfd->name, dev->id);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, node_num + 60 * dev->id);
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

static int __mfc_register_resource(struct platform_device *pdev, struct mfc_dev *dev)
{
	struct device_node *np = dev->device->of_node;
	struct device_node *iommu;
	struct device_node *hwfc;
	struct device_node *mmcache;
	struct device_node *cmu = NULL;
	struct device_node *ssmt = NULL;
	struct device_node *sysreg = NULL;
	struct resource *res;
	int ret;

	mfc_perf_register(dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		return -ENOENT;
	}
	dev->mfc_mem = request_mem_region(res->start, resource_size(res), pdev->name);
	if (dev->mfc_mem == NULL) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		return -ENOENT;
	}
	dev->regs_base = ioremap(dev->mfc_mem->start, resource_size(dev->mfc_mem));
	if (dev->regs_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap address region\n");
		goto err_ioremap;
	}

	iommu = of_get_child_by_name(np, "iommu");
	if (!iommu) {
		dev_err(&pdev->dev, "failed to get iommu node\n");
		goto err_ioremap_mmu0;
	}

	dev->sysmmu0_base = of_iomap(iommu, 0);
	if (dev->sysmmu0_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap sysmmu0 address region\n");
		goto err_ioremap_mmu0;
	}

	dev->sysmmu1_base = of_iomap(iommu, 1);
	if (dev->sysmmu1_base == NULL) {
		dev_dbg(&pdev->dev, "there is only one MFC sysmmu\n");
	} else {
		dev->has_2sysmmu = 1;
	}

	hwfc = of_get_child_by_name(np, "hwfc");
	if (hwfc) {
		dev->hwfc_base = of_iomap(hwfc, 0);
		if (dev->hwfc_base == NULL) {
			dev->has_hwfc = 0;
			dev_err(&pdev->dev, "failed to iomap hwfc address region\n");
			goto err_ioremap_hwfc;
		} else {
			dev->has_hwfc = 1;
		}
	}

	mmcache = of_get_child_by_name(np, "mmcache");
	if (mmcache) {
		dev->mmcache.base = of_iomap(mmcache, 0);
		if (dev->mmcache.base == NULL) {
			dev->has_mmcache = 0;
			dev_err(&pdev->dev, "failed to iomap mmcache address region\n");
			goto err_ioremap_mmcache;
		} else {
			dev->has_mmcache = 1;
		}

		cmu = of_get_child_by_name(np, "cmu");
		if (cmu) {
			dev->cmu_busc_base = of_iomap(cmu, 0);
			if (dev->cmu_busc_base == NULL) {
				dev_err(&pdev->dev, "failed to iomap busc address region\n");
				goto err_ioremap_cmu_busc;
			}
			dev->cmu_mif0_base = of_iomap(cmu, 1);
			if (dev->cmu_mif0_base == NULL) {
				dev_err(&pdev->dev, "failed to iomap mif0 address region\n");
				goto err_ioremap_cmu_mif0;
			}
			dev->cmu_mif1_base = of_iomap(cmu, 2);
			if (dev->cmu_mif1_base == NULL) {
				dev_err(&pdev->dev, "failed to iomap mif1 address region\n");
				goto err_ioremap_cmu_mif1;
			}
			dev->cmu_mif2_base = of_iomap(cmu, 3);
			if (dev->cmu_mif2_base == NULL) {
				dev_err(&pdev->dev, "failed to iomap mif2 address region\n");
				goto err_ioremap_cmu_mif2;
			}
			dev->cmu_mif3_base = of_iomap(cmu, 4);
			if (dev->cmu_mif3_base == NULL) {
				dev_err(&pdev->dev, "failed to iomap mif3 address region\n");
				goto err_ioremap_cmu_mif3;
			}
			dev->has_cmu = 1;
		}
	}

	ssmt = of_get_child_by_name(np, "ssmt");
	if (!ssmt) {
		dev_err(&pdev->dev, "failed to get ssmt node\n");
		goto err_ioremap_ssmt0;
	}

	if (ssmt) {
		dev->ssmt0_base = of_iomap(ssmt, 0);
		if (dev->ssmt0_base == NULL) {
			dev_err(&pdev->dev, "failed to ioremap ssmt0 address region\n");
			goto err_ioremap_ssmt0;
		}

		dev->ssmt1_base = of_iomap(ssmt, 1);
		if (dev->ssmt1_base == NULL) {
			dev_err(&pdev->dev, "failed to ioremap ssmt1 address region\n");
			goto err_ioremap_ssmt1;
		}
	}

	sysreg = of_get_child_by_name(np, "sysreg");
	if (!sysreg) {
		dev_err(&pdev->dev, "failed to get sysreg node\n");
		goto err_ioremap_sysreg;
	}

	if (sysreg) {
		dev->sysreg_base = of_iomap(sysreg, 0);
		if (dev->sysreg_base == NULL) {
			dev_err(&pdev->dev, "failed to ioremap sysreg address region\n");
			goto err_ioremap_sysreg;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get irq resource\n");
		goto err_res_irq;
	}
	dev->irq = res->start;
	ret = request_threaded_irq(dev->irq, mfc_top_half_irq, mfc_irq,
				IRQF_ONESHOT, pdev->name, dev);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to install irq (%d)\n", ret);
		goto err_res_irq;
	}

	return 0;

err_res_irq:
	if (sysreg)
		iounmap(dev->sysreg_base);
err_ioremap_sysreg:
	if (ssmt)
		iounmap(dev->ssmt1_base);
err_ioremap_ssmt1:
	if (ssmt)
		iounmap(dev->ssmt0_base);
err_ioremap_ssmt0:
	if (cmu)
		iounmap(dev->cmu_mif3_base);
err_ioremap_cmu_mif3:
	if (cmu)
		iounmap(dev->cmu_mif2_base);
err_ioremap_cmu_mif2:
	if (cmu)
		iounmap(dev->cmu_mif1_base);
err_ioremap_cmu_mif1:
	if (cmu)
		iounmap(dev->cmu_mif0_base);
err_ioremap_cmu_mif0:
	if (cmu)
		iounmap(dev->cmu_busc_base);
err_ioremap_cmu_busc:
	if (dev->has_mmcache)
		iounmap(dev->mmcache.base);
err_ioremap_mmcache:
	if (dev->has_hwfc)
		iounmap(dev->hwfc_base);
err_ioremap_hwfc:
	if (dev->has_2sysmmu)
		iounmap(dev->sysmmu1_base);
	iounmap(dev->sysmmu0_base);
err_ioremap_mmu0:
	iounmap(dev->regs_base);
err_ioremap:
	release_mem_region(dev->mfc_mem->start, resource_size(dev->mfc_mem));
	return -ENOENT;
}

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
static int __mfc_itmon_notifier(struct notifier_block *nb, unsigned long action, void *nb_data)
{
	struct mfc_dev *dev;
	struct itmon_notifier *itmon_info = nb_data;
	int is_mfc_itmon = 0, is_master = 0;
	int is_mmcache_itmon = 0;
	int ret = NOTIFY_OK;

	dev = container_of(nb, struct mfc_dev, itmon_nb);

	if (IS_ERR_OR_NULL(itmon_info))
		return ret;

	/* print dump if it is an MFC ITMON error */
	if (itmon_info->port &&
			strncmp("MFC", itmon_info->port, sizeof("MFC") - 1) == 0) {
		is_mfc_itmon = 1;
		is_master = 1;
	} else if (itmon_info->master &&
			strncmp("MFC", itmon_info->master, sizeof("MFC") - 1) == 0) {
		is_mfc_itmon = 1;
		is_master = 1;
	} else if (itmon_info->dest &&
			strncmp("MFC", itmon_info->dest, sizeof("MFC") - 1) == 0) {
		is_mfc_itmon = 1;
		is_master = 0;
	} else if (itmon_info->port && dev->has_mmcache &&
			strncmp("M-CACHE", itmon_info->port,
				sizeof("M-CACHE") - 1) == 0) {
		is_mmcache_itmon = 1;
		is_master = 1;
	}

	if (is_mfc_itmon || is_mmcache_itmon) {
		dev_err(dev->device, "mfc_itmon_notifier: %s +\n",
				is_mfc_itmon ? "MFC" : "MMCACHE");
		dev_err(dev->device, "%s is %s\n",
				is_mfc_itmon ? "MFC" : "MMCACHE",
				is_master ? "master" : "dest");
		if (!dev->itmon_notified) {
			dev_err(dev->device, "dump MFC %s information\n",
					is_mmcache_itmon ? "MMCACHE" : "");
			if (is_mmcache_itmon)
				mfc_mmcache_dump_info(dev);
			if (is_master || (!is_master && itmon_info->onoff))
				call_dop(dev, dump_info, dev);
			else
				call_dop(dev, dump_info_without_regs, dev);
		} else {
			dev_err(dev->device, "MFC notifier has already been called. skip MFC information\n");
		}
		dev_err(dev->device, "mfc_itmon_notifier: %s -\n",
				is_mfc_itmon ? "MFC" : "MMCACHE");
		dev->itmon_notified = 1;
		ret = NOTIFY_BAD;
		s3c2410wdt_set_emergency_reset(3, 0);
	}

	return ret;
}
#endif

/* MFC probe function */
static int mfc_probe(struct platform_device *pdev)
{
	struct mfc_dev *dev;
	int ret = -ENOENT;
	int i;

	dev_dbg(&pdev->dev, "%s()\n", __func__);
	dev = devm_kzalloc(&pdev->dev, sizeof(struct mfc_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Not enough memory for MFC device\n");
		return -ENOMEM;
	}

	dev->device = &pdev->dev;
	dev->pdata = pdev->dev.platform_data;

	dev->variant = __mfc_get_drv_data(pdev);

	if (dev->device->of_node)
		dev->id = of_alias_get_id(pdev->dev.of_node, "mfc");

	dev_dbg(&pdev->dev, "of alias get id : mfc-%d \n", dev->id);

	if (dev->id < 0 || dev->id >= dev->variant->num_entities) {
		dev_err(&pdev->dev, "Invalid platform device id: %d\n", dev->id);
		ret = -EINVAL;
		goto err_pm;
	}

	dev->pdata = devm_kzalloc(&pdev->dev, sizeof(struct mfc_platdata), GFP_KERNEL);
	if (!dev->pdata) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_pm;
	}

	__mfc_parse_dt(dev->device->of_node, dev);

	atomic_set(&dev->trace_ref, 0);
	atomic_set(&dev->trace_ref_longterm, 0);
	atomic_set(&dev->trace_ref_log, 0);
	dev->mfc_trace = g_mfc_trace;
	dev->mfc_trace_longterm = g_mfc_trace_longterm;
	dev->mfc_trace_logging = g_mfc_trace_logging;

	dma_set_mask(&pdev->dev, DMA_BIT_MASK(36));

	mfc_pm_init(dev);
	ret = __mfc_register_resource(pdev, dev);
	if (ret)
		goto err_res_mem;

	mutex_init(&dev->mfc_mutex);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto err_v4l2_dev;

	init_waitqueue_head(&dev->cmd_wq);
	mfc_init_listable_wq_dev(dev);

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

	platform_set_drvdata(pdev, dev);

	mfc_init_hwlock(dev);
	mfc_create_bits(&dev->work_bits);

	dev->watchdog_wq =
		create_singlethread_workqueue("mfc/watchdog");
	if (!dev->watchdog_wq) {
		dev_err(&pdev->dev, "failed to create workqueue for watchdog\n");
		goto err_wq_watchdog;
	}
	INIT_WORK(&dev->watchdog_work, mfc_watchdog_worker);
	atomic_set(&dev->watchdog_tick_running, 0);
	atomic_set(&dev->watchdog_tick_cnt, 0);
	atomic_set(&dev->watchdog_run, 0);
	timer_setup(&dev->watchdog_timer, mfc_watchdog_tick, 0);

	/* MFC timer for HW idle checking */
	dev->mfc_idle_wq = create_singlethread_workqueue("mfc/idle");
	if (!dev->mfc_idle_wq) {
		dev_err(&pdev->dev, "failed to create workqueue for MFC QoS idle\n");
		goto err_wq_idle;
	}
	INIT_WORK(&dev->mfc_idle_work, mfc_qos_idle_worker);
	timer_setup(&dev->mfc_idle_timer, mfc_idle_checker, 0);
	mutex_init(&dev->idle_qos_mutex);

	INIT_LIST_HEAD(&dev->qos_queue);

	/* default FW alloc is added */
	dev->butler_wq = alloc_workqueue("mfc/butler", WQ_UNBOUND
					| WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	if (dev->butler_wq == NULL) {
		dev_err(&pdev->dev, "failed to create workqueue for butler\n");
		goto err_butler_wq;
	}
	INIT_WORK(&dev->butler_work, mfc_butler_worker);

	/* dump information call-back function */
	dev->dump_ops = &mfc_dump_ops;

	atomic_set(&dev->qos_req_cur, 0);
	mutex_init(&dev->qos_mutex);

	mfc_dev_info("[QoS] control: mfc_freq(%d), mo(%d), bw(%d)\n",
			dev->pdata->mfc_freq_control, dev->pdata->mo_control, dev->pdata->bw_control);
	mfc_dev_info("[QoS]-------------------Default table\n");
	for (i = 0; i < dev->pdata->num_default_qos_steps; i++)
		mfc_dev_info("[QoS] table[%d] mfc: %d, int: %d, mif: %d, bts_scen: %s(%d)\n",
				i,
				dev->pdata->default_qos_table[i].freq_mfc,
				dev->pdata->default_qos_table[i].freq_int,
				dev->pdata->default_qos_table[i].freq_mif,
				dev->pdata->default_qos_table[i].name,
				dev->pdata->default_qos_table[i].bts_scen_idx);
	mfc_dev_info("[QoS]-------------------Encoder only table\n");
	for (i = 0; i < dev->pdata->num_encoder_qos_steps; i++)
		mfc_dev_info("[QoS] table[%d] mfc: %d, int: %d, mif: %d, bts_scen: %s(%d)\n",
				i,
				dev->pdata->encoder_qos_table[i].freq_mfc,
				dev->pdata->encoder_qos_table[i].freq_int,
				dev->pdata->encoder_qos_table[i].freq_mif,
				dev->pdata->encoder_qos_table[i].name,
				dev->pdata->encoder_qos_table[i].bts_scen_idx);

	ret = iommu_register_device_fault_handler(dev->device,
			mfc_sysmmu_fault_handler, dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register sysmmu fault handler %d\n", ret);
		ret = -EPROBE_DEFER;
		goto err_sysmmu_fault_handler;
	}

	ret = iommu_dma_reserve_iova(dev->device, 0x0,
			MFC_BASE_ADDR + dev->variant->buf_size->firmware_code
			+ dev->variant->buf_size->ctx_buf->dev_ctx);
	if (ret) {
		dev_err(&pdev->dev, "failed to reserve dva for firmware %d\n", ret);
		ret = -EPROBE_DEFER;
		goto err_reserve_iova;
	}

	g_mfc_dev = dev;

	dev->logging_data = devm_kzalloc(&pdev->dev, sizeof(struct mfc_debug), GFP_KERNEL);
	if (!dev->logging_data) {
		dev_err(&pdev->dev, "no memory for logging data\n");
		ret = -ENOMEM;
		goto err_alloc_debug;
	}

	mfc_client_pt_register(dev);

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	dev->itmon_nb.notifier_call = __mfc_itmon_notifier;
	itmon_notifier_chain_register(&dev->itmon_nb);
#endif

	mfc_init_debugfs(dev);

	dev_dbg(&pdev->dev, "%s--\n", __func__);
	return 0;

/* Deinit MFC if probe had failed */
err_alloc_debug:
err_reserve_iova:
	iommu_unregister_device_fault_handler(&pdev->dev);
err_sysmmu_fault_handler:
	destroy_workqueue(dev->butler_wq);
err_butler_wq:
	destroy_workqueue(dev->mfc_idle_wq);
err_wq_idle:
	destroy_workqueue(dev->watchdog_wq);
err_wq_watchdog:
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
	free_irq(dev->irq, dev);
	if (dev->sysreg_base)
		iounmap(dev->sysreg_base);
	if (dev->ssmt0_base)
		iounmap(dev->ssmt0_base);
	if (dev->ssmt1_base)
		iounmap(dev->ssmt1_base);
	if (dev->has_mmcache)
		iounmap(dev->mmcache.base);
	if (dev->has_hwfc)
		iounmap(dev->hwfc_base);
	if (dev->has_2sysmmu)
		iounmap(dev->sysmmu1_base);
	iounmap(dev->sysmmu0_base);
	iounmap(dev->regs_base);
	release_mem_region(dev->mfc_mem->start, resource_size(dev->mfc_mem));
err_res_mem:
	mfc_pm_final(dev);
err_pm:
	return ret;
}

/* Remove the driver */
static int mfc_remove(struct platform_device *pdev)
{
	struct mfc_dev *dev = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s++\n", __func__);
	v4l2_info(&dev->v4l2_dev, "Removing %s\n", pdev->name);
	iommu_unregister_device_fault_handler(&pdev->dev);
	if (timer_pending(&dev->watchdog_timer))
		del_timer(&dev->watchdog_timer);
	flush_workqueue(dev->watchdog_wq);
	destroy_workqueue(dev->watchdog_wq);
	if (timer_pending(&dev->mfc_idle_timer))
		del_timer(&dev->mfc_idle_timer);
	flush_workqueue(dev->mfc_idle_wq);
	destroy_workqueue(dev->mfc_idle_wq);
	flush_workqueue(dev->butler_wq);
	destroy_workqueue(dev->butler_wq);
	video_unregister_device(dev->vfd_enc);
	video_unregister_device(dev->vfd_dec);
	video_unregister_device(dev->vfd_enc_otf);
	video_unregister_device(dev->vfd_enc_otf_drm);
	v4l2_device_unregister(&dev->v4l2_dev);
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	remove_proc_entry(MFC_PROC_FW_STATUS, mfc_proc_entry);
	remove_proc_entry(MFC_PROC_DRM_INSTANCE_NUMBER, mfc_proc_entry);
	remove_proc_entry(MFC_PROC_INSTANCE_NUMBER, mfc_proc_entry);
	remove_proc_entry(MFC_PROC_ROOT, NULL);
#endif
	mfc_destroy_listable_wq_dev(dev);
	mfc_dev_debug(2, "Will now deinit HW\n");
	mfc_run_deinit_hw(dev);
	free_irq(dev->irq, dev);
	if (dev->sysreg_base)
		iounmap(dev->sysreg_base);
	if (dev->ssmt0_base)
		iounmap(dev->ssmt0_base);
	if (dev->ssmt1_base)
		iounmap(dev->ssmt1_base);
	if (dev->has_mmcache)
		iounmap(dev->mmcache.base);
	if (dev->has_hwfc)
		iounmap(dev->hwfc_base);
	if (dev->has_2sysmmu)
		iounmap(dev->sysmmu1_base);
	iounmap(dev->sysmmu0_base);
	iounmap(dev->regs_base);

	mfc_client_pt_unregister(dev);

	release_mem_region(dev->mfc_mem->start, resource_size(dev->mfc_mem));
	mfc_pm_final(dev);
	kfree(dev);
	dev_dbg(&pdev->dev, "%s--\n", __func__);
	return 0;
}

static void mfc_shutdown(struct platform_device *pdev)
{
	struct mfc_dev *dev = platform_get_drvdata(pdev);
	int ret;

	mfc_dev_info("MFC shutdown is called\n");

	if (!mfc_pm_get_pwr_ref_cnt(dev)) {
		dev->shutdown = 1;
		mfc_dev_info("MFC is not running\n");
		return;
	}

	ret = mfc_get_hwlock_dev(dev);
	if (ret < 0)
		mfc_dev_err("Failed to get hwlock\n");

	if (!dev->shutdown) {
		mfc_risc_off(dev);
		dev->shutdown = 1;
		mfc_clear_all_bits(&dev->work_bits);
	}
	mfc_release_hwlock_dev(dev);
	mfc_dev_info("MFC shutdown completed\n");
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int mfc_suspend(struct device *device)
{
	struct mfc_dev *dev = platform_get_drvdata(to_platform_device(device));
	int ret;

	if (!dev) {
		mfc_err("no mfc device to run\n");
		return -EINVAL;
	}

	if (dev->num_inst == 0)
		return 0;

	ret = mfc_get_hwlock_dev(dev);
	if (ret < 0) {
		mfc_dev_err("Failed to get hwlock\n");
		mfc_dev_err("dev:0x%lx, bits:0x%lx, owned:%d, wl:%d, trans:%d\n",
				dev->hwlock.dev, dev->hwlock.bits, dev->hwlock.owned_by_irq,
				dev->hwlock.wl_count, dev->hwlock.transfer_owner);
		return -EBUSY;
	}

	ret = mfc_run_sleep(dev);

	if (dev->has_mmcache && dev->mmcache.is_on_status) {
		mfc_invalidate_mmcache(dev);
		mfc_mmcache_disable(dev);
	}

	if (dev->has_llc && dev->llc_on_status) {
		mfc_llc_flush(dev);
		mfc_llc_disable(dev);
	}

	if (dev->has_slc && dev->slc_on_status)
		mfc_slc_disable(dev);

	mfc_release_hwlock_dev(dev);

	return ret;
}

static int mfc_resume(struct device *device)
{
	struct mfc_dev *dev = platform_get_drvdata(to_platform_device(device));
	int ret;

	if (!dev) {
		mfc_err("no mfc device to run\n");
		return -EINVAL;
	}

	if (dev->num_inst == 0)
		return 0;

	ret = mfc_get_hwlock_dev(dev);
	if (ret < 0) {
		mfc_dev_err("Failed to get hwlock\n");
		mfc_dev_err("dev:0x%lx, bits:0x%lx, owned:%d, wl:%d, trans:%d\n",
				dev->hwlock.dev, dev->hwlock.bits, dev->hwlock.owned_by_irq,
				dev->hwlock.wl_count, dev->hwlock.transfer_owner);
		return -EBUSY;
	}

	if (dev->has_mmcache && (dev->mmcache.is_on_status == 0))
		mfc_mmcache_enable(dev);

	if (dev->has_llc && (dev->llc_on_status == 0))
		mfc_llc_enable(dev);

	if (dev->has_slc && (dev->slc_on_status == 0))
		mfc_slc_enable(dev);

	ret = mfc_run_wakeup(dev);
	mfc_release_hwlock_dev(dev);

	return ret;
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
	.other_dec_ctx	= PAGE_ALIGN(0xC800),	/*  50KB */
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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kamil Debski <k.debski@samsung.com>");
