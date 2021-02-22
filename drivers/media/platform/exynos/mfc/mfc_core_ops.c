/*
 * drivers/media/platform/exynos/mfc/mfc_core_ops.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/soc/samsung/exynos-smc.h>

#include "mfc_common.h"

#include "mfc_core_hwlock.h"
#include "mfc_core_pm.h"
#include "mfc_core_run.h"
#include "mfc_core_nal_q.h"
#include "mfc_core_otf.h"
#include "mfc_core_qos.h"

#include "mfc_perf_measure.h"
#include "mfc_meminfo.h"
#include "mfc_core_hw_reg_api.h"
#include "mfc_llc.h"
#include "mfc_slc.h"

#include "mfc_sync.h"
#include "mfc_buf.h"
#include "mfc_utils.h"
#include "mfc_queue.h"
#include "mfc_mem.h"

static int __mfc_core_init(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = core->dev;
	int ret = 0;

	/* set meerkat timer */
	mod_timer(&core->meerkat_timer, jiffies + msecs_to_jiffies(MEERKAT_TICK_INTERVAL));

	/* set MFC idle timer */
	atomic_set(&core->hw_run_cnt, 0);
	mfc_core_change_idle_mode(core, MFC_IDLE_MODE_NONE);

	/* Load the FW */
	ret = mfc_load_firmware(core);
	if (ret)
		goto err_fw_load;

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (!core->drm_fw_buf.daddr) {
		mfc_core_err("DRM F/W buffer is not allocated\n");
		core->fw.drm_status = 0;
	} else {
		/* Request buffer protection for DRM F/W */
		ret = exynos_smc(SMC_DRM_PPMP_MFCFW_PROT,
				core->drm_fw_buf.daddr, 0, 0);
		if (ret != DRMDRV_OK) {
			mfc_core_err("failed MFC DRM F/W prot(%#x)\n", ret);
			call_dop(core, dump_and_stop_debug_mode, core);
			core->fw.drm_status = 0;
		} else {
			core->fw.drm_status = 1;
		}
	}
#endif

	ret = mfc_alloc_common_context(core);
	if (ret < 0) {
		mfc_core_err("Failed to alloc common context\n");
		goto err_common_ctx;
	}

	if (dbg_enable)
		mfc_alloc_dbg_info_buffer(core);

#if !IS_ENABLED(CONFIG_EXYNOS_IMGLOADER)
	ret = mfc_power_on_verify_fw(core, 0, core->fw_buf.paddr,
				core->fw.fw_size, core->fw_buf.size);
	if (ret < 0)
		goto err_pwr_enable;
#endif

	core->curr_core_ctx = ctx->num;
	core->preempt_core_ctx = MFC_NO_INSTANCE_SET;
	core->curr_core_ctx_is_drm = ctx->is_drm;

	ret = mfc_core_run_init_hw(core);
	if (ret) {
		mfc_core_err("Failed to init mfc h/w\n");
		goto err_hw_init;
	}

	if (core->has_llc && (core->llc_on_status == 0))
		mfc_llc_enable(core);

	if (core->has_slc && (core->slc_on_status == 0))
		mfc_slc_enable(core);

	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->nal_q)) {
		core->nal_q_handle = mfc_core_nal_q_create(core);
		if (core->nal_q_handle == NULL)
			mfc_core_err("[NALQ] Can't create nal q\n");
	}

	return ret;

err_hw_init:
#if !IS_ENABLED(CONFIG_EXYNOS_IMGLOADER)
	mfc_core_pm_power_off(core);

err_pwr_enable:
#endif
	mfc_release_common_context(core);

err_common_ctx:
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if (core->fw.drm_status) {
		int smc_ret = 0;
		core->fw.drm_status = 0;
		/* Request buffer unprotection for DRM F/W */
		smc_ret = exynos_smc(SMC_DRM_PPMP_MFCFW_UNPROT,
					core->drm_fw_buf.daddr, 0, 0);
		if (smc_ret != DRMDRV_OK) {
			mfc_core_err("failed MFC DRM F/W unprot(%#x)\n", smc_ret);
			call_dop(core, dump_and_stop_debug_mode, core);
		}
	}
#endif

err_fw_load:
	del_timer(&core->meerkat_timer);
	del_timer(&core->mfc_idle_timer);

	mfc_core_err("failed to init first instance\n");
	return ret;
}

static int __mfc_wait_close_inst(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int ret = 0;

	if (core->state == MFCCORE_ERROR) {
		mfc_core_info("[MSR] Couldn't close inst. It's Error state\n");
		return 0;
	}

	if (atomic_read(&core->meerkat_run)) {
		mfc_err("meerkat already running!\n");
		return 0;
	}

	if (core_ctx->state <= MFCINST_INIT) {
		mfc_debug(2, "mfc instance didn't opend or already closed\n");
		return 0;
	}

	mfc_clean_core_ctx_int_flags(core_ctx);
	mfc_change_state(core_ctx, MFCINST_RETURN_INST);
	mfc_set_bit(ctx->num, &core->work_bits);

	/* To issue the command 'CLOSE_INSTANCE' */
	if (mfc_core_just_run(core, ctx->num)) {
		mfc_err("failed to run MFC, state: %d\n", core_ctx->state);
		MFC_TRACE_CTX_LT("[ERR][Release] failed to run MFC, state: %d\n", core_ctx->state);
		return -EIO;
	}

	/* Wait until instance is returned or timeout occured */
	ret = mfc_wait_for_done_core_ctx(core_ctx, MFC_REG_R2H_CMD_CLOSE_INSTANCE_RET);
	if (ret == 1) {
		mfc_err("failed to wait CLOSE_INSTANCE(timeout)\n");

		if (mfc_wait_for_done_core_ctx(core_ctx,
					MFC_REG_R2H_CMD_CLOSE_INSTANCE_RET)) {
			mfc_err("waited once more but failed to wait CLOSE_INSTANCE\n");
			core->logging_data->cause |= (1 << MFC_CAUSE_FAIL_CLOSE_INST);
			call_dop(core, dump_and_stop_always, core);
		}
	} else if (ret == -1) {
		mfc_err("failed to wait CLOSE_INSTANCE(err)\n");
		call_dop(core, dump_and_stop_debug_mode, core);
	}

	return 0;
}

static int __mfc_core_deinit(struct mfc_core *core, struct mfc_ctx *ctx)
{
	int ret = 0;

	mfc_clear_bit(ctx->num, &core->work_bits);

	ret = __mfc_wait_close_inst(core, ctx);
	if (ret) {
		mfc_ctx_err("Failed to close instance\n");
		return ret;
	}

	if ((ctx->gdc_votf && core->has_gdc_votf && core->has_mfc_votf) ||
			(ctx->otf_handle && core->has_dpu_votf && core->has_mfc_votf))
		mfc_core_clear_votf(core);

	if (ctx->is_drm)
		core->num_drm_inst--;
	core->num_inst--;

	if (core->num_inst == 0) {
		mfc_core_run_deinit_hw(core);

		if (perf_boost_mode)
			mfc_core_perf_boost_disable(core);

		del_timer(&core->meerkat_timer);
		del_timer(&core->mfc_idle_timer);

		flush_workqueue(core->butler_wq);

		mfc_debug(2, "power off\n");
		mfc_core_pm_power_off(core);

		if (dbg_enable)
			mfc_release_dbg_info_buffer(core);

		mfc_release_common_context(core);

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
		if (core->fw.drm_status) {
			core->fw.drm_status = 0;
			/* Request buffer unprotection for DRM F/W */
			ret = exynos_smc(SMC_DRM_PPMP_MFCFW_UNPROT,
					core->drm_fw_buf.daddr, 0, 0);
			if (ret != DRMDRV_OK) {
				mfc_ctx_err("failed MFC DRM F/W unprot(%#x)\n", ret);
				call_dop(core, dump_and_stop_debug_mode, core);
			}
		}
#endif

		if (core->nal_q_handle)
			mfc_core_nal_q_destroy(core, core->nal_q_handle);

		if (core->state == MFCCORE_ERROR) {
			mfc_core_change_state(core, MFCCORE_INIT);
			mfc_ctx_info("[MSR] MFC-%d will be reset\n", core->id);
		}
	}

	mfc_core_qos_off(core, ctx);

	if (core->has_llc && core->llc_on_status) {
		mfc_llc_flush(core);

		if (core->num_inst == 0)
			mfc_llc_disable(core);
		else
			if (ctx->is_8k)
				mfc_llc_update_size(core, false);
	}

	if (core->has_slc && core->slc_on_status)
		mfc_slc_disable(core);

	return 0;
}

static int __mfc_force_close_inst(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	enum mfc_inst_state prev_state;

	if (core_ctx->state == MFCINST_FREE)
		return 0;

	prev_state = core_ctx->state;
	mfc_change_state(core_ctx, MFCINST_RETURN_INST);
	mfc_set_bit(ctx->num, &core->work_bits);
	mfc_clean_core_ctx_int_flags(core_ctx);
	if (mfc_core_just_run(core, ctx->num)) {
		mfc_err("Failed to run MFC\n");
		mfc_change_state(core_ctx, prev_state);
		return -EIO;
	}

	/* Wait until instance is returned or timeout occured */
	if (mfc_wait_for_done_core_ctx(core_ctx,
				MFC_REG_R2H_CMD_CLOSE_INSTANCE_RET)) {
		mfc_err("Waiting for CLOSE_INSTANCE timed out\n");
		mfc_change_state(core_ctx, prev_state);
		return -EIO;
	}

	/* Free resources */
	mfc_release_instance_context(core_ctx);

	return 0;
}

int mfc_core_instance_init(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = core->dev;
	struct mfc_core_ctx *core_ctx = NULL;
	int ret = 0;

	mfc_core_debug_enter();

	if (core->state == MFCCORE_ERROR) {
		mfc_ctx_err("MFC-%d is ERROR state\n", core->id);
		return -EBUSY;
	}

	ret = mfc_core_get_hwlock_dev(core);
	if (ret < 0) {
		mfc_core_err("Failed to get hwlock\n");
		mfc_core_err("dev.hwlock.dev = 0x%lx, bits = 0x%lx, owned_by_irq = %d, wl_count = %d, transfer_owner = %d\n",
				core->hwlock.dev, core->hwlock.bits, core->hwlock.owned_by_irq,
				core->hwlock.wl_count, core->hwlock.transfer_owner);
		goto err_hw_lock;
	}

	core->num_inst++;
	if (ctx->is_drm)
		core->num_drm_inst++;

	/* Allocate memory for core context */
	core_ctx = kzalloc(sizeof(*core_ctx), GFP_KERNEL);
	if (!core_ctx) {
		mfc_core_err("Not enough memory\n");
		ret = -ENOMEM;
		goto err_core_ctx_alloc;
	}

	core_ctx->core = core;
	core_ctx->ctx = ctx;
	core_ctx->num = ctx->num;
	core_ctx->is_drm = ctx->is_drm;
	core_ctx->inst_no = MFC_NO_INSTANCE_SET;
	core->core_ctx[core_ctx->num] = core_ctx;

	init_waitqueue_head(&core_ctx->cmd_wq);
	mfc_core_init_listable_wq_ctx(core_ctx);
	spin_lock_init(&core_ctx->buf_queue_lock);
	mfc_clear_bit(core_ctx->num, &core->work_bits);
	INIT_LIST_HEAD(&core_ctx->qos_list);

	mfc_create_queue(&core_ctx->src_buf_queue);

	if (core->num_inst == 1) {
		ret = __mfc_core_init(core, ctx);
		if (ret)
			goto err_init_inst;

		if (perf_boost_mode)
			mfc_core_perf_boost_enable(core);

		if (!dev->fw_date)
			dev->fw_date = core->fw.date;
		else if (dev->fw_date > core->fw.date)
			dev->fw_date = core->fw.date;
	}

	mfc_core_release_hwlock_dev(core);
	mfc_perf_init(core);

	mfc_core_debug_leave();

	return ret;

err_init_inst:
	core->core_ctx[core_ctx->num] = 0;
	kfree(core_ctx);
err_core_ctx_alloc:
	core->num_inst--;
	mfc_core_release_hwlock_dev(core);
err_hw_lock:
	return ret;
}

int mfc_core_instance_deinit(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int ret = 0;

	if (!core_ctx) {
		mfc_core_err("There is no instance\n");
		return -EINVAL;
	}

	mfc_clear_bit(ctx->num, &core->work_bits);

	/* If a H/W operation is in progress, wait for it complete */
	if (need_to_wait_nal_abort(core_ctx)) {
		if (mfc_wait_for_done_core_ctx(core_ctx, MFC_REG_R2H_CMD_NAL_ABORT_RET)) {
			mfc_err("Failed to wait nal abort\n");
			mfc_core_cleanup_work_bit_and_try_run(core_ctx);
		}
	}

	ret = mfc_core_get_hwlock_ctx(core_ctx);
	if (ret < 0) {
		mfc_err("Failed to get hwlock\n");
		MFC_TRACE_CTX_LT("[ERR][Release] failed to get hwlock (shutdown: %d)\n", core->shutdown);
		return -EBUSY;
	}

	/* If instance was initialised then return instance and free reosurces */
	ret = __mfc_core_deinit(core, ctx);
	if (ret)
		goto err_release_try;

	mfc_release_codec_buffers(core_ctx);
	mfc_release_instance_context(core_ctx);

	mfc_core_release_hwlock_ctx(core_ctx);
	mfc_core_destroy_listable_wq_ctx(core_ctx);

	if (ctx->type == MFCINST_ENCODER)
		mfc_release_enc_roi_buffer(core_ctx);

	mfc_delete_queue(&core_ctx->src_buf_queue);

	core->core_ctx[core_ctx->num] = 0;
	kfree(core_ctx);

	mfc_perf_print();

	return 0;

err_release_try:
	mfc_core_release_hwlock_ctx(core_ctx);
	mfc_core_cleanup_work_bit_and_try_run(core_ctx);
	return ret;
}

static int __mfc_core_instance_open_dec(struct mfc_ctx *ctx,
				struct mfc_core_ctx *core_ctx)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_dev *dev = core->dev;
	struct mfc_dec *dec = ctx->dec_priv;
	int ret = 0;

	/* In case of calling s_fmt twice or more */
	ret = __mfc_force_close_inst(core, ctx);
	if (ret) {
		mfc_err("Failed to close already opening instance\n");
		mfc_core_release_hwlock_ctx(core_ctx);
		mfc_core_cleanup_work_bit_and_try_run(core_ctx);
		return -EIO;
	}

	ret = mfc_alloc_instance_context(core_ctx);
	if (ret) {
		mfc_err("Failed to allocate dec instance[%d] buffers\n",
				ctx->num);
		mfc_core_release_hwlock_ctx(core_ctx);
		return -ENOMEM;
	}

	/* sh_handle: HDR10+ (HEVC or AV1) SEI meta */
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->hdr10_plus) &&
			(IS_HEVC_DEC(ctx) || IS_AV1_DEC(ctx)))
		dec->hdr10_plus_info = vmalloc(
				(sizeof(struct hdr10_plus_meta) * MFC_MAX_DPBS));

	/* sh_handle: AV1 Film Grain SEI meta */
	if (MFC_FEATURE_SUPPORT(dev, dev->pdata->av1_film_grain) &&
			IS_AV1_DEC(ctx))
		dec->av1_film_grain_info = vmalloc(
				(sizeof(struct av1_film_grain_meta) * MFC_MAX_DPBS));

	return 0;
}

static int __mfc_core_instance_open_enc(struct mfc_ctx *ctx,
				struct mfc_core_ctx *core_ctx)
{
	int ret = 0;

	ret = mfc_alloc_instance_context(core_ctx);
	if (ret) {
		mfc_err("Failed to allocate enc instance[%d] buffers\n",
				core_ctx->num);
		mfc_core_release_hwlock_ctx(core_ctx);
		return -ENOMEM;
	}

	ctx->capture_state = QUEUE_FREE;

	ret = mfc_alloc_enc_roi_buffer(core_ctx);
	if (ret) {
		mfc_err("[ROI] Failed to allocate ROI buffers\n");
		mfc_release_instance_context(core_ctx);
		mfc_core_release_hwlock_ctx(core_ctx);
		return -ENOMEM;
	}

	return 0;
}

int mfc_core_instance_open(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int ret = 0;

	if (!core_ctx) {
		mfc_core_err("There is no instance\n");
		return -EINVAL;
	}

	ret = mfc_core_get_hwlock_ctx(core_ctx);
	if (ret < 0) {
		mfc_err("Failed to get hwlock\n");
		return ret;
	}

	if (ctx->type == MFCINST_DECODER) {
		if (__mfc_core_instance_open_dec(ctx, core_ctx))
			return -EAGAIN;
	} else if (ctx->type == MFCINST_ENCODER) {
		if (__mfc_core_instance_open_enc(ctx, core_ctx))
			return -EAGAIN;
	} else {
		mfc_err("invalid codec type: %d\n", ctx->type);
		return -EINVAL;
	}

	mfc_change_state(core_ctx, MFCINST_INIT);
	mfc_set_bit(ctx->num, &core->work_bits);
	ret = mfc_core_just_run(core, ctx->num);
	if (ret) {
		mfc_err("Failed to run MFC\n");
		goto err_open;
	}

	if (mfc_wait_for_done_core_ctx(core_ctx,
			MFC_REG_R2H_CMD_OPEN_INSTANCE_RET)) {
		mfc_err("failed to wait OPEN_INSTANCE\n");
		mfc_change_state(core_ctx, MFCINST_FREE);
		ret = -EIO;
		goto err_open;
	}

	mfc_core_release_hwlock_ctx(core_ctx);

	mfc_debug(2, "Got instance number inst_no: %d\n", core_ctx->inst_no);

	mfc_ctx_ready_set_bit(core_ctx, &core->work_bits);
	if (ctx->otf_handle)
		mfc_core_otf_ctx_ready_set_bit(core_ctx, &core->work_bits);
	if (mfc_core_is_work_to_do(core))
		queue_work(core->butler_wq, &core->butler_work);

	return ret;

err_open:
	mfc_core_release_hwlock_ctx(core_ctx);
	mfc_core_cleanup_work_bit_and_try_run(core_ctx);
	mfc_release_instance_context(core_ctx);
	if (ctx->type == MFCINST_ENCODER)
		mfc_release_enc_roi_buffer(core_ctx);

	return ret;
}

int mfc_core_instance_move_to(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = NULL;
	int drm_switch = 0;

	core->num_inst++;
	if (ctx->is_drm)
		core->num_drm_inst++;

	/* Allocate memory for core context */
	core_ctx = kzalloc(sizeof(*core_ctx), GFP_KERNEL);
	if (!core_ctx) {
		mfc_core_err("Not enough memory\n");
		return -ENOMEM;
	}

	core_ctx->core = core;
	core_ctx->ctx = ctx;
	core_ctx->num = ctx->num;
	core_ctx->is_drm = ctx->is_drm;
	core->core_ctx[core_ctx->num] = core_ctx;

	init_waitqueue_head(&core_ctx->cmd_wq);
	mfc_core_init_listable_wq_ctx(core_ctx);
	spin_lock_init(&core_ctx->buf_queue_lock);
	mfc_clear_bit(core_ctx->num, &core->work_bits);
	INIT_LIST_HEAD(&core_ctx->qos_list);

	mfc_create_queue(&core_ctx->src_buf_queue);
	core->curr_core_ctx = ctx->num;

	if (core->curr_core_ctx_is_drm != ctx->is_drm)
		drm_switch = 1;

	mfc_core_pm_clock_on(core);
	mfc_core_cache_flush(core, ctx->is_drm, MFC_CACHEFLUSH, drm_switch);
	mfc_core_pm_clock_off(core);

	mfc_ctx_info("to core-%d is ready to move\n", core->id);

	return 0;
}

int mfc_core_instance_move_from(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int ret = 0;
	int inst_no;

	mfc_clean_core_ctx_int_flags(core_ctx);
	mfc_set_bit(ctx->num, &core->work_bits);

	ret = mfc_core_just_run(core, ctx->num);
	if (ret) {
		mfc_err("Failed to run MFC\n");
		return ret;
	}

	if (mfc_wait_for_done_core_ctx(core_ctx, MFC_REG_R2H_CMD_MOVE_INSTANCE_RET)) {
		mfc_err("time out during move instance\n");
		core->logging_data->cause |= (1 << MFC_CAUSE_FAIL_MOVE_INST);
		call_dop(core, dump_and_stop_always, core);
		return -EFAULT;
	}
	inst_no = mfc_core_get_inst_no();

	ret = __mfc_core_deinit(core, ctx);
	if (ret) {
		mfc_err("Failed to close instance\n");
		return ret;
	}

	mfc_ctx_info("inst_no.%d will be changed to no.%d\n", core_ctx->inst_no, inst_no);
	core_ctx->inst_no = inst_no;

	return ret;
}

void mfc_core_instance_dpb_flush(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int index = 0, i, ret;
	int prev_state;

	if ((core->state == MFCCORE_ERROR) || (core_ctx->state == MFCINST_ERROR))
		goto cleanup;

	ret = mfc_core_get_hwlock_ctx(core_ctx);
	if (ret < 0) {
		mfc_err("Failed to get hwlock\n");
		MFC_TRACE_CTX_LT("[ERR][Release] failed to get hwlock (shutdown: %d)\n", core->shutdown);
		return;
	}

	mfc_cleanup_queue(&ctx->buf_queue_lock, &ctx->dst_buf_queue);
	for (i = 0; i < MFC_MAX_DPBS; i++)
		dec->dpb[i].queued = 0;
	dec->queued_dpb = 0;
	ctx->is_dpb_realloc = 0;
	dec->y_addr_for_pb = 0;

	if (!dec->inter_res_change) {
		mfc_cleanup_iovmm(ctx);

		dec->dpb_table_used = 0;
		dec->dynamic_used = 0;
		dec->dynamic_set = 0;
		core_ctx->dynamic_set = 0;
	} else {
		mfc_cleanup_iovmm_except_used(ctx);
		mfc_print_dpb_table(ctx);
	}

	while (index < MFC_MAX_BUFFERS) {
		index = find_next_bit(&ctx->dst_ctrls_avail,
				MFC_MAX_BUFFERS, index);
		if (index < MFC_MAX_BUFFERS)
			call_cop(ctx, reset_buf_ctrls, &ctx->dst_ctrls[index]);
		index++;
	}

	if (ctx->wait_state & WAIT_STOP) {
		ctx->wait_state &= ~(WAIT_STOP);
		mfc_debug(2, "clear WAIT_STOP %d\n", ctx->wait_state);
		MFC_TRACE_CORE_CTX("** DEC clear WAIT_STOP(wait_state %d)\n",
				ctx->wait_state);
	}

	if (core_ctx->state == MFCINST_FINISHING)
		mfc_change_state(core_ctx, MFCINST_RUNNING);

	if (need_to_dpb_flush(core_ctx) && !ctx->dec_priv->inter_res_change) {
		prev_state = core_ctx->state;
		mfc_change_state(core_ctx, MFCINST_DPB_FLUSHING);
		mfc_set_bit(ctx->num, &core->work_bits);
		mfc_clean_core_ctx_int_flags(core_ctx);
		mfc_ctx_info("try to DPB flush\n");
		ret = mfc_core_just_run(core, ctx->num);
		if (ret) {
			mfc_err("Failed to run MFC\n");
			mfc_core_release_hwlock_ctx(core_ctx);
			mfc_core_cleanup_work_bit_and_try_run(core_ctx);
			return;
		}

		if (mfc_wait_for_done_core_ctx(core_ctx, MFC_REG_R2H_CMD_DPB_FLUSH_RET)) {
			mfc_err("time out during DPB flush\n");
			core->logging_data->cause |= (1 << MFC_CAUSE_FAIL_DPB_FLUSH);
			call_dop(core, dump_and_stop_always, core);
		}

		mfc_change_state(core_ctx, prev_state);
	}

	mfc_debug(2, "decoder destination stop sequence done\n");

	mfc_clear_bit(ctx->num, &core->work_bits);
	mfc_core_release_hwlock_ctx(core_ctx);

	mfc_ctx_ready_set_bit(core_ctx, &core->work_bits);
	if (mfc_core_is_work_to_do(core))
		queue_work(core->butler_wq, &core->butler_work);

	return;

cleanup:
	mfc_core_info("[MSR] Cleanup dst buffers. It's Error state\n");
	mfc_cleanup_queue(&ctx->buf_queue_lock, &ctx->dst_buf_queue);
}

void mfc_core_instance_csd_parsing(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	struct mfc_buf *src_mb;
	int index = 0, csd, condition = 0, ret = 0;
	enum mfc_inst_state prev_state = MFCINST_FREE;
	int buf_in_ready;

	if ((core->state == MFCCORE_ERROR) || (core_ctx->state == MFCINST_ERROR))
		goto cleanup;

	ret = mfc_core_get_hwlock_ctx(core_ctx);
	if (ret < 0) {
		mfc_err("Failed to get hwlock\n");
		MFC_TRACE_CTX_LT("[ERR][Release] failed to get hwlock (shutdown: %d)\n", core->shutdown);
		return;
	}

	/* Header parsed buffer is in src_buf_ready_queue */
	mfc_move_buf_all(ctx, &core_ctx->src_buf_queue,
			&ctx->src_buf_ready_queue, MFC_QUEUE_ADD_BOTTOM);
	MFC_TRACE_CORE_CTX("CSD: Move all src to queue\n");

	while (1) {
		buf_in_ready = 0;
		csd = mfc_check_buf_mb_flag(core_ctx, MFC_FLAG_CSD);
		if (csd == 1) {
			mfc_clean_core_ctx_int_flags(core_ctx);
			if (need_to_special_parsing(core_ctx)) {
				prev_state = core_ctx->state;
				mfc_change_state(core_ctx, MFCINST_SPECIAL_PARSING);
				condition = MFC_REG_R2H_CMD_SEQ_DONE_RET;
				if (!IS_SINGLE_MODE(ctx))
					buf_in_ready = 1;
				mfc_ctx_info("try to special parsing! (before NAL_START)\n");
			} else if (need_to_special_parsing_nal(core_ctx)) {
				prev_state = core_ctx->state;
				mfc_change_state(core_ctx, MFCINST_SPECIAL_PARSING_NAL);
				condition = MFC_REG_R2H_CMD_FRAME_DONE_RET;
				mfc_ctx_info("try to special parsing! (after NAL_START)\n");
			} else {
				mfc_ctx_info("can't parsing CSD!, state = %d\n",
						core_ctx->state);
			}

			if (condition) {
				mfc_set_bit(core_ctx->num, &core->work_bits);

				ret = mfc_core_just_run(core, core_ctx->num);
				if (ret) {
					mfc_err("Failed to run MFC\n");
					mfc_change_state(core_ctx, prev_state);
				} else {
					if (mfc_wait_for_done_core_ctx(core_ctx, condition))
						mfc_err("special parsing time out\n");
				}
			}
		}

		/* when multi-mode, special parsed buffer moved to ready_queue */
		if (buf_in_ready)
			src_mb = mfc_get_del_buf(ctx, &ctx->src_buf_ready_queue,
					MFC_BUF_NO_TOUCH_USED);
		else
			src_mb = mfc_get_del_buf(ctx, &core_ctx->src_buf_queue,
					MFC_BUF_NO_TOUCH_USED);
		if (!src_mb)
			break;
		else
			MFC_TRACE_CORE_CTX("CSD: src[%d] DQ\n", src_mb->src_index);

		mfc_debug(2, "src index %d(%d) DQ\n", src_mb->vb.vb2_buf.index,
				src_mb->src_index);
		vb2_set_plane_payload(&src_mb->vb.vb2_buf, 0, 0);
		vb2_buffer_done(&src_mb->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	dec->consumed = 0;
	dec->remained_size = 0;
	core_ctx->check_dump = 0;
	ctx->curr_src_index = -1;
	ctx->serial_src_index = 0;

	if (!list_empty(&core_ctx->src_buf_queue.head)) {
		mfc_err("core_ctx->src_buf_queue is not empty\n");
		mfc_cleanup_queue(&ctx->buf_queue_lock,
				&core_ctx->src_buf_queue);
	}
	if (!list_empty(&ctx->src_buf_ready_queue.head)) {
		mfc_err("ctx->src_buf_ready_queue is not empty\n");
		mfc_cleanup_queue(&ctx->buf_queue_lock,
				&ctx->src_buf_ready_queue);
	}
	mfc_init_queue(&core_ctx->src_buf_queue);
	mfc_init_queue(&ctx->src_buf_ready_queue);

	if (meminfo_enable == 1)
		mfc_meminfo_cleanup_inbuf_q(ctx);

	while (index < MFC_MAX_BUFFERS) {
		index = find_next_bit(&ctx->src_ctrls_avail,
				MFC_MAX_BUFFERS, index);
		if (index < MFC_MAX_BUFFERS)
			call_cop(ctx, reset_buf_ctrls, &ctx->src_ctrls[index]);
		index++;
	}

	if (core_ctx->state == MFCINST_FINISHING)
		mfc_change_state(core_ctx, MFCINST_RUNNING);

	mfc_debug(2, "decoder source stop sequence done\n");

	mfc_clear_bit(ctx->num, &core->work_bits);
	mfc_core_release_hwlock_ctx(core_ctx);

	mfc_ctx_ready_set_bit(core_ctx, &core->work_bits);
	if (mfc_core_is_work_to_do(core))
		queue_work(core->butler_wq, &core->butler_work);

	return;

cleanup:
	mfc_core_info("[MSR] Cleanup src buffers. It's Error state\n");
	mfc_cleanup_queue(&ctx->buf_queue_lock, &core_ctx->src_buf_queue);
	mfc_cleanup_queue(&ctx->buf_queue_lock, &ctx->src_buf_ready_queue);
}

int mfc_core_instance_init_buf(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];

	mfc_set_bit(ctx->num, &core->work_bits);
	mfc_clean_core_ctx_int_flags(core_ctx);
	if (mfc_core_just_run(core, ctx->num)) {
		mfc_err("Failed to run MFC\n");
		return -EIO;
	}

	if (mfc_wait_for_done_core_ctx(core_ctx, MFC_REG_R2H_CMD_INIT_BUFFERS_RET)) {
		mfc_ctx_err("[RM] init buffer timeout\n");
		return -EIO;
	}

	return 0;
}

void mfc_core_instance_q_flush(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int index = 0;
	int ret = 0;

	/* If a H/W operation is in progress, wait for it complete */
	if (need_to_wait_nal_abort(core_ctx)) {
		if (mfc_wait_for_done_core_ctx(core_ctx, MFC_REG_R2H_CMD_NAL_ABORT_RET)) {
			mfc_err("time out during nal abort\n");
			mfc_core_cleanup_work_bit_and_try_run(core_ctx);
		}
	}

	ret = mfc_core_get_hwlock_ctx(core_ctx);
	if (ret < 0) {
		mfc_err("Failed to get hwlock\n");
		MFC_TRACE_CTX_LT("[ERR][Release] failed to get hwlock (shutdown: %d)\n", core->shutdown);
		return;
	}

	mfc_cleanup_enc_dst_queue(ctx);
	if (meminfo_enable == 1)
		mfc_meminfo_cleanup_outbuf_q(ctx);

	while (index < MFC_MAX_BUFFERS) {
		index = find_next_bit(&ctx->dst_ctrls_avail,
				MFC_MAX_BUFFERS, index);
		if (index < MFC_MAX_BUFFERS)
			call_cop(ctx, reset_buf_ctrls, &ctx->dst_ctrls[index]);
		index++;
	}

	if (core_ctx->state == MFCINST_FINISHING)
		mfc_change_state(core_ctx, MFCINST_FINISHED);

	mfc_debug(2, "encoder destination stop sequence done\n");

	mfc_clear_bit(ctx->num, &core->work_bits);
	mfc_core_release_hwlock_ctx(core_ctx);

	mfc_ctx_ready_set_bit(core_ctx, &core->work_bits);
	if (mfc_core_is_work_to_do(core))
		queue_work(core->butler_wq, &core->butler_work);
}

void mfc_core_instance_finishing(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int index = 0;
	int ret = 0;

	/* If a H/W operation is in progress, wait for it complete */
	if (need_to_wait_nal_abort(core_ctx)) {
		if (mfc_wait_for_done_core_ctx(core_ctx, MFC_REG_R2H_CMD_NAL_ABORT_RET)) {
			mfc_err("time out during nal abort\n");
			mfc_core_cleanup_work_bit_and_try_run(core_ctx);
		}
	}

	ret = mfc_core_get_hwlock_ctx(core_ctx);
	if (ret < 0) {
		mfc_err("Failed to get hwlock\n");
		MFC_TRACE_CTX_LT("[ERR][Release] failed to get hwlock (shutdown: %d)\n", core->shutdown);
		return;
	}

	if (core_ctx->state == MFCINST_RUNNING || core_ctx->state == MFCINST_FINISHING) {
		mfc_change_state(core_ctx, MFCINST_FINISHING);
		mfc_set_bit(ctx->num, &core->work_bits);

		while (core_ctx->state != MFCINST_FINISHED) {
			ret = mfc_core_just_run(core, ctx->num);
			if (ret) {
				mfc_err("Failed to run MFC\n");
				break;
			}
			if (mfc_wait_for_done_core_ctx(core_ctx,
					MFC_REG_R2H_CMD_FRAME_DONE_RET)) {
				mfc_err("Waiting for LAST_SEQ timed out\n");
				break;
			}
		}
	}

	mfc_move_buf_all(ctx, &core_ctx->src_buf_queue,
			&ctx->ref_buf_queue, MFC_QUEUE_ADD_BOTTOM);
	mfc_move_buf_all(ctx, &core_ctx->src_buf_queue,
			&ctx->src_buf_ready_queue, MFC_QUEUE_ADD_BOTTOM);
	mfc_cleanup_enc_src_queue(core_ctx);
	if (meminfo_enable == 1)
		mfc_meminfo_cleanup_inbuf_q(ctx);

	while (index < MFC_MAX_BUFFERS) {
		index = find_next_bit(&ctx->src_ctrls_avail,
				MFC_MAX_BUFFERS, index);
		if (index < MFC_MAX_BUFFERS)
			call_cop(ctx, reset_buf_ctrls, &ctx->src_ctrls[index]);
		index++;
	}

	if (core_ctx->state == MFCINST_FINISHING
			|| core_ctx->state == MFCINST_GOT_INST
			|| core_ctx->state == MFCINST_HEAD_PARSED) {
		mfc_debug(2, "%d status can continue encoding without CLOSE_INSTANCE\n",
				core_ctx->state);
		mfc_change_state(core_ctx, MFCINST_FINISHED);
	}

	mfc_debug(2, "encoder source stop sequence done\n");

	mfc_clear_bit(ctx->num, &core->work_bits);
	mfc_core_release_hwlock_ctx(core_ctx);

	mfc_ctx_ready_set_bit(core_ctx, &core->work_bits);
	if (mfc_core_is_work_to_do(core))
		queue_work(core->butler_wq, &core->butler_work);
}

int mfc_core_request_work(struct mfc_core *core, enum mfc_request_work work,
		struct mfc_ctx *ctx)
{
	switch (work) {
	case MFC_WORK_BUTLER:
		mfc_core_debug(3, "request_work: butler\n");
		if (mfc_core_is_work_to_do(core))
			queue_work(core->butler_wq, &core->butler_work);
		break;
	case MFC_WORK_TRY:
		mfc_core_debug(3, "request_work: try_run\n");
		mfc_core_try_run(core);
		break;
	default:
		mfc_core_err("not supported request work type: %#x\n", work);
		return -EINVAL;
	}

	return 0;
}
