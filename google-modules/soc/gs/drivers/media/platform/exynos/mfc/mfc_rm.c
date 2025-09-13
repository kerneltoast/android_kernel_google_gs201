/*
 * drivers/media/platform/exynos/mfc/mfc_rm.c
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
#include "mfc_qos.h"

#include "mfc_core_hwlock.h"
#include "mfc_core_qos.h"
#include "mfc_core_reg_api.h"

#include "mfc_buf.h"
#include "mfc_sync.h"
#include "mfc_queue.h"
#include "mfc_mem.h"

static void __mfc_rm_request_butler(struct mfc_dev *dev, struct mfc_ctx *ctx)
{
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	int i;

	if (ctx) {
		/* maincore first if it is working */
		for (i = 0; i < MFC_CORE_TYPE_NUM; i++) {
			if (ctx->op_core_num[i] == MFC_CORE_INVALID)
				break;

			core = dev->core[ctx->op_core_num[i]];
			if (!core) {
				mfc_ctx_err("[RM] There is no core[%d]\n",
						ctx->op_core_num[i]);
				return;
			}

			core_ctx = core->core_ctx[ctx->num];
			if (mfc_ctx_ready_set_bit(core_ctx, &core->work_bits))
				core->core_ops->request_work(core,
						MFC_WORK_BUTLER, ctx);
		}
	} else {
		/* all of alive core */
		for (i = 0; i < dev->num_core; i++) {
			core = dev->core[i];
			if (!core) {
				mfc_dev_debug(2, "[RM] There is no core[%d]\n", i);
				continue;
			}

			core->core_ops->request_work(core, MFC_WORK_BUTLER, NULL);
		}
	}
}

static int __mfc_rm_get_core_num_by_load(struct mfc_dev *dev, struct mfc_ctx *ctx)
{
	struct mfc_core *core;
	int core_balance = dev->pdata->core_balance;
	int total_load[MFC_NUM_CORE];
	int core_num;
	int curr_load;
	unsigned long mb;

	core = dev->core[MFC_DEC_DEFAULT_CORE];
	total_load[MFC_DEC_DEFAULT_CORE] = core->total_mb * 100 / core->core_pdata->max_mb;
	core = dev->core[MFC_SURPLUS_CORE];
	total_load[MFC_SURPLUS_CORE] = core->total_mb * 100 / core->core_pdata->max_mb;

	if (ctx->ts_is_full)
		mb = ctx->weighted_mb;
	else
		mb = ctx->weighted_mb * MFC_MIN_FPS / ctx->framerate;
	curr_load = mb * 100 / core->core_pdata->max_mb;
	mfc_debug(2, "[RMLB] load%s fixed (curr mb: %ld, load: %d%%)\n",
			ctx->ts_is_full ? " " : " not", mb, curr_load);

	/* 1) Default core has not yet been balanced */
	if (total_load[MFC_DEC_DEFAULT_CORE] < core_balance) {
		if (total_load[MFC_DEC_DEFAULT_CORE] + curr_load <= core_balance) {
			core_num = MFC_DEC_DEFAULT_CORE;
			goto fix_core;
		}
	/* 2) Default core has been balanced */
	} else if ((total_load[MFC_DEC_DEFAULT_CORE] >= core_balance) &&
			(total_load[MFC_SURPLUS_CORE] < core_balance)) {
		core_num = MFC_SURPLUS_CORE;
		goto fix_core;
	}

	if (total_load[MFC_DEC_DEFAULT_CORE] > total_load[MFC_SURPLUS_CORE])
		core_num = MFC_SURPLUS_CORE;
	else
		core_num = MFC_DEC_DEFAULT_CORE;

fix_core:
	mfc_debug(2, "[RMLB] total load: [0] %ld(%d%%), [1] %ld(%d%%), curr_load: %ld(%d%%), select core: %d\n",
			dev->core[0]->total_mb, total_load[0],
			dev->core[1]->total_mb, total_load[1],
			mb, curr_load, core_num);
	MFC_TRACE_RM("[c:%d] load [0] %ld(%d) [1] %ld(%d) curr %ld(%d) select %d\n",
			ctx->num,
			dev->core[0]->total_mb, total_load[0],
			dev->core[1]->total_mb, total_load[1],
			mb, curr_load, core_num);

	return core_num;
}

static int __mfc_rm_get_core_num(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	int core_num;

	/* Default core according to standard */
	switch (ctx->codec_mode) {
	case MFC_REG_CODEC_AV1_DEC:
		ctx->op_core_type = MFC_OP_CORE_FIXED_1;
		core_num = 1;
		break;
	case MFC_REG_CODEC_H264_DEC:
	case MFC_REG_CODEC_H264_MVC_DEC:
	case MFC_REG_CODEC_HEVC_DEC:
		ctx->op_core_type = MFC_OP_CORE_ALL;
		core_num = MFC_DEC_DEFAULT_CORE;
		break;
	default:
		ctx->op_core_type = MFC_OP_CORE_FIXED_0;
		core_num = 0;
		break;
	}

	if (core_balance)
		dev->pdata->core_balance = core_balance;

	if (dev->pdata->core_balance == 100) {
		mfc_debug(4, "[RMLB] do not want to load balancing\n");
		return core_num;
	}

	/* Change core according to load */
	if (ctx->op_core_type == MFC_OP_CORE_ALL)
		core_num = __mfc_rm_get_core_num_by_load(dev, ctx);

	return core_num;
}

static int __mfc_rm_move_core_open(struct mfc_ctx *ctx, int to_core_num, int from_core_num)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *to_core = dev->core[to_core_num];
	struct mfc_core *from_core = dev->core[from_core_num];
	int ret = 0;

	mfc_ctx_info("[RMLB] open core changed MFC%d -> MFC%d\n",
			from_core_num, to_core_num);
	MFC_TRACE_RM("[c:%d] open core changed MFC%d -> MFC%d\n",
			ctx->num, from_core_num, to_core_num);

	ret = from_core->core_ops->instance_deinit(from_core, ctx);
	if (ret) {
		mfc_ctx_err("[RMLB] Failed to deinit\n");
		return ret;
	}

	ctx->op_core_num[MFC_CORE_MAIN] = to_core_num;

	ret = to_core->core_ops->instance_init(to_core, ctx);
	if (ret) {
		ctx->op_core_num[MFC_CORE_MAIN] = MFC_CORE_INVALID;
		mfc_ctx_err("[RMLB] Failed to init\n");
		return ret;
	}

	return ret;
}

static int __mfc_rm_move_core_running(struct mfc_ctx *ctx, int to_core_num, int from_core_num)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *to_core = dev->core[to_core_num];
	struct mfc_core *from_core = dev->core[from_core_num];
	struct mfc_core_ctx *core_ctx;
	int is_to_core = 0;
	int ret = 0;

	core_ctx = from_core->core_ctx[ctx->num];
	if (!core_ctx) {
		mfc_ctx_err("there is no core_ctx\n");
		mutex_unlock(&dev->mfc_migrate_mutex);
		return -EINVAL;
	}

	if (core_ctx->state != MFCINST_RUNNING) {
		mfc_debug(3, "[RMLB] it is not running state: %d\n", core_ctx->state);
		mutex_unlock(&dev->mfc_migrate_mutex);
		return -EAGAIN;
	}

	mfc_get_corelock_migrate(ctx);

	/* 1. Change state on from_core */
	ret = mfc_core_get_hwlock_dev_migrate(from_core, core_ctx);
	if (ret < 0) {
		mfc_ctx_err("Failed to get hwlock\n");
		mfc_release_corelock_migrate(ctx);
		mutex_unlock(&dev->mfc_migrate_mutex);
		return ret;
	}

	mutex_unlock(&dev->mfc_migrate_mutex);

	mfc_change_state(core_ctx, MFCINST_MOVE_INST);
	ctx->is_migration = 1;

	/* 2. Cache flush on to_core */
	ret = mfc_core_get_hwlock_dev(to_core);
	if (ret < 0) {
		mfc_ctx_err("Failed to get hwlock\n");
		goto err_migrate;
	}
	is_to_core = 1;

	ret = to_core->core_ops->instance_move_to(to_core, ctx);
	if (ret) {
		mfc_ctx_err("Failed to instance move init\n");
		goto err_migrate;
	}

	kfree(to_core->core_ctx[ctx->num]);
	ctx->op_core_num[MFC_CORE_MAIN] = MFC_CORE_INVALID;

	/* 3. Set F/W and ctx address on MFC1 */
	if (ctx->is_drm)
		mfc_core_set_migration_addr(dev->core[1], ctx, dev->core[0]->drm_fw_buf.daddr,
				dev->core[0]->drm_common_ctx_buf.daddr);
	else
		mfc_core_set_migration_addr(dev->core[1], ctx, dev->core[0]->fw_buf.daddr,
				dev->core[0]->common_ctx_buf.daddr);

	/* 4. Move and close instance on from_core */
	ret = from_core->core_ops->instance_move_from(from_core, ctx);
	if (ret) {
		mfc_ctx_err("Failed to instance move\n");
		MFC_TRACE_RM("[c:%d] instance_move_from fail\n", ctx->num);
		goto err_migrate;
	}
	mfc_debug(3, "[RMLB] move and close instance on from_core-%d\n", from_core->id);
	MFC_TRACE_RM("[c:%d] move and close inst_no %d\n", ctx->num, core_ctx->inst_no);

	ctx->op_core_num[MFC_CORE_MAIN] = to_core->id;
	to_core->core_ctx[ctx->num] = core_ctx;
	core_ctx->core = to_core;

	mfc_clear_bit(ctx->num, &from_core->work_bits);
	from_core->core_ctx[core_ctx->num] = 0;

	mfc_core_move_hwlock_ctx(to_core, from_core, core_ctx);
	mfc_core_release_hwlock_dev(to_core);
	mfc_core_release_hwlock_dev(from_core);

	mfc_change_state(core_ctx, MFCINST_RUNNING);
	mfc_core_qos_on(to_core, ctx);

	ctx->is_migration = 0;
	mfc_wake_up_ctx_migrate(ctx);

	mfc_release_corelock_migrate(ctx);

	mfc_debug(2, "[RMLB] ctx[%d] migration finished. op_core:%d \n", ctx->num, to_core->id);

	__mfc_rm_request_butler(dev, ctx);

	return 0;

err_migrate:
	mfc_change_state(core_ctx, MFCINST_RUNNING);
	mfc_core_release_hwlock_dev(from_core);
	if (is_to_core)
		mfc_core_release_hwlock_dev(to_core);

	ctx->is_migration = 0;
	mfc_wake_up_ctx_migrate(ctx);

	mfc_release_corelock_migrate(ctx);

	return ret;
}

static struct mfc_core *__mfc_rm_switch_to_single_mode(struct mfc_ctx *ctx, int need_cpb_lock)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *maincore;
	struct mfc_core *subcore;
	struct mfc_core_ctx *core_ctx;
	struct mfc_buf *src_mb = NULL;
	int last_op_core;
	int ret;

	maincore = mfc_get_main_core(ctx->dev, ctx);
	if (!maincore) {
		mfc_ctx_err("[RM] There is no maincore\n");
		return NULL;
	}

	subcore = mfc_get_sub_core(ctx->dev, ctx);
	if (!subcore) {
		mfc_ctx_err("[RM] There is no subcore for switch single\n");
		return NULL;
	}

	ret = mfc_core_get_hwlock_dev(maincore);
	if (ret < 0) {
		mfc_ctx_err("Failed to get maincore hwlock\n");
		return NULL;
	}

	ret = mfc_core_get_hwlock_dev(subcore);
	if (ret < 0) {
		mfc_ctx_err("Failed to get subcore hwlock\n");
		mfc_core_release_hwlock_dev(maincore);
		return NULL;
	}

	if (need_cpb_lock)
		mutex_lock(&ctx->cpb_mutex);
	mfc_change_op_mode(ctx, MFC_OP_SWITCHING);

	/* maincore needs to cleanup src buffer */
	core_ctx = maincore->core_ctx[ctx->num];
	mfc_move_buf_all(ctx, &ctx->src_buf_ready_queue,
			&core_ctx->src_buf_queue, MFC_QUEUE_ADD_TOP);
	mfc_init_queue(&core_ctx->src_buf_queue);
	MFC_TRACE_RM("[c:%d] MODE 2->4: Move src all\n", ctx->num);

	/* subcore should have one src buffer */
	core_ctx = subcore->core_ctx[ctx->num];
	if (mfc_get_queue_count(&ctx->buf_queue_lock, &core_ctx->src_buf_queue) == 0) {
		src_mb = mfc_get_move_buf(ctx, &core_ctx->src_buf_queue,
				&ctx->src_buf_ready_queue,
				MFC_BUF_NO_TOUCH_USED, MFC_QUEUE_ADD_BOTTOM);
		if (src_mb) {
			mfc_debug(2, "[RM][BUFINFO] MFC-%d uses src index: %d(%d)\n",
					subcore->id, src_mb->vb.vb2_buf.index,
					src_mb->src_index);
			MFC_TRACE_RM("[c:%d] MFC-%d uses src index: %d(%d)\n",
					ctx->num, subcore->id, src_mb->vb.vb2_buf.index,
					src_mb->src_index);
		}
	} else {
		mfc_debug(2, "[RM][BUFINFO] MFC-%d has src buffer already\n", subcore->id);
	}

	if (core_ctx->state == MFCINST_FINISHING)
		mfc_change_state(core_ctx, MFCINST_RUNNING);

	/* Change done, it will be work with switch_to_single mode */
	mfc_change_op_mode(ctx, MFC_OP_SWITCH_TO_SINGLE);

	last_op_core = ctx->curr_src_index % ctx->dev->num_core;
	if (last_op_core == 0) {
		mfc_debug(2, "[RMLB] last op core%d, it should operate once with mode2\n",
				last_op_core);
		mfc_change_op_mode(ctx, MFC_OP_SWITCH_BUT_MODE2);
	}

	if (need_cpb_lock)
		mutex_unlock(&ctx->cpb_mutex);

	/* If it is switched to single, interrupt lock is not needed. */
	ctx->intlock.bits = 0;

	mfc_core_release_hwlock_dev(maincore);
	mfc_core_release_hwlock_dev(subcore);
	mfc_core_qos_off(maincore, ctx);
	mfc_core_qos_on(subcore, ctx);

	return subcore;
}

static int __mfc_rm_check_multi_core_mode(struct mfc_dev *dev)
{
	struct mfc_core *core = NULL;
	struct mfc_core_ctx *core_ctx = NULL;
	struct mfc_ctx *ctx = NULL;
	int i;

	for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
		if (test_bit(i, &dev->multi_core_inst_bits)) {
			MFC_TRACE_RM("[c:%d] multi core instance\n", i);
			ctx = dev->ctx[i];
			if (!ctx) {
				mfc_dev_err("[RM] There is no ctx\n");
				continue;
			}

			if (!IS_TWO_MODE2(ctx)) {
				mfc_debug(3, "[RM] mode1 or already switched to single\n");
				continue;
			}

			if (!mfc_rm_query_state(ctx, EQUAL, MFCINST_RUNNING)) {
				mfc_debug(2, "[RM] mode2 but setup of 2core is not yet done\n");
				continue;
			}

			/* Mode2 instance should be switch to single mode */
			core = __mfc_rm_switch_to_single_mode(ctx, 1);
			if (!core)
				return -EINVAL;

			mfc_debug(2, "[RM][2CORE] switch single for multi instance op_mode: %d\n",
					ctx->op_mode);
			MFC_TRACE_RM("[c:%d] switch to single for multi inst\n", i);

			core_ctx = core->core_ctx[ctx->num];
			if (mfc_ctx_ready_set_bit(core_ctx, &core->work_bits))
				core->core_ops->request_work(core, MFC_WORK_BUTLER, ctx);
		}
	}

	return 0;
}

static void __mfc_rm_move_buf_ready_set_bit(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core = NULL;
	struct mfc_core_ctx *core_ctx = NULL;
	struct mfc_buf *src_mb = NULL;
	unsigned int num;

	mutex_lock(&ctx->cpb_mutex);

	/* search for next running core */
	src_mb = mfc_get_buf(ctx, &ctx->src_buf_ready_queue,
			MFC_BUF_NO_TOUCH_USED);
	if (!src_mb) {
		mfc_debug(3, "[RM][2CORE] there is no src buffer\n");
		mutex_unlock(&ctx->cpb_mutex);
		return;
	}

	/* handle last frame with switch_to_single mode */
	if (mfc_check_mb_flag(src_mb, MFC_FLAG_LAST_FRAME)) {
		ctx->serial_src_index = 0;
		mfc_debug(2, "[RM][2CORE] EOS, reset serial src index\n");
		MFC_TRACE_RM("[c:%d] EOS: Reset serial src index\n", ctx->num);

		if (ctx->curr_src_index < src_mb->src_index -1) {
			mfc_debug(2, "[RM][2CORE] waiting src index %d, curr src index %d is working\n",
					src_mb->src_index,
					ctx->curr_src_index);
			mutex_unlock(&ctx->cpb_mutex);
			return;
		}

		core = __mfc_rm_switch_to_single_mode(ctx, 0);
		if (!core) {
			mutex_unlock(&ctx->cpb_mutex);
			return;
		} else {
			mfc_debug(2, "[RM][2CORE] switch single for LAST FRAME(EOS) op_mode: %d\n",
					ctx->op_mode);
		}
		goto butler;
	}

	if (ctx->curr_src_index == src_mb->src_index - 1) {
		num = src_mb->src_index % dev->num_core;
		core = dev->core[num];
		mfc_debug(2, "[RM][2CORE] src index %d(%d) run in MFC-%d, curr: %d\n",
				src_mb->vb.vb2_buf.index,
				src_mb->src_index, num,
				ctx->curr_src_index);
	} else {
		mfc_debug(2, "[RM][2CORE] waiting src index %d, curr src index %d is working\n",
				src_mb->src_index,
				ctx->curr_src_index);
		goto butler;
	}

	/* move src buffer to src_buf_queue from src_buf_ready_queue */
	core_ctx = core->core_ctx[ctx->num];
	src_mb = mfc_get_move_buf(ctx, &core_ctx->src_buf_queue,
			&ctx->src_buf_ready_queue,
			MFC_BUF_NO_TOUCH_USED, MFC_QUEUE_ADD_BOTTOM);
	if (src_mb) {
		mfc_debug(2, "[RM][BUFINFO] MFC-%d uses src index: %d(%d)\n",
				core->id, src_mb->vb.vb2_buf.index,
				src_mb->src_index);
		MFC_TRACE_RM("[c:%d] READY: Move src[%d] to MFC-%d\n",
				ctx->num, src_mb->src_index, core->id);
	}

butler:
	mutex_unlock(&ctx->cpb_mutex);
	__mfc_rm_request_butler(dev, ctx);
}

static void __mfc_rm_guarantee_init_buf(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *maincore;
	struct mfc_core *subcore;
	struct mfc_core_ctx *core_ctx;
	int ret;

	/* maincore ready */
	maincore = mfc_get_main_core(ctx->dev, ctx);
	if (!maincore) {
		mfc_ctx_err("[RM] There is no maincore\n");
		return;
	}
	core_ctx = maincore->core_ctx[ctx->num];
	if (!mfc_ctx_ready_set_bit(core_ctx, &maincore->work_bits))
		return;

	/* subcore ready */
	subcore = mfc_get_sub_core(ctx->dev, ctx);
	if (!subcore) {
		mfc_ctx_err("[RM] There is no subcore for switch single\n");
		return;
	}
	core_ctx = subcore->core_ctx[ctx->num];
	if (!mfc_ctx_ready_set_bit(core_ctx, &subcore->work_bits))
		return;

	/*
	 * No other command should be sent
	 * while sending INIT_BUFFER command to 2 core in mode2
	 */
	ret = mfc_core_get_hwlock_dev(maincore);
	if (ret < 0) {
		mfc_ctx_err("Failed to get maincore hwlock\n");
		return;
	}

	ret = mfc_core_get_hwlock_dev(subcore);
	if (ret < 0) {
		mfc_ctx_err("Failed to get subcore hwlock\n");
		mfc_core_release_hwlock_dev(maincore);
		return;
	}

	/*
	 * If Normal <-> Secure switch,
	 * subcore core need to cache flush without other command.
	 */
	if (IS_TWO_MODE1(ctx)) {
		if (subcore->curr_core_ctx_is_drm != ctx->is_drm) {
			mfc_debug(2, "[RM] subcore need to cache flush for op_mode 1\n");
			subcore->core_ops->instance_cache_flush(subcore, ctx);
		}
	}

	MFC_TRACE_RM("[c:%d] mode2 try INIT_BUFFER\n", ctx->num);
	mfc_debug(3, "[RM] mode2 try INIT_BUFFER\n");
	ret = maincore->core_ops->instance_init_buf(maincore, ctx);
	if (ret < 0) {
		mfc_ctx_err("failed maincore init buffer\n");
		mfc_core_release_hwlock_dev(maincore);
		mfc_core_release_hwlock_dev(subcore);
		return;
	}

	ret = subcore->core_ops->instance_init_buf(subcore, ctx);
	if (ret < 0) {
		mfc_ctx_err("failed subcore init buffer\n");
		mfc_core_release_hwlock_dev(maincore);
		mfc_core_release_hwlock_dev(subcore);
		return;
	}

	mfc_core_release_hwlock_dev(maincore);
	mfc_core_release_hwlock_dev(subcore);

	mfc_debug(2, "[RM][2CORE] mode2 setup done, check multi inst\n");
	MFC_TRACE_RM("[c:%d] mode2 setup done\n", ctx->num);
	if (dev->num_inst > 1)
		__mfc_rm_check_multi_core_mode(dev);
}

static void __mfc_rm_move_buf_request_work(struct mfc_ctx *ctx, enum mfc_request_work work)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	int i;

	if (mfc_rm_query_state(ctx, EQUAL, MFCINST_RUNNING)) {
		/* search for next running core and running */
		__mfc_rm_move_buf_ready_set_bit(ctx);
	} else if (mfc_rm_query_state(ctx, EQUAL, MFCINST_HEAD_PARSED)) {
		__mfc_rm_guarantee_init_buf(ctx);
	} else {
		/*
		 * If it is not RUNNING,
		 * each core can work a given job individually.
		 */
		for (i = 0; i < MFC_CORE_TYPE_NUM; i++) {
			if (ctx->op_core_num[i] == MFC_CORE_INVALID)
				break;

			core = dev->core[ctx->op_core_num[i]];
			if (!core) {
				mfc_ctx_err("[RM] There is no core[%d]\n",
						ctx->op_core_num[i]);
				return;
			}
			mfc_debug(2, "[RM] request work to MFC-%d\n", core->id);

			core_ctx = core->core_ctx[ctx->num];
			if (!core_ctx || (core_ctx && core_ctx->state < MFCINST_GOT_INST))
				continue;

			if (mfc_ctx_ready_set_bit(core_ctx, &core->work_bits))
				if (core->core_ops->request_work(core, work, ctx))
					mfc_debug(3, "[RM] failed to request_work\n");
		}
	}

	return;
}

static int __mfc_rm_switch_to_multi_mode(struct mfc_ctx *ctx)
{
	struct mfc_core *maincore;
	struct mfc_core *subcore;
	struct mfc_core_ctx *core_ctx;
	struct mfc_buf *src_mb;
	unsigned long flags;
	int ret;

	maincore = mfc_get_main_core(ctx->dev, ctx);
	if (!maincore) {
		mfc_ctx_err("[RM] There is no maincore\n");
		return -EINVAL;
	}

	subcore = mfc_get_sub_core(ctx->dev, ctx);
	if (!subcore) {
		mfc_ctx_err("[RM] There is no subcore for switch single\n");
		return -EINVAL;
	}

	ret = mfc_core_get_hwlock_dev(maincore);
	if (ret < 0) {
		mfc_ctx_err("Failed to get maincore hwlock\n");
		return -EINVAL;
	}

	ret = mfc_core_get_hwlock_dev(subcore);
	if (ret < 0) {
		mfc_ctx_err("Failed to get subcore hwlock\n");
		mfc_core_release_hwlock_dev(maincore);
		return -EINVAL;
	}

	if (ctx->op_mode == MFC_OP_SWITCH_BUT_MODE2) {
		mfc_debug(2, "[RMLB] just go to mode2\n");
	} else {
		mfc_change_op_mode(ctx, MFC_OP_SWITCHING);

		/* re-arrangement cpb for mode2 */
		mutex_lock(&ctx->cpb_mutex);
		core_ctx = subcore->core_ctx[ctx->num];
		mfc_move_buf_all(ctx, &ctx->src_buf_ready_queue,
				&core_ctx->src_buf_queue, MFC_QUEUE_ADD_TOP);

		ctx->serial_src_index = 0;
		ctx->curr_src_index = -1;

		spin_lock_irqsave(&ctx->buf_queue_lock, flags);
		if (!list_empty(&ctx->src_buf_ready_queue.head)) {
			list_for_each_entry(src_mb, &ctx->src_buf_ready_queue.head, list) {
				if (src_mb) {
					mfc_debug(2, "[RM][2CORE] src index(%d) changed to %d\n",
							src_mb->src_index, ctx->serial_src_index);
					src_mb->src_index = ctx->serial_src_index++;
				}
			}
		}
		spin_unlock_irqrestore(&ctx->buf_queue_lock, flags);

		mutex_unlock(&ctx->cpb_mutex);
	}

	/* Change done, it will be work with mode2 */
	mfc_change_op_mode(ctx, MFC_OP_TWO_MODE2);

	mfc_core_release_hwlock_dev(maincore);
	mfc_core_release_hwlock_dev(subcore);
	mfc_core_qos_on(maincore, ctx);
	mfc_core_qos_on(subcore, ctx);

	__mfc_rm_move_buf_ready_set_bit(ctx);

	return 0;
}

void mfc_rm_migration_worker(struct work_struct *work)
{
	struct mfc_dev *dev;
	struct mfc_ctx *ctx;
	int to_core_num, from_core_num;
	int i, ret = 0;

	dev = container_of(work, struct mfc_dev, migration_work);

	for (i = 0; i < dev->move_ctx_cnt; i++) {
		mutex_lock(&dev->mfc_migrate_mutex);
		ctx = dev->move_ctx[i];
		dev->move_ctx[i] = NULL;

		/*
		 * If one instance fails migration,
		 * the rest of instnaces will not migrate.
		 */
		if (ret || !ctx) {
			MFC_TRACE_RM("migration fail\n");
			mutex_unlock(&dev->mfc_migrate_mutex);
			continue;
		}

		if (IS_SWITCH_SINGLE_MODE(ctx)) {
			mutex_unlock(&dev->mfc_migrate_mutex);
			mfc_debug(2, "[RMLB][2CORE] ctx[%d] will change op_mode: %d -> 2\n",
					ctx->num, ctx->op_mode);
			MFC_TRACE_RM("[c:%d] will change op_mode: %d -> 2\n",
					ctx->num, ctx->op_mode);
			ret = __mfc_rm_switch_to_multi_mode(ctx);
			if (dev->move_ctx_cnt > 1) {
				mfc_ctx_err("[RMLB] there shouldn't be another instance because of mode2\n");
				MFC_TRACE_RM("[c:%d] no another inst for mode2\n", ctx->num);
				ret = -EINVAL;
			}
			continue;
		}

		from_core_num = ctx->op_core_num[MFC_CORE_MAIN];
		to_core_num = ctx->move_core_num[MFC_CORE_MAIN];
		mfc_debug(2, "[RMLB] ctx[%d] will be moved MFC%d -> MFC%d\n",
				ctx->num, from_core_num, to_core_num);
		MFC_TRACE_RM("[c:%d] will be moved MFC%d -> MFC%d\n",
				ctx->num, from_core_num, to_core_num);
		ret = __mfc_rm_move_core_running(ctx, to_core_num, from_core_num);
		if (ret) {
			mfc_ctx_info("[RMLB] migration stopped by ctx[%d]\n",
					ctx->num);
			MFC_TRACE_RM("migration fail by ctx[%d]\n", ctx->num);
			continue;
		}
	}

	mfc_dev_debug(2, "[RMLB] all instance migration finished\n");
	dev->move_ctx_cnt = 0;

	__mfc_rm_request_butler(dev, NULL);
}

static int __mfc_rm_load_delete(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_ctx *mfc_ctx, *tmp_ctx;

	list_for_each_entry_safe(mfc_ctx, tmp_ctx, &dev->ctx_list, list) {
		if (mfc_ctx == ctx) {
			list_del(&mfc_ctx->list);
			mfc_debug(3, "[RMLB] ctx[%d] is deleted from list\n", ctx->num);
			MFC_TRACE_RM("[c:%d] load delete\n", ctx->num);
			return 0;
		}
	}

	return 1;
}

static int __mfc_rm_load_add(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_ctx *tmp_ctx;

	/* list create */
	if (list_empty(&dev->ctx_list)) {
		list_add(&ctx->list, &dev->ctx_list);
		MFC_TRACE_RM("[c:%d] load add first\n", ctx->num);
		return 1;
	}

	/* If ctx has already added, delete for reordering */
	__mfc_rm_load_delete(ctx);

	/* dev->ctx_list is aligned in descending order of load */
	list_for_each_entry_reverse(tmp_ctx, &dev->ctx_list, list) {
		if (tmp_ctx->weighted_mb > ctx->weighted_mb) {
			list_add(&ctx->list, &tmp_ctx->list);
			mfc_debug(3, "[RMLB] ctx[%d] is added to list\n", ctx->num);
			MFC_TRACE_RM("[c:%d] load add\n", ctx->num);
			return 0;
		}
	}

	/* add to the front of dev->list */
	list_add(&ctx->list, &dev->ctx_list);
	mfc_debug(3, "[RMLB] ctx[%d] is added to list\n", ctx->num);
	MFC_TRACE_RM("[c:%d] load add\n", ctx->num);

	return 0;
}

void mfc_rm_load_balancing(struct mfc_ctx *ctx, int load_add)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	struct mfc_platdata *pdata = dev->pdata;
	struct mfc_ctx *tmp_ctx;
	unsigned long flags;
	int i, core_num, ret = 0;

	if (dev->pdata->core_balance == 100) {
		mfc_debug(4, "[RMLB] do not want to load balancing\n");
		return;
	}

	spin_lock_irqsave(&dev->ctx_list_lock, flags);
	if (load_add == MFC_RM_LOAD_ADD)
		ret = __mfc_rm_load_add(ctx);
	else
		ret = __mfc_rm_load_delete(ctx);
	if (ret) {
		spin_unlock_irqrestore(&dev->ctx_list_lock, flags);
		return;
	}

	/* check the MFC IOVA and control lazy unmap */
	mfc_check_iova(dev);

	if (!ctx->ts_is_full && load_add == MFC_RM_LOAD_ADD) {
		mfc_debug(2, "[RMLB] instance load is not yet fixed\n");
		spin_unlock_irqrestore(&dev->ctx_list_lock, flags);
		return;
	}

	if (dev->move_ctx_cnt || (load_add == MFC_RM_LOAD_DELETE) ||
			(dev->num_inst == 1 && load_add == MFC_RM_LOAD_ADD)) {
		mfc_debug(4, "[RMLB] instance migration isn't need (num_inst: %d, move_ctx: %d)\n",
				dev->num_inst, dev->move_ctx_cnt);
		spin_unlock_irqrestore(&dev->ctx_list_lock, flags);
		return;
	}

	/* Clear total mb each core for load re-calculation */
	for (i = 0; i < dev->num_core; i++)
		dev->core[i]->total_mb = 0;

	/* Load calculation of instnace with fixed core */
	list_for_each_entry(tmp_ctx, &dev->ctx_list, list)
		if (tmp_ctx->op_core_type != MFC_OP_CORE_ALL)
			dev->core[tmp_ctx->op_core_type]->total_mb += tmp_ctx->weighted_mb;

	/* Load balancing of instance with not-fixed core */
	list_for_each_entry(tmp_ctx, &dev->ctx_list, list) {
		/* need to fix core */
		if (tmp_ctx->op_core_type == MFC_OP_CORE_ALL) {
			core_num = __mfc_rm_get_core_num_by_load(dev, tmp_ctx);
			if (IS_MULTI_MODE(tmp_ctx)) {
				core = mfc_get_main_core(dev, tmp_ctx);
				core->total_mb += tmp_ctx->weighted_mb;
				core = mfc_get_sub_core(dev, tmp_ctx);
				core->total_mb += tmp_ctx->weighted_mb;
				mfc_debug(3, "[RMLB] ctx[%d] fix load both core\n",
						tmp_ctx->num);
				MFC_TRACE_RM("[c:%d] fix load both core\n", tmp_ctx->num);
				continue;
			} else if (IS_SWITCH_SINGLE_MODE(tmp_ctx)) {
				if (dev->num_inst == 1) {
					mfc_debug(2, "[RMLB] ctx[%d] can be changed to mode2\n",
							tmp_ctx->num);
					MFC_TRACE_RM("[c:%d] can be changed to mode2\n",
							tmp_ctx->num);
					dev->move_ctx[dev->move_ctx_cnt++] = tmp_ctx;
				}
				core = mfc_get_sub_core(dev, tmp_ctx);
				core->total_mb += tmp_ctx->weighted_mb;
				mfc_debug(3, "[RMLB] ctx[%d] fix load subcore\n",
						tmp_ctx->num);
				MFC_TRACE_RM("[c:%d] fix load subcore\n",
						tmp_ctx->num);
				continue;
			}
			if (core_num == tmp_ctx->op_core_num[MFC_CORE_MAIN]) {
				/* Already select correct core */
				mfc_debug(3, "[RMLB] ctx[%d] keep core%d\n",
						tmp_ctx->num,
						tmp_ctx->op_core_num[MFC_CORE_MAIN]);
				dev->core[core_num]->total_mb += tmp_ctx->weighted_mb;
				continue;
			} else {
				/* Instance should move */
				mfc_debug(3, "[RMLB] ctx[%d] move to core%d\n",
						tmp_ctx->num, core_num);
				MFC_TRACE_RM("[c:%d] move to core%d\n",
						tmp_ctx->num, core_num);
				dev->core[core_num]->total_mb += tmp_ctx->weighted_mb;
				tmp_ctx->move_core_num[MFC_CORE_MAIN] = core_num;
				dev->move_ctx[dev->move_ctx_cnt++] = tmp_ctx;
				continue;
			}
		}
	}

	/* For debugging */
	mfc_debug(3, "[RMLB] ===================ctx list===================\n");
	list_for_each_entry(tmp_ctx, &dev->ctx_list, list)
		mfc_debug(3, "[RMLB] MFC-%d) ctx[%d] %s %dx%d %lufps %s, load: %d%%, op_core_type: %d, op_mode: %d\n",
				IS_SWITCH_SINGLE_MODE(tmp_ctx) ? tmp_ctx->op_core_num[MFC_CORE_SUB]
				: tmp_ctx->op_core_num[MFC_CORE_MAIN], tmp_ctx->num,
				tmp_ctx->type == MFCINST_DECODER ? "DEC" : "ENC",
				tmp_ctx->crop_width, tmp_ctx->crop_height,
				tmp_ctx->framerate / 1000, tmp_ctx->type == MFCINST_DECODER ?
				tmp_ctx->src_fmt->name : tmp_ctx->dst_fmt->name,
				tmp_ctx->load, tmp_ctx->op_core_type, tmp_ctx->op_mode);
	mfc_debug(3, "[RMLB] >>>> core balance %d%%\n", pdata->core_balance);
	for (i = 0; i < dev->num_core; i++)
		mfc_debug(3, "[RMLB] >> MFC-%d total load: %lu%%\n", i,
				dev->core[i]->total_mb * 100 / dev->core[i]->core_pdata->max_mb);
	mfc_debug(3, "[RMLB] ==============================================\n");

	spin_unlock_irqrestore(&dev->ctx_list_lock, flags);

	if (dev->move_ctx_cnt)
		queue_work(dev->migration_wq, &dev->migration_work);

	return;
}

int mfc_rm_instance_init(struct mfc_dev *dev, struct mfc_ctx *ctx)
{
	struct mfc_core *core;
	int i, ret;

	mfc_debug_enter();

	mfc_get_corelock_ctx(ctx);

	/*
	 * The FW memory for all cores is allocated in advance.
	 * (Only once at first time)
	 * Because FW base address should be the lowest address
	 * than all DVA that FW approaches.
	 */
	for (i = 0; i < dev->num_core; i++) {
		core = dev->core[i];
		if (!core) {
			mfc_ctx_err("[RM] There is no MFC-%d\n", i);
			continue;
		}

		if (!(core->fw.status & MFC_FW_ALLOC)) {
			ret = mfc_alloc_firmware(core);
			if (ret)
				goto err_inst_init;
		}

		if (!(core->fw.status & MFC_CTX_ALLOC)) {
			ret = mfc_alloc_common_context(core);
			if (ret)
				goto err_inst_init;
		}
	}

	mfc_change_op_mode(ctx, MFC_OP_SINGLE);
	ctx->op_core_type = MFC_OP_CORE_NOT_FIXED;
	if (ctx->type == MFCINST_DECODER)
		ctx->op_core_num[MFC_CORE_MAIN] = MFC_DEC_DEFAULT_CORE;
	else
		ctx->op_core_num[MFC_CORE_MAIN] = MFC_ENC_DEFAULT_CORE;

	core = mfc_get_main_core(dev, ctx);
	if (!core) {
		mfc_ctx_err("[RM] There is no maincore\n");
		ret = -EINVAL;
		goto err_inst_init;
	}

	mfc_debug(2, "[RM] init instance core-%d\n",
			ctx->op_core_num[MFC_CORE_MAIN]);

	atomic_inc(&core->during_idle_resume);
	/* Trigger idle resume if core is in the idle mode for initializing hardware */
	mfc_rm_qos_control(ctx, MFC_QOS_TRIGGER);

	ret = core->core_ops->instance_init(core, ctx);
	if (ret) {
		ctx->op_core_num[MFC_CORE_MAIN] = MFC_CORE_INVALID;
		mfc_ctx_err("[RM] Failed to init\n");
	}

	atomic_dec(&core->during_idle_resume);
err_inst_init:
	mfc_release_corelock_ctx(ctx);

	mfc_debug_leave();

	return ret;
}

int mfc_rm_instance_deinit(struct mfc_dev *dev, struct mfc_ctx *ctx)
{
	struct mfc_core *core;
	int i, ret = 0;

	mfc_debug_enter();

	mfc_get_corelock_ctx(ctx);

	for (i = 0; i < MFC_CORE_TYPE_NUM; i++) {
		if (ctx->op_core_num[i] == MFC_CORE_INVALID)
			break;

		core = dev->core[ctx->op_core_num[i]];
		if (!core) {
			mfc_ctx_err("[RM] There is no core[%d]\n",
					ctx->op_core_num[i]);
			ret = -EINVAL;
			goto err_inst_deinit;
		}

		mfc_core_debug(2, "[RM] core%d will be deinit, ctx[%d]\n",
				i, ctx->num);
		ret = core->core_ops->instance_deinit(core, ctx);
		if (ret) {
			mfc_core_err("[RM] Failed to deinit\n");
		}

		ctx->op_core_num[i] = MFC_CORE_INVALID;
	}

	clear_bit(ctx->num, &dev->multi_core_inst_bits);
	mfc_change_op_mode(ctx, MFC_OP_SINGLE);
	ctx->op_core_type = MFC_OP_CORE_NOT_FIXED;

err_inst_deinit:
	mfc_release_corelock_ctx(ctx);

	mfc_debug_leave();

	return ret;
}

int mfc_rm_instance_open(struct mfc_dev *dev, struct mfc_ctx *ctx)
{
	struct mfc_core *core;
	int ret = 0, core_num;
	int is_corelock = 0;

	mfc_debug_enter();

	core = mfc_get_main_core(dev, ctx);
	if (!core) {
		mfc_ctx_err("[RM] There is no maincore\n");
		ret = -EINVAL;
		goto err_inst_open;
	}

	if (IS_MULTI_CORE_DEVICE(dev)) {
		/*
		 * When there is instance of multi core mode,
		 * other instance should be open in MFC-0
		 */
		ret = __mfc_rm_check_multi_core_mode(dev);
		if (ret < 0) {
			mfc_ctx_err("[RM] failed multi core instance switching\n");
			goto err_inst_open;
		}

		mfc_get_corelock_ctx(ctx);
		is_corelock = 1;

		/* Core balance by both standard and load */
		core_num = __mfc_rm_get_core_num(ctx);
		if (core_num != core->id) {
			ret = __mfc_rm_move_core_open(ctx, core_num, core->id);
			if (ret)
				goto err_inst_open;

			core = mfc_get_main_core(dev, ctx);
			if (!core) {
				mfc_ctx_err("[RM] There is no maincore\n");
				ret = -EINVAL;
				goto err_inst_open;
			}
		}
	}

	ret = core->core_ops->instance_open(core, ctx);
	if (ret) {
		mfc_core_err("[RM] Failed to open\n");
		goto err_inst_open;
	}

err_inst_open:
	if (is_corelock)
		mfc_release_corelock_ctx(ctx);

	mfc_debug_leave();

	return ret;
}

static void __mfc_rm_inst_dec_dst_stop(struct mfc_dev *dev, struct mfc_ctx *ctx)
{
	struct mfc_core *core;
	int i;

	mfc_debug(3, "op_mode: %d\n", ctx->op_mode);

	mfc_get_corelock_ctx(ctx);

	if (IS_TWO_MODE2(ctx) || IS_SWITCH_SINGLE_MODE(ctx)) {
		for (i = 0; i < MFC_CORE_TYPE_NUM; i++) {
			if (ctx->op_core_num[i] == MFC_CORE_INVALID)
				goto err_dst_stop;

			core = dev->core[ctx->op_core_num[i]];
			if (!core) {
				mfc_ctx_err("[RM] There is no core[%d]\n",
						ctx->op_core_num[i]);
				goto err_dst_stop;
			}

			mfc_core_debug(2, "[RM] core%d will be DPB flush, ctx[%d]\n",
					i, ctx->num);
			core->core_ops->instance_dpb_flush(core, ctx);
		}
	} else {

		core = mfc_get_main_core(dev, ctx);
		if (!core) {
			mfc_ctx_err("[RM] There is no maincore\n");
			goto err_dst_stop;
		}

		core->core_ops->instance_dpb_flush(core, ctx);

	}

	ctx->intlock.bits = 0;

err_dst_stop:
	mfc_release_corelock_ctx(ctx);
}

static void __mfc_rm_inst_dec_src_stop(struct mfc_dev *dev, struct mfc_ctx *ctx)
{
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	enum mfc_op_mode prev_op_mode = ctx->op_mode;

	mfc_debug(2, "op_mode: %d\n", ctx->op_mode);

	mfc_get_corelock_ctx(ctx);

	if (IS_TWO_MODE2(ctx)) {
		core = __mfc_rm_switch_to_single_mode(ctx, 1);
		if (!core)
			goto err_src_stop;
		else
			mfc_debug(2, "[RM][2CORE] switch single for CSD parsing op_mode: %d\n",
					ctx->op_mode);
	} else if (IS_SWITCH_SINGLE_MODE(ctx)) {
		core = mfc_get_sub_core(dev, ctx);
		if (!core) {
			mfc_ctx_err("[RM] There is no subcore for switch single\n");
			goto err_src_stop;
		}
	} else {
		core = mfc_get_main_core(dev, ctx);
		if (!core) {
			mfc_ctx_err("[RM] There is no maincore\n");
			goto err_src_stop;
		}
	}

	core_ctx = core->core_ctx[ctx->num];
	core->core_ops->instance_csd_parsing(core, ctx);

	mfc_change_op_mode(ctx, prev_op_mode);

err_src_stop:
	mfc_release_corelock_ctx(ctx);
}

void mfc_rm_instance_dec_stop(struct mfc_dev *dev, struct mfc_ctx *ctx,
			unsigned int type)
{

	mfc_debug_enter();

	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		__mfc_rm_inst_dec_dst_stop(dev, ctx);
	else if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		__mfc_rm_inst_dec_src_stop(dev, ctx);

	mfc_debug_leave();
}

void mfc_rm_instance_enc_stop(struct mfc_dev *dev, struct mfc_ctx *ctx,
			unsigned int type)
{
	struct mfc_core *core;

	mfc_debug_enter();

	mfc_get_corelock_ctx(ctx);

	core = mfc_get_main_core(dev, ctx);
	if (!core) {
		mfc_ctx_err("[RM] There is no maincore\n");
		mfc_release_corelock_ctx(ctx);
		return;
	}

	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		core->core_ops->instance_q_flush(core, ctx);
	else if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		core->core_ops->instance_finishing(core, ctx);

	mfc_release_corelock_ctx(ctx);

	mfc_debug_leave();
}

int mfc_rm_instance_setup(struct mfc_dev *dev, struct mfc_ctx *ctx)
{
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	struct mfc_buf *src_mb = NULL;
	int sub_core_num;
	int ret = 0;

	if (!IS_MULTI_MODE(ctx)) {
		mfc_ctx_info("[RM] do not need to subcore setup\n");
		return 0;
	}

	if (ctx->op_core_num[MFC_CORE_SUB] != MFC_CORE_INVALID) {
		mfc_ctx_info("[RM] subcore already setup\n");
		return 0;
	}

	/* When instance setup, subcore num is 1 */
	if (ctx->op_core_num[MFC_CORE_MAIN] == MFC_DEC_DEFAULT_CORE)
		sub_core_num = MFC_SURPLUS_CORE;
	else
		sub_core_num = MFC_DEC_DEFAULT_CORE;
	ctx->op_core_num[MFC_CORE_SUB] = sub_core_num;
	core = mfc_get_sub_core(dev, ctx);
	if (!core) {
		mfc_ctx_err("[RM] There is no subcore\n");
		return -EINVAL;
	}

	ret = core->core_ops->instance_init(core, ctx);
	if (ret) {
		ctx->op_core_num[MFC_CORE_SUB] = MFC_CORE_INVALID;
		mfc_ctx_err("[RM] subcore init failed\n");
		goto end_setup;
	}
	core_ctx = core->core_ctx[ctx->num];

	ret = core->core_ops->instance_open(core, ctx);
	if (ret) {
		mfc_ctx_err("[RM] subcore open failed\n");
		goto end_setup;
	}

	ctx->subcore_inst_no = core_ctx->inst_no;
	mfc_debug(2, "[RM] subcore setup inst_no: %d\n", ctx->subcore_inst_no);

	/* Move the header buffer to subcore */
	src_mb = mfc_get_move_buf(ctx, &core_ctx->src_buf_queue,
			&ctx->src_buf_ready_queue,
			MFC_BUF_NO_TOUCH_USED, MFC_QUEUE_ADD_TOP);
	if (!src_mb)
		mfc_ctx_err("[RM] there is no header buffers\n");
	else
		MFC_TRACE_RM("[c:%d] SETUP: Move src[%d] to queue\n",
				ctx->num, src_mb->src_index);

	if (mfc_ctx_ready_set_bit(core_ctx, &core->work_bits)) {
		ret = core->core_ops->request_work(core, MFC_WORK_BUTLER, ctx);
		if (ret) {
			mfc_ctx_err("failed to request_work\n");
			goto end_setup;
		}
	}

	mfc_debug(2, "[RM] waiting for header parsing of subcore\n");
	if (mfc_wait_for_done_core_ctx(core_ctx,
				MFC_REG_R2H_CMD_SEQ_DONE_RET)) {
		mfc_ctx_err("[RM] subcore header parsing failed\n");
		return -EAGAIN;
	}

	/* maincore number of multi core mode should MFC-0 */
	if (sub_core_num == MFC_DEC_DEFAULT_CORE) {
		ctx->op_core_num[MFC_CORE_MAIN] = MFC_DEC_DEFAULT_CORE;
		ctx->op_core_num[MFC_CORE_SUB] = MFC_SURPLUS_CORE;
		mfc_debug(2, "[RM] multi core mode, maincore changed to MFC0\n");
	}

end_setup:
	return ret;
}

void mfc_rm_request_work(struct mfc_dev *dev, enum mfc_request_work work,
		struct mfc_ctx *ctx)
{
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	int is_corelock = 0;

	/* This is a request that may not be struct mfc_ctx */
	if (work == MFC_WORK_BUTLER) {
		__mfc_rm_request_butler(dev, ctx);
		return;
	}

	if (!ctx) {
		mfc_dev_err("[RM] ctx is needed (request work: %#x)\n", work);
		return;
	}

	if (IS_TWO_MODE2(ctx)) {
		__mfc_rm_move_buf_request_work(ctx, work);
		return;
	} else if (IS_SWITCH_SINGLE_MODE(ctx)) {
		core = mfc_get_sub_core(dev, ctx);
		if (!core)
			goto err_req_work;
	} else if (IS_MODE_SWITCHING(ctx)) {
		mfc_debug(3, "[RM] mode switching op_mode: %d\n", ctx->op_mode);
		MFC_TRACE_RM("[c:%d] mode switching op_mode: %d\n", ctx->num, ctx->op_mode);
		return;
	} else {
		mfc_get_corelock_ctx(ctx);
		is_corelock = 1;
		core = mfc_get_main_core(dev, ctx);
		if (!core)
			goto err_req_work;
	}

	/* move src buffer to src_buf_queue from src_buf_ready_queue */
	core_ctx = core->core_ctx[ctx->num];
	mfc_move_buf_all(ctx, &core_ctx->src_buf_queue,
			&ctx->src_buf_ready_queue, MFC_QUEUE_ADD_BOTTOM);
	if (IS_TWO_MODE2(ctx)) {
		mfc_debug(2, "[RM] all buffer is moved but MODE2\n");
		MFC_TRACE_RM("[c:%d] all buffer is moved but MODE2\n", ctx->num);
		mfc_move_buf_all(ctx, &ctx->src_buf_ready_queue,
				&core_ctx->src_buf_queue, MFC_QUEUE_ADD_BOTTOM);
		goto err_req_work;
	}

	mutex_lock(&ctx->drc_wait_mutex);
	if ((ctx->wait_state == WAIT_G_FMT) &&
			(mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->dst_buf_queue) > 0))
		mfc_dec_drc_find_del_buf(core_ctx);
	mutex_unlock(&ctx->drc_wait_mutex);

	/* set core context work bit if it is ready */
	if (mfc_ctx_ready_set_bit(core_ctx, &core->work_bits))
		if (core->core_ops->request_work(core, work, ctx))
			mfc_debug(3, "[RM] failed to request_work\n");

err_req_work:
	if (is_corelock)
		mfc_release_corelock_ctx(ctx);
}

void mfc_rm_qos_control(struct mfc_ctx *ctx, enum mfc_qos_control qos_control)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	bool update_idle = 0;

	mfc_get_corelock_ctx(ctx);

	core = mfc_get_main_core(dev, ctx);
	if (!core) {
		mfc_debug(2, "[RM] There is no maincore\n");
		goto release_corelock;
	}

	switch (qos_control) {
	case MFC_QOS_ON:
		if (IS_MULTI_MODE(ctx) || IS_SINGLE_MODE(ctx))
			mfc_core_qos_on(core, ctx);
		if (IS_MULTI_MODE(ctx) || IS_SWITCH_SINGLE_MODE(ctx)) {
			core = mfc_get_sub_core(dev, ctx);
			if (!core) {
				snprintf(dev->dev_crash_info, MFC_CRASH_INFO_LEN,
					"[RM] There is no subcore\n");
				mfc_ctx_err("%s", dev->dev_crash_info);
				call_dop(dev, dump_and_stop_debug_mode, dev);
				goto release_corelock;
			}

			mfc_core_qos_on(core, ctx);
		}

		if (IS_MULTI_CORE_DEVICE(dev))
			mfc_rm_load_balancing(ctx, MFC_RM_LOAD_ADD);

		break;
	case MFC_QOS_OFF:
		if (IS_MULTI_MODE(ctx) || IS_SINGLE_MODE(ctx))
			mfc_core_qos_off(core, ctx);
		if (IS_MULTI_MODE(ctx) || IS_SWITCH_SINGLE_MODE(ctx)) {
			core = mfc_get_sub_core(dev, ctx);
			if (!core) {
				snprintf(dev->dev_crash_info, MFC_CRASH_INFO_LEN,
					"[RM] There is no subcore\n");
				mfc_ctx_err("%s", dev->dev_crash_info);
				call_dop(dev, dump_and_stop_debug_mode, dev);
				goto release_corelock;
			}

			mfc_core_qos_off(core, ctx);
		}
		break;
	case MFC_QOS_TRIGGER:
		if (IS_MULTI_MODE(ctx) || IS_SINGLE_MODE(ctx)) {
			update_idle = mfc_core_qos_idle_trigger(core, ctx);
			if (update_idle || ctx->update_bitrate || ctx->update_framerate)
				mfc_core_qos_on(core, ctx);
		}
		if (IS_MULTI_MODE(ctx) || IS_SWITCH_SINGLE_MODE(ctx)) {
			core = mfc_get_sub_core(dev, ctx);
			if (!core) {
				snprintf(dev->dev_crash_info, MFC_CRASH_INFO_LEN,
					"[RM] There is no subcore\n");
				call_dop(dev, dump_and_stop_debug_mode, dev);
				goto release_corelock;
			}

			update_idle = mfc_core_qos_idle_trigger(core, ctx);
			if (update_idle || ctx->update_bitrate || ctx->update_framerate)
				mfc_core_qos_on(core, ctx);
		}

		if (IS_MULTI_CORE_DEVICE(dev) && ctx->update_framerate)
			mfc_rm_load_balancing(ctx, MFC_RM_LOAD_ADD);

		ctx->update_bitrate = false;
		if (ctx->type == MFCINST_ENCODER)
			ctx->update_framerate = false;
		break;
	default:
		mfc_ctx_err("[RM] not supported QoS control type: %#x\n",
				qos_control);
	}

release_corelock:
	mfc_release_corelock_ctx(ctx);
}

int mfc_rm_query_state(struct mfc_ctx *ctx, enum mfc_inst_state_query query,
			enum mfc_inst_state state)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_core *core;
	struct mfc_core_ctx *core_ctx;
	enum mfc_inst_state main_state = MFCINST_FREE;
	enum mfc_inst_state sub_state = MFCINST_FREE;
	int main_condition = 0, sub_condition = 0;
	int ret = 0;

	mfc_get_corelock_ctx(ctx);

	core = mfc_get_main_core(dev, ctx);
	if (!core) {
		mfc_debug(3, "[RM] There is no maincore\n");
		goto err_query_state;
	}

	core_ctx = core->core_ctx[ctx->num];
	main_state = core_ctx->state;

	if (IS_MULTI_MODE(ctx)) {
		core = mfc_get_sub_core(dev, ctx);
		if (!core) {
			mfc_debug(4, "[RM] There is no subcore\n");
			goto err_query_state;
		}

		core_ctx = core->core_ctx[ctx->num];
		if (!core_ctx) {
			mfc_debug(4, "[RM] There is no subcore_ctx\n");
			goto err_query_state;
		}
		sub_state = core_ctx->state;
	}

	switch (query) {
	case EQUAL:
		if (main_state == state)
			main_condition = 1;
		if (sub_state == state)
			sub_condition = 1;
		break;
	case BIGGER:
		if (main_state > state)
			main_condition = 1;
		if (sub_state > state)
			sub_condition = 1;
		break;
	case SMALLER:
		if (main_state < state)
			main_condition = 1;
		if (sub_state < state)
			sub_condition = 1;
		break;
	case EQUAL_BIGGER:
		if (main_state >= state)
			main_condition = 1;
		if (sub_state >= state)
			sub_condition = 1;
		break;
	case EQUAL_SMALLER:
		if (main_state <= state)
			main_condition = 1;
		if (sub_state <= state)
			sub_condition = 1;
		break;
	case EQUAL_OR:
		if ((main_state == state) || (sub_state == state)) {
			main_condition = 1;
			sub_condition = 1;
		}
		break;
	default:
		mfc_ctx_err("[RM] not supported state query type: %d\n", query);
		goto err_query_state;
	}

	if (IS_MULTI_MODE(ctx)) {
		if (main_condition && sub_condition)
			ret = 1;
		else
			mfc_debug(2, "[RM] multi core maincore state: %d, subcore state: %d\n",
					main_state, sub_state);
	} else {
		if (main_condition)
			ret = 1;
		else
			mfc_debug(2, "[RM] single core maincore state: %d\n",
					main_state);
	}

err_query_state:
	mfc_release_corelock_ctx(ctx);

	return ret;
}

void mfc_rm_update_real_time(struct mfc_ctx *ctx)
{
	if (ctx->operating_framerate > 0) {
		if (ctx->prio == 0)
			ctx->rt = MFC_RT;
		else if (ctx->prio >= 1)
			ctx->rt = MFC_RT_CON;
		else
			ctx->rt = MFC_RT_LOW;
	} else {
		if ((ctx->prio == 0) && (ctx->type == MFCINST_ENCODER)) {
			if (ctx->enc_priv->params.rc_framerate)
				ctx->rt = MFC_RT;
			else
				ctx->rt = MFC_NON_RT;
		} else if (ctx->prio >= 1) {
			ctx->rt = MFC_NON_RT;
		} else {
			ctx->rt = MFC_RT_UNDEFINED;
		}
	}

	mfc_debug(2, "[PRIO] update real time: %d, operating frame rate: %lu, prio: %d\n",
			ctx->rt, ctx->operating_framerate, ctx->prio);

}
