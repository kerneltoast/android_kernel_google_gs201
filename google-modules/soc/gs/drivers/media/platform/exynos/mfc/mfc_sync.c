/*
 * drivers/media/platform/exynos/mfc/mfc_intr.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "mfc_sync.h"

#include "mfc_perf_measure.h"

#include "mfc_core_hw_reg_api.h"

#include "mfc_queue.h"

#define R2H_BIT(x)	(((x) > 0) ? (1 << ((x) - 1)) : 0)

static inline unsigned int __mfc_r2h_bit_mask(int cmd)
{
	unsigned int mask = R2H_BIT(cmd);

	if (cmd == MFC_REG_R2H_CMD_FRAME_DONE_RET)
		mask |= (R2H_BIT(MFC_REG_R2H_CMD_FIELD_DONE_RET) |
			 R2H_BIT(MFC_REG_R2H_CMD_COMPLETE_SEQ_RET) |
			 R2H_BIT(MFC_REG_R2H_CMD_SLICE_DONE_RET) |
			 R2H_BIT(MFC_REG_R2H_CMD_INIT_BUFFERS_RET) |
			 R2H_BIT(MFC_REG_R2H_CMD_ENC_BUFFER_FULL_RET));
	/* FIXME: Temporal mask for S3D SEI processing */
	else if (cmd == MFC_REG_R2H_CMD_INIT_BUFFERS_RET)
		mask |= (R2H_BIT(MFC_REG_R2H_CMD_FIELD_DONE_RET) |
			 R2H_BIT(MFC_REG_R2H_CMD_SLICE_DONE_RET) |
			 R2H_BIT(MFC_REG_R2H_CMD_FRAME_DONE_RET));

	return (mask |= R2H_BIT(MFC_REG_R2H_CMD_ERR_RET));
}

void mfc_get_corelock_ctx(struct mfc_ctx *ctx)
{
	unsigned long flags;
	unsigned int timeout = MFC_INT_TIMEOUT;
	int ret;

	spin_lock_irqsave(&ctx->corelock.lock, flags);
	mfc_debug(2, "[CORELOCK] get_corelock: cnt %d, migrate %d\n",
			ctx->corelock.cnt, ctx->corelock.migrate);

	if (ctx->corelock.migrate) {
		spin_unlock_irqrestore(&ctx->corelock.lock, flags);
		mfc_debug(2, "[CORELOCK] now migration... start waiting corelock\n");
		ret = wait_event_timeout(ctx->corelock.migrate_wq,
				(ctx->corelock.migrate == 0),
				msecs_to_jiffies(timeout));
		if (ret == 0)
			mfc_ctx_err("[CORELOCK] waiting corelock for migration timed out\n");
		else
			mfc_debug(2, "[CORELOCK] finished waiting corelock for migration\n");

		spin_lock_irqsave(&ctx->corelock.lock, flags);
	}

	ctx->corelock.cnt++;

	spin_unlock_irqrestore(&ctx->corelock.lock, flags);
}

void mfc_release_corelock_ctx(struct mfc_ctx *ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&ctx->corelock.lock, flags);

	ctx->corelock.cnt--;
	if (ctx->corelock.cnt == 0)
		wake_up(&ctx->corelock.wq);
	else if (ctx->corelock.cnt < 0)
		mfc_ctx_err("[CORELOCK] wrong corelock cnt %d\n", ctx->corelock.cnt);

	mfc_debug(2, "[CORELOCK] release_corelock: cnt %d, migrate %d\n",
			ctx->corelock.cnt, ctx->corelock.migrate);
	spin_unlock_irqrestore(&ctx->corelock.lock, flags);
}

void mfc_get_corelock_migrate(struct mfc_ctx *ctx)
{
	unsigned long flags;
	unsigned int timeout = MFC_INT_TIMEOUT;
	int ret;

	spin_lock_irqsave(&ctx->corelock.lock, flags);
	ctx->corelock.migrate = 1;
	mfc_debug(2, "[CORELOCK] get_corelock_migrate: cnt %d, migrate %d\n",
			ctx->corelock.cnt, ctx->corelock.migrate);

	if (ctx->corelock.cnt) {
		spin_unlock_irqrestore(&ctx->corelock.lock, flags);
		mfc_debug(2, "[CORELOCK] start waiting corelock\n");
		ret = wait_event_timeout(ctx->corelock.wq,
				(ctx->corelock.cnt == 0),
				msecs_to_jiffies(timeout));
		if (ret == 0)
			mfc_ctx_err("[CORELOCK] waiting corelock timed out\n");
		else
			mfc_debug(2, "[CORELOCK] finished waiting corelock\n");

		spin_lock_irqsave(&ctx->corelock.lock, flags);
	}

	spin_unlock_irqrestore(&ctx->corelock.lock, flags);
}

void mfc_release_corelock_migrate(struct mfc_ctx *ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&ctx->corelock.lock, flags);

	ctx->corelock.migrate = 0;
	mfc_debug(2, "[CORELOCK] wakeup migration wait\n");
	wake_up(&ctx->corelock.migrate_wq);

	spin_unlock_irqrestore(&ctx->corelock.lock, flags);
}

int mfc_wait_for_done_ctx_migrate(struct mfc_dev *dev, struct mfc_ctx *ctx)
{
	int ret;
	unsigned int timeout = MFC_INT_TIMEOUT;

	mfc_debug(2, "start waiting migration\n");
	ret = wait_event_timeout(ctx->migrate_wq,
			(ctx->is_migration == 0),
			msecs_to_jiffies(timeout));
	if (ret == 0) {
		snprintf(dev->dev_crash_info, MFC_CRASH_INFO_LEN, "migration timed out\n");
		mfc_ctx_err("%s", dev->dev_crash_info);
		call_dop(dev, dump_and_stop_debug_mode, dev);
		return 1;
	}

	mfc_debug(2, "Finished waiting migration\n");
	return 0;
}

void mfc_wake_up_ctx_migrate(struct mfc_ctx *ctx)
{
	wake_up(&ctx->migrate_wq);
}

#define wait_condition(x, c) (x->int_condition &&		\
		(R2H_BIT(x->int_reason) & __mfc_r2h_bit_mask(c)))
#define is_err_cond(x)	((x->int_condition) && (x->int_reason == MFC_REG_R2H_CMD_ERR_RET))

/*
 * Return value description
 * 0: waked up before timeout
 * 1: failed to get the response for the command before timeout
*/
int mfc_wait_for_done_core(struct mfc_core *core, int command)
{
	int ret;

	if (core->state == MFCCORE_ERROR) {
		mfc_core_info("[MSR] Couldn't run HW. It's Error state\n");
		return 0;
	}

	ret = wait_event_timeout(core->cmd_wq,
			wait_condition(core, command),
			msecs_to_jiffies(MFC_INT_TIMEOUT));
	if (ret == 0) {
		mfc_core_err("Interrupt (core->int_reason:%d, command:%d) timed out\n",
							core->int_reason, command);
		if (mfc_core_check_risc2host(core)) {
			ret = wait_event_timeout(core->cmd_wq,
					wait_condition(core, command),
					msecs_to_jiffies(MFC_INT_TIMEOUT * MFC_INT_TIMEOUT_CNT));
			if (ret == 0) {
				mfc_core_err("Timeout: MFC driver waited for upward of %dmsec\n",
						3 * MFC_INT_TIMEOUT);
			} else {
				goto wait_done;
			}
		}
		snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
					"Interrupt (core->int_reason:%d, command:%d) timed out\n",
							core->int_reason, command);
		call_dop(core, dump_and_stop_debug_mode, core);
		return 1;
	}

wait_done:
	if (is_err_cond(core)) {
		mfc_core_err("Finished (core->int_reason:%d, command: %d)\n",
				core->int_reason, command);
		mfc_core_err("But error (core->int_err:%d)\n", core->int_err);
		return -1;
	}

	mfc_core_debug(2, "Finished waiting (core->int_reason:%d, command: %d)\n",
			core->int_reason, command);
	return 0;
}

/*
 * Return value description
 *  0: waked up before timeout
 *  1: failed to get the response for the command before timeout
 * -1: got the error response for the command before timeout
*/
int mfc_wait_for_done_core_ctx(struct mfc_core_ctx *core_ctx, int command)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_ctx *ctx = core_ctx->ctx;
	int ret;
	unsigned int timeout = MFC_INT_TIMEOUT;

	if (core->state == MFCCORE_ERROR) {
		mfc_core_info("[MSR] Couldn't run HW. It's Error state\n");
		return 0;
	}

	if (command == MFC_REG_R2H_CMD_CLOSE_INSTANCE_RET)
		timeout = MFC_INT_SHORT_TIMEOUT;

	ret = wait_event_timeout(core_ctx->cmd_wq,
			wait_condition(core_ctx, command),
			msecs_to_jiffies(timeout));
	if (ret == 0) {
		mfc_err("Interrupt (core_ctx->int_reason:%d, command:%d) timed out\n",
				core_ctx->int_reason, command);
		if (mfc_core_check_risc2host(core)) {
			ret = wait_event_timeout(core_ctx->cmd_wq,
					wait_condition(core_ctx, command),
					msecs_to_jiffies(MFC_INT_TIMEOUT * MFC_INT_TIMEOUT_CNT));
			if (ret == 0) {
				mfc_err("Timeout: MFC driver waited for upward of %dmsec\n",
						3 * MFC_INT_TIMEOUT);
			} else {
				goto wait_done;
			}
		}
		snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
					"Interrupt (core_ctx->int_reason:%d, command:%d) timed out\n",
						core_ctx->int_reason, command);
		call_dop(core, dump_and_stop_debug_mode, core);
		return 1;
	}

wait_done:
	if (is_err_cond(core_ctx)) {
		mfc_err("Finished (core_ctx->int_reason:%d, command: %d)\n",
				core_ctx->int_reason, command);
		mfc_err("But error (core_ctx->int_err:%d)\n", core_ctx->int_err);
		return -1;
	}

	mfc_debug(2, "Finished waiting (core_ctx->int_reason:%d, command: %d)\n",
			core_ctx->int_reason, command);
	return 0;
}

/* Wake up device wait_queue */
void mfc_wake_up_core(struct mfc_core *core, unsigned int reason,
		unsigned int err)
{
	core->int_condition = 1;
	core->int_reason = reason;
	core->int_err = err;
	wake_up(&core->cmd_wq);
}

/* Wake up context wait_queue */
void mfc_wake_up_core_ctx(struct mfc_core_ctx *core_ctx, unsigned int reason,
		unsigned int err)
{
	core_ctx->int_condition = 1;
	core_ctx->int_reason = reason;
	core_ctx->int_err = err;
	wake_up(&core_ctx->cmd_wq);
}

int mfc_core_get_new_ctx(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;
	unsigned long wflags;
	int new_ctx_index = 0;
	int cnt = 0;
	int i;

	spin_lock_irqsave(&core->work_bits.lock, wflags);

	mfc_core_debug(2, "Previous context: %d (bits %08lx)\n",
			core->curr_core_ctx, core->work_bits.bits);

	if (core->preempt_core_ctx > MFC_NO_INSTANCE_SET) {
		new_ctx_index = core->preempt_core_ctx;
		mfc_core_debug(2, "preempt_core_ctx is : %d\n", new_ctx_index);
	} else {
		for (i = 0; i < MFC_NUM_CONTEXTS; i++) {
			if (test_bit(i, &dev->otf_inst_bits)) {
				if (test_bit(i, &core->work_bits.bits)) {
					spin_unlock_irqrestore(&core->work_bits.lock, wflags);
					return i;
				}
				break;
			}
		}

		new_ctx_index = (core->curr_core_ctx + 1) % MFC_NUM_CONTEXTS;
		while (!test_bit(new_ctx_index, &core->work_bits.bits)) {
			new_ctx_index = (new_ctx_index + 1) % MFC_NUM_CONTEXTS;
			cnt++;
			if (cnt > MFC_NUM_CONTEXTS) {
				/* No contexts to run */
				spin_unlock_irqrestore(&core->work_bits.lock, wflags);
				return -EAGAIN;
			}
		}
	}

	spin_unlock_irqrestore(&core->work_bits.lock, wflags);
	return new_ctx_index;
}

int mfc_core_get_next_ctx(struct mfc_core *core)
{
	unsigned long wflags;
	int curr_ctx_index = core->curr_core_ctx;
	int next_ctx_index = (curr_ctx_index + 1) % MFC_NUM_CONTEXTS;

	spin_lock_irqsave(&core->work_bits.lock, wflags);

	mfc_core_debug(2, "Current context: %d (bits %08lx)\n",
			core->curr_core_ctx, core->work_bits.bits);

	while (!test_bit(next_ctx_index, &core->work_bits.bits)) {
		next_ctx_index = (next_ctx_index + 1) % MFC_NUM_CONTEXTS;
		if (next_ctx_index == curr_ctx_index) {
			/* No other context to run */
			spin_unlock_irqrestore(&core->work_bits.lock, wflags);
			return -EAGAIN;
		}
	}

	spin_unlock_irqrestore(&core->work_bits.lock, wflags);
	return next_ctx_index;
}

int __mfc_dec_ctx_ready_set_bit(struct mfc_core_ctx *core_ctx,
			struct mfc_bits *data, bool set)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_ctx *ctx = core_ctx->ctx;
	int src_buf_queue_greater_than_0 = 0;
	int dst_buf_queue_check_available = 0;
	unsigned long flags;
	int is_ready = 0;

	mfc_debug(1, "[MFC-%d][c:%d] src = %d(ready = %d), dst = %d, src_nal = %d, dst_nal = %d, state = %d, capstat = %d, waitstat = %d\n",
			core->id, ctx->num,
			mfc_get_queue_count(&ctx->buf_queue_lock,
				&core_ctx->src_buf_queue),
			mfc_get_queue_count(&ctx->buf_queue_lock,
				&ctx->src_buf_ready_queue),
			mfc_get_queue_count(&ctx->buf_queue_lock,
				&ctx->dst_buf_queue),
			mfc_get_queue_count(&ctx->buf_queue_lock,
				&ctx->src_buf_nal_queue),
			mfc_get_queue_count(&ctx->buf_queue_lock,
				&ctx->dst_buf_nal_queue),
			core_ctx->state, ctx->capture_state, ctx->wait_state);

	src_buf_queue_greater_than_0
		= mfc_is_queue_count_greater(&ctx->buf_queue_lock, &core_ctx->src_buf_queue, 0);
	dst_buf_queue_check_available = mfc_check_for_dpb(core_ctx);

	/* If shutdown is called, do not try any cmd */
	if (core->shutdown)
		return 0;

	/* The ready condition check and set work_bit should be synchronized */
	spin_lock_irqsave(&data->lock, flags);

	/* Context is to parse header */
	if (core_ctx->state == MFCINST_GOT_INST &&
		src_buf_queue_greater_than_0)
		is_ready = 1;

	/* Context is to decode a frame */
	else if (core_ctx->state == MFCINST_RUNNING &&
		ctx->wait_state == WAIT_NONE && src_buf_queue_greater_than_0 &&
		dst_buf_queue_check_available)
		is_ready = 1;

	/* Context is to return last frame */
	else if (core_ctx->state == MFCINST_FINISHING && dst_buf_queue_check_available)
		is_ready = 1;

	/* Context is to set buffers */
	else if (core_ctx->state == MFCINST_HEAD_PARSED &&
		(dst_buf_queue_check_available && ctx->wait_state == WAIT_NONE))
		is_ready = 1;

	/* Resolution change */
	else if ((core_ctx->state == MFCINST_RES_CHANGE_INIT || core_ctx->state == MFCINST_RES_CHANGE_FLUSH) &&
		dst_buf_queue_check_available)
		is_ready = 1;

	else if (core_ctx->state == MFCINST_RES_CHANGE_END &&
		src_buf_queue_greater_than_0)
		is_ready = 1;

	if ((is_ready == 1) && (set == true)) {
		/* if the ctx is ready and request set_bit, set the work_bit */
		__set_bit(ctx->num, &data->bits);
	} else if ((is_ready == 0) && (set == false)) {
		/* if the ctx is not ready and request clear_bit, clear the work_bit */
		__clear_bit(ctx->num, &data->bits);
	} else {
		if (set == true) {
			/* If the ctx is not ready, this is not included to S/W driver margin */
			mfc_perf_cancel_drv_margin(core);
			mfc_debug(2, "ctx is not ready\n");
		}
	}

	spin_unlock_irqrestore(&data->lock, flags);

	return is_ready;
}

static int __mfc_enc_ctx_ready_set_bit(struct mfc_core_ctx *core_ctx,
			struct mfc_bits *data, bool set)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_enc_params *p = &enc->params;
	int src_buf_queue_greater_than_0 = 0;
	int dst_buf_queue_greater_than_0 = 0;
	unsigned long flags;
	int is_ready = 0;

	mfc_debug(1, "[MFC-%d][c:%d] src = %d(ready = %d), dst = %d, src_nal = %d, dst_nal = %d, state = %d, capstat = %d, waitstat = %d\n",
			core->id, ctx->num,
			mfc_get_queue_count(&ctx->buf_queue_lock,
				&core_ctx->src_buf_queue),
			mfc_get_queue_count(&ctx->buf_queue_lock,
				&ctx->src_buf_ready_queue),
			mfc_get_queue_count(&ctx->buf_queue_lock,
				&ctx->dst_buf_queue),
			mfc_get_queue_count(&ctx->buf_queue_lock,
				&ctx->src_buf_nal_queue),
			mfc_get_queue_count(&ctx->buf_queue_lock,
				&ctx->dst_buf_nal_queue),
			core_ctx->state, ctx->capture_state, ctx->wait_state);

	src_buf_queue_greater_than_0
		= mfc_is_queue_count_greater(&ctx->buf_queue_lock, &core_ctx->src_buf_queue, 0);
	dst_buf_queue_greater_than_0
		= mfc_is_queue_count_greater(&ctx->buf_queue_lock, &ctx->dst_buf_queue, 0);

	/* If shutdown is called, do not try any cmd */
	if (core->shutdown)
		return 0;

	/* The ready condition check and set work_bit should be synchronized */
	spin_lock_irqsave(&data->lock, flags);

	/* context is ready to make header */
	if (core_ctx->state == MFCINST_GOT_INST &&
		dst_buf_queue_greater_than_0) {
		if (p->seq_hdr_mode == V4L2_MPEG_VIDEO_HEADER_MODE_AT_THE_READY) {
			if (src_buf_queue_greater_than_0)
				is_ready = 1;
		} else {
			is_ready = 1;
		}
	}

	/* context is ready to allocate DPB */
	else if (core_ctx->state == MFCINST_HEAD_PARSED &&
		dst_buf_queue_greater_than_0)
		is_ready = 1;

	/* context is ready to encode a frame */
	else if (core_ctx->state == MFCINST_RUNNING &&
		src_buf_queue_greater_than_0 && dst_buf_queue_greater_than_0)
		is_ready = 1;

	/* context is ready to encode a frame for NAL_ABORT command */
	else if (core_ctx->state == MFCINST_ABORT_INST &&
		src_buf_queue_greater_than_0 && dst_buf_queue_greater_than_0)
		is_ready = 1;

	/* context is ready to encode remain frames */
	else if (core_ctx->state == MFCINST_FINISHING && dst_buf_queue_greater_than_0)
		is_ready = 1;

	if ((is_ready == 1) && (set == true)) {
		/* if the ctx is ready and request set_bit, set the work_bit */
		__set_bit(ctx->num, &data->bits);
	} else if ((is_ready == 0) && (set == false)) {
		/* if the ctx is not ready and request clear_bit, clear the work_bit */
		__clear_bit(ctx->num, &data->bits);
	} else {
		if (set == true) {
			mfc_perf_cancel_drv_margin(core);
			mfc_debug(2, "ctx is not ready\n");
		}
	}

	spin_unlock_irqrestore(&data->lock, flags);

	return is_ready;
}


int mfc_ctx_ready_set_bit(struct mfc_core_ctx *core_ctx, struct mfc_bits *data)
{
	if (core_ctx->ctx->type == MFCINST_DECODER)
		return __mfc_dec_ctx_ready_set_bit(core_ctx, data, true);
	else if (core_ctx->ctx->type == MFCINST_ENCODER)
		return __mfc_enc_ctx_ready_set_bit(core_ctx, data, true);

	return 0;
}

int mfc_ctx_ready_clear_bit(struct mfc_core_ctx *core_ctx, struct mfc_bits *data)
{
	if (core_ctx->ctx->type == MFCINST_DECODER)
		return __mfc_dec_ctx_ready_set_bit(core_ctx, data, false);
	else if (core_ctx->ctx->type == MFCINST_ENCODER)
		return __mfc_enc_ctx_ready_set_bit(core_ctx, data, false);

	return 0;
}
