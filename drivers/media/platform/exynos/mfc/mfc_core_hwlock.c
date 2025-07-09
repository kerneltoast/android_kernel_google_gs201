/*
 * drivers/media/platform/exynos/mfc/mfc_core_hwlock.c
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

#include "mfc_core_hwlock.h"
#include "mfc_core_nal_q.h"
#include "mfc_core_otf.h"
#include "mfc_core_pm.h"
#include "mfc_core_run.h"
#include "mfc_core_cmd.h"
#include "mfc_core_hw_reg_api.h"

#include "mfc_sync.h"
#include "mfc_queue.h"
#include "mfc_utils.h"

static inline void __mfc_print_hwlock(struct mfc_core *core)
{
	mfc_core_debug(3, "hwlock.dev = 0x%lx, bits = 0x%lx, owned_by_irq = %d, wl_count = %d, transfer_owner = %d, migrate = %d\n",
		core->hwlock.dev, core->hwlock.bits,
		core->hwlock.owned_by_irq, core->hwlock.wl_count,
		core->hwlock.transfer_owner,
		core->hwlock.migrate);
}

void mfc_core_init_hwlock(struct mfc_core *core)
{
	unsigned long flags;

	spin_lock_init(&core->hwlock.lock);
	spin_lock_irqsave(&core->hwlock.lock, flags);

	INIT_LIST_HEAD(&core->hwlock.waiting_list);
	core->hwlock.wl_count = 0;
	core->hwlock.bits = 0;
	core->hwlock.dev = 0;
	core->hwlock.owned_by_irq = 0;
	core->hwlock.transfer_owner = 0;

	spin_unlock_irqrestore(&core->hwlock.lock, flags);
}

static void __mfc_remove_listable_wq_core(struct mfc_core *core)
{
	struct mfc_listable_wq *listable_wq;
	unsigned long flags;

	spin_lock_irqsave(&core->hwlock.lock, flags);
	__mfc_print_hwlock(core);

	list_for_each_entry(listable_wq, &core->hwlock.waiting_list, list) {
		if (!listable_wq->core)
			continue;

		mfc_core_debug(2, "Found dev and will delete it!\n");

		list_del(&listable_wq->list);
		core->hwlock.wl_count--;

		break;
	}

	__mfc_print_hwlock(core);
	spin_unlock_irqrestore(&core->hwlock.lock, flags);
}

static void __mfc_remove_listable_wq_ctx(struct mfc_core_ctx *core_ctx)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_listable_wq *listable_wq;
	unsigned long flags;

	spin_lock_irqsave(&core->hwlock.lock, flags);
	__mfc_print_hwlock(core);

	list_for_each_entry(listable_wq, &core->hwlock.waiting_list, list) {
		if (!listable_wq->core_ctx)
			continue;

		if (listable_wq->core_ctx->num == core_ctx->num) {
			mfc_core_debug(2, "Found ctx and will delete it (%d)!\n", core_ctx->num);

			list_del(&listable_wq->list);
			core->hwlock.wl_count--;
			break;
		}
	}

	__mfc_print_hwlock(core);
	spin_unlock_irqrestore(&core->hwlock.lock, flags);
}

int mfc_core_get_hwlock_dev_migrate(struct mfc_core *core, struct mfc_core_ctx *core_ctx)
{
	int ret = 0;
	unsigned long flags;

	mutex_lock(&core->hwlock_wq.wait_mutex);

	spin_lock_irqsave(&core->hwlock.lock, flags);
	__mfc_print_hwlock(core);

	if (core->shutdown) {
		mfc_core_info("Couldn't lock HW. Shutdown was called\n");
		spin_unlock_irqrestore(&core->hwlock.lock, flags);
		mutex_unlock(&core->hwlock_wq.wait_mutex);
		return -EINVAL;
	}

	if ((core->hwlock.bits != 0) || (core->hwlock.dev != 0)) {
		list_add_tail(&core->hwlock_wq.list, &core->hwlock.waiting_list);
		core->hwlock.wl_count++;

		spin_unlock_irqrestore(&core->hwlock.lock, flags);

		mfc_core_debug(2, "Waiting for hwlock to be released\n");

		ret = wait_event_timeout(core->hwlock_wq.wait_queue,
			((core->hwlock.transfer_owner == 1) && (core->hwlock.dev == 1)),
			msecs_to_jiffies(MFC_HWLOCK_TIMEOUT));

		/* save migrate info */
		core->hwlock.migrate = 1;
		core->hwlock.mig_core_ctx = core_ctx;

		core->hwlock.transfer_owner = 0;
		__mfc_remove_listable_wq_core(core);
		if (ret == 0) {
			mfc_err("Woken up but timed out\n");
			__mfc_print_hwlock(core);
			mutex_unlock(&core->hwlock_wq.wait_mutex);
			return -EIO;
		}

		mfc_core_debug(2, "Woken up and got hwlock for migrate\n");
		__mfc_print_hwlock(core);
		mutex_unlock(&core->hwlock_wq.wait_mutex);
	} else {
		/* save migrate info */
		core->hwlock.migrate = 1;
		core->hwlock.mig_core_ctx = core_ctx;

		core->hwlock.bits = 0;
		core->hwlock.dev = 1;
		core->hwlock.owned_by_irq = 0;

		mfc_core_debug(2, "got hwlock for migrate\n");
		__mfc_print_hwlock(core);
		spin_unlock_irqrestore(&core->hwlock.lock, flags);
		mutex_unlock(&core->hwlock_wq.wait_mutex);
	}

	/* Stop NAL-Q after getting hwlock */
	if (core->nal_q_handle)
		mfc_core_nal_q_stop_if_started(core);

	return 0;
}


/*
 * Return value description
 *    0: succeeded to get hwlock
 * -EIO: failed to get hwlock (time out)
 */
int mfc_core_get_hwlock_dev(struct mfc_core *core)
{
	int ret = 0;
	unsigned long flags;

	if (core->state == MFCCORE_ERROR) {
		mfc_core_info("[MSR] Couldn't lock HW. It's Error state\n");
		return 0;
	}

	if (core->shutdown) {
		mfc_core_info("Couldn't lock HW. Shutdown was called\n");
		return -EINVAL;
	}

	mutex_lock(&core->hwlock_wq.wait_mutex);

	spin_lock_irqsave(&core->hwlock.lock, flags);
	__mfc_print_hwlock(core);

	if ((core->hwlock.bits != 0) || (core->hwlock.dev != 0)) {
		list_add_tail(&core->hwlock_wq.list, &core->hwlock.waiting_list);
		core->hwlock.wl_count++;

		spin_unlock_irqrestore(&core->hwlock.lock, flags);

		mfc_core_debug(2, "Waiting for hwlock to be released\n");

		ret = wait_event_timeout(core->hwlock_wq.wait_queue,
			((core->hwlock.transfer_owner == 1) && (core->hwlock.dev == 1)),
			msecs_to_jiffies(MFC_HWLOCK_TIMEOUT));

		core->hwlock.transfer_owner = 0;
		__mfc_remove_listable_wq_core(core);
		if (ret == 0) {
			mfc_core_err("Woken up but timed out\n");
			__mfc_print_hwlock(core);
			mutex_unlock(&core->hwlock_wq.wait_mutex);
			return -EIO;
		} else {
			mfc_core_debug(2, "Woken up and got hwlock\n");
			__mfc_print_hwlock(core);
			mutex_unlock(&core->hwlock_wq.wait_mutex);
		}
	} else {
		core->hwlock.bits = 0;
		core->hwlock.dev = 1;
		core->hwlock.owned_by_irq = 0;

		__mfc_print_hwlock(core);
		spin_unlock_irqrestore(&core->hwlock.lock, flags);
		mutex_unlock(&core->hwlock_wq.wait_mutex);
	}

	/* Stop NAL-Q after getting hwlock */
	if (core->nal_q_handle)
		mfc_core_nal_q_stop_if_started(core);

	return 0;
}

/*
 * Return value description
 *    0: succeeded to get hwlock
 * -EIO: failed to get hwlock (time out)
 */
int mfc_core_get_hwlock_ctx(struct mfc_core_ctx *core_ctx)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct mfc_dev *dev = ctx->dev;
	int ret = 0;
	unsigned long flags;

	if (core->state == MFCCORE_ERROR) {
		mfc_core_info("[MSR] Couldn't lock HW. It's Error state\n");
		return 0;
	}

	if (core->shutdown) {
		mfc_core_info("Couldn't lock HW. Shutdown was called\n");
		return -EINVAL;
	}

	mutex_lock(&core_ctx->hwlock_wq.wait_mutex);

	spin_lock_irqsave(&core->hwlock.lock, flags);
	__mfc_print_hwlock(core);

	if (core->hwlock.migrate && core->hwlock.mig_core_ctx == core_ctx) {
		mfc_core_info("Waiting for hwlock to be done migration\n");
		spin_unlock_irqrestore(&core->hwlock.lock, flags);
		ret = mfc_wait_for_done_ctx_migrate(dev, ctx);
		if (!ret) {
			__mfc_print_hwlock(core);

			core = core_ctx->core;
			spin_lock_irqsave(&core->hwlock.lock, flags);
			mfc_core_debug(2, "Changed core to MFC-%d during get hwlock\n", core->id);
			__mfc_print_hwlock(core);
		} else {
			mfc_core_debug(2, "Failed to migrate, keep use core MFC-%d\n", core->id);
		}
	}

	if ((core->hwlock.bits != 0) || (core->hwlock.dev != 0)) {
		list_add_tail(&core_ctx->hwlock_wq.list, &core->hwlock.waiting_list);
		core->hwlock.wl_count++;

		spin_unlock_irqrestore(&core->hwlock.lock, flags);

		mfc_core_debug(2, "core_ctx[%d] Waiting for hwlock to be released\n",
				core_ctx->num);

		ret = wait_event_timeout(core_ctx->hwlock_wq.wait_queue,
			((core->hwlock.transfer_owner == 1) && (test_bit(core_ctx->num, &core->hwlock.bits))),
			msecs_to_jiffies(MFC_HWLOCK_TIMEOUT));

		core->hwlock.transfer_owner = 0;
		__mfc_remove_listable_wq_ctx(core_ctx);
		if (ret == 0) {
			mfc_err("Woken up but timed out\n");
			__mfc_print_hwlock(core);
			mutex_unlock(&core_ctx->hwlock_wq.wait_mutex);
			return -EIO;
		} else {
			mfc_core_debug(2, "Woken up and got hwlock\n");
			__mfc_print_hwlock(core);
			mutex_unlock(&core_ctx->hwlock_wq.wait_mutex);
		}
	} else {
		core->hwlock.bits = 0;
		core->hwlock.dev = 0;
		set_bit(core_ctx->num, &core->hwlock.bits);
		core->hwlock.owned_by_irq = 0;

		__mfc_print_hwlock(core);
		spin_unlock_irqrestore(&core->hwlock.lock, flags);
		mutex_unlock(&core_ctx->hwlock_wq.wait_mutex);
	}

	/* Stop NAL-Q after getting hwlock */
	if (core->nal_q_handle)
		mfc_core_nal_q_stop_if_started(core);

	return 0;
}

/*
 * Return value description
 *  0: succeeded to release hwlock
 *  1: succeeded to release hwlock, hwlock is captured by another module
 * -1: error since device is waiting again.
 */
void mfc_core_release_hwlock_dev(struct mfc_core *core)
{
	struct mfc_listable_wq *listable_wq;
	unsigned long flags;

	spin_lock_irqsave(&core->hwlock.lock, flags);
	__mfc_print_hwlock(core);

	/* clear migrate info */
	core->hwlock.migrate = 0;
	core->hwlock.mig_core_ctx = NULL;

	core->hwlock.dev = 0;
	core->hwlock.owned_by_irq = 0;

	if (core->state == MFCCORE_ERROR) {
		mfc_core_debug(2, "[MSR] Couldn't wakeup module. It's Error state\n");
	} else if (core->shutdown) {
		mfc_core_debug(2, "Couldn't wakeup module. Shutdown was called\n");
	} else if (list_empty(&core->hwlock.waiting_list)) {
		mfc_core_debug(2, "No waiting module\n");
	} else {
		mfc_core_debug(2, "There is a waiting module\n");
		listable_wq = list_entry(core->hwlock.waiting_list.next, struct mfc_listable_wq, list);
		list_del(&listable_wq->list);
		core->hwlock.wl_count--;

		if (listable_wq->core) {
			mfc_core_debug(2, "Waking up core\n");
			core->hwlock.dev = 1;
		} else {
			mfc_core_debug(2, "Waking up another ctx\n");
			set_bit(listable_wq->core_ctx->num, &core->hwlock.bits);
		}

		core->hwlock.transfer_owner = 1;

		wake_up(&listable_wq->wait_queue);
	}

	__mfc_print_hwlock(core);
	spin_unlock_irqrestore(&core->hwlock.lock, flags);
}

/*
 * Should be called with hwlock.lock
 *
 * Return value description
 * 0: succeeded to release hwlock
 * 1: succeeded to release hwlock, hwlock is captured by another module
 */
static void __mfc_release_hwlock_ctx_protected(struct mfc_core_ctx *core_ctx)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_listable_wq *listable_wq;

	__mfc_print_hwlock(core);
	clear_bit(core_ctx->num, &core->hwlock.bits);
	core->hwlock.owned_by_irq = 0;

	if (core->state == MFCCORE_ERROR) {
		mfc_core_debug(2, "[MSR] Couldn't wakeup module. It's Error state\n");
	} else if (core->shutdown) {
		mfc_core_debug(2, "Couldn't wakeup module. Shutdown was called\n");
	} else if (list_empty(&core->hwlock.waiting_list)) {
		mfc_core_debug(2, "No waiting module\n");
	} else {
		mfc_core_debug(2, "There is a waiting module\n");
		listable_wq = list_entry(core->hwlock.waiting_list.next, struct mfc_listable_wq, list);
		list_del(&listable_wq->list);
		core->hwlock.wl_count--;

		if (listable_wq->core) {
			mfc_core_debug(2, "Waking up core\n");
			core->hwlock.dev = 1;
		} else {
			mfc_core_debug(2, "Waking up another ctx\n");
			set_bit(listable_wq->core_ctx->num, &core->hwlock.bits);
		}

		core->hwlock.transfer_owner = 1;

		wake_up(&listable_wq->wait_queue);
	}

	__mfc_print_hwlock(core);
}

/*
 * Return value description
 * 0: succeeded to release hwlock
 * 1: succeeded to release hwlock, hwlock is captured by another module
 */
void mfc_core_release_hwlock_ctx(struct mfc_core_ctx *core_ctx)
{
	struct mfc_core *core = core_ctx->core;
	unsigned long flags;

	spin_lock_irqsave(&core->hwlock.lock, flags);
	__mfc_release_hwlock_ctx_protected(core_ctx);
	spin_unlock_irqrestore(&core->hwlock.lock, flags);
}

void mfc_core_move_hwlock_ctx(struct mfc_core *to_core, struct mfc_core *from_core,
		struct mfc_core_ctx *core_ctx)
{
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct mfc_dev *dev = ctx->dev;
	struct mfc_listable_wq *listable_wq;
	bool is_move = false;

	if (from_core->hwlock.bits & (1UL << core_ctx->num))
		is_move = true;

	list_for_each_entry(listable_wq, &from_core->hwlock.waiting_list, list) {
		if (!listable_wq->core_ctx)
			continue;

		if (listable_wq->core_ctx->num == core_ctx->num)
			is_move = true;
	}

	if (is_move) {
		mfc_debug(2, "There is a waiting module to moving core%d\n",
				from_core->id);
		MFC_TRACE_RM("[c:%d] hwlock wait in moving core%d\n",
				ctx->num, from_core->id);
		/* remove waiting list from from_core->hwlock */
		core_ctx->core = from_core;
		__mfc_remove_listable_wq_ctx(core_ctx);

		/* add waiting list to to_core->hwlock */
		core_ctx->core = to_core;
		list_add_tail(&core_ctx->hwlock_wq.list, &to_core->hwlock.waiting_list);
		to_core->hwlock.wl_count++;
		to_core->hwlock.owned_by_irq = 1;
		mfc_ctx_info("hwlock waiting module moved MFC%d -> MFC%d\n",
				from_core->id, to_core->id);
		MFC_TRACE_RM("[c:%d] hwlock module move %d -> %d\n",
				ctx->num, from_core->id, to_core->id);
	}
}

static inline void __mfc_yield_hwlock(struct mfc_core *core,
		struct mfc_core_ctx *core_ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&core->hwlock.lock, flags);

	__mfc_release_hwlock_ctx_protected(core_ctx);

	spin_unlock_irqrestore(&core->hwlock.lock, flags);

	/* Trigger again if other instance's work is waiting */
	if (mfc_core_is_work_to_do(core))
		queue_work(core->butler_wq, &core->butler_work);
}

/*
 * Should be called with hwlock.lock
 */
static inline void __mfc_transfer_hwlock_ctx_protected(struct mfc_core *core,
		int ctx_index)
{
	core->hwlock.dev = 0;
	core->hwlock.bits = 0;
	set_bit(ctx_index, &core->hwlock.bits);
}

/*
 * Should be called with hwlock.lock
 *
 * Return value description
 *   >=0: succeeded to get hwlock_bit for the context, index of new context
 *   -1, -EINVAL: failed to get hwlock_bit for a context
 */
static int __mfc_try_to_get_new_ctx_protected(struct mfc_core *core)
{
	struct mfc_dev *dev = core->dev;
	int ret = 0;
	int index;
	struct mfc_ctx *new_ctx;

	if (core->shutdown) {
		mfc_core_info("Couldn't lock HW. Shutdown was called\n");
		return -EINVAL;
	}

	if (core->sleep) {
		mfc_core_info("Couldn't lock HW. Sleep was called\n");
		return -EINVAL;
	}

	/* Check whether hardware is not running */
	if ((core->hwlock.bits != 0) || (core->hwlock.dev != 0)) {
		/* This is perfectly ok, the scheduled ctx should wait */
		mfc_core_debug(2, "Couldn't lock HW\n");
		return -1;
	}

	/* Choose the context to run */
	index = mfc_core_get_new_ctx(core);
	if (index < 0) {
		/* This is perfectly ok, the scheduled ctx should wait
		 * No contexts to run
		 */
		mfc_core_debug(2, "No ctx is scheduled to be run\n");
		ret = -1;
		return ret;
	}

	new_ctx = dev->ctx[index];
	if (!new_ctx) {
		mfc_core_err("no mfc context to run\n");
		ret = -1;
		return ret;
	}

	set_bit(new_ctx->num, &core->hwlock.bits);
	ret = index;

	return ret;
}

/*
 * Should be called without hwlock holding
 *
 * Try to run an operation on hardware
 */
void mfc_core_try_run(struct mfc_core *core)
{
	int new_ctx_index;
	int ret;
	unsigned long flags;

	if (core->state == MFCCORE_ERROR) {
		mfc_core_info("[MSR] Couldn't run HW. It's Error state\n");
		return;
	}

	spin_lock_irqsave(&core->hwlock.lock, flags);
	__mfc_print_hwlock(core);

	new_ctx_index = __mfc_try_to_get_new_ctx_protected(core);
	if (new_ctx_index < 0) {
		mfc_core_debug(2, "Failed to get new context to run\n");
		__mfc_print_hwlock(core);
		spin_unlock_irqrestore(&core->hwlock.lock, flags);
		return;
	}

	core->hwlock.owned_by_irq = 1;

	__mfc_print_hwlock(core);
	spin_unlock_irqrestore(&core->hwlock.lock, flags);

	ret = mfc_core_just_run(core, new_ctx_index);
	if (ret)
		__mfc_yield_hwlock(core, core->core_ctx[new_ctx_index]);
}

/*
 * Should be called without hwlock holding
 *
 */
void mfc_core_cleanup_work_bit_and_try_run(struct mfc_core_ctx *core_ctx)
{
	struct mfc_core *core = core_ctx->core;

	mfc_clear_bit(core_ctx->num, &core->work_bits);

	mfc_core_try_run(core);
}

void mfc_core_cache_flush(struct mfc_core *core, int is_drm,
		enum mfc_do_cache_flush do_cache_flush, int drm_switch, int reg_clear)
{
	enum mfc_fw_status fw_status;

	if (core->state == MFCCORE_ERROR) {
		mfc_core_info("[MSR] Couldn't lock HW. It's Error state\n");
		return;
	}

	/*
	 * Even if it is determined that the attribute of the previous instance
	 * and the current instance have been changed, (= drm_switch)
	 * there is no need to cache flush if the F/W of the previous instance is unloaded.
	 */
	if (drm_switch) {
		if (is_drm)
			fw_status = core->fw.status;
		else
			fw_status = core->fw.drm_status;

		if (!(fw_status & MFC_FW_LOADED)) {
			mfc_core_debug(2, "F/W has already un-loaded\n");
			do_cache_flush = MFC_NO_CACHEFLUSH;
		}
	}

	if (do_cache_flush == MFC_CACHEFLUSH) {
		mfc_core_cmd_cache_flush(core);
		if (mfc_wait_for_done_core(core, MFC_REG_R2H_CMD_CACHE_FLUSH_RET)) {
			mfc_core_err("Failed to CACHE_FLUSH\n");
			core->logging_data->cause |= (1 << MFC_CAUSE_FAIL_CACHE_FLUSH);
			call_dop(core, dump_and_stop_always, core);
		}
	} else if (do_cache_flush == MFC_NO_CACHEFLUSH) {
		mfc_core_debug(2, "F/W has already done cache flush with prediction\n");
	}

	/* When init_hw(), reg_clear is required between cache flush and (un)protection */
	if (reg_clear) {
		mfc_core_reg_clear(core);
		mfc_core_debug(2, "Done register clear\n");
	}

	mfc_core_change_attribute(core, is_drm);

	/* drm_switch may not occur when cache flush is required during migration. */
	if (!drm_switch)
		return;

	if (is_drm) {
		MFC_TRACE_CORE("Normal -> DRM\n");
		mfc_core_debug(2, "Normal -> DRM need protection\n");
		mfc_core_protection_on(core);
	} else {
		MFC_TRACE_CORE("DRM -> Normal\n");
		mfc_core_debug(2, "DRM -> Normal need un-protection\n");
		mfc_core_protection_off(core);
	}
}

/*
 * Return value description
 *  0: NAL-Q is handled successfully
 *  1: NAL_START command should be handled
 * -1: Error
*/
static int __mfc_nal_q_just_run(struct mfc_core *core, struct mfc_core_ctx *core_ctx,
			int drm_switch)
{
	struct mfc_ctx *ctx = core_ctx->ctx;
	nal_queue_handle *nal_q_handle = core->nal_q_handle;
	unsigned int ret = -1;

	switch (nal_q_handle->nal_q_state) {
	case NAL_Q_STATE_CREATED:
		if (mfc_core_nal_q_check_enable(core) == 0) {
			/* NAL START */
			ret = 1;
		} else {
			mfc_core_nal_q_clock_on(core, nal_q_handle, ctx->num);

			mfc_core_nal_q_init(core, nal_q_handle);

			/* enable NAL QUEUE */
			if (drm_switch)
				mfc_core_cache_flush(
					core, ctx->is_drm, MFC_CACHEFLUSH, drm_switch, 0);

			if (IS_NO_INFOLOG(ctx))
				mfc_debug(2, "[NALQ] start NAL QUEUE\n");
			else
				mfc_ctx_info("[NALQ] start NAL QUEUE\n");
			mfc_core_nal_q_start(core, nal_q_handle);

			if (mfc_core_nal_q_enqueue_in_buf(core, core_ctx, nal_q_handle->nal_q_in_handle)) {
				mfc_debug(2, "[NALQ] Failed to enqueue input data\n");
				mfc_core_nal_q_clock_off(core, nal_q_handle, ctx->num);
			}

			mfc_clear_bit(ctx->num, &core->work_bits);

			mfc_ctx_ready_set_bit(core_ctx, &core->work_bits);
			if (nal_q_handle->nal_q_exception)
				mfc_set_bit(ctx->num, &core->work_bits);

			mfc_core_release_hwlock_ctx(core_ctx);

			if (mfc_core_is_work_to_do(core))
				queue_work(core->butler_wq, &core->butler_work);

			ret = 0;
		}
		break;
	case NAL_Q_STATE_STARTED:
		mfc_core_nal_q_clock_on(core, nal_q_handle, ctx->num);

		if (mfc_core_nal_q_check_enable(core) == 0 ||
				nal_q_handle->nal_q_exception) {
			/* disable NAL QUEUE */
			mfc_core_nal_q_stop(core, nal_q_handle);

			if (IS_NO_INFOLOG(ctx))
				mfc_debug(2, "[NALQ] stop NAL QUEUE\n");
			else
				mfc_ctx_info("[NALQ] stop NAL QUEUE\n");
			if (mfc_wait_for_done_core(core,
					MFC_REG_R2H_CMD_COMPLETE_QUEUE_RET)) {
				mfc_err("[NALQ] Failed to stop queue\n");
				core->logging_data->cause |= (1 << MFC_CAUSE_FAIL_STOP_NAL_Q);
				call_dop(core, dump_and_stop_always, core);
	                }
			/* nal_q_exception 2 means stop NALQ and do not handle NAL_START command */
			if (nal_q_handle->nal_q_exception == 2) {
				mfc_debug(2, "[NALQ] stopped, handle new work\n");
				mfc_clear_bit(ctx->num, &core->work_bits);
				mfc_core_release_hwlock_ctx(core_ctx);

				if (mfc_core_is_work_to_do(core))
					queue_work(core->butler_wq, &core->butler_work);
				ret = 0;
			} else {
				ret = 1;
			}
			break;
		} else {
			/* NAL QUEUE */
			if (mfc_core_nal_q_enqueue_in_buf(core, core_ctx, nal_q_handle->nal_q_in_handle)) {
				mfc_debug(2, "[NALQ] Failed to enqueue input data\n");
				mfc_core_nal_q_clock_off(core, nal_q_handle, ctx->num);
			}

			mfc_clear_bit(ctx->num, &core->work_bits);

			mfc_ctx_ready_set_bit(core_ctx, &core->work_bits);
			if (nal_q_handle->nal_q_exception)
				mfc_set_bit(ctx->num, &core->work_bits);

			mfc_core_release_hwlock_ctx(core_ctx);

			if (mfc_core_is_work_to_do(core))
				queue_work(core->butler_wq, &core->butler_work);
			ret = 0;
		}
		break;
	default:
		mfc_ctx_info("[NALQ] can't try command, nal_q_state : %d\n",
				nal_q_handle->nal_q_state);
		ret = -1;
		break;
	}

	return ret;
}

static int __mfc_just_run_dec(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int ret = 0;

	switch (core_ctx->state) {
	case MFCINST_FINISHING:
		ret = mfc_core_run_dec_last_frames(core, ctx);
		break;
	case MFCINST_RUNNING:
	case MFCINST_SPECIAL_PARSING_NAL:
		ret = mfc_core_run_dec_frame(core, ctx);
		break;
	/* TODO SAY: How about goto run layer -> cmd layer */
	case MFCINST_INIT:
		mfc_core_cmd_open_inst(core, ctx);
		break;
	case MFCINST_RETURN_INST:
		ret = mfc_core_cmd_close_inst(core, ctx);
		break;
	case MFCINST_GOT_INST:
	case MFCINST_SPECIAL_PARSING:
		ret = mfc_core_run_dec_init(core, ctx);
		break;
	case MFCINST_HEAD_PARSED:
		if (core_ctx->codec_buffer_allocated == 0) {
			ctx->clear_work_bit = 1;
			mfc_err("codec buffer is not allocated\n");
			ret = -EAGAIN;
			break;
		}
		if (ctx->wait_state != WAIT_NONE) {
			mfc_err("wait_state(%d) is not ready\n", ctx->wait_state);
			ret = -EAGAIN;
			break;
		}
		ret = mfc_core_cmd_dec_init_buffers(core, ctx);
		break;
	case MFCINST_RES_CHANGE_INIT:
		ret = mfc_core_run_dec_last_frames(core, ctx);
		break;
	case MFCINST_RES_CHANGE_FLUSH:
		ret = mfc_core_run_dec_last_frames(core, ctx);
		break;
	case MFCINST_RES_CHANGE_END:
		mfc_debug(2, "[DRC] Finished remaining frames after resolution change\n");
		ctx->capture_state = QUEUE_FREE;
		mfc_debug(2, "[DRC] Will re-init the codec\n");
		ret = mfc_core_run_dec_init(core, ctx);
		break;
	case MFCINST_DPB_FLUSHING:
		mfc_core_cmd_dpb_flush(core, ctx);
		break;
	case MFCINST_MOVE_INST:
		mfc_core_cmd_move_inst(core, ctx);
		break;
	default:
		mfc_ctx_info("can't try command(decoder just_run), state : %d\n",
				core_ctx->state);
		ret = -EAGAIN;
	}

	return ret;
}

static int __mfc_just_run_enc(struct mfc_core *core, struct mfc_ctx *ctx)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int ret = 0;

	switch (core_ctx->state) {
	case MFCINST_FINISHING:
		ret = mfc_core_run_enc_last_frames(core, ctx);
		break;
	case MFCINST_RUNNING:
		if (ctx->otf_handle) {
			ret = mfc_core_otf_run_enc_frame(core, ctx);
			break;
		}
		ret = mfc_core_run_enc_frame(core, ctx);
		break;
	case MFCINST_INIT:
		mfc_core_cmd_open_inst(core, ctx);
		break;
	case MFCINST_RETURN_INST:
		ret = mfc_core_cmd_close_inst(core, ctx);
		break;
	case MFCINST_GOT_INST:
		if (ctx->otf_handle) {
			ret = mfc_core_otf_run_enc_init(core, ctx);
			break;
		}
		ret = mfc_core_run_enc_init(core, ctx);
		break;
	case MFCINST_HEAD_PARSED:
		ret = mfc_core_cmd_enc_init_buffers(core, ctx);
		break;
	case MFCINST_ABORT_INST:
		mfc_core_cmd_abort_inst(core, ctx);
		break;
	case MFCINST_MOVE_INST:
		mfc_core_cmd_move_inst(core, ctx);
		break;
	default:
		mfc_ctx_info("can't try command(encoder just_run), state : %d\n",
				core_ctx->state);
		ret = -EAGAIN;
	}

	return ret;
}

/* Run an operation on hardware */
int mfc_core_just_run(struct mfc_core *core, int new_ctx_index)
{
	struct mfc_dev *dev = core->dev;
	struct mfc_core_ctx *core_ctx = core->core_ctx[new_ctx_index];
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct mfc_ctx *next_ctx = NULL;
	unsigned int ret = 0;
	int drm_switch = 0;
	int next_ctx_index;

	if ((core->state == MFCCORE_ERROR) || (core_ctx->state == MFCINST_ERROR)) {
		mfc_core_info("[MSR] Couldn't run HW. It's Error state\n");
		return 0;
	}

	atomic_inc(&core->hw_run_cnt);

	if (core_ctx->state == MFCINST_RUNNING)
		mfc_clean_core_ctx_int_flags(core_ctx);

	mfc_debug(2, "New context: %d\n", new_ctx_index);
	core->curr_core_ctx = core_ctx->num;

	/* Got context to run in ctx */
	mfc_debug(2, "src: %d(ready: %d), dst: %d, state: %d, dpb_count = %d\n",
		mfc_get_queue_count(&ctx->buf_queue_lock, &core_ctx->src_buf_queue),
		mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->src_buf_ready_queue),
		mfc_get_queue_count(&ctx->buf_queue_lock, &ctx->dst_buf_queue),
		core_ctx->state, ctx->dpb_count);
	mfc_debug(2, "core_ctx->state = %d\n", core_ctx->state);
	/* Last frame has already been sent to MFC
	 * Now obtaining frames from MFC buffer */

	/* Check if drm switch occurs */
	if (core->curr_core_ctx_is_drm != ctx->is_drm)
		drm_switch = 1;
	else
		mfc_core_change_attribute(core, ctx->is_drm);

	mfc_debug(2, "drm_switch = %d, is_drm = %d\n", drm_switch, ctx->is_drm);

	if (core->nal_q_handle) {
		ret = __mfc_nal_q_just_run(core, core_ctx, drm_switch);
		if (ret == 0) {
			mfc_debug(2, "NAL_Q was handled\n");
			return ret;
		} else if (ret == 1){
			/* Path through */
			mfc_debug(2, "NAL_START will be handled\n");
		} else {
			return ret;
		}
	}

	mfc_debug(2, "continue_clock_on = %d\n", core->continue_clock_on);
	if (!core->continue_clock_on) {
		mfc_core_pm_clock_on(core);
	} else {
		core->continue_clock_on = false;
	}

	if (!MFC_FEATURE_SUPPORT(dev, dev->pdata->drm_switch_predict)
			|| drm_predict_disable) {
		if (drm_switch)
			mfc_core_cache_flush(core, ctx->is_drm, MFC_CACHEFLUSH, drm_switch, 0);
	} else {
		/* If Normal <-> Secure switch, check if cache flush was done */
		if (drm_switch) {
			mfc_debug(2, "%s\n", core->last_cmd_has_cache_flush ?
					"Last command had cache flush" :
					"Last command had No cache flush");
			mfc_core_cache_flush(core, ctx->is_drm,
					core->last_cmd_has_cache_flush ?
					MFC_NO_CACHEFLUSH : MFC_CACHEFLUSH,
					drm_switch, 0);
		}

		/*
		 * Prediction code - if another context is ready,
		 * see if it will cause Normal<->Secure switch.
		 * If so, let F/W do CACHE_FLUSH successively after a command.
		 */
		next_ctx_index = mfc_core_get_next_ctx(core);
		if (next_ctx_index >= 0) {
			next_ctx = dev->ctx[next_ctx_index];
			/* Next ctx causes Normal<->Secure switch */
			if (ctx->is_drm != next_ctx->is_drm)
				core->cache_flush_flag = 1;
		}
	}

	if (ctx->type == MFCINST_DECODER) {
		ret = __mfc_just_run_dec(core, ctx);
	} else if (ctx->type == MFCINST_ENCODER) {
		ret = __mfc_just_run_enc(core, ctx);
	} else {
		mfc_err("invalid context type: %d\n", ctx->type);
		ret = -EAGAIN;
	}

	if (ret) {
		/*
		 * Clear any reserved F/W cache flush for next ctx,
		 * as this will be newly decided in Prediction code.
		 */
		core->cache_flush_flag = 0;
		core->last_cmd_has_cache_flush = 0;

		/*
		 * Check again the ctx condition and clear work bits
		 * if ctx is not available.
		 */
		if (mfc_ctx_ready_clear_bit(core_ctx, &core->work_bits) == 0)
			ctx->clear_work_bit = 0;
		if (ctx->clear_work_bit) {
			mfc_clear_bit(ctx->num, &core->work_bits);
			ctx->clear_work_bit = 0;
		}

		mfc_core_pm_clock_off(core);
	}

	return ret;
}

void mfc_core_hwlock_handler_irq(struct mfc_core *core, struct mfc_ctx *ctx,
		unsigned int reason, unsigned int err)
{
	struct mfc_core_ctx *core_ctx = core->core_ctx[ctx->num];
	int new_ctx_index;
	unsigned long flags;
	int ret, need_butler = 0;

	if (core->state == MFCCORE_ERROR) {
		mfc_core_info("[MSR] Couldn't lock HW. It's Error state\n");
		return;
	}

	spin_lock_irqsave(&core->hwlock.lock, flags);
	__mfc_print_hwlock(core);

	if ((core_ctx->state == MFCINST_RUNNING) && IS_TWO_MODE2(ctx))
		need_butler = 1;

	if (core->hwlock.owned_by_irq) {
		if (core->preempt_core_ctx > MFC_NO_INSTANCE_SET) {
			mfc_debug(2, "There is a preempt_core_ctx\n");
			core->continue_clock_on = true;
			mfc_wake_up_core_ctx(core_ctx, reason, err);
			new_ctx_index = core->preempt_core_ctx;
			mfc_debug(2, "preempt_core_ctx is : %d\n", new_ctx_index);

			spin_unlock_irqrestore(&core->hwlock.lock, flags);

			ret = mfc_core_just_run(core, new_ctx_index);
			if (ret) {
				core->continue_clock_on = false;
				__mfc_yield_hwlock(core, core->core_ctx[new_ctx_index]);
			}
		} else if (!list_empty(&core->hwlock.waiting_list)) {
			mfc_debug(2, "There is a waiting module for hwlock\n");
			core->continue_clock_on = false;
			mfc_core_pm_clock_off(core);

			spin_unlock_irqrestore(&core->hwlock.lock, flags);

			mfc_wake_up_core_ctx(core_ctx, reason, err);
			mfc_core_release_hwlock_ctx(core_ctx);
			queue_work(core->butler_wq, &core->butler_work);
		} else {
			mfc_debug(2, "No preempt_ctx and no waiting module\n");
			new_ctx_index = mfc_core_get_new_ctx(core);
			if (new_ctx_index < 0) {
				mfc_debug(2, "No ctx to run\n");
				/* No contexts to run */
				core->continue_clock_on = false;
				mfc_core_pm_clock_off(core);

				spin_unlock_irqrestore(&core->hwlock.lock, flags);

				mfc_wake_up_core_ctx(core_ctx, reason, err);
				mfc_core_release_hwlock_ctx(core_ctx);
				queue_work(core->butler_wq, &core->butler_work);
			} else {
				mfc_debug(2, "There is a ctx to run\n");
				core->continue_clock_on = true;
				mfc_wake_up_core_ctx(core_ctx, reason, err);

				/* If cache flush command is needed or there is OTF handle, handler should stop */
				if ((core->curr_core_ctx_is_drm != core->core_ctx[new_ctx_index]->is_drm) ||
						core->core_ctx[new_ctx_index]->ctx->otf_handle) {
					mfc_debug(2, "Secure and nomal switching or OTF mode\n");
					mfc_debug(2, "DRM attribute %d->%d\n",
							core->curr_core_ctx_is_drm,
							core->core_ctx[new_ctx_index]->is_drm);

					spin_unlock_irqrestore(&core->hwlock.lock, flags);

					mfc_core_release_hwlock_ctx(core_ctx);
					queue_work(core->butler_wq, &core->butler_work);
				} else {
					mfc_debug(2, "Work to do successively (next ctx: %d)\n", new_ctx_index);
					__mfc_transfer_hwlock_ctx_protected(core, new_ctx_index);

					spin_unlock_irqrestore(&core->hwlock.lock, flags);

					ret = mfc_core_just_run(core, new_ctx_index);
					if (ret) {
						core->continue_clock_on = false;
						__mfc_yield_hwlock(core, core->core_ctx[new_ctx_index]);
					}
				}
			}
		}
	} else {
		mfc_debug(2, "hwlock is NOT owned by irq\n");
		core->continue_clock_on = false;
		mfc_core_pm_clock_off(core);
		mfc_wake_up_core_ctx(core_ctx, reason, err);
		queue_work(core->butler_wq, &core->butler_work);

		spin_unlock_irqrestore(&core->hwlock.lock, flags);
	}

	if (need_butler)
		queue_work(core->dev->butler_wq, &core->dev->butler_work);

	__mfc_print_hwlock(core);
}
