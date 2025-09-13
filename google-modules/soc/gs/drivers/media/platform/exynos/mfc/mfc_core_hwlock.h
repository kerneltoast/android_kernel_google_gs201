/*
 * drivers/media/platform/exynos/mfc/mfc_core_hwlock.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_HWLOCK_H
#define __MFC_CORE_HWLOCK_H __FILE__

#include "mfc_common.h"

static inline void mfc_core_init_listable_wq_dev(struct mfc_core *core)
{
	if (!core) {
		mfc_pr_err("no mfc core device to run\n");
		return;
	}

	INIT_LIST_HEAD(&core->hwlock_wq.list);
	init_waitqueue_head(&core->hwlock_wq.wait_queue);
	mutex_init(&core->hwlock_wq.wait_mutex);
	core->hwlock_wq.core_ctx = NULL;
	core->hwlock_wq.core = core;
}

static inline void mfc_core_init_listable_wq_ctx(struct mfc_core_ctx *core_ctx)
{
	if (!core_ctx) {
		mfc_pr_err("no mfc core context to run\n");
		return;
	}

	INIT_LIST_HEAD(&core_ctx->hwlock_wq.list);
	init_waitqueue_head(&core_ctx->hwlock_wq.wait_queue);
	mutex_init(&core_ctx->hwlock_wq.wait_mutex);
	core_ctx->hwlock_wq.core_ctx = core_ctx;
	core_ctx->hwlock_wq.core = NULL;
}

static inline void mfc_core_destroy_listable_wq_core(struct mfc_core *core)
{
	if (!core) {
		mfc_pr_err("no mfc core device to run\n");
		return;
	}

	mutex_destroy(&core->hwlock_wq.wait_mutex);
}

static inline void mfc_core_destroy_listable_wq_ctx(struct mfc_core_ctx *core_ctx)
{
	if (!core_ctx) {
		mfc_pr_err("no mfc core context to run\n");
		return;
	}

	mutex_destroy(&core_ctx->hwlock_wq.wait_mutex);
}


void mfc_core_init_hwlock(struct mfc_core *core);

int mfc_core_get_hwlock_dev_migrate(struct mfc_core *core, struct mfc_core_ctx *core_ctx);
int mfc_core_get_hwlock_dev(struct mfc_core *core);
int mfc_core_get_hwlock_ctx(struct mfc_core_ctx *core_ctx);

void mfc_core_release_hwlock_dev(struct mfc_core *core);
void mfc_core_release_hwlock_ctx(struct mfc_core_ctx *core_ctx);

void mfc_core_move_hwlock_ctx(struct mfc_core *to_core, struct mfc_core *from_core,
		struct mfc_core_ctx *core_ctx);

void mfc_core_try_run(struct mfc_core *core);
void mfc_core_cleanup_work_bit_and_try_run(struct mfc_core_ctx *core_ctx);
void mfc_core_cache_flush(struct mfc_core *core, int is_drm,
		enum mfc_do_cache_flush do_cache_flush, int drm_switch, int reg_clear);
int mfc_core_just_run(struct mfc_core *core, int new_ctx_index);
void mfc_core_hwlock_handler_irq(struct mfc_core *core, struct mfc_ctx *ctx,
		unsigned int reason, unsigned int err);

#endif /* __MFC_CORE_HWLOCK_H */
