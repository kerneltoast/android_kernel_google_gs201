/*
 * drivers/media/platform/exynos/mfc/mfc_core_intlock.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "mfc_core_intlock.h"
#include "mfc_core_isr.h"

int mfc_get_core_intlock(struct mfc_core_ctx *core_ctx)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_ctx *ctx = core_ctx->ctx;

	if (!(IS_TWO_MODE2(ctx) && core_ctx->state == MFCINST_RUNNING))
		return 0;

	mutex_lock(&ctx->intlock.core_mutex);

	if (ctx->intlock.lock) {
		mfc_debug(2, "[2CORE] previous interrupt isn't handled yet\n");
		set_bit(core->id, &ctx->intlock.pending);
		mutex_unlock(&ctx->intlock.core_mutex);
		return -1;
	}

	/*
	 * 1) First interrupt case, should be core0.
	 * 2) Previous interrupt number should be different with current core.
	 */
	if ((!ctx->intlock.bits && (core->id != 0)) ||
			(ctx->intlock.bits & (1 << core->id))) {
		mfc_debug(2, "[2CORE] interrupt reverse, MFC-%d isr should be delayed handled\n",
				core->id);
		set_bit(core->id, &ctx->intlock.pending);
		mutex_unlock(&ctx->intlock.core_mutex);
		return -1;
	}

	ctx->intlock.lock = 1;
	ctx->intlock.bits = 0;
	set_bit(core->id, &ctx->intlock.bits);
	mfc_debug(3, "[2CORE] get core int lock: %#08lx\n", ctx->intlock.bits);

	mutex_unlock(&ctx->intlock.core_mutex);

	return 0;
}

void mfc_release_core_intlock(struct mfc_core_ctx *core_ctx)
{
	struct mfc_core *core = core_ctx->core;
	struct mfc_dev *dev = core->dev;
	struct mfc_ctx *ctx = core_ctx->ctx;
	struct mfc_core *pending_core = NULL;
	int i;

	mutex_lock(&ctx->intlock.core_mutex);

	if (!ctx->intlock.lock) {
		mfc_debug(4, "[2CORE] have been didn't get intlock\n");
		mutex_unlock(&ctx->intlock.core_mutex);
		return;
	}

	ctx->intlock.lock = 0;
	mfc_debug(3, "[2CORE] release core int lock\n");

	for (i = 0; i < dev->num_core; i++) {
		if (ctx->intlock.pending & (1 << i)) {
			pending_core = dev->core[i];
			clear_bit(i, &ctx->intlock.pending);
			mfc_debug(2, "[2CORE] interrupt pending clear\n");
		}
	}

	mutex_unlock(&ctx->intlock.core_mutex);

	if (pending_core)
		mfc_core_irq(pending_core->irq, pending_core);
}
