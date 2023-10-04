// SPDX-License-Identifier: GPL-2.0
/*
 * Asynchronous jobs management for EdgeTPU driver.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/async.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include "edgetpu-async.h"

static void edgetpu_async_wrapper(void *data, async_cookie_t cookie)
{
	struct edgetpu_async_entry *entry = data;

	entry->ret = entry->job(entry->data);
}

struct edgetpu_async_ctx *edgetpu_async_alloc_ctx(void)
{
	struct edgetpu_async_ctx *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);

	if (!ctx)
		return NULL;
	INIT_LIST_HEAD(&ctx->async_domain.pending);
	/* domain does not participate in async_synchronize_full */
	ctx->async_domain.registered = 0;
	mutex_init(&ctx->lock);
	INIT_LIST_HEAD(&ctx->jobs);

	return ctx;
}

int edgetpu_async_add_job(struct edgetpu_async_ctx *ctx, void *data,
			  edgetpu_async_job_t job)
{
	struct edgetpu_async_entry *entry = kmalloc(sizeof(*entry), GFP_KERNEL);
	int ret = 0;

	if (!entry)
		return -ENOMEM;
	mutex_lock(&ctx->lock);
	/* wait() is called */
	if (ctx->ret) {
		ret = -EINVAL;
		kfree(entry);
		goto out_unlock;
	}
	INIT_LIST_HEAD(&entry->list);
	entry->job = job;
	entry->data = data;
	list_add_tail(&entry->list, &ctx->jobs);
	ctx->n_jobs++;

out_unlock:
	mutex_unlock(&ctx->lock);
	return ret;
}

int edgetpu_async_wait(struct edgetpu_async_ctx *ctx)
{
	struct edgetpu_async_entry *entry;
	int i = 0;
	int ret = 0;

	mutex_lock(&ctx->lock);
	if (ctx->ret)
		goto out_unlock;
	ctx->ret = kmalloc_array(ctx->n_jobs, sizeof(*ctx->ret), GFP_KERNEL);
	if (!ctx->ret) {
		ret = -ENOMEM;
		goto out_unlock;
	}
	for_each_async_job(ctx, entry)
		async_schedule_domain(edgetpu_async_wrapper, entry,
				      &ctx->async_domain);
	async_synchronize_full_domain(&ctx->async_domain);
	i = 0;
	for_each_async_job(ctx, entry)
		ctx->ret[i++] = entry->ret;

out_unlock:
	mutex_unlock(&ctx->lock);
	return ret;
}

void edgetpu_async_free_ctx(struct edgetpu_async_ctx *ctx)
{
	struct edgetpu_async_entry *entry, *tmp;

	if (!ctx)
		return;
	kfree(ctx->ret);
	list_for_each_entry_safe(entry, tmp, &ctx->jobs, list)
		kfree(entry);
	/* resources in async_domain are released when all works are done */
	kfree(ctx);
}
