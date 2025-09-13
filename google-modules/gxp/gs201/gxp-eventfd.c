// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP eventfd
 *
 * Copyright (C) 2022-2024 Google LLC
 */

#include <linux/eventfd.h>
#include <linux/refcount.h>
#include <linux/slab.h>

#include "gxp-eventfd.h"

struct gxp_eventfd {
	struct eventfd_ctx *ctx;
	refcount_t refcount;
};

struct gxp_eventfd *gxp_eventfd_create(int fd)
{
	struct gxp_eventfd *efd;
	int err;

	efd = kmalloc(sizeof(*efd), GFP_KERNEL);
	if (!efd)
		return ERR_PTR(-ENOMEM);

	efd->ctx = eventfd_ctx_fdget(fd);
	if (IS_ERR(efd->ctx)) {
		err = PTR_ERR(efd->ctx);
		goto error;
	}

	refcount_set(&efd->refcount, 1);

	return efd;

error:
	kfree(efd);
	return ERR_PTR(err);
}

bool gxp_eventfd_get(struct gxp_eventfd *eventfd)
{
	return refcount_inc_not_zero(&eventfd->refcount);
}

bool gxp_eventfd_put(struct gxp_eventfd *eventfd)
{
	bool refcount_is_zero;

	refcount_is_zero = refcount_dec_and_test(&eventfd->refcount);
	if (refcount_is_zero) {
		eventfd_ctx_put(eventfd->ctx);
		kfree(eventfd);
	}

	return refcount_is_zero;
}

bool gxp_eventfd_signal(struct gxp_eventfd *eventfd)
{
	bool ret;

	ret = gxp_eventfd_get(eventfd);
	if (ret)
		eventfd_signal(eventfd->ctx, 1);

	gxp_eventfd_put(eventfd);

	return ret;
}
