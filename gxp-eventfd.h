/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP eventfd
 *
 * Copyright (C) 2022 Google LLC
 */
#ifndef __GXP_EVENTFD_H__
#define __GXP_EVENTFD_H__

#include "gxp-internal.h"

struct gxp_eventfd;

/**
 * gxp_eventfd_create() - Open and initialize an eventfd
 * @fd: A file descriptor from user-space describing an eventfd
 *
 * If successful, the gxp_eventfd will be returned with a reference count of 1.
 *
 * Return: A pointer to the new gxp_eventfd or an ERR_PTR on failure
 * * -ENOMEM: Insufficient memory to create the gxp_eventfd
 * * other: Failed to obtain an eventfd from @fd
 */
struct gxp_eventfd *gxp_eventfd_create(int fd);

/**
 * gxp_eventfd_get() - Increment an existing gxp_eventfd's reference count
 * @eventfd: The gxp_eventfd to get a reference to
 *
 * Return: true on success, false if the eventfd's reference count was already 0
 */
bool gxp_eventfd_get(struct gxp_eventfd *eventfd);

/**
 * gxp_eventfd_put() - Decrement an eventfd's reference count
 * @eventfd: The gxp_eventfd to close a reference to, and potentially free
 *
 * If the reference count drops to 0, the @eventfd will be freed.
 *
 * Return: true if the reference count dropped to 0 and the gxp_eventfd was
 *         released, otherwise false
 */
bool gxp_eventfd_put(struct gxp_eventfd *eventfd);

/**
 * gxp_eventfd_signal() - Signal an eventfd.
 * @eventfd: The gxp_eventfd to signal
 *
 * Return: true on success, false if the gxp_eventfd had a reference count of 0
 */
bool gxp_eventfd_signal(struct gxp_eventfd *eventfd);

#endif /* __GXP_EVENTFD_H__ */
