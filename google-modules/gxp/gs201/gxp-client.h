/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP client structure.
 *
 * Copyright (C) 2022 Google LLC
 */
#ifndef __GXP_CLIENT_H__
#define __GXP_CLIENT_H__

#include <linux/file.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/types.h>

#include "gxp-eventfd.h"
#include "gxp-internal.h"
#include "gxp-pm.h"
#include "gxp-vd.h"

/* Holds state belonging to a client */
struct gxp_client {
	struct list_head list_entry;
	struct gxp_dev *gxp;

	/*
	 * Protects all state of this client instance.
	 * Any operation that requires a client hold a particular wakelock must
	 * lock this semaphore for reading for the duration of that operation.
	 */
	struct rw_semaphore semaphore;
	struct lock_class_key key;

	bool has_block_wakelock;
	bool has_vd_wakelock;

	struct gxp_power_states requested_states;

	struct gxp_virtual_device *vd;
	struct file *tpu_file;
	struct gxp_tpu_mbx_desc mbx_desc;

	struct gxp_eventfd *mb_eventfds[GXP_NUM_CORES];

	/* client process thread group ID is really the main process ID. */
	pid_t tgid;
	/* client process ID is really the thread ID, may be transient. */
	pid_t pid;
};

/*
 * Allocates and initializes a client container.
 */
struct gxp_client *gxp_client_create(struct gxp_dev *gxp);

/*
 * Frees up the client container cleaning up any wakelocks, virtual devices, or
 * TPU mailboxes it holds.
 */
void gxp_client_destroy(struct gxp_client *client);

/**
 * gxp_client_get() - Increases the reference count for the target client.
 * @client: The client to increase the reference count.
 *
 * Return: The target client.
 */
struct gxp_client *gxp_client_get(struct gxp_client *client);

/**
 * gxp_client_put() - Decreases the reference count for the target client.
 * @client: The client to decrease the reference count.
 */
void gxp_client_put(struct gxp_client *client);

/**
 * gxp_client_allocate_virtual_device() - Allocates a virtual device for the
 * client.
 *
 * @client: The client to allocate a virtual device
 * @core_count: The requested core count of the virtual device.
 * @flags: The flags passed from the runtime's request.
 *
 * The caller must have locked client->semaphore.
 *
 * Return:
 * * 0          - Success
 * * -EINVAL    - A virtual device of the client has been allocated
 * * Otherwise  - Errno returned by virtual device allocation
 */
int gxp_client_allocate_virtual_device(struct gxp_client *client,
				       uint core_count, u8 flags);
/**
 * gxp_client_acquire_block_wakelock() - Acquires a block wakelock.
 *
 * @client: The client to acquire wakelock.
 * @acquired_wakelock: True if block wakelock has been acquired by this client.
 *
 * The caller must have locked client->semaphore.
 *
 * Note that this function won't increase the PM count. (i.e., won't call gcip_pm_get)
 *
 * Return:
 * * 0          - Success
 * * Otherwise  - Errno returned by block wakelock acquisition
 */
int gxp_client_acquire_block_wakelock(struct gxp_client *client,
				      bool *acquired_wakelock);
/**
 * gxp_client_release_block_wakelock() - Releases the holded block wakelock and
 * revokes the power votes.
 *
 * The caller must have locked client->semaphore.
 *
 * Note that this function won't decrease the PM count. (i.e., won't call gcip_pm_put)
 *
 * Return: false only when @client hasn't held the block wakelock.
 */
bool gxp_client_release_block_wakelock(struct gxp_client *client);
/**
 * gxp_client_acquire_vd_wakelock() - Acquires a VD wakelock for the current
 * virtual device to start the virtual device or resume it if it's suspended.
 * Also the client can request the power votes tied with the acquired wakelock.
 *
 * @client: The client to acquire wakelock and request power votes.
 * @requested_states: The requested power states.
 *
 * The caller must have locked client->semaphore.
 * This function is only meaningful in direct mode. On MCU mode it returns 0 directly.
 *
 * Return:
 * * 0          - Success
 * * -EINVAL    - No holded block wakelock
 * * -ENODEV    - VD state is unavailable
 */
int gxp_client_acquire_vd_wakelock(struct gxp_client *client,
				   struct gxp_power_states requested_states);
/**
 * gxp_client_release_vd_wakelock() - Releases the held VD wakelock to suspend the current virtual
 * device.
 *
 * The caller must have locked client->semaphore.
 * This function is only meaningful in direct mode. On MCU mode it returns directly.
 */
void gxp_client_release_vd_wakelock(struct gxp_client *client);

/**
 * gxp_client_has_available_vd() - Returns whether @client has an available
 * virtual device.
 *
 * @client: The client to check.
 * @name: The string used for logging when the client has an invalid VD.
 *
 * The caller must have locked client->semaphore.
 */
bool gxp_client_has_available_vd(struct gxp_client *client, const char *name);

#endif /* __GXP_CLIENT_H__ */
