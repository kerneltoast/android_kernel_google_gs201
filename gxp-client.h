/* SPDX-License-Identifier: GPL-2.0 */
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

#include "gxp-internal.h"
#include "gxp-eventfd.h"
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

	bool has_block_wakelock;
	bool has_vd_wakelock;
	/* Value is one of the GXP_POWER_STATE_* values from gxp.h. */
	uint requested_power_state;
	/* Value is one of the MEMORY_POWER_STATE_* values from gxp.h. */
	uint requested_memory_power_state;
	bool requested_low_clkmux;

	struct gxp_virtual_device *vd;
	struct file *tpu_file;
	struct gxp_tpu_mbx_desc mbx_desc;

	struct gxp_eventfd *mb_eventfds[GXP_NUM_CORES];

	/* client process thread group ID is really the main process ID. */
	pid_t tgid;
	/* client process ID is really the thread ID, may be transient. */
	pid_t pid;

	/*
	 * Indicates whether the driver needs to disable telemetry when this
	 * client closes. For when the client fails to disable telemetry itself.
	 */
	bool enabled_telemetry_logging;
	bool enabled_telemetry_tracing;
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

#endif /* __GXP_CLIENT_H__ */
