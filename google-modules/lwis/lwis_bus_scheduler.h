/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Bus Scheduler
 *
 * Copyright 2023 Google LLC.
 */

#ifndef LWIS_BUS_SCHED_H_
#define LWIS_BUS_SCHED_H_

#include "lwis_device.h"

struct lwis_process_queue;

/*
 * lwis_process_request:
 * This maintains the node to identify the devices that
 * have a request to be processed on a given bus
 */
struct lwis_process_request {
	struct lwis_client *requesting_client;
	struct list_head request_node;
};

bool lwis_process_request_queue_is_empty(struct lwis_process_queue *process_queue);

void lwis_process_request_queue_initialize(struct lwis_process_queue *process_queue);

void lwis_process_request_queue_destroy(struct lwis_process_queue *process_queue);

#endif /* LWIS_BUS_SCHED_H_ */
