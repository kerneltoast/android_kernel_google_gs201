/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Periodic IO Processor
 *
 * Copyright (c) 2020 Google, LLC
 */

#ifndef LWIS_PERIODIC_IO_H_
#define LWIS_PERIODIC_IO_H_

#include <linux/completion.h>
#include <linux/hrtimer.h>

#include "lwis_commands.h"
#include "lwis_device.h"

enum lwis_hrtimer_state {
	LWIS_HRTIMER_ACTIVE,
	LWIS_HRTIMER_INACTIVE,
	LWIS_HRTIMER_ERROR,
};

/* This represents a Periodic IO submitted and exists in the timer periodic io
 * list until the client is released. The priodic io is deactivated when it is
 * cancelled explicitly or an error occurred druing executing it. A deactivated
 * periodic io is skipped when the timer worker func is processing workload.
 */
struct lwis_periodic_io {
	struct lwis_periodic_io_info info;
	struct lwis_periodic_io_response_header *resp;
	/* Counter of the execution times within a batch. Reset to 0 after a
	 * batch is done
	 */
	int batch_count;
	/* The timer/lwis_periodic_io_list which this periodic_io belongs to */
	struct lwis_periodic_io_list *periodic_io_list;
	/* Whether this periodic io is still active */
	bool active;
	/* The node in the timer periodic io list */
	struct list_head timer_list_node;
	/* Completion barrier to mark if io processing is ongoing */
	struct completion io_done;
	/* A flag to indicate whether the periodic io has more than one writes.
	 * This will be used on the cancellation policy to prevent partial write
	 * during cancellation
	 */
	bool contains_multiple_writes;
};

/* This is a proxy of the Periodic IO. A proxy of the actual periodic io will be
 * created and inserted into the periodic io process queue. When it is their
 * turn to be processed, it redirect the processing to the info and resp in the
 * real periodic io.
 */
struct lwis_periodic_io_proxy {
	/* The real periodic io in the timer periodic io list which this process
	 * queue proxy node is for
	 */
	struct lwis_periodic_io *periodic_io;
	/* The node in the periodic io process queue */
	struct list_head process_queue_node;
};

/* An entry in the lwis client timer list. It also manages a list of Periodic
 * IOs which share the same timer.
 */
struct lwis_periodic_io_list {
	/* High resolution timer */
	struct hrtimer hr_timer;
	/* Time out time in nanosecond */
	int64_t period_ns;
	/* State of the timer */
	enum lwis_hrtimer_state hr_timer_state;
	/* LWIS client this timer/periodic_io_list belongs to */
	struct lwis_client *client;
	/* List of periodic io operations bundled to this timer */
	struct list_head list;
	/* Node in the timer hash table held by the LWIS client */
	struct hlist_node node;
};

int lwis_periodic_io_init(struct lwis_client *client);
int lwis_periodic_io_client_flush(struct lwis_client *client);
int lwis_periodic_io_client_cleanup(struct lwis_client *client);
int lwis_periodic_io_submit(struct lwis_client *client, struct lwis_periodic_io *periodic_io);
int lwis_periodic_io_cancel(struct lwis_client *client, int64_t id);
void lwis_periodic_io_free(struct lwis_device *lwis_dev, struct lwis_periodic_io *periodic_io);
void lwis_process_periodic_io_in_queue(struct lwis_client *client);

#endif /* LWIS_PERIODIC_IO_H_ */
