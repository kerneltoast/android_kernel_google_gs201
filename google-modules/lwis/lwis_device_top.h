/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Top Level Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_DEVICE_TOP_H_
#define LWIS_DEVICE_TOP_H_

#include "lwis_device.h"

#define SCRATCH_MEMORY_SIZE 16

/*
 * struct lwis_event_subscribe_operations
 * This struct contains the 'virtual' functions for lwis_device subclasses
 * Top device should be the only device to implement it.
 */
struct lwis_event_subscribe_operations {
	/* Subscribe an event for subscriber device */
	int (*subscribe_event)(struct lwis_device *lwis_dev, int64_t trigger_event_id,
			       int trigger_device_id, int subscriber_device_id);
	/* Unsubscribe an event for subscriber device */
	int (*unsubscribe_event)(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				 int subscriber_device_id);
	/* Notify subscriber when an event is happening */
	void (*notify_event_subscriber)(struct lwis_device *lwis_dev, int64_t trigger_event_id,
					int64_t trigger_event_count,
					int64_t trigger_event_timestamp);
	/* Clean up event subscription hash table when unloading top device */
	void (*release)(struct lwis_device *lwis_dev);
};

/*
 *  struct lwis_top_device
 *  "Derived" lwis_device struct, with added top device related elements.
 */
struct lwis_top_device {
	struct lwis_device base_dev;
	/* For testing purposes, scratch memory is used as register space in
	 * top device.
	 */
	uint8_t scratch_mem[SCRATCH_MEMORY_SIZE];
	/* Hash table of event subscribers keyed by trigger event id */
	DECLARE_HASHTABLE(event_subscribers, EVENT_HASH_BITS);

	/* Subscription work */
	struct kthread_work subscribe_work;
	struct list_head emitted_event_list_work;

	/* Subscription thread */
	struct kthread_worker subscribe_worker;
	struct task_struct *subscribe_worker_thread;
	struct lwis_event_subscribe_operations subscribe_ops;

	bool transaction_worker_active;
};

int lwis_top_device_init(void);
int lwis_top_device_deinit(void);
void lwis_start_top_device_worker(struct lwis_client *client);
void lwis_stop_top_device_worker(struct lwis_client *client);

#endif /* LWIS_DEVICE_TOP_H_ */
