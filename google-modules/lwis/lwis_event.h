/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Event Utilities
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_EVENT_H_
#define LWIS_EVENT_H_

#include <linux/list.h>

#include "lwis_commands.h"

/*
 *  LWIS Event Defines
 */

/*
 *  LWIS Forward Declarations
 */
struct lwis_client;
struct lwis_device;

/*
 *  LWIS Event Structures
 */
/*
 *  struct lwis_device_event_state
 *  This struct keeps track of device-specific event state and controls
 */
struct lwis_device_event_state {
	int64_t event_id;
	int64_t enable_counter;
	int64_t event_counter;
	bool has_subscriber;
	struct hlist_node node;
};

/*
 * struct lwis_device_event_state_history
 * For debugging purposes, keeps track of event states and the time an event
 * triggered.
 */

struct lwis_device_event_state_history {
	struct lwis_device_event_state state;
	int64_t timestamp;
};

/*
 *  struct lwis_client_event_state
 *  This struct keeps track of client-specific event state and controls
 */
struct lwis_client_event_state {
	struct lwis_event_control event_control;
	struct hlist_node node;
	struct list_head clearance_node;
};

/*
 *  struct lwis_event_entry
 *  This struct can be used to keep track of events inside the client event
 *  queue, or the device events that are waiting to be emitted. These two
 *  types of events are mutually exclusive, i.e. an event can only be one,
 *  but not both, of those queues.
 */
struct lwis_event_entry {
	struct lwis_event_info event_info;
	struct list_head node;
};

/*
 *  LWIS Event Typedefs and Enums
 */

/*
 *  LWIS Event Interface Functions
 */

/*
 * lwis_client_event_control_set: Updates the client and device event states
 * with the new control configuration from userspace
 *
 * Locks: lwisclient->event_lock
 * Alloc: Maybe
 * Assumes: lwisclient->lock may be locked
 * Returns: 0 on success
 */
int lwis_client_event_control_set(struct lwis_client *lwisclient,
				  const struct lwis_event_control *control);

/*
 * lwis_client_event_control_get: Finds and returns the current event state
 * for a particular event id
 *
 * Locks: lwisclient->event_lock
 * Alloc: Maybe
 * Assumes: lwisclient->lock may be locked
 * Returns: 0 on success
 */
int lwis_client_event_control_get(struct lwis_client *lwisclient, int64_t event_id,
				  struct lwis_event_control *control);

/*
 * lwis_client_event_pop_front: Removes an event from the client event queue
 * that is ready to be copied to userspace.
 *
 * if event is not NULL, the caller takes ownership of *event and must free it
 * if event is NULL, this function will free the popped event object
 *
 * Locks: lwis_client->event_lock
 *
 * Alloc: No
 * Returns: 0 on success, -ENOENT if queue empty
 */
int lwis_client_event_pop_front(struct lwis_client *lwis_client, struct lwis_event_entry **event);

/*
 * lwis_client_event_peek_front: Get the front element of the queue without
 * removing it
 *
 * Locks: lwis_client->event_lock
 *
 * Alloc: No
 * Returns: 0 on success, -ENOENT if queue empty
 */
int lwis_client_event_peek_front(struct lwis_client *lwis_client, struct lwis_event_entry **event);

/*
 * lwis_client_event_queue_clear: Clear all entries inside the event queue.
 *
 * Locks: lwis_client->event_lock
 *
 * Alloc: No
 * Returns: void
 */
void lwis_client_event_queue_clear(struct lwis_client *lwis_client);

/*
 * lwis_client_event_pop_front: Removes an event from the client event queue
 * that is ready to be copied to userspace.
 *
 * if event is not NULL, the caller takes ownership of *event and must free it
 * if event is NULL, this function will free the popped event object
 *
 * Locks: lwis_client->event_lock
 *
 * Alloc: No
 * Returns: 0 on success, -ENOENT if queue empty
 */
int lwis_client_error_event_pop_front(struct lwis_client *lwis_client,
				      struct lwis_event_entry **event);

/*
 * lwis_client_event_peek_front: Get the front element of the queue without
 * removing it
 *
 * Locks: lwis_client->event_lock
 *
 * Alloc: No
 * Returns: 0 on success, -ENOENT if queue empty
 */
int lwis_client_error_event_peek_front(struct lwis_client *lwis_client,
				       struct lwis_event_entry **event);

/*
 * lwis_client_error_event_queue_clear: Clear all entries inside the event
 * queue.
 *
 * Locks: lwis_client->event_lock
 *
 * Alloc: No
 * Returns: void
 */
void lwis_client_error_event_queue_clear(struct lwis_client *lwis_client);

/*
 * lwis_client_event_states_clear: Frees all items in lwisclient->event_states
 * and clears the hash table. Used for client shutdown only.
 *
 * Locks: lwisclient->event_lock
 * Indirect Locks: lwisdevice->lock
 * Assumes: lwisclient->lock is locked
 * Alloc: Free only
 * Returns: 0 on success
 */
int lwis_client_event_states_clear(struct lwis_client *lwisclient);

/*
 * lwis_device_event_states_clear: Frees all items in lwisdev->event_states
 * and clears the hash table. Used for device shutdown only.
 *
 * Assumes: lwisdev->lock is locked
 * Returns: 0 on success
 */
int lwis_device_event_states_clear_locked(struct lwis_device *lwisdev);

/*
 * lwis_device_event_flags_updated: Notifies the device that the given event_id
 * has new flags, which allows the device to register/unregister IRQs, and
 * keep track if event is still relevant
 *
 * Locks: lwisdevice->lock
 *
 * Returns: 0 on success
 */
int lwis_device_event_flags_updated(struct lwis_device *lwis_dev, int64_t event_id,
				    uint64_t old_flags, uint64_t new_flags);

/*
 * lwis_device_event_enable: Handles when a device event needs to be
 * enabled or disabled. Actually implements the generic device events, and calls
 * into other parts to see if they care about the event as well.
 *
 * Locks: lwisdevice->lock
 * Alloc: May allocate
 * Returns: 0 on success (event enabled/disabled)
 *          -EINVAL if event unknown
 */
int lwis_device_event_enable(struct lwis_device *lwis_dev, int64_t event_id, bool enabled);

/*
 * lwis_device_event_emit: Emits an event to all the relevant clients of the
 * device
 *
 * Locks: lwis_dev->lock and then lwis_client->event_lock
 * Alloc: May allocate (GFP_ATOMIC or GFP_NOWAIT only)
 * Returns: 0 on success
 */
int lwis_device_event_emit(struct lwis_device *lwis_dev, int64_t event_id, void *payload,
			   size_t payload_size);

/*
 * lwis_device_external_event_emit: Emits an subscribed event to device.
 * The difference to lwis_device_event_emit is
 * 1. Directly assign event count to lwis_dev
 * 2. Won't have subscriber
 * 3. No payload to clients, only deliver event id and event count
 * 4. Not supported chain transaction
 */
void lwis_device_external_event_emit(struct lwis_device *lwis_dev, int64_t event_id,
				     int64_t event_counter, int64_t timestamp);

/*
 * lwis_device_error_event_emit: Emits an error event for all clients.
 * The difference to lwis_device_event_emit is that this directly sends the
 * error event to userspace without needing client to enable this event,
 * because all error events should be propagated to userspace such that
 * userspace can do the proper error handling.
 *
 * Also, no transactions will be triggered by error events.
 */
void lwis_device_error_event_emit(struct lwis_device *lwis_dev, int64_t event_id, void *payload,
				  size_t payload_size);

/*
 * lwis_device_event_state_find_or_create: Looks through the provided device's
 * event state list and tries to find a lwis_device_event_state object with the
 * matching event_id. If not found, function returns NULL pointer.
 *
 * Locks: lwis_dev->lock
 * Alloc: Maybe
 * Returns: device event state object if found, NULL otherwise.
 */
struct lwis_device_event_state *lwis_device_event_state_find(struct lwis_device *lwis_dev,
							     int64_t event_id);

/*
 * lwis_device_event_state_find_or_create: Looks through the provided device's
 * event state list and tries to find a lwis_device_event_state object with the
 * matching event_id. If not found, creates the object with 0 flags and adds it
 * to the list
 *
 * Locks: lwis_dev->lock
 * Alloc: Maybe
 * Returns: device event state object on success, errno on error
 */
struct lwis_device_event_state *lwis_device_event_state_find_or_create(struct lwis_device *lwis_dev,
								       int64_t event_id);

/*
 * lwis_client_event_state_find_or_create: Looks through the provided client's
 * event state list and tries to find a lwis_client_event_state object with the
 * matching event_id. If not found, creates the object with 0 flags and adds it
 * to the list
 *
 * Locks: lwis_client->event_lock
 * Alloc: Maybe
 * Returns: client event state object on success, errno on error
 */
struct lwis_client_event_state *
lwis_client_event_state_find_or_create(struct lwis_client *lwis_client, int64_t event_id);

/*
 * lwis_pending_event_push: Push triggered event into a local pending queue to
 * defer processing until all the current event is done
 *
 * Alloc: Maybe
 * Returns: 0 on success
 */
int lwis_pending_event_push(struct list_head *pending_events, int64_t event_id, void *payload,
			    size_t payload_size);

/*
 * lwis_pending_events_emit: If pending queue is not empty, start processing
 * and emitting the events in queue
 *
 * Returns: 0 on success
 */
int lwis_pending_events_emit(struct lwis_device *lwis_dev, struct list_head *pending_events);

/*
 * lwis_device_event_update_subscriber: The function to notify an event has been subscribed/unsubscribed.
 * Returns: 0 on success, -EINVAL if event id not found in trigger device.
 */
int lwis_device_event_update_subscriber(struct lwis_device *lwis_dev, int64_t event_id,
					bool has_subscriber);

#endif /* LWIS_EVENT_H_ */
