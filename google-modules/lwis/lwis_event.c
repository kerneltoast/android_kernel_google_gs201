// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Event Utilities
 *
 * Copyright (c) 2018 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-event: " fmt

#include <linux/kernel.h>
#include <linux/slab.h>

#include "lwis_device.h"
#include "lwis_device_top.h"
#include "lwis_event.h"
#include "lwis_transaction.h"
#include "lwis_util.h"

/* Maximum number of pending events in the event queues */
#define MAX_NUM_PENDING_EVENTS 2048

/* Exposes the device id embedded in the event id */
#define EVENT_OWNER_DEVICE_ID(x) ((x >> LWIS_EVENT_ID_EVENT_CODE_LEN) & 0xFFFF)

#define lwis_dev_err_ratelimited(dev, fmt, ...)                                                    \
	{                                                                                          \
		static int64_t timestamp;                                                          \
		if (ktime_to_ns(lwis_get_time()) - timestamp > 2000000000LL) {                     \
			dev_err(dev, fmt, ##__VA_ARGS__);                                          \
			timestamp = ktime_to_ns(lwis_get_time());                                  \
		}                                                                                  \
	}

/*
 * client_event_state_find_locked: Looks through the provided client's
 * event state list and tries to find a lwis_client_event_state object with the
 * matching event_id. If not found, returns NULL
 *
 * Assumes: lwis_client->event_lock is locked
 * Alloc: No
 * Returns: client event state object, if found, NULL otherwise
 */
static struct lwis_client_event_state *
client_event_state_find_locked(struct lwis_client *lwis_client, int64_t event_id)
{
	/* Our hash iterator */
	struct lwis_client_event_state *p;
	int64_t new_event_id;

	new_event_id = (event_id & LWIS_OVERFLOW_IRQ_EVENT_FLAG) ?
			       (event_id ^ LWIS_OVERFLOW_IRQ_EVENT_FLAG) :
			       event_id;
	/* Iterate through the hash bucket for this event_id */
	hash_for_each_possible(lwis_client->event_states, p, node, new_event_id) {
		/* If it's indeed the right one, return it */
		if (p->event_control.event_id == new_event_id)
			return p;
	}

	return NULL;
}

/*
 * client_event_state_find: Looks through the provided client's
 * event state list and tries to find a lwis_client_event_state object with the
 * matching event_id. If not found, returns NULL
 *
 * Locks: lwis_client->event_lock
 * Alloc: No
 * Returns: client event state object, if found, NULL otherwise
 */
static struct lwis_client_event_state *client_event_state_find(struct lwis_client *lwis_client,
							       int64_t event_id)
{
	/* Our return value  */
	struct lwis_client_event_state *state;
	/* Flags for IRQ disable */
	unsigned long flags;

	/* Lock and disable to prevent event_states from changing */
	spin_lock_irqsave(&lwis_client->event_lock, flags);
	state = client_event_state_find_locked(lwis_client, event_id);
	/* Unlock and restore */
	spin_unlock_irqrestore(&lwis_client->event_lock, flags);

	return state;
}

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
lwis_client_event_state_find_or_create(struct lwis_client *lwis_client, int64_t event_id)
{
	struct lwis_client_event_state *new_state;
	/* Flags for IRQ disable */
	unsigned long flags;

	/* Try to find a state first, if it already exists */
	struct lwis_client_event_state *state = client_event_state_find(lwis_client, event_id);

	/* If it doesn't, we'll have to create one */
	if (unlikely(state == NULL)) {
		/* Allocate a new state object */
		new_state = kmalloc(sizeof(struct lwis_client_event_state), GFP_ATOMIC);
		/* Oh no, ENOMEM */
		if (!new_state)
			return ERR_PTR(-ENOMEM);

		/* Set the event_id and initialize flags to 0 which pretty much
		 * means everything is disabled. Overall it's expected that
		 * having no client event state entry is equivalent to having
		 * one with flags == 0
		 */
		new_state->event_control.event_id = event_id;
		new_state->event_control.flags = 0;

		/* Critical section for adding to the hash table */
		spin_lock_irqsave(&lwis_client->event_lock, flags);
		/* Let's avoid the race condition in case somebody else beat us
		 * here, and verify that this event_id is still not in the hash
		 * table.
		 */
		state = client_event_state_find_locked(lwis_client, event_id);
		/* Ok, it's not there */
		if (state == NULL) {
			/* Let's add the new state object */
			hash_add(lwis_client->event_states, &new_state->node, event_id);
			state = new_state;
		} else {
			/* Ok, we now suddenly have a valid state so we need to
			 * free the one we allocated, and pretend like nothing
			 * bad happened.
			 */
			kfree(new_state);
		}

		/* End critical section */
		spin_unlock_irqrestore(&lwis_client->event_lock, flags);
	}
	return state;
}
/*
 * device_event_state_find_locked: Looks through the provided device's
 * event state list and tries to find a lwis_device_event_state object with the
 * matching event_id. If not found, returns NULL
 *
 * Assumes: lwis_dev->lock is locked
 * Alloc: No
 * Returns: device event state object, if found, NULL otherwise
 */
static struct lwis_device_event_state *device_event_state_find_locked(struct lwis_device *lwis_dev,
								      int64_t event_id)
{
	/* Our hash iterator */
	struct lwis_device_event_state *p;
	int64_t new_event_id;

	new_event_id = (event_id & LWIS_OVERFLOW_IRQ_EVENT_FLAG) ?
			       (event_id ^ LWIS_OVERFLOW_IRQ_EVENT_FLAG) :
			       event_id;
	/* Iterate through the hash bucket for this event_id */
	hash_for_each_possible(lwis_dev->event_states, p, node, new_event_id) {
		/* If it's indeed the right one, return it */
		if (p->event_id == new_event_id)
			return p;
	}

	return NULL;
}

/*
 * save_device_event_state_to_history_locked: Saves the emitted events in a
 * history buffer for better debugability.
 *
 * Assumes: lwis_dev->lock is locked
 * Alloc : No
 * Returns: None
 */
static void save_device_event_state_to_history_locked(struct lwis_device *lwis_dev,
						      struct lwis_device_event_state *state,
						      int64_t timestamp)
{
	lwis_dev->debug_info.event_hist[lwis_dev->debug_info.cur_event_hist_idx].state = *state;
	lwis_dev->debug_info.event_hist[lwis_dev->debug_info.cur_event_hist_idx].timestamp =
		timestamp;
	lwis_dev->debug_info.cur_event_hist_idx++;
	if (lwis_dev->debug_info.cur_event_hist_idx >= EVENT_DEBUG_HISTORY_SIZE)
		lwis_dev->debug_info.cur_event_hist_idx = 0;
}

/*
 * lwis_device_event_state_find: Looks through the provided device's
 * event state list and tries to find a lwis_device_event_state object with the
 * matching event_id. If not found, returns NULL
 *
 * Locks: lwis_dev->lock
 * Alloc: No
 * Returns: device event state object, if found, NULL otherwise
 */
struct lwis_device_event_state *lwis_device_event_state_find(struct lwis_device *lwis_dev,
							     int64_t event_id)
{
	/* Our return value  */
	struct lwis_device_event_state *state;
	/* Flags for IRQ disable */
	unsigned long flags;

	/* Lock and disable to prevent event_states from changing */
	spin_lock_irqsave(&lwis_dev->lock, flags);
	state = device_event_state_find_locked(lwis_dev, event_id);
	/* Unlock and restore */
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	return state;
}

struct lwis_device_event_state *lwis_device_event_state_find_or_create(struct lwis_device *lwis_dev,
								       int64_t event_id)
{
	struct lwis_device_event_state *new_state;
	/* Flags for IRQ disable */
	unsigned long flags;
	struct lwis_device_event_state *state;

	if (unlikely(event_id == LWIS_EVENT_ID_NONE))
		return ERR_PTR(-EINVAL);

	/* Try to find a state first, if it already exists */
	state = lwis_device_event_state_find(lwis_dev, event_id);

	/* If it doesn't, we'll have to create one */
	if (unlikely(state == NULL)) {
		/* Allocate a new state object */
		new_state = kmalloc(sizeof(struct lwis_device_event_state), GFP_ATOMIC);
		/* Oh no, ENOMEM */
		if (!new_state)
			return ERR_PTR(-ENOMEM);

		/* Set the event_id and initialize ref counter  to 0 which means
		 * off by default
		 */
		new_state->event_id = event_id;
		new_state->enable_counter = 0;
		new_state->event_counter = 0;
		new_state->has_subscriber = false;

		/* Critical section for adding to the hash table */
		spin_lock_irqsave(&lwis_dev->lock, flags);
		/* Let's avoid the race condition in case somebody else beat us
		 * here, and verify that this event_id is still not in the hash
		 * table.
		 */
		state = device_event_state_find_locked(lwis_dev, event_id);
		/* Ok, it's not there */
		if (state == NULL) {
			/* Let's add the new state object */
			hash_add(lwis_dev->event_states, &new_state->node, event_id);
			state = new_state;
		} else {
			/* Ok, we now suddenly have a valid state so we need to
			 * free the one we allocated, and pretend like nothing
			 * bad happened.
			 */
			kfree(new_state);
		}

		/* End critical section */
		spin_unlock_irqrestore(&lwis_dev->lock, flags);
	}
	return state;
}

static int client_event_subscribe(struct lwis_client *lwis_client, int64_t trigger_event_id)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	struct lwis_device *trigger_device;
	struct lwis_top_device *top_dev;
	int trigger_device_id = EVENT_OWNER_DEVICE_ID(trigger_event_id);

	/* Check if top device probe failed */
	if (lwis_dev->top_dev == NULL) {
		dev_err(lwis_dev->dev, "Top device is null\n");
		return -EINVAL;
	}

	trigger_device = lwis_find_dev_by_id(trigger_device_id);
	if (!trigger_device) {
		dev_err(lwis_dev->dev, "Device id : %d doesn't match to any device\n",
			trigger_device_id);
		return -EINVAL;
	}

	/* Create event state to trigger/subscriber device
	 * Because of driver initialize in user space is sequential, it's
	 * possible that subscriber device subscribe an event before trigger
	 * device set it up
	 */
	if (IS_ERR_OR_NULL(lwis_device_event_state_find_or_create(lwis_dev, trigger_event_id)) ||
	    IS_ERR_OR_NULL(lwis_client_event_state_find_or_create(lwis_client, trigger_event_id)) ||
	    IS_ERR_OR_NULL(
		    lwis_device_event_state_find_or_create(trigger_device, trigger_event_id))) {
		dev_err(lwis_dev->dev,
			"Failed to add event id 0x%llx to trigger/subscriber device\n",
			trigger_event_id);

		return -EINVAL;
	}
	top_dev = container_of(lwis_dev->top_dev, struct lwis_top_device, base_dev);
	ret = top_dev->subscribe_ops.subscribe_event(lwis_dev->top_dev, trigger_event_id,
						     trigger_device->id, lwis_dev->id);
	if (ret < 0)
		dev_err(lwis_dev->dev, "Failed to subscribe event: 0x%llx\n", trigger_event_id);

	return ret;
}

static int client_event_unsubscribe(struct lwis_client *lwis_client, int64_t event_id)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	struct lwis_device_event_state *event_state;
	struct lwis_top_device *top_dev;
	unsigned long flags;

	/* Check if top device probe failed */
	if (lwis_dev->top_dev == NULL) {
		dev_err(lwis_dev->dev, "Top device is null\n");
		return -EINVAL;
	}

	top_dev = container_of(lwis_dev->top_dev, struct lwis_top_device, base_dev);
	ret = top_dev->subscribe_ops.unsubscribe_event(lwis_dev->top_dev, event_id, lwis_dev->id);
	if (ret < 0)
		dev_err(lwis_dev->dev, "Failed to unsubscribe event: 0x%llx\n", event_id);

	/* Reset event counter */
	event_state = lwis_device_event_state_find(lwis_dev, event_id);
	if (event_state) {
		spin_lock_irqsave(&lwis_dev->lock, flags);
		event_state->event_counter = 0;
		spin_unlock_irqrestore(&lwis_dev->lock, flags);
	}

	return ret;
}

static int check_event_control_flags(struct lwis_client *lwis_client, int64_t event_id,
				     uint64_t old_flags, uint64_t new_flags)
{
	if (EVENT_OWNER_DEVICE_ID(event_id) == lwis_client->lwis_dev->id) {
		if (event_id & LWIS_HW_IRQ_EVENT_FLAG &&
		    (new_flags & LWIS_EVENT_CONTROL_FLAG_QUEUE_ENABLE) &&
		    !(new_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE) &&
		    !(new_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE_ONCE)) {
			dev_err(lwis_client->lwis_dev->dev,
				"QUEUE_ENABLE without IRQ_ENABLE is not allowed for HW event: 0x%llx\n",
				event_id);
			return -EINVAL;
		}
	} else {
		/* b/187758268 for fixing the hard LOCKUP when running LWIS cross-device tests. */
		if (lwis_client->lwis_dev->type == DEVICE_TYPE_TOP) {
			dev_err(lwis_client->lwis_dev->dev,
				"Disallow top device being the subscriber device\n");
			return -EPERM;
		}
	}
	return 0;
}

int lwis_client_event_control_set(struct lwis_client *lwis_client,
				  const struct lwis_event_control *control)
{
	int ret = 0;
	struct lwis_client_event_state *state;
	uint64_t old_flags, new_flags;
	/* Find, or create, a client event state objcet for this event_id */
	state = lwis_client_event_state_find_or_create(lwis_client, control->event_id);
	if (IS_ERR_OR_NULL(state)) {
		dev_err(lwis_client->lwis_dev->dev,
			"Failed to find or create new client event state\n");
		return -ENOMEM;
	}

	old_flags = state->event_control.flags;
	new_flags = control->flags;
	if (old_flags != new_flags) {
		ret = check_event_control_flags(lwis_client, control->event_id, old_flags,
						new_flags);
		if (ret)
			return ret;

		state->event_control.flags = new_flags;
		ret = lwis_device_event_flags_updated(lwis_client->lwis_dev, control->event_id,
						      old_flags, new_flags);
		if (ret) {
			dev_err(lwis_client->lwis_dev->dev, "Updating device flags failed: %d\n",
				ret);
			return ret;
		}

		if (EVENT_OWNER_DEVICE_ID(control->event_id) != lwis_client->lwis_dev->id) {
			if (new_flags != 0) {
				ret = client_event_subscribe(lwis_client, control->event_id);
				if (ret) {
					dev_err(lwis_client->lwis_dev->dev,
						"Subscribe event failed: %d\n", ret);
				}
			} else {
				ret = client_event_unsubscribe(lwis_client, control->event_id);
				if (ret) {
					dev_err(lwis_client->lwis_dev->dev,
						"UnSubscribe event failed: %d\n", ret);
				}
			}
		}
	}

	return ret;
}

int lwis_client_event_control_get(struct lwis_client *lwis_client, int64_t event_id,
				  struct lwis_event_control *control)
{
	struct lwis_client_event_state *state;

	state = lwis_client_event_state_find_or_create(lwis_client, event_id);

	if (IS_ERR_OR_NULL(state)) {
		dev_err(lwis_client->lwis_dev->dev, "Failed to create new event state\n");
		return -ENOMEM;
	}

	control->flags = state->event_control.flags;

	return 0;
}

static int event_queue_get_front(struct lwis_client *lwis_client, struct list_head *event_queue,
				 size_t *event_queue_size, bool should_remove_entry,
				 struct lwis_event_entry **event_out)
{
	/* Our client event object */
	struct lwis_event_entry *event;
	/* Flags for irqsave */
	unsigned long flags;

	/* Critical section while modifying event queue */
	spin_lock_irqsave(&lwis_client->event_lock, flags);
	if (list_empty(event_queue)) {
		spin_unlock_irqrestore(&lwis_client->event_lock, flags);
		return -ENOENT;
	}
	/* Get the front of the list */
	event = list_first_entry(event_queue, struct lwis_event_entry, node);
	if (should_remove_entry) {
		/* Delete from the queue */
		list_del(&event->node);
		(*event_queue_size)--;
	}
	if (event_out) {
		/* Copy it over */
		*event_out = event;
	} else if (should_remove_entry) {
		/* The caller did not request ownership of the event,
		 * and this is a "pop" operation, we can just free the
		 * event here.
		 */
		kfree(event);
	}
	spin_unlock_irqrestore(&lwis_client->event_lock, flags);

	return 0;
}

static void event_queue_clear(struct lwis_client *lwis_client, struct list_head *event_queue,
			      size_t *event_queue_size)
{
	struct list_head *it_event, *it_tmp;
	struct lwis_event_entry *event;
	unsigned long flags;

	spin_lock_irqsave(&lwis_client->event_lock, flags);
	list_for_each_safe(it_event, it_tmp, event_queue) {
		event = list_entry(it_event, struct lwis_event_entry, node);
		list_del(&event->node);
		kfree(event);
	}
	*event_queue_size = 0;
	spin_unlock_irqrestore(&lwis_client->event_lock, flags);
}

int lwis_client_event_pop_front(struct lwis_client *lwis_client,
				struct lwis_event_entry **event_out)
{
	return event_queue_get_front(lwis_client, &lwis_client->event_queue,
				     &lwis_client->event_queue_size,
				     /*should_remove_entry=*/true, event_out);
}

int lwis_client_event_peek_front(struct lwis_client *lwis_client,
				 struct lwis_event_entry **event_out)
{
	return event_queue_get_front(lwis_client, &lwis_client->event_queue,
				     &lwis_client->event_queue_size,
				     /*should_remove_entry=*/false, event_out);
}

void lwis_client_event_queue_clear(struct lwis_client *lwis_client)
{
	event_queue_clear(lwis_client, &lwis_client->event_queue, &lwis_client->event_queue_size);
}

int lwis_client_error_event_pop_front(struct lwis_client *lwis_client,
				      struct lwis_event_entry **event_out)
{
	return event_queue_get_front(lwis_client, &lwis_client->error_event_queue,
				     &lwis_client->error_event_queue_size,
				     /*should_remove_entry=*/true, event_out);
}

int lwis_client_error_event_peek_front(struct lwis_client *lwis_client,
				       struct lwis_event_entry **event_out)
{
	return event_queue_get_front(lwis_client, &lwis_client->error_event_queue,
				     &lwis_client->error_event_queue_size,
				     /*should_remove_entry=*/false, event_out);
}

void lwis_client_error_event_queue_clear(struct lwis_client *lwis_client)
{
	event_queue_clear(lwis_client, &lwis_client->error_event_queue,
			  &lwis_client->error_event_queue_size);
}

/*
 * client_event_push_back: Inserts new event into the client event queue
 * to be later consumed by userspace. Takes ownership of *event (does not copy,
 * will be freed on the other side)
 *
 * Also wakes up any readers for this client (select() callers, etc.)
 *
 * Locks: lwis_client->event_lock
 *
 * Alloc: No
 * Returns: 0 on success
 */
static int client_event_push_back(struct lwis_client *lwis_client, struct lwis_event_entry *event)
{
	unsigned long flags;
	int64_t timestamp_diff;
	int64_t current_timestamp;
	struct lwis_event_entry *first_event;

	if (!event) {
		dev_err(lwis_client->lwis_dev->dev, "NULL event provided\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&lwis_client->event_lock, flags);

	if (lwis_client->event_queue_size >= MAX_NUM_PENDING_EVENTS) {
		/* Get the front of the list */
		first_event =
			list_first_entry(&lwis_client->event_queue, struct lwis_event_entry, node);
		current_timestamp = lwis_get_time();
		timestamp_diff = ktime_sub(current_timestamp, first_event->event_info.timestamp_ns);
		lwis_dev_err_ratelimited(
			lwis_client->lwis_dev->dev,
			"First event in queue ID: 0x%llx, current timestamp %lld ns, diff: %lld ns\n",
			event->event_info.event_id, current_timestamp, timestamp_diff);
		spin_unlock_irqrestore(&lwis_client->event_lock, flags);
		/* Send an error event to userspace to handle the overflow */
		lwis_device_error_event_emit(lwis_client->lwis_dev,
					     LWIS_ERROR_EVENT_ID_EVENT_QUEUE_OVERFLOW,
					     /*payload=*/NULL, /*payload_size=*/0);
		return -EOVERFLOW;
	}

	list_add_tail(&event->node, &lwis_client->event_queue);
	lwis_client->event_queue_size++;

	spin_unlock_irqrestore(&lwis_client->event_lock, flags);

	wake_up_interruptible(&lwis_client->event_wait_queue);

	return 0;
}

static int client_error_event_push_back(struct lwis_client *lwis_client,
					struct lwis_event_entry *event)
{
	unsigned long flags;

	if (!event) {
		pr_err("NULL event provided\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&lwis_client->event_lock, flags);

	if (lwis_client->error_event_queue_size >= MAX_NUM_PENDING_EVENTS) {
		dev_warn_ratelimited(
			lwis_client->lwis_dev->dev,
			"Failed to push to error event queue, exceeded size limit of %d\n",
			MAX_NUM_PENDING_EVENTS);
		spin_unlock_irqrestore(&lwis_client->event_lock, flags);
		return -EOVERFLOW;
	}

	list_add_tail(&event->node, &lwis_client->error_event_queue);
	lwis_client->error_event_queue_size++;

	spin_unlock_irqrestore(&lwis_client->event_lock, flags);

	wake_up_interruptible(&lwis_client->event_wait_queue);

	return 0;
}

int lwis_client_event_states_clear(struct lwis_client *lwis_client)
{
	/* Our hash table iterator */
	struct lwis_client_event_state *state;
	/* Temporary vars for hash table traversal */
	struct hlist_node *n;
	int i;
	struct list_head *it_event, *it_tmp;
	/* Events to be cleared */
	struct list_head events_to_clear;
	/* Flags for irqsave */
	unsigned long flags;
	int ret;

	INIT_LIST_HEAD(&events_to_clear);
	/* Disable IRQs and lock the event lock */
	spin_lock_irqsave(&lwis_client->event_lock, flags);
	/* Iterate over the entire hash table */
	hash_for_each_safe(lwis_client->event_states, i, n, state, node) {
		/* Delete the node from the client's hash table */
		hash_del(&state->node);
		/* Add the node to to-clear events hash table */
		list_add_tail(&state->clearance_node, &events_to_clear);
	}
	/* Restore the lock */
	spin_unlock_irqrestore(&lwis_client->event_lock, flags);

	/* Clear the individual events */
	list_for_each_safe(it_event, it_tmp, &events_to_clear) {
		state = list_entry(it_event, struct lwis_client_event_state, clearance_node);
		list_del(&state->clearance_node);
		/* Update the device state with zero flags */
		lwis_device_event_flags_updated(lwis_client->lwis_dev,
						state->event_control.event_id,
						state->event_control.flags, 0);
		/* Free the object */
		kfree(state);
	}

	if (lwis_client->lwis_dev->irqs) {
		ret = lwis_interrupt_write_combined_mask_value(lwis_client->lwis_dev->irqs);
		if (ret) {
			dev_err(lwis_client->lwis_dev->dev,
				"Failed to write combined mask value\n");
			return ret;
		}
	}

	return 0;
}

int lwis_device_event_states_clear_locked(struct lwis_device *lwis_dev)
{
	struct lwis_device_event_state *state;
	struct hlist_node *n;
	int i;

	hash_for_each_safe(lwis_dev->event_states, i, n, state, node) {
		hash_del(&state->node);
		kfree(state);
	}

	return 0;
}

int lwis_device_event_flags_updated(struct lwis_device *lwis_dev, int64_t event_id,
				    uint64_t old_flags, uint64_t new_flags)
{
	struct lwis_device_event_state *state;
	int ret = 0;
	bool call_enable_cb = false, event_enabled = false;
	/* Flags for irqsave */
	unsigned long flags;
	/* Find our state */
	state = lwis_device_event_state_find_or_create(lwis_dev, event_id);
	/* Could not find or create one */
	if (IS_ERR_OR_NULL(state)) {
		dev_err(lwis_dev->dev, "Could not find or create device event state\n");
		return PTR_ERR(state);
	}
	/* Disable IRQs and lock the lock */
	spin_lock_irqsave(&lwis_dev->lock, flags);
	/* Are we turning on the event? */
	if ((!(old_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE) ||
	     !(old_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE_ONCE)) &&
	    ((new_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE) ||
	     (new_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE_ONCE))) {
		state->enable_counter++;
		event_enabled = true;
		call_enable_cb = (state->enable_counter == 1);

	} else if (((old_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE) ||
		    (old_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE_ONCE)) &&
		   (!(new_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE) ||
		    !(new_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE_ONCE))) {
		state->enable_counter--;
		event_enabled = false;
		call_enable_cb = (state->enable_counter == 0);
	}
	/* Restore the lock */
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
	/* Handle event being enabled or disabled */
	if (call_enable_cb) {
		/* Call our handler dispatcher */
		ret = lwis_device_event_enable(lwis_dev, event_id, event_enabled);
		if (ret) {
			dev_err(lwis_dev->dev, "Failed to %s event: 0x%llx (err:%d)\n",
				event_enabled ? "enable" : "disable", event_id, ret);
			return ret;
		}

		/* Reset hw event counter if hw event has been disabled */
		if (!event_enabled)
			state->event_counter = 0;
	}

	/* Reset sw event counter when it's going to disable */
	if ((event_id & LWIS_TRANSACTION_EVENT_FLAG ||
	     event_id & LWIS_TRANSACTION_FAILURE_EVENT_FLAG) &&
	    new_flags == 0)
		state->event_counter = 0;

	/* Check if our specialization cares about flags updates */
	if (lwis_dev->vops.event_flags_updated) {
		ret = lwis_dev->vops.event_flags_updated(lwis_dev, event_id, old_flags, new_flags);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Failed updating flags:	%lld %llx -> %llx (err:%d)\n", event_id,
				old_flags, new_flags, ret);
		}
	}

	return ret;
}

int lwis_device_event_enable(struct lwis_device *lwis_dev, int64_t event_id, bool enabled)
{
	int ret = -EINVAL, err = 0;
	/*
	 * The event_id includes 16 bits flags + 16 bits device id + 32 bits event code.
	 * For generic event, the flags will be 0. We do not care about the device id
	 * when we handle generic event. Use a mask here to filter out the device id.
	 */
	int64_t generic_event_id = event_id & 0xFFFF0000FFFFFFFFll;
	/* Generic events */
	if (generic_event_id < LWIS_EVENT_ID_START_OF_SPECIALIZED_RANGE) {
		ret = 0;
		switch (generic_event_id) {
		case LWIS_EVENT_ID_HEARTBEAT: {
			if (enabled)
				mod_timer(&lwis_dev->heartbeat_timer, jiffies);
			else
				del_timer(&lwis_dev->heartbeat_timer);
			break;
		}
		default: {
			/* We treat this as a real error because there really
			 * shouldn't be anything else handling generic events
			 */
			ret = err = -ENOENT;
			dev_err(lwis_dev->dev, "Unknown generic event: %lld\n", event_id);
		}
		};
		/* Non-transaction events */
	} else if ((event_id & LWIS_TRANSACTION_EVENT_FLAG) == 0) {
		if (lwis_dev->irqs) {
			ret = lwis_interrupt_event_enable(lwis_dev->irqs, event_id, enabled);
			if (ret && ret != -EINVAL) {
				dev_err(lwis_dev->dev, "Failed to %s IRQ event: %lld (e:%d)\n",
					enabled ? "enable" : "disable", event_id, ret);
				err = ret;
			}
		}

		if (lwis_dev->irq_gpios_info.irq_list) {
			ret = lwis_interrupt_event_enable(lwis_dev->irq_gpios_info.irq_list,
							  event_id, enabled);
			if (ret && ret != -EINVAL) {
				dev_err(lwis_dev->dev, "Failed to %s GPIO IRQ event: %lld (e:%d)\n",
					enabled ? "enable" : "disable", event_id, ret);
				err = ret;
			}
		}
	}
	/* Check if our specialization cares about event updates */
	if (!err && lwis_dev->vops.event_enable) {
		ret = lwis_dev->vops.event_enable(lwis_dev, event_id, enabled);
		if (ret && ret != -EINVAL) {
			dev_err(lwis_dev->dev, "Failed to %s event: %lld (err:%d)\n",
				enabled ? "enable" : "disable", event_id, ret);
			err = ret;
		}
	}
	return err ? err : ret;
}

static int device_event_emit_impl(struct lwis_device *lwis_dev, int64_t event_id, void *payload,
				  size_t payload_size, struct list_head *pending_events)
{
	struct lwis_client_event_state *client_event_state;
	struct lwis_device_event_state *device_event_state;
	struct lwis_event_entry *event;
	/* Our iterators */
	struct lwis_client *lwis_client;
	struct list_head *p, *n;
	struct lwis_top_device *top_dev;
	int64_t timestamp;
	int64_t event_counter;
	/* Flags for IRQ disable */
	unsigned long flags;
	bool has_subscriber;
	int ret;

	/* Lock and disable to prevent event_states from changing */
	spin_lock_irqsave(&lwis_dev->lock, flags);

	device_event_state = device_event_state_find_locked(lwis_dev, event_id);
	if (IS_ERR_OR_NULL(device_event_state)) {
		dev_err(lwis_dev->dev, "Device event state not found 0x%llx\n", event_id);
		spin_unlock_irqrestore(&lwis_dev->lock, flags);
		return -EINVAL;
	}

	/* Increment the event counter */
	device_event_state->event_counter++;
	/* Save event counter to local variable */
	event_counter = device_event_state->event_counter;
	/* Latch timestamp */
	timestamp = ktime_to_ns(lwis_get_time());
	/* Saves this event to history buffer */
	save_device_event_state_to_history_locked(lwis_dev, device_event_state, timestamp);

	has_subscriber = device_event_state->has_subscriber;

	/* Unlock and restore device lock */
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	/* Emit event to subscriber via top device */
	if (has_subscriber) {
		top_dev = container_of(lwis_dev->top_dev, struct lwis_top_device, base_dev);
		top_dev->subscribe_ops.notify_event_subscriber(lwis_dev->top_dev, event_id,
							       event_counter, timestamp);
	}

	/* Run internal handler if any */
	if (lwis_dev->vops.event_emitted) {
		ret = lwis_dev->vops.event_emitted(lwis_dev, event_id, &payload, &payload_size);
		if (ret)
			dev_warn(lwis_dev->dev, "Warning: vops.event_emitted returned %d\n", ret);
	}

	spin_lock_irqsave(&lwis_dev->lock, flags);
	/* Notify clients */
	list_for_each_safe(p, n, &lwis_dev->clients) {
		bool emit = false;
		unsigned long event_flags;

		lwis_client = list_entry(p, struct lwis_client, node);
		/* Lock the event lock instead */
		spin_lock_irqsave(&lwis_client->event_lock, event_flags);
		client_event_state = client_event_state_find_locked(lwis_client, event_id);

		if (!IS_ERR_OR_NULL(client_event_state)) {
			if (client_event_state->event_control.flags &
			    LWIS_EVENT_CONTROL_FLAG_QUEUE_ENABLE) {
				emit = true;
			}
		}
		/* Restore the event lock */
		spin_unlock_irqrestore(&lwis_client->event_lock, event_flags);

		if (emit) {
			event = kmalloc(sizeof(struct lwis_event_entry) + payload_size, GFP_ATOMIC);
			if (!event) {
				spin_unlock_irqrestore(&lwis_dev->lock, flags);
				return -ENOMEM;
			}

			event->event_info.event_id = event_id;
			event->event_info.event_counter = event_counter;
			event->event_info.timestamp_ns = timestamp;
			event->event_info.payload_size = payload_size;
			if (payload_size > 0) {
				event->event_info.payload_buffer =
					(void *)((uint8_t *)event +
						 sizeof(struct lwis_event_entry));
				memcpy(event->event_info.payload_buffer, payload, payload_size);
			} else {
				event->event_info.payload_buffer = NULL;
			}
			spin_unlock_irqrestore(&lwis_dev->lock, flags);
			ret = client_event_push_back(lwis_client, event);
			spin_lock_irqsave(&lwis_dev->lock, flags);
			if (ret) {
				lwis_dev_err_ratelimited(
					lwis_dev->dev,
					"Failed to push event to queue: ID 0x%llx Counter %lld\n",
					event_id, event_counter);
				kfree(event);
				spin_unlock_irqrestore(&lwis_dev->lock, flags);
				return ret;
			}
		}

		/* Trigger transactions, if there's any that matches this event
		 * ID and counter
		 */
		spin_unlock_irqrestore(&lwis_dev->lock, flags);
		if (lwis_transaction_event_trigger(lwis_client, event_id, event_counter, timestamp,
						   pending_events)) {
			dev_warn(lwis_dev->dev,
				 "Failed to process transactions: Event ID: 0x%llx Counter: %lld\n",
				 event_id, event_counter);
		}
		spin_lock_irqsave(&lwis_dev->lock, flags);
	}
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	return 0;
}

int lwis_device_event_emit(struct lwis_device *lwis_dev, int64_t event_id, void *payload,
			   size_t payload_size)
{
	int ret;
	struct list_head pending_events;

	/* Container to store events that are triggered as a result of this event. */
	INIT_LIST_HEAD(&pending_events);

	/* Emit the original event */
	ret = device_event_emit_impl(lwis_dev, event_id, payload, payload_size, &pending_events);
	if (ret) {
		lwis_dev_err_ratelimited(lwis_dev->dev,
					 "device_event_emit_impl failed: event ID 0x%llx\n",
					 event_id);
		return ret;
	}

	/* Emit pending events */
	return lwis_pending_events_emit(lwis_dev, &pending_events);
}

int lwis_pending_event_push(struct list_head *pending_events, int64_t event_id, void *payload,
			    size_t payload_size)
{
	struct lwis_event_entry *event;

	event = kzalloc(sizeof(struct lwis_event_entry) + payload_size, GFP_ATOMIC);
	if (!event)
		return -ENOMEM;

	event->event_info.event_id = event_id;
	event->event_info.payload_size = payload_size;
	if (payload_size > 0) {
		event->event_info.payload_buffer =
			(void *)((uint8_t *)event + sizeof(struct lwis_event_entry));
		memcpy(event->event_info.payload_buffer, payload, payload_size);
	} else {
		event->event_info.payload_buffer = NULL;
	}

	list_add_tail(&event->node, pending_events);

	return 0;
}

int lwis_pending_events_emit(struct lwis_device *lwis_dev, struct list_head *pending_events)
{
	int return_val = 0, emit_result = 0;
	struct lwis_event_entry *event;

	while (!list_empty(pending_events)) {
		event = list_first_entry(pending_events, struct lwis_event_entry, node);
		emit_result =
			device_event_emit_impl(lwis_dev, event->event_info.event_id,
					       event->event_info.payload_buffer,
					       event->event_info.payload_size, pending_events);
		if (emit_result) {
			return_val = emit_result;
			dev_warn_ratelimited(lwis_dev->dev,
					     "lwis_device_pending_event_emit error on ID 0x%llx\n",
					     event->event_info.event_id);
		}
		list_del(&event->node);
		kfree(event);
	}
	return return_val;
}

int lwis_device_event_update_subscriber(struct lwis_device *lwis_dev, int64_t event_id,
					bool has_subscriber)
{
	int ret = 0;
	unsigned long flags;
	struct lwis_device_event_state *event_state;

	spin_lock_irqsave(&lwis_dev->lock, flags);
	event_state = device_event_state_find_locked(lwis_dev, event_id);
	if (event_state == NULL) {
		dev_err(lwis_dev->dev, "Event not found in trigger device");
		ret = -EINVAL;
		goto out;
	}
	if (event_state->has_subscriber != has_subscriber) {
		event_state->has_subscriber = has_subscriber;
		dev_info(lwis_dev->dev, "Event: %llx, has subscriber: %d", event_id,
			 has_subscriber);
	}
out:
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
	return ret;
}

void lwis_device_external_event_emit(struct lwis_device *lwis_dev, int64_t event_id,
				     int64_t event_counter, int64_t timestamp)
{
	struct lwis_client_event_state *client_event_state;
	struct lwis_device_event_state *device_event_state;
	struct lwis_event_entry *event;
	/* Our iterators */
	struct lwis_client *lwis_client;
	struct list_head *p, *n;
	struct list_head pending_events;
	/* Flags for IRQ disable */
	unsigned long flags;

	INIT_LIST_HEAD(&pending_events);

	device_event_state = lwis_device_event_state_find(lwis_dev, event_id);
	if (IS_ERR_OR_NULL(device_event_state)) {
		dev_err(lwis_dev->dev, "Device external event state not found %llx\n", event_id);
		return;
	}
	/* Lock and disable to prevent event_states from changing */
	spin_lock_irqsave(&lwis_dev->lock, flags);

	/* Update event counter */
	device_event_state->event_counter = event_counter;

	/* Unlock and restore device lock */
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	/* Notify clients */
	list_for_each_safe(p, n, &lwis_dev->clients) {
		bool emit = false;
		unsigned long event_flags;
		lwis_client = list_entry(p, struct lwis_client, node);

		/* Lock the event lock instead */
		spin_lock_irqsave(&lwis_client->event_lock, event_flags);
		client_event_state = client_event_state_find_locked(lwis_client, event_id);

		if (!IS_ERR_OR_NULL(client_event_state)) {
			if (client_event_state->event_control.flags &
			    LWIS_EVENT_CONTROL_FLAG_QUEUE_ENABLE) {
				emit = true;
			}
		}
		/* Restore the event lock */
		spin_unlock_irqrestore(&lwis_client->event_lock, event_flags);

		if (emit) {
			event = kmalloc(sizeof(struct lwis_event_entry), GFP_ATOMIC);
			if (!event)
				return;

			event->event_info.event_id = event_id;
			event->event_info.event_counter = event_counter;
			event->event_info.timestamp_ns = timestamp;
			event->event_info.payload_size = 0;
			event->event_info.payload_buffer = NULL;
			if (client_event_push_back(lwis_client, event)) {
				lwis_dev_err_ratelimited(
					lwis_dev->dev,
					"Failed to push event to queue: ID 0x%llx Counter %lld\n",
					event_id, event_counter);
				kfree(event);
				return;
			}
		}

		if (lwis_transaction_event_trigger(lwis_client, event_id, event_counter, timestamp,
						   &pending_events))
			dev_warn(
				lwis_dev->dev,
				"Failed to process transactions: external event ID: 0x%llx counter: %lld\n",
				event_id, event_counter);
	}
	lwis_pending_events_emit(lwis_dev, &pending_events);
}

void lwis_device_error_event_emit(struct lwis_device *lwis_dev, int64_t event_id, void *payload,
				  size_t payload_size)
{
	struct lwis_event_entry *event;
	/* Our iterators */
	struct lwis_client *lwis_client;
	struct list_head *p, *n;
	int64_t timestamp;
	unsigned long flags;

	if (event_id < LWIS_EVENT_ID_START_OF_ERROR_RANGE ||
	    event_id >= LWIS_EVENT_ID_START_OF_SPECIALIZED_RANGE) {
		pr_err("Event ID %lld is not in the error event range\n", event_id);
		return;
	}

	/* Latch timestamp */
	timestamp = ktime_to_ns(lwis_get_time());

	spin_lock_irqsave(&lwis_dev->lock, flags);
	/* Notify clients */
	list_for_each_safe(p, n, &lwis_dev->clients) {
		lwis_client = list_entry(p, struct lwis_client, node);

		event = kmalloc(sizeof(struct lwis_event_entry) + payload_size, GFP_ATOMIC);
		if (!event) {
			spin_unlock_irqrestore(&lwis_dev->lock, flags);
			return;
		}

		event->event_info.event_id = event_id;
		event->event_info.event_counter = 0;
		event->event_info.timestamp_ns = timestamp;
		event->event_info.payload_size = payload_size;
		if (payload_size > 0) {
			event->event_info.payload_buffer =
				(void *)((uint8_t *)event + sizeof(struct lwis_event_entry));
			memcpy(event->event_info.payload_buffer, payload, payload_size);
		} else {
			event->event_info.payload_buffer = NULL;
		}
		if (client_error_event_push_back(lwis_client, event)) {
			lwis_dev_err_ratelimited(lwis_dev->dev,
						 "Failed to push error event to queue: ID 0x%llx\n",
						 event_id);
			kfree(event);
			spin_unlock_irqrestore(&lwis_dev->lock, flags);
			return;
		}
	}
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
}
