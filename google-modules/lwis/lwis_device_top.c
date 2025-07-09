// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Top Level Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-top-dev: " fmt

#include "lwis_device_top.h"

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "lwis_bus_manager.h"
#include "lwis_device.h"
#include "lwis_event.h"
#include "lwis_util.h"

#include "lwis_device.h"
#include "lwis_event.h"
#include "lwis_util.h"

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_DRIVER_NAME "lwis-top"
#define LWIS_SUBSCRIBER_THREAD_NAME "lwis_s_top"
/*
 * RT priority needed because events need to be transferred to
 * another device in low latency.
 */
#define SUBSCRIBE_THREAD_PRIORITY 99

static int lwis_top_register_io(struct lwis_device *lwis_dev, struct lwis_io_entry *entry,
				int access_size);
static int lwis_top_close(struct lwis_device *lwis_dev);
static struct lwis_device_subclass_operations top_vops = {
	.register_io = lwis_top_register_io,
	.register_io_barrier = NULL,
	.device_enable = NULL,
	.device_disable = NULL,
	.event_enable = NULL,
	.event_flags_updated = NULL,
	.close = lwis_top_close,
};

static int lwis_top_event_subscribe(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				    int trigger_device_id, int subscriber_device_id);
static int lwis_top_event_unsubscribe(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				      int subscriber_device_id);
static void lwis_top_event_notify(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				  int64_t trigger_event_count, int64_t trigger_event_timestamp);
static void lwis_top_event_subscribe_release(struct lwis_device *lwis_dev);
static struct lwis_event_subscribe_operations top_subscribe_ops = {
	.subscribe_event = lwis_top_event_subscribe,
	.unsubscribe_event = lwis_top_event_unsubscribe,
	.notify_event_subscriber = lwis_top_event_notify,
	.release = lwis_top_event_subscribe_release,
};

struct lwis_event_subscribe_info {
	/* Subscribed Event ID */
	int64_t event_id;
	/* LWIS device who subscribed an event */
	struct lwis_device *subscriber_dev;
	/* LWIS device who will trigger an event */
	struct lwis_device *trigger_dev;
	/* Node in the lwis_top_device->event_subscribers list */
	struct list_head list_node;
	/* List of event subscriber info */
	struct lwis_event_subscriber_list *event_subscriber_list;
};

struct lwis_event_subscriber_list {
	int64_t trigger_event_id;
	struct list_head list;
	struct hlist_node node;
};

struct lwis_trigger_event_info {
	/* Store emitted event id from trigger device */
	int64_t trigger_event_id;
	/* Store emitted event count from trigger device */
	int64_t trigger_event_count;
	/* Store emitted event timestamp from trigger device */
	int64_t trigger_event_timestamp;
	/* node of list head */
	struct list_head node;
};

static struct lwis_event_subscriber_list *event_subscriber_list_find(struct lwis_device *lwis_dev,
								     int64_t trigger_event_id)
{
	struct lwis_top_device *lwis_top_dev =
		container_of(lwis_dev, struct lwis_top_device, base_dev);
	struct lwis_event_subscriber_list *list;

	hash_for_each_possible(lwis_top_dev->event_subscribers, list, node, trigger_event_id) {
		if (list->trigger_event_id == trigger_event_id)
			return list;
	}
	return NULL;
}

static struct lwis_event_subscriber_list *event_subscriber_list_create(struct lwis_device *lwis_dev,
								       int64_t trigger_event_id)
{
	struct lwis_top_device *lwis_top_dev =
		container_of(lwis_dev, struct lwis_top_device, base_dev);
	struct lwis_event_subscriber_list *event_subscriber_list =
		kmalloc(sizeof(struct lwis_event_subscriber_list), GFP_ATOMIC);
	if (!event_subscriber_list)
		return NULL;

	event_subscriber_list->trigger_event_id = trigger_event_id;
	INIT_LIST_HEAD(&event_subscriber_list->list);
	hash_add(lwis_top_dev->event_subscribers, &event_subscriber_list->node, trigger_event_id);
	return event_subscriber_list;
}

static struct lwis_event_subscriber_list *
event_subscriber_list_find_or_create(struct lwis_device *lwis_dev, int64_t trigger_event_id)
{
	struct lwis_event_subscriber_list *list =
		event_subscriber_list_find(lwis_dev, trigger_event_id);
	return (list == NULL) ? event_subscriber_list_create(lwis_dev, trigger_event_id) : list;
}

static void subscribe_work_func(struct kthread_work *work)
{
	struct lwis_top_device *lwis_top_dev =
		container_of(work, struct lwis_top_device, subscribe_work);
	struct lwis_trigger_event_info *trigger_event;
	struct lwis_event_subscribe_info *subscribe_info;
	struct list_head *it_sub, *it_sub_tmp;
	struct lwis_event_subscriber_list *event_subscriber_list;
	struct list_head *it_event_subscriber, *it_event_subscriber_tmp;
	unsigned long flags;

	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);
	list_for_each_safe(it_sub, it_sub_tmp, &lwis_top_dev->emitted_event_list_work) {
		trigger_event = list_entry(it_sub, struct lwis_trigger_event_info, node);
		list_del(&trigger_event->node);
		event_subscriber_list = event_subscriber_list_find(&lwis_top_dev->base_dev,
								   trigger_event->trigger_event_id);
		if (!event_subscriber_list || list_empty(&event_subscriber_list->list)) {
			dev_err(lwis_top_dev->base_dev.dev,
				"Failed to find event subscriber list for %llx\n",
				trigger_event->trigger_event_id);
			kfree(trigger_event);
			continue;
		}
		list_for_each_safe(it_event_subscriber, it_event_subscriber_tmp,
				   &event_subscriber_list->list) {
			subscribe_info = list_entry(it_event_subscriber,
						    struct lwis_event_subscribe_info, list_node);
			/* Notify subscriber an event is happening */
			lwis_device_external_event_emit(subscribe_info->subscriber_dev,
							trigger_event->trigger_event_id,
							trigger_event->trigger_event_count,
							trigger_event->trigger_event_timestamp);
		}
		kfree(trigger_event);
	}
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
}

static void lwis_top_event_notify(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				  int64_t trigger_event_count, int64_t trigger_event_timestamp)
{
	struct lwis_top_device *lwis_top_dev =
		container_of(lwis_dev, struct lwis_top_device, base_dev);
	unsigned long flags;

	struct lwis_trigger_event_info *trigger_event =
		kmalloc(sizeof(struct lwis_trigger_event_info), GFP_ATOMIC);
	if (trigger_event == NULL)
		return;

	INIT_LIST_HEAD(&trigger_event->node);
	trigger_event->trigger_event_id = trigger_event_id;
	trigger_event->trigger_event_count = trigger_event_count;
	trigger_event->trigger_event_timestamp = trigger_event_timestamp;
	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);
	list_add_tail(&trigger_event->node, &lwis_top_dev->emitted_event_list_work);
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
	/* Schedule deferred subscribed events */
	kthread_queue_work(&lwis_top_dev->subscribe_worker, &lwis_top_dev->subscribe_work);
}

static int lwis_top_event_subscribe(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				    int trigger_device_id, int subscriber_device_id)
{
	struct lwis_top_device *lwis_top_dev =
		container_of(lwis_dev, struct lwis_top_device, base_dev);
	struct lwis_device *lwis_trigger_dev = lwis_find_dev_by_id(trigger_device_id);
	struct lwis_device *lwis_subscriber_dev = lwis_find_dev_by_id(subscriber_device_id);
	struct lwis_event_subscribe_info *old_subscription;
	struct lwis_event_subscribe_info *new_subscription;
	struct lwis_event_subscriber_list *event_subscriber_list;
	struct list_head *it_event_subscriber;
	unsigned long flags;
	int ret = 0;
	bool has_subscriber = true;

	if (lwis_trigger_dev == NULL || lwis_subscriber_dev == NULL) {
		dev_err(lwis_top_dev->base_dev.dev, "LWIS trigger/subscriber device not found");
		return -EINVAL;
	}

	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);
	event_subscriber_list = event_subscriber_list_find_or_create(lwis_dev, trigger_event_id);
	if (!event_subscriber_list) {
		spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
		dev_err(lwis_dev->dev, "Can't find/create event subscriber list\n");
		return -EINVAL;
	}

	list_for_each(it_event_subscriber, &event_subscriber_list->list) {
		old_subscription = list_entry(it_event_subscriber, struct lwis_event_subscribe_info,
					      list_node);
		/* Event already registered for this device */
		if (old_subscription->subscriber_dev->id == subscriber_device_id) {
			spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
			dev_info(
				lwis_dev->dev,
				"Already subscribed event: %llx, trigger device: %s, subscriber device: %s\n",
				trigger_event_id, old_subscription->trigger_dev->name,
				old_subscription->subscriber_dev->name);
			return 0;
		}
	}
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);

	/* If the subscription does not exist in hash table, create one */
	new_subscription = kmalloc(sizeof(struct lwis_event_subscribe_info), GFP_KERNEL);
	if (!new_subscription)
		return -ENOMEM;

	INIT_LIST_HEAD(&new_subscription->list_node);
	new_subscription->event_id = trigger_event_id;
	new_subscription->subscriber_dev = lwis_subscriber_dev;
	new_subscription->trigger_dev = lwis_trigger_dev;
	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);
	list_add_tail(&new_subscription->list_node, &event_subscriber_list->list);
	new_subscription->event_subscriber_list = event_subscriber_list;
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
	dev_info(lwis_dev->dev, "Subscribe event: %llx, trigger device: %s, subscriber device: %s",
		 trigger_event_id, lwis_trigger_dev->name, lwis_subscriber_dev->name);

	ret = lwis_device_event_update_subscriber(lwis_trigger_dev, trigger_event_id,
						  has_subscriber);
	if (ret < 0)
		dev_err(lwis_top_dev->base_dev.dev, "Failed to subcribe event : %llx\n",
			trigger_event_id);

	return ret;
}

static int lwis_top_event_unsubscribe(struct lwis_device *lwis_dev, int64_t trigger_event_id,
				      int subscriber_device_id)
{
	struct lwis_top_device *lwis_top_dev =
		container_of(lwis_dev, struct lwis_top_device, base_dev);
	struct lwis_device *trigger_dev = NULL;
	struct lwis_event_subscribe_info *subscribe_info = NULL;
	struct lwis_event_subscriber_list *event_subscriber_list;
	struct list_head *it_event_subscriber, *it_event_subscriber_tmp;
	struct lwis_trigger_event_info *pending_event, *n;
	unsigned long flags;
	bool has_subscriber = false;

	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);
	event_subscriber_list = event_subscriber_list_find(lwis_dev, trigger_event_id);
	if (!event_subscriber_list || list_empty(&event_subscriber_list->list)) {
		spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
		dev_err(lwis_top_dev->base_dev.dev,
			"Failed to find event subscriber list for %llx\n", trigger_event_id);
		return -EINVAL;
	}

	list_for_each_safe(it_event_subscriber, it_event_subscriber_tmp,
			   &event_subscriber_list->list) {
		subscribe_info = list_entry(it_event_subscriber, struct lwis_event_subscribe_info,
					    list_node);
		if (subscribe_info->subscriber_dev->id == subscriber_device_id) {
			dev_info(
				lwis_dev->dev,
				"Unsubscribe event: %llx, trigger device: %s, subscriber device: %s\n",
				trigger_event_id, subscribe_info->trigger_dev->name,
				subscribe_info->subscriber_dev->name);
			trigger_dev = subscribe_info->trigger_dev;
			list_del(&subscribe_info->list_node);
			if (list_empty(&subscribe_info->event_subscriber_list->list)) {
				/* Clear pending events */
				list_for_each_entry_safe(pending_event, n,
							 &lwis_top_dev->emitted_event_list_work,
							 node) {
					if (pending_event->trigger_event_id == trigger_event_id) {
						list_del(&pending_event->node);
						kfree(pending_event);
					}
				}
				hash_del(&subscribe_info->event_subscriber_list->node);
				kfree(subscribe_info->event_subscriber_list);
				kfree(subscribe_info);
				spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
				lwis_device_event_update_subscriber(trigger_dev, trigger_event_id,
								    has_subscriber);
				return 0;
			}
			kfree(subscribe_info);
		}
	}
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);
	return 0;
}

static void top_event_subscribe_init(struct lwis_top_device *lwis_top_dev)
{
	hash_init(lwis_top_dev->event_subscribers);
	INIT_LIST_HEAD(&lwis_top_dev->emitted_event_list_work);
	kthread_init_work(&lwis_top_dev->subscribe_work, subscribe_work_func);
}

static void top_event_subscribe_clear(struct lwis_top_device *lwis_top_dev)
{
	struct lwis_event_subscriber_list *event_subscriber_list;
	struct list_head *it_event_subscriber, *it_event_subscriber_tmp;
	struct lwis_event_subscribe_info *subscribe_info;
	struct hlist_node *tmp;
	struct lwis_trigger_event_info *pending_event, *n;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&lwis_top_dev->base_dev.lock, flags);
	/* Clean up subscription table */
	hash_for_each_safe(lwis_top_dev->event_subscribers, i, tmp, event_subscriber_list, node) {
		list_for_each_safe(it_event_subscriber, it_event_subscriber_tmp,
				   &event_subscriber_list->list) {
			subscribe_info = list_entry(it_event_subscriber,
						    struct lwis_event_subscribe_info, list_node);
			/* Delete the node from the hash table */
			list_del(&subscribe_info->list_node);
			if (list_empty(&subscribe_info->event_subscriber_list->list)) {
				hash_del(&subscribe_info->event_subscriber_list->node);
				kfree(subscribe_info->event_subscriber_list);
			}
			kfree(subscribe_info);
		}
	}

	/* Clean up emitted event list */
	list_for_each_entry_safe(pending_event, n, &lwis_top_dev->emitted_event_list_work, node) {
		list_del(&pending_event->node);
		kfree(pending_event);
	}
	spin_unlock_irqrestore(&lwis_top_dev->base_dev.lock, flags);

	if (lwis_top_dev->subscribe_worker_thread)
		kthread_flush_worker(&lwis_top_dev->subscribe_worker);
}

static void lwis_top_event_subscribe_release(struct lwis_device *lwis_dev)
{
	struct lwis_top_device *lwis_top_dev =
		container_of(lwis_dev, struct lwis_top_device, base_dev);

	/* Clean up subscription work */
	top_event_subscribe_clear(lwis_top_dev);
}

static int lwis_top_register_io(struct lwis_device *lwis_dev, struct lwis_io_entry *entry,
				int access_size)
{
	struct lwis_top_device *top_dev = container_of(lwis_dev, struct lwis_top_device, base_dev);

	if (!entry) {
		dev_err(top_dev->base_dev.dev, "IO entry is NULL.\n");
		return -EINVAL;
	}
	lwis_save_register_io_info(lwis_dev, entry, access_size);

	switch (entry->type) {
	case LWIS_IO_ENTRY_READ:
	case LWIS_IO_ENTRY_READ_V2: {
		struct lwis_io_entry_rw *rw;

		rw = &entry->rw;
		if (rw->offset >= SCRATCH_MEMORY_SIZE) {
			dev_err(top_dev->base_dev.dev, "Offset (%llu) must be < %d\n", rw->offset,
				SCRATCH_MEMORY_SIZE);
			return -EINVAL;
		}
		rw->val = top_dev->scratch_mem[rw->offset];
		break;
	}
	case LWIS_IO_ENTRY_READ_BATCH:
	case LWIS_IO_ENTRY_READ_BATCH_V2: {
		struct lwis_io_entry_rw_batch *rw_batch;

		rw_batch = &entry->rw_batch;
		if (rw_batch->offset + rw_batch->size_in_bytes >= SCRATCH_MEMORY_SIZE) {
			dev_err(top_dev->base_dev.dev,
				"Read range[offset(%llu) + size_in_bytes(%zu)] exceeds scratch memory (%d)\n",
				rw_batch->offset, rw_batch->size_in_bytes, SCRATCH_MEMORY_SIZE);
			return -EINVAL;
		}
		memcpy(rw_batch->buf, &top_dev->scratch_mem[rw_batch->offset],
		       rw_batch->size_in_bytes);
		break;
	}
	case LWIS_IO_ENTRY_WRITE:
	case LWIS_IO_ENTRY_WRITE_V2: {
		struct lwis_io_entry_rw *rw;

		rw = &entry->rw;
		if (rw->offset >= SCRATCH_MEMORY_SIZE) {
			dev_err(top_dev->base_dev.dev, "Offset (%llu) must be < %d\n", rw->offset,
				SCRATCH_MEMORY_SIZE);
			return -EINVAL;
		}
		top_dev->scratch_mem[rw->offset] = rw->val;
		break;
	}
	case LWIS_IO_ENTRY_WRITE_BATCH:
	case LWIS_IO_ENTRY_WRITE_BATCH_V2: {
		struct lwis_io_entry_rw_batch *rw_batch;

		rw_batch = &entry->rw_batch;
		if (rw_batch->offset + rw_batch->size_in_bytes >= SCRATCH_MEMORY_SIZE) {
			dev_err(top_dev->base_dev.dev,
				"Write range[offset(%llu) + size_in_bytes(%zu)] exceeds scratch memory (%d)\n",
				rw_batch->offset, rw_batch->size_in_bytes, SCRATCH_MEMORY_SIZE);
			return -EINVAL;
		}
		memcpy(&top_dev->scratch_mem[rw_batch->offset], rw_batch->buf,
		       rw_batch->size_in_bytes);
		break;
	}
	case LWIS_IO_ENTRY_MODIFY: {
		struct lwis_io_entry_modify *mod;
		uint64_t reg_value;

		mod = &entry->mod;
		if (mod->offset >= SCRATCH_MEMORY_SIZE) {
			dev_err(top_dev->base_dev.dev, "Offset (%llu) must be < %d\n", mod->offset,
				SCRATCH_MEMORY_SIZE);
			return -EINVAL;
		}
		reg_value = top_dev->scratch_mem[mod->offset];
		reg_value &= ~mod->val_mask;
		reg_value |= mod->val_mask & mod->val;
		top_dev->scratch_mem[mod->offset] = reg_value;
		break;
	}
	case LWIS_IO_ENTRY_IGNORE: {
		break;
	}
	default:
		dev_err(top_dev->base_dev.dev, "Invalid IO entry type: %d\n", entry->type);
		return -EINVAL;
	}

	return 0;
}

static int lwis_top_close(struct lwis_device *lwis_dev)
{
	struct lwis_top_device *lwis_top_dev =
		container_of(lwis_dev, struct lwis_top_device, base_dev);

	top_event_subscribe_clear(lwis_top_dev);
	return 0;
}

static int top_device_setup(struct lwis_top_device *top_dev)
{
	int ret = 0;

#ifdef CONFIG_OF
	/* Parse device tree for device configurations */
	ret = lwis_top_device_parse_dt(top_dev);
	if (ret)
		dev_err(top_dev->base_dev.dev, "Failed to parse device tree\n");
#else
	/* Non-device-tree init: Save for future implementation */
	ret = -EINVAL;
#endif

	return ret;
}

static int lwis_top_device_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct lwis_top_device *top_dev;
	struct device *dev = &plat_dev->dev;

	/* Allocate top device specific data construct */
	top_dev = devm_kzalloc(dev, sizeof(struct lwis_top_device), GFP_KERNEL);
	if (!top_dev)
		return -ENOMEM;

	top_dev->base_dev.type = DEVICE_TYPE_TOP;
	top_dev->base_dev.vops = top_vops;
	top_dev->subscribe_ops = top_subscribe_ops;
	top_dev->base_dev.plat_dev = plat_dev;
	top_dev->base_dev.k_dev = &plat_dev->dev;
	top_dev->transaction_worker_active = false;

	/* Call the base device probe function */
	ret = lwis_base_probe(&top_dev->base_dev);
	if (ret) {
		dev_err(dev, "Error in lwis base probe\n");
		return ret;
	}
	platform_set_drvdata(plat_dev, &top_dev->base_dev);

	/* Call top device specific setup function */
	ret = top_device_setup(top_dev);
	if (ret) {
		dev_err(top_dev->base_dev.dev, "Error in top device initialization\n");
		lwis_base_unprobe(&top_dev->base_dev);
		return ret;
	}

	top_event_subscribe_init(top_dev);

	kthread_init_worker(&top_dev->subscribe_worker);
	top_dev->subscribe_worker_thread = kthread_run(
		kthread_worker_fn, &top_dev->subscribe_worker, LWIS_SUBSCRIBER_THREAD_NAME);
	if (IS_ERR_OR_NULL(top_dev->subscribe_worker_thread)) {
		dev_err(top_dev->base_dev.dev, "subscribe kthread_run failed\n");
		return ret;
	}

	ret = lwis_set_kthread_priority(&top_dev->base_dev, top_dev->subscribe_worker_thread,
					SUBSCRIBE_THREAD_PRIORITY);
	if (ret) {
		dev_err(top_dev->base_dev.dev,
			"Failed to set LWIS top subscription kthread priority (%d)", ret);
		kthread_stop(top_dev->subscribe_worker_thread);
		lwis_base_unprobe(&top_dev->base_dev);
		return ret;
	}

	dev_info(top_dev->base_dev.dev, "Top Device Probe: Success\n");

	return 0;
}

void lwis_start_top_device_worker(struct lwis_client *client)
{
	int ret;
	struct lwis_device *lwis_dev = client->lwis_dev;
	struct lwis_top_device *top_dev = container_of(lwis_dev, struct lwis_top_device, base_dev);

	if (!top_dev->transaction_worker_active) {
		dev_info(top_dev->base_dev.dev, "Starting top device worker");
		/* Create associated kworker threads */
		ret = lwis_create_kthread_workers(&top_dev->base_dev);
		if (ret) {
			dev_err(top_dev->base_dev.dev, "Failed to create lwis_top_kthread");
			return;
		}
		top_dev->transaction_worker_active = true;
	}
}

void lwis_stop_top_device_worker(struct lwis_client *client)
{
	struct lwis_device *lwis_dev = client->lwis_dev;
	struct lwis_top_device *top_dev = container_of(lwis_dev, struct lwis_top_device, base_dev);

	if (top_dev->transaction_worker_active) {
		dev_info(top_dev->base_dev.dev, "Stopping top device worker");
		kthread_stop(client->lwis_dev->transaction_worker_thread);
		top_dev->transaction_worker_active = false;
	}
}

#ifdef CONFIG_OF
static const struct of_device_id lwis_id_match[] = {
	{ .compatible = LWIS_TOP_DEVICE_COMPAT },
	{},
};
// MODULE_DEVICE_TABLE(of, lwis_id_match);

static struct platform_driver lwis_driver = {
	.probe = lwis_top_device_probe,
	.driver = {
		.name = LWIS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lwis_id_match,
	},
};

#else /* CONFIG_OF not defined */
static struct platform_device_id lwis_driver_id[] = {
	{
		.name = LWIS_DRIVER_NAME,
		.driver_data = 0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, lwis_driver_id);

static struct platform_driver lwis_driver = { .probe = lwis_top_device_probe,
					      .id_table = lwis_driver_id,
					      .driver = {
						      .name = LWIS_DRIVER_NAME,
						      .owner = THIS_MODULE,
					      } };
#endif /* CONFIG_OF */

/*
 *  lwis_top_device_init: Init function that will be called by the kernel
 *  initialization routines.
 */
int __init lwis_top_device_init(void)
{
	int ret = 0;

	pr_info("Top device initialization\n");

	lwis_bus_manager_list_initialize();
	ret = platform_driver_register(&lwis_driver);
	if (ret)
		pr_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

int lwis_top_device_deinit(void)
{
	lwis_bus_manager_list_deinitialize();
	platform_driver_unregister(&lwis_driver);
	return 0;
}
