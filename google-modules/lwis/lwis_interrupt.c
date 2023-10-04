/*
 * Google LWIS Interrupt Handler
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-int: " fmt

#include "lwis_interrupt.h"

#include <linux/kernel.h>
#include <linux/slab.h>

#include "lwis_device.h"
#include "lwis_event.h"
#include "lwis_platform.h"
#include "lwis_transaction.h"
#include "lwis_util.h"

struct lwis_single_event_info {
	/* Event ID of the event we can emit */
	int64_t event_id;
	/* Bit # in the status/reset/mask registers */
	int int_reg_bit;
	/* If critical, print during ISR */
	bool is_critical;
	/* Reference to the device event state */
	struct lwis_device_event_state *state;
	/* Node in the lwis_interrupt->event_infos hash table */
	struct hlist_node node;
	/* Node in the lwis_interrupt->enabled_event_infos list */
	struct list_head node_enabled;
};

static irqreturn_t lwis_interrupt_event_isr(int irq_number, void *data);
static irqreturn_t lwis_interrupt_gpios_event_isr(int irq_number, void *data);

struct lwis_interrupt_list *lwis_interrupt_list_alloc(struct lwis_device *lwis_dev, int count)
{
	struct lwis_interrupt_list *list;

	/* No need to allocate if count is invalid */
	if (count <= 0) {
		return ERR_PTR(-EINVAL);
	}

	list = kmalloc(sizeof(struct lwis_interrupt_list), GFP_KERNEL);
	if (!list) {
		pr_err("Failed to allocate IRQ list\n");
		return ERR_PTR(-ENOMEM);
	}

	list->irq = kmalloc(count * sizeof(struct lwis_interrupt), GFP_KERNEL);
	if (!list->irq) {
		pr_err("Failed to allocate IRQs\n");
		kfree(list);
		return ERR_PTR(-ENOMEM);
	}

	list->count = count;
	list->lwis_dev = lwis_dev;

	return list;
}

void lwis_interrupt_list_free(struct lwis_interrupt_list *list)
{
	int i;
	if (!list) {
		return;
	}

	if (!list->irq) {
		kfree(list);
		return;
	}

	for (i = 0; i < list->count; ++i) {
		free_irq(list->irq[i].irq, &list->irq[i]);
	}
	kfree(list->irq);
}

int lwis_interrupt_get(struct lwis_interrupt_list *list, int index, char *name,
		       struct platform_device *plat_dev)
{
	int irq;
	int ret = 0;

	if (!list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	irq = platform_get_irq(plat_dev, index);
	if (irq <= 0) {
		pr_err("Error retriving interrupt %s at %d\n", name, index);
		return -EINVAL;
	}

	/* Initialize the spinlock */
	spin_lock_init(&list->irq[index].lock);
	list->irq[index].irq = irq;
	strlcpy(list->irq[index].name, name, IRQ_FULL_NAME_LENGTH);
	snprintf(list->irq[index].full_name, IRQ_FULL_NAME_LENGTH, "lwis-%s:%s",
		 list->lwis_dev->name, name);
	list->irq[index].has_events = false;
	list->irq[index].lwis_dev = list->lwis_dev;

	ret = request_irq(irq, lwis_interrupt_event_isr, IRQF_SHARED, list->irq[index].full_name,
			  &list->irq[index]);
	if (ret) {
		dev_err(list->lwis_dev->dev, "Failed to request IRQ %d\n", irq);
		return ret;
	}

	if (lwis_plaform_set_default_irq_affinity(list->irq[index].irq) != 0) {
		dev_warn(list->lwis_dev->dev, "Interrupt %s cannot set affinity.\n",
			 list->irq[index].full_name);
	}

	return 0;
}

int lwis_interrupt_get_gpio_irq(struct lwis_interrupt_list *list, int index, char *name,
				int gpio_irq)
{
	int ret = 0;

	if (!list || index < 0 || index >= list->count || gpio_irq <= 0) {
		return -EINVAL;
	}

	/* Initialize the spinlock */
	spin_lock_init(&list->irq[index].lock);
	list->irq[index].irq = gpio_irq;
	strlcpy(list->irq[index].name, name, IRQ_FULL_NAME_LENGTH);
	snprintf(list->irq[index].full_name, IRQ_FULL_NAME_LENGTH, "lwis-%s:%s",
		 list->lwis_dev->name, name);
	list->irq[index].has_events = false;
	list->irq[index].lwis_dev = list->lwis_dev;

	ret = request_irq(gpio_irq, lwis_interrupt_gpios_event_isr, IRQF_SHARED,
			  list->irq[index].full_name, &list->irq[index]);
	if (ret) {
		dev_err(list->lwis_dev->dev, "Failed to request GPIO IRQ\n");
		return ret;
	}

	if (lwis_plaform_set_default_irq_affinity(list->irq[index].irq) != 0) {
		dev_warn(list->lwis_dev->dev, "Interrupt %s cannot set affinity.\n",
			 list->irq[index].full_name);
	}

	return 0;
}

static struct lwis_single_event_info *
lwis_interrupt_get_single_event_info_locked(struct lwis_interrupt *irq, int64_t event_id)
{
	/* Our hash iterator */
	struct lwis_single_event_info *p;

	if (!irq) {
		pr_err("irq is NULL.\n");
		return NULL;
	}

	/* Iterate through the hash bucket for this event_id */
	hash_for_each_possible (irq->event_infos, p, node, event_id) {
		/* If it's indeed the right one, return it */
		if (p->event_id == event_id) {
			return p;
		}
	}
	return NULL;
}

static int lwis_interrupt_set_mask(struct lwis_interrupt *irq, int int_reg_bit, bool is_set)
{
	int ret = 0;
	uint64_t mask_value = 0;

	if (!irq) {
		pr_err("irq is NULL.\n");
		return -EINVAL;
	}

	/* Read the mask register */
	ret = lwis_device_single_register_read(irq->lwis_dev, irq->irq_reg_bid, irq->irq_mask_reg,
					       &mask_value, irq->irq_reg_access_size);
	if (ret) {
		pr_err("Failed to read IRQ mask register: %d\n", ret);
		return ret;
	}

	/* Unmask the interrupt */
	if (is_set) {
		mask_value |= (1ULL << int_reg_bit);
	} else {
		mask_value &= ~(1ULL << int_reg_bit);
	}

	/* Write the mask register */
	ret = lwis_device_single_register_write(irq->lwis_dev, irq->irq_reg_bid, irq->irq_mask_reg,
						mask_value, irq->irq_reg_access_size);
	if (ret) {
		pr_err("Failed to write IRQ mask register: %d\n", ret);
		return ret;
	}

	return ret;
}

static irqreturn_t lwis_interrupt_event_isr(int irq_number, void *data)
{
	int ret;
	struct lwis_interrupt *irq = (struct lwis_interrupt *)data;
	struct lwis_client_event_state *event_state;
	struct lwis_single_event_info *event;
	struct list_head *p;
	uint64_t source_value, reset_value = 0;
	struct lwis_client *lwis_client;
	struct list_head *t, *n;
#ifdef LWIS_INTERRUPT_DEBUG
	uint64_t mask_value;
#endif
	unsigned long flags;

	/* Read the mask register */
	ret = lwis_device_single_register_read(irq->lwis_dev, irq->irq_reg_bid, irq->irq_src_reg,
					       &source_value, irq->irq_reg_access_size);
	if (ret) {
		dev_err(irq->lwis_dev->dev, "%s: Failed to read IRQ status register: %d\n",
			irq->name, ret);
		goto error;
	}

	/* Write back to the reset register */
	ret = lwis_device_single_register_write(irq->lwis_dev, irq->irq_reg_bid, irq->irq_reset_reg,
						source_value, irq->irq_reg_access_size);
	if (ret) {
		dev_err(irq->lwis_dev->dev, "%s: Failed to write IRQ reset register: %d\n",
			irq->name, ret);
		goto error;
	}

	/* Nothing is triggered, just return */
	if (source_value == 0) {
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&irq->lock, flags);
	list_for_each (p, &irq->enabled_event_infos) {
		event = list_entry(p, struct lwis_single_event_info, node_enabled);

		/* Check if this event needs to be emitted */
		if ((source_value >> event->int_reg_bit) & 0x1) {
			/* Emit the event */
			lwis_device_event_emit(irq->lwis_dev, event->event_id, NULL, 0,
					       /*in_irq=*/true);
			/* Clear this interrupt */
			reset_value |= (1ULL << event->int_reg_bit);

			/* If considered critical, print the event */
			if (event->is_critical) {
				dev_err_ratelimited(irq->lwis_dev->dev,
						    "Caught critical IRQ(%s) event(0x%llx)\n",
						    irq->name, event->event_id);
			}
			/* If enabled once, set interrupt mask to false */
			list_for_each_safe (t, n, &irq->lwis_dev->clients) {
				lwis_client = list_entry(t, struct lwis_client, node);
				hash_for_each_possible (lwis_client->event_states, event_state,
							node, event->event_id) {
					if (event_state->event_control.event_id ==
						    event->event_id &&
					    event_state->event_control.flags &
						    LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE_ONCE) {
						dev_err(irq->lwis_dev->dev,
							"IRQ(%s) event(0x%llx) enabled once\n",
							irq->name, event->event_id);
						lwis_interrupt_set_mask(irq, event->int_reg_bit,
									false);
					}
				}
			}
		}

		/* All enabled and triggered interrupts are handled */
		if (source_value == reset_value) {
			break;
		}
	}
	spin_unlock_irqrestore(&irq->lock, flags);

#ifdef LWIS_INTERRUPT_DEBUG
	/* Make sure the number of interrupts triggered matches the number of
	 * events processed */
	if (source_value != reset_value) {
		lwis_device_single_register_read(irq->lwis_dev, irq->irq_reg_bid, irq->irq_mask_reg,
						 &mask_value, irq->irq_reg_access_size);

		/* This is to detect if there are extra bits set in the source
		 * than what we have enabled for (i.e. mask register) */
		/* Currently these are set to debug logs as some hardware blocks might behave differently
		 * and trigger these, which would result in unintentional log spew in ISRs. */
		if ((mask_value | source_value) != mask_value) {
			dev_dbg(irq->lwis_dev->dev,
				"%s: Spurious interrupt? mask 0x%llx src 0x%llx reset 0x%llx\n",
				irq->name, mask_value, source_value, reset_value);
		} else {
			dev_dbg(irq->lwis_dev->dev,
				"%s: Mismatched hw interrupt and LWIS event enable? mask 0x%llx src 0x%llx reset 0x%llx\n",
				irq->name, mask_value, source_value, reset_value);
		}
	}
#endif
error:
	return IRQ_HANDLED;
}

static irqreturn_t lwis_interrupt_gpios_event_isr(int irq_number, void *data)
{
	unsigned long flags;
	struct lwis_interrupt *irq = (struct lwis_interrupt *)data;
	struct lwis_single_event_info *event;
	struct list_head *p;

	spin_lock_irqsave(&irq->lock, flags);
	list_for_each (p, &irq->enabled_event_infos) {
		event = list_entry(p, struct lwis_single_event_info, node_enabled);
		/* Emit the event */
		lwis_device_event_emit(irq->lwis_dev, event->event_id, NULL, 0, /*in_irq=*/true);
	}
	spin_unlock_irqrestore(&irq->lock, flags);

	return IRQ_HANDLED;
}

int lwis_interrupt_set_event_info(struct lwis_interrupt_list *list, int index,
				  const char *irq_reg_space, int irq_reg_bid, int64_t *irq_events,
				  size_t irq_events_num, uint32_t *int_reg_bits,
				  size_t int_reg_bits_num, int64_t irq_src_reg,
				  int64_t irq_reset_reg, int64_t irq_mask_reg, bool mask_toggled,
				  int irq_reg_access_size, int64_t *critical_events,
				  size_t critical_events_num)
{
	int i, j;
	unsigned long flags;
	bool is_critical = false;

	if (int_reg_bits_num != irq_events_num) {
		pr_err("reg bits num != irq event num.\n");
		return -EINVAL;
	}

	/* Protect the structure */
	spin_lock_irqsave(&list->irq[index].lock, flags);
	/* Set the fields */
	list->irq[index].irq_reg_bid = irq_reg_bid;
	list->irq[index].irq_src_reg = irq_src_reg;
	list->irq[index].irq_reset_reg = irq_reset_reg;
	list->irq[index].irq_mask_reg = irq_mask_reg;
	list->irq[index].mask_toggled = mask_toggled;
	list->irq[index].irq_reg_access_size = irq_reg_access_size;
	/* Empty hash table for event infos */
	hash_init(list->irq[index].event_infos);
	/* Initialize an empty list for enabled events */
	INIT_LIST_HEAD(&list->irq[index].enabled_event_infos);
	spin_unlock_irqrestore(&list->irq[index].lock, flags);

	/* Build the hash table of events we can emit */
	for (i = 0; i < irq_events_num; i++) {
		struct lwis_single_event_info *new_event =
			kzalloc(sizeof(struct lwis_single_event_info), GFP_KERNEL);
		if (!new_event) {
			return -ENOMEM;
		}

		/* Check to see if this event is considered critical */
		is_critical = false;
		for (j = 0; j < critical_events_num; j++) {
			if (critical_events[j] == irq_events[i]) {
				is_critical = true;
				break;
			}
		}

		/* Fill the device id info in event id bit[47..32] */
		irq_events[i] |= (int64_t)(list->lwis_dev->id & 0xFFFF) << 32;
		/* Grab the device state outside of the spinlock */
		new_event->state =
			lwis_device_event_state_find_or_create(list->lwis_dev, irq_events[i]);
		new_event->event_id = irq_events[i];
		new_event->int_reg_bit = int_reg_bits[i];
		new_event->is_critical = is_critical;

		spin_lock_irqsave(&list->irq[index].lock, flags);
		/* Check for duplicate events */
		if (lwis_interrupt_get_single_event_info_locked(&list->irq[index],
								new_event->event_id) != NULL) {
			spin_unlock_irqrestore(&list->irq[index].lock, flags);
			dev_err(list->lwis_dev->dev, "Duplicate event_id: %llx for IRQ: %s\n",
				new_event->event_id, list->irq[index].name);
			kfree(new_event);
			return -EINVAL;
		}
		/* Let's add the new state object */
		hash_add(list->irq[index].event_infos, &new_event->node, new_event->event_id);

		spin_unlock_irqrestore(&list->irq[index].lock, flags);
	}
	/* It might make more sense to make has_events atomic_t instead of
	 * locking a spinlock to write a boolean, but then we might have to deal
	 * with barriers, etc. */
	spin_lock_irqsave(&list->irq[index].lock, flags);
	/* Set flag that we have events */
	list->irq[index].has_events = true;
	spin_unlock_irqrestore(&list->irq[index].lock, flags);

	return 0;
}

int lwis_interrupt_set_gpios_event_info(struct lwis_interrupt_list *list, int index,
					int64_t irq_event)
{
	unsigned long flags;
	struct lwis_single_event_info *new_event;

	/* Protect the structure */
	spin_lock_irqsave(&list->irq[index].lock, flags);
	/* Empty hash table for event infos */
	hash_init(list->irq[index].event_infos);
	/* Initialize an empty list for enabled events */
	INIT_LIST_HEAD(&list->irq[index].enabled_event_infos);
	spin_unlock_irqrestore(&list->irq[index].lock, flags);

	/* Build the hash table of events we can emit */

	new_event = kzalloc(sizeof(struct lwis_single_event_info), GFP_KERNEL);
	if (!new_event) {
		dev_err(list->lwis_dev->dev, "Allocate event info failed\n");
		return -ENOMEM;
	}

	/* Fill the device id info in event id bit[47..32] */
	irq_event |= (int64_t)(list->lwis_dev->id & 0xFFFF) << 32;
	/* Grab the device state outside of the spinlock */
	new_event->state = lwis_device_event_state_find_or_create(list->lwis_dev, irq_event);
	new_event->event_id = irq_event;

	spin_lock_irqsave(&list->irq[index].lock, flags);
	/* Check for duplicate events */
	if (lwis_interrupt_get_single_event_info_locked(&list->irq[index], new_event->event_id) !=
	    NULL) {
		spin_unlock_irqrestore(&list->irq[index].lock, flags);
		dev_err(list->lwis_dev->dev, "Duplicate event_id: %llx for IRQ: %s\n",
			new_event->event_id, list->irq[index].name);
		kfree(new_event);
		return -EINVAL;
	}
	/* Let's add the new state object */
	hash_add(list->irq[index].event_infos, &new_event->node, new_event->event_id);
	spin_unlock_irqrestore(&list->irq[index].lock, flags);

	/* It might make more sense to make has_events atomic_t instead of
	 * locking a spinlock to write a boolean, but then we might have to deal
	 * with barriers, etc. */
	spin_lock_irqsave(&list->irq[index].lock, flags);
	/* Set flag that we have events */
	list->irq[index].has_events = true;
	spin_unlock_irqrestore(&list->irq[index].lock, flags);

	return 0;
}

static int lwis_interrupt_single_event_enable_locked(struct lwis_interrupt *irq,
						     struct lwis_single_event_info *event,
						     bool enabled)
{
	int ret = 0;
	bool is_set;

	if (!irq) {
		pr_err("irq is NULL.\n");
		return -EINVAL;
	}

	if (!event) {
		pr_err("event is NULL.\n");
		return -EINVAL;
	}

	if (enabled) {
		list_add_tail(&event->node_enabled, &irq->enabled_event_infos);
	} else {
		list_del(&event->node_enabled);
	}

	/* If mask_toggled is set, reverse the enable/disable logic. */
	is_set = (!irq->mask_toggled) ? enabled : !enabled;
	ret = lwis_interrupt_set_mask(irq, event->int_reg_bit, is_set);

	return ret;
}

int lwis_interrupt_event_enable(struct lwis_interrupt_list *list, int64_t event_id, bool enabled)
{
	int index, ret = -EINVAL;
	unsigned long flags;
	struct lwis_single_event_info *event;

	if (!list) {
		pr_err("Interrupt list is NULL.\n");
		return -EINVAL;
	}

	for (index = 0; index < list->count; index++) {
		spin_lock_irqsave(&list->irq[index].lock, flags);
		event = lwis_interrupt_get_single_event_info_locked(&list->irq[index], event_id);
		if (event) {
			ret = lwis_interrupt_single_event_enable_locked(&list->irq[index], event,
									enabled);
		}
		spin_unlock_irqrestore(&list->irq[index].lock, flags);
	}
	return ret;
}

void lwis_interrupt_print(struct lwis_interrupt_list *list)
{
	int i;
	for (i = 0; i < list->count; ++i) {
		pr_info("%s: irq: %s\n", __func__, list->irq[i].name);
	}
}
