/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Interrupt Handler
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_INTERRUPT_H_
#define LWIS_INTERRUPT_H_

#include <linux/hashtable.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/platform_device.h>

#define EVENT_INFO_HASH_BITS 8
#define IRQ_FULL_NAME_LENGTH 32
#define LEAF_NODE_HASH_BITS 8

enum lwis_interrupt_types {
	REGULAR_INTERRUPT,
	AGGREGATE_INTERRUPT,
	LEAF_INTERRUPT,
	GPIO_HW_INTERRUPT,
	FAKEEVENT_INTERRUPT
};

/*
 *  struct lwis_interrupt_leaf_node
 *  This is to represent aggregate interrupt's leaf node
 */
struct lwis_interrupt_leaf_node {
	uint32_t int_reg_bit;
	int count;
	int32_t *leaf_irq_indexes;
	struct list_head node;
};

struct lwis_interrupt {
	int irq;
	/* IRQ name */
	char name[IRQ_FULL_NAME_LENGTH];
	/* Full name consists of both device and irq name */
	char full_name[IRQ_FULL_NAME_LENGTH];
	/* Device that owns this interrupt */
	struct lwis_device *lwis_dev;
	/* Spinlock to lock acccess to this struct */
	spinlock_t lock;
	/* Flag if the event info has been set */
	bool has_events;
	/* BID of the register space where the status/reset/mask for this ISR
	 * can be accessed
	 */
	int irq_reg_bid;
	/* Offset of the source register */
	int64_t irq_src_reg;
	/* Offset of the clear/reset register */
	int64_t irq_reset_reg;
	/* Offset of the mask register */
	int64_t irq_mask_reg;
	/* Offset of the overflow register */
	int64_t irq_overflow_reg;
	/* IRQ register access size, in case it is different from the bus
	 * bitwidth
	 */
	int irq_reg_access_size;
	/* If mask_reg actually disable the interrupts. */
	bool mask_toggled;
	/* Type of the interrupt */
	int32_t irq_type;
	/* Hash table of event info */
	/* GUARDED_BY(lock) */
	DECLARE_HASHTABLE(event_infos, EVENT_INFO_HASH_BITS);
	/* List of enabled events */
	/* GUARDED_BY(lock) */
	struct list_head enabled_event_infos;
	/* Select the interrupt line behavior */
	int irq_gpios_types;
	/* List of aggregate interrupt leaf nodes */
	/* GUARDED_BY(lock) */
	struct list_head leaf_nodes;
	/* Store the combined IRQ mask value */
	uint64_t mask_value;
	/* If IRQ mask value is updated */
	bool has_mask_update;
	/* Mask/Unmask the interrupt  */
	bool is_set_reg_bit;
};

/*
 *  struct lwis_interrupt_list
 *  This is to store the list of interrupts the LWIS device uses
 */
struct lwis_interrupt_list {
	struct lwis_interrupt *irq;
	int count;
	/* Device that owns this interrupt list */
	struct lwis_device *lwis_dev;
};

/*
 *  lwis_interrupt_list_alloc: Allocate an instance of the lwis_interrupt_list
 *  and initialize the data structures according to the number of interrupts
 *  specified.
 */
struct lwis_interrupt_list *lwis_interrupt_list_alloc(struct lwis_device *lwis_dev, int count);

/*
 *  lwis_interrupt_free_leaves: Deallocate the leaf_nodes in lwis_interrupt structure.
 */
void lwis_interrupt_free_leaves(struct lwis_interrupt *irq);

/*
 *  lwis_interrupt_list_free: Deallocate the lwis_interrupt_list structure.
 */
void lwis_interrupt_list_free(struct lwis_interrupt_list *list);

/*
 *  lwis_interrupt_init: Initialize the interrupt by index.
 */
int lwis_interrupt_init(struct lwis_interrupt_list *list, int index, char *name);

/*
 *  lwis_interrupt_get: Register the interrupt by index.
 *  Returns: 0 if success, -ve if error
 */
int lwis_interrupt_get(struct lwis_interrupt_list *list, int index,
		       struct platform_device *plat_dev);

/*
 *  lwis_interrupt_get_gpio_irq: Register the GPIO interrupt by index
 *  Returns: 0 if success, -ve if error
 */
int lwis_interrupt_get_gpio_irq(struct lwis_interrupt_list *list, int index, char *name,
				int gpio_irq, int32_t irq_gpios_types);

/*
 * lwis_interrupt_set_basic_info: Provides basic register info for a given
 * interrupt based on index
 */
void lwis_interrupt_set_basic_info(struct lwis_interrupt_list *list, int index,
				   const char *irq_reg_space, int irq_reg_bid, int64_t irq_src_reg,
				   int64_t irq_reset_reg, int64_t irq_mask_reg,
				   int64_t irq_overflow_reg, bool mask_toggled,
				   int irq_reg_access_size, int32_t irq_type);

/*
 * lwis_interrupt_set_event_info: Provides event-info structure for a given
 * interrupt based on index
 *
 * Takes ownership of (and later frees) irq_events and int_reg_bits
 * Does not free irq_reg_space
 * Returns: 0 on success
 */
int lwis_interrupt_set_event_info(struct lwis_interrupt_list *list, int index, int64_t *irq_events,
				  size_t irq_events_num, uint32_t *int_reg_bits,
				  size_t int_reg_bits_num, int64_t *critical_events,
				  size_t critical_events_num);

/*
 * lwis_interrupt_add_leaf: Provides one leaf node information for the aggregate interrupt
 *
 * Returns: 0 on success
 */
int lwis_interrupt_add_leaf(struct lwis_interrupt_list *list, int index, uint32_t int_reg_bit,
			    int count, int32_t *leaf_indexes);

/*
 * lwis_interrupt_set_gpios_event_info: Provides event-info structure for a given
 * interrupt based on index
 *
 * Returns: 0 on success
 */
int lwis_interrupt_set_gpios_event_info(struct lwis_interrupt_list *list, int index,
					int64_t irq_event);

/*
 * lwis_interrupt_write_combined_mask_value: Handles writing combined mask value
 *
 * Locks: May lock list->irq[index].lock
 * Alloc: No
 * Returns: 0 on success
 */
int lwis_interrupt_write_combined_mask_value(struct lwis_interrupt_list *list);

/*
 * lwis_interrupt_event_enable: Handles masking and unmasking interrupts when
 * an event is enabled or disabled
 *
 * Locks: May lock list->irq[index].lock
 * Alloc: No
 * Returns: 0 on success (event enabled/disabled)
 *          -EINVAL if event not known to this list
 */
int lwis_interrupt_event_enable(struct lwis_interrupt_list *list, int64_t event_id, bool enabled);

/*
 *  lwis_interrupt_print: Debug function to print all the interrupts in the
 *  supplied list.
 */
void lwis_interrupt_print(struct lwis_interrupt_list *list);

int lwis_fake_event_inject(void *data);

#endif /* LWIS_INTERRUPT_H_ */
