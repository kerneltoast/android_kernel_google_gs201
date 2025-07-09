/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Misc Utility Functions and Wrappers
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_UTIL_H_
#define LWIS_UTIL_H_

#include <linux/kernel.h>
#include <linux/ktime.h>

#include "lwis_commands.h"

/* Forward declaration for lwis_device. This is needed for the function
 * prototypes below that take a pointer to lwis_device
 */
struct lwis_device;

/*
 * lwis_device_single_register_write: A utility function that allows you to
 * write a single register for a given bid, offset and value on any device
 * that supports register writes.
 *
 * Returns: 0 on success
 * -EAGAIN if non_blocking is true and the operation would need to block
 * -ENXIO if register offset is out of range allowed for bid
 * Other errors are possible
 */
int lwis_device_single_register_write(struct lwis_device *lwis_dev, int bid, uint64_t offset,
				      uint64_t value, int access_size);

/*
 * lwis_device_single_register_read: A utility function that allows you to
 * read a single register for a given bid, offset and value on any device
 * that supports register reads.
 *
 * Returns: 0 on success
 * -EAGAIN if non_blocking is true and the operation would need to block
 * -ENXIO if register offset is out of range allowed for bid
 * Other errors are possible
 */
int lwis_device_single_register_read(struct lwis_device *lwis_dev, int bid, uint64_t offset,
				     uint64_t *value, int access_size);

/*
 * lwis_device_type_to_string: Converts the LWIS device type into a human-
 * readable string. Useful for debug logging.
 */
const char *lwis_device_type_to_string(int32_t type);

/*
 * trigger_condition_node_operator_to_string: Converts the trigger condition
 * node type into a human-readable string. Useful for debug logging.
 */
const char *trigger_condition_node_operator_to_string(int32_t type);

/*
 * lwis_get_time: Returns time since boot, this uses CLOCK_BOOTTIME which
 * does not stop during system suspend.
 *
 * This wrapper is created to encourage consistent usage of clock source
 * throughout LWIS implementations.
 */
static inline ktime_t lwis_get_time(void)
{
	return ktime_get_boottime();
}

/*
 * lwis_create_kthread_workers: Creates kthread workers associated with this lwis device.
 */
int lwis_create_kthread_workers(struct lwis_device *lwis_dev);

/*
 * lwis_set_kthread_priority: Set kthread priority.
 */
int lwis_set_kthread_priority(struct lwis_device *lwis_dev, struct task_struct *task, u32 priority);

/*
 * lwis_check_device_type: Returns true if the passed lwis_device's type is same as 'type'
 */
bool lwis_check_device_type(struct lwis_device *lwis_dev, int32_t type);

/*
 * lwis_value_to_be_buf: transform value to big endian buffer
 */
void lwis_value_to_be_buf(uint64_t value, uint8_t *buf, int buf_size);

/*
 * lwis_be_buf_to_value: transform big endian buffer to value
 */
uint64_t lwis_be_buf_to_value(uint8_t *buf, int buf_size);

#endif // LWIS_UTIL_H_
