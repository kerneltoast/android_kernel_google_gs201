/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS I/O Entry Implementation
 *
 * Copyright (c) 2021 Google, LLC
 */

#ifndef LWIS_IO_ENTRY_H_
#define LWIS_IO_ENTRY_H_

#include "lwis_commands.h"
#include "lwis_device.h"

/* Maximum value of sleep time in us */
#define MAX_WAIT_TIME 1000000
/* Default value of polling timeout */
#define DEFAULT_POLLING_TIMEOUT_MS 5

/*
 * lwis_io_entry_poll:
 * Polls a register for a specified time or until it reaches the expected value.
 */
int lwis_io_entry_poll(struct lwis_device *lwis_dev, struct lwis_io_entry *entry, bool is_short);

/*
 * lwis_io_entry_read_assert:
 * Returns error if a register's value is not as expected.
 */
int lwis_io_entry_read_assert(struct lwis_device *lwis_dev, struct lwis_io_entry *entry);

/*
 * lwis_io_entry_wait:
 * Waits for a settling time to meet the devices to function properly.
 */
int lwis_io_entry_wait(struct lwis_device *lwis_dev, struct lwis_io_entry *entry);

#endif /* LWIS_IO_ENTRY_H_ */
