/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Register I/O Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_IOREG_H_
#define LWIS_IOREG_H_

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include "lwis_commands.h"
#include "lwis_device_ioreg.h"

/*
 *  lwis_ioreg_list_alloc: Allocate a lwis_ioreg_list struct with the specified
 *  number of entries.
 */
int lwis_ioreg_list_alloc(struct lwis_ioreg_device *ioreg_dev, int num_blocks);

/*
 *  lwis_ioreg_list_free: Deallocate the lwis_ioreg list struct provided.
 */
void lwis_ioreg_list_free(struct lwis_ioreg_device *ioreg_dev);

/*
 *  lwis_ioreg_get: Setup the content of a lwis_ioreg entry.
 */
int lwis_ioreg_get(struct lwis_ioreg_device *ioreg_dev, int index, char *name);

/*
 *  lwis_ioreg_put_by_idx: Deinitialize the content of a lwis_ioreg entry
 *  by index.
 */
int lwis_ioreg_put_by_idx(struct lwis_ioreg_device *ioreg_dev, int index);

/*
 *  lwis_ioreg_put_by_name: Deinitialize the content of a lwis_ioreg entry
 *  by name.
 */
int lwis_ioreg_put_by_name(struct lwis_ioreg_device *ioreg_dev, char *name);

/*
 *  lwis_ioreg_io_entry_rw: Read/write registers via io_entry request.
 */
int lwis_ioreg_io_entry_rw(struct lwis_ioreg_device *ioreg_dev, struct lwis_io_entry *entry,
			   int access_size);

/*
 *  lwis_ioreg_read: Read single register.
 */
int lwis_ioreg_read(struct lwis_ioreg_device *ioreg_dev, int index, uint64_t offset,
		    uint64_t *value, int access_size);

/*
 *  lwis_ioreg_write: Write single register.
 */
int lwis_ioreg_write(struct lwis_ioreg_device *ioreg_dev, int index, uint64_t offset,
		     uint64_t value, int access_size);

/*
 * lwis_ioreg_set_io_barrier: Use read/write memory barriers.
 */
int lwis_ioreg_set_io_barrier(struct lwis_ioreg_device *ioreg_dev, bool use_read_barrier,
			      bool use_write_barrier);

#endif /* LWIS_IOREG_H_ */
