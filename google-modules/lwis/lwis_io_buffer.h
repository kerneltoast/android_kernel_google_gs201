/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * LWIS Buffer I/O Implementation
 *
 * Copyright (c) 2024 Google LLC.
 */

#ifndef LWIS_IO_BUFFER_H_
#define LWIS_IO_BUFFER_H_

#include "lwis_commands.h"
#include "lwis_device.h"

/*
 * Map the PDMA buffer
 */
int lwis_io_buffer_map(struct lwis_device *lwis_dev, struct lwis_io_entry *entry);

/*
 * Map the PDMA buffer
 */
void lwis_io_buffer_unmap(struct lwis_io_entry *entry);

/*
 * Write the byte to the buffer.
 */
int lwis_io_buffer_write(struct lwis_device *lwis_dev, struct lwis_io_entry *entry);

#endif /* LWIS_IO_BUFFER_H_ */
