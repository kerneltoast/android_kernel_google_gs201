/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS SPI Interface
 *
 * Copyright (c) 2023 Google, LLC
 */

#ifndef LWIS_SPI_H_
#define LWIS_SPI_H_

#include "lwis_commands.h"
#include "lwis_device_spi.h"

/*
 *  lwis_spi_io_entry_rw: Read/Write from spi bus via io_entry request.
 *  The readback values will be stored in the entry.
 */
int lwis_spi_io_entry_rw(struct lwis_spi_device *spi_dev, struct lwis_io_entry *entry);

#endif /* LWIS_SPI_H_ */
