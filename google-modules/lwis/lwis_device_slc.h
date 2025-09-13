/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS SLC Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 */
#ifndef LWIS_DEVICE_SLC_H_
#define LWIS_DEVICE_SLC_H_

#include "lwis_device.h"
#include <soc/google/pt.h>

#define MAX_NUM_PT 16

struct slc_partition {
	int id;
	size_t size_kb;
	int fd;
	ptid_t partition_id;
	struct pt_handle *partition_handle;
};

/*
 *  struct lwis_slc_device
 *  "Derived" lwis_device struct, with added slc related elements.
 */
struct lwis_slc_device {
	struct lwis_device base_dev;
	int num_pt;
	struct slc_partition pt[MAX_NUM_PT];
	struct pt_handle *partition_handle;
};

int lwis_slc_device_init(void);
int lwis_slc_device_deinit(void);

int lwis_slc_buffer_alloc(struct lwis_device *lwis_dev, struct lwis_alloc_buffer_info *alloc_info);

int lwis_slc_buffer_free(struct lwis_device *lwis_dev, int fd);

#endif /* LWIS_DEVICE_SLC_H_ */
