/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS I/O Mapped Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_DEVICE_IOREG_H_
#define LWIS_DEVICE_IOREG_H_

#include <linux/types.h>

#include "lwis_bus_manager.h"
#include "lwis_device.h"

struct lwis_ioreg {
	phys_addr_t start;
	int size;
	void __iomem *base;
	char *name;
};

struct lwis_ioreg_list {
	struct lwis_ioreg *block;
	int count;
};

/*
 *  struct lwis_ioreg_device
 *  "Derived" lwis_device struct, with added IOREG related elements.
 */
struct lwis_ioreg_device {
	struct lwis_device base_dev;
	struct lwis_ioreg_list reg_list;
	struct lwis_bus_manager *ioreg_bus_manager;
	int device_priority;
	/* Group handle for devices that are managed together */
	u32 device_group;
};

int lwis_ioreg_device_init(void);
int lwis_ioreg_device_deinit(void);
#endif /* LWIS_DEVICE_IOREG_H_ */
