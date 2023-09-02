/*
 * Google LWIS I/O Mapped Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_DEVICE_IOREG_H_
#define LWIS_DEVICE_IOREG_H_

#include <linux/types.h>

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
};

int lwis_ioreg_device_deinit(void);
#endif /* LWIS_DEVICE_IOREG_H_ */
