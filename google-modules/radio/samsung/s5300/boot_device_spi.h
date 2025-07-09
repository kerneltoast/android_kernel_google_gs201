/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019, Samsung Electronics.
 *
 */

#ifndef __BOOT_DEVICE_SPI_H__
#define __BOOT_DEVICE_SPI_H__

#include <linux/spi/spi.h>

struct cpboot_spi {
	struct spi_device *spi;
	struct mutex lock;
};

#if IS_ENABLED(CONFIG_BOOT_DEVICE_SPI)
extern struct cpboot_spi *cpboot_spi_get_device(int bus_num);
extern int cpboot_spi_load_cp_image(struct link_device *ld, struct io_device *iod,
		unsigned long arg);
#else
static inline struct cpboot_spi *cpboot_spi_get_device(int bus_num) { return NULL; }
static inline int cpboot_spi_load_cp_image(struct link_device *ld, struct io_device *iod,
		unsigned long arg) { return 0; }
#endif

#endif /* __BOOT_DEVICE_SPI_H__ */
