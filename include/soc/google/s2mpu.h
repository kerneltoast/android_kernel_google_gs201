/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Google LLC.
 */
#ifndef __S2MPU_H
#define __S2MPU_H

#include <linux/dma-map-ops.h>

struct s2mpu_info;

struct s2mpu_info *s2mpu_fwnode_to_info(struct fwnode_handle *fwnode);
int s2mpu_open(struct s2mpu_info *info, phys_addr_t start_pa, size_t len,
	       enum dma_data_direction dir);
int s2mpu_close(struct s2mpu_info *info, phys_addr_t start_pa, size_t len,
		enum dma_data_direction dir);
int s2mpu_restore(struct s2mpu_info *info);

#endif
