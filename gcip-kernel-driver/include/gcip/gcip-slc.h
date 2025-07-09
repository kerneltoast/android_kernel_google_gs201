/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * System Level Cache support for GCIP devices.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __GCIP_SLC_H__
#define __GCIP_SLC_H__

#include <linux/dcache.h>
#include <linux/device.h>

#define GCIP_SLC_INVALID_PID 0
#define GCIP_SLC_MIN_PID 1
#define GCIP_SLC_MAX_PID 64

#define GCIP_SLC_NAME "slc"
#define GCIP_DEBUGFS_SLC_PID "pid"
#define GCIP_DEBUGFS_SLC_CACHE "cache"
#define GCIP_DEBUGFS_SLC_R_ALLOC_OVERRIDE "r_alloc_override"
#define GCIP_DEBUGFS_SLC_W_ALLOC_OVERRIDE "w_alloc_override"

struct gcip_slc {
	/* Device struct of GCIP device. */
	struct device *dev;
	/* debugfs dir for the slc. */
	struct dentry *d_entry;
	/* SLC partition ID. */
	uint pid;
	/* SLC cache setting. User can configure the value to modify the SLC AXI AxCACHE value. */
	uint cache;
	/*
	 * SLC read allocate override. User can configure the value to override the Read Allocate
	 * value.
	 */
	uint r_alloc_override;
	/*
	 * SLC write allocate override. User can configure the value to override the Write
	 * Allocate value.
	 */
	uint w_alloc_override;
};

/* Initializes the SLC debugfs attributes. */
void gcip_slc_debugfs_init(struct gcip_slc *slc, struct device *dev, struct dentry *d_entry);

/* Cleans up the SLC debugfs attributes. */
void gcip_slc_debugfs_exit(struct gcip_slc *slc);

/*
 * Checks if the SLC attribute is valid by its partition ID.
 * Only configure the registers if the partition ID is valid.
 */
static inline bool gcip_slc_is_valid(struct gcip_slc *slc)
{
	return (slc->pid != GCIP_SLC_INVALID_PID);
}

#endif /* __GCIP_SLC_H__ */
