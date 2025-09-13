// SPDX-License-Identifier: GPL-2.0-only
/*
 * System Level Cache support for GCIP devices.
 *
 * Copyright (C) 2023 Google LLC
 */

#include <linux/dcache.h>
#include <linux/debugfs.h>
#include <linux/device.h>

#include <gcip/gcip-slc.h>

static int gcip_debugfs_slc_pid_set(void *data, u64 val)
{
	struct gcip_slc *slc = data;
	int ret = 0;

	/* User can set a dedicated invalid pid to disable the SLC */
	if ((val >= GCIP_SLC_MIN_PID && val <= GCIP_SLC_MAX_PID) || (val == GCIP_SLC_INVALID_PID)) {
		slc->pid = val;
	} else {
		ret = -EINVAL;
		dev_err(slc->dev, "Setting out of range SLC pid: %llu\n", val);
	}

	return ret;
}

static int gcip_debugfs_slc_pid_get(void *data, u64 *val)
{
	struct gcip_slc *slc = data;

	*val = slc->pid;

	return 0;
}

static int gcip_debugfs_slc_cache_set(void *data, u64 val)
{
	struct gcip_slc *slc = data;

	slc->cache = val;

	return 0;
}

static int gcip_debugfs_slc_cache_get(void *data, u64 *val)
{
	struct gcip_slc *slc = data;

	*val = slc->cache;

	return 0;
}

static int gcip_debugfs_slc_r_alloc_override_set(void *data, u64 val)
{
	struct gcip_slc *slc = data;

	slc->r_alloc_override = val;

	return 0;
}

static int gcip_debugfs_slc_r_alloc_override_get(void *data, u64 *val)
{
	struct gcip_slc *slc = data;

	*val = slc->r_alloc_override;

	return 0;
}

static int gcip_debugfs_slc_w_alloc_override_set(void *data, u64 val)
{
	struct gcip_slc *slc = data;

	slc->w_alloc_override = val;

	return 0;
}

static int gcip_debugfs_slc_w_alloc_override_get(void *data, u64 *val)
{
	struct gcip_slc *slc = data;

	*val = slc->w_alloc_override;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gcip_debugfs_slc_pid_fops, gcip_debugfs_slc_pid_get,
			 gcip_debugfs_slc_pid_set, "%lld\n");

DEFINE_DEBUGFS_ATTRIBUTE(gcip_debugfs_slc_cache_fops, gcip_debugfs_slc_cache_get,
			 gcip_debugfs_slc_cache_set, "0x%llx\n");

DEFINE_DEBUGFS_ATTRIBUTE(gcip_debugfs_slc_r_alloc_override_fops,
			 gcip_debugfs_slc_r_alloc_override_get,
			 gcip_debugfs_slc_r_alloc_override_set, "%lld\n");

DEFINE_DEBUGFS_ATTRIBUTE(gcip_debugfs_slc_w_alloc_override_fops,
			 gcip_debugfs_slc_w_alloc_override_get,
			 gcip_debugfs_slc_w_alloc_override_set, "%lld\n");

void gcip_slc_debugfs_init(struct gcip_slc *slc, struct device *dev, struct dentry *d_entry)
{
	slc->dev = dev;
	slc->d_entry = debugfs_create_dir(GCIP_SLC_NAME, d_entry);
	slc->pid = GCIP_SLC_INVALID_PID;
	if (IS_ERR(slc->d_entry)) {
		slc->d_entry = NULL;
	} else {
		debugfs_create_file(GCIP_DEBUGFS_SLC_PID, 0600, slc->d_entry, slc,
				    &gcip_debugfs_slc_pid_fops);
		debugfs_create_file(GCIP_DEBUGFS_SLC_CACHE, 0600, slc->d_entry, slc,
				    &gcip_debugfs_slc_cache_fops);
		debugfs_create_file(GCIP_DEBUGFS_SLC_R_ALLOC_OVERRIDE, 0600, slc->d_entry, slc,
				    &gcip_debugfs_slc_r_alloc_override_fops);
		debugfs_create_file(GCIP_DEBUGFS_SLC_W_ALLOC_OVERRIDE, 0600, slc->d_entry, slc,
				    &gcip_debugfs_slc_w_alloc_override_fops);
	}
}

void gcip_slc_debugfs_exit(struct gcip_slc *slc)
{
	if (!slc->d_entry)
		return;

	debugfs_remove_recursive(slc->d_entry);
}
