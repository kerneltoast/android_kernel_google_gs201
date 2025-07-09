/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Devfreq support for GCIP devices.
 *
 * Copyright (C) 2024 Google LLC
 */

#ifndef __GCIP_DEVFREQ_H__
#define __GCIP_DEVFREQ_H__

#include <linux/devfreq.h>

/* Max number of supported frequencies. */
#define GCIP_DEVFREQ_MAX_DVFS_FREQ_NUM 25

/* Operators needed for devfreq initialization and implementation. */
struct gcip_devfreq_ops {
	/*
	 * Populate the `dvfs_devfreq_freqs_khz` table with distinct non zero available frequencies.
	 * There is no restriction that `dvfs_devfreq_freqs_khz` table should be updated in the
	 * sorted order of distinct frequencies. Returns the number of available distinct
	 * frequencies for the device. Returns zero in case available frequencies at IP driver
	 * exceeds the max_size supported by the GCIP devfreq interface.
	 *
	 * This callback is used during GCIP devfreq creation and is required for registering the
	 * available frequencies with the OPP library during upstream devfreq initialization. IP
	 * driver should neither provide zero frequency value nor repeated frequency value otherwise
	 * GCIP devfreq interface will abort the devfreq creation.
	 */
	u32 (*get_freq_table)(void *data, u32 *dvfs_table, u32 max_size);

	/*
	 * Callback to pass the updated min and max frequency limit to the underlying IP driver.
	 * GCIP devfreq interface being aware of valid discrete supported frequencies will always
	 * ensure to pass the valid frequency range to the IP driver.
	 */
	void (*update_min_max_freq_range)(void *data, u32 min_freq_khz, u32 max_freq_khz);

	/*
	 * Callback to get the current frequency of the device. This callback is used directly by
	 * the upstream devfreq framework that expects the frequency to be in Hz. Returns zero on
	 * success or < 0 on error.
	 */
	int (*get_cur_freq)(struct device *dev, unsigned long *freq_hz);
};

struct gcip_devfreq {
	/* Device struct of GCIP device. */
	struct device *dev;

	/* Pointer to upstream devfreq instance. */
	struct devfreq *devfreq;

	/*
	 * Pointer to upstream devfreq_dev_profile struct needed for creating the devfreq
	 * instance.
	 */
	struct devfreq_dev_profile *profile;

	/* Notifier block for DEV_PM_QOS_MIN_FREQUENCY. */
	struct notifier_block min_freq_nb;

	/* Notifier block for DEV_PM_QOS_MAX_FREQUENCY. */
	struct notifier_block max_freq_nb;

	/* Array to store the permissible frequencies(in khz) for the device. */
	u32 dvfs_devfreq_freqs_khz[GCIP_DEVFREQ_MAX_DVFS_FREQ_NUM];

	/* The number of DVFS frequencies. */
	u32 dvfs_devfreq_freqs_num;

	/* Private data used for operators below. */
	void *data;

	/*
	 * Lock to protect the min and max freq range calculation in the notifier callbacks for
	 * DEV_PM_QOS{MIN.MAX}_FREQUENCY. When user writes on {min,max}_freq sysfs node exposed by
	 * devfreq framework, it updates the DEV_PM_QOS{MIN.MAX}_FREQUENCY resulting in execution
	 * of respective registered notifier callabcks. For avoiding race condition when both the
	 * {min,max}_freq nodes are updated around same time, lock will ensure the atomic min/max
	 * frequency range calculation in the notifier callbacks.
	 */
	struct mutex min_max_range_lock;

	/* Operators. */
	const struct gcip_devfreq_ops *ops;
};

struct gcip_devfreq_args {
	/* Device struct of GCIP device. */
	struct device *dev;

	/* Private data for operators below. */
	void *data;

	/* Operators. */
	const struct gcip_devfreq_ops *ops;
};

/**
 * gcip_devfreq_create() - Allocate and initialize the GCIP devfreq struct.
 * @args: struct gcip_devfreq_args containing the information for registering the devfreq framework.
 *
 * Return:
 * * Pointer to struct gcip_devfreq on success.
 * * ERR_PTR(-EINVAL) - Invalid values passed via the args argument.
 * * ERR_PTR(-ENOMEM) - Not enough memory left.
 */
struct gcip_devfreq *gcip_devfreq_create(const struct gcip_devfreq_args *args);

/**
 * gcip_devfreq_destroy() - Destroy and free the GCIP devfreq struct.
 * @devfreq: Pointer to gcip devfreq interface.
 */
void gcip_devfreq_destroy(struct gcip_devfreq *devfreq);

#endif /* __GCIP_DEVFREQ_H__ */
