/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GCIP firmware interface.
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __GCIP_FIRMWARE_H__
#define __GCIP_FIRMWARE_H__

#include <linux/dcache.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/types.h>

/*
 * Any tracing level vote with the following bit set will be considered as a default vote.
 * See go/gcip-firmware-dynamic-tracing for details.
 */
#define GCIP_FW_TRACING_DEFAULT_VOTE BIT(8)

enum gcip_fw_status {
	/* No firmware loaded yet, or last firmware failed to run. */
	GCIP_FW_INVALID = 0,
	/* Load in progress. */
	GCIP_FW_LOADING = 1,
	/* Current firmware is valid and can be restarted. */
	GCIP_FW_VALID = 2,
};

/* Firmware flavors returned via KCI FIRMWARE_INFO command. */
enum gcip_fw_flavor {
	/* Unused value for extending enum storage type. */
	GCIP_FW_FLAVOR_ERROR = -1,
	/* Used by host when cannot determine the flavor. */
	GCIP_FW_FLAVOR_UNKNOWN = 0,
	/* Second-stage bootloader (no longer used). */
	GCIP_FW_FLAVOR_BL1 = 1,
	/* Systest app image. */
	GCIP_FW_FLAVOR_SYSTEST = 2,
	/* Default production app image. */
	GCIP_FW_FLAVOR_PROD_DEFAULT = 3,
	/* Custom image produced by other teams. */
	GCIP_FW_FLAVOR_CUSTOM = 4,
};

/* Type of firmware crash. */
enum gcip_fw_crash_type {
	/* Type which will be sent by GCIP_RKCI_FIRMWARE_CRASH reverse KCI. */
	/*Assert happened. */
	GCIP_FW_CRASH_ASSERT_FAIL = 0,
	/* Data abort exception. */
	GCIP_FW_CRASH_DATA_ABORT = 1,
	/* Prefetch abort exception. */
	GCIP_FW_CRASH_PREFETCH_ABORT = 2,
	/* Undefined exception. */
	GCIP_FW_CRASH_UNDEFINED_EXCEPTION = 3,
	/* Exception which cannot be recovered by the firmware itself. */
	GCIP_FW_CRASH_UNRECOVERABLE_FAULT = 4,
	/* Used in debug dump. */
	GCIP_FW_CRASH_DUMMY_CRASH_TYPE = 0xFF,

	/* HW watchdog timeout. */
	GCIP_FW_CRASH_HW_WDG_TIMEOUT = 0x100,
};

/* Firmware info filled out via KCI FIRMWARE_INFO command. */
struct gcip_fw_info {
	uint64_t fw_build_time; /* BuildData::Timestamp() */
	uint32_t fw_flavor; /* enum gcip_fw_flavor */
	uint32_t fw_changelist; /* BuildData::Changelist() */
	uint32_t spare[10];
};

/* Returns the name of @fw_flavor in string. */
char *gcip_fw_flavor_str(enum gcip_fw_flavor fw_flavor);

struct gcip_fw_tracing {
	struct device *dev;
	struct dentry *dentry;
	struct gcip_pm *pm;

	/*
	 * Lock to protect the struct members listed below.
	 *
	 * Note that since the request of tracing level adjusting might happen during power state
	 * transitions (i.e., another thread calling gcip_firmware_tracing_restore_on_powering()
	 * with pm lock held), one must either use the non-blocking gcip_pm_get_if_powered() or make
	 * sure there won't be any new power transition after holding this lock to prevent deadlock.
	 */
	struct mutex lock;
	/* Actual firmware tracing level. */
	unsigned long active_level;
	/* Requested firmware tracing level. */
	unsigned long request_level;

	/* Private data. See struct gcip_fw_tracing_args.*/
	void *data;

	/* Callbacks. See struct gcip_fw_tracing_args. */
	int (*set_level)(void *data, unsigned long level, unsigned long *active_level);
};

struct gcip_fw_tracing_args {
	/* Device struct of GCIP device. */
	struct device *dev;
	/* GCIP power management. */
	struct gcip_pm *pm;
	/* Top-level debugfs directory for the device. */
	struct dentry *dentry;
	/* Private data for callbacks listed below. */
	void *data;
	/*
	 * Callback to set the tracing level.
	 * The actual tracing level clamped by the firmware should be returned by @active_level.
	 */
	int (*set_level)(void *data, unsigned long level, unsigned long *active_level);
};

/* Allocate and initialize the firmware tracing struct. */
struct gcip_fw_tracing *gcip_firmware_tracing_create(const struct gcip_fw_tracing_args *args);

/* Destroy and free the firmware tracing struct. */
void gcip_firmware_tracing_destroy(struct gcip_fw_tracing *fw_tracing);

/*
 * Restore the previous firmware tracing level.
 *
 * This function is designed to restore the thermal state during firmware load and thus it assumes
 * the caller will kept the block powered.
 */
int gcip_firmware_tracing_restore_on_powering(struct gcip_fw_tracing *fw_tracing);

#endif /* __GCIP_FIRMWARE_H__ */
