/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Common platform interfaces for mobile TPU chips.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#ifndef __EDGETPU_MOBILE_PLATFORM_H__
#define __EDGETPU_MOBILE_PLATFORM_H__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <soc/google/exynos_pm_qos.h>

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
#include <bcl.h>
#endif

#include "edgetpu-config.h"
#include "edgetpu-internal.h"

#define to_mobile_dev(etdev) container_of(etdev, struct edgetpu_mobile_platform_dev, edgetpu_dev)

struct edgetpu_mobile_platform_pwr {
	struct dentry *debugfs_dir;
	struct mutex policy_lock;
	enum edgetpu_pwr_state curr_policy;
	struct mutex state_lock;
	u64 min_state;
	u64 requested_state;
	/* INT/MIF requests for memory bandwidth */
	struct exynos_pm_qos_request int_min;
	struct exynos_pm_qos_request mif_min;
	/* BTS */
	unsigned int performance_scenario;
	int scenario_count;
	struct mutex scenario_lock;

	/* LPM callbacks, NULL for chips without LPM */
	int (*lpm_up)(struct edgetpu_dev *etdev);
	void (*lpm_down)(struct edgetpu_dev *etdev);

	/* Block shutdown status callback, may be NULL */
	bool (*is_block_down)(struct edgetpu_dev *etdev);

	/* Firmware shutdown callback. Must be implemented */
	void (*firmware_down)(struct edgetpu_dev *etdev);

	/* Chip-specific setup after the PM interface is created */
	int (*after_create)(struct edgetpu_dev *etdev);

	/* Chip-specific cleanup before the PM interface is destroyed */
	int (*before_destroy)(struct edgetpu_dev *etdev);

	/* ACPM set rate callback. Must be implemented */
	int (*acpm_set_rate)(unsigned int id, unsigned long rate);
};

struct edgetpu_mobile_platform_dev {
	/* Generic edgetpu device */
	struct edgetpu_dev edgetpu_dev;
	/* Common mobile platform power interface */
	struct edgetpu_mobile_platform_pwr platform_pwr;
	/* Physical address of the firmware image */
	phys_addr_t fw_region_paddr;
	/* Size of the firmware region */
	size_t fw_region_size;
	/* Virtual address of the memory region shared with firmware */
	void *shared_mem_vaddr;
	/* Physical address of the memory region shared with firmware */
	phys_addr_t shared_mem_paddr;
	/* Size of the shared memory region size */
	size_t shared_mem_size;
	/* Physical address of the firmware context region */
	phys_addr_t fw_ctx_paddr;
	/* Size of the firmware context region */
	size_t fw_ctx_size;
	/*
	 * Pointer to GSA device for firmware authentication.
	 * May be NULL if the chip does not support firmware authentication
	 */
	struct device *gsa_dev;
	/* Virtual address of the SSMT block for this chip. */
	void __iomem *ssmt_base;
#if IS_ENABLED(CONFIG_EDGETPU_TELEMETRY_TRACE)
	/* Coherent log buffer */
	struct edgetpu_coherent_mem *log_mem;
	/* Coherent trace buffer */
	struct edgetpu_coherent_mem *trace_mem;
#endif
#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	struct bcl_device *bcl_dev;
#endif
	/* Protects TZ Mailbox client pointer */
	struct mutex tz_mailbox_lock;
	/* TZ mailbox client */
	struct edgetpu_client *secure_client;

	/* Length of @irq */
	int n_irq;
	/* Array of IRQ numbers */
	int *irq;
	/* PMU status base address for block status, maybe NULL */
	void __iomem *pmu_status;

	/* callbacks for chip-dependent implementations */

	/*
	 * Called when common device probing procedure is done.
	 *
	 * Return a non-zero value can fail the probe procedure.
	 *
	 * This callback is optional.
	 */
	int (*after_probe)(struct edgetpu_mobile_platform_dev *etmdev);
	/*
	 * Called before common device removal procedure.
	 *
	 * This callback is optional.
	 */
	void (*before_remove)(struct edgetpu_mobile_platform_dev *etmdev);
};

#endif /* __EDGETPU_MOBILE_PLATFORM_H__ */
