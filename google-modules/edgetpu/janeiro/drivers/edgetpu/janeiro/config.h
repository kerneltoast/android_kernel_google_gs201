/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Include all configuration files for Janeiro.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#ifndef __JANEIRO_CONFIG_H__
#define __JANEIRO_CONFIG_H__

#define DRIVER_NAME "janeiro"

#define EDGETPU_DEV_MAX		1

/*
 * Can be coherent with AP
 *
 * Linux IOMMU-DMA APIs optimise cache operations based on "dma-coherent"
 * property in DT. Handle "dma-coherent" property in driver itself instead of
 * specifying in DT so as to support both coherent and non-coherent buffers on
 * VIIs.
 */
#define EDGETPU_IS_DMA_COHERENT

#define EDGETPU_HAS_MULTI_GROUPS
/* 1 context per VII/group plus 1 for KCI */
#define EDGETPU_NCONTEXTS 8
/* Max number of virtual context IDs that can be allocated for one device. */
#define EDGETPU_NUM_VCIDS 16
/* Reserved VCID that uses the extra partition. */
#define EDGETPU_VCID_EXTRA_PARTITION 0

/* Pre-allocate 1 IOMMU domain per VCID */
#define EDGETPU_NUM_PREALLOCATED_DOMAINS EDGETPU_NUM_VCIDS

/* Is a "mobile" style device. */
#define EDGETPU_FEATURE_MOBILE
#define EDGETPU_HAS_WAKELOCK

/* Is able to support external workloads */
#define EDGETPU_FEATURE_INTEROP

/* Metrics are reported for a single default "cluster" component. */
#define EDGETPU_TPU_CLUSTER_COUNT	1

/*
 * The TPU VA where the firmware is located.
 *
 * The address is chosen to not overlap with any memory region specified in the
 * firmware's linker file.
 */
#define FW_IOVA 0x16000000u
/*
 * Size of the area in remapped DRAM reserved for firmware code and internal
 * data. This must match the firmware's linker file.
 */
#define EDGETPU_FW_SIZE_MAX			0x100000

/* Data in remapped DRAM starts after firmware code and internal data */
#define EDGETPU_REMAPPED_DATA_OFFSET		EDGETPU_FW_SIZE_MAX

/*
 * Size of remapped DRAM data region. This must match the firmware's linker
 * file
 */
#define EDGETPU_REMAPPED_DATA_SIZE		0x100000

/*
 * Instruction remap registers make carveout memory appear at address
 * 0x10000000 from the TPU CPU perspective
 */
#define EDGETPU_INSTRUCTION_REMAP_BASE		0x10000000

/* Address from which the TPU CPU can access data in the remapped region */
#define EDGETPU_REMAPPED_DATA_ADDR                                        \
	(EDGETPU_INSTRUCTION_REMAP_BASE + EDGETPU_REMAPPED_DATA_OFFSET)

#include "config-mailbox.h"
#include "config-pwr-state.h"
#include "config-tpu-cpu.h"
#include "csrs.h"

#endif /* __JANEIRO_CONFIG_H__ */
