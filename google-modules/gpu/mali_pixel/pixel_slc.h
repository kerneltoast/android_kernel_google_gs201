// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2024 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */
#ifndef _PIXEL_SLC_H_
#define _PIXEL_SLC_H_

#include <soc/google/pt.h>

/**
 * DOC: SLC partition management
 *
 * Key definitions:
 * + Partition index - The unique index of a partition, relative to the dt node that owns it.
 *                     This index is used when communicating with the underlying SLC driver.
 * + ptid - This is the HW level ID associated with an enabled partition. These id's are allocated
 *          at partition enable time. The GPU driver will never directly use the ptid, but will
 *          track it.
 *          External analysis of the caching behavior (e.g. hit and eviction counters), are
 *          associated with a ptid, not a physical partition index.
 *          This driver attempts to hold on to any allocated ptids until driver termination to make
 *          profiling of caching performance easier.
 * + PBHA - Acronym: Page Based Hardware Attributes. Every physical partition has a PBHA value
 *          associated with it. We insert these attributes into PTEs so that transactions with a
 *          page carry the PBHA within their high bits.
 *          Transactions with PBHA bits set are intercepted by the SLC, where the corresponding
 *          partition and it's caching behavior (Read/write alloc etc.) are looked up and applied to
 *          the transaction.
 */

/**
 * struct slc_partition - Structure for tracking partition state.
 */
struct slc_partition {
	/** @index: The active partition ID for this virtual partition */
	u32 index;

	/** @ptid: The active partition ID for this virtual partition */
	ptid_t ptid;

	/** @pbha: The page based HW attributes for this partition */
	ptpbha_t pbha;

	/** @enabled: Is the partition currently enabled */
	bool enabled;

	/** @refcount: Reference count for this partition */
	atomic_t refcount;

	/** @lock: Lock protecting enable/disable ops on this partition */
	spinlock_t lock;

	/** @signal: Partition enable/disable signal from SLC governor */
	u64 signal;

	/** @pinned: Is the partition pinned to the enabled state */
	bool pinned;
};

/**
 * struct slc_data - Structure for tracking SLC context.
 */
struct slc_data {
	/** @pt_handle: Link to ACPM SLC partition data */
	struct pt_handle *pt_handle;

	/** @partition: Information specific to an individual SLC partition */
	struct slc_partition partition;

	/** @dev: Inherited pointer to device attached */
	struct device *dev;

	/** @disable_work: Work item used to queue lazy SLC partition disable ops. */
	struct delayed_work disable_work;

	/** @signal: Partition enable/disable signal from SLC governor. */
	char __iomem *signal;
};

int slc_init_data(struct slc_data *data, struct device* dev);

void slc_term_data(struct slc_data *data);

u64 slc_set_pbha(struct slc_data const *data, u64 pte);

u64 slc_wipe_pbha(u64 pte);

void slc_inc_refcount(struct slc_data *data);

void slc_dec_refcount(struct slc_data *data);

void slc_pin(struct slc_data *data, bool pin);

void slc_update_signal(struct slc_data *data, u64 signal);

#endif /* _PIXEL_SLC_H_ */
