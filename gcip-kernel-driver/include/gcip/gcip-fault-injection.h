/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The helper functions for fault injection.
 *
 * Copyright (C) 2023 Google LLC
 */

#ifndef __GCIP_FAULT_INJECTION_H__
#define __GCIP_FAULT_INJECTION_H__

#include <linux/dcache.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include <gcip/gcip-pm.h>

#define DEBUGFS_FAULT_INJECTION "fault_injection"
#define GCIP_FAULT_INJECT_OPAQUE_SIZE 16
#define FAULT_INJECT_BUF_SIZE 256

/* Show immediate fault injection supporting status. */
enum fault_inject_status {
	/* Haven't known the fault injection status yet. */
	GCIP_FAULT_INJECT_STATUS_UNKNOWN,
	/* Encountered errors when sending a fault injection request. */
	GCIP_FAULT_INJECT_STATUS_ERROR,
	/* Fault injection is supported. */
	GCIP_FAULT_INJECT_STATUS_SUPPORTED,
	/* Fault injection is not supported by the firmware side. */
	GCIP_FAULT_INJECT_STATUS_UNSUPPORTED
};

/* Show fault injection progress. */
enum fault_inject_progress {
	/* Haven't set the fault injection yet. */
	GCIP_FAULT_INJECT_PROGRESS_NONE,
	/* Fault injection is set but haven't sent to the firmware. */
	GCIP_FAULT_INJECT_PROGRESS_PENDING,
	/* Fault injection is sent. */
	GCIP_FAULT_INJECT_PROGRESS_INJECTED
};

/**
 * struct gcip_fault_inject - The container of fault injection data.
 * @dev: The device pointer used to allocate local memory and print messages.
 * @d_entry: The DebugFS entry.
 * @pm: The power management object used to check mcu status.
 * @send_kci: The callback function used to send KCI.
 * @kci_data: The data that will be passed into send_kci.
 * @lock: Protects opaque and progress.
 * @opaque: It contains the fault injection data and will be read or write by runtime via debugfs.
 *          The callback function send_kci should send FAULT_INJECTION with this to the firmware.
 * @progress: This field records the fault injection progress in the KD side.
 * @fw_support_status: This field records whether the firmware supports the fault injection.
 */
struct gcip_fault_inject {
	struct device *dev;
	struct dentry *d_entry;
	struct gcip_pm *pm;
	int (*send_kci)(struct gcip_fault_inject *injection);
	void *kci_data;

	struct mutex lock;
	uint32_t opaque[GCIP_FAULT_INJECT_OPAQUE_SIZE];
	enum fault_inject_progress progress;

	enum fault_inject_status fw_support_status;
};

/**
 * struct gcip_fault_inject_args - The parameters for fault injection initialization.
 * @parent_dentry: The parent dentry where the "fault_injection" DebugFS node will be created.
 *
 * Except @parent_dentry, all the other fields are identical to the struct gcip_fault_inject.
 */
struct gcip_fault_inject_args {
	struct device *dev;
	struct dentry *parent_dentry;
	struct gcip_pm *pm;
	int (*send_kci)(struct gcip_fault_inject *injection);
	void *kci_data;
};

/**
 * gcip_fault_inject_create() - Creates a DebugFS node and allocates the fault injection object.
 * @args: The arguments needed by fault injection.
 *
 * A DebugFS node will be created for fault injecting. The node can be read or write for 64 bytes
 * data defined by runtime and firmware. On a successful write, the fault will be injected with
 * @send_kci immediately if @pm is powered, otherwise the injection will be pended. A read operation
 * will return the injection status and injection data.
 *
 * Return: If succeed, returns the pointer of the created fault injection object.
 *         Otherwise, returns the error pointer.
 */
struct gcip_fault_inject *gcip_fault_inject_create(const struct gcip_fault_inject_args *args);

/**
 * gcip_fault_inject_destroy() - Removes the DebugFS node and frees the fault injection object.
 * @injection: The container of fault injection data.
 */
void gcip_fault_inject_destroy(struct gcip_fault_inject *injection);

/**
 * gcip_fault_inject_send() - Sends the KCI command to the fw if there is a pending fault injection.
 * @injection: The container of fault injection data.
 *
 * Return:
 * * 0      - If there is no fault pending or the pending fault is injected successfully.
 * * Others - The error code returned by @injection->send_kci().
 */
int gcip_fault_inject_send(struct gcip_fault_inject *injection);

#endif /* __GCIP_FAULT_INJECTION_H__ */
