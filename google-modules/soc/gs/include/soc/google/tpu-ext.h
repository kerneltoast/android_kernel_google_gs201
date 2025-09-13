/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Utility functions for interfacing other modules with Edge TPU ML
 * accelerator.
 *
 * Copyright (C) 2021-2022 Google LLC
 */

#ifndef __TPU_EXT_H__
#define __TPU_EXT_H__

#include <linux/device.h>
#include <linux/file.h>
#include <linux/types.h>

/* Flags for struct edgetpu_ext_client_info. */
#define EDGETPU_EXT_SECURE_CLIENT BIT(0)

/*
 * Data structures to be used for ALLOCATE_EXTERNAL_MAILBOX command
 */
struct edgetpu_ext_mailbox_descriptor {
	phys_addr_t cmdq_pa, respq_pa;
};

struct edgetpu_ext_client_info {
	u32 tpu_fd; /* fd of the opened TPU device */
	u32 mbox_map; /* bitmap of requested mailboxes */
	struct edgetpu_mailbox_attr __user *attr;
	/*
	 * File pointer of opened TPU device. This will be used instead of @tpu_fd when @tpu_fd is
	 * -1. To prevent fd swapping attack, it is encouraged to use this.
	 */
	struct file *tpu_file;
	u32 flags;
	u8 reserved[36]; /* Reserved for future compatibility. */
} __packed;

/*
 * Structure to hold information about mailboxes.
 * Length of @mailboxes should be at least number of set bits in @mbox_map
 */
struct edgetpu_ext_mailbox_info {
	size_t cmdq_size, respq_size;
	/*
	 * array length is equal to number of set bits in
	 * edgetpu_ext_client_info.mbox_map
	 */
	struct edgetpu_ext_mailbox_descriptor mailboxes[];
};

/*
 * Structure to hold information of TPU offload.
 */
struct edgetpu_ext_offload_info {
	u32 client_id; /* ID of the virtual client. */
	u8 reserved[60]; /* Reserved for future compatibility. */
};

enum edgetpu_ext_commands {
	/* in_data: edgetpu_ext_client_info, out_data: edgetpu_ext_mailbox_info */
	ALLOCATE_EXTERNAL_MAILBOX,
	/* in_data: edgetpu_ext_client_info, out_data: unused */
	FREE_EXTERNAL_MAILBOX,
	/* in_data: edgetpu_ext_client_info, out_data: edgetpu_ext_offload_info */
	START_OFFLOAD,
	/*
	 * out_data: iif_manager pointer
	 *
	 * E.g.,
	 * struct iif_manager *mgr;
	 * edgetpu_ext_driver_cmd(..., NULL, &mgr);
	 *
	 * If it succeeds in getting the IIF manager, the refcount of it will be incremented by 1
	 * and it is the responsibility of the caller side to decrement it back.
	 */
	GET_IIF_MANAGER,
};

enum edgetpu_ext_client_type {
	EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
	EDGETPU_EXTERNAL_CLIENT_TYPE_AOC,
};

/*
 * Interface used by other modules to send commands to EdgeTPU driver.
 *
 * @edgetpu_dev: edgetpu device to send command to
 * @client_type: type for identification of client by EdgeTPU driver
 * @cmd_id: type of command to send to EdgeTPU driver
 * @in_data: (in param) any data required to be sent based on cmd_id
 * @out_data: (out param) any data expected to receive based on cmd_id
 *
 * Returns:
 *	0 on success or negative error code on error.
 */
int edgetpu_ext_driver_cmd(struct device *edgetpu_dev, enum edgetpu_ext_client_type client_type,
			   enum edgetpu_ext_commands cmd_id, void *in_data, void *out_data);

#endif /*__TPU_EXT_H__*/
