/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Utility functions for interfacing other modules with Edge TPU ML
 * accelerator.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#ifndef __TPU_EXT_H__
#define __TPU_EXT_H__

#include <linux/device.h>
#include <linux/types.h>

/*
 * Data structures to be used for MAILBOX_EXTERNAL_INFO_GET command
 */
struct edgetpu_external_mailbox_descriptor {
	int mailbox_id;
	phys_addr_t cmdq_pa, respq_pa;
};

/*
 * Structure to hold information about mailboxes.
 * Client driver should allocate at least @count number of @mailboxes array and
 * specify the value in @count.
 *
 * EdgeTPU driver checks this @count to ensure enough memory available before
 * filling in data.
 * EdgeTPU driver changes @count to the number of @mailboxes on
 * edgetpu_send_cmd() success.
 */
struct edgetpu_external_mailbox_info {
	u32 count; /* number of external mailboxes */
	size_t cmdq_size, respq_size;
	struct edgetpu_external_mailbox_descriptor mailboxes[];
};

enum edgetpu_external_commands {
	MAILBOX_EXTERNAL_INFO_GET, /* in_data: client_id, out_data: edgetpu_external_mailbox_info */
};

enum edgetpu_external_client_type {
	EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
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
int edgetpu_ext_driver_cmd(struct device *edgetpu_dev,
			   enum edgetpu_external_client_type client_type,
			   enum edgetpu_external_commands cmd_id, void *in_data, void *out_data);

#endif /*__TPU_EXT_H__*/
