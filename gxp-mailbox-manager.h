/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Mailbox manager abstracts the mailbox interfaces of user commands.
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __GXP_MAILBOX_MANAGER_H__
#define __GXP_MAILBOX_MANAGER_H__

#include "gxp-internal.h"

struct gxp_mailbox;

typedef void __iomem *(*get_mailbox_base_t)(struct gxp_dev *gxp, uint index);

/*
 * Following callbacks will be used for manipulating the mailbox to communicating with the
 * firmware. By using this callbacks instead of calling the functions of each interface directly,
 * we can abstract the mailbox and reduce effort of updating the codes outside of the mailbox when
 * we refactor the mailbox in the future.
 */

/*
 * Called when allocates a mailbox. The mailbox will be release by the `release_mailbox_t`.
 *
 * Return a pointer of allocated mailbox or an error pointer if error occurred.
 *
 * This callback is required if the device is in direct mode, otherwise it is optional.
 */
typedef struct gxp_mailbox *(*allocate_mailbox_t)(
	struct gxp_mailbox_manager *mgr, struct gxp_virtual_device *vd,
	uint virt_core, u8 core_id);

/*
 * Called to release @mailbox previously allocated by `allocate_mailbox_t`.
 *
 * This callback is required if the device is in direct mode, otherwise it is optional.
 */
typedef void (*release_mailbox_t)(struct gxp_mailbox_manager *mgr,
				  struct gxp_virtual_device *vd, uint virt_core,
				  struct gxp_mailbox *mailbox);

/* Called when resets the @mailbox. */
typedef void (*reset_mailbox_t)(struct gxp_mailbox *mailbox);

/*
 * Called when requests synchronous commands. This callback will be called from the
 * `debugfs_mailbox_execute_cmd` function. The response will be returned to the @resp_seq,
 * @resp_status and `retval` of `struct gxp_response` will be returned as the return value of this
 * function.
 * You can pass NULL to @resp_seq and @resp_status if you don't need the result. See the
 * `struct gxp_response` for the details.
 *
 * Returns the value `retval` of `struct gxp_response` when the request succeeds. Otherwise,
 * returns a negative value as an error.
 *
 * This callback is always required regardless of the mode of device.
 */
typedef int (*execute_cmd_t)(struct gxp_client *client,
			     struct gxp_mailbox *mailbox, int virt_core,
			     u16 cmd_code, u8 cmd_priority, u64 cmd_daddr,
			     u32 cmd_size, u32 cmd_flags, u8 num_cores,
			     struct gxp_power_states power_states,
			     u64 *resp_seq, u16 *resp_status);

/*
 * Called when requests asynchronous commands. This callback will be called when
 * `GXP_MAILBOX_COMMAND_COMPAT` or `GXP_MAILBOX_COMMAND` ioctls are fired. The sequence number of
 * the command will be returned to the @cmd_seq. @eventfd will be signalled when the response
 * arrives.
 *
 * Returns a non-zero value when error occurs while putting the command to the cmd_queue of
 * mailbox.
 *
 * This callback is required if the device is in direct mode, otherwise it is optional.
 */
typedef int (*execute_cmd_async_t)(struct gxp_client *client,
				   struct gxp_mailbox *mailbox, int virt_core,
				   u16 cmd_code, u8 cmd_priority, u64 cmd_daddr,
				   u32 cmd_size, u32 cmd_flags,
				   struct gxp_power_states power_states,
				   u64 *cmd_seq);

/*
 * Called when waiting for an asynchronous response which is requested by `execute_cmd_async`.
 * This callback will be called when `GXP_MAILBOX_RESPONSE` ioctl is fired. The response will be
 * returned to the @resp_seq, @resp_status and @resp_retval. You can pass NULL to them if you don't
 * need the result. See the `struct gxp_response` for the details. The corresponding error code of
 * the response status will be set to the @error_code.
 *
 * Returns 0 if it succeed to get the response. Otherwise, returns a non-zero value as an error.
 *
 * This callback is required if the device is in direct mode, otherwise it is optional.
 */
typedef int (*wait_async_resp_t)(struct gxp_client *client, int virt_core,
				 u64 *resp_seq, u16 *resp_status,
				 u32 *resp_retval, u16 *error_code);

/*
 * Called when cleans up unconsumed async responses in the queue which arrived or timed out.
 * This callback will be called when the @vd is released.
 *
 * This callback is always required regardless of the mode of device.
 */
typedef void (*release_unconsumed_async_resps_t)(struct gxp_virtual_device *vd);

/*
 * This structure manages how the mailbox works with user commands.
 * The way how the mailbox works is dependent on the what kind of interface is used by the device.
 * To minimize the effort of updating the codes outside of the mailbox, it abstracts the interfaces
 * by defining the callbacks above.
 */
struct gxp_mailbox_manager {
	struct gxp_dev *gxp;
	u8 num_cores;
	struct gxp_mailbox **mailboxes;
	get_mailbox_base_t get_mailbox_csr_base;
	get_mailbox_base_t get_mailbox_data_base;
	allocate_mailbox_t allocate_mailbox;
	release_mailbox_t release_mailbox;
	reset_mailbox_t reset_mailbox;
	execute_cmd_t execute_cmd;
	execute_cmd_async_t execute_cmd_async;
	wait_async_resp_t wait_async_resp;
	release_unconsumed_async_resps_t release_unconsumed_async_resps;
};

/*
 * Allocate the mailbox manager.
 *
 * In general, only one mailbox manager will be used by @gxp. What kind of mailbox interface will
 * be used is decided internally.
 */
struct gxp_mailbox_manager *gxp_mailbox_create_manager(struct gxp_dev *gxp,
						       uint num_cores);

/* Destroy and free the mailbox manager. */
void gxp_mailbox_destroy_manager(struct gxp_dev *gxp,
				 struct gxp_mailbox_manager *mgr);

#endif /* __GXP_MAILBOX_MANAGER_H__ */
