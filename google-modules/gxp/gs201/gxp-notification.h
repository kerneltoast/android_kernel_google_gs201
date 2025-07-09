/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP notification interface.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_NOTIFICATION_H__
#define __GXP_NOTIFICATION_H__

#include <linux/workqueue.h>

#include "gxp-internal.h"

enum gxp_notification_to_host_type {
	HOST_NOTIF_MAILBOX_RESPONSE = 0,
	HOST_NOTIF_DEBUG_DUMP_READY = 1,
	HOST_NOTIF_CORE_TELEMETRY_STATUS = 2,
	HOST_NOTIF_MAX
};

enum gxp_notification_to_core_type {
	CORE_NOTIF_MAILBOX_COMMAND = 0,
	CORE_NOTIF_GENERATE_DEBUG_DUMP = 1,
	CORE_NOTIF_TELEMETRY_STATUS = 2,
	CORE_NOTIF_SUSPEND_REQUEST = 3,
	CORE_NOTIF_MAX
};

/**
 * gxp_notification_register_handler() - Register a work_struct to be called
 *                                       when the specified @core sends a
 *                                       notification of the specified @type.
 * @gxp: The GXP device to register the handler for
 * @core: The core inside the GXP device to receive notifications from
 * @type: The `gxp_notification_to_host_type` of notification to handle
 * @handler: A callback to be invoked via `schedule_work()` when a notification
 *           of @type arrives.
 *
 * This function requires the specified @core has its firmware loaded and
 * initialized before this function is called.
 *
 * If the callback requires additional context, such as the core number or a
 * pointer to @gxp, the caller should allocate @handler as part of wrapper
 * struct containing any context, then obtain that wrapping struct with
 * `container_of()` inside the handler's callback.
 *
 * Return:
 * * 0       - Success
 * * -EINVAL - The specified @core or @type is not valid
 * * -ENODEV - The specified @core is not running firmware
 */
int gxp_notification_register_handler(struct gxp_dev *gxp, uint core,
				      enum gxp_notification_to_host_type type,
				      struct work_struct *handler);

/**
 * gxp_notification_unregister_handler() - Unregister the work for handling
 *                                         notifications of type @type from core
 *                                         @core.
 * @gxp: The GXP device to unregister the handler for
 * @core: The core inside the GXP device to remove the notifications handler for
 * @type: The `gxp_notification_to_host_type` of notification to unregister
 *
 * This function requires the specified @core has its firmware loaded and
 * initialized before this function is called.
 *
 * Return:
 * * 0       - Success
 * * -EINVAL - The specified @core or @type is not valid
 * * -ENODEV - The specified @core is not running firmware
 */
int gxp_notification_unregister_handler(
	struct gxp_dev *gxp, uint core,
	enum gxp_notification_to_host_type type);

/**
 * gxp_notification_send() - Send a notification of @type to @core.
 * @gxp: The GXP device to send a notification to
 * @core: The core inside the GXP device to route the notification to
 * @type:The `gxp_notification_to_core_type` of notification to send
 *
 * This function requires the specified @core has its firmware loaded and
 * initialized before this function is called.
 *
 * The caller must also hold gxp->vd_semaphore for reading, to ensure firmware
 * continues running until this call completes.
 *
 * Return:
 * * 0       - Success
 * * -EINVAL - The specified @core or @type is not valid
 * * -ENODEV - The specified @core is not running firmware
 */
int gxp_notification_send(struct gxp_dev *gxp, uint core,
			  enum gxp_notification_to_core_type type);

#endif /* __GXP_NOTIFICATION_H__ */
