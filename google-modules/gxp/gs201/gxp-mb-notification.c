// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP notification implementation on top of the mailbox driver.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/bitops.h>

#include "gxp-mailbox.h"
#include "gxp-mailbox-driver.h"
#include "gxp-notification.h"

int gxp_notification_register_handler(struct gxp_dev *gxp, uint core,
				      enum gxp_notification_to_host_type type,
				      struct work_struct *handler)
{
	struct gxp_mailbox *mailbox;

	if (core >= GXP_NUM_CORES || type >= HOST_NOTIF_MAX)
		return -EINVAL;

	mailbox = gxp->mailbox_mgr->mailboxes[core];
	if (!mailbox)
		return -ENODEV;

	return gxp_mailbox_register_interrupt_handler(mailbox, type, handler);
}

int gxp_notification_unregister_handler(struct gxp_dev *gxp, uint core,
					enum gxp_notification_to_host_type type)
{
	struct gxp_mailbox *mailbox;

	if (core >= GXP_NUM_CORES || type >= HOST_NOTIF_MAX)
		return -EINVAL;

	mailbox = gxp->mailbox_mgr->mailboxes[core];
	if (!mailbox)
		return -ENODEV;

	return gxp_mailbox_unregister_interrupt_handler(mailbox, type);
}

int gxp_notification_send(struct gxp_dev *gxp, uint core,
			  enum gxp_notification_to_core_type type)
{
	struct gxp_mailbox *mailbox;

	/*
	 * The mailbox submodule handles outgoing command interrupts directly
	 * so `CORE_NOTIF_MAILBOX_COMMAND` is not a valid input for this
	 * implementation.
	 */
	if (core >= GXP_NUM_CORES || type == CORE_NOTIF_MAILBOX_COMMAND ||
	    type >= GXP_MAILBOX_INT_BIT_COUNT || type >= CORE_NOTIF_MAX)
		return -EINVAL;

	mailbox = gxp->mailbox_mgr->mailboxes[core];
	if (!mailbox)
		return -ENODEV;

	gxp_mailbox_generate_device_interrupt(mailbox, BIT(type));

	return 0;
}
