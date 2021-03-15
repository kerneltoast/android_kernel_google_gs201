// SPDX-License-Identifier: GPL-2.0-only
/* eventpoll.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <linux/fs.h>


/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/
/*
 * This part of code is vendor hook functions, which modify or extend the
 * original functions.
 */
void vh_ep_create_wakeup_source_mod(void *data, char *name, int len)
{
	char buf[64];
	char task_comm_buf[sizeof(current->comm)];

	get_task_comm(task_comm_buf, current);
	strlcpy(buf, name, sizeof(buf));

	if (!strncmp(name, "eventpoll", sizeof("eventpoll")))
		snprintf(name, len, "epoll:%s", task_comm_buf);
	else
		snprintf(name, len, "epollitem:%s.%s", task_comm_buf, buf);
}
