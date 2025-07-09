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
static atomic_t atomic_epoll_id = ATOMIC_INIT(0);

void vh_ep_create_wakeup_source_mod(void *data, char *name, int len)
{
	char buf[64];
	char task_comm_buf[sizeof(current->comm)];
	int epoll_id = atomic_inc_return(&atomic_epoll_id);

	get_task_comm(task_comm_buf, current);
	strlcpy(buf, name, sizeof(buf));

	if (!strncmp(name, "eventpoll", sizeof("eventpoll")))
		snprintf(name, len, "epollm%d:%s", epoll_id, task_comm_buf);
	else
		snprintf(name, len, "epolld%d:%s.%s", epoll_id, task_comm_buf, buf);
}
