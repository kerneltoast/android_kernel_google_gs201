// SPDX-License-Identifier: GPL-2.0-only
/* timerfd.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <linux/fs.h>

/*****************************************************************************/
/*                       Modified Code Section                               */
/*****************************************************************************/

static atomic_t instance_count = ATOMIC_INIT(0);

/*
 * This part of code is vendor hook functions, which modify or extend the
 * original functions.
 */
void vh_timerfd_create_mod(void *data, char *file_name_buf, int len)
{
    char task_comm_buf[sizeof(current->comm)];
    int instance;

    instance = atomic_inc_return(&instance_count);
    get_task_comm(task_comm_buf, current);
    snprintf(file_name_buf, len, "[timerfd%d:%s]", instance, task_comm_buf);
}
