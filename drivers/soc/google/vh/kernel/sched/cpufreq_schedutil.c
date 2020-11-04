// SPDX-License-Identifier: GPL-2.0-only
/* vendor_hook.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <kernel/sched/sched.h>

void vh_set_sugov_sched_attr_pixel_mod(void *data, struct sched_attr *attr)
{
    memset(attr, 0, sizeof(struct sched_attr));
    attr->sched_policy = SCHED_FIFO;
    attr->sched_priority = MAX_USER_RT_PRIO / 2;
}