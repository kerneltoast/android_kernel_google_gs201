/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MM_PIXEL_VMSCAN_H__
#define __MM_PIXEL_VMSCAN_H__

#include <trace/events/vmscan.h>

void vh_direct_reclaim_begin(void *data, int order, gfp_t gfp_mask);
void vh_direct_reclaim_end(void *data, unsigned long nr_reclaimed);
int create_vmscan_sysfs(struct kobject *mm_kobj);
void remove_vmscan_sysfs(void);
#endif
