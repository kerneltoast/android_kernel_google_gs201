// SPDX-License-Identifier: GPL-2.0-only
/* init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <linux/module.h>
#include <trace/hooks/fs.h>

extern void vh_ep_create_wakeup_source_mod(void *data, char *name, int len);
extern void vh_timerfd_create_mod(void *data, char *name, int len);

static int vh_fs_init(void)
{
	int ret;

	ret = register_trace_android_vh_ep_create_wakeup_source(vh_ep_create_wakeup_source_mod,
								NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_timerfd_create(vh_timerfd_create_mod, NULL);
	if (ret)
		return ret;

	return 0;
}

module_init(vh_fs_init);
MODULE_LICENSE("GPL v2");
