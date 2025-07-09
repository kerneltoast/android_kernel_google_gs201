/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PA_KILL_SYSFS_H__
#define __PA_KILL_SYSFS_H__

#include <linux/oom.h>

int pa_kill_sysfs_init(void);
int kill_process(void);

#endif
