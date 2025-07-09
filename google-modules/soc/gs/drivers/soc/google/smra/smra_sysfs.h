/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __SMRA_SYSFS_H__
#define __SMRA_SYSFS_H__

#include <linux/types.h>

#define SMRA_MAX_TARGET_CNT 3
#define SMRA_DEFAULT_BUFFER_SIZE 16384
#define SMRA_DEFAULT_MERGE_THRESHOLD 5000000  /* 5ms */
#define MAX_PID_LEN 12

struct smra_config {
	bool recording_on;
	bool buffer_has_trace;
	s64 merge_threshold;
	int buffer_size;
	int nr_targets;
	pid_t target_pids[SMRA_MAX_TARGET_CNT];
};

extern struct smra_config smra_config;

int smra_sysfs_init(void);

#endif
