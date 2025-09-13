/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __SMRA_CORE_H__
#define __SMRA_CORE_H__

#include <linux/sched.h>
#include <linux/types.h>

#define MAX_PATH_LEN 256

/*
 * @smra_fault_info: information captured during do_read_fault()
 *
 * When smra_fault_info is pushed into buffer, get_file() is used to avoid
 * @file being released when we are still recording. It will be fput() later
 * when post-processing is done.
 */
struct smra_fault_info {
	struct file *file;
	pgoff_t offset;
	struct inode *inode;
	ktime_t time;
};

struct smra_info_buffer {
	struct smra_fault_info *fault_info;
	int cur;
	ssize_t size;
};

struct smra_target {
	pid_t pid;
	spinlock_t buf_lock;
	struct smra_info_buffer *buf;
	struct list_head list;
};

/*
 * @smra_metadata: The data structure for post-processed metadata.
 *
 * During post processing, multiple @smra_fault_info are merged into
 * single @smra_metadata. This is the structure used to present results
 * to the users.
 */
struct smra_metadata {
	char* path;
	pgoff_t offset;
	ktime_t time;
	int nr_pages;
	char buf[MAX_PATH_LEN];
	struct list_head list;
};

int smra_setup(pid_t target_pids[], int nr_targets, int buffer_size);
void smra_start(void);
void smra_stop(void);
void smra_reset(void);

int smra_post_processing(pid_t target_pids[], int nr_targets,
			 s64 merge_threshold, struct list_head footprints[]);
void smra_post_processing_cleanup(struct list_head footprints[],
				  int nr_targets);

#endif
