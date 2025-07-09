/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * IIF driver sync file.
 *
 * To export fences to the userspace, the driver will allocate a sync file to a fence and will
 * return its file descriptor to the user. The user can distinguish fences with it. The driver will
 * convert the file descriptor to the corresponding fence ID and will pass it to the IP.
 *
 * Copyright (C) 2023-2024 Google LLC
 */

#ifndef __IIF_IIF_SYNC_FILE_H__
#define __IIF_IIF_SYNC_FILE_H__

#include <linux/file.h>
#include <linux/wait.h>

#include <iif/iif-fence.h>

#define IIF_SYNC_FILE_FLAGS_POLL_ENABLED 0

/* Sync file which will be exported to the userspace to sync with the fence. */
struct iif_sync_file {
	/* File pointer. */
	struct file *file;
	/* Fence object. */
	struct iif_fence *fence;
	/* Queue of polling the file. */
	wait_queue_head_t wq;
	/* Node which will be added to the callback list of the fence. */
	struct iif_fence_poll_cb poll_cb;
	/*
	 * Flags.
	 *  [0:0]   - Set if the user has been polling the file. (IIF_SYNC_FILE_FLAGS_POLL_ENABLED)
	 *  [1:31]  - Reserved.
	 */
	unsigned long flags;
};

/* Opens a file which will be exported to the userspace to sync with @fence. */
struct iif_sync_file *iif_sync_file_create(struct iif_fence *fence);

/*
 * Gets the sync file from @fd. If @fd is not for iif_sync_file, it will return a negative error
 * pointer.
 *
 * The caller must put the file pointer (i.e., fput(sync_file->file)) to release the file.
 */
struct iif_sync_file *iif_sync_file_fdget(int fd);

#endif /* __IIF_IIF_SYNC_FILE_H__ */
