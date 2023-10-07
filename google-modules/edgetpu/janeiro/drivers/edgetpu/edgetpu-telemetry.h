/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Edge TPU ML accelerator telemetry: logging and tracing.
 *
 * Copyright (C) 2019-2020 Google, Inc.
 */
#ifndef __EDGETPU_TELEMETRY_H__
#define __EDGETPU_TELEMETRY_H__

#include <linux/eventfd.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "edgetpu-internal.h"
#include "edgetpu-kci.h"

/* Log level codes used by edgetpu firmware */
#define EDGETPU_FW_LOG_LEVEL_VERBOSE (2)
#define EDGETPU_FW_LOG_LEVEL_DEBUG (1)
#define EDGETPU_FW_LOG_LEVEL_INFO (0)
#define EDGETPU_FW_LOG_LEVEL_WARN (-1)
#define EDGETPU_FW_LOG_LEVEL_ERROR (-2)

#define EDGETPU_FW_DMESG_LOG_LEVEL (EDGETPU_FW_LOG_LEVEL_ERROR)

/* Buffer size must be a power of 2 */
#define EDGETPU_TELEMETRY_LOG_BUFFER_SIZE (16 * 4096)
#define EDGETPU_TELEMETRY_TRACE_BUFFER_SIZE (64 * 4096)

enum edgetpu_telemetry_state {
	EDGETPU_TELEMETRY_DISABLED = 0,
	EDGETPU_TELEMETRY_ENABLED = 1,
	EDGETPU_TELEMETRY_INVALID = -1,
};

/* To specify the target of operation. */
enum edgetpu_telemetry_type {
	EDGETPU_TELEMETRY_LOG = 0,
	EDGETPU_TELEMETRY_TRACE = 1,
};

struct edgetpu_telemetry_header {
	u32 head;
	u32 size;
	u32 reserved0[14]; /* Place head and tail into different cache lines */
	u32 tail;
	u32 entries_dropped; /* Number of entries dropped due to buffer full */
	u32 reserved1[14]; /* Pad to 128 bytes in total */
};

struct edgetpu_log_entry_header {
	s16 code;
	u16 length;
	u64 timestamp;
	u16 crc16;
} __packed;

struct edgetpu_telemetry {
	struct edgetpu_dev *etdev;

	/*
	 * State transitioning is to prevent racing in IRQ handlers. e.g. the
	 * interrupt comes when the kernel is releasing buffers.
	 */
	enum edgetpu_telemetry_state state;
	spinlock_t state_lock; /* protects state */

	struct edgetpu_coherent_mem coherent_mem;
	struct edgetpu_telemetry_header *header;
	/*
	 * If coherent_mem buffer is provided by the caller in
	 * edgetpu_telemetry_init, the caller is responsible for
	 * releasing/unmapping it.
	 */
	bool caller_mem;

	struct eventfd_ctx *ctx; /* signal this to notify the runtime */
	rwlock_t ctx_lock; /* protects ctx */
	const char *name; /* for debugging */
	bool inited; /* whether telemetry_init() succeeded */

	/* Worker for handling data. */
	struct work_struct work;
	/* Fallback function to call for default log/trace handling. */
	void (*fallback_fn)(struct edgetpu_telemetry *tel);
	struct mutex mmap_lock; /* protects mmapped_count */
	/* number of VMAs that are mapped to this telemetry buffer */
	long mmapped_count;
};

struct edgetpu_telemetry_ctx {
	struct edgetpu_telemetry log;
	struct edgetpu_telemetry trace;
};

/*
 * Allocates resources needed for @etdev->telemetry.
 *
 * Optionally provide arrays of etdev->num_cores coherent_mem buffers for log and trace.
 * If any of these are NULL, they will be allocated and freed by telemetry code.
 *
 * Returns 0 on success, or a negative errno on error.
 */
int edgetpu_telemetry_init(struct edgetpu_dev *etdev,
			   struct edgetpu_coherent_mem *log_mem,
			   struct edgetpu_coherent_mem *trace_mem);

/*
 * Disable the telemetry if enabled, release resources allocated in init().
 */
void edgetpu_telemetry_exit(struct edgetpu_dev *etdev);

/*
 * Sends the KCI commands about telemetry buffers to the device.
 *
 * Returns the code of KCI response, or a negative errno on error.
 */
int edgetpu_telemetry_kci(struct edgetpu_dev *etdev);

/*
 * Sets the eventfd to notify the runtime when an IRQ is sent from the device.
 *
 * Returns 0 on success, or a negative errno on error.
 */
int edgetpu_telemetry_set_event(struct edgetpu_dev *etdev,
				enum edgetpu_telemetry_type type, u32 eventfd);
/* Removes previously set event. */
void edgetpu_telemetry_unset_event(struct edgetpu_dev *etdev,
				   enum edgetpu_telemetry_type type);

/* Checks telemetries and signals eventfd if needed. */
void edgetpu_telemetry_irq_handler(struct edgetpu_dev *etdev);

/* debugfs mappings dump */
void edgetpu_telemetry_mappings_show(struct edgetpu_dev *etdev,
				     struct seq_file *s);

/* Map telemetry buffer into user space. */
int edgetpu_mmap_telemetry_buffer(struct edgetpu_dev *etdev, enum edgetpu_telemetry_type type,
				  struct vm_area_struct *vma, int core_id);
void edgetpu_telemetry_inc_mmap_count(struct edgetpu_dev *etdev, enum edgetpu_telemetry_type type,
				      int core_id);
void edgetpu_telemetry_dec_mmap_count(struct edgetpu_dev *etdev, enum edgetpu_telemetry_type type,
				      int core_id);

#endif /* __EDGETPU_TELEMETRY_H__ */
