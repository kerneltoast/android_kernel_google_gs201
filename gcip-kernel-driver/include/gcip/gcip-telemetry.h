/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GCIP telemetry: logging and tracing.
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __GCIP_TELEMETRY_H__
#define __GCIP_TELEMETRY_H__

#include <linux/device.h>
#include <linux/eventfd.h>
#include <linux/mutex.h>
#include <linux/rwlock_types.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

/* Log level codes used by gcip firmware. */
#define GCIP_FW_LOG_LEVEL_VERBOSE (2)
#define GCIP_FW_LOG_LEVEL_DEBUG (1)
#define GCIP_FW_LOG_LEVEL_INFO (0)
#define GCIP_FW_LOG_LEVEL_WARN (-1)
#define GCIP_FW_LOG_LEVEL_ERROR (-2)
#define GCIP_FW_LOG_LEVEL_FATAL (-3)

#define GCIP_FW_DMESG_LOG_LEVEL (GCIP_FW_LOG_LEVEL_WARN)

/* When log data arrives, recheck for more log data after this delay. */
#define GCIP_TELEMETRY_LOG_RECHECK_DELAY 200 /* ms */

enum gcip_telemetry_state {
	GCIP_TELEMETRY_DISABLED = 0,
	GCIP_TELEMETRY_ENABLED = 1,
	GCIP_TELEMETRY_INVALID = -1,
};

/* To specify the target of operation. */
enum gcip_telemetry_type {
	GCIP_TELEMETRY_LOG = 0,
	GCIP_TELEMETRY_TRACE = 1,
};

struct gcip_telemetry_header {
	u32 head;
	u32 size;
	u32 reserved0[14]; /* Place head and tail into different cache lines */
	u32 tail;
	u32 entries_dropped; /* Number of entries dropped due to buffer full */
	u32 reserved1[14]; /* Pad to 128 bytes in total */
};

struct gcip_log_entry_header {
	s16 code;
	u16 length;
	u64 timestamp;
	u16 crc16;
} __packed;

struct gcip_telemetry {
	/* Device used for logging and memory allocation. */
	struct device *dev;

	/*
	 * State transitioning is to prevent racing in IRQ handlers. e.g. the interrupt comes when
	 * the kernel is releasing buffers.
	 */
	enum gcip_telemetry_state state;
	spinlock_t state_lock; /* protects state */

	struct gcip_telemetry_header *header;

	struct eventfd_ctx *ctx; /* signal this to notify the runtime */
	rwlock_t ctx_lock; /* protects ctx */
	const char *name; /* for debugging */

	struct work_struct work; /* worker for handling data */
	/* Fallback function to call for default log/trace handling. */
	void (*fallback_fn)(struct gcip_telemetry *tel);
	struct mutex mmap_lock; /* protects mmapped_count */
	long mmapped_count; /* number of VMAs that are mapped to this telemetry buffer */
};

struct gcip_kci;

struct gcip_telemetry_kci_args {
	struct gcip_kci *kci;
	u64 addr;
	u32 size;
};

/* Sends telemetry KCI through send kci callback and args. */
int gcip_telemetry_kci(struct gcip_telemetry *tel,
		       int (*send_kci)(struct gcip_telemetry_kci_args *),
		       struct gcip_telemetry_kci_args *args);
/* Sets the eventfd for telemetry. */
int gcip_telemetry_set_event(struct gcip_telemetry *tel, u32 eventfd);
/* Unsets the eventfd for telemetry. */
void gcip_telemetry_unset_event(struct gcip_telemetry *tel);
/* Fallback to log messages from host CPU to dmesg. */
void gcip_telemetry_fw_log(struct gcip_telemetry *log);
/* Fallback to consumes the trace buffer. */
void gcip_telemetry_fw_trace(struct gcip_telemetry *trace);
/* Interrupt handler to schedule the worker when the buffer is not empty. */
void gcip_telemetry_irq_handler(struct gcip_telemetry *tel);
/* Increases the telemetry mmap count. */
void gcip_telemetry_inc_mmap_count(struct gcip_telemetry *tel, int dif);
/* Mmaps the telemetry buffer through mmap callback and args. */
int gcip_telemetry_mmap_buffer(struct gcip_telemetry *tel, int (*mmap)(void *), void *args);
/*
 * Initializes struct gcip_telemetry.
 *
 * @vaddr: Virtual address of the queue buffer.
 * @size: Size of the queue buffer. Must be power of 2 and greater than the size of struct
 * gcip_telemetry_header.
 * @fallback_fn: Fallback function to call for default log/trace handling.
 */
int gcip_telemetry_init(struct device *dev, struct gcip_telemetry *tel, const char *name,
			void *vaddr, const size_t size,
			void (*fallback_fn)(struct gcip_telemetry *));
/* Exits and sets the telemetry state to GCIP_TELEMETRY_INVALID. */
void gcip_telemetry_exit(struct gcip_telemetry *tel);

#endif /* __GCIP_TELEMETRY_H__ */
