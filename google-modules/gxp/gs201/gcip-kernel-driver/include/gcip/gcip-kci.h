/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Kernel Control Interface, implements the protocol between AP kernel and GCIP firmware.
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __GCIP_KCI_H__
#define __GCIP_KCI_H__

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <gcip/gcip-mailbox.h>

/*
 * The status field in a firmware response is set to this by us when the response is fetched from
 * the queue.
 */
#define GCIP_KCI_STATUS_OK GCIP_MAILBOX_STATUS_OK
/*
 * gcip_kci#mailbox.wait_list uses this value to record the status of responses that haven't been
 * received yet.
 */
#define GCIP_KCI_STATUS_WAITING_RESPONSE GCIP_MAILBOX_STATUS_WAITING_RESPONSE
/*
 * Used when an expected response is not received, see the documentation of
 * gcip_mailbox_handle_response() for details.
 */
#define GCIP_KCI_STATUS_NO_RESPONSE GCIP_MAILBOX_STATUS_NO_RESPONSE

/*
 * Command/response sequence numbers capped at half the range of the 64-bit value range. The second
 * half is reserved for incoming requests from firmware.
 * These are tagged with the MSB set.
 */
#define GCIP_KCI_REVERSE_FLAG (0x8000000000000000ull)

/* Command/response queue elements for KCI. */

struct gcip_kci_dma_descriptor {
	u64 address;
	u32 size;
	u32 flags;
};

struct gcip_kci_command_element {
	/*
	 * Set by gcip_kci_push_cmd() in case of KCI cmd and copied from the RKCI cmd in case of
	 * RKCI response.
	 */
	u64 seq;
	u16 code;
	u16 reserved[3]; /* Explicit padding, does not affect alignment. */
	struct gcip_kci_dma_descriptor dma;
} __packed;

struct gcip_kci_response_element {
	u64 seq;
	u16 code;
	/*
	 * Reserved for host use - firmware can't touch this.
	 * If a value is written here it will be discarded and overwritten during response
	 * processing.  However, when repurposed as an RKCI command, the FW can set this field.
	 */
	u16 status;
	/*
	 * Return value is not currently needed by KCI command responses.
	 * For reverse KCI commands this is set as value2.
	 */
	u32 retval;
} __packed;

/*
 * Definition of code in command elements.
 * Code for KCI is a 16-bit unsigned integer.
 */
enum gcip_kci_code {
	GCIP_KCI_CODE_ACK = 0,
	GCIP_KCI_CODE_UNMAP_BUFFER = 1,
	GCIP_KCI_CODE_MAP_LOG_BUFFER = 2,
	GCIP_KCI_CODE_JOIN_GROUP = 3,
	GCIP_KCI_CODE_LEAVE_GROUP = 4,
	GCIP_KCI_CODE_MAP_TRACE_BUFFER = 5,
	GCIP_KCI_CODE_SHUTDOWN = 7,
	GCIP_KCI_CODE_GET_DEBUG_DUMP = 8,
	GCIP_KCI_CODE_OPEN_DEVICE = 9,
	GCIP_KCI_CODE_CLOSE_DEVICE = 10,
	GCIP_KCI_CODE_FIRMWARE_INFO = 11,
	GCIP_KCI_CODE_GET_USAGE = 12,
	GCIP_KCI_CODE_NOTIFY_THROTTLING = 13,
	GCIP_KCI_CODE_BLOCK_BUS_SPEED_CONTROL = 14,
	GCIP_KCI_CODE_ALLOCATE_VMBOX = 15,
	GCIP_KCI_CODE_RELEASE_VMBOX = 16,
	GCIP_KCI_CODE_LINK_OFFLOAD_VMBOX = 17,
	GCIP_KCI_CODE_UNLINK_OFFLOAD_VMBOX = 18,
	GCIP_KCI_CODE_FIRMWARE_TRACING_LEVEL = 19,
	GCIP_KCI_CODE_THERMAL_CONTROL = 20,

	GCIP_KCI_CODE_RKCI_ACK = 256,
};

/*
 * Definition of reverse KCI request code ranges.
 * Code for reverse KCI is a 16-bit unsigned integer.
 * The first half is reserved for the chip specific codes and the generic codes can use the
 * second half.
 */
enum gcip_reverse_kci_code {
	GCIP_RKCI_CHIP_CODE_FIRST = 0,
	GCIP_RKCI_PM_QOS_REQUEST,
	GCIP_RKCI_CHANGE_BTS_SCENARIO,
	GCIP_RKCI_PM_QOS_BTS_REQUEST,
	GCIP_RKCI_DSP_CORE_TELEMETRY_TRY_READ,
	GCIP_RKCI_CLIENT_FATAL_ERROR_NOTIFY,
	GCIP_RKCI_CHIP_CODE_LAST = 0x7FFF,
	GCIP_RKCI_GENERIC_CODE_FIRST = 0x8000,
	GCIP_RKCI_FIRMWARE_CRASH = GCIP_RKCI_GENERIC_CODE_FIRST + 0,
	GCIP_RKCI_JOB_LOCKUP = GCIP_RKCI_GENERIC_CODE_FIRST + 1,
	GCIP_RKCI_GENERIC_CODE_LAST = 0xFFFF,
};

/*
 * Definition of code in response elements.
 * It is a 16-bit unsigned integer.
 */
enum gcip_kci_error {
	GCIP_KCI_ERROR_OK = 0, /* Not an error; returned on success. */
	GCIP_KCI_ERROR_CANCELLED = 1,
	GCIP_KCI_ERROR_UNKNOWN = 2,
	GCIP_KCI_ERROR_INVALID_ARGUMENT = 3,
	GCIP_KCI_ERROR_DEADLINE_EXCEEDED = 4,
	GCIP_KCI_ERROR_NOT_FOUND = 5,
	GCIP_KCI_ERROR_ALREADY_EXISTS = 6,
	GCIP_KCI_ERROR_PERMISSION_DENIED = 7,
	GCIP_KCI_ERROR_RESOURCE_EXHAUSTED = 8,
	GCIP_KCI_ERROR_FAILED_PRECONDITION = 9,
	GCIP_KCI_ERROR_ABORTED = 10,
	GCIP_KCI_ERROR_OUT_OF_RANGE = 11,
	GCIP_KCI_ERROR_UNIMPLEMENTED = 12,
	GCIP_KCI_ERROR_INTERNAL = 13,
	GCIP_KCI_ERROR_UNAVAILABLE = 14,
	GCIP_KCI_ERROR_DATA_LOSS = 15,
	GCIP_KCI_ERROR_UNAUTHENTICATED = 16,
};

/* Type of the chip of the offload vmbox to be linked. */
enum gcip_kci_offload_chip_type {
	GCIP_KCI_OFFLOAD_CHIP_TYPE_TPU = 0,
};

/*
 * Reason for triggering the CMD doorbell.
 * The CMD doorbell is triggered either when a CMD is pushed or the RESP that might blocks the FW is
 * consumed.
 */
enum gcip_kci_doorbell_reason {
	GCIP_KCI_PUSH_CMD,
	GCIP_KCI_CONSUME_RESP,
};

/* Struct to hold a circular buffer for incoming KCI responses. */
struct gcip_reverse_kci {
	/* Reverse kci buffer head. */
	unsigned long head;
	/* Reverse kci buffer tail. */
	unsigned long tail;
	/*
	 * Maximum number of outstanding KCI requests from firmware.
	 * This is used to size a circular buffer, so it must be a power of 2.
	 */
	u32 buffer_size;
	struct gcip_kci_response_element *buffer;
	/* Lock to push elements in the buffer from the interrupt handler. */
	spinlock_t producer_lock;
	/* Lock to pop elements from the buffer in the worker. */
	spinlock_t consumer_lock;
	/* Worker to handle responses. */
	struct work_struct work;
};

struct gcip_kci;

/*
 * KCI operators.
 * For in_interrupt() context, see the implementation of gcip_kci_handle_irq for details.
 */
struct gcip_kci_ops {
	/* Mandatory. */
	/*
	 * Gets the head of mailbox command queue.
	 * Context: normal.
	 */
	u32 (*get_cmd_queue_head)(struct gcip_kci *kci);
	/*
	 * Gets the tail of mailbox command queue.
	 * Context: normal.
	 */
	u32 (*get_cmd_queue_tail)(struct gcip_kci *kci);
	/*
	 * Increases the tail of mailbox command queue by @inc.
	 * Context: normal.
	 */
	void (*inc_cmd_queue_tail)(struct gcip_kci *kci, u32 inc);

	/*
	 * Gets the size of mailbox response queue.
	 * Context: normal.
	 */
	u32 (*get_resp_queue_size)(struct gcip_kci *kci);
	/*
	 * Gets the head of mailbox response queue.
	 * Context: normal and in_interrupt().
	 */
	u32 (*get_resp_queue_head)(struct gcip_kci *kci);
	/*
	 * Gets the tail of mailbox response queue.
	 * Context: normal and in_interrupt().
	 */
	u32 (*get_resp_queue_tail)(struct gcip_kci *kci);
	/*
	 * Increases the head of mailbox response queue by @inc.
	 * Context: normal and in_interrupt().
	 */
	void (*inc_resp_queue_head)(struct gcip_kci *kci, u32 inc);
	/*
	 * Rings the doorbell.
	 * Context: normal.
	 */
	void (*trigger_doorbell)(struct gcip_kci *kci, enum gcip_kci_doorbell_reason);

	/* Optional. */
	/*
	 * Reverse KCI handler called by the worker. Only required if reverse kci is enabled.
	 * Context: normal.
	 */
	void (*reverse_kci_handle_response)(struct gcip_kci *kci,
					    struct gcip_kci_response_element *resp);
	/*
	 * Usage updater called by the worker.
	 * Context: normal.
	 */
	int (*update_usage)(struct gcip_kci *kci);
};

struct gcip_kci {
	/* Device used for logging and memory allocation. */
	struct device *dev;
	/* Mailbox used by KCI. */
	struct gcip_mailbox mailbox;
	/* Protects cmd_queue. */
	struct mutex cmd_queue_lock;
	/* Protects resp_queue. */
	spinlock_t resp_queue_lock;
	/* Queue for waiting for the response doorbell to be rung. */
	wait_queue_head_t resp_doorbell_waitq;
	/* Protects wait_list. */
	spinlock_t wait_list_lock;
	/* Worker of consuming responses. */
	struct work_struct work;
	/* Handler for reverse (firmware -> kernel) requests. */
	struct gcip_reverse_kci rkci;
	/* Worker that sends update usage KCI. */
	struct work_struct usage_work;
	/* KCI operators. */
	const struct gcip_kci_ops *ops;
	/* Private data. */
	void *data;
};

/*
 * Arguments for gcip_kci_init.
 *
 * For the following arguments, see struct gcip_kci and struct gcip_reverse_kci for details.
 * : `dev`, `rkci_buffer_size`, `ops` and `data`.
 *
 * For the following arguments, see struct gcip_mailbox for details. They will be passed to the
 * struct gcip_mailbox using struct gcip_mailbox_args internally.
 * : `dev`, `cmd_queue`, `resp_queue`, `queue_wrap_bit` and `timeout`.
 */
struct gcip_kci_args {
	struct device *dev;
	void *cmd_queue;
	void *resp_queue;
	u32 queue_wrap_bit;
	u32 rkci_buffer_size;
	u32 timeout;
	const struct gcip_kci_ops *ops;
	void *data;
};

/* Initializes a KCI object. */
int gcip_kci_init(struct gcip_kci *kci, const struct gcip_kci_args *args);

/* Cancels KCI and reverse KCI workers and workers that may send KCIs. */
void gcip_kci_cancel_work_queues(struct gcip_kci *kci);

/*
 * Release KCI.
 * Caller must call gcip_kci_cancel_work_queues before calling gcip_kci_release.
 */
void gcip_kci_release(struct gcip_kci *kci);

/*
 * Pushes an element to cmd queue and waits for the response.
 * Returns -ETIMEDOUT if no response is received within kci->mailbox.timeout.
 *
 * Returns the code of response, or a negative errno on error.
 */
int gcip_kci_send_cmd(struct gcip_kci *kci, struct gcip_kci_command_element *cmd);

/*
 * Pushes an element to cmd queue and waits for the response.
 * Returns -ETIMEDOUT if no response is received within kci->mailbox.timeout msecs.
 *
 * Returns the code of response, or a negative errno on error.
 * @resp is updated with the response, as to retrieve returned retval field.
 */
int gcip_kci_send_cmd_return_resp(struct gcip_kci *kci, struct gcip_kci_command_element *cmd,
				  struct gcip_kci_response_element *resp);

/*
 * Interrupt handler.
 * This function should be called when the interrupt of KCI mailbox is fired.
 */
void gcip_kci_handle_irq(struct gcip_kci *kci);

/*
 * Schedules a usage update worker.
 *
 * For functions that don't require the usage to be updated immediately, use this function instead
 * of update_usage in struct gcip_kci_ops.
 */
void gcip_kci_update_usage_async(struct gcip_kci *kci);

/* Gets the KCI private data. */
static inline void *gcip_kci_get_data(struct gcip_kci *kci)
{
	return kci->data;
}

/* Returns the element size according to @type. */
static inline u32 gcip_kci_queue_element_size(enum gcip_mailbox_queue_type type)
{
	if (type == GCIP_MAILBOX_CMD_QUEUE)
		return sizeof(struct gcip_kci_command_element);
	else
		return sizeof(struct gcip_kci_response_element);
}

static inline u64 gcip_kci_get_cur_seq(struct gcip_kci *kci)
{
	return gcip_mailbox_get_cur_seq(&kci->mailbox);
}

static inline struct gcip_kci_command_element *gcip_kci_get_cmd_queue(struct gcip_kci *kci)
{
	return (struct gcip_kci_command_element *)gcip_mailbox_get_cmd_queue(&kci->mailbox);
}

static inline struct gcip_kci_response_element *gcip_kci_get_resp_queue(struct gcip_kci *kci)
{
	return (struct gcip_kci_response_element *)gcip_mailbox_get_resp_queue(&kci->mailbox);
}

static inline u64 gcip_kci_get_queue_wrap_bit(struct gcip_kci *kci)
{
	return gcip_mailbox_get_queue_wrap_bit(&kci->mailbox);
}

static inline struct list_head *gcip_kci_get_wait_list(struct gcip_kci *kci)
{
	return gcip_mailbox_get_wait_list(&kci->mailbox);
}

static inline u32 gcip_kci_get_timeout(struct gcip_kci *kci)
{
	return gcip_mailbox_get_timeout(&kci->mailbox);
}

static inline unsigned long gcip_rkci_get_head(struct gcip_kci *kci)
{
	return kci->rkci.head;
}

static inline unsigned long gcip_rkci_get_tail(struct gcip_kci *kci)
{
	return kci->rkci.tail;
}

#endif /* __GCIP_KCI_H__ */
