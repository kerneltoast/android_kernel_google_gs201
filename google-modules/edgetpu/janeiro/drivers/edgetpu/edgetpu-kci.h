/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Kernel Control Interface, implements the protocol between AP kernel and TPU
 * firmware.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_KCI_H__
#define __EDGETPU_KCI_H__

#include <linux/dma-direction.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "edgetpu-firmware.h"
#include "edgetpu-internal.h"
#include "edgetpu-mailbox.h"

/*
 * Maximum number of outstanding KCI requests from firmware
 * This is used to size a circular buffer, so it must be a power of 2
 */
#define REVERSE_KCI_BUFFER_SIZE		(32)

/*
 * The status field in a firmware response is set to this by us when the
 * response is fetched from the queue.
 */
#define KCI_STATUS_OK			(0)
/*
 * edgetpu_kci#wait_list uses this value to record the status of responses
 * that haven't been received yet.
 */
#define KCI_STATUS_WAITING_RESPONSE	(1)
/*
 * Used when an expected response is not received, see the documentation of
 * edgetpu_kci_consume_wait_list() for details.
 *
 */
#define KCI_STATUS_NO_RESPONSE		(2)


/*
 * Command/response sequence numbers capped at half the range of the 64-bit
 * value range. The second half is reserved for incoming requests from firmware.
 * These are tagged with the MSB set.
 */
#define KCI_REVERSE_FLAG		(0x8000000000000000ull)

/* command/response queue elements for KCI */

struct edgetpu_dma_descriptor {
	u64 address;
	u32 size;
	u32 flags;
};

struct edgetpu_command_element {
	/*
	 * Set by edgetpu_kci_push_cmd() in case of KCI cmd and copied from
	 * the RKCI cmd in case of RKCI response.
	 */
	u64 seq;
	u16 code;
	u16 reserved[3]; /* explicit padding, does not affect alignment */
	struct edgetpu_dma_descriptor dma;
};

struct edgetpu_kci_response_element {
	u64 seq;
	u16 code;
	/*
	 * Reserved for host use - firmware can't touch this.
	 * If a value is written here it will be discarded and overwritten
	 * during response processing. However, when repurposed as an RKCI
	 * command, the FW can set this field.
	 */
	u16 status;
	/*
	 * Return value is not currently needed by KCI command responses.
	 * For reverse KCI commands this is set as value2.
	 */
	u32 retval;
} __packed;

/* VII response element */
/* The size of this structure must match the runtime definition. */
struct edgetpu_vii_response_element {
	u64 seq;
	u16 code;
	u8 reserved[6];	/* padding */
	u64 retval;
} __packed;

/*
 * Definition of code in command elements.
 * Code for KCI is a 16-bit unsigned integer.
 */
enum edgetpu_kci_code {
	KCI_CODE_ACK = 0,
	KCI_CODE_UNMAP_BUFFER = 1,
	KCI_CODE_MAP_LOG_BUFFER = 2,
	KCI_CODE_JOIN_GROUP = 3,
	KCI_CODE_LEAVE_GROUP = 4,
	KCI_CODE_MAP_TRACE_BUFFER = 5,
	KCI_CODE_SHUTDOWN = 7,
	KCI_CODE_GET_DEBUG_DUMP = 8,
	KCI_CODE_OPEN_DEVICE = 9,
	KCI_CODE_CLOSE_DEVICE = 10,
	KCI_CODE_FIRMWARE_INFO = 11,
	/* TODO(b/271372136): remove v1 when v1 firmware no longer in use. */
	KCI_CODE_GET_USAGE_V1 = 12,
	KCI_CODE_NOTIFY_THROTTLING = 13,
	KCI_CODE_BLOCK_BUS_SPEED_CONTROL = 14,
	/* 15..18 not implemented in this branch */
	KCI_CODE_FIRMWARE_TRACING_LEVEL = 19,
	/* 20 not implemented in this branch */
	KCI_CODE_GET_USAGE_V2 = 21,

	KCI_CODE_RKCI_ACK = 256,
};

/*
 * Definition of reverse KCI request code ranges
 * 16-bit unsigned integer
 * First half is reserved for chip specific codes,
 * Generic codes can use the second half.
 */
enum edgetpu_reverse_kci_code {
	RKCI_CHIP_CODE_FIRST = 0,
	RKCI_CHIP_CODE_LAST = 0x7FFF,
	RKCI_GENERIC_CODE_FIRST = 0x8000,
	RKCI_FIRMWARE_CRASH = RKCI_GENERIC_CODE_FIRST + 0,
	RKCI_JOB_LOCKUP = RKCI_GENERIC_CODE_FIRST + 1,
	RKCI_GENERIC_CODE_LAST = 0xFFFF,
};

/*
 * Definition of code in response elements.
 * It is a 16-bit unsigned integer.
 */
enum edgetpu_kci_error {
	KCI_ERROR_OK = 0, /* Not an error; returned on success */
	KCI_ERROR_CANCELLED = 1,
	KCI_ERROR_UNKNOWN = 2,
	KCI_ERROR_INVALID_ARGUMENT = 3,
	KCI_ERROR_DEADLINE_EXCEEDED = 4,
	KCI_ERROR_NOT_FOUND = 5,
	KCI_ERROR_ALREADY_EXISTS = 6,
	KCI_ERROR_PERMISSION_DENIED = 7,
	KCI_ERROR_RESOURCE_EXHAUSTED = 8,
	KCI_ERROR_FAILED_PRECONDITION = 9,
	KCI_ERROR_ABORTED = 10,
	KCI_ERROR_OUT_OF_RANGE = 11,
	KCI_ERROR_UNIMPLEMENTED = 12,
	KCI_ERROR_INTERNAL = 13,
	KCI_ERROR_UNAVAILABLE = 14,
	KCI_ERROR_DATA_LOSS = 15,
	KCI_ERROR_UNAUTHENTICATED = 16,
};

struct edgetpu_kci_wait_list {
	struct list_head list;
	/*
	 * Its content will be updated once a response with sequence number
	 * equal to resp->seq is received.
	 */
	struct edgetpu_kci_response_element *resp;
};

/* Struct to hold a circular buffer for incoming KCI responses */
struct edgetpu_reverse_kci {
	unsigned long head;
	unsigned long tail;
	struct edgetpu_kci_response_element buffer[REVERSE_KCI_BUFFER_SIZE];
	/* Lock to push elements in the buffer from the interrupt handler */
	spinlock_t producer_lock;
	/* Lock to pop elements from the buffer in the worker */
	spinlock_t consumer_lock;
	/* Worker to handle responses */
	struct work_struct work;
};

struct edgetpu_kci {
	struct edgetpu_mailbox *mailbox;
	struct mutex mailbox_lock;	/* protects mailbox */
	u64 cur_seq;
	struct edgetpu_command_element *cmd_queue;
	struct mutex cmd_queue_lock;	/* protects cmd_queue */
	/* Command queue buffer */
	struct edgetpu_coherent_mem cmd_queue_mem;
	struct edgetpu_kci_response_element *resp_queue;
	spinlock_t resp_queue_lock;	/* protects resp_queue */
	/* Response queue buffer */
	struct edgetpu_coherent_mem resp_queue_mem;
	/* queue for waiting for the response doorbell to be rung */
	wait_queue_head_t resp_doorbell_waitq;
	/* add to this list if a command needs to wait for a response */
	struct list_head wait_list;
	spinlock_t wait_list_lock;	/* protects wait_list */
	/* queue for waiting for the wait_list to be consumed */
	wait_queue_head_t wait_list_waitq;
	struct work_struct work;	/* worker of consuming responses */
	/* Handler for reverse (firmware -> kernel) requests */
	struct edgetpu_reverse_kci rkci;
	struct work_struct usage_work;	/* worker that sends update usage KCI */
};

struct edgetpu_kci_device_group_detail {
	u8 n_dies;
	/* virtual ID from 0 ~ n_dies - 1 */
	/* ID 0 for the group master */
	u8 vid;
	u8 reserved[6]; /* padding */
};

struct edgetpu_kci_open_device_detail {
	/* The client privilege level. */
	u16 client_priv;
	/*
	 * Virtual context ID @mailbox_id is associated to.
	 * For device groups with @mailbox_detachable attribute the mailbox attached to the group
	 * can be different after wakelock re-acquired. Firmware uses this VCID to identify the
	 * device group.
	 */
	u16 vcid;
	/*
	 * Extra flags for the attributes of this request.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [0:0]   - first_open: Specifies if this is the first time we are calling mailbox open
	 *             KCI for this VCID after it has been allocated to a device group. This allows
	 *             firmware to clean up/reset the memory allocator for that partition.
	 *   [31:1]  - RESERVED
	 */
	u32 flags;
};

/*
 * Initializes a KCI object.
 *
 * Will request a mailbox from @mgr and allocate cmd/resp queues.
 */
int edgetpu_kci_init(struct edgetpu_mailbox_manager *mgr,
		     struct edgetpu_kci *kci);
/*
 * Re-initializes the initialized KCI object.
 *
 * This function is used when the TPU device is reset, it re-programs CSRs
 * related to KCI mailbox.
 *
 * Returns 0 on success, -errno on error.
 */
int edgetpu_kci_reinit(struct edgetpu_kci *kci);
/*
 * Releases resources allocated by @kci.
 *
 * Note: must invoke this function after the interrupt of mailbox disabled and
 * before free the mailbox pointer.
 */
void edgetpu_kci_release(struct edgetpu_dev *etdev, struct edgetpu_kci *kci);

/*
 * Pushes an element to cmd queue.
 *
 * @cmd's seq field will be set.
 * Will update the CMD_QUEUE_TAIL CSR.
 *
 * @resp will NOT be updated immediately, instead, it will be appended to the
 * wait_list of @kci. Once the response of @cmd is received, @resp will be
 * updated. Compare the value of resp->code with KCI_CODE_WAITING_RESPONSE to
 * check if the response is received.
 * @resp can be NULL if the command doesn't need a response.
 *
 * This is a synchronous function. If the cmd queue is full, it will wait until
 * the queue is consumed.
 *
 * Returns 0 on success, or a negative errno on error.
 */
int edgetpu_kci_push_cmd(struct edgetpu_kci *kci,
			 struct edgetpu_command_element *cmd,
			 struct edgetpu_kci_response_element *resp);

/*
 * Pushes an element to cmd queue and waits for the response.
 * Returns -ETIMEDOUT if no response is received within KCI_TIMEOUT.
 *
 * Returns the code of response, or a negative errno on error.
 */
int edgetpu_kci_send_cmd(struct edgetpu_kci *kci,
			 struct edgetpu_command_element *cmd);

/*
 * Sends a FIRMWARE_INFO command and expects a response with a
 * edgetpu_fw_info struct filled out, including what firmware type is running,
 * along with build CL and time.
 * Also serves as an initial handshake with firmware at load time.
 *
 * @fw_info: a struct edgetpu_fw_info to be filled out by fw
 *
 * Returns >=0 edgetpu_fw_flavor when response received from firmware,
 *         <0 on error communicating with firmware (typically -ETIMEDOUT).
 */
enum edgetpu_fw_flavor edgetpu_kci_fw_info(
	struct edgetpu_kci *kci, struct edgetpu_fw_info *fw_info);

/*
 * Schedules a worker to call edgetpu_kci_update_usage().
 *
 * For functions that don't require the usage to be updated immediately, use
 * this function instead of edgetpu_kci_update_usage().
 */
void edgetpu_kci_update_usage_async(struct edgetpu_dev *etdev);

/*
 * Retrieves usage tracking data from firmware, update info on host.
 * Also used as a watchdog ping to firmware.
 *
 * Returns KCI response code on success or < 0 on error (typically -ETIMEDOUT).
 */
int edgetpu_kci_update_usage(struct edgetpu_dev *etdev);

/*
 * Works the same as edgetpu_kci_update_usage() except the caller of this
 * function must guarantee the device stays powered up, typically by calling
 * edgetpu_pm_get() or by calling this function from the power management
 * functions themselves.
 *
 * Returns KCI response code on success or < 0 on error (typically -ETIMEDOUT).
 */
int edgetpu_kci_update_usage_locked(struct edgetpu_dev *etdev);

/*
 * Sends the "Map Log Buffer" command and waits for remote response.
 *
 * Returns the code of response, or a negative errno on error.
 */
int edgetpu_kci_map_log_buffer(struct edgetpu_kci *kci, tpu_addr_t tpu_addr,
			       u32 size);

/*
 * Sends the "Map Trace Buffer" command and waits for remote response.
 *
 * Returns the code of response, or a negative errno on error.
 */
int edgetpu_kci_map_trace_buffer(struct edgetpu_kci *kci, tpu_addr_t tpu_addr,
				 u32 size);

/*
 * Sent when a group is created with @n_dies dies, and @etdev is the @vid-th
 * die in this group.
 *
 * Returns the code of response, or a negative errno on error.
 */
int edgetpu_kci_join_group(struct edgetpu_kci *kci, u8 n_dies, u8 vid);
/* Informs the TPU to leave the group it currently belongs to. */
int edgetpu_kci_leave_group(struct edgetpu_kci *kci);

/* debugfs mappings dump */
void edgetpu_kci_mappings_show(struct edgetpu_dev *etdev, struct seq_file *s);

/* Send shutdown request to firmware */
int edgetpu_kci_shutdown(struct edgetpu_kci *kci);

/*
 * Inform the firmware to prepare to serve VII mailboxes included in @mailbox_map.
 *
 * You usually shouldn't call this directly - consider using
 * edgetpu_mailbox_activate() or edgetpu_mailbox_activate_bulk() instead.
 */
int edgetpu_kci_open_device(struct edgetpu_kci *kci, u32 mailbox_map, u32 client_priv, s16 vcid,
			    bool first_open);

/*
 * Inform the firmware that the VII mailboxes included in @mailbox_map are closed.
 *
 * You usually shouldn't call this directly - consider using
 * edgetpu_mailbox_deactivate() or edgetpu_mailbox_deactivate_bulk() instead.
 */
int edgetpu_kci_close_device(struct edgetpu_kci *kci, u32 mailbox_map);

/* Cancel work queues or wait until they're done */
void edgetpu_kci_cancel_work_queues(struct edgetpu_kci *kci);

/*
 * Notify the firmware about throttling and the corresponding power level.
 * The request is sent only if the device is already powered on.
 *
 * Returns KCI response code on success or < 0 on error (typically -ETIMEDOUT).
 */
int edgetpu_kci_notify_throttling(struct edgetpu_dev *etdev, u32 level);

/*
 * Request the firmware to {un}block modulating bus clock speeds
 *
 * Used to prevent conflicts when sending a thermal policy request
 */
int edgetpu_kci_block_bus_speed_control(struct edgetpu_dev *etdev, bool block);

/* Set the firmware tracing level. */
int edgetpu_kci_firmware_tracing_level(struct edgetpu_dev *etdev, unsigned long level,
				       unsigned long *active_level);

/*
 * Send an ack to the FW after handling a reverse KCI request.
 *
 * The FW may wait for a response from the kernel for an RKCI request so a
 * response could be sent as an ack.
 */
int edgetpu_kci_resp_rkci_ack(struct edgetpu_dev *etdev,
			      struct edgetpu_kci_response_element *rkci_cmd);


#endif /* __EDGETPU_KCI_H__ */
