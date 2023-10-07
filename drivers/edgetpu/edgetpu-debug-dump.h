/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Module that defines structures and functions to retrieve debug dump segments
 * from edgetpu firmware.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_DEBUG_DUMP_H__
#define __EDGETPU_DEBUG_DUMP_H__

#include "edgetpu-internal.h"

#define DEBUG_DUMP_HOST_CONTRACT_VERSION 3

enum edgetpu_dump_type_bit_position {
	DUMP_TYPE_CRASH_REASON_BIT = 0,
	DUMP_TYPE_STATS_BIT = 1,
	DUMP_TYPE_TCM_BIT = 2,
	DUMP_TYPE_SRAM_BIT = 3,
	DUMP_TYPE_CPU_BIT = 4,
	DUMP_TYPE_CSRS_BIT = 5,

	DUMP_TYPE_KERNEL_ETDEV_BIT = 32,
	DUMP_TYPE_KERNEL_CLIENTS_BIT = 33,
	DUMP_TYPE_KERNEL_GROUPS_BIT = 34,
	DUMP_TYPE_KERNEL_MAPPINGS_BIT = 35,

	DUMP_TYPE_MAX_BIT = 63
};

enum edgetpu_dump_reason {
	DUMP_REASON_DEFAULT = 0,
	/* Host request reasons */
	DUMP_REASON_REQ_BY_USER = 1,

	/* FW side dump reasons */
	DUMP_REASON_FW_CHECKPOINT = 2,
	DUMP_REASON_RECOVERABLE_FAULT = 3,

	DUMP_REASON_NUM = 4
};

struct edgetpu_crash_reason {
	u64 code;	/* code that captures the reset reason */
};

struct edgetpu_debug_stats {
	u64 num_requests;	/* number of dump requests made to the tpu */
	u64 uptime;	/* time since boot up on the tpu */
	u64 current_context;	/* current task context */
};

struct edgetpu_dump_segment {
	u64 type;	/* type of the dump */
	u64 size;	/* size of the dump data */
	u64 src_addr; /* source of the dump on the CPU address map */
};

struct edgetpu_debug_dump {
	u64 magic;	/* word identifying the beginning of the dump info */
	u64 version;	/* host-firmware dump info contract version */
	u64 host_dump_available_to_read;	/* is new info available */
	u64 dump_reason;	/* Reason or context for debug dump */
	u64 reserved[2];
	u64 crash_reason_offset;	/* byte offset to crash reason */
	u64 crash_reason_size;	/* crash reason size */
	u64 debug_stats_offset;	/* byte offset to debug stats */
	u64 debug_stats_size;	/* crash reason size */
	u64 dump_segments_offset;	/* byte offset to dump segments */
	u64 dump_segments_num;	/* number of dump segments populated */
};

struct edgetpu_debug_dump_setup {
	/* types of dumps requested by host */
	u64 type;
	u64 dump_mem_size;	/* total size of memory allocated to dump */
	u64 reserved[2];
};

/*
 * Allocate and initialize debug dump memory.
 */
int edgetpu_debug_dump_init(struct edgetpu_dev *etdev);

/*
 * Free debug dump memory.
 */
void edgetpu_debug_dump_exit(struct edgetpu_dev *etdev);

/*
 * Send KCI request to get fw debug dump segments.
 *
 * This function can be called with @type set to 0 to simply set the dump buffer address and size
 * in the FW without dumping any segments.
 *
 * The caller must ensure that the device is powered on.
 */
int edgetpu_get_debug_dump(struct edgetpu_dev *etdev,
			   u64 type);

/*
 * KCI interrupt handler to handle any debug information dumped by firmware.
 */
void edgetpu_debug_dump_resp_handler(struct edgetpu_dev *etdev);

#endif /* EDGETPU_DEBUG_DUMP_H_ */
