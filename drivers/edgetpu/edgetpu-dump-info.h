/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Structures used for debug dump segments.
 *
 * Copyright (C) 2022 Google LLC
 */

#ifndef __EDGETPU_DUMP_INFO_H__
#define __EDGETPU_DUMP_INFO_H__

/*
 * Note: A copy of this file is maintained in the debug dump parser project, do not include other
 * headers.
 */

/*
 * +------------+------------------+
 * | type ETDEV | edgetpu_dev_info |
 * +------------+------------------+
 */

struct edgetpu_dev_info {
	uint32_t state;
	uint32_t vcid_pool;
	uint32_t job_count;
	uint32_t firmware_crash_count;
	uint32_t watchdog_timeout_count;
	uint32_t reserved[11];
};

/*
 * +--------------+----------------------------+------------------------------+
 * | type CLIENTS | edgetpu_client_info_header | array of edgetpu_client_info |
 * +--------------+----------------------------+------------------------------+
 */

struct edgetpu_client_info {
	uint32_t pid;
	uint32_t tgid;
	uint64_t perdie_events;
	/* wakelock->req_count. ~0u if wakelock is unavailable. */
	uint32_t wakelock_req_count;
	/* workload_id of the group this client belongs to. ~0u if client->group is NULL. */
	uint32_t group_workload_id;
	uint32_t reserved[10];
};

struct edgetpu_client_info_header {
	uint32_t n_clients;
	struct edgetpu_client_info clients[];
};

/*
 * +-------------+---------------------------+-----------------------------+
 * | type GROUPS | edgetpu_group_info_header | array of edgetpu_group_info |
 * +-------------+---------------------------+-----------------------------+
 */

struct edgetpu_group_info {
	uint32_t workload_id;
	uint16_t vcid;
	uint8_t status;
	uint8_t queues_attached; /* whether has VII queues attached */
	uint32_t context_id;
	uint64_t size_host_mappings; /* total size of host mappings, in bytes */
	uint64_t size_dmabuf_mappings; /* total size of dmabuf mappings, in bytes */
	uint32_t reserved[9];
};

struct edgetpu_group_info_header {
	uint32_t n_groups;
	struct edgetpu_group_info groups[];
};

/*
 * +---------------+-----------------------------+-------------------------------+
 * | type MAPPINGS | edgetpu_mapping_info_header | array of edgetpu_mapping_info |
 * +---------------+-----------------------------+-------------------------------+
 */

#define MAPPING_TYPE_HOST 1
#define MAPPING_TYPE_DMABUF 2

struct edgetpu_mapping_info {
	uint64_t host_address;
	uint64_t device_address;
	uint64_t size;
	uint32_t flags;
	uint32_t dir;
	uint32_t reserved[8];
};

struct edgetpu_mapping_info_header {
	uint32_t n_mappings;
	uint32_t group_workload_id;
	uint8_t mapping_type;
	uint8_t padding[7];
	struct edgetpu_mapping_info mappings[];
};

#endif /* __EDGETPU_DUMP_INFO_H__ */
