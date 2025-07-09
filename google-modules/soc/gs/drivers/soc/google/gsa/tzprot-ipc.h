/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2024 Google LLC
 */
#ifndef __LINUX_TZPROT_IPC_H
#define __LINUX_TZPROT_IPC_H

#include <linux/types.h>

#define TZPROT_PORT "com.android.trusty.media_prot"

#define HISTOGRAM_BIN_SIZE (0x80)

enum media_prot_cmd {
	MEDIA_PROT_CMD_RESP = (1U << 31),
	MEDIA_PROT_CMD_SET_IP_PROT = 0,
	MEDIA_PROT_CMD_GET_HISTOGRAM,
};

struct media_prot_set_ip_prot_req {
	u32 dev_id;
	u32 enable;
};

struct media_prot_get_histogram_req {
	u16 dqe_channel;
};

struct media_prot_req {
	u32 cmd;
	union {
		struct media_prot_set_ip_prot_req set_ip_prot_req;
		struct media_prot_get_histogram_req get_histogram_req;
	};
};

struct media_prot_get_histogram_rsp {
	u32 bin[HISTOGRAM_BIN_SIZE];
};

struct media_prot_rsp {
	uint32_t cmd;
	int32_t err;
	union {
		struct media_prot_get_histogram_rsp get_histogram_rsp;
	};
};

#endif

