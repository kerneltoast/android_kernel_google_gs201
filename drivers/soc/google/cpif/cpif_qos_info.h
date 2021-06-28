/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019, Samsung Electronics.
 *
 */

#ifndef __CPIF_QOS_INFO_H__
#define __CPIF_QOS_INFO_H__

#include <linux/types.h>
#include <linux/hashtable.h>

struct hiprio_uid_list {
	DECLARE_HASHTABLE(uid_map, 9);
};

struct hiprio_uid {
	u32 uid;
	struct hlist_node h_node;
};

int cpif_qos_init_list(void);
struct hiprio_uid *cpif_qos_get_node(u32 uid);

#endif /* __CPIF_QOS_INFO_H__ */
