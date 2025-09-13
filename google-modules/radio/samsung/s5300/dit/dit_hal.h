/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS DIT(Direct IP Translator) Driver support
 *
 */

#ifndef __DIT_HAL_H__
#define __DIT_HAL_H__

#include "dit_common.h"

#define DIT_HAL_DEV_NAME	"dit2"

struct iface_info {
	char iface[IFNAMSIZ];
	u16 dst_ring;
} __packed;

struct forward_stats {
	char iface[IFNAMSIZ];
	u64 data_limit;
	u64 rx_bytes;
	u64 tx_bytes;
	u64 rx_diff;
	u64 tx_diff;
} __packed;

/* For tetheroffload hal service V1.1 */
struct forward_limit {
	char iface[IFNAMSIZ];
	u64 data_warning;
	u64 data_limit;
} __packed;

struct nat_local_addr {
	u16 index;
	u8 dst_ring;
	__be32 addr;
	union {
		u8 dev_addr[ETH_ALEN];
		struct {
			u32 dev_addr_l;
			u16 dev_addr_h;
		};
	};
} __packed;

struct nat_local_port {
	u16 reply_port_dst_l;	/* an index of table */
	union {
		struct {
			u32	enable:1,
				reply_port_dst_h:8,
				origin_port_src:16,
				addr_index:4,
				dst_ring:2,
				is_udp:1;
		};
		u32	hw_val;
	};
} __packed;

struct hw_info {
	u32 version;
	u32 capabilities;
} __packed;

#define OFFLOAD_IOC_MAGIC	('D')

#define OFFLOAD_IOCTL_INIT_OFFLOAD		_IO(OFFLOAD_IOC_MAGIC, 0x00)
#define OFFLOAD_IOCTL_STOP_OFFLOAD		_IO(OFFLOAD_IOC_MAGIC, 0x01)
#define OFFLOAD_IOCTL_SET_LOCAL_PRFIX		_IO(OFFLOAD_IOC_MAGIC, 0x02)
#define OFFLOAD_IOCTL_GET_FORWD_STATS		_IOWR(OFFLOAD_IOC_MAGIC, 0x03, struct forward_stats)
#define OFFLOAD_IOCTL_SET_DATA_LIMIT		_IOW(OFFLOAD_IOC_MAGIC, 0x04, struct forward_stats)
#define OFFLOAD_IOCTL_SET_UPSTRM_PARAM		_IOW(OFFLOAD_IOC_MAGIC, 0x05, struct iface_info)
#define OFFLOAD_IOCTL_ADD_DOWNSTREAM		_IOWR(OFFLOAD_IOC_MAGIC, 0x06, struct iface_info)
#define OFFLOAD_IOCTL_REMOVE_DOWNSTRM		_IOW(OFFLOAD_IOC_MAGIC, 0x07, struct iface_info)
#define OFFLOAD_IOCTL_SET_DATA_WARNING_LIMIT	_IOW(OFFLOAD_IOC_MAGIC, 0x08, struct forward_limit)

#define OFFLOAD_IOCTL_SET_NAT_LOCAL_ADDR	_IOW(OFFLOAD_IOC_MAGIC, 0x20, struct nat_local_addr)
#define OFFLOAD_IOCTL_SET_NAT_LOCAL_PORT	_IOW(OFFLOAD_IOC_MAGIC, 0x21, struct nat_local_port)

/* mandatory */
#define OFFLOAD_IOCTL_GET_HW_INFO		_IOR(OFFLOAD_IOC_MAGIC, 0xE0, struct hw_info)

enum offload_event_num {
	OFFLOAD_STARTED			= 1,
	OFFLOAD_STOPPED_ERROR		= 2,
	OFFLOAD_STOPPED_UNSUPPORTED	= 3,
	OFFLOAD_SUPPORT_AVAILABLE	= 4,
	OFFLOAD_STOPPED_LIMIT_REACHED	= 5,
	OFFLOAD_WARNING_REACHED		= 6,

	/* OEM defined event */
	INTERNAL_OFFLOAD_STOPPED	= 5000,
	OFFLOAD_MAX			= S32_MAX,
};

struct offload_event {
	s32 event_num;
} __packed;

struct offload_event_item {
	struct list_head list;
	enum offload_event_num event_num;
};

struct dev_addr_map {
	u32 dev_addr_l;
	u16 dev_addr_h;
} __packed;

struct dit_hal_dst_iface {
	bool iface_set;
	char iface[IFNAMSIZ];
	struct net_device *netdev;
};

struct dit_hal_stats {
	char iface[IFNAMSIZ];
	u64 data_warning;
	u64 data_limit;
	u64 rx_bytes;
	u64 tx_bytes;
	u64 rx_diff;
	u64 tx_diff;
};

struct dit_hal_ctrl_t {
	bool hal_enabled;
	spinlock_t hal_lock;

	struct dit_hal_dst_iface dst_iface[DIT_DST_DESC_RING_MAX];

	struct dit_hal_stats stats;
	spinlock_t stats_lock;

	enum offload_event_num last_event_num;
	wait_queue_head_t wq;
	struct list_head event_q;
	spinlock_t event_lock;
	struct mutex ioctl_lock;
};

int dit_hal_create(struct dit_ctrl_t *dc_ptr);
struct net_device *dit_hal_get_dst_netdev(enum dit_desc_ring ring_num);
void dit_hal_add_data_bytes(u64 rx_bytes, u64 tx_bytes);

#endif /* __DIT_HAL_H__ */

