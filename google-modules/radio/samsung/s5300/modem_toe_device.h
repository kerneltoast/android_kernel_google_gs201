/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * MODEM TOE device support
 *
 */

#ifndef __MODEM_TOE_DEVICE_H__
#define __MODEM_TOE_DEVICE_H__

#include <linux/in.h>
#include <linux/in6.h>
#include <linux/inet.h>

#include "modem_utils.h"

#define IOCTL_TOE_MAGIC			('T')
#define IOCTL_TOE_SET_CLAT_READY	_IOW(IOCTL_TOE_MAGIC, 0x00, uint32_t)
#define IOCTL_TOE_SET_CLAT_IFACES_NUM	_IOW(IOCTL_TOE_MAGIC, 0x01, uint32_t)
#define IOCTL_TOE_SET_CLAT_INFO		_IOW(IOCTL_TOE_MAGIC, 0x02, struct clat_info)

struct clat_info {
	u32 clat_index;
	char ipv6_iface[IFNAMSIZ];
	char ipv4_iface[IFNAMSIZ];
	struct in6_addr ipv6_local_subnet;
	struct in_addr ipv4_local_subnet;
	struct in6_addr plat_subnet;
} __packed;

struct toe_ctrl_t {
	bool clat_hal_ready;
	u32 clat_ifaces_num;
	bool clat_dev_support;
	struct mem_link_device *mld;

	bool (*set_clat_info)(struct mem_link_device *mld, struct clat_info *clat);
	void (*set_iod_clat_netdev)(struct io_device *iod, void *args);
};

void toe_set_iod_clat_netdev(struct io_device *iod, void *args);
int toe_dev_init(struct mem_link_device *mld);
int toe_dev_create(struct platform_device *pdev);

#endif /* __MODEM_TOE_DEVICE_H__ */
