/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __BOOT_H__
#define __BOOT_H__

#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/kfifo.h>
#include <linux/netdevice.h>

#include "circ_queue.h"
#include "sipc5.h"

#define BAD_MSG_BUFFER_SIZE 32

enum legacy_ipc_map {
	IPC_MAP_FMT = 0,
#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
	IPC_MAP_HPRIO_RAW,
#endif
	IPC_MAP_NORM_RAW,
	IPC_MAP_MAX,
};

struct legacy_ipc_device {
	enum legacy_ipc_map id;
	char name[16];

	struct circ_queue txq;
	struct circ_queue rxq;

	u16 msg_mask;
	u16 req_ack_mask;
	u16 res_ack_mask;

	struct sk_buff_head *skb_txq;
	struct sk_buff_head *skb_rxq;

	unsigned int req_ack_cnt[MAX_DIR];

	spinlock_t tx_lock;
};

struct legacy_link_device {

	struct link_device *ld;

	atomic_t active;

	u32 __iomem *magic;
	u32 __iomem *mem_access;

	struct legacy_ipc_device *dev[IPC_MAP_MAX];
};

int create_legacy_link_device(struct mem_link_device *mld);
int init_legacy_link(struct legacy_link_device *bl);
int xmit_to_legacy_link(struct mem_link_device *mld, u8 ch,
		struct sk_buff *skb, enum legacy_ipc_map legacy_buffer_index);
struct sk_buff *recv_from_legacy_link(struct mem_link_device *mld,
		struct legacy_ipc_device *dev, unsigned int in, int *ret);
bool check_legacy_tx_pending(struct mem_link_device *mld);

#endif /* end of __BOOT_H__ */
