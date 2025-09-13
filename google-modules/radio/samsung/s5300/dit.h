/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS DIT(Direct IP Translator) Driver support
 *
 */

#ifndef __DIT_H__
#define __DIT_H__

#include "modem_utils.h"
#include "modem_toe_device.h"

enum dit_direction {
	DIT_DIR_TX,
	DIT_DIR_RX,
	DIT_DIR_MAX
};

enum dit_init_type {
	DIT_INIT_NORMAL = 0,
	DIT_INIT_RETRY,
	DIT_INIT_DEINIT,
};

enum dit_store_type {
	DIT_STORE_NONE = 0,
	DIT_STORE_BACKUP,
	DIT_STORE_RESTORE,
};

int dit_init(struct link_device *ld, enum dit_init_type type, enum dit_store_type store);
int dit_get_irq_affinity(void);
int dit_set_irq_affinity(int affinity);
int dit_set_pktproc_queue_num(enum dit_direction dir, u32 queue_num);
int dit_set_buf_size(enum dit_direction dir, u32 size);
int dit_set_pktproc_base(enum dit_direction dir, phys_addr_t base);
int dit_set_desc_ring_len(enum dit_direction dir, u32 len);
int dit_get_src_usage(enum dit_direction dir, u32 *usage);
extern u32 gs_chipid_get_type(void);

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
int dit_enqueue_src_desc_ring(
	enum dit_direction dir, u8 *src, unsigned long src_paddr,
	u16 len, u8 ch_id, bool csum);
int dit_enqueue_src_desc_ring_skb(enum dit_direction dir, struct sk_buff *skb);
int dit_kick(enum dit_direction dir, bool retry);
bool dit_check_dir_use_queue(enum dit_direction dir, unsigned int queue_num);
int dit_reset_dst_wp_rp(enum dit_direction dir);
struct net_device *dit_get_netdev(void);
bool dit_support_clat(void);
bool dit_hal_set_clat_info(struct mem_link_device *mld, struct clat_info *clat);
#else
static inline int dit_enqueue_src_desc_ring(
	enum dit_direction dir, u8 *src, unsigned long src_paddr,
	u16 len, u8 ch_id, bool csum) { return -1; }
static inline int dit_enqueue_src_desc_ring_skb(
	enum dit_direction dir, struct sk_buff *skb) { return -1; }
static inline int dit_kick(enum dit_direction dir, bool retry) { return -1; }
static inline bool dit_check_dir_use_queue(
	enum dit_direction dir, unsigned int queue_num) { return false; }
static inline int dit_reset_dst_wp_rp(enum dit_direction dir) { return -1; }
static inline struct net_device *dit_get_netdev(void) { return NULL; }
static inline bool dit_support_clat(void) { return false; }
static inline bool dit_hal_set_clat_info(struct mem_link_device *mld, struct clat_info *clat)
{ return false; }
#endif

#endif /* __DIT_H__ */

