/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020-2021, Samsung Electronics.
 *
 */

#ifndef __LINK_TX_PKTPROC_H__
#define __LINK_TX_PKTPROC_H__

#include <linux/skbuff.h>
#include "link_device_memory.h"
#if IS_ENABLED(CONFIG_EXYNOS_CPIF_IOMMU)
#include "cpif_vmapper.h"
#endif

/* Queue Numbers */
enum pktproc_ul_queue_t {
	PKTPROC_UL_HIPRIO = 0,
	PKTPROC_UL_QUEUE_0 = PKTPROC_UL_HIPRIO,
	PKTPROC_UL_NORM = 1,
	PKTPROC_UL_QUEUE_MAX
};

/*
 * Descriptor structure mode
 * 0: End bit is set by AP
 * 1: End bit is set by CP
 */
enum pktproc_end_bit_owner {
	END_BIT_AP,
	END_BIT_CP
};

/* Padding required by CP */
#define CP_PADDING		76
#define MAX_UL_PACKET_SIZE		512
#define HIPRIO_MAX_PACKET_SIZE	roundup_pow_of_two(MAX_UL_PACKET_SIZE + CP_PADDING)

/* Q_info */
struct pktproc_q_info_ul {
	u32 cp_desc_pbase;
	u32 num_desc;
	u32 cp_buff_pbase;
	u32 fore_ptr;
	u32 rear_ptr;
} __packed;

/* info for pktproc UL */
struct pktproc_info_ul {
	u32 num_queues:4, mode:4, max_packet_size:16, end_bit_owner:1, reserve1:7;
	u32 cp_quota:16, reserve2:16;
	struct pktproc_q_info_ul q_info[PKTPROC_UL_QUEUE_MAX];
} __packed;

struct pktproc_desc_ul {
	u32 data_size:20, reserve1:12;
	u32 total_pkt_size:20, reserve2:12;
	u64 sktbuf_point:36, reserve3:12, ap2cp_pbp_info:16;
	u32 last_desc:1, reserve4:31;
	u32 hw_set:1, seg_on:1, reserve5:2, segment:2, reserve6:2,
	    lcid:8, ap2cp_info_pp:16;
	u32 reserve7;
	u32 reserve8;
} __packed;

/* Statistics */
struct pktproc_statistics_ul {
	/* count of trials to write a packet to pktproc UL */
	u64 total_cnt;
	/* number of times failed to write a packet due to full buffer */
	u64 buff_full_cnt;
	/* number of times failed to write a packet due to inactive q */
	u64 inactive_cnt;
	/* number of times succeed to write a packet to pktproc UL */
	u64 pass_cnt;
};

/* Logical view for each queue */
struct pktproc_queue_ul {
	u32 q_idx;
	atomic_t active; /* activated when pktproc ul init */
	atomic_t busy; /* used for flow control */
	spinlock_t lock;

	struct mem_link_device *mld;
	struct pktproc_adaptor_ul *ppa_ul;

	u32 *fore_ptr; /* indicates the last last-bit raised desc pointer */
	u32 done_ptr; /* indicates the last packet written by AP */
	u32 *rear_ptr; /* indicates the last desc read by CP */

	/* Store */
	u64 cp_desc_pbase;
	u32 num_desc;
	u64 cp_buff_pbase;

	struct pktproc_info_ul *ul_info;
	struct pktproc_q_info_ul *q_info;	/* Pointer to q_info of info_v */
	struct pktproc_desc_ul *desc_ul;

	u32 desc_size;
	u64 buff_addr_cp; /* base data address value for cp */
	u32 max_packet_size;

	/* Pointer to data buffer */
	u8 __iomem *q_buff_vbase;
	u32 q_buff_size;

	/* Statistics */
	struct pktproc_statistics_ul stat;

	/* Func */
	int (*send_packet)(struct pktproc_queue_ul *q, struct sk_buff *new_skb);
	int (*update_fore_ptr)(struct pktproc_queue_ul *q, u32 count);
};

/* PktProc adaptor for UL*/
struct pktproc_adaptor_ul {
	bool support;	/* Is support PktProc feature? */

	unsigned long long cp_base;	/* CP base address for pktproc */
	unsigned long info_rgn_offset;	/* Offset of info region */
	unsigned long info_rgn_size;	/* Size of info region */
	unsigned long desc_rgn_offset;	/* Offset of descriptor region */
	unsigned long desc_rgn_size;	/* Size of descriptor region */
	unsigned long buff_rgn_offset;	/* Offset of data buffer region */
	unsigned long buff_rgn_size;	/* Size of data buffer region */

	u32 num_queue;		/* Number of queue */
	u32 default_max_packet_size;	/* packet size pktproc UL can hold */
	u32 hiprio_ack_only;
	enum pktproc_end_bit_owner end_bit_owner;	/* owner to set end bit. AP:0, CP:1 */
	u32 cp_quota;		/* max number of buffers cp allows us to transfer */
	bool use_hw_iocc;	/* H/W IO cache coherency */
	bool info_rgn_cached;
	bool desc_rgn_cached;
	bool buff_rgn_cached;
	bool padding_required;	/* requires extra length. (s5123 EVT1 only) */
#if IS_ENABLED(CONFIG_EXYNOS_CPIF_IOMMU)
	struct cpif_va_mapper *desc_map;
	struct cpif_va_mapper *buff_map;
#endif
	void __iomem *info_vbase;	/* I/O region for information */
	void __iomem *desc_vbase;	/* I/O region for descriptor */
	void __iomem *buff_vbase;	/* I/O region for data buffer */
	struct pktproc_queue_ul *q[PKTPROC_UL_QUEUE_MAX];/* Logical queue */
};

#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
extern int pktproc_create_ul(struct platform_device *pdev,
		struct mem_link_device *mld, unsigned long memaddr, u32 memsize);
extern int pktproc_init_ul(struct pktproc_adaptor_ul *ppa_ul);
static inline int pktproc_check_support_ul(struct pktproc_adaptor_ul *ppa_ul)
{
	return ppa_ul->support;
}
static inline int pktproc_check_ul_q_active(struct pktproc_adaptor_ul *ppa_ul,
		u32 q_idx)
{
	if (!pktproc_check_support_ul(ppa_ul))
		return 0;

	if (!ppa_ul->q[q_idx])
		return 0;

	return atomic_read(&ppa_ul->q[q_idx]->active);
}
static inline bool pktproc_ul_q_empty(struct pktproc_q_info_ul *q_info)
{
	return (q_info->fore_ptr == q_info->rear_ptr);
}
#else
static inline int pktproc_create_ul(struct platform_device *pdev,
		struct mem_link_device *mld,
		unsigned long memaddr, unsigned long memsize) { return 0; }
static inline int pktproc_init_ul(struct pktproc_adaptor_ul *ppa_ul) { return 0; }
static inline int pktproc_check_support_ul(struct pktproc_adaptor_ul *ppa_ul)
{ return 0; }
static inline int pktproc_check_ul_q_active(struct pktproc_adaptor_ul *ppa_ul,
		u32 q_idx) { return 0; }
static inline bool pktproc_ul_q_empty(struct pktproc_q_info_ul *q_info) { return 0; }
#endif

#endif /* __LINK_TX_PKTPROC_H__ */

