/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018-2019, Samsung Electronics.
 *
 */

#ifndef __LINK_RX_PKTPROC_H__
#define __LINK_RX_PKTPROC_H__

/* Debug */
/* #define PKTPROC_DEBUG */
/* #define PKTPROC_DEBUG_PKT */
#ifdef PKTPROC_DEBUG
#define pp_debug(fmt, ...) mif_info(fmt, ##__VA_ARGS__)
#else
#define pp_debug(fmt, ...) no_printk(fmt, ##__VA_ARGS__)
#endif

/* Numbers */
#define PKTPROC_MAX_QUEUE	4

/* Status bit field */
#define PKTPROC_STATUS_DONE	0x01
#define PKTPROC_STATUS_TCPCF	0x04
#define PKTPROC_STATUS_IPCSF	0x08
#define PKTPROC_STATUS_IGNR	0x10
#define PKTPROC_STATUS_TCPC	0x20
#define PKTPROC_STATUS_IPCS	0x40
#define PKTPROC_STATUS_PFD	0x80

/*
 * PktProc info region
 */
/* Queue info */
struct pktproc_q_info {
	u32 cp_desc_pbase;
	u32 num_desc;
	u32 cp_buff_pbase;
	u32 fore_ptr;
	u32 rear_ptr;
} __packed;

/* Info for V1 */
struct pktproc_info_v1 {
	struct pktproc_q_info q_info;
} __packed;

/* Info for V2 */
struct pktproc_info_v2 {
	u32 num_queues:4,
		desc_mode:2,
		irq_mode:2,
		max_packet_size:16,
		reserved:8;
	struct pktproc_q_info q_info[PKTPROC_MAX_QUEUE];
} __packed;

/*
 * PktProc descriptor region
 */
/* RingBuf mode */
struct pktproc_desc_ringbuf {
	u32 cp_data_paddr;
	u32 information;
	u32 reserve1;
	u16 reserve2;
	u16 filter_result;
	u16 length;
	u8 channel_id;
	u8 reserve3;
	u8 status;
	u8 reserve4;
	u16 reserve5;
} __packed;

/* SktBuf mode */
struct pktproc_desc_sktbuf {
	u64 cp_data_paddr:36,
		reserved0:4,
		control:8,
		status:8,
		lro:5,
		reserved1:3;
	u16 length;
	u16 filter_result;
	u16 information;
	u8 channel_id;
	u8 reserved2:4,
		itg:2,
		reserved3:2;
} __packed;

/* Statistics */
struct pktproc_statistics {
	u64 pass_cnt;
	u64 err_len;
	u64 err_chid;
	u64 err_addr;
	u64 err_nomem;
	u64 err_bm_nomem;
	u64 err_csum;
	u64 use_memcpy_cnt;
	u64 err_enqueue_dit;
};

/* Logical view for each queue */
struct pktproc_queue {
	u32 q_idx;
	atomic_t active;
	spinlock_t lock;

	struct mem_link_device *mld;
	struct pktproc_adaptor *ppa;

	/* Pointer to fore_ptr of q_info. Increased AP when desc_mode is ringbuf mode */
	u32 *fore_ptr;
	/* Pointer to rear_ptr of q_info. Increased CP when desc_mode is sktbuf mode */
	u32 *rear_ptr;
	/* Follow rear_ptr when desc_mode is sktbuf mode */
	u32 done_ptr;

	/* Store */
	u32 cp_desc_pbase;
	u32 num_desc;
	u32 cp_buff_pbase;

	/* Pointer to info region by version */
	union {
		struct {
			struct pktproc_info_v1 *info_v1;
		};
		struct {
			struct pktproc_info_v2 *info_v2;
		};
	};
	struct pktproc_q_info *q_info_ptr;	/* Pointer to q_info of info_v */

	/* Pointer to desc region by addr mode */
	union {
		struct {
			struct pktproc_desc_ringbuf *desc_ringbuf;	/* RingBuf mode */
		};
		struct {
			struct pktproc_desc_sktbuf *desc_sktbuf;	/* SktBuf mode */
		};
	};
	u32 desc_size;

	/* Pointer to data buffer for a queue */
	u8 __iomem *q_buff_vbase;
	unsigned long q_buff_pbase;
	u32 q_buff_size;

	/* Buffer manager */
	struct mif_buff_mng *manager;	/* Pointer to buffer manager */
	dma_addr_t *dma_addr;
	bool use_memcpy;	/* memcpy mode on sktbuf mode */

	/* IRQ */
	int irq;
	u32 irq_idx;

	/* NAPI */
	struct net_device netdev;
	struct napi_struct napi;
	struct napi_struct *napi_ptr;
	atomic_t stop_napi_poll;

	/* Statistics */
	struct pktproc_statistics stat;

	/* Func */
	irqreturn_t (*irq_handler)(int irq, void *arg);
	void (*enable_irq)(struct pktproc_queue *q);
	void (*disable_irq)(struct pktproc_queue *q);
	int (*get_packet)(struct pktproc_queue *q, struct sk_buff **new_skb);
	int (*clean_rx_ring)(struct pktproc_queue *q, int budget, int *work_done);
	int (*alloc_rx_buf)(struct pktproc_queue *q);
	int (*update_fore_ptr)(struct pktproc_queue *q, u32 count);
	int (*clear_data_addr)(struct pktproc_queue *q);
};

/*
 * Descriptor structure mode
 * 0: RingBuf mode. Destination data address is decided by CP
 * 1: SktBuf mode. Destination data address is decided by AP
 */
enum pktproc_desc_mode {
	DESC_MODE_RINGBUF,
	DESC_MODE_SKTBUF,
	MAX_DESC_MODE
};

/*
 * PktProc version
 * 1: Single queue, ringbuf desc mode
 * 2: Multi queue, ringbuf/sktbuf desc mode
 */
enum pktproc_version {
	PKTPROC_V1 = 1,
	PKTPROC_V2,
	MAX_VERSION
};

enum pktproc_perftest_mode {
	PERFTEST_MODE_STOP,
	PERFTEST_MODE_IPV4,
	PERFTEST_MODE_CLAT,
	PERFTEST_MODE_IPV6,
	PERFTEST_MODE_MAX
};

struct pktproc_perftest {
	bool test_run;
	enum pktproc_perftest_mode mode;
	int session;
	u16 ch;
	int cpu;
	int udelay;
	u32 seq_counter[PKTPROC_MAX_QUEUE];
	u16 clat_ipv6[8];
	int ipi_cpu[PKTPROC_MAX_QUEUE];
};

struct pktproc_perftest_data {
	u8 header[48];
	u32 header_len;
	u16 dst_port_offset;
	u16 packet_len;
};

/* PktProc adaptor */
struct pktproc_adaptor {
	bool support;	/* Is support PktProc feature? */
	enum pktproc_version version;	/* Version */

	u32 cp_base;		/* CP base address for pktproc */
	u32 info_rgn_offset;	/* Offset of info region */
	u32 info_rgn_size;	/* Size of info region */
	u32 desc_rgn_offset;	/* Offset of descriptor region */
	u32 desc_rgn_size;	/* Size of descriptor region */
	u32 buff_rgn_offset;	/* Offset of data buffer region */

	bool info_desc_rgn_cached;
	bool buff_rgn_cached;

	enum pktproc_desc_mode desc_mode;	/* Descriptor structure mode */
	u32 desc_num_ratio_percent;		/* Number of descriptors ratio as percent */
	u32 num_queue;		/* Number of queue */
	bool use_exclusive_irq;	/* Exclusive interrupt */
	u32 exclusive_irq_idx[PKTPROC_MAX_QUEUE];
	bool use_hw_iocc;	/* H/W IO cache coherency */
	u32 max_packet_size;	/* Max packet size */
	bool use_dedicated_baaw;	/* BAAW for 36bit address */
	bool use_36bit_data_addr;	/* Data is located to 36bit address range */

	struct device *dev;

	bool use_napi;
	bool use_buff_mng;
	struct mif_buff_mng *manager;	/* Buffer manager */
	u32 skb_padding_size;

	void __iomem *info_vbase;	/* I/O region for information */
	void __iomem *desc_vbase;	/* I/O region for descriptor */
	void __iomem *buff_vbase;	/* I/O region for data buffer */
	unsigned long buff_pbase;
	struct pktproc_queue *q[PKTPROC_MAX_QUEUE];	/* Logical queue */

	/* Debug */
	struct pktproc_perftest perftest;
	bool pktgen_gro;
};

#if IS_ENABLED(CONFIG_CP_PKTPROC)
extern int pktproc_create(struct platform_device *pdev, struct mem_link_device *mld,
						unsigned long memaddr, u32 memsize);
extern int pktproc_init(struct pktproc_adaptor *ppa);
extern int pktproc_get_usage(struct pktproc_queue *q);
extern int pktproc_get_usage_fore_rear(struct pktproc_queue *q);

static inline int pktproc_check_support(struct pktproc_adaptor *ppa)
{
	return ppa->support;
}

static inline int pktproc_check_active(struct pktproc_adaptor *ppa, u32 q_idx)
{
	if (!pktproc_check_support(ppa))
		return 0;

	if (!ppa->q[q_idx])
		return 0;

	return atomic_read(&ppa->q[q_idx]->active);
}

static inline int pktproc_stop_napi_poll(struct pktproc_adaptor *ppa, u32 q_idx)
{
	if (!pktproc_check_support(ppa))
		return 0;

	if (!ppa->use_napi)
		return 0;

	if (!ppa->q[q_idx])
		return 0;

	atomic_set(&ppa->q[q_idx]->stop_napi_poll, 1);

	return 0;
}
#else
static inline int pktproc_create(struct platform_device *pdev, struct mem_link_device *mld,
				unsigned long memaddr, u32 memsize) { return 0; }
static inline int pktproc_init(struct pktproc_adaptor *ppa) { return 0; }
static inline int pktproc_get_usage(struct pktproc_queue *q) { return 0; }
static inline int pktproc_get_usage_fore_rear(struct pktproc_queue *q) { return 0; }
static inline int pktproc_check_support(struct pktproc_adaptor *ppa) { return 0; }
static inline int pktproc_check_active(struct pktproc_adaptor *ppa, u32 q_idx) { return 0; }
static inline int pktproc_stop_napi_poll(struct pktproc_adaptor *ppa, u32 q_idx) { return 0; }
#endif

#endif /* __LINK_RX_PKTPROC_H__ */
