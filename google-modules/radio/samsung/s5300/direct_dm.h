/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Samsung Electronics.
 *
 */

#ifndef __DIRECT_DM_H__
#define __DIRECT_DM_H__

#include <linux/interrupt.h>

enum direct_dm_desc_control_bits {
	DDM_DESC_C_END = 3,		/* End of buffer descriptor */
	DDM_DESC_C_INT		/* Interrupt enabled */
};

enum direct_dm_desc_status_bits {
	DDM_DESC_S_DONE,	/* DMA done */
	DDM_DESC_S_TOUT,	/* Transferred on timeout */
	DDM_DESC_S_COMPRESSED = 5	/* "1":compressed log is on, "0":comp is off */
};

struct direct_dm_desc {
	u64 cp_buff_paddr:40,
		_reserved_0:24;
	u64	length:16,
		status:8,
		control:8,
		_reserved_1:32;
} __packed;

struct direct_dm_info_rgn {
	u64 version:4,
		_reserved_0:4,
		max_packet_size:16,
		silent_log:1,
		_reserved_1:39;
	u64	cp_desc_pbase:40,
		_reserved_2:24;
	u64	cp_buff_pbase:40,
		_reserved_3:24;
} __packed;

/* Statistics */
struct direct_dm_statistics {
	u64 err_desc_done;
	u64 err_desc_tout;
	u64 err_length;

	u64 usb_req_cnt;
	u64 err_usb_req;
	u64 usb_complete_cnt;
	u64 err_usb_complete;

	u64 upper_layer_req_cnt;
	u64 err_upper_layer_req;

	u64 rx_timer_req;
	u64 rx_timer_expire;
};

struct direct_dm_ctrl {
	struct link_device *ld;

	u32 version;
	u32 shm_rgn_index;

	u32 hw_iocc;
	u32 info_desc_rgn_cached;
	u32 buff_rgn_cached;

	u32 info_rgn_offset;
	u32 info_rgn_size;
	u32 desc_rgn_offset;
	u32 desc_rgn_size;
	u32 num_desc;
	u32 buff_rgn_offset;
	u32 buff_rgn_size;

	u32 max_packet_size;
	u32 usb_req_num;
	u32 irq_index;

	u32 cp_ddm_pbase;

	struct device *dev;

	u32 curr_desc_pos;
	u32 curr_done_pos;

	bool usb_req_failed;

	spinlock_t rx_lock;
	struct tasklet_struct rx_task;
	bool use_rx_task;

	bool use_rx_timer;
	spinlock_t rx_timer_lock;
	struct hrtimer rx_timer;
	u32 rx_timer_period_msec;

	bool usb_active;

	struct direct_dm_statistics stat;

	bool enable_debug;

	void __iomem *info_vbase;
	void __iomem *desc_vbase;
	void __iomem *buff_vbase;
	unsigned long buff_pbase;

	struct direct_dm_info_rgn *info_rgn;
	struct direct_dm_desc *desc_rgn;
};

#if IS_ENABLED(CONFIG_CPIF_DIRECT_DM)
extern int direct_dm_create(struct platform_device *pdev);
extern int direct_dm_init(struct link_device *ld);
extern int direct_dm_deinit(void);
#else
static inline int direct_dm_create(struct platform_device *pdev) { return 0; }
static inline int direct_dm_init(struct link_device *ld) { return 0; }
static inline int direct_dm_deinit(void) { return 0; }
#endif

#endif /* __DIRECT_DM_H__ */
