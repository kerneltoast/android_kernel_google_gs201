/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_BCM_DBG_DUMP_H_
#define __EXYNOS_BCM_DBG_DUMP_H_

/* BCM DUMP format definition */
#define BCM_DUMP_PRE_DEFINE_SHIFT	(16)
#define BCM_DUMP_MAX_STR		(4 * 1024)
#define BCM_DUMP_MAX_LINE_SIZE		(256)

struct exynos_bcm_dump_info {
	/*
	 * dump_header:
	 * [31] = dump validation
	 * [18:16] = pre-defined event index
	 * [5:0] = BCM IP index
	 */
	u32				dump_header;
	u32				dump_seq_no;
} __attribute__((packed));

struct exynos_bcm_out_data {
	u32				measure_time;
	u32				ccnt;
	u32				pmcnt[BCM_EVT_EVENT_MAX];
};

struct exynos_bcm_dump_entry {
	struct exynos_bcm_dump_info	dump_info;
	struct exynos_bcm_out_data	dump_data;
};

struct exynos_bcm_accumulator_data {
	u64				measure_time;
	u64				ccnt;
	u64				pmcnt[BCM_EVT_EVENT_MAX];
};

#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP)
int exynos_bcm_dbg_buffer_dump(struct exynos_bcm_dbg_data *data);
int exynos_bcm_dbg_dump(struct exynos_bcm_dbg_data *data, char *buff_out, size_t size_buff,
		loff_t off);
#else
#define exynos_bcm_dbg_buffer_dump(a) do {} while (0)
#define exynos_bcm_dbg_dump(b, c, d, e) do {} while (0)
#endif

int exynos_bcm_dbg_print_accumulators(struct exynos_bcm_dbg_data *data,
	bool klog, char *buf, size_t *buf_len, loff_t off, size_t size);

#endif	/* __EXYNOS_BCM_DBG_DUMP_H_ */
