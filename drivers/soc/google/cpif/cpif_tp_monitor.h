/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019-2020, Samsung Electronics.
 *
 */

#ifndef __CPIF_TP_MONITOR_H__
#define __CPIF_TP_MONITOR_H__

#include <linux/ktime.h>
#include <linux/workqueue.h>
#if IS_ENABLED(CONFIG_CPU_FREQ)
#include <linux/cpufreq.h>
#endif
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
#include <soc/google/exynos_pm_qos.h>
#endif

#define MAX_TPMON_DATA	16
#define MAX_TPMON_THRESHOLD	10
#define MAX_TPMON_VALUES	(MAX_TPMON_THRESHOLD+1)
#define MAX_RPS_STRING	8
#define MAX_IRQ_AFFINITY_DATA	5
#define MAX_RX_BYTES_COUNT	1000

struct tpmon_data {
	struct cpif_tpmon *tpmon;

	struct list_head data_node;

	char *name;
	u32 measure;
	u32 target;
	u32 enable;
	u32 extra_idx;

	u32 num_threshold;
	u32 threshold[MAX_TPMON_THRESHOLD];
	u32 curr_threshold_pos;
	u32 prev_threshold_pos;

	u32 num_values;
	u32 values[MAX_TPMON_VALUES];
	u32 curr_value_pos;
	u32 prev_value_pos;
	u32 user_value;

	u32 unboost_tp_mbps[MAX_TPMON_THRESHOLD];
	u64 jiffies_to_unboost;

	void *extra_data;

	void (*set_data)(struct tpmon_data *data);
	u32 (*get_data)(struct tpmon_data *data);
};

struct cpif_tpmon {
	struct link_device *ld;

	atomic_t need_init;
	atomic_t active;
	spinlock_t lock;

	struct list_head data_list;

	u32 trigger_mbps;
	u32 trigger_msec_min;
	u32 trigger_msec_max;
	u32 monitor_interval_msec;
	u32 monitor_hold_msec;
	u32 boost_hold_msec;
	u32 unboost_tp_percent;

	u64 jiffies_to_trigger;

	struct workqueue_struct *monitor_wq;
	struct delayed_work monitor_dwork;

	unsigned long rx_bytes;
	unsigned long rx_mbps;
	unsigned long rx_kbps;
	unsigned long rx_sum;
	unsigned long rx_bytes_data[MAX_RX_BYTES_COUNT];

	u32 pktproc_queue_status;
	u32 netdev_backlog_queue_status;
	u32 dit_src_queue_status;

	u32 use_user_value;
	u32 debug_print;

	struct tpmon_data data[MAX_TPMON_DATA];

#if IS_ENABLED(CONFIG_RPS)
	struct list_head net_node_list;
#endif
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
	struct exynos_pm_qos_request qos_req_mif;
	struct exynos_pm_qos_request qos_req_int;
#endif
#if IS_ENABLED(CONFIG_CPU_FREQ)
	struct notifier_block cpufreq_nb;
	struct freq_qos_request qos_req_cpu_cl0;
	struct freq_qos_request qos_req_cpu_cl1;
	struct freq_qos_request qos_req_cpu_cl2;
#endif
#if IS_ENABLED(CONFIG_EXYNOS_BTS)
	int bts_scen_index;
#endif

	/* Func */
	int (*start)(void);
	int (*stop)(void);
	void (*add_rx_bytes)(unsigned long bytes);
	int (*check_active)(void);
};

#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
extern int tpmon_create(struct platform_device *pdev, struct link_device *ld);
extern int tpmon_start(void);
extern int tpmon_stop(void);
extern int tpmon_init(void);
extern void tpmon_add_rx_bytes(unsigned long bytes);
extern void tpmon_add_net_node(struct list_head *node);
extern int tpmon_check_active(void);
#else
static inline int tpmon_create(struct platform_device *pdev, struct link_device *ld) { return 0; }
static inline int tpmon_start(void) { return 0; }
static inline int tpmon_stop(void) { return 0; }
static inline int tpmon_init(void) { return 0; }
static inline void tpmon_add_rx_bytes(unsigned long bytes) { return; }
static inline void tpmon_add_net_node(struct list_head *node) { return; }
static inline int tpmon_check_active(void) { return 0; }
#endif

#endif /* __CPIF_TP_MONITOR_H__ */
