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
#if IS_ENABLED(CONFIG_ARM_FREQ_QOS_TRACER)
#include <soc/google/freq-qos-tracer.h>
#endif

#define MAX_TPMON_DATA	16
#define MAX_TPMON_THRESHOLD	10
#define MAX_TPMON_LEVEL	(MAX_TPMON_THRESHOLD+1)
#define MAX_RPS_STRING	8
#define MAX_IRQ_AFFINITY_DATA	5
#define MAX_IRQ_AFFINITY_STRING	8
#define MAX_RX_BYTES_COUNT	1000

struct tpmon_data {
	struct cpif_tpmon *tpmon;

	struct list_head data_node;
	struct list_head tp_node;
	struct list_head q_status_node;

	char *name;
	u32 target;

	u32 num_level;
	u32 level[MAX_TPMON_LEVEL];
	u32 curr_level_pos;
	u32 prev_level_pos;
	u32 user_level;

	u32 enable;
	u32 measure;
	u32 proto;
	u32 extra_idx;

	u32 num_threshold;
	u32 threshold[MAX_TPMON_THRESHOLD];
	u32 curr_threshold_pos;
	u32 prev_threshold_pos;

	u32 unboost_threshold_mbps[MAX_TPMON_THRESHOLD];
	ktime_t prev_unboost_time;

	bool need_boost;

	void *extra_data;

	u32 (*get_data)(struct tpmon_data *data);
	void (*set_data)(struct tpmon_data *data);
};

struct cpif_rx_data {
	unsigned long rx_bytes;
	unsigned long rx_mbps;

	ktime_t prev_time;
};

struct cpif_tpmon {
	struct link_device *ld;

	atomic_t need_init;
	atomic_t active;
	spinlock_t lock;

	u32 trigger_msec_min;
	u32 trigger_msec_max;

	u32 monitor_interval_msec;
	u32 monitor_hold_msec;
	u32 monitor_stop_mbps;

	u32 boost_hold_msec;

	struct list_head all_data_list;
	struct list_head tp_node_list;
	struct list_head q_status_list;
	struct list_head net_node_list;

	ktime_t prev_monitor_time;
	struct workqueue_struct *monitor_wq;
	struct delayed_work monitor_dwork;

	atomic_t boost_active;
	struct workqueue_struct *boost_wq;
	struct delayed_work boost_dwork;

	struct cpif_rx_data rx_total;
	struct cpif_rx_data rx_tcp;
	struct cpif_rx_data rx_udp;
	struct cpif_rx_data rx_others;

	struct cpif_rx_data rx_total_stat;
	struct cpif_rx_data rx_tcp_stat;
	struct cpif_rx_data rx_udp_stat;
	struct cpif_rx_data rx_others_stat;

	u32 q_status_pktproc_dl;
	u32 q_status_netdev_backlog;
	u32 q_status_dit_src;
	u32 legacy_packet_count;

	u32 use_user_level;
	u32 debug_print;

	struct tpmon_data data[MAX_TPMON_DATA];

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
	struct exynos_pm_qos_request qos_req_mif;
	struct exynos_pm_qos_request qos_req_mif_max;
	struct exynos_pm_qos_request qos_req_int;
	struct exynos_pm_qos_request qos_req_int_max;
#endif

#if IS_ENABLED(CONFIG_CPU_FREQ)
	struct notifier_block cpufreq_nb;
	struct freq_qos_request qos_req_cpu_cl0;
	struct freq_qos_request qos_req_cpu_cl0_max;
	struct freq_qos_request qos_req_cpu_cl1;
	struct freq_qos_request qos_req_cpu_cl1_max;
	struct freq_qos_request qos_req_cpu_cl2;
	struct freq_qos_request qos_req_cpu_cl2_max;
#endif

#if IS_ENABLED(CONFIG_EXYNOS_BTS)
	int bts_scen_index;
#endif

	/* Func */
	int (*start)(void);
	int (*stop)(void);
	void (*add_rx_bytes)(struct sk_buff *skb);
	int (*check_active)(void);
	void (*reset_data)(char *name);
};

static inline u32 tpmon_get_curr_level(struct tpmon_data *data)
{
	if (!data->enable)
		return 0;

	return data->tpmon->use_user_level ?
		data->user_level : data->level[data->curr_level_pos];
}

#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
extern int tpmon_create(struct platform_device *pdev, struct link_device *ld);
extern int tpmon_start(void);
extern int tpmon_stop(void);
extern int tpmon_init(void);
extern void tpmon_add_rx_bytes(struct sk_buff *skb);
extern void tpmon_add_legacy_packet_count(u32 count);
extern void tpmon_add_net_node(struct list_head *node);
extern int tpmon_check_active(void);
#else
static inline int tpmon_create(struct platform_device *pdev, struct link_device *ld) { return 0; }
static inline int tpmon_start(void) { return 0; }
static inline int tpmon_stop(void) { return 0; }
static inline int tpmon_init(void) { return 0; }
static inline void tpmon_add_rx_bytes(unsigned long bytes) { return; }
static inline void tpmon_add_legacy_packet_count(u32 count) { return; }
static inline void tpmon_add_net_node(struct list_head *node) { return; }
static inline int tpmon_check_active(void) { return 0; }
#endif

#endif /* __CPIF_TP_MONITOR_H__ */
