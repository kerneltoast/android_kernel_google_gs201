/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License Version 2 as published
 * by the Free Software Foundation.
 *
 * Header for BTS Bus Traffic Shaper
 *
 * Includes Data structure for BTS device driver and pre-defined offsets
 * For offset values used in register setting, please refer regs-bts.h
 *
 */

#ifndef __BTS_H__
#define __BTS_H__

#include <linux/types.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <dt-bindings/soc/google/gs-bts.h>
#include <soc/google/exynos-pd.h>
#include <soc/google/bts.h>

#define NUM_CHANNEL 4
#define BUS_WIDTH 16
#define MIF_UTIL 65
#define INT_UTIL 65
#define RT_UTIL 40

#if IS_ENABLED(CONFIG_SOC_ZUMA)
#define NOCL2A_NUM_CHANNEL 2
#define INT_BUS_WIDTH 32
#define URGENT_TIMEOUT_EPOCH 16
#endif

#define DEFAULT_QOS 0x4
#define MAX_QOS 0xF
#define SCIMAX_QOS 0x3
#define MAX_MO 0xFFFF
#define MAX_QUTH 0xFF

struct bts_scen;
struct bts_stat;
struct bts_info;

#if IS_ENABLED(CONFIG_SOC_ZUMA)
struct nocl_ip_info {
	const char *ip_name;
	u32 ip_bus_width;
};
struct nocl_info {
	const char *nocl_name;
	u32 num_ip;
	u32 peak_freq;
	struct nocl_ip_info *nocl_ips;
	struct list_head list;
};
#endif

/**
 * struct bts - Device BTS structure
 * @lock:	a spin-lock to protect accessing bts
 * @mutex_lock: mutex-lock to protect accessing setting DVFS
 *
 * @num_bts:		number of bts hardware
 * @num_scen:		number of scenario be used
 * @num_rts:		number of RT client
 * @num_urgent_lat:	number of bts hardware support set urgent latency
 * @top_scen:		current top scenario
 *
 * @bts_list:	struct bts_info * - start address pointer for BTS devices
 * @scen_list:  struct bts_scen * - start address pointer for BTS scenarios
 * @scen_node:	list node - contains structure about scenario
 *
 * @bts_bw:	struct bts_bw * - struct for saving bandwidth information
 * @peak_bw:	currently max bandwidth
 * @total_bw:	current total bandwidth
 *
 * This structure stores basic BTS information for QoS control
 *
 * Note that it contains only basic information of BTS device driver
 *
 */
struct bts_device {
	struct device *dev;

	/* bts spinlock */
	spinlock_t lock;
	/* mutex-lock to protect accessing setting DVFS */
	struct mutex mutex_lock;

	unsigned int num_bts;
	unsigned int num_scen;
	unsigned int num_rts;
	unsigned int top_scen;

	struct bts_info *bts_list;
	struct bts_scen *scen_list;
	struct list_head scen_node;

	struct bts_bw *bts_bw;
	const char **rt_names;
	unsigned int peak_bw;
	unsigned int total_bw;
	struct bus1_int_map *bus1_int_tbl;
	unsigned int map_row_cnt;

#if IS_ENABLED(CONFIG_SOC_ZUMA)
	unsigned int num_nocl;
	struct nocl_info *nocl_infos;
	unsigned int num_urgent_lat;
	struct bts_urgent_lat *urgent_lat_list;
#endif
};

/**
 * struct bts_ops - operation functions for BTS setting
 * @init_bts:	function for initialize.
 *		It will be run only on initial time and after system resume
 * @set_bts:	operation function to set bts.
 * @get_bts:	operation function to get bts.
 * @set_qos:	operation function to set axqos of bts.
 * @get_qos:	operation function to get axqos of bts.
 * @set_mo:	operation function to set xmo of bts.
 * @get_mo:	operation function to get xmo of bts.
 * @set_urgent:	operation function to set urgent timeout of bts.
 * @get_urgent:	operation function to get urgent timeout of bts.
 * @set_blocking: operation function to set blocking threshold of bts.
 * @get_blocking: operation function to get blocking threshold of bts.
 *
 */
struct bts_ops {
	void (*init_bts)(void __iomem *va);
	int (*set_bts)(void __iomem *va, struct bts_stat *stat);
	int (*get_bts)(void __iomem *va, struct bts_stat *stat);
	int (*set_qos)(void __iomem *va, struct bts_stat *stat);
	int (*get_qos)(void __iomem *va, struct bts_stat *stat);
	int (*set_mo)(void __iomem *va, struct bts_stat *stat);
	int (*get_mo)(void __iomem *va, struct bts_stat *stat);
	int (*set_urgent)(void __iomem *va, struct bts_stat *stat);
	int (*get_urgent)(void __iomem *va, struct bts_stat *stat);
	int (*set_blocking)(void __iomem *va, struct bts_stat *stat);
	int (*get_blocking)(void __iomem *va, struct bts_stat *stat);
	int (*set_vc)(void __iomem *va, unsigned int value);
	int (*get_vc)(void __iomem *va, unsigned int *value);
};

/**
 * struct bts_scen - BTS scenario
 * @node
 * @index:	scenario index
 * @status:	scenario status (0 means scenario is off / 1 means on)
 * @name:	scenario name
 *
 * BTS scenario is defined in Device Tree and parsed in probe function
 * index will be used as priority order of scenarios
 */
struct bts_scen {
	struct list_head node;

	unsigned int index;
	bool status;
	int usage_count;
	const char *name;
};

#if IS_ENABLED(CONFIG_SOC_ZUMA)
struct bts_urgent_lat {
	unsigned int bts_index;
	unsigned int lat_read;
	unsigned int th_read;
	const char *name;
};
#endif

/**
 * struct bts_stat - BTS status
 * @stat_on:		shows current set of config can be used or not
 * @bypass:		when BTS must bypass master request, set as true
 *
 * @arqos:		ArQoS value for master IP (0x0~0xF)
 * @awqos:		AwQoS value for master IP (0x0~0xF)
 * @rmo:		Multiple Outstanding value for Read
 * @wmo:		Multiple Outstanding value for Write
 * @qurgent:		Signal from master which accelerates delayed requests
 * @qurgent_th_r/w:	Threshold for Qurgent. Read/Write is separated
 * @blocking:		Signal from slave which limits master requests
 * @qmax_limit_r/w:	Limitation criteria when QMAX is enabled
 * @qfull_limit_r/w:	Limitation criteria when QFULL is enabled
 * @qbusy_limit_r/w:	Limitation criteria when QBUSY is enabled
 *
 * This structure stores data to control BTS
 *
 */
struct bts_stat {
	bool stat_on;
	bool bypass;
	unsigned int arqos;
	unsigned int awqos;
	unsigned int rmo;
	unsigned int wmo;
	bool qurgent_on;
	bool ex_qurgent_on;
	unsigned int qurgent_th_r;
	unsigned int qurgent_th_w;
	bool blocking_on;
	unsigned int qfull_limit_r;
	unsigned int qfull_limit_w;
	unsigned int qbusy_limit_r;
	unsigned int qbusy_limit_w;
	unsigned int qmax0_limit_r;
	unsigned int qmax0_limit_w;
	unsigned int qmax1_limit_r;
	unsigned int qmax1_limit_w;
	void __iomem *qos_va_base;
};

/**
 * struct bts_info - BTS information
 * @name:	name of BTS
 * @pa_base:	Physical Address data of BTS
 * @va_base:	Virtual Address data of BTS
 * @status:	boolean value whether BTS is on/off
 * @type:	type of bts
 * @pd_name:	name of power domain
 * @pd_on:	whether related power domain is on/off
 * @stat:	list of array that contains BTS status data for QoS control
 * @ops:	operation function classified according to bts type
 *
 */
struct bts_info {
	const char *name;

	unsigned int pa_base;
	void __iomem *va_base;

	bool status;

	unsigned int type;

	unsigned int pd_id;
	bool pd_on;

	struct bts_stat *stat;
	struct bts_ops *ops;
};

int register_btsops(struct bts_info *info);
int exynos_bts_debugfs_init(void);
#endif
