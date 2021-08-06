/* SPDX-License-Identifier: GPL-2.0-only */
//
// UFS Host Controller driver for Exynos specific extensions
//
// Copyright (C) 2013-2014 Samsung Electronics Co., Ltd.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.

#ifndef _UFS_EXYNOS_H_
#define _UFS_EXYNOS_H_

#include <soc/google/exynos_pm_qos.h>
#include "ufs-vs-mmio.h"
#include "ufs-vs-regs.h"
#include "ufs-cal-if.h"
#include "ufs-pixel.h"

#define UFS_VER_0004	4
#define UFS_VER_0005	5

enum {
	UFS_S_MON_LV1 = (1 << 0),
	UFS_S_MON_LV2 = (1 << 1),
};

struct ext_cxt {
	u32 offset;
	u32 mask;
	u32 val;
};

/*
 * H_UTP_BOOST and H_FATAL_ERR arn't in here because they were just
 * defined to enable some callback functions explanation.
 */
enum exynos_host_state {
	H_DISABLED = 0,
	H_RESET = 1,
	H_LINK_UP = 2,
	H_LINK_BOOST = 3,
	H_TM_BUSY = 4,
	H_REQ_BUSY = 5,
	H_HIBERN8 = 6,
	H_SUSPEND = 7,
};

enum exynos_clk_state {
	C_OFF = 0,
	C_ON,
};

enum exynos_ufs_ext_blks {
	EXT_SYSREG = 0,
	EXT_BLK_MAX,
#define EXT_BLK_MAX 1
};

enum exynos_ufs_param_id {
	UFS_S_PARAM_EOM_VER = 0,
	UFS_S_PARAM_EOM_SZ,
	UFS_S_PARAM_EOM_OFS,
	UFS_S_PARAM_LANE,
	UFS_S_PARAM_H8_D_MS,
	UFS_S_PARAM_MON,
	UFS_S_PARAM_NUM,
};

struct exynos_ufs {
	struct device *dev;
	struct ufs_hba *hba;

	/*
	 * Do not change the order of iomem variables.
	 * Standard HCI regision is populated in core driver.
	 */
	void __iomem *reg_hci;			/* exynos-specific hci */
	void __iomem *reg_unipro;		/* unipro */
	void __iomem *reg_ufsp;			/* ufs protector */
	void __iomem *reg_phy;			/* phy */
	void __iomem *reg_cport;		/* cport */
#define NUM_OF_UFS_MMIO_REGIONS 5

	/*
	 * Do not change the order of remap variables.
	 */
	struct regmap *regmap_sys;		/* io coherency */
	struct ext_cxt cxt_phy_iso;
	struct ext_cxt cxt_iocc;

	/*
	 * Do not change the order of clock variables
	 */
	struct clk *clk_hci;
	struct clk *clk_unipro;

	/* exynos specific state */
	enum exynos_host_state h_state;
	enum exynos_host_state h_state_prev;
	enum exynos_clk_state c_state;

	u32 mclk_rate;

	int num_lanes;

	struct uic_pwr_mode req_pmd_parm;
	struct uic_pwr_mode act_pmd_parm;

	int id;

	/* to prevent races to dump among threads */
	spinlock_t dbg_lock;
	int under_dump;

	/* Support system power mode */
	int idle_ip_index;

	/* PM QoS for stability, not for performance */
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
	struct exynos_pm_qos_request	pm_qos_int;
#endif
	s32			pm_qos_int_value;

	/* cal */
	struct ufs_cal_param	cal_param;

	/* performance */
	void *perf;
	struct ufs_vs_handle handle;

	u32 peer_available_lane_rx;
	u32 peer_available_lane_tx;
	u32 available_lane_rx;
	u32 available_lane_tx;

	/*
	 * This variable is to make UFS driver's operations change
	 * for specific purposes, e.g. unit test cases, or report
	 * some information to user land.
	 */
	u32 params[UFS_S_PARAM_NUM];

	/* sysfs */
	struct kobject sysfs_kobj;
	struct work_struct update_sysfs_work;

	/* manual_gc */
	struct ufs_manual_gc manual_gc;

	/* pixel ufs request statistics */
	struct pixel_req_stats req_stats[REQ_TYPE_MAX];
	u64 peak_reqs[REQ_TYPE_MAX];
	u64 peak_queue_depth;
	/* pixel ufs I/O quatity statistics */
	struct pixel_io_stats io_stats[IO_TYPE_MAX];

	/* To monitor slow UFS I/O requests. */
	u64 slowio_min_us;
	u64 slowio[PIXEL_SLOWIO_OP_MAX][PIXEL_SLOWIO_SYS_MAX];

	/* Pointer to GSA device */
	struct device *gsa_dev;

	/* Hibern8 recording */
	struct pixel_ufs_stats ufs_stats;

	/* ufs command logging */
	u8 enable_cmd_log;
	struct pixel_cmd_log cmd_log;

	/* security_out write counter */
	u32 security_out_wc;
};

static inline struct exynos_ufs *to_exynos_ufs(struct ufs_hba *hba)
{
	return dev_get_platdata(hba->dev);
}

int exynos_ufs_init_dbg(struct ufs_vs_handle *handle, struct device *dev);
int exynos_ufs_dbg_set_lanes(struct ufs_vs_handle *handle,
			     struct device *dev, u32 lane);
void exynos_ufs_dump_info(struct ufs_vs_handle *handle, struct device *dev);
void exynos_ufs_cmd_log_start(struct ufs_vs_handle *handle,
			      struct ufs_hba *hba, struct scsi_cmnd *cmd);
void exynos_ufs_cmd_log_end(struct ufs_vs_handle *handle,
			    struct ufs_hba *hba, int tag);

#ifdef CONFIG_SCSI_UFS_CRYPTO
int pixel_ufs_crypto_init(struct ufs_hba *hba);
void pixel_ufs_crypto_resume(struct ufs_hba *hba);
#else
static inline int pixel_ufs_crypto_init(struct ufs_hba *hba)
{
	return 0;
}
static inline void pixel_ufs_crypto_resume(struct ufs_hba *hba)
{
}
#endif /* !CONFIG_SCSI_UFS_CRYPTO */
#endif /* _UFS_EXYNOS_H_ */
