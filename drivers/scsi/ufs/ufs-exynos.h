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

#include <linux/pm_qos.h>
#include <ufs-vs-mmio.h>
#include <ufs-vs-regs.h>
#include "ufs-cal-if.h"
#include "ufs-pixel.h"

/* Version check */
#define UFS_EXYNOS_COMPAT_CAL_MMIO_VER 1
#define UFS_EXYNOS_COMPAT_CAL_IF_VER 1
#if (UFS_VS_MMIO_VER != UFS_EXYNOS_COMPAT_CAL_MMIO_VER)
#error "UFS_VS_MMIO_VER and UFS_EXYNOS_COMPAT_CAL_MMIO_VER aren't matched"
#endif
#if (UFS_CAL_IF_VER != UFS_EXYNOS_COMPAT_CAL_IF_VER)
#error "UFS_CAL_IF_VER and UFS_EXYNOS_COMPAT_CAL_IF_VER aren't matched"
#endif

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
	struct pm_qos_request	pm_qos_int;
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

	/* manual_gc */
	struct ufs_manual_gc manual_gc;
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
void exynos_ufs_fmp_init(struct ufs_hba *hba);
void exynos_ufs_fmp_resume(struct ufs_hba *hba);
int exynos_ufs_fmp_fill_prdt(struct ufs_hba *hba, struct ufshcd_lrb *lrbp,
			     unsigned int segments);
#else
static inline void exynos_ufs_fmp_init(struct ufs_hba *hba)
{
}
static inline void exynos_ufs_fmp_resume(struct ufs_hba *hba)
{
}
#define exynos_ufs_fmp_fill_prdt NULL
#endif /* !CONFIG_SCSI_UFS_CRYPTO */

void pixel_ufs_prepare_command(struct ufs_hba *hba,
			struct request *rq, struct ufshcd_lrb *lrbp);
int pixel_ufs_update_sysfs(struct ufs_hba *hba);

#endif /* _UFS_EXYNOS_H_ */
