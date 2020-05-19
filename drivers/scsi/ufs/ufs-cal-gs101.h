/* SPDX-License-Identifier: GPL-2.0-only */
//
// Exynos UIC configuration driver
//
// Copyright (C) 2020 Samsung Electronics Co., Ltd.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
struct uic_pwr_mode {
	u8 lane;
	u8 gear;
	u8 mode;
	u8 hs_series;
};

enum {
	HOST_EMBD = 0,
	HOST_CARD = 1,
};

enum {
	GEAR_1 = 1,
	GEAR_2,
	GEAR_3,
	GEAR_4,
};

struct ufs_cal_param {
	void *host;		/* Host adaptor */
	u8 available_lane;
	u8 connected_tx_lane;
	u8 connected_rx_lane;
	u8 active_tx_lane;
	u8 active_rx_lane;
	u32 mclk_rate;
	u8 tbl;
	u8 board;
	u8 evt_ver;
	u8 max_gear;
	struct uic_pwr_mode *pmd;

	/* private data */
	u32 mclk_period;
	u32 mclk_period_unipro_18;
};

enum ufs_cal_errno {
	UFS_CAL_NO_ERROR = 0,
	UFS_CAL_TIMEOUT,
	UFS_CAL_ERROR,
	UFS_CAL_INV_ARG,
};

enum {
	__BRD_SMDK,
	__BRD_ASB,
	__BRD_HSIE,
	__BRD_ZEBU,
	__BRD_UNIV,
	__BRD_MAX,
};

#define BRD_SMDK	BIT(__BRD_SMDK)
#define BRD_ASB		BIT(__BRD_ASB)
#define BRD_HSIE	BIT(__BRD_HSIE)
#define BRD_ZEBU	BIT(__BRD_ZEBU)
#define BRD_UNIV	BIT(__BRD_UNIV)
#define BRD_MAX		BIT(__BRD_MAX)
#define BRD_ALL		(BIT(__BRD_MAX) - 1)

/* UFS CAL interface */
enum ufs_cal_errno ufs_cal_post_h8_enter(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_pre_h8_exit(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_post_pmc(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_pre_pmc(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_post_link(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_pre_link(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_init(struct ufs_cal_param *p, int idx);

enum ufs_cal_errno ufs_cal_loopback_init(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_loopback_set_1(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_loopback_set_2(struct ufs_cal_param *p);

/* Adaptor for UFS CAL */
void ufs_lld_dme_set(void *h, u32 addr, u32 val);
void ufs_lld_dme_get(void *h, u32 addr, u32 *val);
void ufs_lld_dme_peer_set(void *h, u32 addr, u32 val);
void ufs_lld_pma_write(void *h, u32 val, u32 addr);
u32 ufs_lld_pma_read(void *h, u32 addr);
void ufs_lld_unipro_write(void *h, u32 val, u32 addr);
void ufs_lld_udelay(u32 val);
void ufs_lld_usleep_delay(u32 min, u32 max);
unsigned long ufs_lld_get_time_count(unsigned long offset);
unsigned long ufs_lld_calc_timeout(const unsigned int ms);
