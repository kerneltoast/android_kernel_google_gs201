/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Google, LLC
 *
 */

#ifndef MAX77759_CHARGER_H_
#define MAX77759_CHARGER_H_

#include "max77759_usecase.h"
#include "max777x9_bcl.h"

struct max77759_chgr_data {
	struct device *dev;

	struct power_supply *psy;
	struct power_supply *wcin_psy;
	struct power_supply *chgin_psy;

	struct power_supply *wlc_psy;
	struct regmap *regmap;

	struct gvotable_election *mode_votable;
	struct max77759_usecase_data uc_data;
	struct delayed_work mode_rerun_work;

	struct gvotable_election *dc_icl_votable;
	struct gvotable_election *dc_suspend_votable;

	/* wcin inlim tracking */
	struct delayed_work wcin_inlim_work;
	uint32_t wcin_inlim_period;
	uint32_t wcin_inlim_flag;
	uint32_t wcin_inlim_headroom;
	uint32_t wcin_inlim_step;
	uint32_t wcin_soft_icl;
	uint32_t wcin_inlim_en;
	uint32_t dc_icl;
	struct mutex wcin_inlim_lock;

#if IS_ENABLED(CONFIG_GPIOLIB)
	struct gpio_chip gpio;
#endif

	bool charge_done;
	bool chgin_input_suspend;
	bool wcin_input_suspend;
	bool wlc_spoof;
	bool thm2_sts;

	int irq_gpio;
	int irq_int;

	uint32_t cc_max;

	struct i2c_client *fg_i2c_client;
	struct i2c_client *pmic_i2c_client;

	struct dentry *de;

	atomic_t insel_cnt;
	bool insel_clear;	/* when set, irq clears CHGINSEL_MASK */

	atomic_t early_topoff_cnt;

	struct mutex io_lock;
	struct mutex reg_dump_lock;
	bool resume_complete;
	bool init_complete;
	struct wakeup_source *usecase_wake_lock;

	int fship_dtls;
	bool online;
	bool wden;

	/* Force to change FCCM mode during OTG at high battery voltage */
	bool otg_changed;
	bool otg_fccm_reset;
	int otg_fccm_vbatt_lowerbd;
	int otg_fccm_vbatt_upperbd;
	struct delayed_work otg_fccm_worker;
	struct wakeup_source *otg_fccm_wake_lock;

	/* debug interface, register to read or write */
	u32 debug_reg_address;

	struct gvotable_election *aicl_active_el;

	struct gvotable_election *msc_last_votable;
	int chg_term_voltage;
	int chg_term_volt_debounce;
};

#endif
