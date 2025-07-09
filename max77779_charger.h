/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Google, LLC
 *
 */

#ifndef MAX77779_CHARGER_H_
#define MAX77779_CHARGER_H_

#include <linux/gpio.h>

#include "max77779_usecase.h"

#define MAX77779_COP_SENSE_RESISTOR_VAL 2 /* 2mOhm */
#define MAX77779_COP_MAX_VALUE (0xffff * 1000 / MAX77779_COP_SENSE_RESISTOR_VAL)
#define MAX77779_COP_WARN_THRESHOLD 105 /* Percentage */
#define MAX77779_COP_MIN_DEBOUNCE_TIME_MS 16
#define MAX77779_CHG_NUM_IRQS 16

struct max77779_chgr_data {
	struct device *dev;

	/* Charger sub-IRQ routing for COP */
	struct irq_domain	*domain;
	uint32_t		mask;
	uint32_t		mask_u;  /* pending updates */
	uint32_t		trig_type;
	struct mutex 		irq_lock;

	struct power_supply *psy;
	struct power_supply *wcin_psy;
	struct power_supply *chgin_psy;

	struct power_supply *wlc_psy;
	struct power_supply *fg_psy;
	struct regmap *regmap;

	struct gvotable_election *mode_votable;
	struct max77779_usecase_data uc_data;
	struct delayed_work mode_rerun_work;

	struct gvotable_election *dc_icl_votable;
	struct gvotable_election *dc_suspend_votable;
	struct gvotable_election *wlc_spoof_votable;

	struct delayed_work cop_enable_work;
	uint32_t cop_warn;
	uint32_t cc_max;

	/* wcin inlim tracking */
	struct delayed_work wcin_inlim_work;
	uint32_t wcin_inlim_t;
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
	bool disable_internal_irq_handler;

	struct device *pmic_dev;

	struct dentry *de;

	atomic_t insel_cnt;
	bool insel_clear;	/* when set, irq clears CHGINSEL_MASK */

	atomic_t early_topoff_cnt;

	struct mutex io_lock;
	struct mutex mode_callback_lock;
	struct mutex prot_lock;
	struct mutex reg_dump_lock;
	bool resume_complete;
	bool init_complete;
	struct wakeup_source *usecase_wake_lock;

	int fship_dtls;
	bool online;
	bool wden;

	/* Force to change FCCM mode during OTG at high battery voltage */
	bool otg_changed;

	/* debug interface, register to read or write */
	u32 debug_reg_address;

	struct gvotable_election *msc_last_votable;
	int chg_term_voltage;
	int chg_term_volt_debounce;
};
int max77779_charger_init(struct max77779_chgr_data *data);
void max77779_charger_remove(struct max77779_chgr_data *data);
bool max77779_chg_is_reg(struct device *dev, unsigned int reg);
#if IS_ENABLED(CONFIG_PM)
int max77779_charger_pm_suspend(struct device *dev);
int max77779_charger_pm_resume(struct device *dev);
#endif
#endif
