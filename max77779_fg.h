/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MAX77779_FG_MODEL_H_
#define MAX77779_FG_MODEL_H_

#include "maxfg_common.h"
#include "max77779.h"

/* change to 1 or 0 to load FG model with default parameters on startup */
#define MAX77779_FG_LOAD_MODEL_DISABLED	-1
#define MAX77779_FG_LOAD_MODEL_IDLE	0
#define MAX77779_FG_LOAD_MODEL_REQUEST	1

#define MAX77779_FG_MODEL_START		MAX77779_FG_OCV0
#define MAX77779_FG_MODEL_SIZE		32

/* model version */
#define MAX77779_FG_INVALID_VERSION	-1

/* Config2: must not enable TAlert */
#define MAX77779_FG_MODEL_VERSION_REG	MAX77779_FG_TAlrtTh

#define MAX77779_FG_NDGB_ADDRESS 0x37

#define MAX77779_FG_MAX_LOG_REGS	30

static const struct maxfg_reg max77779_fg[] = {
	[MAXFG_TAG_avgc] = { ATOM_INIT_REG16(MAX77779_FG_AvgCurrent)},
	[MAXFG_TAG_cnfg] = { ATOM_INIT_REG16(MAX77779_FG_Config)},
	[MAXFG_TAG_mmdv] = { ATOM_INIT_REG16(MAX77779_FG_MaxMinVolt)},
	[MAXFG_TAG_vcel] = { ATOM_INIT_REG16(MAX77779_FG_VCell)},
	[MAXFG_TAG_temp] = { ATOM_INIT_REG16(MAX77779_FG_Temp)},
	[MAXFG_TAG_curr] = { ATOM_INIT_REG16(MAX77779_FG_Current)},
	[MAXFG_TAG_mcap] = { ATOM_INIT_REG16(MAX77779_FG_MixCap)},
	[MAXFG_TAG_vfsoc] = { ATOM_INIT_REG16(MAX77779_FG_VFSOC)},
	[MAXFG_TAG_tempco] = { ATOM_INIT_REG16(MAX77779_FG_NVM_nTempCo)},
	[MAXFG_TAG_rcomp0] = { ATOM_INIT_REG16(MAX77779_FG_NVM_nRComp0)},
	[MAXFG_TAG_timerh] = { ATOM_INIT_REG16(MAX77779_FG_TimerH)},
	[MAXFG_TAG_descap] = { ATOM_INIT_REG16(MAX77779_FG_DesignCap)},
	[MAXFG_TAG_fcnom] = { ATOM_INIT_REG16(MAX77779_FG_FullCapNom)},
	[MAXFG_TAG_fcrep] = { ATOM_INIT_REG16(MAX77779_FG_FullCapRep)},
	[MAXFG_TAG_msoc] = { ATOM_INIT_REG16(MAX77779_FG_MixSOC)},
	[MAXFG_TAG_mmdt] = { ATOM_INIT_REG16(MAX77779_FG_MaxMinTemp)},
	[MAXFG_TAG_mmdc] = { ATOM_INIT_REG16(MAX77779_FG_MaxMinCurr)},
	[MAXFG_TAG_repsoc] = { ATOM_INIT_REG16(MAX77779_FG_RepSOC)},
	[MAXFG_TAG_avcap] = { ATOM_INIT_REG16(MAX77779_FG_AvCap)},
	[MAXFG_TAG_repcap] = { ATOM_INIT_REG16(MAX77779_FG_RepCap)},
	[MAXFG_TAG_fulcap] = { ATOM_INIT_REG16(MAX77779_FG_FullCap)},
	[MAXFG_TAG_qh0] = { ATOM_INIT_REG16(MAX77779_FG_QH0)},
	[MAXFG_TAG_qh] = { ATOM_INIT_REG16(MAX77779_FG_QH)},
	[MAXFG_TAG_dqacc] = { ATOM_INIT_REG16(MAX77779_FG_dQAcc)},
	[MAXFG_TAG_dpacc] = { ATOM_INIT_REG16(MAX77779_FG_dPAcc)},
	[MAXFG_TAG_qresd] = { ATOM_INIT_REG16(MAX77779_FG_QResidual)},
	[MAXFG_TAG_fstat] = { ATOM_INIT_REG16(MAX77779_FG_FStat)},
	[MAXFG_TAG_learn] = { ATOM_INIT_REG16(MAX77779_FG_LearnCfg)},
	[MAXFG_TAG_filcfg] = { ATOM_INIT_REG16(MAX77779_FG_NVM_nFilterCfg)},
	[MAXFG_TAG_vfcap] = { ATOM_INIT_REG16(MAX77779_FG_VFRemCap)},
	[MAXFG_TAG_cycles] = { ATOM_INIT_REG16(MAX77779_FG_Cycles)},
	[MAXFG_TAG_rslow] = { ATOM_INIT_REG16(MAX77779_FG_RSlow)},
	[MAXFG_TAG_vfocv] = { ATOM_INIT_REG16(MAX77779_FG_VFOCV)},
	[MAXFG_TAG_avgt] = { ATOM_INIT_REG16(MAX77779_FG_AvgTA)},
	[MAXFG_TAG_avgv] = { ATOM_INIT_REG16(MAX77779_FG_AvgVCell)},
	[MAXFG_TAG_mixcap] = { ATOM_INIT_REG16(MAX77779_FG_MixCap)},
	[MAXFG_TAG_vfremcap] = { ATOM_INIT_REG16(MAX77779_FG_VFRemCap)},
	[MAXFG_TAG_vfsoc0] = { ATOM_INIT_REG16(MAX77779_FG_VFSOC0)},
	[MAXFG_TAG_qrtable00] = { ATOM_INIT_REG16(MAX77779_FG_QRTable00)},
	[MAXFG_TAG_qrtable10] = { ATOM_INIT_REG16(MAX77779_FG_QRTable10)},
	[MAXFG_TAG_qrtable20] = { ATOM_INIT_REG16(MAX77779_FG_QRTable20)},
	[MAXFG_TAG_qrtable30] = { ATOM_INIT_REG16(MAX77779_FG_QRTable30)},
	[MAXFG_TAG_status] = { ATOM_INIT_REG16(MAX77779_FG_Status)},
	[MAXFG_TAG_fullsocthr] = { ATOM_INIT_REG16(MAX77779_FG_FullSocThr)},
	[MAXFG_TAG_misccfg] = { ATOM_INIT_REG16(MAX77779_FG_MiscCfg)},
};

static const struct maxfg_reg max77779_debug_fg[] = {
	[MAXFG_TAG_tempco] = { ATOM_INIT_REG16(MAX77779_FG_NVM_nTempCo)},
	[MAXFG_TAG_rcomp0] = { ATOM_INIT_REG16(MAX77779_FG_NVM_nRComp0)},
	[MAXFG_TAG_filcfg] = { ATOM_INIT_REG16(MAX77779_FG_NVM_nFilterCfg)},
	[MAXFG_TAG_relaxcfg] = { ATOM_INIT_REG16(MAX77779_FG_NVM_RelaxCFG)},
};

struct max77779_fg_chip {
	struct device *dev;
	struct i2c_client *secondary;
	struct device *pmic_dev;

	int irq;

	struct maxfg_regmap regmap;
	struct maxfg_regmap regmap_debug;
	struct power_supply *psy;
	struct delayed_work init_work;
	struct device_node *batt_node;

	u16 devname;

	/* config */
	void *model_data;
	struct mutex model_lock;
	struct delayed_work model_work;
	int model_next_update;
	/* also used to restore model state from permanent storage */
	u16 reg_prop_capacity_raw;
	int model_reload;
	bool model_ok;		/* model is running */

	int fake_battery;

	u16 RSense;
	u16 RConfig;

	int batt_id;
	int batt_id_defer_cnt;
	int cycle_count;
	u16 eeprom_cycle;
	u16 designcap;

	bool init_complete;
	bool resume_complete;
	bool irq_disabled;
	u16 health_status;
	int fake_capacity;
	int previous_qh;
	int current_capacity;
	int prev_charge_status;
	char serial_number[30];
	bool offmode_charger;
	bool por;

	unsigned int debug_irq_none_cnt;

	/* Capacity Estimation */
	struct gbatt_capacity_estimation cap_estimate;
	struct logbuffer *ce_log;

	/* Dynamic Relax */
	struct maxfg_dynrel_state dynrel_state;

	/* debug interface, register to read or write */
	u32 debug_reg_address;
	u32 debug_dbg_reg_address;

	/* dump data to logbuffer periodically */
	struct logbuffer *monitor_log;
	u16 pre_repsoc;

	struct gbms_desc max77779_fg_psy_desc;

	int bhi_fcn_count;
	int bhi_acim;

	/* battery current criteria for report status charge */
	u32 status_charge_threshold_ma;

	bool current_offset_check_done;

	bool fw_update_mode;

	/* in-field logging */
	unsigned int abnormal_event_bits;
	u16 last_fullcapnom;
	struct mutex check_event_lock;

	/* firmware revision */
	int fw_rev;
	int fw_sub_rev;

	/* total number of model loading attempts counter since boot */
	int ml_cnt;
	/* total number of model loading failures since boot */
	int ml_fails;

	/* buffer for recording learning history */
	struct maxfg_capture_buf cb_lh;

	/* get suspend/resume notification */
	struct mutex save_data_lock;
	struct wakeup_source *fg_wake_lock;

	struct delayed_work stuck_monitor_work;
	int fg_stuck_count;
	u16 timer;

	/* mutex lock to access FG USR reg */
	struct mutex usr_lock;

	/* AAFV: Aged Adjusted Float Voltage */
	int aafv;
	int aafv_config_limits;
	int aafv_cur_idx;
	bool aafv_modified_fus;
	struct aafv_fg_config aafv_cfgs[GBMS_AAFV_DATA_MAX];
};

/** ------------------------------------------------------------------------ */

/*
 * Custom parameters are updated while the device is running.
 * NOTE: a subset (model_state_save) is saved to permanent storage every "n"
 * cycles and restored when the model is reloaded (usually on POR).
 * TODO: handle switching between RC1 and RC2 model types.
 */
struct max77779_custom_parameters {
	u16 nvcfg0;
	u16 relaxcfg;
	u16 learncfg;
	u16 config;
	u16 config2;
	u16 fullsocthr;
	u16 fullcaprep; /* WV */
	u16 designcap;
	u16 dpacc;	/* WV */
	u16 fullcapnom;	/* WV */
	u16 v_empty;
	u16 qresidual00;	/* WV */
	u16 qresidual10;	/* WV */
	u16 qresidual20;	/* WV */
	u16 qresidual30;	/* WV */
	u16 rcomp0;	/* WV */
	u16 tempco;	/* WV */
	u16 ichgterm;
	u16 misccfg;	/* 0x9d0 for internal current sense, 0x8d0 external */
	u16 modelcfg;
	u16 thermcfg;
	u16 filtercfg;
} __attribute__((packed));

/* this is what is saved and restored to/from GMSR */
struct model_state_save {
	u16 qrtable00;
	u16 qrtable10;
	u16 qrtable20;
	u16 qrtable30;
	u16 fullcapnom;
	u16 fullcaprep;
	u16 rcomp0;
	u16 tempco;
	u16 cycles;
	u8 padding[4]; /* keep the same size as 59 for consistency GBMS_GMSR_LEN */
	u8 crc;
} __attribute__((packed));

struct max77779_model_data {
	struct device *dev;
	struct maxfg_regmap *regmap;
	struct maxfg_regmap *debug_regmap;

	/* initial parameters are in device tree they are also learned */
	struct max77779_custom_parameters parameters;
	u16 cycles;
	u16 cv_mixcap;
	u16 hibcfg;

	int custom_model_size;
	u16 *custom_model;
	u32 model_version;
	bool force_reset_model_data;

	/* to/from GMSR */
	struct model_state_save model_save;
};

/** ------------------------------------------------------------------------ */

int max77779_model_read_version(const struct max77779_model_data *model_data);
int max77779_model_write_version(const struct max77779_model_data *model_data, int version);
int max77779_model_get_cap_lsb(const struct max77779_model_data *model_data);
int max77779_reset_state_data(struct max77779_model_data *model_data);
int max77779_needs_reset_model_data(const struct max77779_model_data *model_data);
u16 max77779_get_designcap(const struct max77779_model_data *model_data);
u16 max77779_get_relaxcfg(const struct max77779_model_data *model_data);
void max77779_model_apply_aaf_fullsoc(struct max77779_model_data *model_data,
				      const struct aafv_fg_config *cfg);

/*
 * max77779 might use the low 8 bits of devname to keep the model version number
 * - 0 not M5, !=0 M5
 */
static inline int max77779_check_devname(u16 devname)
{
	const u16 radix = devname >> 8;

	return radix == 0x62 || radix == 0x63 || radix == 0x51;
}

static inline int max77779_fg_model_version(const struct max77779_model_data *model_data)
{
	return model_data ? model_data->model_version : MAX77779_FG_INVALID_VERSION;
}

/*
 * 0 reload, != 0 no reload
 * always reload when the model version is not specified
 */
static inline int max77779_fg_model_check_version(const struct max77779_model_data *model_data)
{
	/* if model version is invalid, no reload */
	if (!model_data || model_data->model_version == MAX77779_FG_INVALID_VERSION)
		return 1;

	return max77779_model_read_version(model_data) == model_data->model_version;
}

enum max77779_fg_reg_sections {
	MAX77779_FG_RAM_SECTION,
	MAX77779_FG_FUNC_SECTION,
	MAX77779_FG_NVM_SECTION,
	MAX77779_FG_ALL_SECTION,
	MAX77779_FG_UNKNOWN_SECTION,
};

/* TODO: b/325642439 add protection during model loading and firmware update */
#define MAX77779_FG_REGMAP_WRITE(regmap, what, value) \
	max77779_fg_register_write(regmap, what, value, false)
#define MAX77779_FG_REGMAP_WRITE_VERIFY(regmap, what, value) \
	max77779_fg_register_write(regmap, what, value, true)

#define MAX77779_FG_N_REGMAP_WRITE(regmap, nregmap, what, value) \
	max77779_fg_nregister_write(regmap, nregmap, what, value, false)
#define MAX77779_FG_N_REGMAP_WRITE_VERIFY(regmap, nregmap, what, value) \
	max77779_fg_nregister_write(regmap, nregmap, what, value, true)


/** ------------------------------------------------------------------------ */

int max77779_fg_usr_lock_section(const struct maxfg_regmap *map, enum max77779_fg_reg_sections section, bool enabled);
int max77779_fg_register_write(const struct maxfg_regmap *regmap, unsigned int reg,
			       u16 value, bool verify);
int max77779_fg_nregister_write(const struct maxfg_regmap *map,
				const struct maxfg_regmap *debug_map,
				unsigned int reg, u16 value, bool verify);
void *max77779_init_data(struct device *dev, struct device_node *batt_node,
			 struct maxfg_regmap *regmap, struct maxfg_regmap *debug_regmap);
void max77779_free_data(struct max77779_model_data *model_data);

int max77779_load_state_data(struct max77779_model_data *model_data);
int max77779_save_state_data(struct max77779_model_data *model_data);

/* read state from the gauge */
int max77779_model_read_state(struct max77779_model_data *model_data);
int max77779_model_check_state(struct max77779_model_data *model_data);

/* load model to gauge */
int max77779_load_gauge_model(struct max77779_model_data *model_data, int fw_rev, int fw_sub_rev);

ssize_t max77779_model_state_cstr(char *buf, int max, struct max77779_model_data *model_data);
int max77779_fg_param_cstr(char *buf, int max, const struct max77779_model_data *model_data);
int max77779_fg_param_sscan(struct max77779_model_data *model_data, const char *buf, int max);
int max77779_fg_model_cstr(char *buf, int max, const struct max77779_model_data *model_data);
int max77779_fg_model_sscan(struct max77779_model_data *model_data, const char *buf, int max);
bool max77779_fg_check_state(struct max77779_model_data *model_data);

/* read saved value */
ssize_t max77779_gmsr_state_cstr(char *buf, int max);

/** ------------------------------------------------------------------------ */

void *max77779_get_model_data(struct device *dev);

int max77779_fg_init(struct max77779_fg_chip *chip);
bool max77779_fg_dbg_is_reg(struct device *dev, unsigned int reg);
bool max77779_fg_is_reg(struct device *dev, unsigned int reg);
void max77779_fg_remove(struct max77779_fg_chip *chip);

#if IS_ENABLED(CONFIG_PM)
int max77779_fg_pm_suspend(struct device *dev);
int max77779_fg_pm_resume(struct device *dev);
#endif
#endif
