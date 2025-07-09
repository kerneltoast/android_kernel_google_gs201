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

#ifndef MAX_M5_H_
#define MAX_M5_H_

#include "max1720x_battery.h"
#include "max_m5_reg.h"

#define MAX_M5_I2C_ADDR 0x6C


/* change to 1 or 0 to load FG model with default parameters on startup */
#define MAX_M5_LOAD_MODEL_DISABLED	-1
#define MAX_M5_LOAD_MODEL_IDLE		0
#define MAX_M5_LOAD_MODEL_REQUEST	1

#define MAX_M5_FG_MODEL_START		0x80
#define MAX_M5_FG_MODEL_SIZE		48

#define MAX_M5_UNLOCK_EXTRA_CONFIG	0x60
#define MAX_M5_UNLOCK_EXTRA_CONFIG_UNLOCK_CODE	0x80
#define MAX_M5_UNLOCK_EXTRA_CONFIG_LOCK_CODE	0x00

#define MAX_M5_UNLOCK_MODEL_ACCESS	0x62
#define MAX_M5_MODEL_ACCESS_UNLOCK_CODE	0xc459
#define MAX_M5_MODEL_ACCESS_LOCK_CODE	0x0000
#define MAX_M5_MODEL_ACCESS_LOCK_OK	0xFFFF

#define MAX_M5_TCURVE	0xB9
#define MAX_M5_VFOCV	0xFB
#define MAX_M5_VFSOC	0xFF

#define MAX_M5_COMMAND	0x60
#define MAX_M5_COMMAND_HARDWARE_RESET 0x000F

/* model version */
#define MAX_M5_INVALID_VERSION	-1

#define MAX_M5_RECAL_MAX_ROUNDS	3

#define MAX_M5_RETRY_TIMES 	3

#define MAX_M5_COTRIM	0xEB

/* AAFV */
#define MAX_M5_AAFV_DEFAULT_FULLSOCTHR	95
#define MAX_M5_AAFV_DEFAULT_FUS		0x0
#define MAX_M5_AAFV_RESTORE_FUS		0x3

/** ------------------------------------------------------------------------ */

/*
 * Custom parameters are updated while the device is runnig.
 * NOTE: a subset (model_state_save) is saved to permanent storage every "n"
 * cycles and restored when the model is reloaded (usually on POR).
 * TODO: handle switching between RC1 and RC2 model types.
 */
struct max_m5_custom_parameters {
	u16 iavg_empty; /* WV */
	u16 relaxcfg;
	u16 learncfg;
	u16 config;
	u16 config2;
	u16 fullsocthr;
	u16 fullcaprep; /* WV */
	u16 designcap;
	u16 dpacc;	/* WV */
	u16 dqacc;	/* WV */
	u16 fullcapnom;	/* WV */
	u16 v_empty;
	u16 qresidual00;	/* WV */
	u16 qresidual10;	/* WV */
	u16 qresidual20;	/* WV */
	u16 qresidual30;	/* WV */
	u16 rcomp0;	/* WV */
	u16 tempco;	/* WV */
	u16 ichgterm;
	u16 tgain;
	u16 toff;
	u16 tcurve; 	/* write to 0x00B9 */
	u16 misccfg;	/* 0x9d0 for internal current sense, 0x8d0 external */

	u16 atrate;
	u16 convgcfg;
	u16 filtercfg; 	/* write to 0x0029 */
	u16 taskperiod;
	u16 cgain;
} __attribute__((packed));

/* this is what is saved and restored to/from GMSR */
struct model_state_save {
	u16 rcomp0;
	u16 tempco;
	u16 fullcaprep;
	u16 cycles;
	u16 fullcapnom;
	u16 qresidual00;
	u16 qresidual10;
	u16 qresidual20;
	u16 qresidual30;
	u16 cv_mixcap;
	u16 halftime;
	u8 crc;
} __attribute__((packed));

struct max_m5_recalibration_data {
	int state;
	int rounds;
	int base_cycle_reg;
	u16 target_cap;
	struct mutex lock;
};

struct max_m5_data {
	struct device *dev;
	struct maxfg_regmap *regmap;
	int cap_lsb;	/* b/177099997 */

	/* initial parameters are in device tree they are also learned */
	struct max_m5_custom_parameters parameters;
	u16 cycles;
	u16 cv_mixcap;
	u16 halftime;

	int custom_model_size;
	u16 *custom_model;
	u32 model_version;
	bool force_reset_model_data;
	int load_retry;

	/* to/from GMSR */
	struct model_state_save model_save;

	/* recalibration */
	struct max_m5_recalibration_data recal;
};

enum max_m5_re_cal_state {
	RE_CAL_STATE_IDLE = 0,
	RE_CAL_STATE_FG_RESET = 1,
	RE_CAL_STATE_LEARNING = 2,
};

enum max_m5_re_cal_algo {
	RE_CAL_ALGO_0 = 0,
	RE_CAL_ALGO_1 = 1,
};

/** ------------------------------------------------------------------------ */

int max_m5_model_read_version(const struct max_m5_data *m5_data);
int max_m5_model_write_version(const struct max_m5_data *m5_data, int version);
int max_m5_model_get_cap_lsb(const struct max_m5_data *m5_data);
int max_m5_reset_state_data(struct max_m5_data *m5_data);
int max_m5_needs_reset_model_data(const struct max_m5_data *m5_data);
int max_m5_recal_state(const struct max_m5_data *m5_data);
int max_m5_recal_cycle(const struct max_m5_data *m5_data);
int max_m5_recalibration(struct max_m5_data *m5_data, int algo, u16 cap);
int max_m5_check_recal_state(struct max_m5_data *m5_data, int algo, u16 eeprom_cycle);
int m5_init_custom_parameters(struct device *dev, struct max_m5_data *m5_data,
			      struct device_node *node);
u16 max_m5_get_designcap(const struct max_m5_data *m5_data);
int max_m5_model_lock(struct regmap *regmap, bool enabled);

/*
 * max_m5 might use the low 8 bits of devname to keep the model version number
 * - 0 not M5, !=0 M5
 */
static inline int max_m5_check_devname(u16 devname)
{
	const u16 radix = devname >> 8;

	return radix == 0x62 || radix == 0x63;
}

/* b/177099997, handle TaskConfig = 351 */
static inline int max_m5_cap_lsb(const struct max_m5_data *m5_data)
{
	return m5_data ? (1 << m5_data->cap_lsb) : 1;
}

static inline int max_m5_fg_model_version(const struct max_m5_data *m5_data)
{
	return m5_data ? m5_data->model_version : MAX_M5_INVALID_VERSION;
}

/*
 * 0 reload, != 0 no reload
 * always reload when the model version is not specified
 */
static inline int max_m5_fg_model_check_version(const struct max_m5_data *m5_data)
{
	if (!m5_data)
		return 1;
	if (m5_data->model_version == MAX_M5_INVALID_VERSION)
		return 0;

	return max_m5_model_read_version(m5_data) == m5_data->model_version;
}

/** ------------------------------------------------------------------------ */

int max_m5_regmap_init(struct maxfg_regmap *regmap,
		       struct i2c_client *primary);

void *max_m5_init_data(struct device *dev, struct device_node *batt_node,
		       struct maxfg_regmap *regmap);
void max_m5_free_data(struct max_m5_data *m5_data);

int max_m5_load_state_data(struct max_m5_data *m5_data);
int max_m5_save_state_data(struct max_m5_data *m5_data);

/* read state from the gauge */
int max_m5_model_read_state(struct max_m5_data *m5_data);
int max_m5_model_check_state(struct max_m5_data *m5_data);

/* load model to gauge */
int max_m5_load_gauge_model(struct max_m5_data *m5_data);

int max_m5_fixup_outliers(struct max1720x_drift_data *ddata,
			  struct max_m5_data *m5_data);

ssize_t max_m5_model_state_cstr(char *buf, int max,
				struct max_m5_data *m5_data);
int max_m5_model_state_sscan(struct max_m5_data *m5_data, const char *buf,
			     int max);
int max_m5_fg_model_sscan(struct max_m5_data *m5_data, const char *buf,
			  int max);
int max_m5_fg_model_cstr(char *buf, int max, const struct max_m5_data *m5_data);
int max_m5_get_rc_switch_param(struct max_m5_data *m5_data, u16 *rc2_tempco, u16 *rc2_learncfg);

/* read saved value */
ssize_t max_m5_gmsr_state_cstr(char *buf, int max);

void max_m5_model_apply_aaf_fullsoc(struct max_m5_data *m5_data, const struct aafv_fg_config *cfg);

/** ------------------------------------------------------------------------ */

/*
 *
 */
#if IS_ENABLED(CONFIG_MAX_M5)

extern int max_m5_read_actual_input_current_ua(struct i2c_client *client,
					       int *iic);
extern int max_m5_read_vbypass(struct i2c_client *client,
					       int *volt);

extern int max_m5_reg_read(struct i2c_client *client, unsigned int reg,
		    unsigned int *val);
extern int max_m5_reg_write(struct i2c_client *client, unsigned int reg,
		     unsigned int val);
#else
static inline int
max_m5_read_actual_input_current_ua(struct i2c_client *client, int *iic)
{
	return -ENODEV;
}

static inline int
max_m5_read_vbypass(struct i2c_client *client, int *volt)
{
	return -ENODEV;
}

static inline int
max_m5_reg_read(struct i2c_client *client, unsigned int reg, unsigned int *val)
{
	return -ENODEV;
}

static inline int max_m5_reg_write(struct i2c_client *client, unsigned int reg,
				   unsigned int val)
{
	return -ENODEV;
}


#endif



/* reach back into max1720x battery */
void *max1720x_get_model_data(struct i2c_client *client);


#endif
