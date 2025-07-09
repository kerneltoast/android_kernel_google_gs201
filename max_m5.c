/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Fuel gauge driver for Maxim Fuel Gauges with M5 Algo
 *
 * Copyright (C) 2018 Google Inc.
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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/crc8.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include "google_bms.h"
#include "google_psy.h"

#include "max_m5.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

/* Config2: must not enable TAlert */
#define MODEL_VERSION_REG	MAX_M5_TALRTTH
#define MODEL_VERSION_SHIFT	8
#define MODEL_VERSION_MASK	0xff

#define MAX_M5_TASKPERIOD_175MS	0x1680
#define MAX_M5_TASKPERIOD_351MS	0x2D00

#define MAX_M5_CRC8_POLYNOMIAL		0x07	/* (x^8) + x^2 + x + 1 */
DECLARE_CRC8_TABLE(m5_crc8_table);

int max_m5_recalibration(struct max_m5_data *m5_data, int algo, u16 cap);

/* input current is in the fuel gauge */
int max_m5_read_actual_input_current_ua(struct i2c_client *client, int *iic_raw)
{
	struct max_m5_data *m5_data = max1720x_get_model_data(client);
	unsigned long sum = 0;
	const int loops = 4;
	unsigned int tmp;
	int i, rtn;

	if (!m5_data || !m5_data->regmap)
		return -ENODEV;

	for (i = 0; i < loops; i++) {
		rtn = regmap_read(m5_data->regmap->regmap, MAX_M5_IIN, &tmp);
		if (rtn) {
			pr_err("Failed to read %x\n", MAX_M5_IIN);
			return rtn;
		}

		sum += tmp;
	}

	*iic_raw  = sum / loops;
	return 0;
}
EXPORT_SYMBOL_GPL(max_m5_read_actual_input_current_ua);

int max_m5_read_vbypass(struct i2c_client *client, int *volt)
{
	struct max_m5_data *m5_data = max1720x_get_model_data(client);
	unsigned int tmp;
	int ret;

	if (!m5_data || !m5_data->regmap)
		return -ENODEV;

	ret = regmap_read(m5_data->regmap->regmap, MAX_M5_VBYP, &tmp);
	if (ret) {
		pr_err("Failed to read %x\n", MAX_M5_VBYP);
		return ret;
	}

	/* LSB: 0.427246mV */
	*volt = div_u64((u64) tmp * 427246, 1000);
	return 0;
}
EXPORT_SYMBOL_GPL(max_m5_read_vbypass);

int max_m5_reg_read(struct i2c_client *client, unsigned int reg,
		    unsigned int *val)
{
	struct max_m5_data *m5_data = max1720x_get_model_data(client);

	if (!m5_data || !m5_data->regmap)
		return -ENODEV;

	return regmap_read(m5_data->regmap->regmap, reg, val);
}
EXPORT_SYMBOL_GPL(max_m5_reg_read);

int max_m5_reg_write(struct i2c_client *client, unsigned int reg,
		    unsigned int val)
{
	struct max_m5_data *m5_data = max1720x_get_model_data(client);

	if (!m5_data || !m5_data->regmap)
		return -ENODEV;

	return regmap_write(m5_data->regmap->regmap, reg, val);
}
EXPORT_SYMBOL_GPL(max_m5_reg_write);


static int max_m5_read_custom_model(struct regmap *regmap, u16 *model_data,
				    int count)
{
	return regmap_raw_read(regmap, MAX_M5_FG_MODEL_START, model_data,
			       count * 2);
}

static int max_m5_write_custom_model(struct regmap *regmap, u16 *model_data,
				     int count)
{
	return regmap_raw_write(regmap, MAX_M5_FG_MODEL_START, model_data,
				count * 2);
}

int max_m5_model_lock(struct regmap *regmap, bool enabled)
{
	u16 code[2] = {0x59, 0xC4};

	if (enabled) {
		code[0] = 0;
		code[1] = 0;
	}

	return regmap_raw_write(regmap, MAX_M5_UNLOCK_MODEL_ACCESS, code,
				sizeof(code));
}

static int mem16test(u16 *data, u16 code, int count)
{
	int same, i;

	for (i = 0, same = 1; same && i < count; i++)
		same = data[i] == code;

	return same;
}

/* load custom model b/137037210 */
static int max_m5_update_custom_model(struct max_m5_data *m5_data)
{
	int retries, ret;
	bool success;
	u16 *data;

	data = kzalloc(m5_data->custom_model_size * 2, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* un lock, update the model */
	for (success = false, retries = 3; !success && retries > 0; retries--) {

		ret = max_m5_model_lock(m5_data->regmap->regmap, false);
		if (ret < 0) {
			dev_err(m5_data->dev, "cannot unlock model access (%d)\n",
				ret);
			continue;
		}

		ret = max_m5_write_custom_model(m5_data->regmap->regmap,
						m5_data->custom_model,
						m5_data->custom_model_size);
		if (ret < 0) {
			dev_err(m5_data->dev, "cannot write custom model (%d)\n",
				ret);
			continue;
		}

		ret = max_m5_read_custom_model(m5_data->regmap->regmap,
					       data,
					       m5_data->custom_model_size);
		if (ret < 0) {
			dev_err(m5_data->dev, "cannot write custom model (%d)\n",
				ret);
			continue;
		}

		ret = memcmp(m5_data->custom_model, data,
			     m5_data->custom_model_size * 2);
		success = ret == 0;
		if (!success) {
			dump_model(m5_data->dev, MAX_M5_FG_MODEL_START, m5_data->custom_model,
				   m5_data->custom_model_size);
			dump_model(m5_data->dev, MAX_M5_FG_MODEL_START, data,
				   m5_data->custom_model_size);
		}

	}

	if (!success) {
		dev_err(m5_data->dev, "cannot write custom model (%d)\n", ret);
		kfree(data);
		return -EIO;
	}

	/* lock and verify lock */
	for (retries = 3; retries > 0; retries--) {
		int same;

		ret = max_m5_model_lock(m5_data->regmap->regmap, true);
		if (ret < 0) {
			dev_err(m5_data->dev, "cannot lock model access (%d)\n",
				ret);
			continue;
		}

		ret = max_m5_read_custom_model(m5_data->regmap->regmap, data,
					       m5_data->custom_model_size);
		if (ret < 0) {
			dev_err(m5_data->dev, "cannot read custom model (%d)\n",
				ret);
			continue;
		}

		/* model is locked when read retuns all 0xffff */
		same = mem16test(data, 0xffff, m5_data->custom_model_size);
		if (same)
			break;
	}

	kfree(data);
	return 0;
}

/* Step 7: Write custom parameters */
static int max_m5_update_custom_parameters(struct max_m5_data *m5_data)
{
	struct max_m5_custom_parameters *cp = &m5_data->parameters;
	struct maxfg_regmap *regmap = m5_data->regmap;
	int tmp, ret;
	u16 vfsoc;

	ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_REPCAP, 0x0);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_RELAXCFG,
					  cp->relaxcfg);
	if (ret < 0)
		return -EIO;

	ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_UNLOCK_EXTRA_CONFIG,
				  MAX_M5_UNLOCK_EXTRA_CONFIG_UNLOCK_CODE);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot unlock extra config (%d)\n", ret);
		return -EIO;
	}

	ret = REGMAP_READ(regmap, MAX_M5_VFSOC, &vfsoc);
	if (ret < 0)
		return ret;

	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_VFSOC0, vfsoc);

	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_LEARNCFG, cp->learncfg);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_CONFIG, cp->config);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_CONFIG2, cp->config2);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_FULLSOCTHR, cp->fullsocthr);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_FULLCAPREP,
					  cp->fullcaprep);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_DESIGNCAP,
					  cp->designcap);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_DPACC, cp->dpacc);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_DQACC, cp->dqacc);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_FULLCAPNOM,
					  cp->fullcapnom);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_VEMPTY, cp->v_empty);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_QRTABLE00,
					  cp->qresidual00);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_QRTABLE10,
					  cp->qresidual10);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_QRTABLE20,
					  cp->qresidual20);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_QRTABLE30,
					  cp->qresidual30);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_RCOMP0, cp->rcomp0);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_TEMPCO, cp->tempco);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_TASKPERIOD, cp->taskperiod);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_ICHGTERM, cp->ichgterm);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_TGAIN, cp->tgain);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_TOFF, cp->toff);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_MISCCFG, cp->misccfg);
	if (ret < 0)
		goto exit_done;

	ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_UNLOCK_EXTRA_CONFIG,
				  MAX_M5_UNLOCK_EXTRA_CONFIG_UNLOCK_CODE);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot unlock extra config (%d)\n", ret);
		goto exit_done;
	}

	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_ATRATE, cp->atrate);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_CV_MIXCAP,
					  (cp->fullcapnom * 75) / 100);
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_CV_HALFTIME, 0x600);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_CONVGCFG, cp->convgcfg);

exit_done:
	tmp = REGMAP_WRITE_VERIFY(regmap, MAX_M5_UNLOCK_EXTRA_CONFIG,
				  MAX_M5_UNLOCK_EXTRA_CONFIG_LOCK_CODE);
	if (tmp < 0) {
		dev_err(m5_data->dev, "cannot lock extra config (%d)\n", tmp);
		return tmp;
	}

	return ret;
}

int max_m5_model_read_version(const struct max_m5_data *m5_data)
{
	u16 version;
	int ret;

	if (!m5_data)
		return -EINVAL;

	ret = REGMAP_READ(m5_data->regmap, MODEL_VERSION_REG, &version);
	if (ret < 0)
		return ret;

	return (version >> MODEL_VERSION_SHIFT) & MODEL_VERSION_MASK;
}

int max_m5_model_write_version(const struct max_m5_data *m5_data, int version)
{
	u16 temp;
	int ret;

	if (!m5_data)
		return -EINVAL;

	if (version == MAX_M5_INVALID_VERSION)
		return 0;

	ret = REGMAP_READ(m5_data->regmap, MODEL_VERSION_REG, &temp);
	if (ret == 0) {
		temp &= ~(MODEL_VERSION_MASK << MODEL_VERSION_SHIFT);
		temp |= (version & MODEL_VERSION_MASK) << MODEL_VERSION_SHIFT;

		ret =  REGMAP_WRITE(m5_data->regmap, MODEL_VERSION_REG, temp);
	}

	return ret;
}

static int max_m5_model_read_rc(const struct max_m5_data *m5_data)
{
	u16 learncfg;
	int ret;

	ret = REGMAP_READ(m5_data->regmap, MAX_M5_LEARNCFG, &learncfg);
	if (ret < 0)
		return ret;

	return (learncfg & MAX_M5_LEARNCFG_RC_VER);
}

int max_m5_reset_state_data(struct max_m5_data *m5_data)
{
	struct model_state_save data;
	int ret = 0;

	if (!m5_data)
		return -EINVAL;

	memset(&data, 0xff, sizeof(data));

	ret = gbms_storage_write(GBMS_TAG_GMSR, &data, sizeof(data));
	if (ret < 0)
		dev_warn(m5_data->dev, "Erase GMSR fail (%d)\n", ret);

	return ret == sizeof(data) ? 0 : ret;
}

int max_m5_needs_reset_model_data(const struct max_m5_data *m5_data)
{
	int read_rc, para_rc;

	if (!m5_data)
		return 0;

	if (m5_data->force_reset_model_data)
		return 1;

	read_rc = max_m5_model_read_rc(m5_data);
	if (read_rc < 0)
		return 0;

	para_rc = m5_data->parameters.learncfg & MAX_M5_LEARNCFG_RC_VER;

	/* RC2 -> RC1 */
	if (read_rc == MAX_M5_LEARNCFG_RC2 && para_rc == MAX_M5_LEARNCFG_RC1)
		return 1;

	return 0;
}

/* convert taskperiod to the scaling factor for capacity */
static int max_m5_period2caplsb(u16 taskperiod)
{
	int cap_lsb = -EINVAL;

	if (taskperiod == MAX_M5_TASKPERIOD_351MS)
		cap_lsb = 1;
	else if (taskperiod == MAX_M5_TASKPERIOD_175MS)
		cap_lsb = 0;

	return cap_lsb;
}

static int max_m5_update_gauge_custom_parameters(struct max_m5_data *m5_data)
{
	struct maxfg_regmap *regmap = m5_data->regmap;
	int ret, retries, temp;
	u16 data;

	/* write parameters (which include state) */
	ret = max_m5_update_custom_parameters(m5_data);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot update custom parameters (%d)\n",
			ret);
		return ret;
	}

	/* tcurve, filterconfig, taskperiod, version are not part of model */
	ret = REGMAP_WRITE(regmap, MAX_M5_TCURVE, m5_data->parameters.tcurve);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot update tcurve (%d)\n", ret);
		return ret;
	}

	ret = REGMAP_WRITE(regmap, MAX_M5_FILTERCFG,
			   m5_data->parameters.filtercfg);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot update filter config (%d)\n", ret);
		return ret;
	}

	ret = REGMAP_WRITE(regmap, MAX_M5_CGAIN,
			   m5_data->parameters.cgain);
	if (ret < 0)
		dev_err(m5_data->dev, "cannot update cgain (%d)\n", ret);

	m5_data->cap_lsb = max_m5_period2caplsb(m5_data->parameters.taskperiod);

	/* trigger load model */
	ret = REGMAP_READ(regmap, MAX_M5_CONFIG2, &data);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX_M5_CONFIG2,
				   data | MAX_M5_CONFIG2_LDMDL);
	if (ret < 0) {
		dev_err(m5_data->dev, "failed start model loading (%d)\n", ret);
		return ret;
	}

	/* around 400ms for this usually */
	for (retries = 20; retries > 0; retries--) {

		mdelay(50);

		ret = REGMAP_READ(regmap, MAX_M5_CONFIG2, &data);
		if (ret == 0 && !(data & MAX_M5_CONFIG2_LDMDL)) {
			ret = REGMAP_READ(regmap, MAX_M5_REPCAP, &data);
			if (ret == 0 && data != 0)
				break;
		}
	}

	if (retries == 0)
		return -ETIMEDOUT;

	/*
	 * version could be in the DT: this will overwrite it if set.
	 * Invalid version is not written out.
	 */
	ret = max_m5_model_write_version(m5_data, m5_data->model_version);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot update version (%d)\n", ret);
		return ret;
	}

	temp = max_m5_model_read_version(m5_data);
	if (m5_data->model_version == MAX_M5_INVALID_VERSION) {
		dev_info(m5_data->dev, "No Model Version, Current %x\n", temp);
		return -EINVAL;
	}

	if (temp != m5_data->model_version) {
		dev_info(m5_data->dev, "Model Version %x, Mismatch %x\n",
			 m5_data->model_version, temp);
		return -EINVAL;
	}

	return 0;
}

/* protected from mutex_lock(&chip->model_lock) */
static int max_m5_check_model_parameters(struct max_m5_data *m5_data)
{
	struct max_m5_custom_parameters *cp = &m5_data->parameters;
	struct maxfg_regmap *regmap = m5_data->regmap;
	int ret, cap_delta_threshold, cap_delta_real;
	u16 fullcaprep, fullcapnom;

	/* b/240115405#comment44 */
	cap_delta_threshold = abs(cp->fullcapnom - cp->fullcaprep) + cp->designcap / 100;

	ret = REGMAP_READ(regmap, MAX_M5_FULLCAPREP, &fullcaprep);
	if (ret < 0)
		return ret;

	ret = REGMAP_READ(regmap, MAX_M5_FULLCAPNOM, &fullcapnom);
	if (ret < 0)
		return ret;

	cap_delta_real = abs(fullcapnom - fullcaprep);

	dev_info(m5_data->dev, "write: nom:%#x, rep:%#x, design:%#x (threshold=%d),"
		 " read: nom:%#x, rep:%#x (delta=%d), retry:%d\n",
		 cp->fullcapnom, cp->fullcaprep, cp->designcap, cap_delta_threshold,
		 fullcapnom, fullcaprep, cap_delta_real, m5_data->load_retry);

	if (cap_delta_real > cap_delta_threshold && m5_data->load_retry < MAX_M5_RETRY_TIMES)
		return -ERANGE;

	return 0;
}

/* 0 is ok , protected from mutex_lock(&chip->model_lock) in max7102x_battery.c */
int max_m5_load_gauge_model(struct max_m5_data *m5_data)
{
	struct maxfg_regmap *regmap = m5_data->regmap;
	int ret, retries;
	u16 data;

	if (!regmap)
		return -EIO;

	if (!m5_data || !m5_data->custom_model || !m5_data->custom_model_size)
		return -ENODATA;

	/* check FStat.DNR to wait it clear for data ready */
	for (retries = 20; retries > 0; retries--) {
		ret = REGMAP_READ(regmap, MAX_M5_FSTAT, &data);
		if (ret == 0 && !(data & MAX_M5_FSTAT_DNR))
			break;
		msleep(50);
	}
	dev_info(m5_data->dev, "retries:%d, FSTAT:%#x\n", retries, data);

	/* loading in progress, this is not good (tm) */
	ret = REGMAP_READ(regmap, MAX_M5_CONFIG2, &data);
	if (ret == 0 && (data & MAX_M5_CONFIG2_LDMDL)) {
		dev_err(m5_data->dev, "load model in progress (%x)\n", data);
		return -EINVAL;
	}

	ret = max_m5_update_custom_model(m5_data);
	if (ret < 0) {
		dev_err(m5_data->dev, "cannot update custom model (%d)\n", ret);
		return ret;
	}

	do {
		msleep(500);

		ret = max_m5_update_gauge_custom_parameters(m5_data);
		if (ret < 0)
			return ret;

		ret = max_m5_check_model_parameters(m5_data);
		if (ret < 0) {
			m5_data->load_retry++;
		} else {
			m5_data->load_retry = 0;
			break;
		}
	} while (m5_data->load_retry < MAX_M5_RETRY_TIMES);

	return ret;
}

/* algo version is ignored here, check code in max1720x_outliers */
int max_m5_fixup_outliers(struct max1720x_drift_data *ddata,
			  struct max_m5_data *m5_data)
{
	if (!ddata || !m5_data)
		return -EINVAL;

	if (ddata->design_capacity != m5_data->parameters.designcap)
		ddata->design_capacity = m5_data->parameters.designcap;
	if (ddata->ini_rcomp0 != m5_data->parameters.rcomp0)
		ddata->ini_rcomp0 = m5_data->parameters.rcomp0;
	if (ddata->ini_tempco != m5_data->parameters.tempco)
		ddata->ini_tempco = m5_data->parameters.tempco;

	return 0;
}

static bool memtst(void *buf, char c, size_t count)
{
	bool same = true;
	int i;

	for (i = 0; same && i < count; i++)
		same = ((char *)buf)[i] == c;

	return same;
}

/* TODO: make it adjustable, set 10% tolerance here */
#define MAX_M5_CAP_MAX_RATIO	110
static int max_m5_check_state_data(struct model_state_save *state,
				   struct max_m5_custom_parameters *ini)
{
	bool bad_residual, empty;
	int max_cap = ini->designcap * MAX_M5_CAP_MAX_RATIO / 100;

	empty = memtst(state, 0xff, sizeof(*state));
	if (empty)
		return -ENODATA;

	if (state->rcomp0 == 0xFF)
		return -ERANGE;

	if (state->tempco == 0xFFFF)
		return -ERANGE;

	bad_residual = state->qresidual00 == 0xffff &&
		       state->qresidual10 == 0xffff &&
		       state->qresidual20 == 0xffff &&
		       state->qresidual30 == 0xffff;

	if (bad_residual)
		return -EINVAL;

	if (state->fullcaprep > max_cap)
		return -ERANGE;

	if (state->fullcapnom > max_cap)
		return -ERANGE;

	return 0;
}

static u8 max_m5_crc(u8 *pdata, size_t nbytes, u8 crc)
{
	return crc8(m5_crc8_table, pdata, nbytes, crc);
}

static u8 max_m5_data_crc(char *reason, struct model_state_save *state)
{
	u8 crc;

	/* Last byte is for saving CRC */
	crc = max_m5_crc((u8 *)state, sizeof(struct model_state_save) - 1,
			  CRC8_INIT_VALUE);

	pr_info("%s gmsr: %X %X %X %X %X %X %X %X %X %X %X %X (%X)\n",
		reason, state->rcomp0, state->tempco,
		state->fullcaprep, state->fullcapnom,
		state->qresidual00, state->qresidual10,
		state->qresidual20, state->qresidual30,
		state->cycles, state->cv_mixcap,
		state->halftime, state->crc, crc);

	return crc;
}

/*
 * Load parameters and model state from permanent storage.
 * Called on boot after POR
 */
int max_m5_load_state_data(struct max_m5_data *m5_data)
{
	struct max_m5_custom_parameters *cp = &m5_data->parameters;
	u8 crc;
	int ret;

	if (!m5_data)
		return -EINVAL;

	/* might return -EAGAIN during init */
	ret = gbms_storage_read(GBMS_TAG_GMSR, &m5_data->model_save,
				sizeof(m5_data->model_save));
	if (ret < 0) {
		dev_info(m5_data->dev, "Load Model Data Failed ret=%d\n", ret);
		return ret;
	}

	ret = max_m5_check_state_data(&m5_data->model_save, cp);
	if (ret < 0)
		return ret;

	crc = max_m5_data_crc("restore", &m5_data->model_save);
	if (crc != m5_data->model_save.crc)
		return -EINVAL;

	cp->rcomp0 = m5_data->model_save.rcomp0;
	cp->tempco = m5_data->model_save.tempco;
	cp->fullcaprep = m5_data->model_save.fullcaprep;
	cp->fullcapnom = m5_data->model_save.fullcapnom;
	cp->qresidual00 = m5_data->model_save.qresidual00;
	cp->qresidual10 = m5_data->model_save.qresidual10;
	cp->qresidual20 = m5_data->model_save.qresidual20;
	cp->qresidual30 = m5_data->model_save.qresidual30;
	/* b/278492168 restore dpacc with fullcapnom for taskperiod=351ms */
	if (cp->taskperiod == 0x2d00 && cp->dpacc == 0x3200)
		cp->dqacc = cp->fullcapnom >> 2;
	else if (cp->taskperiod == 0x2d00 && cp->dpacc == 0x0c80)
		cp->dqacc = cp->fullcapnom >> 4;
	else
		dev_warn(m5_data->dev, "taskperiod:%#x, dpacc:%#x, dqacc:%#x\n",
			 cp->taskperiod, cp->dpacc, cp->dqacc);

	m5_data->cycles = m5_data->model_save.cycles;
	m5_data->cv_mixcap = m5_data->model_save.cv_mixcap;
	m5_data->halftime = m5_data->model_save.halftime;

	return 0;
}

/* save/commit parameters and model state to permanent storage */
int max_m5_save_state_data(struct max_m5_data *m5_data)
{
	struct max_m5_custom_parameters *cp = &m5_data->parameters;
	struct model_state_save rb;
	u16 learncfg;
	int ret = 0;

	if (!m5_data)
		return -EINVAL;

	/* Do not save when in RC1 stage b/213425610 */
	ret = REGMAP_READ(m5_data->regmap, MAX_M5_LEARNCFG, &learncfg);
	if (ret < 0)
		return ret;

	if ((learncfg & MAX_M5_LEARNCFG_RC_VER) == MAX_M5_LEARNCFG_RC1)
		return -ENOSYS;

	m5_data->model_save.rcomp0 = cp->rcomp0;
	m5_data->model_save.tempco = cp->tempco;
	m5_data->model_save.fullcaprep = cp->fullcaprep;
	m5_data->model_save.fullcapnom = cp->fullcapnom;
	m5_data->model_save.qresidual00 = cp->qresidual00;
	m5_data->model_save.qresidual10 = cp->qresidual10;
	m5_data->model_save.qresidual20 = cp->qresidual20;
	m5_data->model_save.qresidual30 = cp->qresidual30;

	m5_data->model_save.cycles = m5_data->cycles;
	m5_data->model_save.cv_mixcap = m5_data->cv_mixcap;
	m5_data->model_save.halftime = m5_data->halftime;

	m5_data->model_save.crc = max_m5_data_crc("save",
				  &m5_data->model_save);

	ret = gbms_storage_write(GBMS_TAG_GMSR,
				 (const void *)&m5_data->model_save,
				 sizeof(m5_data->model_save));
	if (ret < 0)
		return ret;

	if (ret != sizeof(m5_data->model_save))
		return -ERANGE;

	/* Read back to make sure data all good */
	ret = gbms_storage_read(GBMS_TAG_GMSR, &rb, sizeof(rb));
	if (ret < 0) {
		dev_info(m5_data->dev, "Read Back Data Failed ret=%d\n", ret);
		return ret;
	}

	if (rb.rcomp0 != m5_data->model_save.rcomp0 ||
	    rb.tempco != m5_data->model_save.tempco ||
	    rb.fullcaprep != m5_data->model_save.fullcaprep ||
	    rb.fullcapnom != m5_data->model_save.fullcapnom ||
	    rb.qresidual00 != m5_data->model_save.qresidual00 ||
	    rb.qresidual10 != m5_data->model_save.qresidual10 ||
	    rb.qresidual20 != m5_data->model_save.qresidual20 ||
	    rb.qresidual30 != m5_data->model_save.qresidual30 ||
	    rb.cycles != m5_data->model_save.cycles ||
	    rb.cv_mixcap != m5_data->model_save.cv_mixcap ||
	    rb.halftime != m5_data->model_save.halftime ||
	    rb.crc != m5_data->model_save.crc)
		return -EINVAL;

	return 0;
}

/* 0 ok, < 0 error. Call after reading from the FG */
int max_m5_model_check_state(struct max_m5_data *m5_data)
{
	struct max_m5_custom_parameters *fg_param = &m5_data->parameters;
	bool bad_residual;

	if (!m5_data)
		return -EINVAL;

	if (fg_param->rcomp0 == 0xFF)
		return -ERANGE;

	if (fg_param->tempco == 0xFFFF)
		return -ERANGE;

	bad_residual = fg_param->qresidual00 == 0xffff &&
		       fg_param->qresidual10 == 0xffff &&
		       fg_param->qresidual20 == 0xffff &&
		       fg_param->qresidual30 == 0xffff;
	if (bad_residual)
		return -EINVAL;

	return 0;
}

/*
 * read fuel gauge state to parameters/model state.
 * NOTE: Called on boot if POR is not set or during save state.
 */
int max_m5_model_read_state(struct max_m5_data *m5_data)
{
	int rc;
	struct maxfg_regmap *regmap = m5_data->regmap;

	if (!m5_data)
		return -EINVAL;

	rc= REGMAP_READ(regmap, MAX_M5_RCOMP0, &m5_data->parameters.rcomp0);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_TEMPCO,
				 &m5_data->parameters.tempco);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_FULLCAPREP,
				 &m5_data->parameters.fullcaprep);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_CYCLES, &m5_data->cycles);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_FULLCAPNOM,
				 &m5_data->parameters.fullcapnom);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_QRTABLE00,
				 &m5_data->parameters.qresidual00);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_QRTABLE10,
				 &m5_data->parameters.qresidual10);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_QRTABLE20,
				 &m5_data->parameters.qresidual20);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_QRTABLE30,
				 &m5_data->parameters.qresidual30);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_CV_MIXCAP,
				 &m5_data->cv_mixcap);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_CV_HALFTIME,
				 &m5_data->halftime);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX_M5_CGAIN,
				 &m5_data->parameters.cgain);

	return rc;
}

u16 max_m5_get_designcap(const struct max_m5_data *m5_data)
{
	if (!m5_data)
		return -EINVAL;

	return m5_data->parameters.designcap;
}

ssize_t max_m5_model_state_cstr(char *buf, int max,
				struct max_m5_data *m5_data)
{
	int len = 0;

	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_RCOMP0,
			 m5_data->parameters.rcomp0);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_TEMPCO,
			 m5_data->parameters.tempco);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_FULLCAPREP,
			 m5_data->parameters.fullcaprep);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_CYCLES,
			 m5_data->cycles);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_FULLCAPNOM,
			 m5_data->parameters.fullcapnom);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_QRTABLE00,
			 m5_data->parameters.qresidual00);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_QRTABLE10,
			 m5_data->parameters.qresidual10);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_QRTABLE20,
			 m5_data->parameters.qresidual20);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_QRTABLE30,
			 m5_data->parameters.qresidual30);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_CV_MIXCAP,
			 m5_data->cv_mixcap);
	len += scnprintf(&buf[len], max - len,"%02x:%02x\n", MAX_M5_CV_HALFTIME,
			 m5_data->halftime);

	return len;
}

ssize_t max_m5_gmsr_state_cstr(char *buf, int max)
{
	struct model_state_save saved_data;
	int ret = 0, len = 0;

	ret = gbms_storage_read(GBMS_TAG_GMSR, &saved_data, GBMS_GMSR_LEN);
	if (ret < 0)
		return ret;

	len = scnprintf(&buf[len], max - len,
			"rcomp0     :%04X\ntempco     :%04X\n"
			"fullcaprep :%04X\ncycles     :%04X\n"
			"fullcapnom :%04X\nqresidual00:%04X\n"
			"qresidual10:%04X\nqresidual20:%04X\n"
			"qresidual30:%04X\ncv_mixcap  :%04X\n"
			"halftime   :%04X\n",
			saved_data.rcomp0, saved_data.tempco,
			saved_data.fullcaprep, saved_data.cycles,
			saved_data.fullcapnom, saved_data.qresidual00,
			saved_data.qresidual10, saved_data.qresidual20,
			saved_data.qresidual30, saved_data.cv_mixcap,
			saved_data.halftime);

	return len;
}

/* can be use to restore parametes and model state after POR */
int max_m5_model_state_sscan(struct max_m5_data *m5_data, const char *buf,
			     int max)
{
	int ret, index, reg, val;

	if (!m5_data)
		return -EINVAL;

	for (index = 0; index < max ; index += 1) {
		ret = sscanf(&buf[index], "%x:%x", &reg, &val);
		if (ret != 2) {
			dev_err(m5_data->dev, "@%d: sscan error %d\n",
				index, ret);
			return -EINVAL;
		}

		dev_info(m5_data->dev, "@%d: reg=%x val=%x\n", index, reg, val);

		switch (reg) {
		/* model parameters (fg-params) */
		case MAX_M5_IAVGEMPTY:
			m5_data->parameters.iavg_empty = val;
			break;
		case MAX_M5_RELAXCFG:
			m5_data->parameters.relaxcfg = val;
			break;
		case MAX_M5_LEARNCFG:
			m5_data->parameters.learncfg = val;
			break;
		case MAX_M5_CONFIG:
			m5_data->parameters.config = val;
			break;
		case MAX_M5_CONFIG2:
			m5_data->parameters.config2 = val;
			break;
		case MAX_M5_FULLSOCTHR:
			m5_data->parameters.fullsocthr = val;
			break;
		case MAX_M5_DESIGNCAP:
			m5_data->parameters.designcap = val;
			break;
		case MAX_M5_DPACC:
			m5_data->parameters.dpacc = val;
			break;
		case MAX_M5_DQACC:
			m5_data->parameters.dqacc = val;
			break;
		case MAX_M5_VEMPTY:
			m5_data->parameters.v_empty = val;
			break;
		case MAX_M5_TGAIN:
			m5_data->parameters.tgain = val;
			break;
		case MAX_M5_TOFF:
			m5_data->parameters.toff = val;
			break;
		case MAX_M5_TCURVE:
			m5_data->parameters.tcurve = val;
			break;
		case MAX_M5_MISCCFG:
			m5_data->parameters.misccfg = val;
			break;
		case MAX_M5_ATRATE:
			m5_data->parameters.atrate = val;
			break;
		case MAX_M5_CONVGCFG:
			m5_data->parameters.convgcfg = val;
			break;
		case MAX_M5_FILTERCFG:
			m5_data->parameters.filtercfg = val;
			break;
		case MAX_M5_TASKPERIOD:
			if (val != MAX_M5_TASKPERIOD_175MS &&
			    val != MAX_M5_TASKPERIOD_351MS) {
				dev_err(m5_data->dev, "@%d: reg=%x val %x not allowed\n",
					index, reg, val);
				return -EINVAL;
			}

			m5_data->parameters.taskperiod = val;
			break;

		/* model state, saved and restored */
		case MAX_M5_RCOMP0:
			m5_data->parameters.rcomp0 = val;
			break;
		case MAX_M5_TEMPCO:
			m5_data->parameters.tempco = val;
			break;
		case MAX_M5_FULLCAPREP:
			m5_data->parameters.fullcaprep = val;
			break;
		case MAX_M5_CYCLES:
			m5_data->cycles = val;
			break;
		case MAX_M5_FULLCAPNOM:
			m5_data->parameters.fullcapnom = val;
			break;
		case MAX_M5_QRTABLE00:
			m5_data->parameters.qresidual00 = val;
			break;
		case MAX_M5_QRTABLE10:
			m5_data->parameters.qresidual10 = val;
			break;
		case MAX_M5_QRTABLE20:
			m5_data->parameters.qresidual20 = val;
			break;
		case MAX_M5_QRTABLE30:
			m5_data->parameters.qresidual30 = val;
			break;
		case MAX_M5_CV_MIXCAP:
			m5_data->cv_mixcap = val;
			break;
		case MAX_M5_CV_HALFTIME:
			m5_data->halftime = val;
			break;
		case MAX_M5_CGAIN:
			m5_data->parameters.cgain = val;
			break;
		default:
			dev_err(m5_data->dev, "@%d: reg=%x out of range\n",
				index, reg);
			return -EINVAL;
		}

		for ( ; index < max && buf[index] != '\n'; index++)
			;
	}

	return 0;
}

/* b/177099997 TaskPeriod = 351 ms changes the lsb for capacity conversions */
static int max_m5_read_taskperiod(int *cap_lsb, struct maxfg_regmap *regmap)
{
	u16 data;
	int ret;

	/* TaskPeriod = 351 ms changes the lsb for capacity conversions */
	ret = REGMAP_READ(regmap, MAX_M5_TASKPERIOD, &data);
	if (ret == 0)
		ret = max_m5_period2caplsb(data);
	if (ret < 0)
		return ret;

	*cap_lsb = ret;
	return 0;
}

int max_m5_model_get_cap_lsb(const struct max_m5_data *m5_data)
{
	struct maxfg_regmap *regmap = m5_data->regmap;
	int cap_lsb;

	if (!m5_data)
		return -EINVAL;

	return max_m5_read_taskperiod(&cap_lsb, regmap) < 0 ? -1 : cap_lsb;
}

/* custom model parameters */
int max_m5_fg_model_cstr(char *buf, int max, const struct max_m5_data *m5_data)
{
	int i, len;

	if (!m5_data || !m5_data->custom_model || !m5_data->custom_model_size)
		return -EINVAL;

	for (len = 0, i = 0; i < m5_data->custom_model_size; i += 1)
		len += scnprintf(&buf[len], max - len, "%x: %04x\n",
				 MAX_M5_FG_MODEL_START + i,
				 m5_data->custom_model[i]);

	return len;
}

int max_m5_get_rc_switch_param(struct max_m5_data *m5_data, u16 *rc2_tempco, u16 *rc2_learncfg)
{

	if (!m5_data || m5_data->parameters.tempco <= 0 || m5_data->parameters.learncfg <= 0)
		return -EINVAL;

	*rc2_tempco = m5_data->parameters.tempco;
	*rc2_learncfg = m5_data->parameters.learncfg;

	return 0;
}

/* custom model parameters */
int max_m5_fg_model_sscan(struct max_m5_data *m5_data, const char *buf, int max)
{
	int ret, index, reg, val, fg_model_end;

	if (!m5_data || !m5_data->custom_model)
		return -EINVAL;

	/* use the default size */
	if (!m5_data->custom_model_size)
		m5_data->custom_model_size = MAX_M5_FG_MODEL_SIZE;

	fg_model_end = MAX_M5_FG_MODEL_START + m5_data->custom_model_size;
	for (index = 0; index < max ; index += 1) {
		ret = sscanf(&buf[index], "%x:%x", &reg, &val);
		if (ret != 2) {
			dev_err(m5_data->dev, "@%d: sscan error %d\n",
				index, ret);
			return -EINVAL;
		}

		dev_info(m5_data->dev, "@%d: reg=%x val=%x\n", index, reg, val);

		if (reg >= MAX_M5_FG_MODEL_START && reg < fg_model_end) {
			const int offset = reg - MAX_M5_FG_MODEL_START;

			m5_data->custom_model[offset] = val;
		}

		for ( ; index < max && buf[index] != '\n'; index++)
			;
	}

	return 0;
}

static u16 max_m5_recal_dpacc(struct max_m5_data *m5_data)
{
	if (m5_data->parameters.taskperiod == MAX_M5_TASKPERIOD_351MS)
		return 0x0c80;

	return m5_data->parameters.dpacc;
}

static u16 max_m5_recal_dqacc(struct max_m5_data *m5_data, u16 target_cap)
{
	if (m5_data->parameters.taskperiod == MAX_M5_TASKPERIOD_351MS)
		return target_cap >> 4;

	return m5_data->parameters.dqacc;
}

static u16 max_m5_recal_new_cap(struct max_m5_data *m5_data, u16 dqacc, u16 dpacc)
{
	/*
	 * dQacc LSb is 16mAh with 10mohm, *2 by 5mohm sense resistot, *2 by double task period
	 * dPacc LSb is 0.0625% (1/16)
	 * new capacity is dQacc/dPacc, took LSb in units into consideration:
	 *       dQacc * 16 * 2 * 2                                                  dQacc
	 *    ------------------------ * 100(%) / 2 (for write to fullcapnom/cap) = ------- x 51200
	 *       dPacc * 0.0625                                                      dPacc
	*/
	if (m5_data->parameters.taskperiod == MAX_M5_TASKPERIOD_351MS)
		return dqacc * 0xc800 / dpacc;

	return m5_data->parameters.designcap;
}

static int max_m5_end_recal(struct max_m5_data *m5_data, int algo, u16 new_cap)
{
	struct maxfg_regmap *regmap = m5_data->regmap;
	int ret = 0;

	if (algo == RE_CAL_ALGO_0)
		return ret;

	ret = max_m5_model_read_state(m5_data);
	if (ret != 0)
		return ret;

	ret = max_m5_model_check_state(m5_data);
	if (ret != 0)
		return ret;

	m5_data->parameters.fullcaprep = new_cap;
	m5_data->parameters.fullcapnom = new_cap;
	ret = max_m5_save_state_data(m5_data);
	if (ret != 0)
		return ret;

	ret = max_m5_load_state_data(m5_data);
	if (ret != 0)
		return ret;

	ret = REGMAP_WRITE(regmap, MAX_M5_COMMAND, MAX_M5_COMMAND_HARDWARE_RESET);

	return ret;
}

static bool max_m5_needs_recal(struct max_m5_data *m5_data, u16 new_cap)
{
	u16 design_cap = m5_data->parameters.designcap;

	if (new_cap > (design_cap * 110 / 100) &&
	    m5_data->recal.rounds < MAX_M5_RECAL_MAX_ROUNDS)
		return true;
	return false;
}

static int max_m5_recal_release(struct max_m5_data *m5_data)
{
	struct maxfg_regmap *regmap = m5_data->regmap;
	u16 reg_cycle, data, dpacc, dqacc;
	int ret = 0, retries = 0;

	mutex_lock(&m5_data->recal.lock);
	/* use designcap if not set bhi_target_capacity */
	if (m5_data->recal.target_cap == 0)
		m5_data->recal.target_cap = m5_data->parameters.designcap;

	/* save current cycle before reset to 0 */
	ret = REGMAP_READ(regmap, MAX_M5_CYCLES, &reg_cycle);
	if (ret < 0)
		goto error_done;

	m5_data->recal.base_cycle_reg = reg_cycle;

	/* set 200% dPacc/dQacc */
	dpacc = max_m5_recal_dpacc(m5_data);
	dqacc = max_m5_recal_dqacc(m5_data, m5_data->recal.target_cap);
	do {
		retries++;
		ret = REGMAP_WRITE(regmap, MAX_M5_DPACC, dpacc);
		if (ret == 0)
			ret = REGMAP_WRITE(regmap, MAX_M5_DQACC, dqacc);
		if (ret < 0) {
			msleep(50);
			continue;
		}

		break;
	} while (ret < 0 && retries < 3);

	if (ret < 0)
		goto error_done;

	/* set Cycle to 0 */
	ret = REGMAP_WRITE(regmap, MAX_M5_CYCLES, 0x0);
	if (ret < 0)
		goto error_done;

	/* Set LearnCfg: FCLrnStage=0x0, FCLrn=0x2 */
	ret = REGMAP_READ(regmap, MAX_M5_LEARNCFG, &data);
	if (ret < 0)
		goto error_done;

	data = MAX_M5_LEARNCFG_FCLRNSTAGE_CLR(data);
	data = MAX_M5_LEARNCFG_FCLM_CLR(data) | (0x2 << MAX_M5_LEARNCFG_FCLM_SHIFT);
	ret = REGMAP_WRITE_VERIFY(regmap, MAX_M5_LEARNCFG, data);
	if (ret < 0)
                goto error_done;

	m5_data->recal.state = RE_CAL_STATE_LEARNING;

error_done:
        if (ret < 0)
                dev_info(m5_data->dev, "unable to set RECAL data, ret=%d\n", ret);
	mutex_unlock(&m5_data->recal.lock);
	return ret;
}

/* b/291077564 */
static int max_m5_recal_internal(struct max_m5_data *m5_data)
{
	struct max_m5_custom_parameters *cp = &m5_data->parameters;
	struct maxfg_regmap *regmap = m5_data->regmap;
	u16 reg_cycle, learncfg;
	int ret = 0;

	mutex_lock(&m5_data->recal.lock);
	/* save current cycle before reset to 0 */
	ret = REGMAP_READ(regmap, MAX_M5_CYCLES, &reg_cycle);
	if (ret < 0)
		goto error_done;

	m5_data->recal.base_cycle_reg = reg_cycle;

	/* Clear GMSR */
	ret = max_m5_reset_state_data(m5_data);
	if (ret < 0)
		goto error_done;

	/* Set dPacc/dQacc to ther target capacity from 200% */
	cp->dpacc = max_m5_recal_dpacc(m5_data);
	cp->dqacc = max_m5_recal_dqacc(m5_data, cp->fullcapnom);

	/* Set LearnCfg: FCLrnStage=0x0, FCLrn=0x2 */
	learncfg = cp->learncfg;
	learncfg = MAX_M5_LEARNCFG_FCLRNSTAGE_CLR(learncfg);
	learncfg = MAX_M5_LEARNCFG_FCLM_CLR(learncfg) | (0x2 << MAX_M5_LEARNCFG_FCLM_SHIFT);
	cp->learncfg = learncfg;

	/* reset FG */
	ret = REGMAP_WRITE(regmap, MAX_M5_COMMAND, MAX_M5_COMMAND_HARDWARE_RESET);
	if (ret < 0)
		goto error_done;

	m5_data->recal.state = RE_CAL_STATE_FG_RESET;

error_done:
	mutex_unlock(&m5_data->recal.lock);
	return ret;
}

int max_m5_check_recal_state(struct max_m5_data *m5_data, int algo, u16 eeprom_cycle)
{
	struct maxfg_regmap *regmap;
	u16 learncfg, status, reg_cycle, dqacc, dpacc, new_cap;
	int ret;

	if (!m5_data)
		return 0;

	regmap = m5_data->regmap;

	if (m5_data->recal.state == RE_CAL_STATE_IDLE)
		return 0;

	if (m5_data->recal.state == RE_CAL_STATE_FG_RESET) {
		ret = REGMAP_READ(regmap, MAX_M5_STATUS, &status);
		if (ret < 0)
			return ret;
		if ((status & MAX_M5_STATUS_POR) == 0)
			m5_data->recal.state = RE_CAL_STATE_LEARNING;
	}

	/* check learncfg for recalibration status */
	ret = REGMAP_READ(regmap, MAX_M5_LEARNCFG, &learncfg);
	if (ret < 0)
		return ret;

	/* under learning progress */
	if ((learncfg & MAX_M5_LEARNCFG_FCLRNSTAGE) != MAX_M5_LEARNCFG_FCLRNSTAGE)
		return 0;

	if ((learncfg & MAX_M5_LEARNCFG_RC_VER) != MAX_M5_LEARNCFG_RC2)
		return 0;

	/* restore real cycle */
	reg_cycle = eeprom_cycle << 1;
	ret = REGMAP_WRITE(regmap, MAX_M5_CYCLES, reg_cycle);
	if (ret < 0)
		return ret;

	m5_data->recal.base_cycle_reg = 0;

	/* Check learning capacity */
	ret = REGMAP_READ(regmap, MAX_M5_DQACC, &dqacc);
	if (ret < 0)
		return ret;

	ret = REGMAP_READ(regmap, MAX_M5_DPACC, &dpacc);
	if (ret < 0)
		return ret;

	new_cap = max_m5_recal_new_cap(m5_data, dqacc, dpacc);

	if (max_m5_needs_recal(m5_data, new_cap)) {
		ret = max_m5_recalibration(m5_data, algo, m5_data->recal.target_cap);
		return ret;
	}

	ret = max_m5_end_recal(m5_data, algo, new_cap);
	if (ret < 0)
		dev_warn(m5_data->dev, "fail to restore new capacity, ret=%d\n", ret);

	m5_data->recal.state = RE_CAL_STATE_IDLE;

	return ret;
}

int max_m5_recalibration(struct max_m5_data *m5_data, int algo, u16 cap)
{
	int ret = 0;

	if (!m5_data)
		return -EINVAL;

	if (m5_data->recal.rounds >= MAX_M5_RECAL_MAX_ROUNDS)
		return ret;

	m5_data->recal.target_cap = cap;

	/* TODO: add maximum recalibration times */
	if (algo == RE_CAL_ALGO_0)
		ret = max_m5_recal_release(m5_data);
	else if (algo == RE_CAL_ALGO_1)
		ret = max_m5_recal_internal(m5_data);

	if (ret == 0)
		m5_data->recal.rounds++;

	return ret;
}

int max_m5_recal_state(const struct max_m5_data *m5_data)
{
	if (!m5_data)
		return 0;

	return m5_data->recal.state;
}

int max_m5_recal_cycle(const struct max_m5_data *m5_data)
{
	if (!m5_data)
		return 0;

	return m5_data->recal.base_cycle_reg;
}

/* Initial values??? */
#define CGAIN_RESET_VAL 0x0400
int m5_init_custom_parameters(struct device *dev, struct max_m5_data *m5_data,
			      struct device_node *node)
{
	struct max_m5_custom_parameters *cp = &m5_data->parameters;
	const char *propname = "maxim,fg-params";
	const int cnt_default = sizeof(*cp) / 2 - 1;
	const int cnt_w_cgain = sizeof(*cp) / 2;
	int ret, cnt;

	if (!m5_data)
		return -EINVAL;

	memset(cp, 0, sizeof(*cp));

	cnt = of_property_count_elems_of_size(node, propname, sizeof(u16));
	if (cnt < 0)
		return -ENODATA;

	cp->cgain = CGAIN_RESET_VAL;
	if (cnt != cnt_default && cnt != cnt_w_cgain) {
		dev_err(dev, "fg-params: %s has %d elements, need %ld\n",
			propname, cnt, sizeof(*cp) / 2);
		return -ERANGE;
	}

	ret = of_property_read_u16_array(node, propname, (u16 *)cp, cnt);
	if (ret < 0) {
		dev_err(dev, "fg-params: failed to read %s %s: %d\n",
			node->name, propname, ret);
		return -EINVAL;
	}

	return 0;
}

void max_m5_free_data(struct max_m5_data *m5_data)
{
	if (m5_data)
		devm_kfree(m5_data->dev, m5_data);
}

void *max_m5_init_data(struct device *dev, struct device_node *node,
		       struct maxfg_regmap *regmap)
{
	const char *propname = "maxim,fg-model";
	struct max_m5_data *m5_data;
	int cnt, ret;
	u16 *model;
	u32 temp;

	m5_data = devm_kzalloc(dev, sizeof(*m5_data), GFP_KERNEL);
	if (!m5_data) {
		dev_err(dev, "fg-model: %s not found\n", propname);
		return ERR_PTR(-ENOMEM);
	}

	model = devm_kmalloc_array(dev, MAX_M5_FG_MODEL_SIZE, sizeof(u16),
				   GFP_KERNEL);
	if (!model) {
		dev_err(dev, "fg-model: out of memory\n");
		return ERR_PTR(-ENOMEM);
	}

	cnt = of_property_count_elems_of_size(node, propname, sizeof(u16));
	if (cnt != MAX_M5_FG_MODEL_SIZE) {
		dev_err(dev, "fg-model: not found, or invalid %d\n", cnt);
	} else {
		ret = of_property_read_u16_array(node, propname, model, cnt);
		if (ret < 0)
			dev_err(dev, "fg-model: no data cnt=%d %s %s: %d\n",
				cnt, node->name, propname, ret);
		else
			m5_data->custom_model_size = cnt;
	}

	ret = of_property_read_u32(node, "maxim,model-version", &temp);
	if (ret < 0 || temp > 255)
		temp = MAX_M5_INVALID_VERSION;
	m5_data->model_version = temp;

	m5_data->force_reset_model_data =
		of_property_read_bool(node, "maxim,force-reset-model-data");

	/*
	 * Initial values: check max_m5_model_read_state() for the registers
	 * updated from max1720x_model_work()
	 */
	ret = m5_init_custom_parameters(dev, m5_data, node);
	if (ret < 0)
		dev_err(dev, "fg-params: %s not found\n", propname);

	/* b/177099997 TaskPeriod changes LSB for capacity etc. */
	ret = max_m5_read_taskperiod(&m5_data->cap_lsb, regmap);
	if (ret < 0)
		dev_err(dev, "Cannot set TaskPeriod (%d)\n", ret);

	crc8_populate_msb(m5_crc8_table, MAX_M5_CRC8_POLYNOMIAL);

	m5_data->custom_model = model;
	m5_data->regmap = regmap;
	m5_data->dev = dev;

	return m5_data;
}

static bool max_m5_is_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00 ... 0x4F:
	case 0xB0 ... 0xBF:
	case 0xD0:		/* IIC */
	case 0xDC ... 0xDF:
	case 0xFB:
	case 0xFF:		/* VFSOC */
		return true;
	case 0x60:		/* Model unlock */
	case 0x62:		/* Unlock Model Access */
	case 0x63:		/* Unlock Model Access */
	case 0x80 ... 0xAF:	/* FG Model */
		/* TODO: add a check on unlock */
		return true;
	case 0xEB:              /* CoTrim */
		return true;
	}

	return false;
}

const struct regmap_config max_m5_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX_M5_VFSOC,
	.readable_reg = max_m5_is_reg,
	.volatile_reg = max_m5_is_reg,
};

const struct maxfg_reg max_m5[] = {
	[MAXFG_TAG_avgc] = { ATOM_INIT_REG16(MAX_M5_AVGCURRENT)},
	[MAXFG_TAG_cnfg] = { ATOM_INIT_REG16(MAX_M5_CONFIG)},
	[MAXFG_TAG_mmdv] = { ATOM_INIT_REG16(MAX_M5_MAXMINVOLT)},
	[MAXFG_TAG_vcel] = { ATOM_INIT_REG16(MAX_M5_VCELL)},
	[MAXFG_TAG_temp] = { ATOM_INIT_REG16(MAX_M5_TEMP)},
	[MAXFG_TAG_curr] = { ATOM_INIT_REG16(MAX_M5_CURRENT)},
	[MAXFG_TAG_mcap] = { ATOM_INIT_REG16(MAX_M5_MIXCAP)},
	[MAXFG_TAG_vfsoc] = { ATOM_INIT_REG16(MAX_M5_VFSOC)},
	[MAXFG_TAG_vfocv] = { ATOM_INIT_REG16(MAX_M5_VFOCV)},
	[MAXFG_TAG_tempco] = { ATOM_INIT_REG16(MAX_M5_TEMPCO)},
	[MAXFG_TAG_rcomp0] = { ATOM_INIT_REG16(MAX_M5_RCOMP0)},
	[MAXFG_TAG_timerh] = { ATOM_INIT_REG16(MAX_M5_TIMERH)},
	[MAXFG_TAG_descap] = { ATOM_INIT_REG16(MAX_M5_DESIGNCAP)},
	[MAXFG_TAG_fcnom] = { ATOM_INIT_REG16(MAX_M5_FULLCAPNOM)},
	[MAXFG_TAG_fcrep] = { ATOM_INIT_REG16(MAX_M5_FULLCAPREP)},
	[MAXFG_TAG_msoc] = { ATOM_INIT_REG16(MAX_M5_MIXSOC)},
	[MAXFG_TAG_mmdt] = { ATOM_INIT_REG16(MAX_M5_MAXMINTEMP)},
	[MAXFG_TAG_mmdc] = { ATOM_INIT_REG16(MAX_M5_MAXMINCURR)},
	[MAXFG_TAG_repsoc] = { ATOM_INIT_REG16(MAX_M5_REPSOC)},
	[MAXFG_TAG_avcap] = { ATOM_INIT_REG16(MAX_M5_AVCAP)},
	[MAXFG_TAG_repcap] = { ATOM_INIT_REG16(MAX_M5_REPCAP)},
	[MAXFG_TAG_fulcap] = { ATOM_INIT_REG16(MAX_M5_FULLCAP)},
	[MAXFG_TAG_qh0] = { ATOM_INIT_REG16(MAX_M5_QH0)},
	[MAXFG_TAG_qh] = { ATOM_INIT_REG16(MAX_M5_QH)},
	[MAXFG_TAG_dqacc] = { ATOM_INIT_REG16(MAX_M5_DQACC)},
	[MAXFG_TAG_dpacc] = { ATOM_INIT_REG16(MAX_M5_DPACC)},
	[MAXFG_TAG_qresd] = { ATOM_INIT_REG16(MAX_M5_QRESIDUAL)},
	[MAXFG_TAG_fstat] = { ATOM_INIT_REG16(MAX_M5_FSTAT)},
	[MAXFG_TAG_learn] = { ATOM_INIT_REG16(MAX_M5_LEARNCFG)},
	[MAXFG_TAG_filcfg] = { ATOM_INIT_REG16(MAX_M5_FILTERCFG)},
	[MAXFG_TAG_vfcap] = { ATOM_INIT_REG16(MAX_M5_VFREMCAP)},
	[MAXFG_TAG_cycles] = { ATOM_INIT_REG16(MAX_M5_CYCLES)},
	[MAXFG_TAG_rslow] = { ATOM_INIT_REG16(MAX_M5_RSLOW)},
	[MAXFG_TAG_relaxcfg] = { ATOM_INIT_REG16(MAX_M5_RELAXCFG)},
	[MAXFG_TAG_avgt] = { ATOM_INIT_REG16(MAX_M5_AVGTA)},
	[MAXFG_TAG_avgv] = { ATOM_INIT_REG16(MAX_M5_AVGVCELL)},
	[MAXFG_TAG_mixcap] = { ATOM_INIT_REG16(MAX_M5_MIXCAP)},
	[MAXFG_TAG_vfremcap] = { ATOM_INIT_REG16(MAX_M5_VFREMCAP)},
	[MAXFG_TAG_vfsoc0] = { ATOM_INIT_REG16(MAX_M5_VFSOC0)},
	[MAXFG_TAG_qrtable00] = { ATOM_INIT_REG16(MAX_M5_QRTABLE00)},
	[MAXFG_TAG_qrtable10] = { ATOM_INIT_REG16(MAX_M5_QRTABLE10)},
	[MAXFG_TAG_qrtable20] = { ATOM_INIT_REG16(MAX_M5_QRTABLE20)},
	[MAXFG_TAG_qrtable30] = { ATOM_INIT_REG16(MAX_M5_QRTABLE30)},
	[MAXFG_TAG_status] = { ATOM_INIT_REG16(MAX_M5_STATUS)},
	[MAXFG_TAG_fullsocthr] = { ATOM_INIT_REG16(MAX_M5_FULLSOCTHR)},
	[MAXFG_TAG_misccfg] = { ATOM_INIT_REG16(MAX_M5_MISCCFG)},
};

int max_m5_regmap_init(struct maxfg_regmap *regmap, struct i2c_client *clnt)
{
	struct regmap *map;

	map = devm_regmap_init_i2c(clnt, &max_m5_regmap_cfg);
	if (IS_ERR(map))
		return IS_ERR_VALUE(map);

	regmap->regtags.max = ARRAY_SIZE(max_m5);
	regmap->regtags.map = max_m5;
	regmap->regmap = map;
	return 0;
}

/*
 * The model data's custom_parameters contain values for FullSOCThr and MISCCFG.
 *  - before the model data is loaded using max1720x_model_load,
 *    these values must be updated based on aafv.
 */
void max_m5_model_apply_aaf_fullsoc(struct max_m5_data *m5_data, const struct aafv_fg_config *cfg)
{
	struct max_m5_custom_parameters *cp = &m5_data->parameters;

	cp->fullsocthr = percentage_to_reg(cfg->fullsoc);
	cp->misccfg = (MAX_M5_MISCCFG_OOPSFILTER_CLEAR & cp->misccfg) |
		      (cfg->fus << MAX_M5_MISCCFG_OOPSFILTER_SHIFT);
}