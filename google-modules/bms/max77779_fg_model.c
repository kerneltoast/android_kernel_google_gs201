/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Fuel gauge driver for MAX77779 Fuel Gauges with M5 Algo
 *
 * Copyright (C) 2023 Google Inc.
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
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include "google_bms.h"
#include "google_psy.h"

#include "max77779_fg.h"
#include "maxfg_common.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#define MAX7779_FG_CRC8_POLYNOMIAL		0x07	/* (x^8) + x^2 + x + 1 */
DECLARE_CRC8_TABLE(max77779_fg_crc8_table);

/*
 * b/329101930: using MAX77779 sp to save model version
 * SP reset value is undefined and only reset when entering shipmode and UVLO.
 * If current SP value is the same as model version, then model won't reload.
 * Otherwise, model will reload and SP data will be model version.
 * If enter shipmode or UVLO, POR will lead to reload model regardless of SP data.
 * So undefined reset value won't be a side effect for that.
 */
int max77779_model_read_version(const struct max77779_model_data *model_data)
{
	u8 temp;
	int ret;

	ret = gbms_storage_read(GBMS_TAG_MDLV, &temp, sizeof(temp));

	return ret < 0 ? ret : temp;
}

int max77779_model_write_version(const struct max77779_model_data *model_data, int version)
{
	u8 temp;
	int ret;

	if (version == MAX77779_FG_INVALID_VERSION)
		return 0;

	temp = (u8)version;
	ret = gbms_storage_write(GBMS_TAG_MDLV, &temp, sizeof(temp));

	return ret < 0 ? ret : 0;
}

int max77779_reset_state_data(struct max77779_model_data *model_data)
{
	struct model_state_save data;
	struct max77779_fg_chip *chip = dev_get_drvdata(model_data->dev);
	int ret = 0;

	__pm_stay_awake(chip->fg_wake_lock);
	mutex_lock(&chip->save_data_lock);

	memset(&data, 0xff, sizeof(data));

	ret = gbms_storage_write(GBMS_TAG_GMSR, &data, sizeof(data));

	mutex_unlock(&chip->save_data_lock);
	__pm_relax(chip->fg_wake_lock);

	if (ret != GBMS_GMSR_LEN)
		dev_warn(model_data->dev, "Erase GMSR fail (%d)\n", ret);

	return ret == sizeof(data) ? 0 : ret;
}

static int max77779_read_custom_model(struct regmap *regmap, u16 *model_data,
				    int count)
{
	return regmap_raw_read(regmap, MAX77779_FG_MODEL_START, model_data, count * 2);
}

static int max77779_write_custom_model(const struct maxfg_regmap *regmap, u16 *model_data,
				       int count)
{
	int ret;

	ret = regmap_raw_write(regmap->regmap, MAX77779_FG_MODEL_START, model_data, count * 2);
	if (ret < 0)
		pr_err("%s: Failed to write custom model ret=%d\n", __func__, ret);

	return ret;
}

/* Requires the fg registers be unlocked */
static int max77779_update_custom_model(struct max77779_model_data *model_data)
{
	int ret = 0;
	bool success;
	u16 *data;

	data = kzalloc(model_data->custom_model_size * 2, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = REGMAP_WRITE(model_data->regmap, MAX77779_FG_RepCap, 0);
	if (ret < 0)
		goto error_exit;

	ret = max77779_write_custom_model(model_data->regmap, model_data->custom_model,
					  model_data->custom_model_size);
	if (ret < 0) {
		dev_err(model_data->dev, "cannot write custom model (%d)\n", ret);
		goto error_exit;
	}

	ret = max77779_read_custom_model(model_data->regmap->regmap, data,
					 model_data->custom_model_size);
	if (ret < 0) {
		dev_err(model_data->dev, "cannot read custom model (%d)\n", ret);
		goto error_exit;
	}

	ret = memcmp(model_data->custom_model, data, model_data->custom_model_size * 2);
	success = ret == 0;
	if (!success) {
		dev_err(model_data->dev, "cannot write custom model (%d)\n", ret);
		dump_model(model_data->dev, MAX77779_FG_MODEL_START, model_data->custom_model,
			   model_data->custom_model_size);
		dump_model(model_data->dev, MAX77779_FG_MODEL_START, data,
			   model_data->custom_model_size);
		ret = -ERANGE;
	}

error_exit:
	kfree(data);
	return ret;
}

static int max77779_update_custom_parameters(struct max77779_model_data *model_data, int revision,
					     int sub_rev)
{
	struct max77779_custom_parameters *cp = &model_data->parameters;
	struct maxfg_regmap *debug_regmap = model_data->debug_regmap;
	struct maxfg_regmap *regmap = model_data->regmap;
	const u16 hibcfg = model_data->hibcfg > 0 ? model_data->hibcfg : 0x0909;
	int ret, attempt;
	u16 data;

	ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nDesignCap, cp->designcap);
	if (ret < 0)
		return ret;

	ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nFullCapRep, cp->fullcaprep);
	if (ret < 0)
		return ret;

	for (attempt = 0; attempt < 3; attempt++) {
		ret = REGMAP_WRITE(regmap, MAX77779_FG_dPAcc, cp->dpacc);
		if (ret < 0)
			continue;
		msleep(2);
		ret = REGMAP_READ(regmap, MAX77779_FG_dPAcc, &data);
		if (ret == 0 && data == 0xC80)
			break;
	}

	if (attempt == 3)
		return ret;

	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nFullCapNom, cp->fullcapnom);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nIChgTerm, cp->ichgterm);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nVEmpty, cp->v_empty);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nRComp0, cp->rcomp0);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nTempCo, cp->tempco);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nCycles, model_data->cycles);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nQRTable00, cp->qresidual00);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nQRTable10, cp->qresidual10);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nQRTable20, cp->qresidual20);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nQRTable30, cp->qresidual30);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nHibCfg, hibcfg);
	/* b/308287790 - Clear nMOdelCfg.Refresh if firmware revision < 2.6 */
	if (revision < 2 || (revision == 2 && sub_rev < 6)) {
		if (ret == 0)
			ret = REGMAP_READ(debug_regmap, MAX77779_FG_NVM_nModelCfg, &data);
		if (ret == 0)
			ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nModelCfg, data & 0x7FFF);
	}
	if (ret == 0)
		ret = REGMAP_WRITE_VERIFY(debug_regmap, MAX77779_FG_NVM_nLearnCfg, cp->learncfg);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_RelaxCFG, cp->relaxcfg);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX77779_FG_Config, cp->config);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nFullSOCThr, cp->fullsocthr);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nMiscCfg, cp->misccfg);

	/* In INI but not part of model loading guide */
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nThermCfg, cp->thermcfg);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nNVCfg0, cp->nvcfg0);
	if (ret == 0)
		ret = REGMAP_WRITE(debug_regmap, MAX77779_FG_NVM_nFilterCfg, cp->filtercfg);

	return ret;
}

/*
 * Model loading procedure version: 0.2.1
 * 0 is ok
 */
#define MODEL_LOADING_VERSION "0.2.1"
int max77779_load_gauge_model(struct max77779_model_data *model_data, int rev, int sub_rev)
{
	struct maxfg_regmap *regmap = model_data->regmap;
	u16 data, config2, status, temp;
	int rc, ret, retries;

	if (!model_data || !model_data->custom_model || !model_data->custom_model_size)
		return -ENODATA;

	if (!rev && !sub_rev)
		return -EINVAL;

	if (!regmap) {
		dev_err(model_data->dev, "Error! No regmap\n");
		return -EIO;
	}

	dev_info(model_data->dev, "Model loading version:%s\n", MODEL_LOADING_VERSION);

	/*
	 * Step 1: Check for POR (not needed, we're here when POR is set)
	 * substep: check RISC-V status, 0x82 should be present
	 */
	for (retries = 20; retries > 0; retries--) {
		ret = REGMAP_READ(regmap, MAX77779_FG_BOOT_CHECK_REG, &data);
		if (ret == 0 &&
		    (data & MAX77779_FG_BOOT_CHECK_SUCCESS) == MAX77779_FG_BOOT_CHECK_SUCCESS)
			break;

		msleep(10);
	}
	if (retries == 0) {
		dev_err(model_data->dev, "Error RISC-V is not ready\n");
		return -ETIMEDOUT;
	}

	/*
	 * Step 2: Delay until FSTAT.DNR bit == 0
	 * check FStat.DNR to wait it clear for data read
	 */
	for (retries = 20; retries > 0; retries--) {
		ret = REGMAP_READ(regmap, MAX77779_FG_FStat, &data);
		if (ret == 0 && !(data & MAX77779_FG_FStat_DNR_MASK))
			break;
		msleep(10);
	}
	dev_info(model_data->dev, "retries:%d, FSTAT:%#x\n", retries, data);
	if (retries == 0) {
		dev_err(model_data->dev, "Error FSTAT.DNR not clear\n");
		return -ETIMEDOUT;
	}

	/* Step 3.1: Unlock command */
	ret = max77779_fg_usr_lock_section(regmap, MAX77779_FG_ALL_SECTION, false);
	if (ret < 0) {
		dev_err(model_data->dev, "Error Unlock (%d)\n", ret);
		return ret;
	}

	ret = REGMAP_READ(regmap, MAX77779_FG_HibCfg, &model_data->hibcfg);
	if (ret == 0)
		ret = REGMAP_WRITE(regmap, MAX77779_FG_HibCfg, 0);
	if (ret < 0) {
		dev_err(model_data->dev, "Error read/write HibCFG (%d)\n", ret);
		goto error_done;
	}

	/* Step 3.4.1: Write/read/verify the Custom Model */
	ret = max77779_update_custom_model(model_data);
	if (ret < 0) {
		dev_err(model_data->dev, "cannot update custom model (%d)\n", ret);
		goto error_done;
	}

	/* step 3.5 Identify Battery: already done in max77779_load_state_data */

	/* Step 3.6: Write Custom Parameters */
	ret = max77779_update_custom_parameters(model_data, rev, sub_rev);
	if (ret < 0) {
		dev_err(model_data->dev, "cannot update custom parameters (%d)\n", ret);
		goto error_done;
	}

	/* Step 3.6.1: Initiate Model Loading */
	ret = REGMAP_READ(regmap, MAX77779_FG_Config2, &config2);
	if (ret < 0) {
		dev_err(model_data->dev, "Failed read config2 (%d)\n", ret);
		goto error_done;
	}

	ret = REGMAP_WRITE(regmap, MAX77779_FG_Config2, config2 | MAX77779_FG_Config2_LDMdl_MASK);
	if (ret < 0) {
		dev_err(model_data->dev, "Failed initiate model loading (%d)\n", ret);
		goto error_done;
	}

	/* Step 3.6.2: Poll Config2.LdMdl */
	for (retries = 20; retries > 0; retries--) {
		ret = REGMAP_READ(regmap, MAX77779_FG_Config2, &config2);
		if (ret == 0 && !(config2 & MAX77779_FG_Config2_LDMdl_MASK))
			break;

		usleep_range(WAIT_VERIFY, WAIT_VERIFY + 100);
	}

	if (retries == 0) {
		dev_err(model_data->dev, "cannot initiate model loading (%d)\n", ret);
		ret = -ETIMEDOUT;
		goto error_done;
	}

	/* Restore Config2 */
	ret = REGMAP_WRITE(regmap, MAX77779_FG_Config2, model_data->parameters.config2);
	if (ret < 0)
		dev_err(model_data->dev, "cannot restore Config2 (%d)\n", ret);

	/* b/328398641 need delay to internal register re-synchronized when FW ver. < 3.8 */
	if (rev < 3 || (rev == 3 && sub_rev < 8))
		msleep(200);

	/* Step 4.1: Clear POR bit */
	for (retries = 10; retries > 0; retries--) {
		ret = REGMAP_WRITE(regmap, MAX77779_FG_FG_INT_STS,
				   MAX77779_FG_FG_INT_MASK_POR_m_MASK);
		msleep(100);

		if (ret == 0)
			ret = REGMAP_READ(regmap, MAX77779_FG_FG_INT_STS, &status);

		if (ret == 0 && !(status & MAX77779_FG_FG_INT_MASK_POR_m_MASK))
			break;
	}

	if (retries == 0) {
		dev_err(model_data->dev, "cannot clear PONR bit, fg_int_sts:%#x\n", status);
		return -ETIMEDOUT;
	}

	/* Step 4.2: Lock command */
	ret = max77779_fg_usr_lock_section(regmap, MAX77779_FG_ALL_SECTION, true);
	if (ret < 0) {
		dev_err(model_data->dev, "Error Lock (%d)\n", ret);
		return ret;
	}

	/*
	 * NOTE: Not a part of loading guide
	 * version could be in the DT: this will overwrite it if set.
	 * Invalid version is not written out.
	 */
	ret = max77779_model_write_version(model_data, model_data->model_version);
	if (ret < 0) {
		dev_err(model_data->dev, "cannot update version (%d)\n", ret);
		return ret;
	}

	temp = max77779_model_read_version(model_data);
	if (model_data->model_version == MAX77779_FG_INVALID_VERSION) {
		dev_err(model_data->dev, "No Model Version, Current %x\n", temp);
		return -EINVAL;
	} else if (temp != model_data->model_version) {
		dev_err(model_data->dev, "Model Version %x, Mismatch %x\n",
			model_data->model_version, temp);
		return -EINVAL;
	}

	return 0;

error_done:
	rc = max77779_fg_usr_lock_section(regmap, MAX77779_FG_ALL_SECTION, true);
	if (rc < 0)
		dev_err(model_data->dev, "Error Lock (%d)\n", rc);

	return ret;
}

#define MAX77779_FG_CAP_MAX_RATIO	110
#define MAX77779_FG_CAP_MIN_RATIO	50
static int max77779_fg_check_state_data(struct model_state_save *state,
					struct max77779_custom_parameters *ini)
{
	int max_cap = ini->designcap * MAX77779_FG_CAP_MAX_RATIO / 100;
	int min_cap = ini->designcap * MAX77779_FG_CAP_MIN_RATIO / 100;

	if (state->rcomp0 == 0xFFFF || state->rcomp0 == 0)
		return -ERANGE;

	if (state->tempco == 0xFFFF || state->tempco == 0)
		return -ERANGE;

	if (state->fullcaprep > max_cap)
		return -ERANGE;

	if (state->fullcapnom > max_cap)
		return -ERANGE;

	if (state->fullcaprep < min_cap)
		return -ERANGE;

	if (state->fullcapnom < min_cap)
		return -ERANGE;

	if (state->cycles == 0xFFFF)
		return -ERANGE;

	return 0;
}

static u8 max77779_fg_crc(u8 *pdata, size_t nbytes, u8 crc)
{
	return crc8(max77779_fg_crc8_table, pdata, nbytes, crc);
}

static u8 max77779_fg_data_crc(char *reason, struct model_state_save *state)
{
	u8 crc;

	/* Last byte is for saving CRC */
	crc = max77779_fg_crc((u8 *)state, sizeof(struct model_state_save) - 1,
			  CRC8_INIT_VALUE);

	pr_info("%s gmsr: %X %X %X %X %X %X %X %X %X %X (%X)\n",
		reason, state->qrtable00, state->qrtable10, state->qrtable20, state->qrtable30,
		state->fullcaprep, state->fullcapnom, state->rcomp0, state->tempco,
		state->cycles, state->crc, crc);

	return crc;
}

/*
 * Load parameters and model state from permanent storage.
 * Called on boot after POR
 */
int max77779_load_state_data(struct max77779_model_data *model_data)
{
	struct max77779_custom_parameters *cp = &model_data->parameters;
	struct max77779_fg_chip *chip = dev_get_drvdata(model_data->dev);
	u8 crc;
	int ret;

	if (!model_data)
		return -EINVAL;

	/* might return -EAGAIN during init */
	mutex_lock(&chip->save_data_lock);
	ret = gbms_storage_read(GBMS_TAG_GMSR, &model_data->model_save,
				sizeof(model_data->model_save));
	mutex_unlock(&chip->save_data_lock);

	if (ret != GBMS_GMSR_LEN) {
		dev_info(model_data->dev, "Saved Model Data empty\n");
		return ret;
	}

	ret = max77779_fg_check_state_data(&model_data->model_save, cp);
	if (ret < 0)
		return ret;

	crc = max77779_fg_data_crc("restore", &model_data->model_save);
	if (crc != model_data->model_save.crc)
		return -EINVAL;

	cp->qresidual00 = model_data->model_save.qrtable00;
	cp->qresidual10 = model_data->model_save.qrtable10;
	cp->qresidual20 = model_data->model_save.qrtable20;
	cp->qresidual30 = model_data->model_save.qrtable30;
	cp->fullcaprep = model_data->model_save.fullcaprep;
	cp->fullcapnom = model_data->model_save.fullcapnom;
	cp->rcomp0 = model_data->model_save.rcomp0;
	cp->tempco = model_data->model_save.tempco;
	model_data->cycles = model_data->model_save.cycles;

	return 0;
}

/* save/commit parameters and model state to permanent storage */
int max77779_save_state_data(struct max77779_model_data *model_data)
{
	struct max77779_custom_parameters *cp = &model_data->parameters;
	struct max77779_fg_chip *chip = dev_get_drvdata(model_data->dev);

	struct model_state_save rb;
	int ret = 0;

	__pm_stay_awake(chip->fg_wake_lock);
	mutex_lock(&chip->save_data_lock);

	model_data->model_save.qrtable00 = cp->qresidual00;
	model_data->model_save.qrtable10 = cp->qresidual10;
	model_data->model_save.qrtable20 = cp->qresidual20;
	model_data->model_save.qrtable30 = cp->qresidual30;
	model_data->model_save.fullcaprep = cp->fullcaprep;
	model_data->model_save.fullcapnom = cp->fullcapnom;
	model_data->model_save.rcomp0 = cp->rcomp0;
	model_data->model_save.tempco = cp->tempco;
	model_data->model_save.cycles = model_data->cycles;

	model_data->model_save.crc = max77779_fg_data_crc("save", &model_data->model_save);


	ret = gbms_storage_write(GBMS_TAG_GMSR, (const void *)&model_data->model_save,
				 sizeof(model_data->model_save));
	if (ret != GBMS_GMSR_LEN)
		goto max77779_save_state_data_exit;

	if (ret != sizeof(model_data->model_save)) {
		ret = -ERANGE;
		goto max77779_save_state_data_exit;
	}

	/* Read back to make sure data all good */
	ret = gbms_storage_read(GBMS_TAG_GMSR, &rb, sizeof(rb));
	if (ret != GBMS_GMSR_LEN) {
		dev_info(model_data->dev, "Read Back Data Failed ret=%d\n", ret);
		goto max77779_save_state_data_exit;
	}

	if (rb.rcomp0 != model_data->model_save.rcomp0 ||
	    rb.tempco != model_data->model_save.tempco ||
	    rb.fullcaprep != model_data->model_save.fullcaprep ||
	    rb.fullcapnom != model_data->model_save.fullcapnom ||
	    rb.cycles != model_data->model_save.cycles ||
	    rb.crc != model_data->model_save.crc)
		ret = -EINVAL;
	else
		ret = 0;

max77779_save_state_data_exit:
	mutex_unlock(&chip->save_data_lock);
	__pm_relax(chip->fg_wake_lock);

	return ret;
}

bool max77779_fg_check_state(struct max77779_model_data *model_data)
{
	int rc;
	struct maxfg_regmap *regmap = model_data->regmap;
	struct maxfg_regmap *debug_regmap = model_data->debug_regmap;
	struct max77779_custom_parameters *cp = &model_data->parameters;
	const int min_cap = cp->designcap * MAX77779_FG_CAP_MIN_RATIO / 100;
	u16 fullcapnom, fullcaprep, rcomp0, tempco;

	rc = REGMAP_READ(regmap, MAX77779_FG_FullCapRep, &fullcaprep);
	if (rc == 0 && fullcaprep < min_cap)
		return false;

	rc = REGMAP_READ(regmap, MAX77779_FG_FullCapNom, &fullcapnom);
	if (rc == 0 && fullcapnom < min_cap)
		return false;

	rc = REGMAP_READ(debug_regmap, MAX77779_FG_NVM_nRComp0, &rcomp0);
	if (rc == 0 && rcomp0 == 0)
		return false;

	rc = REGMAP_READ(debug_regmap, MAX77779_FG_NVM_nTempCo, &tempco);
	if (rc == 0 && tempco == 0)
		return false;

	return true;
}

/* 0 ok, < 0 error. Call after reading from the FG */
int max77779_model_check_state(struct max77779_model_data *model_data)
{
	struct max77779_custom_parameters *fg_param = &model_data->parameters;

	if (fg_param->rcomp0 == 0xFF)
		return -ERANGE;

	if (fg_param->tempco == 0xFFFF)
		return -ERANGE;
	return 0;
}

/*
 * read fuel gauge state to parameters/model state.
 * NOTE: Called on boot if POR is not set or during save state.
 */
int max77779_model_read_state(struct max77779_model_data *model_data)
{
	int rc;
	struct maxfg_regmap *regmap = model_data->regmap;
	struct maxfg_regmap *debug_regmap = model_data->debug_regmap;
	struct max77779_custom_parameters *cp = &model_data->parameters;

	rc = REGMAP_READ(regmap, MAX77779_FG_QRTable00, &cp->qresidual00);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX77779_FG_QRTable10, &cp->qresidual10);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX77779_FG_QRTable20, &cp->qresidual20);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX77779_FG_QRTable30, &cp->qresidual30);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX77779_FG_FullCapNom, &cp->fullcapnom);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX77779_FG_FullCapRep, &cp->fullcaprep);
	if (rc == 0)
		rc = REGMAP_READ(debug_regmap, MAX77779_FG_NVM_nRComp0, &cp->rcomp0);
	if (rc == 0)
		rc = REGMAP_READ(debug_regmap, MAX77779_FG_NVM_nTempCo, &cp->tempco);
	if (rc == 0)
		rc = REGMAP_READ(regmap, MAX77779_FG_Cycles, &model_data->cycles);

	return rc;
}

u16 max77779_get_relaxcfg(const struct max77779_model_data *model_data)
{
	return model_data->parameters.relaxcfg;
}

u16 max77779_get_designcap(const struct max77779_model_data *model_data)
{
	return model_data->parameters.designcap;
}

ssize_t max77779_model_state_cstr(char *buf, int max, struct max77779_model_data *model_data)
{
	int len = 0;

	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nRComp0,
			 model_data->parameters.rcomp0);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nTempCo,
			 model_data->parameters.tempco);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_FullCapRep,
			 model_data->parameters.fullcaprep);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_Cycles,
			 model_data->cycles);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_FullCapNom,
			 model_data->parameters.fullcapnom);

	return len;
}

int max77779_fg_param_cstr(char *buf, int max, const struct max77779_model_data *model_data)
{
	int len = 0;

	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nNVCfg0,
			 model_data->parameters.nvcfg0);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_RelaxCFG,
			 model_data->parameters.relaxcfg);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nLearnCfg,
			 model_data->parameters.learncfg);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_Config,
			 model_data->parameters.config);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_Config2,
			 model_data->parameters.config2);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nFullSOCThr,
			 model_data->parameters.fullsocthr);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nFullCapRep,
			 model_data->parameters.fullcaprep);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nDesignCap,
			 model_data->parameters.designcap);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_dPAcc,
			 model_data->parameters.dpacc);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nFullCapNom,
			 model_data->parameters.fullcapnom);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nVEmpty,
			 model_data->parameters.v_empty);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nQRTable00,
			 model_data->parameters.qresidual00);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nQRTable10,
			 model_data->parameters.qresidual10);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nQRTable20,
			 model_data->parameters.qresidual20);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nQRTable30,
			 model_data->parameters.qresidual30);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nRComp0,
			 model_data->parameters.rcomp0);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nTempCo,
			 model_data->parameters.tempco);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nIChgTerm,
			 model_data->parameters.ichgterm);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nMiscCfg,
			 model_data->parameters.misccfg);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nModelCfg,
			 model_data->parameters.modelcfg);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nThermCfg,
			 model_data->parameters.thermcfg);
	len += scnprintf(&buf[len], max - len, "%02x: %04x\n", MAX77779_FG_NVM_nFilterCfg,
			 model_data->parameters.filtercfg);

	return len;
}

/* can be use to restore parameters and model state after POR */
int max77779_fg_param_sscan(struct max77779_model_data *model_data, const char *buf, int max)
{
	int ret, index, reg, val;

	for (index = 0; index < max ; index += 1) {
		ret = sscanf(&buf[index], "%x:%x", &reg, &val);
		if (ret != 2) {
			dev_err(model_data->dev, "@%d: sscan error %d\n",
				index, ret);
			return -EINVAL;
		}

		dev_info(model_data->dev, "@%d: reg=%x val=%x\n", index, reg, val);

		switch (reg) {
		/* model parameters (fg-params) */
		case MAX77779_FG_NVM_nNVCfg0:
			model_data->parameters.nvcfg0 = val;
			break;
		case MAX77779_FG_NVM_RelaxCFG:
			model_data->parameters.relaxcfg = val;
			break;
		case MAX77779_FG_NVM_nLearnCfg:
			model_data->parameters.learncfg = val;
			break;
		case MAX77779_FG_Config:
			model_data->parameters.config = val;
			break;
		case MAX77779_FG_Config2:
			model_data->parameters.config2 = val;
			break;
		case MAX77779_FG_NVM_nFullSOCThr:
			model_data->parameters.fullsocthr = val;
			break;
		case MAX77779_FG_NVM_nFullCapRep:
			model_data->parameters.fullcaprep = val;
			break;
		case MAX77779_FG_NVM_nDesignCap:
			model_data->parameters.designcap = val;
			break;
		case MAX77779_FG_dPAcc:
			model_data->parameters.dpacc = val;
			break;
		case MAX77779_FG_NVM_nFullCapNom:
			model_data->parameters.fullcapnom = val;
			break;
		case MAX77779_FG_NVM_nVEmpty:
			model_data->parameters.v_empty = val;
			break;
		case MAX77779_FG_NVM_nQRTable00:
			model_data->parameters.qresidual00 = val;
			break;
		case MAX77779_FG_NVM_nQRTable10:
			model_data->parameters.qresidual10 = val;
			break;
		case MAX77779_FG_NVM_nQRTable20:
			model_data->parameters.qresidual20 = val;
			break;
		case MAX77779_FG_NVM_nQRTable30:
			model_data->parameters.qresidual30 = val;
			break;
		case MAX77779_FG_NVM_nRComp0:
			model_data->parameters.rcomp0 = val;
			break;
		case MAX77779_FG_NVM_nTempCo:
			model_data->parameters.tempco = val;
			break;
		case MAX77779_FG_NVM_nIChgTerm:
			model_data->parameters.ichgterm = val;
			break;
		case MAX77779_FG_NVM_nMiscCfg:
			model_data->parameters.misccfg = val;
			break;
		case MAX77779_FG_NVM_nModelCfg:
			model_data->parameters.modelcfg = val;
			break;
		case MAX77779_FG_NVM_nThermCfg:
			model_data->parameters.thermcfg = val;
			break;
		case MAX77779_FG_NVM_nFilterCfg:
			model_data->parameters.filtercfg = val;
			break;
		default:
			dev_err(model_data->dev, "@%d: reg=%x out of range\n", index, reg);
			return -EINVAL;
		}

		for ( ; index < max && buf[index] != '\n'; index++)
			;
	}

	return 0;
}

ssize_t max77779_gmsr_state_cstr(char *buf, int max)
{
	struct model_state_save saved_data;
	int ret = 0, len = 0;

	ret = gbms_storage_read(GBMS_TAG_GMSR, &saved_data, GBMS_GMSR_LEN);
	if (ret < 0)
		return ret;
	if (ret != GBMS_GMSR_LEN)
		return -EIO;

	len = scnprintf(&buf[len], max - len,
			"rcomp0     :%04X\ntempco     :%04X\n"
			"fullcaprep :%04X\ncycles     :%04X\n"
			"fullcapnom :%04X\nqresidual00:%04X\n"
			"qresidual10:%04X\nqresidual20:%04X\n"
			"qresidual30:%04X\n",
			saved_data.rcomp0, saved_data.tempco,
			saved_data.fullcaprep, saved_data.cycles,
			saved_data.fullcapnom, saved_data.qrtable00,
			saved_data.qrtable10, saved_data.qrtable20,
			saved_data.qrtable30);

	return len;
}

/* custom model parameters */
int max77779_fg_model_cstr(char *buf, int max, const struct max77779_model_data *model_data)
{
	int i, len;

	if (!model_data->custom_model || !model_data->custom_model_size)
		return -EINVAL;

	for (len = 0, i = 0; i < model_data->custom_model_size; i += 1)
		len += scnprintf(&buf[len], max - len, "%x: %04x\n",
				 MAX77779_FG_MODEL_START + i,
				 model_data->custom_model[i]);
	return len;
}

/* custom model parameters */
int max77779_fg_model_sscan(struct max77779_model_data *model_data, const char *buf, int max)
{
	int ret, index, reg, val, fg_model_end;

	if (!model_data->custom_model)
		return -EINVAL;

	/* use the default size */
	if (!model_data->custom_model_size)
		model_data->custom_model_size = MAX77779_FG_MODEL_SIZE;

	fg_model_end = MAX77779_FG_MODEL_START + model_data->custom_model_size;
	for (index = 0; index < max ; index += 1) {
		ret = sscanf(&buf[index], "%x:%x", &reg, &val);
		if (ret != 2) {
			dev_err(model_data->dev, "@%d: sscan error %d\n",
				index, ret);
			return -EINVAL;
		}

		dev_info(model_data->dev, "@%d: reg=%x val=%x\n", index, reg, val);

		if (reg >= MAX77779_FG_MODEL_START && reg < fg_model_end) {
			const int offset = reg - MAX77779_FG_MODEL_START;

			model_data->custom_model[offset] = val;
		}

		for ( ; index < max && buf[index] != '\n'; index++)
			;
	}

	return 0;
}

static int max77779_init_custom_parameters(struct device *dev,
					   struct max77779_custom_parameters *cp,
					   struct device_node *node)
{
	const char *propname = "max77779,fg-params";
	const int cnt_default = sizeof(*cp) / 2;
	int ret, cnt;

	memset(cp, 0, sizeof(*cp));

	cnt = of_property_count_elems_of_size(node, propname, sizeof(u16));
	if (cnt < 0)
		return -ENODATA;

	if (cnt != cnt_default) {
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

void max77779_free_data(struct max77779_model_data *model_data)
{
	devm_kfree(model_data->dev, model_data);
}

/* mark model_data->model_version as invalid to prevent from reloading if failed to read */
void *max77779_init_data(struct device *dev, struct device_node *node,
			 struct maxfg_regmap *regmap, struct maxfg_regmap *debug_regmap)
{
	const char *propname = "max77779,fg-model";
	struct max77779_model_data *model_data;
	int cnt, ret;
	u16 *model;
	u32 temp;

	model_data = devm_kzalloc(dev, sizeof(*model_data), GFP_KERNEL);
	if (!model_data) {
		dev_err(dev, "fg-model: %s not found\n", propname);
		return ERR_PTR(-ENOMEM);
	}

	model = devm_kmalloc_array(dev, MAX77779_FG_MODEL_SIZE, sizeof(u16),
				   GFP_KERNEL);
	if (!model) {
		dev_err(dev, "fg-model: out of memory\n");
		return ERR_PTR(-ENOMEM);
	}

	cnt = of_property_count_elems_of_size(node, propname, sizeof(u16));
	if (cnt != MAX77779_FG_MODEL_SIZE) {
		dev_err(dev, "fg-model: not found, or invalid %d\n", cnt);
		model_data->model_version = MAX77779_FG_INVALID_VERSION;
	} else {
		ret = of_property_read_u16_array(node, propname, model, cnt);
		if (ret < 0) {
			dev_err(dev, "fg-model: no data cnt=%d %s %s: %d\n",
				cnt, node->name, propname, ret);
			model_data->model_version = MAX77779_FG_INVALID_VERSION;
		} else {
			model_data->custom_model_size = cnt;
		}
	}

	model_data->force_reset_model_data =
		of_property_read_bool(node, "max77779,force-reset-model-data");

	/*
	 * Initial values: check max_m5_model_read_state() for the registers
	 * updated from max1720x_model_work()
	 */
	ret = max77779_init_custom_parameters(dev, &model_data->parameters, node);
	if (ret < 0) {
		dev_err(dev, "fg-params: not found ret=%d\n", ret);
		model_data->model_version = MAX77779_FG_INVALID_VERSION;
	}

	if (model_data->model_version != MAX77779_FG_INVALID_VERSION) {
		ret = of_property_read_u32(node, "max77779,model-version", &temp);
		if (ret < 0 || temp > 255)
			temp = MAX77779_FG_INVALID_VERSION;
		model_data->model_version = temp;
	}

	crc8_populate_msb(max77779_fg_crc8_table, MAX7779_FG_CRC8_POLYNOMIAL);

	model_data->custom_model = model;
	model_data->debug_regmap = debug_regmap;
	model_data->regmap = regmap;
	model_data->dev = dev;

	return model_data;
}

/*
 * The model data's custom_parameters contain values for FullSOCThr and MISCCFG.
 *  - before the model data is loaded using max77779_fg_model_load,
 *    these values must be updated based on aafv.
 */
void max77779_model_apply_aaf_fullsoc(struct max77779_model_data *model_data,
				      const struct aafv_fg_config *cfg)
{
	struct max77779_custom_parameters *cp = &model_data->parameters;

	cp->fullsocthr = percentage_to_reg(cfg->fullsoc);
	cp->misccfg = (MAX77779_FG_MiscCfg_FUS_CLEAR & cp->misccfg) |
		      (cfg->fus << MAX77779_FG_MiscCfg_FUS_SHIFT);
}
