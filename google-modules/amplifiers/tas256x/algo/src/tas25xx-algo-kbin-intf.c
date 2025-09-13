/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
**
** File:
**     tas25xx-algo-kbin-intf.c
**
** Description:
**     Algorithm interface wrapper to open the parameter bin file in kernel
**     using "request_firmware" API
**
** =============================================================================
*/
#include <sound/soc.h>
#include <linux/delay.h> /* usleep_range */
#include <linux/kernel.h>
#include <linux/types.h> /* atomic_t */
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include "algo/inc/tas_smart_amp_v2.h"
#include "algo/inc/tas25xx-calib.h"
#include "tas25xx-algo-bin-utils.h"
#include "tas25xx-algo-intf.h"

#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
#include <linux/codec-misc.h>
#endif

#define POISON_VAL		0xDEADDEAD
#define MAX_STRING		(300)
#define QFORMAT19		19
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

static struct delayed_work query_tisa_algo_wrk;
static atomic_t algo_active = ATOMIC_INIT(0);

/*Master Control to Bypass the Smartamp TI CAPIv2 module*/
static int s_tas_smartamp_enable;
static int s_allow_dsp_query;
static int s_calib_test_flag;

/* fwd declaration */
static int tas25xx_get_re_common(int channel);
struct device *tas25xx_algo_get_device(void);

/*Max value supported is 2^8*/
static uint8_t trans_val_to_user_m(uint32_t val, uint8_t qformat)
{
	uint32_t ret = (uint32_t)(((long long)val * 1000) >> qformat) % 1000;

	return (uint8_t)(ret / 10);
}

/*Max value supported is 2^8*/
static uint8_t trans_val_to_user_i(uint32_t val, uint8_t qformat)
{
	return ((val * 100) >> qformat) / 100;
}

#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
static int trans_val_to_int(int val, int scale, int qformat)
{
	return trans_val_to_user_i(val, qformat) * scale +
		trans_val_to_user_m(val, qformat, scale);
}
#endif

#ifdef CONFIG_SET_RE_IN_KERNEL
static int get_calibrated_re_tcalib(uint32_t *rdc_fix, uint32_t *tv_fix, int channel_count)
{
	int ret = 0;
	int calib_re_sz;
	uint32_t *calib_data;
	struct device *dev;
	const struct firmware *fw_entry;

	static uint32_t s_rdc_fix[2] = { POISON_VAL, POISON_VAL };
	static uint32_t s_tv_fix = POISON_VAL;

	if ((s_rdc_fix[0] == POISON_VAL) &&
			(s_rdc_fix[1] == POISON_VAL)) {
		ret = -EINVAL;
		dev = tas25xx_algo_get_device();
		if (dev)
			ret = request_firmware(&fw_entry, "smartamp_calib.bin", dev);

		if (ret) {
			pr_err("[TI-SmartPA: %s] request_firmware failed, err=%d!\n", __func__, ret);
			return ret;
		}

		calib_re_sz = (channel_count * sizeof(uint32_t));
		calib_data = (int32_t *) fw_entry->data;
		/*calib re + temp*/
		if (fw_entry->size < (calib_re_sz + 1)) {
			memcpy(s_rdc_fix, calib_data, calib_re_sz);
			memcpy(&s_tv_fix, calib_data + calib_re_sz, sizeof(uint32_t));
		}

		release_firmware(fw_entry);
	}

	if (ret == 0) {
		if (rdc_fix) {
			rdc_fix[0] = (uint32_t)s_rdc_fix[0];
			if (channel_count == 2)
				rdc_fix[1] = (uint32_t)s_rdc_fix[1];
		}
		if (tv_fix)
			*tv_fix = (uint32_t)s_tv_fix;
	}
	return ret;
}
#endif /*CONFIG_SET_RE_IN_KERNEL*/

static void tas25xx_algo_set_inactive(void)
{
	atomic_set(&algo_active, 0);
	s_tas_smartamp_enable = 0;

	pr_info("[TI-SmartPA:%s] algo disabled..", __func__);

	bin_file_set_profile_id(0);
	s_calib_test_flag = 0;
}

static void tas25xx_algo_set_active(void)
{
	atomic_set(&algo_active, 1);
	schedule_delayed_work(&query_tisa_algo_wrk, msecs_to_jiffies(1000));
}

static void query_tisa_algo(struct work_struct *wrk)
{
	int sleep_ms;
	int sleep_us;
	int sleep_us_max;
	int ret;
	int temp;
	int re_l = 0, re_r = 0;
#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
	int scale = 100000;
#endif

	ret = bin_file_get_custom_value_by_idx(0, &sleep_ms);
	if (ret) {
		sleep_ms = 1000;
	}

	sleep_us = sleep_ms * 1000;
	sleep_us_max = sleep_us + 1000;

	pr_info("TI-SmartPA: Re query interval is set to %d(ms)\n", sleep_ms);

	while (atomic_read(&algo_active)) {

		temp = s_allow_dsp_query;
		s_allow_dsp_query = 1;
		re_l = tas25xx_get_re_common(CHANNEL0);
		re_r = tas25xx_get_re_common(CHANNEL1);
		s_allow_dsp_query = temp;

#if IS_ENABLED(CONFIG_SND_SOC_CODEC_DETECT)
		codec_misc_amp_put(0, (long)trans_val_to_int(re_l, scale, QFORMAT19));
		codec_misc_amp_put(1, (long)trans_val_to_int(re_r, scale, QFORMAT19));
#endif
		pr_debug("[TI-SmartPA:%s] Re value is %02d.%02d (%d), %02d.%02d (%d) \n",
			__func__,
			(int32_t)trans_val_to_user_i(re_l, QFORMAT19),
			(int32_t)trans_val_to_user_m(re_l, QFORMAT19, 100),
			re_l,
			(int32_t)trans_val_to_user_i(re_r, QFORMAT19),
			(int32_t)trans_val_to_user_m(re_r, QFORMAT19, 100),
			re_r);

		usleep_range(sleep_us, sleep_us_max);
	}
}

static int tas25xx_send_kbin_params(void)
{
	int ret = -EINVAL;
	int profile_size_bytes = 0;
	int param_id = 0;
	int sent = 0;
	int ch = CHANNEL0;
	int param_sz_rem;
	int current_idx = 0;
	u16 size;
	int i;

	char *profile_data = NULL;
	int *data;

	int per_ch_param_sz = bin_file_get_per_profile_per_channel_param_count();
	int number_of_ch = bin_file_get_number_of_channels();
	bool success = bin_file_get_profile_data(&profile_data, &profile_size_bytes);

	if (success && profile_data && profile_size_bytes) {
		data = (int *)profile_data;
		ret = 0;

		for (i = 0; i < number_of_ch; i++) {
			ch = i + 1;
			sent = 0;
			param_sz_rem = per_ch_param_sz;
			for (current_idx = 0; current_idx < per_ch_param_sz; ) {
				/*size = MIN(1000, param_sz_rem);*/
				size = MIN(100, param_sz_rem);
				param_id = TAS_CALC_PARAM_IDX(current_idx, size, ch);
				ret = tas25xx_smartamp_algo_ctrl((u8 *)data, param_id,
					TAS_SET_PARAM, size * sizeof(int), TISA_MOD_RX);
				if (ret) {
					pr_info("TI-SmartPA: %s data send error = %d\n", __func__, ret);
				}
				data += size;
				sent += size;
				param_sz_rem -= size;
				current_idx += size;

				pr_info("TI-SmartPA: %s send=%d, param_sz_rem=%d(total=%d), last sent sz=%d\n",
					__func__, sent, param_sz_rem, per_ch_param_sz, size);
			}
		}
	}

	/*send swap index*/
	if (ret == 0) {
		int user_data = 1;

		param_id = TAS_CALC_PARAM_IDX(TAS_DSP_SWAP_IDX, 1, CHANNEL0);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&user_data, param_id,
				TAS_SET_PARAM, sizeof(uint32_t), TISA_MOD_RX);

		if (ch == CHANNEL1) {
			param_id = TAS_CALC_PARAM_IDX(TAS_DSP_SWAP_IDX, 1, CHANNEL1);
			ret = tas25xx_smartamp_algo_ctrl((u8 *)&user_data, param_id,
					TAS_SET_PARAM, sizeof(uint32_t), TISA_MOD_RX);
		}
	}

	return ret;

}

/*Control-1: Set Profile*/
static const char *profile_index_text[] = {
	"NONE", TAS_ALGO_PROFILE_LIST, "CALIB" };
static const struct soc_enum profile_index_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(profile_index_text),
			profile_index_text),
};

static int tas25xx_set_profile_kbin(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int profile_id = pUcontrol->value.integer.value[0];
	int max_number_of_profiles =
		ARRAY_SIZE(profile_index_text);

	if ((profile_id >= max_number_of_profiles) || (profile_id < 0))
		return -EINVAL;

	pr_info("TI-SmartPA: %s: Setting profile %s\n",
			__func__, profile_index_text[profile_id]);
	if (profile_id)
		profile_id -= 1;
	else
		return 0;

	bin_file_set_profile_id(profile_id);

	/*playback started*/
	if (s_tas_smartamp_enable) {
		tas25xx_send_kbin_params();
	}

	return ret;
}

static int tas25xx_get_profile_kbin(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int profile_id = bin_file_get_profile_id();
	int max_number_of_profiles =
		ARRAY_SIZE(profile_index_text);

	pUcontrol->value.integer.value[0] = profile_id;
	if ((profile_id < max_number_of_profiles) && (profile_id > -1))
		pr_info("TI-SmartPA: %s: getting profile %s\n",
				__func__, profile_index_text[profile_id]);

	return 0;
}

/*Control-2: Set Calibrated Rdc*/
static int tas25xx_set_Re_common(int re_value_in, int channel)
{
	int ret;
	int param_id = 0;
	int re_value = re_value_in;

	param_id = TAS_CALC_PARAM_IDX(TAS_SA_SET_RE, 1, channel);
	ret = tas25xx_smartamp_algo_ctrl((u8 *)&re_value, param_id,
			TAS_SET_PARAM, sizeof(uint32_t), TISA_MOD_RX);

	return ret;
}

static int tas25xx_set_Re_left(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int re_value = pUcontrol->value.integer.value[0];

	pr_info("TI-SmartPA: %s: Setting Re %d", __func__, re_value);
	return tas25xx_set_Re_common(re_value, CHANNEL0);
}

static int tas25xx_dummy_get(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;

	pUcontrol->value.integer.value[0] = 0;
	return ret;
}

static int tas25xx_dummy_set(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	return 0;
}

/*Control-3: Calibration and Test(F0,Q,Tv) Controls*/
static const char *tas25xx_calib_test_text[] = {
	"NONE",
	"CALIB_START",
	"CALIB_STOP",
	"TEST_START",
	"TEST_STOP"
};

static const struct soc_enum tas25xx_calib_test_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas25xx_calib_test_text),
			tas25xx_calib_test_text),
};

static int tas25xx_calib_test_set_common(int calib_command, int channel)
{
	int ret = 0;
	int param_id = 0;
	int data = 1;

	switch (calib_command) {
	case CALIB_START:
		pr_info("TI-SmartPA: %s: CALIB_START", __func__);
		s_calib_test_flag = 1;
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_CALIB_INIT, 1, channel);
		break;

	case CALIB_STOP:
		pr_info("TI-SmartPA: %s: CALIB_STOP", __func__);
		s_calib_test_flag = 0;
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_CALIB_DEINIT, 1, channel);
		break;

	case TEST_START:
		s_calib_test_flag = 1;
		break;

	case TEST_STOP:
		s_calib_test_flag = 0;
		break;

	default:
		pr_info("TI-SmartPA: %s: no impl calib_command %d\n",
				__func__, calib_command);
		ret = -EINVAL;
		break;
	}

	if (param_id) {
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, param_id,
				TAS_SET_PARAM, sizeof(uint32_t),
				TISA_MOD_RX);
		if (ret < 0) {
			s_calib_test_flag = 0;
			pr_err("TI-SmartPA: %s: Failed to set calib/test, ret=%d\n",
					__func__, ret);
		}
	}

	return ret;
}

static int tas25xx_calib_test_set(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int user_data = pUcontrol->value.integer.value[0];

	ret = tas25xx_calib_test_set_common(user_data, CHANNEL0);

	if (ret == 0) {
#ifdef CONFIG_TAS25XX_ALGO_STEREO
		ret = tas25xx_calib_test_set_common(user_data, CHANNEL1);
#endif
	}

	return ret;
}

/*Control-4: Get Re*/
/*returns -ve error or +ve re value, 0 if not called*/
static int tas25xx_get_re_common(int channel)
{
	int ret = 0;
	int re_value = 0;
	int param_id = 0;

	pr_debug("TI-SmartPA: %s, channel=%d\n", __func__, channel);

	if (s_tas_smartamp_enable && (s_calib_test_flag || s_allow_dsp_query)) {
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_GET_RE, 1, channel);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&re_value, param_id,
				TAS_GET_PARAM, sizeof(uint32_t),
				TISA_MOD_RX);
		if (ret < 0)
			pr_err("TI-SmartPA: %s: Failed to get Re\n", __func__);
		else
			ret = re_value;
	}

	return ret;
}

static int tas25xx_get_re_left(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = tas25xx_get_re_common(CHANNEL0);
	if (ret >= 0) {
		pUcontrol->value.integer.value[0] = ret;
		ret = 0;
		pr_info("TI-SmartPA: %s: Getting Re %d\n", __func__, ret);
	}

	return ret;
}

/*Control-5: Get F0*/
static int tas25xx_get_f0_common(int channel)
{
	int f0_value = 0;
	int param_id = 0;
	int ret = 0;

	pr_info("TI-SmartPA: %s, channel=%d\n", __func__, channel);

	if (s_tas_smartamp_enable && (s_calib_test_flag || s_allow_dsp_query)) {
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_GET_F0, 1, channel);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&f0_value, param_id,
				TAS_GET_PARAM, sizeof(uint32_t),
				TISA_MOD_RX);
		if (ret < 0)
			pr_err("TI-SmartPA: %s: Failed to get F0\n", __func__);
		else
			ret = f0_value;
	}

	return ret;
}

static int tas25xx_get_f0_left(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = tas25xx_get_f0_common(CHANNEL0);

	if (ret >= 0) {
		pUcontrol->value.integer.value[0] = ret;
		pr_info("TI-SmartPA: %s: Getting F0 val=%d\n", __func__, ret);
		ret = 0;
	}

	return ret;
}

/*Control-6: Get Q*/
static int tas25xx_get_q_common(int channel)
{
	int ret = 0;
	int q_value = 0;
	int param_id = 0;

	pr_info("TI-SmartPA: %s, channel=%d", __func__, channel);

	if (s_tas_smartamp_enable && (s_calib_test_flag || s_allow_dsp_query)) {
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_GET_Q, 1, channel);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&q_value, param_id,
				TAS_GET_PARAM, sizeof(uint32_t),
				TISA_MOD_RX);
		if (ret < 0)
			pr_err("TI-SmartPA: %s: Failed to get F0\n", __func__);
		else
			ret = q_value;
	}

	return ret;
}

static int tas25xx_get_q_left(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = tas25xx_get_q_common(CHANNEL0);

	if (ret >= 0) {
		pUcontrol->value.integer.value[0] = ret;
		pr_info("TI-SmartPA: %s: Getting Q %d\n", __func__, ret);
		ret = 0;
	}

	return ret;
}

/*Control-7: Get Tv*/
static int tas25xx_get_tv_common(int channel)
{
	int ret = 0;
	int tv_value = 0;
	int param_id = 0;

	pr_info("TI-SmartPA: %s, channel=%d", __func__, channel);

	if (s_tas_smartamp_enable && (s_calib_test_flag || s_allow_dsp_query)) {
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_GET_TV, 1, channel);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&tv_value, param_id,
				TAS_GET_PARAM, sizeof(uint32_t),
				TISA_MOD_RX);
		if (ret < 0)
			pr_err("TI-SmartPA: %s: Failed to get Tv\n", __func__);
		else
			ret = tv_value;
	}

	return ret;
}

static int tas25xx_get_tv_left(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{

	int ret = tas25xx_get_tv_common(CHANNEL0);

	if (ret >= 0) {
		pUcontrol->value.integer.value[0] = ret;
		pr_info("TI-SmartPA: %s: Getting Tv %d\n", __func__, ret);
		ret = 0;
	}

	return ret;
}

/*Control-8: Smartamp Enable*/
static const char *tas25xx_smartamp_enable_text[] = {
	"DISABLE",
	"ENABLE"
};

static const struct soc_enum tas25xx_smartamp_enable_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas25xx_smartamp_enable_text),
			tas25xx_smartamp_enable_text),
};

static int tas25xx_smartamp_enable_set_kbin(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int param_id = 0;
	int user_data = pUcontrol->value.integer.value[0];
#ifdef CONFIG_SET_RE_IN_KERNEL
	uint32_t calibration_data[3];
#endif

#ifdef CONFIG_TAS25XX_ALGO_STEREO
	int number_of_ch = 2;
#else
	int number_of_ch = 1;
#endif

	if (tas25xx_get_algo_bypass()) {
		pr_info("TI-SmartPA: bypass enabled, not enabling\n");
		return 0;
	}

	pr_info("TI-SmartPA: %s: case %d, number_of_ch=%d\n",
			__func__, user_data, number_of_ch);

	s_tas_smartamp_enable = user_data;
	if (s_tas_smartamp_enable == 0) {
		pr_info("TI-SmartPA: %s: Disable called\n", __func__);
		return 0;
	}

	pr_info("TI-SmartPA: %s: Sending TX Enable\n", __func__);
	param_id = CAPI_V2_TAS_TX_ENABLE;
	ret = tas25xx_smartamp_algo_ctrl((u8 *)&user_data, param_id,
			TAS_SET_PARAM, sizeof(uint32_t),
			TISA_MOD_TX);
	if (ret) {
		pr_err("TI-SmartPA: %s: TX Enable Failed ret = 0x%x\n",
				__func__, ret);
		goto fail_cmd;
	}

	user_data = 1;
	pr_info("TI-SmartPA: %s: Sending RX Enable\n", __func__);
	param_id = CAPI_V2_TAS_RX_ENABLE;
	ret = tas25xx_smartamp_algo_ctrl((u8 *)&user_data, param_id,
			TAS_SET_PARAM, sizeof(uint32_t),
			TISA_MOD_RX);
	if (ret) {
		pr_err("TI-SmartPA: %s: RX Enable Failed ret = 0x%x\n",
				__func__, ret);
		goto fail_cmd;
	}

	tas25xx_send_kbin_params();

	user_data = 0xCCCCB1B1;
	pr_info("TI-SmartPA: %s: Sending TX Config\n", __func__);
	param_id = CAPI_V2_TAS_TX_CFG;
	ret = tas25xx_smartamp_algo_ctrl((u8 *)&user_data, param_id,
			TAS_SET_PARAM, sizeof(uint32_t),
			TISA_MOD_TX);
	if (ret < 0) {
		pr_err("TI-SmartPA: %s: Failed to set config\n", __func__);
		goto fail_cmd;
	}

	user_data = 0xCCCCB1B1;
	pr_info("TI-SmartPA: %s: Sending RX Config\n", __func__);
	param_id = CAPI_V2_TAS_RX_CFG;
	ret = tas25xx_smartamp_algo_ctrl((u8 *)&user_data, param_id,
			TAS_SET_PARAM, sizeof(uint32_t),
			TISA_MOD_RX);
	if (ret < 0) {
		pr_err("TI-SmartPA: %s: Failed to set config\n", __func__);
		goto fail_cmd;
	}

	tas25xx_send_channel_mapping();

	user_data = get_iv_vbat_format();
	param_id = TAS_CALC_PARAM_IDX(TAS_SA_IV_VBAT_FMT, 1, CHANNEL0);
	pr_info("TI-SmartPA: %s: Sending IV,Vbat format %d\n",
			__func__, user_data);
	ret = tas25xx_smartamp_algo_ctrl((u8 *)&user_data, param_id,
			TAS_SET_PARAM, sizeof(uint32_t),
			TISA_MOD_RX);
	if (ret < 0) {
		pr_err("TI-SmartPA: %s: Failed to set config\n", __func__);
		goto fail_cmd;
	}

	s_tas_smartamp_enable = true;

#ifdef CONFIG_SET_RE_IN_KERNEL
	if (number_of_ch == 2)
		ret = get_calibrated_re_tcalib(calibration_data,
				&calibration_data[2], number_of_ch);
	else
		ret = get_calibrated_re_tcalib(calibration_data,
				&calibration_data[1], number_of_ch);

	if (ret) {
		pr_err("[Smartamp:%s] unable to get the calibration data = 0x%x\n",
				__func__, ret);
		/* TODO: Ignore the calibration read error */
		ret = 0;
	} else {
		int32_t t_cal;
		if (number_of_ch == 2) {
			t_cal = calibration_data[2];
			pr_info("[Smartamp:%s] setting re %d,%d and tcal %d\n",
					__func__, calibration_data[0],
					calibration_data[1],
					calibration_data[2]);
		} else {
			t_cal = calibration_data[1];
			pr_info("[Smartamp:%s] setting re %d and tcal %d\n",
					__func__, calibration_data[0],
					calibration_data[1]);
		}

		param_id = TAS_CALC_PARAM_IDX(TAS_SA_SET_TCAL, 1, CHANNEL0);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&t_cal,
				param_id, TAS_SET_PARAM, sizeof(uint32_t),
				TISA_MOD_RX);
		if (ret < 0) {
			pr_err("[Smartamp:%s] Failed to set Tcal\n", __func__);
			goto fail_cmd;
		}

		param_id = TAS_CALC_PARAM_IDX(TAS_SA_SET_RE, 1, CHANNEL0);
		ret = tas25xx_smartamp_algo_ctrl(
				(u8 *)(&(calibration_data[0])),
				param_id, TAS_SET_PARAM, sizeof(uint32_t),
				TISA_MOD_RX);
		if (ret < 0) {
			pr_err("TI-SmartPA: %s: Failed to set Re\n", __func__);
			goto fail_cmd;
		}

		if (number_of_ch == 2) {
			param_id = TAS_CALC_PARAM_IDX(TAS_SA_SET_RE, 1, CHANNEL1);
			ret = tas25xx_smartamp_algo_ctrl((u8 *)&(calibration_data[1]),
					param_id, TAS_SET_PARAM, sizeof(uint32_t), TISA_MOD_RX);
			if (ret < 0) {
				pr_err("TI-SmartPA: %s: Failed to set Re\n",
						__func__);
				goto fail_cmd;
			}
		}
	}
#endif /* CONFIG_SET_RE_IN_KERNEL */

	tas25xx_algo_set_active();

fail_cmd:
	return ret;
}

static int tas25xx_smartamp_enable_get(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int user_data = s_tas_smartamp_enable;

	pUcontrol->value.integer.value[0] = user_data;
	pr_info("TI-SmartPA: %s: case %d(0=DISABLE, 1=ENABLE)\n",
			__func__, user_data);
	return ret;
}

/*Control-9: Smartamp Bypass */
static const char *tas25xx_smartamp_bypass_text[] = {
	"FALSE",
	"TRUE"
};

static const struct soc_enum tas25xx_smartamp_bypass_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas25xx_smartamp_bypass_text), tas25xx_smartamp_bypass_text),
};

static int tas25xx_smartamp_bypass_set(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int user_data = pUcontrol->value.integer.value[0];

	if (s_tas_smartamp_enable) {
		pr_debug("TI-SmartPA: %s: cannot update while smartamp enabled\n",
				__func__);
		return -EINVAL;
	}

	tas25xx_set_algo_bypass (user_data);
	pr_info("TI-SmartPA: %s: case %d(FALSE=0,TRUE=1)\n",
			__func__, user_data);
	return ret;
}

static int tas25xx_smartamp_bypass_get(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;

	pUcontrol->value.integer.value[0] = tas25xx_get_algo_bypass();
	pr_info("TI-SmartPA: %s: case %d\n", __func__, (int)pUcontrol->value.integer.value[0]);
	return ret;
}

/*Control-9: Smartamp Bypass */
static const char *tas25xx_smartamp_debug_enable_text[] = {
	"FALSE",
	"TRUE"
};

static const struct soc_enum tas25xx_smartamp_debug_enable_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas25xx_smartamp_debug_enable_text),
			tas25xx_smartamp_debug_enable_text),
};

static int tas25xx_smartamp_debug_enable_set(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int user_data = pUcontrol->value.integer.value[0];

	s_allow_dsp_query = user_data;
	pr_info("TI-SmartPA: %s: case %d(FALSE=0,TRUE=1)\n",
			__func__, user_data);
	return ret;
}

static int tas25xx_smartamp_debug_enable_get(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;

	pUcontrol->value.integer.value[0] = s_allow_dsp_query;
	pr_info("TI-SmartPA: %s: case %d\n", __func__, s_allow_dsp_query);
	return ret;
}

static int tas25xx_set_spk_id_kbin(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int spk_id = pUcontrol->value.integer.value[0];
	bin_file_set_spk_id(spk_id);
	pr_info("TI-SmartPA: %s: spk-id set %d\n", __func__, spk_id);
	return ret;
}

static int tas25xx_set_t_calib(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = 0;
	int calib_temp = pUcontrol->value.integer.value[0];
	int param_id = 0;

	pr_info("TI-SmartPA: %s: tcalib set %d\n", __func__, calib_temp);
	if (s_tas_smartamp_enable) {
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_SET_TCAL, 1, CHANNEL0);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&calib_temp, param_id,
				TAS_SET_PARAM, sizeof(uint32_t),
				TISA_MOD_RX);
		if (ret < 0)
			pr_err("TI-SmartPA: %s: Failed to set spk id\n",
					__func__);
	}

	return ret;
}

#ifdef CONFIG_TAS25XX_ALGO_STEREO
static int tas25xx_set_Re_right(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int re_value = pUcontrol->value.integer.value[0];

	pr_info("TI-SmartPA: %s: Setting Re %d\n", __func__, re_value);
	return tas25xx_set_Re_common(re_value, CHANNEL1);
}

static int tas25xx_get_re_right(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret;

	ret = tas25xx_get_re_common(CHANNEL1);
	if (ret >= 0) {
		pUcontrol->value.integer.value[0] = ret;
		ret = 0;
		pr_info("TI-SmartPA: %s: Getting Re value=%d\n",
				__func__, ret);
	}

	return ret;
}

static int tas25xx_get_f0_right(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = tas25xx_get_f0_common(CHANNEL1);

	if (ret >= 0) {
		pUcontrol->value.integer.value[0] = ret;
		pr_info("TI-SmartPA: %s: Getting F0 valu=%d\n", __func__, ret);
		ret = 0;
	}

	return ret;
}

static int tas25xx_get_q_right(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = tas25xx_get_q_common(CHANNEL1);

	if (ret >= 0) {
		pUcontrol->value.integer.value[0] = ret;
		pr_info("TI-SmartPA: %s: Getting Q val=%d\n", __func__, ret);
		ret = 0;
	}

	return ret;
}

static int tas25xx_get_tv_right(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = tas25xx_get_tv_common(CHANNEL1);

	if (ret >= 0) {
		pUcontrol->value.integer.value[0] = ret;
		pr_info("TI-SmartPA: %s: Getting Tv %d\n", __func__, ret);
		ret = 0;
	}

	return ret;
}
#endif

static int tas25xx_smartamp_set_bin_updated(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int update = pUcontrol->value.integer.value[0];
	if (update) {
		bin_file_parse_deinit();
		bin_file_parse_init();
	}

	return 0;
}

static const struct snd_kcontrol_new smartamp_tas25xx_mixer_controls[] = {
	SOC_ENUM_EXT("TAS25XX_DEBUG_ENABLE", tas25xx_smartamp_debug_enable_enum[0],
			tas25xx_smartamp_debug_enable_get,
			tas25xx_smartamp_debug_enable_set),
	SOC_ENUM_EXT("TAS25XX_ALGO_PROFILE", profile_index_enum[0],
			tas25xx_get_profile_kbin, tas25xx_set_profile_kbin),
	SOC_ENUM_EXT("TAS25XX_ALGO_CALIB_TEST", tas25xx_calib_test_enum[0],
			tas25xx_dummy_get, tas25xx_calib_test_set),
	SOC_SINGLE_EXT("TAS25XX_SET_SPK_ID", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_dummy_get, tas25xx_set_spk_id_kbin),
	SOC_SINGLE_EXT("TAS25XX_SET_T_CALIB", SND_SOC_NOPM, 0, 100, 0,
			tas25xx_dummy_get, tas25xx_set_t_calib),

	/*left*/
	SOC_SINGLE_EXT("TAS25XX_SET_RE_LEFT", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_dummy_get, tas25xx_set_Re_left),
	SOC_SINGLE_EXT("TAS25XX_GET_RE_LEFT", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_get_re_left, tas25xx_dummy_set),
	SOC_SINGLE_EXT("TAS25XX_GET_F0_LEFT", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_get_f0_left, tas25xx_dummy_set),
	SOC_SINGLE_EXT("TAS25XX_GET_Q_LEFT", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_get_q_left, tas25xx_dummy_set),
	SOC_SINGLE_EXT("TAS25XX_GET_TV_LEFT", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_get_tv_left, tas25xx_dummy_set),

	SOC_SINGLE_EXT("TAS25XX_PARAMS_UPDATED", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_dummy_get, tas25xx_smartamp_set_bin_updated),

	/*Right*/
#ifdef CONFIG_TAS25XX_ALGO_STEREO
	SOC_SINGLE_EXT("TAS25XX_SET_RE_RIGHT", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_dummy_get, tas25xx_set_Re_right),
	SOC_SINGLE_EXT("TAS25XX_GET_RE_RIGHT", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_get_re_right, tas25xx_dummy_set),
	SOC_SINGLE_EXT("TAS25XX_GET_F0_RIGHT", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_get_f0_right, tas25xx_dummy_set),
	SOC_SINGLE_EXT("TAS25XX_GET_Q_RIGHT", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_get_q_right, tas25xx_dummy_set),
	SOC_SINGLE_EXT("TAS25XX_GET_TV_RIGHT", SND_SOC_NOPM, 0, 0x7fffffff, 0,
			tas25xx_get_tv_right, tas25xx_dummy_set),
#endif
	SOC_ENUM_EXT("TAS25XX_SMARTPA_ENABLE", tas25xx_smartamp_enable_enum[0],
			tas25xx_smartamp_enable_get,
			tas25xx_smartamp_enable_set_kbin),

	SOC_ENUM_EXT("TAS25XX_ALGO_BYPASS", tas25xx_smartamp_bypass_enum[0],
			tas25xx_smartamp_bypass_get,
			tas25xx_smartamp_bypass_set),
};
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
void tas_smartamp_add_codec_mixer_controls(struct snd_soc_component *codec)
#else
void tas_smartamp_add_codec_mixer_controls(struct snd_soc_codec *codec)
#endif
{
	pr_debug("TI-SmartPA: %s: Adding smartamp controls\n", __func__);

	/*Initialize all to global variables to 0s*/
	s_tas_smartamp_enable = 0;
	s_allow_dsp_query = 0;
	s_calib_test_flag = 0;

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE

	snd_soc_add_component_controls(codec, smartamp_tas25xx_mixer_controls,
		ARRAY_SIZE(smartamp_tas25xx_mixer_controls));

#else
	snd_soc_add_codec_controls(codec, smartamp_tas25xx_mixer_controls,
		ARRAY_SIZE(smartamp_tas25xx_mixer_controls));
#endif
	INIT_DELAYED_WORK(&query_tisa_algo_wrk, query_tisa_algo);

}
EXPORT_SYMBOL(tas_smartamp_add_codec_mixer_controls);

void tas_smartamp_kbin_deinitalize (void)
{
	tas25xx_algo_set_inactive();
	cancel_delayed_work_sync(&query_tisa_algo_wrk);
}
