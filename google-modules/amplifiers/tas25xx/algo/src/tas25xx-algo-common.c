/*
 ** ============================================================================
 ** Copyright (c) 2016  Texas Instruments Inc.
 **
 ** This program is free software; you can redistribute it and/or modify it
 ** under the terms of the GNU General Public License as published by the Free
 ** Software Foundation; version 2.
 **
 ** This program is distributed in the hope that it will be useful, but WITHOUT
 ** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 ** FITNESS
 ** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 ** details
 **
 ** File:
 **     tas25xx-algo-common.c
 **
 ** Description:
 **     Basic common interface required for the algorithm.
 **
 ** ============================================================================
 */
#include <sound/soc.h>
#include <linux/types.h>
#include <linux/device.h>
#include "algo/src/tas25xx-calib-validation.h"
#include "algo/inc/tas25xx-calib.h"
#include "algo/inc/tas_smart_amp_v2.h"
#include "tas25xx-algo-bin-utils.h"
#include "tas25xx-algo-intf.h"
#include "tas25xx.h"

#define TDM_MAX_CHANNELS 4

#if IS_ENABLED(CONFIG_PLATFORM_QCOM)
#include  <dsp/tas_qualcomm.h>
void tas25xx_parse_algo_bin_qdsp_intf(int ch_count, u8 *buf);
#endif /*CONFIG_PLATFORM_XXX*/

#if IS_ENABLED(CONFIG_TAS25XX_CALIB_VAL_BIG)
#include "tas25xx-calib-validation.h"
#endif

#if IS_ENABLED(CONFIG_TISA_SYSFS_INTF)
#include "tas25xx-sysfs-debugfs-utils.h"
#endif

#if IS_ENABLED(CONFIG_TISA_KBIN_INTF)
static struct device *s_device;
#endif

static char s_channel_map[TDM_MAX_CHANNELS] = {0, 1, 2, 3};
static int s_allow_algo_controls;
static int s_algo_intf_initialised;

void tas25xx_algo_enable_common_controls(int value)
{
	s_allow_algo_controls = value;
}

struct soc_multi_control_ch_map {
	int min, max, platform_max, count;
	unsigned int reg, rreg, shift, rshift, invert;
};

#if IS_ENABLED(CONFIG_TISA_KBIN_INTF)
static void tas25xx_algo_set_device(struct device *dev)
{
	s_device = dev;
}

struct device *tas25xx_algo_get_device(void)
{
	return s_device;
}
#endif

bool tas25xx_set_iv_bit_fomat(int iv_data_width, int vbat, int update_now)
{
	int32_t param_id;
	int32_t ret;
	int32_t iv_width_vbat;
	bool success;

	success = true;

	if (update_now) {
		/* New method
		 * send IV Width and VBat info
		 */
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_IV_WIDTH_VBAT_MON, 1, CHANNEL0);
		iv_width_vbat = (iv_data_width & 0xFFFF) | ((vbat & 0xFFFF) << 16);
		pr_info("TI-SmartPA: %s: Sending IV-width=%d,Vbat-mon=%d, iv_width_vbat=%d\n",
			__func__, iv_data_width, vbat, iv_width_vbat);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&iv_width_vbat, param_id,
				TAS_SET_PARAM, sizeof(int32_t), TISA_MOD_RX);
		if (ret < 0) {
			pr_err("TI-SmartPA: %s: Failed to set config TAS_SA_IV_WIDTH_VBAT_MON\n",
				__func__);
			success = false;
		}
	}

	return success;
}
EXPORT_SYMBOL(tas25xx_set_iv_bit_fomat);

void tas25xx_send_channel_mapping(void)
{
	int ret;
	int32_t param_id;

	param_id = TAS_CALC_PARAM_IDX(TAS_PCM_CHANNEL_MAPPING,
			TDM_MAX_CHANNELS/sizeof(int32_t), CHANNEL0);
	ret = tas25xx_smartamp_algo_ctrl((u8 *)s_channel_map, param_id,
			TAS_SET_PARAM, TDM_MAX_CHANNELS, TISA_MOD_RX);
	if (ret)
		pr_err("TI-SmartPA: %s, Error sending, ret=%d\n",
			__func__, ret);
}

#if IS_ENABLED(CONFIG_TAS25XX_CALIB_VAL_BIG) || IS_ENABLED(CONFIG_TISA_SYSFS_INTF)
static void tas25xx_send_drv_op_mode(void)
{
	int ret = -EINVAL;
	int32_t param_id;
	int32_t drv_opmode;

	drv_opmode = tas25xx_get_drv_channel_opmode();
	if (drv_opmode > 0) {
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_SET_DRV_OP_MODE, 1, CHANNEL0);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&drv_opmode, param_id,
			TAS_SET_PARAM, sizeof(int32_t), TISA_MOD_RX);
	}

	pr_err("TI-SmartPA: %s, sending drv_opmode=%d, ret=%d\n",
			__func__, drv_opmode, ret);
}
#endif

/*
 *Common Mixer Controls required for all interfaces
 */
/* get api shall return the cached values from set api.
 *Default mapping if not set
 */
static int tas25xx_algo_get_channel_map(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int i;

	for (i = 0; i < TDM_MAX_CHANNELS; i++) {
		pr_debug("TI-SmartPA: %s idx=%d, value=%d\n", __func__,
			i, (int)s_channel_map[i]);
		ucontrol->value.integer.value[i] =
			(unsigned int) s_channel_map[i];
	}

	return 0;
}

static int tas25xx_algo_put_channel_map(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int i;
	char channel_map[TDM_MAX_CHANNELS];

	for (i = 0; i < TDM_MAX_CHANNELS; i++) {
		channel_map[i] = (char)(ucontrol->value.integer.value[i]);
		pr_debug("TI-SmartPA: %s mapping - index %d = channel %d\n",
			__func__, i, channel_map[i]);
	}

	memcpy(s_channel_map, channel_map, sizeof(channel_map));
	return 0;
}

static int tas25xx_algo_channel_map_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_multi_control_ch_map *mc =
		(struct soc_multi_control_ch_map *)kcontrol->private_value;

	pr_debug("TI-SmartPA: %s count=%d, platform_max=%d\n",
		__func__, mc->count,
		mc->platform_max);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = mc->count;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mc->platform_max;

	return 0;
}

static int tas25xx_get_digital_gain_common(int channel)
{
	int ret = 0;
	int digital_gain = 0;
	int param_id = 0;

	if (s_allow_algo_controls) {
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_DIGITAL_GAIN, 1, channel);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&digital_gain, param_id,
				TAS_GET_PARAM, sizeof(uint32_t),
				TISA_MOD_RX);
		if (ret < 0)
			pr_err("TI-SmartPA: %s: failed to get digital gain\n",
				__func__);
		else
			ret = digital_gain;
	} else {
		pr_info("TI-SmartPA: %s: Not calling algo. Playback inactive\n",
				__func__);
		ret = 0;
	}

	return ret;
}

static int tas25xx_get_digital_gain_left(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = tas25xx_get_digital_gain_common(CHANNEL0);

	pUcontrol->value.integer.value[0] = ret;
	pr_info("TI-SmartPA: %s: got digital gain for left ch %d\n",
		__func__, ret);

	return 0;
}

static int tas25xx_get_digital_gain_right(struct snd_kcontrol *pKcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int ret = tas25xx_get_digital_gain_common(CHANNEL1);

	pUcontrol->value.integer.value[0] = ret;
	pr_info("TI-SmartPA: %s: Getting digital gain right ch %d\n",
		__func__, ret);

	return 0;
}

static int tas25xx_set_digital_gain_common(int digital_gain, int channel)
{
	int ret;
	int param_id = 0;
	int gain = digital_gain;

	pr_info("TI-SmartPA: %s: set gain =%d =>(gain - 108)dB=%d, ch=%d\n",
		__func__, gain, (gain - 108), channel);

	if (s_allow_algo_controls) {
		param_id = TAS_CALC_PARAM_IDX(TAS_SA_DIGITAL_GAIN, 1, channel);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&gain, param_id,
				TAS_SET_PARAM, sizeof(uint32_t), TISA_MOD_RX);
	} else {
		pr_info("TI-SmartPA: %s: Not calling algo.Playback inactive\n",
				__func__);
		ret = -EAGAIN;
	}

	return ret;
}

static int tas25xx_set_digital_gain_left(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int digital_gain = pUcontrol->value.integer.value[0];

	return tas25xx_set_digital_gain_common(digital_gain, CHANNEL0);
}

static int tas25xx_set_digital_gain_right(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *pUcontrol)
{
	int digital_gain = pUcontrol->value.integer.value[0];

	return tas25xx_set_digital_gain_common(digital_gain, CHANNEL1);
}

static const struct snd_kcontrol_new tas25xx_algo_common_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "TAS25XX_ALGO_RX_CH_MAP",
		.info = tas25xx_algo_channel_map_info,
		.get = tas25xx_algo_get_channel_map,
		.put = tas25xx_algo_put_channel_map,
		.private_value =
			(unsigned long) &(struct soc_multi_control_ch_map) {
				.reg = SND_SOC_NOPM,
				.shift = 0,
				.rshift = 0,
				.max = TDM_MAX_CHANNELS,
				.count = TDM_MAX_CHANNELS,
				.platform_max = 16,
				.invert = 0,
			}
	},
	SOC_SINGLE_EXT("TAS25XX_ALGO_DIGITAL_GAIN_A", SND_SOC_NOPM, 0, 126, 0,
			tas25xx_get_digital_gain_left, tas25xx_set_digital_gain_left),
	SOC_SINGLE_EXT("TAS25XX_ALGO_DIGITAL_GAIN_B", SND_SOC_NOPM, 0, 126, 0,
			tas25xx_get_digital_gain_right, tas25xx_set_digital_gain_right),
};

int tas25xx_start_algo_processing(int iv_width, int vbat_on)
{
	if (!tas25xx_check_if_algo_ctrl_bypassed(0)) {
		tas25xx_algo_enable_common_controls(1);
#if IS_ENABLED(CONFIG_TAS25XX_CALIB_VAL_BIG) || IS_ENABLED(CONFIG_TISA_SYSFS_INTF)
		tas_set_algo_run_status(1);
		tas25xx_send_channel_mapping();
		tas25xx_send_drv_op_mode();
		tas25xx_send_algo_calibration();
#endif /* CONFIG_TAS25XX_CALIB_VAL_BIG || CONFIG_TISA_SYSFS_INTF */
		if (!tas25xx_set_iv_bit_fomat(iv_width, vbat_on, 1))
			pr_info("TI-SmartPA: Error sending IV/Vbat infor to algo");
	} else {
		pr_info("TI-SmartPA: %s algo comm not required", __func__);
	}

	return 0;
}

int tas25xx_stop_algo_processing(void)
{
	if (!tas25xx_check_if_algo_ctrl_bypassed(0)) {
#if IS_ENABLED(CONFIG_TAS25XX_CALIB_VAL_BIG)
		tas25xx_update_big_data();
		tas_set_algo_run_status(0);
#endif /* CONFIG_TAS25XX_CALIB_VAL_BIG */
		tas25xx_algo_enable_common_controls(0);
	}

#if IS_ENABLED(CONFIG_TISA_KBIN_INTF)
		tas25xx_algo_set_inactive();
#endif /* CONFIG_TISA_KBIN_INTF */

	return 0;
}

void tas25xx_parse_algo_bin(int ch_count, u8 *buf)
{
	uint32_t *ptr = (uint32_t *)buf;
#if IS_ENABLED(CONFIG_PLATFORM_QCOM)
	tas25xx_parse_algo_bin_qdsp_intf(ch_count, buf);
#endif
	/* skip 10 * 4bytes */
	ptr += 10;
	buf = (u8 *)ptr;
#if IS_ENABLED(CONFIG_TISA_SYSFS_INTF)
	tas25xx_parse_algo_bin_sysfs(ch_count, buf);
#endif
}

void tas_smartamp_add_algo_controls(struct snd_soc_component *codec,
	struct device *dev, int number_of_channels)
{
	pr_info("TI-SmartPA: %s: Adding smartamp controls\n", __func__);

	tas25xx_smartamp_set_card(codec->card);
	tas25xx_smartamp_alg_intf_init();

#if IS_ENABLED(CONFIG_TISA_SYSFS_INTF)
	pr_info("TI-SmartPA: %s: Adding debugfs controls\n", __func__);
	tas_smartamp_add_algo_controls_debugfs(codec, number_of_channels);
#endif

#if IS_ENABLED(CONFIG_TAS25XX_CALIB_VAL_BIG)
	tas25xx_algo_add_calib_valid_bigdata(number_of_channels);
#endif

	/* Some common interfaces used*/
	snd_soc_add_component_controls(codec, tas25xx_algo_common_controls,
		ARRAY_SIZE(tas25xx_algo_common_controls));

#if IS_ENABLED(CONFIG_TISA_BIN_INTF)
	pr_info("TI-SmartPA: %s: Adding bin intf controls\n", __func__);
	tas_smartamp_add_codec_mixer_controls(codec);
#endif

#if IS_ENABLED(CONFIG_TISA_KBIN_INTF)
	tas_smartamp_add_codec_mixer_controls(codec);
	tas25xx_algo_set_device(dev);
	bin_file_set_device(dev);
	bin_file_parse_init();

	pr_info("TI-SmartPA: %s: Initialising kbin file done\n", __func__);
#endif

	tas_calib_init();

	s_algo_intf_initialised = 1;
}
EXPORT_SYMBOL(tas_smartamp_add_algo_controls);

void tas_smartamp_remove_algo_controls(struct snd_soc_component *codec)
{
	if (!s_algo_intf_initialised)
		return;

#if IS_ENABLED(CONFIG_TISA_SYSFS_INTF)
	tas_smartamp_remove_algo_controls_debugfs(codec);
#endif

#if IS_ENABLED(CONFIG_TAS25XX_CALIB_VAL_BIG)
	tas25xx_algo_remove_calib_valid_bigdata();
#endif

#if IS_ENABLED(CONFIG_TISA_KBIN_INTF)
	tas_smartamp_kbin_deinitalize();
	tas25xx_algo_set_device(NULL);
	bin_file_set_device(NULL);
	bin_file_parse_deinit();
#endif

	tas25xx_smartamp_alg_intf_deinit();
	tas_calib_exit();
	s_algo_intf_initialised = 0;
}
EXPORT_SYMBOL(tas_smartamp_remove_algo_controls);
