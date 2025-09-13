/*
 * ALSA SoC Texas Instruments TAS256X High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2016 Texas Instruments, Inc.
 *
 * Author: saiprasad
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#if IS_ENABLED(CONFIG_TAS256X_REGMAP)
#ifdef CONFIG_DYNAMIC_DEBUG
#define DEBUG 5
#endif
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqnr.h>
#include <linux/pm.h>
#include <linux/version.h>
#include "physical_layer/inc/tas256x.h"
#include "logical_layer/inc/tas256x-logic.h"
#include "physical_layer/inc/tas256x-device.h"
#include "os_layer/inc/tas256x-codec.h"
#include "os_layer/inc/tas256x-regmap.h"
#include "misc/tas256x-misc.h"

#if IS_ENABLED(CONFIG_TAS25XX_ALGO)
#include "algo/inc/tas_smart_amp_v2.h"

#if IS_ENABLED(CONFIG_PLATFORM_QCOM)
#include <dsp/tas_qualcomm.h>
static dc_detection_data_t s_dc_detect;
#endif /*CONFIG_PLATFORM_QCOM*/
#endif /*CONFIG_TAS25XX_ALGO*/
/*For mixer_control implementation*/
#define MAX_STRING	200

static const char *dts_tag[][3] = {
	{
		"ti,left-channel",
		"ti,reset-gpio",
		"ti,irq-gpio"
	},
	{
		"ti,right-channel",
		"ti,reset-gpio2",
		"ti,irq-gpio2"
	}
};

static const char *reset_gpio_label[2] = {
	"TAS256X_RESET", "TAS256X_RESET2"
};

static const char *irq_gpio_label[2] = {
	"TAS256X-IRQ", "TAS256X-IRQ2"
};

static int tas256x_regmap_write(void *plat_data, unsigned int i2c_addr,
	unsigned int reg, unsigned int value)
{
	int nResult = 0;
	int retry_count = TAS256X_I2C_RETRY_COUNT;
	struct linux_platform *platform_data =
		(struct linux_platform *)plat_data;

	if (platform_data->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	platform_data->client->addr = i2c_addr;
	while (retry_count--) {
		nResult = regmap_write(platform_data->regmap, reg,
			value);
		if (nResult >= 0)
			break;
		msleep(20);
	}
	if (retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas256x_regmap_bulk_write(void *plat_data, unsigned int i2c_addr,
	unsigned int reg, unsigned char *pData,
	unsigned int nLength)
{
	int nResult = 0;
	int retry_count = TAS256X_I2C_RETRY_COUNT;
	struct linux_platform *platform_data =
		(struct linux_platform *)plat_data;

	if (platform_data->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	platform_data->client->addr = i2c_addr;
	while (retry_count--) {
		nResult = regmap_bulk_write(platform_data->regmap, reg,
			 pData, nLength);
		if (nResult >= 0)
			break;
		msleep(20);
	}
	if (retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas256x_regmap_read(void *plat_data, unsigned int i2c_addr,
	unsigned int reg, unsigned int *value)
{
	int nResult = 0;
	int retry_count = TAS256X_I2C_RETRY_COUNT;
	struct linux_platform *platform_data =
		(struct linux_platform *)plat_data;

	if (platform_data->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	platform_data->client->addr = i2c_addr;
	while (retry_count--) {
		nResult = regmap_read(platform_data->regmap, reg,
			value);
		if (nResult >= 0)
			break;
		msleep(20);
	}
	if (retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas256x_regmap_bulk_read(void *plat_data, unsigned int i2c_addr,
	unsigned int reg, unsigned char *pData,
	unsigned int nLength)
{
	int nResult = 0;
	int retry_count = TAS256X_I2C_RETRY_COUNT;
	struct linux_platform *platform_data =
		(struct linux_platform *)plat_data;

	if (platform_data->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	platform_data->client->addr = i2c_addr;
	while (retry_count--) {
		nResult = regmap_bulk_read(platform_data->regmap, reg,
			 pData, nLength);
		if (nResult >= 0)
			break;
		msleep(20);
	}
	if (retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas256x_regmap_update_bits(void *plat_data, unsigned int i2c_addr,
	unsigned int reg, unsigned int mask,
	unsigned int value)
{
	int nResult = 0;
	int retry_count = TAS256X_I2C_RETRY_COUNT;
	struct linux_platform *platform_data =
		(struct linux_platform *)plat_data;

	if (platform_data->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	platform_data->client->addr = i2c_addr;
	while (retry_count--) {
		nResult = regmap_update_bits(platform_data->regmap, reg,
			mask, value);
		if (nResult >= 0)
			break;
		msleep(20);
	}
	if (retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static char const *tas2564_rx_mode_text[] = {"Speaker", "Receiver"};

static const struct soc_enum tas2564_rx_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas2564_rx_mode_text),
		tas2564_rx_mode_text),
};

static int tas2564_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct tas256x_priv *p_tas256x = NULL;
	struct linux_platform *plat_data = NULL;
	int ret = -1;

	if (codec == NULL) {
		pr_err("%s:codec is NULL\n", __func__);
		return ret;
	}

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	if (p_tas256x == NULL) {
		pr_err("%s:p_tas256x is NULL\n", __func__);
		return ret;
	}
	plat_data = (struct linux_platform *) p_tas256x->platform_data;

	if (strnstr(ucontrol->id.name, "LEFT", MAX_STRING))
		ret = tas2564_rx_mode_update(p_tas256x,
			ucontrol->value.integer.value[0], channel_left);
	else if (strnstr(ucontrol->id.name, "RIGHT", MAX_STRING))
		ret = tas2564_rx_mode_update(p_tas256x,
			ucontrol->value.integer.value[0], channel_right);
	else
		dev_err(plat_data->dev, "Invalid Channel %s\n",
			ucontrol->id.name);

	return ret;
}

static int tas2564_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	int ret = -1;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct linux_platform *plat_data = NULL;
	struct tas256x_priv *p_tas256x = NULL;

	if (codec == NULL) {
		pr_err("%s:codec is NULL\n", __func__);
		return ret;
	}

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	if (p_tas256x == NULL) {
		pr_err("%s:p_tas256x is NULL\n", __func__);
		return ret;
	}
	plat_data = (struct linux_platform *) p_tas256x->platform_data;

	if (strnstr(ucontrol->id.name, "LEFT", MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[0]->rx_mode;
	else if (strnstr(ucontrol->id.name, "RIGHT", MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[1]->rx_mode;
	else
		dev_err(plat_data->dev, "Invalid Channel %s\n",
			ucontrol->id.name);

	return 0;
}

static int tas256x_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct tas256x_priv *p_tas256x = NULL;
	int ret = -1;

	if ((codec == NULL) || (mc == NULL)) {
		pr_err("%s:codec or control is NULL\n", __func__);
		return ret;
	}

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	if (p_tas256x == NULL) {
		pr_err("%s:p_tas256x is NULL\n", __func__);
		return ret;
	}

	if (ucontrol->value.integer.value[0] > mc->max)
		return ret;

	switch (mc->reg) {
	case DVC_PCM:
		ret = tas256x_update_playback_volume(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case LIM_MAX_ATN:
		ret = tas256x_update_lim_max_attenuation(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case LIMB_INF_PT:
		ret = tas256x_update_lim_inflection_point(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case LIMB_SLOPE:
		ret = tas256x_update_lim_slope(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case LIMB_ATK_RT:
		ret = tas256x_update_limiter_attack_rate(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case LIMB_RLS_RT:
		ret = tas256x_update_limiter_release_rate(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case LIMB_RLS_ST:
		ret = tas256x_update_limiter_release_step_size(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case LIMB_ATK_ST:
		ret = tas256x_update_limiter_attack_step_size(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case BOP_ATK_RT:
		ret = tas256x_update_bop_attack_rate(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case BOP_ATK_ST:
		ret = tas256x_update_bop_attack_step_size(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case BOP_HLD_TM:
		ret = tas256x_update_bop_hold_time(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case BST_VREG:
		ret = tas256x_update_boost_voltage(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case BST_ILIM:
		ret = tas256x_update_current_limit(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case LIMB_TH_MAX:
		ret = tas256x_update_lim_max_thr(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case LIMB_TH_MIN:
		ret = tas256x_update_lim_min_thr(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case BOP_TH:
		ret = tas256x_update_bop_thr(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case BOSD_TH:
		ret = tas256x_update_bosd_thr(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case CLASSH_TIMER:
		ret = tas256x_update_classh_timer(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case AMPOUTPUT_LVL:
		ret = tas256x_update_ampoutput_level(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case ICN_THR:
		ret = tas256x_update_icn_threshold(p_tas256x,
			ucontrol->value.integer.value[0], mc->shift);
	break;
	case ICN_HYST:
		ret = tas256x_update_icn_hysterisis(p_tas256x,
			ucontrol->value.integer.value[0],
			p_tas256x->mn_sampling_rate, mc->shift);
	break;
	}
	return ret;
}

static int tas256x_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct tas256x_priv *p_tas256x = NULL;
	int ret = -1;

	if ((codec == NULL) || (mc == NULL)) {
		pr_err("%s:codec or control is NULL\n", __func__);
		return ret;
	}

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	if (p_tas256x == NULL) {
		pr_err("%s:p_tas256x is NULL\n", __func__);
		return ret;
	}

	switch (mc->reg) {
	case DVC_PCM:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->dvc_pcm;
	break;
	case LIM_MAX_ATN:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->lim_max_attn;
	break;
	case LIMB_INF_PT:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->lim_infl_pt;
	break;
	case LIMB_SLOPE:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->lim_trk_slp;
	break;
	case LIMB_ATK_RT:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->lim_att_rate;
	break;
	case LIMB_RLS_RT:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->lim_rel_rate;
	break;
	case LIMB_RLS_ST:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->lim_rel_stp_size;
	break;
	case LIMB_ATK_ST:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->lim_att_stp_size;
	break;
	case BOP_ATK_RT:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->bop_att_rate;
	break;
	case BOP_ATK_ST:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->bop_att_stp_size;
	break;
	case BOP_HLD_TM:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->bop_hld_time;
	break;
	case BST_VREG:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->bst_vltg;
	break;
	case BST_ILIM:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->bst_ilm;
	break;
	case LIMB_TH_MAX:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->lim_thr_max;
	break;
	case LIMB_TH_MIN:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->lim_thr_min;
	break;
	case BOP_TH:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->bop_thd;
	break;
	case BOSD_TH:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->bosd_thd;
	break;
	case CLASSH_TIMER:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->classh_timer;
	break;
	case AMPOUTPUT_LVL:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->ampoutput_lvl;
	break;
	case ICN_THR:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->icn_thr;
	break;
	case ICN_HYST:
		ucontrol->value.integer.value[0] =
			p_tas256x->devs[mc->shift-1]->icn_hyst;
	break;
	}
	return 0;
}

static int tas256x_enum_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	int ret = -1;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct tas256x_priv *p_tas256x = NULL;
	struct linux_platform *plat_data = NULL;

	if (codec == NULL) {
		pr_err("%s:codec is NULL\n", __func__);
		return ret;
	}

	if (ucontrol == NULL) {
		pr_err("%s:ucontrol is NULL\n", __func__);
		return ret;
	}

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	if (p_tas256x == NULL) {
		pr_err("%s:p_tas256x is NULL\n", __func__);
		return ret;
	}
	plat_data = (struct linux_platform *) p_tas256x->platform_data;

	if (strnstr(ucontrol->id.name, "Version", MAX_STRING)) {
		ucontrol->value.integer.value[0] = 0;
	} else if (strnstr(ucontrol->id.name, "LEFT", MAX_STRING)) {
		if (strnstr(ucontrol->id.name, "LIMITER SWITCH",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[0]->lim_switch;
		else if (strnstr(ucontrol->id.name, "BOP ENABLE",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[0]->bop_enable;
		else if (strnstr(ucontrol->id.name, "BOP MUTE",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[0]->bop_mute;
		else if (strnstr(ucontrol->id.name, "BROWNOUT SHUTDOWN",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[0]->bosd_enable;
		else if (strnstr(ucontrol->id.name, "VBAT LPF",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[0]->vbat_lpf;
		else if (strnstr(ucontrol->id.name, "RECIEVER ENABLE",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[0]->reciever_enable;
		else if (strnstr(ucontrol->id.name, "NOISE GATE",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[0]->noise_gate;
		else
			dev_err(plat_data->dev, "Invalid controll %s\n",
				ucontrol->id.name);
	} else if (strnstr(ucontrol->id.name, "RIGHT", MAX_STRING)) {
		if (strnstr(ucontrol->id.name, "LIMITER SWITCH",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[1]->lim_switch;
		else if (strnstr(ucontrol->id.name, "BOP ENABLE",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[1]->bop_enable;
		else if (strnstr(ucontrol->id.name, "BOP MUTE",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[1]->bop_mute;
		else if (strnstr(ucontrol->id.name, "BROWNOUT SHUTDOWN",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[1]->bosd_enable;
		else if (strnstr(ucontrol->id.name, "VBAT LPF",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[1]->vbat_lpf;
		else if (strnstr(ucontrol->id.name, "RECIEVER ENABLE",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[1]->reciever_enable;
		else if (strnstr(ucontrol->id.name, "NOISE GATE",
			MAX_STRING))
			ucontrol->value.integer.value[0] =
				p_tas256x->devs[1]->noise_gate;
		else
			dev_err(plat_data->dev, "Invalid controll %s\n",
				ucontrol->id.name);
	} else {
		dev_err(plat_data->dev, "Invalid Channel %s\n",
			ucontrol->id.name);
	}
	return 0;
}

static int tas256x_enum_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct tas256x_priv *p_tas256x = NULL;
	struct linux_platform *plat_data = NULL;
	int ret = -1;

	if ((codec == NULL) || (mc == NULL)) {
		pr_err("%s:codec or control is NULL\n", __func__);
		return ret;
	}

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	if (p_tas256x == NULL) {
		pr_err("%s:p_tas256x is NULL\n", __func__);
		return ret;
	}
	plat_data = (struct linux_platform *) p_tas256x->platform_data;

	if (strnstr(ucontrol->id.name, "LEFT", MAX_STRING)) {
		if (strnstr(ucontrol->id.name, "LIMITER SWITCH",
			MAX_STRING))
			ret = tas256x_update_limiter_enable(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_left);
		else if (strnstr(ucontrol->id.name, "BOP ENABLE",
			MAX_STRING))
			ret = tas256x_update_bop_enable(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_left);
		else if (strnstr(ucontrol->id.name, "BOP MUTE",
			MAX_STRING))
			ret = tas256x_update_bop_mute(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_left);
		else if (strnstr(ucontrol->id.name, "BROWNOUT SHUTDOWN",
			MAX_STRING))
			ret = tas256x_update_bop_shutdown_enable(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_left);
		else if (strnstr(ucontrol->id.name, "VBAT LPF",
			MAX_STRING))
			ret = tas256x_update_vbat_lpf(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_left);
		else if (strnstr(ucontrol->id.name, "RECIEVER ENABLE",
			MAX_STRING))
			ret = tas256x_enable_reciever_mode(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_left);
		else if (strnstr(ucontrol->id.name, "NOISE GATE",
			MAX_STRING))
			ret = tas256x_enable_noise_gate(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_left);
		else
			dev_err(plat_data->dev, "Invalid Control %s\n",
				ucontrol->id.name);
	} else if (strnstr(ucontrol->id.name, "RIGHT", MAX_STRING)) {
		if (strnstr(ucontrol->id.name, "LIMITER SWITCH",
			MAX_STRING))
			ret = tas256x_update_limiter_enable(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_right);
		else if (strnstr(ucontrol->id.name, "BOP ENABLE",
				MAX_STRING))
			ret = tas256x_update_bop_enable(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_right);
		else if (strnstr(ucontrol->id.name, "BOP MUTE",
				MAX_STRING))
			ret = tas256x_update_bop_mute(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_right);
		else if (strnstr(ucontrol->id.name, "BROWNOUT SHUTDOWN",
				MAX_STRING))
			ret = tas256x_update_bop_shutdown_enable(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_right);
		else if (strnstr(ucontrol->id.name, "VBAT LPF",
				MAX_STRING))
			ret = tas256x_update_vbat_lpf(p_tas256x,
					ucontrol->value.integer.value[0],
					channel_right);
		else if (strnstr(ucontrol->id.name, "RECIEVER ENABLE",
			MAX_STRING))
			ret = tas256x_enable_reciever_mode(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_right);
		else if (strnstr(ucontrol->id.name, "NOISE GATE",
			MAX_STRING))
			ret = tas256x_enable_noise_gate(p_tas256x,
				ucontrol->value.integer.value[0],
				channel_right);
		else
			dev_err(plat_data->dev, "Invalid control %s\n",
				ucontrol->id.name);
	} else {
		dev_err(plat_data->dev, "Invalid Channel %s\n",
			ucontrol->id.name);
	}
	return ret;
}

static char const *tas256x_rx_switch_text[] = {"DISABLE", "ENABLE"};
static const struct soc_enum tas256x_rx_switch_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas256x_rx_switch_text),
		tas256x_rx_switch_text),
};

static char const *tas256x_version_text[] = {TAS256X_DRIVER_TAG};
static const struct soc_enum tas256x_version_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas256x_version_text),
		tas256x_version_text),
};

static const struct snd_kcontrol_new tas256x_left_controls[] = {
	SOC_ENUM_EXT("TAS256X Version", tas256x_version_enum[0],
		tas256x_enum_get, NULL),
	SOC_SINGLE_EXT("TAS256X PLAYBACK VOLUME LEFT", DVC_PCM, 1, 56, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM MAX ATTN LEFT", LIM_MAX_ATN, 1, 15, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM THR MAX LEFT", LIMB_TH_MAX,
		1, 26, 0, tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM THR MIN LEFT", LIMB_TH_MIN,
		1, 26, 0, tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM INFLECTION POINT LEFT", LIMB_INF_PT,
		1, 40, 0, tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM SLOPE LEFT", LIMB_SLOPE, 1, 6, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOP THR LEFT", BOP_TH,
		1, 15, 0, tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOSD THR LEFT", BOSD_TH,
		1, 15, 0, tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM ATTACT RATE LEFT", LIMB_ATK_RT, 1, 7, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM RELEASE RATE LEFT", LIMB_RLS_RT, 1, 7, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM ATTACK STEP LEFT", LIMB_ATK_ST, 1, 3, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM RELEASE STEP LEFT", LIMB_RLS_ST, 1, 3, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOP ATTACK RATE LEFT", BOP_ATK_RT, 1, 7, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOP ATTACK STEP LEFT", BOP_ATK_ST, 1, 3, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOP HOLD TIME LEFT", BOP_HLD_TM, 1, 7, 0,
		tas256x_get, tas256x_put),
	SOC_ENUM_EXT("TAS256X LIMITER SWITCH LEFT", tas256x_rx_switch_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_ENUM_EXT("TAS256X BOP ENABLE LEFT", tas256x_rx_switch_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_ENUM_EXT("TAS256X BOP MUTE LEFT", tas256x_rx_switch_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_ENUM_EXT("TAS256X BROWNOUT SHUTDOWN LEFT",
		tas256x_rx_switch_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_SINGLE_EXT("TAS256X CLASSH TIMER LEFT", CLASSH_TIMER, 1, 22, 0,
		tas256x_get, tas256x_put),
	SOC_ENUM_EXT("TAS256X RECIEVER ENABLE LEFT", tas256x_rx_switch_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_SINGLE_EXT("TAS256X ICN THRESHOLD LEFT", ICN_THR, 1, 84, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X ICN HYSTERISIS LEFT", ICN_HYST, 1, 19, 0,
		tas256x_get, tas256x_put),
	SOC_ENUM_EXT("TAS256X NOISE GATE LEFT", tas256x_rx_switch_enum[0],
		tas256x_enum_get, tas256x_enum_put),
};

static const struct snd_kcontrol_new tas256x_right_controls[] = {
	SOC_SINGLE_EXT("TAS256X PLAYBACK VOLUME RIGHT", DVC_PCM, 2, 56, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM MAX ATTN RIGHT", LIM_MAX_ATN, 2, 15, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM THR MAX RIGHT", LIMB_TH_MAX,
		2, 26, 0, tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM THR MIN RIGHT", LIMB_TH_MIN,
		2, 26, 0, tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM INFLECTION POINT RIGHT", LIMB_INF_PT,
		2, 40, 0, tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM SLOPE RIGHT", LIMB_SLOPE, 2, 6, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOP THR RIGHT", BOP_TH,
		2, 15, 0, tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOSD THR RIGHT", BOSD_TH,
		2, 15, 0, tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM ATTACT RATE RIGHT", LIMB_ATK_RT, 2, 7, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM RELEASE RATE RIGHT", LIMB_RLS_RT, 2, 7, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM ATTACK STEP RIGHT", LIMB_ATK_ST, 2, 3, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X LIM RELEASE STEP RIGHT", LIMB_RLS_ST, 2, 3, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOP ATTACK RATE RIGHT", BOP_ATK_RT, 2, 7, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOP ATTACK STEP RIGHT", BOP_ATK_ST, 2, 3, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOP HOLD TIME RIGHT", BOP_HLD_TM, 2, 7, 0,
		tas256x_get, tas256x_put),
	SOC_ENUM_EXT("TAS256X LIMITER SWITCH RIGHT", tas256x_rx_switch_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_ENUM_EXT("TAS256X BOP ENABLE RIGHT", tas256x_rx_switch_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_ENUM_EXT("TAS256X BOP MUTE RIGHT", tas256x_rx_switch_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_ENUM_EXT("TAS256X BROWNOUT SHUTDOWN RIGHT",
		tas256x_rx_switch_enum[0], tas256x_enum_get, tas256x_enum_put),
	SOC_SINGLE_EXT("TAS256X CLASSH TIMER RIGHT", CLASSH_TIMER, 2, 22, 0,
		tas256x_get, tas256x_put),
	SOC_ENUM_EXT("TAS256X RECIEVER ENABLE RIGHT", tas256x_rx_switch_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_SINGLE_EXT("TAS256X ICN THRESHOLD RIGHT", ICN_THR, 2, 84, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X ICN HYSTERISIS RIGHT", ICN_HYST, 2, 19, 0,
		tas256x_get, tas256x_put),
	SOC_ENUM_EXT("TAS256X NOISE GATE RIGHT", tas256x_rx_switch_enum[0],
		tas256x_enum_get, tas256x_enum_put),
};

static char const *tas2564_vbat_lpf_text[] = {
	"DISABLE", "HZ_10", "HZ_100", "KHZ_1"};
static const struct soc_enum tas2564_vbat_lpf_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas2564_vbat_lpf_text),
		tas2564_vbat_lpf_text),
};

static char const *tas2562_vbat_lpf_text[] = {
	"DISABLE", "HZ_100", "KHZ_1", "KHZ_10"};
static const struct soc_enum tas2562_vbat_lpf_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tas2562_vbat_lpf_text),
		tas2562_vbat_lpf_text),
};

static const struct snd_kcontrol_new tas2564_left_controls[] = {
	SOC_ENUM_EXT("TAS256X RX MODE LEFT", tas2564_rx_mode_enum[0],
		tas2564_get, tas2564_put),
	SOC_ENUM_EXT("TAS256X VBAT LPF LEFT", tas2564_vbat_lpf_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_SINGLE_EXT("TAS256X BOOST VOLTAGE LEFT", BST_VREG, 1, 15, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOOST CURRENT LEFT", BST_ILIM, 1, 63, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X AMP OUTPUT LVL LEFT", AMPOUTPUT_LVL, 1, 0x1C, 0,
		tas256x_get, tas256x_put),
};

static const struct snd_kcontrol_new tas2564_right_controls[] = {
	SOC_ENUM_EXT("TAS256X RX MODE RIGHT", tas2564_rx_mode_enum[0],
		tas2564_get, tas2564_put),
	SOC_ENUM_EXT("TAS256X VBAT LPF RIGHT", tas2564_vbat_lpf_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_SINGLE_EXT("TAS256X BOOST VOLTAGE RIGHT", BST_VREG, 2, 15, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOOST CURRENT RIGHT", BST_ILIM, 2, 63, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X AMP OUTPUT LVL RIGHT", AMPOUTPUT_LVL, 2, 0x1C, 0,
		tas256x_get, tas256x_put),
};

static const struct snd_kcontrol_new tas2562_left_controls[] = {
	SOC_ENUM_EXT("TAS256X VBAT LPF LEFT", tas2562_vbat_lpf_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_SINGLE_EXT("TAS256X BOOST VOLTAGE LEFT", BST_VREG, 1, 12, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOOST CURRENT LEFT", BST_ILIM, 1, 55, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X AMP OUTPUT LVL LEFT", AMPOUTPUT_LVL, 1, 0x1C, 0,
		tas256x_get, tas256x_put),
};

static const struct snd_kcontrol_new tas2562_right_controls[] = {
	SOC_ENUM_EXT("TAS256X VBAT LPF RIGHT", tas2562_vbat_lpf_enum[0],
		tas256x_enum_get, tas256x_enum_put),
	SOC_SINGLE_EXT("TAS256X BOOST VOLTAGE RIGHT", BST_VREG, 2, 12, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X BOOST CURRENT RIGHT", BST_ILIM, 2, 55, 0,
		tas256x_get, tas256x_put),
	SOC_SINGLE_EXT("TAS256X AMP OUTPUT LVL RIGHT", AMPOUTPUT_LVL, 2, 0x1C, 0,
		tas256x_get, tas256x_put),
};

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static int tas2564_probe(struct tas256x_priv *p_tas256x,
	struct snd_soc_component *codec, int chn)
{
	int ret = -1;
	struct linux_platform *plat_data = NULL;

	if ((!p_tas256x) || (!codec)) {
		pr_err("tas256x:%s p_tas256x or codec is Null\n", __func__);
		return ret;
	}

	plat_data = (struct linux_platform *) p_tas256x->platform_data;
	dev_dbg(plat_data->dev, "%s channel %d", __func__, chn);

	tas256x_update_default_params(p_tas256x, chn);
	if (chn == channel_left) {
		ret = snd_soc_add_component_controls(codec, tas256x_left_controls,
			ARRAY_SIZE(tas256x_left_controls));
		ret = snd_soc_add_component_controls(codec, tas2564_left_controls,
			ARRAY_SIZE(tas2564_left_controls));
	} else if (chn == channel_right) {
		ret = snd_soc_add_component_controls(codec, tas256x_right_controls,
			ARRAY_SIZE(tas256x_right_controls));
		ret = snd_soc_add_component_controls(codec, tas2564_right_controls,
			ARRAY_SIZE(tas2564_right_controls));
	} else {
		dev_err(plat_data->dev, "Invalid Channel %d\n", chn);
	}

	return ret;
}
#else
static int tas2564_probe(struct tas256x_priv *p_tas256x,
	struct snd_soc_codec *codec, int chn)
{
	int ret = -1;
	struct linux_platform *plat_data = NULL;

	if ((!p_tas256x) || (!codec)) {
		pr_err("tas256x:%s p_tas256x or codec is Null\n", __func__);
		return ret;
	}

	plat_data = (struct linux_platform *) p_tas256x->platform_data;
	dev_dbg(plat_data->dev, "%s channel %d", __func__, chn);

	tas256x_update_default_params(p_tas256x, chn);
	if (chn == channel_left) {
		ret = snd_soc_add_codec_controls(codec, tas256x_left_controls,
			ARRAY_SIZE(tas256x_left_controls));
		ret = snd_soc_add_codec_controls(codec, tas2564_left_controls,
			ARRAY_SIZE(tas2564_left_controls));
	} else if (chn == channel_right) {
		ret = snd_soc_add_codec_controls(codec, tas256x_right_controls,
			ARRAY_SIZE(tas256x_right_controls));
		ret = snd_soc_add_codec_controls(codec, tas2564_right_controls,
			ARRAY_SIZE(tas2564_right_controls));
	} else {
		dev_err(plat_data->dev, "Invalid Channel %d\n", chn);
	}

	return ret;
}
#endif

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static int tas2562_probe(struct tas256x_priv *p_tas256x,
	struct snd_soc_component *codec, int chn)
{
	struct linux_platform *plat_data = NULL;
	int ret = -1;

	if ((!p_tas256x) || (!codec)) {
		pr_err("tas256x:%s p_tas256x or codec is Null\n", __func__);
		return ret;
	}
	plat_data = (struct linux_platform *) p_tas256x->platform_data;
	dev_dbg(plat_data->dev, "%s channel %d", __func__, chn);

	tas256x_update_default_params(p_tas256x, chn);
	if (chn == channel_left) {
		ret = snd_soc_add_component_controls(codec, tas256x_left_controls,
			ARRAY_SIZE(tas256x_left_controls));
		ret = snd_soc_add_component_controls(codec, tas2562_left_controls,
			ARRAY_SIZE(tas2562_left_controls));
	} else if (chn == channel_right) {
		ret = snd_soc_add_component_controls(codec, tas256x_right_controls,
			ARRAY_SIZE(tas256x_right_controls));
		ret = snd_soc_add_component_controls(codec, tas2562_right_controls,
			ARRAY_SIZE(tas2562_right_controls));
	} else {
		dev_err(plat_data->dev, "Invalid Channel %d\n", chn);
	}

	return ret;
}
#else
static int tas2562_probe(struct tas256x_priv *p_tas256x,
	struct snd_soc_codec *codec, int chn)
{
	struct linux_platform *plat_data = NULL;
	int ret = -1;

	if ((!p_tas256x) || (!codec)) {
		pr_err("tas256x:%s p_tas256x or codec is Null\n", __func__);
		return ret;
	}
	plat_data = (struct linux_platform *) p_tas256x->platform_data;
	dev_dbg(plat_data->dev, "%s channel %d", __func__, chn);

	tas256x_update_default_params(p_tas256x, chn);
	if (chn == channel_left) {
		ret = snd_soc_add_codec_controls(codec, tas256x_left_controls,
			ARRAY_SIZE(tas256x_left_controls));
		ret = snd_soc_add_codec_controls(codec, tas2562_left_controls,
			ARRAY_SIZE(tas2562_left_controls));
	} else if (chn == channel_right) {
		ret = snd_soc_add_codec_controls(codec, tas256x_right_controls,
			ARRAY_SIZE(tas256x_right_controls));
		ret = snd_soc_add_codec_controls(codec, tas2562_right_controls,
			ARRAY_SIZE(tas2562_right_controls));
	} else {
		dev_err(plat_data->dev, "Invalid Channel %d\n", chn);
	}

	return ret;
}
#endif

static bool tas256x_volatile(struct device *dev, unsigned int reg)
{
	return true;
}

static bool tas256x_writeable(struct device *dev, unsigned int reg)
{
	return true;
}
static const struct regmap_config tas256x_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas256x_writeable,
	.volatile_reg = tas256x_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 1 * 128,
};

static void tas256x_hw_reset(struct tas256x_priv *p_tas256x)
{
	struct linux_platform *plat_data = NULL;
	int i = 0;

	plat_data = (struct linux_platform *) p_tas256x->platform_data;
	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (gpio_is_valid(p_tas256x->devs[i]->mn_reset_gpio)) {
			gpio_direction_output(
				p_tas256x->devs[i]->mn_reset_gpio, 0);
		}
	}
	msleep(20);

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (gpio_is_valid(p_tas256x->devs[i]->mn_reset_gpio)) {
			gpio_direction_output(
				p_tas256x->devs[i]->mn_reset_gpio, 1);
		}
		p_tas256x->devs[i]->mn_current_book = -1;
		p_tas256x->devs[i]->mn_current_page = -1;
	}
	msleep(20);

	dev_info(plat_data->dev, "reset gpio up !!\n");
}

static void tas256x_enable_irq(struct tas256x_priv *p_tas256x, bool enable)
{
	static int irq_enabled[2] = {0};
	struct irq_desc *desc = NULL;
	struct linux_platform *plat_data = NULL;
	int i = 0;

	plat_data = (struct linux_platform *) p_tas256x->platform_data;
	if (enable) {
		if (p_tas256x->mb_irq_eable)
			return;
		for (i = 0; i < p_tas256x->mn_channels; i++) {
			if (gpio_is_valid(p_tas256x->devs[i]->mn_irq_gpio) &&
				irq_enabled[i] == 0) {
				desc = irq_data_to_desc(irq_get_irq_data(
							p_tas256x->devs[i]->mn_irq));
				dev_dbg(plat_data->dev, "### enable-irq %d, irq desc=%p, depth=%d",
						p_tas256x->devs[i]->mn_irq, desc, desc ? desc->depth : 0);
				if (desc && desc->depth > 0)
					enable_irq(p_tas256x->devs[i]->mn_irq);
				else
					dev_info(plat_data->dev,
							"### irq already enabled");
				irq_enabled[i] = 1;
			}
		}
		p_tas256x->mb_irq_eable = true;
	} else {
		for (i = 0; i < p_tas256x->mn_channels; i++) {
			if (gpio_is_valid(p_tas256x->devs[i]->mn_irq_gpio)
					&& irq_enabled[i] == 1) {
				desc = irq_data_to_desc(irq_get_irq_data(
							p_tas256x->devs[i]->mn_irq));
				dev_dbg(plat_data->dev, "### disable-irq %d, irq desc=%p, depth=%d",
						p_tas256x->devs[i]->mn_irq, desc, desc ? desc->depth : 0);
				disable_irq_nosync(p_tas256x->devs[i]->mn_irq);
				irq_enabled[i] = 0;
			}
		}
		p_tas256x->mb_irq_eable = false;
	}
}

static void irq_work_routine(struct work_struct *work)
{
	struct tas256x_priv *p_tas256x =
		container_of(work, struct tas256x_priv, irq_work.work);
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas256x->platform_data;
	dev_info(plat_data->dev, "%s\n", __func__);
#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_lock(&p_tas256x->codec_lock);
#endif
	if (plat_data->mb_runtime_suspend) {
		pr_info("%s, Runtime Suspended\n", __func__);
		goto end;
	}
	/*Logical Layer IRQ function, return is ignored*/
	tas256x_irq_work_func(p_tas256x);

end:
#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_unlock(&p_tas256x->codec_lock);
#endif
	return;
}

static void init_work_routine(struct work_struct *work)
{
	struct tas256x_priv *p_tas256x =
		container_of(work, struct tas256x_priv, init_work.work);

#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_lock(&p_tas256x->codec_lock);
#endif
	/*Init Work Function. return is ignored*/
	tas256x_init_work_func(p_tas256x);

#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_unlock(&p_tas256x->codec_lock);
#endif
}

static irqreturn_t tas256x_irq_handler(int irq, void *dev_id)
{
	struct tas256x_priv *p_tas256x = (struct tas256x_priv *)dev_id;

	/* get IRQ status after 100 ms */
	schedule_delayed_work(&p_tas256x->irq_work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static int tas256x_runtime_suspend(struct tas256x_priv *p_tas256x)
{
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas256x->platform_data;
	dev_dbg(plat_data->dev, "%s\n", __func__);

	plat_data->mb_runtime_suspend = true;

	if (delayed_work_pending(&p_tas256x->irq_work)) {
		dev_dbg(plat_data->dev, "cancel IRQ work\n");
		cancel_delayed_work(&p_tas256x->irq_work);
	}

	return 0;
}

static int tas256x_runtime_resume(struct tas256x_priv *p_tas256x)
{
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas256x->platform_data;
	dev_dbg(plat_data->dev, "%s\n", __func__);

	plat_data->mb_runtime_suspend = false;

	return 0;
}

static int tas256x_pm_suspend(struct device *dev)
{
	struct tas256x_priv *p_tas256x = dev_get_drvdata(dev);

	if (!p_tas256x) {
		dev_err(dev, "drvdata is NULL\n");
		return -EINVAL;
	}

	mutex_lock(&p_tas256x->codec_lock);
	tas256x_runtime_suspend(p_tas256x);
	mutex_unlock(&p_tas256x->codec_lock);
	return 0;
}

static int tas256x_pm_resume(struct device *dev)
{
	struct tas256x_priv *p_tas256x = dev_get_drvdata(dev);

	if (!p_tas256x) {
		dev_err(dev, "drvdata is NULL\n");
		return -EINVAL;
	}

	mutex_lock(&p_tas256x->codec_lock);
	tas256x_runtime_resume(p_tas256x);
	mutex_unlock(&p_tas256x->codec_lock);
	return 0;
}

#if IS_ENABLED(CONFIG_TAS25XX_ALGO)
#if IS_ENABLED(CONFIG_PLATFORM_QCOM)
struct tas256x_priv *g_p_tas256x;

void tas256x_software_reset(void *prv_data)
{
	pr_err("[TI-SmartPA:%s]\n", __func__);
	schedule_delayed_work(&g_p_tas256x->dc_work, msecs_to_jiffies(10));
}

static void dc_work_routine(struct work_struct *work)
{
	struct tas256x_priv *p_tas256x =
		container_of(work, struct tas256x_priv, dc_work.work);
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas256x->platform_data;
#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_lock(&p_tas256x->codec_lock);
#endif
	tas256x_dc_work_func(p_tas256x, s_dc_detect.channel);

#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_unlock(&p_tas256x->codec_lock);
#endif

}
#endif/*CONFIG_PLATFORM_QCOM*/
#endif /*CONFIG_TAS25XX_ALGO*/

static void schedule_init_work(struct tas256x_priv *p_tas256x)
{
	schedule_delayed_work(&p_tas256x->init_work, msecs_to_jiffies(50));
}

static int tas256x_parse_dt(struct device *dev,
					struct tas256x_priv *p_tas256x)
{
	struct device_node *np = dev->of_node;
	int rc = 0, i = 0;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas256x->platform_data;
	rc = of_property_read_u32(np, "ti,channels", &p_tas256x->mn_channels);
	if (rc) {
		dev_err(plat_data->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,channels", np->full_name, rc);
		goto EXIT;
	} else {
		dev_dbg(plat_data->dev, "ti,channels=%d",
			p_tas256x->mn_channels);
	}

	/*the device structures array*/
	p_tas256x->devs =
		kmalloc(p_tas256x->mn_channels * sizeof(struct tas_device *),
			GFP_KERNEL);
	for (i = 0; i < p_tas256x->mn_channels; i++) {
		p_tas256x->devs[i] = kmalloc(sizeof(struct tas_device),
					GFP_KERNEL);
		if (p_tas256x->devs[i] == NULL) {
			dev_err(plat_data->dev,
			"%s:%u:kmalloc failed!\n", __func__, __LINE__);
			rc = -1;
			break;
		}

		rc = of_property_read_u32(np, dts_tag[i][0],
			&p_tas256x->devs[i]->mn_addr);
		if (rc) {
			dev_err(plat_data->dev,
				"Looking up %s property in node %s failed %d\n",
				dts_tag[i][0], np->full_name, rc);
			break;
		} else {
			dev_dbg(plat_data->dev, "%s = 0x%02x",
				dts_tag[i][0], p_tas256x->devs[i]->mn_addr);
		}

		p_tas256x->devs[i]->mn_reset_gpio =
			of_get_named_gpio(np, dts_tag[i][1], 0);
		if (!gpio_is_valid(p_tas256x->devs[i]->mn_reset_gpio))
			dev_err(plat_data->dev,
				"Looking up %s property in node %s failed %d\n",
				dts_tag[i][1], np->full_name,
				p_tas256x->devs[i]->mn_reset_gpio);
		else
			dev_dbg(plat_data->dev, "%s = %d",
				dts_tag[i][1],
				p_tas256x->devs[i]->mn_reset_gpio);

		p_tas256x->devs[i]->mn_irq_gpio =
			of_get_named_gpio(np, dts_tag[i][2], 0);
		if (!gpio_is_valid(p_tas256x->devs[i]->mn_irq_gpio)) {
			dev_err(plat_data->dev,
				"Looking up %s property in node %s failed %d\n",
				dts_tag[i][2], np->full_name,
				p_tas256x->devs[i]->mn_irq_gpio);
		} else {
			dev_dbg(plat_data->dev, "%s = %d",
				dts_tag[i][2],
				p_tas256x->devs[i]->mn_irq_gpio);
		}
		p_tas256x->devs[i]->spk_control = 1;
	}

	if (rc)
		goto EXIT;

	rc = of_property_read_u32(np, "ti,frame-start", &p_tas256x->mn_frame_start);
	if (rc) {
		dev_info(plat_data->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,frame-start", np->full_name, rc);
		p_tas256x->mn_frame_start = 0;
	} else {
		dev_dbg(plat_data->dev, "ti,frame-start=0x%x",
			p_tas256x->mn_frame_start);
	}

	rc = of_property_read_u32(np, "ti,rx-offset", &p_tas256x->mn_rx_offset);
	if (rc) {
		dev_info(plat_data->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,rx-offset", np->full_name, rc);
		p_tas256x->mn_rx_offset = 1;
	} else {
		dev_dbg(plat_data->dev, "ti,rx-offset=0x%x",
			p_tas256x->mn_rx_offset);
	}

	rc = of_property_read_u32(np, "ti,rx-edge", &p_tas256x->mn_rx_edge);
	if (rc) {
		dev_info(plat_data->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,rx-edge", np->full_name, rc);
		p_tas256x->mn_rx_edge = 1;
	} else {
		dev_dbg(plat_data->dev, "ti,rx-edge=0x%x",
			p_tas256x->mn_rx_edge);
	}

	rc = of_property_read_u32(np, "ti,tx-offset", &p_tas256x->mn_tx_offset);
	if (rc) {
		dev_info(plat_data->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,tx-offset", np->full_name, rc);
		p_tas256x->mn_tx_offset = 0;
	} else {
		dev_dbg(plat_data->dev, "ti,tx-offset=0x%x",
			p_tas256x->mn_tx_offset);
	}

	rc = of_property_read_u32(np, "ti,tx-edge", &p_tas256x->mn_tx_edge);
	if (rc) {
		dev_info(plat_data->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,tx-edge", np->full_name, rc);
		p_tas256x->mn_tx_edge = 0;
	} else {
		dev_dbg(plat_data->dev, "ti,tx-edge=0x%x",
			p_tas256x->mn_tx_edge);
	}

	rc = of_property_read_u32(np, "ti,iv-width", &p_tas256x->mn_iv_width);
	if (rc) {
		dev_err(plat_data->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,iv-width", np->full_name, rc);
	} else {
		dev_dbg(plat_data->dev, "ti,iv-width=0x%x",
			p_tas256x->mn_iv_width);
	}

	rc = of_property_read_u32(np, "ti,vbat-mon", &p_tas256x->mn_vbat);
	if (rc) {
		dev_err(plat_data->dev,
				"Looking up %s property in node %s failed %d\n",
			"ti,vbat-mon", np->full_name, rc);
	} else {
		dev_dbg(plat_data->dev, "ti,vbat-mon=0x%x",
			p_tas256x->mn_vbat);
	}
#if IS_ENABLED(CONFIG_TAS25XX_ALGO)
	/* Needs to be enabled always */
	tas25xx_parse_algo_dt(np);
#endif /*CONFIG_TAS25XX_ALGO*/
EXIT:
	return rc;
}

static int tas256x_i2c_probe(struct i2c_client *p_client,
			const struct i2c_device_id *id)
{
	struct tas256x_priv *p_tas256x;
	struct linux_platform *plat_data;
	int n_result = 0;
	int i = 0;

	dev_info(&p_client->dev, "Driver Tag: %s\n", TAS256X_DRIVER_TAG);
	dev_info(&p_client->dev, "%s enter\n", __func__);

	p_tas256x = devm_kzalloc(&p_client->dev,
		sizeof(struct tas256x_priv), GFP_KERNEL);
	if (p_tas256x == NULL) {
		dev_err(&p_client->dev, "failed to allocate memory\n");
		n_result = -ENOMEM;
		goto err;
	}

	plat_data = (struct linux_platform *)devm_kzalloc(&p_client->dev,
		sizeof(struct linux_platform), GFP_KERNEL);
	if (p_tas256x == NULL) {
		dev_err(&p_client->dev, "failed to allocate memory\n");
		n_result = -ENOMEM;
		goto err;
	}

	p_tas256x->platform_data = plat_data;
#if IS_ENABLED(CONFIG_TAS256X_REGBIN_PARSER)
	p_tas256x->profile_cfg_id = -1;
#endif
	p_tas256x->plat_write = tas256x_regmap_write;
	p_tas256x->plat_read = tas256x_regmap_read;
	p_tas256x->plat_bulk_write = tas256x_regmap_bulk_write;
	p_tas256x->plat_bulk_read = tas256x_regmap_bulk_read;
	p_tas256x->plat_update_bits = tas256x_regmap_update_bits;

	plat_data->client = p_client;
	plat_data->dev = &p_client->dev;
	i2c_set_clientdata(p_client, p_tas256x);
	dev_set_drvdata(&p_client->dev, p_tas256x);
	plat_data->regmap = devm_regmap_init_i2c(p_client,
				&tas256x_i2c_regmap);
	if (IS_ERR(plat_data->regmap)) {
		n_result = PTR_ERR(plat_data->regmap);
		dev_err(&p_client->dev,
			"Failed to allocate register map: %d\n",
			n_result);
		goto err;
	}

	mutex_init(&p_tas256x->dev_lock);
	p_tas256x->hw_reset = tas256x_hw_reset;
	p_tas256x->enable_irq = tas256x_enable_irq;
	p_tas256x->schedule_init_work = schedule_init_work;
#if IS_ENABLED(CODEC_PM)
	plat_data->runtime_suspend = tas256x_runtime_suspend;
	plat_data->runtime_resume = tas256x_runtime_resume;
	plat_data->mn_power_state = TAS256X_POWER_SHUTDOWN;
#endif

	if (p_client->dev.of_node)
		tas256x_parse_dt(&p_client->dev, p_tas256x);

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (gpio_is_valid(p_tas256x->devs[i]->mn_reset_gpio)) {
			n_result = gpio_request(
					p_tas256x->devs[i]->mn_reset_gpio,
					reset_gpio_label[i]);
			if (n_result) {
				dev_err(plat_data->dev,
					"%s: Failed to request gpio %d\n",
					__func__,
					p_tas256x->devs[i]->mn_reset_gpio);
				n_result = -EINVAL;
				goto err;
			}
		}
	}

	n_result = tas256x_register_device(p_tas256x);
	if (n_result < 0)
		goto err;

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		switch (p_tas256x->devs[i]->device_id) {
		case DEVICE_TAS2562:
			p_tas256x->devs[i]->dev_ops.tas_probe = tas2562_probe;
			break;
		case DEVICE_TAS2564:
			p_tas256x->devs[i]->dev_ops.tas_probe = tas2564_probe;
			break;
		default:
			p_tas256x->devs[i]->dev_ops.tas_probe = NULL;
			break;
		}
	}

	INIT_DELAYED_WORK(&p_tas256x->irq_work, irq_work_routine);

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (gpio_is_valid(p_tas256x->devs[i]->mn_irq_gpio)) {
			n_result =
				gpio_request(
					p_tas256x->devs[i]->mn_irq_gpio,
					irq_gpio_label[i]);
			if (n_result < 0) {
				dev_err(plat_data->dev,
					"%s:%u: ch 0x%02x: GPIO %d request error\n",
					__func__, __LINE__,
					p_tas256x->devs[i]->mn_addr,
					p_tas256x->devs[i]->mn_irq_gpio);
				goto err;
			}
			gpio_direction_input(p_tas256x->devs[i]->mn_irq_gpio);
			/*tas256x_dev_write(p_tas256x,
			 *	(i == 0)? channel_left : channel_right,
			 *	TAS256X_MISCCONFIGURATIONREG0, 0xce);
			 */

			p_tas256x->devs[i]->mn_irq =
				gpio_to_irq(p_tas256x->devs[i]->mn_irq_gpio);
			dev_info(plat_data->dev, "irq = %d\n",
				p_tas256x->devs[i]->mn_irq);

			n_result = request_threaded_irq(
					p_tas256x->devs[i]->mn_irq,
					tas256x_irq_handler,
					NULL,
					IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
					p_client->name, p_tas256x);
			if (n_result < 0) {
				dev_err(plat_data->dev,
					"request_irq failed, %d\n", n_result);
				goto err;
			}
			disable_irq_nosync(p_tas256x->devs[i]->mn_irq);
		}
	}

	tas256x_enable_irq(p_tas256x, true);
	INIT_DELAYED_WORK(&p_tas256x->init_work, init_work_routine);

#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_init(&p_tas256x->codec_lock);
	n_result = tas256x_register_codec(p_tas256x);
	if (n_result < 0) {
		dev_err(plat_data->dev,
			"register codec failed, %d\n", n_result);
		goto err;
	}
#endif

#if IS_ENABLED(CONFIG_TAS256X_MISC)
	mutex_init(&p_tas256x->file_lock);
	n_result = tas256x_register_misc(p_tas256x);
	if (n_result < 0) {
		dev_err(plat_data->dev, "register codec failed %d\n",
			n_result);
		goto err;
	}
#endif


#if IS_ENABLED(CONFIG_TAS25XX_ALGO)
#if IS_ENABLED(CONFIG_PLATFORM_QCOM)
	INIT_DELAYED_WORK(&p_tas256x->dc_work, dc_work_routine);
	g_p_tas256x = p_tas256x;
	register_tas256x_reset_func(tas256x_software_reset, &s_dc_detect);
#endif /*CONFIG_PLATFORM_QCOM*/
#endif /*CONFIG_TAS25XX_ALGO*/


err:
	return n_result;
}

static void tas256x_i2c_remove(struct i2c_client *p_client)
{
	int i = 0;
	struct tas256x_priv *p_tas256x = i2c_get_clientdata(p_client);
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas256x->platform_data;
	dev_info(plat_data->dev, "%s\n", __func__);

	/*Cancel all the work routine before exiting*/
	cancel_delayed_work_sync(&p_tas256x->irq_work);
	cancel_delayed_work_sync(&p_tas256x->init_work);
	cancel_delayed_work_sync(&p_tas256x->dc_work);

#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	tas256x_deregister_codec(p_tas256x);
	mutex_destroy(&p_tas256x->codec_lock);
#endif

#if IS_ENABLED(CONFIG_TAS256X_MISC)
	tas256x_deregister_misc(p_tas256x);
	mutex_destroy(&p_tas256x->file_lock);
#endif

	mutex_destroy(&p_tas256x->dev_lock);

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (gpio_is_valid(p_tas256x->devs[i]->mn_reset_gpio))
			gpio_free(p_tas256x->devs[i]->mn_reset_gpio);
		if (gpio_is_valid(p_tas256x->devs[i]->mn_irq_gpio))
			gpio_free(p_tas256x->devs[i]->mn_irq_gpio);
		kfree(p_tas256x->devs[i]);
	}

	kfree(p_tas256x->devs);
}


static const struct i2c_device_id tas256x_i2c_id[] = {
	{ "tas256x", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas256x_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas256x_of_match[] = {
	{ .compatible = "ti, tas256x" },
	{ .compatible = "ti, tas2562" },
	{ .compatible = "ti, tas2564" },
	{},
};
MODULE_DEVICE_TABLE(of, tas256x_of_match);
#endif

static const struct dev_pm_ops tas256x_pm_ops = {
	.suspend = tas256x_pm_suspend,
	.resume = tas256x_pm_resume
};

static struct i2c_driver tas256x_i2c_driver = {
	.driver = {
		.name   = "tas256x",
		.owner  = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(tas256x_of_match),
#endif
		.pm = &tas256x_pm_ops,
	},
	.probe      = tas256x_i2c_probe,
	.remove     = tas256x_i2c_remove,
	.id_table   = tas256x_i2c_id,
};

module_i2c_driver(tas256x_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS256X I2C Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
#endif

