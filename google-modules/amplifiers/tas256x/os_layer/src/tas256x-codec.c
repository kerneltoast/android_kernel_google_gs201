/*
 * =============================================================================
 * Copyright (c) 2016  Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.See the GNU General Public License for more details.
 *
 * File:
 *     tas256x-codec.c
 *
 * Description:
 *     ALSA SoC driver for Texas Instruments TAS256X High Performance 4W Smart
 *     Amplifier
 *
 * =============================================================================
 */

#if IS_ENABLED(CONFIG_TAS256X_CODEC)
#ifdef CONFIG_DYNAMIC_DEBUG
#define DEBUG 5
#endif
#include <linux/module.h>
#include <linux/moduleparam.h>
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/version.h>

#include "physical_layer/inc/tas256x.h"
#include "physical_layer/inc/tas256x-device.h"
#include "logical_layer/inc/tas256x-logic.h"
#include "os_layer/inc/tas256x-regmap.h"
#include "os_layer/inc/tas256x-codec.h"
#if IS_ENABLED(CONFIG_TAS25XX_ALGO)
#include "algo/inc/tas_smart_amp_v2.h"
#include "algo/inc/tas25xx-calib.h"
#endif /*CONFIG_TAS25XX_ALGO*/

#define TAS256X_MDELAY 0xFFFFFFFE
#define TAS256X_MSLEEP 0xFFFFFFFD
#define TAS256X_IVSENSER_ENABLE  1
#define TAS256X_IVSENSER_DISABLE 0
/* #define TAS2558_CODEC */

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static unsigned int tas256x_codec_read(struct snd_soc_component *codec,
		unsigned int reg)
{
	unsigned int value = 0;
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
	int ret = 0;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	switch (reg) {
	case TAS256X_LEFT_SWITCH:
		value = p_tas256x->devs[0]->spk_control;
		break;
	case TAS256X_RIGHT_SWITCH:
		value = p_tas256x->devs[1]->spk_control;
		break;
	case RX_SCFG_LEFT:
		value = p_tas256x->devs[0]->rx_cfg;
		break;
	case RX_SCFG_RIGHT:
		value = p_tas256x->devs[1]->rx_cfg;
		break;
	default:
		ret = p_tas256x->read(p_tas256x, channel_left, reg,
			&value);
		break;
	}

	dev_dbg(plat_data->dev, "%s, reg=%d, value=%d", __func__, reg, value);

	if (ret == 0)
		return value;
	else
		return ret;
}
#else
static unsigned int tas256x_codec_read(struct snd_soc_codec *codec,
		unsigned int reg)
{
	unsigned int value = 0;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	switch (reg) {
	case TAS256X_LEFT_SWITCH:
		value = p_tas256x->devs[0]->spk_control;
		break;
	case TAS256X_RIGHT_SWITCH:
		value = p_tas256x->devs[1]->spk_control;
		break;
	case RX_SCFG_LEFT:
		value = p_tas256x->devs[0]->rx_cfg;
		break;
	case RX_SCFG_RIGHT:
		value = p_tas256x->devs[1]->rx_cfg;
		break;
	default:
		ret = p_tas256x->read(p_tas256x, channel_left, reg,
			&value);
		break;
	}

	dev_dbg(plat_data->dev, "%s, reg=%d, value=%d", __func__, reg, value);

	if (ret == 0)
		return value;
	else
		return ret;
}
#endif

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static int tas256x_codec_write(struct snd_soc_component *codec,
				unsigned int reg, unsigned int value)
{
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	dev_dbg(plat_data->dev, "%s: %d, %d", __func__, reg, value);

	switch (reg) {
	case TAS256X_LEFT_SWITCH:
		p_tas256x->devs[0]->spk_control = value;
		break;
	case TAS256X_RIGHT_SWITCH:
		p_tas256x->devs[1]->spk_control = value;
		break;
	case RX_SCFG_LEFT:
		ret = tas256x_update_rx_cfg(p_tas256x, value,
			channel_left);
		break;
	case RX_SCFG_RIGHT:
		ret = tas256x_update_rx_cfg(p_tas256x, value,
			channel_right);
		break;
	default:
		ret = p_tas256x->write(p_tas256x, channel_both,
			reg, value);
		break;
	}

	return ret;
}
#else
static int tas256x_codec_write(struct snd_soc_codec *codec,
				unsigned int reg, unsigned int value)
{
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	dev_dbg(plat_data->dev, "%s: %d, %d", __func__, reg, value);

	switch (reg) {
	case TAS256X_LEFT_SWITCH:
		p_tas256x->devs[0]->spk_control = value;
		break;
	case TAS256X_RIGHT_SWITCH:
		p_tas256x->devs[1]->spk_control = value;
		break;
	case RX_SCFG_LEFT:
		ret = tas256x_update_rx_cfg(p_tas256x, value,
			channel_left);
		break;
	case RX_SCFG_RIGHT:
		ret = tas256x_update_rx_cfg(p_tas256x, value,
			channel_right);
		break;
	default:
		ret = p_tas256x->write(p_tas256x, channel_both,
			reg, value);
		break;
	}

	return ret;
}
#endif

#if IS_ENABLED(CODEC_PM)
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static int tas256x_codec_suspend(struct snd_soc_component *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	mutex_lock(&p_tas256x->codec_lock);

	dev_dbg(plat_data->dev, "%s\n", __func__);
	plat_data->runtime_suspend(p_tas256x);

	mutex_unlock(&p_tas256x->codec_lock);
	return ret;
}

static int tas256x_codec_resume(struct snd_soc_component *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	mutex_lock(&p_tas256x->codec_lock);

	dev_dbg(plat_data->dev, "%s\n", __func__);
	plat_data->runtime_resume(p_tas256x);

	mutex_unlock(&p_tas256x->codec_lock);
	return ret;
}
#else
static int tas256x_codec_suspend(struct snd_soc_codec *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	mutex_lock(&p_tas256x->codec_lock);

	dev_dbg(plat_data->dev, "%s\n", __func__);
	plat_data->runtime_suspend(p_tas256x);

	mutex_unlock(&p_tas256x->codec_lock);
	return ret;
}

static int tas256x_codec_resume(struct snd_soc_codec *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;

	mutex_lock(&p_tas256x->codec_lock);

	dev_dbg(plat_data->dev, "%s\n", __func__);
	plat_data->runtime_resume(p_tas256x);

	mutex_unlock(&p_tas256x->codec_lock);
	return ret;
}
#endif
#endif

static int tas256x_dac_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = snd_soc_dapm_to_component(w->dapm);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		dev_info(plat_data->dev, "SND_SOC_DAPM_POST_PMU\n");
		if (p_tas256x->mb_power_up == false)
			tas256x_set_power_state(p_tas256x,
				TAS256X_POWER_ACTIVE);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		dev_info(plat_data->dev, "SND_SOC_DAPM_PRE_PMD\n");
		if (p_tas256x->mb_power_up == true)
			tas256x_set_power_state(p_tas256x,
				TAS256X_POWER_SHUTDOWN);
		break;
	}

	return 0;
}

static const char * const tas256x_ASI1_src[] = {
	"I2C offset", "Left", "Right", "LeftRightDiv2",
};

static SOC_ENUM_SINGLE_DECL(tas2562_ASI1_src_left_enum, RX_SCFG_LEFT, 0,
			    tas256x_ASI1_src);
static SOC_ENUM_SINGLE_DECL(tas2562_ASI1_src_right_enum, RX_SCFG_RIGHT, 0,
			    tas256x_ASI1_src);

static const struct snd_kcontrol_new dapm_switch_left =
	SOC_DAPM_SINGLE("Switch", TAS256X_LEFT_SWITCH, 0, 1, 0);
static const struct snd_kcontrol_new dapm_switch_right =
	SOC_DAPM_SINGLE("Switch", TAS256X_RIGHT_SWITCH, 0, 1, 0);
static const struct snd_kcontrol_new tas256x_asi1_left_mux =
	SOC_DAPM_ENUM("Mux", tas2562_ASI1_src_left_enum);
static const struct snd_kcontrol_new tas256x_asi1_right_mux =
	SOC_DAPM_ENUM("Mux", tas2562_ASI1_src_right_enum);

static const struct snd_soc_dapm_widget tas256x_dapm_widgets_stereo[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SWITCH("TAS256X ASI Left", SND_SOC_NOPM, 0, 0,
		&dapm_switch_left),
	SND_SOC_DAPM_SWITCH("TAS256X ASI Right", SND_SOC_NOPM, 0, 0,
		&dapm_switch_right),
	SND_SOC_DAPM_MUX("TAS256X ASI1 SEL LEFT", SND_SOC_NOPM, 0, 0,
		&tas256x_asi1_left_mux),
	SND_SOC_DAPM_MUX("TAS256X ASI1 SEL RIGHT", SND_SOC_NOPM, 0, 0,
		&tas256x_asi1_right_mux),
	SND_SOC_DAPM_AIF_OUT("Voltage Sense", "ASI1 Capture",  1,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("Current Sense", "ASI1 Capture",  0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC_E("DAC1", NULL, SND_SOC_NOPM, 0, 0, tas256x_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_DAC_E("DAC2", NULL, SND_SOC_NOPM, 0, 0, tas256x_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT1"),
	SND_SOC_DAPM_OUTPUT("OUT2"),
	SND_SOC_DAPM_SIGGEN("VMON"),
	SND_SOC_DAPM_SIGGEN("IMON")
};

static const struct snd_soc_dapm_route tas256x_audio_map_stereo[] = {
	{"TAS256X ASI1 SEL LEFT", "Left", "ASI1"},
	{"TAS256X ASI1 SEL LEFT", "Right", "ASI1"},
	{"TAS256X ASI1 SEL LEFT", "LeftRightDiv2", "ASI1"},
	{"TAS256X ASI1 SEL LEFT", "I2C offset", "ASI1"},
	{"TAS256X ASI1 SEL RIGHT", "Left", "ASI1"},
	{"TAS256X ASI1 SEL RIGHT", "Right", "ASI1"},
	{"TAS256X ASI1 SEL RIGHT", "LeftRightDiv2", "ASI1"},
	{"TAS256X ASI1 SEL RIGHT", "I2C offset", "ASI1"},
	{"DAC1", NULL, "TAS256X ASI1 SEL LEFT"},
	{"DAC2", NULL, "TAS256X ASI1 SEL RIGHT"},
	{"TAS256X ASI Left", "Switch", "DAC1"},
	{"TAS256X ASI Right", "Switch", "DAC2"},
	{"OUT1", NULL, "TAS256X ASI Left"},
	{"OUT2", NULL, "TAS256X ASI Right"},
	{"Voltage Sense", NULL, "VMON"},
	{"Current Sense", NULL, "IMON"}
};

static const struct snd_soc_dapm_widget tas256x_dapm_widgets_mono[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SWITCH("TAS256X ASI", SND_SOC_NOPM, 0, 0,
		&dapm_switch_left),
	SND_SOC_DAPM_MUX("TAS256X ASI1 SEL", SND_SOC_NOPM, 0, 0,
		&tas256x_asi1_left_mux),
	SND_SOC_DAPM_AIF_OUT("Voltage Sense", "ASI1 Capture",  0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("Current Sense", "ASI1 Capture",  0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC_E("DAC", NULL, SND_SOC_NOPM, 0, 0, tas256x_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_SIGGEN("VMON"),
	SND_SOC_DAPM_SIGGEN("IMON")
};

static const struct snd_soc_dapm_route tas256x_audio_map_mono[] = {
	{"TAS256X ASI1 SEL", "Left", "ASI1"},
	{"TAS256X ASI1 SEL", "Right", "ASI1"},
	{"TAS256X ASI1 SEL", "LeftRightDiv2", "ASI1"},
	{"TAS256X ASI1 SEL", "I2C offset", "ASI1"},
	{"DAC", NULL, "TAS256X ASI1 SEL"},
	{"TAS256X ASI", "Switch", "DAC"},
	{"OUT", NULL, "TAS256X ASI"},
	{"Voltage Sense", NULL, "VMON"},
	{"Current Sense", NULL, "IMON"}
};

static int tas256x_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = dai->component;
	struct tas256x_priv *p_tas256x
			= snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data = (struct linux_platform *) p_tas256x->platform_data;
	int bitwidth = 16;
	int n_result = 0;
	unsigned int channels = params_channels(params);

	dev_dbg(plat_data->dev, "%s, stream %s format: %d\n", __func__,
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? ("Playback") : ("Capture"),
		params_format(params));

	mutex_lock(&p_tas256x->codec_lock);
#ifndef TDM_MACHINE
	/*Assumed TDM*/
	if (channels > 2) {
		p_tas256x->mn_fmt_mode = 2;

		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			bitwidth = 16;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			bitwidth = 24;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			bitwidth = 32;
			break;
		}

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			n_result = tas256x_set_tdm_rx_slot(p_tas256x, channels, bitwidth);
		else /*Assumed Capture*/
			n_result = tas256x_set_tdm_tx_slot(p_tas256x, channels, bitwidth);
	} else { /*Assumed I2S Mode*/
		p_tas256x->mn_fmt_mode = 1;
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			bitwidth = 16;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			bitwidth = 24;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			bitwidth = 32;
			break;
		}

		n_result = tas256x_set_bitwidth(p_tas256x,
				bitwidth, substream->stream);
		if (n_result < 0) {
			dev_info(plat_data->dev, "set bitwidth failed, %d\n",
				n_result);
			goto ret;
		}
	}
#else
	if (p_tas256x->mn_fmt_mode != 2) {
		p_tas256x->mn_fmt_mode = 1;
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			bitwidth = 16;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			bitwidth = 24;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			bitwidth = 32;
			break;
		}

		n_result = tas256x_set_bitwidth(p_tas256x,
				bitwidth, substream->stream);
		if (n_result < 0) {
			dev_info(plat_data->dev, "set bitwidth failed, %d\n",
				n_result);
			goto ret;
		}
	}
#endif

	dev_info(plat_data->dev, "%s, stream %s sample rate: %d\n", __func__,
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? ("Playback") : ("Capture"),
		params_rate(params));

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		n_result = tas256x_set_samplerate(p_tas256x,
			params_rate(params), channel_both);

ret:
	mutex_unlock(&p_tas256x->codec_lock);
	return n_result;
}

static int tas256x_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = dai->component;
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data = (struct linux_platform *) p_tas256x->platform_data;
	int ret = 0;
	u8 tdm_rx_start_slot = 0, asi_cfg_1 = 0, asi_cfg_2 = 0;

	dev_dbg(plat_data->dev, "%s, format=0x%x\n", __func__, fmt);

	p_tas256x->mn_fmt = 1;
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		asi_cfg_1 = 0x00;
		asi_cfg_2 = 0x00;
		break;
	default:
		dev_err(plat_data->dev, "ASI format master is not found\n");
		ret = -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		dev_info(plat_data->dev, "INV format: NBNF\n");
		asi_cfg_1 |= 0;
		asi_cfg_2 |= 0;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		dev_info(plat_data->dev, "INV format: IBNF\n");
		asi_cfg_1 |= 1;
		asi_cfg_2 |= 0;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		dev_info(plat_data->dev, "INV format: NBIF\n");
		asi_cfg_1 |= 0;
		asi_cfg_2 |= 1;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		dev_info(plat_data->dev, "INV format: IBIF\n");
		asi_cfg_1 |= 1;
		asi_cfg_2 |= 1;
		break;
	default:
		dev_err(plat_data->dev, "ASI format Inverse is not found\n");
		ret = -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case (SND_SOC_DAIFMT_I2S):
		tdm_rx_start_slot = 1;
		dev_info(plat_data->dev, " SND_SOC_DAIFMT_I2S tdm_rx_start_slot = 1\n");
		break;
	case (SND_SOC_DAIFMT_DSP_A):
		tdm_rx_start_slot = 1;
		dev_info(plat_data->dev, "SND_SOC_DAIFMT_DSP_A tdm_rx_start_slot =1\n");
		break;
	case (SND_SOC_DAIFMT_DSP_B):
		tdm_rx_start_slot = 0;
		dev_info(plat_data->dev, "SND_SOC_DAIFMT_DSP_B tdm_rx_start_slot = 0\n");
		break;
	case (SND_SOC_DAIFMT_LEFT_J):
		tdm_rx_start_slot = 0;
		dev_info(plat_data->dev, "SND_SOC_DAIFMT_LEFT_J tdm_rx_start_slot = 0\n");
		break;
	default:
	dev_err(plat_data->dev, "DAI Format is not found, fmt=0x%x\n", fmt);
	ret = -EINVAL;
		break;
	}

	ret = tas256x_rx_set_start_slot(p_tas256x,
		tdm_rx_start_slot, channel_both);
	if (ret)
		goto end;

	/*TX Offset is same as RX Offset*/
	ret = tas256x_tx_set_start_slot(p_tas256x,
		tdm_rx_start_slot, channel_both);
	if (ret)
		goto end;

	ret = tas256x_rx_set_edge(p_tas256x,
		asi_cfg_1, channel_both);
	if (ret)
		goto end;

	/*TX Edge is reverse of RX Edge*/
	ret = tas256x_tx_set_edge(p_tas256x,
		!asi_cfg_1, channel_both);
	if (ret)
		goto end;

	ret = tas256x_rx_set_frame_start(p_tas256x,
		asi_cfg_2, channel_both);
	if (ret)
		goto end;

end:
	return ret;
}

static int tas256x_set_dai_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	int ret = 0;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = dai->component;
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data = (struct linux_platform *) p_tas256x->platform_data;

	dev_dbg(plat_data->dev, "%s, tx_mask:%d, rx_mask:%d",
		__func__, tx_mask, rx_mask);
	dev_dbg(plat_data->dev, "%s, slots:%d,slot_width:%d",
		__func__, slots, slot_width);

	if (rx_mask) {
		p_tas256x->mn_fmt_mode = 2; /*TDM Mode*/
		ret = tas256x_set_tdm_rx_slot(p_tas256x, slots, slot_width);
	} else if (tx_mask) {
		p_tas256x->mn_fmt_mode = 2;
		ret = tas256x_set_tdm_tx_slot(p_tas256x, slots, slot_width);
	} else {
		dev_err(plat_data->dev, "%s, Invalid Mask",
				__func__);
		p_tas256x->mn_fmt_mode = 0;
	}

	return ret;
}

static int tas256x_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = dai->component;
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	dev_dbg(plat_data->dev, "%s, stream %s mute %d\n", __func__,
		(stream == SNDRV_PCM_STREAM_PLAYBACK) ? ("Playback") : ("Capture"),
		mute);

	return 0;
}

static struct snd_soc_dai_ops tas256x_dai_ops = {
	.hw_params  = tas256x_hw_params,
	.set_fmt    = tas256x_set_dai_fmt,
	.set_tdm_slot = tas256x_set_dai_tdm_slot,
	.mute_stream = tas256x_mute_stream,
};

#define TAS256X_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
						SNDRV_PCM_FMTBIT_S20_3LE |\
						SNDRV_PCM_FMTBIT_S24_LE |\
						SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver tas256x_dai_driver[] = {
	{
		.name = "tas256x ASI1",
		.id = 0,
		.playback = {
			.stream_name    = "ASI1 Playback",
			.channels_min   = 1,
			.channels_max   = 8,
			.rates      = SNDRV_PCM_RATE_8000_192000,
			.formats    = TAS256X_FORMATS,
		},
		.capture = {
			.stream_name    = "ASI1 Capture",
			.channels_min   = 1,
			.channels_max   = 8,
			.rates          = SNDRV_PCM_RATE_8000_192000,
			.formats    = TAS256X_FORMATS,
		},
		.ops = &tas256x_dai_ops,
		.symmetric_rate = 1,
	},
};

/*Generic Control-1: IV Sense enable*/
static char const *iv_enable_text[] = {"Off", "On"};
static const struct soc_enum tas256x_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(iv_enable_text), iv_enable_text),
};

static int tas256xiv_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec
				= snd_soc_kcontrol_component(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct tas256x_priv *p_tas256x = NULL;
	int iv_enable = 0, n_result = 0;

	if (codec == NULL) {
		pr_err("%s:codec is NULL\n", __func__);
		return 0;
	}

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	if (p_tas256x == NULL) {
		pr_err("%s:p_tas256x is NULL\n", __func__);
		return 0;
	}

	iv_enable = ucontrol->value.integer.value[0];

	n_result = tas256x_iv_sense_enable_set(p_tas256x, iv_enable,
		channel_both);

	pr_debug("%s: tas256x->iv_enable = %d\n", __func__,
		p_tas256x->iv_enable);

	return n_result;
}

static int tas256xiv_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct tas256x_priv *p_tas256x = NULL;
	struct linux_platform *plat_data = NULL;

	if (codec == NULL) {
		pr_err("%s:codec is NULL\n", __func__);
		return 0;
	}

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	if (p_tas256x == NULL) {
		pr_err("%s:p_tas256x is NULL\n", __func__);
		return 0;
	}

	plat_data = (struct linux_platform *) p_tas256x->platform_data;

	ucontrol->value.integer.value[0] =
		tas256x_iv_sense_enable_get(p_tas256x, channel_left);
	p_tas256x->iv_enable = ucontrol->value.integer.value[0];

	dev_info(plat_data->dev, "p_tas256x->iv_enable %d\n",
		p_tas256x->iv_enable);

	return 0;
}

static const struct snd_kcontrol_new tas256x_controls[] = {
SOC_ENUM_EXT("TAS256X IVSENSE ENABLE", tas256x_enum[0],
			tas256xiv_get, tas256xiv_put),
};

#if IS_ENABLED(CONFIG_TAS256X_REGBIN_PARSER)
/* max. length of a alsa mixer control name */
#define MAX_CONTROL_NAME        48

static char *fw_name = "tas256x_reg.bin";

const char *blocktype[5] = {
	"COEFF",
	"POST_POWER_UP",
	"PRE_SHUTDOWN",
	"PRE_POWER_UP",
	"POST_SHUTDOWN"
};

static int tas256x_process_block(void *pContext, unsigned char *data,
	unsigned char dev_idx, int sublocksize)
{
	struct tas256x_priv *pTAS256x = (struct tas256x_priv *)pContext;
	unsigned char subblk_typ = data[1];
	int subblk_offset = 2;
	enum channel chn = (dev_idx == 0) ? channel_both : (enum channel)dev_idx;

	switch (subblk_typ) {
	case TAS256X_CMD_SING_W: {
/*
		dev_idx		: one byte
		subblk_type	: one byte
		payload_len	: two bytes
		{
			book	: one byte
			page	: one byte
			reg		: one byte
			val		: one byte
		}[payload_len/4]
*/
		int i = 0;
		unsigned short len = SMS_HTONS(data[2], data[3]);

		subblk_offset += 2;
		if (subblk_offset + 4 * len > sublocksize) {
			pr_err("Out of memory %s: %u\n", __func__, __LINE__);
			break;
		}

		for (i = 0; i < len; i++) {
			pTAS256x->write(pTAS256x, chn,
				TAS256X_REG(data[subblk_offset], data[subblk_offset + 1], data[subblk_offset + 2]),
				data[subblk_offset + 3]);
			subblk_offset += 4;
		}
	}
	break;
	case TAS256X_CMD_BURST: {
/*
		dev_idx	: one byte
		subblk_type : one byte
		payload_len	: two bytes
		book		: one byte
		page		: one byte
		reg		: one byte
		reserve		: one byte
		payload		: payload_len bytes
*/
		unsigned short len = SMS_HTONS(data[2], data[3]);

		subblk_offset += 2;
		if (subblk_offset + 4 + len > sublocksize) {
			pr_err("Out of memory %s: %u\n", __func__, __LINE__);
			break;
		}
		if (len % 4) {
			pr_err("Burst len is wrong %s: %u\n", __func__, __LINE__);
			break;
		}

		pTAS256x->bulk_write(pTAS256x, chn,
			TAS256X_REG(data[subblk_offset], data[subblk_offset + 1],
			data[subblk_offset + 2]), &(data[subblk_offset + 4]), len);
		subblk_offset += (len + 4);
	}
	break;
	case TAS256X_CMD_DELAY: {
/*
		dev_idx	: one byte
		subblk_type : one byte
		delay_time	: two bytes
*/
		unsigned short delay_time = 0;

		if (subblk_offset + 2 > sublocksize) {
			pr_err("Out of memory %s: %u\n", __func__, __LINE__);
			break;
		}
		delay_time = SMS_HTONS(data[2], data[3]);
		usleep_range(delay_time*1000, delay_time*1000);
		subblk_offset += 2;
	}
	break;
	case TAS256X_CMD_FIELD_W:
/*
		dev_idx	: one byte
		subblk_type : one byte
		reserve		: one byte
		mask		: one byte
		book		: one byte
		page		: one byte
		reg		: one byte
		reserve	: one byte
		payload	: payload_len bytes
*/
	if (subblk_offset + 6 > sublocksize) {
		pr_err("Out of memory %s: %u\n", __func__, __LINE__);
		break;
	}
	pTAS256x->update_bits(pTAS256x, chn,
		TAS256X_REG(data[subblk_offset + 2], data[subblk_offset + 3], data[subblk_offset + 4]),
		data[subblk_offset + 1], data[subblk_offset + 5]);
	subblk_offset += 6;
	break;
	default:
	break;
	};

	return subblk_offset;

}

void tas256x_select_cfg_blk(void *pContext, int conf_no, unsigned char block_type)
{
	struct tas256x_priv *pTAS256x = (struct tas256x_priv *) pContext;
	struct tas256x_config_info **cfg_info = pTAS256x->cfg_info;
	int i = 0, j = 0, k = 0;

	if (conf_no > pTAS256x->ncfgs || conf_no < 0 || NULL == cfg_info) {
		pr_err("conf_no shoud be in range from 0 to %u\n", pTAS256x->ncfgs - 1);
		goto EXIT;
	} else {
		pr_info("%s:%u:profile_conf_id = %d\n", __func__, __LINE__, conf_no);
	}
	for (i = 0; i < pTAS256x->ncfgs; i++) {
		if (conf_no == i) {
			for (j = 0; j < (int)cfg_info[i]->real_nblocks; j++) {
				unsigned int length = 0, rc = 0;

				if (block_type > 5 || block_type < 2) {
					pr_err("ERROR!!!block_type shoud be in range from 2 to 5\n");
					goto EXIT;
				}
				if (block_type != cfg_info[i]->blk_data[j]->block_type)
					continue;
				pr_info("%s:%u:conf %d\n", __func__, __LINE__, i);
				pr_info("%s:%u:block type:%s\t device idx = 0x%02x\n",
					 __func__, __LINE__, blocktype[cfg_info[i]->blk_data[j]->block_type - 1],
					cfg_info[i]->blk_data[j]->dev_idx);
				for (k = 0; k < (int)cfg_info[i]->blk_data[j]->nSublocks; k++) {
					rc = tas256x_process_block(pTAS256x, cfg_info[i]->blk_data[j]->regdata + length,
						cfg_info[i]->blk_data[j]->dev_idx, cfg_info[i]->blk_data[j]->block_size - length);
					length += rc;
					if (cfg_info[i]->blk_data[j]->block_size < length) {
						pr_err("%s:%u:ERROR:%u %u out of memory\n", __func__, __LINE__,
							length, cfg_info[i]->blk_data[j]->block_size);
						break;
					}
				}
				if (length != cfg_info[i]->blk_data[j]->block_size) {
					pr_err("%s:%u:ERROR: %u %u size is not same\n", __func__,
						__LINE__, length, cfg_info[i]->blk_data[j]->block_size);
				}
			}
		} else {
			continue;
		}
	}
EXIT:
	return;
}

static struct tas256x_config_info *tas256x_add_config(unsigned char *config_data, unsigned int config_size)
{
	struct tas256x_config_info *cfg_info = NULL;
	int config_offset = 0, i = 0;

	cfg_info = (struct tas256x_config_info *)kzalloc(sizeof(struct tas256x_config_info), GFP_KERNEL);
	if (!cfg_info) {
		pr_err("%s:%u:Memory alloc failed!\n", __func__, __LINE__);
		goto EXIT;
	}
	if (config_offset + 4 > (int)config_size) {
		pr_err("%s:%u:Out of memory\n", __func__, __LINE__);
		goto EXIT;
	}
	cfg_info->nblocks = SMS_HTONL(config_data[config_offset], config_data[config_offset+1],
	config_data[config_offset+2], config_data[config_offset+3]);
	config_offset +=  4;
	pr_info("cfg_info->num_blocks = %u\n", cfg_info->nblocks);
	cfg_info->blk_data = (struct tas256x_block_data **)kzalloc(
		cfg_info->nblocks*sizeof(struct tas256x_block_data *),
		GFP_KERNEL);
	if (!cfg_info->blk_data) {
		pr_err("%s:%u:Memory alloc failed!\n", __func__, __LINE__);
		goto EXIT;
	}
	cfg_info->real_nblocks = 0;
	for (i = 0; i < (int)cfg_info->nblocks; i++) {
		if (config_offset + 12 > config_size) {
		pr_err("%s:%u:Out of memory: i = %d nblocks = %u!\n",
			__func__, __LINE__, i, cfg_info->nblocks);
		break;
		}
		cfg_info->blk_data[i] = (struct tas256x_block_data *)kzalloc(
			sizeof(struct tas256x_block_data), GFP_KERNEL);
		if (!cfg_info->blk_data[i]) {
			pr_err("%s:%u:Memory alloc failed!\n", __func__, __LINE__);
			break;
		}
		cfg_info->blk_data[i]->dev_idx = config_data[config_offset];
		config_offset++;
		pr_info("blk_data(%d).dev_idx = 0x%02x\n", i,
			cfg_info->blk_data[i]->dev_idx);
		cfg_info->blk_data[i]->block_type = config_data[config_offset];
		config_offset++;
		pr_info("blk_data(%d).block_type = 0x%02x\n", i,
			cfg_info->blk_data[i]->block_type);
		cfg_info->blk_data[i]->yram_checksum = SMS_HTONS(config_data[config_offset],
			config_data[config_offset+1]);
		config_offset += 2;
		cfg_info->blk_data[i]->block_size = SMS_HTONL(config_data[config_offset],
			config_data[config_offset + 1], config_data[config_offset + 2],
		config_data[config_offset + 3]);
		config_offset += 4;
		pr_info("blk_data(%d).block_size = %u\n", i,
		cfg_info->blk_data[i]->block_size);
		cfg_info->blk_data[i]->nSublocks = SMS_HTONL(config_data[config_offset],
			config_data[config_offset + 1], config_data[config_offset + 2],
		config_data[config_offset + 3]);
		pr_info("blk_data(%d).num_subblocks = %u\n", i,
		cfg_info->blk_data[i]->nSublocks);
		config_offset += 4;
		pr_info("config_offset = %d\n", config_offset);
		cfg_info->blk_data[i]->regdata = (unsigned char *)kzalloc(
			cfg_info->blk_data[i]->block_size, GFP_KERNEL);
		if (!cfg_info->blk_data[i]->regdata) {
			pr_err("%s:%u:Memory alloc failed!\n", __func__, __LINE__);
			goto EXIT;
		}
		if (config_offset + cfg_info->blk_data[i]->block_size > config_size) {
			pr_err("%s:%u:Out of memory: i = %d nblocks = %u!\n",
				__func__, __LINE__, i, cfg_info->nblocks);
			break;
		}
		memcpy(cfg_info->blk_data[i]->regdata, &config_data[config_offset],
		cfg_info->blk_data[i]->block_size);
		config_offset += cfg_info->blk_data[i]->block_size;
		cfg_info->real_nblocks += 1;
	}
EXIT:
	return cfg_info;
}

static int tas256x_info_profile(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_info *uinfo)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec
					= snd_soc_kcontrol_component(kcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_lock(&p_tas256x->codec_lock);
#endif
	uinfo->count = 1;
#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_unlock(&p_tas256x->codec_lock);
#endif
	uinfo->value.integer.min = -1;
	uinfo->value.integer.max = max(-1, p_tas256x->ncfgs - 1);
	pr_info("%s: max profile = %d\n", __func__, (int)uinfo->value.integer.max);

	return 0;
}

static int tas256x_get_profile_id(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec
		= snd_soc_kcontrol_component(kcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif

#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_lock(&p_tas256x->codec_lock);
#endif
	ucontrol->value.integer.value[0] = p_tas256x->profile_cfg_id;
#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_unlock(&p_tas256x->codec_lock);
#endif
	return 0;
}

static int tas256x_set_profile_id(struct snd_kcontrol *kcontrol,
		   struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec
	  = snd_soc_kcontrol_component(kcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif

#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_lock(&p_tas256x->codec_lock);
#endif
	p_tas256x->profile_cfg_id = ucontrol->value.integer.value[0];
#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_unlock(&p_tas256x->codec_lock);
#endif

	return 0;
}

static int tas256x_create_controls(struct tas256x_priv *pTAS256x)
{
	int  nr_controls = 1, ret = 0, mix_index = 0;
	char *name = NULL;
	struct linux_platform *platform_data =
		(struct linux_platform *)pTAS256x->platform_data;

	struct snd_kcontrol_new *tas256x_profile_controls = NULL;

	tas256x_profile_controls = devm_kzalloc(platform_data->dev,
			nr_controls * sizeof(tas256x_profile_controls[0]), GFP_KERNEL);
	if (tas256x_profile_controls == NULL) {
		ret = -ENOMEM;
		goto EXIT;
	}

	/* Create a mixer item for selecting the active profile */
	name = devm_kzalloc(platform_data->dev, MAX_CONTROL_NAME, GFP_KERNEL);
	if (!name) {
		ret = -ENOMEM;
		goto EXIT;
	}
	scnprintf(name, MAX_CONTROL_NAME, "TAS256x Profile id");
	tas256x_profile_controls[mix_index].name = name;
	tas256x_profile_controls[mix_index].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	tas256x_profile_controls[mix_index].info = tas256x_info_profile;
	tas256x_profile_controls[mix_index].get = tas256x_get_profile_id;
	tas256x_profile_controls[mix_index].put = tas256x_set_profile_id;
	mix_index++;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	ret = snd_soc_add_component_controls(platform_data->codec,
		tas256x_profile_controls, nr_controls < mix_index ? nr_controls : mix_index);
#else
	ret = snd_soc_add_codec_controls(platform_data->codec,
		tas256x_profile_controls, nr_controls < mix_index ? nr_controls : mix_index);
#endif
EXIT:
	return ret;
}

static void tas256x_fw_ready(const struct firmware *pFW, void *pContext)
{
	struct tas256x_priv *pTAS256x = (struct tas256x_priv *) pContext;
	struct tas256x_fw_hdr *fw_hdr = &(pTAS256x->fw_hdr);
	struct tas256x_config_info **cfg_info = NULL;
	unsigned char *buf = NULL;
	int offset = 0, i = 0;
	unsigned int total_config_sz = 0;

	pTAS256x->fw_state = TAS256X_DSP_FW_FAIL;

	if (unlikely(!pFW) || unlikely(!pFW->data)) {
		pr_err("Failed to read %s, no side-effect on driver running\n", fw_name);
		return;
	}
	buf = (unsigned char *)pFW->data;
#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_lock(&pTAS256x->codec_lock);
#endif
#if IS_ENABLED(CONFIG_TAS256X_MISC)
	mutex_lock(&pTAS256x->file_lock);
#endif
	pr_info("%s: start\n", __func__);
	fw_hdr->img_sz = SMS_HTONL(buf[offset], buf[offset + 1], buf[offset + 2], buf[offset + 3]);
	offset += 4;
	if (fw_hdr->img_sz != pFW->size) {
		pr_err("File size not match, %d %u", (int)pFW->size, fw_hdr->img_sz);
		goto EXIT;
	}

	fw_hdr->checksum = SMS_HTONL(buf[offset], buf[offset + 1], buf[offset + 2], buf[offset + 3]);
	offset += 4;
	fw_hdr->binnary_version_num = SMS_HTONL(buf[offset], buf[offset + 1], buf[offset + 2], buf[offset + 3]);
	offset += 4;
	fw_hdr->drv_fw_version = SMS_HTONL(buf[offset], buf[offset + 1], buf[offset + 2], buf[offset + 3]);
	offset += 4;
	fw_hdr->timestamp = SMS_HTONL(buf[offset], buf[offset + 1], buf[offset + 2], buf[offset + 3]);
	offset += 4;
	fw_hdr->plat_type = buf[offset];
	offset += 1;
	fw_hdr->dev_family = buf[offset];
	offset += 1;
	fw_hdr->reserve = buf[offset];
	offset += 1;
	fw_hdr->ndev = buf[offset];
	offset += 1;

	pr_info("ndev = %u\n", fw_hdr->ndev);

	if (offset + TAS256X_DEVICE_SUM > fw_hdr->img_sz) {
		pr_err("%s:%u:Out of Memory!\n", __func__, __LINE__);
		goto EXIT;
	}

	for (i = 0; i < TAS256X_DEVICE_SUM; i++) {
		fw_hdr->devs[i] = buf[offset];
		offset += 1;
		pr_info("devs[%d] = %u\n", i, fw_hdr->devs[i]);
	}
	fw_hdr->nconfig = SMS_HTONL(buf[offset], buf[offset + 1], buf[offset + 2], buf[offset + 3]);
	offset += 4;
	pr_info("nconfig = %u\n", fw_hdr->nconfig);
	for (i = 0; i < TAS256X_CONFIG_SIZE; i++) {
		fw_hdr->config_size[i] = SMS_HTONL(buf[offset], buf[offset + 1], buf[offset + 2], buf[offset + 3]);
		offset += 4;
		pr_info("config_size[%d] = %u\n", i, fw_hdr->config_size[i]);
		total_config_sz += fw_hdr->config_size[i];
	}
	pr_info("img_sz = %u total_config_sz = %u offset = %d\n",
	fw_hdr->img_sz, total_config_sz, offset);
	if (fw_hdr->img_sz - total_config_sz != (unsigned int)offset) {
		pr_err("Bin file error!\n");
		goto EXIT;
	}
	cfg_info = (struct tas256x_config_info **)kzalloc(
		fw_hdr->nconfig*sizeof(struct tas256x_config_info *),
		GFP_KERNEL);
	if (!cfg_info) {
		pr_err("%s:%u:Memory alloc failed!\n", __func__, __LINE__);
		goto EXIT;
	}
	pTAS256x->cfg_info = cfg_info;
	pTAS256x->ncfgs = 0;
	for (i = 0; i < (int)fw_hdr->nconfig; i++) {
		cfg_info[i] = tas256x_add_config(&buf[offset], fw_hdr->config_size[i]);
		if (!cfg_info[i]) {
			pr_err("%s:%u:Memory alloc failed!\n", __func__, __LINE__);
			break;
		}
		offset += (int)fw_hdr->config_size[i];
		pTAS256x->ncfgs += 1;
	}

	pTAS256x->fw_state = TAS256X_DSP_FW_OK;
	tas256x_create_controls(pTAS256x);
EXIT:
#if IS_ENABLED(CONFIG_TAS256X_MISC)
	mutex_unlock(&pTAS256x->file_lock);
#endif
#if IS_ENABLED(CONFIG_TAS256X_CODEC)
	mutex_unlock(&pTAS256x->codec_lock);
#endif
	release_firmware(pFW);
	pr_info("%s: Firmware init complete\n", __func__);
    return;
}

int tas256x_load_container(struct tas256x_priv *pTAS256x)
{
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) pTAS256x->platform_data;
	pTAS256x->fw_state = TAS256X_DSP_FW_PENDING;
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		fw_name, plat_data->dev, GFP_KERNEL, pTAS256x, tas256x_fw_ready);
}

void tas256x_config_info_remove(void *pContext)
{
	struct tas256x_priv *pTAS256x = (struct tas256x_priv *) pContext;
	struct tas256x_config_info **cfg_info = pTAS256x->cfg_info;
	int i = 0, j = 0;

	if (cfg_info) {
		for (i = 0; i < pTAS256x->ncfgs; i++) {
			if (cfg_info[i]) {
				for (j = 0; j < (int)cfg_info[i]->real_nblocks; j++) {
					kfree(cfg_info[i]->blk_data[j]->regdata);
					kfree(cfg_info[i]->blk_data[j]);
				}
				kfree(cfg_info[i]->blk_data);
				kfree(cfg_info[i]);
			}
		}
		kfree(cfg_info);
	}
}
#endif

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static int tas256x_codec_probe(struct snd_soc_component *codec)
{
	int ret, i;
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(codec);

	if (plat_data)
		plat_data->codec = codec;

	ret = snd_soc_add_component_controls(codec, tas256x_controls,
					 ARRAY_SIZE(tas256x_controls));
	if (ret < 0) {
		pr_err("%s: add_codec_controls failed, err %d\n",
			__func__, ret);
		return ret;
	}

	snd_soc_dapm_ignore_suspend(dapm, "ASI1 Playback");
	snd_soc_dapm_ignore_suspend(dapm, "ASI1 Capture");
	if (p_tas256x->mn_channels == 2) {
		snd_soc_dapm_ignore_suspend(dapm, "OUT1");
		snd_soc_dapm_ignore_suspend(dapm, "OUT2");
		snd_soc_dapm_ignore_suspend(dapm, "VMON");
		snd_soc_dapm_ignore_suspend(dapm, "IMON");
	} else {
		snd_soc_dapm_ignore_suspend(dapm, "OUT");
		snd_soc_dapm_ignore_suspend(dapm, "VMON");
		snd_soc_dapm_ignore_suspend(dapm, "IMON");
	}

	snd_soc_dapm_sync(dapm);
	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (p_tas256x->devs[i]->dev_ops.tas_probe)
			ret |= (p_tas256x->devs[i]->dev_ops.tas_probe)(p_tas256x, codec, i+1);
	}

	/* Generic Probe */
	ret = tas256x_probe(p_tas256x);
	dev_dbg(plat_data->dev, "%s\n", __func__);

	return 0;
}
#else
static int tas256x_codec_probe(struct snd_soc_codec *codec)
{
	int ret, i;
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	struct snd_soc_dapm_context *dapm =
			snd_soc_codec_get_dapm(codec);

	if (plat_data)
		plat_data->codec = codec;

	dev_info(plat_data->dev, "Driver Tag: %s\n", TAS256X_DRIVER_TAG);
	ret = snd_soc_add_codec_controls(codec, tas256x_controls,
					 ARRAY_SIZE(tas256x_controls));
	if (ret < 0) {
		pr_err("%s: add_codec_controls failed, err %d\n",
			__func__, ret);
		return ret;
	}
	snd_soc_dapm_ignore_suspend(dapm, "ASI1 Playback");
	snd_soc_dapm_ignore_suspend(dapm, "ASI1 Capture");
	if (p_tas256x->mn_channels == 2) {
		snd_soc_dapm_ignore_suspend(dapm, "OUT1");
		snd_soc_dapm_ignore_suspend(dapm, "OUT2");
		snd_soc_dapm_ignore_suspend(dapm, "VMON");
		snd_soc_dapm_ignore_suspend(dapm, "IMON");
	} else {
		snd_soc_dapm_ignore_suspend(dapm, "OUT");
		snd_soc_dapm_ignore_suspend(dapm, "VMON");
		snd_soc_dapm_ignore_suspend(dapm, "IMON");
	}

	snd_soc_dapm_sync(dapm);

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (p_tas256x->devs[i]->dev_ops.tas_probe)
			ret |= (p_tas256x->devs[i]->dev_ops.tas_probe)(p_tas256x, codec, i+1);
	}
	/* Generic Probe */
	ret = tas256x_probe(p_tas256x);
	dev_dbg(plat_data->dev, "%s\n", __func__);

	return ret;
}
#endif

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static void tas256x_codec_remove(struct snd_soc_component *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);

	tas256x_remove(p_tas256x);
}
#else
static int tas256x_codec_remove(struct snd_soc_codec *codec)
{
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);

	tas256x_remove(p_tas256x);
	return 0;
}
#endif

/*snd control-1: SmartPA System Mute(Master) Control*/
static int tas256x_system_mute_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec
					= snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	pValue->value.integer.value[0] = p_tas256x->mb_mute;
	dev_dbg(plat_data->dev, "%s = %d\n",
		__func__, p_tas256x->mb_mute);

	return 0;
}

static int tas256x_system_mute_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int mb_mute = pValue->value.integer.value[0];

	dev_dbg(plat_data->dev, "%s = %d\n", __func__, mb_mute);

	p_tas256x->mb_mute = !!mb_mute;

	return 0;
}

/*snd control-2: SmartPA Mute Control*/
static int tas256x_mute_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	pValue->value.integer.value[0] = p_tas256x->mb_mute;

	if ((p_tas256x->mb_power_up == true) &&
		(p_tas256x->mn_power_state == TAS256X_POWER_ACTIVE))
		pValue->value.integer.value[0] = 0;
	else
		pValue->value.integer.value[0] = 1;

	dev_dbg(plat_data->dev, "%s = %ld\n",
		__func__, pValue->value.integer.value[0]);

	return 0;
}

static int tas256x_mute_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int mute = pValue->value.integer.value[0];

	dev_dbg(plat_data->dev, "%s, %d\n", __func__, mute);
	mutex_lock(&p_tas256x->codec_lock);

	if (mute)
		tas256x_set_power_state(p_tas256x, TAS256X_POWER_MUTE);
	else
		tas256x_set_power_state(p_tas256x, TAS256X_POWER_ACTIVE);

	mutex_unlock(&p_tas256x->codec_lock);
	return 0;
}

/*snd control-3: DAC Mute Control*/
static int tas256x_dac_mute_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	 struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	pValue->value.integer.value[0] = p_tas256x->dac_mute;

	dev_dbg(plat_data->dev, "%s = %ld\n",
		__func__, pValue->value.integer.value[0]);

	return 0;
}

static int tas256x_dac_mute_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
	int n_result = 0;
	enum channel chn = channel_left;
	int mute = pValue->value.integer.value[0];
	int i = 0, chnTemp = 0;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data = (struct linux_platform *) p_tas256x->platform_data;

	dev_dbg(plat_data->dev, "%s, %d\n", __func__, mute);
	mutex_lock(&p_tas256x->codec_lock);

	for (i = 0; i < p_tas256x->mn_channels; i++) {
		if (p_tas256x->devs[i]->spk_control == 1)
			chnTemp |= 1<<i;
	}
	chn = (chnTemp == 0)?chn:(enum channel)chnTemp;

	if (mute) {
		n_result = tas256x_set_power_mute(p_tas256x, chn);
	} else {
		msleep(50);
#if IS_ENABLED(CONFIG_TAS256X_REGBIN_PARSER)
		/*set p_tas256x->profile_cfg_id by tinymix*/
		tas256x_select_cfg_blk(p_tas256x, p_tas256x->profile_cfg_id,
			TAS256X_BIN_BLK_PRE_POWER_UP);
#endif
		n_result = tas256x_set_power_up(p_tas256x, chn);
#if IS_ENABLED(CONFIG_TAS256X_REGBIN_PARSER)
		/*set p_tas256x->profile_cfg_id by tinymix*/
		tas256x_select_cfg_blk(p_tas256x, p_tas256x->profile_cfg_id,
			TAS256X_BIN_BLK_POST_POWER_UP);
#endif
	}

	p_tas256x->dac_mute = mute;
	mutex_unlock(&p_tas256x->codec_lock);

	return n_result;
}

/*ICN Disable*/
static const char * const icn_sw_text[] = {"Enable", "Disable"};
static const struct soc_enum icn_sw_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(icn_sw_text),
	icn_sw_text),
};
static int tas256x_get_icn_switch(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *p_u_control)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data = (struct linux_platform *) p_tas256x->platform_data;

	dev_info(plat_data->dev, "%s, icn_sw = %ld\n",
			__func__, p_u_control->value.integer.value[0]);
	p_u_control->value.integer.value[0] = p_tas256x->icn_sw;
	return 0;
}
static int tas256x_set_icn_switch(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *p_u_control)
{
	int ret  = 0;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif

	p_tas256x->icn_sw = p_u_control->value.integer.value[0];
	ret = tas256x_icn_disable(p_tas256x, p_tas256x->icn_sw, channel_both);

	return ret;
}

/*Rx Slot*/
static int tas256x_set_rx_slot_map_single(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
	int ret = 0;
	int value = 0;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	value = pValue->value.integer.value[0];

	ret = tas256x_rx_set_slot(p_tas256x, value, channel_left);
	dev_dbg(plat_data->dev, "%s = %ld\n",
		__func__, pValue->value.integer.value[0]);

	return ret;
}

static int tas256x_get_rx_slot_map_single(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data = (struct linux_platform *) p_tas256x->platform_data;

	pValue->value.integer.value[0] = p_tas256x->mn_rx_slot_map[0];
	dev_dbg(plat_data->dev, "%s = %ld\n",
		__func__, pValue->value.integer.value[0]);
	return 0;
}

static const struct snd_kcontrol_new tas256x_snd_controls_mono[] = {
	SOC_SINGLE_EXT("SmartPA System Mute", SND_SOC_NOPM, 0, 0x0001, 0,
		tas256x_system_mute_ctrl_get, tas256x_system_mute_ctrl_put),
	SOC_SINGLE_EXT("SmartPA Mute", SND_SOC_NOPM, 0, 0x0001, 0,
		tas256x_mute_ctrl_get, tas256x_mute_ctrl_put),
	SOC_SINGLE_EXT("TAS256X DAC Mute", SND_SOC_NOPM, 0, 0x0001, 0,
		tas256x_dac_mute_ctrl_get, tas256x_dac_mute_ctrl_put),
	SOC_ENUM_EXT("TAS256X ICN Switch", icn_sw_enum[0],
		tas256x_get_icn_switch,
		tas256x_set_icn_switch),
	SOC_SINGLE_EXT("TAS256X_RX_SLOT_MAP", SND_SOC_NOPM, 0, 7, 0,
		tas256x_get_rx_slot_map_single,
		tas256x_set_rx_slot_map_single),
};

struct soc_multi_control_ch_map {
	int min, max, platform_max, count;
	unsigned int reg, rreg, shift, rshift, invert;
};

static int tas256x_rx_slot_map_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_multi_control_ch_map *mc =
		(struct soc_multi_control_ch_map *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = mc->count;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mc->platform_max;

	return 0;
}

static int tas256x_get_rx_slot_map_multi(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(kcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int i;

	for (i = 0; i < 2; i++) {
		dev_info(plat_data->dev, "%s idx=%d, value=%d\n",
			__func__, i, (int)p_tas256x->mn_rx_slot_map[i]);
		ucontrol->value.integer.value[i] =
			(unsigned int) p_tas256x->mn_rx_slot_map[i];
	}

       return 0;
}

static int tas256x_set_rx_slot_map_multi(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	struct snd_soc_component *codec =
		snd_soc_kcontrol_component(kcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas256x_priv *p_tas256x = snd_soc_codec_get_drvdata(codec);
#endif
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
	int i, ret = 0;
	char slot_map[2];

	for (i = 0; i < 2; i++) {
		slot_map[i] = (char)(ucontrol->value.integer.value[i]);
		ret = tas256x_rx_set_slot(p_tas256x, slot_map[i], i+1);
		dev_info(plat_data->dev, "%s mapping - index %d = channel %d\n",
			__func__, i, slot_map[i]);
	}

	return ret;
}

static const struct snd_kcontrol_new tas256x_snd_controls_stereo[] = {
	SOC_SINGLE_EXT("SmartPA System Mute", SND_SOC_NOPM, 0, 0x0001, 0,
		tas256x_system_mute_ctrl_get, tas256x_system_mute_ctrl_put),
	SOC_SINGLE_EXT("SmartPA Mute", SND_SOC_NOPM, 0, 0x0001, 0,
		tas256x_mute_ctrl_get, tas256x_mute_ctrl_put),
	SOC_SINGLE_EXT("TAS256X DAC Mute", SND_SOC_NOPM, 0, 0x0001, 0,
		tas256x_dac_mute_ctrl_get, tas256x_dac_mute_ctrl_put),
	SOC_ENUM_EXT("TAS256X ICN Switch", icn_sw_enum[0],
		tas256x_get_icn_switch,
		tas256x_set_icn_switch),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "TAS256X_RX_SLOT_MAP",
		.info = tas256x_rx_slot_map_info,
		.get = tas256x_get_rx_slot_map_multi,
		.put = tas256x_set_rx_slot_map_multi,
		.private_value = (unsigned long) &(struct soc_multi_control_ch_map) {
				.reg = SND_SOC_NOPM,
				.shift = 0,
				.rshift = 0,
				.max = 7,
				.count = 2,
				.platform_max = 7,
				.invert = 0,
		}
	},
};

#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
static struct snd_soc_component_driver soc_codec_driver_tas256x = {
	.probe			= tas256x_codec_probe,
	.remove			= tas256x_codec_remove,
	.read			= tas256x_codec_read,
	.write			= tas256x_codec_write,
#if IS_ENABLED(CODEC_PM)
	.suspend		= tas256x_codec_suspend,
	.resume			= tas256x_codec_resume,
#endif
	.controls		= tas256x_snd_controls_mono,
	.num_controls		= ARRAY_SIZE(tas256x_snd_controls_mono),
	.dapm_widgets		= tas256x_dapm_widgets_mono,
	.num_dapm_widgets	= ARRAY_SIZE(tas256x_dapm_widgets_mono),
	.dapm_routes		= tas256x_audio_map_mono,
	.num_dapm_routes	= ARRAY_SIZE(tas256x_audio_map_mono),
};
#else
static struct snd_soc_codec_driver soc_codec_driver_tas256x = {
	.probe			= tas256x_codec_probe,
	.remove			= tas256x_codec_remove,
	.read			= tas256x_codec_read,
	.write			= tas256x_codec_write,
#if IS_ENABLED(CODEC_PM)
	.suspend		= tas256x_codec_suspend,
	.resume			= tas256x_codec_resume,
#endif
	.component_driver = {
		.controls		= tas256x_snd_controls_mono,
		.num_controls		= ARRAY_SIZE(tas256x_snd_controls_mono),
		.dapm_widgets		= tas256x_dapm_widgets_mono,
		.num_dapm_widgets	= ARRAY_SIZE(tas256x_dapm_widgets_mono),
		.dapm_routes		= tas256x_audio_map_mono,
		.num_dapm_routes	= ARRAY_SIZE(tas256x_audio_map_mono),
	},
};
#endif

int tas256x_register_codec(struct tas256x_priv *p_tas256x)
{
	int n_result = 0;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;

	dev_info(plat_data->dev, "%s, enter\n", __func__);

	if (p_tas256x->mn_channels == 2) {
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
		soc_codec_driver_tas256x.controls =
			tas256x_snd_controls_stereo;
		soc_codec_driver_tas256x.num_controls =
			ARRAY_SIZE(tas256x_snd_controls_stereo);
		soc_codec_driver_tas256x.dapm_widgets =
			tas256x_dapm_widgets_stereo;
		soc_codec_driver_tas256x.num_dapm_widgets =
			ARRAY_SIZE(tas256x_dapm_widgets_stereo);
		soc_codec_driver_tas256x.dapm_routes =
			tas256x_audio_map_stereo;
		soc_codec_driver_tas256x.num_dapm_routes =
			ARRAY_SIZE(tas256x_audio_map_stereo);
		n_result = devm_snd_soc_register_component(plat_data->dev,
			&soc_codec_driver_tas256x,
			tas256x_dai_driver, ARRAY_SIZE(tas256x_dai_driver));
#else
		soc_codec_driver_tas256x.component_driver.controls =
			tas256x_snd_controls_stereo;
		soc_codec_driver_tas256x.component_driver.num_controls =
			ARRAY_SIZE(tas256x_snd_controls_stereo);
		soc_codec_driver_tas256x.component_driver.dapm_widgets =
			tas256x_dapm_widgets_stereo;
		soc_codec_driver_tas256x.component_driver.num_dapm_widgets =
			ARRAY_SIZE(tas256x_dapm_widgets_stereo);
		soc_codec_driver_tas256x.component_driver.dapm_routes =
			tas256x_audio_map_stereo;
		soc_codec_driver_tas256x.component_driver.num_dapm_routes =
			ARRAY_SIZE(tas256x_audio_map_stereo);
		n_result = snd_soc_register_codec(plat_data->dev,
			&soc_codec_driver_tas256x,
			tas256x_dai_driver, ARRAY_SIZE(tas256x_dai_driver));
#endif
	} else {
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
		n_result = devm_snd_soc_register_component(plat_data->dev,
			&soc_codec_driver_tas256x,
			tas256x_dai_driver, ARRAY_SIZE(tas256x_dai_driver));
#else
		n_result = snd_soc_register_codec(plat_data->dev,
			&soc_codec_driver_tas256x,
			tas256x_dai_driver, ARRAY_SIZE(tas256x_dai_driver));
#endif
	}
	return n_result;
}
EXPORT_SYMBOL_GPL(tas256x_register_codec);

int tas256x_deregister_codec(struct tas256x_priv *p_tas256x)
{
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas256x->platform_data;
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
	snd_soc_unregister_component(plat_data->dev);
#else
	snd_soc_unregister_codec(plat_data->dev);
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(tas256x_deregister_codec);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS256X ALSA SOC Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
#endif /* CONFIG_TAS256X_CODEC */

