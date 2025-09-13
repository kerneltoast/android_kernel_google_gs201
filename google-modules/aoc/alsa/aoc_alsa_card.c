// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC ALSA Machine Driver
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/uio.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/input.h>
#include <linux/version.h>

#include "aoc_alsa_drv.h"
#include "aoc_alsa.h"
#include "google-aoc-enum.h"

static const char *aoc_detect[] = {
	"aoc_audio_state",
	"aoc_aocdump_state",
};

extern int snd_soc_component_set_jack(struct snd_soc_component *component,
				      struct snd_soc_jack *jack, void *data);

#define MK_BE_PARAMS(id, fmt, chan, sr)				\
	[AOC_ID_TO_INDEX(id)] =					\
		{ .format = fmt, .channel = chan, .rate = sr },

#define MK_TDM_BE_PARAMS(id, fmt, chan, sr, nslot, slotfmt)		\
	[AOC_ID_TO_INDEX(id)] = { .format = fmt,			\
		 .channel = chan,					\
		 .rate = sr,						\
		 .slot_num = nslot,					\
		 .slot_fmt = slotfmt },

#define MK_HW_SR_CTRL(port, xenum, xget, xput)			\
	SOC_ENUM_EXT(port " Sample Rate", xenum, xget, xput),

#define MK_HW_FMT_CTRL(port, xenum, xget, xput)			\
	SOC_ENUM_EXT(port " Format", xenum, xget, xput),

#define MK_HW_CH_CTRL(port, xenum, xget, xput)			\
	SOC_ENUM_EXT(port " Chan", xenum, xget, xput),

#define MK_HW_SLOT_NUM_CTRL(port, xenum, xget, xput)			\
	SOC_ENUM_EXT(port " nSlot", xenum, xget, xput),

#define MK_HW_SLOT_FMT_CTRL(port, xenum, xget, xput)			\
	SOC_ENUM_EXT(port " SlotFmt", xenum, xget, xput),

#define MK_HW_PARAM_CTRLS(port, name)					\
	static const struct snd_kcontrol_new _##port##_ctrls[] = {	\
		MK_HW_SR_CTRL(name, enum_sr, aoc_be_sr_get,		\
			aoc_be_sr_put)					\
		MK_HW_FMT_CTRL(name, enum_fmt, aoc_be_fmt_get,	\
				aoc_be_fmt_put)			\
		MK_HW_CH_CTRL(name, enum_ch, aoc_be_ch_get,		\
				aoc_be_ch_put)				\
	}

#define MK_TDM_HW_PARAM_CTRLS(port, name)				\
	static const struct snd_kcontrol_new _##port##_ctrls[] = {	\
		MK_HW_SR_CTRL(name, enum_sr, aoc_be_sr_get,		\
				aoc_be_sr_put)				\
		MK_HW_FMT_CTRL(name, enum_fmt, aoc_be_fmt_get,	\
				aoc_be_fmt_put)   \
		MK_HW_CH_CTRL(name, enum_ch, aoc_be_ch_get,		\
				aoc_be_ch_put)				\
		MK_HW_SLOT_NUM_CTRL(name, enum_ch, aoc_slot_num_get,	\
				aoc_slot_num_put)			\
		MK_HW_SLOT_FMT_CTRL(name, enum_fmt,			\
			aoc_slot_fmt_get, aoc_slot_fmt_put)		\
	}


#define MK_BE_RES_ITEM(port, xops, xfixup)				\
	[AOC_ID_TO_INDEX(port)] = {					\
		.ops = xops,						\
		.fixup = xfixup,					\
		.num_controls = ARRAY_SIZE(_##port##_ctrls),		\
		.controls = _##port##_ctrls,				\
	},

#define MK_STR_MAP(xstr, xval) { .str = xstr, .value = xval },

typedef int (*fixup_fn)(struct snd_soc_pcm_runtime *,
			struct snd_pcm_hw_params *);

enum {
	SRC_MCLK = 0,
	SRC_BCLK,
	SRC_PLL,
};

struct clk_ctrl {
	u32 src;
	u32 fix_clk;
	int id;
	int srcid;
	int in_mul;
	int out_mul;
	int dai_id;
	struct device_node *np;
};

struct dai_link_res_map {
	const struct snd_soc_ops *ops;
	fixup_fn fixup;
	int num_controls;
	const struct snd_kcontrol_new *controls;
};

struct be_param_cache {
	snd_pcm_format_t format;
	u32 channel;
	u32 rate;
	u32 slot_num;
	u32 slot_fmt;
};

struct snd_card_pdata {
	struct aoc_chip g_chip;
	bool has_jack;
	bool jack_init;
	u32 jack_be_id;
	u32 sys_clk_num;
	u32 pll_clk_num;
	struct mutex mutex;
	struct be_param_cache be_params[PORT_MAX];
	struct snd_soc_jack jack;
	struct device_node *jack_np;
	struct clk_ctrl *sys_clks;
	struct clk_ctrl *pll_clks;
};

struct str_to_val {
	const char *str;
	u32 value;
};

static int i2s_startup(struct snd_pcm_substream *);
static void i2s_shutdown(struct snd_pcm_substream *);
static int i2s_hw_params(struct snd_pcm_substream *,
			 struct snd_pcm_hw_params *);

static int hw_params_fixup(struct snd_soc_pcm_runtime *,
			   struct snd_pcm_hw_params *);

static int tdm_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *param);


static const struct snd_soc_ops aoc_i2s_ops = {
	.startup = i2s_startup,
	.shutdown = i2s_shutdown,
	.hw_params = i2s_hw_params,
};

static const struct snd_soc_ops aoc_tdm_ops = {
	.startup = i2s_startup,
	.shutdown = i2s_shutdown,
	.hw_params = tdm_hw_params,
};

static const struct str_to_val clksrc_map[] = {
	MK_STR_MAP("MCLK", SRC_MCLK)
	MK_STR_MAP("BCLK", SRC_BCLK)
	MK_STR_MAP("PLL", SRC_PLL)
};

static const struct str_to_val sr_map[] = {
	MK_STR_MAP("SR_8K", 8000)
	MK_STR_MAP("SR_11P025K", 11025)
	MK_STR_MAP("SR_16K", 16000)
	MK_STR_MAP("SR_22P05K", 22050)
	MK_STR_MAP("SR_32K", 32000)
	MK_STR_MAP("SR_44P1K", 44100)
	MK_STR_MAP("SR_48K", 48000)
	MK_STR_MAP("SR_88P2K", 88200)
	MK_STR_MAP("SR_96K", 96000)
	MK_STR_MAP("SR_176P4K", 176400)
	MK_STR_MAP("SR_192K", 192000)
};

static const struct str_to_val fmt_map[] = {
	MK_STR_MAP("S16_LE", SNDRV_PCM_FORMAT_S16_LE)
	MK_STR_MAP("S24_LE", SNDRV_PCM_FORMAT_S24_LE)
	MK_STR_MAP("S24_3LE", SNDRV_PCM_FORMAT_S24_3LE)
	MK_STR_MAP("S32_LE", SNDRV_PCM_FORMAT_S32_LE)
	MK_STR_MAP("FLOAT_LE", SNDRV_PCM_FORMAT_FLOAT_LE)
};

static const struct str_to_val ch_map[] = {
	MK_STR_MAP("One", 1)
	MK_STR_MAP("Two", 2)
	MK_STR_MAP("Three", 3)
	MK_STR_MAP("Four", 4)
	MK_STR_MAP("Five", 5)
	MK_STR_MAP("Six", 6)
	MK_STR_MAP("Seven", 7)
	MK_STR_MAP("Eight", 8)
};

static const char *sr_text[ARRAY_SIZE(sr_map)] = {};

static const char *fmt_text[ARRAY_SIZE(fmt_map)] = {};

static const char *ch_text[ARRAY_SIZE(ch_map)] = {};

static struct soc_enum enum_sr =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sr_text), sr_text);

static struct soc_enum enum_fmt =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fmt_text), fmt_text);

static struct soc_enum enum_ch =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ch_text), ch_text);

static const struct be_param_cache default_be_params[PORT_MAX] = {
	MK_BE_PARAMS(I2S_0_RX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_BE_PARAMS(I2S_0_TX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_BE_PARAMS(I2S_1_RX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_BE_PARAMS(I2S_1_TX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_BE_PARAMS(I2S_2_RX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_BE_PARAMS(I2S_2_TX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_TDM_BE_PARAMS(TDM_0_RX, SNDRV_PCM_FORMAT_S16_LE,
			2, 48000, 4, SNDRV_PCM_FORMAT_S32_LE)
	MK_TDM_BE_PARAMS(TDM_0_TX, SNDRV_PCM_FORMAT_S16_LE,
			2, 48000, 4, SNDRV_PCM_FORMAT_S32_LE)
	MK_TDM_BE_PARAMS(TDM_1_RX, SNDRV_PCM_FORMAT_S16_LE,
			2, 48000, 4, SNDRV_PCM_FORMAT_S32_LE)
	MK_TDM_BE_PARAMS(TDM_1_TX, SNDRV_PCM_FORMAT_S16_LE,
			2, 48000, 4, SNDRV_PCM_FORMAT_S32_LE)
	MK_BE_PARAMS(INTERNAL_MIC_TX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_BE_PARAMS(INTERNAL_MIC_US_TX, SNDRV_PCM_FORMAT_S32_LE, 2, 96000)
	MK_BE_PARAMS(ERASER_TX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_BE_PARAMS(BT_RX, SNDRV_PCM_FORMAT_S16_LE, 1, 16000)
	MK_BE_PARAMS(BT_TX, SNDRV_PCM_FORMAT_S16_LE, 1, 16000)
	MK_BE_PARAMS(USB_RX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_BE_PARAMS(USB_TX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_BE_PARAMS(INCALL_RX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_BE_PARAMS(INCALL_TX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
	MK_TDM_BE_PARAMS(HAPTIC_RX, SNDRV_PCM_FORMAT_S32_LE,
			4, 48000, 4, SNDRV_PCM_FORMAT_S32_LE)
	MK_BE_PARAMS(DP_DMA_RX, SNDRV_PCM_FORMAT_S16_LE, 2, 48000)
};

static struct snd_soc_dai_link_component null_component = {
	.name = "snd-soc-dummy",
	.dai_name = "snd-soc-dummy-dai",
};

static __poll_t audio_state_poll(struct snd_info_entry *entry,
		void *private_data, struct file *f, poll_table *wait)
{
	struct aoc_state_client_t *client = entry->private_data;

	return aoc_audio_state_poll(f, wait, client);
}

static ssize_t audio_state_read(struct snd_info_entry *entry,
		void *private_data, struct file *file, char __user *buf,
		size_t count, loff_t pos)
{
	struct aoc_state_client_t *client = entry->private_data;

	if (!client || !client->inuse)
		return -ENODEV;

	if (!buf || count != sizeof(client->online) || pos != 0)
		return -EINVAL;

	client->online = aoc_audio_current_state();

	return copy_to_user(buf, &client->online, sizeof(client->online));
}

static int audio_state_open(struct snd_info_entry *entry,
		unsigned short mode, void **private_data)
{
	struct aoc_state_client_t *client = entry->private_data;

	/* Only allow one client */
	if (!client || client->inuse)
		return -ENODEV;

	client->inuse = true;
	client->exit = false;
	return 0;
}

static int audio_state_release(struct snd_info_entry *entry,
		unsigned short mode, void *private_data)
{
	struct aoc_state_client_t *client = entry->private_data;

	if (!client)
		return -ENODEV;

	client->exit = true;
	client->inuse = false;

	return 0;
}

static struct snd_info_entry_ops audio_state_ops = {
	.poll = audio_state_poll,
	.read = audio_state_read,
	.open = audio_state_open,
	.release = audio_state_release,
};

static void audio_state_private_free(struct snd_info_entry *entry)
{
	struct aoc_state_client_t *client =
		(struct aoc_state_client_t *)entry->private_data;

	if (!client)
		return;

	entry->private_data = NULL;
	free_audio_state_client(client);
}

static int hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			   struct snd_pcm_hw_params *params)
{
	struct snd_mask *fmt_mask =
		hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	struct snd_interval *rate =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(rtd->card);
	struct snd_card_pdata *pdata =
			container_of(chip, struct snd_card_pdata, g_chip);

	u32 id = AOC_ID_TO_INDEX(cpu_dai->id);
	u32 sr, ch;
	snd_pcm_format_t fmt;

	if (id >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid id %u found for %s", __func__, id,
		       rtd->dai_link->name);
		return -EINVAL;
	}

	mutex_lock(&pdata->mutex);
	sr = pdata->be_params[id].rate;
	ch = pdata->be_params[id].channel;
	fmt = pdata->be_params[id].format;
	mutex_unlock(&pdata->mutex);

	pr_debug("%s: fixup ch %u rate %u fmt %u for %s", __func__, ch, sr,
		fmt, rtd->dai_link->name);

	rate->min = rate->max = sr;
	channels->min = channels->max = ch;

	snd_mask_none(fmt_mask);
	snd_mask_set_format(fmt_mask, fmt);

	return 0;
}

static int i2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	pr_debug("%s: %s dai_fmt = 0x%x\n", __func__,
			dai_link->name, dai_link->dai_fmt);

	return snd_soc_runtime_set_dai_fmt(rtd, dai_link->dai_fmt);
}

static void i2s_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	pr_debug("%s close\n", dai_link->name);
}

static struct clk_ctrl *find_clk(struct snd_soc_dai *dai, int clk_num,
	struct clk_ctrl *clks)
{
	struct clk_ctrl *clk = NULL;
	struct device_node *np;
	int i;

	if (!clks || !dai)
		return clk;

	np = dai->component->dev->of_node;
	for (i = 0; i < clk_num; i++, clks++) {
		if ((np && clks->np == np) || clks->dai_id == dai->id) {
			clk = clks;
			break;
		}
	}
	return clk;
}

static void set_pll_clk(struct snd_soc_dai *dai, u32 bclk, u32 rate,
	struct snd_card_pdata *pdata)
{
	struct clk_ctrl *pll;
	int ret;
	u32 clk;

	pll = find_clk(dai, pdata->pll_clk_num, pdata->pll_clks);
	if (!pll)
		return;

	if (pll->fix_clk)
		clk = pll->fix_clk;
	else {
		if (pll->src == SRC_MCLK)
			clk = rate;
		else
			clk = bclk;
	}

	pr_debug("%s: %s pll %u src %u freq_in %u freq_out %u", __func__,
		dai->name, pll->id, pll->srcid, clk * pll->in_mul,
		clk * pll->out_mul);

	ret = snd_soc_dai_set_pll(dai, pll->id, pll->srcid,
		clk * pll->in_mul, clk * pll->out_mul);
	if (ret && ret != -ENOTSUPP)
		pr_warn("%s: set codec_dai pll %s fail %d",
			  __func__, dai->name, ret);

	ret = snd_soc_component_set_pll(dai->component, pll->id, pll->srcid,
		clk * pll->in_mul, clk * pll->out_mul);
	if (ret && ret != -ENOTSUPP)
		pr_warn("%s: set codec pll %s fail %d",
			 __func__, dai->name, ret);
}

static void set_sys_clk(struct snd_soc_dai *dai, u32 bclk, u32 rate,
	u32 dir, struct snd_card_pdata *pdata)
{
	struct clk_ctrl *sys;
	int ret;
	u32 clk;

	sys = find_clk(dai, pdata->sys_clk_num, pdata->sys_clks);
	if (!sys)
		return;

	if (sys->fix_clk)
		clk = sys->fix_clk;
	else {
		if (sys->src == SRC_MCLK)
			clk = rate;
		else
			clk = bclk;
	}

	if (dir == SND_SOC_CLOCK_IN)
		clk *= sys->in_mul;
	else
		clk *= sys->out_mul;

	pr_debug("%s: %s clkid %u clk %u", __func__,
		dai->name, sys->id, clk);

	ret = snd_soc_dai_set_sysclk(dai, sys->id, clk, dir);
	if (ret && ret != -ENOTSUPP)
		pr_warn("%s: set codec_dai clk %s fail %d",
			__func__, dai->name, ret);

	ret = snd_soc_component_set_sysclk(dai->component,
		sys->id, sys->srcid, clk, dir);
	if (ret && ret != -ENOTSUPP)
		pr_warn("%s: set codec sys clk %s fail %d",
			  __func__, dai->name, ret);
}

static int i2s_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *param)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);

	u32 rate, bclk, channel;
	int i, bit_width, ret;
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(rtd->card);
	struct snd_card_pdata *pdata =
			container_of(chip, struct snd_card_pdata, g_chip);

	bit_width = params_physical_width(param);
	if (bit_width < 0) {
		pr_err("%s: invalid bit width %d", __func__, bit_width);
		return -EINVAL;
	}

	channel = params_channels(param);
	rate = params_rate(param);
	bclk = rate * ((u32)bit_width) * channel;

	pr_debug("%sv2: ch %u rate %d bit %d", __func__,
			channel, rate, bit_width);

	for_each_rtd_cpu_dais(rtd, i, cpu_dai) {
		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0,
			channel, bit_width);
		if (ret && ret != -ENOTSUPP)
			pr_warn("%s: set tdm slot %s fail %d", __func__,
				 cpu_dai->name, ret);

		set_sys_clk(cpu_dai, bclk, rate, SND_SOC_CLOCK_OUT, pdata);
	}

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_set_tdm_slot(codec_dai, 0, 0,
			channel, bit_width);
		if (ret && ret != -ENOTSUPP)
			pr_warn("%s: set tdm slot %s fail %d", __func__,
				 cpu_dai->name, ret);

		set_pll_clk(codec_dai, bclk, rate, pdata);
		set_sys_clk(codec_dai, bclk, rate, SND_SOC_CLOCK_IN, pdata);
	}
	return 0;
}

static int tdm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *param)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	u32 rate, bclk, channel, tdmslot;
	int i, bit_width, ret, slot_width;
	snd_pcm_format_t format;
	u32 idx = AOC_ID_TO_INDEX(cpu_dai->id);
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(rtd->card);
	struct snd_card_pdata *pdata =
			container_of(chip, struct snd_card_pdata, g_chip);

	if (idx >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid id %u found for %s", __func__, idx,
		       dai_link->name);
		return -EINVAL;
	}

	bit_width = params_physical_width(param);
	if (bit_width < 0) {
		pr_err("%s: invalid bit width %d", __func__, bit_width);
		return -EINVAL;
	}

	channel = params_channels(param);
	switch (dai_link->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* I2S mode */
		tdmslot = channel;
		slot_width = bit_width;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		/* TDM mode */
		tdmslot = pdata->be_params[idx].slot_num;
		format = pdata->be_params[idx].slot_fmt;
		slot_width = snd_pcm_format_physical_width(format);

		if (tdmslot < channel || slot_width < bit_width) {
			pr_err("%s: inval ch %u slot %u, bit %d, slot_bit %d",
				__func__, channel, tdmslot,
				bit_width, slot_width);
			return -EINVAL;
		}
		break;
	default:
		pr_err("%s: unsupport fmt %u on %s", __func__,
			dai_link->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK,
			dai_link->name);
		return -EINVAL;
	}

	rate = params_rate(param);

	bclk = rate * ((u32)slot_width) * tdmslot;
	pr_debug("%s:ch %u tdm slot %u bit %d, slot_bit %d", __func__,
				channel, tdmslot, bit_width, slot_width);

	for_each_rtd_cpu_dais(rtd, i, cpu_dai) {
		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0,
			tdmslot, slot_width);
		if (ret && ret != -ENOTSUPP)
			pr_warn("%s: set tdm slot %s fail %d", __func__,
				 cpu_dai->name, ret);

		set_sys_clk(cpu_dai, bclk, rate, SND_SOC_CLOCK_OUT, pdata);
	}

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_set_tdm_slot(codec_dai, 0, 0,
				 tdmslot, slot_width);
		if (ret && ret != -ENOTSUPP)
			pr_warn("%s: set tdm slot %s fail %d", __func__,
					 codec_dai->name, ret);

		set_pll_clk(codec_dai, bclk, rate, pdata);
		set_sys_clk(codec_dai, bclk, rate, SND_SOC_CLOCK_IN, pdata);
	}
	return 0;
}

static int aoc_slot_num_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = (struct snd_soc_dai *)
			snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip = (struct aoc_chip *)
		snd_soc_card_get_drvdata(cpu_dai->component->card);
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	u32 id = AOC_ID_TO_INDEX(cpu_dai->id), i;

	if (id >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid idx %u", __func__, id);
		return -EINVAL;
	}

	mutex_lock(&pdata->mutex);
	for (i = 0; i < ARRAY_SIZE(ch_map); i++) {
		if (pdata->be_params[id].slot_num == ch_map[i].value) {
			break;
		}
	}
	mutex_unlock(&pdata->mutex);

	if (i == ARRAY_SIZE(ch_map))
		return -EINVAL;

	ucontrol->value.integer.value[0] = (int)i;
	return 0;
}

static int aoc_slot_num_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai =
		(struct snd_soc_dai *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip = (struct aoc_chip *)
		snd_soc_card_get_drvdata(cpu_dai->component->card);
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	u32 id = AOC_ID_TO_INDEX(cpu_dai->id);
	int idx = ucontrol->value.integer.value[0];

	if (id >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid idx %u", __func__, id);
		return -EINVAL;
	}

	if (idx < 0 || idx >= ARRAY_SIZE(ch_map)) {
		pr_err("%s: invalid idx %d", __func__, idx);
		return -EINVAL;
	}

	mutex_lock(&pdata->mutex);
	pdata->be_params[id].slot_num = ch_map[idx].value;
	mutex_unlock(&pdata->mutex);
	return 0;
}

static int aoc_slot_fmt_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai =
		(struct snd_soc_dai *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip = (struct aoc_chip *)
		snd_soc_card_get_drvdata(cpu_dai->component->card);
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	u32 id = AOC_ID_TO_INDEX(cpu_dai->id), i;

	if (id >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid idx %u", __func__, id);
		return -EINVAL;
	}

	mutex_lock(&pdata->mutex);
	for (i = 0; i < ARRAY_SIZE(fmt_map); i++) {
		if (pdata->be_params[id].slot_fmt == fmt_map[i].value) {
			break;
		}
	}
	mutex_unlock(&pdata->mutex);

	if (i == ARRAY_SIZE(fmt_map))
		return -EINVAL;

	ucontrol->value.integer.value[0] = (int)i;
	return 0;
}

static int aoc_slot_fmt_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai =
		(struct snd_soc_dai *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip = (struct aoc_chip *)
		snd_soc_card_get_drvdata(cpu_dai->component->card);
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	u32 id = AOC_ID_TO_INDEX(cpu_dai->id);
	int idx = ucontrol->value.integer.value[0];

	if (id >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid idx %u", __func__, id);
		return -EINVAL;
	}

	if (idx < 0 || idx >= ARRAY_SIZE(fmt_map))
		return -EINVAL;

	mutex_lock(&pdata->mutex);
	pdata->be_params[id].slot_fmt = fmt_map[idx].value;
	mutex_unlock(&pdata->mutex);
	return 0;
}

static int aoc_be_sr_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai =
		(struct snd_soc_dai *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip = (struct aoc_chip *)
		snd_soc_card_get_drvdata(cpu_dai->component->card);
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	u32 id = AOC_ID_TO_INDEX(cpu_dai->id), i;

	if (id >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid idx %u", __func__, id);
		return -EINVAL;
	}

	mutex_lock(&pdata->mutex);
	for (i = 0; i < ARRAY_SIZE(sr_map); i++) {
		if (pdata->be_params[id].rate == sr_map[i].value) {
			break;
		}
	}
	mutex_unlock(&pdata->mutex);

	if (i == ARRAY_SIZE(sr_map))
		return -EINVAL;

	ucontrol->value.integer.value[0] = (int)i;
	return 0;
}

static int aoc_be_sr_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai =
		(struct snd_soc_dai *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip = (struct aoc_chip *)
		snd_soc_card_get_drvdata(cpu_dai->component->card);
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	u32 id = AOC_ID_TO_INDEX(cpu_dai->id);
	int idx = ucontrol->value.integer.value[0];

	if (id >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid idx %u", __func__, id);
		return -EINVAL;
	}

	if (idx < 0 || idx >= ARRAY_SIZE(sr_map)) {
		pr_err("%s: invalid idx %d", __func__, idx);
		return -EINVAL;
	}

	mutex_lock(&pdata->mutex);
	pdata->be_params[id].rate = sr_map[idx].value;
	mutex_unlock(&pdata->mutex);
	return 0;
}

static int aoc_be_fmt_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai =
		(struct snd_soc_dai *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip = (struct aoc_chip *)
		snd_soc_card_get_drvdata(cpu_dai->component->card);
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	u32 id = AOC_ID_TO_INDEX(cpu_dai->id), i;

	if (id >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid idx %u", __func__, id);
		return -EINVAL;
	}

	mutex_lock(&pdata->mutex);
	for (i = 0; i < ARRAY_SIZE(fmt_map); i++) {
		if (pdata->be_params[id].format == fmt_map[i].value) {
			break;
		}
	}
	mutex_unlock(&pdata->mutex);

	if (i == ARRAY_SIZE(fmt_map))
		return -EINVAL;

	ucontrol->value.integer.value[0] = (int)i;
	return 0;
}

static int aoc_be_fmt_put(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai =
		(struct snd_soc_dai *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip = (struct aoc_chip *)
		snd_soc_card_get_drvdata(cpu_dai->component->card);
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	u32 id = AOC_ID_TO_INDEX(cpu_dai->id);
	int idx = ucontrol->value.integer.value[0];

	if (id >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid idx %u", __func__, id);
		return -EINVAL;
	}

	if (idx < 0 || idx >= ARRAY_SIZE(fmt_map))
		return -EINVAL;

	mutex_lock(&pdata->mutex);
	pdata->be_params[id].format = fmt_map[idx].value;
	mutex_unlock(&pdata->mutex);
	return 0;
}

static int aoc_be_ch_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai =
		(struct snd_soc_dai *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip = (struct aoc_chip *)
		snd_soc_card_get_drvdata(cpu_dai->component->card);
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	u32 id = AOC_ID_TO_INDEX(cpu_dai->id), i;

	if (id >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid idx %u", __func__, id);
		return -EINVAL;
	}

	mutex_lock(&pdata->mutex);
	for (i = 0; i < ARRAY_SIZE(ch_map); i++) {
		if (pdata->be_params[id].channel == ch_map[i].value) {
			break;
		}
	}
	mutex_unlock(&pdata->mutex);

	if (i == ARRAY_SIZE(ch_map))
		return -EINVAL;

	ucontrol->value.integer.value[0] = (int)i;
	return 0;
}

static int aoc_be_ch_put(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai =
		(struct snd_soc_dai *)snd_kcontrol_chip(kcontrol);
	struct aoc_chip *chip = (struct aoc_chip *)
		snd_soc_card_get_drvdata(cpu_dai->component->card);
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	u32 id = AOC_ID_TO_INDEX(cpu_dai->id);
	int idx = ucontrol->value.integer.value[0];

	if (id >= ARRAY_SIZE(pdata->be_params)) {
		pr_err("%s: invalid idx %u", __func__, id);
		return -EINVAL;
	}

	if (idx < 0 || idx >= ARRAY_SIZE(ch_map))
		return -EINVAL;

	mutex_lock(&pdata->mutex);
	pdata->be_params[id].channel = ch_map[idx].value;
	mutex_unlock(&pdata->mutex);
	return 0;
}

/*
 * Declare the Sample rate, bit width, Channel controls
 * of the hardware backend port.
 *
 * Examples:
 * "I2S_0_RX Sample Rate"
 * "I2S_0_RX Format"
 * "I2S_0_RX Chan"
 */
MK_HW_PARAM_CTRLS(I2S_0_RX, "I2S_0_RX");
MK_HW_PARAM_CTRLS(I2S_0_TX, "I2S_0_TX");
MK_HW_PARAM_CTRLS(I2S_1_RX, "I2S_1_RX");
MK_HW_PARAM_CTRLS(I2S_1_TX, "I2S_1_TX");
MK_HW_PARAM_CTRLS(I2S_2_RX, "I2S_2_RX");
MK_HW_PARAM_CTRLS(I2S_2_TX, "I2S_2_TX");
MK_TDM_HW_PARAM_CTRLS(TDM_0_RX, "TDM_0_RX");
MK_TDM_HW_PARAM_CTRLS(TDM_0_TX, "TDM_0_TX");
MK_TDM_HW_PARAM_CTRLS(TDM_1_RX, "TDM_1_RX");
MK_TDM_HW_PARAM_CTRLS(TDM_1_TX, "TDM_1_TX");
MK_HW_PARAM_CTRLS(INTERNAL_MIC_TX, "INTERNAL_MIC_TX");
MK_HW_PARAM_CTRLS(INTERNAL_MIC_US_TX, "INTERNAL_MIC_US_TX");
MK_HW_PARAM_CTRLS(ERASER_TX, "ERASER_TX");
MK_HW_PARAM_CTRLS(BT_RX, "BT_RX");
MK_HW_PARAM_CTRLS(BT_TX, "BT_TX");
MK_HW_PARAM_CTRLS(USB_RX, "USB_RX");
MK_HW_PARAM_CTRLS(USB_TX, "USB_TX");
MK_HW_PARAM_CTRLS(INCALL_RX, "INCALL_RX");
MK_HW_PARAM_CTRLS(INCALL_TX, "INCALL_TX");
MK_TDM_HW_PARAM_CTRLS(HAPTIC_RX, "HAPTIC_RX");
MK_HW_PARAM_CTRLS(DP_DMA_RX, "DP_DMA_RX");

/*
 * The resource array that have ALSA controls, ops and fixup
 * funciton of each backend port.
 *
 */
static const struct dai_link_res_map be_res_map[PORT_MAX] = {
	MK_BE_RES_ITEM(I2S_0_RX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(I2S_0_TX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(I2S_1_RX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(I2S_1_TX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(I2S_2_RX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(I2S_2_TX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(TDM_0_RX, &aoc_tdm_ops, hw_params_fixup)
	MK_BE_RES_ITEM(TDM_0_TX, &aoc_tdm_ops, hw_params_fixup)
	MK_BE_RES_ITEM(TDM_1_RX, &aoc_tdm_ops, hw_params_fixup)
	MK_BE_RES_ITEM(TDM_1_TX, &aoc_tdm_ops, hw_params_fixup)
	MK_BE_RES_ITEM(INTERNAL_MIC_TX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(INTERNAL_MIC_US_TX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(ERASER_TX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(BT_RX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(BT_TX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(USB_RX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(USB_TX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(INCALL_RX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(INCALL_TX, &aoc_i2s_ops, hw_params_fixup)
	MK_BE_RES_ITEM(HAPTIC_RX, &aoc_tdm_ops, hw_params_fixup)
	MK_BE_RES_ITEM(DP_DMA_RX, &aoc_i2s_ops, hw_params_fixup)
};

static void put_component(struct snd_soc_dai_link_component *component,
	unsigned int number)
{
	uint32_t i;

	if (!component)
		return;

	for (i = 0; i < number; i++, component++) {
		if (!component->of_node)
			continue;

		of_node_put(component->of_node);
		component->of_node = NULL;
	}
}

static void put_dai_links(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *dai_link;
	uint32_t i;

	if (!card || !card->dai_link || !card->num_links)
		return;

	dai_link = card->dai_link;

	for (i = 0; i < card->num_links; i++, dai_link++) {
		put_component(dai_link->cpus, dai_link->num_cpus);
		put_component(dai_link->codecs, dai_link->num_codecs);
		put_component(dai_link->platforms, dai_link->num_platforms);
	}
}

static int of_parse_dai_platform(struct device *dev,
	struct device_node *node, struct snd_soc_dai_link *dai)
{
	struct device_node *of_platform_root = NULL;
	struct device_node *of_node;
	struct device_node *np;
	int count, ret = 0;
	struct snd_soc_dai_link_component *component;

	/*
	 * The platform is not must have
	 */
	of_platform_root = of_get_child_by_name(node, "platform");
	if (!of_platform_root)
		return 0;

	count = of_get_available_child_count(of_platform_root);
	if (count <= 0) {
		pr_err("invalid child count %d for %s\n", count,
			of_platform_root->name);
		ret = -EINVAL;
		goto exit;
	}

	component = devm_kzalloc(dev,
		sizeof(struct snd_soc_dai_link_component) * count, GFP_KERNEL);
	if (!component) {
		ret = -ENOMEM;
		goto exit;
	}

	dai->platforms = component;
	dai->num_platforms = count;

	count = 0;
	for_each_available_child_of_node(of_platform_root, np) {
		of_node = of_parse_phandle(np, "of_drv", 0);
		if (!of_node) {
			pr_err("%s: no of_drv for %s", __func__,
				of_node->name);
			ret = -EINVAL;
			break;
		}

		component->of_node = of_node;
		component++;
		count++;
	}

exit:
	if (of_platform_root)
		of_node_put(of_platform_root);
	return ret;
}

static int of_parse_dai_cpu(struct device *dev,
	struct device_node *node, struct snd_soc_dai_link *dai)
{
	struct device_node *of_cpu_root = NULL, *of_node;
	struct snd_soc_dai_link_component *component;
	int ret;

	/*
	 * Each FE/BE must specify the cpu dai
	 */
	of_cpu_root = of_get_child_by_name(node, "cpu");
	if (!of_cpu_root) {
		pr_err("%s: can't find cpu node for %s", __func__, dai->name);
		return -EINVAL;
	}

	of_node = of_parse_phandle(of_cpu_root, "sound-dai", 0);
	if (!of_node) {
		pr_err("%s: fail to get cpu dai for %s", __func__, dai->name);
		ret = -EINVAL;
		goto exit;
	}

	component = devm_kzalloc(dev,
		sizeof(struct snd_soc_dai_link_component), GFP_KERNEL);
	if (!component) {
		ret = -ENOMEM;
		goto exit;
	}

	/* Only support single cpu dai */
	dai->cpus = component;
	dai->num_cpus = 1;
	component->of_node = of_node;

	ret = snd_soc_of_get_dai_name(of_cpu_root, &component->dai_name);
	if (ret) {
		if (ret == -EPROBE_DEFER) {
			pr_info("%s: wait cpu_dai for %s", __func__, dai->name);
		} else {
			pr_err("%s: get cpu_dai fail for %s", __func__,
			       dai->name);
		}
	}

exit:
	if (of_cpu_root)
		of_node_put(of_cpu_root);
	return ret;
}

static int of_parse_dai_codec(struct device *dev,
	struct device_node *node, struct snd_soc_dai_link *dai)
{
	struct device_node *of_codec_root;
	int ret;

	of_codec_root = of_get_child_by_name(node, "codec");
	if (!of_codec_root) {
		/* default codec */
		dai->codecs = &null_component;
		dai->num_codecs = 1;
		return 0;
	}

	ret = snd_soc_of_get_dai_link_codecs(dev, of_codec_root, dai);
	of_node_put(of_codec_root);
	return ret;
}

static int of_parse_one_dai(struct device_node *node, struct device *dev,
	struct snd_soc_dai_link *dai)
{
	int ret = 0;
	bool ops, fixup;
	u32 trigger, id;
	struct device_node *daifmt = NULL;

	if (!node || !dai)
		return -EINVAL;

	ret = of_property_read_string(node, "dai-name", &dai->name);
	if (ret) {
		pr_err("%s: fail to get dai name %d", __func__, ret);
		goto exit;
	}

	ret = of_property_read_string(node, "stream-name", &dai->stream_name);
	if (ret) {
		pr_err("%s: fail to get dai stream name %d", __func__, ret);
		goto exit;
	}

	ret = of_parse_dai_cpu(dev, node, dai);
	if (ret) {
		pr_err("%s: fail to parse cpu %d for %s", __func__, ret, dai->name);
		goto exit;
	}

	ret = of_parse_dai_platform(dev, node, dai);
	if (ret) {
		pr_err("%s: fail to parse platform %d for %s", __func__, ret, dai->name);
		goto exit;
	}

	ret = of_parse_dai_codec(dev, node, dai);
	if (ret) {
		pr_err("%s: fail to parse codec %d for %s", __func__, ret, dai->name);
		goto exit;
	}

	ret = of_property_read_u32_index(node, "trigger", 0, &trigger);
	if (ret == 0) {
		switch (trigger) {
		case 1:
			dai->trigger[0] = SND_SOC_DPCM_TRIGGER_POST;
			dai->trigger[1] = SND_SOC_DPCM_TRIGGER_POST;
			break;
		case 2:
			dai->trigger[0] = SND_SOC_DPCM_TRIGGER_BESPOKE;
			dai->trigger[1] = SND_SOC_DPCM_TRIGGER_BESPOKE;
			break;
		default:
			dai->trigger[0] = SND_SOC_DPCM_TRIGGER_PRE;
			dai->trigger[1] = SND_SOC_DPCM_TRIGGER_PRE;
			break;
		}
	} else {
		/* default setting */
		dai->trigger[0] = SND_SOC_DPCM_TRIGGER_POST;
		dai->trigger[1] = SND_SOC_DPCM_TRIGGER_POST;
	}

	ret = of_property_read_u32_index(node, "id", 0, &id);
	if (ret == 0) {
		dai->id = id;
		id = AOC_ID_TO_INDEX(id);

		if (dai->id & AOC_BE) {
			if (id < ARRAY_SIZE(be_res_map)) {
				ops = of_property_read_bool(node, "useops");
				fixup = of_property_read_bool(node, "usefixup");

				if (ops)
					dai->ops = be_res_map[id].ops;

				if (fixup)
					dai->be_hw_params_fixup =
						be_res_map[id].fixup;
			}
		}
	}

	daifmt = of_get_child_by_name(node, "daifmt");
	if (daifmt) {
		dai->dai_fmt = snd_soc_daifmt_parse_format(daifmt, NULL);
		dai->dai_fmt |=
			snd_soc_daifmt_parse_clock_provider_as_flag(daifmt, NULL);
		of_node_put(daifmt);
		pr_debug("%s: daifmt 0x%x for %s", __func__, dai->dai_fmt,
			dai->name);
	}

	dai->dpcm_playback = of_property_read_bool(node, "playback");
	dai->dpcm_capture = of_property_read_bool(node, "capture");
	dai->no_pcm = of_property_read_bool(node, "no-pcm");
	dai->dynamic = of_property_read_bool(node, "dynamic");
	dai->ignore_pmdown_time =
		!of_property_read_bool(node, "require-pmdown-time");
	dai->ignore_suspend = !of_property_read_bool(node, "require-suspend");
exit:

	return ret;
}

static int aoc_of_parse_dai_link(struct device_node *node,
				 struct snd_soc_card *card)
{
	int ret = 0, count;
	struct device_node *np_dai;
	struct device_node *np = NULL;
	struct device *dev = card->dev;
	struct snd_soc_dai_link *dai_link;

	np_dai = of_get_child_by_name(node, "dai_link");
	if (!np_dai) {
		pr_err("%s: can't find dai-link node", __func__);
		return -EINVAL;
	}

	count = (int)of_get_available_child_count(np_dai);
	if (count <= 0) {
		pr_err("%s: count %d invalid", __func__, count);
		ret = -EINVAL;
		goto err;
	}

	dai_link = devm_kzalloc(dev, sizeof(struct snd_soc_dai_link) * count,
				GFP_KERNEL);
	if (!dai_link) {
		pr_err("%s: fail to allocate memory for dai_link", __func__);
		ret = -ENOMEM;
		goto err;
	}

	card->num_links = count;
	card->dai_link = dai_link;

	count = 0;
	for_each_available_child_of_node (np_dai, np) {
		if (count >= card->num_links) {
			pr_err("%s: dai link num is full %u", __func__, count);
			break;
		}

		ret = of_parse_one_dai(np, card->dev, dai_link);
		if (ret) {
			if (ret == -EPROBE_DEFER) {
				pr_info("%s: register sound card later",
					__func__);
			} else {
				/*
				 * If any dai link error, then
				 * sound card should be fail
				 */
				pr_err("%s: fail to parse %s", __func__,
					np->name);
			}
			break;
		}
#ifdef DUMP_DAI_LINK_INFO
		pr_info("dai: %s\n", dai_link->name);
		pr_info("id: %u\n", (uint32_t)dai_link->id);
		pr_info("playback %u capture %u\n", dai_link->dpcm_playback,
			dai_link->dpcm_capture);
		pr_info("no-pcm: %u\n", dai_link->no_pcm);
		pr_info("dynamic: %u\n", dai_link->dynamic);
		pr_info("\n");
#endif
		dai_link++;
		count++;
	}
	if (ret < 0)
		goto err;

	card->num_links = count;
	of_node_put(np_dai);
	return ret;

err:
	put_dai_links(card);
	of_node_put(np_dai);
	return ret;
}

static int of_parse_one_codec_cfg(struct device_node *node,
	struct snd_soc_codec_conf *codec_cfg)
{
	int ret = 0;
	struct device_node *of_node;

	if (!node || !codec_cfg)
		return -EINVAL;

	of_node = of_parse_phandle(node, "of_node", 0);
	if (!of_node) {
		pr_err("%s: fail to get of_node for %s", __func__, node->name);
		ret = -EINVAL;
		goto err;
	}

	codec_cfg->dlc.of_node = of_node;

	ret = of_property_read_string(node, "prefix", &codec_cfg->name_prefix);
	if (ret) {
		pr_err("%s: fail to get prefix for %s %d", __func__, node->name,
		       ret);
		goto err;
	}
	return 0;

err:
	return ret;
}

static int aoc_of_parse_codec_conf(struct device_node *node,
	struct snd_soc_card *card)
{
	int ret = 0, count;
	struct device_node *np_cfg;
	struct device_node *np = NULL;
	struct snd_soc_codec_conf *codec_cfg;
	struct device *dev = card->dev;

	np_cfg = of_get_child_by_name(node, "codec_cfg");
	if (!np_cfg) {
		pr_info("%s: can't find codec cfg node", __func__);
		return 0;
	}

	count = (int)of_get_available_child_count(np_cfg);
	if (count <= 0) {
		pr_err("%s: count %d invalid", __func__, count);
		ret = -EINVAL;
		goto err;
	}

	codec_cfg = devm_kzalloc(dev, sizeof(struct snd_soc_codec_conf) * count,
				 GFP_KERNEL);
	if (!codec_cfg) {
		pr_err("%s: fail to allocate memory for codec_cfg", __func__);
		ret = -ENOMEM;
		goto err;
	}

	card->num_configs = count;
	card->codec_conf = codec_cfg;

	count = 0;
	for_each_available_child_of_node (np_cfg, np) {
		if (count >= card->num_configs) {
			pr_err("%s: conf num is full %u", __func__, count);
			break;
		}

		ret = of_parse_one_codec_cfg(np, codec_cfg);
		if (ret) {
			memset(codec_cfg, 0, sizeof(*codec_cfg));
			continue;
		}

		codec_cfg++;
		count++;
	}
	card->num_configs = count;
	ret = 0;

err:
	of_node_put(np_cfg);
	return ret;
}

static int aoc_of_parse_hs_jack(struct device_node *node,
	struct snd_card_pdata *pdata)
{
	struct device_node *np_cfg;
	int ret;

	np_cfg = of_get_child_by_name(node, "hs_jack");
	if (!np_cfg) {
		pr_info("%s: no hs jack", __func__);
		return 0;
	}

	ret = of_property_read_u32_index(np_cfg, "be_id",
						0, &pdata->jack_be_id);
	if (ret != 0) {
		pr_err("%s: fail to parse id %d\n", __func__, ret);
		goto err_exit;
	}

	pdata->jack_np = of_parse_phandle(np_cfg, "codec", 0);
	if (!pdata->jack_np) {
		pr_err("%s: fail to codec np\n", __func__);
		goto err_exit;
	}

	pdata->has_jack = true;
	return 0;

err_exit:
	of_node_put(np_cfg);
	return ret;
}

static int aoc_of_parse_hac_amp(struct device_node *node,
	struct snd_soc_card *card, struct snd_card_pdata *pdata)
{
	pdata->g_chip.hac_amp_en_gpio = devm_gpiod_get_optional(card->dev,
			"hac_amp_en", GPIOD_OUT_LOW);
	if (!IS_ERR(pdata->g_chip.hac_amp_en_gpio)) {
		pr_info("platform has hac amp\n");
	}
	return 0;
}

static int aoc_of_parse_clk(struct device_node *np_clk,
	struct snd_soc_card *card, u32 *clk_num, struct clk_ctrl **clks)
{
	struct device *dev = card->dev;
	struct device_node *np;
	struct clk_ctrl *cur;
	int count, ret = 0, i;
	const char *clk_type = NULL;
	u32 fixclk;

	if (!clks || !clk_num || !np_clk)
		return -EINVAL;

	*clk_num = 0;
	count = (int)of_get_available_child_count(np_clk);
	if (count <= 0)
		return ret;

	*clks = devm_kzalloc(dev, sizeof(struct clk_ctrl) * count, GFP_KERNEL);
	if (!(*clks)) {
		pr_err("parse_clk: fail to alloc mem");
		return -ENOMEM;
	}

	cur = *clks;

	for_each_available_child_of_node(np_clk, np) {
		if (*clk_num >= count) {
			pr_err("%s: %s clk number overflow %d %d\n",
				__func__, np->name, *clk_num, count);
			ret = -EINVAL;
			goto err_exit;
		}

		cur->np = of_parse_phandle(np, "comp", 0);
		if (!cur->np) {
			ret = of_property_read_u32(np, "dai_id", &cur->dai_id);
			if (ret) {
				pr_err("%s: %s fail to parse comp\n",
					__func__, np->name);
				goto err_exit;
			}
		} else {
			/*
			 * If the of_node exists, then set the dai_id to
			 * be 0xFFFFFFFF
			 */
			cur->dai_id = 0xFFFFFFFF;
		}

		ret = of_property_read_string(np, "src", &clk_type);
		if (ret) {
			pr_err("%s: %s fail to clk type %d",
				__func__, np->name, ret);
			goto err_exit;
		}

		if (!clk_type) {
			pr_err("%s: %s clk_type is NULL", __func__, np->name);
			ret = -EINVAL;
			goto err_exit;
		}

		ret = -EINVAL;
		for (i = 0; i < ARRAY_SIZE(clksrc_map); i++) {
			if (!strcmp(clk_type, clksrc_map[i].str)) {
				ret = 0;
				cur->src = clksrc_map[i].value;
				break;
			}
		}
		if (ret) {
			pr_err("%s: %s fail to convert type %s",
				__func__, np->name, clk_type);
			goto err_exit;
		}

		ret = of_property_read_u32(np, "id", &cur->id);
		if (ret != 0) {
			pr_err("%s: %s fail to parse id %d\n",
				__func__, np->name, ret);
			goto err_exit;
		}

		ret = of_property_read_u32(np, "srcid", &cur->srcid);
		if (ret != 0) {
			pr_err("%s: %s fail to parse srcid %d\n",
				__func__, np->name, ret);
			goto err_exit;
		}

		ret = of_property_read_u32(np, "in_mul", &cur->in_mul);
		if (ret != 0) {
			pr_err("%s: %s fail to parse in_mul %d\n",
				__func__, np->name, ret);
			goto err_exit;
		}

		ret = of_property_read_u32(np, "out_mul", &cur->out_mul);
		if (ret != 0) {
			pr_err("%s: %s fail to parse out_mul %d\n",
				__func__, np->name, ret);
			goto err_exit;
		}

		ret = of_property_read_u32(np, "fixclk", &fixclk);
		if (ret == 0)
			cur->fix_clk = fixclk;

		if (cur->src == SRC_PLL && !cur->fix_clk) {
			pr_err("%s: %s PLL requires fixup clk\n",
				__func__, np->name);
			goto err_exit;
		}

		(*clk_num)++;
		cur++;
	}

	return 0;

err_exit:
	*clk_num = 0;
	devm_kfree(dev, *clks);
	*clks = NULL;
	return ret;
}

static int aoc_of_parse_clks(struct device_node *node,
	struct snd_soc_card *card, struct snd_card_pdata *pdata)
{
	struct device_node *np_clks, *cur;
	int ret = 0;

	np_clks = of_get_child_by_name(node, "clks");
	if (!np_clks) {
		pr_info("%s: no clks", __func__);
		return ret;
	}

	/* Parse the sys clock */
	cur = of_get_child_by_name(np_clks, "sys");
	if (cur) {
		ret = aoc_of_parse_clk(cur, card, &pdata->sys_clk_num,
				 &pdata->sys_clks);
		of_node_put(cur);
		if (ret < 0) {
			pr_err("%s: fail to parse sysclk %d", __func__, ret);
			goto err_exit;
		}
	}

	/* Parse the pll clock */
	cur = of_get_child_by_name(np_clks, "pll");
	if (cur) {
		ret = aoc_of_parse_clk(cur, card, &pdata->pll_clk_num,
				 &pdata->pll_clks);
		of_node_put(cur);
		if (ret < 0) {
			pr_err("%s: fail to parse sysclk %d", __func__, ret);
			goto err_exit;
		}
	}

err_exit:
	of_node_put(np_clks);
	return ret;
}

static int aoc_snd_card_parse_of(struct device_node *node,
	struct snd_soc_card *card, struct snd_card_pdata *pdata)
{
	int ret;

	ret = aoc_of_parse_dai_link(node, card);
	if (ret) {
		pr_err("%s: fail to parse fai_link %d", __func__, ret);
		goto err;
	}

	ret = aoc_of_parse_codec_conf(node, card);
	if (ret) {
		pr_err("%s: fail to parse codec conf %d", __func__, ret);
		goto err;
	}

	ret = aoc_of_parse_hs_jack(node, pdata);
	if (ret) {
		pr_err("%s: fail to parse hs jack %d", __func__, ret);
		goto err;
	}

	ret = aoc_of_parse_hac_amp(node, card, pdata);
	if (ret) {
		pr_err("%s: fail to parse hac amp %d", __func__, ret);
		goto err;
	}

	ret = aoc_of_parse_clks(node, card, pdata);
	if (ret) {
		pr_err("%s: fail to parse clks %d", __func__, ret);
		goto err;
	}

	ret = snd_soc_of_parse_card_name(card, "aoc-card-name");
	if (ret) {
		pr_err("%s: fail to parse snd card name %d", __func__, ret);
		goto err;
	}

err:
	return ret;
}

static void init_audio_state_query(struct snd_soc_card *card, const char *name)
{
	struct snd_info_entry *entry = NULL;
	struct aoc_state_client_t *client;

	client = alloc_audio_state_client();
	if (!client) {
		pr_err("fail to allocate %s client\n", name);
		return;
	}

	snd_card_proc_new(card->snd_card, name, &entry);
	if (!entry) {
		pr_warn("%s: fail to create entry %s\n", __func__, name);
		free_audio_state_client(client);
		return;
	}

	entry->content = SNDRV_INFO_CONTENT_DATA;
	entry->private_data = client;
	entry->c.ops = &audio_state_ops;
	entry->private_free = audio_state_private_free;
	entry->size = sizeof(client->online);
}

static void init_headset_jack(struct snd_soc_card *card,
	struct snd_soc_pcm_runtime *rtd, u32 id)
{
	struct snd_soc_dai *codec_dai, *target_dai = NULL;
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(card);
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	int i, err;

	if (!pdata || !pdata->has_jack || pdata->jack_init ||
		pdata->jack_be_id != id)
		return;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		if (pdata->jack_np ==
			codec_dai->component->dev->of_node) {
			target_dai = codec_dai;
			break;
		}
	}

	if (!target_dai) {
		pr_err("Fail to find target_dai for headset jack\n");
		return;
	}

	/* setup hs jack */
	err = snd_soc_card_jack_new_pins(card, "Headset Jack",
		SND_JACK_HEADSET | SND_JACK_BTN_0 |
		SND_JACK_BTN_1 | SND_JACK_BTN_2 |
		SND_JACK_BTN_3, &pdata->jack,	NULL, 0);
	if (err) {
		pr_err("Fail to create headset jack %d\n",
			err);
		return;
	}

	snd_jack_set_key(pdata->jack.jack,
		SND_JACK_BTN_0, KEY_MEDIA);

	snd_jack_set_key(pdata->jack.jack,
		SND_JACK_BTN_1, KEY_VOICECOMMAND);

	snd_jack_set_key(pdata->jack.jack,
		SND_JACK_BTN_2, KEY_VOLUMEUP);

	snd_jack_set_key(pdata->jack.jack,
		SND_JACK_BTN_3, KEY_VOLUMEDOWN);

	err = snd_soc_component_set_jack(target_dai->component,
		&pdata->jack, NULL);
	if (err == 0)
		pdata->jack_init = true;

	if (!pdata->jack_init)
		pr_warn("%s: fail to init hs jack %s\n", __func__,
			(pdata->jack_np)?pdata->jack_np->name:"");
}

static void init_backend_control(struct snd_soc_pcm_runtime *rtd, u32 id)
{
	u32 idx;
	struct snd_soc_dai *cpu_dai;

	idx = AOC_ID_TO_INDEX(id);
	if (idx >= ARRAY_SIZE(be_res_map))
		return;

	if (be_res_map[idx].num_controls == 0 ||
	    !be_res_map[idx].controls)
		return;

	cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	snd_soc_add_dai_controls(cpu_dai,
		 be_res_map[idx].controls, be_res_map[idx].num_controls);
}

static int aoc_card_late_probe(struct snd_soc_card *card)
{
	struct aoc_chip *chip =
		(struct aoc_chip *)snd_soc_card_get_drvdata(card);
	int err, i;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_card_pdata *pdata =
		container_of(chip, struct snd_card_pdata, g_chip);
	u32 id;

	chip->card = card->snd_card;

	/*
	 * TODO: make the service list
	 * NOT have to be in the same order as pcm device list
	 */
	for (i = 0; i < aoc_audio_service_num() - 2; i++) {
		chip->avail_substreams |= (1 << i);
	}

	err = snd_aoc_new_ctl(chip);
	if (err < 0)
		pr_err("%s: fail to new ctrl %d", __func__, err);


	/* Default BE setting */
	memcpy(pdata->be_params, default_be_params, sizeof(default_be_params));

	/* Register HW control */
	list_for_each_entry (rtd, &card->rtd_list, list) {
		if (!rtd->dai_link->no_pcm)
			continue;

		id = rtd->dai_link->id;
		if (!(id & AOC_BE))
			continue;

		init_headset_jack(card, rtd, id);
		init_backend_control(rtd, id);
	}

	/* add for aocdump detect aoc SSR */
	for (i = 0; i < ARRAY_SIZE(aoc_detect); i++)
		init_audio_state_query(card, aoc_detect[i]);

	return 0;
}

static int snd_aoc_init(struct aoc_chip *chip)
{
	int i;
	struct device_node *aoc_node;

	chip->mic_loopback_enabled = 0;

	chip->default_mic_id = DEFAULT_MICPHONE_ID;
	chip->buildin_mic_id_list[0] = DEFAULT_MICPHONE_ID;
	chip->buildin_us_mic_id_list[0] = DEFAULT_MICPHONE_ID;
	for (i = 1; i < NUM_OF_BUILTIN_MIC; i++) {
		chip->buildin_mic_id_list[i] = -1;
		chip->buildin_us_mic_id_list[i] = -1;
	}

	for (i = 0; i < NUM_OF_MIC_BROKEN_RECORD; i++) {
		chip->buildin_mic_broken_detect[i] = -1;
	}

	chip->default_sink_id = DEFAULT_AUDIO_SINK_ID;
	chip->sink_id_list[0] = DEFAULT_AUDIO_SINK_ID;
	for (i = 1; i < ARRAY_SIZE(chip->sink_id_list); i++) {
		chip->sink_id_list[i] = -1;
	}

	chip->audio_capture_mic_source = BUILTIN_MIC;
	chip->voice_call_mic_source = 0;
	chip->voice_call_mic_mute = 0;
	chip->ft_aec_ref_source = DEFAULT_PLAYBACK;
	chip->eraser_aec_ref_source = DEFAULT_PLAYBACK;
	chip->compr_offload_volume = 15;
	chip->voice_call_audio_enable = 1;
	chip->mic_spatial_module_enable = 0;
	chip->capture_eraser_enable = 0;
	chip->hotword_tap_enable = 0;
	chip->sidetone_enable = 0;
	chip->voip_rx_prepared = 0;
	chip->voip_tx_prepared = 0;
	chip->telephony_curr_mic = NULL_PATH;
	chip->telephony_curr_sink = NULL_PATH;
	chip->telephony_expect_mic = NULL_PATH;
	chip->telephony_expect_sink = NULL_PATH;

	chip->pcm_wait_time_in_ms = DEFAULT_PCM_WAIT_TIME_IN_MSECS;
	chip->voice_pcm_wait_time_in_ms = DEFAULT_VOICE_PCM_WAIT_TIME_IN_MSECS;

	/* Default values for playback volume and mute */
	chip->volume = 1000;
	chip->mute = 1;

	/* Default values for incall mic gain and mute */
	chip->incall_mic_muted = false;
	chip->incall_mic_gain_current = 0;
	chip->incall_mic_gain_target = 0;

	mutex_init(&chip->audio_mutex);
	mutex_init(&chip->audio_cmd_chan_mutex);
	spin_lock_init(&chip->audio_lock);

	aoc_node = of_find_compatible_node(NULL, NULL, "google,aoc");
	if (!aoc_node)
		pr_err("Cannot find aoc device node\n");

	chip->hotword_supported = of_property_read_bool(aoc_node, "hotword-supported");
	chip->chre_supported = of_property_read_bool(aoc_node, "chre-supported");

	of_node_put(aoc_node);
	return 0;
}

static int aoc_snd_card_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct snd_soc_card *card;
	int ret;
	struct aoc_service_dev *aoc_dev;
	struct snd_card_pdata *pdata;

	pr_info("%s", __func__);
	if (!np)
		return -ENOSYS;

	/* Check if the AoC service is up */
	ret = alloc_aoc_audio_service(CMD_OUTPUT_CHANNEL, &aoc_dev, NULL, NULL);
	if (ret < 0) {
		if (ret == -EPROBE_DEFER)
			pr_info("%s: wait for aoc output ctrl\n", __func__);
		else
			pr_err("%s: Failed to get aoc output ctrl %d\n",
			       __func__, ret);
		goto err;
	} else {
		free_aoc_audio_service(CMD_OUTPUT_CHANNEL, aoc_dev);
	}

	/* Allocate the private data and the DAI link array */
	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card) {
		pr_err("%s: fail to allocate mem", __func__);
		return -ENOMEM;
	}

	/* Allocate the private data */
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("%s: fail to allocate mem for pdata", __func__);
		return -ENOMEM;
	}

	ret = snd_aoc_init(&pdata->g_chip);
	if (ret < 0) {
		pr_err("%s: Failed to init aoc chip\n", __func__);
		goto err;
	}

	pdata->g_chip.wakelock = wakeup_source_register(dev, dev_name(dev));

	card->driver_name = AOC_SND_CARD;
	card->owner = THIS_MODULE;
	card->dev = dev;
	card->late_probe = aoc_card_late_probe;
	mutex_init(&pdata->mutex);

	ret = aoc_snd_card_parse_of(np, card, pdata);
	if (ret) {
		goto err;
	}

	snd_soc_card_set_drvdata(card, &pdata->g_chip);
	ret = snd_soc_register_card(card);
	if (ret < 0) {
		if (ret == -EPROBE_DEFER) {
			pr_info("%s: defer the probe %d", __func__, ret);
		} else
			pr_info("%s: snd register fail %d", __func__, ret);
		goto err;
	}

	return 0;

err:
	return ret;
}

static int aoc_snd_card_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	if (card) {
		snd_soc_unregister_card(card);
		snd_soc_card_set_drvdata(card, NULL);
	}

	return 0;
}

static const struct of_device_id aoc_snd_of_match[] = {
	{
		.compatible = "google-aoc-snd-card",
	},
	{},
};
MODULE_DEVICE_TABLE(of, aoc_snd_of_match);

static struct platform_driver aoc_snd_card_drv = {
	.driver = {
		.name = "google-aoc-snd-card",
		.of_match_table = aoc_snd_of_match,
	},
	.probe = aoc_snd_card_probe,
	.remove = aoc_snd_card_remove,
};

static int aoc_card_init(void)
{
	int ret = 0, i;
	pr_info("%s", __func__);
	for (i = 0; i < ARRAY_SIZE(sr_map); i++)
		sr_text[i] = sr_map[i].str;

	for (i = 0; i < ARRAY_SIZE(fmt_map); i++)
		fmt_text[i] = fmt_map[i].str;

	for (i = 0; i < ARRAY_SIZE(ch_map); i++)
		ch_text[i] = ch_map[i].str;

	ret = platform_driver_register(&aoc_snd_card_drv);
	if (ret) {
		pr_err("error registering aoc pcm drv %d .\n", ret);
		goto exit;
	}

exit:
	return ret;
}

static void aoc_card_exit(void)
{
	platform_driver_unregister(&aoc_snd_card_drv);
}

module_init(aoc_card_init);
module_exit(aoc_card_exit);

MODULE_AUTHOR("google aoc team");
MODULE_DESCRIPTION("Alsa driver for aoc sound card");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:aoc_alsa_card");
