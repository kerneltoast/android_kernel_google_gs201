// SPDX-License-Identifier: GPL-2.0
//
// ALSA SoC driver for CS40L20/CS40L25/CS40L25A/CS40L25B
//
// Copyright (C) 2019-2020 Cirrus Logic, Inc.

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/mfd/cs40l2x.h>

#define CS40L2X_MCLK_FREQ		32768

enum cs40l2x_clk_src {
	CS40L2X_32KHZ_CLK,
	CS40L2X_SCLK
};

struct cs40l2x_codec {
	struct cs40l2x_private *core;
	struct device *dev;
	struct regmap *regmap;
	int tuning;
	int tuning_prev;
	char *bin_file;

	unsigned int daifmt;
	unsigned int rx1_slot;
	unsigned int rx2_slot;
	int sysclk_rate;
	bool dsp_a;
};

struct cs40l2x_pll_sysclk_config {
	int freq;
	int clk_cfg;
};

static const struct cs40l2x_pll_sysclk_config cs40l2x_pll_sysclk[] = {
	{ 32768,        0x00 },
	{ 8000,         0x01 },
	{ 11025,        0x02 },
	{ 12000,        0x03 },
	{ 16000,        0x04 },
	{ 22050,        0x05 },
	{ 24000,        0x06 },
	{ 32000,        0x07 },
	{ 44100,        0x08 },
	{ 48000,        0x09 },
	{ 88200,        0x0A },
	{ 96000,        0x0B },
	{ 128000,       0x0C },
	{ 176400,       0x0D },
	{ 192000,       0x0E },
	{ 256000,       0x0F },
	{ 352800,       0x10 },
	{ 384000,       0x11 },
	{ 512000,       0x12 },
	{ 705600,       0x13 },
	{ 750000,       0x14 },
	{ 768000,       0x15 },
	{ 1000000,      0x16 },
	{ 1024000,      0x17 },
	{ 1200000,      0x18 },
	{ 1411200,      0x19 },
	{ 1500000,      0x1A },
	{ 1536000,      0x1B },
	{ 2000000,      0x1C },
	{ 2048000,      0x1D },
	{ 2400000,      0x1E },
	{ 2822400,      0x1F },
	{ 3000000,      0x20 },
	{ 3072000,      0x21 },
	{ 3200000,      0x22 },
	{ 4000000,      0x23 },
	{ 4096000,      0x24 },
	{ 4800000,      0x25 },
	{ 5644800,      0x26 },
	{ 6000000,      0x27 },
	{ 6144000,      0x28 },
	{ 6250000,      0x29 },
	{ 6400000,      0x2A },
	{ 6500000,      0x2B },
	{ 6750000,      0x2C },
	{ 7526400,      0x2D },
	{ 8000000,      0x2E },
	{ 8192000,      0x2F },
	{ 9600000,      0x30 },
	{ 11289600,     0x31 },
	{ 12000000,     0x32 },
	{ 12288000,     0x33 },
	{ 12500000,     0x34 },
	{ 12800000,     0x35 },
	{ 13000000,     0x36 },
	{ 13500000,     0x37 },
	{ 19200000,     0x38 },
	{ 22579200,     0x39 },
	{ 24000000,     0x3A },
	{ 24576000,     0x3B },
	{ 25000000,     0x3C },
	{ 25600000,     0x3D },
	{ 26000000,     0x3E },
	{ 27000000,     0x3F },
};

static int cs40l2x_get_clk_config(int freq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs40l2x_pll_sysclk); i++) {
		if (cs40l2x_pll_sysclk[i].freq == freq)
			return cs40l2x_pll_sysclk[i].clk_cfg;
	}

	return -EINVAL;
}

static int cs40l2x_swap_ext_clk(struct cs40l2x_codec *priv,
				const enum cs40l2x_clk_src src)
{
	struct device *dev = priv->dev;
	struct regmap *regmap = priv->regmap;
	int clk_cfg;

	if (src == CS40L2X_32KHZ_CLK)
		clk_cfg = cs40l2x_get_clk_config(CS40L2X_MCLK_FREQ);
	else
		clk_cfg = cs40l2x_get_clk_config(priv->sysclk_rate);

	if (clk_cfg < 0) {
		dev_err(dev, "Invalid SYS Clock Frequency\n");
		return -EINVAL;
	}

	regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
			   CS40L2X_PLL_OPEN_LOOP_MASK,
			   CS40L2X_PLL_OPEN_LOOP_MASK);

	regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
			   CS40L2X_PLL_REFCLK_FREQ_MASK,
			   clk_cfg << CS40L2X_PLL_REFCLK_FREQ_SHIFT);

	if (src == CS40L2X_32KHZ_CLK)
		regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
				   CS40L2X_PLL_REFCLK_SEL_MASK,
				   CS40L2X_PLLSRC_MCLK);
	else
		regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
				   CS40L2X_PLL_REFCLK_SEL_MASK,
				   CS40L2X_PLLSRC_SCLK);

	regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
			   CS40L2X_PLL_OPEN_LOOP_MASK, 0);
	regmap_update_bits(regmap, CS40L2X_PLL_CLK_CTRL,
			   CS40L2X_PLL_REFCLK_EN_MASK,
			   CS40L2X_PLL_REFCLK_EN_MASK);

	usleep_range(1000, 1500);

	return 0;
}

static int cs40l2x_clk_en(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);
	struct cs40l2x_private *core = priv->core;
	struct device *dev = core->dev;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		dev_info(dev, "%s: SND_SOC_DAPM_POST_PMU\n", __func__);
		mutex_lock(&core->lock);
		core->a2h_enable = true;
                cs40l2x_set_state(core, CS40L2X_VIBE_STATE_RUNNING);
		mutex_unlock(&core->lock);

		ret = cs40l2x_swap_ext_clk(priv, CS40L2X_SCLK);
		if (ret)
			return ret;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		dev_info(dev, "%s: SND_SOC_DAPM_PRE_PMD\n", __func__);
		ret = cs40l2x_swap_ext_clk(priv, CS40L2X_32KHZ_CLK);
		if (ret)
			return ret;

		mutex_lock(&core->lock);
		core->a2h_enable = false;
                cs40l2x_set_state(core, CS40L2X_VIBE_STATE_STOPPED);
		mutex_unlock(&core->lock);
		break;
	default:
		dev_err(dev, "Invalid event %d\n", event);
		return -EINVAL;
	}

	return 0;
}

static int cs40l2x_a2h_ev(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);
	struct cs40l2x_private *core = priv->core;
	const struct firmware *fw;
	unsigned int reg;
	int ret;

	if (!core->dsp_reg)
		return 0;

	reg = core->dsp_reg(core, "A2HEN", CS40L2X_XM_UNPACKED_TYPE,
			    CS40L2X_ALGO_ID_A2H);
	if (!reg) {
		dev_err(priv->dev, "Cannot find the A2HENABLED register\n");
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		if (priv->tuning != priv->tuning_prev) {
			ret = request_firmware(&fw, priv->bin_file, priv->dev);
			if (ret) {
				dev_err(priv->dev, "Failed to request %s file\n",
					priv->bin_file);
				return ret;
			}

			ret = cs40l2x_ack_write(core, CS40L2X_MBOX_POWERCONTROL,
						CS40L2X_PWRCTL_FORCE_STBY,
						CS40L2X_PWRCTL_NONE);
			if (ret)
				return ret;

			ret = cs40l2x_coeff_file_parse(core, fw);
			if (ret)
				return ret;

			priv->tuning_prev = priv->tuning;

			ret =  cs40l2x_ack_write(core, CS40L2X_MBOX_POWERCONTROL,
						 CS40L2X_PWRCTL_WAKE,
						 CS40L2X_POWERCONTROL_NONE);
			if (ret)
				return ret;
		}

		return regmap_write(priv->regmap, reg, CS40L2X_A2H_ENABLE);
	case SND_SOC_DAPM_PRE_PMD:
		return regmap_write(priv->regmap, reg, CS40L2X_A2H_DISABLE);
	default:
		dev_err(priv->dev, "Invalid A2H event: %d\n", event);
		return -EINVAL;
	}
}

static int cs40l2x_pcm_ev(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);
	struct cs40l2x_private *core = priv->core;
	struct regmap *regmap = priv->regmap;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		dev_info(core->dev, "%s: SND_SOC_DAPM_POST_PMU\n", __func__);
		ret = regmap_update_bits(regmap, CS40L2X_BSTCVRT_VCTRL2,
					 CS40L2X_BST_CTL_SEL_MASK,
					 CS40L2X_BST_CTL_SEL_CLASSH);
		if (ret)
			return ret;

		ret = regmap_update_bits(regmap, CS40L2X_PWR_CTRL3,
					 CS40L2X_CLASSH_EN_MASK,
					 CS40L2X_CLASSH_EN_MASK);
		if (ret)
			return ret;

		/* Enable I2S in the DSP */
		ret = regmap_update_bits(regmap, CS40L2X_SP_ENABLES,
					 CS40L2X_ASP_RX_ENABLE_MASK,
					 CS40L2X_ASP_RX_ENABLE_MASK);
		if (ret)
			return ret;

		return cs40l2x_ack_write(core, CS40L2X_MBOX_USER_CONTROL,
					 CS40L2X_A2H_I2S_START,
					 CS40L2X_A2H_DISABLE);
	case SND_SOC_DAPM_PRE_PMD:
		dev_info(core->dev, "%s: SND_SOC_DAPM_PRE_PMD\n", __func__);
		ret = regmap_update_bits(regmap, CS40L2X_SP_ENABLES,
					 CS40L2X_ASP_RX_ENABLE_MASK, 0);
		if (ret)
			return ret;

		if (!core->cond_class_h_en) {
			ret = regmap_update_bits(regmap, CS40L2X_BSTCVRT_VCTRL2,
					 CS40L2X_BST_CTL_SEL_MASK, 0);
			if (ret)
				return ret;

			ret = regmap_update_bits(regmap, CS40L2X_PWR_CTRL3,
					 CS40L2X_CLASSH_EN_MASK, 0);
			if (ret)
				return ret;
		}

		return cs40l2x_ack_write(core, CS40L2X_MBOX_USER_CONTROL,
					 CS40L2X_A2H_I2S_END,
					 CS40L2X_A2H_DISABLE);
	default:
		dev_err(priv->dev, "Invalid PCM event: %d\n", event);
		return -EINVAL;
	}
}

static int cs40l2x_vol_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);
	struct regmap *regmap = priv->regmap;
	struct device *dev = priv->dev;
	struct cs40l2x_private *core = priv->core;
	unsigned int val = 0, reg;
	int ret;

	if (!core->dsp_reg || core->fw_id_remap != CS40L2X_FW_ID_A2H)
		return 0;

	reg = core->dsp_reg(core, "VOLUMELEVEL", CS40L2X_XM_UNPACKED_TYPE,
			    CS40L2X_ALGO_ID_A2H);
	if (!reg) {
		dev_err(dev, "Cannot find the VOLUMELEVEL register\n");
		return -EINVAL;
	}

	pm_runtime_get_sync(priv->dev);

	ret = regmap_read(regmap, reg, &val);
	if (ret)
		goto vol_get_err;

	if (val == CS40L2X_VOL_LVL_MAX)
		val = CS40L2X_VOL_LVL_MAX_STEPS;
	else
		val /= (CS40L2X_VOL_LVL_MAX / CS40L2X_VOL_LVL_MAX_STEPS);

vol_get_err:
	pm_runtime_mark_last_busy(priv->dev);
	pm_runtime_put_autosuspend(priv->dev);

	ucontrol->value.integer.value[0] = val;

	return ret;
}

static int cs40l2x_vol_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);
	struct regmap *regmap = priv->regmap;
	struct device *dev = priv->dev;
	struct cs40l2x_private *core = priv->core;
	unsigned int val, reg;
	int ret;

	if (!core->dsp_reg || core->fw_id_remap != CS40L2X_FW_ID_A2H)
		return 0;

	reg = core->dsp_reg(core, "VOLUMELEVEL", CS40L2X_XM_UNPACKED_TYPE,
			    CS40L2X_ALGO_ID_A2H);
	if (!reg) {
		dev_err(dev, "Cannot find the VOLUMELEVEL register\n");
		return -EINVAL;
	}

	val = ucontrol->value.integer.value[0];

	if (val == CS40L2X_VOL_LVL_MAX_STEPS)
		val = CS40L2X_VOL_LVL_MAX;
	else
		val *= (CS40L2X_VOL_LVL_MAX / CS40L2X_VOL_LVL_MAX_STEPS);

	pm_runtime_get_sync(priv->dev);

	ret = regmap_write(regmap, reg, val);

	pm_runtime_mark_last_busy(priv->dev);
	pm_runtime_put_autosuspend(priv->dev);

	return ret;
}

static int cs40l2x_tuning_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = priv->tuning;

	return 0;
}

static int cs40l2x_tuning_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);
	struct cs40l2x_private *core = priv->core;

	if (ucontrol->value.enumerated.item[0] == priv->tuning)
		return 0;

	if (core->a2h_enable)
		return -EBUSY;

	priv->tuning = ucontrol->value.enumerated.item[0];

	memset(priv->bin_file, 0, PAGE_SIZE);
	priv->bin_file[PAGE_SIZE - 1] = '\0';

	if (priv->tuning > 0)
		snprintf(priv->bin_file, PAGE_SIZE, "cs40l25a_a2h%d.bin",
			 priv->tuning);
	else
		snprintf(priv->bin_file, PAGE_SIZE, "cs40l25a_a2h.bin");

	return 0;
}

static int cs40l2x_delay_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);
	struct regmap *regmap = priv->regmap;
	struct device *dev = priv->dev;
	struct cs40l2x_private *core = priv->core;
	unsigned int val = 0, reg;
	int ret;

	if (!core->dsp_reg || core->fw_id_remap != CS40L2X_FW_ID_A2H)
		return 0;

	reg = core->dsp_reg(core, "LRADELAYSAMPS", CS40L2X_XM_UNPACKED_TYPE,
			    CS40L2X_ALGO_ID_A2H);
	if (!reg) {
		dev_err(dev, "Cannot find the LRADELAYSAMPS register\n");
		return -EINVAL;
	}

	pm_runtime_get_sync(priv->dev);

	ret = regmap_read(regmap, reg, &val);
	if (!ret)
		ucontrol->value.integer.value[0] = val;

	pm_runtime_mark_last_busy(priv->dev);
	pm_runtime_put_autosuspend(priv->dev);

	return ret;
}

static int cs40l2x_delay_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);
	struct regmap *regmap = priv->regmap;
	struct device *dev = priv->dev;
	struct cs40l2x_private *core = priv->core;
	unsigned int val, reg;
	int ret;

	if (!core->dsp_reg || core->fw_id_remap != CS40L2X_FW_ID_A2H)
		return 0;

	reg = core->dsp_reg(core, "LRADELAYSAMPS", CS40L2X_XM_UNPACKED_TYPE,
			    CS40L2X_ALGO_ID_A2H);

	if (!reg) {
		dev_err(dev, "Cannot find the LRADELAYSAMPS register\n");
		return -EINVAL;
	}

	val = ucontrol->value.integer.value[0];

	pm_runtime_get_sync(priv->dev);

	ret = regmap_write(regmap, reg, val);

	pm_runtime_mark_last_busy(priv->dev);
	pm_runtime_put_autosuspend(priv->dev);

	return ret;
}

static int cs40l2x_slots_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);

	ucontrol->value.integer.value[0] = priv->rx1_slot;
	ucontrol->value.integer.value[1] = priv->rx2_slot;

	return 0;
}

static int cs40l2x_slots_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);

	priv->rx1_slot = ucontrol->value.integer.value[0];
	priv->rx2_slot = ucontrol->value.integer.value[1];

	return 0;
}

static const struct snd_kcontrol_new cs40l2x_controls[] = {
	SOC_SINGLE_EXT("A2H Volume Level", 0, 0, CS40L2X_VOL_LVL_MAX_STEPS, 0,
		cs40l2x_vol_get, cs40l2x_vol_put),
	SOC_SINGLE_EXT("A2H Tuning", 0, 0, CS40L2X_A2H_MAX_TUNING, 0,
		cs40l2x_tuning_get, cs40l2x_tuning_put),
	SOC_SINGLE_EXT("A2H Delay", 0, 0, CS40L2X_A2H_DELAY_MAX, 0,
		cs40l2x_delay_get, cs40l2x_delay_put),
	SOC_DOUBLE_EXT("RX Slots", 0, 0, 1, CS40L2X_ASP_RX1_SLOT_MAX, 0,
		cs40l2x_slots_get, cs40l2x_slots_put),
};

static const char * const cs40l2x_out_mux_texts[] = { "PCM", "A2H" };
static SOC_ENUM_SINGLE_VIRT_DECL(cs40l2x_out_mux_enum, cs40l2x_out_mux_texts);
static const struct snd_kcontrol_new cs40l2x_out_mux =
	SOC_DAPM_ENUM("Haptics Source", cs40l2x_out_mux_enum);

static const struct snd_soc_dapm_widget cs40l2x_dapm_widgets[] = {
SND_SOC_DAPM_SUPPLY_S("ASP PLL", 0, SND_SOC_NOPM, 0, 0, cs40l2x_clk_en,
		      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_AIF_IN("ASPRX1", NULL, 0, SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_AIF_IN("ASPRX2", NULL, 0, SND_SOC_NOPM, 0, 0),

SND_SOC_DAPM_PGA_E("PCM", SND_SOC_NOPM, 0, 0, NULL, 0, cs40l2x_pcm_ev,
		   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_MIXER_E("A2H", SND_SOC_NOPM, 0, 0, NULL, 0, cs40l2x_a2h_ev,
		     SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

SND_SOC_DAPM_MUX("Haptics Source", SND_SOC_NOPM, 0, 0, &cs40l2x_out_mux),
SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route cs40l2x_dapm_routes[] = {
	{ "ASP Playback", NULL, "ASP PLL" },
	{ "ASPRX1", NULL, "ASP Playback" },
	{ "ASPRX2", NULL, "ASP Playback" },

	{ "PCM", NULL, "ASPRX1" },
	{ "PCM", NULL, "ASPRX2" },
	{ "A2H", NULL, "PCM" },

	{ "Haptics Source", "PCM", "PCM" },
	{ "Haptics Source", "A2H", "A2H" },
	{ "OUT", NULL, "Haptics Source" },
};

static int cs40l2x_component_set_sysclk(struct snd_soc_component *comp,
					int clk_id, int source,
					unsigned int freq, int dir)
{
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);
	int ret;

	ret = cs40l2x_get_clk_config(freq);
	if (ret < 0) {
		dev_err(priv->dev, "Invalid clock frequency: %d\n", freq);
		return -EINVAL;
	}

	switch (clk_id) {
	case 0:
		break;
	default:
		dev_err(priv->dev, "Invalid input clock\n");
		return -EINVAL;
	}

	priv->sysclk_rate = freq;

	return 0;
}

static int cs40l2x_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(dai->component);
	unsigned int asp_fmt;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		dev_err(priv->dev, "This device can be slave only\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		asp_fmt = CS40L2X_ASP_FMT_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		priv->dsp_a = true;
		asp_fmt = CS40L2X_ASP_FMT_TDM1;
		break;
	default:
		dev_err(priv->dev, "Invalid format.\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_IF:
		priv->daifmt = CS40L2X_ASP_FSYNC_INV_MASK;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		priv->daifmt = CS40L2X_ASP_BCLK_INV_MASK;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		priv->daifmt = CS40L2X_ASP_FSYNC_INV_MASK |
			       CS40L2X_ASP_BCLK_INV_MASK;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		priv->daifmt = 0;
		break;
	default:
		dev_err(priv->dev, "Invalid DAI clock format\n");
		return -EINVAL;
	}

	return regmap_update_bits(priv->regmap, CS40L2X_SP_FORMAT,
					CS40L2X_ASP_FMT_MASK,
					asp_fmt << CS40L2X_ASP_FMT_SHIFT);
}

static int cs40l2x_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_component *comp = dai->component;
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);
	unsigned int mask = CS40L2X_ASP_WIDTH_RX_MASK |
			    CS40L2X_ASP_FSYNC_INV_MASK |
			    CS40L2X_ASP_BCLK_INV_MASK;
	unsigned int bclk_rate, asp_wl, asp_width;
	int ret = 0;

	asp_wl = params_width(params);
	asp_width = params_physical_width(params);

	bclk_rate = snd_soc_params_to_bclk(params);
	dev_dbg(priv->dev, "asp_wl %d asp_width %d bclk_rate %d\n", asp_wl,
			asp_width, bclk_rate);

	if (priv->sysclk_rate != bclk_rate)
		dev_warn(priv->dev, "Expect BCLK of %dHz but got %dHz\n",
			 priv->sysclk_rate, bclk_rate);
	ret = cs40l2x_get_clk_config(bclk_rate);
	if (ret < 0)
		return ret;

	ret = regmap_write(priv->regmap, CS40L2X_SP_RATE_CTRL, ret);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, CS40L2X_SP_FORMAT, mask,
			(asp_width << CS40L2X_ASP_WIDTH_RX_SHIFT)
			| priv->daifmt);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, CS40L2X_SP_RX_WL,
			CS40L2X_ASP_RX_WL_MASK, asp_wl);
	if (ret)
		return ret;

	if (priv->dsp_a)
		ret = regmap_write(priv->regmap, CS40L2X_SP_FRAME_RX_SLOT,
				(priv->rx1_slot & CS40L2X_ASP_RX1_SLOT_MASK) |
				((priv->rx2_slot << CS40L2X_ASP_RX2_SLOT_SHIFT)
				& CS40L2X_ASP_RX2_SLOT_MASK));
	else
		ret = regmap_write(priv->regmap, CS40L2X_SP_FRAME_RX_SLOT,
				1 << CS40L2X_ASP_RX2_SLOT_SHIFT); /* default */

	return ret;
}

#define CS40L2X_RATES SNDRV_PCM_RATE_48000
#define CS40L2X_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_dai_ops cs40l2x_dai_ops = {
	.set_fmt = cs40l2x_set_dai_fmt,
	.hw_params = cs40l2x_pcm_hw_params,
};

static struct snd_soc_dai_driver cs40l2x_dai[] = {
	{
		.name = "cs40l2x-pcm",
		.id = 0,
		.playback = {
			.stream_name = "ASP Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = CS40L2X_RATES,
			.formats = CS40L2X_FORMATS,
		},
		.ops = &cs40l2x_dai_ops,
		.symmetric_rate = 1,
	},
};

static int cs40l2x_codec_probe(struct snd_soc_component *comp)
{
	struct cs40l2x_codec *priv = snd_soc_component_get_drvdata(comp);
	struct regmap *regmap = priv->regmap;

	priv->bin_file = devm_kzalloc(priv->dev, PAGE_SIZE, GFP_KERNEL);
	if (!priv->bin_file)
		return -ENOMEM;

	priv->bin_file[PAGE_SIZE - 1] = '\0';
	snprintf(priv->bin_file, PAGE_SIZE, "cs40l25a_a2h.bin");
	complete(&priv->core->hap_done);

	regmap_write(regmap, CS40L2X_DSP1RX1_INPUT, CS40L2X_DSP1_RXn_SRC_ASPRX1);
	regmap_write(regmap, CS40L2X_DSP1RX5_INPUT, CS40L2X_DSP1_RXn_SRC_ASPRX2);

	priv->sysclk_rate = 1536000;
	priv->rx1_slot = 0; /* RX1 HW default slot */
	priv->rx2_slot = 1; /* RX2 HW default slot */

	return 0;
}

static const struct snd_soc_component_driver soc_codec_dev_cs40l2x = {
	.probe = cs40l2x_codec_probe,
	.set_sysclk = cs40l2x_component_set_sysclk,

	.dapm_widgets		= cs40l2x_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(cs40l2x_dapm_widgets),
	.dapm_routes		= cs40l2x_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(cs40l2x_dapm_routes),
	.controls		= cs40l2x_controls,
	.num_controls		= ARRAY_SIZE(cs40l2x_controls),
};

static int cs40l2x_probe(struct platform_device *pdev)
{
	struct cs40l2x_private *core = dev_get_drvdata(pdev->dev.parent);
	struct cs40l2x_codec *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->core = core;
	priv->regmap = core->regmap;
	priv->dev = &pdev->dev;

	platform_set_drvdata(pdev, priv);

	pm_runtime_enable(&pdev->dev);

	ret = snd_soc_register_component(&pdev->dev, &soc_codec_dev_cs40l2x,
					 cs40l2x_dai, ARRAY_SIZE(cs40l2x_dai));
	if (ret < 0)
		dev_err(&pdev->dev, "Failed to register codec: %d\n", ret);

	return ret;
}

static int cs40l2x_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static struct platform_driver cs40l2x_codec_driver = {
	.driver = {
		.name = "cs40l2x-codec",
	},

	.probe = cs40l2x_probe,
	.remove = cs40l2x_remove,
};
module_platform_driver(cs40l2x_codec_driver);

MODULE_DESCRIPTION("ASoC CS40L2X driver");
MODULE_AUTHOR("Paul Handrigan <paul.handrigan@cirrus.com");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cs40l2x-codec");
