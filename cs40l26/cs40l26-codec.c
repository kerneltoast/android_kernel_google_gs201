// SPDX-License-Identifier: GPL-2.0
//
// cs40l26.c -- ALSA SoC Audio driver for Cirrus Logic Haptic Device: CS40L26
//
// Copyright 2022 Cirrus Logic. Inc.

#include "cs40l26.h"

static const struct cs40l26_pll_sysclk_config cs40l26_pll_sysclk[] = {
	{CS40L26_PLL_CLK_FRQ_32768, CS40L26_PLL_CLK_CFG_32768},
	{CS40L26_PLL_CLK_FRQ_1536000, CS40L26_PLL_CLK_CFG_1536000},
	{CS40L26_PLL_CLK_FRQ_3072000, CS40L26_PLL_CLK_CFG_3072000},
	{CS40L26_PLL_CLK_FRQ_6144000, CS40L26_PLL_CLK_CFG_6144000},
	{CS40L26_PLL_CLK_FRQ_9600000, CS40L26_PLL_CLK_CFG_9600000},
	{CS40L26_PLL_CLK_FRQ_12288000, CS40L26_PLL_CLK_CFG_12288000},
};

static int cs40l26_get_clk_config(u32 freq, u8 *clk_cfg)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs40l26_pll_sysclk); i++) {
		if (cs40l26_pll_sysclk[i].freq == freq) {
			*clk_cfg = cs40l26_pll_sysclk[i].clk_cfg;
			return 0;
		}
	}

	return -EINVAL;
}

static int cs40l26_swap_ext_clk(struct cs40l26_codec *codec, u8 clk_src)
{
	struct regmap *regmap = codec->regmap;
	struct device *dev = codec->dev;
	u8 clk_cfg, clk_sel;
	int ret;

	switch (clk_src) {
	case CS40L26_PLL_REFCLK_BCLK:
		clk_sel = CS40L26_PLL_CLK_SEL_BCLK;

		ret = cs40l26_get_clk_config(codec->sysclk_rate, &clk_cfg);
		break;
	case CS40L26_PLL_REFCLK_MCLK:
		clk_sel = CS40L26_PLL_CLK_SEL_MCLK;

		ret = cs40l26_get_clk_config(CS40L26_PLL_CLK_FRQ_32768,
				&clk_cfg);
		break;
	case CS40L26_PLL_REFCLK_FSYNC:
		ret = -EPERM;
		break;
	default:
		ret = -EINVAL;
	}

	if (ret) {
		dev_err(dev, "Failed to get clock configuration\n");
		return ret;
	}

	ret = cs40l26_set_pll_loop(codec->core,
			CS40L26_PLL_REFCLK_SET_OPEN_LOOP);
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, CS40L26_REFCLK_INPUT,
			CS40L26_PLL_REFCLK_FREQ_MASK |
			CS40L26_PLL_REFCLK_SEL_MASK, (clk_cfg <<
			CS40L26_PLL_REFCLK_FREQ_SHIFT) | clk_sel);
	if (ret) {
		dev_err(dev, "Failed to update REFCLK input\n");
		return ret;
	}

	ret = cs40l26_set_pll_loop(codec->core,
			CS40L26_PLL_REFCLK_SET_CLOSED_LOOP);

	return ret;
}

static int cs40l26_clk_en(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_dapm_to_component(w->dapm));
	struct cs40l26_private *cs40l26 = codec->core;
	struct device *dev = cs40l26->dev;
	int ret;

	dev_info(dev, "%s: %s\n", __func__,
			event == SND_SOC_DAPM_POST_PMU ? "PMU" : "PMD");

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mutex_lock(&cs40l26->lock);
		cs40l26_vibe_state_update(cs40l26,
					CS40L26_VIBE_STATE_EVENT_ASP_START);
		ret = cs40l26_asp_start(cs40l26);
		mutex_unlock(&cs40l26->lock);
		if (ret)
			return ret;

		if (!completion_done(&cs40l26->i2s_cont)) {
			if (!wait_for_completion_timeout(&cs40l26->i2s_cont,
				msecs_to_jiffies(CS40L26_ASP_START_TIMEOUT)))
				dev_warn(codec->dev,
					"SVC calibration not complete\n");
		}

		ret = cs40l26_swap_ext_clk(codec, CS40L26_PLL_REFCLK_BCLK);
		if (ret)
			return ret;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		ret = cs40l26_swap_ext_clk(codec, CS40L26_PLL_REFCLK_MCLK);
		if (ret)
			return ret;

		mutex_lock(&cs40l26->lock);
		cs40l26_vibe_state_update(cs40l26,
					CS40L26_VIBE_STATE_EVENT_ASP_STOP);
		mutex_unlock(&cs40l26->lock);

		break;
	default:
		dev_err(dev, "Invalid event: %d\n", event);
		return -EINVAL;
	}

	return 0;
}

static int cs40l26_a2h_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_dapm_to_component(w->dapm));
	struct cs40l26_private *cs40l26 = codec->core;
	struct device *dev = cs40l26->dev;
	const struct firmware *fw;
	int ret;
	u32 reg;

	dev_dbg(dev, "%s: %s\n", __func__,
			event == SND_SOC_DAPM_POST_PMU ? "PMU" : "PMD");

	if (codec->bypass_dsp) {
		dev_err(dev, "Cannot apply A2H if DSP is bypassed\n");
		return -EPERM;
	}

	ret = cl_dsp_get_reg(cs40l26->dsp, "A2HEN", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_A2H_ALGO_ID, &reg);
	if (ret)
		return ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		if (codec->tuning != codec->tuning_prev) {
			ret = request_firmware(&fw, codec->bin_file, dev);
			if (ret) {
				dev_err(codec->dev, "Failed to request %s\n",
						codec->bin_file);
				return ret;
			}

			ret = cl_dsp_coeff_file_parse(cs40l26->dsp, fw);
			release_firmware(fw);
			if (ret) {
				dev_warn(dev,
					"Failed to load %s, %d. Continuing...",
					codec->bin_file, ret);
				return ret;
			}

			dev_info(dev, "%s Loaded Successfully\n",
							codec->bin_file);

			codec->tuning_prev = codec->tuning;

			ret = cs40l26_ack_write(cs40l26,
					CS40L26_DSP_VIRTUAL1_MBOX_1,
					CS40L26_DSP_MBOX_CMD_A2H_REINIT,
					CS40L26_DSP_MBOX_RESET);
			if (ret)
				return ret;
		}
		return regmap_write(cs40l26->regmap, reg, 1);
	case SND_SOC_DAPM_PRE_PMD:
		return regmap_write(cs40l26->regmap, reg, 0);
	default:
		dev_err(dev, "Invalid A2H event: %d\n", event);
		return -EINVAL;
	}
}

static int cs40l26_pcm_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_dapm_to_component(w->dapm));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	u32 asp_en_mask = CS40L26_ASP_TX1_EN_MASK | CS40L26_ASP_TX2_EN_MASK |
			CS40L26_ASP_RX1_EN_MASK | CS40L26_ASP_RX2_EN_MASK;
	u32 asp_enables;
	u8 data_src;
	int ret;

	dev_info(dev, "%s: %s\n", __func__,
			event == SND_SOC_DAPM_POST_PMU ? "PMU" : "PMD");

	mutex_lock(&cs40l26->lock);

	data_src = codec->bypass_dsp ? CS40L26_DATA_SRC_ASPRX1 :
			CS40L26_DATA_SRC_DSP1TX1;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = regmap_update_bits(regmap, CS40L26_DACPCM1_INPUT,
			CS40L26_DATA_SRC_MASK, data_src);
		if (ret) {
			dev_err(dev, "Failed to set DAC PCM input\n");
			goto err_mutex;
		}

		ret = regmap_update_bits(regmap, CS40L26_ASPTX1_INPUT,
			CS40L26_DATA_SRC_MASK, data_src);
		if (ret) {
			dev_err(dev, "Failed to set ASPTX1 input\n");
			goto err_mutex;
		}

		asp_enables = 1 | (1 << CS40L26_ASP_TX2_EN_SHIFT)
				| (1 << CS40L26_ASP_RX1_EN_SHIFT)
				| (1 << CS40L26_ASP_RX2_EN_SHIFT);

		ret = regmap_update_bits(regmap, CS40L26_ASP_ENABLES1,
				asp_en_mask, asp_enables);
		if (ret) {
			dev_err(dev, "Failed to enable ASP channels\n");
			goto err_mutex;
		}

		break;
	case SND_SOC_DAPM_PRE_PMD:
		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				CS40L26_DSP_MBOX_CMD_STOP_I2S,
				CS40L26_DSP_MBOX_RESET);
		if (ret)
			goto err_mutex;

		ret = regmap_update_bits(regmap, CS40L26_ASP_ENABLES1,
				asp_en_mask, 0);
		if (ret) {
			dev_err(dev, "Failed to clear ASPTX1 input\n");
			goto err_mutex;
		}

		ret = regmap_update_bits(regmap, CS40L26_ASPTX1_INPUT,
			CS40L26_DATA_SRC_MASK, CS40L26_DATA_SRC_VMON);
		if (ret)
			dev_err(dev, "Failed to set ASPTX1 input\n");
		break;
	default:
		dev_err(dev, "Invalid PCM event: %d\n", event);
		ret = -EINVAL;
	}

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return ret;
}

static int cs40l26_i2s_vmon_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	int ret;
	u32 val;

	ret = cs40l26_pm_enter(cs40l26->dev);
	if (ret)
		return ret;

	ret = regmap_read(cs40l26->regmap, CS40L26_SPKMON_VMON_DEC_OUT_DATA,
			&val);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get VMON Data for I2S\n");
		goto pm_err;
	}

	if (val & CS40L26_VMON_OVFL_FLAG_MASK) {
		dev_err(cs40l26->dev, "I2S VMON overflow detected\n");
		ret = -EOVERFLOW;
		goto pm_err;
	}

	ucontrol->value.enumerated.item[0] = val &
			CS40L26_VMON_DEC_OUT_DATA_MASK;

pm_err:
	cs40l26_pm_exit(cs40l26->dev);

	return ret;
}

static int cs40l26_bypass_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;

	mutex_lock(&cs40l26->lock);

	if (codec->bypass_dsp)
		ucontrol->value.enumerated.item[0] = 1;
	else
		ucontrol->value.enumerated.item[0] = 0;

	mutex_unlock(&cs40l26->lock);

	return 0;
}

static int cs40l26_bypass_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;

	mutex_lock(&cs40l26->lock);

	if (ucontrol->value.enumerated.item[0])
		codec->bypass_dsp = true;
	else
		codec->bypass_dsp = false;

	mutex_unlock(&cs40l26->lock);

	return 0;
}

static int cs40l26_svc_for_streaming_data_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val = 0, reg;
	int ret = 0;

	ret = cl_dsp_get_reg(cs40l26->dsp, "FLAGS",
		CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return ret;

	ret = regmap_read(regmap, reg, &val);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to read FLAGS\n");
		goto pm_err;
	}

	if (val & CS40L26_SVC_FOR_STREAMING_MASK)
		ucontrol->value.enumerated.item[0] = 1;
	else
		ucontrol->value.enumerated.item[0] = 0;

pm_err:
	cs40l26_pm_exit(dev);

	return ret;
}

static int cs40l26_svc_for_streaming_data_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
	snd_soc_component_get_dapm(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret = 0;
	unsigned int reg;

	ret = cl_dsp_get_reg(cs40l26->dsp, "FLAGS",
		CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return ret;

	snd_soc_dapm_mutex_lock(dapm);

	ret = regmap_update_bits(regmap, reg,
			CS40L26_SVC_FOR_STREAMING_MASK,
			ucontrol->value.enumerated.item[0]);
	if (ret)
		dev_err(cs40l26->dev, "Failed to specify SVC for streaming\n");

	snd_soc_dapm_mutex_unlock(dapm);

	cs40l26_pm_exit(dev);

	return ret;
}

static int cs40l26_invert_streaming_data_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val = 0, reg;
	int ret = 0;

	ret = cl_dsp_get_reg(cs40l26->dsp, "SOURCE_INVERT",
		CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return ret;

	ret = regmap_read(regmap, reg, &val);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to read SOURCE_INVERT\n");
		goto pm_err;
	}

	if (val)
		ucontrol->value.enumerated.item[0] = 1;
	else
		ucontrol->value.enumerated.item[0] = 0;

pm_err:
	cs40l26_pm_exit(dev);

	return ret;
}

static int cs40l26_invert_streaming_data_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
	snd_soc_component_get_dapm(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret = 0;
	unsigned int reg;

	ret = cl_dsp_get_reg(cs40l26->dsp, "SOURCE_INVERT",
		CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return ret;

	snd_soc_dapm_mutex_lock(dapm);

	ret = regmap_write(regmap, reg, ucontrol->value.enumerated.item[0]);
	if (ret)
		dev_err(cs40l26->dev, "Failed to specify invert streaming data\n");

	snd_soc_dapm_mutex_unlock(dapm);

	cs40l26_pm_exit(dev);

	return ret;
}

static int cs40l26_tuning_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));

	ucontrol->value.enumerated.item[0] = codec->tuning;

	return 0;
}

static int cs40l26_tuning_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;

	if (ucontrol->value.enumerated.item[0] == codec->tuning)
		return 0;

	if (cs40l26->asp_enable)
		return -EBUSY;

	codec->tuning = ucontrol->value.enumerated.item[0];

	memset(codec->bin_file, 0, PAGE_SIZE);
	codec->bin_file[PAGE_SIZE - 1] = '\0';

	if (codec->tuning > 0)
		snprintf(codec->bin_file, PAGE_SIZE, "cs40l26-a2h%d.bin",
				codec->tuning);
	else
		snprintf(codec->bin_file, PAGE_SIZE, "cs40l26-a2h.bin");

	return 0;
}

static int cs40l26_a2h_volume_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val = 0, reg;
	int ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "VOLUMELEVEL",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_A2H_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return ret;

	ret = regmap_read(regmap, reg, &val);
	if (ret) {
		dev_err(dev, "Failed to get VOLUMELEVEL\n");
		goto pm_err;
	}

	ucontrol->value.integer.value[0] = val;

pm_err:
	cs40l26_pm_exit(dev);

	return ret;
}

static int cs40l26_a2h_volume_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
	snd_soc_component_get_dapm(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val = 0, reg;
	int ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "VOLUMELEVEL",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_A2H_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return ret;

	snd_soc_dapm_mutex_lock(dapm);

	if (ucontrol->value.integer.value[0] > CS40L26_A2H_VOLUME_MAX)
		val = CS40L26_A2H_VOLUME_MAX;
	else if (ucontrol->value.integer.value[0] < 0)
		val = 0;
	else
		val = ucontrol->value.integer.value[0];

	ret = regmap_write(regmap, reg, val);
	if (ret)
		dev_err(dev, "Failed to set VOLUMELEVEL\n");

	snd_soc_dapm_mutex_unlock(dapm);

	cs40l26_pm_exit(dev);

	return ret;
}

static int cs40l26_slots_get(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(
		snd_soc_kcontrol_component(kcontrol));

	ucontrol->value.integer.value[0] = codec->tdm_slot[0];
	ucontrol->value.integer.value[1] = codec->tdm_slot[1];

	return 0;
}

static int cs40l26_slots_put(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(
		snd_soc_kcontrol_component(kcontrol));

	codec->tdm_slot[0] = ucontrol->value.integer.value[0];
	codec->tdm_slot[1] = ucontrol->value.integer.value[1];

	return 0;
}

static int cs40l26_a2h_delay_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val = 0, reg;
	int ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "LRADELAYSAMPS",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_A2H_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return ret;

	ret = regmap_read(regmap, reg, &val);
	if (ret) {
		dev_err(dev, "Failed to get LRADELAYSAMPS\n");
		goto err;
	}

	ucontrol->value.integer.value[0] = val;

err:
	cs40l26_pm_exit(dev);

	return ret;
}

static int cs40l26_a2h_delay_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
	snd_soc_component_get_dapm(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_codec *codec =
	snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val = 0, reg;
	int ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "LRADELAYSAMPS",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_A2H_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return ret;

	snd_soc_dapm_mutex_lock(dapm);

	if (ucontrol->value.integer.value[0] > CS40L26_A2H_DELAY_MAX)
		val = CS40L26_A2H_DELAY_MAX;
	else if (ucontrol->value.integer.value[0] < 0)
		val = 0;
	else
		val = ucontrol->value.integer.value[0];

	ret = regmap_write(regmap, reg, val);
	if (ret)
		dev_err(dev, "Failed to set LRADELAYSAMPS\n");

	snd_soc_dapm_mutex_unlock(dapm);

	cs40l26_pm_exit(dev);

	return ret;
}

static const struct snd_kcontrol_new cs40l26_controls[] = {
	SOC_SINGLE_EXT("A2H Tuning", 0, 0, CS40L26_A2H_MAX_TUNINGS, 0,
			cs40l26_tuning_get, cs40l26_tuning_put),
	SOC_SINGLE_EXT("A2H Volume", 0, 0, CS40L26_A2H_VOLUME_MAX, 0,
			cs40l26_a2h_volume_get, cs40l26_a2h_volume_put),
	SOC_SINGLE_EXT("SVC for streaming data", 0, 0, 1, 0,
			cs40l26_svc_for_streaming_data_get,
			cs40l26_svc_for_streaming_data_put),
	SOC_SINGLE_EXT("Invert streaming data", 0, 0, 1, 0,
			cs40l26_invert_streaming_data_get,
			cs40l26_invert_streaming_data_put),
	SOC_SINGLE_EXT("I2S VMON", 0, 0, CS40L26_VMON_DEC_OUT_DATA_MAX, 0,
			cs40l26_i2s_vmon_get, NULL),
	SOC_SINGLE_EXT("DSP Bypass", 0, 0, 1, 0, cs40l26_bypass_get,
			cs40l26_bypass_put),
	SOC_SINGLE_EXT("A2H Delay", 0, 0, CS40L26_A2H_DELAY_MAX, 0,
			cs40l26_a2h_delay_get, cs40l26_a2h_delay_put),
	SOC_DOUBLE_EXT("RX Slots", 0, 0, 1, 63, 0, cs40l26_slots_get,
			cs40l26_slots_put),
};

static const char * const cs40l26_out_mux_texts[] = { "Off", "PCM", "A2H" };
static SOC_ENUM_SINGLE_VIRT_DECL(cs40l26_out_mux_enum, cs40l26_out_mux_texts);
static const struct snd_kcontrol_new cs40l26_out_mux =
	SOC_DAPM_ENUM("Haptics Source", cs40l26_out_mux_enum);

static const struct snd_soc_dapm_widget
		cs40l26_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY_S("ASP PLL", 0, SND_SOC_NOPM, 0, 0, cs40l26_clk_en,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_AIF_IN("ASPRX1", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASPRX2", NULL, 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_PGA_E("PCM", SND_SOC_NOPM, 0, 0, NULL, 0, cs40l26_pcm_ev,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER_E("A2H", SND_SOC_NOPM, 0, 0, NULL, 0, cs40l26_a2h_ev,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_MUX("Haptics Source", SND_SOC_NOPM, 0, 0,
			&cs40l26_out_mux),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route
		cs40l26_dapm_routes[] = {
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

static int cs40l26_component_set_sysclk(struct snd_soc_component *component,
		int clk_id, int source, unsigned int freq, int dir)
{
	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(component);
	struct device *dev = codec->dev;
	u8 clk_cfg;
	int ret;

	ret = cs40l26_get_clk_config((u32) (CS40L26_PLL_CLK_FREQ_MASK & freq),
			&clk_cfg);
	if (ret) {
		dev_err(dev, "Invalid Clock Frequency: %u Hz\n", freq);
		return ret;
	}

	if (clk_id != 0) {
		dev_err(dev, "Invalid Input Clock (ID: %d)\n", clk_id);
		return -EINVAL;
	}

	codec->sysclk_rate = (u32) (CS40L26_PLL_CLK_FREQ_MASK & freq);

	return 0;
}

static int cs40l26_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(codec_dai->component);

	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
		dev_err(codec->dev, "Device can not be master\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		codec->daifmt = 0;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		codec->daifmt = CS40L26_ASP_FSYNC_INV_MASK;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		codec->daifmt = CS40L26_ASP_BCLK_INV_MASK;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		codec->daifmt = CS40L26_ASP_FSYNC_INV_MASK |
				CS40L26_ASP_BCLK_INV_MASK;
		break;
	default:
		dev_err(codec->dev, "Invalid DAI clock INV\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		codec->daifmt |= ((CS40L26_ASP_FMT_TDM1_DSPA <<
				CS40L26_ASP_FMT_SHIFT) & CS40L26_ASP_FMT_MASK);
		break;
	case SND_SOC_DAIFMT_I2S:
		codec->daifmt |= ((CS40L26_ASP_FMT_I2S <<
				CS40L26_ASP_FMT_SHIFT) & CS40L26_ASP_FMT_MASK);
		break;
	default:
		dev_err(codec->dev, "Invalid DAI format: 0x%X\n",
				fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	return 0;
}

static int cs40l26_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(dai->component);
	u8 asp_rx_wl, asp_rx_width, global_fs;
	int ret, lrck;

	ret = cs40l26_pm_enter(codec->dev);
	if (ret)
		return ret;

	lrck = params_rate(params);
	switch (lrck) {
	case 48000:
		global_fs = CS40L26_GLOBAL_FS_48K;
		break;
	case 96000:
		global_fs = CS40L26_GLOBAL_FS_96K;
		break;
	default:
		ret = -EINVAL;
		dev_err(codec->dev, "Invalid sample rate: %d Hz\n", lrck);
		goto err_pm;
	}

	ret = regmap_update_bits(codec->regmap, CS40L26_GLOBAL_SAMPLE_RATE,
			CS40L26_GLOBAL_FS_MASK, global_fs);
	if (ret) {
		dev_err(codec->dev, "Failed to write global fs\n");
		goto err_pm;
	}

	asp_rx_wl = (u8) (params_width(params) & 0xFF);
	ret = regmap_update_bits(codec->regmap, CS40L26_ASP_DATA_CONTROL5,
			CS40L26_ASP_RX_WL_MASK, asp_rx_wl);
	if (ret) {
		dev_err(codec->dev, "Failed to update ASP RX WL\n");
		goto err_pm;
	}

	if (!codec->tdm_width)
		asp_rx_width = asp_rx_wl;
	else
		asp_rx_width = (u8) (codec->tdm_width & 0xFF);

	codec->daifmt |= ((asp_rx_width << CS40L26_ASP_RX_WIDTH_SHIFT) &
			CS40L26_ASP_RX_WIDTH_MASK);

	ret = regmap_update_bits(codec->regmap, CS40L26_ASP_CONTROL2,
			CS40L26_ASP_FSYNC_INV_MASK | CS40L26_ASP_BCLK_INV_MASK |
			CS40L26_ASP_FMT_MASK | CS40L26_ASP_RX_WIDTH_MASK,
			codec->daifmt);
	if (ret) {
		dev_err(codec->dev, "Failed to update ASP RX width\n");
		goto err_pm;
	}

	ret = regmap_update_bits(codec->regmap, CS40L26_ASP_FRAME_CONTROL5,
			CS40L26_ASP_RX1_SLOT_MASK | CS40L26_ASP_RX2_SLOT_MASK,
			codec->tdm_slot[0] | (codec->tdm_slot[1] <<
			CS40L26_ASP_RX2_SLOT_SHIFT));
	if (ret) {
		dev_err(codec->dev, "Failed to update ASP slot number\n");
		goto err_pm;
	}

	dev_dbg(codec->dev, "ASP: %d bits in %d bit slots, slot #s: %d, %d\n",
			asp_rx_wl, asp_rx_width, codec->tdm_slot[0],
			codec->tdm_slot[1]);

err_pm:
	cs40l26_pm_exit(codec->dev);

	return ret;
}

static const struct snd_soc_dai_ops cs40l26_dai_ops = {
	.set_fmt = cs40l26_set_dai_fmt,
	.hw_params = cs40l26_pcm_hw_params,
};

static struct snd_soc_dai_driver cs40l26_dai[] = {
	{
		.name = "cs40l26-pcm",
		.id = 0,
		.playback = {
			.stream_name = "ASP Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = CS40L26_RATES,
			.formats = CS40L26_FORMATS,
		},
		.ops = &cs40l26_dai_ops,
		.symmetric_rates = 1,
	},
};

static int cs40l26_codec_probe(struct snd_soc_component *component)
{
	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(component);

	codec->bin_file = devm_kzalloc(codec->dev, PAGE_SIZE, GFP_KERNEL);
	if (!codec->bin_file)
		return -ENOMEM;

	codec->bin_file[PAGE_SIZE - 1] = '\0';
	snprintf(codec->bin_file, PAGE_SIZE, CS40L26_A2H_TUNING_FILE_NAME);

	/* Default audio SCLK frequency */
	codec->sysclk_rate = CS40L26_PLL_CLK_FRQ_1536000;

	codec->tdm_slot[0] = 2;
	codec->tdm_slot[1] = 3;

	return 0;
}

static const struct snd_soc_component_driver soc_codec_dev_cs40l26 = {
	.probe = cs40l26_codec_probe,
	.set_sysclk = cs40l26_component_set_sysclk,

	.dapm_widgets = cs40l26_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs40l26_dapm_widgets),
	.dapm_routes = cs40l26_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(cs40l26_dapm_routes),
	.controls = cs40l26_controls,
	.num_controls = ARRAY_SIZE(cs40l26_controls),
};

static int cs40l26_codec_driver_probe(struct platform_device *pdev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(pdev->dev.parent);
	struct cs40l26_codec *codec;
	int ret;

	codec = devm_kzalloc(&pdev->dev, sizeof(struct cs40l26_codec),
			GFP_KERNEL);
	if (!codec)
		return -ENOMEM;

	codec->core = cs40l26;
	codec->regmap = cs40l26->regmap;
	codec->dev = &pdev->dev;

	platform_set_drvdata(pdev, codec);

	pm_runtime_enable(&pdev->dev);

	ret = snd_soc_register_component(&pdev->dev, &soc_codec_dev_cs40l26,
			cs40l26_dai, ARRAY_SIZE(cs40l26_dai));
	if (ret < 0)
		dev_err(&pdev->dev, "Failed to register codec: %d\n", ret);

	return ret;
}

static int cs40l26_codec_driver_remove(struct platform_device *pdev)
{
	struct cs40l26_codec *codec = dev_get_drvdata(&pdev->dev);

	pm_runtime_disable(codec->dev);

	snd_soc_unregister_component(codec->dev);

	return 0;
}

static struct platform_driver cs40l26_codec_driver = {
	.driver = {
		.name = "cs40l26-codec",
	},
	.probe = cs40l26_codec_driver_probe,
	.remove = cs40l26_codec_driver_remove,
};
module_platform_driver(cs40l26_codec_driver);

MODULE_DESCRIPTION("ASoC CS40L26 driver");
MODULE_AUTHOR("Fred Treven <fred.treven@cirrus.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cs40l26-codec");
