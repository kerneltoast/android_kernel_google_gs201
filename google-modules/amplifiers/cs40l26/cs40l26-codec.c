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
	int error;

	switch (clk_src) {
	case CS40L26_PLL_REFCLK_BCLK:
		clk_sel = CS40L26_PLL_CLK_SEL_BCLK;
		error = cs40l26_get_clk_config(codec->sysclk_rate, &clk_cfg);
		break;
	case CS40L26_PLL_REFCLK_MCLK:
		clk_sel = CS40L26_PLL_CLK_SEL_MCLK;
		error = cs40l26_get_clk_config(CS40L26_PLL_CLK_FRQ_32768, &clk_cfg);
		break;
	case CS40L26_PLL_REFCLK_FSYNC:
		error = -EPERM;
		break;
	default:
		error = -EINVAL;
	}

	if (error) {
		dev_err(dev, "Failed to get clock configuration\n");
		return error;
	}

	error = cs40l26_set_pll_loop(codec->core, CS40L26_PLL_REFCLK_SET_OPEN_LOOP);
	if (error)
		return error;

	error = regmap_update_bits(regmap, CS40L26_REFCLK_INPUT, CS40L26_PLL_REFCLK_FREQ_MASK |
			CS40L26_PLL_REFCLK_SEL_MASK, (clk_cfg << CS40L26_PLL_REFCLK_FREQ_SHIFT) |
			clk_sel);
	if (error) {
		dev_err(dev, "Failed to update REFCLK input\n");
		return error;
	}

	return cs40l26_set_pll_loop(codec->core, CS40L26_PLL_REFCLK_SET_CLOSED_LOOP);
}

static int cs40l26_clk_en(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_dapm_to_component(w->dapm));
	struct cs40l26_private *cs40l26 = codec->core;
	struct device *dev = cs40l26->dev;
	int error;

	dev_info(dev, "%s: %s\n", __func__, event == SND_SOC_DAPM_POST_PMU ? "PMU" : "PMD");

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mutex_lock(&cs40l26->lock);
		cs40l26_vibe_state_update(cs40l26, CS40L26_VIBE_STATE_EVENT_ASP_START);
		error = cs40l26_asp_start(cs40l26);
		mutex_unlock(&cs40l26->lock);
		if (error)
			return error;

		if (!completion_done(&cs40l26->i2s_cont)) {
			if (!wait_for_completion_timeout(&cs40l26->i2s_cont,
					msecs_to_jiffies(CS40L26_ASP_START_TIMEOUT)))
				dev_warn(codec->dev, "SVC calibration not complete\n");
		}

		error = cs40l26_swap_ext_clk(codec, CS40L26_PLL_REFCLK_BCLK);
		if (error)
			return error;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		error = cs40l26_swap_ext_clk(codec, CS40L26_PLL_REFCLK_MCLK);
		if (error)
			return error;

		mutex_lock(&cs40l26->lock);
		cs40l26_vibe_state_update(cs40l26, CS40L26_VIBE_STATE_EVENT_ASP_STOP);
		mutex_unlock(&cs40l26->lock);

		break;
	default:
		dev_err(dev, "Invalid event: %d\n", event);
		return -EINVAL;
	}

	return 0;
}

static int cs40l26_dsp_tx(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event)
{	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_dapm_to_component(w->dapm));
	struct cs40l26_private *cs40l26 = codec->core;
	struct device *dev = cs40l26->dev;
	const struct firmware *fw;
	int error;
	u32 reg;

	dev_info(dev, "%s: %s\n", __func__, event == SND_SOC_DAPM_POST_PMU ? "PMU" : "PMD");

	if (codec->dsp_bypass) {
		dev_err(dev, "Cannot use A2H while bypassing DSP\n");
		return -EPERM;
	}

	error = cl_dsp_get_reg(cs40l26->dsp, "A2HEN", CL_DSP_XM_UNPACKED_TYPE, CS40L26_A2H_ALGO_ID,
			&reg);
	if (error)
		return error;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		if (codec->tuning != codec->tuning_prev) {
			error = request_firmware(&fw, codec->bin_file, dev);
			if (error) {
				dev_err(codec->dev, "Failed to request %s\n", codec->bin_file);
				return error;
			}

			error = cl_dsp_coeff_file_parse(cs40l26->dsp, fw);
			release_firmware(fw);
			if (error) {
				dev_warn(dev, "Failed to load %s, %d. Continuing...",
						codec->bin_file, error);
				return error;
			}

			dev_info(dev, "%s Loaded Successfully\n", codec->bin_file);

			codec->tuning_prev = codec->tuning;

			error = cs40l26_mailbox_write(cs40l26, CS40L26_DSP_MBOX_CMD_A2H_REINIT);
			if (error)
				return error;
		}
		return regmap_write(cs40l26->regmap, reg, 1);
	case SND_SOC_DAPM_PRE_PMD:
		return regmap_write(cs40l26->regmap, reg, 0);
	default:
		dev_err(dev, "Invalid A2H event: %d\n", event);
		return -EINVAL;
	}
}

static int cs40l26_asp_rx(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event)
{	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_dapm_to_component(w->dapm));
	bool is_revid_b2 = (codec->core->revid == (CS40L26_REVID_B2)) ? true : false;
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	u32 asp_en_mask = CS40L26_ASP_TX1_EN_MASK | CS40L26_ASP_TX2_EN_MASK |
			CS40L26_ASP_RX1_EN_MASK | CS40L26_ASP_RX2_EN_MASK;
	u32 flags = 0, reg = 0;
	u32 asp_enables;
	u8 data_src;
	int error;

	dev_info(dev, "%s: %s\n", __func__, event == SND_SOC_DAPM_POST_PMU ? "PMU" : "PMD");

	mutex_lock(&cs40l26->lock);

	data_src = codec->dsp_bypass ? CS40L26_DATA_SRC_ASPRX1 : CS40L26_DATA_SRC_DSP1TX1;

	if (is_revid_b2) {
		error = cl_dsp_get_reg(cs40l26->dsp, "FLAGS", CL_DSP_XM_UNPACKED_TYPE,
				cs40l26->fw_id, &reg);
		if (error)
			goto err_mutex;

		error = regmap_read(regmap, reg, &flags);
		if (error)
			goto err_mutex;
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		error = regmap_update_bits(regmap, CS40L26_DACPCM1_INPUT,
				CS40L26_DATA_SRC_MASK, data_src);
		if (error) {
			dev_err(dev, "Failed to set DAC PCM input\n");
			goto err_mutex;
		}

		error = regmap_update_bits(regmap, CS40L26_ASPTX1_INPUT, CS40L26_DATA_SRC_MASK,
				data_src);
		if (error) {
			dev_err(dev, "Failed to set ASPTX1 input\n");
			goto err_mutex;
		}

		asp_enables = 1 | (1 << CS40L26_ASP_TX2_EN_SHIFT) | (1 << CS40L26_ASP_RX1_EN_SHIFT)
				| (1 << CS40L26_ASP_RX2_EN_SHIFT);

		error = regmap_update_bits(regmap, CS40L26_ASP_ENABLES1, asp_en_mask, asp_enables);
		if (error) {
			dev_err(dev, "Failed to enable ASP channels\n");
			goto err_mutex;
		}

		/* Force open-loop if closed-loop not set */
		if (!(flags & CS40L26_FLAGS_I2S_SVC_EN_MASK) && is_revid_b2) {
			codec->svc_ol_forced = true;
			error = regmap_set_bits(regmap, reg, CS40L26_FLAGS_I2S_SVC_EN_MASK |
					CS40L26_FLAGS_I2S_SVC_LOOP_MASK);
			if (error)
				goto err_mutex;
		} else {
			codec->svc_ol_forced = false;
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		error = cs40l26_mailbox_write(cs40l26, CS40L26_DSP_MBOX_CMD_STOP_I2S);
		if (error)
			goto err_mutex;

		if (codec->svc_ol_forced) {
			error = regmap_clear_bits(regmap, reg, CS40L26_FLAGS_I2S_SVC_EN_MASK |
					CS40L26_FLAGS_I2S_SVC_LOOP_MASK);
			if (error)
				goto err_mutex;
		}

		error = regmap_update_bits(regmap, CS40L26_ASP_ENABLES1, asp_en_mask, 0);
		if (error) {
			dev_err(dev, "Failed to clear ASPTX1 input\n");
			goto err_mutex;
		}

		error = regmap_update_bits(regmap, CS40L26_ASPTX1_INPUT, CS40L26_DATA_SRC_MASK,
				CS40L26_DATA_SRC_VMON);
		if (error)
			dev_err(dev, "Failed to set ASPTX1 input\n");
		break;
	default:
		dev_err(dev, "Invalid PCM event: %d\n", event);
		error = -EINVAL;
	}

err_mutex:
	mutex_unlock(&cs40l26->lock);

	return error;
}

static int cs40l26_i2s_vmon_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	int error;
	u32 val;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = regmap_read(cs40l26->regmap, CS40L26_SPKMON_VMON_DEC_OUT_DATA, &val);
	if (error) {
		dev_err(cs40l26->dev, "Failed to get VMON Data for I2S\n");
		goto pm_err;
	}

	if (val & CS40L26_VMON_OVFL_FLAG_MASK) {
		dev_err(cs40l26->dev, "I2S VMON overflow detected\n");
		error = -EOVERFLOW;
		goto pm_err;
	}

	ucontrol->value.enumerated.item[0] = val & CS40L26_VMON_DEC_OUT_DATA_MASK;

pm_err:
	cs40l26_pm_exit(cs40l26->dev);

	return error;
}

static int cs40l26_dsp_bypass_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;

	mutex_lock(&cs40l26->lock);

	if (codec->dsp_bypass)
		ucontrol->value.enumerated.item[0] = 1;
	else
		ucontrol->value.enumerated.item[0] = 0;

	mutex_unlock(&cs40l26->lock);

	return 0;
}

static int cs40l26_dsp_bypass_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;

	mutex_lock(&cs40l26->lock);

	if (ucontrol->value.enumerated.item[0])
		codec->dsp_bypass = true;
	else
		codec->dsp_bypass = false;

	mutex_unlock(&cs40l26->lock);

	return 0;
}

static int cs40l26_svc_en_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	unsigned int algo_id, val = 0, reg;
	struct device *dev = cs40l26->dev;
	int error;

	error = cs40l26_pm_enter(dev);
	if (error)
		return error;

	error = cs40l26_get_ram_ext_algo_id(cs40l26, &algo_id);
	if (error)
		goto pm_err;

	error = cl_dsp_get_reg(cs40l26->dsp, "FLAGS", CL_DSP_XM_UNPACKED_TYPE, algo_id,
			&reg);
	if (error)
		goto pm_err;

	error = regmap_read(regmap, reg, &val);
	if (error) {
		dev_err(cs40l26->dev, "Failed to read FLAGS\n");
		goto pm_err;
	}

	if (val & CS40L26_SVC_EN_MASK)
		ucontrol->value.enumerated.item[0] = 1;
	else
		ucontrol->value.enumerated.item[0] = 0;

pm_err:
	cs40l26_pm_exit(dev);

	return error;
}

static int cs40l26_svc_en_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int algo_id, reg;
	int error;

	error = cs40l26_pm_enter(dev);
	if (error)
		return error;

	error = cs40l26_get_ram_ext_algo_id(cs40l26, &algo_id);
	if (error)
		goto pm_err;

	error = cl_dsp_get_reg(cs40l26->dsp, "FLAGS", CL_DSP_XM_UNPACKED_TYPE, algo_id,
			&reg);
	if (error)
		goto pm_err;

	snd_soc_dapm_mutex_lock(dapm);

	error = regmap_update_bits(regmap, reg, CS40L26_SVC_EN_MASK,
			ucontrol->value.enumerated.item[0]);
	if (error)
		dev_err(cs40l26->dev, "Failed to specify SVC for streaming\n");

	snd_soc_dapm_mutex_unlock(dapm);

pm_err:
	cs40l26_pm_exit(dev);

	return error;
}

static int cs40l26_invert_streaming_data_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	unsigned int algo_id, val = 0, reg;
	struct device *dev = cs40l26->dev;
	int error;

	error = cs40l26_pm_enter(dev);
	if (error)
		return error;

	error = cs40l26_get_ram_ext_algo_id(cs40l26, &algo_id);
	if (error)
		goto pm_err;

	error = cl_dsp_get_reg(cs40l26->dsp, "SOURCE_INVERT",
			CL_DSP_XM_UNPACKED_TYPE, algo_id, &reg);
	if (error)
		goto pm_err;

	error = regmap_read(regmap, reg, &val);
	if (error) {
		dev_err(cs40l26->dev, "Failed to read SOURCE_INVERT\n");
		goto pm_err;
	}

	if (val)
		ucontrol->value.enumerated.item[0] = 1;
	else
		ucontrol->value.enumerated.item[0] = 0;

pm_err:
	cs40l26_pm_exit(dev);

	return error;
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
	unsigned int algo_id, reg;
	int error;

	error = cs40l26_pm_enter(dev);
	if (error)
		return error;

	error = cs40l26_get_ram_ext_algo_id(cs40l26, &algo_id);
	if (error)
		goto pm_err;

	error = cl_dsp_get_reg(cs40l26->dsp, "SOURCE_INVERT",
			CL_DSP_XM_UNPACKED_TYPE, algo_id, &reg);
	if (error)
		goto pm_err;

	snd_soc_dapm_mutex_lock(dapm);

	error = regmap_write(regmap, reg, ucontrol->value.enumerated.item[0]);
	if (error)
		dev_err(cs40l26->dev, "Failed to specify invert streaming data\n");

	snd_soc_dapm_mutex_unlock(dapm);

pm_err:
	cs40l26_pm_exit(dev);

	return error;
}

static int cs40l26_tuning_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));

	ucontrol->value.enumerated.item[0] = codec->tuning;

	return 0;
}

static int cs40l26_tuning_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
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
		snprintf(codec->bin_file, PAGE_SIZE, "cs40l26-a2h%d.bin", codec->tuning);
	else
		snprintf(codec->bin_file, PAGE_SIZE, "cs40l26-a2h.bin");

	return 0;
}

static int cs40l26_a2h_level_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val = 0, reg;
	int error;

	error = cl_dsp_get_reg(cs40l26->dsp, "VOLUMELEVEL", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_A2H_ALGO_ID, &reg);
	if (error)
		return error;

	error = cs40l26_pm_enter(dev);
	if (error)
		return error;

	error = regmap_read(regmap, reg, &val);
	if (error) {
		dev_err(dev, "Failed to get VOLUMELEVEL\n");
		goto pm_err;
	}

	ucontrol->value.integer.value[0] = val;

pm_err:
	cs40l26_pm_exit(dev);

	return error;
}

static int cs40l26_a2h_level_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val = 0, reg;
	int error;

	error = cl_dsp_get_reg(cs40l26->dsp, "VOLUMELEVEL", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_A2H_ALGO_ID, &reg);
	if (error)
		return error;

	error = cs40l26_pm_enter(dev);
	if (error)
		return error;

	snd_soc_dapm_mutex_lock(dapm);

	if (ucontrol->value.integer.value[0] > CS40L26_A2H_LEVEL_MAX)
		val = CS40L26_A2H_LEVEL_MAX;
	else if (ucontrol->value.integer.value[0] < CS40L26_A2H_LEVEL_MIN)
		val = CS40L26_A2H_LEVEL_MIN;
	else
		val = ucontrol->value.integer.value[0];

	error = regmap_write(regmap, reg, val);
	if (error)
		dev_err(dev, "Failed to set VOLUMELEVEL\n");

	snd_soc_dapm_mutex_unlock(dapm);

	cs40l26_pm_exit(dev);

	return error;
}

static int cs40l26_i2s_atten_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val = 0, reg;
	int error;

	error = cl_dsp_get_reg(cs40l26->dsp, "I2S_ATTENUATION", CL_DSP_XM_UNPACKED_TYPE,
			cs40l26->fw_id, &reg);
	if (error)
		goto pm_err;

	error = cs40l26_pm_enter(dev);
	if (error)
		return error;

	error = regmap_read(regmap, reg, &val);
	if (error)
		goto pm_err;

	ucontrol->value.integer.value[0] = val;

pm_err:
	cs40l26_pm_exit(dev);

	return error;
}

static int cs40l26_i2s_atten_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(comp);
	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(comp);
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	u32 val = 0, reg;
	int error;

	error = cs40l26_pm_enter(dev);
	if (error)
		return error;

	error = cl_dsp_get_reg(cs40l26->dsp, "I2S_ATTENUATION", CL_DSP_XM_UNPACKED_TYPE,
			cs40l26->fw_id, &reg);
	if (error)
		goto pm_err;

	snd_soc_dapm_mutex_lock(dapm);

	if (ucontrol->value.integer.value[0] > CS40L26_I2S_ATTENUATION_MAX)
		val = CS40L26_I2S_ATTENUATION_MAX;
	else if (ucontrol->value.integer.value[0] < 0)
		val = 0;
	else
		val = ucontrol->value.integer.value[0];

	error = regmap_write(regmap, reg, val);

	snd_soc_dapm_mutex_unlock(dapm);
pm_err:
	cs40l26_pm_exit(dev);

	return error;
}

static int cs40l26_slots_get(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(
		snd_soc_kcontrol_component(kcontrol));

	dev_dbg(codec->dev, "%s: 0: %ld->%d, 1: %ld->%d\n", __func__,
		ucontrol->value.integer.value[0], codec->tdm_slot[0],
		ucontrol->value.integer.value[1], codec->tdm_slot[1]);

	ucontrol->value.integer.value[0] = codec->tdm_slot[0];
	ucontrol->value.integer.value[1] = codec->tdm_slot[1];

	return 0;
}

static int cs40l26_slots_put(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(
		snd_soc_kcontrol_component(kcontrol));

	dev_dbg(codec->dev, "%s: 0: %d->%ld, 1: %d->%ld\n", __func__,
		codec->tdm_slot[0], ucontrol->value.integer.value[0],
		codec->tdm_slot[1], ucontrol->value.integer.value[1]);

	codec->tdm_slot[0] = ucontrol->value.integer.value[0];
	codec->tdm_slot[1] = ucontrol->value.integer.value[1];

	return 0;
}

static int cs40l26_a2h_delay_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val = 0, reg;
	int error;

	error = cl_dsp_get_reg(cs40l26->dsp, "LRADELAYSAMPS",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_A2H_ALGO_ID, &reg);
	if (error)
		return error;

	error = cs40l26_pm_enter(dev);
	if (error)
		return error;

	error = regmap_read(regmap, reg, &val);
	if (error) {
		dev_err(dev, "Failed to get LRADELAYSAMPS\n");
		goto err;
	}

	ucontrol->value.integer.value[0] = val;

err:
	cs40l26_pm_exit(dev);

	return error;
}

static int cs40l26_a2h_delay_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val = 0, reg;
	int error;

	error = cl_dsp_get_reg(cs40l26->dsp, "LRADELAYSAMPS",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_A2H_ALGO_ID, &reg);
	if (error)
		return error;

	error = cs40l26_pm_enter(dev);
	if (error)
		return error;

	snd_soc_dapm_mutex_lock(dapm);

	if (ucontrol->value.integer.value[0] > CS40L26_A2H_DELAY_MAX)
		val = CS40L26_A2H_DELAY_MAX;
	else if (ucontrol->value.integer.value[0] < 0)
		val = 0;
	else
		val = ucontrol->value.integer.value[0];

	error = regmap_write(regmap, reg, val);
	if (error)
		dev_err(dev, "Failed to set LRADELAYSAMPS\n");

	snd_soc_dapm_mutex_unlock(dapm);

	cs40l26_pm_exit(dev);

	return error;
}

static int cs40l26_boost_disable_delay_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(snd_soc_kcontrol_component(kcontrol));
	struct cs40l26_private *cs40l26 = codec->core;
	u32 algo_id, delay, reg;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cs40l26_get_ram_ext_algo_id(cs40l26, &algo_id);
	if (error)
		goto pm_err;

	error = cl_dsp_get_reg(cs40l26->dsp, "BOOST_DISABLE_DELAY", CL_DSP_XM_UNPACKED_TYPE,
			algo_id, &reg);
	if (error)
		goto pm_err;

	error = regmap_read(cs40l26->regmap, reg, &delay);
	if (error)
		goto pm_err;

	ucontrol->value.integer.value[0] = delay;

pm_err:
	cs40l26_pm_exit(cs40l26->dev);

	return error;
}

static int cs40l26_boost_disable_delay_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(comp);
	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(comp);
	struct cs40l26_private *cs40l26 = codec->core;
	u32 algo_id, delay, reg;
	int error;

	error = cs40l26_pm_enter(cs40l26->dev);
	if (error)
		return error;

	error = cs40l26_get_ram_ext_algo_id(cs40l26, &algo_id);
	if (error)
		goto pm_err;

	error = cl_dsp_get_reg(cs40l26->dsp, "BOOST_DISABLE_DELAY", CL_DSP_XM_UNPACKED_TYPE,
			algo_id, &reg);
	if (error)
		goto pm_err;

	snd_soc_dapm_mutex_lock(dapm);

	delay = ucontrol->value.integer.value[0];

	error = regmap_write(cs40l26->regmap, reg, delay);

	snd_soc_dapm_mutex_unlock(dapm);

pm_err:
	cs40l26_pm_exit(cs40l26->dev);

	return error;
}

static const struct snd_kcontrol_new cs40l26_controls[] = {
	SOC_SINGLE_EXT("A2H Tuning", 0, 0, CS40L26_A2H_MAX_TUNINGS, 0, cs40l26_tuning_get,
			cs40l26_tuning_put),
	SOC_SINGLE_EXT("A2H Level", 0, 0, CS40L26_A2H_LEVEL_MAX, 0, cs40l26_a2h_level_get,
			cs40l26_a2h_level_put),
	SOC_SINGLE_EXT("SVC Algo Enable", 0, 0, 1, 0, cs40l26_svc_en_get, cs40l26_svc_en_put),
	SOC_SINGLE_EXT("Invert streaming data", 0, 0, 1, 0, cs40l26_invert_streaming_data_get,
			cs40l26_invert_streaming_data_put),
	SOC_SINGLE_EXT("I2S VMON", 0, 0, CS40L26_VMON_DEC_OUT_DATA_MAX, 0,
			cs40l26_i2s_vmon_get, NULL),
	SOC_SINGLE_EXT("DSP Bypass", 0, 0, 1, 0, cs40l26_dsp_bypass_get, cs40l26_dsp_bypass_put),
	SOC_SINGLE_EXT("A2H Delay", 0, 0, CS40L26_A2H_DELAY_MAX, 0, cs40l26_a2h_delay_get,
			cs40l26_a2h_delay_put),
	SOC_SINGLE_EXT("Boost Disable Delay", 0, 0, CS40L26_BOOST_DISABLE_DELAY_MAX, 0,
			cs40l26_boost_disable_delay_get, cs40l26_boost_disable_delay_put),
	/*
	 * Use kcontrol "RX Slots" inherited from CS40L25 for the in-house Audio HAL alignment.
	 * Disable cs40l26_set_tdm_slot for now.
	 */
	SOC_DOUBLE_EXT("RX Slots", 0, 0, 1, 63, 0, cs40l26_slots_get,
			cs40l26_slots_put),
};

static const struct snd_kcontrol_new cs40l26_b2_controls[] = {
	SOC_SINGLE_EXT("I2S Attenuation", 0, 0, CS40L26_I2S_ATTENUATION_MAX, 0,
			cs40l26_i2s_atten_get, cs40l26_i2s_atten_put),
};

static const char * const cs40l26_out_mux_texts[] = { "Off", "PCM", "A2H" };
static SOC_ENUM_SINGLE_VIRT_DECL(cs40l26_out_mux_enum, cs40l26_out_mux_texts);
static const struct snd_kcontrol_new cs40l26_out_mux =
		SOC_DAPM_ENUM("Haptics Source", cs40l26_out_mux_enum);

static const struct snd_soc_dapm_widget cs40l26_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY_S("ASP PLL", 0, SND_SOC_NOPM, 0, 0, cs40l26_clk_en,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_AIF_IN("ASPRX1", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASPRX2", NULL, 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_PGA_E("PCM", SND_SOC_NOPM, 0, 0, NULL, 0, cs40l26_asp_rx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER_E("A2H", SND_SOC_NOPM, 0, 0, NULL, 0, cs40l26_dsp_tx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_MUX("Haptics Source", SND_SOC_NOPM, 0, 0, &cs40l26_out_mux),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route cs40l26_dapm_routes[] = {
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
	int error;

	error = cs40l26_get_clk_config((u32) (CS40L26_PLL_CLK_FREQ_MASK & freq), &clk_cfg);
	if (error) {
		dev_err(dev, "Invalid Clock Frequency: %u Hz\n", freq);
		return error;
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
		codec->daifmt = CS40L26_ASP_FSYNC_INV_MASK | CS40L26_ASP_BCLK_INV_MASK;
		break;
	default:
		dev_err(codec->dev, "Invalid DAI clock INV\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		codec->daifmt |= ((CS40L26_ASP_FMT_TDM1_DSPA << CS40L26_ASP_FMT_SHIFT) &
				CS40L26_ASP_FMT_MASK);
		break;
	case SND_SOC_DAIFMT_I2S:
		codec->daifmt |= ((CS40L26_ASP_FMT_I2S << CS40L26_ASP_FMT_SHIFT) &
				CS40L26_ASP_FMT_MASK);
		break;
	default:
		dev_err(codec->dev, "Invalid DAI format: 0x%X\n", fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	return 0;
}

static int cs40l26_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(dai->component);
	int error, lrck = params_rate(params);
	u32 asp_rx_wl, asp_rx_width, ultrasonic;

	error = cs40l26_pm_enter(codec->dev);
	if (error)
		return error;

	if (lrck == 48000)
		ultrasonic = 0;
	else if (lrck == 96000)
		ultrasonic = 1;
	else
		error = -EINVAL;

	if (error) {
		dev_err(codec->dev, "Invalid sample rate: %d Hz\n", lrck);
		goto err_pm;
	}

	error = regmap_update_bits(codec->regmap, CS40L26_MONITOR_FILT,
				   CS40L26_VIMON_DUAL_RATE_MASK,
				   FIELD_PREP(CS40L26_VIMON_DUAL_RATE_MASK, ultrasonic));
	if (error)
		goto err_pm;

	asp_rx_wl = (u8) (params_width(params) & 0xFF);
	error = regmap_update_bits(codec->regmap, CS40L26_ASP_DATA_CONTROL5,
			CS40L26_ASP_RX_WL_MASK, asp_rx_wl);
	if (error) {
		dev_err(codec->dev, "Failed to update ASP RX WL\n");
		goto err_pm;
	}

	if (!codec->tdm_width)
		asp_rx_width = asp_rx_wl;
	else
		asp_rx_width = (u8) (codec->tdm_width & 0xFF);

	codec->daifmt |= ((asp_rx_width << CS40L26_ASP_RX_WIDTH_SHIFT) &
			CS40L26_ASP_RX_WIDTH_MASK);

	error = regmap_update_bits(codec->regmap, CS40L26_ASP_CONTROL2,
			CS40L26_ASP_FSYNC_INV_MASK | CS40L26_ASP_BCLK_INV_MASK |
			CS40L26_ASP_FMT_MASK | CS40L26_ASP_RX_WIDTH_MASK, codec->daifmt);
	if (error) {
		dev_err(codec->dev, "Failed to update ASP RX width\n");
		goto err_pm;
	}

	error = regmap_update_bits(codec->regmap, CS40L26_ASP_FRAME_CONTROL5,
			CS40L26_ASP_RX1_SLOT_MASK | CS40L26_ASP_RX2_SLOT_MASK,
			codec->tdm_slot[0] | (codec->tdm_slot[1] << CS40L26_ASP_RX2_SLOT_SHIFT));
	if (error) {
		dev_err(codec->dev, "Failed to update ASP slot number\n");
		goto err_pm;
	}

	dev_dbg(codec->dev, "ASP: %d bits in %d bit slots, slot #s: %d, %d\n",
			asp_rx_wl, asp_rx_width, codec->tdm_slot[0], codec->tdm_slot[1]);

err_pm:
	cs40l26_pm_exit(codec->dev);

	return error;
}

#if 0
/*
 * Use kcontrol "RX Slots" inherited from CS40L25 for the in-house Audio HAL alignment for now.
 */
static int cs40l26_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
		unsigned int rx_mask, int slots, int slot_width)
{
	struct cs40l26_codec *codec =
			snd_soc_component_get_drvdata(dai->component);

	codec->tdm_width = slot_width;
	codec->tdm_slots = slots;

	/* Reset to slots 0,1 if TDM is being disabled, and catch the case
	 * where both RX1 and RX2 would be set to slot 0 since that causes
	 * hardware to flag an error
	 */
	if (!slots || rx_mask == 0x1)
		rx_mask = 0x3;

	codec->tdm_slot[0] = ffs(rx_mask) - 1;
	rx_mask &= ~(1 << codec->tdm_slot[0]);
	codec->tdm_slot[1] = ffs(rx_mask) - 1;

	dev_dbg(codec->dev,
		"%s: tx_mask:0x%X, rx_mask:0x%X, slots:%d, slot_width:%d, "
		"slot#0: %d, slot#1: %d\n", __func__, tx_mask, rx_mask, slots,
		slot_width, codec->tdm_slot[0], codec->tdm_slot[1]);
	return 0;
}
#endif

static const struct snd_soc_dai_ops cs40l26_dai_ops = {
	.set_fmt = cs40l26_set_dai_fmt,
#if 0
	.set_tdm_slot = cs40l26_set_tdm_slot,
#endif
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
		.symmetric_rate = 1,
	},
};

static int cs40l26_codec_probe(struct snd_soc_component *component)
{
	struct cs40l26_codec *codec = snd_soc_component_get_drvdata(component);

	codec->bin_file = devm_kzalloc(codec->dev, PAGE_SIZE, GFP_KERNEL);
	if (!codec->bin_file)
		return -ENOMEM;

	codec->bin_file[PAGE_SIZE - 1] = '\0';
	snprintf(codec->bin_file, PAGE_SIZE, "cs40l26-a2h.bin");

	/* Default audio SCLK frequency */
	codec->sysclk_rate = CS40L26_PLL_CLK_FRQ_1536000;

	codec->tdm_slot[0] = 2;
	codec->tdm_slot[1] = 3;

	if (codec->core->revid == CS40L26_REVID_B2)
		snd_soc_add_component_controls(component, cs40l26_b2_controls,
				ARRAY_SIZE(cs40l26_b2_controls));

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

static const struct snd_kcontrol_new cs40l26_controls_nop[] = {
	SOC_SINGLE("HAPTIC Disabled", SND_SOC_NOPM, 0, 0, 0),
};

static const struct snd_soc_component_driver soc_codec_dev_cs40l26_nop = {
	.controls = cs40l26_controls_nop,
	.num_controls = ARRAY_SIZE(cs40l26_controls_nop),
};

static int cs40l26_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	pr_warn("%s: enter\n", __func__);
	return 0;
}

static const struct snd_soc_dai_ops cs40l26_dai_nop_ops = {
	.startup = cs40l26_startup,
};

static struct snd_soc_dai_driver cs40l26_dai_nop[] = {
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
		.ops = &cs40l26_dai_nop_ops,
		.symmetric_rate = 1,
	},
};

static int cs40l26_codec_register_nop_codec(struct device *dev)
{
	int ret;
	ret = snd_soc_register_component(dev, &soc_codec_dev_cs40l26_nop,
			cs40l26_dai_nop, ARRAY_SIZE(cs40l26_dai_nop));
	if (ret < 0)
		dev_err(dev, "Failed to register codec: %d\n", ret);
	else
		dev_info(dev, "Register nop haptic codec\n");
	return ret;
}

static int cs40l26_codec_driver_probe(struct platform_device *pdev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(pdev->dev.parent);
	struct cs40l26_codec *codec;
	int error;

	if (cs40l26->cs40l26_not_probed) {
		return cs40l26_codec_register_nop_codec(&pdev->dev);
	}

	codec = devm_kzalloc(&pdev->dev, sizeof(struct cs40l26_codec), GFP_KERNEL);
	if (!codec)
		return -ENOMEM;

	codec->core = cs40l26;
	codec->regmap = cs40l26->regmap;
	codec->dev = &pdev->dev;

	platform_set_drvdata(pdev, codec);

	pm_runtime_enable(&pdev->dev);

	error = snd_soc_register_component(&pdev->dev, &soc_codec_dev_cs40l26,
			cs40l26_dai, ARRAY_SIZE(cs40l26_dai));
	if (error < 0)
		dev_err(&pdev->dev, "Failed to register codec: %d\n", error);

	return error;
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
